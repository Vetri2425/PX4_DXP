#!/usr/bin/env python3
"""Spray actuator controller for PX4 AUX outputs via MAVROS CommandLong.

Production distance-aware mode consumes `/rpp/conditioned_path` plus identity,
owns timing/flow/safety compensation, applies debounce and safety gates, then
commands MAV_CMD_DO_SET_ACTUATOR. `/spray/active` is retained only as a legacy
fallback when distance-aware mode is disabled. The controller only drives an
already-configured PX4 actuator set output; QGC remains the source of truth for
AUX pin/function/PWM limits.

Manual override (/spray/manual, std_msgs/Bool) lets the server bench-test the
actuator: True holds spray ON for at most `manual_override_timeout_s`
(node-side hard expiry — never latches), False cancels immediately. The
override is subordinate to every fail-safe: disarm, mode loss, and node
shutdown all clear it. While the override is active the /spray/active
staleness watchdog only clears the *auto* desire (manual has its own timeout
and does not depend on the RPP stream). Actual override state is reported on
/spray/manual_state for the server.
"""

from __future__ import annotations

import math
import json
import signal
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger

from spray_config import SprayConfiguration, SprayMode, validate_spray_configuration
from spray_controller_modes import (
    ContinuousSprayRuntimeState,
    DwellState,
    auto_safety_status,
    build_path_model_for_config,
    continuous_distance_decision,
    point_mode_decision,
)
from spray_runtime_protocol import (
    RUNTIME_STATUS_TOPIC,
    deserialize_dwell_command,
    dwell_response_message,
    serialize_runtime_status,
)
from path_identity import (
    CONDITIONED_PATH_IDENTITY_TOPIC,
    parse_path_identity,
    path_geometry_fingerprint,
)


MAV_CMD_DO_SET_ACTUATOR = 187
MAV_CMD_DO_SET_SERVO = 183
_SERVO_PWM_MAX_US = 2200
_VALID_ACTUATOR_BACKENDS = {"mavlink_servo_pwm", "mavlink_actuator"}
from spray_path_model import (  # noqa: E402
    MARK_TO_TRANSIT,
    TRANSIT_TO_MARK,
    SprayBoundary,
    SprayDecision,
    SprayPathModel,
    SprayProjection,
    SprayProjectionState,
    build_path_model as _build_path_model,
    make_spray_decision as _make_spray_decision,
    next_boundary as _next_boundary,
    nozzle_position_ned as _nozzle_position_ned,
    pose_to_ned as _pose_to_ned,
    project_onto_path as _project_onto_path,
    yaw_ned_from_enu_quaternion as _yaw_ned_from_enu_quaternion,
)


@dataclass
class ActuatorState:
    desired_on: bool = False
    commanded_on: bool = False
    pending: bool = False
    pending_on: Optional[bool] = None
    pending_sequence: int = 0
    accepted_on: bool = False
    accepted_sequence: int = 0
    requested_on: bool = False
    current_on: bool = False
    debounce_state: str = "idle"
    last_on_timestamp_s: float = 0.0
    last_off_timestamp_s: float = 0.0
    command_duration_s: float = 0.0
    command_sequence: int = 0
    pending_pwm: float = 0.0
    pending_value: float = 0.0
    current_pwm: float = 0.0
    current_value: float = 0.0
    last_command_failure: str = ""
    off_confirmed: bool = False
    off_confirmation_source: str = "none"
    physical_confirmation_available: bool = False
    physical_on: Optional[bool] = None
    physical_confirmation_source: str = "none"
    physical_feedback_timestamp_monotonic_s: Optional[float] = None
    physical_feedback_age_s: Optional[float] = None
    physical_feedback_timeout_s: float = 1.0
    physical_feedback_stale: bool = True


def _best_effort_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


def _state_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


def _path_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class SprayControllerNode(Node):
    """Edge-triggered spray servo/solenoid controller."""

    def __init__(self) -> None:
        super().__init__("spray_controller")

        self.declare_parameter("actuator_set_index", 1)
        # Normalized actuator values for mavlink_actuator backend (cmd 187).
        # Mapping assumes PWM_AUX_MIN1=0, PWM_AUX_MAX1=2000 in QGC:
        #   on_value  1.0  → 3000 µs  (spray ON, full flow; requires PWM_AUX_MAX1=3000 in QGC)
        #   off_value -1.0 →    0 µs  (spray OFF, motor fully stopped)
        # Requires PWM_AUX_MIN1=0, PWM_AUX_DIS1=0, PWM_AUX_MAX1=3000 in QGC.
        self.declare_parameter("on_value", 1.0)
        self.declare_parameter("off_value", -1.0)
        self.declare_parameter("debounce_samples", 3)
        self.declare_parameter("reassert_hz", 2.0)
        self.declare_parameter("require_offboard", True)
        self.declare_parameter("active_timeout_s", 0.5)
        self.declare_parameter("manual_override_timeout_s", 10.0)
        self.declare_parameter("command_service", "/mavros/cmd/command")
        self.declare_parameter("use_distance_aware_spray", True)
        self.declare_parameter("nozzle_forward_offset_m", 0.0)
        self.declare_parameter("nozzle_lateral_offset_m", 0.0)
        self.declare_parameter("solenoid_open_delay_s", 0.10)
        self.declare_parameter("solenoid_close_delay_s", 0.05)
        # Legacy V2 name kept so old launch overrides do not fail. New code
        # uses explicit ON/OFF margins below to avoid shortening MARK tails.
        self.declare_parameter("anticipatory_margin_m", 0.02)
        self.declare_parameter("on_overspray_margin_m", 0.02)
        self.declare_parameter("off_overspray_margin_m", 0.0)
        self.declare_parameter("min_spray_speed_mps", 0.05)
        self.declare_parameter("max_spray_speed_mps", 1.0)
        self.declare_parameter("unsafe_speed_behavior", "BLOCK_SPRAY")
        self.declare_parameter("max_xtrack_error_m", 0.10)
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter("velocity_timeout_s", 0.5)
        self.declare_parameter("vehicle_state_timeout_s", 2.0)
        self.declare_parameter("allow_legacy_spray_active_fallback", False)
        self.declare_parameter("diagnostic_profile", False)
        self.declare_parameter("diagnostic_lease_active", False)
        self.declare_parameter("max_along_track_heading_error_deg", 30.0)
        self.declare_parameter("max_cross_track_speed_mps", 0.10)
        self.declare_parameter("max_reverse_speed_tolerance_mps", 0.03)
        self.declare_parameter("max_projection_jump_m", 0.50)
        self.declare_parameter("max_backward_projection_jump_m", 0.10)
        self.declare_parameter("projection_ambiguity_distance_m", 0.03)
        self.declare_parameter("max_lead_distance_m", 0.50)
        self.declare_parameter("min_on_distance_m", 0.05)
        self.declare_parameter("min_off_distance_m", 0.05)
        self.declare_parameter("flow_mode", "mapped")
        self.declare_parameter("min_target_flow", 0.0)
        self.declare_parameter("max_target_flow", 1.0)
        self.declare_parameter("max_pwm_change_per_s", 500.0)
        self.declare_parameter("low_speed_anti_puddle_behavior", "block")
        self.declare_parameter("high_speed_underflow_behavior", "block")
        # Backend selector: "mavlink_actuator" (cmd 187, normalized) or
        # "mavlink_servo_pwm" (cmd 183, absolute PWM µs).
        self.declare_parameter("actuator_backend", "mavlink_actuator")
        # servo_instance: MUST validate in QGC Actuator Outputs which instance
        # number maps to the physical AUX pin driving the spray driver.
        self.declare_parameter("servo_instance", 1)
        self.declare_parameter("off_pwm_us", 0)
        self.declare_parameter("on_pwm_us", 1800)
        # Master enable gate. When False the node will not command spray ON
        # from any source (manual override, mission auto-spray, reassert).
        # The server sets this via the /api/spray/enable and /api/spray/disable
        # endpoints. Fail closed after node restart; mission loading never
        # changes this operator-owned authorization state.
        self.declare_parameter("spray_enabled", False)
        self.declare_parameter("spray_mode", "continuous")
        self.declare_parameter("dash_on_distance_m", 0.30)
        self.declare_parameter("dash_off_distance_m", 0.30)
        self.declare_parameter("dash_phase_reset", "per_mark_region")
        self.declare_parameter("point_default_dwell_s", 2.0)
        self.declare_parameter("point_arrival_tolerance_m", 0.05)
        self.declare_parameter("point_settle_time_s", 0.10)
        self.declare_parameter("point_leg_timeout_s", 120.0)
        self.declare_parameter("point_settle_speed_mps", 0.05)
        self.declare_parameter("point_settle_yaw_rate_rad_s", 0.05)
        self.declare_parameter("point_max_dwell_s", 60.0)
        self.declare_parameter("point_leg_trajectory_mode", "two_point")
        self.declare_parameter("point_leg_spacing_m", 0.08)
        self.declare_parameter("point_hold_drift_tolerance_m", 0.08)
        self.declare_parameter("point_hold_drift_policy", "fail")
        # GPS-safety and obstacle gating fields are part of the mission spray
        # configuration contract (configuration_to_param_dict). They are declared
        # here so the runtime bulk-set is accepted; enforcement of this gating
        # inside the node is not yet implemented (params currently unused).
        self.declare_parameter("gps_required_fix_type", 6)
        self.declare_parameter("gps_global_position_max_age_ms", 500.0)
        self.declare_parameter("gps_local_pose_max_age_ms", 500.0)
        self.declare_parameter("gps_fix_max_age_ms", 500.0)
        self.declare_parameter("gps_max_pose_global_skew_ms", 100.0)
        self.declare_parameter("gps_runtime_policy", "pause")
        self.declare_parameter("gps_resume_policy", "manual")
        self.declare_parameter("gps_recovery_stable_s", 2.0)
        # F-01 (hybrid): the server feeds GPS_SURVEYED runtime safety to this
        # node over /spray/gps_gate. This is the maximum age of that feed before
        # the node treats it as stale and fail-closed blocks spray ON on its own
        # (independent of the server watchdog — see _evaluate_gps_surveyed_runtime_safety).
        self.declare_parameter("gps_runtime_gate_max_age_s", 3.0)
        self.declare_parameter("obstacle_integration_enabled", False)
        self.declare_parameter("obstacle_signal_max_age_s", 2.0)
        self.declare_parameter("configuration_revision", 0)
        self.declare_parameter("mission_config_mission_id", "")
        self.declare_parameter("mission_config_path_fingerprint", "")
        self.declare_parameter("calibration_profile_id", "factory_default")
        self.declare_parameter("calibration_profile_version", 1)
        self.declare_parameter("target_paint_density", 1.0)
        self.declare_parameter(
            "speed_pwm_table",
            "[{\"speed_mps\":0.05,\"pwm\":1200.0},{\"speed_mps\":0.35,\"pwm\":1800.0}]",
        )
        self.declare_parameter("actuator_min_pwm", 0.0)
        self.declare_parameter("actuator_max_pwm", 2200.0)
        self.declare_parameter("actuator_off_pwm", 0.0)
        self.declare_parameter("actuator_min_value", -1.0)
        self.declare_parameter("actuator_max_value", 1.0)
        self.declare_parameter("actuator_off_value", -1.0)
        self.declare_parameter("timing_only_compatibility", False)
        self.declare_parameter("pump_inertia_enabled", False)
        self.declare_parameter("pwm_ramp_prediction_enabled", False)
        self.declare_parameter("pressure_stabilization_enabled", False)
        self.declare_parameter("temperature_viscosity_compensation_enabled", False)
        # One JSON envelope is one atomic ROS parameter transaction. Trigger
        # validates its revision; cancellation invalidates prepared envelopes.
        self.declare_parameter("pending_dwell_command_json", "")
        self.declare_parameter("dwell_cancel_revision", 0)
        self._validate_actuator_backend()

        self._state_group = MutuallyExclusiveCallbackGroup()
        self._latency_group = self._state_group
        self._model_group = MutuallyExclusiveCallbackGroup()
        self._service_group = self._state_group
        self._config_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._config_ready = False
        self._config_error = ""
        self._active_config = self._configuration_from_node_parameters()
        self._config_ready = True
        self._model_revision = 0
        self._dwell_state: Optional[DwellState] = None
        self._last_dwell_revision = 0
        self._invalidated_dwell_revision = 0
        self._last_transition = "startup"
        self._desired_raw = False
        self._candidate: Optional[bool] = None
        self._candidate_count = 0
        self._desired_debounced = False
        self._commanded = False
        self._last_active_time = None
        self._legacy_active_raw = False
        self._manual_active = False
        self._manual_deadline_ns: Optional[int] = None
        self._armed = False
        self._mode = "UNKNOWN"
        self._state_recv_monotonic_s: float | None = None
        self._service_ready = False
        # Actuator state is UNKNOWN at startup — a previous instance may have
        # left the output ON. Start unconfirmed so the node drives a confirmed
        # OFF before trusting the believed state (see end of __init__).
        self._last_off_send_time_ns: Optional[int] = None
        self._last_safety_allows_on: bool | None = None
        # Monotonic command id. Each dispatched command carries the id current
        # at send time; _command_done ignores any result that is not the latest
        # so a late/out-of-order MAVROS reply cannot overwrite newer state.
        self._cmd_seq = 0
        self._actuator_state = ActuatorState()
        self._actuator_state.current_value = float(self.get_parameter("off_value").value)
        self._path_model: Optional[SprayPathModel] = None
        self._conditioned_path_identity: dict[str, object] = {}
        self._conditioned_path_source = "none"
        self._last_conditioned_identity_time = None
        self._pose_ned: Optional[tuple[float, float, float]] = None
        self._pose_recv_time = None
        self._vel_ned = (0.0, 0.0)
        self._vel_recv_time = None
        self._last_auto_source = ""
        self._last_distance_event = ""
        self._last_safety_block_reason = ""
        self._last_decision: Optional[SprayDecision] = None
        self._target_flow = 0.0
        self._current_pwm = 0.0
        self._current_value = float(self.get_parameter("off_value").value)
        self._pose_stale_logged = False
        self._velocity_stale_logged = False
        # F-01 (hybrid): GPS_SURVEYED runtime gate fed by the server watchdog.
        # active=False (default) means the gate is disengaged — LOCAL_NED and
        # non-surveyed missions are never RTK-gated here. _gps_gate_recv_time is
        # the monotonic receive time used for the fail-closed staleness fallback.
        self._gps_gate_active = False
        self._gps_gate_ok = True
        self._gps_gate_reason = ""
        self._gps_gate_seq = 0
        self._gps_gate_recv_time: Optional[float] = None
        self._projection_state = SprayProjectionState()
        self._continuous_runtime = ContinuousSprayRuntimeState(
            projection=self._projection_state
        )
        self._geometry_hash = ""
        self._runtime_spray_geometry_hash = ""
        self._waypoint_count = 0
        self._spray_flag_count = 0
        self._mark_waypoint_count = 0
        self._boundary_count = 0
        self._legacy_fallback_block_reason = ""

        command_service = str(self.get_parameter("command_service").value)
        self._command_cli = self.create_client(
            CommandLong,
            command_service,
            callback_group=self._service_group,
        )

        self._state_pub = self.create_publisher(Bool, "/spray/state", _best_effort_qos())
        self._desired_pub = self.create_publisher(
            Bool, "/spray/desired", _best_effort_qos()
        )
        self._commanded_pub = self.create_publisher(
            Bool, "/spray/commanded", _best_effort_qos()
        )
        self._debug_pub = self.create_publisher(
            Float32MultiArray, "/spray/debug", _best_effort_qos()
        )
        self._manual_state_pub = self.create_publisher(
            Bool, "/spray/manual_state", _best_effort_qos()
        )
        self._runtime_status_pub = self.create_publisher(
            String, RUNTIME_STATUS_TOPIC, _best_effort_qos()
        )
        self.create_subscription(
            Bool,
            "/spray/active",
            self._active_cb,
            _best_effort_qos(),
            callback_group=self._latency_group,
        )
        self.create_subscription(
            Path,
            "/rpp/conditioned_path",
            self._path_cb,
            _path_qos(),
            callback_group=self._model_group,
        )
        self.create_subscription(
            String,
            CONDITIONED_PATH_IDENTITY_TOPIC,
            self._conditioned_path_identity_cb,
            _path_qos(),
            callback_group=self._model_group,
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose_cb,
            _best_effort_qos(),
            callback_group=self._latency_group,
        )
        self.create_subscription(
            TwistStamped,
            "/mavros/local_position/velocity_local",
            self._vel_cb,
            _best_effort_qos(),
            callback_group=self._latency_group,
        )
        # Reliable VOLATILE (depth 1): a manual command must arrive, but a
        # stale override must never be re-delivered to a restarted node.
        self.create_subscription(
            Bool,
            "/spray/manual",
            self._manual_cb,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            ),
            callback_group=self._service_group,
        )
        self.create_subscription(
            State,
            "/mavros/state",
            self._state_cb,
            _state_qos(),
            callback_group=self._latency_group,
        )
        # F-01 (hybrid): GPS_SURVEYED runtime safety feed from the server.
        self.create_subscription(
            String,
            "/spray/gps_gate",
            self._gps_gate_cb,
            _best_effort_qos(),
            callback_group=self._latency_group,
        )

        self.create_service(
            Trigger,
            "/spray/apply_mission_config",
            self._apply_mission_config_srv,
            callback_group=self._service_group,
        )
        self.create_service(
            Trigger,
            "/spray/start_dwell",
            self._start_dwell_srv,
            callback_group=self._service_group,
        )
        self.create_service(
            Trigger,
            "/spray/cancel_dwell",
            self._cancel_dwell_srv,
            callback_group=self._service_group,
        )

        self._watchdog_timer = self.create_timer(
            0.02, self._watchdog_tick, callback_group=self._latency_group
        )
        self._runtime_status_timer = self.create_timer(
            0.1, self._publish_runtime_status, callback_group=self._latency_group
        )
        reassert_hz = max(0.0, float(self.get_parameter("reassert_hz").value))
        self._reassert_timer = None
        if reassert_hz > 0.0:
            self._reassert_timer = self.create_timer(
                1.0 / reassert_hz,
                self._reassert_tick,
                callback_group=self._latency_group,
            )

        if self._command_cli.wait_for_service(timeout_sec=2.0):
            self._service_ready = True
        else:
            self.get_logger().warn(
                f"{command_service} not ready; spray commands idle until service appears"
            )
            self.create_timer(1.0, self._service_probe_tick)

        backend = str(self.get_parameter("actuator_backend").value)
        if backend == "mavlink_servo_pwm":
            self.get_logger().warn(
                f"Spray backend=mavlink_servo_pwm "
                f"servo_instance={self.get_parameter('servo_instance').value} "
                f"off_pwm_us={self.get_parameter('off_pwm_us').value} "
                f"on_pwm_us={self.get_parameter('on_pwm_us').value}"
            )
        else:
            self.get_logger().info("Spray backend=mavlink_actuator (normalized -1/+1)")

        self._publish_state(False)
        self.get_logger().info("spray_controller started")
        # Proactively drive the actuator OFF on startup. If the service is not
        # yet ready, _send_command leaves _off_confirmed False and the watchdog
        # / service-probe retry path issues the OFF as soon as it appears.
        self._send_command(False, reason="startup")

    @property
    def _off_confirmed(self) -> bool:
        return self._actuator_state.off_confirmed

    @_off_confirmed.setter
    def _off_confirmed(self, value: bool) -> None:
        self._actuator_state.off_confirmed = bool(value)

    def _physical_feedback_runtime_fields(
        self, actuator: ActuatorState, now_monotonic_s: float
    ) -> dict[str, Any]:
        if not actuator.physical_confirmation_available:
            return {
                "physical_feedback_timestamp_monotonic_s": None,
                "physical_feedback_age_s": None,
                "physical_feedback_timeout_s": actuator.physical_feedback_timeout_s,
                "physical_feedback_stale": True,
            }
        timestamp_s = actuator.physical_feedback_timestamp_monotonic_s
        if timestamp_s is None:
            age_s = None
            stale = True
        else:
            age_s = now_monotonic_s - timestamp_s
            stale = age_s > actuator.physical_feedback_timeout_s
        if actuator.physical_feedback_stale:
            stale = True
        return {
            "physical_feedback_timestamp_monotonic_s": timestamp_s,
            "physical_feedback_age_s": age_s,
            "physical_feedback_timeout_s": actuator.physical_feedback_timeout_s,
            "physical_feedback_stale": stale,
        }

    def _configuration_from_node_parameters(self) -> SprayConfiguration:
        speed_pwm_raw = self.get_parameter("speed_pwm_table").value
        if isinstance(speed_pwm_raw, str):
            try:
                speed_pwm_raw = json.loads(speed_pwm_raw)
            except ValueError:
                pass
        raw = {
            "spray_mode": str(self.get_parameter("spray_mode").value),
            "solenoid_open_delay_s": float(self.get_parameter("solenoid_open_delay_s").value),
            "solenoid_close_delay_s": float(self.get_parameter("solenoid_close_delay_s").value),
            "on_overspray_margin_m": float(self.get_parameter("on_overspray_margin_m").value),
            "off_overspray_margin_m": float(self.get_parameter("off_overspray_margin_m").value),
            "min_spray_speed_mps": float(self.get_parameter("min_spray_speed_mps").value),
            "max_spray_speed_mps": float(self.get_parameter("max_spray_speed_mps").value),
            "unsafe_speed_behavior": str(self.get_parameter("unsafe_speed_behavior").value),
            "max_xtrack_error_m": float(self.get_parameter("max_xtrack_error_m").value),
            "nozzle_forward_offset_m": float(self.get_parameter("nozzle_forward_offset_m").value),
            "nozzle_lateral_offset_m": float(self.get_parameter("nozzle_lateral_offset_m").value),
            "dash_on_distance_m": float(self.get_parameter("dash_on_distance_m").value),
            "dash_off_distance_m": float(self.get_parameter("dash_off_distance_m").value),
            "dash_phase_reset": str(self.get_parameter("dash_phase_reset").value),
            "point_default_dwell_s": float(self.get_parameter("point_default_dwell_s").value),
            "point_arrival_tolerance_m": float(
                self.get_parameter("point_arrival_tolerance_m").value
            ),
            "point_settle_time_s": float(self.get_parameter("point_settle_time_s").value),
            "point_leg_timeout_s": float(self.get_parameter("point_leg_timeout_s").value),
            "point_settle_speed_mps": float(self.get_parameter("point_settle_speed_mps").value),
            "point_settle_yaw_rate_rad_s": float(
                self.get_parameter("point_settle_yaw_rate_rad_s").value
            ),
            "require_offboard": bool(self.get_parameter("require_offboard").value),
            "debounce_samples": int(self.get_parameter("debounce_samples").value),
            "pose_timeout_s": float(self.get_parameter("pose_timeout_s").value),
            "velocity_timeout_s": float(self.get_parameter("velocity_timeout_s").value),
            "configuration_revision": int(self.get_parameter("configuration_revision").value),
            "mission_id": str(self.get_parameter("mission_config_mission_id").value),
            "path_fingerprint": str(self.get_parameter("mission_config_path_fingerprint").value),
            "calibration_profile_id": str(self.get_parameter("calibration_profile_id").value),
            "calibration_profile_version": int(self.get_parameter("calibration_profile_version").value),
            "target_paint_density": float(self.get_parameter("target_paint_density").value),
            "speed_pwm_table": speed_pwm_raw,
            "actuator_min_pwm": float(self.get_parameter("actuator_min_pwm").value),
            "actuator_max_pwm": float(self.get_parameter("actuator_max_pwm").value),
            "actuator_off_pwm": float(self.get_parameter("actuator_off_pwm").value),
            "actuator_min_value": float(self.get_parameter("actuator_min_value").value),
            "actuator_max_value": float(self.get_parameter("actuator_max_value").value),
            "actuator_off_value": float(self.get_parameter("actuator_off_value").value),
            "timing_only_compatibility": bool(self.get_parameter("timing_only_compatibility").value),
            "pump_inertia_enabled": bool(self.get_parameter("pump_inertia_enabled").value),
            "pwm_ramp_prediction_enabled": bool(self.get_parameter("pwm_ramp_prediction_enabled").value),
            "pressure_stabilization_enabled": bool(self.get_parameter("pressure_stabilization_enabled").value),
            "temperature_viscosity_compensation_enabled": bool(
                self.get_parameter("temperature_viscosity_compensation_enabled").value
            ),
            "max_along_track_heading_error_deg": float(
                self.get_parameter("max_along_track_heading_error_deg").value
            ),
            "max_cross_track_speed_mps": float(
                self.get_parameter("max_cross_track_speed_mps").value
            ),
            "max_reverse_speed_tolerance_mps": float(
                self.get_parameter("max_reverse_speed_tolerance_mps").value
            ),
            "max_projection_jump_m": float(self.get_parameter("max_projection_jump_m").value),
            "max_backward_projection_jump_m": float(
                self.get_parameter("max_backward_projection_jump_m").value
            ),
            "projection_ambiguity_distance_m": float(
                self.get_parameter("projection_ambiguity_distance_m").value
            ),
            "max_lead_distance_m": float(self.get_parameter("max_lead_distance_m").value),
            "min_on_distance_m": float(self.get_parameter("min_on_distance_m").value),
            "min_off_distance_m": float(self.get_parameter("min_off_distance_m").value),
            "flow_mode": str(self.get_parameter("flow_mode").value),
            "min_target_flow": float(self.get_parameter("min_target_flow").value),
            "max_target_flow": float(self.get_parameter("max_target_flow").value),
            "max_pwm_change_per_s": float(self.get_parameter("max_pwm_change_per_s").value),
            "low_speed_anti_puddle_behavior": str(
                self.get_parameter("low_speed_anti_puddle_behavior").value
            ),
            "high_speed_underflow_behavior": str(
                self.get_parameter("high_speed_underflow_behavior").value
            ),
            "allow_legacy_spray_active_fallback": bool(
                self.get_parameter("allow_legacy_spray_active_fallback").value
            ),
        }
        return validate_spray_configuration(raw)

    def _get_config_snapshot(self) -> SprayConfiguration:
        with self._config_lock:
            return self._active_config

    def _set_config_snapshot(self, config: SprayConfiguration, *, ready: bool, error: str = "") -> None:
        with self._config_lock:
            self._active_config = config
            self._config_ready = ready
            self._config_error = error

    def _reset_projection_runtime_state(self) -> None:
        self._projection_state.reset()
        self._continuous_runtime.last_spray_transition_s = None
        self._continuous_runtime.last_geometry_state = False
        self._continuous_runtime.previous_pwm = float(
            self.get_parameter("actuator_off_pwm").value
        )
        self._continuous_runtime.last_tick_mono_s = None

    def _reset_decision_state(self, reason: str) -> None:
        self._candidate = None
        self._candidate_count = 0
        self._desired_raw = False
        self._desired_debounced = False
        self._last_distance_event = ""
        self._last_safety_block_reason = ""
        self._last_transition = reason
        self._reset_projection_runtime_state()

    def _invalidate_dwell(self, reason: str) -> None:
        with self._state_lock:
            self._invalidated_dwell_revision = max(
                self._invalidated_dwell_revision, self._last_dwell_revision
            )
            if self._dwell_state is not None:
                self._dwell_state = DwellState(
                    command_id=self._dwell_state.command_id,
                    mission_id=self._dwell_state.mission_id,
                    point_index=self._dwell_state.point_index,
                    start_mono_ns=self._dwell_state.start_mono_ns,
                    expiry_mono_ns=self._dwell_state.expiry_mono_ns,
                    cancelled=True,
                )
            self._last_transition = reason

    def _apply_mission_config_from_parameters(self) -> tuple[bool, str]:
        self._force_off("mission configuration apply", force=True)
        self._invalidate_dwell("mission configuration apply")
        try:
            config = self._configuration_from_node_parameters()
        except ValueError as exc:
            self._set_config_snapshot(self._get_config_snapshot(), ready=False, error=str(exc))
            return False, str(exc)
        self._set_config_snapshot(config, ready=True, error="")
        with self._state_lock:
            self._path_model = None
            self._conditioned_path_identity = {}
            self._conditioned_path_source = "none"
            self._geometry_hash = ""
            self._runtime_spray_geometry_hash = ""
            self._waypoint_count = 0
            self._spray_flag_count = 0
            self._mark_waypoint_count = 0
            self._boundary_count = 0
            self._model_revision += 1
            self._reset_decision_state("mission_config_applied")
        self.get_logger().info(
            f"spray mission config applied: mode={config.mode.value} "
            f"revision={config.revision} mission_id={config.mission_id!r}"
        )
        self._publish_runtime_status()
        return True, "configuration applied"

    def _apply_mission_config_srv(self, _request, response):
        ok, message = self._apply_mission_config_from_parameters()
        response.success = ok
        response.message = message
        return response

    def _start_dwell_srv(self, _request, response):
        config = self._get_config_snapshot()
        if config.mode != SprayMode.POINT:
            response.success = False
            response.message = "dwell rejected: spray_mode is not point"
            return response
        if not self._config_ready:
            response.success = False
            response.message = f"dwell rejected: spray config not ready ({self._config_error})"
            return response
        try:
            command = deserialize_dwell_command(
                str(self.get_parameter("pending_dwell_command_json").value)
            )
        except (KeyError, TypeError, ValueError) as exc:
            response.success = False
            response.message = f"dwell rejected: invalid command envelope ({exc})"
            return response
        command_id = command["command_id"]
        mission_id = command["mission_id"]
        point_index = command["point_index"]
        duration_s = command["duration_s"]
        revision = command["revision"]
        if command_id <= 0:
            response.success = False
            response.message = "dwell rejected: invalid command_id"
            return response
        if point_index < 0:
            response.success = False
            response.message = "dwell rejected: invalid point_index"
            return response
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            response.success = False
            response.message = "dwell rejected: duration_s must be > 0"
            return response
        if config.mission_id and mission_id and mission_id != config.mission_id:
            response.success = False
            response.message = "dwell rejected: mission_id mismatch"
            return response
        if command["configuration_revision"] != config.revision:
            response.success = False
            response.message = "dwell rejected: configuration revision mismatch"
            return response
        if not self._safety_allows_on():
            response.success = False
            response.message = "dwell rejected: safety gate blocks spray ON"
            return response
        with self._state_lock:
            now_ns = time.monotonic_ns()
            if revision <= self._invalidated_dwell_revision:
                response.success = False
                response.message = "dwell rejected: command revision was cancelled"
                return response
            if revision <= self._last_dwell_revision:
                response.success = False
                response.message = "dwell rejected: stale or duplicate command revision"
                return response
            if (
                self._dwell_state is not None
                and self._dwell_state.active
                and now_ns < self._dwell_state.expiry_mono_ns
            ):
                response.success = False
                response.message = "dwell rejected: another dwell is active"
                return response
            expiry_ns = now_ns + int(duration_s * 1e9)
            self._dwell_state = DwellState(
                command_id=command_id,
                mission_id=mission_id,
                point_index=point_index,
                start_mono_ns=now_ns,
                expiry_mono_ns=expiry_ns,
            )
            self._last_dwell_revision = revision
            self._last_transition = f"dwell_started:{point_index}"
        response.success = True
        response.message = dwell_response_message(command_id, expiry_ns * 1e-9)
        self._publish_runtime_status()
        return response

    def _cancel_dwell_srv(self, _request, response):
        with self._state_lock:
            self._invalidated_dwell_revision = max(
                self._invalidated_dwell_revision,
                int(self.get_parameter("dwell_cancel_revision").value),
            )
        try:
            pending = deserialize_dwell_command(
                str(self.get_parameter("pending_dwell_command_json").value)
            )
            with self._state_lock:
                self._invalidated_dwell_revision = max(
                    self._invalidated_dwell_revision, pending["revision"]
                )
        except (KeyError, TypeError, ValueError):
            pass
        self._force_off("dwell_cancelled", force=True)
        response.success = True
        response.message = "dwell cancelled"
        self._publish_runtime_status()
        return response

    def get_runtime_status(self) -> dict[str, Any]:
        config = self._get_config_snapshot()
        now_ns = time.monotonic_ns()
        with self._state_lock:
            dwell = self._dwell_state
            active_dwell = bool(
                dwell is not None and dwell.active and now_ns < dwell.expiry_mono_ns
            )
            dwell_remaining_s = (
                max(0.0, (dwell.expiry_mono_ns - now_ns) * 1e-9)
                if active_dwell and dwell is not None
                else 0.0
            )
            gps_gate_active = self._gps_gate_active
            gps_gate_seq = self._gps_gate_seq
            snapshot = {
                "model_revision": self._model_revision,
                "commanded_on": self._commanded,
                "off_acknowledged": self._actuator_state.off_confirmed,
                "confirmed_off": (
                    self._actuator_state.off_confirmed and not self._commanded
                ),
                "desired_on": self._effective_desired(),
                "last_transition": self._last_transition,
                "conditioned_path_source": self._conditioned_path_source,
                "path_identity": dict(self._conditioned_path_identity),
                "last_decision": self._last_decision,
                "target_flow": self._target_flow,
                "current_pwm": self._current_pwm,
                "current_value": self._current_value,
                "actuator": self._actuator_state,
            }
        gps_eval_ok, gps_eval_reason = self._evaluate_gps_surveyed_runtime_safety()
        physical_feedback = self._physical_feedback_runtime_fields(
            snapshot["actuator"], now_ns * 1e-9
        )
        state_fresh, state_age_s, state_block_reason = self._vehicle_state_freshness()
        geometry_spray_request = (
            bool(snapshot["last_decision"].geometry_desired)
            if snapshot["last_decision"] is not None
            else False
        )
        dry_run_active = config.continuous.flow_mode == "disabled"
        return {
            "timestamp_monotonic_s": now_ns * 1e-9,
            "spray_mode": config.mode.value,
            "active_mode": config.mode.value,
            "configuration_revision": config.revision,
            "model_revision": snapshot["model_revision"],
            "ready": self._config_ready,
            "operator_enabled": bool(self.get_parameter("spray_enabled").value),
            "desired_on": snapshot["desired_on"],
            "commanded_on": snapshot["commanded_on"],
            "confirmed_off": snapshot["confirmed_off"],
            "off_acknowledged": snapshot["off_acknowledged"],
            "off_confirmation_source": snapshot["actuator"].off_confirmation_source,
            "active_dwell": active_dwell,
            "dwell_command_id": dwell.command_id if dwell is not None else None,
            "dwell_mission_id": dwell.mission_id if dwell is not None else None,
            "dwell_point_index": dwell.point_index if dwell is not None else None,
            "dwell_remaining_s": dwell_remaining_s,
            "last_transition": snapshot["last_transition"],
            "last_error": self._config_error,
            "actual_speed_mps": math.hypot(self._vel_ned[0], self._vel_ned[1]),
            "target_speed_mps": None,
            "target_speed_source": "unavailable",
            "vehicle_state_age_s": (
                state_age_s if self._state_recv_monotonic_s is not None else None
            ),
            "vehicle_state_stale": not state_fresh,
            "vehicle_state_block_reason": state_block_reason if not state_fresh else "",
            "dry_run_active": dry_run_active,
            "geometry_spray_request": geometry_spray_request,
            "target_flow": snapshot["target_flow"],
            "current_pwm": snapshot["current_pwm"],
            "current_value": snapshot["current_value"],
            "spray_state": self._spray_state_label(snapshot["actuator"]),
            "commanded_spray_state": "ON" if snapshot["commanded_on"] else "OFF",
            "pending_command": snapshot["actuator"].pending,
            "pending_command_on": snapshot["actuator"].pending_on,
            "accepted_command_on": snapshot["actuator"].accepted_on,
            "accepted_command_sequence": snapshot["actuator"].accepted_sequence,
            "software_actuator_state": "ON" if snapshot["actuator"].current_on else "OFF",
            "physical_actuator_state": self._physical_state_label(snapshot["actuator"]),
            "physical_confirmation_available": (
                snapshot["actuator"].physical_confirmation_available
            ),
            "physical_confirmed_off": (
                snapshot["actuator"].physical_confirmation_available
                and snapshot["actuator"].physical_on is False
            ),
            "physical_confirmation_source": snapshot["actuator"].physical_confirmation_source,
            **physical_feedback,
            "distance_to_next_boundary_m": (
                snapshot["last_decision"].distance_to_boundary_m
                if snapshot["last_decision"] is not None else None
            ),
            "current_segment_index": (
                snapshot["last_decision"].projection.segment_index
                if snapshot["last_decision"] is not None
                and snapshot["last_decision"].projection is not None else None
            ),
            "current_segment_type": (
                "MARK"
                if snapshot["last_decision"] is not None
                and snapshot["last_decision"].projection is not None
                and snapshot["last_decision"].projection.current_flag
                else "TRANSIT"
            ),
            "current_decision_event": (
                snapshot["last_decision"].event
                if snapshot["last_decision"] is not None else ""
            ),
            "safety_reason": (
                snapshot["last_decision"].safety_reason
                if snapshot["last_decision"] is not None else self._last_safety_block_reason
            ),
            "conditioned_path_source": snapshot["conditioned_path_source"],
            "path_identity": snapshot["path_identity"],
            "mission_id": config.mission_id,
            "path_fingerprint": config.path_fingerprint,
            "geometry_hash": self._geometry_hash,
            "runtime_spray_geometry_hash": self._runtime_spray_geometry_hash,
            "waypoint_count": self._waypoint_count,
            "spray_flag_count": self._spray_flag_count,
            "mark_waypoint_count": self._mark_waypoint_count,
            "boundary_count": self._boundary_count,
            "distance_aware_spray_enabled": bool(
                self.get_parameter("use_distance_aware_spray").value
            ),
            "legacy_fallback_configured": bool(
                self.get_parameter("allow_legacy_spray_active_fallback").value
            ),
            "legacy_fallback_active": self._legacy_fallback_allowed(),
            "legacy_fallback_block_reason": self._legacy_fallback_block_reason,
            "projected_arc_length_m": (
                snapshot["last_decision"].projection.s
                if snapshot["last_decision"] is not None
                and snapshot["last_decision"].projection is not None
                else None
            ),
            "projection_segment_index": (
                snapshot["last_decision"].projection.segment_index
                if snapshot["last_decision"] is not None
                and snapshot["last_decision"].projection is not None
                else None
            ),
            "projection_xtrack_error_m": (
                snapshot["last_decision"].projection.xtrack_error_m
                if snapshot["last_decision"] is not None
                and snapshot["last_decision"].projection is not None
                else None
            ),
            "projection_jump_m": (
                snapshot["last_decision"].projection.projection_jump_m
                if snapshot["last_decision"] is not None
                and snapshot["last_decision"].projection is not None
                else None
            ),
            "projection_backward_jump_m": (
                snapshot["last_decision"].projection.backward_jump_m
                if snapshot["last_decision"] is not None
                and snapshot["last_decision"].projection is not None
                else None
            ),
            "ambiguity_clearance_confidence": (
                snapshot["last_decision"].projection.ambiguity_clearance_confidence
                if snapshot["last_decision"] is not None
                and snapshot["last_decision"].projection is not None
                else None
            ),
            "projection_confidence": (
                snapshot["last_decision"].projection.ambiguity_clearance_confidence
                if snapshot["last_decision"] is not None
                and snapshot["last_decision"].projection is not None
                else None
            ),
            "projection_ambiguous": (
                snapshot["last_decision"].projection.ambiguous
                if snapshot["last_decision"] is not None
                and snapshot["last_decision"].projection is not None
                else None
            ),
            "projection_ambiguity_reason": (
                snapshot["last_decision"].projection.ambiguity_reason
                if snapshot["last_decision"] is not None
                and snapshot["last_decision"].projection is not None
                else None
            ),
            "along_track_speed_mps": (
                snapshot["last_decision"].along_track_speed_mps
                if snapshot["last_decision"] is not None else None
            ),
            "cross_track_speed_mps": (
                snapshot["last_decision"].cross_track_speed_mps
                if snapshot["last_decision"] is not None else None
            ),
            "velocity_heading_error_deg": (
                snapshot["last_decision"].velocity_heading_error_deg
                if snapshot["last_decision"] is not None else None
            ),
            "raw_on_lead_m": (
                snapshot["last_decision"].raw_on_lead_m
                if snapshot["last_decision"] is not None else None
            ),
            "bounded_on_lead_m": (
                snapshot["last_decision"].bounded_on_lead_m
                if snapshot["last_decision"] is not None else None
            ),
            "raw_off_lead_m": (
                snapshot["last_decision"].raw_off_lead_m
                if snapshot["last_decision"] is not None else None
            ),
            "bounded_off_lead_m": (
                snapshot["last_decision"].bounded_off_lead_m
                if snapshot["last_decision"] is not None else None
            ),
            "lead_clamped": (
                snapshot["last_decision"].lead_clamped
                if snapshot["last_decision"] is not None else None
            ),
            "lead_block_reason": (
                snapshot["last_decision"].lead_block_reason
                if snapshot["last_decision"] is not None else None
            ),
            "current_run_remaining_m": (
                snapshot["last_decision"].current_run_remaining_m
                if snapshot["last_decision"] is not None else None
            ),
            "current_run_length_m": (
                snapshot["last_decision"].current_run_length_m
                if snapshot["last_decision"] is not None else None
            ),
            "next_run_length_m": (
                snapshot["last_decision"].next_run_length_m
                if snapshot["last_decision"] is not None else None
            ),
            "next_boundary_kind": (
                snapshot["last_decision"].next_boundary.kind
                if snapshot["last_decision"] is not None
                and snapshot["last_decision"].next_boundary is not None
                else None
            ),
            "distance_to_boundary_m": (
                snapshot["last_decision"].distance_to_boundary_m
                if snapshot["last_decision"] is not None else None
            ),
            "flow_mode": (
                snapshot["last_decision"].flow_mode
                if snapshot["last_decision"] is not None else config.continuous.flow_mode
            ),
            "raw_target_flow": (
                snapshot["last_decision"].raw_target_flow
                if snapshot["last_decision"] is not None else None
            ),
            "target_paint_density": (
                snapshot["last_decision"].target_paint_density
                if snapshot["last_decision"] is not None
                else config.calibration.target_paint_density
            ),
            "flow_clamp_reason": (
                snapshot["last_decision"].flow_clamp_reason
                if snapshot["last_decision"] is not None else None
            ),
            "flow_under_capacity": (
                snapshot["last_decision"].flow_under_capacity
                if snapshot["last_decision"] is not None else None
            ),
            "target_pwm": (
                snapshot["last_decision"].target_pwm
                if snapshot["last_decision"] is not None else None
            ),
            "command_pwm": (
                snapshot["last_decision"].command_pwm
                if snapshot["last_decision"] is not None else None
            ),
            "pwm_ramp_limited": (
                snapshot["last_decision"].pwm_ramp_limited
                if snapshot["last_decision"] is not None else None
            ),
            # F-01/F-05: surveyed runtime-safety gate visibility
            "gps_surveyed_active": gps_gate_active,
            "gps_safety_ok": gps_eval_ok,
            "gps_safety_reason": gps_eval_reason,
            "gps_gate_seq": gps_gate_seq,
            "manual_resume_required": bool(gps_gate_active and not gps_eval_ok),
            "actuator_failure_state": snapshot["actuator"].last_command_failure,
            "actuator": {
                "desired_on": snapshot["actuator"].desired_on,
                "commanded_on": snapshot["actuator"].commanded_on,
                "requested_on": snapshot["actuator"].requested_on,
                "pending": snapshot["actuator"].pending,
                "pending_on": snapshot["actuator"].pending_on,
                "pending_sequence": snapshot["actuator"].pending_sequence,
                "accepted_on": snapshot["actuator"].accepted_on,
                "accepted_sequence": snapshot["actuator"].accepted_sequence,
                "current_on": snapshot["actuator"].current_on,
                "debounce_state": snapshot["actuator"].debounce_state,
                "last_on_timestamp_s": snapshot["actuator"].last_on_timestamp_s,
                "last_off_timestamp_s": snapshot["actuator"].last_off_timestamp_s,
                "command_duration_s": snapshot["actuator"].command_duration_s,
                "command_sequence": snapshot["actuator"].command_sequence,
                "pending_pwm": snapshot["actuator"].pending_pwm,
                "pending_value": snapshot["actuator"].pending_value,
                "current_pwm": snapshot["actuator"].current_pwm,
                "current_value": snapshot["actuator"].current_value,
                "last_command_failure": snapshot["actuator"].last_command_failure,
                "off_confirmed": snapshot["actuator"].off_confirmed,
                "off_confirmation_source": snapshot["actuator"].off_confirmation_source,
                "physical_confirmation_available": (
                    snapshot["actuator"].physical_confirmation_available
                ),
                "physical_on": snapshot["actuator"].physical_on,
                "physical_confirmation_source": snapshot["actuator"].physical_confirmation_source,
                "physical_feedback_timestamp_monotonic_s": physical_feedback[
                    "physical_feedback_timestamp_monotonic_s"
                ],
                "physical_feedback_age_s": physical_feedback["physical_feedback_age_s"],
                "physical_feedback_timeout_s": physical_feedback[
                    "physical_feedback_timeout_s"
                ],
                "physical_feedback_stale": physical_feedback["physical_feedback_stale"],
            },
        }

    def _spray_state_label(self, actuator: ActuatorState) -> str:
        if actuator.pending:
            return "PENDING_ON" if actuator.pending_on else "PENDING_OFF"
        return "ACCEPTED_ON" if actuator.accepted_on else "ACCEPTED_OFF"

    def _physical_state_label(self, actuator: ActuatorState) -> str:
        if not actuator.physical_confirmation_available:
            return "UNAVAILABLE"
        if actuator.physical_on is None:
            return "UNKNOWN"
        return "ON" if actuator.physical_on else "OFF"

    def _publish_runtime_status(self) -> None:
        msg = String()
        msg.data = serialize_runtime_status(self.get_runtime_status())
        self._runtime_status_pub.publish(msg)

    def _service_probe_tick(self) -> None:
        if self._service_ready:
            return
        if self._command_cli.service_is_ready():
            self._service_ready = True
            self.get_logger().info("spray command service is ready")
            if not self._off_confirmed:
                self._maybe_retry_off("service ready startup OFF", force=True)

    def _state_cb(self, msg: State) -> None:
        prev_safe = self._safety_allows_on()
        self._state_recv_monotonic_s = time.monotonic()
        self._armed = bool(msg.armed)
        self._mode = str(msg.mode)
        now_safe = self._safety_allows_on()
        if prev_safe and not now_safe:
            self._invalidate_dwell("safety loss")
            self._force_off("FCU left armed/OFFBOARD safe state", force=True)
        elif not prev_safe and now_safe and self._desired_debounced:
            self._commit_desired_state()

    def _legacy_fallback_allowed(self) -> bool:
        return (
            bool(self.get_parameter("diagnostic_profile").value)
            and bool(self.get_parameter("diagnostic_lease_active").value)
            and bool(self.get_parameter("allow_legacy_spray_active_fallback").value)
        )

    def _active_cb(self, msg: Bool) -> None:
        self._last_active_time = self.get_clock().now()
        self._legacy_active_raw = bool(msg.data)
        if not bool(self.get_parameter("use_distance_aware_spray").value):
            if self._legacy_fallback_allowed():
                self._set_auto_desired(self._legacy_active_raw, source="legacy")
            else:
                self._legacy_fallback_block_reason = (
                    "legacy spray active fallback disabled in production"
                )
                self._set_auto_desired(False, source="legacy_blocked")

    def _conditioned_path_identity_cb(self, msg: String) -> None:
        with self._state_lock:
            self._conditioned_path_identity = parse_path_identity(msg.data)
            self._conditioned_path_source = str(
                self._conditioned_path_identity.get("source", "rpp_conditioned_path")
                or "rpp_conditioned_path"
            )
            self._last_conditioned_identity_time = self.get_clock().now()
            if self._conditioned_path_source == "clear":
                self._path_model = None
                self._model_revision += 1
                self._last_safety_block_reason = "conditioned path cleared"
                self._set_auto_desired(False, source="distance")
                self._force_off("conditioned path cleared", force=True)
                return
            matches, why = self._conditioned_identity_matches_config(
                self._get_config_snapshot()
            )
            if self._path_model is not None and not matches:
                self._path_model = None
                self._model_revision += 1
                self._last_safety_block_reason = why
                self._set_auto_desired(False, source="distance")
                self._force_off(why, force=True)

    def _conditioned_identity_matches_config(self, config: SprayConfiguration) -> tuple[bool, str]:
        identity = dict(self._conditioned_path_identity or {})
        expected_fp = config.path_fingerprint
        actual_fp = str(identity.get("path_fingerprint", "") or "")
        expected_mission = config.mission_id
        actual_mission = str(identity.get("mission_id", "") or "")
        expected_revision = int(config.revision or 0)
        actual_revision = int(identity.get("configuration_revision", 0) or 0)
        strict = bool(expected_mission and not config.calibration.timing_only_compatibility)
        if strict and not identity:
            return False, "missing conditioned path identity"
        if expected_fp:
            if not actual_fp:
                return False, "missing conditioned path identity"
            if actual_fp != expected_fp:
                return False, "path identity mismatch"
        elif strict:
            return False, "mission path fingerprint missing"
        if expected_mission:
            if strict and not actual_mission:
                return False, "missing mission identity"
            if actual_mission and actual_mission != expected_mission:
                return False, "mission identity mismatch"
        if strict:
            if expected_revision <= 0:
                return False, "mission configuration revision missing"
            if actual_revision <= 0:
                return False, "conditioned path configuration revision missing"
            if actual_revision != expected_revision:
                return False, "conditioned path configuration revision mismatch"
        return True, ""

    def _reject_conditioned_path(self, reason: str) -> None:
        with self._state_lock:
            self._path_model = None
            self._model_revision += 1
            self._last_safety_block_reason = reason
        self._set_auto_desired(False, source="distance")
        self._force_off(reason, force=True)

    def _path_cb(self, msg: Path) -> None:
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        flags = [p.pose.position.z > 0.5 for p in msg.poses]
        config = self._get_config_snapshot()
        matches, why = self._conditioned_identity_matches_config(config)
        if not matches:
            self._reject_conditioned_path(why)
            self.get_logger().warn(f"spray conditioned path rejected: {why}")
            return
        if not points:
            self._reject_conditioned_path("conditioned path cleared")
            self.get_logger().warn("spray path cleared: received empty /rpp/conditioned_path")
            return
        try:
            base_model = _build_path_model(points, flags)
            model = build_path_model_for_config(base_model, config)
            base_geometry_hash = path_geometry_fingerprint(
                base_model.points,
                base_model.flags,
            )
            runtime_spray_geometry_hash = path_geometry_fingerprint(
                model.points,
                model.flags,
            )
        except ValueError as exc:
            self._reject_conditioned_path(f"invalid conditioned path: {exc}")
            self.get_logger().warn(f"spray path rejected: {exc}")
            return
        expected_fp = config.path_fingerprint
        strict = bool(
            config.mission_id and not config.calibration.timing_only_compatibility
        )
        if strict and expected_fp and base_geometry_hash != expected_fp:
            self._reject_conditioned_path("conditioned geometry hash mismatch")
            self.get_logger().warn(
                f"spray path rejected: conditioned geometry hash "
                f"{base_geometry_hash!r} != configured {expected_fp!r}"
            )
            return
        current = self._get_config_snapshot()
        if current.revision != config.revision or current.mode != config.mode:
            self.get_logger().warn("discarded path model built for replaced spray configuration")
            return
        with self._state_lock:
            self._path_model = model
            self._geometry_hash = base_geometry_hash
            self._runtime_spray_geometry_hash = runtime_spray_geometry_hash
            self._waypoint_count = len(model.points)
            self._spray_flag_count = len(model.flags)
            self._mark_waypoint_count = sum(1 for flag in model.flags if flag)
            self._boundary_count = len(model.boundaries)
            self._model_revision += 1
            self._reset_decision_state("path_model_updated")
        self.get_logger().info(
            f"spray path loaded: {len(points)} points, "
            f"{len(model.boundaries)} boundaries, mode={config.mode.value}"
        )

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._pose_ned = _pose_to_ned(msg)
        self._pose_recv_time = self.get_clock().now()
        self._pose_stale_logged = False

    def _vel_cb(self, msg: TwistStamped) -> None:
        self._vel_ned = (
            float(msg.twist.linear.y),
            float(msg.twist.linear.x),
        )
        self._vel_recv_time = self.get_clock().now()
        self._velocity_stale_logged = False

    def _manual_cb(self, msg: Bool) -> None:
        # /spray/manual is a trusted bench-test input. In production it must
        # only be published by the server/safety UI, which owns mission-state
        # policy; this node still applies FCU fail-safes before honoring it.
        # Manual override only requires armed — OFFBOARD is NOT required so
        # bench testing works in any armed flight mode (cmd 187 is accepted
        # by PX4 in any armed mode; OFFBOARD is an auto-spray constraint only).
        if msg.data:
            if not bool(self.get_parameter("spray_enabled").value):
                self.get_logger().warn(
                    "manual spray ON rejected: spray system disabled"
                )
                self._manual_active = False
                self._manual_deadline_ns = None
            elif not self._armed:
                self.get_logger().warn(
                    "manual spray ON rejected: FCU disarmed"
                )
                self._manual_active = False
                self._manual_deadline_ns = None
            else:
                timeout_s = max(
                    0.5,
                    float(self.get_parameter("manual_override_timeout_s").value),
                )
                self._manual_active = True
                self._manual_deadline_ns = (
                    self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
                )
                self.get_logger().info(
                    f"manual spray ON (expires in {timeout_s:.1f}s)"
                )
        else:
            if self._manual_active:
                self.get_logger().info("manual spray override cancelled")
            self._manual_active = False
            self._manual_deadline_ns = None
        self._commit_desired_state()
        self._publish_manual_state()

    def _effective_desired(self) -> bool:
        """Manual ON-override wins over the auto (MARK-segment) desire."""
        return True if self._manual_active else self._desired_debounced

    def _set_auto_desired(self, desired: bool, source: str) -> None:
        if source != self._last_auto_source:
            if source == "legacy":
                self.get_logger().info("legacy /spray/active fallback used")
            self._last_auto_source = source
        self._desired_raw = bool(desired)
        self._apply_debounce()

    def _apply_debounce(self) -> None:
        if self._candidate is None or self._candidate != self._desired_raw:
            self._candidate = self._desired_raw
            self._candidate_count = 1
            self._actuator_state.debounce_state = "collecting"
        else:
            self._candidate_count += 1

        debounce_samples = max(0, int(self.get_parameter("debounce_samples").value))
        if self._candidate_count < max(1, debounce_samples):
            return
        self._actuator_state.debounce_state = "stable"
        if self._desired_debounced == self._candidate:
            if self._effective_desired() != self._commanded:
                self._commit_desired_state()
            return

        self._desired_debounced = bool(self._candidate)
        self._commit_desired_state()

    def _watchdog_tick(self) -> None:
        # Manual override hard expiry — never latches, independent of /spray/active.
        if self._manual_active and self._manual_deadline_ns is not None:
            if self.get_clock().now().nanoseconds >= self._manual_deadline_ns:
                self._manual_active = False
                self._manual_deadline_ns = None
                self.get_logger().info("manual spray override expired — reverting")
                self._commit_desired_state()

        self._dwell_expiry_tick()
        if bool(self.get_parameter("use_distance_aware_spray").value):
            self._distance_aware_tick()
        elif self._legacy_fallback_allowed():
            self._legacy_active_watchdog_tick()
        else:
            self._legacy_fallback_block_reason = (
                "legacy spray active fallback disabled in production"
            )
            self._set_auto_desired(False, source="legacy_blocked")
            if self._last_safety_block_reason != self._legacy_fallback_block_reason:
                self._last_safety_block_reason = self._legacy_fallback_block_reason

        now_safe = self._safety_allows_on()
        prev_safe = self._last_safety_allows_on
        if prev_safe is None:
            if not now_safe and (self._commanded or not self._off_confirmed):
                self._force_off("safety gate", force=True)
        elif prev_safe and not now_safe:
            self._force_off("safety gate", force=True)
        elif not now_safe and (self._commanded or not self._off_confirmed):
            self._maybe_retry_off("safety gate", force=False)
        self._last_safety_allows_on = now_safe
        self._publish_manual_state()

    def _legacy_active_watchdog_tick(self) -> None:
        timeout_s = max(0.0, float(self.get_parameter("active_timeout_s").value))
        if self._last_active_time is not None:
            age_s = (self.get_clock().now() - self._last_active_time).nanoseconds * 1e-9
            if age_s > timeout_s:
                self._desired_raw = False
                self._desired_debounced = False
                self._candidate = False
                self._candidate_count = 0
                # Staleness kills the *auto* desire only; an active manual
                # override has its own timeout and does not depend on RPP.
                if not self._manual_active:
                    self._force_off(f"/spray/active stale ({age_s:.2f}s)")
                    self._publish_manual_state()
                    return

    def _distance_aware_tick(self) -> None:
        config = self._get_config_snapshot()
        model = self._path_model
        pose_fresh, pose_age_s = self._pose_is_fresh(config)
        velocity_fresh, velocity_age_s = self._velocity_is_fresh(config)
        pose = self._pose_ned if pose_fresh else None
        vel_ned = self._vel_ned if velocity_fresh else (0.0, 0.0)

        if self._pose_recv_time is not None and not pose_fresh and not self._pose_stale_logged:
            self.get_logger().warn(f"spray pose stale ({pose_age_s:.2f}s)")
            self._pose_stale_logged = True
        if (
            self._vel_recv_time is not None
            and not velocity_fresh
            and not self._velocity_stale_logged
        ):
            self.get_logger().warn(f"spray velocity stale ({velocity_age_s:.2f}s)")
            self._velocity_stale_logged = True

        dwell_active = (
            self._dwell_state is not None
            and self._dwell_state.active
            and time.monotonic_ns() < self._dwell_state.expiry_mono_ns
        )
        safety_ok, safety_reason = self._auto_safety_status(
            pose_fresh,
            0.0,
            velocity_fresh=velocity_fresh,
            dwell_active=dwell_active,
            check_speed_window=False,
        )
        if config.mode == SprayMode.POINT:
            decision = point_mode_decision(
                dwell=self._dwell_state,
                now_mono_ns=time.monotonic_ns(),
                safety_ok=safety_ok,
                safety_reason=safety_reason,
            )
        else:
            decision = continuous_distance_decision(
                model=model,
                pose_ned=pose,
                vel_ned=vel_ned,
                safety_ok=safety_ok,
                safety_reason=safety_reason,
                config=config,
                runtime_state=self._continuous_runtime,
                dwell_active=dwell_active,
                on_value=float(self.get_parameter("on_value").value),
            )
            speed_ok, speed_reason = self._auto_safety_status(
                pose_fresh,
                decision.along_track_speed_mps,
                velocity_fresh=velocity_fresh,
                dwell_active=dwell_active,
                check_speed_window=True,
            )
            if safety_ok and not speed_ok:
                decision = SprayDecision(
                    desired=False,
                    geometry_desired=decision.geometry_desired,
                    safety_ok=False,
                    safety_reason=speed_reason,
                    projection=decision.projection,
                    next_boundary=decision.next_boundary,
                    distance_to_boundary_m=decision.distance_to_boundary_m,
                    event="SAFETY_BLOCKED",
                    debug=decision.debug,
                    target_flow=decision.target_flow,
                    target_pwm=decision.target_pwm,
                    actuator_value=decision.actuator_value,
                    along_track_speed_mps=decision.along_track_speed_mps,
                    cross_track_speed_mps=decision.cross_track_speed_mps,
                    velocity_heading_error_deg=decision.velocity_heading_error_deg,
                    raw_on_lead_m=decision.raw_on_lead_m,
                    bounded_on_lead_m=decision.bounded_on_lead_m,
                    raw_off_lead_m=decision.raw_off_lead_m,
                    bounded_off_lead_m=decision.bounded_off_lead_m,
                    lead_clamped=decision.lead_clamped,
                    lead_block_reason=decision.lead_block_reason,
                    current_run_remaining_m=decision.current_run_remaining_m,
                    next_run_length_m=decision.next_run_length_m,
                    flow_mode=decision.flow_mode,
                    raw_target_flow=decision.raw_target_flow,
                    target_paint_density=decision.target_paint_density,
                    flow_clamp_reason=decision.flow_clamp_reason,
                    flow_under_capacity=decision.flow_under_capacity,
                    command_pwm=decision.command_pwm,
                    pwm_ramp_limited=decision.pwm_ramp_limited,
                )
        with self._state_lock:
            self._last_decision = decision
            self._target_flow = float(decision.target_flow)
            self._current_pwm = float(
                decision.command_pwm if decision.command_pwm else decision.target_pwm
            )
            self._current_value = float(decision.actuator_value)
        self._publish_debug(decision.debug)

        if decision.event and decision.event != self._last_distance_event:
            if decision.event == "ON_EARLY":
                self.get_logger().info("Spray ON early before MARK start")
            elif decision.event == "OFF_EARLY":
                self.get_logger().info("Spray OFF early before MARK end")
        self._last_distance_event = decision.event

        if not decision.safety_ok:
            if decision.safety_reason != self._last_safety_block_reason:
                if decision.safety_reason:
                    self.get_logger().warn(
                        f"Safety blocked spray: {decision.safety_reason}"
                    )
                self._last_safety_block_reason = decision.safety_reason
        elif decision.safety_ok:
            self._last_safety_block_reason = ""

        source = "point" if config.mode == SprayMode.POINT else "distance"
        self._set_auto_desired(decision.desired, source=source)

    def _dwell_expiry_tick(self) -> None:
        dwell = self._dwell_state
        if dwell is None or not dwell.active:
            return
        now_ns = time.monotonic_ns()
        if now_ns < dwell.expiry_mono_ns:
            return
        self._force_off("dwell_expired", force=True)
        self._last_transition = f"dwell_expired:{dwell.point_index}"
        self._publish_runtime_status()

    def _pose_is_fresh(self, config: SprayConfiguration | None = None) -> tuple[bool, float]:
        if self._pose_recv_time is None:
            return False, float("inf")
        age_s = (self.get_clock().now() - self._pose_recv_time).nanoseconds * 1e-9
        if config is None:
            timeout_s = max(0.0, float(self.get_parameter("pose_timeout_s").value))
        else:
            timeout_s = config.safety.pose_timeout_s
        return age_s <= timeout_s, age_s

    def _velocity_is_fresh(self, config: SprayConfiguration | None = None) -> tuple[bool, float]:
        if self._vel_recv_time is None:
            return False, float("inf")
        age_s = (self.get_clock().now() - self._vel_recv_time).nanoseconds * 1e-9
        if config is None:
            timeout_s = max(0.0, float(self.get_parameter("velocity_timeout_s").value))
        else:
            timeout_s = config.safety.velocity_timeout_s
        return age_s <= timeout_s, age_s

    def _gps_gate_cb(self, msg: String) -> None:
        """Receive the server's GPS_SURVEYED runtime-safety feed (F-01 hybrid).

        The server publishes {active, ok, reason, seq} on /spray/gps_gate. We
        record it plus a monotonic receive time so the node can independently
        fail-closed if the feed goes stale (server watchdog death)."""
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warn("invalid /spray/gps_gate payload ignored")
            return
        active = bool(data.get("active", False))
        ok = bool(data.get("ok", False))
        reason = str(data.get("reason", ""))
        try:
            seq = int(data.get("seq", 0))
        except (TypeError, ValueError):
            seq = 0
        with self._state_lock:
            self._gps_gate_active = active
            self._gps_gate_ok = ok
            self._gps_gate_reason = reason
            self._gps_gate_seq = seq
            self._gps_gate_recv_time = time.monotonic()
        # Drop spray immediately on a fault edge rather than waiting for the next
        # decision tick — but only when spray is (or might be) ON, to avoid
        # re-issuing OFF on every heartbeat while already safely off.
        if active and not ok and (self._commanded or not self._off_confirmed):
            self._force_off(f"GPS_SURVEYED safety: {reason or 'not ok'}", force=True)

    def _evaluate_gps_surveyed_runtime_safety(self) -> tuple[bool, str]:
        """Independent node-side GPS_SURVEYED gate (hybrid fallback for F-01).

        Engages only when the server feeds an active surveyed gate, so ordinary
        LOCAL_NED missions are never RTK-gated. Returns (False, reason) when the
        server reports unsafe OR when the gate feed is stale/absent — the latter
        keeps spray OFF even if the server watchdog dies (regression case #7)."""
        with self._state_lock:
            active = self._gps_gate_active
            ok = self._gps_gate_ok
            reason = self._gps_gate_reason
            recv_time = self._gps_gate_recv_time
        if not active:
            return True, ""
        max_age = float(self.get_parameter("gps_runtime_gate_max_age_s").value)
        now = time.monotonic()
        if recv_time is None or (now - recv_time) > max_age:
            age = float("inf") if recv_time is None else (now - recv_time)
            return False, (
                f"GPS_SURVEYED gate feed stale/absent ({age:.1f}s > {max_age:.1f}s)"
            )
        if not ok:
            return False, reason or "GPS_SURVEYED safety not satisfied"
        return True, ""

    def _auto_safety_status(
        self,
        pose_fresh: bool,
        along_track_speed: float,
        velocity_fresh: bool = True,
        dwell_active: bool = False,
        check_speed_window: bool = True,
    ) -> tuple[bool, str]:
        config = self._get_config_snapshot()
        if not self._config_ready:
            return False, self._config_error or "spray configuration not ready"
        ok, reason = auto_safety_status(
            config=config,
            armed=self._armed,
            mode=self._mode,
            path_model=self._path_model,
            pose_fresh=pose_fresh,
            along_track_speed=along_track_speed,
            velocity_fresh=velocity_fresh,
            dwell_active=dwell_active,
            check_speed_window=check_speed_window,
        )
        if not ok:
            return ok, reason
        # F-01 (hybrid): AND-in the independent GPS_SURVEYED gate last so a
        # surveyed RTK fault/feed-loss blocks spray ON regardless of geometry.
        gps_ok, gps_reason = self._evaluate_gps_surveyed_runtime_safety()
        if not gps_ok:
            return False, gps_reason
        return True, reason

    def _reassert_tick(self) -> None:
        if (
            self._effective_desired()
            and self._commanded
            and self._safety_allows_on()
            and not self._actuator_state.pending
        ):
            self._send_command(True, reason="reassert")
        elif not self._effective_desired() and not self._off_confirmed:
            self._maybe_retry_off("OFF reassert")

    def _commit_desired_state(self) -> None:
        desired = self._effective_desired()
        if self._actuator_state.desired_on != desired:
            self._actuator_state.desired_on = desired
            self._publish_desired_state(desired)
        if desired and not self._safety_allows_on():
            self._force_off("desired ON blocked by safety gate")
            return
        if not desired:
            if self._commanded or not self._off_confirmed:
                self._maybe_retry_off("desired OFF")
            return
        if desired != self._commanded and not (
            self._actuator_state.pending and self._actuator_state.pending_on is True
        ):
            self._send_command(desired, reason="edge")

    def _vehicle_state_freshness(self) -> tuple[bool, float, str]:
        if self._state_recv_monotonic_s is None:
            return False, float("inf"), "vehicle state never received"
        age_s = time.monotonic() - self._state_recv_monotonic_s
        timeout_s = max(
            0.0, float(self.get_parameter("vehicle_state_timeout_s").value)
        )
        if age_s > timeout_s:
            return False, age_s, "vehicle state stale"
        return True, age_s, ""

    def _safety_allows_on(self) -> bool:
        state_fresh, _, _ = self._vehicle_state_freshness()
        if not state_fresh:
            return False
        if not bool(self.get_parameter("spray_enabled").value):
            return False
        if not self._armed:
            return False
        if self._manual_active:
            # Manual bench-test: armed is sufficient. OFFBOARD is enforced for
            # autonomous spray only — cmd 187 is accepted in any armed mode.
            return True
        require_offboard = bool(self.get_parameter("require_offboard").value)
        if require_offboard and self._mode != "OFFBOARD":
            return False
        return True

    def _force_off(self, reason: str, force: bool = False) -> None:
        self._invalidate_dwell(reason)
        self._manual_active = False
        self._manual_deadline_ns = None
        self._actuator_state.desired_on = False
        self._publish_desired_state(False)
        if self._commanded or not self._off_confirmed:
            self.get_logger().warn(f"forcing spray OFF: {reason}", throttle_duration_sec=1.0)
            # force=True only on a genuine edge (safety-loss transition,
            # shutdown). The periodic watchdog call leaves force=False so the
            # retry honors the 0.5 s throttle instead of firing every tick.
            self._maybe_retry_off(f"failsafe: {reason}", force=force)
        else:
            self._publish_state(False)

    def _maybe_retry_off(self, reason: str, force: bool = False) -> None:
        now_ns = self.get_clock().now().nanoseconds
        retry_interval_ns = 500_000_000
        if (
            not force
            and self._last_off_send_time_ns is not None
            and now_ns - self._last_off_send_time_ns < retry_interval_ns
        ):
            return
        if self._actuator_state.pending and self._actuator_state.pending_on is False:
            return
        self.get_logger().warn(
            f"retrying spray OFF command: {reason}",
            throttle_duration_sec=1.0,
        )
        self._send_command(False, reason=reason)

    def _send_command(self, on: bool, reason: str) -> None:
        if on and not self._safety_allows_on():
            on = False
        # A new command intent supersedes any in-flight request; bump the id
        # before the service-ready check so a stale reply is invalidated even
        # when the new intent cannot be dispatched.
        self._cmd_seq += 1
        seq = self._cmd_seq
        self._actuator_state.requested_on = bool(on)
        self._actuator_state.command_sequence = seq
        if not self._service_ready:
            self.get_logger().warn(
                "spray command service not ready; command suppressed",
                throttle_duration_sec=1.0,
            )
            if not on:
                self._off_confirmed = False
                self._actuator_state.off_confirmed = False
                self._actuator_state.off_confirmation_source = "none"
            self._actuator_state.last_command_failure = "service not ready"
            return

        req = self._build_command_request(on)
        target_pwm, target_value = self._command_output_from_request(req, on)
        self._actuator_state.pending = True
        self._actuator_state.pending_on = bool(on)
        self._actuator_state.pending_sequence = seq
        self._actuator_state.pending_pwm = target_pwm
        self._actuator_state.pending_value = target_value
        if on:
            self._off_confirmed = False
            self._actuator_state.off_confirmed = False
            self._actuator_state.off_confirmation_source = "none"
            self._actuator_state.last_on_timestamp_s = self.get_clock().now().nanoseconds * 1e-9
        else:
            self._off_confirmed = False
            self._actuator_state.off_confirmed = False
            self._actuator_state.off_confirmation_source = "none"
            self._last_off_send_time_ns = self.get_clock().now().nanoseconds
            self._actuator_state.last_off_timestamp_s = self._last_off_send_time_ns * 1e-9
        future = self._command_cli.call_async(req)
        future.add_done_callback(
            lambda fut, requested=on, why=reason, s=seq: self._command_done(fut, requested, why, s)
        )

    def _command_output_from_request(
        self, req: CommandLong.Request, on: bool
    ) -> tuple[float, float]:
        backend = self._validate_actuator_backend()
        if backend == "mavlink_servo_pwm":
            return float(req.param2), float(req.param2)
        if on:
            set_index = int(self.get_parameter("actuator_set_index").value)
            if set_index < 1 or set_index > 6:
                set_index = 1
            value = float(
                [
                    req.param1,
                    req.param2,
                    req.param3,
                    req.param4,
                    req.param5,
                    req.param6,
                ][set_index - 1]
            )
            return float(self._current_pwm), value
        off_value = float(
            self._get_config_snapshot().calibration.actuator_limits.off_value
        )
        return float(self._get_config_snapshot().calibration.actuator_limits.off_pwm), off_value

    def _build_command_request(self, on: bool) -> CommandLong.Request:
        req = CommandLong.Request()
        req.broadcast = False
        req.confirmation = 0
        backend = self._validate_actuator_backend()
        if backend == "mavlink_servo_pwm":
            return self._build_servo_pwm_request(req, on)
        if backend == "mavlink_actuator":
            return self._build_actuator_request(req, on)
        raise RuntimeError(f"unreachable actuator_backend={backend!r}")

    def _validate_actuator_backend(self) -> str:
        backend = str(self.get_parameter("actuator_backend").value)
        if backend not in _VALID_ACTUATOR_BACKENDS:
            raise ValueError(
                "Unknown actuator_backend="
                f"{backend!r}; expected one of {sorted(_VALID_ACTUATOR_BACKENDS)}"
            )
        return backend

    def _build_actuator_request(self, req: CommandLong.Request, on: bool) -> CommandLong.Request:
        set_index = int(self.get_parameter("actuator_set_index").value)
        if set_index < 1 or set_index > 6:
            self.get_logger().warn(
                f"actuator_set_index={set_index} out of range 1..6; using 1",
                throttle_duration_sec=5.0,
            )
            set_index = 1
        if on:
            # Manual override is full configured ON — it must NOT reuse the
            # autonomous decision value (self._current_value), which a latched
            # continuous-mode path leaves at off_value whenever the rover is not
            # in an active MARK zone. Using it silently turned manual /spray/on
            # into a no-op (command accepted, actuator value = OFF).
            if self._manual_active:
                value = float(self.get_parameter("on_value").value)
            elif self._last_decision is not None:
                value = float(self._current_value)
            else:
                value = float(self.get_parameter("on_value").value)
        else:
            value = float(
                self._get_config_snapshot().calibration.actuator_limits.off_value
            )
        req.command = MAV_CMD_DO_SET_ACTUATOR
        params = [math.nan] * 6
        params[set_index - 1] = value
        req.param1, req.param2, req.param3 = params[0], params[1], params[2]
        req.param4, req.param5, req.param6 = params[3], params[4], params[5]
        req.param7 = 0.0
        return req

    def _build_servo_pwm_request(self, req: CommandLong.Request, on: bool) -> CommandLong.Request:
        instance = int(self.get_parameter("servo_instance").value)
        if on:
            if self._manual_active:
                pwm = int(round(float(self.get_parameter("on_pwm_us").value)))
            else:
                pwm = int(round(
                    self._current_pwm
                    if self._last_decision is not None
                    else float(self.get_parameter("on_pwm_us").value)
                ))
            pwm = max(0, min(pwm, _SERVO_PWM_MAX_US))
        else:
            pwm = int(round(self._get_config_snapshot().calibration.actuator_limits.off_pwm))
        self._actuator_state.current_pwm = float(pwm)
        self._actuator_state.current_value = float(pwm)
        self.get_logger().info(
            f"Sending spray {'ON' if on else 'OFF'} PWM {pwm}µs (instance={instance})",
            throttle_duration_sec=1.0,
        )
        req.command = MAV_CMD_DO_SET_SERVO
        req.param1 = float(instance)
        req.param2 = float(pwm)
        req.param3 = req.param4 = req.param5 = req.param6 = req.param7 = 0.0
        return req

    def _command_done(self, future, requested: bool, reason: str, seq: int) -> None:
        if seq != self._cmd_seq:
            # A newer command was issued before this result arrived; ignoring
            # it prevents a stale reply from corrupting current spray state.
            self.get_logger().debug(
                f"ignoring stale spray command result "
                f"(seq={seq}, latest={self._cmd_seq}, requested={requested}, reason={reason})"
            )
            return
        try:
            resp = future.result()
        except Exception as exc:
            self._clear_pending_command(seq)
            self._actuator_state.last_command_failure = str(exc)
            if not requested:
                self._off_confirmed = False
                self.get_logger().warn(
                    f"spray OFF command {reason} failed; will retry: {exc}"
                )
            else:
                self.get_logger().warn(f"spray command {reason} failed: {exc}")
                self._maybe_retry_off("ON command failure", force=True)
            return
        success = bool(getattr(resp, "success", False))
        result = getattr(resp, "result", None)
        if not success:
            self._clear_pending_command(seq)
            self._actuator_state.last_command_failure = f"result={result}"
            if not requested:
                self._off_confirmed = False
                self.get_logger().warn(
                    f"spray OFF command {reason} rejected; will retry: result={result}"
                )
            else:
                self.get_logger().warn(
                    f"spray command {reason} rejected: requested={requested} result={result}"
                )
                self._maybe_retry_off("ON command rejected", force=True)
            return
        pending_pwm = self._actuator_state.pending_pwm
        pending_value = self._actuator_state.pending_value
        self._clear_pending_command(seq)
        self._actuator_state.last_command_failure = ""
        if not requested:
            self._off_confirmed = True
            self._actuator_state.off_confirmed = True
            self._actuator_state.off_confirmation_source = "command_ack"
            self._commanded = False
            self._actuator_state.commanded_on = False
            self._actuator_state.accepted_on = False
            self._actuator_state.accepted_sequence = seq
            self._actuator_state.current_on = False
            self._actuator_state.current_pwm = pending_pwm
            self._actuator_state.current_value = pending_value
            self._publish_state(False)
        else:
            self._off_confirmed = False
            self._actuator_state.off_confirmed = False
            self._actuator_state.off_confirmation_source = "none"
            self._commanded = True
            self._actuator_state.commanded_on = True
            self._actuator_state.accepted_on = True
            self._actuator_state.accepted_sequence = seq
            self._actuator_state.current_on = True
            self._actuator_state.current_pwm = pending_pwm
            self._actuator_state.current_value = pending_value
            self._publish_state(True)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        started = (
            self._actuator_state.last_on_timestamp_s
            if requested else self._actuator_state.last_off_timestamp_s
        )
        if started:
            self._actuator_state.command_duration_s = max(0.0, now_s - started)

    def _clear_pending_command(self, seq: int) -> None:
        if self._actuator_state.pending_sequence != seq:
            return
        self._actuator_state.pending = False
        self._actuator_state.pending_on = None
        self._actuator_state.pending_sequence = 0

    def _publish_state(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self._state_pub.publish(msg)
        self._commanded_pub.publish(msg)

    def _publish_desired_state(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self._desired_pub.publish(msg)

    def _publish_debug(self, values: list[float]) -> None:
        msg = Float32MultiArray()
        msg.data = [float(v) for v in values]
        self._debug_pub.publish(msg)

    def _publish_manual_state(self) -> None:
        msg = Bool()
        msg.data = bool(self._manual_active)
        self._manual_state_pub.publish(msg)

    def shutdown_off(self) -> None:
        self._invalidate_dwell("shutdown")
        self._desired_raw = False
        self._desired_debounced = False
        self._manual_active = False
        self._manual_deadline_ns = None
        self._maybe_retry_off("shutdown", force=True)
        # Flush: spin briefly so the OFF actually reaches MAVROS and is
        # confirmed before the executor stops. Best-effort and bounded so
        # shutdown can never hang.
        spin_once = getattr(rclpy, "spin_once", None)
        if spin_once is None:
            return
        deadline = time.monotonic() + 1.0
        while not self._off_confirmed and time.monotonic() < deadline:
            try:
                spin_once(self, timeout_sec=0.1)
            except Exception:
                break
            if not self._off_confirmed:
                self._maybe_retry_off("shutdown flush", force=True)


def main() -> None:
    rclpy.init()
    node: SprayControllerNode | None = None
    try:
        node = SprayControllerNode()

        def _signal_handler(signum, frame):
            raise KeyboardInterrupt

        signal.signal(signal.SIGINT, _signal_handler)
        signal.signal(signal.SIGTERM, _signal_handler)
        # Single-threaded executor: rclpy's MultiThreadedExecutor busy-spins a
        # full core with timers on Humble, which starved MAVROS on the Jetson
        # (FCU read as disconnected). All heavy work here is already async
        # (call_async), so one thread is sufficient; serialized callbacks also
        # remove any cross-callback-group data race (_state_lock kept as guard).
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.shutdown_off()
            except Exception:
                pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
