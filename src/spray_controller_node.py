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
)


MAV_CMD_DO_SET_ACTUATOR = 187
MAV_CMD_DO_SET_SERVO = 183
_SERVO_PWM_MAX_US = 2200
from spray_path_model import (  # noqa: E402
    MARK_TO_TRANSIT,
    TRANSIT_TO_MARK,
    SprayBoundary,
    SprayDecision,
    SprayPathModel,
    SprayProjection,
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
    commanded_on: bool = False
    current_on: bool = False
    debounce_state: str = "idle"
    last_on_timestamp_s: float = 0.0
    last_off_timestamp_s: float = 0.0
    command_duration_s: float = 0.0
    command_sequence: int = 0
    current_pwm: float = 0.0
    current_value: float = 0.0
    last_command_failure: str = ""
    off_confirmed: bool = False


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
        self.declare_parameter("allow_legacy_spray_active_fallback", True)
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
        self._service_ready = False
        # Actuator state is UNKNOWN at startup — a previous instance may have
        # left the output ON. Start unconfirmed so the node drives a confirmed
        # OFF before trusting the believed state (see end of __init__).
        self._off_confirmed = False
        self._last_off_send_time_ns: Optional[int] = None
        # Monotonic command id. Each dispatched command carries the id current
        # at send time; _command_done ignores any result that is not the latest
        # so a late/out-of-order MAVROS reply cannot overwrite newer state.
        self._cmd_seq = 0
        self._actuator_state = ActuatorState()
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

    def _reset_decision_state(self, reason: str) -> None:
        self._candidate = None
        self._candidate_count = 0
        self._desired_raw = False
        self._desired_debounced = False
        self._last_distance_event = ""
        self._last_safety_block_reason = ""
        self._last_transition = reason

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
            snapshot = {
                "model_revision": self._model_revision,
                "commanded_on": self._commanded,
                "confirmed_off": self._off_confirmed and not self._commanded,
                "last_transition": self._last_transition,
                "conditioned_path_source": self._conditioned_path_source,
                "path_identity": dict(self._conditioned_path_identity),
                "last_decision": self._last_decision,
                "target_flow": self._target_flow,
                "current_pwm": self._current_pwm,
                "current_value": self._current_value,
                "actuator": self._actuator_state,
            }
        return {
            "timestamp_monotonic_s": now_ns * 1e-9,
            "spray_mode": config.mode.value,
            "active_mode": config.mode.value,
            "configuration_revision": config.revision,
            "model_revision": snapshot["model_revision"],
            "ready": self._config_ready,
            "operator_enabled": bool(self.get_parameter("spray_enabled").value),
            "commanded_on": snapshot["commanded_on"],
            "confirmed_off": snapshot["confirmed_off"],
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
            "target_flow": snapshot["target_flow"],
            "current_pwm": snapshot["current_pwm"],
            "current_value": snapshot["current_value"],
            "spray_state": "COMMAND_ON" if snapshot["commanded_on"] else "COMMAND_OFF",
            "commanded_spray_state": "ON" if snapshot["commanded_on"] else "OFF",
            "software_actuator_state": "ON" if snapshot["actuator"].current_on else "OFF",
            "physical_actuator_state": "UNAVAILABLE",
            "physical_confirmation_available": False,
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
            "actuator_failure_state": snapshot["actuator"].last_command_failure,
            "actuator": {
                "commanded_on": snapshot["actuator"].commanded_on,
                "current_on": snapshot["actuator"].current_on,
                "debounce_state": snapshot["actuator"].debounce_state,
                "last_on_timestamp_s": snapshot["actuator"].last_on_timestamp_s,
                "last_off_timestamp_s": snapshot["actuator"].last_off_timestamp_s,
                "command_duration_s": snapshot["actuator"].command_duration_s,
                "command_sequence": snapshot["actuator"].command_sequence,
                "current_pwm": snapshot["actuator"].current_pwm,
                "current_value": snapshot["actuator"].current_value,
                "last_command_failure": snapshot["actuator"].last_command_failure,
                "off_confirmed": snapshot["actuator"].off_confirmed,
            },
        }

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
        self._armed = bool(msg.armed)
        self._mode = str(msg.mode)
        now_safe = self._safety_allows_on()
        if prev_safe and not now_safe:
            self._invalidate_dwell("safety loss")
            self._force_off("FCU left armed/OFFBOARD safe state", force=True)
        elif not prev_safe and now_safe and self._desired_debounced:
            self._commit_desired_state()

    def _active_cb(self, msg: Bool) -> None:
        self._last_active_time = self.get_clock().now()
        self._legacy_active_raw = bool(msg.data)
        if (
            not bool(self.get_parameter("use_distance_aware_spray").value)
            and bool(self.get_parameter("allow_legacy_spray_active_fallback").value)
        ):
            self._set_auto_desired(self._legacy_active_raw, source="legacy")

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
        except ValueError as exc:
            self._reject_conditioned_path(f"invalid conditioned path: {exc}")
            self.get_logger().warn(f"spray path rejected: {exc}")
            return
        current = self._get_config_snapshot()
        if current.revision != config.revision or current.mode != config.mode:
            self.get_logger().warn("discarded path model built for replaced spray configuration")
            return
        with self._state_lock:
            self._path_model = model
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
        elif bool(self.get_parameter("allow_legacy_spray_active_fallback").value):
            self._legacy_active_watchdog_tick()
        else:
            self._set_auto_desired(False, source="disabled")

        if not self._safety_allows_on():
            # Periodic enforcement — throttled so a stuck/failing OFF retries at
            # the retry cadence rather than flooding MAVROS at the tick rate.
            self._force_off("safety gate")
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
        speed = math.hypot(self._vel_ned[0], self._vel_ned[1]) if velocity_fresh else 0.0

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
            speed,
            velocity_fresh=velocity_fresh,
            dwell_active=dwell_active,
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
                speed_mps=speed,
                safety_ok=safety_ok,
                safety_reason=safety_reason,
                config=config,
            )
        with self._state_lock:
            self._last_decision = decision
            self._target_flow = float(decision.target_flow)
            self._current_pwm = float(decision.target_pwm)
            self._current_value = float(decision.actuator_value)
        self._publish_debug(decision.debug)

        if decision.event and decision.event != self._last_distance_event:
            if decision.event == "ON_EARLY":
                self.get_logger().info("Spray ON early before MARK start")
            elif decision.event == "OFF_EARLY":
                self.get_logger().info("Spray OFF early before MARK end")
        self._last_distance_event = decision.event

        if decision.geometry_desired and not decision.safety_ok:
            if decision.safety_reason != self._last_safety_block_reason:
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

    def _auto_safety_status(
        self,
        pose_fresh: bool,
        speed: float,
        velocity_fresh: bool = True,
        dwell_active: bool = False,
    ) -> tuple[bool, str]:
        config = self._get_config_snapshot()
        if not self._config_ready:
            return False, self._config_error or "spray configuration not ready"
        return auto_safety_status(
            config=config,
            armed=self._armed,
            mode=self._mode,
            path_model=self._path_model,
            pose_fresh=pose_fresh,
            speed=speed,
            velocity_fresh=velocity_fresh,
            dwell_active=dwell_active,
        )

    def _reassert_tick(self) -> None:
        if self._effective_desired() and self._commanded and self._safety_allows_on():
            self._send_command(True, reason="reassert")
        elif not self._effective_desired() and not self._off_confirmed:
            self._maybe_retry_off("OFF reassert")

    def _commit_desired_state(self) -> None:
        desired = self._effective_desired()
        self._publish_desired_state(desired)
        if desired and not self._safety_allows_on():
            self._force_off("desired ON blocked by safety gate")
            return
        if not desired:
            if self._commanded or not self._off_confirmed:
                self._maybe_retry_off("desired OFF")
            return
        if desired != self._commanded:
            self._send_command(desired, reason="edge")

    def _safety_allows_on(self) -> bool:
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
        self._actuator_state.commanded_on = bool(on)
        self._actuator_state.command_sequence = seq
        if not self._service_ready:
            self.get_logger().warn(
                "spray command service not ready; command suppressed",
                throttle_duration_sec=1.0,
            )
            if not on:
                self._off_confirmed = False
                self._actuator_state.off_confirmed = False
            self._actuator_state.last_command_failure = "service not ready"
            return

        if on:
            self._off_confirmed = False
            self._actuator_state.off_confirmed = False
            self._actuator_state.last_on_timestamp_s = self.get_clock().now().nanoseconds * 1e-9
        else:
            self._off_confirmed = False
            self._actuator_state.off_confirmed = False
            self._last_off_send_time_ns = self.get_clock().now().nanoseconds
            self._actuator_state.last_off_timestamp_s = self._last_off_send_time_ns * 1e-9
        if on:
            self._commanded = True
            self._actuator_state.current_on = True
            self._actuator_state.current_pwm = self._current_pwm
            self._actuator_state.current_value = self._current_value
            self._publish_state(True)
        req = self._build_command_request(on)
        future = self._command_cli.call_async(req)
        future.add_done_callback(
            lambda fut, requested=on, why=reason, s=seq: self._command_done(fut, requested, why, s)
        )

    def _build_command_request(self, on: bool) -> CommandLong.Request:
        req = CommandLong.Request()
        req.broadcast = False
        req.confirmation = 0
        backend = str(self.get_parameter("actuator_backend").value)
        if backend == "mavlink_servo_pwm":
            return self._build_servo_pwm_request(req, on)
        elif backend == "mavlink_actuator":
            return self._build_actuator_request(req, on)
        else:
            self.get_logger().error(
                f"Unknown actuator_backend={backend!r}; sending OFF via mavlink_servo_pwm",
                throttle_duration_sec=5.0,
            )
            return self._build_servo_pwm_request(req, False)

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
            self._actuator_state.last_command_failure = str(exc)
            if not requested:
                self._off_confirmed = False
                self.get_logger().warn(
                    f"spray OFF command {reason} failed; will retry: {exc}"
                )
            else:
                self.get_logger().warn(f"spray command {reason} failed: {exc}")
                self._commanded = False
                self._actuator_state.current_on = False
                self._maybe_retry_off("ON command failure", force=True)
            return
        success = bool(getattr(resp, "success", False))
        result = getattr(resp, "result", None)
        if not success:
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
                self._commanded = False
                self._actuator_state.current_on = False
                self._maybe_retry_off("ON command rejected", force=True)
            return
        if not requested:
            self._off_confirmed = True
            self._actuator_state.off_confirmed = True
            self._commanded = False
            self._actuator_state.current_on = False
            self._actuator_state.current_pwm = (
                self._get_config_snapshot().calibration.actuator_limits.off_pwm
            )
            self._actuator_state.current_value = (
                self._get_config_snapshot().calibration.actuator_limits.off_value
            )
            self._publish_state(False)
        else:
            self._actuator_state.last_command_failure = ""
        now_s = self.get_clock().now().nanoseconds * 1e-9
        started = (
            self._actuator_state.last_on_timestamp_s
            if requested else self._actuator_state.last_off_timestamp_s
        )
        if started:
            self._actuator_state.command_duration_s = max(0.0, now_s - started)

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
