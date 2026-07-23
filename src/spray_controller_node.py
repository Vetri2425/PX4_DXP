#!/usr/bin/env python3
"""Spray actuator controller for PX4 AUX outputs via MAVROS CommandLong.

Subscribes to /spray/active (desired MARK state from RPP), applies debounce
and safety gates, then commands MAV_CMD_DO_SET_ACTUATOR. The controller only
drives an already-configured PX4 actuator set output; QGC remains the source
of truth for AUX pin/function/PWM limits.

Manual override (/spray/manual, std_msgs/Bool) lets the server bench-test the
actuator: True holds spray ON for at most `manual_override_timeout_s`
(node-side hard expiry — never latches), False cancels immediately. The
override is subordinate to every fail-safe: disarm, mode loss, and node
shutdown all clear it. While the override is active the /spray/active
staleness watchdog only clears the *auto* desire (manual has its own timeout
and does not depend on the RPP stream). Actual override state is reported on
/spray/manual_state for the server.

Spray Controller V2, Phase A (docs/Architecture/SPRAY_CONTROLLER_V2_PLAN.md,
Mode 1 / continuous only, behavior-preserving): actuator command state is now
owned by `spray_fsm.SpraySafetyStateMachine` instead of scattered booleans
(`_commanded`, `_off_confirmed`, `_cmd_seq`, ad-hoc retry timers), and the
node maintains an internal `spray_session_config.SpraySessionConfig`
representation of the mission geometry alongside the existing `_path_model`.
A new `/spray/status` (std_msgs/String, JSON) publishes a typed
`spray_status.SpraySessionStatus` snapshot every control tick. All existing
Mode-1 distance-aware behavior, topics, and ROS params are unchanged, with
one intended exception: `/spray/state` now reflects a *confirmed* ON only
(see `_publish_actuator_state`).
"""

from __future__ import annotations

import json
import math
import signal
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import GPSRAW, State
from mavros_msgs.srv import CommandLong
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32MultiArray, String

from spray_fsm import SpraySafetyStateMachine, SprayCommand, SprayState
from spray_flow_model import FlowModulator
from spray_modes import DashMeter, PointMeter
from spray_session_config import (
    ConfigSchemaError,
    SpraySessionConfig,
    cleared_config,
    continuous_config_from_path,
    parse_session_config,
)
from spray_status import make_status, status_to_json_safe


MAV_CMD_DO_SET_ACTUATOR = 187
MAV_CMD_DO_SET_SERVO = 183
_SERVO_PWM_MAX_US = 2200
TRANSIT_TO_MARK = "TRANSIT_TO_MARK"
MARK_TO_TRANSIT = "MARK_TO_TRANSIT"

# Mirrors rpp_controller_node.SegmentStateCode.CORNER_ALIGN, read off
# /rpp/segment_debug data[1]. Duplicated rather than imported: the spray node
# must not take a build/runtime dependency on the controller module, and this
# node already runs standalone in tests. If the RPP's enum ever renumbers, this
# breaks silently -- test_spray_pivot_gate.py pins the contract.
_SEGMENT_STATE_CORNER_ALIGN = 3

# GPSRAW.fix_type → human name (Phase B RTK gate, §7.6). Same mapping the RPP
# node's P0.3 gate uses. 6 = RTK_FIXED (the marking bar), 5 = RTK_FLOAT.
_GPS_FIX_NAMES = {
    0: "NO_FIX", 1: "NO_FIX", 2: "2D", 3: "3D",
    4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED",
}


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


@dataclass(frozen=True)
class SprayBoundary:
    s: float
    kind: str


@dataclass(frozen=True)
class SprayPathModel:
    points: list[tuple[float, float]]
    flags: list[bool]
    cumulative_s: list[float]
    boundaries: list[SprayBoundary]


@dataclass(frozen=True)
class SprayProjection:
    segment_index: int
    t: float
    proj_n: float
    proj_e: float
    s: float
    xtrack_error_m: float
    current_flag: bool


@dataclass(frozen=True)
class SprayDecision:
    desired: bool
    geometry_desired: bool
    safety_ok: bool
    safety_reason: str
    projection: Optional[SprayProjection]
    next_boundary: Optional[SprayBoundary]
    distance_to_boundary_m: float
    event: str
    debug: list[float]
    point_update: object = None  # Phase D: the PointUpdate for this tick, else None


def _build_path_model(
    points: list[tuple[float, float]],
    flags: list[bool],
) -> SprayPathModel:
    clean_points = [(float(n), float(e)) for n, e in points]
    clean_flags = [bool(f) for f in flags]
    if len(clean_points) != len(clean_flags):
        raise ValueError("points and flags must have equal length")
    cumulative_s: list[float] = []
    total = 0.0
    for i, point in enumerate(clean_points):
        if i > 0:
            prev = clean_points[i - 1]
            total += math.hypot(point[0] - prev[0], point[1] - prev[1])
        cumulative_s.append(total)

    boundaries: list[SprayBoundary] = []
    for i in range(1, len(clean_flags)):
        if clean_flags[i - 1] == clean_flags[i]:
            continue
        kind = TRANSIT_TO_MARK if clean_flags[i] else MARK_TO_TRANSIT
        boundaries.append(SprayBoundary(cumulative_s[i], kind))

    # Hardening: if the path ends on a MARK point there is no terminal
    # MARK->TRANSIT boundary, so _next_boundary returns None and off_early can
    # never fire — the nozzle can latch ON at the endpoint. The engine now
    # appends a trailing TRANSIT run-out (see engine.py merge step), but a path
    # from any other source could still end on MARK; synthesize the terminal
    # boundary at the final station so shutoff is guaranteed regardless.
    if clean_flags and clean_flags[-1]:
        boundaries.append(SprayBoundary(cumulative_s[-1], MARK_TO_TRANSIT))

    return SprayPathModel(clean_points, clean_flags, cumulative_s, boundaries)


def _yaw_ned_from_enu_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_enu = math.atan2(siny_cosp, cosy_cosp)
    return (math.pi / 2.0 - yaw_enu + math.pi) % (2.0 * math.pi) - math.pi


def _pose_to_ned(pose_msg: PoseStamped) -> tuple[float, float, float]:
    north = float(pose_msg.pose.position.y)
    east = float(pose_msg.pose.position.x)
    yaw_ned = _yaw_ned_from_enu_quaternion(pose_msg.pose.orientation)
    return north, east, yaw_ned


def _nozzle_position_ned(
    pose_n: float,
    pose_e: float,
    yaw_ned: float,
    forward_offset_m: float,
    lateral_offset_m: float,
) -> tuple[float, float]:
    """Apply body-frame nozzle offsets in NED; lateral is positive to rover-right."""
    nozzle_n = (
        pose_n
        + forward_offset_m * math.cos(yaw_ned)
        - lateral_offset_m * math.sin(yaw_ned)
    )
    nozzle_e = (
        pose_e
        + forward_offset_m * math.sin(yaw_ned)
        + lateral_offset_m * math.cos(yaw_ned)
    )
    return nozzle_n, nozzle_e


def _project_onto_path(
    model: SprayPathModel,
    point_n: float,
    point_e: float,
) -> Optional[SprayProjection]:
    if not model.points:
        return None
    if len(model.points) == 1:
        n, e = model.points[0]
        return SprayProjection(
            segment_index=0,
            t=0.0,
            proj_n=n,
            proj_e=e,
            s=0.0,
            xtrack_error_m=math.hypot(point_n - n, point_e - e),
            current_flag=model.flags[0],
        )

    best: Optional[SprayProjection] = None
    best_dist = float("inf")
    for i in range(len(model.points) - 1):
        a_n, a_e = model.points[i]
        b_n, b_e = model.points[i + 1]
        d_n = b_n - a_n
        d_e = b_e - a_e
        seg_len_sq = d_n * d_n + d_e * d_e
        if seg_len_sq <= 1e-12:
            t = 0.0
            proj_n, proj_e = a_n, a_e
            seg_len = 0.0
        else:
            t = ((point_n - a_n) * d_n + (point_e - a_e) * d_e) / seg_len_sq
            t = max(0.0, min(1.0, t))
            proj_n = a_n + t * d_n
            proj_e = a_e + t * d_e
            seg_len = math.sqrt(seg_len_sq)

        dist = math.hypot(point_n - proj_n, point_e - proj_e)
        # Equal-distance ties happen exactly at shared vertices. Prefer the
        # later segment so a TRANSIT->MARK vertex is considered MARK, and a
        # MARK->TRANSIT vertex is considered TRANSIT.
        if dist < best_dist - 1e-12 or abs(dist - best_dist) <= 1e-12:
            current_flag = model.flags[i + 1] if t >= 1.0 - 1e-12 else model.flags[i]
            best_dist = dist
            best = SprayProjection(
                segment_index=i,
                t=t,
                proj_n=proj_n,
                proj_e=proj_e,
                s=model.cumulative_s[i] + t * seg_len,
                xtrack_error_m=dist,
                current_flag=current_flag,
            )
    return best


def _next_boundary(
    model: SprayPathModel,
    current_s: float,
    current_flag: bool,
) -> Optional[SprayBoundary]:
    wanted = MARK_TO_TRANSIT if current_flag else TRANSIT_TO_MARK
    for boundary in model.boundaries:
        if boundary.kind == wanted and boundary.s > current_s + 1e-9:
            return boundary
    return None


def _make_spray_decision(
    model: Optional[SprayPathModel],
    nozzle_n: Optional[float],
    nozzle_e: Optional[float],
    speed_mps: float,
    safety_ok: bool,
    safety_reason: str,
    solenoid_open_delay_s: float,
    solenoid_close_delay_s: float,
    on_overspray_margin_m: float,
    off_overspray_margin_m: float,
    max_xtrack_error_m: float,
    mode: str = "continuous",
    dash_meter: Optional["DashMeter"] = None,
    dt_s: float = 0.0,
    point_meter: Optional["PointMeter"] = None,
    yaw: float = 0.0,
    now_s: float = 0.0,
    off_confirmed: bool = False,
) -> SprayDecision:
    projection: Optional[SprayProjection] = None
    boundary: Optional[SprayBoundary] = None
    distance_to_boundary = float("inf")
    geometry_desired = False
    event = ""

    # Point mode (plan §7.3): no path projection — arrival is the nozzle vs the
    # coordinate list. Uses the standard safety_ok from the gate stack (armed/
    # offboard/pose/gps/pivot-exempt), computed by the caller.
    if mode == "point" and point_meter is not None:
        pu = None
        if nozzle_n is not None and nozzle_e is not None:
            pu = point_meter.update(
                nozzle_n, nozzle_e, yaw, speed_mps, now_s, off_confirmed
            )
            geometry_desired = pu.geometry_desired
        desired = bool(geometry_desired and safety_ok)
        debug = [
            2.0,  # [0] mode marker: 2 = point (1 would be "model present")
            float(speed_mps),
            float(nozzle_n) if nozzle_n is not None else math.nan,
            float(nozzle_e) if nozzle_e is not None else math.nan,
            float(pu.target_index) if pu is not None else math.nan,
            math.nan,
            1.0 if geometry_desired else 0.0,
            math.nan,
            float("inf"),
            1.0 if geometry_desired else 0.0,
            1.0 if safety_ok else 0.0,
            1.0 if desired else 0.0,
        ]
        return SprayDecision(
            desired=desired,
            geometry_desired=geometry_desired,
            safety_ok=safety_ok,
            safety_reason=safety_reason,
            projection=None,
            next_boundary=None,
            distance_to_boundary_m=float("inf"),
            event="",
            debug=debug,
            point_update=pu,
        )

    if model is not None and nozzle_n is not None and nozzle_e is not None:
        projection = _project_onto_path(model, nozzle_n, nozzle_e)
    if projection is not None:
        boundary = _next_boundary(model, projection.s, projection.current_flag)
        geometry_desired = projection.current_flag
        if projection.xtrack_error_m > max_xtrack_error_m:
            safety_ok = False
            safety_reason = (
                f"xtrack error {projection.xtrack_error_m:.3f}m "
                f"> {max_xtrack_error_m:.3f}m"
            )
        if mode == "dash" and dash_meter is not None:
            # Dash metering (plan §7.2): geometry_desired comes from cumulative
            # arc-length, not the static MARK/transit flag. Boundaries are
            # dynamic, so there is no single "next boundary" — null it (status
            # then reports distance_to_boundary_m=None per §6). The same
            # solenoid lead as continuous is applied inside the meter so the
            # physical toggle lands on the ideal grid. Corner deferral is
            # handled upstream by the pivot-state gate (plan §5), so the meter
            # keeps integrating through a stop and the phase never drifts.
            on_lead = speed_mps * solenoid_open_delay_s + on_overspray_margin_m
            off_lead = max(
                0.0, speed_mps * solenoid_close_delay_s - off_overspray_margin_m
            )
            xtrack_ok = projection.xtrack_error_m <= max_xtrack_error_m
            du = dash_meter.update(
                projection.s, speed_mps, dt_s, xtrack_ok, on_lead, off_lead
            )
            geometry_desired = du.geometry_desired
            boundary = None
            distance_to_boundary = float("inf")
        elif boundary is not None:
            distance_to_boundary = boundary.s - projection.s
            # ON is intentionally early by solenoid delay plus overspray
            # margin. OFF is early only by close delay; an explicit OFF
            # overspray margin delays shutoff so the MARK tail is not cut short.
            on_lead = speed_mps * solenoid_open_delay_s + on_overspray_margin_m
            off_lead = max(
                0.0,
                speed_mps * solenoid_close_delay_s - off_overspray_margin_m,
            )
            if (
                not projection.current_flag
                and boundary.kind == TRANSIT_TO_MARK
                and distance_to_boundary <= on_lead
            ):
                geometry_desired = True
                event = "on_early"
            elif (
                projection.current_flag
                and boundary.kind == MARK_TO_TRANSIT
                and distance_to_boundary <= off_lead
            ):
                geometry_desired = False
                event = "off_early"

    desired = bool(geometry_desired and safety_ok)
    debug = [
        1.0 if model is not None else 0.0,
        float(speed_mps),
        float(nozzle_n) if nozzle_n is not None else math.nan,
        float(nozzle_e) if nozzle_e is not None else math.nan,
        projection.s if projection is not None else math.nan,
        projection.xtrack_error_m if projection is not None else math.nan,
        1.0 if projection is not None and projection.current_flag else 0.0,
        boundary.s if boundary is not None else math.nan,
        distance_to_boundary,
        1.0 if geometry_desired else 0.0,
        1.0 if safety_ok else 0.0,
        1.0 if desired else 0.0,
    ]
    return SprayDecision(
        desired=desired,
        geometry_desired=geometry_desired,
        safety_ok=safety_ok,
        safety_reason=safety_reason,
        projection=projection,
        next_boundary=boundary,
        distance_to_boundary_m=distance_to_boundary,
        event=event,
        debug=debug,
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
        # ── Phase E: speed-proportional flow (plan §7.5) — default OFF ───────
        # OFF preserves today's behaviour exactly (every ON commands on_value =
        # full flow). When enabled (mavlink_actuator backend only), the ON
        # command value scales with ground speed between min_flow_value and
        # on_value so paint-per-metre stays constant. Requires the §7.5 bench
        # calibration first — set rated at the TOP of the mission speed range.
        self.declare_parameter("flow_modulation_enabled", False)
        self.declare_parameter("min_flow_value", 0.2)             # normalized floor
        self.declare_parameter("rated_marking_speed_mps", 0.35)   # full flow at/above this
        self.declare_parameter("max_flow_slew_per_s", 2.0)        # pump ramp cap (/s)
        # Point-mode dots spray at a standstill (speed 0), so they are NOT
        # speed-scaled — they use this fixed, separately-calibrated value.
        self.declare_parameter("point_dwell_flow_value", 1.0)
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
        # DEPRECATED as an on/off gate (2026-07-17). Still declared because the
        # server's param serializer sends it and an undeclared param is a hard
        # load rejection. It no longer decides whether to spray: speed governs
        # HOW MUCH (flow, upcoming phase), never WHETHER.
        #
        # Why it was removed: a bare `speed < 0.05` test collided with the
        # frozen RPP corner speeds (brake cap 0.08, min corner 0.08, endpoint
        # approach 0.03). The rover deliberately crawls across that threshold,
        # so the gate dithered. In the 2026-07-17 bags this produced 157 valve
        # transitions where the path geometry asked for 25 -- 132 spurious
        # fires, all of them explained by this one comparison. It also made
        # endpoint approach (0.03) structurally unsprayable.
        self.declare_parameter("min_spray_speed_mps", 0.05)
        # Replacement for the speed gate: suppress spray only while the rover
        # is pivoting in place, which is the actual thing we needed to avoid
        # (a stationary nozzle sweeping an arc puddles paint). Sourced from the
        # RPP's own state machine rather than inferred from a speed threshold --
        # a discrete state cannot dither the way a threshold does.
        self.declare_parameter("spray_off_during_pivot", True)
        self.declare_parameter("segment_state_timeout_s", 1.0)
        self.declare_parameter("max_xtrack_error_m", 0.10)
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter("velocity_timeout_s", 0.5)
        # ── Phase B: RTK / GPS fix-quality gate (plan §7.6) ──────────────────
        # Master enable. True demands a healthy RTK fix before AUTO spray;
        # set False for SITL / bench runs with no RTK (mirrors the RPP node's
        # `require_rtk_fix`). Manual /spray/test is unaffected — it uses the
        # armed-only gate, not this one.
        self.declare_parameter("spray_require_rtk_fix", True)
        # Minimum GPSRAW.fix_type to spray. Default 6 = RTK_FIXED, the same bar
        # the driving controller uses. Lower to 5 for RTK_FLOAT sites.
        self.declare_parameter("spray_min_fix_type", 6)
        # A missing GPSRAW is a FAIL ("gps stale"), never "fix ok, just quiet".
        # Looser than the 0.5 s pose/velocity gates because GPSRAW is slower.
        self.declare_parameter("gps_fix_timeout_s", 2.0)
        # Asymmetric hysteresis: drop is instant (unsafe edge, no debounce);
        # re-enable only after fix has been continuously good this long.
        self.declare_parameter("gps_recover_hold_s", 1.0)
        # ── Phase D: point/dwell mode (plan §7.3) ────────────────────────────
        # A dot counts as "arrived" only at/below this speed (a dwell sprays at
        # a standstill). Per-point tolerances/settle/dwell come from the
        # session config, not params.
        self.declare_parameter("point_arrival_max_speed_mps", 0.05)
        # A point unreachable this long is skipped (logged, never sprayed) so a
        # single bad coordinate can't wedge the whole mission.
        self.declare_parameter("point_arrival_timeout_s", 60.0)
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
        # endpoints. Default True so the node works standalone without the
        # server; the server starts with disabled state and sets False on boot.
        self.declare_parameter("spray_enabled", True)

        self._group = ReentrantCallbackGroup()
        self._desired_raw = False
        self._candidate: Optional[bool] = None
        self._candidate_count = 0
        self._desired_debounced = False
        self._last_active_time = None
        self._legacy_active_raw = False
        self._manual_active = False
        self._manual_deadline_ns: Optional[int] = None
        self._armed = False
        self._mode = "UNKNOWN"
        self._service_ready = False
        # Actuator command state machine (Spray Controller V2 §4). Replaces
        # the scattered _commanded/_off_confirmed/_cmd_seq booleans and the
        # old flat-500ms retry throttle in _maybe_retry_off/_force_off — the
        # FSM now owns cmd_seq, RECOVERY backoff, and retry. State starts
        # OFF_UNCONFIRMED (actuator's real state unknown at boot — a
        # previous instance may have left the output ON); the FSM will not
        # accept an ON until a confirmed OFF ack lands (see the startup
        # drive at the end of __init__).
        self._fsm = SpraySafetyStateMachine()
        self._path_model: Optional[SprayPathModel] = None
        # Internal SpraySessionConfig representation (plan §3), kept
        # alongside _path_model. /path stays the geometry/path-model source
        # for ALL modes (continuous mode is field-validated off it and is left
        # byte-identical); /spray/session_config (B0) layers the operator's
        # MODE + mode params on top. The server publishes both from the same
        # staged mission, so their geometry agrees by construction.
        self._session_config: SpraySessionConfig = cleared_config()
        self._config_fingerprint: str = self._session_config.path_fingerprint()
        # B0 / Phase C mode state. Default "continuous" so a node that never
        # receives a session_config behaves exactly as it did before B0.
        self._session_mode: str = "continuous"
        self._dash_meter: Optional[DashMeter] = None
        # Phase D point mode. _last_point_update lets _auto_safety_status apply
        # the pivot-gate exemption using the most recent point FSM state (it
        # runs one step before _make_spray_decision updates the meter).
        self._point_meter: Optional[PointMeter] = None
        self._last_point_update = None
        # Measured tick interval for arc-length jump-tolerance (§7.2). Seeded
        # on the first dash tick; monotonic clock, never wall time.
        self._last_tick_monotonic: Optional[float] = None
        # Phase E speed-proportional flow. Modulator is built lazily on the
        # disabled→enabled edge (picks up fresh calibration params); the
        # commanded value + source feed the ON command and telemetry.
        self._flow_modulator: Optional[FlowModulator] = None
        self._prev_fsm_commanded: bool = False
        self._commanded_flow_value: Optional[float] = None
        self._flow_source: str = "n/a"
        # Phase B RTK gate state (§7.6). fix_type 0 = no fix; recv_time None
        # until the first GPSRAW; recover_since is set the moment fix goes good
        # and reset to None on any bad/stale sample (asymmetric hysteresis).
        self._gps_fix_type: int = 0
        self._gps_recv_time = None
        self._gps_recover_since = None
        self._last_decision: Optional[SprayDecision] = None
        self._pose_ned: Optional[tuple[float, float, float]] = None
        self._pose_recv_time = None
        self._vel_ned = (0.0, 0.0)
        self._vel_recv_time = None
        self._segment_state: Optional[int] = None
        self._segment_state_recv_time = None
        self._last_auto_source = ""
        self._last_distance_event = ""
        self._last_safety_block_reason = ""
        self._pose_stale_logged = False
        self._velocity_stale_logged = False

        command_service = str(self.get_parameter("command_service").value)
        self._command_cli = self.create_client(
            CommandLong,
            command_service,
            callback_group=self._group,
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
        # NEW (Phase A telemetry foundation, plan §6): typed JSON status
        # snapshot. Additive only — none of the five publishers above are
        # removed or renamed.
        self._status_pub = self.create_publisher(
            String, "/spray/status", _best_effort_qos()
        )
        self.create_subscription(
            Bool,
            "/spray/active",
            self._active_cb,
            _best_effort_qos(),
            callback_group=self._group,
        )
        self.create_subscription(
            Path,
            "/path",
            self._path_cb,
            _path_qos(),
            callback_group=self._group,
        )
        # B0 (plan §3): the mission-config transport. RELIABLE + TRANSIENT_LOCAL
        # (same durability class as /path) so a late-joining / restarted spray
        # node re-latches the current mission's mode automatically. The node is
        # the ONLY parser of this schema (defect #2/#3 avoidance) and never
        # trusts a caller-supplied fingerprint. Absent this message the node
        # stays in its default continuous behaviour — fully backward-compatible
        # with a mission that publishes only /path.
        self.create_subscription(
            String,
            "/spray/session_config",
            self._session_config_cb,
            _path_qos(),
            callback_group=self._group,
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose_cb,
            _best_effort_qos(),
            callback_group=self._group,
        )
        self.create_subscription(
            TwistStamped,
            "/mavros/local_position/velocity_local",
            self._vel_cb,
            _best_effort_qos(),
            callback_group=self._group,
        )
        # Read-only: the RPP's segment state, used solely to know when the rover
        # is pivoting in place. We never command the controller and never read
        # its tuning -- the frozen RPP baseline is untouched by this.
        self.create_subscription(
            Float32MultiArray,
            "/rpp/segment_debug",
            self._segment_debug_cb,
            _best_effort_qos(),
            callback_group=self._group,
        )
        # Phase B (§7.6): RTK fix quality. Same topic/source as the RPP node's
        # P0.3 gate — read-only, spray-scoped.
        self.create_subscription(
            GPSRAW,
            "/mavros/gpsstatus/gps1/raw",
            self._gps_cb,
            _best_effort_qos(),
            callback_group=self._group,
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
            callback_group=self._group,
        )
        self.create_subscription(
            State,
            "/mavros/state",
            self._state_cb,
            _state_qos(),
            callback_group=self._group,
        )

        self._watchdog_timer = self.create_timer(0.02, self._watchdog_tick)
        reassert_hz = max(0.0, float(self.get_parameter("reassert_hz").value))
        self._reassert_timer = None
        if reassert_hz > 0.0:
            self._reassert_timer = self.create_timer(1.0 / reassert_hz, self._reassert_tick)

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

        self._publish_actuator_state()
        self.get_logger().info("spray_controller started")
        # Proactively drive the actuator OFF on startup through the FSM
        # (OFF_UNCONFIRMED -> dispatch OFF -> OFF_CONFIRMED once acked). If
        # the service is not yet ready, _dispatch_command treats that as an
        # immediate ack-failure, which the FSM routes into RECOVERY;
        # _service_probe_tick calls _drive_fsm_tick again once the service
        # appears, which retries the still-pending OFF.
        self._drive_fsm_tick("startup")

    def _service_probe_tick(self) -> None:
        if self._service_ready:
            return
        if self._command_cli.service_is_ready():
            self._service_ready = True
            self.get_logger().info("spray command service is ready")
            self._drive_fsm_tick("service ready startup OFF")

    def _state_cb(self, msg: State) -> None:
        was_safe = self._safety_allows_on()
        self._armed = bool(msg.armed)
        self._mode = str(msg.mode)
        now_safe = self._safety_allows_on()
        if was_safe and not now_safe:
            # Fail-safes outrank the manual override — clear it so spray
            # cannot resume ON without a fresh, safety-gated manual command.
            self._manual_active = False
            self._manual_deadline_ns = None
        self._drive_fsm_tick("state changed")

    def _active_cb(self, msg: Bool) -> None:
        self._last_active_time = self.get_clock().now()
        self._legacy_active_raw = bool(msg.data)
        if (
            not bool(self.get_parameter("use_distance_aware_spray").value)
            and bool(self.get_parameter("allow_legacy_spray_active_fallback").value)
        ):
            self._set_auto_desired(self._legacy_active_raw, source="legacy")

    def _path_cb(self, msg: Path) -> None:
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        # position.z is a bitfield: bit0 = spray ON, bit1 = must-hit vertex.
        # MUST bit-test, not `> 0.5`: a spray-OFF must-hit point encodes as 2.0
        # and a `> 0.5` test would spray a transit leg.
        flags = [bool(int(round(p.pose.position.z)) & 1) for p in msg.poses]
        if not points:
            self._path_model = None
            self._session_config = cleared_config()
            self._config_fingerprint = self._session_config.path_fingerprint()
            self._fsm.note_event_reset(time.monotonic())
            self._set_auto_desired(False, source="distance")
            self.get_logger().warn("spray path cleared: received empty /path")
            return
        try:
            self._path_model = _build_path_model(points, flags)
        except ValueError as exc:
            self._path_model = None
            self._session_config = cleared_config()
            self._config_fingerprint = self._session_config.path_fingerprint()
            self._set_auto_desired(False, source="distance")
            self.get_logger().warn(f"spray path rejected: {exc}")
            return
        # Internal SpraySessionConfig mirror of the same geometry (plan §3).
        self._session_config = continuous_config_from_path(points, flags)
        self._config_fingerprint = self._session_config.path_fingerprint()
        # A new mission/config load resets the FSM's RECOVERY backoff so a
        # fresh mission gets a fast first retry rather than inheriting a
        # stale backoff window left over from whatever happened on the
        # previous one (plan §4's backoff-reset rule).
        self._fsm.note_event_reset(time.monotonic())
        self.get_logger().info(
            f"spray path loaded: {len(points)} points, "
            f"{len(self._path_model.boundaries)} boundaries"
        )

    def _session_config_cb(self, msg: String) -> None:
        """B0 — receive the operator-selected mode + mode params (plan §3).

        Fail static: any malformed or schema-mismatched config is logged and
        the last-known-good mode is kept — never fail open into a wrong mode.
        Geometry is NOT taken from here (it stays on /path); this selects the
        mode and, for dash, builds the arc-length meter.
        """
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError) as exc:
            self.get_logger().warn(
                f"spray session_config: bad JSON, keeping mode={self._session_mode}: {exc}"
            )
            return
        try:
            cfg = parse_session_config(data)
        except ConfigSchemaError as exc:
            self.get_logger().warn(
                f"spray session_config rejected ({exc}); keeping mode={self._session_mode}"
            )
            return

        prev_mode = self._session_mode
        self._session_mode = cfg.mode
        self._session_config = cfg

        self._dash_meter = None
        self._point_meter = None
        self._last_point_update = None
        if cfg.mode == "dash" and cfg.dash is not None:
            try:
                self._dash_meter = DashMeter(
                    cfg.dash.on_distance_m,
                    cfg.dash.off_distance_m,
                    cfg.dash.start_state,
                )
            except ValueError as exc:
                self.get_logger().warn(
                    f"dash config invalid ({exc}); reverting to continuous"
                )
                self._session_mode = "continuous"
        elif cfg.mode == "point" and cfg.points_mode is not None:
            pm = cfg.points_mode
            try:
                self._point_meter = PointMeter(
                    pm.coordinates,
                    pm.arrival_tolerance_m,
                    pm.arrival_settle_s,
                    pm.dwell_s,
                    pm.heading_tolerance_deg,
                    point_arrival_max_speed_mps=max(
                        0.0, float(self.get_parameter("point_arrival_max_speed_mps").value)
                    ),
                    point_arrival_timeout_s=max(
                        0.0, float(self.get_parameter("point_arrival_timeout_s").value)
                    ),
                )
            except ValueError as exc:
                self.get_logger().warn(
                    f"point config invalid ({exc}); reverting to continuous"
                )
                self._session_mode = "continuous"

        self._last_tick_monotonic = None  # reseed dt on the next dash tick
        # A mode/config change resets the FSM RECOVERY backoff (plan §4).
        self._fsm.note_event_reset(time.monotonic())
        if cfg.mode != prev_mode:
            extra = ""
            if cfg.mode == "dash" and self._dash_meter is not None:
                extra = (
                    f" (on={self._dash_meter.on_distance_m}m "
                    f"off={self._dash_meter.off_distance_m}m "
                    f"start={self._dash_meter.phase})"
                )
            self.get_logger().info(f"spray mode: {prev_mode} -> {self._session_mode}{extra}")

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
        self._drive_fsm_tick("manual override changed")
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
        else:
            self._candidate_count += 1

        debounce_samples = max(0, int(self.get_parameter("debounce_samples").value))
        if self._candidate_count >= max(1, debounce_samples):
            self._desired_debounced = bool(self._candidate)

        # Centralized FSM drive point for the auto/legacy desire pipeline —
        # this runs every distance-aware tick (the node's control-loop
        # cadence), so the FSM always sees a fresh (desired, safety_ok,
        # enabled) tuple. Manual overrides and FCU state changes drive the
        # FSM separately (see _manual_cb / _state_cb) since they bypass this
        # debounce pipeline entirely.
        self._drive_fsm_tick("debounce")

    def _watchdog_tick(self) -> None:
        # Manual override hard expiry — never latches, independent of /spray/active.
        if self._manual_active and self._manual_deadline_ns is not None:
            if self.get_clock().now().nanoseconds >= self._manual_deadline_ns:
                self._manual_active = False
                self._manual_deadline_ns = None
                self.get_logger().info("manual spray override expired — reverting")
                self._drive_fsm_tick("manual expired")

        # Each mode branch drives the FSM exactly once per tick (~50 Hz), so
        # safety/enable state is enforced every tick in EVERY mode:
        #   - distance-aware: via _distance_aware_tick -> _apply_debounce
        #   - disabled:       via _set_auto_desired    -> _apply_debounce
        #   - legacy:         drives unconditionally at the end of its tick
        # (this replaced an earlier unconditional drive here that
        # double-drove — and double-published /spray/status — in the default
        # distance-aware mode).
        if bool(self.get_parameter("use_distance_aware_spray").value):
            self._distance_aware_tick()
        elif bool(self.get_parameter("allow_legacy_spray_active_fallback").value):
            self._legacy_active_watchdog_tick()
        else:
            self._set_auto_desired(False, source="disabled")

        self._publish_manual_state()

    def _legacy_active_watchdog_tick(self) -> None:
        timeout_s = max(0.0, float(self.get_parameter("active_timeout_s").value))
        reason = "legacy watchdog"
        if self._last_active_time is not None:
            age_s = (self.get_clock().now() - self._last_active_time).nanoseconds * 1e-9
            if age_s > timeout_s and not self._manual_active:
                # Staleness kills the *auto* desire only; an active manual
                # override has its own timeout and does not depend on RPP.
                self._desired_raw = False
                self._desired_debounced = False
                self._candidate = False
                self._candidate_count = 0
                reason = f"/spray/active stale ({age_s:.2f}s)"
        # Drive the FSM every tick (not only when stale): this is the
        # legacy mode's single per-tick FSM drive, so a spray_enabled=False
        # disable or any safety loss is enforced within one watchdog tick.
        self._drive_fsm_tick(reason)

    def _distance_aware_tick(self) -> None:
        model = self._path_model
        pose_fresh, pose_age_s = self._pose_is_fresh()
        velocity_fresh, velocity_age_s = self._velocity_is_fresh()
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

        nozzle_n: Optional[float] = None
        nozzle_e: Optional[float] = None
        if pose is not None:
            nozzle_n, nozzle_e = _nozzle_position_ned(
                pose[0],
                pose[1],
                pose[2],
                float(self.get_parameter("nozzle_forward_offset_m").value),
                float(self.get_parameter("nozzle_lateral_offset_m").value),
            )

        safety_ok, safety_reason = self._auto_safety_status(
            pose_fresh,
            speed,
            velocity_fresh=velocity_fresh,
        )
        # Measured tick interval for dash arc-length jump-tolerance (§7.2).
        # Monotonic clock; the first tick after a mode change seeds it (dt=0),
        # which the meter treats as "no travel" — safe (no toggle that tick).
        now_mono = time.monotonic()
        dt_s = 0.0
        if self._last_tick_monotonic is not None:
            dt_s = max(0.0, now_mono - self._last_tick_monotonic)
        self._last_tick_monotonic = now_mono
        # Phase E: recompute the speed-proportional flow value for this tick
        # (no-op / full-flow when disabled). Feeds the ON command + telemetry.
        self._update_flow(speed, dt_s)
        decision = _make_spray_decision(
            model=model,
            nozzle_n=nozzle_n,
            nozzle_e=nozzle_e,
            speed_mps=speed,
            safety_ok=safety_ok,
            safety_reason=safety_reason,
            mode=self._session_mode,
            dash_meter=self._dash_meter,
            dt_s=dt_s,
            point_meter=self._point_meter,
            yaw=pose[2] if pose is not None else 0.0,
            now_s=now_mono,
            off_confirmed=(self._fsm.state == SprayState.OFF_CONFIRMED),
            solenoid_open_delay_s=max(
                0.0,
                float(self.get_parameter("solenoid_open_delay_s").value),
            ),
            solenoid_close_delay_s=max(
                0.0,
                float(self.get_parameter("solenoid_close_delay_s").value),
            ),
            on_overspray_margin_m=max(
                0.0,
                float(self.get_parameter("on_overspray_margin_m").value),
            ),
            off_overspray_margin_m=max(
                0.0,
                float(self.get_parameter("off_overspray_margin_m").value),
            ),
            max_xtrack_error_m=max(
                0.0,
                float(self.get_parameter("max_xtrack_error_m").value),
            ),
        )
        # Feeds _fsm_safety_ok() so the FSM's safety_ok input reflects the
        # full distance-aware gate stack (armed/offboard/path/pose/vel/speed
        # from _auto_safety_status, plus the xtrack gate folded in above).
        self._last_decision = decision
        # Phase D: keep the latest point update for the pivot-gate exemption
        # (read next tick by _auto_safety_status) and surface skips/completion.
        if decision.point_update is not None:
            pu = decision.point_update
            if pu.skipped_index >= 0:
                self.get_logger().warn(
                    f"point mode: target {pu.skipped_index} unreachable — "
                    f"skipped (not sprayed)"
                )
            if pu.done and (
                self._last_point_update is None or not self._last_point_update.done
            ):
                self.get_logger().info("point mode: all targets complete")
            self._last_point_update = pu
        self._publish_debug(decision.debug)

        if decision.event and decision.event != self._last_distance_event:
            if decision.event == "on_early":
                self.get_logger().info("Spray ON early before MARK start")
            elif decision.event == "off_early":
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

        self._set_auto_desired(decision.desired, source="distance")

    def _pose_is_fresh(self) -> tuple[bool, float]:
        if self._pose_recv_time is None:
            return False, float("inf")
        age_s = (self.get_clock().now() - self._pose_recv_time).nanoseconds * 1e-9
        timeout_s = max(0.0, float(self.get_parameter("pose_timeout_s").value))
        return age_s <= timeout_s, age_s

    def _velocity_is_fresh(self) -> tuple[bool, float]:
        if self._vel_recv_time is None:
            return False, float("inf")
        age_s = (self.get_clock().now() - self._vel_recv_time).nanoseconds * 1e-9
        timeout_s = max(0.0, float(self.get_parameter("velocity_timeout_s").value))
        return age_s <= timeout_s, age_s

    def _segment_debug_cb(self, msg: Float32MultiArray) -> None:
        """Track the RPP segment state. data[1] is SegmentStateCode."""
        if len(msg.data) < 2:
            return
        self._segment_state = int(msg.data[1])
        self._segment_state_recv_time = self.get_clock().now()

    def _gps_cb(self, msg: GPSRAW) -> None:
        """Track GPS fix quality for the Phase B RTK gate (§7.6)."""
        prev = self._gps_fix_type
        self._gps_fix_type = int(msg.fix_type)
        self._gps_recv_time = self.get_clock().now()
        if prev != self._gps_fix_type:
            self.get_logger().info(
                f"spray GPS fix: {_GPS_FIX_NAMES.get(prev, '?')} -> "
                f"{_GPS_FIX_NAMES.get(self._gps_fix_type, '?')} "
                f"(fix_type={self._gps_fix_type})"
            )

    def _gps_health(self) -> tuple[bool, bool, str]:
        """Pure read of GPS health: (fresh, fix_ok, name). No state mutation.

        Used both by the gate and by status telemetry, so it must not touch the
        recover-hold timer. `fix_ok` is the instantaneous "fix_type >= min AND
        fresh" check; the recover-hold delay lives only in _gps_gate().
        """
        name = _GPS_FIX_NAMES.get(self._gps_fix_type, f"fix_{self._gps_fix_type}")
        if self._gps_recv_time is None:
            return False, False, "no_data"
        age_s = (self.get_clock().now() - self._gps_recv_time).nanoseconds * 1e-9
        timeout_s = max(0.0, float(self.get_parameter("gps_fix_timeout_s").value))
        if age_s > timeout_s:
            return False, False, name
        min_fix = int(self.get_parameter("spray_min_fix_type").value)
        return True, self._gps_fix_type >= min_fix, name

    def _gps_gate(self) -> tuple[bool, str]:
        """RTK gate with asymmetric hysteresis (§7.6). Mutates the recover timer.

        Drop is instant on a below-threshold OR stale sample (unsafe edge, no
        debounce). Re-enable only after fix has been continuously good for
        gps_recover_hold_s — a single good sample after a dropout does not
        re-open spray. Call exactly once per control tick.
        """
        if not bool(self.get_parameter("spray_require_rtk_fix").value):
            return True, ""
        fresh, fix_ok, _name = self._gps_health()
        if not fresh:
            self._gps_recover_since = None  # any stale sample resets recovery
            return False, "gps stale"
        if not fix_ok:
            self._gps_recover_since = None  # any bad fix resets recovery
            min_fix = int(self.get_parameter("spray_min_fix_type").value)
            return False, f"gps fix {self._gps_fix_type} < required {min_fix}"
        # Fix is good this sample — apply the slow re-enable.
        now = self.get_clock().now()
        if self._gps_recover_since is None:
            self._gps_recover_since = now
        hold_s = max(0.0, float(self.get_parameter("gps_recover_hold_s").value))
        held_s = (now - self._gps_recover_since).nanoseconds * 1e-9
        if held_s < hold_s:
            return False, f"gps recovering ({held_s:.1f}/{hold_s:.1f}s)"
        return True, ""

    def _pivot_is_active(self) -> bool:
        """True only while the RPP is pivoting the rover in place.

        Gated on CORNER_ALIGN alone, deliberately NOT on CORNER_STOP. Read the
        SegmentStateCode comment in rpp_controller_node: CORNER_STOP means
        "commanding zero, waiting for the rover to physically stop" -- the rover
        is still coasting through it and still laying the last ~2 cm of the leg
        (measured across the 2026-07-17 bags: 11.8 cm over 6 corners). Cutting
        spray there would trade the chatter bug for a 2 cm gap at every corner.
        CORNER_ALIGN is entered only once the RPP has CONFIRMED zero motion, so
        it is both the correct moment and an already-debounced one.

        Absent/stale state is treated as "not pivoting" (permissive): geometry
        keeps its authority, which preserves behaviour against an RPP that does
        not publish this topic. A missed pivot costs a small puddle; wrongly
        asserting a pivot would silently kill spray for a whole run.
        """
        if not bool(self.get_parameter("spray_off_during_pivot").value):
            return False
        if self._segment_state is None or self._segment_state_recv_time is None:
            return False
        timeout_s = max(0.0, float(self.get_parameter("segment_state_timeout_s").value))
        age_s = (
            self.get_clock().now() - self._segment_state_recv_time
        ).nanoseconds * 1e-9
        if age_s > timeout_s:
            return False
        return self._segment_state == _SEGMENT_STATE_CORNER_ALIGN

    def _auto_safety_status(
        self,
        pose_fresh: bool,
        speed: float,
        velocity_fresh: bool = True,
    ) -> tuple[bool, str]:
        if not self._armed:
            return False, "disarmed"
        require_offboard = bool(self.get_parameter("require_offboard").value)
        if require_offboard and self._mode != "OFFBOARD":
            return False, "not OFFBOARD"
        # Mode-appropriate config gate (§5): continuous/dash need the path model
        # (geometry rides /path); point needs the coordinate list (its own).
        if self._session_mode == "point":
            if self._point_meter is None:
                return False, "point config not loaded"
        elif self._path_model is None:
            return False, "path not loaded"
        if not pose_fresh:
            return False, "pose stale"
        if not velocity_fresh:
            return False, "velocity stale"
        # Phase B (§7.6): RTK fix-quality gate. Fail fast on a GPS dropout —
        # ordered before the pivot check so an RTK loss surfaces as the reason,
        # not "pivoting". No-op when spray_require_rtk_fix is False (SITL/bench).
        gps_ok, gps_reason = self._gps_gate()
        if not gps_ok:
            return False, gps_reason
        # NOTE: `speed` is intentionally NOT compared against a minimum here.
        # See min_spray_speed_mps's declaration for why the old gate was removed.
        # Spraying is a question of WHERE the nozzle is, not how fast it is
        # moving; slow means thin (flow control), never off.
        if self._pivot_is_active():
            # Point-mode pivot-gate exemption (Rev 4 §7.3): a dwell sprays at a
            # standstill, which the pivot gate would otherwise suppress. Exempt
            # ONLY when point mode is dwelling within tolerance of the active
            # target — never mode-wide, so transits between dots keep the gate.
            # Uses last tick's point state (this runs before the meter updates).
            if not (
                self._session_mode == "point"
                and self._last_point_update is not None
                and self._last_point_update.exempt_pivot
            ):
                return False, "pivoting in place"
        return True, ""

    def _safety_allows_on(self) -> bool:
        """Coarse gate: is a *manual* ON request currently honorable at all.

        Kept as its own helper (unchanged from pre-V2) — used by _manual_cb
        to decide whether to accept a manual ON, and by _reassert_on_command
        to gate the periodic ON heartbeat. This is deliberately not the same
        thing as the FSM's `safety_ok` input (see _fsm_safety_ok): this gate
        includes `spray_enabled`, which the FSM instead receives as its own
        separate `enabled` input so a disable event is modeled distinctly
        (-> DISABLED) from a safety-loss event (-> forced OFF + RECOVERY).
        """
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

    def _fsm_safety_ok(self) -> tuple[bool, str]:
        """Safety input fed to SpraySafetyStateMachine.tick().

        Deliberately excludes spray_enabled — that is the FSM's separate
        `enabled` input (see _drive_fsm_tick), so a disable event is modeled
        distinctly from a safety-loss event per plan §4. For a manual
        override this is the manual gate (armed only, no OFFBOARD
        requirement); for auto/distance-aware mode this is the full gate
        stack computed by the latest _distance_aware_tick() (armed,
        offboard, path loaded, pose/velocity freshness, min speed, xtrack);
        for the legacy /spray/active fallback it is armed + offboard.
        """
        if not self._armed:
            return False, "disarmed"
        if self._manual_active:
            return True, ""
        if bool(self.get_parameter("use_distance_aware_spray").value):
            if self._last_decision is not None:
                return self._last_decision.safety_ok, self._last_decision.safety_reason
            return False, "distance-aware safety not yet evaluated"
        require_offboard = bool(self.get_parameter("require_offboard").value)
        if require_offboard and self._mode != "OFFBOARD":
            return False, "not OFFBOARD"
        return True, ""

    def _drive_fsm_tick(self, reason: str) -> None:
        """Single entry point that advances the actuator FSM by one tick and
        dispatches whatever command it returns (plan §4/§10 — the node's
        control-tick loop). Also publishes the current actuator/desired/
        status telemetry so every drive point stays consistent.
        """
        desired = self._effective_desired()
        safety_ok, safety_reason = self._fsm_safety_ok()
        enabled = bool(self.get_parameter("spray_enabled").value)
        cmd = self._fsm.tick(
            desired=desired, safety_ok=safety_ok, enabled=enabled, now=time.monotonic()
        )
        if cmd is not None:
            self._dispatch_command(cmd, reason)
        self._publish_actuator_state()
        self._publish_desired_state(desired)
        self._publish_status(desired, safety_ok, safety_reason)

    def _dispatch_command(self, cmd: Optional[SprayCommand], reason: str) -> None:
        """Send an FSM-issued SprayCommand to MAVROS.

        Loops through any immediate follow-up the FSM emits: if the command
        service is not ready, that is treated as an instant ack-failure
        (fed back into the FSM via on_ack), which for an ON dispatch
        produces a follow-up OFF per the FSM's own transition table, and
        for an OFF dispatch enters RECOVERY (returns None, ending the loop).
        """
        while cmd is not None:
            if not self._service_ready:
                self.get_logger().warn(
                    "spray command service not ready; command suppressed",
                    throttle_duration_sec=1.0,
                )
                cmd = self._fsm.on_ack(cmd.seq, False, now=time.monotonic())
                continue
            seq = cmd.seq
            on = cmd.on
            req = self._build_command_request(on)
            future = self._command_cli.call_async(req)
            future.add_done_callback(
                lambda fut, s=seq, o=on, why=reason: self._command_done(fut, s, o, why)
            )
            cmd = None

    def _update_flow(self, speed_mps: float, dt_s: float) -> None:
        """Phase E (§7.5): recompute the commanded flow value for this tick.

        No-op unless flow_modulation_enabled AND the mavlink_actuator backend
        (servo backend keeps full on_pwm_us — documented limitation). Manual
        bench ON is not modulated (full flow expected). The modulator is
        (re)built on the disabled→enabled edge so a fresh calibration takes
        effect; it can only ever move the command within [min_flow, on_value].
        """
        commanded = self._fsm.commanded
        rising = commanded and not self._prev_fsm_commanded
        self._prev_fsm_commanded = commanded

        enabled = (
            bool(self.get_parameter("flow_modulation_enabled").value)
            and str(self.get_parameter("actuator_backend").value) == "mavlink_actuator"
            and not self._manual_active
        )
        if not enabled:
            self._flow_modulator = None
            self._commanded_flow_value = None
            self._flow_source = "n/a"
            return
        if self._flow_modulator is None:
            # Build from current (freshly-calibrated) params on the enable edge.
            self._flow_modulator = FlowModulator(
                min_flow_value=float(self.get_parameter("min_flow_value").value),
                on_value=float(self.get_parameter("on_value").value),
                rated_marking_speed_mps=float(
                    self.get_parameter("rated_marking_speed_mps").value
                ),
                max_slew_per_s=float(self.get_parameter("max_flow_slew_per_s").value),
            )
            rising = True  # seed the slew filter to the floor on first build
        if rising:
            self._flow_modulator.reset()
        if not commanded:
            self._commanded_flow_value = None
            self._flow_source = "n/a"
            return
        if self._session_mode == "point":
            # A dot sprays at a standstill — fixed, separately-calibrated flow.
            self._commanded_flow_value = float(
                self.get_parameter("point_dwell_flow_value").value
            )
            self._flow_source = "point_fixed"
        else:
            self._commanded_flow_value = self._flow_modulator.update(speed_mps, dt_s)
            self._flow_source = "speed_scaled"

    def _current_on_value(self) -> float:
        """The actuator ON value to command now — modulated flow if Phase E is
        active this tick, else the full on_value (today's behaviour)."""
        if self._commanded_flow_value is not None:
            return self._commanded_flow_value
        return float(self.get_parameter("on_value").value)

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
        value = (
            self._current_on_value()  # Phase E: modulated flow when active, else on_value
            if on else
            float(self.get_parameter("off_value").value)
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
            pwm = int(self.get_parameter("on_pwm_us").value)
            pwm = max(0, min(pwm, _SERVO_PWM_MAX_US))
        else:
            pwm = int(self.get_parameter("off_pwm_us").value)
        self.get_logger().info(
            f"Sending spray {'ON' if on else 'OFF'} PWM {pwm}µs (instance={instance})",
            throttle_duration_sec=1.0,
        )
        req.command = MAV_CMD_DO_SET_SERVO
        req.param1 = float(instance)
        req.param2 = float(pwm)
        req.param3 = req.param4 = req.param5 = req.param6 = req.param7 = 0.0
        return req

    def _command_done(self, future, seq: int, on: bool, reason: str) -> None:
        if seq != self._fsm.cmd_seq:
            # A newer command was issued before this result arrived; ignoring
            # it prevents a stale reply from corrupting current spray state.
            # (INVARIANT 2, spray_fsm.py — enforced again here purely to
            # avoid an unnecessary on_ack()/publish round-trip; on_ack()
            # would no-op on a seq mismatch regardless.)
            self.get_logger().debug(
                f"ignoring stale spray command result "
                f"(seq={seq}, latest={self._fsm.cmd_seq}, on={on}, reason={reason})"
            )
            return
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f"spray command ({'ON' if on else 'OFF'}) {reason} raised: {exc}"
            )
            followup = self._fsm.on_ack(seq, False, now=time.monotonic())
            self._publish_actuator_state()
            self._dispatch_command(followup, f"{reason} ack-exception-followup")
            return
        success = bool(getattr(resp, "success", False))
        if not success:
            self.get_logger().warn(
                f"spray command ({'ON' if on else 'OFF'}) {reason} rejected: "
                f"result={getattr(resp, 'result', None)}"
            )
        followup = self._fsm.on_ack(seq, success, now=time.monotonic())
        self._publish_actuator_state()
        self._dispatch_command(followup, f"{reason} ack-followup")

    def _reassert_tick(self) -> None:
        self._drive_fsm_tick("reassert")
        if self._effective_desired() and self._fsm.commanded and self._safety_allows_on():
            self._reassert_on_command()

    def _reassert_on_command(self) -> None:
        """Periodic re-affirmation of an already-(pending-or-)confirmed ON
        command — unchanged from today's reassert_hz behavior. This is a
        wire-level heartbeat re-send of the current ON value; it does NOT
        go through the FSM state machine (no state transition, no cmd_seq
        bump) — a failure here is logged only, matching prior behavior
        where reassert failures had no side effect on commanded/off_confirmed
        state.
        """
        if not self._service_ready:
            return
        seq = self._fsm.cmd_seq
        req = self._build_command_request(True)
        future = self._command_cli.call_async(req)
        future.add_done_callback(lambda fut, s=seq: self._reassert_command_done(fut, s))

    def _reassert_command_done(self, future, seq: int) -> None:
        if seq != self._fsm.cmd_seq:
            return
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warn(f"spray ON reassert failed: {exc}")
            return
        if not bool(getattr(resp, "success", False)):
            self.get_logger().warn(
                f"spray ON reassert rejected: result={getattr(resp, 'result', None)}"
            )

    def _fsm_off_confirmed(self) -> bool:
        """True once the FSM has a confirmed-OFF actuator state (OFF_CONFIRMED
        or the terminal DISABLED, which per spray_fsm.py is only entered once
        OFF is confirmed). Equivalent to the old `_off_confirmed` bool for
        shutdown-flush purposes.
        """
        return self._fsm.state in (SprayState.OFF_CONFIRMED, SprayState.DISABLED)

    def _publish_actuator_state(self) -> None:
        commanded_msg = Bool()
        commanded_msg.data = bool(self._fsm.commanded)
        self._commanded_pub.publish(commanded_msg)
        # INTENDED BEHAVIOR CHANGE (Spray Controller V2 plan §4, defect #6 —
        # "no optimistic-ON"): /spray/state now reflects state == ON_CONFIRMED
        # only. Previously this published True the instant an ON command was
        # dispatched (see the old _send_command, before any MAVROS ack) — the
        # exact "commanded == spraying" conflation the V2 FSM makes
        # structurally impossible (spray_fsm.py INVARIANT 1). This is the
        # ONE intended Mode-1 behavior change in Phase A; everything else in
        # this file is behavior-preserving.
        state_msg = Bool()
        state_msg.data = bool(self._fsm.spraying)
        self._state_pub.publish(state_msg)

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

    def _publish_status(self, desired: bool, safety_ok: bool, safety_reason: str) -> None:
        decision = self._last_decision
        distance_to_boundary_m: Optional[float] = None
        xtrack_error_m: Optional[float] = None
        if decision is not None:
            if decision.next_boundary is not None:
                distance_to_boundary_m = decision.distance_to_boundary_m
            if decision.projection is not None:
                xtrack_error_m = decision.projection.xtrack_error_m
        mode_state: dict = {}
        if self._session_mode == "dash" and self._dash_meter is not None:
            mode_state = self._dash_meter.mode_state()
        elif self._session_mode == "point" and self._point_meter is not None:
            mode_state = self._point_meter.mode_state()
        # Phase E flow telemetry (§7.5): was this line thin because of a gate or
        # because of the flow formula — must be visible, not inferred. None when
        # not spraying / flow modulation off.
        mode_state["commanded_flow_value"] = self._commanded_flow_value
        mode_state["flow_source"] = self._flow_source
        # Phase B (§7.6): report real GPS health. gps_fix_ok is the instantaneous
        # (fresh AND fix>=min); the recover-hold nuance shows up in safety_reason
        # ("gps recovering") when the RTK gate is the blocking cause.
        gps_fresh, gps_fix_ok_raw, gps_fix_name = self._gps_health()
        gps_fix_ok = bool(gps_fresh and gps_fix_ok_raw)
        status = make_status(
            mode=self._session_mode,
            fsm_state=self._fsm.state.value,
            spraying=self._fsm.spraying,
            desired=desired,
            manual_active=self._manual_active,
            safety_ok=safety_ok,
            safety_reason=safety_reason,
            distance_to_boundary_m=distance_to_boundary_m,
            # Phase B (§7.6): real RTK health (computed above). The gate blocks
            # AUTO spray when spray_require_rtk_fix is True and fix is bad/stale;
            # safety_reason carries the specific cause.
            gps_fix_ok=gps_fix_ok,
            gps_fix_name=gps_fix_name,
            xtrack_error_m=xtrack_error_m,
            mode_state=mode_state,
        )
        msg = String()
        msg.data = status_to_json_safe(status)
        self._status_pub.publish(msg)

    def shutdown_off(self) -> None:
        self._desired_raw = False
        self._desired_debounced = False
        self._candidate = None
        self._candidate_count = 0
        self._manual_active = False
        self._manual_deadline_ns = None
        # Reset any RECOVERY backoff so the shutdown OFF is retried
        # immediately: without this, if the FSM is mid-RECOVERY (a prior OFF
        # ack failed) its backoff deadline (up to backoff_max_s = 5 s) can
        # outlast the 1 s flush window below, and the node would tear down
        # with the actuator possibly still energized. note_event_reset makes
        # the next tick's RECOVERY retry fire now.
        self._fsm.note_event_reset(time.monotonic())
        self._drive_fsm_tick("shutdown")
        # Flush: spin briefly so the OFF actually reaches MAVROS and is
        # confirmed before the executor stops. Best-effort and bounded so
        # shutdown can never hang.
        spin_once = getattr(rclpy, "spin_once", None)
        if spin_once is None:
            return
        deadline = time.monotonic() + 1.0
        while not self._fsm_off_confirmed() and time.monotonic() < deadline:
            try:
                spin_once(self, timeout_sec=0.1)
            except Exception:
                break
            if not self._fsm_off_confirmed():
                self._drive_fsm_tick("shutdown flush")


def main() -> None:
    rclpy.init()
    node: SprayControllerNode | None = None
    try:
        node = SprayControllerNode()

        def _signal_handler(signum, frame):
            raise KeyboardInterrupt

        signal.signal(signal.SIGINT, _signal_handler)
        signal.signal(signal.SIGTERM, _signal_handler)
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
