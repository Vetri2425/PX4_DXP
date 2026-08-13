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

import mission_progress as mp
from mission_progress import (
    MilestoneEvent,
    MilestoneMsg,
    MissionPhase,
    PointDoneMsg,
    ProgressMsg,
)
from spray_fsm import SpraySafetyStateMachine, SprayCommand, SprayState
from spray_flow_model import FlowModulator
from spray_modes import DashMeter, PointMeter
from spray_session_config import (
    ConfigSchemaError,
    DashConfig,
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


def _rpp_kind_for(in_mark: bool, next_boundary: str) -> str:
    """Map an `/rpp/progress` boundary onto a local spray boundary kind (G2).

    RPP announces the next MARK↔non-MARK transition as "MARK_START" / "MARK_END"
    (and "REACHED_END" at the path terminus). Translate to the spray node's own
    kinds so the existing lead math is unchanged — only the *source* of the
    distance moves from this node's projection to RPP's single authority:

      * "MARK_START"  → TRANSIT_TO_MARK  (lead the valve ON before entering)
      * "MARK_END"    → MARK_TO_TRANSIT  (lead the valve OFF before leaving)
      * "REACHED_END" while inside a mark → MARK_TO_TRANSIT — the mark ends
        because the *path* ends; this guarantees terminal shutoff even though
        RPP never emits an explicit MARK_END there.
      * anything else → "" (no upcoming boundary to lead onto).
    """
    if next_boundary == "MARK_START":
        return TRANSIT_TO_MARK
    if next_boundary == "MARK_END":
        return MARK_TO_TRANSIT
    if next_boundary == "REACHED_END" and in_mark:
        return MARK_TO_TRANSIT
    return ""

# Mirrors rpp_controller_node.SegmentStateCode.CORNER_ALIGN, read off
# /rpp/segment_debug data[1]. Duplicated rather than imported: the spray node
# must not take a build/runtime dependency on the controller module, and this
# node already runs standalone in tests. If the RPP's enum ever renumbers, this
# breaks silently -- test_spray_pivot_gate.py pins the contract.
_SEGMENT_STATE_CORNER_ALIGN = 3

# B5: SegmentStateCode values that mean "the RPP is actively driving along the
# line" (TRACK_SEGMENT=1, PRE_CORNER_SLOWDOWN=2). Seeing any of these on
# /rpp/segment_debug since the last /path load is the positive evidence that the
# run has actually started — the B5 gate below suppresses geometry-desired spray
# until then, so path arrival alone (rover parked ON a spray-flagged vertex 0,
# armed + OFFBOARD) cannot open the valve. CORNER_ALIGN(3)/DONE(4)/CORNER_STOP(5)
# /INACTIVE(0) are NOT tracking and never satisfy the gate. With the B3(a) RPP
# fix the SMOOTH profile also emits TRACK_SEGMENT, so this works for both
# profiles.
_SEGMENT_TRACKING_STATES = frozenset({1, 2})

# G2: MissionPhase values that mean "the nozzle is currently over a MARK region"
# for RPP-sourced boundary anticipation. Matches progress_classifier._MARK_PHASES
# (imported by value, not the private name, so the contract is explicit here).
_RPP_IN_MARK_PHASES = frozenset({MissionPhase.MARK_TRACKING, MissionPhase.MARK_END})

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


def _reliable_volatile_qos(depth: int = 10) -> QoSProfile:
    """RELIABLE + VOLATILE (G4 handshake channels: /rpp/milestone, /spray/point_done).

    Must arrive, but never TRANSIENT_LOCAL — a restart must not replay a stale
    milestone/done. Mirrors mp.MILESTONE_QOS / mp.POINT_DONE_QOS.
    """
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
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
    # P0-2 hysteresis latch (2026-07-30): True while the xtrack gate is tripped.
    # The node feeds this back in next tick so the gate clears at the tight
    # threshold only after tripping at the wide one — no single-threshold chatter.
    xtrack_tripped: bool = False


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
    prev_s: Optional[float] = None,
    window_back_m: float = 0.0,
    window_fwd_m: float = 0.0,
    reacquire_dist_m: float = 0.0,
    heading_rad: Optional[float] = None,
    direction_gate_cos: float = 0.0,
) -> Optional[SprayProjection]:
    """Nearest point on the path, optionally CONSTRAINED to a window around the
    previous projection.

    Why the window exists (2026-08-04). A run `/path` can double back on
    itself: the engine emits the approach leg and the marked leg in ONE message,
    and on an out-and-back line the two legs sit centimetres apart. This
    function used to re-scan every segment from scratch each tick and take the
    globally nearest one, with no memory — so "nearest" flip-flopped between the
    two legs and the reported station teleported.

    Measured on bag stg_d8a4f2ad (110 pts, 8.52 m, direction reversal at segment
    29, MARK vertices 34..104 = s 4.721..8.024). The reported station jumped
    between s=1.67 and s=7.54 while the rover was still on the approach leg, and
    the MARK flag did not settle true until s=5.10 — i.e. 41 cm past its own
    boundary. Same mechanism produced 1-3 cm paint gaps mid-mark (4-8 valve
    edges where there should be 2) and one run that never sprayed at all.

    The safety xtrack gate is what kept those spurious early flags off the
    ground; it is load-bearing and must not be loosened.

    With `prev_s` supplied, the search is restricted to segments overlapping
    [prev_s - window_back_m, prev_s + window_fwd_m]. If nothing in that window
    is within `reacquire_dist_m`, we fall back to a global scan so a genuine
    relocalisation (EKF jump, operator repositioning) can still re-acquire.
    prev_s=None (path just loaded) is always a global scan.

    Why the window alone is not enough (2026-08-05). The window is SPATIAL, so
    on an out-and-back path where the two legs sit centimetres apart it cannot
    separate them: both are inside [prev_s - back, prev_s + fwd], and "nearest"
    is decided by a couple of millimetres of lateral noise. Measured on bag
    stg_46ba8830: the station jumped 4.45 -> 5.05 in one step with ZERO samples
    in between, straddling the MARK boundary at s=4.770, so current_flag never
    saw the boundary and the valve opened 29.7 cm inside the mark. Four of
    eighteen runs lost 23-40 cm of line this way, with RTK fixed, safety_ok
    true and xtrack under 2.2 cm throughout - nothing in telemetry flagged it.

    Distance cannot disambiguate coincident legs, but DIRECTION can: the two
    legs have opposite bearings and the vehicle heading says which one it is
    on. With `heading_rad` supplied and `direction_gate_cos` > 0, segments
    whose unit direction has cos(angle to heading) below the threshold are
    rejected before the nearest-segment comparison. Fallback is layered so the
    gate can never strand the projection: direction-filtered window -> plain
    window -> global scan.

    Passing prev_s=None / zero windows reproduces the pre-fix behaviour exactly
    and is the A/B arm; direction_gate_cos=0.0 disables the direction gate
    alone, leaving the window behaviour untouched.
    """
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

    gate_active = (
        heading_rad is not None
        and direction_gate_cos > 0.0
        and math.isfinite(heading_rad)
    )
    if gate_active:
        head_n = math.cos(heading_rad)
        head_e = math.sin(heading_rad)

    def _scan(indices, use_direction_gate: bool = False):
        best: Optional[SprayProjection] = None
        best_dist = float("inf")
        for i in indices:
            a_n, a_e = model.points[i]
            b_n, b_e = model.points[i + 1]
            d_n = b_n - a_n
            d_e = b_e - a_e
            seg_len_sq = d_n * d_n + d_e * d_e
            if use_direction_gate and seg_len_sq > 1e-12:
                # Reject segments running against the vehicle. On coincident
                # out-and-back legs this is the only signal that separates
                # them; lateral distance is noise at that point.
                inv = 1.0 / math.sqrt(seg_len_sq)
                if (d_n * inv) * head_n + (d_e * inv) * head_e < direction_gate_cos:
                    continue
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
                current_flag = (
                    model.flags[i + 1] if t >= 1.0 - 1e-12 else model.flags[i]
                )
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
        return best, best_dist

    n_seg = len(model.points) - 1
    windowed = (
        prev_s is not None
        and (window_back_m > 0.0 or window_fwd_m > 0.0)
    )
    if windowed:
        lo = prev_s - max(0.0, window_back_m)
        hi = prev_s + max(0.0, window_fwd_m)
        # A segment is a candidate when its [s_start, s_end] overlaps [lo, hi].
        idx = [
            i for i in range(n_seg)
            if model.cumulative_s[i + 1] >= lo and model.cumulative_s[i] <= hi
        ]
        if idx:
            # Layered fallback: the direction gate may legitimately eliminate
            # every candidate (pivoting on the spot, reversing), so it is only
            # ever a preference, never a trap.
            if gate_active:
                best, best_dist = _scan(idx, use_direction_gate=True)
                if best is not None and (
                    reacquire_dist_m <= 0.0 or best_dist <= reacquire_dist_m
                ):
                    return best
            best, best_dist = _scan(idx)
            # Only trust the window while it actually explains where we are.
            if best is not None and (
                reacquire_dist_m <= 0.0 or best_dist <= reacquire_dist_m
            ):
                return best
    if gate_active:
        best, best_dist = _scan(range(n_seg), use_direction_gate=True)
        if best is not None and (
            reacquire_dist_m <= 0.0 or best_dist <= reacquire_dist_m
        ):
            return best
    return _scan(range(n_seg))[0]


def _direction_gate_cos(gate_deg: float) -> float:
    """Half-angle in degrees -> cos threshold for the projection direction gate.

    <=0 or >=180 disables the gate (returns 0.0, which _project_onto_path
    treats as off). 90 deg -> 0.0 would collide with "disabled", so the
    disabled sentinel is checked first and 90 deg maps to a tiny positive
    threshold: reject anything that opposes the heading at all.
    """
    if gate_deg <= 0.0 or gate_deg >= 180.0:
        return 0.0
    return max(1e-6, math.cos(math.radians(gate_deg)))


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


def _first_mark_start_s(model: SprayPathModel) -> Optional[float]:
    """Arc-length of the first MARK_START station (dash pattern geometry origin).

    Prefers the first TRANSIT_TO_MARK boundary. If the path opens already on a
    MARK (no transit lead-in), that is s=0 at the first station.
    """
    for boundary in model.boundaries:
        if boundary.kind == TRANSIT_TO_MARK:
            return float(boundary.s)
    if model.flags and model.flags[0]:
        return float(model.cumulative_s[0])
    return None


def _apply_mark_boundary_lead(
    geometry_desired: bool,
    src_kind: Optional[str],
    src_dist: float,
    on_lead: float,
    off_lead: float,
) -> tuple[bool, str]:
    """Apply solenoid/overspray lead at a MARK↔TRANSIT boundary.

    ON fires early by open-delay + on-overspray; OFF fires early by close-delay
    (minus off-overspray). Returns (new_geometry_desired, event) where event is
    '' | 'on_early' | 'off_early'. Shared by continuous and dash so both modes
    get the same physical paint edges (B2).
    """
    if src_kind is None or not math.isfinite(src_dist):
        return geometry_desired, ""
    if (
        not geometry_desired
        and src_kind == TRANSIT_TO_MARK
        and src_dist <= on_lead
    ):
        return True, "on_early"
    if (
        geometry_desired
        and src_kind == MARK_TO_TRANSIT
        and src_dist <= off_lead
    ):
        return False, "off_early"
    return geometry_desired, ""


def _mark_region_with_lead(
    model: SprayPathModel,
    projection: SprayProjection,
    on_lead: float,
    off_lead: float,
) -> bool:
    """True when the nozzle is inside the led MARK region (dash flag gate).

    Same lead math as continuous: opens early before TRANSIT_TO_MARK, closes
    early before MARK_TO_TRANSIT. Without this, AND-ing raw current_flag makes
    the first dash of every marked run start late and overrun the end.
    """
    base = projection.current_flag
    boundary = _next_boundary(model, projection.s, projection.current_flag)
    if boundary is None:
        return base
    src_dist = boundary.s - projection.s
    gated, _event = _apply_mark_boundary_lead(
        base, boundary.kind, src_dist, on_lead, off_lead
    )
    return gated


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
    max_xtrack_source: str = "param",
    # P0-2 (2026-07-30): xtrack gate hysteresis. A single hard threshold
    # chatters when tracking error sits ON it — the 07-30 curve run cut a
    # 31 cm hole mid-mark at 5.12 cm vs a 5.00 cm gate while geometry still
    # wanted paint. Trip at the WIDE level, clear at the tight one, and once
    # tripped stay off at least xtrack_gate_min_off_s (solenoid protection).
    # trip <= clear (e.g. 0.0 default) degrades to the old single-threshold
    # behaviour exactly.
    xtrack_trip_error_m: float = 0.0,
    xtrack_tripped: bool = False,
    xtrack_tripped_elapsed_s: float = float("inf"),
    xtrack_gate_min_off_s: float = 0.0,
    mode: str = "continuous",
    dash_meter: Optional["DashMeter"] = None,
    dt_s: float = 0.0,
    point_meter: Optional["PointMeter"] = None,
    yaw: float = 0.0,
    now_s: float = 0.0,
    off_confirmed: bool = False,
    # B4 — terminal shutoff (speed-independent). The OFF lead is speed-scaled
    # (speed x solenoid_close_delay_s), so at the terminal creep speed it is
    # ~1 mm against a ~14 mm stopping gap: the geometric MARK->TRANSIT boundary
    # at the FINAL station is never crossed and the valve latches ON after the
    # rover stops short. When the nozzle is within terminal_off_epsilon_m of the
    # final path station AND the rover has effectively stopped, force geometry
    # OFF. Gated on near-zero speed, so a moving MARK tail is never cut.
    terminal_off_epsilon_m: float = 0.05,
    terminal_off_speed_mps: float = 0.05,
    # G2 — RPP progress boundary sourcing (continuous mode only). When
    # rpp_in_mark is None (flag off / stale / absent), the continuous branch
    # uses this node's own /path projection exactly as before (byte-for-byte
    # frozen). When provided, RPP is the single authority for the boundary the
    # lead math anticipates, killing dual-projection drift on moving marks.
    rpp_in_mark: Optional[bool] = None,
    rpp_boundary_kind: str = "",
    rpp_dist_to_boundary_m: float = float("inf"),
    # G4 — point handshake gate. None (default) → the PointMeter self-arrives from
    # pose exactly as before (frozen). A bool → RPP is the arrival authority for
    # the meter's current target: True iff /rpp/milestone AT_POINT fired for it.
    rpp_at_point_gate: Optional[bool] = None,
    # Projection continuity (2026-08-04). See _project_onto_path: a run /path
    # that doubles back on itself makes a memoryless nearest-segment search
    # flip between the two legs, which delayed the MARK flag by up to 41 cm and
    # punched gaps mid-mark. Threaded like xtrack_tripped: caller passes the
    # previous station in and stores projection.s back out. Zero windows
    # reproduce the pre-fix behaviour exactly (A/B arm).
    prev_projection_s: Optional[float] = None,
    projection_window_back_m: float = 0.0,
    projection_window_fwd_m: float = 0.0,
    projection_reacquire_dist_m: float = 0.0,
    # Direction gate (2026-08-05). The spatial window cannot separate the two
    # legs of an out-and-back path; vehicle heading can. 0.0 disables, leaving
    # the window behaviour byte-for-byte. See _project_onto_path.
    projection_direction_gate_cos: float = 0.0,
) -> SprayDecision:
    projection: Optional[SprayProjection] = None
    boundary: Optional[SprayBoundary] = None
    distance_to_boundary = float("inf")
    geometry_desired = False
    event = ""

    # Point mode (plan §7.3): no path projection — arrival is the nozzle vs the
    # coordinate list. Uses the standard safety_ok from the gate stack (armed/
    # offboard/pose/gps/pivot-exempt), computed by the caller.
    if mode == "point":
        # Point mode NEVER falls through to path projection: with no meter yet
        # (coordinates not resolved from /path), the safe answer is spray OFF —
        # not continuous spraying off the geometry mirror.
        pu = None
        if point_meter is not None and nozzle_n is not None and nozzle_e is not None:
            pu = point_meter.update(
                nozzle_n, nozzle_e, yaw, speed_mps, now_s, off_confirmed,
                require_arrival_gate=rpp_at_point_gate,
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
        projection = _project_onto_path(
            model, nozzle_n, nozzle_e,
            prev_s=prev_projection_s,
            window_back_m=projection_window_back_m,
            window_fwd_m=projection_window_fwd_m,
            reacquire_dist_m=projection_reacquire_dist_m,
            heading_rad=yaw,
            direction_gate_cos=projection_direction_gate_cos,
        )
    new_xtrack_tripped = False
    if projection is not None:
        boundary = _next_boundary(model, projection.s, projection.current_flag)
        geometry_desired = projection.current_flag
        # Hysteresis: trip at the wide level; once tripped, clear only when the
        # error is back under the tight level AND the minimum off-dwell has
        # elapsed. With trip <= clear this reduces to the old single threshold.
        # R3 contract: a MISSION override IS the gate — it may tighten below
        # the param trip, so the param band must not widen it (the Jetson
        # rclpy suite pins this). Param-sourced gates keep the anti-chatter
        # band; the min-off dwell applies to both.
        if max_xtrack_source == "mission":
            trip_level = max_xtrack_error_m
        else:
            trip_level = max(max_xtrack_error_m, xtrack_trip_error_m)
        if xtrack_tripped:
            new_xtrack_tripped = (
                projection.xtrack_error_m > max_xtrack_error_m
                or xtrack_tripped_elapsed_s < xtrack_gate_min_off_s
            )
        else:
            new_xtrack_tripped = projection.xtrack_error_m > trip_level
        if new_xtrack_tripped:
            safety_ok = False
            safety_reason = (
                f"xtrack error {projection.xtrack_error_m:.3f}m "
                f"gate trip>{trip_level:.3f}m clear<={max_xtrack_error_m:.3f}m "
                f"({max_xtrack_source})"
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
            # R5/B2: meter still integrates across TRANSIT connectors (locked
            # "continuous across mission"), but the valve follows the led MARK
            # region — same on_early/off_early as continuous, not raw flag.
            on_lead = speed_mps * solenoid_open_delay_s + on_overspray_margin_m
            off_lead = max(
                0.0, speed_mps * solenoid_close_delay_s - off_overspray_margin_m
            )
            xtrack_ok = projection.xtrack_error_m <= max_xtrack_error_m
            du = dash_meter.update(
                projection.s, speed_mps, dt_s, xtrack_ok, on_lead, off_lead
            )
            mark_gate = _mark_region_with_lead(
                model, projection, on_lead, off_lead
            )
            geometry_desired = du.geometry_desired and mark_gate
            boundary = None
            distance_to_boundary = float("inf")
        else:
            # Continuous lead. The boundary the valve anticipates comes from one
            # of two sources; the lead equations below are identical either way.
            #   * RPP progress (G2): rpp_in_mark is not None → single authority,
            #     no dual projection. `boundary` is re-synthesized for reporting.
            #   * /path projection (frozen default): rpp_in_mark is None → the
            #     exact pre-G2 behaviour, byte-for-byte.
            if rpp_in_mark is not None:
                geometry_desired = rpp_in_mark
                src_kind = rpp_boundary_kind or None
                src_dist = rpp_dist_to_boundary_m
                boundary = (
                    SprayBoundary(projection.s + src_dist, src_kind)
                    if src_kind is not None and math.isfinite(src_dist)
                    else None
                )
            elif boundary is not None:
                src_kind = boundary.kind
                src_dist = boundary.s - projection.s
            else:
                src_kind = None
                src_dist = float("inf")

            if src_kind is not None and math.isfinite(src_dist):
                distance_to_boundary = src_dist
                # ON is intentionally early by solenoid delay plus overspray
                # margin. OFF is early only by close delay; an explicit OFF
                # overspray margin delays shutoff so the MARK tail is not cut
                # short. Shared helper with dash (B2) — continuous output must
                # stay byte-for-byte identical to the inlined form.
                on_lead = speed_mps * solenoid_open_delay_s + on_overspray_margin_m
                off_lead = max(
                    0.0,
                    speed_mps * solenoid_close_delay_s - off_overspray_margin_m,
                )
                geometry_desired, lead_event = _apply_mark_boundary_lead(
                    geometry_desired, src_kind, src_dist, on_lead, off_lead
                )
                if lead_event:
                    event = lead_event

        # B4 terminal shutoff — fires independently of any boundary/lead so
        # it works even when the rover stops short of the final MARK station
        # (the off_early path above cannot: its lead is ~1 mm at creep
        # speed). END-specific, NOT a speed gate: it requires proximity to
        # the FINAL station, so a slow mid-line MARK keeps painting.
        # Shared by continuous and dash (R6): previously nested under the
        # continuous-only else, so dash sessions never reached it.
        if (
            geometry_desired
            and model.cumulative_s
            and speed_mps <= terminal_off_speed_mps
            and (model.cumulative_s[-1] - projection.s) <= terminal_off_epsilon_m
        ):
            geometry_desired = False
            event = "terminal_off"

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
        xtrack_tripped=new_xtrack_tripped,
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
        # 2026-08-13 field spray timing test: visible paint starts remained
        # late at 0.5-1.0 m/s while OFF timing was clean. Advance ON only.
        self.declare_parameter("solenoid_open_delay_s", 0.18)
        self.declare_parameter("solenoid_close_delay_s", 0.05)
        # Legacy V2 name kept so old launch overrides do not fail. New code
        # uses explicit ON/OFF margins below to avoid shortening MARK tails.
        self.declare_parameter("anticipatory_margin_m", 0.02)
        self.declare_parameter("on_overspray_margin_m", 0.02)
        self.declare_parameter("off_overspray_margin_m", 0.0)
        # ── B4: terminal shutoff (speed-independent) ─────────────────────────
        # Guarantees the valve closes at the end of the path. The rover finishes
        # at a creep speed (0.005-0.03 m/s), where the speed-scaled off_lead is
        # sub-millimetre and the geometric MARK->TRANSIT boundary at the final
        # station is never crossed. When the nozzle is within
        # terminal_off_epsilon_m of the final station AND effectively stopped,
        # spray is forced OFF. End-specific (proximity to the LAST station), so
        # a slow mid-line MARK is unaffected.
        self.declare_parameter("terminal_off_epsilon_m", 0.05)
        self.declare_parameter("terminal_off_speed_mps", 0.05)
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
        # ⚠ INERT — DECLARED BUT NEVER READ. Nothing consumes this value; the
        # only other mention in this file is the note at the old gate site. It
        # is kept solely so the server/UI settings contract does not change, and
        # `test_min_spray_speed_param_is_inert` pins that it has no authority.
        # Do NOT wire it back up as a bare threshold — that is the exact change
        # that produced the 132 spurious fires described above. If stationary
        # spraying needs suppressing, use a DISCRETE signal (as
        # spray_off_during_pivot does); a threshold on a quantity the rover
        # deliberately crawls across will always dither.
        self.declare_parameter("min_spray_speed_mps", 0.05)
        # Replacement for the speed gate: suppress spray only while the rover
        # is pivoting in place, which is the actual thing we needed to avoid
        # (a stationary nozzle sweeping an arc puddles paint). Sourced from the
        # RPP's own state machine rather than inferred from a speed threshold --
        # a discrete state cannot dither the way a threshold does.
        self.declare_parameter("spray_off_during_pivot", True)
        self.declare_parameter("segment_state_timeout_s", 1.0)
        # ── RPP progress consumption (design RPP_PROGRESS_HANDSHAKE, G2/G4) ──
        # G0: declared now, default OFF, NOT yet consumed. When enabled, the
        # boundary for the moving-mark lead math is sourced from /rpp/progress
        # instead of this node's own /path projection (single source, no dual-
        # projection drift), falling back to /path if progress is stale beyond
        # progress_timeout_s. Generalizes the existing pivot-gate pattern.
        self.declare_parameter("consume_rpp_progress", False)
        self.declare_parameter("progress_timeout_s", 0.3)   # s → fallback to /path
        # Suppress spray when the nozzle's cross-track error exceeds this.
        #
        # 0.10 -> 0.03 in c4fdcde (Stage 1 R3, 2026-07-29) on the reasoning that
        # 0.10 was 5x the +/-2 cm spec and therefore never actually fired. The
        # 2026-07-30 bags showed why that reasoning was wrong: 0.03 sits INSIDE
        # the rover's own error distribution, so the gate chatters and breaks a
        # single line into 2-3 painted fragments. Every suppression in all 11
        # bags that day had the same reason string, "xtrack error 0.031m >
        # 0.030m (param)", and the gate ate 19-41% of the mark window on the
        # worst runs -- while the underlying tracking was a benign +/-3-4 cm
        # entry transient, not an off-path excursion.
        #
        # 0.05 is the operator's call: above the observed p95 (2.4-4.8 cm) so a
        # settling line still paints continuously, but still tight enough to cut
        # a genuine excursion. A broken line is worse than a line 3 cm off.
        #
        # This is a FLOOR on paint continuity, not an accuracy spec -- do not
        # read it as "5 cm is acceptable marking error". Per-mission override
        # via session_config max_xtrack_error_m (R3) still wins when present.
        self.declare_parameter("max_xtrack_error_m", 0.05)
        # P0-2 (2026-07-30): hysteresis on the gate above. The 07-30 curve run
        # showed 0.05 STILL chatters when tracking error rides the threshold
        # (max 5.12 cm vs gate 5.00 → a 31 cm unpainted hole mid-mark, and the
        # same mechanism that made 0.03 chatter before e4b04d1). Trip only
        # above this WIDE level; clear back at max_xtrack_error_m. Set <= the
        # clear level to disable hysteresis (old single-threshold behaviour).
        self.declare_parameter("xtrack_trip_error_m", 0.08)
        # Once tripped, hold the gate off at least this long — bounds solenoid
        # cycling if error oscillates fast across both levels. Keep small: every
        # extra tenth of a second tripped is ~3.5 cm of unpainted line at cruise.
        self.declare_parameter("xtrack_gate_min_off_s", 0.2)
        # Projection continuity window (2026-08-04). A run /path can carry the
        # approach leg and the marked leg in one message; on an out-and-back
        # line the two legs sit centimetres apart and a memoryless
        # nearest-segment search flips between them (measured: station jumping
        # 1.67 <-> 7.54 m, MARK flag 41 cm late, 1-3 cm gaps mid-mark, one run
        # that never sprayed). Restrict the search to a window around the last
        # station. Back window covers reverse creep and jitter; forward window
        # must exceed the furthest the nozzle can advance between ticks with
        # margin (50 Hz at 0.35 m/s is ~7 mm, so 2.0 m is ~280x headroom and
        # still far short of the ~3.8 m leg spacing that caused the confusion).
        # Set both to 0 to restore the pre-fix global search (A/B arm).
        self.declare_parameter("projection_window_back_m", 0.5)
        self.declare_parameter("projection_window_fwd_m", 2.0)
        # If nothing in the window is within this distance, fall back to a
        # global scan so a real relocalisation can still re-acquire. Must stay
        # well above normal cross-track (cm) and below the leg spacing.
        self.declare_parameter("projection_reacquire_dist_m", 1.0)
        # Direction gate (2026-08-05). Half-angle, degrees, about the vehicle
        # heading; segments outside it are rejected before the nearest-segment
        # comparison. 0 disables. 90 accepts anything not actively opposing the
        # vehicle, which is what separates the two legs of an out-and-back path
        # -- the spatial window cannot, because both legs are inside it. Four of
        # eighteen runs on 2026-08-05 lost 23-40 cm of line to that ambiguity.
        #
        # DEFAULT 0.0 (INERT) until an offline replay reproduces the measured
        # baseline. The first harness attempt did not: its gate-OFF arm scored
        # 114849 at 330 cm late when the field bag shows it 2 cm EARLY, so it
        # was not modelling _proj_prev_s seeding or the 10 Hz nozzle feed. A
        # treatment arm means nothing while the control arm is wrong. Do not
        # raise this on the vehicle until that replay is faithful.
        self.declare_parameter("projection_direction_gate_deg", 0.0)
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
        # A14 (2026-07-27): fix_type alone is NOT accuracy. GPSRAW carries
        # h_acc on the very same message the gate already reads, and until now
        # it was discarded — so an RTK_FIXED claim opened the valve regardless
        # of the reported error. This is the accuracy half of the gate.
        #
        # FALLBACK BY DESIGN, which is why it ships ON: h_acc == 0 is the
        # driver's "unknown" sentinel (~half of boots on this hardware report
        # it, latched per boot — see A14), and refusing on unknown would ground
        # the rover for reasons unrelated to safety. So: when accuracy IS
        # reported, enforce it; when it is NOT, behave exactly as before. That
        # is strictly safer than today on every boot that reports, and
        # byte-identical on every boot that does not.
        # Default 0.10 m ≈ 6x the 1.4–1.6 cm an RTK_FIXED solution measures on
        # this rig, so it only trips on a genuinely degraded fix that is still
        # claiming fix_type 6. Set 0 to disable the accuracy half entirely.
        self.declare_parameter("spray_max_hrms_m", 0.10)
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
        # B4 fail-closed watchdog: when /spray/active first went False (cleared
        # to None on True). Lets the DEFAULT distance-aware path force spray OFF
        # once the RPP has stopped asserting a MARK for active_timeout_s, even
        # though it computes geometry locally and does not consume /spray/active
        # for the decision. Mission end publishes active=False continuously.
        self._active_false_since = None
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
        # Stashed dash params from session_config — mirrored by
        # _rebuild_dash_meter on every /path (same pattern as _point_params /
        # _rebuild_point_meter). A new path is a new arc-length origin; patching
        # anchor_s on an already-armed meter is a no-op at arm-time read.
        self._dash_config: Optional[DashConfig] = None
        # P0-2 xtrack-gate hysteresis latch: tripped flag + monotonic stamp of
        # the trip edge, fed back into _make_spray_decision each tick.
        self._xtrack_tripped: bool = False
        self._xtrack_trip_mono: Optional[float] = None
        # Last path station, so the next projection stays on the same leg of a
        # doubled-back path. None = acquire globally (fresh path / no fix yet).
        self._proj_prev_s: Optional[float] = None
        # Per-mission xtrack gate from session_config (R3). Stashed separately
        # because _path_cb overwrites _session_config with a continuous geometry
        # mirror (same reason _dash_config is stashed). None → ROS param.
        self._mission_max_xtrack_error_m: Optional[float] = None
        # Phase D point mode. _last_point_update lets _auto_safety_status apply
        # the pivot-gate exemption using the most recent point FSM state (it
        # runs one step before _make_spray_decision updates the meter).
        self._point_meter: Optional[PointMeter] = None
        self._last_point_update = None
        # Phase D coordinate source. Point dwell targets are the must-hit
        # vertices carried on /path (bit1) — the ONLY frame-correct source,
        # because GPS_SURVEYED placement offsets the whole path into the live
        # EKF frame at mission start (server-staged coords are pre-placement and
        # would land the dots off by that offset). _point_config_coords is a
        # fallback used only when a session_config ships explicit coordinates
        # and /path has no must-hit vertices (bench/direct use, tests).
        self._path_must_hit_points: list[tuple[float, float]] = []
        self._point_config_coords: list[tuple[float, float]] = []
        self._point_params: dict = {}
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
        # None = the receiver did not report accuracy this boot (A14 sentinel).
        self._gps_h_acc_m: Optional[float] = None
        self._gps_recv_time = None
        self._gps_recover_since = None
        self._last_decision: Optional[SprayDecision] = None
        self._pose_ned: Optional[tuple[float, float, float]] = None
        self._pose_recv_time = None
        self._vel_ned = (0.0, 0.0)
        self._vel_recv_time = None
        self._segment_state: Optional[int] = None
        self._segment_state_recv_time = None
        # B5: has the RPP reported an actively-tracking state since the last
        # /path load? Default True (permissive) so a node that is driven by
        # direct model injection — never by a real /path message — behaves as
        # before. A real /path arrival (`_path_cb`) resets it to False; the
        # first TRACK_SEGMENT/PRE_CORNER_SLOWDOWN on /rpp/segment_debug sets it
        # back to True. Until then, `_auto_safety_status` refuses geometry ON
        # ("awaiting tracking"), so path load onto a spray-flagged vertex 0
        # cannot blip the valve. Two-stage entry missions reset it on EACH new
        # path (entry path first, then marking path) — correct: no spray during
        # entry, and the entry path carries no spray flags anyway.
        self._tracking_seen_since_path_load = True
        # G2: latest RPP progress (boundary authority for continuous marks).
        # None until the first message; recv_time drives the staleness fallback
        # to /path (progress_timeout_s). _rpp_source tracks which boundary source
        # the last tick actually used, so a switch is logged once (rate-limited),
        # not every tick.
        self._rpp_progress: Optional[ProgressMsg] = None
        self._rpp_progress_recv_time = None
        self._rpp_source = ""
        # G4 — point handshake. The latest /rpp/milestone AT_POINT index (the RPP
        # confirms the rover is precisely stopped on this point); _point_done_seq
        # is the monotonic counter for our outbound /spray/point_done. Reset when
        # the point session is (re)built so a stale AT_POINT can't gate a new run.
        self._at_point_index: int = -1
        self._at_point_seq: int = -1
        self._point_done_seq: int = 0
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
        # G4 — point-handshake completion (spray → RPP). RELIABLE VOLATILE depth 10
        # (mp.POINT_DONE_QOS): must arrive so the RPP advances, but never
        # TRANSIENT_LOCAL — a restart must not replay a stale "done".
        self._point_done_pub = self.create_publisher(
            String, mp.TOPIC_POINT_DONE, _reliable_volatile_qos(mp.POINT_DONE_QOS.depth)
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
        # G2: RPP mission-progress channel. BEST_EFFORT depth 1 (matches
        # mp.PROGRESS_QOS and /rpp/segment_debug) — republished every 50 Hz tick,
        # so loss is self-healing. Read-only; the frozen controller is untouched.
        # Only consumed when consume_rpp_progress is set AND the stream is fresh;
        # otherwise the node falls back to its own /path projection.
        self.create_subscription(
            String,
            mp.TOPIC_PROGRESS,
            self._rpp_progress_cb,
            _best_effort_qos(),
            callback_group=self._group,
        )
        # G4 — RPP discrete milestones (RELIABLE VOLATILE depth 10). The point
        # handshake reacts ONLY to AT_POINT i: the RPP has precisely stopped the
        # rover on point i, so the dwell FSM may spray with proof. Read-only; used
        # only when consume_rpp_progress is set AND mode is point.
        self.create_subscription(
            String,
            mp.TOPIC_MILESTONE,
            self._milestone_cb,
            _reliable_volatile_qos(mp.MILESTONE_QOS.depth),
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
        now = self.get_clock().now()
        self._last_active_time = now
        raw = bool(msg.data)
        # B4: track the start of a sustained-False span so the distance-aware
        # watchdog can time it against active_timeout_s (below). True clears it.
        if raw:
            self._active_false_since = None
        elif self._active_false_since is None:
            self._active_false_since = now
        self._legacy_active_raw = raw
        if (
            not bool(self.get_parameter("use_distance_aware_spray").value)
            and bool(self.get_parameter("allow_legacy_spray_active_fallback").value)
        ):
            self._set_auto_desired(self._legacy_active_raw, source="legacy")

    def _path_cb(self, msg: Path) -> None:
        # B5: any new /path resets the "run has started" evidence. Auto-spray is
        # then held ("awaiting tracking") until the RPP reports an actively-
        # tracking state, so path arrival onto a spray-flagged vertex 0 cannot
        # open the valve before the rover is actually driving the line.
        self._tracking_seen_since_path_load = False
        # A new path invalidates the previous station — the next projection must
        # acquire globally rather than snap to a window of the OLD geometry.
        self._proj_prev_s = None
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        # position.z is a bitfield: bit0 = spray ON, bit1 = must-hit vertex.
        # MUST bit-test, not `> 0.5`: a spray-OFF must-hit point encodes as 2.0
        # and a `> 0.5` test would spray a transit leg.
        flags = [bool(int(round(p.pose.position.z)) & 1) for p in msg.poses]
        # bit1 = must-hit vertex → the point-mode dwell targets, in the SAME
        # (placed, live-EKF) frame as the pose the meter compares against.
        must_hit = [bool(int(round(p.pose.position.z)) & 2) for p in msg.poses]
        self._path_must_hit_points = [
            pt for pt, m in zip(points, must_hit) if m
        ]
        if not points:
            self._path_model = None
            self._path_must_hit_points = []
            self._session_config = cleared_config()
            self._config_fingerprint = self._session_config.path_fingerprint()
            self._fsm.note_event_reset(time.monotonic())
            self._set_auto_desired(False, source="distance")
            self._rebuild_point_meter()
            self._rebuild_dash_meter()
            self.get_logger().warn("spray path cleared: received empty /path")
            return
        try:
            self._path_model = _build_path_model(points, flags)
        except ValueError as exc:
            self._path_model = None
            self._session_config = cleared_config()
            self._config_fingerprint = self._session_config.path_fingerprint()
            self._set_auto_desired(False, source="distance")
            self._rebuild_point_meter()
            self._rebuild_dash_meter()
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
        # Point mode: the dwell targets just changed (new placed /path), so
        # rebuild the meter from the fresh must-hit vertices. No-op otherwise.
        self._rebuild_point_meter()
        # Dash (B1): a new /path is a new arc-length origin — rebuild the meter
        # with the fresh MARK_START anchor. set_anchor_s alone is not enough:
        # the anchor is read once at arm time, so an already-armed entry-leg
        # meter would keep its stale grid through advance_entry_to_marking.
        self._rebuild_dash_meter()
        self.get_logger().info(
            f"spray path loaded: {len(points)} points, "
            f"{len(self._path_model.boundaries)} boundaries"
            f"{f', {len(self._path_must_hit_points)} must-hit' if self._path_must_hit_points else ''}"
        )

    def _rebuild_dash_meter(self) -> None:
        """(Re)build the dash arc-length meter from stashed config + current /path.

        Mirrors `_rebuild_point_meter`: session_config stashes the on/off/start
        params; every /path rebuilds so the geometry anchor (first MARK_START)
        matches the path the rover is about to drive. No-op outside dash mode.
        """
        if self._session_mode != "dash" or self._dash_config is None:
            return
        anchor_s = (
            _first_mark_start_s(self._path_model)
            if self._path_model is not None
            else None
        )
        try:
            self._dash_meter = DashMeter(
                self._dash_config.on_distance_m,
                self._dash_config.off_distance_m,
                self._dash_config.start_state,
                anchor_s=anchor_s,
            )
        except ValueError as exc:
            self.get_logger().warn(
                f"dash meter rebuild failed ({exc}); reverting to continuous"
            )
            self._dash_meter = None
            self._dash_config = None
            self._session_mode = "continuous"

    def _rebuild_point_meter(self) -> None:
        """(Re)build the point-dwell meter from the current coordinate source.

        Frame-correct source priority: the must-hit vertices on /path (placed
        into the live EKF frame by the controller) win; explicit session_config
        coordinates are a fallback for bench/direct use with no must-hit path.
        No coordinates → no meter (point mode then commands spray OFF, safe).
        Only acts in point mode; a no-op for continuous/dash.
        """
        if self._session_mode != "point":
            return
        # G4: a rebuilt session invalidates any cached AT_POINT — a stale
        # milestone must not gate the new run's first point.
        self._at_point_index = -1
        self._at_point_seq = -1
        coords = self._path_must_hit_points or self._point_config_coords
        if not coords:
            self._point_meter = None
            self._last_point_update = None
            return
        params = self._point_params
        try:
            self._point_meter = PointMeter(
                coords,
                params.get("arrival_tolerance_m", 0.05),
                params.get("arrival_settle_s", 0.2),
                params.get("dwell_s", 1.0),
                params.get("heading_tolerance_deg"),
                point_arrival_max_speed_mps=max(
                    0.0, float(self.get_parameter("point_arrival_max_speed_mps").value)
                ),
                point_arrival_timeout_s=max(
                    0.0, float(self.get_parameter("point_arrival_timeout_s").value)
                ),
            )
            self._last_point_update = None
        except ValueError as exc:
            self.get_logger().warn(
                f"point meter rebuild failed ({exc}); reverting to continuous"
            )
            self._point_meter = None
            self._session_mode = "continuous"

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
        # R3: stash per-mission xtrack (survives _path_cb geometry mirror).
        self._mission_max_xtrack_error_m = cfg.max_xtrack_error_m

        self._dash_meter = None
        self._dash_config = None
        self._point_meter = None
        self._last_point_update = None
        if cfg.mode == "dash" and cfg.dash is not None:
            # Stash params; rebuild picks up the current /path anchor (or None
            # if path has not arrived yet — a later /path rebuilds again).
            self._dash_config = cfg.dash
            self._rebuild_dash_meter()
        elif cfg.mode == "point" and cfg.points_mode is not None:
            # Store the dwell/tolerance PARAMS + any config-supplied coordinates
            # (fallback). The authoritative dwell targets are the /path must-hit
            # vertices; _rebuild_point_meter prefers them and only uses these
            # coordinates when /path carries none (bench/direct use).
            pm = cfg.points_mode
            self._point_config_coords = list(pm.coordinates)
            self._point_params = {
                "arrival_tolerance_m": pm.arrival_tolerance_m,
                "arrival_settle_s": pm.arrival_settle_s,
                "dwell_s": pm.dwell_s,
                "heading_tolerance_deg": pm.heading_tolerance_deg,
            }
            self._rebuild_point_meter()

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

    def _active_heartbeat_forces_off(self) -> bool:
        """B4 mission-level fail-closed watchdog, live in distance-aware mode.

        The RPP's /spray/active is the authority on whether a mission MARK is in
        progress at all; it is republished every control tick (True during a
        MARK, False otherwise, and False continuously once the mission ends).
        Distance-aware mode decides geometry locally from /path and previously
        ignored /spray/active entirely, so at mission end the valve stayed open
        with the rover stopped short of the final station. Force OFF when
        /spray/active has been absent (RPP silent) OR False for longer than
        active_timeout_s. Returns False until the first /spray/active is ever
        seen, so a deployment that never wires the topic keeps geometry
        authoritative (and every existing distance-aware test, which sends no
        /spray/active, is unaffected).
        """
        if self._last_active_time is None:
            return False
        timeout_s = max(0.0, float(self.get_parameter("active_timeout_s").value))
        now = self.get_clock().now()
        age_s = (now - self._last_active_time).nanoseconds * 1e-9
        if age_s > timeout_s:
            return True  # RPP stopped publishing — fail closed
        if self._active_false_since is not None:
            false_age_s = (now - self._active_false_since).nanoseconds * 1e-9
            if false_age_s > timeout_s:
                return True  # RPP has held not-active (e.g. mission ended)
        return False

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
        # G2: pick the boundary source (RPP progress vs local /path projection).
        rpp_in_mark, rpp_boundary_kind, rpp_dist_m = self._rpp_boundary_inputs(
            self._session_mode
        )
        # G4: point-handshake arrival gate for the meter's current target (None
        # unless consume_rpp_progress + point mode → frozen self-arrival).
        rpp_at_point_gate = (
            self._point_handshake_gate(self._point_meter.target_index)
            if self._point_meter is not None
            else None
        )
        # R3: one resolved gate value for the decision. Mission session override
        # wins when present; otherwise the ROS param. Provenance rides only in
        # safety_reason so the operator knows which knob refused the spray.
        if self._mission_max_xtrack_error_m is not None:
            max_xtrack_error_m = float(self._mission_max_xtrack_error_m)
            max_xtrack_source = "mission"
        else:
            max_xtrack_error_m = max(
                0.0,
                float(self.get_parameter("max_xtrack_error_m").value),
            )
            max_xtrack_source = "param"
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
            max_xtrack_error_m=max_xtrack_error_m,
            max_xtrack_source=max_xtrack_source,
            xtrack_trip_error_m=max(
                0.0,
                float(self.get_parameter("xtrack_trip_error_m").value),
            ),
            xtrack_tripped=self._xtrack_tripped,
            xtrack_tripped_elapsed_s=(
                (now_mono - self._xtrack_trip_mono)
                if self._xtrack_trip_mono is not None
                else float("inf")
            ),
            xtrack_gate_min_off_s=max(
                0.0,
                float(self.get_parameter("xtrack_gate_min_off_s").value),
            ),
            terminal_off_epsilon_m=max(
                0.0,
                float(self.get_parameter("terminal_off_epsilon_m").value),
            ),
            terminal_off_speed_mps=max(
                0.0,
                float(self.get_parameter("terminal_off_speed_mps").value),
            ),
            rpp_in_mark=rpp_in_mark,
            rpp_boundary_kind=rpp_boundary_kind,
            rpp_dist_to_boundary_m=rpp_dist_m,
            rpp_at_point_gate=rpp_at_point_gate,
            prev_projection_s=self._proj_prev_s,
            projection_window_back_m=float(
                self.get_parameter("projection_window_back_m").value
            ),
            projection_window_fwd_m=float(
                self.get_parameter("projection_window_fwd_m").value
            ),
            projection_reacquire_dist_m=float(
                self.get_parameter("projection_reacquire_dist_m").value
            ),
            projection_direction_gate_cos=_direction_gate_cos(
                float(
                    self.get_parameter("projection_direction_gate_deg").value
                )
            ),
        )
        # Carry the station forward so the next tick's search stays on this leg.
        if decision.projection is not None:
            self._proj_prev_s = decision.projection.s
        # P0-2: advance the hysteresis latch. Stamp the rising edge so the
        # min-off dwell measures from the trip, not from every tripped tick.
        if decision.xtrack_tripped and not self._xtrack_tripped:
            self._xtrack_trip_mono = now_mono
        self._xtrack_tripped = decision.xtrack_tripped
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
            # G4: on a confirmed dwell-complete (OFF-confirmed advance), tell the
            # RPP so it releases the hold. Only under the handshake — off it, the
            # RPP uses its own fixed point_hold_s timer and ignores point_done.
            if pu.completed_index >= 0 and rpp_at_point_gate is not None:
                self._publish_point_done(pu.completed_index)
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
            elif decision.event == "terminal_off":
                self.get_logger().info("Spray OFF at path end (terminal shutoff)")
        self._last_distance_event = decision.event

        if decision.geometry_desired and not decision.safety_ok:
            if decision.safety_reason != self._last_safety_block_reason:
                self.get_logger().warn(
                    f"Safety blocked spray: {decision.safety_reason}"
                )
                self._last_safety_block_reason = decision.safety_reason
        elif decision.safety_ok:
            self._last_safety_block_reason = ""

        desired = decision.desired
        # B4 mission-level fail-closed watchdog (default distance-aware path):
        # override geometry OFF when the RPP has stopped asserting a MARK. This
        # is the safety net behind the terminal shutoff — it also covers a dead
        # RPP and any dual-projection drift past mission end.
        if desired and self._active_heartbeat_forces_off():
            desired = False
            self.get_logger().warn(
                "Spray forced OFF: /spray/active stale/inactive beyond "
                "active_timeout_s (mission-level fail-closed watchdog)",
                throttle_duration_sec=5.0,
            )
        self._set_auto_desired(desired, source="distance")

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
        # B5: an actively-tracking state is the positive evidence the run has
        # started. Latch it for the rest of this /path so a later pivot/stop
        # does not re-arm the gate (the pivot gate handles those separately).
        if self._segment_state in _SEGMENT_TRACKING_STATES:
            self._tracking_seen_since_path_load = True

    def _rpp_progress_cb(self, msg: String) -> None:
        """Cache the latest RPP progress (G2). Lenient parse never raises."""
        self._rpp_progress = ProgressMsg.from_json(msg.data)
        self._rpp_progress_recv_time = self.get_clock().now()

    def _milestone_cb(self, msg: String) -> None:
        """G4: record the RPP AT_POINT milestone (the point-handshake trigger).

        Lenient parse never raises. Only AT_POINT is retained — it is the proof
        that the rover is precisely stopped on that point, which gates the point
        dwell FSM (design §5). Other milestones are observability only.
        """
        m = MilestoneMsg.from_json(msg.data)
        if m.event == MilestoneEvent.AT_POINT and m.index >= 0:
            self._at_point_index = m.index
            self._at_point_seq = m.seq

    def _point_handshake_gate(self, target_index: int):
        """G4: arrival authority for the point meter's current target.

        Returns None (frozen — PointMeter self-arrives from pose) unless the
        handshake is active: consume_rpp_progress set AND mode is point. Then it
        returns True iff the RPP has emitted AT_POINT for this target index (the
        rover is confirmed on the point), else False (hold, don't guess).

        Index alignment: RPP's milestone index is the must-hit rank within its
        active run; the meter's index is the mission-order must-hit index. For a
        single-run point mission (the common case — must-hit points are transit
        stops, so /path is one spray-OFF run) these coincide. The RPP's
        point_hold_max_s backstop makes any divergence safe, never wedged.
        """
        if not bool(self.get_parameter("consume_rpp_progress").value):
            return None
        if self._session_mode != "point":
            return None
        if self._at_point_index == target_index:
            return True
        # Re-sync fallback (design §6, G4.4): a dropped AT_POINT milestone is
        # recovered from the fresh /rpp/progress phase — DWELL_HOLD/AT_POINT at
        # this point index is the same "rover stopped on the point" proof.
        pm = self._rpp_progress
        if pm is not None and self._rpp_progress_recv_time is not None:
            age_s = (
                self.get_clock().now() - self._rpp_progress_recv_time
            ).nanoseconds * 1e-9
            timeout_s = max(0.0, float(self.get_parameter("progress_timeout_s").value))
            if age_s <= timeout_s and int(pm.point_index) == target_index:
                try:
                    ph = MissionPhase(int(pm.phase))
                except ValueError:
                    ph = MissionPhase.IDLE
                if ph in (MissionPhase.AT_POINT, MissionPhase.DWELL_HOLD):
                    return True
        return False

    def _publish_point_done(self, index: int) -> None:
        """G4: tell the RPP a point's dwell is done (OFF-confirmed) so it advances."""
        self._point_done_seq += 1
        msg = String()
        msg.data = PointDoneMsg(
            point_index=int(index),
            seq=self._point_done_seq,
            done=True,
            reason="dwell_complete",
        ).to_json()
        self._point_done_pub.publish(msg)
        self.get_logger().info(f"point handshake: /spray/point_done {index} (dwell complete)")

    def _rpp_boundary_inputs(self, mode: str):
        """Resolve the boundary source for this tick (G2.1/G2.2).

        Returns (rpp_in_mark, rpp_boundary_kind, rpp_dist_m) to pass into
        `_make_spray_decision`, or (None, "", inf) to fall back to the local
        /path projection (frozen behaviour). RPP is used only when:
          * consume_rpp_progress is set,
          * mode is continuous (dash meters arc-length locally; point uses the
            handshake, not boundary sourcing), and
          * a progress message has arrived within progress_timeout_s.
        A source switch (rpp↔path↔off) is logged once, rate-limited.
        """
        off = (None, "", float("inf"))
        if not bool(self.get_parameter("consume_rpp_progress").value):
            self._note_rpp_source("off")
            return off
        if mode != "continuous":
            # Only continuous marks are boundary-sourced in G2.
            self._note_rpp_source("off")
            return off
        if self._rpp_progress is None or self._rpp_progress_recv_time is None:
            self._note_rpp_source("path", reason="no progress yet")
            return off
        age_s = (
            self.get_clock().now() - self._rpp_progress_recv_time
        ).nanoseconds * 1e-9
        timeout_s = max(0.0, float(self.get_parameter("progress_timeout_s").value))
        if age_s > timeout_s:
            self._note_rpp_source("path", reason=f"progress stale ({age_s:.2f}s)")
            return off
        pm = self._rpp_progress
        try:
            phase = MissionPhase(int(pm.phase))
        except ValueError:
            phase = MissionPhase.IDLE
        in_mark = phase in _RPP_IN_MARK_PHASES
        kind = _rpp_kind_for(in_mark, pm.next_boundary)
        dist = float(pm.dist_to_next_boundary_m)
        self._note_rpp_source("rpp")
        return in_mark, kind, dist

    def _note_rpp_source(self, source: str, reason: str = "") -> None:
        """Log a boundary-source change once (rate-limited), not every tick."""
        if source == self._rpp_source:
            return
        self._rpp_source = source
        if source == "rpp":
            self.get_logger().info("spray boundary source: RPP /rpp/progress")
        elif source == "path":
            self.get_logger().warn(
                f"spray boundary source: /path fallback ({reason})",
                throttle_duration_sec=5.0,
            )

    def _gps_cb(self, msg: GPSRAW) -> None:
        """Track GPS fix quality for the Phase B RTK gate (§7.6)."""
        prev = self._gps_fix_type
        self._gps_fix_type = int(msg.fix_type)
        self._gps_recv_time = self.get_clock().now()
        # GPSRAW.h_acc is uint32 MILLIMETRES (MAVLink GPS_RAW_INT). 0 is the
        # "not supplied" sentinel, not a perfect fix — keep it as None so the
        # gate can tell "good" from "unknown" (A14).
        try:
            h_acc_mm = int(msg.h_acc)
            self._gps_h_acc_m = (h_acc_mm * 1e-3) if h_acc_mm > 0 else None
        except (AttributeError, TypeError, ValueError):
            self._gps_h_acc_m = None
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
        if self._gps_fix_type < min_fix:
            return True, False, name
        # A14 accuracy half. Only enforced when the receiver actually reported
        # an accuracy; an unreported one leaves the pre-A14 behaviour intact.
        max_hrms = float(self.get_parameter("spray_max_hrms_m").value)
        h_acc = self._gps_h_acc_m
        if max_hrms > 0.0 and h_acc is not None and h_acc > max_hrms:
            return True, False, f"{name}_hacc_{h_acc:.3f}m"
        return True, True, name

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
            # B3(b): fail OPEN on staleness *immediately*. A latched CORNER_ALIGN
            # with no fresh message would otherwise keep this branch re-latching
            # the gate every tick (and, before B3(a), the smooth profile went
            # silent for the whole MARK span — 1.0 s of unpainted line). Clear
            # the stale state so the gate releases and stays released until a
            # genuinely fresh message arrives.
            self._segment_state = None
            self._segment_state_recv_time = None
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
        # B5: require positive evidence the run has started before honouring
        # geometry. On path load the rover is parked ON vertex 0 (which carries
        # the spray bit), already armed + OFFBOARD, so geometry alone would open
        # the valve the instant /path lands (a paint blob at the start vertex).
        # Hold until the RPP reports an actively-tracking state SINCE this /path
        # load. This gate dominates the pivot gate: even if no CORNER_ALIGN is
        # ever seen, spray stays OFF until real tracking begins. Point mode is
        # exempt — it dwells at a standstill and never "tracks" a segment, and
        # it has its own arrival/dwell FSM gating.
        if (
            self._session_mode != "point"
            and not getattr(self, "_tracking_seen_since_path_load", True)
        ):
            return False, "awaiting tracking"
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
            # G5: mirror the RPP mission phase so the app can show the "Next
            # point" button while the RPP holds in WAIT_OPERATOR (manual gate).
            # rpp_point_index is the point the RPP is holding — the expect_index
            # the frontend echoes back on /api/spray/point/advance.
            pm = self._rpp_progress
            if pm is not None:
                try:
                    ph = MissionPhase(int(pm.phase))
                except ValueError:
                    ph = MissionPhase.IDLE
                mode_state["rpp_phase"] = int(ph)
                mode_state["rpp_phase_name"] = ph.name
                mode_state["rpp_point_index"] = int(pm.point_index)
                mode_state["wait_operator"] = ph == MissionPhase.WAIT_OPERATOR
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
