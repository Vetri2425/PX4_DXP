"""ROS-independent spray path model and distance-aware decision engine."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

TRANSIT_TO_MARK = "TRANSIT_TO_MARK"
MARK_TO_TRANSIT = "MARK_TO_TRANSIT"


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

    tangent_n: float = 0.0
    tangent_e: float = 0.0
    projection_jump_m: float = 0.0
    backward_jump_m: float = 0.0
    # 1.0 = on-segment; 0.0 = x-track at ambiguity-distance threshold (not max-xtrack gate).
    ambiguity_clearance_confidence: float = 1.0
    ambiguous: bool = False
    ambiguity_reason: str = ""

    @property
    def confidence(self) -> float:
        """Compatibility alias for ambiguity_clearance_confidence."""
        return self.ambiguity_clearance_confidence


@dataclass
class SprayProjectionState:
    valid: bool = False
    segment_index: int = 0
    s: float = 0.0

    def reset(self) -> None:
        self.valid = False
        self.segment_index = 0
        self.s = 0.0


@dataclass(frozen=True)
class BoundaryContext:
    previous_boundary: SprayBoundary | None
    next_boundary: SprayBoundary | None
    current_run_start_s: float
    current_run_end_s: float
    current_run_length_m: float
    next_run_length_m: float


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
    target_flow: float = 0.0
    target_pwm: float = 0.0
    actuator_value: float = 0.0
    along_track_speed_mps: float = 0.0
    cross_track_speed_mps: float = 0.0
    velocity_heading_error_deg: float = 0.0
    raw_on_lead_m: float = 0.0
    bounded_on_lead_m: float = 0.0
    raw_off_lead_m: float = 0.0
    bounded_off_lead_m: float = 0.0
    lead_clamped: bool = False
    lead_block_reason: str = ""
    current_run_remaining_m: float = 0.0
    current_run_length_m: float = 0.0
    next_run_length_m: float = 0.0
    flow_mode: str = "mapped"
    raw_target_flow: float = 0.0
    target_paint_density: float = 0.0
    flow_clamp_reason: str = ""
    flow_under_capacity: bool = False
    command_pwm: float = 0.0
    pwm_ramp_limited: bool = False


@dataclass(frozen=True)
class _SegmentCandidate:
    segment_index: int
    t: float
    proj_n: float
    proj_e: float
    s: float
    xtrack_error_m: float
    current_flag: bool
    tangent_n: float
    tangent_e: float


def build_path_model(
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

    return SprayPathModel(clean_points, clean_flags, cumulative_s, boundaries)


def yaw_ned_from_enu_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_enu = math.atan2(siny_cosp, cosy_cosp)
    return (math.pi / 2.0 - yaw_enu + math.pi) % (2.0 * math.pi) - math.pi


def pose_to_ned(pose_msg) -> tuple[float, float, float]:
    north = float(pose_msg.pose.position.y)
    east = float(pose_msg.pose.position.x)
    yaw_ned = yaw_ned_from_enu_quaternion(pose_msg.pose.orientation)
    return north, east, yaw_ned


def nozzle_position_ned(
    pose_n: float,
    pose_e: float,
    yaw_ned: float,
    forward_offset_m: float,
    lateral_offset_m: float,
) -> tuple[float, float]:
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


def _segment_candidate(
    model: SprayPathModel,
    segment_index: int,
    point_n: float,
    point_e: float,
) -> _SegmentCandidate:
    a_n, a_e = model.points[segment_index]
    b_n, b_e = model.points[segment_index + 1]
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

    if seg_len > 1e-9:
        tangent_n = d_n / seg_len
        tangent_e = d_e / seg_len
    else:
        tangent_n = 0.0
        tangent_e = 0.0

    dist = math.hypot(point_n - proj_n, point_e - proj_e)
    current_flag = (
        model.flags[segment_index + 1] if t >= 1.0 - 1e-12 else model.flags[segment_index]
    )
    return _SegmentCandidate(
        segment_index=segment_index,
        t=t,
        proj_n=proj_n,
        proj_e=proj_e,
        s=model.cumulative_s[segment_index] + t * seg_len,
        xtrack_error_m=dist,
        current_flag=current_flag,
        tangent_n=tangent_n,
        tangent_e=tangent_e,
    )


def _candidate_to_projection(
    candidate: _SegmentCandidate,
    *,
    projection_jump_m: float = 0.0,
    backward_jump_m: float = 0.0,
    ambiguity_clearance_confidence: float = 1.0,
    ambiguous: bool = False,
    ambiguity_reason: str = "",
) -> SprayProjection:
    return SprayProjection(
        segment_index=candidate.segment_index,
        t=candidate.t,
        proj_n=candidate.proj_n,
        proj_e=candidate.proj_e,
        s=candidate.s,
        xtrack_error_m=candidate.xtrack_error_m,
        current_flag=candidate.current_flag,
        tangent_n=candidate.tangent_n,
        tangent_e=candidate.tangent_e,
        projection_jump_m=projection_jump_m,
        backward_jump_m=backward_jump_m,
        ambiguity_clearance_confidence=ambiguity_clearance_confidence,
        ambiguous=ambiguous,
        ambiguity_reason=ambiguity_reason,
    )


def project_onto_path(
    model: SprayPathModel,
    point_n: float,
    point_e: float,
    *,
    previous_segment_index: int | None = None,
    previous_s: float | None = None,
    max_projection_jump_m: float = 0.50,
    max_backward_jump_m: float = 0.10,
    ambiguity_distance_m: float = 0.03,
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

    candidates = [
        _segment_candidate(model, i, point_n, point_e)
        for i in range(len(model.points) - 1)
    ]

    if previous_segment_index is not None:
        windowed = [
            c
            for c in candidates
            if abs(c.segment_index - previous_segment_index) <= 1
        ]
        if windowed:
            candidates = windowed

    if previous_s is not None:
        filtered: list[_SegmentCandidate] = []
        for candidate in candidates:
            forward_jump = candidate.s - previous_s
            if forward_jump < -max_backward_jump_m:
                continue
            if abs(forward_jump) > max_projection_jump_m:
                continue
            filtered.append(candidate)
        if not filtered:
            return _candidate_to_projection(
                min(candidates, key=lambda c: (c.xtrack_error_m, c.segment_index)),
                projection_jump_m=abs(
                    min(candidates, key=lambda c: (c.xtrack_error_m, c.segment_index)).s
                    - previous_s
                ),
                backward_jump_m=max(
                    0.0,
                    previous_s
                    - min(
                        candidates, key=lambda c: (c.xtrack_error_m, c.segment_index)
                    ).s,
                ),
                ambiguity_clearance_confidence=0.0,
                ambiguous=True,
                ambiguity_reason="projection jump rejected",
            )
        candidates = filtered

    candidates.sort(key=lambda c: (c.xtrack_error_m, c.segment_index))

    if previous_segment_index is not None and previous_s is not None:
        tied = [
            c
            for c in candidates
            if abs(c.xtrack_error_m - candidates[0].xtrack_error_m) <= 1e-12
        ]
        if len(tied) > 1:
            progress_consistent = [
                c
                for c in tied
                if c.segment_index == previous_segment_index
                or (
                    c.segment_index == previous_segment_index + 1
                    and c.s >= previous_s - 1e-9
                )
            ]
            if progress_consistent:
                candidates = sorted(
                    progress_consistent,
                    key=lambda c: (c.xtrack_error_m, c.segment_index),
                )

    best = candidates[0]
    projection_jump_m = abs(best.s - previous_s) if previous_s is not None else 0.0
    backward_jump_m = (
        max(0.0, previous_s - best.s) if previous_s is not None else 0.0
    )

    close = [
        c
        for c in candidates
        if abs(c.xtrack_error_m - best.xtrack_error_m) <= ambiguity_distance_m
        and abs(c.s - best.s) > max_projection_jump_m
    ]
    if len(close) >= 1:
        return _candidate_to_projection(
            best,
            projection_jump_m=projection_jump_m,
            backward_jump_m=backward_jump_m,
            ambiguity_clearance_confidence=0.0,
            ambiguous=True,
            ambiguity_reason="multiple projections with similar xtrack",
        )

    clearance_confidence = max(
        0.0,
        min(1.0, 1.0 - best.xtrack_error_m / max(ambiguity_distance_m, 1e-9)),
    )
    return _candidate_to_projection(
        best,
        projection_jump_m=projection_jump_m,
        backward_jump_m=backward_jump_m,
        ambiguity_clearance_confidence=clearance_confidence,
    )


def next_boundary(
    model: SprayPathModel,
    current_s: float,
    current_flag: bool,
) -> Optional[SprayBoundary]:
    wanted = MARK_TO_TRANSIT if current_flag else TRANSIT_TO_MARK
    for boundary in model.boundaries:
        if boundary.kind == wanted and boundary.s > current_s + 1e-9:
            return boundary
    return None


def boundary_context(
    model: SprayPathModel,
    current_s: float,
    current_flag: bool,
) -> BoundaryContext:
    previous_boundary: SprayBoundary | None = None
    next_boundary_obj: SprayBoundary | None = None
    for boundary in model.boundaries:
        if boundary.s < current_s - 1e-9:
            previous_boundary = boundary
        elif boundary.s > current_s + 1e-9:
            next_boundary_obj = boundary
            break

    total_len = model.cumulative_s[-1] if model.cumulative_s else 0.0
    if current_flag:
        start_kind = TRANSIT_TO_MARK
        end_kind = MARK_TO_TRANSIT
    else:
        start_kind = MARK_TO_TRANSIT
        end_kind = TRANSIT_TO_MARK

    current_run_start_s = 0.0
    for boundary in model.boundaries:
        if boundary.kind == start_kind and boundary.s <= current_s + 1e-9:
            current_run_start_s = boundary.s

    current_run_end_s = total_len
    for boundary in model.boundaries:
        if boundary.kind == end_kind and boundary.s > current_s + 1e-9:
            current_run_end_s = boundary.s
            break

    current_run_length_m = max(0.0, current_run_end_s - current_run_start_s)

    next_run_length_m = 0.0
    if current_run_end_s < total_len - 1e-9:
        next_start = current_run_end_s
        next_end = total_len
        next_start_kind = end_kind
        next_end_kind = start_kind
        for boundary in model.boundaries:
            if boundary.kind == next_end_kind and boundary.s > next_start + 1e-9:
                next_end = boundary.s
                break
        next_run_length_m = max(0.0, next_end - next_start)

    return BoundaryContext(
        previous_boundary=previous_boundary,
        next_boundary=next_boundary_obj,
        current_run_start_s=current_run_start_s,
        current_run_end_s=current_run_end_s,
        current_run_length_m=current_run_length_m,
        next_run_length_m=next_run_length_m,
    )


def _lead_distances(
    along_track_speed_mps: float,
    *,
    solenoid_open_delay_s: float,
    solenoid_close_delay_s: float,
    on_overspray_margin_m: float,
    off_overspray_margin_m: float,
    max_lead_distance_m: float,
) -> tuple[float, float, float, float, bool]:
    raw_on_lead = (
        max(0.0, along_track_speed_mps) * solenoid_open_delay_s + on_overspray_margin_m
    )
    raw_off_lead = max(
        0.0,
        max(0.0, along_track_speed_mps) * solenoid_close_delay_s - off_overspray_margin_m,
    )
    bounded_on_lead = min(raw_on_lead, max_lead_distance_m)
    bounded_off_lead = min(raw_off_lead, max_lead_distance_m)
    lead_clamped = (
        bounded_on_lead < raw_on_lead - 1e-12 or bounded_off_lead < raw_off_lead - 1e-12
    )
    return raw_on_lead, bounded_on_lead, raw_off_lead, bounded_off_lead, lead_clamped


def _validate_lead_geometry(
    *,
    projection: SprayProjection,
    ctx: BoundaryContext,
    boundary: SprayBoundary | None,
    bounded_on_lead: float,
    bounded_off_lead: float,
    min_on_distance_m: float,
    min_off_distance_m: float,
) -> tuple[bool, str]:
    if (
        boundary is not None
        and boundary.kind == TRANSIT_TO_MARK
        and not projection.current_flag
    ):
        available_pre_run = ctx.current_run_length_m
        if bounded_on_lead > available_pre_run + 1e-9:
            return False, "ON lead exceeds available TRANSIT/PRE run"
        if ctx.next_run_length_m < min_on_distance_m - 1e-9:
            return False, "upcoming MARK shorter than min_on_distance_m"
        if bounded_on_lead > ctx.next_run_length_m + 1e-9:
            return False, "ON lead spans beyond upcoming MARK run"

    if (
        projection.current_flag
        and bounded_off_lead > 0.0
        and ctx.current_run_length_m < min_on_distance_m + bounded_off_lead - 1e-9
    ):
        return False, "OFF lead eliminates required minimum MARK distance"

    if (
        boundary is not None
        and boundary.kind == MARK_TO_TRANSIT
        and projection.current_flag
        and ctx.next_run_length_m > 1e-9
        and ctx.next_run_length_m < min_off_distance_m - 1e-9
    ):
        return False, "upcoming OFF gap shorter than min_off_distance_m"

    return True, ""


def apply_distance_hysteresis(
    geometry_desired: bool,
    current_s: float,
    *,
    last_transition_s: float | None,
    last_geometry_state: bool,
    min_on_distance_m: float,
    min_off_distance_m: float,
    safety_ok: bool,
    progress_forward: bool,
) -> bool:
    if not safety_ok:
        return False
    if last_transition_s is None:
        return geometry_desired

    distance_since = current_s - last_transition_s
    if not progress_forward or distance_since < -1e-9:
        return last_geometry_state

    if last_geometry_state:
        if not geometry_desired and distance_since < min_on_distance_m:
            return True
    elif geometry_desired and distance_since < min_off_distance_m:
        return False
    return geometry_desired


def ramp_pwm(
    target_pwm: float,
    previous_pwm: float,
    *,
    max_pwm_change_per_s: float,
    dt_s: float,
) -> tuple[float, bool]:
    if dt_s <= 0.0:
        return target_pwm, False
    max_delta = max_pwm_change_per_s * dt_s
    low = previous_pwm - max_delta
    high = previous_pwm + max_delta
    command_pwm = max(low, min(high, target_pwm))
    limited = abs(command_pwm - target_pwm) > 1e-9
    return command_pwm, limited


def make_spray_decision(
    model: Optional[SprayPathModel],
    nozzle_n: Optional[float],
    nozzle_e: Optional[float],
    vel_n: float,
    vel_e: float,
    safety_ok: bool,
    safety_reason: str,
    *,
    solenoid_open_delay_s: float,
    solenoid_close_delay_s: float,
    on_overspray_margin_m: float,
    off_overspray_margin_m: float,
    max_xtrack_error_m: float,
    max_along_track_heading_error_deg: float = 30.0,
    max_cross_track_speed_mps: float = 0.10,
    max_reverse_speed_tolerance_mps: float = 0.03,
    max_projection_jump_m: float = 0.50,
    max_backward_projection_jump_m: float = 0.10,
    projection_ambiguity_distance_m: float = 0.03,
    max_lead_distance_m: float = 0.50,
    min_on_distance_m: float = 0.05,
    min_off_distance_m: float = 0.05,
    min_spray_speed_mps: float = 0.05,
    max_spray_speed_mps: float = 1.0,
    projection_state: Optional[SprayProjectionState] = None,
    flow_mode: str = "mapped",
    target_paint_density: float = 1.0,
    min_target_flow: float = 0.0,
    max_target_flow: float = 1.0,
    low_speed_anti_puddle_behavior: str = "block",
    high_speed_underflow_behavior: str = "block",
    speed_pwm_table_max_speed: float = 1.0,
    dwell_active: bool = False,
) -> SprayDecision:
    projection: Optional[SprayProjection] = None
    boundary: Optional[SprayBoundary] = None
    distance_to_boundary = float("inf")
    geometry_desired = False
    event = "WAITING_FOR_BOUNDARY"
    along_track_speed = 0.0
    cross_track_speed = 0.0
    velocity_heading_error_deg = 0.0
    raw_on_lead = bounded_on_lead = raw_off_lead = bounded_off_lead = 0.0
    lead_clamped = False
    lead_block_reason = ""
    current_run_remaining_m = 0.0
    current_run_length_m = 0.0
    next_run_length_m = 0.0
    raw_target_flow = 0.0
    target_flow = 0.0
    flow_clamp_reason = ""
    flow_under_capacity = False

    prev_seg = projection_state.segment_index if projection_state and projection_state.valid else None
    prev_s = projection_state.s if projection_state and projection_state.valid else None

    if model is not None and nozzle_n is not None and nozzle_e is not None:
        projection = project_onto_path(
            model,
            nozzle_n,
            nozzle_e,
            previous_segment_index=prev_seg,
            previous_s=prev_s,
            max_projection_jump_m=max_projection_jump_m,
            max_backward_jump_m=max_backward_projection_jump_m,
            ambiguity_distance_m=projection_ambiguity_distance_m,
        )

    if projection is not None and projection.ambiguous:
        safety_ok = False
        safety_reason = projection.ambiguity_reason or "ambiguous projection"
        event = "PROJECTION_AMBIGUOUS"
    elif projection is not None:
        along_track_speed = vel_n * projection.tangent_n + vel_e * projection.tangent_e
        cross_track_speed = (
            -vel_n * projection.tangent_e + vel_e * projection.tangent_n
        )
        velocity_heading_error_deg = math.degrees(
            math.atan2(abs(cross_track_speed), max(along_track_speed, 1e-9))
        )

        if along_track_speed < -max_reverse_speed_tolerance_mps:
            safety_ok = False
            safety_reason = f"reverse along-track speed {along_track_speed:.3f} m/s"
            event = "REVERSE_MOTION"
        elif abs(cross_track_speed) > max_cross_track_speed_mps:
            safety_ok = False
            safety_reason = f"cross-track speed {abs(cross_track_speed):.3f} m/s"
            event = "SIDEWAYS_MOTION"
        elif velocity_heading_error_deg > max_along_track_heading_error_deg:
            safety_ok = False
            safety_reason = (
                f"velocity heading error {velocity_heading_error_deg:.1f} deg"
            )
            event = "HEADING_DISAGREEMENT"
        elif projection.xtrack_error_m > max_xtrack_error_m:
            safety_ok = False
            safety_reason = (
                f"xtrack error {projection.xtrack_error_m:.3f}m "
                f"> {max_xtrack_error_m:.3f}m"
            )
            event = "XTRACK_BLOCKED"

        ctx = boundary_context(model, projection.s, projection.current_flag)
        boundary = next_boundary(model, projection.s, projection.current_flag)
        geometry_desired = projection.current_flag
        current_run_remaining_m = max(0.0, ctx.current_run_end_s - projection.s)
        current_run_length_m = ctx.current_run_length_m
        next_run_length_m = ctx.next_run_length_m

        (
            raw_on_lead,
            bounded_on_lead,
            raw_off_lead,
            bounded_off_lead,
            lead_clamped,
        ) = _lead_distances(
            along_track_speed,
            solenoid_open_delay_s=solenoid_open_delay_s,
            solenoid_close_delay_s=solenoid_close_delay_s,
            on_overspray_margin_m=on_overspray_margin_m,
            off_overspray_margin_m=off_overspray_margin_m,
            max_lead_distance_m=max_lead_distance_m,
        )

        if boundary is not None:
            distance_to_boundary = boundary.s - projection.s
            lead_ok, lead_reason = _validate_lead_geometry(
                projection=projection,
                ctx=ctx,
                boundary=boundary,
                bounded_on_lead=bounded_on_lead,
                bounded_off_lead=bounded_off_lead,
                min_on_distance_m=min_on_distance_m,
                min_off_distance_m=min_off_distance_m,
            )
            if not lead_ok:
                safety_ok = False
                safety_reason = lead_reason
                lead_block_reason = lead_reason
                event = "LEAD_GEOMETRY_UNSAFE"
            elif safety_ok:
                if (
                    not projection.current_flag
                    and boundary.kind == TRANSIT_TO_MARK
                    and distance_to_boundary <= bounded_on_lead + 1e-9
                ):
                    geometry_desired = True
                    event = "ON_EARLY"
                elif (
                    projection.current_flag
                    and boundary.kind == MARK_TO_TRANSIT
                    and distance_to_boundary <= bounded_off_lead + 1e-9
                ):
                    geometry_desired = False
                    event = "OFF_EARLY"
                else:
                    event = "FOLLOW_FLAG"
        elif safety_ok:
            event = "FOLLOW_FLAG"

        if not dwell_active and safety_ok:
            if along_track_speed < min_spray_speed_mps:
                if low_speed_anti_puddle_behavior == "block":
                    if geometry_desired:
                        safety_ok = False
                        safety_reason = "below min along-track spray speed"
                        event = "LOW_SPEED_ANTI_PUDDLE"
            elif along_track_speed > max_spray_speed_mps:
                flow_under_capacity = True
                if high_speed_underflow_behavior == "block":
                    safety_ok = False
                    safety_reason = "above max spray speed"
                    event = "HIGH_SPEED_UNDERFLOW"

    if flow_mode == "disabled":
        raw_target_flow = 0.0
        target_flow = 0.0
    else:
        raw_target_flow = target_paint_density * max(0.0, along_track_speed)
        target_flow = min(
            max(raw_target_flow, min_target_flow),
            max_target_flow,
        )
        if raw_target_flow < min_target_flow - 1e-12:
            flow_clamp_reason = "below_min"
        elif raw_target_flow > max_target_flow + 1e-12:
            flow_clamp_reason = "above_max"

        if along_track_speed > speed_pwm_table_max_speed + 1e-9:
            flow_under_capacity = True
            if high_speed_underflow_behavior == "block" and safety_ok:
                safety_ok = False
                safety_reason = "speed exceeds calibrated PWM table capacity"
                event = "HIGH_SPEED_UNDERFLOW"

    if not safety_ok:
        if event not in {
            "PROJECTION_AMBIGUOUS",
            "REVERSE_MOTION",
            "SIDEWAYS_MOTION",
            "HEADING_DISAGREEMENT",
            "LEAD_GEOMETRY_UNSAFE",
            "LOW_SPEED_ANTI_PUDDLE",
            "HIGH_SPEED_UNDERFLOW",
            "XTRACK_BLOCKED",
        }:
            event = "SAFETY_BLOCKED"

    desired = bool(geometry_desired and safety_ok)
    debug = [
        1.0 if model is not None else 0.0,
        float(along_track_speed),
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
        target_flow=target_flow,
        raw_target_flow=raw_target_flow,
        target_paint_density=target_paint_density,
        flow_clamp_reason=flow_clamp_reason,
        flow_under_capacity=flow_under_capacity,
        flow_mode=flow_mode,
        along_track_speed_mps=along_track_speed,
        cross_track_speed_mps=cross_track_speed,
        velocity_heading_error_deg=velocity_heading_error_deg,
        raw_on_lead_m=raw_on_lead,
        bounded_on_lead_m=bounded_on_lead,
        raw_off_lead_m=raw_off_lead,
        bounded_off_lead_m=bounded_off_lead,
        lead_clamped=lead_clamped,
        lead_block_reason=lead_block_reason,
        current_run_remaining_m=current_run_remaining_m,
        current_run_length_m=current_run_length_m,
        next_run_length_m=next_run_length_m,
    )