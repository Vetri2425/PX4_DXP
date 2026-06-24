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


def project_onto_path(
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


def make_spray_decision(
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
) -> SprayDecision:
    projection: Optional[SprayProjection] = None
    boundary: Optional[SprayBoundary] = None
    distance_to_boundary = float("inf")
    geometry_desired = False
    event = "WAITING_FOR_BOUNDARY"

    if model is not None and nozzle_n is not None and nozzle_e is not None:
        projection = project_onto_path(model, nozzle_n, nozzle_e)
    if projection is not None:
        boundary = next_boundary(model, projection.s, projection.current_flag)
        geometry_desired = projection.current_flag
        if projection.xtrack_error_m > max_xtrack_error_m:
            safety_ok = False
            safety_reason = (
                f"xtrack error {projection.xtrack_error_m:.3f}m "
                f"> {max_xtrack_error_m:.3f}m"
            )
        if boundary is not None:
            distance_to_boundary = boundary.s - projection.s
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
                event = "ON_EARLY"
            elif (
                projection.current_flag
                and boundary.kind == MARK_TO_TRANSIT
                and distance_to_boundary <= off_lead
            ):
                geometry_desired = False
                event = "OFF_EARLY"
            else:
                event = "FOLLOW_FLAG"
        else:
            event = "FOLLOW_FLAG"

    if not safety_ok:
        event = "SAFETY_BLOCKED"

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
