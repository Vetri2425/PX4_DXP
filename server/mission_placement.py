"""Pure mission placement helpers."""

from __future__ import annotations

import math
from collections.abc import Iterable

from config import (
    GLOBAL_POSITION_STALE_MS,
    GPS_FIX_STALE_MS,
    POSE_GLOBAL_MAX_SKEW_MS,
    POSE_STALE_MS,
)
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from spray_config import GpsSurveyedSafetyParams  # noqa: E402
from path_engine.ned import apply_affine_transform, latlon_to_ned

LOCAL_NED = "LOCAL_NED"
GPS_SURVEYED = "GPS_SURVEYED"

# mavros_msgs/msg/GPSRAW.msg mirrors MAVLink GPS_FIX_TYPE and defines
# GPS_FIX_TYPE_RTK_FIXED as 6. This is not sensor_msgs/NavSatFix.status.
GPS_FIX_TYPE_RTK_FIXED = 6


class PlacementError(ValueError):
    """Raised when a mission cannot be placed safely in the live local frame."""


def align_design_points(
    source_points: Iterable[tuple[float, float]], alignment_metadata: dict | None
) -> list[tuple[float, float]]:
    """Apply the path engine's exact design-to-anchor-NED affine transform."""
    meta = alignment_metadata or {}
    if not meta.get("method"):
        raise PlacementError("DESIGN Point coordinates require alignment metadata")
    try:
        scale = float(meta["scale"])
        theta = math.radians(float(meta["rotation_deg"]))
        offset_n = float(meta["offset_n"])
        offset_e = float(meta["offset_e"])
    except (KeyError, TypeError, ValueError) as exc:
        raise PlacementError("Point alignment metadata is incomplete") from exc
    return [
        apply_affine_transform(_finite_pair(point, "design point"), scale, theta, offset_n, offset_e)
        for point in source_points
    ]


def _finite_pair(value, label: str) -> tuple[float, float]:
    try:
        if value is None or len(value) != 2:
            raise PlacementError(f"{label} is missing or invalid")
        pair = (float(value[0]), float(value[1]))
    except PlacementError:
        raise
    except (TypeError, ValueError, IndexError):
        raise PlacementError(f"{label} is missing or invalid")
    if not all(math.isfinite(v) for v in pair):
        raise PlacementError(f"{label} contains non-finite values")
    return pair


def _fresh_age(state: dict, key: str, limit_ms: float, label: str) -> float:
    raw = state.get(key)
    if raw is None:
        raise PlacementError(f"{label} freshness is unavailable")
    try:
        age_ms = float(raw)
    except (TypeError, ValueError):
        raise PlacementError(f"{label} freshness is invalid")
    if not math.isfinite(age_ms) or age_ms < 0.0 or age_ms > limit_ms:
        raise PlacementError(
            f"{label} is stale ({age_ms:.0f} ms > {limit_ms:.0f} ms)"
        )
    return age_ms


def resolve_surveyed_points(
    source_points: Iterable[tuple[float, float]],
    origin_gps: tuple[float, float] | None,
    state: dict,
    *,
    safety: GpsSurveyedSafetyParams | None = None,
) -> tuple[list[tuple[float, float]], tuple[float, float]]:
    """Translate anchor-relative NED points into the current PX4 local-NED frame."""
    limits = safety or GpsSurveyedSafetyParams()
    anchor_lat, anchor_lon = _finite_pair(origin_gps, "survey GPS anchor")
    if not (-90.0 <= anchor_lat <= 90.0 and -180.0 <= anchor_lon <= 180.0):
        raise PlacementError("survey GPS anchor is outside valid latitude/longitude bounds")

    if not state.get("connected", False):
        raise PlacementError("FCU disconnected")

    if not state.get("pose_received", False):
        raise PlacementError("local pose has not been received")
    if not state.get("global_position_received", False):
        raise PlacementError("fused global position has not been received")
    if not state.get("gps_fix_received", False):
        raise PlacementError("GPS fix information has not been received")

    _fresh_age(state, "local_pose_age_ms", limits.local_pose_max_age_ms, "local pose")
    _fresh_age(
        state,
        "global_position_age_ms",
        limits.global_position_max_age_ms,
        "fused global position",
    )
    _fresh_age(state, "gps_fix_age_ms", limits.gps_fix_max_age_ms, "GPS fix information")

    skew_raw = state.get("pose_global_skew_ms")
    if skew_raw is None:
        raise PlacementError("local/global position receive-time skew is unavailable")
    try:
        skew_ms = float(skew_raw)
    except (TypeError, ValueError):
        raise PlacementError("local/global position receive-time skew is invalid")
    if not math.isfinite(skew_ms) or skew_ms < 0.0 or skew_ms > limits.max_pose_global_skew_ms:
        raise PlacementError(
            "local/global position samples are not sufficiently aligned "
            f"({skew_ms:.0f} ms > {limits.max_pose_global_skew_ms:.0f} ms)"
        )

    try:
        fix_type = int(state.get("gps_fix"))
    except (TypeError, ValueError):
        raise PlacementError("GPS fix type is invalid")
    if fix_type < limits.required_fix_type:
        raise PlacementError(
            f"GPS fix_type={fix_type} is below required ({limits.required_fix_type})"
        )

    rover_local_n, rover_local_e = _finite_pair(
        (state.get("pos_n"), state.get("pos_e")), "rover local position"
    )
    rover_lat, rover_lon = _finite_pair(
        (state.get("lat"), state.get("lon")), "rover fused global position"
    )
    if not (-90.0 <= rover_lat <= 90.0 and -180.0 <= rover_lon <= 180.0):
        raise PlacementError("rover fused global position is outside valid bounds")

    delta_n, delta_e = latlon_to_ned(
        anchor_lat,
        anchor_lon,
        rover_lat,
        rover_lon,
    )
    translation = (rover_local_n + delta_n, rover_local_e + delta_e)
    if not all(math.isfinite(v) for v in translation):
        raise PlacementError("survey translation contains non-finite values")

    resolved: list[tuple[float, float]] = []
    for point in source_points:
        n, e = _finite_pair(point, "mission waypoint")
        resolved_point = (n + translation[0], e + translation[1])
        if not all(math.isfinite(v) for v in resolved_point):
            raise PlacementError("resolved mission waypoint contains non-finite values")
        resolved.append(resolved_point)

    return resolved, translation
