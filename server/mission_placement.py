"""Pure mission placement — survey-frame waypoints → live EKF local NED.

Re-derived for test/colinear-fix (Epic 2). Do not paste reference-branch
call sites blindly; the invariant is locked by unit tests.

Invariant
---------
Source waypoints ``P`` are NED metres relative to the survey GPS anchor.
At start, the rover is at local EKF ``L = (pos_n, pos_e)`` and at geodesic
position ``R_anchor`` relative to the same anchor:

    R_anchor = latlon_to_ned(rover_lat, rover_lon, anchor_lat, anchor_lon)
    P_live   = P + L - R_anchor

This is a uniform translation only — inter-waypoint deltas are preserved.
"""

from __future__ import annotations

import math
from collections.abc import Iterable

from config import (
    GLOBAL_POSITION_STALE_MS,
    GPS_FIX_STALE_MS,
    ORIGIN_REQUIRE_DECLARED,
    POSE_GLOBAL_MAX_SKEW_MS,
    POSE_STALE_MS,
)
from logging_setup import get_logger
from origin_health import (
    INCONSISTENT,
    NO_ORIGIN,
    OK,
    evaluate_origin_health,
)
from path_engine.ned import latlon_to_ned

log = get_logger("server.placement")

LOCAL_NED = "LOCAL_NED"
GPS_SURVEYED = "GPS_SURVEYED"

# mavros_msgs/GPSRAW fix_type — RTK_FIXED
GPS_FIX_TYPE_RTK_FIXED = 6


class PlacementError(ValueError):
    """Raised when a mission cannot be placed safely in the live local frame."""


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
) -> tuple[list[tuple[float, float]], tuple[float, float]]:
    """Translate anchor-relative NED points into the current PX4 local-NED frame."""
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

    _fresh_age(state, "local_pose_age_ms", POSE_STALE_MS, "local pose")
    _fresh_age(
        state,
        "global_position_age_ms",
        GLOBAL_POSITION_STALE_MS,
        "fused global position",
    )
    _fresh_age(state, "gps_fix_age_ms", GPS_FIX_STALE_MS, "GPS fix information")

    skew_raw = state.get("pose_global_skew_ms")
    if skew_raw is None:
        raise PlacementError("local/global position receive-time skew is unavailable")
    try:
        skew_ms = float(skew_raw)
    except (TypeError, ValueError):
        raise PlacementError("local/global position receive-time skew is invalid")
    if not math.isfinite(skew_ms) or skew_ms < 0.0 or skew_ms > POSE_GLOBAL_MAX_SKEW_MS:
        raise PlacementError(
            "local/global position samples are not sufficiently aligned "
            f"({skew_ms:.0f} ms > {POSE_GLOBAL_MAX_SKEW_MS:.0f} ms)"
        )

    try:
        fix_type = int(state.get("gps_fix"))
    except (TypeError, ValueError):
        raise PlacementError("GPS fix type is invalid")
    if fix_type < GPS_FIX_TYPE_RTK_FIXED:
        raise PlacementError(
            f"GPS fix_type={fix_type} is below RTK_FIXED ({GPS_FIX_TYPE_RTK_FIXED})"
        )

    # ── Preferred: the EKF's own declared local-frame origin ─────────────────
    # PX4 sets this once at first GPS fix and then holds it for the EKF session
    # (ekf_helper.cpp: routine GNSS position resets project THROUGH the existing
    # origin and move only the vehicle estimate). It is the same datum PX4's own
    # rover controller projects mission waypoints against, re-initialising its
    # MapProjection only when ref_timestamp changes.
    #
    # Using it makes the translation a pure function of (anchor, origin) — no
    # rover sample at all — so the same staged mission produces a BIT-IDENTICAL
    # path on every load. The live pose/global pair below cannot do that:
    # GLOBAL_POSITION_INT carries lat/lon as int32 degE7, quantising to ~1.1 cm
    # at this latitude, and the two topics are never sampled simultaneously
    # (the skew gate permits 100 ms, which is 3.5 cm at 0.35 m/s). Measured
    # consequence of the live pair: the same file published paths 0.39-1.53 cm
    # apart, in scattered directions, run to run.
    #
    # But "we have an origin" is NOT the same as "the origin is right". The
    # server caches it in process state, so a MAVROS restart or an FCU reboot
    # leaves a datum from a DEAD EKF session in place — measured 2026-07-27:
    # two runs placed 2.15 m and 2.25 m off the surveyed line, with no symptom
    # until the rover moved. Freshness is not sufficient either: the same rig
    # later held a present, recent origin that was still 0.91 m from the frame
    # PX4 was publishing. So the origin is MEASURED against the live frame
    # before it is used (origin_health.evaluate_origin_health), and anything
    # short of agreement is a refusal — never a quiet downgrade to the live-pair
    # fallback below, which would hide exactly this fault.
    health = evaluate_origin_health(state)
    if health.status == OK:
        origin_lat, origin_lon = health.declared
        translation = latlon_to_ned(anchor_lat, anchor_lon, origin_lat, origin_lon)
        if not all(math.isfinite(v) for v in translation):
            raise PlacementError(
                "survey translation through the EKF origin is non-finite "
                f"(anchor {anchor_lat}, {anchor_lon}; "
                f"origin {origin_lat}, {origin_lon})"
            )
        return _apply_translation(source_points, translation)

    if health.status != NO_ORIGIN or ORIGIN_REQUIRE_DECLARED:
        # INCONSISTENT / ORIGIN_INVALID / UNVERIFIABLE always refuse; NO_ORIGIN
        # refuses too unless the operator explicitly opted into the degraded
        # live-pair mode. The message carries the measured numbers so the
        # refusal is actionable rather than mysterious.
        log.error("surveyed placement refused: %s", health.detail)
        raise PlacementError(
            "EKF local-frame origin is not trustworthy — "
            + health.detail
            + " Placement refuses rather than silently displace the mission; "
            "check GET /api/health/origin and re-place once the origin is OK."
        )

    # ── Degraded, explicitly enabled (ROVER_ORIGIN_REQUIRE_DECLARED=0) ────────
    # Only reachable when no origin was ever declared. A single live pair is
    # self-consistent with the current frame — accurate — but not reproducible.
    log.warning(
        "no EKF local-frame origin (%s) and ROVER_ORIGIN_REQUIRE_DECLARED=0 — "
        "placing from the live pose/global pair; placement will vary ~1 cm run "
        "to run and is NOT reproducible", health.detail)

    # ── Fallback: single live pose/global pair (non-deterministic) ────────────
    rover_local_n, rover_local_e = _finite_pair(
        (state.get("pos_n"), state.get("pos_e")), "rover local position"
    )
    rover_lat, rover_lon = _finite_pair(
        (state.get("lat"), state.get("lon")), "rover fused global position"
    )
    if not (-90.0 <= rover_lat <= 90.0 and -180.0 <= rover_lon <= 180.0):
        raise PlacementError("rover fused global position is outside valid bounds")

    # Rover NED relative to survey anchor (clear 4-arg form — do not invert).
    r_anchor_n, r_anchor_e = latlon_to_ned(
        rover_lat, rover_lon, anchor_lat, anchor_lon
    )
    translation = (rover_local_n - r_anchor_n, rover_local_e - r_anchor_e)
    if not all(math.isfinite(v) for v in translation):
        raise PlacementError("survey translation contains non-finite values")

    return _apply_translation(source_points, translation)


def _apply_translation(
    source_points: Iterable[tuple[float, float]],
    translation: tuple[float, float],
) -> tuple[list[tuple[float, float]], tuple[float, float]]:
    resolved: list[tuple[float, float]] = []
    for point in source_points:
        n, e = _finite_pair(point, "mission waypoint")
        resolved_point = (n + translation[0], e + translation[1])
        if not all(math.isfinite(v) for v in resolved_point):
            raise PlacementError("resolved mission waypoint contains non-finite values")
        resolved.append(resolved_point)
    return resolved, translation
