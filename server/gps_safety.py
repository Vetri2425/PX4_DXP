"""GPS/RTK placement safety for GPS_SURVEYED point missions."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Iterable

from config import (
    GLOBAL_POSITION_STALE_MS,
    GPS_FIX_STALE_MS,
    POSE_GLOBAL_MAX_SKEW_MS,
    POSE_STALE_MS,
)
from mission_placement import GPS_FIX_TYPE_RTK_FIXED, PlacementError, resolve_surveyed_points

GPS_SAFETY_OK = "ok"
GPS_SAFETY_FAULT = "fault"
GPS_SAFETY_RECOVERING = "recovering"
GPS_SAFETY_PAUSED = "paused"
GPS_SAFETY_NA = "not_applicable"

RUNTIME_POLICY_PAUSE = "pause"
RUNTIME_POLICY_FAIL = "fail"
RESUME_POLICY_MANUAL = "manual"
RESUME_POLICY_AUTO = "auto"


@dataclass(frozen=True)
class GpsSurveyedSafetyParams:
    required_fix_type: int = GPS_FIX_TYPE_RTK_FIXED
    global_position_max_age_ms: float = GLOBAL_POSITION_STALE_MS
    local_pose_max_age_ms: float = POSE_STALE_MS
    gps_fix_max_age_ms: float = GPS_FIX_STALE_MS
    max_pose_global_skew_ms: float = POSE_GLOBAL_MAX_SKEW_MS
    runtime_policy: str = RUNTIME_POLICY_PAUSE
    resume_policy: str = RESUME_POLICY_MANUAL
    recovery_stable_s: float = 2.0


@dataclass
class GpsSafetyVerdict:
    ok: bool
    reason: str = ""
    gps_safety_state: str = GPS_SAFETY_OK
    gps_safety_ok: bool = True
    required_fix_type: int = GPS_FIX_TYPE_RTK_FIXED
    current_fix_type: int | None = None
    global_position_age_ms: float | None = None
    local_pose_age_ms: float | None = None
    gps_fix_age_ms: float | None = None
    pose_global_skew_ms: float | None = None
    anchor_valid: bool = False
    runtime_policy: str = RUNTIME_POLICY_PAUSE
    resume_policy: str = RESUME_POLICY_MANUAL
    recovery_ready: bool = False
    gps_fault_count: int = 0
    last_gps_safety_reason: str = ""
    last_gps_fault_time_s: float | None = None
    telemetry: dict[str, Any] = field(default_factory=dict)

    def as_status_dict(self) -> dict[str, Any]:
        return {
            "gps_safety_state": self.gps_safety_state,
            "gps_safety_ok": self.gps_safety_ok,
            "gps_required_fix_type": self.required_fix_type,
            "gps_current_fix_type": self.current_fix_type,
            "gps_global_position_age_ms": self.global_position_age_ms,
            "gps_local_pose_age_ms": self.local_pose_age_ms,
            "gps_fix_age_ms": self.gps_fix_age_ms,
            "gps_pose_global_skew_ms": self.pose_global_skew_ms,
            "gps_anchor_valid": self.anchor_valid,
            "gps_last_safety_reason": self.last_gps_safety_reason,
            "gps_fault_count": self.gps_fault_count,
            "gps_last_fault_time_s": self.last_gps_fault_time_s,
            "gps_recovery_ready": self.recovery_ready,
            "gps_runtime_policy": self.runtime_policy,
            "gps_resume_policy": self.resume_policy,
        }


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


def _anchor_valid(origin_gps: tuple[float, float] | None) -> bool:
    try:
        lat, lon = _finite_pair(origin_gps, "survey GPS anchor")
    except PlacementError:
        return False
    return -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0


def _read_age(state: dict, key: str) -> float | None:
    raw = state.get(key)
    if raw is None:
        return None
    try:
        age = float(raw)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(age) or age < 0.0:
        return None
    return age


def evaluate_gps_surveyed_safety(
    state: dict[str, Any],
    origin_gps: tuple[float, float] | None,
    source_points: Iterable[tuple[float, float]] | None,
    params: GpsSurveyedSafetyParams,
    *,
    recovery_since: float | None = None,
    fault_count: int = 0,
    last_fault_time_s: float | None = None,
    paused: bool = False,
) -> GpsSafetyVerdict:
    """Evaluate GPS_SURVEYED placement safety; LOCAL_NED callers should skip this."""
    points = list(source_points or [])
    verdict = GpsSafetyVerdict(
        ok=True,
        required_fix_type=params.required_fix_type,
        runtime_policy=params.runtime_policy,
        resume_policy=params.resume_policy,
        gps_fault_count=fault_count,
        last_gps_fault_time_s=last_fault_time_s,
        anchor_valid=_anchor_valid(origin_gps),
    )
    verdict.global_position_age_ms = _read_age(state, "global_position_age_ms")
    verdict.local_pose_age_ms = _read_age(state, "local_pose_age_ms")
    verdict.gps_fix_age_ms = _read_age(state, "gps_fix_age_ms")
    skew = state.get("pose_global_skew_ms")
    try:
        verdict.pose_global_skew_ms = float(skew) if skew is not None else None
    except (TypeError, ValueError):
        verdict.pose_global_skew_ms = None
    try:
        verdict.current_fix_type = int(state.get("gps_fix"))
    except (TypeError, ValueError):
        verdict.current_fix_type = None

    def _fail(reason: str) -> GpsSafetyVerdict:
        verdict.ok = False
        verdict.gps_safety_ok = False
        verdict.reason = reason
        verdict.last_gps_safety_reason = reason
        verdict.gps_safety_state = GPS_SAFETY_PAUSED if paused else GPS_SAFETY_FAULT
        verdict.telemetry = {
            "connected": state.get("connected"),
            "gps_fix": verdict.current_fix_type,
            "global_position_age_ms": verdict.global_position_age_ms,
            "local_pose_age_ms": verdict.local_pose_age_ms,
            "gps_fix_age_ms": verdict.gps_fix_age_ms,
            "pose_global_skew_ms": verdict.pose_global_skew_ms,
        }
        return verdict

    if not state.get("connected", False):
        return _fail("FCU disconnected")

    if not verdict.anchor_valid:
        return _fail("survey GPS anchor is missing or invalid")

    if not state.get("pose_received", False):
        return _fail("local pose has not been received")
    if not state.get("global_position_received", False):
        return _fail("fused global position has not been received")
    if not state.get("gps_fix_received", False):
        return _fail("GPS fix information has not been received")

    if verdict.local_pose_age_ms is None:
        return _fail("local pose freshness is unavailable")
    if verdict.local_pose_age_ms > params.local_pose_max_age_ms:
        return _fail(
            f"local pose is stale ({verdict.local_pose_age_ms:.0f} ms > "
            f"{params.local_pose_max_age_ms:.0f} ms)"
        )

    if verdict.global_position_age_ms is None:
        return _fail("fused global position freshness is unavailable")
    if verdict.global_position_age_ms > params.global_position_max_age_ms:
        return _fail(
            f"fused global position is stale ({verdict.global_position_age_ms:.0f} ms > "
            f"{params.global_position_max_age_ms:.0f} ms)"
        )

    if verdict.gps_fix_age_ms is None:
        return _fail("GPS fix information freshness is unavailable")
    if verdict.gps_fix_age_ms > params.gps_fix_max_age_ms:
        return _fail(
            f"GPS fix information is stale ({verdict.gps_fix_age_ms:.0f} ms > "
            f"{params.gps_fix_max_age_ms:.0f} ms)"
        )

    if verdict.pose_global_skew_ms is None:
        return _fail("local/global position receive-time skew is unavailable")
    if verdict.pose_global_skew_ms > params.max_pose_global_skew_ms:
        return _fail(
            "local/global position samples are not sufficiently aligned "
            f"({verdict.pose_global_skew_ms:.0f} ms > "
            f"{params.max_pose_global_skew_ms:.0f} ms)"
        )

    if verdict.current_fix_type is None:
        return _fail("GPS fix type is invalid")
    if verdict.current_fix_type < params.required_fix_type:
        return _fail(
            f"GPS fix_type={verdict.current_fix_type} is below required "
            f"({params.required_fix_type})"
        )

    try:
        resolve_surveyed_points(points, origin_gps, state, safety=params)
    except PlacementError as exc:
        return _fail(str(exc))

    now = time.monotonic()
    if recovery_since is not None:
        stable = now - recovery_since
        verdict.recovery_ready = stable >= params.recovery_stable_s
        verdict.gps_safety_state = (
            GPS_SAFETY_OK if verdict.recovery_ready else GPS_SAFETY_RECOVERING
        )
    else:
        verdict.recovery_ready = True
        verdict.gps_safety_state = GPS_SAFETY_OK if not paused else GPS_SAFETY_PAUSED

    verdict.gps_safety_ok = True
    verdict.ok = True
    return verdict


def local_ned_gps_status() -> dict[str, Any]:
    """Diagnostics payload when GPS safety does not apply."""
    return {
        "gps_safety_state": GPS_SAFETY_NA,
        "gps_safety_ok": True,
        "gps_required_fix_type": None,
        "gps_current_fix_type": None,
        "gps_global_position_age_ms": None,
        "gps_local_pose_age_ms": None,
        "gps_fix_age_ms": None,
        "gps_pose_global_skew_ms": None,
        "gps_anchor_valid": None,
        "gps_last_safety_reason": "",
        "gps_fault_count": 0,
        "gps_last_fault_time_s": None,
        "gps_recovery_ready": True,
        "gps_runtime_policy": None,
        "gps_resume_policy": None,
    }