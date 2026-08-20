"""ROS-independent RTK sample quality evaluation shared by rover controllers."""

from __future__ import annotations

import math
from dataclasses import dataclass


GPS_FIX_NAMES = {
    0: "NO_GPS",
    1: "NO_FIX",
    2: "2D_FIX",
    3: "3D_FIX",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
    7: "STATIC",
    8: "PPP",
}


@dataclass(frozen=True)
class RTKQuality:
    fresh: bool
    acceptable: bool
    fix_name: str
    reason: str


def evaluate_rtk_quality(
    *,
    fix_type: int,
    h_acc_m: float | None,
    sample_age_s: float | None,
    timeout_s: float,
    min_fix_type: int = 6,
    max_h_acc_m: float = 0.10,
    require_accuracy: bool = True,
) -> RTKQuality:
    """Evaluate one GPSRAW sample conservatively.

    ``GPSRAW.h_acc == 0`` is represented by ``None`` by callers. Production
    gates fail closed on that unknown accuracy by default; bench deployments
    can explicitly set ``require_accuracy=False``.
    """
    fix_name = GPS_FIX_NAMES.get(int(fix_type), f"FIX_{int(fix_type)}")
    timeout_s = max(0.0, float(timeout_s))
    if sample_age_s is None or not math.isfinite(float(sample_age_s)):
        return RTKQuality(False, False, fix_name, "gps data unavailable")
    age_s = float(sample_age_s)
    if age_s < 0.0 or age_s > timeout_s:
        return RTKQuality(
            False,
            False,
            fix_name,
            f"gps stale ({max(0.0, age_s):.2f}s > {timeout_s:.2f}s)",
        )
    fix_type = int(fix_type)
    min_fix_type = int(min_fix_type)
    if fix_type < min_fix_type:
        return RTKQuality(
            True,
            False,
            fix_name,
            f"gps fix {fix_type} < required {min_fix_type}",
        )
    # MAVLink GPS_FIX_TYPE is an enum, not an accuracy scale. Values above
    # RTK_FIXED are STATIC (7) and PPP (8); accepting them merely because they
    # are numerically greater than 6 can authorize a moving rover with a
    # non-rover solution. This gate accepts only RTK_FLOAT/RTK_FIXED, subject
    # to the configured minimum.
    if fix_type not in (5, 6):
        return RTKQuality(
            True,
            False,
            fix_name,
            f"gps fix {fix_type} is not rover RTK_FLOAT/RTK_FIXED",
        )

    accuracy = None
    if h_acc_m is not None:
        try:
            candidate = float(h_acc_m)
            if math.isfinite(candidate) and candidate >= 0.0:
                accuracy = candidate
        except (TypeError, ValueError):
            pass
    if accuracy is None:
        if require_accuracy:
            return RTKQuality(True, False, fix_name, "gps horizontal accuracy unknown")
        return RTKQuality(True, True, fix_name, "")

    max_h_acc_m = float(max_h_acc_m)
    if max_h_acc_m > 0.0 and accuracy > max_h_acc_m:
        return RTKQuality(
            True,
            False,
            fix_name,
            f"gps horizontal accuracy {accuracy:.3f}m > {max_h_acc_m:.3f}m",
        )
    return RTKQuality(True, True, fix_name, "")
