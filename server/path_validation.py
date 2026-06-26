"""Controller-bound path validation helpers.

These checks sit at runtime trust boundaries, after planning/staging but before
mission state or ROS publication accepts geometry.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Iterable

_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from path_identity import path_geometry_fingerprint  # noqa: E402


def normalize_path_points(
    points: Iterable[tuple[float, float]],
    *,
    label: str = "path",
    min_points: int = 1,
) -> list[tuple[float, float]]:
    """Return finite `(north, east)` points or raise `ValueError`."""
    normalized: list[tuple[float, float]] = []
    for index, point in enumerate(points):
        try:
            n_raw, e_raw = point
            n = float(n_raw)
            e = float(e_raw)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"{label} point {index} must contain numeric north/east"
            ) from exc
        if not math.isfinite(n) or not math.isfinite(e):
            raise ValueError(f"{label} point {index} must contain finite coordinates")
        normalized.append((n, e))

    if len(normalized) < min_points:
        raise ValueError(f"{label} must contain at least {min_points} point(s)")
    return normalized


def normalize_spray_flags(
    flags: Iterable[bool] | None,
    expected_len: int,
    *,
    default: bool = False,
) -> list[bool]:
    if flags is None:
        return [default] * expected_len
    normalized = [bool(flag) for flag in flags]
    if len(normalized) != expected_len:
        raise ValueError(
            f"spray_flags length {len(normalized)} does not match path length {expected_len}"
        )
    return normalized


def verified_path_fingerprint(
    points: Iterable[tuple[float, float]],
    flags: Iterable[bool],
    supplied: str = "",
) -> str:
    """Compute fingerprint and reject any supplied fingerprint mismatch."""
    point_list = list(points)
    flag_list = list(flags)
    if len(point_list) < 2:
        if supplied:
            raise ValueError("supplied path fingerprint requires at least two points")
        return ""
    computed = path_geometry_fingerprint(point_list, flag_list)
    supplied = str(supplied or "")
    if supplied and supplied != computed:
        raise ValueError("supplied path fingerprint does not match path geometry")
    return supplied or computed
