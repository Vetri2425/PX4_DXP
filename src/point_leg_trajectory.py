"""Point-mode navigation leg geometry and RPP conditioning prediction.

Point legs publish from live rover pose to a resolved CSV target. Two
production modes exist:

- ``two_point``: publish exactly two endpoints (segment profile in RPP).
- ``densified``: linearly resample the straight leg; RPP uses smooth
  resample-only conditioning (no corner smooth) so intermediates are
  projection geometry, not segment corner goals.
"""

from __future__ import annotations

import math
from enum import Enum

_SPACING_EPS = 1e-6
_COLLINEAR_TOL_DEG = 5.0


class PointLegTrajectoryMode(str, Enum):
    TWO_POINT = "two_point"
    DENSIFIED = "densified"

    @classmethod
    def parse(cls, value: object) -> PointLegTrajectoryMode:
        if isinstance(value, cls):
            return value
        text = str(value or cls.TWO_POINT.value).strip().lower()
        try:
            return cls(text)
        except ValueError as exc:
            raise ValueError(
                f"invalid point_leg_trajectory_mode {value!r}; "
                f"expected {cls.TWO_POINT.value} or {cls.DENSIFIED.value}"
            ) from exc


def leg_length_m(
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    return math.hypot(end[0] - start[0], end[1] - start[1])


def densify_point_leg(
    start: tuple[float, float],
    end: tuple[float, float],
    spacing_m: float,
) -> list[tuple[float, float]]:
    """Linearly resample a straight leg, preserving exact endpoints."""
    if spacing_m <= 0.0 or not math.isfinite(spacing_m):
        raise ValueError("point_leg_spacing_m must be finite and > 0")
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        # Degenerate (start ≈ end): emit two endpoints to match two_point mode
        # so a coincident leg never silently collapses to a one-point hold path.
        return [start, end]
    n_intervals = max(1, int(math.ceil((length - _SPACING_EPS) / spacing_m)))
    n_steps = n_intervals + 1
    pts: list[tuple[float, float]] = []
    for i in range(n_steps):
        t = i / (n_steps - 1)
        pts.append((start[0] + t * dx, start[1] + t * dy))
    pts[0] = start
    pts[-1] = end
    return pts


def build_point_leg_path(
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    mode: PointLegTrajectoryMode | str = PointLegTrajectoryMode.TWO_POINT,
    spacing_m: float = 0.08,
) -> list[tuple[float, float]]:
    """Build orchestrator publish geometry for one point leg."""
    traj_mode = (
        mode if isinstance(mode, PointLegTrajectoryMode) else PointLegTrajectoryMode.parse(mode)
    )
    if traj_mode == PointLegTrajectoryMode.TWO_POINT:
        return [start, end]
    return densify_point_leg(start, end, spacing_m)


def _segment_heading(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _heading_delta(a: float, b: float) -> float:
    d = b - a
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return abs(d)


def is_collinear_straight_leg(
    pts: list[tuple[float, float]],
    *,
    collinear_tol_deg: float = _COLLINEAR_TOL_DEG,
) -> bool:
    """True when every interior vertex lies on one straight segment."""
    if len(pts) < 2:
        return True
    clean: list[tuple[float, float]] = [pts[0]]
    for pt in pts[1:]:
        if math.hypot(pt[0] - clean[-1][0], pt[1] - clean[-1][1]) >= 1e-6:
            clean.append(pt)
    if len(clean) < 3:
        return True
    tol = math.radians(collinear_tol_deg)
    for i in range(1, len(clean) - 1):
        h0 = _segment_heading(clean[i - 1], clean[i])
        h1 = _segment_heading(clean[i], clean[i + 1])
        if _heading_delta(h0, h1) > tol:
            return False
    return True


def simplify_collinear_path(
    pts: list[tuple[float, float]],
    *,
    collinear_tol_deg: float = _COLLINEAR_TOL_DEG,
) -> list[tuple[float, float]]:
    """Mirror RPP segment-mode collinear simplification (no flag boundaries)."""
    if not pts:
        return []
    clean_pts: list[tuple[float, float]] = [pts[0]]
    for pt in pts[1:]:
        if math.hypot(pt[0] - clean_pts[-1][0], pt[1] - clean_pts[-1][1]) < 1e-6:
            continue
        clean_pts.append(pt)
    if len(clean_pts) < 3:
        return clean_pts
    tol = math.radians(collinear_tol_deg)
    out_pts: list[tuple[float, float]] = [clean_pts[0]]
    for i in range(1, len(clean_pts) - 1):
        prev_pt = out_pts[-1]
        this_pt = clean_pts[i]
        next_pt = clean_pts[i + 1]
        h0 = _segment_heading(prev_pt, this_pt)
        h1 = _segment_heading(this_pt, next_pt)
        if _heading_delta(h0, h1) <= tol:
            continue
        out_pts.append(this_pt)
    out_pts.append(clean_pts[-1])
    return out_pts


def resample_polyline(
    pts: list[tuple[float, float]],
    spacing: float,
) -> list[tuple[float, float]]:
    """Mirror RPP ``_resample_path`` geometry (straight segments stay straight)."""
    if len(pts) < 2 or spacing <= 0.0:
        return list(pts)
    cum = [0.0]
    for i in range(1, len(pts)):
        cum.append(
            cum[-1]
            + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        )
    total = cum[-1]
    if total < spacing:
        return [pts[0], pts[-1]]
    n_samples = max(2, int(math.ceil(total / spacing)) + 1)
    out: list[tuple[float, float]] = []
    seg = 0
    for k in range(n_samples):
        target = (k / (n_samples - 1)) * total
        while seg + 1 < len(cum) - 1 and cum[seg + 1] < target:
            seg += 1
        seg_len = cum[seg + 1] - cum[seg]
        if seg_len < 1e-12:
            out.append(pts[seg])
            continue
        t = (target - cum[seg]) / seg_len
        t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
        n = pts[seg][0] + t * (pts[seg + 1][0] - pts[seg][0])
        e = pts[seg][1] + t * (pts[seg + 1][1] - pts[seg][1])
        out.append((n, e))
    out[0] = pts[0]
    out[-1] = pts[-1]
    return out


def predict_rpp_conditioning(
    published_pts: list[tuple[float, float]],
    *,
    runtime_entry: bool = True,
    resample_spacing_m: float = 0.08,
) -> tuple[str, list[tuple[float, float]]]:
    """Predict RPP profile and conditioned geometry for a point leg."""
    if not published_pts:
        return "segment", []
    if (
        runtime_entry
        and len(published_pts) >= 3
        and is_collinear_straight_leg(published_pts)
    ):
        conditioned = resample_polyline(published_pts, resample_spacing_m)
        return "smooth", conditioned
    simplified = simplify_collinear_path(published_pts)
    if len(simplified) <= 2:
        return "segment", simplified
    return "segment", simplified


def interior_spacing_stats(
    pts: list[tuple[float, float]],
) -> tuple[float, float]:
    """Return (min_spacing, max_spacing) between consecutive interior samples."""
    if len(pts) < 2:
        return (0.0, 0.0)
    spacings = [
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(pts[:-1], pts[1:])
    ]
    return (min(spacings), max(spacings))