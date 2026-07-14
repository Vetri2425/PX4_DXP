"""Dimensional-fidelity invariants: a 2 m square DXF must produce a 2 m trajectory.

This is the operator's acceptance criterion expressed as a test:

    "If I give a Square DXF with 2 m, the trajectory must be 2 m — production-grade."

Cross-track RMS cannot police this. It measures distance *to the path*, so a path
that has been silently resized is tracked perfectly and scores perfectly. These
tests check the thing RMS is blind to: that the path itself has the right dimensions
and uniform spacing.

Covers two defects found in the 2026-07-14 audit:
  * free-scale alignment stretching geometry to absorb survey noise
  * shape-grouping culling every other densified waypoint (tol == mark_spacing)
"""
from __future__ import annotations

import math

import pytest

from path_engine.core import PathSegment, SegmentType
from path_engine.ned import (
    apply_affine_transform,
    dxf_to_ned_affine,
    estimate_fit_scale,
)
from path_engine.optimizers.shape_grouping import group_connected_segments
from path_engine.planners.straight_line import densify_segment

SIDE_M = 2.0
MARK_SPACING = 0.05
TOL_MM = 1e-3  # 1 mm dimensional tolerance


def _square_edges(side: float = SIDE_M) -> list[PathSegment]:
    """A square authored as 4 separate LINE entities — a common CAD export style,
    and the case that triggers shape-grouping (a single closed LWPOLYLINE would not).
    """
    corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]
    return [
        PathSegment(
            segment_type=SegmentType.MARK,
            points=[corners[i], corners[i + 1]],
            speed=0.35,
            segment_id=f"s{i}",
            source_entity=f"LINE#{i}",
            metadata={"geometry_type": "LINE", "line_like": True},
        )
        for i in range(4)
    ]


def _gaps(pts: list[tuple[float, float]]) -> list[float]:
    return [
        math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        for i in range(len(pts) - 1)
    ]


# ── Spacing invariant ─────────────────────────────────────────────────────────

def test_grouped_square_keeps_uniform_mark_spacing():
    """Every interval must be <= mark_spacing. No culled waypoints.

    Regression: _merge_chain used the endpoint-coincidence tolerance (0.05 m) as a
    duplicate-point test. That equals the default mark_spacing, so every legitimate
    densified sample looked like a duplicate and every other one was dropped —
    producing alternating 5cm/10cm gaps. Total length was preserved, so it never
    surfaced as a dimensional error, but it corrupts spray metering.
    """
    densified = [densify_segment(s, MARK_SPACING, 0.15) for s in _square_edges()]
    grouped = group_connected_segments(densified, tol=0.05)

    # The 4 edges chain into one closed composite.
    assert len(grouped) == 1, f"expected one chained shape, got {len(grouped)}"

    gaps = _gaps(grouped[0].points)
    assert max(gaps) <= MARK_SPACING + 1e-9, (
        f"spacing invariant violated: max gap {max(gaps) * 100:.2f} cm "
        f"> {MARK_SPACING * 100:.0f} cm. Distinct gaps: {sorted(set(round(g, 4) for g in gaps))}"
    )


def test_grouped_square_preserves_perimeter():
    densified = [densify_segment(s, MARK_SPACING, 0.15) for s in _square_edges()]
    grouped = group_connected_segments(densified, tol=0.05)
    perimeter = sum(_gaps(grouped[0].points))
    assert perimeter == pytest.approx(4 * SIDE_M, abs=TOL_MM)


def test_grouping_does_not_displace_corners():
    """The 4 corner vertices must survive grouping at their exact coordinates."""
    densified = [densify_segment(s, MARK_SPACING, 0.15) for s in _square_edges()]
    grouped = group_connected_segments(densified, tol=0.05)
    pts = grouped[0].points
    for corner in [(0.0, 0.0), (SIDE_M, 0.0), (SIDE_M, SIDE_M), (0.0, SIDE_M)]:
        assert any(
            math.hypot(p[0] - corner[0], p[1] - corner[1]) < TOL_MM for p in pts
        ), f"corner {corner} missing after grouping"


# ── Dimensional invariant (the acceptance criterion) ──────────────────────────

@pytest.mark.parametrize(
    "ned_refs,label",
    [
        ([(0.0, 0.0), (2.0, 0.0)], "survey agrees with drawing"),
        ([(0.0, 0.0), (2.10, 0.0)], "survey 5% long"),
        ([(0.0, 0.0), (1.90, 0.0)], "survey 5% short"),
        ([(0.0, 0.0), (0.0, 2.04)], "survey long + rotated 90deg"),
    ],
)
def test_two_metre_square_stays_two_metres_under_any_survey(ned_refs, label):
    """THE acceptance criterion. Whatever the reference points say, a 2 m square
    must come out 2 m. Survey error may move it and rotate it — never resize it.
    """
    dxf_refs = [(0.0, 0.0), (SIDE_M, 0.0)]  # two corners of the square
    scale, theta, off_n, off_e, _, _ = dxf_to_ned_affine(dxf_refs, ned_refs)

    assert scale == 1.0, f"[{label}] geometry was rescaled (scale={scale})"

    corners = [(0.0, 0.0), (SIDE_M, 0.0), (SIDE_M, SIDE_M), (0.0, SIDE_M)]
    tx = [apply_affine_transform(p, scale, theta, off_n, off_e) for p in corners]

    sides = _gaps(tx + [tx[0]])
    for i, s in enumerate(sides):
        assert s == pytest.approx(SIDE_M, abs=TOL_MM), (
            f"[{label}] side {i} is {s:.4f} m, expected {SIDE_M} m"
        )

    # Diagonals too — catches shear, which would keep sides right but skew the shape.
    for d in (
        math.hypot(tx[2][0] - tx[0][0], tx[2][1] - tx[0][1]),
        math.hypot(tx[3][0] - tx[1][0], tx[3][1] - tx[1][1]),
    ):
        assert d == pytest.approx(SIDE_M * math.sqrt(2), abs=TOL_MM)


def test_survey_disagreement_is_reported_not_absorbed():
    """A survey that disagrees with the drawing must surface as RMSE.

    Under the old free-scale fit this was structurally impossible to detect: a
    similarity fit through 2 points is exactly determined, so RMSE was always ~0 no
    matter how badly the size was wrong. Locking scale gives the rigid 2-point fit
    one residual degree of freedom — and it is exactly the baseline-length mismatch.
    """
    dxf_refs = [(0.0, 0.0), (SIDE_M, 0.0)]
    ned_refs = [(0.0, 0.0), (2.10, 0.0)]  # survey claims 2.10 m for a 2.00 m edge

    _, _, _, _, _, rmse_rigid = dxf_to_ned_affine(dxf_refs, ned_refs)
    _, _, _, _, _, rmse_free = dxf_to_ned_affine(dxf_refs, ned_refs, lock_scale=False)

    assert rmse_free < 1e-9, "free-scale fit is blind by construction (the old bug)"
    assert rmse_rigid > 0.04, f"rigid fit must report the disagreement, got {rmse_rigid}"

    # The diagnostic still tells us what a free fit *would* have done.
    assert estimate_fit_scale(dxf_refs, ned_refs) == pytest.approx(1.05, abs=1e-6)
