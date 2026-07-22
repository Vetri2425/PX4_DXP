"""Vertex provenance: source geometry must be distinguishable from fill.

Densification turns a handful of surveyed vertices into hundreds of waypoints.
Once that happens the two are numerically indistinguishable, and downstream
simplification deleted near-collinear survey vertices as if they were fill —
the rover then drove the straight chord and missed them by the offset.

`densify_segment` records which densified indices came from the source
geometry; `PathEngine` propagates that to `PlannedPath.must_hit`, parallel to
`merged_waypoints`. These tests pin that contract.
"""

import math

from path_engine.core import PathSegment, SegmentType
from path_engine.planners.straight_line import densify_segment


# tes_cross_line.dxf: interior vertices bend the line only 1.45°/2.79° but sit
# 3.4/4.4 cm off the end-to-end chord. The real-world case this exists for.
CROSS_LINE = [
    (-0.860, -1.217),
    (-0.286, -0.442),
    (0.266, 0.344),
    (0.880, 1.315),
]


def _vertices_of(seg: PathSegment) -> list[tuple[float, float]]:
    idx = seg.metadata["vertex_indices"]
    return [seg.points[i] for i in idx]


def test_densify_records_one_index_per_source_vertex():
    seg = PathSegment(segment_type=SegmentType.MARK, points=list(CROSS_LINE))
    dense = densify_segment(seg, mark_spacing=0.05)

    assert len(dense.points) > 50, "fixture should actually densify"
    idx = dense.metadata["vertex_indices"]
    assert len(idx) == len(CROSS_LINE)
    assert idx[0] == 0
    assert idx[-1] == len(dense.points) - 1
    assert idx == sorted(idx), "indices must be monotonic"


def test_recorded_indices_point_at_the_original_coordinates():
    seg = PathSegment(segment_type=SegmentType.MARK, points=list(CROSS_LINE))
    dense = densify_segment(seg, mark_spacing=0.05)

    for original, recovered in zip(CROSS_LINE, _vertices_of(dense)):
        assert math.hypot(
            original[0] - recovered[0], original[1] - recovered[1]
        ) < 1e-9, f"vertex {original} not recoverable from provenance"


def test_interior_vertices_are_flagged_not_just_endpoints():
    """The whole point: the *interior* near-collinear vertices must be marked.

    Flagging only the endpoints would leave exactly the vertices that the
    old simplifier deleted unprotected.
    """
    seg = PathSegment(segment_type=SegmentType.MARK, points=list(CROSS_LINE))
    dense = densify_segment(seg, mark_spacing=0.05)
    idx = set(dense.metadata["vertex_indices"])

    interior = idx - {0, len(dense.points) - 1}
    assert len(interior) == 2, f"expected 2 interior vertices, got {len(interior)}"
    # ...and they are genuinely interior fill positions, not artefacts.
    for i in interior:
        assert 0 < i < len(dense.points) - 1


def test_single_point_and_empty_segments_carry_provenance():
    single = densify_segment(PathSegment(points=[(1.0, 2.0)]), 0.05)
    assert single.metadata["vertex_indices"] == [0]

    empty = densify_segment(PathSegment(points=[]), 0.05)
    assert empty.metadata["vertex_indices"] == []


def test_redensify_is_idempotent_and_keeps_vertices_recoverable():
    """The pipeline re-densifies; provenance must survive the second pass."""
    seg = PathSegment(segment_type=SegmentType.MARK, points=list(CROSS_LINE))
    once = densify_segment(seg, mark_spacing=0.05)
    twice = densify_segment(once, mark_spacing=0.05)

    assert len(twice.points) == len(once.points), "re-densify changed point count"
    # After a re-densify every point is a "source" point of that pass, which is
    # correct: the input to pass 2 was already the delivered geometry.
    for original in CROSS_LINE:
        assert min(
            math.hypot(original[0] - p[0], original[1] - p[1]) for p in twice.points
        ) < 1e-9


def test_transit_segments_also_carry_provenance():
    seg = PathSegment(
        segment_type=SegmentType.TRANSIT, points=[(0.0, 0.0), (0.0, 3.0)]
    )
    dense = densify_segment(seg, mark_spacing=0.05, transit_spacing=0.15)
    idx = dense.metadata["vertex_indices"]

    assert idx == [0, len(dense.points) - 1]
    assert len(dense.points) > 2, "transit should densify at transit_spacing"


def test_engine_emits_must_hit_parallel_to_merged_waypoints():
    from path_engine.engine import PathEngine

    engine = PathEngine(mark_spacing=0.05, optimize_order=False)
    plan = engine.plan_segments([
        PathSegment(segment_type=SegmentType.MARK, points=list(CROSS_LINE))
    ])

    assert len(plan.must_hit) == len(plan.merged_waypoints), \
        "must_hit must stay parallel to merged_waypoints"
    assert any(plan.must_hit), "no vertex survived as must-hit"
    assert not all(plan.must_hit), "densification fill was wrongly marked must-hit"

    # Every source vertex is present AND flagged.
    flagged = [p for p, m in zip(plan.merged_waypoints, plan.must_hit) if m]
    for original in CROSS_LINE:
        assert min(
            math.hypot(original[0] - p[0], original[1] - p[1]) for p in flagged
        ) < 1e-6, f"source vertex {original} not flagged must-hit in the plan"
