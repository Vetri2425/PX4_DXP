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


# ---------------------------------------------------------------------------
# Multi-piece shapes: grouping must not clobber provenance (A9 regression)
#
# A survey export gives a square/rectangle as SEPARATE line entities, not one
# continuous polyline. `group_connected_segments` chains them into a composite,
# and `_merge_chain` used to copy the FIRST edge's `vertex_indices` verbatim —
# so the composite flagged only 2 of the shape's corners as must-hit and the RPP
# simplifier was free to round off the other two. 64c12ff fixed the continuous
# case; these pin the assembled-from-pieces case that it never covered.
# ---------------------------------------------------------------------------

def _square_edges(side: float = 2.0) -> list[PathSegment]:
    corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)]
    edges = list(zip(corners, corners[1:] + corners[:1]))
    return [
        densify_segment(
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[a, b],
                source_entity=f"LINE_{i}",
                metadata={"geometry_type": "LINE"},
            ),
            mark_spacing=0.05,
        )
        for i, (a, b) in enumerate(edges)
    ]


def _flagged_coords(seg: PathSegment) -> list[tuple[float, float]]:
    vidx = seg.metadata["vertex_indices"]
    return [seg.points[i] for i in vidx if 0 <= i < len(seg.points)]


def test_grouped_square_flags_all_four_corners_not_two():
    from path_engine.optimizers.shape_grouping import group_connected_segments

    grouped = group_connected_segments(_square_edges(2.0), tol=0.05)
    assert len(grouped) == 1, "four connected edges should chain into one run"
    composite = grouped[0]

    flagged = _flagged_coords(composite)
    for corner in [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]:
        assert min(
            math.hypot(corner[0] - p[0], corner[1] - p[1]) for p in flagged
        ) < 1e-6, f"corner {corner} lost its must-hit flag through grouping"

    # Provenance must not have been over-applied to the densified fill either.
    assert len(composite.metadata["vertex_indices"]) < len(composite.points)


def test_grouped_rectangle_unequal_sides_keeps_every_corner():
    """A rectangle's corners land at DIFFERENT composite indices per edge, so a
    stale [0, N] index set cannot coincidentally cover them (unlike a square)."""
    from path_engine.optimizers.shape_grouping import group_connected_segments

    corners = [(0.0, 0.0), (3.0, 0.0), (3.0, 1.0), (0.0, 1.0)]
    edges = list(zip(corners, corners[1:] + corners[:1]))
    segs = [
        densify_segment(
            PathSegment(
                segment_type=SegmentType.MARK, points=[a, b],
                source_entity=f"LINE_{i}", metadata={"geometry_type": "LINE"},
            ),
            mark_spacing=0.05,
        )
        for i, (a, b) in enumerate(edges)
    ]
    composite = group_connected_segments(segs, tol=0.05)[0]
    flagged = _flagged_coords(composite)
    for corner in corners:
        assert min(
            math.hypot(corner[0] - p[0], corner[1] - p[1]) for p in flagged
        ) < 1e-6, f"rectangle corner {corner} lost its must-hit flag"


def test_grouped_near_collinear_pieces_preserve_interior_vertex():
    """The actually-damaging case: two lines meeting at a shallow (~2°) bend.

    A 90° corner survives simplification on angle alone, but a shallow surveyed
    bend only survives if it is flagged must-hit. If grouping drops the seam
    vertex's provenance, the rover smooths the bend to a straight chord.
    """
    from path_engine.optimizers.shape_grouping import group_connected_segments

    mid = (2.0, 0.07)  # ~2° kink over a 4 m span
    segs = [
        densify_segment(
            PathSegment(
                segment_type=SegmentType.MARK, points=[a, b],
                source_entity=f"LINE_{i}", metadata={"geometry_type": "LINE"},
            ),
            mark_spacing=0.05,
        )
        for i, (a, b) in enumerate([((0.0, 0.0), mid), (mid, (4.0, 0.0))])
    ]
    composite = group_connected_segments(segs, tol=0.05)[0]
    flagged = _flagged_coords(composite)
    assert min(
        math.hypot(mid[0] - p[0], mid[1] - p[1]) for p in flagged
    ) < 1e-6, "the seam bend vertex must stay must-hit through grouping"


def test_decompose_edges_remap_provenance_into_edge_index_space():
    """Per-line mode splits the composite back into edges; each edge's
    vertex_indices must index its OWN points, not the parent's."""
    from path_engine.optimizers.shape_grouping import group_connected_segments
    from path_engine.planners.extensions import decompose_line_chain_to_edges

    composite = group_connected_segments(_square_edges(2.0), tol=0.05)[0]
    edges = decompose_line_chain_to_edges(composite)
    assert len(edges) >= 4, "square composite should split into >=4 edges"

    for edge in edges:
        vidx = edge.metadata["vertex_indices"]
        assert vidx, "edge lost all provenance"
        assert all(0 <= i < len(edge.points) for i in vidx), \
            "edge vertex_indices point outside the edge's own point list"
        # An edge's own two endpoints are corner vertices and must be flagged.
        assert 0 in vidx and (len(edge.points) - 1) in vidx


def test_engine_multipiece_square_flags_all_corners_must_hit():
    """End-to-end: a square built from 4 separate LINE segments must land all
    four corners in plan.must_hit — the field-visible promise A9 broke."""
    from path_engine.engine import PathEngine

    corners = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    edges = list(zip(corners, corners[1:] + corners[:1]))
    segs = [
        PathSegment(
            segment_type=SegmentType.MARK, points=[a, b],
            source_entity=f"LINE_{i}", metadata={"geometry_type": "LINE"},
        )
        for i, (a, b) in enumerate(edges)
    ]
    engine = PathEngine(mark_spacing=0.05, optimize_order=False, group_shapes=True)
    plan = engine.plan_segments(segs)

    flagged = [p for p, m in zip(plan.merged_waypoints, plan.must_hit) if m]
    for corner in corners:
        assert min(
            math.hypot(corner[0] - p[0], corner[1] - p[1]) for p in flagged
        ) < 1e-6, f"corner {corner} not flagged must-hit end-to-end"


# ---------------------------------------------------------------------------
# Declared control_indices: None vs [] vs subset (plan-trajectory contract)
#
# The schema promises `must_hit_indices: []` is a REAL declaration — "nothing
# here is must-hit" — distinct from absent (None), which falls back to
# all-source-vertices. Both densify (remap) and the engine merge used
# truthiness (`if ctrl:`), so a declared-empty list silently degraded to the
# fallback: 8 collinear points with control_indices=[] staged must_hit=8.
# These tests pin the three-way contract at the engine level.

# The 2026-07-29 field shape: a 2-point survey line pre-subdivided to 8 points.
EIGHT_COLLINEAR = [(i * 0.348, 0.0) for i in range(8)]


def _mark_segment(ctrl):
    meta = {} if ctrl is _ABSENT else {"control_indices": list(ctrl)}
    return PathSegment(
        segment_type=SegmentType.MARK, points=list(EIGHT_COLLINEAR), metadata=meta
    )


_ABSENT = object()


def test_densify_preserves_declared_empty_control_indices():
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=list(EIGHT_COLLINEAR),
        metadata={"control_indices": []},
    )
    dense = densify_segment(seg, mark_spacing=0.05)
    assert dense.metadata["control_indices"] == [], \
        "declared-empty [] must survive densification as [], not vanish"


def test_engine_declared_empty_protects_nothing():
    from path_engine.engine import PathEngine

    engine = PathEngine(mark_spacing=0.05, optimize_order=False)
    plan = engine.plan_segments([_mark_segment([])])

    assert len(plan.must_hit) == len(plan.merged_waypoints)
    assert not any(plan.must_hit), \
        "control_indices=[] is a declaration: NO waypoint may be must-hit"


def test_engine_undeclared_falls_back_to_all_source_vertices():
    from path_engine.engine import PathEngine

    engine = PathEngine(mark_spacing=0.05, optimize_order=False)
    plan = engine.plan_segments([_mark_segment(_ABSENT)])

    flagged = [p for p, m in zip(plan.merged_waypoints, plan.must_hit) if m]
    # No declaration → every one of the 8 input points is source geometry.
    for original in EIGHT_COLLINEAR:
        assert min(
            math.hypot(original[0] - p[0], original[1] - p[1]) for p in flagged
        ) < 1e-6, f"undeclared source vertex {original} lost its fallback must-hit"


def test_engine_declared_subset_narrows_to_exactly_those_vertices():
    from path_engine.engine import PathEngine

    engine = PathEngine(mark_spacing=0.05, optimize_order=False)
    plan = engine.plan_segments([_mark_segment([0, 7])])

    flagged = [p for p, m in zip(plan.merged_waypoints, plan.must_hit) if m]
    assert len(flagged) == 2, \
        f"declared [0, 7] must flag exactly 2 waypoints, got {len(flagged)}"
    for original in (EIGHT_COLLINEAR[0], EIGHT_COLLINEAR[7]):
        assert min(
            math.hypot(original[0] - p[0], original[1] - p[1]) for p in flagged
        ) < 1e-6, f"declared vertex {original} not flagged"
