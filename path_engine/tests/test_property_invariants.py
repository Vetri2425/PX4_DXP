"""P3 — randomized geometry invariants (hypothesis).

475 example-based tests pin known shapes; these pin the INVARIANTS on shapes
nobody thought to draw. The one that matters most: no MARK point present in
the input may silently vanish from the output — P1's dedup false positive was
exactly that failure class.

Engine configuration under test is the identity-placement path: origin (0,0),
no GPS, no alignment, extensions off, smoothing off — so input vertices must
survive to the output except through the sanctioned mechanisms:

  1. Step 1c duplicate drop — must be REPORTED in duplicate_stats and the
     dropped stroke must be COVERED by a kept twin within _DUPLICATE_TOL_M;
  2. the ≤ 1 cm junction de-dup in the merge — keeps a coincident twin;
  3. the shape-grouping SEAM FOLD (_merge_chain): when two chained edges'
     endpoints sit within group_join_tol_m (5 cm), the seam vertex is folded
     onto its twin — a deliberate CAD-gap bridge whose error is bounded by
     the tolerance. Found BY these tests while they were being written: two
     overlapping collinear strokes sharing a start point lose their outermost
     ≤ 5 cm at the seam. Bounded and documented behaviour, so the TIGHT
     presence property below keeps all endpoints ≥ 7.5 cm apart (grouping
     then never fires), and a LOOSE global property asserts the 5 cm bound
     holds over arbitrary line soup.

Skips cleanly where hypothesis is not installed (e.g. the Jetson, whose
runner only covers src/).
"""
from __future__ import annotations

import math

import pytest

pytest.importorskip("hypothesis")
from hypothesis import assume, given, settings, strategies as st

from path_engine.core import PathSegment, SegmentType
from path_engine.engine import PathEngine

# Tight bound: identity transform + vertex-preserving densify put the true
# bound at the 1 cm junction de-dup (+ 1 cm dedup-twin coverage); 2.5 cm gives
# float headroom while staying far below the 4 cm "distinct parallel lines"
# scale that P1 protects.
_PRESENCE_TOL_M = 0.025
# Loose global bound: the seam fold may move a junction vertex by up to
# group_join_tol_m (5 cm); nothing may EVER move a source vertex further.
_SEAM_BOUND_M = 0.06
# Endpoint separation that keeps both Step 1c and shape grouping out of play.
_SEPARATION_M = 0.075

_coord = st.floats(
    min_value=-50.0, max_value=50.0, allow_nan=False, allow_infinity=False
).map(lambda v: round(v, 3))


def _mark(pts, name):
    return PathSegment(
        segment_type=SegmentType.MARK, points=pts, speed=0.35,
        source_entity=name, metadata={"geometry_type": "LINE", "line_like": True},
    )


def _engine(**kw):
    kw.setdefault("optimize_order", True)
    kw.setdefault("group_shapes", True)
    return PathEngine(**kw)


def _separated(a_pts, b_pts, min_d=_SEPARATION_M):
    """No endpoint of one segment near any endpoint of the other — keeps both
    the Step 1c dedup and the grouping chain/seam machinery out of play."""
    return all(
        math.dist(p, q) > min_d
        for p in (a_pts[0], a_pts[-1])
        for q in (b_pts[0], b_pts[-1])
    )


@st.composite
def separated_lines(draw, min_lines=1, max_lines=6):
    n = draw(st.integers(min_lines, max_lines))
    segs: list[list[tuple[float, float]]] = []
    for _ in range(n):
        p0 = (draw(_coord), draw(_coord))
        p1 = (draw(_coord), draw(_coord))
        assume(math.dist(p0, p1) >= 0.3)
        pts = [p0, p1]
        assume(all(_separated(pts, other) for other in segs))
        segs.append(pts)
    return [_mark(pts, f"LINE_{i}") for i, pts in enumerate(segs)]


@st.composite
def separated_polylines(draw):
    """1-3 open polylines (2-6 vertices, legs 0.3-3 m), mutually separated and
    individually open by more than the grouping tolerance."""
    n_lines = draw(st.integers(1, 3))
    segs = []
    for i in range(n_lines):
        start = (draw(_coord), draw(_coord))
        pts = [start]
        for _ in range(draw(st.integers(1, 5))):
            ang = draw(st.floats(0.0, 2 * math.pi, allow_nan=False))
            step = draw(st.floats(0.3, 3.0, allow_nan=False))
            last = pts[-1]
            pts.append((round(last[0] + step * math.cos(ang), 3),
                        round(last[1] + step * math.sin(ang), 3)))
        assume(math.dist(pts[0], pts[-1]) > _SEPARATION_M)  # no self-chaining
        assume(all(_separated(pts, s.points) for s in segs))
        segs.append(_mark(pts, f"PL_{i}"))
    return segs


@st.composite
def line_soup(draw, min_lines=1, max_lines=6):
    """Arbitrary lines — overlaps, near-duplicates and chainable endpoints all
    allowed. Exercises Step 1c and the grouping seam machinery."""
    n = draw(st.integers(min_lines, max_lines))
    segs = []
    for i in range(n):
        p0 = (draw(_coord), draw(_coord))
        p1 = (draw(_coord), draw(_coord))
        assume(math.dist(p0, p1) >= 0.3)
        segs.append(_mark([p0, p1], f"LINE_{i}"))
    return segs


@settings(max_examples=60, deadline=None)
@given(separated_polylines())
def test_output_arrays_parallel_and_finite(segs):
    plan = _engine().plan_segments(segs)
    assert len(plan.spray_flags) == len(plan.merged_waypoints) == len(plan.must_hit)
    for n, e in plan.merged_waypoints:
        assert math.isfinite(n) and math.isfinite(e)
    for seg in plan.segments:
        for n, e in seg.points:
            assert math.isfinite(n) and math.isfinite(e)


@settings(max_examples=60, deadline=None)
@given(separated_polylines())
def test_mark_segment_spacing_bounded(segs):
    eng = _engine()
    plan = eng.plan_segments(segs)
    for seg in plan.segments:
        if seg.segment_type != SegmentType.MARK:
            continue
        for a, b in zip(seg.points, seg.points[1:]):
            assert math.dist(a, b) <= eng.mark_spacing + 1e-9


@settings(max_examples=60, deadline=None)
@given(separated_polylines())
def test_totals_match_segment_lengths(segs):
    plan = _engine().plan_segments(segs)
    seg_sum = sum(
        sum(math.dist(a, b) for a, b in zip(s.points, s.points[1:]))
        for s in plan.segments
    )
    total = plan.total_mark_length + plan.total_transit_length
    # The terminal MARK->TRANSIT run-out (0.1 m, extensions off) is appended to
    # merged_waypoints/totals but not to plan.segments; it is the only
    # sanctioned divergence here (close_loop off).
    diff = total - seg_sum
    assert (abs(diff) < 1e-6) or (abs(diff - 0.1) < 1e-6), diff


@settings(max_examples=80, deadline=None)
@given(separated_lines())
def test_no_input_mark_point_lost(segs):
    """THE load-bearing one, tight form: with every endpoint pair separated
    beyond dedup AND grouping reach, nothing is allowed to remove or move any
    input vertex — every one must reappear within 2.5 cm of an output MARK
    waypoint, and Step 1c must report zero removals."""
    plan = _engine().plan_segments(segs)
    assert plan.planning_metadata["duplicate_geometry"]["removed"] == 0
    mark_pts = [p for p, f in zip(plan.merged_waypoints, plan.spray_flags) if f]
    assert mark_pts
    for seg in segs:
        for p in seg.points:
            nearest = min(math.dist(p, q) for q in mark_pts)
            assert nearest <= _PRESENCE_TOL_M, (p, nearest)


@settings(max_examples=80, deadline=None)
@given(line_soup())
def test_no_vertex_strays_beyond_seam_bound(segs):
    """Loose global form over ARBITRARY line soup (duplicates, overlaps,
    chainable junctions): every input vertex must still be within the 5 cm
    seam bound of an output MARK waypoint. Catches whole-segment loss and any
    future pass that moves source geometry beyond the sanctioned tolerances.

    A Step 1c drop is legal only when reported — and the dropped stroke's
    vertices then sit within _DUPLICATE_TOL_M of the kept twin, far inside
    this bound."""
    plan = _engine().plan_segments(segs)
    mark_pts = [p for p, f in zip(plan.merged_waypoints, plan.spray_flags) if f]
    assert mark_pts
    for seg in segs:
        for p in seg.points:
            nearest = min(math.dist(p, q) for q in mark_pts)
            assert nearest <= _SEAM_BOUND_M, (seg.source_entity, p, nearest)


@settings(max_examples=60, deadline=None)
@given(separated_lines(min_lines=1, max_lines=3),
       st.floats(0.0, 0.008, allow_nan=False),
       st.integers(0, 100))
def test_duplicate_drop_is_reported_and_covered(segs, jitter, seed_pick):
    """Add a within-tolerance near-clone of one input line: it must be dropped
    (reported in duplicate_stats) AND every point of the dropped line must
    still be covered by a kept MARK waypoint — dropping is only legal when a
    twin carries the geometry."""
    victim = segs[seed_pick % len(segs)]
    clone_pts = [(p[0] + jitter, p[1]) for p in victim.points]
    clone = _mark(clone_pts, "CLONE")
    plan = _engine().plan_segments(segs + [clone])
    assert plan.planning_metadata["duplicate_geometry"]["removed"] == 1
    assert "CLONE" in plan.planning_metadata["duplicate_geometry"]["sources"]
    mark_pts = [p for p, f in zip(plan.merged_waypoints, plan.spray_flags) if f]
    for p in clone_pts:
        assert min(math.dist(p, q) for q in mark_pts) <= _PRESENCE_TOL_M


@settings(max_examples=60, deadline=None)
@given(separated_lines(min_lines=1, max_lines=3),
       st.floats(0.012, 0.035, allow_nan=False),
       st.integers(0, 100))
def test_beyond_tolerance_neighbour_is_kept(segs, offset, seed_pick):
    """A parallel neighbour displaced past the dedup tolerance is surveyed
    intent: Step 1c must remove nothing. Under the pre-P1 rounding buckets an
    offset in this range could land in the victim's bucket and delete it."""
    victim = segs[seed_pick % len(segs)]
    neighbour = _mark([(p[0] + offset, p[1]) for p in victim.points], "NEIGHBOUR")
    plan = _engine().plan_segments(segs + [neighbour])
    assert plan.planning_metadata["duplicate_geometry"]["removed"] == 0
