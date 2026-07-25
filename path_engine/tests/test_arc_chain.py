"""Arc fit for surveyed LINE_CHAINs (PathEngine.fit_arcs)."""

import math
import random

import pytest

from path_engine.core import PathSegment, SegmentType
from path_engine.engine import PathEngine
from path_engine.planners.arc_chain import (
    _fit_circle_kasa,
    _max_chord_deviation,
    _segment_by_curvature,
    _split_at_corners,
    fit_line_chain,
)


def _arc_points(cn, ce, r, a0_deg, a1_deg, n, noise=0.0, seed=1):
    """n surveyed (north, east) samples along a circle, optional Gaussian noise."""
    rng = random.Random(seed)
    pts = []
    for i in range(n):
        a = math.radians(a0_deg + (a1_deg - a0_deg) * i / (n - 1))
        pn = cn + r * math.sin(a)
        pe = ce + r * math.cos(a)
        if noise:
            pn += rng.gauss(0.0, noise)
            pe += rng.gauss(0.0, noise)
        pts.append((pn, pe))
    return pts


def _max_radial_dev(points, cn, ce, r):
    return max(abs(math.hypot(p[0] - cn, p[1] - ce) - r) for p in points)


# --- circle fit primitive ---------------------------------------------------

def test_kasa_recovers_a_clean_circle():
    pts = _arc_points(cn=5.0, ce=-3.0, r=6.4, a0_deg=10, a1_deg=100, n=9)
    cn, ce, r = _fit_circle_kasa(pts)
    assert cn == pytest.approx(5.0, abs=1e-6)
    assert ce == pytest.approx(-3.0, abs=1e-6)
    assert r == pytest.approx(6.4, abs=1e-6)


def test_kasa_returns_none_for_collinear_points():
    assert _fit_circle_kasa([(0, 0), (0, 1), (0, 2), (0, 3)]) is None


# --- corner splitting -------------------------------------------------------

def test_square_corners_split_the_chain():
    sq = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (1, 2), (0, 2)]
    runs = _split_at_corners(sq, math.radians(35))
    # Corners at index 2 (2,0) and 4 (2,2): three runs.
    assert runs == [(0, 2), (2, 4), (4, 6)]


def test_gentle_arc_is_not_split():
    pts = _arc_points(cn=0, ce=0, r=8.0, a0_deg=0, a1_deg=60, n=9)
    runs = _split_at_corners(pts, math.radians(35))
    assert runs == [(0, len(pts) - 1)]  # one whole run, no false corners


# --- fit_line_chain: geometry ----------------------------------------------

def test_clean_arc_fits_within_tolerance():
    true = dict(cn=2.0, ce=1.0, r=6.4)
    pts = _arc_points(**true, a0_deg=20, a1_deg=110, n=8)
    out, ctrl = fit_line_chain(pts, rms_m=0.025)
    assert len(out) > len(pts), "arc should be tessellated denser than input"
    assert _max_radial_dev(out, **true) < 0.005
    # Corners (endpoints) are the only control points on a single arc.
    assert ctrl == [0, len(out) - 1]
    # Endpoints snap to the surveyed points exactly.
    assert out[0] == pytest.approx(pts[0])
    assert out[-1] == pytest.approx(pts[-1])


def test_noisy_arc_stays_within_2cm_of_truth():
    """A 10 m, ~1.7 cm-RMS surveyed arc fits to within ~2 cm (acceptance)."""
    true = dict(cn=0.0, ce=0.0, r=7.0)   # ~11 m of arc over 90 deg
    pts = _arc_points(**true, a0_deg=0, a1_deg=90, n=11, noise=0.017, seed=7)
    out, _ = fit_line_chain(pts, rms_m=0.025)
    # Interior fitted points lie on the fitted circle; only the snapped
    # endpoints carry the raw per-point noise. Straight-chord densification of
    # this same input would sag ~30 cm off the true arc.
    interior = out[1:-1]
    assert _max_radial_dev(interior, **true) < 0.02


def test_straight_noisy_run_is_kept_straight():
    rng = random.Random(3)
    pts = [(i * 1.0, rng.gauss(0.0, 0.01)) for i in range(6)]  # along +north
    out, ctrl = fit_line_chain(pts, rms_m=0.025)
    assert out == pts                       # unchanged, no arc fit
    assert ctrl == list(range(len(pts)))    # every surveyed vertex is control


def test_square_stays_square_corners_kept():
    sq = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (1, 2), (0, 2)]
    out, ctrl = fit_line_chain(sq, rms_m=0.025)
    assert out == sq                        # straight sides, nothing fit
    # All three 90 deg corners (and every straight-run vertex) stay control.
    assert ctrl == list(range(len(sq)))
    for corner in [(0, 0), (2, 0), (2, 2), (0, 2)]:
        assert corner in out


def test_arc_then_corner_then_line_joins_cleanly():
    arc = _arc_points(cn=0, ce=0, r=5.0, a0_deg=0, a1_deg=80, n=7)
    # A sharp corner then a straight run away from the arc's end.
    tail = [(arc[-1][0] + 1.0, arc[-1][1] - 2.0),
            (arc[-1][0] + 2.0, arc[-1][1] - 4.0)]
    out, ctrl = fit_line_chain(arc + tail, rms_m=0.025)
    # No duplicated junction and the polyline is continuous.
    for i in range(len(out) - 1):
        assert math.dist(out[i], out[i + 1]) > 1e-9
    assert out[-1] == pytest.approx(tail[-1])


def _bbox(pts):
    ns = [p[0] for p in pts]; es = [p[1] for p in pts]
    return (max(ns) - min(ns), max(es) - min(es))


def test_near_straight_noisy_run_does_not_blow_up_into_a_circle():
    """Regression: a near-straight noisy run 'fits' a huge circle; the old code
    drew it as a ~360° arc (start=-46°, end=-45.5° → +2π). It must stay bounded."""
    rng = random.Random(11)
    # 3 m line along +east with ~2 cm wobble — curved enough to trip the arc gate,
    # but nowhere near a real arc.
    pts = [(rng.gauss(0.0, 0.02), i * 0.25) for i in range(13)]
    out, _ = fit_line_chain(pts, rms_m=0.025)
    inbb = _bbox(pts)
    outbb = _bbox(out)
    assert outbb[0] < inbb[0] + 0.2 and outbb[1] < inbb[1] + 0.2, "arc blew up"


def test_non_circular_curve_is_kept_as_raw_points_not_a_bad_circle():
    """A run that isn't a circle (e.g. a gentle S / road) must not be forced onto
    one circle — the residual guard keeps the raw polyline."""
    # Half a sine wave: curved but NOT a circular arc.
    pts = [(math.sin(t) * 2.0, t) for t in [i * 0.3 for i in range(12)]]
    out, _ = fit_line_chain(pts, rms_m=0.025)
    inbb = _bbox(pts)
    outbb = _bbox(out)
    assert outbb[0] < inbb[0] * 1.4 + 0.5 and outbb[1] < inbb[1] * 1.4 + 0.5


def test_short_chain_is_a_noop():
    assert fit_line_chain([(0, 0), (1, 1)]) == ([(0, 0), (1, 1)], [0, 1])


# --- curvature segmentation -------------------------------------------------
#
# The corner split alone cannot handle a real survey line: a road is ONE
# corner-free run of straights and gentle bends, and forcing one circle through
# the whole thing fits nothing, so every curve silently stayed a chord.

def _line_points(n0, e0, n1, e1, step=0.5):
    """Surveyed samples every `step` along a straight, endpoints included."""
    length = math.hypot(n1 - n0, e1 - e0)
    k = max(1, round(length / step))
    return [(n0 + (n1 - n0) * i / k, e0 + (e1 - e0) * i / k) for i in range(k + 1)]


def _polygonised_circle(cn, ce, r, facets):
    """A circle surveyed as an N-gon: vertices on the circle, samples on the
    CHORDS between them. This is what the Egmore roundabout CSV actually is —
    the shape the operator sees as a polygon instead of a circle."""
    pts = []
    for f in range(facets):
        a0 = 2 * math.pi * f / facets
        a1 = 2 * math.pi * (f + 1) / facets
        p0 = (cn + r * math.sin(a0), ce + r * math.cos(a0))
        p1 = (cn + r * math.sin(a1), ce + r * math.cos(a1))
        seg = _line_points(*p0, *p1, step=0.5)
        pts.extend(seg[:-1])
    pts.append(pts[0])
    return pts


def _road_straight_bend_straight():
    """40 m straight → 90 deg bend on a 15 m radius → 40 m straight. No vertex
    turns more than ~2 deg, so the corner split never fires."""
    arc = _arc_points(cn=0.0, ce=0.0, r=15.0, a0_deg=0, a1_deg=90, n=48)
    head = _line_points(arc[0][0] - 40.0, arc[0][1], arc[0][0], arc[0][1])
    tail = _line_points(arc[-1][0], arc[-1][1], arc[-1][0], arc[-1][1] - 40.0)
    return head[:-1] + arc + tail[1:]


def test_curvature_split_finds_the_bend_inside_a_corner_free_road():
    pts = _road_straight_bend_straight()
    assert _split_at_corners(pts, math.radians(35)) == [(0, len(pts) - 1)], \
        "precondition: no vertex is a corner, so only curvature can split this"
    subs = _segment_by_curvature(pts)
    assert [is_arc for _, _, is_arc in subs] == [False, True, False]


def test_curvature_split_cuts_an_s_bend_at_the_inflection():
    """Two opposite arcs must not be fit as one circle."""
    left = _arc_points(cn=0.0, ce=0.0, r=20.0, a0_deg=0, a1_deg=60, n=40)
    # Mirror the second half about the joint so curvature flips sign.
    jn, je = left[-1]
    right = [(jn + (jn - p[0]), je + (p[1] - je)) for p in reversed(left[:-1])]
    subs = _segment_by_curvature(left + right)
    arc_signs = [is_arc for _, _, is_arc in subs]
    assert arc_signs.count(True) >= 2, f"S-bend collapsed into {subs}"


def test_road_bend_is_fit_as_an_arc_not_left_as_chords():
    """Regression: a road-shaped chain used to fit ONE circle through the whole
    run (huge residual → rejected), so its bend stayed a straight-chord polygon."""
    pts = _road_straight_bend_straight()
    whole = _fit_circle_kasa(pts)
    assert whole is not None
    cn, ce, r = whole
    whole_resid = max(abs(math.hypot(p[0] - cn, p[1] - ce) - r) for p in pts)
    assert whole_resid > 1.0, "precondition: one circle cannot explain this chain"

    out, _ = fit_line_chain(pts, max_spacing_m=0.05)
    assert len(out) > len(pts), "the bend should be tessellated, not left as chords"
    # Every point of the fitted bend sits on the true 15 m circle.
    on_arc = [p for p in out if abs(math.hypot(p[0], p[1]) - 15.0) < 0.05]
    assert len(on_arc) > 100


def test_polygonised_circle_is_recovered_as_a_true_circle():
    """The roundabout case: a circle surveyed as a ~20-gon must come back as a
    circle, not as the polygon. Facet sagitta here is ~14 cm."""
    pts = _polygonised_circle(cn=2.0, ce=-3.0, r=11.5, facets=20)
    sagitta = 11.5 * (1 - math.cos(math.pi / 20))
    assert sagitta > 0.12, "precondition: the source really is a coarse polygon"

    out, _ = fit_line_chain(pts, max_spacing_m=0.05)
    # The samples sit on the facets, i.e. inside the true circle, so the best-fit
    # radius lands between the inscribed and circumscribed one. What matters is
    # that the result IS a circle: constant radius about its own centre.
    fit = _fit_circle_kasa(out)
    assert fit is not None
    cn, ce, r = fit
    assert _max_radial_dev(out, cn=cn, ce=ce, r=r) < 0.01, "output is not circular"
    assert abs(r - 11.5) < sagitta, f"recovered radius {r:.3f} is off"


def test_polygonised_circle_stays_polygonal_when_the_tolerance_is_tight():
    """max_dev_m is the operator's control over exactly that trade."""
    pts = _polygonised_circle(cn=0.0, ce=0.0, r=11.5, facets=20)
    out, _ = fit_line_chain(pts, max_spacing_m=0.05, max_dev_m=0.02)
    assert out == pts, "a tight tolerance must refuse the fit and keep raw points"


def _max_turn_deg(pts):
    worst = 0.0
    for i in range(1, len(pts) - 1):
        a, b, c = pts[i - 1], pts[i], pts[i + 1]
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        m1, m2 = math.hypot(*v1), math.hypot(*v2)
        if m1 < 1e-9 or m2 < 1e-9:
            continue
        dot = (v1[0] * v2[0] + v1[1] * v2[1]) / (m1 * m2)
        worst = max(worst, math.degrees(math.acos(max(-1.0, min(1.0, dot)))))
    return worst


def test_arc_straight_joints_carry_no_kink():
    """Regression: pinning an arc end onto the circle leaves it up to max_dev
    from the neighbouring straight's surveyed vertex. Moving that boundary point
    alone dumps the whole shift into one 2 cm tessellation step — a 60 deg spike.
    The shift must be tapered back into the runs instead."""
    pts = _road_straight_bend_straight()
    out, _ = fit_line_chain(pts, max_spacing_m=0.05)
    # A residual step remains and is inherent: the tangent point can only be
    # localised to sqrt(2 R tol) ~= 1.5 m at this radius and tolerance, which is
    # ~6 deg of the bend. What must never come back is the 60 deg spike from
    # dumping the whole endpoint correction into one tessellation step.
    assert _max_turn_deg(out) < 12.0, "kink at an arc/straight joint"


def test_closed_circle_seam_carries_no_kink():
    """A full surveyed circle's first and last points are the same joint; the
    seam must close smoothly rather than snap back onto one noisy sample."""
    pts = _polygonised_circle(cn=0.0, ce=0.0, r=11.5, facets=20)
    out, _ = fit_line_chain(pts, max_spacing_m=0.05)
    assert _max_turn_deg(out) < 5.0


def test_fitted_output_stays_within_tolerance_of_every_surveyed_point():
    """Whatever the fit does, no surveyed point may end up further than
    max_dev_m from the reconstructed path."""
    pts = _road_straight_bend_straight()
    out, _ = fit_line_chain(pts, max_spacing_m=0.05, max_dev_m=0.15)
    for p in pts:
        assert min(math.dist(p, q) for q in out) < 0.15


# --- engine integration -----------------------------------------------------

def _line_chain_segment(points):
    return PathSegment(
        segment_type=SegmentType.MARK,
        points=list(points),
        speed=0.35,
        segment_id=0,
        source_entity="csv:L_1",
        metadata={
            "geometry_type": "LINE_CHAIN",
            "line_like": True,
            "control_indices": list(range(len(points))),
        },
    )


def test_engine_flag_off_is_unchanged_straight_chords():
    """fit_arcs=False must leave a LINE_CHAIN exactly as the old pipeline did."""
    pts = _arc_points(cn=0, ce=0, r=7.0, a0_deg=0, a1_deg=90, n=8)
    seg = _line_chain_segment(pts)
    base = PathEngine(mark_spacing=0.05, optimize_order=False).plan_segments(
        [_line_chain_segment(pts)])
    off = PathEngine(mark_spacing=0.05, optimize_order=False, fit_arcs=False).plan_segments([seg])
    assert off.merged_waypoints == base.merged_waypoints


def test_engine_fit_arcs_tracks_the_true_arc():
    true = dict(cn=0.0, ce=0.0, r=7.0)
    pts = _arc_points(**true, a0_deg=0, a1_deg=90, n=9, noise=0.012, seed=5)
    seg = _line_chain_segment(pts)
    plan = PathEngine(mark_spacing=0.05, optimize_order=False, fit_arcs=True).plan_segments([seg])

    # Straight-chord densification would sag well over 10 cm; the fit stays tight.
    interior = plan.merged_waypoints[2:-2]
    assert _max_radial_dev(interior, **true) < 0.025
    # Only the two arc endpoints are must-hit.
    assert sum(plan.must_hit) == 2


def test_engine_fit_arcs_keeps_square_corners_must_hit():
    sq = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (1, 2), (0, 2)]
    seg = _line_chain_segment(sq)
    plan = PathEngine(mark_spacing=0.10, optimize_order=False, fit_arcs=True).plan_segments([seg])
    # Every surveyed vertex on the straight sides stays must-hit.
    assert sum(plan.must_hit) == len(sq)


def test_engine_fit_arcs_does_not_touch_dxf_line_geometry():
    """A plain LINE (not LINE_CHAIN) is excluded even with the flag on."""
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (0.0, 3.0)],
        speed=0.35,
        source_entity="LINE_1",
        metadata={"geometry_type": "LINE"},
    )
    on = PathEngine(mark_spacing=0.05, optimize_order=False, fit_arcs=True).plan_segments(
        [PathSegment(segment_type=SegmentType.MARK, points=[(0.0, 0.0), (0.0, 3.0)],
                     speed=0.35, source_entity="LINE_1", metadata={"geometry_type": "LINE"})])
    off = PathEngine(mark_spacing=0.05, optimize_order=False, fit_arcs=False).plan_segments([seg])
    assert on.merged_waypoints == off.merged_waypoints
