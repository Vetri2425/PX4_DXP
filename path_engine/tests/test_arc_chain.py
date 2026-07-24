"""Arc fit for surveyed LINE_CHAINs (PathEngine.fit_arcs)."""

import math
import random

import pytest

from path_engine.core import PathSegment, SegmentType
from path_engine.engine import PathEngine
from path_engine.planners.arc_chain import (
    _fit_circle_kasa,
    _max_chord_deviation,
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


def test_short_chain_is_a_noop():
    assert fit_line_chain([(0, 0), (1, 1)]) == ([(0, 0), (1, 1)], [0, 1])


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
