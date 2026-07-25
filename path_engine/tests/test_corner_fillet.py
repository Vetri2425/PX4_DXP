"""Corner fillet — round a surveyed corner into a drivable arc (PathEngine.fillet_corners_m).

Distinct from arc_chain: this INVENTS an arc at a corner the survey captured as
two straights, using a radius from the marking spec. It must therefore never fire
unless the operator asks for it, and never touch geometry that is already curved.
"""

import math

import pytest

from path_engine.core import PathSegment, SegmentType
from path_engine.engine import PathEngine
from path_engine.planners.corner_fillet import _corner_groups, fillet_corners


def _line(n0, e0, n1, e1, step=0.5):
    length = math.hypot(n1 - n0, e1 - e0)
    k = max(1, round(length / step))
    return [(n0 + (n1 - n0) * i / k, e0 + (e1 - e0) * i / k) for i in range(k + 1)]


def _corner(turn_deg, leg=30.0, step=0.5):
    """Two `leg`-metre straights meeting at the origin, turning `turn_deg`."""
    a = math.radians(turn_deg)
    head = _line(-leg, 0.0, 0.0, 0.0, step)
    tail = _line(0.0, 0.0, leg * math.cos(a), leg * math.sin(a), step)
    return head[:-1] + tail


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


# --- corner detection -------------------------------------------------------

def test_corner_group_sums_signed_turn_not_noise():
    """7-dp coordinate rounding puts +-1 deg dither on every vertex; summing
    absolute turns would read that as a curve. Only the signed sum is real."""
    pts = _corner(30.0)
    groups = _corner_groups(pts, math.radians(4.0))
    assert len(groups) == 1
    _, _, net = groups[0]
    assert math.degrees(abs(net)) == pytest.approx(30.0, abs=0.5)


def test_straight_line_has_no_corners():
    assert _corner_groups(_line(0, 0, 40, 0), math.radians(4.0)) == []


# --- geometry ---------------------------------------------------------------

@pytest.mark.parametrize("turn", [30.0, 90.0, -90.0, -45.0, 120.0])
def test_fillet_removes_the_corner(turn):
    pts = _corner(turn)
    assert _max_turn_deg(pts) == pytest.approx(abs(turn), abs=0.5)
    out, _ = fillet_corners(pts, None, radius_m=10.0, max_spacing_m=0.05)
    assert _max_turn_deg(out) < 1.0, "corner survived the fillet"


def test_fillet_is_tangent_to_both_straights():
    """A tangent fillet of radius R at a 90 deg corner passes R*(sqrt2-1) from
    the apex, and every arc point lies on the circle of radius R."""
    pts = _corner(90.0)
    out, _ = fillet_corners(pts, None, radius_m=10.0, max_spacing_m=0.05)
    closest = min(math.hypot(p[0], p[1]) for p in out)
    assert closest == pytest.approx(10.0 * (math.sqrt(2) - 1), abs=0.01)
    centre = (-10.0, 10.0)   # right turn: centre is 10 m east of the tangent point
    on_arc = [p for p in out
              if abs(math.hypot(p[0] - centre[0], p[1] - centre[1]) - 10.0) < 0.01]
    assert len(on_arc) > 100


def test_zero_radius_is_a_noop():
    pts = _corner(90.0)
    out, ctrl = fillet_corners(pts, None, radius_m=0.0)
    assert out == pts
    assert ctrl == list(range(len(pts)))


def test_fillet_never_touches_a_curve():
    """A run that is already an arc has no straight either side of any vertex, so
    the straightness guard must refuse it. Without that guard the direction is
    measured along the curve and the 'fillet' becomes a >100 deg spike."""
    arc = [(12.0 * math.sin(math.radians(t)), 12.0 * math.cos(math.radians(t)))
           for t in [i * 0.6 for i in range(150)]]
    out, _ = fillet_corners(arc, None, radius_m=10.0, max_spacing_m=0.05)
    assert out == arc


def test_radius_is_reduced_not_overrun_when_corners_are_close():
    """Two corners 6 m apart cannot both take a 10 m fillet (which needs 10 m of
    tangent each at 90 deg). The radius shrinks; the fillets must not overlap or
    reverse the path."""
    pts = _line(-30, 0, 0, 0)[:-1] + _line(0, 0, 0, 6)[:-1] + _line(0, 6, 30, 6)
    out, _ = fillet_corners(pts, None, radius_m=10.0, max_spacing_m=0.05)
    assert _max_turn_deg(out) < _max_turn_deg(pts)
    # Monotone along the path: no doubling back.
    for i in range(len(out) - 1):
        assert math.dist(out[i], out[i + 1]) > 1e-9
    # Total length stays sane (a fillet shortens slightly, never explodes).
    length = sum(math.dist(out[i], out[i + 1]) for i in range(len(out) - 1))
    original = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    assert 0.7 * original < length < original + 1.0


def test_surveyed_vertex_inside_the_fillet_stops_being_must_hit():
    """Rounding a corner cuts the corner off, so that surveyed point is no longer
    on the path and must not still be advertised as must-hit."""
    pts = _corner(90.0)
    apex = pts.index((0.0, 0.0))
    out, ctrl = fillet_corners(pts, list(range(len(pts))), radius_m=10.0,
                               max_spacing_m=0.05)
    assert all(math.dist(out[i], (0.0, 0.0)) > 1.0 for i in ctrl), \
        "a control point still sits at the cut-off apex"
    assert len(ctrl) >= 2   # the two tangent points survive


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


def test_engine_default_is_off():
    pts = _corner(90.0)
    base = PathEngine(mark_spacing=0.05, optimize_order=False).plan_segments(
        [_line_chain_segment(pts)])
    off = PathEngine(mark_spacing=0.05, optimize_order=False,
                     fillet_corners_m=0.0).plan_segments([_line_chain_segment(pts)])
    assert off.merged_waypoints == base.merged_waypoints


def test_engine_fillet_rounds_a_line_chain_corner():
    pts = _corner(90.0)
    plan = PathEngine(mark_spacing=0.05, optimize_order=False,
                      fillet_corners_m=8.0).plan_segments([_line_chain_segment(pts)])
    assert _max_turn_deg(plan.merged_waypoints) < 2.0


def test_engine_fillet_leaves_dxf_geometry_alone():
    """Only LINE_CHAIN (survey) geometry is eligible — a DXF LINE is untouched
    even with a radius set."""
    def seg():
        return PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 0.0), (3.0, 0.0), (3.0, 3.0)],
            speed=0.35, source_entity="LINE_1",
            metadata={"geometry_type": "LINE"},
        )
    on = PathEngine(mark_spacing=0.05, optimize_order=False,
                    fillet_corners_m=1.0).plan_segments([seg()])
    off = PathEngine(mark_spacing=0.05, optimize_order=False).plan_segments([seg()])
    assert on.merged_waypoints == off.merged_waypoints


def test_engine_rejects_a_negative_radius():
    with pytest.raises(ValueError, match="fillet_corners_m"):
        PathEngine(fillet_corners_m=-1.0)
