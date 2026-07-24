"""close_shape: paint the closing side of an open MARK shape (PathEngine.close_shape)."""

import math

import pytest

from path_engine.core import PathSegment, SegmentType
from path_engine.engine import PathEngine


def _line_chain(points, closed_hint=False):
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


# Open 4-corner square (first != last): sides = 3 open, 4 when closed.
_SQUARE = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]


def _engine(**kw):
    return PathEngine(mark_spacing=0.10, optimize_order=False, **kw)


def test_open_square_gains_a_painted_closing_side():
    off = _engine(close_shape=False).plan_segments([_line_chain(_SQUARE)])
    on = _engine(close_shape=True).plan_segments([_line_chain(_SQUARE)])

    # Three sides open -> ~6 m of paint; four sides closed -> ~8 m.
    assert off.total_mark_length == pytest.approx(6.0, abs=0.05)
    assert on.total_mark_length == pytest.approx(8.0, abs=0.05)


def test_closed_path_first_equals_last():
    on = _engine(close_shape=True).plan_segments([_line_chain(_SQUARE)])
    wp = on.merged_waypoints
    assert math.dist(wp[0], wp[-1]) < 1e-6


def test_the_closing_side_is_sprayed_not_deadhead():
    on = _engine(close_shape=True).plan_segments([_line_chain(_SQUARE)])
    # A single MARK shape: every merged waypoint is spray-on. close_loop, by
    # contrast, would append a spray-OFF TRANSIT run.
    assert all(on.spray_flags)


def test_closing_point_is_must_hit():
    on = _engine(close_shape=True).plan_segments([_line_chain(_SQUARE)])
    # Four surveyed corners plus the appended closing point (a control point, per
    # the plan) = five must-hit flags. The rover must reach the start corner to
    # actually close the shape, so it is must-hit at both ends of the path.
    assert sum(on.must_hit) == 5
    assert on.must_hit[0] and on.must_hit[-1]


def test_open_stripe_is_untouched():
    """A 2-point open stripe has no shape to close."""
    stripe = [(0.0, 0.0), (0.0, 3.0)]
    off = _engine(close_shape=False).plan_segments([_line_chain(stripe)])
    on = _engine(close_shape=True).plan_segments([_line_chain(stripe)])
    assert on.merged_waypoints == off.merged_waypoints


def test_already_closed_shape_is_a_noop():
    closed = _SQUARE + [(0.0, 0.0)]     # first == last already
    off = _engine(close_shape=False).plan_segments([_line_chain(closed)])
    on = _engine(close_shape=True).plan_segments([_line_chain(closed)])
    assert on.merged_waypoints == off.merged_waypoints


def test_flag_off_is_byte_for_byte_unchanged():
    base = _engine().plan_segments([_line_chain(_SQUARE)])
    off = _engine(close_shape=False).plan_segments([_line_chain(_SQUARE)])
    assert off.merged_waypoints == base.merged_waypoints
    assert off.must_hit == base.must_hit


def test_close_shape_composes_with_fit_arcs():
    """Both flags on: arc runs fit, then the shape still closes."""
    on = _engine(fit_arcs=True, close_shape=True).plan_segments([_line_chain(_SQUARE)])
    wp = on.merged_waypoints
    assert math.dist(wp[0], wp[-1]) < 1e-6         # closed
    assert on.total_mark_length == pytest.approx(8.0, abs=0.05)  # square stays square
