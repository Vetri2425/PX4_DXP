#!/usr/bin/env python3
"""The terminal MARK->TRANSIT boundary must land on the MARK END, not the tail.

The bug (2026-07-30): the engine's terminal run-out block emitted ONE point —
the far end of the tail — flagged TRANSIT. But `spray_flags[i]` is the flag of
the segment LEAVING point i, so a MARK->TRANSIT boundary has to be a DUPLICATED
coincident vertex (mark end as True, then again as False). With only the far
point emitted, the flag transition sat on the far end and the whole run-out read
as MARK: the spray node placed its terminal boundary `runout` metres too far
along and held the valve open for the entire tail.

Field evidence — /spray/debug[7] is `boundary.s`, the station the spray node
believed was the mark end:

    bag                       run-out  true end  boundary.s  overspray
    stg_21a505b9_..._150612     0.1     3.0687     3.1687     +9.4 cm
    stg_21a505b9_..._150750     0.1     3.0687     3.1687     +9.6 cm
    stg_8b8cf09c_..._174708     0.1     2.3372     2.4372     +9.5 cm
    stg_8b8cf09c_..._174916     0.1     2.3372     2.4372    +10.0 cm
    stg_1bda4c36_..._151006     1.0     3.5687     3.5687 OK  -0.8 cm

Overspray == the run-out length. The clean run is the extensions-ON case, whose
AFT run-out is a real TRANSIT PathSegment through the merge loop and therefore
already carried this vertex.

NOT a speed effect: the OFF lead is solenoid_close_delay_s * v = 0.05*v, worth
~1 cm of the 10. Even at full 0.35 m/s a 0.1 m tail oversprays ~8.2 cm. Speed
and run-out length are confounded in field data because a rover braking to rest
inside 0.1 m is always slow at the mark end.

These tests assert the INVARIANT the spray node consumes (where does the flag
first go False, in arc length?) rather than the point count, so they keep working
if the tail geometry is retuned.

Run:  python3 -m pytest path_engine/tests/test_terminal_boundary_vertex.py
"""
import math

from path_engine.core import PathSegment, SegmentType
from path_engine.engine import PathEngine


def _mark(points):
    return PathSegment(segment_type=SegmentType.MARK, points=list(points), speed=0.35)


def _transit(points):
    return PathSegment(segment_type=SegmentType.TRANSIT, points=list(points), speed=0.5)


def _cumulative_s(pts):
    s = [0.0]
    for i in range(len(pts) - 1):
        s.append(s[-1] + math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]))
    return s


def _first_transit_s(plan):
    """Arc length at which the flag first goes False.

    This is exactly how the spray node derives its MARK->TRANSIT station
    (`_build_path_model` in src/spray_controller_node.py), so it is the quantity
    the defect actually corrupted.
    """
    s = _cumulative_s(plan.merged_waypoints)
    for i, f in enumerate(plan.spray_flags):
        if not f:
            return s[i]
    return None


def _plan_single_line(length_m=3.0, **kw):
    eng = PathEngine(optimize_order=False, compensate_spray=False, **kw)
    return eng.plan_segments([_mark([(0.0, 0.0), (length_m, 0.0)])])


class TestBoundaryLandsOnTheMarkEnd:
    def test_boundary_is_at_the_mark_end_not_the_tail(self):
        plan = _plan_single_line(3.0)
        assert _first_transit_s(plan) == 3.0, (
            "the MARK->TRANSIT station must be the mark end; if it is 3.1 the "
            "run-out is being encoded as MARK and the valve stays open for it"
        )

    def test_mark_length_is_unchanged_by_the_terminator(self):
        plan = _plan_single_line(3.0)
        assert plan.total_mark_length == 3.0
        assert round(plan.total_transit_length, 6) == 0.1

    def test_the_terminator_is_coincident_with_the_mark_end(self):
        plan = _plan_single_line(3.0)
        w = plan.merged_waypoints
        # last three: mark end (True), terminator (False), tail (False)
        assert plan.spray_flags[-3] is True
        assert plan.spray_flags[-2] is False
        assert plan.spray_flags[-1] is False
        assert math.hypot(w[-2][0] - w[-3][0], w[-2][1] - w[-3][1]) < 1e-9
        assert round(math.hypot(w[-1][0] - w[-2][0], w[-1][1] - w[-2][1]), 6) == 0.1

    def test_terminator_is_not_must_hit(self):
        plan = _plan_single_line(3.0)
        assert plan.must_hit[-1] is False
        assert plan.must_hit[-2] is False

    def test_holds_for_a_longer_runout(self):
        """extensions ON widens the tail; the boundary must not move with it."""
        plan = _plan_single_line(3.0, enable_path_extensions=True,
                                 pre_extension_m=0.5, aft_extension_m=0.5)
        s = _first_transit_s(plan)
        marked = [p for p, f in zip(plan.merged_waypoints, plan.spray_flags) if f]
        # The boundary sits at the end of the marked span, wherever extensions
        # put that — not at the far end of the tail.
        assert s is not None
        assert s == max(_cumulative_s(plan.merged_waypoints)[i]
                        for i, f in enumerate(plan.spray_flags) if f)
        assert marked, "extensions ON must still mark something"

    def test_the_defect_would_fail_this(self):
        """Sensitivity guard — recompute the pre-fix encoding and show it breaks.

        If this ever stops failing the assertions above have gone blind.
        """
        plan = _plan_single_line(3.0)
        w = list(plan.merged_waypoints)
        f = list(plan.spray_flags)
        # Undo the terminator: drop the duplicated vertex, as the old code did.
        del w[-2]
        del f[-2]
        s = _cumulative_s(w)
        old_boundary = next(s[i] for i, x in enumerate(f) if not x)
        assert round(old_boundary, 6) == 3.1, old_boundary
        assert old_boundary > 3.0, "pre-fix boundary sat past the mark end"


class TestInteriorBoundariesUnaffected:
    def test_mark_transit_mark_keeps_its_interior_boundaries(self):
        eng = PathEngine(optimize_order=False, compensate_spray=False)
        plan = eng.plan_segments([
            _mark([(0.0, 0.0), (3.0, 0.0)]),
            _transit([(3.0, 0.0), (4.0, 0.0)]),
            _mark([(4.0, 0.0), (7.0, 0.0)]),
        ])
        s = _cumulative_s(plan.merged_waypoints)
        flips = [s[i] for i in range(1, len(plan.spray_flags))
                 if plan.spray_flags[i] != plan.spray_flags[i - 1]]
        # 0->3 MARK, 3->4 TRANSIT, 4->7 MARK, then the terminal boundary at 7.
        assert [round(x, 6) for x in flips] == [3.0, 4.0, 7.0]

    def test_closed_shape_gets_no_runout_and_no_terminator(self):
        """A shape ending where it started must not grow a spur."""
        eng = PathEngine(optimize_order=False, compensate_spray=False)
        plan = eng.plan_segments([
            _mark([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]),
        ])
        w = plan.merged_waypoints
        assert math.hypot(w[-1][0] - w[0][0], w[-1][1] - w[0][1]) < 0.01
        assert all(plan.spray_flags), "closed shape must stay entirely MARK"
