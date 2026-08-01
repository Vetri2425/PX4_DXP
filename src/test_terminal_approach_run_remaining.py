#!/usr/bin/env python3
"""Goal-approach deceleration is measured to the RUN END, not the segment end.

Regression guard for bag stg_9ecf2985 (2026-08-01 15:47, test_line_2): the rover
overshot its endpoint by 54.6 cm — 64.6 cm past the painted line.

The approach ramp was gated on `final_segment` and scaled by `dist_to_corner`,
the distance to the end of the CURRENT segment. Its zone (approach_d, ~0.9 m)
was therefore silently clipped to the length of whatever the last segment
happened to be. That was harmless while the aft run-out was its own run, but
a54fd2d fuses a short transit tail into the mark, making the 0.10 m run-out the
final segment: the ramp got 0.10 m of its 0.9 m, and the 2.337 m mark ahead of
it was driven at full command to within 6 cm of its end.

These tests drive the REAL node. A mirror of the arithmetic would have
reproduced the bug just as faithfully as the code did — the whole defect was
WHICH DISTANCE was fed in, not how it was scaled.

Run on a ROS2-sourced host (needs rclpy):
    python3 -m pytest src/test_terminal_approach_run_remaining.py
"""
import math

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.parameter import Parameter  # noqa: E402


MAX_V = 0.7          # mission_speed on the failing run
MARK_LEN = 2.337     # painted segment
RUNOUT = 0.100       # unpainted aft run-out — the segment the ramp was clipped to


def _pose(n, e):
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    ps.pose.position.x = float(n)
    ps.pose.position.y = float(e)
    ps.pose.orientation.w = 1.0
    return ps


@pytest.fixture(scope="module")
def node():
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    from rpp_controller_node import RPPControllerNode
    n = RPPControllerNode()
    n.set_parameters([Parameter("max_linear_vel", value=MAX_V)])
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _commanded_speed(node, north, *, path, cum_s, flags, seg_idx,
                     run_len=None, closed=False, break_cache=False,
                     no_runs=False):
    """Steady commanded speed with the rover at `north` on a due-north path."""
    captured = {}
    node._publish_velocity = lambda vn, ve: captured.update(sp=math.hypot(vn, ve))
    node._reset_corner_pivot_state()
    node._path = [_pose(n, 0.0) for n in path]
    # A cache out of step with _path is the degraded case the fallback exists for.
    node._path_s = list(cum_s)[:-1] if break_cache else list(cum_s)
    node._spray_flags = list(flags)
    node._segment_idx = seg_idx
    node._path_travel_m = north
    node._latest_yaw_rate_ned = 0.0
    node._last_speed_cmd = MAX_V          # decel is unbounded; ramp only caps rises
    node._run_idx = 0
    node._runs = [] if no_runs else [{
        "length": float(run_len if run_len is not None else cum_s[-1]),
        "cum_s": list(cum_s),
        "closed": bool(closed),
        "flags": list(flags),
    }]
    sp = float("nan")
    for _ in range(40):
        captured.clear()
        node._control_segment_profile(north, 0.0, 0.0, 0.0, cum_s[-1] - north)
        sp = captured.get("sp", float("nan"))
    return sp


# The failing geometry: one fused run, mark then a 0.10 m transit run-out.
FUSED = dict(
    path=[0.0, MARK_LEN, MARK_LEN + RUNOUT],
    cum_s=[0.0, MARK_LEN, MARK_LEN + RUNOUT],
    flags=[True, True, False],
    seg_idx=0,
)


def test_brakes_while_still_on_the_mark_segment(node):
    """0.80 m from the RUN END the rover must already be slowing.

    It is still on segment 0 here (0.70 m from the mark's own end), so the old
    `final_segment` gate was False and this commanded full speed. That is the
    bug: all braking was deferred to a 0.10 m segment.
    """
    north = MARK_LEN + RUNOUT - 0.80
    sp = _commanded_speed(node, north, **FUSED)
    assert sp < MAX_V - 0.02, f"no deceleration 0.80 m from the run end: {sp:.3f} m/s"


def test_speed_falls_monotonically_across_the_segment_boundary(node):
    """The ramp must not reset or step when segment 0 hands over to the run-out.

    A per-segment measure is discontinuous at the joint — it jumps back up to
    the new segment's length. Measured to the run end it is continuous.
    """
    total = MARK_LEN + RUNOUT
    prev = None
    for remaining in [0.90, 0.70, 0.50, 0.30, 0.15, 0.10, 0.05, 0.0]:
        north = total - remaining
        seg_idx = 0 if north < MARK_LEN else 1
        sp = _commanded_speed(node, north, **{**FUSED, "seg_idx": seg_idx})
        if prev is not None:
            assert sp <= prev + 1e-6, (
                f"speed rose at {remaining:.2f} m remaining: {prev:.3f} -> {sp:.3f}"
            )
        prev = sp


def test_arrives_slow_enough_to_stop_in_the_run_out(node):
    """At the mark end the rover must be slow enough to stop inside 0.10 m.

    The bag measured 0.57 m/s^2 of real deceleration, so 0.10 m buys ~0.34 m/s.
    It entered at 0.79 m/s and needed 0.55 m.
    """
    sp = _commanded_speed(node, MARK_LEN, **FUSED)
    stopping_m = sp * sp / (2.0 * 0.57)
    assert stopping_m <= RUNOUT, (
        f"entering the run-out at {sp:.3f} m/s needs {stopping_m*100:.0f} cm, has "
        f"{RUNOUT*100:.0f} cm"
    )


def test_long_final_segment_is_unchanged(node):
    """A plain 2 m line with no run-out keeps its previous profile.

    Here segment end and run end coincide, so the two measures agree and this
    pins that the change is inert on the ordinary case.
    """
    sp_far = _commanded_speed(node, 1.0, path=[0.0, 2.0], cum_s=[0.0, 2.0],
                              flags=[True, True], seg_idx=0)
    sp_near = _commanded_speed(node, 1.9, path=[0.0, 2.0], cum_s=[0.0, 2.0],
                               flags=[True, True], seg_idx=0)
    assert sp_far == pytest.approx(MAX_V, abs=1e-6), f"premature braking: {sp_far:.3f}"
    assert sp_near < 0.15, f"should be crawling 0.10 m out, got {sp_near:.3f}"


def test_closed_run_is_not_throttled_at_its_seam(node):
    """A circle starts AND ends at the seam — it must not brake from cycle 0.

    Field bag 20260613_200921 crept at ~3 cm/s for 118 s and never went around,
    because a Euclidean dist-to-goal reads ~0 at the seam. The along-run
    remaining distance is what makes this safe, so it is pinned here.
    """
    circ = 6.0
    sp = _commanded_speed(node, 0.05, path=[0.0, 3.0, 6.0], cum_s=[0.0, 3.0, circ],
                          flags=[True, True, True], seg_idx=0,
                          run_len=circ, closed=True)
    assert sp == pytest.approx(MAX_V, abs=1e-6), f"throttled at the seam: {sp:.3f}"


def test_degrades_to_the_old_measure_when_the_cache_is_unusable(node):
    """A broken progress cache must not remove braking that already worked.

    _run_remaining_along returns None here; the fallback is the previous
    final-segment measure, so the run-out still brakes even though the early
    (segment-0) braking is lost.
    """
    for kwargs in ({"break_cache": True}, {"no_runs": True}):
        sp_final_seg = _commanded_speed(
            node, MARK_LEN + 0.05, **{**FUSED, "seg_idx": 1}, **kwargs
        )
        assert sp_final_seg < MAX_V, f"lost endpoint braking with {kwargs}: {sp_final_seg:.3f}"
