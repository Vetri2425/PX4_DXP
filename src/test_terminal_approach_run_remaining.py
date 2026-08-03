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
    # Pin BOTH halves of the effective ceiling min(max_linear_vel, mission_speed):
    # this test reproduces a 0.7 m/s run and must not drift with the node's
    # mission_speed default (0.7 -> 0.5 at cfbf5f3 broke exactly that).
    n.set_parameters([Parameter("max_linear_vel", value=MAX_V),
                      Parameter("mission_speed", value=MAX_V)])
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


@pytest.mark.parametrize("remaining", [0.80, 0.60, 0.40, 0.25])
def test_brakes_while_still_on_the_mark_segment(node, remaining):
    """THE REGRESSION TEST. Braking must track the distance to the RUN END.

    At every one of these the rover is still on segment 0 — 0.15 m to 0.70 m
    from the mark's own end — so the old `final_segment` gate was False and all
    four commanded a flat 0.700 m/s. All braking was deferred to a 0.10 m
    segment that cannot absorb it.

    Pinning the ramp VALUE, not merely "less than cruise": the defect was which
    distance was fed in, so the test has to prove the right one is.
    """
    approach_d = max(0.9, MAX_V * MAX_V / (2.0 * 0.5) + 0.10)
    north = MARK_LEN + RUNOUT - remaining
    assert north < MARK_LEN, "must still be on the mark segment to discriminate"
    sp = _commanded_speed(node, north, **FUSED)
    assert sp == pytest.approx(MAX_V * remaining / approach_d, abs=0.02), (
        f"{remaining:.2f} m from the run end: commanded {sp:.3f} m/s"
    )


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


def test_off_switch_restores_the_previous_behaviour(node):
    """endpoint_approach_run_remaining=False must reproduce the old profile.

    Every fix in this stack carries a field off switch and the checklist's
    FALLBACKS section depends on it. Pinned by asserting the OLD (wrong)
    number comes back, so a switch that silently does nothing fails here.
    """
    north = MARK_LEN + RUNOUT - 0.80
    node.set_parameters([Parameter("endpoint_approach_run_remaining", value=False)])
    try:
        sp = _commanded_speed(node, north, **FUSED)
        assert sp == pytest.approx(MAX_V, abs=1e-6), (
            f"off switch did not restore per-segment braking: {sp:.3f} m/s"
        )
    finally:
        node.set_parameters([Parameter("endpoint_approach_run_remaining", value=True)])
    assert _commanded_speed(node, north, **FUSED) < MAX_V - 0.02, "switch did not re-enable"


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


# ── 2026-08-01 (later): the run-out relaxation applied where it must not ──────
#
# `_run_tail_is_transit` was `bool(flags) and not flags[-1]`, which is also true
# of a run that paints NOWHERE — the approach leg that carries the rover to the
# mission start. It inherited the 0.10 m relaxed tolerance and stopped 7.4-8.5 cm
# short of the mission start (was 1.4 cm before). And 0.10 m is EXACTLY the
# standard run-out length, so on the mark run the tolerance swallowed the whole
# run-out: the rover parked ON the wet end of the line.

def _install(node, path, flags):
    """Install a run WITHOUT depending on the new helper existing.

    These tests must discriminate on BEHAVIOUR, not on the presence of a new
    method: against the unfixed controller they have to fail with the wrong
    tolerance (0.10 where 0.02 is required), not with AttributeError.
    """
    node._path = [_pose(n, 0.0) for n, _ in path]
    node._spray_flags = list(flags)
    measure = getattr(node, "_measure_tail_transit_m", None)
    node._run_tail_transit_m = measure() if measure else 0.0


def test_approach_leg_keeps_full_endpoint_precision(node):
    """A run that paints NOWHERE is not a run-out — no relaxation.

    Its endpoint is where paint begins, the one place that most needs the 2 cm
    tolerance.
    """
    _install(node, [(0.0, 0), (3.0, 0)], [False, False])
    assert node._run_tail_is_transit() is False, "approach leg wrongly treated as a run-out"
    assert node._goal_tol_effective(0.02) == pytest.approx(0.02), \
        "approach leg must not inherit the run-out tolerance"


def test_runout_relaxation_never_swallows_the_runout(node):
    """The tolerance must stay under the segment it guards.

    0.10 m tolerance on a 0.10 m run-out means DONE fires at the mark end and
    the run-out is never entered.
    """
    _install(node, [(0.0, 0), (2.337, 0), (2.437, 0)], [True, True, False])
    assert node._run_tail_is_transit() is True
    tol = node._goal_tol_effective(0.02)
    assert tol == pytest.approx(0.05, abs=1e-6), f"expected half the tail, got {tol}"
    assert tol < 0.10, "tolerance must be INSIDE the 0.10 m run-out"


def test_long_aft_extension_still_gets_the_full_param(node):
    """Self-scaling: a 0.9 m run-out is long enough for the whole 0.10 m."""
    _install(node, [(0.0, 0), (3.0, 0), (3.9, 0)], [True, True, False])
    assert node._goal_tol_effective(0.02) == pytest.approx(0.10, abs=1e-6)


def test_run_ending_on_paint_is_unchanged(node):
    """A run that ends ON a painted point keeps the strict tolerance."""
    _install(node, [(0.0, 0), (2.0, 0)], [True, True])
    assert node._goal_tol_effective(0.02) == pytest.approx(0.02)
