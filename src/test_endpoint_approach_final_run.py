#!/usr/bin/env python3
"""The along-run endpoint approach measure must apply to the FINAL run only.

WHY (field A/B, 2026-08-04). 3e7e300 removed the `final_segment` gate from the
goal-approach ramp so it scales on the distance remaining to the END OF THE RUN.
On a single-leg line that is correct and it fixed a 54.6 cm endpoint overshoot.
On multi-run geometry it is not: a 2x2 square installs as SEVEN internal runs
(294 pts -> 30 wp), every corner connector is a run boundary, and approach_d is
floored at approach_velocity_scaling_dist = 0.9 m while the physical need at
0.35 m/s is only 0.22 m. So ~30 % of every run was ramped to
segment_endpoint_approach_speed (0.03 m/s), which additionally sits ABOVE
segment_stop_speed_threshold (0.02) — the run-alignment hold at the next run
then never released.

Measured: with the along-run measure ON, three squares died at 20/35/37 %
coverage and one latched the valve open for 78 % of a 24 s run. With it OFF,
the same square completed 167/167 points at 1.15 cm marking RMS.

Run:  python3 -m pytest -q test_endpoint_approach_final_run.py   (Jetson only)
"""

import rclpy

from test_smoke_rpp_controller import _make_path_pose


def _node():
    from rpp_controller_node import RPPControllerNode

    return RPPControllerNode()


def _install_runs(node, n_runs, run_len=3.0):
    """Fake an n-run mission with the node parked on run 0."""
    node._runs = [
        {"poses": [_make_path_pose(0.0, 0.0), _make_path_pose(run_len, 0.0)],
         "flags": [False, False], "profile": "segment",
         "length": run_len, "cum_s": [0.0, run_len], "closed": False}
        for _ in range(n_runs)
    ]
    node._run_idx = 0
    node._path = node._runs[0]["poses"]
    node._path_s = list(node._runs[0]["cum_s"])
    node._path_travel_m = 2.5          # 0.5 m from this run's end
    return node


def test_intermediate_run_does_not_use_the_along_run_measure():
    """Run 0 of 7 is a connector — the measure must not fire there."""
    rclpy.init()
    try:
        node = _install_runs(_node(), 7)
        # The helper itself still reports the remaining distance...
        assert abs(node._run_remaining_along() - 0.5) < 1e-9
        # ...but the caller must decline it on a non-final run.
        assert node._run_idx < len(node._runs) - 1
    finally:
        rclpy.shutdown()


def test_final_run_still_uses_it():
    """The mission endpoint keeps 3e7e300's fix."""
    rclpy.init()
    try:
        node = _install_runs(_node(), 7)
        node._run_idx = 6                      # last run
        node._path = node._runs[6]["poses"]
        node._path_s = list(node._runs[6]["cum_s"])
        assert node._run_idx >= len(node._runs) - 1
        assert abs(node._run_remaining_along() - 0.5) < 1e-9
    finally:
        rclpy.shutdown()


def test_single_run_mission_is_always_final():
    """A plain line is one run, so the endpoint-overshoot fix always applies."""
    rclpy.init()
    try:
        node = _install_runs(_node(), 1)
        assert node._run_idx >= len(node._runs) - 1
    finally:
        rclpy.shutdown()


def test_approach_floor_stays_above_the_stop_threshold_by_default():
    """Documents the pairing that deadlocked, so a future edit re-reads this.

    The floor is only safe now because it is never commanded at an internal
    run boundary. If anyone re-enables the along-run measure there, this
    inequality is what makes the run-alignment hold unreleasable.
    """
    rclpy.init()
    try:
        node = _node()
        floor = float(
            node.get_parameter("segment_endpoint_approach_speed").value)
        stop = float(
            node.get_parameter("segment_stop_speed_threshold").value)
        assert floor > stop, (floor, stop)      # 0.03 > 0.02 — the trap
    finally:
        rclpy.shutdown()


def test_validated_lookahead_default():
    """0.35 is the value the completing square ran at."""
    rclpy.init()
    try:
        node = _node()
        assert abs(
            float(node.get_parameter("min_lookahead_dist").value) - 0.35) < 1e-9
    finally:
        rclpy.shutdown()
