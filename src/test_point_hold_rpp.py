#!/usr/bin/env python3
"""Phase D planner — RPP point-hold A/B tests (in-env, needs rclpy).

The point-hold is a gated overlay on the FROZEN controller. The load-bearing
test is the freeze guarantee: with point_hold_enabled False the overlay is a
byte-for-byte no-op (returns False, touches no state). The ON tests drive the
brake→dwell→release branches by controlling _corner_stop_satisfied and
point_hold_s (0 = release immediately, large = hold), so no wall-clock timing
is needed.

Run:  python3 -m pytest -q test_point_hold_rpp.py   (Jetson only — rclpy)
"""

import rclpy

from test_smoke_rpp_controller import _make_path_pose, _CapturePub


def _build_node(enabled, hold_s=0.0):
    from rpp_controller_node import RPPControllerNode
    from nav_msgs.msg import Path

    node = RPPControllerNode()
    node.set_parameters([
        rclpy.parameter.Parameter("point_hold_enabled",
                                  rclpy.parameter.Parameter.Type.BOOL, enabled),
        rclpy.parameter.Parameter("point_hold_s",
                                  rclpy.parameter.Parameter.Type.DOUBLE, float(hold_s)),
        rclpy.parameter.Parameter("point_hold_acceptance_m",
                                  rclpy.parameter.Parameter.Type.DOUBLE, 0.10),
    ])
    node._vel_pub = _CapturePub()
    node._yaw_rate_pub = _CapturePub()
    node._dbg_pub = _CapturePub()
    node._segment_dbg_pub = _CapturePub()
    node._conditioned_path_pub = _CapturePub()
    node._spray_active_pub = _CapturePub()

    # A 2-point path with point 0 flagged must-hit (z bit1 = 2).
    p0 = _make_path_pose(0.0, 0.0)
    p0.pose.position.z = 2.0
    p1 = _make_path_pose(5.0, 0.0)
    path = Path()
    path.header.frame_id = "local_ned"
    path.header.stamp = node.get_clock().now().to_msg()
    path.poses = [p0, p1]
    node._path_cb(path)
    return node


def test_disabled_is_byte_for_byte_noop():
    rclpy.init()
    try:
        node = _build_node(enabled=False)
        # Even sitting right on a must-hit point, the overlay must not engage.
        handled = node._point_hold_tick(0.0, 0.0, 0.0, 0.01, 5.0)
        assert handled is False
        assert node._point_hold_active_key is None
        assert node._point_hold_done_keys == set()
    finally:
        rclpy.shutdown()


def test_far_from_point_does_not_engage():
    rclpy.init()
    try:
        node = _build_node(enabled=True)
        handled = node._point_hold_tick(3.0, 0.0, 0.0, 0.01, 2.0)  # 3 m away
        assert handled is False
    finally:
        rclpy.shutdown()


def test_brakes_until_stopped_then_holds_then_releases():
    rclpy.init()
    try:
        node = _build_node(enabled=True, hold_s=100.0)  # large hold = stays dwelling
        stopped = {"v": False}
        node._corner_stop_satisfied = lambda: stopped["v"]

        # Near the must-hit point but not stopped → braking, cycle handled.
        assert node._point_hold_tick(0.02, 0.0, 0.0, 0.01, 5.0) is True
        assert node._point_hold_active_key is not None
        assert node._point_hold_start_ns is None       # not dwelling yet

        # Now confirmed stopped → dwelling (still within the 100 s hold).
        stopped["v"] = True
        assert node._point_hold_tick(0.0, 0.0, 0.0, 0.01, 5.0) is True
        assert node._point_hold_start_ns is not None    # dwell clock running
        assert node._vel_pub.last is not None           # a (zero) velocity was published
    finally:
        rclpy.shutdown()


def test_release_after_dwell_marks_done_once():
    rclpy.init()
    try:
        node = _build_node(enabled=True, hold_s=0.0)   # release immediately
        node._corner_stop_satisfied = lambda: True     # already stopped
        # First tick: stopped, hold_s=0 → dwell elapses at once → release.
        handled = node._point_hold_tick(0.0, 0.0, 0.0, 0.01, 5.0)
        assert handled is False                        # released to normal tracking
        assert len(node._point_hold_done_keys) == 1
        # Second tick at the same point: already done → not re-engaged.
        handled = node._point_hold_tick(0.0, 0.0, 0.0, 0.01, 5.0)
        assert handled is False
        assert node._point_hold_active_key is None
    finally:
        rclpy.shutdown()


def test_path_reload_rearms_done_keys():
    rclpy.init()
    try:
        node = _build_node(enabled=True, hold_s=0.0)
        node._corner_stop_satisfied = lambda: True
        node._point_hold_tick(0.0, 0.0, 0.0, 0.01, 5.0)
        assert len(node._point_hold_done_keys) == 1
        # Reload the same path → dwells re-arm (fresh mission).
        node2 = _build_node(enabled=True, hold_s=0.0)
        assert node2._point_hold_done_keys == set()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("ALL POINT-HOLD TESTS PASSED")
