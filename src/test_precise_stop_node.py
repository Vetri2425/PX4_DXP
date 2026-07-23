#!/usr/bin/env python3
"""G3.6 — precise-stop node glue tests (in-env, needs rclpy).

Drives the real RPPControllerNode through `_point_hold_tick` /
`_precise_stop_ready` with injected pose + velocity. The frozen guarantee is the
load-bearing case: with point_precise_stop_enabled False the point-hold overlay
is byte-for-byte the pre-G3 brake-when-near path. The ON tests exercise the
feed-forward decel profile, the at-point+stopped dwell gate, and the servo creep
+ timeout backstop.

Deterministic timing: precise_stop_max_s is set to 0.0 to fire the servo
timeout on the first creep tick, so no wall-clock waiting is needed.

Run:  python3 -m pytest -q test_precise_stop_node.py   (Jetson only — rclpy)
"""

import math

import rclpy
from rclpy.parameter import Parameter

from test_smoke_rpp_controller import _make_path_pose, _CapturePub


def _p(name, value):
    if isinstance(value, bool):
        return Parameter(name, Parameter.Type.BOOL, value)
    if isinstance(value, str):
        return Parameter(name, Parameter.Type.STRING, value)
    return Parameter(name, Parameter.Type.DOUBLE, float(value))


def _build_node(**params):
    from rpp_controller_node import RPPControllerNode
    from nav_msgs.msg import Path

    node = RPPControllerNode()
    defaults = {
        "point_hold_enabled": True,
        "point_hold_s": 100.0,               # large → stays dwelling once reached
        "point_hold_acceptance_m": 0.10,
        "point_precise_stop_enabled": True,
        "point_arrival_tolerance_m": 0.02,
        "precise_stop_mode": "feedforward",
        "precise_stop_decel_m_s2": 0.30,
        "precise_stop_creep_speed": 0.05,
        "precise_stop_max_s": 8.0,
    }
    defaults.update(params)
    node.set_parameters([_p(k, v) for k, v in defaults.items()])
    for attr in ("_vel_pub", "_yaw_rate_pub", "_dbg_pub", "_segment_dbg_pub",
                 "_conditioned_path_pub", "_spray_active_pub"):
        setattr(node, attr, _CapturePub())

    # 2-point path; point 0 at the origin flagged must-hit (z bit1 = 2).
    p0 = _make_path_pose(0.0, 0.0)
    p0.pose.position.z = 2.0
    p1 = _make_path_pose(5.0, 0.0)
    path = Path()
    path.header.frame_id = "local_ned"
    path.header.stamp = node.get_clock().now().to_msg()
    path.poses = [p0, p1]
    node._path_cb(path)
    return node


def _set_velocity(node, v_n, v_e, fresh=True):
    node._latest_vel_ned = (v_n, v_e)
    node._latest_vel_time = node.get_clock().now() if fresh else None


# ---------------------------------------------------------------------------
# Freeze guarantee: precise OFF == the frozen brake-when-near path
# ---------------------------------------------------------------------------
def test_precise_off_uses_frozen_acceptance_radius():
    rclpy.init()
    try:
        node = _build_node(point_precise_stop_enabled=False)
        _set_velocity(node, 0.35, 0.0)
        node._corner_stop_satisfied = lambda: False
        # 0.18 m from the point: OUTSIDE the 0.10 acceptance radius. Frozen path
        # does not engage (feed-forward's larger trigger would have).
        assert node._point_hold_tick(-0.18, 0.0, 0.0, 0.01, 5.0) is False
        assert node._point_hold_active_key is None
    finally:
        rclpy.shutdown()


def test_precise_off_disabled_overlay_is_noop():
    rclpy.init()
    try:
        node = _build_node(point_hold_enabled=False,
                           point_precise_stop_enabled=True)
        # Even sitting on the point, the whole overlay is inert.
        assert node._point_hold_tick(0.0, 0.0, 0.0, 0.01, 5.0) is False
        assert node._point_hold_active_key is None
        assert node._point_hold_done_keys == set()
    finally:
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Feed-forward: engages earlier, decelerates toward the point, dwells on arrival
# ---------------------------------------------------------------------------
def test_feedforward_engages_beyond_acceptance_radius():
    rclpy.init()
    try:
        node = _build_node()
        _set_velocity(node, 0.35, 0.0)              # trigger ≈ 0.204 m
        node._corner_stop_satisfied = lambda: False
        # 0.18 m out: outside the 0.10 acceptance radius but inside the v²/2a
        # feed-forward trigger → the overlay engages and drives the profile.
        assert node._point_hold_tick(-0.18, 0.0, 0.0, 0.01, 5.0) is True
        assert node._point_hold_active_key is not None
        assert node._point_hold_start_ns is None    # approaching, not dwelling
        last = node._vel_pub.last
        assert last is not None
        # _publish_velocity emits NED: vector.x = v_n. Facing north, the point is
        # ahead → a positive north command that respects the approach-speed cap.
        assert last.vector.x > 0.0
        assert last.vector.x <= 0.35 + 1e-6
        assert abs(last.vector.y) < 1e-9            # no lateral command
    finally:
        rclpy.shutdown()


def test_feedforward_holds_off_dwell_until_at_point():
    rclpy.init()
    try:
        node = _build_node()
        _set_velocity(node, 0.05, 0.0)
        node._corner_stop_satisfied = lambda: True   # physically stopped...
        # ...but 0.06 m short of the point (> 0.02 tol) → NOT yet dwelling.
        assert node._point_hold_tick(-0.06, 0.0, 0.0, 0.01, 5.0) is True
        assert node._point_hold_start_ns is None
        # Now on the point AND stopped → dwell begins.
        assert node._point_hold_tick(0.0, 0.0, 0.0, 0.01, 5.0) is True
        assert node._point_hold_start_ns is not None
    finally:
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Servo: creep toward the point, and the timeout backstop
# ---------------------------------------------------------------------------
def test_servo_creeps_toward_point():
    rclpy.init()
    try:
        node = _build_node(precise_stop_mode="servo")
        _set_velocity(node, 0.0, 0.0)
        node._corner_stop_satisfied = lambda: True   # coarse stop done
        # 0.06 m short, stopped → creep forward (not dwell yet).
        assert node._point_hold_tick(-0.06, 0.0, 0.0, 0.01, 5.0) is True
        assert node._point_hold_start_ns is None
        assert node._point_servo_start_ns is not None
    finally:
        rclpy.shutdown()


def test_servo_timeout_accepts_best_position():
    rclpy.init()
    try:
        node = _build_node(precise_stop_mode="servo", precise_stop_max_s=0.0)
        _set_velocity(node, 0.0, 0.0)
        node._corner_stop_satisfied = lambda: True
        # Off-mark but the 0-second timeout fires immediately → accept + dwell,
        # never wedge.
        assert node._point_hold_tick(-0.06, 0.0, 0.0, 0.01, 5.0) is True
        assert node._point_hold_start_ns is not None   # dwelling despite residual
    finally:
        rclpy.shutdown()


def test_precise_release_marks_done_once():
    rclpy.init()
    try:
        node = _build_node(point_hold_s=0.0)          # release immediately
        _set_velocity(node, 0.0, 0.0)
        node._corner_stop_satisfied = lambda: True
        # On the point, stopped, hold 0 → dwell elapses at once → release.
        assert node._point_hold_tick(0.0, 0.0, 0.0, 0.01, 5.0) is False
        assert len(node._point_hold_done_keys) == 1
        assert node._point_servo_start_ns is None
        # Already done → not re-engaged.
        assert node._point_hold_tick(0.0, 0.0, 0.0, 0.01, 5.0) is False
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("ALL PRECISE-STOP NODE TESTS PASSED")
