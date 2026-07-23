#!/usr/bin/env python3
"""G4/G5 — RPP point-handshake node glue (in-env, needs rclpy).

Drives the real RPPControllerNode's handshake release logic
(`_point_handshake_ready`) and the `_point_hold_tick` integration. The
load-bearing case is the freeze guarantee: with point_handshake_enabled False
the point-hold uses the frozen fixed point_hold_s timer byte-for-byte and
ignores /spray/point_done entirely. The ON tests cover auto release on
point_done, the point_hold_max_s backstop, and the manual WAIT_OPERATOR gate
(advance / wrong expect_index / timeout).

Deterministic timing: the backstop/timeout are probed by passing a dwell-start
(or setting _point_wait_start_ns) in the past, so no wall-clock waiting.

Run:  python3 -m pytest -q test_point_handshake_rpp.py   (Jetson only — rclpy)
"""

import rclpy
from rclpy.parameter import Parameter

from test_smoke_rpp_controller import _make_path_pose, _CapturePub

_SEC = 1_000_000_000


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
        "point_hold_s": 100.0,              # large → the fixed timer never fires in ON tests
        "point_hold_acceptance_m": 0.10,
        "point_precise_stop_enabled": False,
        "point_handshake_enabled": True,
        "point_execution_mode": "auto",
        "point_hold_max_s": 10.0,
        "manual_wait_timeout_s": 0.0,
    }
    defaults.update(params)
    node.set_parameters([_p(k, v) for k, v in defaults.items()])
    for attr in ("_vel_pub", "_yaw_rate_pub", "_dbg_pub", "_segment_dbg_pub",
                 "_conditioned_path_pub", "_spray_active_pub"):
        setattr(node, attr, _CapturePub())

    # 2-point path; point 0 at the origin flagged must-hit (z bit1 = 2) → rank 0.
    p0 = _make_path_pose(0.0, 0.0)
    p0.pose.position.z = 2.0
    p1 = _make_path_pose(5.0, 0.0)
    path = Path()
    path.header.frame_id = "local_ned"
    path.header.stamp = node.get_clock().now().to_msg()
    path.poses = [p0, p1]
    node._path_cb(path)
    return node


def _target_key(node):
    return node._pt_key((0.0, 0.0))


def _arm(node):
    """Snapshot the handshake as _point_hold_tick would on entering a hold."""
    node._point_done_seq_at_arm = node._point_done_seq
    node._point_spray_done_this_hold = False
    node._point_wait_start_ns = None


# ---------------------------------------------------------------------------
# Freeze: handshake OFF ignores point_done, uses the fixed timer
# ---------------------------------------------------------------------------
def test_handshake_off_ignores_point_done():
    rclpy.init()
    try:
        node = _build_node(point_handshake_enabled=False, point_hold_s=100.0)
        node._corner_stop_satisfied = lambda: True
        # First tick: stop confirmed → dwell begins (fixed timer, 100 s).
        assert node._point_hold_tick(0.0, 0.0, 0.0, 0.01, 5.0) is True
        assert node._point_hold_start_ns is not None
        # A point_done arrives — the frozen path must NOT early-release on it.
        node._point_done_index = 0
        node._point_done_seq = node._point_done_seq_at_arm + 1
        assert node._point_hold_tick(0.0, 0.0, 0.0, 0.01, 5.0) is True   # still holding
        assert node._point_hold_active_key is not None
    finally:
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Auto: release on /spray/point_done, or the point_hold_max_s backstop
# ---------------------------------------------------------------------------
def test_auto_holds_until_point_done():
    rclpy.init()
    try:
        node = _build_node(point_execution_mode="auto")
        tk = _target_key(node)
        _arm(node)
        now = node.get_clock().now().nanoseconds
        # No point_done yet, within the backstop window → keep holding.
        assert node._point_handshake_ready(tk, now) is False
        # point_done for rank 0 arrives (fresh seq) → release.
        node._point_done_index = 0
        node._point_done_seq = node._point_done_seq_at_arm + 1
        assert node._point_handshake_ready(tk, now) is True
    finally:
        rclpy.shutdown()


def test_auto_ignores_point_done_for_wrong_point():
    rclpy.init()
    try:
        node = _build_node(point_execution_mode="auto")
        tk = _target_key(node)
        _arm(node)
        now = node.get_clock().now().nanoseconds
        node._point_done_index = 7          # not rank 0
        node._point_done_seq = node._point_done_seq_at_arm + 1
        assert node._point_handshake_ready(tk, now) is False
    finally:
        rclpy.shutdown()


def test_auto_ignores_stale_point_done_before_arm():
    rclpy.init()
    try:
        node = _build_node(point_execution_mode="auto")
        tk = _target_key(node)
        # A point_done that arrived BEFORE this hold armed must not release it.
        node._point_done_index = 0
        node._point_done_seq = 5
        _arm(node)                          # seq_at_arm = 5
        now = node.get_clock().now().nanoseconds
        assert node._point_handshake_ready(tk, now) is False
    finally:
        rclpy.shutdown()


def test_auto_backstop_advances_without_point_done():
    rclpy.init()
    try:
        node = _build_node(point_execution_mode="auto", point_hold_max_s=10.0)
        tk = _target_key(node)
        _arm(node)
        now = node.get_clock().now().nanoseconds
        past = now - 20 * _SEC              # dwell began 20 s ago > 10 s cap
        assert node._point_handshake_ready(tk, past) is True
    finally:
        rclpy.shutdown()


def test_auto_end_to_end_release_via_tick():
    rclpy.init()
    try:
        node = _build_node(point_execution_mode="auto")
        node._corner_stop_satisfied = lambda: True
        # Tick 1: stop confirmed → dwell begins, no point_done → holds.
        assert node._point_hold_tick(0.0, 0.0, 0.0, 0.01, 5.0) is True
        assert node._point_hold_start_ns is not None
        # point_done arrives → tick releases (returns False, marks done once).
        node._point_done_index = 0
        node._point_done_seq = node._point_done_seq_at_arm + 1
        assert node._point_hold_tick(0.0, 0.0, 0.0, 0.01, 5.0) is False
        assert len(node._point_hold_done_keys) == 1
        assert node._point_hold_active_key is None
    finally:
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Manual: WAIT_OPERATOR after point_done, release on matching /point/advance
# ---------------------------------------------------------------------------
def test_manual_waits_for_operator_then_advances():
    rclpy.init()
    try:
        node = _build_node(point_execution_mode="manual")
        tk = _target_key(node)
        _arm(node)
        now = node.get_clock().now().nanoseconds
        # Spray proof arrives, but manual mode must NOT auto-release.
        node._point_done_index = 0
        node._point_done_seq = node._point_done_seq_at_arm + 1
        assert node._point_handshake_ready(tk, now) is False
        assert node._point_wait_start_ns is not None      # entered WAIT_OPERATOR
        # A wrong-index advance is ignored (double-tap guard).
        node._advance_index = 7
        node._advance_count += 1
        assert node._point_handshake_ready(tk, now) is False
        # The matching advance releases.
        node._advance_index = 0
        node._advance_count += 1
        assert node._point_handshake_ready(tk, now) is True
    finally:
        rclpy.shutdown()


def test_manual_backstop_still_waits_for_operator():
    """If spray never confirms, the phase-1 backstop must NOT skip the operator."""
    rclpy.init()
    try:
        node = _build_node(point_execution_mode="manual", point_hold_max_s=10.0)
        tk = _target_key(node)
        _arm(node)
        now = node.get_clock().now().nanoseconds
        past = now - 20 * _SEC
        # Backstop fires (no point_done) but manual still holds for the operator.
        assert node._point_handshake_ready(tk, past) is False
        assert node._point_wait_start_ns is not None
    finally:
        rclpy.shutdown()


def test_manual_wait_timeout_advances():
    rclpy.init()
    try:
        node = _build_node(point_execution_mode="manual", manual_wait_timeout_s=1.0)
        tk = _target_key(node)
        _arm(node)
        now = node.get_clock().now().nanoseconds
        node._point_done_index = 0
        node._point_done_seq = node._point_done_seq_at_arm + 1
        assert node._point_handshake_ready(tk, now) is False   # enters wait
        # Backdate the wait start past the 1 s timeout → advance despite no tap.
        node._point_wait_start_ns = now - 5 * _SEC
        assert node._point_handshake_ready(tk, now) is True
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("ALL POINT-HANDSHAKE RPP TESTS PASSED")
