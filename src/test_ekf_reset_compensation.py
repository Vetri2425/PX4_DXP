#!/usr/bin/env python3
"""A3 — EKF reset compensation test for RPPControllerNode.

Reproduces the A3 bug and verifies the fix:

  Bug (compensation OFF): when the EKF teleports its position estimate (an RTK
  re-lock), the rover did NOT physically move, but the controller sees a big
  cross-track error and steers to close it — painting a kink. Baseline code
  skips exactly one cycle (JUMP_SKIP) and then chases the offset.

  Fix (compensation ON): the jump is absorbed into a running offset and
  subtracted from the tracking pose, so the path relationship stays continuous
  and net cross-track stays ~0 across the reset.

Both paths share the same P0.2 jump *detection*; only the response differs.
The default is OFF, so the frozen baseline is unchanged unless opted in.

Run:  python -X utf8 test_ekf_reset_compensation.py
      (or: pytest -q test_ekf_reset_compensation.py)
"""

import math
import rclpy

from test_smoke_rpp_controller import (
    _make_mavros_pose_from_ned,
    _make_path_pose,
    _CapturePub,
)


def _build_node(compensation: bool):
    """Instantiate a node on a 5 m North path, RTK-fixed, at the origin."""
    from rpp_controller_node import RPPControllerNode
    from nav_msgs.msg import Path
    from mavros_msgs.msg import GPSRAW

    node = RPPControllerNode()
    node.set_parameters(
        [rclpy.parameter.Parameter(
            "ekf_reset_compensation",
            rclpy.parameter.Parameter.Type.BOOL,
            compensation,
        )]
    )

    node._vel_pub = _CapturePub()
    node._yaw_rate_pub = _CapturePub()
    node._dbg_pub = _CapturePub()
    node._segment_dbg_pub = _CapturePub()
    node._conditioned_path_pub = _CapturePub()
    node._spray_active_pub = _CapturePub()

    path_msg = Path()
    path_msg.header.frame_id = "local_ned"
    path_msg.header.stamp = node.get_clock().now().to_msg()
    path_msg.poses = [_make_path_pose(0.0, 0.0), _make_path_pose(5.0, 0.0)]
    node._path_cb(path_msg)

    gps = GPSRAW()
    gps.fix_type = 6  # RTK_FIXED
    node._gps_cb(gps)
    return node


def _tick_at(node, north, east):
    """Inject a fresh MAVROS pose at (north, east) NED and tick once."""
    node._pose_cb(_make_mavros_pose_from_ned(north, east, 0.0))
    node._control_loop()


def _cross_track(node):
    """Latest published /rpp/debug cross-track (index [0])."""
    assert node._dbg_pub.last is not None, "no debug message published"
    return node._dbg_pub.last.data[0]


# 12 cm east teleport — well above the ~5 cm jump threshold at default speeds,
# well below the 30 cm absorb cap, so it is treated as a clean reset.
RESET_E = 0.12


def test_compensation_absorbs_reset():
    """ON: the reset is absorbed; cross-track stays ~0, no kink."""
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    try:
        node = _build_node(compensation=True)

        # On the line, moving north. Establishes _last_pos.
        _tick_at(node, 0.0, 0.0)
        xt_before = _cross_track(node)
        assert abs(xt_before) < 0.01, f"should start on-line, got {xt_before:.3f}"

        # EKF teleports 12 cm east — the rover did not move.
        _tick_at(node, 0.0, RESET_E)

        # The jump was absorbed into the offset, not steered against.
        assert node._ekf_reset_count == 1, node._ekf_reset_count
        assert abs(node._ekf_reset_offset[1] - RESET_E) < 1e-6, node._ekf_reset_offset
        assert abs(node._ekf_reset_offset[0]) < 1e-6, node._ekf_reset_offset

        # Cross-track stayed continuous (~0), NOT jumped to ~0.12.
        xt_after = _cross_track(node)
        assert abs(xt_after) < 0.01, (
            f"cross-track should stay continuous, got {xt_after:.3f} "
            f"(bug would show ~{RESET_E:.2f})"
        )

        # A following genuine 2 cm east drift is still tracked normally, on top
        # of the held offset (offset does not swallow real error).
        _tick_at(node, 0.0, RESET_E + 0.02)
        assert node._ekf_reset_count == 1, "no new reset should be counted"
        xt_drift = _cross_track(node)
        assert abs(xt_drift - 0.02) < 0.005, f"real drift lost: {xt_drift:.3f}"
        print("PASS: compensation ON absorbs the reset and keeps tracking")
    finally:
        rclpy.shutdown()


def test_baseline_off_is_unchanged():
    """OFF: no offset accrues; the reset cycle skips (frozen P0.2 behavior)."""
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    try:
        from rpp_controller_node import StateCode

        node = _build_node(compensation=False)

        _tick_at(node, 0.0, 0.0)
        _tick_at(node, 0.0, RESET_E)

        # Baseline never touches the offset frame.
        assert node._ekf_reset_offset == (0.0, 0.0), node._ekf_reset_offset
        assert node._ekf_reset_count == 0, node._ekf_reset_count

        # The reset cycle emitted JUMP_SKIP (state 5) and zero velocity.
        assert node._dbg_pub.last.data[7] == float(StateCode.JUMP_SKIP.value)
        print("PASS: compensation OFF preserves the frozen skip-one-cycle path")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    test_compensation_absorbs_reset()
    test_baseline_off_is_unchanged()
    print("ALL A3 TESTS PASSED")
