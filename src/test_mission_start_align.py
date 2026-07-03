#!/usr/bin/env python3
"""Tests for the mission-start heading pre-align gate.

Covers the 2026-07-03 circle-run bug: _apply_run(0) has no prev_run to diff
heading against, so a large mis-heading at mission start (rover parked
facing the wrong way) was never caught — the rover drove the first leg
relying only on the BUG-T3 forward-cone clamp, producing a slow in-place
spin mid-transit plus an overshoot at the transit→body junction instead of
pivoting cleanly before moving.

The fix defers the decision to the control loop's first tick for run 0
(self._run0_align_decision_pending), since self._pose may not have arrived
yet when _apply_run(0) runs during /path conditioning.

Run on a ROS2-sourced host (needs rclpy):
    python3 -X utf8 src/test_mission_start_align.py
"""
import math
import sys

import rclpy
from rclpy.parameter import Parameter


def _pose(n, e, yaw_ned=0.0):
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    # MAVROS ENU: x=East, y=North; yaw_enu = pi/2 - yaw_ned (CLAUDE.md convention)
    ps.pose.position.x = float(e)
    ps.pose.position.y = float(n)
    yaw_enu = math.pi / 2 - yaw_ned
    ps.pose.orientation.z = math.sin(yaw_enu / 2)
    ps.pose.orientation.w = math.cos(yaw_enu / 2)
    return ps


def _straight_path(n0, e0, n1, e1):
    # /path is RPP's internal NED path: pose.position.(x, y) = (north, east)
    # directly (see _path_cb: raw_pts = [(p.position.x, p.position.y) ...]),
    # unlike /mavros/local_position/pose which is MAVROS ENU.
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    p = Path()
    p.header.frame_id = "local_ned"
    a = PoseStamped(); a.pose.position.x = n0; a.pose.position.y = e0
    b = PoseStamped(); b.pose.position.x = n1; b.pose.position.y = e1
    p.poses = [a, b]
    return p


def main():
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    ok = True
    try:
        from rpp_controller_node import RPPControllerNode
        node = RPPControllerNode()
        node.set_parameters([Parameter("require_rtk_fix", value=False)])
        node._gps_fix_type = 6

        # ---- reproduces the 2026-07-03 circle-run bug exactly ------------
        # Rover parked at (1.083, -0.554) facing 186° NED; first leg bears 24.29°.
        node._pose_cb(_pose(1.083, -0.554, math.radians(186.0)))
        node._path_cb(_straight_path(1.083, -0.554, 1.128, -0.534))

        assert node._run0_align_decision_pending is True, (
            "decision must be deferred at path-load time, not decided immediately "
            "(pose may not have arrived yet when _apply_run(0) runs)"
        )
        assert node._run_align_pending is False, (
            "must not decide before a pose-fresh control tick"
        )
        print("PASS: run-0 alignment decision deferred at path-load time")

        node._control_loop()

        assert node._run0_align_decision_pending is False, "must be one-shot"
        assert node._run_align_pending is True, (
            "162° mis-heading at mission start must trigger the pre-align pivot"
        )
        turn_deg = math.degrees(node._run_align_turn_rad)
        assert abs(turn_deg - 162.0) < 1.0, f"expected ~162° turn, got {turn_deg:.1f}°"
        print(f"PASS: 162° mis-heading at mission start triggers pre-align (turn={turn_deg:.1f}°)")

        node.destroy_node()

        # ---- small mis-heading (below corner threshold) must NOT pivot ---
        node = RPPControllerNode()
        node.set_parameters([Parameter("require_rtk_fix", value=False)])
        node._gps_fix_type = 6
        node._pose_cb(_pose(1.083, -0.554, math.radians(24.29)))  # already aligned
        node._path_cb(_straight_path(1.083, -0.554, 1.128, -0.534))
        node._control_loop()
        assert node._run0_align_decision_pending is False, "must be one-shot"
        assert node._run_align_pending is False, (
            "an already-aligned mission start must not pivot"
        )
        print("PASS: already-aligned mission start does not trigger a spurious pivot")

        node.destroy_node()
        print("\n=== ALL MISSION-START ALIGN TESTS PASSED ===")
    except AssertionError as e:
        ok = False
        print(f"\nFAIL: {e}")
    finally:
        rclpy.shutdown()
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
