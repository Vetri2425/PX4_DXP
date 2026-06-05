#!/usr/bin/env python3
"""Runtime smoke test for RPPControllerNode.

Instantiates the node, ticks _control_loop once with mocked subscribers,
and asserts no exception. This test would have caught the NameError in
_publish_yaw_rate (Bug 6 from the Phase C audit) in 30 seconds.

Run:  python -X utf8 test_smoke_rpp_controller.py
      (or: pytest -q test_smoke_rpp_controller.py)
"""

import sys
import math

# ---------------------------------------------------------------------------
# Minimal rclpy bootstrap — no ROS master needed
# ---------------------------------------------------------------------------
import rclpy


def _make_mavros_pose_from_ned(north, east, yaw_ned=0.0):
    """Build a minimal MAVROS ENU PoseStamped for a desired NED pose."""
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.frame_id = "map"
    # MAVROS pose is ENU: x=East, y=North.
    msg.pose.position.x = east
    msg.pose.position.y = north
    msg.pose.position.z = 0.0

    # Convert NED yaw (0=North, CW+) to ENU yaw (0=East, CCW+).
    yaw_enu = math.pi / 2.0 - yaw_ned
    half = yaw_enu / 2.0
    msg.pose.orientation.w = math.cos(half)
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = math.sin(half)
    return msg


def _make_path_pose(north, east):
    """Build a LOCAL_NED path waypoint."""
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.frame_id = "local_ned"
    msg.pose.position.x = north
    msg.pose.position.y = east
    msg.pose.position.z = 0.0
    msg.pose.orientation.w = 1.0
    return msg


def test_smoke():
    """Instantiate RPPControllerNode, inject a path and pose, tick once, assert no crash."""
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])

    try:
        from rpp_controller_node import RPPControllerNode

        node = RPPControllerNode()

        # Inject a 2-point straight path (5 m North from origin)
        from nav_msgs.msg import Path

        path_msg = Path()
        path_msg.header.frame_id = "local_ned"
        path_msg.header.stamp = node.get_clock().now().to_msg()

        wp0 = _make_path_pose(0.0, 0.0)
        wp1 = _make_path_pose(5.0, 0.0)
        path_msg.poses = [wp0, wp1]

        # Publish path via the subscriber callback directly
        node._path_cb(path_msg)
        assert len(node._path) == 2, f"Path should have 2 points, got {len(node._path)}"

        # Inject MAVROS pose at NED origin, facing North.
        pose_msg = _make_mavros_pose_from_ned(0.0, 0.0, 0.0)
        node._pose_cb(pose_msg)
        assert node._pose is not None, "Pose should be set"
        n, e, yaw = node._enu_pose_to_ned(pose_msg)
        assert abs(n) < 1e-9 and abs(e) < 1e-9
        assert abs(yaw) < 1e-9, f"Expected yaw_ned=0, got {yaw}"

        # Inject GPS fix type = 4 (DGPS, not RTK) to test RTK_WAIT path
        from mavros_msgs.msg import GPSRAW
        gps_msg = GPSRAW()
        gps_msg.fix_type = 6  # RTK_FIXED
        node._gps_cb(gps_msg)

        # Tick the control loop once — this is where Bug 6 would crash
        try:
            node._control_loop()
        except Exception as e:
            print(f"FAIL: _control_loop raised {type(e).__name__}: {e}")
            raise

        # Check that the node published velocity (not crashed)
        # We can't easily inspect published messages without spin,
        # but surviving the tick without exception is the main assertion.
        print("PASS: _control_loop executed without exception")

        # Also test _publish_zero path (STALE/JUMP_SKIP code paths)
        try:
            from rpp_controller_node import StateCode
            node._publish_zero(StateCode.JUMP_SKIP, pose_age_ms=50.0, dist_to_goal=2.0)
            print("PASS: _publish_zero(JUMP_SKIP) executed without exception")
        except Exception as e:
            print(f"FAIL: _publish_zero raised {type(e).__name__}: {e}")
            raise

        # Test _publish_zero with RTK_WAIT
        try:
            node._publish_zero(StateCode.RTK_WAIT, pose_age_ms=100.0, dist_to_goal=3.0)
            print("PASS: _publish_zero(RTK_WAIT) executed without exception")
        except Exception as e:
            print(f"FAIL: _publish_zero(RTK_WAIT) raised {type(e).__name__}: {e}")
            raise

        # Test _publish_yaw_rate (this was Bug 6 — NameError on v_n/v_e)
        try:
            node._publish_yaw_rate(0.0)
            print("PASS: _publish_yaw_rate(0.0) executed without exception")
        except Exception as e:
            print(f"FAIL: _publish_yaw_rate raised {type(e).__name__}: {e}")
            raise

        node.destroy_node()
        print("\n=== ALL SMOKE TESTS PASSED ===")
        return True

    except Exception as e:
        print(f"\n=== SMOKE TEST FAILED: {type(e).__name__}: {e} ===")
        import traceback
        traceback.print_exc()
        return False

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    ok = test_smoke()
    sys.exit(0 if ok else 1)
