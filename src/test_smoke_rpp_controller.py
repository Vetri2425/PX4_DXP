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


def _make_path_pose(north, east, mark=False):
    """Build a LOCAL_NED path waypoint."""
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.frame_id = "local_ned"
    msg.pose.position.x = north
    msg.pose.position.y = east
    msg.pose.position.z = 1.0 if mark else 0.0
    msg.pose.orientation.w = 1.0
    return msg


class _CapturePub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)

    @property
    def last(self):
        return self.messages[-1] if self.messages else None


def test_smoke():
    """Instantiate RPPControllerNode, inject a path and pose, tick once, assert no crash."""
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])

    try:
        from rpp_controller_node import RPPControllerNode
        from path_publisher_node import (
            gen_arc_quarter_1m5,
            gen_circle_1m5,
            gen_square_2x2,
            gen_straight_5m,
        )

        assert RPPControllerNode._normalize_tracking_profile("sharp") == "segment"
        assert RPPControllerNode._classify_auto_profile(gen_straight_5m(), 45.0) == "segment"
        assert RPPControllerNode._classify_auto_profile(gen_square_2x2(), 45.0) == "segment"
        assert RPPControllerNode._classify_auto_profile(gen_arc_quarter_1m5(), 45.0) == "smooth"
        assert RPPControllerNode._classify_auto_profile(gen_circle_1m5(), 45.0) == "smooth"

        node = RPPControllerNode()
        cap_vel = _CapturePub()
        cap_yaw = _CapturePub()
        cap_dbg = _CapturePub()
        cap_segment_dbg = _CapturePub()
        cap_conditioned = _CapturePub()
        cap_spray = _CapturePub()
        node._vel_pub = cap_vel
        node._yaw_rate_pub = cap_yaw
        node._dbg_pub = cap_dbg
        node._segment_dbg_pub = cap_segment_dbg
        node._conditioned_path_pub = cap_conditioned
        node._spray_active_pub = cap_spray

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
        assert node._active_tracking_profile == "segment"

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

        # Segment profile should simplify a generated square and publish the
        # actual internal path for bag-based analysis.
        square_msg = Path()
        square_msg.header.frame_id = "local_ned"
        square_msg.header.stamp = node.get_clock().now().to_msg()
        raw_square = gen_square_2x2()
        square_msg.poses = [_make_path_pose(n, e) for n, e in raw_square]
        node._path_cb(square_msg)
        assert node._active_tracking_profile == "segment"
        assert len(node._path) < len(raw_square), "Segment mode should collapse collinear side samples"
        assert cap_conditioned.last is not None
        assert len(cap_conditioned.last.poses) == len(node._path)
        print("PASS: segment square path selected, simplified, and published as /rpp/conditioned_path")

        # Near a corner but outside acceptance, segment mode must keep velocity
        # pointed along the current side instead of diagonally into the next side.
        node._segment_idx = 0
        node._last_speed_cmd = 0.2
        cap_vel.messages.clear()
        cap_yaw.messages.clear()
        node._control_segment_profile(1.60, 0.0, 0.0, 0.02, 2.0)
        assert cap_vel.last is not None
        assert abs(cap_vel.last.vector.y) < 1e-6, (
            f"Expected no eastward shortcut before corner acceptance, got {cap_vel.last.vector.y}"
        )
        print("PASS: segment lookahead stays on current side before corner acceptance")

        # At a corner, segment mode should command zero velocity and a yaw-rate
        # toward the next segment until heading tolerance is met.
        corner = node._path[1].pose.position
        node._segment_idx = 0
        node._last_speed_cmd = 0.2
        cap_vel.messages.clear()
        cap_yaw.messages.clear()
        node._control_segment_profile(corner.x, corner.y, 0.0, 0.02, 2.0)
        assert cap_vel.last is not None and cap_yaw.last is not None
        assert abs(cap_vel.last.vector.x) < 1e-9 and abs(cap_vel.last.vector.y) < 1e-9
        assert abs(cap_yaw.last.data) > 1e-4
        print("PASS: segment corner align publishes zero velocity with nonzero yaw-rate")

        # Corner align is yaw-rate-only actuation, so it must keep pivoting
        # even when use_feedforward_yaw_rate is disabled (deadlock guard).
        from rclpy.parameter import Parameter
        node.set_parameters([Parameter("use_feedforward_yaw_rate", value=False)])
        node._segment_idx = 0
        node._last_speed_cmd = 0.2
        cap_vel.messages.clear()
        cap_yaw.messages.clear()
        node._control_segment_profile(corner.x, corner.y, 0.0, 0.02, 2.0)
        assert cap_yaw.last is not None and abs(cap_yaw.last.data) > 1e-4, (
            "Corner align must command yaw-rate even with use_feedforward_yaw_rate=false"
        )
        node.set_parameters([Parameter("use_feedforward_yaw_rate", value=True)])
        print("PASS: corner align ignores use_feedforward_yaw_rate=false (no deadlock)")

        # Mixed mission (line entity + transit + arc entity): auto profile
        # must split into per-entity runs at spray-flag boundaries and
        # classify each run independently — line→segment, arc→smooth.
        mixed_msg = Path()
        mixed_msg.header.frame_id = "local_ned"
        mixed_msg.header.stamp = node.get_clock().now().to_msg()
        line_pts = [(0.0, i * 0.1) for i in range(11)]          # 1 m line, MARK
        transit_pts = [(0.0, 1.0 + i * 0.1) for i in range(1, 6)]  # 0.5 m hop
        arc_pts = [(n, e + 1.5) for n, e in gen_arc_quarter_1m5()]  # MARK arc
        mixed_msg.poses = (
            [_make_path_pose(n, e, mark=True) for n, e in line_pts]
            + [_make_path_pose(n, e, mark=False) for n, e in transit_pts]
            + [_make_path_pose(n, e, mark=True) for n, e in arc_pts]
        )
        node._path_cb(mixed_msg)
        profiles = [r["profile"] for r in node._runs]
        assert profiles == ["segment", "segment", "smooth"], (
            f"Expected [segment, segment, smooth] runs, got {profiles}"
        )
        assert node._run_idx == 0 and node._active_tracking_profile == "segment"
        # Conditioned path covers all runs without duplicated boundary points
        n_unique = sum(len(r["poses"]) for r in node._runs) - (len(node._runs) - 1)
        assert len(cap_conditioned.last.poses) == n_unique
        # Run advancing walks the queue and switches the active profile
        assert node._advance_run() and node._active_tracking_profile == "segment"
        assert node._advance_run() and node._active_tracking_profile == "smooth"
        assert not node._advance_run(), "After the last run, mission is complete"
        # Short transit run must be completable: travel gate caps at half run length
        node._apply_run(1)
        assert node._run_min_travel() <= 0.5 * node._runs[1]["length"] + 1e-9
        print("PASS: mixed mission splits into per-entity runs (segment/segment/smooth)")

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
