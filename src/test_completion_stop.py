#!/usr/bin/env python3
"""D3 — final-waypoint completion latch (_hold_at_completion).

Proves the terminal stop brakes to a CONFIRMED physical stop before DONE,
reusing the same body-axis brake + measured-stop dwell the run-boundary stop
and the D0 bench use (no parallel completion-settle mechanism, no off-nose
recenter). Regression target: bag 2026-07-10_20-07 Line_2m ran 1.08 m past the
goal because a bare zero setpoint coasts and the goal check flipped back to
tracking.

Run on a ROS2-sourced host (needs rclpy), ideally with rpp-pipeline stopped or
on an isolated ROS_DOMAIN_ID so the test node does not collide with the live one:
    ROS_DOMAIN_ID=42 python3 -X utf8 src/test_completion_stop.py
"""
import sys

import rclpy
from rclpy.parameter import Parameter


def _pose(n, e):
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    ps.pose.position.x = float(n)
    ps.pose.position.y = float(e)
    ps.pose.orientation.w = 1.0
    return ps


def main():
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    ok = True
    try:
        from rpp_controller_node import RPPControllerNode, SegmentStateCode
        node = RPPControllerNode()
        P = lambda **kw: node.set_parameters([Parameter(k, value=v) for k, v in kw.items()])
        now = lambda: node.get_clock().now()

        # A minimal 2-point path so _publish_segment_debug's len(path)-2 is valid.
        node._path = [_pose(0.0, 0.0), _pose(2.0, 0.0)]
        P(segment_brake_velocity_cap_m_s=0.08, segment_stop_speed_threshold=0.02,
          segment_stop_yaw_rate_threshold=0.05)

        # ---- TEST 1: arriving with speed → LATCH + brake, NOT done ----------
        node._reset_corner_pivot_state()
        node._completion_stop_pending = False
        node._path_done = False
        node._latest_vel_time = now(); node._latest_vel_ned = (0.20, 0.0)
        node._latest_yaw_rate_ned = 0.0
        node._hold_at_completion(0.0, 0.0, 0.0, 0.01, 0.02)
        assert node._completion_stop_pending is True, "must latch on first entry"
        assert node._path_done is False, "must NOT complete while still moving"
        assert node._segment_state == SegmentStateCode.CORNER_STOP, "must hold in CORNER_STOP"
        print("PASS 1: arriving with speed latches + brakes, does not DONE")

        # ---- TEST 2: still coasting on a later cycle → still not done --------
        # The regression: a bare zero coasts past goal_tol and tracking resumes.
        # With the latch, repeated calls while moving keep braking, never DONE.
        node._latest_vel_time = now(); node._latest_vel_ned = (0.14, 0.0)
        node._hold_at_completion(0.05, 0.0, 0.0, 0.01, 0.07)   # drifted 5 cm past
        assert node._path_done is False, "coast must never flip to DONE while moving"
        assert node._completion_stop_pending is True, "latch must persist through the coast"
        print("PASS 2: coast while latched never completes (no drive-away)")

        # ---- TEST 3: confirmed physical stop → DONE -------------------------
        P(segment_stop_dwell_s=0.0)     # immediate confirm once speed+yaw are low
        node._reset_corner_pivot_state()
        node._completion_stop_pending = False
        node._path_done = False
        node._latest_vel_time = now(); node._latest_vel_ned = (0.005, 0.0)
        node._latest_yaw_rate_ned = 0.0
        node._hold_at_completion(0.0, 0.0, 0.0, 0.01, 0.01)
        assert node._path_done is True, "must complete once physically stopped"
        assert node._segment_state == SegmentStateCode.DONE, "state must be DONE on completion"
        print("PASS 3: confirmed stop (speed<thresh, dwell met) → DONE")

        # ---- TEST 4: brake is body-axis (I1) — reuses _corner_brake_velocity -
        # (sanity: the handler must not invent an off-nose vector; the brake it
        # calls is the same one the run-boundary stop / D0 used.)
        node._latest_vel_time = now(); node._latest_vel_ned = (0.20, 0.0)
        bn, be = node._corner_brake_velocity(0.0)
        assert bn < 0 and abs(be) < 1e-9, "completion brake must be body-longitudinal only (I1)"
        print("PASS 4: completion brake is body-axis only (invariant I1)")

        print("\nALL D3 COMPLETION-LATCH TESTS PASSED")
    except AssertionError as e:
        ok = False
        print(f"FAIL: {e}")
    except Exception as e:  # noqa: BLE001
        ok = False
        print(f"ERROR: {type(e).__name__}: {e}")
    finally:
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
