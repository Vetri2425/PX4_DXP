#!/usr/bin/env python3
"""Test: run-endpoint approach speed (lever D for per-line MARK-entry drift).

A per-line PRE/AFT run ends AT a corner (final_segment=True). The endpoint
approach floor must be the dedicated segment_endpoint_approach_speed (low, so the
rover arrives slow enough for active braking to stop on the corner), NOT the
smooth/arc min_approach_linear_velocity and NOT the pivot rollout speed
segment_min_corner_speed. The same low floor also applies to hard within-run
corners because those are stop-and-pivot transitions too.

Run on a ROS2-sourced host (needs rclpy):
    python3 -X utf8 src/test_endpoint_approach.py
"""
import math
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
        from rpp_controller_node import RPPControllerNode
        node = RPPControllerNode()
        P = lambda **kw: node.set_parameters([Parameter(k, value=v) for k, v in kw.items()])

        # ---- decoupling: the three approach floors are distinct params -------
        assert node.get_parameter("segment_endpoint_approach_speed").value == 0.03, "endpoint floor default 0.03"
        assert node.get_parameter("min_approach_linear_velocity").value == 0.1, "smooth/arc floor unchanged (0.10)"
        assert node.get_parameter("segment_min_corner_speed").value == 0.08, "pivot rollout speed unchanged (0.08)"
        print("PASS decoupling: endpoint/hard-corner=0.03, smooth/arc=0.10, pivot rollout=0.08 are separate")

        # ---- functional: the endpoint floor controls final-segment speed -----
        captured = {}
        node._publish_velocity = lambda vn, ve: captured.update(sp=math.hypot(vn, ve))

        def converged_endpoint_speed():
            # Single straight segment 0→2 m north; rover held 0.10 m from the
            # endpoint, heading aligned. final_segment = (seg_idx 0 >= n_pts-2).
            # Loop so the speed-loop ramp converges to the steady commanded speed
            # (one call is ramp-limited and does not reflect the floor).
            node._reset_corner_pivot_state()
            node._path = [_pose(0.0, 0.0), _pose(2.0, 0.0)]
            node._path_s = [0.0, 2.0]
            node._spray_flags = [True, True]
            node._segment_idx = 0
            node._path_travel_m = 1.9
            node._latest_yaw_rate_ned = 0.0
            node._last_speed_cmd = 0.35
            sp = float("nan")
            for _ in range(120):
                captured.clear()
                node._control_segment_profile(1.9, 0.0, 0.0, 0.0, 0.10)
                sp = captured.get("sp", float("nan"))
            return sp

        # Low endpoint floor → arrives slow at 0.10 m out.
        P(segment_endpoint_approach_speed=0.03)
        sp_low = converged_endpoint_speed()
        assert 0.0 < sp_low < 0.10, f"endpoint floor 0.03 must yield slow approach, got {sp_low:.3f}"

        # Raising the endpoint floor raises the approach speed at the same point,
        # proving the dedicated param controls the run-endpoint floor.
        P(segment_endpoint_approach_speed=0.20)
        sp_high = converged_endpoint_speed()
        assert sp_high > sp_low + 0.05, f"endpoint speed must track the param ({sp_low:.3f} vs {sp_high:.3f})"
        print(f"PASS endpoint floor controls final-segment approach: 0.03→{sp_low:.3f} m/s, 0.20→{sp_high:.3f} m/s")

        # Hard within-run corners use the same stop approach floor, not the
        # pivot rollout speed. This prevents arriving at CORNER_STOP still too
        # fast to hold the point.
        node._path = [_pose(0.0, 0.0), _pose(2.0, 0.0), _pose(2.0, 2.0)]
        node._path_s = [0.0, 2.0, 4.0]
        node._spray_flags = [True, True, True]
        node._segment_idx = 0
        node._path_travel_m = 1.9
        node._latest_yaw_rate_ned = 0.0
        node._last_speed_cmd = 0.35
        P(segment_endpoint_approach_speed=0.03, segment_min_corner_speed=0.20)
        for _ in range(120):
            captured.clear()
            node._control_segment_profile(1.9, 0.0, 0.0, 0.0, 0.10)
        sp_corner = captured.get("sp", float("nan"))
        assert 0.0 < sp_corner < 0.10, (
            "hard-corner approach must use endpoint floor, not pivot rollout "
            f"speed; got {sp_corner:.3f}"
        )
        print(f"PASS hard-corner approach uses stop floor: {sp_corner:.3f} m/s")

        node.destroy_node()
        print("\n=== ALL ENDPOINT-APPROACH TESTS PASSED ===")
    except AssertionError as e:
        ok = False
        print(f"\nFAIL: {e}")
    finally:
        rclpy.shutdown()
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
