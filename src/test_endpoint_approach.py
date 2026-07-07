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


def _mavros_pose(n, e, yaw_ned=0.0):
    """MAVROS ENU pose: x=East, y=North; yaw_enu = pi/2 - yaw_ned."""
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    ps.pose.position.x = float(e)
    ps.pose.position.y = float(n)
    yaw_enu = math.pi / 2.0 - yaw_ned
    ps.pose.orientation.z = math.sin(yaw_enu / 2.0)
    ps.pose.orientation.w = math.cos(yaw_enu / 2.0)
    return ps


def _runtime_entry_two_run_path(entry_len, mark_len):
    """Collinear runtime-entry(OFF) → MARK(ON) path, both north (NED)."""
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    p = Path()
    p.header.frame_id = "local_ned"

    def wp(n, e, z):
        ps = PoseStamped()
        ps.pose.position.x = float(n)
        ps.pose.position.y = float(e)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        return ps

    a = wp(0.0, 0.0, 0.0)
    a.pose.orientation.x = 1.0  # runtime-entry marker
    a.pose.orientation.w = 0.0
    b = wp(entry_len, 0.0, 0.0)               # entry end == boundary
    c = wp(entry_len, 0.0, 1.0)               # MARK start (duplicate vertex)
    d = wp(entry_len + mark_len, 0.0, 1.0)    # MARK end
    p.poses = [a, b, c, d]
    return p


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

        # ---- smooth runtime-entry leg decelerates before the run boundary ----
        # 2026-07-07 14:26 bag: a 1.4 m runtime-entry (smooth) leg reached the
        # entry→MARK boundary at full mission speed (0.37 m/s), overshot ~26 cm,
        # and oscillated ~11 s. approach_velocity_scaling_dist=1.5 m gates the
        # final-goal approach behind path_travel_m >= 1.5 m, which a sub-1.5 m
        # entry leg can never satisfy. The run-boundary approach branch must
        # decelerate on remaining distance regardless of that gate.
        node = RPPControllerNode()
        node.set_parameters([
            Parameter("require_rtk_fix", value=False),
            Parameter("tracking_profile", value="smooth"),
            Parameter("mission_speed", value=0.35),
        ])
        node._gps_fix_type = 6
        cap = {}
        node._publish_velocity = lambda vn, ve: cap.update(sp=math.hypot(vn, ve))

        entry_len, mark_len = 1.4, 2.0
        node._pose_cb(_mavros_pose(0.0, 0.0, 0.0))
        node._path_cb(_runtime_entry_two_run_path(entry_len, mark_len))
        assert len(node._runs) == 2, f"expected entry+MARK runs, got {len(node._runs)}"
        assert node._runs[0].get("runtime_entry") is True
        assert node._runs[0]["profile"] != "segment", "entry leg must be smooth"

        def converged_speed_at(dist_before_boundary):
            # Rover on the entry leg, aligned north, `dist_before_boundary`
            # metres short of the boundary. Loop so the accel/decel speed loop
            # converges to the steady commanded speed at that point.
            n = entry_len - dist_before_boundary
            node._run_idx = 0
            node._last_speed_cmd = 0.35
            node._path_travel_m = max(0.0, n)
            sp = float("nan")
            for _ in range(150):
                cap.clear()
                node._pose_cb(_mavros_pose(n, 0.0, 0.0))
                node._control_loop()
                sp = cap.get("sp", sp)
            return sp

        # Far from boundary (well outside the boundary approach window): full speed.
        sp_far = converged_speed_at(1.2)
        # Near boundary (inside the 0.5*run_len window): decelerated.
        sp_near = converged_speed_at(0.15)
        assert sp_far == sp_far and sp_near == sp_near, "captured speeds must be finite"
        assert sp_near < sp_far, (
            f"entry leg must slow approaching the boundary "
            f"(far={sp_far:.3f} near={sp_near:.3f})"
        )
        assert sp_near < 0.20, (
            f"near-boundary entry speed must be well below mission speed, got {sp_near:.3f}"
        )
        print(
            f"PASS runtime-entry leg decelerates into boundary: "
            f"far={sp_far:.3f} m/s → near={sp_near:.3f} m/s"
        )

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
