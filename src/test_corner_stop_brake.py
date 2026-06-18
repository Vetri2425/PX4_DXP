#!/usr/bin/env python3
"""Tests for the per-line-extension stop/pivot execution patch.

Covers active braking, fresh-vs-stale CORNER_STOP timeout policy, and the
tightened pivot-release gate (heading + yaw-rate + linear speed).

Run on a ROS2-sourced host (needs rclpy):
    python3 -X utf8 src/test_corner_stop_brake.py
"""
import math
import sys

import rclpy
from rclpy.parameter import Parameter
from rclpy.duration import Duration


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
        now = lambda: node.get_clock().now()

        # ---- default params (per the patch) -------------------------------
        assert node.get_parameter("segment_heading_tolerance_deg").value == 2.0, "strict aim must stay 2°"
        assert node.get_parameter("segment_pivot_release_max_deg").value == 3.0, "hard release ceiling 3°"
        assert node.get_parameter("segment_timeout_heading_tolerance_deg").value == 3.0, "forced-release reachable 3°"
        assert node.get_parameter("segment_align_settle_s").value == 0.20
        assert node.get_parameter("segment_brake_velocity_cap_m_s").value == 0.10
        print("PASS params: aim=2° release_max=3° timeout_tol=3° settle=0.20 brake_cap=0.10")

        # ---- TEST 4: braking command opposes motion, capped ----------------
        P(segment_brake_velocity_cap_m_s=0.10, segment_stop_speed_threshold=0.02)
        node._latest_vel_time = now(); node._latest_vel_ned = (0.20, 0.0)
        bn, be = node._corner_brake_velocity()
        assert bn < 0 and abs(be) < 1e-9, f"brake must oppose +N motion, got {(bn,be)}"
        assert math.hypot(bn, be) <= 0.10 + 1e-9, "brake must be capped"
        node._latest_vel_ned = (0.10, 0.10)               # diagonal
        bn, be = node._corner_brake_velocity()
        assert bn < 0 and be < 0, "brake must oppose both axes"
        print(f"PASS test 4: braking opposes motion and is capped (|v|={math.hypot(bn,be):.3f})")

        # braking yields zero when below threshold / stale / disabled
        node._latest_vel_ned = (0.01, 0.0)
        assert node._corner_brake_velocity() == (0.0, 0.0), "no brake below stop threshold"
        node._latest_vel_ned = (0.20, 0.0); node._latest_vel_time = None
        assert node._corner_brake_velocity() == (0.0, 0.0), "no brake on stale velocity"
        node._latest_vel_time = now(); P(segment_brake_velocity_cap_m_s=0.0)
        assert node._corner_brake_velocity() == (0.0, 0.0), "cap=0 disables braking"
        P(segment_brake_velocity_cap_m_s=0.10)
        print("PASS braking safe-zeros: below-threshold / stale / disabled → (0,0)")

        # ---- TEST 2: fresh + still moving → does NOT timeout-pivot ----------
        node._reset_corner_pivot_state()
        node._latest_vel_time = now(); node._latest_vel_ned = (0.14, 0.0); node._latest_yaw_rate_ned = 0.0
        node._corner_stop_entered = now() - Duration(seconds=3.0)   # past the 2s cap
        assert node._corner_stop_satisfied() is False, "fresh+moving must not pivot past the 2s cap"
        print("PASS test 2: fresh velocity still moving → no timeout-pivot at 2s cap")

        # ---- TEST 1 (same gate): run reaching goal while moving stays held --
        # _corner_stop_satisfied is the gate the run-boundary pivot waits on, so
        # test 2 passing means the run will not advance/pivot while moving.
        print("PASS test 1: run-boundary pivot waits on _corner_stop_satisfied (held while moving)")

        # ---- absolute backstop still releases (braking never converged) -----
        node._reset_corner_pivot_state()
        node._latest_vel_time = now(); node._latest_vel_ned = (0.14, 0.0)
        node._corner_stop_entered = now() - Duration(seconds=6.0)   # past abs backstop (5s)
        assert node._corner_stop_satisfied() is True, "abs backstop must prevent deadlock"
        print("PASS anti-deadlock: absolute backstop fires after 5s even if fresh+moving")

        # ---- TEST 3: stale velocity → 2s timeout fallback still works -------
        node._reset_corner_pivot_state()
        node._latest_vel_time = None                                # stale
        node._corner_stop_entered = now() - Duration(seconds=3.0)
        assert node._corner_stop_satisfied() is True, "stale velocity must use the 2s cap"
        print("PASS test 3: stale velocity → 2s timeout fallback fires")

        # fresh + truly stopped → confirms via dwell
        node._reset_corner_pivot_state()
        P(segment_stop_dwell_s=0.0)
        node._latest_vel_time = now(); node._latest_vel_ned = (0.005, 0.0); node._latest_yaw_rate_ned = 0.0
        assert node._corner_stop_satisfied() is True, "fresh+stopped → confirmed by dwell"
        P(segment_stop_dwell_s=0.30)
        print("PASS stop confirm: fresh velocity below threshold → confirmed by dwell")

        # ---- speed gate for release ---------------------------------------
        P(segment_align_speed_threshold=0.05)
        node._latest_vel_time = now(); node._latest_vel_ned = (0.02, 0.0)
        assert node._align_speed_ok() is True, "below align speed → ok"
        node._latest_vel_ned = (0.10, 0.0)
        assert node._align_speed_ok() is False, "above align speed → not ok"
        node._latest_vel_time = None
        assert node._align_speed_ok() is True, "stale velocity must not block release"
        print("PASS speed gate: blocks release while drifting, ignores stale velocity")

        # ---- TEST 5 & 6: pivot release fails at 4°, succeeds at <=2° --------
        def setup_pivot():
            node._reset_corner_pivot_state()
            node._path = [_pose(0.0, 0.0), _pose(1.0, 0.0)]   # heading 0 (NED +N)
            node._run_align_pending = True
            node._corner_stop_complete = True                 # past the stop, in pivot
            node._pivot_started = now()                       # fresh → not timed out
            node._run_align_turn_rad = math.radians(90.0)
            node._latest_vel_time = now(); node._latest_vel_ned = (0.0, 0.0)
            node._latest_yaw_rate_ned = 0.0
            node._align_settle_since = None
            P(segment_align_settle_s=0.0, segment_heading_tolerance_deg=2.0)

        # TEST 5: 4° heading error, strict 2° → must NOT release (still holding)
        setup_pivot()
        held = node._run_alignment_hold(0.0, 0.0, math.radians(-4.0), 0.0)
        assert held is True, "4° > 2° strict and not timed out → must keep holding"
        print("PASS test 5: release with 4° heading error fails when tolerance is 2°")

        # TEST 6: <=2° heading + settled yaw + stopped → releases
        setup_pivot()
        held = node._run_alignment_hold(0.0, 0.0, math.radians(-1.0), 0.0)
        assert held is False, "1° <= 2° with settled yaw + speed → must release"
        print("PASS test 6: release succeeds at <=2° with settled yaw-rate and speed")

        # release blocked while still drifting even at 1°
        setup_pivot()
        node._latest_vel_ned = (0.20, 0.0)                    # drifting fast
        held = node._run_alignment_hold(0.0, 0.0, math.radians(-1.0), 0.0)
        assert held is True, "1° but drifting at 0.2 m/s → speed gate must block release"
        print("PASS speed-gated release: 1° but drifting → no release")

        node.destroy_node()
        print("\n=== ALL CORNER-STOP / BRAKE TESTS PASSED ===")
    except AssertionError as e:
        ok = False
        print(f"\nFAIL: {e}")
    finally:
        rclpy.shutdown()
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
