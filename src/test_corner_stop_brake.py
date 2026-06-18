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
        assert node.get_parameter("segment_timeout_heading_tolerance_deg").value == 2.0, "precision timeout must stay 2°"
        assert node.get_parameter("segment_align_settle_s").value == 0.20
        assert node.get_parameter("segment_brake_velocity_cap_m_s").value == 0.08
        assert node.get_parameter("segment_align_speed_threshold").value == 0.02
        print("PASS params: aim=2° release_max=3° timeout_tol=2° settle=0.20 brake_cap=0.08")

        # ---- TEST 4: braking command opposes motion, capped ----------------
        P(segment_brake_velocity_cap_m_s=0.08, segment_stop_speed_threshold=0.02)
        node._latest_vel_time = now(); node._latest_vel_ned = (0.20, 0.0)
        bn, be = node._corner_brake_velocity(0.0)
        assert bn < 0 and abs(be) < 1e-9, f"brake must oppose +N motion, got {(bn,be)}"
        assert math.hypot(bn, be) <= 0.08 + 1e-9, "brake must be capped"
        node._latest_vel_ned = (0.0, 0.20)                # lateral-only motion
        assert node._corner_brake_velocity(0.0) == (0.0, 0.0), "lateral motion must not create reverse-flip command"
        node._latest_vel_ned = (0.0, 0.20)                # forward when yaw is East
        bn, be = node._corner_brake_velocity(math.pi / 2)
        assert abs(bn) < 1e-9 and be < 0, "brake must be body-longitudinal at nonzero yaw"
        print(f"PASS test 4: braking opposes motion and is capped (|v|={math.hypot(bn,be):.3f})")

        # braking yields zero when below threshold / stale / disabled
        node._latest_vel_ned = (0.01, 0.0)
        assert node._corner_brake_velocity(0.0) == (0.0, 0.0), "no brake below stop threshold"
        node._latest_vel_ned = (0.20, 0.0); node._latest_vel_time = None
        assert node._corner_brake_velocity(0.0) == (0.0, 0.0), "no brake on stale velocity"
        node._latest_vel_time = now(); P(segment_brake_velocity_cap_m_s=0.0)
        assert node._corner_brake_velocity(0.0) == (0.0, 0.0), "cap=0 disables braking"
        P(segment_brake_velocity_cap_m_s=0.08)
        print("PASS braking safe-zeros: below-threshold / stale / disabled → (0,0)")

        # ---- TEST 2: fresh + still moving → does NOT timeout-pivot ----------
        node._reset_corner_pivot_state()
        node._latest_vel_time = now(); node._latest_vel_ned = (0.14, 0.0); node._latest_yaw_rate_ned = 0.0
        node._corner_stop_entered = now() - Duration(seconds=3.0)   # past the 2s cap
        assert node._corner_stop_satisfied() is False, "fresh+moving must not pivot past the 2s cap"
        print("PASS test 2: fresh velocity still moving → no timeout-pivot at 2s cap")

        # ---- TEST 1: real run boundary does not advance while moving --------
        def run(poses):
            return {
                "poses": poses,
                "flags": [False] * len(poses),
                "profile": "segment",
                "length": 1.0,
                "cum_s": [float(i) for i in range(len(poses))],
                "closed": False,
            }

        node._runs = [
            run([_pose(0.0, 0.0), _pose(1.0, 0.0)]),
            run([_pose(1.0, 0.0), _pose(1.0, 1.0)]),
        ]
        node._apply_run(0)
        node._latest_vel_time = now(); node._latest_vel_ned = (0.14, 0.0)
        node._latest_yaw_rate_ned = 0.0
        node._hold_before_run_advance(1.0, 0.0, 0.0, 0.0, 0.0)
        assert node._run_idx == 0, "moving rover must remain on current run"
        assert node._run_boundary_stop_pending is True

        P(segment_stop_dwell_s=0.0)
        node._latest_vel_time = now(); node._latest_vel_ned = (0.0, 0.0)
        node._hold_before_run_advance(1.0, 0.0, 0.0, 0.0, 0.0)
        node._hold_before_run_advance(1.0, 0.0, 0.0, 0.0, 0.0)
        assert node._run_idx == 1, "confirmed stop must advance exactly once"
        assert node._run_align_pending is True
        assert node._corner_stop_complete is True, "pre-stop must carry into pivot without duplicate stop"
        P(segment_stop_dwell_s=0.30)
        print("PASS test 1: run boundary stops before advance and carries stop confirmation")

        # ---- fresh telemetry never timeout-pivots while still moving --------
        node._reset_corner_pivot_state()
        node._latest_vel_time = now(); node._latest_vel_ned = (0.14, 0.0)
        node._corner_stop_entered = now() - Duration(seconds=6.0)   # past abs backstop (5s)
        assert node._corner_stop_satisfied() is False, "fresh+moving must never timeout into pivot"
        print("PASS fresh safety: no timeout-pivot even after 6s while still moving")

        # ---- TEST 3: stale velocity → 2s timeout fallback still works -------
        node._reset_corner_pivot_state()
        node._latest_vel_time = None                                # stale
        node._corner_stop_entered = now() - Duration(seconds=0.4)
        assert node._corner_stop_satisfied() is False, "stale velocity must not pass via the 0.3s dwell"
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
        P(segment_align_speed_threshold=0.02)
        node._latest_vel_time = now(); node._latest_vel_ned = (0.019, 0.0)
        assert node._align_speed_ok() is True, "below align speed → ok"
        node._latest_vel_ned = (0.10, 0.0)
        assert node._align_speed_ok() is False, "above align speed → not ok"
        node._latest_vel_time = None
        assert node._align_speed_ok() is False, "stale velocity must block precision release"
        print("PASS speed gate: blocks release while drifting or stale")

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

        # Stale velocity must not release before the watchdog, but must have a
        # bounded fallback once the angle-aware pivot timeout has elapsed.
        setup_pivot()
        node._latest_vel_time = None
        held = node._run_alignment_hold(0.0, 0.0, math.radians(-1.0), 0.0)
        assert held is True, "stale velocity before timeout must keep alignment held"
        node._pivot_started = now() - Duration(seconds=6.0)
        held = node._run_alignment_hold(0.0, 0.0, math.radians(-1.0), 0.0)
        assert held is False, "stale velocity after timeout must use bounded heading fallback"
        print("PASS stale alignment: held before watchdog, bounded release after timeout")

        # A normal, non-extension square remains one segment run. Even if the
        # rover is already pointed at the next side, it must confirm the stop
        # before the intra-run corner can advance.
        square_run = run([
            _pose(0.0, 0.0),
            _pose(1.0, 0.0),
            _pose(1.0, 1.0),
        ])
        square_run["length"] = 2.0
        square_run["cum_s"] = [0.0, 1.0, 2.0]
        node._runs = [square_run]
        node._apply_run(0)
        node._path_travel_m = 1.0
        node._latest_vel_time = now(); node._latest_vel_ned = (0.0, 0.0)
        node._latest_yaw_rate_ned = 0.0
        P(segment_stop_dwell_s=0.30, segment_align_settle_s=0.0)
        node._control_segment_profile(1.0, 0.0, math.pi / 2, 0.0, 1.0)
        assert node._segment_idx == 0, "non-extension corner must not bypass stop dwell"
        assert node._corner_stop_complete is False

        P(segment_stop_dwell_s=0.0)
        node._latest_vel_time = now()
        node._control_segment_profile(1.0, 0.0, math.pi / 2, 0.0, 1.0)
        assert node._corner_stop_complete is True
        node._latest_vel_time = now()
        node._control_segment_profile(1.0, 0.0, math.pi / 2, 0.0, 1.0)
        assert node._segment_idx == 1, "confirmed stop must allow non-extension square to advance"
        print("PASS non-extension square: stop confirmation preserved, then corner advances")

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
