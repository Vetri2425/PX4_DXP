#!/usr/bin/env python3
"""Tests for the unified hard-boundary TRUE-STOP (entry + segment RUN_BOUNDARY).

Every hard-boundary stop -- runtime-entry (OFF -> MARK) AND a plain segment
RUN_BOUNDARY (e.g. a square corner) alike -- runs the SAME true-stop
machinery: active _corner_brake_velocity inside segment_entry_true_stop_dist_m,
then a low-capped _corner_hold_velocity for the final cm -- and certifies via
_corner_stop_satisfied with the tighter PARKED speed gate
segment_entry_stop_speed_m_s (0.03) instead of the loose
segment_stop_speed_threshold (0.08). This replaces the old pure-zero stop
(_entry_pure_stop_hold), which commanded exactly (0,0), let PX4 coast, and
certified at 0.005 m/s inside the noise floor (deadlock). PR-A (2026-07-10)
widened this from entry-only to every hard boundary, since segment
RUN_BOUNDARY corners were bag-confirmed to take the exact same code path
minus the active brake and the tight cert (M1 square corner C1: 133.71cm arc).

Invariants under test:
  * inside the stop window while moving -> nonzero brake opposing motion, NOT (0,0)
  * certify ONLY when parked: measured speed <= 0.03 AND position <= 0.02, held
    for the dwell -- must NOT certify while still creeping at 0.05 m/s
  * position gate still blocks cert off-point
  * stale velocity -> 2 s fallback cap certifies (no deadlock)
  * the pure-stop branch/latch/params are gone
  * segment-profile RUN_BOUNDARY corners get the identical active brake +
    unified parked-speed cert (not the old segment-only-excluded uncapped
    tangent-frame fallback)

Run on a ROS2-sourced host (needs rclpy):
    python3 -X utf8 src/test_entry_true_stop.py
"""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter


def _pose(n, e):
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    ps.pose.position.x = float(n)
    ps.pose.position.y = float(e)
    ps.pose.orientation.w = 1.0
    return ps


class _CapturePub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)

    @property
    def last(self):
        return self.messages[-1] if self.messages else None

    def clear(self):
        self.messages.clear()


def _run(poses, *, runtime_entry=False, flags=None, profile="smooth"):
    return {
        "poses": poses,
        "flags": flags if flags is not None else [False] * len(poses),
        "profile": profile,
        "length": 1.0,
        "cum_s": [float(i) for i in range(len(poses))],
        "closed": False,
        "runtime_entry": runtime_entry,
    }


def main():
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    try:
        from rpp_controller_node import RPPControllerNode, StopReason

        node = RPPControllerNode()
        now = lambda: node.get_clock().now()

        vel_cap = _CapturePub()
        node._vel_pub = vel_cap
        node._yaw_rate_pub = _CapturePub()
        node._dbg_pub = _CapturePub()
        node._segment_dbg_pub = _CapturePub()
        node._stop_dbg_pub = _CapturePub()
        node._spray_active_pub = _CapturePub()

        # ---- params: single parked gate; pure-stop trio retired ------------
        assert node.get_parameter("segment_entry_stop_speed_m_s").value == 0.03
        for dead in (
            "segment_entry_pure_stop_dist_m",
            "segment_entry_pure_stop_speed_m_s",
            "segment_entry_pure_stop_dwell_s",
        ):
            assert not node.has_parameter(dead), f"retired param {dead} must be gone"
        assert not hasattr(node, "_entry_pure_stop_hold"), "pure-stop helper must be removed"
        assert not hasattr(node, "_entry_pure_stop_latched"), "pure-stop latch must be removed"
        print("PASS params: segment_entry_stop_speed_m_s=0.03; pure-stop removed")

        # Boundary at (1.0, 0.0); prev point (0.0, 0.0) -> tangent along +N.
        # run 0 = smooth entry (OFF), run 1 = MARK. Entry boundary always
        # requires alignment (_runtime_entry_to_mark_boundary).
        node._runs = [
            _run([_pose(0.0, 0.0), _pose(1.0, 0.0)], runtime_entry=True),
            _run([_pose(1.0, 0.0), _pose(2.0, 0.0)], flags=[True, True]),
        ]
        node._apply_run(0)
        node._active_tracking_profile = "smooth"   # run 0 entry is smooth

        # ---- TEST 1: inside the stop window, still moving fast -> ACTIVE
        #      brake opposing motion, NOT a zero command; no certify.
        node._run_boundary_stop_pending = False    # let first call arm cleanly
        node._latest_vel_time = now()
        node._latest_vel_ned = (0.20, 0.0)          # 0.20 m/s toward the point (+N)
        node._latest_yaw_rate_ned = 0.0
        vel_cap.clear()
        handled = node._hold_before_run_advance(0.93, 0.0, 0.0, 0.0, 0.07)  # pos_error≈0.07
        assert handled is True, "boundary stop must claim the control cycle"
        assert node._run_idx == 0, "must not certify while moving at 0.20 m/s"
        v = vel_cap.last
        assert v is not None and v.vector.x < -1e-6 and abs(v.vector.y) < 1e-9, (
            f"must ACTIVE-brake (nonzero, opposing +N motion), got "
            f"{(v.vector.x, v.vector.y) if v else None}"
        )
        print("PASS test 1: active brake opposing motion (not zero), no certify at 0.20 m/s")

        # ---- TEST 2: ON the point but still creeping at 0.05 m/s (above the
        #      0.03 parked gate, below the old 0.08). Two bugs the fix targets:
        #      (a) loose gate certified here and the pivot walked past;
        #      (b) _corner_brake_velocity used the 0.08 deadband so reverse
        #      brake was a pure no-op in the 3–8 cm/s band → zero cmd + PX4
        #      coast (bags M1/M2 entry HOLD: cmd 0 while actual 5–8 cm/s).
        node._latest_vel_ned = (0.05, 0.0)          # 5 cm/s
        vel_cap.clear()
        handled = node._hold_before_run_advance(1.0, 0.0, 0.0, 0.0, 0.0)
        assert handled is True
        assert node._run_idx == 0, "must NOT certify while creeping at 0.05 m/s (> 0.03 parked gate)"
        v = vel_cap.last
        assert v is not None and v.vector.x < -1e-6 and abs(v.vector.y) < 1e-9, (
            f"must ACTIVE-brake the 5 cm/s creep band (not zero), got "
            f"{(v.vector.x, v.vector.y) if v else None}"
        )
        print("PASS test 2: no certify + active reverse brake while creeping at 0.05 m/s")

        # ---- TEST 3: parked (speed <= 0.03, on point), dwell satisfied ->
        #      STOP_CERTIFIED + advance into the pivot.
        node._latest_vel_ned = (0.02, 0.0)          # 2 cm/s <= 0.03 parked gate
        node._hold_before_run_advance(1.0, 0.0, 0.0, 0.0, 0.0)   # arm dwell timer
        assert node._run_idx == 0, "one satisfying cycle is not enough (0.30 s dwell)"
        # backdate the settle window past the dwell
        node._corner_stop_settle_since = now() - Duration(seconds=0.4)
        node._corner_stop_entered = now() - Duration(seconds=0.4)
        handled = node._hold_before_run_advance(1.0, 0.0, 0.0, 0.0, 0.0)
        assert handled is True
        assert node._run_idx == 1, "must advance once parked for the full dwell"
        assert node._run_align_pending is True, "run-boundary pivot must be armed"
        assert node._run_boundary_stop_pending is False
        assert node._stop_certificate is not None
        assert node._stop_certificate.reason == StopReason.RUNTIME_ENTRY_TO_MARK
        print("PASS test 3: certifies + advances once parked (<=0.03 m/s) for the dwell")

        # ---- TEST 4: off the 2 cm position gate -> never certifies, even slow.
        node._runs = [
            _run([_pose(0.0, 0.0), _pose(1.0, 0.0)], runtime_entry=True),
            _run([_pose(1.0, 0.0), _pose(2.0, 0.0)], flags=[True, True]),
        ]
        node._apply_run(0)
        node._active_tracking_profile = "smooth"
        node._run_boundary_stop_pending = False
        node._latest_vel_time = now()
        node._latest_vel_ned = (0.01, 0.0)          # slow, but...
        node._corner_stop_settle_since = now() - Duration(seconds=1.0)
        node._corner_stop_entered = now() - Duration(seconds=1.0)
        handled = node._hold_before_run_advance(0.96, 0.0, 0.0, 0.0, 0.04)  # pos_error≈0.04 > 0.02
        assert handled is True
        assert node._run_idx == 0, "must not certify while 4 cm off the point"
        print("PASS test 4: position gate (2 cm) blocks cert off-point")

        # ---- TEST 5b: segment-profile RUN_BOUNDARY (square corner) now gets
        #      the SAME active brake as entry (PR-A, 2026-07-10). Position
        #      error 7cm — inside true_stop_dist (0.10m default) but OUTSIDE
        #      corner_position_tolerance_m (0.02m), so pre-PR-A this landed in
        #      the bare, uncapped tangent-frame _corner_hold_velocity()
        #      fallback (segment_brake_velocity_cap_m_s=0.18, arbitrary
        #      bearing) instead of the longitudinal-only brake. Bag-confirmed:
        #      M1's square corner C1 (HOLD start 1.9cm@12.5cm/s) rode that
        #      fallback into a 133.71cm arc before parking.
        node._runs = [
            _run([_pose(0.0, 0.0), _pose(1.0, 0.0)], runtime_entry=False, profile="segment"),
            _run([_pose(1.0, 0.0), _pose(1.0, 1.0)], profile="segment"),
        ]
        node._apply_run(0)
        node._active_tracking_profile = "segment"
        node._run_boundary_stop_pending = False
        node._latest_vel_time = now()
        node._latest_vel_ned = (0.125, 0.0)     # 12.5 cm/s toward the boundary (+N)
        node._latest_yaw_rate_ned = 0.0
        vel_cap.clear()
        handled = node._hold_before_run_advance(0.93, 0.0, 0.0, 0.0, 0.07)  # pos_error=0.07
        assert handled is True
        assert node._run_idx == 0, "must not certify a segment corner moving at 12.5 cm/s"
        v = vel_cap.last
        assert v is not None
        expected_n, expected_e = node._corner_brake_velocity(0.0)
        assert abs(v.vector.x - expected_n) < 1e-9 and abs(v.vector.y - expected_e) < 1e-9, (
            f"segment RUN_BOUNDARY inside true_stop_dist must use the active "
            f"longitudinal brake (_corner_brake_velocity), got "
            f"({v.vector.x:.4f},{v.vector.y:.4f}) vs expected "
            f"({expected_n:.4f},{expected_e:.4f})"
        )
        assert v.vector.x < -1e-6 and abs(v.vector.y) < 1e-9, (
            "brake must oppose +N motion (pure longitudinal, no lateral component)"
        )
        print("PASS test 5b: segment RUN_BOUNDARY gets active longitudinal brake "
              "(not the old uncapped tangent-frame servo)")

        # ---- TEST 5c: segment RUN_BOUNDARY creeping at 0.05 m/s (above the
        #      0.03 parked gate, below the old 0.08 segment default) must NOT
        #      certify AND must reverse-brake (not zero-cmd coast).
        node._latest_vel_ned = (0.05, 0.0)      # 5 cm/s
        vel_cap.clear()
        handled = node._hold_before_run_advance(1.0, 0.0, 0.0, 0.0, 0.0)
        assert handled is True
        assert node._run_idx == 0, (
            "must NOT certify a segment RUN_BOUNDARY creeping at 0.05 m/s "
            "(> 0.03 unified parked gate)"
        )
        v = vel_cap.last
        assert v is not None and v.vector.x < -1e-6, (
            f"segment RUN_BOUNDARY must reverse-brake at 5 cm/s, got "
            f"{(v.vector.x, v.vector.y) if v else None}"
        )
        print("PASS test 5c: segment RUN_BOUNDARY does not cert + reverse-brakes "
              "while creeping at 0.05 m/s")

        # ---- TEST 5d: segment RUN_BOUNDARY arriving already slow
        #      (PRE_CORNER_SLOWDOWN working correctly, <=0.03 m/s) certifies
        #      promptly once parked for the dwell — confirms PR-A adds no
        #      added latency when the slowdown profile already delivers a
        #      slow arrival.
        node._latest_vel_ned = (0.02, 0.0)      # 2 cm/s <= 0.03 parked gate
        node._hold_before_run_advance(1.0, 0.0, 0.0, 0.0, 0.0)   # arm dwell timer
        assert node._run_idx == 0, "one satisfying cycle is not enough (0.30 s dwell)"
        node._corner_stop_settle_since = now() - Duration(seconds=0.4)
        node._corner_stop_entered = now() - Duration(seconds=0.4)
        handled = node._hold_before_run_advance(1.0, 0.0, 0.0, 0.0, 0.0)
        assert handled is True
        assert node._run_idx == 1, "must advance once parked for the full dwell"
        assert node._run_align_pending is True, "run-boundary pivot must be armed"
        assert node._stop_certificate.reason == StopReason.RUN_BOUNDARY
        print("PASS test 5d: segment RUN_BOUNDARY certifies promptly once "
              "parked (<=0.03 m/s) for the dwell, no added latency")

        # ---- TEST 5e: M1-style regression bound. Forward-simulate (simple
        #      point-mass, perfect velocity tracking, 50 Hz control loop) from
        #      the exact bag-observed HOLD-start condition — M1 square corner
        #      C1, 1.9cm off the boundary at 12.5 cm/s — and assert the
        #      position error stays bounded instead of ballooning the way the
        #      pre-fix bag showed (peak 133.71cm). This is a coarse
        #      point-mass regression net around the fixed code path, not a
        #      firmware-accurate reproduction of the PX4 differential-drive
        #      arc dynamics that produced the original overshoot — the
        #      structural mechanism guard is test 5b above.
        node._runs = [
            _run([_pose(0.0, 0.0), _pose(1.0, 0.0)], runtime_entry=False, profile="segment"),
            _run([_pose(1.0, 0.0), _pose(1.0, 1.0)], profile="segment"),
        ]
        node._apply_run(0)
        node._active_tracking_profile = "segment"
        node._run_boundary_stop_pending = False

        dt = 1.0 / 50.0
        pos_n, pos_e = 1.0 - 0.019, 0.0    # 1.9 cm short of the boundary
        vel_n, vel_e = 0.125, 0.0          # 12.5 cm/s toward it
        node._latest_vel_time = now()
        node._latest_vel_ned = (vel_n, vel_e)
        node._latest_yaw_rate_ned = 0.0
        max_err = 0.0
        for _ in range(250):   # 5 s of simulated approach
            dist = math.hypot(pos_n - 1.0, pos_e)
            max_err = max(max_err, dist)
            vel_cap.clear()
            node._latest_vel_time = now()
            node._hold_before_run_advance(pos_n, pos_e, 0.0, 0.0, dist)
            if node._run_idx != 0:
                break
            v = vel_cap.last
            vn = v.vector.x if v else 0.0
            ve = v.vector.y if v else 0.0
            pos_n += vn * dt
            pos_e += ve * dt
            node._latest_vel_ned = (vn, ve)
        assert max_err < 0.15, (
            f"peak simulated position error {max_err * 100:.1f} cm exceeded "
            f"the 15 cm regression bound (pre-fix bag: 133.71 cm)"
        )
        print(f"PASS test 5e: M1-style forward sim bounded at "
              f"{max_err * 100:.1f} cm (< 15 cm)")

        # ---- TEST 5: stale velocity -> 2 s fallback cap certifies (no deadlock).
        node._runs = [
            _run([_pose(0.0, 0.0), _pose(1.0, 0.0)], runtime_entry=True),
            _run([_pose(1.0, 0.0), _pose(2.0, 0.0)], flags=[True, True]),
        ]
        node._apply_run(0)
        node._active_tracking_profile = "smooth"
        node._run_boundary_stop_pending = True      # skip the arming reset
        node._corner_stop_entered = now() - Duration(seconds=2.1)  # past the 2 s cap
        node._corner_stop_settle_since = None
        node._latest_vel_time = None                # stale -> cannot confirm speed
        handled = node._hold_before_run_advance(1.0, 0.0, 0.0, 0.0, 0.0)
        assert handled is True
        assert node._run_idx == 1, "stale-velocity fallback must certify at the 2 s cap"
        print("PASS test 5: stale-velocity fallback certifies at 2 s cap (no deadlock)")

        # ---- TEST 6: reverse ONLY inside true_stop_dist (0.10 m), not 0.50 m.
        #      At 30 cm: forward capture (no reverse). At 7 cm + 0.20 m/s: reverse.
        node._runs = [
            _run([_pose(0.0, 0.0), _pose(1.0, 0.0)], runtime_entry=True),
            _run([_pose(1.0, 0.0), _pose(2.0, 0.0)], flags=[True, True]),
        ]
        node._apply_run(0)
        node._active_tracking_profile = "smooth"
        node._run_boundary_stop_pending = False
        node._latest_vel_time = now()
        node._latest_vel_ned = (0.20, 0.0)
        node._latest_yaw_rate_ned = 0.0
        vel_cap.clear()
        handled = node._hold_before_run_advance(0.70, 0.0, 0.0, 0.0, 0.30)  # 30 cm
        assert handled is True and node._run_idx == 0
        v = vel_cap.last
        assert v is not None and v.vector.x > 1e-6, (
            f"at 30 cm must NOT reverse (capture toward point), got "
            f"{(v.vector.x, v.vector.y) if v else None}"
        )
        vel_cap.clear()
        handled = node._hold_before_run_advance(0.93, 0.0, 0.0, 0.0, 0.07)  # 7 cm
        assert handled is True and node._run_idx == 0
        v = vel_cap.last
        assert v is not None and v.vector.x < -1e-6, (
            f"at 7 cm / 0.20 m/s must reverse, got "
            f"{(v.vector.x, v.vector.y) if v else None}"
        )
        print("PASS test 6: reverse only inside 10 cm; capture at 30 cm")

        # ---- TEST 7: slow inside 10 cm but 4 cm off point → hold toward point
        #      (not reverse, not cert) so we can still reach the 2 cm gate.
        node._latest_vel_ned = (0.01, 0.0)  # below park 0.03
        node._corner_stop_settle_since = None
        node._corner_stop_entered = now()
        vel_cap.clear()
        handled = node._hold_before_run_advance(0.96, 0.0, 0.0, 0.0, 0.04)
        assert handled is True and node._run_idx == 0
        v = vel_cap.last
        assert v is not None and math.hypot(v.vector.x, v.vector.y) > 1e-6, (
            "4 cm off while slow must still command hold toward the point"
        )
        print("PASS test 7: slow + 4 cm off → hold toward 2 cm gate (no cert)")

        node.destroy_node()
    finally:
        rclpy.shutdown()

    print("\n=== ALL ENTRY TRUE-STOP TESTS PASSED ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
