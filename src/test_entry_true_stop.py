#!/usr/bin/env python3
"""Tests for the runtime-entry (OFF -> MARK) TRUE-STOP.

The entry->MARK boundary now runs the SAME segment true-stop machinery as a
smooth RUN_BOUNDARY -- active _corner_brake_velocity inside
segment_entry_true_stop_dist_m, then a low-capped _corner_hold_velocity for the
final cm -- and certifies via _corner_stop_satisfied with the tighter PARKED
speed gate segment_entry_stop_speed_m_s (0.03) instead of the loose
segment_stop_speed_threshold (0.08). This replaces the old pure-zero stop
(_entry_pure_stop_hold), which commanded exactly (0,0), let PX4 coast, and
certified at 0.005 m/s inside the noise floor (deadlock).

Invariants under test:
  * inside the stop window while moving -> nonzero brake opposing motion, NOT (0,0)
  * certify ONLY when parked: measured speed <= 0.03 AND position <= 0.02, held
    for the dwell -- must NOT certify while still creeping at 0.05 m/s
  * position gate still blocks cert off-point
  * stale velocity -> 2 s fallback cap certifies (no deadlock)
  * the pure-stop branch/latch/params are gone

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
        #      0.03 parked gate, below the old 0.08). This is the bug the fix
        #      targets: the loose gate certified here and the pivot walked past.
        node._latest_vel_ned = (0.05, 0.0)          # 5 cm/s
        vel_cap.clear()
        handled = node._hold_before_run_advance(1.0, 0.0, 0.0, 0.0, 0.0)
        assert handled is True
        assert node._run_idx == 0, "must NOT certify while creeping at 0.05 m/s (> 0.03 parked gate)"
        print("PASS test 2: no certify while creeping at 0.05 m/s (parked gate blocks it)")

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

        node.destroy_node()
    finally:
        rclpy.shutdown()

    print("\n=== ALL ENTRY TRUE-STOP TESTS PASSED ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
