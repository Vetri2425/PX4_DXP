#!/usr/bin/env python3
"""Tests for the pure zero-command runtime-entry (OFF -> MARK) stop.

Covers _entry_pure_stop_hold, wired into _hold_before_run_advance ahead of
the generic _corner_stop_satisfied gate and the true-stop/capture/corner-hold
velocity branches. Inside segment_entry_pure_stop_dist_m of the
RUNTIME_ENTRY_TO_MARK boundary point, the rover must command EXACTLY zero
velocity every cycle -- never _corner_brake_velocity, _corner_hold_velocity,
or _smooth_capture_velocity -- and may only certify STOP_CERTIFIED (and
advance into the pivot) once the measured speed has stayed at/under
segment_entry_pure_stop_speed_m_s for a continuous segment_entry_pure_stop_
dwell_s window.

Run on a ROS2-sourced host (needs rclpy):
    python3 -X utf8 src/test_entry_pure_stop.py
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
        P = lambda **kw: node.set_parameters([Parameter(k, value=v) for k, v in kw.items()])
        now = lambda: node.get_clock().now()

        vel_cap = _CapturePub()
        node._vel_pub = vel_cap
        node._yaw_rate_pub = _CapturePub()
        node._dbg_pub = _CapturePub()
        node._segment_dbg_pub = _CapturePub()
        node._stop_dbg_pub = _CapturePub()
        node._spray_active_pub = _CapturePub()

        # ---- default params -------------------------------------------------
        assert node.get_parameter("segment_entry_pure_stop_dist_m").value == 0.10
        assert node.get_parameter("segment_entry_pure_stop_speed_m_s").value == 0.005
        assert node.get_parameter("segment_entry_pure_stop_dwell_s").value == 1.0
        print("PASS params: pure_stop_dist=0.10 speed=0.005 dwell=1.0")

        # Boundary at (1.0, 0.0); prev point (0.0, 0.0) -> tangent along +N.
        node._runs = [
            _run([_pose(0.0, 0.0), _pose(1.0, 0.0)], runtime_entry=True),
            _run([_pose(1.0, 0.0), _pose(2.0, 0.0)], flags=[True, True]),
        ]
        node._apply_run(0)

        def hold_pivot_wraps():
            """Fail loudly if the pure-stop branch ever calls the old
            hold/brake/capture helpers (it must not)."""
            calls = {"brake": 0, "hold": 0, "capture": 0}
            orig = {
                "brake": node._corner_brake_velocity,
                "hold": node._corner_hold_velocity,
                "capture": node._smooth_capture_velocity,
            }

            def _wrap(name):
                def _fn(*a, **kw):
                    calls[name] += 1
                    return orig[name](*a, **kw)
                return _fn

            node._corner_brake_velocity = _wrap("brake")
            node._corner_hold_velocity = _wrap("hold")
            node._smooth_capture_velocity = _wrap("capture")
            return calls

        calls = hold_pivot_wraps()

        # ---- TEST 1: inside pure-stop zone, still moving -> zero cmd, no cert
        node._latest_vel_time = now()
        node._latest_vel_ned = (0.07, 0.0)   # 7 cm/s -- well above the 5 mm/s gate
        node._latest_yaw_rate_ned = 0.0
        vel_cap.clear()
        handled = node._hold_before_run_advance(1.0, 0.05, 0.0, 0.0, 0.05)
        assert handled is True, "pure-stop branch must claim the control cycle"
        assert node._run_idx == 0, "must not certify/advance while measured speed is 0.07 m/s"
        v = vel_cap.last
        assert v is not None and v.vector.x == 0.0 and v.vector.y == 0.0, (
            f"commanded velocity must be exactly zero inside the pure-stop zone, got "
            f"{(v.vector.x, v.vector.y) if v else None}"
        )
        assert node._entry_pure_stop_since is None, "dwell must not start while still moving"
        print("PASS test 1: zero command published, no certify at measured_speed=0.07 m/s")

        # ---- TEST L: coast-out past the pure-stop zone must NOT release the
        #      latch. _entry_pure_stop_hold must keep running (zero cmd, no
        #      old branches) even once pos_error grows back past
        #      segment_entry_pure_stop_dist_m.
        assert node._entry_pure_stop_latched is True, (
            "latch must be set once pos_error dipped inside the pure-stop zone (test 1)"
        )
        vel_cap.clear()
        handled = node._hold_before_run_advance(1.0, 0.50, 0.0, 0.0, 0.50)  # coasted 50cm out
        assert handled is True, "latched pure-stop must still claim the control cycle outside the zone"
        assert node._run_idx == 0, "coasting out must not certify/advance"
        v = vel_cap.last
        assert v is not None and v.vector.x == 0.0 and v.vector.y == 0.0, (
            f"latched pure-stop must keep commanding exactly zero even after coasting "
            f"outside the 10cm zone, got {(v.vector.x, v.vector.y) if v else None}"
        )
        assert node._entry_pure_stop_latched is True, "latch must remain set on a coast-out"
        print("PASS test L: coast-out past the pure-stop zone keeps the latch (and zero cmd) active")

        # ---- TEST 2: inside zone, off the tight position gate (pos_error=5cm,
        #      still < 10cm pure-stop zone) but slow -> still no certify (dwell
        #      requires position_ok too), command still exactly zero.
        node._latest_vel_ned = (0.003, 0.0)   # 3 mm/s -- below the 5 mm/s gate
        vel_cap.clear()
        handled = node._hold_before_run_advance(1.0, 0.05, 0.0, 0.0, 0.05)
        assert handled is True
        assert node._run_idx == 0, "must not certify while position_ok is False (5cm > 2cm tol)"
        v = vel_cap.last
        assert v.vector.x == 0.0 and v.vector.y == 0.0, "command must stay exactly zero"
        assert node._entry_pure_stop_since is None, "dwell must not start while off the position gate"
        print("PASS test 2: off position-gate + slow -> still zero cmd, no dwell, no certify")

        # ---- TEST 3: inside position gate AND slow -> dwell starts; must NOT
        #      certify before segment_entry_pure_stop_dwell_s elapses.
        vel_cap.clear()
        handled = node._hold_before_run_advance(1.0, 0.01, 0.0, 0.0, 0.01)
        assert handled is True
        assert node._run_idx == 0, "must not certify on the very first satisfying cycle (dwell=1.0s)"
        assert node._entry_pure_stop_since is not None, "dwell timer must start once position+speed are OK"
        v = vel_cap.last
        assert v.vector.x == 0.0 and v.vector.y == 0.0
        print("PASS test 3: dwell timer starts, no early certify before 1.0s elapses")

        # ---- TEST 4: any violation mid-dwell resets the timer (no partial credit)
        node._latest_vel_ned = (0.07, 0.0)   # blip back above the speed gate
        node._hold_before_run_advance(1.0, 0.01, 0.0, 0.0, 0.01)
        assert node._entry_pure_stop_since is None, "a speed violation must reset the dwell timer"
        node._latest_vel_ned = (0.003, 0.0)  # recover
        node._hold_before_run_advance(1.0, 0.01, 0.0, 0.0, 0.01)
        assert node._entry_pure_stop_since is not None, "dwell must be able to restart after recovering"
        print("PASS test 4: mid-dwell violation resets the timer, no partial credit")

        # ---- TEST 5: once measured speed <= 0.005 m/s holds for the full
        #      dwell, certify STOP_CERTIFIED and advance into the pivot.
        node._entry_pure_stop_since = now() - Duration(seconds=1.1)
        handled = node._hold_before_run_advance(1.0, 0.01, 0.0, 0.0, 0.01)
        assert handled is True
        assert node._run_idx == 1, "must advance to the next run once the dwell is satisfied"
        assert node._run_align_pending is True, "run-boundary pivot must be armed"
        assert node._run_boundary_stop_pending is False
        assert node._stop_certificate is not None
        assert node._stop_certificate.reason == StopReason.RUNTIME_ENTRY_TO_MARK
        print("PASS test 5: certifies STOP_CERTIFIED and advances after speed<=0.005 m/s for 1.0s")

        # ---- TEST 6: the old hold/brake/capture helpers were never called
        assert calls["brake"] == 0, f"_corner_brake_velocity must never run in the pure-stop zone, called {calls['brake']}x"
        assert calls["hold"] == 0, f"_corner_hold_velocity must never run in the pure-stop zone, called {calls['hold']}x"
        assert calls["capture"] == 0, f"_smooth_capture_velocity must never run in the pure-stop zone, called {calls['capture']}x"
        print("PASS test 6: _corner_brake_velocity / _corner_hold_velocity / _smooth_capture_velocity never called")

        # ---- TEST 7: STOP_CERTIFIED + _advance_run(pre_stopped=True) (test 5,
        #      above) must have cleared the latch.
        assert node._entry_pure_stop_latched is False, (
            "latch must be cleared once the pure-stop certifies and the run advances"
        )
        print("PASS test 7: latch cleared after STOP_CERTIFIED + _advance_run")

        # ---- TEST 8: _reset_corner_pivot_state() explicitly clears the latch.
        node._entry_pure_stop_latched = True
        node._reset_corner_pivot_state()
        assert node._entry_pure_stop_latched is False, "_reset_corner_pivot_state must clear the latch"
        print("PASS test 8: _reset_corner_pivot_state clears the latch")

        # ---- TEST 9: a new path (mission reset / replace) clears the latch
        #      via _path_cb -> _apply_run(0) -> _reset_corner_pivot_state().
        node._entry_pure_stop_latched = True
        from nav_msgs.msg import Path
        fresh = Path()
        fresh.poses = [_pose(0.0, 0.0), _pose(1.0, 0.0), _pose(2.0, 0.0)]
        node._path_cb(fresh)
        assert node._entry_pure_stop_latched is False, (
            "a new path must clear the latch (_path_cb -> _apply_run(0) -> _reset_corner_pivot_state)"
        )
        print("PASS test 9: new path (mission reset/replace) clears the latch")

        node.destroy_node()
    finally:
        rclpy.shutdown()

    print("\n=== ALL ENTRY PURE-STOP TESTS PASSED ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
