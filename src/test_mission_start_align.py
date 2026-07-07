#!/usr/bin/env python3
"""Tests for the mission-start heading pre-align gate.

Covers the 2026-07-03 circle-run bug: _apply_run(0) has no prev_run to diff
heading against, so a large mis-heading at mission start (rover parked
facing the wrong way) was never caught — the rover drove the first leg
relying only on the BUG-T3 forward-cone clamp, producing a slow in-place
spin mid-transit plus an overshoot at the transit→body junction instead of
pivoting cleanly before moving.

The fix defers the decision to the control loop's first tick for run 0
(self._run0_align_decision_pending), since self._pose may not have arrived
yet when _apply_run(0) runs during /path conditioning.

Run on a ROS2-sourced host (needs rclpy):
    python3 -X utf8 src/test_mission_start_align.py
"""
import math
import sys

import rclpy
from rclpy.parameter import Parameter


class _CapturePub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)

    @property
    def last(self):
        return self.messages[-1] if self.messages else None


def _wire_captures(node):
    caps = {
        "vel": _CapturePub(),
        "yaw": _CapturePub(),
        "debug": _CapturePub(),
        "segment": _CapturePub(),
        "stop": _CapturePub(),
        "spray": _CapturePub(),
    }
    node._vel_pub = caps["vel"]
    node._yaw_rate_pub = caps["yaw"]
    node._dbg_pub = caps["debug"]
    node._segment_dbg_pub = caps["segment"]
    node._stop_dbg_pub = caps["stop"]
    node._spray_active_pub = caps["spray"]
    return caps


def _pose(n, e, yaw_ned=0.0):
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    # MAVROS ENU: x=East, y=North; yaw_enu = pi/2 - yaw_ned (CLAUDE.md convention)
    ps.pose.position.x = float(e)
    ps.pose.position.y = float(n)
    yaw_enu = math.pi / 2 - yaw_ned
    ps.pose.orientation.z = math.sin(yaw_enu / 2)
    ps.pose.orientation.w = math.cos(yaw_enu / 2)
    return ps


def _straight_path(n0, e0, n1, e1):
    # /path is RPP's internal NED path: pose.position.(x, y) = (north, east)
    # directly (see _path_cb: raw_pts = [(p.position.x, p.position.y) ...]),
    # unlike /mavros/local_position/pose which is MAVROS ENU.
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    p = Path()
    p.header.frame_id = "local_ned"
    a = PoseStamped(); a.pose.position.x = n0; a.pose.position.y = e0
    b = PoseStamped(); b.pose.position.x = n1; b.pose.position.y = e1
    p.poses = [a, b]
    return p


def _runtime_entry_path(n_entry, e_entry, n_mark, e_mark, n1, e1):
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    p = Path()
    p.header.frame_id = "local_ned"
    a = PoseStamped()
    a.pose.position.x = n_entry
    a.pose.position.y = e_entry
    a.pose.orientation.x = 1.0
    a.pose.orientation.w = 0.0
    b = PoseStamped()
    b.pose.position.x = n_mark
    b.pose.position.y = e_mark
    c = PoseStamped()
    c.pose.position.x = n1
    c.pose.position.y = e1
    c.pose.position.z = 1.0
    p.poses = [a, b, c]
    return p


def main():
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    ok = True
    try:
        from rpp_controller_node import RPPControllerNode
        node = RPPControllerNode()
        node.set_parameters([Parameter("require_rtk_fix", value=False)])
        node._gps_fix_type = 6

        # ---- reproduces the 2026-07-03 circle-run bug exactly ------------
        # Rover parked at (1.083, -0.554) facing 186° NED; first leg bears 24.29°.
        node._pose_cb(_pose(1.083, -0.554, math.radians(186.0)))
        node._path_cb(_straight_path(1.083, -0.554, 1.128, -0.534))

        assert node._run0_align_decision_pending is True, (
            "decision must be deferred at path-load time, not decided immediately "
            "(pose may not have arrived yet when _apply_run(0) runs)"
        )
        assert node._run_align_pending is False, (
            "must not decide before a pose-fresh control tick"
        )
        print("PASS: run-0 alignment decision deferred at path-load time")

        node._control_loop()

        assert node._run0_align_decision_pending is False, "must be one-shot"
        assert node._run_align_pending is True, (
            "162° mis-heading at mission start must trigger the pre-align pivot"
        )
        turn_deg = math.degrees(node._run_align_turn_rad)
        assert abs(turn_deg - 162.0) < 1.0, f"expected ~162° turn, got {turn_deg:.1f}°"
        print(f"PASS: 162° mis-heading at mission start triggers pre-align (turn={turn_deg:.1f}°)")

        node.destroy_node()

        # ---- small mis-heading (below corner threshold) must NOT pivot ---
        node = RPPControllerNode()
        node.set_parameters([Parameter("require_rtk_fix", value=False)])
        node._gps_fix_type = 6
        node._pose_cb(_pose(1.083, -0.554, math.radians(24.29)))  # already aligned
        node._path_cb(_straight_path(1.083, -0.554, 1.128, -0.534))
        node._control_loop()
        assert node._run0_align_decision_pending is False, "must be one-shot"
        assert node._run_align_pending is False, (
            "an already-aligned mission start must not pivot"
        )
        print("PASS: already-aligned mission start does not trigger a spurious pivot")

        node.destroy_node()

        # ---- runtime entry must NOT pre-align to the transit leg ------------
        # Reproduces 2026-07-07 Line_2m GPS_SURVEYED: rover parked on MARK
        # facing ~0° while the injected PRE entry leg bears ~63°. Aligning to
        # that transit heading at mission start wastes minutes; drive the entry
        # leg with forward-cone and pivot at RUNTIME_ENTRY_TO_MARK instead.
        node = RPPControllerNode()
        node.set_parameters([Parameter("require_rtk_fix", value=False)])
        node._gps_fix_type = 6
        node._pose_cb(_pose(0.0, 0.0, 0.0))
        node._path_cb(_runtime_entry_path(-2.0, -4.0, 0.0, 0.0, 2.0, 0.0))
        assert node._runs[0].get("runtime_entry") is True
        assert node._run0_align_decision_pending is True
        node._control_loop()
        assert node._run0_align_decision_pending is False
        assert node._run_align_pending is False, (
            "runtime entry must not trigger run-0 pre-align to the transit leg"
        )
        print("PASS: runtime entry skips run-0 pre-align (forward-cone on entry leg)")

        node.destroy_node()

        # ---- align release ignores yaw-rate once heading is in band ----------
        node = RPPControllerNode()
        node.set_parameters([Parameter("require_rtk_fix", value=False)])
        node._gps_fix_type = 6
        node._latest_vel_time = node.get_clock().now()
        node._latest_vel_ned = (0.0, 0.0)
        node._latest_yaw_rate_ned = 0.25
        yr_ok, sp_ok = node._align_release_motion_ok(
            heading_ok=True,
            vel_fresh=True,
            timed_out=False,
            yaw_rate_tol=0.05,
        )
        assert yr_ok is True, "heading in band must bypass yaw-rate settle gate"
        assert sp_ok is True
        yr_ok, sp_ok = node._align_release_motion_ok(
            heading_ok=False,
            vel_fresh=True,
            timed_out=False,
            yaw_rate_tol=0.05,
        )
        assert yr_ok is False, "still turning: yaw-rate gate must stay strict"
        assert sp_ok is True
        print("PASS: align release bypasses yaw-rate when heading already OK")

        node.destroy_node()

        # ---- runtime-entry MARK pivot must not creep down the MARK line ----
        from rpp_controller_node import StopReason

        node = RPPControllerNode()
        node.set_parameters([
            Parameter("require_rtk_fix", value=False),
            Parameter("segment_align_settle_s", value=0.0),
        ])
        node._gps_fix_type = 6
        caps = _wire_captures(node)
        node._path_cb(_runtime_entry_path(-2.0, -4.0, 0.0, 0.0, 2.0, 0.0))
        assert len(node._runs) == 2
        boundary = node._runs[0]["poses"][-1].pose.position
        boundary_seg_idx = max(0, len(node._runs[0]["poses"]) - 2)
        node._make_stop_certificate(
            StopReason.RUNTIME_ENTRY_TO_MARK,
            boundary.x,
            boundary.y,
            0.0,
            0.0,
            segment_idx=boundary_seg_idx,
        )
        assert node._advance_run(pre_stopped=True)
        assert node._run_idx == 1
        assert node._run_align_pending is True
        assert node._corner_stop_complete is True
        node._latest_vel_time = node.get_clock().now()
        node._latest_vel_ned = (0.0, 0.0)
        node._latest_yaw_rate_ned = 0.0

        yaw_ned = math.radians(-84.0)
        caps["vel"].messages.clear()
        held = node._run_alignment_hold(0.0, 0.0, yaw_ned, 0.0)
        assert held is True
        v = caps["vel"].last.vector
        speed = math.hypot(v.x, v.y)
        bearing = math.atan2(v.y, v.x)
        offset = abs(node._angle_wrap(bearing - yaw_ned))
        assert speed <= 0.051, f"runtime-entry pivot speed must stay small, got {speed:.3f}"
        assert offset <= math.radians(20.0) + 1e-6, (
            f"runtime-entry pivot must use tight forward cone, got {math.degrees(offset):.1f}°"
        )
        assert v.x < 0.04, (
            f"runtime-entry pivot must not command a large MARK-forward component, got {v.x:.3f}"
        )

        caps["vel"].messages.clear()
        held = node._run_alignment_hold(0.015, 0.0, yaw_ned, 0.0)
        assert held is True
        v = caps["vel"].last.vector
        assert v.x < 0.0, (
            "inside the normal 2 cm stop gate but outside the stricter "
            "runtime-entry pivot-start gate, command recovery back to MARK start"
        )
        print("PASS: runtime-entry MARK pivot uses anti-creep limits and strict start recovery")

        node.destroy_node()
        print("\n=== ALL MISSION-START ALIGN TESTS PASSED ===")
    except AssertionError as e:
        ok = False
        print(f"\nFAIL: {e}")
    finally:
        rclpy.shutdown()
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
