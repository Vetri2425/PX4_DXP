#!/usr/bin/env python3
"""G1.6 — in-env progress publication test for RPPControllerNode (design §4).

Requires rclpy (runs on the Jetson, not the Mac). Ticks the real node with
`progress_publish_enabled` and asserts:
  * /rpp/progress emits a valid ProgressMsg every tick;
  * a path with a MARK region ahead reports next_boundary == MARK_START with a
    positive distance while in transit;
  * driving through the mark surfaces MARK_TRACKING and fires a MARK_START
    milestone;
  * with the flag OFF (default), NOTHING is published on /rpp/progress —
    the frozen observability-free behaviour.

Run (on the Jetson):  python3 -m pytest -q test_progress_publication.py
"""

import math

import rclpy
from rclpy.parameter import Parameter

from mission_progress import MilestoneMsg, MissionPhase, ProgressMsg


def _mavros_pose(north, east, yaw_ned=0.0):
    from geometry_msgs.msg import PoseStamped
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = east      # MAVROS ENU: x=East
    msg.pose.position.y = north     # y=North
    yaw_enu = math.pi / 2.0 - yaw_ned
    half = yaw_enu / 2.0
    msg.pose.orientation.w = math.cos(half)
    msg.pose.orientation.z = math.sin(half)
    return msg


def _path_pose(north, east, mark=False):
    from geometry_msgs.msg import PoseStamped
    msg = PoseStamped()
    msg.header.frame_id = "local_ned"
    msg.pose.position.x = north
    msg.pose.position.y = east
    msg.pose.position.z = 1.0 if mark else 0.0   # bit0 = spray ON
    msg.pose.orientation.w = 1.0
    return msg


class _CapturePub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def _wire_captures(node):
    """Replace every publisher the control + progress path touches."""
    caps = {}
    for attr in ("_vel_pub", "_yaw_rate_pub", "_dbg_pub", "_segment_dbg_pub",
                 "_conditioned_path_pub", "_spray_active_pub",
                 "_progress_pub", "_milestone_pub"):
        caps[attr] = _CapturePub()
        setattr(node, attr, caps[attr])
    return caps


def _mark_path(node):
    """6 m N line; the 2–4 m stretch is MARK (spray ON). Vertices 1 and 2 carry
    the spray flag, so segment (1→2) is the painted region and the flag
    transitions are clean anchors that survive conditioning."""
    from nav_msgs.msg import Path
    path = Path()
    path.header.frame_id = "local_ned"
    path.header.stamp = node.get_clock().now().to_msg()
    path.poses = [
        _path_pose(0.0, 0.0), _path_pose(2.0, 0.0, mark=True),
        _path_pose(4.0, 0.0, mark=True), _path_pose(6.0, 0.0),
    ]
    node._path_cb(path)


def _rtk_fix(node):
    from mavros_msgs.msg import GPSRAW
    g = GPSRAW()
    g.fix_type = 6
    node._gps_cb(g)


def _tick_at(node, north):
    node._pose_cb(_mavros_pose(north, 0.0, 0.0))
    node._control_loop()


def test_progress_on_emits_and_transitions():
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    try:
        from rpp_controller_node import RPPControllerNode
        node = RPPControllerNode()
        caps = _wire_captures(node)
        node.set_parameters([
            Parameter("progress_publish_enabled", Parameter.Type.BOOL, True),
            Parameter("require_rtk_fix", Parameter.Type.BOOL, False),
        ])
        _mark_path(node)
        _rtk_fix(node)

        # Tick at the start (transit, mark ahead).
        _tick_at(node, 0.3)
        assert caps["_progress_pub"].messages, "no /rpp/progress emitted"
        first = ProgressMsg.from_json(caps["_progress_pub"].messages[-1].data)
        assert isinstance(MissionPhase(first.phase), MissionPhase)
        assert first.next_boundary == "MARK_START"
        assert first.dist_to_next_boundary_m > 0.0

        # Drive up the line through the mark, collecting phases + milestones.
        phases = {first.phase}
        for n in (0.5, 1.0, 1.8, 2.2, 3.0, 3.8):
            _tick_at(node, n)
            phases.add(ProgressMsg.from_json(
                caps["_progress_pub"].messages[-1].data).phase)

        assert MissionPhase.MARK_TRACKING in phases, f"phases seen: {phases}"
        events = [MilestoneMsg.from_json(m.data).event
                  for m in caps["_milestone_pub"].messages]
        assert "MARK_START" in events, f"milestones: {events}"
        # seq is monotonic
        seqs = [MilestoneMsg.from_json(m.data).seq
                for m in caps["_milestone_pub"].messages]
        assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)
        print("PASS: progress emits + MARK_START milestone + monotonic seq")
    finally:
        rclpy.shutdown()


def test_progress_off_is_silent():
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    try:
        from rpp_controller_node import RPPControllerNode
        node = RPPControllerNode()   # progress_publish_enabled defaults False
        caps = _wire_captures(node)
        node.set_parameters([
            Parameter("require_rtk_fix", Parameter.Type.BOOL, False),
        ])
        _mark_path(node)
        _rtk_fix(node)
        for n in (0.3, 1.0, 2.5):
            _tick_at(node, n)
        assert not caps["_progress_pub"].messages, "progress leaked while OFF"
        assert not caps["_milestone_pub"].messages, "milestone leaked while OFF"
        # ...but the controller still drove (frozen path unaffected).
        assert caps["_vel_pub"].messages, "controller did not publish velocity"
        print("PASS: progress silent when disabled; control unaffected")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    test_progress_on_emits_and_transitions()
    test_progress_off_is_silent()
