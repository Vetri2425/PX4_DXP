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
    """8 m N line; the 3–5 m stretch is MARK (spray ON). Two OFF lead-in
    vertices (0,1) and two OFF tail vertices (7,8) anchor the ends so the
    collinear simplifier keeps the flag-boundary vertices — verified in-env,
    the conditioned path is 6 pts with seg_mark [F,F,T,F,F], a genuine interior
    mark region. (A bare 4-pt collinear mark collapses: the mark-start vertex
    is dropped and the region expands to s=0, leaving no pre-mark transit —
    hence this longer, conditioning-robust fixture.)"""
    from nav_msgs.msg import Path
    path = Path()
    path.header.frame_id = "local_ned"
    path.header.stamp = node.get_clock().now().to_msg()
    path.poses = [
        _path_pose(0.0, 0.0), _path_pose(1.0, 0.0),
        _path_pose(3.0, 0.0, mark=True), _path_pose(5.0, 0.0, mark=True),
        _path_pose(7.0, 0.0), _path_pose(8.0, 0.0),
    ]
    node._path_cb(path)


def _rtk_fix(node):
    from mavros_msgs.msg import GPSRAW
    g = GPSRAW()
    g.fix_type = 6
    node._gps_cb(g)


def _seg_for_north(node, north):
    """Segment index whose endpoints bracket `north` on the (N-axis) path."""
    xs = [p.pose.position.x for p in node._path]
    for i in range(len(xs) - 1):
        if xs[i] <= north <= xs[i + 1]:
            return i
    return max(0, len(xs) - 2)


def _tick_at(node, north):
    """Inject a pose and tick. Pose-only injection (no arm/OFFBOARD/velocity
    feedback) leaves the corner state machine's `_segment_idx` parked at 0, so
    we place the rover on the segment its pose actually sits on — the same
    `_segment_idx` the live controller would hold there. Advancement only ever
    increments, so pre-setting it forward is stable across the tick. This is
    the wiring proof: given real node state, does the gated publisher emit the
    right progress? The phase table itself is covered purely in
    test_progress_classifier.py."""
    node._pose_cb(_mavros_pose(north, 0.0, 0.0))
    node._segment_idx = _seg_for_north(node, north)
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

        # Tick at the start (pre-mark transit, mark region ahead at s=3).
        _tick_at(node, 0.3)
        assert caps["_progress_pub"].messages, "no /rpp/progress emitted"
        first = ProgressMsg.from_json(caps["_progress_pub"].messages[-1].data)
        assert isinstance(MissionPhase(first.phase), MissionPhase)
        assert first.next_boundary == "MARK_START"
        assert first.dist_to_next_boundary_m > 0.0

        # Drive up the line through the mark (s=3..5), collecting phases + events.
        phases = {first.phase}
        for n in (0.5, 1.5, 2.5, 3.2, 3.8, 4.5, 5.5, 6.5):
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
