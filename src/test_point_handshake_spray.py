#!/usr/bin/env python3
"""G4 — spray-side point-handshake glue (in-env, needs rclpy + mavros_msgs).

Exercises the real SprayControllerNode's handshake glue: the AT_POINT milestone
intake (`_milestone_cb`), the arrival-gate resolver (`_point_handshake_gate`,
which is None unless consume_rpp_progress AND point mode), and the outbound
`/spray/point_done` publish. The PointMeter arrival-gate + completed_index
mechanics themselves are covered purely in `test_point_handshake.py`.

Run:  python3 -m pytest -q test_point_handshake_spray.py   (Jetson only)
"""

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from mission_progress import MilestoneEvent, MilestoneMsg, PointDoneMsg


class _CapturePub:
    def __init__(self):
        self.last = None
        self.count = 0

    def publish(self, msg):
        self.last = msg
        self.count += 1


def _build(consume):
    from spray_controller_node import SprayControllerNode

    node = SprayControllerNode()
    node.set_parameters([
        Parameter("consume_rpp_progress", Parameter.Type.BOOL, consume),
    ])
    node._point_done_pub = _CapturePub()
    return node


def test_milestone_cb_records_at_point_only():
    rclpy.init()
    try:
        node = _build(consume=True)
        m = String()
        m.data = MilestoneMsg(event=MilestoneEvent.AT_POINT, seq=3, index=2).to_json()
        node._milestone_cb(m)
        assert node._at_point_index == 2
        assert node._at_point_seq == 3
        # A non-AT_POINT milestone does not move the handshake target.
        other = String()
        other.data = MilestoneMsg(event=MilestoneEvent.MARK_START, seq=4, index=9).to_json()
        node._milestone_cb(other)
        assert node._at_point_index == 2
    finally:
        rclpy.shutdown()


def test_gate_none_unless_consume_and_point():
    rclpy.init()
    try:
        # consume off → always None (frozen self-arrival).
        node = _build(consume=False)
        node._session_mode = "point"
        node._at_point_index = 0
        assert node._point_handshake_gate(0) is None

        # consume on but not point mode → None.
        node2 = _build(consume=True)
        node2._session_mode = "continuous"
        assert node2._point_handshake_gate(0) is None
    finally:
        rclpy.shutdown()


def test_gate_matches_at_point_index():
    rclpy.init()
    try:
        node = _build(consume=True)
        node._session_mode = "point"
        node._at_point_index = 2
        assert node._point_handshake_gate(2) is True    # RPP confirmed this point
        assert node._point_handshake_gate(3) is False   # not this one yet
    finally:
        rclpy.shutdown()


def test_gate_resyncs_from_progress_phase_on_missed_milestone():
    """A dropped AT_POINT milestone is recovered from the fresh /rpp/progress."""
    rclpy.init()
    try:
        from mission_progress import MissionPhase, ProgressMsg
        node = _build(consume=True)
        node._session_mode = "point"
        node._at_point_index = -1                       # milestone never arrived
        node._rpp_progress = ProgressMsg(
            phase=MissionPhase.DWELL_HOLD, point_index=1
        )
        node._rpp_progress_recv_time = node.get_clock().now()
        assert node._point_handshake_gate(1) is True    # re-synced from phase
        assert node._point_handshake_gate(2) is False   # different point
    finally:
        rclpy.shutdown()


def test_publish_point_done_payload_and_monotonic_seq():
    rclpy.init()
    try:
        node = _build(consume=True)
        node._publish_point_done(4)
        node._publish_point_done(5)
        assert node._point_done_pub.count == 2
        pd = PointDoneMsg.from_json(node._point_done_pub.last.data)
        assert pd.point_index == 5
        assert pd.done is True
        assert pd.reason == "dwell_complete"
        assert pd.seq == 2                                # monotonic (1, then 2)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("ALL POINT-HANDSHAKE SPRAY TESTS PASSED")
