import asyncio
import os
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(__file__))

import offboard_controller as offboard_module
from config import RPP_IDLE, RPP_TRACKING
from models import MissionState
from offboard_controller import OffboardController


class FakeRppMonitor:
    def reset(self):
        pass


class FakeNode:
    def __init__(self, states):
        self._states = list(states)
        self.calls = []

    def get_state(self):
        if len(self._states) > 1:
            return self._states.pop(0)
        return self._states[0]

    def get_rpp_monitor(self):
        return FakeRppMonitor()

    def publish_path(self, points):
        self.calls.append(("publish_path", list(points)))

    async def arm_async(self, arm):
        self.calls.append(("arm", arm))
        return True, ""

    async def set_mode_async(self, mode):
        self.calls.append(("set_mode", mode))
        return True, ""


def run(coro):
    return asyncio.run(coro)


def test_start_publishes_path_before_arm_and_offboard():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        node = FakeNode([
            {"connected": True, "rpp_state": RPP_TRACKING},
            {"connected": True, "rpp_state": RPP_TRACKING},
        ])
        ctrl = OffboardController(node, deque())
        ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="test")

        ok, msg = run(ctrl.start_async())

        assert ok is True
        assert msg == "running"
        assert ctrl.state == MissionState.RUNNING
        assert node.calls == [
            ("publish_path", [(1.0, 2.0), (3.0, 4.0)]),
            ("arm", True),
            ("set_mode", "OFFBOARD"),
        ]
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_start_disarms_if_rpp_stays_idle_after_path_publish():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        node = FakeNode([
            {"connected": True, "rpp_state": RPP_TRACKING},
            {"connected": True, "rpp_state": RPP_IDLE},
        ])
        ctrl = OffboardController(node, deque())
        ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="test")

        ok, msg = run(ctrl.start_async())

        assert ok is False
        assert "RPP IDLE after path publish" in msg
        assert ctrl.state == MissionState.ERROR
        assert node.calls == [
            ("publish_path", [(1.0, 2.0), (3.0, 4.0)]),
            ("arm", True),
            ("arm", False),
        ]
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace
