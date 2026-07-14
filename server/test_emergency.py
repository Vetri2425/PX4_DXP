"""Emergency-stop regression tests.

The e-stop path had no test coverage at all before the 2026-07-14 audit, and it
contained a bug with the worst possible failure direction: on a fresh boot, e-stop
physically stopped the rover (stop-path, MANUAL, disarm all succeeded) and then
crashed while updating mission state, so the operator was told **the e-stop failed**.
An operator who believes the e-stop did not take will escalate — approach the rover,
or hit something else.

The cause: `OffboardController._lock` is created lazily on the first lifecycle call
(start/stop/abort/clear). `emergency.py` reached for the raw attribute, which is
`None` until then, so `async with None:` raised AttributeError.
"""
from __future__ import annotations

import asyncio
import os
import sys
from collections import deque

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from emergency import EmergencyHandler
from models import MissionState
from offboard_controller import OffboardController


class FakeNode:
    def __init__(self) -> None:
        self.calls: list = []

    def publish_stop_path(self):
        self.calls.append("publish_stop_path")
        return (0.0, 0.0)

    async def set_mode_async(self, mode):
        self.calls.append(("set_mode", mode))
        return True, ""

    async def arm_async(self, arm):
        self.calls.append(("arm", arm))
        return True, ""


def _handler() -> tuple[EmergencyHandler, FakeNode, OffboardController]:
    node = FakeNode()
    ctrl = OffboardController(node, None)
    return EmergencyHandler(node, ctrl, deque(maxlen=50)), node, ctrl


def test_estop_succeeds_as_the_very_first_lifecycle_call():
    """Fresh boot, no mission ever started, operator hits E-STOP.

    This is the regression. `_lock` has never been created, and the handler must not
    blow up reaching for it. Previously: AttributeError → HTTP 500 → operator told the
    e-stop failed, while the rover had in fact stopped.
    """
    handler, node, ctrl = _handler()
    assert ctrl._lock is None, "precondition: lock not yet created (fresh boot)"

    result = asyncio.run(handler.estop_async())

    assert result["success"] is True, f"e-stop reported failure: {result['message']}"
    assert ctrl.state == MissionState.ABORTED
    # And it actually reached the FCU, in the right order.
    assert node.calls == [
        "publish_stop_path",
        ("set_mode", "MANUAL"),
        ("arm", False),
    ]


def test_estop_reports_failure_only_when_the_fcu_actually_failed():
    """A real FCU failure must still be reported as a failure."""
    handler, node, _ = _handler()

    async def failing_set_mode(mode):
        node.calls.append(("set_mode", mode))
        return False, "rejected by FCU"

    node.set_mode_async = failing_set_mode

    result = asyncio.run(handler.estop_async())

    assert result["success"] is False
    assert "MANUAL" in result["message"]


def test_estop_still_stops_the_rover_if_state_update_fails():
    """Steps 1-3 are what stop the rover. A bookkeeping failure afterwards must never
    mask them — but it must still be surfaced rather than silently swallowed.
    """
    handler, node, ctrl = _handler()

    class Boom:
        def __call__(self):
            raise RuntimeError("lock exploded")

    ctrl._lifecycle_lock = Boom()

    result = asyncio.run(handler.estop_async())

    # The rover was still commanded to stop.
    assert node.calls == [
        "publish_stop_path",
        ("set_mode", "MANUAL"),
        ("arm", False),
    ]
    # ...and the bookkeeping failure is reported, not hidden.
    assert result["success"] is False
    assert "ABORTED" in result["message"] or "lock exploded" in result["message"]


def test_estop_without_ros_node_short_circuits_cleanly():
    handler = EmergencyHandler(None, None, deque(maxlen=50))
    result = asyncio.run(handler.estop_async())
    assert result["success"] is False
    assert "ROS node not available" in result["message"]
