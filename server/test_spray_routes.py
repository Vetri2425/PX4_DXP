"""Tests for the manual spray override endpoints (/api/spray/*)."""
import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest
from fastapi import HTTPException

import main
import routes.spray as spray_module
from models import MissionState, SprayTestRequest
from routes.spray import spray_status, spray_test


class FakeNode:
    def __init__(self, state=None):
        self.state = state or {}
        self.manual_calls = []

    def get_state(self):
        return dict(self.state)

    def publish_spray_manual(self, on):
        self.manual_calls.append(bool(on))


class FakeController:
    def __init__(self, state=MissionState.IDLE):
        self.state = state


@pytest.fixture(autouse=True)
def _reset_auto_off():
    spray_module._cancel_auto_off()
    yield
    spray_module._cancel_auto_off()


def test_spray_test_on_publishes_manual_and_schedules_auto_off(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        resp = await spray_test(SprayTestRequest(on=True, duration_s=0.05))
        assert resp == {"manual": True, "duration_s": 0.05}
        assert node.manual_calls == [True]
        assert spray_module._auto_off_task is not None
        await asyncio.sleep(0.15)
        assert node.manual_calls == [True, False]

    asyncio.run(run())


def test_spray_test_off_cancels_pending_auto_off(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        await spray_test(SprayTestRequest(on=True, duration_s=5.0))
        task = spray_module._auto_off_task
        resp = await spray_test(SprayTestRequest(on=False))
        assert resp == {"manual": False}
        assert node.manual_calls == [True, False]
        await asyncio.sleep(0)
        assert task.cancelled() or task.done()
        assert spray_module._auto_off_task is None

    asyncio.run(run())


def test_spray_test_on_blocked_while_mission_running(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController(MissionState.RUNNING))

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_test(SprayTestRequest(on=True))
        assert exc.value.status_code == 409
        assert node.manual_calls == []

    asyncio.run(run())


def test_spray_test_off_allowed_while_mission_running(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController(MissionState.RUNNING))

    async def run():
        resp = await spray_test(SprayTestRequest(on=False))
        assert resp == {"manual": False}
        assert node.manual_calls == [False]

    asyncio.run(run())


def test_spray_test_on_requires_armed(monkeypatch):
    node = FakeNode({"armed": False})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_test(SprayTestRequest(on=True))
        assert exc.value.status_code == 409
        assert node.manual_calls == []

    asyncio.run(run())


def test_spray_test_duration_clamped_and_validated(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        resp = await spray_test(SprayTestRequest(on=True, duration_s=60.0))
        assert resp["duration_s"] == spray_module.MAX_SPRAY_TEST_DURATION_S

        with pytest.raises(HTTPException) as exc:
            await spray_test(SprayTestRequest(on=True, duration_s=-1.0))
        assert exc.value.status_code == 400

        resp = await spray_test(SprayTestRequest(on=True))
        assert resp["duration_s"] == spray_module.DEFAULT_SPRAY_TEST_DURATION_S

    asyncio.run(run())


def test_spray_test_503_without_ros(monkeypatch):
    monkeypatch.setattr(main, "ros_node", None)
    monkeypatch.setattr(main, "offboard_ctrl", None)

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_test(SprayTestRequest(on=True))
        assert exc.value.status_code == 503

    asyncio.run(run())


def test_spray_status_reflects_node_state(monkeypatch):
    node = FakeNode(
        {"spraying": True, "spray_active": False, "spray_manual": True}
    )
    monkeypatch.setattr(main, "ros_node", node)

    async def run():
        resp = await spray_status()
        assert resp == {
            "spraying": True,
            "spray_active_desired": False,
            "manual_override": True,
        }

    asyncio.run(run())


def test_spray_status_safe_defaults_without_ros(monkeypatch):
    monkeypatch.setattr(main, "ros_node", None)

    async def run():
        resp = await spray_status()
        assert resp == {
            "spraying": False,
            "spray_active_desired": False,
            "manual_override": False,
        }

    asyncio.run(run())
