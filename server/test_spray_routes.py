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
from routes.spray import spray_disable, spray_enable, spray_off, spray_on, spray_status, spray_test


class FakeNode:
    def __init__(self, state=None):
        self.state = state or {}
        self.manual_calls = []

    def get_state(self):
        return dict(self.state)

    def publish_spray_manual(self, on):
        self.manual_calls.append(bool(on))

    def get_spray_runtime_status(self):
        return {
            "spray_mode": "continuous",
            "configuration_revision": 2,
            "model_revision": 3,
            "ready": True,
            "operator_enabled": True,
            "status_stale": False,
            "status_age_s": 0.01,
            "active_dwell": False,
            "dwell_remaining_s": 0.0,
            "commanded_on": bool(self.state.get("spraying", False)),
            "confirmed_off": not bool(self.state.get("spraying", False)),
        }


class FakeController:
    def __init__(self, state=MissionState.IDLE):
        self.state = state


@pytest.fixture(autouse=True)
def _reset_tasks():
    spray_module._cancel_all()
    spray_module._spray_enabled = True  # existing tests assume enabled
    yield
    spray_module._cancel_all()
    spray_module._spray_enabled = False  # restore safe default between runs


# ── spray_enable / spray_disable ─────────────────────────────────────────────

class FakeNodeWithParams(FakeNode):
    def __init__(self, state=None):
        super().__init__(state)
        self.param_sets = {}

    async def set_spray_param_async(self, name, value):
        self.param_sets[name] = value
        return True, ""


def test_spray_enable_sets_flag_and_node_param(monkeypatch):
    node = FakeNodeWithParams()
    monkeypatch.setattr(main, "ros_node", node)
    spray_module._spray_enabled = False

    async def run():
        resp = await spray_enable()
        assert resp == {"enabled": True}
        assert spray_module._spray_enabled is True
        assert node.param_sets.get("spray_enabled") is True

    asyncio.run(run())


def test_spray_disable_sets_flag_cancels_tasks_sends_off(monkeypatch):
    node = FakeNodeWithParams({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())
    spray_module._spray_enabled = True

    async def run():
        await spray_on()
        assert spray_module._keepalive_task is not None
        resp = await spray_disable()
        assert resp == {"enabled": False}
        assert spray_module._spray_enabled is False
        assert node.manual_calls[-1] is False
        assert spray_module._keepalive_task is None
        assert node.param_sets.get("spray_enabled") is False

    asyncio.run(run())


def test_spray_disable_safe_without_ros(monkeypatch):
    monkeypatch.setattr(main, "ros_node", None)
    spray_module._spray_enabled = True

    async def run():
        resp = await spray_disable()
        assert resp == {"enabled": False}
        assert spray_module._spray_enabled is False

    asyncio.run(run())


def test_spray_on_blocked_when_disabled(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())
    spray_module._spray_enabled = False

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_on()
        assert exc.value.status_code == 409
        assert node.manual_calls == []

    asyncio.run(run())


def test_spray_test_on_blocked_when_disabled(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())
    spray_module._spray_enabled = False

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_test(SprayTestRequest(on=True, duration_s=3.0))
        assert exc.value.status_code == 409
        assert node.manual_calls == []

    asyncio.run(run())


def test_spray_test_off_allowed_when_disabled(monkeypatch):
    """Test cancel (on=False) is always safe even when disabled."""
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())
    spray_module._spray_enabled = False

    async def run():
        resp = await spray_test(SprayTestRequest(on=False))
        assert resp == {"manual": False}
        assert node.manual_calls == [False]

    asyncio.run(run())


def test_spray_off_allowed_when_disabled(monkeypatch):
    """OFF is always safe regardless of enabled state."""
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    spray_module._spray_enabled = False

    async def run():
        resp = await spray_off()
        assert resp == {"spraying": False, "hold": False}
        assert node.manual_calls == [False]

    asyncio.run(run())


def test_spray_status_includes_enabled_field(monkeypatch):
    monkeypatch.setattr(main, "ros_node", None)

    async def run():
        spray_module._spray_enabled = False
        resp = await spray_status()
        assert resp["enabled"] is False
        spray_module._spray_enabled = True
        resp = await spray_status()
        assert resp["enabled"] is True

    asyncio.run(run())


# ── spray_on ─────────────────────────────────────────────────────────────────

def test_spray_on_publishes_true_and_starts_keepalive(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        resp = await spray_on()
        assert resp == {"spraying": True, "hold": True}
        assert node.manual_calls == [True]
        assert spray_module._keepalive_task is not None
        assert not spray_module._keepalive_task.done()

    asyncio.run(run())


def test_spray_on_keepalive_reasserts(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())
    original_interval = spray_module.KEEPALIVE_INTERVAL_S
    spray_module.KEEPALIVE_INTERVAL_S = 0.05

    async def run():
        await spray_on()
        await asyncio.sleep(0.18)
        # Initial ON + at least 2 keepalive re-asserts
        assert node.manual_calls.count(True) >= 3

    asyncio.run(run())
    spray_module.KEEPALIVE_INTERVAL_S = original_interval


def test_spray_on_blocked_while_mission_running(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController(MissionState.RUNNING))

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_on()
        assert exc.value.status_code == 409
        assert node.manual_calls == []

    asyncio.run(run())


def test_spray_on_blocked_when_disarmed(monkeypatch):
    node = FakeNode({"armed": False})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_on()
        assert exc.value.status_code == 409
        assert node.manual_calls == []

    asyncio.run(run())


def test_spray_on_503_without_ros(monkeypatch):
    monkeypatch.setattr(main, "ros_node", None)
    monkeypatch.setattr(main, "offboard_ctrl", None)

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_on()
        assert exc.value.status_code == 503

    asyncio.run(run())


# ── spray_off ────────────────────────────────────────────────────────────────

def test_spray_off_publishes_false_and_cancels_keepalive(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        await spray_on()
        keepalive = spray_module._keepalive_task
        resp = await spray_off()
        assert resp == {"spraying": False, "hold": False}
        assert node.manual_calls[-1] is False
        await asyncio.sleep(0)
        assert keepalive.cancelled() or keepalive.done()
        assert spray_module._keepalive_task is None

    asyncio.run(run())


def test_spray_off_cancels_bench_test_timer(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        await spray_test(SprayTestRequest(on=True, duration_s=5.0))
        task = spray_module._auto_off_task
        await spray_off()
        await asyncio.sleep(0)
        assert task.cancelled() or task.done()
        assert spray_module._auto_off_task is None
        assert node.manual_calls[-1] is False

    asyncio.run(run())


def test_spray_off_safe_without_ros(monkeypatch):
    monkeypatch.setattr(main, "ros_node", None)
    monkeypatch.setattr(main, "offboard_ctrl", None)

    async def run():
        resp = await spray_off()
        assert resp == {"spraying": False, "hold": False}

    asyncio.run(run())


def test_spray_off_allowed_while_mission_running(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController(MissionState.RUNNING))

    async def run():
        resp = await spray_off()
        assert resp == {"spraying": False, "hold": False}
        assert node.manual_calls == [False]

    asyncio.run(run())


def test_spray_on_supersedes_bench_test(monkeypatch):
    """spray_on() should cancel an active auto-off timer."""
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        await spray_test(SprayTestRequest(on=True, duration_s=5.0))
        old_task = spray_module._auto_off_task
        await spray_on()
        await asyncio.sleep(0)
        assert old_task.cancelled() or old_task.done()
        assert spray_module._auto_off_task is None
        assert spray_module._keepalive_task is not None

    asyncio.run(run())


# ── spray_test (existing, preserved) ─────────────────────────────────────────

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


def test_spray_test_cancels_keepalive(monkeypatch):
    """spray_test() should cancel an active hold keepalive."""
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        await spray_on()
        old_keepalive = spray_module._keepalive_task
        await spray_test(SprayTestRequest(on=True, duration_s=5.0))
        await asyncio.sleep(0)
        assert old_keepalive.cancelled() or old_keepalive.done()
        assert spray_module._keepalive_task is None
        assert spray_module._auto_off_task is not None

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


# ── spray_status ──────────────────────────────────────────────────────────────

def test_spray_status_reflects_node_state(monkeypatch):
    node = FakeNode(
        {"spraying": True, "spray_active": False, "spray_manual": True}
    )
    monkeypatch.setattr(main, "ros_node", node)

    async def run():
        resp = await spray_status()
        assert resp["spraying"] is True
        assert resp["spray_active_desired"] is False
        assert resp["manual_override"] is True
        assert resp["hold_active"] is False

    asyncio.run(run())


def test_spray_status_hold_active_flag(monkeypatch):
    node = FakeNode({"armed": True, "spraying": True, "spray_manual": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        await spray_on()
        resp = await spray_status()
        assert resp["hold_active"] is True
        await spray_off()
        resp = await spray_status()
        assert resp["hold_active"] is False

    asyncio.run(run())


def test_spray_status_safe_defaults_without_ros(monkeypatch):
    monkeypatch.setattr(main, "ros_node", None)

    async def run():
        resp = await spray_status()
        assert resp["spraying"] is False
        assert resp["spray_active_desired"] is False
        assert resp["manual_override"] is False
        assert resp["hold_active"] is False

    asyncio.run(run())
