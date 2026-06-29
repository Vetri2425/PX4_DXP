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
from point_mission import PointMissionStatus
from routes.spray import spray_disable, spray_enable, spray_off, spray_on, spray_status, spray_test


class FakeNode:
    def __init__(self, state=None):
        self.state = state or {}
        self.manual_calls = []
        self._manual_on = False

    def get_state(self):
        return dict(self.state)

    def publish_spray_manual(self, on):
        self._manual_on = bool(on)
        self.manual_calls.append(bool(on))
        if not on:
            self.state["spraying"] = False

    def get_spray_runtime_status(self):
        if self.manual_calls:
            spraying = self._manual_on
        else:
            spraying = bool(self.state.get("spraying", False))
        off = not spraying
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
            "commanded_on": spraying,
            "confirmed_off": off,
            "off_acknowledged": off,
            "accepted_command_on": spraying,
            "pending_command": False,
            "physical_confirmation_available": False,
        }

    async def cancel_spray_dwell_async(self):
        return True, "ok"


class FakeController:
    def __init__(self, state=MissionState.IDLE):
        self.state = state


class FakePointMission:
    def __init__(self, status: PointMissionStatus):
        self.status = status


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
        assert resp["enabled"] is False
        assert resp["confirmed_off"] is True
        assert resp["spray_off_result"]["success"] is True
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
        assert resp == {
            "enabled": False,
            "confirmed_off": None,
            "spray_off_result": None,
        }
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
            await spray_test(
                SprayTestRequest(on=True, duration_s=3.0, diagnostic_authorized=True)
            )
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
        assert resp["manual"] is False
        assert resp["confirmed_off"] is True
        assert resp["spray_off_result"]["success"] is True
        assert node.manual_calls == [False]

    asyncio.run(run())


def test_spray_off_allowed_when_disabled(monkeypatch):
    """OFF is always safe regardless of enabled state."""
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    spray_module._spray_enabled = False

    async def run():
        resp = await spray_off()
        assert resp["spraying"] is False
        assert resp["hold"] is False
        assert resp["confirmed_off"] is True
        assert resp["spray_off_result"]["success"] is True
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
        assert resp["spraying"] is True
        assert resp["hold"] is True
        assert resp["accepted_on"] is True
        assert resp["commanded_on"] is True
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
        assert resp["spraying"] is False
        assert resp["hold"] is False
        assert resp["confirmed_off"] is True
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
        await spray_test(SprayTestRequest(on=True, diagnostic_authorized=True, duration_s=5.0))
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
        assert resp == {
            "spraying": False,
            "hold": False,
            "confirmed_off": None,
            "spray_off_result": None,
        }

    asyncio.run(run())


def test_spray_off_allowed_while_mission_running(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController(MissionState.RUNNING))

    async def run():
        resp = await spray_off()
        assert resp["spraying"] is False
        assert resp["hold"] is False
        assert resp["confirmed_off"] is True
        assert node.manual_calls == [False]

    asyncio.run(run())


def test_spray_on_supersedes_bench_test(monkeypatch):
    """spray_on() should cancel an active auto-off timer."""
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        await spray_test(SprayTestRequest(on=True, diagnostic_authorized=True, duration_s=5.0))
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
        resp = await spray_test(SprayTestRequest(on=True, diagnostic_authorized=True, duration_s=0.05))
        assert resp["manual"] is True
        assert resp["duration_s"] == 0.05
        assert resp["accepted_on"] is True
        assert resp["commanded_on"] is True
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
        await spray_test(SprayTestRequest(on=True, diagnostic_authorized=True, duration_s=5.0))
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
        await spray_test(SprayTestRequest(on=True, diagnostic_authorized=True, duration_s=5.0))
        task = spray_module._auto_off_task
        resp = await spray_test(SprayTestRequest(on=False))
        assert resp["manual"] is False
        assert resp["confirmed_off"] is True
        assert resp["spray_off_result"]["success"] is True
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
            await spray_test(SprayTestRequest(on=True, diagnostic_authorized=True))
        assert exc.value.status_code == 409
        assert node.manual_calls == []

    asyncio.run(run())


def test_spray_test_off_allowed_while_mission_running(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController(MissionState.RUNNING))

    async def run():
        resp = await spray_test(SprayTestRequest(on=False))
        assert resp["manual"] is False
        assert resp["confirmed_off"] is True
        assert node.manual_calls == [False]

    asyncio.run(run())


def test_spray_test_on_requires_armed(monkeypatch):
    node = FakeNode({"armed": False})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_test(SprayTestRequest(on=True, diagnostic_authorized=True))
        assert exc.value.status_code == 409
        assert node.manual_calls == []

    asyncio.run(run())


def test_spray_test_duration_clamped_and_validated(monkeypatch):
    node = FakeNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def run():
        resp = await spray_test(SprayTestRequest(on=True, diagnostic_authorized=True, duration_s=60.0))
        assert resp["duration_s"] == spray_module.MAX_SPRAY_TEST_DURATION_S

        with pytest.raises(HTTPException) as exc:
            await spray_test(SprayTestRequest(on=True, diagnostic_authorized=True, duration_s=-1.0))
        assert exc.value.status_code == 400

        resp = await spray_test(SprayTestRequest(on=True, diagnostic_authorized=True))
        assert resp["duration_s"] == spray_module.DEFAULT_SPRAY_TEST_DURATION_S

    asyncio.run(run())


def test_spray_test_503_without_ros(monkeypatch):
    monkeypatch.setattr(main, "ros_node", None)
    monkeypatch.setattr(main, "offboard_ctrl", None)

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_test(SprayTestRequest(on=True, diagnostic_authorized=True))
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
        assert resp["marking_state"] == "off"
        assert resp["spray_accepted_command_on"] is False
        assert resp["spray_recovery_required"] is False
        assert resp["spray_active_desired"] is False
        assert resp["manual_override"] is False
        assert resp["hold_active"] is False

    asyncio.run(run())


def test_spray_status_stale_cached_on_fails_closed(monkeypatch):
    class StaleCachedOnNode(FakeNode):
        def get_state(self):
            return {"spraying": True, "armed": True}

        def get_spray_runtime_status(self):
            return {
                "status_stale": True,
                "status_age_s": 2.0,
                "accepted_command_on": True,
                "desired_on": True,
                "pending_command": False,
            }

    monkeypatch.setattr(main, "ros_node", StaleCachedOnNode())
    monkeypatch.setattr(main, "offboard_ctrl", FakeController(MissionState.RUNNING))

    async def run():
        resp = await spray_status()
        assert resp["spraying"] is False
        assert resp["marking_state"] != "marking"
        assert resp["spray_recovery_required"] is True

    asyncio.run(run())


def test_spray_status_disabled_flow_operator_fields_stay_off(monkeypatch):
    class DisabledFlowNode(FakeNode):
        def get_state(self):
            return {"spraying": True, "armed": True}

        def get_spray_runtime_status(self):
            return {
                "status_stale": False,
                "status_age_s": 0.02,
                "accepted_command_on": True,
                "commanded_on": True,
                "desired_on": True,
                "pending_command": False,
                "flow_mode": "disabled",
                "dry_run_active": True,
                "geometry_spray_request": True,
            }

    monkeypatch.setattr(main, "ros_node", DisabledFlowNode())
    monkeypatch.setattr(main, "offboard_ctrl", FakeController(MissionState.RUNNING))

    async def run():
        resp = await spray_status()
        assert resp["spraying"] is False
        assert resp["commanded_on"] is False
        assert resp["spray_accepted_command_on"] is False
        assert resp["spray_desired_on"] is True
        assert resp["dry_run_active"] is True
        assert resp["geometry_spray_request"] is True
        assert resp["marking_state"] == "transit"
        assert resp["spray_state"] == "DRY_RUN"

    asyncio.run(run())


def test_spray_status_pending_on_is_not_spraying(monkeypatch):
    class PendingOnNode(FakeNode):
        def get_spray_runtime_status(self):
            return {
                "status_stale": False,
                "status_age_s": 0.02,
                "pending_command": True,
                "pending_command_on": True,
                "accepted_command_on": False,
            }

    monkeypatch.setattr(main, "ros_node", PendingOnNode({"armed": True}))
    monkeypatch.setattr(main, "offboard_ctrl", FakeController(MissionState.RUNNING))

    async def run():
        resp = await spray_status()
        assert resp["spraying"] is False
        assert resp["marking_state"] == "transit"
        assert resp["spray_pending_command"] is True

    asyncio.run(run())


def test_spray_status_fresh_accepted_on_reports_spraying(monkeypatch):
    class AcceptedOnNode(FakeNode):
        def get_state(self):
            return {"spraying": True, "armed": True}

        def get_spray_runtime_status(self):
            return {
                "status_stale": False,
                "status_age_s": 0.02,
                "accepted_command_on": True,
                "pending_command": False,
            }

    monkeypatch.setattr(main, "ros_node", AcceptedOnNode())
    monkeypatch.setattr(main, "offboard_ctrl", FakeController(MissionState.RUNNING))

    async def run():
        resp = await spray_status()
        assert resp["spraying"] is True
        assert resp["marking_state"] == "marking"
        assert resp["spray_accepted_command_on"] is True

    asyncio.run(run())


class UnconfirmedOffNode(FakeNode):
    def get_spray_runtime_status(self):
        return {
            "status_stale": False,
            "status_age_s": 0.02,
            "accepted_command_on": True,
            "off_acknowledged": False,
            "confirmed_off": False,
            "commanded_on": True,
            "pending_command": False,
            "physical_confirmation_available": False,
        }


def test_spray_off_degraded_when_live_off_unconfirmed(monkeypatch):
    from spray_safety import SprayOffResult

    node = UnconfirmedOffNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    async def _fail_off(*_args, **_kwargs):
        return SprayOffResult(
            success=False,
            attempted=True,
            timeout=True,
            fault=False,
            live=True,
            message="spray OFF confirmation timed out: spray accepted command is ON",
            command_off_acknowledged=False,
            physical_confirmation_available=False,
            physical_off_confirmed=False,
            recovery_required=True,
            failure_reason="spray accepted command is ON",
        )

    monkeypatch.setattr(spray_module, "force_spray_off_confirmed", _fail_off)

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_off()
        assert exc.value.status_code == 503
        detail = exc.value.detail
        assert detail["confirmed_off"] is False
        assert detail["recovery_required"] is True
        assert detail["spray_off_result"]["success"] is False

    asyncio.run(run())


def test_spray_disable_degraded_when_live_off_unconfirmed(monkeypatch):
    from spray_safety import SprayOffResult

    node = UnconfirmedOffNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", FakeController())
    spray_module._spray_enabled = True

    async def _fail_off(*_args, **_kwargs):
        return SprayOffResult(
            success=False,
            attempted=True,
            timeout=True,
            fault=False,
            live=True,
            message="spray OFF confirmation timed out",
            recovery_required=True,
            failure_reason="spray OFF not acknowledged",
        )

    monkeypatch.setattr(spray_module, "force_spray_off_confirmed", _fail_off)

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_disable()
        assert exc.value.status_code == 503
        assert exc.value.detail["enabled"] is False
        assert exc.value.detail["confirmed_off"] is False

    asyncio.run(run())


def test_spray_off_command_level_success_without_physical_feedback(monkeypatch):
    from spray_safety import SprayOffResult

    class CommandOffNode(FakeNode):
        def get_spray_runtime_status(self):
            return {
                "status_stale": False,
                "status_age_s": 0.02,
                "accepted_command_on": False,
                "off_acknowledged": True,
                "confirmed_off": True,
                "commanded_on": False,
                "pending_command": False,
                "physical_confirmation_available": False,
            }

    node = CommandOffNode({"armed": True})
    monkeypatch.setattr(main, "ros_node", node)

    async def _ok_off(*_args, **_kwargs):
        return SprayOffResult(
            success=True,
            attempted=True,
            timeout=False,
            fault=False,
            live=True,
            message="accepted OFF (command-level; physical feedback unavailable)",
            command_off_acknowledged=True,
            physical_confirmation_available=False,
            physical_off_confirmed=False,
            recovery_required=False,
            confirmation_level="command",
        )

    monkeypatch.setattr(spray_module, "force_spray_off_confirmed", _ok_off)

    async def run():
        resp = await spray_off()
        assert resp["confirmed_off"] is True
        assert resp["off_acknowledged"] is True
        assert resp["physical_off_confirmed"] is False
        assert resp["recovery_required"] is False

    asyncio.run(run())


def test_spray_status_separates_point_fields_without_overwriting_spray(monkeypatch):
    class RuntimeNode(FakeNode):
        def get_spray_runtime_status(self):
            status = super().get_spray_runtime_status()
            status.update(
                {
                    "ready": True,
                    "active_dwell": False,
                    "dwell_remaining_s": 0.25,
                    "last_transition": "spray-runtime-transition",
                    "last_error": "spray-runtime-error",
                }
            )
            return status

    point_status = PointMissionStatus(
        ready=False,
        active_dwell=True,
        dwell_remaining_s=9.5,
        last_transition="point-transition",
        last_error="point-error",
        hold_active=True,
    )
    monkeypatch.setattr(main, "ros_node", RuntimeNode())
    monkeypatch.setattr(main, "point_mission", FakePointMission(point_status))

    async def run():
        resp = await spray_status()
        assert resp["spray_ready"] is True
        assert resp["spray_active_dwell"] is False
        assert resp["spray_dwell_remaining_s"] == 0.25
        assert resp["spray_last_transition"] == "spray-runtime-transition"
        assert resp["spray_last_error"] == "spray-runtime-error"

        assert resp["point_ready"] is False
        assert resp["point_active_dwell"] is True
        assert resp["point_dwell_remaining_s"] == 9.5
        assert resp["point_last_transition"] == "point-transition"
        assert resp["point_last_error"] == "point-error"

        assert resp["ready"] is True
        assert resp["active_dwell"] is False
        assert resp["dwell_remaining_s"] == 0.25
        assert resp["last_transition"] == "spray-runtime-transition"
        assert resp["last_error"] == "spray-runtime-error"
        assert resp["hold_active"] is False
        assert resp["point_hold_active"] is True

    asyncio.run(run())
