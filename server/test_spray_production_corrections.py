"""Production correction tests for startup, sockets, routes, and telemetry."""

from __future__ import annotations

import asyncio
import os
import sys

import pytest
from fastapi import HTTPException

sys.path.insert(0, os.path.dirname(__file__))

import main
from mission_loading import MissionLoadConflict, start_mission_for_controller
from models import SprayTestRequest
from routes.spray import spray_on, spray_test
from spray_safety import (
    CANONICAL_SPRAY_TELEMETRY_FIELDS,
    build_spray_telemetry_fields,
    disarm_with_spray_safety,
    set_mode_with_spray_safety,
)
from spray_startup_reconciliation import SprayStartupReconciliation


class _RosNoBridge:
    pass


class _RosResidual:
    def __init__(self, *, runtime=None, off_confirms=True):
        self.runtime = runtime or {
            "status_stale": False,
            "active_dwell": False,
            "commanded_on": False,
            "confirmed_off": True,
            "off_acknowledged": True,
            "accepted_command_on": False,
            "pending_command": False,
        }
        self.off_confirms = off_confirms
        self.manual_calls = []
        self.cancel_calls = 0
        self._reads = 0

    def get_spray_runtime_status(self):
        self._reads += 1
        status = dict(self.runtime)
        if self.off_confirms and self._reads > 1:
            status.update(
                {
                    "commanded_on": False,
                    "confirmed_off": True,
                    "off_acknowledged": True,
                    "accepted_command_on": False,
                    "pending_command": False,
                }
            )
        return status

    def publish_spray_manual(self, on: bool):
        self.manual_calls.append(bool(on))

    async def cancel_spray_dwell_async(self):
        self.cancel_calls += 1
        return True, "ok"


class _RosSprayOn:
    def __init__(self, *, accept=True):
        self.accept = accept
        self.manual_calls = []
        self._reads = 0

    def get_state(self):
        return {"armed": True, "mode": "MANUAL"}

    def publish_spray_manual(self, on: bool):
        self.manual_calls.append(bool(on))

    def get_spray_runtime_status(self):
        self._reads += 1
        spraying = self.accept and any(self.manual_calls)
        return {
            "status_stale": False,
            "status_age_s": 0.01,
            "accepted_command_on": spraying,
            "commanded_on": spraying,
            "pending_command": False,
            "confirmed_off": not spraying,
            "off_acknowledged": not spraying,
            "physical_confirmation_available": False,
        }

    async def cancel_spray_dwell_async(self):
        return True, "ok"


class _RosArm:
    def __init__(self, *, spray_stays_on=False, mode="OFFBOARD"):
        self.mode = mode
        self.armed = True
        self.arm_calls = []
        self.mode_calls = []
        self._spray_on = True if spray_stays_on else False

    def get_state(self):
        return {"mode": self.mode, "armed": self.armed}

    async def arm_async(self, arm: bool):
        self.arm_calls.append(arm)
        if not arm:
            self.armed = False
        return True, ""

    async def set_mode_async(self, mode: str):
        self.mode_calls.append(mode)
        self.mode = mode
        return True, ""

    def get_spray_runtime_status(self):
        return {
            "status_stale": False,
            "accepted_command_on": self._spray_on,
            "commanded_on": self._spray_on,
            "pending_command": False,
            "confirmed_off": not self._spray_on,
            "off_acknowledged": not self._spray_on,
            "physical_confirmation_available": False,
        }

    def publish_spray_manual(self, on: bool):
        if not on and not self._spray_on:
            pass
        elif not on:
            pass

    async def cancel_spray_dwell_async(self):
        return True, "ok"


@pytest.mark.asyncio
async def test_no_bridge_startup_marks_recovery_required():
    reconciler = SprayStartupReconciliation()
    reconciler.mark_bridge_unavailable()
    assert reconciler.is_ready() is False
    assert reconciler.state.recovery_required is True
    assert reconciler.blocks_mission_operations() is True
    assert "spray runtime unavailable" in reconciler.block_reason()


@pytest.mark.asyncio
async def test_spray_runtime_unavailable_at_startup_sets_recovery():
    ros = _RosNoBridge()
    reconciler = SprayStartupReconciliation()
    await reconciler.start(ros)
    assert reconciler.state.recovery_required is True
    assert reconciler.is_ready() is False


@pytest.mark.asyncio
async def test_stale_runtime_at_startup_sets_recovery():
    ros = _RosResidual(
        runtime={
            "status_stale": True,
            "active_dwell": False,
            "commanded_on": False,
            "confirmed_off": True,
        }
    )
    reconciler = SprayStartupReconciliation()
    await reconciler.start(ros)
    assert reconciler.state.recovery_required is True
    assert reconciler.is_ready() is False


@pytest.mark.asyncio
async def test_bridge_available_later_reconciliation_succeeds():
    ros = _RosResidual()
    reconciler = SprayStartupReconciliation()
    await reconciler.start(ros)
    assert reconciler.is_ready() is True
    assert reconciler.state.recovery_required is False
    assert reconciler.blocks_mission_operations() is False


@pytest.mark.asyncio
async def test_mission_blocked_until_recovery_cleared(monkeypatch):
    reconciler = SprayStartupReconciliation()
    reconciler.mark_bridge_unavailable()
    ctrl = type("C", (), {"spray_mode": "continuous", "has_protected_mission": False})()
    monkeypatch.setattr(main, "spray_startup_reconciliation", reconciler)
    with pytest.raises(MissionLoadConflict):
        await start_mission_for_controller(ctrl, None, _RosResidual(), name="x")


@pytest.mark.asyncio
async def test_disarm_with_spray_safety_success():
    ros = _RosArm(spray_stays_on=False)
    result = await disarm_with_spray_safety(ros)
    assert result.success is True
    assert result.transition_ok is True
    assert ros.arm_calls == [False]


@pytest.mark.asyncio
async def test_disarm_blocked_when_off_unconfirmed():
    ros = _RosArm(spray_stays_on=True)
    result = await disarm_with_spray_safety(ros)
    assert result.success is False
    assert result.degraded is True


@pytest.mark.asyncio
async def test_offboard_exit_requires_confirmed_off():
    ros = _RosArm(mode="OFFBOARD", spray_stays_on=False)
    result = await set_mode_with_spray_safety(
        ros, target_mode="MANUAL", current_mode="OFFBOARD"
    )
    assert result.success is True
    assert ros.mode_calls == ["MANUAL"]


@pytest.mark.asyncio
async def test_offboard_exit_blocked_when_off_unconfirmed():
    ros = _RosArm(mode="OFFBOARD", spray_stays_on=True)
    result = await set_mode_with_spray_safety(
        ros, target_mode="MANUAL", current_mode="OFFBOARD"
    )
    assert result.success is False
    assert result.degraded is True


def test_canonical_telemetry_includes_vehicle_and_dry_run_fields():
    fields = build_spray_telemetry_fields(
        legacy_spraying=False,
        spray_rt={
            "status_stale": False,
            "status_age_s": 0.02,
            "accepted_command_on": False,
            "pending_command": False,
            "off_acknowledged": True,
            "flow_mode": "disabled",
            "dry_run_active": True,
            "geometry_spray_request": True,
            "vehicle_state_age_s": 0.1,
            "vehicle_state_stale": False,
            "vehicle_state_block_reason": "",
        },
        mission_running=True,
        mission_dash={
            "dash_feasible": True,
            "dash_feasibility_reason": "",
            "dash_expected_speed_mps": 0.35,
            "dash_feasibility_speed_source": "staged_marking_speed_mps",
            "dash_phase_reset": "per_mark_region",
        },
    )
    for key in CANONICAL_SPRAY_TELEMETRY_FIELDS:
        assert key in fields
    assert fields["spraying"] is False
    assert fields["dry_run_active"] is True
    assert fields["geometry_spray_request"] is True
    assert fields["dash_expected_speed_mps"] == 0.35


def test_disabled_flow_does_not_report_spraying():
    fields = build_spray_telemetry_fields(
        legacy_spraying=True,
        spray_rt={
            "status_stale": False,
            "accepted_command_on": True,
            "commanded_on": True,
            "desired_on": True,
            "pending_command": False,
            "flow_mode": "disabled",
            "dry_run_active": True,
            "geometry_spray_request": True,
        },
        mission_running=True,
    )
    assert fields["spraying"] is False
    assert fields["accepted_command_on"] is False
    assert fields["commanded_on"] is False
    assert fields["dry_run_active"] is True
    assert fields["geometry_spray_request"] is True
    assert fields["marking_state"] == "transit"
    assert fields["spray_state"] == "DRY_RUN"


def test_spray_on_waits_for_acceptance(monkeypatch):
    import routes.spray as spray_module

    node = _RosSprayOn(accept=True)
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", type("C", (), {"state": "idle"})())
    spray_module._spray_enabled = True

    async def run():
        resp = await spray_on()
        assert resp["accepted_on"] is True
        assert resp["spraying"] is True

    asyncio.run(run())


def test_spray_on_rejected_when_not_accepted(monkeypatch):
    import routes.spray as spray_module

    node = _RosSprayOn(accept=False)
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", type("C", (), {"state": "idle"})())
    spray_module._spray_enabled = True

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_on()
        assert exc.value.status_code in {409, 503}
        assert exc.value.detail["accepted_on"] is False

    asyncio.run(run())


def test_spray_test_requires_diagnostic_authorization(monkeypatch):
    import routes.spray as spray_module

    node = _RosSprayOn(accept=True)
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(main, "offboard_ctrl", type("C", (), {"state": "idle"})())
    spray_module._spray_enabled = True

    async def run():
        with pytest.raises(HTTPException) as exc:
            await spray_test(SprayTestRequest(on=True, duration_s=1.0))
        assert exc.value.status_code == 403

    asyncio.run(run())
