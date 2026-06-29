"""Tests for spray startup residual-dwell reconciliation."""

from __future__ import annotations

import asyncio
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from mission_loading import MissionLoadConflict, start_mission_for_controller
from spray_startup_reconciliation import (
    SprayStartupReconciliation,
    indicates_residual_spray_activity,
)


class ResidualSprayRos:
    def __init__(self, *, runtime=None, off_confirms=True):
        self.runtime = runtime or {
            "status_stale": False,
            "active_dwell": True,
            "commanded_on": True,
            "confirmed_off": False,
            "off_acknowledged": False,
            "accepted_command_on": True,
            "pending_command": False,
            "pending_command_on": False,
        }
        self.off_confirms = off_confirms
        self.manual_calls = []
        self.cancel_calls = 0
        self._reads = 0

    def get_spray_runtime_status(self):
        self._reads += 1
        status = dict(self.runtime)
        if self.off_confirms and self._reads > 1 and self.cancel_calls > 0:
            status.update(
                {
                    "active_dwell": False,
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
        self.runtime["active_dwell"] = False
        return True, "ok"


@pytest.mark.asyncio
async def test_residual_detection_flags_command_layer_on_states():
    residual, reason = indicates_residual_spray_activity(
        {
            "status_stale": False,
            "active_dwell": False,
            "commanded_on": True,
            "confirmed_off": True,
            "accepted_command_on": False,
            "pending_command": False,
        }
    )
    assert residual is True
    assert "commanded_on" in reason


@pytest.mark.asyncio
async def test_stale_status_treated_as_residual():
    residual, reason = indicates_residual_spray_activity(
        {
            "status_stale": True,
            "active_dwell": False,
            "commanded_on": False,
            "confirmed_off": True,
        }
    )
    assert residual is True
    assert "stale" in reason


@pytest.mark.asyncio
async def test_startup_reconcile_cancels_active_dwell():
    ros = ResidualSprayRos()
    reconciler = SprayStartupReconciliation()
    await reconciler.start(ros)
    assert reconciler.is_ready()
    assert reconciler.state.residual_detected is True
    assert ros.cancel_calls >= 1
    assert False in ros.manual_calls
    assert reconciler.state.recovery_required is False
    assert reconciler.state.spray_off_result["success"] is True


@pytest.mark.asyncio
async def test_startup_reconcile_commanded_on_without_active_dwell():
    ros = ResidualSprayRos(
        runtime={
            "status_stale": False,
            "active_dwell": False,
            "commanded_on": True,
            "confirmed_off": False,
            "off_acknowledged": False,
            "accepted_command_on": True,
            "pending_command": False,
        }
    )
    reconciler = SprayStartupReconciliation()
    await reconciler.start(ros)
    assert reconciler.state.residual_detected is True
    assert ros.cancel_calls >= 1
    assert reconciler.state.recovery_required is False


@pytest.mark.asyncio
async def test_startup_reconcile_failure_sets_recovery_required():
    ros = ResidualSprayRos(off_confirms=False)
    reconciler = SprayStartupReconciliation()
    await reconciler.start(ros)
    assert reconciler.is_ready() is False
    assert reconciler.state.recovery_required is True
    assert reconciler.state.reason


@pytest.mark.asyncio
async def test_mission_start_blocked_until_reconciliation_completes(monkeypatch):
    import main

    ros = ResidualSprayRos()
    reconciler = SprayStartupReconciliation()
    task = reconciler.start(ros)

    ctrl = type("C", (), {"spray_mode": "continuous", "has_protected_mission": False})()
    monkeypatch.setattr(main, "spray_startup_reconciliation", reconciler)

    with pytest.raises(MissionLoadConflict, match="reconciliation in progress"):
        await start_mission_for_controller(ctrl, None, ros, name="x")

    await task
    assert reconciler.is_ready()


@pytest.mark.asyncio
async def test_mission_start_blocked_when_recovery_required(monkeypatch):
    import main

    ros = ResidualSprayRos(off_confirms=False)
    reconciler = SprayStartupReconciliation()
    await reconciler.start(ros)
    assert reconciler.state.recovery_required is True

    ctrl = type("C", (), {"spray_mode": "continuous", "has_protected_mission": False})()
    monkeypatch.setattr(main, "spray_startup_reconciliation", reconciler)

    with pytest.raises(MissionLoadConflict):
        await start_mission_for_controller(ctrl, None, ros, name="x")