#!/usr/bin/env python3
"""Point run-loop failure routing through terminal_cleanup."""

from __future__ import annotations

import asyncio
import os
import sys
import time
from collections import deque
from unittest.mock import AsyncMock, MagicMock

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import main
from mission_loading import start_mission_for_controller
from mission_ops import MissionOperationCoordinator
from models import MissionState
from offboard_controller import OffboardController
from point_events import get_point_event_journal, reset_point_event_journal_for_tests
from point_ingest import SprayPoint
from point_mission import PointMissionOrchestrator, PointMissionState
from spray_config import PointSprayParams, SprayConfiguration, SprayMode
from test_point_mission import FakeOffboard, FakeRos


@pytest.fixture(autouse=True)
def _coordinator():
    main.operation_coordinator = MissionOperationCoordinator()
    reset_point_event_journal_for_tests()
    yield


def _cfg(dwell_s=0.15):
    return SprayConfiguration(
        mode=SprayMode.POINT,
        point=PointSprayParams(
            default_dwell_s=dwell_s,
            arrival_tolerance_m=0.05,
            settle_time_s=0.0,
            leg_timeout_s=3.0,
        ),
        revision=1,
    )


class RestartFaultRos(FakeRos):
    """After dwell fingerprint is bound, report a controller restart on next poll."""

    def __init__(self):
        super().__init__()
        self.auto_arrive = True
        self._dwell_polls = 0

    def get_spray_runtime_status(self):
        status = super().get_spray_runtime_status()
        if status.get("active_dwell"):
            self._dwell_polls += 1
            if self._dwell_polls >= 2:
                self.live_dwell = None
                status = {
                    **status,
                    "configuration_revision": 99,
                    "model_revision": 1,
                    "active_dwell": False,
                    "commanded_on": False,
                    "confirmed_off": True,
                    "off_acknowledged": True,
                    "dwell_remaining_s": 0.0,
                }
        return status


@pytest.mark.anyio
async def test_spray_controller_restart_during_dwell_emits_point_failed():
    ros = RestartFaultRos()
    offboard = FakeOffboard()
    offboard.abort_count = 0
    original_abort = offboard.abort_async

    async def counted_abort():
        offboard.abort_count += 1
        return await original_abort()

    offboard.abort_async = counted_abort

    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="m1",
        points=[SprayPoint(0, 0, 5.0, 0, mark=True)],
        config=_cfg(dwell_s=5.0),
    )
    await orch.start(ros, offboard)
    for _ in range(80):
        if orch.status.state == PointMissionState.FAILED:
            break
        await asyncio.sleep(0.05)
    assert orch.status.state == PointMissionState.FAILED
    assert offboard.abort_count == 1
    terminal = [
        e for e in get_point_event_journal().history()["events"] if e.terminal
    ]
    assert len(terminal) == 1
    assert terminal[0].event_type == "point_failed"
    assert terminal[0].reason == "dwell_fault"


@pytest.mark.anyio
async def test_start_failure_runs_point_terminal_cleanup(monkeypatch):
    node = FakeRos()
    ctrl = OffboardController(node, deque())
    ctrl.load_path(
        [(0.0, 0.0), (1.0, 0.0)],
        name="sf.csv",
        spray_flags=[False, False],
        mission_id="sf1",
        is_staged=True,
        allow_replace_protected=True,
        spray_mode="point",
    )
    ctrl.state = MissionState.IDLE
    ctrl.start_async = AsyncMock(return_value=(True, "started"))
    ctrl.stop_async = AsyncMock(return_value={"success": True})

    orch = PointMissionOrchestrator()
    orch.prepare = MagicMock()
    orch.start = AsyncMock(return_value=(False, "point start failed"))
    terminal_cleanup = AsyncMock(
        return_value=MagicMock(
            success=True,
            idempotent=False,
            terminal_event_emitted=True,
        )
    )
    orch.terminal_cleanup = terminal_cleanup

    monkeypatch.setattr(main, "point_mission", orch)
    monkeypatch.setattr(main, "hold_owner", None)
    monkeypatch.setattr(main, "operation_coordinator", MissionOperationCoordinator())

    path_mgr = MagicMock()
    path_mgr.load_path.return_value = [(0.0, 0.0), (1.0, 0.0)]
    path_mgr.preview_path.return_value = MagicMock(
        waypoints=[MagicMock(spray=True), MagicMock(spray=True)]
    )

    ok, message = await start_mission_for_controller(
        ctrl,
        path_mgr,
        node,
        mission_id="sf1",
    )

    assert ok is False
    assert message == "point start failed"
    terminal_cleanup.assert_awaited_once()
    call_kwargs = terminal_cleanup.await_args.kwargs
    assert call_kwargs["reason"] == "start_failure"
    assert call_kwargs["terminal_state"] == PointMissionState.FAILED