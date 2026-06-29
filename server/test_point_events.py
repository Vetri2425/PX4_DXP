#!/usr/bin/env python3
"""Point event journal tests."""

from __future__ import annotations

import asyncio
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from models import PointMissionEvent
from point_events import PointEventJournal, reset_point_event_journal_for_tests
from point_mission import PointMissionOrchestrator, PointMissionState
from test_point_mission import FakeOffboard, FakeRos
from point_ingest import SprayPoint
from spray_config import PointSprayParams, SprayConfiguration, SprayMode


def _event(**kwargs) -> PointMissionEvent:
    base = dict(
        ts="2026-06-29T00:00:00.000Z",
        event_type="point_leg_started",
        mission_id="m1",
        parent_mission_id="m1",
        point_mission_generation=1,
        point_mission_state="navigating",
        hold_active=False,
        obstacle_signal_state="not_configured",
        gps_safety_state="not_applicable",
        terminal=False,
        reason="",
        status={},
    )
    base.update(kwargs)
    return PointMissionEvent(**base)


def test_point_event_journal_capacity_512():
    journal = PointEventJournal()
    for i in range(600):
        journal.append(_event(mission_id=f"m{i % 3}"))
    hist = journal.history()
    assert len(hist["events"]) == 512
    assert hist["latest_event_id"] == 600
    assert hist["oldest_available_event_id"] == 89


def test_point_event_history_evicted_flag():
    journal = PointEventJournal()
    for i in range(520):
        journal.append(_event())
    hist = journal.history(since_event_id=500)
    assert hist["history_evicted"] is False
    hist2 = journal.history(since_event_id=1)
    assert hist2["history_evicted"] is True


def test_point_events_emit_nonblocking_when_socket_emit_fails():
    async def run():
        journal = PointEventJournal()
        loop = asyncio.get_running_loop()

        async def bad_emit(_event, _payload):
            raise RuntimeError("socket down")

        journal.configure_emit(loop, bad_emit)
        journal.append(_event())
        await asyncio.sleep(0.01)

    asyncio.run(run())


def test_manual_point_event_sequence_wait_continue_complete():
    async def run():
        reset_point_event_journal_for_tests()
        ros = FakeRos()
        ros.auto_arrive = True
        offboard = FakeOffboard()
        orch = PointMissionOrchestrator()
        from point_mission import PointExecutionMode

        orch.load(
            mission_id="m1",
            points=[SprayPoint(0, 0, 0.05, 0), SprayPoint(1, 0, 0.05, 1)],
            config=SprayConfiguration(mode=SprayMode.POINT, revision=1),
            execution_mode=PointExecutionMode.MANUAL,
        )
        await orch.start(ros, offboard)
        await asyncio.sleep(0.2)
        assert orch.status.state == PointMissionState.WAITING_FOR_CONTINUE
        await orch.continue_point(ros)
        await asyncio.sleep(0.3)
        from point_events import get_point_event_journal

        types = [e.event_type for e in get_point_event_journal().history()["events"]]
        assert "point_waiting_for_continue" in types
        assert "point_completed" in types

    asyncio.run(run())