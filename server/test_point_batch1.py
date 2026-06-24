"""Batch 1 integration tests: point clear lifecycle and load hard-fail."""

from __future__ import annotations

import asyncio
import os
import sys

import pytest
from fastapi import HTTPException

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import main
from models import MissionState
from offboard_controller import OffboardController
from point_ingest import SprayPoint
from point_mission import PointMissionOrchestrator, PointMissionState
from routes.mission import clear_mission
from spray_config import SprayConfiguration, SprayMode
from test_point_mission import FakeRos


class FakeNode:
    def get_rpp_monitor(self):
        class Monitor:
            def reset(self):
                return None

        return Monitor()


def _load_point_mission(ctrl: OffboardController) -> PointMissionOrchestrator:
    orch = PointMissionOrchestrator()
    cfg = SprayConfiguration(mode=SprayMode.POINT)
    orch.load(
        mission_id="pt_batch1",
        points=[SprayPoint(1.0, 2.0, 1.0, 0), SprayPoint(3.0, 4.0, 1.0, 1)],
        config=cfg,
    )
    orch._resolved_points = list(orch._points)
    ctrl.load_path(
        [(1.0, 2.0), (1.0, 2.0)],
        name="point.csv",
        spray_flags=[False, False],
        mission_id="pt_batch1",
        is_staged=True,
        allow_replace_protected=True,
        spray_mode="point",
    )
    ctrl.state = MissionState.IDLE
    return orch


@pytest.mark.anyio
async def test_clear_drains_point_mission_before_controller(monkeypatch):
    ctrl = OffboardController(FakeNode(), __import__("collections").deque())
    ros = FakeRos()
    orch = _load_point_mission(ctrl)
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "point_mission", orch)
    monkeypatch.setattr(main, "ros_node", ros)

    response = await clear_mission()

    assert response.cleared is True
    assert orch.status.state == PointMissionState.IDLE
    assert orch.status.ready is False
    assert orch._points == []
    assert orch._resolved_points == []
    assert orch._config is None
    assert orch._run_token is None
    assert response.status.loaded is False
    assert ctrl.state == MissionState.IDLE


@pytest.mark.anyio
async def test_clear_cancels_active_point_task(monkeypatch):
    ctrl = OffboardController(FakeNode(), __import__("collections").deque())
    ros = FakeRos()
    orch = _load_point_mission(ctrl)
    offboard = type("Offboard", (), {"state": MissionState.RUNNING, "_running_mission_id": "pt_batch1"})()
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "point_mission", orch)
    monkeypatch.setattr(main, "ros_node", ros)

    started, _ = await orch.start(ros, offboard)
    assert started
    await asyncio.sleep(0.05)

    ctrl.state = MissionState.IDLE
    response = await clear_mission()

    assert response.cleared is True
    assert orch.status.state == PointMissionState.IDLE
    assert orch._points == []
    assert orch._task is None