"""Batch 2 tests: execution modes, continue endpoint, mark=false, diagnostics."""

from __future__ import annotations

import asyncio
import os
import sys
import time

import pytest
from fastapi import HTTPException

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import main
from models import MissionState
from offboard_controller import OffboardController
from point_ingest import SprayPoint
from point_mission import PointExecutionMode, PointMissionOrchestrator, PointMissionState
from routes.mission import clear_mission, point_mission_continue, point_mission_status
from spray_config import PointSprayParams, SprayConfiguration, SprayMode, staged_spray_defaults
from test_point_mission import FakeOffboard, FakeRos


def _fast_cfg(**overrides) -> SprayConfiguration:
    point = PointSprayParams(
        default_dwell_s=0.04,
        arrival_tolerance_m=0.05,
        settle_time_s=0.0,
        leg_timeout_s=2.0,
        settle_speed_mps=0.05,
        settle_yaw_rate_rad_s=0.05,
    )
    return SprayConfiguration(mode=SprayMode.POINT, point=point, revision=1, **overrides)


def _make_ctrl_point_mode() -> OffboardController:
    class FakeNode:
        def get_rpp_monitor(self):
            class Monitor:
                def reset(self):
                    return None

            return Monitor()

    ctrl = OffboardController(FakeNode(), __import__("collections").deque())
    ctrl.load_path(
        [(0.0, 0.0), (0.0, 0.0)],
        name="point.csv",
        spray_flags=[False, False],
        mission_id="pt2",
        is_staged=True,
        allow_replace_protected=True,
        spray_mode="point",
    )
    ctrl.state = MissionState.RUNNING
    return ctrl


async def _wait_for_state(orch, state: PointMissionState, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if orch.status.state == state:
            return
        await asyncio.sleep(0.02)
    raise AssertionError(f"timed out waiting for {state.value}, got {orch.status.state.value}")


async def _wait_for_path_count(ros: FakeRos, count: int, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if len(ros.paths) >= count:
            return
        await asyncio.sleep(0.02)
    raise AssertionError(f"timed out waiting for {count} path publish(es), got {len(ros.paths)}")


@pytest.mark.anyio
async def test_auto_mode_advances_all_points():
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="auto3",
        points=[SprayPoint(i, i, 0.04, i) for i in range(3)],
        config=_fast_cfg(),
        execution_mode=PointExecutionMode.AUTO,
    )
    started, _ = await orch.start(ros, FakeOffboard())
    assert started
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state == PointMissionState.COMPLETED
    assert len(ros.paths) == 3
    assert len(ros.dwells) == 3
    assert orch.status.waiting_for_continue is False


@pytest.mark.anyio
async def test_manual_mode_pauses_after_first_point_without_next_leg():
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="manual3",
        points=[SprayPoint(i, i, 0.04, i) for i in range(3)],
        config=_fast_cfg(),
        execution_mode=PointExecutionMode.MANUAL,
    )
    started, _ = await orch.start(ros, FakeOffboard())
    assert started
    await _wait_for_state(orch, PointMissionState.WAITING_FOR_CONTINUE)
    assert len(ros.paths) == 1
    assert orch.status.last_completed_point_index == 0
    assert orch.status.next_point_index == 1
    assert orch.status.waiting_for_continue is True
    assert orch.status.point_execution_mode == "manual"


@pytest.mark.anyio
async def test_continue_publishes_exactly_one_next_leg():
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="manual3",
        points=[SprayPoint(i, i, 0.04, i) for i in range(3)],
        config=_fast_cfg(),
        execution_mode=PointExecutionMode.MANUAL,
    )
    await orch.start(ros, FakeOffboard())
    await _wait_for_state(orch, PointMissionState.WAITING_FOR_CONTINUE)
    ok, msg, code = await orch.continue_point()
    assert ok and code == 200
    await _wait_for_path_count(ros, 2)
    await _wait_for_state(orch, PointMissionState.WAITING_FOR_CONTINUE)
    ok, msg, code = await orch.continue_point()
    assert ok and code == 200
    await _wait_for_path_count(ros, 3)
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state == PointMissionState.COMPLETED
    assert len(ros.paths) == 3


@pytest.mark.anyio
async def test_duplicate_continue_rejected():
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    slow_cfg = SprayConfiguration(
        mode=SprayMode.POINT,
        point=PointSprayParams(
            default_dwell_s=1.0,
            arrival_tolerance_m=0.05,
            settle_time_s=0.0,
            leg_timeout_s=2.0,
            settle_speed_mps=0.05,
            settle_yaw_rate_rad_s=0.05,
        ),
        revision=1,
    )
    orch.load(
        mission_id="dup",
        points=[SprayPoint(i, i, 1.0, i) for i in range(3)],
        config=slow_cfg,
        execution_mode=PointExecutionMode.MANUAL,
    )
    await orch.start(ros, FakeOffboard())
    await _wait_for_state(orch, PointMissionState.WAITING_FOR_CONTINUE)
    ok, _, code = await orch.continue_point()
    assert ok and code == 200
    await _wait_for_path_count(ros, 2)
    ok, msg, code = await orch.continue_point()
    assert not ok and code == 409
    assert "not waiting for continue" in msg


@pytest.mark.anyio
async def test_premature_continue_rejected():
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="early",
        points=[SprayPoint(0, 0, 0.04, 0)],
        config=_fast_cfg(),
        execution_mode=PointExecutionMode.MANUAL,
    )
    ok, msg, code = await orch.continue_point()
    assert not ok and code == 409
    assert "not waiting for continue" in msg


@pytest.mark.anyio
async def test_final_point_completes_without_waiting():
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="single",
        points=[SprayPoint(0, 0, 0.04, 0)],
        config=_fast_cfg(),
        execution_mode=PointExecutionMode.MANUAL,
    )
    await orch.start(ros, FakeOffboard())
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state == PointMissionState.COMPLETED
    assert orch.status.waiting_for_continue is False
    ok, msg, code = await orch.continue_point()
    assert not ok and code == 409
    assert "completed" in msg


@pytest.mark.anyio
async def test_abort_while_waiting_drains_cleanly():
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="abort_wait",
        points=[SprayPoint(0, 0, 0.04, 0), SprayPoint(1, 1, 0.04, 1)],
        config=_fast_cfg(),
        execution_mode=PointExecutionMode.MANUAL,
    )
    await orch.start(ros, FakeOffboard())
    await _wait_for_state(orch, PointMissionState.WAITING_FOR_CONTINUE)
    await orch.abort(ros)
    assert orch.status.state in {PointMissionState.FAILED, PointMissionState.ABORTING}
    assert orch.status.waiting_for_continue is False
    assert orch.is_active() is False


@pytest.mark.anyio
async def test_clear_while_waiting_drains_cleanly(monkeypatch):
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    ctrl = _make_ctrl_point_mode()
    orch.load(
        mission_id="clear_wait",
        points=[SprayPoint(0, 0, 0.04, 0), SprayPoint(1, 1, 0.04, 1)],
        config=_fast_cfg(),
        execution_mode=PointExecutionMode.MANUAL,
    )
    await orch.start(ros, FakeOffboard())
    await _wait_for_state(orch, PointMissionState.WAITING_FOR_CONTINUE)
    ctrl.state = MissionState.IDLE
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "point_mission", orch)
    monkeypatch.setattr(main, "ros_node", ros)
    await clear_mission()
    assert orch.status.state == PointMissionState.IDLE
    assert orch.status.target_north_m is None
    assert orch.status.next_point_index is None
    assert orch._points == []


@pytest.mark.anyio
async def test_replaced_run_cannot_consume_stale_continue():
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    cfg = _fast_cfg()
    orch.load(
        mission_id="old",
        points=[SprayPoint(0, 0, 0.04, 0), SprayPoint(1, 1, 0.04, 1)],
        config=cfg,
        execution_mode=PointExecutionMode.MANUAL,
    )
    await orch.start(ros, FakeOffboard())
    await _wait_for_state(orch, PointMissionState.WAITING_FOR_CONTINUE)
    old_run = orch._run_token
    assert old_run is not None
    if old_run.continue_gate is not None and not old_run.continue_gate.done():
        old_run.continue_gate.set_result(True)
    await asyncio.sleep(0.05)
    staged = {
        "mission_id": "new",
        "point_mission_points": [
            {"north_m": 2.0, "east_m": 2.0, "dwell_s": 0.04, "source_index": 0, "mark": True}
        ],
        "point_source_frame": "LOCAL_NED",
        "point_execution_mode": "manual",
    }
    await orch.replace_from_staged(staged, cfg, ros)
    ok, msg, code = await orch.continue_point()
    assert not ok and code == 409
    assert "not waiting for continue" in msg


@pytest.mark.anyio
async def test_mark_false_starts_no_dwell():
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="nav_only",
        points=[SprayPoint(1.0, 2.0, 2.0, 0, mark=False)],
        config=_fast_cfg(),
    )
    await orch.start(ros, FakeOffboard())
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state == PointMissionState.COMPLETED
    assert len(ros.paths) == 1
    assert len(ros.dwells) == 0


@pytest.mark.anyio
async def test_mark_false_still_settles_and_confirms_off():
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="nav_only",
        points=[SprayPoint(1.0, 2.0, 2.0, 0, mark=False)],
        config=_fast_cfg(),
    )
    await orch.start(ros, FakeOffboard())
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.settle_met is True
    assert orch.status.arrival_met is True
    assert ros.live_dwell is None


@pytest.mark.anyio
async def test_mixed_mark_mission():
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="mixed",
        points=[
            SprayPoint(0, 0, 0.04, 0, mark=False),
            SprayPoint(1, 1, 0.04, 1, mark=True),
        ],
        config=_fast_cfg(),
        execution_mode=PointExecutionMode.AUTO,
    )
    await orch.start(ros, FakeOffboard())
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert len(ros.paths) == 2
    assert len(ros.dwells) == 1


@pytest.mark.anyio
async def test_diagnostics_across_major_states():
    ros = FakeRos()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="diag",
        points=[SprayPoint(3.0, 4.0, 0.04, 0)],
        config=_fast_cfg(),
    )
    assert orch.status.point_execution_mode == "auto"
    assert orch.status.ready is True
    await orch.start(ros, FakeOffboard())
    await asyncio.sleep(0.05)
    assert orch.status.run_active is True
    assert orch.status.target_north_m == 3.0
    assert orch.status.target_east_m == 4.0
    assert orch.status.mark_enabled is True
    await orch.abort(ros)
    assert orch.status.last_failure_reason == "cancelled"


@pytest.mark.anyio
async def test_legacy_staged_defaults_to_auto_and_mark_true():
    defaults = staged_spray_defaults()
    assert defaults["point_execution_mode"] == "auto"
    staged = {
        "mission_id": "legacy",
        "point_mission_points": [{"north_m": 1, "east_m": 2, "dwell_s": 1, "source_index": 0}],
        "point_source_frame": "LOCAL_NED",
    }
    ros = FakeRos()
    orch = PointMissionOrchestrator()
    await orch.replace_from_staged(staged, _fast_cfg(), ros)
    assert orch.status.point_execution_mode == "auto"
    assert orch._points[0].mark is True


@pytest.mark.anyio
async def test_continue_endpoint_success_and_rejections(monkeypatch):
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    ctrl = _make_ctrl_point_mode()
    orch.load(
        mission_id="api",
        points=[SprayPoint(0, 0, 0.04, 0), SprayPoint(1, 1, 0.04, 1)],
        config=_fast_cfg(),
        execution_mode=PointExecutionMode.MANUAL,
    )
    monkeypatch.setattr(main, "point_mission", orch)
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "ros_node", ros)

    with pytest.raises(HTTPException) as exc:
        await point_mission_continue()
    assert exc.value.status_code == 409

    await orch.start(ros, FakeOffboard())
    await _wait_for_state(orch, PointMissionState.WAITING_FOR_CONTINUE)
    response = await point_mission_continue()
    assert response.continued is True
    await asyncio.sleep(0.05)
    assert orch.status.waiting_for_continue is False

    ctrl._spray_mode = "continuous"
    with pytest.raises(HTTPException) as exc:
        await point_mission_continue()
    assert exc.value.status_code == 409


@pytest.mark.anyio
async def test_point_status_endpoint(monkeypatch):
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="status_api",
        points=[SprayPoint(1, 2, 0.04, 0)],
        config=_fast_cfg(),
    )
    monkeypatch.setattr(main, "point_mission", orch)
    payload = await point_mission_status()
    assert payload.point_mission_id == "status_api"
    assert payload.total_points == 1
    assert payload.point_execution_mode == "auto"