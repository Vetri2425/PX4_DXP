"""Batch 3 tests: pause/resume, obstacle hook, hold ownership, unified stop."""

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
from mission_stop import stop_active_mission
from models import MissionState, ObstacleStatusRequest
from offboard_controller import OffboardController
from point_ingest import SprayPoint
from point_mission import PointExecutionMode, PointMissionOrchestrator, PointMissionState
from routes.mission import (
    pause_mission,
    point_mission_continue,
    resume_mission,
    set_obstacle_status,
    stop_mission,
)
from setpoint_hold import SetpointHoldOwner
from spray_config import (
    ObstacleSafetyParams,
    PointSprayParams,
    SprayConfiguration,
    SprayMode,
)
from test_point_mission import FakeOffboard, FakeRos


def _fast_cfg(**overrides) -> SprayConfiguration:
    point = PointSprayParams(
        default_dwell_s=0.08,
        arrival_tolerance_m=0.05,
        settle_time_s=0.0,
        leg_timeout_s=2.0,
        settle_speed_mps=0.05,
        settle_yaw_rate_rad_s=0.05,
    )
    return SprayConfiguration(mode=SprayMode.POINT, point=point, revision=1, **overrides)


def _obstacle_cfg(**overrides) -> SprayConfiguration:
    """Fast config with the obstacle hook explicitly enabled (large stale window)."""
    return _fast_cfg(
        obstacle=ObstacleSafetyParams(enabled=True, signal_max_age_s=5.0),
        **overrides,
    )


def _slow_dwell_cfg(dwell_s: float = 1.0, **overrides) -> SprayConfiguration:
    return SprayConfiguration(
        mode=SprayMode.POINT,
        point=PointSprayParams(
            default_dwell_s=dwell_s,
            arrival_tolerance_m=0.05,
            settle_time_s=0.0,
            leg_timeout_s=3.0,
            settle_speed_mps=0.05,
            settle_yaw_rate_rad_s=0.05,
        ),
        revision=1,
        **overrides,
    )


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
        mission_id="pt3",
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


@pytest.mark.anyio
async def test_operator_pause_during_navigation():
    ros = FakeRos()
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="pause_nav",
        points=[SprayPoint(5.0, 0.0, 0.08, 0)],
        config=_fast_cfg(),
    )
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.sleep(0.05)
    ok, msg, code = await orch.pause_mission(ros, hold)
    assert ok and code == 200
    await _wait_for_state(orch, PointMissionState.PAUSED_HOLD)
    assert hold.active
    assert orch.status.pause_reason == "operator"
    assert orch.status.pre_pause_state == PointMissionState.NAVIGATING.value
    assert orch.status.resume_available is True
    assert len([p for p in ros.paths if len(p[0]) == 1]) >= 1


@pytest.mark.anyio
async def test_resume_republishes_fresh_leg():
    ros = FakeRos()
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="resume_nav",
        points=[SprayPoint(5.0, 0.0, 0.08, 0)],
        config=_fast_cfg(),
    )
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.sleep(0.05)
    await orch.pause_mission(ros, hold)
    await _wait_for_state(orch, PointMissionState.PAUSED_HOLD)
    paths_before = len(ros.paths)
    ok, msg, code = await orch.resume_mission(ros, hold)
    assert ok and code == 200
    await asyncio.sleep(0.08)
    assert len(ros.paths) > paths_before
    assert not hold.active
    assert orch.status.state != PointMissionState.PAUSED_HOLD


@pytest.mark.anyio
async def test_obstacle_during_navigation_pauses_safely():
    ros = FakeRos()
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="obs_nav",
        points=[SprayPoint(4.0, 0.0, 0.08, 0)],
        config=_obstacle_cfg(),
    )
    orch.set_obstacle_clear(True)
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.sleep(0.05)
    orch.set_obstacle_clear(False)
    await _wait_for_state(orch, PointMissionState.PAUSED_OBSTACLE)
    assert hold.active
    assert orch.status.pause_reason == "obstacle"
    assert orch.status.obstacle_signal_state == "blocked"
    assert orch.status.active_dwell is False


@pytest.mark.anyio
async def test_obstacle_during_dwell_cancels_spray():
    ros = FakeRos()
    ros.auto_arrive = True
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="obs_dwell",
        points=[SprayPoint(1.0, 0.0, 1.0, 0)],
        config=_slow_dwell_cfg(1.0, obstacle=ObstacleSafetyParams(enabled=True, signal_max_age_s=5.0)),
    )
    orch.set_obstacle_clear(True)
    await orch.start(ros, FakeOffboard(), hold)
    await _wait_for_state(orch, PointMissionState.DWELLING)
    assert ros.live_dwell is not None
    orch.set_obstacle_clear(False)
    await _wait_for_state(orch, PointMissionState.PAUSED_OBSTACLE)
    assert ros.live_dwell is None
    assert orch.status.dwell_cancelled is True
    assert orch.status.pre_pause_state == PointMissionState.DWELLING.value
    assert hold.active


@pytest.mark.anyio
async def test_resume_blocked_while_obstacle_present():
    ros = FakeRos()
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="obs_block",
        points=[SprayPoint(2.0, 0.0, 0.08, 0)],
        config=_obstacle_cfg(),
    )
    orch.set_obstacle_clear(True)
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.sleep(0.05)
    orch.set_obstacle_clear(False)
    await _wait_for_state(orch, PointMissionState.PAUSED_OBSTACLE)
    ok, msg, code = await orch.resume_mission(ros, hold)
    assert not ok and code == 409
    assert "obstacle" in msg.lower()


@pytest.mark.anyio
async def test_continue_rejected_while_paused():
    ros = FakeRos()
    ros.auto_arrive = True
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="cont_pause",
        points=[SprayPoint(0, 0, 0.08, 0), SprayPoint(1, 1, 0.08, 1)],
        config=_fast_cfg(),
        execution_mode=PointExecutionMode.MANUAL,
    )
    await orch.start(ros, FakeOffboard(), hold)
    await _wait_for_state(orch, PointMissionState.WAITING_FOR_CONTINUE)
    await orch.pause_mission(ros, hold)
    await _wait_for_state(orch, PointMissionState.PAUSED_HOLD)
    ok, msg, code = await orch.continue_point()
    assert not ok and code == 409
    assert "paused" in msg


@pytest.mark.anyio
async def test_continue_rejected_immediately_after_obstacle_block():
    ros = FakeRos()
    ros.auto_arrive = True
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="cont_obs",
        points=[SprayPoint(0, 0, 0.08, 0), SprayPoint(1, 1, 0.08, 1)],
        config=_obstacle_cfg(),
        execution_mode=PointExecutionMode.MANUAL,
    )
    orch.set_obstacle_clear(True)
    await orch.start(ros, FakeOffboard(), hold)
    await _wait_for_state(orch, PointMissionState.WAITING_FOR_CONTINUE)
    orch.set_obstacle_clear(False)
    ok, msg, code = await orch.continue_point(ros)
    assert not ok and code == 409
    assert "obstacle" in msg.lower()


@pytest.mark.anyio
async def test_stale_generation_resume_rejected():
    ros = FakeRos()
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="stale",
        points=[SprayPoint(1.0, 0.0, 0.08, 0)],
        config=_fast_cfg(),
    )
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.sleep(0.05)
    await orch.pause_mission(ros, hold)
    await _wait_for_state(orch, PointMissionState.PAUSED_HOLD)
    ok, msg, code = await orch.resume_mission(
        ros, hold, expected_generation=orch.status.generation - 1
    )
    assert not ok and code == 409
    assert "stale" in msg


@pytest.mark.anyio
async def test_duplicate_pause_rejected():
    ros = FakeRos()
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="dup_pause",
        points=[SprayPoint(1.0, 0.0, 0.08, 0)],
        config=_fast_cfg(),
    )
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.sleep(0.05)
    await orch.pause_mission(ros, hold)
    await _wait_for_state(orch, PointMissionState.PAUSED_HOLD)
    ok, msg, code = await orch.pause_mission(ros, hold)
    assert not ok and code == 409
    assert "already paused" in msg


@pytest.mark.anyio
async def test_final_hold_after_completion():
    ros = FakeRos()
    ros.auto_arrive = True
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="final_hold",
        points=[SprayPoint(2.0, 3.0, 0.08, 0)],
        config=_fast_cfg(),
    )
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state == PointMissionState.COMPLETED
    assert hold.active
    assert hold.source == SetpointHoldOwner.SOURCE_HOLD
    hold_dict = hold.as_dict(ros)
    assert hold_dict["hold_north_m"] == 2.0
    assert hold_dict["hold_east_m"] == 3.0


@pytest.mark.anyio
async def test_stop_mission_drains_and_releases_hold():
    ros = FakeRos()
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    ctrl = _make_ctrl_point_mode()
    orch.load(
        mission_id="stop_hold",
        points=[SprayPoint(1.0, 0.0, 0.08, 0)],
        config=_fast_cfg(),
    )
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.sleep(0.05)
    await orch.pause_mission(ros, hold)
    await _wait_for_state(orch, PointMissionState.PAUSED_HOLD)
    await orch.stop_mission(ros, hold, reason="operator_stop")
    assert not hold.active
    assert not orch.is_active()
    assert not orch.is_paused()


@pytest.mark.anyio
async def test_pause_resume_api_endpoints(monkeypatch):
    ros = FakeRos()
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    ctrl = _make_ctrl_point_mode()
    orch.load(
        mission_id="api_pause",
        points=[SprayPoint(2.0, 0.0, 0.08, 0)],
        config=_fast_cfg(),
    )
    monkeypatch.setattr(main, "point_mission", orch)
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "ros_node", ros)
    monkeypatch.setattr(main, "hold_owner", hold)

    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.sleep(0.05)
    pause_resp = await pause_mission()
    assert pause_resp.paused is True
    await _wait_for_state(orch, PointMissionState.PAUSED_HOLD)

    resume_resp = await resume_mission()
    assert resume_resp.resumed is True

    obs_resp = await set_obstacle_status(ObstacleStatusRequest(clear=False))
    assert obs_resp.obstacle_clear is False


@pytest.mark.anyio
async def test_rest_stop_uses_unified_lifecycle(monkeypatch):
    ros = FakeRos()
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    ctrl = _make_ctrl_point_mode()

    class StopCtrl:
        spray_mode = "point"
        state = MissionState.RUNNING
        stop_called = False

        async def stop_async(self):
            StopCtrl.stop_called = True
            self.state = MissionState.IDLE
            return {"success": True, "state": self.state.value}

    ctrl = StopCtrl()

    orch.load(
        mission_id="rest_stop",
        points=[SprayPoint(1.0, 0.0, 0.08, 0)],
        config=_fast_cfg(),
    )
    monkeypatch.setattr(main, "point_mission", orch)
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "ros_node", ros)
    monkeypatch.setattr(main, "hold_owner", hold)
    monkeypatch.setattr(main, "mission_capture", None)

    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.sleep(0.05)
    await orch.pause_mission(ros, hold)
    await _wait_for_state(orch, PointMissionState.PAUSED_HOLD)

    result = await stop_mission()
    assert result.get("success") is True
    assert StopCtrl.stop_called is True
    assert not hold.active
    assert not orch.is_active()


@pytest.mark.anyio
async def test_stop_active_mission_socket_parity():
    ros = FakeRos()
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()

    class StopCtrl:
        state = MissionState.RUNNING

        async def stop_async(self):
            self.state = MissionState.IDLE
            return {"success": True, "state": "idle"}

    ctrl = StopCtrl()
    orch.load(
        mission_id="sock_stop",
        points=[SprayPoint(1.0, 0.0, 0.08, 0)],
        config=_fast_cfg(),
    )
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.sleep(0.05)
    await orch.pause_mission(ros, hold)
    await _wait_for_state(orch, PointMissionState.PAUSED_HOLD)

    result = await stop_active_mission(
        ctrl, orch, ros, hold, mission_capture=None, transport="socket"
    )
    assert result.get("success") is True
    assert not hold.active
    assert not orch.is_paused()
