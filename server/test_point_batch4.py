"""Batch 4 tests: GPS_SURVEYED safety gate, runtime monitoring, hold integration."""

from __future__ import annotations

import asyncio
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from mission_placement import GPS_SURVEYED, PlacementError
from point_ingest import SprayPoint
from point_mission import PointExecutionMode, PointMissionOrchestrator, PointMissionState
from setpoint_hold import SetpointHoldOwner
from spray_config import GpsSurveyedSafetyParams, PointSprayParams, SprayConfiguration, SprayMode
from test_mission_placement import ANCHOR, healthy_state
from test_point_mission import FakeOffboard, FakeRos


def _gps_cfg(**gps_overrides) -> SprayConfiguration:
    point = PointSprayParams(
        default_dwell_s=0.08,
        arrival_tolerance_m=0.05,
        settle_time_s=0.0,
        leg_timeout_s=3.0,
        settle_speed_mps=0.05,
        settle_yaw_rate_rad_s=0.05,
    )
    gps = GpsSurveyedSafetyParams(**gps_overrides)
    return SprayConfiguration(mode=SprayMode.POINT, point=point, gps_safety=gps, revision=1)


def _load_surveyed(orch: PointMissionOrchestrator, cfg: SprayConfiguration) -> None:
    orch.load(
        mission_id="gps4",
        points=[SprayPoint(0.0, -0.035, 0.08, 0), SprayPoint(1.0, -0.035, 0.08, 1)],
        config=cfg,
    )
    orch._source_frame = GPS_SURVEYED
    orch._origin_gps = ANCHOR


class SurveyedRos(FakeRos):
    """Fake ROS with GPS_SURVEYED telemetry defaults."""

    def __init__(self, **state_overrides):
        super().__init__()
        self.state.update(healthy_state())
        self.state.update(state_overrides)


class DegradingSurveyedRos(SurveyedRos):
    def __init__(self, degrade_after: int = 3, **bad):
        super().__init__()
        self._polls = 0
        self._degrade_after = degrade_after
        self._bad = bad

    def get_state(self):
        state = super().get_state()
        self._polls += 1
        if self._polls >= self._degrade_after:
            state.update(self._bad)
        return state


async def _wait_for_state(orch, state: PointMissionState, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if orch.status.state == state:
            return
        await asyncio.sleep(0.02)
    raise AssertionError(f"timed out waiting for {state.value}, got {orch.status.state.value}")


@pytest.mark.anyio
async def test_start_gate_rejects_bad_fix():
    orch = PointMissionOrchestrator()
    _load_surveyed(orch, _gps_cfg())
    with pytest.raises(PlacementError, match="below required"):
        orch.prepare(SurveyedRos(gps_fix=5).get_state())


@pytest.mark.anyio
async def test_start_gate_rejects_stale_global():
    orch = PointMissionOrchestrator()
    _load_surveyed(orch, _gps_cfg())
    with pytest.raises(PlacementError, match="global position is stale"):
        orch.prepare(SurveyedRos(global_position_age_ms=800.0).get_state())


@pytest.mark.anyio
async def test_start_gate_rejects_stale_local():
    orch = PointMissionOrchestrator()
    _load_surveyed(orch, _gps_cfg())
    with pytest.raises(PlacementError, match="local pose is stale"):
        orch.prepare(SurveyedRos(local_pose_age_ms=800.0).get_state())


@pytest.mark.anyio
async def test_start_gate_rejects_unsafe_skew():
    orch = PointMissionOrchestrator()
    _load_surveyed(orch, _gps_cfg())
    with pytest.raises(PlacementError, match="not sufficiently aligned"):
        orch.prepare(SurveyedRos(pose_global_skew_ms=250.0).get_state())


@pytest.mark.anyio
async def test_start_gate_rejects_missing_anchor():
    orch = PointMissionOrchestrator()
    _load_surveyed(orch, _gps_cfg())
    orch._origin_gps = None
    with pytest.raises(PlacementError, match="anchor"):
        orch.prepare(SurveyedRos().get_state())


@pytest.mark.anyio
async def test_runtime_gps_fault_pauses_navigation():
    ros = DegradingSurveyedRos(degrade_after=3, gps_fix=5)
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    _load_surveyed(orch, _gps_cfg())
    orch.prepare(ros.get_state())
    await orch.start(ros, FakeOffboard(), hold)
    await _wait_for_state(orch, PointMissionState.PAUSED_GPS_SAFETY)
    assert hold.active
    assert orch.status.gps_fault_count >= 1
    assert orch.status.dwell_cancelled is False or orch.status.pre_pause_state == "navigating"


@pytest.mark.anyio
async def test_runtime_gps_fault_during_dwell_cancels_spray():
    ros = SurveyedRos()
    ros.auto_arrive = True
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    cfg = _gps_cfg()
    cfg = SprayConfiguration(
        mode=cfg.mode,
        point=PointSprayParams(
            default_dwell_s=1.0,
            arrival_tolerance_m=0.05,
            settle_time_s=0.0,
            leg_timeout_s=3.0,
            settle_speed_mps=0.05,
            settle_yaw_rate_rad_s=0.05,
        ),
        gps_safety=cfg.gps_safety,
        revision=1,
    )
    _load_surveyed(orch, cfg)
    orch.prepare(ros.get_state())
    await orch.start(ros, FakeOffboard(), hold)
    await _wait_for_state(orch, PointMissionState.DWELLING)
    deadline = time.monotonic() + 1.0
    while ros.live_dwell is None and time.monotonic() < deadline:
        await asyncio.sleep(0.01)
    assert ros.live_dwell is not None
    ros.state["gps_fix"] = 5
    await _wait_for_state(orch, PointMissionState.PAUSED_GPS_SAFETY)
    assert ros.live_dwell is None
    assert orch.status.dwell_cancelled is True
    assert hold.active


@pytest.mark.anyio
async def test_manual_resume_after_recovery_debounce():
    ros = DegradingSurveyedRos(degrade_after=3, gps_fix=5)
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    _load_surveyed(orch, _gps_cfg(recovery_stable_s=0.05))
    orch.prepare(healthy_state())
    await orch.start(ros, FakeOffboard(), hold)
    await _wait_for_state(orch, PointMissionState.PAUSED_GPS_SAFETY)
    ros._polls = 0
    ros._bad = {}
    ros.state.update(healthy_state())
    await asyncio.sleep(0.12)
    ok, msg, code = await orch.resume_mission(ros, hold)
    assert ok and code == 200, msg


@pytest.mark.anyio
async def test_resume_rejected_before_recovery_ready():
    ros = DegradingSurveyedRos(degrade_after=2, gps_fix=5)
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    _load_surveyed(orch, _gps_cfg(recovery_stable_s=5.0))
    orch.prepare(healthy_state())
    await orch.start(ros, FakeOffboard(), hold)
    await _wait_for_state(orch, PointMissionState.PAUSED_GPS_SAFETY)
    ros._degrade_after = 10_000
    ros._polls = 0
    ros.state.update(healthy_state())
    orch._gps_recovery_since = time.monotonic()
    ok, msg, code = await orch.resume_mission(ros, hold)
    assert not ok and code == 409
    assert "recovery" in msg.lower()


@pytest.mark.anyio
async def test_fail_policy_enters_failed_gps_safety():
    ros = DegradingSurveyedRos(degrade_after=2, gps_fix=5)
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    _load_surveyed(orch, _gps_cfg(runtime_policy="fail"))
    orch.prepare(healthy_state())
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state == PointMissionState.FAILED_GPS_SAFETY
    assert hold.active


@pytest.mark.anyio
async def test_runtime_gps_fault_during_manual_wait():
    ros = SurveyedRos()
    ros.auto_arrive = True
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    _load_surveyed(orch, _gps_cfg())
    orch.prepare(healthy_state())
    orch._execution_mode = PointExecutionMode.MANUAL
    orch.status.point_execution_mode = "manual"
    await orch.start(ros, FakeOffboard(), hold)
    await _wait_for_state(orch, PointMissionState.WAITING_FOR_CONTINUE)
    ros.state["gps_fix"] = 5
    await _wait_for_state(orch, PointMissionState.PAUSED_GPS_SAFETY)
    assert hold.active


@pytest.mark.anyio
async def test_manual_continue_rejected_immediately_after_gps_fault(monkeypatch):
    import main

    ros = SurveyedRos()
    ros.auto_arrive = True
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    _load_surveyed(orch, _gps_cfg())
    orch.prepare(healthy_state())
    orch._execution_mode = PointExecutionMode.MANUAL
    orch.status.point_execution_mode = "manual"
    await orch.start(ros, FakeOffboard(), hold)
    await _wait_for_state(orch, PointMissionState.WAITING_FOR_CONTINUE)
    monkeypatch.setattr(main, "ros_node", ros)
    ros.state["gps_fix"] = 5
    ok, msg, code = await orch.continue_point()
    assert not ok and code == 409
    assert "gps safety" in msg.lower()


@pytest.mark.anyio
async def test_local_ned_unaffected_by_gps_fault():
    ros = DegradingSurveyedRos(degrade_after=2, gps_fix=5)
    ros.auto_arrive = True
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="local",
        points=[SprayPoint(1.0, 0.0, 0.08, 0)],
        config=_gps_cfg(),
    )
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state == PointMissionState.COMPLETED
    assert orch.status.gps_safety_state == "not_applicable"
