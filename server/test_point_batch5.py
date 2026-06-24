"""Batch 5 tests: trajectory policy, hold drift, terminal completion gate."""

from __future__ import annotations

import asyncio
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from point_ingest import SprayPoint
from point_mission import PointMissionOrchestrator, PointMissionRun, PointMissionState
from setpoint_hold import SetpointHoldOwner
from spray_config import PointSprayParams, SprayConfiguration, SprayMode
from test_point_mission import FakeOffboard, FakeRos


def _cfg(**point_overrides) -> SprayConfiguration:
    base = {
        "default_dwell_s": 0.06,
        "arrival_tolerance_m": 0.05,
        "settle_time_s": 0.0,
        "leg_timeout_s": 2.0,
        "settle_speed_mps": 0.05,
        "settle_yaw_rate_rad_s": 0.05,
    }
    base.update(point_overrides)
    point = PointSprayParams(**base)
    return SprayConfiguration(mode=SprayMode.POINT, point=point, revision=1)


async def _wait_for_state(orch, state: PointMissionState, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if orch.status.state == state:
            return
        await asyncio.sleep(0.02)
    raise AssertionError(f"timed out waiting for {state.value}, got {orch.status.state.value}")


def test_two_point_leg_publishes_two_endpoints():
    async def run():
        ros = FakeRos()
        orch = PointMissionOrchestrator()
        cfg = _cfg()
        orch.load(
            mission_id="traj2",
            points=[SprayPoint(5.0, 0.0, 1.0, 0)],
            config=cfg,
        )
        token = PointMissionRun(orch.status.generation, "traj2", asyncio.Event())
        orch._run_token = token
        await orch._publish_fresh_leg(
            token,
            ros,
            SprayPoint(5.0, 0.0, 1.0, 0),
            cfg.point,
        )
        pts, flags, runtime = ros.paths[-1]
        assert len(pts) == 2
        assert pts[0] == (0.0, 0.0)
        assert pts[-1] == (5.0, 0.0)
        assert runtime is True
        assert orch.status.point_leg_published_count == 2
        assert orch.status.point_leg_conditioned_count == 2
        assert orch.status.active_trajectory_mode == "segment"

    asyncio.run(run())


def test_densified_leg_spacing_and_diagnostics():
    async def run():
        ros = FakeRos()
        orch = PointMissionOrchestrator()
        cfg = _cfg(leg_trajectory_mode="densified", leg_spacing_m=0.08)
        orch.load(mission_id="trajd", points=[SprayPoint(5.0, 0.0, 1.0, 0)], config=cfg)
        token = PointMissionRun(orch.status.generation, "trajd", asyncio.Event())
        orch._run_token = token
        await orch._publish_fresh_leg(
            token,
            ros,
            SprayPoint(5.0, 0.0, 1.0, 0),
            cfg.point,
        )
        pts, _, _ = ros.paths[-1]
        assert len(pts) > 2
        assert pts[0] == (0.0, 0.0)
        assert pts[-1] == (5.0, 0.0)
        assert orch.status.point_leg_trajectory_mode == "densified"
        assert orch.status.point_leg_published_count == len(pts)
        assert orch.status.point_leg_conditioned_count > 2
        assert orch.status.active_trajectory_mode == "smooth"
        assert abs(orch.status.point_leg_length_m - 5.0) < 1e-6

    asyncio.run(run())


def test_fresh_leg_per_point_uses_live_pose():
    async def run():
        ros = FakeRos()
        ros.auto_arrive = True
        hold = SetpointHoldOwner()
        orch = PointMissionOrchestrator()
        cfg = _cfg(default_dwell_s=0.04)
        orch.load(
            mission_id="fresh",
            points=[
                SprayPoint(1.0, 0.0, 0.04, 0),
                SprayPoint(2.0, 0.0, 0.04, 1),
            ],
            config=cfg,
        )
        await orch.start(ros, FakeOffboard(), hold)
        await asyncio.wait_for(orch._task, timeout=3.0)
        nav_legs = [p for p in ros.paths if len(p[0]) == 2 and p[2]]
        assert len(nav_legs) == 2
        assert nav_legs[0][0][0] == (0.0, 0.0)
        assert nav_legs[1][0][0] == (1.0, 0.0)
    asyncio.run(run())


class DriftingDwellRos(FakeRos):
    """Inject hold drift once dwell is active."""

    def __init__(self):
        super().__init__()
        self._inject_drift = False

    def get_spray_runtime_status(self):
        status = super().get_spray_runtime_status()
        if status.get("active_dwell"):
            self._inject_drift = True
        return status

    def get_state(self):
        state = super().get_state()
        if self._inject_drift:
            state["pos_n"] = 0.2
        return state


@pytest.mark.anyio
async def test_dwell_drift_cancels_spray_and_fails():
    ros = DriftingDwellRos()
    ros.auto_arrive = True
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    cfg = _cfg(
        default_dwell_s=1.0,
        hold_drift_tolerance_m=0.05,
        hold_drift_policy="fail",
    )
    orch.load(mission_id="drift", points=[SprayPoint(0.0, 0.0, 1.0, 0)], config=cfg)
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state == PointMissionState.FAILED
    assert orch.status.dwell_cancelled is True
    assert "hold drift" in orch.status.last_failure_reason.lower()
    assert ros.live_dwell is None


@pytest.mark.anyio
async def test_final_hold_before_completed():
    ros = FakeRos()
    ros.auto_arrive = True
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    cfg = _cfg(default_dwell_s=0.04)
    orch.load(mission_id="final", points=[SprayPoint(3.0, 4.0, 0.04, 0)], config=cfg)
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state == PointMissionState.COMPLETED
    assert hold.active
    assert orch.status.hold_active
    assert orch.status.hold_north_m == 3.0
    assert orch.status.hold_east_m == 4.0
    hold_paths = [p for p in ros.paths if len(p[0]) == 1]
    assert hold_paths, "expected single-point hold path publishes"
    assert hold_paths[-1][0][0] == (3.0, 4.0)


@pytest.mark.anyio
async def test_manual_wait_no_stale_two_point_reactivation():
    ros = FakeRos()
    ros.auto_arrive = True
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    from point_mission import PointExecutionMode

    cfg = _cfg(default_dwell_s=0.04)
    orch.load(
        mission_id="manual",
        points=[SprayPoint(1.0, 0.0, 0.04, 0), SprayPoint(2.0, 0.0, 0.04, 1)],
        config=cfg,
        execution_mode=PointExecutionMode.MANUAL,
    )
    await orch.start(ros, FakeOffboard(), hold)
    await _wait_for_state(orch, PointMissionState.WAITING_FOR_CONTINUE)
    nav_paths = [p for p in ros.paths if len(p[0]) == 2]
    hold_paths = [p for p in ros.paths if len(p[0]) == 1]
    assert len(nav_paths) == 1
    assert hold_paths
    assert hold_paths[-1][0][0] == (1.0, 0.0)
    ok, _, code = await orch.continue_point()
    assert ok and code == 200
    await asyncio.wait_for(orch._task, timeout=3.0)
    nav_legs = [p for p in ros.paths if len(p[0]) == 2 and p[2]]
    assert len(nav_legs) == 2
    assert nav_legs[1][0][0] == (1.0, 0.0)
    assert orch.status.state == PointMissionState.COMPLETED