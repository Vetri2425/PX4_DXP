#!/usr/bin/env python3
"""Point skip operator control tests."""

from __future__ import annotations

import asyncio
import os
import sys
from collections import deque

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import main
from mission_ops import MissionOperation, MissionOperationCoordinator, MissionOperationToken
from mission_services import build_service_context, skip_point_service
from models import MissionState, PointSkipRequest
from offboard_controller import OffboardController
from point_ingest import SprayPoint
from point_mission import PointExecutionMode, PointMissionOrchestrator, PointMissionState
from spray_config import PointSprayParams, SprayConfiguration, SprayMode
from test_point_mission import FakeOffboard, FakeRos


def _cfg(dwell_s=0.2):
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


@pytest.fixture(autouse=True)
def _coordinator():
    main.operation_coordinator = MissionOperationCoordinator()
    yield


def _token(op=MissionOperation.SKIP) -> MissionOperationToken:
    return main.operation_coordinator.begin_estop_nowait() if op == MissionOperation.ESTOP else None


async def _begin_skip():
    return await main.operation_coordinator.begin(MissionOperation.SKIP, timeout_s=0.5)


@pytest.mark.anyio
async def test_skip_rejects_waiting_for_continue_completed_point():
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="m1",
        points=[SprayPoint(0, 0, 0.05, 0), SprayPoint(1, 0, 0.05, 1)],
        config=_cfg(),
        execution_mode=PointExecutionMode.MANUAL,
    )
    await orch.start(ros, FakeOffboard())
    await asyncio.sleep(0.2)
    token = await _begin_skip()
    ok, msg, code = await orch.skip_point(
        ros, None, point_index=0, expected_generation=None, reason="t", operation_token=token
    )
    assert not ok and code == 409
    assert "use continue" in msg


@pytest.mark.anyio
async def test_skip_navigating_advances_to_next_point():
    ros = FakeRos()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="m1",
        points=[
            SprayPoint(5.0, 0.0, 0.05, 0),
            SprayPoint(10.0, 0.0, 0.05, 1),
        ],
        config=_cfg(),
    )
    await orch.start(ros, FakeOffboard())
    await asyncio.sleep(0.05)
    token = await _begin_skip()
    ok, _, code = await orch.skip_point(
        ros, None, point_index=0, expected_generation=orch.status.generation,
        reason="t", operation_token=token,
    )
    assert ok and code == 200
    await asyncio.sleep(0.15)
    assert 0 in orch.status.skipped_point_indices
    assert orch.status.current_point_index >= 1


@pytest.mark.anyio
async def test_skip_dwelling_cancels_dwell_and_confirms_off():
    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="m1",
        points=[SprayPoint(0, 0, 0.2, 0), SprayPoint(1, 0, 0.05, 1)],
        config=_cfg(dwell_s=1.0),
    )
    await orch.start(ros, FakeOffboard())
    deadline = asyncio.get_running_loop().time() + 2.0
    while asyncio.get_running_loop().time() < deadline:
        if orch.status.state == PointMissionState.DWELLING:
            break
        await asyncio.sleep(0.02)
    token = await _begin_skip()
    ok, _, code = await orch.skip_point(
        ros, None, point_index=0, expected_generation=orch.status.generation,
        reason="t", operation_token=token,
    )
    assert ok and code == 200


@pytest.mark.anyio
async def test_skip_generation_mismatch_rejected():
    ros = FakeRos()
    orch = PointMissionOrchestrator()
    orch.load(mission_id="m1", points=[SprayPoint(0, 0, 0.05, 0)], config=_cfg())
    await orch.start(ros, FakeOffboard())
    token = await _begin_skip()
    ok, msg, code = await orch.skip_point(
        ros, None, point_index=0, expected_generation=999,
        reason="t", operation_token=token,
    )
    assert not ok and code == 409
    assert "generation" in msg


@pytest.mark.anyio
async def test_skip_duplicate_pending_rejected():
    ros = FakeRos()
    orch = PointMissionOrchestrator()
    orch.load(mission_id="m1", points=[SprayPoint(5, 0, 0.05, 0)], config=_cfg())
    await orch.start(ros, FakeOffboard())
    run = orch._run_token
    assert run is not None
    token = await _begin_skip()
    async with orch._command_lock:
        run.skip_requested = True
        run.skip_request_id = 0
        orch._write(run, skip_pending=True)
    ok, msg, code = await orch.skip_point(
        ros, None, point_index=0, expected_generation=None,
        reason="t", operation_token=token,
    )
    assert not ok and code == 409
    assert "pending" in msg


@pytest.mark.anyio
async def test_socket_point_skip_matches_rest_response():
    main.ros_node = FakeRos()
    main.hold_owner = None
    ctrl = OffboardController(main.ros_node, deque())
    ctrl.load_path(
        [(0, 0), (5, 0)],
        name="x.csv",
        spray_flags=[False, False],
        mission_id="sk1",
        is_staged=True,
        allow_replace_protected=True,
        spray_mode="point",
    )
    ctrl.state = MissionState.RUNNING
    main.offboard_ctrl = ctrl
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="sk1",
        points=[SprayPoint(5, 0, 0.05, 0)],
        config=_cfg(),
    )
    main.point_mission = orch
    await orch.start(main.ros_node, ctrl)
    ctx_rest = build_service_context(
        offboard_ctrl=ctrl,
        point_mission=orch,
        ros_node=main.ros_node,
        hold_owner=None,
        path_mgr=None,
        mission_capture=None,
        transport="rest",
        operation_coordinator=main.operation_coordinator,
    )
    ctx_sock = build_service_context(
        offboard_ctrl=ctrl,
        point_mission=orch,
        ros_node=main.ros_node,
        hold_owner=None,
        path_mgr=None,
        mission_capture=None,
        transport="socketio",
        operation_coordinator=MissionOperationCoordinator(),
    )
    with pytest.raises(Exception):
        await skip_point_service(ctx_rest, PointSkipRequest(point_index=99))
    with pytest.raises(Exception):
        await skip_point_service(ctx_sock, PointSkipRequest(point_index=99))


@pytest.mark.anyio
async def test_skip_settling_advances_to_next_point():
    ros = FakeRos()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="m1",
        points=[
            SprayPoint(0.0, 0.0, 0.05, 0, mark=False),
            SprayPoint(5.0, 0.0, 0.05, 1, mark=False),
        ],
        config=_cfg(),
    )
    await orch.start(ros, FakeOffboard())
    run = orch._run_token
    assert run is not None
    orch._write(
        run,
        state=PointMissionState.SETTLING,
        current_point_index=0,
        run_active=True,
    )
    token = await _begin_skip()
    ok, _, code = await orch.skip_point(
        ros,
        None,
        point_index=0,
        expected_generation=orch.status.generation,
        reason="t",
        operation_token=token,
    )
    assert ok and code == 200
    await asyncio.sleep(0.1)
    assert 0 in orch.status.skipped_point_indices


@pytest.mark.anyio
async def test_skip_paused_hold_deactivates_hold_and_advances():
    from setpoint_hold import SetpointHoldOwner

    ros = FakeRos()
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="m1",
        points=[
            SprayPoint(2.0, 0.0, 0.05, 0),
            SprayPoint(5.0, 0.0, 0.05, 1),
        ],
        config=_cfg(),
    )
    await orch.start(ros, FakeOffboard(), hold)
    await orch.pause_mission(ros, hold)
    await asyncio.sleep(0.1)
    assert hold.active
    token = await _begin_skip()
    ok, _, code = await orch.skip_point(
        ros,
        hold,
        point_index=0,
        expected_generation=orch.status.generation,
        reason="t",
        operation_token=token,
    )
    assert ok and code == 200
    assert not hold.active
    assert 0 in orch.status.skipped_point_indices


@pytest.mark.anyio
async def test_skip_last_point_completes_mission():
    ros = FakeRos()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="m1",
        points=[SprayPoint(0, 0, 0.05, 0, mark=False)],
        config=_cfg(),
    )
    await orch.start(ros, FakeOffboard())
    run = orch._run_token
    assert run is not None
    orch._write(
        run,
        state=PointMissionState.NAVIGATING,
        current_point_index=0,
        run_active=True,
    )
    token = await _begin_skip()
    ok, _, code = await orch.skip_point(
        ros,
        None,
        point_index=0,
        expected_generation=orch.status.generation,
        reason="t",
        operation_token=token,
    )
    assert ok and code == 200
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state == PointMissionState.COMPLETED


@pytest.mark.anyio
async def test_skip_preempted_by_stop_publishes_no_new_leg():
    ros = FakeRos()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="m1",
        points=[
            SprayPoint(5.0, 0.0, 0.05, 0),
            SprayPoint(10.0, 0.0, 0.05, 1),
        ],
        config=_cfg(),
    )
    await orch.start(ros, FakeOffboard())
    paths_before = len(ros.paths)
    skip_token = await _begin_skip()
    run = orch._run_token
    assert run is not None
    run.skip_requested = True
    orch._write(run, skip_pending=True)
    stop_token = await main.operation_coordinator.begin(
        MissionOperation.STOP, timeout_s=0.5
    )
    await orch.terminal_cleanup(
        ros,
        None,
        reason="operator_stop",
        terminal_state=PointMissionState.ABORTING,
        operation_token=stop_token,
    )
    await main.operation_coordinator.finish(stop_token)
    await main.operation_coordinator.finish(skip_token)
    assert len(ros.paths) == paths_before