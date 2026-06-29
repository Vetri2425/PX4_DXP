#!/usr/bin/env python3
"""Point terminal cleanup integration tests."""

from __future__ import annotations

import asyncio
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import main
from mission_ops import MissionOperation, MissionOperationCoordinator
from mission_services import build_service_context, continue_point_service, stop_mission_service
from point_events import reset_point_event_journal_for_tests
from point_mission import PointMissionOrchestrator, PointMissionState
from test_point_mission import FakeOffboard, FakeRos
from point_ingest import SprayPoint
from spray_config import PointSprayParams, SprayConfiguration, SprayMode


@pytest.fixture(autouse=True)
def _coordinator():
    main.operation_coordinator = MissionOperationCoordinator()
    reset_point_event_journal_for_tests()
    yield


def _orch():
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="m1",
        points=[SprayPoint(0, 0, 0.05, 0), SprayPoint(1, 0, 0.05, 1)],
        config=SprayConfiguration(
            mode=SprayMode.POINT,
            point=PointSprayParams(default_dwell_s=0.05, settle_time_s=0.0, leg_timeout_s=2.0),
            revision=1,
        ),
    )
    return orch


@pytest.mark.anyio
async def test_point_terminal_cleanup_stop_cancels_gates_and_emits_once():
    ros = FakeRos()
    orch = _orch()
    await orch.start(ros, FakeOffboard())
    token = await main.operation_coordinator.begin(MissionOperation.STOP, timeout_s=0.5)
    await orch.terminal_cleanup(
        ros, None,
        reason="operator_stop",
        terminal_state=PointMissionState.ABORTING,
        operation_token=token,
    )
    await main.operation_coordinator.finish(token)
    assert orch.status.state == PointMissionState.ABORTING
    from point_events import get_point_event_journal
    terminal = [
        e for e in get_point_event_journal().history()["events"] if e.terminal
    ]
    assert len(terminal) == 1
    assert terminal[0].event_type == "point_aborted"


@pytest.mark.anyio
async def test_point_terminal_cleanup_abort_idempotent():
    ros = FakeRos()
    orch = _orch()
    await orch.start(ros, FakeOffboard())
    token = await main.operation_coordinator.begin(MissionOperation.ABORT, timeout_s=0.25)
    first = await orch.terminal_cleanup(
        ros, None,
        reason="operator_abort",
        terminal_state=PointMissionState.ABORTING,
        operation_token=token,
    )
    second = await orch.terminal_cleanup(
        ros, None,
        reason="operator_stop",
        terminal_state=PointMissionState.ABORTING,
        operation_token=token,
    )
    await main.operation_coordinator.finish(token)
    assert first.idempotent is False
    assert second.idempotent is True


@pytest.mark.anyio
async def test_point_continue_rejected_after_stop_cleanup():
    ros = FakeRos()
    orch = _orch()
    from point_mission import PointExecutionMode
    orch.load(
        mission_id="m1",
        points=[SprayPoint(0, 0, 0.05, 0)],
        config=SprayConfiguration(mode=SprayMode.POINT, revision=1),
        execution_mode=PointExecutionMode.MANUAL,
    )
    main.ros_node = ros
    main.offboard_ctrl = type("C", (), {"spray_mode": "point", "state": "running"})()
    main.point_mission = orch
    main.hold_owner = None
    main.path_mgr = None
    await orch.start(ros, FakeOffboard())
    token = await main.operation_coordinator.begin(MissionOperation.STOP, timeout_s=0.5)
    await orch.terminal_cleanup(
        ros, None,
        reason="operator_stop",
        terminal_state=PointMissionState.ABORTING,
        operation_token=token,
    )
    await main.operation_coordinator.finish(token)
    ok, msg, code = await orch.continue_point(ros)
    assert not ok and code == 409
    assert "terminal" in msg


@pytest.mark.anyio
async def test_point_skip_rejected_after_abort_cleanup():
    ros = FakeRos()
    orch = _orch()
    await orch.start(ros, FakeOffboard())
    token = await main.operation_coordinator.begin(MissionOperation.ABORT, timeout_s=0.25)
    await orch.terminal_cleanup(
        ros, None,
        reason="operator_abort",
        terminal_state=PointMissionState.ABORTING,
        operation_token=token,
    )
    await main.operation_coordinator.finish(token)
    token2 = await main.operation_coordinator.begin(MissionOperation.SKIP, timeout_s=0.5)
    ok, msg, code = await orch.skip_point(
        ros, None, point_index=0, expected_generation=None,
        reason="t", operation_token=token2,
    )
    await main.operation_coordinator.finish(token2)
    assert not ok and code == 409


@pytest.mark.anyio
async def test_point_terminal_cleanup_estop_does_not_wait_for_lower_operation():
    coordinator = main.operation_coordinator
    stop_token = await coordinator.begin(MissionOperation.STOP, timeout_s=0.5)

    async def slow_stop():
        await asyncio.sleep(2.0)
        await coordinator.finish(stop_token)

    stop_task = asyncio.create_task(slow_stop())
    await asyncio.sleep(0.01)
    estop_token = coordinator.begin_estop_nowait()
    assert estop_token.generation > stop_token.generation
    assert stop_token.is_preempted()
    await coordinator.finish(estop_token)
    stop_task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await stop_task


@pytest.mark.anyio
async def test_point_resume_rejected_after_estop_cleanup():
    ros = FakeRos()
    orch = _orch()
    hold = __import__("setpoint_hold").SetpointHoldOwner()
    await orch.start(ros, FakeOffboard(), hold)
    await orch.pause_mission(ros, hold)
    await asyncio.sleep(0.05)
    estop_token = main.operation_coordinator.begin_estop_nowait()
    await orch.terminal_cleanup(
        ros,
        hold,
        reason="emergency_stop",
        terminal_state=PointMissionState.ABORTING,
        operation_token=estop_token,
        require_spray_confirm=False,
    )
    ok, msg, code = await orch.resume_mission(ros, hold)
    assert not ok and code == 409
    assert "terminal" in msg


@pytest.mark.anyio
async def test_point_terminal_event_emitted_once_after_stop_abort_estop():
    from mission_services import abort_mission_service
    from point_events import get_point_event_journal

    ros = FakeRos()
    orch = _orch()
    hold = __import__("setpoint_hold").SetpointHoldOwner()

    class StopCtrl(FakeOffboard):
        spray_mode = "point"

        async def stop_async(self):
            self.state = __import__("models").MissionState.IDLE
            return {"success": True, "state": "idle"}

        async def abort_async(self):
            self.state = __import__("models").MissionState.ABORTED
            return {
                "success": True,
                "message": "aborted",
                "errors": [],
                "spray_off_result": {"success": True},
            }

    ctrl = StopCtrl()
    main.ros_node = ros
    main.offboard_ctrl = ctrl
    main.point_mission = orch
    main.hold_owner = hold
    main.path_mgr = None
    await orch.start(ros, ctrl, hold)
    ctx = build_service_context(
        offboard_ctrl=ctrl,
        point_mission=orch,
        ros_node=ros,
        hold_owner=hold,
        path_mgr=None,
        mission_capture=None,
        transport="rest",
        operation_coordinator=main.operation_coordinator,
    )
    await stop_mission_service(ctx)
    terminal = [
        e for e in get_point_event_journal().history()["events"] if e.terminal
    ]
    assert len(terminal) == 1
    assert terminal[0].event_type == "point_aborted"


    reset_point_event_journal_for_tests()
    orch2 = _orch()
    await orch2.start(ros, ctrl, hold)
    main.point_mission = orch2
    await abort_mission_service(ctx)
    terminal = [
        e for e in get_point_event_journal().history()["events"] if e.terminal
    ]
    assert len(terminal) == 1
    assert terminal[0].event_type == "point_aborted"

    reset_point_event_journal_for_tests()
    orch3 = _orch()
    await orch3.start(ros, ctrl, hold)
    estop_token = main.operation_coordinator.begin_estop_nowait()
    await orch3.terminal_cleanup(
        ros,
        hold,
        reason="emergency_stop",
        terminal_state=PointMissionState.ABORTING,
        operation_token=estop_token,
        require_spray_confirm=False,
    )
    terminal = [
        e for e in get_point_event_journal().history()["events"] if e.terminal
    ]
    assert len(terminal) == 1
    assert terminal[0].event_type == "point_aborted"


@pytest.mark.anyio
async def test_terminal_cleanup_from_run_task_preserves_completed_task_and_event_once():
    from point_events import get_point_event_journal

    ros = FakeRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="self-clean",
        points=[SprayPoint(1, 0, 0.0, 0, mark=False)],
        config=SprayConfiguration(
            mode=SprayMode.POINT,
            point=PointSprayParams(default_dwell_s=0.05, settle_time_s=0.0, leg_timeout_s=2.0),
            revision=1,
        ),
    )
    await orch.start(ros, FakeOffboard())
    task = orch._task
    await asyncio.wait_for(task, timeout=3.0)

    assert orch.status.state == PointMissionState.COMPLETED
    assert orch._task is task
    assert task.done()
    assert task.exception() is None
    terminal = [
        e for e in get_point_event_journal().history()["events"] if e.terminal
    ]
    assert len(terminal) == 1
    assert terminal[0].event_type == "point_completed"
