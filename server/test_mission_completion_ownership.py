#!/usr/bin/env python3
"""Completion ownership and operation preemption tests."""

from __future__ import annotations

import asyncio
import os
import sys
import time
from collections import deque

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import main
from mission_ops import MissionOperation, MissionOperationCoordinator
from models import MissionState
from offboard_controller import OffboardController
from point_ingest import SprayPoint
from point_mission import PointExecutionMode, PointMissionOrchestrator, PointMissionState
from spray_config import PointSprayParams, SprayConfiguration, SprayMode
from test_point_mission import FakeOffboard, FakeRos


class DoneRppMonitor:
    def __init__(self, done: bool = True):
        self._done = done

    def reset(self):
        pass

    def is_done(self):
        return self._done

    def has_snapshot(self, fresh=False):
        return True

    def get_snapshot(self):
        class Snap:
            state_code = 3
            dist_to_goal_m = 0.0
            speed_m_s = 0.0
            xtrack_m = 0.0
            pose_age_ms = 10.0

        return Snap()

    def snapshot_age_s(self):
        return 0.01


def _fast_cfg(**overrides) -> SprayConfiguration:
    return SprayConfiguration(
        mode=SprayMode.POINT,
        point=PointSprayParams(
            default_dwell_s=0.05,
            arrival_tolerance_m=0.05,
            settle_time_s=0.0,
            leg_timeout_s=2.0,
        ),
        revision=1,
        **overrides,
    )


def _make_ctrl(spray_mode: str = "point") -> OffboardController:
    class Node:
        def get_rpp_monitor(self):
            return DoneRppMonitor()

    ctrl = OffboardController(Node(), deque())
    ctrl.load_path(
        [(0.0, 0.0), (1.0, 0.0)],
        name="t.csv",
        spray_flags=[False, False],
        mission_id="m1",
        is_staged=True,
        allow_replace_protected=True,
        spray_mode=spray_mode,
    )
    ctrl.state = MissionState.RUNNING
    return ctrl


@pytest.fixture(autouse=True)
def _reset_coordinator():
    main.operation_coordinator = MissionOperationCoordinator()
    yield


def test_active_point_rpp_done_does_not_call_parent_complete():
    assert main._should_global_rpp_complete(_make_ctrl("point"), object(), object()) is False


def test_point_waiting_for_continue_rpp_done_does_not_complete():
    async def run():
        ros = FakeRos()
        ros.auto_arrive = True
        ctrl = _make_ctrl("point")
        orch = PointMissionOrchestrator()
        orch.load(
            mission_id="m1",
            points=[SprayPoint(0, 0, 0.05, 0), SprayPoint(1, 0, 0.05, 1)],
            config=_fast_cfg(),
            execution_mode=PointExecutionMode.MANUAL,
        )
        completes = []

        class TrackingOffboard(FakeOffboard):
            async def complete_async(self):
                completes.append(True)
                return await super().complete_async()

        offboard = TrackingOffboard()
        await orch.start(ros, offboard)
        await asyncio.sleep(0.15)
        assert orch.status.state == PointMissionState.WAITING_FOR_CONTINUE
        assert ctrl.state == MissionState.RUNNING
        assert main._should_global_rpp_complete(ctrl, orch, ros) is False
        assert not completes

    asyncio.run(run())


def test_point_dwelling_rpp_done_does_not_complete():
    ctrl = _make_ctrl("point")
    assert main._should_global_rpp_complete(ctrl, object(), object()) is False


def test_continuous_rpp_done_calls_complete_once():
    ros = FakeRos()
    ros.rpp_monitor = DoneRppMonitor()
    ctrl = _make_ctrl("continuous")
    assert main._should_global_rpp_complete(ctrl, None, ros) is True


def test_dash_rpp_done_calls_complete_once():
    ros = FakeRos()
    ros.rpp_monitor = DoneRppMonitor()
    ctrl = _make_ctrl("dash")
    assert main._should_global_rpp_complete(ctrl, None, ros) is True


def test_repeated_done_ticks_do_not_emit_duplicate_completion():
    async def run():
        coordinator = MissionOperationCoordinator()
        completions = []

        class Ctrl:
            state = MissionState.RUNNING
            loaded_path_name = "x"
            uses_global_rpp_completion = True

            async def complete_async(self):
                completions.append(time.monotonic())
                return {"success": True}

        ctrl = Ctrl()
        ros = FakeRos()
        ros.rpp_monitor = DoneRppMonitor()
        for _ in range(3):
            if main._should_global_rpp_complete(ctrl, None, ros):
                token = await coordinator.begin(MissionOperation.COMPLETION, timeout_s=0.25)
                try:
                    if coordinator.is_current(token):
                        await ctrl.complete_async()
                finally:
                    await coordinator.finish(token)
        assert len(completions) == 3

    asyncio.run(run())


def test_stop_preempts_completion():
    async def run():
        coordinator = MissionOperationCoordinator()
        completion_token = await coordinator.begin(MissionOperation.COMPLETION, timeout_s=0.25)
        stop_token = await coordinator.begin(MissionOperation.STOP, timeout_s=0.5)
        assert stop_token.generation > completion_token.generation
        assert completion_token.is_preempted()
        await coordinator.finish(stop_token)
        await coordinator.finish(completion_token)

    asyncio.run(run())


def test_abort_preempts_completion():
    async def run():
        coordinator = MissionOperationCoordinator()
        completion_token = await coordinator.begin(MissionOperation.COMPLETION, timeout_s=0.25)
        abort_token = await coordinator.begin(MissionOperation.ABORT, timeout_s=0.25)
        assert abort_token.generation > completion_token.generation
        assert completion_token.is_preempted()
        await coordinator.finish(abort_token)
        await coordinator.finish(completion_token)

    asyncio.run(run())


def test_estop_preempts_completion_without_wait():
    coordinator = MissionOperationCoordinator()

    async def run():
        completion_token = await coordinator.begin(MissionOperation.COMPLETION, timeout_s=0.25)
        estop_token = coordinator.begin_estop_nowait()
        assert estop_token.generation > completion_token.generation
        assert completion_token.is_preempted()
        await coordinator.finish(completion_token)
        await coordinator.finish(estop_token)

    asyncio.run(run())