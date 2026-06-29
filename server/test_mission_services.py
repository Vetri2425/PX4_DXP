#!/usr/bin/env python3
"""Shared mission service tests."""

from __future__ import annotations

import asyncio
import os
import sys
from collections import deque

import pytest
from fastapi import HTTPException

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import main
import sockets.events as socket_events
from control_arbiter import ControlArbiterError, reset_control_arbiter_for_tests
from mission_ops import MissionOperationCoordinator
from mission_services import (
    MissionServiceError,
    build_service_context,
    continue_point_service,
    pause_point_service,
    resume_point_service,
)
from models import MissionState
from offboard_controller import OffboardController
from point_ingest import SprayPoint
from point_mission import PointExecutionMode, PointMissionOrchestrator, PointMissionState
from routes.mission import pause_mission, resume_mission
from spray_config import PointSprayParams, SprayConfiguration, SprayMode
from test_point_mission import FakeRos


class FakeSio:
    def __init__(self):
        self.handlers = {}
        self.emitted = []

    def event(self, fn):
        self.handlers[fn.__name__] = fn
        return fn

    def on(self, name):
        def decorator(fn):
            self.handlers[name] = fn
            return fn

        return decorator

    async def emit(self, event, data, to=None):
        self.emitted.append((event, data, to))


def _ctx():
    return build_service_context(
        offboard_ctrl=main.offboard_ctrl,
        point_mission=main.point_mission,
        ros_node=main.ros_node,
        hold_owner=main.hold_owner,
        path_mgr=main.path_mgr,
        mission_capture=None,
        transport="rest",
        operation_coordinator=main.operation_coordinator,
    )


@pytest.fixture
def setup_point_mission(monkeypatch):
    reset_control_arbiter_for_tests()
    main.operation_coordinator = MissionOperationCoordinator()
    ros = FakeRos()
    ros.auto_arrive = True
    main.ros_node = ros
    main.hold_owner = __import__("setpoint_hold").SetpointHoldOwner()
    main.path_mgr = None
    ctrl = OffboardController(ros, deque())
    ctrl.load_path(
        [(0.0, 0.0), (1.0, 0.0)],
        name="svc.csv",
        spray_flags=[False, False],
        mission_id="svc1",
        is_staged=True,
        allow_replace_protected=True,
        spray_mode="point",
    )
    ctrl.state = MissionState.RUNNING
    main.offboard_ctrl = ctrl
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="svc1",
        points=[SprayPoint(0, 0, 0.05, 0), SprayPoint(1, 0, 0.05, 1)],
        config=SprayConfiguration(
            mode=SprayMode.POINT,
            point=PointSprayParams(default_dwell_s=0.05, settle_time_s=0.0, leg_timeout_s=2.0),
            revision=1,
        ),
        execution_mode=PointExecutionMode.AUTO,
    )
    main.point_mission = orch
    yield


@pytest.mark.anyio
async def test_rest_pause_uses_shared_service(setup_point_mission):
    await main.point_mission.start(main.ros_node, main.offboard_ctrl, main.hold_owner)
    await asyncio.sleep(0.05)
    resp = await pause_point_service(_ctx())
    assert resp.paused is True


@pytest.mark.anyio
async def test_socket_pause_uses_same_service_response(setup_point_mission):
    await main.point_mission.start(main.ros_node, main.offboard_ctrl, main.hold_owner)
    await asyncio.sleep(0.05)
    ctx = build_service_context(
        offboard_ctrl=main.offboard_ctrl,
        point_mission=main.point_mission,
        ros_node=main.ros_node,
        hold_owner=main.hold_owner,
        path_mgr=None,
        mission_capture=None,
        transport="socketio",
        operation_coordinator=main.operation_coordinator,
    )
    resp = await pause_point_service(ctx)
    assert resp.paused is True


@pytest.mark.anyio
async def test_rest_socket_resume_error_mapping_matches(setup_point_mission, monkeypatch):
    async def _deny(self, _):
        raise ControlArbiterError("joystick_owner", "joystick owns motion")

    monkeypatch.setattr(
        "mission_services.get_control_arbiter",
        lambda: type("A", (), {"ensure_mission_motion_allowed": _deny})(),
    )
    await main.point_mission.start(main.ros_node, main.offboard_ctrl, main.hold_owner)
    with pytest.raises(MissionServiceError) as rest_exc:
        await continue_point_service(_ctx())
    assert rest_exc.value.status_code == 409
    with pytest.raises(MissionServiceError) as sock_exc:
        await continue_point_service(
            build_service_context(
                offboard_ctrl=main.offboard_ctrl,
                point_mission=main.point_mission,
                ros_node=main.ros_node,
                hold_owner=main.hold_owner,
                path_mgr=None,
                mission_capture=None,
                transport="socketio",
                operation_coordinator=main.operation_coordinator,
            )
        )
    assert sock_exc.value.message == rest_exc.value.message


@pytest.mark.anyio
async def test_socket_continue_rejects_joystick_owner(setup_point_mission, monkeypatch):
    async def _deny(self, _):
        raise ControlArbiterError("joystick_owner", "joystick owns motion")

    monkeypatch.setattr(
        "mission_services.get_control_arbiter",
        lambda: type("A", (), {"ensure_mission_motion_allowed": _deny})(),
    )
    await main.point_mission.start(main.ros_node, main.offboard_ctrl, main.hold_owner)
    with pytest.raises(MissionServiceError) as exc:
        await continue_point_service(_ctx())
    assert exc.value.status_code == 409


@pytest.mark.anyio
async def test_socket_handlers_emit_command_result_on_service_and_unexpected_errors(monkeypatch):
    sio = FakeSio()
    monkeypatch.setattr(socket_events, "_auth_ok", lambda sid: True)
    socket_events.register_handlers(sio)

    async def boom_service(*args, **kwargs):
        raise RuntimeError("boom")

    async def rejected_restart(ctx, req):
        raise MissionServiceError(409, "restart rejected", "restart_rejected")

    for service_name in (
        "stop_mission_service",
        "abort_mission_service",
        "pause_point_service",
        "resume_point_service",
        "continue_point_service",
        "set_point_obstacle_service",
        "skip_point_service",
        "restart_mission_service",
    ):
        monkeypatch.setattr(f"mission_services.{service_name}", boom_service)

    for handler_name, result_event, payload in (
        ("mission_stop", "mission_stop_result", {}),
        ("mission_abort", "mission_abort_result", {}),
        ("mission_pause", "mission_pause_result", {}),
        ("mission_resume", "mission_resume_result", {}),
        ("point_continue", "point_continue_result", {}),
        ("mission_obstacle", "mission_obstacle_result", {"clear": True}),
        ("point_skip", "point_skip_result", {"point_index": 0}),
        ("mission_restart", "mission_restart_result", {"mission_id": "m1"}),
    ):
        before = len(sio.emitted)
        await sio.handlers[handler_name]("sid", payload)
        emitted = sio.emitted[before:]
        expected = {
            "success": False,
            "status": 500,
            "code": "error",
            "message": "boom",
        }
        assert ("mission_error", expected, "sid") in emitted
        assert (result_event, expected, "sid") in emitted

    monkeypatch.setattr("mission_services.restart_mission_service", rejected_restart)

    await sio.handlers["mission_restart"]("sid", {"mission_id": "m1"})
    restart_payload = {
        "success": False,
        "status": 409,
        "code": "restart_rejected",
        "message": "restart rejected",
    }
    assert ("mission_error", restart_payload, "sid") in sio.emitted
    assert ("mission_restart_result", restart_payload, "sid") in sio.emitted
