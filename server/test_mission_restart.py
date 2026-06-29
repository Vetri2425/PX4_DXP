#!/usr/bin/env python3
"""Mission restart service tests."""

from __future__ import annotations

import os
import sys
from collections import deque

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import main
from mission_ops import MissionOperationCoordinator
from mission_services import MissionServiceError, build_service_context, restart_mission_service
from models import MissionRestartRequest, MissionState
from offboard_controller import OffboardController
from point_mission import PointMissionOrchestrator
from test_point_mission import FakeRos


def _ctx():
    return build_service_context(
        offboard_ctrl=main.offboard_ctrl,
        point_mission=main.point_mission,
        ros_node=main.ros_node,
        hold_owner=None,
        path_mgr=None,
        mission_capture=None,
        transport="rest",
        operation_coordinator=main.operation_coordinator,
    )


@pytest.fixture
def setup_point_idle(monkeypatch):
    main.operation_coordinator = MissionOperationCoordinator()
    main.ros_node = FakeRos()
    ctrl = OffboardController(main.ros_node, deque())
    ctrl.load_path(
        [(0, 0), (1, 0)],
        name="r.csv",
        spray_flags=[False, False],
        mission_id="rid1",
        is_staged=True,
        allow_replace_protected=True,
        spray_mode="point",
    )
    ctrl.state = MissionState.IDLE
    main.offboard_ctrl = ctrl
    from point_ingest import SprayPoint
    from spray_config import SprayConfiguration, SprayMode

    main.point_mission = PointMissionOrchestrator()
    main.point_mission.load(
        mission_id="rid1",
        points=[SprayPoint(0, 0, 0.05, 0)],
        config=SprayConfiguration(mode=SprayMode.POINT, revision=1),
    )
    yield


@pytest.mark.anyio
async def test_restart_requires_mission_id(setup_point_idle):
    with pytest.raises(MissionServiceError) as exc:
        await restart_mission_service(_ctx(), MissionRestartRequest(mission_id=""))
    assert exc.value.status_code == 422


@pytest.mark.anyio
async def test_restart_rejects_identity_mismatch(setup_point_idle):
    with pytest.raises(MissionServiceError) as exc:
        await restart_mission_service(
            _ctx(), MissionRestartRequest(mission_id="wrong-id")
        )
    assert exc.value.status_code == 409


@pytest.mark.anyio
async def test_restart_active_without_stop_first_conflict(setup_point_idle):
    main.offboard_ctrl.state = MissionState.RUNNING
    with pytest.raises(MissionServiceError) as exc:
        await restart_mission_service(
            _ctx(), MissionRestartRequest(mission_id="rid1")
        )
    assert "stop_first" in exc.value.message


@pytest.mark.anyio
async def test_restart_reset_only_does_not_start_or_arm(setup_point_idle):
    gen_before = main.point_mission.status.generation
    resp = await restart_mission_service(
        _ctx(), MissionRestartRequest(mission_id="rid1")
    )
    assert resp.success and resp.reset and not resp.started
    assert resp.point_mission_generation > gen_before
    assert main.offboard_ctrl.state == MissionState.IDLE


@pytest.mark.anyio
async def test_restart_continuous_resets_to_beginning_not_midrun(setup_point_idle):
    ctrl = main.offboard_ctrl
    ctrl.load_path(
        [(0, 0), (1, 0), (2, 0)],
        name="c.csv",
        spray_flags=[True, True, True],
        mission_id="cont1",
        is_staged=True,
        allow_replace_protected=True,
        spray_mode="continuous",
    )
    ctrl.state = MissionState.COMPLETED
    main.point_mission = None
    resp = await restart_mission_service(
        _ctx(), MissionRestartRequest(mission_id="cont1")
    )
    assert resp.success and resp.reset
    assert ctrl.state == MissionState.IDLE


@pytest.mark.anyio
async def test_restart_stop_first_runs_point_terminal_cleanup(setup_point_idle, monkeypatch):
    from unittest.mock import AsyncMock, MagicMock

    main.offboard_ctrl.state = MissionState.RUNNING
    orch = main.point_mission
    await orch.start(main.ros_node, main.offboard_ctrl)
    terminal_cleanup = AsyncMock(
        return_value=MagicMock(success=True, idempotent=False, terminal_event_emitted=True)
    )
    monkeypatch.setattr(orch, "terminal_cleanup", terminal_cleanup)
    main.offboard_ctrl.stop_async = AsyncMock(
        return_value={"success": True, "state": MissionState.IDLE.value}
    )
    resp = await restart_mission_service(
        _ctx(),
        MissionRestartRequest(mission_id="rid1", stop_first=True),
    )
    assert resp.success and resp.reset and not resp.started
    terminal_cleanup.assert_awaited_once()
    assert terminal_cleanup.await_args.kwargs["reason"] == "restart_stop_first"


@pytest.mark.anyio
async def test_restart_start_after_reset_starts_only_when_requested(
    setup_point_idle, monkeypatch
):
    from unittest.mock import AsyncMock

    started = {"called": False}

    async def fake_start(*args, **kwargs):
        started["called"] = True
        main.offboard_ctrl.state = MissionState.RUNNING
        return True, "started"

    monkeypatch.setattr(
        "mission_services.start_mission_for_controller",
        fake_start,
    )
    resp = await restart_mission_service(
        _ctx(),
        MissionRestartRequest(
            mission_id="rid1",
            start_after_reset=True,
        ),
    )
    assert resp.success and resp.started
    assert started["called"] is True


@pytest.mark.anyio
async def test_restart_gps_surveyed_point_resolves_fresh_on_next_start(setup_point_idle):
    from mission_placement import GPS_SURVEYED
    from point_ingest import SprayPoint

    orch = main.point_mission
    orch._source_frame = GPS_SURVEYED
    orch._origin_gps = (37.0, -122.0, 10.0)
    orch._resolved_points = [SprayPoint(99.0, 88.0, 0.05, 0)]
    await restart_mission_service(_ctx(), MissionRestartRequest(mission_id="rid1"))
    assert orch._resolved_points == []
    assert orch.status.state.value == "idle"


@pytest.mark.anyio
async def test_socket_restart_matches_rest_response(setup_point_idle):
    ctx_rest = _ctx()
    ctx_sock = build_service_context(
        offboard_ctrl=main.offboard_ctrl,
        point_mission=main.point_mission,
        ros_node=main.ros_node,
        hold_owner=None,
        path_mgr=None,
        mission_capture=None,
        transport="socketio",
        operation_coordinator=main.operation_coordinator,
    )
    req = MissionRestartRequest(mission_id="rid1")
    rest = await restart_mission_service(ctx_rest, req)
    sock = await restart_mission_service(ctx_sock, req)
    rest_dump = rest.model_dump()
    sock_dump = sock.model_dump()
    rest_dump.pop("point_mission_generation", None)
    sock_dump.pop("point_mission_generation", None)
    assert rest_dump == sock_dump