"""Integration tests for Task_17 server/telemetry closure."""
from __future__ import annotations

import asyncio
import os
import sys
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from bridge_health import BridgeHealthManager
from mission_loading import start_mission_for_controller
from models import ArmRequest, ModeRequest, MissionState
from routes.vehicle import arm_vehicle, set_mode
from models import VehicleMode
from spray_safety import SprayOffResult, build_spray_telemetry_fields


class SprayTrackingNode:
    def __init__(self, *, state=None, runtime=None, spray_off_success=True):
        self.state = state or {"armed": True, "mode": "OFFBOARD", "spraying": True}
        self.runtime = runtime or {
            "status_stale": False,
            "status_age_s": 0.05,
            "accepted_command_on": False,
            "off_acknowledged": True,
            "confirmed_off": True,
            "commanded_on": False,
            "pending_command": False,
            "spray_state": "ACCEPTED_OFF",
            "physical_confirmation_available": False,
            "physical_actuator_state": "UNAVAILABLE",
        }
        self.manual_calls = []
        self.arm_calls = []
        self.mode_calls = []
        self.spray_off_success = spray_off_success
        self._spray_off_calls = 0

    def get_state(self):
        return dict(self.state)

    def get_spray_runtime_status(self):
        status = dict(self.runtime)
        if not self.spray_off_success and self._spray_off_calls > 0:
            status.update(
                {
                    "accepted_command_on": True,
                    "off_acknowledged": False,
                    "confirmed_off": False,
                    "commanded_on": True,
                }
            )
        return status

    def publish_spray_manual(self, on):
        self.manual_calls.append(bool(on))

    async def cancel_spray_dwell_async(self):
        return True, "ok"

    async def arm_async(self, arm):
        self.arm_calls.append(bool(arm))
        return True, "ok"

    async def set_mode_async(self, mode):
        self.mode_calls.append(mode)
        return True, "ok"


@pytest.fixture
def activity_log():
    import main

    main.activity_log.clear()
    yield main.activity_log
    main.activity_log.clear()


@pytest.mark.asyncio
async def test_disarm_calls_spray_off_before_fcu_disarm(activity_log):
    import main

    node = SprayTrackingNode()
    main.ros_node = node

    resp = await arm_vehicle(ArmRequest(arm=False))

    assert node.manual_calls == [False]
    assert node.arm_calls == [False]
    assert resp.disarmed is True
    assert resp.spray_off_confirmed is True
    assert resp.success is True


@pytest.mark.asyncio
async def test_disarm_off_timeout_does_not_report_safe_success(activity_log):
    import main

    node = SprayTrackingNode(spray_off_success=False)
    main.ros_node = node

    async def slow_off(*args, **kwargs):
        node._spray_off_calls += 1
        node.publish_spray_manual(False)
        await asyncio.sleep(0.15)
        return SprayOffResult(
            success=False,
            attempted=True,
            timeout=True,
            fault=False,
            live=True,
            message="spray OFF confirmation timed out: spray accepted command is ON",
        )

    with patch("spray_safety.force_spray_off_confirmed", side_effect=slow_off):
        resp = await arm_vehicle(ArmRequest(arm=False))

    assert resp.disarmed is True
    assert resp.spray_off_confirmed is False
    assert resp.success is False
    assert "not confirmed" in resp.message


@pytest.mark.asyncio
async def test_set_mode_offboard_to_manual_confirms_spray_off_first(activity_log):
    import main

    node = SprayTrackingNode(state={"armed": True, "mode": "OFFBOARD"})
    main.ros_node = node

    resp = await set_mode(ModeRequest(mode=VehicleMode.MANUAL))

    assert node.manual_calls == [False]
    assert node.mode_calls == ["MANUAL"]
    assert resp.spray_off_confirmed is True
    assert resp.success is True


@pytest.mark.asyncio
async def test_set_mode_manual_to_manual_skips_spray_off(activity_log):
    import main

    node = SprayTrackingNode(state={"armed": True, "mode": "MANUAL"})
    main.ros_node = node

    resp = await set_mode(ModeRequest(mode=VehicleMode.MANUAL))

    assert node.manual_calls == []
    assert node.mode_calls == ["MANUAL"]
    assert resp.spray_off_result is None
    assert resp.success is True


@pytest.mark.asyncio
async def test_bridge_recovery_sets_recovery_required_when_off_unconfirmed():
    node = SprayTrackingNode(spray_off_success=False)
    ctrl = MagicMock()
    ctrl.state = MissionState.RUNNING
    ctrl.stop_async = AsyncMock(return_value={"success": True})

    mgr = BridgeHealthManager(
        node,
        ctrl,
        record=lambda *_: None,
        emit=AsyncMock(),
        auto_recover=True,
    )

    with patch.object(mgr, "_within_backoff", return_value=True), patch.object(
        mgr, "_restart_px4dxp", new=AsyncMock(return_value=0)
    ), patch.object(mgr, "_transition", new=AsyncMock()), patch(
        "bridge_health.force_spray_off_confirmed",
        new=AsyncMock(
            return_value=type(
                "R",
                (),
                {
                    "success": False,
                    "attempted": True,
                    "live": True,
                    "message": "timeout",
                    "as_dict": lambda self: {
                        "success": False,
                        "attempted": True,
                        "live": True,
                        "message": "timeout",
                    },
                },
            )()
        ),
    ):
        await mgr._recover("fcu_disconnected")

    assert mgr._spray_recovery_required is True
    assert mgr._spray_recovery_reason == "timeout"
    ctrl.stop_async.assert_awaited_once()


@pytest.mark.asyncio
async def test_mission_start_failure_cleanup_always_attempts_confirmed_off(monkeypatch):
    import main

    node = SprayTrackingNode()
    ctrl = MagicMock()
    ctrl.spray_mode = "point"
    ctrl.has_protected_mission = False
    ctrl.state = MissionState.IDLE
    ctrl.start_async = AsyncMock(return_value=(True, "started"))
    ctrl.stop_async = AsyncMock(return_value={"success": True})

    point_mission = MagicMock()
    point_mission.prepare = MagicMock()
    point_mission.start = AsyncMock(return_value=(False, "point start failed"))
    hold_owner = MagicMock()

    monkeypatch.setattr(main, "point_mission", point_mission)
    monkeypatch.setattr(main, "hold_owner", hold_owner)

    path_mgr = MagicMock()
    path_mgr.load_path.return_value = [(0.0, 0.0), (1.0, 0.0)]
    path_mgr.preview_path.return_value = MagicMock(
        waypoints=[MagicMock(spray=True), MagicMock(spray=True)]
    )

    ok, message = await start_mission_for_controller(
        ctrl,
        path_mgr,
        node,
        name="test_path",
    )

    assert ok is False
    assert message == "point start failed"
    assert node.manual_calls == [False]
    ctrl.stop_async.assert_awaited_once()


def test_rest_telemetry_forwards_spray_runtime_detail_fields():
    from models import TelemetryData
    from routes.telemetry import telemetry_latest

    class Node:
        def get_state(self):
            return {"spraying": False, "armed": True, "rpp_state": 0}

        def get_spray_runtime_status(self):
            return {
                "status_stale": False,
                "status_age_s": 0.03,
                "accepted_command_on": False,
                "off_acknowledged": True,
                "confirmed_off": True,
                "projected_arc_length_m": 2.5,
                "projection_segment_index": 1,
                "geometry_hash": "base",
                "runtime_spray_geometry_hash": "dash",
                "flow_mode": "mapped",
            }

    class Ctrl:
        state = MissionState.IDLE

        def loaded_path_summary(self):
            return {
                "dash_feasible": True,
                "dash_feasibility_reason": "",
                "shortest_dash_on_run_m": 0.3,
                "shortest_dash_off_gap_m": 0.1,
                "dash_phase_reset": "per_mark_region",
                "dash_expected_speed_mps": 0.42,
                "dash_feasibility_speed_source": "staged_marking_speed_mps",
            }

    import main

    main.ros_node = Node()
    main.offboard_ctrl = Ctrl()

    async def run():
        data = await telemetry_latest()
        assert isinstance(data, TelemetryData)
        assert data.projection_s == 2.5
        assert data.geometry_hash == "base"
        assert data.runtime_spray_geometry_hash == "dash"
        assert data.dash_feasible is True
        assert data.dash_phase_reset == "per_mark_region"
        assert data.dash_expected_speed_mps == 0.42
        assert data.dash_feasibility_speed_source == "staged_marking_speed_mps"

    asyncio.run(run())


def test_websocket_telemetry_fields_include_command_state():
    fields = build_spray_telemetry_fields(
        legacy_spraying=True,
        spray_rt={
            "status_stale": False,
            "status_age_s": 0.04,
            "spray_state": "PENDING_ON",
            "desired_on": True,
            "pending_command": True,
            "pending_command_on": True,
            "accepted_command_on": False,
            "off_acknowledged": False,
            "physical_confirmation_available": False,
            "physical_actuator_state": "UNAVAILABLE",
            "actuator_failure_state": "",
        },
        mission_running=True,
    )
    required = {
        "spray_state",
        "desired_on",
        "pending_command",
        "accepted_command_on",
        "accepted_command_off",
        "off_acknowledged",
        "physical_feedback_supported",
        "physical_actuator_state",
        "spray_runtime_status_age_s",
        "spray_faulted",
        "spray_recovery_required",
        "last_spray_command_result",
        "last_spray_command_reason",
        "spraying",
        "marking_state",
    }
    assert required.issubset(fields.keys())
    assert fields["physical_actuator_state"] == "UNAVAILABLE"
    assert fields["physical_feedback_supported"] is False
    assert fields["pending_command"] is True
