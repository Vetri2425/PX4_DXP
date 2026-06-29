#!/usr/bin/env python3
"""Task_19 point dwell and dash spray mode hardening tests."""

from __future__ import annotations

import asyncio
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "server"))

from point_ingest import SprayPoint
from point_mission import PointMissionOrchestrator, PointMissionRun, PointMissionState, SprayRuntimeSchemaError
from spray_config import DashPhaseReset, PointSprayParams, SprayConfiguration, SprayMode, validate_spray_configuration
from spray_dash import apply_dash_pattern, validate_dash_feasibility
from path_identity import path_geometry_fingerprint
from spray_path_model import build_path_model
from test_point_mission import FakeOffboard, FakeRos


def _point_cfg(**overrides) -> SprayConfiguration:
    point = PointSprayParams(
        default_dwell_s=0.05,
        arrival_tolerance_m=0.05,
        settle_time_s=0.0,
        leg_timeout_s=2.0,
        settle_speed_mps=0.05,
        settle_yaw_rate_rad_s=0.05,
        hold_drift_tolerance_m=0.02,
        hold_drift_policy="fail",
    )
    return SprayConfiguration(mode=SprayMode.POINT, point=point, revision=3, **overrides)


class CountingOffboard(FakeOffboard):
    def __init__(self):
        self.abort_calls = 0

    async def abort_async(self):
        self.abort_calls += 1
        return await super().abort_async()


class FailingCompleteOffboard(FakeOffboard):
    async def complete_async(self):
        return {
            "success": False,
            "message": "parent completion failed",
            "warnings": [],
            "spray_off_result": {"success": True},
        }


class HungDwellCancelRos(FakeRos):
    def __init__(self, hang_s: float = 0.2):
        super().__init__()
        self.hang_s = hang_s
        self.cancel_attempts = 0

    async def cancel_spray_dwell_async(self):
        self.cancel_attempts += 1
        await asyncio.sleep(self.hang_s)
        self.live_dwell = None
        return True, "ok"


class ResidualDwellRos(FakeRos):
    """Simulates spray node retaining dwell after server would be IDLE."""

    def __init__(self):
        super().__init__()
        self._residual = True
        self.cancel_calls = 0

    async def cancel_spray_dwell_async(self):
        self.cancel_calls += 1
        self._residual = False
        return True, "ok"

    def publish_spray_manual(self, on: bool):
        super().publish_spray_manual(on)
        if on is False:
            self._residual = False

    def get_spray_runtime_status(self):
        if self._residual:
            return {
                "status_stale": False,
                "ready": True,
                "active_dwell": True,
                "dwell_remaining_s": 1.0,
                "commanded_on": True,
                "confirmed_off": False,
                "off_acknowledged": False,
                "accepted_command_on": True,
                "pending_command": False,
                "dwell_command_id": 9,
                "dwell_mission_id": "orphan",
                "dwell_point_index": 0,
                "configuration_revision": 1,
                "model_revision": 0,
                "timestamp_monotonic_s": time.monotonic(),
                "last_error": "",
            }
        return {
            "status_stale": False,
            "ready": True,
            "active_dwell": False,
            "dwell_remaining_s": 0.0,
            "commanded_on": False,
            "confirmed_off": True,
            "off_acknowledged": True,
            "accepted_command_on": False,
            "pending_command": False,
            "dwell_command_id": None,
            "dwell_mission_id": "",
            "dwell_point_index": None,
            "configuration_revision": 1,
            "model_revision": 0,
            "timestamp_monotonic_s": time.monotonic(),
            "last_error": "",
        }


# ── DASH integration ──────────────────────────────────────────────────────────


def test_dash_exact_split_inserts_boundary_points():
    model = build_path_model(
        points=[(0.0, 0.0), (2.0, 0.0)],
        flags=[True, True],
    )
    dashed = apply_dash_pattern(model, 0.5, 0.5, DashPhaseReset.PER_MARK_REGION)
    assert len(dashed.points) > len(model.points)
    lengths = []
    for i in range(1, len(dashed.points)):
        if dashed.flags[i - 1]:
            lengths.append(dashed.cumulative_s[i] - dashed.cumulative_s[i - 1])
    assert any(abs(length - 0.5) < 0.02 for length in lengths)


def test_dash_base_hash_unchanged_runtime_hash_differs():
    points = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]
    flags = [False, True, True]
    base = build_path_model(points, flags)
    base_hash = path_geometry_fingerprint(points, flags)
    dashed = apply_dash_pattern(base, 0.5, 0.5, DashPhaseReset.PER_MARK_REGION)
    runtime_hash = path_geometry_fingerprint(dashed.points, dashed.flags)
    assert base_hash == path_geometry_fingerprint(points, flags)
    assert runtime_hash != base_hash


def test_dash_solid_mode_off_distance_zero():
    model = build_path_model([(0.0, 0.0), (2.0, 0.0)], [True, True])
    dashed = apply_dash_pattern(model, 0.5, 0.0, DashPhaseReset.PER_MARK_REGION)
    assert all(dashed.flags)


def test_dash_all_gap_mode_on_distance_zero():
    model = build_path_model([(0.0, 0.0), (2.0, 0.0)], [True, True])
    dashed = apply_dash_pattern(model, 0.0, 0.5, DashPhaseReset.PER_MARK_REGION)
    assert not any(dashed.flags)


def test_dash_feasibility_rejects_short_on_run():
    model = build_path_model([(0.0, 0.0), (0.04, 0.0)], [True, True])
    config = validate_spray_configuration(
        {
            "mission_id": "dash-short",
            "path_fingerprint": "fp",
            "configuration_revision": 1,
            "spray_mode": "dash",
            "dash_on_distance_m": 0.02,
            "dash_off_distance_m": 0.5,
            "continuous": {"min_on_distance_m": 0.05},
        }
    )
    result = validate_dash_feasibility(model, config, expected_speed_mps=0.35)
    assert result.dash_feasible is False
    assert "shortest dash ON" in result.dash_feasibility_reason


def test_dash_infeasible_terminal_partial_rejected():
    model = build_path_model([(0.0, 0.0), (0.03, 0.0)], [True, True])
    config = validate_spray_configuration(
        {
            "mission_id": "dash-partial",
            "path_fingerprint": "fp",
            "configuration_revision": 1,
            "spray_mode": "dash",
            "dash_on_distance_m": 0.02,
            "dash_off_distance_m": 0.5,
            "continuous": {"min_on_distance_m": 0.05},
        }
    )
    result = validate_dash_feasibility(model, config, expected_speed_mps=0.35)
    assert result.dash_feasible is False


def test_dash_lead_and_hysteresis_against_exact_boundaries():
    model = build_path_model([(0.0, 0.0), (10.0, 0.0)], [True, True])
    config = validate_spray_configuration(
        {
            "mission_id": "dash-lead",
            "path_fingerprint": "fp",
            "configuration_revision": 1,
            "spray_mode": "dash",
            "dash_on_distance_m": 1.0,
            "dash_off_distance_m": 1.0,
            "continuous": {"min_on_distance_m": 0.05},
        }
    )
    result = validate_dash_feasibility(model, config, expected_speed_mps=0.35)
    assert result.dash_feasible is True
    assert result.expected_on_lead_m > 0.0
    assert result.expected_off_lead_m > 0.0


# ── Schema / identity ─────────────────────────────────────────────────────────


def test_schema_error_on_missing_commanded_on():
    orch = PointMissionOrchestrator()
    with pytest.raises(SprayRuntimeSchemaError):
        orch._validate_dwell_poll_status(
            {
                "dwell_command_id": 1,
                "dwell_mission_id": "m1",
                "dwell_point_index": 0,
                "confirmed_off": True,
                "off_acknowledged": True,
                "active_dwell": False,
                "status_stale": False,
            }
        )


def test_empty_dwell_mission_id_rejected_during_active_dwell():
    orch = PointMissionOrchestrator()
    with pytest.raises(SprayRuntimeSchemaError, match="non-empty dwell_mission_id"):
        orch._validate_dwell_poll_status(
            {
                "dwell_command_id": 1,
                "dwell_mission_id": "",
                "dwell_point_index": 0,
                "commanded_on": True,
                "confirmed_off": False,
                "off_acknowledged": False,
                "active_dwell": True,
                "status_stale": False,
            }
        )


@pytest.mark.anyio
async def test_dwell_revision_mismatch_triggers_fault():
    ros = FakeRos()
    orch = PointMissionOrchestrator()
    orch.load(mission_id="m1", points=[SprayPoint(0, 0, 0.1, 0)], config=_point_cfg())
    token = PointMissionRun(orch.status.generation, "m1", asyncio.Event(), parent_mission_id="m1")
    orch._run_token = token
    orch._bind_dwell_identity(
        token, command_id=7, command_revision=1, point_index=0, source_index=0
    )
    base = {
        "status_stale": False,
        "ready": True,
        "last_error": "",
        "dwell_remaining_s": 0.05,
        "dwell_mission_id": "m1",
        "dwell_point_index": 0,
        "commanded_on": True,
        "confirmed_off": False,
        "off_acknowledged": False,
        "configuration_revision": 99,
        "model_revision": 0,
        "timestamp_monotonic_s": time.monotonic(),
        "dwell_command_id": 7,
        "active_dwell": True,
    }
    ros.runtime_statuses = [base]
    with pytest.raises(RuntimeError, match="mismatch"):
        await orch._wait_dwell_complete(
            token,
            ros,
            None,
            FakeOffboard(),
            SprayPoint(0, 0, 0.1, 0),
            0.1,
            7,
            _point_cfg().point,
        )


@pytest.mark.anyio
async def test_dwell_point_index_mismatch_triggers_fault():
    ros = FakeRos()
    orch = PointMissionOrchestrator()
    orch.load(mission_id="m1", points=[SprayPoint(0, 0, 0.1, 0)], config=_point_cfg())
    token = PointMissionRun(orch.status.generation, "m1", asyncio.Event(), parent_mission_id="m1")
    orch._run_token = token
    orch._bind_dwell_identity(
        token, command_id=7, command_revision=1, point_index=0, source_index=5
    )
    ros.runtime_statuses = [
        {
            "status_stale": False,
            "ready": True,
            "last_error": "",
            "dwell_remaining_s": 0.05,
            "dwell_mission_id": "m1",
            "dwell_point_index": 1,
            "commanded_on": True,
            "confirmed_off": False,
            "off_acknowledged": False,
            "configuration_revision": 3,
            "model_revision": 0,
            "timestamp_monotonic_s": time.monotonic(),
            "dwell_command_id": 7,
            "active_dwell": True,
        }
    ]
    with pytest.raises(RuntimeError, match="mismatch"):
        await orch._wait_dwell_complete(
            token,
            ros,
            None,
            FakeOffboard(),
            SprayPoint(0, 0, 0.1, 5),
            0.1,
            7,
            _point_cfg().point,
        )


# ── Parent lifecycle / cancellation ───────────────────────────────────────────


@pytest.mark.anyio
async def test_point_completion_uses_parent_complete_async():
    ros = FakeRos()
    ros.auto_arrive = True
    ctrl = FakeOffboard()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="m1",
        points=[SprayPoint(1.0, 0.0, 0.05, 0, mark=True)],
        config=_point_cfg(),
    )
    await orch.start(ros, ctrl)
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state == PointMissionState.COMPLETED
    assert ctrl.state.value == "completed"


@pytest.mark.anyio
async def test_parent_complete_failure_prevents_completed():
    ros = FakeRos()
    ros.auto_arrive = True
    ctrl = FailingCompleteOffboard()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="m1",
        points=[SprayPoint(1.0, 0.0, 0.05, 0, mark=True)],
        config=_point_cfg(),
    )
    await orch.start(ros, ctrl)
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state == PointMissionState.FAILED
    assert orch.status.last_transition == "completion_degraded"


@pytest.mark.anyio
async def test_abort_during_dwell_forces_off_and_clears():
    ros = FakeRos()
    ros.auto_arrive = True
    ctrl = CountingOffboard()
    orch = PointMissionOrchestrator()
    orch._DRAIN_TIMEOUT_S = 0.3
    orch.load(
        mission_id="dwell-abort",
        points=[SprayPoint(1.0, 0.0, 2.0, 0, mark=True)],
        config=_point_cfg(),
    )
    await orch.start(ros, ctrl)
    await asyncio.sleep(0.08)
    await orch.abort(ros, offboard_ctrl=ctrl)
    assert orch._task is None
    assert orch.status.spray_off_result is not None
    assert orch.status.dwell_cancel_result is not None
    assert ctrl.abort_calls == 1


@pytest.mark.anyio
async def test_cancel_reports_independent_dwell_and_off_results():
    ros = HungDwellCancelRos(hang_s=0.05)
    ros.auto_arrive = True
    ctrl = CountingOffboard()
    orch = PointMissionOrchestrator()
    orch._DRAIN_TIMEOUT_S = 0.3
    orch.load(
        mission_id="dwell-cancel-timeout",
        points=[SprayPoint(1.0, 0.0, 0.2, 0, mark=True)],
        config=_point_cfg(),
    )
    await orch.start(ros, ctrl)
    await asyncio.sleep(0.08)
    await orch.abort(ros, offboard_ctrl=ctrl)
    assert orch.status.dwell_cancel_result is not None
    assert orch.status.spray_off_result is not None
    assert orch.status.dwell_cancel_result is not orch.status.spray_off_result
    assert "success" in orch.status.dwell_cancel_result
    assert "success" in orch.status.spray_off_result


@pytest.mark.anyio
async def test_cleanup_failure_sets_recovery_required():
    class NeverOffRos(FakeRos):
        def get_spray_runtime_status(self):
            return {
                "status_stale": False,
                "ready": True,
                "active_dwell": False,
                "commanded_on": True,
                "confirmed_off": False,
                "off_acknowledged": False,
                "accepted_command_on": True,
                "pending_command": False,
                "dwell_command_id": None,
                "dwell_mission_id": "",
                "dwell_point_index": None,
                "configuration_revision": 1,
                "model_revision": 0,
                "timestamp_monotonic_s": time.monotonic(),
                "last_error": "",
            }

    ros = NeverOffRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch._DRAIN_TIMEOUT_S = 0.25
    orch.load(
        mission_id="recovery",
        points=[SprayPoint(1.0, 0.0, 0.2, 0, mark=True)],
        config=_point_cfg(),
    )
    await orch.start(ros, FakeOffboard())
    await asyncio.sleep(0.08)
    await orch.abort(ros, offboard_ctrl=FakeOffboard())
    assert orch.status.recovery_required is True


@pytest.mark.anyio
async def test_invalidate_dwell_identity_does_not_clear_active_dwell_flag():
    orch = PointMissionOrchestrator()
    orch.load(mission_id="m1", points=[SprayPoint(0, 0, 0.1, 0)], config=_point_cfg())
    token = PointMissionRun(orch.status.generation, "m1", asyncio.Event(), parent_mission_id="m1")
    orch._run_token = token
    orch._write(token, active_dwell=True, dwell_remaining_s=0.5)
    orch._invalidate_dwell_identity(token)
    assert orch.status.dwell_ownership_invalidated is True
    assert orch.status.active_dwell is True


# ── Restart / races ───────────────────────────────────────────────────────────


@pytest.mark.anyio
async def test_spray_runtime_restart_during_dwell_faults():
    ros = FakeRos()
    orch = PointMissionOrchestrator()
    orch.load(mission_id="m1", points=[SprayPoint(0, 0, 0.1, 0)], config=_point_cfg())
    token = PointMissionRun(orch.status.generation, "m1", asyncio.Event(), parent_mission_id="m1")
    orch._run_token = token
    orch._bind_dwell_identity(
        token, command_id=7, command_revision=1, point_index=0, source_index=0
    )
    now = time.monotonic()
    ros.runtime_statuses = [
        {
            "status_stale": False,
            "ready": True,
            "last_error": "",
            "dwell_remaining_s": 0.05,
            "dwell_mission_id": "m1",
            "dwell_point_index": 0,
            "commanded_on": True,
            "confirmed_off": False,
            "off_acknowledged": False,
            "configuration_revision": 3,
            "model_revision": 0,
            "timestamp_monotonic_s": now,
            "dwell_command_id": 7,
            "active_dwell": True,
        },
        {
            "status_stale": False,
            "ready": True,
            "last_error": "",
            "dwell_remaining_s": 0.05,
            "dwell_mission_id": "m1",
            "dwell_point_index": 0,
            "commanded_on": True,
            "confirmed_off": False,
            "off_acknowledged": False,
            "configuration_revision": 3,
            "model_revision": 99,
            "timestamp_monotonic_s": now + 0.1,
            "dwell_command_id": 7,
            "active_dwell": True,
        },
    ]
    with pytest.raises(RuntimeError, match="restarted"):
        await orch._wait_dwell_complete(
            token,
            ros,
            None,
            FakeOffboard(),
            SprayPoint(0, 0, 0.1, 0),
            0.1,
            7,
            _point_cfg().point,
        )


@pytest.mark.anyio
async def test_startup_reconciliation_module_cancels_residual_dwell():
    from spray_startup_reconciliation import SprayStartupReconciliation

    ros = ResidualDwellRos()
    reconciler = SprayStartupReconciliation()
    await reconciler.start(ros)
    assert reconciler.is_ready()
    assert reconciler.state.residual_detected is True
    assert reconciler.state.recovery_required is False


def main():
    test_dash_exact_split_inserts_boundary_points()
    test_dash_feasibility_rejects_short_on_run()
    test_schema_error_on_missing_commanded_on()
    print("PASS")


if __name__ == "__main__":
    main()