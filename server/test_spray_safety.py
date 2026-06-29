"""Unit tests for server/spray_safety.py (Task_17 closure)."""
from __future__ import annotations

import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from spray_safety import (
    _accepted_off_status,
    _evaluate_off_status,
    build_spray_telemetry_fields,
    force_spray_off_confirmed,
    requires_spray_off_before_mode_change,
    spray_off_blocks_success,
    SprayOffResult,
)


def _fresh_off_status(**overrides) -> dict:
    base = {
        "status_stale": False,
        "pending_command": False,
        "accepted_command_on": False,
        "commanded_on": False,
        "off_acknowledged": True,
        "confirmed_off": True,
        "physical_confirmation_available": False,
        "physical_actuator_state": "UNAVAILABLE",
        "physical_feedback_timestamp_monotonic_s": None,
        "physical_feedback_age_s": None,
        "physical_feedback_timeout_s": 1.0,
        "physical_feedback_stale": True,
    }
    base.update(overrides)
    return base


def _physical_off_feedback(**overrides) -> dict:
    actuator = {
        "physical_confirmation_available": True,
        "physical_on": False,
        "physical_confirmation_source": "feedback_sensor",
        "physical_feedback_timestamp_monotonic_s": 100.0,
        "physical_feedback_age_s": 0.05,
        "physical_feedback_timeout_s": 1.0,
        "physical_feedback_stale": False,
    }
    base = _fresh_off_status(
        physical_confirmation_available=True,
        physical_actuator_state="OFF",
        physical_confirmed_off=True,
        physical_confirmation_source="feedback_sensor",
        physical_feedback_timestamp_monotonic_s=100.0,
        physical_feedback_age_s=0.05,
        physical_feedback_timeout_s=1.0,
        physical_feedback_stale=False,
        actuator=dict(actuator),
    )
    overrides = dict(overrides)
    actuator_overrides = overrides.pop("actuator", None)
    base.update(overrides)
    if actuator_overrides:
        base["actuator"] = {**actuator, **actuator_overrides}
    return base


class FakeSprayNode:
    def __init__(self, status: dict):
        self.status = dict(status)
        self.manual_calls = []

    def get_spray_runtime_status(self):
        return dict(self.status)

    def publish_spray_manual(self, on):
        self.manual_calls.append(bool(on))

    async def cancel_spray_dwell_async(self):
        return True, "ok"


def test_accepted_off_status_happy_path():
    ok, reason = _accepted_off_status(_fresh_off_status())
    assert ok is True
    assert reason == "accepted OFF (command-level; physical feedback unavailable)"


def test_accepted_off_status_stale():
    ok, reason = _accepted_off_status({"status_stale": True})
    assert ok is False
    assert reason == "spray runtime status stale"


def test_accepted_off_status_pending_off():
    ok, reason = _accepted_off_status(
        _fresh_off_status(pending_command=True, pending_command_on=False, off_acknowledged=False)
    )
    assert ok is False
    assert "pending" in reason


def test_accepted_off_status_commanded_on_true():
    ok, reason = _accepted_off_status(
        _fresh_off_status(commanded_on=True, off_acknowledged=False)
    )
    assert ok is False
    assert reason == "spray commanded_on is true"


def test_accepted_off_status_off_acknowledged_false():
    ok, reason = _accepted_off_status(
        _fresh_off_status(off_acknowledged=False, confirmed_off=False)
    )
    assert ok is False
    assert reason == "spray OFF not acknowledged"


def test_accepted_off_status_missing_fields_fail_closed():
    ok, reason = _accepted_off_status({})
    assert ok is False
    assert reason == "spray runtime status stale"


def test_accepted_off_status_nested_actuator_fallback():
    ok, reason = _accepted_off_status(
        {
            "status_stale": False,
            "actuator": {
                "pending": False,
                "accepted_on": False,
                "commanded_on": False,
                "off_confirmed": True,
            },
        }
    )
    assert ok is True


def test_force_off_command_ack_without_physical_feedback_is_command_level_only():
    node = FakeSprayNode(_fresh_off_status())

    result = asyncio.run(
        force_spray_off_confirmed(node, timeout_s=0.05, poll_interval_s=0.01)
    )

    assert result.success is True
    assert result.command_off_acknowledged is True
    assert result.physical_confirmation_available is False
    assert result.physical_off_confirmed is False
    assert result.confirmation_level == "command"
    assert "command-level" in result.message
    assert node.manual_calls == [False]


def test_force_off_with_physical_feedback_off_is_physically_confirmed():
    node = FakeSprayNode(_physical_off_feedback())

    result = asyncio.run(
        force_spray_off_confirmed(node, timeout_s=0.05, poll_interval_s=0.01)
    )

    assert result.success is True
    assert result.command_off_acknowledged is True
    assert result.physical_confirmation_available is True
    assert result.physical_off_confirmed is True
    assert result.recovery_required is False
    assert result.confirmation_level == "physical"


def test_force_off_with_physical_feedback_on_is_rejected_for_recovery():
    node = FakeSprayNode(
        _physical_off_feedback(
            physical_actuator_state="ON",
            physical_confirmed_off=False,
            actuator={"physical_on": True},
        )
    )

    result = asyncio.run(
        force_spray_off_confirmed(node, timeout_s=0.03, poll_interval_s=0.01)
    )

    assert result.success is False
    assert result.command_off_acknowledged is True
    assert result.physical_confirmation_available is True
    assert result.physical_off_confirmed is False
    assert result.recovery_required is True
    assert "physical actuator state is ON" in result.failure_reason


def test_physical_feedback_stale_is_rejected():
    evaluation = _evaluate_off_status(
        _fresh_off_status(
            status_stale=True,
            physical_confirmation_available=True,
            physical_actuator_state="OFF",
            physical_confirmed_off=True,
            physical_confirmation_source="feedback_sensor",
            actuator={
                "physical_confirmation_available": True,
                "physical_on": False,
                "physical_confirmation_source": "feedback_sensor",
            },
        )
    )

    assert evaluation.accepted is False
    assert evaluation.command_off_acknowledged is False
    assert evaluation.physical_confirmation_available is True
    assert evaluation.physical_off_confirmed is False
    assert evaluation.recovery_required is True
    assert "stale" in evaluation.reason


def test_physical_feedback_stale_flag_rejects_otherwise_valid_off():
    evaluation = _evaluate_off_status(
        _physical_off_feedback(physical_feedback_stale=True)
    )

    assert evaluation.accepted is False
    assert evaluation.command_off_acknowledged is True
    assert evaluation.physical_confirmation_available is True
    assert evaluation.physical_off_confirmed is False
    assert evaluation.recovery_required is True
    assert "physical feedback stale" in evaluation.reason


def test_physical_off_conflicts_with_nested_actuator_on():
    evaluation = _evaluate_off_status(
        _physical_off_feedback(actuator={"physical_on": True})
    )

    assert evaluation.accepted is False
    assert evaluation.command_off_acknowledged is True
    assert evaluation.physical_confirmation_available is True
    assert evaluation.physical_off_confirmed is False
    assert evaluation.recovery_required is True
    assert "physical actuator state is ON" in evaluation.reason


def test_physical_confirmation_available_without_definitive_proof_fails():
    evaluation = _evaluate_off_status(
        _physical_off_feedback(
            physical_actuator_state="UNKNOWN",
            physical_confirmed_off=False,
            actuator={"physical_on": None},
        )
    )

    assert evaluation.accepted is False
    assert evaluation.command_off_acknowledged is True
    assert evaluation.physical_confirmation_available is True
    assert evaluation.physical_off_confirmed is False
    assert evaluation.recovery_required is True
    assert "not definitive OFF" in evaluation.reason


def test_physical_feedback_unknown_unavailable_or_fault_state_is_rejected():
    for state in ("UNKNOWN", "UNAVAILABLE", "FAULT"):
        evaluation = _evaluate_off_status(
            _fresh_off_status(
                physical_confirmation_available=True,
                physical_actuator_state=state,
                physical_confirmed_off=False,
                physical_confirmation_source="feedback_sensor",
                actuator={
                    "physical_confirmation_available": True,
                    "physical_on": None,
                    "physical_confirmation_source": "feedback_sensor",
                },
            )
        )
        assert evaluation.accepted is False
        assert evaluation.command_off_acknowledged is True
        assert evaluation.physical_confirmation_available is True
        assert evaluation.physical_off_confirmed is False
        assert evaluation.recovery_required is True


def test_requires_spray_off_only_when_leaving_offboard():
    assert requires_spray_off_before_mode_change("OFFBOARD", "MANUAL") is True
    assert requires_spray_off_before_mode_change("MANUAL", "STABILIZED") is False
    assert requires_spray_off_before_mode_change("OFFBOARD", "OFFBOARD") is False


def test_spray_off_blocks_success_only_for_live_unconfirmed():
    assert spray_off_blocks_success(
        SprayOffResult(
            success=False,
            attempted=True,
            timeout=True,
            fault=False,
            live=True,
            message="timeout",
        )
    )
    assert not spray_off_blocks_success(
        SprayOffResult(
            success=True,
            attempted=True,
            timeout=False,
            fault=False,
            live=True,
            message="ok",
        )
    )
    assert not spray_off_blocks_success(
        SprayOffResult(
            success=False,
            attempted=False,
            timeout=False,
            fault=True,
            live=False,
            message="unavailable",
        )
    )


def test_build_spray_telemetry_stale_suppresses_spraying_and_marking():
    fields = build_spray_telemetry_fields(
        legacy_spraying=True,
        spray_rt={
            "status_stale": True,
            "status_age_s": 2.0,
            "accepted_command_on": True,
            "physical_confirmation_available": False,
            "physical_actuator_state": "UNAVAILABLE",
        },
        mission_running=True,
    )
    assert fields["spraying"] is False
    assert fields["marking_state"] == "transit"
    assert fields["spray_recovery_required"] is True
    assert fields["physical_actuator_state"] is None
    assert fields["physical_feedback_supported"] is None


def test_build_spray_telemetry_marking_requires_fresh_accepted_on():
    fields = build_spray_telemetry_fields(
        legacy_spraying=True,
        spray_rt={
            "status_stale": False,
            "status_age_s": 0.05,
            "spray_state": "ACCEPTED_ON",
            "accepted_command_on": True,
            "desired_on": True,
            "pending_command": False,
            "off_acknowledged": False,
            "physical_confirmation_available": False,
            "physical_actuator_state": "UNAVAILABLE",
        },
        mission_running=True,
    )
    assert fields["spraying"] is True
    assert fields["marking_state"] == "marking"
    assert fields["last_spray_command_result"] == "accepted_on"


def test_physical_available_without_timestamp_fails_closed():
    evaluation = _evaluate_off_status(
        _physical_off_feedback(
            physical_feedback_timestamp_monotonic_s=None,
            actuator={"physical_feedback_timestamp_monotonic_s": None},
        )
    )
    assert evaluation.accepted is False
    assert "timestamp" in evaluation.reason


def test_physical_available_with_stale_age_fails_closed():
    evaluation = _evaluate_off_status(
        _physical_off_feedback(physical_feedback_age_s=2.0)
    )
    assert evaluation.accepted is False
    assert "age" in evaluation.reason


def test_physical_available_with_fresh_off_is_accepted():
    evaluation = _evaluate_off_status(_physical_off_feedback())
    assert evaluation.accepted is True
    assert evaluation.physical_off_confirmed is True


def test_physical_available_with_fresh_on_is_rejected():
    evaluation = _evaluate_off_status(
        _physical_off_feedback(
            physical_actuator_state="ON",
            physical_confirmed_off=False,
            actuator={"physical_on": True},
        )
    )
    assert evaluation.accepted is False
    assert "physical actuator state is ON" in evaluation.reason


def test_physical_available_with_non_finite_age_fails_closed():
    evaluation = _evaluate_off_status(
        _physical_off_feedback(physical_feedback_age_s=float("nan"))
    )
    assert evaluation.accepted is False
    assert "non_finite" in evaluation.reason or "invalid" in evaluation.reason


def test_physical_contradictory_stale_flag_with_fresh_age_fails_closed():
    evaluation = _evaluate_off_status(
        _physical_off_feedback(
            physical_feedback_age_s=0.05,
            physical_feedback_stale=True,
        )
    )
    assert evaluation.accepted is False
    assert "stale" in evaluation.reason


def test_build_spray_telemetry_forwards_runtime_detail_fields():
    runtime = {
        "status_stale": False,
        "status_age_s": 0.02,
        "desired_on": True,
        "pending_command": False,
        "accepted_command_on": True,
        "off_acknowledged": False,
        "commanded_on": True,
        "confirmed_off": False,
        "physical_confirmation_available": True,
        "physical_confirmed_off": True,
        "physical_feedback_stale": False,
        "physical_feedback_age_s": 0.04,
        "physical_actuator_state": "OFF",
        "actuator_failure_state": "",
        "projected_arc_length_m": 12.5,
        "projection_segment_index": 3,
        "projection_xtrack_error_m": 0.01,
        "projection_jump_m": 0.0,
        "ambiguity_clearance_confidence": 0.9,
        "projection_ambiguous": False,
        "along_track_speed_mps": 0.35,
        "cross_track_speed_mps": 0.01,
        "velocity_heading_error_deg": 1.2,
        "current_run_length_m": 0.4,
        "next_run_length_m": 0.3,
        "next_boundary_kind": "MARK_TO_TRANSIT",
        "distance_to_next_boundary_m": 0.12,
        "raw_on_lead_m": 0.11,
        "bounded_on_lead_m": 0.11,
        "raw_off_lead_m": 0.05,
        "bounded_off_lead_m": 0.05,
        "lead_clamped": False,
        "lead_block_reason": "",
        "flow_mode": "mapped",
        "target_flow": 0.8,
        "target_pwm": 1500.0,
        "command_pwm": 1490.0,
        "pwm_ramp_limited": False,
        "flow_under_capacity": False,
        "flow_clamp_reason": "",
        "geometry_hash": "base-hash",
        "runtime_spray_geometry_hash": "dash-hash",
        "actuator": {"pending_value": 1.0},
    }
    fields = build_spray_telemetry_fields(
        legacy_spraying=True,
        spray_rt=runtime,
        mission_running=True,
        mission_dash={
            "dash_feasible": True,
            "dash_feasibility_reason": "",
            "shortest_dash_on_run_m": 0.3,
            "shortest_dash_off_gap_m": 0.1,
            "dash_phase_reset": "per_mark_region",
        },
    )
    assert fields["projection_s"] == 12.5
    assert fields["next_boundary_kind"] == "MARK_TO_TRANSIT"
    assert fields["pending_value"] == 1.0
    assert fields["runtime_spray_geometry_hash"] == "dash-hash"
    assert fields["dash_feasible"] is True
    assert fields["shortest_dash_off_gap_m"] == 0.1


def test_build_spray_telemetry_stale_suppresses_detail_fields():
    fields = build_spray_telemetry_fields(
        legacy_spraying=True,
        spray_rt={
            "status_stale": True,
            "status_age_s": 2.0,
            "accepted_command_on": True,
            "projected_arc_length_m": 4.0,
            "geometry_hash": "hash",
        },
        mission_running=True,
    )
    assert fields["projection_s"] is None
    assert fields["desired_on"] is None
    assert fields["geometry_hash"] is None
    assert fields["spraying"] is False


def test_build_spray_telemetry_pending_fields():
    fields = build_spray_telemetry_fields(
        legacy_spraying=False,
        spray_rt={
            "status_stale": False,
            "status_age_s": 0.02,
            "pending_command": True,
            "pending_command_on": True,
            "accepted_command_on": False,
            "physical_actuator_state": "UNAVAILABLE",
        },
        mission_running=True,
    )
    assert fields["pending_command"] is True
    assert fields["spraying"] is False
    assert fields["marking_state"] == "transit"
    assert fields["last_spray_command_result"] == "pending_on"
