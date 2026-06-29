"""Shared spray terminal-safety helpers."""

from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable

from logging_setup import get_logger

log = get_logger("server.spray_safety")


@dataclass
class SprayOffResult:
    success: bool
    attempted: bool
    timeout: bool
    fault: bool
    live: bool
    message: str
    details: dict[str, Any] = field(default_factory=dict)
    command_off_acknowledged: bool = False
    physical_confirmation_available: bool = False
    physical_off_confirmed: bool = False
    recovery_required: bool = False
    failure_reason: str = ""
    confirmation_level: str = "none"

    def as_dict(self) -> dict[str, Any]:
        return {
            "success": self.success,
            "attempted": self.attempted,
            "timeout": self.timeout,
            "fault": self.fault,
            "live": self.live,
            "message": self.message,
            "details": dict(self.details),
            "command_off_acknowledged": self.command_off_acknowledged,
            "physical_confirmation_available": self.physical_confirmation_available,
            "physical_off_confirmed": self.physical_off_confirmed,
            "recovery_required": self.recovery_required,
            "failure_reason": self.failure_reason,
            "confirmation_level": self.confirmation_level,
        }


NON_SPRAY_OPERATIONAL_MODES = frozenset({"OFFBOARD"})


@dataclass
class OffStatusEvaluation:
    accepted: bool
    reason: str
    command_off_acknowledged: bool
    physical_confirmation_available: bool
    physical_off_confirmed: bool
    recovery_required: bool
    details: dict[str, Any] = field(default_factory=dict)

    @property
    def confirmation_level(self) -> str:
        if not self.accepted:
            return "none"
        if self.physical_off_confirmed:
            return "physical"
        return "command"


def requires_spray_off_before_mode_change(
    current_mode: str | None, target_mode: str
) -> bool:
    """True when leaving OFFBOARD — the only mode where auto-spray can be active."""
    current = str(current_mode or "UNKNOWN").upper()
    target = str(target_mode).upper()
    return current == "OFFBOARD" and target not in NON_SPRAY_OPERATIONAL_MODES


def spray_off_blocks_success(result: SprayOffResult) -> bool:
    """Live spray node that could not confirm accepted OFF must not read as safe."""
    return bool(result.attempted and result.live and not result.success)


def _runtime_detail_value(
    spray_rt: dict[str, Any],
    actuator: dict[str, Any],
    *,
    runtime_fresh: bool,
    primary: str,
    fallback: str | None = None,
    actuator_key: str | None = None,
) -> Any:
    if not runtime_fresh:
        return None
    if primary in spray_rt:
        return spray_rt.get(primary)
    if fallback and fallback in spray_rt:
        return spray_rt.get(fallback)
    if actuator_key is not None:
        return actuator.get(actuator_key)
    if fallback and actuator_key is None:
        return actuator.get(fallback)
    return None


def _mission_dash_fields(mission_dash: dict[str, Any] | None) -> dict[str, Any]:
    if not mission_dash:
        return {
            "dash_feasible": None,
            "dash_feasibility_reason": None,
            "shortest_dash_on_run_m": None,
            "shortest_dash_off_gap_m": None,
            "dash_phase_reset": None,
            "dash_expected_speed_mps": None,
            "dash_feasibility_speed_source": None,
        }
    return {
        "dash_feasible": mission_dash.get("dash_feasible"),
        "dash_feasibility_reason": mission_dash.get("dash_feasibility_reason"),
        "shortest_dash_on_run_m": mission_dash.get("shortest_dash_on_run_m"),
        "shortest_dash_off_gap_m": mission_dash.get("shortest_dash_off_gap_m"),
        "dash_phase_reset": mission_dash.get("dash_phase_reset"),
        "dash_expected_speed_mps": mission_dash.get("dash_expected_speed_mps"),
        "dash_feasibility_speed_source": mission_dash.get(
            "dash_feasibility_speed_source"
        ),
    }


CANONICAL_SPRAY_TELEMETRY_FIELDS = (
    "spraying",
    "marking_state",
    "spray_state",
    "desired_on",
    "pending_command",
    "pending_value",
    "accepted_command_on",
    "accepted_command_off",
    "off_acknowledged",
    "commanded_on",
    "confirmed_off",
    "physical_feedback_supported",
    "physical_confirmation_available",
    "physical_feedback_stale",
    "physical_feedback_age_s",
    "physical_actuator_state",
    "physical_off_confirmed",
    "spray_runtime_status_age_s",
    "spray_faulted",
    "spray_recovery_required",
    "last_spray_command_result",
    "last_spray_command_reason",
    "last_command_error",
    "spray_safety_reason",
    "vehicle_state_age_s",
    "vehicle_state_stale",
    "vehicle_state_block_reason",
    "geometry_spray_request",
    "dry_run_active",
    "projection_s",
    "projection_segment_index",
    "projection_xtrack_error_m",
    "projection_jump_m",
    "projection_ambiguous",
    "ambiguity_clearance_confidence",
    "along_track_speed_mps",
    "cross_track_speed_mps",
    "velocity_heading_error_deg",
    "current_run_length_m",
    "next_run_length_m",
    "next_boundary_kind",
    "distance_to_boundary_m",
    "raw_on_lead_m",
    "bounded_on_lead_m",
    "raw_off_lead_m",
    "bounded_off_lead_m",
    "lead_clamped",
    "lead_block_reason",
    "flow_mode",
    "target_flow",
    "target_pwm",
    "command_pwm",
    "pwm_ramp_limited",
    "flow_capacity_limited",
    "flow_under_capacity",
    "flow_clamp_reason",
    "geometry_hash",
    "runtime_spray_geometry_hash",
    "gps_safety_ok",
    "manual_resume_required",
    "dash_feasible",
    "dash_feasibility_reason",
    "dash_expected_speed_mps",
    "dash_feasibility_speed_source",
    "shortest_dash_on_run_m",
    "shortest_dash_off_gap_m",
    "dash_phase_reset",
)


def build_spray_telemetry_fields(
    *,
    legacy_spraying: bool,
    spray_rt: dict[str, Any],
    mission_running: bool,
    mission_dash: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Derive fail-closed spray telemetry from runtime status.

    ``spraying`` means MAVROS-command-accepted ON (not physical paint flow).
    Stale or missing runtime status suppresses ``spraying`` and ``marking_state``
    marking.
    """
    stale = bool(spray_rt.get("status_stale", True))
    runtime_fresh = not stale
    actuator = spray_rt.get("actuator")
    if not isinstance(actuator, dict):
        actuator = {}

    pending = bool(spray_rt.get("pending_command", actuator.get("pending", False)))
    pending_on = spray_rt.get("pending_command_on", actuator.get("pending_on"))
    raw_accepted_on = bool(
        spray_rt.get("accepted_command_on", actuator.get("accepted_on", False))
    )
    raw_commanded_on = bool(
        spray_rt.get("commanded_on", actuator.get("commanded_on", False))
    )
    desired_on = bool(spray_rt.get("desired_on", actuator.get("desired_on", False)))
    off_ack = bool(
        spray_rt.get(
            "off_acknowledged",
            spray_rt.get("confirmed_off", actuator.get("off_confirmed", False)),
        )
    )

    geometry_spray_request = (
        spray_rt.get("geometry_spray_request") if runtime_fresh else None
    )
    flow_mode_value = _runtime_detail_value(
        spray_rt,
        actuator,
        runtime_fresh=runtime_fresh,
        primary="flow_mode",
    )
    disabled_flow = bool(runtime_fresh and flow_mode_value == "disabled")
    accepted_on = False if disabled_flow else raw_accepted_on
    commanded_on = False if disabled_flow else raw_commanded_on
    dry_run_active = (
        True
        if disabled_flow
        else (bool(spray_rt.get("dry_run_active", False)) if runtime_fresh else None)
    )
    spray_state = spray_rt.get("spray_state")
    if disabled_flow:
        spray_state = "DRY_RUN"
    elif not spray_state:
        if pending:
            spray_state = "PENDING_ON" if pending_on else "PENDING_OFF"
        else:
            spray_state = "ACCEPTED_ON" if accepted_on else "ACCEPTED_OFF"
    spraying = (
        runtime_fresh
        and accepted_on
        and not bool(dry_run_active)
        and flow_mode_value != "disabled"
    )

    if not mission_running:
        marking_state = "off"
    elif not runtime_fresh:
        marking_state = "transit"
    elif accepted_on:
        marking_state = "marking"
    else:
        marking_state = "transit"

    accepted_command_off = (
        runtime_fresh and off_ack and not accepted_on and not pending
    )

    physical_feedback_supported = bool(
        spray_rt.get("physical_confirmation_available", False)
    )
    physical_actuator_state = str(
        spray_rt.get("physical_actuator_state", "UNAVAILABLE")
    )

    failure = str(
        spray_rt.get("actuator_failure_state")
        or actuator.get("last_command_failure")
        or ""
    ).strip()
    spray_faulted = bool(failure)
    manual_resume = bool(spray_rt.get("manual_resume_required", False))
    spray_recovery_required = bool(
        spray_faulted
        or manual_resume
        or (
            stale
            and (
                legacy_spraying
                or raw_accepted_on
                or pending
                or desired_on
                or mission_running
            )
        )
    )

    if pending:
        last_result = f"pending_{'on' if pending_on else 'off'}"
    elif accepted_on:
        last_result = "accepted_on"
    elif accepted_command_off:
        last_result = "accepted_off"
    elif failure:
        last_result = "failed"
    else:
        last_result = "unknown"

    last_reason = str(
        failure
        or spray_rt.get("safety_reason")
        or spray_rt.get("gps_safety_reason")
        or spray_rt.get("last_transition")
        or ""
    )

    pending_value = _runtime_detail_value(
        spray_rt,
        actuator,
        runtime_fresh=runtime_fresh,
        primary="pending_value",
        actuator_key="pending_value",
    )
    last_command_error = _runtime_detail_value(
        spray_rt,
        actuator,
        runtime_fresh=runtime_fresh,
        primary="actuator_failure_state",
        fallback="last_command_failure",
        actuator_key="last_command_failure",
    )
    if runtime_fresh and not last_command_error:
        last_command_error = failure or None

    detail = {
        "spraying": spraying,
        "marking_state": marking_state,
        "spray_state": spray_state,
        "desired_on": desired_on if runtime_fresh else None,
        "pending_command": pending if runtime_fresh else None,
        "pending_value": pending_value,
        "accepted_command_on": accepted_on if runtime_fresh else None,
        "accepted_command_off": accepted_command_off if runtime_fresh else None,
        "off_acknowledged": off_ack if runtime_fresh else None,
        "physical_feedback_supported": (
            physical_feedback_supported if runtime_fresh else None
        ),
        "physical_confirmation_available": (
            physical_feedback_supported if runtime_fresh else None
        ),
        "physical_feedback_stale": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="physical_feedback_stale",
        ),
        "physical_feedback_age_s": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="physical_feedback_age_s",
        ),
        "physical_actuator_state": (
            physical_actuator_state if runtime_fresh else None
        ),
        "spray_runtime_status_age_s": spray_rt.get("status_age_s"),
        "spray_faulted": spray_faulted,
        "spray_recovery_required": spray_recovery_required,
        "last_spray_command_result": last_result,
        "last_spray_command_reason": last_reason,
        "commanded_on": commanded_on if runtime_fresh else None,
        "confirmed_off": spray_rt.get("confirmed_off") if runtime_fresh else None,
        "physical_off_confirmed": (
            spray_rt.get("physical_confirmed_off") if runtime_fresh else None
        ),
        "last_command_error": last_command_error,
        "spray_safety_reason": (
            spray_rt.get("gps_safety_reason") or spray_rt.get("safety_reason")
        ),
        "projection_s": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="projected_arc_length_m",
        ),
        "projection_segment_index": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="projection_segment_index",
        ),
        "projection_xtrack_error_m": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="projection_xtrack_error_m",
        ),
        "projection_jump_m": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="projection_jump_m",
        ),
        "projection_ambiguous": (
            spray_rt.get("projection_ambiguous") if runtime_fresh else None
        ),
        "ambiguity_clearance_confidence": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="ambiguity_clearance_confidence",
            fallback="projection_confidence",
        ),
        "along_track_speed_mps": (
            spray_rt.get("along_track_speed_mps") if runtime_fresh else None
        ),
        "cross_track_speed_mps": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="cross_track_speed_mps",
        ),
        "velocity_heading_error_deg": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="velocity_heading_error_deg",
        ),
        "current_run_length_m": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="current_run_length_m",
        ),
        "next_run_length_m": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="next_run_length_m",
        ),
        "next_boundary_kind": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="next_boundary_kind",
        ),
        "distance_to_boundary_m": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="distance_to_boundary_m",
            fallback="distance_to_next_boundary_m",
        ),
        "raw_on_lead_m": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="raw_on_lead_m",
        ),
        "bounded_on_lead_m": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="bounded_on_lead_m",
        ),
        "raw_off_lead_m": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="raw_off_lead_m",
        ),
        "bounded_off_lead_m": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="bounded_off_lead_m",
        ),
        "lead_clamped": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="lead_clamped",
        ),
        "lead_block_reason": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="lead_block_reason",
        ),
        "flow_mode": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="flow_mode",
        ),
        "target_flow": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="target_flow",
        ),
        "target_pwm": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="target_pwm",
        ),
        "command_pwm": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="command_pwm",
        ),
        "pwm_ramp_limited": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="pwm_ramp_limited",
        ),
        "flow_capacity_limited": (
            spray_rt.get("flow_under_capacity") if runtime_fresh else None
        ),
        "flow_under_capacity": (
            spray_rt.get("flow_under_capacity") if runtime_fresh else None
        ),
        "flow_clamp_reason": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="flow_clamp_reason",
        ),
        "geometry_hash": spray_rt.get("geometry_hash") if runtime_fresh else None,
        "runtime_spray_geometry_hash": _runtime_detail_value(
            spray_rt,
            actuator,
            runtime_fresh=runtime_fresh,
            primary="runtime_spray_geometry_hash",
        ),
        "gps_safety_ok": spray_rt.get("gps_safety_ok"),
        "manual_resume_required": manual_resume,
        "vehicle_state_age_s": (
            spray_rt.get("vehicle_state_age_s") if runtime_fresh else None
        ),
        "vehicle_state_stale": (
            spray_rt.get("vehicle_state_stale") if runtime_fresh else None
        ),
        "vehicle_state_block_reason": (
            spray_rt.get("vehicle_state_block_reason") if runtime_fresh else None
        ),
        "geometry_spray_request": geometry_spray_request,
        "dry_run_active": dry_run_active,
    }
    detail.update(_mission_dash_fields(mission_dash))
    return detail


def _truthy_status_field(
    status: dict[str, Any],
    actuator: dict[str, Any],
    name: str,
    default: Any = False,
) -> bool:
    return bool(status.get(name, actuator.get(name, default)))


_CLOCK_TOLERANCE_S = 1e-3


def _physical_feedback_field(
    status: dict[str, Any], actuator: dict[str, Any], name: str, default: Any = None
) -> Any:
    if name in status:
        return status.get(name)
    return actuator.get(name, default)


def _physical_feedback_is_stale(
    status: dict[str, Any], actuator: dict[str, Any]
) -> tuple[bool, str]:
    physical_available = _truthy_status_field(
        status, actuator, "physical_confirmation_available", False
    )
    if not physical_available:
        return False, ""

    for name in (
        "physical_confirmation_stale",
        "physical_actuator_state_stale",
    ):
        if _truthy_status_field(status, actuator, name, False):
            return True, name

    timestamp_s = _physical_feedback_field(
        status, actuator, "physical_feedback_timestamp_monotonic_s"
    )
    age_s = _physical_feedback_field(status, actuator, "physical_feedback_age_s")
    timeout_s = _physical_feedback_field(
        status, actuator, "physical_feedback_timeout_s", 1.0
    )
    stale_flag = _physical_feedback_field(status, actuator, "physical_feedback_stale")

    if timestamp_s is None:
        return True, "physical_feedback_timestamp_missing"
    if age_s is None:
        return True, "physical_feedback_age_missing"
    try:
        timestamp_f = float(timestamp_s)
        age_f = float(age_s)
        timeout_f = float(timeout_s)
    except (TypeError, ValueError):
        return True, "physical_feedback_timestamp_or_age_invalid"
    if not math.isfinite(timestamp_f):
        return True, "physical_feedback_timestamp_non_finite"
    if not math.isfinite(age_f):
        return True, "physical_feedback_age_non_finite"
    if not math.isfinite(timeout_f) or timeout_f <= 0.0:
        return True, "physical_feedback_timeout_invalid"
    if age_f < -_CLOCK_TOLERANCE_S:
        return True, "physical_feedback_age_negative"
    if age_f > timeout_f + 1e-9:
        return True, "physical_feedback_age_exceeds_timeout"
    if stale_flag is True:
        return True, "physical_feedback_stale"
    return False, ""


def _physical_feedback_fault(
    status: dict[str, Any], actuator: dict[str, Any], state: str
) -> tuple[bool, str]:
    for name in (
        "physical_feedback_faulted",
        "physical_confirmation_faulted",
        "physical_actuator_faulted",
    ):
        if _truthy_status_field(status, actuator, name, False):
            return True, name
    if state in {"FAULT", "FAULTED", "ERROR"}:
        return True, f"physical actuator state {state}"
    return False, ""


def _evaluate_off_status(status: dict[str, Any]) -> OffStatusEvaluation:
    actuator = status.get("actuator")
    if not isinstance(actuator, dict):
        actuator = {}

    if bool(status.get("status_stale", True)):
        physical_available = _truthy_status_field(
            status, actuator, "physical_confirmation_available", False
        )
        return OffStatusEvaluation(
            accepted=False,
            reason="spray runtime status stale",
            command_off_acknowledged=False,
            physical_confirmation_available=physical_available,
            physical_off_confirmed=False,
            recovery_required=physical_available,
        )

    pending = status.get("pending_command", actuator.get("pending", False))
    if bool(pending):
        pending_on = status.get("pending_command_on", actuator.get("pending_on"))
        return OffStatusEvaluation(
            accepted=False,
            reason=f"spray command pending ({pending_on})",
            command_off_acknowledged=False,
            physical_confirmation_available=_truthy_status_field(
                status, actuator, "physical_confirmation_available", False
            ),
            physical_off_confirmed=False,
            recovery_required=False,
        )

    accepted_on = status.get("accepted_command_on", actuator.get("accepted_on"))
    if accepted_on is True:
        return OffStatusEvaluation(
            accepted=False,
            reason="spray accepted command is ON",
            command_off_acknowledged=False,
            physical_confirmation_available=_truthy_status_field(
                status, actuator, "physical_confirmation_available", False
            ),
            physical_off_confirmed=False,
            recovery_required=False,
        )

    if bool(status.get("commanded_on", actuator.get("commanded_on", False))):
        return OffStatusEvaluation(
            accepted=False,
            reason="spray commanded_on is true",
            command_off_acknowledged=False,
            physical_confirmation_available=_truthy_status_field(
                status, actuator, "physical_confirmation_available", False
            ),
            physical_off_confirmed=False,
            recovery_required=False,
        )

    off_ack = status.get(
        "off_acknowledged",
        status.get("confirmed_off", actuator.get("off_confirmed", False)),
    )
    if not bool(off_ack):
        return OffStatusEvaluation(
            accepted=False,
            reason="spray OFF not acknowledged",
            command_off_acknowledged=False,
            physical_confirmation_available=_truthy_status_field(
                status, actuator, "physical_confirmation_available", False
            ),
            physical_off_confirmed=False,
            recovery_required=False,
        )

    physical_available = _truthy_status_field(
        status, actuator, "physical_confirmation_available", False
    )
    if not physical_available:
        return OffStatusEvaluation(
            accepted=True,
            reason="accepted OFF (command-level; physical feedback unavailable)",
            command_off_acknowledged=True,
            physical_confirmation_available=False,
            physical_off_confirmed=False,
            recovery_required=False,
            details={
                "physical_actuator_state": status.get(
                    "physical_actuator_state", "UNAVAILABLE"
                ),
                "physical_confirmation_source": status.get(
                    "physical_confirmation_source",
                    actuator.get("physical_confirmation_source", "none"),
                ),
            },
        )

    state = str(status.get("physical_actuator_state", "UNKNOWN")).upper()
    stale, stale_reason = _physical_feedback_is_stale(status, actuator)
    if stale:
        return OffStatusEvaluation(
            accepted=False,
            reason=f"physical feedback stale: {stale_reason}",
            command_off_acknowledged=True,
            physical_confirmation_available=True,
            physical_off_confirmed=False,
            recovery_required=True,
            details={"physical_actuator_state": state},
        )

    faulted, fault_reason = _physical_feedback_fault(status, actuator, state)
    if faulted:
        return OffStatusEvaluation(
            accepted=False,
            reason=f"physical feedback faulted: {fault_reason}",
            command_off_acknowledged=True,
            physical_confirmation_available=True,
            physical_off_confirmed=False,
            recovery_required=True,
            details={"physical_actuator_state": state},
        )

    if state in {"UNKNOWN", "UNAVAILABLE", "NONE", ""}:
        return OffStatusEvaluation(
            accepted=False,
            reason=f"physical actuator state not definitive OFF: {state}",
            command_off_acknowledged=True,
            physical_confirmation_available=True,
            physical_off_confirmed=False,
            recovery_required=True,
            details={"physical_actuator_state": state},
        )

    physical_on = status.get("physical_on", actuator.get("physical_on"))
    if physical_on is True or state == "ON":
        return OffStatusEvaluation(
            accepted=False,
            reason="physical actuator state is ON",
            command_off_acknowledged=True,
            physical_confirmation_available=True,
            physical_off_confirmed=False,
            recovery_required=True,
            details={
                "physical_actuator_state": state,
                "physical_on": physical_on,
            },
        )

    source = str(
        status.get(
            "physical_confirmation_source",
            actuator.get("physical_confirmation_source", "none"),
        )
        or "none"
    ).strip().lower()
    if source in {"", "none", "unavailable", "unknown"}:
        return OffStatusEvaluation(
            accepted=False,
            reason="physical feedback source unavailable",
            command_off_acknowledged=True,
            physical_confirmation_available=True,
            physical_off_confirmed=False,
            recovery_required=True,
            details={"physical_actuator_state": state, "physical_source": source},
        )

    explicit_confirmed = bool(status.get("physical_confirmed_off", False))
    physical_off = bool(explicit_confirmed or physical_on is False or state == "OFF")
    if not physical_off:
        return OffStatusEvaluation(
            accepted=False,
            reason=f"physical actuator state not definitive OFF: {state}",
            command_off_acknowledged=True,
            physical_confirmation_available=True,
            physical_off_confirmed=False,
            recovery_required=True,
            details={
                "physical_actuator_state": state,
                "physical_on": physical_on,
                "physical_confirmed_off": explicit_confirmed,
            },
        )

    return OffStatusEvaluation(
        accepted=True,
        reason="accepted OFF with physical confirmation",
        command_off_acknowledged=True,
        physical_confirmation_available=True,
        physical_off_confirmed=True,
        recovery_required=False,
        details={
            "physical_actuator_state": state,
            "physical_on": physical_on,
            "physical_confirmed_off": explicit_confirmed,
            "physical_source": source,
        },
    )


def _accepted_off_status(status: dict[str, Any]) -> tuple[bool, str]:
    evaluation = _evaluate_off_status(status)
    return evaluation.accepted, evaluation.reason


@dataclass
class SprayOnAckResult:
    success: bool
    timeout: bool
    requested_on: bool
    pending_on: bool
    accepted_on: bool
    commanded_on: bool
    message: str
    failure_reason: str = ""
    recovery_required: bool = False
    status: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "success": self.success,
            "timeout": self.timeout,
            "requested_on": self.requested_on,
            "pending_on": self.pending_on,
            "accepted_on": self.accepted_on,
            "commanded_on": self.commanded_on,
            "message": self.message,
            "failure_reason": self.failure_reason,
            "recovery_required": self.recovery_required,
            "status": dict(self.status),
        }


async def wait_for_spray_on_acknowledged(
    ros_node,
    *,
    timeout_s: float = 2.0,
    poll_interval_s: float = 0.05,
) -> SprayOnAckResult:
    """Wait for fresh runtime acknowledgement of a manual spray ON request."""
    if ros_node is None or not hasattr(ros_node, "get_spray_runtime_status"):
        return SprayOnAckResult(
            success=False,
            timeout=False,
            requested_on=True,
            pending_on=False,
            accepted_on=False,
            commanded_on=False,
            message="spray runtime status unavailable",
            failure_reason="spray runtime status unavailable",
        )

    deadline = time.monotonic() + max(0.0, float(timeout_s))
    last_status: dict[str, Any] = {}
    last_reason = "not checked"
    while time.monotonic() <= deadline:
        try:
            last_status = dict(ros_node.get_spray_runtime_status())
        except Exception as exc:
            return SprayOnAckResult(
                success=False,
                timeout=False,
                requested_on=True,
                pending_on=False,
                accepted_on=False,
                commanded_on=False,
                message=f"spray runtime status failed: {exc}",
                failure_reason=f"spray runtime status failed: {exc}",
                recovery_required=True,
            )

        if bool(last_status.get("status_stale", True)):
            last_reason = "spray runtime status stale"
        else:
            actuator = last_status.get("actuator")
            if not isinstance(actuator, dict):
                actuator = {}
            pending = bool(
                last_status.get("pending_command", actuator.get("pending", False))
            )
            pending_on = last_status.get(
                "pending_command_on", actuator.get("pending_on")
            )
            accepted_on = bool(
                last_status.get(
                    "accepted_command_on", actuator.get("accepted_on", False)
                )
            )
            commanded_on = bool(
                last_status.get("commanded_on", actuator.get("commanded_on", False))
            )
            failure = str(
                last_status.get("actuator_failure_state")
                or actuator.get("last_command_failure")
                or ""
            ).strip()
            if failure:
                return SprayOnAckResult(
                    success=False,
                    timeout=False,
                    requested_on=True,
                    pending_on=bool(pending and pending_on is not False),
                    accepted_on=accepted_on,
                    commanded_on=commanded_on,
                    message=f"spray ON rejected: {failure}",
                    failure_reason=failure,
                    status=last_status,
                )
            if accepted_on and commanded_on:
                return SprayOnAckResult(
                    success=True,
                    timeout=False,
                    requested_on=True,
                    pending_on=False,
                    accepted_on=True,
                    commanded_on=True,
                    message="spray ON accepted",
                    status=last_status,
                )
            if pending and pending_on is False:
                last_reason = "spray OFF pending after ON request"
            elif pending:
                last_reason = "spray ON pending"
            else:
                last_reason = "spray ON not yet accepted"
        await asyncio.sleep(max(0.01, float(poll_interval_s)))

    actuator = last_status.get("actuator")
    if not isinstance(actuator, dict):
        actuator = {}
    pending = bool(last_status.get("pending_command", actuator.get("pending", False)))
    pending_on = last_status.get("pending_command_on", actuator.get("pending_on"))
    accepted_on = bool(
        last_status.get("accepted_command_on", actuator.get("accepted_on", False))
    )
    commanded_on = bool(
        last_status.get("commanded_on", actuator.get("commanded_on", False))
    )
    stale = bool(last_status.get("status_stale", True))
    return SprayOnAckResult(
        success=False,
        timeout=True,
        requested_on=True,
        pending_on=bool(pending and pending_on is not False),
        accepted_on=accepted_on,
        commanded_on=commanded_on,
        message=f"spray ON acknowledgement timed out: {last_reason}",
        failure_reason=last_reason,
        recovery_required=stale,
        status=last_status,
    )


@dataclass
class VehicleTransitionResult:
    success: bool
    message: str
    transition_ok: bool
    spray_off_confirmed: bool | None
    spray_off_result: dict[str, Any] | None
    recovery_required: bool = False
    degraded: bool = False

    def as_socket_payload(self, *, transition_key: str, transition_value: Any) -> dict[str, Any]:
        return {
            "success": self.success,
            "message": self.message,
            transition_key: transition_value,
            "degraded": self.degraded,
            "recovery_required": self.recovery_required,
            "spray_off_confirmed": self.spray_off_confirmed,
            "spray_off_result": self.spray_off_result,
        }


async def disarm_with_spray_safety(ros_node) -> VehicleTransitionResult:
    """Disarm only after confirmed spray OFF when the runtime is live."""
    spray_off = await force_spray_off_confirmed(ros_node, timeout_s=2.0)
    ok, why = await ros_node.arm_async(False)
    spray_off_confirmed = (
        spray_off.success
        if spray_off.attempted and spray_off.live
        else None
    )
    spray_blocked = spray_off_blocks_success(spray_off)
    if not ok:
        return VehicleTransitionResult(
            success=False,
            message=f"Disarm FAILED: {why}",
            transition_ok=False,
            spray_off_confirmed=spray_off_confirmed,
            spray_off_result=spray_off.as_dict(),
            recovery_required=bool(spray_off.recovery_required),
            degraded=spray_blocked,
        )
    if spray_blocked:
        return VehicleTransitionResult(
            success=False,
            message=f"Disarmed but spray OFF not confirmed: {spray_off.message}",
            transition_ok=True,
            spray_off_confirmed=False,
            spray_off_result=spray_off.as_dict(),
            recovery_required=bool(spray_off.recovery_required),
            degraded=True,
        )
    return VehicleTransitionResult(
        success=True,
        message="Disarmed OK",
        transition_ok=True,
        spray_off_confirmed=spray_off_confirmed,
        spray_off_result=spray_off.as_dict(),
    )


async def set_mode_with_spray_safety(
    ros_node,
    *,
    target_mode: str,
    current_mode: str | None,
) -> VehicleTransitionResult:
    """Change flight mode with terminal spray OFF policy when leaving OFFBOARD."""
    spray_off = None
    if requires_spray_off_before_mode_change(current_mode, target_mode):
        spray_off = await force_spray_off_confirmed(ros_node, timeout_s=2.0)
    ok, why = await ros_node.set_mode_async(target_mode)
    spray_off_confirmed = (
        spray_off.success
        if spray_off is not None and spray_off.attempted and spray_off.live
        else None
    )
    spray_blocked = spray_off is not None and spray_off_blocks_success(spray_off)
    if not ok:
        return VehicleTransitionResult(
            success=False,
            message=f"Mode {target_mode} FAILED: {why}",
            transition_ok=False,
            spray_off_confirmed=spray_off_confirmed,
            spray_off_result=spray_off.as_dict() if spray_off is not None else None,
            recovery_required=bool(spray_off.recovery_required) if spray_off else False,
            degraded=spray_blocked,
        )
    if spray_blocked:
        return VehicleTransitionResult(
            success=False,
            message=(
                f"Mode {target_mode} set but spray OFF not confirmed: "
                f"{spray_off.message}"
            ),
            transition_ok=True,
            spray_off_confirmed=False,
            spray_off_result=spray_off.as_dict(),
            recovery_required=bool(spray_off.recovery_required),
            degraded=True,
        )
    return VehicleTransitionResult(
        success=True,
        message=f"Mode {target_mode} set",
        transition_ok=True,
        spray_off_confirmed=spray_off_confirmed,
        spray_off_result=spray_off.as_dict() if spray_off is not None else None,
    )


async def force_spray_off_confirmed(
    ros_node,
    *,
    timeout_s: float = 2.0,
    poll_interval_s: float = 0.05,
    check_cancel: Callable[[], None] | None = None,
) -> SprayOffResult:
    """Command spray OFF and wait for fresh OFF proof.

    Without physical feedback this confirms command-accepted OFF only. When the
    spray node advertises physical confirmation support, command ACK is not
    enough: physical state must also be fresh and definitively OFF.
    """
    if ros_node is None or not hasattr(ros_node, "get_spray_runtime_status"):
        return SprayOffResult(
            success=False,
            attempted=False,
            timeout=False,
            fault=True,
            live=False,
            message="spray runtime status unavailable",
            failure_reason="spray runtime status unavailable",
            recovery_required=False,
        )

    initial_status: dict[str, Any] = {}
    live = False
    try:
        initial_status = dict(ros_node.get_spray_runtime_status())
        live = not bool(initial_status.get("status_stale", True))
    except Exception as exc:
        return SprayOffResult(
            success=False,
            attempted=False,
            timeout=False,
            fault=True,
            live=False,
            message=f"spray runtime status failed: {exc}",
            details={"exception": str(exc)},
            failure_reason=f"spray runtime status failed: {exc}",
            recovery_required=False,
        )

    try:
        if hasattr(ros_node, "publish_spray_manual"):
            ros_node.publish_spray_manual(False)
        if hasattr(ros_node, "cancel_spray_dwell_async"):
            await ros_node.cancel_spray_dwell_async()
    except Exception as exc:
        log.exception("force spray OFF command failed")
        return SprayOffResult(
            success=False,
            attempted=True,
            timeout=False,
            fault=True,
            live=live,
            message=f"spray OFF command failed: {exc}",
            details={"initial_status": initial_status, "exception": str(exc)},
            failure_reason=f"spray OFF command failed: {exc}",
            recovery_required=live,
        )

    deadline = time.monotonic() + max(0.0, float(timeout_s))
    last_status = initial_status
    last_reason = "not checked"
    while time.monotonic() <= deadline:
        if check_cancel is not None:
            check_cancel()
        try:
            last_status = dict(ros_node.get_spray_runtime_status())
            live = live or not bool(last_status.get("status_stale", True))
        except Exception as exc:
            return SprayOffResult(
                success=False,
                attempted=True,
                timeout=False,
                fault=True,
                live=live,
                message=f"spray runtime status failed: {exc}",
                details={"initial_status": initial_status, "exception": str(exc)},
                failure_reason=f"spray runtime status failed: {exc}",
                recovery_required=live,
            )

        evaluation = _evaluate_off_status(last_status)
        last_reason = evaluation.reason
        if evaluation.accepted:
            return SprayOffResult(
                success=True,
                attempted=True,
                timeout=False,
                fault=False,
                live=True,
                message=evaluation.reason,
                details={"status": last_status, "evaluation": evaluation.details},
                command_off_acknowledged=evaluation.command_off_acknowledged,
                physical_confirmation_available=(
                    evaluation.physical_confirmation_available
                ),
                physical_off_confirmed=evaluation.physical_off_confirmed,
                recovery_required=False,
                failure_reason="",
                confirmation_level=evaluation.confirmation_level,
            )
        await asyncio.sleep(max(0.01, float(poll_interval_s)))

    final_evaluation = _evaluate_off_status(last_status)
    return SprayOffResult(
        success=False,
        attempted=True,
        timeout=True,
        fault=False,
        live=live,
        message=f"spray OFF confirmation timed out: {last_reason}",
        details={
            "initial_status": initial_status,
            "last_status": last_status,
            "evaluation": final_evaluation.details,
        },
        command_off_acknowledged=final_evaluation.command_off_acknowledged,
        physical_confirmation_available=(
            final_evaluation.physical_confirmation_available
        ),
        physical_off_confirmed=final_evaluation.physical_off_confirmed,
        recovery_required=bool(live and final_evaluation.recovery_required),
        failure_reason=last_reason,
        confirmation_level="none",
    )


async def cleanup_mission_start_failure(ros_node, offboard_ctrl) -> dict[str, Any]:
    """Confirmed spray OFF then soft-stop after a failed point-mission start."""
    spray_off = await force_spray_off_confirmed(ros_node, timeout_s=2.0)
    if offboard_ctrl is not None:
        try:
            await offboard_ctrl.stop_async()
        except Exception:
            log.exception("mission start cleanup: stop_async failed")
    return spray_off.as_dict()
