"""Mode dispatch helpers for spray_controller_node (ROS-independent logic)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

from spray_config import (
    DashPhaseReset,
    FlowMode,
    SprayConfiguration,
    SprayMode,
    UnsafeSpeedBehavior,
    interpolate_speed_pwm,
    pwm_to_normalized_value,
)
from spray_dash import apply_dash_pattern
from spray_path_model import (
    SprayProjectionState,
    apply_distance_hysteresis,
    make_spray_decision,
    nozzle_position_ned,
    ramp_pwm,
)

if TYPE_CHECKING:
    from spray_controller_node import SprayDecision, SprayPathModel


@dataclass(frozen=True)
class DwellState:
    command_id: int
    mission_id: str
    point_index: int
    start_mono_ns: int
    expiry_mono_ns: int
    cancelled: bool = False

    @property
    def active(self) -> bool:
        return not self.cancelled


@dataclass
class ContinuousSprayRuntimeState:
    projection: SprayProjectionState
    last_spray_transition_s: float | None = None
    last_geometry_state: bool = False
    previous_pwm: float = 0.0
    last_tick_mono_s: float | None = None


def build_path_model_for_config(
    base_model: "SprayPathModel",
    config: SprayConfiguration,
) -> "SprayPathModel":
    if config.mode != SprayMode.DASH:
        return base_model
    return apply_dash_pattern(
        base_model,
        on_distance_m=config.dash.on_distance_m,
        off_distance_m=config.dash.off_distance_m,
        reset_mode=config.dash.phase_reset,
    )


def continuous_distance_decision(
    *,
    model: Optional["SprayPathModel"],
    pose_ned: Optional[tuple[float, float, float]],
    vel_ned: tuple[float, float],
    safety_ok: bool,
    safety_reason: str,
    config: SprayConfiguration,
    runtime_state: ContinuousSprayRuntimeState,
    dwell_active: bool = False,
    on_value: float = 1.0,
) -> "SprayDecision":
    nozzle_n: Optional[float] = None
    nozzle_e: Optional[float] = None
    if pose_ned is not None:
        nozzle_n, nozzle_e = nozzle_position_ned(
            pose_ned[0],
            pose_ned[1],
            pose_ned[2],
            config.continuous.nozzle_forward_offset_m,
            config.continuous.nozzle_lateral_offset_m,
        )

    table = config.calibration.speed_pwm_table
    table_max_speed = table[-1].speed_mps if table else config.continuous.max_spray_speed_mps

    decision = make_spray_decision(
        model=model,
        nozzle_n=nozzle_n,
        nozzle_e=nozzle_e,
        vel_n=vel_ned[0],
        vel_e=vel_ned[1],
        safety_ok=safety_ok,
        safety_reason=safety_reason,
        solenoid_open_delay_s=config.continuous.solenoid_open_delay_s,
        solenoid_close_delay_s=config.continuous.solenoid_close_delay_s,
        on_overspray_margin_m=config.continuous.on_overspray_margin_m,
        off_overspray_margin_m=config.continuous.off_overspray_margin_m,
        max_xtrack_error_m=config.continuous.max_xtrack_error_m,
        max_along_track_heading_error_deg=(
            config.continuous.max_along_track_heading_error_deg
        ),
        max_cross_track_speed_mps=config.continuous.max_cross_track_speed_mps,
        max_reverse_speed_tolerance_mps=(
            config.continuous.max_reverse_speed_tolerance_mps
        ),
        max_projection_jump_m=config.continuous.max_projection_jump_m,
        max_backward_projection_jump_m=config.continuous.max_backward_projection_jump_m,
        projection_ambiguity_distance_m=config.continuous.projection_ambiguity_distance_m,
        max_lead_distance_m=config.continuous.max_lead_distance_m,
        min_on_distance_m=config.continuous.min_on_distance_m,
        min_off_distance_m=config.continuous.min_off_distance_m,
        min_spray_speed_mps=config.continuous.min_spray_speed_mps,
        max_spray_speed_mps=config.continuous.max_spray_speed_mps,
        projection_state=runtime_state.projection,
        flow_mode=config.continuous.flow_mode,
        target_paint_density=config.calibration.target_paint_density,
        min_target_flow=config.continuous.min_target_flow,
        max_target_flow=config.continuous.max_target_flow,
        low_speed_anti_puddle_behavior=config.continuous.low_speed_anti_puddle_behavior,
        high_speed_underflow_behavior=config.continuous.high_speed_underflow_behavior,
        speed_pwm_table_max_speed=table_max_speed,
        dwell_active=dwell_active,
    )

    progress_forward = (
        decision.projection is not None
        and not decision.projection.ambiguous
        and decision.projection.backward_jump_m <= config.continuous.max_backward_projection_jump_m
    )
    if (
        decision.projection is not None
        and not decision.projection.ambiguous
        and progress_forward
    ):
        runtime_state.projection.valid = True
        runtime_state.projection.segment_index = decision.projection.segment_index
        runtime_state.projection.s = decision.projection.s

    hysteresis_desired = apply_distance_hysteresis(
        decision.geometry_desired,
        decision.projection.s if decision.projection is not None else 0.0,
        last_transition_s=runtime_state.last_spray_transition_s,
        last_geometry_state=runtime_state.last_geometry_state,
        min_on_distance_m=config.continuous.min_on_distance_m,
        min_off_distance_m=config.continuous.min_off_distance_m,
        safety_ok=decision.safety_ok,
        progress_forward=progress_forward,
    )

    if progress_forward and decision.projection is not None:
        if hysteresis_desired != runtime_state.last_geometry_state:
            runtime_state.last_spray_transition_s = decision.projection.s
            runtime_state.last_geometry_state = hysteresis_desired

    geometry_desired = bool(hysteresis_desired)
    desired = bool(hysteresis_desired and decision.safety_ok)
    limits = config.calibration.actuator_limits
    target_pwm = limits.off_pwm
    actuator_value = limits.off_value
    command_pwm = limits.off_pwm
    pwm_ramp_limited = False
    target_flow = 0.0

    flow_mode = config.continuous.flow_mode
    dry_run_active = flow_mode == FlowMode.DISABLED.value
    if dry_run_active:
        desired = False
        target_flow = 0.0
        runtime_state.previous_pwm = limits.off_pwm
        runtime_state.last_tick_mono_s = None
    elif desired:
        if flow_mode == FlowMode.FIXED.value:
            actuator_value = on_value
            target_pwm = limits.max_pwm
            command_pwm = target_pwm
        else:
            speed_for_pwm = max(0.0, decision.along_track_speed_mps)
            try:
                target_pwm = interpolate_speed_pwm(
                    speed_for_pwm,
                    table,
                    clamp=config.continuous.high_speed_underflow_behavior == "clamp",
                )
            except ValueError:
                target_pwm = limits.off_pwm
                desired = False
            actuator_value = pwm_to_normalized_value(target_pwm, limits)
            import time

            now_s = time.monotonic()
            dt_s = 0.02
            if runtime_state.last_tick_mono_s is not None:
                dt_s = max(0.0, now_s - runtime_state.last_tick_mono_s)
            runtime_state.last_tick_mono_s = now_s
            command_pwm, pwm_ramp_limited = ramp_pwm(
                target_pwm,
                runtime_state.previous_pwm,
                max_pwm_change_per_s=config.continuous.max_pwm_change_per_s,
                dt_s=dt_s,
            )
            if desired:
                actuator_value = pwm_to_normalized_value(command_pwm, limits)
                runtime_state.previous_pwm = command_pwm
    else:
        runtime_state.previous_pwm = limits.off_pwm
        runtime_state.last_tick_mono_s = None

    return type(decision)(
        desired=desired,
        geometry_desired=geometry_desired,
        safety_ok=decision.safety_ok,
        safety_reason=decision.safety_reason,
        projection=decision.projection,
        next_boundary=decision.next_boundary,
        distance_to_boundary_m=decision.distance_to_boundary_m,
        event=decision.event,
        debug=decision.debug,
        target_flow=target_flow if dry_run_active else decision.target_flow,
        target_pwm=target_pwm,
        actuator_value=actuator_value,
        along_track_speed_mps=decision.along_track_speed_mps,
        cross_track_speed_mps=decision.cross_track_speed_mps,
        velocity_heading_error_deg=decision.velocity_heading_error_deg,
        raw_on_lead_m=decision.raw_on_lead_m,
        bounded_on_lead_m=decision.bounded_on_lead_m,
        raw_off_lead_m=decision.raw_off_lead_m,
        bounded_off_lead_m=decision.bounded_off_lead_m,
        lead_clamped=decision.lead_clamped,
        lead_block_reason=decision.lead_block_reason,
        current_run_remaining_m=decision.current_run_remaining_m,
        next_run_length_m=decision.next_run_length_m,
        flow_mode=decision.flow_mode,
        raw_target_flow=decision.raw_target_flow,
        target_paint_density=decision.target_paint_density,
        flow_clamp_reason=decision.flow_clamp_reason,
        flow_under_capacity=decision.flow_under_capacity,
        command_pwm=command_pwm,
        pwm_ramp_limited=pwm_ramp_limited,
    )


def point_mode_decision(
    *,
    dwell: Optional[DwellState],
    now_mono_ns: int,
    safety_ok: bool,
    safety_reason: str,
) -> "SprayDecision":
    from spray_controller_node import SprayDecision
    geometry_desired = False
    if dwell is not None and dwell.active and now_mono_ns < dwell.expiry_mono_ns:
        geometry_desired = True
    desired = bool(geometry_desired and safety_ok)
    debug = [
        0.0,
        0.0,
        float("nan"),
        float("nan"),
        float("nan"),
        float("nan"),
        1.0 if geometry_desired else 0.0,
        float("nan"),
        float("inf"),
        1.0 if geometry_desired else 0.0,
        1.0 if safety_ok else 0.0,
        1.0 if desired else 0.0,
    ]
    return SprayDecision(
        desired=desired,
        geometry_desired=geometry_desired,
        safety_ok=safety_ok,
        safety_reason=safety_reason,
        projection=None,
        next_boundary=None,
        distance_to_boundary_m=float("inf"),
        event="dwell" if geometry_desired else "",
        debug=debug,
        target_flow=0.0,
        target_pwm=0.0,
        actuator_value=0.0,
    )


def auto_safety_status(
    *,
    config: SprayConfiguration,
    armed: bool,
    mode: str,
    path_model: Optional["SprayPathModel"],
    pose_fresh: bool,
    along_track_speed: float,
    velocity_fresh: bool,
    dwell_active: bool,
    check_speed_window: bool = True,
) -> tuple[bool, str]:
    if not armed:
        return False, "disarmed"
    if config.safety.require_offboard and mode != "OFFBOARD":
        return False, "not OFFBOARD"
    if config.mode != SprayMode.POINT and path_model is None:
        return False, "path not loaded"
    if not pose_fresh:
        return False, "pose stale"
    if not velocity_fresh:
        return False, "velocity stale"
    if not check_speed_window:
        return True, ""
    min_speed = config.continuous.min_spray_speed_mps
    max_speed = config.continuous.max_spray_speed_mps
    bypass_min_speed = config.mode == SprayMode.POINT and dwell_active
    if not bypass_min_speed and along_track_speed < min_speed:
        if config.continuous.unsafe_speed_behavior == UnsafeSpeedBehavior.CLAMP_PWM:
            return True, ""
        return False, "below min spray speed"
    if not bypass_min_speed and along_track_speed > max_speed:
        if config.continuous.unsafe_speed_behavior == UnsafeSpeedBehavior.CLAMP_PWM:
            return True, ""
        return False, "above max spray speed"
    return True, ""


def dash_phase_reset_from_string(value: str) -> DashPhaseReset:
    return DashPhaseReset.parse(value)