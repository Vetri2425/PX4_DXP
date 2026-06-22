"""Mode dispatch helpers for spray_controller_node (ROS-independent logic)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

from spray_config import DashPhaseReset, SprayConfiguration, SprayMode
from spray_dash import apply_dash_pattern

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
    speed_mps: float,
    safety_ok: bool,
    safety_reason: str,
    config: SprayConfiguration,
) -> "SprayDecision":
    from spray_path_model import make_spray_decision, nozzle_position_ned
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
    return make_spray_decision(
        model=model,
        nozzle_n=nozzle_n,
        nozzle_e=nozzle_e,
        speed_mps=speed_mps,
        safety_ok=safety_ok,
        safety_reason=safety_reason,
        solenoid_open_delay_s=config.continuous.solenoid_open_delay_s,
        solenoid_close_delay_s=config.continuous.solenoid_close_delay_s,
        on_overspray_margin_m=config.continuous.on_overspray_margin_m,
        off_overspray_margin_m=config.continuous.off_overspray_margin_m,
        max_xtrack_error_m=config.continuous.max_xtrack_error_m,
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
    )


def auto_safety_status(
    *,
    config: SprayConfiguration,
    armed: bool,
    mode: str,
    path_model: Optional["SprayPathModel"],
    pose_fresh: bool,
    speed: float,
    velocity_fresh: bool,
    dwell_active: bool,
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
    min_speed = config.continuous.min_spray_speed_mps
    bypass_min_speed = config.mode == SprayMode.POINT and dwell_active
    if not bypass_min_speed and speed < min_speed:
        return False, "below min spray speed"
    return True, ""


def dash_phase_reset_from_string(value: str) -> DashPhaseReset:
    return DashPhaseReset.parse(value)
