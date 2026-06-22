"""Validated spray-mode configuration models (ROS-independent)."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from typing import Any


class SprayMode(str, Enum):
    CONTINUOUS = "continuous"
    DASH = "dash"
    POINT = "point"

    @classmethod
    def parse(cls, value: Any) -> SprayMode:
        if isinstance(value, cls):
            return value
        text = str(value).strip().lower()
        try:
            return cls(text)
        except ValueError as exc:
            raise ValueError(
                f"invalid spray_mode {value!r}; expected "
                f"{cls.CONTINUOUS.value}, {cls.DASH.value}, or {cls.POINT.value}"
            ) from exc


class DashPhaseReset(str, Enum):
    PER_MARK_REGION = "per_mark_region"
    CONTINUOUS = "continuous"

    @classmethod
    def parse(cls, value: Any) -> DashPhaseReset:
        if isinstance(value, cls):
            return value
        text = str(value).strip().lower()
        try:
            return cls(text)
        except ValueError as exc:
            raise ValueError(
                f"invalid dash_phase_reset {value!r}; expected "
                f"{cls.PER_MARK_REGION.value} or {cls.CONTINUOUS.value}"
            ) from exc


@dataclass(frozen=True)
class ContinuousSprayParams:
    solenoid_open_delay_s: float = 0.10
    solenoid_close_delay_s: float = 0.05
    on_overspray_margin_m: float = 0.02
    off_overspray_margin_m: float = 0.0
    min_spray_speed_mps: float = 0.05
    max_xtrack_error_m: float = 0.10
    nozzle_forward_offset_m: float = 0.0
    nozzle_lateral_offset_m: float = 0.0


@dataclass(frozen=True)
class DashSprayParams:
    on_distance_m: float = 0.30
    off_distance_m: float = 0.30
    phase_reset: DashPhaseReset = DashPhaseReset.PER_MARK_REGION


@dataclass(frozen=True)
class PointSprayParams:
    default_dwell_s: float = 2.0
    arrival_tolerance_m: float = 0.05
    settle_time_s: float = 0.10
    leg_timeout_s: float = 120.0
    settle_speed_mps: float = 0.05
    settle_yaw_rate_rad_s: float = 0.05


@dataclass(frozen=True)
class SafetySprayParams:
    require_offboard: bool = True
    debounce_samples: int = 3
    pose_timeout_s: float = 0.5
    velocity_timeout_s: float = 0.5


@dataclass(frozen=True)
class SprayConfiguration:
    mode: SprayMode = SprayMode.CONTINUOUS
    continuous: ContinuousSprayParams = ContinuousSprayParams()
    dash: DashSprayParams = DashSprayParams()
    point: PointSprayParams = PointSprayParams()
    safety: SafetySprayParams = SafetySprayParams()
    revision: int = 0
    mission_id: str = ""

    def with_revision(self, revision: int, mission_id: str = "") -> SprayConfiguration:
        return replace(self, revision=revision, mission_id=mission_id)


def _finite_positive(name: str, value: Any, *, allow_zero: bool = False) -> float:
    try:
        num = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number") from exc
    if not (num == num):  # NaN
        raise ValueError(f"{name} must be finite")
    if allow_zero:
        if num < 0.0:
            raise ValueError(f"{name} must be >= 0")
    elif num <= 0.0:
        raise ValueError(f"{name} must be > 0")
    return num


def _finite_non_negative(name: str, value: Any) -> float:
    return _finite_positive(name, value, allow_zero=True)


def validate_spray_configuration(
    raw: dict[str, Any],
    *,
    previous: SprayConfiguration | None = None,
) -> SprayConfiguration:
    """Validate a mission-bound spray configuration snapshot."""
    mode = SprayMode.parse(raw.get("spray_mode", SprayMode.CONTINUOUS.value))

    continuous = ContinuousSprayParams(
        solenoid_open_delay_s=_finite_non_negative(
            "solenoid_open_delay_s", raw.get("solenoid_open_delay_s", 0.10)
        ),
        solenoid_close_delay_s=_finite_non_negative(
            "solenoid_close_delay_s", raw.get("solenoid_close_delay_s", 0.05)
        ),
        on_overspray_margin_m=_finite_non_negative(
            "on_overspray_margin_m", raw.get("on_overspray_margin_m", 0.02)
        ),
        off_overspray_margin_m=_finite_non_negative(
            "off_overspray_margin_m", raw.get("off_overspray_margin_m", 0.0)
        ),
        min_spray_speed_mps=_finite_non_negative(
            "min_spray_speed_mps", raw.get("min_spray_speed_mps", 0.05)
        ),
        max_xtrack_error_m=_finite_positive(
            "max_xtrack_error_m", raw.get("max_xtrack_error_m", 0.10)
        ),
        nozzle_forward_offset_m=float(raw.get("nozzle_forward_offset_m", 0.0)),
        nozzle_lateral_offset_m=float(raw.get("nozzle_lateral_offset_m", 0.0)),
    )

    dash = DashSprayParams(
        on_distance_m=_finite_non_negative(
            "dash_on_distance_m", raw.get("dash_on_distance_m", 0.30)
        ),
        off_distance_m=_finite_non_negative(
            "dash_off_distance_m", raw.get("dash_off_distance_m", 0.30)
        ),
        phase_reset=DashPhaseReset.parse(
            raw.get("dash_phase_reset", DashPhaseReset.PER_MARK_REGION.value)
        ),
    )
    if mode == SprayMode.DASH:
        if dash.on_distance_m <= 0.0 and dash.off_distance_m <= 0.0:
            raise ValueError(
                "dash mode requires dash_on_distance_m or dash_off_distance_m > 0"
            )

    point = PointSprayParams(
        default_dwell_s=_finite_positive(
            "point_default_dwell_s", raw.get("point_default_dwell_s", 2.0)
        ),
        arrival_tolerance_m=_finite_positive(
            "point_arrival_tolerance_m", raw.get("point_arrival_tolerance_m", 0.05)
        ),
        settle_time_s=_finite_non_negative(
            "point_settle_time_s", raw.get("point_settle_time_s", 0.10)
        ),
        leg_timeout_s=_finite_positive(
            "point_leg_timeout_s", raw.get("point_leg_timeout_s", 120.0)
        ),
        settle_speed_mps=_finite_non_negative(
            "point_settle_speed_mps", raw.get("point_settle_speed_mps", 0.05)
        ),
        settle_yaw_rate_rad_s=_finite_non_negative(
            "point_settle_yaw_rate_rad_s",
            raw.get("point_settle_yaw_rate_rad_s", 0.05),
        ),
    )

    debounce = int(raw.get("debounce_samples", 3))
    if debounce < 1 or debounce > 20:
        raise ValueError("debounce_samples must be in [1, 20]")

    safety = SafetySprayParams(
        require_offboard=bool(raw.get("require_offboard", True)),
        debounce_samples=debounce,
        pose_timeout_s=_finite_non_negative(
            "pose_timeout_s", raw.get("pose_timeout_s", 0.5)
        ),
        velocity_timeout_s=_finite_non_negative(
            "velocity_timeout_s", raw.get("velocity_timeout_s", 0.5)
        ),
    )

    revision = int(raw.get("configuration_revision", 0))
    mission_id = str(raw.get("mission_id", "") or "")

    config = SprayConfiguration(
        mode=mode,
        continuous=continuous,
        dash=dash,
        point=point,
        safety=safety,
        revision=revision,
        mission_id=mission_id,
    )

    if previous is not None and mode != previous.mode:
        # Explicit mode change is always accepted when validated as a whole.
        return config
    return config


def configuration_to_param_dict(config: SprayConfiguration) -> dict[str, Any]:
    """Map a validated configuration to spray_controller ROS parameters."""
    return {
        "spray_mode": config.mode.value,
        "solenoid_open_delay_s": config.continuous.solenoid_open_delay_s,
        "solenoid_close_delay_s": config.continuous.solenoid_close_delay_s,
        "on_overspray_margin_m": config.continuous.on_overspray_margin_m,
        "off_overspray_margin_m": config.continuous.off_overspray_margin_m,
        "min_spray_speed_mps": config.continuous.min_spray_speed_mps,
        "max_xtrack_error_m": config.continuous.max_xtrack_error_m,
        "nozzle_forward_offset_m": config.continuous.nozzle_forward_offset_m,
        "nozzle_lateral_offset_m": config.continuous.nozzle_lateral_offset_m,
        "dash_on_distance_m": config.dash.on_distance_m,
        "dash_off_distance_m": config.dash.off_distance_m,
        "dash_phase_reset": config.dash.phase_reset.value,
        "point_default_dwell_s": config.point.default_dwell_s,
        "point_arrival_tolerance_m": config.point.arrival_tolerance_m,
        "point_settle_time_s": config.point.settle_time_s,
        "point_leg_timeout_s": config.point.leg_timeout_s,
        "point_settle_speed_mps": config.point.settle_speed_mps,
        "point_settle_yaw_rate_rad_s": config.point.settle_yaw_rate_rad_s,
        "require_offboard": config.safety.require_offboard,
        "debounce_samples": config.safety.debounce_samples,
        "pose_timeout_s": config.safety.pose_timeout_s,
        "velocity_timeout_s": config.safety.velocity_timeout_s,
        "configuration_revision": config.revision,
        "mission_config_mission_id": config.mission_id,
    }


def staged_spray_defaults() -> dict[str, Any]:
    """Default spray fields for staged mission artifacts."""
    cfg = SprayConfiguration()
    return {
        "spray_mode": cfg.mode.value,
        "dash_on_distance_m": cfg.dash.on_distance_m,
        "dash_off_distance_m": cfg.dash.off_distance_m,
        "dash_phase_reset": cfg.dash.phase_reset.value,
        "point_default_dwell_s": cfg.point.default_dwell_s,
        "point_arrival_tolerance_m": cfg.point.arrival_tolerance_m,
        "point_settle_time_s": cfg.point.settle_time_s,
        "point_leg_timeout_s": cfg.point.leg_timeout_s,
        "point_settle_speed_mps": cfg.point.settle_speed_mps,
        "point_settle_yaw_rate_rad_s": cfg.point.settle_yaw_rate_rad_s,
        "point_mission_points": [],
    }


def parse_staged_spray_config(staged: dict[str, Any]) -> SprayConfiguration:
    """Parse spray configuration from a staged mission artifact."""
    raw = staged_spray_defaults()
    for key in raw:
        if key in staged:
            raw[key] = staged[key]
    raw["mission_id"] = str(staged.get("mission_id", "") or "")
    raw["configuration_revision"] = int(staged.get("configuration_revision", 0))
    return validate_spray_configuration(raw)