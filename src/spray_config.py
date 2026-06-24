"""Validated spray-mode configuration models (ROS-independent)."""

from __future__ import annotations

import json
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


class UnsafeSpeedBehavior(str, Enum):
    BLOCK_SPRAY = "BLOCK_SPRAY"
    CLAMP_PWM = "CLAMP_PWM"

    @classmethod
    def parse(cls, value: Any) -> UnsafeSpeedBehavior:
        if isinstance(value, cls):
            return value
        text = str(value).strip().upper()
        try:
            return cls(text)
        except ValueError as exc:
            raise ValueError(
                f"invalid unsafe_speed_behavior {value!r}; expected "
                "BLOCK_SPRAY or CLAMP_PWM"
            ) from exc


@dataclass(frozen=True)
class SpeedPwmPoint:
    speed_mps: float
    pwm: float


@dataclass(frozen=True)
class ActuatorLimits:
    min_pwm: float = 0.0
    max_pwm: float = 2200.0
    off_pwm: float = 0.0
    min_value: float = -1.0
    max_value: float = 1.0
    off_value: float = -1.0


@dataclass(frozen=True)
class HardwareCompensationHooks:
    pump_inertia_enabled: bool = False
    pwm_ramp_prediction_enabled: bool = False
    pressure_stabilization_enabled: bool = False
    temperature_viscosity_compensation_enabled: bool = False


@dataclass(frozen=True)
class CalibrationProfile:
    profile_id: str = "factory_default"
    version: int = 1
    target_paint_density: float = 1.0
    speed_pwm_table: tuple[SpeedPwmPoint, ...] = (
        SpeedPwmPoint(0.05, 1200.0),
        SpeedPwmPoint(0.35, 1800.0),
    )
    actuator_limits: ActuatorLimits = ActuatorLimits()
    timing_only_compatibility: bool = False
    hooks: HardwareCompensationHooks = HardwareCompensationHooks()


@dataclass(frozen=True)
class ContinuousSprayParams:
    solenoid_open_delay_s: float = 0.10
    solenoid_close_delay_s: float = 0.05
    on_overspray_margin_m: float = 0.02
    off_overspray_margin_m: float = 0.0
    min_spray_speed_mps: float = 0.05
    max_spray_speed_mps: float = 1.00
    unsafe_speed_behavior: UnsafeSpeedBehavior = UnsafeSpeedBehavior.BLOCK_SPRAY
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
    max_dwell_s: float = 60.0
    arrival_tolerance_m: float = 0.05
    settle_time_s: float = 0.10
    leg_timeout_s: float = 120.0
    settle_speed_mps: float = 0.05
    settle_yaw_rate_rad_s: float = 0.05
    leg_trajectory_mode: str = "two_point"
    leg_spacing_m: float = 0.08
    hold_drift_tolerance_m: float = 0.08
    hold_drift_policy: str = "fail"


@dataclass(frozen=True)
class GpsSurveyedSafetyParams:
    required_fix_type: int = 6
    global_position_max_age_ms: float = 500.0
    local_pose_max_age_ms: float = 500.0
    gps_fix_max_age_ms: float = 500.0
    max_pose_global_skew_ms: float = 100.0
    runtime_policy: str = "pause"
    resume_policy: str = "manual"
    recovery_stable_s: float = 2.0


@dataclass(frozen=True)
class SafetySprayParams:
    require_offboard: bool = True
    debounce_samples: int = 3
    pose_timeout_s: float = 0.5
    velocity_timeout_s: float = 0.5


@dataclass(frozen=True)
class ObstacleSafetyParams:
    """Obstacle-hook integration policy for point missions.

    Disabled by default so deployments without an obstacle publisher report
    ``not_configured`` (honest) instead of silently appearing clear. When
    enabled, a missing or stale ``/rover/obstacle_clear`` signal pauses the
    mission safely (fail-closed).
    """

    enabled: bool = False
    signal_max_age_s: float = 2.0


@dataclass(frozen=True)
class SprayConfiguration:
    mode: SprayMode = SprayMode.CONTINUOUS
    continuous: ContinuousSprayParams = ContinuousSprayParams()
    dash: DashSprayParams = DashSprayParams()
    point: PointSprayParams = PointSprayParams()
    gps_safety: GpsSurveyedSafetyParams = GpsSurveyedSafetyParams()
    safety: SafetySprayParams = SafetySprayParams()
    obstacle: ObstacleSafetyParams = ObstacleSafetyParams()
    calibration: CalibrationProfile = CalibrationProfile()
    revision: int = 0
    mission_id: str = ""
    path_fingerprint: str = ""

    def with_revision(
        self,
        revision: int,
        mission_id: str = "",
        path_fingerprint: str = "",
    ) -> SprayConfiguration:
        return replace(
            self,
            revision=revision,
            mission_id=mission_id,
            path_fingerprint=path_fingerprint,
        )


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


def _bool_value(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _parse_speed_pwm_table(raw: Any) -> tuple[SpeedPwmPoint, ...]:
    if raw is None:
        raw = [
            {"speed_mps": 0.05, "pwm": 1200.0},
            {"speed_mps": 0.35, "pwm": 1800.0},
        ]
    if not isinstance(raw, (list, tuple)):
        raise ValueError("speed_pwm_table must be a list")
    points: list[SpeedPwmPoint] = []
    last_speed: float | None = None
    for item in raw:
        if isinstance(item, dict):
            speed_raw = item.get("speed_mps", item.get("speed"))
            pwm_raw = item.get("pwm", item.get("value"))
        elif isinstance(item, (list, tuple)) and len(item) == 2:
            speed_raw, pwm_raw = item
        else:
            raise ValueError("speed_pwm_table entries must be objects or [speed, pwm]")
        speed = _finite_non_negative("speed_pwm_table.speed_mps", speed_raw)
        pwm = _finite_non_negative("speed_pwm_table.pwm", pwm_raw)
        if last_speed is not None and speed <= last_speed:
            raise ValueError("speed_pwm_table speed entries must be strictly increasing")
        points.append(SpeedPwmPoint(speed, pwm))
        last_speed = speed
    if len(points) < 2:
        raise ValueError("speed_pwm_table requires at least two points")
    return tuple(points)


def interpolate_speed_pwm(
    speed_mps: float,
    table: tuple[SpeedPwmPoint, ...],
    *,
    clamp: bool = True,
) -> float:
    """Interpolate PWM for speed. OFF values are handled outside this helper."""
    speed = float(speed_mps)
    if not table:
        raise ValueError("speed_pwm_table is empty")
    if speed <= table[0].speed_mps:
        if not clamp:
            raise ValueError("speed below calibration table")
        return table[0].pwm
    if speed >= table[-1].speed_mps:
        if not clamp:
            raise ValueError("speed above calibration table")
        return table[-1].pwm
    for left, right in zip(table[:-1], table[1:]):
        if left.speed_mps <= speed <= right.speed_mps:
            span = right.speed_mps - left.speed_mps
            if span <= 0.0:
                raise ValueError("speed_pwm_table speed entries must increase")
            frac = (speed - left.speed_mps) / span
            return left.pwm + frac * (right.pwm - left.pwm)
    return table[-1].pwm


def _parse_calibration_profile(raw: dict[str, Any]) -> CalibrationProfile:
    limits = ActuatorLimits(
        min_pwm=_finite_non_negative("actuator_min_pwm", raw.get("actuator_min_pwm", 0.0)),
        max_pwm=_finite_positive("actuator_max_pwm", raw.get("actuator_max_pwm", 2200.0)),
        off_pwm=_finite_non_negative("actuator_off_pwm", raw.get("actuator_off_pwm", raw.get("off_pwm_us", 0.0))),
        min_value=float(raw.get("actuator_min_value", -1.0)),
        max_value=float(raw.get("actuator_max_value", 1.0)),
        off_value=float(raw.get("actuator_off_value", raw.get("off_value", -1.0))),
    )
    if limits.max_pwm <= limits.min_pwm:
        raise ValueError("actuator_max_pwm must be greater than actuator_min_pwm")
    if not (limits.min_pwm <= limits.off_pwm <= limits.max_pwm):
        raise ValueError("actuator_off_pwm must be within actuator PWM limits")
    if limits.max_value <= limits.min_value:
        raise ValueError("actuator_max_value must be greater than actuator_min_value")
    if not (limits.min_value <= limits.off_value <= limits.max_value):
        raise ValueError("actuator_off_value must be within actuator value limits")

    table = _parse_speed_pwm_table(raw.get("speed_pwm_table"))
    for point in table:
        if not (limits.min_pwm <= point.pwm <= limits.max_pwm):
            raise ValueError("speed_pwm_table PWM entries must be within actuator limits")

    version = int(raw.get("calibration_profile_version", raw.get("profile_version", 1)))
    if version < 1:
        raise ValueError("calibration_profile_version must be >= 1")

    hooks = HardwareCompensationHooks(
        pump_inertia_enabled=_bool_value(raw.get("pump_inertia_enabled", False)),
        pwm_ramp_prediction_enabled=_bool_value(raw.get("pwm_ramp_prediction_enabled", False)),
        pressure_stabilization_enabled=_bool_value(raw.get("pressure_stabilization_enabled", False)),
        temperature_viscosity_compensation_enabled=_bool_value(
            raw.get("temperature_viscosity_compensation_enabled", False)
        ),
    )

    return CalibrationProfile(
        profile_id=str(raw.get("calibration_profile_id", "factory_default") or "factory_default"),
        version=version,
        target_paint_density=_finite_positive(
            "target_paint_density",
            raw.get("target_paint_density", 1.0),
        ),
        speed_pwm_table=table,
        actuator_limits=limits,
        timing_only_compatibility=_bool_value(raw.get("timing_only_compatibility", False)),
        hooks=hooks,
    )


def pwm_to_normalized_value(pwm: float, limits: ActuatorLimits) -> float:
    span_pwm = limits.max_pwm - limits.min_pwm
    span_value = limits.max_value - limits.min_value
    if span_pwm <= 0.0 or span_value <= 0.0:
        raise ValueError("invalid actuator limits")
    frac = (float(pwm) - limits.min_pwm) / span_pwm
    value = limits.min_value + frac * span_value
    return max(limits.min_value, min(limits.max_value, value))


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
        max_spray_speed_mps=_finite_positive(
            "max_spray_speed_mps", raw.get("max_spray_speed_mps", 1.0)
        ),
        unsafe_speed_behavior=UnsafeSpeedBehavior.parse(
            raw.get("unsafe_speed_behavior", UnsafeSpeedBehavior.BLOCK_SPRAY.value)
        ),
        max_xtrack_error_m=_finite_positive(
            "max_xtrack_error_m", raw.get("max_xtrack_error_m", 0.10)
        ),
        nozzle_forward_offset_m=float(raw.get("nozzle_forward_offset_m", 0.0)),
        nozzle_lateral_offset_m=float(raw.get("nozzle_lateral_offset_m", 0.0)),
    )
    if continuous.max_spray_speed_mps <= continuous.min_spray_speed_mps:
        raise ValueError("max_spray_speed_mps must be greater than min_spray_speed_mps")

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
        max_dwell_s=_finite_positive(
            "point_max_dwell_s", raw.get("point_max_dwell_s", 60.0)
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
        leg_trajectory_mode=str(
            raw.get("point_leg_trajectory_mode", "two_point")
        ).strip().lower(),
        leg_spacing_m=_finite_positive(
            "point_leg_spacing_m", raw.get("point_leg_spacing_m", 0.08)
        ),
        hold_drift_tolerance_m=_finite_positive(
            "point_hold_drift_tolerance_m",
            raw.get("point_hold_drift_tolerance_m", 0.08),
        ),
        hold_drift_policy=str(
            raw.get("point_hold_drift_policy", "fail")
        ).strip().lower(),
    )
    if point.default_dwell_s > point.max_dwell_s:
        raise ValueError("point_default_dwell_s exceeds point_max_dwell_s")
    if point.leg_trajectory_mode not in {"two_point", "densified"}:
        raise ValueError(
            "point_leg_trajectory_mode must be two_point or densified"
        )
    if point.hold_drift_policy not in {"fail", "pause"}:
        raise ValueError("point_hold_drift_policy must be fail or pause")

    runtime_policy = str(raw.get("gps_runtime_policy", "pause")).strip().lower()
    if runtime_policy not in {"pause", "fail"}:
        raise ValueError("gps_runtime_policy must be pause or fail")
    resume_policy = str(raw.get("gps_resume_policy", "manual")).strip().lower()
    if resume_policy not in {"manual", "auto"}:
        raise ValueError("gps_resume_policy must be manual or auto")
    try:
        required_fix = int(raw.get("gps_required_fix_type", 6))
    except (TypeError, ValueError) as exc:
        raise ValueError("gps_required_fix_type must be an integer") from exc
    if required_fix < 0 or required_fix > 8:
        raise ValueError("gps_required_fix_type must be in [0, 8]")

    gps_safety = GpsSurveyedSafetyParams(
        required_fix_type=required_fix,
        global_position_max_age_ms=_finite_positive(
            "gps_global_position_max_age_ms",
            raw.get("gps_global_position_max_age_ms", 500.0),
        ),
        local_pose_max_age_ms=_finite_positive(
            "gps_local_pose_max_age_ms",
            raw.get("gps_local_pose_max_age_ms", 500.0),
        ),
        gps_fix_max_age_ms=_finite_positive(
            "gps_fix_max_age_ms", raw.get("gps_fix_max_age_ms", 500.0)
        ),
        max_pose_global_skew_ms=_finite_positive(
            "gps_max_pose_global_skew_ms",
            raw.get("gps_max_pose_global_skew_ms", 100.0),
        ),
        runtime_policy=runtime_policy,
        resume_policy=resume_policy,
        recovery_stable_s=_finite_positive(
            "gps_recovery_stable_s", raw.get("gps_recovery_stable_s", 2.0)
        ),
    )

    obstacle = ObstacleSafetyParams(
        enabled=_bool_value(raw.get("obstacle_integration_enabled", False)),
        signal_max_age_s=_finite_positive(
            "obstacle_signal_max_age_s",
            raw.get("obstacle_signal_max_age_s", 2.0),
        ),
    )

    debounce = int(raw.get("debounce_samples", 3))
    if debounce < 1 or debounce > 20:
        raise ValueError("debounce_samples must be in [1, 20]")

    safety = SafetySprayParams(
        require_offboard=_bool_value(raw.get("require_offboard", True)),
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
    path_fingerprint = str(raw.get("path_fingerprint", "") or "")
    calibration = _parse_calibration_profile(raw)
    if mission_id and not calibration.timing_only_compatibility:
        if not path_fingerprint:
            raise ValueError(
                "mission path_fingerprint is required unless timing_only_compatibility is explicit"
            )
        if revision <= 0:
            raise ValueError(
                "configuration_revision must be > 0 for mission-bound spray configs"
            )

    config = SprayConfiguration(
        mode=mode,
        continuous=continuous,
        dash=dash,
        point=point,
        gps_safety=gps_safety,
        safety=safety,
        obstacle=obstacle,
        calibration=calibration,
        revision=revision,
        mission_id=mission_id,
        path_fingerprint=path_fingerprint,
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
        "max_spray_speed_mps": config.continuous.max_spray_speed_mps,
        "unsafe_speed_behavior": config.continuous.unsafe_speed_behavior.value,
        "max_xtrack_error_m": config.continuous.max_xtrack_error_m,
        "nozzle_forward_offset_m": config.continuous.nozzle_forward_offset_m,
        "nozzle_lateral_offset_m": config.continuous.nozzle_lateral_offset_m,
        "dash_on_distance_m": config.dash.on_distance_m,
        "dash_off_distance_m": config.dash.off_distance_m,
        "dash_phase_reset": config.dash.phase_reset.value,
        "point_default_dwell_s": config.point.default_dwell_s,
        "point_max_dwell_s": config.point.max_dwell_s,
        "point_arrival_tolerance_m": config.point.arrival_tolerance_m,
        "point_settle_time_s": config.point.settle_time_s,
        "point_leg_timeout_s": config.point.leg_timeout_s,
        "point_settle_speed_mps": config.point.settle_speed_mps,
        "point_settle_yaw_rate_rad_s": config.point.settle_yaw_rate_rad_s,
        "point_leg_trajectory_mode": config.point.leg_trajectory_mode,
        "point_leg_spacing_m": config.point.leg_spacing_m,
        "point_hold_drift_tolerance_m": config.point.hold_drift_tolerance_m,
        "point_hold_drift_policy": config.point.hold_drift_policy,
        "gps_required_fix_type": config.gps_safety.required_fix_type,
        "gps_global_position_max_age_ms": config.gps_safety.global_position_max_age_ms,
        "gps_local_pose_max_age_ms": config.gps_safety.local_pose_max_age_ms,
        "gps_fix_max_age_ms": config.gps_safety.gps_fix_max_age_ms,
        "gps_max_pose_global_skew_ms": config.gps_safety.max_pose_global_skew_ms,
        "gps_runtime_policy": config.gps_safety.runtime_policy,
        "gps_resume_policy": config.gps_safety.resume_policy,
        "gps_recovery_stable_s": config.gps_safety.recovery_stable_s,
        "obstacle_integration_enabled": config.obstacle.enabled,
        "obstacle_signal_max_age_s": config.obstacle.signal_max_age_s,
        "require_offboard": config.safety.require_offboard,
        "debounce_samples": config.safety.debounce_samples,
        "pose_timeout_s": config.safety.pose_timeout_s,
        "velocity_timeout_s": config.safety.velocity_timeout_s,
        "configuration_revision": config.revision,
        "mission_config_mission_id": config.mission_id,
        "mission_config_path_fingerprint": config.path_fingerprint,
        "calibration_profile_id": config.calibration.profile_id,
        "calibration_profile_version": config.calibration.version,
        "target_paint_density": config.calibration.target_paint_density,
        "speed_pwm_table": json.dumps([
            {"speed_mps": p.speed_mps, "pwm": p.pwm}
            for p in config.calibration.speed_pwm_table
        ], separators=(",", ":")),
        "actuator_min_pwm": config.calibration.actuator_limits.min_pwm,
        "actuator_max_pwm": config.calibration.actuator_limits.max_pwm,
        "actuator_off_pwm": config.calibration.actuator_limits.off_pwm,
        "actuator_min_value": config.calibration.actuator_limits.min_value,
        "actuator_max_value": config.calibration.actuator_limits.max_value,
        "actuator_off_value": config.calibration.actuator_limits.off_value,
        "timing_only_compatibility": config.calibration.timing_only_compatibility,
        "pump_inertia_enabled": config.calibration.hooks.pump_inertia_enabled,
        "pwm_ramp_prediction_enabled": config.calibration.hooks.pwm_ramp_prediction_enabled,
        "pressure_stabilization_enabled": config.calibration.hooks.pressure_stabilization_enabled,
        "temperature_viscosity_compensation_enabled": (
            config.calibration.hooks.temperature_viscosity_compensation_enabled
        ),
    }


def staged_spray_defaults() -> dict[str, Any]:
    """Default spray fields for staged mission artifacts."""
    cfg = SprayConfiguration()
    return {
        "spray_mode": cfg.mode.value,
        "solenoid_open_delay_s": cfg.continuous.solenoid_open_delay_s,
        "solenoid_close_delay_s": cfg.continuous.solenoid_close_delay_s,
        "on_overspray_margin_m": cfg.continuous.on_overspray_margin_m,
        "off_overspray_margin_m": cfg.continuous.off_overspray_margin_m,
        "min_spray_speed_mps": cfg.continuous.min_spray_speed_mps,
        "max_spray_speed_mps": cfg.continuous.max_spray_speed_mps,
        "unsafe_speed_behavior": cfg.continuous.unsafe_speed_behavior.value,
        "max_xtrack_error_m": cfg.continuous.max_xtrack_error_m,
        "nozzle_forward_offset_m": cfg.continuous.nozzle_forward_offset_m,
        "nozzle_lateral_offset_m": cfg.continuous.nozzle_lateral_offset_m,
        "dash_on_distance_m": cfg.dash.on_distance_m,
        "dash_off_distance_m": cfg.dash.off_distance_m,
        "dash_phase_reset": cfg.dash.phase_reset.value,
        "point_default_dwell_s": cfg.point.default_dwell_s,
        "point_max_dwell_s": cfg.point.max_dwell_s,
        "point_arrival_tolerance_m": cfg.point.arrival_tolerance_m,
        "point_settle_time_s": cfg.point.settle_time_s,
        "point_leg_timeout_s": cfg.point.leg_timeout_s,
        "point_settle_speed_mps": cfg.point.settle_speed_mps,
        "point_settle_yaw_rate_rad_s": cfg.point.settle_yaw_rate_rad_s,
        "point_leg_trajectory_mode": cfg.point.leg_trajectory_mode,
        "point_leg_spacing_m": cfg.point.leg_spacing_m,
        "point_hold_drift_tolerance_m": cfg.point.hold_drift_tolerance_m,
        "point_hold_drift_policy": cfg.point.hold_drift_policy,
        "point_execution_mode": "auto",
        "point_mission_points": [],
        "gps_required_fix_type": 6,
        "gps_global_position_max_age_ms": 500.0,
        "gps_local_pose_max_age_ms": 500.0,
        "gps_fix_max_age_ms": 500.0,
        "gps_max_pose_global_skew_ms": 100.0,
        "gps_runtime_policy": "pause",
        "gps_resume_policy": "manual",
        "gps_recovery_stable_s": 2.0,
        "obstacle_integration_enabled": cfg.obstacle.enabled,
        "obstacle_signal_max_age_s": cfg.obstacle.signal_max_age_s,
        "calibration_profile_id": cfg.calibration.profile_id,
        "calibration_profile_version": cfg.calibration.version,
        "target_paint_density": cfg.calibration.target_paint_density,
        "speed_pwm_table": [
            {"speed_mps": p.speed_mps, "pwm": p.pwm}
            for p in cfg.calibration.speed_pwm_table
        ],
        "actuator_min_pwm": cfg.calibration.actuator_limits.min_pwm,
        "actuator_max_pwm": cfg.calibration.actuator_limits.max_pwm,
        "actuator_off_pwm": cfg.calibration.actuator_limits.off_pwm,
        "actuator_min_value": cfg.calibration.actuator_limits.min_value,
        "actuator_max_value": cfg.calibration.actuator_limits.max_value,
        "actuator_off_value": cfg.calibration.actuator_limits.off_value,
        "timing_only_compatibility": cfg.calibration.timing_only_compatibility,
        "pump_inertia_enabled": cfg.calibration.hooks.pump_inertia_enabled,
        "pwm_ramp_prediction_enabled": cfg.calibration.hooks.pwm_ramp_prediction_enabled,
        "pressure_stabilization_enabled": cfg.calibration.hooks.pressure_stabilization_enabled,
        "temperature_viscosity_compensation_enabled": (
            cfg.calibration.hooks.temperature_viscosity_compensation_enabled
        ),
    }


def parse_staged_spray_config(staged: dict[str, Any]) -> SprayConfiguration:
    """Parse spray configuration from a staged mission artifact."""
    raw = staged_spray_defaults()
    for key in raw:
        if key in staged:
            raw[key] = staged[key]
    raw["mission_id"] = str(staged.get("mission_id", "") or "")
    raw["path_fingerprint"] = str(staged.get("path_fingerprint", "") or "")
    raw["configuration_revision"] = int(staged.get("configuration_revision", 0))
    return validate_spray_configuration(raw)
