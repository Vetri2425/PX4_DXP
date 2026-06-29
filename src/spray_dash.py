"""Dash spray pattern transform on SprayPathModel (ROS-independent)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from spray_config import DashPhaseReset, SprayConfiguration
from spray_path_model import SprayPathModel, build_path_model


_ABSOLUTE_TOLERANCE = 1e-9
_RELATIVE_TOLERANCE = 1e-12


@dataclass(frozen=True)
class DashTransformParams:
    on_distance_m: float
    off_distance_m: float
    reset_mode: DashPhaseReset


@dataclass(frozen=True)
class DashFeasibilityResult:
    dash_feasible: bool
    dash_feasibility_reason: str
    shortest_dash_on_run_m: float
    shortest_dash_off_gap_m: float
    expected_on_lead_m: float
    expected_off_lead_m: float
    minimum_achievable_on_m: float
    minimum_achievable_off_m: float
    dash_expected_speed_mps: float = 0.0
    dash_feasibility_speed_source: str = "max_spray_speed_mps"

    def as_dict(self) -> dict:
        return {
            "dash_feasible": self.dash_feasible,
            "dash_feasibility_reason": self.dash_feasibility_reason,
            "shortest_dash_on_run_m": self.shortest_dash_on_run_m,
            "shortest_dash_off_gap_m": self.shortest_dash_off_gap_m,
            "expected_on_lead_m": self.expected_on_lead_m,
            "expected_off_lead_m": self.expected_off_lead_m,
            "minimum_achievable_on_m": self.minimum_achievable_on_m,
            "minimum_achievable_off_m": self.minimum_achievable_off_m,
            "dash_expected_speed_mps": self.dash_expected_speed_mps,
            "dash_feasibility_speed_source": self.dash_feasibility_speed_source,
        }


def _dash_epsilon(*values: float) -> float:
    scale = max((abs(v) for v in values), default=0.0)
    return max(_ABSOLUTE_TOLERANCE, scale * _RELATIVE_TOLERANCE)


def _is_near(a: float, b: float, epsilon: float) -> bool:
    return abs(a - b) <= epsilon


def _normalized_mod(value: float, period: float, epsilon: float) -> float:
    if period <= epsilon:
        return 0.0
    remainder = value % period
    if remainder < epsilon or abs(remainder - period) < epsilon:
        return 0.0
    return remainder


def _flag_at_distance(
    distance_m: float,
    on_distance_m: float,
    off_distance_m: float,
    phase_offset_m: float,
) -> bool:
    period = on_distance_m + off_distance_m
    if period <= 0.0:
        return on_distance_m > 0.0
    epsilon = max(
        _ABSOLUTE_TOLERANCE,
        period * _RELATIVE_TOLERANCE,
        abs(distance_m) * _RELATIVE_TOLERANCE,
        abs(phase_offset_m) * _RELATIVE_TOLERANCE,
    )
    phase = _normalized_mod(phase_offset_m + distance_m, period, epsilon)
    if _is_near(phase, period, epsilon):
        phase = 0.0
    if _is_near(phase, on_distance_m, epsilon):
        return False
    return phase < on_distance_m


def _mark_regions(flags: list[bool]) -> list[tuple[int, int]]:
    regions: list[tuple[int, int]] = []
    i = 0
    n = len(flags)
    while i < n:
        if not flags[i]:
            i += 1
            continue
        start = i
        while i < n and flags[i]:
            i += 1
        regions.append((start, i - 1))
    return regions


def _collect_phase_boundaries(
    region_len: float,
    on_distance_m: float,
    off_distance_m: float,
    phase_offset_m: float,
) -> list[float]:
    """Local arc-length positions [0, region_len] at each dash phase transition."""
    epsilon = _dash_epsilon(region_len, on_distance_m, off_distance_m, phase_offset_m)
    if region_len <= epsilon:
        return [0.0]
    period = on_distance_m + off_distance_m
    if period <= epsilon:
        return [0.0, region_len]

    boundaries = {0.0, region_len}
    max_k = int(math.ceil((region_len + abs(phase_offset_m)) / period)) + 1
    for k in range(-1, max_k + 1):
        for phase_target in (0.0, on_distance_m):
            local_d = phase_target - phase_offset_m + k * period
            if local_d <= epsilon or local_d >= region_len - epsilon:
                continue
            if epsilon < local_d < region_len - epsilon:
                boundaries.add(local_d)
    return sorted(boundaries)


def extract_mark_region_runs(
    dashed: SprayPathModel,
    original: SprayPathModel,
) -> list[tuple[float, bool]]:
    """Return (length_m, is_on) runs inside each original MARK region."""
    runs: list[tuple[float, bool]] = []
    for start, end in _mark_regions(list(original.flags)):
        region_start_s = original.cumulative_s[start]
        region_end_s = original.cumulative_s[end]
        local_bounds = [
            s
            for s in dashed.cumulative_s
            if region_start_s - 1e-9 <= s <= region_end_s + 1e-9
        ]
        local_bounds = _merge_distances(local_bounds)
        if len(local_bounds) < 2:
            continue
        for left, right in zip(local_bounds, local_bounds[1:]):
            run_len = right - left
            if run_len <= 1e-12:
                continue
            mid_flag = _flag_at_dashed_midpoint(dashed, 0.5 * (left + right))
            runs.append((run_len, mid_flag))
    return runs


def _interpolate_point(
    a: tuple[float, float],
    b: tuple[float, float],
    ratio: float,
) -> tuple[float, float]:
    return (
        a[0] + ratio * (b[0] - a[0]),
        a[1] + ratio * (b[1] - a[1]),
    )


def _point_at_arc_length(
    points: list[tuple[float, float]],
    cumulative_s: list[float],
    target_s: float,
) -> tuple[float, float]:
    if not points:
        raise ValueError("empty path")
    if target_s <= cumulative_s[0] + 1e-12:
        return points[0]
    if target_s >= cumulative_s[-1] - 1e-12:
        return points[-1]
    for idx in range(1, len(points)):
        end_s = cumulative_s[idx]
        if target_s > end_s + 1e-12:
            continue
        start_s = cumulative_s[idx - 1]
        seg_len = end_s - start_s
        if seg_len <= 1e-12:
            return points[idx]
        ratio = (target_s - start_s) / seg_len
        return _interpolate_point(points[idx - 1], points[idx], ratio)
    return points[-1]


def _merge_distances(distances: list[float], *, eps: float = 1e-6) -> list[float]:
    if not distances:
        return []
    merged = [distances[0]]
    for value in distances[1:]:
        if abs(value - merged[-1]) <= eps:
            continue
        merged.append(value)
    return merged


def _flag_for_global_s(
    s: float,
    *,
    base_flags: list[bool],
    cumulative_s: list[float],
    on_distance_m: float,
    off_distance_m: float,
    reset_mode: DashPhaseReset,
) -> bool:
    idx = 0
    while idx < len(cumulative_s) and cumulative_s[idx] < s - 1e-9:
        idx += 1
    if idx >= len(base_flags) or not base_flags[idx]:
        return False

    for start, end in _mark_regions(base_flags):
        region_start_s = cumulative_s[start]
        region_end_s = cumulative_s[end]
        if s < region_start_s - 1e-9 or s > region_end_s + 1e-9:
            continue
        local_s = s - region_start_s
        if reset_mode == DashPhaseReset.PER_MARK_REGION:
            phase_offset = 0.0
        else:
            global_mark_s = 0.0
            for rs, re in _mark_regions(base_flags):
                if re < start:
                    global_mark_s += cumulative_s[re] - cumulative_s[rs]
            phase_offset = global_mark_s % (on_distance_m + off_distance_m) if (
                on_distance_m + off_distance_m
            ) > 0 else 0.0
        return _flag_at_distance(local_s, on_distance_m, off_distance_m, phase_offset)
    return False


def _flag_at_dashed_midpoint(dashed: SprayPathModel, mid_s: float) -> bool:
    if not dashed.flags:
        return False
    idx = 0
    for i, s in enumerate(dashed.cumulative_s):
        if s <= mid_s + 1e-9:
            idx = i
    return dashed.flags[idx]


def _shortest_runs_in_mark_regions(
    dashed: SprayPathModel,
    original: SprayPathModel,
) -> tuple[float, float]:
    shortest_on = float("inf")
    shortest_off = float("inf")
    for start, end in _mark_regions(list(original.flags)):
        region_start_s = original.cumulative_s[start]
        region_end_s = original.cumulative_s[end]
        local_bounds = [
            s
            for s in dashed.cumulative_s
            if region_start_s - 1e-9 <= s <= region_end_s + 1e-9
        ]
        local_bounds = _merge_distances(local_bounds)
        if len(local_bounds) < 2:
            continue
        for left, right in zip(local_bounds, local_bounds[1:]):
            run_len = right - left
            if run_len <= 1e-12:
                continue
            mid_flag = _flag_at_dashed_midpoint(dashed, 0.5 * (left + right))
            if mid_flag:
                shortest_on = min(shortest_on, run_len)
            else:
                shortest_off = min(shortest_off, run_len)
    if shortest_on == float("inf"):
        shortest_on = 0.0
    if shortest_off == float("inf"):
        shortest_off = 0.0
    return shortest_on, shortest_off


def _bounded_leads(
    speed_mps: float,
    *,
    open_delay_s: float,
    close_delay_s: float,
    on_overspray_m: float,
    off_overspray_m: float,
    max_lead_m: float,
) -> tuple[float, float]:
    raw_on = max(0.0, speed_mps) * open_delay_s + on_overspray_m
    raw_off = max(
        0.0,
        max(0.0, speed_mps) * close_delay_s - off_overspray_m,
    )
    return min(raw_on, max_lead_m), min(raw_off, max_lead_m)


def validate_dash_feasibility(
    model: SprayPathModel,
    config: SprayConfiguration,
    *,
    expected_speed_mps: float | None = None,
) -> DashFeasibilityResult:
    """Reject dash missions whose geometry cannot be achieved at expected speed."""
    on_m = config.dash.on_distance_m
    off_m = config.dash.off_distance_m
    dashed = apply_dash_pattern(
        model,
        on_distance_m=on_m,
        off_distance_m=off_m,
        reset_mode=config.dash.phase_reset,
    )
    shortest_on, shortest_off = _shortest_runs_in_mark_regions(dashed, model)

    if expected_speed_mps is None:
        speed = config.continuous.max_spray_speed_mps
        speed_source = "max_spray_speed_mps"
    else:
        speed = float(expected_speed_mps)
        speed_source = "staged_marking_speed_mps"
    if not math.isfinite(speed) or speed < 0.0:
        raise ValueError("expected_speed_mps must be a finite non-negative number")
    speed = max(0.0, speed)

    bounded_on_lead, bounded_off_lead = _bounded_leads(
        speed,
        open_delay_s=config.continuous.solenoid_open_delay_s,
        close_delay_s=config.continuous.solenoid_close_delay_s,
        on_overspray_m=config.continuous.on_overspray_margin_m,
        off_overspray_m=config.continuous.off_overspray_margin_m,
        max_lead_m=config.continuous.max_lead_distance_m,
    )
    min_on = config.continuous.min_on_distance_m
    min_off = config.continuous.min_off_distance_m
    min_achievable_on = min_on
    min_achievable_off = min_off

    reasons: list[str] = []
    if on_m > 0.0 and shortest_on + 1e-9 < min_achievable_on:
        reasons.append(
            f"shortest dash ON run {shortest_on:.3f} m < minimum achievable "
            f"{min_achievable_on:.3f} m"
        )
    if off_m > 0.0 and shortest_off + 1e-9 < min_achievable_off:
        reasons.append(
            f"shortest dash OFF gap {shortest_off:.3f} m < minimum achievable "
            f"{min_achievable_off:.3f} m"
        )
    if on_m > 0.0 and shortest_on + 1e-9 < bounded_on_lead:
        reasons.append(
            f"ON lead {bounded_on_lead:.3f} m would erase dash run {shortest_on:.3f} m"
        )
    if off_m > 0.0 and shortest_off + 1e-9 < bounded_off_lead:
        reasons.append(
            f"OFF lead {bounded_off_lead:.3f} m would erase dash gap {shortest_off:.3f} m"
        )
    if on_m > 0.0 and shortest_on + 1e-9 < (
        speed * config.continuous.solenoid_open_delay_s
    ):
        reasons.append("dash ON run shorter than solenoid open delay distance")

    feasible = not reasons
    return DashFeasibilityResult(
        dash_feasible=feasible,
        dash_feasibility_reason="; ".join(reasons),
        shortest_dash_on_run_m=shortest_on,
        shortest_dash_off_gap_m=shortest_off,
        expected_on_lead_m=bounded_on_lead,
        expected_off_lead_m=bounded_off_lead,
        minimum_achievable_on_m=min_achievable_on,
        minimum_achievable_off_m=min_achievable_off,
        dash_expected_speed_mps=speed,
        dash_feasibility_speed_source=speed_source,
    )


def apply_dash_pattern(
    model: SprayPathModel,
    on_distance_m: float,
    off_distance_m: float,
    reset_mode: DashPhaseReset,
) -> SprayPathModel:
    """Return a new model with exact dash boundaries inside MARK regions only."""
    if on_distance_m < 0.0 or off_distance_m < 0.0:
        raise ValueError("dash distances must be non-negative")
    if on_distance_m <= 0.0 and off_distance_m <= 0.0:
        raise ValueError("dash requires at least one positive ON/OFF distance")

    points = list(model.points)
    base_flags = list(model.flags)
    cumulative_s = list(model.cumulative_s)
    if not points:
        return build_path_model([], [])

    split_distances = {0.0, cumulative_s[-1]}
    global_mark_s = 0.0
    period = on_distance_m + off_distance_m

    for start, end in _mark_regions(base_flags):
        region_start_s = cumulative_s[start]
        region_len = cumulative_s[end] - cumulative_s[start]
        if reset_mode == DashPhaseReset.PER_MARK_REGION:
            phase_offset = 0.0
        else:
            phase_offset = global_mark_s % period if period > 0.0 else 0.0
        for local_d in _collect_phase_boundaries(
            region_len, on_distance_m, off_distance_m, phase_offset
        ):
            split_distances.add(region_start_s + local_d)
        if reset_mode == DashPhaseReset.CONTINUOUS and region_len > 0.0:
            global_mark_s += region_len

    ordered_s = _merge_distances(sorted(split_distances))
    new_points: list[tuple[float, float]] = []
    new_flags: list[bool] = []
    for s in ordered_s:
        new_points.append(_point_at_arc_length(points, cumulative_s, s))
        new_flags.append(
            _flag_for_global_s(
                s,
                base_flags=base_flags,
                cumulative_s=cumulative_s,
                on_distance_m=on_distance_m,
                off_distance_m=off_distance_m,
                reset_mode=reset_mode,
            )
        )
    return build_path_model(new_points, new_flags)


def apply_dash_pattern_from_params(
    model: SprayPathModel,
    params: DashTransformParams,
) -> SprayPathModel:
    return apply_dash_pattern(
        model,
        on_distance_m=params.on_distance_m,
        off_distance_m=params.off_distance_m,
        reset_mode=params.reset_mode,
    )