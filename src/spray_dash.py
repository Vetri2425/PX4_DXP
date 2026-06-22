"""Dash spray pattern transform on SprayPathModel (ROS-independent)."""

from __future__ import annotations

from dataclasses import dataclass
from spray_config import DashPhaseReset
from spray_path_model import SprayPathModel, build_path_model


@dataclass(frozen=True)
class DashTransformParams:
    on_distance_m: float
    off_distance_m: float
    reset_mode: DashPhaseReset


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


def _flag_at_distance(
    distance_m: float,
    on_distance_m: float,
    off_distance_m: float,
    phase_offset_m: float,
) -> bool:
    period = on_distance_m + off_distance_m
    if period <= 0.0:
        return on_distance_m > 0.0
    pos = (phase_offset_m + distance_m) % period
    return pos < on_distance_m


def apply_dash_pattern(
    model: SprayPathModel,
    on_distance_m: float,
    off_distance_m: float,
    reset_mode: DashPhaseReset,
) -> SprayPathModel:
    """Return a new model with dash ON/OFF flags inside MARK regions only."""
    if on_distance_m < 0.0 or off_distance_m < 0.0:
        raise ValueError("dash distances must be non-negative")
    if on_distance_m <= 0.0 and off_distance_m <= 0.0:
        raise ValueError("dash requires at least one positive ON/OFF distance")

    points = list(model.points)
    base_flags = list(model.flags)
    if not points:
        return build_path_model([], [])

    new_flags = [False] * len(base_flags)
    regions = _mark_regions(base_flags)

    global_mark_s = 0.0
    for start, end in regions:
        region_len = model.cumulative_s[end] - model.cumulative_s[start]
        if reset_mode == DashPhaseReset.PER_MARK_REGION:
            phase_offset = 0.0
            for idx in range(start, end + 1):
                local_s = model.cumulative_s[idx] - model.cumulative_s[start]
                new_flags[idx] = _flag_at_distance(
                    local_s, on_distance_m, off_distance_m, phase_offset
                )
        else:
            for idx in range(start, end + 1):
                local_s = model.cumulative_s[idx] - model.cumulative_s[start]
                mark_distance = global_mark_s + local_s
                new_flags[idx] = _flag_at_distance(
                    mark_distance, on_distance_m, off_distance_m, 0.0
                )
            if region_len > 0.0:
                global_mark_s += region_len

    return build_path_model(points, new_flags)


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
