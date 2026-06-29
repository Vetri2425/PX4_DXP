#!/usr/bin/env python3
"""Unit tests for dash spray pattern transform."""

from __future__ import annotations

import copy
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from spray_config import DashPhaseReset
from spray_path_model import build_path_model as _build_path_model
from spray_dash import (
    apply_dash_pattern,
    extract_mark_region_runs,
    validate_dash_feasibility,
)
from spray_config import validate_spray_configuration


_RUN_LEN_TOL = 0.02


def _assert_runs(model, dashed, expected):
    runs = extract_mark_region_runs(dashed, model)
    assert len(runs) == len(expected), f"runs={runs} expected={expected}"
    for (length, is_on), (exp_len, exp_on) in zip(runs, expected):
        assert abs(length - exp_len) < _RUN_LEN_TOL, (
            f"length {length} != {exp_len}"
        )
        assert is_on is exp_on, f"flag {is_on} != {exp_on} for len {length}"


def _one_meter_mark():
    return _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
        flags=[False, True, True, False],
    )


def _dash_config(**overrides):
    raw = {
        "mission_id": "dash",
        "path_fingerprint": "fp",
        "configuration_revision": 1,
        "spray_mode": "dash",
        "dash_on_distance_m": 0.3,
        "dash_off_distance_m": 0.3,
    }
    raw.update(overrides)
    return validate_spray_configuration(raw)


def _mark_path():
    return _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
        flags=[False, True, True, False],
    )


def test_straight_mark_region_dash_boundaries():
    model = apply_dash_pattern(_mark_path(), 0.3, 0.3, DashPhaseReset.PER_MARK_REGION)
    assert model.flags[0] is False
    on_runs = []
    for i in range(1, len(model.points)):
        if model.flags[i - 1]:
            on_runs.append(model.cumulative_s[i] - model.cumulative_s[i - 1])
    assert any(abs(run - 0.3) < 0.02 for run in on_runs)
    assert model.flags[-1] is False


def test_transit_always_off():
    model = apply_dash_pattern(_mark_path(), 0.3, 0.3, DashPhaseReset.PER_MARK_REGION)
    assert model.flags[0] is False
    assert model.flags[-1] is False


def test_multiple_mark_regions_per_region_reset():
    model = _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)],
        flags=[False, True, False, True, False],
    )
    dashed = apply_dash_pattern(model, 0.5, 0.5, DashPhaseReset.PER_MARK_REGION)
    mark_region_flags = [
        dashed.flags[i]
        for i, s in enumerate(dashed.cumulative_s)
        if 0.9 < s < 1.1 or 2.9 < s < 3.1
    ]
    assert any(mark_region_flags)


def test_continuous_phase_carries_across_regions():
    model = _build_path_model(
        points=[(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)],
        flags=[False, True, False, True, False],
    )
    dashed = apply_dash_pattern(model, 0.6, 0.4, DashPhaseReset.CONTINUOUS)
    second_region_mid = 1.75
    idx = max(
        i for i, s in enumerate(dashed.cumulative_s) if s <= second_region_mid + 1e-9
    )
    assert dashed.flags[idx] is True


def test_l_shape_cumulative_distance():
    model = _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
        flags=[True, True, True],
    )
    dashed = apply_dash_pattern(model, 0.3, 0.4, DashPhaseReset.PER_MARK_REGION)
    assert dashed.flags[0] is True
    assert len(dashed.points) >= len(model.points)


def test_input_model_not_mutated():
    model = _mark_path()
    original_flags = list(model.flags)
    _ = apply_dash_pattern(model, 0.3, 0.3, DashPhaseReset.PER_MARK_REGION)
    assert model.flags == original_flags


def test_recomputed_boundaries():
    model = apply_dash_pattern(_mark_path(), 0.5, 0.5, DashPhaseReset.PER_MARK_REGION)
    assert len(model.boundaries) >= 1


def test_invalid_parameters_rejected():
    model = _mark_path()
    try:
        apply_dash_pattern(model, -0.1, 0.3, DashPhaseReset.PER_MARK_REGION)
        assert False, "expected ValueError"
    except ValueError:
        pass
    try:
        apply_dash_pattern(model, 0.0, 0.0, DashPhaseReset.PER_MARK_REGION)
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_dash_feasibility_exposes_metrics():
    model = _mark_path()
    result = validate_dash_feasibility(model, _dash_config())
    assert result.shortest_dash_on_run_m > 0.0
    assert "dash_feasible" in result.as_dict()


def test_dash_1m_03_03_terminal_partial_off_not_on():
    model = _one_meter_mark()
    dashed = apply_dash_pattern(model, 0.3, 0.3, DashPhaseReset.PER_MARK_REGION)
    _assert_runs(
        model,
        dashed,
        [(0.3, True), (0.3, False), (0.3, True), (0.1, False)],
    )
    result = validate_dash_feasibility(model, _dash_config())
    assert result.dash_feasible is True
    assert abs(result.shortest_dash_on_run_m - 0.3) < _RUN_LEN_TOL
    assert abs(result.shortest_dash_off_gap_m - 0.1) < _RUN_LEN_TOL
    assert "erase dash run 0.100" not in result.dash_feasibility_reason


def test_dash_09m_ends_at_phase_boundary():
    model = _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (1.9, 0.0), (2.0, 0.0)],
        flags=[False, True, True, False],
    )
    dashed = apply_dash_pattern(model, 0.3, 0.3, DashPhaseReset.PER_MARK_REGION)
    _assert_runs(
        model,
        dashed,
        [(0.3, True), (0.3, False), (0.3, True)],
    )


def test_dash_12m_complete_periods():
    model = _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (2.2, 0.0), (3.0, 0.0)],
        flags=[False, True, True, False],
    )
    dashed = apply_dash_pattern(model, 0.3, 0.3, DashPhaseReset.PER_MARK_REGION)
    _assert_runs(
        model,
        dashed,
        [(0.3, True), (0.3, False), (0.3, True), (0.3, False)],
    )


def test_dash_boundary_at_original_waypoint():
    model = _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (1.3, 0.0), (2.0, 0.0), (3.0, 0.0)],
        flags=[False, True, True, True, False],
    )
    dashed = apply_dash_pattern(model, 0.3, 0.3, DashPhaseReset.PER_MARK_REGION)
    assert any(abs(s - 1.3) < 1e-6 for s in dashed.cumulative_s)
    _assert_runs(
        model,
        dashed,
        [(0.3, True), (0.3, False), (0.3, True), (0.1, False)],
    )


def test_dash_sparse_segment_multiple_boundaries():
    model = _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (5.0, 0.0), (6.0, 0.0)],
        flags=[False, True, True, False],
    )
    dashed = apply_dash_pattern(model, 0.3, 0.3, DashPhaseReset.PER_MARK_REGION)
    mark_splits = [
        s for s in dashed.cumulative_s if 1.0 - 1e-9 <= s <= 5.0 + 1e-9
    ]
    assert len(mark_splits) >= 8
    runs = extract_mark_region_runs(dashed, model)
    assert all(length > 0.05 for length, _ in runs)
    assert not any(
        length < 0.25 and is_on for length, is_on in runs
    ), "no false short ON run in OFF gap"


def test_dash_densified_path_same_run_lengths():
    sparse = _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
        flags=[False, True, True, False],
    )
    dense = _build_path_model(
        points=[(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)],
        flags=[False, True, True, True, False],
    )
    sparse_dashed = apply_dash_pattern(
        sparse, 0.3, 0.3, DashPhaseReset.PER_MARK_REGION
    )
    dense_dashed = apply_dash_pattern(
        dense, 0.3, 0.3, DashPhaseReset.PER_MARK_REGION
    )
    sparse_runs = extract_mark_region_runs(sparse_dashed, sparse)
    dense_runs = extract_mark_region_runs(dense_dashed, dense)
    assert len(sparse_runs) == len(dense_runs)
    for (sl, sf), (dl, df) in zip(sparse_runs, dense_runs):
        assert abs(sl - dl) < _RUN_LEN_TOL
        assert sf is df


def test_dash_l_shape_boundary_at_corner():
    model = _build_path_model(
        points=[(0.0, 0.0), (0.6, 0.0), (0.6, 0.6)],
        flags=[True, True, True],
    )
    dashed = apply_dash_pattern(model, 0.3, 0.3, DashPhaseReset.PER_MARK_REGION)
    assert any(abs(s - 0.6) < 1e-6 for s in dashed.cumulative_s)
    runs = extract_mark_region_runs(dashed, model)
    assert runs
    assert all(length > 0.0 for length, _ in runs)


def test_dash_continuous_phase_across_regions():
    model = _build_path_model(
        points=[
            (0.0, 0.0),
            (0.6, 0.0),
            (1.2, 0.0),
            (1.8, 0.0),
            (2.4, 0.0),
            (3.0, 0.0),
            (3.6, 0.0),
        ],
        flags=[False, True, True, False, True, True, False],
    )
    dashed = apply_dash_pattern(model, 0.6, 0.4, DashPhaseReset.CONTINUOUS)
    second_region_mid = 2.7
    idx = max(
        i for i, s in enumerate(dashed.cumulative_s) if s <= second_region_mid + 1e-9
    )
    assert dashed.flags[idx] is False
    per_region = apply_dash_pattern(
        model, 0.6, 0.4, DashPhaseReset.PER_MARK_REGION
    )
    idx_reset = max(
        i for i, s in enumerate(per_region.cumulative_s) if s <= second_region_mid + 1e-9
    )
    assert per_region.flags[idx_reset] is True


def test_dash_per_region_phase_reset():
    model = _build_path_model(
        points=[(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)],
        flags=[False, True, False, True, False],
    )
    dashed = apply_dash_pattern(model, 0.3, 0.3, DashPhaseReset.PER_MARK_REGION)
    runs = extract_mark_region_runs(dashed, model)
    on_runs = [length for length, is_on in runs if is_on]
    assert all(abs(length - 0.3) < _RUN_LEN_TOL for length in on_runs)


def test_dash_very_small_valid_distances_near_tolerance():
    model = _build_path_model([(0.0, 0.0), (0.2, 0.0)], [True, True])
    dashed = apply_dash_pattern(model, 0.05, 0.05, DashPhaseReset.PER_MARK_REGION)
    runs = extract_mark_region_runs(dashed, model)
    assert runs
    assert all(length > 0.0 for length, _ in runs)


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print(f"ok {test.__name__}")
    print("PASS")


if __name__ == "__main__":
    main()