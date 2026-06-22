#!/usr/bin/env python3
"""Unit tests for dash spray pattern transform."""

from __future__ import annotations

import copy
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from spray_config import DashPhaseReset
from spray_path_model import build_path_model as _build_path_model
from spray_dash import apply_dash_pattern


def _mark_path():
    return _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
        flags=[False, True, True, False],
    )


def test_straight_mark_region_dash_boundaries():
    model = apply_dash_pattern(_mark_path(), 0.3, 0.3, DashPhaseReset.PER_MARK_REGION)
    assert model.flags[0] is False
    assert model.flags[1] is True
    assert model.flags[2] is False  # local_s=1.0 lands in OFF phase
    assert model.flags[3] is False


def test_transit_always_off():
    model = apply_dash_pattern(_mark_path(), 0.3, 0.3, DashPhaseReset.PER_MARK_REGION)
    assert model.flags[0] is False
    assert model.flags[3] is False


def test_multiple_mark_regions_per_region_reset():
    model = _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)],
        flags=[False, True, False, True, False],
    )
    dashed = apply_dash_pattern(model, 0.5, 0.5, DashPhaseReset.PER_MARK_REGION)
    assert dashed.flags[1] is True
    assert dashed.flags[3] is True


def test_continuous_phase_carries_across_regions():
    model = _build_path_model(
        points=[(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)],
        flags=[False, True, False, True, False],
    )
    dashed = apply_dash_pattern(model, 0.6, 0.4, DashPhaseReset.CONTINUOUS)
    assert dashed.flags[1] is True
    assert dashed.flags[4] is False


def test_l_shape_cumulative_distance():
    model = _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
        flags=[True, True, True],
    )
    dashed = apply_dash_pattern(model, 0.3, 0.4, DashPhaseReset.PER_MARK_REGION)
    assert dashed.flags[0] is True   # local_s=0 in ON phase
    assert dashed.flags[1] is False  # local_s=1 → period pos 0.3 (OFF phase)
    assert dashed.flags[2] is False  # local_s=2 → period pos 0.6 (OFF phase)


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


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print(f"ok {test.__name__}")
    print("PASS")


if __name__ == "__main__":
    main()