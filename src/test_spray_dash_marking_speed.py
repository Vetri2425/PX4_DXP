"""DASH feasibility must use staged marking speed, not max spray speed."""

from __future__ import annotations

import os
import sys
from dataclasses import replace

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from spray_config import SprayConfiguration, SprayMode, validate_spray_configuration
from spray_dash import validate_dash_feasibility
from spray_path_model import build_path_model


def _dash_config(*, max_speed: float = 1.0) -> SprayConfiguration:
    raw = {
        "mission_id": "dash",
        "path_fingerprint": "fp",
        "configuration_revision": 1,
        "spray_mode": "dash",
        "dash_on_distance_m": 0.30,
        "dash_off_distance_m": 0.30,
        "max_spray_speed_mps": max_speed,
    }
    return validate_spray_configuration(raw)


def _one_meter_mark():
    return build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
        flags=[False, True, True, False],
    )


def test_same_geometry_feasible_at_low_marking_speed():
    model = _one_meter_mark()
    config = _dash_config(max_speed=1.0)
    low = validate_dash_feasibility(model, config, expected_speed_mps=0.35)
    assert low.dash_feasible is True
    assert low.dash_expected_speed_mps == pytest.approx(0.35)
    assert low.dash_feasibility_speed_source == "staged_marking_speed_mps"


def test_same_geometry_infeasible_at_higher_marking_speed():
    model = _one_meter_mark()
    config = replace(
        _dash_config(max_speed=1.0),
        dash=replace(_dash_config().dash, on_distance_m=0.10, off_distance_m=0.10),
    )
    high = validate_dash_feasibility(model, config, expected_speed_mps=0.90)
    assert high.dash_feasible is False
    assert high.dash_feasibility_speed_source == "staged_marking_speed_mps"


def test_staged_marking_speed_differs_from_max_spray_speed():
    model = _one_meter_mark()
    config = _dash_config(max_speed=1.0)
    by_max = validate_dash_feasibility(model, config)
    by_mark = validate_dash_feasibility(model, config, expected_speed_mps=0.35)
    assert by_max.dash_feasibility_speed_source == "max_spray_speed_mps"
    assert by_mark.dash_feasibility_speed_source == "staged_marking_speed_mps"
    assert by_max.dash_expected_speed_mps == pytest.approx(1.0)
    assert by_mark.dash_expected_speed_mps == pytest.approx(0.35)