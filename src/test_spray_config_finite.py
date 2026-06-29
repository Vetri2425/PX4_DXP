"""Non-finite spray configuration rejection tests."""

from __future__ import annotations

import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from spray_config import (
    configuration_to_param_dict,
    parse_staged_spray_config,
    staged_spray_defaults,
    validate_spray_configuration,
)


def _base_config() -> dict:
    raw = staged_spray_defaults()
    raw.update(
        {
            "mission_id": "m1",
            "path_fingerprint": "fp1",
            "configuration_revision": 1,
        }
    )
    return raw


@pytest.mark.parametrize(
    "field,value",
    [
        ("max_spray_speed_mps", float("inf")),
        ("min_spray_speed_mps", float("-inf")),
        ("nozzle_forward_offset_m", float("nan")),
        ("actuator_max_pwm", float("inf")),
        ("target_paint_density", float("-inf")),
        ("dash_on_distance_m", float("nan")),
        ("point_default_dwell_s", float("inf")),
        ("solenoid_open_delay_s", float("-inf")),
        ("max_lead_distance_m", float("nan")),
        ("max_pwm_change_per_s", float("inf")),
    ],
)
def test_scalar_non_finite_rejected(field, value):
    raw = _base_config()
    raw[field] = value
    with pytest.raises(ValueError, match="finite"):
        validate_spray_configuration(raw)


def test_speed_pwm_table_rejects_non_finite_entries():
    raw = _base_config()
    raw["speed_pwm_table"] = [
        {"speed_mps": 0.05, "pwm": 1200.0},
        {"speed_mps": float("inf"), "pwm": 1800.0},
    ]
    with pytest.raises(ValueError, match="finite"):
        validate_spray_configuration(raw)


def test_actuator_limits_reject_non_finite_values():
    raw = _base_config()
    raw["actuator_min_value"] = float("inf")
    with pytest.raises(ValueError, match="finite"):
        validate_spray_configuration(raw)


def test_staged_request_rejects_non_finite_nested_mode_fields():
    staged = staged_spray_defaults()
    staged.update(
        {
            "mission_id": "m1",
            "path_fingerprint": "fp1",
            "configuration_revision": 1,
            "point_leg_spacing_m": float("nan"),
        }
    )
    with pytest.raises(ValueError, match="finite"):
        parse_staged_spray_config(staged)


def test_ros_param_dict_round_trip_stays_finite():
    cfg = validate_spray_configuration(_base_config())
    params = configuration_to_param_dict(cfg)
    table = json.loads(params["speed_pwm_table"])
    for row in table:
        assert math.isfinite(row["speed_mps"])
        assert math.isfinite(row["pwm"])
    for key in (
        "max_spray_speed_mps",
        "min_spray_speed_mps",
        "target_paint_density",
        "dash_on_distance_m",
    ):
        assert math.isfinite(params[key])