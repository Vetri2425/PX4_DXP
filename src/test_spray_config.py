#!/usr/bin/env python3
"""Unit tests for spray configuration validation."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from spray_config import (
    SprayMode,
    parse_staged_spray_config,
    staged_spray_defaults,
    validate_spray_configuration,
)


def test_defaults_continuous():
    cfg = validate_spray_configuration({})
    assert cfg.mode == SprayMode.CONTINUOUS


def test_invalid_mode_rejected():
    try:
        validate_spray_configuration({"spray_mode": "zigzag"})
        assert False
    except ValueError:
        pass


def test_dash_requires_positive_distance():
    try:
        validate_spray_configuration(
            {"spray_mode": "dash", "dash_on_distance_m": 0.0, "dash_off_distance_m": 0.0}
        )
        assert False
    except ValueError:
        pass


def test_backward_compatible_staged_missing_mode():
    staged = {
        "mission_id": "stg_test",
        "waypoints": [[0.0, 0.0], [1.0, 0.0]],
        "spray_flags": [True, True],
    }
    cfg = parse_staged_spray_config(staged)
    assert cfg.mode == SprayMode.CONTINUOUS


def test_staged_defaults_include_spray_mode():
    defaults = staged_spray_defaults()
    assert defaults["spray_mode"] == "continuous"


def test_point_max_dwell_defaults():
    cfg = validate_spray_configuration({})
    assert cfg.point.max_dwell_s == 60.0


def test_point_leg_trajectory_defaults():
    cfg = validate_spray_configuration({})
    assert cfg.point.leg_trajectory_mode == "two_point"
    assert cfg.point.leg_spacing_m == 0.08
    assert cfg.point.hold_drift_tolerance_m == 0.08
    assert cfg.point.hold_drift_policy == "fail"


def test_gps_safety_defaults():
    cfg = validate_spray_configuration({})
    assert cfg.gps_safety.required_fix_type == 6
    assert cfg.gps_safety.runtime_policy == "pause"
    assert cfg.gps_safety.resume_policy == "manual"


def test_default_dwell_exceeding_max_rejected():
    try:
        validate_spray_configuration(
            {"point_default_dwell_s": 70.0, "point_max_dwell_s": 60.0}
        )
        assert False
    except ValueError as exc:
        assert "point_default_dwell_s exceeds point_max_dwell_s" in str(exc)


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print(f"ok {test.__name__}")
    print("PASS")


if __name__ == "__main__":
    main()