#!/usr/bin/env python3
"""Unit tests for spray configuration validation."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from spray_config import (
    SprayMode,
    UnsafeSpeedBehavior,
    configuration_to_param_dict,
    interpolate_speed_pwm,
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
        "timing_only_compatibility": True,
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


def test_speed_window_and_policy_defaults():
    cfg = validate_spray_configuration({})
    assert cfg.continuous.max_spray_speed_mps == 1.0
    assert cfg.continuous.unsafe_speed_behavior == UnsafeSpeedBehavior.BLOCK_SPRAY


def test_speed_pwm_interpolation():
    cfg = validate_spray_configuration(
        {
            "speed_pwm_table": [
                {"speed_mps": 0.2, "pwm": 1200},
                {"speed_mps": 0.4, "pwm": 2000},
            ]
        }
    )
    assert interpolate_speed_pwm(0.3, cfg.calibration.speed_pwm_table) == 1600.0


def test_speed_pwm_table_must_be_monotonic():
    try:
        validate_spray_configuration(
            {
                "speed_pwm_table": [
                    {"speed_mps": 0.3, "pwm": 1200},
                    {"speed_mps": 0.2, "pwm": 1500},
                ]
            }
        )
        assert False
    except ValueError as exc:
        assert "strictly increasing" in str(exc)


def test_pwm_must_be_inside_actuator_limits():
    try:
        validate_spray_configuration(
            {
                "actuator_max_pwm": 1800,
                "speed_pwm_table": [
                    {"speed_mps": 0.2, "pwm": 1200},
                    {"speed_mps": 0.4, "pwm": 2000},
                ],
            }
        )
        assert False
    except ValueError as exc:
        assert "within actuator limits" in str(exc)


def test_staged_path_fingerprint_propagates():
    cfg = parse_staged_spray_config(
        {
            "mission_id": "stg_test",
            "path_fingerprint": "abc123",
            "configuration_revision": 1,
            "waypoints": [[0.0, 0.0], [1.0, 0.0]],
            "spray_flags": [False, True],
        }
    )
    assert cfg.path_fingerprint == "abc123"


def test_timing_only_compatibility_defaults_false():
    cfg = validate_spray_configuration({})
    assert cfg.calibration.timing_only_compatibility is False


def test_mission_bound_config_requires_fingerprint_and_revision_by_default():
    try:
        validate_spray_configuration({"mission_id": "stg_test"})
        assert False
    except ValueError as exc:
        assert "path_fingerprint" in str(exc)
    try:
        validate_spray_configuration({"mission_id": "stg_test", "path_fingerprint": "fp"})
        assert False
    except ValueError as exc:
        assert "configuration_revision" in str(exc)


def test_explicit_legacy_timing_only_compatibility_allows_missing_fingerprint():
    cfg = validate_spray_configuration(
        {"mission_id": "legacy", "timing_only_compatibility": True}
    )
    assert cfg.calibration.timing_only_compatibility is True


def test_pause_mission_speed_policy_rejected_until_lifecycle_pause_exists():
    try:
        validate_spray_configuration({"unsafe_speed_behavior": "PAUSE_MISSION"})
        assert False
    except ValueError as exc:
        assert "BLOCK_SPRAY or CLAMP_PWM" in str(exc)


def test_staged_calibration_propagates_to_ros_params():
    cfg = validate_spray_configuration(
        {
            "mission_id": "stg_test",
            "path_fingerprint": "fp",
            "configuration_revision": 1,
            "calibration_profile_id": "bench_a",
            "calibration_profile_version": 2,
            "target_paint_density": 1.4,
            "speed_pwm_table": [
                {"speed_mps": 0.2, "pwm": 1300},
                {"speed_mps": 0.4, "pwm": 1900},
            ],
            "max_spray_speed_mps": 0.5,
            "unsafe_speed_behavior": "CLAMP_PWM",
        }
    )
    params = configuration_to_param_dict(cfg)
    assert params["mission_config_path_fingerprint"] == "fp"
    assert params["calibration_profile_id"] == "bench_a"
    assert params["calibration_profile_version"] == 2
    assert params["target_paint_density"] == 1.4
    assert "1300" in params["speed_pwm_table"]
    assert params["unsafe_speed_behavior"] == "CLAMP_PWM"


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print(f"ok {test.__name__}")
    print("PASS")


if __name__ == "__main__":
    main()
