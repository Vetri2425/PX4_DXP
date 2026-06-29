#!/usr/bin/env python3
"""Regression: continuous mode must match pre-feature distance-aware decisions."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from spray_config import SprayConfiguration, SprayMode
from spray_controller_modes import ContinuousSprayRuntimeState, continuous_distance_decision
from spray_path_model import (
    SprayProjectionState,
    build_path_model as _build_path_model,
    make_spray_decision as _make_spray_decision,
)


def _straight_mark_path():
    return _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
        flags=[False, True, True, False],
    )


def _runtime_state() -> ContinuousSprayRuntimeState:
    return ContinuousSprayRuntimeState(projection=SprayProjectionState())


def test_continuous_dispatch_matches_legacy_engine():
    model = _straight_mark_path()
    config = SprayConfiguration(mode=SprayMode.CONTINUOUS)
    table = config.calibration.speed_pwm_table
    table_max_speed = table[-1].speed_mps if table else config.continuous.max_spray_speed_mps
    shared = {
        "solenoid_open_delay_s": config.continuous.solenoid_open_delay_s,
        "solenoid_close_delay_s": config.continuous.solenoid_close_delay_s,
        "on_overspray_margin_m": config.continuous.on_overspray_margin_m,
        "off_overspray_margin_m": config.continuous.off_overspray_margin_m,
        "max_xtrack_error_m": config.continuous.max_xtrack_error_m,
        "min_spray_speed_mps": config.continuous.min_spray_speed_mps,
        "max_spray_speed_mps": config.continuous.max_spray_speed_mps,
        "speed_pwm_table_max_speed": table_max_speed,
        "high_speed_underflow_behavior": config.continuous.high_speed_underflow_behavior,
    }
    legacy = _make_spray_decision(
        model=model,
        nozzle_n=0.97,
        nozzle_e=0.0,
        vel_n=0.35,
        vel_e=0.0,
        safety_ok=True,
        safety_reason="",
        **shared,
    )
    dispatched = continuous_distance_decision(
        model=model,
        pose_ned=(0.97, 0.0, 0.0),
        vel_ned=(0.35, 0.0),
        safety_ok=True,
        safety_reason="",
        config=config,
        runtime_state=_runtime_state(),
    )
    assert dispatched.desired == legacy.desired
    assert dispatched.event == legacy.event
    assert dispatched.geometry_desired == legacy.geometry_desired


def main():
    test_continuous_dispatch_matches_legacy_engine()
    print("ok test_continuous_dispatch_matches_legacy_engine")
    print("PASS")


if __name__ == "__main__":
    main()