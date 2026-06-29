"""Production correction tests: disabled flow dry-run and MAVROS state freshness."""

from __future__ import annotations

import os
import sys
import time
from dataclasses import replace

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from spray_config import SprayConfiguration
from spray_controller_modes import ContinuousSprayRuntimeState, continuous_distance_decision
from spray_path_model import SprayProjectionState, build_path_model


class _Param:
    def __init__(self, value):
        self.value = value


def _make_node_for_state_tests():
    from test_spray_manual_override import make_node

    return make_node(armed=True, mode="OFFBOARD")


def test_disabled_flow_geometry_on_does_not_command_spray():
    points = [(0.0, 0.0), (2.0, 0.0)]
    flags = [True, True]
    model = build_path_model(points, flags)
    config = replace(
        SprayConfiguration(),
        continuous=replace(SprayConfiguration().continuous, flow_mode="disabled"),
    )
    runtime = ContinuousSprayRuntimeState(projection=SprayProjectionState())
    decision = continuous_distance_decision(
        model=model,
        pose_ned=(0.5, 0.0, 0.0),
        vel_ned=(0.35, 0.0),
        safety_ok=True,
        safety_reason="",
        config=config,
        runtime_state=runtime,
        on_value=1.0,
    )
    assert decision.geometry_desired is True
    assert decision.desired is False
    assert decision.target_pwm == 0.0
    assert decision.actuator_value == config.calibration.actuator_limits.off_value
    assert decision.target_flow == 0.0


def test_fresh_vehicle_state_permits_on():
    node = _make_node_for_state_tests()
    node._params["spray_enabled"] = _Param(True)
    node._state_recv_monotonic_s = time.monotonic()
    assert node._safety_allows_on() is True


def test_stale_vehicle_state_blocks_on():
    node = _make_node_for_state_tests()
    node._params["spray_enabled"] = _Param(True)
    node._params["vehicle_state_timeout_s"] = _Param(0.05)
    node._state_recv_monotonic_s = time.monotonic() - 1.0
    assert node._safety_allows_on() is False
    fresh, age, reason = node._vehicle_state_freshness()
    assert fresh is False
    assert reason == "vehicle state stale"
    assert age > 0.05


def test_stale_vehicle_state_forces_off_while_commanded():
    node = _make_node_for_state_tests()
    node._params["spray_enabled"] = _Param(True)
    node._params["vehicle_state_timeout_s"] = _Param(0.01)
    node._state_recv_monotonic_s = time.monotonic() - 1.0
    node._commanded = True
    node._desired_debounced = True
    node._watchdog_tick()
    assert node._commanded is False or node._actuator_state.desired_on is False


def test_fresh_state_restores_eligibility_without_auto_on():
    node = _make_node_for_state_tests()
    node._params["spray_enabled"] = _Param(True)
    node._params["vehicle_state_timeout_s"] = _Param(0.05)
    node._state_recv_monotonic_s = time.monotonic() - 1.0
    assert node._safety_allows_on() is False
    node._state_recv_monotonic_s = time.monotonic()
    assert node._safety_allows_on() is True
    assert node._commanded is False


def test_non_offboard_and_disarmed_remain_blocked():
    from test_spray_manual_override import make_node

    node = make_node(armed=False, mode="OFFBOARD")
    node._state_recv_monotonic_s = time.monotonic()
    assert node._safety_allows_on() is False
    node = make_node(armed=True, mode="MANUAL", require_offboard=True)
    node._state_recv_monotonic_s = time.monotonic()
    assert node._safety_allows_on() is False