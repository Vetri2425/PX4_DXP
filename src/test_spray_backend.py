#!/usr/bin/env python3
"""Unit tests for spray actuator backend selection.

Tests _build_command_request / _build_actuator_request / _build_servo_pwm_request
without ROS2. Verifies that mavlink_servo_pwm (cmd 183) and mavlink_actuator
(cmd 187) produce correct CommandLong fields for every path that sends a command
(startup OFF, shutdown OFF, ON, OFF retry, unknown-backend fail-safe).
"""

from __future__ import annotations

import math
import os
import sys

from test_spray_manual_override import _Cli, _Param, make_node  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from spray_controller_node import (  # noqa: E402
    MAV_CMD_DO_SET_ACTUATOR,
    MAV_CMD_DO_SET_SERVO,
    _SERVO_PWM_MAX_US,
    SprayControllerNode,
)


# ── helpers ───────────────────────────────────────────────────────────────────

def _servo_node(
    servo_instance: int = 1,
    off_pwm_us: int = 0,
    on_pwm_us: int = 2500,
) -> SprayControllerNode:
    node = make_node()
    node._params["actuator_backend"] = _Param("mavlink_servo_pwm")
    node._params["servo_instance"] = _Param(servo_instance)
    node._params["off_pwm_us"] = _Param(off_pwm_us)
    node._params["on_pwm_us"] = _Param(on_pwm_us)
    # legacy params must exist even for servo backend (make_node sets them)
    return node


def _actuator_node(
    set_index: int = 1,
    on_value: float = 1.0,
    off_value: float = -1.0,
) -> SprayControllerNode:
    node = make_node()
    node._params["actuator_backend"] = _Param("mavlink_actuator")
    node._params["actuator_set_index"] = _Param(set_index)
    node._params["on_value"] = _Param(on_value)
    node._params["off_value"] = _Param(off_value)
    return node


def _req(node: SprayControllerNode, on: bool):
    return node._build_command_request(on)


# ── mavlink_servo_pwm backend ─────────────────────────────────────────────────

def test_servo_pwm_off_command():
    node = _servo_node(servo_instance=1, off_pwm_us=0, on_pwm_us=1800)
    req = _req(node, on=False)
    assert req.command == MAV_CMD_DO_SET_SERVO, f"expected 183, got {req.command}"
    assert req.param1 == 1.0, f"expected param1=1.0 (instance), got {req.param1}"
    assert req.param2 == 0.0, f"expected param2=0 (OFF PWM), got {req.param2}"
    assert req.param3 == 0.0
    assert req.param7 == 0.0
    print("PASS test_servo_pwm_off_command")


def test_servo_pwm_on_command():
    node = _servo_node(servo_instance=1, off_pwm_us=0, on_pwm_us=2500)
    req = _req(node, on=True)
    assert req.command == MAV_CMD_DO_SET_SERVO
    assert req.param1 == 1.0
    assert req.param2 == 2500.0, f"expected param2=2500, got {req.param2}"
    print("PASS test_servo_pwm_on_command")


def test_servo_pwm_on_2000():
    node = _servo_node(on_pwm_us=2000)
    req = _req(node, on=True)
    assert req.param2 == 2000.0
    print("PASS test_servo_pwm_on_2000")


def test_servo_pwm_on_clamped():
    """on_pwm_us above _SERVO_PWM_MAX_US is silently clamped."""
    node = _servo_node(on_pwm_us=9999)
    req = _req(node, on=True)
    assert req.param2 == float(_SERVO_PWM_MAX_US), (
        f"expected clamped to {_SERVO_PWM_MAX_US}, got {req.param2}"
    )
    print(f"PASS test_servo_pwm_on_clamped (clamped to {_SERVO_PWM_MAX_US})")


def test_servo_pwm_instance_2():
    """servo_instance propagates correctly."""
    node = _servo_node(servo_instance=2)
    req_off = _req(node, on=False)
    req_on = _req(node, on=True)
    assert req_off.param1 == 2.0
    assert req_on.param1 == 2.0
    print("PASS test_servo_pwm_instance_2")


def test_servo_pwm_startup_off_is_pwm_zero():
    """Simulates the startup OFF call: _build_command_request(on=False) must send PWM 0."""
    node = _servo_node(off_pwm_us=0, on_pwm_us=1800)
    req = _req(node, on=False)
    assert req.command == MAV_CMD_DO_SET_SERVO
    assert req.param2 == 0.0, f"startup OFF must be PWM 0, got {req.param2}"
    print("PASS test_servo_pwm_startup_off_is_pwm_zero")


def test_servo_pwm_shutdown_off_is_pwm_zero():
    """Same path as startup — shutdown_off calls _send_command(False)."""
    node = _servo_node(off_pwm_us=0)
    req = _req(node, on=False)
    assert req.param2 == 0.0
    print("PASS test_servo_pwm_shutdown_off_is_pwm_zero")


def test_servo_pwm_off_retry_is_pwm_zero():
    """OFF retry goes through _build_command_request(on=False); must send PWM 0."""
    node = _servo_node(off_pwm_us=0)
    for _ in range(3):
        req = _req(node, on=False)
        assert req.param2 == 0.0
    print("PASS test_servo_pwm_off_retry_is_pwm_zero")


def test_servo_pwm_non_nan_params():
    """CMD 183 must not have NaN in any field (unlike CMD 187 which uses NaN for unused slots)."""
    node = _servo_node()
    for on in (True, False):
        req = _req(node, on=on)
        for field in ("param1", "param2", "param3", "param4", "param5", "param6", "param7"):
            v = getattr(req, field)
            assert not math.isnan(v), f"CMD 183 field {field} must not be NaN, got {v}"
    print("PASS test_servo_pwm_non_nan_params")


# ── mavlink_actuator backend (legacy) ─────────────────────────────────────────

def test_actuator_off_command():
    node = _actuator_node(set_index=1, off_value=-1.0)
    req = _req(node, on=False)
    assert req.command == MAV_CMD_DO_SET_ACTUATOR, f"expected 187, got {req.command}"
    assert req.param1 == -1.0, f"expected param1=-1.0, got {req.param1}"
    assert math.isnan(req.param2)
    print("PASS test_actuator_off_command")


def test_actuator_on_command():
    node = _actuator_node(set_index=1, on_value=1.0)
    req = _req(node, on=True)
    assert req.command == MAV_CMD_DO_SET_ACTUATOR
    assert req.param1 == 1.0
    assert math.isnan(req.param2)
    print("PASS test_actuator_on_command")


def test_actuator_set_index_2():
    node = _actuator_node(set_index=2, on_value=1.0)
    req = _req(node, on=True)
    assert math.isnan(req.param1), "param1 (index 0) should be NaN when set_index=2"
    assert req.param2 == 1.0, "param2 (index 1) should hold the value for set_index=2"
    print("PASS test_actuator_set_index_2")


def test_actuator_bad_index_clamps():
    node = _actuator_node(set_index=99, on_value=1.0)
    req = _req(node, on=True)
    assert req.command == MAV_CMD_DO_SET_ACTUATOR
    assert req.param1 == 1.0, "bad index should fall back to set_index=1"
    print("PASS test_actuator_bad_index_clamps")


# ── unknown backend fail-safe ─────────────────────────────────────────────────

def test_unknown_backend_fails_safe():
    """Unknown backend must not raise and must produce an OFF-equivalent command."""
    node = make_node()
    node._params["actuator_backend"] = _Param("bogus_backend")
    node._params["servo_instance"] = _Param(1)
    node._params["off_pwm_us"] = _Param(0)
    node._params["on_pwm_us"] = _Param(1800)
    # Requesting ON with an unknown backend must degrade to OFF (safe)
    req = _req(node, on=True)
    assert req.command == MAV_CMD_DO_SET_SERVO, "unknown backend should fall back to servo cmd"
    assert req.param2 == 0.0, "unknown backend should fall back to OFF (PWM 0)"
    print("PASS test_unknown_backend_fails_safe")


# ── run all ───────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    test_servo_pwm_off_command()
    test_servo_pwm_on_command()
    test_servo_pwm_on_2000()
    test_servo_pwm_on_clamped()
    test_servo_pwm_instance_2()
    test_servo_pwm_startup_off_is_pwm_zero()
    test_servo_pwm_shutdown_off_is_pwm_zero()
    test_servo_pwm_off_retry_is_pwm_zero()
    test_servo_pwm_non_nan_params()
    test_actuator_off_command()
    test_actuator_on_command()
    test_actuator_set_index_2()
    test_actuator_bad_index_clamps()
    test_unknown_backend_fails_safe()
    print("\nAll backend tests PASSED")
