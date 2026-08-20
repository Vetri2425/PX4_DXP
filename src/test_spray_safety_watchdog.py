#!/usr/bin/env python3
"""ROS-stubbed tests for the watchdog's only physical action: actuator OFF."""

import time

# Installs the shared rclpy/mavros/std_msgs stubs before importing the node.
import test_spray_manual_override  # noqa: F401

from spray_safety_lease import (
    SprayLeaseMonitor,
    SpraySafetyLease,
    lease_to_json,
)
from spray_safety_watchdog_node import (
    MAV_CMD_DO_SET_ACTUATOR,
    MAV_CMD_DO_SET_SERVO,
    SpraySafetyWatchdogNode,
)


def _node(mapping: SpraySafetyLease):
    node = SpraySafetyWatchdogNode.__new__(SpraySafetyWatchdogNode)
    node._monitor = SprayLeaseMonitor()
    node._monitor.last_lease = mapping
    node._fallback = mapping
    return node


def test_watchdog_actuator_backend_can_only_build_off():
    node = _node(
        SpraySafetyLease(False, 4, "mavlink_actuator", 2, -1.0, 1, 0)
    )
    req = node._build_off_request()
    assert req.command == MAV_CMD_DO_SET_ACTUATOR
    assert req.param2 == -1.0
    assert req.param1 != -1.0
    assert req.param7 == 0.0


def test_watchdog_servo_backend_can_only_build_off():
    node = _node(
        SpraySafetyLease(False, 4, "mavlink_servo_pwm", 1, -1.0, 3, 900)
    )
    req = node._build_off_request()
    assert req.command == MAV_CMD_DO_SET_SERVO
    assert req.param1 == 3.0
    assert req.param2 == 900.0
    assert req.param3 == 0.0
    assert req.param7 == 0.0


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


class _Response:
    success = True
    result = 0


class _Future:
    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return _Response()

    def cancel(self):
        return True


class _Client:
    def __init__(self):
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return _Future()


def _tick_node(mapping: SpraySafetyLease):
    node = _node(mapping)
    node._params = {
        "command_ack_timeout_s": _Param(1.0),
        "off_retry_hz": _Param(2.0),
        "off_burst_hz": _Param(20.0),
        "off_burst_duration_s": _Param(1.5),
    }
    node.get_parameter = lambda name: node._params[name]
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    node._command_cli = _Client()
    node._inflight = None
    node._inflight_since_s = None
    node._off_confirmed = False
    node._next_off_s = 0.0
    node._off_burst_until_s = 0.0
    node._last_reason = None
    node._last_status_s = time.monotonic()
    node._shutdown = False
    return node


def test_missing_lease_dispatches_independent_off():
    mapping = SpraySafetyLease(False, 0, "mavlink_actuator", 1, -1.0, 1, 0)
    node = _tick_node(mapping)
    node._tick()
    assert len(node._command_cli.requests) == 1
    assert node._command_cli.requests[0].command == MAV_CMD_DO_SET_ACTUATOR
    assert node._command_cli.requests[0].param1 == -1.0


def test_fresh_explicit_on_lease_is_the_only_case_without_off_dispatch():
    mapping = SpraySafetyLease(True, 8, "mavlink_actuator", 1, -1.0, 1, 0)
    node = _tick_node(mapping)
    node._monitor.observe(lease_to_json(mapping), time.monotonic())
    node._tick()
    assert node._command_cli.requests == []


def test_successful_off_ack_proves_independent_off_authority():
    mapping = SpraySafetyLease(False, 0, "mavlink_actuator", 1, -1.0, 1, 0)
    node = _tick_node(mapping)
    future = _Future()
    node._inflight = future
    node._inflight_since_s = time.monotonic()
    node._off_done(future)
    assert node._off_confirmed is True
