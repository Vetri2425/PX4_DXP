#!/usr/bin/env python3
"""Unit tests for the manual override logic in spray_controller_node.

Runs without ROS2: rclpy / mavros_msgs / std_msgs are stubbed, and the node
is instantiated without __init__ — only the pure decision logic is exercised
(manual precedence, node-side expiry, fail-safe priority, staleness scoping).
"""

from __future__ import annotations

import os
import sys
import types


def _install_ros_stubs() -> None:
    if "rclpy" in sys.modules:
        return

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.try_shutdown = lambda *a, **k: None
    sys.modules["rclpy"] = rclpy

    cbg = types.ModuleType("rclpy.callback_groups")
    cbg.ReentrantCallbackGroup = object
    sys.modules["rclpy.callback_groups"] = cbg

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    sys.modules["rclpy.node"] = rclpy_node

    rclpy_qos = types.ModuleType("rclpy.qos")

    class _Enum:
        BEST_EFFORT = RELIABLE = VOLATILE = TRANSIENT_LOCAL = KEEP_LAST = 1

    rclpy_qos.QoSProfile = lambda *a, **k: None
    rclpy_qos.ReliabilityPolicy = _Enum
    rclpy_qos.DurabilityPolicy = _Enum
    rclpy_qos.HistoryPolicy = _Enum
    sys.modules["rclpy.qos"] = rclpy_qos

    mavros_msgs = types.ModuleType("mavros_msgs")
    mavros_msg = types.ModuleType("mavros_msgs.msg")

    class _State:
        armed = False
        mode = ""

    mavros_msg.State = _State
    mavros_srv = types.ModuleType("mavros_msgs.srv")

    class _CommandLongRequest:
        def __init__(self):
            self.broadcast = False
            self.command = 0
            self.confirmation = 0
            self.param1 = self.param2 = self.param3 = 0.0
            self.param4 = self.param5 = self.param6 = self.param7 = 0.0

    class _CommandLong:
        Request = _CommandLongRequest

    mavros_srv.CommandLong = _CommandLong
    sys.modules["mavros_msgs"] = mavros_msgs
    sys.modules["mavros_msgs.msg"] = mavros_msg
    sys.modules["mavros_msgs.srv"] = mavros_srv

    std_msgs = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")

    class _Bool:
        def __init__(self):
            self.data = False

    std_msg.Bool = _Bool
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msg


_install_ros_stubs()
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from spray_controller_node import SprayControllerNode  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402  (stubbed)


# ── Test harness ──────────────────────────────────────────────────────────────

class _Param:
    def __init__(self, value):
        self.value = value


class _Clock:
    def __init__(self):
        self.ns = 1_000_000_000

    def now(self):
        clock = self

        class _T:
            nanoseconds = clock.ns

            def __sub__(self, other):
                class _D:
                    nanoseconds = clock.ns - other.nanoseconds
                return _D()

        return _T()


class _Logger:
    def info(self, *a, **k):
        pass

    warn = info


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(bool(msg.data))


class _Future:
    def add_done_callback(self, cb):
        pass


class _Cli:
    def __init__(self):
        self.requests = []

    def call_async(self, req):
        self.requests.append(req)
        return _Future()


def make_node(armed=True, mode="OFFBOARD", require_offboard=True):
    node = SprayControllerNode.__new__(SprayControllerNode)
    node._params = {
        "actuator_set_index": _Param(1),
        "on_value": _Param(1.0),
        "off_value": _Param(-1.0),
        "debounce_samples": _Param(1),
        "reassert_hz": _Param(0.0),
        "require_offboard": _Param(require_offboard),
        "active_timeout_s": _Param(0.5),
        "manual_override_timeout_s": _Param(10.0),
    }
    node.get_parameter = lambda name: node._params[name]
    node._clock = _Clock()
    node.get_clock = lambda: node._clock
    node.get_logger = lambda: _Logger()
    node._command_cli = _Cli()
    node._state_pub = _Pub()
    node._manual_state_pub = _Pub()
    node._desired_raw = False
    node._candidate = None
    node._candidate_count = 0
    node._desired_debounced = False
    node._commanded = False
    node._last_active_time = None
    node._manual_active = False
    node._manual_deadline_ns = None
    node._armed = armed
    node._mode = mode
    node._service_ready = True
    return node


def _bool_msg(data):
    msg = Bool()
    msg.data = data
    return msg


def _last_param1(node):
    return node._command_cli.requests[-1].param1


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_manual_on_commands_actuator_and_sets_deadline():
    node = make_node()
    node._manual_cb(_bool_msg(True))
    assert node._manual_active is True
    assert node._manual_deadline_ns == node._clock.ns + 10_000_000_000
    assert node._commanded is True
    assert _last_param1(node) == 1.0
    assert node._manual_state_pub.msgs[-1] is True


def test_manual_on_rejected_when_disarmed():
    node = make_node(armed=False)
    node._manual_cb(_bool_msg(True))
    assert node._manual_active is False
    assert node._commanded is False
    assert all(p.param1 != 1.0 for p in node._command_cli.requests)


def test_manual_on_rejected_when_not_offboard():
    node = make_node(mode="MANUAL", require_offboard=True)
    node._manual_cb(_bool_msg(True))
    assert node._manual_active is False
    assert node._commanded is False


def test_manual_off_cancels_and_reverts_to_auto_off():
    node = make_node()
    node._manual_cb(_bool_msg(True))
    node._manual_cb(_bool_msg(False))
    assert node._manual_active is False
    assert node._commanded is False
    assert _last_param1(node) == -1.0


def test_manual_expires_via_watchdog():
    node = make_node()
    node._manual_cb(_bool_msg(True))
    node._clock.ns += 10_500_000_000  # past the 10 s deadline
    node._watchdog_tick()
    assert node._manual_active is False
    assert node._commanded is False
    assert _last_param1(node) == -1.0


def test_manual_survives_spray_active_staleness():
    node = make_node()
    node._active_cb(_bool_msg(False))  # auto stream alive, desires OFF
    node._manual_cb(_bool_msg(True))
    assert node._commanded is True
    node._clock.ns += 1_000_000_000  # /spray/active now stale (>0.5 s)
    node._watchdog_tick()
    # Staleness clears the auto desire but not the (self-timed) manual hold.
    assert node._manual_active is True
    assert node._commanded is True


def test_disarm_failsafe_outranks_manual():
    node = make_node()
    node._manual_cb(_bool_msg(True))
    assert node._commanded is True
    state = types.SimpleNamespace(armed=False, mode="OFFBOARD")
    node._state_cb(state)
    assert node._manual_active is False
    assert node._commanded is False
    assert _last_param1(node) == -1.0


def test_auto_stream_off_does_not_override_manual_on():
    node = make_node()
    node._manual_cb(_bool_msg(True))
    for _ in range(5):
        node._active_cb(_bool_msg(False))  # RPP keeps saying TRANSIT
    assert node._commanded is True  # manual still wins
    node._manual_cb(_bool_msg(False))
    assert node._commanded is False


def test_shutdown_clears_manual_and_sends_off():
    node = make_node()
    node._manual_cb(_bool_msg(True))
    node.shutdown_off()
    assert node._manual_active is False
    assert _last_param1(node) == -1.0


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"ok {t.__name__}")
    print("PASS")


if __name__ == "__main__":
    main()
