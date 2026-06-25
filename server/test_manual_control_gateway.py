import os
import sys
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from manual_control_gateway import (
    ManualControlGateway,
    MavrosManualControlTransport,
    NEUTRAL_FRAME,
    encode_manual_control,
)


class FakeTransport:
    name = "fake"

    def __init__(self, healthy=True):
        self.healthy = healthy
        self.frames = []

    def is_healthy(self):
        return self.healthy

    def health_reason(self):
        return "" if self.healthy else "fake unhealthy"

    def send_frame(self, frame):
        self.frames.append(frame)

    def shutdown(self):
        pass


def test_manual_control_encoding_axes():
    assert encode_manual_control(0.0, 0.0) == NEUTRAL_FRAME
    assert encode_manual_control(1.0, 0.0).z == 1000
    assert encode_manual_control(-1.0, 0.0).z == 0
    assert encode_manual_control(0.0, 1.0).y == 1000
    assert encode_manual_control(0.0, -0.5).y == -500
    combined = encode_manual_control(0.5, 0.25)
    assert combined.z == 750
    assert combined.y == 250


def test_mavros_transport_unavailable_fails_closed_on_mac_without_interface():
    transport = MavrosManualControlTransport(ros_node=None)
    assert transport.is_healthy() is False
    assert "ROS node not available" in transport.health_reason()


def test_mavros_transport_repeated_publish_errors_fail_closed():
    class FakeManualControl:
        pass

    class FailingPublisher:
        def publish(self, msg):
            raise RuntimeError("publish broke")

    transport = MavrosManualControlTransport.__new__(MavrosManualControlTransport)
    transport._ros_node = SimpleNamespace(count_subscribers=lambda topic: 1)
    transport._topic = "/mavros/manual_control/send"
    transport._require_subscriber = False
    transport._publisher = FailingPublisher()
    transport._msg_type = FakeManualControl
    transport._reason = ""
    transport._publish_errors = 0
    transport._publish_error_limit = 2

    with pytest.raises(RuntimeError):
        transport.send_frame(NEUTRAL_FRAME)
    assert transport.is_healthy() is True
    with pytest.raises(RuntimeError):
        transport.send_frame(NEUTRAL_FRAME)
    assert transport.is_healthy() is False
    assert "publish failed 2 consecutive times" in transport.health_reason()


def test_gateway_stale_timeout_replaces_nonzero_with_neutral():
    transport = FakeTransport()
    gateway = ManualControlGateway(transport, rate_hz=100.0, stale_timeout_s=0.04)
    gateway.start()
    try:
        gateway.accept_command(0.5, 0.5)
        time.sleep(0.025)
        assert any(frame != NEUTRAL_FRAME for frame in transport.frames)
        time.sleep(0.08)
        assert transport.frames[-1] == NEUTRAL_FRAME
    finally:
        gateway.shutdown()


def test_gateway_shutdown_sends_neutral():
    transport = FakeTransport()
    gateway = ManualControlGateway(transport, rate_hz=100.0, stale_timeout_s=0.04)
    gateway.accept_command(0.5, 0.5)
    gateway.shutdown()
    assert transport.frames[-1] == NEUTRAL_FRAME
