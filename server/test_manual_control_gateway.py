"""ManualControlGateway unit tests (plan phase J1).

Golden-frame tests document the CURRENT (§7.1-unverified) axis encoding so a
future bench-verified correction is an explicit, visible diff here — not a
silent behaviour change. The sender-thread tests use a fast rate/timeout so
they run in well under a second.
"""
from __future__ import annotations

import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from manual_control_gateway import (
    ManualControlFrame,
    ManualControlGateway,
    NEUTRAL_FRAME,
    encode_manual_control,
)


class FakeTransport:
    name = "fake"

    def __init__(self, *, healthy=True):
        self._healthy = healthy
        self.sent: list[ManualControlFrame] = []
        self.shutdown_called = False

    def is_healthy(self):
        return self._healthy

    def health_reason(self):
        return "" if self._healthy else "fake transport down"

    def send_frame(self, frame):
        self.sent.append(frame)

    def shutdown(self):
        self.shutdown_called = True


# ── encode_manual_control golden frames ─────────────────────────────────────


def test_neutral_frame_is_zero_throttle_zero_steering():
    assert encode_manual_control(0.0, 0.0) == NEUTRAL_FRAME
    assert NEUTRAL_FRAME == ManualControlFrame(x=0, y=0, z=500, r=0, buttons=0)


def test_golden_frame_full_forward_full_right():
    # §7.1 confirmed mapping: steering → r (yaw), throttle → z, y always 0.
    frame = encode_manual_control(throttle=1.0, steering=1.0)
    assert frame == ManualControlFrame(x=0, y=0, z=1000, r=1000, buttons=0)


def test_golden_frame_full_reverse_full_left():
    frame = encode_manual_control(throttle=-1.0, steering=-1.0)
    assert frame == ManualControlFrame(x=0, y=0, z=0, r=-1000, buttons=0)


def test_golden_frame_half_forward_no_steering():
    frame = encode_manual_control(throttle=0.5, steering=0.0)
    assert frame == ManualControlFrame(x=0, y=0, z=750, r=0, buttons=0)


def test_encode_clamps_out_of_range_inputs():
    frame = encode_manual_control(throttle=5.0, steering=-5.0)
    assert frame == ManualControlFrame(x=0, y=0, z=1000, r=-1000, buttons=0)


def test_encode_rejects_non_finite_inputs():
    with pytest.raises(ValueError):
        encode_manual_control(float("nan"), 0.0)
    with pytest.raises(ValueError):
        encode_manual_control(0.0, float("inf"))


# ── Gateway sender / watchdog ────────────────────────────────────────────────


def _gateway(**kwargs):
    transport = FakeTransport()
    gw = ManualControlGateway(transport, rate_hz=100.0, stale_timeout_s=0.08, **kwargs)
    return gw, transport


def test_gateway_streams_neutral_by_default():
    gw, transport = _gateway()
    gw.start()
    try:
        time.sleep(0.05)
        assert transport.sent, "sender thread never published"
        assert transport.sent[-1] == NEUTRAL_FRAME
    finally:
        gw.shutdown()


def test_gateway_streams_accepted_command_then_goes_neutral_when_stale():
    gw, transport = _gateway()
    gw.start()
    try:
        gw.activate_neutral()
        frame = gw.accept_command(throttle=0.2, steering=-0.1)
        time.sleep(0.03)
        assert transport.sent[-1] == frame
        assert frame != NEUTRAL_FRAME

        time.sleep(0.15)  # > stale_timeout_s
        assert transport.sent[-1] == NEUTRAL_FRAME
    finally:
        gw.shutdown()


def test_deactivate_neutral_flushes_neutral_frames():
    gw, transport = _gateway()
    gw.start()
    try:
        gw.accept_command(throttle=0.5, steering=0.5)
        gw.deactivate_neutral()
        assert transport.sent[-1] == NEUTRAL_FRAME
        assert gw.is_neutral is True
    finally:
        gw.shutdown()


def test_snapshot_reports_transport_health_and_frame():
    gw, transport = _gateway()
    gw.start()
    try:
        gw.accept_command(throttle=0.3, steering=0.0)
        time.sleep(0.03)
        snap = gw.snapshot()
        assert snap["transport"] == "fake"
        assert snap["transport_healthy"] is True
        assert snap["gateway_active"] is True
        assert snap["gateway_last_frame"]["z"] > 500
    finally:
        gw.shutdown()


def test_unhealthy_transport_reflected_in_health_reason():
    transport = FakeTransport(healthy=False)
    gw = ManualControlGateway(transport, rate_hz=100.0, stale_timeout_s=0.08)
    assert gw.is_healthy() is False
    assert "fake transport down" in gw.health_reason()


def test_shutdown_is_idempotent_and_calls_transport_shutdown():
    gw, transport = _gateway()
    gw.start()
    gw.shutdown()
    gw.shutdown()  # must not raise
    assert transport.shutdown_called is True
