"""Unit tests for hardened NTRIP RTCM injection behaviour."""

from __future__ import annotations

import importlib.util
import socket
import sys
import threading
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

import json
import time

from rtcm3_parser import Rtcm3StreamParser, build_rtcm3_frame
from rtk_transport import TransportRateTracker, configure_tcp_keepalive, redact_rtk_secrets

_REPO = Path(__file__).resolve().parent


def _load_ntrip_module():
    if "ntrip_rtcm_node_test" in sys.modules:
        return sys.modules["ntrip_rtcm_node_test"]
    rclpy_stub = type(sys)("rclpy")
    rclpy_stub.init = lambda *a, **k: None
    rclpy_stub.try_shutdown = lambda *a, **k: None
    rclpy_stub.ok = lambda: True
    rclpy_stub.spin_once = lambda *a, **k: None
    node_mod = type(sys)("rclpy.node")

    class _FakeNodeBase:
        def destroy_node(self):
            pass

        def get_logger(self):
            return MagicMock()

        def create_timer(self, _period, _callback):
            return MagicMock()

        def destroy_timer(self, _timer):
            pass

        def create_publisher(self, *_a, **_k):
            return MagicMock()

        def create_subscription(self, *_a, **_k):
            return None

    node_mod.Node = _FakeNodeBase
    qos_mod = type(sys)("rclpy.qos")
    qos_mod.QoSProfile = object
    qos_mod.ReliabilityPolicy = type("R", (), {"RELIABLE": 1, "BEST_EFFORT": 2})()
    qos_mod.DurabilityPolicy = type("D", (), {"VOLATILE": 1})()
    mavros_mod = type(sys)("mavros_msgs.msg")

    class _RtcmMsg:
        def __init__(self):
            self.header = type("H", (), {"stamp": None})()
            self.data = []

    mavros_mod.RTCM = _RtcmMsg
    sensor_mod = type(sys)("sensor_msgs.msg")
    sensor_mod.NavSatFix = object
    sensor_mod.NavSatStatus = type("S", (), {"STATUS_GBAS_FIX": 1, "STATUS_SBAS_FIX": 2})()
    sys.modules["rclpy"] = rclpy_stub
    sys.modules["rclpy.node"] = node_mod
    sys.modules["rclpy.qos"] = qos_mod
    sys.modules["mavros_msgs.msg"] = mavros_mod
    sys.modules["sensor_msgs.msg"] = sensor_mod
    spec = importlib.util.spec_from_file_location(
        "ntrip_rtcm_node_test", _REPO / "ntrip_rtcm_node.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules["ntrip_rtcm_node_test"] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _frame() -> bytes:
    payload = bytes([0x3E, 0xDF, 0x00])
    return build_rtcm3_frame(payload)


def _make_node():
    mod = _load_ntrip_module()
    node = mod.NtripNode.__new__(mod.NtripNode)
    node._stats_lock = threading.Lock()
    node._parser = Rtcm3StreamParser()
    node._rate_tracker = MagicMock()
    node._inject_rate_tracker = MagicMock()
    node._bytes_injected = 0
    node._frames_published = 0
    node._valid_rtcm_bytes = 0
    node._publish_error_count = 0
    node._publish_errors_window = 0
    node._publish_errors_window_start = 0.0
    node._publish_error_unhealthy_threshold = 3
    node._injection_healthy = True
    node._last_error = None
    node._lifecycle_state = "connected"
    node._no_rtcm_warn_s = 15.0
    node._no_rtcm_reconnect_s = 45.0
    node._handshake_complete_monotonic = None
    node._connected = True
    node._transport_reason = None
    node._max_frame_gap_s = 0.0
    node._last_frame_gap_anchor = None
    node._status_dirty = True
    node._status_file = None
    node._session_id = "sess"
    node._process_id = 1
    node.pub = MagicMock()
    node.pub.get_subscription_count.return_value = 1
    node.get_clock = MagicMock(
        return_value=MagicMock(
            now=MagicMock(return_value=MagicMock(to_msg=MagicMock(return_value=object())))
        )
    )
    node._set_lifecycle = MagicMock()
    return node


def test_valid_frame_publish_increments_injection():
    node = _make_node()
    frame = _frame()
    node._publish_frames([frame])
    assert node._frames_published == 1
    assert node._bytes_injected == len(frame)
    node.pub.publish.assert_called_once()


def test_publish_exception_does_not_raise():
    node = _make_node()
    node._publish_error_unhealthy_threshold = 1
    node.pub.publish.side_effect = RuntimeError("dds down")
    node._publish_frames([_frame()])
    assert node._publish_error_count == 1
    assert node._injection_healthy is False


def test_publish_recovery_clears_unhealthy():
    node = _make_node()
    node._injection_healthy = False
    node._publish_frames([_frame()])
    assert node._injection_healthy is True


def test_redact_basic_auth():
    text = "Authorization: Basic c2VjcmV0"
    assert "***" in redact_rtk_secrets(text)


def test_tcp_keepalive_setsockopt():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        configure_tcp_keepalive(sock)
        assert sock.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE) != 0
    finally:
        sock.close()


def test_valid_rtcm_age_uses_handshake_before_first_frame():
    node = _make_node()
    node._handshake_complete_monotonic = 100.0
    stats = node._parser.snapshot_stats()
    age = node._valid_rtcm_age_s(130.0, stats)
    assert age == pytest.approx(30.0)


def _prepare_node_for_destroy(node, status_path):
    """Wire up the minimal attributes destroy_node() touches."""
    node._rate_tracker = TransportRateTracker()
    node._inject_rate_tracker = TransportRateTracker()
    node._status_file = status_path
    node._status_write_interval_s = 1.0
    node._last_status_write = 0.0
    node._reconnect_count = 0
    node._max_frame_gap_s = 0.0
    node._shutting_down = False
    node._stop_event = threading.Event()
    node._sock_lock = threading.Lock()
    node._active_sock = None
    node._gga_lock = threading.Lock()
    node._gga_sock = None
    node._status_timer = None
    node._health_timer = None
    node._thread = MagicMock()
    node._thread.is_alive.return_value = False


def test_destroy_node_preserves_terminal_auth_status(tmp_path):
    """destroy_node() must not clobber a terminal auth_failed status with
    'stopping' — the manager/operator must still see the real cause."""
    node = _make_node()
    status_path = tmp_path / "ntrip_status.json"
    _prepare_node_for_destroy(node, status_path)
    node._write_status("auth_failed")
    assert json.loads(status_path.read_text())["state"] == "auth_failed"

    node.destroy_node()

    payload = json.loads(status_path.read_text())
    assert payload["state"] == "auth_failed"
    assert payload["lifecycle_state"] == "auth_failed"


def test_destroy_node_reports_stopping_for_non_terminal(tmp_path):
    """A normally-running node still reports 'stopping' on shutdown."""
    node = _make_node()
    status_path = tmp_path / "ntrip_status.json"
    _prepare_node_for_destroy(node, status_path)
    node._write_status("connected")

    node.destroy_node()

    assert json.loads(status_path.read_text())["state"] == "stopping"


def _make_health_node(age_s):
    """Node fake configured for _check_stream_health with a given RTCM age."""
    node = _make_node()
    node._shutting_down = False
    node._connected = True
    node._no_rtcm_warn_s = 15.0
    node._no_rtcm_reconnect_s = 45.0
    node._no_rtcm_warned = False
    node._handshake_complete_monotonic = time.monotonic() - age_s
    node._force_reconnect = threading.Event()
    node._close_active_socket = MagicMock()
    return node


def test_stream_watchdog_forces_reconnect_when_stale():
    """No valid RTCM past the reconnect threshold must flag _force_reconnect and
    close the socket (so the read thread treats it as an intentional reconnect,
    not an [Errno 9] transport error)."""
    node = _make_health_node(age_s=100.0)  # > no_rtcm_reconnect_s
    node._check_stream_health()
    assert node._force_reconnect.is_set()
    node._close_active_socket.assert_called_once()


def test_stream_watchdog_idle_when_fresh():
    """A healthy stream must not force a reconnect."""
    node = _make_health_node(age_s=1.0)  # < no_rtcm_warn_s
    node._check_stream_health()
    assert not node._force_reconnect.is_set()
    node._close_active_socket.assert_not_called()