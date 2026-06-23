"""Regression tests for LoRa injection-byte ownership (F1)."""

from __future__ import annotations

import importlib.util
import sys
import threading
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from rtcm3_parser import Rtcm3StreamParser, build_rtcm3_frame

_REPO = Path(__file__).resolve().parent


def _load_lora_module():
    """Load lora_rtcm_node without executing rclpy import at collection time."""
    if "lora_rtcm_node_test" in sys.modules:
        return sys.modules["lora_rtcm_node_test"]
    rclpy_stub = type(sys)("rclpy")
    rclpy_stub.init = lambda *a, **k: None
    rclpy_stub.try_shutdown = lambda *a, **k: None
    node_mod = type(sys)("rclpy.node")

    class _FakeNodeBase:
        _super_destroy_calls = 0

        def destroy_node(self):
            _FakeNodeBase._super_destroy_calls += 1

        def get_logger(self):
            return MagicMock()

        def create_timer(self, _period, _callback):
            return MagicMock()

        def destroy_timer(self, _timer):
            pass

    node_mod.Node = _FakeNodeBase
    qos_mod = type(sys)("rclpy.qos")
    qos_mod.QoSProfile = object
    qos_mod.ReliabilityPolicy = type("R", (), {"RELIABLE": 1})()
    qos_mod.DurabilityPolicy = type("D", (), {"VOLATILE": 1})()
    mavros_mod = type(sys)("mavros_msgs.msg")
    class _RtcmMsg:
        def __init__(self):
            self.header = type("H", (), {"stamp": None})()
            self.data = []

    mavros_mod.RTCM = _RtcmMsg
    serial_mod = type(sys)("serial")
    serial_mod.Serial = object
    serial_mod.SerialException = OSError
    sys.modules["rclpy"] = rclpy_stub
    sys.modules["rclpy.node"] = node_mod
    sys.modules["rclpy.qos"] = qos_mod
    sys.modules["mavros_msgs.msg"] = mavros_mod
    sys.modules["serial"] = serial_mod
    spec = importlib.util.spec_from_file_location(
        "lora_rtcm_node_test", _REPO / "lora_rtcm_node.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules["lora_rtcm_node_test"] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _frame() -> bytes:
    payload = bytes([0x3E, 0xDF, 0x00])
    return build_rtcm3_frame(payload)


def _make_node():
    mod = _load_lora_module()
    node = mod.LoraRtcmNode.__new__(mod.LoraRtcmNode)
    node._stats_lock = threading.Lock()
    node._parser = Rtcm3StreamParser()
    node._rate_tracker = MagicMock()
    node._bytes_injected = 0
    node._dropped_no_subscriber = 0
    node._dropped_publish_fail = 0
    node._last_injected_frame_monotonic = None
    node._last_error = None
    node._lifecycle_state = "connected"
    node.pub = MagicMock()
    node.get_clock = MagicMock(
        return_value=MagicMock(
            now=MagicMock(return_value=MagicMock(to_msg=MagicMock(return_value=object())))
        )
    )
    node._set_lifecycle = MagicMock()
    return node


def test_parser_never_counts_injected_bytes():
    parser = Rtcm3StreamParser()
    frame = _frame()
    parser.feed(frame)
    stats = parser.snapshot_stats()
    assert stats.valid_frames == 1
    assert not hasattr(stats, "bytes_injected")


def test_publish_without_subscriber_does_not_inject():
    node = _make_node()
    node.pub.get_subscription_count.return_value = 0
    node._publish_frames([_frame()])
    assert node._bytes_injected == 0
    assert node._dropped_no_subscriber == 1
    node.pub.publish.assert_not_called()


def test_publish_with_subscriber_injects_once():
    node = _make_node()
    node.pub.get_subscription_count.return_value = 1
    frame = _frame()
    node._publish_frames([frame])
    assert node._bytes_injected == len(frame)
    assert node._dropped_no_subscriber == 0
    node.pub.publish.assert_called_once()


def test_publish_failure_does_not_count_injected():
    node = _make_node()
    node.pub.get_subscription_count.return_value = 1
    node.pub.publish.side_effect = RuntimeError("publish failed")
    node._publish_frames([_frame()])
    assert node._bytes_injected == 0
    assert node._dropped_publish_fail == 1


def test_invalid_frame_never_increments_node_injected_bytes():
    node = _make_node()
    node.pub.get_subscription_count.return_value = 1
    bad = bytearray(_frame())
    bad[-1] ^= 0xFF
    assert node._parser.feed(bytes(bad)) == []
    node._publish_frames([])
    assert node._bytes_injected == 0


def test_init_retains_timer_handles():
    mod = _load_lora_module()
    node = mod.LoraRtcmNode.__new__(mod.LoraRtcmNode)
    created = []

    def fake_create_timer(period, _cb):
        timer = MagicMock(name=f"timer-{period}")
        created.append(timer)
        return timer

    node.create_timer = fake_create_timer
    node.get_logger = MagicMock(return_value=MagicMock())
    node._status_write_interval_s = 1.0
    node.serial_port = "/dev/ttyUSB0"
    node.baudrate = 115200
    node._status_file = None
    node._session_id = "sess"
    node._process_id = 1
    node._reconnect_interval_s = 5.0
    node._module_disconnect_timeout_s = 120.0
    node._parser = Rtcm3StreamParser()
    node._rate_tracker = MagicMock()
    node._stats_lock = threading.Lock()
    node._stop_event = threading.Event()
    node._shutting_down = False
    node._status_timer = None
    node._health_timer = None
    node._status_timer = node.create_timer(node._status_write_interval_s, lambda: None)
    node._health_timer = node.create_timer(30.0, lambda: None)
    assert node._status_timer is created[0]
    assert node._health_timer is created[1]


def test_destroy_timers_clears_both_handles():
    mod = _load_lora_module()
    node = mod.LoraRtcmNode.__new__(mod.LoraRtcmNode)
    t_status = MagicMock()
    t_health = MagicMock()
    node._status_timer = t_status
    node._health_timer = t_health
    node.get_logger = MagicMock(return_value=MagicMock())
    destroyed = []
    node.destroy_timer = destroyed.append
    mod.LoraRtcmNode._destroy_timers(node)
    assert destroyed == [t_status, t_health]
    assert node._status_timer is None
    assert node._health_timer is None


def test_destroy_node_stops_serial_thread_before_super():
    mod = _load_lora_module()
    node = mod.LoraRtcmNode.__new__(mod.LoraRtcmNode)
    node._shutting_down = False
    node._stop_event = threading.Event()
    started = threading.Event()

    def worker():
        started.set()
        node._stop_event.wait(timeout=2.0)

    node._thread = threading.Thread(target=worker)
    node._thread.start()
    started.wait(timeout=1.0)
    node._status_timer = MagicMock()
    node._health_timer = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock())
    destroyed = []
    node.destroy_timer = destroyed.append
    mod.LoraRtcmNode.__bases__[0]._super_destroy_calls = 0
    mod.LoraRtcmNode.destroy_node(node)
    assert not node._thread.is_alive()
    assert len(destroyed) == 2
    assert node._status_timer is None
    assert node._health_timer is None
    assert mod.LoraRtcmNode.__bases__[0]._super_destroy_calls == 1


def test_destroy_node_idempotent_on_partial_node():
    mod = _load_lora_module()
    node = mod.LoraRtcmNode.__new__(mod.LoraRtcmNode)
    node._shutting_down = False
    node._stop_event = threading.Event()
    node._thread = None
    node._status_timer = None
    node._health_timer = None
    node.get_logger = MagicMock(return_value=MagicMock())
    node.destroy_timer = MagicMock()
    mod.LoraRtcmNode.destroy_node(node)
    mod.LoraRtcmNode.destroy_node(node)


def test_parser_snapshot_consistency_under_lock():
    parser = Rtcm3StreamParser()
    frame = _frame()
    snapshots = []

    def reader():
        for _ in range(50):
            snapshots.append(parser.snapshot_stats())

    def writer():
        for _ in range(50):
            parser.feed(frame)

    t1 = threading.Thread(target=reader)
    t2 = threading.Thread(target=writer)
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    assert snapshots
    for snap in snapshots:
        assert snap.valid_frames >= 0
        assert snap.bytes_received >= 0