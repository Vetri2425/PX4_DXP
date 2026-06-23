#!/usr/bin/env python3
"""LoRa -> MAVROS RTK injection for PX4.

Reads RTCM3 frames from a local serial LoRa module, validates CRC-24Q,
and publishes them to /mavros/gps_rtk/send_rtcm.  Serial reconnect and
MAVROS outage are recoverable without a new API start request.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import threading
import time
import uuid
from pathlib import Path

import rclpy
import serial
from mavros_msgs.msg import RTCM
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from rtcm3_parser import (
    Rtcm3StreamParser,
    build_rtcm3_frame,
    parse_allowed_message_types,
    rtcm3_crc24q,
)
from rtk_transport import TransportRateTracker

# Re-export for tests that import CRC helpers from this module.
__all__ = ["LoraRtcmNode", "build_rtcm3_frame", "rtcm3_crc24q"]


class LoraRtcmNode(Node):
    """ROS2 node that streams validated RTCM3 corrections from serial LoRa."""

    def __init__(
        self,
        serial_port: str,
        baudrate: int,
        *,
        status_file: str | None = None,
        session_id: str | None = None,
        reconnect_interval_s: float = 5.0,
        module_disconnect_timeout_s: float = 120.0,
        max_frame_size: int = 1029,
        max_bytes_per_sec: float = 65536.0,
        max_frames_per_sec: float = 50.0,
        allowed_message_types: str | None = None,
        status_write_interval_s: float = 1.0,
    ):
        super().__init__("lora_rtcm_node")

        self.serial_port = serial_port
        self.baudrate = baudrate
        self._status_file = Path(status_file) if status_file else None
        self._session_id = session_id or uuid.uuid4().hex
        self._process_id = os.getpid()
        self._reconnect_interval_s = max(0.5, reconnect_interval_s)
        self._module_disconnect_timeout_s = max(5.0, module_disconnect_timeout_s)
        self._status_write_interval_s = max(0.25, status_write_interval_s)

        self._parser = Rtcm3StreamParser(
            max_frame_size=max_frame_size,
            allowed_message_types=parse_allowed_message_types(allowed_message_types),
            max_bytes_per_sec=max_bytes_per_sec,
            max_frames_per_sec=max_frames_per_sec,
        )
        self._rate_tracker = TransportRateTracker()

        rtcm_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(RTCM, "/mavros/gps_rtk/send_rtcm", rtcm_qos)

        self._stop_event = threading.Event()
        self._stats_lock = threading.Lock()
        self._serial_open = False
        self._connected = False
        self._lifecycle_state = "starting"
        self._last_error: str | None = None
        self._last_injected_frame_monotonic: float | None = None
        self._serial_open_since_monotonic: float | None = None
        self._serial_absent_since: float | None = None
        self._bytes_injected = 0
        self._dropped_no_subscriber = 0
        self._dropped_publish_fail = 0
        self._status_dirty = True
        self._last_status_write = 0.0
        self._shutting_down = False
        self._status_timer = None
        self._health_timer = None

        self._status_timer = self.create_timer(
            self._status_write_interval_s,
            self._maybe_write_status,
        )
        self._health_timer = self.create_timer(30.0, self._check_health)

        self.get_logger().info(
            f"Starting LoRa RTK session={self._session_id} on {serial_port} @ {baudrate}"
        )
        self._write_status("starting")

        self._thread = threading.Thread(target=self._run, daemon=True, name="lora-serial")
        self._thread.start()

    @property
    def session_id(self) -> str:
        return self._session_id

    def request_stop(self) -> None:
        self._stop_event.set()

    def _injection_topic_ready(self) -> bool:
        try:
            return self.pub.get_subscription_count() > 0
        except Exception:
            return False

    def _set_lifecycle(self, state: str, *, last_error: str | None = None) -> None:
        with self._stats_lock:
            self._lifecycle_state = state
            if last_error is not None:
                self._last_error = last_error
            self._status_dirty = True

    def _build_status_payload(self, state: str) -> dict:
        with self._stats_lock:
            stats = self._parser.snapshot_stats()
            now = time.monotonic()
            self._rate_tracker.sample(now, stats.valid_frames, stats.bytes_received)
            dropped_total = (
                stats.dropped_frames + self._dropped_no_subscriber + self._dropped_publish_fail
            )
            return {
                "session_id": self._session_id,
                "process_id": self._process_id,
                "mode": "lora",
                "lifecycle_state": state,
                "state": state,
                "serial_port": self.serial_port,
                "baudrate": self.baudrate,
                "serial_open": self._serial_open,
                "serial_open_since_monotonic": self._serial_open_since_monotonic,
                "connected": self._connected,
                "process_alive": True,
                "valid_frames": stats.valid_frames,
                "invalid_frames": stats.invalid_frames,
                "crc_errors": stats.crc_errors,
                "dropped_frames": dropped_total,
                "dropped_no_subscriber": self._dropped_no_subscriber,
                "bytes_received": stats.bytes_received,
                "bytes_injected": self._bytes_injected,
                "last_valid_frame_time": stats.last_valid_frame_monotonic,
                "last_injected_frame_time": self._last_injected_frame_monotonic,
                "valid_frame_rate_hz": self._rate_tracker.valid_frame_rate_hz,
                "bytes_per_sec": self._rate_tracker.bytes_per_sec,
                "injection_topic_ready": self._injection_topic_ready(),
                "last_error": self._last_error,
                "updated_at": time.time(),
                "updated_at_monotonic": now,
            }

    def _maybe_write_status(self) -> None:
        if self._shutting_down:
            return
        if self._status_file is None:
            return
        if not self._status_dirty and (time.monotonic() - self._last_status_write) < self._status_write_interval_s:
            return
        self._write_status(self._lifecycle_state)

    def _write_status(self, state: str) -> None:
        if self._status_file is None:
            return
        payload = self._build_status_payload(state)
        with self._stats_lock:
            self._lifecycle_state = state
            self._status_dirty = False
            self._last_status_write = time.monotonic()
        tmp_path = self._status_file.with_suffix(".tmp")
        try:
            tmp_path.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
            os.replace(tmp_path, self._status_file)
        except Exception as exc:
            self.get_logger().debug(f"Failed to write status file: {exc}")

    def _check_health(self) -> None:
        if self._shutting_down:
            return
        stats = self._parser.snapshot_stats()
        last = stats.last_valid_frame_monotonic
        age = time.monotonic() - last if last is not None else None
        if age is None or age > 30.0:
            self.get_logger().warn(
                f"No LoRa RTCM data for {age:.0f}s" if age is not None else "No LoRa RTCM data yet"
            )
        else:
            self.get_logger().info(
                f"LoRa RTCM: valid={stats.valid_frames} injected={self._bytes_injected}B"
            )

    def _publish_frames(self, frames: list[bytes]) -> None:
        topic_ready = self._injection_topic_ready()
        for frame in frames:
            if not topic_ready:
                with self._stats_lock:
                    self._dropped_no_subscriber += 1
                    self._status_dirty = True
                continue
            msg = RTCM()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.data = list(frame)
            try:
                self.pub.publish(msg)
            except Exception as exc:
                with self._stats_lock:
                    self._dropped_publish_fail += 1
                    self._last_error = str(exc)
                    self._status_dirty = True
                continue
            with self._stats_lock:
                # bytes_injected counts validated RTCM frame bytes handed to the
                # ROS publisher when a subscriber was visible and publish()
                # returned without raising. It cannot prove downstream
                # MAVROS/PX4 receipt.
                self._bytes_injected += len(frame)
                self._last_injected_frame_monotonic = time.monotonic()
                self._last_error = None
                self._status_dirty = True
            self._set_lifecycle("streaming")

    def _run(self) -> None:
        while not self._stop_event.is_set():
            ser = None
            try:
                self.get_logger().info(
                    f"Opening {self.serial_port} @ {self.baudrate} baud"
                )
                ser = serial.Serial(self.serial_port, self.baudrate, timeout=1.0)
                with self._stats_lock:
                    self._serial_open = True
                    self._connected = True
                    self._serial_open_since_monotonic = time.monotonic()
                    self._serial_absent_since = None
                    self._last_error = None
                    self._status_dirty = True
                self._set_lifecycle("connected")

                while not self._stop_event.is_set():
                    try:
                        chunk = ser.read(1024)
                    except serial.SerialException as exc:
                        raise exc
                    if not chunk:
                        continue

                    frames = self._parser.feed(chunk)
                    if frames:
                        self._publish_frames(frames)
                    elif self._parser.snapshot_stats().valid_frames == 0:
                        self._set_lifecycle("connected")

            except serial.SerialException as exc:
                with self._stats_lock:
                    self._serial_open = False
                    self._connected = False
                    self._serial_open_since_monotonic = None
                    self._last_error = str(exc)
                    self._status_dirty = True
                    if self._serial_absent_since is None:
                        self._serial_absent_since = time.monotonic()
                    absent_for = time.monotonic() - self._serial_absent_since
                if absent_for >= self._module_disconnect_timeout_s:
                    self._set_lifecycle("module_disconnected", last_error=str(exc))
                else:
                    self._set_lifecycle("reconnecting", last_error=str(exc))
                self.get_logger().error(f"LoRa serial error: {exc}")
            except Exception as exc:
                with self._stats_lock:
                    self._serial_open = False
                    self._connected = False
                    self._serial_open_since_monotonic = None
                    self._last_error = str(exc)
                    self._status_dirty = True
                self._set_lifecycle("reconnecting", last_error=str(exc))
                self.get_logger().error(f"LoRa unexpected error: {exc}")
            finally:
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                with self._stats_lock:
                    self._serial_open = False
                    self._connected = False
                    self._serial_open_since_monotonic = None
                    self._status_dirty = True

            if self._stop_event.is_set():
                break

            self._set_lifecycle("reconnecting")
            self._stop_event.wait(self._reconnect_interval_s)

    def _destroy_timers(self) -> None:
        for label, attr in (("status", "_status_timer"), ("health", "_health_timer")):
            timer = getattr(self, attr, None)
            if timer is None:
                continue
            try:
                self.destroy_timer(timer)
            except Exception as exc:
                self.get_logger().debug(f"Failed to destroy {label} timer: {exc}")
            setattr(self, attr, None)

    def destroy_node(self) -> None:
        self._shutting_down = True
        self.get_logger().info("Shutting down LoRa RTK node...")
        self.request_stop()
        thread = getattr(self, "_thread", None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=5)
        self._destroy_timers()
        super().destroy_node()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="LoRa RTCM3 injector")
    parser.add_argument("--serial-port", required=True, help="Serial device, e.g. /dev/ttyUSB0")
    parser.add_argument("--baudrate", required=True, type=int, help="Serial baudrate, e.g. 115200")
    parser.add_argument("--status-file", help="JSON status file for the FastAPI RTK manager")
    parser.add_argument("--session-id", help="Session identity echoed in status reports")
    parser.add_argument("--reconnect-interval-s", type=float, default=5.0)
    parser.add_argument("--module-disconnect-timeout-s", type=float, default=120.0)
    parser.add_argument("--max-frame-size", type=int, default=1029)
    parser.add_argument("--max-bytes-per-sec", type=float, default=65536.0)
    parser.add_argument("--max-frames-per-sec", type=float, default=50.0)
    parser.add_argument(
        "--allowed-message-types",
        default="",
        help="Comma-separated RTCM message types to allow; empty allows all",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    rclpy.init()
    node = LoraRtcmNode(
        args.serial_port,
        args.baudrate,
        status_file=args.status_file,
        session_id=args.session_id,
        reconnect_interval_s=args.reconnect_interval_s,
        module_disconnect_timeout_s=args.module_disconnect_timeout_s,
        max_frame_size=args.max_frame_size,
        max_bytes_per_sec=args.max_bytes_per_sec,
        max_frames_per_sec=args.max_frames_per_sec,
        allowed_message_types=args.allowed_message_types or None,
    )

    def _handle_signal(signum, _frame):
        node.get_logger().info(f"Received signal {signum}; stopping LoRa node")
        node.request_stop()
        rclpy.try_shutdown()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()