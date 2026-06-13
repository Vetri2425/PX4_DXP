#!/usr/bin/env python3
"""LoRa -> MAVROS RTK injection for PX4.

Reads RTCM3 frames from a local serial LoRa module, validates CRC-24Q,
and publishes them to /mavros/gps_rtk/send_rtcm.
Auto-reconnects with exponential backoff on serial errors.
"""

import argparse
import json
import os
import signal
import threading
import time
from pathlib import Path

import rclpy
import serial
from mavros_msgs.msg import RTCM
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# RTCM3 frame constants
_RTCM3_PREAMBLE = 0xD3
_RTCM3_HEADER_LEN = 3
_RTCM3_CRC_LEN = 3

# CRC-24Q lookup table
_CRC24Q_TABLE = [0] * 256
for _i in range(256):
    _crc = _i << 16
    for _ in range(8):
        _crc = ((_crc << 1) ^ 0x1864CFB) if (_crc & 0x800000) else (_crc << 1)
        _crc &= 0xFFFFFF
    _CRC24Q_TABLE[_i] = _crc


def _rtcm3_crc(data: bytes, length: int) -> int:
    crc = 0
    for i in range(length):
        crc = ((crc << 8) ^ _CRC24Q_TABLE[((crc >> 16) & 0xFF) ^ data[i]]) & 0xFFFFFF
    return crc


class LoraRtcmNode(Node):
    """ROS2 node that streams RTCM3 corrections from serial LoRa to MAVROS.

    Reconnects automatically with exponential backoff on serial errors.
    """

    def __init__(self, serial_port: str, baudrate: int, status_file: str | None = None):
        super().__init__("lora_rtcm_node")

        self.serial_port = serial_port
        self.baudrate = baudrate
        self._status_file = Path(status_file) if status_file else None

        rtcm_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(RTCM, "/mavros/gps_rtk/send_rtcm", rtcm_qos)

        self._stop_event = threading.Event()
        self._stats_lock = threading.Lock()
        self._connected = False
        self._frames = 0
        self._bytes = 0
        self._last_frame_wall_time = None
        self._last_error = None
        self._reconnect_count = 0
        self._window_frames = 0
        self._window_bytes = 0

        self.create_timer(1.0, lambda: self._write_status("connected" if self._connected else "connecting"))
        self.create_timer(30.0, self._check_health)

        self.get_logger().info(f"Starting LoRa RTK on {serial_port} @ {baudrate} baud")
        self._write_status("starting")

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def request_stop(self):
        self._stop_event.set()

    def _write_status(self, state: str):
        if self._status_file is None:
            return
        with self._stats_lock:
            payload = {
                "mode": "lora",
                "state": state,
                "connected": self._connected,
                "frames": self._frames,
                "bytes": self._bytes,
                "last_frame_time": self._last_frame_wall_time,
                "last_error": self._last_error,
                "updated_at": time.time(),
            }
        tmp_path = self._status_file.with_suffix(".tmp")
        try:
            tmp_path.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
            os.replace(tmp_path, self._status_file)
        except Exception as exc:
            self.get_logger().debug(f"Failed to write status file: {exc}")

    def _check_health(self):
        with self._stats_lock:
            frames = self._window_frames
            bytess = self._window_bytes
            self._window_frames = 0
            self._window_bytes = 0
            last = self._last_frame_wall_time

        age = time.time() - last if last is not None else None
        if age is None or age > 30.0:
            self.get_logger().warn(
                f"No LoRa RTCM data for {age:.0f}s" if age is not None else "No LoRa RTCM data yet"
                f" (reconnects: {self._reconnect_count})"
            )
        else:
            self.get_logger().info(
                f"LoRa RTCM: {frames} frames, {bytess} bytes in last 30s"
                f" (reconnects: {self._reconnect_count})"
            )

    @staticmethod
    def _parse_rtcm_frames(buf: bytes, logger=None):
        """Extract complete, CRC-valid RTCM3 frames from a byte buffer."""
        frames = []
        i = 0
        while i < len(buf):
            if buf[i] != _RTCM3_PREAMBLE:
                i += 1
                continue

            if i + _RTCM3_HEADER_LEN > len(buf):
                break

            length_field = (buf[i + 1] << 8) | buf[i + 2]
            msg_len = length_field & 0x03FF
            total_frame = _RTCM3_HEADER_LEN + msg_len + _RTCM3_CRC_LEN

            if i + total_frame > len(buf):
                break

            frame = buf[i : i + total_frame]
            payload_len = _RTCM3_HEADER_LEN + msg_len
            expected_crc = (
                (frame[payload_len] << 16)
                | (frame[payload_len + 1] << 8)
                | frame[payload_len + 2]
            )
            computed_crc = _rtcm3_crc(frame, payload_len)
            if computed_crc != expected_crc:
                if logger:
                    logger.warn(
                        f"RTCM3 CRC mismatch at offset {i}: "
                        f"expected 0x{expected_crc:06X}, got 0x{computed_crc:06X}"
                    )
                i += 1
                continue

            frames.append(frame)
            i += total_frame

        return frames, buf[i:]

    def _run(self):
        attempt = 0

        while not self._stop_event.is_set():
            buf = b""
            try:
                self.get_logger().info(
                    f"Opening {self.serial_port} @ {self.baudrate} baud"
                    + (f" (reconnect #{self._reconnect_count})" if self._reconnect_count else "")
                )
                with serial.Serial(self.serial_port, self.baudrate, timeout=1.0) as ser:
                    with self._stats_lock:
                        self._connected = True
                        self._last_error = None
                    self._write_status("connected")
                    attempt = 0  # reset backoff on successful open

                    while not self._stop_event.is_set():
                        chunk = ser.read(1024)
                        if not chunk:
                            continue

                        buf += chunk
                        frames, buf = self._parse_rtcm_frames(buf, self.get_logger())
                        for frame in frames:
                            msg = RTCM()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.data = list(frame)
                            self.pub.publish(msg)
                            with self._stats_lock:
                                self._frames += 1
                                self._bytes += len(frame)
                                self._window_frames += 1
                                self._window_bytes += len(frame)
                                self._last_frame_wall_time = time.time()
                                self._last_error = None
                            self._write_status("streaming")

            except serial.SerialException as exc:
                with self._stats_lock:
                    self._connected = False
                    self._last_error = str(exc)
                    self._reconnect_count += 1
                self.get_logger().error(
                    f"LoRa serial error: {exc} — reconnect #{self._reconnect_count}"
                )
                self._write_status("error")
            except Exception as exc:
                with self._stats_lock:
                    self._connected = False
                    self._last_error = str(exc)
                    self._reconnect_count += 1
                self.get_logger().error(
                    f"LoRa unexpected error: {exc} — reconnect #{self._reconnect_count}"
                )
                self._write_status("error")
            else:
                # Clean exit from inner loop — stop was requested
                with self._stats_lock:
                    self._connected = False
                break

            if self._stop_event.is_set():
                break

            # Exponential backoff: min(5 * 2^attempt, 60) seconds
            backoff = min(5 * (2 ** attempt), 60)
            attempt += 1
            self.get_logger().info(f"Reconnecting LoRa in {backoff}s...")
            self._write_status("reconnecting")
            self._stop_event.wait(backoff)

    def destroy_node(self):
        self.get_logger().info("Shutting down LoRa RTK node...")
        self._write_status("stopping")
        self.request_stop()
        if self._thread.is_alive():
            self._thread.join(timeout=2)
        super().destroy_node()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="LoRa RTCM3 injector")
    parser.add_argument("--serial-port", required=True, help="Serial device, e.g. /dev/ttyUSB0")
    parser.add_argument("--baudrate", required=True, type=int, help="Serial baudrate, e.g. 115200")
    parser.add_argument(
        "--status-file",
        help="Optional JSON status file for the FastAPI RTK manager",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    rclpy.init()
    node = LoraRtcmNode(args.serial_port, args.baudrate, status_file=args.status_file)

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
