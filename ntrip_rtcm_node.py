#!/usr/bin/env python3
"""NTRIP client -> MAVROS RTK injection for PX4.

Connects to an NTRIP caster, parses RTCM3 frames from the stream,
and publishes them to /mavros/gps_rtk/send_rtcm for PX4 RTK injection.
Sends GGA back-feed every 10 seconds as required by NTRIP v1 casters.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import random
import signal
import socket
import sys
import threading
import time
import uuid
from pathlib import Path

import rclpy
from mavros_msgs.msg import RTCM
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus

from rtcm3_parser import Rtcm3StreamParser, rtcm3_message_type
from rtk_transport import (
    TransportRateTracker,
    classify_ntrip_handshake,
    configure_tcp_keepalive,
    redact_rtk_secrets,
)

# Exit code for confirmed HTTP 401/403 authentication rejection.
AUTH_EXIT_CODE = 2

NTRIP_HOST = ""
NTRIP_PORT = 2101
NTRIP_MOUNTPT = ""
_NTRIP_USER = ""
_NTRIP_PASS = ""


class NtripAuthRejected(Exception):
    """Confirmed HTTP 401/403 authentication rejection."""


class NtripConnectFailure(Exception):
    """NTRIP connect/handshake failure with a classified reason."""

    def __init__(self, reason: str, message: str):
        super().__init__(message)
        self.reason = reason


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Stream RTCM3 corrections from an NTRIP caster into MAVROS."
    )
    parser.add_argument("--host", required=True, help="NTRIP caster hostname or IP")
    parser.add_argument("--port", required=True, type=int, help="NTRIP caster TCP port")
    parser.add_argument("--mountpoint", required=True, help="NTRIP mountpoint")
    parser.add_argument("--user", required=True, help="NTRIP username")
    parser.add_argument(
        "--pass-stdin",
        action="store_true",
        required=True,
        help="Read the NTRIP password from stdin instead of argv",
    )
    parser.add_argument(
        "--status-file",
        help="Optional JSON status file for the FastAPI RTK manager",
    )
    parser.add_argument("--session-id", help="Session identity echoed in status reports")
    parser.add_argument("--connect-timeout-s", type=float, default=10.0)
    parser.add_argument("--recv-timeout-s", type=float, default=12.0)
    parser.add_argument("--no-rtcm-warn-s", type=float, default=15.0)
    parser.add_argument("--no-rtcm-reconnect-s", type=float, default=45.0)
    parser.add_argument("--reconnect-initial-s", type=float, default=2.0)
    parser.add_argument("--reconnect-max-s", type=float, default=30.0)
    parser.add_argument("--reconnect-jitter-frac", type=float, default=0.2)
    parser.add_argument("--publish-error-unhealthy-threshold", type=int, default=5)
    return parser.parse_args(argv)


def configure_from_args(args) -> None:
    global NTRIP_HOST, NTRIP_PORT, NTRIP_MOUNTPT, _NTRIP_USER, _NTRIP_PASS
    NTRIP_HOST = args.host
    NTRIP_PORT = args.port
    NTRIP_MOUNTPT = args.mountpoint
    _NTRIP_USER = args.user
    _NTRIP_PASS = sys.stdin.readline().rstrip("\r\n")
    if not _NTRIP_PASS:
        raise RuntimeError("NTRIP password was not provided on stdin")


def _validate_runtime_config(args) -> None:
    if args.connect_timeout_s <= 0:
        raise ValueError("connect-timeout-s must be > 0")
    if args.recv_timeout_s <= 0:
        raise ValueError("recv-timeout-s must be > 0")
    if args.no_rtcm_warn_s <= 0:
        raise ValueError("no-rtcm-warn-s must be > 0")
    if args.no_rtcm_reconnect_s <= args.no_rtcm_warn_s:
        raise ValueError("no-rtcm-reconnect-s must be > no-rtcm-warn-s")
    if args.reconnect_max_s < args.reconnect_initial_s:
        raise ValueError("reconnect-max-s must be >= reconnect-initial-s")
    if not (0.0 <= args.reconnect_jitter_frac <= 0.5):
        raise ValueError("reconnect-jitter-frac must be between 0.0 and 0.5")
    if args.publish_error_unhealthy_threshold < 1:
        raise ValueError("publish-error-unhealthy-threshold must be >= 1")


def _nmea_checksum(sentence: str) -> str:
    chk = 0
    for c in sentence:
        chk ^= ord(c)
    return f"{chk:02X}"


class NtripNode(Node):
    """ROS2 node that streams RTCM3 corrections from an NTRIP caster to MAVROS."""

    def __init__(
        self,
        *,
        status_file: str | None = None,
        session_id: str | None = None,
        connect_timeout_s: float = 10.0,
        recv_timeout_s: float = 12.0,
        no_rtcm_warn_s: float = 15.0,
        no_rtcm_reconnect_s: float = 45.0,
        reconnect_initial_s: float = 2.0,
        reconnect_max_s: float = 30.0,
        reconnect_jitter_frac: float = 0.2,
        publish_error_unhealthy_threshold: int = 5,
        status_write_interval_s: float = 1.0,
    ):
        super().__init__("ntrip_rtcm_node")
        self._status_file = Path(status_file) if status_file else None
        self._session_id = session_id or uuid.uuid4().hex
        self._process_id = os.getpid()
        self._connect_timeout_s = connect_timeout_s
        self._recv_timeout_s = recv_timeout_s
        self._no_rtcm_warn_s = no_rtcm_warn_s
        self._no_rtcm_reconnect_s = no_rtcm_reconnect_s
        self._reconnect_initial_s = reconnect_initial_s
        self._reconnect_max_s = reconnect_max_s
        self._reconnect_jitter_frac = reconnect_jitter_frac
        self._publish_error_unhealthy_threshold = publish_error_unhealthy_threshold
        self._status_write_interval_s = max(0.25, status_write_interval_s)

        self._parser = Rtcm3StreamParser()
        self._rate_tracker = TransportRateTracker()
        self._inject_rate_tracker = TransportRateTracker()

        rtcm_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(RTCM, "/mavros/gps_rtk/send_rtcm", rtcm_qos)

        gps_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._gps_fix = None
        self._gps_lock = threading.Lock()
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/raw/fix",
            self._gps_callback,
            gps_qos,
        )

        self._stop_event = threading.Event()
        # Set by the stream-health watchdog right before it closes the socket to
        # force a reconnect, so the read thread treats the resulting recv()
        # failure as an intentional reconnect rather than a transport error.
        self._force_reconnect = threading.Event()
        self._gga_lock = threading.Lock()
        self._sock_lock = threading.Lock()
        self._active_sock = None
        self._gga_sock = None
        self._gga_fail_count = 0

        self._stats_lock = threading.Lock()
        self._lifecycle_state = "starting"
        self._connected = False
        self._handshake_complete_monotonic: float | None = None
        self._last_error: str | None = None
        self._transport_reason: str | None = None
        self._bytes_injected = 0
        self._frames_published = 0
        self._valid_rtcm_bytes = 0
        self._publish_error_count = 0
        self._publish_errors_window = 0
        self._publish_errors_window_start = time.monotonic()
        self._injection_healthy = True
        self._no_rtcm_warned = False
        self._status_dirty = True
        self._last_status_write = 0.0
        self._shutting_down = False
        self._reconnect_count = 0
        self._max_frame_gap_s = 0.0
        self._last_frame_gap_anchor: float | None = None

        self.create_timer(10.0, self._send_gga)
        self._status_timer = self.create_timer(
            self._status_write_interval_s,
            self._maybe_write_status,
        )
        self._health_timer = self.create_timer(1.0, self._check_stream_health)

        self.get_logger().info(
            f"Starting NTRIP client session={self._session_id}: "
            f"{NTRIP_HOST}:{NTRIP_PORT}/{NTRIP_MOUNTPT}"
        )
        self._write_status("starting")

        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="ntrip-tcp",
        )
        self._thread.start()

    def request_stop(self) -> None:
        self._stop_event.set()
        self._close_active_socket()

    def _set_lifecycle(self, state: str, *, last_error: str | None = None, reason: str | None = None) -> None:
        with self._stats_lock:
            self._lifecycle_state = state
            if last_error is not None:
                self._last_error = redact_rtk_secrets(last_error)
            if reason is not None:
                self._transport_reason = reason
            self._status_dirty = True

    def _injection_topic_ready(self) -> bool:
        try:
            return self.pub.get_subscription_count() > 0
        except Exception:
            return False

    def _valid_rtcm_age_s(self, now: float, stats) -> float | None:
        if stats.last_valid_frame_monotonic is not None:
            return max(0.0, now - stats.last_valid_frame_monotonic)
        if self._handshake_complete_monotonic is not None:
            return max(0.0, now - self._handshake_complete_monotonic)
        return None

    def _build_status_payload(self, state: str) -> dict:
        with self._stats_lock:
            stats = self._parser.snapshot_stats()
            now = time.monotonic()
            self._rate_tracker.sample(now, stats.valid_frames, stats.bytes_received)
            self._inject_rate_tracker.sample(
                now, self._frames_published, self._bytes_injected
            )
            last_valid_age = self._valid_rtcm_age_s(now, stats)
            return {
                "session_id": self._session_id,
                "process_id": self._process_id,
                "mode": "ntrip",
                "lifecycle_state": state,
                "state": state,
                "host": NTRIP_HOST,
                "port": NTRIP_PORT,
                "mountpoint": NTRIP_MOUNTPT,
                "username": _NTRIP_USER,
                "connected": self._connected,
                "process_alive": True,
                "reconnecting": state in {"reconnecting", "connecting"},
                "valid_frames": stats.valid_frames,
                "invalid_scan_events": stats.invalid_frames,
                "crc_errors": stats.crc_errors,
                "dropped_complete_frames": stats.dropped_frames,
                "bytes_received": stats.bytes_received,
                "valid_rtcm_bytes": self._valid_rtcm_bytes,
                "bytes_injected": self._bytes_injected,
                "frames_published": self._frames_published,
                "last_valid_rtcm_age_s": last_valid_age,
                "valid_frame_rate_hz": self._rate_tracker.valid_frame_rate_hz,
                "bytes_per_sec": self._rate_tracker.bytes_per_sec,
                "injected_frame_rate_hz": self._inject_rate_tracker.valid_frame_rate_hz,
                "injected_bytes_per_sec": self._inject_rate_tracker.bytes_per_sec,
                "publish_error_count": self._publish_error_count,
                "injection_topic_ready": self._injection_topic_ready(),
                "injection_healthy": self._injection_healthy,
                "stream_healthy": (
                    state == "streaming_valid_rtcm"
                    and last_valid_age is not None
                    and last_valid_age <= self._no_rtcm_warn_s
                ),
                "transport_reason": self._transport_reason,
                "last_error": redact_rtk_secrets(self._last_error),
                "last_rtcm_message_type": None,
                "max_frame_gap_s": self._max_frame_gap_s,
                "reconnect_count": self._reconnect_count,
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
        with self._stats_lock:
            state = self._lifecycle_state
        self._write_status(state)

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

    def _write_terminal_auth_status(self) -> None:
        self._set_lifecycle(
            "auth_failed",
            last_error="NTRIP authentication rejected",
            reason="auth_failed",
        )
        self._write_status("auth_failed")

    def _gps_callback(self, msg: NavSatFix) -> None:
        with self._gps_lock:
            self._gps_fix = msg

    def _format_gga(self) -> str | None:
        with self._gps_lock:
            fix = self._gps_fix
        if fix is None:
            return None

        lat = fix.latitude
        lon = fix.longitude
        alt = fix.altitude

        lat_abs = abs(lat)
        lat_deg = int(lat_abs)
        lat_min = (lat_abs - lat_deg) * 60.0
        lat_dir = "N" if lat >= 0 else "S"
        lat_str = f"{lat_deg:02d}{lat_min:07.4f}"

        lon_abs = abs(lon)
        lon_deg = int(lon_abs)
        lon_min = (lon_abs - lon_deg) * 60.0
        lon_dir = "E" if lon >= 0 else "W"
        lon_str = f"{lon_deg:03d}{lon_min:07.4f}"

        stamp_sec = fix.header.stamp.sec
        if stamp_sec > 0:
            t = time.gmtime(stamp_sec)
            time_str = f"{t.tm_hour:02d}{t.tm_min:02d}{t.tm_sec:02d}.00"
        else:
            time_str = "000000.00"

        quality = 1
        if fix.status.status == NavSatStatus.STATUS_GBAS_FIX:
            quality = 4 if fix.position_covariance[0] < 0.01 else 5
        elif fix.status.status == NavSatStatus.STATUS_SBAS_FIX:
            quality = 2

        body = (
            f"GPGGA,{time_str},{lat_str},{lat_dir},{lon_str},{lon_dir},"
            f"{quality},8,1.0,{alt:.1f},M,0.0,M,,"
        )
        ck = _nmea_checksum(body)
        return f"${body}*{ck}\r\n"

    def _send_gga(self) -> None:
        gga = self._format_gga()
        if gga is None:
            return
        with self._gga_lock:
            sock = self._gga_sock
        if sock is None:
            return
        try:
            sock.sendall(gga.encode())
            self._gga_fail_count = 0
        except Exception as e:
            self._gga_fail_count += 1
            if self._gga_fail_count <= 3:
                self.get_logger().warn(f"Failed to send GGA: {redact_rtk_secrets(str(e))}")

    def _set_active_socket(self, sock: socket.socket | None) -> None:
        with self._sock_lock:
            self._active_sock = sock

    def _clear_active_socket(self, sock: socket.socket | None) -> None:
        with self._sock_lock:
            if self._active_sock is sock:
                self._active_sock = None

    def _close_active_socket(self) -> None:
        with self._sock_lock:
            sock = self._active_sock
            self._active_sock = None
        with self._gga_lock:
            self._gga_sock = None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def _check_stream_health(self) -> None:
        if self._shutting_down or not self._connected:
            return
        stats = self._parser.snapshot_stats()
        now = time.monotonic()
        age = self._valid_rtcm_age_s(now, stats)
        if age is None:
            return
        if age >= self._no_rtcm_warn_s and not self._no_rtcm_warned:
            self._no_rtcm_warned = True
            self.get_logger().warn(
                f"No valid RTCM for {age:.0f}s (reconnect at {self._no_rtcm_reconnect_s:.0f}s)"
            )
        if age >= self._no_rtcm_reconnect_s:
            self.get_logger().warn(
                f"No valid RTCM for {age:.0f}s — closing socket to reconnect"
            )
            self._set_lifecycle("no_valid_rtcm", reason="no_valid_rtcm")
            self._force_reconnect.set()
            self._close_active_socket()

    def _connect(self) -> tuple[socket.socket, bytes]:
        creds = base64.b64encode(f"{_NTRIP_USER}:{_NTRIP_PASS}".encode()).decode()
        req = (
            f"GET /{NTRIP_MOUNTPT} HTTP/1.0\r\n"
            f"Host: {NTRIP_HOST}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: NTRIP ROS2/1.0\r\n"
            f"Authorization: Basic {creds}\r\n\r\n"
        )

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        configure_tcp_keepalive(s, idle_s=10.0, interval_s=5.0, count=3)
        s.settimeout(self._connect_timeout_s)
        try:
            s.connect((NTRIP_HOST, NTRIP_PORT))
        except socket.gaierror as exc:
            s.close()
            raise NtripConnectFailure("dns_failed", f"DNS resolution failed: {exc}") from exc
        except (TimeoutError, socket.timeout) as exc:
            s.close()
            raise NtripConnectFailure(
                "caster_unreachable",
                f"NTRIP connect timeout after {self._connect_timeout_s:.0f}s",
            ) from exc
        except OSError as exc:
            s.close()
            raise NtripConnectFailure(
                "caster_unreachable",
                f"NTRIP caster unreachable: {exc}",
            ) from exc

        self._set_active_socket(s)
        try:
            s.sendall(req.encode())
            resp = b""
            while True:
                chunk = s.recv(256)
                if not chunk:
                    raise NtripConnectFailure(
                        "caster_unreachable",
                        "Caster closed connection during handshake",
                    )
                resp += chunk
                if b"\r\n\r\n" in resp:
                    header, leftover = resp.split(b"\r\n\r\n", 1)
                    header_text = header.decode(errors="ignore")
                    break
                if resp.startswith(b"ICY 200 OK") and b"\r\n" in resp:
                    lines = resp.split(b"\r\n", 1)
                    header_text = lines[0].decode(errors="ignore")
                    leftover = lines[1] if len(lines) > 1 else b""
                    break
                if len(resp) > 2048:
                    raise NtripConnectFailure(
                        "protocol_error",
                        "NTRIP handshake response too large",
                    )
        except Exception:
            self._clear_active_socket(s)
            try:
                s.close()
            except OSError:
                pass
            raise

        outcome, err = classify_ntrip_handshake(header_text)
        if outcome == "auth_failed":
            self._clear_active_socket(s)
            try:
                s.close()
            except OSError:
                pass
            raise NtripAuthRejected(err or "NTRIP authentication rejected")
        if outcome != "ok":
            self._clear_active_socket(s)
            try:
                s.close()
            except OSError:
                pass
            raise NtripConnectFailure(outcome, err or outcome)

        s.settimeout(self._recv_timeout_s)
        with self._gga_lock:
            self._gga_sock = s
        with self._stats_lock:
            self._connected = True
            self._handshake_complete_monotonic = time.monotonic()
            self._no_rtcm_warned = False
            self._last_error = None
            self._transport_reason = None
        self._set_lifecycle("connected")
        self.get_logger().info("Connected — awaiting valid RTCM3")
        return s, leftover

    def _record_publish_error(self, exc: Exception) -> None:
        now = time.monotonic()
        with self._stats_lock:
            self._publish_error_count += 1
            if now - self._publish_errors_window_start >= 10.0:
                self._publish_errors_window_start = now
                self._publish_errors_window = 0
            self._publish_errors_window += 1
            self._last_error = redact_rtk_secrets(str(exc))
            if self._publish_errors_window >= self._publish_error_unhealthy_threshold:
                self._injection_healthy = False
            self._status_dirty = True

    def _record_publish_success(self, frame_len: int) -> None:
        now = time.monotonic()
        with self._stats_lock:
            self._bytes_injected += frame_len
            self._frames_published += 1
            self._publish_errors_window = 0
            self._publish_errors_window_start = now
            self._injection_healthy = True
            self._last_error = None
            if self._last_frame_gap_anchor is not None:
                gap = now - self._last_frame_gap_anchor
                if gap > self._max_frame_gap_s:
                    self._max_frame_gap_s = gap
            self._last_frame_gap_anchor = now
            self._status_dirty = True

    def _publish_frames(self, frames: list[bytes]) -> None:
        topic_ready = self._injection_topic_ready()
        for frame in frames:
            with self._stats_lock:
                self._valid_rtcm_bytes += len(frame)
            if not topic_ready:
                continue
            msg = RTCM()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.data = list(frame)
            try:
                self.pub.publish(msg)
            except Exception as exc:
                self._record_publish_error(exc)
                continue
            self._record_publish_success(len(frame))
            self._set_lifecycle("streaming_valid_rtcm")

    def _backoff_delay(self, attempt: int) -> float:
        base = min(self._reconnect_initial_s * (2 ** attempt), self._reconnect_max_s)
        jitter = self._reconnect_jitter_frac
        if jitter <= 0:
            return base
        scale = 1.0 + random.uniform(-jitter, jitter)
        return max(0.0, base * scale)

    def _run(self) -> None:
        attempt = 0

        while not self._stop_event.is_set():
            sock = None
            self._force_reconnect.clear()
            try:
                self._set_lifecycle("connecting")
                sock, buf = self._connect()
                attempt = 0

                while not self._stop_event.is_set():
                    stats = self._parser.snapshot_stats()
                    age = self._valid_rtcm_age_s(time.monotonic(), stats)
                    if age is not None and age >= self._no_rtcm_reconnect_s:
                        self._set_lifecycle("no_valid_rtcm", reason="no_valid_rtcm")
                        break

                    try:
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        continue
                    except OSError:
                        # Socket closed under the blocked recv() — by the
                        # stream-health watchdog forcing a reconnect, or by
                        # request_stop(). Intentional, not a transport error:
                        # break to the clean reconnect path instead of logging
                        # an ERROR and tagging transport_reason=transport_error.
                        if self._stop_event.is_set() or self._force_reconnect.is_set():
                            break
                        raise

                    if not chunk:
                        if not self._force_reconnect.is_set():
                            self.get_logger().warn("NTRIP stream ended, reconnecting")
                        break

                    frames = self._parser.feed(chunk)
                    if frames:
                        self._publish_frames(frames)

            except NtripAuthRejected as exc:
                self.get_logger().error(redact_rtk_secrets(str(exc)))
                self._write_terminal_auth_status()
                self._stop_event.set()
                return
            except NtripConnectFailure as exc:
                self._reconnect_count += 1
                with self._stats_lock:
                    self._connected = False
                    self._handshake_complete_monotonic = None
                self._set_lifecycle(
                    exc.reason,
                    last_error=str(exc),
                    reason=exc.reason,
                )
                self.get_logger().error(
                    f"NTRIP connect error ({exc.reason}): {redact_rtk_secrets(str(exc))}"
                )
            except Exception as exc:
                self._reconnect_count += 1
                with self._stats_lock:
                    self._connected = False
                    self._handshake_complete_monotonic = None
                self._set_lifecycle(
                    "reconnecting",
                    last_error=str(exc),
                    reason="transport_error",
                )
                self.get_logger().error(
                    f"NTRIP error: {redact_rtk_secrets(str(exc))}"
                )
            finally:
                with self._stats_lock:
                    self._connected = False
                self._clear_active_socket(sock)
                if sock is not None:
                    try:
                        sock.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        pass
                    try:
                        sock.close()
                    except OSError:
                        pass

            if self._stop_event.is_set():
                break

            self._set_lifecycle("reconnecting", reason="backoff")
            delay = self._backoff_delay(attempt)
            attempt += 1
            self.get_logger().info(f"Reconnecting in {delay:.1f}s...")
            self._stop_event.wait(delay)

    def _destroy_timers(self) -> None:
        for label, attr in (
            ("status", "_status_timer"),
            ("health", "_health_timer"),
        ):
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
        self.get_logger().info("Shutting down NTRIP node...")
        # Preserve a terminal auth_failed status: the manager/operator must see
        # the real cause. Only a non-terminal node reports "stopping" on exit.
        with self._stats_lock:
            terminal_auth = self._lifecycle_state == "auth_failed"
        if not terminal_auth:
            self._write_status("stopping")
        self.request_stop()
        if self._thread.is_alive():
            self._thread.join(timeout=5)
        self._destroy_timers()
        super().destroy_node()


def main(argv=None):
    args = parse_args(argv)
    _validate_runtime_config(args)
    configure_from_args(args)

    rclpy.init()
    node = NtripNode(
        status_file=args.status_file,
        session_id=args.session_id,
        connect_timeout_s=args.connect_timeout_s,
        recv_timeout_s=args.recv_timeout_s,
        no_rtcm_warn_s=args.no_rtcm_warn_s,
        no_rtcm_reconnect_s=args.no_rtcm_reconnect_s,
        reconnect_initial_s=args.reconnect_initial_s,
        reconnect_max_s=args.reconnect_max_s,
        reconnect_jitter_frac=args.reconnect_jitter_frac,
        publish_error_unhealthy_threshold=args.publish_error_unhealthy_threshold,
    )

    def _handle_signal(signum, _frame):
        node.get_logger().info(f"Received signal {signum}; stopping NTRIP node")
        node.request_stop()
        rclpy.try_shutdown()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    exit_code = 0
    try:
        while rclpy.ok():
            if node._lifecycle_state == "auth_failed":
                exit_code = AUTH_EXIT_CODE
                break
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if node._lifecycle_state == "auth_failed":
            exit_code = AUTH_EXIT_CODE
        node.destroy_node()
        rclpy.try_shutdown()
    if exit_code:
        sys.exit(exit_code)


if __name__ == "__main__":
    main()