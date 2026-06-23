"""Transport helpers shared by LoRa and NTRIP RTCM injection (no ROS dependencies)."""

from __future__ import annotations

import re
import socket
import sys
from dataclasses import dataclass


@dataclass
class TransportRateTracker:
    """Delta-based transport rate calculator (serial input rates)."""

    min_interval_s: float = 0.5
    valid_frame_rate_hz: float | None = None
    bytes_per_sec: float | None = None
    _last_sample_time: float | None = None
    _last_valid_frames: int = 0
    _last_bytes_received: int = 0

    def reset(self) -> None:
        self.valid_frame_rate_hz = None
        self.bytes_per_sec = None
        self._last_sample_time = None
        self._last_valid_frames = 0
        self._last_bytes_received = 0

    def sample(self, now: float, valid_frames: int, bytes_received: int) -> None:
        if self._last_sample_time is None:
            self._last_sample_time = now
            self._last_valid_frames = valid_frames
            self._last_bytes_received = bytes_received
            return

        elapsed = now - self._last_sample_time
        if elapsed < self.min_interval_s:
            return

        frame_delta = valid_frames - self._last_valid_frames
        byte_delta = bytes_received - self._last_bytes_received
        if frame_delta < 0 or byte_delta < 0:
            self._last_sample_time = now
            self._last_valid_frames = valid_frames
            self._last_bytes_received = bytes_received
            return

        self.valid_frame_rate_hz = frame_delta / elapsed
        self.bytes_per_sec = byte_delta / elapsed
        self._last_sample_time = now
        self._last_valid_frames = valid_frames
        self._last_bytes_received = bytes_received


def configure_tcp_keepalive(
    sock: socket.socket,
    *,
    idle_s: float = 10.0,
    interval_s: float = 5.0,
    count: int = 3,
) -> None:
    """Enable TCP keepalive with platform-specific idle/interval/count tuning."""
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    if sys.platform == "darwin":
        # macOS uses TCP_KEEPALIVE for idle time (seconds).
        if hasattr(socket, "TCP_KEEPALIVE"):
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPALIVE, int(idle_s))
        return

    if sys.platform.startswith("linux"):
        if hasattr(socket, "TCP_KEEPIDLE"):
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, int(idle_s))
        if hasattr(socket, "TCP_KEEPINTVL"):
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, int(interval_s))
        if hasattr(socket, "TCP_KEEPCNT"):
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, int(count))


_SECRET_PATTERNS = (
    re.compile(r"(?i)(authorization:\s*basic\s+)[A-Za-z0-9+/=]+"),
    re.compile(r"(?i)(basic\s+)[A-Za-z0-9+/=]{8,}"),
    re.compile(r"(?i)((?:password|passwd|pass)\s*[:=]\s*)[^\s,;]+"),
)


def sanitize_ntrip_response_line(header: str) -> str:
    """Return a short status line without response bodies or credentials."""
    first = header.split("\r\n", 1)[0].strip()
    return first[:160] if first else "empty response"


def ntrip_http_status_code(first_line: str) -> int | None:
    upper = first_line.upper()
    if upper.startswith("ICY"):
        return 200 if "200" in upper else None
    match = re.match(r"HTTP/\d\.\d\s+(\d{3})", first_line, re.IGNORECASE)
    if match:
        return int(match.group(1))
    return None


def classify_ntrip_handshake(header: str) -> tuple[str, str | None]:
    """Classify caster handshake outcome.

    Returns (outcome, error_message). outcome is 'ok' or a lifecycle reason.
    Only HTTP 401/403 map to auth_failed.
    """
    line = sanitize_ntrip_response_line(header)
    code = ntrip_http_status_code(line)
    upper = line.upper()

    if code == 401 or code == 403:
        return "auth_failed", f"NTRIP authentication rejected (HTTP {code})"
    if code == 404:
        return "mount_not_found", f"NTRIP mountpoint not found (HTTP 404): {line}"
    if code is not None and 500 <= code <= 599:
        return "caster_unreachable", f"NTRIP caster server error (HTTP {code})"
    if "SOURCETABLE" in upper and code != 200:
        return "mount_not_found", f"NTRIP sourcetable returned instead of stream: {line}"
    if code == 200 or (upper.startswith("ICY") and "200" in upper):
        return "ok", None
    if code is not None and 400 <= code <= 499 and code not in {401, 403, 404}:
        return "protocol_error", f"NTRIP client/protocol error (HTTP {code}): {line}"
    if code is not None and code >= 400:
        return "protocol_error", f"NTRIP handshake rejected (HTTP {code}): {line}"
    return "protocol_error", f"Unrecognized NTRIP handshake response: {line}"


def redact_rtk_secrets(text: str | None) -> str | None:
    """Defense-in-depth redaction for RTK status and log error strings."""
    if text is None:
        return None
    redacted = str(text)
    for pattern in _SECRET_PATTERNS:
        redacted = pattern.sub(r"\1***", redacted)
    return redacted