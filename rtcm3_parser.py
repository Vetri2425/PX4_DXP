"""Streaming RTCM3 frame parser with CRC-24Q validation.

Used by LoRa and NTRIP RTCM injection nodes.  Only complete frames with valid
CRC are returned from :meth:`Rtcm3StreamParser.feed`.  Injection-byte accounting
belongs to the transport node, not the parser.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field, replace
from typing import Iterable

# RTCM3 protocol constants
RTCM3_PREAMBLE = 0xD3
RTCM3_HEADER_LEN = 3
RTCM3_CRC_LEN = 3
RTCM3_MAX_PAYLOAD_LEN = 1023  # 10-bit length field maximum
RTCM3_MAX_FRAME_LEN = RTCM3_HEADER_LEN + RTCM3_MAX_PAYLOAD_LEN + RTCM3_CRC_LEN

# CRC-24Q lookup table (polynomial 0x1864CFB)
_CRC24Q_TABLE = [0] * 256
for _i in range(256):
    _crc = _i << 16
    for _ in range(8):
        _crc = ((_crc << 1) ^ 0x1864CFB) if (_crc & 0x800000) else (_crc << 1)
        _crc &= 0xFFFFFF
    _CRC24Q_TABLE[_i] = _crc


def rtcm3_crc24q(data: bytes, length: int) -> int:
    """Compute RTCM3 CRC-24Q over ``data[0:length]``."""
    crc = 0
    for i in range(length):
        crc = ((crc << 8) ^ _CRC24Q_TABLE[((crc >> 16) & 0xFF) ^ data[i]]) & 0xFFFFFF
    return crc


def rtcm3_message_type(frame: bytes) -> int | None:
    """Extract the 12-bit RTCM message type from a complete frame."""
    if len(frame) < RTCM3_HEADER_LEN + 2:
        return None
    payload = frame[RTCM3_HEADER_LEN:]
    return ((payload[0] << 4) | (payload[1] >> 4)) & 0xFFF


def build_rtcm3_frame(payload: bytes) -> bytes:
    """Build a valid RTCM3 frame from raw payload bytes (for tests)."""
    if len(payload) > RTCM3_MAX_PAYLOAD_LEN:
        raise ValueError(f"payload too long: {len(payload)}")
    length_field = len(payload) & 0x03FF
    header = bytes([RTCM3_PREAMBLE, (length_field >> 8) & 0xFF, length_field & 0xFF])
    body = header + payload
    crc = rtcm3_crc24q(body, len(body))
    return body + bytes([(crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF])


@dataclass(frozen=True)
class Rtcm3ParseStats:
    valid_frames: int = 0
    invalid_frames: int = 0
    crc_errors: int = 0
    dropped_frames: int = 0
    bytes_received: int = 0
    last_valid_frame_monotonic: float | None = None


@dataclass
class Rtcm3StreamParser:
    """Incremental RTCM3 parser with bounded buffer and optional filters."""

    max_frame_size: int = RTCM3_MAX_FRAME_LEN
    max_buffer_size: int = 8192
    allowed_message_types: frozenset[int] | None = None
    max_bytes_per_sec: float = 65536.0
    max_frames_per_sec: float = 50.0
    _stats: Rtcm3ParseStats = field(default_factory=Rtcm3ParseStats, init=False, repr=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)
    _buf: bytearray = field(default_factory=bytearray, init=False, repr=False)
    _rate_window_start: float = field(default_factory=time.monotonic, init=False, repr=False)
    _rate_window_bytes: int = field(default=0, init=False, repr=False)
    _rate_window_frames: int = field(default=0, init=False, repr=False)

    @property
    def stats(self) -> Rtcm3ParseStats:
        """Backward-compatible stats view (immutable snapshot)."""
        return self.snapshot_stats()

    def snapshot_stats(self) -> Rtcm3ParseStats:
        with self._lock:
            return replace(self._stats)

    def reset_session_stats(self) -> None:
        with self._lock:
            self._stats = Rtcm3ParseStats()
            self._rate_window_start = time.monotonic()
            self._rate_window_bytes = 0
            self._rate_window_frames = 0
            self._buf.clear()

    def feed(self, chunk: bytes) -> list[bytes]:
        """Consume raw bytes and return zero or more validated RTCM3 frames."""
        if not chunk:
            return []
        with self._lock:
            return self._feed_locked(chunk)

    def _feed_locked(self, chunk: bytes) -> list[bytes]:
        self._stats = replace(self._stats, bytes_received=self._stats.bytes_received + len(chunk))
        self._buf.extend(chunk)
        if len(self._buf) > self.max_buffer_size:
            overflow = len(self._buf) - self.max_buffer_size
            del self._buf[:overflow]
            self._stats = replace(
                self._stats,
                invalid_frames=self._stats.invalid_frames + 1,
                dropped_frames=self._stats.dropped_frames + 1,
            )

        frames: list[bytes] = []
        i = 0
        buf = self._buf
        while i < len(buf):
            if buf[i] != RTCM3_PREAMBLE:
                i += 1
                self._stats = replace(self._stats, invalid_frames=self._stats.invalid_frames + 1)
                continue

            if i + RTCM3_HEADER_LEN > len(buf):
                break

            length_field = (buf[i + 1] << 8) | buf[i + 2]
            msg_len = length_field & 0x03FF
            total_frame = RTCM3_HEADER_LEN + msg_len + RTCM3_CRC_LEN

            if msg_len > RTCM3_MAX_PAYLOAD_LEN or total_frame > self.max_frame_size:
                i += 1
                self._stats = replace(
                    self._stats,
                    invalid_frames=self._stats.invalid_frames + 1,
                    dropped_frames=self._stats.dropped_frames + 1,
                )
                continue

            if i + total_frame > len(buf):
                break

            frame = bytes(buf[i : i + total_frame])
            payload_len = RTCM3_HEADER_LEN + msg_len
            expected_crc = (
                (frame[payload_len] << 16)
                | (frame[payload_len + 1] << 8)
                | frame[payload_len + 2]
            )
            computed_crc = rtcm3_crc24q(frame, payload_len)
            if computed_crc != expected_crc:
                self._stats = replace(
                    self._stats,
                    crc_errors=self._stats.crc_errors + 1,
                    invalid_frames=self._stats.invalid_frames + 1,
                    dropped_frames=self._stats.dropped_frames + 1,
                )
                i += 1
                continue

            msg_type = rtcm3_message_type(frame)
            if self.allowed_message_types and msg_type not in self.allowed_message_types:
                self._stats = replace(self._stats, dropped_frames=self._stats.dropped_frames + 1)
                i += total_frame
                continue

            if not self._rate_allow_locked(len(frame)):
                self._stats = replace(self._stats, dropped_frames=self._stats.dropped_frames + 1)
                i += total_frame
                continue

            frames.append(frame)
            self._stats = replace(
                self._stats,
                valid_frames=self._stats.valid_frames + 1,
                last_valid_frame_monotonic=time.monotonic(),
            )
            i += total_frame

        del self._buf[:i]
        return frames

    def _rate_allow_locked(self, frame_len: int) -> bool:
        now = time.monotonic()
        elapsed = now - self._rate_window_start
        if elapsed >= 1.0:
            self._rate_window_start = now
            self._rate_window_bytes = 0
            self._rate_window_frames = 0
            elapsed = 0.0

        projected_bytes = self._rate_window_bytes + frame_len
        projected_frames = self._rate_window_frames + 1
        if elapsed < 1.0:
            over_limit = (
                projected_bytes > self.max_bytes_per_sec
                or projected_frames > self.max_frames_per_sec
            )
        else:
            over_limit = (
                projected_bytes / elapsed > self.max_bytes_per_sec
                or projected_frames / elapsed > self.max_frames_per_sec
            )

        if over_limit:
            return False

        self._rate_window_bytes = projected_bytes
        self._rate_window_frames = projected_frames
        return True


def parse_allowed_message_types(raw: str | None) -> frozenset[int] | None:
    """Parse comma-separated RTCM message type list; empty => allow all."""
    if raw is None or not str(raw).strip():
        return None
    types: set[int] = set()
    for part in str(raw).split(","):
        part = part.strip()
        if not part:
            continue
        types.add(int(part))
    return frozenset(types) if types else None