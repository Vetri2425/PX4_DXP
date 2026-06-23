"""Transport helpers shared by LoRa RTCM injection (no ROS dependencies)."""

from __future__ import annotations

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