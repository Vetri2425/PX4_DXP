"""Tests for TransportRateTracker delta calculations."""

from __future__ import annotations

from rtk_transport import TransportRateTracker


def test_frame_rate_delta():
    tracker = TransportRateTracker(min_interval_s=2.0)
    tracker.sample(1000.0, valid_frames=0, bytes_received=0)
    tracker.sample(1002.0, valid_frames=10, bytes_received=1000)
    assert tracker.valid_frame_rate_hz == 5.0
    assert tracker.bytes_per_sec == 500.0


def test_counter_reset_does_not_negative_rate():
    tracker = TransportRateTracker(min_interval_s=1.0)
    tracker.sample(0.0, valid_frames=100, bytes_received=5000)
    tracker.sample(2.0, valid_frames=5, bytes_received=200)
    assert tracker.valid_frame_rate_hz is None
    tracker.sample(4.0, valid_frames=15, bytes_received=1200)
    assert tracker.valid_frame_rate_hz == 5.0
    assert tracker.bytes_per_sec == 500.0


def test_short_interval_preserves_last_rate():
    tracker = TransportRateTracker(min_interval_s=2.0)
    tracker.sample(0.0, valid_frames=0, bytes_received=0)
    tracker.sample(2.0, valid_frames=10, bytes_received=1000)
    tracker.sample(2.5, valid_frames=20, bytes_received=2000)
    assert tracker.valid_frame_rate_hz == 5.0
    assert tracker.bytes_per_sec == 500.0