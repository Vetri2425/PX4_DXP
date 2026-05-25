"""Tests for path_engine spray latency compensation."""

import math

from path_engine.core import PathSegment, SegmentType
from path_engine.spray import apply_spray_latency_compensation, _shift_point


def test_shift_point_basic():
    """Shift point towards another by a given distance."""
    # Shift (0,0) towards (1,0) by 0.5m → (0.5, 0)
    result = _shift_point((0.0, 0.0), (1.0, 0.0), 0.5)
    assert abs(result[0] - 0.5) < 0.001
    assert abs(result[1]) < 0.001


def test_shift_point_away():
    """Negative distance shifts away from target."""
    # Shift (0,0) away from (1,0) by -0.5m → (-0.5, 0)
    result = _shift_point((0.0, 0.0), (1.0, 0.0), -0.5)
    assert abs(result[0] - (-0.5)) < 0.001
    assert abs(result[1]) < 0.001


def test_shift_point_diagonal():
    """Diagonal shift preserves direction."""
    result = _shift_point((0.0, 0.0), (3.0, 4.0), 5.0)  # 3-4-5 triangle
    assert abs(result[0] - 3.0) < 0.001
    assert abs(result[1] - 4.0) < 0.001


def test_shift_point_zero_distance():
    """Zero distance returns original point."""
    result = _shift_point((1.0, 2.0), (5.0, 6.0), 0.0)
    assert result == (1.0, 2.0)


def test_shift_point_coincident():
    """Coincident points: shift returns original point (can't determine direction)."""
    result = _shift_point((1.0, 2.0), (1.0, 2.0), 0.5)
    assert result == (1.0, 2.0)


def test_spray_compensation_mark_lead_in():
    """MARK segment gets a pre-start point shifted opposite to travel direction."""
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (1.0, 0.0)],
        speed=0.35,
    )
    result = apply_spray_latency_compensation(seg, spray_on_latency_s=0.10, spray_off_latency_s=0.01)
    # Lead-in: 0.10s × 0.35 m/s = 0.035m before start
    assert len(result.points) == 3  # pre-start + original 2 + modified end
    # Pre-start point should be at (-0.035, 0)
    assert abs(result.points[0][0] - (-0.035)) < 0.001
    assert abs(result.points[0][1]) < 0.001


def test_spray_compensation_mark_lead_out():
    """MARK segment end point is trimmed towards second-to-last."""
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (1.0, 0.0)],
        speed=0.35,
    )
    result = apply_spray_latency_compensation(seg, spray_on_latency_s=0.10, spray_off_latency_s=0.01)
    # Lead-out: 0.01s × 0.35 m/s = 0.0035m trimmed from end
    # After lead-in insertion, points are [(-0.035,0), (0,0), (1,0)]
    # lead-out shifts last point towards second-to-last by 0.0035
    # But second-to-last is now (0,0) — the shift direction is (0,0) to (1,0)
    # Moving (1,0) towards (0,0) by 0.0035 gives (0.9965, 0)
    end_n = result.points[-1][0]
    end_e = result.points[-1][1]
    assert end_n < 1.0  # Trimmed back from 1.0
    assert abs(end_e) < 0.001


def test_spray_compensation_transit_passthrough():
    """TRANSIT segments pass through unchanged."""
    seg = PathSegment(
        segment_type=SegmentType.TRANSIT,
        points=[(0.0, 0.0), (5.0, 0.0)],
        speed=0.50,
    )
    result = apply_spray_latency_compensation(seg)
    assert result.segment_type == SegmentType.TRANSIT
    assert result.points == [(0.0, 0.0), (5.0, 0.0)]


def test_spray_compensation_single_point():
    """Single-point MARK segment passes through unchanged (no direction to shift)."""
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(1.0, 2.0)],
        speed=0.35,
    )
    result = apply_spray_latency_compensation(seg)
    assert result.points == [(1.0, 2.0)]


def test_spray_compensation_preserves_metadata():
    """Compensation preserves segment_id and source_entity."""
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0, 0), (1, 0)],
        speed=0.35,
        segment_id=5,
        source_entity="LINE_E005",
    )
    result = apply_spray_latency_compensation(seg)
    assert result.segment_id == 5
    assert result.source_entity == "LINE_E005"