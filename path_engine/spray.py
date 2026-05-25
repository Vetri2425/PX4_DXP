"""Spray latency compensation for marking solenoid.

When the solenoid opens, there's a delay before paint reaches the surface.
When it closes, there's a shorter delay before paint stops.

This module shifts the spray ON/OFF boundaries to compensate:
- Spray ON fires early (lead-in) so paint is flowing when the line starts
- Spray OFF fires early (lead-out) so paint stops when the line ends

Values are based on the marking_rover_core mission_generator defaults:
  spray_on_latency  = 0.10 s  (time for solenoid to fully open)
  spray_off_latency = 0.01 s  (time for solenoid to close — much shorter)
  marking_speed     = 0.35 m/s

Lead-in distance  = 0.10 × 0.35 = 0.035 m (3.5 cm)
Lead-out distance  = 0.01 × 0.35 = 0.0035 m (3.5 mm)
"""

from __future__ import annotations

import math

from .core import PathSegment, SegmentType


def _shift_point(
    point: tuple[float, float],
    towards: tuple[float, float],
    distance: float,
) -> tuple[float, float]:
    """Shift `point` along the direction towards `towards` by `distance` metres."""
    dx = towards[0] - point[0]
    dy = towards[1] - point[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return point
    return (
        point[0] + distance * dx / length,
        point[1] + distance * dy / length,
    )


def apply_spray_latency_compensation(
    segment: PathSegment,
    spray_on_latency_s: float = 0.10,
    spray_off_latency_s: float = 0.01,
) -> PathSegment:
    """Shift spray start/end points to compensate for solenoid latency.

    For MARK segments only:
    - Pre-start point added: spray fires early by (latency × speed) metres
    - End point trimmed: spray stops early by (latency × speed) metres

    TRANSIT segments are passed through unchanged.

    Args:
        segment: Input PathSegment.
        spray_on_latency_s: Seconds for solenoid to fully open (default 0.10).
        spray_off_latency_s: Seconds for solenoid to close (default 0.01).

    Returns:
        New PathSegment with compensated endpoints.
    """
    if segment.segment_type != SegmentType.MARK or len(segment.points) < 2:
        return PathSegment(
            segment_type=segment.segment_type,
            points=list(segment.points),
            speed=segment.speed,
            segment_id=segment.segment_id,
            source_entity=segment.source_entity,
        )

    speed = segment.speed
    lead_in = spray_on_latency_s * speed   # 3.5 cm at 0.35 m/s
    lead_out = spray_off_latency_s * speed  # 3.5 mm at 0.35 m/s

    pts = list(segment.points)

    # Pre-start: add a point before the first waypoint, shifted along
    # the direction from first to second point
    if lead_in > 0.001 and len(pts) >= 2:
        pre_start = _shift_point(pts[0], pts[1], -lead_in)
        pts.insert(0, pre_start)

    # Early end: trim the last point towards the second-to-last
    if lead_out > 0.001 and len(pts) >= 2:
        pts[-1] = _shift_point(pts[-1], pts[-2], lead_out)

    return PathSegment(
        segment_type=segment.segment_type,
        points=pts,
        speed=segment.speed,
        segment_id=segment.segment_id,
        source_entity=segment.source_entity,
    )