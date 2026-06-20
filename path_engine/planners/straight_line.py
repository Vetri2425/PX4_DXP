"""Straight-line waypoint densification.

Generates equally-spaced waypoints along line segments for precise
path following. Tighter spacing on MARK segments (spray ON) for
drawing accuracy, coarser spacing on TRANSIT for faster travel.
"""

from __future__ import annotations

import math

from ..core import PathSegment, SegmentType

# Float-noise guard for interval counting. A segment whose length equals the
# spacing can arrive as ``0.05000000000000004`` after upstream arithmetic;
# without this epsilon ``ceil(length / spacing)`` rounds 1.0000…→2 and splits a
# clean 5 cm interval into two 2.5 cm ones. Subdividing only when the length
# exceeds one spacing by more than _SPACING_EPS keeps the max-interval guarantee
# (worst case spacing + 1e-6) while eliminating the spurious split. 1e-6 matches
# the project's geometry tolerance (see tests/test_production_geometry._TOL).
_SPACING_EPS = 1e-6


def densify_line(
    start: tuple[float, float],
    end: tuple[float, float],
    spacing: float = 0.05,
) -> list[tuple[float, float]]:
    """Generate equally-spaced waypoints along a straight line.

    Args:
        start: (north_m, east_m) start point.
        end: (north_m, east_m) end point.
        spacing: Distance between waypoints in metres.

    Returns:
        List of (north_m, east_m) from start to end inclusive.
        Always includes both endpoints exactly.
    """
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)

    if length < 1e-9:
        return [start]

    n_intervals = max(1, int(math.ceil((length - _SPACING_EPS) / spacing)))
    n_steps = n_intervals + 1
    pts: list[tuple[float, float]] = []
    for i in range(n_steps):
        t = i / (n_steps - 1)
        n = start[0] + t * dx
        e = start[1] + t * dy
        pts.append((n, e))

    # Force exact endpoints
    pts[0] = start
    pts[-1] = end
    return pts


def densify_segment(
    segment: PathSegment,
    mark_spacing: float = 0.05,
    transit_spacing: float = 0.15,
) -> PathSegment:
    """Densify a PathSegment's points at the appropriate spacing.

    For MARK segments, uses mark_spacing (default 5cm for drawing accuracy).
    For TRANSIT segments, uses transit_spacing (default 15cm for faster travel).

    Single-point segments (from POINT entities) are passed through unchanged.

    Args:
        segment: Input segment with potentially sparse points.
        mark_spacing: Waypoint spacing for MARK segments (metres).
        transit_spacing: Waypoint spacing for TRANSIT segments (metres).

    Returns:
        New PathSegment with densified points, preserving all other attributes.
    """
    if len(segment.points) <= 1:
        # Single point or empty — pass through
        return PathSegment(
            segment_type=segment.segment_type,
            points=list(segment.points),
            speed=segment.speed,
            segment_id=segment.segment_id,
            source_entity=segment.source_entity,
            metadata=dict(segment.metadata),
        )

    spacing = mark_spacing if segment.segment_type == SegmentType.MARK else transit_spacing
    dense_pts: list[tuple[float, float]] = []

    for i in range(len(segment.points) - 1):
        line_pts = densify_line(segment.points[i], segment.points[i + 1], spacing)
        # Avoid duplicating the junction point
        if dense_pts and line_pts:
            dense_pts.extend(line_pts[1:])
        else:
            dense_pts.extend(line_pts)

    return PathSegment(
        segment_type=segment.segment_type,
        points=dense_pts,
        speed=segment.speed,
        segment_id=segment.segment_id,
        source_entity=segment.source_entity,
        metadata=dict(segment.metadata),
    )