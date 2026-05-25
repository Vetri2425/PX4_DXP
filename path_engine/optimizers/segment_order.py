"""Segment ordering optimization — nearest-neighbor TSP with endpoint reversal.

Reorders MARK segments to minimize total transit distance. At each step,
picks the nearest unvisited segment by considering both endpoints.
If entering from the end point, the segment's point order is reversed.

Inserts TRANSIT segments between consecutive MARK segments.
"""

from __future__ import annotations

import math

from ..core import PathSegment, SegmentType


def _distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
    """Euclidean distance between two NED points."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def optimize_segment_order(
    segments: list[PathSegment],
    start_position: tuple[float, float] | None = None,
    transit_speed: float = 0.50,
) -> list[PathSegment]:
    """Reorder MARK segments using nearest-neighbor heuristic with endpoint reversal.

    At each step, considers both endpoints of each unvisited MARK segment.
    If the nearest approach is via the segment's end point, the segment's
    point order is reversed so the rover enters from that end.

    Inserts TRANSIT segments between consecutive MARK segments with
    spray_on=False and speed=transit_speed.

    Args:
        segments: Input segments (MARK and TRANSIT).
        start_position: Rover starting (north, east) position. If None,
                        starts from the first segment's start point.
        transit_speed: Speed for inserted TRANSIT segments (m/s).

    Returns:
        Reordered segments with TRANSIT segments inserted between MARK segments.
        MARK segments may have their point order reversed for optimal traversal.
    """
    mark_segments = [s for s in segments if s.segment_type == SegmentType.MARK]
    if not mark_segments:
        return segments  # No MARK segments — nothing to reorder

    if len(mark_segments) == 1:
        # Single segment — just add transit from start if needed
        result: list[PathSegment] = []
        seg = mark_segments[0]
        if start_position and seg.points:
            d = _distance(start_position, seg.points[0])
            if d > 0.01:
                result.append(PathSegment(
                    segment_type=SegmentType.TRANSIT,
                    points=[start_position, seg.points[0]],
                    speed=transit_speed,
                    source_entity="transit:start",
                ))
        result.append(seg)
        return result

    # Nearest-neighbor heuristic with endpoint reversal
    remaining: list[tuple[int, PathSegment]] = [(i, s) for i, s in enumerate(mark_segments)]
    ordered: list[PathSegment] = []

    # Start from start_position or first segment
    if start_position:
        current_pos = start_position
    else:
        first = mark_segments[0]
        current_pos = first.points[0] if first.points else (0.0, 0.0)

    transit_count = 0

    while remaining:
        best_idx = 0
        best_dist = float("inf")
        best_reverse = False

        for idx, (orig_i, seg) in enumerate(remaining):
            if not seg.points:
                continue

            # Distance to start of segment
            d_start = _distance(current_pos, seg.points[0])
            if d_start < best_dist:
                best_dist = d_start
                best_idx = idx
                best_reverse = False

            # Distance to end of segment (entering backwards)
            d_end = _distance(current_pos, seg.points[-1])
            if d_end < best_dist:
                best_dist = d_end
                best_idx = idx
                best_reverse = True

        orig_i, seg = remaining.pop(best_idx)

        # Reverse point order if entering from the end
        if best_reverse and len(seg.points) > 1:
            seg = PathSegment(
                segment_type=seg.segment_type,
                points=list(reversed(seg.points)),
                speed=seg.speed,
                segment_id=seg.segment_id,
                source_entity=seg.source_entity,
            )

        # Insert TRANSIT segment from current position to segment start
        if ordered and seg.points:
            transit_start = current_pos
            transit_end = seg.points[0]
            d = _distance(transit_start, transit_end)
            if d > 0.01:
                transit_count += 1
                ordered.append(PathSegment(
                    segment_type=SegmentType.TRANSIT,
                    points=[transit_start, transit_end],
                    speed=transit_speed,
                    source_entity=f"transit:{transit_count}",
                ))

        ordered.append(seg)
        if seg.points:
            current_pos = seg.points[-1]

    return ordered