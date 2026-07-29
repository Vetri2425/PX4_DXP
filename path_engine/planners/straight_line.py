"""Straight-line waypoint densification.

Generates equally-spaced waypoints along line segments for precise
path following. Tighter spacing on MARK segments (spray ON) for
drawing accuracy, coarser spacing on TRANSIT for faster travel.
"""

from __future__ import annotations

import math

from ..core import PathSegment, SegmentType


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

    # The epsilon makes densification IDEMPOTENT, which the pipeline relies on:
    # extension run-ups get densified once when they are built and again in the
    # re-densify pass, and TRANSIT connectors can be densified more than once.
    #
    # Without it, re-densifying an already-5cm-spaced line halves every interval.
    # An interval that is *exactly* `spacing` accumulates float error into
    # length = 0.050000000000000003, so ceil(length/spacing) = ceil(1.0000000000000007)
    # = 2 instead of 1, and each 5cm step is split into two 2.5cm steps. Subtracting
    # a relative epsilon before the ceil absorbs that noise without affecting any
    # interval that genuinely exceeds the spacing.
    n_intervals = max(1, int(math.ceil(length / spacing - 1e-9)))
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

    PROVENANCE: the returned segment carries ``metadata["vertex_indices"]`` — the
    indices, into the *densified* point list, of the points that came from the
    input geometry rather than from interpolation. Downstream simplification uses
    this to distinguish surveyed intent from machine-generated fill: an
    interpolated point may be dropped freely, an original vertex may not. Without
    it the two are numerically indistinguishable, which is how near-collinear
    survey vertices were silently deleted (see `_simplify_path_for_profile`).

    Args:
        segment: Input segment with potentially sparse points.
        mark_spacing: Waypoint spacing for MARK segments (metres).
        transit_spacing: Waypoint spacing for TRANSIT segments (metres).

    Returns:
        New PathSegment with densified points, preserving all other attributes.
    """
    if len(segment.points) <= 1:
        # Single point or empty — pass through. Every point is original.
        meta = dict(segment.metadata)
        meta["vertex_indices"] = list(range(len(segment.points)))
        # control_indices already index the (unchanged) point list.
        return PathSegment(
            segment_type=segment.segment_type,
            points=list(segment.points),
            speed=segment.speed,
            segment_id=segment.segment_id,
            source_entity=segment.source_entity,
            metadata=meta,
        )

    spacing = mark_spacing if segment.segment_type == SegmentType.MARK else transit_spacing
    dense_pts: list[tuple[float, float]] = []
    vertex_indices: list[int] = []

    for i in range(len(segment.points) - 1):
        line_pts = densify_line(segment.points[i], segment.points[i + 1], spacing)
        # Avoid duplicating the junction point
        if dense_pts and line_pts:
            # segment.points[i] is already in dense_pts as the previous run's
            # last element, which was recorded as a vertex on that iteration.
            dense_pts.extend(line_pts[1:])
        else:
            vertex_indices.append(0)   # segment.points[0] lands at index 0
            dense_pts.extend(line_pts)
        # segment.points[i + 1] is always the last point just appended.
        vertex_indices.append(len(dense_pts) - 1)

    meta = dict(segment.metadata)
    meta["vertex_indices"] = vertex_indices
    # Declared control points index the ORIGINAL vertex list; remap them onto the
    # densified list so the declaration survives densification. Without this the
    # indices would silently point at interpolated fill.
    # `is not None`, not truthiness: [] is a real declaration ("protect
    # nothing"), distinct from absent — it must survive densification as [].
    ctrl = segment.metadata.get("control_indices")
    if ctrl is not None:
        meta["control_indices"] = [
            vertex_indices[k] for k in ctrl if 0 <= k < len(vertex_indices)
        ]
    return PathSegment(
        segment_type=segment.segment_type,
        points=dense_pts,
        speed=segment.speed,
        segment_id=segment.segment_id,
        source_entity=segment.source_entity,
        metadata=meta,
    )