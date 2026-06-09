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


def _reverse_segment(seg: PathSegment) -> PathSegment:
    """Return a copy of seg with traversal direction reversed."""
    new_meta = dict(seg.metadata)
    if "start_tangent" in seg.metadata and "end_tangent" in seg.metadata:
        st = seg.metadata["start_tangent"]
        et = seg.metadata["end_tangent"]
        new_meta["start_tangent"] = (-et[0], -et[1])
        new_meta["end_tangent"] = (-st[0], -st[1])
    new_meta["reversed"] = not bool(seg.metadata.get("reversed", False))
    return PathSegment(
        segment_type=seg.segment_type,
        points=list(reversed(seg.points)),
        speed=seg.speed,
        segment_id=seg.segment_id,
        source_entity=seg.source_entity,
        metadata=new_meta,
    )


def _deadhead_cost(route: list[PathSegment], start_position: tuple[float, float] | None) -> float:
    """Transit-only cost for an oriented segment route."""
    if not route:
        return 0.0

    cost = 0.0
    current = start_position
    for seg in route:
        if not seg.points:
            continue
        if current is not None:
            cost += _distance(current, seg.points[0])
        current = seg.points[-1]
    return cost


def _apply_two_opt(
    route: list[PathSegment],
    start_position: tuple[float, float] | None,
    max_passes: int = 20,
) -> tuple[list[PathSegment], float, float, int]:
    """Improve an oriented route with 2-opt slice reversals."""
    n = len(route)
    if n < 4:
        cost = _deadhead_cost(route, start_position)
        return route, cost, cost, 0

    best = list(route)
    before = _deadhead_cost(best, start_position)
    best_cost = before
    improvements = 0

    for _ in range(max_passes):
        changed = False
        for i in range(0, n - 2):
            for k in range(i + 1, n):
                candidate = (
                    best[:i]
                    + [_reverse_segment(seg) for seg in reversed(best[i:k + 1])]
                    + best[k + 1:]
                )
                cand_cost = _deadhead_cost(candidate, start_position)
                if cand_cost + 1e-9 < best_cost:
                    best = candidate
                    best_cost = cand_cost
                    improvements += 1
                    changed = True
        if not changed:
            break

    return best, before, best_cost, improvements


def _insert_transits(
    route: list[PathSegment],
    start_position: tuple[float, float] | None,
    transit_speed: float,
    include_start_transit: bool,
) -> list[PathSegment]:
    """Insert TRANSIT links between an oriented MARK route."""
    ordered: list[PathSegment] = []
    current_pos = start_position
    transit_count = 0

    for seg in route:
        if current_pos is not None and seg.points and (ordered or include_start_transit):
            d = _distance(current_pos, seg.points[0])
            if d > 0.01:
                transit_count += 1
                source = "transit:start" if not ordered else f"transit:{transit_count}"
                ordered.append(PathSegment(
                    segment_type=SegmentType.TRANSIT,
                    points=[current_pos, seg.points[0]],
                    speed=transit_speed,
                    source_entity=source,
                ))
        ordered.append(seg)
        if seg.points:
            current_pos = seg.points[-1]

    return ordered


def optimize_segment_order(
    segments: list[PathSegment],
    start_position: tuple[float, float] | None = None,
    transit_speed: float = 0.50,
    use_two_opt: bool = True,
    max_two_opt_segments: int = 80,
    stats: dict | None = None,
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
        # Single segment: check whether entering from the end is closer.
        # Mirrors the multi-segment nearest-neighbour reversal logic so that
        # a lone ARC/CIRCLE can also be entered backward when appropriate.
        seg = mark_segments[0]
        should_reverse = False
        if start_position is not None and seg.points and len(seg.points) > 1:
            d_start = _distance(start_position, seg.points[0])
            d_end   = _distance(start_position, seg.points[-1])
            should_reverse = d_end < d_start

        if should_reverse:
            seg = _reverse_segment(seg)

        result = _insert_transits([seg], start_position, transit_speed, include_start_transit=True)
        if stats is not None:
            cost = _deadhead_cost([seg], start_position)
            stats.update({
                "method": "nearest_neighbor",
                "mark_segments": 1,
                "deadhead_before_2opt_m": cost,
                "deadhead_after_2opt_m": cost,
                "two_opt_improvements": 0,
                "two_opt_skipped_reason": "single mark segment",
                "max_two_opt_segments": max_two_opt_segments,
            })
        return result

    # Nearest-neighbor heuristic with endpoint reversal
    remaining: list[tuple[int, PathSegment]] = [(i, s) for i, s in enumerate(mark_segments)]
    route: list[PathSegment] = []

    # Start from start_position or first segment
    if start_position is not None:
        current_pos = start_position
    else:
        first = mark_segments[0]
        current_pos = first.points[0] if first.points else (0.0, 0.0)

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

        if best_reverse and len(seg.points) > 1:
            seg = _reverse_segment(seg)

        route.append(seg)
        if seg.points:
            current_pos = seg.points[-1]

    deadhead_before = _deadhead_cost(route, start_position)
    deadhead_after = deadhead_before
    improvements = 0
    two_opt_skipped_reason = None
    if use_two_opt and len(route) <= max_two_opt_segments:
        route, deadhead_before, deadhead_after, improvements = _apply_two_opt(route, start_position)
    elif use_two_opt:
        two_opt_skipped_reason = (
            f"mark segment count {len(route)} exceeds cap {max_two_opt_segments}"
        )

    if stats is not None:
        stats.update({
            "method": "nearest_neighbor_2opt" if use_two_opt and two_opt_skipped_reason is None else "nearest_neighbor",
            "mark_segments": len(route),
            "deadhead_before_2opt_m": deadhead_before,
            "deadhead_after_2opt_m": deadhead_after,
            "two_opt_improvements": improvements,
            "two_opt_skipped_reason": two_opt_skipped_reason,
            "max_two_opt_segments": max_two_opt_segments,
        })

    return _insert_transits(route, start_position, transit_speed, include_start_transit=False)
