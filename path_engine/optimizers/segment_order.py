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
    # Composite line-chains carry their constituent edges in `chain_members` so
    # per-edge extensions can be applied downstream. Reversing the chain must
    # reverse the member list and each member's orientation, or the members would
    # no longer match the reversed composite. Members are line-like (no tangents),
    # so the recursive call only flips their point order.
    members = seg.metadata.get("chain_members")
    if members:
        new_meta["chain_members"] = [_reverse_segment(m) for m in reversed(members)]
    return PathSegment(
        segment_type=seg.segment_type,
        points=list(reversed(seg.points)),
        speed=seg.speed,
        segment_id=seg.segment_id,
        source_entity=seg.source_entity,
        metadata=new_meta,
    )


def _simplify(points: list[tuple[float, float]], tol: float = 0.01) -> list[tuple[float, float]]:
    """Drop collinear interior points. A densified straight line collapses to 2 points.

    Crossing tests run inside the 2-opt inner loop, so the obstacle polylines have to be
    cheap. A 2 m mark line arrives here as ~41 densified waypoints and leaves as 2.
    """
    if len(points) < 3:
        return list(points)
    out = [points[0]]
    for i in range(1, len(points) - 1):
        ax, ay = out[-1]
        bx, by = points[i]
        cx, cy = points[i + 1]
        # Perpendicular distance of b from the line a->c.
        ex, ey = cx - ax, cy - ay
        L = math.hypot(ex, ey)
        if L < 1e-9:
            continue
        d = abs((bx - ax) * ey - (by - ay) * ex) / L
        if d > tol:
            out.append(points[i])
    out.append(points[-1])
    return out


def _cross_point(p1, p2, p3, p4) -> tuple[float, float] | None:
    """Where the open segments p1p2 and p3p4 properly cross, or None."""
    d = (p2[0] - p1[0]) * (p4[1] - p3[1]) - (p2[1] - p1[1]) * (p4[0] - p3[0])
    if abs(d) < 1e-12:
        return None
    t = ((p3[0] - p1[0]) * (p4[1] - p3[1]) - (p3[1] - p1[1]) * (p4[0] - p3[0])) / d
    u = ((p3[0] - p1[0]) * (p2[1] - p1[1]) - (p3[1] - p1[1]) * (p2[0] - p1[0])) / d
    if 0.0 < t < 1.0 and 0.0 < u < 1.0:
        return (p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]))
    return None


class _PaintAwareness:
    """Costs a route by how much ALREADY-PAINTED geometry its connectors drive over.

    E3. The connector between two runs is a straight shot, and nothing stopped it
    crossing lines the rover had already finished — the wheels go through wet paint.

    The fix is ordering, not rerouting, and it turns on one observation: **crossing a
    line that has not been painted yet is free.** star_3x3m's crosshair lines sit inside
    the star and the square, so *some* connector must cross that geometry — but if the
    crosshairs are marked FIRST, there is no paint there to cross. Penalising only
    already-laid paint lets the optimizer discover that ordering by itself.

    Connectors are modelled AFT-tip -> next PRE-start (extensions shift both ends
    outward along the mark's own direction), matching what the engine actually emits.
    """

    def __init__(
        self,
        marks: list[PathSegment],
        penalty_m: float,
        pre_m: float,
        aft_m: float,
    ) -> None:
        self.penalty_m = penalty_m
        self.pre_m = pre_m
        self.aft_m = aft_m
        # Stable identity per mark, so the cache survives _reverse_segment() copies.
        self._poly: dict[int, list[tuple[float, float]]] = {}
        for i, s in enumerate(marks):
            s.metadata["_opt_id"] = i
            self._poly[i] = _simplify(s.points)
        self._cache: dict[tuple, frozenset] = {}

    @staticmethod
    def _key(seg: PathSegment) -> tuple[int, bool]:
        return (seg.metadata.get("_opt_id", -1),
                bool(seg.metadata.get("reversed", False)))

    def _exit(self, seg: PathSegment) -> tuple[float, float]:
        """Where the rover actually leaves this run — the AFT tip."""
        if self.aft_m <= 0 or len(seg.points) < 2:
            return seg.points[-1]
        d = _unit(seg.points[-2], seg.points[-1])
        if d is None:
            return seg.points[-1]
        return (seg.points[-1][0] + d[0] * self.aft_m,
                seg.points[-1][1] + d[1] * self.aft_m)

    def _entry(self, seg: PathSegment) -> tuple[float, float]:
        """Where the rover actually joins the next run — the PRE start."""
        if self.pre_m <= 0 or len(seg.points) < 2:
            return seg.points[0]
        d = _unit(seg.points[0], seg.points[1])
        if d is None:
            return seg.points[0]
        return (seg.points[0][0] - d[0] * self.pre_m,
                seg.points[0][1] - d[1] * self.pre_m)

    # A connector springs from one mark's run-out and lands on the next mark's run-up, so
    # it necessarily grazes both near its own endpoints. Ignore crossings within this
    # distance of either end — but NOT the marks themselves: a connector leaving a CLOSED
    # shape (a circle) can genuinely re-cross it further along, and blanket-skipping the
    # source mark hid exactly that on square_circle and sct.
    _ENDPOINT_GRAZE_M = 0.06

    def crossed_by_connector(self, a: PathSegment, b: PathSegment) -> frozenset:
        """Ids of the marks the a->b connector drives over (regardless of paint order)."""
        key = (self._key(a), self._key(b))
        hit = self._cache.get(key)
        if hit is not None:
            return hit

        p1, p2 = self._exit(a), self._entry(b)
        ids = set()
        for mid, poly in self._poly.items():
            for i in range(len(poly) - 1):
                x = _cross_point(p1, p2, poly[i], poly[i + 1])
                if x is None:
                    continue
                if (_distance(x, p1) < self._ENDPOINT_GRAZE_M
                        or _distance(x, p2) < self._ENDPOINT_GRAZE_M):
                    continue  # graze at the run-out tip / run-up start
                ids.add(mid)
                break
        hit = frozenset(ids)
        self._cache[key] = hit
        return hit

    def penalty(self, route: list[PathSegment]) -> float:
        """Metres of equivalent cost for driving over paint already on the ground."""
        if self.penalty_m <= 0.0:
            return 0.0
        painted: set[int] = set()
        total = 0.0
        for a, b in zip(route, route[1:]):
            if not a.points or not b.points:
                continue
            painted.add(self._key(a)[0])
            # Only paint ALREADY laid counts. Crossing a line the rover has yet to
            # mark is free — and that is exactly what lets an enclosed shape be
            # visited first.
            total += self.penalty_m * len(self.crossed_by_connector(a, b) & painted)
        return total


def _unit(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float] | None:
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    return (dx / L, dy / L) if L > 1e-9 else None


def _deadhead_cost(
    route: list[PathSegment],
    start_position: tuple[float, float] | None,
    paint: "_PaintAwareness | None" = None,
) -> float:
    """Transit-only cost for an oriented segment route, plus any wet-paint penalty."""
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

    if paint is not None:
        cost += paint.penalty(route)
    return cost


def _apply_two_opt(
    route: list[PathSegment],
    start_position: tuple[float, float] | None,
    max_passes: int = 20,
    paint: "_PaintAwareness | None" = None,
) -> tuple[list[PathSegment], float, float, int]:
    """Improve an oriented route with 2-opt slice reversals."""
    n = len(route)
    if n < 4:
        cost = _deadhead_cost(route, start_position, paint)
        return route, cost, cost, 0

    best = list(route)
    before = _deadhead_cost(best, start_position, paint)
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
                cand_cost = _deadhead_cost(candidate, start_position, paint)
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
    insert_transits: bool = True,
    avoid_wet_paint: bool = True,
    wet_paint_penalty_m: float = 5.0,
    pre_extension_m: float = 0.0,
    aft_extension_m: float = 0.0,
) -> list[PathSegment]:
    """Reorder MARK segments using nearest-neighbor heuristic with endpoint reversal.

    At each step, considers both endpoints of each unvisited MARK segment.
    If the nearest approach is via the segment's end point, the segment's
    point order is reversed so the rover enters from that end.

    Args:
        segments: Input segments (MARK and TRANSIT).
        start_position: Rover starting (north, east) position. If None,
                        starts from the first segment's start point.
        transit_speed: Speed for inserted TRANSIT segments (m/s).
        insert_transits: Emit TRANSIT links between consecutive MARK segments.

            Set False when the caller is going to wrap each MARK in PRE/AFT
            extensions afterwards. A transit link generated *here* connects the
            ORIGINAL mark endpoints, and once extensions are added it no longer
            reaches them — the rover then has to drive out along the AFT, reverse
            180 deg back over it to the original endpoint to pick up the stale
            transit, cross to the next mark, overshoot it along its PRE, and
            reverse 180 deg again. Two pointless reversals per transition, with
            the run-up/run-out exactly cancelled.

            With this False the caller inserts the connectors *after* extension,
            so travel runs AFT-tip -> next PRE-start directly.

    Returns:
        Reordered MARK segments (point order possibly reversed), with TRANSIT
        links inserted between them iff `insert_transits`.
    """
    mark_segments = [s for s in segments if s.segment_type == SegmentType.MARK]
    if not mark_segments:
        return segments  # No MARK segments — nothing to reorder

    # E3: cost connectors that drive over paint the rover has ALREADY laid, so the
    # optimizer prefers an order that avoids them. Crossing not-yet-marked geometry is
    # free, which is what lets an enclosed shape (star_3x3m's centre crosshairs) be
    # visited before the shape that encloses it.
    paint = (
        _PaintAwareness(mark_segments, wet_paint_penalty_m,
                        pre_extension_m, aft_extension_m)
        if avoid_wet_paint and wet_paint_penalty_m > 0 and len(mark_segments) > 1
        else None
    )

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

        result = (
            _insert_transits([seg], start_position, transit_speed,
                             include_start_transit=True)
            if insert_transits else [seg]
        )
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

    deadhead_before = _deadhead_cost(route, start_position, paint)
    deadhead_after = deadhead_before
    improvements = 0
    two_opt_skipped_reason = None
    if use_two_opt and len(route) <= max_two_opt_segments:
        route, deadhead_before, deadhead_after, improvements = _apply_two_opt(
            route, start_position, paint=paint
        )
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

    if not insert_transits:
        return route
    return _insert_transits(route, start_position, transit_speed, include_start_transit=False)
