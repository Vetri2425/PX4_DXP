"""Round a surveyed corner into a drivable arc of an operator-chosen radius.

This is deliberately NOT ``arc_chain``. That module RECOVERS an arc the surveyor
actually measured — it only ever fits a circle the data already determines. This
module INVENTS geometry: it replaces a surveyed corner with a fillet whose radius
comes from the marking spec, not from the survey. Keep the two separate so it is
always clear which parts of a path are surveyed truth and which are design.

Why it is needed: a road survey routinely captures a bend as two straights
meeting at one vertex. Of the 26 ``curve``-labelled zones in the Chennai roads
CSV, 22 contain exactly ONE direction change and none contain the three a circle
needs — so there is no arc to recover, only a corner to round.

Why it cannot reuse ``corner_smooth_radius_m``: that stage runs on the densified
polyline, where every segment is one spacing step (5 cm) long. It needs the
adjacent segments to be at least as long as the radius, so on a densified survey
it skips essentially every vertex ("adjacent segments are too short for radius
10.000m", 239-713 vertices skipped). This module works from the corner's
straights instead of its immediate neighbours, so densification is irrelevant.
"""

from __future__ import annotations

import logging
import math

from .arc_curve import arc_waypoints

log = logging.getLogger("path_engine.corner_fillet")

Point = tuple[float, float]  # (north, east)

# A vertex must turn at least this much to be worth rounding. Below it the
# corner is within survey noise and a fillet would be indistinguishable from the
# straight it replaces.
MIN_CORNER_DEG = 4.0
# Arc length used to measure the direction of the straight either side of a
# corner. Long enough to average out per-point noise (1 cm over 0.5 m is 1.1 deg
# on a single step), short enough to stay on one straight.
DIRECTION_BASELINE_M = 4.0
# How far the straight either side of a corner may bow and still count as
# straight. Comfortably above 7-decimal coordinate rounding (1.1 cm) and
# survey noise, well below any real curve.
STRAIGHT_TOL_M = 0.05


def _signed_turn(a: Point, b: Point, c: Point) -> float:
    """Signed heading change at ``b`` (radians). Positive = left/CCW."""
    v1n, v1e = b[0] - a[0], b[1] - a[1]
    v2n, v2e = c[0] - b[0], c[1] - b[1]
    if math.hypot(v1n, v1e) < 1e-12 or math.hypot(v2n, v2e) < 1e-12:
        return 0.0
    return math.atan2(v1e * v2n - v1n * v2e, v1n * v2n + v1e * v2e)


def _cumulative(points: list[Point]) -> list[float]:
    out = [0.0]
    for i in range(len(points) - 1):
        out.append(out[-1] + math.dist(points[i], points[i + 1]))
    return out


def _corner_groups(points: list[Point], min_turn_rad: float) -> list[tuple[int, int, float]]:
    """Group consecutive turning vertices into ``(first, last, net_turn)`` corners.

    A survey corner is often spread over two or three vertices, and 7-decimal
    coordinate rounding puts a dither of alternating +-1 deg on every vertex in
    between. Grouping by proximity and summing the SIGNED turn recovers the one
    real direction change; summing absolute turns would count the dither.
    """
    n = len(points)
    turns = [0.0] * n
    for i in range(1, n - 1):
        turns[i] = _signed_turn(points[i - 1], points[i], points[i + 1])

    hits = [i for i in range(1, n - 1) if abs(turns[i]) >= min_turn_rad]
    groups: list[list[int]] = []
    for i in hits:
        if groups and i - groups[-1][-1] <= 3:
            groups[-1].append(i)
        else:
            groups.append([i])

    out = []
    for g in groups:
        lo, hi = g[0], g[-1]
        net = sum(turns[lo:hi + 1])
        if abs(net) >= min_turn_rad:
            out.append((lo, hi, net))
    return out


def _straight_dir(points: list[Point], lo: int, hi: int,
                  flat_tol: float) -> tuple[float, float] | None:
    """Unit direction of ``points[lo:hi+1]``, but ONLY if that span is straight.

    The straightness check is what keeps the fillet honest. This stage runs after
    the arc fit, so a "corner" may really be the joint between a recovered arc
    and its neighbour — and a direction measured over 4 m of a curve is
    meaningless, which turns the fillet into a 120 deg spike. Refusing to fillet
    unless BOTH sides are genuinely straight also handles two corners closer
    together than the baseline, where the direction would be read across the
    wrong one.
    """
    if hi - lo < 1:
        return None
    dn, de = points[hi][0] - points[lo][0], points[hi][1] - points[lo][1]
    m = math.hypot(dn, de)
    if m < 1e-9:
        return None
    for k in range(lo + 1, hi):
        cross = abs((points[k][0] - points[lo][0]) * de
                    - (points[k][1] - points[lo][1]) * dn) / m
        if cross > flat_tol:
            return None
    return (dn / m, de / m)


def _direction_before(points: list[Point], arc: list[float], idx: int,
                      baseline: float, flat_tol: float) -> tuple[float, float] | None:
    """Unit direction of travel arriving at ``idx`` over a straight ``baseline``."""
    target = arc[idx] - baseline
    j = idx
    while j > 0 and arc[j] > target:
        j -= 1
    return _straight_dir(points, j, idx, flat_tol)


def _direction_after(points: list[Point], arc: list[float], idx: int,
                     baseline: float, flat_tol: float) -> tuple[float, float] | None:
    """Unit direction of travel leaving ``idx`` over a straight ``baseline``."""
    target = arc[idx] + baseline
    j = idx
    n = len(points)
    while j < n - 1 and arc[j] < target:
        j += 1
    return _straight_dir(points, idx, j, flat_tol)


def _intersect(p: Point, d1: tuple[float, float],
               q: Point, d2: tuple[float, float]) -> Point | None:
    """Intersection of the lines p + t*d1 and q + u*d2 (the true corner apex)."""
    det = d1[1] * d2[0] - d1[0] * d2[1]
    if abs(det) < 1e-9:
        return None
    t = ((q[1] - p[1]) * d2[0] - (q[0] - p[0]) * d2[1]) / det
    return (p[0] + t * d1[0], p[1] + t * d1[1])


def fillet_corners(
    points: list[Point],
    control_indices: list[int] | None = None,
    radius_m: float = 0.0,
    min_corner_deg: float = MIN_CORNER_DEG,
    straight_tol_m: float = STRAIGHT_TOL_M,
    chord_error_m: float = 0.005,
    min_spacing_m: float = 0.02,
    max_spacing_m: float = 0.10,
) -> tuple[list[Point], list[int]]:
    """Replace each corner in ``points`` with a tangent arc of ``radius_m``.

    The arc is tangent to both straights, so the path stays G1-continuous and the
    rover never has to pivot. Where two corners are too close to fit two full
    fillets, the radius is reduced for that corner rather than skipping it, and
    the reduction is logged.

    ``control_indices`` (surveyed-vertex provenance / must-hit) are carried
    across: an index inside a filleted span is dropped, because that surveyed
    point is no longer on the path — rounding a corner necessarily cuts it. The
    arc's own two tangent points become control points in its place.

    Returns ``(points, control_indices)``. A radius of 0 is a no-op.
    """
    n = len(points)
    if radius_m <= 0.0 or n < 3:
        return list(points), list(control_indices if control_indices is not None
                                  else range(n))

    keep = set(control_indices) if control_indices is not None else set(range(n))
    arc = _cumulative(points)
    corners = _corner_groups(points, math.radians(min_corner_deg))
    if not corners:
        return list(points), sorted(keep)

    # Plan every fillet first so neighbours can be checked for overlap.
    plans = []
    for lo, hi, net in corners:
        d_in = _direction_before(points, arc, lo, DIRECTION_BASELINE_M, straight_tol_m)
        d_out = _direction_after(points, arc, hi, DIRECTION_BASELINE_M, straight_tol_m)
        if d_in is None or d_out is None:
            continue
        theta = abs(math.atan2(d_in[1] * d_out[0] - d_in[0] * d_out[1],
                               d_in[0] * d_out[0] + d_in[1] * d_out[1]))
        if theta < math.radians(min_corner_deg) or theta > math.radians(175.0):
            continue
        apex = _intersect(points[lo], d_in, points[hi], d_out)
        if apex is None:
            continue
        plans.append({"lo": lo, "hi": hi, "apex": apex, "d_in": d_in,
                      "d_out": d_out, "theta": theta,
                      "ccw": net > 0, "s": arc[lo], "e": arc[hi]})

    if not plans:
        return list(points), sorted(keep)

    # Shrink any radius whose tangent would run into its neighbour's. Two
    # adjacent corners SHARE the straight between them, so each may use at most
    # half of it — giving each the whole gap (the obvious-looking bound) makes
    # the two fillets overlap and doubles the path back on itself.
    reduced = 0
    for i, pl in enumerate(plans):
        mid = (pl["s"] + pl["e"]) / 2.0
        if i > 0:
            prev_mid = (plans[i - 1]["s"] + plans[i - 1]["e"]) / 2.0
            avail_before = (mid - prev_mid) / 2.0
        else:
            avail_before = pl["s"]
        if i + 1 < len(plans):
            next_mid = (plans[i + 1]["s"] + plans[i + 1]["e"]) / 2.0
            avail_after = (next_mid - mid) / 2.0
        else:
            avail_after = arc[-1] - pl["e"]
        room = max(0.0, min(avail_before, avail_after))
        half = math.tan(pl["theta"] / 2.0)
        want = radius_m * half
        if want > room:
            pl["radius"] = max(0.0, room / half) if half > 1e-9 else 0.0
            reduced += 1
        else:
            pl["radius"] = radius_m
        pl["tangent"] = pl["radius"] * half

    out: list[Point] = []
    control: list[int] = []
    cursor = 0                    # next original index not yet emitted
    for pl in plans:
        if pl["radius"] < 1e-6 or pl["tangent"] < 1e-6:
            continue
        apex, d_in, d_out, T = pl["apex"], pl["d_in"], pl["d_out"], pl["tangent"]
        start = (apex[0] - T * d_in[0], apex[1] - T * d_in[1])
        end = (apex[0] + T * d_out[0], apex[1] + T * d_out[1])

        # Centre sits on the turn side of the incoming straight. Rotating
        # (north, east) by +90 deg CCW gives (east, -north); a left turn puts the
        # centre on that side, a right turn on the opposite one.
        sign = 1.0 if pl["ccw"] else -1.0
        nrm = (sign * d_in[1], -sign * d_in[0])
        centre = (start[0] + pl["radius"] * nrm[0], start[1] + pl["radius"] * nrm[1])

        # Emit the untouched original points up to the fillet's start.
        s_arc = arc[pl["lo"]] - T
        while cursor < n and arc[cursor] < s_arc:
            if cursor in keep:
                control.append(len(out))
            out.append(points[cursor])
            cursor += 1

        a0 = math.degrees(math.atan2(start[0] - centre[0], start[1] - centre[1]))
        a1 = math.degrees(math.atan2(end[0] - centre[0], end[1] - centre[1]))
        pts = arc_waypoints(centre, pl["radius"], a0, a1,
                            chord_error=chord_error_m, min_spacing=min_spacing_m,
                            max_spacing=max_spacing_m,
                            direction="CCW" if pl["ccw"] else "CW")
        if len(pts) < 2:
            continue
        pts[0], pts[-1] = start, end
        if out and math.dist(out[-1], pts[0]) < 1e-9:
            pts = pts[1:]
        control.append(len(out))                  # entry tangent point
        out.extend(pts)
        control.append(len(out) - 1)              # exit tangent point

        # Skip every original point the fillet replaced.
        e_arc = arc[pl["hi"]] + T
        while cursor < n and arc[cursor] <= e_arc:
            cursor += 1

    while cursor < n:
        if cursor in keep:
            control.append(len(out))
        out.append(points[cursor])
        cursor += 1

    log.info("corner_fillet: rounded %d corner(s) at r=%.2f m (%d radius-limited) "
             "-> %d points", len(plans), radius_m, reduced, len(out))
    return out, sorted(set(control))
