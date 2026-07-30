"""Blend single-vertex tangent kinks into G1 biarcs, bounded by a deviation cap.

Why this exists (2026-07-30, mission ``curve_6_points-1_x3``): the app's sparse
arc fit joins per-span fitted arcs AT the surveyed stakes with no tangent
constraint, so the staged polyline carries G0 joints — measured −9.98°, +4.04°
and −9.99° of tangent jump in a single vertex, each landing exactly on a stake.
A tangent step demands infinite curvature; at 0.35 m/s the controller saturated
at κ = −3.7 m⁻¹ against a 0.384 rad/s firmware yaw-rate limit, ran 6.4 cm wide,
and the spray xtrack gate cut a 31 cm hole in the mark. Reproduced in
simulation; blending the joint removes the excursion without slowing down.

Why not ``corner_fillet``: that pass requires a genuinely STRAIGHT 4 m baseline
on both sides of the corner (by design — its directions are meaningless on a
curve). These kinks sit BETWEEN arcs, so it refuses them all. This pass instead
cuts a short window around the kink, measures the LOCAL tangents at the cut
points, and joins them with a biarc that is G1 at both cuts and G1 at its own
join. The window grows only as far as every replaced original point (the stake
included) stays within ``max_dev_m`` of the new geometry — so the fit can never
legally move the line more than the cap, unlike ``fit_arcs_max_dev_m``'s 15 cm.

Scope guard: a vertex qualifies by its turn's EXCESS over the median turn of
its neighbours (see MIN/MAX_KINK_EXCESS_DEG) — so a coarsely sampled smooth arc,
where every vertex turns alike, is never touched, while an isolated joint or a
reverse jog is. Below the floor is survey noise; above the ceiling is a genuine
corner that the segment tracker should stop-and-pivot through, not drive.

The output is ``(points, control_indices, report)``. Control points inside a
blended window are snapped to the nearest emitted sample — which the deviation
check has already bounded to ``max_dev_m`` of their surveyed position.
"""

from __future__ import annotations

import logging
import math

log = logging.getLogger("path_engine.kink_blend")

Point = tuple[float, float]  # (north, east)

# A kink is an ISOLATED tangent jump, not a large turn per se: a smoothly
# sampled arc turns at every vertex by the same amount and must not be touched
# (its per-vertex turn can exceed any absolute threshold at coarse sampling),
# while the field kinks are 10° jumps amid uniform ±2.7° neighbours — and the
# stake-10 defect is a +4° REVERSE jog amid −2.7° turns, small in absolute
# terms but a 6.7° discontinuity. So the detector measures the EXCESS of a
# vertex's signed turn over the median signed turn of its neighbours. Below
# the floor is noise; above the ceiling is a real corner — pivot logic owns it.
MIN_KINK_EXCESS_DEG = 4.0
MAX_KINK_EXCESS_DEG = 30.0
# Hard cap on the half-window either side of a kink. 0.40 m spreads a 10° jump
# over ≤0.8 m of path: κ ≈ 0.22 m⁻¹, yaw rate at 0.35 m/s ≈ 0.08 rad/s — far
# inside the 0.384 rad/s firmware limit, with no speed reduction needed.
MAX_HALF_WINDOW_M = 0.40
# Smallest usable half-window. Below this the biarc is shorter than the
# controller's own lookahead resolution and fixes nothing.
MIN_HALF_WINDOW_M = 0.05
# Sample spacing along emitted biarcs (densify tightens it further downstream).
ARC_SPACING_M = 0.05


def _unit(dn: float, de: float) -> tuple[float, float] | None:
    m = math.hypot(dn, de)
    if m < 1e-12:
        return None
    return (dn / m, de / m)


def _signed_turn(a: Point, b: Point, c: Point) -> float:
    v1n, v1e = b[0] - a[0], b[1] - a[1]
    v2n, v2e = c[0] - b[0], c[1] - b[1]
    if math.hypot(v1n, v1e) < 1e-12 or math.hypot(v2n, v2e) < 1e-12:
        return 0.0
    return math.atan2(v1n * v2e - v1e * v2n, v1n * v2n + v1e * v2e)


def _cumulative(points: list[Point]) -> list[float]:
    out = [0.0]
    for i in range(len(points) - 1):
        out.append(out[-1] + math.dist(points[i], points[i + 1]))
    return out


def _tangent_at(points: list[Point], i: int) -> tuple[float, float] | None:
    """Local tangent at vertex ``i`` by central difference (one-sided at ends)."""
    lo = max(0, i - 1)
    hi = min(len(points) - 1, i + 1)
    return _unit(points[hi][0] - points[lo][0], points[hi][1] - points[lo][1])


def _arc_points(p: Point, t: tuple[float, float], q: Point,
                spacing: float) -> list[Point] | None:
    """Sample the circular arc that starts at ``p`` tangent to ``t`` and ends at
    ``q``. Returns points from just after ``p`` through ``q`` inclusive, or a
    straight interpolation when the three constraints are collinear."""
    cn, ce = q[0] - p[0], q[1] - p[1]
    chord = math.hypot(cn, ce)
    if chord < 1e-9:
        return []
    # Perpendicular offset of q from the tangent line at p.
    cross = t[0] * ce - t[1] * cn
    if abs(cross) < 1e-6 * chord:
        n_steps = max(1, int(math.ceil(chord / spacing)))
        return [(p[0] + cn * k / n_steps, p[1] + ce * k / n_steps)
                for k in range(1, n_steps + 1)]
    radius = chord * chord / (2.0 * abs(cross))
    # Centre sits on the left normal of t when the arc bends left (cross > 0).
    if cross > 0:
        nrm = (-t[1], t[0])
    else:
        nrm = (t[1], -t[0])
    centre = (p[0] + radius * nrm[0], p[1] + radius * nrm[1])
    a0 = math.atan2(p[1] - centre[1], p[0] - centre[0])
    a1 = math.atan2(q[1] - centre[1], q[0] - centre[0])
    sweep = a1 - a0
    # The tangent direction fixes the sweep sense: left-bending arcs sweep CCW.
    if cross > 0:
        while sweep <= 0:
            sweep += 2 * math.pi
    else:
        while sweep >= 0:
            sweep -= 2 * math.pi
    arc_len = abs(sweep) * radius
    n_steps = max(2, int(math.ceil(arc_len / spacing)))
    out = []
    for k in range(1, n_steps + 1):
        a = a0 + sweep * k / n_steps
        out.append((centre[0] + radius * math.cos(a),
                    centre[1] + radius * math.sin(a)))
    out[-1] = q
    return out


def _biarc(p0: Point, t0: tuple[float, float], p1: Point,
           t1: tuple[float, float], spacing: float) -> list[Point] | None:
    """G1 biarc from (p0, t0) to (p1, t1): equal-parameter join (Juckett form).

    Returns samples from just after ``p0`` through ``p1``, or None when the
    construction is degenerate (anti-parallel tangents, zero chord).
    """
    vn, ve = p1[0] - p0[0], p1[1] - p0[1]
    vv = vn * vn + ve * ve
    if vv < 1e-12:
        return None
    un, ue = t0[0] + t1[0], t0[1] + t1[1]
    vu = vn * un + ve * ue
    denom = 2.0 * (1.0 - (t0[0] * t1[0] + t0[1] * t1[1]))
    if denom < 1e-9:
        # Tangents (near-)parallel: single arc from (p0, t0) to p1 suffices.
        return _arc_points(p0, t0, p1, spacing)
    disc = vu * vu + denom * vv
    if disc < 0.0:
        return None
    d = (-vu + math.sqrt(disc)) / denom
    if not math.isfinite(d) or d <= 0.0:
        return None
    join = ((p0[0] + d * t0[0] + p1[0] - d * t1[0]) / 2.0,
            (p0[1] + d * t0[1] + p1[1] - d * t1[1]) / 2.0)
    first = _arc_points(p0, t0, join, spacing)
    if first is None:
        return None
    # Tangent at the join = direction the second arc must LEAVE with, which by
    # biarc symmetry is the reflection carrying the join toward p1 at t1: the
    # second arc is built backwards from (p1, -t1) to the join, then reversed.
    second_rev = _arc_points(p1, (-t1[0], -t1[1]), join, spacing)
    if second_rev is None:
        return None
    second = list(reversed(second_rev))
    if second and first and math.dist(first[-1], second[0]) < 1e-9:
        second = second[1:]
    # second currently ends just before p1 (it started there); ensure p1 last.
    if not second or math.dist(second[-1], p1) > 1e-9:
        second.append(p1)
    return first + second


def _max_deviation(replaced: list[Point], blended: list[Point]) -> float:
    """Max distance from each replaced original point to the blended polyline."""
    worst = 0.0
    for p in replaced:
        best = float("inf")
        for j in range(len(blended) - 1):
            a, b = blended[j], blended[j + 1]
            dn, de = b[0] - a[0], b[1] - a[1]
            s2 = dn * dn + de * de
            if s2 < 1e-18:
                continue
            t = max(0.0, min(1.0, ((p[0] - a[0]) * dn + (p[1] - a[1]) * de) / s2))
            q = (a[0] + t * dn, a[1] + t * de)
            best = min(best, math.hypot(p[0] - q[0], p[1] - q[1]))
        worst = max(worst, best)
    return worst


def _kink_excess_rad(turns: list[float], i: int) -> float:
    """|signed turn at i − median signed turn of its ±3 neighbours|.

    Zero on a uniformly sampled arc (everything turns alike), the full jump on
    an isolated joint, and the sign-flip magnitude on a reverse jog.
    """
    n = len(turns)
    neigh = [turns[j] for j in range(max(1, i - 3), min(n - 1, i + 4)) if j != i]
    if not neigh:
        return abs(turns[i])
    med = sorted(neigh)[len(neigh) // 2]
    return abs(turns[i] - med)


def blend_kinks(
    points: list[Point],
    control_indices: list[int] | None = None,
    max_dev_m: float = 0.01,
    min_kink_deg: float = MIN_KINK_EXCESS_DEG,
    max_kink_deg: float = MAX_KINK_EXCESS_DEG,
    max_half_window_m: float = MAX_HALF_WINDOW_M,
    spacing_m: float = ARC_SPACING_M,
) -> tuple[list[Point], list[int], list[dict]]:
    """Blend every qualifying tangent kink in ``points`` into a G1 biarc.

    Grows each kink's window as wide as the deviation cap allows (largest
    window whose biarc keeps every replaced original point within
    ``max_dev_m``), so the heading change is spread over the most path the cap
    permits — feasibility without slowing down. Kinks whose smallest window
    still violates the cap are left untouched and reported as skipped.

    Returns ``(points, control_indices, report)``; a report entry per kink with
    ``blended`` True/False, the turn angle, window and achieved deviation.
    """
    n = len(points)
    keep = sorted(set(control_indices)) if control_indices is not None else list(range(n))
    if n < 3 or max_dev_m <= 0.0:
        return list(points), list(keep), []

    arc = _cumulative(points)
    turns = [0.0] * n
    for i in range(1, n - 1):
        turns[i] = _signed_turn(points[i - 1], points[i], points[i + 1])
    excess = {i: _kink_excess_rad(turns, i) for i in range(1, n - 1)}
    kinks = [i for i in range(1, n - 1)
             if math.radians(min_kink_deg) <= excess[i] <= math.radians(max_kink_deg)]
    if not kinks:
        return list(points), list(keep), []

    report: list[dict] = []
    plans: list[tuple[int, int, int, list[Point], float, float]] = []
    prev_hi = 0   # emission-order guard: windows must be index-disjoint
    for ki, k in enumerate(kinks):
        # Half-window budget: hard cap, clipped so adjacent kink windows and
        # the segment ends are never consumed.
        budget = max_half_window_m
        if ki > 0:
            budget = min(budget, (arc[k] - arc[kinks[ki - 1]]) / 2.0 - 0.01)
        if ki + 1 < len(kinks):
            budget = min(budget, (arc[kinks[ki + 1]] - arc[k]) / 2.0 - 0.01)
        budget = min(budget, arc[k] - arc[0] - 1e-6, arc[-1] - arc[k] - 1e-6)
        entry = {"vertex": k, "turn_deg": round(math.degrees(turns[k]), 2),
                 "excess_deg": round(math.degrees(excess[k]), 2),
                 "blended": False}
        if budget < MIN_HALF_WINDOW_M:
            entry["skip"] = "no room (adjacent kink or segment end)"
            report.append(entry)
            continue
        done = False
        for frac in (1.0, 0.75, 0.55, 0.4, 0.28, 0.2):
            half = budget * frac
            if half < MIN_HALF_WINDOW_M:
                break
            lo = k
            while lo > 0 and arc[k] - arc[lo] < half:
                lo -= 1
            hi = k
            while hi < n - 1 and arc[hi] - arc[k] < half:
                hi += 1
            if hi - lo < 2 or lo >= k or hi <= k:
                continue
            # The while-loops overshoot by up to one sample; never let this
            # window reach back into the previous kink's already-planned one.
            if lo < prev_hi:
                continue
            t0 = _tangent_at(points, lo)
            t1 = _tangent_at(points, hi)
            if t0 is None or t1 is None:
                continue
            blended = _biarc(points[lo], t0, points[hi], t1, spacing_m)
            if not blended or len(blended) < 2:
                continue
            dev = _max_deviation(points[lo + 1:hi], [points[lo]] + blended)
            if dev <= max_dev_m:
                plans.append((k, lo, hi, blended, dev, half))
                entry.update({"blended": True, "half_window_m": round(half, 3),
                              "max_dev_m": round(dev, 4)})
                prev_hi = hi
                done = True
                break
        if not done and "skip" not in entry:
            entry["skip"] = f"deviation cap {max_dev_m:.3f} m unmet at every window"
        report.append(entry)

    if not plans:
        return list(points), list(keep), report

    out: list[Point] = []
    control: list[int] = []
    keep_set = set(keep)
    cursor = 0
    for k, lo, hi, blended, dev, half in plans:
        while cursor <= lo:
            if cursor in keep_set:
                control.append(len(out))
            out.append(points[cursor])
            cursor += 1
        base = len(out)
        out.extend(blended)
        # Controls inside the window (the stake apex included) snap to the
        # nearest emitted sample — bounded by the deviation check above.
        for c in range(lo + 1, hi):
            if c in keep_set:
                p = points[c]
                j = min(range(base, len(out)), key=lambda m: math.dist(out[m], p))
                control.append(j)
        cursor = hi + 1
        # The window's end point points[hi] is blended[-1] == out[-1].
        if hi in keep_set:
            control.append(len(out) - 1)
    while cursor < n:
        if cursor in keep_set:
            control.append(len(out))
        out.append(points[cursor])
        cursor += 1

    n_blend = sum(1 for r in report if r["blended"])
    log.info("kink_blend: %d/%d kink(s) blended (cap %.1f cm), %d -> %d points",
             n_blend, len(report), 100 * max_dev_m, n, len(out))
    return out, sorted(set(control)), report
