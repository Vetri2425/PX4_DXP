"""Fit a surveyed LINE_CHAIN into straight runs + circular arcs.

A field-surveyed chain is a sequence of noisy point samples. The default
pipeline densifies it into straight chords (``straight_line.densify_segment``),
which turns a genuine surveyed curve into a polygon. This module, used only when
``PathEngine(fit_arcs=True)``, does the minimal thing that fixes that:

  1. Split the chain at genuine corners — vertices where the heading changes by
     more than ``corner_angle_deg``. A square's 90 deg corners split cleanly; an
     arc's small per-chord turns do not, so squares stay square and arcs stay
     whole. Turn-angle (not Douglas-Peucker) is what the operator chose.
  2. Segment each corner-free run by CURVATURE into alternating straight and arc
     sub-runs (``_segment_by_curvature``). This is what makes the fitter work on
     real survey lines: a road is one corner-free run of straights and bends, and
     forcing ONE circle through the whole thing fits nothing (a 500 m road
     "fits" a 1.1 km circle with 29 m of residual and is rejected, so the curves
     silently stay chords). Curvature segmentation cuts it into the runs that
     actually are arcs before any circle is fit.
  3. Per sub-run: a straight one keeps its surveyed points verbatim; an arc one
     is fit to ONE circle by Kasa least-squares (inline, no new dependency) and
     re-emitted through the EXISTING curvature-adaptive tessellator
     ``arc_curve.arc_waypoints``. No new densification is written.

The output is a plain ``(points, control_indices)`` pair, so everything
downstream (densify, grouping, merge, must-hit) is unchanged. ``control_indices``
index the returned point list: every surveyed corner is a control (must-hit)
point, straight-run surveyed vertices stay control points (preserving the
point-line must-hit behaviour), and interpolated arc fill is not.

A rigid transform (the GPS alignment) preserves circles and turn angles, so it
does not matter whether this runs before or after alignment.
"""

from __future__ import annotations

import logging
import math

from .arc_curve import arc_waypoints

log = logging.getLogger("path_engine.arc_chain")

Point = tuple[float, float]  # (north, east)

# Curvature segmentation defaults. A run whose fitted radius exceeds
# MAX_ARC_RADIUS_M is a straight for marking purposes (a 1 km "arc" over a 30 m
# run deviates <0.2 mm from its chord), and a curvature stencil shorter than a
# few metres just measures survey noise — see PRELINE_ROBUST_METHODS.md §2.2.
MAX_ARC_RADIUS_M = 300.0
CURVATURE_STENCIL_M = 5.0
MIN_SUBRUN_M = 3.0
# Max distance a fitted arc may sit from the surveyed points it replaces. The
# binding case is a survey that samples a curve as a coarse polygon: the Egmore
# roundabout CSV is a ~20-gon whose 3.5 m facets sit 13 cm inside the true
# 11.5 m circle, so anything tighter refuses to recover the circle at all.
MAX_ARC_DEVIATION_M = 0.15
# Arc length over which a joint correction is blended back into a run, so the
# straight/arc transition carries no single deflected step.
JOINT_TAPER_M = 1.0


def _turn_angle(a: Point, b: Point, c: Point) -> float:
    """Heading change (radians, 0..pi) at ``b`` going a -> b -> c."""
    v1n, v1e = b[0] - a[0], b[1] - a[1]
    v2n, v2e = c[0] - b[0], c[1] - b[1]
    m1 = math.hypot(v1n, v1e)
    m2 = math.hypot(v2n, v2e)
    if m1 < 1e-9 or m2 < 1e-9:
        return 0.0
    dot = (v1n * v2n + v1e * v2e) / (m1 * m2)
    return math.acos(max(-1.0, min(1.0, dot)))


def _split_at_corners(points: list[Point], corner_angle_rad: float) -> list[tuple[int, int]]:
    """Index ranges [start, end] (inclusive) of runs between corners.

    Adjacent runs share the corner vertex (it is the end of one and the start of
    the next), so the chain stays connected.
    """
    n = len(points)
    if n < 3:
        return [(0, n - 1)] if n >= 2 else []
    corners = [
        i for i in range(1, n - 1)
        if _turn_angle(points[i - 1], points[i], points[i + 1]) > corner_angle_rad
    ]
    bounds = [0, *corners, n - 1]
    return [(bounds[k], bounds[k + 1]) for k in range(len(bounds) - 1)]


def _menger_curvature(a: Point, b: Point, c: Point) -> float:
    """Signed curvature (1/m) of the circle through three points.

    ``2 * signed_area / (|ab| |bc| |ca|)``. Positive = left turn (CCW). Zero when
    the points are collinear or coincident.
    """
    ax, ay = a[1], a[0]
    bx, by = b[1], b[0]
    cx, cy = c[1], c[0]
    twice_area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
    denom = math.dist(a, b) * math.dist(b, c) * math.dist(c, a)
    if denom < 1e-12:
        return 0.0
    return 2.0 * twice_area / denom


def _curvature_profile(run: list[Point], stencil_m: float) -> list[float]:
    """Signed curvature at each point, measured over a wide stencil.

    Adjacent-triple curvature on a survey line is dominated by per-point noise
    (a 1 cm error over a 0.5 m chord swamps a 40 m radius), so the stencil spans
    ``stencil_m`` of arc length either side and the result is median-smoothed.
    Points too close to an end for a full stencil inherit the nearest fully
    measured value rather than being declared straight — otherwise every arc
    would lose its head and tail to a false straight.
    """
    n = len(run)
    arclen = [0.0]
    for i in range(n - 1):
        arclen.append(arclen[-1] + math.dist(run[i], run[i + 1]))
    total = arclen[-1]
    if total < 1e-9:
        return [0.0] * n
    # Never let the stencil exceed a quarter of the run, or a short genuine arc
    # would have no point with a full stencil at all.
    half = max(min(stencil_m, total / 4.0), 1e-6)

    def _index_at(i: int, offset: float) -> int:
        target = arclen[i] + offset
        j = i
        step = 1 if offset > 0 else -1
        while 0 <= j + step < n and abs(arclen[j + step] - arclen[i]) < abs(offset):
            j += step
        return max(0, min(n - 1, j))

    raw: list[float | None] = [None] * n
    for i in range(n):
        lo = _index_at(i, -half)
        hi = _index_at(i, half)
        # Require a genuinely two-sided stencil; ends are filled in below.
        if lo == i or hi == i:
            continue
        if arclen[i] - arclen[lo] < half * 0.5 or arclen[hi] - arclen[i] < half * 0.5:
            continue
        raw[i] = _menger_curvature(run[lo], run[i], run[hi])

    measured = [k for k in raw if k is not None]
    if not measured:
        return [0.0] * n
    first = next(i for i, k in enumerate(raw) if k is not None)
    last = next(i for i in range(n - 1, -1, -1) if raw[i] is not None)
    filled = [
        raw[i] if raw[i] is not None else (raw[first] if i < first else raw[last])
        for i in range(n)
    ]
    # Median smoothing over a 5-sample window kills the residual spikes that a
    # single rounded coordinate still puts into the wide-stencil signal.
    out = []
    for i in range(n):
        window = filled[max(0, i - 2): min(n, i + 3)]
        out.append(sorted(window)[len(window) // 2])
    return out


def _trim_arc_bounds(run: list[Point], lo: int, hi: int, max_dev: float,
                     rms: float, min_points: int = 5) -> tuple[int, int] | None:
    """Shrink ``[lo, hi]`` to the range that is genuinely one circular arc.

    The curvature stencil spans several metres, so a point that far into the
    adjoining straight still reads as curved and the detected arc bleeds past
    the real tangent point. Those straight tails drag the circle fit off (a 90
    deg bend with 3 m of straight on each end fits with 24 cm of residual and is
    refused outright).

    Localisation runs at the SURVEY's own precision (``3 * rms``), not at
    ``max_dev``: with a 15 cm budget the tangent point of a 15 m bend can only be
    pinned to about 2 m, which leaves an 8 deg kink at the joint. But a coarse
    survey cannot support that precision at all — a circle sampled as a polygon
    carries 11 cm of residual at every point, and trimming it to 3 cm would eat
    the whole arc. So: try the precise trim, and if it collapses the run, fall
    back to judging the detected run as a whole against ``max_dev``.

    Returns None when no sub-range is circular enough for ``max_dev``.
    """
    tight = max(3.0 * rms, 0.02)
    tlo, thi = lo, hi
    ok = False
    for _ in range((hi - lo) + 1):
        if thi - tlo + 1 < min_points:
            break
        sub = run[tlo:thi + 1]
        fit = _fit_circle_kasa(sub)
        if fit is None:
            break
        cn, ce, r = fit
        resid = [abs(math.hypot(p[0] - cn, p[1] - ce) - r) for p in sub]
        if max(resid) <= tight:
            ok = True
            break
        # The bleed is always at an end, so retreat from the worse one.
        if resid[0] >= resid[-1]:
            tlo += 1
        else:
            thi -= 1
    # Keep the precise result only if it still covers most of the detected arc;
    # otherwise the survey is too coarse for this tolerance and we judge whole.
    if ok and (thi - tlo) >= 0.5 * (hi - lo):
        return (tlo, thi)

    fit = _fit_circle_kasa(run[lo:hi + 1])
    if fit is None:
        return None
    cn, ce, r = fit
    worst = max(abs(math.hypot(p[0] - cn, p[1] - ce) - r) for p in run[lo:hi + 1])
    return (lo, hi) if worst <= max_dev else None


def _segment_by_curvature(
    run: list[Point],
    max_arc_radius_m: float = MAX_ARC_RADIUS_M,
    stencil_m: float = CURVATURE_STENCIL_M,
    min_subrun_m: float = MIN_SUBRUN_M,
    max_dev_m: float = MAX_ARC_DEVIATION_M,
    rms_m: float = 0.025,
) -> list[tuple[int, int, bool]]:
    """Split a corner-free run into ``(start, end, is_arc)`` sub-runs.

    A point is ARC when |curvature| clears ``1 / max_arc_radius_m``, STRAIGHT when
    it falls back under half that (hysteresis, so a run does not chatter across
    the threshold), and a curvature sign flip starts a new sub-run so an S-bend
    becomes two arcs rather than one impossible circle. Sub-runs shorter than
    ``min_subrun_m`` are absorbed into their predecessor — below that length the
    classification is noise and the geometry is a chord either way.

    Indices are inclusive and adjacent sub-runs share their boundary vertex, so
    the chain stays connected.
    """
    n = len(run)
    if n < 3:
        return [(0, n - 1, False)]
    kappa = _curvature_profile(run, stencil_m)
    enter = 1.0 / max_arc_radius_m
    exit_ = enter * 0.5

    states: list[int] = []           # -1 CW arc, 0 straight, +1 CCW arc
    state = 0
    for k in kappa:
        mag = abs(k)
        sign = 1 if k > 0 else -1
        if state == 0:
            if mag > enter:
                state = sign
        elif mag < exit_:
            state = 0
        elif sign != state and mag > enter:
            state = sign
        states.append(state)

    bounds: list[tuple[int, int, int]] = []
    start = 0
    for i in range(1, n + 1):
        if i == n or states[i] != states[start]:
            bounds.append((start, i - 1, states[start]))
            start = i

    def _length(a: int, b: int) -> float:
        return sum(math.dist(run[i], run[i + 1]) for i in range(a, b))

    merged: list[list[int]] = []
    for a, b, st in bounds:
        if merged and _length(a, b) < min_subrun_m:
            merged[-1][1] = b            # absorb into the previous sub-run
        else:
            merged.append([a, b, st])
    # A leading stub can only be absorbed forwards.
    while len(merged) > 1 and _length(merged[0][0], merged[0][1]) < min_subrun_m:
        merged[1][0] = merged[0][0]
        merged.pop(0)

    # Refine each detected arc back to the range one circle actually explains,
    # then rebuild the runs from the corrected labels so the trimmed tails
    # rejoin the neighbouring straights.
    refined = [0] * n
    for a, b, st in merged:
        if st == 0:
            continue
        bounds_ = _trim_arc_bounds(run, a, b, max_dev_m, rms_m)
        if bounds_ is None:
            continue                     # not circular anywhere: leave it straight
        lo, hi = bounds_
        if _length(lo, hi) < min_subrun_m:
            continue
        for i in range(lo, hi + 1):
            refined[i] = st

    blocks: list[list[int]] = []
    start = 0
    for i in range(1, n + 1):
        if i == n or refined[i] != refined[start]:
            blocks.append([start, i - 1, refined[start]])
            start = i

    # Two arcs of opposite curvature meet at an inflection, where the curvature
    # is zero by definition — give that vertex its own straight block so the two
    # arcs are never neighbours and each still has a run to share a joint with.
    separated: list[list[int]] = []
    for a, b, st in blocks:
        if separated and separated[-1][2] != 0 and st != 0:
            if b > a:
                separated.append([a, a, 0])
                a += 1
            else:
                st = 0
        separated.append([a, b, st])

    # Share the boundary vertex so consecutive sub-runs join exactly. The
    # STRAIGHT side reaches out to take it: an arc's bounds were trimmed to the
    # range one circle explains, and growing them back would undo that.
    out: list[tuple[int, int, bool]] = []
    for idx, (a, b, st) in enumerate(separated):
        if st == 0:
            if idx > 0 and separated[idx - 1][2] != 0:
                a = max(0, a - 1)
            if idx < len(separated) - 1 and separated[idx + 1][2] != 0:
                b = min(n - 1, b + 1)
        out.append((a, b, st != 0))
    return out


def _max_chord_deviation(run: list[Point]) -> float:
    """Max perpendicular distance of interior points from the endpoint chord."""
    if len(run) < 3:
        return 0.0
    (n0, e0), (n1, e1) = run[0], run[-1]
    dn, de = n1 - n0, e1 - e0
    length = math.hypot(dn, de)
    if length < 1e-9:
        # Degenerate chord (a closed run): fall back to spread about the start.
        return max(math.hypot(p[0] - n0, p[1] - e0) for p in run)
    worst = 0.0
    for pn, pe in run[1:-1]:
        # |cross((p-p0), chord)| / |chord|
        cross = abs((pn - n0) * de - (pe - e0) * dn)
        worst = max(worst, cross / length)
    return worst


def _fit_circle_kasa(run: list[Point]) -> tuple[float, float, float] | None:
    """Kasa algebraic circle fit. Returns (center_n, center_e, radius) or None.

    Minimises sum((x^2 + y^2 + D x + E y + F)^2) with x=east, y=north, giving the
    linear normal equations solved by Cramer's rule below. Returns None when the
    points are collinear (singular system).
    """
    n = len(run)
    if n < 3:
        return None
    Sxx = Syy = Sxy = Sx = Sy = 0.0
    Sxz = Syz = Sz = 0.0
    for pn, pe in run:
        x, y = pe, pn                      # x=east, y=north
        z = x * x + y * y
        Sxx += x * x
        Syy += y * y
        Sxy += x * y
        Sx += x
        Sy += y
        Sxz += x * z
        Syz += y * z
        Sz += z
    # Solve A [D E F]^T = b, A symmetric 3x3.
    a11, a12, a13 = Sxx, Sxy, Sx
    a21, a22, a23 = Sxy, Syy, Sy
    a31, a32, a33 = Sx, Sy, float(n)
    b1, b2, b3 = -Sxz, -Syz, -Sz
    det = (a11 * (a22 * a33 - a23 * a32)
           - a12 * (a21 * a33 - a23 * a31)
           + a13 * (a21 * a32 - a22 * a31))
    if abs(det) < 1e-12:
        return None
    dD = (b1 * (a22 * a33 - a23 * a32)
          - a12 * (b2 * a33 - a23 * b3)
          + a13 * (b2 * a32 - a22 * b3))
    dE = (a11 * (b2 * a33 - a23 * b3)
          - b1 * (a21 * a33 - a23 * a31)
          + a13 * (a21 * b3 - b2 * a31))
    dF = (a11 * (a22 * b3 - b2 * a32)
          - a12 * (a21 * b3 - b2 * a31)
          + b1 * (a21 * a32 - a22 * a31))
    D, E, F = dD / det, dE / det, dF / det
    cx, cy = -D / 2.0, -E / 2.0            # center in (east, north)
    disc = cx * cx + cy * cy - F
    if disc <= 0.0:
        return None
    radius = math.sqrt(disc)
    return (cy, cx, radius)                # (center_n, center_e, radius)


def _fit_arc_run(run: list[Point], rms: float, chord_error: float,
                 min_spacing: float, max_spacing: float,
                 max_dev: float = MAX_ARC_DEVIATION_M,
                 max_radius: float = MAX_ARC_RADIUS_M) -> list[Point] | None:
    """Fit ``run`` to a circle and tessellate the minor arc through its points.

    Endpoints are pinned to the surveyed endpoints' BEARINGS from the fitted
    centre, projected onto the circle — so the arc spans exactly the surveyed
    extent while every emitted point still lies on the fitted circle. Snapping
    them to the raw surveyed positions instead (as this did originally) drags the
    first and last point up to ``max_dev`` off the circle, which at 2 cm
    tessellation reads as a 70 deg kink at every arc end. ``fit_line_chain``
    reconciles the resulting joints with the neighbouring runs.

    Returns None if no valid circle could be fit.
    """
    fit = _fit_circle_kasa(run)
    if fit is None:
        return None
    cn, ce, r = fit
    if r < 1e-6 or not math.isfinite(r):
        return None

    # Reject a run that is not actually circular. A straight road, or a
    # near-straight noisy run, "fits" a huge circle that passes through the
    # points but whose arc bears no relation to the shape — using it produces
    # the blow-up. Two independent guards:
    #   * radius — beyond ``max_radius`` the "arc" is a straight for marking
    #     purposes, and its fitted centre is pure noise amplification;
    #   * deviation — every surveyed point must sit on the fitted circle to
    #     within ``max_dev``, the paint error the operator is willing to accept
    #     from replacing their samples with an arc.
    # Otherwise keep the raw polyline.
    if r > max_radius:
        return None
    resid = max(abs(math.hypot(pn - cn, pe - ce) - r) for pn, pe in run)
    if resid > max(max_dev, 3.0 * rms):
        return None

    # True signed sweep about the centre (sum of per-chord turns). Its sign is
    # the traversal direction; its magnitude is the arc actually driven.
    sweep = 0.0
    for i in range(len(run) - 1):
        a_n, a_e = run[i][0] - cn, run[i][1] - ce
        b_n, b_e = run[i + 1][0] - cn, run[i + 1][1] - ce
        sweep += math.atan2(a_e * b_n - a_n * b_e, a_n * b_n + a_e * b_e)
    if abs(sweep) < math.radians(2.0):
        return None  # essentially straight — keep it straight, never a full circle

    start_deg = math.degrees(math.atan2(run[0][0] - cn, run[0][1] - ce))
    direction = "CCW" if sweep > 0 else "CW"
    # Drive the end angle from start + the TRUE sweep, so arc_waypoints traces
    # exactly this arc and can never wrap to a spurious ~360° circle (the old
    # bug: a 0.5° run rendered as 359.5°).
    end_deg = start_deg + math.degrees(sweep)
    pts = arc_waypoints((cn, ce), r, start_deg, end_deg,
                        chord_error=chord_error, min_spacing=min_spacing,
                        max_spacing=max_spacing, direction=direction)
    if len(pts) < 2:
        return None
    # Pin the ends to the surveyed bearings, projected onto the circle.
    def _on_circle(p: Point) -> Point:
        dn, de = p[0] - cn, p[1] - ce
        m = math.hypot(dn, de)
        if m < 1e-9:
            return p
        return (cn + r * dn / m, ce + r * de / m)

    pts[0] = _on_circle(run[0])
    pts[-1] = _on_circle(run[-1])
    return pts


def _taper_shift(run: list[Point], at_start: bool, target: Point,
                 taper_m: float) -> None:
    """Move ``run``'s boundary point to ``target``, blending the shift inwards.

    The full correction lands on the boundary point and decays linearly to zero
    ``taper_m`` of arc length into the run, so no single step absorbs the whole
    displacement. Mutates ``run`` in place.
    """
    if not run:
        return
    end = 0 if at_start else len(run) - 1
    dn = target[0] - run[end][0]
    de = target[1] - run[end][1]
    if math.hypot(dn, de) < 1e-12:
        return
    order = range(len(run)) if at_start else range(len(run) - 1, -1, -1)
    travelled = 0.0
    prev: Point | None = None
    for i in order:
        if prev is not None:
            travelled += math.dist(prev, run[i])
        prev = run[i]
        if travelled >= taper_m:
            break
        w = 1.0 - travelled / taper_m
        run[i] = (run[i][0] + dn * w, run[i][1] + de * w)
    run[end] = target


def fit_line_chain(
    points: list[Point],
    rms_m: float = 0.025,
    corner_angle_deg: float = 35.0,
    chord_error_m: float = 0.005,
    min_spacing_m: float = 0.02,
    max_spacing_m: float = 0.10,
    max_dev_m: float = MAX_ARC_DEVIATION_M,
    max_arc_radius_m: float = MAX_ARC_RADIUS_M,
) -> tuple[list[Point], list[int]]:
    """Fit a surveyed chain into straight runs + arcs.

    Args:
        points: surveyed (north, east) vertices, in order.
        rms_m: straight-vs-arc threshold. A sub-run whose max deviation from its
            endpoint chord is <= this stays straight (default 2.5 cm, ~= the
            survey lateral RMS observed in the field).
        corner_angle_deg: split the chain wherever the heading turns by more than
            this (default 35 deg — above an arc's per-chord turn, below a corner).
        chord_error_m, min_spacing_m, max_spacing_m: passed to the existing
            arc tessellator.
        max_dev_m: how far a fitted arc may sit from the surveyed points it
            replaces before the fit is refused and the raw polyline kept.
        max_arc_radius_m: above this fitted radius a sub-run is treated as
            straight.

    Returns:
        (new_points, control_indices). control_indices index new_points.
    """
    if len(points) < 3:
        return list(points), list(range(len(points)))

    corner_angle_rad = math.radians(corner_angle_deg)
    corner_runs = _split_at_corners(points, corner_angle_rad)

    # Corner split first (squares stay square), then curvature split inside each
    # corner-free run so a road's individual bends are fit separately instead of
    # one hopeless circle through the whole road.
    runs: list[tuple[int, int, bool]] = []
    for start, end in corner_runs:
        for sub_a, sub_b, is_arc in _segment_by_curvature(
            points[start:end + 1], max_arc_radius_m=max_arc_radius_m,
            max_dev_m=max_dev_m, rms_m=rms_m,
        ):
            runs.append((start + sub_a, start + sub_b, is_arc))

    # Pass 1 — fit each sub-run independently.
    emits: list[list[Point]] = []
    is_fitted: list[bool] = []
    fitted_any = False
    for start, end, is_arc in runs:
        run = points[start:end + 1]
        arc_pts: list[Point] | None = None
        straight = not is_arc or _max_chord_deviation(run) <= rms_m
        if not straight:
            arc_pts = _fit_arc_run(run, rms_m, chord_error_m, min_spacing_m,
                                   max_spacing_m, max_dev=max_dev_m,
                                   max_radius=max_arc_radius_m)
        if arc_pts is not None:
            fitted_any = True
            emits.append(arc_pts)
            is_fitted.append(True)
        else:
            emits.append(list(run))
            is_fitted.append(False)

    # Pass 2 — reconcile the shared joints. Adjacent sub-runs share one surveyed
    # vertex, but a fitted arc has moved its copy onto the circle, so the two
    # sides no longer agree. Give the joint to the fitted side (the arc is the
    # reconstruction; the straight's vertex is one noisy sample of the same
    # place), or split the difference when both sides are arcs. Without this the
    # chain either breaks or kinks by up to ``max_dev_m`` at every transition.
    for i in range(len(emits) - 1):
        if not emits[i] or not emits[i + 1]:
            continue
        left_fitted, right_fitted = is_fitted[i], is_fitted[i + 1]
        if not left_fitted and not right_fitted:
            continue                       # both raw: already the same vertex
        a, b = emits[i][-1], emits[i + 1][0]
        if left_fitted and right_fitted:
            joint = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        else:
            joint = a if left_fitted else b
        # Move the joint by TAPERING the shift back into each run rather than
        # displacing the boundary point alone. A bare endpoint move deflects the
        # single adjacent step by the full shift, which on a 2 cm-tessellated arc
        # is a 60 deg spike — the kink it was meant to remove, just relocated.
        _taper_shift(emits[i], at_start=False, target=joint, taper_m=JOINT_TAPER_M)
        _taper_shift(emits[i + 1], at_start=True, target=joint, taper_m=JOINT_TAPER_M)

    out: list[Point] = []
    control: list[int] = []
    for emit, fitted in zip(emits, is_fitted):
        if fitted:
            # Arc ends are control; interpolated arc fill is not.
            local_control = {0, len(emit) - 1}
        else:
            # Straight run: keep every surveyed vertex as a control point.
            local_control = set(range(len(emit)))

        if out and emit and math.dist(out[-1], emit[0]) < 1e-9:
            # Shared corner already appended by the previous run.
            base = len(out) - 1
            out.extend(emit[1:])
            for k in local_control:
                idx = base + k
                if idx not in control:
                    control.append(idx)
        else:
            base = len(out)
            out.extend(emit)
            control.extend(base + k for k in local_control)

    control = sorted(set(control))
    if fitted_any:
        log.info("arc_chain: fit %d run(s) from %d surveyed points -> %d points",
                 len(runs), len(points), len(out))
    return out, control
