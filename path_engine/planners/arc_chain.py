"""Fit a surveyed LINE_CHAIN into straight runs + circular arcs.

A field-surveyed chain is a sequence of noisy point samples. The default
pipeline densifies it into straight chords (``straight_line.densify_segment``),
which turns a genuine surveyed curve into a polygon. This module, used only when
``PathEngine(fit_arcs=True)``, does the minimal thing that fixes that:

  1. Split the chain at genuine corners — vertices where the heading changes by
     more than ``corner_angle_deg``. A square's 90 deg corners split cleanly; an
     arc's small per-chord turns do not, so squares stay square and arcs stay
     whole. Turn-angle (not Douglas-Peucker) is what the operator chose.
  2. Per run between corners: near-collinear (max deviation from the endpoint
     chord <= ``rms_m``, the survey's lateral RMS) stays a straight run; anything
     more curved is fit to ONE circle by Kasa least-squares (inline, no new
     dependency) and re-emitted through the EXISTING curvature-adaptive
     tessellator ``arc_curve.arc_waypoints``. No new densification is written.

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
                 min_spacing: float, max_spacing: float) -> list[Point] | None:
    """Fit ``run`` to a circle and tessellate the minor arc through its points.

    Endpoints are snapped back to the surveyed corner points so runs join
    exactly at the corners. Returns None if no valid circle could be fit.
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
    # the blow-up. Require the points to sit on the fitted circle to within a
    # few times the survey noise; otherwise keep the raw polyline.
    resid = max(abs(math.hypot(pn - cn, pe - ce) - r) for pn, pe in run)
    if resid > max(3.0 * rms, 0.10):
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
    # Snap to the true surveyed endpoints (fit lands them ~mm off the circle).
    pts[0] = run[0]
    pts[-1] = run[-1]
    return pts


def fit_line_chain(
    points: list[Point],
    rms_m: float = 0.025,
    corner_angle_deg: float = 35.0,
    chord_error_m: float = 0.005,
    min_spacing_m: float = 0.02,
    max_spacing_m: float = 0.10,
) -> tuple[list[Point], list[int]]:
    """Fit a surveyed chain into straight runs + arcs.

    Args:
        points: surveyed (north, east) vertices, in order.
        rms_m: straight-vs-arc threshold. A run whose max deviation from its
            endpoint chord is <= this stays straight (default 2.5 cm, ~= the
            survey lateral RMS observed in the field).
        corner_angle_deg: split the chain wherever the heading turns by more than
            this (default 35 deg — above an arc's per-chord turn, below a corner).
        chord_error_m, min_spacing_m, max_spacing_m: passed to the existing
            arc tessellator.

    Returns:
        (new_points, control_indices). control_indices index new_points.
    """
    if len(points) < 3:
        return list(points), list(range(len(points)))

    corner_angle_rad = math.radians(corner_angle_deg)
    runs = _split_at_corners(points, corner_angle_rad)

    out: list[Point] = []
    control: list[int] = []
    fitted_any = False
    for start, end in runs:
        run = points[start:end + 1]
        arc_pts: list[Point] | None = None
        straight = _max_chord_deviation(run) <= rms_m
        if not straight:
            arc_pts = _fit_arc_run(run, rms_m, chord_error_m, min_spacing_m, max_spacing_m)

        if arc_pts is not None:
            fitted_any = True
            emit = arc_pts
            # Corners (run endpoints) are control; interpolated arc fill is not.
            local_control = {0, len(emit) - 1}
        else:
            emit = list(run)
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
