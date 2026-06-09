"""Path smoothing helpers for planner-side geometry conditioning."""

from __future__ import annotations

import logging
import math

log = logging.getLogger(__name__)


def _corner_arc(
    prev_pt: tuple[float, float],
    vertex: tuple[float, float],
    next_pt: tuple[float, float],
    radius: float,
    samples: int,
) -> tuple[list[tuple[float, float]], bool]:
    """Compute the inscribed arc that rounds a single corner.

    Returns ``(points_to_emit, skipped)`` where ``points_to_emit`` is the list of
    points that should replace ``vertex`` in the output:
      - ``[]``            → degenerate (coincident neighbour); drop the vertex.
      - ``[vertex]``      → corner left sharp (near-straight, or too short for the
                            requested radius). ``skipped`` is True only for the
                            too-short case.
      - arc point list    → the rounded corner (entry → arc samples → exit).
    """
    ax, ay = prev_pt
    px, py = vertex
    bx, by = next_pt
    v1n, v1e = ax - px, ay - py
    v2n, v2e = bx - px, by - py
    l1 = math.hypot(v1n, v1e)
    l2 = math.hypot(v2n, v2e)
    if l1 < 1e-9 or l2 < 1e-9:
        return [], False

    u1n, u1e = v1n / l1, v1e / l1
    u2n, u2e = v2n / l2, v2e / l2

    dot = max(-1.0, min(1.0, u1n * u2n + u1e * u2e))
    theta = math.acos(dot)
    if theta < 1e-3 or math.pi - theta < 1e-3:
        return [vertex], False

    tangent_len = radius / math.tan(theta / 2.0)
    if tangent_len > 0.45 * min(l1, l2):
        return [vertex], True

    sa_n = px + tangent_len * u1n
    sa_e = py + tangent_len * u1e
    sb_n = px + tangent_len * u2n
    sb_e = py + tangent_len * u2e

    bis_n = u1n + u2n
    bis_e = u1e + u2e
    bis_len = math.hypot(bis_n, bis_e)
    if bis_len < 1e-9:
        return [vertex], False
    bis_n /= bis_len
    bis_e /= bis_len

    center_dist = radius / math.sin(theta / 2.0)
    cx_n = px + center_dist * bis_n
    cx_e = py + center_dist * bis_e

    r1n = sa_n - cx_n
    r1e = sa_e - cx_e
    r2n = sb_n - cx_n
    r2e = sb_e - cx_e
    ang1 = math.atan2(r1e, r1n)
    ang2 = math.atan2(r2e, r2n)
    cross_z = r1n * r2e - r1e * r2n
    sweep = ang2 - ang1
    if cross_z >= 0:
        if sweep < 0:
            sweep += 2.0 * math.pi
    else:
        if sweep > 0:
            sweep -= 2.0 * math.pi

    arc: list[tuple[float, float]] = [(sa_n, sa_e)]
    for k in range(1, samples):
        a = ang1 + sweep * (k / samples)
        arc.append((cx_n + radius * math.cos(a), cx_e + radius * math.sin(a)))
    arc.append((sb_n, sb_e))
    return arc, False


def smooth_corners(
    pts: list[tuple[float, float]],
    radius: float,
    arc_pts: int,
) -> tuple[list[tuple[float, float]], int]:
    """Replace interior vertices with inscribed circular arcs.

    The returned path bounds smoothed corner curvature at approximately
    1 / radius. Vertices whose adjacent segments are too short for the requested
    radius are left sharp and counted as skipped.

    Closed paths (first point ≈ last point) are smoothed cyclically so the
    start/end junction is rounded like any other corner. Open paths keep their
    original endpoints.
    """
    n = len(pts)
    if n < 3 or radius <= 0.0:
        return list(pts), 0

    samples = max(2, int(arc_pts))
    skipped = 0

    is_closed = math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 1e-9

    if is_closed:
        # Treat the unique vertices as a cycle so the closure corner is rounded too.
        verts = pts[:-1]
        m = len(verts)
        if m < 3:
            return list(pts), 0
        out: list[tuple[float, float]] = []
        for i in range(m):
            arc, sk = _corner_arc(verts[i - 1], verts[i], verts[(i + 1) % m], radius, samples)
            skipped += int(sk)
            out.extend(arc)
        # Re-close the loop.
        if out:
            out.append(out[0])
    else:
        out = [pts[0]]
        for i in range(1, n - 1):
            arc, sk = _corner_arc(pts[i - 1], pts[i], pts[i + 1], radius, samples)
            skipped += int(sk)
            out.extend(arc)
        out.append(pts[-1])

    if skipped:
        log.warning(
            "corner smoothing skipped %d vertices; adjacent segments are too short "
            "for radius %.3fm",
            skipped,
            radius,
        )

    return out, skipped
