"""Arc and circle waypoint discretization.

Curvature-adaptive discretization using chord-error (sagitta) method.
Tighter spacing on tight curves (small radius), coarser on gentle curves.

DXF angle convention: 0° = East (positive x), angles increase CCW.
NED coordinate mapping: north = DXF y, east = DXF x.
So: north = center_n + r*sin(angle), east = center_e + r*cos(angle).
"""

from __future__ import annotations

import logging
import math

log = logging.getLogger("path_engine.arc_curve")


def arc_waypoints(
    center: tuple[float, float],
    radius: float,
    start_angle_deg: float,
    end_angle_deg: float,
    chord_error: float = 0.005,
    min_spacing: float = 0.02,
    max_spacing: float = 0.10,
    direction: str = "CCW",
) -> list[tuple[float, float]]:
    """Generate curvature-adaptive waypoints along a circular arc.

    Uses chord-error method: angular step is computed from
    theta_step = 2 * arccos(1 - e / r), where e = max chord error
    and r = radius. This guarantees the polyline never deviates
    more than `chord_error` metres from the true arc.

    Args:
        center: (north_m, east_m) arc centre.
        radius: Arc radius in metres.
        start_angle_deg: Start angle in degrees (0=East, CCW positive, DXF convention).
        end_angle_deg: End angle in degrees.
        chord_error: Maximum deviation from true arc in metres (default 5mm).
        min_spacing: Minimum waypoint spacing in metres.
        max_spacing: Maximum waypoint spacing in metres.
        direction: "CCW" (counter-clockwise) or "CW" (clockwise).

    Returns:
        List of (north_m, east_m) waypoints along the arc.
        Always includes both start and end points exactly.
    """
    if radius < 1e-9:
        return [center]

    # Compute angular step from chord error
    # theta = 2 * arccos(1 - e / r)
    if chord_error >= radius:
        angular_step = math.pi
    else:
        angular_step = 2.0 * math.acos(1.0 - chord_error / radius)

    # Clamp to spacing bounds
    if radius > min_spacing / 2:
        min_angular = 2.0 * math.asin(min_spacing / (2.0 * radius))
    else:
        min_angular = math.pi

    if radius > max_spacing / 2:
        max_angular = 2.0 * math.asin(max_spacing / (2.0 * radius))
    else:
        max_angular = math.pi

    angular_step = max(angular_step, min_angular)
    angular_step = min(angular_step, max_angular)

    # Compute sweep angle
    start_rad = math.radians(start_angle_deg)
    end_rad = math.radians(end_angle_deg)

    if direction.upper() == "CCW":
        sweep = end_rad - start_rad
        if sweep <= 0:
            log.warning(
                "arc_waypoints: CCW sweep <= 0 (start=%.1f°, end=%.1f°) — "
                "adding 2π wrap. If this is unintended, set direction='CW'.",
                start_angle_deg, end_angle_deg,
            )
            sweep += 2.0 * math.pi
    else:  # CW
        sweep = start_rad - end_rad
        if sweep <= 0:
            log.warning(
                "arc_waypoints: CW sweep <= 0 (start=%.1f°, end=%.1f°) — "
                "adding 2π wrap. If this is unintended, set direction='CCW'.",
                start_angle_deg, end_angle_deg,
            )
            sweep += 2.0 * math.pi
        # For CW, we iterate from start_angle going clockwise
        # Equivalent to CCW iteration from end_angle with reversed sweep

    n_points = max(2, int(math.ceil(sweep / angular_step)) + 1)

    cn, ce = center
    pts: list[tuple[float, float]] = []

    for i in range(n_points):
        t = i / (n_points - 1)
        if direction.upper() == "CCW":
            angle = start_rad + t * sweep
        else:  # CW
            angle = start_rad - t * sweep

        # DXF angle from East (x-axis): north = sin(angle), east = cos(angle)
        n = cn + radius * math.sin(angle)
        e = ce + radius * math.cos(angle)
        pts.append((n, e))

    # Force exact endpoints
    if direction.upper() == "CCW":
        pts[0] = (cn + radius * math.sin(start_rad),
                   ce + radius * math.cos(start_rad))
        pts[-1] = (cn + radius * math.sin(end_rad),
                    ce + radius * math.cos(end_rad))
    else:
        pts[0] = (cn + radius * math.sin(start_rad),
                   ce + radius * math.cos(start_rad))
        # CW end: start_rad - sweep
        cw_end_rad = start_rad - sweep
        pts[-1] = (cn + radius * math.sin(cw_end_rad),
                    ce + radius * math.cos(cw_end_rad))

    return pts


def densify_circle(
    center: tuple[float, float],
    radius: float,
    chord_error: float = 0.005,
    min_spacing: float = 0.02,
    max_spacing: float = 0.10,
) -> list[tuple[float, float]]:
    """Generate curvature-adaptive waypoints around a full circle.

    Convenience wrapper around arc_waypoints for 0 to 360 degrees.
    Returns a closed loop (last point ≈ first point).

    Args:
        center: (north_m, east_m) circle centre.
        radius: Circle radius in metres.
        chord_error: Maximum deviation from true circle (default 5mm).
        min_spacing: Minimum waypoint spacing.
        max_spacing: Maximum waypoint spacing.

    Returns:
        List of (north_m, east_m) waypoints, last ≈ first.
    """
    pts = arc_waypoints(
        center, radius, 0, 360,
        chord_error=chord_error,
        min_spacing=min_spacing,
        max_spacing=max_spacing,
        direction="CCW",
    )
    # Close the loop: last point should match first (tolerance scaled to chord_error)
    tol = max(0.1 * chord_error, 1e-9)
    if pts and (abs(pts[-1][0] - pts[0][0]) > tol or abs(pts[-1][1] - pts[0][1]) > tol):
        pts.append(pts[0])
    return pts


def densify_arc_from_dxf(
    center: tuple[float, float],
    radius: float,
    start_angle_deg: float,
    end_angle_deg: float,
    unit_scale: float = 1.0,
    chord_error: float = 0.005,
    min_spacing: float = 0.02,
    max_spacing: float = 0.10,
) -> list[tuple[float, float]]:
    """Discretize a DXF ARC entity.

    DXF arcs are always counter-clockwise from start_angle to end_angle.
    If end < start, the arc wraps through 360°.

    Args:
        center: (north_m, east_m) already scaled.
        radius: Radius in metres (already scaled).
        start_angle_deg: DXF start angle in degrees.
        end_angle_deg: DXF end angle in degrees.
        unit_scale: DXF-to-metres scale (used to adjust chord_error).
        chord_error: Max deviation in metres.
        min_spacing: Min waypoint spacing in metres.
        max_spacing: Max waypoint spacing in metres.

    Returns:
        List of (north_m, east_m) waypoints along the arc.
    """
    # DXF ARC is always CCW from start to end
    # If end < start, it wraps through 360° (e.g., 350° to 10° = 20° CCW)
    return arc_waypoints(
        center, radius, start_angle_deg, end_angle_deg,
        chord_error=chord_error,
        min_spacing=min_spacing,
        max_spacing=max_spacing,
        direction="CCW",
    )


def densify_lwpolyline_bulge(
    vertices: list[tuple[float, float]],
    bulges: list[float],
    closed: bool = False,
    chord_error: float = 0.005,
    min_spacing: float = 0.02,
    max_spacing: float = 0.10,
) -> list[tuple[float, float]]:
    """Discretize an LWPOLYLINE with bulge values into waypoints.

    Each segment between consecutive vertices is either:
    - Straight line (bulge = 0): densified at mark_spacing
    - Arc (bulge != 0): converted to arc waypoints using bulge parameters

    Args:
        vertices: List of (north_m, east_m) vertex positions.
        bulges: List of bulge values (one per vertex). Bulge = tan(angle/4)
                where angle is the included angle. Positive = CCW, negative = CW.
        closed: Whether the polyline is closed (last vertex connects to first).
        chord_error: Max deviation for arc segments.
        min_spacing: Min waypoint spacing for arcs.
        max_spacing: Max waypoint spacing for arcs.

    Returns:
        Flat list of (north_m, east_m) waypoints.
    """
    if not vertices:
        return []

    n_verts = len(vertices)
    pts: list[tuple[float, float]] = [vertices[0]]

    num_segments = n_verts if closed else n_verts - 1

    for i in range(num_segments):
        j = (i + 1) % n_verts
        start = vertices[i]
        end = vertices[j]
        bulge = bulges[i] if i < len(bulges) else 0.0

        if abs(bulge) < 1e-9:
            # Straight line segment — just add the endpoint
            pts.append(end)
        else:
            # Arc segment — compute center, radius, angles from bulge
            # Bulge = tan(included_angle / 4)
            # Sign: positive = CCW, negative = CW
            arc_pts = _bulge_to_arc_points(start, end, bulge, chord_error, min_spacing, max_spacing)
            if arc_pts:
                # Skip first point (already in pts from previous segment)
                pts.extend(arc_pts[1:])
            else:
                pts.append(end)

    return pts


def _bulge_to_arc_points(
    start: tuple[float, float],
    end: tuple[float, float],
    bulge: float,
    chord_error: float,
    min_spacing: float,
    max_spacing: float,
) -> list[tuple[float, float]]:
    """Convert a bulge arc segment to arc waypoints.

    Uses the standard bulge-to-arc math:
      - chord = distance(start, end)
      - included_angle = 4 * arctan(|bulge|)
      - radius = chord / (2 * sin(included_angle / 2))
      - sagitta = |bulge| * chord / 2

    Args:
        start: (north, east) start vertex.
        end: (north, east) end vertex.
        bulge: Bulge value. Positive = CCW, negative = CW.
        chord_error: Max deviation for arc discretization.
        min_spacing: Min waypoint spacing.
        max_spacing: Max waypoint spacing.

    Returns:
        List of (north, east) waypoints along the arc, including both endpoints.
    """
    import math

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    chord = math.hypot(dx, dy)

    if chord < 1e-9:
        return [start]

    abs_bulge = abs(bulge)
    included_angle = 4.0 * math.atan(abs_bulge)

    if included_angle < 1e-9:
        return [start, end]

    radius = chord / (2.0 * math.sin(included_angle / 2.0))

    if radius < 1e-9:
        return [start, end]

    # Compute center point of the arc
    # The center is at distance 'r' from both start and end, on the side
    # indicated by the bulge sign (positive bulge = center is to the left of start→end)
    # Midpoint of chord
    mid_n = (start[0] + end[0]) / 2.0
    mid_e = (start[1] + end[1]) / 2.0

    # Sagitta (distance from chord midpoint to arc midpoint)
    sagitta = abs_bulge * chord / 2.0

    # Direction perpendicular to chord
    # Perpendicular unit vector: rotate (dx, dy) by 90°
    # Left (CCW) perpendicular: (-dy, dx) normalized
    perp_n = -dy / chord
    perp_e = dx / chord

    # For positive bulge (CCW arc), center is on the left side
    # For negative bulge (CW arc), center is on the right side
    # sagitta direction: from chord midpoint towards arc
    # For CCW (positive bulge): center is on opposite side of arc from chord
    # Distance from midpoint to center = r - sagitta (for minor arc) or r + sagitta (for major arc)
    # For bulge < 1 (minor arc, included angle < 180°): center_distance = r - sagitta
    # Actually: distance from midpoint to center = sqrt(r² - (chord/2)²)
    half_chord = chord / 2.0
    center_dist = math.sqrt(max(0, radius * radius - half_chord * half_chord))

    # Direction from midpoint to center
    # For positive bulge (CCW), center is on the left side of start→end
    sign = 1.0 if bulge > 0 else -1.0

    center_n = mid_n + sign * perp_n * center_dist
    center_e = mid_e + sign * perp_e * center_dist

    # Compute start and end angles from center
    start_angle = math.atan2(start[0] - center_n, start[1] - center_e)
    end_angle = math.atan2(end[0] - center_n, end[1] - center_e)

    # Convert to degrees
    start_angle_deg = math.degrees(start_angle)
    end_angle_deg = math.degrees(end_angle)

    # Direction: positive bulge = CCW, negative = CW
    direction = "CCW" if bulge > 0 else "CW"

    return arc_waypoints(
        (center_n, center_e), radius,
        start_angle_deg, end_angle_deg,
        chord_error=chord_error,
        min_spacing=min_spacing,
        max_spacing=max_spacing,
        direction=direction,
    )