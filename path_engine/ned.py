"""Coordinate transforms — lat/lon to NED metres.

Uses GeographicLib Karney geodesic (WGS84) for accurate lat/lon conversion.
Centralized here so both the ROS2 path_publisher and the FastAPI server
use the same implementation.
"""

from __future__ import annotations

import logging
import math

log = logging.getLogger(__name__)

try:
    from geographiclib.geodesic import Geodesic
    _HAS_GEOGRAPHICLIB = True
except ImportError:
    _HAS_GEOGRAPHICLIB = False


def latlon_to_ned(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    """Convert lat/lon to NED metres relative to an origin using Karney geodesic.

    Args:
        lat: Target latitude (degrees).
        lon: Target longitude (degrees).
        origin_lat: Origin latitude (degrees).
        origin_lon: Origin longitude (degrees).

    Returns:
        (north_m, east_m) relative to origin.

    Raises:
        ImportError: If geographiclib is not installed.
    """
    if not _HAS_GEOGRAPHICLIB:
        raise ImportError(
            "geographiclib is required for lat/lon conversion. "
            "Install: pip install geographiclib"
        )

    geod = Geodesic.WGS84
    result = geod.Inverse(origin_lat, origin_lon, lat, lon)
    dist = result["s12"]
    bearing_rad = math.radians(result["azi1"])
    north = dist * math.cos(bearing_rad)
    east = dist * math.sin(bearing_rad)
    return (north, east)


def ned_to_latlon(
    north: float,
    east: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    """Inverse of latlon_to_ned: NED metres about an origin → WGS84 lat/lon.

    Exact inverse using the same Karney geodesic. Forward maps a lat/lon to
    (dist, azimuth) then (north, east) = (d·cos, d·sin); this recovers
    dist = hypot(n, e), azimuth = atan2(east, north), and walks the geodesic
    from the origin. Used to render a local-frame /path back into geo
    coordinates for the post-mission geo overlay (tools/analyze_mission.py).

    Args:
        north: metres north of the origin.
        east: metres east of the origin.
        origin_lat: Origin latitude (degrees).
        origin_lon: Origin longitude (degrees).

    Returns:
        (lat, lon) in degrees.

    Raises:
        ImportError: If geographiclib is not installed.
    """
    if not _HAS_GEOGRAPHICLIB:
        raise ImportError(
            "geographiclib is required for lat/lon conversion. "
            "Install: pip install geographiclib"
        )
    dist = math.hypot(float(north), float(east))
    azimuth_deg = math.degrees(math.atan2(float(east), float(north)))
    result = Geodesic.WGS84.Direct(origin_lat, origin_lon, azimuth_deg, dist)
    return (result["lat2"], result["lon2"])


def _fit_similarity_coeffs(
    dxf_points: list[tuple[float, float]],
    ref_ned_points: list[tuple[float, float]],
) -> tuple[float, float, float, float, float, float, int]:
    """Least-squares similarity coefficients (a, b) plus both centroids.

    Shared by the rigid solve and the free-scale diagnostic. ``a`` and ``b`` are
    the coefficients of NED = [[a, -b], [b, a]] @ DXF + offset, from which
    scale = hypot(a, b) and theta = atan2(b, a).
    """
    n_pts = min(len(dxf_points), len(ref_ned_points))
    if n_pts < 2:
        raise ValueError("Need at least 2 reference point pairs for affine transform")

    mean_dy = sum(pt[0] for pt in dxf_points[:n_pts]) / n_pts
    mean_dx = sum(pt[1] for pt in dxf_points[:n_pts]) / n_pts
    mean_n = sum(pt[0] for pt in ref_ned_points[:n_pts]) / n_pts
    mean_e = sum(pt[1] for pt in ref_ned_points[:n_pts]) / n_pts

    u = [pt[0] - mean_dy for pt in dxf_points[:n_pts]]
    v = [pt[1] - mean_dx for pt in dxf_points[:n_pts]]
    x = [pt[0] - mean_n for pt in ref_ned_points[:n_pts]]
    y = [pt[1] - mean_e for pt in ref_ned_points[:n_pts]]

    # Normal equations:
    # [ u_i  -v_i ] [ a ]   [ x_i ]
    # [ v_i   u_i ] [ b ] = [ y_i ]
    denom = sum(u_i * u_i + v_i * v_i for u_i, v_i in zip(u, v))
    if denom < 1e-9:
        raise ValueError("DXF reference points are coincident")

    a = sum(u_i * x_i + v_i * y_i for u_i, v_i, x_i, y_i in zip(u, v, x, y)) / denom
    b = sum(u_i * y_i - v_i * x_i for u_i, v_i, x_i, y_i in zip(u, v, x, y)) / denom

    return (a, b, mean_dy, mean_dx, mean_n, mean_e, n_pts)


def estimate_fit_scale(
    dxf_points: list[tuple[float, float]],
    ref_ned_points: list[tuple[float, float]],
) -> float:
    """Free-scale diagnostic: the scale a *similarity* fit would have chosen.

    This is NOT applied to geometry — ``dxf_to_ned_affine`` is rigid. It exists
    purely as a health signal:

      - ~100 or ~0.01  → unit/frame mismatch (e.g. cm ref points vs metre geometry)
      - ~1.00 ± small  → healthy; the residual disagreement shows up in RMSE instead

    Raises:
        ValueError: fewer than 2 pairs, or coincident DXF points.
    """
    a, b, *_ = _fit_similarity_coeffs(dxf_points, ref_ned_points)
    return math.hypot(a, b)


def dxf_to_ned_affine(
    dxf_points: list[tuple[float, float]],
    ref_ned_points: list[tuple[float, float]],
    lock_scale: bool = True,
) -> tuple[float, float, float, float, list[float], float]:
    """Compute a 2D **rigid** transform (rotation + translation) from DXF to NED.

    The transform is: NED = scale * R(θ) @ DXF + offset, with **scale forced to
    1.0** by default.

    Why scale is locked
    -------------------
    The DXF is dimensionally authoritative: a 2 m square in the drawing must be a
    2 m square on the ground. Reference points tell us *where* the drawing goes and
    *which way* it is turned — never *how big* it is.

    A free-scale (similarity/Umeyama) fit absorbs GPS/survey noise into ``scale``,
    silently stretching every dimension of the drawing. With an exactly-determined
    2-point fit that stretch is unbounded by residuals, because a similarity fit
    through 2 points has zero residual *by construction* — so RMSE reads ~0 no
    matter how wrong the size is, and cross-track error cannot see it either
    (a perfectly-tracked wrong-sized path scores perfectly).

    Locking scale makes RMSE meaningful: a rigid fit through 2 points has one
    residual degree of freedom, and it is exactly the baseline-length mismatch. So
    a survey that disagrees with the drawing's dimensions now shows up as RMSE and
    is caught by the existing RMSE gate, instead of being absorbed into geometry.

    Use ``estimate_fit_scale()`` for the free-scale value as a *diagnostic*.

    Args:
        dxf_points: Reference points in DXF coordinates [(dxf_y, dxf_x)].
        ref_ned_points: Corresponding points in NED [(north, east)].
        lock_scale: Force scale to 1.0 (rigid). Default True. Setting this False
            restores the legacy free-scale behaviour and re-opens the geometry
            distortion described above — it exists only for the diagnostic tests.

    Returns:
        (scale, theta_rad, offset_north, offset_east, residuals, rmse)
        ``scale`` is the scale actually *applied* (1.0 when locked).

    Raises:
        ValueError: If fewer than 2 reference point pairs are provided or if points
            are coincident.
    """
    a, b, mean_dy, mean_dx, mean_n, mean_e, n_pts = _fit_similarity_coeffs(
        dxf_points, ref_ned_points
    )

    theta = math.atan2(b, a)

    if lock_scale:
        # Rigid: rotation from the fit, scale pinned to unity.
        scale = 1.0
        ca, cb = math.cos(theta), math.sin(theta)
    else:
        scale = math.hypot(a, b)
        ca, cb = a, b

    # Translation places the DXF centroid onto the NED centroid under (scale, theta).
    offset_n = mean_n - (ca * mean_dy - cb * mean_dx)
    offset_e = mean_e - (cb * mean_dy + ca * mean_dx)

    # Residuals/RMSE are measured against the transform we actually APPLY, so a
    # size disagreement between survey and drawing is reported, not absorbed.
    residuals = []
    sq_err_sum = 0.0
    for dxf_pt, ned_pt in zip(dxf_points[:n_pts], ref_ned_points[:n_pts]):
        pred = apply_affine_transform(dxf_pt, scale, theta, offset_n, offset_e)
        res = math.hypot(ned_pt[0] - pred[0], ned_pt[1] - pred[1])
        residuals.append(res)
        sq_err_sum += res * res

    rmse = math.sqrt(sq_err_sum / n_pts)

    return (scale, theta, offset_n, offset_e, residuals, rmse)


def apply_affine_transform(
    point: tuple[float, float],
    scale: float,
    theta: float,
    offset_n: float,
    offset_e: float,
) -> tuple[float, float]:
    """Apply a 2D affine transform to a DXF point.

    Args:
        point: (dxf_y, dxf_x) in DXF coordinates.
        scale: Uniform scale factor.
        theta: Rotation angle in radians.
        offset_n: North offset in metres.
        offset_e: East offset in metres.

    Returns:
        (north_m, east_m) in NED.
    """
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    sx = point[0] * scale
    sy = point[1] * scale
    north = sx * cos_t - sy * sin_t + offset_n
    east = sx * sin_t + sy * cos_t + offset_e
    return (north, east)