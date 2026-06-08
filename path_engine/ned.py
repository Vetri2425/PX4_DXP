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


def dxf_to_ned_affine(
    dxf_points: list[tuple[float, float]],
    ref_ned_points: list[tuple[float, float]],
) -> tuple[float, float, float, float, list[float], float]:
    """Compute 2D affine transform from DXF coordinates to NED.

    Uses least-squares parameter fitting (supporting N >= 2 point pairs) to compute:
      - Scale (assumed uniform)
      - Rotation
      - Translation (north, east offsets)
      - Residuals for each point pair
      - Root-mean-square error (RMSE)

    The transform is: NED = scale * R(θ) @ DXF + offset

    Args:
        dxf_points: Reference points in DXF coordinates [(dxf_y, dxf_x)].
        ref_ned_points: Corresponding points in NED [(north, east)].

    Returns:
        (scale, theta_rad, offset_north, offset_east, residuals, rmse)

    Raises:
        ValueError: If fewer than 2 reference point pairs are provided or if points are coincident.
    """
    n_pts = min(len(dxf_points), len(ref_ned_points))
    if n_pts < 2:
        raise ValueError("Need at least 2 reference point pairs for affine transform")

    # Centroids of both point sets
    sum_dy = sum(pt[0] for pt in dxf_points[:n_pts])
    sum_dx = sum(pt[1] for pt in dxf_points[:n_pts])
    sum_n = sum(pt[0] for pt in ref_ned_points[:n_pts])
    sum_e = sum(pt[1] for pt in ref_ned_points[:n_pts])

    mean_dy = sum_dy / n_pts
    mean_dx = sum_dx / n_pts
    mean_n = sum_n / n_pts
    mean_e = sum_e / n_pts

    # Centered points
    u = [pt[0] - mean_dy for pt in dxf_points[:n_pts]]
    v = [pt[1] - mean_dx for pt in dxf_points[:n_pts]]
    x = [pt[0] - mean_n for pt in ref_ned_points[:n_pts]]
    y = [pt[1] - mean_e for pt in ref_ned_points[:n_pts]]

    # Solve normal equations:
    # [ u_i  -v_i ] [ a ]   [ x_i ]
    # [ v_i   u_i ] [ b ] = [ y_i ]
    # Least-squares solution:
    # a = sum(u_i * x_i + v_i * y_i) / sum(u_i^2 + v_i^2)
    # b = sum(u_i * y_i - v_i * x_i) / sum(u_i^2 + v_i^2)
    denom = sum(u_i * u_i + v_i * v_i for u_i, v_i in zip(u, v))
    if denom < 1e-9:
        raise ValueError("DXF reference points are coincident")

    a = sum(u_i * x_i + v_i * y_i for u_i, v_i, x_i, y_i in zip(u, v, x, y)) / denom
    b = sum(u_i * y_i - v_i * x_i for u_i, v_i, x_i, y_i in zip(u, v, x, y)) / denom

    scale = math.hypot(a, b)
    theta = math.atan2(b, a)

    # Translation
    offset_n = mean_n - (a * mean_dy - b * mean_dx)
    offset_e = mean_e - (b * mean_dy + a * mean_dx)

    # Compute residuals and RMSE
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