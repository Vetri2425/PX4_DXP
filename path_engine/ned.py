"""Coordinate transforms — lat/lon to NED metres.

Uses GeographicLib Karney geodesic (WGS84) for accurate lat/lon conversion.
Centralized here so both the ROS2 path_publisher and the FastAPI server
use the same implementation.
"""

from __future__ import annotations

import math

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
) -> tuple[float, float, float, float]:
    """Compute 2D affine transform from DXF coordinates to NED.

    Uses 2 reference point pairs to compute:
      - Scale (assumed uniform)
      - Rotation
      - Translation (north, east offsets)

    The transform is: NED = scale * R(θ) @ DXF + offset

    Args:
        dxf_points: 2 reference points in DXF coordinates [(dxf_y, dxf_x)].
        ref_ned_points: 2 corresponding points in NED [(north, east)].

    Returns:
        (scale, theta_rad, offset_north, offset_east)

    Raises:
        ValueError: If fewer than 2 reference point pairs are provided.
    """
    if len(dxf_points) < 2 or len(ref_ned_points) < 2:
        raise ValueError("Need at least 2 reference point pairs for affine transform")

    # Vector in DXF space
    dx = dxf_points[1][0] - dxf_points[0][0]
    dy = dxf_points[1][1] - dxf_points[0][1]
    dxf_dist = math.hypot(dx, dy)

    # Vector in NED space
    dn = ref_ned_points[1][0] - ref_ned_points[0][0]
    de = ref_ned_points[1][1] - ref_ned_points[0][1]
    ned_dist = math.hypot(dn, de)

    if dxf_dist < 1e-9:
        raise ValueError("DXF reference points are coincident")

    scale = ned_dist / dxf_dist
    theta = math.atan2(de, dn) - math.atan2(dy, dx)

    # Translation: ref_ned = scale * R(θ) @ dxf + offset
    # offset = ref_ned - scale * R(θ) @ dxf
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    scaled_dx = dxf_points[0][0] * scale
    scaled_dy = dxf_points[0][1] * scale
    offset_n = ref_ned_points[0][0] - (scaled_dx * cos_t - scaled_dy * sin_t)
    offset_e = ref_ned_points[0][1] - (scaled_dx * sin_t + scaled_dy * cos_t)

    return (scale, theta, offset_n, offset_e)


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