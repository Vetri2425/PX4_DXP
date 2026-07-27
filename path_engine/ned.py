"""Coordinate transforms — lat/lon to NED metres.

FRAME CONTRACT (B6', 2026-07-25)
--------------------------------
``latlon_to_ned`` / ``ned_to_latlon`` implement **PX4's own map projection**:
a spherical azimuthal-equidistant projection with R = 6 371 000 m, the exact
math of PX4-Autopilot ``src/lib/geo/geo.cpp`` (``CONSTANTS_RADIUS_OF_EARTH``).

PX4's EKF defines the local NED frame by projecting every GPS fix through that
model, so any companion conversion crossing into or out of PX4 local NED MUST
use the same model. Using a WGS84 geodesic here instead (as this module did
until 2026-07-25) puts a pure scale error on every metre of distance from the
origin: at 13 °N the two models diverge by −0.51 cm per metre north and
+0.13 cm per metre east — measured as a north-walking placement residual in
the 2026-07-25 field bags (docs/FIELD_BUG_REPORT_2026-07-25.md, B6').

The WGS84 Karney geodesic remains available ONLY for true ground distance
between two lat/lon pairs (``geodesic_ground_distance_m``) — e.g. grading a
painted line against surveyed truth. Never use it for anything that produces
or consumes PX4 local-frame coordinates.

Centralized here so the ROS2 nodes, the FastAPI server and the analysis tools
all share one implementation of each model.
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

# PX4 geo.h:55 CONSTANTS_RADIUS_OF_EARTH — the sphere the EKF's local frame
# lives on. Not a physical Earth radius; a frame definition. Do not "improve"
# it to a WGS84 radius: matching PX4 exactly is the whole point.
PX4_EARTH_RADIUS_M = 6371000.0


def latlon_to_ned(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    """Convert lat/lon to PX4 local NED metres relative to an origin.

    Exact port of PX4 ``MapProjection::project()`` (geo.cpp): spherical
    azimuthal-equidistant projection at R = 6 371 000 m. This is the frame the
    EKF navigates in — see the module docstring for why WGS84 must not be used
    here.

    Args:
        lat: Target latitude (degrees).
        lon: Target longitude (degrees).
        origin_lat: Origin latitude (degrees).
        origin_lon: Origin longitude (degrees).

    Returns:
        (north_m, east_m) relative to origin, in PX4 local-frame metres.
    """
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat = math.radians(origin_lat)
    ref_lon = math.radians(origin_lon)

    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_ref = math.sin(ref_lat)
    cos_ref = math.cos(ref_lat)
    cos_d_lon = math.cos(lon_rad - ref_lon)

    arg = sin_ref * sin_lat + cos_ref * cos_lat * cos_d_lon
    arg = max(-1.0, min(1.0, arg))
    c = math.acos(arg)
    k = c / math.sin(c) if abs(c) > 1e-12 else 1.0

    north = k * (cos_ref * sin_lat - sin_ref * cos_lat * cos_d_lon) * PX4_EARTH_RADIUS_M
    east = k * cos_lat * math.sin(lon_rad - ref_lon) * PX4_EARTH_RADIUS_M
    return (north, east)


def ned_to_latlon(
    north: float,
    east: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    """Inverse of latlon_to_ned: PX4 local NED metres → lat/lon.

    Exact port of PX4 ``MapProjection::reproject()`` (geo.cpp), so it is the
    true inverse of ``latlon_to_ned`` and of the projection the EKF applied to
    build the local frame in the first place. Used to render a local-frame
    /path back into geo coordinates for the post-mission geo overlay
    (tools/analyze_mission.py §12).

    Args:
        north: PX4 local-frame metres north of the origin.
        east: PX4 local-frame metres east of the origin.
        origin_lat: Origin latitude (degrees).
        origin_lon: Origin longitude (degrees).

    Returns:
        (lat, lon) in degrees.
    """
    x_rad = float(north) / PX4_EARTH_RADIUS_M
    y_rad = float(east) / PX4_EARTH_RADIUS_M
    c = math.hypot(x_rad, y_rad)

    ref_lat = math.radians(origin_lat)
    ref_lon = math.radians(origin_lon)

    if c > 1e-12:
        sin_c = math.sin(c)
        cos_c = math.cos(c)
        lat_rad = math.asin(
            cos_c * math.sin(ref_lat) + (x_rad * sin_c * math.cos(ref_lat)) / c
        )
        lon_rad = ref_lon + math.atan2(
            y_rad * sin_c,
            c * math.cos(ref_lat) * cos_c - x_rad * math.sin(ref_lat) * sin_c,
        )
        return (math.degrees(lat_rad), math.degrees(lon_rad))
    return (float(origin_lat), float(origin_lon))


def geodesic_ground_distance_m(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
) -> float:
    """True ground distance between two lat/lon pairs (WGS84 Karney geodesic).

    This is the ONLY sanctioned use of the WGS84 model in this module: grading
    real-world separation (e.g. §8 miss distance, survey QA). It must never be
    used to build PX4 local-frame coordinates — that is ``latlon_to_ned``.

    Raises:
        ImportError: If geographiclib is not installed.
    """
    if not _HAS_GEOGRAPHICLIB:
        raise ImportError(
            "geographiclib is required for ground-distance measurement. "
            "Install: pip install geographiclib"
        )
    return float(Geodesic.WGS84.Inverse(lat1, lon1, lat2, lon2)["s12"])


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
