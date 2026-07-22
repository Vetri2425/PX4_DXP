"""Georeferenced-DXF detection and projection to a local metric frame.

A DXF has no native CRS: coordinates are just numbers in "drawing units". When a
file carries raw WGS84 lat/lon in those fields (a common GPS/GIS export), every
metre-based operation downstream — densify, merge, extension run-ups, bounds —
silently corrupts, because a metre tolerance applied to angular degrees fuses a
whole 2 m square into a single point. See the 2026-07-18 test_1.dxf case: the
polyline collapsed to one waypoint in /preview.

This module detects that case and projects lat/lon → local ENU metres so the
rest of the pipeline (which assumes metres) works unchanged.

Method — equirectangular tangent plane about the survey centroid, scaled by the
WGS84 ellipsoid's LOCAL RADII OF CURVATURE at that origin:

    N = (lat - lat0) * m_per_deg_north      m_per_deg_north from M(lat0)
    E = (lon - lon0) * m_per_deg_east       m_per_deg_east  from N(lat0)*cos(lat0)

where M is the meridional radius of curvature and N the prime vertical radius:

    M(lat) = a(1 - e^2) / (1 - e^2 sin^2 lat)^(3/2)
    N(lat) = a / (1 - e^2 sin^2 lat)^(1/2)

Using the semi-major axis `a` for BOTH axes — as this module did until
2026-07-22 — is wrong on the NORTH axis, because the Earth is flattened: near
the equator the meridian is more tightly curved than `a` implies. The resulting
error is a pure scale factor, so it is invisible on small geometry and grows
linearly with run length:

    lat  0.00 deg   +0.674 %   -> +67 cm per 100 m
    lat 13.07 deg   +0.622 %   -> +62 cm per 100 m   (Chennai test site)
    lat 30.00 deg   +0.421 %   -> +42 cm per 100 m
    lat 45.00 deg   +0.169 %   -> +17 cm per 100 m

Verified against a field survey of the same line (Emlid Reach RS3, RTK FIX,
2026-07-18): the true WGS84 geodesic is 2.3255 m and the surveyor's own
projected grid reads 2.3264 m (+0.04 %, a normal grid scale factor). The old
code produced 2.3399 m (+0.62 %); this code reproduces the geodesic.

Beyond the radius fix this remains a tangent-plane approximation. Its residual
error is of order (extent / R)^2 * extent — sub-millimetre over a marking site —
so a full projection (UTM via pyproj) still buys nothing here, and pyproj is not
installed on the rover. The origin's lat/lon is returned so the local frame ties
back to GPS for GPS_SURVEYED placement.

Parsed geometry stores tuples as (north, east). For a geographic DXF that means
(latitude, longitude), because the parser maps DXF y->north and x->east and
INSUNITS leaves the values unscaled.
"""
from __future__ import annotations

import logging
import math
from typing import Optional

log = logging.getLogger(__name__)

# WGS84 defining parameters.
WGS84_A = 6378137.0            # semi-major axis, metres
WGS84_F = 1.0 / 298.257223563  # flattening
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)   # first eccentricity squared

# Kept as the semi-major axis for any external reader; NOT used as the north
# scale any more (see the module docstring for why that was wrong).
R_EARTH = WGS84_A

# Absolute-position coordinate keys (get the full origin shift + scale).
_POINT_KEYS = ("start", "end", "position", "center")
# Displacement/vector keys (scale only — no origin shift).
_VECTOR_KEYS = ("major_axis",)

# --- detection thresholds --------------------------------------------------
# A geographic DXF is separated from a metric one by how far the coordinates sit
# from the origin RELATIVE to the drawing's own extent. lat/lon values (13, 80)
# dwarf a local site's angular extent (~2e-5 deg for 2 m), giving a ratio in the
# millions; a metric drawing's coordinates are the same order as its extent
# (ratio ~1). 1000 is orders of magnitude clear of both — even a 1 km geographic
# site (~0.01 deg) at the smallest plausible offset still exceeds it, while no
# metric drawing near or far comes close.
_GEO_OFFSET_EXTENT_RATIO = 1000.0
# A local site is never a whole degree across (~111 km). Backstop against a
# pathological metric drawing that happens to be far from origin AND tiny.
_GEO_MAX_EXTENT_DEG = 1.0


def _absolute_points(entities) -> list[tuple[float, float]]:
    """Every absolute (lat, lon) candidate across all entities. Vectors excluded
    — a displacement says nothing about where the drawing sits."""
    pts: list[tuple[float, float]] = []
    for ent in entities:
        geom = ent.geometry or {}
        for key in _POINT_KEYS:
            v = geom.get(key)
            if isinstance(v, (tuple, list)) and len(v) >= 2:
                pts.append((float(v[0]), float(v[1])))
        for v in geom.get("vertices", []) or []:
            if isinstance(v, (tuple, list)) and len(v) >= 2:
                pts.append((float(v[0]), float(v[1])))
    return pts


def looks_geographic(points: list[tuple[float, float]]) -> tuple[bool, str]:
    """Decide whether (lat, lon) points are WGS84 geographic, with a reason.

    Coordinate-driven on purpose: INSUNITS lies (test_1.dxf declares metres but
    stores degrees) and GEODATA is usually absent on GPS exports, so the values
    themselves are the only trustworthy signal.
    """
    if len(points) < 2:
        return False, "too few points"
    lats = [p[0] for p in points]
    lons = [p[1] for p in points]
    if max(abs(v) for v in lats) > 90.0 or max(abs(v) for v in lons) > 180.0:
        return False, "out of lat/lon range (projected metres)"
    lat_span = max(lats) - min(lats)
    lon_span = max(lons) - min(lons)
    extent = max(lat_span, lon_span)
    if extent <= 0:
        return False, "degenerate extent"
    if extent >= _GEO_MAX_EXTENT_DEG:
        return False, f"extent {extent:.3f} too large for a local site in degrees"
    cen_lat = sum(lats) / len(lats)
    cen_lon = sum(lons) / len(lons)
    offset = max(abs(cen_lat), abs(cen_lon))
    ratio = offset / extent
    if ratio < _GEO_OFFSET_EXTENT_RATIO:
        return False, f"offset/extent {ratio:.1f} below geographic threshold"
    return True, f"lat/lon centroid ({cen_lat:.6f}, {cen_lon:.6f}), extent {extent:.6g} deg"


def metres_per_degree(lat0_deg: float) -> tuple[float, float]:
    """(north, east) metres per degree at *lat0_deg* on the WGS84 ellipsoid.

    North uses the meridional radius of curvature M, east the prime vertical N
    times cos(lat). Using the semi-major axis for north instead of M is a
    +0.62 % scale error at 13 deg latitude — see the module docstring.
    """
    lat0 = math.radians(lat0_deg)
    s = math.sin(lat0)
    w2 = 1.0 - WGS84_E2 * s * s
    w = math.sqrt(w2)
    m_meridional = WGS84_A * (1.0 - WGS84_E2) / (w2 * w)   # a(1-e^2)/(1-e^2 sin^2)^1.5
    n_prime_vertical = WGS84_A / w                          # a/(1-e^2 sin^2)^0.5
    per_rad = math.radians(1.0)
    return (m_meridional * per_rad, n_prime_vertical * per_rad * math.cos(lat0))


def _project_point(lat: float, lon: float, lat0: float, lon0: float,
                   mdeg_n: float, mdeg_e: float) -> tuple[float, float]:
    return ((lat - lat0) * mdeg_n, (lon - lon0) * mdeg_e)


def _project_vector(dlat: float, dlon: float,
                    mdeg_n: float, mdeg_e: float) -> tuple[float, float]:
    return (dlat * mdeg_n, dlon * mdeg_e)


def detect_and_project(entities) -> Optional[tuple[float, float]]:
    """If *entities* hold geographic (lat/lon) coordinates, project every
    coordinate to local ENU metres IN PLACE and return the origin (lat0, lon0).

    Returns None and leaves geometry untouched for a metric DXF, so the existing
    metric path is byte-for-byte unchanged.

    The origin is the survey centroid; the shape ends up centred on (0, 0) in
    metres. ``ent.geo_origin`` is stamped on each entity so GPS_SURVEYED
    placement can recover the WGS84 frame.
    """
    points = _absolute_points(entities)
    is_geo, reason = looks_geographic(points)
    if not is_geo:
        return None

    lats = [p[0] for p in points]
    lons = [p[1] for p in points]
    lat0 = sum(lats) / len(lats)
    lon0 = sum(lons) / len(lons)
    mdeg_n, mdeg_e = metres_per_degree(lat0)
    # Radii scale by the LATITUDE rate. A geographic CIRCLE/ARC is inherently
    # distorted E-W by cos(lat); scaling by the north rate is the standard local
    # approximation and is the undistorted axis.
    m_per_deg = mdeg_n

    log.warning(
        "Georeferenced DXF detected (%s) — projecting to local ENU metres "
        "about origin (%.8f, %.8f)", reason, lat0, lon0)

    for ent in entities:
        geom = ent.geometry or {}
        for key in _POINT_KEYS:
            v = geom.get(key)
            if isinstance(v, (tuple, list)) and len(v) >= 2:
                geom[key] = _project_point(float(v[0]), float(v[1]), lat0, lon0, mdeg_n, mdeg_e)
        if "vertices" in geom and geom["vertices"]:
            geom["vertices"] = [
                _project_point(float(v[0]), float(v[1]), lat0, lon0, mdeg_n, mdeg_e)
                if isinstance(v, (tuple, list)) and len(v) >= 2 else v
                for v in geom["vertices"]
            ]
        for key in _VECTOR_KEYS:
            v = geom.get(key)
            if isinstance(v, (tuple, list)) and len(v) >= 2:
                geom[key] = _project_vector(float(v[0]), float(v[1]), mdeg_n, mdeg_e)
        if "radius" in geom and isinstance(geom["radius"], (int, float)):
            geom["radius"] = float(geom["radius"]) * m_per_deg
        ent.geo_origin = (lat0, lon0)

    return (lat0, lon0)
