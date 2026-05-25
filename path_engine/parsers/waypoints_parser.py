"""QGC WPL 110 .waypoints file reader.

Converts lat/lon waypoints to NED metres using Karney geodesic
(GeographicLib, same method as arc generators).

The home waypoint (current=1) is used as the NED origin.
All other waypoints are converted to metres North/East from home.
"""

from __future__ import annotations

import math

try:
    from geographiclib.geodesic import Geodesic
    _HAS_GEOGRAPHICLIB = True
except ImportError:
    _HAS_GEOGRAPHICLIB = False

from ..core import PathSegment, SegmentType


def read_qgc_waypoints(filepath: str) -> list[tuple[float, float]]:
    """Read QGC WPL 110 .waypoints file and convert lat/lon to NED metres.

    Uses the home waypoint (current=1) as the NED origin.
    All mission waypoints converted to metres North/East from home
    using Karney geodesic on WGS84 ellipsoid.

    Returns:
        List of (north_m, east_m) tuples relative to home.
    """
    if not _HAS_GEOGRAPHICLIB:
        raise ImportError(
            "geographiclib is required for QGC .waypoints files. "
            "Install: pip install geographiclib"
        )

    geod = Geodesic.WGS84
    wps: list[tuple[float, float]] = []
    home_lat: float | None = None
    home_lon: float | None = None

    with open(filepath, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("QGC"):
                continue
            fields = line.split("\t")
            if len(fields) < 11:
                continue

            try:
                current = int(fields[1])
                lat = float(fields[8])
                lon = float(fields[9])
            except (ValueError, IndexError):
                continue

            if current == 1:
                home_lat, home_lon = lat, lon
            else:
                wps.append((lat, lon))

    if home_lat is None:
        if wps:
            home_lat, home_lon = wps[0]
            wps = wps[1:]
        else:
            raise ValueError(f"No waypoints found in {filepath}")

    pts: list[tuple[float, float]] = []
    for lat, lon in wps:
        result = geod.Inverse(home_lat, home_lon, lat, lon)
        dist = result["s12"]
        bearing_rad = math.radians(result["azi1"])
        north = dist * math.cos(bearing_rad)
        east = dist * math.sin(bearing_rad)
        pts.append((north, east))

    return pts


def read_qgc_waypoints_as_segment(
    filepath: str,
    segment_type: SegmentType = SegmentType.MARK,
    speed: float = 0.35,
) -> PathSegment:
    """Read QGC .waypoints and return a single PathSegment."""
    pts = read_qgc_waypoints(filepath)
    return PathSegment(
        segment_type=segment_type,
        points=pts,
        speed=speed,
        source_entity=f"waypoints:{filepath}",
    )