"""QGC WPL 110 .waypoints file reader.

Converts lat/lon waypoints to PX4 local NED metres via path_engine.ned
(PX4-spherical projection — B6', see that module's frame contract).

The home waypoint (current=1) is used as the NED origin.
All other waypoints are converted to metres North/East from home.
"""

from __future__ import annotations

import logging

log = logging.getLogger("path_engine.waypoints_parser")

from ..core import PathSegment, SegmentType
from ..ned import latlon_to_ned


def read_qgc_waypoints(filepath: str) -> list[tuple[float, float]]:
    """Read QGC WPL 110 .waypoints file and convert lat/lon to NED metres.

    Uses the home waypoint (current=1) as the NED origin.
    All mission waypoints converted to PX4 local-frame metres North/East of
    home via the PX4-spherical projection (path_engine.ned, B6').

    Returns:
        List of (north_m, east_m) tuples relative to home.
    """
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
                command = int(fields[3]) if len(fields) > 3 else 16
                lat = float(fields[8])
                lon = float(fields[9])
            except (ValueError, IndexError):
                continue

            if current == 1:
                home_lat, home_lon = lat, lon
            elif command == 16:  # NAV_WAYPOINT only
                wps.append((lat, lon))
            else:
                log.debug("Skipping non-WAYPOINT command %d", command)

    if home_lat is None:
        if wps:
            home_lat, home_lon = wps[0]
            wps = wps[1:]
        else:
            raise ValueError(f"No waypoints found in {filepath}")

    pts: list[tuple[float, float]] = []
    for lat, lon in wps:
        pts.append(latlon_to_ned(lat, lon, home_lat, home_lon))

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