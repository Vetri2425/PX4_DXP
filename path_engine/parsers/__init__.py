"""Path file parsers — auto-detect format by extension.

Supported formats:
  .waypoints → QGC WPL 110 (lat/lon → NED via Karney geodesic)
  .csv       → NED metres (backward-compatible 2-col + enhanced 6-col)
  .dxf       → DXF CAD file (ezdxf-based parsing)
"""

from __future__ import annotations

import os

from ..core import PathSegment, SegmentType
from .waypoints_parser import read_qgc_waypoints, read_qgc_waypoints_as_segment
from .csv_parser import read_ned_csv, read_ned_csv_enhanced
from .dxf_parser import parse_dxf, entities_to_segments


def load_mission_file(filepath: str) -> list[tuple[float, float]]:
    """Auto-detect file format and load waypoints.

    Returns a flat list of (north_m, east_m) tuples for backward
    compatibility with path_publisher_node.py.

    .waypoints → QGC WPL 110 (lat/lon → NED via Karney)
    .csv       → simple NED metres (2-column backward compatible)
    .dxf       → DXF file (all LINE entities concatenated)

    Raises:
        FileNotFoundError: If filepath does not exist.
        ImportError: If required library is missing (ezdxf, geographiclib).
    """
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"Mission file not found: {filepath}")

    ext = os.path.splitext(filepath)[1].lower()

    if ext == ".waypoints":
        return read_qgc_waypoints(filepath)
    elif ext == ".csv":
        return read_ned_csv(filepath)
    elif ext == ".dxf":
        entities = parse_dxf(filepath)
        segments = entities_to_segments(entities)
        # Flatten all segment points into a single polyline
        pts: list[tuple[float, float]] = []
        for seg in segments:
            pts.extend(seg.points)
        return pts
    else:
        # Try QGC format first, fall back to CSV
        try:
            return read_qgc_waypoints(filepath)
        except Exception:
            return read_ned_csv(filepath)


def load_mission_segments(filepath: str) -> list[PathSegment]:
    """Auto-detect file format and load as PathSegments with MARK/TRANSIT.

    Returns a list of PathSegment objects with spray state, speed,
    and segment IDs. For .csv files with 6 columns, this preserves
    the spray_on/speed/segment_id information.

    .waypoints → single MARK segment
    .csv       → segments grouped by (segment_id, spray_on)
    .dxf       → segments classified by layer
    """
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"Mission file not found: {filepath}")

    ext = os.path.splitext(filepath)[1].lower()

    if ext == ".waypoints":
        seg = read_qgc_waypoints_as_segment(filepath)
        return [seg]
    elif ext == ".csv":
        return read_ned_csv_enhanced(filepath)
    elif ext == ".dxf":
        entities = parse_dxf(filepath)
        return entities_to_segments(entities)
    else:
        # Default: treat as single MARK segment
        pts = load_mission_file(filepath)
        return [PathSegment(
            segment_type=SegmentType.MARK,
            points=pts,
            speed=0.35,
            source_entity=f"file:{filepath}",
        )]