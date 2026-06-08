"""Path planning engine for DYX marking rover.

Pure Python — zero ROS2 dependency. Produces NED waypoint paths from
DXF, CSV, and QGC .waypoints files for the RPP controller pipeline.

Pipeline:
  parse → entities_to_segments → densify → optimize → compensate → merge
"""
from .core import SegmentType, PathSegment, PlannedPath, DXFEntity
from .engine import PathEngine
from .validator import PathValidator

__all__ = [
    "SegmentType",
    "PathSegment",
    "PlannedPath",
    "DXFEntity",
    "PathEngine",
    "PathValidator",
]