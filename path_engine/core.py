"""Core data models for the path planning engine."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
import math
import re
from typing import Optional


def dxf_arc_tangent(angle_deg: float) -> tuple[float, float]:
    """CCW travel tangent at DXF angle θ (0°=East), in (north, east).

    Single source of truth for the formula used by dxf_parser metadata and
    the entity extension preview: tangent = (cos θ, -sin θ). Verified against
    arc_waypoints() point ordering (Stage 7A audit).
    """
    a = math.radians(angle_deg)
    return (math.cos(a), -math.sin(a))


class SegmentType(IntEnum):
    """Spray state for a path segment."""
    MARK = 0      # Spray ON — draw/paint this segment
    TRANSIT = 1   # Spray OFF — fast travel between marks


@dataclass
class PathSegment:
    """One continuous geometry segment with spray state and speed.

    Attributes:
        segment_type: MARK (spray on) or TRANSIT (spray off, fast travel).
        points: NED waypoints as (north_m, east_m) tuples.
        speed: Target speed in m/s for this segment.
        segment_id: Integer ID matching DXF entity or CSV row group.
        source_entity: Human-readable label (e.g. "LINE_E042", "ARC_circle_1").
        metadata: Optional geometry metadata dict.  Keys injected by parsers:
            "geometry_type"  : "ARC" | "CIRCLE" (set by dxf_parser)
            "start_tangent"  : (north, east) unit vector at segment start
            "end_tangent"    : (north, east) unit vector at segment end
            "direction"      : "CCW" | "CW" (arc traversal direction)
            "reversed"       : True if optimizer reversed the point order
            "extension_role" : "pre" | "aft" (set on extension TRANSIT segments)
            "parent_source_entity": source of the parent MARK segment
    """
    segment_type: SegmentType = SegmentType.MARK
    points: list[tuple[float, float]] = field(default_factory=list)
    speed: float = 0.35
    segment_id: int = 0
    source_entity: str = ""
    metadata: dict = field(default_factory=dict)

    @property
    def length(self) -> float:
        """Total arc length in metres."""
        total = 0.0
        for i in range(1, len(self.points)):
            dx = self.points[i][0] - self.points[i - 1][0]
            dy = self.points[i][1] - self.points[i - 1][1]
            total += (dx * dx + dy * dy) ** 0.5
        return total


@dataclass
class DXFEntity:
    """Parsed DXF entity before planning.

    Attributes:
        entity_type: "LINE", "ARC", "CIRCLE", "LWPOLYLINE", "SPLINE",
                     "ELLIPSE", "POINT".
        layer: DXF layer name (used for MARK/TRANSIT classification).
        color: AutoCAD color index.
        entity_id: ezdxf handle string (e.g. "1A3").
        geometry: Dict with type-specific keys:
            LINE: start=(N,E), end=(N,E)
            ARC: center=(N,E), radius, start_angle, end_angle (degrees)
            CIRCLE: center=(N,E), radius
            LWPOLYLINE: vertices=[(N,E),...], closed=bool, bulges=[float,...]
            SPLINE: control_points=[(N,E),...], degree
            ELLIPSE: center=(N,E), major_axis=(dN,dE), ratio, start_param, end_param
            POINT: position=(N,E)
        unit_scale: DXF-to-metres conversion factor applied.
        is_mark_override: Operator spray decision (True=mark, False=transit).
            None means no override. An 'ignore' classification from
            layer_mapping always wins over the override — ignored entities
            must never be planned, overridden or not.
    """
    entity_type: str
    layer: str
    color: int = 7  # AutoCAD default white
    entity_id: str = ""
    geometry: dict = field(default_factory=dict)
    unit_scale: float = 0.01  # default: DXF units are centimetres
    is_mark_override: Optional[bool] = None

    def classify(self, layer_mapping: dict[str, str] | None = None) -> str:
        """Classify this entity as 'mark', 'transit', or 'ignore'."""
        base = self._classify_by_rules(layer_mapping)
        if base == "ignore":
            return "ignore"
        if self.is_mark_override is not None:
            return "mark" if self.is_mark_override else "transit"
        return base

    def _classify_by_rules(self, layer_mapping: dict[str, str] | None = None) -> str:
        if layer_mapping:
            for pattern, seg_type in layer_mapping.items():
                # Color match
                if pattern.lower().startswith("color:"):
                    color_val = pattern.split(":", 1)[1].strip().lower()
                    color_map = {
                        "red": 1, "yellow": 2, "green": 3, "cyan": 4,
                        "blue": 5, "magenta": 6, "white": 7, "black": 7,
                    }
                    target_color = None
                    if color_val.isdigit():
                        target_color = int(color_val)
                    elif color_val in color_map:
                        target_color = color_map[color_val]
                    if target_color is not None and self.color == target_color:
                        return seg_type.lower()
                else:
                    # Regex/substring match
                    try:
                        if re.search(pattern, self.layer, re.IGNORECASE):
                            return seg_type.lower()
                    except re.error:
                        if pattern.upper() in self.layer.upper():
                            return seg_type.lower()

        # Default rules
        upper = self.layer.upper()
        transit_keywords = ("TRANSIT", "TRAVEL", "MOVE", "RAPID")
        for kw in transit_keywords:
            if kw in upper:
                return "transit"
        return "mark"

    def is_mark(self, layer_mapping: dict[str, str] | None = None) -> bool:
        """Classify this entity as MARK (spray on) or TRANSIT (spray off)."""
        return self.classify(layer_mapping) != "transit"



@dataclass
class PlannedPath:
    """Full output of the path planning pipeline.

    Attributes:
        segments: Ordered list of PathSegments (MARK + TRANSIT).
        merged_waypoints: Single polyline for the /path topic.
        spray_flags: Parallel to merged_waypoints; True = spray ON.
        total_mark_length: Total metres of spray-on path.
        total_transit_length: Total metres of dead-heading.
        origin: (north_m, east_m) NED origin used for lat/lon conversion.
        alignment_metadata: Alignment stats/residuals from GPS/DXF reference points.
        planning_metadata: Counts/timings/sanity metadata from the planning run.
    """
    segments: list[PathSegment] = field(default_factory=list)
    merged_waypoints: list[tuple[float, float]] = field(default_factory=list)
    spray_flags: list[bool] = field(default_factory=list)
    total_mark_length: float = 0.0
    total_transit_length: float = 0.0
    origin: tuple[float, float] = (0.0, 0.0)
    alignment_metadata: dict = field(default_factory=dict)
    planning_metadata: dict = field(default_factory=dict)

    @property
    def num_waypoints(self) -> int:
        return len(self.merged_waypoints)

    @property
    def total_length(self) -> float:
        return self.total_mark_length + self.total_transit_length
