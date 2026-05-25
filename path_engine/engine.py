"""Path planning engine — orchestrates parse → plan → optimize → compensate → merge.

Usage:
    from path_engine import PathEngine

    engine = PathEngine()
    plan = engine.plan_file("soccer_field.dxf")
    print(f"Waypoints: {plan.num_waypoints}, Mark: {plan.total_mark_length:.1f}m")
"""

from __future__ import annotations

import math
import os

from .core import PlannedPath, PathSegment, SegmentType, DXFEntity
from .parsers import load_mission_file, load_mission_segments, parse_dxf, entities_to_segments
from .parsers.csv_parser import read_ned_csv_enhanced
from .parsers.waypoints_parser import read_qgc_waypoints_as_segment
from .planners.straight_line import densify_line, densify_segment
from .optimizers.segment_order import optimize_segment_order
from .spray import apply_spray_latency_compensation


class PathEngine:
    """Main orchestrator for the path planning pipeline.

    Pipeline: parse → segments → densify → optimize → compensate → merge

    The engine is a pure-Python library with no ROS2 dependency.
    It produces PlannedPath objects that can be published to /path
    by path_publisher_node or the FastAPI server.
    """

    def __init__(
        self,
        mark_spacing: float = 0.05,
        transit_spacing: float = 0.15,
        marking_speed: float = 0.35,
        transit_speed: float = 0.50,
        spray_on_latency: float = 0.10,
        spray_off_latency: float = 0.01,
        optimize_order: bool = True,
        compensate_spray: bool = True,
    ):
        self.mark_spacing = mark_spacing
        self.transit_spacing = transit_spacing
        self.marking_speed = marking_speed
        self.transit_speed = transit_speed
        self.spray_on_latency = spray_on_latency
        self.spray_off_latency = spray_off_latency
        self.optimize_order = optimize_order
        self.compensate_spray = compensate_spray

    def plan_file(
        self,
        filepath: str,
        layer_mapping: dict[str, str] | None = None,
        unit_scale: float | None = None,
        origin: tuple[float, float] = (0.0, 0.0),
        start_position: tuple[float, float] | None = None,
    ) -> PlannedPath:
        """Parse a file and run the full planning pipeline.

        Supports .dxf, .csv, and .waypoints files.
        Auto-detects format by extension.

        Args:
            filepath: Path to the mission file.
            layer_mapping: Dict mapping DXF layer patterns to "mark"/"transit"/"ignore".
            unit_scale: Metres per DXF unit (None = auto-detect from $INSUNITS).
            origin: (north_m, east_m) NED coordinate offset.
            start_position: (north_m, east_m) rover position for TSP optimization.
                           None → fallback to origin, then first segment start.

        Returns:
            PlannedPath with merged waypoints and spray flags.
        """
        ext = os.path.splitext(filepath)[1].lower()

        if ext == ".dxf":
            entities = parse_dxf(filepath, unit_scale=unit_scale)
            segments = entities_to_segments(
                entities, layer_mapping=layer_mapping,
                mark_speed=self.marking_speed, transit_speed=self.transit_speed,
            )
        else:
            # CSV and .waypoints: use the parser dispatcher
            segments = load_mission_segments(filepath)

        return self._plan_from_segments(segments, origin=origin, start_position=start_position)

    def plan_dxf_entities(
        self,
        entities: list[DXFEntity],
        layer_mapping: dict[str, str] | None = None,
        origin: tuple[float, float] = (0.0, 0.0),
        start_position: tuple[float, float] | None = None,
    ) -> PlannedPath:
        """Plan from pre-parsed DXF entities.

        Useful when the front-end has already parsed the DXF and
        the user has selected/reordered entities.

        Args:
            entities: List of DXFEntity objects.
            layer_mapping: Layer classification rules.
            origin: NED coordinate offset.
            start_position: Rover position for TSP optimization.

        Returns:
            PlannedPath with merged waypoints and spray flags.
        """
        segments = entities_to_segments(
            entities, layer_mapping=layer_mapping,
            mark_speed=self.marking_speed, transit_speed=self.transit_speed,
        )
        return self._plan_from_segments(segments, origin=origin, start_position=start_position)

    def plan_segments(
        self,
        segments: list[PathSegment],
        origin: tuple[float, float] = (0.0, 0.0),
        start_position: tuple[float, float] | None = None,
    ) -> PlannedPath:
        """Plan from pre-built PathSegments.

        Useful for programmatic segment construction.

        Args:
            segments: List of PathSegment objects.
            origin: NED coordinate offset.
            start_position: Rover position for TSP optimization.

        Returns:
            PlannedPath with merged waypoints and spray flags.
        """
        return self._plan_from_segments(segments, origin=origin, start_position=start_position)

    def _resolve_start_position(
        self,
        segments: list[PathSegment],
        origin: tuple[float, float],
        start_position: tuple[float, float] | None,
    ) -> tuple[float, float] | None:
        """Resolve start position for TSP using fallback chain.

        Fallback: explicit start_position → origin (if non-zero) → first segment start → None.
        """
        # A: Explicit start_position provided
        if start_position is not None:
            return start_position
        # B: origin is non-zero (georeferenced path)
        if origin != (0.0, 0.0):
            return origin
        # C: Use first segment's start point
        for seg in segments:
            if seg.points:
                return seg.points[0]
        return None

    def _plan_from_segments(
        self,
        segments: list[PathSegment],
        origin: tuple[float, float] = (0.0, 0.0),
        start_position: tuple[float, float] | None = None,
    ) -> PlannedPath:
        """Run the full pipeline on a list of segments.

        Pipeline:
          1. Densify (straight lines at appropriate spacing)
          2. Optimize segment order (nearest-neighbor TSP with endpoint reversal)
          3. Insert TRANSIT segments between disconnected MARK segments
          4. Apply spray latency compensation
          5. Merge into single polyline with spray flags

        Args:
            segments: Input segments (may be sparse).
            origin: (north, east) coordinate offset applied to all points.
            start_position: (north, east) rover position for TSP. None → fallback chain.

        Returns:
            PlannedPath ready for /path topic publication.
        """
        if not segments:
            return PlannedPath(origin=origin)

        # Step 1: Densify segments
        densified: list[PathSegment] = []
        for seg in segments:
            densified.append(densify_segment(seg, self.mark_spacing, self.transit_spacing))

        # Resolve start position for TSP: explicit → origin → first segment start
        resolved_start = self._resolve_start_position(densified, origin, start_position)

        # Step 2: Optimize segment order (nearest-neighbor TSP with endpoint reversal)
        if self.optimize_order and any(s.segment_type == SegmentType.MARK for s in densified):
            ordered = optimize_segment_order(
                densified,
                start_position=resolved_start,
                transit_speed=self.transit_speed,
            )
        else:
            ordered = densified

        # Step 3: Apply spray latency compensation to MARK segments
        if self.compensate_spray:
            compensated: list[PathSegment] = []
            for seg in ordered:
                compensated.append(apply_spray_latency_compensation(
                    seg,
                    spray_on_latency_s=self.spray_on_latency,
                    spray_off_latency_s=self.spray_off_latency,
                ))
            ordered = compensated

        # Step 4: Merge into single polyline with spray flags
        merged_waypoints: list[tuple[float, float]] = []
        spray_flags: list[bool] = []
        total_mark = 0.0
        total_transit = 0.0

        for seg in ordered:
            is_mark = seg.segment_type == SegmentType.MARK
            for i, pt in enumerate(seg.points):
                # Apply origin offset
                offset_pt = (pt[0] + origin[0], pt[1] + origin[1])
                merged_waypoints.append(offset_pt)
                spray_flags.append(is_mark)

                # Compute segment length
                if i > 0:
                    prev = seg.points[i - 1]
                    d = math.hypot(pt[0] - prev[0], pt[1] - prev[1])
                    if is_mark:
                        total_mark += d
                    else:
                        total_transit += d

        return PlannedPath(
            segments=ordered,
            merged_waypoints=merged_waypoints,
            spray_flags=spray_flags,
            total_mark_length=total_mark,
            total_transit_length=total_transit,
            origin=origin,
        )