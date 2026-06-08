"""Path planning engine — orchestrates parse → plan → optimize → compensate → merge.

Usage:
    from path_engine import PathEngine

    engine = PathEngine()
    plan = engine.plan_file("soccer_field.dxf")
    print(f"Waypoints: {plan.num_waypoints}, Mark: {plan.total_mark_length:.1f}m")
"""

from __future__ import annotations

import logging
import math
import os

from .core import PlannedPath, PathSegment, SegmentType, DXFEntity
from .parsers import load_mission_file, load_mission_segments, parse_dxf, entities_to_segments
from .parsers.csv_parser import read_ned_csv_enhanced
from .parsers.waypoints_parser import read_qgc_waypoints_as_segment
from .planners.straight_line import densify_line, densify_segment
from .planners.extensions import split_mark_segment_with_extensions
from .optimizers.segment_order import optimize_segment_order
from .spray import apply_spray_latency_compensation
from .ned import latlon_to_ned, dxf_to_ned_affine, apply_affine_transform

log = logging.getLogger(__name__)


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
        enable_path_extensions: bool = False,
        pre_extension_m: float = 0.5,
        aft_extension_m: float = 0.5,
    ):
        if mark_spacing <= 0:
            raise ValueError(f"mark_spacing must be > 0, got {mark_spacing}")
        if transit_spacing <= 0:
            raise ValueError(f"transit_spacing must be > 0, got {transit_spacing}")
        if marking_speed <= 0:
            raise ValueError(f"marking_speed must be > 0, got {marking_speed}")
        if transit_speed <= 0:
            raise ValueError(f"transit_speed must be > 0, got {transit_speed}")
        if pre_extension_m < 0.0:
            raise ValueError(f"pre_extension_m must be >= 0.0, got {pre_extension_m}")
        if aft_extension_m < 0.0:
            raise ValueError(f"aft_extension_m must be >= 0.0, got {aft_extension_m}")
        self.mark_spacing = mark_spacing
        self.transit_spacing = transit_spacing
        self.marking_speed = marking_speed
        self.transit_speed = transit_speed
        self.spray_on_latency = spray_on_latency
        self.spray_off_latency = spray_off_latency
        self.optimize_order = optimize_order
        self.compensate_spray = compensate_spray
        self.enable_path_extensions = enable_path_extensions
        self.pre_extension_m = pre_extension_m
        self.aft_extension_m = aft_extension_m

    def plan_file(
        self,
        filepath: str,
        layer_mapping: dict[str, str] | None = None,
        unit_scale: float | None = None,
        origin: tuple[float, float] = (0.0, 0.0),
        start_position: tuple[float, float] | None = None,
        origin_gps: tuple[float, float] | None = None,
        rotation_deg: float = 0.0,
        ref_points_dxf: list[tuple[float, float]] | None = None,
        ref_points_gps: list[tuple[float, float]] | None = None,
        close_loop: bool = False,
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
            origin_gps: GPS reference (lat, lon).
            rotation_deg: Rotation to align DXF north with true north (clockwise).
            ref_points_dxf: DXF coordinates of alignment points.
            ref_points_gps: GPS coordinates (lat, lon) of alignment points.
            close_loop: True to close open loop paths.

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

        return self._plan_from_segments(
            segments,
            origin=origin,
            start_position=start_position,
            origin_gps=origin_gps,
            rotation_deg=rotation_deg,
            ref_points_dxf=ref_points_dxf,
            ref_points_gps=ref_points_gps,
            close_loop=close_loop,
        )

    def plan_dxf_entities(
        self,
        entities: list[DXFEntity],
        layer_mapping: dict[str, str] | None = None,
        origin: tuple[float, float] = (0.0, 0.0),
        start_position: tuple[float, float] | None = None,
        origin_gps: tuple[float, float] | None = None,
        rotation_deg: float = 0.0,
        ref_points_dxf: list[tuple[float, float]] | None = None,
        ref_points_gps: list[tuple[float, float]] | None = None,
        close_loop: bool = False,
    ) -> PlannedPath:
        """Plan from pre-parsed DXF entities.

        Useful when the front-end has already parsed the DXF and
        the user has selected/reordered entities.

        Args:
            entities: List of DXFEntity objects.
            layer_mapping: Layer classification rules.
            origin: NED coordinate offset.
            start_position: Rover position for TSP optimization.
            origin_gps: GPS reference (lat, lon).
            rotation_deg: Rotation to align DXF north with true north.
            ref_points_dxf: DXF coordinates of alignment points.
            ref_points_gps: GPS coordinates (lat, lon) of alignment points.
            close_loop: True to close open loop paths.

        Returns:
            PlannedPath with merged waypoints and spray flags.
        """
        segments = entities_to_segments(
            entities, layer_mapping=layer_mapping,
            mark_speed=self.marking_speed, transit_speed=self.transit_speed,
        )
        return self._plan_from_segments(
            segments,
            origin=origin,
            start_position=start_position,
            origin_gps=origin_gps,
            rotation_deg=rotation_deg,
            ref_points_dxf=ref_points_dxf,
            ref_points_gps=ref_points_gps,
            close_loop=close_loop,
        )

    def plan_segments(
        self,
        segments: list[PathSegment],
        origin: tuple[float, float] = (0.0, 0.0),
        start_position: tuple[float, float] | None = None,
        origin_gps: tuple[float, float] | None = None,
        rotation_deg: float = 0.0,
        ref_points_dxf: list[tuple[float, float]] | None = None,
        ref_points_gps: list[tuple[float, float]] | None = None,
        close_loop: bool = False,
    ) -> PlannedPath:
        """Plan from pre-built PathSegments.

        Useful for programmatic segment construction.

        Args:
            segments: List of PathSegment objects.
            origin: NED coordinate offset.
            start_position: Rover position for TSP optimization.
            origin_gps: GPS reference (lat, lon).
            rotation_deg: Rotation to align DXF north with true north.
            ref_points_dxf: DXF coordinates of alignment points.
            ref_points_gps: GPS coordinates (lat, lon) of alignment points.
            close_loop: True to close open loop paths.

        Returns:
            PlannedPath with merged waypoints and spray flags.
        """
        return self._plan_from_segments(
            segments,
            origin=origin,
            start_position=start_position,
            origin_gps=origin_gps,
            rotation_deg=rotation_deg,
            ref_points_dxf=ref_points_dxf,
            ref_points_gps=ref_points_gps,
            close_loop=close_loop,
        )

    def _resolve_start_position(
        self,
        segments: list[PathSegment],
        origin: tuple[float, float],
        start_position: tuple[float, float] | None,
    ) -> tuple[float, float] | None:
        """Resolve start position for TSP in the segment (pre-offset) frame.

        start_position is in the offset (output) frame, so subtract origin
        to compare against raw segment points (which haven't been offset yet).
        Fallback: explicit start_position → first segment start → None.
        Never falls back to origin — it's in the wrong frame for TSP.
        """
        # A: Explicit start_position — de-offset into segment frame
        if start_position is not None:
            return (start_position[0] - origin[0], start_position[1] - origin[1])
        # B: Use first segment's start point (already in segment frame)
        for seg in segments:
            if seg.points:
                return seg.points[0]
        return None

    def _plan_from_segments(
        self,
        segments: list[PathSegment],
        origin: tuple[float, float] = (0.0, 0.0),
        start_position: tuple[float, float] | None = None,
        origin_gps: tuple[float, float] | None = None,
        rotation_deg: float = 0.0,
        ref_points_dxf: list[tuple[float, float]] | None = None,
        ref_points_gps: list[tuple[float, float]] | None = None,
        close_loop: bool = False,
    ) -> PlannedPath:
        """Run the full pipeline on a list of segments.

        Pipeline:
          1. Apply GPS or least-squares alignment/rotation transforms (if requested)
          2. Densify (straight lines at appropriate spacing)
          3. Optimize segment order (nearest-neighbor TSP with endpoint reversal)
          4. Insert TRANSIT segments between disconnected MARK segments
          5. Apply drive extensions (PRE/AFT TRANSIT) to line-like MARK segments
          6. Apply spray latency compensation to MARK segments only
          7. Merge into single polyline with spray flags (and de-duplicate junctions)

        Args:
            segments: Input segments (may be sparse).
            origin: (north, east) coordinate offset applied to all points.
            start_position: (north, east) rover position for TSP. None → fallback chain.
            origin_gps: WGS84 lat/lon origin coordinates.
            rotation_deg: Rotation to align DXF north with True north (clockwise).
            ref_points_dxf: List of control points in DXF coordinates.
            ref_points_gps: List of control points in WGS84 lat/lon.
            close_loop: True to close open loop paths.

        Returns:
            PlannedPath ready for /path topic publication.
        """
        if not segments:
            return PlannedPath(origin=origin)

        # Deep-copy input segments to avoid mutating caller's data
        segments = [
            PathSegment(
                segment_type=seg.segment_type,
                points=list(seg.points),
                speed=seg.speed,
                segment_id=seg.segment_id,
                source_entity=seg.source_entity,
                metadata=dict(seg.metadata),
            )
            for seg in segments
        ]

        alignment_meta = {}
        has_alignment = False
        scale_val, theta_val, offset_n_val, offset_e_val = 1.0, 0.0, 0.0, 0.0

        if ref_points_dxf and ref_points_gps and len(ref_points_dxf) >= 2 and len(ref_points_gps) >= 2:
            # Multi-point least-squares alignment. Rotation is derived from the
            # point fit, so an explicit rotation_deg is ignored in this mode.
            if rotation_deg:
                log.warning(
                    "rotation_deg=%.3f ignored: rotation is derived from least-squares "
                    "fit of the %d reference points.", rotation_deg, len(ref_points_dxf),
                )
            ref_gps_origin = origin_gps if origin_gps is not None else ref_points_gps[0]
            ref_ned_points = []
            for gps_pt in ref_points_gps:
                n, e = latlon_to_ned(gps_pt[0], gps_pt[1], ref_gps_origin[0], ref_gps_origin[1])
                ref_ned_points.append((n, e))
            
            scale_val, theta_val, offset_n_val, offset_e_val, residuals, rmse = dxf_to_ned_affine(
                ref_points_dxf, ref_ned_points
            )
            alignment_meta = {
                "method": "least_squares",
                "scale": scale_val,
                "rotation_deg": math.degrees(theta_val),
                "offset_n": offset_n_val,
                "offset_e": offset_e_val,
                "residuals": residuals,
                "rmse": rmse,
                "origin_gps": ref_gps_origin,
            }
            has_alignment = True

        elif origin_gps is not None:
            # Simple GPS origin + optional rotation alignment
            scale_val = 1.0
            theta_val = math.radians(rotation_deg)
            offset_n_val = 0.0
            offset_e_val = 0.0
            alignment_meta = {
                "method": "gps_origin",
                "scale": scale_val,
                "rotation_deg": rotation_deg,
                "offset_n": offset_n_val,
                "offset_e": offset_e_val,
                "origin_gps": origin_gps,
            }
            has_alignment = True

        if has_alignment:
            for seg in segments:
                seg.points = [
                    apply_affine_transform(pt, scale_val, theta_val, offset_n_val, offset_e_val)
                    for pt in seg.points
                ]
                # Also transform segment metadata tangents if they exist
                if "start_tangent" in seg.metadata and "end_tangent" in seg.metadata:
                    st = seg.metadata["start_tangent"]
                    et = seg.metadata["end_tangent"]
                    # Rotate the tangents
                    cos_t = math.cos(theta_val)
                    sin_t = math.sin(theta_val)
                    seg.metadata["start_tangent"] = (st[0] * cos_t - st[1] * sin_t, st[0] * sin_t + st[1] * cos_t)
                    seg.metadata["end_tangent"] = (et[0] * cos_t - et[1] * sin_t, et[0] * sin_t + et[1] * cos_t)

        # Step 1: Densify segments
        densified: list[PathSegment] = []
        for seg in segments:
            densified.append(densify_segment(seg, self.mark_spacing, self.transit_spacing))

        # Resolve start position for TSP:
        # If we applied alignment, segments' points are already in the target NED frame.
        # So we do not de-offset the start_position. Otherwise we de-offset it by origin.
        if has_alignment:
            resolved_start = start_position
            if resolved_start is None:
                for seg in densified:
                    if seg.points:
                        resolved_start = seg.points[0]
                        break
        else:
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

        # Step 3: Apply drive extensions to line-like MARK segments.
        if self.enable_path_extensions:
            extended: list[PathSegment] = []
            for seg in ordered:
                extended.extend(split_mark_segment_with_extensions(
                    seg,
                    pre_extension_m=self.pre_extension_m,
                    aft_extension_m=self.aft_extension_m,
                    transit_speed=self.transit_speed,
                ))
            ordered = extended

        # Step 4: Apply spray latency compensation to MARK segments
        if self.compensate_spray:
            compensated: list[PathSegment] = []
            for seg in ordered:
                compensated.append(apply_spray_latency_compensation(
                    seg,
                    spray_on_latency_s=self.spray_on_latency,
                    spray_off_latency_s=self.spray_off_latency,
                ))
            ordered = compensated

        # Step 5: Merge into single polyline with spray flags (and de-duplicate junctions)
        merged_waypoints: list[tuple[float, float]] = []
        spray_flags: list[bool] = []
        total_mark = 0.0
        total_transit = 0.0

        for seg in ordered:
            is_mark = seg.segment_type == SegmentType.MARK
            for i, pt in enumerate(seg.points):
                # Apply origin offset (only if not already aligned using GPS/affine)
                if has_alignment:
                    offset_pt = pt
                else:
                    offset_pt = (pt[0] + origin[0], pt[1] + origin[1])

                # Junction de-duplication: skip adjacent duplicate points within 1 cm
                if merged_waypoints:
                    d = math.hypot(offset_pt[0] - merged_waypoints[-1][0], offset_pt[1] - merged_waypoints[-1][1])
                    if d < 0.01:
                        continue

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

        # Optional loop closing
        if close_loop and merged_waypoints:
            d_start_end = math.hypot(merged_waypoints[-1][0] - merged_waypoints[0][0], merged_waypoints[-1][1] - merged_waypoints[0][1])
            if d_start_end > 0.01:
                merged_waypoints.append(merged_waypoints[0])
                spray_flags.append(spray_flags[0])
                # Account for the closing leg in the totals
                if spray_flags[0]:
                    total_mark += d_start_end
                else:
                    total_transit += d_start_end

        return PlannedPath(
            segments=ordered,
            merged_waypoints=merged_waypoints,
            spray_flags=spray_flags,
            total_mark_length=total_mark,
            total_transit_length=total_transit,
            origin=origin if not has_alignment else (0.0, 0.0),
            alignment_metadata=alignment_meta,
        )