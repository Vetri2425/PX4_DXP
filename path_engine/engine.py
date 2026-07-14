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
import time

from .core import (
    CURVED_GEOMETRY_TYPES,
    PlannedPath,
    PathSegment,
    SegmentType,
    DXFEntity,
)
from .parsers import load_mission_file, load_mission_segments, parse_dxf, entities_to_segments
from .parsers.csv_parser import read_ned_csv_enhanced
from .parsers.waypoints_parser import read_qgc_waypoints_as_segment
from .planners.straight_line import densify_segment
from .planners.extensions import (
    decompose_line_chain_to_edges,
    split_mark_segment_with_extensions,
)
from .planners.smooth import smooth_corners
from .optimizers.segment_order import optimize_segment_order
from .optimizers.shape_grouping import group_connected_segments
from .spray import apply_spray_latency_compensation
from .ned import (
    latlon_to_ned,
    dxf_to_ned_affine,
    apply_affine_transform,
    estimate_fit_scale,
)

# Survey-quality warning threshold on the *diagnostic* free-scale fit. Geometry is
# never scaled (the fit is rigid), so this only logs: it means the reference points
# disagree with the DXF's own dimensions by more than this fraction.
SURVEY_SCALE_WARN = 0.002  # 0.2%

log = logging.getLogger(__name__)

# Curved geometry is already smoothly discretised by the parser — re-running the
# corner rounder on it would distort the curve. Sourced from the shared taxonomy
# so "what counts as a curve" stays defined in exactly one place (core.py).
# LINE_CHAIN is deliberately absent: a grouped polygon still rounds its corners.
_SMOOTH_SKIP_GEOMETRY_TYPES = CURVED_GEOMETRY_TYPES


_SEGMENT_JOIN_TOL_M = 0.01


def _point_distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _insert_transit_connectors_between_segments(
    segments: list[PathSegment],
    transit_speed: float,
    gap_tol_m: float = _SEGMENT_JOIN_TOL_M,
) -> list[PathSegment]:
    """Make all inter-segment travel explicit.

    The final /path topic is a flat polyline, so adjacent segment endpoints that do
    not touch become real rover motion even if no PathSegment represents the
    connector. Insert explicit TRANSIT segments so the geometry, spray flags, segment
    metadata, and transit length all describe the same trajectory.

    With extensions enabled this is the ONLY routing pass: Step 3 withholds the
    optimizer's transit links (see its `insert_transits` arg) so that by the time we
    get here every run is already [PRE, MARK, AFT], and the connector we emit
    therefore spans AFT-tip -> next PRE-start. Previously the optimizer's links were
    generated first, against the ORIGINAL mark endpoints, and this pass could only
    reconcile them by driving the rover back over its own AFT — two 180 deg reversals
    per transition, with the run-up/run-out exactly cancelled.
    """
    if len(segments) < 2:
        return list(segments)

    stitched: list[PathSegment] = []
    connector_count = 0
    previous: PathSegment | None = None
    for seg in segments:
        if (
            previous is not None
            and previous.points
            and seg.points
            and _point_distance(previous.points[-1], seg.points[0]) > gap_tol_m
        ):
            connector_count += 1
            stitched.append(PathSegment(
                segment_type=SegmentType.TRANSIT,
                points=[previous.points[-1], seg.points[0]],
                speed=transit_speed,
                source_entity=f"transit:extension_join:{connector_count}",
                metadata={
                    "extension_connector": True,
                    "from_source_entity": previous.source_entity,
                    "to_source_entity": seg.source_entity,
                },
            ))
        stitched.append(seg)
        previous = seg
    return stitched


def _align_extension_boundaries_to_compensated_marks(
    segments: list[PathSegment],
) -> list[PathSegment]:
    """Keep PRE/MARK/AFT continuous after spray compensation shifts MARK ends."""
    aligned = list(segments)
    for i in range(len(aligned) - 1):
        prev = aligned[i]
        curr = aligned[i + 1]

        if (
            prev.segment_type == SegmentType.TRANSIT
            and curr.segment_type == SegmentType.MARK
            and prev.metadata.get("extension_role") == "pre"
            and prev.metadata.get("parent_source_entity") == curr.source_entity
            and prev.points
            and curr.points
        ):
            prev.points[-1] = curr.points[0]

        if (
            prev.segment_type == SegmentType.MARK
            and curr.segment_type == SegmentType.TRANSIT
            and curr.metadata.get("extension_role") == "aft"
            and curr.metadata.get("parent_source_entity") == prev.source_entity
            and prev.points
            and curr.points
        ):
            curr.points[0] = prev.points[-1]

    return aligned


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
        per_line_extensions: bool = False,
        corner_smooth_radius_m: float = 0.0,
        corner_smooth_arc_pts: int = 6,
        use_two_opt: bool = True,
        max_two_opt_segments: int = 80,
        group_shapes: bool = True,
        # Endpoint-coincidence tolerance for deciding whether two shape primitives
        # meet at the same junction. Stays at 5cm — real CAD exports leave gaps this
        # big between segments that are meant to connect. This is a *connectivity*
        # tolerance; it is NOT a point-culling tolerance (see _merge_chain).
        group_join_tol_m: float = 0.05,
        # ── Extension safety limits (see docs/upgrade_path/03_EXTENSION_GEOMETRY_DEFECTS.md)
        # E2: pre/aft_extension_m is only a CEILING. A 10 cm line was being given 97 cm
        # of run-up + run-out (7.4x the line it serves), so cap by line length and skip
        # lines too short to benefit at all.
        extension_max_line_fraction: float = 0.5,
        extension_min_line_length_m: float = 0.30,
        # E1: an extension continues its own line's direction and is laid down wherever
        # that lands — including on top of another line the rover paints. Ray-cast
        # against the rest of the drawing and stop short.
        extension_obstacle_clearance_m: float = 0.10,
        extension_min_useful_m: float = 0.10,
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
        if corner_smooth_radius_m < 0.0:
            raise ValueError(
                f"corner_smooth_radius_m must be >= 0.0, got {corner_smooth_radius_m}"
            )
        if corner_smooth_arc_pts < 2:
            raise ValueError(f"corner_smooth_arc_pts must be >= 2, got {corner_smooth_arc_pts}")
        if max_two_opt_segments < 0:
            raise ValueError(f"max_two_opt_segments must be >= 0, got {max_two_opt_segments}")
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
        self.per_line_extensions = per_line_extensions
        self.corner_smooth_radius_m = corner_smooth_radius_m
        self.corner_smooth_arc_pts = corner_smooth_arc_pts
        self.use_two_opt = use_two_opt
        self.max_two_opt_segments = max_two_opt_segments
        self.group_shapes = group_shapes
        self.group_join_tol_m = group_join_tol_m
        self.extension_max_line_fraction = extension_max_line_fraction
        self.extension_min_line_length_m = extension_min_line_length_m
        self.extension_obstacle_clearance_m = extension_obstacle_clearance_m
        self.extension_min_useful_m = extension_min_useful_m

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
        anchor: str = "drawing_origin",
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
            anchor: Which point the ``origin`` offset places at the rover.
                "drawing_origin" (default) anchors the DXF drawing origin (0,0)
                — the historical behavior. "first_waypoint" anchors the first
                driven merged waypoint (the PRE run-up point when extensions add
                one) so the rover starts driving forward from ``origin``.

        Returns:
            PlannedPath with merged waypoints and spray flags.
        """
        ext = os.path.splitext(filepath)[1].lower()

        if ext == ".dxf":
            entities = parse_dxf(filepath, unit_scale=unit_scale)
            detected_unit_scale = entities[0].unit_scale if entities else unit_scale
            segments = entities_to_segments(
                entities, layer_mapping=layer_mapping,
                mark_speed=self.marking_speed, transit_speed=self.transit_speed,
            )
        else:
            # CSV and .waypoints: use the parser dispatcher
            detected_unit_scale = None
            segments = load_mission_segments(filepath)

        plan = self._plan_from_segments(
            segments,
            origin=origin,
            start_position=start_position,
            origin_gps=origin_gps,
            rotation_deg=rotation_deg,
            ref_points_dxf=ref_points_dxf,
            ref_points_gps=ref_points_gps,
            close_loop=close_loop,
            anchor=anchor,
        )
        plan.planning_metadata["source"] = {
            "filepath": filepath,
            "extension": ext,
            "unit_scale_m_per_unit": detected_unit_scale,
        }
        return plan

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
        anchor: str = "drawing_origin",
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
            anchor: "drawing_origin" (default) or "first_waypoint" — see
                ``plan_file`` for semantics.

        Returns:
            PlannedPath with merged waypoints and spray flags.
        """
        segments = entities_to_segments(
            entities, layer_mapping=layer_mapping,
            mark_speed=self.marking_speed, transit_speed=self.transit_speed,
        )
        plan = self._plan_from_segments(
            segments,
            origin=origin,
            start_position=start_position,
            origin_gps=origin_gps,
            rotation_deg=rotation_deg,
            ref_points_dxf=ref_points_dxf,
            ref_points_gps=ref_points_gps,
            close_loop=close_loop,
            anchor=anchor,
        )
        plan.planning_metadata["source"] = {
            "extension": ".dxf",
            "unit_scale_m_per_unit": entities[0].unit_scale if entities else None,
        }
        return plan

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
        anchor: str = "drawing_origin",
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
            anchor: "drawing_origin" (default) or "first_waypoint" — see
                ``plan_file`` for semantics.

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
            anchor=anchor,
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
        anchor: str = "drawing_origin",
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
            ref_points_dxf: Control points in local-NED metres. They arrive
                pre-scaled from the /entities preview (DXF parser already applied
                $INSUNITS), same metric frame as segment geometry — no unit
                scaling here (re-applying unit_scale double-scales and corrupts fit).
            ref_points_gps: List of control points in WGS84 lat/lon.
            close_loop: True to close open loop paths.
            anchor: "drawing_origin" (default) anchors the DXF drawing origin
                (0,0) at ``origin``; "first_waypoint" anchors the first driven
                merged waypoint at ``origin`` (extension-aware auto-origin).

        Returns:
            PlannedPath ready for /path topic publication.
        """
        if not segments:
            return PlannedPath(origin=origin)

        t0 = time.perf_counter()
        input_segment_count = len(segments)
        input_waypoint_count = sum(len(seg.points) for seg in segments)

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

        # Refs already in local-NED metres — feed affine solve unchanged.
        metric_ref_points_dxf = list(ref_points_dxf) if ref_points_dxf else None

        if metric_ref_points_dxf and ref_points_gps and len(metric_ref_points_dxf) >= 2 and len(ref_points_gps) >= 2:
            # Multi-point least-squares alignment. Rotation is derived from the
            # point fit, so an explicit rotation_deg is ignored in this mode.
            if rotation_deg:
                log.warning(
                    "rotation_deg=%.3f ignored: rotation is derived from least-squares "
                    "fit of the %d reference points.", rotation_deg, len(metric_ref_points_dxf),
                )
            ref_gps_origin = origin_gps if origin_gps is not None else ref_points_gps[0]
            ref_ned_points = []
            for gps_pt in ref_points_gps:
                n, e = latlon_to_ned(gps_pt[0], gps_pt[1], ref_gps_origin[0], ref_gps_origin[1])
                ref_ned_points.append((n, e))

            # Rigid fit: rotation + translation only. Scale is pinned to 1.0 so the
            # DXF stays dimensionally authoritative (a 2 m square stays 2 m). The
            # free-scale value is kept as a *diagnostic* only — it is the signal for
            # a unit/frame mismatch, and it is never applied to geometry.
            scale_val, theta_val, offset_n_val, offset_e_val, residuals, rmse = dxf_to_ned_affine(
                metric_ref_points_dxf, ref_ned_points
            )
            fitted_scale = estimate_fit_scale(metric_ref_points_dxf, ref_ned_points)
            if abs(fitted_scale - 1.0) > SURVEY_SCALE_WARN:
                log.warning(
                    "alignment survey-quality: free-scale fit would be %.5f (%.2f%% off unity) — "
                    "reference points disagree with DXF dimensions by that much. Scale is LOCKED "
                    "to 1.0, so geometry is preserved; the disagreement is reported as rmse=%.4f m. "
                    "Re-check the survey if this is large.",
                    fitted_scale, (fitted_scale - 1.0) * 100.0, rmse,
                )
            alignment_meta = {
                "method": "least_squares",
                "scale": scale_val,          # applied scale — always 1.0 (rigid)
                "fitted_scale": fitted_scale,  # diagnostic — what free-scale WOULD have picked
                "rotation_deg": math.degrees(theta_val),
                "offset_n": offset_n_val,
                "offset_e": offset_e_val,
                "residuals": residuals,
                "rmse": rmse,
                "origin_gps": ref_gps_origin,
            }
            has_alignment = True

        elif metric_ref_points_dxf and ref_points_gps and len(metric_ref_points_dxf) == 1 and len(ref_points_gps) == 1:
            # Gap B: single reference point + operator heading. One point carries
            # no scale information (scale=1) and no residual, so this mode bypasses
            # the RMSE gate by definition. Translation snaps the rotated ref point
            # onto its NED target; rotation comes from rotation_deg.
            ref_gps_origin = origin_gps if origin_gps is not None else ref_points_gps[0]
            n, e = latlon_to_ned(
                ref_points_gps[0][0], ref_points_gps[0][1],
                ref_gps_origin[0], ref_gps_origin[1],
            )
            scale_val = 1.0
            theta_val = math.radians(rotation_deg)
            rp = metric_ref_points_dxf[0]
            rot_n = rp[0] * math.cos(theta_val) - rp[1] * math.sin(theta_val)
            rot_e = rp[0] * math.sin(theta_val) + rp[1] * math.cos(theta_val)
            offset_n_val = n - rot_n
            offset_e_val = e - rot_e
            alignment_meta = {
                "method": "single_point_heading",
                "scale": scale_val,
                "rotation_deg": rotation_deg,
                "offset_n": offset_n_val,
                "offset_e": offset_e_val,
                "rmse": 0.0,
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

        # Step 1: Smooth sparse MARK geometry before densification. Running this
        # after densification makes production corners look too short to round.
        sparse_waypoint_count = sum(len(seg.points) for seg in segments)
        smoothing_stats = {
            "enabled": self.corner_smooth_radius_m > 0.0,
            "radius_m": self.corner_smooth_radius_m,
            "arc_pts": self.corner_smooth_arc_pts,
            "segments_smoothed": 0,
            "vertices_skipped": 0,
            "waypoints_before": sparse_waypoint_count,
            "waypoints_after": sparse_waypoint_count,
        }
        if self.corner_smooth_radius_m > 0.0:
            smoothed: list[PathSegment] = []
            after = 0
            for seg in segments:
                geometry_type = str(seg.metadata.get("geometry_type", "")).upper()
                is_precurved = (
                    geometry_type in _SMOOTH_SKIP_GEOMETRY_TYPES
                    or seg.source_entity.startswith(("ARC_", "CIRCLE_", "ELLIPSE_", "SPLINE_"))
                )
                if seg.segment_type == SegmentType.MARK and len(seg.points) >= 3 and not is_precurved:
                    pts, skipped = smooth_corners(
                        seg.points,
                        self.corner_smooth_radius_m,
                        self.corner_smooth_arc_pts,
                    )
                    if pts != seg.points:
                        smoothing_stats["segments_smoothed"] += 1
                    smoothing_stats["vertices_skipped"] += skipped
                    new_seg = PathSegment(
                        segment_type=seg.segment_type,
                        points=pts,
                        speed=seg.speed,
                        segment_id=seg.segment_id,
                        source_entity=seg.source_entity,
                        metadata=dict(seg.metadata),
                    )
                    smoothed.append(new_seg)
                    after += len(pts)
                else:
                    smoothed.append(seg)
                    after += len(seg.points)
            segments = smoothed
            smoothing_stats["waypoints_after"] = after

        # Step 2: Densify segments  (E1 fix: renumbered — was duplicate "Step 2")
        densified: list[PathSegment] = []
        for seg in segments:
            densified.append(densify_segment(seg, self.mark_spacing, self.transit_spacing))
        densified_waypoint_count = sum(len(seg.points) for seg in densified)

        # Step 2b: Group connected line-like MARK primitives into shape runs.
        # Multi-shape DXFs arrive as loose LINE primitives; without grouping the
        # nearest-neighbour optimizer can interleave and reverse individual
        # edges, destroying shape-level traversal (square edges mixed into the
        # triangle, arbitrary shared-edge handoff). Chaining connected edges
        # into composite runs makes the optimizer order whole shapes and confine
        # TRANSIT links to the boundaries between them. Curved MARK entities
        # (circle/arc) are never absorbed, so they keep the smooth profile.
        grouping_stats = {"enabled": self.group_shapes}
        if self.group_shapes:
            before_segs = len(densified)
            densified = group_connected_segments(densified, tol=self.group_join_tol_m)
            grouping_stats.update({
                "segments_before": before_segs,
                "segments_after": len(densified),
                "runs_merged": before_segs - len(densified),
            })

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

        # Step 3: Optimize segment order (nearest-neighbor TSP with endpoint reversal)
        #
        # When extensions are enabled we deliberately DO NOT let the optimizer emit its
        # transit links here. Those links connect the ORIGINAL mark endpoints, but Step 4
        # is about to wrap each mark in PRE/AFT — after which the links no longer reach
        # anything, and the connector pass can only patch the gap by driving the rover
        # backwards over its own AFT (two 180 deg reversals per transition, run-up and
        # run-out exactly cancelled). Deferring transit generation until after extension
        # lets travel run AFT-tip -> next PRE-start directly.
        optimization_stats = {}
        if self.optimize_order and any(s.segment_type == SegmentType.MARK for s in densified):
            ordered = optimize_segment_order(
                densified,
                start_position=resolved_start,
                transit_speed=self.transit_speed,
                use_two_opt=self.use_two_opt,
                max_two_opt_segments=self.max_two_opt_segments,
                stats=optimization_stats,
                insert_transits=not self.enable_path_extensions,
            )
        else:
            ordered = densified
            optimization_stats = {
                "method": "disabled",
                "mark_segments": sum(1 for s in densified if s.segment_type == SegmentType.MARK),
                "deadhead_before_2opt_m": 0.0,
                "deadhead_after_2opt_m": 0.0,
                "two_opt_improvements": 0,
                "two_opt_skipped_reason": "optimization disabled",
                "max_two_opt_segments": self.max_two_opt_segments,
            }

        # Step 4: Apply drive extensions to line-like MARK segments.
        #
        # per_line=True explodes a composite line-chain (square / rect / polygon / L
        # perimeter) into its individual edges, and every edge becomes an INDEPENDENT
        # PRE -> MARK -> AFT pass. Each CAD line is then approached already settled
        # on-line and up to speed, marked dead straight, and exited — instead of the
        # rover pivoting through the corner mid-spray. That is the point of the mode.
        #
        # It necessarily costs travel: consecutive edges no longer touch, so the rover
        # drives out along edge N's AFT, turns, and comes back to edge N+1's PRE. Those
        # connectors are emitted below — AFTER extension, so they run AFT-tip ->
        # next-PRE-start directly. (Routing them BEFORE extension is what produced the
        # 180 deg double-back over the rover's own AFT — the d82317d field failure.
        # See the Step 3 note.)
        #
        # per_line=False keeps the chain whole: one continuous sprayed run, extended at
        # its true open ends only, corners sprayed straight through.
        if self.enable_path_extensions:
            # Decompose FIRST, so the obstacle set is built from the same geometry the
            # extensions spring from. (Building it from the un-decomposed chains would
            # make every edge's own parent chain an obstacle to itself.)
            decomposed: list[PathSegment] = []
            for seg in ordered:
                decomposed.extend(
                    decompose_line_chain_to_edges(seg)
                    if self.per_line_extensions and seg.segment_type == SegmentType.MARK
                    else [seg]
                )

            # E1: ray-cast every extension against ALL OTHER marked geometry so it can
            # never be laid down across a line the rover paints. Sibling edges of the
            # same shape ARE obstacles — an extension that dives back into its own
            # polygon is exactly the star_3x3m failure. An extension touching its
            # neighbour at the shared vertex hits at t~0 and is ignored by the ray test.
            mark_idx = [
                i for i, s in enumerate(decomposed)
                if s.segment_type == SegmentType.MARK and len(s.points) >= 2
            ]
            mark_polys = {i: list(decomposed[i].points) for i in mark_idx}

            extended: list[PathSegment] = []
            for i, seg in enumerate(decomposed):
                obstacles = [poly for j, poly in mark_polys.items() if j != i]
                parts = split_mark_segment_with_extensions(
                    seg,
                    pre_extension_m=self.pre_extension_m,
                    aft_extension_m=self.aft_extension_m,
                    transit_speed=self.transit_speed,
                    suppress_closed_loops=not self.per_line_extensions,
                    obstacles=obstacles,
                    max_line_fraction=self.extension_max_line_fraction,
                    min_line_length_m=self.extension_min_line_length_m,
                    obstacle_clearance_m=self.extension_obstacle_clearance_m,
                    min_useful_extension_m=self.extension_min_useful_m,
                )
                extended.extend(parts)

            # Travel between runs is inserted HERE, after extension, so each connector
            # spans AFT-tip -> next PRE-start. Step 3 withheld its own transit links
            # precisely so this is the only routing pass. Step 5b densifies them.
            ordered = _insert_transit_connectors_between_segments(
                extended,
                transit_speed=self.transit_speed,
            )

        # Step 5: Apply spray latency compensation — at REAL spray boundaries only.
        #
        # A grouped perimeter is decomposed into one MARK per edge, and those edges now
        # meet directly at the corners (see Step 4). The rover sprays straight through
        # such a corner — the spray never toggles there — so it is NOT a boundary.
        # Compensating it anyway pulls spray OFF 3.5 mm before the corner and back ON
        # 3.5 cm after it, which leaves an unpainted notch at every corner and makes the
        # planner stitch a small spray-off diagonal outside it. A 2 m square ended up
        # with 8 spray transitions instead of 2.
        #
        # A junction is interior iff both sides are MARK and they touch.
        if self.compensate_spray:
            def _is_interior_mark_junction(a: PathSegment, b: PathSegment) -> bool:
                return (
                    a.segment_type == SegmentType.MARK
                    and b.segment_type == SegmentType.MARK
                    and bool(a.points) and bool(b.points)
                    and _point_distance(a.points[-1], b.points[0]) <= _SEGMENT_JOIN_TOL_M
                )

            compensated: list[PathSegment] = []
            for i, seg in enumerate(ordered):
                prev_seg = ordered[i - 1] if i > 0 else None
                next_seg = ordered[i + 1] if i + 1 < len(ordered) else None
                compensated.append(apply_spray_latency_compensation(
                    seg,
                    spray_on_latency_s=self.spray_on_latency,
                    spray_off_latency_s=self.spray_off_latency,
                    compensate_start=not (
                        prev_seg is not None
                        and _is_interior_mark_junction(prev_seg, seg)
                    ),
                    compensate_end=not (
                        next_seg is not None
                        and _is_interior_mark_junction(seg, next_seg)
                    ),
                ))
            ordered = compensated
            if self.enable_path_extensions:
                ordered = _align_extension_boundaries_to_compensated_marks(ordered)
                ordered = _insert_transit_connectors_between_segments(
                    ordered,
                    transit_speed=self.transit_speed,
                )

        # Step 5b: Densify TRANSIT connectors created during ordering/extension.
        # Densification (Step 2) runs BEFORE ordering, so TRANSIT links inserted
        # by the optimizer, extension run-ups, and explicit extension-join
        # connectors can reach this point with only their two endpoints. Do this
        # after spray compensation so PRE/AFT segments are sampled from their
        # final, compensation-aligned endpoints.
        redensified: list[PathSegment] = []
        for seg in ordered:
            if seg.segment_type == SegmentType.TRANSIT and len(seg.points) >= 2:
                # PRE/AFT run-ups are colinear continuations of a mark line, so
                # sample them at MARK spacing (not the coarser transit spacing) —
                # the rover then tracks the run-up as tightly as the marked line
                # and is fully settled on-line before/after the spray boundary.
                #
                # E5: inter-run CONNECTORS get the same treatment. They were left at
                # transit_spacing (0.15 m), which put ~15 cm gaps on the short diagonal
                # the rover crosses BETWEEN TWO 135 deg PIVOTS — the sparsest sampling
                # at the hardest point on the whole path.
                is_extension = (
                    seg.metadata.get("extension_role") in ("pre", "aft")
                    or seg.metadata.get("extension_connector") is True
                )
                transit_spacing = (
                    self.mark_spacing if is_extension else self.transit_spacing
                )
                redensified.append(
                    densify_segment(seg, self.mark_spacing, transit_spacing)
                )
            else:
                redensified.append(seg)
        ordered = redensified

        # Step 6: Merge into single polyline with spray flags (and de-duplicate junctions)
        merged_waypoints: list[tuple[float, float]] = []
        spray_flags: list[bool] = []
        total_mark = 0.0
        total_transit = 0.0

        # Resolve the effective translation. With anchor="first_waypoint" the
        # origin offset is shifted so the first driven merged waypoint
        # (ordered[0].points[0] — the PRE run-up point when extensions add one)
        # lands exactly at `origin` (the rover pose), instead of anchoring the
        # drawing origin (0,0). Geometry shape is preserved either way since the
        # offset is uniform. Skipped when GPS/affine alignment already placed
        # the points in the target NED frame.
        effective_offset = origin
        if (
            anchor == "first_waypoint"
            and not has_alignment
            and ordered
            and ordered[0].points
        ):
            first_local = ordered[0].points[0]
            effective_offset = (
                origin[0] - first_local[0],
                origin[1] - first_local[1],
            )

        for seg in ordered:
            is_mark = seg.segment_type == SegmentType.MARK
            for i, pt in enumerate(seg.points):
                # Apply origin offset (only if not already aligned using GPS/affine)
                if has_alignment:
                    offset_pt = pt
                else:
                    offset_pt = (pt[0] + effective_offset[0], pt[1] + effective_offset[1])

                # Junction de-duplication: skip adjacent duplicate points within 1 cm
                if merged_waypoints:
                    d = math.hypot(offset_pt[0] - merged_waypoints[-1][0], offset_pt[1] - merged_waypoints[-1][1])
                    if d < 0.01 and spray_flags[-1] == is_mark:
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
                # E2 fix: the closing leg connects the END of the last segment
                # back to the path START — it is a deadhead (transit) move, not
                # a marking pass. Copying spray_flags[0] inherited spray ON
                # whenever the path started on a MARK segment (the common case),
                # which would paint the closing leg. Always close with spray OFF.
                merged_waypoints.append(merged_waypoints[0])
                spray_flags.append(False)
                # Account for the closing leg in the totals (transit)
                total_transit += d_start_end

        bbox = None
        if merged_waypoints:
            norths = [p[0] for p in merged_waypoints]
            easts = [p[1] for p in merged_waypoints]
            bbox = {
                "min_n": min(norths),
                "max_n": max(norths),
                "min_e": min(easts),
                "max_e": max(easts),
                "width_m": max(easts) - min(easts),
                "height_m": max(norths) - min(norths),
            }

        # E4: `bbox` above is the SWEPT area — it includes the run-ups/run-outs, which
        # reach beyond the drawing. An operator who draws a 3x3 m square and turns on
        # 0.5 m extensions gets a 4x4 m swept area, and until now nothing said so. On a
        # bounded site (wall, kerb, pad edge, parked vehicle) that is an out-of-bounds
        # excursion. Report the marked bbox alongside the swept one and the overshoot
        # between them, so the app and the operator can see it.
        extension_report = None
        if self.enable_path_extensions and merged_waypoints:
            mark_pts = [p for p, f in zip(merged_waypoints, spray_flags) if f]
            if mark_pts:
                mn = [p[0] for p in mark_pts]
                me = [p[1] for p in mark_pts]
                marked_bbox = {
                    "min_n": min(mn), "max_n": max(mn),
                    "min_e": min(me), "max_e": max(me),
                    "width_m": max(me) - min(me),
                    "height_m": max(mn) - min(mn),
                }
                overshoot_m = max(
                    marked_bbox["min_n"] - bbox["min_n"],
                    bbox["max_n"] - marked_bbox["max_n"],
                    marked_bbox["min_e"] - bbox["min_e"],
                    bbox["max_e"] - marked_bbox["max_e"],
                )
                clamped = sum(
                    1 for s in ordered if s.metadata.get("extension_clamped")
                )
                extension_report = {
                    "marked_bbox": marked_bbox,
                    "swept_bbox": bbox,
                    "max_overshoot_m": overshoot_m,
                    "clamped_extensions": clamped,
                    "limits": {
                        "max_line_fraction": self.extension_max_line_fraction,
                        "min_line_length_m": self.extension_min_line_length_m,
                        "obstacle_clearance_m": self.extension_obstacle_clearance_m,
                    },
                }
                log.info(
                    "extensions: swept area %.2f x %.2f m vs marked %.2f x %.2f m "
                    "(rover reaches %.2f m beyond the drawing); %d extension(s) clamped",
                    bbox["width_m"], bbox["height_m"],
                    marked_bbox["width_m"], marked_bbox["height_m"],
                    overshoot_m, clamped,
                )

        planning_time_s = time.perf_counter() - t0
        planning_meta = {
            "input_segments": input_segment_count,
            "input_waypoints": input_waypoint_count,
            "densified_waypoints": densified_waypoint_count,
            "final_segments": len(ordered),
            "final_waypoints": len(merged_waypoints),
            "bbox": bbox,
            "extensions": extension_report,
            "spacing": {
                "mark_m": self.mark_spacing,
                "transit_m": self.transit_spacing,
            },
            "smoothing": smoothing_stats,
            "grouping": grouping_stats,
            "optimization": optimization_stats,
            "planning_time_s": planning_time_s,
            "anchor": {
                "mode": anchor,
                "requested_origin": origin,
                "effective_offset": effective_offset,
            },
        }
        log.info(
            "planned path in %.3fs: segments %d -> %d, waypoints %d -> %d, length %.2fm",
            planning_time_s,
            input_segment_count,
            len(ordered),
            input_waypoint_count,
            len(merged_waypoints),
            total_mark + total_transit,
        )

        return PlannedPath(
            segments=ordered,
            merged_waypoints=merged_waypoints,
            spray_flags=spray_flags,
            total_mark_length=total_mark,
            total_transit_length=total_transit,
            origin=origin if not has_alignment else (0.0, 0.0),
            alignment_metadata=alignment_meta,
            planning_metadata=planning_meta,
        )
