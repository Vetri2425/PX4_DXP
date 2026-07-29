"""Path management endpoints (auth-protected).

GET    /api/paths              — list built-in + uploaded paths
GET    /api/path/{name}/preview — return local-NED points for display
GET    /api/path/{name}/entities — return per-entity DXF geometry for selection
POST   /api/path/{name}/entities — save per-entity DXF spray overrides
GET    /api/path/{name}/extensions — return saved DXF extension config
POST   /api/path/{name}/extensions — save DXF extension config
POST   /api/path/upload        — upload .waypoints, .csv, or .dxf
POST   /api/path/publish       — publish named path to /path topic
POST   /api/path/parse-dxf     — parse DXF file, return entity list
POST   /api/path/plan          — run full planning pipeline, return PlannedPath
POST   /api/path/{name}/align          — alignment only (coords + residuals)
GET    /api/path/{name}/segments       — verification segments (MARK/TRANSIT/ext)
POST   /api/path/{name}/plan-and-stage — heavy final plan + stage
POST   /api/path/plan-trajectory       — densify + stage an app-planned trajectory
GET    /api/path/staged/{mission_id}   — read a staged mission artifact
DELETE /api/path/{filename}    — delete uploaded file
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import tempfile
import time
import uuid
from typing import Optional

from fastapi import APIRouter, Depends, File, HTTPException, UploadFile

from auth import require_token
from config import (
    MAX_TRAJECTORY_POINTS,
    MAX_UPLOAD_BYTES,
    MISSION_DIR,
    RMSE_MAX,
    SCALE_FIT_TOLERANCE,
    SPRAY_DEFAULT_ON,
    SPRAY_LITERS_PER_METER,
    STAGING_DIR,
    STAGING_TTL_S,
)
from mission_placement import PlacementError
from models import (
    AlignRequest,
    AlignResponse,
    DXFEntitiesResponse,
    DXFEntityOverridesRequest,
    DXFEntityOverridesResponse,
    DXFEntityPreview,
    DXFEntityInfo,
    DXFParseResponse,
    EntityExtensionPreview,
    EntityExtensionRun,
    EntityOrderUpdateRequest,
    EntityOrderUpdateResponse,
    EntityTransitPreview,
    LoadMissionRequest,
    LoadedPathResponse,
    MissionSummary,
    PathExtensionConfig,
    PathExtensionConfigResponse,
    SurveyLineConfig,
    SurveyLineConfigResponse,
    PathPlanRequest,
    PathPlanResponse,
    PathPreviewBounds,
    PathPreviewResponse,
    PathPublishRequest,
    PathSegmentsResponse,
    PlanTrajectoryRequest,
    PlanTrajectoryResponse,
    RefPointResidual,
    SegmentInfo,
    SprayModeDashRequest,
    SprayModePointRequest,
    StagedMissionResponse,
    TrajectoryRunEcho,
)
from path_manager import UploadValidationError
from path_engine.core import PathSegment, SegmentType
from path_engine.engine import PathEngine
from path_engine.entity_order import apply_entity_order as _apply_entity_order_shared

log = logging.getLogger("server.routes.path")

# Two distinct routers so the URL structure is explicit and stable.
paths_router = APIRouter(prefix="/paths", tags=["path"],
                         dependencies=[Depends(require_token)])
path_router  = APIRouter(prefix="/path",  tags=["path"],
                         dependencies=[Depends(require_token)])


# ── Listing ───────────────────────────────────────────────────────────────────

@paths_router.get("")
async def list_paths():
    from main import path_mgr
    # list_paths() parses (and for DXF/CSV fully plans) every file in the
    # missions dir — seconds each. Offload to a thread so a dir full of DXFs
    # cannot block the event loop and freeze every other GET/POST behind it.
    try:
        paths = await asyncio.wait_for(
            asyncio.to_thread(path_mgr.list_paths),
            timeout=30.0,
        )
    except asyncio.TimeoutError:
        raise HTTPException(504, "Path listing timed out (30s limit)")
    return [p.model_dump() for p in paths]


# ── Preview ───────────────────────────────────────────────────────────────────

@path_router.get("/{name}/preview", response_model=PathPreviewResponse)
async def preview_path(name: str):
    # DXF previews run the full PathEngine planner — offload to a thread so a
    # heavy parse never blocks the event loop (telemetry WS, other endpoints).
    from main import path_mgr
    try:
        return await asyncio.wait_for(
            asyncio.to_thread(path_mgr.preview_path, name),
            timeout=15.0,
        )
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except ImportError as exc:
        raise HTTPException(500, str(exc))
    except asyncio.TimeoutError:
        raise HTTPException(504, "Preview timed out (15s limit)")
    except Exception as exc:
        raise HTTPException(422, f"Preview failed: {exc}")


# ── Entity-level DXF preview ──────────────────────────────────────────────────

def _ned_point(pt) -> dict[str, float]:
    return {"north": float(pt[0]), "east": float(pt[1])}


def _geo_origin_of(entities) -> Optional[list[float]]:
    """The DXF's WGS84 origin if georef projected it, else None.

    georef stamps the same (lat, lon) on every entity, so the first non-None
    wins. Returned as a JSON list [lat, lon] for the response.
    """
    for ent in entities:
        geo = getattr(ent, "geo_origin", None)
        if geo is not None:
            return [float(geo[0]), float(geo[1])]
    return None


def _arc_points(
    center: tuple[float, float],
    radius: float,
    start_angle_deg: float,
    end_angle_deg: float,
    min_points: int = 4,
    full_circle_points: int = 64,
) -> list[tuple[float, float]]:
    if radius < 1e-9:
        return [center]
    sweep_deg = (end_angle_deg - start_angle_deg) % 360.0
    if abs(sweep_deg) < 1e-9:
        sweep_deg = 360.0
    n_points = max(min_points, math.ceil(sweep_deg / 360.0 * full_circle_points) + 1)
    start = math.radians(start_angle_deg)
    sweep = math.radians(sweep_deg)
    cn, ce = center
    return [
        (cn + radius * math.sin(start + sweep * i / (n_points - 1)),
         ce + radius * math.cos(start + sweep * i / (n_points - 1)))
        for i in range(n_points)
    ]


def _subsample_points(
    pts: list[tuple[float, float]],
    max_points: int = 200,
) -> list[tuple[float, float]]:
    if len(pts) <= max_points:
        return pts
    if max_points < 2:
        return pts[:max_points]
    step = (len(pts) - 1) / (max_points - 1)
    return [pts[round(i * step)] for i in range(max_points)]


# Matches PathEngine.group_join_tol_m: two mark endpoints this close are the
# same chain junction, so neither is a free end eligible for an extension.
_EXTENSION_JUNCTION_TOL_M = 0.05
# Same value as engine.py's _COLLINEAR_DOT (~20 deg): at a junction this steep
# the two runs continue straight through each other, so a run-out there can only
# be retraced by the connector.
#
# Equal VALUE, deliberately weaker TEST. The planner (engine.py _retraces) knows
# its traversal order, so it takes a signed dot of exit->entry between segments
# it already knows are adjacent. The preview keeps DXF order while /plan reorders
# via TSP, so it knows neither: it compares |dot| against every other mark end.
# That is the conservative direction — the preview may suppress a run-up the
# planner would keep at an anti-collinear or non-adjacent junction. Both agree
# wherever it matters today (square corners are perpendicular, |dot|~0). Keeping
# the value in sync is necessary but NOT sufficient for preview==plan; changing
# either predicate needs a paired check, not just a matching constant.
_EXTENSION_COLLINEAR_DOT = 0.94


def _unit_dir(
    a: tuple[float, float], b: tuple[float, float]
) -> Optional[tuple[float, float]]:
    """Unit vector a->b, or None when the two points are coincident."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    h = math.hypot(dx, dy)
    if h < 1e-9:
        return None
    return (dx / h, dy / h)


def _extension_endpoint_freeness(
    runs: list[tuple[
        tuple[float, float], tuple[float, float],
        Optional[tuple[float, float]], Optional[tuple[float, float]],
    ]],
    tol: float = _EXTENSION_JUNCTION_TOL_M,
    per_line: bool = False,
) -> list[tuple[bool, bool]]:
    """Per mark entity (start, end, start_dir, end_dir), is each end FREE?

    Two policies, matching the two the planner actually runs:

    * per_line=False (chain ends) — an end is free only when it coincides with
      no OTHER mark entity's endpoint: extensions live at a chain's true open
      ends, never at internal corners or on closed loops. Mirrors
      split_mark_segment_with_extensions(suppress_closed_loops=True).

    * per_line=True — every CAD edge is an independent PRE→MARK→AFT pass, so a
      shared corner does NOT block a run-up; each side of a closed square gets
      its own. Only a *collinear* junction blocks, because there the run-out and
      the next run-in lie along the same line and the connector can only double
      straight back over them (the d82317d retrace field failure). Mirrors the
      planner's decompose_line_chain_to_edges() + suppress_closed_loops=False,
      whose _touches() is a DIRECTION test (_COLLINEAR_DOT), not a shared point.

    Collinearity is compared as |dot| so it holds regardless of which way each
    entity happens to be drawn or traversed. Doing this by endpoint geometry
    (not entity order) keeps preview and plan agreeing even though the preview
    keeps DXF order while the planner reorders via TSP.
    """
    def _collinear(d1, d2) -> bool:
        if d1 is None or d2 is None:
            return False
        return abs(d1[0] * d2[0] + d1[1] * d2[1]) > _EXTENSION_COLLINEAR_DOT

    def _blocked(pt, my_dir, skip_idx) -> bool:
        for j, (s, e, s_dir, e_dir) in enumerate(runs):
            if j == skip_idx:
                continue
            for other_pt, other_dir in ((s, s_dir), (e, e_dir)):
                if math.hypot(pt[0] - other_pt[0], pt[1] - other_pt[1]) > tol:
                    continue
                if not per_line:
                    return True  # chain ends: any junction blocks
                if _collinear(my_dir, other_dir):
                    return True  # per-line: only a retrace blocks
        return False

    freeness = []
    for i, (start, end, s_dir, e_dir) in enumerate(runs):
        if math.hypot(start[0] - end[0], start[1] - end[1]) <= tol:
            # Self-closed entity (circle / closed polyline): no linear free end
            # to run off, in either mode.
            freeness.append((False, False))
            continue
        freeness.append((not _blocked(start, s_dir, i), not _blocked(end, e_dir, i)))
    return freeness


def _entity_extension_edges(
    ent,
    tangent_pts: list[tuple[float, float]],
    per_line: bool,
) -> list[tuple[list[tuple[float, float]], Optional[tuple[float, float]], Optional[tuple[float, float]]]]:
    """The edges a mark entity's extensions attach to, matching the planner.

    In per-line mode a line-like polyline is split at its corners exactly as
    ``decompose_line_chain_to_edges`` does in the plan (engine.py, gated on
    ``per_line_extensions``), so each side is an independent PRE/MARK/AFT pass.
    This is what lets a single *closed* LWPOLYLINE (a square drawn as one
    polyline) grow the same per-side run-ups the mission actually drives —
    previously the preview treated it as one self-closed run and produced none,
    disagreeing with the plan.

    Everything else stays whole (one edge):
      - chain-ends mode (per_line=False) never decomposes, same as the planner;
      - ARC / CIRCLE / SPLINE / ELLIPSE are curved — decompose returns them
        unchanged and they keep their analytic tangents, never finite-difference.

    Returns ``[(points, start_dir, end_dir)]``; dirs are None when a direction
    cannot be inferred (caller then emits no run for that end).
    """
    from path_engine.core import PathSegment, SegmentType
    from path_engine.planners.extensions import (
        decompose_line_chain_to_edges,
        entity_extension_directions,
    )

    whole_dirs = entity_extension_directions(ent, tangent_pts)

    def _whole():
        s = whole_dirs[0] if whole_dirs else None
        e = whole_dirs[1] if whole_dirs else None
        return [(list(tangent_pts), s, e)]

    if not per_line or len(tangent_pts) < 3:
        return _whole()

    # geometry_type drives _is_line_like_segment: line-like polylines split at
    # corners, curved geometry is returned unchanged (single edge).
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=list(tangent_pts),
        source_entity=str(ent.entity_id),
        metadata={"geometry_type": str(ent.entity_type).upper()},
    )
    parts = decompose_line_chain_to_edges(seg)
    if len(parts) <= 1:
        # Not split (curved, or already a single straight edge) — keep analytic
        # tangents rather than a finite-difference approximation.
        return _whole()

    edges = []
    for p in parts:
        pts = p.points
        if len(pts) < 2:
            continue
        edges.append((
            list(pts),
            _unit_dir(pts[0], pts[1]),
            _unit_dir(pts[-2], pts[-1]),
        ))
    return edges or _whole()


def _entity_transit_previews(
    mark_endpoints: list[tuple[str, tuple[float, float], tuple[float, float]]],
) -> list[EntityTransitPreview]:
    """Straight no-spray connectors between consecutive MARK entities.

    *mark_endpoints* is (entity_id, entry_pt, exit_pt) per drawable MARK entity,
    in DXF/entity order. Callers must already have dropped entities with no
    preview points, so a degenerate entity cannot break the chain — its drawable
    neighbours still get connected, like the planner would.

    entry/exit are the extension TIPS when that end has a run-up, so a connector
    spans AFT-tip -> next PRE-start — matching the planner, which routes travel
    only AFTER extension (_insert_transit_connectors_between_segments). In
    per-line mode this is what makes a square's corners grow real connectors:
    the edges no longer touch once each has run off its own end.

    NOTE (known, pre-existing): the order here is DXF order, while /plan reorders
    via TSP. These connectors are therefore honest about GEOMETRY per junction,
    but not about which junctions the mission will actually drive.
    """
    transits = []
    for (from_id, _, start), (to_id, end, _) in zip(mark_endpoints, mark_endpoints[1:]):
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        if length < 1e-9:
            continue
        transits.append(EntityTransitPreview(
            from_entity_id=from_id,
            to_entity_id=to_id,
            length_m=round(length, 3),
            points=[_ned_point(start), _ned_point(end)],
        ))
    return transits


def _entity_length_m(ent) -> float:
    geom = ent.geometry
    etype = ent.entity_type
    if etype == "LINE":
        s = geom.get("start", (0.0, 0.0))
        e = geom.get("end", (0.0, 0.0))
        return math.hypot(s[0] - e[0], s[1] - e[1])
    if etype == "CIRCLE":
        return 2.0 * math.pi * geom.get("radius", 0.0)
    if etype == "ARC":
        sweep_deg = (geom.get("end_angle", 360.0) - geom.get("start_angle", 0.0)) % 360.0
        if abs(sweep_deg) < 1e-9:
            sweep_deg = 360.0
        return geom.get("radius", 0.0) * math.radians(sweep_deg)

    pts = _entity_preview_tuples(ent, max_points=10000)
    return sum(
        math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        for i in range(1, len(pts))
    )


def _entity_preview_tuples(ent, max_points: int = 200) -> list[tuple[float, float]]:
    geom = ent.geometry
    etype = ent.entity_type

    if etype == "LINE":
        pts = [geom.get("start", (0.0, 0.0)), geom.get("end", (0.0, 0.0))]
    elif etype == "POINT":
        pts = [geom.get("position", (0.0, 0.0))]
    elif etype == "CIRCLE":
        center = geom.get("center", (0.0, 0.0))
        radius = geom.get("radius", 0.0)
        pts = _arc_points(center, radius, 0.0, 360.0, min_points=65, full_circle_points=64)
    elif etype == "ARC":
        pts = _arc_points(
            geom.get("center", (0.0, 0.0)),
            geom.get("radius", 0.0),
            geom.get("start_angle", 0.0),
            geom.get("end_angle", 360.0),
        )
    elif etype == "LWPOLYLINE":
        vertices = list(geom.get("vertices", []))
        bulges = list(geom.get("bulges", [0.0] * len(vertices)))
        closed = bool(geom.get("closed", False))
        if any(abs(b) > 1e-9 for b in bulges):
            from path_engine.planners.arc_curve import densify_lwpolyline_bulge
            pts = densify_lwpolyline_bulge(
                vertices,
                bulges,
                closed,
                chord_error=0.05,
                min_spacing=0.05,
                max_spacing=0.50,
            )
        else:
            pts = vertices
            if closed and pts and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) > 1e-9:
                pts = pts + [pts[0]]
    elif etype in ("SPLINE", "ELLIPSE"):
        pts = list(geom.get("vertices", []))
    else:
        pts = []

    return _subsample_points([(float(n), float(e)) for n, e in pts], max_points=max_points)


# Known GCS / test fixture anchors that must never stage a surveyed mission.
_PLACEHOLDER_ORIGINS = frozenset({
    (13.0, 80.0),
})


def _assert_alignment_scale(alignment_meta: dict) -> None:
    """Reject an alignment whose reference points imply a gross unit/frame mismatch.

    The applied transform is RIGID (scale pinned to 1.0), so survey noise can no
    longer stretch the drawing — a 2 m square stays 2 m regardless of what the
    reference points say. What we still must reject is a *gross* mismatch, e.g. cm
    ref points against metre geometry (the historical scale≈100 double-scaling),
    which means the operator supplied the wrong data entirely rather than merely
    noisy data.

    So this gate reads ``fitted_scale`` — the free-scale value a similarity fit
    *would* have chosen — as a pure diagnostic. It is never applied to geometry.

    Moderate disagreement (a few %) is no longer a geometry hazard and is allowed
    through; it surfaces as a non-zero RMSE, which the RMSE gate handles. That gate
    is only meaningful now *because* scale is locked: a rigid 2-point fit has one
    residual degree of freedom and it is exactly the baseline-length mismatch,
    whereas a free-scale 2-point fit is exactly determined and always reports
    RMSE≈0 no matter how wrong the size is.

    single_point/gps_origin modes carry no fitted_scale and pass by definition.
    """
    scale = alignment_meta.get("fitted_scale", alignment_meta.get("scale", 1.0))
    if not math.isfinite(scale) or scale <= 0.0:
        raise HTTPException(
            422,
            f"Alignment produced a non-physical scale ({scale}). "
            "Re-verify the reference points.",
        )
    if abs(scale - 1.0) > SCALE_FIT_TOLERANCE:
        raise HTTPException(
            422,
            f"Reference points imply a scale of {scale:.4f} vs the DXF's own "
            f"dimensions — outside the safe range "
            f"[{1.0 - SCALE_FIT_TOLERANCE:.2f}, {1.0 + SCALE_FIT_TOLERANCE:.2f}]. "
            "This is a unit/frame mismatch between reference points and geometry "
            "(geometry is never rescaled to fit). Re-verify the reference points.",
        )


def _assert_origin_gps_usable(origin_gps) -> None:
    """Reject missing-bounds or known placeholder survey anchors."""
    if origin_gps is None:
        return
    try:
        lat = float(origin_gps[0])
        lon = float(origin_gps[1])
    except (TypeError, ValueError, IndexError):
        raise HTTPException(422, "origin_gps is missing or invalid")
    if not (math.isfinite(lat) and math.isfinite(lon)):
        raise HTTPException(422, "origin_gps contains non-finite values")
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise HTTPException(422, "origin_gps is outside valid latitude/longitude bounds")
    key = (round(lat, 6), round(lon, 6))
    if key in _PLACEHOLDER_ORIGINS or (lat, lon) in _PLACEHOLDER_ORIGINS:
        raise HTTPException(
            422,
            "origin_gps looks like a GCS placeholder/fixture coordinate — "
            "re-survey or omit the fake anchor before staging.",
        )


def _surveyed_origin_health():
    """(health_dict, reason_if_unavailable) for the live EKF local-frame origin."""
    from main import ros_node
    if ros_node is None:
        return None, "ROS bridge not available — the EKF origin cannot be verified"
    try:
        return ros_node.get_origin_health(), None
    except Exception as exc:  # noqa: BLE001 — never let a health probe 500 a route
        return None, f"EKF origin health probe failed: {exc}"


def _assert_origin_trusted_for_surveyed(mission_id: str) -> None:
    """Refuse a GPS_SURVEYED commitment when the EKF origin cannot be trusted.

    Fail CLOSED: an unavailable probe is a refusal, not a pass. A surveyed
    mission placed against a stale origin is displaced by the origin delta
    (measured 2.15 m / 2.25 m on 2026-07-27) and shows no symptom at all until
    the rover drives.
    """
    health, unavailable = _surveyed_origin_health()
    if unavailable is not None:
        raise HTTPException(503, f"Refusing surveyed mission {mission_id}: {unavailable}")
    if not health.get("trusted"):
        raise HTTPException(
            409,
            f"Refusing surveyed mission {mission_id}: EKF local-frame origin is "
            f"not trustworthy [{health.get('status')}] — {health.get('detail')} "
            "See GET /api/health/origin.",
        )


def _jsonable_geometry(geometry: dict) -> dict:
    def convert(value):
        if isinstance(value, tuple):
            return [convert(v) for v in value]
        if isinstance(value, list):
            return [convert(v) for v in value]
        if isinstance(value, dict):
            return {str(k): convert(v) for k, v in value.items()}
        return value

    return {str(k): convert(v) for k, v in geometry.items()}


# S5 — bound planning concurrency. asyncio.to_thread uses the default
# 32-thread executor while the unit caps the whole server at CPUQuota=200%:
# several concurrent plan requests each get a CPU slice and ALL hit the 15 s
# timeout, instead of one finishing. Two permits ≈ the CPU quota; excess
# requests queue INSIDE their route's asyncio.wait_for, so a queued request
# whose turn never comes still times out with the route's own named 504.
_PLAN_SEMAPHORE = asyncio.Semaphore(2)


async def _plan_in_thread(fn, /, *args, **kwargs):
    """to_thread for the heavy planners (plan_path / plan_segments) only.

    Cheap sidecar reads stay on plain to_thread — queueing a 5 ms preview
    behind a 10 s plan would be a regression, not a fix.
    """
    async with _PLAN_SEMAPHORE:
        return await asyncio.to_thread(fn, *args, **kwargs)


async def _sidecar_call(fn, *args, what: str, timeout: float = 5.0):
    """Run a blocking PathManager sidecar operation off the event loop.

    Maps the shared exception set to HTTP errors: 404 missing file,
    422 invalid input, 504 timeout, 500 anything else (server bug — never
    blame the client for it).
    """
    try:
        return await asyncio.wait_for(asyncio.to_thread(fn, *args), timeout=timeout)
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    except ImportError as exc:
        raise HTTPException(500, str(exc))
    except asyncio.TimeoutError:
        raise HTTPException(504, f"{what} timed out ({timeout:.0f}s limit)")
    except Exception as exc:
        raise HTTPException(500, f"{what} failed: {exc}")


def _apply_entity_order(entities: list, saved_order: list[str]) -> list:
    """Delegate to the shared helper in path_engine.entity_order.

    Kept as a thin wrapper so existing internal call sites in this module
    are undisturbed.  The shared helper is the single source of truth.
    """
    return _apply_entity_order_shared(entities, saved_order)


@path_router.get("/{name}/entities", response_model=DXFEntitiesResponse)
async def path_entities(name: str):
    """Return per-entity DXF preview geometry without full path planning."""
    from main import path_mgr

    safe = os.path.basename(name)
    fpath = os.path.join(MISSION_DIR, safe)
    if not os.path.isfile(fpath):
        raise HTTPException(404, f"Path not found: {name!r}")
    if os.path.splitext(fpath)[1].lower() != ".dxf":
        raise HTTPException(415, "Entity preview is only available for DXF files")

    try:
        entities = await asyncio.wait_for(
            asyncio.to_thread(path_mgr.parse_dxf, fpath),
            timeout=5.0,
        )
    except ImportError as exc:
        raise HTTPException(500, str(exc))
    except asyncio.TimeoutError:
        raise HTTPException(504, "Entity preview timed out (5s limit)")
    except Exception as exc:
        raise HTTPException(422, f"DXF entity preview failed: {exc}")

    # Apply saved entity ordering
    saved_order = await asyncio.to_thread(path_mgr.load_entity_order, safe)
    entities = _apply_entity_order(entities, saved_order)

    previews = []
    all_pts: list[tuple[float, float]] = []
    # Sidecar reads are tiny, but keep ALL filesystem work off the event loop
    # (same rule as the parse above — telemetry WS shares this loop).
    overrides = await asyncio.to_thread(path_mgr.load_entity_overrides, safe)
    extension_config_data = await asyncio.to_thread(path_mgr.load_extension_config, safe)
    extension_config = PathExtensionConfig(**extension_config_data)

    # Pre-pass: resolve each entity's preview/tangent points and mark state so
    # extension previews can be made connectivity-aware. Extensions only belong
    # at a chain's true open ends; internal junctions (square corners) and
    # closed loops get none — same rule split_mark_segment_with_extensions()
    # applies. Computed here (before the build loop) because freeness of one
    # entity's end depends on every other mark entity's endpoints.
    per_line = bool(extension_config.per_line)
    resolved: list[tuple] = []  # (ent, preview_pts, tangent_pts, default_is_mark, is_mark)
    # Freeness runs over EDGES, not whole entities: in per-line mode a polyline
    # is split at its corners (see _entity_extension_edges) so each side is its
    # own run — this is why a single closed square grows the same per-side
    # run-ups the plan drives. A lone LINE/ARC contributes one edge, so this is
    # a superset of the old per-entity behaviour, not a change to it.
    entity_edges: dict[int, list[tuple]] = {}   # order_index -> [(pts, sdir, edir)]
    edge_freeness_input: list[tuple] = []       # (start, end, sdir, edir) per edge
    edge_ref: list[tuple[int, int]] = []        # parallel: (order_index, local_edge_i)
    for idx, ent in enumerate(entities):
        preview_pts = _entity_preview_tuples(ent)
        # SPLINE/ELLIPSE previews are subsampled for payload size. Compute
        # extension directions/endpoints from dense flattened vertices so the
        # preview arrow follows the same endpoint tangent used by execution.
        tangent_pts = (
            _entity_preview_tuples(ent, max_points=10000)
            if ent.entity_type in ("SPLINE", "ELLIPSE")
            else preview_pts
        )
        default_is_mark = ent.is_mark()
        is_mark = overrides.get(ent.entity_id, default_is_mark)
        order_index = len(resolved)
        resolved.append((ent, preview_pts, tangent_pts, default_is_mark, is_mark))
        if is_mark and tangent_pts and len(tangent_pts) >= 2:
            edges = _entity_extension_edges(ent, tangent_pts, per_line)
            entity_edges[order_index] = edges
            for local_i, (pts, sdir, edir) in enumerate(edges):
                edge_ref.append((order_index, local_i))
                edge_freeness_input.append((pts[0], pts[-1], sdir, edir))

    freeness_list = _extension_endpoint_freeness(edge_freeness_input, per_line=per_line)
    edge_freeness: dict[tuple[int, int], tuple[bool, bool]] = {
        edge_ref[i]: freeness_list[i] for i in range(len(edge_ref))
    }

    # (entity_id, entry, exit) per drawable MARK EDGE, in order — endpoints only,
    # so large point lists aren't retained past the loop. entry/exit are the
    # extension TIPS where a run-up exists, so connectors span AFT-tip -> next
    # PRE-start like the planner routes them; they fall back to the mark vertex
    # when that end has no extension. Per-edge (not per-entity) so a split
    # polyline's sides get their corner connectors.
    from path_engine.planners.extensions import offset_point

    edge_endpoints: list[tuple[str, tuple[float, float], tuple[float, float]]] = []
    extension_runs: list[EntityExtensionRun] = []
    for order_index, (ent, preview_pts, tangent_pts, default_is_mark, is_mark) in enumerate(resolved):
        all_pts.extend(preview_pts)
        pre_m = extension_config.pre_extension_m
        aft_m = extension_config.aft_extension_m
        make_ext = extension_config.enabled and is_mark

        # Per-edge run-ups, matching the plan. For a lone LINE/ARC there is one
        # edge, so this reduces to the previous single PRE/AFT pair. For a split
        # polyline every side is handled independently.
        edges = entity_edges.get(order_index, [])
        entity_pre_pts: list = []   # first free PRE across this entity's edges
        entity_aft_pts: list = []   # last  free AFT across this entity's edges
        for local_i, (pts, sdir, edir) in enumerate(edges):
            sfree, efree = edge_freeness.get((order_index, local_i), (True, True))
            pre_pts: list = []
            aft_pts: list = []
            if make_ext and pre_m > 0 and sfree and sdir is not None:
                pre_pts = [_ned_point(offset_point(pts[0], sdir, -pre_m)), _ned_point(pts[0])]
            if make_ext and aft_m > 0 and efree and edir is not None:
                aft_pts = [_ned_point(pts[-1]), _ned_point(offset_point(pts[-1], edir, aft_m))]
            if pre_pts:
                extension_runs.append(EntityExtensionRun(
                    entity_id=ent.entity_id, role="pre", edge_index=local_i,
                    length_m=round(pre_m, 3), points=pre_pts))
                if not entity_pre_pts:
                    entity_pre_pts = pre_pts
            if aft_pts:
                extension_runs.append(EntityExtensionRun(
                    entity_id=ent.entity_id, role="aft", edge_index=local_i,
                    length_m=round(aft_m, 3), points=aft_pts))
                entity_aft_pts = aft_pts
            for ext_pt in pre_pts + aft_pts:
                all_pts.append((ext_pt["north"], ext_pt["east"]))
            # Per-EDGE connector endpoints. A connector runs from this edge's
            # exit tip to the NEXT edge's entry tip, so the sides of a split
            # polyline grow the corner connectors the plan drives — previously a
            # single polyline was one endpoint and produced none. Travel starts
            # at the run-up tip when present (rover drives out along AFT, turns,
            # comes back to the next PRE), else the mark vertex.
            edge_entry = (pre_pts[0]["north"], pre_pts[0]["east"]) if pre_pts else pts[0]
            edge_exit = (aft_pts[-1]["north"], aft_pts[-1]["east"]) if aft_pts else pts[-1]
            edge_endpoints.append((ent.entity_id, edge_entry, edge_exit))

        # Per-entity summary (single PRE/AFT pair). Authoritative, complete
        # geometry is `extensions[]` above; this stays for per-entity display and
        # is the entity's outermost run-up pair. Identical to the old field for a
        # single-edge entity.
        extension_preview = EntityExtensionPreview(
            enabled=bool(entity_pre_pts or entity_aft_pts),
            pre_length_m=pre_m if entity_pre_pts else 0.0,
            aft_length_m=aft_m if entity_aft_pts else 0.0,
            pre_points=entity_pre_pts,
            aft_points=entity_aft_pts,
        )
        geometry = ent.geometry
        if ent.entity_type in ("SPLINE", "ELLIPSE"):
            # Flattened spline/ellipse vertices duplicate preview_points
            # (same flattening, just unsubsampled) — strip them so a
            # spline-heavy file doesn't ship the shape twice.
            geometry = {k: v for k, v in geometry.items() if k != "vertices"}
        previews.append(DXFEntityPreview(
            entity_id=ent.entity_id,
            entity_type=ent.entity_type,
            layer=ent.layer,
            color=ent.color,
            default_is_mark=default_is_mark,
            is_mark=is_mark,
            order_index=order_index,
            length_m=round(_entity_length_m(ent), 3),
            geometry=_jsonable_geometry(geometry),
            preview_points=[_ned_point(pt) for pt in preview_pts],
            extension_preview=extension_preview,
        ))

    # Transit connectors join entity endpoints that are already in all_pts,
    # so bounds cover them without re-adding the points.
    transit_preview = _entity_transit_previews(edge_endpoints)

    bounds = None
    if all_pts:
        norths = [n for n, _ in all_pts]
        easts = [e for _, e in all_pts]
        bounds = PathPreviewBounds(
            north_min=min(norths),
            north_max=max(norths),
            east_min=min(easts),
            east_max=max(easts),
        )

    geo_origin = _geo_origin_of(entities)
    return DXFEntitiesResponse(
        name=safe,
        num_entities=len(previews),
        is_geographic=geo_origin is not None,
        geo_origin=geo_origin,
        bounds=bounds,
        extension_config=extension_config,
        transit_preview=transit_preview,
        extensions=extension_runs,
        entities=previews,
    )


@path_router.post("/{name}/entities/order", response_model=EntityOrderUpdateResponse)
async def update_entity_order(name: str, req: EntityOrderUpdateRequest):
    """Persist entity execution order for a DXF file."""
    from main import path_mgr

    safe = os.path.basename(name)
    fpath = os.path.join(MISSION_DIR, safe)
    if not os.path.isfile(fpath):
        raise HTTPException(404, f"Path not found: {name!r}")
    if os.path.splitext(fpath)[1].lower() != ".dxf":
        raise HTTPException(415, "Entity ordering is only available for DXF files")

    # Parse DXF to get valid entity IDs
    entities = await _sidecar_call(
        path_mgr.parse_dxf, fpath,
        what="Parsing DXF for entity order validation",
    )
    # The order sequences DRIVABLE shapes only. POINT entities (survey markers)
    # carry no traversable path and the planner drops them; entities on ignore
    # layers (DIM/DEFPOINTS/...) are never driven either. The client orders the
    # drawable line/curve shapes and legitimately omits these, so requiring them
    # in a "full order" would reject every valid order for a DXF that has any
    # survey points — which is exactly what happened for a georeferenced square
    # (one LWPOLYLINE + 5 POINTs). Exclude them from the contract.
    orderable = [
        ent for ent in entities
        if ent.entity_type != "POINT" and ent.classify() != "ignore"
    ]
    valid_set = {ent.entity_id for ent in orderable}
    posted = req.entity_order
    posted_set = set(posted)

    # Full-order contract over the ORDERABLE set: no dup, no unknown, and every
    # orderable entity present (so the traversal sequence stays deterministic).
    if len(posted) != len(posted_set):
        raise HTTPException(422, "Duplicate entity IDs in entity_order")

    unknown = posted_set - valid_set
    missing = valid_set - posted_set

    if unknown:
        raise HTTPException(422, f"Unknown entity IDs: {sorted(unknown)}")
    if missing:
        raise HTTPException(422, f"Missing entity IDs: {sorted(missing)}")

    await asyncio.to_thread(path_mgr.save_entity_order, safe, posted)

    return EntityOrderUpdateResponse(
        name=safe,
        num_entities=len(posted),
        entity_order=list(posted),
    )


@path_router.post("/{name}/entities", response_model=DXFEntityOverridesResponse)
async def save_path_entity_overrides(name: str, req: DXFEntityOverridesRequest):
    """Persist per-entity spray ON/OFF decisions for a DXF file."""
    from main import path_mgr

    overrides = {item.entity_id: item.is_mark for item in req.overrides}
    num_overrides = await _sidecar_call(
        path_mgr.save_entity_overrides, name, overrides,
        what="Saving entity overrides",
    )
    return DXFEntityOverridesResponse(
        name=os.path.basename(name),
        num_overrides=num_overrides,
    )


def _load_extension_config_checked(path_mgr, name: str) -> dict:
    """Blocking helper: validate the DXF exists, then load its config."""
    safe = os.path.basename(name)
    fpath = os.path.join(MISSION_DIR, safe)
    if not os.path.isfile(fpath):
        raise FileNotFoundError(f"Path not found: {name!r}")
    # A16: the pre-line CSV product needs extensions too — they are what move
    # the entry transient and the terminal shutoff OFF the painted line. Mirror
    # PathManager._require_extendable rather than re-deriving the rule.
    if os.path.splitext(fpath)[1].lower() != ".dxf" and not path_mgr._is_survey_csv(safe):
        raise ValueError(
            "Path extensions are only configurable for DXF files and survey CSVs"
        )
    return path_mgr.load_extension_config(safe)


@path_router.get("/{name}/extensions", response_model=PathExtensionConfigResponse)
async def get_path_extensions(name: str):
    """Return saved PRE/AFT extension config for a DXF file."""
    from main import path_mgr

    config = await _sidecar_call(
        _load_extension_config_checked, path_mgr, name,
        what="Loading extension config",
    )
    return PathExtensionConfigResponse(
        name=os.path.basename(name), saved=True, **config,
    )


@path_router.post("/{name}/extensions", response_model=PathExtensionConfigResponse)
async def save_path_extensions(name: str, req: PathExtensionConfig):
    """Persist PRE/AFT extension config for a DXF file."""
    from main import path_mgr

    config = await _sidecar_call(
        path_mgr.save_extension_config,
        name, req.enabled, req.pre_extension_m, req.aft_extension_m, req.per_line,
        what="Saving extension config",
    )
    return PathExtensionConfigResponse(
        name=os.path.basename(name),
        saved=True,
        **config,
    )


@path_router.get("/{name}/line-config", response_model=SurveyLineConfigResponse)
async def get_line_config(name: str):
    """Return saved survey-line reconstruction settings for a CSV file."""
    from main import path_mgr

    config = await _sidecar_call(
        path_mgr.load_line_config, name, what="Loading line config",
    )
    return SurveyLineConfigResponse(
        name=os.path.basename(name), saved=True, **config,
    )


@path_router.post("/{name}/line-config", response_model=SurveyLineConfigResponse)
async def save_line_config(name: str, req: SurveyLineConfig):
    """Persist survey-line reconstruction settings for a CSV file.

    Applies to preview, plan and load alike, so raising the corner radius
    changes what the operator sees before it changes what the rover drives.
    """
    from main import path_mgr

    config = await _sidecar_call(
        path_mgr.save_line_config,
        name, req.fillet_corners_m, req.fit_arcs_max_dev_m,
        what="Saving line config",
    )
    return SurveyLineConfigResponse(
        name=os.path.basename(name), saved=True, **config,
    )


# ── Point-mission CSV ingest (mobile app) ───────────────────────────────────────
# The app's CSV import posts here BEFORE staging: these routes only parse + return
# staged-ready points (they do not touch the controller). The frontend then feeds
# the returned point_mission_points into POST /{name}/plan-and-stage, which bridges
# them onto /path as must-hit vertices (see plan_and_stage). point_ingest lives in
# ../src alongside the ROS nodes, so add it to sys.path lazily like the node code.

def _import_point_ingest():
    import sys
    from pathlib import Path as _FsPath

    src = _FsPath(__file__).resolve().parents[2] / "src"
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))
    import point_ingest  # noqa: E402
    return point_ingest


@path_router.post("/parse-point-csv")
async def parse_point_csv(file: UploadFile = File(...)):
    """Parse a point-mission CSV (north,east[,dwell_s[,mark]]) into staged-ready points.

    NED metres, anchor-free. Returns the point list only — stage via
    /{name}/plan-and-stage. Dwell policy uses point_ingest's field defaults
    (2 s default, 60 s max); the mission's actual dwell is set at stage time from
    PathPlanRequest.point_dwell_s.
    """
    pi = _import_point_ingest()
    # Read up to MAX_UPLOAD_BYTES + 1 to detect oversize (same cap as /upload)
    raw = await file.read(MAX_UPLOAD_BYTES + 1)
    if len(raw) > MAX_UPLOAD_BYTES:
        raise HTTPException(413, f"File exceeds {MAX_UPLOAD_BYTES} bytes")
    content = raw.decode("utf-8", errors="replace")
    try:
        points = pi.parse_point_csv_text(content)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    return {
        "num_points": len(points),
        "point_mission_points": pi.points_to_staged_dict(points),
    }


@path_router.post("/parse-point-gps-csv")
async def parse_point_gps_csv(file: UploadFile = File(...)):
    """Parse a GPS point-mission CSV (lat,lon[,dwell_s][,mark]) into staged-ready points.

    The first data row is the survey anchor; every row is projected to
    anchor-relative NED metres (Karney geodesic). Returns num_points, anchor,
    point_source_frame="GPS_SURVEYED" and point_mission_points — the exact shape
    the mobile app hands back to /{name}/plan-and-stage.
    """
    pi = _import_point_ingest()
    # Read up to MAX_UPLOAD_BYTES + 1 to detect oversize (same cap as /upload)
    raw = await file.read(MAX_UPLOAD_BYTES + 1)
    if len(raw) > MAX_UPLOAD_BYTES:
        raise HTTPException(413, f"File exceeds {MAX_UPLOAD_BYTES} bytes")
    content = raw.decode("utf-8", errors="replace")
    try:
        parsed = pi.parse_point_gps_csv_text(content)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    except ImportError as exc:  # geographiclib missing on the host
        raise HTTPException(500, str(exc))
    return pi.gps_point_mission_parse_payload(parsed)


# ── Upload ────────────────────────────────────────────────────────────────────

@path_router.post("/upload")
async def upload_path(file: UploadFile = File(...)):
    from main import path_mgr
    # Read up to MAX_UPLOAD_BYTES + 1 to detect oversize
    content = await file.read(MAX_UPLOAD_BYTES + 1)
    if len(content) > MAX_UPLOAD_BYTES:
        raise HTTPException(413, f"File exceeds {MAX_UPLOAD_BYTES} bytes")
    try:
        saved = path_mgr.save_uploaded(file.filename or "", content)
    except UploadValidationError as exc:
        raise HTTPException(415, str(exc))
    return {"saved": saved, "size": len(content)}


# ── Publish ────────────────────────────────────────────────────────────────────

@path_router.post("/publish")
async def publish_path(req: PathPublishRequest):
    from main import ros_node, path_mgr
    if ros_node is None:
        raise HTTPException(503, "ROS node not ready")
    name = req.name or req.file
    if not name:
        raise HTTPException(400, "Provide name or file")
    try:
        pts = path_mgr.load_path(name)
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    spray_flags: list[bool] | None = None
    try:
        preview = path_mgr.preview_path(name)
        spray_flags = [bool(wp.spray) for wp in preview.waypoints]
    except Exception:
        spray_flags = [SPRAY_DEFAULT_ON] * len(pts)
    ros_node.publish_path(pts, frame_id=req.frame_id, spray_flags=spray_flags)
    return {"published": name, "num_points": len(pts)}


# ── DXF Parse ─────────────────────────────────────────────────────────────────

@path_router.post("/parse-dxf")
async def parse_dxf_file(file: UploadFile = File(...)):
    """Upload and parse a DXF file, returning entity summaries."""
    from main import path_mgr

    content = await file.read(MAX_UPLOAD_BYTES + 1)
    if len(content) > MAX_UPLOAD_BYTES:
        raise HTTPException(413, f"File exceeds {MAX_UPLOAD_BYTES} bytes")

    filename = file.filename or "upload.dxf"
    ext = os.path.splitext(filename)[1].lower()
    if ext != ".dxf":
        raise HTTPException(415, f"Expected .dxf file, got {ext!r}")

    # Write to temp file first — only persist to missions dir on successful parse.
    # Create the temp IN MISSION_DIR so the final os.replace is same-filesystem:
    # on the Jetson /tmp is a separate tmpfs, and a cross-device os.replace raises
    # EXDEV ("Invalid cross-device link"), which would break every DXF upload.
    os.makedirs(MISSION_DIR, exist_ok=True)
    safe = os.path.basename(filename)
    tmp = tempfile.NamedTemporaryFile(suffix=".dxf", delete=False, dir=MISSION_DIR)
    try:
        tmp.write(content)
        tmp.close()
        fpath = tmp.name

        from path_engine.parsers.dxf_parser import parse_dxf
        # Offloaded: a large CAD parse otherwise blocks the single event loop, which
        # stalls the 10Hz telemetry push AND queues Socket.IO emergency_stop behind
        # it. Every other parse_dxf call site in this module already does this.
        entities = await asyncio.to_thread(parse_dxf, fpath)

        entity_infos = []
        layer_names = set()
        unit_scale = entities[0].unit_scale if entities else 0.01

        for ent in entities:
            layer_names.add(ent.layer)
            length = 0.0
            if ent.entity_type == "LINE":
                s = ent.geometry.get("start", (0, 0))
                e = ent.geometry.get("end", (0, 0))
                length = ((s[0]-e[0])**2 + (s[1]-e[1])**2)**0.5
            elif ent.entity_type == "CIRCLE":
                length = 2 * math.pi * ent.geometry.get("radius", 0)
            elif ent.entity_type == "ARC":
                r = ent.geometry.get("radius", 0)
                a1 = ent.geometry.get("start_angle", 0)
                a2 = ent.geometry.get("end_angle", 360)
                sweep_deg = (a2 - a1) % 360.0
                length = r * math.radians(sweep_deg)

            entity_infos.append(DXFEntityInfo(
                entity_type=ent.entity_type,
                layer=ent.layer,
                color=ent.color,
                entity_id=ent.entity_id,
                is_mark=ent.is_mark(),
                length_m=round(length, 3),
            ))

        # Parse succeeded — move temp file to final location
        final_path = os.path.join(MISSION_DIR, safe)
        os.replace(fpath, final_path)
        path_mgr.clear_entity_overrides(safe)
        path_mgr.clear_extension_config(safe)
        path_mgr.clear_entity_order(safe)

        geo_origin = _geo_origin_of(entities)
        return DXFParseResponse(
            filename=safe,
            num_entities=len(entities),
            entities=entity_infos,
            unit_scale=unit_scale,
            layer_names=sorted(layer_names),
            is_geographic=geo_origin is not None,
            geo_origin=geo_origin,
        )
    except ImportError:
        os.unlink(fpath)
        raise HTTPException(500, "ezdxf not installed. Run: pip install ezdxf")
    except Exception as exc:
        os.unlink(fpath)
        raise HTTPException(422, f"DXF parse error: {exc}")


# ── Plan ──────────────────────────────────────────────────────────────────────

@path_router.post("/plan")
async def plan_path(req: PathPlanRequest):
    """Run the full planning pipeline and return merged waypoints with spray flags."""
    from main import path_mgr

    unsupported = []
    if req.selected_entities is not None:
        unsupported.append("selected_entities")
    if req.overrides is not None:
        unsupported.append("overrides")
    if req.order is not None:
        unsupported.append("order")
    if unsupported:
        raise HTTPException(
            422,
            "Preview fields not implemented yet: " + ", ".join(unsupported),
        )

    # Extension fields moved to GET/POST /api/path/{name}/extensions — tell
    # old clients their explicit values are being ignored instead of silently
    # planning with different settings.
    deprecation_warning = None
    deprecated_set = {
        "enable_path_extensions", "pre_extension_m", "aft_extension_m",
    } & req.model_fields_set
    if deprecated_set:
        deprecation_warning = (
            "Ignored deprecated field(s) "
            + ", ".join(sorted(deprecated_set))
            + ": path extensions are configured per DXF via "
            "GET/POST /api/path/{name}/extensions."
        )
        log.warning("/api/path/plan: %s", deprecation_warning)

    # Planner-side spray latency compensation is GONE, not merely defaulted off. The
    # spray controller node already leads the solenoid at runtime from the rover's ACTUAL
    # speed (spray_controller_node.py:276); doing it here too shifted the boundary a
    # further 3.5 cm, so paint began ~9 cm before the CAD line. A client that still asks
    # for it would silently get that double compensation back, so the field is refused
    # rather than honoured — the plan always carries the true CAD geometry.
    spray_compensation_warning = None
    if req.compensate_spray:
        spray_compensation_warning = (
            "Ignored compensate_spray=true: solenoid latency is compensated by the "
            "spray controller at runtime, from actual speed. Applying it in the planner "
            "as well double-compensates and starts paint ~9 cm early. The plan marks "
            "exactly the CAD geometry."
        )
        log.warning("/api/path/plan: %s", spray_compensation_warning)

    origin = tuple(req.origin) if req.origin else (0.0, 0.0)
    start_position = tuple(req.start_position) if req.start_position else None
    summary_only = not (req.include_waypoints)
    origin_gps = tuple(req.origin_gps) if req.origin_gps else None
    ref_points_dxf = [(pt.dxf_y, pt.dxf_x) for pt in req.ref_points] if req.ref_points is not None else None
    ref_points_gps = [(pt.lat, pt.lon) for pt in req.ref_points] if req.ref_points is not None else None

    try:
        result = await asyncio.wait_for(
            _plan_in_thread(
                path_mgr.plan_path,
                req.source,
                summary_only=summary_only,
                line_spacing=req.line_spacing,
                transit_spacing=req.transit_spacing,
                marking_speed=req.marking_speed,
                transit_speed=req.transit_speed,
                layer_mapping=req.layer_mapping,
                optimize=req.optimize,
                compensate_spray=False,   # never: see spray_compensation_warning above
                # Extension settings are configured per DXF via
                # GET/POST /api/path/{name}/extensions, then loaded by
                # PathManager during planning.
                corner_smooth_radius_m=req.corner_smooth_radius_m,
                corner_smooth_arc_pts=req.corner_smooth_arc_pts,
                fit_arcs=req.fit_arcs,
                fit_arcs_rms_m=req.fit_arcs_rms_m,
                fit_arcs_corner_deg=req.fit_arcs_corner_deg,
                fit_arcs_max_dev_m=req.fit_arcs_max_dev_m,
                close_shape=req.close_shape,
                use_two_opt=req.use_two_opt,
                max_two_opt_segments=req.max_two_opt_segments,
                max_waypoints=req.max_waypoints,
                max_segments=req.max_segments,
                origin=origin,
                start_position=start_position,
                origin_gps=origin_gps,
                rotation_deg=req.rotation_deg,
                ref_points_dxf=ref_points_dxf,
                ref_points_gps=ref_points_gps,
                close_loop=req.close_loop,
            ),
            timeout=15.0,
        )
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except ImportError as exc:
        raise HTTPException(500, str(exc))
    except asyncio.TimeoutError:
        raise HTTPException(504, "Planning timed out (15s limit)")
    except Exception as exc:
        raise HTTPException(422, f"Planning error: {exc}")

    extra_warnings = [w for w in (deprecation_warning, spray_compensation_warning) if w]
    if extra_warnings:
        result["warnings"] = list(result.get("warnings") or []) + extra_warnings

    alignment_meta = result.get("alignment_metadata") or {}

    # Gap D: RMSE quality gate. Only least-squares alignment produces a residual;
    # single-point/gps-origin modes report rmse=0 and pass by definition.
    rmse = alignment_meta.get("rmse", 0.0)
    if rmse > RMSE_MAX:
        raise HTTPException(
            422,
            f"Alignment error too high (rmse={rmse:.3f} m, max {RMSE_MAX:.3f} m). "
            "Re-verify the reference points.",
        )
    if alignment_meta.get("method"):
        _assert_alignment_scale(alignment_meta)
        _assert_origin_gps_usable(alignment_meta.get("origin_gps") or origin_gps)

    # Gaps C & E: stage the fully-aligned mission so the operator can confirm and
    # load exactly what was previewed. Scoped to the aligned-DXF flow only — built-in
    # and CSV/.waypoints paths keep using /api/mission/load (no alignment to reproduce).
    mission_summary = None
    if alignment_meta.get("method") and req.include_waypoints and result.get("merged_waypoints"):
        mission_summary = _stage_mission(req, result, alignment_meta, rmse)

    return PathPlanResponse(
        source=result["source"],
        num_waypoints=result["num_waypoints"],
        num_segments=result["num_segments"],
        mark_length_m=result["mark_length_m"],
        transit_length_m=result["transit_length_m"],
        total_length_m=result["total_length_m"],
        segments=result["segments"],
        merged_waypoints=result.get("merged_waypoints", []),
        spray_flags=result.get("spray_flags", []),
        must_hit=result.get("must_hit", []),
        alignment_metadata=alignment_meta or None,
        planning_metadata=result.get("planning_metadata"),
        warnings=result.get("warnings"),
        mission_summary=mission_summary,
    )


def _prune_staging() -> None:
    """Remove staged missions older than STAGING_TTL_S. Best-effort."""
    try:
        now = time.time()
        for fname in os.listdir(STAGING_DIR):
            if not fname.endswith(".json"):
                continue
            fpath = os.path.join(STAGING_DIR, fname)
            try:
                if now - os.path.getmtime(fpath) > STAGING_TTL_S:
                    os.remove(fpath)
            except OSError:
                continue
    except FileNotFoundError:
        pass


def _source_detail(result: dict) -> dict | None:
    """Provenance of the file this plan was built from, for the staged artifact.

    PathEngine.plan_file() records ``filepath`` / ``extension`` /
    ``unit_scale_m_per_unit`` under planning_metadata. The entity-list planner
    records the same block minus ``filepath`` (it never opened a file by name),
    so resolve the flat source name against MISSION_DIR to fill the gap —
    ``tools/analyze_mission.py`` needs a real path to re-read the surveyed
    geometry and report absolute accuracy.

    Returns None when there is nothing useful to record (e.g. a builtin path),
    so the field is absent rather than a dict of nulls.
    """
    planning = result.get("planning_metadata") or {}
    src = planning.get("source")
    detail = dict(src) if isinstance(src, dict) else {}

    name = result.get("source")
    if isinstance(name, str) and name and not name.startswith("builtin:"):
        detail.setdefault("name", name)
        if not detail.get("filepath"):
            candidate = os.path.join(MISSION_DIR, os.path.basename(name))
            if os.path.isfile(candidate):
                detail["filepath"] = candidate
        if not detail.get("extension"):
            ext = os.path.splitext(name)[1].lower()
            if ext:
                detail["extension"] = ext

    return detail or None


def _stage_mission(req: PathPlanRequest | PlanTrajectoryRequest, result: dict,
                   alignment_meta: dict, rmse: float,
                   survey_ground_truth: list[dict] | None = None) -> MissionSummary:
    """Write the aligned mission to a staging file and return its summary.

    The staged artifact is the single source of truth for the subsequent
    /load-to-controller step, so the operator loads exactly what was previewed.

    ``req`` is duck-typed: everything read off it (spray_mode, the dash trio,
    the point pair, survey_tolerance_m, marking_speed/transit_speed) is present
    on both PathPlanRequest and PlanTrajectoryRequest, which is what lets the
    app-planned trajectory flow reuse this staging code unchanged.

    ``survey_ground_truth`` — surveyed lat/lon bound to the mission's own NED
    vertices, staged INSIDE the artifact. Only the app-planned trajectory flow
    passes it: that flow has no source file, so ``analyze_mission``'s §8
    absolute accuracy has nothing to re-read and would go silent on every
    mission from it — shipping the feature with the one whole-mission-
    misplacement detector switched off. File-based missions pass None and keep
    re-reading their source, byte-for-byte as before.
    """
    os.makedirs(STAGING_DIR, exist_ok=True)
    _prune_staging()

    mission_id = f"stg_{uuid.uuid4().hex[:8]}_{int(time.time())}"

    # Gap E: definitive global anchor header for the controller / microcontroller.
    anchor = None
    origin_gps = alignment_meta.get("origin_gps")
    if origin_gps:
        anchor = {
            "frame": "local_ned",
            "lat": origin_gps[0],
            "lon": origin_gps[1],
            "rotation_deg": alignment_meta.get("rotation_deg", 0.0),
            "scale": alignment_meta.get("scale", 1.0),
        }

    # Anchor leads the artifact (Gap E): the microcontroller/controller consumes
    # the global anchor header before the waypoint stream.
    staged_payload = {
        "anchor": anchor,
        "mission_id": mission_id,
        "created_at": time.time(),
        # Surveyed staged missions must re-bind into the live EKF at start.
        "placement_mode": "GPS_SURVEYED" if origin_gps else "LOCAL_NED",
        "origin_gps": list(origin_gps) if origin_gps else None,
        "waypoints": result.get("merged_waypoints", []),
        "spray_flags": result.get("spray_flags", []),
        # Vertex provenance: True = source geometry, never simplify away.
        "must_hit": result.get("must_hit", []),
        # Spray mode (B0/Phase C). Rides to the controller at load and is
        # published on /spray/session_config. Geometry stays in spray_flags/
        # waypoints; this carries only the mode + dash metering distances.
        "spray_session": {
            "mode": req.spray_mode,
            "dash_on_distance_m": req.dash_on_distance_m,
            "dash_off_distance_m": req.dash_off_distance_m,
            "dash_start_state": req.dash_start_state,
            # Point-mode dwell params. Coordinates are NOT staged — they ride
            # /path as must-hit vertices and are placed into the live EKF frame
            # at start; the spray node reads them there.
            "point_dwell_s": req.point_dwell_s,
            "point_arrival_tolerance_m": req.point_arrival_tolerance_m,
        },
        "alignment_metadata": alignment_meta,
        "metadata": {
            "source": result["source"],
            # metadata.source is the flat filename and stays a string — the
            # frontend renders it. The bag recorder needs the file's provenance
            # (absolute path, extension, unit scale) to run §8 absolute accuracy,
            # so carry the planner's own source block alongside it. PathEngine
            # already builds exactly this dict in plan_file(); see
            # path_engine/engine.py:366.
            "source_detail": _source_detail(result),
            # Operator-set, per survey. Absent (None) means "use the analyser's
            # default" — deliberately NOT defaulted to a number here, or every
            # mission would claim an explicit tolerance it never chose.
            "survey_tolerance_m": req.survey_tolerance_m,
            "mark_length_m": result["mark_length_m"],
            "transit_length_m": result["transit_length_m"],
            "total_length_m": result["total_length_m"],
        },
    }

    # Absent, not null, when there is none — so an artifact from a file-based
    # mission is byte-identical to what it was before this key existed.
    if survey_ground_truth:
        staged_payload["survey_ground_truth"] = survey_ground_truth

    staging_file = os.path.join(STAGING_DIR, f"{mission_id}.json")
    tmp = staging_file + ".tmp"
    with open(tmp, "w") as f:
        json.dump(staged_payload, f)
    os.replace(tmp, staging_file)  # atomic publish

    # Advisory only. Staging is a planning act and legitimately happens with the
    # rover off or still acquiring RTK, so it is NOT refused here — the hard
    # gates are /load-to-controller and mission start, which are the points that
    # actually bind the mission to the ground. This warning exists so a stale
    # origin leaves a trace in the log at the moment the mission was authored.
    if origin_gps:
        health, unavailable = _surveyed_origin_health()
        if unavailable is not None:
            log.info("staged surveyed mission %s — origin not checkable: %s",
                     mission_id, unavailable)
        elif not health.get("trusted"):
            log.warning(
                "staged surveyed mission %s while the EKF local-frame origin is "
                "NOT trustworthy [%s]: %s — load and start will refuse until this "
                "clears", mission_id, health.get("status"), health.get("detail"))

    # Commercial estimates. Speeds are > 0 (engine validates before we get here).
    paint_l = result["mark_length_m"] * SPRAY_LITERS_PER_METER
    runtime_s = (
        result["mark_length_m"] / req.marking_speed
        + result["transit_length_m"] / req.transit_speed
    )

    return MissionSummary(
        mission_id=mission_id,
        num_waypoints=result["num_waypoints"],
        total_length_m=result["total_length_m"],
        estimated_paint_l=round(paint_l, 3),
        estimated_runtime_s=round(runtime_s, 1),
        rmse_m=round(rmse, 4),
    )


@path_router.post("/load-to-controller")
async def load_mission_to_controller(req: LoadMissionRequest):
    """Commit a previously staged, aligned mission to the OffboardController.

    Reads the staged artifact and pushes the already-aligned waypoints down to
    the controller — no re-planning, no re-alignment — so the loaded mission is
    byte-for-byte what the operator confirmed in the preview.
    """
    from main import offboard_ctrl
    from models import MissionState

    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")

    # Field-safety: refuse to swap the loaded path while a mission is active or
    # mid-lifecycle. Loading is only meaningful from a settled state; the operator
    # must stop/abort first. (load_path itself only warns — make it an explicit 409.)
    _load_blocked = {
        MissionState.RUNNING,
        MissionState.ENTRY,
        MissionState.LOADING,
        MissionState.ARMING,
        MissionState.SWITCHING_OFFBOARD,
        MissionState.STOPPING,
        MissionState.DISARMING,
    }
    if offboard_ctrl.state in _load_blocked:
        raise HTTPException(
            409,
            f"Controller is {offboard_ctrl.state.value} — stop the active mission "
            "before loading a new one.",
        )

    safe_id = os.path.basename(req.mission_id)
    staging_file = os.path.join(STAGING_DIR, f"{safe_id}.json")
    if not os.path.isfile(staging_file):
        raise HTTPException(404, "Staged mission not found or expired.")

    try:
        with open(staging_file) as f:
            staged = json.load(f)
    except (OSError, ValueError) as exc:
        raise HTTPException(422, f"Could not read staged mission: {exc}")

    waypoints = [tuple(pt) for pt in staged.get("waypoints", [])]
    if not waypoints:
        raise HTTPException(422, "Staged mission has no waypoints.")

    anchor = staged.get("anchor")
    if anchor:
        import logging
        logging.getLogger("server.path").info(
            "loading mission %s with anchor lat=%.7f lon=%.7f rot=%.2f scale=%.4f",
            safe_id, anchor["lat"], anchor["lon"],
            anchor.get("rotation_deg", 0.0), anchor.get("scale", 1.0),
        )

    placement_mode = staged.get("placement_mode") or (
        "GPS_SURVEYED" if staged.get("origin_gps") else "LOCAL_NED"
    )

    # ── Fail closed on an untrustworthy EKF local-frame origin ───────────────
    # A surveyed mission is bound to the ground at START, through the EKF's
    # declared local-frame origin. Refusing only at start is too late to be
    # useful: load is the commitment point — it publishes the mission geometry
    # the spray node latches, and it is what the operator does before walking
    # the rover out. So the same verdict start enforces is checked here, and
    # the operator finds out at the desk instead of in the field.
    # (Staging is deliberately NOT gated: planning legitimately happens with the
    # rover off. It logs a warning instead — see _stage_mission.)
    if placement_mode == "GPS_SURVEYED":
        _assert_origin_trusted_for_surveyed(safe_id)

    try:
        spray_flags = [bool(f) for f in staged.get("spray_flags", [])]
        must_hit = [bool(f) for f in staged.get("must_hit", [])]
        origin_gps = staged.get("origin_gps")
        if origin_gps is not None:
            origin_gps = (float(origin_gps[0]), float(origin_gps[1]))
        offboard_ctrl.load_path(
            waypoints,
            name=safe_id,
            spray_flags=spray_flags,
            must_hit=must_hit,
            placement_mode=placement_mode,
            origin_gps=origin_gps,
            is_staged=True,
        )
    except PlacementError as exc:
        raise HTTPException(422, str(exc))
    except Exception as exc:
        raise HTTPException(409, f"Controller load failed: {exc}")

    # B0/Phase C: publish the spray mode on /spray/session_config right after
    # /path so geometry and mode reach the spray node from the same load. The
    # node is the sole parser and fails static on anything it rejects, so a
    # publish failure here must never fail the mission load — log and continue.
    spray_session = staged.get("spray_session") or {}
    spray_mode = spray_session.get("mode", "continuous")
    try:
        from main import ros_node
        from spray_session_builder import build_session_config_json

        cfg_json = build_session_config_json(
            spray_mode,
            dash_on_distance_m=spray_session.get("dash_on_distance_m"),
            dash_off_distance_m=spray_session.get("dash_off_distance_m"),
            dash_start_state=spray_session.get("dash_start_state", "on"),
            # Point dwell params. Coordinates stay empty on purpose — the node
            # fills them from the placed /path must-hit vertices (the only
            # frame-correct source; the placement offset is unknown until start).
            point_dwell_s=spray_session.get("point_dwell_s", 1.0),
            point_arrival_tolerance_m=spray_session.get(
                "point_arrival_tolerance_m", 0.10
            ),
        )
        ros_node.publish_spray_session_config(cfg_json)
    except Exception as exc:  # noqa: BLE001 — mode publish is best-effort
        import logging
        logging.getLogger("server.path").warning(
            "spray session_config publish failed for %s (mode=%s): %s — "
            "spray node keeps its last mode", safe_id, spray_mode, exc,
        )

    return {
        "status": "success",
        "mission_id": safe_id,
        "num_waypoints": len(waypoints),
        "anchor_loaded": anchor is not None,
        "placement_mode": placement_mode,
        "origin_gps": list(origin_gps) if origin_gps else None,
    }


# ── Spray pattern ("Apply pattern" from the mobile app) ─────────────────────────
# The app sets the spray pattern on a selected PATH, separate from planning:
#   PUT /api/path/{name}/spray-mode/{continuous|dash|point}
# Geometry (MARK/transit + must-hit) rides /path from the loaded mission; these
# routes only publish the MODE overlay on /spray/session_config, which the spray
# node latches (RELIABLE + TRANSIENT_LOCAL) and applies to the current geometry.
#
# Ordering: apply AFTER the mission is loaded. load-to-controller republishes
# the mission's staged mode (continuous by default), which would override a mode
# set before load.

def _publish_spray_session(safe_name: str, cfg_json: str, mode: str) -> dict:
    """Publish a session_config to the spray node. Best-effort; never raises.

    Off-ROS (Mac dev) or with a missing publisher this returns published=False
    with the reason — the route still succeeds so the contract is verifiable
    without a live ROS graph.
    """
    try:
        from main import ros_node
        ros_node.publish_spray_session_config(cfg_json)
        return {"published": True, "detail": None}
    except Exception as exc:  # noqa: BLE001 — publish is best-effort/off-ROS
        log.warning("spray-mode publish failed for %s (mode=%s): %s",
                    safe_name, mode, exc)
        return {"published": False, "detail": str(exc)}


@path_router.put("/{name}/spray-mode/continuous")
async def set_spray_mode_continuous(name: str):
    """Set the selected path to continuous spray (plan-start → plan-stop)."""
    from spray_session_builder import build_session_config_json

    safe = os.path.basename(name)
    cfg_json = build_session_config_json("continuous")
    pub = _publish_spray_session(safe, cfg_json, "continuous")
    resp = {
        "status": "ok",
        "path": safe,
        "mode": "continuous",
        "applied_config": json.loads(cfg_json),
        "published": pub["published"],
        "warnings": [],
    }
    if pub["detail"]:
        resp["publish_error"] = pub["detail"]
    return resp


@path_router.put("/{name}/spray-mode/dash")
async def set_spray_mode_dash(name: str, req: SprayModeDashRequest):
    """Set the selected path to dash spray (ON/OFF by metres over the path)."""
    from spray_session_builder import build_session_config_json

    safe = os.path.basename(name)
    cfg_json = build_session_config_json(
        "dash",
        dash_on_distance_m=req.dash_on_distance_m,
        dash_off_distance_m=req.dash_off_distance_m,
        dash_start_state="on",
    )
    cfg = json.loads(cfg_json)
    warnings: list[str] = []
    if cfg.get("mode") != "dash":
        warnings.append(
            "dash config incomplete — the node stays continuous "
            "(both on/off distances must be > 0)."
        )
    if req.dash_phase_reset == "per_mark_region":
        warnings.append(
            "dash_phase_reset='per_mark_region' accepted but not yet honored: "
            "the shipped meter runs continuous across the mission."
        )
    pub = _publish_spray_session(safe, cfg_json, "dash")
    resp = {
        "status": "ok",
        "path": safe,
        "mode": "dash",
        "dash_phase_reset": req.dash_phase_reset,
        "applied_config": cfg,
        "published": pub["published"],
        "warnings": warnings,
    }
    if pub["detail"]:
        resp["publish_error"] = pub["detail"]
    return resp


@path_router.put("/{name}/spray-mode/point")
async def set_spray_mode_point(name: str, req: SprayModePointRequest):
    """Acknowledge point spray for the selected path.

    The app's point contract carries no marking-point coordinates, so this route
    deliberately does NOT publish a session_config: publishing point with no
    coordinates resolves to continuous and would demote a point mission already
    loaded with its coordinates. Point marking is realized through the staged
    point mission (coordinates + must-hit on /path) plus the RPP point-hold A/B
    (`point_hold_enabled`, default OFF).
    """
    safe = os.path.basename(name)
    # G4.5 — carry point_execution_mode to the RPP (design §8 option (a): the
    # server sets it as an RPP param at load). auto|manual selects whether the
    # RPP auto-advances on /spray/point_done or holds in WAIT_OPERATOR for the
    # operator's /point/advance. Behaviour is gated by point_handshake_enabled on
    # the RPP; setting the mode alone changes nothing until the handshake is on.
    exec_mode = str(req.point_execution_mode or "auto").lower()
    warnings = [
        "point marking-point coordinates are not part of this contract; the "
        "spray node is not switched here. Stop-and-dwell at each point is the "
        "RPP point-hold A/B — set `point_hold_enabled true` on /rpp_controller "
        "and run a mission whose must-hit points carry the coordinates."
    ]
    exec_applied = False
    if exec_mode not in ("auto", "manual"):
        warnings.append(
            f"point_execution_mode {exec_mode!r} not in auto|manual; leaving the "
            f"RPP param unchanged"
        )
    else:
        try:
            from main import ros_node
            ok, msg = await ros_node.set_rpp_param_async(
                "point_execution_mode", exec_mode
            )
            exec_applied = bool(ok)
            if not ok:
                warnings.append(f"could not set RPP point_execution_mode: {msg}")
        except Exception as exc:  # noqa: BLE001 — best-effort/off-ROS
            log.warning("set point_execution_mode failed for %s: %s", safe, exc)
            warnings.append(f"could not set RPP point_execution_mode: {exc}")
    return {
        "status": "ok",
        "path": safe,
        "mode": "point",
        "point_execution_mode": exec_mode,
        "point_execution_mode_applied": exec_applied,
        "published": False,
        "warnings": warnings,
    }


# ── Staged workflow: stage-specific endpoints ──────────────────────────────────
# These split the monolithic POST /api/path/plan into composable stages.
# /plan stays untouched; everything below is additive.

def _require_dxf(name: str) -> str:
    """Resolve a DXF in MISSION_DIR or raise 404/415. Returns the safe basename."""
    safe = os.path.basename(name)
    fpath = os.path.join(MISSION_DIR, safe)
    if not os.path.isfile(fpath):
        raise HTTPException(404, f"Path not found: {name!r}")
    if os.path.splitext(fpath)[1].lower() != ".dxf":
        raise HTTPException(415, "This stage is only available for DXF files")
    return safe


def _read_json(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def _spray_runs(waypoints: list, spray_flags: list) -> list[dict]:
    """Derive contiguous spray-ON/OFF runs from a parallel flag list."""
    if not spray_flags or len(spray_flags) != len(waypoints):
        return []
    runs: list[dict] = []
    start = 0
    for i in range(1, len(spray_flags) + 1):
        if i == len(spray_flags) or bool(spray_flags[i]) != bool(spray_flags[start]):
            runs.append({
                "type": "MARK" if spray_flags[start] else "TRANSIT",
                "spray_on": bool(spray_flags[start]),
                "start_index": start,
                "end_index": i - 1,
                "num_points": i - start,
            })
            start = i
    return runs


@path_router.post("/{name}/align", response_model=AlignResponse)
async def align_path(name: str, req: AlignRequest):
    """Stage 6/7 — alignment ONLY: transformed coords + per-refpoint residuals.

    Reuses path_manager.plan_path's alignment path but forces optimize/extend/
    smoothing OFF and never stages or loads the controller.
    """
    from main import path_mgr

    safe = _require_dxf(name)
    if not req.ref_points and not req.origin_gps:
        raise HTTPException(422, "Provide ref_points or origin_gps to align.")

    origin = tuple(req.origin) if req.origin else (0.0, 0.0)
    ref_points_dxf = [(pt.dxf_y, pt.dxf_x) for pt in req.ref_points] if req.ref_points else None
    ref_points_gps = [(pt.lat, pt.lon) for pt in req.ref_points] if req.ref_points else None
    origin_gps = tuple(req.origin_gps) if req.origin_gps else None

    try:
        result = await asyncio.wait_for(
            _plan_in_thread(
                path_mgr.plan_path,
                safe,
                summary_only=False,
                optimize=False,             # alignment only — no reordering
                enable_path_extensions=False,
                compensate_spray=False,
                corner_smooth_radius_m=0.0,
                origin=origin,
                auto_origin=req.auto_origin,
                origin_gps=origin_gps,
                rotation_deg=req.rotation_deg,
                ref_points_dxf=ref_points_dxf,
                ref_points_gps=ref_points_gps,
            ),
            timeout=15.0,
        )
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except asyncio.TimeoutError:
        raise HTTPException(504, "Alignment timed out (15s limit)")
    except Exception as exc:
        raise HTTPException(422, f"Alignment error: {exc}")

    meta = result.get("alignment_metadata") or {}
    if not meta.get("method"):
        raise HTTPException(422, "No alignment produced — check ref_points / origin_gps.")
    _assert_alignment_scale(meta)
    _assert_origin_gps_usable(meta.get("origin_gps") or origin_gps)

    waypoints = result.get("merged_waypoints", [])
    sample = [list(p) for p in waypoints[:req.sample_points]] if req.sample_points else []

    residuals_out: list[RefPointResidual] = []
    res_list = meta.get("residuals") or []
    if req.ref_points and len(res_list) == len(req.ref_points):
        residuals_out = [
            RefPointResidual(
                dxf_x=pt.dxf_x, dxf_y=pt.dxf_y, lat=pt.lat, lon=pt.lon,
                residual_m=round(float(r), 4),
            )
            for pt, r in zip(req.ref_points, res_list)
        ]

    return AlignResponse(
        source=result["source"],
        method=meta.get("method"),
        rmse_m=round(float(meta.get("rmse", 0.0)), 4),
        scale=float(meta.get("scale", 1.0)),
        rotation_deg=float(meta.get("rotation_deg", 0.0)),
        offset_n=float(meta.get("offset_n", 0.0)),
        offset_e=float(meta.get("offset_e", 0.0)),
        origin_gps=list(meta["origin_gps"]) if meta.get("origin_gps") else None,
        num_waypoints=result["num_waypoints"],
        sample_coords=sample,
        residuals=residuals_out,
        warnings=result.get("warnings") or None,
    )


@path_router.get("/{name}/segments", response_model=PathSegmentsResponse)
async def path_segments(name: str):
    """Stage 8 — verification segments: MARK/TRANSIT, PRE/AFT roles, spray flags.

    Reuses saved entity order / overrides / extension config. No staging, no
    controller load, no GPS alignment (local NED).
    """
    from main import path_mgr

    safe = _require_dxf(name)
    try:
        result = await asyncio.wait_for(
            _plan_in_thread(
                path_mgr.plan_path,
                safe,
                summary_only=False,
                include_segment_points=True,
            ),
            timeout=15.0,
        )
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except asyncio.TimeoutError:
        raise HTTPException(504, "Segment build timed out (15s limit)")
    except Exception as exc:
        raise HTTPException(422, f"Segment build error: {exc}")

    seg_models = [
        SegmentInfo(
            index=s["runtime_segment_index"],
            sequence=s["runtime_sequence"],
            type=s["type"],
            segment_role=s.get("segment_role"),
            source_entity=s.get("source", ""),
            is_extension=s.get("is_extension", False),
            spray_on=s.get("spray_on", s["type"] == "MARK"),
            speed=s.get("speed", 0.0),
            length_m=s.get("length_m", 0.0),
            points=s.get("points", []),
        )
        for s in result["segments"]
    ]

    ext_cfg = await asyncio.to_thread(path_mgr.load_extension_config, safe)

    return PathSegmentsResponse(
        name=safe,
        num_segments=result["num_segments"],
        num_waypoints=result["num_waypoints"],
        mark_length_m=result["mark_length_m"],
        transit_length_m=result["transit_length_m"],
        total_length_m=result["total_length_m"],
        extension_config=PathExtensionConfig(**ext_cfg),
        segments=seg_models,
        warnings=result.get("warnings") or None,
    )


@path_router.post("/{name}/plan-and-stage", response_model=PathPlanResponse)
async def plan_and_stage(name: str, req: PathPlanRequest):
    """Stage 9 — heavy final planning + staging only.

    Same pipeline as POST /api/path/plan, but ``name`` comes from the path and
    the staged artifact is always written when waypoints exist, so the result
    can be inspected via GET /api/path/staged/{mission_id} and committed with
    /load-to-controller. ``source`` in the body must match ``name`` (or be
    omitted) — the path name is authoritative.
    """
    from main import path_mgr

    safe = os.path.basename(name)

    # Parity with /api/path/plan: reject preview fields that aren't wired up,
    # so a caller can't unknowingly stage a broader mission than requested.
    unsupported = [
        field for field, val in (
            ("selected_entities", req.selected_entities),
            ("overrides", req.overrides),
            ("order", req.order),
        ) if val is not None
    ]
    if unsupported:
        raise HTTPException(
            422, "Preview fields not implemented yet: " + ", ".join(unsupported),
        )

    # The path name is authoritative; a mismatched body.source is almost always
    # a client bug, so surface it instead of silently planning a different file.
    if req.source and os.path.basename(req.source) != safe:
        raise HTTPException(
            422, f"source {req.source!r} does not match path name {safe!r}",
        )

    origin = tuple(req.origin) if req.origin else (0.0, 0.0)
    start_position = tuple(req.start_position) if req.start_position else None
    origin_gps = tuple(req.origin_gps) if req.origin_gps else None
    ref_points_dxf = [(pt.dxf_y, pt.dxf_x) for pt in req.ref_points] if req.ref_points is not None else None
    ref_points_gps = [(pt.lat, pt.lon) for pt in req.ref_points] if req.ref_points is not None else None

    # Point mission (mobile CSV flow): the waypoints ARE the surveyed points —
    # there is no file-based line geometry to plan. path_mgr.plan_path() would
    # read `name` as NED/DXF line geometry and raise on a point CSV, so skip the
    # planner entirely and synthesize a result that stages the points as must-hit
    # /path vertices (the native Upgrade_Spray point model: RPP point-hold + the
    # spray node's per-must-hit dwell). `mark` maps to BOTH the must-hit flag and
    # the spray flag, so only marked points become stop + spray-dwell targets and
    # an unmarked point is a plain transit vertex (never sprayed — the spray node
    # dwells on EVERY must-hit vertex regardless of the spray bit). Requires the
    # GPS_SURVEYED frame + an origin_gps anchor, matching the app's contract; any
    # other shape falls through to the ordinary line planner unchanged.
    is_point_mission = bool(
        req.point_mission_points
        and req.point_source_frame == "GPS_SURVEYED"
        and req.origin_gps
    )

    if is_point_mission:
        pts = req.point_mission_points
        waypoints = [[float(p.north_m), float(p.east_m)] for p in pts]
        flags = [bool(p.mark) for p in pts]
        transit_len = sum(
            math.hypot(waypoints[i + 1][0] - waypoints[i][0],
                       waypoints[i + 1][1] - waypoints[i][1])
            for i in range(len(waypoints) - 1)
        )
        # Dots are dwell-sprayed, not line-sprayed → the mission's spray mode is
        # "point" regardless of the (defaulted) request field; force it so the
        # staged spray_session tells the node to run its point-dwell FSM.
        req.spray_mode = "point"
        result = {
            "source": safe,
            "num_waypoints": len(waypoints),
            "num_segments": 0,
            "mark_length_m": 0.0,
            "transit_length_m": transit_len,
            "total_length_m": transit_len,
            "segments": [],
            "merged_waypoints": waypoints,
            "spray_flags": flags,
            "must_hit": flags,
            "alignment_metadata": {
                "method": "gps_origin",
                "origin_gps": list(req.origin_gps),
                "rotation_deg": req.rotation_deg,
                "scale": 1.0,
                "fitted_scale": 1.0,
                "rmse": 0.0,
            },
            "planning_metadata": {},
            "warnings": [],
        }
    else:
        try:
            result = await asyncio.wait_for(
                _plan_in_thread(
                    path_mgr.plan_path,
                    safe,
                    summary_only=False,
                    line_spacing=req.line_spacing,
                    transit_spacing=req.transit_spacing,
                    marking_speed=req.marking_speed,
                    transit_speed=req.transit_speed,
                    layer_mapping=req.layer_mapping,
                    optimize=req.optimize,
                    compensate_spray=False,   # never: see spray_compensation_warning above
                    corner_smooth_radius_m=req.corner_smooth_radius_m,
                    corner_smooth_arc_pts=req.corner_smooth_arc_pts,
                    use_two_opt=req.use_two_opt,
                    max_two_opt_segments=req.max_two_opt_segments,
                    max_waypoints=req.max_waypoints,
                    max_segments=req.max_segments,
                    origin=origin,
                    start_position=start_position,
                    origin_gps=origin_gps,
                    rotation_deg=req.rotation_deg,
                    ref_points_dxf=ref_points_dxf,
                    ref_points_gps=ref_points_gps,
                    close_loop=req.close_loop,
                    # A15 (2026-07-27): these five were accepted by the request
                    # model, documented, and then SILENTLY DROPPED on the way to
                    # the planner — only /api/path/plan forwarded them. So the
                    # preview route honoured `fit_arcs` while plan-and-stage, the
                    # route that produces the mission the rover actually drives,
                    # ignored it and always took path_manager's survey-CSV
                    # auto-ON default. Measured on curve_6_points-1: preview with
                    # fit_arcs=false gave 98 wp / 8 of 8 must-hit / 0.00 cm from
                    # the surveyed stations; the staged mission gave 97 wp / 2 of
                    # 8 / 1.95 cm mean, 3.36 cm max — HTTP 200, no warning.
                    #
                    # Passing them straight through (not conditionally) is
                    # deliberate: all four fit_arcs fields default to None in
                    # PathPlanRequest, and path_manager reads None as "auto"
                    # (`fit_arcs = _is_survey_csv(name) if kw is None else ...`),
                    # so omitting them from the request is byte-for-byte
                    # unchanged. close_shape defaults False in both. This mirrors
                    # /api/path/plan exactly, which is the point — the two routes
                    # disagreeing is the bug.
                    fit_arcs=req.fit_arcs,
                    fit_arcs_rms_m=req.fit_arcs_rms_m,
                    fit_arcs_corner_deg=req.fit_arcs_corner_deg,
                    fit_arcs_max_dev_m=req.fit_arcs_max_dev_m,
                    close_shape=req.close_shape,
                ),
                timeout=15.0,
            )
        except FileNotFoundError as exc:
            raise HTTPException(404, str(exc))
        except asyncio.TimeoutError:
            raise HTTPException(504, "Planning timed out (15s limit)")
        except Exception as exc:
            raise HTTPException(422, f"Planning error: {exc}")

    alignment_meta = result.get("alignment_metadata") or {}
    rmse = alignment_meta.get("rmse", 0.0)
    if rmse > RMSE_MAX:
        raise HTTPException(
            422,
            f"Alignment error too high (rmse={rmse:.3f} m, max {RMSE_MAX:.3f} m). "
            "Re-verify the reference points.",
        )
    if alignment_meta.get("method"):
        _assert_alignment_scale(alignment_meta)
        _assert_origin_gps_usable(alignment_meta.get("origin_gps") or origin_gps)

    mission_summary = None
    if result.get("merged_waypoints"):
        mission_summary = _stage_mission(req, result, alignment_meta, rmse)

    # Parity with /api/path/plan: the extension trio is sidecar-driven now.
    warnings = list(result.get("warnings") or [])
    deprecated = {"enable_path_extensions", "pre_extension_m", "aft_extension_m"} & req.model_fields_set
    if deprecated:
        warnings.append(
            "Ignored deprecated field(s) " + ", ".join(sorted(deprecated))
            + ": path extensions are configured per DXF via "
            "GET/POST /api/path/{name}/extensions."
        )

    return PathPlanResponse(
        source=result["source"],
        num_waypoints=result["num_waypoints"],
        num_segments=result["num_segments"],
        mark_length_m=result["mark_length_m"],
        transit_length_m=result["transit_length_m"],
        total_length_m=result["total_length_m"],
        segments=result["segments"],
        merged_waypoints=result.get("merged_waypoints", []),
        spray_flags=result.get("spray_flags", []),
        must_hit=result.get("must_hit", []),
        alignment_metadata=alignment_meta or None,
        planning_metadata=result.get("planning_metadata"),
        warnings=warnings or None,
        mission_summary=mission_summary,
    )


# ── App-planned trajectory: POST /api/path/plan-trajectory ────────────────────
#
# The app has already fitted the geometry and decided the order. This endpoint
# densifies it and stages it. Nothing else.
#
# It exists because the file flow can express the same mission but has five
# parameter traps, every one of which fails SILENTLY and paints the wrong thing:
#
#   optimize=False without explicit travel rows  → two paint runs 50 m apart
#                                                  merge; the rover paints the gap
#   header "north,east" not "north_m,east_m"     → routed to read_survey_csv and
#                                                  the app's fit is re-derived
#   corner_smooth_radius_m > 0                   → re-smooths already-fitted arcs
#                                                  (not gated for NED CSV)
#   two runs sharing an endpoint                 → group_shapes (hardcoded True in
#                                                  plan_path) fuses them
#   two coincident runs                          → engine step 1c drops one, so a
#                                                  deliberate double pass vanishes
#
# Here all five are unreachable by construction rather than by the caller
# remembering to avoid them: the request model cannot express the knobs, and the
# engine is configured at THIS call site (see the contract block in
# plan_trajectory) instead of taking values from the body.

# How far two run endpoints may sit apart and still count as touching. The app
# guarantees exact equality; this is the guard against the next client.
_TRAJ_JOIN_TOL_M = 0.05
# Heading change between consecutive runs above which the operator is warned
# (not refused — a sharp reversal is legitimate on a real marking job).
_TRAJ_HEADING_WARN_DEG = 120.0
# Per-category cap. A 4000-run import with a systematic problem would otherwise
# return 4000 warnings, which is the same as returning none — nobody reads it.
_TRAJ_MAX_WARNINGS = 10


def _traj_len(points) -> float:
    """Arc length of a run, in metres."""
    return sum(math.hypot(points[i + 1][0] - points[i][0],
                          points[i + 1][1] - points[i][1])
               for i in range(len(points) - 1))


def _traj_heading(p0, p1) -> float | None:
    """Bearing of p0→p1 in radians, or None for a zero-length step."""
    dn, de = p1[0] - p0[0], p1[1] - p0[1]
    if math.hypot(dn, de) < 1e-9:
        return None
    return math.atan2(de, dn)


def _validate_trajectory(req: PlanTrajectoryRequest) -> list[str]:
    """Reject the trajectories that today produce wrong paint with no error.

    Raises HTTPException(422) on rule 1-8; returns the operator-judgement
    warnings (which never block). Runs BEFORE any planning, so a bad payload
    costs nothing and the message names the run that is wrong.

    Rule 1 is the one that matters: two adjacent MARK runs with no travel leg
    between them are one contiguous spray region to everything downstream, so
    the rover paints straight across the gap between them.
    """
    runs = req.runs
    warnings: list[str] = []

    # Size ceiling first, before anything walks the points. MAX_UPLOAD_BYTES
    # guards multipart /upload and never sees a JSON body, so this is the only
    # thing standing between a pathological payload and a worker held for the
    # full timeout.
    total_points = sum(len(run.points) for run in runs)
    if total_points > MAX_TRAJECTORY_POINTS:
        raise HTTPException(
            422,
            f"trajectory has {total_points} points, over the {MAX_TRAJECTORY_POINTS} "
            "limit for a single request — split the mission.",
        )

    # Rule 5 next: every later rule does geometry, and hypot(nan) propagates
    # quietly into a "gap" of nan that compares False against every threshold —
    # so an unchecked nan would sail through the continuity rules below.
    for i, run in enumerate(runs):
        for j, pt in enumerate(run.points):
            if not (math.isfinite(pt[0]) and math.isfinite(pt[1])):
                raise HTTPException(422, f"runs[{i}].points[{j}] is not finite")
        # Rule 4. Normally unreachable — TrajectoryRun pins min_length=2, so
        # pydantic rejects first. Kept because this function is also the
        # contract for any caller that builds the model programmatically.
        if len(run.points) < 2:
            raise HTTPException(
                422,
                f"runs[{i}] has {len(run.points)} point"
                f"{'' if len(run.points) == 1 else 's'}; a run needs at least 2",
            )

    # Rule 6.
    if not any(run.kind == "mark" for run in runs):
        raise HTTPException(422, "trajectory has no marking runs")

    sharp_turns: list[tuple[int, float]] = []
    for i in range(len(runs) - 1):
        a, b = runs[i], runs[i + 1]

        # Rule 1.
        if a.kind == "mark" and b.kind == "mark":
            raise HTTPException(
                422,
                f"runs[{i}] and runs[{i + 1}] are both 'mark' with no travel run "
                "between them — the rover would paint across the gap",
            )

        # Rules 2/3. Stated in the request for travel legs specifically, but
        # enforced at EVERY boundary: with optimize_order off the engine inserts
        # no connectors, so any gap left here is not a gap the rover drives —
        # it is a teleport in the merged waypoint list.
        gap = math.hypot(b.points[0][0] - a.points[-1][0],
                         b.points[0][1] - a.points[-1][1])
        if gap > _TRAJ_JOIN_TOL_M:
            verb = "start" if b.kind == "travel" else "continue"
            raise HTTPException(
                422,
                f"runs[{i + 1}] {b.kind} leg does not {verb} where runs[{i}] ends "
                f"(gap {gap:.1f} m)",
            )

        # Warn-only: a near-reversal between runs is legitimate (a double-back
        # pass) but is also what a mis-ordered trajectory looks like.
        head_a = _traj_heading(a.points[-2], a.points[-1])
        head_b = _traj_heading(b.points[0], b.points[1])
        if head_a is not None and head_b is not None:
            turn = abs((math.degrees(head_b - head_a) + 180.0) % 360.0 - 180.0)
            if turn > _TRAJ_HEADING_WARN_DEG:
                sharp_turns.append((i, turn))

    for i, turn in sharp_turns[:_TRAJ_MAX_WARNINGS]:
        warnings.append(
            f"runs[{i}] -> runs[{i + 1}] turns {turn:.0f} deg — check the run "
            "order if that reversal was not intended."
        )
    if len(sharp_turns) > _TRAJ_MAX_WARNINGS:
        warnings.append(
            f"...and {len(sharp_turns) - _TRAJ_MAX_WARNINGS} more run-to-run "
            f"turns over {_TRAJ_HEADING_WARN_DEG:.0f} deg."
        )

    # Warn-only: two MARK runs on the same ground. Deliberate (a double pass) or
    # a duplicated import — the operator decides, but they get told.
    #
    # Uses the SAME key the engine's step 1c uses to decide what to drop
    # (endpoints rounded onto a mark_spacing grid, direction-insensitive, plus
    # the rounded length), so the warning fires exactly when the drop will
    # happen rather than on an independent test that could disagree with it.
    # An all-pairs distance test would be truer to "within 5 cm" but is O(n^2)
    # — 5000 runs is 12.5M hypots on a request that has not been planned yet.
    tol = max(req.line_spacing, 1e-3)
    seen_marks: dict[tuple, int] = {}
    coincident: list[tuple[int, int]] = []
    for i, run in enumerate(runs):
        if run.kind != "mark":
            continue
        a = (round(run.points[0][0] / tol), round(run.points[0][1] / tol))
        b = (round(run.points[-1][0] / tol), round(run.points[-1][1] / tol))
        key = (min(a, b), max(a, b), round(_traj_len(run.points) / tol))
        if key in seen_marks:
            coincident.append((seen_marks[key], i))
        else:
            seen_marks[key] = i
    for ia, ib in coincident[:_TRAJ_MAX_WARNINGS]:
        warnings.append(
            f"runs[{ia}] and runs[{ib}] are coincident mark runs — "
            "the same ground is marked twice."
        )
    if len(coincident) > _TRAJ_MAX_WARNINGS:
        warnings.append(
            f"...and {len(coincident) - _TRAJ_MAX_WARNINGS} more coincident "
            "mark run pair(s)."
        )

    # Rule 7.
    _assert_origin_gps_usable(list(req.origin_gps))

    # Rule 8 — pre-flight, mirroring the DXF guard in path_manager.plan_path
    # (which only runs `if is_dxf`, so a CSV mission spends the whole 15 s
    # budget and surfaces an opaque 504). Here the geometry is straight-line
    # densification of known runs, so the estimate is EXACT rather than a lower
    # bound — no slack factor is warranted.
    mark_len = sum(_traj_len(r.points) for r in runs if r.kind == "mark")
    travel_len = sum(_traj_len(r.points) for r in runs if r.kind == "travel")
    est = int(mark_len / max(req.line_spacing, 1e-3)
              + travel_len / max(req.transit_spacing, 1e-3))
    if est > req.max_waypoints:
        raise HTTPException(
            422,
            f"Too many waypoints: ~{est} exceeds limit {req.max_waypoints}. "
            "Increase line_spacing/transit_spacing or split the mission.",
        )

    return warnings


def _trajectory_run_echo(runs, plan) -> list[TrajectoryRunEcho]:
    """Echo the PLANNED run structure so the client can verify it survived.

    Built from ``plan.segments`` — never from ``spray_flags``. Two adjacent MARK
    segments produce one contiguous run of True flags, so a flag-derived view
    (which is what ``_spray_runs`` and the staged ``segment_runs`` are) reports
    ONE run where there are two. That is correct for their own purpose and
    useless for verifying structure: a fusion is exactly what it cannot see.

    The terminal run-out is appended separately: the engine adds it to
    ``merged_waypoints`` only, not to ``plan.segments`` (path_engine/engine.py,
    "Guarantee a terminal MARK->TRANSIT boundary"), so mapping segments alone
    would under-count the waypoints the client is asked to reconcile.
    """
    labels = {i: run.label for i, run in enumerate(runs)}
    echo = [
        TrajectoryRunEcho(
            index=idx,
            kind="mark" if seg.segment_type == SegmentType.MARK else "travel",
            num_points=len(seg.points),
            length_m=round(seg.length, 3),
            label=labels.get(seg.segment_id),
        )
        for idx, seg in enumerate(plan.segments)
    ]

    # Terminal run-out detection is exact, not heuristic: the engine appends it
    # iff the mission ended on a MARK that is not a closed shape, and close_loop
    # is False here. So "last segment is MARK but the last flag is False" can
    # only mean the run-out point.
    if (plan.segments
            and plan.segments[-1].segment_type == SegmentType.MARK
            and plan.spray_flags and not plan.spray_flags[-1]
            and len(plan.merged_waypoints) >= 2):
        tail = plan.merged_waypoints[-1]
        prev = plan.merged_waypoints[-2]
        echo.append(TrajectoryRunEcho(
            index=len(echo),
            kind="travel",
            num_points=1,
            length_m=round(math.hypot(tail[0] - prev[0], tail[1] - prev[1]), 3),
            label="run-out",
            generated=True,
        ))

    return echo


def _resolve_ground_truth(req: PlanTrajectoryRequest) -> list[dict] | None:
    """Bind each surveyed lat/lon to the NED vertex it was shot at.

    Indices are validated rather than clamped: a ground truth pointing at a run
    that does not exist is a client bug, and silently dropping it would leave §8
    reporting a number computed from a subset nobody chose.
    """
    if not req.ground_truth:
        return None
    out: list[dict] = []
    for k, gt in enumerate(req.ground_truth):
        if gt.run_index >= len(req.runs):
            raise HTTPException(
                422,
                f"ground_truth[{k}].run_index {gt.run_index} is out of range "
                f"({len(req.runs)} runs)",
            )
        pts = req.runs[gt.run_index].points
        if gt.point_index >= len(pts):
            raise HTTPException(
                422,
                f"ground_truth[{k}].point_index {gt.point_index} is out of range "
                f"(runs[{gt.run_index}] has {len(pts)} points)",
            )
        if not (math.isfinite(gt.lat) and math.isfinite(gt.lon)
                and -90.0 <= gt.lat <= 90.0 and -180.0 <= gt.lon <= 180.0):
            raise HTTPException(
                422, f"ground_truth[{k}] lat/lon is not a valid coordinate")
        pt = pts[gt.point_index]
        out.append({
            "north_m": float(pt[0]),
            "east_m": float(pt[1]),
            "lat": float(gt.lat),
            "lon": float(gt.lon),
        })
    return out


def _trajectory_run_metadata(run) -> dict:
    """Segment metadata for one app-planned run — must-hit provenance only.

    Returns {} when the app declares no ``must_hit_indices``, so the engine's
    all-vertex fallback stands and the planned geometry is byte-identical to
    before the field existed.

    Indices are validated against the run's own point count and de-duplicated;
    an out-of-range index is dropped rather than raising, because a stale index
    must never block a mission whose geometry is otherwise sound. Declaring an
    EMPTY list is meaningful and is preserved — it says "nothing here is
    must-hit", which is different from declaring nothing at all.
    """
    raw = getattr(run, "must_hit_indices", None)
    if raw is None:
        return {}
    n = len(run.points)
    return {"control_indices": sorted({i for i in raw if 0 <= i < n})}


@path_router.post("/plan-trajectory", response_model=PlanTrajectoryResponse)
async def plan_trajectory(req: PlanTrajectoryRequest):
    """Densify and stage an app-planned trajectory. No file, no re-planning.

    The line-mission analogue of the point-mission bypass in /plan-and-stage:
    the caller's geometry is already final, so the planner is skipped as a
    PLANNER and used only as a densifier. Everything downstream —
    GET /staged/{mission_id}, /load-to-controller, /api/mission/start — is
    unchanged and reused verbatim.

    THE CONTRACT, in one sentence: the only thing this endpoint may do to the
    caller's geometry is insert points along it. Point count changes; shape,
    order and run boundaries do not.
    """
    warnings = _validate_trajectory(req)
    ground_truth = _resolve_ground_truth(req)

    # Label only — no file is read or written. Basenamed anyway because it lands
    # in the staged artifact's metadata.source, which _source_detail resolves
    # against MISSION_DIR.
    safe = os.path.basename(req.mission_name.strip())
    if not safe:
        raise HTTPException(422, "mission_name is empty after normalisation")

    segments = [
        PathSegment(
            segment_type=(SegmentType.MARK if run.kind == "mark"
                          else SegmentType.TRANSIT),
            points=[(float(n), float(e)) for n, e in run.points],
            speed=float(run.speed_m_s),
            segment_id=i,
            source_entity=run.label or f"run_{i}",
            # Still NO geometry metadata, deliberately. Both fit passes
            # (fit_arcs, fillet_corners) and the smoothing skip-list are gated
            # on metadata["geometry_type"] == "LINE_CHAIN"; carrying none keeps
            # those gates shut regardless of the flags below.
            #
            # control_indices is the ONE exception and it is not geometry: the
            # engine reads it purely as must-hit provenance (engine.py merge
            # step), so declaring it cannot change the planned shape — only
            # which points the RPP simplifier is forbidden to delete.
            # Omitted when the app declares nothing, so the engine's
            # all-vertex fallback stands and behaviour is byte-identical to
            # before this field existed.
            metadata=_trajectory_run_metadata(run),
        )
        for i, run in enumerate(req.runs)
    ]

    # Densify-only configuration. Every geometry-modifying pass is off HERE, at
    # the call site — not defaulted, and not reachable from the request body.
    #
    #   optimize_order=False        the app decided the order
    #   group_shapes=False          two touching runs must stay two runs. The
    #                               PathEngine default stays True; DXF depends
    #                               on it and is not touched.
    #   corner_smooth_radius_m=0    would re-smooth already-fitted arcs
    #   fit_arcs=False              already fitted
    #   fillet_corners_m=0          already filleted
    #   close_shape=False           shape closure is the app's decision
    #   enable_path_extensions=False PRE/AFT is a DXF concept; travel legs here
    #                               are explicit runs
    #   compensate_spray=False      runtime compensation only (spray node)
    #
    # close_loop is a plan_segments argument, not an __init__ kwarg — passed
    # False below, or the engine would append a closing TRANSIT leg nobody asked
    # for.
    engine = PathEngine(
        mark_spacing=req.line_spacing,
        transit_spacing=req.transit_spacing,
        marking_speed=req.marking_speed,
        transit_speed=req.transit_speed,
        optimize_order=False,
        group_shapes=False,
        corner_smooth_radius_m=0.0,
        fit_arcs=False,
        fillet_corners_m=0.0,
        close_shape=False,
        enable_path_extensions=False,
        compensate_spray=False,
    )

    origin_gps = (float(req.origin_gps[0]), float(req.origin_gps[1]))
    try:
        plan = await asyncio.wait_for(
            _plan_in_thread(
                engine.plan_segments,
                segments,
                origin=(0.0, 0.0),
                origin_gps=origin_gps,
                close_loop=False,
            ),
            # This only densifies, so it should finish far inside the budget.
            # Bounded anyway: a pathological payload fails cleanly instead of
            # holding a worker.
            timeout=15.0,
        )
    except asyncio.TimeoutError:
        raise HTTPException(504, "Densification timed out (15s limit)")
    except Exception as exc:
        raise HTTPException(422, f"Planning error: {exc}")

    run_echo = _trajectory_run_echo(req.runs, plan)

    # Step 1c (coincident-MARK drop) runs unconditionally in the engine and has
    # no flag. Rather than add one, surface it: the removal is now VISIBLE in
    # warnings (it was only ever in planning_metadata), and the run count in
    # run_echo drops too, which the client fails closed on.
    dropped = ((plan.planning_metadata or {}).get("duplicate_geometry")
               or {}).get("removed") or 0
    if dropped:
        warnings.append(
            f"{dropped} coincident mark run(s) were dropped by the planner as "
            "duplicate geometry — a deliberate double pass will NOT be marked "
            "twice. Offset the second pass or send it as one run."
        )

    alignment_meta = dict(plan.alignment_metadata or {})

    # The engine's gps_origin branch emits method/scale/rotation/offsets/origin
    # but NOT rmse or fitted_scale (path_engine/engine.py — unlike the
    # least_squares branch, there is no fit to residual against). The change
    # request asserts they are already there; they are not. Fill them in HERE
    # rather than in the engine, which the DXF flow shares:
    #   - rmse feeds MissionSummary.rmse_m and the client reads it back;
    #   - fitted_scale is what _assert_alignment_scale gates on.
    # 0.0 / 1.0 are the truthful values: a single GPS origin is an exact
    # translation, so there is no residual and no free scale to disagree with.
    alignment_meta.setdefault("rmse", 0.0)
    alignment_meta.setdefault("fitted_scale", 1.0)

    result = {
        "source": safe,
        "num_waypoints": len(plan.merged_waypoints),
        # Counts the terminal run-out, so the client's waypoint arithmetic and
        # this figure agree.
        "num_segments": len(run_echo),
        "mark_length_m": round(plan.total_mark_length, 3),
        "transit_length_m": round(plan.total_transit_length, 3),
        "total_length_m": round(plan.total_mark_length + plan.total_transit_length, 3),
        "segments": [
            {
                "runtime_segment_index": idx,
                "runtime_sequence": idx + 1,
                "type": "MARK" if seg.segment_type == SegmentType.MARK else "TRANSIT",
                "segment_role": "mark" if seg.segment_type == SegmentType.MARK else "transit",
                "source": seg.source_entity,
                "parent_source_entity": seg.source_entity,
                "parent_entity_id": None,
                "order_source": "app_trajectory",
                "is_extension": False,
                "speed": seg.speed,
                "length_m": round(seg.length, 3),
            }
            for idx, seg in enumerate(plan.segments)
        ],
        "merged_waypoints": [list(p) for p in plan.merged_waypoints],
        "spray_flags": list(plan.spray_flags),
        "must_hit": list(plan.must_hit or []),
        "alignment_metadata": alignment_meta,
        "planning_metadata": plan.planning_metadata or {},
        "warnings": warnings,
    }

    # plan_segments with origin_gps produces method="gps_origin", rmse 0.0,
    # scale 1.0 — the same alignment block the point-mission bypass synthesizes
    # by hand. Assert it rather than trust it: _stage_mission keys placement
    # mode off origin_gps, and a mission staged LOCAL_NED would silently skip
    # the surveyed re-bind at start.
    _assert_alignment_scale(alignment_meta)
    _assert_origin_gps_usable(alignment_meta.get("origin_gps") or list(origin_gps))

    mission_summary = None
    if result["merged_waypoints"]:
        mission_summary = _stage_mission(
            req, result, alignment_meta, alignment_meta.get("rmse", 0.0),
            survey_ground_truth=ground_truth,
        )

    return PlanTrajectoryResponse(
        source=result["source"],
        num_waypoints=result["num_waypoints"],
        num_segments=result["num_segments"],
        mark_length_m=result["mark_length_m"],
        transit_length_m=result["transit_length_m"],
        total_length_m=result["total_length_m"],
        segments=result["segments"],
        merged_waypoints=result["merged_waypoints"],
        spray_flags=result["spray_flags"],
        must_hit=result["must_hit"],
        alignment_metadata=alignment_meta or None,
        planning_metadata=result["planning_metadata"],
        warnings=warnings or None,
        mission_summary=mission_summary,
        run_echo=run_echo,
    )


@path_router.get("/staged/{mission_id}", response_model=StagedMissionResponse)
async def get_staged_mission(mission_id: str):
    """Stage 9 verify — return the exact staged mission artifact."""
    safe_id = os.path.basename(mission_id)
    staging_file = os.path.join(STAGING_DIR, f"{safe_id}.json")
    if not os.path.isfile(staging_file):
        raise HTTPException(404, "Staged mission not found or expired.")

    # Enforce the same TTL the loader and pruner use: an expired artifact is
    # treated as missing (and pruned) rather than handed back as if still valid.
    try:
        age = time.time() - os.path.getmtime(staging_file)
    except OSError:
        raise HTTPException(404, "Staged mission not found or expired.")
    if age > STAGING_TTL_S:
        _prune_staging()
        raise HTTPException(404, "Staged mission expired.")

    try:
        staged = await asyncio.to_thread(_read_json, staging_file)
    except (OSError, ValueError) as exc:
        raise HTTPException(422, f"Could not read staged mission: {exc}")
    if not isinstance(staged, dict):
        raise HTTPException(422, "Malformed staged mission (not an object).")

    waypoints = staged.get("waypoints", []) or []
    spray_flags = staged.get("spray_flags", []) or []
    try:
        wp_out = [[float(p[0]), float(p[1])] for p in waypoints]
    except (TypeError, ValueError, IndexError, KeyError) as exc:
        raise HTTPException(422, f"Malformed staged waypoints: {exc}")

    return StagedMissionResponse(
        mission_id=staged.get("mission_id", safe_id),
        created_at=staged.get("created_at"),
        anchor=staged.get("anchor"),
        num_waypoints=len(wp_out),
        waypoints=wp_out,
        spray_flags=[bool(f) for f in spray_flags],
        must_hit=[bool(f) for f in (staged.get("must_hit", []) or [])],
        segment_runs=_spray_runs(wp_out, spray_flags),
        alignment_metadata=staged.get("alignment_metadata"),
        metadata=staged.get("metadata"),
    )


# ── Delete ─────────────────────────────────────────────────────────────────────

@path_router.delete("/{filename}")
async def delete_path(filename: str):
    from main import path_mgr
    if not path_mgr.delete_file(filename):
        raise HTTPException(404, f"File not found: {filename!r}")
    return {"deleted": filename}
