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

from fastapi import APIRouter, Depends, File, HTTPException, UploadFile

from auth import require_token
from config import (
    MAX_UPLOAD_BYTES,
    MISSION_DIR,
    RMSE_MAX,
    SCALE_FIT_TOLERANCE,
    SPRAY_DEFAULT_ON,
    SPRAY_LITERS_PER_METER,
    STAGING_DIR,
    STAGING_TTL_S,
)
from spray_mission_config import (
    apply_spray_mission_config,
    is_point_mode_staged,
    next_configuration_revision,
    path_geometry_fingerprint,
    staged_spray_defaults,
    validate_staged_spray_config,
)

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
    EntityOrderUpdateRequest,
    EntityOrderUpdateResponse,
    EntityTransitPreview,
    LoadMissionRequest,
    LoadedPathResponse,
    MissionState,
    MissionSummary,
    PathExtensionConfig,
    PathExtensionConfigResponse,
    PathPlanRequest,
    PathPlanResponse,
    PathPreviewBounds,
    PathPreviewResponse,
    PathPublishRequest,
    PathSegmentsResponse,
    RefPointResidual,
    SegmentInfo,
    StagedMissionResponse,
)
from path_manager import UploadValidationError
from path_engine.entity_order import apply_entity_order as _apply_entity_order_shared
from mission_placement import GPS_SURVEYED, LOCAL_NED, PlacementError, align_design_points

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


def _extension_endpoint_freeness(
    endpoints: list[tuple[tuple[float, float], tuple[float, float]]],
    tol: float = _EXTENSION_JUNCTION_TOL_M,
) -> list[tuple[bool, bool]]:
    """Per mark entity (start, end), decide whether each end is a FREE end.

    An end is *free* when it does not coincide with any OTHER mark entity's
    endpoint — i.e. it is the outer end of a chain, not an internal junction.
    A self-closed entity (start ≈ end) has no free end.

    This mirrors the vertex-anchored planner policy: extensions live only at a
    chain's true open ends, never at internal corners or on closed loops. Doing
    it by endpoint connectivity (not entity order) means preview and plan agree
    even though the preview keeps DXF order while the planner reorders via TSP.
    """
    def _shared(pt, skip_idx) -> bool:
        for j, (s, e) in enumerate(endpoints):
            if j == skip_idx:
                continue
            if math.hypot(pt[0] - s[0], pt[1] - s[1]) <= tol:
                return True
            if math.hypot(pt[0] - e[0], pt[1] - e[1]) <= tol:
                return True
        return False

    freeness = []
    for i, (start, end) in enumerate(endpoints):
        if math.hypot(start[0] - end[0], start[1] - end[1]) <= tol:
            freeness.append((False, False))  # self-closed loop — no free end
            continue
        freeness.append((not _shared(start, i), not _shared(end, i)))
    return freeness


def _entity_extension_preview(
    ent,
    preview_pts: list[tuple[float, float]],
    enabled: bool,
    is_mark: bool,
    pre_extension_m: float,
    aft_extension_m: float,
    start_is_free: bool = True,
    end_is_free: bool = True,
) -> EntityExtensionPreview:
    # Direction math is shared with the planner (analytic arc tangents,
    # finite differences for line-like geometry) so the preview cannot
    # drift from what split_mark_segment_with_extensions() actually plans.
    # start_is_free/end_is_free gate WHERE a run-up may appear: only at a
    # chain's true open ends, matching the vertex-anchored planner — an
    # internal corner or a closed loop yields no preview extension.
    from path_engine.planners.extensions import (
        entity_extension_directions,
        offset_point,
    )

    if not enabled or not is_mark or len(preview_pts) < 2:
        return EntityExtensionPreview(enabled=False)

    dirs = entity_extension_directions(ent, preview_pts)
    if dirs is None:
        return EntityExtensionPreview(enabled=False)
    start_dir, end_dir = dirs

    pre_points = []
    aft_points = []
    if pre_extension_m > 0 and start_is_free:
        start = preview_pts[0]
        pre_points = [
            _ned_point(offset_point(start, start_dir, -pre_extension_m)),
            _ned_point(start),
        ]
    if aft_extension_m > 0 and end_is_free:
        end = preview_pts[-1]
        aft_points = [
            _ned_point(end),
            _ned_point(offset_point(end, end_dir, aft_extension_m)),
        ]

    return EntityExtensionPreview(
        enabled=bool(pre_points or aft_points),
        pre_length_m=pre_extension_m if pre_points else 0.0,
        aft_length_m=aft_extension_m if aft_points else 0.0,
        pre_points=pre_points,
        aft_points=aft_points,
    )


def _entity_transit_previews(
    mark_endpoints: list[tuple[str, tuple[float, float], tuple[float, float]]],
) -> list[EntityTransitPreview]:
    """Straight no-spray connectors between consecutive MARK entities.

    *mark_endpoints* is (entity_id, first_pt, last_pt) per drawable MARK
    entity, in DXF/entity order. Callers must already have dropped entities
    with no preview points, so a degenerate entity cannot break the chain —
    its drawable neighbours still get connected, like the planner would.
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


def _assert_alignment_scale(alignment_meta: dict) -> None:
    """Reject an alignment whose least-squares scale strays too far from unity.

    Ref points and segment geometry share a metric frame, so a healthy multi-point
    fit lands scale≈1.0. A large deviation signals a unit/frame mismatch (e.g. the
    historical double-scaling of cm ref points → scale≈100). A 2-point fit is
    exactly determined so its RMSE is ~0 and the RMSE gate cannot catch this —
    this scale gate is the only defense. single_point/gps_origin modes report
    scale=1.0 and pass by definition.
    """
    scale = alignment_meta.get("scale", 1.0)
    if not math.isfinite(scale) or scale <= 0.0:
        raise HTTPException(
            422,
            f"Alignment produced a non-physical scale ({scale}). "
            "Re-verify the reference points.",
        )
    if abs(scale - 1.0) > SCALE_FIT_TOLERANCE:
        raise HTTPException(
            422,
            f"Alignment scale {scale:.4f} is outside the safe range "
            f"[{1.0 - SCALE_FIT_TOLERANCE:.2f}, {1.0 + SCALE_FIT_TOLERANCE:.2f}] — "
            "likely a unit/frame mismatch between reference points and geometry. "
            "Re-verify the reference points.",
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
    resolved: list[tuple] = []  # (ent, preview_pts, tangent_pts, default_is_mark, is_mark)
    mark_endpoints_for_freeness: list[tuple[tuple[float, float], tuple[float, float]]] = []
    mark_slot_of_entity: dict[int, int] = {}
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
        resolved.append((ent, preview_pts, tangent_pts, default_is_mark, is_mark))
        if is_mark and tangent_pts:
            mark_slot_of_entity[idx] = len(mark_endpoints_for_freeness)
            mark_endpoints_for_freeness.append((tangent_pts[0], tangent_pts[-1]))

    freeness = _extension_endpoint_freeness(mark_endpoints_for_freeness)

    # (entity_id, first_pt, last_pt) per drawable MARK entity — endpoints
    # only, so large per-entity point lists aren't retained past the loop.
    mark_endpoints: list[tuple[str, tuple[float, float], tuple[float, float]]] = []
    for order_index, (ent, preview_pts, tangent_pts, default_is_mark, is_mark) in enumerate(resolved):
        all_pts.extend(preview_pts)
        if is_mark and tangent_pts:
            mark_endpoints.append((ent.entity_id, tangent_pts[0], tangent_pts[-1]))
        slot = mark_slot_of_entity.get(order_index)
        start_is_free, end_is_free = freeness[slot] if slot is not None else (True, True)
        extension_preview = _entity_extension_preview(
            ent,
            tangent_pts,
            enabled=extension_config.enabled,
            is_mark=is_mark,
            pre_extension_m=extension_config.pre_extension_m,
            aft_extension_m=extension_config.aft_extension_m,
            start_is_free=start_is_free,
            end_is_free=end_is_free,
        )
        for ext_pt in extension_preview.pre_points + extension_preview.aft_points:
            all_pts.append((ext_pt.north, ext_pt.east))
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
    transit_preview = _entity_transit_previews(mark_endpoints)

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

    return DXFEntitiesResponse(
        name=safe,
        num_entities=len(previews),
        bounds=bounds,
        extension_config=extension_config,
        transit_preview=transit_preview,
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
    valid_ids = [ent.entity_id for ent in entities]
    valid_set = set(valid_ids)
    posted = req.entity_order
    posted_set = set(posted)

    # Full-order contract: must contain exactly the current entity ID set.
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
    if os.path.splitext(fpath)[1].lower() != ".dxf":
        raise ValueError("Path extensions are only configurable for DXF files")
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


@path_router.post("/parse-point-csv")
async def parse_point_csv(file: UploadFile = File(...)):
    """Parse a point-mission CSV (north,east[,dwell_s]) into staged-ready points."""
    import sys
    from pathlib import Path as FsPath

    src = FsPath(__file__).resolve().parents[2] / "src"
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))
    from point_ingest import parse_point_csv_text, points_to_staged_dict
    from spray_config import staged_spray_defaults

    content = (await file.read()).decode("utf-8", errors="replace")
    defaults = staged_spray_defaults()
    try:
        points = parse_point_csv_text(
            content,
            default_dwell_s=float(defaults["point_default_dwell_s"]),
            max_dwell_s=float(defaults["point_max_dwell_s"]),
        )
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    return {
        "num_points": len(points),
        "point_mission_points": points_to_staged_dict(points),
    }


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
    from main import offboard_ctrl, ros_node, path_mgr
    if ros_node is None:
        raise HTTPException(503, "ROS node not ready")
    name = req.name or req.file
    if not name:
        raise HTTPException(400, "Provide name or file")
    if offboard_ctrl is not None and (
        offboard_ctrl.has_protected_mission
        or offboard_ctrl.state in {
            MissionState.LOADING,
            MissionState.ARMING,
            MissionState.SWITCHING_OFFBOARD,
            MissionState.RUNNING,
            MissionState.STOPPING,
            MissionState.DISARMING,
        }
    ):
        raise HTTPException(
            409,
            "Cannot publish a diagnostic path while a protected mission is loaded "
            f"or controller is {offboard_ctrl.state.value}",
        )
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
        entities = parse_dxf(fpath)

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

        return DXFParseResponse(
            filename=safe,
            num_entities=len(entities),
            entities=entity_infos,
            unit_scale=unit_scale,
            layer_names=sorted(layer_names),
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

    origin = tuple(req.origin) if req.origin else (0.0, 0.0)
    start_position = tuple(req.start_position) if req.start_position else None
    summary_only = not (req.include_waypoints)
    origin_gps = tuple(req.origin_gps) if req.origin_gps else None
    ref_points_dxf = [(pt.dxf_y, pt.dxf_x) for pt in req.ref_points] if req.ref_points is not None else None
    ref_points_gps = [(pt.lat, pt.lon) for pt in req.ref_points] if req.ref_points is not None else None

    try:
        result = await asyncio.wait_for(
            asyncio.to_thread(
                path_mgr.plan_path,
                req.source,
                summary_only=summary_only,
                line_spacing=req.line_spacing,
                transit_spacing=req.transit_spacing,
                marking_speed=req.marking_speed,
                transit_speed=req.transit_speed,
                layer_mapping=req.layer_mapping,
                optimize=req.optimize,
                compensate_spray=req.compensate_spray,
                # Extension settings are configured per DXF via
                # GET/POST /api/path/{name}/extensions, then loaded by
                # PathManager during planning.
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

    if deprecation_warning:
        result["warnings"] = list(result.get("warnings") or []) + [deprecation_warning]

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
    _assert_alignment_scale(alignment_meta)

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


def _stage_mission(req: PathPlanRequest, result: dict, alignment_meta: dict,
                   rmse: float) -> MissionSummary:
    """Write the aligned mission to a staging file and return its summary.

    The staged artifact is the single source of truth for the subsequent
    /load-to-controller step, so the operator loads exactly what was previewed.
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
    spray_defaults = staged_spray_defaults()
    configuration_revision = next_configuration_revision()
    path_fingerprint = path_geometry_fingerprint(
        [(float(p[0]), float(p[1])) for p in result.get("merged_waypoints", [])],
        [bool(f) for f in result.get("spray_flags", [])],
    )

    # Spray field resolution:
    #   req.spray_mode is None  → use per-path sidecar written by /spray-mode endpoints
    #   req.spray_mode explicit → use req body fields directly (legacy, backward-compat)
    from spray_mode_store import load_spray_mode
    if req.spray_mode is None:
        _spray = load_spray_mode(MISSION_DIR, req.source or "")
    else:
        # Legacy plan body carries no continuous-mode timing fields, so the
        # explicit-spray-mode flow keeps factory continuous params (unchanged
        # behavior). Per-path continuous config is set via the /spray-mode
        # sidecar endpoints and only takes effect through the sidecar branch.
        _spray = {
            "spray_mode": req.spray_mode,
            "solenoid_open_delay_s": spray_defaults["solenoid_open_delay_s"],
            "solenoid_close_delay_s": spray_defaults["solenoid_close_delay_s"],
            "on_overspray_margin_m": spray_defaults["on_overspray_margin_m"],
            "off_overspray_margin_m": spray_defaults["off_overspray_margin_m"],
            "min_spray_speed_mps": spray_defaults["min_spray_speed_mps"],
            "max_spray_speed_mps": spray_defaults["max_spray_speed_mps"],
            "unsafe_speed_behavior": spray_defaults["unsafe_speed_behavior"],
            "max_xtrack_error_m": spray_defaults["max_xtrack_error_m"],
            "nozzle_forward_offset_m": spray_defaults["nozzle_forward_offset_m"],
            "nozzle_lateral_offset_m": spray_defaults["nozzle_lateral_offset_m"],
            "dash_on_distance_m": req.dash_on_distance_m,
            "dash_off_distance_m": req.dash_off_distance_m,
            "dash_phase_reset": req.dash_phase_reset,
            "point_default_dwell_s": req.point_default_dwell_s,
            "point_max_dwell_s": req.point_max_dwell_s,
            "point_arrival_tolerance_m": req.point_arrival_tolerance_m,
            "point_settle_time_s": req.point_settle_time_s,
            "point_leg_timeout_s": req.point_leg_timeout_s,
            "point_settle_speed_mps": req.point_settle_speed_mps,
            "point_settle_yaw_rate_rad_s": req.point_settle_yaw_rate_rad_s,
            "point_execution_mode": req.point_execution_mode,
            "point_leg_trajectory_mode": req.point_leg_trajectory_mode,
            "point_leg_spacing_m": req.point_leg_spacing_m,
            "point_hold_drift_tolerance_m": req.point_hold_drift_tolerance_m,
            "point_hold_drift_policy": req.point_hold_drift_policy,
            "gps_required_fix_type": req.gps_required_fix_type,
            "gps_global_position_max_age_ms": req.gps_global_position_max_age_ms,
            "gps_local_pose_max_age_ms": req.gps_local_pose_max_age_ms,
            "gps_fix_max_age_ms": req.gps_fix_max_age_ms,
            "gps_max_pose_global_skew_ms": req.gps_max_pose_global_skew_ms,
            "gps_runtime_policy": req.gps_runtime_policy,
            "gps_resume_policy": req.gps_resume_policy,
            "gps_recovery_stable_s": req.gps_recovery_stable_s,
            "obstacle_integration_enabled": req.obstacle_integration_enabled,
            "obstacle_signal_max_age_s": req.obstacle_signal_max_age_s,
            "calibration_profile_id": spray_defaults["calibration_profile_id"],
            "calibration_profile_version": spray_defaults["calibration_profile_version"],
            "target_paint_density": spray_defaults["target_paint_density"],
            "speed_pwm_table": spray_defaults["speed_pwm_table"],
            "actuator_min_pwm": spray_defaults["actuator_min_pwm"],
            "actuator_max_pwm": spray_defaults["actuator_max_pwm"],
            "actuator_off_pwm": spray_defaults["actuator_off_pwm"],
            "actuator_min_value": spray_defaults["actuator_min_value"],
            "actuator_max_value": spray_defaults["actuator_max_value"],
            "actuator_off_value": spray_defaults["actuator_off_value"],
            "timing_only_compatibility": spray_defaults["timing_only_compatibility"],
            "pump_inertia_enabled": spray_defaults["pump_inertia_enabled"],
            "pwm_ramp_prediction_enabled": spray_defaults["pwm_ramp_prediction_enabled"],
            "pressure_stabilization_enabled": spray_defaults["pressure_stabilization_enabled"],
            "temperature_viscosity_compensation_enabled": (
                spray_defaults["temperature_viscosity_compensation_enabled"]
            ),
        }

    point_rows = list(req.point_mission_points or spray_defaults["point_mission_points"])
    original_point_rows = [dict(row) for row in point_rows]
    point_source_frame = req.point_source_frame
    if point_rows and point_source_frame == "DESIGN":
        try:
            aligned = align_design_points(
                [(float(row["north_m"]), float(row["east_m"])) for row in point_rows],
                alignment_meta,
            )
        except (KeyError, TypeError, ValueError, PlacementError) as exc:
            raise HTTPException(422, f"Point coordinate alignment failed: {exc}") from exc
        point_rows = [
            {**row, "north_m": n, "east_m": e}
            for row, (n, e) in zip(point_rows, aligned)
        ]
        point_source_frame = GPS_SURVEYED
    elif point_rows and point_source_frame == GPS_SURVEYED and anchor is None:
        raise PlacementError("GPS_SURVEYED Point coordinates require a mission anchor")

    staged_payload = {
        "anchor": anchor,
        "mission_id": mission_id,
        "created_at": time.time(),
        "waypoints": result.get("merged_waypoints", []),
        "spray_flags": result.get("spray_flags", []),
        "path_fingerprint": path_fingerprint,
        "configuration_revision": configuration_revision,
        "spray_mode": _spray["spray_mode"],
        "solenoid_open_delay_s": _spray.get("solenoid_open_delay_s", spray_defaults["solenoid_open_delay_s"]),
        "solenoid_close_delay_s": _spray.get("solenoid_close_delay_s", spray_defaults["solenoid_close_delay_s"]),
        "on_overspray_margin_m": _spray.get("on_overspray_margin_m", spray_defaults["on_overspray_margin_m"]),
        "off_overspray_margin_m": _spray.get("off_overspray_margin_m", spray_defaults["off_overspray_margin_m"]),
        "min_spray_speed_mps": _spray.get("min_spray_speed_mps", spray_defaults["min_spray_speed_mps"]),
        "max_spray_speed_mps": _spray.get("max_spray_speed_mps", spray_defaults["max_spray_speed_mps"]),
        "unsafe_speed_behavior": _spray.get("unsafe_speed_behavior", spray_defaults["unsafe_speed_behavior"]),
        "max_xtrack_error_m": _spray.get("max_xtrack_error_m", spray_defaults["max_xtrack_error_m"]),
        "nozzle_forward_offset_m": _spray.get("nozzle_forward_offset_m", spray_defaults["nozzle_forward_offset_m"]),
        "nozzle_lateral_offset_m": _spray.get("nozzle_lateral_offset_m", spray_defaults["nozzle_lateral_offset_m"]),
        "dash_on_distance_m": _spray["dash_on_distance_m"],
        "dash_off_distance_m": _spray["dash_off_distance_m"],
        "dash_phase_reset": _spray["dash_phase_reset"],
        "point_default_dwell_s": _spray["point_default_dwell_s"],
        "point_max_dwell_s": _spray["point_max_dwell_s"],
        "point_arrival_tolerance_m": _spray["point_arrival_tolerance_m"],
        "point_settle_time_s": _spray["point_settle_time_s"],
        "point_leg_timeout_s": _spray["point_leg_timeout_s"],
        "point_settle_speed_mps": _spray["point_settle_speed_mps"],
        "point_settle_yaw_rate_rad_s": _spray["point_settle_yaw_rate_rad_s"],
        "point_execution_mode": _spray["point_execution_mode"],
        "point_leg_trajectory_mode": _spray["point_leg_trajectory_mode"],
        "point_leg_spacing_m": _spray["point_leg_spacing_m"],
        "point_hold_drift_tolerance_m": _spray["point_hold_drift_tolerance_m"],
        "point_hold_drift_policy": _spray["point_hold_drift_policy"],
        "point_mission_points": point_rows,
        "point_mission_points_original": original_point_rows,
        "point_source_frame": point_source_frame,
        "gps_required_fix_type": _spray["gps_required_fix_type"],
        "gps_global_position_max_age_ms": _spray["gps_global_position_max_age_ms"],
        "gps_local_pose_max_age_ms": _spray["gps_local_pose_max_age_ms"],
        "gps_fix_max_age_ms": _spray["gps_fix_max_age_ms"],
        "gps_max_pose_global_skew_ms": _spray["gps_max_pose_global_skew_ms"],
        "gps_runtime_policy": _spray["gps_runtime_policy"],
        "gps_resume_policy": _spray["gps_resume_policy"],
        "gps_recovery_stable_s": _spray["gps_recovery_stable_s"],
        "obstacle_integration_enabled": _spray.get("obstacle_integration_enabled", False),
        "obstacle_signal_max_age_s": _spray.get("obstacle_signal_max_age_s", 2.0),
        "calibration_profile_id": _spray.get("calibration_profile_id", spray_defaults["calibration_profile_id"]),
        "calibration_profile_version": _spray.get("calibration_profile_version", spray_defaults["calibration_profile_version"]),
        "target_paint_density": _spray.get("target_paint_density", spray_defaults["target_paint_density"]),
        "speed_pwm_table": _spray.get("speed_pwm_table", spray_defaults["speed_pwm_table"]),
        "actuator_min_pwm": _spray.get("actuator_min_pwm", spray_defaults["actuator_min_pwm"]),
        "actuator_max_pwm": _spray.get("actuator_max_pwm", spray_defaults["actuator_max_pwm"]),
        "actuator_off_pwm": _spray.get("actuator_off_pwm", spray_defaults["actuator_off_pwm"]),
        "actuator_min_value": _spray.get("actuator_min_value", spray_defaults["actuator_min_value"]),
        "actuator_max_value": _spray.get("actuator_max_value", spray_defaults["actuator_max_value"]),
        "actuator_off_value": _spray.get("actuator_off_value", spray_defaults["actuator_off_value"]),
        "timing_only_compatibility": _spray.get("timing_only_compatibility", spray_defaults["timing_only_compatibility"]),
        "pump_inertia_enabled": _spray.get("pump_inertia_enabled", spray_defaults["pump_inertia_enabled"]),
        "pwm_ramp_prediction_enabled": _spray.get("pwm_ramp_prediction_enabled", spray_defaults["pwm_ramp_prediction_enabled"]),
        "pressure_stabilization_enabled": _spray.get("pressure_stabilization_enabled", spray_defaults["pressure_stabilization_enabled"]),
        "temperature_viscosity_compensation_enabled": _spray.get(
            "temperature_viscosity_compensation_enabled",
            spray_defaults["temperature_viscosity_compensation_enabled"],
        ),
        "alignment_metadata": alignment_meta,
        "metadata": {
            "source": result["source"],
            "mark_length_m": result["mark_length_m"],
            "transit_length_m": result["transit_length_m"],
            "total_length_m": result["total_length_m"],
        },
    }

    staging_file = os.path.join(STAGING_DIR, f"{mission_id}.json")
    tmp = staging_file + ".tmp"
    with open(tmp, "w") as f:
        json.dump(staged_payload, f)
    os.replace(tmp, staging_file)  # atomic publish

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

    point_mode = is_point_mode_staged(staged)
    waypoints = [tuple(pt) for pt in staged.get("waypoints", [])]
    if not point_mode and not waypoints:
        raise HTTPException(422, "Staged mission has no waypoints.")
    if point_mode and not staged.get("point_mission_points"):
        raise HTTPException(422, "Point-mode staged mission has no point_mission_points.")

    anchor = staged.get("anchor")
    if anchor:
        import logging
        logging.getLogger("server.path").info(
            "loading mission %s with anchor lat=%.7f lon=%.7f rot=%.2f scale=%.4f",
            safe_id, anchor["lat"], anchor["lon"],
            anchor.get("rotation_deg", 0.0), anchor.get("scale", 1.0),
        )

    from main import point_mission, ros_node

    try:
        try:
            validated_spray_config = validate_staged_spray_config(staged)
        except ValueError as exc:
            raise HTTPException(422, f"Invalid spray configuration: {exc}") from exc
        ok, why, spray_config = await apply_spray_mission_config(ros_node, staged)
        spray_mode = str(staged.get("spray_mode", "continuous")).lower()
        spray_config_degraded = False
        if not ok:
            if spray_mode in {"dash", "point"}:
                raise HTTPException(503, f"Spray controller dependency unavailable: {why}")
            # Legacy and Continuous navigation retain their previous load
            # behavior. Spray stays under its existing operator gate, and the
            # response explicitly reports that mission config was not applied.
            spray_config_degraded = True
            spray_config = validated_spray_config

        spray_flags = [bool(f) for f in staged.get("spray_flags", [])]
        if spray_config_degraded:
            spray_flags = [False] * len(waypoints)
        origin_gps = None
        if anchor is not None:
            origin_gps = (anchor.get("lat"), anchor.get("lon"))
        source_name = (staged.get("metadata") or {}).get("source") or safe_id

        if point_mode:
            if point_mission is None:
                raise HTTPException(503, "Point mission orchestrator unavailable")
            await point_mission.replace_from_staged(staged, spray_config, ros_node)
            first = staged["point_mission_points"][0]
            load_points = [
                (float(first["north_m"]), float(first["east_m"])),
                (float(first["north_m"]), float(first["east_m"])),
            ]
            spray_flags = [False, False]
        else:
            if point_mission is not None:
                await point_mission.cancel_and_drain(ros_node, reason="non_point_load")
            load_points = waypoints

        offboard_ctrl.load_path(
            load_points,
            name=source_name,
            spray_flags=spray_flags,
            placement_mode=GPS_SURVEYED if anchor is not None else LOCAL_NED,
            origin_gps=origin_gps,
            mission_id=safe_id,
            source_name=source_name,
            is_staged=True,
            allow_replace_protected=True,
            spray_mode=staged.get("spray_mode", "continuous"),
            path_fingerprint=str(staged.get("path_fingerprint", "") or ""),
            configuration_revision=int(staged.get("configuration_revision", 0)),
        )
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(409, f"Controller load failed: {exc}")

    return {
        "status": "success",
        "mission_id": safe_id,
        "num_waypoints": len(waypoints),
        "anchor_loaded": anchor is not None,
        "spray_config_applied": not spray_config_degraded,
        "spray_config_degraded_reason": why if spray_config_degraded else "",
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
            asyncio.to_thread(
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
            asyncio.to_thread(
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

    try:
        result = await asyncio.wait_for(
            asyncio.to_thread(
                path_mgr.plan_path,
                safe,
                summary_only=False,
                line_spacing=req.line_spacing,
                transit_spacing=req.transit_spacing,
                marking_speed=req.marking_speed,
                transit_speed=req.transit_speed,
                layer_mapping=req.layer_mapping,
                optimize=req.optimize,
                compensate_spray=req.compensate_spray,
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
    _assert_alignment_scale(alignment_meta)

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
        alignment_metadata=alignment_meta or None,
        planning_metadata=result.get("planning_metadata"),
        warnings=warnings or None,
        mission_summary=mission_summary,
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

    spray_defaults = staged_spray_defaults()
    return StagedMissionResponse(
        mission_id=staged.get("mission_id", safe_id),
        created_at=staged.get("created_at"),
        anchor=staged.get("anchor"),
        num_waypoints=len(wp_out),
        waypoints=wp_out,
        spray_flags=[bool(f) for f in spray_flags],
        segment_runs=_spray_runs(wp_out, spray_flags),
        spray_mode=staged.get("spray_mode", spray_defaults["spray_mode"]),
        path_fingerprint=str(staged.get("path_fingerprint", "")),
        max_spray_speed_mps=float(
            staged.get("max_spray_speed_mps", spray_defaults["max_spray_speed_mps"])
        ),
        unsafe_speed_behavior=str(
            staged.get("unsafe_speed_behavior", spray_defaults["unsafe_speed_behavior"])
        ),
        calibration_profile_id=str(
            staged.get("calibration_profile_id", spray_defaults["calibration_profile_id"])
        ),
        calibration_profile_version=int(
            staged.get("calibration_profile_version", spray_defaults["calibration_profile_version"])
        ),
        target_paint_density=float(
            staged.get("target_paint_density", spray_defaults["target_paint_density"])
        ),
        speed_pwm_table=list(staged.get("speed_pwm_table", spray_defaults["speed_pwm_table"])),
        actuator_min_pwm=float(staged.get("actuator_min_pwm", spray_defaults["actuator_min_pwm"])),
        actuator_max_pwm=float(staged.get("actuator_max_pwm", spray_defaults["actuator_max_pwm"])),
        actuator_off_pwm=float(staged.get("actuator_off_pwm", spray_defaults["actuator_off_pwm"])),
        actuator_min_value=float(staged.get("actuator_min_value", spray_defaults["actuator_min_value"])),
        actuator_max_value=float(staged.get("actuator_max_value", spray_defaults["actuator_max_value"])),
        actuator_off_value=float(staged.get("actuator_off_value", spray_defaults["actuator_off_value"])),
        timing_only_compatibility=bool(
            staged.get("timing_only_compatibility", spray_defaults["timing_only_compatibility"])
        ),
        dash_on_distance_m=float(staged.get("dash_on_distance_m", spray_defaults["dash_on_distance_m"])),
        dash_off_distance_m=float(staged.get("dash_off_distance_m", spray_defaults["dash_off_distance_m"])),
        dash_phase_reset=staged.get("dash_phase_reset", spray_defaults["dash_phase_reset"]),
        point_default_dwell_s=float(staged.get("point_default_dwell_s", spray_defaults["point_default_dwell_s"])),
        point_max_dwell_s=float(staged.get("point_max_dwell_s", spray_defaults["point_max_dwell_s"])),
        point_arrival_tolerance_m=float(
            staged.get("point_arrival_tolerance_m", spray_defaults["point_arrival_tolerance_m"])
        ),
        point_settle_time_s=float(staged.get("point_settle_time_s", spray_defaults["point_settle_time_s"])),
        point_leg_timeout_s=float(staged.get("point_leg_timeout_s", spray_defaults["point_leg_timeout_s"])),
        point_settle_speed_mps=float(
            staged.get("point_settle_speed_mps", spray_defaults["point_settle_speed_mps"])
        ),
        point_settle_yaw_rate_rad_s=float(
            staged.get("point_settle_yaw_rate_rad_s", spray_defaults["point_settle_yaw_rate_rad_s"])
        ),
        point_execution_mode=str(
            staged.get("point_execution_mode", spray_defaults["point_execution_mode"])
        ),
        point_leg_trajectory_mode=str(
            staged.get(
                "point_leg_trajectory_mode",
                spray_defaults["point_leg_trajectory_mode"],
            )
        ),
        point_leg_spacing_m=float(
            staged.get("point_leg_spacing_m", spray_defaults["point_leg_spacing_m"])
        ),
        point_hold_drift_tolerance_m=float(
            staged.get(
                "point_hold_drift_tolerance_m",
                spray_defaults["point_hold_drift_tolerance_m"],
            )
        ),
        point_hold_drift_policy=str(
            staged.get("point_hold_drift_policy", spray_defaults["point_hold_drift_policy"])
        ),
        point_mission_points=list(staged.get("point_mission_points") or []),
        point_source_frame=str(staged.get("point_source_frame") or ""),
        point_mission_points_original=list(staged.get("point_mission_points_original") or []),
        gps_required_fix_type=int(
            staged.get("gps_required_fix_type", spray_defaults["gps_required_fix_type"])
        ),
        gps_global_position_max_age_ms=float(
            staged.get(
                "gps_global_position_max_age_ms",
                spray_defaults["gps_global_position_max_age_ms"],
            )
        ),
        gps_local_pose_max_age_ms=float(
            staged.get(
                "gps_local_pose_max_age_ms",
                spray_defaults["gps_local_pose_max_age_ms"],
            )
        ),
        gps_fix_max_age_ms=float(
            staged.get("gps_fix_max_age_ms", spray_defaults["gps_fix_max_age_ms"])
        ),
        gps_max_pose_global_skew_ms=float(
            staged.get(
                "gps_max_pose_global_skew_ms",
                spray_defaults["gps_max_pose_global_skew_ms"],
            )
        ),
        gps_runtime_policy=str(
            staged.get("gps_runtime_policy", spray_defaults["gps_runtime_policy"])
        ),
        gps_resume_policy=str(
            staged.get("gps_resume_policy", spray_defaults["gps_resume_policy"])
        ),
        gps_recovery_stable_s=float(
            staged.get("gps_recovery_stable_s", spray_defaults["gps_recovery_stable_s"])
        ),
        configuration_revision=int(staged.get("configuration_revision", 0)),
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
