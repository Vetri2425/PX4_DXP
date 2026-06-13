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
    SPRAY_DEFAULT_ON,
    SPRAY_LITERS_PER_METER,
    STAGING_DIR,
    STAGING_TTL_S,
)
from models import (
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
    MissionSummary,
    PathExtensionConfig,
    PathExtensionConfigResponse,
    PathPlanRequest,
    PathPlanResponse,
    PathPreviewBounds,
    PathPreviewResponse,
    PathPublishRequest,
)
from path_manager import UploadValidationError

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


def _entity_extension_preview(
    ent,
    preview_pts: list[tuple[float, float]],
    enabled: bool,
    is_mark: bool,
    pre_extension_m: float,
    aft_extension_m: float,
) -> EntityExtensionPreview:
    # Direction math is shared with the planner (analytic arc tangents,
    # finite differences for line-like geometry) so the preview cannot
    # drift from what split_mark_segment_with_extensions() actually plans.
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
    if pre_extension_m > 0:
        start = preview_pts[0]
        pre_points = [
            _ned_point(offset_point(start, start_dir, -pre_extension_m)),
            _ned_point(start),
        ]
    if aft_extension_m > 0:
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
    """Reorder entities according to saved entity order.

    If *saved_order* is empty, return parser order unchanged.
    Entities present in *saved_order* appear in that order.
    Any entities not in *saved_order* are appended at the end
    in parser order. Saved IDs that no longer exist in the DXF
    are silently ignored. No duplicates.
    """
    if not saved_order:
        return list(entities)

    by_id = {ent.entity_id: ent for ent in entities}
    used: set[str] = set()
    ordered: list = []

    for entity_id in saved_order:
        ent = by_id.get(entity_id)
        if ent is not None and entity_id not in used:
            ordered.append(ent)
            used.add(entity_id)

    for ent in entities:
        if ent.entity_id not in used:
            ordered.append(ent)
            used.add(ent.entity_id)

    return ordered


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
    # (entity_id, first_pt, last_pt) per drawable MARK entity — endpoints
    # only, so large per-entity point lists aren't retained past the loop.
    mark_endpoints: list[tuple[str, tuple[float, float], tuple[float, float]]] = []
    for order_index, ent in enumerate(entities):
        pts = _entity_preview_tuples(ent)
        all_pts.extend(pts)
        default_is_mark = ent.is_mark()
        is_mark = overrides.get(ent.entity_id, default_is_mark)
        if is_mark and pts:
            mark_endpoints.append((ent.entity_id, pts[0], pts[-1]))
        extension_preview = _entity_extension_preview(
            ent,
            pts,
            enabled=extension_config.enabled,
            is_mark=is_mark,
            pre_extension_m=extension_config.pre_extension_m,
            aft_extension_m=extension_config.aft_extension_m,
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
            preview_points=[_ned_point(pt) for pt in pts],
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
        name, req.enabled, req.pre_extension_m, req.aft_extension_m,
        what="Saving extension config",
    )
    return PathExtensionConfigResponse(
        name=os.path.basename(name),
        saved=True,
        **config,
    )


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
    staged_payload = {
        "anchor": anchor,
        "mission_id": mission_id,
        "created_at": time.time(),
        "waypoints": result.get("merged_waypoints", []),
        "spray_flags": result.get("spray_flags", []),
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

    try:
        spray_flags = [bool(f) for f in staged.get("spray_flags", [])]
        offboard_ctrl.load_path(waypoints, name=safe_id, spray_flags=spray_flags)
    except Exception as exc:
        raise HTTPException(409, f"Controller load failed: {exc}")

    return {
        "status": "success",
        "mission_id": safe_id,
        "num_waypoints": len(waypoints),
        "anchor_loaded": anchor is not None,
    }


# ── Delete ─────────────────────────────────────────────────────────────────────

@path_router.delete("/{filename}")
async def delete_path(filename: str):
    from main import path_mgr
    if not path_mgr.delete_file(filename):
        raise HTTPException(404, f"File not found: {filename!r}")
    return {"deleted": filename}
