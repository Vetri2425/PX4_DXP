"""Path management endpoints (auth-protected).

GET    /api/paths              — list built-in + uploaded paths
POST   /api/path/upload        — upload .waypoints, .csv, or .dxf
POST   /api/path/publish       — publish named path to /path topic
POST   /api/path/parse-dxf     — parse DXF file, return entity list
POST   /api/path/plan          — run full planning pipeline, return PlannedPath
DELETE /api/path/{filename}    — delete uploaded file
"""
from __future__ import annotations

import os

from fastapi import APIRouter, Depends, File, HTTPException, UploadFile

from auth import require_token
from config import MAX_UPLOAD_BYTES, MISSION_DIR
from models import PathPublishRequest, PathPlanRequest, DXFParseResponse, DXFEntityInfo, PathPlanResponse
from path_manager import UploadValidationError

# Two distinct routers so the URL structure is explicit and stable.
paths_router = APIRouter(prefix="/paths", tags=["path"],
                         dependencies=[Depends(require_token)])
path_router  = APIRouter(prefix="/path",  tags=["path"],
                         dependencies=[Depends(require_token)])


# ── Listing ───────────────────────────────────────────────────────────────────

@paths_router.get("")
async def list_paths():
    from main import path_mgr
    return [p.model_dump() for p in path_mgr.list_paths()]


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
    ros_node.publish_path(pts, frame_id=req.frame_id)
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

    # Save temporarily for parsing
    saved = path_mgr.save_uploaded(filename, content)
    fpath = os.path.join(MISSION_DIR, saved)

    try:
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
                length = 2 * 3.14159265 * ent.geometry.get("radius", 0)
            elif ent.entity_type == "ARC":
                r = ent.geometry.get("radius", 0)
                a1 = ent.geometry.get("start_angle", 0)
                a2 = ent.geometry.get("end_angle", 360)
                length = r * abs(a2 - a1) * 3.14159265 / 180

            entity_infos.append(DXFEntityInfo(
                entity_type=ent.entity_type,
                layer=ent.layer,
                color=ent.color,
                entity_id=ent.entity_id,
                is_mark=ent.is_mark(),
                length_m=round(length, 3),
            ))

        return DXFParseResponse(
            filename=saved,
            num_entities=len(entities),
            entities=entity_infos,
            unit_scale=unit_scale,
            layer_names=sorted(layer_names),
        )
    except ImportError:
        raise HTTPException(500, "ezdxf not installed. Run: pip install ezdxf")
    except Exception as exc:
        raise HTTPException(422, f"DXF parse error: {exc}")


# ── Plan ──────────────────────────────────────────────────────────────────────

@path_router.post("/plan")
async def plan_path(req: PathPlanRequest):
    """Run the full planning pipeline and return merged waypoints with spray flags."""
    from main import path_mgr

    origin = tuple(req.origin) if req.origin else (0.0, 0.0)
    start_position = tuple(req.start_position) if req.start_position else None

    try:
        result = path_mgr.plan_path(
            req.source,
            line_spacing=req.line_spacing,
            transit_spacing=req.transit_spacing,
            marking_speed=req.marking_speed,
            transit_speed=req.transit_speed,
            origin=origin,
            start_position=start_position,
        )
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except ImportError as exc:
        raise HTTPException(500, str(exc))
    except Exception as exc:
        raise HTTPException(422, f"Planning error: {exc}")

    return PathPlanResponse(
        source=result["source"],
        num_waypoints=result["num_waypoints"],
        num_segments=result["num_segments"],
        mark_length_m=result["mark_length_m"],
        transit_length_m=result["transit_length_m"],
        total_length_m=result["total_length_m"],
        segments=result["segments"],
        merged_waypoints=result["merged_waypoints"],
        spray_flags=result["spray_flags"],
    )


# ── Delete ─────────────────────────────────────────────────────────────────────

@path_router.delete("/{filename}")
async def delete_path(filename: str):
    from main import path_mgr
    if not path_mgr.delete_file(filename):
        raise HTTPException(404, f"File not found: {filename!r}")
    return {"deleted": filename}