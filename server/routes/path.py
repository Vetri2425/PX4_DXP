"""Path management endpoints (auth-protected).

GET    /api/paths              — list built-in + uploaded paths
GET    /api/path/{name}/preview — return local-NED points for display
POST   /api/path/upload        — upload .waypoints, .csv, or .dxf
POST   /api/path/publish       — publish named path to /path topic
POST   /api/path/parse-dxf     — parse DXF file, return entity list
POST   /api/path/plan          — run full planning pipeline, return PlannedPath
DELETE /api/path/{filename}    — delete uploaded file
"""
from __future__ import annotations

import json
import math
import os
import tempfile
import time
import uuid

from fastapi import APIRouter, Depends, File, HTTPException, UploadFile

from auth import require_token
from config import MAX_UPLOAD_BYTES, MISSION_DIR, RMSE_MAX, SPRAY_LITERS_PER_METER, STAGING_DIR, STAGING_TTL_S
from models import (
    DXFEntityInfo,
    DXFParseResponse,
    LoadMissionRequest,
    MissionSummary,
    PathPlanRequest,
    PathPlanResponse,
    PathPreviewResponse,
    PathPublishRequest,
)
from path_manager import UploadValidationError

# Two distinct routers so the URL structure is explicit and stable.
paths_router = APIRouter(prefix="/paths", tags=["path"],
                         dependencies=[Depends(require_token)])
path_router  = APIRouter(prefix="/path",  tags=["path"],
                         dependencies=[Depends(require_token)])


# ── Listing ───────────────────────────────────────────────────────────────────

@paths_router.get("")
async def list_paths():
    import asyncio
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
    import asyncio
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
    import asyncio
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
                enable_path_extensions=req.enable_path_extensions,
                pre_extension_m=req.pre_extension_m,
                aft_extension_m=req.aft_extension_m,
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
        offboard_ctrl.load_path(waypoints, name=safe_id)
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
