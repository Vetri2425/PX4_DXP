"""Mission endpoints (auth-protected).

POST /api/mission/load    — load path by name or file
POST /api/mission/start   — arm → OFFBOARD → publish path
POST /api/mission/stop    — publish stop-path (stay armed)
POST /api/mission/abort   — hard abort (stop-path + MANUAL + disarm)
GET  /api/mission/status  — current state + RPP snapshot
"""
from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException

from auth import require_token
from config import RPP_STALE, RPP_STATE_NAMES
from mission_loading import (
    MissionLoadConflict,
    load_path_for_controller,
    pose_origin_or_error,
)
from models import MissionLoadRequest, MissionStartRequest, MissionStatus

router = APIRouter(prefix="/mission", tags=["mission"],
                   dependencies=[Depends(require_token)])


@router.post("/load")
async def load_mission(req: MissionLoadRequest):
    from main import offboard_ctrl, path_mgr
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    name = req.path_name or req.mission_file
    if not name:
        raise HTTPException(400, "Provide path_name or mission_file")
    try:
        pts = path_mgr.load_path(name)
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except Exception as exc:
        raise HTTPException(400, f"Load failed: {exc}")
    offboard_ctrl.load_path(pts, name=name)
    return {"loaded": name, "num_points": len(pts)}


@router.post("/start")
async def start_mission(req: MissionStartRequest | None = None):
    from main import offboard_ctrl, path_mgr, ros_node
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")

    auto_origin = req.auto_origin if req else False
    name = (req.path_name or req.mission_file) if req else None
    origin = (0.0, 0.0)
    start_position = None
    origin_pre_applied = False

    if auto_origin:
        if ros_node is None:
            raise HTTPException(503, "ROS node not ready")
        s = ros_node.get_state()
        pose_origin = pose_origin_or_error(s)
        if isinstance(pose_origin, str):
            raise HTTPException(409, pose_origin)
        origin = pose_origin
        start_position = origin
        if not name:
            loaded = offboard_ctrl.loaded_path_name
            if loaded and loaded != "unknown":
                name = loaded

    if name:
        try:
            pts = await load_path_for_controller(
                offboard_ctrl,
                path_mgr,
                name,
                origin=origin,
                start_position=start_position,
            )
        except FileNotFoundError as exc:
            raise HTTPException(404, str(exc))
        except MissionLoadConflict as exc:
            raise HTTPException(409, str(exc))
        except Exception as exc:
            raise HTTPException(400, f"Path load failed: {exc}")
        origin_pre_applied = auto_origin

    ok, msg = await offboard_ctrl.start_async(
        auto_origin=auto_origin and not origin_pre_applied
    )
    if not ok:
        raise HTTPException(409, f"Mission start failed: {msg}")
    return {"state": offboard_ctrl.state.value, "message": msg}


@router.post("/stop")
async def stop_mission():
    from main import offboard_ctrl
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    return await offboard_ctrl.stop_async()


@router.post("/abort")
async def abort_mission():
    from main import offboard_ctrl
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    return await offboard_ctrl.abort_async()


@router.get("/status", response_model=MissionStatus)
async def mission_status():
    from main import offboard_ctrl, ros_node
    state = offboard_ctrl.state if offboard_ctrl else "idle"
    last_path_loaded = offboard_ctrl.loaded_path_name if offboard_ctrl else None
    s = {}
    if ros_node is not None:
        try:
            s = ros_node.get_state()
        except Exception:
            s = {}

    code = RPP_STALE
    dist_to_goal = None
    speed = None
    xtrack = None
    pose_age_ms = s.get("pose_age_ms")
    if ros_node is not None:
        try:
            monitor = ros_node.get_rpp_monitor()
            if monitor.has_snapshot():
                rpp = monitor.get_snapshot()
                code = rpp.state_code
                dist_to_goal = rpp.dist_to_goal_m
                speed = rpp.speed_m_s
                xtrack = rpp.xtrack_m
                pose_age_ms = rpp.pose_age_ms
        except Exception:
            code = s.get("rpp_state", RPP_STALE)
            dist_to_goal = s.get("dist_to_goal_m")
            speed = s.get("speed_m_s")
            xtrack = s.get("xtrack_m")

    return MissionStatus(
        state          = state,
        rpp_state      = code,
        rpp_state_name = RPP_STATE_NAMES.get(code, "UNKNOWN"),
        dist_to_goal   = dist_to_goal,
        speed          = speed,
        xtrack         = xtrack,
        pose_age_ms    = pose_age_ms,
        fcu_connected  = s.get("connected"),
        last_path_loaded = last_path_loaded,
    )
