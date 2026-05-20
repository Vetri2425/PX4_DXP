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
from config import RPP_STATE_NAMES
from models import MissionLoadRequest, MissionStartRequest, MissionStatus

router = APIRouter(prefix="/mission", tags=["mission"],
                   dependencies=[Depends(require_token)])


@router.post("/load")
async def load_mission(req: MissionLoadRequest):
    from main import offboard_ctrl, path_mgr
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
    from main import offboard_ctrl, path_mgr
    if req and (req.path_name or req.mission_file):
        name = req.path_name or req.mission_file
        try:
            pts = path_mgr.load_path(name)
        except FileNotFoundError as exc:
            raise HTTPException(404, str(exc))
        except Exception as exc:
            raise HTTPException(400, f"Path load failed: {exc}")
        offboard_ctrl.load_path(pts, name=name)
    ok, msg = await offboard_ctrl.start_async()
    if not ok:
        raise HTTPException(409, f"Mission start failed: {msg}")
    return {"state": offboard_ctrl.state.value, "message": msg}


@router.post("/stop")
async def stop_mission():
    from main import offboard_ctrl
    await offboard_ctrl.stop_async()
    return {"state": offboard_ctrl.state.value}


@router.post("/abort")
async def abort_mission():
    from main import offboard_ctrl
    await offboard_ctrl.abort_async()
    return {"state": offboard_ctrl.state.value}


@router.get("/status", response_model=MissionStatus)
async def mission_status():
    from main import offboard_ctrl, ros_node
    s = ros_node.get_state() if ros_node else {}
    code = s.get("rpp_state", 0)
    return MissionStatus(
        state          = offboard_ctrl.state,
        rpp_state      = code,
        rpp_state_name = RPP_STATE_NAMES.get(code, "UNKNOWN"),
        dist_to_goal   = s.get("dist_to_goal_m"),
        speed          = s.get("speed_m_s"),
        xtrack         = s.get("xtrack_m"),
    )
