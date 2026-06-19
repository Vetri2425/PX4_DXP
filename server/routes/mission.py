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
    start_mission_for_controller,
)
from mission_placement import PlacementError
from models import (
    LoadedPathResponse,
    MissionLoadRequest,
    MissionStartRequest,
    MissionStatus,
)

router = APIRouter(prefix="/mission", tags=["mission"],
                   dependencies=[Depends(require_token)])


@router.get("/loaded-path", response_model=LoadedPathResponse)
async def loaded_path():
    """Stage 10 — confirm the coordinates currently resident in the controller."""
    from main import offboard_ctrl
    if offboard_ctrl is None:
        return LoadedPathResponse(loaded=False, state="idle")
    return LoadedPathResponse(**offboard_ctrl.loaded_path_summary())


@router.post("/load")
async def load_mission(req: MissionLoadRequest):
    from main import offboard_ctrl, path_mgr
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    name = req.path_name or req.mission_file
    if not name:
        raise HTTPException(400, "Provide path_name or mission_file")
    try:
        pts = await load_path_for_controller(offboard_ctrl, path_mgr, name)
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except MissionLoadConflict as exc:
        raise HTTPException(409, str(exc))
    except PlacementError as exc:
        raise HTTPException(422, str(exc))
    except Exception as exc:
        raise HTTPException(400, f"Load failed: {exc}")
    return {
        "loaded": name,
        "mission_id": offboard_ctrl.loaded_mission_id,
        "num_points": len(pts),
    }


@router.post("/start")
async def start_mission(req: MissionStartRequest | None = None):
    from main import mission_capture, offboard_ctrl, path_mgr, ros_node
    from mission_debug_capture import CaptureUnavailable
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")

    auto_origin = req.auto_origin if req else False
    name = (req.path_name or req.mission_file) if req else None
    mission_id = req.mission_id if req else None
    try:
        ok, msg = await start_mission_for_controller(
            offboard_ctrl,
            path_mgr,
            ros_node,
            name=name,
            mission_id=mission_id,
            auto_origin=auto_origin,
            capture_coordinator=mission_capture,
            transport="rest",
            start_request={
                "path_name": req.path_name if req else None,
                "mission_file": req.mission_file if req else None,
                "mission_id": mission_id,
                "auto_origin": auto_origin,
            },
        )
    except CaptureUnavailable as exc:
        raise HTTPException(503, f"Mission capture unavailable: {exc}")
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except MissionLoadConflict as exc:
        raise HTTPException(409, str(exc))
    except PlacementError as exc:
        raise HTTPException(422, str(exc))
    except Exception as exc:
        raise HTTPException(400, f"Path load failed: {exc}")
    if not ok:
        raise HTTPException(409, f"Mission start failed: {msg}")
    return {"state": offboard_ctrl.state.value, "message": msg}


@router.post("/stop")
async def stop_mission():
    from main import mission_capture, offboard_ctrl
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    result = await offboard_ctrl.stop_async()
    if result.get("success") and mission_capture is not None:
        mission_capture.record_terminal(
            None, "operator_stop", state=offboard_ctrl.state.value, details=result
        )
    return result


@router.post("/abort")
async def abort_mission():
    from main import mission_capture, offboard_ctrl
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    result = await offboard_ctrl.abort_async()
    if mission_capture is not None:
        mission_capture.record_terminal(
            None, "operator_abort", state=offboard_ctrl.state.value, details=result
        )
    return result


@router.get("/debug-capture/status")
async def debug_capture_status():
    from main import mission_capture
    if mission_capture is None:
        return {"state": "unavailable", "required": None}
    return mission_capture.get_status()


@router.get("/status", response_model=MissionStatus)
async def mission_status():
    from main import offboard_ctrl, ros_node
    state = offboard_ctrl.state if offboard_ctrl else "idle"
    last_path_loaded = offboard_ctrl.loaded_path_name if offboard_ctrl else None
    loaded_mission_id = offboard_ctrl.loaded_mission_id if offboard_ctrl else None
    running_mission_id = offboard_ctrl.running_mission_id if offboard_ctrl else None
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
        loaded_mission_id = loaded_mission_id,
        running_mission_id = running_mission_id,
    )
