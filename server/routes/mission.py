"""Mission endpoints (auth-protected).

POST /api/mission/load    — load path by name or file
POST /api/mission/start   — arm → OFFBOARD → publish path
POST /api/mission/stop    — publish stop-path (stay armed)
POST /api/mission/abort   — hard abort (stop-path + MANUAL + disarm)
POST /api/mission/clear   — clear the resident mission (in-memory only)
POST /api/mission/pause   — resumable OFFBOARD hold (point missions)
POST /api/mission/resume  — resume paused point mission from live pose
POST /api/mission/obstacle — set obstacle clear/blocked hook state
POST /api/mission/point/continue — advance manual point mission after operator approval
GET  /api/mission/point/status   — Point Mode runtime diagnostics
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
    MissionClearResponse,
    MissionLoadRequest,
    MissionResumeRequest,
    MissionStartRequest,
    MissionStatus,
    ObstacleStatusRequest,
    ObstacleStatusResponse,
    PointContinueResponse,
    PointMissionStatusResponse,
    PointPauseResponse,
    PointResumeResponse,
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


def _merge_point_status() -> dict:
    from main import hold_owner, point_mission, ros_node

    if point_mission is None:
        return {}
    payload = point_mission.status.as_dict()
    if hold_owner is not None:
        hold = hold_owner.as_dict(ros_node)
        payload.update(
            {
                "setpoint_source": hold["setpoint_source"],
                "hold_active": hold["hold_active"],
                "hold_north_m": hold["hold_north_m"],
                "hold_east_m": hold["hold_east_m"],
                "hold_heading_ned_rad": hold["hold_heading_ned_rad"],
                "hold_error_m": hold["hold_error_m"],
            }
        )
    return payload


def _require_point_mode(offboard_ctrl) -> None:
    if offboard_ctrl.spray_mode != "point":
        raise HTTPException(409, "loaded mission is not in point spray mode")


@router.post("/clear", response_model=MissionClearResponse)
async def clear_mission():
    """Clear an idle/completed resident mission without deleting artifacts."""
    from main import hold_owner, offboard_ctrl, point_mission, ros_node
    from offboard_controller import MissionClearConflict

    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    if hold_owner is not None:
        hold_owner.deactivate(ros_node)
    if point_mission is not None:
        await point_mission.clear_mission(
            ros_node, reason="cleared", offboard_ctrl=offboard_ctrl
        )
    try:
        status = await offboard_ctrl.clear_mission_async()
    except MissionClearConflict as exc:
        raise HTTPException(409, str(exc))
    return MissionClearResponse(
        cleared=True,
        status=LoadedPathResponse(**status),
    )


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
    from main import hold_owner, mission_capture, offboard_ctrl, point_mission, ros_node
    from mission_stop import stop_active_mission

    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    return await stop_active_mission(
        offboard_ctrl,
        point_mission,
        ros_node,
        hold_owner,
        mission_capture=mission_capture,
        transport="rest",
    )


@router.post("/abort")
async def abort_mission():
    from main import hold_owner, mission_capture, offboard_ctrl, point_mission, ros_node
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    if hold_owner is not None:
        hold_owner.deactivate(ros_node)
    if point_mission is not None:
        await point_mission.abort(ros_node, offboard_ctrl=offboard_ctrl)
    result = await offboard_ctrl.abort_async()
    if mission_capture is not None:
        mission_capture.record_terminal(
            None, "operator_abort", state=offboard_ctrl.state.value, details=result
        )
    return result


def _point_status_payload() -> PointMissionStatusResponse | None:
    from main import point_mission

    if point_mission is None:
        return None
    return PointMissionStatusResponse(**_merge_point_status())


@router.get("/point/status", response_model=PointMissionStatusResponse)
async def point_mission_status():
    """Point Mode runtime diagnostics for the loaded/active point mission."""
    from main import point_mission

    if point_mission is None:
        raise HTTPException(503, "Point mission orchestrator unavailable")
    return PointMissionStatusResponse(**_merge_point_status())


@router.post("/pause", response_model=PointPauseResponse)
async def pause_mission():
    """Request a resumable OFFBOARD hold for the active point mission."""
    from main import hold_owner, offboard_ctrl, point_mission, ros_node

    if point_mission is None:
        raise HTTPException(503, "Point mission orchestrator unavailable")
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    _require_point_mode(offboard_ctrl)
    ok, message, status_code = await point_mission.pause_mission(ros_node, hold_owner)
    status = PointMissionStatusResponse(**_merge_point_status())
    if not ok:
        raise HTTPException(status_code, message)
    return PointPauseResponse(paused=True, message=message, status=status)


@router.post("/resume", response_model=PointResumeResponse)
async def resume_mission(req: MissionResumeRequest | None = None):
    """Resume a paused point mission from the current live pose."""
    from main import hold_owner, offboard_ctrl, point_mission, ros_node
    from control_arbiter import ControlArbiterError, get_control_arbiter

    if point_mission is None:
        raise HTTPException(503, "Point mission orchestrator unavailable")
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    _require_point_mode(offboard_ctrl)
    try:
        await get_control_arbiter().ensure_mission_motion_allowed(offboard_ctrl)
    except ControlArbiterError as exc:
        raise HTTPException(409, exc.message) from exc
    expected_generation = req.expected_generation if req else None
    ok, message, status_code = await point_mission.resume_mission(
        ros_node,
        hold_owner,
        expected_generation=expected_generation,
    )
    status = PointMissionStatusResponse(**_merge_point_status())
    if not ok:
        raise HTTPException(status_code, message)
    return PointResumeResponse(resumed=True, message=message, status=status)


@router.post("/obstacle", response_model=ObstacleStatusResponse)
async def set_obstacle_status(req: ObstacleStatusRequest):
    """Set obstacle clear/blocked hook state for point mission pause/resume."""
    from main import point_mission

    if point_mission is None:
        raise HTTPException(503, "Point mission orchestrator unavailable")
    point_mission.set_obstacle_clear(req.clear)
    status = PointMissionStatusResponse(**_merge_point_status())
    return ObstacleStatusResponse(obstacle_clear=req.clear, status=status)


@router.post("/point/continue", response_model=PointContinueResponse)
async def point_mission_continue():
    """Advance a manual point mission after operator approval."""
    from main import offboard_ctrl, point_mission, ros_node
    from control_arbiter import ControlArbiterError, get_control_arbiter

    if point_mission is None:
        raise HTTPException(503, "Point mission orchestrator unavailable")
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    if offboard_ctrl.spray_mode != "point":
        raise HTTPException(409, "loaded mission is not in point spray mode")
    try:
        await get_control_arbiter().ensure_mission_motion_allowed(offboard_ctrl)
    except ControlArbiterError as exc:
        raise HTTPException(409, exc.message) from exc
    ok, message, status_code = await point_mission.continue_point(ros_node)
    status = PointMissionStatusResponse(**_merge_point_status())
    if not ok:
        raise HTTPException(status_code, message)
    return PointContinueResponse(continued=True, message=message, status=status)


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
    rpp_debug_age_ms = s.get("rpp_debug_age_ms")
    rpp_debug_fresh = s.get("rpp_debug_fresh")
    measured_speed_m_s = s.get("measured_speed_m_s")
    if ros_node is not None:
        try:
            monitor = ros_node.get_rpp_monitor()
            if monitor.has_snapshot(fresh=True):
                rpp = monitor.get_snapshot()
                code = rpp.state_code
                dist_to_goal = rpp.dist_to_goal_m
                speed = rpp.speed_m_s
                xtrack = rpp.xtrack_m
                pose_age_ms = rpp.pose_age_ms
                age_s = monitor.snapshot_age_s()
                rpp_debug_age_ms = age_s * 1000.0 if age_s is not None else None
                rpp_debug_fresh = True
            elif monitor.has_snapshot():
                age_s = monitor.snapshot_age_s()
                rpp_debug_age_ms = age_s * 1000.0 if age_s is not None else None
                rpp_debug_fresh = False
        except Exception:
            code = s.get("rpp_state", RPP_STALE)
            dist_to_goal = s.get("dist_to_goal_m")
            speed = s.get("speed_m_s")
            xtrack = s.get("xtrack_m")

    point_payload = _point_status_payload()
    return MissionStatus(
        state          = state,
        rpp_state      = code,
        rpp_state_name = RPP_STATE_NAMES.get(code, "UNKNOWN"),
        dist_to_goal   = dist_to_goal,
        speed          = speed,
        xtrack         = xtrack,
        pose_age_ms    = pose_age_ms,
        rpp_debug_age_ms = rpp_debug_age_ms,
        rpp_debug_fresh = rpp_debug_fresh,
        measured_speed_m_s = measured_speed_m_s,
        fcu_connected  = s.get("connected"),
        last_path_loaded = last_path_loaded,
        loaded_mission_id = loaded_mission_id,
        running_mission_id = running_mission_id,
        point = point_payload,
    )
