"""Mission endpoints (auth-protected).

POST /api/mission/load    — load path by name or file
POST /api/mission/start   — arm → OFFBOARD → publish path
POST /api/mission/stop    — publish stop-path (stay armed)
POST /api/mission/abort   — hard abort (stop-path + MANUAL + disarm)
POST /api/mission/clear   — clear the resident mission (in-memory only)
GET  /api/mission/status  — current state + RPP snapshot
"""
from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException

from auth import require_operator_or_machine, require_token
from config import RPP_STALE, RPP_STATE_NAMES
from mission_loading import (
    MissionLoadConflict,
    load_path_for_controller,
    must_hit_for_path,
    pose_origin_or_error,
    spray_flags_for_path,
)
from mission_placement import PlacementError
from models import (
    LoadedPathResponse,
    MissionClearResponse,
    MissionLoadRequest,
    MissionStartRequest,
    MissionStatus,
)

router = APIRouter(prefix="/mission", tags=["mission"])


@router.get(
    "/loaded-path",
    response_model=LoadedPathResponse,
    dependencies=[Depends(require_operator_or_machine("mission:loaded-path"))],
)
async def loaded_path():
    """Stage 10 — confirm the coordinates currently resident in the controller."""
    from main import offboard_ctrl
    if offboard_ctrl is None:
        return LoadedPathResponse(loaded=False, state="idle")
    return LoadedPathResponse(**offboard_ctrl.loaded_path_summary())


@router.post("/load", dependencies=[Depends(require_token)])
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
    spray_flags = spray_flags_for_path(path_mgr, name, len(pts))
    must_hit = must_hit_for_path(path_mgr, name, len(pts))
    offboard_ctrl.load_path(
        pts, name=name, spray_flags=spray_flags, must_hit=must_hit
    )
    return {"loaded": name, "num_points": len(pts)}


@router.post("/start", dependencies=[Depends(require_token)])
async def start_mission(req: MissionStartRequest | None = None):
    from main import offboard_ctrl, path_mgr, ros_node
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")

    auto_origin = req.auto_origin if req else False
    name = (req.path_name or req.mission_file) if req else None

    # A caller may name the staged mission it verified. Cross-check it against
    # what the controller actually holds rather than trusting either side: this
    # is the guard against starting a mission the client never inspected (e.g.
    # after a background re-stage, or a second device loading something else).
    expected_id = (req.mission_id or "").strip() if req else ""
    if expected_id:
        if name:
            raise HTTPException(
                422,
                "start: pass mission_id OR path_name, not both — mission_id starts "
                "the already-loaded staged mission, path_name re-loads from disk "
                "and would discard its surveyed placement",
            )
        loaded_id = (offboard_ctrl.loaded_path_summary(sample=0).get("mission_id") or "")
        if not loaded_id:
            raise HTTPException(
                409,
                f"start: mission {expected_id} was requested but no staged mission "
                f"is loaded — load it to the controller first",
            )
        if loaded_id != expected_id:
            raise HTTPException(
                409,
                f"start: requested mission {expected_id} but {loaded_id} is loaded "
                f"— re-load before starting",
            )
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
                auto_origin=auto_origin,
            )
        except FileNotFoundError as exc:
            raise HTTPException(404, str(exc))
        except MissionLoadConflict as exc:
            raise HTTPException(409, str(exc))
        except Exception as exc:
            raise HTTPException(400, f"Path load failed: {exc}")
        origin_pre_applied = auto_origin

    try:
        ok, msg = await offboard_ctrl.start_async(
            auto_origin=auto_origin and not origin_pre_applied
        )
    except PlacementError as exc:
        raise HTTPException(422, str(exc))
    if not ok:
        raise HTTPException(409, f"Mission start failed: {msg}")
    return {"state": offboard_ctrl.state.value, "message": msg}


@router.post("/stop", dependencies=[Depends(require_token)])
async def stop_mission():
    from main import offboard_ctrl
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    return await offboard_ctrl.stop_async()


@router.post("/abort", dependencies=[Depends(require_token)])
async def abort_mission():
    from main import offboard_ctrl
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    return await offboard_ctrl.abort_async()


@router.post("/clear", response_model=MissionClearResponse, dependencies=[Depends(require_token)])
async def clear_mission():
    """Clear an idle/completed resident mission without deleting artifacts.

    In-memory only. Reduced from fix/runtime-entry-stop: this baseline has no
    point_mission / hold_owner, so only the controller's resident mission is
    cleared. A live mission returns 409 — stop or abort it first.
    """
    from main import offboard_ctrl
    from offboard_controller import MissionClearConflict

    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    try:
        status = await offboard_ctrl.clear_mission_async()
    except MissionClearConflict as exc:
        raise HTTPException(409, str(exc))

    # B0 (plan §3): publish an EXPLICIT cleared spray config, don't just stop
    # publishing. /spray/session_config is TRANSIENT_LOCAL, so silence would
    # let a restarted spray node re-latch the last mission's mode (e.g. dash)
    # over a now-empty path. Best-effort — never fail the clear on this.
    try:
        from main import ros_node
        from spray_session_builder import cleared_config_json

        if ros_node is not None:
            ros_node.publish_spray_session_config(cleared_config_json())
    except Exception as exc:  # noqa: BLE001
        import logging
        logging.getLogger("server.mission").warning(
            "cleared spray session_config publish failed: %s", exc
        )
    return MissionClearResponse(cleared=True, status=LoadedPathResponse(**status))


@router.get(
    "/status",
    response_model=MissionStatus,
    dependencies=[Depends(require_operator_or_machine("mission:status"))],
)
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
