"""Socket.IO event handlers — client → server commands.

All control events require an `auth` field with a valid token. Telemetry is
broadcast to all connected sids unconditionally; control commands are
rejected with a `socket_error` event when auth fails.
"""
from __future__ import annotations

import datetime

from auth import check_socket_token
from logging_setup import get_logger

log = get_logger("server.socket")


def _now() -> str:
    return datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"


def _auth_ok(data) -> bool:
    if not isinstance(data, dict):
        return check_socket_token(None)
    return check_socket_token(data.get("auth"))


async def _emit_unauth(sio, sid):
    await sio.emit("socket_error", {"reason": "unauthorised"}, to=sid)


def register_handlers(sio) -> None:
    """Attach all client → server event handlers to the given AsyncServer."""

    @sio.event
    async def connect(sid, environ, auth=None):
        from main import activity_log
        activity_log.append({"timestamp": _now(), "level": "info",
                              "message": f"Socket connected: {sid}"})

    @sio.event
    async def disconnect(sid):
        from main import activity_log
        activity_log.append({"timestamp": _now(), "level": "info",
                              "message": f"Socket disconnected: {sid}"})

    # ── Vehicle control ───────────────────────────────────────────────────────

    @sio.on("arm")
    async def on_arm(sid, data):
        from main import ros_node, activity_log
        if not _auth_ok(data):
            return await _emit_unauth(sio, sid)
        if ros_node is None:
            return
        arm_val = data.get("arm", True) if isinstance(data, dict) else bool(data)
        ok, why = await ros_node.arm_async(arm_val)
        verb = "Armed" if arm_val else "Disarmed"
        activity_log.append({"timestamp": _now(),
                              "level": "info" if ok else "error",
                              "message": f"{verb} via socket: "
                                         f"{'OK' if ok else f'FAILED ({why})'}"})
        await sio.emit("arm_result",
                       {"success": ok, "arm": arm_val, "message": why}, to=sid)

    @sio.on("set_mode")
    async def on_set_mode(sid, data):
        from main import ros_node, activity_log
        if not _auth_ok(data):
            return await _emit_unauth(sio, sid)
        if ros_node is None:
            return
        mode = data.get("mode", "MANUAL") if isinstance(data, dict) else str(data)
        ok, why = await ros_node.set_mode_async(mode)
        activity_log.append({"timestamp": _now(),
                              "level": "info" if ok else "error",
                              "message": f"set_mode {mode}: "
                                         f"{'OK' if ok else f'FAILED ({why})'}"})
        await sio.emit("mode_result",
                       {"success": ok, "mode": mode, "message": why}, to=sid)

    @sio.on("emergency_stop")
    async def on_estop(sid, data=None):
        from main import emergency_handler
        if not _auth_ok(data):
            return await _emit_unauth(sio, sid)
        if emergency_handler is None:
            return
        result = await emergency_handler.estop_async()
        await sio.emit("estop_result", result, to=sid)

    # ── Mission control ───────────────────────────────────────────────────────

    @sio.on("mission_load")
    async def on_mission_load(sid, data):
        from main import offboard_ctrl, path_mgr
        from mission_loading import MissionLoadConflict, load_path_for_controller
        if not _auth_ok(data):
            return await _emit_unauth(sio, sid)
        name = (data.get("path_name") or data.get("mission_file")
                if isinstance(data, dict) else None)
        if not name:
            await sio.emit("mission_error",
                           {"message": "No path name provided"}, to=sid)
            return
        try:
            pts = await load_path_for_controller(offboard_ctrl, path_mgr, name)
            await sio.emit("mission_loaded",
                           {"name": name,
                            "mission_id": offboard_ctrl.loaded_mission_id,
                            "num_points": len(pts)}, to=sid)
        except MissionLoadConflict as exc:
            await sio.emit("mission_error",
                           {"message": str(exc), "status": 409}, to=sid)
        except Exception as exc:
            await sio.emit("mission_error", {"message": str(exc)}, to=sid)

    @sio.on("mission_start")
    async def on_mission_start(sid, data=None):
        from main import mission_capture, offboard_ctrl, path_mgr, ros_node
        from mission_debug_capture import CaptureUnavailable
        from mission_loading import MissionLoadConflict, start_mission_for_controller
        from mission_placement import PlacementError
        if not _auth_ok(data):
            return await _emit_unauth(sio, sid)
        payload = data if isinstance(data, dict) else {}
        name = payload.get("path_name") or payload.get("mission_file")
        try:
            ok, msg = await start_mission_for_controller(
                offboard_ctrl,
                path_mgr,
                ros_node,
                name=name,
                mission_id=payload.get("mission_id"),
                auto_origin=bool(payload.get("auto_origin", False)),
                capture_coordinator=mission_capture,
                transport="socketio",
                start_request={
                    "path_name": payload.get("path_name"),
                    "mission_file": payload.get("mission_file"),
                    "mission_id": payload.get("mission_id"),
                    "auto_origin": bool(payload.get("auto_origin", False)),
                },
            )
        except CaptureUnavailable as exc:
            ok, msg, status = False, f"Mission capture unavailable: {exc}", 503
        except MissionLoadConflict as exc:
            ok, msg, status = False, str(exc), 409
        except PlacementError as exc:
            ok, msg, status = False, str(exc), 422
        except Exception as exc:
            ok, msg, status = False, str(exc), 409
        else:
            status = 200 if ok else 409
        await sio.emit("mission_status_update",
                       {"state":   offboard_ctrl.state.value,
                        "success": ok,
                        "message": msg,
                        "status": status}, to=sid)

    @sio.on("mission_stop")
    async def on_mission_stop(sid, data=None):
        from main import hold_owner, mission_capture, offboard_ctrl, point_mission, ros_node
        from mission_stop import stop_active_mission
        if not _auth_ok(data):
            return await _emit_unauth(sio, sid)
        result = await stop_active_mission(
            offboard_ctrl,
            point_mission,
            ros_node,
            hold_owner,
            mission_capture=mission_capture,
            transport="socket",
        )
        await sio.emit("mission_status_update", result, to=sid)

    @sio.on("mission_abort")
    async def on_mission_abort(sid, data=None):
        from main import hold_owner, mission_capture, offboard_ctrl, point_mission, ros_node
        if not _auth_ok(data):
            return await _emit_unauth(sio, sid)
        if hold_owner is not None:
            hold_owner.deactivate(ros_node)
        if point_mission is not None:
            await point_mission.abort(ros_node)
        result = await offboard_ctrl.abort_async()
        if mission_capture is not None:
            mission_capture.record_terminal(
                None, "operator_abort", state=offboard_ctrl.state.value, details=result
            )
        await sio.emit("mission_status_update", result, to=sid)

    @sio.on("request_params")
    async def on_request_params(sid, data):
        from main import ros_node
        if not _auth_ok(data):
            return await _emit_unauth(sio, sid)
        if ros_node is None:
            return
        names = data.get("names", []) if isinstance(data, dict) else []
        out = {}
        for name in names:
            ok, value, _ = await ros_node.get_param_async(name)
            out[name] = value if ok else None
        await sio.emit("params_result", out, to=sid)
