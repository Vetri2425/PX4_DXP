"""Socket.IO event handlers — client → server commands.

Socket.IO authenticates once at connect with the operator session token. After
that, control events trust the authenticated SID state rather than passwords or
per-event secrets.
"""
from __future__ import annotations

import datetime

from auth import bind_socket_sid, socket_authenticated, unbind_socket_sid
from logging_setup import get_logger

log = get_logger("server.socket")


def _now() -> str:
    return datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"


def _auth_ok(sid: str) -> bool:
    return socket_authenticated(sid)


async def _emit_unauth(sio, sid):
    await sio.emit("socket_error", {"reason": "unauthorised"}, to=sid)


def register_handlers(sio) -> None:
    """Attach all client → server event handlers to the given AsyncServer."""

    @sio.event
    async def connect(sid, environ, auth=None):
        from main import activity_log
        token = None
        if isinstance(auth, dict):
            token = auth.get("token") or auth.get("auth")
        elif isinstance(auth, str):
            token = auth
        if bind_socket_sid(sid, token) is None:
            raise ConnectionRefusedError("unauthorised")
        activity_log.append({"timestamp": _now(), "level": "info",
                              "message": f"Socket connected: {sid}"})

    @sio.event
    async def disconnect(sid):
        from main import activity_log, joystick_ctrl
        if joystick_ctrl is not None and joystick_ctrl.owner_sid == sid:
            try:
                await joystick_ctrl.release(sid, reason="disconnect", force=True)
            except Exception:
                log.exception("joystick release-on-disconnect failed for sid=%s", sid)
        unbind_socket_sid(sid)
        activity_log.append({"timestamp": _now(), "level": "info",
                              "message": f"Socket disconnected: {sid}"})

    # ── Vehicle control ───────────────────────────────────────────────────────

    @sio.on("arm")
    async def on_arm(sid, data):
        from main import ros_node, activity_log
        if not _auth_ok(sid):
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
        if not _auth_ok(sid):
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
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        if emergency_handler is None:
            return
        result = await emergency_handler.estop_async()
        await sio.emit("estop_result", result, to=sid)

    # ── Mission control ───────────────────────────────────────────────────────

    @sio.on("mission_load")
    async def on_mission_load(sid, data):
        from main import offboard_ctrl, path_mgr
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        name = (data.get("path_name") or data.get("mission_file")
                if isinstance(data, dict) else None)
        if not name:
            await sio.emit("mission_error",
                           {"message": "No path name provided"}, to=sid)
            return
        try:
            pts = path_mgr.load_path(name)
            from mission_loading import spray_flags_for_path
            spray_flags = spray_flags_for_path(path_mgr, name, len(pts))
            offboard_ctrl.load_path(pts, name=name, spray_flags=spray_flags)
            await sio.emit("mission_loaded",
                           {"name": name, "num_points": len(pts)}, to=sid)
        except Exception as exc:
            await sio.emit("mission_error", {"message": str(exc)}, to=sid)

    @sio.on("mission_start")
    async def on_mission_start(sid, data=None):
        from main import offboard_ctrl
        from mission_placement import PlacementError
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        try:
            ok, msg = await offboard_ctrl.start_async()
        except PlacementError as exc:
            ok, msg = False, str(exc)
        await sio.emit("mission_status_update",
                       {"state":   offboard_ctrl.state.value,
                        "success": ok,
                        "message": msg}, to=sid)

    @sio.on("mission_stop")
    async def on_mission_stop(sid, data=None):
        from main import offboard_ctrl
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        result = await offboard_ctrl.stop_async()
        await sio.emit("mission_status_update", result, to=sid)

    @sio.on("mission_abort")
    async def on_mission_abort(sid, data=None):
        from main import offboard_ctrl
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        result = await offboard_ctrl.abort_async()
        await sio.emit("mission_status_update", result, to=sid)

    @sio.on("request_params")
    async def on_request_params(sid, data):
        from main import ros_node
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        if ros_node is None:
            return
        names = data.get("names", []) if isinstance(data, dict) else []
        out = {}
        for name in names:
            ok, value, _ = await ros_node.get_param_async(name)
            out[name] = value if ok else None
        await sio.emit("params_result", out, to=sid)

    # ── Joystick / manual control ──────────────────────────────────────────────
    # docs/Architecture/JOYSTICK_CONTROLLER_PLAN.md §4.4. Auth is enforced the
    # same way as every other handler in this file: `_auth_ok(sid)` against the
    # session bound at socket connect (auth.bind_socket_sid). The client
    # contract still sends `auth` in every payload for schema parity with the
    # other client (DYX_GCS_V) and forward-compat with per-event re-validation;
    # the server does not currently re-check it per event.

    @sio.on("joystick_acquire")
    async def on_joystick_acquire(sid, data):
        from main import joystick_ctrl
        data = data if isinstance(data, dict) else {}
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        if joystick_ctrl is None:
            await sio.emit(
                "joystick_error",
                {"type": "joystick_error", "code": "unavailable", "message": "joystick subsystem not initialised"},
                to=sid,
            )
            return
        try:
            result = await joystick_ctrl.acquire(sid, data)
        except Exception as exc:
            code = getattr(exc, "code", "acquire_failed")
            await sio.emit(
                "joystick_error",
                {"type": "joystick_error", "code": code, "message": str(exc)},
                to=sid,
            )
            return
        await sio.emit("joystick_acquired", result, to=sid)

    @sio.on("joystick_command")
    async def on_joystick_command(sid, data):
        from main import joystick_ctrl
        data = data if isinstance(data, dict) else {}
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        if joystick_ctrl is None:
            return
        try:
            joystick_ctrl.handle_command(sid, data)
        except Exception as exc:
            code = getattr(exc, "code", "command_rejected")
            await sio.emit(
                "joystick_error",
                {"type": "joystick_error", "code": code, "message": str(exc)},
                to=sid,
            )
        # Successful commands are not individually acked (plan §4.2) — the
        # client reconciles via the periodic telemetry snapshot instead.

    @sio.on("joystick_release")
    async def on_joystick_release(sid, data):
        from main import joystick_ctrl
        data = data if isinstance(data, dict) else {}
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        if joystick_ctrl is None:
            return
        try:
            result = await joystick_ctrl.release(
                sid,
                session_id=data.get("session_id"),
                lease_id=data.get("lease_id"),
                reason="explicit",
            )
        except Exception as exc:
            code = getattr(exc, "code", "release_failed")
            await sio.emit(
                "joystick_error",
                {"type": "joystick_error", "code": code, "message": str(exc)},
                to=sid,
            )
            return
        await sio.emit("joystick_released", result)
