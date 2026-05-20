"""Drawing Rover FastAPI backend.

Lifespan order (startup → ready → shutdown):
  1. Configure logging
  2. Initialise auth (load or create rover token)
  3. rclpy.init() + RosBridgeNode + MultiThreadedExecutor in daemon thread
  4. Build shared singletons (PathManager, OffboardController, EmergencyHandler)
  5. Register Socket.IO handlers
  6. Start telemetry push loop (10 Hz) — also runs:
       · auto-completion (RUNNING → COMPLETED on RPP DONE settle)
       · pose-stale watchdog (RUNNING + STALE > grace → estop)
       · disconnect notification
  7. Start UDP discovery beacon

Shutdown reverses the order. Telemetry loop catches and logs every exception
without dying. Beacon and rclpy threads use Event-based stop signals so
shutdown completes within ~1 s.
"""
from __future__ import annotations

import asyncio
import datetime
import time
from collections import deque
from contextlib import asynccontextmanager
from typing import Optional

import socketio
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from auth import init_auth
from config import (
    BEACON_INTERVAL, BEACON_PORT, CORS_ALLOW_CREDENTIALS, CORS_ALLOW_ORIGINS,
    DEFAULT_PORT, MAX_ACTIVITY_LOG, MISSION_DIR, POSE_STALE_MS, ROVER_ID,
    RPP_STALE, RPP_STATE_NAMES, SAFETY_STALE_GRACE_S, TELEMETRY_HZ,
)
from logging_setup import configure_logging, get_logger
from models import MissionState

# ── Module-level singletons (populated in lifespan) ───────────────────────────
ros_node:          Optional["object"] = None
offboard_ctrl:     Optional["object"] = None
path_mgr:          Optional["object"] = None
emergency_handler: Optional["object"] = None
_executor:         Optional["object"] = None
_beacon:           Optional["object"] = None
_telemetry_task:   Optional[asyncio.Task] = None

# Bounded, thread-safe ring buffer (deque maxlen). All log appends are atomic
# under the GIL; bounded eviction is built in. Replaces the racy list+trim.
activity_log: deque = deque(maxlen=MAX_ACTIVITY_LOG)

log = get_logger("server.main")


# ── Socket.IO ASGI app ────────────────────────────────────────────────────────
# cors_allowed_origins must match the REST CORS policy — they are independent
# implementations and both must agree.
sio = socketio.AsyncServer(
    async_mode="asgi",
    cors_allowed_origins=CORS_ALLOW_ORIGINS,
)
socket_app = socketio.ASGIApp(sio)


# ── Lifespan ──────────────────────────────────────────────────────────────────

@asynccontextmanager
async def lifespan(app: FastAPI):
    global ros_node, offboard_ctrl, path_mgr, emergency_handler
    global _executor, _beacon, _telemetry_task

    configure_logging()
    init_auth()

    # ── Start ROS2 ────────────────────────────────────────────────────────────
    try:
        import rclpy
        from ros_node import RosBridgeNode, RosExecutorThread
        if not rclpy.ok():
            rclpy.init()
        ros_node = RosBridgeNode()
        _executor = RosExecutorThread(num_threads=4)
        _executor.add_node(ros_node)
        _executor.start()
        _record("info", "ROS2 bridge started")
    except Exception as exc:
        log.exception("ROS2 startup failed — continuing without MAVROS")
        _record("warning",
                f"ROS2 unavailable — server running without MAVROS: {exc}")

    # ── Build shared objects ──────────────────────────────────────────────────
    from beacon import RoverBeacon
    from emergency import EmergencyHandler
    from offboard_controller import OffboardController
    from path_manager import PathManager

    path_mgr          = PathManager(MISSION_DIR)
    offboard_ctrl     = OffboardController(ros_node, activity_log)
    emergency_handler = EmergencyHandler(ros_node, offboard_ctrl, activity_log)

    # ── Register Socket.IO handlers ───────────────────────────────────────────
    from sockets.events import register_handlers
    register_handlers(sio)

    # ── Start telemetry + watchdog loop ───────────────────────────────────────
    _telemetry_task = asyncio.create_task(_telemetry_loop(),
                                          name="telemetry-loop")

    # ── Start UDP discovery beacon ────────────────────────────────────────────
    _beacon = RoverBeacon(
        port=BEACON_PORT, interval=BEACON_INTERVAL,
        rover_id=ROVER_ID, server_port=DEFAULT_PORT,
    )
    _beacon.start()

    _record("info", f"Server ready on port {DEFAULT_PORT}")
    log.info("server ready: port=%d telemetry=%dHz", DEFAULT_PORT, TELEMETRY_HZ)

    yield  # ─── Running ───────────────────────────────────────────────────────

    # ── Shutdown ──────────────────────────────────────────────────────────────
    log.info("shutting down…")

    if _telemetry_task:
        _telemetry_task.cancel()
        try:
            await _telemetry_task
        except (asyncio.CancelledError, Exception):
            pass

    if _beacon:
        _beacon.stop()

    if _executor:
        _executor.stop()

    if ros_node:
        try:
            ros_node.destroy_node()
        except Exception:
            log.exception("destroy_node raised")
    try:
        import rclpy
        rclpy.try_shutdown()
    except Exception:
        pass

    _record("info", "Server stopped")


# ── FastAPI app factory ───────────────────────────────────────────────────────

def create_app() -> FastAPI:
    app = FastAPI(
        title="Drawing Rover API",
        version="1.0.0",
        lifespan=lifespan,
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=CORS_ALLOW_ORIGINS,
        allow_credentials=CORS_ALLOW_CREDENTIALS,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # REST routers
    from routes.system    import router as sys_router
    from routes.vehicle   import router as veh_router
    from routes.mission   import router as mis_router
    from routes.path      import paths_router, path_router
    from routes.params    import router as par_router
    from routes.telemetry import router as tel_router

    app.include_router(sys_router,  prefix="/api")
    app.include_router(veh_router,  prefix="/api")
    app.include_router(mis_router,  prefix="/api")
    app.include_router(paths_router, prefix="/api")   # → /api/paths
    app.include_router(path_router,  prefix="/api")   # → /api/path/*
    app.include_router(par_router,  prefix="/api")
    app.include_router(tel_router,  prefix="/api")

    # Socket.IO
    app.mount("/socket.io", socket_app)
    return app


app = create_app()


# ── Telemetry loop with watchdog and auto-completion ──────────────────────────

async def _telemetry_loop() -> None:
    interval = 1.0 / TELEMETRY_HZ
    prev_connected: Optional[bool] = None
    stale_since: Optional[float] = None
    consecutive_errors = 0

    log.info("telemetry loop started @ %d Hz", TELEMETRY_HZ)
    try:
        while True:
            try:
                await asyncio.sleep(interval)
                if ros_node is None:
                    continue

                s    = ros_node.get_state()
                code = s.get("rpp_state", 0)
                now  = time.time()

                # ── 1. Push telemetry ──────────────────────────────────────────
                telem = {
                    "pos_n":           s.get("pos_n"),
                    "pos_e":           s.get("pos_e"),
                    "heading_ned_deg": s.get("heading_ned_deg"),
                    "xtrack_m":        s.get("xtrack_m"),
                    "heading_err_deg": s.get("heading_err_deg"),
                    "lookahead_m":     s.get("lookahead_m"),
                    "speed_m_s":       s.get("speed_m_s"),
                    "kappa":           s.get("kappa"),
                    "dist_to_goal_m":  s.get("dist_to_goal_m"),
                    "pose_age_ms":     s.get("pose_age_ms"),
                    "rpp_state":       code,
                    "rpp_state_name":  RPP_STATE_NAMES.get(code, "UNKNOWN"),
                    "armed":           s.get("armed"),
                    "mode":            s.get("mode"),
                    "connected":       s.get("connected"),
                    "battery_v":       s.get("battery_v"),
                    "battery_pct":     s.get("battery_pct"),
                    "gps_fix":         s.get("gps_fix"),
                    "gps_sat":         s.get("gps_sat"),
                    "lat":             s.get("lat"),
                    "lon":             s.get("lon"),
                    "alt":             s.get("alt"),
                }
                await sio.emit("telemetry", telem)

                mission_status = {
                    "state":          (offboard_ctrl.state.value
                                       if offboard_ctrl else "idle"),
                    "rpp_state":      code,
                    "rpp_state_name": RPP_STATE_NAMES.get(code, "UNKNOWN"),
                    "dist_to_goal":   s.get("dist_to_goal_m"),
                    "speed":          s.get("speed_m_s"),
                    "xtrack":         s.get("xtrack_m"),
                }
                await sio.emit("mission_status", mission_status)

                # ── 2. Auto-completion: RUNNING + DONE settled → COMPLETED ─────
                if (offboard_ctrl is not None
                        and offboard_ctrl.state == MissionState.RUNNING
                        and ros_node.get_rpp_monitor().is_done()):
                    offboard_ctrl.mark_completed()
                    await sio.emit("mission_completed",
                                   {"state": offboard_ctrl.state.value,
                                    "name":  offboard_ctrl.loaded_path_name})

                # ── 3. Watchdog: RUNNING + STALE/disconnected → estop ──────────
                pose_age = s.get("pose_age_ms") or 0.0
                running  = (offboard_ctrl is not None
                            and offboard_ctrl.state == MissionState.RUNNING)
                unhealthy = (
                    code == RPP_STALE
                    or pose_age > POSE_STALE_MS
                    or s.get("connected") is False
                )
                if running and unhealthy:
                    if stale_since is None:
                        stale_since = now
                    elif now - stale_since > SAFETY_STALE_GRACE_S:
                        if emergency_handler is not None:
                            log.warning(
                                "safety abort: stale=%.0fms rpp=%s connected=%s",
                                pose_age, code, s.get("connected"),
                            )
                            await emergency_handler.estop_async()
                            await sio.emit("safety_abort", {
                                "reason":      "pose stale or FCU disconnected",
                                "pose_age_ms": pose_age,
                                "rpp_state":   code,
                                "connected":   s.get("connected"),
                            })
                        stale_since = None
                else:
                    stale_since = None

                # ── 4. Disconnect notification (transition: was connected) ─────
                connected = bool(s.get("connected", False))
                if prev_connected is True and not connected:
                    await sio.emit("rover_disconnected", {})
                    _record("warning", "FCU disconnected")
                prev_connected = connected

                consecutive_errors = 0

            except asyncio.CancelledError:
                raise
            except Exception:
                consecutive_errors += 1
                log.exception("telemetry loop iteration failed (n=%d)",
                              consecutive_errors)
                # Exponential back-off on repeated failures, capped at 1 s
                await asyncio.sleep(min(1.0, 0.05 * consecutive_errors))
    finally:
        log.info("telemetry loop exited")


# ── Internal helper ───────────────────────────────────────────────────────────

def _record(level: str, message: str) -> None:
    activity_log.append({
        "timestamp": datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z",
        "level":     level,
        "message":   message,
    })
    getattr(log, level if level in ("info", "warning", "error", "debug") else "info")(
        message
    )
