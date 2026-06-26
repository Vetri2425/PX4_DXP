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
import math
import time
from collections import deque
from contextlib import asynccontextmanager
from typing import Optional

import socketio
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.gzip import GZipMiddleware

from auth import init_auth
from config import (
    BEACON_INTERVAL,
    BEACON_PORT,
    CORS_ALLOW_CREDENTIALS,
    CORS_ALLOW_ORIGINS,
    DEFAULT_PORT,
    GPS_FIX_NAMES,
    MAX_ACTIVITY_LOG,
    MISSION_DIR,
    POSE_STALE_MS,
    ROVER_ID,
    RPP_STATE_NAMES,
    RPP_UNHEALTHY_CODES,
    SAFETY_STALE_GRACE_S,
    TELEMETRY_HZ,
)
from logging_setup import configure_logging, get_logger
from models import MissionState

# ── sd_notify for systemd watchdog ────────────────────────────────────────────
_sd_notifier = None
try:
    import sdnotify

    _sd_notifier = sdnotify.SystemdNotifier()
except ImportError:
    pass

# ── Module-level singletons (populated in lifespan) ───────────────────────────
ros_node: Optional["object"] = None
offboard_ctrl: Optional["object"] = None
path_mgr: Optional["object"] = None
emergency_handler: Optional["object"] = None
_executor: Optional["object"] = None
_beacon: Optional["object"] = None
_listener: Optional["object"] = None
_telemetry_task: Optional[asyncio.Task] = None
bridge_health: Optional["object"] = None
rtk_manager: Optional["object"] = None
mission_capture: Optional["object"] = None
point_mission: Optional["object"] = None
hold_owner: Optional["object"] = None
manual_gateway: Optional["object"] = None
joystick_ctrl: Optional["object"] = None

# Bounded, thread-safe ring buffer (deque maxlen). All log appends are atomic
# under the GIL; bounded eviction is built in. Replaces the racy list+trim.
activity_log: deque = deque(maxlen=MAX_ACTIVITY_LOG)

log = get_logger("server.main")


# ── Socket.IO ASGI app ────────────────────────────────────────────────────────
# cors_allowed_origins must match the REST CORS policy — they are independent
# implementations and both must agree.
sio = socketio.AsyncServer(
    async_mode="asgi",
    cors_allowed_origins="*" if "*" in CORS_ALLOW_ORIGINS else CORS_ALLOW_ORIGINS,
)
socket_app = socketio.ASGIApp(sio)


# ── Lifespan ──────────────────────────────────────────────────────────────────


@asynccontextmanager
async def lifespan(app: FastAPI):
    global ros_node, offboard_ctrl, path_mgr, emergency_handler
    global _executor, _beacon, _listener, _telemetry_task, bridge_health, rtk_manager
    global mission_capture, point_mission, hold_owner, manual_gateway, joystick_ctrl

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
        # Reset operator authorization after a server-only restart too; the
        # spray node may have survived with its prior parameter value.
        disabled = False
        disable_reason = "spray_controller not discovered"
        for _ in range(5):
            disabled, disable_reason = await ros_node.set_spray_param_async(
                "spray_enabled", False, timeout=1.0
            )
            if disabled:
                break
            await asyncio.sleep(0.1)
        if not disabled:
            _record("warning", f"Could not reset spray authorization: {disable_reason}")
    except Exception as exc:
        log.exception("ROS2 startup failed — continuing without MAVROS")
        _record("warning", f"ROS2 unavailable — server running without MAVROS: {exc}")

    # ── Build shared objects ──────────────────────────────────────────────────
    from beacon import RoverBeacon, BeaconListener
    from emergency import EmergencyHandler
    from offboard_controller import OffboardController
    from path_manager import PathManager
    from rtk_manager import AsyncRTKManager
    from mission_debug_capture import MissionDebugCoordinator
    from manual_control_gateway import ManualControlGateway, build_manual_transport
    from point_mission import PointMissionOrchestrator
    from setpoint_hold import SetpointHoldOwner

    path_mgr = PathManager(MISSION_DIR)
    offboard_ctrl = OffboardController(ros_node, activity_log)
    manual_gateway = ManualControlGateway(build_manual_transport(ros_node))
    manual_gateway.start()
    from joystick_controller import JoystickController

    joystick_ctrl = JoystickController(ros_node, offboard_ctrl, manual_gateway)
    hold_owner = SetpointHoldOwner()
    point_mission = PointMissionOrchestrator()
    point_mission.set_logger(_record)
    if ros_node is not None:
        ros_node.set_obstacle_callback(point_mission.set_obstacle_clear)
    mission_capture = MissionDebugCoordinator()
    emergency_handler = EmergencyHandler(
        ros_node, offboard_ctrl, activity_log, mission_capture
    )
    def _rtk_navigation_state() -> dict:
        if ros_node is None:
            return {}
        return ros_node.get_state()

    rtk_manager = AsyncRTKManager(navigation_provider=_rtk_navigation_state)

    # ── Register Socket.IO handlers ───────────────────────────────────────────
    from sockets.events import register_handlers

    register_handlers(sio)

    # ── Start telemetry + watchdog loop ───────────────────────────────────────
    _telemetry_task = asyncio.create_task(_telemetry_loop(), name="telemetry-loop")

    # ── Start bridge-health watchdog (Phase 3A: observe-only by default) ───────
    try:
        from bridge_health import BridgeHealthManager

        bridge_health = BridgeHealthManager(
            ros_node, offboard_ctrl, _record, sio.emit
        )
        bridge_health.start()
    except Exception as exc:
        log.exception("BridgeHealthManager failed to start")
        _record("warning", f"bridge-health watchdog unavailable: {exc}")

    # ── Start UDP discovery beacon ────────────────────────────────────────────
    _beacon = RoverBeacon(
        port=BEACON_PORT,
        interval=BEACON_INTERVAL,
        rover_id=ROVER_ID,
        server_port=DEFAULT_PORT,
    )
    _beacon.start()
    _listener = BeaconListener(port=BEACON_PORT)
    _listener.start()

    _record("info", f"Server ready on port {DEFAULT_PORT}")
    log.info("server ready: port=%d telemetry=%dHz", DEFAULT_PORT, TELEMETRY_HZ)

    # Notify systemd that we're ready (Type=notify)
    if _sd_notifier:
        _sd_notifier.notify("READY=1")

    yield  # ─── Running ───────────────────────────────────────────────────────

    # ── Shutdown ──────────────────────────────────────────────────────────────
    log.info("shutting down…")

    if joystick_ctrl is not None:
        try:
            await joystick_ctrl.shutdown()
        except Exception:
            log.exception("joystick shutdown raised")

    if rtk_manager is not None:
        try:
            await rtk_manager.shutdown()
        except Exception:
            log.exception("RTK manager stop raised")

    if bridge_health is not None:
        try:
            await bridge_health.stop()
        except Exception:
            log.exception("bridge-health stop raised")

    if _telemetry_task:
        _telemetry_task.cancel()
        try:
            await _telemetry_task
        except (asyncio.CancelledError, Exception):
            pass

    if _listener:
        _listener.stop()
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
    app.add_middleware(GZipMiddleware, minimum_size=1024)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=CORS_ALLOW_ORIGINS,
        allow_credentials=CORS_ALLOW_CREDENTIALS,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # REST routers
    from routes.system import router as sys_router
    from routes.vehicle import router as veh_router
    from routes.mission import router as mis_router
    from routes.path import paths_router, path_router
    from routes.params import router as par_router
    from routes.rpp_params import router as rpp_par_router
    from routes.telemetry import router as tel_router
    from routes.rtk import router as rtk_router
    from routes.spray import router as spray_router
    from routes.spray_params import router as spray_par_router
    from routes.spray_mode import router as spray_mode_router

    app.include_router(sys_router, prefix="/api")
    app.include_router(veh_router, prefix="/api")
    app.include_router(mis_router, prefix="/api")
    app.include_router(paths_router, prefix="/api")  # → /api/paths
    app.include_router(path_router, prefix="/api")  # → /api/path/*
    app.include_router(spray_mode_router, prefix="/api")  # → /api/path/{name}/spray-mode/*
    app.include_router(par_router, prefix="/api")
    app.include_router(rpp_par_router, prefix="/api")
    app.include_router(tel_router, prefix="/api")
    app.include_router(rtk_router, prefix="/api")
    app.include_router(spray_router, prefix="/api")   # → /api/spray/*
    app.include_router(spray_par_router, prefix="/api")  # → /api/spray/params/*

    # Socket.IO
    app.mount("/socket.io", socket_app)
    return app


app = create_app()


# ── Telemetry loop with watchdog and auto-completion ──────────────────────────


def _sanitize(d: dict) -> dict:
    """Replace float NaN/Inf with None so Socket.IO emits valid JSON.

    Python's json encoder writes the bare token NaN for float('nan'), which is
    illegal JSON and causes JS JSON.parse() to throw, disconnecting the client.
    """
    return {
        k: (None if isinstance(v, float) and not math.isfinite(v) else v)
        for k, v in d.items()
    }


async def _telemetry_loop() -> None:
    interval = 1.0 / TELEMETRY_HZ
    prev_connected: Optional[bool] = None
    stale_since: Optional[float] = None
    consecutive_errors = 0
    _watchdog_counter = 0
    _WATCHDOG_EVERY_N = TELEMETRY_HZ * 3  # ping systemd every ~3s

    log.info("telemetry loop started @ %d Hz", TELEMETRY_HZ)
    try:
        while True:
            try:
                await asyncio.sleep(interval)
                if ros_node is None:
                    continue

                s = ros_node.get_state()
                code = s.get("rpp_state", 0)
                now = time.time()
                spraying = bool(s.get("spraying", False))
                mission_running = (
                    offboard_ctrl is not None
                    and offboard_ctrl.state == MissionState.RUNNING
                    and bool(s.get("armed", False))
                )
                if not mission_running:
                    marking_state = "off"
                elif spraying:
                    marking_state = "marking"
                else:
                    marking_state = "transit"

                # ── 1. Push telemetry ──────────────────────────────────────────
                telem = {
                    "pos_n": s.get("pos_n"),
                    "pos_e": s.get("pos_e"),
                    "heading_ned_deg": s.get("heading_ned_deg"),
                    "xtrack_m": s.get("xtrack_m"),
                    "heading_err_deg": s.get("heading_err_deg"),
                    "lookahead_m": s.get("lookahead_m"),
                    "speed_m_s": s.get("speed_m_s"),
                    "measured_speed_m_s": s.get("measured_speed_m_s"),
                    "kappa": s.get("kappa"),
                    "dist_to_goal_m": s.get("dist_to_goal_m"),
                    "pose_age_ms": s.get("pose_age_ms"),
                    "rpp_state": code,
                    "rpp_state_name": RPP_STATE_NAMES.get(code, "UNKNOWN"),
                    "spraying": spraying,
                    "marking_state": marking_state,
                    "armed": s.get("armed"),
                    "mode": s.get("mode"),
                    "connected": s.get("connected"),
                    "battery_v": s.get("battery_v"),
                    "battery_pct": s.get("battery_pct"),
                    "gps_fix": s.get("gps_fix"),
                    "gps_fix_name": GPS_FIX_NAMES.get(s.get("gps_fix", 0), "UNKNOWN"),
                    "gps_sat": s.get("gps_sat"),
                    "hrms": s.get("hrms"),
                    "vrms": s.get("vrms"),
                    "lat": s.get("lat"),
                    "lon": s.get("lon"),
                    "alt": s.get("alt"),
                }
                if joystick_ctrl is not None:
                    telem.update(joystick_ctrl.snapshot())
                await sio.emit("telemetry", _sanitize(telem))

                mission_status = {
                    "state": (offboard_ctrl.state.value if offboard_ctrl else "idle"),
                    "rpp_state": code,
                    "rpp_state_name": RPP_STATE_NAMES.get(code, "UNKNOWN"),
                    "dist_to_goal": s.get("dist_to_goal_m"),
                    "speed": s.get("speed_m_s"),
                    "xtrack": s.get("xtrack_m"),
                    "rpp_debug_age_ms": s.get("rpp_debug_age_ms"),
                    "rpp_debug_fresh": s.get("rpp_debug_fresh"),
                    "measured_speed_m_s": s.get("measured_speed_m_s"),
                }
                await sio.emit("mission_status", _sanitize(mission_status))

                # ── 2. Auto-completion: RUNNING + DONE settled → COMPLETED ─────
                if (
                    offboard_ctrl is not None
                    and offboard_ctrl.state == MissionState.RUNNING
                    and ros_node.get_rpp_monitor().is_done()
                ):
                    completion = await offboard_ctrl.complete_async()
                    completion_ok = bool(completion.get("success"))
                    terminal_reason = (
                        "mission_completed"
                        if completion_ok else "mission_completion_degraded"
                    )
                    if mission_capture is not None:
                        mission_capture.record_terminal(
                            None,
                            terminal_reason,
                            state=offboard_ctrl.state.value,
                            details=completion,
                        )
                    await sio.emit(
                        terminal_reason,
                        {
                            "state": offboard_ctrl.state.value,
                            "name": offboard_ctrl.loaded_path_name,
                            "terminal": completion,
                        },
                    )

                # ── 3. Watchdog: RUNNING + unhealthy/disconnected → estop ──────
                # B2: RPP_UNHEALTHY_CODES covers STALE (-1), RTK_WAIT (4),
                # JUMP_SKIP (5). All three mean "controller is publishing
                # zero velocity for a safety reason" — same response.
                pose_age = s.get("pose_age_ms") or 0.0
                running = (
                    offboard_ctrl is not None
                    and offboard_ctrl.state == MissionState.RUNNING
                )
                unhealthy = (
                    code in RPP_UNHEALTHY_CODES
                    or pose_age > POSE_STALE_MS
                    or s.get("connected") is False
                    or s.get("rpp_debug_fresh") is not True
                )
                if running and unhealthy:
                    if stale_since is None:
                        stale_since = now
                    elif now - stale_since > SAFETY_STALE_GRACE_S:
                        if emergency_handler is not None:
                            rpp_name = RPP_STATE_NAMES.get(code, f"?{code}")
                            log.warning(
                                "safety abort: stale=%.0fms rpp=%s(%s) connected=%s",
                                pose_age,
                                code,
                                rpp_name,
                                s.get("connected"),
                            )
                            await emergency_handler.estop_async()
                            await sio.emit(
                                "safety_abort",
                                {
                                    "reason": "pose stale or FCU disconnected",
                                    "pose_age_ms": pose_age,
                                    "rpp_state": code,
                                    "rpp_state_name": RPP_STATE_NAMES.get(
                                        code, "UNKNOWN"
                                    ),
                                    "connected": s.get("connected"),
                                },
                            )
                        stale_since = None
                else:
                    stale_since = None

                # Joystick MANUAL mode has a 500 ms PX4 RC_LOSS fallback. On
                # confirmed FCU/MAVROS disconnect, revoke ownership and run the
                # established hard e-stop path.
                if (
                    joystick_ctrl is not None
                    and joystick_ctrl.is_active
                    and s.get("connected") is False
                ):
                    await joystick_ctrl.force_release(reason="fcu_disconnected")
                    if emergency_handler is not None:
                        await emergency_handler.estop_async()

                # ── 4. Disconnect notification (transition: was connected) ─────
                connected = bool(s.get("connected", False))
                if prev_connected is True and not connected:
                    await sio.emit("rover_disconnected", {})
                    _record("warning", "FCU disconnected")
                prev_connected = connected

                consecutive_errors = 0

                # ── 5. Systemd watchdog heartbeat ──────────────────────────────
                _watchdog_counter += 1
                if _sd_notifier and _watchdog_counter >= _WATCHDOG_EVERY_N:
                    _sd_notifier.notify("WATCHDOG=1")
                    _watchdog_counter = 0

            except asyncio.CancelledError:
                raise
            except Exception:
                consecutive_errors += 1
                log.exception(
                    "telemetry loop iteration failed (n=%d)", consecutive_errors
                )
                # Exponential back-off on repeated failures, capped at 1 s
                await asyncio.sleep(min(1.0, 0.05 * consecutive_errors))
    finally:
        log.info("telemetry loop exited")


# ── Internal helper ───────────────────────────────────────────────────────────


def _record(level: str, message: str) -> None:
    activity_log.append(
        {
            "timestamp": datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z",
            "level": level,
            "message": message,
        }
    )
    getattr(log, level if level in ("info", "warning", "error", "debug") else "info")(
        message
    )
