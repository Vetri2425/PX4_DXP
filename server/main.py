"""Drawing Rover FastAPI backend.

Lifespan order (startup → ready → shutdown):
  1. Configure logging
  2. Initialise auth (load password hash + machine token records)
  3. rclpy.init() + RosBridgeNode + MultiThreadedExecutor in daemon thread
  4. Build shared singletons (PathManager, OffboardController, EmergencyHandler)
  5. Register Socket.IO handlers
  6. Start telemetry push loop (10 Hz) — auto-completion, disconnect notify,
     and safety_abort Socket.IO emits (fed by the safety task's queue)
  7. Start safety watchdog task (10 Hz) — E-stop on unhealthy RUNNING/ENTRY;
     systemd WATCHDOG=1 heartbeat (must fire even when ros_node is None)
  8. Start UDP discovery beacon

Shutdown reverses the order. Both loops catch and log every exception without
dying. Beacon and rclpy threads use Event-based stop signals so shutdown
completes within ~1 s.
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

from auth import authenticated_sids, init_auth, is_configured
from config import (
    AUTH_DISABLED,
    BEACON_INTERVAL,
    BEACON_PORT,
    CORS_ALLOW_CREDENTIALS,
    CORS_ALLOW_ORIGINS,
    DEFAULT_PORT,
    GPS_FIX_NAMES,
    format_gps_coord,
    MAX_ACTIVITY_LOG,
    MISSION_DIR,
    NTRIP_AUTOSTART,
    NTRIP_ENV_FILE,
    NTRIP_PROFILES_FILE,
    POSE_STALE_MS,
    RPP_DEBUG_STALE_MS,
    ROVER_ID,
    RPP_STATE_NAMES,
    RPP_UNHEALTHY_CODES,
    SAFETY_STALE_GRACE_S,
    TELEMETRY_HZ,
)
from logging_setup import configure_logging, get_logger
from models import MissionState
from origin_health import evaluate_origin_health

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
_safety_task: Optional[asyncio.Task] = None
# Safety task → telemetry loop: estop payloads for Socket.IO (safety never emits).
_safety_abort_q: Optional[asyncio.Queue] = None
bridge_health: Optional["object"] = None
rtk_manager: Optional["object"] = None
ntrip_profile_store: Optional["object"] = None
joystick_ctrl: Optional["object"] = None

# Bounded, thread-safe ring buffer (deque maxlen). All log appends are atomic
# under the GIL; bounded eviction is built in. Replaces the racy list+trim.
activity_log: deque = deque(maxlen=MAX_ACTIVITY_LOG)

log = get_logger("server.main")

# Per-SID emit ceiling. A phone leaving WiFi with a full TCP buffer must not
# stall the telemetry tick (or, historically, the co-located E-stop watchdog).
_EMIT_TIMEOUT_S = 0.5


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
    global _executor, _beacon, _listener, _telemetry_task, _safety_task
    global _safety_abort_q, bridge_health, rtk_manager, ntrip_profile_store
    global joystick_ctrl

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
        _record("warning", f"ROS2 unavailable — server running without MAVROS: {exc}")

    # ── Build shared objects ──────────────────────────────────────────────────
    from beacon import RoverBeacon, BeaconListener
    from emergency import EmergencyHandler
    from offboard_controller import OffboardController
    from path_manager import PathManager
    from ntrip_profile_store import NtripProfileStore
    from rtk_manager import AsyncRTKManager

    path_mgr = PathManager(MISSION_DIR)
    offboard_ctrl = OffboardController(ros_node, activity_log)

    # Joystick / manual control (docs/Architecture/JOYSTICK_CONTROLLER_PLAN.md).
    # Enablement is deployment-controlled via ROVER_JOYSTICK_MANUAL_ENABLED
    # (config.JOYSTICK_MANUAL_ENABLED); when False, acquire() rejects with
    # manual_control_disabled and the subsystem only streams neutral frames.
    try:
        from config import JOYSTICK_MANUAL_ENABLED
        from joystick_controller import JoystickController
        from manual_control_gateway import ManualControlGateway, build_manual_transport

        _manual_transport = build_manual_transport(ros_node)
        _manual_gateway = ManualControlGateway(_manual_transport)
        _manual_gateway.start()
        joystick_ctrl = JoystickController(ros_node, offboard_ctrl, _manual_gateway)
        _record(
            "info",
            "Joystick subsystem initialised (manual control "
            + ("ENABLED" if JOYSTICK_MANUAL_ENABLED else "disabled")
            + f", transport={_manual_transport.name})",
        )
    except Exception as exc:
        log.exception("joystick subsystem failed to initialise")
        _record("warning", f"joystick subsystem unavailable: {exc}")

    emergency_handler = EmergencyHandler(
        ros_node, offboard_ctrl, activity_log, joystick_controller=joystick_ctrl
    )
    rtk_manager = AsyncRTKManager()
    ntrip_profile_store = NtripProfileStore(NTRIP_PROFILES_FILE, NTRIP_ENV_FILE)
    try:
        migrated = ntrip_profile_store.initialize()
    except Exception:
        # A corrupt registry must fail closed: do not silently revert to the
        # legacy credential file and connect to an unintended caster.
        ntrip_profile_store = None
        log.exception("NTRIP profile registry initialization failed")
        _record(
            "warning",
            "NTRIP profile registry unavailable; inspect server logs",
        )
    else:
        if migrated:
            _record("info", "Legacy NTRIP configuration migrated to profile registry")
        elif ntrip_profile_store.migration_warning:
            log.warning("%s", ntrip_profile_store.migration_warning)
            _record("warning", ntrip_profile_store.migration_warning)
    if NTRIP_AUTOSTART:
        try:
            if ntrip_profile_store is None:
                raise RuntimeError("NTRIP profile registry unavailable")
            profile_id, profile_revision, ntrip_config = (
                ntrip_profile_store.default_config()
            )
            ntrip_status = await rtk_manager.start_ntrip_profile(
                ntrip_config,
                profile_id=profile_id,
                profile_revision=profile_revision,
            )
        except Exception as exc:
            # RTK availability is safety-gated by the controllers. Keep the
            # backend available so the operator can diagnose/fix credentials.
            await rtk_manager.mark_ntrip_unavailable(str(exc))
            log.exception("NTRIP autostart failed")
            _record(
                "warning",
                "NTRIP autostart unavailable; check RTK status and server logs",
            )
        else:
            _record("info", f"NTRIP autostarted pid={ntrip_status.pid}")
    else:
        _record("warning", "NTRIP autostart disabled by deployment configuration")

    # ── Register Socket.IO handlers ───────────────────────────────────────────
    from sockets.events import register_handlers

    register_handlers(sio)

    # ── Start telemetry + safety watchdog (separate tasks — S2) ───────────────
    # Unbounded + put_nowait: a future maxsize must not turn enqueue into a
    # blocking await on the E-stop path. Prefer dropping a UI notification.
    _safety_abort_q = asyncio.Queue()
    _telemetry_task = asyncio.create_task(_telemetry_loop(), name="telemetry-loop")
    _safety_task = asyncio.create_task(_safety_watchdog_loop(), name="safety-watchdog")

    # ── Start bridge-health watchdog (Phase 3A: observe-only by default) ───────
    try:
        from bridge_health import BridgeHealthManager

        async def _bounded_bridge_emit(event: str, data: dict) -> None:
            await asyncio.wait_for(sio.emit(event, data), timeout=_EMIT_TIMEOUT_S)

        bridge_health = BridgeHealthManager(
            ros_node, offboard_ctrl, _record, _bounded_bridge_emit
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
        auth_required=(not AUTH_DISABLED) and is_configured(),
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
            log.exception("joystick controller shutdown raised")

    if rtk_manager is not None:
        try:
            await rtk_manager.stop_all()
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

    if _safety_task:
        _safety_task.cancel()
        try:
            await _safety_task
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
    from routes.auth import router as auth_router
    from routes.vehicle import router as veh_router
    from routes.mission import router as mis_router
    from routes.path import paths_router, path_router
    from routes.params import router as par_router
    from routes.rpp_params import router as rpp_par_router
    from routes.telemetry import router as tel_router
    from routes.rtk import router as rtk_router
    from routes.spray import router as spray_router
    from routes.spray_params import router as spray_par_router

    app.include_router(sys_router, prefix="/api")
    app.include_router(auth_router, prefix="/api")
    app.include_router(veh_router, prefix="/api")
    app.include_router(mis_router, prefix="/api")
    app.include_router(paths_router, prefix="/api")  # → /api/paths
    app.include_router(path_router, prefix="/api")  # → /api/path/*
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


async def _emit_authenticated(event: str, data: dict) -> None:
    """Push an event only to Socket.IO SIDs that passed connect-time auth.

    Each emit is isolated: a sid that disconnects between the authenticated_sids()
    snapshot and its awaited emit raises, and without this guard that exception
    would abandon the whole tick — dropping telemetry for every *other* connected
    operator too. Each emit is also time-bounded (S2): a phone leaving WiFi with
    a full TCP buffer must not stall the tick — log and continue to the next SID.
    """
    for sid in authenticated_sids():
        try:
            await asyncio.wait_for(
                sio.emit(event, data, to=sid),
                timeout=_EMIT_TIMEOUT_S,
            )
        except asyncio.TimeoutError:
            log.warning(
                "emit %s to sid=%s timed out after %.1fs — continuing",
                event,
                sid,
                _EMIT_TIMEOUT_S,
            )
        except Exception:
            log.debug("emit %s to sid=%s failed (client likely gone)", event, sid, exc_info=True)


async def _drain_safety_aborts() -> None:
    """Forward queued safety-abort payloads to authenticated Socket.IO clients.

    The safety task never touches Socket.IO — it only enqueues. Emitting here
    keeps a wedged emit from starving the E-stop path (S2).
    """
    if _safety_abort_q is None:
        return
    while True:
        try:
            payload = _safety_abort_q.get_nowait()
        except asyncio.QueueEmpty:
            break
        await _emit_authenticated("safety_abort", payload)


async def _safety_watchdog_loop() -> None:
    """E-stop watchdog + systemd heartbeat — never touches Socket.IO (S2).

    Calls only ros_node.get_state() and emergency_handler.estop_async(). Abort
    UI notification is queued for the telemetry loop. WATCHDOG=1 lives here
    because the process state worth protecting is the safety abort, not the
    telemetry push; must still fire when ros_node is None (S1-a).

    Estop wall-clock budget vs WatchdogSec=15: estop_async can block this task
    ~10 s on two back-to-back 5 s set_mode/arm service timeouts. That misses
    ~3 of 5 WATCHDOG beats but still lands inside 15 s, so systemd does not
    restart mid-stop. publish_stop_path() is synchronous and runs FIRST inside
    estop_async, so the vehicle-stopping ROS publish lands before any await.
    Do not add a third awaited service call (or raise those timeouts) without
    revisiting this budget — a longer stall would let systemd kill the process
    in the middle of an emergency stop.
    """
    interval = 1.0 / TELEMETRY_HZ
    stale_since: Optional[float] = None
    consecutive_errors = 0
    _watchdog_counter = 0
    _WATCHDOG_EVERY_N = TELEMETRY_HZ * 3  # ping systemd every ~3s

    log.info("safety watchdog started @ %d Hz", TELEMETRY_HZ)
    try:
        while True:
            try:
                await asyncio.sleep(interval)

                # Systemd heartbeat — loop alive, not "ROS healthy". Above the
                # ros_node None continue so degraded boot still feeds WatchdogSec.
                _watchdog_counter += 1
                if _sd_notifier and _watchdog_counter >= _WATCHDOG_EVERY_N:
                    _sd_notifier.notify("WATCHDOG=1")
                    _watchdog_counter = 0

                if ros_node is None:
                    continue

                s = ros_node.get_state()
                code = s.get("rpp_state", 0)
                now = time.time()
                # B2: RPP_UNHEALTHY_CODES covers STALE (-1), RTK_WAIT (4),
                # JUMP_SKIP (5). All three mean "controller is publishing
                # zero velocity for a safety reason" — same response.
                pose_age = s.get("pose_age_ms") or 0.0
                running = (
                    offboard_ctrl is not None
                    and offboard_ctrl.state in (MissionState.RUNNING, MissionState.ENTRY)
                )
                # Controller-death detection. `code` and `pose_age` above are BOTH
                # self-reported by the RPP controller, so when that process dies they
                # freeze at their last healthy values and this watchdog would happily
                # keep trusting a corpse. rpp_debug_age_ms is measured by us, on
                # receipt, and is the only field the dead process cannot fake.
                #
                # This also closes the mid-mission restart hazard: /path is published
                # TRANSIENT_LOCAL and RPP's _path_cb always _apply_run(0) with no
                # persisted progress, so a crashed-and-restarted controller would pick
                # the latched mission back up and re-drive it from run 0 across
                # already-marked ground. (PX4's own failsafe does NOT save us here:
                # twist_to_setpoint keeps streaming zero-velocity setpoints, so the
                # OFFBOARD stream never gaps and the rover stays armed.) Tripping the
                # watchdog runs estop_async(), which publishes a single-point stop-path
                # — replacing the latched mission — so a restarting RPP wakes up to a
                # stop, not a re-run.
                #
                # None => never heard from RPP at all; we can't judge, so don't trip
                # (avoids false aborts where /rpp/debug simply isn't wired).
                rpp_age = s.get("rpp_debug_age_ms")
                rpp_dead = rpp_age is not None and rpp_age > RPP_DEBUG_STALE_MS
                unhealthy = (
                    code in RPP_UNHEALTHY_CODES
                    or pose_age > POSE_STALE_MS
                    or rpp_dead
                    or s.get("connected") is False
                )
                if running and unhealthy:
                    if stale_since is None:
                        stale_since = now
                    elif now - stale_since > SAFETY_STALE_GRACE_S:
                        if emergency_handler is not None:
                            rpp_name = RPP_STATE_NAMES.get(code, f"?{code}")
                            # Name the actual cause. "controller not responding" and
                            # "pose stale" call for very different operator responses,
                            # and a dead controller must not be reported as a bad fix.
                            if rpp_dead:
                                reason = "RPP controller not responding (process died?)"
                            elif s.get("connected") is False:
                                reason = "FCU disconnected"
                            elif pose_age > POSE_STALE_MS:
                                reason = "pose stale"
                            else:
                                reason = f"RPP unhealthy: {rpp_name}"
                            log.warning(
                                "safety abort: %s | pose_stale=%.0fms rpp_debug_age=%s "
                                "rpp=%s(%s) connected=%s",
                                reason,
                                pose_age,
                                f"{rpp_age:.0f}ms" if rpp_age is not None else "never",
                                code,
                                rpp_name,
                                s.get("connected"),
                            )
                            await emergency_handler.estop_async()
                            if _safety_abort_q is not None:
                                try:
                                    _safety_abort_q.put_nowait(
                                        {
                                            "reason": reason,
                                            "pose_age_ms": pose_age,
                                            "rpp_debug_age_ms": rpp_age,
                                            "rpp_state": code,
                                            "rpp_state_name": RPP_STATE_NAMES.get(
                                                code, "UNKNOWN"
                                            ),
                                            "connected": s.get("connected"),
                                        }
                                    )
                                except asyncio.QueueFull:
                                    # Unrepresentable with unbounded Queue; if a
                                    # maxsize appears later, drop the UI notice
                                    # rather than block the E-stop path.
                                    log.error(
                                        "safety_abort queue full — UI notification "
                                        "dropped (estop already ran)"
                                    )
                        stale_since = None
                else:
                    stale_since = None

                consecutive_errors = 0

            except asyncio.CancelledError:
                raise
            except Exception:
                consecutive_errors += 1
                log.exception(
                    "safety watchdog iteration failed (n=%d)", consecutive_errors
                )
                await asyncio.sleep(min(1.0, 0.05 * consecutive_errors))
    finally:
        log.info("safety watchdog exited")


async def _telemetry_loop() -> None:
    interval = 1.0 / TELEMETRY_HZ
    prev_connected: Optional[bool] = None
    consecutive_errors = 0

    log.info("telemetry loop started @ %d Hz", TELEMETRY_HZ)
    try:
        while True:
            try:
                await asyncio.sleep(interval)
                # Forward any E-stop notifications from the safety task before
                # other work — each emit is time-bounded (S2).
                await _drain_safety_aborts()

                if ros_node is None:
                    continue

                s = ros_node.get_state()
                origin_health = evaluate_origin_health(s)
                code = s.get("rpp_state", 0)
                spraying = bool(s.get("spraying", False))
                mission_running = (
                    offboard_ctrl is not None
                    and offboard_ctrl.state in (MissionState.RUNNING, MissionState.ENTRY)
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
                    "gps_fix_name": GPS_FIX_NAMES.get(
                        s.get("gps_fix", 0), f"FIX_{s.get('gps_fix', 0)}"
                    ),
                    "gps_sat": s.get("gps_sat"),
                    "hrms": s.get("hrms"),
                    "vrms": s.get("vrms"),
                    "gps_accuracy_known": s.get("gps_accuracy_known"),
                    "lat": format_gps_coord(s.get("lat")),
                    "lon": format_gps_coord(s.get("lon")),
                    "alt": format_gps_coord(s.get("alt")),
                    # Freshness. These were already computed but never streamed, so
                    # the client had no way to tell "RTK_FIXED now" from "was
                    # RTK_FIXED five minutes ago" — local pose keeps updating from
                    # wheel odometry after an RTK drop, so everything else still
                    # looks healthy. rpp_debug_age_ms is the one that reveals a dead
                    # controller: every other rpp_* field freezes at its last value.
                    "rpp_debug_age_ms": s.get("rpp_debug_age_ms"),
                    "local_pose_age_ms": s.get("local_pose_age_ms"),
                    "global_position_age_ms": s.get("global_position_age_ms"),
                    "gps_fix_age_ms": s.get("gps_fix_age_ms"),
                    "pose_global_skew_ms": s.get("pose_global_skew_ms"),
                    # EKF origin trust — the only warning the operator gets
                    # before a stale origin displaces the whole mission. Same
                    # evaluation the placement gate enforces.
                    "origin_status": origin_health.status,
                    "origin_trusted": origin_health.trusted,
                    "origin_delta_m": origin_health.delta_m,
                }
                # Joystick/arbiter snapshot (plan §5) — load-bearing for client
                # safety: the client detects lease loss / mission takeover only
                # through these fields. Merged in raw (not pydantic-validated;
                # this emit is a plain dict, unlike the REST TelemetryData model).
                if joystick_ctrl is not None:
                    try:
                        telem.update(joystick_ctrl.snapshot())
                    except Exception:
                        log.exception("joystick snapshot failed")
                await _emit_authenticated("telemetry", _sanitize(telem))

                mission_status = {
                    "state": (offboard_ctrl.state.value if offboard_ctrl else "idle"),
                    "rpp_state": code,
                    "rpp_state_name": RPP_STATE_NAMES.get(code, "UNKNOWN"),
                    "dist_to_goal": s.get("dist_to_goal_m"),
                    "speed": s.get("speed_m_s"),
                    "xtrack": s.get("xtrack_m"),
                }
                await _emit_authenticated("mission_status", _sanitize(mission_status))

                # ── 1b. D1 entry phase 2: ENTRY + DONE settled → publish mark ──
                # The rover has driven the spray-OFF entry leg and stopped on the
                # first point (D3 completion latch → RPP DONE). Publish the
                # stashed marking path and transition ENTRY→RUNNING.
                if (
                    offboard_ctrl is not None
                    and offboard_ctrl.state == MissionState.ENTRY
                    and ros_node.get_rpp_monitor().is_done()
                ):
                    if offboard_ctrl.advance_entry_to_marking():
                        await _emit_authenticated(
                            "entry_complete",
                            {
                                "state": offboard_ctrl.state.value,
                                "name": offboard_ctrl.loaded_path_name,
                            },
                        )

                # ── 2. Auto-completion: RUNNING + DONE settled → COMPLETED ─────
                if (
                    offboard_ctrl is not None
                    and offboard_ctrl.state == MissionState.RUNNING
                    and ros_node.get_rpp_monitor().is_done()
                ):
                    if offboard_ctrl.mark_completed():
                        await _emit_authenticated(
                            "mission_completed",
                            {
                                "state": offboard_ctrl.state.value,
                                "name": offboard_ctrl.loaded_path_name,
                            },
                        )
                        # B4: end the mission spray-OFF + DISARMED without an
                        # operator E-stop. Self-gated on DISARM_ON_COMPLETE; runs
                        # strictly AFTER the COMPLETED transition (never keyed on
                        # a raw DONE — B16: /rpp/debug can flash DONE mid-mission,
                        # but the RPP monitor's settle logic gates the transition
                        # above). Never raises.
                        try:
                            await offboard_ctrl.disarm_on_complete_async()
                        except Exception:
                            log.exception("disarm-on-complete failed")

                # ── 3. Disconnect notification (transition: was connected) ─────
                # E-stop watchdog lives in _safety_watchdog_loop (S2).
                connected = bool(s.get("connected", False))
                if prev_connected is True and not connected:
                    await _emit_authenticated("rover_disconnected", {})
                    _record("warning", "FCU disconnected")
                prev_connected = connected

                consecutive_errors = 0

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
