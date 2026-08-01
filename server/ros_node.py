"""Single rclpy node + MultiThreadedExecutor running in a background thread.

Threading model:
  - One rclpy node (`RosBridgeNode`).
  - Spun by a `MultiThreadedExecutor(num_threads=4)` on a daemon thread.
  - All service clients live in a `ReentrantCallbackGroup` so they can be
    invoked from any thread without deadlock.
  - Public methods are *async*: each wraps `call_async` with
    `add_done_callback` + `loop.call_soon_threadsafe(future.set_result, ...)`,
    so the FastAPI event loop is never blocked.

Routes / sockets call `await ros_node.arm_async(...)` etc. The legacy sync
methods `arm()` / `set_mode()` are kept as thin wrappers that block the
caller's thread (used only by the offboard controller's sync `start()`
shim if ever needed) but **must not** be called from the asyncio loop.
"""

from __future__ import annotations

import asyncio
import json
import math
import threading
import time
from typing import Any, Callable

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32MultiArray, String

from config import (
    ORIGIN_CONSISTENCY_MAX_M,
    ORIGIN_LINK_GAP_S,
    ORIGIN_REQUEST_MAX_TRIES,
    ORIGIN_REQUEST_PERIOD_S,
    SRV_RPP_GET_PARAMS,
    SRV_RPP_LIST_PARAMS,
    SRV_RPP_SET_PARAMS,
    SRV_SPRAY_GET_PARAMS,
    SRV_SPRAY_SET_PARAMS,
)
from logging_setup import get_logger
from origin_health import INCONSISTENT, OK, UNVERIFIABLE, evaluate_origin_health
from rpp_status import RppStatusMonitor

log = get_logger("server.ros")

# ── Optional MAVROS imports ───────────────────────────────────────────────────
try:
    from mavros_msgs.msg import State
    from sensor_msgs.msg import BatteryState, NavSatFix
    from mavros_msgs.srv import CommandBool, CommandLong, SetMode

    _HAS_MAVROS = True
except ImportError:
    _HAS_MAVROS = False
    State = BatteryState = NavSatFix = CommandBool = SetMode = None  # type: ignore
    CommandLong = None  # type: ignore

try:
    from geographic_msgs.msg import GeoPointStamped

    _HAS_GEOPOINT = True
except ImportError:
    _HAS_GEOPOINT = False
    GeoPointStamped = None  # type: ignore

try:
    from mavros_msgs.msg import GPSRAW

    _HAS_GPSRAW = True
except ImportError:
    _HAS_GPSRAW = False
    GPSRAW = None  # type: ignore

# Standard rcl_interfaces param services (always available with ROS2)
try:
    from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

    _HAS_PARAM_SRV = True
except ImportError:
    _HAS_PARAM_SRV = False
    GetParameters = SetParameters = ListParameters = None  # type: ignore
    Parameter = ParameterValue = ParameterType = None  # type: ignore


# ── QoS helpers ───────────────────────────────────────────────────────────────


def _qos_reliable_tl(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


def _qos_best_effort(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


# ── Executor lifecycle helper ─────────────────────────────────────────────────


class RosExecutorThread:
    """Owns a MultiThreadedExecutor, drains it cooperatively in a thread."""

    def __init__(self, num_threads: int = 4) -> None:
        self._exe = MultiThreadedExecutor(num_threads=num_threads)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def add_node(self, node: Node) -> None:
        self._exe.add_node(node)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._spin_loop, daemon=True, name="rclpy-mt-spin"
        )
        self._thread.start()

    def stop(self, join_timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=join_timeout)

    def _spin_loop(self) -> None:
        try:
            while rclpy.ok() and not self._stop.is_set():
                # spin_once with timeout lets us notice the stop event
                self._exe.spin_once(timeout_sec=0.1)
        except ExternalShutdownException:
            # SIGTERM: rclpy's signal handler shuts the context down while
            # spin_once is blocked in it. Expected on every service stop —
            # logging it as a crash buried real tracebacks in restart noise.
            log.info("rclpy context externally shut down — executor thread exiting")
        except Exception:
            log.exception("rclpy executor crashed")
        finally:
            try:
                self._exe.shutdown()
            except Exception:
                pass


# ── Node ──────────────────────────────────────────────────────────────────────


class RosBridgeNode(Node):
    """Single rclpy node; thread-safe shared state dict."""

    _DEFAULT_STATE: dict[str, Any] = {
        "armed": False,
        "mode": "UNKNOWN",
        "connected": False,
        "pos_n": 0.0,
        "pos_e": 0.0,
        "pose_received": False,
        "global_position_received": False,
        # EKF-declared local-frame origin (GPS_GLOBAL_ORIGIN -> gp_origin). Fixed
        # for the EKF session, so using it makes mission placement deterministic.
        "ekf_origin_lat": 0.0,
        "ekf_origin_lon": 0.0,
        "ekf_origin_received": False,
        "ekf_origin_stamp": None,
        # Why the cached origin was last dropped (FCU reboot / MAVROS restart /
        # measured inconsistency) and when — surfaced verbatim in the placement
        # refusal and on GET /api/health/origin so the operator sees the cause,
        # not just the symptom. None => never invalidated in this process.
        "ekf_origin_invalid_reason": None,
        "ekf_origin_invalidated_at": None,
        "ekf_origin_invalidations": 0,
        "gps_fix_received": False,
        "heading_ned_deg": 0.0,
        "battery_v": 0.0,
        "battery_pct": 0.0,
        "lat": 0.0,
        "lon": 0.0,
        "alt": 0.0,
        "gps_fix": 0,
        "gps_sat": 0,
        # A14: None = "not reported yet / not trustworthy", never 0.0. A zero
        # here is indistinguishable from a perfect fix.
        "hrms": None,
        "vrms": None,
        "gps_accuracy_known": False,
        "position_covariance_type": 0,
        "xtrack_m": 0.0,
        "heading_err_deg": 0.0,
        "lookahead_m": 0.0,
        "speed_m_s": 0.0,
        "kappa": 0.0,
        "dist_to_goal_m": 0.0,
        "pose_age_ms": 0.0,
        "rpp_state": 0,
        "v_north": 0.0,
        "v_east": 0.0,
        # B1 — predictive κ and pre-clamp Ld for tuning analysis
        "l_d_raw_m": 0.0,
        "kappa_speed": 0.0,
        "spraying": False,
        "spray_active": False,
        "spray_manual": False,
        # Current spray MODE + mode_state, mirrored from the node's rich
        # /spray/status (String JSON). None until the spray node is first heard
        # from, so callers can distinguish "continuous" from "node not reporting".
        "spray_mode": None,
        "spray_mode_state": None,
        # Safety gate as the spray NODE sees it. `spraying` alone cannot explain
        # a dry pass: the valve stays shut both when the geometry says "no paint
        # here" and when a gate refuses, and only the reason string tells the
        # operator which. None until the node is first heard from.
        "spray_safety_ok": None,
        "spray_safety_reason": None,
        "spray_fsm_state": None,
        "spray_xtrack_error_m": None,
        "spray_gps_fix_ok": None,
        "spray_gps_fix_name": None,
    }

    def __init__(self) -> None:
        super().__init__("fastapi_bridge")
        self._lock = threading.Lock()
        self._state: dict[str, Any] = dict(self._DEFAULT_STATE)
        self._rpp_monitor = RppStatusMonitor()
        # Track last time /mavros/state was received.
        # TRANSIENT_LOCAL means a MAVROS process crash produces no new
        # messages; connected stays "True" forever from cached value.
        # We expose this timestamp so callers can detect true process death.
        self._state_recv_time: float | None = None
        self._pose_recv_time: float | None = None  # last /mavros/local_position/pose
        self._global_pos_recv_time: float | None = None
        self._gps_fix_recv_time: float | None = None
        # Same reasoning applies to the RPP controller: when it dies, its last
        # /rpp/debug message stays in _state forever — still claiming TRACKING with a
        # low self-reported pose_age_ms. Anything that reads those fields to judge
        # health is then reading a dead process's last words. Track when we actually
        # HEARD from it, which is the only fact the corpse cannot fake.
        self._rpp_debug_recv_time: float | None = None
        self._MAVROS_STATE_TIMEOUT_S = 2.0  # MAVROS publishes /state ~10 Hz

        # Callback groups: subs mutually exclusive, services reentrant.
        # /mavros/state gets its OWN group: it is the liveness + mode signal
        # (get_state() flips connected=False after 2 s without it, and the
        # joystick MANUAL-mode check reads it). In the shared group a burst of
        # pose/GPS/rpp callbacks can starve it, which cascades into joystick
        # transport/mode rejections → gateway deadman neutral → jerky drive.
        # Same argument for pose (telemetry + joystick deadman freshness) and
        # /rpp/debug (safety-watchdog rpp_debug_age_ms + rpp_state): isolate
        # them so a GPS/battery/spray burst cannot starve the hot paths.
        # Do NOT change num_threads or touch _svc_group — ReentrantCallbackGroup
        # on service clients is what keeps arm/disarm responsive (safety path).
        self._sub_group = MutuallyExclusiveCallbackGroup()
        self._state_sub_group = MutuallyExclusiveCallbackGroup()
        self._pose_group = MutuallyExclusiveCallbackGroup()   # /mavros/local_position/pose
        self._rpp_group = MutuallyExclusiveCallbackGroup()    # /rpp/debug
        self._svc_group = ReentrantCallbackGroup()

        if not _HAS_MAVROS:
            log.warning("mavros_msgs not available — running without MAVROS topics")

        # ── Subscribers ───────────────────────────────────────────────────────
        if _HAS_MAVROS:
            self.create_subscription(
                State,
                "/mavros/state",
                self._cb_state,
                _qos_reliable_tl(),
                callback_group=self._state_sub_group,
            )
            self.create_subscription(
                PoseStamped,
                "/mavros/local_position/pose",
                self._cb_pose,
                _qos_best_effort(),
                callback_group=self._pose_group,
            )
            self.create_subscription(
                BatteryState,
                "/mavros/battery",
                self._cb_battery,
                _qos_best_effort(),
                callback_group=self._sub_group,
            )
            self.create_subscription(
                NavSatFix,
                "/mavros/global_position/global",
                self._cb_global_pos,
                _qos_best_effort(),
                callback_group=self._sub_group,
            )
            # EKF local-frame origin. mavros publishes this with LatchedStateQoS
            # (RELIABLE + TRANSIENT_LOCAL, depth 1), so we MUST match durability
            # or we silently never receive the one latched message.
            if _HAS_GEOPOINT:
                self.create_subscription(
                    GeoPointStamped,
                    "/mavros/global_position/gp_origin",
                    self._cb_gp_origin,
                    _qos_reliable_tl(),
                    callback_group=self._sub_group,
                )
            if _HAS_GPSRAW:
                self.create_subscription(
                    GPSRAW,
                    "/mavros/gpsstatus/gps1/raw",
                    self._cb_gps_raw,
                    _qos_best_effort(),
                    callback_group=self._sub_group,
                )

        self.create_subscription(
            Float32MultiArray,
            "/rpp/debug",
            self._cb_rpp_debug,
            _qos_best_effort(),
            callback_group=self._rpp_group,
        )
        self.create_subscription(
            Vector3Stamped,
            "/rpp/velocity_ned",
            self._cb_rpp_velocity,
            _qos_best_effort(),
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Bool,
            "/spray/state",
            self._cb_spray_state,
            _qos_best_effort(),
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Bool,
            "/spray/manual_state",
            self._cb_spray_manual_state,
            _qos_best_effort(),
            callback_group=self._sub_group,
        )
        # Rich spray status (mode + mode_state) — the node's own /spray/status
        # (std_msgs/String JSON, best-effort). Mirrored so /api/spray/status can
        # report the live mode and its config without a second ROS hop.
        self.create_subscription(
            String,
            "/spray/status",
            self._cb_spray_status,
            _qos_best_effort(),
            callback_group=self._sub_group,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._path_pub = self.create_publisher(Path, "/path", _qos_reliable_tl())
        # B0 (plan §3): spray mode + mode params, published once per mission
        # load. RELIABLE + TRANSIENT_LOCAL (same class as /path) so a restarted
        # spray node re-latches the current mission's mode. The spray node is
        # the sole parser; we send it a JSON String and never validate on its
        # behalf.
        self._spray_session_config_pub = self.create_publisher(
            String, "/spray/session_config", _qos_reliable_tl()
        )
        # Manual spray override command — reliable VOLATILE (depth 1): must
        # arrive, but a stale override must never replay to a restarted node.
        self._spray_manual_pub = self.create_publisher(Bool, "/spray/manual", 1)
        # G5 — manual point advance (server → RPP). Default QoS is RELIABLE
        # VOLATILE (depth 1), same class as /spray/manual: the operator "next
        # point" command must arrive, but a restart must never replay a stale
        # advance (never TRANSIENT_LOCAL).
        self._point_advance_pub = self.create_publisher(String, "/point/advance", 1)

        # ── Service clients (reentrant group, can be called from any thread) ──
        self._arming_cli = None
        self._set_mode_cli = None
        self._param_get_cli = None
        self._param_set_cli = None
        if _HAS_MAVROS:
            self._arming_cli = self.create_client(
                CommandBool, "/mavros/cmd/arming", callback_group=self._svc_group
            )
            # Used only to ask PX4 to (re)send GPS_GLOBAL_ORIGIN — see
            # _request_ekf_origin_tick. Read-only telemetry request.
            self._command_cli = self.create_client(
                CommandLong, "/mavros/cmd/command", callback_group=self._svc_group
            )
            # PX4 force-sends GPS_GLOBAL_ORIGIN once when its MAVLink stream
            # starts, then only on change. If MAVROS connects after that (a
            # rover-server restart, a MAVROS restart) the latched topic stays
            # EMPTY FOREVER and mission placement silently falls back to the
            # non-deterministic live pose/global pair. Verified on the rig:
            # echo returned nothing until MAV_CMD_REQUEST_MESSAGE was sent, and
            # the value latched immediately afterwards. So ask for it.
            self._origin_req_count = 0
            self.create_timer(
                ORIGIN_REQUEST_PERIOD_S,
                self._request_ekf_origin_tick,
                callback_group=self._svc_group,
            )
            self._set_mode_cli = self.create_client(
                SetMode, "/mavros/set_mode", callback_group=self._svc_group
            )
        if _HAS_PARAM_SRV:
            self._param_get_cli = self.create_client(
                GetParameters,
                "/mavros/param/get_parameters",
                callback_group=self._svc_group,
            )
            self._param_set_cli = self.create_client(
                SetParameters,
                "/mavros/param/set_parameters",
                callback_group=self._svc_group,
            )

        # ── RPP controller param service clients ──────────────────────────────
        # These talk to the running rpp_controller node via standard ROS2
        # rcl_interfaces services. The controller starts independently and may
        # not be up when the bridge starts; readiness is checked on demand.
        self._rpp_param_get_cli: GetParameters.Request | None = None
        self._rpp_param_set_cli: SetParameters.Request | None = None
        self._rpp_param_list_cli: ListParameters.Request | None = None
        if _HAS_PARAM_SRV:
            self._rpp_param_get_cli = self.create_client(
                GetParameters,
                SRV_RPP_GET_PARAMS,
                callback_group=self._svc_group,
            )
            self._rpp_param_set_cli = self.create_client(
                SetParameters,
                SRV_RPP_SET_PARAMS,
                callback_group=self._svc_group,
            )
            self._rpp_param_list_cli = self.create_client(
                ListParameters,
                SRV_RPP_LIST_PARAMS,
                callback_group=self._svc_group,
            )

        # ── Spray controller param service clients ────────────────────────────
        self._spray_param_get_cli = None
        self._spray_param_set_cli = None
        if _HAS_PARAM_SRV:
            self._spray_param_get_cli = self.create_client(
                GetParameters,
                SRV_SPRAY_GET_PARAMS,
                callback_group=self._svc_group,
            )
            self._spray_param_set_cli = self.create_client(
                SetParameters,
                SRV_SPRAY_SET_PARAMS,
                callback_group=self._svc_group,
            )

        # Do not wait for services here: RosBridgeNode is constructed
        # inside FastAPI lifespan, so startup service discovery must not block
        # the asyncio loop. Request paths perform a fail-fast readiness check.
        log.info("RosBridgeNode initialised; service readiness checked fail-fast")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_state(self, msg) -> None:
        now = time.monotonic()
        prev_recv = self._state_recv_time
        self._state_recv_time = now
        with self._lock:
            prev_connected = bool(self._state.get("connected", False))
            self._state["armed"] = msg.armed
            self._state["mode"] = msg.mode
            self._state["connected"] = msg.connected

        # ── Cached-origin invalidation on link re-establishment ──────────────
        # The EKF local-frame origin is per-EKF-session, but it lives in THIS
        # process's state — which survives an FCU reboot and a MAVROS restart.
        # Field bug 2026-07-27: the server kept serving a dead session's origin
        # and placed two missions 2.15 m / 2.25 m off the surveyed line.
        #
        # Two independent signals, OR'd, because either alone can miss:
        #   * connected False->True — MAVROS itself declaring the FCU link came
        #     back (heartbeat timeout), the normal FCU-reboot signature.
        #   * a long gap in /mavros/state — MAVROS restarted or the topic
        #     stalled, which connected cannot report because a TRANSIENT_LOCAL
        #     State message keeps reading connected=True while the process dies.
        # Neither is trusted to be complete: the consistency gate in
        # origin_health is the mechanism-independent backstop. This half is
        # about RECOVERY (re-arm the request path), not detection.
        # The FIRST /mavros/state message of this process is the INITIAL
        # connection, not a re-establishment: `connected` starts False in
        # _DEFAULT_STATE, so every startup produced a spurious False->True and
        # invalidated a perfectly good origin ~50 ms after it arrived (observed
        # live 2026-07-27 16:55:18). It self-healed in ~10 s via the re-request
        # path, but it burned a request attempt and cried wolf on every restart,
        # which trains the operator to ignore the one warning that matters.
        #
        # Skipping it is safe because this half is only RECOVERY. A fresh
        # process holds no cached origin to invalidate; the one real hazard —
        # MAVROS handing us a STALE latched gp_origin from a dead EKF session —
        # is caught by the consistency gate in origin_health, which measures the
        # declared origin against the frame PX4 is actually publishing and does
        # not care how the value got there.
        gap_s = (now - prev_recv) if prev_recv is not None else None
        first_state_msg = prev_recv is None
        if first_state_msg:
            pass
        elif bool(msg.connected) and not prev_connected:
            self._invalidate_ekf_origin("FCU link re-established (connected False->True)")
        elif gap_s is not None and gap_s > ORIGIN_LINK_GAP_S:
            self._invalidate_ekf_origin(
                f"/mavros/state gap of {gap_s:.1f} s (> {ORIGIN_LINK_GAP_S:.1f} s) "
                "— MAVROS restarted or the link stalled"
            )

    def _cb_pose(self, msg) -> None:
        """ENU (MAVROS REP-103) → NED conversion."""
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_enu = math.atan2(siny_cosp, cosy_cosp)
        yaw_ned = math.pi / 2.0 - yaw_enu
        yaw_ned = math.atan2(math.sin(yaw_ned), math.cos(yaw_ned))
        with self._lock:
            self._pose_recv_time = time.monotonic()
            self._state["pos_n"] = msg.pose.position.y  # ENU y = North → pos_n
            self._state["pos_e"] = msg.pose.position.x  # ENU x = East  → pos_e
            self._state["pose_received"] = True
            self._state["heading_ned_deg"] = math.degrees(yaw_ned)

    def _cb_battery(self, msg) -> None:
        pct = msg.percentage
        if pct is not None and 0.0 <= pct <= 1.0:
            pct = pct * 100.0
        with self._lock:
            self._state["battery_v"] = msg.voltage
            self._state["battery_pct"] = pct if pct is not None else 0.0

    def _cb_global_pos(self, msg) -> None:
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        alt = float(msg.altitude)
        # NaN lat/lon is the driver's "no fix yet" sentinel (same convention
        # _cb_gp_origin and tools/analyze_mission.py already guard against).
        # Never let it into state: Socket.IO nulls NaN via _sanitize, but the
        # REST /telemetry/latest path would serialize it as a bare NaN token —
        # invalid JSON that throws in the client's JSON.parse.
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return
        # A14 (2026-07-27): the covariance is only in METRES when the driver
        # says so. NavSatFix.position_covariance_type is
        #   0 UNKNOWN · 1 APPROXIMATED · 2 DIAGONAL_KNOWN · 3 KNOWN
        # and only 2/3 carry metre-valued variances. Type 1 is filled from DOP —
        # a DIMENSIONLESS quality score — so taking sqrt() unconditionally
        # reported an HDOP of 0.50 as "0.707 m" of horizontal error, and a
        # zeroed vertical term as a hard "0.000 m", which reads as PERFECT
        # accuracy when it actually means NO INFORMATION. Observed live on this
        # rover: type 2 sessions give hrms 0.025 / vrms 0.033, type 1 sessions
        # give 0.707 / 0.000 while `gps_fix_name` still says RTK_FIXED.
        #
        # Unknown is now reported as None, never as a number. `hrms`/`vrms` are
        # already Optional[float] in models.TelemetryData, and Socket.IO/REST
        # serialise None as null.
        cov_type = 0
        try:
            cov_type = int(msg.position_covariance_type)
        except (AttributeError, TypeError, ValueError):
            pass
        accuracy_known = cov_type >= 2  # DIAGONAL_KNOWN or KNOWN
        hrms = None
        vrms = None
        if accuracy_known:
            try:
                cov = msg.position_covariance
                hrms = round(math.sqrt(abs(cov[0]) + abs(cov[4])), 3)
                vrms = round(math.sqrt(abs(cov[8])), 3)
            except (ValueError, IndexError, TypeError):
                hrms = vrms = None
            if hrms is not None and not math.isfinite(hrms):
                hrms = None
            if vrms is not None and not math.isfinite(vrms):
                vrms = None

        with self._lock:
            self._global_pos_recv_time = time.monotonic()
            self._state["lat"] = lat
            self._state["lon"] = lon
            if math.isfinite(alt):
                self._state["alt"] = alt
            self._state["hrms"] = hrms
            self._state["vrms"] = vrms
            self._state["gps_accuracy_known"] = accuracy_known
            self._state["position_covariance_type"] = cov_type
            self._state["global_position_received"] = True


    def _cb_gp_origin(self, msg) -> None:
        """EKF-declared local-frame origin (mavros ~/gp_origin, latched).

        This is the datum PX4 itself projects every global coordinate against
        (`vehicle_local_position.ref_lat/ref_lon`). It is set once at first GPS
        fix and then fixed for the EKF session, so using it for mission
        placement makes the published path identical on every load — which a
        live pose/global pair cannot be, because GLOBAL_POSITION_INT quantises
        lat/lon to 1e-7 deg (~1.1 cm here) and the two samples are never
        simultaneous.
        """
        lat = float(msg.position.latitude)
        lon = float(msg.position.longitude)
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return
        if abs(lat) > 90.0 or abs(lon) > 180.0 or (lat == 0.0 and lon == 0.0):
            return
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        with self._lock:
            prev_lat = self._state.get("ekf_origin_lat")
            prev_lon = self._state.get("ekf_origin_lon")
            prev_recv = self._state.get("ekf_origin_received")
            self._state["ekf_origin_lat"] = lat
            self._state["ekf_origin_lon"] = lon
            self._state["ekf_origin_stamp"] = stamp
            self._state["ekf_origin_received"] = True
            # A value has arrived, so whatever caused the last drop is answered.
            # Keep the counter (how many times this process has been through it)
            # as a field-diagnostic breadcrumb.
            self._state["ekf_origin_invalid_reason"] = None
        # Fresh datum in hand: give the request path its full budget back for
        # the NEXT fault episode, instead of burning the process-lifetime cap on
        # the first one. Bounded per episode, not per process.
        self._origin_req_count = 0
        if not prev_recv:
            log.info("EKF local-frame origin received: %.8f, %.8f", lat, lon)
        elif prev_lat != lat or prev_lon != lon:
            # A moved origin invalidates any already-placed mission.
            log.warning(
                "EKF local-frame origin CHANGED from %.8f, %.8f to %.8f, %.8f — "
                "previously placed missions are no longer valid, re-place before "
                "driving", prev_lat, prev_lon, lat, lon)

    def _invalidate_ekf_origin(self, reason: str) -> None:
        """Drop the cached EKF origin and re-arm the GPS_GLOBAL_ORIGIN request.

        Called when something happened that could have started a NEW EKF session
        (FCU reboot, MAVROS restart) or when the cached origin has been measured
        to disagree with the live frame. The coordinates are cleared, not merely
        flagged, so no consumer can read a dead session's datum by forgetting to
        check `ekf_origin_received`.
        """
        with self._lock:
            had = bool(self._state.get("ekf_origin_received"))
            prev = (self._state.get("ekf_origin_lat"),
                    self._state.get("ekf_origin_lon"))
            self._state["ekf_origin_received"] = False
            self._state["ekf_origin_lat"] = 0.0
            self._state["ekf_origin_lon"] = 0.0
            self._state["ekf_origin_stamp"] = None
            self._state["ekf_origin_invalid_reason"] = reason
            self._state["ekf_origin_invalidated_at"] = time.time()
            if had:
                self._state["ekf_origin_invalidations"] = (
                    int(self._state.get("ekf_origin_invalidations", 0)) + 1
                )
        # Re-arm the bounded request path: without this reset the tick would
        # never ask again once the process-lifetime cap had been reached.
        self._origin_req_count = 0
        if had:
            log.warning(
                "EKF local-frame origin INVALIDATED (was %.8f, %.8f): %s — surveyed "
                "placement will REFUSE until a fresh GPS_GLOBAL_ORIGIN arrives; "
                "re-place any staged mission before driving", prev[0], prev[1], reason)

    def _request_ekf_origin_tick(self) -> None:
        """Ask PX4 for GPS_GLOBAL_ORIGIN while we do not have a trustworthy one.

        MAV_CMD_REQUEST_MESSAGE (512) with param1 = 49. This is a pure telemetry
        request — it cannot move the vehicle — but it is still a command to the
        FCU, so it is bounded.

        Two changes from the original "ask until we have any value at all":
          * A measured-INCONSISTENT origin also re-requests. That is the case
            where MAVROS is latching a value from a dead EKF session; asking
            makes PX4 re-send the live one. (If PX4 itself is the one reporting
            the wrong datum, re-asking cannot fix it — placement still refuses,
            which is the point.)
          * The budget resets whenever a trustworthy origin is in hand, so the
            cap is per fault episode rather than per process lifetime. A rover
            left running across several FCU reboots used to exhaust it once and
            then never ask again.
        """
        state = self.get_state()
        if not state.get("connected"):
            return
        health = evaluate_origin_health(state, ORIGIN_CONSISTENCY_MAX_M)
        if health.status == OK:
            self._origin_req_count = 0
            return
        if health.status == UNVERIFIABLE and state.get("ekf_origin_received"):
            # We hold an origin but cannot currently check it (no RTK fix yet,
            # stale pose). Do not spam the FCU on the strength of a maybe.
            return
        if self._origin_req_count >= ORIGIN_REQUEST_MAX_TRIES:
            return
        if self._command_cli is None or not self._command_cli.service_is_ready():
            return
        self._origin_req_count += 1
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 512               # MAV_CMD_REQUEST_MESSAGE
        req.confirmation = 0
        req.param1 = 49.0               # GPS_GLOBAL_ORIGIN
        try:
            fut = self._command_cli.call_async(req)
            fut.add_done_callback(lambda _f: None)
            log.info(
                "requested GPS_GLOBAL_ORIGIN from PX4 (attempt %d/%d) — origin "
                "status is %s", self._origin_req_count, ORIGIN_REQUEST_MAX_TRIES,
                health.status)
        except Exception:
            log.exception("GPS_GLOBAL_ORIGIN request failed")
        if health.status == INCONSISTENT:
            log.warning("origin re-request reason: %s", health.detail)
    def _cb_gps_raw(self, msg) -> None:
        with self._lock:
            self._gps_fix_recv_time = time.monotonic()
            self._state["gps_fix"] = msg.fix_type
            self._state["gps_sat"] = msg.satellites_visible
            self._state["gps_fix_received"] = True

    def _cb_rpp_debug(self, msg: Float32MultiArray) -> None:
        # /rpp/debug is append-only. Consume stable legacy fields first and
        # read newer fields only when present so old bag replays still work.
        if len(msg.data) >= 8:
            data = list(msg.data)
            self._rpp_monitor.update(data)
            with self._lock:
                self._rpp_debug_recv_time = time.monotonic()
                self._state["xtrack_m"] = data[0]
                self._state["heading_err_deg"] = math.degrees(data[1])
                self._state["lookahead_m"] = data[2]
                self._state["speed_m_s"] = data[3]
                self._state["kappa"] = data[4]
                self._state["dist_to_goal_m"] = data[5]
                self._state["pose_age_ms"] = data[6]
                self._state["rpp_state"] = int(data[7])
                # B1 — only populate if the producer is the new version
                self._state["l_d_raw_m"] = data[8] if len(data) >= 9 else float("nan")
                self._state["kappa_speed"] = (
                    data[9] if len(data) >= 10 else float("nan")
                )
                if len(data) >= 40:
                    self._state["spray_active"] = data[39] > 0.5

    def _cb_rpp_velocity(self, msg: Vector3Stamped) -> None:
        with self._lock:
            self._state["v_north"] = msg.vector.x
            self._state["v_east"] = msg.vector.y

    def _cb_spray_state(self, msg: Bool) -> None:
        with self._lock:
            self._state["spraying"] = bool(msg.data)

    def _cb_spray_manual_state(self, msg: Bool) -> None:
        with self._lock:
            self._state["spray_manual"] = bool(msg.data)

    def _cb_spray_status(self, msg: String) -> None:
        """Mirror the node's mode + mode_state from /spray/status (JSON String).

        The node is the sole author; we mirror `mode`/`mode_state` plus the
        safety gate it reports. A malformed payload is ignored (keep
        last-known-good) — never crash the subscription over one bad frame.

        The safety fields exist because a shut valve is ambiguous. `spraying`
        false means "no paint", but not WHY: the geometry may simply be dry, or
        a gate (cross-track, GPS fix, OFFBOARD) may be refusing. `safety_reason`
        is the node's own sentence and is passed through verbatim — it names the
        gate and, for cross-track, whether the limit came from the ROS param or
        a per-mission override, which is what the operator needs to know to act.
        """
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(data, dict):
            return
        with self._lock:
            self._state["spray_mode"] = data.get("mode")
            ms = data.get("mode_state")
            self._state["spray_mode_state"] = ms if isinstance(ms, dict) else {}
            self._state["spray_safety_ok"] = data.get("safety_ok")
            self._state["spray_safety_reason"] = data.get("safety_reason")
            self._state["spray_fsm_state"] = data.get("fsm_state")
            self._state["spray_xtrack_error_m"] = data.get("xtrack_error_m")
            self._state["spray_gps_fix_ok"] = data.get("gps_fix_ok")
            self._state["spray_gps_fix_name"] = data.get("gps_fix_name")

    # ── Public API: spray manual override ────────────────────────────────────

    def publish_spray_manual(self, on: bool) -> None:
        """Command the spray_controller manual override (True=ON, False=cancel)."""
        msg = Bool()
        msg.data = bool(on)
        self._spray_manual_pub.publish(msg)
        log.info("published /spray/manual: %s", "ON" if on else "OFF")

    def publish_point_advance(self, expect_index: int) -> None:
        """G5: command the RPP to advance past the point it is holding (manual).

        `expect_index` is the point the frontend saw as WAIT_OPERATOR; the RPP
        rejects the advance if it is holding a different point (stale double-tap
        guard). JSON matches mission_progress.AdvanceMsg.
        """
        msg = String()
        msg.data = json.dumps({"advance": True, "expect_index": int(expect_index)})
        self._point_advance_pub.publish(msg)
        log.info("published /point/advance: expect_index=%d", int(expect_index))

    def publish_spray_session_config(self, config_json: str) -> None:
        """Publish the spray mode/config JSON string on /spray/session_config (B0).

        `config_json` must already be a valid SpraySessionConfig dict serialized
        to JSON (built by the caller). The spray node is the only parser and
        fails static on anything it does not accept, so we do not validate here.
        Published on every mission load AND on mission clear (a cleared config,
        not silence) — because the topic is TRANSIENT_LOCAL, simply stopping
        would let a restarted node re-latch stale mission geometry (plan §3).
        """
        msg = String()
        msg.data = str(config_json)
        self._spray_session_config_pub.publish(msg)
        log.info("published /spray/session_config (%d bytes)", len(msg.data))

    # ── Public API: state ─────────────────────────────────────────────────────

    def get_state(self) -> dict[str, Any]:
        """Return a shallow copy of current telemetry state (thread-safe).

        The `connected` field is overridden to False if no /mavros/state
        message has been received within MAVROS_STATE_TIMEOUT_S, which
        catches the case where the MAVROS process dies (its last
        TRANSIENT_LOCAL State message stays cached with connected=True,
        but no new messages arrive to reflect the crash).
        """
        with self._lock:
            state = dict(self._state)
            pose_recv_time = self._pose_recv_time
            global_pos_recv_time = self._global_pos_recv_time
            gps_fix_recv_time = self._gps_fix_recv_time
            rpp_debug_recv_time = self._rpp_debug_recv_time
        now = time.monotonic()
        # How long since the RPP controller last spoke. None => never heard from it.
        # This is independent of the controller's OWN self-reported pose_age_ms, which
        # freezes at its last value when the process dies.
        state["rpp_debug_age_ms"] = (
            (now - rpp_debug_recv_time) * 1000.0
            if rpp_debug_recv_time is not None
            else None
        )
        state["local_pose_age_ms"] = (
            (now - pose_recv_time) * 1000.0 if pose_recv_time is not None else None
        )
        state["global_position_age_ms"] = (
            (now - global_pos_recv_time) * 1000.0
            if global_pos_recv_time is not None
            else None
        )
        state["gps_fix_age_ms"] = (
            (now - gps_fix_recv_time) * 1000.0
            if gps_fix_recv_time is not None
            else None
        )
        # Callback receive-time skew (monotonic clocks), not ROS header sync.
        state["pose_global_skew_ms"] = (
            abs(pose_recv_time - global_pos_recv_time) * 1000.0
            if pose_recv_time is not None and global_pos_recv_time is not None
            else None
        )
        # Outside the lock — monotonic check does not need it
        if self._state_recv_time is not None:
            age = time.monotonic() - self._state_recv_time
            if age > self._MAVROS_STATE_TIMEOUT_S:
                state["connected"] = False
        return state

    def get_bridge_snapshot(self) -> dict[str, Any]:
        """Cheap, cached-only health view of the MAVROS bridge link.

        `state_age_ms` (freshness of /mavros/state) is the authoritative
        liveness signal — it goes stale when MAVROS dies or the FCU link
        drops, even while the TRANSIENT_LOCAL cached State still reads
        connected=True. `pose_age_ms` is INFORMATIONAL only: pose can be
        legitimately absent (EKF without a GPS/RTK solution) and must NOT
        be used to declare the bridge frozen.
        """
        with self._lock:
            armed = self._state.get("armed")
            mode = self._state.get("mode")
            connected_cached = self._state.get("connected", False)
        now = time.monotonic()
        state_age_ms = (
            (now - self._state_recv_time) * 1000.0
            if self._state_recv_time is not None
            else None
        )
        pose_age_ms = (
            (now - self._pose_recv_time) * 1000.0
            if self._pose_recv_time is not None
            else None
        )
        # True process-death-aware connected flag (mirror get_state override).
        fcu_connected = bool(connected_cached)
        if state_age_ms is not None and state_age_ms > self._MAVROS_STATE_TIMEOUT_S * 1000.0:
            fcu_connected = False
        try:
            mavros_state_publishers = self.count_publishers("/mavros/state")
        except Exception:
            mavros_state_publishers = -1  # unknown (graph query failed)
        return {
            "fcu_connected": fcu_connected,
            "state_age_ms": state_age_ms,
            "pose_age_ms": pose_age_ms,
            "mavros_state_publishers": mavros_state_publishers,
            "armed": armed,
            "mode": mode,
        }

    def get_origin_health(self) -> dict[str, Any]:
        """Is the cached EKF local-frame origin the frame PX4 is publishing?

        The SAME evaluation surveyed placement enforces, so the operator answer
        and the machine decision can never disagree. Read-only; commands nothing.
        """
        state = self.get_state()
        health = evaluate_origin_health(state, ORIGIN_CONSISTENCY_MAX_M)
        out = health.as_dict()
        out["invalidated_at"] = state.get("ekf_origin_invalidated_at")
        out["invalidations"] = state.get("ekf_origin_invalidations", 0)
        out["origin_stamp"] = state.get("ekf_origin_stamp")
        out["fcu_connected"] = bool(state.get("connected", False))
        return out

    def get_rpp_monitor(self) -> RppStatusMonitor:
        return self._rpp_monitor

    # ── Public API: async service wrappers ────────────────────────────────────

    async def _call_async(
        self,
        cli,
        request,
        timeout: float,
        success_attr: str,
    ) -> tuple[bool, str]:
        """Common async-friendly wrapper for any rclpy service client.

        Returns (ok, message). `ok` reflects future completion AND the
        success flag (`success_attr`) on the response. `message` is empty
        on success or a short diagnostic on failure.
        """
        if cli is None:
            return False, "service client not available"
        if not await self._service_ready_async(cli, timeout_sec=0.5):
            return False, f"service {cli.srv_name} not ready"

        future = cli.call_async(request)

        try:
            result = await self._await_ros_future(future, timeout=timeout)
        except asyncio.TimeoutError:
            return False, f"service {cli.srv_name} timed out after {timeout}s"
        except Exception as exc:
            return False, f"service {cli.srv_name} raised: {exc}"

        if result is None:
            return False, "service returned None"
        flag = getattr(result, success_attr, None)
        if flag is None:
            # No success attr — treat presence of result as success
            return True, ""
        return bool(flag), "" if flag else f"service rejected (success={flag})"

    async def _service_ready_async(self, cli, timeout_sec: float = 0.5) -> bool:
        """Fail-fast service readiness check for command/control paths."""
        if cli is None:
            return False
        return bool(cli.service_is_ready())

    async def _await_ros_future(self, future, timeout: float):
        """Await an rclpy Future from asyncio without late-result races.

        `asyncio.wait_for()` cancels the asyncio-side future on timeout. ROS
        service responses can still arrive later, so the callback must not call
        set_result/set_exception on an already-done Future.
        """
        loop = asyncio.get_running_loop()
        af: asyncio.Future = loop.create_future()

        def _done_cb(f) -> None:
            def _complete_result(result) -> None:
                if not af.done():
                    af.set_result(result)

            def _complete_exception(exc: BaseException) -> None:
                if not af.done():
                    af.set_exception(exc)

            try:
                result = f.result()
            except Exception as exc:
                loop.call_soon_threadsafe(_complete_exception, exc)
                return
            loop.call_soon_threadsafe(_complete_result, result)

        future.add_done_callback(_done_cb)
        try:
            return await asyncio.wait_for(af, timeout=timeout)
        except asyncio.TimeoutError:
            af.cancel()
            raise

    async def arm_async(self, arm: bool, timeout: float = 5.0) -> tuple[bool, str]:
        if self._arming_cli is None:
            return False, "mavros not available"
        req = CommandBool.Request()
        req.value = arm
        return await self._call_async(self._arming_cli, req, timeout, "success")

    async def set_mode_async(self, mode: str, timeout: float = 5.0) -> tuple[bool, str]:
        if self._set_mode_cli is None:
            return False, "mavros not available"
        req = SetMode.Request()
        req.custom_mode = mode
        return await self._call_async(self._set_mode_cli, req, timeout, "mode_sent")

    async def get_param_async(
        self, name: str, timeout: float = 5.0
    ) -> tuple[bool, Any, str]:
        """Returns (ok, value, message). value is None when ok=False."""
        if self._param_get_cli is None:
            return False, None, "param service not available"
        req = GetParameters.Request()
        req.names = [name]
        if not await self._service_ready_async(self._param_get_cli, timeout_sec=0.5):
            return False, None, "param get service not ready"

        try:
            result = await self._await_ros_future(
                self._param_get_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, None, "param get timed out"
        except Exception as exc:
            return False, None, f"param get failed: {exc}"
        if result is None or not result.values:
            return False, None, "param not found"
        return True, _param_value_to_python(result.values[0]), ""

    async def set_param_async(
        self, name: str, value: float | int | bool | str, timeout: float = 5.0
    ) -> tuple[bool, str]:
        if self._param_set_cli is None:
            return False, "param service not available"
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = _python_to_param_value(value)
        req.parameters = [param]

        ok, _, msg = await self._call_set_param(req, timeout)
        return ok, msg

    async def _call_set_param(self, req, timeout: float) -> tuple[bool, list, str]:
        if not await self._service_ready_async(self._param_set_cli, timeout_sec=0.5):
            return False, [], "param set service not ready"
        try:
            result = await self._await_ros_future(
                self._param_set_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, [], "param set timed out"
        except Exception as exc:
            return False, [], f"param set failed: {exc}"
        if result is None:
            return False, [], "param set returned None"
        results = list(result.results)
        if results and not results[0].successful:
            return False, results, results[0].reason or "param set rejected"
        return True, results, ""

    # ── Public API: RPP controller params (via rcl_interfaces) ───────────────

    async def get_rpp_param_async(
        self, name: str, timeout: float = 5.0
    ) -> tuple[bool, Any, str]:
        """Returns (ok, value, message) for a single RPP controller param."""
        if self._rpp_param_get_cli is None:
            return False, None, "RPP param service not available"
        req = GetParameters.Request()
        req.names = [name]
        if not await self._service_ready_async(
            self._rpp_param_get_cli, timeout_sec=0.5
        ):
            return False, None, "RPP controller not running"
        try:
            result = await self._await_ros_future(
                self._rpp_param_get_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, None, "RPP param get timed out"
        except Exception as exc:
            return False, None, f"RPP param get failed: {exc}"
        if result is None or not result.values:
            return False, None, f"param '{name}' not found on RPP controller"
        return True, _param_value_to_python(result.values[0]), ""

    async def get_rpp_params_bulk_async(
        self, names: list[str], timeout: float = 5.0
    ) -> tuple[bool, dict[str, Any], str]:
        """Returns (ok, {name: value, ...}, message) for multiple params."""
        if self._rpp_param_get_cli is None:
            return False, {}, "RPP param service not available"
        req = GetParameters.Request()
        req.names = names
        if not await self._service_ready_async(
            self._rpp_param_get_cli, timeout_sec=0.5
        ):
            return False, {}, "RPP controller not running"
        try:
            result = await self._await_ros_future(
                self._rpp_param_get_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, {}, "RPP param get timed out"
        except Exception as exc:
            return False, {}, f"RPP param get failed: {exc}"
        if result is None:
            return False, {}, "RPP param get returned None"
        if len(result.values) != len(names):
            log.warning(
                "RPP bulk get: expected %d values, got %d — some params missing",
                len(names),
                len(result.values),
            )
        values = {}
        for n, v in zip(names, result.values):
            values[n] = _param_value_to_python(v)
        return True, values, ""

    async def set_rpp_param_async(
        self, name: str, value: float | int | bool | str, timeout: float = 5.0
    ) -> tuple[bool, str]:
        """Set a single RPP controller parameter at runtime."""
        if self._rpp_param_set_cli is None:
            return False, "RPP param service not available"
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = _python_to_param_value(value)
        req.parameters = [param]
        ok, _, msg = await self._call_rpp_set_param(req, timeout)
        return ok, msg

    async def set_rpp_params_bulk_async(
        self, params: dict[str, float | int | bool | str], timeout: float = 5.0
    ) -> tuple[bool, list[bool], str]:
        """Set multiple RPP controller params atomically.

        Returns (ok, per_param_success_flags, message). When one param fails
        the entire batch is rejected by the RPP controller.
        """
        if self._rpp_param_set_cli is None:
            return False, [], "RPP param service not available"
        req = SetParameters.Request()
        for name, value in params.items():
            param = Parameter()
            param.name = name
            param.value = _python_to_param_value(value)
            req.parameters.append(param)
        ok, results, msg = await self._call_rpp_set_param(req, timeout)
        flags = [r.successful for r in results] if results else []
        return ok, flags, msg

    async def _call_rpp_set_param(self, req, timeout: float) -> tuple[bool, list, str]:
        """Shared rcl SetParameters call wrapper for RPP controller."""
        if not await self._service_ready_async(
            self._rpp_param_set_cli, timeout_sec=0.5
        ):
            return False, [], "RPP controller not running"
        try:
            result = await self._await_ros_future(
                self._rpp_param_set_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, [], "RPP param set timed out"
        except Exception as exc:
            return False, [], f"RPP param set failed: {exc}"
        if result is None:
            return False, [], "RPP param set returned None"
        results = list(result.results)
        if results and not results[0].successful:
            return False, results, results[0].reason or "RPP param set rejected"
        return True, results, ""

    # ── Spray controller param access ─────────────────────────────────────────

    async def get_spray_param_async(
        self, name: str, timeout: float = 5.0
    ) -> tuple[bool, Any, str]:
        """Returns (ok, value, message) for a single spray_controller param."""
        if self._spray_param_get_cli is None:
            return False, None, "Spray param service not available"
        req = GetParameters.Request()
        req.names = [name]
        if not await self._service_ready_async(
            self._spray_param_get_cli, timeout_sec=0.5
        ):
            return False, None, "spray_controller not running"
        try:
            result = await self._await_ros_future(
                self._spray_param_get_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, None, "Spray param get timed out"
        except Exception as exc:
            return False, None, f"Spray param get failed: {exc}"
        if result is None or not result.values:
            return False, None, f"param '{name}' not found on spray_controller"
        return True, _param_value_to_python(result.values[0]), ""

    async def get_spray_params_bulk_async(
        self, names: list[str], timeout: float = 5.0
    ) -> tuple[bool, dict[str, Any], str]:
        """Returns (ok, {name: value, ...}, message) for multiple spray params."""
        if self._spray_param_get_cli is None:
            return False, {}, "Spray param service not available"
        req = GetParameters.Request()
        req.names = names
        if not await self._service_ready_async(
            self._spray_param_get_cli, timeout_sec=0.5
        ):
            return False, {}, "spray_controller not running"
        try:
            result = await self._await_ros_future(
                self._spray_param_get_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, {}, "Spray param get timed out"
        except Exception as exc:
            return False, {}, f"Spray param get failed: {exc}"
        if result is None:
            return False, {}, "Spray param get returned None"
        values = {}
        for n, v in zip(names, result.values):
            values[n] = _param_value_to_python(v)
        return True, values, ""

    async def set_spray_param_async(
        self, name: str, value: float | int | bool | str, timeout: float = 5.0
    ) -> tuple[bool, str]:
        """Set a single spray_controller parameter at runtime."""
        if self._spray_param_set_cli is None:
            return False, "Spray param service not available"
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = _python_to_param_value(value)
        req.parameters = [param]
        ok, _, msg = await self._call_spray_set_param(req, timeout)
        return ok, msg

    async def set_spray_params_bulk_async(
        self, params: dict[str, float | int | bool | str], timeout: float = 5.0
    ) -> tuple[bool, list[bool], str]:
        """Set multiple spray_controller params atomically."""
        if self._spray_param_set_cli is None:
            return False, [], "Spray param service not available"
        req = SetParameters.Request()
        for name, value in params.items():
            param = Parameter()
            param.name = name
            param.value = _python_to_param_value(value)
            req.parameters.append(param)
        ok, results, msg = await self._call_spray_set_param(req, timeout)
        flags = [r.successful for r in results] if results else []
        return ok, flags, msg

    async def _call_spray_set_param(self, req, timeout: float) -> tuple[bool, list, str]:
        """Shared rcl SetParameters call wrapper for spray_controller."""
        if not await self._service_ready_async(
            self._spray_param_set_cli, timeout_sec=0.5
        ):
            return False, [], "spray_controller not running"
        try:
            result = await self._await_ros_future(
                self._spray_param_set_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, [], "Spray param set timed out"
        except Exception as exc:
            return False, [], f"Spray param set failed: {exc}"
        if result is None:
            return False, [], "Spray param set returned None"
        results = list(result.results)
        if results and not results[0].successful:
            return False, results, results[0].reason or "Spray param set rejected"
        return True, results, ""

    async def list_rpp_params_async(
        self, timeout: float = 5.0
    ) -> tuple[bool, list[str], str]:
        """List all parameter names on the RPP controller node."""
        if self._rpp_param_list_cli is None:
            return False, [], "RPP param service not available"
        req = ListParameters.Request()
        req.depth = 0  # 0 = unlimited recursion (flat list)
        if not await self._service_ready_async(
            self._rpp_param_list_cli, timeout_sec=0.5
        ):
            return False, [], "RPP controller not running"
        try:
            result = await self._await_ros_future(
                self._rpp_param_list_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, [], "RPP list params timed out"
        except Exception as exc:
            return False, [], f"RPP list params failed: {exc}"
        if result is None:
            return False, [], "RPP list params returned None"
        if result.result is None:
            return False, [], "RPP list params returned null result (service bug)"
        names = list(result.result.names)
        return True, names, ""

    # ── Public API: path publishing ───────────────────────────────────────────

    def publish_path(
        self,
        points: list[tuple[float, float]],
        frame_id: str = "local_ned",
        spray_flags: list[bool] | None = None,
        must_hit_flags: list[bool] | None = None,
    ) -> None:
        """Publish nav_msgs/Path. Empty list → see publish_stop_path().

        `pose.position.z` is a BITFIELD, not a boolean:
            bit 0 (1) = spray ON
            bit 1 (2) = must-hit (source geometry vertex, never simplify away)
        Legacy readers that tested `z > 0.5` will misread a spray-OFF must-hit
        point (z=2) as spray ON — every reader must bit-test `int(round(z)) & 1`.
        """
        if spray_flags is None:
            flags = [False] * len(points)
        elif len(spray_flags) != len(points):
            log.warning(
                "publish_path: spray_flags length %d != points length %d — forcing all OFF",
                len(spray_flags),
                len(points),
            )
            flags = [False] * len(points)
        else:
            flags = [bool(f) for f in spray_flags]

        if must_hit_flags is None:
            mh = [False] * len(points)
        elif len(must_hit_flags) != len(points):
            log.warning(
                "publish_path: must_hit_flags length %d != points length %d — "
                "dropping provenance (simplification falls back to geometry only)",
                len(must_hit_flags),
                len(points),
            )
            mh = [False] * len(points)
        else:
            mh = [bool(f) for f in must_hit_flags]

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        for (n, e), spray, must in zip(points, flags, mh):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(n)
            ps.pose.position.y = float(e)
            ps.pose.position.z = float((1 if spray else 0) | (2 if must else 0))
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._path_pub.publish(path)
        log.info(
            "published path: %d points → %s (spray_on=%d, must_hit=%d)",
            len(points),
            frame_id,
            sum(1 for f in flags if f),
            sum(1 for f in mh if f),
        )

    def publish_stop_path(
        self, frame_id: str = "local_ned"
    ) -> tuple[float, float] | None:
        """Publish a single-point path at the rover's current NED position.

        Workaround for the upstream RPP node that ignores empty-path messages.
        A single-point path is treated as DONE on the first control tick (the
        rover is already within `xy_goal_tolerance` of itself), so RPP zeroes
        its velocity output. This is the safe `mission_stop` semantic.

        Guard: if the server has never received a pose, publishing at origin
        (0,0) could issue an unintended movement command if the rover is not
        actually at the EKF origin. In that case we publish nothing, log a
        warning, and return None. `set_mode_async("MANUAL")` in the abort
        chain still fires, which is the actual safety net.
        """
        s = self.get_state()
        n, e = float(s.get("pos_n", 0.0)), float(s.get("pos_e", 0.0))
        if not s.get("pose_received", False):
            log.warning(
                "publish_stop_path: no pose received yet — "
                "no stop-path published"
            )
            return None
        self.publish_path([(n, e)], frame_id=frame_id)
        log.info("published stop-path at (N=%.2f, E=%.2f)", n, e)
        return (n, e)


# ── Param value <-> Python helpers ────────────────────────────────────────────


def _param_value_to_python(pv) -> Any:
    if not _HAS_PARAM_SRV:
        return None
    t = pv.type
    if t == ParameterType.PARAMETER_BOOL:
        return bool(pv.bool_value)
    if t == ParameterType.PARAMETER_INTEGER:
        return int(pv.integer_value)
    if t == ParameterType.PARAMETER_DOUBLE:
        return float(pv.double_value)
    if t == ParameterType.PARAMETER_STRING:
        return str(pv.string_value)
    return None


def _python_to_param_value(value: Any):
    if not _HAS_PARAM_SRV:
        raise RuntimeError("param services not available")
    pv = ParameterValue()
    if isinstance(value, bool):
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = value
    elif isinstance(value, int):
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = value
    elif isinstance(value, float):
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = value
    elif isinstance(value, str):
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = value
    else:
        raise TypeError(f"Unsupported param value type: {type(value).__name__}")
    return pv
