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
import math
import threading
import time
from typing import Any, Callable

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray

from config import SRV_RPP_GET_PARAMS, SRV_RPP_LIST_PARAMS, SRV_RPP_SET_PARAMS
from logging_setup import get_logger
from rpp_status import RppStatusMonitor

log = get_logger("server.ros")

# ── Optional MAVROS imports ───────────────────────────────────────────────────
try:
    from mavros_msgs.msg import State
    from sensor_msgs.msg import BatteryState, NavSatFix
    from mavros_msgs.srv import CommandBool, SetMode

    _HAS_MAVROS = True
except ImportError:
    _HAS_MAVROS = False
    State = BatteryState = NavSatFix = CommandBool = SetMode = None  # type: ignore

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
        "heading_ned_deg": 0.0,
        "battery_v": 0.0,
        "battery_pct": 0.0,
        "lat": 0.0,
        "lon": 0.0,
        "alt": 0.0,
        "gps_fix": 0,
        "gps_sat": 0,
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
        self._MAVROS_STATE_TIMEOUT_S = 2.0  # MAVROS publishes /state ~10 Hz

        # Callback groups: subs mutually exclusive, services reentrant
        self._sub_group = MutuallyExclusiveCallbackGroup()
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
                callback_group=self._sub_group,
            )
            self.create_subscription(
                PoseStamped,
                "/mavros/local_position/pose",
                self._cb_pose,
                _qos_best_effort(),
                callback_group=self._sub_group,
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
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Vector3Stamped,
            "/rpp/velocity_ned",
            self._cb_rpp_velocity,
            _qos_best_effort(),
            callback_group=self._sub_group,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._path_pub = self.create_publisher(Path, "/path", _qos_reliable_tl())

        # ── Service clients (reentrant group, can be called from any thread) ──
        self._arming_cli = None
        self._set_mode_cli = None
        self._param_get_cli = None
        self._param_set_cli = None
        if _HAS_MAVROS:
            self._arming_cli = self.create_client(
                CommandBool, "/mavros/cmd/arming", callback_group=self._svc_group
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
        # not be up when the bridge starts; wait_for_service is deferred to
        # each call site.
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

        # Non-blocking startup wait — services may come up after us
        for cli, name in (
            (self._arming_cli, "/mavros/cmd/arming"),
            (self._set_mode_cli, "/mavros/set_mode"),
            (self._param_get_cli, "/mavros/param/get_parameters"),
            (self._param_set_cli, "/mavros/param/set_parameters"),
            (self._rpp_param_get_cli, SRV_RPP_GET_PARAMS),
            (self._rpp_param_set_cli, SRV_RPP_SET_PARAMS),
            (self._rpp_param_list_cli, SRV_RPP_LIST_PARAMS),
        ):
            if cli is not None and not cli.wait_for_service(timeout_sec=2.0):
                log.warning("service %s not yet available — will retry on demand", name)

        log.info("RosBridgeNode initialised")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_state(self, msg) -> None:
        self._state_recv_time = time.monotonic()
        with self._lock:
            self._state["armed"] = msg.armed
            self._state["mode"] = msg.mode
            self._state["connected"] = msg.connected

    def _cb_pose(self, msg) -> None:
        """ENU (MAVROS REP-103) → NED conversion."""
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_enu = math.atan2(siny_cosp, cosy_cosp)
        yaw_ned = math.pi / 2.0 - yaw_enu
        yaw_ned = math.atan2(math.sin(yaw_ned), math.cos(yaw_ned))
        with self._lock:
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
        with self._lock:
            self._state["lat"] = msg.latitude
            self._state["lon"] = msg.longitude
            self._state["alt"] = msg.altitude

    def _cb_gps_raw(self, msg) -> None:
        with self._lock:
            self._state["gps_fix"] = msg.fix_type
            self._state["gps_sat"] = msg.satellites_visible

    def _cb_rpp_debug(self, msg: Float32MultiArray) -> None:
        # B1: layout is 10 fields; fall back gracefully to legacy 8-field
        # producers (old replays). New fields default to NaN when absent.
        if len(msg.data) >= 8:
            data = list(msg.data)
            self._rpp_monitor.update(data)
            with self._lock:
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

    def _cb_rpp_velocity(self, msg: Vector3Stamped) -> None:
        with self._lock:
            self._state["v_north"] = msg.vector.x
            self._state["v_east"] = msg.vector.y

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
        # Outside the lock — monotonic check does not need it
        if self._state_recv_time is not None:
            age = time.monotonic() - self._state_recv_time
            if age > self._MAVROS_STATE_TIMEOUT_S:
                state["connected"] = False
        return state

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
        if not cli.service_is_ready():
            # Try a brief re-check (services can come up late)
            if not cli.wait_for_service(timeout_sec=0.5):
                return False, f"service {cli.srv_name} not ready"

        future = cli.call_async(request)
        loop = asyncio.get_running_loop()
        af: asyncio.Future = loop.create_future()

        def _done_cb(f) -> None:
            try:
                result = f.result()
            except Exception as exc:
                loop.call_soon_threadsafe(af.set_exception, exc)
                return
            loop.call_soon_threadsafe(af.set_result, result)

        future.add_done_callback(_done_cb)

        try:
            result = await asyncio.wait_for(af, timeout=timeout)
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
        if not self._param_get_cli.service_is_ready():
            if not self._param_get_cli.wait_for_service(timeout_sec=0.5):
                return False, None, "param get service not ready"

        future = self._param_get_cli.call_async(req)
        loop = asyncio.get_running_loop()
        af: asyncio.Future = loop.create_future()

        def _done_cb(f) -> None:
            try:
                result = f.result()
            except Exception as exc:
                loop.call_soon_threadsafe(af.set_exception, exc)
                return
            loop.call_soon_threadsafe(af.set_result, result)

        future.add_done_callback(_done_cb)
        try:
            result = await asyncio.wait_for(af, timeout=timeout)
        except asyncio.TimeoutError:
            return False, None, "param get timed out"
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
        if not self._param_set_cli.service_is_ready():
            if not self._param_set_cli.wait_for_service(timeout_sec=0.5):
                return False, [], "param set service not ready"
        future = self._param_set_cli.call_async(req)
        loop = asyncio.get_running_loop()
        af: asyncio.Future = loop.create_future()

        def _done_cb(f) -> None:
            try:
                result = f.result()
            except Exception as exc:
                loop.call_soon_threadsafe(af.set_exception, exc)
                return
            loop.call_soon_threadsafe(af.set_result, result)

        future.add_done_callback(_done_cb)
        try:
            result = await asyncio.wait_for(af, timeout=timeout)
        except asyncio.TimeoutError:
            return False, [], "param set timed out"
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
        if not self._rpp_param_get_cli.service_is_ready():
            if not self._rpp_param_get_cli.wait_for_service(timeout_sec=0.5):
                return False, None, "RPP controller not running"
        future = self._rpp_param_get_cli.call_async(req)
        loop = asyncio.get_running_loop()
        af: asyncio.Future = loop.create_future()

        def _done_cb(f) -> None:
            try:
                loop.call_soon_threadsafe(af.set_result, f.result())
            except Exception as exc:
                loop.call_soon_threadsafe(af.set_exception, exc)

        future.add_done_callback(_done_cb)
        try:
            result = await asyncio.wait_for(af, timeout=timeout)
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
        if not self._rpp_param_get_cli.service_is_ready():
            if not self._rpp_param_get_cli.wait_for_service(timeout_sec=0.5):
                return False, {}, "RPP controller not running"
        future = self._rpp_param_get_cli.call_async(req)
        loop = asyncio.get_running_loop()
        af: asyncio.Future = loop.create_future()

        def _done_cb(f) -> None:
            try:
                loop.call_soon_threadsafe(af.set_result, f.result())
            except Exception as exc:
                loop.call_soon_threadsafe(af.set_exception, exc)

        future.add_done_callback(_done_cb)
        try:
            result = await asyncio.wait_for(af, timeout=timeout)
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
        if not self._rpp_param_set_cli.service_is_ready():
            if not self._rpp_param_set_cli.wait_for_service(timeout_sec=0.5):
                return False, [], "RPP controller not running"
        future = self._rpp_param_set_cli.call_async(req)
        loop = asyncio.get_running_loop()
        af: asyncio.Future = loop.create_future()

        def _done_cb(f) -> None:
            try:
                loop.call_soon_threadsafe(af.set_result, f.result())
            except Exception as exc:
                loop.call_soon_threadsafe(af.set_exception, exc)

        future.add_done_callback(_done_cb)
        try:
            result = await asyncio.wait_for(af, timeout=timeout)
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

    async def list_rpp_params_async(
        self, timeout: float = 5.0
    ) -> tuple[bool, list[str], str]:
        """List all parameter names on the RPP controller node."""
        if self._rpp_param_list_cli is None:
            return False, [], "RPP param service not available"
        req = ListParameters.Request()
        req.depth = 0  # 0 = unlimited recursion (flat list)
        if not self._rpp_param_list_cli.service_is_ready():
            if not self._rpp_param_list_cli.wait_for_service(timeout_sec=0.5):
                return False, [], "RPP controller not running"
        future = self._rpp_param_list_cli.call_async(req)
        loop = asyncio.get_running_loop()
        af: asyncio.Future = loop.create_future()

        def _done_cb(f) -> None:
            try:
                loop.call_soon_threadsafe(af.set_result, f.result())
            except Exception as exc:
                loop.call_soon_threadsafe(af.set_exception, exc)

        future.add_done_callback(_done_cb)
        try:
            result = await asyncio.wait_for(af, timeout=timeout)
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
        self, points: list[tuple[float, float]], frame_id: str = "local_ned"
    ) -> None:
        """Publish nav_msgs/Path. Empty list → see publish_stop_path()."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        for n, e in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(n)
            ps.pose.position.y = float(e)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._path_pub.publish(path)
        log.info("published path: %d points → %s", len(points), frame_id)

    def publish_stop_path(self, frame_id: str = "local_ned") -> None:
        """Publish a single-point path at the rover's current NED position.

        Workaround for the upstream RPP node that ignores empty-path messages.
        A single-point path is treated as DONE on the first control tick (the
        rover is already within `xy_goal_tolerance` of itself), so RPP zeroes
        its velocity output. This is the safe `mission_stop` semantic.

        Guard: if the server has never received a pose (pos_n=0.0, pos_e=0.0
        and connected=False), publishing at origin (0,0) could issue an
        unintended movement command if the rover is not actually at the EKF
        origin. In that case we fall back to publishing an empty path and
        log a warning. RPP ignores the empty path (early-return), but
        `set_mode_async("MANUAL")` in the abort chain still fires, which is
        the actual safety net.
        """
        s = self.get_state()
        n, e = float(s.get("pos_n", 0.0)), float(s.get("pos_e", 0.0))
        pose_received = not (n == 0.0 and e == 0.0 and not s.get("connected", False))
        if not pose_received:
            log.warning(
                "publish_stop_path: no pose received yet — "
                "publishing empty path (RPP ignores it; MANUAL switch is the fallback)"
            )
            self.publish_path([], frame_id=frame_id)
            return
        self.publish_path([(n, e)], frame_id=frame_id)
        log.info("published stop-path at (N=%.2f, E=%.2f)", n, e)


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
