# Drawing Rover FastAPI Backend — Build Specification

> **Target directory:** `PX4_DXP/server/`
> **Stack:** Python 3.10+, FastAPI, python-socketio, rclpy (ROS2 Humble), uvicorn
> **Runs on:** Jetson Orin (Ubuntu aarch64) alongside existing ROS2 nodes

---

## 1. What We're Building

A FastAPI backend server that bridges a web frontend to the Drawing Rover's RPP (Regulated Pure Pursuit) OFFBOARD pipeline:

```
Frontend (React/Vue)  ←→  FastAPI + Socket.IO  ←→  ROS2 Topics/Services  ←→  MAVROS  ←→  PX4
```

The server runs on the Jetson Orin companion computer (same machine as the ROS2 nodes). It uses **pure rclpy** for ROS2 communication — no roslibpy, no CLI fallbacks.

**This is NOT a port of NRP_ROS.** NRP_ROS is ArduRover AUTO-specific and stays untouched for future Marking Rover migration. This is a lean, OFFBOARD-focused server.

---

## 2. Architecture Diagram

```
┌─────────────┐     Socket.IO / REST      ┌──────────────────────────┐
│  Frontend   │ ◄──────────────────────► │  FastAPI Server           │
│  (React/Vue)│                           │                           │
└─────────────┘                            │  ┌─ OffboardController   │
                                           │  ├─ PathManager           │
                                           │  ├─ RppStatusMonitor      │
                                           │  ├─ MavrosBridge          │
                                           │  ├─ EmergencyHandler      │
                                           │  └─ Beacon                │
                                           │                           │
                                           │  rclpy node (bg thread)   │
                                           └──────────┬───────────────┘
                                                      │ ROS2 topics/services
                                           ┌──────────▼───────────────┐
                                           │  RPP Pipeline Nodes       │
                                           │  (already running)        │
                                           └──────────┬───────────────┘
                                                      │ MAVROS
                                           ┌──────────▼───────────────┐
                                           │  PX4 (CubeOrangePlus)     │
                                           └──────────────────────────┘
```

---

## 3. Existing RPP Pipeline (DO NOT MODIFY)

These 5 ROS2 nodes already exist in `PX4_DXP/src/`. The server interacts with them via topics and services — it does NOT edit them.

### Topic Map (server must interact with)

```
Server PUBLISHES:
  /path                    [nav_msgs/Path, RELIABLE+TRANSIENT_LOCAL]  — inject mission path

Server SUBSCRIBES:
  /rpp/debug               [std_msgs/Float32MultiArray]              — 8-element RPP diagnostics
  /rpp/velocity_ned        [geometry_msgs/Vector3Stamped]            — NED velocity output
  /mavros/state             [mavros_msgs/State]                       — armed, mode, connected
  /mavros/local_position/pose  [geometry_msgs/PoseStamped]            — ENU pose (convert to NED)
  /mavros/battery           [sensor_msgs/BatteryState]               — voltage, percentage
  /mavros/global_position/global [sensor_msgs/NavSatFix]             — lat, lon, alt
  /mavros/gpsstatus/gps1/raw [mavros_msgs/GPSRAW]                    — fix_type, satellites

Server CALLS SERVICES:
  /mavros/cmd/arming        [mavros_msgs/srv/CommandBool]            — arm/disarm
  /mavros/set_mode           [mavros_msgs/srv/SetMode]                — MANUAL / OFFBOARD
  /mavros/param/get_parameters [mavros_msgs/srv/GetParameters]       — read PX4 params
  /mavros/param/set_parameters [mavros_msgs/srv/SetParameters]       — write PX4 params
```

### RPP Debug Array Format (`/rpp/debug`)

```
[0] xtrack_m          — cross-track error in metres
[1] heading_err_rad   — heading error in radians
[2] lookahead_m       — lookahead distance in metres
[3] speed_m_s         — commanded speed in m/s
[4] kappa             — path curvature (1/radius)
[5] dist_to_goal_m    — distance to final waypoint in metres
[6] pose_age_ms       — pose message staleness in milliseconds
[7] state_code        — -1=STALE, 0=IDLE, 1=TRACKING, 2=APPROACH, 3=DONE
```

### ENU to NED Conversion (CRITICAL)

MAVROS publishes poses in ENU frame. The server MUST convert:
```python
import math
yaw_ned_rad = math.pi / 2.0 - yaw_enu_rad  # then normalize to [-pi, pi]
pos_n = pose_enu_y   # ENU y → NED North
pos_e = pose_enu_x   # ENU x → NED East
```

### OFFBOARD Pre-Stream Requirement (CRITICAL)

PX4 REQUIRES setpoints streamed at ≥2 Hz BEFORE switching to OFFBOARD mode, or it rejects the mode switch. The `twist_to_setpoint_node` already handles this (streams zeros when no input). The server must verify streaming is active before attempting OFFBOARD switch by checking `/rpp/debug[7]` is not STALE (-1).

---

## 4. Design Decisions

1. **Pure rclpy** — Server runs on Jetson, same machine as ROS2. Single rclpy node in background thread. No roslibpy.
2. **Server owns OFFBOARD lifecycle** — When frontend is connected, server manages arm/mode/path. The existing `mission_runner_node` is for headless operation only.
3. **Path injection via /path topic** — Server publishes `nav_msgs/Path` to `/path`. Replaces `path_publisher_node` when UI is active.
4. **Emergency stop = switch to MANUAL** — Safest method. PX4 exits OFFBOARD and stops motors. Also publishes empty Path to make RPP go IDLE.
5. **Socket.IO for telemetry (10Hz), REST for commands** — Same pattern as NRP_ROS frontend expects.

---

## 5. File Tree

```
PX4_DXP/server/
├── __init__.py                    # Package marker (empty)
├── main.py                        # FastAPI app factory, lifespan, Socket.IO mount
├── config.py                      # Constants, topic names, defaults
├── models.py                      # Pydantic request/response models
├── ros_node.py                    # Single rclpy node: all subscriptions + service clients
├── offboard_controller.py         # OFFBOARD lifecycle state machine
├── path_manager.py                # Path loading (QGC, CSV, hardcoded) + Path msg building
├── rpp_status.py                  # /rpp/debug decoder + structured state
├── mavros_bridge.py               # MAVROS service calls + telemetry aggregation
├── emergency.py                   # Emergency stop handler
├── beacon.py                      # UDP broadcast for LAN discovery
├── routes/
│   ├── __init__.py                # Package marker (empty)
│   ├── system.py                  # GET /api/ping, GET /api/activity
│   ├── vehicle.py                 # POST /api/arm, POST /api/set_mode, POST /api/estop
│   ├── mission.py                 # POST /api/mission/load, /start, /stop, /abort
│   │                              # GET /api/mission/status
│   ├── path.py                    # GET /api/paths, POST /api/path/upload, /publish
│   ├── params.py                  # GET/POST /api/params
│   └── telemetry.py               # GET /api/telemetry/latest
├── sockets/
│   ├── __init__.py                # Package marker (empty)
│   └── events.py                  # Socket.IO event handlers
├── missions/                      # Uploaded .waypoints and .csv files (gitignored)
│   └── .gitkeep
├── requirements.txt
├── run.sh                         # source /opt/ros/humble/setup.bash && uvicorn
└── ARCHITECTURE.md                # THIS FILE
```

---

## 6. Component Specifications

### 6.1 config.py

```python
import os

# ── ROS2 Topic Names ──
TOPIC_PATH = "/path"
TOPIC_RPP_DEBUG = "/rpp/debug"
TOPIC_RPP_VELOCITY = "/rpp/velocity_ned"
TOPIC_MAVROS_STATE = "/mavros/state"
TOPIC_MAVROS_POSE = "/mavros/local_position/pose"
TOPIC_MAVROS_SETPOINT = "/mavros/setpoint_raw/local"
TOPIC_MAVROS_BATTERY = "/mavros/battery"
TOPIC_MAVROS_GLOBAL_POS = "/mavros/global_position/global"
TOPIC_MAVROS_GPS_RAW = "/mavros/gpsstatus/gps1/raw"

# ── ROS2 Service Names ──
SRV_ARMING = "/mavros/cmd/arming"
SRV_SET_MODE = "/mavros/set_mode"
SRV_GET_PARAMS = "/mavros/param/get_parameters"
SRV_SET_PARAMS = "/mavros/param/set_parameters"

# ── RPP State Codes ──
RPP_STALE = -1
RPP_IDLE = 0
RPP_TRACKING = 1
RPP_APPROACH = 2
RPP_DONE = 3

# ── Server Defaults ──
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5001
TELEMETRY_HZ = 10
MAX_ACTIVITY_LOG = 500
MISSION_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "missions")

# ── QoS Profiles ──
# /path publisher must match path_publisher_node: RELIABLE, TRANSIENT_LOCAL
# /rpp/debug subscriber: BEST_EFFORT, VOLATILE
# /mavros/* subscribers: BEST_EFFORT, VOLATILE (MAVROS default)
```

### 6.2 models.py

```python
from enum import Enum
from pydantic import BaseModel

class VehicleMode(str, Enum):
    MANUAL = "MANUAL"
    OFFBOARD = "OFFBOARD"

class MissionState(str, Enum):
    IDLE = "idle"
    LOADING = "loading"
    ARMING = "arming"
    SWITCHING_OFFBOARD = "switching_offboard"
    RUNNING = "running"
    STOPPING = "stopping"
    DISARMING = "disarming"
    COMPLETED = "completed"
    ABORTED = "aborted"
    ERROR = "error"

class ArmRequest(BaseModel):
    arm: bool  # True = arm, False = disarm

class ModeRequest(BaseModel):
    mode: VehicleMode

class PathPublishRequest(BaseModel):
    name: str | None = None          # hardcoded path name (e.g. "square_2x2")
    file: str | None = None          # uploaded filename in missions/
    frame_id: str = "local_ned"

class MissionStartRequest(BaseModel):
    path_name: str | None = None     # hardcoded path name
    mission_file: str | None = None  # uploaded filename

class TelemetryData(BaseModel):
    # Position (NED metres)
    pos_n: float | None = None
    pos_e: float | None = None
    heading_ned_deg: float | None = None
    # RPP
    xtrack_m: float | None = None
    heading_err_deg: float | None = None
    lookahead_m: float | None = None
    speed_m_s: float | None = None
    kappa: float | None = None
    dist_to_goal_m: float | None = None
    pose_age_ms: float | None = None
    rpp_state: int | None = None      # -1 to 3
    # FCU
    armed: bool | None = None
    mode: str | None = None
    connected: bool | None = None
    # Battery
    battery_v: float | None = None
    battery_pct: float | None = None
    # GPS
    gps_fix: int | None = None
    gps_sat: int | None = None

class PathInfo(BaseModel):
    name: str
    description: str
    num_points: int
    source: str  # "builtin" or "file"

class MissionStatus(BaseModel):
    state: MissionState
    rpp_state: int | None
    rpp_state_name: str | None  # "STALE"/"IDLE"/"TRACKING"/"APPROACH"/"DONE"
    dist_to_goal: float | None
    speed: float | None
    xtrack: float | None

class ActivityEntry(BaseModel):
    timestamp: str
    level: str  # "info"/"warn"/"error"
    message: str

class EstopResponse(BaseModel):
    success: bool
    message: str
```

### 6.3 ros_node.py

Single ROS2 node running in a background daemon thread. All subscriptions and service clients live here. FastAPI routes read from shared state dict and call service methods.

```python
import threading
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import State, BatteryState, GPSRAW
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool, SetMode

class RosBridgeNode(Node):
    """Single rclpy node handling all MAVROS subscriptions and service clients."""

    def __init__(self):
        super().__init__("fastapi_bridge")

        # Thread-safe shared telemetry state
        self._lock = threading.Lock()
        self._state = {
            "armed": False, "mode": "UNKNOWN", "connected": False,
            "pos_n": 0.0, "pos_e": 0.0, "heading_ned_deg": 0.0,
            "battery_v": 0.0, "battery_pct": 0.0,
            "lat": 0.0, "lon": 0.0, "alt": 0.0,
            "gps_fix": 0, "gps_sat": 0,
            "xtrack_m": 0.0, "heading_err_deg": 0.0,
            "lookahead_m": 0.0, "speed_m_s": 0.0,
            "kappa": 0.0, "dist_to_goal_m": 0.0,
            "pose_age_ms": 0.0, "rpp_state": 0,
            "v_north": 0.0, "v_east": 0.0,
        }

        # QoS profiles
        qos_reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST)
        qos_best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE,
                                     history=HistoryPolicy.KEEP_LAST)

        # ── Subscribers ──
        self.create_subscription(State, "/mavros/state", self._cb_state, qos_reliable)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_pose, qos_best_effort)
        self.create_subscription(BatteryState, "/mavros/battery",
                                 self._cb_battery, qos_best_effort)
        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self._cb_global_pos, qos_best_effort)
        self.create_subscription(GPSRAW, "/mavros/gpsstatus/gps1/raw",
                                 self._cb_gps_raw, qos_best_effort)
        self.create_subscription(Float32MultiArray, "/rpp/debug",
                                 self._cb_rpp_debug, qos_best_effort)
        self.create_subscription(Vector3Stamped, "/rpp/velocity_ned",
                                 self._cb_rpp_velocity, qos_best_effort)

        # ── Publishers ──
        self._path_pub = self.create_publisher(Path, "/path", qos_reliable)

        # ── Service Clients ──
        self._arming_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._set_mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        # Wait for services
        self._arming_cli.wait_for_service(timeout_sec=5.0)
        self._set_mode_cli.wait_for_service(timeout_sec=5.0)

    # ── Callbacks update shared state ──
    def _cb_state(self, msg: State):
        with self._lock:
            self._state["armed"] = msg.armed
            self._state["mode"] = msg.mode
            self._state["connected"] = msg.connected

    def _cb_pose(self, msg: PoseStamped):
        # ENU → NED conversion
        q = msg.pose.orientation
        # Extract yaw from quaternion (ENU)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_enu = math.atan2(siny_cosp, cosy_cosp)
        yaw_ned = math.pi / 2.0 - yaw_enu
        # Normalize to [-pi, pi]
        yaw_ned = math.atan2(math.sin(yaw_ned), math.cos(yaw_ned))

        with self._lock:
            self._state["pos_n"] = msg.pose.position.y   # ENU Y → NED North
            self._state["pos_e"] = msg.pose.position.x   # ENU X → NED East
            self._state["heading_ned_deg"] = math.degrees(yaw_ned)

    def _cb_battery(self, msg: BatteryState):
        with self._lock:
            self._state["battery_v"] = msg.voltage
            self._state["battery_pct"] = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage

    def _cb_global_pos(self, msg: NavSatFix):
        with self._lock:
            self._state["lat"] = msg.latitude
            self._state["lon"] = msg.longitude
            self._state["alt"] = msg.altitude

    def _cb_gps_raw(self, msg: GPSRAW):
        with self._lock:
            self._state["gps_fix"] = msg.fix_type
            self._state["gps_sat"] = msg.satellites_visible

    def _cb_rpp_debug(self, msg: Float32MultiArray):
        if len(msg.data) >= 8:
            with self._lock:
                self._state["xtrack_m"] = msg.data[0]
                self._state["heading_err_deg"] = math.degrees(msg.data[1])
                self._state["lookahead_m"] = msg.data[2]
                self._state["speed_m_s"] = msg.data[3]
                self._state["kappa"] = msg.data[4]
                self._state["dist_to_goal_m"] = msg.data[5]
                self._state["pose_age_ms"] = msg.data[6]
                self._state["rpp_state"] = int(msg.data[7])

    def _cb_rpp_velocity(self, msg: Vector3Stamped):
        with self._lock:
            self._state["v_north"] = msg.vector.x
            self._state["v_east"] = msg.vector.y

    # ── Public API ──
    def get_state(self) -> dict:
        with self._lock:
            return dict(self._state)

    def arm(self, arm: bool, timeout: float = 5.0) -> bool:
        req = CommandBool.Request()
        req.value = arm
        future = self._arming_cli.call_async(req)
        rclpy.spin_until_future_done(self, future, timeout_sec=timeout)
        return future.result() is not None and future.result().success

    def set_mode(self, mode: str, timeout: float = 5.0) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self._set_mode_cli.call_async(req)
        rclpy.spin_until_future_done(self, future, timeout_sec=timeout)
        return future.result() is not None and future.result().success

    def publish_path(self, points: list, frame_id: str = "local_ned"):
        """Publish nav_msgs/Path to /path topic."""
        path = Path()
        from geometry_msgs.msg import PoseStamped as PS
        from builtin_interfaces.msg import Time
        import time
        now = time.time()
        path.header.stamp.sec = int(now)
        path.header.stamp.nanosec = int((now - int(now)) * 1e9)
        path.header.frame_id = frame_id
        for n, e in points:
            ps = PS()
            ps.header = path.header
            ps.pose.position.x = float(n)
            ps.pose.position.y = float(e)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._path_pub.publish(path)
```

**Thread model:** rclpy spins in a daemon thread started by main.py lifespan handler. The `RosBridgeNode` is created once and stored as a global. Service calls use `call_async` with `spin_until_future_done` (runs in the rclpy thread context).

### 6.4 main.py

```python
import threading
import rclpy
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import socketio
from routes import system, vehicle, mission, path, params, telemetry
from sockets import events
from config import DEFAULT_HOST, DEFAULT_PORT

# Socket.IO ASGI app
sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")
asgi_app = socketio.ASGIApp(sio)

# Global references (set during lifespan)
ros_node = None
activity_log = []

def create_app() -> FastAPI:
    app = FastAPI(title="Drawing Rover API", version="1.0.0")

    app.add_middleware(CORSMiddleware, allow_origins=["*"],
                       allow_credentials=True, allow_methods=["*"],
                       allow_headers=["*"])

    # Include route routers
    app.include_router(system.router, prefix="/api")
    app.include_router(vehicle.router, prefix="/api")
    app.include_router(mission.router, prefix="/api")
    app.include_router(path.router, prefix="/api")
    app.include_router(params.router, prefix="/api")
    app.include_router(telemetry.router, prefix="/api")

    @app.on_event("startup")
    async def startup():
        global ros_node
        rclpy.init()
        ros_node = RosBridgeNode()
        # Start rclpy spin in daemon thread
        thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
        thread.start()
        # Start telemetry push loop
        # (use asyncio.create_task for periodic sio.emit)

    @app.on_event("shutdown")
    async def shutdown():
        global ros_node
        if ros_node:
            ros_node.destroy_node()
        rclpy.try_shutdown()

    return app

app = create_app()
```

### 6.5 offboard_controller.py

State machine for UI-driven OFFBOARD missions.

```
States: IDLE → ARMING → SWITCHING_OFFBOARD → RUNNING → STOPPING → DISARMING → IDLE
                                                          ↓ (error/estop)
                                                       ABORTED
```

```python
import enum
import time

class OffboardState(enum.Enum):
    IDLE = "idle"
    ARMING = "arming"
    SWITCHING_OFFBOARD = "switching_offboard"
    RUNNING = "running"
    STOPPING = "stopping"
    DISARMING = "disarming"
    COMPLETED = "completed"
    ABORTED = "aborted"
    ERROR = "error"

class OffboardController:
    def __init__(self, ros_node, activity_log):
        self._node = ros_node
        self._log = activity_log
        self._state = OffboardState.IDLE
        self._loaded_path = None  # list of (north, east) tuples
        self._path_name = None

    @property
    def state(self) -> OffboardState:
        return self._state

    def load_path(self, points: list, name: str = None):
        """Load path points for next mission."""
        self._loaded_path = points
        self._path_name = name
        self._log.append({"level": "info", "message": f"Path loaded: {name or 'unknown'}, {len(points)} points"})

    def start(self) -> bool:
        """Execute full OFFBOARD mission sequence."""
        if not self._loaded_path:
            self._state = OffboardState.ERROR
            return False

        # 1. Check FCU connected
        state = self._node.get_state()
        if not state.get("connected", False):
            self._state = OffboardState.ERROR
            return False

        # 2. Arm
        self._state = OffboardState.ARMING
        if not self._node.arm(True):
            self._state = OffboardState.ERROR
            return False

        # 3. Switch to OFFBOARD
        self._state = OffboardState.SWITCHING_OFFBOARD
        # Wait briefly for setpoint stream to be active
        time.sleep(0.5)
        if not self._node.set_mode("OFFBOARD"):
            self._state = OffboardState.ERROR
            self._node.arm(False)  # Disarm on failure
            return False

        # 4. Publish path
        self._node.publish_path(self._loaded_path)
        self._state = OffboardState.RUNNING
        self._log.append({"level": "info", "message": f"Mission started: {self._path_name}"})
        return True

    def stop(self):
        """Stop path following, stay armed."""
        # Publish empty path → RPP goes IDLE → zero velocity
        self._node.publish_path([], frame_id="local_ned")
        self._state = OffboardState.IDLE
        self._log.append({"level": "info", "message": "Mission stopped"})

    def abort(self):
        """Emergency abort: empty path + MANUAL + disarm."""
        self._node.publish_path([], frame_id="local_ned")
        self._node.set_mode("MANUAL")
        self._node.arm(False)
        self._state = OffboardState.ABORTED
        self._log.append({"level": "error", "message": "Mission ABORTED"})

    def disarm(self):
        """Disarm the vehicle."""
        self._node.arm(False)
        self._state = OffboardState.IDLE
```

### 6.6 path_manager.py

Reuses path generation logic from `path_publisher_node.py`.

```python
import os
import csv
import math
from models import PathInfo

# ── Hardcoded path generators (same as path_publisher_node.py) ──

def gen_straight_5m(spacing=0.5):
    return [(i * spacing, 0.0) for i in range(int(5.0 / spacing) + 1)]

def gen_arc_quarter_1m5(radius=1.5, arc_spacing=0.1):
    arc_len = radius * (math.pi / 2.0)
    n = max(2, int(arc_len / arc_spacing) + 1)
    return [(radius * math.sin((math.pi/2) * i/(n-1)),
             radius * (1.0 - math.cos((math.pi/2) * i/(n-1)))) for i in range(n)]

def gen_lshape_2x2(spacing=0.25):
    pts = [(i * spacing, 0.0) for i in range(int(2.0/spacing) + 1)]
    pts += [(2.0, i * spacing) for i in range(1, int(2.0/spacing) + 1)]
    return pts

def gen_square_2x2(spacing=0.25):
    side = 2.0
    pts = [(i * spacing, 0.0) for i in range(int(side/spacing) + 1)]
    pts += [(side, i * spacing) for i in range(1, int(side/spacing) + 1)]
    pts += [(side - i * spacing, side) for i in range(1, int(side/spacing) + 1)]
    pts += [(0.0, side - i * spacing) for i in range(1, int(side/spacing) + 1)]
    return pts

def gen_rectangle_3x2(spacing=0.25):
    pts = [(i * spacing, 0.0) for i in range(int(3.0/spacing) + 1)]
    pts += [(3.0, i * spacing) for i in range(1, int(2.0/spacing) + 1)]
    pts += [(3.0 - i * spacing, 2.0) for i in range(1, int(3.0/spacing) + 1)]
    pts += [(0.0, 2.0 - i * spacing) for i in range(1, int(2.0/spacing) + 1)]
    return pts

def gen_circle_1m5(radius=1.5, arc_spacing=0.1):
    n = max(4, int(radius * 2 * math.pi / arc_spacing) + 1)
    pts = [(radius * math.sin(2*math.pi*i/n),
             radius * (1.0 - math.cos(2*math.pi*i/n))) for i in range(n)]
    pts.append((0.0, 0.0))  # close loop
    return pts

BUILTIN_PATHS = {
    "straight_5m":      {"gen": gen_straight_5m,   "desc": "5m straight north, 50cm spacing"},
    "arc_quarter_1m5":  {"gen": gen_arc_quarter_1m5, "desc": "Quarter circle, R=1.5m, north then east"},
    "lshape_2x2":       {"gen": gen_lshape_2x2,   "desc": "2m north then 2m east, 25cm spacing"},
    "square_2x2":       {"gen": gen_square_2x2,    "desc": "2m x 2m closed square, 25cm spacing"},
    "rectangle_3x2":    {"gen": gen_rectangle_3x2,  "desc": "3m north x 2m east rectangle"},
    "circle_1m5":       {"gen": gen_circle_1m5,    "desc": "Full circle, R=1.5m, closed loop"},
}

def read_qgc_waypoints(filepath: str) -> list:
    """Read QGC WPL 110 .waypoints file. Uses home waypoint as NED origin."""
    from geographiclib.geodesic import Geodesic
    geod = Geodesic.WGS84
    wps, home_lat, home_lon = [], None, None
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("QGC"):
                continue
            fields = line.split("\t")
            if len(fields) < 11:
                continue
            try:
                current = int(fields[1])
                lat, lon = float(fields[8]), float(fields[9])
            except (ValueError, IndexError):
                continue
            if current == 1:
                home_lat, home_lon = lat, lon
            else:
                wps.append((lat, lon))
    if home_lat is None:
        if wps:
            home_lat, home_lon = wps[0]
            wps = wps[1:]
        else:
            raise ValueError(f"No waypoints in {filepath}")
    pts = []
    for lat, lon in wps:
        r = geod.Inverse(home_lat, home_lon, lat, lon)
        dist, bearing = r["s12"], math.radians(r["azi1"])
        pts.append((dist * math.cos(bearing), dist * math.sin(bearing)))
    return pts

def read_ned_csv(filepath: str) -> list:
    """Read simple CSV with north_m,east_m columns."""
    pts = []
    with open(filepath) as f:
        for row in csv.reader(f):
            if not row or row[0].strip().startswith("#"):
                continue
            try:
                n, e = float(row[0].strip()), float(row[1].strip()) if len(row) > 1 else 0.0
                pts.append((n, e))
            except ValueError:
                continue
    return pts

class PathManager:
    def __init__(self, missions_dir: str):
        self._dir = missions_dir
        os.makedirs(missions_dir, exist_ok=True)

    def list_paths(self) -> list[PathInfo]:
        result = []
        for name, info in BUILTIN_PATHS.items():
            pts = info["gen"]()
            result.append(PathInfo(name=name, description=info["desc"],
                                   num_points=len(pts), source="builtin"))
        for fname in sorted(os.listdir(self._dir)):
            fpath = os.path.join(self._dir, fname)
            if not os.path.isfile(fpath):
                continue
            try:
                pts = self._load_file(fpath)
                result.append(PathInfo(name=fname, description=f"Uploaded: {fname}",
                                       num_points=len(pts), source="file"))
            except Exception:
                continue
        return result

    def load_path(self, name: str) -> list[tuple[float, float]]:
        if name in BUILTIN_PATHS:
            return BUILTIN_PATHS[name]["gen"]()
        fpath = os.path.join(self._dir, name)
        if os.path.isfile(fpath):
            return self._load_file(fpath)
        raise FileNotFoundError(f"Path not found: {name}")

    def _load_file(self, fpath: str) -> list:
        ext = os.path.splitext(fpath)[1].lower()
        if ext == ".waypoints":
            return read_qgc_waypoints(fpath)
        elif ext == ".csv":
            return read_ned_csv(fpath)
        else:
            try:
                return read_qgc_waypoints(fpath)
            except Exception:
                return read_ned_csv(fpath)

    def save_uploaded(self, filename: str, content: bytes) -> str:
        fpath = os.path.join(self._dir, filename)
        with open(fpath, "wb") as f:
            f.write(content)
        return filename

    def delete_file(self, filename: str) -> bool:
        fpath = os.path.join(self._dir, filename)
        if os.path.isfile(fpath):
            os.remove(fpath)
            return True
        return False
```

### 6.7 rpp_status.py

```python
import time
from dataclasses import dataclass

RPP_STATE_NAMES = {-1: "STALE", 0: "IDLE", 1: "TRACKING", 2: "APPROACH", 3: "DONE"}

@dataclass
class RppState:
    xtrack_m: float = 0.0
    heading_err_deg: float = 0.0
    lookahead_m: float = 0.0
    speed_m_s: float = 0.0
    kappa: float = 0.0
    dist_to_goal_m: float = 0.0
    pose_age_ms: float = 0.0
    state_code: int = 0
    state_name: str = "IDLE"
    timestamp: float = 0.0

class RppStatusMonitor:
    def __init__(self, done_settle_s: float = 1.0):
        self._done_settle_s = done_settle_s
        self._done_since: float | None = None
        self._state = RppState()

    def update(self, data: list[float]):
        if len(data) >= 8:
            self._state = RppState(
                xtrack_m=data[0], heading_err_deg=math.degrees(data[1]),
                lookahead_m=data[2], speed_m_s=data[3],
                kappa=data[4], dist_to_goal_m=data[5],
                pose_age_ms=data[6], state_code=int(data[7]),
                state_name=RPP_STATE_NAMES.get(int(data[7]), "UNKNOWN"),
                timestamp=time.time()
            )
            if self._state.state_code == 3:  # DONE
                if self._done_since is None:
                    self._done_since = time.time()
            else:
                self._done_since = None

    def get_state(self) -> RppState:
        return self._state

    def is_done(self) -> bool:
        if self._done_since is None:
            return False
        return (time.time() - self._done_since) >= self._done_settle_s

    def is_tracking(self) -> bool:
        return self._state.state_code in (1, 2)
```

### 6.8 emergency.py

```python
class EmergencyHandler:
    def __init__(self, ros_node, offboard_controller, activity_log):
        self._node = ros_node
        self._controller = offboard_controller
        self._log = activity_log

    def estop(self):
        """Emergency stop: empty path + MANUAL mode + disarm."""
        # 1. Publish empty path → RPP goes IDLE → zero velocity
        self._node.publish_path([], frame_id="local_ned")
        # 2. Switch to MANUAL → PX4 exits OFFBOARD, stops motors
        self._node.set_mode("MANUAL")
        # 3. Disarm
        self._node.arm(False)
        # 4. Update state
        self._controller._state = OffboardState.ABORTED
        self._log.append({"level": "error", "message": "EMERGENCY STOP executed"})
```

### 6.9 beacon.py

```python
import socket
import json
import time
import threading

class RoverBeacon:
    def __init__(self, port=5002, interval=2.0, rover_id="drawing_rover_1",
                 server_port=5001):
        self._port = port
        self._interval = interval
        self._payload = json.dumps({
            "rover_id": rover_id,
            "ip": self._get_local_ip(),
            "port": server_port,
            "type": "drawing",
            "version": "1.0.0"
        }).encode()
        self._running = False
        self._thread = None

    def _get_local_ip(self):
        # Get LAN IP for broadcasting
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "192.168.1.102"

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._broadcast_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _broadcast_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        while self._running:
            try:
                sock.sendto(self._payload, ("<broadcast>", self._port))
            except Exception:
                pass
            time.sleep(self._interval)
```

---

## 7. API Routes

### System (`routes/system.py`)
| Method | Path | Handler |
|---|---|---|
| GET | `/api/ping` | `{status: "ok", timestamp: ...}` |
| GET | `/api/activity` | Return last 500 activity log entries |

### Vehicle (`routes/vehicle.py`)
| Method | Path | Handler |
|---|---|---|
| POST | `/api/arm` | Arm/disarm: `{arm: true/false}` |
| POST | `/api/set_mode` | Set mode: `{mode: "OFFBOARD"/"MANUAL"}` |
| POST | `/api/estop` | Emergency stop (empty path + MANUAL + disarm) |

### Mission (`routes/mission.py`)
| Method | Path | Handler |
|---|---|---|
| POST | `/api/mission/load` | Load path by name or file |
| POST | `/api/mission/start` | Execute: arm → OFFBOARD → publish path |
| POST | `/api/mission/stop` | Publish empty path, stay armed |
| POST | `/api/mission/abort` | Emergency abort |
| GET | `/api/mission/status` | Current state + RPP status |

### Path (`routes/path.py`)
| Method | Path | Handler |
|---|---|---|
| GET | `/api/paths` | List built-in + uploaded paths |
| POST | `/api/path/upload` | Upload .waypoints or .csv file |
| POST | `/api/path/publish` | Publish named path to /path topic |
| DELETE | `/api/path/{filename}` | Delete uploaded file |

### Parameters (`routes/params.py`)
| Method | Path | Handler |
|---|---|---|
| GET | `/api/params` | List all PX4 params |
| GET | `/api/params/{name}` | Get single param |
| POST | `/api/params/{name}` | Set param value |

### Telemetry (`routes/telemetry.py`)
| Method | Path | Handler |
|---|---|---|
| GET | `/api/telemetry/latest` | Latest telemetry snapshot JSON |

---

## 8. Socket.IO Events

### Server → Client (10Hz push)
| Event | Payload | Description |
|---|---|---|
| `telemetry` | TelemetryData dict | Position, heading, RPP state, battery, GPS |
| `mission_status` | MissionStatus dict | Mission state, RPP state, goal distance |
| `server_log` | `{level, message, timestamp}` | Activity log entry |
| `rover_disconnected` | `{}` | FCU disconnect detected |

### Client → Server
| Event | Payload | Action |
|---|---|---|
| `arm` | `{arm: true/false}` | Arm/disarm |
| `set_mode` | `{mode: "OFFBOARD"/"MANUAL"}` | Set PX4 mode |
| `emergency_stop` | — | Emergency stop |
| `mission_load` | `{path_name}` or `{mission_file}` | Load path |
| `mission_start` | — | Start OFFBOARD mission |
| `mission_stop` | — | Stop mission |
| `mission_abort` | — | Abort mission |
| `request_params` | `{names: [...]}` | Request param values |

---

## 9. Mission Flow (Frontend-Driven)

```
1. Frontend: POST /api/path/upload → server saves .waypoints to missions/
2. Frontend: POST /api/mission/load {path_name: "square_2x2"} → server loads path
3. Frontend: POST /api/mission/start → OffboardController:
   a. Check FCU connected (mavros/state)
   b. Check twist_to_setpoint streaming (rpp_state != STALE)
   c. Arm vehicle (mavros/cmd/arming)
   d. Switch to OFFBOARD (mavros/set_mode)
   e. Publish path to /path topic
4. Server: emit "telemetry" at 10Hz
5. RPP controller: tracks path, outputs velocity
6. twist_to_setpoint: streams PositionTarget at 50Hz
7. Frontend: POST /api/mission/stop → server publishes empty path → RPP → IDLE
8. Frontend: POST /api/arm {arm: false} → disarm
```

---

## 10. Emergency Stop Flow

```
POST /api/estop → EmergencyHandler:
1. Publish empty Path to /path → RPP → IDLE → zero velocity
2. Call /mavros/set_mode MANUAL → PX4 exits OFFBOARD
3. Call /mavros/cmd/arming {value: false} → disarm
4. Set state = ABORTED
5. Emit "mission_status" {state: "aborted"}
```

---

## 11. What NOT to Build (MVP Scope Cuts)

These NRP_ROS features are explicitly excluded:
1. **Manual control / virtual joystick** — Drawing Rover is autonomous
2. **Obstacle avoidance** — No ultrasonic sensors
3. **GPS failsafe monitor** — Add later
4. **Servo/sprayer control** — Phase 2 feature
5. **LoRa RTK** — NTRIP handled by ntrip_rtcm_node.py systemd service
6. **LED controller** — No hardware
7. **TTS** — Not needed
8. **Mission mode (auto/manual/continuous/dash)** — Only OFFBOARD and MANUAL
9. **Waypoint upload to FCU** — Path published to /path topic, not uploaded to PX4

---

## 12. Implementation Order

### Phase A: Core (build standalone, test with rclpy mock)
1. `config.py`
2. `models.py`
3. `ros_node.py` (rclpy background node)
4. `mavros_bridge.py` (service calls + telemetry read)

### Phase B: Server framework
5. `main.py` (FastAPI app, lifespan, Socket.IO mount)
6. `routes/system.py`
7. `routes/vehicle.py`
8. `emergency.py`

### Phase C: Mission control
9. `path_manager.py`
10. `rpp_status.py`
11. `offboard_controller.py`
12. `routes/mission.py`
13. `routes/path.py`

### Phase D: Telemetry & discovery
14. `routes/telemetry.py`
15. `sockets/events.py`
16. `beacon.py`
17. `routes/params.py`

### Phase E: Polish
18. `run.sh`
19. `requirements.txt`
20. Integration testing

---

## 13. Verification Plan

1. **Unit test path_manager:** Load each hardcoded path, verify point count
2. **Unit test offboard_controller:** State transitions with mocked services
3. **SITL integration test:** PX4 SITL + MAVROS + RPP pipeline + server:
   - `/api/ping` → 200
   - `/api/paths` → 6 built-in paths
   - `/api/arm` → vehicle arms
   - `/api/set_mode` OFFBOARD → mode switch
   - `/api/path/publish` → path published to /path
   - Socket.IO `telemetry` streaming at 10Hz
   - `/api/estop` → MANUAL + disarm
4. **Hardware test:** Deploy to Jetson, full mission cycle

---

## 14. Requirements (`requirements.txt`)

```
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
python-socketio>=5.10.0
websockets>=12.0
pydantic>=2.5.0
geographiclib>=2.0
```

Note: `rclpy` and `mavros_msgs` are provided by ROS2 Humble installation — not in pip requirements.

---

## 15. Startup Script (`run.sh`)

```bash
#!/bin/bash
# Source ROS2 Humble
source /opt/ros/humble/setup.bash
# Source workspace if needed
# source ~/PX4_DXP/install/setup.bash

# Set environment
export ROS_DOMAIN_ID=0
export FASTAPI_PORT=${FASTAPI_PORT:-5001}

# Run server
cd "$(dirname "$0")"
exec python3 -m uvicorn main:app --host 0.0.0.0 --port $FASTAPI_PORT
```