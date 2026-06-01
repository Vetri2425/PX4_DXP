"""Central configuration: topic names, service names, constants."""

from __future__ import annotations

import os

# ── ROS2 Topic Names ──────────────────────────────────────────────────────────
TOPIC_PATH = "/path"
TOPIC_RPP_DEBUG = "/rpp/debug"
TOPIC_RPP_VELOCITY = "/rpp/velocity_ned"
TOPIC_MAVROS_STATE = "/mavros/state"
TOPIC_MAVROS_POSE = "/mavros/local_position/pose"
TOPIC_MAVROS_SETPOINT = "/mavros/setpoint_raw/local"
TOPIC_MAVROS_BATTERY = "/mavros/battery"
TOPIC_MAVROS_GLOBAL_POS = "/mavros/global_position/global"
TOPIC_MAVROS_GPS_RAW = "/mavros/gpsstatus/gps1/raw"

# ── ROS2 Service Names ────────────────────────────────────────────────────────
SRV_ARMING = "/mavros/cmd/arming"
SRV_SET_MODE = "/mavros/set_mode"
SRV_GET_PARAMS = "/mavros/param/get_parameters"
SRV_SET_PARAMS = "/mavros/param/set_parameters"

# ── RPP Controller Parameter Services ──────────────────────────────────────────
RPP_NODE_NAME = "rpp_controller"
SRV_RPP_GET_PARAMS = f"/{RPP_NODE_NAME}/get_parameters"
SRV_RPP_SET_PARAMS = f"/{RPP_NODE_NAME}/set_parameters"
SRV_RPP_LIST_PARAMS = f"/{RPP_NODE_NAME}/list_parameters"

# ── RPP State Codes ───────────────────────────────────────────────────────────
RPP_STALE = -1
RPP_IDLE = 0
RPP_TRACKING = 1
RPP_APPROACH = 2
RPP_DONE = 3
RPP_RTK_WAIT = 4  # B2: GPS fix < RTK_FIXED; controller refusing to drive
RPP_JUMP_SKIP = 5  # B2: one-cycle position-jump skip (EKF reset / RTK lock-on)

RPP_STATE_NAMES = {
    RPP_STALE: "STALE",
    RPP_IDLE: "IDLE",
    RPP_TRACKING: "TRACKING",
    RPP_APPROACH: "APPROACH",
    RPP_DONE: "DONE",
    RPP_RTK_WAIT: "RTK_WAIT",
    RPP_JUMP_SKIP: "JUMP_SKIP",
}

# B2: codes that mean "controller is not driving safely". Treat the same as
# STALE for safety-abort and OFFBOARD-start guard purposes. Centralised here
# so server/main.py and server/offboard_controller.py stay in sync.
RPP_UNHEALTHY_CODES = {RPP_STALE, RPP_RTK_WAIT, RPP_JUMP_SKIP}

# ── Server Defaults ───────────────────────────────────────────────────────────
DEFAULT_HOST = "0.0.0.0"  # overridden below when ROVER_DISABLE_AUTH is set
DEFAULT_PORT = int(os.environ.get("FASTAPI_PORT", "5001"))
TELEMETRY_HZ = 10  # Socket.IO push rate
MAX_ACTIVITY_LOG = 500
BEACON_PORT = 5002
BEACON_INTERVAL = 2.0
ROVER_ID = "drawing_rover_1"

MISSION_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "missions")

# ── Safety / watchdog thresholds ──────────────────────────────────────────────
POSE_STALE_MS = 500.0  # consider pose stale above this
SAFETY_STALE_GRACE_S = 1.0  # auto-abort after this long in STALE
DONE_SETTLE_S = 1.0  # require this much DONE before auto-completing
SETPOINT_STREAM_GRACE_S = 0.5  # delay between OFFBOARD switch and path publish

# ── Auth ──────────────────────────────────────────────────────────────────────
TOKEN_FILE_DEFAULT = os.path.expanduser("~/.rover_token")
TOKEN_HEADER_NAME = "X-Rover-Token"

# ── File upload limits ────────────────────────────────────────────────────────
ALLOWED_UPLOAD_EXTENSIONS = {".waypoints", ".csv", ".dxf"}
MAX_UPLOAD_BYTES = 5 * 1024 * 1024  # 5 MiB (DXF files can be large)

# ── CORS ──────────────────────────────────────────────────────────────────────
if os.environ.get("ROVER_DISABLE_AUTH"):
    CORS_ALLOW_ORIGINS = ["*"]
    DEFAULT_HOST = "127.0.0.1"
else:
    CORS_ALLOW_ORIGINS = ["*"]
    DEFAULT_HOST = "0.0.0.0"
CORS_ALLOW_CREDENTIALS = False
