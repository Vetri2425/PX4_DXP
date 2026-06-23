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

# ── Spray Controller Parameter Services ────────────────────────────────────────
SPRAY_NODE_NAME = "spray_controller"
SRV_SPRAY_GET_PARAMS = f"/{SPRAY_NODE_NAME}/get_parameters"
SRV_SPRAY_SET_PARAMS = f"/{SPRAY_NODE_NAME}/set_parameters"
SRV_SPRAY_APPLY_MISSION_CONFIG = "/spray/apply_mission_config"
SRV_SPRAY_START_DWELL = "/spray/start_dwell"
SRV_SPRAY_CANCEL_DWELL = "/spray/cancel_dwell"

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

# GPS Fix Type Names (from MAVROS sensor_msgs/NavSatStatus.msg fix_type)
GPS_FIX_NAMES = {
    0: "NO_FIX",
    1: "GPS",
    2: "DGPS",
    4: "DGPS",  # duplicate for compatibility
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
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
# Aligned-DXF missions are staged here before the operator confirms a load.
STAGING_DIR = os.path.join(MISSION_DIR, "staging")

# Mission-scoped debug capture coordination. Field deployments set capture
# required so an API mission cannot publish /path until rosbag is ready. The
# code default is OFF (fail-open) so developer installs and the test suite never
# hard-depend on the recorder; rover-server.service sets
# MISSION_CAPTURE_REQUIRED=1 to enforce fail-closed capture in the field.
MISSION_DEBUG_CONTROL_DIR = os.environ.get(
    "MISSION_DEBUG_CONTROL_DIR",
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                 "runtime", "mission-debug"),
)
MISSION_CAPTURE_REQUIRED = os.environ.get("MISSION_CAPTURE_REQUIRED", "0") == "1"
MISSION_CAPTURE_ACK_TIMEOUT_S = float(
    os.environ.get("MISSION_CAPTURE_ACK_TIMEOUT_S", "3.0")
)

# ── DXF alignment / mission-handoff ───────────────────────────────────────────
# Max allowable least-squares RMSE (metres) for multi-point DXF→NED alignment.
# Plans whose residual exceeds this are rejected (422) and never staged.
RMSE_MAX = float(os.environ.get("ROVER_ALIGN_RMSE_MAX", "0.05"))
# Max allowable deviation of the least-squares scale from unity for multi-point
# DXF→NED alignment. Ref points and segment geometry share a metric frame, so a
# healthy fit lands scale≈1.0; a large deviation means a unit/frame mismatch
# (e.g. double-scaled cm points → scale≈100). A 2-point fit is exactly determined
# so its RMSE is ~0 and cannot catch this — the scale gate is the only defense.
# Rejected (422) and never staged. 0.25 → accept scale in [0.75, 1.25].
SCALE_FIT_TOLERANCE = float(os.environ.get("ROVER_ALIGN_SCALE_TOL", "0.25"))
# Staged-mission lifetime (seconds). Older staging files are pruned on each plan.
STAGING_TTL_S = float(os.environ.get("ROVER_STAGING_TTL_S", "3600"))
# Litres of marking material consumed per metre of MARK path (site-tunable).
SPRAY_LITERS_PER_METER = float(os.environ.get("ROVER_SPRAY_L_PER_M", "0.012"))
# Default MARK flags for built-in / legacy non-DXF paths that carry no spray metadata.
SPRAY_DEFAULT_ON = os.environ.get("ROVER_SPRAY_DEFAULT_ON", "1") == "1"

# ── Safety / watchdog thresholds ──────────────────────────────────────────────
POSE_STALE_MS = 500.0  # consider pose stale above this
GLOBAL_POSITION_STALE_MS = float(os.environ.get("ROVER_GLOBAL_POS_STALE_MS", "500"))
GPS_FIX_STALE_MS = float(os.environ.get("ROVER_GPS_FIX_STALE_MS", "500"))
POSE_GLOBAL_MAX_SKEW_MS = float(os.environ.get("ROVER_POSE_GPS_MAX_SKEW_MS", "100"))
SAFETY_STALE_GRACE_S = 1.0  # auto-abort after this long in STALE
DONE_SETTLE_S = 1.0  # require this much DONE before auto-completing
SETPOINT_STREAM_GRACE_S = 0.5  # path/setpoint settle time before OFFBOARD request

# ── Bridge health watchdog (Phase 3) ──────────────────────────────────────────
BRIDGE_HEALTH_POLL_S = 1.0          # how often BridgeHealthManager checks
BRIDGE_STATE_STALE_MS = 2500.0      # /mavros/state older than this => link frozen
BRIDGE_FROZEN_GRACE_S = 6.0         # sustained-frozen duration before recovery
BRIDGE_RECOVERY_MAX = 3             # max auto-recoveries within the window
BRIDGE_RECOVERY_WINDOW_S = 300.0    # backoff window (5 min)
BRIDGE_RECOVERY_COOLDOWN_S = 30.0   # suppress detection after a recovery (MAVROS comes back)
# Phase 3A = observe-only by default. Flip to "1" (env) to enable auto-restart
# of px4-dxp (Phase 3B) only after detection is validated in the field.
BRIDGE_AUTO_RECOVER = os.environ.get("ROVER_BRIDGE_AUTO_RECOVER", "0") == "1"

# ── LoRa RTK transport (Task_03) ─────────────────────────────────────────────
LORA_NO_DATA_WARN_S = float(os.environ.get("LORA_NO_DATA_WARN_S", "15.0"))
LORA_NO_DATA_FAIL_S = float(os.environ.get("LORA_NO_DATA_FAIL_S", "60.0"))
LORA_RECONNECT_INTERVAL_S = float(os.environ.get("LORA_RECONNECT_INTERVAL_S", "5.0"))
LORA_MAX_RESTARTS_PER_MIN = int(os.environ.get("LORA_MAX_RESTARTS_PER_MIN", "5"))
LORA_MODULE_DISCONNECT_TIMEOUT_S = float(
    os.environ.get("LORA_MODULE_DISCONNECT_TIMEOUT_S", "120.0")
)
LORA_MAX_FRAME_SIZE = int(os.environ.get("LORA_MAX_FRAME_SIZE", "1029"))
LORA_MAX_BYTES_PER_SEC = float(os.environ.get("LORA_MAX_BYTES_PER_SEC", "65536"))
LORA_MAX_FRAMES_PER_SEC = float(os.environ.get("LORA_MAX_FRAMES_PER_SEC", "50"))
LORA_ALLOWED_MESSAGE_TYPES = os.environ.get("LORA_ALLOWED_MESSAGE_TYPES", "").strip()

# ── Auth ──────────────────────────────────────────────────────────────────────
TOKEN_FILE_DEFAULT = os.environ.get(
    "ROVER_TOKEN_FILE",
    os.path.expanduser("~/.rover_token"),
)
TOKEN_HEADER_NAME = "X-Rover-Token"

# ── File upload limits ────────────────────────────────────────────────────────
ALLOWED_UPLOAD_EXTENSIONS = {".waypoints", ".csv", ".dxf"}
MAX_UPLOAD_BYTES = 5 * 1024 * 1024  # 5 MiB (DXF files can be large)

# ── CORS ──────────────────────────────────────────────────────────────────────
if os.environ.get("ROVER_DISABLE_AUTH"):
    CORS_ALLOW_ORIGINS = [
        "http://localhost:3000", "http://127.0.0.1:3000",
        "http://localhost:5001", "http://127.0.0.1:5001",
    ]
    DEFAULT_HOST = "127.0.0.1"
else:
    CORS_ALLOW_ORIGINS = ["*"]
    DEFAULT_HOST = "0.0.0.0"

# Explicit override (deployment-specific, set via systemd drop-in). Comma-
# separated list of allowed origins, or "*" for any. Lets a trusted/isolated
# LAN serve the browser/mobile frontend (whose origin is an arbitrary LAN IP)
# even with auth disabled, without baking an open policy into the repo.
_cors_env = os.environ.get("ROVER_CORS_ORIGINS")
if _cors_env:
    CORS_ALLOW_ORIGINS = [o.strip() for o in _cors_env.split(",") if o.strip()]

CORS_ALLOW_CREDENTIALS = False
