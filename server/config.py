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
TOPIC_MAVROS_MANUAL_CONTROL = "/mavros/manual_control/send"

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

# ── DXF alignment / mission-handoff ───────────────────────────────────────────
# Max allowable least-squares RMSE (metres) for multi-point DXF→NED alignment.
# Plans whose residual exceeds this are rejected (422) and never staged.
RMSE_MAX = float(os.environ.get("ROVER_ALIGN_RMSE_MAX", "0.05"))
# Max allowable |scale - 1.0| for multi-point DXF→NED alignment. Ref points and
# geometry share a metric frame, so a healthy fit lands scale≈1.0. A 2-point fit
# has RMSE≈0 and cannot catch unit/frame mismatch (e.g. double-scaled cm → ~100);
# this gate is the defense. 0.25 → accept scale in [0.75, 1.25].
SCALE_FIT_TOLERANCE = float(os.environ.get("ROVER_ALIGN_SCALE_TOL", "0.25"))
# Staged-mission lifetime (seconds). Older staging files are pruned on each plan.
STAGING_TTL_S = float(os.environ.get("ROVER_STAGING_TTL_S", "3600"))
# Litres of marking material consumed per metre of MARK path (site-tunable).
SPRAY_LITERS_PER_METER = float(os.environ.get("ROVER_SPRAY_L_PER_M", "0.012"))
# Default MARK flags for built-in / legacy non-DXF paths that carry no spray metadata.
SPRAY_DEFAULT_ON = os.environ.get("ROVER_SPRAY_DEFAULT_ON", "1") == "1"

# ── Safety / watchdog thresholds ──────────────────────────────────────────────
POSE_STALE_MS = 500.0  # consider pose stale above this
# Placement freshness (GPS_SURVEYED live EKF re-bind). Defaults match pose gate
# unless overridden — keep global/GPS slightly looser than local pose.
GLOBAL_POSITION_STALE_MS = float(os.environ.get("ROVER_GLOBAL_POS_STALE_MS", "500"))
GPS_FIX_STALE_MS = float(os.environ.get("ROVER_GPS_FIX_STALE_MS", "500"))
# PX4 only force-sends GPS_GLOBAL_ORIGIN at MAVLink stream start; if MAVROS
# connects later the latched gp_origin topic stays empty and placement loses its
# fixed datum. Ask for it, bounded.
ORIGIN_REQUEST_PERIOD_S = float(os.environ.get("ROVER_ORIGIN_REQ_PERIOD_S", "10.0"))
ORIGIN_REQUEST_MAX_TRIES = int(os.environ.get("ROVER_ORIGIN_REQ_MAX_TRIES", "30"))

POSE_GLOBAL_MAX_SKEW_MS = float(os.environ.get("ROVER_POSE_GLOBAL_SKEW_MS", "100"))
# Liveness of the RPP controller itself, measured on receipt of /rpp/debug (which it
# publishes every control tick). Distinct from the controller's own self-reported
# pose_age_ms, which freezes at its last value if the process dies. Generous relative
# to the tick rate so a scheduling hiccup can't trip it; a real death blows straight
# past it and is caught within SAFETY_STALE_GRACE_S.
RPP_DEBUG_STALE_MS = float(os.environ.get("ROVER_RPP_DEBUG_STALE_MS", "1000"))
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

# ── Auth ──────────────────────────────────────────────────────────────────────
TOKEN_HEADER_NAME = "X-Rover-Token"
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AUTH_PASSWORD_FILE = os.environ.get(
    "ROVER_PASSWORD_FILE",
    os.path.join(_REPO_ROOT, "config", "rover_password.json"),
)
AUTH_MACHINE_TOKENS_FILE = os.environ.get(
    "ROVER_MACHINE_TOKENS_FILE",
    os.path.join(_REPO_ROOT, "config", "rover_machine_tokens.json"),
)
AUTH_SESSION_TTL_S = float(os.environ.get("ROVER_SESSION_TTL_S", str(12 * 3600)))
AUTH_PBKDF2_ITERATIONS = int(os.environ.get("ROVER_AUTH_PBKDF2_ITERATIONS", "260000"))
# Accept both names: baseline ROVER_DISABLE_AUTH and ref ROVER_AUTH_DISABLED.
AUTH_DISABLED = (
    os.environ.get("ROVER_AUTH_DISABLED", "").lower() in {"1", "true", "yes"}
    or os.environ.get("ROVER_DISABLE_AUTH", "").lower() in {"1", "true", "yes"}
)
# Legacy path name for bag tooling; machine tokens live in AUTH_MACHINE_TOKENS_FILE.
TOKEN_FILE_DEFAULT = os.environ.get(
    "ROVER_TOKEN_FILE",
    os.path.join(_REPO_ROOT, "config", "bag_autorecord.token"),
)

# ── File upload limits ────────────────────────────────────────────────────────
ALLOWED_UPLOAD_EXTENSIONS = {".waypoints", ".csv", ".dxf"}
MAX_UPLOAD_BYTES = 5 * 1024 * 1024  # 5 MiB (DXF files can be large)

# ── Joystick / manual control (docs/Architecture/JOYSTICK_CONTROLLER_PLAN.md) ──
# Master switch. MUST stay "0" until the firmware gates in the plan (§7.1 axis
# mapping, §7.2 COM_RC_IN_MODE) are bench-verified — see plan §8 phase J3.
JOYSTICK_MANUAL_ENABLED = os.environ.get("ROVER_JOYSTICK_MANUAL_ENABLED", "0") == "1"
JOYSTICK_MANUAL_TRANSPORT = os.environ.get("ROVER_JOYSTICK_TRANSPORT", "mavros")
JOYSTICK_COMMAND_RATE_HZ = float(os.environ.get("ROVER_JOYSTICK_COMMAND_RATE_HZ", "20.0"))
JOYSTICK_GATEWAY_RATE_HZ = float(os.environ.get("ROVER_JOYSTICK_GATEWAY_RATE_HZ", "50.0"))
# Timeout-ordering safety chain (plan §4.5) — validated below at import time.
JOYSTICK_SERVER_STOP_TIMEOUT_S = float(
    os.environ.get("ROVER_JOYSTICK_SERVER_STOP_TIMEOUT_S", "0.30")
)
JOYSTICK_GATEWAY_STALE_TIMEOUT_S = float(
    os.environ.get("ROVER_JOYSTICK_GATEWAY_STALE_TIMEOUT_S", "0.40")
)
JOYSTICK_PX4_RC_LOSS_S = float(os.environ.get("ROVER_JOYSTICK_PX4_RC_LOSS_S", "0.50"))
JOYSTICK_LEASE_REVOKE_TIMEOUT_S = float(
    os.environ.get("ROVER_JOYSTICK_LEASE_REVOKE_TIMEOUT_S", "2.0")
)
JOYSTICK_LEASE_EXPIRY_S = float(os.environ.get("ROVER_JOYSTICK_LEASE_EXPIRY_S", "30.0"))
JOYSTICK_NEUTRAL_PRESTREAM_S = float(
    os.environ.get("ROVER_JOYSTICK_NEUTRAL_PRESTREAM_S", "0.20")
)
JOYSTICK_MODE_CONFIRM_TIMEOUT_S = float(
    os.environ.get("ROVER_JOYSTICK_MODE_CONFIRM_TIMEOUT_S", "3.0")
)
# Pinned conservative first-field-run defaults (plan §4.5/§7.8/§7.9) — do not
# inherit whichever default happens to drift between reference sources.
JOYSTICK_MAX_ABS_THROTTLE = float(os.environ.get("ROVER_JOYSTICK_MAX_ABS_THROTTLE", "0.10"))
JOYSTICK_MAX_ABS_STEERING = float(os.environ.get("ROVER_JOYSTICK_MAX_ABS_STEERING", "0.20"))
JOYSTICK_MAVROS_REQUIRE_SUBSCRIBER = (
    os.environ.get("ROVER_JOYSTICK_MAVROS_REQUIRE_SUBSCRIBER", "1") == "1"
)
JOYSTICK_MAVROS_PUBLISH_ERROR_LIMIT = int(
    os.environ.get("ROVER_JOYSTICK_MAVROS_PUBLISH_ERROR_LIMIT", "10")
)
JOYSTICK_PYMAVLINK_ENDPOINT = os.environ.get(
    "ROVER_JOYSTICK_PYMAVLINK_ENDPOINT", "udpout:127.0.0.1:14540"
)


def _validate_joystick_timeout_ordering() -> None:
    """Refuse to start if the joystick timeout safety chain is out of order.

    Meaning (plan §4.5): server zeros the command first, the gateway
    independently goes neutral next, PX4's own RC-loss failsafe is the
    backstop, and only after that is the lease revoked. Any other ordering
    lets a later, coarser layer misfire before an earlier, finer one has had
    a chance to act.
    """
    chain = [
        ("JOYSTICK_SERVER_STOP_TIMEOUT_S", JOYSTICK_SERVER_STOP_TIMEOUT_S),
        ("JOYSTICK_GATEWAY_STALE_TIMEOUT_S", JOYSTICK_GATEWAY_STALE_TIMEOUT_S),
        ("JOYSTICK_PX4_RC_LOSS_S", JOYSTICK_PX4_RC_LOSS_S),
        ("JOYSTICK_LEASE_REVOKE_TIMEOUT_S", JOYSTICK_LEASE_REVOKE_TIMEOUT_S),
    ]
    for (name_a, val_a), (name_b, val_b) in zip(chain, chain[1:]):
        if not val_a < val_b:
            raise RuntimeError(
                f"joystick timeout-ordering invariant violated: "
                f"{name_a}={val_a} must be < {name_b}={val_b}"
            )


_validate_joystick_timeout_ordering()

# ── CORS ──────────────────────────────────────────────────────────────────────
if AUTH_DISABLED:
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
