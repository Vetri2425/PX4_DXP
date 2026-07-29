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

# GPS Fix Type Names — MAVLink GPS_FIX_TYPE enum, fed by mavros_msgs/GPSRAW
# .fix_type (/mavros/gpsstatus/gps1/raw). NOT the ROS NavSatStatus.status enum
# (-1..2), which is what the old table here was written against: it had no
# key 3, so a plain 3D fix (the receiver's state whenever RTK corrections are
# absent) fell through to "UNKNOWN" on every telemetry frame. Keep in sync
# with the copies in src/spray_controller_node.py and src/rpp_controller_node.py.
GPS_FIX_NAMES = {
    0: "NO_GPS",
    1: "NO_FIX",
    2: "2D_FIX",
    3: "3D_FIX",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
    7: "STATIC",
    8: "PPP",
}

# B2: codes that mean "controller is not driving safely". Treat the same as
# STALE for safety-abort and OFFBOARD-start guard purposes. Centralised here
# so server/main.py and server/offboard_controller.py stay in sync.
RPP_UNHEALTHY_CODES = {RPP_STALE, RPP_RTK_WAIT, RPP_JUMP_SKIP}

# Decimal places for lat/lon/alt in outbound telemetry (WS + REST) only.
# 8 decimal degrees is sub-millimetre resolution — finer than RTK's real
# ~1-2 cm — so nothing is lost; this just replaces the variable 6-17 digit
# count of Python's shortest-round-trip float repr with a consistent wire
# format at both client-facing boundaries. Internal state (ros_node.py) and
# mission placement (resolve_surveyed_points) keep full float64 for
# anchor/EKF math.
GPS_TELEMETRY_DECIMALS = 8


def format_gps_coord(value: float | None) -> float | None:
    """Round a lat/lon/alt value to GPS_TELEMETRY_DECIMALS for client emit."""
    return None if value is None else round(value, GPS_TELEMETRY_DECIMALS)

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
# On natural mission completion (RPP DONE settled), command spray OFF and disarm
# the rover so it ends the mission safe without an operator E-stop (field bug
# B4, 2026-07-25). Default ON. Set ROVER_DISARM_ON_COMPLETE=0 to keep the rover
# armed at completion (old behaviour: mark COMPLETED only).
DISARM_ON_COMPLETE = os.environ.get("ROVER_DISARM_ON_COMPLETE", "1") == "1"

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

# ── EKF local-frame origin trust (2026-07-27 stale-origin field bug) ──────────
# Max allowed disagreement (metres) between the DECLARED origin (gp_origin) and
# the origin IMPLIED by a simultaneous (lat, lon, pos_n, pos_e) sample. Beyond
# this, placement refuses — see server/origin_health.py for the full derivation.
#
# Budget for a HEALTHY sample:
#   GLOBAL_POSITION_INT degE7 quantisation .......... ~1.1 cm N, ~1.1 cm E
#   pose/global receive skew (<=POSE_GLOBAL_MAX_SKEW_MS
#     = 100 ms) at up to 1.0 m/s (SPD-T1 ceiling) ... <=10 cm
#   RTK_FIXED solution noise ......................... ~1-2 cm
#   => worst-case healthy ~12 cm; measured on the rig 2026-07-27: 1.9-2.0 cm.
# Smallest observed FAULT: 0.91 m (declared-but-wrong), 2.15/2.25 m (stale).
# 0.30 m sits 2.5x above the worst healthy case and 3x below the smallest
# observed fault — the widest gap available between the two populations.
ORIGIN_CONSISTENCY_MAX_M = float(os.environ.get("ROVER_ORIGIN_MAX_DELTA_M", "0.30"))
# Fail closed when NO declared origin is available at all. With this set the
# non-deterministic live pose/global fallback in resolve_surveyed_points is
# unreachable and surveyed missions refuse to place until gp_origin arrives
# (the bounded MAV_CMD_REQUEST_MESSAGE retry normally fetches it within
# ORIGIN_REQUEST_PERIOD_S). Set ROVER_ORIGIN_REQUIRE_DECLARED=0 to re-enable
# the fallback — it places accurately but varies ~1 cm run to run, so a mission
# placed that way is NOT bit-reproducible.
ORIGIN_REQUIRE_DECLARED = os.environ.get("ROVER_ORIGIN_REQUIRE_DECLARED", "1") == "1"
# A gap this long in /mavros/state means the FCU link or MAVROS itself went
# away and came back — a new EKF session is possible, so the cached origin is
# dropped and re-requested. Deliberately far above BRIDGE_STATE_STALE_MS
# (2.5 s): a spurious invalidation costs a real refusal window, so it must take
# several consecutive missed /mavros/state publishes, not one scheduling hiccup.
ORIGIN_LINK_GAP_S = float(os.environ.get("ROVER_ORIGIN_LINK_GAP_S", "5.0"))

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
# Headless first-boot: ship a known default password, then FORCE rotation via
# require_operator_token (403 password_change_required) until changed.
# Env-overridable so a site can set its own at deploy time. NEVER log the value.
# Set ROVER_AUTH_BOOTSTRAP_ENABLED=0 to keep the old "CLI setup required" behaviour.
AUTH_BOOTSTRAP_ENABLED = os.environ.get(
    "ROVER_AUTH_BOOTSTRAP_ENABLED", "1"
).lower() in {"1", "true", "yes"}
AUTH_BOOTSTRAP_PASSWORD = os.environ.get(
    "ROVER_BOOTSTRAP_PASSWORD", "rover-setup-0000"
)
if len(AUTH_BOOTSTRAP_PASSWORD) < 8:
    raise ValueError(
        "ROVER_BOOTSTRAP_PASSWORD must be at least 8 characters "
        f"(got length {len(AUTH_BOOTSTRAP_PASSWORD)})"
    )
# Legacy path name for bag tooling; machine tokens live in AUTH_MACHINE_TOKENS_FILE.
TOKEN_FILE_DEFAULT = os.environ.get(
    "ROVER_TOKEN_FILE",
    os.path.join(_REPO_ROOT, "config", "bag_autorecord.token"),
)

# ── File upload limits ────────────────────────────────────────────────────────
ALLOWED_UPLOAD_EXTENSIONS = {".waypoints", ".csv", ".dxf"}
MAX_UPLOAD_BYTES = 5 * 1024 * 1024  # 5 MiB (DXF files can be large)

# ── App-planned trajectory limits (POST /api/path/plan-trajectory) ────────────
# MAX_UPLOAD_BYTES guards multipart /upload; it does NOT see a JSON body, so
# without these an oversized trajectory reaches the densifier and surfaces as an
# opaque timeout. Both are checked by the request model / route validation, so a
# too-large payload is a clear 422 naming the count and the limit.
#
# Sizing: the app's own worst case is ~1.1 km of road ≈ 4 k input points across
# a few hundred runs. These sit an order of magnitude above that — they are a
# runaway guard, not a working limit.
MAX_TRAJECTORY_RUNS = 5000
MAX_TRAJECTORY_POINTS = 200000

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
# Arm-on-acquire: acquire() arms the vehicle itself after MANUAL is confirmed
# and neutral MANUAL_CONTROL is already streaming (so PX4's manual-control-loss
# arming check is satisfied). Gives open→drive without a separate arm
# round-trip from the client. Still master-gated by JOYSTICK_MANUAL_ENABLED.
JOYSTICK_AUTO_ARM_ENABLED = os.environ.get("ROVER_JOYSTICK_AUTO_ARM", "1") == "1"
JOYSTICK_ARM_CONFIRM_TIMEOUT_S = float(
    os.environ.get("ROVER_JOYSTICK_ARM_CONFIRM_TIMEOUT_S", "5.0")
)
# Pinned conservative first-field-run defaults (plan §4.5/§7.8/§7.9) — do not
# inherit whichever default happens to drift between reference sources.
JOYSTICK_MAX_ABS_THROTTLE = float(os.environ.get("ROVER_JOYSTICK_MAX_ABS_THROTTLE", "0.35"))
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
