#!/usr/bin/env python3
"""Auto rosbag recorder — never forget to capture a mission again.

Polls the rover FastAPI mission status (`GET /api/mission/status`). The moment a
mission becomes active (ARMING/RUNNING/…), it starts `ros2 bag record` on the
curated debug topic set; when the mission returns to a terminal state
(IDLE/COMPLETED/ABORTED/ERROR) it sends SIGINT so rosbag2 finalises the
`.db3` + `metadata.yaml`, then waits for the next mission.

Bags are written to BAGS_DIR (default ~/bags_jet) named
`<loaded_path>_<YYYYmmdd_HHMMSS>`.

Pure stdlib (urllib + subprocess). Runs under systemd; ROS env is sourced by the
wrapper `bag_autorecord.sh`.

Env overrides:
  ROVER_API_BASE              default http://127.0.0.1:5001
  ROVER_MACHINE_TOKEN         raw machine token (preferred if set)
  ROVER_MACHINE_TOKEN_FILE    default <repo>/config/bag_autorecord.token
  ROVER_TOKEN_FILE            legacy alias for ROVER_MACHINE_TOKEN_FILE
  ROVER_DISABLE_AUTH /
  ROVER_AUTH_DISABLED         skip auth header when set
  BAGS_DIR                    default ~/bags_jet
  BAG_RECORD_ALL              "1" → record ALL topics (`-a`) instead of the curated list
  BAG_POLL_S                  default 0.2   (status poll interval)
  BAG_MAX_S                   default 1800  (hard cap on a single recording, safety)
  BAG_API_GRACE_S             default 8     (stop+finalise if API unreachable this long while recording)
"""
from __future__ import annotations
import hashlib, json, os, re, shutil, signal, socket, subprocess, sys, time, urllib.request
from datetime import datetime, timezone, timedelta

API_BASE   = os.environ.get("ROVER_API_BASE", "http://127.0.0.1:5001").rstrip("/")
STATUS_URL = f"{API_BASE}/api/mission/status"
LOADED_URL = f"{API_BASE}/api/mission/loaded-path"
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DEFAULT_MACHINE_TOKEN_FILE = os.path.join(_REPO_ROOT, "config", "bag_autorecord.token")
TOKEN_FILE = os.environ.get(
    "ROVER_MACHINE_TOKEN_FILE",
    os.environ.get("ROVER_TOKEN_FILE", _DEFAULT_MACHINE_TOKEN_FILE),
)
AUTH_OFF = (
    os.environ.get("ROVER_AUTH_DISABLED", "").lower() in {"1", "true", "yes"}
    or os.environ.get("ROVER_DISABLE_AUTH", "").lower() in {"1", "true", "yes"}
)
BAGS_DIR   = os.environ.get("BAGS_DIR", os.path.expanduser("~/bags_jet"))
RECORD_ALL = os.environ.get("BAG_RECORD_ALL", "0") == "1"
POLL_S     = float(os.environ.get("BAG_POLL_S", "0.2"))
MAX_S      = float(os.environ.get("BAG_MAX_S", "1800"))
API_GRACE_S = float(os.environ.get("BAG_API_GRACE_S", "8"))

# ── manifest / integrity / config capture (G2 + G3) ──────────────────────────
# IST is Asia/Kolkata (UTC+5:30). Computed from UTC so the readable local field
# is correct regardless of the Jetson's own TZ setting.
IST = timezone(timedelta(hours=5, minutes=30), name="IST")
# Best-effort FCU param snapshot (via MAVROS ParamGet) — the exact knobs the plan
# says behaviour must be attributable to. Capture is bounded + best-effort; any
# failure records null for that param and never blocks the bundle.
CAPTURE_FCU_PARAMS = os.environ.get("BAG_FCU_PARAMS", "1") == "1"
FCU_PARAM_NAMES = [
    "COM_OF_LOSS_T", "RO_YAW_P", "RO_YAW_RATE_LIM", "RO_MAX_THR_SPEED",
    "RD_TRANS_TRN_ARM", "RD_TRANS_ARM_TRN",
    "EKF2_WENC_CTRL", "RBCLW_COUNTS_REV",
    "NAV_ACC_RAD",
    "PWM_AUX_FUNC1", "PWM_AUX_MIN1", "PWM_AUX_MAX1", "PWM_AUX_DIS1",
]
# The RPP tuning block lives in /rpp/debug[11..38] (see rpp_controller_node.py).
# index -> readable label, so the manifest names each number.
RPP_DEBUG_PARAM_LABELS = {
    11: "max_linear_vel", 12: "min_linear_vel", 13: "min_lookahead_dist",
    14: "max_lookahead_dist", 15: "lookahead_time", 16: "a_lat_max",
    17: "regulated_linear_scaling_min_speed", 18: "xy_goal_tolerance",
    19: "min_goal_travel_m", 20: "approach_velocity_scaling_dist",
    21: "min_approach_linear_velocity", 22: "p4_zero_vel_threshold",
    23: "pose_max_age_s", 24: "ekf_jump_threshold_m", 25: "require_rtk_fix",
    26: "preview_curvature_n", 27: "xtrack_lookahead_gain",
    28: "path_resample_spacing_m", 29: "corner_smooth_radius_m",
    30: "corner_smooth_arc_pts", 31: "use_imu_extrapolation",
    32: "imu_max_extrap_age_s", 33: "use_feedforward_yaw_rate",
    34: "yaw_rate_feedback_gain", 35: "max_yaw_rate_body",
    36: "max_linear_accel", 37: "max_linear_decel", 38: "mission_speed",
}
# Services whose active-state is recorded in the manifest environment block.
WATCH_SERVICES = ["rover-server", "rpp-pipeline", "px4-dxp", "bag-autorecord"]

# ── disk management (G4) ─────────────────────────────────────────────────────
_GiB = 1024 ** 3
MIN_FREE_BYTES  = int(float(os.environ.get("BAG_MIN_FREE_BYTES",  str(5 * _GiB))))  # refuse to start below this
LOW_FREE_BYTES  = int(float(os.environ.get("BAG_LOW_FREE_BYTES",  str(2 * _GiB))))  # rotate to reclaim below this
MAX_TOTAL_BYTES = int(float(os.environ.get("BAG_MAX_TOTAL_BYTES", str(50 * _GiB)))) # rotate when bundles exceed this

# ── auto behaviour analysis on finalise (G6 wiring, P5) ──────────────────────
# Spawned detached + best-effort: a failed/absent analyser never affects the bag
# or the rover. Off the mission critical path (mission already terminal).
AUTO_ANALYZE = os.environ.get("BAG_AUTO_ANALYZE", "1") == "1"
_ANALYZER = os.path.join(_REPO_ROOT, "tools", "analyze_mission.py")

# Terminal mission states (anything else = active → record).
TERMINAL = {"idle", "completed", "aborted", "error", "none", ""}

# Curated debug/verification topic set (commanded vs actual, tracking, spray).
# EXPLICIT, not `-a`: keeps bag size bounded and lets the QoS overrides be
# targeted. Every entry is verified to exist on THIS tree (test/colinear-fix).
# main's G7 list also named /path/identity, /rpp/conditioned_path_identity,
# /rpp/setpoint_bridge_debug and /spray/runtime_status — NONE exist here, so they
# are deliberately omitted. /mavros/local_position/velocity_body was dropped
# (verified non-existent on the Jetson pluginlist).
TOPICS = [
    "/mavros/local_position/pose",        # actual trajectory + heading (ENU)
    "/mavros/local_position/velocity_local",  # measured ground speed (ENU)
    "/mavros/setpoint_raw/local",         # commanded vel/yaw → FCU (twist_to_setpoint)
    "/mavros/setpoint_velocity/cmd_vel",  # legacy vel setpoint (if used)
    "/mavros/state",                      # armed / mode (OFFBOARD drops)
    "/mavros/statustext",                 # PX4 failsafe / arm-reject reasons
    "/mavros/imu/data",                   # attitude / heading
    "/mavros/global_position/global",     # lat/lon
    "/mavros/gpsstatus/gps1/raw",         # RTK fix type / hrms / vrms
    "/path",                              # commanded path (LATCHED — see QoS override)
    "/rpp/conditioned_path",              # controller-conditioned path (LATCHED)
    "/rpp/debug",                         # xtrack, heading_err, speed, κ, state, params
    "/rpp/segment_debug",                 # segment FSM (state, seg idx, corner angle)
    "/rpp/velocity_ned",                  # commanded velocity NED
    "/rpp/yaw_rate_body",                 # commanded yaw rate
    "/spray/active",                      # desired MARK (RPP)
    "/spray/desired",                     # spray-controller desired state
    "/spray/commanded",                   # what the controller commanded to PX4 AUX
    "/spray/state",                       # actual sprayer state (controller)
    "/spray/debug",                       # spray timing / boundary metrics
    # ── added 2026-07-22: verified present on Upgrade_Spray (grep create_publisher)
    "/spray/status",                      # Spray V2 Phase A typed status (std_msgs/String JSON)
    "/spray/manual_state",                # manual-override state (POST /api/spray/test)
    "/dyx/mission/progress",              # 0.0→1.0 completion @1Hz (path_publisher)
    # ── added 2026-07-24: RPP↔spray progress handshake (G1–G5), needed to
    # validate the point-mode A/B from the bag. VOLATILE (not latched); progress
    # is BEST_EFFORT (see the QoS override), the rest RELIABLE. All silent unless
    # progress_publish_enabled / point_handshake_enabled are set at the rover.
    "/rpp/progress",                      # mission phase + dist-to-boundary @50Hz (BEST_EFFORT)
    "/rpp/milestone",                     # discrete edges: MARK_START/AT_POINT/… (RELIABLE)
    "/spray/point_done",                  # spray→RPP dwell-complete proof (RELIABLE)
    "/point/advance",                     # operator "next point" (G5 manual, RELIABLE)
    # ── added 2026-07-24: EKF local-frame origin (LATCHED — see QoS override).
    # The datum the local /path is expressed against; analyze_mission uses it to
    # render /path back into lat/lon for the geo overlay (surveyed vs commanded
    # vs driven). Published once early (after the server's MAV_CMD_REQUEST_MESSAGE).
    "/mavros/global_position/gp_origin",  # geographic_msgs/GeoPointStamped (TRANSIENT_LOCAL)
    # ── added 2026-07-26: the receiver's OWN lat/lon (NavSatFix from GPS_RAW_INT,
    # global_position plugin — not denylisted, so present). This is the only
    # dense position stream UPSTREAM of the EKF: /mavros/global_position/global
    # is ekf_origin + local NED (the EKF grading itself), and GPSRAW is only
    # ~5 Hz. Needed as the independent ruler for the WENC A/B drift analysis.
    # VOLATILE sensor topic — captured via the recorder's QoS adaptation, same
    # as /mavros/global_position/global; no override entry required.
    "/mavros/global_position/raw/fix",    # sensor_msgs/NavSatFix (EKF-independent)
]

# QoS profile overrides so the LATCHED (TRANSIENT_LOCAL) topics above are actually
# captured even though the recorder subscribes after they were published. Without
# this the bag has no /path and every downstream analysis is worthless (G1).
_DEFAULT_QOS_OVERRIDES = os.path.join(_REPO_ROOT, "config", "rosbag_qos_overrides.yaml")
QOS_OVERRIDES = os.environ.get("BAG_QOS_OVERRIDES", _DEFAULT_QOS_OVERRIDES)


def log(msg: str) -> None:
    print(f"[bag_autorecord] {datetime.now().isoformat(timespec='seconds')} {msg}", flush=True)


def _token() -> str | None:
    if AUTH_OFF:
        return None
    env_tok = os.environ.get("ROVER_MACHINE_TOKEN")
    if env_tok:
        return env_tok.strip() or None
    try:
        with open(TOKEN_FILE) as f:
            return f.read().strip() or None
    except OSError:
        return None


def poll_status() -> tuple[bool, str | None, str | None]:
    """Return (ok, state_lower, last_path_loaded). ok=False on HTTP/parse failure."""
    req = urllib.request.Request(STATUS_URL)
    tok = _token()
    if tok:
        req.add_header("X-Rover-Token", tok)
    try:
        with urllib.request.urlopen(req, timeout=1.0) as r:
            data = json.loads(r.read().decode())
    except Exception:
        return False, None, None
    state = str(data.get("state", "")).split(".")[-1].lower()  # handles "running" or "MissionState.RUNNING"
    return True, state, data.get("last_path_loaded")


def _safe_name(name: str | None) -> str:
    base = re.sub(r"[^A-Za-z0-9._-]+", "_", (name or "mission").strip()) or "mission"
    return base[:60]


# ── secret masking (G3 / R5) ─────────────────────────────────────────────────
# Nothing sensitive should ever reach a manifest, a log line, or a captured
# statustext. Two shapes: key/value secrets, and credentials embedded in URLs.
# Match the FULL key (so NTRIP_PASSWORD, X-Rover-Token, api_key all mask) followed
# by its value. The keyword may sit anywhere inside a longer identifier.
_SECRET_KV = re.compile(
    r"(?i)([a-z0-9_.\-]*(?:token|password|passwd|secret|api[_-]?key|authorization)[a-z0-9_.\-]*)"
    r"\s*[:=]\s*['\"]?([^\s'\",;}]+)"
)
_URL_CREDS = re.compile(r"(?i)\b([a-z][a-z0-9+.\-]*://)([^/\s:@]+):([^/\s@]+)@")


def _redact(s):
    """Mask secrets in a single string. Non-strings pass through unchanged."""
    if not isinstance(s, str) or not s:
        return s
    s = _URL_CREDS.sub(r"\1\2:***@", s)
    s = _SECRET_KV.sub(lambda m: f"{m.group(1)}=***", s)
    return s


def _redact_obj(obj):
    """Recursively redact every string inside a JSON-able structure."""
    if isinstance(obj, dict):
        return {k: _redact_obj(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_redact_obj(v) for v in obj]
    return _redact(obj)


# ── time helpers ─────────────────────────────────────────────────────────────
def _now_utc() -> datetime:
    return datetime.now(timezone.utc)


def _stamp(dt: datetime) -> dict:
    """UTC ISO + human-readable IST for one instant."""
    return {
        "utc": dt.astimezone(timezone.utc).isoformat(timespec="seconds"),
        "ist": dt.astimezone(IST).strftime("%Y-%m-%d %H:%M:%S %Z"),
        "epoch": round(dt.timestamp(), 3),
    }


# ── subprocess + integrity helpers ───────────────────────────────────────────
def _run(cmd: list[str], timeout: float = 5.0) -> str | None:
    """Best-effort capture of a command's stdout. None on any failure/timeout."""
    try:
        out = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout,
        )
        if out.returncode != 0:
            return None
        return out.stdout
    except Exception:
        return None


def _sha256_file(path: str) -> str | None:
    try:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b""):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return None


def _dir_bytes(path: str) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total


def _bundle_integrity(bundle_dir: str, exclude: set[str]) -> dict:
    """SHA256 every file in the bundle (except the manifest itself)."""
    files = {}
    for root, _dirs, names in os.walk(bundle_dir):
        for name in names:
            full = os.path.join(root, name)
            rel = os.path.relpath(full, bundle_dir)
            if rel in exclude:
                continue
            files[rel] = {
                "bytes": (os.path.getsize(full) if os.path.exists(full) else None),
                "sha256": _sha256_file(full),
            }
    return {"file_count": len(files), "files": files}


# ── as-run config + environment capture (G2) ─────────────────────────────────
def _git_sha() -> str | None:
    out = _run(["git", "-C", _REPO_ROOT, "rev-parse", "--short=12", "HEAD"], timeout=3.0)
    return out.strip() if out else None


def _service_states() -> dict:
    states = {}
    for svc in WATCH_SERVICES:
        out = _run(["systemctl", "is-active", svc], timeout=3.0)
        states[svc] = (out.strip() if out else "unknown")
    return states


def _environment() -> dict:
    return {
        "git_sha": _git_sha(),
        "services": _service_states(),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "hostname": socket.gethostname(),
        "recorder_pid": os.getpid(),
    }


def _fcu_params() -> dict:
    """Best-effort MAVROS ParamGet snapshot of the curated FCU knobs.

    Off the mission critical path (runs at finalise), bounded per-param, and
    tolerant: a param that can't be read is recorded as null, never an error.
    """
    if not CAPTURE_FCU_PARAMS:
        return {"captured": False, "reason": "disabled", "values": {}}
    values: dict = {}
    any_ok = False
    for pid in FCU_PARAM_NAMES:
        out = _run(
            ["ros2", "service", "call", "/mavros/param/get",
             "mavros_msgs/srv/ParamGet", f"{{param_id: '{pid}'}}"],
            timeout=4.0,
        )
        val = None
        if out and "success=True" in out.replace(" ", ""):
            # response embeds mavros_msgs/ParamValue{integer, real}
            m_int = re.search(r"integer=(-?\d+)", out)
            m_real = re.search(r"real=(-?\d+\.?\d*(?:e-?\d+)?)", out)
            iv = int(m_int.group(1)) if m_int else 0
            rv = float(m_real.group(1)) if m_real else 0.0
            val = rv if rv != 0.0 else iv
            any_ok = True
        values[pid] = val
    return {"captured": any_ok, "values": values}


def _rpp_param_block() -> dict:
    """One /rpp/debug sample → the RPP tuning block [11..38], labelled.

    Captured while RPP is actively publishing (called right after record start).
    """
    out = _run(
        ["ros2", "topic", "echo", "--once", "--field", "data",
         "/rpp/debug", "std_msgs/msg/Float32MultiArray"],
        timeout=6.0,
    )
    if not out:
        return {"captured": False, "values": {}}
    nums = re.findall(r"-?\d+\.?\d*(?:e-?\d+)?", out)
    try:
        arr = [float(x) for x in nums]
    except ValueError:
        return {"captured": False, "values": {}}
    values = {}
    for idx, label in RPP_DEBUG_PARAM_LABELS.items():
        values[label] = (round(arr[idx], 6) if idx < len(arr) else None)
    return {"captured": bool(values) and len(arr) > 38, "values": values}


def _loaded_path_identity() -> dict:
    """Best-effort read-only identity from GET /api/mission/loaded-path.

    Failure → minimal identity (never skip the bag). Never raises.
    """
    req = urllib.request.Request(LOADED_URL)
    tok = _token()
    if tok:
        req.add_header("X-Rover-Token", tok)
    try:
        with urllib.request.urlopen(req, timeout=2.0) as r:
            data = json.loads(r.read().decode())
    except Exception:
        return {"available": False}
    keep = ("loaded", "name", "num_waypoints", "num_mark", "num_transit",
            "has_spray_flags", "placement_mode", "origin_gps", "is_staged",
            "mission_id")
    ident = {k: data.get(k) for k in keep if k in data}
    ident["available"] = True
    return ident


# ── staged-mission provenance ────────────────────────────────────────────────
# Without this the bundle cannot answer "what file produced this drive, and was
# it georeferenced?". manifest.identity only carries counts and origin_gps —
# nothing about the SOURCE. The 2026-07-18 georef investigation had to guess the
# DXF from a hand-copied ref_dxf/ folder someone happened to create.
#
# The staged mission JSON holds the global anchor (lat/lon/rotation/scale) and
# the alignment fit (method, rmse, fitted_scale, residuals). The bundle name IS
# the mission_id, so it is a direct lookup.
#
# It did NOT hold the file provenance: this reader was written against an assumed
# metadata.source dict, but the server wrote a bare filename STRING there, so
# source_file was empty in every bundle recorded before 2026-07-22 and §8
# absolute accuracy reported "unavailable". The server now also writes
# metadata.source_detail (a dict); _source_block below reads either shape and
# resolves a legacy bare filename against server/missions/.
STAGING_DIR = os.path.join(_REPO_ROOT, "server", "missions", "staging")


MISSIONS_DIR = os.path.join(_REPO_ROOT, "server", "missions")


def _source_block(metadata: dict) -> dict:
    """Normalise the staged artifact's source provenance to a dict.

    Three shapes exist on disk and all three must work, because staged files
    written by an older server outlive the deploy that fixed them:

      * ``source_detail`` — a dict (current server).
      * ``source`` — a dict (never shipped, but the schema allows it).
      * ``source`` — a bare filename string (every file staged before this fix).
        Resolve it against server/missions/ so §8 still gets a real path.

    Returns {} when nothing resolves. Must not raise: the caller's contract is
    that provenance is best-effort and never blocks a recording.
    """
    detail = metadata.get("source_detail")
    if isinstance(detail, dict) and detail:
        return detail

    source = metadata.get("source")
    if isinstance(source, dict):
        return source
    if isinstance(source, str) and source and not source.startswith("builtin:"):
        out = {"name": source, "extension": os.path.splitext(source)[1].lower() or None}
        candidate = os.path.join(MISSIONS_DIR, os.path.basename(source))
        if os.path.isfile(candidate):
            out["filepath"] = candidate
        return out
    return {}


def _staged_mission(mission_id: str | None) -> dict:
    """Read the staged mission artifact for *mission_id*. Never raises.

    Returns the provenance block for the manifest. The full artifact is written
    to the bundle separately (see BagSession.start) so the exact commanded
    geometry survives even after STAGING_TTL_S prunes the original.
    """
    if not mission_id:
        return {"available": False, "reason": "no mission_id in identity"}
    path = os.path.join(STAGING_DIR, f"{mission_id}.json")
    try:
        with open(path) as f:
            d = json.load(f)
    except Exception as exc:
        return {"available": False, "reason": f"{type(exc).__name__}: {exc}"}

    anchor = d.get("anchor") or {}
    align = d.get("alignment_metadata") or {}
    source = _source_block(d.get("metadata") or {})
    # A georeferenced DXF is one the parser projected from lat/lon: georef.py
    # stamps geo_origin, the planner promotes it to origin_gps, and the plan is
    # staged GPS_SURVEYED with alignment method "gps_origin" and no ref points.
    return {
        "available": True,
        "staged_file": path,
        "source_file": source.get("filepath"),
        "source_extension": source.get("extension"),
        "unit_scale_m_per_unit": source.get("unit_scale_m_per_unit"),
        # Operator's per-survey vertex tolerance; None = analyser default.
        "survey_tolerance_m": (d.get("metadata") or {}).get("survey_tolerance_m"),
        "placement_mode": d.get("placement_mode"),
        "anchor": anchor or None,
        "alignment": {
            "method": align.get("method"),
            "rotation_deg": align.get("rotation_deg"),
            "scale": align.get("scale"),
            "fitted_scale": align.get("fitted_scale"),
            "rmse": align.get("rmse"),
            "residuals": align.get("residuals"),
        },
        "is_georeferenced": bool(d.get("origin_gps")) and align.get("method") == "gps_origin",
        "num_waypoints": len(d.get("waypoints") or []),
        "num_mark": sum(1 for f in (d.get("spray_flags") or []) if f),
        "mark_length_m": (d.get("metadata") or {}).get("mark_length_m"),
        "transit_length_m": (d.get("metadata") or {}).get("transit_length_m"),
    }


def _snapshot_staged_artifact(bundle_dir: str, mission_id: str | None) -> None:
    """Copy the full staged mission JSON into the bundle. Best-effort, never raises."""
    if not mission_id:
        return
    src = os.path.join(STAGING_DIR, f"{mission_id}.json")
    try:
        with open(src) as f:
            data = f.read()
        with open(os.path.join(bundle_dir, "staged_mission.json"), "w") as f:
            f.write(data)
        log(f"staged mission artifact snapshotted ({len(data)} bytes)")
    except Exception as exc:
        log(f"staged mission snapshot skipped: {type(exc).__name__}: {exc}")


# ── manifest read/write (G2) ─────────────────────────────────────────────────
MANIFEST_NAME = "manifest.json"
INCOMPLETE_SENTINEL = "INCOMPLETE"


def _write_manifest(bundle_dir: str, manifest: dict) -> None:
    """Atomically write manifest.json, redacting every string first (G3)."""
    safe = _redact_obj(manifest)
    tmp = os.path.join(bundle_dir, MANIFEST_NAME + ".tmp")
    final = os.path.join(bundle_dir, MANIFEST_NAME)
    try:
        with open(tmp, "w") as f:
            json.dump(safe, f, indent=2, sort_keys=False)
        os.replace(tmp, final)
    except OSError as e:
        log(f"  manifest write failed: {e}")


def _read_manifest(bundle_dir: str) -> dict | None:
    try:
        with open(os.path.join(bundle_dir, MANIFEST_NAME)) as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return None


# ── disk management (G4) ─────────────────────────────────────────────────────
def _free_bytes(path: str) -> int:
    try:
        return shutil.disk_usage(path).free
    except OSError:
        return 1 << 62  # unknown → don't block (degrade only on real evidence)


def _list_bundles(bags_dir: str) -> list[str]:
    """Absolute paths of bundle dirs (dirs holding a manifest.json), oldest first."""
    out = []
    try:
        for name in os.listdir(bags_dir):
            full = os.path.join(bags_dir, name)
            if os.path.isdir(full) and os.path.exists(os.path.join(full, MANIFEST_NAME)):
                out.append(full)
    except OSError:
        return []
    out.sort(key=lambda p: os.path.getmtime(p) if os.path.exists(p) else 0)
    return out


def _preflight_free_space(bags_dir: str) -> bool:
    """True if there is room to start a capture. Warns + refuses below the floor."""
    free = _free_bytes(bags_dir)
    if free < MIN_FREE_BYTES:
        log(f"WARN low disk: {free/_GiB:.1f} GiB free < floor {MIN_FREE_BYTES/_GiB:.1f} GiB "
            f"— refusing to start capture (rover services unaffected)")
        return False
    return True


def _enforce_retention(bags_dir: str) -> None:
    """Delete oldest bundles until under the byte cap AND above the low-free mark.

    Never touches an in-progress bundle (no end + no manifest outcome) that is the
    newest; only fully-finalised older bundles are candidates. Degrades the
    recorder's own store only — never any rover service.
    """
    try:
        bundles = _list_bundles(bags_dir)
        if not bundles:
            return
        total = sum(_dir_bytes(b) for b in bundles)
        # keep the newest bundle regardless (it may be the one just written)
        candidates = bundles[:-1]
        while candidates and (
            total > MAX_TOTAL_BYTES or _free_bytes(bags_dir) < LOW_FREE_BYTES
        ):
            victim = candidates.pop(0)
            vbytes = _dir_bytes(victim)
            try:
                shutil.rmtree(victim)
                total -= vbytes
                log(f"  retention: removed oldest bundle {os.path.basename(victim)} "
                    f"({vbytes/_GiB:.2f} GiB)")
            except OSError as e:
                log(f"  retention: could not remove {victim}: {e}")
                break
    except Exception as e:
        log(f"  retention error (ignored): {e}")


# ── crash reconciliation (G5) ────────────────────────────────────────────────
def reconcile_incomplete(bags_dir: str) -> None:
    """On daemon start, label any bundle that never got an `end` timestamp.

    A power-cut / OOM / kill-9 leaves a bundle whose manifest has no
    outcome.recorder_end. Mark it INCOMPLETE (+ sentinel file) and finalise the
    manifest with whatever integrity we can still compute, so evidence is
    labelled rather than silently corrupt.
    """
    for bundle in _list_bundles(bags_dir):
        manifest = _read_manifest(bundle)
        if manifest is None:
            continue
        outcome = manifest.get("outcome") or {}
        if outcome.get("recorder_end"):
            continue  # cleanly finalised
        log(f"reconcile: unfinalised bundle {os.path.basename(bundle)} → INCOMPLETE")
        sentinel = os.path.join(bundle, INCOMPLETE_SENTINEL)
        try:
            with open(sentinel, "w") as f:
                f.write(_now_utc().isoformat(timespec="seconds") + "\n")
        except OSError:
            pass
        outcome["status"] = "INCOMPLETE"
        outcome["recorder_end"] = _stamp(_now_utc())
        outcome["note"] = "finalised by crash reconciliation on daemon start"
        outcome["integrity"] = _bundle_integrity(
            bundle, exclude={MANIFEST_NAME, MANIFEST_NAME + ".tmp"}
        )
        manifest["outcome"] = outcome
        _write_manifest(bundle, manifest)


def _spawn_analyzer(bundle_dir: str) -> None:
    """Fire-and-forget behaviour analysis on a finalised bundle (best-effort).

    Detached (own session), non-blocking; output goes to analyze.log in the
    bundle. Any failure here is swallowed — the bag and the rover are untouched.
    """
    if not AUTO_ANALYZE or not os.path.isfile(_ANALYZER):
        return
    try:
        logpath = os.path.join(bundle_dir, "analyze.log")
        logf = open(logpath, "w")
        subprocess.Popen(
            ["python3", _ANALYZER, bundle_dir, "--quiet"],
            start_new_session=True, stdout=logf, stderr=subprocess.STDOUT,
        )
        log(f"  analyser spawned → {os.path.join(bundle_dir, 'analysis.json')}")
    except Exception as e:
        log(f"  analyser spawn failed (ignored): {e}")


class Recorder:
    def __init__(self) -> None:
        self.proc: subprocess.Popen | None = None
        self.bundle_dir: str | None = None   # <mission>_<utc>/ holding bag + manifest
        self.bag_dir: str | None = None       # <bundle>/bag  (ros2 bag -o target)
        self.manifest: dict | None = None
        self.start_t: float = 0.0
        self._last_refuse_log: float = 0.0

    @property
    def active(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self, path_name: str | None) -> None:
        os.makedirs(BAGS_DIR, exist_ok=True)

        # G4 preflight — a full disk degrades the recorder only, never the rover.
        if not _preflight_free_space(BAGS_DIR):
            # throttle the warning so a low-disk mission doesn't spam the journal
            now = time.time()
            if now - self._last_refuse_log > 30.0:
                self._last_refuse_log = now
            return  # rec stays inactive; mission proceeds unaffected (R1)

        # G4 — reclaim space up front if we're already tight (best-effort).
        _enforce_retention(BAGS_DIR)

        started = _now_utc()
        stamp = started.astimezone(IST).strftime("%Y%m%d_%H%M%S")
        name = f"{_safe_name(path_name)}_{stamp}"
        self.bundle_dir = os.path.join(BAGS_DIR, name)
        os.makedirs(self.bundle_dir, exist_ok=True)
        self.bag_dir = os.path.join(self.bundle_dir, "bag")

        cmd = ["ros2", "bag", "record", "-o", self.bag_dir]
        # Capture the latched /path & /rpp/conditioned_path (G1). Only applies to
        # the curated set; `-a` mode can't target per-topic QoS reliably.
        if not RECORD_ALL and QOS_OVERRIDES and os.path.isfile(QOS_OVERRIDES):
            cmd += ["--qos-profile-overrides-path", QOS_OVERRIDES]
        elif not RECORD_ALL and QOS_OVERRIDES:
            log(f"WARN qos overrides file missing ({QOS_OVERRIDES}) — latched /path may not be captured")
        cmd += ["-a"] if RECORD_ALL else TOPICS
        log(f"START recording → {self.bundle_dir}  ({'ALL topics' if RECORD_ALL else f'{len(TOPICS)} topics'})")
        # own process group so SIGINT targets the whole ros2 bag tree
        self.proc = subprocess.Popen(cmd, start_new_session=True,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        self.start_t = time.time()

        # Build the initial manifest AFTER the bag is already recording, so none
        # of this best-effort capture can lose data or block the mission (R1).
        identity = _loaded_path_identity()
        mission_id = identity.get("mission_id")
        _snapshot_staged_artifact(self.bundle_dir, mission_id)
        self.manifest = {
            "schema": "bag_autorecord/manifest@1",
            "bundle": name,
            "identity": identity,
            "plan_provenance": _staged_mission(mission_id),
            "timestamps": {
                "recorder_start": _stamp(started),
                "mission_start_observed": _stamp(started),
            },
            "as_run_config": {
                "rpp_params": _rpp_param_block(),   # RPP publishing now — capture live
                "fcu_params": {"captured": False, "values": {}},  # filled at finalise
                "recorder": {
                    "topics": ("ALL" if RECORD_ALL else TOPICS),
                    "qos_overrides": (QOS_OVERRIDES if os.path.isfile(QOS_OVERRIDES or "") else None),
                },
            },
            "environment": _environment(),
            "outcome": {"status": "RECORDING", "recorder_end": None},
        }
        _write_manifest(self.bundle_dir, self.manifest)

    def stop(self, reason: str) -> None:
        if not self.active:
            self.proc = None
            return
        bundle = self.bundle_dir
        log(f"STOP recording ({reason}) → finalising {os.path.basename(bundle or '')}")
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)  # rosbag2 writes metadata.yaml on SIGINT
            self.proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            log("  finalise slow — sending SIGTERM")
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                self.proc.wait(timeout=10)
            except Exception:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        except Exception as e:
            log(f"  stop error: {e}")
        self.proc = None

        # Finalise the manifest (G2/G3) — bag is now closed on disk.
        if bundle and self.manifest is not None:
            try:
                ended = _now_utc()
                # FCU params captured here: mission is terminal, so a few seconds
                # of ParamGet can't affect it. MAVROS (px4-dxp) is still up.
                self.manifest["as_run_config"]["fcu_params"] = _fcu_params()
                self.manifest["timestamps"]["recorder_end"] = _stamp(ended)
                self.manifest["timestamps"]["mission_end_observed"] = _stamp(ended)
                self.manifest["outcome"] = {
                    # RECORDER status only: the bag was closed in an orderly way.
                    # It is NOT a statement about the mission — a clean abort at
                    # 40% and a full traversal both land here, which is why runs
                    # covering 24/64 and 76/86 waypoints both read COMPLETE.
                    # mission_end_reason distinguishes WHY it ended; the traversal
                    # block below says how much of the path was actually driven.
                    "status": "COMPLETE",
                    "means": "recorder finalised cleanly; see traversal for mission coverage",
                    "mission_end_reason": reason,
                    "recorder_end": _stamp(ended),
                    "integrity": _bundle_integrity(
                        bundle, exclude={MANIFEST_NAME, MANIFEST_NAME + ".tmp"}
                    ),
                }
                # Coverage needs the closed bag, so the analyser fills this in
                # (it is spawned just below). PENDING is written now so that a
                # missing verdict reads as "not analysed yet" rather than as a
                # silent pass — absence must never look like success.
                self.manifest["traversal"] = {
                    "status": "PENDING",
                    "source": "awaiting analyze_mission",
                }
                _write_manifest(bundle, self.manifest)
                log(f"  saved: {bundle}  (manifest + integrity written)")
            except Exception as e:
                log(f"  finalise-manifest error (bag is safe): {e}")
            # G6 wiring — kick off the offline behaviour analysis (detached).
            _spawn_analyzer(bundle)
            # G4 — keep the store bounded after each capture.
            _enforce_retention(BAGS_DIR)
        else:
            log(f"  saved: {bundle}")

        self.bundle_dir = None
        self.bag_dir = None
        self.manifest = None


def main() -> int:
    rec = Recorder()
    stop_flag = {"v": False}

    def _sig(_s, _f):
        stop_flag["v"] = True
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    log(f"watching {STATUS_URL}  bags→{BAGS_DIR}  auth={'off' if AUTH_OFF else 'on'}")

    # G5 — before watching, label any bundle a previous crash left unfinalised.
    try:
        reconcile_incomplete(BAGS_DIR)
    except Exception as e:
        log(f"reconcile error (ignored): {e}")

    api_fail_since: float | None = None

    while not stop_flag["v"]:
        ok, state, path = poll_status()

        if ok:
            api_fail_since = None
            is_active = state not in TERMINAL
            if is_active and not rec.active:
                rec.start(path)
            elif (not is_active) and rec.active:
                rec.stop(f"mission {state or 'terminal'}")
        else:
            # transient API failure: don't stop immediately (server may be restarting),
            # but if it stays unreachable while recording, finalise to protect the bag.
            if rec.active:
                api_fail_since = api_fail_since or time.time()
                if time.time() - api_fail_since > API_GRACE_S:
                    rec.stop("api_unreachable")
                    api_fail_since = None

        # safety cap on a single recording
        if rec.active and (time.time() - rec.start_t) > MAX_S:
            rec.stop("max_duration_cap")

        time.sleep(POLL_S)

    if rec.active:
        rec.stop("service_shutdown")
    log("exiting")
    return 0


if __name__ == "__main__":
    sys.exit(main())
