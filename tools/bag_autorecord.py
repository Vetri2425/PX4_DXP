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
  ROVER_API_BASE     default http://127.0.0.1:5001
  ROVER_TOKEN_FILE   default ~/.rover_token   (X-Rover-Token; skipped if ROVER_DISABLE_AUTH=1)
  BAGS_DIR           default ~/bags_jet
  BAG_RECORD_ALL     "1" → record ALL topics (`-a`) instead of the curated list
  BAG_POLL_S         default 0.2   (status poll interval)
  BAG_MAX_S          default 1800  (hard cap on a single recording, safety)
  BAG_API_GRACE_S    default 8     (stop+finalise if API unreachable this long while recording)
"""
from __future__ import annotations
import json, os, re, signal, subprocess, sys, time, urllib.request
from datetime import datetime

API_BASE   = os.environ.get("ROVER_API_BASE", "http://127.0.0.1:5001").rstrip("/")
STATUS_URL = f"{API_BASE}/api/mission/status"
TOKEN_FILE = os.environ.get("ROVER_TOKEN_FILE", os.path.expanduser("~/.rover_token"))
AUTH_OFF   = os.environ.get("ROVER_DISABLE_AUTH", "0") == "1"
BAGS_DIR   = os.environ.get("BAGS_DIR", os.path.expanduser("~/bags_jet"))
RECORD_ALL = os.environ.get("BAG_RECORD_ALL", "0") == "1"
POLL_S     = float(os.environ.get("BAG_POLL_S", "0.2"))
MAX_S      = float(os.environ.get("BAG_MAX_S", "1800"))
API_GRACE_S = float(os.environ.get("BAG_API_GRACE_S", "8"))

# Terminal mission states (anything else = active → record).
TERMINAL = {"idle", "completed", "aborted", "error", "none", ""}

# Curated debug/verification topic set (commanded vs actual, tracking, spray).
TOPICS = [
    "/mavros/local_position/pose",        # actual trajectory + heading
    "/mavros/local_position/velocity_body",
    "/mavros/local_position/velocity_local",
    "/mavros/setpoint_raw/local",         # commanded vel/yaw → FCU
    "/mavros/setpoint_velocity/cmd_vel",
    "/mavros/state",                      # armed / mode
    "/mavros/imu/data",                   # attitude / heading
    "/mavros/global_position/global",     # lat/lon
    "/mavros/gpsstatus/gps1/raw",         # RTK fix type
    "/path",                              # commanded path
    "/rpp/debug",                         # xtrack, heading_err, speed, κ, state, params
    "/rpp/segment_debug",                 # segment FSM (state, seg idx, corner angle)
    "/rpp/velocity_ned",                  # commanded velocity NED
    "/rpp/yaw_rate_body",                 # commanded yaw rate
    "/spray/active",                      # desired MARK (RPP)
    "/spray/state",                       # actual sprayer state (controller)
]


def log(msg: str) -> None:
    print(f"[bag_autorecord] {datetime.now().isoformat(timespec='seconds')} {msg}", flush=True)


def _token() -> str | None:
    if AUTH_OFF:
        return None
    try:
        with open(TOKEN_FILE) as f:
            return f.read().strip()
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


class Recorder:
    def __init__(self) -> None:
        self.proc: subprocess.Popen | None = None
        self.outdir: str | None = None
        self.start_t: float = 0.0

    @property
    def active(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self, path_name: str | None) -> None:
        os.makedirs(BAGS_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        name = f"{_safe_name(path_name)}_{stamp}"
        self.outdir = os.path.join(BAGS_DIR, name)
        cmd = ["ros2", "bag", "record", "-o", self.outdir]
        cmd += ["-a"] if RECORD_ALL else TOPICS
        log(f"START recording → {self.outdir}  ({'ALL topics' if RECORD_ALL else f'{len(TOPICS)} topics'})")
        # own process group so SIGINT targets the whole ros2 bag tree
        self.proc = subprocess.Popen(cmd, start_new_session=True,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        self.start_t = time.time()

    def stop(self, reason: str) -> None:
        if not self.active:
            self.proc = None
            return
        log(f"STOP recording ({reason}) → finalising {os.path.basename(self.outdir or '')}")
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
        log(f"  saved: {self.outdir}")
        self.proc = None
        self.outdir = None


def main() -> int:
    rec = Recorder()
    stop_flag = {"v": False}

    def _sig(_s, _f):
        stop_flag["v"] = True
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    log(f"watching {STATUS_URL}  bags→{BAGS_DIR}  auth={'off' if AUTH_OFF else 'on'}")
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
