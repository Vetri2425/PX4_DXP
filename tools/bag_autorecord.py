#!/usr/bin/env python3
"""Mission-scoped rosbag recorder and correlated debug-bundle finalizer."""
from __future__ import annotations

import datetime
import hashlib
import json
import os
import re
import shutil
import signal
import sqlite3
import subprocess
import sys
import time
import urllib.request
from pathlib import Path
from typing import Any

API_BASE = os.environ.get("ROVER_API_BASE", "http://127.0.0.1:5001").rstrip("/")
TOKEN_FILE = os.environ.get("ROVER_TOKEN_FILE", os.path.expanduser("~/.rover_token"))
AUTH_OFF = os.environ.get("ROVER_DISABLE_AUTH", "0") == "1"
REPO_DIR = Path(os.environ.get("PX4_DXP_DIR", Path(__file__).resolve().parents[1]))
BUNDLES_DIR = Path(os.environ.get("BAGS_DIR", os.path.expanduser("~/bags_jet")))
CONTROL_DIR = Path(os.environ.get("MISSION_DEBUG_CONTROL_DIR", REPO_DIR / "runtime" / "mission-debug"))
QOS_OVERRIDES = Path(os.environ.get("BAG_QOS_OVERRIDES", REPO_DIR / "config" / "rosbag_qos_overrides.yaml"))

POLL_S = float(os.environ.get("BAG_POLL_S", "0.1"))
READY_S = float(os.environ.get("BAG_READY_S", "0.5"))
POST_TAIL_S = float(os.environ.get("BAG_POST_TAIL_S", "2.0"))
MAX_S = float(os.environ.get("BAG_MAX_S", "1800"))
API_GRACE_S = float(os.environ.get("BAG_API_GRACE_S", "8"))
# A request older than this when the recorder first sees it is treated as stale
# (server/recorder crashed between request and ack) and rejected rather than
# replayed as a phantom recording. The server's ack timeout is ~3s, so any
# request this old is certainly abandoned.
REQUEST_TTL_S = float(os.environ.get("BAG_REQUEST_TTL_S", "120"))
# Walking the whole bundle tree is O(files); keep it off the 10 Hz hot loop.
DIRSIZE_THROTTLE_S = float(os.environ.get("BAG_DIRSIZE_THROTTLE_S", "5.0"))
MIN_FREE_BYTES = int(os.environ.get("BAG_MIN_FREE_BYTES", str(5 * 1024**3)))
LOW_FREE_BYTES = int(os.environ.get("BAG_LOW_FREE_BYTES", str(2 * 1024**3)))
MAX_TOTAL_BYTES = int(os.environ.get("BAG_MAX_TOTAL_BYTES", str(50 * 1024**3)))
JOURNAL_MAX_BYTES = int(os.environ.get("BAG_JOURNAL_MAX_BYTES", str(5 * 1024**2)))
FINALIZE_TIMEOUT_S = float(os.environ.get("BAG_FINALIZE_TIMEOUT_S", "15"))

TERMINAL = {"idle", "completed", "aborted", "error", "none", ""}
TOPICS = [
    "/path",
    "/path/identity",
    "/mavros/local_position/pose",
    "/mavros/local_position/velocity_local",
    "/mavros/setpoint_raw/local",
    "/mavros/state",
    "/mavros/statustext",
    "/mavros/imu/data",
    "/mavros/global_position/global",
    "/mavros/gpsstatus/gps1/raw",
    "/rpp/debug",
    "/rpp/segment_debug",
    "/rpp/velocity_ned",
    "/rpp/yaw_rate_body",
    "/rpp/conditioned_path",
    "/rpp/conditioned_path_identity",
    "/spray/active",
    "/spray/desired",
    "/spray/commanded",
    "/spray/state",
    "/spray/debug",
    "/spray/runtime_status",
]

SERVICES = ("rover-server", "rpp-pipeline", "px4-dxp", "bag-autorecord")
FCU_PARAM_IDS = (
    "COM_OF_LOSS_T",
    "COM_OBL_RC_ACT",
    "RO_YAW_P",
    "RO_YAW_RATE_LIM",
    "RD_TRANS_DRV_TRN",
    "RD_TRANS_TRN_DRV",
    "EKF2_WENC_CTRL",
    "RBCLW_COUNTS_REV",
    "PWM_AUX_MIN1",
    "PWM_AUX_MAX1",
    "PWM_AUX_DIS1",
)
_SECRET_PATTERNS = (
    re.compile(r"(?i)(authorization\s*[:=]\s*)(\S.*)"),
    re.compile(r"(?i)((?:token|password|passwd|secret)\s*[:=]\s*)([^\s,;]+)"),
    re.compile(r"(?i)(https?://)([^/@\s:]+):([^/@\s]+)@"),
    re.compile(r"(?i)(basic|bearer)\s+[A-Za-z0-9+/=_\-.]+"),
)


def utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def parse_utc(value: Any) -> datetime.datetime | None:
    """Parse an ISO-8601 'Z' timestamp; return None on anything unparseable."""
    if not isinstance(value, str) or not value:
        return None
    try:
        parsed = datetime.datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=datetime.timezone.utc)
    return parsed


def request_age_s(request: dict[str, Any]) -> float | None:
    """Seconds since the request was created, or None if its stamp is unusable."""
    stamped = parse_utc(request.get("requested_at_utc"))
    if stamped is None:
        return None
    return (datetime.datetime.now(datetime.timezone.utc) - stamped).total_seconds()


def log(message: str) -> None:
    print(f"[bag_autorecord] {utc_now()} {message}", flush=True)


def safe_component(value: Any, fallback: str = "mission", limit: int = 80) -> str:
    base = os.path.basename(str(value or fallback).strip())
    clean = re.sub(r"[^A-Za-z0-9._-]+", "_", base).strip("._") or fallback
    return clean[:limit]


def bundle_name(source: Any, mission_id: Any, when: datetime.datetime | None = None) -> str:
    now = when or datetime.datetime.now(datetime.timezone.utc)
    stamp = now.astimezone(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%f")[:19] + "Z"
    return f"{stamp}_{safe_component(source)}_{safe_component(mission_id, 'unknown')}"


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.chmod(tmp, 0o600)
    os.replace(tmp, path)


def read_json(path: Path) -> dict[str, Any]:
    with open(path, encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"JSON object required: {path}")
    return value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def redact(text: str, known_token: str | None = None) -> tuple[str, int]:
    count = 0
    if known_token:
        hits = text.count(known_token)
        if hits:
            text = text.replace(known_token, "[REDACTED]")
            count += hits
    for pattern in _SECRET_PATTERNS:
        def replacement(match):
            nonlocal count
            count += 1
            if pattern is _SECRET_PATTERNS[2]:
                return match.group(1) + "[REDACTED]@"
            if match.lastindex and match.lastindex >= 1 and pattern is not _SECRET_PATTERNS[3]:
                return match.group(1) + "[REDACTED]"
            return "[REDACTED]"
        text = pattern.sub(replacement, text)
    return text, count


def _token() -> str | None:
    if AUTH_OFF:
        return None
    try:
        return Path(TOKEN_FILE).read_text(encoding="utf-8").strip() or None
    except OSError:
        return None


def api_get(path: str) -> dict[str, Any] | list[Any]:
    request = urllib.request.Request(f"{API_BASE}{path}")
    token = _token()
    if token:
        request.add_header("X-Rover-Token", token)
    with urllib.request.urlopen(request, timeout=1.5) as response:
        return json.loads(response.read().decode("utf-8"))


def directory_size(path: Path) -> int:
    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += (Path(root) / name).stat().st_size
            except OSError:
                pass
    return total


def rotate_bundles() -> None:
    BUNDLES_DIR.mkdir(parents=True, exist_ok=True)
    candidates = []
    for child in BUNDLES_DIR.iterdir():
        manifest = child / "manifest.json"
        if not child.is_dir() or not manifest.is_file():
            continue
        try:
            data = read_json(manifest)
        except (OSError, ValueError, json.JSONDecodeError):
            continue
        if data.get("capture", {}).get("state") == "recording":
            continue
        candidates.append((child.stat().st_mtime, child, directory_size(child)))
    candidates.sort()
    total = directory_size(BUNDLES_DIR)
    newest = candidates[-1][1] if candidates else None
    for _, child, size in candidates:
        free = shutil.disk_usage(BUNDLES_DIR).free
        if total <= MAX_TOTAL_BYTES and free >= MIN_FREE_BYTES:
            break
        if child == newest:
            continue
        shutil.rmtree(child)
        total -= size
        log(f"retention removed {child.name}")


def check_preflight_space() -> None:
    rotate_bundles()
    free = shutil.disk_usage(BUNDLES_DIR).free
    if free < MIN_FREE_BYTES:
        raise RuntimeError(f"low disk before capture: {free} bytes free, {MIN_FREE_BYTES} required")
    used = directory_size(BUNDLES_DIR)
    if used >= MAX_TOTAL_BYTES:
        raise RuntimeError(
            f"bundle storage limit reached: {used} bytes used, limit {MAX_TOTAL_BYTES}"
        )


def reconcile_incomplete_bundles() -> int:
    """Mark captures abandoned by an ungraceful recorder/service exit."""
    reconciled = 0
    if not BUNDLES_DIR.exists():
        return reconciled
    for manifest_path in BUNDLES_DIR.glob("*/manifest.json"):
        try:
            manifest = read_json(manifest_path)
            state = manifest.get("capture", {}).get("state")
            integrity = manifest.get("outcome", {}).get("integrity")
            if state not in {"starting", "recording"} and integrity != "recording":
                continue
            manifest.setdefault("capture", {})["state"] = "abandoned"
            outcome = manifest.setdefault("outcome", {})
            outcome["integrity"] = "incomplete"
            outcome["stop_reason"] = "recorder_service_restart"
            outcome.setdefault("warnings", []).append(
                "capture was active when the recorder process restarted"
            )
            manifest.setdefault("timestamps", {})["recorder_end_utc"] = utc_now()
            atomic_json(manifest_path, manifest)
            (manifest_path.parent / "INCOMPLETE").write_text(
                "incomplete\n", encoding="ascii"
            )
            reconciled += 1
        except (OSError, ValueError, json.JSONDecodeError):
            continue
    return reconciled


def _run_text(command: list[str], timeout: float = 10.0) -> dict[str, Any]:
    try:
        result = subprocess.run(
            command,
            cwd=REPO_DIR,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
        )
        return {"command": command, "returncode": result.returncode,
                "stdout": result.stdout, "stderr": result.stderr}
    except Exception as exc:
        return {"command": command, "returncode": None, "error": str(exc)}


class CaptureSession:
    def __init__(self, request: dict[str, Any]) -> None:
        self.request = request
        self.capture_id = str(request["capture_id"])
        self.bundle: Path | None = None
        self.bag_dir: Path | None = None
        self.proc: subprocess.Popen | None = None
        self.bag_log = None
        self.started_wall = time.time()
        self.started_utc = utc_now()
        self.terminal_seen_at: float | None = None
        self.stop_reason: str | None = None
        self.api_fail_since: float | None = None
        self.mission_became_active = False
        self.manifest: dict[str, Any] = {}
        # Throttled cache for the expensive total-bundle-size guard.
        self._dirsize_checked_at: float | None = None
        self._dirsize_cached: int = 0

    def _control(self, group: str) -> Path:
        return CONTROL_DIR / group / f"{self.capture_id}.json"

    def _write_manifest(self) -> None:
        assert self.bundle is not None
        atomic_json(self.bundle / "manifest.json", self.manifest)

    def _cleanup_control_files(self) -> None:
        # Remove the request first so deleting its acknowledgement cannot make it
        # eligible for another recording pass.
        for group in (
            "requests", "placement", "start-results", "terminal", "cancelled", "acks"
        ):
            try:
                self._control(group).unlink()
            except FileNotFoundError:
                pass
            except OSError as exc:
                self.manifest["outcome"]["warnings"].append(
                    f"control cleanup failed for {group}: {exc}"
                )

    def prepare(self) -> None:
        check_preflight_space()
        if not QOS_OVERRIDES.is_file():
            raise RuntimeError(f"QoS override file missing: {QOS_OVERRIDES}")

        base = bundle_name(self.request.get("source_name"), self.request.get("loaded_mission_id"))
        bundle = BUNDLES_DIR / base
        try:
            bundle.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            bundle = BUNDLES_DIR / f"{base}_{self.capture_id[:8]}"
            bundle.mkdir(parents=True, exist_ok=False)
        self.bundle = bundle
        for name in ("rosbag", "server/journals", "mission/sidecars", "path_engine", "config", "system"):
            (bundle / name).mkdir(parents=True, exist_ok=True)
        self.bag_dir = bundle / "rosbag" / "bag"

        staged_info = {"path": None, "sha256": None, "copied_sha256": None}
        staged_raw = self.request.get("staged_artifact_path")
        if staged_raw:
            staged = Path(staged_raw).resolve()
            staging_root = (REPO_DIR / "server" / "missions" / "staging").resolve()
            if staging_root not in staged.parents or not staged.is_file():
                raise RuntimeError("staged artifact is missing or outside staging directory")
            target = bundle / "mission" / "staged.json"
            shutil.copyfile(staged, target)
            source_hash = sha256_file(staged)
            copied_hash = sha256_file(target)
            if source_hash != copied_hash:
                raise RuntimeError("staged artifact byte-copy verification failed")
            staged_info = {"path": str(staged), "sha256": source_hash,
                           "copied_sha256": copied_hash}

        shutil.copyfile(QOS_OVERRIDES, bundle / "config" / QOS_OVERRIDES.name)
        atomic_json(bundle / "system" / "capture_request.json", self.request)
        loaded = self.request.get("loaded_path") or {}
        self.manifest = {
            "schema_version": "mission-debug-bundle/v1",
            "capture_id": self.capture_id,
            "identity": {
                "source_filename": self.request.get("source_name"),
                "staged_mission_id": (
                    self.request.get("loaded_mission_id") if staged_raw else None
                ),
                "loaded_mission_id": self.request.get("loaded_mission_id"),
                "running_mission_id": None,
            },
            "timestamps": {
                "requested_utc": self.request.get("requested_at_utc"),
                "recorder_start_utc": self.started_utc,
                "terminal_utc": None,
                "recorder_end_utc": None,
            },
            "placement": {
                "placement_mode": self.request.get("placement_mode"),
                "origin_gps": self.request.get("origin_gps"),
                "rover_local_ned_at_resolution": None,
                "survey_translation_ned": None,
                "resolved_first_waypoint_ned": None,
                "published_first_waypoint_ned": None,
                "entry_transit_added": None,
                "entry_start_ned": None,
                "entry_target_ned": None,
                "entry_distance_m": None,
                "entry_target_original_spray_on": None,
                "final_local_pose_age_ms": None,
            },
            "mission": {
                "point_count": loaded.get("num_waypoints"),
                "source_point_count": loaded.get("num_waypoints"),
                "published_point_count": None,
                "spray_on_count": loaded.get("num_mark"),
                "spray_off_count": loaded.get("num_transit"),
                "staged_artifact": staged_info,
            },
            "software": {"git_commit": None, "git_dirty": None, "dirty_paths": []},
            "capture": {
                "state": "starting",
                "topics": list(TOPICS),
                "recorder_pid": os.getpid(),
                "rosbag_pid": None,
                "exit_code": None,
                "duration_s": None,
                "message_count": None,
                "recorded_topics": [],
                "missing_topics": [],
                "post_terminal_tail_s": POST_TAIL_S,
            },
            "outcome": {
                "start_result": None,
                "stop_reason": None,
                "final_mission_status": None,
                "integrity": "recording",
                "warnings": [],
            },
        }
        if staged_raw is None:
            self.manifest["outcome"]["warnings"].append("no staged artifact for legacy/local mission")
        self._write_manifest()

    def start_rosbag(self) -> None:
        assert self.bundle is not None and self.bag_dir is not None
        command = [
            "ros2", "bag", "record",
            "--qos-profile-overrides-path", str(QOS_OVERRIDES),
            "-o", str(self.bag_dir),
            *TOPICS,
        ]
        self.bag_log = open(self.bundle / "system" / "rosbag.log", "w", encoding="utf-8")
        self.proc = subprocess.Popen(
            command,
            start_new_session=True,
            stdout=self.bag_log,
            stderr=subprocess.STDOUT,
        )
        deadline = time.monotonic() + READY_S
        output_ready = False
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError(f"rosbag exited during readiness with code {self.proc.returncode}")
            output_ready = self.bag_dir.is_dir()
            time.sleep(0.05)
        if not output_ready and not self.bag_dir.is_dir():
            raise RuntimeError("rosbag process is alive but did not create its output directory")
        self.manifest["capture"].update({
            "state": "recording",
            "rosbag_pid": self.proc.pid,
        })
        self._write_manifest()

    def acknowledge(self) -> None:
        assert self.bundle is not None and self.proc is not None
        ack = {
            "capture_id": self.capture_id,
            "ready": True,
            "acknowledged_at_utc": utc_now(),
            "bundle_path": str(self.bundle),
            "recorder_pid": os.getpid(),
            "rosbag_pid": self.proc.pid,
            "topics": TOPICS,
        }
        atomic_json(self._control("acks"), ack)
        atomic_json(self.bundle / "system" / "capture_ack.json", ack)
        self._snapshot_start()

    def reject(self, error: Exception) -> None:
        if self.bag_log is not None:
            self.bag_log.close()
            self.bag_log = None
        if self.bundle is not None:
            if not self.manifest:
                self.manifest = {
                    "schema_version": "mission-debug-bundle/v1",
                    "capture_id": self.capture_id,
                    "capture": {"state": "failed_preflight"},
                    "outcome": {
                        "integrity": "failed_preflight",
                        "stop_reason": "failed_preflight",
                        "warnings": [str(error)],
                    },
                }
            else:
                self.manifest["capture"]["state"] = "failed_preflight"
                self.manifest["outcome"].update({
                    "integrity": "failed_preflight",
                    "stop_reason": "failed_preflight",
                })
                self.manifest["outcome"]["warnings"].append(str(error))
            self._write_manifest()
        atomic_json(self._control("acks"), {
            "capture_id": self.capture_id,
            "ready": False,
            "acknowledged_at_utc": utc_now(),
            "error": str(error),
        })
        atomic_json(CONTROL_DIR / "status.json", {
            "state": "failed_preflight", "capture_id": self.capture_id,
            "error": str(error), "updated_at_utc": utc_now(),
        })

    def _snapshot_start(self) -> None:
        assert self.bundle is not None
        try:
            loaded = api_get("/api/mission/loaded-path")
            atomic_json(self.bundle / "mission" / "loaded_path.json", loaded)
        except Exception as exc:
            self.manifest["outcome"]["warnings"].append(f"loaded-path snapshot failed: {exc}")
        try:
            activity = api_get("/api/activity")
            atomic_json(self.bundle / "server" / "activity_start.json", {"entries": activity})
        except Exception as exc:
            self.manifest["outcome"]["warnings"].append(f"activity start snapshot failed: {exc}")

        commit = _run_text(["git", "rev-parse", "HEAD"])
        dirty = _run_text(["git", "status", "--porcelain=v1", "--untracked-files=no"])
        dirty_lines = dirty.get("stdout", "").splitlines()
        self.manifest["software"] = {
            "git_commit": commit.get("stdout", "").strip() or None,
            "git_dirty": bool(dirty_lines),
            "dirty_paths": dirty_lines,
        }
        atomic_json(self.bundle / "system" / "git.json", {
            "commit": commit, "status": dirty,
        })

        services = {}
        properties = "Id,ActiveState,SubState,MainPID,NRestarts,ExecMainStartTimestamp"
        for service in SERVICES:
            services[service] = _run_text([
                "systemctl", "show", f"{service}.service", f"--property={properties}"
            ])
        atomic_json(self.bundle / "system" / "services.json", services)
        atomic_json(self.bundle / "system" / "versions.json", {
            "uname": _run_text(["uname", "-a"]),
            "packages": _run_text([
                "dpkg-query", "-W", "ros-humble-rosbag2", "ros-humble-mavros"
            ]),
        })

        for node, filename in (
            ("/rpp_controller", "rpp_controller.yaml"),
            ("/spray_controller", "spray_controller.yaml"),
            ("/mavros", "mavros.yaml"),
        ):
            result = _run_text(["ros2", "param", "dump", node])
            output = result.get("stdout", "")
            if result.get("returncode") == 0 and output:
                (self.bundle / "config" / filename).write_text(output, encoding="utf-8")
            else:
                self.manifest["outcome"]["warnings"].append(f"parameter dump failed for {node}")
        self._snapshot_fcu_params()
        self._copy_sidecars()
        self._write_manifest()

    def _snapshot_fcu_params(self) -> None:
        assert self.bundle is not None
        evidence: dict[str, Any] = {
            "pulled_at_utc": utc_now(),
            "param_pull": _run_text([
                "ros2", "service", "call", "/mavros/param/pull",
                "mavros_msgs/srv/ParamPull", "{force_pull: true}",
            ], timeout=8.0),
            "params": {},
        }
        for param_id in FCU_PARAM_IDS:
            evidence["params"][param_id] = _run_text([
                "ros2", "service", "call", "/mavros/param/get",
                "mavros_msgs/srv/ParamGet", f"{{param_id: {param_id}}}",
            ], timeout=4.0)
        atomic_json(self.bundle / "config" / "fcu_params.json", evidence)

    def _copy_sidecars(self) -> None:
        assert self.bundle is not None
        source = safe_component(self.request.get("source_name"))
        missions = REPO_DIR / "server" / "missions"
        for suffix in ("entities.json", "extensions.json", "entity_order.json"):
            path = missions / f".{source}.{suffix}"
            if path.is_file():
                shutil.copyfile(path, self.bundle / "mission" / "sidecars" / path.name)
        source_path = missions / source
        if source_path.is_file():
            atomic_json(self.bundle / "mission" / "source.json", {
                "filename": source,
                "size_bytes": source_path.stat().st_size,
                "sha256": sha256_file(source_path),
            })

    def update_from_control(self) -> None:
        assert self.bundle is not None
        placement_path = self._control("placement")
        if placement_path.is_file() and not (self.bundle / "path_engine" / "placement.json").exists():
            placement = read_json(placement_path)
            atomic_json(self.bundle / "path_engine" / "placement.json", placement)
            for key in self.manifest["placement"]:
                if key in placement:
                    self.manifest["placement"][key] = placement[key]
            for key in (
                "point_count", "source_point_count", "published_point_count",
                "spray_on_count", "spray_off_count",
            ):
                if key in placement:
                    self.manifest["mission"][key] = placement[key]

        result_path = self._control("start-results")
        if result_path.is_file() and not (self.bundle / "server" / "start_result.json").exists():
            result = read_json(result_path)
            atomic_json(self.bundle / "server" / "start_result.json", result)
            self.manifest["outcome"]["start_result"] = result
            if result.get("success"):
                self.mission_became_active = True
                self.manifest["identity"]["running_mission_id"] = self.request.get("loaded_mission_id")

        terminal_path = self._control("terminal")
        if terminal_path.is_file() and self.terminal_seen_at is None:
            terminal = read_json(terminal_path)
            self.stop_reason = str(terminal.get("reason") or "terminal_event")
            self.terminal_seen_at = time.monotonic()
            self.manifest["timestamps"]["terminal_utc"] = terminal.get("terminal_at_utc")
            atomic_json(self.bundle / "server" / "terminal.json", terminal)
        cancelled_path = self._control("cancelled")
        if cancelled_path.is_file() and self.terminal_seen_at is None:
            cancellation = read_json(cancelled_path)
            self.stop_reason = "capture_request_cancelled"
            self.terminal_seen_at = time.monotonic() - POST_TAIL_S
            self.manifest["timestamps"]["terminal_utc"] = cancellation.get("cancelled_at_utc")
            atomic_json(self.bundle / "server" / "cancellation.json", cancellation)
        self._write_manifest()

    def poll_mission(self) -> None:
        try:
            status = api_get("/api/mission/status")
            if not isinstance(status, dict):
                raise ValueError("invalid mission status response")
            self.api_fail_since = None
            state = str(status.get("state", "")).split(".")[-1].lower()
            if state not in TERMINAL:
                self.mission_became_active = True
            elif self.mission_became_active and self.terminal_seen_at is None:
                self.stop_reason = f"mission_{state or 'terminal'}"
                self.terminal_seen_at = time.monotonic()
                self.manifest["timestamps"]["terminal_utc"] = utc_now()
        except Exception:
            self.api_fail_since = self.api_fail_since or time.monotonic()
            if time.monotonic() - self.api_fail_since >= API_GRACE_S and self.terminal_seen_at is None:
                self.stop_reason = "api_unreachable"
                self.terminal_seen_at = time.monotonic()
                self.manifest["timestamps"]["terminal_utc"] = utc_now()

    def _throttled_total_bytes(self) -> int:
        """Total bundle-tree size, recomputed at most every DIRSIZE_THROTTLE_S.

        ``directory_size`` walks every historical bundle plus the growing active
        bag; running it on each 10 Hz poll is real, sustained I/O on the Jetson.
        Free-space (``disk_usage``) is cheap and stays per-tick; this soft cap is
        sampled instead.
        """
        now = time.monotonic()
        if (
            self._dirsize_checked_at is None
            or now - self._dirsize_checked_at >= DIRSIZE_THROTTLE_S
        ):
            self._dirsize_cached = directory_size(BUNDLES_DIR)
            self._dirsize_checked_at = now
        return self._dirsize_cached

    def should_stop(self) -> bool:
        assert self.proc is not None
        if self.proc.poll() is not None:
            self.stop_reason = "recorder_exit"
            return True
        if shutil.disk_usage(BUNDLES_DIR).free < LOW_FREE_BYTES:
            self.stop_reason = "disk_low_during_recording"
            return True
        if self._throttled_total_bytes() >= MAX_TOTAL_BYTES:
            self.stop_reason = "storage_limit_during_recording"
            return True
        if time.time() - self.started_wall >= MAX_S:
            self.stop_reason = "max_duration_cap"
            return True
        return self.terminal_seen_at is not None and (
            time.monotonic() - self.terminal_seen_at >= POST_TAIL_S
        )

    def _signal_group(self, sig: int) -> bool:
        """Signal the rosbag process group; tolerate it having already exited.

        Returns False if the group is already gone (ProcessLookupError) so the
        caller can stop escalating instead of crashing finalization.
        """
        assert self.proc is not None
        try:
            os.killpg(os.getpgid(self.proc.pid), sig)
            return True
        except ProcessLookupError:
            return False

    def stop_and_finalize(self, reason: str | None = None) -> None:
        assert self.bundle is not None and self.proc is not None
        self.stop_reason = reason or self.stop_reason or "unknown"
        if self.proc.poll() is None:
            try:
                if self._signal_group(signal.SIGINT):
                    self.proc.wait(timeout=FINALIZE_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                self.manifest["outcome"]["warnings"].append("SIGINT finalization timed out; sent SIGTERM")
                if self._signal_group(signal.SIGTERM):
                    try:
                        self.proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        if self._signal_group(signal.SIGKILL):
                            self.proc.wait(timeout=5)
        exit_code = self.proc.returncode
        if self.bag_log:
            self.bag_log.close()
            self.bag_log = None

        self._snapshot_final()
        stats = self._bag_stats()
        missing = [topic for topic in TOPICS if topic not in stats["recorded_topics"]]
        metadata_ok = self.bag_dir is not None and (self.bag_dir / "metadata.yaml").is_file()
        if self.stop_reason in {
            "recorder_exit", "disk_low_during_recording",
            "storage_limit_during_recording", "capture_request_cancelled",
        } or not metadata_ok or stats["message_count"] == 0:
            integrity = "incomplete"
        elif missing:
            integrity = "partial"
        else:
            integrity = "complete"

        self.manifest["timestamps"]["recorder_end_utc"] = utc_now()
        self.manifest["capture"].update({
            "state": "finalized",
            "exit_code": exit_code,
            "duration_s": stats["duration_s"],
            "message_count": stats["message_count"],
            "recorded_topics": stats["recorded_topics"],
            "missing_topics": missing,
        })
        self.manifest["outcome"].update({
            "stop_reason": self.stop_reason,
            "integrity": integrity,
        })
        if not metadata_ok:
            self.manifest["outcome"]["warnings"].append("rosbag metadata.yaml missing")
        if missing:
            self.manifest["outcome"]["warnings"].append(
                "mandatory topics absent: " + ", ".join(missing)
            )
        self._write_manifest()
        if integrity != "complete":
            (self.bundle / "INCOMPLETE").write_text(integrity + "\n", encoding="ascii")
        atomic_json(CONTROL_DIR / "status.json", {
            "state": "finalized",
            "capture_id": self.capture_id,
            "bundle_path": str(self.bundle),
            "integrity": integrity,
            "stop_reason": self.stop_reason,
            "updated_at_utc": utc_now(),
        })
        self._cleanup_control_files()
        self._write_manifest()
        log(f"finalized {self.bundle.name}: {integrity} ({self.stop_reason})")

    def _snapshot_final(self) -> None:
        assert self.bundle is not None
        end_utc = utc_now()
        try:
            final_status = api_get("/api/mission/status")
            atomic_json(self.bundle / "server" / "final_status.json", final_status)
            self.manifest["outcome"]["final_mission_status"] = final_status
        except Exception as exc:
            self.manifest["outcome"]["warnings"].append(f"final status snapshot failed: {exc}")
        try:
            activity = api_get("/api/activity")
            atomic_json(self.bundle / "server" / "activity_final.json", {"entries": activity})
        except Exception as exc:
            self.manifest["outcome"]["warnings"].append(f"activity final snapshot failed: {exc}")
        self._export_journals(self.started_utc, end_utc)

    def _export_journals(self, start_utc: str, end_utc: str) -> None:
        assert self.bundle is not None
        token = _token()
        for service in SERVICES:
            command = [
                "journalctl", "--utc", "--no-pager", "--output=short-iso-precise",
                "--since", start_utc, "--until", end_utc, "-u", f"{service}.service",
            ]
            try:
                proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                chunks: list[str] = []
                size = 0
                redactions = 0
                truncated = False
                assert proc.stdout is not None
                for line in proc.stdout:
                    clean, count = redact(line, token)
                    encoded = clean.encode("utf-8", errors="replace")
                    if size + len(encoded) > JOURNAL_MAX_BYTES:
                        truncated = True
                        proc.terminate()
                        break
                    chunks.append(clean)
                    size += len(encoded)
                    redactions += count
                proc.wait(timeout=5)
                if truncated:
                    chunks.append("\n[TRUNCATED BY MISSION DEBUG CAPTURE]\n")
                    self.manifest["outcome"]["warnings"].append(f"journal truncated: {service}")
                (self.bundle / "server" / "journals" / f"{service}.log").write_text(
                    "".join(chunks), encoding="utf-8"
                )
                if redactions:
                    self.manifest["outcome"]["warnings"].append(
                        f"journal redactions applied: {service} ({redactions})"
                    )
            except Exception as exc:
                self.manifest["outcome"]["warnings"].append(f"journal export failed for {service}: {exc}")

    def _bag_stats(self) -> dict[str, Any]:
        assert self.bag_dir is not None
        topics: set[str] = set()
        count = 0
        first_ns = None
        last_ns = None
        for database in self.bag_dir.glob("*.db3"):
            try:
                connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
                rows = connection.execute("SELECT name FROM topics").fetchall()
                topics.update(row[0] for row in rows)
                row = connection.execute("SELECT COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages").fetchone()
                connection.close()
                count += int(row[0] or 0)
                if row[1] is not None:
                    first_ns = row[1] if first_ns is None else min(first_ns, row[1])
                    last_ns = row[2] if last_ns is None else max(last_ns, row[2])
            except (OSError, sqlite3.Error) as exc:
                self.manifest["outcome"]["warnings"].append(f"bag database inspection failed: {exc}")
        duration = ((last_ns - first_ns) / 1e9) if first_ns is not None and last_ns is not None else 0.0
        return {"recorded_topics": sorted(topics), "message_count": count, "duration_s": duration}


class RecorderDaemon:
    def __init__(self) -> None:
        self.session: CaptureSession | None = None
        self.stop_requested = False

    def request_stop(self, _signum=None, _frame=None) -> None:
        self.stop_requested = True

    @staticmethod
    def _purge_control(capture_id: str) -> None:
        """Remove request/ack/cancelled handshake files for a dead capture id.

        Used for cancelled-before-ack and stale requests whose server is no
        longer listening, so abandoned handshakes do not accumulate forever
        under the control directory.
        """
        for group in ("requests", "cancelled", "acks"):
            try:
                (CONTROL_DIR / group / f"{capture_id}.json").unlink()
            except FileNotFoundError:
                pass
            except OSError as exc:
                log(f"control purge failed for {group}/{capture_id}: {exc}")

    def _next_request(self) -> dict[str, Any] | None:
        request_dir = CONTROL_DIR / "requests"
        request_dir.mkdir(parents=True, exist_ok=True)
        for path in sorted(request_dir.glob("*.json"), key=lambda item: item.stat().st_mtime):
            capture_id = path.stem
            if (CONTROL_DIR / "acks" / f"{capture_id}.json").exists():
                continue
            if (CONTROL_DIR / "cancelled" / f"{capture_id}.json").exists():
                # Server gave up before readiness (e.g. ack timeout) and is no
                # longer waiting; drop the whole handshake instead of replaying.
                self._purge_control(capture_id)
                log(f"dropped cancelled capture request {capture_id}")
                continue
            try:
                request = read_json(path)
                if request.get("protocol_version") != 1 or request.get("capture_id") != capture_id:
                    raise ValueError("invalid capture request identity/protocol")
            except Exception as exc:
                atomic_json(CONTROL_DIR / "acks" / f"{capture_id}.json", {
                    "capture_id": capture_id, "ready": False, "error": str(exc),
                    "acknowledged_at_utc": utc_now(),
                })
                continue
            age = request_age_s(request)
            if age is not None and age > REQUEST_TTL_S:
                # Request outlived its server (crash between write and ack). Do
                # not start a phantom recording for a mission that is long over.
                self._purge_control(capture_id)
                log(f"rejected stale capture request {capture_id} (age {age:.0f}s)")
                continue
            return request
        return None

    def run(self) -> int:
        BUNDLES_DIR.mkdir(parents=True, exist_ok=True)
        CONTROL_DIR.mkdir(parents=True, exist_ok=True)
        reconciled = reconcile_incomplete_bundles()
        if reconciled:
            log(f"marked {reconciled} interrupted capture(s) incomplete")
        atomic_json(CONTROL_DIR / "status.json", {
            "state": "idle", "recorder_pid": os.getpid(), "updated_at_utc": utc_now(),
        })
        log(f"watching {CONTROL_DIR} bundles->{BUNDLES_DIR}")
        while not self.stop_requested:
            if self.session is None:
                request = self._next_request()
                if request is not None:
                    session = CaptureSession(request)
                    try:
                        session.prepare()
                        session.start_rosbag()
                        if session._control("cancelled").is_file():
                            session.stop_and_finalize("capture_request_cancelled")
                            raise RuntimeError("capture request cancelled during recorder startup")
                        session.acknowledge()
                        self.session = session
                        atomic_json(CONTROL_DIR / "status.json", {
                            "state": "recording", "capture_id": session.capture_id,
                            "bundle_path": str(session.bundle), "updated_at_utc": utc_now(),
                        })
                        log(f"recording {session.bundle.name if session.bundle else session.capture_id}")
                    except Exception as exc:
                        log(f"capture rejected: {exc}")
                        if session.proc is not None and session.proc.poll() is None:
                            session.stop_and_finalize("startup_failure")
                        session.reject(exc)
                time.sleep(POLL_S)
                continue

            self.session.update_from_control()
            self.session.poll_mission()
            if self.session.should_stop():
                self.session.stop_and_finalize()
                self.session = None
            time.sleep(POLL_S)

        if self.session is not None:
            if self.session.terminal_seen_at is None:
                self.session.terminal_seen_at = time.monotonic()
                time.sleep(max(0.0, POST_TAIL_S))
            self.session.stop_and_finalize("service_shutdown")
        atomic_json(CONTROL_DIR / "status.json", {
            "state": "stopped", "updated_at_utc": utc_now(),
        })
        return 0


def main() -> int:
    daemon = RecorderDaemon()
    signal.signal(signal.SIGTERM, daemon.request_stop)
    signal.signal(signal.SIGINT, daemon.request_stop)
    return daemon.run()


if __name__ == "__main__":
    sys.exit(main())
