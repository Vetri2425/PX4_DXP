import asyncio
import datetime
import json
import os
import sqlite3
import sys
import time
from collections import deque, namedtuple
from pathlib import Path

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import bag_autorecord as recorder
import offboard_controller as offboard_module
from config import RPP_TRACKING
from mission_debug_capture import (
    CaptureUnavailable,
    MissionDebugCoordinator,
    _atomic_json as atomic_json,
)
from offboard_controller import OffboardController


class SummaryController:
    def loaded_path_summary(self):
        return {
            "name": "field.dxf",
            "source_name": "field.dxf",
            "mission_id": "stg_abc",
            "placement_mode": "GPS_SURVEYED",
            "origin_gps": [13.0, 80.0],
            "is_staged": True,
            "num_waypoints": 2,
            "num_mark": 1,
            "num_transit": 1,
        }


@pytest.mark.anyio
async def test_atomic_request_ack_handshake(tmp_path):
    coordinator = MissionDebugCoordinator(str(tmp_path), required=True, ack_timeout_s=1.0)

    async def acknowledge():
        request_dir = tmp_path / "requests"
        while not list(request_dir.glob("*.json")):
            await asyncio.sleep(0.01)
        request = json.loads(next(request_dir.glob("*.json")).read_text())
        atomic_json(tmp_path / "acks" / f"{request['capture_id']}.json", {
            "capture_id": request["capture_id"],
            "ready": True,
            "bundle_path": "/tmp/bundle",
        })

    task = asyncio.create_task(acknowledge())
    capture_id = await coordinator.begin_capture(
        SummaryController(), start_request={"mission_id": "stg_abc"}, transport="test"
    )
    await task
    assert capture_id
    assert coordinator.get_status()["state"] == "ready"


@pytest.mark.anyio
async def test_required_timeout_fails_and_optional_timeout_continues(tmp_path):
    required = MissionDebugCoordinator(str(tmp_path / "required"), required=True, ack_timeout_s=0.05)
    with pytest.raises(CaptureUnavailable, match="did not acknowledge"):
        await required.begin_capture(SummaryController(), start_request={}, transport="test")

    optional = MissionDebugCoordinator(str(tmp_path / "optional"), required=False, ack_timeout_s=0.05)
    assert await optional.begin_capture(SummaryController(), start_request={}, transport="test") is None


def _configure_recorder(monkeypatch, tmp_path):
    repo = tmp_path / "repo"
    staging = repo / "server" / "missions" / "staging"
    staging.mkdir(parents=True)
    bundles = tmp_path / "bundles"
    bundles.mkdir()
    control = repo / "runtime" / "mission-debug"
    qos = repo / "config" / "rosbag_qos_overrides.yaml"
    qos.parent.mkdir(parents=True)
    qos.write_text("/path:\n  reliability: reliable\n", encoding="utf-8")
    monkeypatch.setattr(recorder, "REPO_DIR", repo)
    monkeypatch.setattr(recorder, "BUNDLES_DIR", bundles)
    monkeypatch.setattr(recorder, "CONTROL_DIR", control)
    monkeypatch.setattr(recorder, "QOS_OVERRIDES", qos)
    monkeypatch.setattr(recorder, "MIN_FREE_BYTES", 0)
    monkeypatch.setattr(recorder, "MAX_TOTAL_BYTES", 10**12)
    return repo, staging, bundles


def test_bundle_naming_is_utc_source_and_mission():
    when = datetime.datetime(2026, 6, 19, 12, 34, 56, 789000,
                             tzinfo=datetime.timezone.utc)
    assert recorder.bundle_name("../Field Name.dxf", "stg:abc", when) == (
        "20260619T123456.789Z_Field_Name.dxf_stg_abc"
    )


def test_staged_json_is_copied_byte_for_byte(monkeypatch, tmp_path):
    _, staging, _ = _configure_recorder(monkeypatch, tmp_path)
    staged = staging / "stg_abc.json"
    raw = b'{  "mission_id" : "stg_abc",\n"waypoints":[[0,0],[1,0]]}\n'
    staged.write_bytes(raw)
    session = recorder.CaptureSession({
        "capture_id": "capture-1",
        "requested_at_utc": "2026-06-19T12:00:00.000Z",
        "source_name": "field.dxf",
        "loaded_mission_id": "stg_abc",
        "placement_mode": "GPS_SURVEYED",
        "origin_gps": [13.0, 80.0],
        "staged_artifact_path": str(staged),
        "loaded_path": {"num_waypoints": 2, "num_mark": 1, "num_transit": 1},
    })
    session.prepare()
    assert (session.bundle / "mission" / "staged.json").read_bytes() == raw
    assert session.manifest["mission"]["staged_artifact"]["sha256"] == recorder.sha256_file(staged)


def test_disk_low_preflight_refuses(monkeypatch, tmp_path):
    _configure_recorder(monkeypatch, tmp_path)
    usage = namedtuple("usage", "total used free")
    monkeypatch.setattr(recorder, "MIN_FREE_BYTES", 100)
    monkeypatch.setattr(recorder.shutil, "disk_usage", lambda _path: usage(1000, 999, 1))
    with pytest.raises(RuntimeError, match="low disk before capture"):
        recorder.check_preflight_space()


def test_bag_stats_reports_topics_messages_and_duration(monkeypatch, tmp_path):
    _configure_recorder(monkeypatch, tmp_path)
    session = recorder.CaptureSession({"capture_id": "capture-2"})
    session.bag_dir = tmp_path / "bag"
    session.bag_dir.mkdir()
    database = session.bag_dir / "bag_0.db3"
    connection = sqlite3.connect(database)
    connection.executescript(
        "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT);"
        "CREATE TABLE messages(timestamp INTEGER);"
        "INSERT INTO topics VALUES(1, '/path');"
        "INSERT INTO messages VALUES(1000000000);"
        "INSERT INTO messages VALUES(3000000000);"
    )
    connection.commit()
    connection.close()
    stats = session._bag_stats()
    assert stats == {
        "recorded_topics": ["/path"], "message_count": 2, "duration_s": 2.0
    }


def test_service_restart_marks_recording_bundle_incomplete(monkeypatch, tmp_path):
    _, _, bundles = _configure_recorder(monkeypatch, tmp_path)
    bundle = bundles / "interrupted"
    bundle.mkdir()
    recorder.atomic_json(bundle / "manifest.json", {
        "capture_id": "capture-3",
        "capture": {"state": "recording"},
        "outcome": {"integrity": "recording", "warnings": []},
        "timestamps": {},
    })
    assert recorder.reconcile_incomplete_bundles() == 1
    manifest = recorder.read_json(bundle / "manifest.json")
    assert manifest["capture"]["state"] == "abandoned"
    assert manifest["outcome"]["integrity"] == "incomplete"
    assert manifest["outcome"]["stop_reason"] == "recorder_service_restart"
    assert (bundle / "INCOMPLETE").is_file()


def test_redaction_removes_tokens_and_url_credentials():
    text, count = recorder.redact(
        "Authorization: Bearer abc\nurl=https://user:pass@example.test token=secret\n",
        "abc",
    )
    assert count >= 3
    assert "abc" not in text and "pass" not in text and "secret" not in text


def test_post_terminal_tail_and_recorder_exit_detection(monkeypatch, tmp_path):
    _configure_recorder(monkeypatch, tmp_path)

    class Proc:
        returncode = None

        def poll(self):
            return self.returncode

    session = recorder.CaptureSession({"capture_id": "capture-4"})
    session.proc = Proc()
    session.started_wall = time.time()
    session.terminal_seen_at = time.monotonic()
    monkeypatch.setattr(recorder, "LOW_FREE_BYTES", 0)
    monkeypatch.setattr(recorder, "MAX_TOTAL_BYTES", 10**12)
    assert session.should_stop() is False
    session.terminal_seen_at -= recorder.POST_TAIL_S + 0.01
    assert session.should_stop() is True

    session.terminal_seen_at = None
    session.proc.returncode = 7
    assert session.should_stop() is True
    assert session.stop_reason == "recorder_exit"


class HookNode:
    def __init__(self, events):
        self.events = events

    def get_state(self):
        return {"connected": True, "rpp_state": RPP_TRACKING,
                "pose_received": True, "pos_n": 2.0, "pos_e": 3.0}

    def get_rpp_monitor(self):
        return type("Monitor", (), {"reset": lambda self: None})()

    def publish_path(self, points, frame_id="local_ned", spray_flags=None):
        self.events.append(("publish", list(points)))

    async def arm_async(self, arm):
        return True, ""

    async def set_mode_async(self, mode):
        return True, ""


def test_placement_hook_runs_immediately_before_path_publish():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        events = []
        ctrl = OffboardController(HookNode(events), deque())
        ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="local.csv")
        ok, _ = asyncio.run(ctrl.start_async(
            pre_publish_hook=lambda placement: events.append(("placement", placement))
        ))
        assert ok
        assert [event[0] for event in events] == ["placement", "publish"]
        assert events[0][1]["resolved_first_waypoint_ned"] == [1.0, 2.0]
        assert events[0][1]["rover_local_ned_at_resolution"] == [2.0, 3.0]
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_pre_publish_hook_failure_never_aborts_start():
    # B1: a capture-sidecar failure (e.g. full disk) inside the hook must be
    # swallowed so the mission still arms/publishes.
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        events = []
        ctrl = OffboardController(HookNode(events), deque())
        ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="local.csv")

        def boom(_placement):
            raise OSError("No space left on device")

        ok, _ = asyncio.run(ctrl.start_async(pre_publish_hook=boom))
        assert ok
        # publish_path still happened despite the hook raising.
        assert [event[0] for event in events] == ["publish"]
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_capture_required_code_default_is_off():
    # B2: code default must be fail-open; the systemd unit enables it in field.
    import importlib

    import config as config_module

    monkey_env = {k: v for k, v in os.environ.items()}
    monkey_env.pop("MISSION_CAPTURE_REQUIRED", None)
    saved = os.environ.get("MISSION_CAPTURE_REQUIRED")
    try:
        os.environ.pop("MISSION_CAPTURE_REQUIRED", None)
        importlib.reload(config_module)
        assert config_module.MISSION_CAPTURE_REQUIRED is False
    finally:
        if saved is not None:
            os.environ["MISSION_CAPTURE_REQUIRED"] = saved
        importlib.reload(config_module)


def test_record_terminal_clears_active_capture_id(tmp_path):
    # N2: active id is cleared after a terminal so a later idle event cannot be
    # mis-attributed or orphan-written against a finalized capture.
    coordinator = MissionDebugCoordinator(str(tmp_path), required=False)
    coordinator.active_capture_id = "cap-active"
    coordinator.record_terminal(None, "mission_completed", state="COMPLETED")
    assert coordinator.active_capture_id is None
    assert (tmp_path / "terminal" / "cap-active.json").is_file()
    # A second terminal with no active id is a no-op, not a crash.
    coordinator.record_terminal(None, "operator_stop", state="IDLE")


def test_record_methods_swallow_io_errors(tmp_path, monkeypatch):
    # B1 (coordinator side): record_* must never raise into mission control flow.
    import mission_debug_capture as mdc

    coordinator = mdc.MissionDebugCoordinator(str(tmp_path), required=False)

    def boom(*_a, **_k):
        raise OSError("disk full")

    monkeypatch.setattr(mdc, "_atomic_json", boom)
    coordinator.record_placement("cap", {"x": 1})
    coordinator.record_start_result("cap", success=True, state="RUNNING", message="ok")
    coordinator.record_terminal("cap", "mission_completed", state="COMPLETED")


def test_throttled_total_bytes_recomputes_at_most_every_window(monkeypatch, tmp_path):
    # N1: full-tree directory_size is sampled, not run on every 10 Hz poll.
    _configure_recorder(monkeypatch, tmp_path)
    calls = {"n": 0}

    def counting_size(_path):
        calls["n"] += 1
        return 123

    monkeypatch.setattr(recorder, "directory_size", counting_size)
    session = recorder.CaptureSession({"capture_id": "cap-throttle"})
    assert session._throttled_total_bytes() == 123
    assert session._throttled_total_bytes() == 123
    assert calls["n"] == 1  # second call served from cache
    # Age the cache past the throttle window → one more recompute.
    session._dirsize_checked_at -= recorder.DIRSIZE_THROTTLE_S + 0.1
    assert session._throttled_total_bytes() == 123
    assert calls["n"] == 2


def test_signal_group_tolerates_already_exited(monkeypatch, tmp_path):
    # fix 6: ProcessLookupError during shutdown must not crash finalization.
    _configure_recorder(monkeypatch, tmp_path)
    session = recorder.CaptureSession({"capture_id": "cap-sig"})

    class Proc:
        pid = 4242
        returncode = None

    session.proc = Proc()
    monkeypatch.setattr(recorder.os, "getpgid", lambda _pid: 4242)

    def gone(_pgid, _sig):
        raise ProcessLookupError

    monkeypatch.setattr(recorder.os, "killpg", gone)
    assert session._signal_group(recorder.signal.SIGINT) is False

    monkeypatch.setattr(recorder.os, "killpg", lambda _pgid, _sig: None)
    assert session._signal_group(recorder.signal.SIGTERM) is True


def test_cancelled_request_is_purged(monkeypatch, tmp_path):
    # fix 5: cancelled-before-ack handshakes are dropped, not replayed/leaked.
    _configure_recorder(monkeypatch, tmp_path)
    capture_id = "cap-cancelled"
    request_path = recorder.CONTROL_DIR / "requests" / f"{capture_id}.json"
    recorder.atomic_json(request_path, {
        "protocol_version": 1, "capture_id": capture_id,
        "requested_at_utc": recorder.utc_now(),
    })
    recorder.atomic_json(recorder.CONTROL_DIR / "cancelled" / f"{capture_id}.json", {
        "capture_id": capture_id, "reason": "ack timeout",
    })
    daemon = recorder.RecorderDaemon()
    assert daemon._next_request() is None
    assert not request_path.exists()
    assert not (recorder.CONTROL_DIR / "cancelled" / f"{capture_id}.json").exists()


def test_stale_request_is_rejected_fresh_is_returned(monkeypatch, tmp_path):
    # fix 7: a request that outlived its server (crash) is dropped, not run as a
    # phantom recording; a fresh request is still returned.
    _configure_recorder(monkeypatch, tmp_path)
    stale_id = "cap-stale"
    stale_path = recorder.CONTROL_DIR / "requests" / f"{stale_id}.json"
    recorder.atomic_json(stale_path, {
        "protocol_version": 1, "capture_id": stale_id,
        "requested_at_utc": "2000-01-01T00:00:00.000Z",
    })
    daemon = recorder.RecorderDaemon()
    assert daemon._next_request() is None
    assert not stale_path.exists()

    fresh_id = "cap-fresh"
    recorder.atomic_json(recorder.CONTROL_DIR / "requests" / f"{fresh_id}.json", {
        "protocol_version": 1, "capture_id": fresh_id,
        "requested_at_utc": recorder.utc_now(),
    })
    nxt = daemon._next_request()
    assert nxt is not None and nxt["capture_id"] == fresh_id
