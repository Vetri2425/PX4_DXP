"""Unit tests for RTK stream health judgment (reconnect-anchor grace).

The child's last_frame_time survives reconnects, so health must be judged on
the newest of (last frame, this connection's handshake) or every fresh
reconnect after an outage longer than the health window reads as unhealthy
before it can possibly stream.
"""

from __future__ import annotations

import json
import os
import sys
import types
from pathlib import Path
import asyncio

sys.path.insert(0, os.path.dirname(__file__))

from rtk_manager import AsyncRTKManager, NtripConfig, RTKProcessError, load_ntrip_config

NOW = 1_000_000.0
GRACE = AsyncRTKManager._HEALTHY_MAX_AGE_S


# ── _health_anchor_age_s ──────────────────────────────────────────────────────


def test_no_anchors_returns_none():
    assert AsyncRTKManager._health_anchor_age_s({}, NOW) is None
    assert (
        AsyncRTKManager._health_anchor_age_s(
            {"last_frame_time": None, "connected_since": None}, NOW
        )
        is None
    )


def test_legacy_child_without_connected_since_uses_frame_age():
    status = {"last_frame_time": NOW - 3.0}
    assert AsyncRTKManager._health_anchor_age_s(status, NOW) == 3.0


def test_fresh_reconnect_outranks_stale_frame_time():
    # Outage of 300 s, then a reconnect 2 s ago: the handshake anchor must
    # win, giving the new connection its grace window.
    status = {"last_frame_time": NOW - 300.0, "connected_since": NOW - 2.0}
    assert AsyncRTKManager._health_anchor_age_s(status, NOW) == 2.0


def test_streaming_connection_uses_frame_time():
    status = {"last_frame_time": NOW - 1.0, "connected_since": NOW - 120.0}
    assert AsyncRTKManager._health_anchor_age_s(status, NOW) == 1.0


def test_both_stale_reports_stale():
    status = {"last_frame_time": NOW - 300.0, "connected_since": NOW - 60.0}
    assert AsyncRTKManager._health_anchor_age_s(status, NOW) == 60.0


def test_bool_anchor_values_are_ignored():
    # JSON true must not be mistaken for a timestamp near the epoch.
    status = {"last_frame_time": True, "connected_since": True}
    assert AsyncRTKManager._health_anchor_age_s(status, NOW) is None


def test_future_anchor_clamps_to_zero():
    status = {"last_frame_time": NOW + 5.0}
    assert AsyncRTKManager._health_anchor_age_s(status, NOW) == 0.0


# ── _status_locked healthy verdict ────────────────────────────────────────────


def _manager_with_status(tmp_path: Path, payload: dict) -> AsyncRTKManager:
    mgr = AsyncRTKManager()
    status_file = tmp_path / "rtk_status.json"
    status_file.write_text(json.dumps(payload), encoding="utf-8")
    mgr._status_file = status_file
    mgr._process = types.SimpleNamespace(returncode=None, pid=4242)
    mgr._mode = "ntrip"
    return mgr


def test_healthy_during_reconnect_grace(tmp_path):
    import time as _time

    now = _time.time()
    mgr = _manager_with_status(
        tmp_path,
        {
            "state": "connected",
            "connected": True,
            "last_frame_time": now - 300.0,
            "connected_since": now - 1.0,
            "frames": 42,
            "bytes": 4096,
        },
    )
    status = mgr._status_locked()
    assert status.healthy is True
    # Display age stays honest: it reports the real frame age, not the anchor.
    assert status.last_frame_age_s is not None and status.last_frame_age_s > GRACE


def test_unhealthy_when_disconnected_despite_fresh_anchor(tmp_path):
    import time as _time

    now = _time.time()
    mgr = _manager_with_status(
        tmp_path,
        {
            "state": "reconnecting",
            "connected": False,
            "last_frame_time": now - 1.0,
            "connected_since": None,
        },
    )
    assert mgr._status_locked().healthy is False


def test_unhealthy_when_connection_never_streams_past_grace(tmp_path):
    import time as _time

    now = _time.time()
    mgr = _manager_with_status(
        tmp_path,
        {
            "state": "connected",
            "connected": True,
            "last_frame_time": None,
            "connected_since": now - (GRACE + 5.0),
        },
    )
    assert mgr._status_locked().healthy is False


# ── protected config loading ──────────────────────────────────────────────────


def test_load_ntrip_config_supports_export_quotes_and_normalizes_mount(tmp_path):
    path = tmp_path / "ntrip.env"
    path.write_text(
        "export NTRIP_HOST='caster.example.com'\n"
        "NTRIP_PORT=2101\n"
        "NTRIP_MOUNTPT=\"/ROVER\"\n"
        "NTRIP_USER=field-user\n"
        "NTRIP_PASS='secret with spaces'\n",
        encoding="utf-8",
    )
    path.chmod(0o600)
    assert load_ntrip_config(path) == NtripConfig(
        host="caster.example.com",
        port=2101,
        mountpoint="ROVER",
        user="field-user",
        password="secret with spaces",
    )


def test_load_ntrip_config_rejects_missing_keys_without_exposing_values(tmp_path):
    path = tmp_path / "ntrip.env"
    path.write_text("NTRIP_PASS=do-not-leak\n", encoding="utf-8")
    try:
        load_ntrip_config(path)
    except RTKProcessError as exc:
        assert "do-not-leak" not in str(exc)
        assert "missing required keys" in str(exc)
    else:
        raise AssertionError("missing config keys must be rejected")


def test_status_exposes_desired_restart_state():
    mgr = AsyncRTKManager()
    mgr._desired_mode = "ntrip"
    mgr._desired_ntrip = NtripConfig("caster", 2101, "MOUNT", "user", "pass")
    mgr._restart_task = types.SimpleNamespace(done=lambda: False)
    status = mgr._status_locked()
    assert status.running is False
    assert status.desired_mode == "ntrip"
    assert status.source_state == "restarting"


def test_autostart_configuration_failure_is_visible_in_status():
    async def scenario():
        mgr = AsyncRTKManager()
        status = await mgr.mark_ntrip_unavailable("config missing")
        assert status.running is False
        assert status.desired_mode == "ntrip"
        assert status.source_state == "unavailable"
        assert status.last_error == "config missing"

    asyncio.run(scenario())


def test_supervised_restart_retains_desired_config_and_counts_success():
    async def scenario():
        mgr = AsyncRTKManager()
        config = NtripConfig("caster", 2101, "MOUNT", "user", "pass")
        mgr._desired_mode = "ntrip"
        mgr._desired_ntrip = config
        seen = []

        async def fake_start(received):
            seen.append(received)

        mgr._start_ntrip_locked = fake_start
        await mgr._restart_ntrip_after(0.0)
        assert seen == [config]
        assert mgr._supervisor_restarts == 1
        assert mgr._restart_attempt == 0

    asyncio.run(scenario())
