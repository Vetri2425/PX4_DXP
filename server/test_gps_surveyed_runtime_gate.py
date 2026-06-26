"""F-02/F-03/F-04: GPS_SURVEYED runtime gate, confirmed-OFF stop, clear reset.

Mandatory regression cases:
  #1 surveyed continuous fix 6→5 → watchdog fault (would force OFF + estop)
  #2 surveyed dash global-position stale → watchdog fault
  #3 recovery: fix restored is not "ready" until stable for recovery_stable_s
  #4 LOCAL_NED / point missions are NOT line/dash RTK-gated (no context)
  #5 stop continuous/dash → degraded unless spray node confirms OFF
  #6 clear mission → live spray config reset (revision 0, empty identity)
"""

from __future__ import annotations

import asyncio
import os
import sys
import time
from collections import deque

sys.path.insert(0, os.path.dirname(__file__))

import spray_mission_config
from gps_safety import GpsSurveyedSafetyParams, evaluate_gps_surveyed_safety
from models import MissionState
from offboard_controller import OffboardController
from mission_stop import stop_active_mission
from test_offboard_controller import FakeNode, load_surveyed, surveyed_state


def run(coro):
    return asyncio.run(coro)


# ── F-02: watchdog context only arms for surveyed continuous/dash RUNNING ──────

def _ctrl_running_surveyed(spray_mode="continuous"):
    node = FakeNode([surveyed_state()])
    ctrl = OffboardController(node, deque())
    load_surveyed(ctrl)
    ctrl._spray_mode = spray_mode
    ctrl._state = MissionState.RUNNING
    ctrl._running_mission_id = ctrl._loaded_mission_id
    return ctrl


def test_context_active_for_surveyed_continuous():
    ctrl = _ctrl_running_surveyed("continuous")
    ctx = ctrl.gps_surveyed_runtime_context()
    assert ctx is not None
    assert ctx["spray_mode"] == "continuous"
    assert ctx["origin_gps"] is not None
    assert len(ctx["source_points"]) > 0


def test_context_active_for_surveyed_dash():
    ctrl = _ctrl_running_surveyed("dash")
    assert ctrl.gps_surveyed_runtime_context() is not None


def test_context_none_for_point_mission():
    ctrl = _ctrl_running_surveyed("point")
    assert ctrl.gps_surveyed_runtime_context() is None


def test_context_none_for_local_ned():
    """#4: LOCAL_NED missions are never line/dash RTK-gated."""
    node = FakeNode([surveyed_state()])
    ctrl = OffboardController(node, deque())
    ctrl.load_path([(0.0, 0.0), (1.0, 0.0)], name="local")
    ctrl._state = MissionState.RUNNING
    assert ctrl.gps_surveyed_runtime_context() is None


def test_context_none_when_not_running():
    ctrl = _ctrl_running_surveyed("continuous")
    ctrl._state = MissionState.IDLE
    assert ctrl.gps_surveyed_runtime_context() is None


# ── F-02: the verdict the watchdog acts on ────────────────────────────────────

def _verdict(state, recovery_since=None):
    ctrl = _ctrl_running_surveyed("continuous")
    ctx = ctrl.gps_surveyed_runtime_context()
    return evaluate_gps_surveyed_safety(
        state,
        ctx["origin_gps"],
        ctx["source_points"],
        GpsSurveyedSafetyParams(),
        recovery_since=recovery_since,
    )


def test_fix_drop_faults():
    """#1: fix 6→5 mid-mission → verdict not ok (watchdog forces OFF + estop)."""
    v = _verdict(surveyed_state(gps_fix=5))
    assert v.ok is False
    assert "below required" in v.reason


def test_global_position_stale_faults():
    """#2: dash run, fused global position goes stale → verdict not ok."""
    v = _verdict(surveyed_state(global_position_age_ms=5000.0))
    assert v.ok is False
    assert "global position" in v.reason


def test_recovery_not_ready_until_stable():
    """#3: a freshly-recovered fix is not 'ready' until stable for the window."""
    just_now = _verdict(surveyed_state(), recovery_since=time.monotonic())
    assert just_now.ok is True
    assert just_now.recovery_ready is False  # not stable long enough yet
    stable = _verdict(
        surveyed_state(),
        recovery_since=time.monotonic() - (GpsSurveyedSafetyParams().recovery_stable_s + 1.0),
    )
    assert stable.recovery_ready is True


# ── F-03: confirmed-OFF on continuous/dash stop ───────────────────────────────

class _StopNode:
    def __init__(self, *, live, confirm_after_off):
        self._live = live
        self._confirm_after_off = confirm_after_off
        self._off_commanded = False
        self.calls = []

    def publish_spray_manual(self, on):
        self.calls.append(("manual", on))
        if not on:
            self._off_commanded = True

    async def cancel_spray_dwell_async(self):
        self.calls.append(("cancel_dwell",))
        return True, ""

    def get_spray_runtime_status(self):
        confirmed = self._off_commanded and self._confirm_after_off
        return {
            "status_stale": not self._live,
            "commanded_on": not confirmed,
            "confirmed_off": confirmed,
        }


class _StopController:
    def __init__(self):
        self.state = MissionState.IDLE

    async def stop_async(self):
        return {"success": True, "message": "stopped"}


def test_stop_degraded_when_off_not_confirmed():
    """#5: a live spray node that never confirms OFF downgrades the stop."""
    node = _StopNode(live=True, confirm_after_off=False)
    ctrl = _StopController()
    result = run(stop_active_mission(ctrl, None, node, None))
    assert result["success"] is False
    assert result["spray_off_degraded"] is True
    assert result["spray_confirmed_off"] is False
    assert "not confirmed" in result["message"]


def test_stop_success_when_off_confirmed():
    node = _StopNode(live=True, confirm_after_off=True)
    ctrl = _StopController()
    result = run(stop_active_mission(ctrl, None, node, None))
    assert result["success"] is True
    assert result["spray_confirmed_off"] is True
    assert ("manual", False) in node.calls


def test_stop_not_degraded_when_no_live_spray_node():
    """Non-spray / offline node must not be reported as a degraded stop."""
    node = _StopNode(live=False, confirm_after_off=False)
    ctrl = _StopController()
    result = run(stop_active_mission(ctrl, None, node, None))
    assert result["success"] is True
    assert result["spray_off_attempted"] is False


# ── F-04: clear mission resets live spray config ──────────────────────────────

def test_clear_resets_live_spray_config(monkeypatch):
    """#6: clear → apply default config with revision=0 and empty identity."""
    captured = {}

    async def fake_apply(node, staged, *, revision=None):
        captured["staged"] = staged
        captured["revision"] = revision
        return True, "applied", None

    monkeypatch.setattr(spray_mission_config, "apply_spray_mission_config", fake_apply)

    node = FakeNode([surveyed_state()])
    ctrl = OffboardController(node, deque())
    load_surveyed(ctrl)
    ctrl._spray_mode = "dash"
    ctrl._configuration_revision = 999
    ctrl._path_fingerprint = "abc123"

    run(ctrl.clear_mission_async())

    assert captured["revision"] == 0
    assert captured["staged"]["spray_mode"] == "continuous"
    assert captured["staged"]["mission_id"] == ""
    assert captured["staged"]["path_fingerprint"] == ""
    # in-memory identity also cleared
    assert ctrl._configuration_revision == 0
    assert ctrl._path_fingerprint == ""
    assert ctrl.state == MissionState.IDLE
