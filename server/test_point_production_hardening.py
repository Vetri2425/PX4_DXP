#!/usr/bin/env python3
"""Production-hardening tests for point navigation (Task_01 review blockers).

Covers:
- spray confirmation policy (mark=false vs mark=true; stale spray node),
- cancellation/drain reliability under a slow/hung spray service,
- unconditional cleanup + no leaked task exception,
- obstacle signal safety states (disabled / fresh / stale / never-received),
- status schema fields.
"""

from __future__ import annotations

import asyncio
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from models import PointMissionStatusResponse
from point_ingest import SprayPoint
from point_mission import (
    OBSTACLE_BLOCKED,
    OBSTACLE_MISSING,
    OBSTACLE_NOT_CONFIGURED,
    OBSTACLE_OK,
    OBSTACLE_STALE,
    PointMissionOrchestrator,
    PointMissionState,
)
from setpoint_hold import SetpointHoldOwner
from spray_config import (
    ObstacleSafetyParams,
    PointSprayParams,
    SprayConfiguration,
    SprayMode,
)
from test_point_mission import FakeOffboard, FakeRos


def _cfg(*, mark_default_dwell: float = 0.05, **overrides) -> SprayConfiguration:
    point = PointSprayParams(
        default_dwell_s=mark_default_dwell,
        arrival_tolerance_m=0.05,
        settle_time_s=0.0,
        leg_timeout_s=2.0,
        settle_speed_mps=0.05,
        settle_yaw_rate_rad_s=0.05,
    )
    return SprayConfiguration(mode=SprayMode.POINT, point=point, revision=1, **overrides)


class StaleSprayRos(FakeRos):
    """FakeRos whose spray runtime status is always stale/unavailable."""

    def get_spray_runtime_status(self):
        return {
            "status_stale": True,
            "ready": False,
            "active_dwell": False,
            "dwell_remaining_s": 0.0,
            "commanded_on": False,
            "confirmed_off": False,
            "off_acknowledged": False,
            "last_error": "",
            "dwell_command_id": None,
            "dwell_mission_id": "",
            "dwell_point_index": None,
            "configuration_revision": 0,
            "model_revision": 0,
            "timestamp_monotonic_s": 0.0,
        }


class HungCancelRos(FakeRos):
    """FakeRos whose dwell-cancel service hangs (simulates a wedged service)."""

    def __init__(self, hang_s: float = 1.0):
        super().__init__()
        self.hang_s = hang_s
        self.manual_off_calls = 0

    async def cancel_spray_dwell_async(self):
        await asyncio.sleep(self.hang_s)
        self.live_dwell = None
        return True, "ok"

    def publish_spray_manual(self, on: bool):
        super().publish_spray_manual(on)
        if on is False:
            self.manual_off_calls += 1


async def _wait_for_state(orch, state, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if orch.status.state == state:
            return
        await asyncio.sleep(0.02)
    raise AssertionError(f"timed out waiting for {state}, got {orch.status.state}")


# ───────────────────────── Spray confirmation policy ─────────────────────────

@pytest.mark.anyio
async def test_navonly_mission_completes_with_stale_spray_status():
    """Pure mark=false navigation must complete despite a stale spray node."""
    ros = StaleSprayRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="navonly",
        points=[SprayPoint(1.0, 0.0, 0.0, 0, mark=False),
                SprayPoint(2.0, 0.0, 0.0, 1, mark=False)],
        config=_cfg(),
    )
    started, _ = await orch.start(ros, FakeOffboard())
    assert started
    # Stale spray makes each (best-effort) OFF confirm wait its full ~1s window;
    # a healthy node confirms instantly. Allow headroom for the degraded path.
    await asyncio.wait_for(orch._task, timeout=10.0)
    assert orch.status.state == PointMissionState.COMPLETED
    # Spray was never engaged → terminal safety stays OK (no spray to confirm).
    assert orch.status.terminal_safety_ok is True
    assert orch._spray_ever_on is False
    # OFF was still commanded for every leg + terminal cleanup.
    assert ros.manual is False
    # No leaked exception on the success path.
    assert orch._task.exception() is None


@pytest.mark.anyio
async def test_marked_mission_does_not_complete_without_spray_confirmation():
    """A marked leg requires a healthy spray node; stale status must FAIL it."""
    ros = StaleSprayRos()
    ros.auto_arrive = True
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="marked",
        points=[SprayPoint(1.0, 0.0, 0.05, 0, mark=True)],
        config=_cfg(),
    )
    await orch.start(ros, FakeOffboard())
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state != PointMissionState.COMPLETED
    assert orch.status.state == PointMissionState.FAILED
    assert orch._task.exception() is None  # _run swallows; no leaked exception


@pytest.mark.anyio
async def test_force_spray_off_raises_when_confirmation_required():
    ros = StaleSprayRos()
    orch = PointMissionOrchestrator()
    with pytest.raises(TimeoutError):
        await orch._force_spray_off_confirmed(ros, require_confirm=True)
    assert ros.manual is False  # OFF still commanded


@pytest.mark.anyio
async def test_force_spray_off_warns_when_confirmation_optional():
    ros = StaleSprayRos()
    orch = PointMissionOrchestrator()
    confirmed = await orch._force_spray_off_confirmed(ros, require_confirm=False)
    assert confirmed is False
    assert ros.manual is False  # OFF still commanded, just unconfirmed


# ──────────────────── Cancellation / drain reliability ───────────────────────

@pytest.mark.anyio
async def test_hung_spray_service_stop_does_not_wedge():
    """A hung dwell-cancel service must not wedge or raise on stop."""
    ros = HungCancelRos(hang_s=1.0)
    ros.auto_arrive = False  # stay navigating
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch._DRAIN_TIMEOUT_S = 0.3
    orch.load(
        mission_id="hung_stop",
        points=[SprayPoint(9.0, 0.0, 0.0, 0, mark=False)],
        config=_cfg(),
    )
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.sleep(0.05)
    t0 = time.monotonic()
    await orch.stop_mission(ros, hold, reason="operator_stop")  # must not raise
    elapsed = time.monotonic() - t0
    assert elapsed < 2.5, f"stop wedged for {elapsed:.2f}s"
    # Cleanup ran regardless of the hung service.
    assert orch._task is None
    assert orch._run_token is None
    assert not orch.is_active()
    assert not hold.active
    assert ros.manual_off_calls >= 1  # OFF was commanded synchronously


@pytest.mark.anyio
@pytest.mark.parametrize("op", ["abort", "clear"])
async def test_hung_spray_service_abort_clear_do_not_wedge(op):
    """abort/clear share cancel_and_drain; a hung service must not wedge them."""
    ros = HungCancelRos(hang_s=1.0)
    ros.auto_arrive = False
    orch = PointMissionOrchestrator()
    orch._DRAIN_TIMEOUT_S = 0.3
    orch.load(
        mission_id=f"hung_{op}",
        points=[SprayPoint(9.0, 0.0, 0.0, 0, mark=False)],
        config=_cfg(),
    )
    await orch.start(ros, FakeOffboard())
    await asyncio.sleep(0.05)
    t0 = time.monotonic()
    if op == "abort":
        await orch.abort(ros)
    else:
        await orch.clear_mission(ros, reason="cleared")
    assert time.monotonic() - t0 < 2.5
    assert orch._task is None
    assert orch._run_token is None
    assert not orch.is_active()
    assert ros.manual_off_calls >= 1


@pytest.mark.anyio
async def test_cancel_and_drain_always_clears_references():
    ros = FakeRos()
    ros.auto_arrive = False
    hold = SetpointHoldOwner()
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="drain_refs",
        points=[SprayPoint(5.0, 0.0, 0.0, 0, mark=False)],
        config=_cfg(),
    )
    await orch.start(ros, FakeOffboard(), hold)
    await asyncio.sleep(0.05)
    await orch.cancel_and_drain(ros, reason="abort")
    assert orch._task is None
    assert orch._run_token is None
    assert orch.status.run_active is False
    assert orch.status.active_dwell_command_id is None


@pytest.mark.anyio
async def test_no_leaked_task_exception_on_failure():
    """A failing run is caught in _run; its task must not carry an exception."""
    ros = FakeRos()
    ros.auto_arrive = False  # never arrives → leg timeout → FAILED
    orch = PointMissionOrchestrator()
    # Short leg timeout → fast deterministic failure (never arrives).
    fail_cfg = SprayConfiguration(
        mode=SprayMode.POINT,
        point=PointSprayParams(leg_timeout_s=0.2, arrival_tolerance_m=0.01,
                               settle_time_s=0.0),
        revision=1,
    )
    orch.load(
        mission_id="leak_fail",
        points=[SprayPoint(1.0, 0.0, 0.0, 0, mark=False)],
        config=fail_cfg,
    )
    await orch.start(ros, FakeOffboard())
    await asyncio.wait_for(orch._task, timeout=3.0)
    assert orch.status.state == PointMissionState.FAILED
    assert orch._task.exception() is None


# ───────────────────────── Obstacle signal safety ────────────────────────────

def test_obstacle_disabled_reports_not_configured():
    orch = PointMissionOrchestrator()
    orch.load(mission_id="ob_off", points=[SprayPoint(1, 0, 0.0, 0, mark=False)],
              config=_cfg())  # obstacle disabled by default
    orch.set_obstacle_clear(False)  # even an explicit block is ignored when off
    blocked, state, _ = orch._obstacle_gate()
    assert blocked is False
    assert state == OBSTACLE_NOT_CONFIGURED


def test_obstacle_enabled_fresh_clear_is_ok():
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="ob_ok",
        points=[SprayPoint(1, 0, 0.0, 0, mark=False)],
        config=_cfg(obstacle=ObstacleSafetyParams(enabled=True, signal_max_age_s=5.0)),
    )
    orch.set_obstacle_clear(True)
    blocked, state, age_ms = orch._obstacle_gate()
    assert blocked is False
    assert state == OBSTACLE_OK
    assert age_ms is not None and age_ms >= 0.0


def test_obstacle_enabled_blocked_pauses():
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="ob_blk",
        points=[SprayPoint(1, 0, 0.0, 0, mark=False)],
        config=_cfg(obstacle=ObstacleSafetyParams(enabled=True, signal_max_age_s=5.0)),
    )
    orch.set_obstacle_clear(False)
    blocked, state, _ = orch._obstacle_gate()
    assert blocked is True
    assert state == OBSTACLE_BLOCKED


def test_obstacle_enabled_stale_pauses():
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="ob_stale",
        points=[SprayPoint(1, 0, 0.0, 0, mark=False)],
        config=_cfg(obstacle=ObstacleSafetyParams(enabled=True, signal_max_age_s=0.5)),
    )
    orch.set_obstacle_clear(True)
    # Age the last receipt beyond the stale threshold deterministically.
    orch._obstacle_last_recv = time.monotonic() - 5.0
    blocked, state, age_ms = orch._obstacle_gate()
    assert blocked is True
    assert state == OBSTACLE_STALE
    assert age_ms is not None and age_ms > 500.0


def test_obstacle_enabled_never_received_is_missing():
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="ob_missing",
        points=[SprayPoint(1, 0, 0.0, 0, mark=False)],
        config=_cfg(obstacle=ObstacleSafetyParams(enabled=True, signal_max_age_s=5.0)),
    )
    blocked, state, age_ms = orch._obstacle_gate()
    assert blocked is True
    assert state == OBSTACLE_MISSING
    assert age_ms is None


# ───────────────────────────── Status schema ─────────────────────────────────

def test_status_schema_round_trips_new_fields():
    orch = PointMissionOrchestrator()
    orch.load(
        mission_id="schema",
        points=[SprayPoint(1, 0, 0.0, 0, mark=False)],
        config=_cfg(obstacle=ObstacleSafetyParams(enabled=True, signal_max_age_s=5.0)),
    )
    payload = orch.status.as_dict()
    # Construct the response model directly from as_dict (no key/field mismatch).
    resp = PointMissionStatusResponse(**payload)
    # New diagnostic fields must be present in both dict and model.
    for field in (
        "obstacle_integration_enabled",
        "obstacle_signal_state",
        "obstacle_signal_age_ms",
        "terminal_safety_ok",
        "terminal_safety_reason",
        "point_leg_trajectory_mode",
        "point_leg_spacing_m",
        "point_leg_published_count",
        "point_leg_conditioned_count",
        "active_trajectory_mode",
        "point_leg_length_m",
    ):
        assert field in payload, f"{field} missing from as_dict()"
        assert hasattr(resp, field), f"{field} missing from response model"


def main():
    # Allow running standalone (mirrors sibling test modules).
    import inspect

    loop = asyncio.new_event_loop()
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_"):
            continue
        if inspect.iscoroutinefunction(fn):
            loop.run_until_complete(fn())
        else:
            fn()
    loop.close()
    print("PASS")


if __name__ == "__main__":
    main()
