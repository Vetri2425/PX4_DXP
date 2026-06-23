"""Verify mission safety does not depend on LoRa lifecycle state."""

from __future__ import annotations

import asyncio
import inspect
import os
import sys
from collections import deque

import pytest

sys.path.insert(0, os.path.dirname(__file__))

import offboard_controller as offboard_module
from config import POSE_STALE_MS, RPP_RTK_WAIT, RPP_TRACKING, RPP_UNHEALTHY_CODES
from mission_placement import GPS_SURVEYED, PlacementError, resolve_surveyed_points
from offboard_controller import MissionState, OffboardController
from test_offboard_controller import FakeNode, load_surveyed, surveyed_state


LORA_LIFECYCLE_STATES = (
    "NO_DATA",
    "RECONNECTING",
    "MODULE_DISCONNECTED",
    "FAILED",
)


def _run(coro):
    return asyncio.run(coro)


def _watchdog_would_estop(rpp_state: int, pose_age_ms: float, connected: bool) -> bool:
    """Mirror server/main.py RUNNING watchdog predicate (no LoRa inputs)."""
    return (
        rpp_state in RPP_UNHEALTHY_CODES
        or pose_age_ms > POSE_STALE_MS
        or not connected
    )


def test_rpp_unhealthy_codes_exclude_lora_lifecycle():
    assert RPP_RTK_WAIT in RPP_UNHEALTHY_CODES
    assert "lora" not in {str(code).lower() for code in RPP_UNHEALTHY_CODES}


def test_offboard_start_guard_uses_rpp_not_lora():
    source = inspect.getsource(OffboardController.start_async)
    assert "RTK_WAIT" in source or "RPP_RTK_WAIT" in source or "rpp_code" in source
    assert "lifecycle_state" not in source
    assert "desired_source" not in source


def test_mission_placement_uses_gps_fix_not_lora():
    source = inspect.getsource(resolve_surveyed_points)
    assert "gps_fix" in source
    assert "lora" not in source.lower()


@pytest.mark.parametrize("lora_lifecycle", LORA_LIFECYCLE_STATES)
def test_surveyed_placement_ignores_lora_lifecycle(lora_lifecycle: str):
    """Placement gates on GPS fix freshness, not LoRa transport state."""
    state = surveyed_state()
    state["lora_lifecycle_state"] = lora_lifecycle
    state["stream_healthy"] = False
    pts, _ = resolve_surveyed_points(
        [(0.0, 0.0), (1.0, 0.0)],
        (13.072066, 80.261956),
        state,
    )
    assert len(pts) == 2

    bad = surveyed_state(gps_fix=5)
    bad["lora_lifecycle_state"] = lora_lifecycle
    with pytest.raises(PlacementError, match="below RTK_FIXED"):
        resolve_surveyed_points([(0.0, 0.0)], (13.072066, 80.261956), bad)


@pytest.mark.parametrize("lora_lifecycle", LORA_LIFECYCLE_STATES)
def test_mission_start_succeeds_with_healthy_rpp_despite_lora_transport(lora_lifecycle: str):
    """Offboard start uses FCU/RPP state only; LoRa fields are not consumed."""
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        state = surveyed_state()
        state["lora_lifecycle_state"] = lora_lifecycle
        state["stream_healthy"] = False
        state["transport_reason"] = "transmitter_silent"
        node = FakeNode([state])
        ctrl = OffboardController(node, deque())
        load_surveyed(ctrl)

        ok, msg = _run(ctrl.start_async(expected_mission_id="stg_field"))

        assert ok is True
        assert msg == "running"
        assert ctrl.state == MissionState.RUNNING
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_mission_start_rejects_rpp_rtk_wait_not_lora_transport():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        state = surveyed_state(rpp_state=RPP_RTK_WAIT)
        state["lora_lifecycle_state"] = "STREAMING_VALID_RTCM"
        state["stream_healthy"] = True
        node = FakeNode([state])
        ctrl = OffboardController(node, deque())
        load_surveyed(ctrl)

        ok, msg = _run(ctrl.start_async(expected_mission_id="stg_field"))

        assert ok is False
        assert "RTK_WAIT" in msg
        assert ctrl.state == MissionState.ERROR
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


@pytest.mark.parametrize("lora_lifecycle", LORA_LIFECYCLE_STATES)
def test_watchdog_does_not_estop_on_lora_lifecycle_alone(lora_lifecycle: str):
    """Telemetry watchdog estops on RPP/pose/FCU — not LoRa lifecycle strings."""
    assert not _watchdog_would_estop(RPP_TRACKING, 10.0, True)
    assert _watchdog_would_estop(RPP_RTK_WAIT, 10.0, True)


def test_main_watchdog_source_has_no_lora_inputs():
    import main

    source = inspect.getsource(main._telemetry_loop)
    assert "RPP_UNHEALTHY_CODES" in source
    assert "lifecycle_state" not in source
    assert "desired_source" not in source
    assert "lora" not in source.lower()


def test_spray_routes_have_no_lora_gates():
    import routes.spray as spray_routes

    source = inspect.getsource(spray_routes)
    assert "lora" not in source.lower()
    assert "rtk_manager" not in source