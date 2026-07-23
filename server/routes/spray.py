"""Manual spray servo control — enable gate, bench test, and operator ON/OFF.

Endpoints
---------
POST /api/spray/enable  — lift the master spray gate; no motor action
POST /api/spray/disable — close the master spray gate; stops any active spray
POST /api/spray/on      — hold spray motor ON (requires enabled + armed)
POST /api/spray/off     — stop spray motor; always safe, no gate
POST /api/spray/test    — timed bench test: ON with auto-off (requires enabled)
GET  /api/spray/status  — enabled state, actual vs desired vs manual-override

Safety model (server layer; node and firmware layers sit beneath):
- /spray/enable / /spray/disable: master gate. When disabled, /on and /test ON
  are blocked (409). The node's spray_enabled parameter is also set so
  autonomous mission spray is suppressed at the actuator level.
- /spray/on requires enabled + armed + no RUNNING mission.
- /spray/off is always accepted — OFF is unconditionally safe.
- /spray/disable always succeeds and immediately stops any active spray/hold.
- spray_controller_node's disarm / mode-loss / shutdown fail-safes outrank
  every server-side command entirely.
"""
from __future__ import annotations

import asyncio
import math
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel

from auth import require_token
from logging_setup import get_logger
from models import MissionState, SprayTestRequest

log = get_logger("server.spray")

router = APIRouter(prefix="/spray", tags=["spray"],
                   dependencies=[Depends(require_token)])

MAX_SPRAY_TEST_DURATION_S = 10.0
DEFAULT_SPRAY_TEST_DURATION_S = 3.0

# Must be less than spray_controller_node's manual_override_timeout_s (default
# 10s) so the hold stays alive. 8s gives a 2s margin before the node times out.
KEEPALIVE_INTERVAL_S = 8.0

# Master enable gate. False = spray system disabled; /on and /test ON are
# blocked; node param spray_enabled is also set to False so autonomous mission
# spray cannot fire. Default False — operator must explicitly enable before use.
_spray_enabled: bool = False

_auto_off_task: Optional[asyncio.Task] = None
_keepalive_task: Optional[asyncio.Task] = None


# ── Task management ───────────────────────────────────────────────────────────

def _cancel_auto_off() -> None:
    global _auto_off_task
    if _auto_off_task is not None and not _auto_off_task.done():
        _auto_off_task.cancel()
    _auto_off_task = None


def _cancel_keepalive() -> None:
    global _keepalive_task
    if _keepalive_task is not None and not _keepalive_task.done():
        _keepalive_task.cancel()
    _keepalive_task = None


def _cancel_all() -> None:
    """Cancel both the bench-test auto-off timer and the hold keepalive."""
    _cancel_auto_off()
    _cancel_keepalive()


# ── Background coroutines ─────────────────────────────────────────────────────

async def _auto_off_after(duration_s: float) -> None:
    """Publish manual OFF after the test window. Cancellation = superseded."""
    try:
        await asyncio.sleep(duration_s)
    except asyncio.CancelledError:
        return
    from main import ros_node
    if ros_node is not None:
        ros_node.publish_spray_manual(False)


async def _keepalive_loop() -> None:
    """Re-publish manual ON every KEEPALIVE_INTERVAL_S so the node's
    manual_override_timeout_s backstop never fires while a hold is active.
    Cancelled by spray_off() or a superseding spray_test() call."""
    from main import ros_node
    try:
        while True:
            await asyncio.sleep(KEEPALIVE_INTERVAL_S)
            if ros_node is not None:
                ros_node.publish_spray_manual(True)
    except asyncio.CancelledError:
        return


# ── Helpers ───────────────────────────────────────────────────────────────────

def _check_spray_enabled() -> None:
    """Raise 409 if the spray system has not been enabled."""
    if not _spray_enabled:
        raise HTTPException(
            409,
            "Spray system is disabled — call POST /api/spray/enable first",
        )


# ── Endpoints ─────────────────────────────────────────────────────────────────

@router.post("/enable")
async def spray_enable():
    """Lift the master spray gate.

    No motor action. Allows /spray/on and /spray/test to reach the actuator.
    Also sets spray_enabled=True on the spray_controller node so autonomous
    mission spray can fire during path execution.
    """
    global _spray_enabled
    _spray_enabled = True
    from main import ros_node
    if ros_node is not None:
        try:
            await ros_node.set_spray_param_async("spray_enabled", True)
        except Exception:
            log.warning("Could not set node spray_enabled=True; server gate active", exc_info=True)
    return {"enabled": True}


@router.post("/disable")
async def spray_disable():
    """Close the master spray gate and stop any active spray immediately.

    Cancels any active hold or bench-test timer, publishes manual OFF to the
    node, and sets spray_enabled=False on the node so autonomous mission spray
    is also suppressed. Always succeeds — disabling is always safe.
    """
    global _spray_enabled
    _spray_enabled = False
    _cancel_all()
    from main import ros_node
    if ros_node is not None:
        ros_node.publish_spray_manual(False)
        try:
            await ros_node.set_spray_param_async("spray_enabled", False)
        except Exception:
            log.warning("Could not set node spray_enabled=False; server gate active", exc_info=True)
    return {"enabled": False}


@router.post("/on")
async def spray_on():
    """Hold spray ON until POST /api/spray/off is called.

    Publishes manual ON immediately and starts a keepalive that re-asserts
    every 8 s so the node's 10s timeout never fires during an active hold.
    Rejected while a mission is RUNNING or the FCU is disarmed.
    """
    from main import offboard_ctrl, ros_node
    global _keepalive_task

    _check_spray_enabled()
    if ros_node is None:
        raise HTTPException(503, "ROS bridge not ready")
    if offboard_ctrl is not None and offboard_ctrl.state in (
        MissionState.RUNNING, MissionState.ENTRY
    ):
        raise HTTPException(409, "Manual spray is blocked while a mission is RUNNING")
    state = ros_node.get_state()
    if not bool(state.get("armed", False)):
        raise HTTPException(
            409,
            "Spray ON requires an armed FCU — the AUX output holds its "
            "DISARMED (OFF) PWM while disarmed",
        )

    _cancel_all()
    ros_node.publish_spray_manual(True)
    _keepalive_task = asyncio.create_task(_keepalive_loop())
    return {"spraying": True, "hold": True}


@router.post("/off")
async def spray_off():
    """Turn spray OFF immediately and cancel any active hold or bench-test timer.

    Always succeeds — OFF is unconditionally safe.
    """
    from main import ros_node
    _cancel_all()
    if ros_node is not None:
        ros_node.publish_spray_manual(False)
    return {"spraying": False, "hold": False}


@router.post("/test")
async def spray_test(req: SprayTestRequest):
    """Bench-test spray override: ON with timed auto-off, or immediate cancel.

    ON is capped at MAX_SPRAY_TEST_DURATION_S (10s). Use /spray/on for an
    indefinite hold. Cancels any active hold keepalive before starting.
    """
    from main import offboard_ctrl, ros_node
    global _auto_off_task

    if ros_node is None:
        raise HTTPException(503, "ROS bridge not ready")

    if not req.on:
        _cancel_all()
        ros_node.publish_spray_manual(False)
        return {"manual": False}

    _check_spray_enabled()
    if offboard_ctrl is not None and offboard_ctrl.state in (
        MissionState.RUNNING, MissionState.ENTRY
    ):
        raise HTTPException(409, "Manual spray is blocked while a mission is RUNNING")
    state = ros_node.get_state()
    if not bool(state.get("armed", False)):
        raise HTTPException(
            409,
            "Manual spray requires an armed FCU — the AUX output holds its "
            "DISARMED (OFF) PWM while disarmed",
        )

    duration = (
        DEFAULT_SPRAY_TEST_DURATION_S
        if req.duration_s is None
        else float(req.duration_s)
    )
    if not math.isfinite(duration) or duration <= 0.0:
        raise HTTPException(400, "duration_s must be a positive number")
    duration = min(duration, MAX_SPRAY_TEST_DURATION_S)

    _cancel_all()
    ros_node.publish_spray_manual(True)
    _auto_off_task = asyncio.create_task(_auto_off_after(duration))
    return {"manual": True, "duration_s": duration}


class PointAdvanceRequest(BaseModel):
    """G5 manual gate: advance past the point the RPP is holding.

    `expect_index` is the point index the frontend saw reported as
    WAIT_OPERATOR (from /api/spray/status.point.rpp_point_index). The RPP
    rejects the advance unless it matches the point it is actually holding, so a
    stale double-tap for an already-advanced point is a no-op.
    """

    expect_index: int


@router.post("/point/advance")
async def spray_point_advance(req: PointAdvanceRequest):
    """G5: operator "Next point" — release the RPP's manual WAIT_OPERATOR hold.

    Publishes /point/advance {advance:true, expect_index}. Fire-and-forget: the
    RPP acts on it only while it is holding `expect_index` in WAIT_OPERATOR
    (manual point_execution_mode); otherwise it is safely ignored. Returns the
    published command so the caller can confirm the target.
    """
    from main import ros_node

    if ros_node is None:
        raise HTTPException(503, "ROS bridge not ready")
    if req.expect_index < 0:
        raise HTTPException(422, "expect_index must be >= 0")
    ros_node.publish_point_advance(req.expect_index)
    return {"advance": True, "expect_index": req.expect_index}


@router.get("/status")
async def spray_status():
    """Enabled gate, actual commanded state, RPP MARK desire, manual-override, hold state."""
    from main import ros_node
    hold_active = _keepalive_task is not None and not _keepalive_task.done()
    if ros_node is None:
        return {
            "enabled": _spray_enabled,
            "spraying": False,
            "spray_active_desired": False,
            "manual_override": False,
            "hold_active": hold_active,
            # No ROS graph → the spray node is not reporting a mode.
            "mode": None,
        }
    s = ros_node.get_state()
    resp = {
        "enabled": _spray_enabled,
        "spraying": bool(s.get("spraying", False)),
        "spray_active_desired": bool(s.get("spray_active", False)),
        "manual_override": bool(s.get("spray_manual", False)),
        "hold_active": hold_active,
        # Live spray mode mirrored from the node's /spray/status. None means the
        # spray node has not been heard from (not "continuous").
        "mode": s.get("spray_mode"),
    }
    # Attach the mode-specific config the node is actually running, so the app
    # can show it without a separate call. Sourced from the node's mode_state
    # (single source of truth), not from what the server last published.
    mode = resp["mode"]
    mode_state = s.get("spray_mode_state") or {}
    if mode == "dash":
        resp["dash"] = {
            "on_distance_m": mode_state.get("on_distance_m"),
            "off_distance_m": mode_state.get("off_distance_m"),
            "phase": mode_state.get("phase"),
            "armed": mode_state.get("armed"),
        }
    elif mode == "point":
        resp["point"] = {
            "dwell_s": mode_state.get("dwell_s"),
            "arrival_tolerance_m": mode_state.get("arrival_tolerance_m"),
            "num_points": mode_state.get("num_points"),
            "target_index": mode_state.get("target_index"),
            "phase": mode_state.get("phase"),
            # G5 — RPP mission phase mirrored via the spray node's /spray/status.
            # wait_operator=True ⇒ the RPP is holding in WAIT_OPERATOR (manual
            # gate); show the "Next point" button and POST /api/spray/point/advance
            # with rpp_point_index as expect_index. None until the RPP publishes
            # progress (progress_publish_enabled) — the frontend hides the button.
            "rpp_phase_name": mode_state.get("rpp_phase_name"),
            "rpp_point_index": mode_state.get("rpp_point_index"),
            "wait_operator": bool(mode_state.get("wait_operator", False)),
        }
    return resp
