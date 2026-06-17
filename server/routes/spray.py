"""Manual spray servo control — bench test and operator ON/OFF control.

Endpoints
---------
POST /api/spray/on     — hold spray ON until explicit OFF (operator control)
POST /api/spray/off    — turn spray OFF immediately, cancel any active hold/timer
POST /api/spray/test   — timed bench test: ON with auto-off, or immediate cancel
GET  /api/spray/status — actual vs desired vs manual-override spray state

Safety model (server layer; node and firmware layers sit beneath):
- Manual ON is rejected while a mission is RUNNING — MARK control owns the
  actuator during missions.
- Manual ON is rejected when disarmed (FCU holds DISARMED PWM anyway; rejecting
  here gives the operator an actionable error instead of silence).
- /spray/on holds via a server-side keepalive that re-asserts True every
  KEEPALIVE_INTERVAL_S seconds. If the server dies the node's
  manual_override_timeout_s (10s default) turns spray OFF automatically.
- /spray/off is always accepted — OFF is always safe regardless of state.
- spray_controller_node's disarm / mode-loss / shutdown fail-safes outrank
  every server-side command entirely.
"""
from __future__ import annotations

import asyncio
import math
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException

from auth import require_token
from models import MissionState, SprayTestRequest

router = APIRouter(prefix="/spray", tags=["spray"],
                   dependencies=[Depends(require_token)])

MAX_SPRAY_TEST_DURATION_S = 10.0
DEFAULT_SPRAY_TEST_DURATION_S = 3.0

# Must be less than spray_controller_node's manual_override_timeout_s (default
# 10s) so the hold stays alive. 8s gives a 2s margin before the node times out.
KEEPALIVE_INTERVAL_S = 8.0

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


# ── Endpoints ─────────────────────────────────────────────────────────────────

@router.post("/on")
async def spray_on():
    """Hold spray ON until POST /api/spray/off is called.

    Publishes manual ON immediately and starts a keepalive that re-asserts
    every 8 s so the node's 10s timeout never fires during an active hold.
    Rejected while a mission is RUNNING or the FCU is disarmed.
    """
    from main import offboard_ctrl, ros_node
    global _keepalive_task

    if ros_node is None:
        raise HTTPException(503, "ROS bridge not ready")
    if offboard_ctrl is not None and offboard_ctrl.state == MissionState.RUNNING:
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

    if offboard_ctrl is not None and offboard_ctrl.state == MissionState.RUNNING:
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


@router.get("/status")
async def spray_status():
    """Actual commanded state, RPP MARK desire, manual-override, and hold state."""
    from main import ros_node
    hold_active = _keepalive_task is not None and not _keepalive_task.done()
    if ros_node is None:
        return {
            "spraying": False,
            "spray_active_desired": False,
            "manual_override": False,
            "hold_active": hold_active,
        }
    s = ros_node.get_state()
    return {
        "spraying": bool(s.get("spraying", False)),
        "spray_active_desired": bool(s.get("spray_active", False)),
        "manual_override": bool(s.get("spray_manual", False)),
        "hold_active": hold_active,
    }
