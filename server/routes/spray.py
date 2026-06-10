"""Manual spray servo control — bench test and operator override.

POST /api/spray/test   — timed manual ON (auto-off) or immediate OFF/cancel.
GET  /api/spray/status — actual vs desired vs manual-override spray state.

Safety model (server layer; the node and firmware layers sit beneath it):
- Manual ON is rejected while a mission is RUNNING — automatic MARK control
  owns the actuator during missions.
- Manual ON is rejected when disarmed (the FCU holds DISARMED PWM anyway;
  rejecting here gives the operator an actionable error instead of silence).
- Every manual ON gets a server-side auto-off timer (≤ MAX_SPRAY_TEST_DURATION_S);
  spray_controller_node's manual_override_timeout_s is the node-side backstop,
  and its disarm / mode / shutdown fail-safes outrank the override entirely.
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

_auto_off_task: Optional[asyncio.Task] = None


def _cancel_auto_off() -> None:
    global _auto_off_task
    if _auto_off_task is not None and not _auto_off_task.done():
        _auto_off_task.cancel()
    _auto_off_task = None


async def _auto_off_after(duration_s: float) -> None:
    """Publish manual OFF after the test window. Cancellation = superseded."""
    try:
        await asyncio.sleep(duration_s)
    except asyncio.CancelledError:
        return
    from main import ros_node
    if ros_node is not None:
        ros_node.publish_spray_manual(False)


@router.post("/test")
async def spray_test(req: SprayTestRequest):
    """Manual spray override: ON with timed auto-off, or immediate cancel."""
    from main import offboard_ctrl, ros_node
    global _auto_off_task

    if ros_node is None:
        raise HTTPException(503, "ROS bridge not ready")

    if not req.on:
        _cancel_auto_off()
        ros_node.publish_spray_manual(False)
        return {"manual": False}

    if offboard_ctrl is not None and offboard_ctrl.state == MissionState.RUNNING:
        raise HTTPException(
            409, "Manual spray is blocked while a mission is RUNNING"
        )
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

    _cancel_auto_off()
    ros_node.publish_spray_manual(True)
    _auto_off_task = asyncio.create_task(_auto_off_after(duration))
    return {"manual": True, "duration_s": duration}


@router.get("/status")
async def spray_status():
    """Actual commanded state, RPP MARK desire, and manual-override state."""
    from main import ros_node
    if ros_node is None:
        return {
            "spraying": False,
            "spray_active_desired": False,
            "manual_override": False,
        }
    s = ros_node.get_state()
    return {
        "spraying": bool(s.get("spraying", False)),
        "spray_active_desired": bool(s.get("spray_active", False)),
        "manual_override": bool(s.get("spray_manual", False)),
    }
