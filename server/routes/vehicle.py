"""Vehicle control routes: arm, set_mode, estop. Auth-protected."""
from __future__ import annotations

import datetime

from fastapi import APIRouter, Depends, HTTPException

from auth import require_token
from models import (
    ArmRequest, ArmResponse, EstopResponse, ModeRequest, ModeResponse,
)
from spray_safety import (
    disarm_with_spray_safety,
    set_mode_with_spray_safety,
)

router = APIRouter(tags=["vehicle"], dependencies=[Depends(require_token)])


def _now() -> str:
    return datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"


def _record(level: str, message: str) -> None:
    from main import activity_log
    activity_log.append({"timestamp": _now(), "level": level, "message": message})


@router.post("/arm", response_model=ArmResponse)
async def arm_vehicle(req: ArmRequest):
    from main import ros_node
    if ros_node is None:
        raise HTTPException(503, "ROS node not ready")

    if req.arm:
        ok, why = await ros_node.arm_async(True)
        msg = f"Armed {'OK' if ok else f'FAILED: {why}'}"
        _record("info" if ok else "error", msg)
        return ArmResponse(success=ok, message=msg)

    result = await disarm_with_spray_safety(ros_node)
    _record("info" if result.success else "warning", result.message)
    return ArmResponse(
        success=result.success,
        message=result.message,
        spray_off_confirmed=result.spray_off_confirmed,
        spray_off_result=result.spray_off_result,
        disarmed=result.transition_ok,
    )


@router.post("/set_mode", response_model=ModeResponse)
async def set_mode(req: ModeRequest):
    from main import ros_node
    if ros_node is None:
        raise HTTPException(503, "ROS node not ready")
    if req.mode.value == "OFFBOARD":
        raise HTTPException(409, "OFFBOARD transitions must use mission start")

    state = ros_node.get_state()
    current_mode = str(state.get("mode", "UNKNOWN"))
    result = await set_mode_with_spray_safety(
        ros_node,
        target_mode=req.mode.value,
        current_mode=current_mode,
    )
    _record("info" if result.success else "warning", result.message)
    return ModeResponse(
        success=result.success,
        message=result.message,
        spray_off_confirmed=result.spray_off_confirmed,
        spray_off_result=result.spray_off_result,
    )


@router.post("/estop", response_model=EstopResponse)
async def emergency_stop():
    from main import emergency_handler
    if emergency_handler is None:
        raise HTTPException(503, "Emergency handler not ready")
    result = await emergency_handler.estop_async()
    return EstopResponse(**result)