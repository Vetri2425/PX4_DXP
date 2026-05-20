"""PX4 parameter read/write via MAVROS rcl_interfaces services.

GET  /api/params/{name}    — get single param
POST /api/params/{name}    — set param value (int / float / bool / str)
"""
from __future__ import annotations

import datetime

from fastapi import APIRouter, Depends, HTTPException

from auth import require_token
from models import ParamSetRequest

router = APIRouter(prefix="/params", tags=["params"],
                   dependencies=[Depends(require_token)])


def _record(level: str, message: str) -> None:
    from main import activity_log
    ts = datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"
    activity_log.append({"timestamp": ts, "level": level, "message": message})


@router.get("/{name}")
async def get_param(name: str):
    from main import ros_node
    if ros_node is None:
        raise HTTPException(503, "ROS node not ready")
    ok, value, why = await ros_node.get_param_async(name)
    if not ok:
        raise HTTPException(404, why or "param not found")
    return {"name": name, "value": value}


@router.post("/{name}")
async def set_param(name: str, req: ParamSetRequest):
    from main import ros_node
    if ros_node is None:
        raise HTTPException(503, "ROS node not ready")
    ok, why = await ros_node.set_param_async(name, req.value)
    if not ok:
        _record("error", f"param set {name}={req.value} failed: {why}")
        raise HTTPException(400, why or "param set failed")
    _record("info", f"param set {name}={req.value}")
    return {"name": name, "value": req.value, "ok": True}
