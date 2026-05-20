"""System routes: ping, healthz, activity log."""
from __future__ import annotations

import time

from fastapi import APIRouter

from config import MAX_ACTIVITY_LOG

router = APIRouter(tags=["system"])


@router.get("/ping")
async def ping():
    return {"status": "ok", "timestamp": time.time()}


@router.get("/healthz")
async def healthz():
    """Liveness + readiness probe.

    Distinct from /ping: returns ROS connectivity, FCU connection, RPP state,
    pose freshness — used by systemd Watchdog or external monitors.
    """
    from main import ros_node, offboard_ctrl
    s = ros_node.get_state() if ros_node else {}
    return {
        "ros_node":      ros_node is not None,
        "fcu_connected": s.get("connected", False),
        "armed":         s.get("armed", False),
        "mode":          s.get("mode", "UNKNOWN"),
        "rpp_state":     s.get("rpp_state"),
        "pose_age_ms":   s.get("pose_age_ms"),
        "mission_state": offboard_ctrl.state.value if offboard_ctrl else None,
    }


@router.get("/activity")
async def activity():
    from main import activity_log
    # activity_log is a deque(maxlen=MAX_ACTIVITY_LOG); slice as a list
    return list(activity_log)[-MAX_ACTIVITY_LOG:]
