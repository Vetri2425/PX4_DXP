"""System routes: ping, healthz, activity log."""
from __future__ import annotations

import time

from fastapi import APIRouter, Depends

from auth import require_operator_or_machine
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


@router.get("/health/bridge")
async def health_bridge():
    """MAVROS bridge liveness + recovery state (Phase 3).

    Reports the BridgeHealthManager's view: health (healthy/degraded/
    recovering/failed), the cached bridge snapshot (fcu_connected,
    state_age_ms, pose_age_ms, mavros_state_publishers, armed, mode), and
    recovery metadata. Read-only; never commands the FCU.
    """
    from main import bridge_health, ros_node
    if bridge_health is not None:
        return bridge_health.get_status()
    # Fallback if the watchdog never started.
    snap = ros_node.get_bridge_snapshot() if ros_node is not None else {}
    return {
        "health": "unknown",
        "auto_recover": False,
        "recovery_count": 0,
        "last_recovery_ts": None,
        "last_recovery_reason": None,
        **snap,
    }


@router.get(
    "/activity",
    dependencies=[Depends(require_operator_or_machine("activity:read"))],
)
async def activity():
    from main import activity_log
    # activity_log is a deque(maxlen=MAX_ACTIVITY_LOG); slice as a list
    return list(activity_log)[-MAX_ACTIVITY_LOG:]

@router.post("/discover")
async def discover():
    """Return rovers discovered via UDP beacon on the LAN.

    The beacon listener caches beacons from other rover servers
    broadcasting on the discovery port. Returns active beacons
    seen within the TTL window.
    """
    from beacon import get_beacons
    beacons = get_beacons()
    return {
        "beacons": beacons,
        "count": len(beacons),
    }
