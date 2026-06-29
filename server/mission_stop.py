"""Unified non-resumable mission stop (REST + Socket.IO parity)."""

from __future__ import annotations

from typing import Any

from logging_setup import get_logger
from spray_safety import force_spray_off_confirmed

log = get_logger("server.mission_stop")


async def _force_spray_off_confirmed(
    ros_node, *, timeout_s: float = 1.5
) -> tuple[bool, bool, dict[str, Any]]:
    """Command spray OFF and wait for the node to confirm (F-03).

    Returns (attempted, confirmed). ``attempted`` is False when there is no live
    spray node to confirm against (node absent/stale) — a non-spray or offline
    mission must not be reported as a degraded stop. ``confirmed`` is True only
    when the spray node reports ``confirmed_off`` with no commanded ON."""
    result = await force_spray_off_confirmed(ros_node, timeout_s=timeout_s)
    attempted = bool(result.attempted and result.live)
    return attempted, bool(result.success), result.as_dict()


async def stop_active_mission(
    offboard_ctrl,
    point_mission,
    ros_node,
    hold_owner,
    *,
    mission_capture=None,
    transport: str = "rest",
) -> dict[str, Any]:
    """Cancel point mission (if any), release hold, then soft-stop controller."""
    if point_mission is not None and (
        point_mission.is_active() or point_mission.is_paused()
    ):
        await point_mission.stop_mission(ros_node, hold_owner, reason="operator_stop")
    if hold_owner is not None:
        hold_owner.deactivate(ros_node)
    # F-03: continuous/dash stop reuses the point-mode confirmed-OFF guarantee.
    # The point branch above already confirms OFF; this covers line/dash stops
    # (and is idempotent for point). A live spray node that cannot confirm OFF
    # downgrades the stop to a non-success degraded result.
    spray_attempted, spray_confirmed, spray_off_result = await _force_spray_off_confirmed(
        ros_node
    )
    result = await offboard_ctrl.stop_async()
    result["spray_off_attempted"] = spray_attempted
    result["spray_confirmed_off"] = spray_confirmed
    result["spray_off_result"] = spray_off_result
    if spray_attempted and not spray_confirmed:
        result["spray_off_degraded"] = True
        result["success"] = False
        base = result.get("message", "") or ""
        result["message"] = (base + " (spray OFF not confirmed)").strip()
        log.warning("stop: spray OFF was not confirmed by the spray node")
    if result.get("success") and mission_capture is not None:
        mission_capture.record_terminal(
            None,
            "operator_stop",
            state=offboard_ctrl.state.value,
            details={**result, "transport": transport},
        )
    return result
