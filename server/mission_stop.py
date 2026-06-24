"""Unified non-resumable mission stop (REST + Socket.IO parity)."""

from __future__ import annotations

from typing import Any


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
    result = await offboard_ctrl.stop_async()
    if result.get("success") and mission_capture is not None:
        mission_capture.record_terminal(
            None,
            "operator_stop",
            state=offboard_ctrl.state.value,
            details={**result, "transport": transport},
        )
    return result