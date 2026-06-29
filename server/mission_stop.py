"""Unified non-resumable mission stop (REST + Socket.IO parity)."""

from __future__ import annotations

from typing import Any

from logging_setup import get_logger
from mission_ops import MissionOperation, MissionOperationCoordinator
from point_mission import PointMissionState
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


def _coordinator() -> MissionOperationCoordinator:
    try:
        from main import operation_coordinator

        if operation_coordinator is not None:
            return operation_coordinator
    except Exception:
        pass
    return MissionOperationCoordinator()


async def stop_active_mission(
    offboard_ctrl,
    point_mission,
    ros_node,
    hold_owner,
    *,
    mission_capture=None,
    transport: str = "rest",
    operation_coordinator: MissionOperationCoordinator | None = None,
) -> dict[str, Any]:
    """Cancel point mission (if any), release hold, then soft-stop controller."""
    coordinator = operation_coordinator or _coordinator()
    token = await coordinator.begin(MissionOperation.STOP, timeout_s=0.5)
    try:
        if point_mission is not None and (
            point_mission.is_active() or point_mission.is_paused()
        ):
            await point_mission.terminal_cleanup(
                ros_node,
                hold_owner,
                reason="operator_stop",
                terminal_state=PointMissionState.ABORTING,
                operation_token=token,
                require_spray_confirm=True,
            )
        if hold_owner is not None:
            hold_owner.deactivate(ros_node)
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
    finally:
        await coordinator.finish(token)