"""Shared mission path loading guards."""

from __future__ import annotations

import asyncio
from typing import Optional

from config import POSE_STALE_MS, SPRAY_DEFAULT_ON
from models import MissionState

MIN_MISSION_POINTS = 2
LOAD_ALLOWED_STATES = {
    MissionState.IDLE,
    MissionState.COMPLETED,
    MissionState.ABORTED,
    MissionState.ERROR,
}

_load_lock = asyncio.Lock()


class MissionLoadConflict(Exception):
    """Raised when lifecycle state makes mission loading unsafe."""


def load_block_reason(state: MissionState) -> Optional[str]:
    if state in LOAD_ALLOWED_STATES:
        return None
    return f"Cannot load mission while controller state is {state.value}"


def validate_point_count(points: list[tuple[float, float]]) -> None:
    if len(points) < MIN_MISSION_POINTS:
        raise ValueError(
            f"Mission path must contain at least {MIN_MISSION_POINTS} points "
            f"(got {len(points)})"
        )


def spray_flags_for_path(path_mgr, name: str, points_len: int) -> list[bool]:
    """Return per-point spray flags for a loaded path, or a configured legacy default."""
    try:
        preview = path_mgr.preview_path(name)
        flags = [bool(wp.spray) for wp in preview.waypoints]
    except Exception:
        flags = [SPRAY_DEFAULT_ON] * points_len
    if len(flags) != points_len:
        flags = [SPRAY_DEFAULT_ON] * points_len
    return flags


def pose_origin_or_error(state: dict) -> tuple[float, float] | str:
    if not state.get("pose_received", False):
        return "auto_origin requested but no local pose received yet"
    pose_age_raw = state.get("pose_age_ms")
    if pose_age_raw is None:
        pose_age = POSE_STALE_MS + 1.0
    else:
        pose_age = float(pose_age_raw)
    if pose_age > POSE_STALE_MS:
        return (
            f"auto_origin requested but local pose is stale "
            f"({pose_age:.0f} ms > {POSE_STALE_MS:.0f} ms)"
        )
    return (float(state.get("pos_n", 0.0)), float(state.get("pos_e", 0.0)))


async def load_path_for_controller(
    offboard_ctrl,
    path_mgr,
    name: str,
    *,
    origin: tuple[float, float] = (0.0, 0.0),
    start_position: tuple[float, float] | None = None,
) -> list[tuple[float, float]]:
    """Load and validate a path without blocking the FastAPI event loop."""
    async with _load_lock:
        prior_state = offboard_ctrl.state
        reason = load_block_reason(prior_state)
        if reason:
            raise MissionLoadConflict(reason)

        offboard_ctrl.state = MissionState.LOADING
        try:
            points = await asyncio.to_thread(
                path_mgr.load_path,
                name,
                origin=origin,
                start_position=start_position,
            )
            validate_point_count(points)
            spray_flags = spray_flags_for_path(path_mgr, name, len(points))
        except Exception:
            offboard_ctrl.state = prior_state
            raise

        offboard_ctrl.state = prior_state
        offboard_ctrl.load_path(points, name=name, spray_flags=spray_flags)
        return points
