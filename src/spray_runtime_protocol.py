"""Wire-format helpers for spray runtime status and dwell commands."""

from __future__ import annotations

import json
import math
from typing import Any


RUNTIME_STATUS_TOPIC = "/spray/runtime_status"
RUNTIME_STATUS_MAX_AGE_S = 0.5


def serialize_runtime_status(status: dict[str, Any]) -> str:
    """Serialize the bounded status schema; callers own transport timing."""
    return json.dumps(status, separators=(",", ":"), sort_keys=True, allow_nan=False)


def deserialize_runtime_status(payload: str) -> dict[str, Any]:
    raw = json.loads(payload)
    if not isinstance(raw, dict):
        raise ValueError("runtime status must be a JSON object")
    required = {
        "timestamp_monotonic_s",
        "spray_mode",
        "configuration_revision",
        "model_revision",
        "ready",
        "commanded_on",
        "confirmed_off",
        "active_dwell",
        "dwell_command_id",
        "dwell_mission_id",
        "dwell_point_index",
        "dwell_remaining_s",
        "last_error",
    }
    missing = required.difference(raw)
    if missing:
        raise ValueError(f"runtime status missing fields: {sorted(missing)}")
    return raw


def serialize_dwell_command(
    *,
    revision: int,
    mission_id: str,
    point_index: int,
    command_id: int,
    duration_s: float,
    configuration_revision: int,
) -> str:
    if revision <= 0 or command_id <= 0 or point_index < 0:
        raise ValueError("invalid dwell command identity")
    if not mission_id:
        raise ValueError("mission_id is required")
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be finite and > 0")
    return json.dumps(
        {
            "revision": int(revision),
            "mission_id": mission_id,
            "point_index": int(point_index),
            "command_id": int(command_id),
            "duration_s": float(duration_s),
            "configuration_revision": int(configuration_revision),
        },
        separators=(",", ":"),
        sort_keys=True,
        allow_nan=False,
    )


def deserialize_dwell_command(payload: str) -> dict[str, Any]:
    raw = json.loads(payload)
    if not isinstance(raw, dict):
        raise ValueError("dwell command must be a JSON object")
    return {
        "revision": int(raw["revision"]),
        "mission_id": str(raw["mission_id"]),
        "point_index": int(raw["point_index"]),
        "command_id": int(raw["command_id"]),
        "duration_s": float(raw["duration_s"]),
        "configuration_revision": int(raw["configuration_revision"]),
    }


def dwell_response_message(command_id: int, deadline_monotonic_s: float) -> str:
    return json.dumps(
        {
            "accepted": True,
            "command_id": int(command_id),
            "deadline_monotonic_s": float(deadline_monotonic_s),
        },
        separators=(",", ":"),
        sort_keys=True,
    )


def parse_dwell_response(payload: str) -> dict[str, Any]:
    raw = json.loads(payload)
    if not isinstance(raw, dict) or not raw.get("accepted"):
        raise ValueError("dwell response was not accepted")
    return raw
