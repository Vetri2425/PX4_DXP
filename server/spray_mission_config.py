"""Atomic spray mission configuration plumbing for the FastAPI server."""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any

_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from spray_config import (  # noqa: E402
    SprayConfiguration,
    SprayMode,
    configuration_to_param_dict,
    parse_staged_spray_config,
    staged_spray_defaults,
    validate_spray_configuration,
)
from path_identity import path_geometry_fingerprint  # noqa: E402

from logging_setup import get_logger

log = get_logger("server.spray_mission_config")


def spray_fields_from_staged(staged: dict[str, Any]) -> dict[str, Any]:
    """Extract spray configuration fields from a staged mission artifact."""
    defaults = staged_spray_defaults()
    out = {key: staged.get(key, defaults[key]) for key in defaults}
    out["mission_id"] = str(staged.get("mission_id", "") or "")
    out["path_fingerprint"] = str(staged.get("path_fingerprint", "") or "")
    out["configuration_revision"] = int(staged.get("configuration_revision", 0))
    return out


def validate_staged_spray_config(staged: dict[str, Any]) -> SprayConfiguration:
    return parse_staged_spray_config(staged)


async def apply_spray_mission_config(
    ros_node,
    staged_or_raw: dict[str, Any],
    *,
    revision: int | None = None,
) -> tuple[bool, str, SprayConfiguration | None]:
    """Force spray OFF, validate, bulk-set params, and trigger node apply."""
    if ros_node is None:
        return False, "ROS node not ready", None

    raw = spray_fields_from_staged(staged_or_raw)
    if revision is not None:
        raw["configuration_revision"] = revision
    try:
        config = validate_spray_configuration(raw)
    except ValueError as exc:
        return False, str(exc), None

    # Mission configuration never changes the operator-owned master enable.
    params = configuration_to_param_dict(config)

    ok, flags, why = await ros_node.set_spray_params_bulk_async(params)
    if not ok or not all(flags):
        return False, why or "spray parameter bulk set failed", None

    ok, why = await ros_node.trigger_spray_apply_mission_config_async()
    if not ok:
        return False, why or "spray apply_mission_config failed", None

    log.info(
        "applied spray mission config mode=%s revision=%s mission_id=%s",
        config.mode.value,
        config.revision,
        config.mission_id,
    )
    return True, "applied", config


def default_backward_compatible_staged_fields() -> dict[str, Any]:
    """Fields to inject when staging legacy missions without spray_mode."""
    return staged_spray_defaults()


def is_point_mode_staged(staged: dict[str, Any]) -> bool:
    mode = str(staged.get("spray_mode", SprayMode.CONTINUOUS.value)).strip().lower()
    return mode == SprayMode.POINT.value


def next_configuration_revision() -> int:
    return time.time_ns()
