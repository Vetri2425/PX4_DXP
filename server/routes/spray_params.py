"""Spray controller parameter read/write via rcl_interfaces services.

Spray params are ROS2 parameters on the `spray_controller` node. They are
tunable at runtime — no node restart required. The node name is
`spray_controller` (see config.SRV_SPRAY_GET_PARAMS / SRV_SPRAY_SET_PARAMS).

Endpoints
---------
GET    /api/spray/params            — list all params with schema + live values
GET    /api/spray/params/schema     — param metadata only (no ROS calls)
GET    /api/spray/params/{name}     — get single param value from ROS
PUT    /api/spray/params/{name}     — set single param value
PUT    /api/spray/params            — bulk-set multiple params
"""

from __future__ import annotations

import datetime
from typing import Any

from fastapi import APIRouter, Depends, HTTPException

from auth import require_token
from logging_setup import get_logger
from models import (
    RppParamGetResponse as SprayParamGetResponse,
    RppParamInfo as SprayParamInfo,
    RppParamListResponse as SprayParamListResponse,
    RppParamSetBulkRequest as SprayParamSetBulkRequest,
    RppParamSetBulkResponse as SprayParamSetBulkResponse,
    RppParamSetRequest as SprayParamSetRequest,
    RppParamSetResponse as SprayParamSetResponse,
)

log = get_logger("server.spray_params")

router = APIRouter(
    prefix="/spray/params", tags=["spray_params"], dependencies=[Depends(require_token)]
)


def _record(level: str, message: str) -> None:
    from main import activity_log

    ts = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")
    activity_log.append({"timestamp": ts, "level": level, "message": message})
    getattr(log, level if level in ("info", "warning", "error", "debug") else "info")(
        message
    )


# ── Parameter Schema ──────────────────────────────────────────────────────────
# Operator-facing subset of spray_controller_node declare_parameter() calls.
# Internal params (pose_timeout_s, velocity_timeout_s, reassert_hz, etc.) are
# omitted — they are not runtime-tunable by operators in the field.

SPRAY_PARAM_SCHEMA: dict[str, dict] = {
    # ── Actuator Backend ──────────────────────────────────────────────────────
    "actuator_backend": {
        "type": "string",
        "default": "mavlink_actuator",
        "group": "Actuator Backend",
        "description": (
            'Backend: "mavlink_actuator" (cmd 187, normalized -1/+1) or '
            '"mavlink_servo_pwm" (cmd 183, absolute PWM µs). '
            "Validated production config uses mavlink_actuator."
        ),
    },
    "actuator_set_index": {
        "type": "int",
        "default": 1,
        "group": "Actuator Backend",
        "description": "Actuator slot index 1–6 for mavlink_actuator backend (cmd 187 param position)",
        "min": 1,
        "max": 6,
    },
    "on_value": {
        "type": "float",
        "default": 1.0,
        "group": "Actuator Backend",
        "description": (
            "Normalized ON value for mavlink_actuator backend. "
            "1.0 → 3000 µs with PWM_AUX_MAX1=3000 in QGC (full flow, field-confirmed)"
        ),
        "min": -1.0,
        "max": 1.0,
    },
    "off_value": {
        "type": "float",
        "default": -1.0,
        "group": "Actuator Backend",
        "description": (
            "Normalized OFF value for mavlink_actuator backend. "
            "-1.0 → 0 µs with PWM_AUX_MIN1=0 in QGC (motor fully stopped, field-confirmed)"
        ),
        "min": -1.0,
        "max": 1.0,
    },
    "servo_instance": {
        "type": "int",
        "default": 1,
        "group": "Actuator Backend",
        "description": "Servo instance for mavlink_servo_pwm backend (cmd 183 param1). Validate AUX pin in QGC",
        "min": 1,
        "max": 8,
    },
    "on_pwm_us": {
        "type": "int",
        "default": 1800,
        "group": "Actuator Backend",
        "description": "Absolute PWM ON value in µs for mavlink_servo_pwm backend",
        "min": 0,
        "max": 2200,
    },
    "off_pwm_us": {
        "type": "int",
        "default": 0,
        "group": "Actuator Backend",
        "description": "Absolute PWM OFF value in µs for mavlink_servo_pwm backend",
        "min": 0,
        "max": 2200,
    },
    # ── Distance-Aware Spray ──────────────────────────────────────────────────
    "use_distance_aware_spray": {
        "type": "bool",
        "default": True,
        "group": "Distance-Aware Spray",
        "description": "Use path-geometry-based ON/OFF decisions instead of legacy /spray/active topic",
    },
    "solenoid_open_delay_s": {
        "type": "float",
        "default": 0.10,
        "group": "Distance-Aware Spray",
        "description": "Spray ON is commanded this many seconds early to compensate solenoid/pump open lag",
        "min": 0.0,
        "max": 1.0,
    },
    "solenoid_close_delay_s": {
        "type": "float",
        "default": 0.05,
        "group": "Distance-Aware Spray",
        "description": "Spray OFF is commanded this many seconds early to compensate solenoid/pump close lag",
        "min": 0.0,
        "max": 1.0,
    },
    "on_overspray_margin_m": {
        "type": "float",
        "default": 0.02,
        "group": "Distance-Aware Spray",
        "description": "Extra lead distance (m) added to ON anticipation so the MARK start is not cut short",
        "min": 0.0,
        "max": 0.5,
    },
    "off_overspray_margin_m": {
        "type": "float",
        "default": 0.0,
        "group": "Distance-Aware Spray",
        "description": "Extra delay distance (m) added to OFF so the MARK tail is not cut short",
        "min": 0.0,
        "max": 0.5,
    },
    "min_spray_speed_mps": {
        "type": "float",
        "default": 0.05,
        "group": "Distance-Aware Spray",
        "description": "Spray is suppressed when rover speed falls below this (m/s). Prevents static over-spray",
        "min": 0.0,
        "max": 1.0,
    },
    "max_xtrack_error_m": {
        "type": "float",
        "default": 0.10,
        "group": "Distance-Aware Spray",
        "description": "Spray is suppressed when cross-track error exceeds this (m). Prevents off-path marking",
        "min": 0.01,
        "max": 1.0,
    },
    "nozzle_forward_offset_m": {
        "type": "float",
        "default": 0.0,
        "group": "Distance-Aware Spray",
        "description": "Forward body-frame offset of the spray nozzle from the GPS antenna (m)",
        "min": -2.0,
        "max": 2.0,
    },
    "nozzle_lateral_offset_m": {
        "type": "float",
        "default": 0.0,
        "group": "Distance-Aware Spray",
        "description": "Lateral body-frame offset of the spray nozzle (m, positive = rover-right)",
        "min": -2.0,
        "max": 2.0,
    },
    # ── Safety ────────────────────────────────────────────────────────────────
    "require_offboard": {
        "type": "bool",
        "default": True,
        "group": "Safety",
        "description": (
            "Require OFFBOARD mode for autonomous spray. "
            "Manual override (/spray/on) only requires armed — this gate applies to auto spray only"
        ),
    },
    "manual_override_timeout_s": {
        "type": "float",
        "default": 10.0,
        "group": "Safety",
        "description": (
            "Max duration (s) a manual spray ON hold stays active on the node side. "
            "Server keepalive re-asserts every 8s to beat this timeout during an active hold"
        ),
        "min": 1.0,
        "max": 60.0,
    },
    "debounce_samples": {
        "type": "int",
        "default": 3,
        "group": "Safety",
        "description": "Consecutive same-state samples required before changing auto-spray state. 1 disables debounce",
        "min": 1,
        "max": 20,
    },
}


# ── Helpers ───────────────────────────────────────────────────────────────────

def _roster() -> list[str]:
    return list(SPRAY_PARAM_SCHEMA.keys())


def _merge_schema_with_values(values: dict[str, Any] | None) -> list[SprayParamInfo]:
    result: list[SprayParamInfo] = []
    for name in _roster():
        entry = SPRAY_PARAM_SCHEMA[name]
        result.append(
            SprayParamInfo(
                name=name,
                type=entry["type"],
                default=entry["default"],
                current=values.get(name) if values else None,
                group=entry["group"],
                description=entry["description"],
                min=entry.get("min"),
                max=entry.get("max"),
            )
        )
    return result


# ── Endpoints ─────────────────────────────────────────────────────────────────

@router.get("")
async def list_spray_params() -> SprayParamListResponse:
    """List all spray controller params with schema metadata and live values."""
    from main import ros_node

    params = _merge_schema_with_values(None)
    if ros_node is not None:
        try:
            ok, values, _ = await ros_node.get_spray_params_bulk_async(_roster())
            if ok and values:
                params = _merge_schema_with_values(values)
        except Exception:
            log.warning("Spray param bulk-get failed — serving schema-only", exc_info=True)
            _record("warning", "Spray param bulk-get failed, serving schema without live values")

    return SprayParamListResponse(parameters=params, count=len(params))


@router.get("/schema")
async def get_spray_param_schema() -> SprayParamListResponse:
    """Return param metadata only — no ROS calls. Works when node is offline."""
    params = _merge_schema_with_values(None)
    return SprayParamListResponse(parameters=params, count=len(params))


@router.get("/{name}")
async def get_spray_param(name: str) -> SprayParamGetResponse:
    """Get a single spray controller parameter's current value."""
    if name not in SPRAY_PARAM_SCHEMA:
        known = ", ".join(_roster())
        raise HTTPException(422, f"Unknown spray param '{name}'. Known: {known}")

    from main import ros_node

    if ros_node is None:
        raise HTTPException(503, "ROS node not ready")

    ok, value, why = await ros_node.get_spray_param_async(name)
    if not ok:
        raise HTTPException(503, why or "spray_controller not reachable")
    return SprayParamGetResponse(name=name, value=value)


@router.put("/{name}")
async def set_spray_param(name: str, req: SprayParamSetRequest) -> SprayParamSetResponse:
    """Set a single spray controller parameter at runtime. Takes effect immediately."""
    if name not in SPRAY_PARAM_SCHEMA:
        known = ", ".join(_roster())
        raise HTTPException(422, f"Unknown spray param '{name}'. Known: {known}")

    from main import ros_node

    if ros_node is None:
        raise HTTPException(503, "ROS node not ready")

    ok, why = await ros_node.set_spray_param_async(name, req.value)
    if not ok:
        _record("error", f"Spray param set {name}={req.value} failed: {why}")
        raise HTTPException(400, why or "Spray param set failed")
    _record("info", f"Spray param set {name}={req.value}")
    return SprayParamSetResponse(name=name, value=req.value, ok=True)


@router.put("")
async def set_spray_params_bulk(req: SprayParamSetBulkRequest) -> SprayParamSetBulkResponse:
    """Set multiple spray controller parameters atomically."""
    unknown = [k for k in req.parameters if k not in SPRAY_PARAM_SCHEMA]
    if unknown:
        known = ", ".join(_roster())
        raise HTTPException(
            422,
            f"Unknown spray param(s): {', '.join(unknown)}. Known: {known}",
        )

    from main import ros_node

    if ros_node is None:
        raise HTTPException(503, "ROS node not ready")

    ok, flags, why = await ros_node.set_spray_params_bulk_async(req.parameters)
    if not ok:
        _record("error", f"Spray bulk set failed: {why}")
        raise HTTPException(400, why or "Spray bulk param set failed")

    result = dict(zip(req.parameters.keys(), flags))
    n_ok = sum(1 for v in result.values() if v)
    _record("info", f"Spray bulk set: {n_ok}/{len(result)} params OK")
    return SprayParamSetBulkResponse(parameters=result, ok=all(flags))
