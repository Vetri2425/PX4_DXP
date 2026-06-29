"""Manual spray servo control — enable gate, bench test, and operator ON/OFF.

Endpoints
---------
POST /api/spray/enable  — lift the master spray gate; no motor action
POST /api/spray/disable — close the master spray gate; stops any active spray
POST /api/spray/on      — hold spray motor ON (requires enabled + armed)
POST /api/spray/off     — stop spray motor; always safe, no gate
POST /api/spray/test    — timed bench test: ON with auto-off (requires enabled)
GET  /api/spray/status  — enabled state, actual vs desired vs manual-override

Safety model (server layer; node and firmware layers sit beneath):
- /spray/enable / /spray/disable: master gate. When disabled, /on and /test ON
  are blocked (409). The node's spray_enabled parameter is also set so
  autonomous mission spray is suppressed at the actuator level.
- /spray/on requires enabled + armed + no RUNNING mission.
- /spray/off is always accepted — OFF is unconditionally safe.
- /spray/disable always succeeds and immediately stops any active spray/hold.
- spray_controller_node's disarm / mode-loss / shutdown fail-safes outrank
  every server-side command entirely.
"""
from __future__ import annotations

import asyncio
import math
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException

from auth import require_token
from logging_setup import get_logger
from models import MissionState, SprayTestRequest
from spray_safety import (
    CANONICAL_SPRAY_TELEMETRY_FIELDS,
    SprayOffResult,
    build_spray_telemetry_fields,
    force_spray_off_confirmed,
    spray_off_blocks_success,
    wait_for_spray_on_acknowledged,
)

log = get_logger("server.spray")

router = APIRouter(prefix="/spray", tags=["spray"],
                   dependencies=[Depends(require_token)])

MAX_SPRAY_TEST_DURATION_S = 10.0
DEFAULT_SPRAY_TEST_DURATION_S = 3.0
SPRAY_ON_ACK_TIMEOUT_S = 2.0

# Must be less than spray_controller_node's manual_override_timeout_s (default
# 10s) so the hold stays alive. 8s gives a 2s margin before the node times out.
KEEPALIVE_INTERVAL_S = 8.0

# Master enable gate. False = spray system disabled; /on and /test ON are
# blocked; node param spray_enabled is also set to False so autonomous mission
# spray cannot fire. Default False — operator must explicitly enable before use.
_spray_enabled: bool = False

_auto_off_task: Optional[asyncio.Task] = None
_keepalive_task: Optional[asyncio.Task] = None


# ── Task management ───────────────────────────────────────────────────────────

def _cancel_auto_off() -> None:
    global _auto_off_task
    if _auto_off_task is not None and not _auto_off_task.done():
        _auto_off_task.cancel()
    _auto_off_task = None


def _cancel_keepalive() -> None:
    global _keepalive_task
    if _keepalive_task is not None and not _keepalive_task.done():
        _keepalive_task.cancel()
    _keepalive_task = None


def _cancel_all() -> None:
    """Cancel both the bench-test auto-off timer and the hold keepalive."""
    _cancel_auto_off()
    _cancel_keepalive()


# ── Background coroutines ─────────────────────────────────────────────────────

async def _auto_off_after(duration_s: float) -> None:
    """Publish manual OFF after the test window. Cancellation = superseded."""
    try:
        await asyncio.sleep(duration_s)
    except asyncio.CancelledError:
        return
    from main import ros_node
    if ros_node is not None:
        await force_spray_off_confirmed(ros_node, timeout_s=2.0)


async def _keepalive_loop() -> None:
    """Re-publish manual ON every KEEPALIVE_INTERVAL_S so the node's
    manual_override_timeout_s backstop never fires while a hold is active.
    Cancelled by spray_off() or a superseding spray_test() call."""
    from main import ros_node
    try:
        while True:
            await asyncio.sleep(KEEPALIVE_INTERVAL_S)
            if ros_node is not None:
                ros_node.publish_spray_manual(True)
    except asyncio.CancelledError:
        return


# ── Helpers ───────────────────────────────────────────────────────────────────

def _spray_off_route_payload(
    result: SprayOffResult,
    *,
    extra: dict | None = None,
) -> dict:
    payload = {
        "confirmed_off": result.success,
        "off_acknowledged": result.command_off_acknowledged,
        "physical_off_confirmed": result.physical_off_confirmed,
        "recovery_required": result.recovery_required,
        "recovery_reason": result.failure_reason or result.message,
        "spray_off_result": result.as_dict(),
    }
    if extra:
        payload.update(extra)
    if spray_off_blocks_success(result):
        raise HTTPException(
            503,
            detail={
                "message": f"spray OFF not confirmed: {result.message}",
                **payload,
            },
        )
    return payload


def _check_spray_enabled() -> None:
    """Raise 409 if the spray system has not been enabled."""
    if not _spray_enabled:
        raise HTTPException(
            409,
            "Spray system is disabled — call POST /api/spray/enable first",
        )


# ── Endpoints ─────────────────────────────────────────────────────────────────

@router.post("/enable")
async def spray_enable():
    """Lift the master spray gate.

    No motor action. Allows /spray/on and /spray/test to reach the actuator.
    Also sets spray_enabled=True on the spray_controller node so autonomous
    mission spray can fire during path execution.
    """
    global _spray_enabled
    from main import ros_node
    if ros_node is None:
        raise HTTPException(503, "ROS bridge not ready")
    try:
        ok, why = await ros_node.set_spray_param_async("spray_enabled", True)
    except Exception as exc:
        raise HTTPException(503, f"Could not enable spray controller: {exc}") from exc
    if not ok:
        raise HTTPException(503, why or "Could not enable spray controller")
    _spray_enabled = True
    return {"enabled": True}


@router.post("/disable")
async def spray_disable():
    """Close the master spray gate and stop any active spray immediately.

    Cancels any active hold or bench-test timer, publishes manual OFF to the
    node, and sets spray_enabled=False on the node so autonomous mission spray
    is also suppressed. Always succeeds — disabling is always safe.
    """
    global _spray_enabled
    _spray_enabled = False
    _cancel_all()
    from main import ros_node
    if ros_node is not None:
        try:
            await ros_node.set_spray_param_async("spray_enabled", False)
        except Exception:
            log.warning("Could not set node spray_enabled=False; server gate active", exc_info=True)
        result = await force_spray_off_confirmed(ros_node, timeout_s=2.0)
        if not result.success and result.live:
            log.warning("spray disable: OFF not confirmed: %s", result.as_dict())
        return _spray_off_route_payload(result, extra={"enabled": False})
    return {"enabled": False, "confirmed_off": None, "spray_off_result": None}


@router.post("/on")
async def spray_on():
    """Hold spray ON until POST /api/spray/off is called.

    Publishes manual ON immediately and starts a keepalive that re-asserts
    every 8 s so the node's 10s timeout never fires during an active hold.
    Rejected while a mission is RUNNING or the FCU is disarmed.
    """
    from main import offboard_ctrl, ros_node
    global _keepalive_task

    _check_spray_enabled()
    if ros_node is None:
        raise HTTPException(503, "ROS bridge not ready")
    if offboard_ctrl is not None and offboard_ctrl.state == MissionState.RUNNING:
        raise HTTPException(409, "Manual spray is blocked while a mission is RUNNING")
    state = ros_node.get_state()
    if not bool(state.get("armed", False)):
        raise HTTPException(
            409,
            "Spray ON requires an armed FCU — the AUX output holds its "
            "DISARMED (OFF) PWM while disarmed",
        )

    _cancel_all()
    ros_node.publish_spray_manual(True)
    ack = await wait_for_spray_on_acknowledged(
        ros_node, timeout_s=SPRAY_ON_ACK_TIMEOUT_S
    )
    if not ack.success:
        off_result = await force_spray_off_confirmed(ros_node, timeout_s=2.0)
        detail = {
            "message": ack.message,
            "requested_on": ack.requested_on,
            "pending_on": ack.pending_on,
            "accepted_on": ack.accepted_on,
            "commanded_on": ack.commanded_on,
            "spraying": False,
            "hold": False,
            "recovery_required": ack.recovery_required or off_result.recovery_required,
            "failure_reason": ack.failure_reason or ack.message,
            "spray_off_result": off_result.as_dict(),
        }
        raise HTTPException(503 if ack.timeout or ack.recovery_required else 409, detail=detail)
    _keepalive_task = asyncio.create_task(_keepalive_loop())
    return {
        "requested_on": ack.requested_on,
        "pending_on": ack.pending_on,
        "accepted_on": ack.accepted_on,
        "commanded_on": ack.commanded_on,
        "spraying": ack.accepted_on,
        "hold": ack.accepted_on,
    }


@router.post("/off")
async def spray_off():
    """Turn spray OFF immediately and cancel any active hold or bench-test timer.

    Always succeeds — OFF is unconditionally safe.
    """
    from main import ros_node
    _cancel_all()
    if ros_node is not None:
        result = await force_spray_off_confirmed(ros_node, timeout_s=2.0)
        if not result.success and result.live:
            log.warning("spray off: OFF not confirmed: %s", result.as_dict())
        return _spray_off_route_payload(
            result,
            extra={"spraying": False, "hold": False},
        )
    return {
        "spraying": False,
        "hold": False,
        "confirmed_off": None,
        "spray_off_result": None,
    }


@router.post("/test")
async def spray_test(req: SprayTestRequest):
    """Bench-test spray override: ON with timed auto-off, or immediate cancel.

    ON is capped at MAX_SPRAY_TEST_DURATION_S (10s). Use /spray/on for an
    indefinite hold. Cancels any active hold keepalive before starting.
    """
    from main import offboard_ctrl, ros_node
    global _auto_off_task

    if ros_node is None:
        raise HTTPException(503, "ROS bridge not ready")

    if not req.on:
        _cancel_all()
        result = await force_spray_off_confirmed(ros_node, timeout_s=2.0)
        if not result.success and result.live:
            log.warning("spray test OFF: OFF not confirmed: %s", result.as_dict())
        return {
            "manual": False,
            "confirmed_off": result.success,
            "spray_off_result": result.as_dict(),
        }

    if not req.diagnostic_authorized:
        raise HTTPException(
            403,
            "Bench spray test requires diagnostic_authorized=true",
        )
    _check_spray_enabled()
    if offboard_ctrl is not None and offboard_ctrl.state == MissionState.RUNNING:
        raise HTTPException(409, "Manual spray is blocked while a mission is RUNNING")
    state = ros_node.get_state()
    if not bool(state.get("armed", False)):
        raise HTTPException(
            409,
            "Manual spray requires an armed FCU — the AUX output holds its "
            "DISARMED (OFF) PWM while disarmed",
        )

    duration = (
        DEFAULT_SPRAY_TEST_DURATION_S
        if req.duration_s is None
        else float(req.duration_s)
    )
    if not math.isfinite(duration) or duration <= 0.0:
        raise HTTPException(400, "duration_s must be a positive number")
    duration = min(duration, MAX_SPRAY_TEST_DURATION_S)

    _cancel_all()
    ros_node.publish_spray_manual(True)
    ack = await wait_for_spray_on_acknowledged(
        ros_node, timeout_s=SPRAY_ON_ACK_TIMEOUT_S
    )
    if not ack.success:
        off_result = await force_spray_off_confirmed(ros_node, timeout_s=2.0)
        detail = {
            "message": ack.message,
            "manual": False,
            "requested_on": ack.requested_on,
            "pending_on": ack.pending_on,
            "accepted_on": ack.accepted_on,
            "commanded_on": ack.commanded_on,
            "recovery_required": ack.recovery_required or off_result.recovery_required,
            "failure_reason": ack.failure_reason or ack.message,
            "spray_off_result": off_result.as_dict(),
        }
        raise HTTPException(503 if ack.timeout or ack.recovery_required else 409, detail=detail)
    try:
        _auto_off_task = asyncio.create_task(_auto_off_after(duration))
        return {
            "manual": True,
            "duration_s": duration,
            "requested_on": ack.requested_on,
            "pending_on": ack.pending_on,
            "accepted_on": ack.accepted_on,
            "commanded_on": ack.commanded_on,
        }
    except Exception:
        off_result = await force_spray_off_confirmed(ros_node, timeout_s=2.0)
        if not off_result.success and off_result.live:
            raise HTTPException(
                503,
                detail={
                    "message": "spray test failed and OFF not confirmed",
                    "recovery_required": True,
                    "spray_off_result": off_result.as_dict(),
                },
            ) from None
        raise


def _point_status_payload(point_mission) -> dict:
    if point_mission is None:
        return {
            "point_ready": False,
            "point_active_dwell": False,
            "point_dwell_remaining_s": 0.0,
            "point_last_transition": "",
            "point_last_error": "",
        }
    status = point_mission.status
    if hasattr(status, "as_spray_status_dict"):
        return status.as_spray_status_dict()
    raw = status.as_dict()
    return {
        "point_ready": bool(raw.get("ready", False)),
        "point_active_dwell": bool(raw.get("active_dwell", False)),
        "point_dwell_remaining_s": float(raw.get("dwell_remaining_s", 0.0)),
        "point_last_transition": raw.get("last_transition", ""),
        "point_last_error": raw.get("last_error", ""),
        **{
            key: value
            for key, value in raw.items()
            if key
            not in {
                "ready",
                "active_dwell",
                "dwell_remaining_s",
                "last_transition",
                "last_error",
                "hold_active",
            }
        },
    }


@router.get("/status")
async def spray_status():
    """Enabled gate, spray runtime state, manual override, and point status."""
    from main import offboard_ctrl, point_mission, ros_node
    hold_active = _keepalive_task is not None and not _keepalive_task.done()
    if ros_node is None:
        payload = {
            "enabled": _spray_enabled,
            "spraying": False,
            "marking_state": "off",
            "spray_active_desired": False,
            "manual_override": False,
            "hold_active": hold_active,
            "spray_mode": "continuous",
            "active_mode": "continuous",
            "configuration_revision": 0,
            "model_revision": 0,
            # Legacy generic fields are deterministic spray-runtime aliases.
            "ready": False,
            "active_dwell": False,
            "dwell_remaining_s": 0.0,
            "last_transition": "",
            "last_error": "",
            "spray_ready": False,
            "spray_active_dwell": False,
            "spray_dwell_remaining_s": 0.0,
            "spray_last_transition": "",
            "spray_last_error": "",
            "spray_state": "UNKNOWN",
            "spray_desired_on": False,
            "spray_pending_command": False,
            "spray_accepted_command_on": False,
            "spray_accepted_command_off": False,
            "spray_off_acknowledged": False,
            "spray_physical_feedback_supported": False,
            "spray_physical_actuator_state": "UNAVAILABLE",
            "spray_runtime_status_age_s": None,
            "spray_faulted": False,
            "spray_recovery_required": False,
            "spray_last_command_result": "unknown",
            "spray_last_command_reason": "",
        }
        payload.update(_point_status_payload(point_mission))
        return payload
    s = ros_node.get_state()
    runtime = ros_node.get_spray_runtime_status()
    mission_running = (
        offboard_ctrl is not None
        and offboard_ctrl.state == MissionState.RUNNING
        and bool(s.get("armed", False))
    )
    mission_dash = None
    if offboard_ctrl is not None and hasattr(offboard_ctrl, "loaded_path_summary"):
        summary = offboard_ctrl.loaded_path_summary()
        mission_dash = {
            "dash_feasible": summary.get("dash_feasible"),
            "dash_feasibility_reason": summary.get("dash_feasibility_reason"),
            "shortest_dash_on_run_m": summary.get("shortest_dash_on_run_m"),
            "shortest_dash_off_gap_m": summary.get("shortest_dash_off_gap_m"),
            "dash_phase_reset": summary.get("dash_phase_reset"),
            "dash_expected_speed_mps": summary.get("dash_expected_speed_mps"),
            "dash_feasibility_speed_source": summary.get(
                "dash_feasibility_speed_source"
            ),
        }
    spray_fields = build_spray_telemetry_fields(
        legacy_spraying=bool(s.get("spraying", False)),
        spray_rt=runtime,
        mission_running=mission_running,
        mission_dash=mission_dash,
    )
    spray_ready = bool(runtime.get("ready", False)) and not runtime.get(
        "status_stale", True
    )
    spray_active_dwell = bool(runtime.get("active_dwell", False))
    spray_dwell_remaining_s = float(runtime.get("dwell_remaining_s", 0.0))
    spray_last_transition = runtime.get("last_transition", "")
    spray_last_error = runtime.get("last_error", "")
    payload = {
        "enabled": _spray_enabled,
        "spraying": bool(spray_fields["spraying"]),
        "marking_state": spray_fields["marking_state"],
        "spray_active_desired": bool(s.get("spray_active", False)),
        "manual_override": bool(s.get("spray_manual", False)),
        "hold_active": hold_active,
        "spray_mode": runtime.get("spray_mode", "continuous"),
        "active_mode": runtime.get("spray_mode", "continuous"),
        "configuration_revision": int(runtime.get("configuration_revision", 0)),
        "model_revision": int(runtime.get("model_revision", 0)),
        # Legacy generic fields are deterministic spray-runtime aliases.
        "ready": spray_ready,
        "node_operator_enabled": bool(runtime.get("operator_enabled", False)),
        "active_dwell": spray_active_dwell,
        "dwell_remaining_s": spray_dwell_remaining_s,
        "commanded_on": bool(spray_fields["commanded_on"]),
        "confirmed_off": bool(runtime.get("confirmed_off", False)),
        "status_age_s": runtime.get("status_age_s"),
        "status_stale": runtime.get("status_stale", True),
        "last_transition": spray_last_transition,
        "last_error": spray_last_error,
        "spray_ready": spray_ready,
        "spray_active_dwell": spray_active_dwell,
        "spray_dwell_remaining_s": spray_dwell_remaining_s,
        "spray_last_transition": spray_last_transition,
        "spray_last_error": spray_last_error,
        "spray_state": spray_fields["spray_state"],
        "spray_desired_on": spray_fields["desired_on"],
        "spray_pending_command": spray_fields["pending_command"],
        "spray_accepted_command_on": spray_fields["accepted_command_on"],
        "spray_accepted_command_off": spray_fields["accepted_command_off"],
        "spray_off_acknowledged": spray_fields["off_acknowledged"],
        "spray_physical_feedback_supported": spray_fields[
            "physical_feedback_supported"
        ],
        "spray_physical_actuator_state": spray_fields["physical_actuator_state"],
        "spray_runtime_status_age_s": spray_fields["spray_runtime_status_age_s"],
        "spray_faulted": spray_fields["spray_faulted"],
        "spray_recovery_required": spray_fields["spray_recovery_required"],
        "spray_last_command_result": spray_fields["last_spray_command_result"],
        "spray_last_command_reason": spray_fields["last_spray_command_reason"],
    }
    for key in CANONICAL_SPRAY_TELEMETRY_FIELDS:
        if key in spray_fields and key not in payload:
            payload[key] = spray_fields[key]
    payload.update(_point_status_payload(point_mission))
    return payload
