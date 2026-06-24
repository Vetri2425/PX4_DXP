"""Per-path spray mode configuration endpoints.

GET    /api/path/{name}/spray-mode              — read sidecar (or factory defaults)
DELETE /api/path/{name}/spray-mode              — reset to factory defaults
PUT    /api/path/{name}/spray-mode/continuous   — set mode=continuous + timing params
PUT    /api/path/{name}/spray-mode/dash         — set mode=dash + on/off distances
PUT    /api/path/{name}/spray-mode/point        — set mode=point + nav/dwell params

The sidecar is merged into the staged mission artifact at plan-and-stage time when
spray_mode is omitted from PathPlanRequest (None → use sidecar). Sending an
explicit spray_mode in the plan body uses the legacy path unchanged.

Safety gate: PUT and DELETE return 409 if a mission is in a live state.
"""
from __future__ import annotations

import os

from fastapi import APIRouter, Depends, HTTPException

from auth import require_token
from config import MISSION_DIR
from logging_setup import get_logger
from models import (
    ContinuousModeRequest,
    DashModeRequest,
    MissionState,
    PointModeRequest,
    SprayModeResponse,
)
from spray_mode_store import (
    delete_spray_mode,
    load_spray_mode,
    save_spray_mode,
    sidecar_exists,
)

log = get_logger("server.spray_mode")

router = APIRouter(
    prefix="/path",
    tags=["spray-mode"],
    dependencies=[Depends(require_token)],
)

_LIVE_STATES = {
    MissionState.RUNNING,
    MissionState.LOADING,
    MissionState.ARMING,
    MissionState.SWITCHING_OFFBOARD,
    MissionState.STOPPING,
    MissionState.DISARMING,
}


def _guard_live() -> None:
    from main import offboard_ctrl

    if offboard_ctrl is not None and offboard_ctrl.state in _LIVE_STATES:
        raise HTTPException(
            409, "Cannot reconfigure spray mode while a mission is active"
        )


def _safe_name(name: str) -> str:
    safe = os.path.basename(name)
    if not safe:
        raise HTTPException(422, "Invalid path name")
    return safe


def _build_response(
    name: str, *, applied: bool = False, apply_detail: str = ""
) -> SprayModeResponse:
    config = load_spray_mode(MISSION_DIR, name)
    return SprayModeResponse(
        name=name,
        spray_mode=str(config.get("spray_mode", "continuous")),
        config=config,
        has_sidecar=sidecar_exists(MISSION_DIR, name),
        applied=applied,
        apply_detail=apply_detail,
    )


async def _apply_after_save(safe_name: str, config: dict) -> tuple[bool, str]:
    """Push the just-saved sidecar config to the live spray controller, but only
    when it is safe and unambiguous to do so:

    - the spray controller (ros_node) is available,
    - the edited path is the currently *loaded* mission (basename match), and
    - no live mission is in progress (already enforced by `_guard_live`).

    Point mode is never hot-applied here: the point orchestrator caches its own
    revision-stamped config at load time, so it must be re-loaded to stay in
    sync. In all deferred cases the sidecar is still saved and takes effect on
    the next plan-and-stage + load of this path.
    """
    from main import offboard_ctrl, point_mission, ros_node

    mode = str(config.get("spray_mode", "continuous")).lower()
    if mode == "point":
        return False, "saved; point mode applies on next mission load"
    if ros_node is None:
        return False, "saved; spray controller unavailable, applies on next mission load"
    loaded = getattr(offboard_ctrl, "loaded_path_name", None)
    if not loaded or os.path.basename(str(loaded)) != safe_name:
        return False, "saved; applies when this path's mission is loaded"
    if point_mission is not None and getattr(point_mission, "is_active", lambda: False)():
        return False, "saved; a mission is active, applies on next mission load"

    # Preserve the loaded mission identity; bump the revision so the controller
    # sees a fresh config. apply_spray_mission_config force-OFFs spray first.
    from spray_mission_config import (
        apply_spray_mission_config,
        next_configuration_revision,
    )

    apply_cfg = {
        **config,
        "mission_id": getattr(offboard_ctrl, "loaded_mission_id", "") or "",
        "path_fingerprint": getattr(offboard_ctrl, "_path_fingerprint", "") or "",
    }
    try:
        ok, why, _ = await apply_spray_mission_config(
            ros_node, apply_cfg, revision=next_configuration_revision()
        )
    except Exception as exc:  # pragma: no cover - defensive
        log.exception("live spray-mode apply raised for %s", safe_name)
        return False, f"saved; live apply failed: {exc}"
    if ok:
        return True, "applied to live spray controller"
    return False, f"saved; live apply rejected: {why}"


# ── GET ───────────────────────────────────────────────────────────────────────

@router.get("/{name}/spray-mode", response_model=SprayModeResponse)
async def get_spray_mode(name: str) -> SprayModeResponse:
    """Return the saved spray mode config for this path, or factory defaults."""
    safe = _safe_name(name)
    return _build_response(safe)


# ── DELETE ────────────────────────────────────────────────────────────────────

@router.delete("/{name}/spray-mode", response_model=SprayModeResponse)
async def reset_spray_mode(name: str) -> SprayModeResponse:
    """Delete the spray mode sidecar; next plan-and-stage uses factory defaults."""
    safe = _safe_name(name)
    _guard_live()
    deleted = delete_spray_mode(MISSION_DIR, safe)
    log.info("spray-mode sidecar reset for %s (existed=%s)", safe, deleted)
    return _build_response(safe)


# ── PUT /continuous ───────────────────────────────────────────────────────────

@router.put("/{name}/spray-mode/continuous", response_model=SprayModeResponse)
async def set_continuous_mode(name: str, req: ContinuousModeRequest) -> SprayModeResponse:
    """Set spray mode to continuous and update timing/compensation parameters.

    Other mode params (dash distances, point dwell) are preserved in the sidecar
    so switching modes never destroys prior configuration.
    """
    safe = _safe_name(name)
    _guard_live()

    config = load_spray_mode(MISSION_DIR, safe)
    config["spray_mode"] = "continuous"
    config["solenoid_open_delay_s"] = req.solenoid_open_delay_s
    config["solenoid_close_delay_s"] = req.solenoid_close_delay_s
    config["on_overspray_margin_m"] = req.on_overspray_margin_m
    config["off_overspray_margin_m"] = req.off_overspray_margin_m
    config["min_spray_speed_mps"] = req.min_spray_speed_mps
    config["max_spray_speed_mps"] = req.max_spray_speed_mps
    config["unsafe_speed_behavior"] = req.unsafe_speed_behavior
    config["max_xtrack_error_m"] = req.max_xtrack_error_m
    config["nozzle_forward_offset_m"] = req.nozzle_forward_offset_m
    config["nozzle_lateral_offset_m"] = req.nozzle_lateral_offset_m
    config["calibration_profile_id"] = req.calibration_profile_id
    config["calibration_profile_version"] = req.calibration_profile_version
    config["target_paint_density"] = req.target_paint_density
    config["speed_pwm_table"] = req.speed_pwm_table
    config["actuator_min_pwm"] = req.actuator_min_pwm
    config["actuator_max_pwm"] = req.actuator_max_pwm
    config["actuator_off_pwm"] = req.actuator_off_pwm
    config["actuator_min_value"] = req.actuator_min_value
    config["actuator_max_value"] = req.actuator_max_value
    config["actuator_off_value"] = req.actuator_off_value
    config["timing_only_compatibility"] = req.timing_only_compatibility

    try:
        save_spray_mode(MISSION_DIR, safe, config)
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc

    applied, detail = await _apply_after_save(safe, config)
    log.info("spray-mode set to continuous for %s (applied=%s: %s)", safe, applied, detail)
    return _build_response(safe, applied=applied, apply_detail=detail)


# ── PUT /dash ─────────────────────────────────────────────────────────────────

@router.put("/{name}/spray-mode/dash", response_model=SprayModeResponse)
async def set_dash_mode(name: str, req: DashModeRequest) -> SprayModeResponse:
    """Set spray mode to dash and configure on/off distances."""
    safe = _safe_name(name)
    _guard_live()

    config = load_spray_mode(MISSION_DIR, safe)
    config["spray_mode"] = "dash"
    config["dash_on_distance_m"] = req.dash_on_distance_m
    config["dash_off_distance_m"] = req.dash_off_distance_m
    config["dash_phase_reset"] = req.dash_phase_reset

    try:
        save_spray_mode(MISSION_DIR, safe, config)
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc

    applied, detail = await _apply_after_save(safe, config)
    log.info(
        "spray-mode set to dash (on=%.2fm off=%.2fm) for %s (applied=%s: %s)",
        req.dash_on_distance_m,
        req.dash_off_distance_m,
        safe,
        applied,
        detail,
    )
    return _build_response(safe, applied=applied, apply_detail=detail)


# ── PUT /point ────────────────────────────────────────────────────────────────

@router.put("/{name}/spray-mode/point", response_model=SprayModeResponse)
async def set_point_mode(name: str, req: PointModeRequest) -> SprayModeResponse:
    """Set spray mode to point and configure navigation and dwell parameters."""
    safe = _safe_name(name)
    _guard_live()

    config = load_spray_mode(MISSION_DIR, safe)
    config["spray_mode"] = "point"
    config["point_default_dwell_s"] = req.point_default_dwell_s
    config["point_max_dwell_s"] = req.point_max_dwell_s
    config["point_arrival_tolerance_m"] = req.point_arrival_tolerance_m
    config["point_settle_time_s"] = req.point_settle_time_s
    config["point_leg_timeout_s"] = req.point_leg_timeout_s
    config["point_settle_speed_mps"] = req.point_settle_speed_mps
    config["point_settle_yaw_rate_rad_s"] = req.point_settle_yaw_rate_rad_s
    config["point_execution_mode"] = req.point_execution_mode
    config["point_leg_trajectory_mode"] = req.point_leg_trajectory_mode
    config["point_leg_spacing_m"] = req.point_leg_spacing_m
    config["point_hold_drift_tolerance_m"] = req.point_hold_drift_tolerance_m
    config["point_hold_drift_policy"] = req.point_hold_drift_policy

    try:
        save_spray_mode(MISSION_DIR, safe, config)
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc

    applied, detail = await _apply_after_save(safe, config)
    log.info(
        "spray-mode set to point (dwell=%.1fs) for %s (applied=%s: %s)",
        req.point_default_dwell_s,
        safe,
        applied,
        detail,
    )
    return _build_response(safe, applied=applied, apply_detail=detail)
