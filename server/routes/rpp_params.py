"""RPP controller parameter read/write via rcl_interfaces services.

RPP params are ROS2 parameters on the `rpp_controller` node. They are distinct
from PX4 FCU params (which go through MAVROS) — these live entirely in the
controller process and are tunable at runtime via standard `rcl_interfaces`
services.

Endpoints
---------
GET    /api/rpp/params            — list all params with schema + current values
GET    /api/rpp/params/schema     — param metadata only (no ROS calls)
GET    /api/rpp/params/{name}     — get single param value from ROS
PUT    /api/rpp/params/{name}     — set single param value
PUT    /api/rpp/params            — bulk-set multiple params
"""

from __future__ import annotations

import datetime
from typing import Any

from fastapi import APIRouter, Depends, HTTPException

from auth import require_token
from logging_setup import get_logger
from models import (
    RppParamGetResponse,
    RppParamInfo,
    RppParamListResponse,
    RppParamSetBulkRequest,
    RppParamSetBulkResponse,
    RppParamSetRequest,
    RppParamSetResponse,
)

log = get_logger("server.rpp_params")

router = APIRouter(
    prefix="/rpp/params", tags=["rpp_params"], dependencies=[Depends(require_token)]
)


def _record(level: str, message: str) -> None:
    from main import activity_log

    ts = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")
    activity_log.append({"timestamp": ts, "level": level, "message": message})
    getattr(log, level if level in ("info", "warning", "error", "debug") else "info")(
        message
    )


# ── Parameter Schema (server-side metadata) ───────────────────────────────────
# This is the source of truth for type, group, description, and bounds.
# Parameters from rpp_controller_node.py are listed here, organised
# into functional groups matching the controller code's docstring sections.
#
# The frontend uses this schema to render the correct input controls (slider,
# toggle, number field), show human-readable descriptions, and enforce
# client-side min/max validation before sending a PUT.
#
# Fields per entry:
#   type   — "float" | "int" | "bool" | "string"
#   default — factory default value (from declare_parameter in controller)
#   group   — UI grouping label
#   description — one-line purpose
#   min, max — bounds (only for numeric types; None = unbounded)
#
# All runtime-tunable params use ROS2 declare_parameter() without descriptors,
# so describe_parameters service would return empty descriptions. Storing the
# metadata here avoids coupling the route to the running node's descriptor
# content and works even when the RPP controller is offline (schema endpoint).

RPP_PARAM_SCHEMA: dict[str, dict] = {
    # ── RPP Geometry ───────────────────────────────────────────────────────────
    "max_linear_vel": {
        "type": "float",
        "default": 0.8,
        "group": "RPP Geometry",
        "description": "Hardware ceiling for linear velocity (m/s)",
        "min": 0.0,
        "max": 3.0,
    },
    "min_linear_vel": {
        "type": "float",
        "default": 0.15,
        "group": "RPP Geometry",
        "description": "Minimum linear velocity floor (m/s)",
        "min": 0.0,
        "max": 1.0,
    },
    "min_lookahead_dist": {
        "type": "float",
        "default": 0.52,
        "group": "RPP Geometry",
        "description": "Minimum lookahead distance (m)",
        "min": 0.1,
        "max": 5.0,
    },
    "max_lookahead_dist": {
        "type": "float",
        "default": 1.0,
        "group": "RPP Geometry",
        "description": "Maximum lookahead distance (m)",
        "min": 0.1,
        "max": 10.0,
    },
    "lookahead_time": {
        "type": "float",
        "default": 1.6,
        "group": "RPP Geometry",
        "description": "Lookahead time (s); used for closed-loop velocity-scaled L_d",
        "min": 0.1,
        "max": 10.0,
    },
    # ── Curvature Regulation (P4.1) ────────────────────────────────────────────
    "a_lat_max": {
        "type": "float",
        "default": 0.3,
        "group": "Curvature Regulation",
        "description": "Lateral acceleration constraint (m/s²). v ≤ sqrt(a_lat / |κ|)",
        "min": 0.05,
        "max": 2.0,
    },
    "regulated_linear_scaling_min_speed": {
        "type": "float",
        "default": 0.3,
        "group": "Curvature Regulation",
        "description": "Floor for curvature-regulated speed (m/s)",
        "min": 0.0,
        "max": 2.0,
    },
    # ── Goal Handling ──────────────────────────────────────────────────────────
    "xy_goal_tolerance": {
        "type": "float",
        "default": 0.02,
        "group": "Goal Handling",
        "description": "Goal position tolerance (m). 2 cm default",
        "min": 0.005,
        "max": 1.0,
    },
    "min_goal_travel_m": {
        "type": "float",
        "default": 0.5,
        "group": "Goal Handling",
        "description": "Minimum path travel before goal check activates (m). 0 disables",
        "min": 0.0,
        "max": 10.0,
    },
    "approach_velocity_scaling_dist": {
        "type": "float",
        "default": 0.6,
        "group": "Goal Handling",
        "description": "Distance from goal where approach speed scaling begins (m)",
        "min": 0.0,
        "max": 5.0,
    },
    "min_approach_linear_velocity": {
        "type": "float",
        "default": 0.1,
        "group": "Goal Handling",
        "description": "Minimum speed in the approach zone (m/s)",
        "min": 0.0,
        "max": 1.0,
    },
    "p4_zero_vel_threshold": {
        "type": "float",
        "default": 0.02,
        "group": "Goal Handling",
        "description": "Speed floor — commands below this are snapped to exactly 0 (m/s). Must be < min_approach_linear_velocity",
        "min": 0.0,
        "max": 0.5,
    },
    # ── Safety ─────────────────────────────────────────────────────────────────
    "pose_max_age_s": {
        "type": "float",
        "default": 0.5,
        "group": "Safety",
        "description": "Pose staleness threshold (s). 200 ms default",
        "min": 0.05,
        "max": 5.0,
    },
    "path_frame_id": {
        "type": "string",
        "default": "local_ned",
        "group": "Safety",
        "description": "Expected frame_id for incoming path messages",
    },
    "ekf_jump_threshold_m": {
        "type": "float",
        "default": 0.05,
        "group": "Safety",
        "description": "Position jump detection threshold (m). Auto-derived from mission_speed at runtime; this param is a floor",
        "min": 0.01,
        "max": 1.0,
    },
    "require_rtk_fix": {
        "type": "bool",
        "default": True,
        "group": "Safety",
        "description": "Gate: refuse non-zero velocity unless GPS fix_type = 6 (RTK_FIXED). Set false for SITL",
    },
    # ── Predictive Curvature (P1.1) ───────────────────────────────────────────
    "preview_curvature_n": {
        "type": "int",
        "default": 4,
        "group": "Predictive Curvature",
        "description": "Number of preview points for worst-κ path lookahead. 1 disables predictive regulation (matches baseline RPP)",
        "min": 1,
        "max": 20,
    },
    # ── Adaptive Lookahead (P1.2) ──────────────────────────────────────────────
    "xtrack_lookahead_gain": {
        "type": "float",
        "default": 0.05,
        "group": "Adaptive Lookahead",
        "description": "Cross-track error gain for adaptive L_d. L_d += k_e · |xtrack|. 0 disables the xtrack term",
        "min": 0.0,
        "max": 5.0,
    },
    # ── Path Conditioning (P1.3) ──────────────────────────────────────────────
    "path_resample_spacing_m": {
        "type": "float",
        "default": 0.08,
        "group": "Path Conditioning",
        "description": "Linear resample spacing (m). >0 densifies sparse polylines for predictive κ. 0 disables",
        "min": 0.0,
        "max": 1.0,
    },
    "corner_smooth_radius_m": {
        "type": "float",
        "default": 0.5,
        "group": "Path Conditioning",
        "description": "Corner smoothing arc radius (m). Replaces vertices with inscribed arcs. 0 disables",
        "min": 0.0,
        "max": 5.0,
    },
    "corner_smooth_arc_pts": {
        "type": "int",
        "default": 6,
        "group": "Path Conditioning",
        "description": "Number of points per inscribed arc (only used when corner_smooth_radius_m > 0)",
        "min": 3,
        "max": 50,
    },
    # ── Tracking Profile ─────────────────────────────────────────────────────
    "tracking_profile": {
        "type": "string",
        "default": "auto",
        "group": "Tracking Profile",
        "description": "Tracking profile: auto, segment, smooth. auto splits the mission into per-entity runs at spray boundaries and picks segment (lines/polygons) or smooth (arcs/circles) per run; segment/smooth force one profile for the whole path. 'sharp' is an alias for segment. Applies on next path load",
    },
    "segment_corner_threshold_deg": {
        "type": "float",
        "default": 45.0,
        "group": "Tracking Profile",
        "description": "Auto-profile threshold: heading changes at or above this angle select segment mode",
        "min": 1.0,
        "max": 180.0,
    },
    "segment_slowdown_dist": {
        "type": "float",
        "default": 0.50,
        "group": "Tracking Profile",
        "description": "Segment mode slowdown distance before a non-final corner (m)",
        "min": 0.0,
        "max": 5.0,
    },
    "segment_min_corner_speed": {
        "type": "float",
        "default": 0.08,
        "group": "Tracking Profile",
        "description": "Minimum forward speed while slowing into a segment-mode corner (m/s)",
        "min": 0.0,
        "max": 1.0,
    },
    "segment_corner_acceptance_radius": {
        "type": "float",
        "default": 0.05,
        "group": "Tracking Profile",
        "description": "Distance from a segment end where corner alignment begins (m)",
        "min": 0.005,
        "max": 1.0,
    },
    "segment_heading_tolerance_deg": {
        "type": "float",
        "default": 2.0,
        "group": "Tracking Profile",
        "description": "Heading error tolerance before advancing from corner alignment to the next segment (deg)",
        "min": 0.1,
        "max": 45.0,
    },
    "segment_timeout_heading_tolerance_deg": {
        "type": "float",
        "default": 5.0,
        "group": "Tracking Profile",
        "description": "Relaxed heading tolerance after the pivot timeout; yaw rate must still settle before advancing (deg)",
        "min": 0.1,
        "max": 45.0,
    },
    "segment_yaw_rate_gain": {
        "type": "float",
        "default": 1.5,
        "group": "Tracking Profile",
        "description": "Segment mode yaw-rate gain applied to heading error (rad/s per rad)",
        "min": 0.0,
        "max": 10.0,
    },
    # ── Latency Closure (P2.4) ────────────────────────────────────────────────
    "use_imu_extrapolation": {
        "type": "bool",
        "default": False,
        "group": "Latency Closure",
        "description": "Enable velocity-based pose extrapolation to close MAVROS pose latency",
    },
    "imu_max_extrap_age_s": {
        "type": "float",
        "default": 0.10,
        "group": "Latency Closure",
        "description": "Max extrapolation age beyond pose_max_age_s (s). 0.10 + 0.20 = 300 ms total budget",
        "min": 0.0,
        "max": 1.0,
    },
    # ── Feedforward Yaw Rate (P3.1) ────────────────────────────────────────────
    "use_feedforward_yaw_rate": {
        "type": "bool",
        "default": True,
        "group": "Feedforward Yaw Rate",
        "description": "Enable feedforward yaw rate (body-rate mode). Publishes ω = κ·v + k_ψ·θ_e. Bypasses PX4 spot-turn FSM",
    },
    "yaw_rate_feedback_gain": {
        "type": "float",
        "default": 0.0,
        "group": "Feedforward Yaw Rate",
        "description": "Yaw rate feedback gain on heading error. 0 = pure feedforward. Tune up once sign is confirmed",
        "min": 0.0,
        "max": 10.0,
    },
    "max_yaw_rate_body": {
        "type": "float",
        "default": 0.45,
        "group": "Feedforward Yaw Rate",
        "description": "Max body yaw rate clamp (rad/s). ≈57°/s at 1.0. 0 disables clamping",
        "min": 0.0,
        "max": 5.0,
    },
    # ── Mission Control (P4.2) ──────────────────────────────────────────────────
    "max_linear_accel": {
        "type": "float",
        "default": 0.35,
        "group": "Mission Control",
        "description": "Acceleration ramp limit (m/s²). Caps speed increase per cycle. 0 disables",
        "min": 0.0,
        "max": 5.0,
    },
    "mission_speed": {
        "type": "float",
        "default": 0.35,
        "group": "Mission Control",
        "description": "Operator-facing mission speed (m/s). Single knob per job. 1.0 for roads, 0.3-0.5 for fields",
        "min": 0.0,
        "max": 2.0,
    },
    "max_linear_decel": {
        "type": "float",
        "default": 0.5,
        "group": "Mission Control",
        "description": "Deceleration limit for braking-distance derivation (m/s²). Separate from max_linear_accel",
        "min": 0.0,
        "max": 5.0,
    },
}


# ── Helpers ────────────────────────────────────────────────────────────────────


def _roster() -> list[str]:
    """Return all parameter names in schema order."""
    return list(RPP_PARAM_SCHEMA.keys())


def _merge_schema_with_values(
    values: dict[str, Any] | None,
) -> list[RppParamInfo]:
    """Merge schema metadata with optional current values from ROS."""
    result: list[RppParamInfo] = []
    for name in _roster():
        entry = RPP_PARAM_SCHEMA[name]
        result.append(
            RppParamInfo(
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


# ── Endpoints ──────────────────────────────────────────────────────────────────


@router.get("")
async def list_rpp_params() -> RppParamListResponse:
    """List all RPP controller params with schema metadata and live values.

    Fetches current values from the running RPP controller. If the controller
    is offline, returns the schema with current=None for all params.
    """
    from main import ros_node

    params = _merge_schema_with_values(None)
    if ros_node is not None:
        try:
            ok, values, _ = await ros_node.get_rpp_params_bulk_async(_roster())
            if ok and values:
                params = _merge_schema_with_values(values)
        except Exception:
            log.warning("RPP param bulk-get failed — serving schema-only", exc_info=True)
            _record("warning", "RPP param bulk-get failed, serving schema without live values")

    return RppParamListResponse(parameters=params, count=len(params))


@router.get("/schema")
async def get_rpp_param_schema() -> RppParamListResponse:
    """Return param metadata only — no ROS calls.

    Works even when the RPP controller is offline. Useful for the frontend
    to render input controls before the connection is established.
    """
    params = _merge_schema_with_values(None)
    return RppParamListResponse(parameters=params, count=len(params))


@router.get("/{name}")
async def get_rpp_param(name: str) -> RppParamGetResponse:
    """Get a single RPP controller parameter's current value."""
    if name not in RPP_PARAM_SCHEMA:
        known = ", ".join(_roster())
        raise HTTPException(422, f"Unknown param '{name}'. Known: {known}")

    from main import ros_node

    if ros_node is None:
        raise HTTPException(503, "ROS node not ready")

    ok, value, why = await ros_node.get_rpp_param_async(name)
    if not ok:
        raise HTTPException(503, why or "RPP controller not reachable")
    return RppParamGetResponse(name=name, value=value)


@router.put("/{name}")
async def set_rpp_param(name: str, req: RppParamSetRequest) -> RppParamSetResponse:
    """Set a single RPP controller parameter at runtime.

    The change takes effect immediately — no node restart needed.
    Use this for live tuning (e.g. `mission_speed`, `a_lat_max`).
    """
    if name not in RPP_PARAM_SCHEMA:
        known = ", ".join(_roster())
        raise HTTPException(422, f"Unknown param '{name}'. Known: {known}")

    from main import ros_node

    if ros_node is None:
        raise HTTPException(503, "ROS node not ready")

    ok, why = await ros_node.set_rpp_param_async(name, req.value)
    if not ok:
        _record("error", f"RPP param set {name}={req.value} failed: {why}")
        raise HTTPException(400, why or "RPP param set failed")
    _record("info", f"RPP param set {name}={req.value}")
    return RppParamSetResponse(name=name, value=req.value, ok=True)


@router.put("")
async def set_rpp_params_bulk(
    req: RppParamSetBulkRequest,
) -> RppParamSetBulkResponse:
    """Set multiple RPP controller parameters atomically.

    All params are sent in a single SetParameters service call. If any param
    name is unknown, the entire batch is rejected with a 422 error listing
    the invalid names.
    """
    unknown = [k for k in req.parameters if k not in RPP_PARAM_SCHEMA]
    if unknown:
        known = ", ".join(_roster())
        raise HTTPException(
            422,
            f"Unknown param(s): {', '.join(unknown)}. Known: {known}",
        )

    from main import ros_node

    if ros_node is None:
        raise HTTPException(503, "ROS node not ready")

    ok, flags, why = await ros_node.set_rpp_params_bulk_async(req.parameters)
    if not ok:
        _record("error", f"RPP bulk set failed: {why}")
        raise HTTPException(400, why or "RPP bulk param set failed")

    result = dict(zip(req.parameters.keys(), flags))
    n_ok = sum(1 for v in result.values() if v)
    _record("info", f"RPP bulk set: {n_ok}/{len(result)} params OK")
    return RppParamSetBulkResponse(parameters=result, ok=all(flags))
