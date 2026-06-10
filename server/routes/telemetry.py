"""GET /api/telemetry/latest — snapshot of all telemetry fields.

Read-only; not auth-protected so dashboards / health checks can poll cheaply.
"""
from __future__ import annotations

from fastapi import APIRouter

from config import GPS_FIX_NAMES, RPP_STATE_NAMES
from models import MissionState, TelemetryData

router = APIRouter(prefix="/telemetry", tags=["telemetry"])


@router.get("/latest", response_model=TelemetryData)
async def telemetry_latest():
    from main import offboard_ctrl, ros_node
    if ros_node is None:
        return TelemetryData()
    s = ros_node.get_state()
    code = s.get("rpp_state", 0)
    spraying = bool(s.get("spraying", False))
    mission_running = (
        offboard_ctrl is not None
        and offboard_ctrl.state == MissionState.RUNNING
        and bool(s.get("armed", False))
    )
    if not mission_running:
        marking_state = "off"
    elif spraying:
        marking_state = "marking"
    else:
        marking_state = "transit"
    return TelemetryData(
        pos_n           = s.get("pos_n"),
        pos_e           = s.get("pos_e"),
        heading_ned_deg = s.get("heading_ned_deg"),
        xtrack_m        = s.get("xtrack_m"),
        heading_err_deg = s.get("heading_err_deg"),
        lookahead_m     = s.get("lookahead_m"),
        speed_m_s       = s.get("speed_m_s"),
        kappa           = s.get("kappa"),
        dist_to_goal_m  = s.get("dist_to_goal_m"),
        pose_age_ms     = s.get("pose_age_ms"),
        rpp_state       = code,
        rpp_state_name  = RPP_STATE_NAMES.get(code, "UNKNOWN"),
        spraying        = spraying,
        marking_state   = marking_state,
        armed           = s.get("armed"),
        mode            = s.get("mode"),
        connected       = s.get("connected"),
        battery_v       = s.get("battery_v"),
        battery_pct     = s.get("battery_pct"),
        gps_fix         = s.get("gps_fix"),
        gps_fix_name    = GPS_FIX_NAMES.get(s.get("gps_fix", 0), "UNKNOWN"),
        gps_sat         = s.get("gps_sat"),
        hrms            = s.get("hrms"),
        vrms            = s.get("vrms"),
        lat             = s.get("lat"),
        lon             = s.get("lon"),
        alt             = s.get("alt"),
    )
