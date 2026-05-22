"""Pydantic request / response models."""
from __future__ import annotations

from enum import Enum
from typing import Literal, Optional, Union

from pydantic import BaseModel


class VehicleMode(str, Enum):
    MANUAL   = "MANUAL"
    OFFBOARD = "OFFBOARD"


class MissionState(str, Enum):
    IDLE               = "idle"
    LOADING            = "loading"
    ARMING             = "arming"
    SWITCHING_OFFBOARD = "switching_offboard"
    RUNNING            = "running"
    STOPPING           = "stopping"
    DISARMING          = "disarming"
    COMPLETED          = "completed"
    ABORTED            = "aborted"
    ERROR              = "error"


# ── Request bodies ────────────────────────────────────────────────────────────

class ArmRequest(BaseModel):
    arm: bool


class ModeRequest(BaseModel):
    mode: VehicleMode


class PathPublishRequest(BaseModel):
    name:     Optional[str] = None
    file:     Optional[str] = None
    frame_id: str = "local_ned"


class MissionStartRequest(BaseModel):
    path_name:    Optional[str] = None
    mission_file: Optional[str] = None


class MissionLoadRequest(BaseModel):
    path_name:    Optional[str] = None
    mission_file: Optional[str] = None


class ParamSetRequest(BaseModel):
    # PX4 has int (SYS_AUTOSTART), float (RO_YAW_RATE_P), and bool params.
    value: Union[bool, int, float, str]


# ── Response / payload models ─────────────────────────────────────────────────

class TelemetryData(BaseModel):
    # Position (NED metres)
    pos_n:           Optional[float] = None
    pos_e:           Optional[float] = None
    heading_ned_deg: Optional[float] = None
    # RPP diagnostics
    xtrack_m:        Optional[float] = None
    heading_err_deg: Optional[float] = None
    lookahead_m:     Optional[float] = None
    speed_m_s:       Optional[float] = None
    kappa:           Optional[float] = None
    dist_to_goal_m:  Optional[float] = None
    pose_age_ms:     Optional[float] = None
    rpp_state:       Optional[Literal[-1, 0, 1, 2, 3, 4, 5]] = None
    rpp_state_name:  Optional[str]   = None
    # FCU
    armed:     Optional[bool] = None
    mode:      Optional[str]  = None
    connected: Optional[bool] = None
    # Battery
    battery_v:   Optional[float] = None
    battery_pct: Optional[float] = None
    # GPS
    gps_fix: Optional[int]   = None
    gps_sat: Optional[int]   = None
    lat:     Optional[float] = None
    lon:     Optional[float] = None
    alt:     Optional[float] = None


class PathInfo(BaseModel):
    name:        str
    description: str
    num_points:  int
    source:      str  # "builtin" | "file"


class MissionStatus(BaseModel):
    state:          MissionState
    rpp_state:      Optional[int]   = None
    rpp_state_name: Optional[str]   = None
    dist_to_goal:   Optional[float] = None
    speed:          Optional[float] = None
    xtrack:         Optional[float] = None


class ActivityEntry(BaseModel):
    timestamp: str
    level:     str
    message:   str


class EstopResponse(BaseModel):
    success: bool
    message: str


class PingResponse(BaseModel):
    status:    str
    timestamp: float


class ArmResponse(BaseModel):
    success: bool
    message: str


class ModeResponse(BaseModel):
    success: bool
    message: str
