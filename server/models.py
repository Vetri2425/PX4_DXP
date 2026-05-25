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


# ── Path planning request / response models ────────────────────────────────────

class DXFEntityInfo(BaseModel):
    """Parsed DXF entity summary for API responses."""
    entity_type: str        # LINE, ARC, CIRCLE, LWPOLYLINE, POINT, etc.
    layer: str              # DXF layer name
    color: int = 7          # AutoCAD color index
    entity_id: str = ""     # ezdxf handle
    is_mark: bool = True    # True = spray ON, False = TRANSIT
    length_m: float = 0.0   # Approximate arc length in metres


class DXFParseResponse(BaseModel):
    """Response from /api/path/parse-dxf."""
    filename: str
    num_entities: int
    entities: list[DXFEntityInfo]
    unit_scale: float       # metres per DXF unit
    layer_names: list[str]  # unique layer names found


class RefPoint(BaseModel):
    """A reference point mapping DXF coordinates to real-world lat/lon."""
    dxf_x: float    # DXF x coordinate
    dxf_y: float    # DXF y coordinate
    lat: float      # WGS84 latitude
    lon: float      # WGS84 longitude


class PathPlanRequest(BaseModel):
    """Request for /api/path/plan."""
    source: str                                       # filename or "builtin:square_2x2"
    selected_entities: Optional[list[str]] = None     # entity IDs to include (None = all)
    overrides: Optional[dict[str, dict]] = None       # {entity_id: {scale, offsetX, offsetY, traverse}}
    order: Optional[list[str]] = None                 # entity IDs in execution order
    layer_mapping: Optional[dict[str, str]] = None    # {layer_pattern: "mark" | "transit" | "ignore"}
    origin: Optional[list[float]] = None              # [north, east] NED offset
    start_position: Optional[list[float]] = None      # [north, east] rover position for TSP
    ref_points: Optional[list[RefPoint]] = None       # 2 reference points for DXF→NED affine
    line_spacing: float = 0.05                         # MARK waypoint spacing (m)
    transit_spacing: float = 0.15                     # TRANSIT waypoint spacing (m)
    marking_speed: float = 0.35                       # MARK speed (m/s)
    transit_speed: float = 0.50                       # TRANSIT speed (m/s)
    optimize: bool = True                              # Reorder segments for minimal dead-heading
    compensate_spray: bool = True                      # Apply spray latency compensation
    include_waypoints: bool = True                     # If False, return summary only (no waypoint arrays)


class PathPlanResponse(BaseModel):
    """Response from /api/path/plan."""
    source: str
    num_waypoints: int
    num_segments: int
    mark_length_m: float
    transit_length_m: float
    total_length_m: float
    segments: list[dict]                               # [{type, points, speed, source}]
    merged_waypoints: list[list[float]]                # [[north, east], ...]
    spray_flags: list[bool]                             # True = MARK
