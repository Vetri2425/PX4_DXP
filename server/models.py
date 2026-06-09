"""Pydantic request / response models."""

from __future__ import annotations

from enum import Enum
from typing import Literal, Optional, Union

from pydantic import BaseModel, Field


class VehicleMode(str, Enum):
    MANUAL = "MANUAL"
    OFFBOARD = "OFFBOARD"


class MissionState(str, Enum):
    IDLE = "idle"
    LOADING = "loading"
    ARMING = "arming"
    SWITCHING_OFFBOARD = "switching_offboard"
    RUNNING = "running"
    STOPPING = "stopping"
    DISARMING = "disarming"
    COMPLETED = "completed"
    ABORTED = "aborted"
    ERROR = "error"


# ── Request bodies ────────────────────────────────────────────────────────────


class ArmRequest(BaseModel):
    arm: bool


class ModeRequest(BaseModel):
    mode: VehicleMode


class PathPublishRequest(BaseModel):
    name: Optional[str] = None
    file: Optional[str] = None
    frame_id: str = "local_ned"


class MissionStartRequest(BaseModel):
    path_name: Optional[str] = None
    mission_file: Optional[str] = None
    auto_origin: bool = False


class MissionLoadRequest(BaseModel):
    path_name: Optional[str] = None
    mission_file: Optional[str] = None


class ParamSetRequest(BaseModel):
    # PX4 has int (SYS_AUTOSTART), float (RO_YAW_RATE_P), and bool params.
    value: Union[bool, int, float, str]


# ── Response / payload models ─────────────────────────────────────────────────


class TelemetryData(BaseModel):
    # Position (NED metres)
    pos_n: Optional[float] = None
    pos_e: Optional[float] = None
    heading_ned_deg: Optional[float] = None
    # RPP diagnostics
    xtrack_m: Optional[float] = None
    heading_err_deg: Optional[float] = None
    lookahead_m: Optional[float] = None
    speed_m_s: Optional[float] = None
    kappa: Optional[float] = None
    dist_to_goal_m: Optional[float] = None
    pose_age_ms: Optional[float] = None
    rpp_state: Optional[Literal[-1, 0, 1, 2, 3, 4, 5]] = None
    rpp_state_name: Optional[str] = None
    # FCU
    armed: Optional[bool] = None
    mode: Optional[str] = None
    connected: Optional[bool] = None
    # Battery
    battery_v: Optional[float] = None
    battery_pct: Optional[float] = None
    # GPS
    gps_fix: Optional[int] = None
    gps_fix_name: Optional[str] = None
    gps_sat: Optional[int] = None
    hrms: Optional[float] = None
    vrms: Optional[float] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt: Optional[float] = None


class PathInfo(BaseModel):
    name: str
    description: str
    num_points: int
    source: str  # "builtin" | "file"


class PathPreviewPoint(BaseModel):
    north: float
    east: float
    spray: bool = True


class PathPreviewBounds(BaseModel):
    north_min: float
    north_max: float
    east_min: float
    east_max: float


class PathPreviewResponse(BaseModel):
    name: str
    frame: str = "local_ned"
    num_points: int
    bounds: Optional[PathPreviewBounds] = None
    waypoints: list[PathPreviewPoint]


class MissionStatus(BaseModel):
    state: MissionState
    rpp_state: Optional[int] = None
    rpp_state_name: Optional[str] = None
    dist_to_goal: Optional[float] = None
    speed: Optional[float] = None
    xtrack: Optional[float] = None
    pose_age_ms: Optional[float] = None
    fcu_connected: Optional[bool] = None
    last_path_loaded: Optional[str] = None


class ActivityEntry(BaseModel):
    timestamp: str
    level: str
    message: str


class RppParamSetRequest(BaseModel):
    """Set a single RPP controller parameter."""

    value: Union[bool, int, float, str]


class RppParamSetBulkRequest(BaseModel):
    """Set multiple RPP controller parameters atomically."""

    parameters: dict[str, Union[bool, int, float, str]]


class RppParamInfo(BaseModel):
    """RPP parameter schema entry with current value."""

    name: str
    type: str  # "float" | "int" | "bool" | "string"
    default: Union[float, int, bool, str, None] = None
    current: Union[float, int, bool, str, None] = None
    group: str  # category for UI grouping
    description: str  # human-readable purpose
    min: Union[float, int, None] = None
    max: Union[float, int, None] = None


class RppParamListResponse(BaseModel):
    """Response for listing all RPP params with current values."""

    parameters: list[RppParamInfo]
    count: int


class RppParamGetResponse(BaseModel):
    """Response for a single RPP param value."""

    name: str
    value: Union[bool, int, float, str, None]


class RppParamSetResponse(BaseModel):
    """Response after setting an RPP param."""

    name: str
    value: Union[bool, int, float, str]
    ok: bool = True


class RppParamSetBulkResponse(BaseModel):
    """Response after bulk-setting RPP params."""

    parameters: dict[str, bool]  # {name: success}
    ok: bool


class EstopResponse(BaseModel):
    success: bool
    message: str


class PingResponse(BaseModel):
    status: str
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

    entity_type: str  # LINE, ARC, CIRCLE, LWPOLYLINE, POINT, etc.
    layer: str  # DXF layer name
    color: int = 7  # AutoCAD color index
    entity_id: str = ""  # ezdxf handle
    is_mark: bool = True  # True = spray ON, False = TRANSIT
    length_m: float = 0.0  # Approximate arc length in metres


class DXFParseResponse(BaseModel):
    """Response from /api/path/parse-dxf."""

    filename: str
    num_entities: int
    entities: list[DXFEntityInfo]
    unit_scale: float  # metres per DXF unit
    layer_names: list[str]  # unique layer names found


class RefPoint(BaseModel):
    """A reference point mapping DXF coordinates to real-world lat/lon."""

    dxf_x: float  # DXF x coordinate
    dxf_y: float  # DXF y coordinate
    lat: float  # WGS84 latitude
    lon: float  # WGS84 longitude


class PathPlanRequest(BaseModel):
    """Request for /api/path/plan."""

    source: str  # filename or "builtin:square_2x2"
    selected_entities: Optional[list[str]] = None  # entity IDs to include (None = all)
    overrides: Optional[dict[str, dict]] = (
        None  # {entity_id: {scale, offsetX, offsetY, traverse}}
    )
    order: Optional[list[str]] = None  # entity IDs in execution order
    layer_mapping: Optional[dict[str, str]] = (
        None  # {layer_pattern: "mark" | "transit" | "ignore"}
    )
    origin: Optional[list[float]] = None  # [north, east] NED offset
    start_position: Optional[list[float]] = None  # [north, east] rover position for TSP
    ref_points: Optional[list[RefPoint]] = None  # reference points for DXF→NED affine
    origin_gps: Optional[list[float]] = None  # [latitude, longitude] WGS84 reference
    rotation_deg: float = 0.0  # DXF rotation relative to true north
    close_loop: bool = False  # True to close open loop paths
    line_spacing: float = 0.05  # MARK waypoint spacing (m)
    transit_spacing: float = 0.15  # TRANSIT waypoint spacing (m)
    marking_speed: float = 0.35  # MARK speed (m/s)
    transit_speed: float = 0.50  # TRANSIT speed (m/s)
    optimize: bool = True  # Reorder segments for minimal dead-heading
    compensate_spray: bool = True  # Apply spray latency compensation
    enable_path_extensions: bool = False  # Add PRE/AFT drive extensions around MARK geometry
    pre_extension_m: float = Field(0.5, ge=0.0)  # Drive-in distance before MARK start (m)
    aft_extension_m: float = Field(0.5, ge=0.0)  # Drive-out distance after MARK end (m)
    corner_smooth_radius_m: float = Field(0.0, ge=0.0)  # Planner-side corner radius; 0 disables
    corner_smooth_arc_pts: int = Field(6, ge=2)  # Points per smoothed corner arc
    use_two_opt: bool = True  # Improve greedy segment order with 2-opt
    max_two_opt_segments: int = Field(80, ge=0, le=1000)  # Skip 2-opt above this MARK count
    max_waypoints: int = Field(10000, ge=100, le=500000)  # Hard publication guard
    max_segments: int = Field(2000, ge=1, le=100000)  # Hard segment-count guard
    include_waypoints: bool = True  # If False, return summary only (no waypoint arrays)


class PathPlanResponse(BaseModel):
    """Response from /api/path/plan."""

    source: str
    num_waypoints: int
    num_segments: int
    mark_length_m: float
    transit_length_m: float
    total_length_m: float
    segments: list[dict]  # [{type, points, speed, source}]
    merged_waypoints: list[list[float]]  # [[north, east], ...]
    spray_flags: list[bool]  # True = MARK
    alignment_metadata: Optional[dict] = None  # alignment stats/residuals
    planning_metadata: Optional[dict] = None  # counts/timings/bbox/unit metadata
    warnings: Optional[list[str]] = None  # geometry/safety warnings
