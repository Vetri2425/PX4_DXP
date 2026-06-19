"""Pydantic request / response models."""

from __future__ import annotations

from enum import Enum
from typing import Any, Literal, Optional, Union

from pydantic import BaseModel, Field, field_validator


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
    mission_id: Optional[str] = None
    auto_origin: bool = False


class MissionLoadRequest(BaseModel):
    path_name: Optional[str] = None
    mission_file: Optional[str] = None


class SprayTestRequest(BaseModel):
    on: bool
    # Seconds to hold manual spray ON before server-side auto-off.
    # Clamped to MAX_SPRAY_TEST_DURATION_S; the node's
    # manual_override_timeout_s is the hard backstop.
    duration_s: Optional[float] = None


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
    spraying: Optional[bool] = None
    marking_state: Optional[Literal["marking", "transit", "off"]] = None
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
    loaded_mission_id: Optional[str] = None
    running_mission_id: Optional[str] = None


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


class EntityPreviewPoint(BaseModel):
    """Lightweight local-NED point used to render/select a DXF entity."""

    north: float
    east: float


class DXFEntityPreview(BaseModel):
    """Entity-level DXF preview geometry for canvas rendering and hit-testing."""

    entity_id: str
    entity_type: str
    layer: str
    color: int = 7
    default_is_mark: bool = True
    is_mark: bool = True
    order_index: int = 0
    length_m: float = 0.0
    geometry: dict[str, Any] = Field(default_factory=dict)
    preview_points: list[EntityPreviewPoint]
    extension_preview: Optional["EntityExtensionPreview"] = None


class DXFEntitiesResponse(BaseModel):
    """Response from /api/path/{name}/entities."""

    name: str
    frame: str = "local_ned"
    num_entities: int
    bounds: Optional[PathPreviewBounds] = None
    extension_config: Optional["PathExtensionConfig"] = None
    transit_preview: list["EntityTransitPreview"] = Field(default_factory=list)
    entities: list[DXFEntityPreview]


class EntityTransitPreview(BaseModel):
    """Lightweight no-spray connector between consecutive MARK entities."""

    from_entity_id: str
    to_entity_id: str
    length_m: float = 0.0
    points: list[EntityPreviewPoint]


class EntityExtensionPreview(BaseModel):
    """Lightweight PRE/AFT extension geometry for an entity preview."""

    enabled: bool = False
    pre_length_m: float = 0.0
    aft_length_m: float = 0.0
    pre_points: list[EntityPreviewPoint] = Field(default_factory=list)
    aft_points: list[EntityPreviewPoint] = Field(default_factory=list)


class EntityMarkOverride(BaseModel):
    """User-editable spray classification for a single DXF entity."""

    entity_id: str
    is_mark: bool


class DXFEntityOverridesRequest(BaseModel):
    """Persist per-entity spray overrides for a DXF file."""

    overrides: list[EntityMarkOverride]


class DXFEntityOverridesResponse(BaseModel):
    """Response from POST /api/path/{name}/entities."""

    name: str
    saved: bool = True
    num_overrides: int


class EntityOrderUpdateRequest(BaseModel):
    """Persist entity execution order for a DXF file."""

    entity_order: list[str]


class EntityOrderUpdateResponse(BaseModel):
    """Response from POST /api/path/{name}/entities/order."""

    name: str
    num_entities: int
    entity_order: list[str]


class PathExtensionConfig(BaseModel):
    """Per-file path extension settings."""

    enabled: bool = False
    pre_extension_m: float = Field(0.5, ge=0.0)
    aft_extension_m: float = Field(0.5, ge=0.0)
    # When True, every CAD line is an independent PRE→MARK→AFT pass (each side of
    # a square/rectangle/polygon gets its own run-up/run-out, closed loops are no
    # longer suppressed). When False, the connectivity-aware policy extends only a
    # chain's true open ends.
    #
    # None = "leave unchanged" on save: a client that omits per_line (e.g. an older
    # frontend that predates this field) preserves the saved value instead of
    # silently resetting it to False. Reads always resolve to a concrete bool.
    per_line: Optional[bool] = None


class PathExtensionConfigResponse(PathExtensionConfig):
    """Response from GET/POST /api/path/{name}/extensions."""

    name: str
    saved: bool = True


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
    # Planner preserves CAD geometry; runtime spray_controller owns latency
    # anticipation (use_distance_aware_spray). Strict: True is rejected by the
    # validator below — geometric pre-compensation is offline-only (PathEngine).
    compensate_spray: bool = False
    # Deprecated trio: ignored by /api/path/plan (a warning is logged and
    # returned in the response `warnings` when set explicitly). Configure
    # extensions via GET/POST /api/path/{name}/extensions instead.
    enable_path_extensions: bool = False
    pre_extension_m: float = Field(0.5, ge=0.0)
    aft_extension_m: float = Field(0.5, ge=0.0)
    corner_smooth_radius_m: float = Field(0.0, ge=0.0)  # Planner-side corner radius; 0 disables
    corner_smooth_arc_pts: int = Field(6, ge=2)  # Points per smoothed corner arc
    use_two_opt: bool = True  # Improve greedy segment order with 2-opt
    max_two_opt_segments: int = Field(80, ge=0, le=1000)  # Skip 2-opt above this MARK count
    max_waypoints: int = Field(10000, ge=100, le=500000)  # Hard publication guard
    max_segments: int = Field(2000, ge=1, le=100000)  # Hard segment-count guard
    include_waypoints: bool = True  # If False, return summary only (no waypoint arrays)

    @field_validator("compensate_spray")
    @classmethod
    def _reject_geometric_compensation(cls, v: bool) -> bool:
        # STRICT controller-only ownership: production planning routes preserve
        # exact CAD MARK geometry; the runtime spray_controller owns latency
        # anticipation (use_distance_aware_spray). Reject geometric pre-shift at
        # the API edge so a client cannot resurrect 2.0315 m MARK segments.
        # Offline/diagnostic geometric compensation remains available by calling
        # PathEngine(compensate_spray=True) directly, off the API.
        if v:
            raise ValueError(
                "compensate_spray=true is not accepted on production planning "
                "routes: the planner preserves exact CAD geometry and the "
                "spray_controller owns latency anticipation. For offline "
                "geometric pre-compensation, call PathEngine directly."
            )
        return v


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
    mission_summary: Optional["MissionSummary"] = None  # staged-mission handoff summary


class AnchorBlock(BaseModel):
    """Definitive global anchor for the aligned mission (Gap E).

    Written as the first object of a staged mission so the controller can
    re-project NED waypoints back to WGS84 if it needs to recompute a
    deviation mid-run.
    """

    frame: str = "local_ned"
    lat: float
    lon: float
    rotation_deg: float = 0.0
    scale: float = 1.0


class MissionSummary(BaseModel):
    """High-level summary returned for operator confirmation (Gap C)."""

    mission_id: str
    num_waypoints: int
    total_length_m: float
    estimated_paint_l: float
    estimated_runtime_s: float
    rmse_m: float


# ── Staged workflow: stage-specific endpoints ─────────────────────────────────
# These split the monolithic /api/path/plan into composable stages. /plan stays
# unchanged; each stage below is additive.

class AlignRequest(BaseModel):
    """Stage 6/7 — alignment only. No optimize / extend / stage / load."""

    ref_points: Optional[list[RefPoint]] = None       # DXF→NED affine fit
    origin_gps: Optional[list[float]] = None          # [lat, lon]
    rotation_deg: float = 0.0
    origin: Optional[list[float]] = None              # [north, east] NED offset
    auto_origin: bool = False
    sample_points: int = Field(20, ge=0, le=2000)     # transformed coords to return


class RefPointResidual(BaseModel):
    """Per-reference-point alignment residual (least-squares mode only)."""

    dxf_x: float
    dxf_y: float
    lat: float
    lon: float
    residual_m: float


class AlignResponse(BaseModel):
    """Response for POST /api/path/{name}/align."""

    source: str
    method: Optional[str] = None          # least_squares | single_point_heading | gps_origin
    rmse_m: float = 0.0
    scale: float = 1.0
    rotation_deg: float = 0.0
    offset_n: float = 0.0
    offset_e: float = 0.0
    origin_gps: Optional[list[float]] = None
    num_waypoints: int = 0
    sample_coords: list[list[float]] = Field(default_factory=list)   # [[n, e], ...]
    residuals: list[RefPointResidual] = Field(default_factory=list)
    warnings: Optional[list[str]] = None


class SegmentInfo(BaseModel):
    """One verification segment (stage 8)."""

    index: int
    sequence: int
    type: str                       # MARK | TRANSIT
    segment_role: Optional[str] = None   # mark | pre_transit | aft_transit | transit
    source_entity: str = ""
    is_extension: bool = False
    spray_on: bool = False
    speed: float = 0.0
    length_m: float = 0.0
    points: list[list[float]] = Field(default_factory=list)   # [[n, e], ...]


class PathSegmentsResponse(BaseModel):
    """Response for GET /api/path/{name}/segments."""

    name: str
    num_segments: int
    num_waypoints: int
    mark_length_m: float
    transit_length_m: float
    total_length_m: float
    extension_config: Optional["PathExtensionConfig"] = None
    segments: list[SegmentInfo] = Field(default_factory=list)
    warnings: Optional[list[str]] = None


class StagedMissionResponse(BaseModel):
    """Response for GET /api/path/staged/{mission_id}. Exact staged content."""

    mission_id: str
    created_at: Optional[float] = None
    anchor: Optional[dict] = None
    num_waypoints: int = 0
    waypoints: list[list[float]] = Field(default_factory=list)
    spray_flags: list[bool] = Field(default_factory=list)
    segment_runs: list[dict] = Field(default_factory=list)  # derived spray on/off runs
    alignment_metadata: Optional[dict] = None
    metadata: Optional[dict] = None


class LoadedPathResponse(BaseModel):
    """Response for GET /api/mission/loaded-path (stage 10)."""

    loaded: bool = False
    name: Optional[str] = None
    mission_id: Optional[str] = None
    running_mission_id: Optional[str] = None
    source_name: Optional[str] = None
    placement_mode: str = "LOCAL_NED"
    origin_gps: Optional[list[float]] = None
    is_staged: bool = False
    protected: bool = False
    state: str = "idle"
    num_waypoints: int = 0
    num_mark: int = 0
    num_transit: int = 0
    has_spray_flags: bool = False
    sample_coords: list[list[float]] = Field(default_factory=list)
    sample_truncated: bool = False


class LoadMissionRequest(BaseModel):
    """Payload for committing a staged mission to the controller."""

    mission_id: str
