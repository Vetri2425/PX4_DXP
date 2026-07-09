# Runtime Entry Path Generation & Controller Tracking

Complete flow from server (FastAPI) → ROS2 nodes (src). All file paths are relative to `~/PX4_DXP/`.

**Runtime entry is not universal.** Only two publishers set the RPP runtime-entry marker:

| Caller | When | How |
|--------|------|-----|
| Continuous `GPS_SURVEYED` start | Rover not coincident with resolved wp0 | `_build_runtime_entry_path()` prepends OFF densified leg; `publish_path(..., runtime_entry=True)` |
| Point-mode legs | Every per-point navigate leg | `PointMissionOrchestrator._publish_fresh_leg()` always publishes with `runtime_entry=True` and spray flags all OFF |

Local / auto_origin continuous missions **never** get an entry prefix (path is offset onto the rover instead). See `test_local_mission_never_gets_runtime_entry_prefix`.

---

## 1. Server: Path Loading & Validation

### `server/path_manager.py` — PathManager

**`load_path()`** (line 590) — entry point for loading a path from any source (builtin, CSV, DXF, waypoints). Routes DXF through the full `PathEngine` pipeline; applies origin offset when needed.

```python
def load_path(
    self,
    name: str,
    origin: tuple[float, float] = (0.0, 0.0),
    start_position: tuple[float, float] | None = None,
    auto_origin: bool = False,
) -> list[tuple[float, float]]:
```

**`plan_path()`** (line 838) — runs the full path_engine pipeline with all sidecar configs (extensions, entity overrides, saved order). Returns dict with merged_waypoints, spray_flags, segments, metadata.

**`preview_path()`** (line 637) — returns NED points for display without mutating mission state. Shares same engine config as `plan_path()` to keep preview and execution in lock-step.

**Key path_engine params in plan_path** (line 1004):
```python
engine = PathEngine(
    mark_spacing=line_spacing,        # 0.05 m default
    transit_spacing=transit_spacing,  # 0.15 m default
    marking_speed=marking_speed,      # 0.35 m/s
    transit_speed=transit_speed,      # 0.50 m/s
    optimize_order=effective_optimize,
    compensate_spray=compensate_spray,
    enable_path_extensions=enable_path_extensions,
    pre_extension_m=pre_extension_m,
    aft_extension_m=aft_extension_m,
    per_line_extensions=per_line_extensions,
    corner_smooth_radius_m=corner_smooth_radius_m,
    corner_smooth_arc_pts=corner_smooth_arc_pts,
    use_two_opt=use_two_opt,
    max_two_opt_segments=max_two_opt_segments,
)
```

---

### `server/mission_loading.py` — Load Guards & Start Orchestration

**`load_path_for_controller()`** (line 83) — async wrapper under `_load_lock`:

1. Spray-startup + protected-mission + lifecycle state guards
2. Temporarily sets `MissionState.LOADING` while loading
3. `path_mgr.load_path()` (thread offload) → `validate_point_count()` → `spray_flags_for_path()`
4. Restores prior controller state, then `offboard_ctrl.load_path(...)`

```python
async def load_path_for_controller(offboard_ctrl, path_mgr, name, ...):
    async with _load_lock:
        # guards: spray startup, protected mission, load_block_reason(prior_state)
        prior_state = offboard_ctrl.state
        offboard_ctrl.state = MissionState.LOADING
        try:
            points = await asyncio.to_thread(path_mgr.load_path, name, ...)
            validate_point_count(points)
            spray_flags = spray_flags_for_path(path_mgr, name, len(points))
        except Exception:
            offboard_ctrl.state = prior_state
            raise
        offboard_ctrl.state = prior_state
        offboard_ctrl.load_path(points, name=name, spray_flags=spray_flags)
        return points
```

`spray_flags_for_path()` uses `preview_path()` waypoints; on preview failure or length mismatch falls back to `SPRAY_DEFAULT_ON` for every point.

**`start_mission_for_controller()`** (line 127) — full load-and-start pipeline:

1. Resolve auto_origin → rover's current EKF position (only when not a protected/surveyed staged mission)
2. Load path if name given (`origin_pre_applied` if auto_origin already applied at load)
3. Begin debug capture (if coordinator present)
4. If `spray_mode == "point"`: `point_mission.prepare(ros_node.get_state())`
5. Call `offboard_ctrl.start_async()`:
   - **Point:** `_start_point_shell_async` — arm + OFFBOARD only; **no** full path publish
   - **Continuous/dash:** publish full resolved path once (entry prepend only if `GPS_SURVEYED`)
6. If point mission and start ok: `point_mission.start()` (publishes per-leg paths with `runtime_entry=True`)

---

### `server/offboard_controller.py` — Runtime Entry Transit

**Module constants** (not in `config.py`):

```python
ENTRY_COINCIDENT_TOLERANCE_M = 1e-6
ENTRY_DENSIFY_SPACING_M = 0.05
```

**`_build_runtime_entry_path()`** (line 101–137) — prepends a spray-OFF acquisition leg from the rover's current NED position to the mission's first waypoint. Called **only** from the `GPS_SURVEYED` branch of `_start_async_locked()`.

```python
def _build_runtime_entry_path(
    points: list[tuple[float, float]],
    spray_flags: list[bool] | None,
    rover_ned: tuple[float, float],
) -> tuple[list[tuple[float, float]], list[bool] | None, dict[str, Any]]:
    original_first = points[0]
    distance_m = math.hypot(
        rover_ned[0] - original_first[0], rover_ned[1] - original_first[1]
    )
    evidence = {
        "entry_transit_added": distance_m > ENTRY_COINCIDENT_TOLERANCE_M,
        "entry_start_ned": list(rover_ned),
        "entry_target_ned": list(original_first),
        "entry_distance_m": distance_m,
        "entry_target_original_spray_on": (
            bool(spray_flags[0]) if spray_flags is not None else None
        ),
    }
    if not evidence["entry_transit_added"]:
        return list(points), list(spray_flags) if spray_flags is not None else None, evidence

    entry_leg = _densify_leg(rover_ned, original_first)  # ENTRY_DENSIFY_SPACING_M
    evidence["entry_leg_point_count"] = len(entry_leg)
    entry_off = [False] * len(entry_leg)
    if spray_flags is None:
        # Fail-closed: keep flags None so publish_path forces all OFF
        return [*entry_leg, *points], None, evidence
    return [*entry_leg, *points], [*entry_off, *spray_flags], evidence
```

`entry_leg` ends on the exact `original_first` coordinate; the following mission list also starts with that point — that **duplicate** marks the entry-run boundary for RPP (`split_leading_entry_transit`).

**`_densify_leg()`** (line 71–88) — straight leg `start→end` (both inclusive), spacing `ENTRY_DENSIFY_SPACING_M = 0.05`. Final point is EXACT `end` coordinate. Legs shorter than one spacing stay a clean 2-point segment.

**`start_async()` → `_start_async_locked()`** (line 502–800) — continuous-mission start state machine:

```
IDLE / COMPLETED / ABORTED / ERROR → ARMING → SWITCHING_OFFBOARD → RUNNING
```

(`LOADING` is used by path load, not as a sticky mid-start state.)

Key sequence for **continuous** missions:

1. FCU connected; reject if already RUNNING / mid-lifecycle
2. If `spray_mode == "point"` → early return to `_start_point_shell_async` (no path publish)
3. Copy loaded source points + spray flags
4. If `GPS_SURVEYED`: first `resolve_surveyed_points()` (typed placement errors before RPP gates)
5. RPP health: `rpp_debug_fresh` and state ∉ `{STALE, RTK_WAIT, JUMP_SKIP}`
6. If not surveyed and `auto_origin`: offset mission points onto current pose (**no** entry prepend)
7. If `GPS_SURVEYED`: re-resolve from a fresh FCU snapshot, then `_build_runtime_entry_path()` (line 680–686)
8. Optional `pre_publish_hook` (debug capture placement evidence)
9. Publish path; `runtime_entry=True` **only if** `entry_evidence["entry_transit_added"]`
10. Arm → sleep `SETPOINT_STREAM_GRACE_S` (0.5 s) → re-check RPP (also reject IDLE) → `set_mode_async("OFFBOARD")` → RUNNING

**Local continuous missions** skip step 7; published geometry is the loaded/auto-origin path with `runtime_entry=False`.

**`_start_point_shell_async()`** (line 802–874) — point missions:

- Arm + OFFBOARD only (no `/path` at shell start)
- Per-leg geometry is published later by the orchestrator with `runtime_entry=True`

**`_publish_path_to_node()`** (line 489–500) — forwards points + spray_flags + mission_id + configuration_revision + path_fingerprint + optional `runtime_entry`:

```python
publish_kwargs = {
    "spray_flags": spray_flags_to_publish,
    "mission_id": self._loaded_mission_id or "",
    "configuration_revision": self._configuration_revision,
    "path_fingerprint": self._path_fingerprint,
    "verify_supplied_fingerprint": False,
}
if entry_evidence["entry_transit_added"]:
    publish_kwargs["runtime_entry"] = True
self._publish_path_to_node(pts_to_publish, **publish_kwargs)
```

---

### `server/ros_node.py` — Bridge + Path Publishing

**`publish_path()`** (line 1253–1314) — serializes NED points into `nav_msgs/Path` with **two** encodings:

| Channel | Field | Meaning |
|---------|-------|---------|
| Spray | `pose.position.z` | `1.0` MARK/ON, `0.0` OFF (RPP: `z > 0.5`) |
| Runtime entry | first pose quaternion | `orientation.x = 1.0`, `orientation.w = 0.0` when `runtime_entry=True` |

```python
ps.pose.position.x = float(n)   # NED North
ps.pose.position.y = float(e)   # NED East
ps.pose.position.z = 1.0 if spray else 0.0
if runtime_entry and index == 0:
    ps.pose.orientation.x = 1.0   # marker for RPP conditioning only
    ps.pose.orientation.w = 0.0
else:
    ps.pose.orientation.w = 1.0
```

If `spray_flags is None` or length mismatch → **all OFF** (fail-closed). Also publishes `/path/identity` (JSON envelope with `mission_id`, `path_fingerprint`, `configuration_revision`, `source="raw_path"`).

**`publish_stop_path()`** (line 1346–1372) — single-point path at current rover pose → RPP treats as DONE → zero velocity. Guard: only publishes if `pose_received=True`.

**`RppStatusMonitor`** (from `rpp_status.py`, line 41–99) — done-settle logic (`DONE_SETTLE_S` default **1.0 s**):

```python
def is_done(self) -> bool:
    if self._done_since is None:
        return False
    if not self.is_fresh():
        return False
    return (time.monotonic() - self._done_since) >= self._done_settle_s
```

State codes from `/rpp/debug` index [7]:

| Code | Name | Meaning |
|------|------|---------|
| -1   | STALE | Pose timeout — emergency zero |
| 0    | IDLE | No path / no pose |
| 1    | TRACKING | Normal RPP tracking |
| 2    | APPROACH | Within approach zone, speed scaling |
| 3    | DONE | Path complete — outputting zero |
| 4    | RTK_WAIT | GPS fix < RTK_FIXED |
| 5    | JUMP_SKIP | EKF jump detected, one-cycle skip |

---

## 2. Server: Control Lifecycle

### `server/control_arbiter.py` — Ownership Arbitration

Single lock preventing mission/joystick conflicts:

```python
class ControlOwner(str, Enum):
    IDLE = "idle"
    MISSION = "mission"
    JOYSTICK_ACQUIRING = "joystick_acquiring"
    JOYSTICK_ACTIVE = "joystick_active"
    JOYSTICK_HELD = "joystick_held"
    RELEASING = "releasing"
```

**`mission_start()`** context manager — raises `ControlArbiterError` if joystick owns control.

### `server/mission_ops.py` — Operation Coordination

Priority-based preemption for mission operations (ESTOP > ABORT > STOP > COMPLETION > RESTART > SKIP > PAUSE/RESUME/CONTINUE):

```python
OPERATION_PRIORITY = {
    MissionOperation.ESTOP: 100,
    MissionOperation.ABORT: 90,
    MissionOperation.STOP: 80,
    MissionOperation.COMPLETION: 70,
    MissionOperation.RESTART: 60,
    MissionOperation.SKIP: 50,
    MissionOperation.PAUSE: 40,
    MissionOperation.RESUME: 40,
    MissionOperation.CONTINUE: 40,
}
```

`begin()` acquires a token with optional timeout; `finish()` releases it. Higher priority operations preempt lower ones.

### `server/point_mission.py` — Point Mission Orchestrator

Per-point state machine for point-mode missions:

```
IDLE → PREPARING_LEG → NAVIGATING → SETTLING → DWELLING → ADVANCING → ...
              ↕              ↕              ↕
        PAUSED_HOLD   PAUSED_OBSTACLE  PAUSED_GPS_SAFETY
```

**Per-leg publish** (`_publish_fresh_leg`, line 2408–2435):

```python
published, diag = self._build_point_leg(state, point, params)
ros_node.publish_path(
    published,
    spray_flags=[False] * len(published),  # transit only; dwell owns spray
    runtime_entry=True,                    # always mark for RPP conditioning
)
```

**`_build_point_leg()`** (line 799–826) builds geometry via `build_point_leg_path()` and predicts conditioning:

```python
def _build_point_leg(self, state, point, params):
    start = (float(state["pos_n"]), float(state["pos_e"]))
    end = (point.north_m, point.east_m)
    mode = PointLegTrajectoryMode.parse(params.leg_trajectory_mode)
    published = build_point_leg_path(
        start, end, mode=mode, spacing_m=params.leg_spacing_m,
    )
    profile, conditioned = predict_rpp_conditioning(
        published,
        runtime_entry=True,
        resample_spacing_m=params.leg_spacing_m,
    )
    return published, { ... diagnostics ... }
```

### `server/rpp_status.py` — RPP Snapshot Decoder

**`RppSnapshot.from_debug_array()`** — decodes only the legacy **[0–7]** fields from `/rpp/debug` (server consumers do not need the full 47-element param snapshot):

```python
@classmethod
def from_debug_array(cls, data: list[float]) -> "RppSnapshot":
    code = int(data[7])
    return cls(
        xtrack_m=data[0], heading_err_deg=math.degrees(data[1]),
        lookahead_m=data[2], speed_m_s=data[3],
        kappa=data[4], dist_to_goal_m=data[5],
        pose_age_ms=data[6], state_code=code,
        state_name=RPP_STATE_NAMES.get(code, "UNKNOWN"),
    )
```

---

## 3. ROS2 Nodes (src): Path Ingestion & Conditioning

### `src/rpp_controller_node.py` — RPP Controller

**Path receipt → `_path_cb()`** (line 860–1030):

1. Detect runtime entry marker (first pose `orientation.x` ≈ 1.0, `orientation.w` ≈ 0.0); spray flags from `position.z > 0.5`
2. Split leading entry transit via `split_leading_entry_transit()`:
   ```python
   entry_run, profile_pts, profile_flags = split_leading_entry_transit(
       raw_pts, raw_flags, marked=runtime_entry_marked,
   )
   ```
3. Auto-profile: split into runs at flag boundaries + short-connector absorb + corner-split + collinear merge:
   ```python
   raw_runs = [
       sub for run in self._split_runs_by_flag(profile_pts, profile_flags)
       for sub in self._split_run_at_corners(
           *self._absorb_short_connectors(*run, threshold, connector_m, min_corner_deg),
           threshold,
       )
   ]
   raw_runs = self._merge_collinear_runs(raw_runs, threshold)
   ```
   If `entry_run is not None`, it is **inserted at index 0** before conditioning.
4. Per-run conditioning:
   - Point-leg densified runs (`runtime_entry_marked` + collinear + entry/OFF) → profile **smooth**, resample only (no corner smooth)
   - Segment runs → `_simplify_path_for_profile()` (collinear collapse)
   - Smooth runs → optional `_smooth_corners()` + `_resample_path()`
5. Drop sliver runs &lt; 5 cm **except** `runtime_entry` runs (always kept)
6. `_apply_run(0)` sets `self._path`, `self._spray_flags`, tracking profile

**Tracking profile classification** (line 1251–1287):

Auto mode: simplify → max heading delta ≥ corner threshold → **segment**; else sustained turning (≥3 bends &gt; 2°, sum &gt; 20°) → **smooth**; otherwise segment.

**Segment tracking state machine** (enum codes; execution order differs):

| State | Code | Action |
|-------|------|--------|
| INACTIVE | 0 | No segment active |
| TRACK_SEGMENT | 1 | Pure pursuit along straight side |
| PRE_CORNER_SLOWDOWN | 2 | Deceleration before corner |
| CORNER_ALIGN | 3 | Pivot (spot turn) to next heading |
| DONE | 4 | Final segment complete |
| CORNER_STOP | 5 | Zero-velocity hold at corner point |

**Execution order** at a hard corner:

```
TRACK_SEGMENT → PRE_CORNER_SLOWDOWN → CORNER_STOP → CORNER_ALIGN → next TRACK_SEGMENT
```

**`_control_segment_profile()`** (line 3045+) — segment-mode control loop follows that order, then advances segment/run.

**Run transitions → `_hold_before_run_advance()`** (line 1748+; pure-zero helper `_entry_pure_stop_hold` at 1948):

- Collinear → advance immediately
- Hard boundary → latch CORNER_STOP (same physical stop as intra-run corners):
  - **Pure-zero stop** at runtime-entry→MARK boundary (`StopReason.RUNTIME_ENTRY_TO_MARK`, latched once inside `segment_entry_pure_stop_dist_m`)
  - True-stop / corner-hold / capture velocity branches for smooth-run terminals
  - Stop certification + alignment certification → `_advance_run(pre_stopped=True)`

**Certificates** — explicit gates before pivot/release (line 255–305):

```python
@dataclass
class StopCertificate:      # position + speed dwell confirmed
@dataclass
class AlignmentCertificate:  # heading + yaw-rate settled
@dataclass
class FinalStopCertificate:  # final endpoint reached
```

**Control loop output** — `/rpp/velocity_ned` (Vector3Stamped) + `/rpp/yaw_rate_body` when feedforward enabled:

```python
vel.x = v_north  # m/s NED North
vel.y = v_east   # m/s NED East
vel.z = 0.0
```

**`/rpp/debug` diagnostic array** — 47 fields (`size=47`, layout lines 113–161 / publish at 5198+):

| Indices | Content |
|---------|---------|
| [0–7] | Legacy: xtrack, heading_err_rad, lookahead, speed, kappa, dist_to_goal, pose_age_ms, state_code |
| [8–9] | B1: l_d_raw, kappa_speed |
| [10] | yaw_rate_cmd_rad_s |
| [11–38] | Tunable param snapshot (max_linear_vel … mission_speed) |
| [39] | spray_active |
| [40–46] | tracking_profile_code + segment params |

Also publishes `/rpp/segment_debug` and `/rpp/stop_debug` for FSM forensics.

---

### `src/rpp_path_conditioning.py` — Entry Transit Splitter

**`split_leading_entry_transit()`** (line 7–48) — extracts the runtime-inserted OFF acquisition leg from before the duplicated waypoint 0:

```python
def split_leading_entry_transit(points, flags, *, marked):
    if not marked or len(points) < 4 or len(points) != len(flags) or flags[0]:
        return None, list(points), list(flags)

    for i in range(1, len(points)):
        if flags[i - 1]:
            break
        duplicate = math.hypot(...) < 1e-6    # points[i-1] == points[i]
        moved = math.hypot(...) >= 1e-6        # points[0] != points[i-1]
        leading_off = not any(flags[:i])
        mark_or_pre_boundary = bool(flags[i] or any(flags[i:]))
        if duplicate and moved and leading_off and mark_or_pre_boundary:
            entry_pts = list(points[:i])
            return (entry_pts, [False]*len(entry_pts)), points[i:], flags[i:]
        if flags[i]:
            break
    return None, list(points), list(flags)
```

Returns `(entry_run | None, remaining_pts, remaining_flags)`. Densified legs are supported — the duplicate is not required to sit at index 1.

---

### `src/path_publisher_node.py` — Path Generation

Hardcoded path generators (line 195–297) — SITL/testing paths:

- `gen_straight_5m()` / `gen_straight_3m()`
- `gen_arc_quarter_1m5()` / `gen_arc_half_1m5()`
- `gen_lshape_2x2()` / `gen_square_2x2()` / `gen_rectangle_3x2()`
- `gen_circle_1m5()`

**`PathPublisherNode._publish_once()`** (line 524+) — loads path from file or generator, applies auto-origin offset, publishes `/path` with spray flags encoded in `pose.position.z` (1.0 = ON, 0.0 = OFF). Does **not** set the runtime-entry quaternion marker (server/GPS surveyed and point legs do).

Also publishes `/dyx/mission/progress` (1 Hz) for waypoint completion fraction.

---

### `src/point_leg_trajectory.py` — Point Leg Geometry

**`build_point_leg_path()`** (line 72–85):

```python
def build_point_leg_path(start, end, *, mode=TWO_POINT, spacing_m=0.08):
    if mode == TWO_POINT:
        return [start, end]
    return densify_point_leg(start, end, spacing_m)
```

**`predict_rpp_conditioning()`** (line 191–210) — predicts RPP profile + conditioned geometry for a point leg (orchestrator diagnostics). Densified + collinear + `runtime_entry=True` → `"smooth"` after resample; otherwise simplify → `"segment"`.

**`is_collinear_straight_leg()`** — True when every interior vertex is within **≤ 5°** of a straight line (`_COLLINEAR_TOL_DEG = 5.0`).

---

### `src/point_ingest.py` — Point Mission CSV/DXF Parsing

**`SprayPoint`** dataclass (line 27–32):

```python
@dataclass(frozen=True)
class SprayPoint:
    north_m: float
    east_m: float
    dwell_s: float | None
    source_index: int
    mark: bool = True
```

**`parse_point_csv_text()`** — header: `north,east[,dwell_s[,mark]]`  
**`parse_point_gps_csv_text()`** — header: `lat,lon[,dwell_s][,mark]` (first row = GPS anchor)  
**`parse_dxf_point_entities()`** — extract POINT entities from PathEngine DXFEntity objects.

---

### `src/path_identity.py` — Fingerprint & Identity

**`path_geometry_fingerprint()`** — SHA-256 of `[[n, e, flag], ...]` (precision 6 decimals).

**`make_path_identity()`** — JSON envelope: `{mission_id, path_fingerprint, configuration_revision, source}`.

Topics: `/path/identity` (raw), `/rpp/conditioned_path_identity` (conditioned by RPP).

---

## 4. Full Runtime Entry Flow Diagram

```
FastAPI Route
  │
  ├─ mission_loading.load_path_for_controller()
  │    ├─ path_mgr.load_path()               → PathEngine → waypoints
  │    ├─ spray_flags_for_path()             → preview_path → spray schedule
  │    └─ offboard_ctrl.load_path()          → normalize + store in memory
  │
  └─ start_mission_for_controller()
       └─ offboard_ctrl.start_async()
            └─ _start_async_locked()
                 │
                 ├─ [point] _start_point_shell_async()
                 │     arm → grace → OFFBOARD → RUNNING
                 │     (no /path; orchestrator publishes legs later)
                 │
                 └─ [continuous]
                       ├─ resolve_surveyed_points()     (if GPS_SURVEYED)
                       ├─ RPP health gate
                       ├─ auto_origin offset            (if local + auto_origin)
                       ├─ _build_runtime_entry_path()   (GPS_SURVEYED only)
                       │    └─ _densify_leg()           (0.05 m spacing)
                       ├─ ros_node.publish_path()
                       │    ├─ z = spray flag
                       │    ├─ quat marker if entry_transit_added
                       │    └─ /path + /path/identity
                       ├─ arm_async()
                       ├─ sleep(SETPOINT_STREAM_GRACE_S)  # 0.5 s
                       ├─ verify RPP ≠ STALE/RTK_WAIT/JUMP_SKIP/IDLE
                       └─ set_mode_async("OFFBOARD") → RUNNING

[Point legs only] PointMissionOrchestrator._publish_fresh_leg()
  └─ publish_path(leg, spray_flags=all OFF, runtime_entry=True)

[ROS2] rpp_controller_node._path_cb()
  ├─ detect runtime_entry_marked (quaternion check)
  ├─ spray flags from position.z
  ├─ split_leading_entry_transit() → entry_run + remaining
  ├─ _split_runs_by_flag()         → per-entity runs
  ├─ _split_run_at_corners()       → sub-split at hard corners
  ├─ _merge_collinear_runs()       → fuse collinear flag runs
  ├─ insert entry_run at index 0
  ├─ per-run conditioning:
  │    densified entry/OFF collinear: smooth resample only
  │    smooth: _smooth_corners() + _resample_path()
  │    segment: _simplify_path_for_profile()
  ├─ drop slivers < 5 cm (keep runtime_entry)
  └─ _apply_run(0)                 → set as active path

[rpp_controller_node._control_loop()]
  50 Hz → /rpp/velocity_ned + /rpp/debug (+ segment_debug / stop_debug)
  │
  ├─ TRACK_SEGMENT (pure pursuit on active segment)
  │    └─ → detected corner → PRE_CORNER_SLOWDOWN
  │
  ├─ CORNER_STOP (zero velocity hold at corner / run boundary)
  │    ├─ segment_stop_speed_threshold (0.08 m/s)
  │    ├─ segment_stop_dwell_s (0.30 s)
  │    ├─ entry pure-zero: dist 0.10 m, speed ≤ 0.005 m/s, dwell 1.0 s
  │    └─ → stop_certificate → CORNER_ALIGN
  │
  ├─ CORNER_ALIGN (pivot to next heading)
  │    ├─ segment_heading_tolerance_deg (2.0°)
  │    ├─ segment_align_settle_s (0.20 s)
  │    └─ → alignment_certificate → next segment / _advance_run
  │
  ├─ _hold_before_run_advance() (run boundary)
  │    └─ → collinear pass-through OR same CORNER_STOP + CORNER_ALIGN cycle
  │
  └─ DONE → /rpp/debug state_code=3

[Server] Telemetry Loop
  ├─ monitors /rpp/debug via RppStatusMonitor
  ├─ RPP DONE held for DONE_SETTLE_S (1.0 s) + fresh → complete_async()
  └─ complete_async()  (RUNNING only):
       ├─ _terminalize_spray()        → force_spray_off_confirmed + spray_enabled=False
       ├─ publish_stop_path()         → single-point path at rover
       ├─ _wait_until_rest()          → measured_speed < MISSION_COMPLETE_REST_SPEED_M_S
       ├─ set_mode_async("MANUAL")    → if MISSION_COMPLETE_SET_MANUAL (default on)
       ├─ arm_async(False)            → if MISSION_COMPLETE_DISARM (default on)
       └─ all confirmed → mark_completed()
            else → ERROR (completion_degraded) + warnings
```

---

## 5. Key Constants

### `server/offboard_controller.py` (module-level)

| Constant | Value | Used In |
|----------|-------|---------|
| `ENTRY_DENSIFY_SPACING_M` | 0.05 | `_densify_leg()` |
| `ENTRY_COINCIDENT_TOLERANCE_M` | 1e-6 | `_build_runtime_entry_path()` |

### `server/config.py`

| Constant | Value | Used In |
|----------|-------|---------|
| `DONE_SETTLE_S` | **1.0** | `RppStatusMonitor` done gate |
| `SETPOINT_STREAM_GRACE_S` | 0.5 | stream settle before OFFBOARD |
| `RPP_STALE` | -1 | unhealthy RPP detection |
| `RPP_IDLE` | 0 | post-publish not-ready gate |
| `RPP_DONE` | 3 | mission complete trigger |
| `RPP_UNHEALTHY_CODES` | {-1, 4, 5} | STALE / RTK_WAIT / JUMP_SKIP |
| `MISSION_COMPLETE_REST_SPEED_M_S` | 0.03 | rest gate in `complete_async` |
| `MISSION_COMPLETE_REST_TIMEOUT_S` | 2.0 | rest wait budget |
| `MISSION_COMPLETE_SET_MANUAL` | true (env) | gate MANUAL on complete |
| `MISSION_COMPLETE_DISARM` | true (env) | gate disarm on complete |

---

## 6. Key RPP Controller Params

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `mission_speed` | 0.35 m/s | Operator speed knob |
| `max_linear_vel` | 0.8 m/s | Hardware ceiling |
| `a_lat_max` | 0.3 m/s² | Lateral acceleration constraint |
| `min_lookahead_dist` | 0.52 m | Lookahead floor |
| `max_lookahead_dist` | 1.0 m | Lookahead ceiling |
| `lookahead_time` | 1.6 s | Velocity-scaled Ld |
| `xy_goal_tolerance` | 0.02 m | Goal acceptance radius |
| `approach_velocity_scaling_dist` | 1.5 m | Deceleration start distance |
| `path_resample_spacing_m` | 0.08 m | Path resample spacing |
| `corner_smooth_radius_m` | 0.5 m | Inscribed arc radius |
| `segment_corner_threshold_deg` | 45.0° | Hard corner classification |
| `segment_min_corner_speed` | 0.08 m/s | Pivot speed floor |
| `segment_stop_speed_threshold` | 0.08 m/s | Stop physical gate |
| `segment_stop_dwell_s` | 0.30 s | Stop settle dwell |
| `segment_align_settle_s` | 0.20 s | Align settle dwell |
| `segment_heading_tolerance_deg` | 2.0° | Align heading gate |
| `segment_boundary_capture_radius_m` | 0.50 m | Smooth-run terminal capture |
| `segment_entry_pure_stop_dist_m` | 0.10 m | Zero-command stop zone |
| `segment_entry_pure_stop_dwell_s` | 1.0 s | Pure-stop certification dwell |
| `segment_entry_pure_stop_speed_m_s` | 0.005 m/s | Pure-stop speed gate |
| `tracking_profile` | "auto" | auto/segment/smooth |
| `require_stop_certificates` | True | Production FSM gates |
| `use_feedforward_yaw_rate` | True | P3.1 body-rate mode |
| `max_yaw_rate_body` | 0.45 rad/s | Yaw rate clamp |
