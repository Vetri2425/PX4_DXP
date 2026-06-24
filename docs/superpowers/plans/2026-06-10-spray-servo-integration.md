# Phase 3 — Spray / Marking Servo Integration (Pixhawk AUX)

**Date:** 2026-06-10 (rev 4 — Task 02 architecture update; software implementation reconciled
against current source)
**Author:** Vetri
**Scope:** Wire a marking servo/solenoid to a CubeOrangePlus AUX output, drive it from the
runtime MARK/TRANSIT state with low-latency `rclpy` commands, and surface a
`marking / transit` status to the GCS frontend.
**Layers touched:** PX4 firmware *config only* (QGC params), Jetson companion (ROS2/MAVROS),
FastAPI server, frontend telemetry.

> Task 02 supersedes older `/spray/active`-driven wording in historical sections below:
> production spray uses `/rpp/conditioned_path` identity-bound geometry; `/spray/active`
> is compatibility fallback/telemetry only when distance-aware spray is disabled.

---

## 0. Current codebase audit — 2026-06-11

This document started as an implementation plan. The runtime software items below are now
implemented in the current repo; keep only hardware/field-validation items in the tracker.

### Implemented in source

- `server/ros_node.py` — `publish_path(points, spray_flags=...)` writes `pose.position.z`
  (`1.0` MARK, `0.0` TRANSIT/OFF), subscribes `/spray/state` and `/spray/manual_state`,
  and publishes `/spray/manual` for bench override.
- `server/mission_loading.py`, `server/routes/mission.py`, `server/routes/path.py`,
  `server/sockets/events.py`, and `server/offboard_controller.py` forward `spray_flags`
  into controller-loaded/published paths.
- `path_engine/core.py` and `path_engine/engine.py` carry `PlannedPath.spray_flags`,
  MARK/TRANSIT segments, entity overrides, and exact CAD PRE/MARK/AFT geometry.
  `path_engine/spray.py` is legacy offline compensation only.
- `src/rpp_controller_node.py` reads exact CAD PRE/MARK/AFT flags from `/path` z,
  preserves transition vertices through `_smooth_corners()` and `_resample_path()`,
  publishes `/rpp/conditioned_path` plus identity for production spray timing, and
  keeps `/spray/active` as telemetry/legacy fallback.
- `src/spray_controller_node.py` consumes `/rpp/conditioned_path`, validates mission
  identity/configuration revision, owns runtime timing, flow, speed safety, actuator
  state, manual override, and fail-safe OFF on disarm / non-OFFBOARD / shutdown.
- `src/launch/rpp_pipeline.launch.py` and `rpp_start.sh` both start
  `spray_controller_node.py`; systemd uses `rpp_start.sh`.
- `server/routes/spray.py`, `server/models.py`, `server/routes/telemetry.py`, and
  `server/main.py` expose `/api/spray/test`, `/api/spray/status`, `spraying`, and
  `marking_state`.
- Tests exist for the high-risk software paths: `src/test_spray_flag_conditioning.py`,
  `src/test_spray_manual_override.py`, `server/test_spray_routes.py`, and
  `path_engine/tests/test_spray.py`.

### Still open outside source

- QGC AUX function/min/max/disarmed/failsafe configuration on the hardware.
- Solenoid/relay/MOSFET wiring, flyback protection, and isolated supply.
- Bench confirmation that `MAV_CMD_DO_SET_ACTUATOR` command 187 moves the chosen AUX pin.
- Hardware safety validation: disarm, E-stop, non-OFFBOARD, staleness, and shutdown all
  force spray OFF with the real actuator.
- Latency calibration with the real solenoid/driver and production debounce settings.
- Frontend/mobile status-pill verification in the GCS app, which is outside this repo.

### Key existing hooks we will reuse
- `rpp_controller_node` already tracks the **current segment index** (`_closest_seg_hint`)
  at 50 Hz and publishes `/rpp/debug` (append-only `Float32MultiArray`, indices `[0..46]`
  used). This is the natural, lowest-latency place to decide "are we on a MARK segment."
- `twist_to_setpoint_node` owns the OFFBOARD heartbeat — **do not** add actuator logic
  there; keep it pure.
- MAVROS `command` plugin is **not** in the denylist (`px4_pluginlists_rover.yaml`), so
  `/mavros/cmd/command` (`CommandLong`) is available. (`actuator_control` *is* denied —
  we must not use raw PWM passthrough.)

---

## 1. Firmware / FCU side — the correct command (config in QGC only)

> CLAUDE.md hard rules: **do not edit PX4 firmware on Jetson, do not push FCU params from
> Jetson.** All FCU config below is done in **QGC on the Mac** (source of truth).

### 1.1 Verified command — `MAV_CMD_DO_SET_ACTUATOR` (187)

Confirmed in the fork firmware (`PX4-Autopilot`, v1.16.2 base):

- `src/lib/mixer_module/functions/FunctionActuatorSet.hpp` subscribes to `vehicle_command`,
  matches `VEHICLE_CMD_DO_SET_ACTUATOR`, requires **`param7` (index) == 0**, and maps:

  | MAVLink param | Actuator set | Output function |
  |---|---|---|
  | `param1` | Set 1 | `Peripheral_via_Actuator_Set1` |
  | `param2` | Set 2 | `Peripheral_via_Actuator_Set2` |
  | … | … | … |
  | `param6` | Set 6 | `Peripheral_via_Actuator_Set6` |
  | `param7` | **index = 0** (required) | — |

- Value range: **`-1.0 … +1.0`** maps to the output's `MIN … MAX` PWM. `NaN` = "leave
  this channel unchanged" (so we only ever set `param1`, leaving the other five `NaN`).
- `src/lib/mixer_module/output_functions.yaml`: `Peripheral_via_Actuator_Set` **start =
  301** → `Set1 = 301`, `Set2 = 302`, …
- `Commander.cpp:1571` acks `DO_SET_ACTUATOR` as `ACCEPTED`; the mixer consumes the
  `vehicle_command` directly, so a `COMMAND_LONG` over MAVLink reaches the AUX pin.

**Why this and not `DO_SET_SERVO` (183):** PX4's first-class, supported path for an
offboard auxiliary actuator is the actuator-set function. `DO_SET_SERVO` support on PX4 is
inconsistent across outputs; `DO_SET_ACTUATOR` + `Peripheral_via_Actuator_Set` is the
documented, control-allocation-aware mechanism. (We are PX4, not ArduRover — ArduRover is
abandoned per CLAUDE.md.)

### 1.2 QGC configuration (one-time, on the Mac)

1. Pick a free **AUX** output on the CubeOrangePlus (e.g. AUX5).
2. Set that output's **function param → `301`** (Peripheral via Actuator Set 1).
   (Actuators tab in QGC → assign "Peripheral via Actuator Set 1" to the AUX pin.)
3. Set the output limits for that pin:
   - `MIN` PWM → spray **OFF** PWM (e.g. 1000 µs)
   - `MAX` PWM → spray **ON** PWM (e.g. 2000 µs)
   - **`DISARMED`** value → **OFF** PWM. This is the critical safety property:
     **when the vehicle is disarmed, the AUX output holds the disarmed (OFF) value**, so
     disarm == spray off, always.
   - `FAILSAFE` → OFF PWM.
4. Record the chosen Actuator Set index (1 → `param1`) and the AUX pin in
   `~/PX4_DXP/config/` notes.

### 1.3 Hardware wiring

- **On/off solenoid (recommended for marking paint):** AUX signal pin → a logic-level
  MOSFET / opto-isolated relay module → solenoid valve. Solenoid powered from its own
  supply with a flyback diode; **do not** draw solenoid current from the FMU rail.
  Command ON = `param1 = +1.0` (drives `MAX` PWM → driver high); OFF = `param1 = -1.0`.
- **Proportional servo valve (alternative):** AUX signal → servo directly; ON/OFF map to
  two PWM endpoints, or use intermediate values for partial flow.
- Common ground between FMU and the driver. Keep the signal wire short / away from motor
  power leads (RoboClaw lines) to avoid PWM noise.

> **Decision needed from operator:** on/off solenoid via relay, or a PWM servo valve?
> The software below is identical (binary ON/OFF); only the QGC `MIN/MAX` endpoints and
> the wiring differ.

---

## 2. Companion (Jetson) — ROS2 / MAVROS implementation

### Data flow (implemented)

```
Planner (spray_flags) ──► server.publish_path() ──► /path  (nav_msgs/Path)
                                                     pose.position.z = spray flag (1.0/0.0)
                                                          │
rpp_controller_node ── _path_cb: read z → per-point flags ─┐
                       propagate flags through corner-     │
                       smoothing + resampling (P1.3)       │
                       _closest_seg_hint (50 Hz) ──────────┘
                          │ decides MARK vs TRANSIT for current segment
                          ├─► /rpp/debug[39] = spray_active (telemetry)
                          └─► /spray/active  (std_msgs/Bool, best-effort, 50 Hz)
                                       │
                          spray_controller_node  ◄── edge-detect + debounce
                                       │             + staleness watchdog (OFF if quiet)
                                       ├─► /mavros/cmd/command  (CommandLong: DO_SET_ACTUATOR)
                                       └─► /spray/state (std_msgs/Bool) ──► server ──► frontend
```

**Why the flag rides inside `/path` (z-channel) instead of a side topic:**
1. `std_msgs` MultiArray types have **no header** — a separate `/spray/flags` topic
   cannot be stamp-matched to the path, leaving a race where flags from path N apply
   to path N+1. Embedding the flag in the `PoseStamped` makes path+flags **atomic**.
2. The controller's `_path_cb` **conditions** the path (corner smoothing + linear
   resampling, P1.3) — waypoint count and indices change (`corner_smooth_radius_m=0.5`
   is a validated, active param). Any externally-indexed flag array would misalign with
   `_closest_seg_hint`, which indexes the *conditioned* path. Carrying the flag on each
   point lets conditioning propagate it naturally (§2.2).
3. `pose.position.z` is currently ignored by the controller (`raw_pts` takes only
   `x, y` at `_path_cb`) and is always set to `0.0` by `publish_path()` — backward
   compatible: an old-format path reads as all-`False` (spray off, fail-safe).
   No new message package, no QoS pairing, no new topic.

**Why split controller (decision) from servo node (action):** the controller already owns
path geometry and the segment projection — it is the authoritative, lowest-latency source
of "on a MARK segment." The dedicated **servo node** owns the actuator contract
(edge detection, debounce, MAVLink command, fail-safe). This matches the existing
separation (`rpp_controller` = geometry, `twist_to_setpoint` = OFFBOARD contract) and
satisfies the "create a proper node for servo controller" requirement.

### 2.1 Propagate spray flags to runtime (via `/path` z-channel) — implemented

**`server/ros_node.py`**
- `publish_path()` accepts `spray_flags: list[bool] | None = None` and sets
  `ps.pose.position.z = 1.0 if flag else 0.0` per point. `None` or a length-mismatched
  list → all `0.0` (spray off, fail-safe) + one warning log.
- `publish_stop_path()` needs **no change**: it calls `publish_path([(n, e)])` with no
  flags → the single point carries `z = 0.0` → spray off. (Belt-and-braces: the
  single-point path is DONE on the first tick, and the controller forces
  `spray_active = False` in DONE — §2.2.)

**Flag convention (pinned — single source of truth):**
- `PlannedPath.spray_flags` is **per-point**, parallel to `merged_waypoints`
  (`path_engine/core.py`: "True = spray ON" at that point). Keep it per-point on the
  wire (`pose.position.z`).
- The runtime decision is per-*segment*: **segment i→i+1 is MARK iff
  `flags[i] AND flags[i+1]`** (both endpoints ON). Rationale: at a TRANSIT→MARK
  boundary the shared merged waypoint may carry either value depending on merge order;
  AND-ing makes the boundary deterministic and errs toward OFF (no paint outside the
  mark). Production no longer inserts planner lead-in points; exact CAD MARK
  boundaries remain in the planned geometry, and runtime timing anticipation is
  handled by the spray controller against RPP-conditioned geometry.
- **Calibration invariant:** this convention shifts the effective ON boundary by at most
  one waypoint spacing; it is constant for a given path, so §4's measured-latency
  feedback absorbs it. Do not change the convention after latency calibration without
  re-measuring.

**Callers:** current mission/path routes and Socket.IO load handlers pass the
`spray_flags` produced by `path_manager` / staged plans. Built-in or legacy paths without
MARK metadata use the configured legacy default from `server/config.py`.

### 2.2 `rpp_controller_node` — decide MARK state (no actuator I/O here) — implemented

**Critical constraint this design must respect:** `_path_cb` conditions the path —
`_smooth_corners()` then `_resample_path()` (P1.3) — so `len(self._path)` ≠ the
published waypoint count whenever `corner_smooth_radius_m > 0` or
`path_resample_spacing_m > 0` (both active with validated params). `_closest_seg_hint`
indexes the **conditioned** path. Spray flags must therefore be **propagated through
conditioning**, never index-matched against the raw path.

- `_path_cb` reads per-point flags from the incoming poses:
  `raw_flags = [p.pose.position.z > 0.5 for p in msg.poses]` (alongside the existing
  `raw_pts` extraction).
- **Flags propagate through conditioning.** `_smooth_corners()` and `_resample_path()`
  carry a parallel flag list:
  - *Resampling:* an interpolated point inherits the flag of the **raw segment** it
    lies on, evaluated with the §2.1 AND rule (`flags[i] and flags[i+1]`).
  - *Corner smoothing:* arc points replacing corner `i` inherit
    `flags[i-1] and flags[i] and flags[i+1]` (a corner is sprayed only if both
    adjoining segments are MARK) — errs toward OFF at mixed corners.
  - Unit-test both propagations (raw→conditioned flag mapping) — this is the highest
    bug-risk step in the whole plan.
- `self._spray_flags` is stored parallel to `self._path` after conditioning and resets with
  `_closest_seg_hint` (line ~504). Old-format paths (all `z = 0.0`) yield all-`False`.
- In the 50 Hz control tick, after the segment projection updates `_closest_seg_hint`:
  `seg = self._closest_seg_hint` →
  `spray_active = self._spray_flags[seg] and self._spray_flags[min(seg + 1, len - 1)]`.
  Force `spray_active = False` whenever the controller is not actively tracking
  (state `DONE`, `RTK_WAIT`, `JUMP_SKIP`, emergency-stop, or pose stale) — never spray
  while parked or in fault.
- Publishes:
  - `/spray/active` — `std_msgs/Bool`, **best-effort, depth 1**, every tick (50 Hz).
    A dedicated typed topic is lower-latency for the servo node than decoding the float
    debug array.
  - `/rpp/debug[39]` — `spray_active` (1.0/0.0). The current layout extends through
    `[46]`; indices `[0..7]` remain unchanged.

### 2.3 `src/spray_controller_node.py` (rclpy) — implemented servo controller

Responsibilities: edge-detect the desired spray state, debounce, fire the MAVLink
actuator command with minimal latency, expose manual bench override, and enforce fail-safes.

Design for **low latency** (`rclpy`):
- Subscribe `/spray/active` (`Bool`, best-effort depth 1) on a `ReentrantCallbackGroup`.
- Pre-create the `CommandLong` client to `/mavros/cmd/command` at startup and
  `wait_for_service()` once; never re-create it in the hot path.
- On a **state change** (rising/falling edge), immediately issue `call_async()` with an
  `add_done_callback` (fire-and-forget — never block the callback / never
  `spin_until_future_complete`, same rule the rest of the codebase follows).
- **Debounce / anti-chatter:** require the new state to persist for `N` consecutive
  samples (e.g. 2–3 ticks ≈ 40–60 ms) **or** a small dwell timer before committing, so
  flag flicker at a segment boundary cannot machine-gun the solenoid. Tunable
  `debounce_samples` param.
- **Re-assert latch (reliability):** the command is fire-and-forget over a serial link, so
  re-send the *current desired* state at a low rate (e.g. 2 Hz) to guarantee the AUX pin
  matches intent even if one `COMMAND_LONG` is dropped. Tunable `reassert_hz`.
- **Fail-safe OFF:** subscribe `/mavros/state`; if `armed == False` **or**
  `mode != OFFBOARD`, force OFF and stop re-asserting. On node shutdown / SIGINT, send a
  final OFF. (The FCU `DISARMED` PWM is the hardware backstop; this is the software one.)
- **Staleness watchdog (required):** if no `/spray/active` message arrives for
  `active_timeout_s` (default **0.5 s** ≈ 25 missed ticks at 50 Hz), force OFF and stop
  re-asserting until messages resume. Without this, an `rpp_controller_node` crash
  mid-MARK would leave the re-assert loop holding the solenoid ON until disarm.
- **Ack handling:** in the `add_done_callback`, check `response.success` /
  `response.result` (PX4 acks `DO_SET_ACTUATOR` as `ACCEPTED`); log a warning on
  failure. The re-assert loop is the recovery path — never retry synchronously.
- Publish `/spray/state` (`std_msgs/Bool`) — the *actual commanded* state — for the server.

CommandLong field mapping (spray ON):

```python
req = CommandLong.Request()
req.command = 187          # MAV_CMD_DO_SET_ACTUATOR
req.param1  = 1.0          # Actuator Set 1 → +1.0 = ON (MAX pwm)
                           # NOTE: which paramN carries the value is derived from the
                           # `actuator_set_index` param (Set k → param<k>); all other
                           # set params stay NaN. Do not hardcode param1 in the impl.
req.param2  = float('nan') # leave Set 2..6 unchanged
req.param3  = float('nan')
req.param4  = float('nan')
req.param5  = float('nan')
req.param6  = float('nan')
req.param7  = 0.0          # index 0 (REQUIRED by FunctionActuatorSet)
req.confirmation = 0
req.broadcast    = False
# OFF: req.param1 = -1.0
```

Params (declared with defaults): `actuator_set_index` (1), `on_value` (1.0),
`off_value` (-1.0), `debounce_samples` (3), `reassert_hz` (2.0),
`require_offboard` (True), `active_timeout_s` (0.5),
`manual_override_timeout_s` (10.0), `command_service` ("/mavros/cmd/command").

### 2.4 Launch + service wiring — implemented

- `src/launch/rpp_pipeline.launch.py` includes `spray_controller_node`.
- `rpp_start.sh` starts and watchdogs `spray_controller_node`; this is the systemd path.
- Confirm the `command` plugin loads (it is not denied). If `/mavros/cmd/command` is
  missing at runtime, the node logs and idles (no crash).
- systemd: `spray_controller` belongs to the `rpp-pipeline` unit (Python-only, ~2 s
  restart, does not drop MAVROS — matches the restart table in CLAUDE.md).

---

## 3. Server + frontend — `marking / transit` status

**Backend status:** implemented.

**`server/ros_node.py`**
- Subscribes `/spray/state` (`Bool`); stores `self._state["spraying"]`.

**`server/models.py` — `TelemetryData`**
- Add `spraying: Optional[bool] = None`
- Add `marking_state: Optional[Literal["marking", "transit", "off"]] = None`

**Telemetry builder / `sockets/events.py`**
- Derive `marking_state`:
  - not armed / mission not `RUNNING` → `"off"`
  - running + `spraying` → `"marking"`
  - running + not `spraying` → `"transit"`
- Push over the existing WebSocket telemetry stream (10 Hz).

**Frontend (web + mobile)**
- Verify the mobile/frontend repo displays `telemetry.marking_state` as MARKING /
  TRANSIT / OFF. That repo is outside `PX4_DXP`, so this remains an external UI
  verification task rather than a backend-code task here.

---

## 4. Latency budget & tuning

End-to-end ON/OFF latency from "rover crosses MARK boundary" to "solenoid actuates":

| Stage | Approx. | Notes |
|---|---|---|
| Segment detection | ≤ 20 ms | controller 50 Hz tick |
| `/spray/active` publish + servo node callback | ~1–5 ms | best-effort, intra-host DDS |
| Debounce dwell | `debounce_samples × 20 ms` | tunable; 0 for min latency |
| MAVROS service → MAVLink serial | ~5–20 ms | `/dev/ttyACM0` @ 921600 |
| PX4 mixer → AUX PWM update | ~2–5 ms | output rate |
| Solenoid mechanical | device-specific | dominant, often 10–50 ms |

**Production ownership:** the planner preserves exact CAD PRE/MARK/AFT geometry,
RPP owns motion tracking and path conditioning, and the spray controller owns
runtime timing, flow, safety, and actuator state. Procedure:
1. Measure real end-to-end latency on the bench (§5) **with the production
   `debounce_samples` value in place** — debounce is part of the latency being
   compensated (3 samples ≈ 60 ms ≈ 2.1 cm at 0.35 m/s).
2. Feed measured open/close delays, margins, speed window, and speed-to-PWM table
   into the spray controller calibration profile. Do not shift MARK geometry in
   the planner.
3. Keep `debounce_samples` as low as chatter allows — every debounce tick is added lag.
4. **Re-measure** after changing `debounce_samples`, the flag boundary convention
   (§2.1), or mission speed — all three shift the effective boundary.

This split (exact geometry in planning + distance-aware timing/flow at runtime) is
what keeps the **marking start/stop accurate** without moving spray ownership into
RPP or the planner.

---

## 5. Testing & validation

**Bench (wheels off ground, props/blades irrelevant — it's a rover):**
1. QGC: confirm AUX output assigned to function 301; arm in a safe mode; verify
   `DISARMED` PWM = OFF on the pin (scope or servo tester).
2. Manual command test:
   ```bash
   ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
     "{command: 187, param1: 1.0, param2: .nan, param3: .nan, param4: .nan, \
       param5: .nan, param6: .nan, param7: 0.0}"
   ```
   Confirm the AUX pin goes to MAX (ON); repeat with `param1: -1.0` → MIN (OFF).
   **Must be armed — any mode works.** Verified in the fork's
   `mixer_module.cpp::output_limit_calc()`: disarmed → all channels hold `DISARMED`
   PWM; armed → the function value drives the pin regardless of flight mode. (The
   OFFBOARD requirement exists only in `spray_controller_node`'s software fail-safe,
   not in firmware — do not let a non-OFFBOARD bench test mislead you.)
3. `ros2 topic echo /spray/active` and `/spray/state` while stepping a known path.

**Integration (SITL or short hardware run):**
- Run a `lshape_2x2` / `square_2x2` path with alternating MARK/TRANSIT flags; verify
  `/spray/active` toggles exactly at segment boundaries and `/spray/state` follows.
- Verify **disarm forces OFF** mid-mission, and **E-stop** (`publish_stop_path`) forces
  OFF (single-point path → `z = 0.0` → all-`False`, and controller DONE state forces
  `spray_active = False`).
- Verify mode-out-of-OFFBOARD (RC override) forces OFF.
- Verify the **staleness watchdog**: `kill` the rpp controller mid-MARK (bench only,
  wheels off) → spray must drop OFF within `active_timeout_s`.
- Verify **flag propagation through conditioning**: publish a path with a MARK→TRANSIT
  boundary at a smoothed corner; confirm `/spray/active` falls OFF at/before the corner
  (AND rule), never after it.

**Latency measurement:**
- Instrument with an LED/current sensor on the solenoid line + a GPIO/log timestamp at
  command send; or `ros2 bag record /spray/active /spray/state /mavros/local_position/pose`
  and correlate the boundary crossing vs. measured actuation. Feed result into §4 step 2.

**Unit tests implemented in this repo:**
- `src/test_spray_manual_override.py` covers manual override, timeout, fail-safe priority,
  staleness scoping, shutdown OFF, and command value behavior.
- `src/test_spray_flag_conditioning.py` covers resampling, smoothing, and endpoint AND
  behavior for propagated flags.
- `server/test_spray_routes.py` covers `/api/spray/test` and `/api/spray/status`.
- `path_engine/tests/test_spray.py` covers the legacy offline compensation helper.
  Production double-compensation guards live in production geometry and server
  path tests.

**Still needs hardware/integration validation:** real AUX output movement, solenoid
latency, and fail-safe OFF behavior on the wired actuator.

---

## 6. File-by-file change checklist

| File | Status / Change |
|---|---|
| **QGC (Mac)** | Open: assign AUX output function = 301; set MIN/MAX/DISARMED/FAILSAFE PWM (OFF disarmed) |
| `server/ros_node.py` | Implemented: writes spray flag into `pose.position.z`; subscribes `/spray/state`; stores `spraying`; publishes `/spray/manual` |
| `server/path_manager.py` / `server/routes/path.py` | Implemented: `spray_flags` are planned, previewed, staged, and forwarded to runtime |
| `server/models.py` | Implemented: `TelemetryData.spraying`, `TelemetryData.marking_state`, `SprayTestRequest` |
| `server/main.py` / `server/routes/telemetry.py` | Implemented: derive + emit `marking_state` |
| `src/rpp_controller_node.py` | Implemented: reads z flags, conditions flags, computes `spray_active`, publishes `/spray/active` + `/rpp/debug[39]`, forces OFF when not tracking |
| `src/spray_controller_node.py` | Implemented: edge-detect, debounce, staleness watchdog, manual override, `DO_SET_ACTUATOR`, ack-check + log, re-assert, fail-safe OFF, publish `/spray/state` |
| `src/launch/rpp_pipeline.launch.py` / `rpp_start.sh` | Implemented: `spray_controller_node` included in launch and systemd startup path |
| Tests | Implemented for spray flag conditioning, manual override, server routes, and controller-owned timing/flow guards |
| Frontend (web + mobile) | External verification remains: MARKING / TRANSIT / OFF status pill bound to `marking_state` |

### Remaining validation order
1. FCU config in QGC + bench-verify `DO_SET_ACTUATOR` manually (§5.1–5.2). *Proves the
   hardware path before any code.*
2. Endpoint-level manual test through `/api/spray/test`.
3. Mixed MARK/TRANSIT mission-path check with real actuator disconnected or safely benched.
4. Hardware fail-safe validation: disarm, E-stop, mode exit, upstream staleness, shutdown.
5. Frontend status verification in the GCS repo.
6. Latency measurement + feed back into the spray controller calibration profile.

---

## 7. Safety summary (must hold before field use)

- Disarm → AUX holds `DISARMED` (OFF) PWM at the **firmware** level (hardware backstop,
  verified in `mixer_module.cpp::output_limit_calc()` — applies in every flight mode).
- `spray_controller_node` forces OFF on: disarm, mode ≠ OFFBOARD, node shutdown,
  **`/spray/active` staleness > `active_timeout_s`** (upstream node crash), E-stop
  (single-point path → `z = 0.0` → all-`False`), and whenever the controller is not
  actively tracking.
- Unknown/old path format (no z flags) degrades to all-OFF, never all-ON.
- Solenoid powered from its own supply with flyback protection; never off the FMU rail.
- No FCU params pushed from Jetson; QGC remains source of truth (CLAUDE.md hard rule).

---

## 8. Open decisions for the operator
1. **Marking device:** on/off solenoid via relay/MOSFET, or PWM servo valve? (Affects QGC
   endpoints + wiring only.)
2. **AUX pin + Actuator Set index** (default: AUX5, Set 1 → `param1`).
3. **Default spray flags for non-DXF/built-in paths:** all-ON (current behaviour) or
   all-OFF for test paths?
4. **Debounce vs. latency trade-off:** starting `debounce_samples` (default 3 ≈ 60 ms) and
   `reassert_hz` (default 2 Hz).
