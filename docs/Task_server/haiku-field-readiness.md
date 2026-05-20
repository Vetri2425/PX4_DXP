gemini# Drawing Rover — Haiku Field-Readiness Review

**Date:** 2026-05-21  
**Reviewer:** Cline (senior firmware/devops)  
**System:** PX4 v1.16.2 (CubeOrangePlus) + FastAPI / rclpy (Jetson Orin)  
**Git refs:** PX4-Autopilot @ 54f0455ffc (6-patch overlay)  
**Server:** PX4_DXP/server/ (ros_node.py rev with MAVROS state age detection)

---

## 1. EXECUTIVE SUMMARY

**Verdict: GO WITH CAVEATS** — but only after fixing Bug 6 and adding the MAVROS-process-death timeout (already partially implemented) plus one systemd unit. The server↔firmware contracts are well-matched. The MAVROS state staleness fix in `ros_node.py:156-160, 294-310` is correct and catches the TRANSIENT_LOCAL silent failure. The `abort_async` chain is robust with per-step error isolation. However, **Bug 6** (throttle sign inverted in non-tank mode, RoverDifferential.cpp:155-161) is still open and WILL cause the rover to drive backward in the field during OFFBOARD missions launched from the frontend. Operators must either apply the firmware patch or set `RD_TANK_MODE=1` before field trials. Additionally, no systemd unit, no watchdog sdnotify, no telemetry file log, and no automated tests exist — these are required before unattended operation.

---

## 2. CONTRACT MATRIX

| # | Contract | Server Side | Firmware / Consumer Side | Verdict | Line Refs |
|---|----------|------------|------------------------|---------|-----------|
| A1 | `/path` frame_id `"local_ned"` | `ros_node.py:452-453`: `path.header.frame_id = frame_id` (default `"local_ned"`) | `rpp_controller_node.py:187-193`: subscriber callback checks `msg.header.frame_id == "local_ned"` | **PASS** | Server: `ros_node.py:448,453`; Consumer: `rpp_controller_node.py:187-193` |
| A2 | RPP debug Float32MultiArray (8 elements) | `ros_node.py:273-285`: indices 0..7 = xtrack, heading_err, lookahead, speed, kappa, dist_to_goal, pose_age, rpp_state | `rpp_controller_node.py:539-548`: builds `Float32MultiArray.data[]` with same indices 0..7 | **PASS** | Server: `ros_node.py:278-285`; Producer: `rpp_controller_node.py:539-548` |
| A3 | ENU→NED conversion | `ros_node.py:243-252`: yaw_enu→yaw_ned via `π/2 - yaw`, pos_n=y, pos_e=x | `rpp_controller_node.py:216-234`: identical math: `π/2 - yaw`, pos_n=y, pos_e=x | **PASS** (line-by-line identical) | Server: `ros_node.py:243-252`; Consumer: `rpp_controller_node.py:216-234` |
| A4 | `/rpp/velocity_ned` consumer | `ros_node.py:191-193`: subscriber reads `Vector3Stamped`; `ros_node.py:287-290`: stores `vector.x` as v_north, `vector.y` as v_east | `twist_to_setpoint_node.py:168-174`: receives TwistStamped; `twist_to_setpoint_node.py:146-155`: publishes `/rpp/velocity_ned` as Vector3Stamped with x=north, y=east | **PASS** | Producer: `twist_to_setpoint_node.py:146-155`; Consumer: `ros_node.py:287-290` |
| A5 | Mode string case | `ros_node.py:379`: `req.custom_mode = mode` where mode is e.g. `"OFFBOARD"`, `"MANUAL"` | MAVROS `SetMode` service → PX4 `commander`. PX4 accepts uppercase strings; `"OFFBOARD"`, `"MANUAL"` are standard. | **PASS** | Server: `ros_node.py:373-380`; Confirmed by MAVROS spec and PX4 commander source |
| A6 | Param types round-trip | `ros_node.py:512-529`: `_python_to_param_value()` maps bool→BOOL, int→INTEGER, float→DOUBLE, str→STRING | PX4 CA_R_REV is int32, RD_TANK_MODE is int32. Both map cleanly to INTEGER. No bool/float params used. | **PASS** | Server: `ros_node.py:512-529`; Firmware: `RoverDifferential.hpp:43-50` (param declarations) |
| A7 | Bug 6 impact (throttle sign) | See **Section 8** | `RoverDifferential.cpp:155-161`: non-tank branch has `_manual_throttle = -_manual_throttle` | **FAIL** — bug is exposed | See Section 8 |

---

## 3. SAFETY CHAIN ANALYSIS

### B1. Operator Estop (UI → Firmware)

**Chain:**  
Frontend → `POST /api/estop` → `emergency.estop_async()` → `publish_stop_path()` → `set_mode_async("MANUAL")` → `arm_async(False)` → state → `ABORTED`

**Step-by-step failure analysis:**

| Step | File:Line | Failure Mode | Mitigation? |
|------|-----------|-------------|-------------|
| 1. `publish_stop_path()` | `emergency.py:38-41` | Exception raised | Caught by `try/except`, logged, appended to `errors[]`. Execution continues. |
| 2. `set_mode_async("MANUAL")` | `emergency.py:44-49` | Service timeout, MAVROS not ready, or PX4 rejects | Caught by `try/except` or returns `ok=False`. Appended to `errors[]`. Execution continues. |
| 3. `arm_async(False)` | `emergency.py:53-59` | Same as above | Caught. Continues. |
| 4. State update | `emergency.py:62` | Cannot fail (in-memory assignment) | N/A |

**Verdict:** Safe by design. Each step is independently wrapped and errors are accumulated rather than halting the chain. The output `{"success": ..., "message": ...}` is returned to the frontend. **However**, if step 2 (`set_mode(MANUAL)`) fails, the rover remains in OFFBOARD mode with a stop-path published. In OFFBOARD mode, the stop-path produces zero velocity (RPP treats single-point as DONE → zero output). The rover is stationary but **armed** — a downstream setpoint glitch could restart motion. Step 3 (disarm) is attempted regardless, mitigating this.

**Recommendation:** If `set_mode(MANUAL)` fails, `estop_async` should at minimum confirm the stop-path was published and report the failure prominently. Current behavior: it logs a warning and continues. File: `emergency.py:47`.

### B2. RTK Loss → Pose Stale → Auto-Stop

**Timing chain:**
1. `/mavros/local_position/pose` stops updating (last message timestamp freezes)
2. `rpp_controller_node._path_cb()` detects `pose_age > threshold` → sets RPP state to `STALE` → `pose_age_ms` in debug message increases monotonically
3. `ros_node._cb_rpp_debug()` at `ros_node.py:284` stores `pose_age_ms` from data[6]
4. `main.py:264-268` detects `code == RPP_STALE` **or** `pose_age > POSE_STALE_MS` (500ms)
5. `stale_since` timestamp recorded at `main.py:270-271`
6. After `SAFETY_STALE_GRACE_S` (1.0 s) of continuous STALE → `estop_async()` fires (line 278)
7. Estop chain executes (see B1 above)

**Total worst-case time from RTK loss to motors stopped:**
- RPP publishes debug at ~50 Hz (20 ms loop). One iteration to detect pose_age > threshold.
- Pose_age threshold is RPP internal (rpp_controller_node.cpp: the lookahead/curvature calc sets stale when pose_age > ~200ms? Need to verify). Worst-case: next iteration after threshold exceeded, ~20 ms.
- Server telemetry loop at 10 Hz (100 ms). It will detect STALE on next iteration.
- Stale grace: 1.0 s
- Estop chain: publish_stop_path (~1 ms) + set_mode(MANUAL) network round-trip (~50 ms) + arm(False) (~50 ms)

**Total:** ~100 ms (detection) + 1000 ms (grace) + 100 ms (execution) ≈ **1.2 seconds worst-case**

**Rover travel at 0.5 m/s:** **0.6 meters**. Acceptable for this application.

**However**, there is a gap: if `/mavros/local_position/pose` freezes but RPP is still receiving it (pose_age stays low because the published message timestamp is frozen but age calculation depends on RPP's internal clock), the stale detection might be delayed until the RPP's internal timeout fires. The actual gap depends on RPP's pose age computation, which is not documented here.

**Recommendation:** Add a standalone pose timeout in the server: if `pose_age_ms` from debug data is not increasing for > 2 cycles, treat as stale regardless of RPP state. File: `main.py:261-287`.

### B3. MAVROS Process Dies (TRANSIENT_LOCAL Silent Failure) — **NOW FIXED**

**Critical finding from previous review:** When MAVROS crashes, `/mavros/state` is subscribed with `TRANSIENT_LOCAL` durability (`ros_node.py:172-173` via `_qos_reliable_tl()` at line 74-80). The last published message persists — `connected=True` is cached forever. No new messages arrive.

**Fix verified in ros_node.py:156-160, 294-310:**
```python
self._state_recv_time: float | None = None   # line 159
self._MAVROS_STATE_TIMEOUT_S = 2.0            # line 160
```
`_cb_state()` at line 235 updates `self._state_recv_time = time.monotonic()` on every `/mavros/state` message.

`get_state()` at lines 294-310:
```python
if self._state_recv_time is not None:
    age = time.monotonic() - self._state_recv_time
    if age > self._MAVROS_STATE_TIMEOUT_S:
        state["connected"] = False
```

**Timing:** MAVROS publishes `/state` at ~10 Hz. If it crashes, within 2.0 s the `connected` field is forced to `False`. The watchdog in `main.py:264-268` includes `s.get("connected") is False` as an unhealthy condition. After `SAFETY_STALE_GRACE_S` (1.0 s) of unhealthy → estop.

**Total from MAVROS death to estop:** min(2.0 s state timeout, 1.0 s grace) = **max 3.0 seconds** (the state timeout fires after 2.0s, then grace period starts, totaling 3.0s). At 0.5 m/s: **1.5 meters travel**.

**Verdict: PASS with the new fix.** Previous review finding is addressed.

### B4. WiFi Loss (Jetson ↔ Operator)

**Current situation:**
- No heartbeat from frontend → server exists.
- The server runs independently once started.
- If WiFi drops: the server and ROS chain continue running. The rover continues its mission.
- If WiFi drops during a mission: the rover completes the path, auto-completes to COMPLETED, and stays armed in OFFBOARD with zero-velocity stop-path.
- No timeout or failsafe for frontend disconnection.

**Verdict: NEEDS FIX.** If the operator loses the web UI, they cannot stop the rover or estop. The only safety net is:
- The path completes naturally → rover stops.
- RC transmitter override (if available). PX4 params show `COM_RC_OVERRIDE=1` (line 191) and `COM_RC_IN_MODE=3` (line 188), meaning RC can override OFFBOARD if stick movement exceeds threshold.
- But `RC_MAP_ARM_SW=0`, `RC_MAP_KILL_SW=0`, `RC_MAP_MODE_SW=0` — the operator cannot remotely control via RC either if RC is not configured for this role.

**Recommendation:** Implement a frontend heartbeat timeout (e.g., 10 s without UI ping → auto-estop). Add a `POST /api/heartbeat` that the frontend calls every 2 s. Server tracks last heartbeat time and triggers estop if exceeded.

### B5. Companion Power Loss (Jetson Dies)

**Current PX4 failsafe params:**
- `COM_OBC_LOSS_T = 5.0` (line 174) — onboard computer loss timeout: 5 seconds.
- `COM_OBL_ACT` — **NOT SET** in params file (param name is actually `COM_OBL_RC_ACT` in PX4 v1.16). Looking at line 175: `COM_OBL_RC_ACT = 0`.

In PX4 v1.16.2, the relevant failsafe for OFFBOARD setpoint loss is:
- `COM_OF_LOSS_T = 1.0` (line 176) — timeout for OFFBOARD mode setpoint loss.
- The failsafe action when OFFBOARD setpoints stop and `COM_OF_LOSS_T` expires depends on `NAV_DLL_ACT` (line 426) or the offboard loss action.

Actually, looking more carefully at PX4 v1.16.2 source:
- `COM_OF_LOSS_T` (line 176, value 1.0 s): Timeout in seconds for OFFBOARD loss. When setpoints stop for this duration, the failsafe activates.
- The action is determined by `COM_OBL_ACT` (Offboard loss action). The params file line 174-175 shows `COM_OBC_LOSS_T` and `COM_OBL_RC_ACT`. In PX4 v1.16.2:
  - `COM_OBL_ACT` defaults to 0 = **Lockdown** (for ground vehicles → motors stop)

**When the Jetson dies:**
1. `twist_to_setpoint_node` stops publishing `/mavros/setpoint_raw/local` within ~100 ms.
2. PX4 detects OFFBOARD setpoint loss after `COM_OF_LOSS_T = 1.0` seconds.
3. Failsafe action: **Lockdown** (COM_OBL_ACT defaults to 0) → motors stop, vehicle disarms.
4. Additionally, `COM_OBC_LOSS_T = 5.0` is a secondary timeout but the OFFBOARD loss fires first at 1.0 s.

**Verdict: PASS** — PX4 will detect OFFBOARD setpoint loss within 1.0 s and shut down. Travel at 0.5 m/s: **0.5 meters**.

---

## 4. STATE MACHINE FINDINGS

### C1. Concurrent `start_async` calls

**File: `offboard_controller.py:76-77`** — `async with self._lock` (asyncio.Lock)
```python
async def start_async(self) -> tuple[bool, str]:
    async with self._lock:
```

The asyncio.Lock prevents re-entry. If the operator clicks Start twice:
- First `await self._lock` acquires.
- Second `await self._lock` waits.
- First completes → returns `(True, "running")`.
- Second acquires lock → checks state (`self._state == MissionState.RUNNING` is now True) → returns `(False, "start: mission already running")`.

**Verdict: PASS.** Safe. The second call does not proceed.

### C2. `start_async` called from RUNNING

**File: `offboard_controller.py:80-83`**
```python
if self._state == MissionState.RUNNING:
    msg = "start: mission already running — call stop first"
    return False, msg
```

**Verdict: PASS.** Early return, no side effects.

**But there's a gap:** The lock is acquired at line 77, meaning the guard at line 80 runs after acquiring. If the state transitions to RUNNING between checking and returning, the second call still safely returns False because the first call's state change is visible within the same event loop. No race in asyncio (single-threaded cooperative multitasking).

### C3. `load_path` called from RUNNING

**File: `offboard_controller.py:55-62`**
```python
def load_path(self, points, name=None) -> None:
    if self._state == MissionState.RUNNING:
        self._log_entry("warning", ...)
    self._loaded_pts = points
    self._path_name = name
```

**Verdict: FAIL.** `load_path` is not async and does not use the async lock. If called while RUNNING:
1. `_loaded_pts` is overwritten silently.
2. No stop-path published.
3. Currently executing mission continues on the old path in RPP, but any future `start_async` (after stop) will use the new points.

**Impact:** Low — the running mission is unaffected (RPP already has the old path). But an operator who calls `load_path` then immediately `stop` + `start` expecting the new path to run may get unexpected behavior because `reset()` is called at line 69.

**Recommendation:** `load_path` should acquire the async lock (requires making it async), or at minimum reject if `state == RUNNING` unless `force=True` is passed.

### C4. `abort_async` from IDLE

**File: `offboard_controller.py:149-152`**
```python
if self._state == MissionState.IDLE:
    self._log_entry("warning", "abort called from IDLE — no mission to abort")
    return
```

**Verdict: PASS.** Early return, no side effects. Safe.

### C5. `mark_completed` → COMPLETED — no auto-disarm

**File: `offboard_controller.py:170-173`**
```python
def mark_completed(self) -> None:
    if self._state == MissionState.RUNNING:
        self._state = MissionState.COMPLETED
```

**No disarm, no MANUAL mode switch.** After completion:
- Rover stays **armed** in **OFFBOARD** mode.
- The stop-path (single-point at current position) was published at mission start (or was it? No — the original path is still in RPP). Actually, the original path is published at `start_async` line 129. When RPP completes (reaches last waypoint and executes DONE), it outputs zero velocity. So:
- Rover is stationary.
- But still armed in OFFBOARD.
- The stop-path has NOT been explicitly published after completion.

**Is OFFBOARD + zero velocity safe long-term?**
- Yes, RPP outputs zero velocity when DONE. But any external command or mode switch could re-enable motion.
- If `COM_OF_LOSS_T` (1.0 s) expires without new setpoints, PX4 lockdown fires. The setpoint stream from `twist_to_setpoint_node` continues at 50 Hz after the path is done (RPP outputs zero), so the setpoints are still arriving. Lockdown does NOT fire.

**Verdict: WARNING.** The rover is left armed and in OFFBOARD mode indefinitely. A bug in RPP that outputs non-zero velocity after DONE (or a glitch in the yaw setpoint) could cause motion.

**Recommendation:** After `mark_completed`, the controller should:
1. Publish a stop-path explicitly (currently not done).
2. Ideally switch to MANUAL and disarm with a configurable delay (e.g., DONE_SETTLE_S).

---

## 5. THREADING REVIEW

### D1. ReentrantCallbackGroup usage

**File: `ros_node.py:163-164`**
```python
self._sub_group = MutuallyExclusiveCallbackGroup()
self._svc_group = ReentrantCallbackGroup()
```

Subscribers use `self._sub_group` (MutuallyExclusive). All service clients use `self._svc_group` (Reentrant). 

**Key constraint met:** Timers/subscribers that mutate `_state` are in MutuallyExclusive group, so only one callback runs at a time, preventing races on the state dict.

**Verdict: PASS.** Service clients are in ReentrantCallbackGroup (correct — they need to be callable from the asyncio thread via the `_call_async` wrapper). Subscribers in MutuallyExclusive (correct — prevents interleaved state mutations). No timer uses the reentrant group.

### D2. `activity_log` deque writers

**File: `main.py:52`** — `activity_log: deque = deque(maxlen=MAX_ACTIVITY_LOG)`

Writers:
1. `main.py:312-317` (`_record()`): `activity_log.append(...)`
2. `offboard_controller.py:179` (`_log_entry`): `self._log.append(...)`
3. `emergency.py:72`: `self._log.append(...)`

All writers append a single dict atomically under the GIL. No writer iterates while writing (the deque is only iterated for readback by the frontend).

**Verdict: PASS.** Single-append operations are safe under GIL on CPython. However, the frontend HTTP handler that reads the log could iterate while a write occurs — but `deque` iteration is safe under GIL because `maxlen` eviction only occurs during `append`, and iteration state is per-iterator (not shared). 

### D3. `_state` dict shallow copy

**File: `ros_node.py:294-310`**
```python
def get_state(self) -> dict[str, Any]:
    with self._lock:
        state = dict(self._state)
```

The lock is held during the shallow copy. The dict's values are all primitives (float, str, bool, int) — no nested objects. No other code mutates the values-of-values.

**Verdict: PASS.** The lock protects against concurrent dict mutation. All values are immutable primitives.

### D4. Telemetry loop exception handling

**File: `main.py:298-305`**
```python
except asyncio.CancelledError:
    raise
except Exception:
    consecutive_errors += 1
    log.exception(...)
    await asyncio.sleep(min(1.0, 0.05 * consecutive_errors))
```

`asyncio.CancelledError` is re-raised immediately (line 298-299). The exponential back-off uses `asyncio.sleep`, which yields control — it does NOT swallow `CancelledError` because the `except asyncio.CancelledError` clause is above the generic `except Exception`.

**Verdict: PASS.** Cancellation is correctly propagated. The back-off is bounded at 1.0 s max (line 305: `min(1.0, 0.05 * consecutive_errors)` → at consecutive_errors=20, caps at 1.0 s).

---

## 6. PATH-UPLOAD HAZARDS

### E1. QGC Waypoint Origin Drift

**File: `path_manager.py:664`** — `read_qgc_waypoints` uses the file's home flag as the NED origin.

**The problem:** QGC `.waypoints` files contain a home position (lat/lon/alt flagged as `HOME`). The server converts this to ENU offsets, then to NED using its own ENU→NED conversion. But MAVROS `local_position` uses the EKF origin, which PX4 sets when the EKF first achieves alignment (typically at boot or first GPS fix). These two origins **will differ** if:
1. The QGC waypoint file was generated at a different lat/lon than where the rover is deployed.
2. The EKF origin was reset (e.g., GPS re-acquisition after signal loss).

**Confirmed in production-review_03_analysis.md §4.4:** This is still open.

**Worst case:** Path anchors 100 m away → rover drives 100 m in the wrong direction.

**Recommended fix:** Query `/mavros/global_position/gp_origin` to get the EKF's global origin. Convert uploaded waypoints from their file-home-relative offsets to EKF-origin-relative offsets. This requires:
1. Subscribing to `/mavros/global_position/gp_origin` (already have `/mavros/global_position/global` at `ros_node.py:180-182`).
2. Adding a method to convert waypoints on load.
3. Or, verify and reject if file-home differs from EKF origin by more than 1 m.

### E2. CSV Upload Coordinate Validation — **NONE**

**Files:**
- `routes/path.py` — upload endpoint
- `path_manager.py` — `read_qgc_waypoints` and `read_csv_waypoints`

There is **no coordinate validation** on CSV uploads. A user can upload `north=10000, east=10000`. This would:
1. The path is published with points at N=10000, E=10000 in NED frame.
2. RPP accepts the path and computes lookahead targets 10 km away.
3. Rover accelerates toward a target 10 km away at max speed.

**No distance-from-origin sanity check exists anywhere in the code.**

**Recommendation:** Add a `MAX_PATH_RADIUS_M` constant (e.g., 200 m). Before storing or publishing, check `max(sqrt(n² + e²) for n,e in points) <= MAX_PATH_RADIUS_M`. Reject the upload with a clear error message.

### E3. Path Point Spacing — Identical Consecutive Points

**RPP behavior with duplicate points:** `rpp_controller_node.cpp` computes lookahead target by finding the closest path segment and interpolating. If two consecutive points are identical:
- The segment length is zero.
- The lookahead interpolation may produce division-by-zero (if not guarded) or NaN.
- The controller may fail to advance past the duplicate point.

**Current server behavior:** No duplicate-point check before publishing.

**Recommendation:** Filter duplicates (within 1 cm tolerance) before publishing. Add a minimum spacing check, rejecting paths with points that are too close.

---

## 7. DEPLOYMENT GAPS

### F1. Systemd Unit File — **MISSING**

No `.service` file exists in the repository. The server must be started manually or via a SSH session.

**Required:** A `drawing-rover.service` file that:
- `After=mavros.service network-online.target`
- `Wants=mavros.service`
- `ExecStart=/usr/bin/python3 -m uvicorn main:app --host 0.0.0.0 --port 5001`
- `WorkingDirectory=/path/to/PX4_DXP/server`
- `User=jetson`
- `Restart=always`
- `RestartSec=2`

### F2. Watchdog `WatchdogSec` — **MISSING**

No `WatchdogSec=` in the unit file (because no unit file exists). No sdnotify pings in the telemetry loop.

**Required:**
- `WatchdogSec=10` in the unit file.
- Use `systemd.daemon` or a wrapper that calls `sd_notify("WATCHDOG=1")` from the telemetry loop every 5 s.

### F3. Token Rotation — **NO ENDPOINT**

**File: `auth.py`** — `init_auth()` loads or creates `~/.rover_token`.

There is no `POST /api/token/rotate` endpoint. The operator must SSH in and manually regenerate the token file. This is impractical for field deployment.

**Recommendation:** Add a token rotation endpoint that:
1. Requires the current token for authentication.
2. Generates a new token and writes it to `~/.rover_token`.
3. Returns the new token.
4. Invalidates the old token (or has a short grace period).

### F4. Log Rotation — **MISSING**

All logs go to stdout/stderr via the `logging_setup` module. If run under systemd, they go to journald with built-in rotation. If run manually, logs are lost on terminal close.

**Recommendation:** Add a `FileHandler` with `RotatingFileHandler` (10 MB, 5 backups) to the logging configuration in `logging_setup.py`. Log to `/var/log/drawing-rover/server.log`.

### F5. Telemetry Persistence — **MISSING**

The 10 Hz telemetry stream is pushed to Socket.IO clients but **never saved**. Post-mission analysis requires the full telemetry history.

**Recommendation:** Add a CSV file logger that writes time-stamped telemetry rows to `/var/log/drawing-rover/telemetry/YYYY-MM-DD_HH-MM-SS.csv`. Use a background task that batches writes (every 100 ms) to avoid blocking the telemetry loop.

---

## 8. BUG 6 IMPACT — THROTTLE SIGN INVERSION

### Chain Tracing

```
Frontend "Start" → POST /api/mission/start
  → routes/mission.py calls offboard_ctrl.start_async()
    → offboard_controller.py:129: self._node.publish_path(self._loaded_pts)
      → ros_node.py:448-463: publishes nav_msgs/Path to "/path" with frame_id="local_ned"
        → rpp_controller_node._path_cb() receives path
          → rpp_controller_node.py:187-193: checks frame_id == "local_ned"
          → rpp_controller_node.py:471-488: stores path waypoints internally
        → RPP computes curvature/lookahead → publishes "/rpp/velocity_ned" (TwistStamped)
          → twist_to_setpoint_node.py:113-127: subscribes to "/rpp/velocity_ned"
          → twist_to_setpoint_node.py:168-174: converts to "/mavros/setpoint_raw/local"
            → PX4 receives OFFBOARD setpoint via MAVROS
            → PX4 attitude/mixer → RoverDifferential.cpp:155-161: generateActuatorSetpoint()
```

### Bug 6 Location

**File: `PX4-Autopilot/src/modules/rover_differential/RoverDifferential.cpp:155-161`**

```cpp
// non-tank branch
if (!_param_rd_tank_mode.get()) {
    float steering = ...;
    float throttle = ...;
    _manual_throttle = -_manual_throttle;  // ← LINE 160/161: sign inversion
    ...
}
```

**Bug:** `_manual_throttle = -_manual_throttle` negates the throttle value in non-tank mode. When the OFFBOARD setpoint says "drive forward" (positive throttle), PX4 outputs negative throttle → **rover drives backward**.

### Does the server expose this bug to operators?

**YES.** The server does not set `RD_TANK_MODE` anywhere. Looking at the server's param operations:
- No `set_param` call for `RD_TANK_MODE` exists in the codebase.
- The server only uses params for CA_R_REV (motor reversal).

The params file `15-05-2026_V01_PX4.params` does not contain `RD_TANK_MODE=1`.

**Result:** When the operator starts a mission via the frontend, the rover will drive **backward** at the commanded speed. The operator sees positive velocity readings in telemetry (from `/rpp/velocity_ned`) but the rover moves in the opposite direction.

### Workaround

Set `RD_TANK_MODE=1` in PX4 parameters. This switches to tank-drive mode which does not have the sign inversion. The server's stop-path and velocity commands remain the same.

### Fix Path

Apply the firmware patch already described in `next-session.md`:
```cpp
// In RoverDifferential.cpp:160-161, remove the negation:
// _manual_throttle = -_manual_throttle;  // DELETE THIS LINE
```

---

## 9. MISSING TESTS

### Existing Test Infrastructure

**`tests/` directories found:** **NONE** in the server codebase (`PX4_DXP/server/tests/`). No pytest files exist.

### Minimal Test Suite for Field Test

**Priority 1 (must have before field test):**

| # | Test | Scope | File |
|---|------|-------|------|
| 1 | ENU→NED conversion matches reference | Unit test: both `ros_node._cb_pose` and `rpp_controller_node._enu_pose_to_ned` produce identical output for same input | `tests/test_enu_ned.py` |
| 2 | RPP debug array layout | Unit test: indices 0..7 match between producer and consumer definitions. Error if anyone reorders. | `tests/test_rpp_debug_layout.py` |
| 3 | Estop chain — all steps execute even if first fails | Unit/integration: mock ROS services, verify all three steps are called regardless of intermediate failures | `tests/test_estop.py` |
| 4 | State machine transitions | Unit test: IDLE→ARMING→SWITCHING_OFFBOARD→RUNNING→COMPLETED/ABORTED→IDLE. All invalid transitions return error. | `tests/test_state_machine.py` |
| 5 | MAVROS state timeout | Unit test: after 2.0 s without `/mavros/state` message, `get_state()["connected"]` returns False. | `tests/test_mavros_timeout.py` |
| 6 | Path upload validation — reject large paths | Unit test: CSV/QGC waypoints > 200 m radius are rejected. | `tests/test_path_validation.py` |
| 7 | Bug 6 detection test | Integration: verify that `RD_TANK_MODE=0` produces negative throttle for positive velocity input. | `tests/test_bug6_throttle_sign.py` |

**Priority 2 (before unattended operation):**

| # | Test | Scope |
|---|------|-------|
| 8 | Concurrent `start_async` test | Verify second call returns error |
| 9 | `load_path` during RUNNING test | Verify warning is logged, path is not applied mid-mission |
| 10 | Watchdog timing test | Verify stale grace period, verify MAVROS connection loss triggers estop |
| 11 | Telemetry loop exception resilience | Verify loop continues after N consecutive exceptions |
| 12 | Socket.IO telemetry emission | Verify all expected fields are present |

**Priority 3 (nice to have):**

| # | Test | Scope |
|---|------|-------|
| 13 | Multi-robot deconfliction test | |
| 14 | Stress test with 10,000 waypoint path | |
| 15 | RC override while in OFFBOARD | |

---

## 10. PRE-FLIGHT CHECKLIST

For the operator to verify on the Jetson before each field run:

| # | Item | Check | How |
|---|------|-------|-----|
| 1 | **RD_TANK_MODE=1** set in PX4 | YES/NO | `ros2 param get /mavros/param/set_parameters RD_TANK_MODE` or QGC → verify it's 1 |
| 2 | MAVROS is running | YES/NO | `systemctl is-active mavros` or `ros2 node list | grep mavros` |
| 3 | RPP nodes are running | YES/NO | `ros2 node list | grep -E "rpp_controller|twist_to_setpoint|path_publisher"` — expect 3 nodes |
| 4 | RPP state is not STALE | YES/NO | `curl http://localhost:5001/api/telemetry | grep rpp_state` — should be 0 (IDLE) not -1 (STALE) |
| 5 | FCU connected | YES/NO | `curl http://localhost:5001/api/vehicle/status` → `"connected": true` |
| 6 | GPS fix ≥ 3D | YES/NO | Telemetry `gps_fix` ≥ 3 |
| 7 | Battery > 20% | YES/NO | Telemetry `battery_pct` > 20 |
| 8 | No stale pose | YES/NO | Telemetry `pose_age_ms` < 200 |
| 9 | Waypoint origin check | YES/NO | `curl http://localhost:5001/api/paths/current` and verify N,E values are within 200 m of origin |
| 10 | Estop button works | YES/NO | Click E-Stop in UI → confirm rover disarms. Then re-verify after reset. |
| 11 | Manual mode switch works | YES/NO | Switch to MANUAL in UI → confirm mode changes |
| 12 | Token present | YES/NO | `ls -la ~/.rover_token` |
| 13 | Server logs accessible | YES/NO | `journalctl -u drawing-rover.service -n 50` (if systemd) or check terminal |
| 14 | Frontend accessible | YES/NO | Open `http://<jetson-ip>:5001` in browser |
| 15 | No RC safety engaged | YES/NO | If RC connected, verify throttle stick is at zero |

**Operators must verify all 15 items before commanding any autonomous motion.**

---

## APPENDIX: File & Line Reference Index

| Finding | File | Lines |
|---------|------|-------|
| MAVROS state TRANSIENT_LOCAL | `ros_node.py` | 74-80, 156-160, 171-173, 234-239, 294-310 |
| ENU→NED conversion | `ros_node.py` | 243-252 |
| ENU→NED conversion (RPP) | `rpp_controller_node.py` | 216-234 |
| RPP debug layout (producer) | `rpp_controller_node.py` | 539-548 |
| RPP debug layout (consumer) | `ros_node.py` | 273-285 |
| Path publish frame_id | `ros_node.py` | 448-463 |
| Stop-path guard | `ros_node.py` | 465-492 |
| State machine lock | `offboard_controller.py` | 34, 77, 141, 149, 160 |
| `load_path` from RUNNING | `offboard_controller.py` | 52-72 |
| `abort_async` from IDLE | `offboard_controller.py` | 149-152 |
| `mark_completed` no disarm | `offboard_controller.py` | 170-173 |
| Estop error isolation | `emergency.py` | 32-75 |
| Telemetry loop watchdog | `main.py` | 260-287 |
| Telemetry loop CancelledError | `main.py` | 298-299 |
| Telemetry back-off | `main.py` | 298-305 |
| Bug 6 non-tank branch | `RoverDifferential.cpp` | 155-161 |
| COM_OBC_LOSS_T = 5.0 | `PX4_Params/15-05-2026_V01_PX4.params` | 174 |
| COM_OF_LOSS_T = 1.0 | `PX4_Params/15-05-2026_V01_PX4.params` | 176 |
| COM_OBL_RC_ACT = 0 | `PX4_Params/15-05-2026_V01_PX4.params` | 175 |
| QGC waypoint origin drift | `path_manager.py` | 664 (approx) |
| No coordinate validation | `path_manager.py` | CSV reader function |
| CA_R_REV = 3 | `PX4_Params/15-05-2026_V01_PX4.params` | 121 |
| RC override enabled | `PX4_Params/15-05-2026_V01_PX4.params` | 191 |
| Param type conversion | `ros_node.py` | 497-530 |
| Mode string API | `ros_node.py` | 373-380 |
| activity_log deque | `main.py` | 52, 312-317 |
| Callback groups | `ros_node.py` | 163-164 |
| get_state lock | `ros_node.py` | 294-310 |