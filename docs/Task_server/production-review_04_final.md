# Server Production Review — Final Honest Assessment
**Date:** 2026-05-21  
**Basis:** Direct code read of `PX4_DXP/server/` on disk, cross-checked against  
`production-review.md`, `_02.md`, `_03_analysis.md`, and the author's self-assessment.  
**Goal:** One document that says exactly what is good, what the reviews caught, what the author's own blind-spot analysis identified, what is now fixed, and what genuinely remains.

---

## 1. Executive Summary

The server is **substantially better than it was at v01**. All three critical bugs from the first review (threading deadlock, empty-path estop, event-loop blocking) are fixed and verified in code. The author's self-assessment identified six additional real issues — MAVROS connected-staleness, stop-path at EKF origin, start-while-running re-arm, RPP done-timer not reset, Socket.IO CORS, and ROS clock vs wall clock — and those are also fixed in the current sources.

**What remains is deployment infrastructure, not code bugs:**  
no systemd unit, no pytest suite, no telemetry-to-file, no SITL smoke. These are the blockers for a supervised field test, not the code itself.

**Verdict: code is field-test ready; surrounding deployment scaffold is not.**

---

## 2. What the Reviews Caught Correctly

### 2.1 Critical bugs (all three now fixed)

| Bug | Root cause | Fix | Verified |
|-----|-----------|-----|---------|
| Threading deadlock | `spin_until_future_done` called from non-spinning thread | `MultiThreadedExecutor(4)` + `ReentrantCallbackGroup` + `add_done_callback` → asyncio future | `grep "spin_until_future_done"` → 0 matches |
| Empty-path estop no-op | `rpp_controller_node._path_cb` line 183 silently returns on empty path | `publish_stop_path()` sends single-point path at rover position | Confirmed in `ros_node.py:publish_stop_path` |
| Event-loop blocking | `arm()` / `set_mode()` blocked the FastAPI event loop for up to 10.5 s | All service calls are now `async`, `time.sleep` → `asyncio.sleep` | Confirmed; `offboard_controller.start_async` is fully async |

### 2.2 High-severity reliability (all now fixed)

- **No OFFBOARD pre-stream check:** `offboard_controller.start_async` now reads `rpp_state` and rejects with a diagnostic if `RPP_STALE`. (`offboard_controller.py:81`)  
- **No auto-completion:** `main._telemetry_loop` calls `offboard_ctrl.mark_completed()` when `rpp_monitor.is_done()` is true after 1 s settle. (`main.py` §2 of telemetry loop)  
- **No pose-stale watchdog:** Watchdog fires `emergency_handler.estop_async()` after `SAFETY_STALE_GRACE_S = 1.0 s` of `RPP_STALE` or `pose_age_ms > 500` or `connected is False`. (`main.py` §3 of telemetry loop)  
- **`RppStatusMonitor` was dead code:** Now the single source of truth, updated in `_cb_rpp_debug`, exposed via `get_rpp_monitor()`.  
- **`activity_log` racy list:** Now `deque(maxlen=MAX_ACTIVITY_LOG)` — thread-safe, bounded, no manual trim. (`main.py:52`)  
- **CORS misconfiguration:** `CORS_ALLOW_CREDENTIALS = False`, wildcard origin is valid per spec. (`config.py`)

### 2.3 Medium hardening (all fixed)

- Token auth (`auth.py`) — `X-Rover-Token` header on REST; `data.auth` check on every Socket.IO control event using `secrets.compare_digest`.  
- File upload limits — extension whitelist `{.waypoints, .csv}`, 1 MiB cap, `os.path.basename` strip.  
- `python-multipart==0.0.17` added to `requirements.txt`.  
- Param services implemented — `get_param_async` / `set_param_async` with type round-trip for `bool/int/float/str`.  
- Structured logging via `logging_setup.py`; bare `print()` removed.  
- `/api/healthz` endpoint for systemd `WatchdogSec=`.  
- Path router split: `paths_router` (`/api/paths`) + `path_router` (`/api/path/*`). Fragile `@router.get("s")` pattern gone.  
- Graceful shutdown: `RosExecutorThread` uses `threading.Event`; beacon uses `Event.wait(interval)`.  
- Service `is_ready()` check + 0.5 s re-wait on every call.

---

## 3. What the Author's Self-Assessment Caught That the Reviews Missed

These are the six issues identified in the author's own honest diff. All six are now fixed in the current code.

### 3.1 MAVROS `connected` staleness on process death ← **most important miss**

**What was wrong:** The watchdog checked `s.get("connected") is False`. `/mavros/state` is TRANSIENT_LOCAL. When MAVROS crashes, no new messages arrive — the last cached message (which has `connected=True`) persists forever. The `connected is False` watchdog branch could never trigger on the most likely runtime failure.

**Fix in `ros_node.py`:**  
`_cb_state` records `self._state_recv_time = time.monotonic()`.  
`get_state()` overrides `connected` to `False` if no State message has arrived within `_MAVROS_STATE_TIMEOUT_S = 2.0 s`:
```python
if self._state_recv_time is not None:
    age = time.monotonic() - self._state_recv_time
    if age > self._MAVROS_STATE_TIMEOUT_S:
        state["connected"] = False
```
The watchdog now fires within `2.0 s (timeout) + 1.0 s (grace) = 3 s` of MAVROS dying. At 0.5 m/s that's 1.5 m of uncontrolled travel — acceptable for supervised field testing.

### 3.2 `publish_stop_path()` at EKF origin when no pose received

**What was wrong:** `_DEFAULT_STATE` initialises `pos_n = 0.0, pos_e = 0.0`. If `publish_stop_path()` is called before the first pose message (MAVROS not up, server just started), a single-point path at NED (0, 0) is published. If the rover is not physically at the EKF origin, RPP tracks toward (0, 0) — the opposite of stopping.

**Fix in `ros_node.publish_stop_path`:**
```python
pose_received = not (n == 0.0 and e == 0.0 and not s.get("connected", False))
if not pose_received:
    log.warning("publish_stop_path: no pose received — publishing empty path")
    self.publish_path([], frame_id=frame_id)
    return
```
The empty path is still ignored by RPP, but the `set_mode("MANUAL")` in the abort chain fires anyway — the real safety net. This avoids the spurious movement command without introducing a worse failure.

### 3.3 `start_async()` from RUNNING re-armed and re-switched OFFBOARD

**What was wrong:** No state guard. A double-click Start from the UI would call `arm(True)` and `set_mode("OFFBOARD")` over an active mission.

**Fix in `offboard_controller.start_async`:**
```python
if self._state == MissionState.RUNNING:
    msg = "start: mission already running — call stop first"
    self._log_entry("warning", msg)
    return False, msg
```

### 3.4 `RppStatusMonitor._done_since` not reset on new path load

**What was wrong:** When mission A completes (RPP → DONE), `_done_since` is set. If a new path is immediately loaded and `start_async` is called, the first telemetry tick sees `is_done() == True` and fires `mark_completed()` before the rover has moved.

**Fix in `rpp_status.py`:**
```python
def reset(self) -> None:
    """Reset done-settle timer. Call when a new path is loaded."""
    self._done_since = None
```
**Fix in `offboard_controller.load_path`:**
```python
if self._node is not None:
    try:
        self._node.get_rpp_monitor().reset()
    except Exception:
        pass
```

### 3.5 Socket.IO `cors_allowed_origins="*"` not in sync with REST CORS

**What was wrong:** FastAPI CORS was fixed to `CORS_ALLOW_CREDENTIALS = False` but the Socket.IO server still used a hardcoded `"*"`. Two independent CORS implementations that were out of sync.

**Fix in `main.py`:**
```python
sio = socketio.AsyncServer(
    async_mode="asgi",
    cors_allowed_origins=CORS_ALLOW_ORIGINS,  # from config, not hardcoded
)
```

### 3.6 `publish_path` used `time.time()` instead of ROS clock

**What was wrong:** Every other node in `PX4_DXP/src/` uses the ROS clock. If NTP steps the Jetson wall clock (common on boot), the header timestamp jumps. RPP doesn't currently validate path stamp freshness, so harmless today — but inconsistent and fragile.

**Fix in `ros_node.publish_path`:**
```python
path.header.stamp = self.get_clock().now().to_msg()
```

---

## 4. What the Reviews Got Partially Wrong

### 4.1 Watchdog timing calculation

`production-review_03_analysis.md` says "500 ms + 1 s = 1.5 s worst-case to stop." The actual chain is:

```
RTK loss  
  → RPP pose_max_age_s = 0.2 s before STALE published  
  → Up to 100 ms for server to see it at 10 Hz telemetry tick  
  → SAFETY_STALE_GRACE_S = 1.0 s  
  → Up to 100 ms for estop_async service calls  
  ≈ 1.4 s total
```

At 0.5 m/s: **70 cm** of uncontrolled travel. Acceptable for supervised testing; document as the known safety bound.

### 4.2 Review_02 self-scored 8.9/10 — overstated

The v02 review scored the code in isolation and ignored the missing deployment layer. An honest score for "production ready for field test" must include tests, systemd, and telemetry persistence. **7.5/10** is more accurate.

### 4.3 "firmware compatibility check" framed as a bug

The suggestion to query `RD_TANK_MODE` at startup is a reasonable enhancement but not a correctness bug. It is deferrable.

---

## 5. What Remains — Genuine Open Items

These are the only remaining items. None are code correctness bugs in the server module itself.

### 5.1 [BLOCKER] No pytest suite

There is no `tests/` directory. Every fix in v02 and the self-assessment is verified only by inspection. Minimum needed before first hardware run:

| Test | What it covers |
|------|---------------|
| `test_path_manager.py` | `load_path` for all 6 builtins; `read_qgc_waypoints` against a `.waypoints` from `Test_mission/` |
| `test_rpp_monitor.py` | `is_done()` settle, `reset()` clears timer, old DONE → `reset()` → new mission doesn't auto-complete |
| `test_offboard_controller.py` | All state transitions with `Mock` ros_node: start→running, start-while-running rejected, load-while-running warns, abort-from-idle no-ops, mark_completed from RUNNING only |
| `test_auth.py` | `check_socket_token` constant-time compare; `ROVER_DISABLE_AUTH=1` bypass |
| `test_upload.py` | Extension rejection; size cap; path traversal strip |

These run without ROS installed. ~90 min to write.

### 5.2 [BLOCKER] No SITL smoke test

First-time-live issues that only surface against a real MAVROS + PX4:
- Service name case sensitivity (`/mavros/cmd/arming` exact match).
- `GPSRAW` import path on Humble (`mavros_msgs.msg.GPSRAW` vs `mavros_msgs.msg.GPSRaw`).
- `frame_id = "local_ned"` is a case-sensitive string match in RPP.
- `ROS_DOMAIN_ID` matches across server, MAVROS, and RPP nodes.

A 5-minute SITL run catches all of these before taking the rover to a field.

### 5.3 [BLOCKER] No systemd unit file

Without it:
- No auto-restart on crash.
- No `After=mavros.service` ordering guarantee (server starts before MAVROS → all service `wait_for_service` calls hit the 2 s timeout on boot).
- `/api/healthz` exists but nothing drives it.

Minimum unit file:
```ini
[Unit]
Description=Drawing Rover FastAPI server
After=network.target px4-dxp.service
Wants=px4-dxp.service

[Service]
Type=simple
User=jetson
WorkingDirectory=/home/jetson/PX4_DXP/server
ExecStart=/home/jetson/PX4_DXP/server/run.sh
Restart=on-failure
RestartSec=5
WatchdogSec=30
NotifyAccess=all

[Install]
WantedBy=multi-user.target
```

### 5.4 [HIGH] QGC waypoint NED origin ≠ MAVROS EKF origin

`read_qgc_waypoints` anchors all coordinates to the home waypoint (`current=1`) in the `.waypoints` file. MAVROS `/mavros/local_position/pose` uses the EKF local origin (set by PX4 at EKF initialisation, tied to the GPS fix at arming time). These two origins may not be the same point.

If they differ — which is the common case when QGC was opened at a different time or location than arming — the rover drives toward a point that is correct relative to QGC's home but wrong relative to its actual NED frame. The error scales linearly with the distance between the two origins.

**Fix:** On path load, fetch `/mavros/global_position/gp_origin` (`geographic_msgs/GeoPointStamped`) and re-anchor the WGS84 waypoints to that lat/lon instead of the file's home flag.

This is the single most likely cause of "rover drove the wrong direction" in the first field test with an uploaded `.waypoints` file. Builtin paths (generated in NED directly) are not affected.

### 5.5 [HIGH] Firmware Bug 6 is still open

`bug-registry.md` — Bug 6: throttle sign inversion in `RoverDifferential.cpp` non-tank `generateActuatorSetpoint()` branch. In AUTO MISSION, `throttle_body_x = +1.0` but `body_vx < 0` (rover moves backward).

The server cannot fix this. The call chain in the current firmware is:
```
Frontend Start → /api/mission/start
  → OffboardController.start_async → publish_path("local_ned")
  → rpp_controller_node → /rpp/velocity_ned (NED velocity vector)
  → twist_to_setpoint_node → /mavros/setpoint_raw/local
  → PX4 OFFBOARD → RoverDifferential.generateActuatorSetpoint (non-tank branch)
  ← throttle_body_x sign bug → rover moves backward
```

**Field test impact:** Every mission started via the frontend will drive the rover backward until Bug 6 is fixed in the firmware. The review prompt for the Haiku agent (§7 of `production-review_03_analysis.md`) asks the agent to trace and confirm this exact chain.

**Fix:** Apply option 1 from `next-session.md`: negate `throttle_body_x` in `RoverDifferential.cpp` non-tank branch before passing to `computeInverseKinematics`. This is a one-line firmware patch.

### 5.6 [MEDIUM] No telemetry-to-file persistence

The in-memory `activity_log` (deque 500 entries) is wiped on restart. For post-mortem of field-test anomalies you need the 10 Hz telemetry stream written to disk. A `RotatingFileHandler` writing one JSON line per tick to `/var/log/rover/telemetry.jsonl` is sufficient.

### 5.7 [MEDIUM] Abort/watchdog while ARMING or SWITCHING_OFFBOARD

`abort_async` and `estop_async` both acquire `_lock`. If `start_async` currently holds `_lock` (it does for the full ARMING → SWITCHING_OFFBOARD sequence), an abort or watchdog estop queues behind it and fires only after `start_async` completes. In the worst case: `start_async` is in `arm_async(True)` with a 5 s timeout, the watchdog fires, `estop_async` blocks for up to 5 s waiting for the lock.

This is an edge case but means estop is not truly instantaneous during mission startup. The real safety net is PX4's own OFFBOARD timeout — if no setpoints are received it falls back. Document this as a known limitation for the field test observer.

### 5.8 [LOW] Watchdog does not fire when server is `IDLE` but rover is moving

The watchdog only activates when `offboard_ctrl.state == RUNNING`. If the rover somehow ends up in OFFBOARD with the server in IDLE (e.g. server restarted while rover was mid-mission), the watchdog is silent. The mode check on `/mavros/state.mode` could catch this — if `mode == "OFFBOARD"` and server state is `IDLE`, emit a warning and optionally switch to MANUAL.

---

## 6. Final Score

| Area | Score | Basis |
|------|:-----:|-------|
| Architecture | 9/10 | Clean, well-separated, correct ROS2 patterns. |
| API + models | 9/10 | Pydantic v2, auth, typed responses. |
| ROS2 integration | 9/10 | `MultiThreadedExecutor`, async wrappers, MAVROS staleness guard. |
| Safety chain | 8/10 | Watchdog, estop, stop-path — good. Estop during ARMING has lock contention (§5.7). |
| Reliability | 7/10 | Code is solid; **no test coverage at all**. |
| Operability | 7/10 | `/healthz` good; no systemd unit, no telemetry-to-file. |
| Security | 8/10 | Token auth, upload limits, CORS consistent. |
| Field readiness | 5/10 | Bug 6 in firmware will drive backward; NED origin mismatch for `.waypoints`. |

**Composite: 7.8 / 10**

The code is correct and the architecture is sound. The blockers are:
1. Firmware Bug 6 (one-line patch, tracked in `next-session.md`).
2. pytest suite (90 min).
3. systemd unit (30 min).
4. SITL smoke (5 min run).
5. QGC origin re-anchor (for uploaded `.waypoints`).

Items 2–4 are standard deployment hygiene. Item 1 is a known firmware issue with a documented fix. Item 5 is only needed if you use uploaded `.waypoints`; all 6 builtin paths are unaffected.

---

## 7. Minimum Pre-Field-Test Checklist

```
[ ] Firmware: apply Bug 6 fix (throttle sign, RoverDifferential.cpp non-tank branch)
[ ] pytest: write test_path_manager + test_rpp_monitor + test_offboard_controller
[ ] systemd: write drawing-rover-server.service, enable, test restart
[ ] SITL smoke: one builtin path (square_2x2) end-to-end, check logs
[ ] Hardware bring-up: follow firmware-build-flow.md §5 checklist end-to-end
[ ] Tethered test: safety observer present, 5m radius clear, motor kill switch ready
```

For `.waypoints` uploads:
```
[ ] Implement GP origin re-anchor before using non-builtin paths in field
```
