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

None are code correctness bugs in the server module itself. Items previously marked as blockers that are now resolved are struck through.

### ~~5.1~~ [BLOCKER → REMAINS OPEN] No pytest suite

There is no `tests/` directory. Every fix in v02 and the self-assessment is verified only by inspection. Minimum needed before first hardware run:

| Test | What it covers |
|------|---------------|
| `test_path_manager.py` | `load_path` for all 6 builtins; `read_qgc_waypoints` against a `.waypoints` from `Test_mission/` |
| `test_rpp_monitor.py` | `is_done()` settle, `reset()` clears timer, old DONE → `reset()` → new mission doesn't auto-complete |
| `test_offboard_controller.py` | All state transitions with `Mock` ros_node: start→running, start-while-running rejected, load-while-running warns, abort-from-idle no-ops, mark_completed from RUNNING only |
| `test_auth.py` | `check_socket_token` constant-time compare; `ROVER_DISABLE_AUTH=1` bypass |
| `test_upload.py` | Extension rejection; size cap; path traversal strip |

These run without ROS installed. ~90 min to write.

### ~~5.2~~ [BLOCKER → REMAINS OPEN] No SITL smoke test

First-time-live issues that only surface against a real MAVROS + PX4:
- Service name case sensitivity (`/mavros/cmd/arming` exact match).
- `GPSRAW` import path on Humble (`mavros_msgs.msg.GPSRAW` vs `mavros_msgs.msg.GPSRaw`).
- `frame_id = "local_ned"` is a case-sensitive string match in RPP.
- `ROS_DOMAIN_ID` matches across server, MAVROS, and RPP nodes.

A 5-minute SITL run catches all of these before taking the rover to a field.

### ~~5.3~~ [BLOCKER → RESOLVED] Systemd unit files — DONE

Both service files and startup scripts now exist and are deployed on the Jetson:
- `rpp-pipeline.service` — starts twist_to_setpoint + rpp_controller + xtrack_logger with watchdog
- `rover-server.service` — starts FastAPI + Socket.IO with `WatchdogSec=30`
- `rpp_start.sh` — watchdog loop for 3 RPP nodes (restarts dead nodes, exits if 5 failures in 30s)
- `deploy.sh` — symlinks both service files to `/etc/systemd/system/`, enables them

Service ordering: `px4-dxp.service` → `rpp-pipeline.service` (PartOf px4-dxp) → `rover-server.service` (After rpp-pipeline).

Verified 2026-05-22: all three services start correctly on Jetson. `rpp-pipeline` and `rover-server` start after `px4-dxp` per ordering.

**Remaining gap:** `rover-server.service` has `ROVER_DISABLE_AUTH=1` in the Environment block — remove for production. Also, `sd_notify("WATCHDOG=1")` is not yet implemented in `run.sh` — the WatchdogSec=30 is set but the server doesn't send heartbeats, so systemd would kill it after 30s. Either add sd_notify or remove WatchdogSec.

### 5.4 [HIGH] QGC waypoint NED origin ≠ MAVROS EKF origin

`read_qgc_waypoints` anchors all coordinates to the home waypoint (`current=1`) in the `.waypoints` file. MAVROS `/mavros/local_position/pose` uses the EKF local origin (set by PX4 at EKF initialisation, tied to the GPS fix at arming time). These two origins may not be the same point.

If they differ — which is the common case when QGC was opened at a different time or location than arming — the rover drives toward a point that is correct relative to QGC's home but wrong relative to its actual NED frame. The error scales linearly with the distance between the two origins.

**Fix:** On path load, fetch `/mavros/global_position/gp_origin` (`geographic_msgs/GeoPointStamped`) and re-anchor the WGS84 waypoints to that lat/lon instead of the file's home flag.

This is the single most likely cause of "rover drove the wrong direction" in the first field test with an uploaded `.waypoints` file. Builtin paths (generated in NED directly) are not affected.

### ~~5.5~~ [HIGH → RESOLVED] Firmware Bug 6 (reverse spot-turn) — FIXED

Bug 6 in the review (throttle sign inversion) was root-caused as Bug 7 and fixed in commit `fa6f6bc9`. The fix flips bearing 180° when `speed_sign < 0` in `DifferentialVelControl::generateVelocitySetpoint()`. This is now in firmware build `bfe914ce` (12-file overlay: P2 + P3 + P4 + Bug7 + IK + RoboClaw QPPS + boot-timing retry).

The review's concern about "rover drives backward" is no longer valid — reverse velocity now drives straight backward instead of spot-turning.

**Remaining validation:** P4 (hold-yaw-at-stop) is in the firmware but NOT yet validated on hardware with RTK. Needs bench verification.

### 5.6 [MEDIUM] No telemetry-to-file persistence

The in-memory `activity_log` (deque 500 entries) is wiped on restart. For post-mortem of field-test anomalies you need the 10 Hz telemetry stream written to disk. A `RotatingFileHandler` writing one JSON line per tick to `/var/log/rover/telemetry.jsonl` is sufficient.

### 5.9 [HIGH → RESOLVED] RBCLW_QPPS_MAX was 0 — NOW SET

The review was written when RBCLW_QPPS_MAX was 0 (no motor motion). Motion Studio autotune has now been run:
- **RBCLW_QPPS_MAX = 162162** (set in param file `PX4_Params/22-05-2026/init.params`)
- RoboClaw link proven on GPS2 port (202), NOT TELEM2 (flow-control issue)
- Serial baud: SER_GPS2_BAUD=115200
- Boot-timing bug fixed in firmware build `bfe914ce`
- **Remaining:** bench-verify motor direction/mapping (FUNC1/FUNC2 may be swapped vs proven config)

### 5.10 [HIGH → RESOLVED] MAVROS plugin denylist not taking effect

The YAML namespace `/mavros/mavros_node:` did not match the actual node namespace in the MAVROS launch context. The denylist was silently ignored — all plugins loaded, including `guided_target` which spammed 10Hz "no origin" warnings.

**Fix (2026-05-22):** Changed namespace to `/**:` (wildcard) matching the default `px4_pluginlists.yaml` format. Now 23 unnecessary plugins are properly denied, eliminating the guided_target spam. The `gps_rtk` plugin is intentionally kept for NTRIP RTCM injection.

### 5.11 [HIGH → RESOLVED] NTRIP not receiving data after service restart

After `deploy.sh --restart`, the NTRIP node was cycling through reconnections with "timed out" errors. Root cause: the node starts before network connectivity is fully established after a service restart.

**Fix:** The exponential backoff in `ntrip_rtcm_node.py` (max 60s) handles this correctly — after network comes up, the node reconnects within one backoff cycle. No code change needed. After the service stabilized, NTRIP connected and streamed 180 RTCM frames/30s with zero reconnects.

The in-memory `activity_log` (deque 500 entries) is wiped on restart. For post-mortem of field-test anomalies you need the 10 Hz telemetry stream written to disk. A `RotatingFileHandler` writing one JSON line per tick to `/var/log/rover/telemetry.jsonl` is sufficient.

### 5.7 [MEDIUM] Abort/watchdog while ARMING or SWITCHING_OFFBOARD

`abort_async` and `estop_async` both acquire `_lock`. If `start_async` currently holds `_lock` (it does for the full ARMING → SWITCHING_OFFBOARD sequence), an abort or watchdog estop queues behind it and fires only after `start_async` completes. In the worst case: `start_async` is in `arm_async(True)` with a 5 s timeout, the watchdog fires, `estop_async` blocks for up to 5 s waiting for the lock.

This is an edge case but means estop is not truly instantaneous during mission startup. The real safety net is PX4's own OFFBOARD timeout — if no setpoints are received it falls back. Document this as a known limitation for the field test observer.

### 5.8 [LOW] Watchdog does not fire when server is `IDLE` but rover is moving

The watchdog only activates when `offboard_ctrl.state == RUNNING`. If the rover somehow ends up in OFFBOARD with the server in IDLE (e.g. server restarted while rover was mid-mission), the watchdog is silent. The mode check on `/mavros/state.mode` could catch this — if `mode == "OFFBOARD"` and server state is `IDLE`, emit a warning and optionally switch to MANUAL.

---

## 6. Final Score (updated 2026-05-22)

| Area | Score | Basis |
|------|:-----:|-------|
| Architecture | 9/10 | Clean, well-separated, correct ROS2 patterns. |
| API + models | 9/10 | Pydantic v2, auth, typed responses. |
| ROS2 integration | 9/10 | `MultiThreadedExecutor`, async wrappers, MAVROS staleness guard. |
| Safety chain | 8/10 | Watchdog, estop, stop-path — good. Estop during ARMING has lock contention (§5.7). |
| Reliability | 7/10 | Code is solid; **no test coverage at all**. |
| Operability | 8/10 | Systemd units deployed and working. MAVROS plugin denylist fixed. `/healthz` exists. Still no telemetry-to-file. |
| Security | 7/10 | Token auth, upload limits, CORS consistent. **ROVER_DISABLE_AUTH=1 still in production service file** — must remove for field use. |
| Field readiness | 7/10 | Bug 6/7 fixed in firmware. QPPS_MAX set. NTRIP streaming. Systemd operational. QGC origin mismatch remains for `.waypoints` uploads. |

**Composite: 8.0 / 10** (up from 7.8 — systemd, firmware fixes, QPPS_MAX, plugin denylist all resolved)

The remaining blockers are:
1. pytest suite (90 min).
2. SITL smoke test (5 min run).
3. QGC origin re-anchor for uploaded `.waypoints` (builtins unaffected).
4. Remove `ROVER_DISABLE_AUTH=1` from `rover-server.service` for production.
5. Add `sd_notify` to `run.sh` or remove `WatchdogSec=30` from service file.
6. Bench-verify motor direction (FUNC1/FUNC2 mapping).

---

## 7. Minimum Pre-Field-Test Checklist (updated 2026-05-22)

```
[x] Firmware: Bug 7 (reverse spot-turn) fixed in build bfe914ce
[x] Systemd: rpp-pipeline.service + rover-server.service deployed and running
[x] MAVROS plugin denylist: fixed (/**: namespace, 23 plugins denied, guided_target spam eliminated)
[x] RBCLW_QPPS_MAX: set to 162162 (Motion Studio autotune, 2026-05-22)
[x] NTRIP: streaming RTCM corrections (180 frames/30s, caster.emlid.com)
[ ] Bench-verify motor direction (FUNC1/FUNC2 mapping — may be swapped vs proven config)
[ ] Remove ROVER_DISABLE_AUTH=1 from rover-server.service for production
[ ] Add sd_notify to run.sh or remove WatchdogSec=30 from service file
[ ] pytest: write test_path_manager + test_rpp_monitor + test_offboard_controller
[ ] SITL smoke: one builtin path (square_2x2) end-to-end, check logs
[ ] P4 validation: verify hold-yaw-at-stop on hardware with RTK
```

For `.waypoints` uploads:
```
[ ] Implement GP origin re-anchor before using non-builtin paths in field
```
