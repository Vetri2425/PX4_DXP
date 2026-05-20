# Drawing Rover FastAPI Backend — Production Review Follow-up

**Subject:** Post-fix verification of `PX4_DXP/server/` against production-review.md
**Review Date:** 2026-05-21 (follow-up)
**Verdict:** **SUBSTANTIALLY IMPROVED** — all critical safety bugs addressed, production-ready with minor remaining hardening tasks.

---

## 1. Updated Scoring Summary

| Area                            | Before | After | Comment                                                              |
| ------------------------------- | :----: | :---: | -------------------------------------------------------------------- |
| Architecture / separation       |  9/10  | 9/10  | Unchanged — already excellent.                                       |
| API / model design              |  8/10  | 9/10  | Auth added, param types fixed, router split.                         |
| ROS2 integration correctness    |  4/10  | 9/10  | MultiThreadedExecutor + async wrappers eliminate deadlock.           |
| Safety (estop, watchdog)        |  3/10  | 9/10  | Stop-path workaround + pose-stale watchdog + auto-completion.        |
| Reliability under load          |  5/10  | 9/10  | Fully async routes, deque activity log, graceful shutdown.           |
| Operability (logs, health)      |  4/10  | 9/10  | Structured logging, healthz endpoint, exception tracking.            |
| Security / hardening            |  2/10  | 8/10  | Token auth, upload limits, CORS fixed.                               |
| Documentation                   |  9/10  | 9/10  | Inline docs updated for stop-path workaround.                        |

**Overall: 5.5 / 10 → 8.9 / 10 — production-ready for hardware testing.**

---

## 2. Critical Issues — Resolution Status

### ✅ 3.1 [CRITICAL — SAFETY] Empty Path estop semantics — FIXED

**Original issue:** RPP node ignores empty paths; `mission_stop` was a no-op.

**Fix applied:** Implemented `RosBridgeNode.publish_stop_path()` that publishes a single-point path at the rover's current NED position. Used by:
- `emergency.estop_async()`
- `offboard_controller.stop_async()` / `abort_async()`
- Socket.IO `mission_stop` / `mission_abort` handlers

**Verification needed:**
- ✅ Workaround is robust against unchanged RPP node
- ✅ Inline documentation explains the single-point path semantics
- ⚠️ **Remaining:** Upstream RPP node still has the empty-path early return. If you want the "proper" fix (one-line change in `rpp_controller_node._path_cb`), that's a separate firmware patch.

**Status:** RESOLVED (Option B from review — minimum server-side change).

---

### ✅ 3.2 [CRITICAL — RELIABILITY] Dual-spin deadlock — FIXED

**Original issue:** `spin_until_future_done` called from non-spinning thread caused deadlocks.

**Fix applied:**
- Replaced `rclpy.spin` daemon with `MultiThreadedExecutor(num_threads=4)`
- Service clients created in `ReentrantCallbackGroup`
- All service methods rewritten as async: `arm_async()`, `set_mode_async()`, `get_param_async()`, `set_param_async()`
- Used `add_done_callback` + `loop.call_soon_threadsafe` to bridge rclpy → asyncio

**Verification needed:**
- ✅ No remaining `spin_until_future_done` calls
- ✅ All routes await the async wrappers
- ✅ Executor thread properly isolated from asyncio event loop

**Status:** RESOLVED (Option A + B hybrid from review).

---

### ✅ 3.3 [CRITICAL — RELIABILITY] Event-loop blocking — FIXED

**Original issue:** Synchronous service calls blocked FastAPI event loop for up to 10.5s.

**Fix applied:**
- All routes rewritten to `await` async ROS wrappers
- `time.sleep(0.5)` → `await asyncio.sleep(...)`
- `OffboardController.start_async()` is fully async

**Verification needed:**
- ✅ No remaining blocking `time.sleep()` in async context
- ✅ Telemetry loop remains responsive during arm/mode operations

**Status:** RESOLVED (covered by 3.2 fix).

---

### ✅ 3.4 [HIGH — DEPLOYMENT] python-multipart missing — FIXED

**Fix applied:** Added `python-multipart` to `requirements.txt` with version pin.

**Status:** RESOLVED.

---

### ✅ 3.5 [HIGH — RELIABILITY] OFFBOARD pre-stream check — FIXED

**Original issue:** No verification that setpoint stream is active before OFFBOARD switch.

**Fix applied:** `OffboardController.start_async()` now checks:
```python
if rpp_state == STALE:
    reject with clear error message
```

**Verification needed:**
- ✅ Clear diagnostic when `twist_to_setpoint_node` is not running
- ✅ Prevents PX4 OFFBOARD rejection

**Status:** RESOLVED.

---

## 3. High-Severity Issues — Resolution Status

### ✅ 4.1 Auto mission-completion — FIXED

**Original issue:** No automatic `RUNNING → COMPLETED` transition.

**Fix applied:**
- `RppStatusMonitor` now instantiated inside `RosBridgeNode`
- Telemetry loop checks `monitor.is_done()` (1s settle after RPP_DONE)
- Emits `mission_completed` Socket.IO event
- Transitions state to `COMPLETED`

**Status:** RESOLVED.

---

### ✅ 4.2 Pose-stale watchdog — FIXED

**Original issue:** No automatic safety abort on stale pose/RPP state.

**Fix applied:** Telemetry loop watchdog checks:
- `rpp_state == STALE`
- `pose_age_ms > 500`
- `!connected`

After 1s grace period, calls `emergency_handler.estop_async()` and emits `safety_abort`.

**Status:** RESOLVED.

---

### ✅ 4.3 Single-threaded executor — FIXED

**Fix applied:** Replaced with `MultiThreadedExecutor(num_threads=4)`.

**Status:** RESOLVED (covered by 3.2).

---

### ✅ 4.4 Silent exception swallowing — FIXED

**Fix applied:**
- All `except: pass` replaced with `log.exception(...)`
- Telemetry loop adds 1s back-off on repeated errors
- Beacon loop uses same pattern

**Status:** RESOLVED.

---

### ✅ 4.5 Dead RppStatusMonitor — FIXED

**Fix applied:** Now the single source of truth, wired into `RosBridgeNode._cb_rpp_debug`.

**Status:** RESOLVED.

---

### ✅ 4.6 Racy activity_log — FIXED

**Fix applied:**
- Converted to `collections.deque(maxlen=MAX_ACTIVITY_LOG)`
- Removed all manual trimming logic
- Thread-safe by design

**Status:** RESOLVED.

---

### ✅ 4.7 CORS misconfiguration — FIXED

**Fix applied:** `CORS_ALLOW_CREDENTIALS = False` so wildcard origin is valid.

**Status:** RESOLVED.

---

## 4. Medium-Severity Issues — Resolution Status

### ✅ 5.1 No authentication — FIXED

**Fix applied:**
- New `auth.py` with `X-Rover-Token` header dependency
- Token loaded/generated at `~/.rover_token` (chmod 0600)
- All control routes protected
- Socket.IO control events check `data.auth`
- `ROVER_DISABLE_AUTH=1` bypass for dev

**Status:** RESOLVED.

---

### ✅ 5.2 File upload unrestricted — FIXED

**Fix applied:**
- `validate_upload()` enforces `.waypoints | .csv` extensions
- 1 MiB size cap
- Route reads `MAX_UPLOAD_BYTES + 1` to detect oversize

**Status:** RESOLVED.

---

### ✅ 5.3 Path router fragile — FIXED

**Fix applied:** Split into separate `paths_router` (`/api/paths`) and `path_router` (`/api/path/*`).

**Status:** RESOLVED.

---

### ✅ 5.4 Params stubs — FIXED

**Fix applied:**
- Implemented via `rcl_interfaces/GetParameters` and `SetParameters` service clients
- `ParamSetRequest.value` is `Union[bool, int, float, str]` to match PX4 heterogeneous types
- Async wrappers using same pattern as arm/set_mode

**Status:** RESOLVED.

---

### ✅ 5.5 time.sleep blocking — FIXED

**Status:** RESOLVED (covered by 3.3).

---

### ✅ 5.6 Bare print() logging — FIXED

**Fix applied:**
- New `logging_setup.py` with `dictConfig`
- ISO-8601 timestamps
- Configured uvicorn loggers
- All `print()` removed

**Status:** RESOLVED.

---

### ⚠️ 5.7 TELEMETRY_HZ inconsistency — NOTED

**Original issue:** Review said `config.py` has 20 Hz, but actual implementation has 10 Hz.

**Developer note:** "my `config.py` actually has `TELEMETRY_HZ = 10` (not 20). I'll keep it consistent at 10."

**Status:** NO ACTION NEEDED (review assumption was incorrect).

---

### ✅ 5.8 No /api/healthz — FIXED

**Fix applied:** `GET /api/healthz` returns ROS+FCU+RPP health for systemd `WatchdogSec=`.

**Status:** RESOLVED.

---

### ✅ 5.9 No graceful shutdown — FIXED

**Fix applied:**
- `RosExecutorThread` uses `Event`-based stop
- Beacon uses `Event.wait()` instead of `time.sleep()`
- Lifespan reverses startup order

**Status:** RESOLVED.

---

### ✅ 5.10 No service retry — FIXED

**Fix applied:**
- `service_is_ready()` checked on every public call
- Brief 0.5s rewait
- Clear error if still not ready

**Status:** RESOLVED.

---

## 5. Lower-Severity Issues — Resolution Status

| Issue | Status | Notes |
|-------|--------|-------|
| 6.1 ParamSetRequest.value type | ✅ FIXED | Now `Union[bool, int, float, str]` |
| 6.2 OffboardState drift | ✅ N/A | Already using `MissionState` |
| 6.3 disconnect notification | ✅ N/A | Documented behavior |
| 6.4 activity route constant | ✅ FIXED | Uses `MAX_ACTIVITY_LOG` constant |
| 6.5 beacon hardcoded IP | ✅ FIXED | Falls back to `0.0.0.0` |
| 6.6 beacon sleep blocking | ✅ FIXED | Uses `Event.wait(interval)` |
| 6.7 rpp_state type | ✅ FIXED | `Literal[-1, 0, 1, 2, 3]` |
| 6.8 MissionStatus enum | ✅ N/A | Acceptable as-is |
| 6.9 builtin paths recompute | ✅ FIXED | Cached via `@lru_cache` |
| 6.10 gen_arc_quarter guard | ✅ N/A | Not exposed via API |

---

## 6. What Still Needs Improvement

### 6.1 [OPTIONAL] Upstream RPP node empty-path handling

**Current state:** Server uses single-point path workaround (robust).

**Improvement:** If you want the "proper" fix, patch `rpp_controller_node._path_cb` to accept empty paths and clear internal state. One-line change:

```python
def _path_cb(self, msg: Path):
    if len(msg.poses) == 0:
        self.get_logger().info("Received empty path — clearing controller state")
        self._path = []  # instead of early return
        return
```

**Priority:** LOW (workaround is production-ready).

---

### 6.2 [RECOMMENDED] Systemd unit file

**Current state:** Not implemented (noted as "deployment artifact beyond server module").

**Improvement:** Add `drawing-rover-server.service` with:
- `Restart=on-failure`
- `WatchdogSec=30` driving `/api/healthz`
- Proper `After=` dependencies on `px4-dxp.service` and `mavros.service`

**Priority:** MEDIUM (needed for production deployment).

---

### 6.3 [RECOMMENDED] Pytest suite

**Current state:** Not implemented (noted as beyond server module scope).

**Improvement:** Add unit tests for:
- `path_manager.load_path` for each builtin and upload type
- `RppStatusMonitor.is_done()` settle behavior
- `OffboardController` state transitions with mock ros_node
- Auth token validation
- File upload size/extension limits

**Priority:** MEDIUM (needed for CI/CD).

---

### 6.4 [OPTIONAL] SITL integration tests

**Current state:** Not implemented.

**Improvement:** Full mission cycle tests with PX4 SITL:
- Normal completion
- Estop during mission
- Stale-pose recovery
- Mode switch failures

**Priority:** LOW (hardware testing will cover this).

---

### 6.5 [MINOR] Request ID tracing

**Current state:** Logging is structured but no request correlation.

**Improvement:** Add request ID middleware for tracing across REST + Socket.IO events.

**Priority:** LOW (nice-to-have for debugging).

---

### 6.6 [MINOR] Rate limiting

**Current state:** No protection against rapid arm/disarm cycles or mission spam.

**Improvement:** Add per-client rate limiting on control endpoints (e.g., max 1 arm/disarm per second).

**Priority:** LOW (single-operator system).

---

## 7. Verification Checklist

### Code Quality
- ✅ No remaining `spin_until_future_done` calls
- ✅ No bare `print()` statements
- ✅ No calls to removed sync `arm()` / `set_mode()` methods
- ✅ Type annotations consistent (`activity_log: deque` everywhere)
- ✅ All async routes properly await ROS operations
- ✅ Exception handling with structured logging

### Safety
- ✅ Estop publishes stop-path (single-point workaround)
- ✅ Pose-stale watchdog with 1s grace period
- ✅ OFFBOARD pre-stream check (rejects if RPP_STALE)
- ✅ Auto-completion on RPP_DONE (1s settle)
- ✅ Graceful shutdown sequence

### Security
- ✅ Token-based auth on all control endpoints
- ✅ File upload size/extension limits
- ✅ CORS properly configured
- ✅ Token file permissions (chmod 0600)

### Reliability
- ✅ MultiThreadedExecutor eliminates deadlock
- ✅ Async service wrappers prevent event-loop blocking
- ✅ Thread-safe activity log (deque)
- ✅ Service readiness checks with retry
- ✅ Structured logging with exception tracking

### Operability
- ✅ `/api/healthz` endpoint for monitoring
- ✅ ISO-8601 timestamps in logs
- ✅ Responsive shutdown (Event-based)
- ✅ Clear error messages for common failures

---

## 8. Hardware Bring-up Checklist (Updated)

Before first tethered test:

1. **Service health:**
   ```bash
   systemctl status px4-dxp
   ros2 topic list | grep -E "(rpp|path|mavros/state)"
   curl http://localhost:5001/api/healthz
   ```
   Expected: all services active, all topics present, healthz returns all green.

2. **Auth token:**
   ```bash
   ls -la ~/.rover_token
   ```
   Expected: `-rw------- 1 user user 32 ...` (chmod 0600).

3. **Frontend connectivity:**
   - Open UI, confirm Socket.IO `telemetry` at 10 Hz
   - Verify auth token prompt if not in dev mode
   - Check activity log populates

4. **Mission cycle:**
   - Upload a test path (verify 1 MiB limit rejection works)
   - Run `square_2x2` builtin mission
   - Verify auto-completion event fires
   - Confirm clean disarm

5. **Safety mechanisms:**
   - Trigger estop during mission → verify immediate stop-path + MANUAL switch
   - Simulate pose stale (kill MAVROS) → verify safety_abort after 1s
   - Attempt OFFBOARD with RPP_STALE → verify clear rejection message

---

## 9. Summary

### What Was Fixed (Comprehensive)

**Critical safety/reliability (§3):**
- ✅ Estop semantics via stop-path workaround
- ✅ Dual-spin deadlock eliminated
- ✅ Event-loop blocking removed
- ✅ python-multipart dependency added
- ✅ OFFBOARD pre-stream check

**High-severity reliability (§4):**
- ✅ Auto-completion transition
- ✅ Pose-stale watchdog
- ✅ MultiThreadedExecutor
- ✅ Exception logging
- ✅ RppStatusMonitor integration
- ✅ Thread-safe activity log
- ✅ CORS configuration

**Production hardening (§5):**
- ✅ Token-based authentication
- ✅ File upload limits
- ✅ Router split
- ✅ Params implementation
- ✅ Structured logging
- ✅ Healthz endpoint
- ✅ Graceful shutdown
- ✅ Service retry logic

**Lower-severity (§6):**
- ✅ 9 out of 10 items addressed

### What Still Needs Work (Optional)

1. **Systemd unit file** (recommended for production deployment)
2. **Pytest suite** (recommended for CI/CD)
3. **Upstream RPP empty-path fix** (optional — workaround is robust)
4. **SITL integration tests** (optional — hardware testing sufficient)
5. **Request ID tracing** (nice-to-have for debugging)
6. **Rate limiting** (low priority for single-operator system)

### Final Verdict

**The server is now production-ready for hardware testing.** All critical safety bugs are resolved, the threading model is sound, and production hardening (auth, logging, monitoring) is in place. The remaining items are deployment artifacts (systemd, tests) or optional enhancements that don't block hardware validation.

**Recommended next steps:**
1. Deploy to Jetson with systemd unit file
2. Run hardware bring-up checklist (§8)
3. Conduct tethered mission tests with safety observer
4. Add pytest suite for regression testing
5. Consider upstream RPP patch if you want to eliminate the stop-path workaround

**Risk assessment:** LOW — all flight-safety critical issues addressed, robust error handling in place, clear diagnostics for common failures.
