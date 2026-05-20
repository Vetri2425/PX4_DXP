# Drawing Rover FastAPI Backend — Production Readiness Review

**Subject:** `PX4_DXP/server/`
**Reviewer:** Code review against `ARCHITECTURE.md` + actual implementation in
`server/*.py` and the live RPP pipeline in `PX4_DXP/src/rpp_controller_node.py`.
**Date:** 2026-05-21
**Verdict:** Sound architecture, but **NOT production ready**. Two critical bugs
(one safety-critical) and several reliability gaps must be fixed before any
hardware mission. See section 4 for a prioritised fix list.

---

## 1. Scoring Summary

| Area                            | Score  | Comment                                                              |
| ------------------------------- | :----: | -------------------------------------------------------------------- |
| Architecture / separation       |  9/10  | Clean module split, single rclpy node, no circular deps.             |
| API / model design              |  8/10  | Pydantic v2, typed responses, REST + Socket.IO split is correct.     |
| ROS2 integration correctness    |  4/10  | Threading model is unsafe; service calls can deadlock the event loop.|
| Safety (estop, watchdog)        |  3/10  | Documented estop path is **broken**; no pose-stale watchdog.         |
| Reliability under load          |  5/10  | Single-thread executor + blocking calls in async handlers.           |
| Operability (logs, health)      |  4/10  | `print()` only; silent `except: pass`; no `/api/healthz`.            |
| Security / hardening            |  2/10  | No auth, CORS misconfigured, file upload unrestricted.               |
| Documentation                   |  9/10  | ARCHITECTURE.md is excellent and up to date.                         |

**Overall: 5.5 / 10 — solid skeleton, two critical flight-safety bugs.**

---

## 2. What is Good

### 2.1 Architecture
- Single rclpy node (`RosBridgeNode`) owns all subscriptions, the `/path`
  publisher, and service clients. Routes never touch ROS directly. This is
  the right pattern for a FastAPI + rclpy hybrid.
- Lifespan handler uses the modern `@asynccontextmanager` form (not the
  deprecated `@app.on_event`). Good.
- Background daemon thread for `rclpy.spin` is the standard approach.
- Module boundaries are clean: `path_manager`, `offboard_controller`,
  `emergency`, `beacon`, `rpp_status` are all single-responsibility.
- ENU → NED conversion is implemented correctly and matches
  `rpp_controller_node._enu_pose_to_ned` exactly (verified line by line).

### 2.2 Implementation
- QoS profiles match upstream contracts:
  - `/path` → `RELIABLE + TRANSIENT_LOCAL` (matches `path_publisher_node`).
  - `/mavros/state` → `RELIABLE + TRANSIENT_LOCAL`.
  - `/rpp/debug`, pose, battery → `BEST_EFFORT + VOLATILE`.
- Thread-safe shared state in `RosBridgeNode._state` via `threading.Lock`
  with shallow-copy reads. No torn reads possible.
- `_cb_battery` correctly normalises the 0..1 vs 0..100 ambiguity in
  MAVROS `BatteryState.percentage`.
- `set_mode` correctly checks `result.mode_sent` (the architecture text in
  ARCHITECTURE.md says `success`, which is wrong; the code is right).
- `path_manager.save_uploaded` uses `os.path.basename(filename)` to strip
  directory traversal. Good.
- Pydantic v2 models use `from __future__ import annotations` and modern
  `X | None` syntax. Consistent.
- `_log()` helper trims `activity_log` to `MAX_ACTIVITY_LOG`. No unbounded
  growth.
- Beacon socket uses `settimeout(1.0)` so `stop()` is responsive.

### 2.3 Documentation
- `ARCHITECTURE.md` is detailed, accurate on contracts (RPP debug array
  format, OFFBOARD pre-stream requirement, ENU→NED math), and explicitly
  flags MVP scope cuts.
- Topic and service names are centralised in `config.py`.

---

## 3. Critical Issues (must fix before any flight test)

### 3.1 [CRITICAL — SAFETY] Empty `Path` does NOT make RPP go IDLE

**Where:** `emergency.py:estop()`, `offboard_controller.stop()`, `.abort()`,
`sockets/events.py:on_mission_stop`, `on_mission_abort`.

**Documented behaviour (ARCHITECTURE.md §4, §10):**
> Emergency stop = switch to MANUAL. Also publishes empty Path to make
> RPP go IDLE.

**Actual behaviour:** `rpp_controller_node._path_cb` (line 183):
```python
def _path_cb(self, msg: Path):
    if len(msg.poses) == 0:
        self.get_logger().warn("Received empty path — ignoring")
        return
```

The RPP node **early-returns and ignores empty paths entirely**. The previously
loaded path is retained, and the controller continues publishing velocity
toward the last waypoint until `_path_done`. Stop / abort / estop in the
server therefore have **no effect on RPP's internal state**.

**Why it still mostly works today:** the second leg of estop (`set_mode
MANUAL` + disarm) does stop the motors via PX4. So the rover halts. But:
- `mission_stop` (which deliberately stays armed) is a **no-op** — RPP keeps
  driving.
- If the OFFBOARD-MANUAL switch fails for any reason (FCU rejected,
  service timeout), RPP keeps driving.
- The system has no graceful "cancel mission" state.

**Fix (pick one — both are valid):**

1. **Change RPP** (preferred): make `_path_cb` accept empty path and clear
   `_path` so the controller publishes zero velocity until a new path
   arrives. One-line change in `rpp_controller_node.py`.

2. **Change server**: replace empty-path with publishing a single-point path
   at the rover's current NED position. RPP will treat that as DONE
   immediately and zero its output. Robust against the unchanged RPP node.

Until this is fixed, **`mission_stop` is unsafe** — the architecture
description is a lie about its behaviour.

---

### 3.2 [CRITICAL — RELIABILITY] `rclpy.spin_until_future_done` called from non-spinning thread

**Where:** `ros_node.RosBridgeNode.arm()` and `.set_mode()`.

```python
def arm(self, arm: bool, timeout: float = 5.0) -> bool:
    ...
    future = self._arming_cli.call_async(req)
    rclpy.spin_until_future_done(self, future, timeout_sec=timeout)
```

**Problem:** the rclpy node is already being spun by the daemon thread
started in `main.lifespan` (`threading.Thread(target=rclpy.spin, ...)`).
Calling `rclpy.spin_until_future_done(self, ...)` on the **same node** from
a different thread is undefined behaviour. Symptoms in production:
- The daemon thread eats the future before `spin_until_future_done` sees it,
  so the call hangs for the full timeout and returns `None`.
- Or both threads race on the executor and rclpy raises
  `rclpy._rclpy_pybind11.RCLError` (sometimes, intermittent).
- Worst case: deadlock — the daemon spin holds the executor lock while
  this thread waits for it.

**Fix:** Replace the dual-spin pattern with one of:

```python
# Option A: future.add_done_callback + asyncio
async def arm_async(self, arm: bool, timeout: float = 5.0) -> bool:
    if self._arming_cli is None:
        return False
    req = CommandBool.Request()
    req.value = arm
    future = self._arming_cli.call_async(req)
    loop = asyncio.get_running_loop()
    fut = loop.create_future()
    future.add_done_callback(
        lambda f: loop.call_soon_threadsafe(fut.set_result, f.result())
    )
    try:
        result = await asyncio.wait_for(fut, timeout=timeout)
    except asyncio.TimeoutError:
        return False
    return result is not None and result.success
```

```python
# Option B: MultiThreadedExecutor + ReentrantCallbackGroup
# - Create the service clients in a ReentrantCallbackGroup
# - Spin with MultiThreadedExecutor (num_threads >= 2) instead of rclpy.spin
# - Service callbacks run on a separate executor thread, so call_async +
#   future.result(timeout=...) is safe
```

Either fixes the threading bug. Option A is recommended because it also
solves issue 3.3.

---

### 3.3 [CRITICAL — RELIABILITY] Service calls block the FastAPI event loop

**Where:** `routes/vehicle.py`, `routes/mission.py`, `sockets/events.py`,
`offboard_controller.start()`.

```python
@router.post("/arm", response_model=ArmResponse)
async def arm_vehicle(req: ArmRequest):
    ...
    ok = ros_node.arm(req.arm)   # blocks for up to 5 seconds
```

The route is `async def`, which means it runs **on the asyncio event loop**.
`ros_node.arm()` is a synchronous, blocking call (up to 5 s). While
`arm()` is in flight:
- Telemetry push loop stops.
- Socket.IO heartbeats stop — clients may disconnect.
- All other HTTP requests queue.
- A second `arm` request from the UI arrives → blocks the event loop for
  another 5 s.

`offboard_controller.start()` is even worse: it does `arm(5s) +
time.sleep(0.5) + set_mode(5s)` synchronously = up to **10.5 seconds** of
event-loop blocking per mission start.

**Fix:** Wrap every service call in `await asyncio.to_thread(...)`:

```python
@router.post("/arm")
async def arm_vehicle(req: ArmRequest):
    ok = await asyncio.to_thread(ros_node.arm, req.arm)
    ...
```

If 3.2 is fixed with Option A, the methods themselves become async and this
section also resolves.

---

### 3.4 [HIGH — DEPLOYMENT] `python-multipart` missing from `requirements.txt`

**Where:** `requirements.txt`, used by `routes/path.py:upload_path`.

FastAPI's `UploadFile = File(...)` requires `python-multipart` at runtime.
It is not installed by `fastapi` itself. The first time a user uploads a
mission file, the server will return 500 with:

```
RuntimeError: Form data requires "python-multipart" to be installed.
```

**Fix:** add to `requirements.txt`:
```
python-multipart>=0.0.9
```

---

### 3.5 [HIGH — RELIABILITY] OFFBOARD pre-stream check missing

**Where:** `offboard_controller.start()`.

`ARCHITECTURE.md §3` correctly states:
> The server must verify streaming is active before attempting OFFBOARD
> switch by checking `/rpp/debug[7]` is not STALE (-1).

The implementation does not do this. It just `time.sleep(0.5)` and hopes.
If `twist_to_setpoint_node` is not running (service not started, crashed,
ROS bag replay paused), PX4 rejects the OFFBOARD switch, the controller
correctly disarms and reports ERROR, but the user has no diagnostic.

**Fix in `start()`:**
```python
state = self._node.get_state()
if state.get("rpp_state", -1) == RPP_STALE:
    self._state = MissionState.ERROR
    self._log_entry("error", "Cannot start: setpoint stream is STALE — "
                              "is twist_to_setpoint_node running?")
    return False
```

Also, for a full check, verify that `/mavros/state.connected == True`
(already done) AND that `pose_age_ms` is fresh (< 200 ms).

---

## 4. High-Severity Issues

### 4.1 No automatic mission-completion transition

**Where:** `offboard_controller.py`.

The state machine has `RUNNING → STOPPING → IDLE` but no automatic
transition to `COMPLETED` when RPP reports DONE. The frontend must poll
`/api/mission/status` and infer completion. This is fragile and contradicts
the documented state machine.

**Fix:** in the 10 Hz telemetry loop in `main._telemetry_loop`, when:
- `offboard_ctrl.state == RUNNING`, and
- `s["rpp_state"] == RPP_DONE` for ≥ 1 s (use `RppStatusMonitor.is_done()`,
  which is currently dead code — see 4.5),

then transition `offboard_ctrl.state = MissionState.COMPLETED` and emit a
`mission_completed` Socket.IO event. Optionally auto-disarm if a config
flag is set.

---

### 4.2 No pose-stale watchdog → no automatic safety abort

**Where:** absent from the code.

For a vehicle that can move autonomously, a stale pose (RTK lost, MAVROS
crashed) must trigger an automatic abort. Today, RPP itself publishes
zero velocity on `STALE`, but:
- The server does not detect or react to STALE.
- If the stale persists, the rover sits in OFFBOARD mode armed with no path
  control until the operator notices.

**Fix:** add a watchdog tick inside the telemetry loop:
```python
if state.value == RUNNING and s.get("rpp_state") == RPP_STALE:
    if stale_since is None:
        stale_since = now
    elif now - stale_since > 1.0:
        emergency_handler.estop()
        await sio.emit("safety_abort",
                        {"reason": "pose stale > 1s"})
else:
    stale_since = None
```

Also covers `pose_age_ms > 500` and `connected == False`.

---

### 4.3 Single-threaded rclpy executor

**Where:** `main.lifespan` — `threading.Thread(target=rclpy.spin, args=(ros_node,))`.

`rclpy.spin(node)` uses a `SingleThreadedExecutor` by default. Subscription
callbacks AND service result callbacks all run on one thread. If one
callback is slow (e.g. `Path` publish during high-rate telemetry), all
others block.

**Fix:**
```python
from rclpy.executors import MultiThreadedExecutor
exe = MultiThreadedExecutor(num_threads=4)
exe.add_node(ros_node)
threading.Thread(target=exe.spin, daemon=True, name="rclpy-spin").start()
```

Also create the service clients in a `ReentrantCallbackGroup` so they can
be invoked from any thread without deadlock.

---

### 4.4 Silent exception swallowing in telemetry loop

**Where:** `main._telemetry_loop`.

```python
except Exception:
    pass  # never let the telemetry loop die silently
```

The intent is correct, the implementation is wrong: bugs and ROS errors
disappear forever and the operator has no idea why telemetry stopped
updating.

**Fix:**
```python
import logging
log = logging.getLogger("server.telemetry")
...
except asyncio.CancelledError:
    raise
except Exception as exc:
    log.exception("telemetry loop iteration failed: %s", exc)
    await asyncio.sleep(1.0)  # back off on repeated errors
```

Same pattern in `beacon._loop` (currently `except Exception: pass`).

---

### 4.5 Dead code: `RppStatusMonitor` defined, never instantiated

**Where:** `rpp_status.py` is imported nowhere.

The decoder logic is fine, but it duplicates `_cb_rpp_debug` in
`ros_node.py`. Pick one source of truth:
- Either inject `RppStatusMonitor` into `RosBridgeNode` and call
  `monitor.update(msg.data)` from `_cb_rpp_debug`, then use
  `monitor.is_done()` in the watchdog, **or**
- Delete `rpp_status.py` and inline `RPP_STATE_NAMES` lookup (already in
  `config.py`).

Recommendation: **keep `rpp_status.py`** and use it for `is_done()` /
`is_tracking()` in the mission state machine (resolves 4.1). It is the
right abstraction.

---

### 4.6 `activity_log` is racy

**Where:** `main.activity_log`, `routes/vehicle.py`, `sockets/events.py`,
`offboard_controller._log_entry`, `emergency.estop`.

The plain `list` is mutated from at least three thread/loop contexts:
- FastAPI thread pool (route handlers).
- asyncio event loop (telemetry loop, Socket.IO).
- rclpy daemon (`OffboardController._log_entry` is called from
  `start()/stop()/abort()` which originate in route handlers — same thread,
  but the helper trims to `MAX_ACTIVITY_LOG` non-atomically).

`list.append` is GIL-protected (atomic) but `del self._log[: len-cap]` is
**not atomic with concurrent appends**. Two contributors trimming
simultaneously can corrupt indices.

**Fix:** wrap with a lock or use `collections.deque(maxlen=MAX_ACTIVITY_LOG)`,
which is thread-safe and self-trimming. Recommended:

```python
from collections import deque
activity_log = deque(maxlen=MAX_ACTIVITY_LOG)
```
Then drop every `if len > MAX: del ...` block from the codebase.

---

### 4.7 CORS misconfiguration

**Where:** `main.create_app`.

```python
allow_origins=["*"],
allow_credentials=True,
```

Per CORS spec, `Access-Control-Allow-Origin: *` and
`Access-Control-Allow-Credentials: true` together are invalid; browsers
reject the response. Either:
```python
allow_origins=["http://localhost:5173", "http://192.168.1.0/24"],
allow_credentials=True,
```
or:
```python
allow_origins=["*"],
allow_credentials=False,
```
For a LAN-only deployment, the second form is fine.

---

## 5. Medium-Severity Issues

### 5.1 No authentication

This is a safety-critical autonomous vehicle endpoint. Anyone on the LAN
can:
```
curl -X POST http://192.168.1.102:5001/api/arm -d '{"arm":true}'
curl -X POST http://192.168.1.102:5001/api/mission/start
```

**Fix:** at minimum, require a shared-secret header (e.g.
`X-Rover-Token`) for all `/api/arm`, `/api/set_mode`, `/api/mission/*`,
`/api/path/*`, `/api/params/*` and Socket.IO control events. Validate via
a single FastAPI dependency. A 16-byte token in `~/.rover_token` is
sufficient for first iteration.

### 5.2 File upload is unrestricted

`routes/path.py:upload_path`:
- No size limit. A 100 MB file will be loaded into memory.
- No content type check.
- No filename suffix whitelist.
- Overwrites silently.

**Fix:**
```python
ALLOWED = {".waypoints", ".csv"}
MAX_BYTES = 1 * 1024 * 1024  # 1 MB

async def upload_path(file: UploadFile = File(...)):
    if Path(file.filename).suffix.lower() not in ALLOWED:
        raise HTTPException(415, "Only .waypoints / .csv allowed")
    content = await file.read(MAX_BYTES + 1)
    if len(content) > MAX_BYTES:
        raise HTTPException(413, "File too large")
    ...
```

### 5.3 `routes/path.py` route declaration is fragile

```python
router = APIRouter(prefix="/path")
@router.get("s")            # → /api/path + s = /api/paths   (works by accident)
async def list_paths(): ...
```

This relies on FastAPI's prefix concatenation. Trivially breaks if anyone
edits the prefix. **Fix:** declare two routers:

```python
paths_router = APIRouter(prefix="/paths", tags=["path"])
path_router  = APIRouter(prefix="/path", tags=["path"])
# /api/paths   → list_paths under paths_router
# /api/path/*  → upload, publish, delete under path_router
```

Then mount both in `main.create_app`.

### 5.4 `params.py` is mostly stubs that 501

The whole router lies about its capabilities and returns a 501 page after
the user fills the form. Either implement `GetParameters` /
`SetParameters` service clients (≈30 lines) or remove the router and the
`/api/params` link from the frontend.

For implementation, add to `RosBridgeNode`:
```python
from rcl_interfaces.srv import GetParameters, SetParameters
self._param_get_cli = self.create_client(
    GetParameters, "/mavros/param/get_parameters")
self._param_set_cli = self.create_client(
    SetParameters, "/mavros/param/set_parameters")
```
and async wrappers using the same `add_done_callback` trick from 3.2.

### 5.5 `time.sleep(0.5)` inside `start()` blocks the event loop

Same root cause as 3.3. Use `await asyncio.sleep(0.5)` after making
`start()` async.

### 5.6 Bare `print()` for logging

`main._log` does `print(f"[{level}] {msg}")`. Production should:
- use `logging` with a configured handler (file rotate via
  `RotatingFileHandler`, journald via `systemd.journal.JournalHandler`, or
  `uvicorn`'s configured logger),
- include timestamps in ISO-8601,
- add a request ID for traceability across REST + Socket.IO.

Recommended baseline:
```python
logging.config.dictConfig({...})
log = logging.getLogger("server")
```

### 5.7 `TELEMETRY_HZ = 20` in `config.py` but `ARCHITECTURE.md` says 10 Hz

Inconsistency. 20 Hz is fine on a Jetson Orin, but it doubles outgoing
Socket.IO traffic. Pick one and update both.

### 5.8 No `/api/healthz`

Production deployments need a cheap liveness probe distinct from `/api/ping`.
Add:
```python
@router.get("/healthz")
async def healthz():
    from main import ros_node
    s = ros_node.get_state() if ros_node else {}
    return {
        "ros_node":      ros_node is not None,
        "fcu_connected": s.get("connected", False),
        "rpp_state":     s.get("rpp_state"),
        "pose_age_ms":   s.get("pose_age_ms"),
    }
```
Used by `systemd` `Watchdog=` or external monitors.

### 5.9 No graceful shutdown on rclpy thread

`threading.Thread(target=rclpy.spin, daemon=True)` is convenient but does
not flush publishers / unregister subscriptions. A long-lived publisher
losing its `TRANSIENT_LOCAL` snapshot is mostly harmless, but on Jetson
restarts you can see ROS2 graph entries lingering for ~10 s. Replace with:
```python
self._stop = threading.Event()
def _spin():
    while rclpy.ok() and not self._stop.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
```

### 5.10 No retry on transient MAVROS service unavailability

`wait_for_service(timeout_sec=5.0)` runs once at startup. If MAVROS comes
up after the FastAPI server (boot order race on Jetson), the service
clients are bound but `arm()` may silently fail later. Not catastrophic
because `call_async` returns a future that errors out, but operators
get a confusing "FAILED" message with no diagnostic.

**Fix:** check `cli.service_is_ready()` in each public method and refuse
with a clear message. Optionally retry once after 1 s.

---

## 6. Lower-Severity Issues

| #     | Issue                                                                                  | Fix                                                                  |
| ----- | -------------------------------------------------------------------------------------- | -------------------------------------------------------------------- |
| 6.1   | `ParamSetRequest.value: float` — PX4 has int params (`SYS_AUTOSTART`, etc.).           | Use `Union[int, float]` or two routes.                               |
| 6.2   | `MissionState` enum from `models.py` and `OffboardState` mentioned in spec drift.      | Spec already updated to `MissionState`; delete `OffboardState` mention. |
| 6.3   | `disconnect` notification fires only once per disconnect.                              | Acceptable; document it.                                              |
| 6.4   | `routes/system.py:activity` uses `[-500:]` instead of the configured `MAX_ACTIVITY_LOG`. | Use the constant.                                                  |
| 6.5   | `beacon.py:_get_local_ip` falls back to a hardcoded `192.168.1.102`.                   | Make configurable; on failure, broadcast `0.0.0.0` and let frontend skip. |
| 6.6   | `beacon.py:_loop` sleeps a full `interval` even after `stop()`.                        | Use `Event.wait(timeout)` instead of `time.sleep`.                   |
| 6.7   | No type for `TelemetryData.rpp_state` — should be `Literal[-1, 0, 1, 2, 3]`.           | Already fine; consider `Literal` for OpenAPI clarity.                 |
| 6.8   | `MissionStatus.state` field is the enum, but `mission_status` Socket.IO emits `.value`. | Document or use `model_dump(mode="json")` consistently.            |
| 6.9   | `path_manager.list_paths` recomputes builtin paths every call.                         | Cache after first call; trivial perf gain.                           |
| 6.10  | `gen_arc_quarter_1m5` divides by `(n - 1)` — fine for n ≥ 2, but if `radius=0` n=2 ok. | Add a guard if you ever expose radius via API.                        |

---

## 7. Crash Surface — What can take the server down

The primary failure modes that can stop telemetry / kill the process:

1. **rclpy executor deadlock** (3.2). Symptom: `/api/arm` hangs, telemetry
   loop seems alive but service calls never return. Recovery: restart
   process. **Must fix.**
2. **`python-multipart` missing** (3.4). Symptom: 500 on first upload.
   **Must fix before first frontend integration.**
3. **MAVROS not running at startup**. The server starts fine but `arm()`
   returns False forever. No crash, but no diagnostic either. (5.10)
4. **rclpy.init() called twice** if uvicorn `--reload` is used. Symptom:
   `RCLError: rcl_init() failed`. Mitigation: `if not rclpy.ok(): rclpy.init()`.
5. **Socket.IO emits when no clients connected** — never crashes, just
   wasteful.
6. **Path file with malformed UTF-8 or massive size** — `read_qgc_waypoints`
   reads the whole file with `open(filepath)`. No size guard. (5.2)
7. **Unhandled exception in a route** propagates to FastAPI's exception
   handler, returns 500. Acceptable. Add a global exception logger so
   stack traces hit the log.

---

## 8. Reliable Communication Checklist

| Layer                     | Status     | Notes                                                                         |
| ------------------------- | :--------: | ----------------------------------------------------------------------------- |
| `/path` QoS               | ✅ correct  | RELIABLE + TRANSIENT_LOCAL, matches RPP.                                      |
| `/mavros/state` QoS       | ✅ correct  | RELIABLE + TRANSIENT_LOCAL.                                                   |
| `/rpp/debug` QoS          | ✅ correct  | BEST_EFFORT, OK for 50 Hz diagnostic.                                         |
| ENU → NED conversion      | ✅ correct  | Matches `rpp_controller_node._enu_pose_to_ned` exactly.                       |
| Service-call timeouts     | ⚠️ broken   | See 3.2.                                                                      |
| Mode switch pre-check     | ❌ missing  | See 3.5.                                                                      |
| Stale-pose watchdog       | ❌ missing  | See 4.2.                                                                      |
| Empty-path stop semantics | ❌ broken   | See 3.1.                                                                      |
| Telemetry rate            | ⚠️ 20 Hz    | Spec says 10; Jetson can do 20 but document the choice.                       |
| Socket.IO heartbeat       | ✅ default  | python-socketio's default ping interval is 25s — fine.                        |
| Beacon discovery          | ✅ working  | UDP broadcast every 2 s on port 5002.                                         |

---

## 9. Recommended Fix Order (1–2 day plan)

**Day 1 — safety + reliability foundation:**

1. Fix 3.1 — empty-path semantics. Choose option B (single-point path) for
   minimum change to the firmware tree. ~15 lines in `emergency.py`,
   `offboard_controller.py`, `sockets/events.py`.
2. Fix 3.2 — rewrite `arm()`/`set_mode()` as `arm_async()`/`set_mode_async()`
   using `add_done_callback` + asyncio future. Update all callers. ~50
   lines in `ros_node.py`, `routes/vehicle.py`, `routes/mission.py`,
   `offboard_controller.py`.
3. Fix 3.3 — `await asyncio.to_thread(...)` everywhere a sync ROS call is
   invoked from async context (covered by step 2 if you go full async).
4. Fix 3.4 — add `python-multipart` to `requirements.txt`.
5. Fix 3.5 — RPP STALE pre-check in `OffboardController.start()`.

**Day 2 — observability + safety net:**

6. 4.2 — pose-stale watchdog in telemetry loop.
7. 4.1 — auto-transition to `COMPLETED` via `RppStatusMonitor.is_done()`.
8. 4.3 — `MultiThreadedExecutor` + `ReentrantCallbackGroup` for service clients.
9. 4.4 — replace bare `pass` with structured logging.
10. 4.6 — `activity_log = deque(maxlen=...)`.
11. 4.7 — fix CORS.
12. 5.8 — add `/api/healthz`.
13. 5.10 — `service_is_ready()` checks.

**Day 3 — production hardening:**

14. 5.1 — shared-secret header auth.
15. 5.2 — file upload limits.
16. 5.3 — split path routers.
17. 5.4 — implement `params` GET / SET via service clients.
18. 5.6 — `logging` configuration; remove `print`.
19. systemd unit file (`drawing-rover-server.service`) with
    `Restart=on-failure`, `WatchdogSec=` driving `/api/healthz`.

---

## 10. Build / Verify Plan

After fixes, the verification chain:

1. **Unit tests** (pytest): `path_manager.load_path` for each builtin and
   each upload type; `RppStatusMonitor.is_done()` settle behaviour;
   `OffboardController` state transitions with a `Mock` ros_node.
2. **Integration tests** with PX4 SITL on Linux: full mission cycle, estop,
   stale-pose recovery. (Note: `ARCHITECTURE.md §11` says SITL is out of
   scope for the firmware patches; that doesn't apply here — the server can
   and should be SITL-tested.)
3. **Hardware bring-up checklist**:
   - `systemctl status px4-dxp` → active.
   - `ros2 topic list | grep -E "(rpp|path|mavros/state)"` → all present.
   - `curl http://localhost:5001/api/healthz` → all green.
   - `curl http://localhost:5001/api/paths` → 6 builtins.
   - QGC visible and `/mavros/state` shows `connected: true`.
   - Open frontend → confirm Socket.IO `telemetry` at expected rate.
   - Run `square_2x2` mission, verify completion event fires, vehicle
     disarms cleanly.

---

## 11. Single-Sentence Summary

**The architecture is sound and the implementation is 80% there, but the
estop mechanism is silently broken in RPP and the rclpy threading model
will deadlock the event loop under load — fix sections 3.1, 3.2, 3.3, 3.4
before any tethered hardware test.**
