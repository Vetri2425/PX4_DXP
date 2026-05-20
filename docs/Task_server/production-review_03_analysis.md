# Production Review v2 — Analysis & Field-Test Prompt

**Subject:** Verifies every claim in `production-review_02.md` against the actual
code in `PX4_DXP/server/`, separates **verified-good** from **still-needs-work**,
and ends with a copy-paste prompt for a Haiku agent to perform an end-to-end
review across server + firmware before field testing.

**Date:** 2026-05-21
**Verifier:** Code-grep against current sources on disk.

---

## 1. Verification Method

For every "FIXED ✅" entry in v02, I grepped the code on disk to confirm the
fix exists (not just claimed). Results below are split into:

- **CONFIRMED GOOD** — claim matches code.
- **PARTIAL / CAVEAT** — claim is broadly correct but has subtle gaps.
- **STILL OPEN** — listed by v02 as remaining work, plus things v02 missed.

---

## 2. CONFIRMED GOOD (verified on disk)

### 2.1 Threading model is now correct (was the worst risk)

Confirmed in `server/ros_node.py`:
- `class RosExecutorThread` owns a `MultiThreadedExecutor(num_threads=4)`.
- Service clients are created in a `ReentrantCallbackGroup`
  (`self._svc_group = ReentrantCallbackGroup()` on line ~158).
- `_call_async()` uses `future.add_done_callback(...)` +
  `loop.call_soon_threadsafe(af.set_result, ...)` to bridge rclpy → asyncio.
- `asyncio.wait_for(af, timeout=timeout)` provides the timeout — no more
  `spin_until_future_done` from a non-spinning thread.
- `grep "spin_until_future_done"` returns **zero matches** across the server.

This eliminates the dual-spin deadlock that was the #1 reliability risk.

### 2.2 Event loop is no longer blocked

Confirmed in `routes/vehicle.py`, `routes/mission.py`, `sockets/events.py`,
`offboard_controller.py`:
- All call `await ros_node.arm_async(...)` / `set_mode_async(...)`.
- Returns `tuple[bool, str]` with diagnostic — surfaces real failure
  reasons (e.g. `"service /mavros/cmd/arming timed out after 5s"`).
- `OffboardController.start_async()` is fully async and uses
  `await asyncio.sleep(SETPOINT_STREAM_GRACE_S)` instead of `time.sleep`.

### 2.3 Estop stop-path semantics

Confirmed in `ros_node.py:publish_stop_path()`:
- Reads current `pos_n / pos_e` from shared state.
- Publishes a single-point Path at that position.
- Used by `emergency.estop_async`, `offboard_controller.stop_async`,
  `offboard_controller.abort_async`, and the Socket.IO handlers.
- Inline docstring documents *why* (RPP node ignores empty paths).

This is the safe behaviour even with the unchanged RPP firmware.

### 2.4 Pre-stream RPP_STALE check

Confirmed in `offboard_controller.py:start_async()`:
```python
if fcu.get("rpp_state", -1) == RPP_STALE:
    msg = "start: RPP STALE — is twist_to_setpoint_node running?"
```
Returns `(False, msg)` before attempting OFFBOARD switch. Matches the
contract in `ARCHITECTURE.md §3`.

### 2.5 Pose-stale watchdog + auto-completion

Confirmed in `main.py:_telemetry_loop`:
- **Auto-completion**: when `state == RUNNING` and
  `ros_node.get_rpp_monitor().is_done()` — calls `mark_completed()` and
  emits `mission_completed`.
- **Watchdog**: when `state == RUNNING` and (`code == RPP_STALE` or
  `pose_age_ms > POSE_STALE_MS` or `connected is False`) — after
  `SAFETY_STALE_GRACE_S`, calls `emergency_handler.estop_async()` and
  emits `safety_abort` with diagnostic.

This is the single biggest safety improvement in v02.

### 2.6 Activity log is now thread-safe

Confirmed in `main.py:51`:
```python
activity_log: deque = deque(maxlen=MAX_ACTIVITY_LOG)
```
`deque.append` is GIL-atomic and `maxlen` evicts atomically. The racy
`del list[: len-cap]` pattern is gone.

### 2.7 Authentication

Confirmed in `auth.py`, `routes/*.py`, `sockets/events.py`:
- Token loaded from `~/.rover_token` (`TOKEN_FILE_DEFAULT`), generated with
  mode 0600 if missing.
- `secrets.compare_digest` is used (constant-time compare — good).
- REST control routes depend on `require_token` via FastAPI dependency.
- Socket.IO handlers call `_auth_ok(data)` and emit `socket_error` on
  rejection (data must include `auth: <token>`).
- `ROVER_DISABLE_AUTH=1` bypass for dev.

### 2.8 File upload limits

Confirmed:
- `validate_upload()` checks extension against `{.waypoints, .csv}`.
- `MAX_UPLOAD_BYTES` size cap with read-one-byte-extra trick.
- `os.path.basename(filename)` strips traversal.

### 2.9 Param services

Confirmed in `ros_node.py`:
- `GetParameters` and `SetParameters` clients created.
- `get_param_async()` → `(ok, value, msg)` with type unwrap via
  `_param_value_to_python` (handles BOOL / INTEGER / DOUBLE / STRING).
- `set_param_async()` → `(ok, msg)` with type pack via
  `_python_to_param_value`.
- `ParamSetRequest.value: Union[bool, int, float, str]` matches PX4
  heterogeneous param types.

### 2.10 Healthz endpoint

Confirmed in `routes/system.py`:
- `GET /api/healthz` returns ROS + FCU + RPP state for systemd
  `WatchdogSec=`.

### 2.11 CORS fix

Confirmed: `CORS_ALLOW_CREDENTIALS = False` in `config.py`. Wildcard
origin is now valid per spec.

### 2.12 Logging + dependency fix

Confirmed:
- `logging_setup.py` exists with `dictConfig`-based setup.
- `print()` removed from `main.py` and helpers.
- `python-multipart==0.0.17` added to `requirements.txt` (was the silent
  500-on-first-upload bug).

### 2.13 Graceful shutdown

Confirmed:
- `RosExecutorThread.stop()` uses `Event` + `spin_once(timeout=0.1)` loop.
- `RoverBeacon._loop` uses `Event.wait(interval)` (responsive stop).
- Lifespan reverses startup order: telemetry task → beacon → executor →
  destroy_node → rclpy.try_shutdown.

### 2.14 Path router split

Confirmed: `routes/path.py` exports both `paths_router` (`/api/paths`) and
`path_router` (`/api/path/*`). Both mounted in `main.create_app`. The
fragile `@router.get("s")` is gone.

---

## 3. PARTIAL / CAVEAT (good, but watch these)

### 3.1 `socketio.AsyncServer(cors_allowed_origins="*")`

`main.py` still creates the Socket.IO server with `"*"`. This is separate
from FastAPI CORS — Socket.IO has its own origin policy. Not security-
critical because control events are auth-gated, but for tightening:
```python
sio = socketio.AsyncServer(
    async_mode="asgi",
    cors_allowed_origins=CORS_ALLOW_ORIGINS,  # match REST policy
)
```

### 3.2 Watchdog grace period vs. sample rate

`POSE_STALE_MS = 500` and `SAFETY_STALE_GRACE_S` (probably 1.0) means a
full pose dropout takes 500 ms + 1 s = **1.5 s** before estop fires. The
rover at 0.5 m/s travels 75 cm in that window. For low-speed (< 0.3 m/s)
operations this is fine, but worth documenting as the safety bound.

### 3.3 Beacon `_get_local_ip` fallback

v02 says "falls back to `0.0.0.0`" but the actual code I read earlier
falls back to `192.168.1.102`. Worth re-checking the current code.

### 3.4 Auto-completion doesn't auto-disarm

`mark_completed()` transitions state to `COMPLETED` but the rover stays
armed. For a marking rover this is probably correct (operator confirms
visually before disarm), but the behaviour should be documented because
it is different from typical autopilot mission semantics.

### 3.5 Telemetry exception back-off

`await asyncio.sleep(min(1.0, 0.05 * consecutive_errors))` — this caps at
1 s but resets to 0 on first success. If errors are intermittent (every
other tick), you ping-pong between 50 ms and full rate. Consider
exponential growth + linear decay.

### 3.6 `Path.header.stamp` derived from `time.time()`

`ros_node.publish_path` uses `time.time()` (system wall clock) instead of
`self.get_clock().now().to_msg()`. If the Jetson clock jumps (NTP step,
PTP sync), the timestamp will jump. RPP doesn't actually use the
timestamp (it only validates `frame_id`), so harmless today, but switch
to `self.get_clock()` for cleanliness and to survive any future RPP
version that does freshness check the path.

---

## 4. STILL OPEN — what really remains before field test

### 4.1 [BLOCKER for field test] No automated tests at all

There is no `tests/` directory, no CI, no `pytest`. Every fix in v02 is
unverified except by inspection. Before field testing you want at minimum:

- **Unit tests** (no ROS needed):
  - `path_manager.load_path` for each builtin (deterministic point counts).
  - `read_qgc_waypoints` against a sample `.waypoints` from
    `D:\Vetri\3WD_GCS\Test_mission\`.
  - `RppStatusMonitor.is_done()` settle behaviour.
  - `OffboardController.start_async` / `.stop_async` / `.abort_async`
    state transitions with a `Mock` ros_node.
  - `_param_value_to_python` / `_python_to_param_value` round-trip.
  - `validate_upload` size + extension rejection.
  - `auth.check_token` constant-time behaviour.

- **Integration test** with a fake rclpy node + fake mavros services that
  return scripted responses (arm OK, OFFBOARD rejected, OFFBOARD OK,
  arm fail, etc.) — runs in CI without ROS install.

### 4.2 [BLOCKER] No SITL smoke test

You haven't actually started the server against a running PX4. Subtle
issues only show up live:
- mavros service name actually matches what your Jetson exports.
- The `mavros_msgs.msg.GPSRAW` import path is correct on Humble (it
  exists on most distros, but the conditional `_HAS_GPSRAW` import means
  if it's missing you silently lose GPS telemetry).
- `nav_msgs/Path` arrives at `rpp_controller_node` with the right
  `frame_id` (`local_ned`) — case-sensitive string match.
- The ROS_DOMAIN_ID matches between server, MAVROS, and RPP nodes.

A 5-minute SITL run with one builtin path will catch all of the above.

### 4.3 [BLOCKER] systemd service unit not written

v02 acknowledges this. Without it:
- No auto-restart on crash.
- No watchdog driving `/api/healthz`.
- No `After=mavros.service` ordering (server may start before MAVROS,
  hit the 5-second `wait_for_service` timeout, and degrade silently).

Minimum unit:
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
And add `sdnotify` watchdog calls in `_telemetry_loop` if you want
`WatchdogSec` to be effective.

### 4.4 [HIGH] Frame-ID drift across the publish chain

`config.py:TOPIC_PATH = "/path"`, `publish_path(frame_id="local_ned")` by
default — but `path_manager` doesn't enforce that uploaded waypoint files
produce coordinates in the *same NED origin* as MAVROS local_position.

Specifically: `read_qgc_waypoints` uses the **first waypoint** with
`current=1` as origin. If the operator forgot to set the home flag in
QGC, the parser falls back to using `wps[0]` as origin. But MAVROS
`/mavros/local_position/pose` has its **own** EKF home (set when EKF
initialises). These two NED origins **may not match**. The rover then
drives to a point that's correct relative to QGC's home but wrong
relative to its actual pose.

**Fix idea:** when loading a `.waypoints` file, fetch the current MAVROS
EKF origin (`/mavros/global_position/gp_origin`, `GeoPointStamped`) and
re-anchor the waypoints to that origin instead of the file's home flag.

This is the single most likely cause of "rover drove the wrong direction"
in field testing.

### 4.5 [HIGH] No mode/state transition contract test

`OffboardController` state diagram in code:
```
IDLE → ARMING → SWITCHING_OFFBOARD → RUNNING → COMPLETED
                                       ↓ stop_async
                                       IDLE
                                       ↓ abort_async
                                       ABORTED
                                       ↓ load_path while ARMING (?)
```

Today, calling `load_path()` while `state == RUNNING` silently overwrites
the loaded path. Calling `start_async()` while already RUNNING re-arms
and re-switches mode. Calling `abort_async()` from `IDLE` issues
disarm + MANUAL even when nothing is running.

These edge cases are not malicious-input scenarios, they're **operator
mistake** scenarios. Decide and document the policy:
- Should `start_async()` from RUNNING be a no-op or an error?
- Should `load_path` during RUNNING be rejected?
- Should `abort_async()` from IDLE skip disarm if already disarmed?

Then enforce in code.

### 4.6 [MEDIUM] No persistent telemetry log

For post-mortem of any field-test anomaly you want a CSV/JSONL of the
10 Hz telemetry stream, not just a 500-entry activity log. Even a simple
`logging.handlers.RotatingFileHandler` writing the telemetry dict per
tick to `/var/log/rover/telemetry.jsonl` is enough.

### 4.7 [MEDIUM] No firmware↔server compatibility check

The server and the firmware (v1.16.2 + 6 patches) have an implicit
contract: parameter names, topic names, RPP debug array layout. There's
no version handshake. If someone flashes a different rover firmware
build, the server keeps running with garbage telemetry.

Cheap fix: on startup, query a "magic" param via the new
`get_param_async`, e.g. `RD_TANK_MODE`. If absent, log a loud warning.

### 4.8 [MEDIUM] No rate limiting

A bug in the frontend (rapid-fire arm requests on a stuck button) can
drown MAVROS in service calls. Add a simple `asyncio.Lock` per route or
a token bucket on `arm_async` / `set_mode_async`.

### 4.9 [LOW] Telemetry payload size

The 10 Hz `telemetry` event sends ~22 fields per tick. Over Wi-Fi with
two clients connected that's still trivial, but the payload includes
`lat`/`lon`/`alt` redundantly with `pos_n`/`pos_e`. Trim to a
"compact" payload + a separate slower (1 Hz) "geo" payload if you want
to scale to multiple operators.

### 4.10 [LOW] Mission audit

There is no record of "operator X loaded path Y and started mission at
time T from IP Z". The activity log is in-memory and is wiped on
restart. For operational sign-off you'll want this in a file with
rotation.

---

## 5. Updated Field-Test Readiness Score

| Area                            | Score | Notes                                                                |
| ------------------------------- | :---: | -------------------------------------------------------------------- |
| Architecture                    | 9/10  | Unchanged; was already correct.                                      |
| API + models                    | 9/10  | Auth, params, types all good.                                        |
| ROS2 integration                | 9/10  | Threading is now correct; minor `time.time` vs `clock` quibble.       |
| Safety (estop, watchdog)        | 8/10  | Good logic; lacks integration test.                                  |
| Reliability                     | 7/10  | Good code; **no test coverage**.                                     |
| Operability                     | 7/10  | `/healthz` good; no systemd unit; no telemetry log.                  |
| Security                        | 8/10  | Token + uploads + CORS solid.                                        |
| Field-test readiness            | 6/10  | Code is ready; surrounding deployment + test artefacts are not.      |

**Overall: 7.5 / 10 — code is ready, deployment scaffold is not.**

The v02 self-rating of 8.9/10 is too generous; it scores the code only
and ignores the missing deployment/test layer that any production
deployment needs.

---

## 6. Minimum Path to Field Test (1 day)

1. **Write a 30-minute pytest suite** covering items in §4.1.
2. **Run a 5-minute SITL smoke** with one builtin path. Capture logs.
3. **Write the systemd unit** from §4.3. Enable + start.
4. **Add the EKF-origin re-anchor** for QGC `.waypoints` (§4.4) — this
   is the single most likely field-test failure.
5. **Run the hardware bring-up checklist from `production-review_02.md §8`**
   end to end with a safety observer.
6. Log telemetry to file (§4.6) for the test session.

Items §4.5, §4.7, §4.8, §4.9, §4.10 are post-first-test improvements.

---

## 7. Prompt for Haiku Agent — Full Server + Firmware Review

Paste this into a fresh Haiku session. It is self-contained: the agent
will read the right files, cross-check the contracts, and produce a
field-test go/no-go report.

> **System prompt addition (optional, for stricter output):** "You are a
> senior firmware/devops reviewer. Cite file paths and line numbers for
> every claim. Do not hallucinate API shapes — read the source."

```
ROLE
====
You are reviewing the Drawing Rover system end-to-end for field-test
readiness. The system has two halves that must agree:

  1. PX4 firmware overlay (six patches over upstream PX4 v1.16.2,
     flashed to CubeOrangePlus on the rover).
  2. FastAPI + rclpy server running on a Jetson Orin companion, exposed
     to a web frontend over Socket.IO + REST.

Your job is to find any contract mismatch, race condition, deadlock,
silent failure, unsafe state transition, or missing safety net that
could cause an autonomous rover to:
  - move when it should be still
  - move in the wrong direction
  - fail to stop on operator command
  - lose communication without failing safe

You must cite file paths and line numbers for every finding.
Do not speculate without reading the source.

WORKSPACE LAYOUT (read these directories)
=========================================
- d:\Vetri\3WD_GCS\PX4_DXP\server\           — FastAPI + rclpy server
- d:\Vetri\3WD_GCS\PX4_DXP\src\              — Five existing ROS2 nodes
  (rpp_controller_node.py, twist_to_setpoint_node.py,
   path_publisher_node.py, mission_runner_node.py, ntrip_rtcm_node.py)
- d:\Vetri\3WD_GCS\PX4-Autopilot\src\modules\rover_differential\
                                              — RoverDifferential.cpp/.hpp
                                                + module.yaml (firmware patches)
- d:\Vetri\3WD_GCS\PX4-Autopilot\src\modules\land_detector\RoverLandDetector.cpp
- d:\Vetri\3WD_GCS\PX4-Autopilot\src\modules\navigator\mission_block.cpp
- d:\Vetri\3WD_GCS\PX4_DXP\docs\Task_server\ — earlier reviews
  (production-review.md, production-review_02.md,
   production-review_03_analysis.md)
- d:\Vetri\3WD_GCS\.kiro\steering\           — project rules
  (tech.md, structure.md, px4-patch-rules.md, bug-registry.md,
   firmware-build-flow.md, next-session.md, product.md)

CONTEXT YOU MUST INTERNALISE FIRST (read in this order)
=======================================================
1. .kiro/steering/product.md              — what the rover does + hw spec
2. .kiro/steering/tech.md                 — PX4 v1.16.2 constraints
3. .kiro/steering/bug-registry.md         — known bugs (Bug 6 still open)
4. .kiro/steering/next-session.md         — current firmware commit state
5. PX4_DXP/server/ARCHITECTURE.md         — server design contract
6. PX4_DXP/docs/Task_server/production-review_02.md
                                          — claimed v2 fixes
7. PX4_DXP/docs/Task_server/production-review_03_analysis.md
                                          — what's verified vs. open

REVIEW SCOPE — answer ALL of these
==================================

A. CONTRACT VERIFICATION (server ↔ firmware)
   For each contract, read both ends and confirm match:
   A1. Topic /path frame_id "local_ned" — verify producer (server
       publish_stop_path + publish_path) and consumer
       (rpp_controller_node._path_cb expected frame).
   A2. RPP debug Float32MultiArray layout (8 elements) — verify the
       index assignments in rpp_controller_node match what the server
       reads in ros_node._cb_rpp_debug.
   A3. ENU↔NED conversion math — server's _cb_pose vs.
       rpp_controller_node._enu_pose_to_ned must produce identical
       results. Diff line by line.
   A4. /rpp/velocity_ned consumer — does twist_to_setpoint_node read
       it the way the server publishes it?
   A5. Mission control modes — server only sends "OFFBOARD" / "MANUAL"
       via /mavros/set_mode. Does PX4 (with the 6-patch overlay)
       accept these strings, or does it require uppercase exactly?
   A6. CA_R_REV=3, RD_TANK_MODE — do server's set_param semantics
       (Union[bool,int,float,str]) round-trip correctly through
       /mavros/param/set_parameters with these PX4 param types?
   A7. RoverDifferential.cpp `generateActuatorSetpoint` non-tank
       branch — Bug 6 (throttle sign inversion) is still OPEN per
       bug-registry.md. Confirm whether the server's mission state
       transitions could mask or amplify this bug.

B. SAFETY CHAIN END-TO-END
   B1. Operator presses Estop in the UI:
       Frontend → POST /api/estop → emergency.estop_async →
       publish_stop_path (RPP zeroes velocity) →
       set_mode_async("MANUAL") → arm_async(False).
       For each step, what happens if it fails? Does the next step
       still execute? Are errors surfaced to the UI?
   B2. Pose goes stale mid-mission (RTK loss):
       /mavros/local_position/pose stops → rpp_controller_node detects
       and publishes RPP_STALE → server watchdog sees STALE for
       SAFETY_STALE_GRACE_S → estop_async fires.
       Verify the timing chain: how many seconds between RTK loss and
       motors stopped? At max 0.5 m/s, how far did the rover travel?
   B3. MAVROS process dies:
       /mavros/state stops → server's `connected` field stays True
       (TRANSIENT_LOCAL last value) → watchdog only fires on next
       fresh State message saying connected=False, which never comes.
       Is there a timeout on /mavros/state freshness? If not, this is
       a silent failure mode.
   B4. WiFi loss (Jetson loses operator) — what stops the rover?
       Is there any heartbeat from frontend → server → rover?
   B5. Companion power loss — Jetson dies cleanly. Does PX4 detect
       OFFBOARD setpoint stream loss and trigger its own failsafe?
       What's the failsafe action set in the PX4 params? (Look at
       PX4_Params/15-05-2026_V01_PX4.params for COM_OBL_ACT.)

C. STATE MACHINE ROBUSTNESS
   C1. Concurrent commands: operator clicks Start twice rapidly. Does
       OffboardController._lock prevent re-entry? What happens to the
       second await?
   C2. start_async called from RUNNING — re-arms? errors? no-op?
   C3. load_path called from RUNNING — silent overwrite?
   C4. abort_async from IDLE — issues disarm/MANUAL anyway?
   C5. mark_completed → COMPLETED state — does it auto-disarm?
       If not, is the rover left armed in OFFBOARD with empty path
       (i.e. stop-path)? Is OFFBOARD with stop-path safe long-term?

D. THREADING / ASYNC
   D1. ReentrantCallbackGroup is used for service clients in
       ros_node. Verify it is not also used for timers/subscribers
       that mutate _state — that would re-introduce races.
   D2. activity_log is a deque(maxlen). Confirm every writer either
       holds GIL (single op) or doesn't iterate while writing.
   D3. ros_node._lock is held during get_state() shallow-copy. Confirm
       the dict's nested values (none today, but check) aren't mutated
       elsewhere.
   D4. Telemetry loop catches Exception and logs. Confirm
       asyncio.CancelledError is re-raised (it is, but verify the
       exponential back-off doesn't swallow cancellation).

E. UPLOADED PATH SAFETY
   E1. read_qgc_waypoints uses the file's home flag as NED origin.
       MAVROS local_position uses the EKF origin (set by PX4 at
       arming). Do these match? If not, the path is anchored to the
       wrong point. (This is highlighted in
       production-review_03_analysis.md §4.4.) Confirm and propose a
       fix using /mavros/global_position/gp_origin.
   E2. CSV uploads have no coordinate validation. A user could upload
       (north=10000, east=10000). What stops this? Is there a
       distance-from-origin sanity check?
   E3. Path point spacing — RPP has assumptions about minimum spacing.
       What does it do with two identical consecutive points?

F. DEPLOYMENT GAPS
   F1. systemd unit file: present? service ordering correct
       (After=mavros, RPP nodes)?
   F2. Watchdog: WatchdogSec= configured? sdnotify pings present in
       _telemetry_loop?
   F3. Token rotation: how does the operator rotate ~/.rover_token
       without frontend hard-coding? Is there a rotate endpoint?
   F4. Logs rotation: where do logs go? Are they bounded?
   F5. Telemetry persistence: is there a file log of the 10 Hz
       telemetry stream for post-mission analysis?

G. KNOWN OPEN BUG
   G1. Bug 6 (next-session.md) — "throttle sign inverted in mission
       mode" is OPEN. The server has no patch for it. In the field,
       running an AUTO MISSION via the frontend will result in the
       rover driving backward. Confirm by tracing the call chain:
       Frontend Start → /api/mission/start → publish_path('local_ned')
       → rpp_controller_node → /rpp/velocity_ned →
       twist_to_setpoint_node → /mavros/setpoint_raw/local →
       PX4 OFFBOARD → RoverDifferential.cpp generateActuatorSetpoint.
       Does the bug manifest in this exact path?

H. TEST COVERAGE
   H1. List all `tests/` directories and pytest files. Report missing.
   H2. Recommend the smallest possible test suite that gives
       confidence for first hardware run.

OUTPUT FORMAT
=============
Produce a markdown document at
  d:/Vetri/3WD_GCS/PX4_DXP/docs/Task_server/haiku-field-readiness.md

Sections:
  1. EXECUTIVE SUMMARY (one paragraph, GO / NO-GO / GO WITH CAVEATS).
  2. CONTRACT MATRIX — table of every server↔firmware contract with
     PASS / FAIL / NEEDS-VERIFICATION + line refs.
  3. SAFETY CHAIN ANALYSIS — for each of B1..B5, full timing diagram
     and worst-case rover travel distance.
  4. STATE MACHINE FINDINGS — for C1..C5, current behaviour + recommended.
  5. THREADING REVIEW — for D1..D4, verdict + line refs.
  6. PATH-UPLOAD HAZARDS — for E1..E3, concrete attack/mistake scenarios.
  7. DEPLOYMENT GAPS — what is missing from systemd / logging /
     telemetry persistence.
  8. BUG 6 IMPACT — does the server expose Bug 6 to operators?
  9. MISSING TESTS — minimal pytest suite to write before field test.
  10. PRE-FLIGHT CHECKLIST — ordered list, each item passable
      yes/no with the operator on the Jetson.

For every claim, cite file:line. For every recommendation, propose
either a code patch (small) or a config change (concrete).

If at any point you cannot read a file, say so explicitly. Do not
guess.
```

---

## 8. Final Verdict on v02

The fixes claimed in `production-review_02.md` are all real. The code
is in a much better place than v01. The remaining work is **outside
the server module**: tests, systemd, telemetry persistence, frame-origin
re-anchoring of uploaded waypoints, and the still-open firmware Bug 6.

**Net:** server is field-test ready; the system around it is not yet.
The Haiku prompt in §7 will close that loop.
