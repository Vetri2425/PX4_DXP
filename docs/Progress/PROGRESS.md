# DYX 3WD Marking Rover — Project Progress Log

Running log of all work. Each entry: what built, what fixed, what's next, time spent.

---

## 2026-05-15 — Phase 1 Start (3 sessions)

### Built
- PX4 v1.16.2 firmware built and flashed to CubeOrangePlus
- Generic Rover Differential airframe configured (SYS_AUTOSTART=50000)
- Motor outputs mapped: PWM_MAIN_FUNC1=102 (Right), PWM_MAIN_FUNC3=101 (Left)
- RC setup: R8EF v1.6 SBUS, tank mode two-paddle
- MAVROS2 connection established on Jetson via USB serial

### Fixed
- Bug 1: RO_YAW_RATE_LIM=0.87 was deg/s not rad/s → rover never moved in AUTO
- Bug 2: CA_R_REV=3 confirmed correct (bidirectional PWM, not direction flag)
- Bug 3: Waypoint never accepted → firmware fix ecf1d7b5 (mission_block.cpp rover bypass)
- Bug 4: QGC shows "flying" → RoverLandDetector always returns grounded (firmware fix)

### Next
- Fix IK sign reversal (Bug 5)
- Fix throttle sign (Bug 6)
- Begin PID tuning for straight-line AUTO

---

## 2026-05-18 — ArduRover Abandoned (1 session)

### Decision
- Full pivot from ArduRover to PX4+ROS2
- GPL-3 license blocks commercial sale
- ArduRover cannot draw arcs (NAV_LOITER_TURNS only does full circles, densified WPs is the only partial arc method)
- `~/ardupilot/` on Jetson declared dead weight

### Built
- Multi-AI architecture review process started (ChatGPT, Grok, Claude)

---

## 2026-05-19 — Phase 1 Complete (4 sessions)

### Built
- Firmware bug 5 (IK signs) fixed in commit 62619611 (RoverDifferential.cpp)
- Physical wiring fix applied for bug 6 (throttle sign)
- GPS_YAW_OFFSET=180 + IK sign fix confirmed correct orientation
- AUTO mode now works with nose-first motion
- PX4 PID baseline tuning achieved:
  - RO_YAW_RATE_P=0.5, RO_YAW_RATE_I=0.3, RO_YAW_RATE_LIM=30.0
  - RO_SPEED_P=0.5, RO_SPEED_I=0.1
  - NAV_ACC_RAD=0.1, MIS_YAW_ERR=25.0
- Log evidence: NAV_ACC_RAD=0.1 gives xtrack avg=0.006m (best)

### Fixed
- All 6 firmware bugs resolved
- Wiring fix resolved physical direction issues

### Next
- Production-harden runtime stack (systemd, NTRIP, service)
- Begin architecture review for Phase 2

---

## 2026-05-20 — Phase 1.5 Complete (3 sessions)

### Built
- `ntrip_rtcm_node.py` — full rewrite with 20+ fixes:
  - CRC-24Q validation on every RTCM3 frame (discard corrupt)
  - Reserved bits soft-check (warns but proceeds for non-compliant casters)
  - GGA send failure suppression counter (3 warns then silent)
  - `_gga_lock` threading.Lock for `_gga_sock` race condition
  - NavSatStatus constants corrected (STATUS_GBAS_FIX / STATUS_SBAS_FIX)
  - QoS: BEST_EFFORT depth=10 (was RELIABLE depth=1)
  - Health monitoring: 30s timer, reconnect counter
  - Exponential backoff: min(5×2^attempt, 60), interruptible
- `px4_start_service.sh` — production hardening:
  - NTRIP_SCRIPT derived from SCRIPT_DIR (inside repo)
  - `ntrip_watchdog()` with own TERM/INT trap, restart loop
  - Env var validation before NTRIP watchdog start
  - Log rotation at startup if >10MB
  - `free_port()` graceful: SIGTERM first, SIGKILL only if needed
  - Named timing constants (no magic numbers)
  - FCU validation: `ros2 topic echo /mavros/state --once --timeout 5`
  - pkill patterns fixed: "mavros.*node.launch" + "ntrip_rtcm_node"
- `px4-dxp.service` — hardened systemd unit:
  - BindsTo=dev-ttyACM0.device, After=dev-ttyACM0.device
  - ProtectSystem=strict, ReadWritePaths narrowed
  - EnvironmentFile uncommented (deploy.sh creates env file)
  - WatchdogSec commented out (needs sd_notify, not yet implemented)
  - CPUQuota=400% (4 cores for Phase 2)
- `deploy.sh` — symlink-based deployment:
  - Symlinks systemd service → /etc/systemd/system/
  - Symlinks logrotate config → /etc/logrotate.d/
  - Creates NTRIP env file (prompts once, skips if exists)
  - Reloads systemd daemon + enables service
  - --restart flag for immediate service restart
- `ntrip.logrotate` — daily rotation, 7-day retain, 10MB max, copytruncate
- `px4_pluginlists_rover.yaml` — 10 denied plugins with inline comments + gps_rtk intent note
- `docs/MAVROS_vs_DDS.md` — MAVROS2 vs uXRCE-DDS comparison
- `docs/Pure_DDS.md` — Pure DDS architecture + migration path
- `docs/Architecture/FINAL_ARCHITECTURE.md` — consolidated final architecture
- `docs/Progress/PROGRESS.md` — this file

### Fixed
- All 20+ bugs from audit + Kiro review resolved
- NTRIP_SCRIPT path ordering bug (SCRIPT_DIR must be defined before NTRIP_SCRIPT references it)
- Stale comment "depth=1" in ntrip_rtcm_node.py QoS (now depth=10)
- .gitignore: ntrip_rtcm_node.py stays in version control (credentials via env vars)

### Design decisions
- All runtime files inside `~/PX4_DXP/` (git repo) — no scattered files outside
- System files symlinked by deploy.sh — git pull auto-updates, just restart service
- NTRIP node inside repo — old `~/ntrip_rtcm_node.py` is dead
- NTRIP credentials in `~/.config/ntrip/env` (not in repo, created by deploy.sh)

### Next
- Deploy to Jetson: `git pull && rm ~/ntrip_rtcm_node.py && ./deploy.sh --restart`
- Phase 2: ROS2 Offboard control node
- OFFBOARD mode: stream setpoints ≥2Hz → arm → mode switch
- First milestone: velocity setpoint → straight-line motion

---

## 2026-05-20 — Phase 2 Prep (1 session)

### Built
- OFFBOARD audit complete (Kiro Opus): 3 firmware bugs found, 4 patches specified
- MAVROS2-only architecture decision finalized (DDS shelved)
- Full stack license audit: all permissive, zero GPL contamination
- Architecture docs committed: MAVROS2_ONLY_DECISION.md, LICENSE_AUDIT.md, KIRO_OPUS_OFFBOARD_AUDIT_PROMPT.md
- CubeOrangePlus port map verified from param files (TELEM2 free for future DDS)

### Fixed
- Identified OFFBOARD bug #1: velocity sign lost (`velocity.norm()` always positive, can't reverse)
- Identified OFFBOARD bug #2: North-snap at zero velocity (`atan2f(0,0)=0`, rover yaws to North on stop)
- Identified OFFBOARD bug #3: latent runaway on OFFBOARD exit (cached position setpoint never NaN-invalidated)
- Identified OFFBOARD bug #4: no was_armed guard in RoverDifferential (one-cycle motor linger on disarm)
- Corrected #18346 analysis: POSCTL fallback goes through manualPositionMode (reads RC stick = zero → safe stop), NOT goToPositionMode. Bug is latent, not active.

### Next
- Set FCU safety params (COM_OBL_RC_ACT=5, COM_OF_LOSS_T=0.3, COM_RCL_EXCEPT=4, RD_TANK_MODE=0)
- Apply firmware patches P1-P4 to PX4 fork
- Extend build_rover.yml to copy VelControl + PosControl files
- Push fork, CI build, flash to CubeOrangePlus
- Start Phase 2: write OFFBOARD ROS2 node on Jetson

## 2026-05-20 — Phase 2 Start: OFFBOARD Patches Applied (1 session)

### Built
- Firmware patches P1-P4 committed and pushed to fork (commit 1e2ce81a)
  - P1: DifferentialPosControl — NaN-invalidate cached position on OFFBOARD exit + disarm
  - P2: RoverDifferential — _was_armed guard, zero actuator on disarm, slew-rate reset
  - P3: DifferentialVelControl — signed speed projection (body-x axis) for reverse motion
  - P4: DifferentialVelControl — hold-yaw-at-stop (freeze _vehicle_yaw when vel < 0.01 m/s)
- DifferentialVelControl directory created in fork (was missing from overlay)
- build_rover.yml extended: now copies VelControl (.cpp/.hpp/CMakeLists.txt) + PosControl (.cpp/.hpp)
- CI build triggered on push to main

### Next
- Monitor CI build at https://github.com/Vetri2425/PX4-Autopilot/actions
- Download firmware artifact, flash to CubeOrangePlus
- Set 13 FCU params in QGC (safety + performance)
- Begin OFFBOARD ROS2 node on Jetson (Phase 2 milestone 1: straight-line velocity)

## Phase 2 Entries Start Below

## 2026-05-20 — Phase 2 Session 5: RPP Pipeline Built (1 session)

### Built
- **rpp_controller_node.py** (~577 lines) — Regulated Pure Pursuit controller
  - Outputs **NED velocity vector** (Vector3Stamped on /rpp/velocity_ned), NOT body-frame (v, ω)
  - PX4 derives yaw from atan2(vE, vN) in DifferentialOffboardMode — no ω command needed
  - Segment projection (not vertex search) for closest-point on path
  - Curvature-regulated speed: slows on tight curves, full speed on straights
  - Approach scaling: linear deceleration in last 0.6m to goal
  - P4 zero-vel floor: below 2 cm/s, set speed=0 to trigger heading-hold
  - Pose freshness check: stale >200ms → emergency stop (0,0,0), OFFBOARD stays alive
  - Publishes /rpp/debug (8 floats: xtrack, heading_err, lookahead, speed, κ, dist_goal, pose_age, state)
  - No rotate-to-heading FSM — PX4 spot-turn handles large heading errors (RD_TRANS_DRV_TRN)
- **twist_to_setpoint_node.py** (~231 lines) — MAVROS OFFBOARD heartbeat bridge
  - 50Hz PositionTarget stream, type_mask=3527, FRAME_LOCAL_NED
  - Input already in NED — no body→NED transform needed (RPP outputs NED)
  - Stale input (>200ms) → zero velocity (safe fail-stop, OFFBOARD stays live)
  - NaN/Inf rejection on input
- **path_publisher_node.py** (~185 lines) — Test paths
  - straight_5m, arc_quarter_1m5, lshape_2x2
  - TRANSIENT_LOCAL durability, frame_id validation
- **xtrack_logger_node.py** (~269 lines) — 20Hz CSV logger
  - 18 columns: t, pose, xtrack, heading_err, speed, κ, state, velocity, MAVROS setpoint
  - Flushes every ~1s for crash resilience
- **mission_runner_node.py** (~350 lines) — OFFBOARD lifecycle state machine
  - INIT → WAIT_FCU → WAIT_STREAM → SWITCH_OFFBOARD → ARM → RUNNING → DISARM → MANUAL → FINISHED
  - 5Hz tick, mission timeout (5 min default), external OFFBOARD exit detection
  - Dry run mode for telemetry capture without arming
  - Monitors /rpp/debug state_code for DONE detection
- **launch/rpp_pipeline.launch.py** (~169 lines) — Ordered startup
  - twist_to_setpoint first (heartbeat), rpp_controller second, path_publisher after 2s
  - auto_run flag: mission_runner after 4s (OFFBOARD + arm)
  - dry_run flag: skip arm/mode commands

### Key architectural change from original T3 spec
- Original spec: RPP outputs body-frame (v, ω) → twist_to_setpoint does body→NED rotation
- Built system: RPP outputs NED velocity vector → twist_to_setpoint just wraps in PositionTarget
- Reason: PX4 v1.16 DifferentialOffboardMode computes `bearing = atan2(vE, vN)` from velocity vector direction. It ignores yaw/yaw_rate in the setpoint. Sending ω would be pointless.
- PX4's internal spot-turn FSM (RD_TRANS_DRV_TRN ≈ 30° → spot-turn, RD_TRANS_TRN_DRV ≈ 5° → resume driving) handles large heading errors automatically.

### Research task status
- T1 (Mission Formats) — TODO
- T2 (Trajectory Planning) — TODO
- T3 (Controller Pipeline) — **COMPLETE** (code written, not yet tested)
- T4 (Sensor Fusion) — TODO
- T5 (RPP Arc Controller) — **MERGED INTO T3**
- T6 (Full System Architecture) — TODO

### Next (pre-hardware checklist, in order)
1. Run Motion Studio autotune → get RBCLW_QPPS_MAX value
2. Add SER_TEL2_BAUD = 115200 to param file
3. Flash firmware with RoboClaw QPPS patch
4. Verify both motors spin forward with positive command
5. Fix NTRIP → validate RTK → retest velocity mode
6. SITL validation of RPP pipeline (Gazebo + PX4 SITL)
7. Hardware bring-up with RTK (straight line → arc → L-shape)
8. Research T1/T2/T4 for Phase 3 (CAD → mission pipeline)

---

## 2026-05-20 — Phase 2 Session 4: Research & Architecture (1 session)

### Built
- Research tasks T1-T6 created in `docs/Researches/COMMERCIAL_ROVER_RESEARCH/`
- T3 Controller Pipeline synthesis completed (multi-AI research: ChatGPT, Gemini, GLM, Grok + primary sources)
- T3 FINAL_SYNTHESIS.md: RPP on Jetson, velocity setpoints only, MAVROS2 only, no Nav2 stack
- RoboClaw driver patch (Kiro Opus): open-loop duty → closed-loop velocity QPPS (opcodes 35/36)
- Param file `Param_with_Roboclaw.params` created with RoboClaw params + safety params

### Decisions from T3 synthesis
1. **RPP (Regulated Pure Pursuit)** on Jetson — NOT Stanley, NOT MPC
2. **Velocity setpoints only** (type_mask 3527) — position setpoints stack two pure-pursuit controllers = oscillation
3. **MAVROS2 only** — uXRCE-DDS rover offboard broken (forum bug 48430, unresolved)
4. **No Nav2 stack** — overkill for marking with no obstacles
5. **Custom rpp_controller_node.py** (~200 lines) + **twist_to_setpoint_node.py**
6. Build order: RPP node → twist_to_setpoint → path source → logger → SITL → hardware

### RoboClaw driver update
- **Opus patch applied**: `setMotorSpeed()` now sends QPPS velocity commands (opcodes 35/36) instead of duty (opcodes 0/1/4/5)
- **RBCLW_QPPS_MAX = 0** in param file — **CRITICAL**: must be set from Motion Studio autotune, 0 = no motion
- **SER_TEL2_BAUD missing** — must be added (recommend 115200, must match RoboClaw config)
- PWM_MAIN_FUNC1-8 all set to 0 (motors moved from PWM to RoboClaw)
- CA_R_REV = 3 still applies (control allocator reversal before driver)

### Open issues
- **RBCLW_QPPS_MAX** — must be measured with Motion Studio autotune before flashing
- **SER_TEL2_BAUD** — must be added to param file (115200 recommended)
- **P3 (reverse motion)** — not validated without RTK
- **P4 (heading hold)** — not validated without RTK
- **NTRIP server 502** — external issue, blocks RTK testing

### Next
- Run Motion Studio autotune → get RBCLW_QPPS_MAX value
- Add SER_TEL2_BAUD = 115200 to param file
- Flash firmware with RoboClaw QPPS patch
- Test RoboClaw motor direction (both forward with positive command)
- Fix NTRIP → validate RTK → retest velocity mode
- Build rpp_controller_node.py (can write code now, test later with RTK)

---

## 2026-05-21 — Phase 2 Session 6: FastAPI Backend Server Built (1 session)

### Built
- **FastAPI backend server** (17 files, ~2500 lines) in `PX4_DXP/server/`
  - `main.py` — FastAPI app factory with lifespan, Socket.IO mount, 10Hz telemetry loop with watchdog
  - `ros_node.py` — Single rclpy node in background thread with MultiThreadedExecutor (4 threads)
    - Subscribes to 7 MAVROS/RPP topics (state, pose, battery, GPS, RPP debug, RPP velocity)
    - Publishes `/path` topic (TRANSIENT_LOCAL QoS for late-joining subscribers)
    - Service clients for arm/disarm, set_mode, param get/set
    - ENU→NED conversion for pose and heading
    - Async service wrappers (`arm_async`, `set_mode_async`, `get_param_async`, `set_param_async`) using `call_async` + `add_done_callback`
    - MAVROS process-crash detection: `_state_recv_time` timeout overrides TRANSIENT_LOCAL cached `connected=True`
  - `offboard_controller.py` — Async OFFBOARD lifecycle state machine
    - States: IDLE → ARMING → SWITCHING_OFFBOARD → RUNNING → STOPPING → IDLE (COMPLETED, ABORTED branches)
    - Pre-flight checks: FCU connected, RPP not STALE
    - OFFBOARD pre-stream grace period (0.5s delay before path publish)
    - `publish_stop_path()` — publishes single-point path at rover's current position (empty Path ignored by RPP)
    - Async lock on lifecycle calls to prevent concurrent arm/mode-switch
  - `path_manager.py` — Path loading (6 built-in generators + QGC .waypoints + CSV)
    - `lru_cache` on builtin generators for fast repeated access
    - Upload validation: extension whitelist (.waypoints, .csv), 1MiB size limit
    - Karney geodesic conversion for QGC WPL 110 format (same method as path_publisher_node)
  - `rpp_status.py` — RPP debug array decoder with done-settle detection (1.0s default)
  - `emergency.py` — Async e-stop: stop-path + MANUAL mode + disarm (3-step chain with per-step error handling)
  - `beacon.py` — UDP broadcast for LAN discovery (port 5002, every 2s)
  - `auth.py` — Shared-secret token auth (`~/.rover_token`, auto-generated, `ROVER_DISABLE_AUTH=1` to bypass)
  - `logging_setup.py` — Structured logging with ISO-8601 timestamps
  - `config.py` — All constants centralized: topic names, service names, QoS profiles, safety thresholds
  - `models.py` — Pydantic v2 request/response models with typed enums
  - Routes (6 modules): system, vehicle, mission, path, params, telemetry — all auth-protected except telemetry and ping
  - Socket.IO events: arm, set_mode, emergency_stop, mission_load/start/stop/abort, request_params — all auth-protected
  - Telemetry loop (10Hz): pushes telemetry + mission_status via Socket.IO, auto-completes on RPP DONE, auto-aborts on pose stale/disconnect

### Key architecture decisions
- **Pure rclpy** (no roslibpy, no CLI fallback) — server runs on same Jetson as ROS2 nodes
- **Async service calls** — `call_async` + `add_done_callback` + `loop.call_soon_threadsafe`, never blocks FastAPI event loop
- **MultiThreadedExecutor(4)** — prevents callback starvation from service calls blocking subscriptions
- **Token auth** — shared secret, auto-generated, bypass with env var for dev/LAN-only
- **Stop-path instead of empty Path** — RPP node ignores empty Path (early return), so e-stop publishes single point at rover's current position

### API endpoints
| Method | Path | Purpose |
|---|---|---|
| GET | /api/ping | Health check |
| GET | /api/healthz | Detailed readiness (FCU, RPP state, pose age) |
| GET | /api/activity | Activity log (last 500) |
| POST | /api/arm | Arm/disarm vehicle |
| POST | /api/set_mode | Set MANUAL/OFFBOARD |
| POST | /api/estop | Emergency stop |
| POST | /api/mission/load | Load path by name |
| POST | /api/mission/start | Start OFFBOARD mission |
| POST | /api/mission/stop | Soft stop (stay armed) |
| POST | /api/mission/abort | Hard abort (MANUAL + disarm) |
| GET | /api/mission/status | Current state + RPP status |
| GET | /api/paths | List built-in + uploaded paths |
| POST | /api/path/upload | Upload .waypoints or .csv |
| POST | /api/path/publish | Publish path to /path topic |
| DELETE | /api/path/{filename} | Delete uploaded file |
| GET | /api/params/{name} | Get PX4 param |
| POST | /api/params/{name} | Set PX4 param |
| GET | /api/telemetry/latest | Telemetry snapshot |

### Next
- Add server to `px4-dxp.service` or create separate systemd unit
- Test with SITL (PX4 SITL + MAVROS + RPP pipeline + server)
- Hardware bring-up: verify full mission cycle via API
- Build frontend (React dashboard)
- Research T1/T2/T4/T6 for Phase 3

---

## 2026-05-20 — Phase 2 Sessions 1-3: OFFBOARD Test Node (1 session)

### Built
- `src/offboard_test.py` — OFFBOARD test node with two modes:
  - **Position mode** (Session 2): 1m North in NED, hold, stop, disarm
  - **Velocity mode** (Session 3): forward 0.3 m/s → stop → reverse -0.3 m/s → stop → hold → disarm
  - 50Hz setpoint stream, 1s preflight, OFFBOARD mode confirmation, auto-disarm on exit
  - STATUSTEXT subscription for FCU denial reasons
  - ExtendedState subscription for landed state / system status
  - Mode reset to MANUAL before OFFBOARD (prevents stale state from previous test)
- Position mode: **WORKING** — armed, drove toward NED target, disarmed
- Velocity mode forward: **WORKING** — both motors same direction after ENU→NED fix
- Velocity mode reverse: **NOT WORKING** — P3 not active, rover spot-turns instead of reversing
- Jetson + laptop PX4_DXP repos synced (both at commit `dd2a134`)

### Fixed
- **Bug: FRAME_BODY_OFFSET_NED (9) rejected** — PX4 rover firmware error `coordinate frame 9 unsupported`. Fix: use FRAME_LOCAL_NED (1) + body→NED velocity transform in node code
- **Bug: ENU→NED yaw 90° error** — MAVROS `/mavros/local_position/pose` publishes quaternions in ENU frame (0°=East, CCW). Code was using ENU yaw as NED yaw (0°=North, CW), rotating all velocity setpoints 90° off heading. Fix: `yaw_NED = π/2 - yaw_ENU`
- **Bug: Arming denied without stable heading** — NTRIP server 502 → no RTK → heading estimate unstable → PX4 refuses arm (ERROR, not WARN). COM_ARM_WO_GPS=1 does NOT bypass heading stability check. Workaround: disable GPS preflight check in QGC
- **Bug: Stale OFFBOARD mode from previous test** — shutdown tried HOLD (rover doesn't have HOLD). Fix: switch to MANUAL on shutdown; reset to MANUAL before starting OFFBOARD sequence
- **Bug: Double disarm race condition** — shutdown handler fires before state callback updates. Harmless but noisy.

### Open issues
- **P3 (reverse motion) not working in OFFBOARD** — PX4 rover velocity controller interprets negative speed as "turn 180° and drive forward" instead of "drive backward." P3 patch should fix this but appears NOT active in OFFBOARD velocity control path. Need to verify: is commit `24d78a81` actually flashed? Does P3 apply in OFFBOARD mode?
- **P4 (heading hold at stop) NOT validated** — heading too unstable without RTK to test
- **Throttle ramp slow** — 60% throttle produced only 0.01 m/s in 3s. Acceleration limiting (RO_ACCEL_LIM) causes gradual ramp. Not hardware.
- **NTRIP server down** — external 502 Bad Gateway, no RTCM corrections flowing
- **13 safety params NOT set on FCU** — COM_OF_LOSS_T, COM_OBL_RC_ACT, COM_RCL_EXCEPT, RD_TANK_MODE, etc.

### Next
- **Fix NTRIP** (external server issue) → RTK corrections → stable heading → retest velocity mode
- **Verify P3 firmware** — is commit `24d78a81` actually flashed on CubeOrangePlus?
- **Set safety params** on FCU via QGC
- **Session 4**: Pure-pursuit arc controller node (can write code now, test with RTK later)

---

<!-- Template for future entries:
## YYYY-MM-DD — [Phase] [Description] (N sessions)

### Built
- [what was built]

### Fixed
- [what was fixed]

### Next
- [what's next]
-->