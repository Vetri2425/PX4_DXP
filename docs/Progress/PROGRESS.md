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

## Phase 2 Entries Start Below

<!-- Template for future entries:
## YYYY-MM-DD — [Phase] [Description] (N sessions)

### Built
- [what was built]

### Fixed
- [what was fixed]

### Next
- [what's next]
-->