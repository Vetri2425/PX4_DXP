# PX4_DXP Production Upgrade Roadmap

**Project:** PX4_DXP Autonomous Line Marking Rover  
**Objective:** Upgrade current prototype into production-grade field-ready autonomous rover before large-scale field validation.

Status as of **29 June 2026**

---

# Overview

We completed architectural forensic audits of the full rover stack and identified major production weaknesses.

The goal is **NOT feature development**.

The goal is:

- production reliability  
- deterministic safety  
- autonomous mission robustness  
- real-world field validation readiness  
- zero hidden failure modes  

This task folder contains the mandatory hardening tasks before production field testing.

---

# Completed Architecture Tasks

---

## Task — Virtual Joystick V2 (Production-Grade Manual Control)

### Goal

Operator-driven manual control of the rover from the React Native app via MAVLink
`MANUAL_CONTROL`, safety-gated (armed + confirmed MANUAL mode, lease ownership,
dead-man, fixed-rate gateway with stale-command watchdog), without disturbing the
OFFBOARD mission pipeline.

### Architecture

MAVLink `MANUAL_CONTROL` via mode switch to MANUAL (body-frame, no dual-publisher
conflict). Backend gateway publishes `/mavros/manual_control/send` at ~50 Hz;
neutral pre-stream before acquire; single-owner lease with sequence/replay/rate
validation and a 2 s lease-revoke watchdog. Full plan in
[`Task_Virtual_Joystick_V2.md`](Task_Virtual_Joystick_V2.md).

### Delivered + bugs fixed

- **Backend float32 fix** (`server/manual_control_gateway.py`, commit `8cbc071`):
  was publishing `int` into `mavros_msgs/msg/ManualControl` (float32 x/y/z/r) →
  AssertionError swallowed → **zero MANUAL_CONTROL frames ever sent** while
  `transport_healthy` lied True. Cast to `float()`; assignments moved inside the
  try so `_publish_errors`/`is_healthy()` trip honestly. `3cad3b8` adds server-side
  rejection logging. Topic now streams ~47 Hz (z=500 neutral / 575 fwd / 425 rev,
  y=±500 steer).
- **Frontend heartbeat-starvation fix** (`Three_Wheel_v2`
  `src/hooks/useVirtualJoystick.ts`): the real `lease_timeout` cause — `setIntent`
  called `scheduleCommandLoop(0)` every ~60–120 Hz gesture frame, cancelling the
  self-perpetuating ~18 Hz command heartbeat so its timer was starved and never
  fired → backend saw silence → revoked the lease. Guarded the kick behind
  `!commandLoopRunningRef.current` in `setIntent`/`setDeadman`.
- **Bench harness**: `joy_frontend_sim.py` + `joy_motion_seq.py` (python-socketio,
  mirror the frontend) used to isolate backend-vs-frontend and validate
  arm/acquire/forward/reverse/left/right/disarm.

### Status

**COMPLETED & PRODUCTION-VALIDATED — 26 June 2026.** Field-validated end-to-end on
the Jetson (wheels off-ground): Acquire → arm → forward/reverse/left/right →
release/disarm, lease held, rover drives correctly. Backend committed + deployed
(`8cbc071`, `3cad3b8`). Frontend fix applied in `Three_Wheel_v2` (deploy = EAS/
native rebuild, operator side).

Deferred (non-blocking): backend `_validate_rate` zero-tolerance gate (send
≤~18 Hz until fixed with a jitter margin); pin `ROVER_JOYSTICK_MANUAL_ENABLED=1`
via systemd drop-in (currently rides on an uncommitted `config.py` default on the
Jetson).

---

## Task_01 — PX4 Point Navigation Mode Production Hardening

> Note: an earlier draft of this entry was titled "Controller-Owned Spray
> Compensation Architecture." The canonical task definition is
> [`Task_01.md`](Task_01.md) — *PX4 Point Navigation Mode Production Hardening*.
> The spray-flow-compensation concept lives in Task_02. This entry now matches
> `Task_01.md` and the existing cross-references in Task_10 and Task_12.

### Goal

Harden the existing PX4 Point Mission architecture (PointMissionOrchestrator →
one runtime leg per CSV point → RPP track → arrival/settle → atomic dwell spray
→ advance) for production bench/field validation, without rewriting Point Mode.

### Delivered (Task_01.md Actions 1–11)

- Point orchestrator dependency: point load fails cleanly (HTTP 503) instead of
  silently loading a placeholder path
- Mission clear drains the point task, cancels active dwell, forces spray OFF,
  resets point state to IDLE/unloaded
- Auto / Manual-continue execution modes (`POST /api/mission/point/continue`,
  `WAITING_FOR_CONTINUE`)
- Obstacle pause/resume hook (`/rover/obstacle_clear`) with explicit enable/age
  policy: `not_configured` / `ok` / `missing` / `stale` / `blocked`,
  fail-closed pause when enabled
- Strict GPS/RTK failsafe for `GPS_SURVEYED` (fix-type, staleness, skew, anchor,
  resolution gates; `FAILED_GPS_SAFETY` / `PAUSED_GPS_SAFETY`); `LOCAL_NED`
  unaffected
- Dwell + terminal hold (single-point hold path keeps RPP in DONE/zero-velocity;
  no coast on a stale 2-point leg after COMPLETED)
- Per-point `mark=true` (dwell) / `mark=false` (navigate-only) support
- Max-dwell validation (`point_max_dwell_s`); reject zero/negative/non-finite/over-max
- CSV contract: headerless `north,east[,dwell_s]` + optional header with `mark`
- Full point runtime diagnostics surfaced in `GET /api/mission/point/status`
- Point-leg trajectory module (`two_point` / `densified`) with RPP conditioning
  prediction

### Production-readiness hardening (2026-06-24 review blockers)

- Spray-OFF confirmation policy: always command OFF; confirmation mandatory for
  marked legs / spraying pause-fault / stop-abort-clear-terminal cleanup;
  best-effort for pure `mark=false` navigation so a stale spray node can't fail
  a nav-only run
- `cancel_and_drain` bounded so a slow/hung spray service can't wedge or 500
  stop/abort/clear/start-replace; gate cancel, forced spray-OFF, and
  task/run-token cleanup always run
- `_run` terminal cleanup never leaks an unretrieved task exception; truthful
  `terminal_safety_ok` / `terminal_safety_reason` separating mission-work
  completion from degraded terminal safety
- Leg-diagnostic response fields, `continue_point(ros_node)`, source-index
  provenance, pause/resume transition logging, near-zero densified-leg consistency

### Follow-up — per-path spray-mode configuration (commit `a37e99b` + fix)

Added per-path spray-mode endpoints backed by a hidden JSON sidecar, so an
operator can set continuous/dash/point parameters per path without editing the
plan-and-stage body (`spray_mode` omitted → sidecar flow; explicit → legacy):

- `GET/DELETE /api/path/{name}/spray-mode`
- `PUT /api/path/{name}/spray-mode/{continuous|dash|point}`
- new `server/spray_mode_store.py` (atomic write, validation→422, path-safe,
  corrupt-file fallback to defaults)

**Production fix (this change, working tree — not yet committed/deployed):**
continuous-mode params (`solenoid_*_delay_s`, `*_overspray_margin_m`,
`min_spray_speed_mps`, `max_xtrack_error_m`, `nozzle_*_offset_m`) were absent
from `staged_spray_defaults()` and never written into the staged artifact, so
config set via the new `/spray-mode/continuous` endpoint was **silently
ignored** — the controller always received factory defaults. Fixed by adding
the continuous keys to `staged_spray_defaults()` and propagating them through
both branches of `_stage_mission` into the staged artifact (legacy/explicit
flow keeps factory continuous params, unchanged). Regression coverage added in
`server/test_spray_mode.py` (8 tests). Full server suite 344 passed,
non-ROS `src/` 35 passed, compileall clean, `git diff --check` clean.

### Status

**COMPLETED — 24 June 2026. Committed, pushed, and deployed to Jetson.**

Implementation complete; committed `38c6d8e` and pushed to `origin/main`.
Automated tests PASS: server suite 336 passed, non-ROS `src/` 35 passed,
`compileall` clean, `git diff --check` clean. New deterministic coverage in
`server/test_point_production_hardening.py` (all-`mark=false` stale-spray
completion, marked-leg OFF confirmation, hung-service stop/abort/clear,
cleanup-always-cleared, no-leaked-exception, obstacle disabled/fresh/stale/
never-received, status-schema). Deployed to Jetson `38c6d8e` (`rover-server` +
`rpp-pipeline` restarted, both active, `NRestarts=0`, no import errors;
`px4-dxp`/MAVROS untouched).

Pending (operator, at the rover): controlled Jetson **bench validation** of
Point Navigation Mode, then field validation — including the trajectory/
path-generation checks in `Task_01.md` (densified spacing, endpoint
preservation, no intermediate corner-stop, terminal stop/hold). RTK must reach
FIXED (currently observed at fix_type=5/FLOAT, so the controller correctly
refuses to drive) before a drive test.

---

## Task_02 — Dynamic Speed-Based Spray Flow Control

### Goal

Make spray density consistent regardless of rover speed changes.

### Core Architecture

Runtime continuously adjusts spray flow using actual rover speed.

Formula:

target_flow = desired_paint_density × actual_speed

### Key Improvements

- fixed line thickness
- no over-spray during slow movement
- no under-spray during fast movement
- controller owns physical fluid behavior

### Delivered (software — production spray architecture upgrade)

Controller-owned spray architecture hardened end-to-end, preserving the
Planner=geometry / RPP=motion / Spray=timing+flow ownership model:

- **Deterministic path identity** — new `src/path_identity.py` (mission_id +
  geometry/flag SHA-256 fingerprint + configuration revision envelope). Server
  computes the staged fingerprint over exact merged geometry
  (`server/routes/path.py`); RPP republishes the **raw** staged fingerprint on
  `/rpp/conditioned_path_identity` to bind config→conditioned geometry, with the
  conditioned-geometry fingerprint kept diagnostic-only.
- **Strict mission binding** — spray controller refuses a conditioned path
  unless mission id, path fingerprint, and a positive configuration revision all
  match (`_conditioned_identity_matches_config`); mismatch/cleared identity drops
  the model and forces spray OFF.
- **Deterministic fingerprint validation** — rejects NaN/Inf, malformed/
  non-numeric coordinates, non-bool flags, and single-point geometry.
- **`timing_only_compatibility`** as the only explicit legacy opt-in that relaxes
  strict binding (default off everywhere).
- **Compensation lockout** — production planning routes reject
  `compensate_spray=True` at both the API model (`server/models.py`) and
  `PathManager.plan_path`; geometric pre-shift stays offline-only.
- **Unsafe-speed policy closed** to `BLOCK_SPRAY` / `CLAMP_PWM`; `PAUSE_MISSION`
  rejected everywhere (`src/spray_config.py`, `spray_controller_modes.py`).
- **Truthful telemetry** — separates actual measured speed, unavailable target
  speed (`target_speed_source: "unavailable"`), commanded actuator state,
  software actuator state, and unavailable physical confirmation
  (`physical_actuator_state: "UNAVAILABLE"`).
- **Exact cross-run spray boundaries** — conditioned-path publisher preserves
  duplicate boundary vertices whose spray flag / profile differ; spray is ON only
  when both endpoints of a segment are MARK (conservative OFF→ON / ON→OFF).
- New regression coverage: `src/test_path_identity.py`,
  `server/test_task02_spray_architecture.py`, `src/test_path_publisher_node.py`,
  plus a repo-wide `conftest.py` removing the need for ad-hoc `PYTHONPATH`.

### Status

**SOFTWARE COMPLETE — 24 June 2026. Strict read-only production review: CONDITIONAL GO.**

Final review verified all ten safety-critical points (cross-run boundary
exactness, `PAUSE_MISSION` rejection, strict mission/fingerprint/revision
binding, clear-forces-OFF, explicit `timing_only_compatibility`, deterministic
fingerprint rejection of malformed input, telemetry state separation, planner
compensation lockout, raw-vs-conditioned fingerprint consistency, ownership
docs). No blocking findings.

Automated tests PASS (non-ROS, this environment): path-identity + spray-config +
Task_02 architecture guards 24 passed; spray-controller / flag-conditioning /
manual-override / path-publisher 66 passed; `path_engine` engine 47 passed;
offboard controller 22 passed; `git diff --check` clean.
`server/test_path_api.py` could not run here (missing optional `python-multipart`
dependency, not installed); its compensate-spray-rejection cases are independently
covered by `server/test_task02_spray_architecture.py`.

Known non-blocking item: stale PWM doc comment in
`src/spray_controller_node.py` (says `PWM_AUX_MAX1=2000`/`3000`; validated
production value is `15000`) — comment-only, no behavioral effect; correct before
field day.

Pending (operator, at the rover): Jetson **bench validation** (OFF→ON→OFF
actuator at run boundaries via `SERVO_OUTPUT_RAW.servo9_raw`; mismatched/cleared
mission rejection on the live ROS graph; clear-forces-OFF), then field validation
of speed-adaptive flow / paint-density consistency, and sustained-run thermal
check at `PWM_AUX_MAX1=15000`. Speed-based variable-flow compensation
(`target_flow = density × speed`) remains future hardware work.

Deployment remains **NO-GO** until Jetson bench and field validation pass.

---

## Task_03 — Reliable LoRa RTK Injection Layer

### Goal

Build production-grade LoRa correction transport.

### Requirements

User manually starts LoRa via frontend API.

After start:

- connection must remain persistent
- automatic reconnect forever
- no operator intervention during mission
- reconnect on temporary signal loss
- abort only when:
  - user stops manually
  - receiver hardware disconnected
  - transmitter stops sending valid RTCM

### Core Features

- watchdog
- reconnect supervisor
- stream validation
- RTCM integrity checks
- transport state machine

### Status

**COMPLETED — 23 June 2026.**

Software implementation complete; independent production audit PASS; automated tests PASS (62/62); committed `648b054` and deployed to Jetson.

Implemented: streaming RTCM3 parser with CRC-24Q integrity checks + bounded buffer + rate limiting (`rtcm3_parser.py`), self-healing LoRa node with internal serial reconnect and module-disconnect detection (`lora_rtcm_node.py`), per-source lifecycle state machine + restart-bounded reconnect supervisor + navigation snapshot in status (`server/rtk_manager.py`), `POST /rtk/lora/stop` endpoint.

Pending (operator, at the rover): hardware bench validation, UM982 RTK FLOAT/FIXED validation, final field acceptance.

---

## Task_03.1 — NTRIP Injection Hardening

### Goal

Upgrade current NTRIP transport layer for production reliability.

### Critical Problems Found

- no TCP keepalive
- no child process restart supervisor
- stale stream detection weak
- no stream timeout forced reconnect
- no proper health monitoring

### Required Fixes

- SO_KEEPALIVE
- proactive reconnect
- stream watchdog
- child process auto-restart
- RTCM rate monitoring
- better network failure handling

### Status

**COMPLETED (code) — 23 June 2026. Pushed; real-world test + Jetson deploy PENDING.**

Software implementation complete; independent production review PASS; automated tests PASS (71/71: 23 root + 48 server); committed `67abdbe` and pushed to `origin/main`. **Not yet deployed to the Jetson and not yet validated on real hardware.**

Implemented: NTRIP lifecycle state machine (CONNECTING/STREAMING_VALID_RTCM/NO_VALID_RTCM/AUTH_FAILED/DNS_FAILED/CASTER_UNREACHABLE/PROCESS_CRASHED/…), TCP keepalive (`SO_KEEPALIVE` + idle/interval/count), configurable connect/recv/no-RTCM timeouts, last-valid-RTCM stream watchdog forcing reconnect (keepalive bytes don't count as healthy), strict RTCM3 CRC-24Q validation with metrics, RTCM rate/bandwidth monitoring, MAVROS publish wrapped (publish errors counted, node never crashes), process-group isolation + bounded child auto-restart with jittered backoff and per-minute restart throttle/cooldown, transport-vs-navigation health split, and a hardened `/api/rtk/status`. Manual-start philosophy preserved (no boot auto-start; self-heal only after operator start).

Production-review closure fixes (this push): `AUTH_FAILED` detection moved into a shared reaper used by both `status()` and the supervisor so a credential rejection becomes terminal without depending on `/api/rtk/status` polling; `destroy_node()` no longer clobbers a terminal `auth_failed` status with `stopping`; NTRIP start route maps `RTKValidationError` → HTTP 422; `NTRIP_*` config documented as boot-critical. Regression tests added (auth-after-grace-without-poll proven red-without-fix).

Pending (operator, at the rover): **deploy to Jetson (`git pull` + restart `px4-dxp`)**, then bench validation (caster-close reconnect, bad-RTCM drop, child-kill restart, MAVROS-restart survival), and field validation (RTK_FIXED, brief internet loss → reconnect, mission only stops on GPS-fix degradation).

---

## Task — Current System Production Stabilization

### Goal

Close the seven P0/P1 software blockers identified in `Current_System_Production_Stability_Audit.md`
before Jetson bench validation, without expanding the roadmap or touching unrelated subsystems.

### What Was Fixed

- **RPP freshness fail-closed.** `RppStatusMonitor.is_done()` and `is_tracking()` now require a
  fresh `/rpp/debug` snapshot within `ROVER_RPP_DEBUG_STALE_MS`. A stale or missing snapshot
  prevents auto-completion, blocks mission start, and triggers the running watchdog. Mission status
  and telemetry expose `rpp_debug_age_ms` and `rpp_debug_fresh`.

- **Safe normal completion.** `OffboardController.complete_async()` replaces the metadata-only
  `mark_completed()` call. On fresh RPP DONE, completion now: cancels spray intent, sets
  `spray_enabled=False`, waits for spray OFF confirmation, publishes a current-position stop path,
  waits for measured rest, switches to MANUAL, disarms, then calls `mark_completed()`.

- **Degraded completion is `MissionState.ERROR`.** If any terminal safety confirmation fails (spray
  OFF, measured rest, MANUAL, disarm, or second fresh-DONE check), completion transitions to
  `MissionState.ERROR` with terminal reason `mission_completion_degraded` — not an unqualified
  `COMPLETED`. Socket.IO events and mission capture reflect the degraded state.

- **E-stop no longer depends on lazy lifecycle lock.** `EmergencyHandler.estop_async()` uses
  `OffboardController._lifecycle_lock()` instead of directly entering a possibly uninitialized
  `_lock`. Physical safety commands (stop path, MANUAL, disarm) still execute before the lock is
  taken.

- **Controller-bound, staged, and published geometry reject non-finite inputs.** `PathValidator`
  hard-rejects empty paths and non-finite coordinates. All CSV readers reject missing columns,
  malformed numeric fields, and NaN/Inf. DXF parsing rejects ambiguous `$INSUNITS=0` without an
  explicit scale and unsupported entities by default. `OffboardController.load_path()` validates
  all controller-bound points. `RosBridgeNode.publish_path()` revalidates geometry even when a
  staged fingerprint is supplied — the external fingerprint does not bypass finite-coordinate
  checking.

- **Runtime fingerprints verified; surveyed-runtime transform exception documented.** `publish_path()`
  validates geometry for every runtime path, including staged JSON and point-mode shell points.
  The surveyed-runtime transform exception (the alignment-adjusted path intentionally diverges in
  ENU from the staged NED plan) is explicitly documented and retains finite-coordinate validation.

- **Unknown spray actuator backends fail closed.** `spray_controller_node.py` raises `ValueError`
  at startup and command build for any unrecognised `actuator_backend`; the previous silent OFF
  fallback after software state was marked ON is removed.

- **Mission debug capture extended.** Mandatory rosbag topics now include `/path/identity`,
  `/rpp/conditioned_path`, `/rpp/conditioned_path_identity`, `/spray/desired`, `/spray/commanded`,
  `/spray/debug`, and `/spray/runtime_status`. QoS overrides cover transient-local identity topics.
  Bundles attempt read-only MAVROS/FCU param snapshots written to `config/fcu_params.json`.

### Verification (software-only, no Jetson)

| Suite | Result |
|---|---|
| `python3 -m pytest server -q` | **396 passed** |
| `python3 -m pytest path_engine/tests -q` | **341 passed** |
| Spray suites (6 files) | **95 passed** |
| RTK/NTRIP/LoRa suites (5 files) | **42 passed** |
| Final Opus combined review | **737 passed, compile clean** |
| `git diff --check` | passed |

### Status

**SOFTWARE COMPLETE — FINAL PRODUCTION REVIEW GO. Safe to commit and deploy to Jetson for bench
validation only.**

The final Claude Opus production review found no blocking software defects. All P0/P1 source
blockers from the audit are closed and covered by tests.

### Remaining Validation (hardware — not complete, not claimed)

The following have **not** been run and are required before production road-line paint:

1. **Jetson service and restart checks** — `systemctl status`, no tracebacks on boot, `Upholds`
   cascade proven, `rpp-pipeline` restart without MAVROS drop.
2. **Live RPP freshness timing** — bench-prove stale-DONE thresholds under Jetson load; no false
   abort during normal OFFBOARD grace period.
3. **FCU/MAVROS/OFFBOARD confirmation** — live readback of `COM_OF_LOSS_T`, `COM_OBL_RC_ACT`,
   rover yaw/transition, wheel encoder, and spray AUX params; OFFBOARD stream at 50 Hz confirmed;
   OFFBOARD loss behavior observed.
4. **Physical spray OFF** — actuator OFF at boot, disable, disarm, MANUAL, OFFBOARD loss, stale
   path/status, node shutdown, and service restart; AUX1 PWM verified via QGC
   `SERVO_OUTPUT_RAW.servo9_raw`.
5. **RTK FIXED validation** — NTRIP stream reaches `/mavros/gps_rtk/send_rtcm`, GPSRAW reaches
   `RTK_FIXED`; RPP gate correctly holds in `RTK_WAIT` on fix loss.
6. **Diagnostic bundle completeness** — one dry mission must produce a complete bundle with all
   newly mandatory topics present, staged artifact hash, git status, systemd snapshot, ROS param
   dump, and FCU param export.
7. **Dry, water-only, and short paint field tests** — in that order; pass criteria: median
   cross-track ≤ 5 cm, no unintended spray during transit, no false `COMPLETED` on degraded
   terminal, and diagnostic bundle integrity complete.

---

## Task_04 — Full Production System Hardening

### Goal

Harden complete rover stack before field validation.

### Areas Covered

### FastAPI Server

- restart recovery
- API ownership
- authentication
- lifecycle safety
- transactional mission loading

### Mission System

- atomic start/load
- terminal safe states
- recovery-required states

### Safety Layer

- reliable e-stop
- hardware confirmation
- no race conditions

### Controller Layer

- stale GPS detection
- stale RTK detection
- EKF jump latching
- controller restart detection

### Deployment

- production systemd profile
- watchdog supervision
- no untracked motion
- secure LAN deployment

### Debug Layer

- complete mission bundle
- rosbag completeness
- service journal capture

### Status

Architecture finalized.

Implementation pending.

---

# Current Project Status

Production readiness:

Codebase still NOT safe for unrestricted field testing.

Large architectural weaknesses identified.

Layer 1 server audit refined on **23 June 2026** using Composer forensic verification.

Layer 2 mission lifecycle audit completed on **23 June 2026** and refined after Composer forensic verification.

Layer 3 path engine / geometry pipeline audit completed on **23 June 2026** and refined after Composer forensic verification.

Layer 4 RPP controller / motion-control audit completed on **23 June 2026**. Composer verified Red. Task_14 through Task_16 are under refinement and are not implementation-ready until this refinement pass is complete.

Layer 6 PX4 / MAVROS / vehicle authority audit completed on **23 June 2026**.

Composer verified Layer 6 on **23 June 2026** — **Production Risky / Green audit accuracy**:
Task_20 through Task_22 are directionally correct but required prerequisite refinement before implementation. A standalone Layer 6 audit report file does not currently exist; Task_20, Task_21, and Task_22 carry the Layer 6 audit evidence and Composer verification notes.

Layer 5 spray controller / actuator safety audit completed on **23 June 2026**.

Composer verified Layer 5 on **23 June 2026** — **RED/YELLOW**:
core findings confirmed; Task_17 through Task_19 required refinement before implementation.

**Layer 5 CLOSED — 29 June 2026. Task_17 through Task_19 implemented, reviewed (GO after a test-only mock-target fix), committed `35a1518`, and deployed to Jetson (`rover-server` + `rpp-pipeline`).** Spray command state is now truthful: explicit pending / accepted / off-confirmed states; `/spray/state=True` only after MAVROS accepts (no optimistic ON); ON rejection forces OFF and never claims ON; reassert is gated on accepted + not-pending. Every terminal path (disarm, set_mode, e-stop, mission stop / abort / completion, point cancel / drain `finally`, bridge / startup recovery) routes through one shared `force_spray_off_confirmed()` helper that is physical-feedback-aware. Startup reconciliation fails closed on residual spray / dwell state and blocks mission ops until reconciled. Distance-aware spray is progress-aware (along-track velocity, bounded ON/OFF leads, speed window, geometry-identity check); point dwell cancel forces OFF in a non-skippable `finally` bound to parent mission identity; dash boundaries are exact / validated. Telemetry is fail-closed (stale runtime suppresses `spraying` + `marking_state=marking`) with explicit `spray_*` / `point_*` fields. Tests: server **490 passed**, spray src **179 passed**, Task_17 closure **57 passed**. Post-deploy spray state verified safe (`ACCEPTED_OFF`, not faulted, runtime fresh).

> **Field gate:** physical actuator feedback is unsupported (`physical_feedback_supported=false`) — OFF is reported as **command-acknowledged only**, never physical proof. The **first live run must be a dry-run (no paint)** to validate the state machine and confirmed-OFF terminal paths on hardware.

**Backend Mission-Mode Gaps (Legacy NRP_ROS → PX4_DXP migration) — IMPLEMENTED + PUSHED 30 June 2026, FIELD-VALIDATION PENDING.** Closes the legacy-vs-current backend capability gaps catalogued in `NRP_ROS_vs_PX4_DXP_Backend_Capability_Gap_Audit.md` and the 7-changeset plan in `PX4_DXP_Mission_Mode_Gaps_Implementation_Plan.md`. Commit `a31f36d`: priority operation coordinator (`mission_ops.py`), single idempotent `terminal_cleanup()` with confirmed spray-OFF on every terminal path, Point-owns-completion split, REST/Socket parity service layer (`mission_services.py`), bounded Point event journal (`point_events.py` + `point_mission_event` + `GET /api/mission/point/events`), **operator Point skip** (`POST /api/mission/point/skip`) and **mission restart** (`POST /api/mission/restart`). Follow-on backend GCS-migration surfaces: activity CSV (`GET /api/activity.csv`), AsyncAPI (`GET /api/docs/asyncapi`), read-only `GET /api/nodes` + `GET /api/network`, GCS migration matrix (`396ccd2`/`a784ca0`/`a5d63ee`/`58b0200`), plus event-loop-safety + cwd-safe-test fix (`5d65b5e`). Tests: 47 new + full server suite green (1 pre-existing DXF-fixture env failure). **Still NO-GO:** Continuous/Dash pause-resume (Phase-4 product gate), stop-to-MANUAL (opt-in, not built). **PENDING (next session):** Jetson was OFFLINE at push — deploy (`git pull` + restart `rover-server`) then run the spray-disabled dry-run → spray bench → low-speed field validation per `02_Jetson_Post_Deployment_Validation_Prompt.md`. Genuinely-open backend items now: GPS-failsafe operator-ack API, continuous/dash pause + event-journal extension, `I-T4` configurable xtrack CSV path, `I-T2` dead-launch cleanup; TTS / RTK `force_clear` deferred-by-decision; spray P3-T1/T2/T8/T11/T12 hardware/QGC.

Layer 7 multi-node system integration audit completed on **23 June 2026**.

Layer 7 verdict: **RED / not field-test ready as an integrated robot**.
Task_23 through Task_25 are required to prevent split-brain ownership, stale ROS topic replay, partial process restart hazards, and uncoordinated recovery across motion, spray, RTK, MAVROS, and capture.

Composer verified Layer 7 on **23 June 2026**.

Composer verdict: **RED**. Audit accurate. Missing additions integrated:

- `conditioned_path` replay
- path non-revocation
- emergency lock bug
- point mission COMPLETED bypass
- RPP death TRACKING blind spot

Layer 8 diagnostics / logging / blackbox capture audit completed on **23 June 2026**.

Layer 8 verdict: **RED / production validation invalid without fixes**.
v1 mission capture is deployed: `bag-autorecord.service` exists, and `rover-server.service` sets `MISSION_CAPTURE_REQUIRED=1` to gate mission start on recorder readiness. The blocker is v2 identity/replay readiness: mid-mission capture liveness, correlation IDs, full runtime geometry, topic completeness, retention protection, and offline replay are not production-ready. Task_26 through Task_28 are required before production field validation can rely on mission evidence, replay, or failure bundles.

Composer verified Layer 8 on **23 June 2026** — **RED / directionally accurate audit**.
Composer refinements incorporated:

- STATUSTEXT `/recv` is unverified; actual Jetson topic must be discovered before hardcoding.
- missing `/spray/desired` topic coverage added.
- mid-mission capture loss added as production-critical evidence/liveness risk.
- frozen RPP telemetry added: stale `RppStatusMonitor` snapshots can make final status look healthy.
- bundle retention evidence loss added.

Layer 9 configuration / parameter governance audit completed on **23 June 2026**.

Layer 9 verdict: **RED / not production-governed**.
Task_29 through Task_31 are required to prevent production/dev profile drift, unsafe parameter mutation, unverified PX4/RPP/spray profiles, missing config hashes, and field runs without reproducible provenance.

Composer verified Layer 9 on **23 June 2026**.

Composer verdict: **RED system verdict / GREEN audit accuracy**. Refinement required from Composer report and integrated into Layer 9 audit/tasks:

| Missing risk / refinement | Primary Task | Why it blocks field validation |
|---|---|---|
| Unauthenticated `/api/telemetry/latest`, `/api/healthz`, and Socket.IO telemetry exposure | Task_29 | Weak/off auth can expose pose, GPS quality, armed/mode, and mission state even when command mutation is later restricted. |
| In-repo PX4 param baseline mismatch | Task_30 | `params/Param_with_Roboclaw.params` already differs from validated spray AUX documentation, so it cannot be treated as production-approved as-is. |
| Planner speed vs execution speed split | Task_30, Task_31 | Planner `marking_speed` and RPP `mission_speed` are separate knobs; field runs must show which speed governed estimates and which governed execution. |
| `git_commit` placeholder provenance gap | Task_31 | Existing bag provenance has staged artifact SHA-256, but null software commit/config/param/service hashes leave runs non-reproducible. |

Layer 10 deployment / systemd / process supervision audit completed on **23 June 2026**.

Layer 10 verdict: **RED / not production-reliable as a supervised field runtime**.
Task_32 through Task_34 are required to prevent false-green systemd states, boot-readiness races, child-process restart split-brain, watchdog blind spots, resource/log exhaustion, live symlink update risk, missing rollback, and unsafe service exposure.

Composer verified Layer 10 on **23 June 2026** — **RED system verdict / GREEN audit accuracy**.

Composer refinements incorporated:

- `bag-autorecord` dies on `px4-dxp` restart due to `PartOf` without `Upholds`.
- `deploy.sh --restart` omits bag restart.
- RTK child crash is not auto-restarted.
- MAVROS post-ready hang is not detected after initial node discovery.
- `rotate_bundles()` can delete incomplete/abandoned evidence.
- `server/run.sh` misdocuments `Type=notify`.
- `docs/Progress/PROGRESS.md` deployment section is stale.
- No `Requires=` exists in production units.

Layer 10 audit generated exactly three deployment hardening tasks:

| Task | Scope | Why it blocks field validation |
|---|---|---|
| Task_32 | Systemd service topology and boot readiness | MAVROS/RPP/server/bag services must start and recover as one coherent robot runtime, not as active-but-degraded processes. |
| Task_33 | Process watchdog, resources, and log retention | The field runtime must detect hangs, not only death, and must preserve evidence under restart, disk pressure, and resource contention. |
| Task_34 | Deployment, rollback, and service security | Field updates must be atomic, rollback-capable, health-validated, and locked to the approved production profile. |

New blocking server-layer sequence before field validation:

- Task_05 - Server Control Authority Hardening
- Task_06 - API Permissions and Production Auth
- Task_07 - Server Restart Reconciliation
- Task_08 - Mission Lifecycle State Machine Hardening
- Task_09 - Transactional Mission Load and Start
- Task_10 - Point Mission Lifecycle Hardening

New blocking geometry-layer sequence before field validation:

- Task_11 - Path Engine Geometry Validation Hardening
- Task_12 - DXF / CSV Parsing Contract Hardening
- Task_13 - Preview / Staged / Runtime Geometry Consistency

New blocking motion-control layer sequence before field validation:

- Task_14 - Runtime Safety / Liveness Hardening
- Task_15 - Mission Lifecycle / OFFBOARD Safety
- Task_16 - Trajectory Contract / Speed Correctness

Spray actuator-safety sequence (**COMPLETE — deployed `35a1518`, 29 June 2026**):

- Task_17 - Spray State Machine and Actuator Acknowledgement ✅
- Task_18 - Spray Runtime Distance and Flow Control Hardening ✅
- Task_19 - Point Dwell and Dash Spray Mode Hardening ✅

New blocking PX4/MAVROS authority sequence before field validation:

- Task_20 - PX4 Mode and Arming Authority Hardening
- Task_21 - MAVROS FCU Disconnect and Recovery Hardening
- Task_22 - Offboard Setpoint Stream and Vehicle Failsafe Hardening

New blocking multi-node integration sequence before field validation:

- Task_23 - Multi-Node Ownership and Health Model
- Task_24 - ROS2 QoS Stale Topic and Restart Hardening
- Task_25 - System Recovery Coordination Hardening

New blocking diagnostics-layer sequence before production field validation:

- Task_26 - Mission Debug Bundle V2
- Task_27 - Rosbag and Runtime Topic Coverage
- Task_28 - Production Log Correlation and Replay Readiness

New blocking configuration-governance sequence before field validation:

- Task_29 - Production Profile and Config Lockdown
- Task_30 - PX4 / RPP / Spray Parameter Governance
- Task_31 - Config Hash / Drift Detection / Run Provenance

New blocking deployment/supervision sequence before field validation:

- Task_32 - Systemd Service Topology and Boot Readiness
- Task_33 - Process Watchdog / Resource / Log Retention Hardening
- Task_34 - Deployment / Update / Rollback / Service Security

# Layer 4 Dependency Order

Task_14, Task_15, and Task_16 depend on Task_05, Task_07, Task_08, Task_09, and Task_13.

Task_14 must land before field execution can trust RPP/GPS/DDS/setpoint liveness, stale debug detection, intermittent EKF jump recovery, or telemetry truthfulness.

Task_15 depends on Task_14 liveness signals and Task_08 terminal FSM semantics. Task_15 completion safety also depends on Task_17 spray terminal behavior and Task_10 point orchestrator drain.

Task_16 depends on Task_13 geometry identity and must preserve the speed/trajectory contract while coordinating first-motion and terminal behavior with Task_15.

Layer 4 status: Composer verified Red. Refinement required / in progress. Do not begin implementation from Task_14 through Task_16 until this refinement pass is complete.

# Layer 6 Dependency Order

Task_20 must land before any field mission can trust mode, arming, disarming, abort, e-stop, completion, or manual takeover status.

Task_21 depends on Task_20 confirmation semantics and adds bridge/session recovery, startup reconciliation, and authority-loss latching.

Task_22 depends on Task_20 and Task_21 so OFFBOARD setpoint ownership, stream freshness, and PX4 parameter safety can be tied to confirmed hardware and recovery state. Task_22's first milestone is observe-only setpoint stream health instrumentation; enforcement must wait until Task_20 hardware confirmation and Task_22 stream heartbeat evidence are both validated.

Task_20 depends on Task_05's authority gate design. Task_21 depends on Task_20's confirmed arm/mode helpers. Task_22 depends on Task_20/21 and must not be treated as implementation-ready until the observe-only stream-status milestone is complete.

Layer 6 Composer missing-risk refinements:

| Risk | Primary Task | Why it blocks field validation |
|---|---|---|
| `/api/path/publish` motion bypass | Task_20, Task_22, cross-reference Task_05 | Path publish can become motion if PX4 remains armed/OFFBOARD while server appears `IDLE`. |
| Emergency lock bug | Task_20, cross-reference Task_05 | `emergency.py` uses raw `_controller._lock`, which may be `None` before lifecycle lock initialization. |
| Point-shell RPP gate gap | Task_22, cross-reference Task_10 | Point shell can request OFFBOARD without the same RPP readiness proof as continuous start. |
| RPP death/frozen telemetry | Task_21, Task_22 | Cached RPP state can remain healthy while the setpoint streamer hides upstream failure with zero output. |
| Open PX4 parameter writes | Task_22, cross-reference Task_05/06 | `/api/params/{name}` can mutate safety-critical FCU params without field-profile allowlist/readback/gating. |

Blocking field-validation order:

1. Task_05 through Task_10 - server and mission authority baseline.
2. Task_11 through Task_13 - geometry consistency gates before production mission execution.
3. Task_14 through Task_16 - RPP/controller motion-control, OFFBOARD lifecycle, trajectory/speed correctness.
4. Task_20 - confirmed PX4 mode/arming authority.
5. Task_21 - MAVROS/FCU disconnect and recovery latching.
6. Task_22 - OFFBOARD setpoint stream and PX4 failsafe hardening.
7. Task_23 - authoritative multi-node ownership and health model.
8. Task_24 - ROS2 stale-topic, QoS replay, and restart hardening.
9. Task_25 - coordinated system recovery and operator acknowledgement.
10. Task_26 - mission debug bundle identity, hashes, metadata, snapshots, and terminal reason schema.
11. Task_27 - mandatory rosbag/runtime topic coverage and transition timelines.
12. Task_28 - exportable correlated bundle and offline replay/analyzer readiness.
13. Task_29 - production profile and config lockdown.
14. Task_30 - PX4, RPP, and spray parameter governance.
15. Task_31 - config hash, drift detection, and run provenance.
16. Task_32 - systemd service topology and boot readiness.
17. Task_33 - process watchdog, resource, and log retention hardening.
18. Task_34 - deployment, update, rollback, and service security.

New blocking spray-layer sequence before paint field validation:

- Task_17 - Spray State Machine and Actuator Acknowledgement
- Task_18 - Spray Runtime Distance and Flow Control Hardening
- Task_19 - Point Dwell and Dash Spray Mode Hardening

Layer 5 dependency notes:

- Task_17 server terminal spray OFF is shared with Task_05 control authority; node-only command-state work may start independently, but terminal path integration must use the shared authority gate.
- Task_18 depends on Task_17 acknowledgement truth. Variable flow control must wait for truthful command-accepted/fault states.
- Task_19 depends on Task_10 point lifecycle and Task_17 `force_spray_off_confirmed()` helper.

Recommended dependency order:

1. Complete Task_05, Task_06, and Task_07 for authority, auth, and restart recovery.
2. Complete Task_08, Task_09, and Task_10 for mission/point lifecycle safety.
3. Complete Task_11, Task_12, and Task_13 for geometry and spray-flag truth.
4. Complete Task_17 so spray command/actuator status is truthful.
5. Complete Task_18 so runtime distance and flow control are field-safe.
6. Complete Task_19 so point dwell and dash modes inherit the hardened spray/mission contracts.

Layer 7 dependency order:

1. Task_23 after Task_05, Task_07, Task_08, Task_14, Task_17, Task_20, Task_21, and Task_22. Task_23 must explicitly include the `point_mission` child ownership model.
2. Task_24 after Task_23, coordinated with Task_13, Task_17, Task_21, and Task_22.
3. Task_25 after Task_23 and Task_24, and after Task_15, Task_17, Task_20, Task_21, and Task_22 terminal/recovery foundations are available.

Task_23 through Task_25 planning is complete. Implementation is blocked until lower layers complete:

- Task_05
- Task_07
- Task_08
- Task_14
- Task_17
- Task_20
- Task_21
- Task_22

Layer 8 dependency order:

1. Task_26 after Task_08, Task_09, and Task_13 for mission identity and geometry consistency, coordinated with Task_17, Task_20, Task_21, Task_23, Task_24, Task_25, Task_03, and Task_03.1 for truthful spray, PX4/MAVROS, system-health, stale-topic, recovery, and RTK evidence. Task_26 can start in parallel with Task_22's observe-only setpoint stream health work because it primarily defines bundle schema and evidence contracts.
2. Task_27 after Task_26 because expanded topic coverage and timelines must write into the v2 bundle identity and integrity schema, coordinated with Task_17, Task_21, Task_22, Task_23, Task_24, Task_25, Task_03, and Task_03.1.
3. Task_28 after Task_26 and Task_27, and after Task_05, Task_06, Task_07, Task_08, Task_20, Task_21, Task_22, Task_23, Task_24, and Task_25 so exported bundles carry operator identity, restart recovery truth, mission terminal truth, PX4 authority truth, setpoint ownership evidence, system-health evidence, and coordinated recovery records.

Task_23 through Task_25 frozen telemetry and coordinated recovery truth must land before Task_28's replay verdict is meaningful. A replay package cannot be trusted if upstream system health can still report stale RPP/spray/bridge state or collapse recovery causes into generic terminal events.

Layer 8 implementation status: audit and task generation complete. Implementation pending.

Layer 9 dependency order:

1. Task_29 after Task_05 and Task_06 so profile lockdown uses the shared authority, auth, route-policy, and diagnostic-lease model.
2. Task_30 after Task_29, coordinated with Task_14 through Task_16, Task_17 through Task_19, and Task_20 through Task_22.
3. Task_31 after Task_29 and Task_30, coordinated with Task_09, Task_13, Task_18, and mission-debug capture.

Layer 9 implementation status: audit and task generation complete. Implementation pending.

Layer 10 dependency order:

1. Task_32 after Task_07, Task_21, Task_23, Task_24, Task_25, and Task_29 so deployment topology and boot readiness inherit restart reconciliation, MAVROS recovery, multi-node ownership, stale-topic policy, coordinated recovery, and production profile gates.
2. Task_33 after Task_32, coordinated with Task_22, Task_26, Task_27, Task_28, and Task_31 so watchdog/resource/log policies align with OFFBOARD stream health, mission bundles, topic coverage, replay, and provenance.
3. Task_34 after Task_29, Task_30, Task_31, Task_32, and Task_33 so release/rollback/security validation uses the approved production profile, parameter baselines, service topology, watchdog health, and run provenance.

Layer 10 implementation status: Composer verified; refinement integrated; implementation pending.

---

# Next Pending Tasks

---

## Task_05 - Server Control Authority Hardening

### Goal

Make FastAPI the single safe control authority for motion, mode, arming, path ownership, spray commands, and runtime mutation.

### Source Audit

`Layer_1_Audit_Server_FastAPI_Control_Authority.md`

### Critical Work

- e-stop must be preemptive, idempotent, point-aware, spray-aware, and hardware-confirmed
- telemetry watchdog safety abort must use the same hardened e-stop path
- direct arm/mode APIs must be blocked outside mission or admin diagnostic lease
- terminal mission states must force spray OFF, leave OFFBOARD, disarm, and clear runtime ownership
- bridge recovery and point mission start failure must not fall back to unsafe soft stop
- diagnostic path publish must require disarmed admin diagnostic mode
- REST and Socket.IO control behavior must be unified
- staged mission load must be transactional
- one lifecycle authority lock must replace split `_load_lock`, `_lifecycle_lock`, and ad-hoc route checks

### Status

Layer 1 audit completed and refined with Composer verification.

Implementation pending.

---

## Task_06 - API Permissions and Production Auth

### Goal

Make production LAN/mobile API exposure safe.

### Critical Work

- remove auth-disabled field default
- fail startup on unsafe auth-disabled LAN binding
- add scoped permissions and admin diagnostic leases
- harden Socket.IO authentication and telemetry exposure
- add socket telemetry and watchdog safety event permission scopes
- restrict CORS in field profile
- add operator identity and reason audit trail for all mutations

### Status

Layer 1 audit completed and refined with Composer verification.

Implementation pending.

---

## Task_07 - Server Restart Reconciliation

### Goal

Prevent a restarted server from declaring IDLE while live hardware still has motion, spray, RTK, or OFFBOARD authority.

### Critical Work

- inspect live FCU/RPP/spray/RTK state on startup
- reconcile point mission orchestrator and spray keepalive/task state
- add `RECOVERY_REQUIRED`
- block load/start/publish/delete/artifact mutation/spray/param mutation while recovery is required
- make abort/e-stop act on live hardware even when server state is IDLE
- expose `recovery_required` and hardware/server divergence in mission status
- reconcile bridge auto-recovery and orphan RTK processes

### Status

Layer 1 audit completed and refined with Composer verification.

Implementation pending.

---

## Task_08 - Mission Lifecycle State Machine Hardening

### Goal

Make mission lifecycle behavior one deterministic state machine from staged artifact through load/start/run/terminal/clear/reload.

### Source Audit

`Layer_2_Audit_Mission_Lifecycle_State_Machine.md`

### Dependencies

- Depends on Task_05 central authority gate and lifecycle serialization.
- Depends on Task_07 recovery-required and hardware/server divergence model.
- Coordinates with Task_09 transactional load/start.
- Coordinates with Task_10 point mission child lifecycle.

### Critical Work

- add explicit loaded/starting/completing/safe-terminal/recovery states
- stop overloading `IDLE` for loaded, empty, stopped, and unknown hardware states
- make completion a hardware-confirmed terminal flow, not metadata-only
- unify REST, Socket.IO, emergency, watchdog, bridge, and completion command semantics
- make clear reconcile hardware, point mission, spray, capture, and resident identity
- expand mission status truthfulness

### Status

Layer 2 audit completed. Composer refinement required and incorporated. Implementation pending.

---

## Task_09 - Transactional Mission Load and Start

### Goal

Make staged mission load and mission start atomic across artifact identity, spray config, point orchestrator, controller resident path, capture, RPP readiness, arm, and OFFBOARD.

### Source Audit

`Layer_2_Audit_Mission_Lifecycle_State_Machine.md`

### Dependencies

- Depends on Task_05 shared lifecycle lock/authority gate.
- Depends on Task_08 explicit mission FSM states.
- Coordinates with Task_10 point-mode transactional start.

### Critical Work

- enforce staged artifact TTL at load
- verify staged JSON mission id matches requested id
- compute and expose artifact hash
- roll back spray/point/controller mutations on staged load failure
- make continuous no-spray degraded load explicit
- make start rollback physically safe after any partial arm/OFFBOARD/path publish
- close load/start/stop/abort/clear races under one lifecycle lock

### Status

Layer 2 audit completed. Composer refinement required and incorporated. Implementation pending.

---

## Task_10 - Point Mission Lifecycle Hardening

### Goal

Make point missions a first-class child state machine owned by the parent mission lifecycle.

### Source Audit

`Layer_2_Audit_Mission_Lifecycle_State_Machine.md`

### Dependencies

- Depends on Task_08 parent mission FSM.
- Depends on Task_09 transactional point-mode load/start.
- Coordinates with Task_01 point navigation production hardening.

### Critical Work

- propagate point background failures to parent mission terminal/recovery state
- prevent RPP DONE from completing parent while point mission still has points/dwell
- make stop/abort/e-stop/clear/reload cancel point task and dwell through one path
- clear stale point loaded/ready status on mission clear and non-point reload
- expose point status in mission status, not only spray status
- add restart/recovery behavior for unrecoverable point task ownership

### Status

Layer 2 audit completed. Composer refinement required and incorporated. Implementation pending.

---

## Task_11 - Path Engine Geometry Validation Hardening

### Goal

Make path geometry validation reject unsafe, non-finite, ambiguous, or physically invalid plans before preview, staging, publishing, or controller load.

### Source Audit

`Layer_3_Audit_Path_Engine_Geometry_Pipeline.md`

### Dependencies

- Depends on Task_05 authority gate for production route enforcement.
- Depends on Task_08 and Task_09 for staged artifact load/start enforcement.

### Critical Work

- hard-error non-finite coordinates, empty paths, waypoint/spray flag mismatch, huge bbox, huge gaps/connectors, invalid curves, and unsafe spacing
- add staging write-time validation for waypoint/flag cardinality and non-finite staged geometry before JSON write
- validate PRE/AFT and connector continuity, length, spacing, and spray OFF flags
- validate MARK/TRANSIT run invariants and point-mode placeholder no-spray behavior
- define alignment confidence policy for single-point, two-point, and exactly determined RMSE=0 fits
- define optimizer-on/off explicit TRANSIT preservation policy
- measure and bound merge de-duplication effects
- convert field-critical validator warnings into production errors
- add pathological CSV/DXF/connector/extension tests

### Status

Layer 3 audit completed. Composer refinement required and incorporated. Implementation pending.

---

## Task_12 - DXF / CSV Parsing Contract Hardening

### Goal

Make file ingest deterministic, explicit, and operator-visible so CAD/CSV mistakes cannot silently change field geometry.

### Source Audit

`Layer_3_Audit_Path_Engine_Geometry_Pipeline.md`

### Dependencies

- Depends on Task_11 shared finite/geometry validation.
- Coordinates with Task_13 parser reports and geometry hashes.
- Coordinates with Task_01/Task_10 only for point mission mark/navigate fields.

### Critical Work

- replace silent path CSV row skipping with strict line-numbered parse errors
- document and harden current old CSV 3/4/5-column, missing-east, invalid-spray, and speed-column behavior
- strictly validate QGC `.waypoints` for production or production-disable it
- reject missing columns, invalid spray values, non-finite coordinates, invalid speeds, and unsafe duplicates
- require explicit unit confirmation for DXF `$INSUNITS=0` / missing units in production planning
- expose unsupported/skipped DXF entities and block unacknowledged missing drawable geometry, while accurately reporting already-supported INSERT sub-entities
- include layer-ignored counts, unified length metadata, entity counts, unit scale, bbox, layer, color, and parser report metadata
- keep point mission CSV stricter and separate unless intentionally unified

### Status

Layer 3 audit completed. Composer refinement required and incorporated. Implementation pending.

---

## Task_13 - Preview / Staged / Runtime Geometry Consistency

### Goal

Guarantee the operator-previewed geometry is the exact geometry staged and loaded for runtime.

### Source Audit

`Layer_3_Audit_Path_Engine_Geometry_Pipeline.md`

### Dependencies

- Depends on Task_11 validation hardening.
- Depends on Task_12 parser reports and unit/source metadata.
- Depends on Task_09 artifact identity and transactional load.

### Critical Work

- add canonical geometry hashes over waypoints, spray flags, segment roles, source entity order, extensions, optimizer settings, alignment, source hash, and sidecars
- expose source/sidecar/parser/alignment/final geometry hashes in preview, segments, plan, staged JSON, and loaded-path status
- distinguish `/entities` edit view, `preview_path` local planned polyline, and `/plan` or `/segments` authoritative pre-stage output
- add `is_final_geometry` and `geometry_hash` semantics to each artifact
- make final planned segments the production preview authority and document `/segments` local-NED vs staged GPS-aligned divergence
- remove silent spray flag fallback in production paths, including `path_publisher_node`
- preserve explicit TRANSIT geometry or reject mixed optimized missions until a corridor-aware policy exists
- stage write-time geometry hash and invariant summaries: bbox, max gap, max connector length, MARK/TRANSIT runs, segment summary, point hash
- make `GET /staged` reject flag/count mismatch instead of silently returning empty runs
- require loaded-path summary to expose `geometry_hash`

### Status

Layer 3 audit completed. Composer refinement required and incorporated. Implementation pending.

---

## Task_14 - Runtime Safety / Liveness Hardening

### Goal

Make runtime control-loop liveness fail closed across RPP, GPS, DDS, pose, velocity, debug telemetry, and setpoint streams.

### Source Audit

`Layer_4_Audit_RPP_Controller_Motion_Control_Safety.md`

### Dependencies

- Depends on Task_05, Task_07, Task_08, Task_09, and Task_13.
- Coordinates with Task_15 for OFFBOARD terminal safety behavior.

### Critical Work

- consume independent MAVROS freshness in the RUNNING watchdog: `local_pose_age_ms`, `gps_fix_age_ms`, and velocity age when available
- track `/rpp/debug` receive age and treat stale debug as unsafe even if the last state was `TRACKING`
- detect RPP death when `twist_to_setpoint` is only preserving OFFBOARD with zero setpoints
- add GPSRAW freshness so stale RTK_FIXED cannot remain latched
- latch recovery for repeated intermittent `JUMP_SKIP` events
- expose `CORNER_STOP` and `CORNER_ALIGN` as non-normal motion substates, not ordinary healthy `TRACKING`

### Status

Layer 4 Composer verified Red. Refinement required / in progress. Implementation pending after this pass is complete.

---

## Task_15 - Mission Lifecycle / OFFBOARD Safety

### Goal

Make start, stop, abort, completion, watchdog abort, and restart behavior hardware-confirmed and physically safe.

### Source Audit

`Layer_4_Audit_RPP_Controller_Motion_Control_Safety.md`

### Dependencies

- Depends on Task_05, Task_07, Task_08, Task_09, Task_13, and Task_14.
- Completion safety also depends on Task_17 spray terminal behavior and Task_10 point orchestrator drain.

### Critical Work

- document the current start sequence: path publish, arm, grace, OFFBOARD, then service ACK trusted as RUNNING
- do not mark RUNNING until PX4 live state confirms OFFBOARD and required arm state
- require fresh pose for `publish_stop_path()`, `abort_async()`, and emergency e-stop stop-path publication
- actively leave OFFBOARD and disarm on completion/abort when production terminal policy requires it
- do not rely on PX4 offboard-loss failsafe while the zero setpoint stream is intentionally keeping OFFBOARD alive
- coordinate `mark_completed()` with Task_17 spray OFF, Task_10 point drain, and Task_08 terminal FSM

### Status

Layer 4 Composer verified Red. Refinement required / in progress. Implementation pending after this pass is complete.

---

## Task_16 - Trajectory Contract / Speed Correctness

### Goal

Guarantee the operator-approved trajectory and speed contract match what RPP executes.

### Source Audit

`Layer_4_Audit_RPP_Controller_Motion_Control_Safety.md`

### Dependencies

- Depends on Task_05, Task_07, Task_08, Task_09, Task_13, Task_14, and Task_15.

### Critical Work

- use existing `/rpp/conditioned_path` as the starting point for a hashable runtime-conditioned artifact
- define whether planner segment speeds are executable or whether global `mission_speed` owns runtime speed
- ensure operator-facing metadata reflects actual executed speed ownership
- preserve MARK, TRANSIT, PRE, AFT, connector, runtime-entry roles through conditioning
- add first-run heading alignment while preserving BUG-T3 forward-cone clamp behavior
- narrow overshoot work to false DONE, post-jump wrong-side projection, residual endpoint overshoot, and corner-stop/align telemetry truth

### Status

Layer 4 Composer verified Red. Refinement required / in progress. Implementation pending after this pass is complete.

---

## Future Field Validation Gate - Full Field Validation Protocol

Design full production field test protocol.

This is intentionally unnumbered to avoid colliding with Layer 4 Task_14. It must come after Task_05 through Task_22 and later-layer audits/tasks that may add additional blockers.

Must validate:

### Mission Pipeline

- upload
- parse
- alignment
- staging
- mission load
- mission start
- mission stop
- mission abort
- mission clear

### RTK Validation

- NTRIP reconnect tests
- LoRa reconnect tests
- temporary network failure tests
- RTK degradation tests
- FIXED → FLOAT transition tests

### Rover Motion Validation

- trajectory accuracy
- x-track error
- runtime entry path
- stop accuracy
- corner behavior
- point mission dwell behavior

### Spray Validation

- ON/OFF latency
- flow consistency
- density consistency
- speed variation behavior
- physical actuator acknowledgement

### Failure Injection Tests

- kill server during mission
- restart MAVROS
- restart RPP
- restart spray controller
- stale GPS
- stale pose
- FCU disconnect
- telemetry disconnect

### Logging Validation

- bag integrity
- debug bundle completeness
- capture finalization
- journal preservation

---

## Future Validation Gate - Production Simulation Stress Testing

Before real field testing.

Need large repeated automated tests.

Examples:

- 100 mission continuous run
- repeated start/stop cycles
- network disconnect simulation
- process crash simulation
- ROS2 restart simulation
- MAVROS restart simulation

---

## Future Deployment Gate - Production Deployment Profile

Finalize deployment configuration.

Includes:

- systemd
- watchdogs
- authentication
- LAN security
- process priority
- CPU affinity
- thermal management
- startup dependency chain

---

# Current Rule

NO FIELD DEMO

until Tasks 01 through 31 are implemented and tested, Layer 4 Task_14 through Task_16 are complete, later-layer blockers are resolved, and the unnumbered field validation protocol is approved.

Geometry-layer, motion-control, vehicle-authority, multi-node integration, and configuration-governance implementation remain field-validation blockers. Spray actuator-safety (Task_17 through Task_19) is implemented and deployed (`35a1518`, 29 June 2026) but still requires hardware dry-run validation before paint. Task_11 through Task_31 (excluding the deployed spray layer) are refined planning tasks, not runtime readiness evidence.

---

# Next Plan

Continue with:

**Task_05 - Server Control Authority Hardening**

Focus:

"Make the FastAPI server unable to report safe control unless live hardware is actually safe."

Then continue mission-layer hardening in order:

1. Task_08 - Mission Lifecycle State Machine Hardening
2. Task_09 - Transactional Mission Load and Start
3. Task_10 - Point Mission Lifecycle Hardening

Then continue geometry-layer hardening in order:

1. Task_11 - Path Engine Geometry Validation Hardening
2. Task_12 - DXF / CSV Parsing Contract Hardening
3. Task_13 - Preview / Staged / Runtime Geometry Consistency

---

End of Session  
23 June 2026
