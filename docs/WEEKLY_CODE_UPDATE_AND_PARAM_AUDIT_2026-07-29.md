# Weekly Code Update, Feature & Parameter Audit (2026-07-22 to 2026-07-29)

> **Audit Period:** 2026-07-22 00:00:00 to 2026-07-29 12:00:00 (Local Time)  
> **Total Commits Audited:** 100 commits  
> **Code Implementation Commits:** 79 commits  
> **Documentation / Skill Commits:** 21 commits  
> **New Code Files Created:** 42 files  

---

## 1. Executive Summary

This document captures the complete code updates, structural implementations, bug fixes, parameter switches, and feature flag defaults landed in `PX4_DXP` from Wednesday July 22, 2026 to Wednesday July 29, 2026.

Major architectural milestones achieved during this period include:
1. **Server Safety & Resilience:** Systemd notify watchdog integration, callback group isolation, and ROS-degraded heartbeat output.
2. **Trajectory Planning & Survey Ingest:** Staging densified trajectories via `POST /api/path/plan-trajectory`, curvature-based arc fitting for Emlid survey CSVs, and Douglas-Peucker vertex provenance preservation.
3. **RPP ↔ Spray Handshake (Phase B–G):** ROS 2 progress handshake topics (`/rpp/progress`, `/rpp/milestone`), speed-proportional spray flow model (§7.5), 2 cm precision point stopping, and RTK/GPS fix-quality gating (§7.6).
4. **Controller Tuning & Frame Validation:** Gated signed-lateral smooth profile corrections (B2 gains landed as defaults), EKF local-frame origin validation, and joystick manual control responsiveness improvements.

---

## 2. New Code & Test Files Created (42 Files)

### Path Engine & Planners (`path_engine/`)
* `path_engine/parsers/survey_csv.py` — Emlid and survey CSV parser with control point extraction.
* `path_engine/planners/arc_chain.py` — Curvature-segmented arc fitting for survey line chains.
* `path_engine/planners/corner_fillet.py` — Corner filleting generator for survey lines lacking explicit arcs.
* `path_engine/tests/test_arc_chain.py` — Unit tests for arc chain fitting.
* `path_engine/tests/test_close_shape.py` — Unit tests for closed shape painting logic.
* `path_engine/tests/test_corner_fillet.py` — Unit tests for corner filleting.
* `path_engine/tests/test_survey_csv.py` — Unit tests for survey CSV parsing.
* `path_engine/tests/test_validator_wet_paint_index.py` — Unit tests for spatial indexing of wet paint checks.
* `path_engine/tests/test_vertex_provenance.py` — Unit tests for preserving original surveyed lat/lon provenance.

### Rover Server & Health Architecture (`server/`)
* `server/origin_health.py` — EKF local-frame origin freshness monitor & fail-closed origin gates.
* `server/spray_session_builder.py` — Session configuration builder for spray controller transport.
* `server/test_systemd_watchdog.py` — Integration tests for systemd `sd_notify` watchdog.
* `server/test_plan_trajectory.py` — Integration tests for trajectory planning endpoint.
* `server/test_origin_health.py` — Unit tests for origin health verification.
* `server/test_origin_invalidation.py` — Tests for origin invalidation behavior.
* `server/test_mission_start_identity.py` — Verification tests for `/start` mission ID matching.
* `server/test_spray_session_builder.py` — Unit tests for spray session builder.
* `server/test_rtk_manager.py` — Unit tests for RTK connection management.

### ROS Node Pipeline & Spray Controller (`src/`)
* `src/mission_progress.py` — Core mission progress classification & tracking logic.
* `src/mission_progress_ros.py` — ROS 2 wrapper publishing `/rpp/progress` and `/rpp/milestone`.
* `src/point_ingest.py` — Point mission CSV ingestion and must-hit waypoint staging.
* `src/precise_stop.py` — 2 cm point-hold precision stop controller.
* `src/progress_classifier.py` — Real-time path section and landmark classifier.
* `src/spray_flow_model.py` — Speed-proportional spray valve flow rate calculator.
* `src/spray_modes.py` — State machine for spray modes (Line, Point, Handshake).
* **Test Modules in `src/`:**
  * `src/test_ekf_reset_compensation.py`
  * `src/test_mission_progress.py`
  * `src/test_point_handshake.py`
  * `src/test_point_handshake_rpp.py`
  * `src/test_point_handshake_spray.py`
  * `src/test_point_hold_rpp.py`
  * `src/test_point_ingest.py`
  * `src/test_precise_stop.py`
  * `src/test_precise_stop_node.py`
  * `src/test_progress_classifier.py`
  * `src/test_progress_publication.py`
  * `src/test_spray_dash_v2.py`
  * `src/test_spray_flow_model.py`
  * `src/test_spray_point_v2.py`
  * `src/test_spray_rpp_boundary.py`
  * `src/test_spray_rtk_gate.py`

### Tools
* `tools/render_open_bugs.py` — CLI bug register rendering utility.

---

## 3. Parameter Switches, Feature Flags & Default Configuration Matrix

| Feature / Parameter | Subsystem | Default State / Value | Category | Description |
|---|---|---|---|---|
| `require_rtk_fix` | Safety / ROS | **Default ON** (`True`) | Feature Flag | RTK/GPS fix-quality gate (§7.6). Refuses non-zero velocity/spray if fix_type < 6 (`RTK_FIXED`). |
| `B2 Gains` | RPP Controller | **Default ON** (Landed) | Controller Tuning | Signed-lateral correction gains for smooth arc tracking (field validated 2026-07-25, landed in `ab6ee26`). |
| `systemd_watchdog` | Server | **Default ON** (`True`) | Safety Feature | Systemd `sd_notify` watchdog enabled in `rover-server.service` (`0cf6de7`). |
| `auto_fit_arcs` | Survey Ingest | **Default ON** (`True`) | Path Feature | Automatically fits circular arcs to survey CSV polylines in preview and execution (`beae196`). |
| `corner_fillet` | Preline Engine | **Default ON** (`True`) | Path Feature | Applies corner fillets to survey polylines lacking explicit arc parameters. |
| `xtrack_gate_m` | Guidance Safety | **`0.03 m`** (3 cm) | Safety Param | Cross-track error gate threshold (tightened from `0.10 m` in `c4fdcde`); per-mission override allowed. |
| `max_throttle` | Joystick Control | **`0.35`** (35%) | Manual Control | Manual control max throttle default (raised from `0.10` in `212ac9a`); steering stays at `0.20`. |
| `terminal_run_out_m` | Path Staging | **`0.10 m`** | Path Param | Terminal extension distance; reverted back to `0.10 m` (`d0222a0`) from `0.05 m`. |
| `segment_slowdown_dist` | RPP Controller | **`0.50 m`** | Tracking Profile | Distance before non-final corner where segment mode slowdown begins. |
| `segment_brake_velocity_cap_m_s` | RPP Controller | **`0.08 m/s`** | Tracking Profile | Active corner braking velocity cap. |
| `segment_min_corner_speed` | RPP Controller | **`0.08 m/s`** | Tracking Profile | Speed floor entering segment mode PRE_CORNER zone. |
| `G0` (`rpp_progress_enabled`) | RPP ↔ Spray | **Default OFF** (`False`) | A/B Switch | Mission progress handshake contract & parameter group initialization. |
| `G1` (`publish_rpp_progress`) | Observability | **Default OFF** (`False`) | A/B Switch | Publishes `/rpp/progress` and `/rpp/milestone` observability topics. |
| `G2` (`spray_mark_from_progress`)| Spray Node | **Default OFF** (`False`) | A/B Switch | Sources MARK boundaries from `/rpp/progress` instead of raw position. |
| `G3` (`rpp_precise_stop`) | RPP Controller | **Default OFF** (`False`) | A/B Switch | Enables 2 cm precise point-stopping algorithm on target vertices. |
| `G4` / `G5` (`point_handshake`) | RPP + Spray | **Default OFF** (`False`) | A/B Switch | Enables point mission handshake and manual operator gate. |
| `Phase D` (`planner_point_hold`) | Planner / RPP | **Default OFF** (`False`) | A/B Switch | Planner point-hold mode on frozen RPP controller. |
| `Phase E` (`speed_proportional`)| Spray Node | **Default OFF** (`False`) | A/B Switch | Speed-proportional flow rate valve output regulation (§7.5). |
| `A3` (`ekf_reset_compensation`) | RPP Controller | **Default OFF** (`False`) | Safety Switch | Position jump and EKF-reset compensation logic. |
| `fit_arcs` | Path Engine | **Default OFF** (`False`) | Geometry Switch | Fits surveyed `LINE_CHAIN` polylines to true circular arcs. |
| `close_shape` | Path Engine | **Default OFF** (`False`) | Geometry Switch | Paints the closing edge of closed polygon shapes. |
| `survey_tolerance_m` | Survey Parser | **Operator-Set** | Ingest Metadata | Operator position error tolerance set per survey with provenance tracking. |

---

## 4. Detailed Code Commit Ledger (79 Commits)

| Commit | Date & Time | Category | Commit Summary |
|---|---|---|---|
| `ceef42e` | 2026-07-29 12:08:41 | Code Update | stage2: R1 — callback groups; bounded bridge-health emit |
| `2aeaa5d` | 2026-07-29 11:58:56 | Code Update | stage2: S2 — emit timeout + safety-task split |
| `abaf626` | 2026-07-29 11:54:34 | Fix / Safety | stage1: S1-a — heartbeat must fire in ROS-degraded mode |
| `0cf6de7` | 2026-07-29 11:48:40 | Feature | stage1: S1 — enable systemd notify watchdog for rover-server |
| `c4fdcde` | 2026-07-29 11:36:16 | Tuning | stage1: R3 — xtrack gate 0.10 -> 0.03, per-mission override |
| `d030154` | 2026-07-29 11:26:14 | Feature | stage1: B1, B2 — dash: rebuild meter on new path, restore boundary lead |
| `6dd247d` | 2026-07-29 11:17:14 | Feature | stage1: R5, R6, R4 — dash: respect transit flag, terminal shutoff, geometry anchor |
| `6fe6c76` | 2026-07-28 18:35:28 | Merge | promote trajectory upgrade (plan-trajectory endpoint) to baseline |
| `291f5f1` | 2026-07-28 18:34:51 | Feature | PX4_DXP_FRT trajectory upgrade — POST /api/path/plan-trajectory |
| `71f5eda` | 2026-07-27 22:45:04 | Merge | promote Upgrade_Spray (105 commits, 2026-07-11 -> 07-27) to baseline |
| `132e614` | 2026-07-27 20:02:05 | Code Update | -extension_upgrade |
| `7c18377` | 2026-07-27 19:25:58 | Merge | Merge remote-tracking branch 'origin/Upgrade_Spray' into Upgrade_Trajectory |
| `2fa1016` | 2026-07-27 18:25:29 | Feature | feat(path): A16 — allow path extensions on survey CSVs, not DXF only |
| `4a0b3e1` | 2026-07-27 18:05:30 | Fix | fix(path): A15 — plan-and-stage silently dropped the five shape controls |
| `796432c` | 2026-07-27 17:48:35 | Feature | feat(path): POST /api/path/plan-trajectory — densify + stage an app-planned trajectory |
| `eb6a040` | 2026-07-27 17:16:28 | Fix | fix(gps): A14 — gate on position accuracy, and stop reporting DOP as metres |
| `ed358f0` | 2026-07-27 16:59:28 | Fix | fix(origin): don't invalidate on the FIRST /mavros/state message |
| `779d9f0` | 2026-07-27 16:47:58 | Fix | fix(placement): fail closed on a stale/wrong EKF local-frame origin |
| `d0222a0` | 2026-07-27 15:04:46 | Revert | revert(path): terminal run-out 0.05 -> 0.10 — 0.05 broke the mission endgame |
| `cd3d8ea` | 2026-07-27 14:43:23 | Fix | fix(provenance): correct RD_TRANS param names + capture the WENC A/B knobs |
| `7b57c40` | 2026-07-27 14:26:17 | Fix | fix(provenance+path): A12 FCU param capture via mavros2 + terminal run-out 0.10->0.05 |
| `0e85c64` | 2026-07-27 11:25:02 | Feature | feat(field): joystick arm-on-acquire + GPS status truth + RTK reconnect grace |
| `4c94899` | 2026-07-26 00:14:20 | Feature | feat(analysis): WENC A/B prereqs — full GPSRAW parse + record raw/fix |
| `ab6ee26` | 2026-07-25 19:32:26 | Feature | feat(rpp): land B2 smooth-arc gains as defaults (field-validated 2026-07-25) |
| `8da931d` | 2026-07-25 18:59:50 | Fix | fix(spray): B3 + B5 — stale pivot latch fails open; no valve before tracking |
| `d0ae176` | 2026-07-25 18:59:50 | Fix | fix(spray): B4 — terminal shutoff + fail-closed watchdog + disarm on complete |
| `3634200` | 2026-07-25 18:17:38 | Feature | feat(rpp): B2 — gated signed-lateral correction for the smooth profile |
| `22c02de` | 2026-07-25 18:17:38 | Fix | fix(analyzer): B12 NaN-blind verdicts, B7 diluted xtrack gate, B9 manifest key |
| `2dc7155` | 2026-07-25 18:17:14 | Fix | fix(frame): B6' — every PX4-local-NED conversion now uses PX4's own sphere |
| `a686995` | 2026-07-25 15:57:19 | Feature | feat(mission): /start verifies the mission_id the caller asked for |
| `01c68ba` | 2026-07-25 15:38:10 | Fix | fix(api): /api/path/plan silently disabled the survey arc fit |
| `0fc5097` | 2026-07-25 14:49:34 | Feature | feat(preview): emit the original surveyed lat/lon as control_points |
| `420c40f` | 2026-07-25 12:53:30 | Fix | fix(preline): collapse re-occupied survey stations before any geometry |
| `0c10fa6` | 2026-07-25 12:14:05 | Perf | perf(validator): spatially index the wet-paint check (35s -> 0.15s) |
| `3f0008f` | 2026-07-25 12:04:03 | Feature | feat(preline): corner fillet for surveys with no arcs; raise waypoint cap |
| `9d9a749` | 2026-07-25 11:22:30 | Fix | fix(preline): curvature-segmented arc fit — curves were still chords |
| `8d2b8b9` | 2026-07-24 18:39:13 | Fix | fix(preline): survey grouping (feature/road/seq) + arc-fit blow-up guard |
| `beae196` | 2026-07-24 18:13:46 | Fix | fix(preline): auto arc-fit survey CSVs in preview + execution (was chords) |
| `99eb3d8` | 2026-07-24 17:31:44 | Feature | feat(preview): survey-CSV preview is WYSIWYG — real must-hit + geo_origin |
| `a84fcaf` | 2026-07-24 17:11:41 | Feature | feat(engine): close_shape paints the closing side (default off) |
| `b5ee685` | 2026-07-24 17:08:13 | Feature | feat(engine): fit surveyed LINE_CHAINs to arcs behind fit_arcs (default off) |
| `fcb891b` | 2026-07-24 16:57:47 | Fix | fix(survey_csv): strip trailing-space vendor headers so rows parse |
| `740b325` | 2026-07-24 12:41:16 | Feature | feat(analysis): geo overlay — surveyed vs commanded /path vs driven, in lat/lon |
| `6026f53` | 2026-07-24 11:53:20 | Feature | feat(bag): record the RPP↔spray handshake topics for point-mode A/B |
| `2441614` | 2026-07-24 11:53:20 | Feature | feat(ingest): read Emlid/survey-export CSV in the point-mission flow |
| `a4bce1d` | 2026-07-23 19:23:09 | Feature | feat(path): point-mission CSV ingest → must-hit staging (frontend CSV flow) |
| `055eb6c` | 2026-07-23 18:47:34 | Feature | feat(rpp+spray): G4/G5 — point handshake + manual gate (default OFF, frozen A/B) |
| `f41fc37` | 2026-07-23 18:03:54 | Feature | feat(rpp): G3 — precise 2 cm point stop (default OFF, frozen A/B) |
| `fa7768f` | 2026-07-23 18:03:54 | Feature | feat(spray): G2 — source MARK boundaries from /rpp/progress (default OFF, A/B) |
| `4dd454c` | 2026-07-23 17:39:11 | Test | test(rpp): G1.6 in-env fixture — conditioning-robust mark path + deterministic segment placement |
| `40c11d0` | 2026-07-23 17:31:47 | Feature | feat(rpp): G1 — publish /rpp/progress + /rpp/milestone (default OFF, observability only) |
| `87d71b7` | 2026-07-23 17:08:59 | Feature | feat(rpp): G0 — mission-progress handshake contract + params (no behavior change) |
| `67ef615` | 2026-07-23 16:25:26 | Fix | fix(spray): point dwell targets from placed /path must-hit vertices |
| `0cd284a` | 2026-07-23 15:59:26 | Feature | feat(spray): expose live mode + mode config in /api/spray/status |
| `fc9a025` | 2026-07-23 15:37:50 | Feature | feat(spray): backend routes matching the app's spray-mode contract |
| `d2b1b9e` | 2026-07-23 14:40:18 | Feature | feat(spray): Phase E — speed-proportional flow (§7.5), default OFF |
| `25850c9` | 2026-07-23 14:23:13 | Feature | feat(rpp): Phase D planner — point-hold A/B on the frozen controller (default OFF) |
| `9218de4` | 2026-07-23 14:11:09 | Feature | feat(spray): Phase D — point-mode FSM (spray-node half) |
| `5947dd6` | 2026-07-23 13:38:49 | Test | test(spray): add GPSRAW to the ROS stub so Phase B tests import |
| `fdf83b0` | 2026-07-23 13:31:38 | Feature | feat(spray): Phase B — RTK/GPS fix-quality gate (§7.6) |
| `0545fa1` | 2026-07-23 13:12:09 | Feature | feat(spray): B0 server — publish /spray/session_config on load + clear |
| `73c3b3b` | 2026-07-23 13:05:27 | Feature | feat(spray): Phase C dash engine + B0 node session-config transport |
| `8f17e5b` | 2026-07-23 12:13:57 | Fix | fix(controller): A3 EKF-reset compensation — gated, default OFF |
| `212ac9a` | 2026-07-23 12:06:49 | Tuning | tune(joystick): raise max throttle default 0.10 -> 0.35 (steering stays 0.20) |
| `19048f0` | 2026-07-23 11:32:03 | Fix | fix(controller): force run-0 pre-align when mission starts on MARK |
| `6aa6426` | 2026-07-23 11:31:53 | Fix | fix(spray): guarantee terminal MARK->TRANSIT boundary on extensions-off missions |
| `75e6012` | 2026-07-23 11:31:39 | Fix | fix(path): keep must-hit on every corner of multi-entity shapes (A9) |
| `f2eaaa6` | 2026-07-23 10:29:24 | Fix | fix(joystick): revert MANUAL_CONTROL steering to y (roll) — field regression |
| `7d77565` | 2026-07-22 19:07:34 | Feature | feat(survey): survey_tolerance_m is operator-set per survey, with provenance |
| `5f820be` | 2026-07-22 19:03:13 | Feature | feat(analysis): §9 TRAVERSAL — how much of the path did the rover actually drive |
| `21a8b05` | 2026-07-22 18:53:06 | Fix | fix(mission): pass vertex provenance on every load route, not just staged |
| `e3939e3` | 2026-07-22 18:46:20 | Fix | fix(staging): carry the source file's provenance so §8 can run |
| `dd166b3` | 2026-07-22 17:57:48 | Fix | fix(placement): use the EKF's declared origin so a staged mission places identically |
| `a61118d` | 2026-07-22 16:52:56 | Fix | fix(analysis): NavSatFix parser read service as uint8, not uint16 |
| `f1c8610` | 2026-07-22 15:47:04 | Feature | feat(analysis): §8 ABSOLUTE ACCURACY — did the rover reach the surveyed lat/lon? |
| `211244b` | 2026-07-22 15:24:33 | Feature | feat(ingest): survey CSV ingest + POINT-layer control points |
| `e483d53` | 2026-07-22 15:13:48 | Fix | fix(georef): north axis used the semi-major axis, not the meridional radius |
| `64c12ff` | 2026-07-22 13:51:07 | Fix | fix(rpp): stop deleting surveyed vertices — real Douglas-Peucker + vertex provenance |
| `de943e4` | 2026-07-22 12:08:38 | Feature | feat(analysis): geometry-fidelity report + fix coast false-FAIL + richer bags |
