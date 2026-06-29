# TODO — 2026-06-13

> Generated 2026-06-12 after a codebase-validation pass. Every item below was **verified against the source** as genuinely open. Tasks already implemented were marked complete in `PX4_DXP_Tracker.xlsx`, `docs/Progress/PROGRESS.md`, and the firmware memory — they are **not** repeated here.
>
> Status legend: 🔴 not started · 🟡 in progress/active · 🧱 hardware/bench/QGC blocker (can't be done from code).

---

## 2026-06-29 software backlog update

### Completed in software
- [x] ✅ **GCS-SW1 — GCS migration compatibility matrix.** Added `docs/API/GCS_MIGRATION_COMPATIBILITY.md`.
- [x] ✅ **GCS-SW2 — Activity log CSV export.** Added `GET /api/activity.csv`, exporting the existing bounded in-memory activity records only.
- [x] ✅ **GCS-SW3 — AsyncAPI documentation for current Socket.IO events.** Added `GET /api/docs/asyncapi` plus `docs/API/SOCKETIO_ASYNCAPI.md`; tests guard the event-name contract against route/socket drift.
- [x] ✅ **GCS-SW4 — Read-only ROS node monitoring API.** Added `GET /api/nodes`; it reports expected nodes as present/missing/unknown and degrades safely if ROS graph/CLI access is unavailable.
- [x] ✅ **GCS-SW5 — Wi-Fi/network telemetry API.** Added `GET /api/network`; it uses bounded read-only `ip`/`iw`/sysfs probes and reports unavailable data explicitly.
- [x] ✅ **GCS-SW6 — Unit and integration coverage for the new work.** Added tests for CSV export, node/network degradation and parsing, route registration, and Socket.IO AsyncAPI drift.
- [x] ✅ **GCS-SW7 — Backlog refresh.** This section separates completed software, pending validation/software cleanup, and hardware-deferred items.

### Still pending / software validation
- [ ] **VAL-T1** 🔴 HIGH — Full 9-run hardware validation campaign remains pending.
- [ ] **PE-T1 / PE-T2** 🟡 HIGH — Continue path-engine hardening and CRS validation; do not rebuild validated controller behavior.
- [ ] **PE-T4 / PE-T5** 🔴 HIGH — Validate extensions, transit, and mark/transit transitions on real missions.
- [ ] **I-T2 / I-T3 / I-T4** 🔴 LOW-MED — Cleanup dead launch file after confirmation, field-test `tracking_profile=auto`, and make xtrack CSV path configurable.
- [ ] **COM_OF_LOSS_T production flip** 🔧 — Set via QGC/operator process before field/production; Jetson must not push FCU params.

### Hardware-deferred / explicitly not implemented here
- [ ] **Spray hardware/QGC tasks P3-T1/P3-T2/P3-T8/P3-T11/P3-T12** 🧱 — Require bench/QGC/hardware.
- [ ] **Frontend marking-state pill P3-T7** 🔴 — Separate React-Native GCS repo.
- [ ] **TTS, ultrasonic hardware publishing, WS2812** 🧱 — Not implemented in this software-only pass.
- [ ] **RTK `force_clear`** — Not implemented; existing RTK injection behavior remains unchanged.

---

## ★ TOP PRIORITY — 3 bug-fix tasks (analyse bag + codebase, then fix)

### ✅ BUG-T3 — DONE 2026-06-13 (commit `510be9b`)
- **Fix:** `fix(rpp): forward-cone clamp on first-segment velocity`. Wrong initial turn + reverse navigation resolved.
- **Deploy:** fast-forward pull on Jetson → `rpp-pipeline` restarted (2 s, MAVROS intact) → 3/3 services active, no tracebacks, API ping OK. Mac == origin == Jetson @ `510be9b`. ✓
- _Recommend a confirmation bag (start heading opposing first segment) on next run to close the loop visually._

### 🛠️ BUG-T1 — FIX COMMITTED `036f116`, validating
- **Fix:** yaw-rate settle gates. CORNER_STOP = joint AND gate (speed **and** `|yaw_rate|`<`segment_stop_yaw_rate_threshold`=0.05 for dwell); CORNER_ALIGN exit needs heading+yaw-rate both stable for `segment_align_settle_s`=0.10s; `segment_debug` expanded 9→**10 fields**, new **`[9]` = actual yaw-rate NED (rad/s)**.
- **No firmware change needed:** Opus's "`RO_YAW_RATE_LIM` 90→35" is moot — already **30** (validated `log_183`).
- **Validate on next corner/square bag** (use the auto-recorder): per corner — state `[1]`=5 (STOP) → `[9]` yaw-rate →0 before →3 (ALIGN); state `[1]`=3 → `[7]` heading err + `[9]` yaw-rate both stable near 0 before →1 (TRACK); after TRACK, speed ramps from 0 (no step from 0.08). Analyser must read `segment_debug[9]`.

### 🛠️ BUG-T2 — FIX COMMITTED `1af51ac`, validating
- **Fix:** hard stops at smooth run/segment boundaries. Validate the U-turn bag: speed must NOT drop to 0 at the tangent line↔arc junction.

### BUG-T1 (original framing)  🟡 HIGH
- **Symptom:** at segment stop-and-pivot corners the rover oscillates/jerks during STOP→ALIGN→resume.
- **Where to look:** segment FSM (`CORNER_STOP`/`CORNER_ALIGN`), pivot yaw control (`segment_yaw_rate_gain=1.5`, `RO_YAW_RATE_P=0.17`/`I=0.01`, `RO_YAW_RATE_LIM=30`), decel overshoot near v≈0.
- **Hypothesis:** yaw-rate limit-cycle near zero speed (rate loop with no deadband) and/or `RO_YAW_RATE_I` windup; pivot entry/exit gating via `segment_heading_tolerance_deg`.
- **Validate:** decode a corner/square bag — overlay commanded vs actual yaw rate through the pivot; look for ringing/sign flips.

### BUG-T2 🔴 HIGH — U-turn (line + tangent arc) not continuous (stop & go)
- **Symptom:** tangent U-turn = line(1m) → arc(R0.5m / 2m dia) → line. Tangent junctions are C1-continuous (no heading jump), so motion **should** be smooth. Actual: tracks line, **STOPS** at junction, pivots, tracks arc, **STOPS**, resumes line.
- **Where to look:** path-engine profile splitting (`tracking_profile=auto`: per-entity run split + pivot-align at transitions) and the RPP segment/smooth FSM transition logic.
- **Hypothesis:** the per-entity run splitter inserts a stop/align transition even when junction angle ≈ 0°. Gate continuous flow across tangent junctions (`segment_corner_threshold_deg`); stop-pivot only at hard corners.
- **Validate:** decode the U-turn bag — confirm speed does NOT drop to 0 at the tangent line↔arc boundary after the fix.

---

## 0. Tomorrow's primary — full build validation (9 bags + 9 logs)
- [ ] **VAL-T1** 🔴 HIGH — Validate all 3 shipped fixes + tracking stability over the 9-run campaign.
  - **Tool (ready + proven):** `python3 tools/validate_build.py <dir>` → `BUILD_VALIDATION.md`. Per-bag geo + RPP cross-track (TRACKING-only), per-bug PASS/WARN/FAIL (BUG-T1 pivot ringing via `segment_debug[9]`, BUG-T2 stop-state-aware, BUG-T3 turn direction + no-reverse), per-shape aggregate (mean **and** spread), ulog clipping/param sanity, and an overall **BUILD: STABLE/REVIEW** verdict. Verified against existing bags (reproduces 2.09/3.47 arc, 0.52 square).
  - **Coverage matrix — make the 9 exercise every fix** (don't run 9 arcs):
    - 3× **square** → BUG-T1 (hard-corner pivots), repeatability
    - 3× **lshape** → BUG-T1, repeatability
    - 3× **U-turn (line+tangent arc)** → **BUG-T2** (must NOT stop at tangent junction); start **≥1 run heading-opposed to the first segment** → **BUG-T3**
  - Auto-recorder drops all 9 bags in `~/bags_jet`; pair each with its ulog (`log_*.ulg`). Pull both, drop in one folder, run the tool.
  - Then update per-shape confidence + move BUG-T1/T2 Done if PASS, in `CLAUDE.md` + `PROGRESS.md` + tracker.

---

## 1. Active focus — Path engine / CRS (controller is closed)
- [ ] **PE-T1** 🟡 HIGH — Path engine + trajectory planning hardening: per-entity segment splitting, corner handling, resample/smooth robustness, transit insertion. (Engine exists & "Live" — this is deepening + edge-case validation.)
- [ ] **PE-T2** 🟡 HIGH — CRS / coordinate handling: coordinate reference system + geodesic conversion for path import (DXF/QGC → local NED). Validate least-squares DXF→NED alignment RMSE ≤0.05 m end-to-end. (Karney geodesic + alignment exist — validate, don't rebuild.)
- [ ] **PE-T3** 🔴 HIGH — Full-pipeline validation on hardware: CAD/DXF → path → mission → drive → spray. Depends on spray-hardware tasks + PE-T1/T2.
- [ ] **PE-T4** 🔴 HIGH (priority #4) — **Validate path-engine Extensions + transit.** Test `enable_path_extensions` (`pre_extension_m=0.035` / `aft_extension_m=0.0035` spray lead-in/out) and that **TRANSIT** segments (`transit_speed=0.5`, `transit_spacing=0.15`) properly connect MARK segments. Confirm extension points are inserted with correct spray flags + geometry. **Done = extensions + proper transit validated end-to-end.**
- [ ] **PE-T5** 🔴 HIGH (priority #5) — **Verify transit/mark state transitions per profile.** Confirm `marking_state` / `/spray/active` flips correctly at segment boundaries (MARK on mark segments, TRANSIT on repositioning) under **both** `segment` (stop-pivot) **and** `smooth` profiles. Cross-check telemetry vs path z-channel flags on a bag; watch for missed/late transitions at corners and profile switches.

---

## 2. Spray — HARDWARE / BENCH / QGC only
> Spray **software** is built & unit/flag-tested (transport, flag-conditioning carry-through, launch wiring, `marking_state`, manual-override + failsafe tests — P3-T3/T4/T5/T6/T9/T10 closed). What remains is physical/bench.
- [ ] **P3-T1** 🧱 HIGH — QGC AUX config: set AUX output function = 301, MIN/MAX/DISARMED/FAILSAFE PWM.
- [ ] **P3-T2** 🧱 HIGH — Hardware wiring: solenoid relay/MOSFET on AUX, flyback diode, isolated 12 V supply.
- [ ] **P3-T12** 🧱 MED — Bench-test `MAV_CMD_DO_SET_ACTUATOR` (cmd 187) fires the actuator via QGC actuator testing (pre-req for the rest).
- [ ] **P3-T8** 🧱 MED — Measure end-to-end ON/OFF latency (solenoid LED + log timestamp); feed into `path_engine.spray` lead-in/out compensation.
- [ ] **P3-T11** 🧱 HIGH — Safety validation on hardware before field use: disarm→OFF, E-stop→OFF, MANUAL→OFF, staleness watchdog→OFF, solenoid flyback. **Blocks field use.**
- [ ] **P3-T7** 🔴 MED — Frontend marking-state pill (green=marking / grey=transit / off). Backend `marking_state` is live; verify/implement in the React-Native app (separate repo — not in this workspace, confirm status there).

---

## 3. Infra / cleanup
- [ ] **I-T2** 🔴 LOW — Remove/retire dead `src/launch/rpp_pipeline.launch.py`. Confirmed dead: services launch via `rpp_start.sh` (`rpp-pipeline.service` ExecStart). Cosmetic but avoids confusion.
- [ ] **I-T3** 🔴 MED — Field-test `tracking_profile=auto` entity switching with a real DXF mission. (Segment profile already field-validated on the square; auto/per-entity switching on a mixed mission not yet.)
- [ ] **I-T4** 🔴 LOW — Make `xtrack_logger_node` CSV `log_file` path configurable via systemd env (avoid filling Jetson disk on long runs).

---

## 4. Param status (clarified by operator 2026-06-12)
- [x] ✅ **Live FCU encoder-fusion params CONFIRMED** on `log_183`: `EKF2_WENC_CTRL=1`, `RBCLW_COUNTS_REV=148000`, `EKF2_WENC_RAD=0.1524`. Caveat cleared.
- [x] ✅ `RO_YAW_RATE_LIM=30` — operator fix today, VALIDATED (xtrack <2cm). Not a drift.
- [x] ✅ `EKF2_GPS_CHECK=927` — operator-set, validated. Supersedes doc 831.
- [ ] 🔧 **Set `COM_OF_LOSS_T=0.3` before field/production.** Currently `30` is an intentional DEBUG/TEST value (avoids nuisance OFFBOARD-loss failsafes during testing). Flip to 0.3 s for real missions.

---

### Closed during the 2026-06-12 audit (for traceability — do not redo)
- Controller + tuning: **CLOSED/frozen** (square <2 cm validated; smooth arc deferred at structural floor).
- Sprint 1 corner-xtrack: S1-T1/T2/T5 Done, S1-T3 Superseded (`yaw_rate_feedback_gain` no-op in velocity mode), S1-T4 Closed, S1-T6 N/A.
- Spray software: P3-T3, P3-T4 (CRITICAL), P3-T5, P3-T6, P3-T9, P3-T10 → **Done**.
- Encoder fusion: **Done via EKF2** (validated log_150). robot_localization S2-T7/T8 + I-T1 → **Superseded**, S2-T9 → Done.
