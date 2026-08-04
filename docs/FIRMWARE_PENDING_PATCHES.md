# Firmware Pending / Patches

> Task list for the PX4 fork (`Vetri2425/PX4-Autopilot`, branch `main`, reviewed @ `4152220`).
> Findings from code review 2026-08-03. **Nothing here is implemented** — this is the backlog.
> Rules: firmware is built via CI overlay on stock v1.16.2, flashed from the Mac via QGC.
> Never edit firmware on the Jetson; never push FCU params from the Jetson.

---

## A. Encoder fusion (EKF2 wheel-encoder body-frame velocity)

Verdict from review: the Kalman math (shared `fuseBodyFrameVelocity`) is sound, but the
feature is undermined at both ends — unreliable data in, under-defended model around it.
Reminder: in the June logs the fusion fused **zero samples** (RoboClaw serial dead ~49×/s),
and `EKF2_WENC_CTRL=0` on the vehicle today.

### A1. RoboClaw driver — serial desync never recovers (ROOT CAUSE of the field no-op)
- `readResponse()` (`src/drivers/roboclaw/Roboclaw.cpp:442`) reads exactly N bytes,
  no framing, no resync; nothing ever flushes the UART RX buffer on error.
- One stray byte (e.g. late ACK from a motor command) shifts every later read by one
  → every CRC fails **forever** → the persistent ~49/s "Checksum mismatch / Error
  reading encoders" signature.
- **Patch:** `tcflush(_uart_fd, TCIFLUSH)` on any transaction error and before each
  read cycle. Single highest-value firmware change in this list.

### A2. RoboClaw driver — decouple encoder reads from the mixer rate
- `readEncoder()` runs every `Run()` (MixingOutput Auto scheduling): 3 blocking serial
  round-trips per cycle on the same UART as motor commands + ACKs.
- EKF2 consumes at most 1 sample / 10 ms; `wheel_angle` (cmd 78, ReadEncoderCounters)
  is read every cycle and consumed by **nobody**.
- **Patch:** decimate speed reads to ~50 Hz; drop or slow the counters transaction
  (1–5 Hz). ~1/3 less bus traffic and desync exposure.

### A3. RoboClaw driver — health visibility
- `PX4_ERR` spam unthrottled at failure rate; no error counters in `print_status`;
  no health flag anywhere. A 100 %-dead encoder link went unnoticed for weeks.
- **Patch:** consecutive-failure counters, rate-limited error print, link-health
  exposure (e.g. field on `wheel_encoders` or status pub).

### A4. Timestamping / delay
- `hrt_absolute_time()` stamped **after** all three blocking `select()` reads
  (`Roboclaw.cpp:265`); cmd 18/19 measure over a trailing 1/300 s window.
- **Patch:** stamp before the first read; measure and raise `EKF2_WENC_DELAY`
  (yaml default 5 ms understates true latency).

### A5. Fusion model — lever-arm compensation (attacks the measured pivot walk)
- Fuser observes `(v_fwd, 0, 0)` raw (`wheel_encoder_fusion.cpp:70`). The zero-lateral
  constraint is only valid at the driven-axle midpoint; at the IMU it is wrong by
  `ω × r` in any turn. The EV body-vel path does this correctly (`ev_vel.h:52-54`).
- Field evidence: stationary-pivot 1.06 cm sinusoid = lever arm, measured 08-03;
  pivot walk 4.6 cm is the largest unaddressed error-budget term.
- **Patch:** add `EKF2_WENC_POS_X/Y/Z`, subtract `ω × (sensor_pos − imu_pos)`,
  mirroring the EV code. Requires physically measuring Cube-to-axle offset.

### A6. Fusion model — no-slip constraint applied unconditionally
- This rover pivots in place (counter-rotating wheels + scrubbing caster) — exactly
  where "forward = wheel average" and "lateral = 0" are both false.
- **Patch:** gate or inflate variance when turning hard: `|v_r − v_l| / (|v_r| + |v_l|)`
  large, or gyro yaw rate over threshold.

### A7. Fusion model — unused slip detector
- Wheel difference gives `ω_wheels = (v_r − v_l) / RD_WHEEL_TRACK`; comparing against
  the gyro is a free online slip detector. Variance is currently a constant param
  regardless of speed/turning/slip.
- **Patch:** inflate `vel_fwd_var` (or skip sample) on gyro-vs-wheel disagreement.

### A8. Per-wheel calibration
- Single `RBCLW_COUNTS_REV` for both wheels (open bug B2). L/R mismatch becomes an
  along-track scale error + false yaw signal. **Patch:** per-wheel counts params.

### A9. Estimator safety — WENC masks GNSS loss, disables GSF yaw rescue
- `fuseBodyFrameVelocity` refreshes the **global** `_time_last_hor_vel_fuse` /
  `_time_last_ver_vel_fuse` (`ev_vel_control.cpp:203-204`). While WENC fuses:
  local-position validity never times out on GNSS-vel rejection, and the EKF-GSF
  emergency yaw reset **can never fire** — the rescue for exactly the wrong-heading
  failure the assumed 180° antenna offset (bug B1) could produce.
- **Patch:** give WENC its own fuse timestamp; do not refresh global timers from a
  secondary aid. Genuine safety defect, independent of accuracy.

### A10. Estimator safety — no fault path
- `stopWheelEncoderFusion()` is a never-wired stub; persistent gate rejection is
  silent. **Patch:** rejection-timeout → stop fusion + logged event.

### A11. Per-axis gating
- 3-axis all-or-nothing gate: forward-axis rejection (slip) throws away the lateral
  constraint too, and vice versa. **Patch:** per-axis gates (different failure modes).

### A12. Param-only companion work (no rebuild — do together with re-enable)
- Yaml defaults `EKF2_WENC_NOISE/LAT_N = 0.1` @ ~50 Hz ≈ 98 % weight vs 10 Hz GNSS →
  1° heading bias leaks to position at `v·sin(ψ_err)` ≈ the measured 1.9 cm/m drift.
  Live FCU already at 0.35/0.35, gate 5.0 (saner).
- **`EKF2_GPS_P_NOISE = 1.0 m`** floors RTK's 1.5 cm to metre-class — position can
  never pull drift back. Drop toward ~0.05 before any WENC re-enable A/B.
- Maybe consider odometry (delta-position from counts) instead of 1/300 s speed window —
  bigger refactor, only after A1–A9.

**Recommended order:** A1+A2 (else fusing nothing) → A5+A6 (the two measured
artifacts) → A9 (safety) → params A12 → the rest.

---

## B. Manual-mode driving — throttle slewed, steering instant

Symptom (operator, 2026-08-03): forward accel/decel ramps slowly; steering is
instant on and instant off. **Confirmed in code; inherited from stock v1.16.2,
not fork-introduced.**

### Root cause
Both sticks land in `RoverDifferential::generateActuatorSetpoint()`
(`src/modules/rover_differential/RoverDifferential.cpp:125`) but take different paths:
- **Throttle** → `RoverControl::throttleControl()` (`RoverDifferential.cpp:156`) →
  `SlewRate` using `RO_ACCEL_LIM / RO_MAX_THR_SPEED` up, `RO_DECEL_LIM / RO_MAX_THR_SPEED`
  down (`RoverControl.cpp:49-61`). At live values (0.2 / 0.3 / 1.28):
  ~6.4 s zero→full, ~4.3 s full→zero.
- **Steering** → `manual_control_setpoint.roll` copied raw into `normalized_speed_diff`
  (`RoverDifferential.cpp:115`) → straight into `computeInverseKinematics()`
  (`RoverDifferential.cpp:160`). No slew, no expo, no gain, no deadzone. Nothing.

### Why no param fixes steering (verified — do not burn a field session on these)
- `RO_YAW_ACCEL_LIM` / `RO_YAW_DECEL_LIM` / `RO_YAW_RATE_LIM` — only consumed by
  `DifferentialRateControl` / `DifferentialAttControl`; full Manual bypasses both.
- `RO_YAW_EXPO` / `RO_YAW_SUPEXPO` / `RO_YAW_STICK_DZ` / `RD_YAW_STK_GAIN` — used only
  by `DifferentialManualMode::manual()` (`DifferentialManualMode.cpp:55-69`), which is
  **compiled but never instantiated anywhere in the fork — dead code.** The active
  manual path is `RoverDifferential::generateSteeringAndThrottleSetpoint()` (raw stick).
- Corollary: Acro/Stab manual submodes almost certainly non-functional in this fork
  (nothing publishes stick-driven `rover_rate_setpoint`) — no mode-switch workaround.

### Side findings
1. **Steering defeats the throttle slew at the mixer.** `computeInverseKinematics()`
   (`RoverDifferential.cpp:166-180`) prioritizes yaw: if `|thr| + |diff| > 1` it
   instantly subtracts the excess from throttle — motor commands can step
   discontinuously even on the throttle axis during a hard steer at speed.
2. **08-01 calibration side effect:** raising `RO_MAX_THR_SPEED` 0.96 → 1.28 softened
   manual throttle response by 33 % (slew is normalized by it).

### Options
- **Param-only (QGC, today):**
  - Restore pre-08-01 throttle feel: `RO_ACCEL_LIM≈0.27`, `RO_DECEL_LIM≈0.40`.
  - Or make throttle instant like steering: set both to 0 (explicit fallthrough at
    `RoverControl.cpp:69-70`) — consistent but jerky; wheel-slip risk.
  - Nothing param-side can smooth steering.
- **Firmware patch (pick one):**
  - **B-P1:** wire up `DifferentialManualMode::manual()` in place of the raw path —
    activates existing gain + expo params (curve shaping, still no true slew).
  - **B-P2 (preferred for symmetry):** add a `SlewRate<float>` on
    `normalized_speed_diff` in `generateActuatorSetpoint()` beside the throttle one,
    with one new param (e.g. `RD_STEER_SLEW`). ~15-line diff.
- Priority: LOW — manual is positioning/recovery, not marking. Batch with the next
  planned flash window; do not flash for this alone.

---

## C. GNSS "topics sometimes not publishing" — audit 2026-08-03

> ⚠ File refs in this section are `/Users/dyx_a1/px4-rover-build/...` (= v1.16.2 +
> overlay, the DEPLOYED tree). The fork's `main` checkout is v1.17-dev based and is
> NOT what runs on the Cube — gps/mavlink/sensors in the deployed binary are pristine
> v1.16.2; only EKF2 (re-anchored to v1.16.2 + WENC), logger, roboclaw,
> rover_differential, land_detector, navigator are overlaid.

### C1. 🔴 THE mechanism: heading dropout kills global-position topics entirely
Chain (every step verified in the deployed tree):
UM982 loses dual-antenna heading (RTK degraded → `heading_stddev==0` dropped at
`nmea.cpp:967`, or UNIHEADINGA absent) → `sensor_gps.heading=NaN` (cleared every
publish, `gps.cpp:1224` ≈ `:318`) → GNSS-yaw fusion failing 7 s →
`gnss_yaw_control.cpp:85`: `if (!in_air)` → **clears `yaw_align`** — and `in_air` is
PERMANENTLY false on this rover because the fork's `RoverLandDetector` always returns
landed → `gps_control.cpp:130/177`: yaw_align is a continuing condition → GNSS vel+pos
fusion STOP → `EKF2.cpp:1194`: `vehicle_global_position` published only if
`global_origin_valid && yaw_align` → **topic stops** → `GLOBAL_POSITION_INT` stops →
`/mavros/global_position/*` silent. A stock MAV in flight keeps yaw_align; our
always-landed patch turns any heading dropout into total global-position loss.
**Patch idea:** in the overlay, treat rover-armed-and-moving as the `in_air` branch in
`gnss_yaw_control.cpp` (or gate yaw_align clearing on vehicle-at-rest).
**Live evidence 2026-08-03 19:39:** RTK down (NTRIP dead after 19:01 restart), FCU
spamming "Preflight Fail: heading estimate not stable".

### C2. 🔴 NMEA driver restart cycle = multi-second sensor_gps blackouts
`nmea.cpp:1027-1029` returns **−1** (not 0) on any 500 ms window without BOTH a new
position AND velocity → exits the only loop that calls `publish()` (`gps.cpp:993`) →
UART close/reopen + full `configure()` (up to 6×400 ms baud probing). Aggravators:
- `_POS_received` requires strictly increasing UTC (`nmea.cpp:227`) — whole-second
  GGA timestamps make 5 Hz GGA count as 1 Hz.
- **AGRICA one-way latch** (`unicore.h:104`, set at `unicore.cpp:226`, never cleared,
  `reset()` doesn't touch it): once one AGRICA parsed, RMC/VTG velocity fallback is
  disabled for the parser's lifetime (`nmea.cpp:542,890`) — AGRICA stall ⇒ C2 fires.
- Driver stall also stops RTCM injection (`handleInjectDataTopic` only reachable
  from inside `receive()`, `gps.cpp:479→542`) — positive feedback: no RTCM → fix
  degrades → heading dies → C1.

### C3. 🔴 Config-string spam during heading loss (self-perpetuating)
`nmea.cpp:995-997`: while heading is missing, EVERY AGRICA (5 Hz) triggers
`request_unicore_messages()` (~550 B/s TX) on the same UART carrying RTCM — because
`_unicore_heading_received_last` is only updated on GotHeading. Should be throttled
by request time, not receive time.

### C4. 🟠 GNSS pre-flight checks latch while rover moves
`gps_checks.cpp:88/146`: drift/speed checks only re-evaluate at rest; the in-air
force-pass (`:136`) is unreachable (always landed). A failing check at motion start
stays failed → 7 s → `stopGnssFusion()` ("GNSS quality poor - stopping use").
Live `EKF2_GPS_CHECK=927` has the speed/drift bits set — consider clearing them, or
patch the latch in the overlay.

### C5. Param/documentation flags (verified against live QGC export)
- `GPS_1_PROTOCOL=6` ✓ (NMEA; note auto=0 can NEVER detect a UM982), `GPS_1_CONFIG=101`
  ✓, no MAV port collision (`MAV_0_CONFIG=0`) ✓, `SENS_GPS_MASK=7` (blending on).
- **`EKF2_GPS_YAW_OFF=180` is DEAD CODE** on this setup: `EKF2.cpp:2479` only applies
  it when `heading_offset` is NaN, but the driver always publishes a finite
  `heading_offset` (`gps.cpp:937`). The WORKING knob is driver `GPS_YAW_OFFSET=180`,
  which exactly cancels the hardcoded +180° master-front assumption
  (`nmea.cpp:975-977` adds 180, `nmea.cpp:1037` subtracts the offset) → net raw
  UM982 baseline heading (coherent with reversed antenna mounting; applied ONCE).
  CLAUDE.md lists both params as active — misleading; fix docs.
- Never-publishing topics that are NORMAL here (don't chase them): `gps2/raw`
  (`sensor_gps` instance 1 never advertised), `gps_rtk/rtk_baseline`, `gp_origin`
  (only after MAV_CMD_REQUEST_MESSAGE 49).

### C6. 🟡 Smaller firmware defects (upstream, present in v1.16.2)
- `GLOBAL_POSITION_INT.hpp:69`: short-circuit `&&` consumes and DISCARDS a fresh
  global-position sample when local-position hasn't updated — jitter/dropouts.
- `GPS_RAW_INT.hpp:111`: NO_GPS heartbeat only arms after first real message —
  cold-boot driver failure = total GPS_RAW_INT silence (indistinguishable from
  dead link).
- `logged_topics.cpp` overlay lacks `estimator_aid_src_gnss_pos/vel/yaw` — C1/C4
  are invisible in default logs; add them (WENC topics were added the same way).

### C7. Companion-side issues found in the same audit (NOT firmware — tracked here
for completeness; fix in PX4_DXP)
- **H1** `rpp_controller_node.py` RTK gate latches `_gps_fix_type` with NO staleness
  check — topic dies mid-mission ⇒ drives forever on stale `fix_type=6`
  (spray node does it right: 2 s timeout).
- **H2/H3** server telemetry: `gps_fix`/`hrms` sticky forever, `gps_fix_age_ms`
  exposed but ignored; mission-health watchdog has NO GPS-staleness term — UI shows
  RTK_FIXED indefinitely after GPS death, no abort.
- **H4 (verified live)** `mavros_node` + all 3 Python nodes pinned CPUAffinity=4 at
  SCHED_FIFO prio 80 — a Python burst starves the MAVLink reader; 5 Hz GPS topics
  vanish first while 1 Hz state ticks. Separate cores or priorities.
- **H5/H6** NTRIP: no autostart after rover-server restart (ACTIVE INCIDENT 08-03
  19:01); GGA back-feed derives from `raw/fix` so a GPS stall stops GGA → caster
  stops RTCM → self-sustaining outage; health check only logs, never reconnects.
- **M1** placement `pose_global_skew_ms` compares last-receive times of a 30 Hz and
  a ~10 Hz topic against a 100 ms gate — intermittent false UNVERIFIABLE on a
  healthy rig.

**Priority within section C:** C1 (+ C7-H1/H4/H6 companion twins) → C2/C3 → C4 →
C6 logging → docs C5.

---

## D. Cross-audit inconsistency index (2026-08-03)

Condensed list of every place two parts of the system disagree. Details in §A–C.

### D-i. Same job, two standards
| # | Inconsistency | Ref |
|---|---|---|
| 1 | EV body-vel fusion compensates lever arm; WENC path doesn't (same fuser) | A5 |
| 2 | WENC refreshes global fuse timers (acts as aid) but sets no control-status flag and its stop function is a stub (not accounted as aid) | A9, A10 |
| 3 | Spray node: GPS staleness timeout + correct `h_acc=0` handling. RPP node: latches `fix_type` forever, never reads accuracy | C7-H1 |
| 4 | Manual throttle slew-limited (~6.4 s to full); manual steering raw stick. OFFBOARD yaw slewed; Manual yaw not | B |
| 5 | Throttle slew applied pre-mixer, but IK yaw-priority trim steps motor outputs instantly anyway | B side-finding 1 |
| 6 | NMEA `receive()` returns −1 (restart driver) where convention is 0 (wait) — quiet 500 ms ⇒ multi-second blackout | C2 |
| 7 | `UnicoreParser::reset()` doesn't reset `_agrica_valid` (one-way latch kills velocity fallback) | C2 |
| 8 | Unicore re-request throttled on last *receive*, not last *request* → 5 Hz config spam during heading loss | C3 |

### D-ii. Params vs reality
| # | Inconsistency | Ref |
|---|---|---|
| 9 | `EKF2_GPS_YAW_OFF=180` dead code; `GPS_YAW_OFFSET=180` is the real knob (cancels hardcoded +180) | C5 |
| 10 | `RO_YAW_EXPO`/`SUPEXPO`/`STICK_DZ`/`RD_YAW_STK_GAIN` consumed only by never-instantiated `DifferentialManualMode`; Acro/Stab submodes dead too | B |
| 11 | `CPUQuota=200%` no-op under FIFO — px4-dxp.service's own comment says so, yet both services still share core 4 @ FIFO 80 | C7-H4 |
| 12 | WENC yaml defaults 0.1/0.1 vs live FCU 0.35/0.35 | A12 |
| 13 | Trust inversion: RTK (1.5 cm) floored to 1.0 m by `EKF2_GPS_P_NOISE`; wheel encoder σ0.1 @ 50 Hz | A12 |
| 14 | `RO_MAX_THR_SPEED` is a calibration but silently sets manual throttle feel (0.96→1.28 = 33 % softer) | B |

### D-iii. Patch vs assumption (overlay side effects)
| # | Inconsistency | Ref |
|---|---|---|
| 15 | Always-landed `RoverLandDetector` vs EKF `in_air` branches: heading dropout clears yaw-align (kills global topics); GNSS drift checks latch last at-rest verdict while driving | C1, C4 |
| 16 | Logger overlay logs WENC debug topics but NOT `estimator_aid_src_gnss_*` — the GNSS dropout is invisible in default ulogs | C6 |
| 17 | Fork `main` (v1.17-dev, unbuildable) ≠ deployed binary (v1.16.2+overlay @ `~/px4-rover-build`); `ver_sw` can't tell them apart | C header |

### D-iv. Producer vs consumer
| # | Inconsistency | Ref |
|---|---|---|
| 18 | `gps_fix_age_ms` computed + streamed, acted on by nothing; telemetry gps fields sticky; health watchdog has pose-stale but no GPS-stale term | C7-H2/H3 |
| 19 | `pose_global_skew_ms` compares receive times of 30 Hz vs ~10 Hz topics against 100 ms gate — measures phase, not skew | C7-M1 |
| 20 | RoboClaw reads `wheel_angle` every cycle, zero consumers; encoder read rate = mixer rate vs EKF ≤1 sample/10 ms | A2 |
| 21 | "Absent" indistinguishable from "dead": GPS_RAW_INT NO_GPS heartbeat arms only after first msg; gps2/rtk_baseline/gp_origin silent by design; `gps_fix_received` not in telemetry | C5, C6, C7-L1 |
| 22 | NTRIP: clean stop on rover-server restart, no start ever (no autostart; health check logs, never reconnects; `ntrip.service` needs binaries that don't exist) | C7-H5/H6 |

### D-v. Docs vs vehicle (see also §E — estimation audit findings F1/F6)
| # | Inconsistency | Ref |
|---|---|---|
| 23 | CLAUDE.md listed `EKF2_GPS_YAW_OFF` as the active heading knob (**fixed 2026-08-03**); WENC rows previously wrong (fixed earlier); cross-project integration.md still carried `RO_MAX_THR_SPEED=0.9` prereq (**fixed 2026-08-03**) | C5 |

---

## E. Estimation → tracking audit for 1 cm RMS / ≤1° heading (2026-08-03, 2 Opus agents)

Deployed tree `~/px4-rover-build` + PX4_DXP `Upgrade_speed`. Question: can the stack
deliver 1 cm RMS xtrack, no swing, ≤1° sustained heading?
**Verdict: NOT as configured (≈4–5× off); YES in principle (~0.9–1.2 cm after fixes
— at the physical floor of a 1.5 cm RTK receiver at 5 Hz, zero margin).**

### E1. Firmware findings (F-series)
- **F2 🔴 HARD BLOCKER: GNSS yaw obs noise is hardcoded `0.1 rad` (5.7°) with NO
  param** (`common.h:357` `gnss_heading_noise`, used `gnss_yaw_control.cpp:139,230`
  via `fmaxf(yaw_acc, …)` — discards the UM982's real ~0.35° by 11–20×).
  Steady-state estimator heading σ = **1.46°** (τ≈3.1 s to absorb a heading step)
  → ≤1° is impossible at the ESTIMATOR, before any control. Patch: bind to new
  `EKF2_GPS_YAW_N`, set ~0.01 → σ 0.28°, τ 0.31 s. **The single number that
  decides the heading spec.**
- **F1: `EKF2_HEAD_NOISE` is a NO-OP here** — binds to `mag_heading_noise`
  (`EKF2.cpp:140`). Delete from config mentally.
- **F3 (corrected to LIVE params): heading-control deadband =
  `RO_YAW_RATE_TH / RO_YAW_P` = 1.0/1.5 = **0.67°**** (live TH=1.0, not default 3.0
  — the agent's "2.0°" used the default). Both the yaw-rate SETPOINT and the
  MEASURED yaw rate are zeroed below TH (`DifferentialRateControl.cpp`). Still
  eats 2/3 of a 1° budget; `TH=0.5` → 0.33°. Do AFTER F2 (else injects 1.46°
  estimator noise into the wheels).
- **F4: `EKF2_GPS_P_NOISE=0.05` is a FLOOR over hacc** (`gps_control.cpp:257`) —
  RTK 1.5 cm inflated to 5 cm, 11× variance discarded. → 0.015.
  Same for vel: `EKF2_GPS_V_NOISE=0.3` floor (`:219`) makes the 14 mm/s pivot
  lever-arm signal invisible (0.047σ). → 0.05.
- **F5 🔴 `at_rest` REGRESSION from always-landed overlay:** `at_rest` now = pure
  vibration metric; a smooth 0.35 m/s crawl or slow pivot can arm
  **`ZeroGyroUpdate` which fuses LIVE gyro output as gyro BIAS while moving**
  (`control.cpp:164-166`, `ZeroGyroUpdate.cpp:52-70`). Prime suspect co-factor for
  pivot walk + heading error. Also: every GNSS-yaw restart takes the hard
  `resetYawToGnss()` branch (heading STEPS, `gnss_yaw_control.cpp:100-111`).
  Fix with C1 (same root cause).
- **F6 LEVER ARM — CORRECT PARAMETERISATION (supersedes the sketch's proposal):**
  set **`EKF2_IMU_POS_Y = +0.035`**, keep `EKF2_GPS_POS_Y = 0`. This defines body
  origin ≡ antenna ≡ nozzle (the point we control); output predictor then
  de-rotates pos+vel correctly. Setting `EKF2_GPS_POS_Y=-0.035` instead would move
  the reported point to the FC and make the pivot sinusoid WORSE (22 vs 13 mm).
  Numbers: pivot sinusoid = 13 mm rotation-centre offset (predicts 1.3 vs
  observed 1.06 cm); pivot walk = 2×(35−13)=44 mm IMU↔rotation-centre
  irreconcilable orbit (predicts 4.4 vs observed 4.6 cm).
- **F7: NMEA driver units bug — `s_variance_m_s` filled with a VARIANCE, consumed
  as σ** (`nmea.cpp:1003-1007` vs `SensorGps.msg:13`) — silently disables the
  `EKF2_REQ_SACC` check (bit 4 of EKF2_GPS_CHECK=927).
- **F8: `/dev/ttyACM0` mavlink runs NORMAL mode → LOCAL_POSITION_NED 1 Hz,
  ATTITUDE 15 Hz from firmware**; the observed 30 Hz exists only because MAVROS
  sends SET_MESSAGE_INTERVAL. Silent 1 Hz fallback if that ever fails. Pin with
  `-m config` in rcS (30/50 Hz) — or move to ODOMETRY (has `timestamp_sample`;
  LOCAL_POSITION_NED stamps publication time → companion can't age-compensate).
- Latency truth: wire age of pose ≈ 20–45 ms (not 100–150); the 0.25 s
  output-predictor τ applies to CORRECTIONS (5 % overshoot, `output_predictor.cpp:323`).
  `EKF2_GPS_DELAY=50 ms` is an uncalibrated guess — sweep 30–90 vs innovations.
- Confirmed structural: velocity OFFBOARD derives yaw from `atan2(vel)` ONLY
  (`DifferentialVelControl.cpp:141-166` overlay) — companion has ZERO independent
  heading authority; the 2.8° crab is uncorrectable inside PX4 today. Fix ideas:
  `RD_CRAB_OFF` param added to bearing, or allow attitude+velocity offboard.

### E2. Companion findings (D-series, `Upgrade_speed`)
Frames audit: **clean** — all 9 ENU↔NED sites correct, no filtering on pose/yaw,
estimate passed through faithfully (arrival-time staleness is conservative).
- **D1 🔴 Pivot fires ≤5 cm short of the corner (acceptance 0.05) and exit gate is
  heading-only — position NEVER checked**; pivot target = segment direction
  (`rpp_controller_node.py:2652`, `:3551`, `:3581`). Exit xtrack 0–5 cm, E≈2.5 cm
  = the 2.4 cm pivot walk. **Fix: pivot-to-intercept (~6 lines).**
- **D3 `pose_latency_bias_s=0.0`** — extrapolation exists and is correct
  (`:4143-4166`) but is fed only the ~19 ms transport age; set ≈0.03–0.05 (per
  F8-corrected wire latency; calibrate) and raise `imu_max_extrap_age_s`.
- **D4 Pure-P course loop, no integrator** — crab 2.8° ⇒ e_ss = Ld·tanβ ≈ 1.7 cm
  standing offset; yaw is NEVER in a feedback path on straights (`:3849-3852`).
- **D5** pivot-exit tolerance 3/4/5° ⇒ 0.9–1.5 cm swing seed on top of D1.
- **D6 🔴 ambiguous yaw-rate sidechannel:** segment mode publishes `1.5·θe` as
  "feedforward" while a comment claims yaw_rate is inert; bridge promotes
  type_mask to 455 when present (`twist_to_setpoint_node.py:283-288`). Per
  DifferentialVelControl it IS discarded — but A/B `use_feedforward_yaw_rate:=false`
  to make the loop identifiable before ANY gain work.
- **D8/D9 lookahead pinned at clamp floor 0.35** ⇒ `lookahead_time` and
  `xtrack_lookahead_gain` are DEAD knobs at 0.35 m/s; loop gain v/Ld = 1.0 s⁻¹,
  ω_c 0.145 Hz = the swing band. Raising Ld→0.55–0.70 = +14° PM but DOUBLES the
  crab offset — only admissible after D4/F6.
- D7 pivot/hold paths publish `cross_track=0.0` literals → walk invisible in
  `/rpp/debug`; D10 simplifier tol 0.01 = whole budget (→0.003); D11
  `_absorb_short_connectors` accepts 0.5 m apex displacement; D12
  `path_publisher_node.py:600` drops the must-hit bit (dev-path trap); D13 jump
  guard threshold latches on worst gap in 2 s window (use p95). D14 run-remaining
  braking: verified CORRECT.

### E3. Error budget (0.35 m/s straight; cm xtrack)
As configured: GNSS-as-used 2.5 + heading-σ over Ld 1.28 (RSS 2.8) + 3.5 static
lever ambiguity + deadband/limit-cycle + 0.9 crab ⇒ **≫4 cm**. After fixes:
RSS(0.75, 0.25) ≈ 0.8 + residuals ⇒ **0.9–1.2 cm**. Wheel encoder (§A patches
first, then re-enable with loose LAT_N≈0.05 and the NEW gps noise params) buys
the inter-epoch margin — treat as required for a durable 1 cm.

### E4. Combined ranked plan
> **REVISED 08-03 evening (operator):** FC physically moves to the centreline on
> 08-04 ⇒ `EKF2_IMU_POS_Y` param route DROPPED; `EKF2_GPS_POS_*` untouched by
> decision. Remaining geometry item = fore-aft FC→axle distance: mount FC over
> the axle midpoint if possible (zero params), else `EKF2_IMU_POS_X=+d` (reboot).
> Re-verify `EKF2_GPS_POS_Z=−0.4` after remount. Sequencing: **1 cm RMS first,
> encoder enable + robustness second.**
1. **QGC (no rebuild):** after FC move — geometry per note above, `EKF2_GPS_P_NOISE=0.015`,
   `EKF2_GPS_V_NOISE=0.05`, later `RO_YAW_RATE_TH=0.5` (only after F2 patch).
2. **Companion (small diffs):** D1 pivot-to-intercept → D7 real xtrack in debug →
   D3 latency bias → D6 A/B → D10 tol 0.003.
3. **Firmware patch batch (one flash):** F2 `EKF2_GPS_YAW_N` + F5/C1 in_air-vs-at_rest
   fix + F7 units + F8 `-m config` + §A serial fixes (A1/A2).
4. **Then:** raise `min_lookahead_dist` 0.55–0.70 (swing), add xtrack integrator
   (D4) or `RD_CRAB_OFF` (crab), re-enable WENC, field-validate the ladder.

### E5. Evening verification results (08-03 night, bags/d1_verify_20260803)
- **D1/D7 field-verified**: 6/6 pivots released clean (no chatter), run entries
  0.39/0.74 cm (vs 2.4 baseline); walk DURING pivot persists (−1.4…−3.4 cm,
  firmware-side as predicted).
- **Command law verified EXACT in every phase** (transit/pre/mark/aft):
  recomputed pursuit vs published cmd = 0.06–0.34° RMS, zero speed-cap
  violations. Companion is closed; remaining error is plant/estimate.
- Mark RMS 1.35/1.77/2.08 cm (2 PASS, 1 marginal). Phase pattern: run = one
  half-cycle of the slow swing (enter left from pivot → finish right);
  **aft-ext always drifts body-RIGHT (+2–4 cm)** = the same body-fixed defect
  that biases mark's tail (+1.1…+1.8 mean).
- **Swing = TWO components**: slow 0.16–0.18 Hz (loop, λ 1.6–1.9 m) + fast
  λ 0.88–0.95 m ≈ WHEEL CIRCUMFERENCE (0.958 m). Pattern is BODY-fixed, not
  ground-fixed (opp-direction anti-correlation −0.44…−0.83). 0.5 m/s rung
  separates wheel-locked vs loop-locked.
- **Crab re-measured: ~1° and SIGN-FLIPS with direction** (not 2.8°) —
  re-measure per direction before investing in RD_CRAB_OFF.
- **NEW controller item D15**: lookahead point collapses to 0.6–2 cm at the 5
  conditioned-vertex handovers (two sit AT the spray boundaries) — transient
  gain spike; guard = floor the rover→lookahead distance at handovers.
- Conditioning verified: 81 wp @5.4 cm → 6 vtx, 0.00 cm deviation; segment
  profile used for ALL phases incl. transit. Not a source of phase differences.
- Minor open: 21:34 aft endgame had a 3 s window of 63° cmd-vs-law deviation
  (endpoint/hold branch?) — one look at its seg-state sequence, low priority.

### E6. Morning checklist 08-04 (in order)
1. ✅ **DONE 08-04.** FC remounted: lateral offset now physically ZERO, FC sits
   100–105 mm FORWARD of the master antenna. `EKF2_IMU_POS_X=0.100` set,
   `_Y=_Z=0`, `EKF2_GPS_POS_*` untouched at 0/0/−0.4. Verified live on the FC.
   ⇒ body origin ≡ master antenna ≡ axle midpoint ≡ rotation centre ≡ nozzle;
   the reported position IS the paint point. Confirmed against the deployed tree
   that `EKF2_IMU_POS_*` is the correct lever and is **not reboot-required**:
   `output_predictor.h getLatLonAlt()` returns
   `_global_ref + (pos − R_to_earth·_imu_pos_body)` (position at body origin,
   not at the IMU), velocity likewise via `ang_rate % _imu_pos_body`; applied on
   parameter update in `EKF2.cpp` (`set_imu_offset`).
   ⚠ **OWED — pivot sign check, before any field run:** arm, one slow ~180°
   in-place pivot, watch `/mavros/local_position/pose`. <1 cm wander = correct;
   ~20 cm circle = sign flipped (offset doubling instead of cancelling).
2. ✅ **DONE 08-04.** `EKF2_GPS_P_NOISE=0.015`, `EKF2_GPS_V_NOISE=0.05` — both
   verified live.
   🚨 **Param-verification trap found today:** `/mavros/param`'s ROS-parameter
   mirror is populated once at MAVROS plugin init and does NOT track later FCU
   changes. After a QGC edit, `ros2 param get /mavros/param X` and
   `tools/quick_params.py` both return the PRE-EDIT value, and
   `ros2 service call /mavros/param/pull "{force_pull: true}"` reports
   `success=True, param_received=881` **without fixing it** — it refreshes
   MAVROS's internal cache, not the ROS mirror. This produced a false
   "the params did not land on the FC" verdict this morning. **Only a
   `px4-dxp` restart refreshes the mirror; a param read is valid only if
   px4-dxp restarted AFTER the last QGC write.**
3. Firmware batch build+flash: F2 `EKF2_GPS_YAW_N` + F5 at_rest/in_air + F7
   s_variance units + F8 `-m config` + A1/A2 RoboClaw serial.
4. After F2 lands: `RO_YAW_RATE_TH 1.0→0.5`.
5. Field: re-run 1_Aug-II both directions @0.35 (before/after remount compare:
   body-right drift + pivot walk + crab per direction), then 0.5 m/s rung
   (wheel-ripple separator + the ≤2 cm curve+straight close-out gate).
6. GNSS serial check via `GPS_DUMP_COMM=1` (fractional-second GGA, AGRICA rates).
