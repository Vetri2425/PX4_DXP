# Field + Firmware Plan — 2026-08-05 (live from 11:10 IST)

> **LAST DEVELOPMENT DAY.** After today: **test + param tuning only**, no new firmware.
> Scope is therefore deliberately narrow. Everything below is either (a) verified in
> source, (b) verified against the **live FCU today**, or (c) explicitly marked
> unverified/deferred. Nothing is speculative.
>
> Firmware repo: `/Users/dyx_a1/Vetri/PX4-Autopilot` @ `d62f42fed0` (== `origin/main`, verified).
> Build base: **stock v1.16.2 + 26-file overlay** via `.github/workflows/build_rover.yml`.
> There is **no local build script** — CI is the only build path.
>
> **The day runs on two tracks in parallel.** The rover is up and RTK-fixed *now*, and CI
> takes ~30 min. Doing the flash-independent field work while the firmware builds is what
> makes both fit in one day — and it keeps one variable per measurement.

---

## 0. The one-line thesis

The rover's straight-line error is a **yaw limit cycle**, not a path-following error.
It has exactly two drivers, both measured:

| Driver | Evidence | Fix | Available |
|---|---|---|---|
| Heading **noise** injected by the estimator | `gnss_heading_noise` hardcoded 0.1 rad (5.73°) discards the UM982's real 0.77–1.10° | **F2** (firmware) | after flash |
| Yaw loop at **marginal phase margin** | steady yaw gain **1.121 ± 0.008**, plant A **4.843 ± 0.063**, verified from firmware source (§6d) | **`RD_MAX_THR_YAW_R`** 0.95 → **1.14** ✅ **DONE 08-05** | applied |

Leverage order is settled: **F2 > `RD_MAX_THR_YAW_R` > pivot release.**
Sim (calibrated at 0.35 m/s): F2 collapses band 4.5 → 1.4 cm, steady RMS 0.86 → 0.30.

⚠ The sim **diverges at 0.5 m/s**, which is the speed we care about. Treat as direction, not prediction.

**Sequencing consequence:** the FF calibration is flash-independent, F2 is not. Land the
calibration **first, in the field, while CI builds**. Then the post-flash F2 A/B runs on an
already-correct loop and its effect is attributable to F2 alone. Do not change both into
the same wobble measurement.

---

## 0b. Boot state — DONE at 11:05 IST

| | |
|---|---|
| Companion | mac == origin == Jetson @ **`2cdf78b`** (branch `field/spray-d15`) |
| Restarted | `rpp-pipeline`, `rover-server` (disarmed, MANUAL) — 0 tracebacks, `/api/ping` ok |
| RTK | **RTK_FIXED** (`fix_type: 6`, 27 sats), NTRIP live on `caster.emlid.com:2101` / MP23960 |
| Live runtime params | `stop_latch_enabled` **False** (A/B knob), `min_lookahead_dist` 0.52, `mission_speed` 0.50 |

⚠ If `rover-server` is restarted again, NTRIP dies **silently** — `POST /api/rtk/ntrip/start` + re-login.
⚠ Runtime params (`stop_latch_enabled`, `mission_speed`) do **not** survive an `rpp-pipeline` restart.

### Live FCU readings taken today (11:05) — these correct the plan

| Param | Live value | Consequence for this plan |
|---|---|---|
| `RD_MAX_THR_YAW_R` | 0.95 | step 1 stands |
| `RO_YAW_RATE_P` / `RO_YAW_RATE_I` | 0.13 / 0.01 | model inputs confirmed |
| `RO_YAW_P` | 1.5 | — |
| `RD_WHEEL_TRACK` | 0.470 | `R_opt` input confirmed |
| `EKF2_IMU_POS_X` | 0.100 | remount confirmed; **sign still owed** |
| `RO_SPEED_TH` | 0.10 | stop-latch threshold matches |
| `RD_TRANS_TRN_DRV` | 0.0349 rad = 2.0° | pivot-release pair input |
| `RO_MAX_THR_SPEED` | 1.28 | HOLD (§6b) |
| `GPS_YAW_OFFSET` | 180.0 | gated on the 4-leg split |
| `RO_YAW_RATE_TH` | 1.0 | (fw default is 3.0 — already lowered) |
| **`EKF2_GPS_YAW_N`** | **`Parameter not set`** | **confirms F2 needs the flash. Cannot be tuned before it.** |
| **`EKF2_GPS_P_NOISE`** | **0.015** | **already at target — struck from §6** |
| **`EKF2_GPS_V_NOISE`** | **0.05** | **already at target — struck from §6** |
| **`EKF2_WENC_CTRL`** | **0** | §7 correct. **CLAUDE.md says 1 — CLAUDE.md is stale, fix it.** |

---

## 1. Scope decision — what ships today and what does not

| ID | Item | Verdict | Reason |
|---|---|---|---|
| **F2** | `EKF2_GPS_YAW_N` param binding | ✅ **SHIP** | The hard blocker. Small, contained. Param confirmed absent today. |
| **A1** | RoboClaw `tcflush` resync | ✅ **SHIP** | Root cause of the encoder no-op. Highest value/line in the backlog. |
| **A2** | Decouple encoder reads from mixer rate | ✅ **SHIP** | Same file as A1, ~1/3 less bus traffic. |
| **F5** | `at_rest` gate on real speed | ✅ **SHIP** | Stops gyro-bias corruption while crawling. Contained to EKF2.cpp. |
| **F8** | `-m config` pin 30/50 Hz | 🟡 **OPTIONAL** | Robustness only, not accuracy. New ROMFS overlay. Ship only if 1–4 are clean by the CI push. |
| **F7** | `s_variance_m_s` units bug | ⛔ **DROP** | **VERIFIED TODAY:** `.gitmodules:13` — `src/drivers/gps/devices` is the `PX4-GPSDrivers` submodule. The 26-file `cp` overlay **cannot reach `nmea.cpp`.** Would need a submodule fork or a CI patch step — not a last-day change. |
| **A5** | WENC lever arm | ⛔ **DEFER** | WENC stays OFF (§7). No value today. |
| **A9** | WENC own fuse timestamp | ⛔ **DEFER** | Only matters if WENC is enabled. See §7. |

---

## 2. Patches — verified against v1.16.2 stock

All line numbers below are from `git show v1.16.2:<path>`.
**Base-discipline rule applies:** any file not already in the overlay must be re-anchored
on v1.16.2 stock before editing:

```sh
git show v1.16.2:<path> > <path>          # then verify `git diff v1.16.2 -- <path>` is clean
```

---

### F2 — bind GNSS heading noise to a parameter

**The defect:**

```
EKF/common.h:349            float gnss_heading_noise{0.1f};   // 5.73°, no param binding
gnss_yaw_control.cpp:139    R_YAW = sq(fmaxf(yaw_acc, _params.gnss_heading_noise))
gnss_yaw_control.cpp:228    yaw_variance = sq(fmaxf(gnss_heading_noise, 1.e-2f))
```

The `fmaxf` floors the receiver's reported accuracy. UM982 reports 0.77–1.10°
(0.0134–0.019 rad); the filter substitutes 0.1 rad. Estimator heading σ = 1.46°, τ ≈ 3.1 s.

**★ Key simplification: `gnss_yaw_control.cpp` does NOT need patching.**
Because the floor is `fmaxf(yaw_acc, param)`, simply *lowering the param* lets the
receiver's real accuracy win. Set `EKF2_GPS_YAW_N = 0.01` → `fmaxf(0.0134, 0.01)` = 0.0134
= the receiver's own value. **This avoids adding `gnss_yaw_control.cpp` to the overlay.**

Note `gnss_heading_noise` is inside `#if defined(CONFIG_EKF2_GNSS_YAW)` — the binding
must carry the same guard.

**Files touched — all three already in the overlay. Zero new overlay entries for the code.**

`src/modules/ekf2/EKF2.hpp` (after the `EKF2_GPS_V_NOISE`/`P_NOISE` pair, ~line 522):
```diff
 		(ParamExtFloat<px4::params::EKF2_GPS_V_NOISE>) _param_ekf2_gps_v_noise,
 		(ParamExtFloat<px4::params::EKF2_GPS_P_NOISE>) _param_ekf2_gps_p_noise,
+
+#if defined(CONFIG_EKF2_GNSS_YAW)
+		(ParamExtFloat<px4::params::EKF2_GPS_YAW_N>) _param_ekf2_gps_yaw_n,
+#endif // CONFIG_EKF2_GNSS_YAW
```

`src/modules/ekf2/EKF2.cpp` (constructor initialiser list, ~line 88):
```diff
 	_param_ekf2_gps_v_noise(_params->gps_vel_noise),
 	_param_ekf2_gps_p_noise(_params->gps_pos_noise),
+#if defined(CONFIG_EKF2_GNSS_YAW)
+	_param_ekf2_gps_yaw_n(_params->gnss_heading_noise),
+#endif // CONFIG_EKF2_GNSS_YAW
```
⚠ Initialiser-list order must match declaration order in the header, or GCC warns
(`-Wreorder`, and PX4 builds with `-Werror`). Place both consistently.

**`src/modules/ekf2/params_gnss.yaml` — NEW overlay file** (one new `cp` line).
Insert alongside `EKF2_GPS_V_NOISE`:
```yaml
    EKF2_GPS_YAW_N:
      description:
        short: Measurement noise for GNSS heading fusion
        long: |
          Standard deviation of the GNSS (dual-antenna) heading measurement.
          Acts as a FLOOR over the accuracy reported by the receiver: the fusion
          uses max(reported_accuracy, this). Set at or below the receiver's real
          heading accuracy so the reported value is used. A UM982 RTK baseline
          reports ~0.8-1.1 deg (0.014-0.019 rad).
      type: float
      default: 0.1
      min: 0.001
      max: 1.0
      unit: rad
      decimal: 4
```

**Default stays 0.1 → stock behaviour is bit-for-bit preserved until set in QGC.**
That is deliberate: the flash itself changes nothing, so a bad flash and a bad tune
are separable failures.

---

### F5 — stop `at_rest` latching true while the rover crawls

**The defect** (three hops):

```
RoverLandDetector.cpp:67    return true;   // rovers are ALWAYS landed (our overlay)
LandDetector.cpp:141        const bool at_rest = landDetected && _at_rest;
LandDetector.cpp:244        _at_rest = (hrt_elapsed_time(&_time_last_move_detect_us) > 1_s);
EKF2.cpp:2616               flags.at_rest = vehicle_land_detected.at_rest;
```

Because `landDetected` is unconditionally true for us, `at_rest` collapses to a
**vibration-only metric**. A smooth 0.35 m/s crawl satisfies it → `ZeroGyroUpdate`
(`control.cpp:159-161`) **fuses live gyro output as gyro bias while moving**.

**Fix stays inside `EKF2.cpp` (already overlaid) — no new overlay file:**

```diff
 		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)
 		    && (vehicle_land_detected.timestamp != 0)) {
-			flags.at_rest = vehicle_land_detected.at_rest;
+			// Rover overlay: RoverLandDetector reports landed == true unconditionally,
+			// so vehicle_land_detected.at_rest degenerates into a vibration-only metric
+			// and can latch true during a smooth low-speed crawl. ZeroGyroUpdate would
+			// then fuse live gyro output as gyro bias while the vehicle is turning.
+			// Require the estimator's own horizontal speed to be near zero as well.
+			flags.at_rest = vehicle_land_detected.at_rest
+					&& (_ekf.getVelocity().xy().norm() < 0.05f);
 			flags.in_air = !vehicle_land_detected.landed;
 		}
```

`_ekf.getVelocity()` is already used in `EKF2.cpp` (`:1551`, `:1676`) — accessor confirmed.

⚠ **Not fixed by this:** `flags.in_air` remains permanently false, which is the
separate C1 GNSS-dropout chain (heading dropout → `yaw_align` cleared →
`vehicle_global_position` stops). That is coupled to `mission_block` waypoint acceptance
and is **not** a last-day change. Recorded as open (§8).

---

### A1 — RoboClaw serial desync never recovers

**The defect:** `readResponse()` (`Roboclaw.cpp:442`) reads exactly N bytes with no
framing, and nothing ever flushes RX on error. One stray byte (a late motor-command ACK)
shifts every later read by one → **every CRC fails forever**. This is the measured
~49/s signature: 2054 `Error reading encoders` + 1229 `Checksum mismatch` in 42 s.

`Roboclaw.cpp` is **already in the overlay.**

```diff
 int Roboclaw::readResponse(Command command, uint8_t *read_buffer, size_t bytes_to_read)
 {
 	size_t total_bytes_read = 0;
 
 	while (total_bytes_read < bytes_to_read) {
 		...
 		if (select_status <= 0) {
-			PX4_ERR("Select timeout %d\n", select_status);
+			// Any failed transaction can leave a partial frame in the RX buffer.
+			// Without a flush the stream stays shifted and every later CRC fails
+			// permanently (observed ~49 failures/s for entire flights).
+			tcflush(_uart_fd, TCIFLUSH);
+			recordTransactionError();
 			return ERROR;
 		}
 
 		int bytes_read = read(_uart_fd, &read_buffer[total_bytes_read], bytes_to_read - total_bytes_read);
 
 		if (bytes_read <= 0) {
-			PX4_ERR("Read timeout %d\n", select_status);
+			tcflush(_uart_fd, TCIFLUSH);
+			recordTransactionError();
 			return ERROR;
 		}
```

Apply the same `tcflush` + `recordTransactionError()` on the CRC-mismatch return path.

**A3 (health visibility) folded in** — the reason a 100%-dead encoder link went unnoticed
for weeks is that the error print was unthrottled and there were no counters. Add:

```cpp
void Roboclaw::recordTransactionError()
{
	_transaction_errors++;
	_consecutive_errors++;

	// Rate-limit: at ~49 failures/s an unthrottled PX4_ERR floods the console
	// and hides everything else.
	if (hrt_elapsed_time(&_last_error_print) > 1_s) {
		PX4_ERR("RoboClaw serial: %" PRIu32 " errors (%" PRIu32 " consecutive)",
			_transaction_errors, _consecutive_errors);
		_last_error_print = hrt_absolute_time();
	}
}
```
Reset `_consecutive_errors = 0` on any successful transaction, and print both counters
in `print_status()`.

Requires `#include <termios.h>` (likely already present for `tcsetattr`) and new members
in `Roboclaw.hpp` (**also already in the overlay**).

---

### A2 — decouple encoder reads from the mixer rate

`readEncoder()` runs every `Run()` (MixingOutput Auto scheduling): **three blocking
serial round-trips per cycle** on the same UART as motor commands and their ACKs.
EKF2 consumes at most 1 sample / 10 ms. `wheel_angle` (cmd 78, `ReadEncoderCounters`)
is read every cycle and **consumed by nobody**.

```diff
 	// Speed reads feed EKF2, which consumes at most 1 sample / 10 ms. Reading them
 	// at the mixer rate triples UART traffic and desync exposure for no benefit.
+	const hrt_abstime now = hrt_absolute_time();
+
+	if (now - _last_encoder_read < 20_ms) {   // ~50 Hz
+		return OK;
+	}
+
+	_last_encoder_read = now;
```

And decimate the counters transaction to 1–5 Hz (or drop it — nothing subscribes to
`wheel_angle`). **A4 (timestamping) folded in:** move `hrt_absolute_time()` to *before*
the first read — currently stamped at `Roboclaw.cpp:265`, **after** all three blocking
reads, which understates `EKF2_WENC_DELAY` by the full transaction time.

---

## 3. Overlay changes (`build_rover.yml`)

Only **one** new `cp` line is needed. Add under "Apply rover patches", keeping the
existing alphabetical-ish grouping:

```yaml
          cp fork_patches/src/modules/ekf2/params_gnss.yaml \
             src/modules/ekf2/params_gnss.yaml
```

Overlay count **26 → 27** (26 confirmed today by `grep -c "^ *cp fork_patches"`).
Update the count in `CLAUDE.md` ("Patch domains") and in `.claude/memory/build.md` in the
same commit that adds the `cp` line, so they cannot drift.

Files touched that are **already** overlaid (no yml change): `EKF2.cpp`, `EKF2.hpp`,
`EKF/common.h`, `src/drivers/roboclaw/Roboclaw.cpp`, `src/drivers/roboclaw/Roboclaw.hpp`.

---

## 4. Commit plan — clean, reviewable git log

One logical change per commit, conventional format, **no AI attribution**.
Run `make format` on all changed C/C++ before each commit (CI enforces `check_format`).

```
1  fix(ekf2): bind GNSS heading noise to new EKF2_GPS_YAW_N parameter

   The GNSS heading observation noise was hardcoded to 0.1 rad (5.73 deg) in
   common.h with no parameter binding. gnss_yaw_control.cpp floors the
   receiver-reported accuracy with fmaxf(yaw_acc, gnss_heading_noise), so a
   UM982 reporting 0.8-1.1 deg had its real accuracy discarded by 5-7x.
   Measured steady-state estimator heading sigma was 1.46 deg with tau 3.1 s,
   which makes sub-1 deg heading unreachable before any control acts.

   Bind the existing parameter to EKF2_GPS_YAW_N. Default stays 0.1 so stock
   behaviour is unchanged until the parameter is set on the vehicle.

2  fix(ekf2): require near-zero speed for at_rest on rover builds

   RoverLandDetector reports landed unconditionally, so at_rest in
   LandDetector.cpp reduces to a vibration-only metric and can latch true
   during a smooth low-speed crawl. ZeroGyroUpdate then fuses live gyro
   output as gyro bias while the vehicle is moving, corrupting the bias
   state during exactly the steady runs being measured.

   Gate at_rest on the estimator's own horizontal speed as well.

3  fix(roboclaw): flush RX and count errors so serial desync can recover

   readResponse() read a fixed byte count with no framing and never flushed
   the RX buffer on error. A single stray byte shifted every later read by
   one and all subsequent CRCs failed permanently: 2054 encoder errors plus
   1229 checksum mismatches over 42 s of flight, which made wheel-encoder
   fusion a silent no-op for weeks.

   Flush on every error path, add consecutive/total error counters, and
   rate-limit the error print so a dead link is visible instead of drowning
   the console.

4  perf(roboclaw): decimate encoder reads to 50 Hz and stamp before reading

   readEncoder() issued three blocking serial round-trips every mixer cycle
   on the UART shared with motor commands, while EKF2 consumes at most one
   sample per 10 ms and nothing subscribes to wheel_angle. Decimate speed
   reads to ~50 Hz, slow the counters transaction, and move the timestamp
   ahead of the blocking reads so EKF2_WENC_DELAY is not understated.

5  ci(rover): overlay params_gnss.yaml for EKF2_GPS_YAW_N

   Adds the 27th overlay file and updates the documented patch count.
```

---

## 5. Timeline — TWO TRACKS, 11:10 start

**FIELD** = at the rover, needs nobody at the Mac. **BENCH** = at the Mac, needs nobody at the rover.
They run concurrently until the flash.

| Time | FIELD track (rover, RTK up) | BENCH track (Mac, CI) |
|---|---|---|
| ~~11:10~~ | ✅ **Pivot sign check CLOSED — offline, no rover.** Every square corner is an in-place rotation; fitting a body-fixed lever arm to 08-04 bags gives \|L\| = 1.34 / 1.51 cm (need ≥120° arc — short arcs give spurious 40 cm). **Sign CORRECT.** | Branch `fw/heading-stability-20260805`. Re-anchor `params_gnss.yaml` on v1.16.2 stock. |
| ~~11:10~~ | ⚠️ **UNPLANNED: RC dead — virtual joystick was holding PX4's manual-control lock** at 48 Hz (`server/manual_control_gateway.py:384` streams NEUTRAL forever when released; app toggle is a no-op). Fixed by `COM_RC_IN_MODE` 2 → **0**. See §8. | — |
| **11:40** | ✅ **Baseline block @ 0.95 — 12 bundles, 11 ULogs.** 4 @ 0.50 + 4 @ 0.35, both directions. Gave the gain measurement (§6a), the direction split (§6 row 4), and §6c. | Commits 1–2 (EKF2). `make format`. |
| **12:00** | ✅ QGC write `RD_MAX_THR_YAW_R` 0.95 → **1.14**. **No `px4-dxp` restart** — verify in QGC + the ULog snapshot, so RTK survives. | Commits 3–4 (RoboClaw). `make format`. |
| **12:15** | **Post block @ 1.14** — same 8 runs. **Gate: steady gain 1.00 ± 0.02, saturation-filtered.** | Commit 5 (overlay + doc counts). Push → CI auto-triggers. |
| **12:45** | **Stop-latch A/B** — `stop_latch_enabled true`, 2×2 square + short-entry line. Watch rest point vs corner-advance tolerance, creep past stops, corner dead time (was 2–6 s). | `gh run watch <id> --repo Vetri2425/PX4-Autopilot --exit-status`. **CI green.** If red, fix and re-push — do not flash a local build. |
| **13:30** | **4-leg direction split** — Aug-II both directions × both speeds. Gates `GPS_YAW_OFFSET` 180.72. | Download artifact, record `board_id` + sha256. |
| **14:15** | — | **FLASH** CubeOrangePlus via QGC. |
| **14:30** | **Flash sanity, params UNCHANGED.** One 0.35 line. **Gate: behaviour identical to the 1.134 post pair — proves the flash is neutral.** | — |
| **14:45** | **F2 tune:** `EKF2_GPS_YAW_N` 0.1 → **0.01**, restart `px4-dxp`. Re-run the same line pair. **Gate: wobble ±4° → ~±1°, EKF−receiver σ → <1°.** | — |
| **15:30** | `RO_YAW_RATE_TH` 1.0 → 0.5 (**only after F2 lands**). Re-run pair. | — |
| **16:00** | Optional: pivot-release pair; full-throttle MANUAL run to settle `RO_MAX_THR_SPEED` (§6b). | — |
| **16:30** | **Day-close gate: 0.5 m/s straight ≤2 cm RMS + PHYSICAL TAPE CHECK** (budget 4 runs — no log substitutes). | — |

**Standing rules for the whole day:**
- ULog **every** run. `tools/analyze_bag_ulog.py --ulog` is the authoritative param + gain source.
- After **any** QGC write, restart `px4-dxp` before verifying — the `/mavros/param` mirror
  freezes at MAVROS init (stale-mirror trap).
- Check `armed` + mission state before **any** `rpp-pipeline` restart. Dropping setpoints in
  OFFBOARD trips the failsafe, and `NAV_RCL_ACT`/`NAV_DLL_ACT` are both **Disarm**.

---

## 6. Param tuning order — corrected against the live FCU

PX4's own rover tuning guide gives the canonical sequence, and it matters here because
**tuning feedforward with the feedback loop live is not identifiable**:

> "To tune this parameter, first make sure you set `RO_YAW_RATE_P` and `RO_YAW_RATE_I`
> to zero. This way the yaw rate is only controlled by the feed-forward term."
> — [PX4 Rate Tuning](https://docs.px4.io/main/en/config_rover/rate_tuning)

We do **not** follow that literally — `analyze_bag_ulog.py` identifies the plant from
closed-loop data with Kp in the model (§6a), which is why zeroing the gains is unnecessary.

| # | Param | change | Needs flash? | Gate |
|---|---|---|---|---|
| 1 | `RD_MAX_THR_YAW_R` | 0.95 → **1.14** ✅ **APPLIED 08-05** | no | ulog steady gain **1.00 ± 0.02** — post-change block running |
| 2 | `EKF2_GPS_YAW_N` | (created by F2) → **0.01** | **YES** | wobble ±4° → ~±1° |
| 3 | `RO_YAW_RATE_TH` | 1.0 → **0.5** | after #2 | deadband = `TH/RO_YAW_P` = 0.67° eats 2/3 of a 1° budget |
| 4 | `GPS_YAW_OFFSET` | 180 → 180.72 | no — ✅ **GATE SATISFIED 08-05** | 4-leg split gives body-fixed 0.60–0.93°, brackets +0.72° ± 0.17 |
| 5 | `RO_YAW_RATE_LIM` | 22 → ? | no — **DO NOT TOUCH YET** | §6c — new 0.5 m/s blocker, needs its own A/B |
| ~~—~~ | ~~`EKF2_GPS_P_NOISE` → 0.015~~ | **ALREADY 0.015** | — | **struck — verified live 08-05** |
| ~~—~~ | ~~`EKF2_GPS_V_NOISE` → 0.05~~ | **ALREADY 0.05** | — | **struck — verified live 08-05** |
| — | `RO_MAX_THR_SPEED` | **HOLD at 1.28** | — | see §6b — tool and field measurement disagree |

**Do not touch** `RO_YAW_RATE_I` (0.01 → 0.05) until step 1 lands — single integrator in
the rate loop, never before FF calibration.

**`RO_YAW_RATE_TH` semantics verified today** in v1.16.2 source: it is a genuine deadband
on **both** ends — `DifferentialRateControl.cpp:72` zeroes the *measured* yaw rate below
threshold, `:149` zeroes the *setpoint*. Firmware default is 3.0°/s
(`rovercontrol_params.c:65`); we are already at 1.0. Gating the drop to 0.5 behind F2 is
correct: at σψ 1.45–2.88° a tighter deadband only lets more heading noise into the rate loop.

### 6a. Why 1.134 and not 1.06 — `RD_MAX_THR_YAW_R`

**1.06 was wrong and is superseded.** It came from `0.95 × 1.11` — taking the measured
11% over-rotation and scaling the current parameter by it. That arithmetic silently
assumes the loop is **pure feedforward**, so that a 11% excess in the output implies an
11% excess in the FF gain.

It isn't. `RO_YAW_RATE_P = 0.13` is live (**confirmed on the FCU today**), and the
proportional term **already claws back part of the error** — the measured gain of 1.119 is
the closed-loop result *after* Kp has acted, not the raw FF error. Correcting the FF alone
therefore needs a **larger** move than the naive ratio suggests, because part of the
correction you observe is being supplied by feedback that will keep supplying it afterwards.

The correct relation, fitted per run by `tools/analyze_bag_ulog.py`:

```
G = A(WT/2R + Kp) / (1 + A·Kp)          →   G = 1  gives   R_opt = A·WT/2
```

where `A` is the identified plant gain and `WT` = `RD_WHEEL_TRACK` = 0.47 (**confirmed live**).
This is licensed by **R3 residual = 0.00000** on all 9 logs — the firmware rate/FF model
reproduces the logged `normalized_speed_diff` exactly, so `R_opt` is *derived*, not guessed.

**Independently re-derived 2026-08-05 and self-consistent:** substituting `R = A·WT/2`
collapses `G` to exactly 1 algebraically; and evaluating the forward direction at the
*current* `R = 0.95` with A=4.826, Kp=0.13, WT=0.47 predicts **G = 1.1192** against a
measured **1.119 ± 0.011**. The model reproduces the observation, it does not merely fit it.

Cross-run result over the 9 runs flown on the current config:

| | value |
|---|---|
| steady yaw gain G | 1.119 ± 0.011 |
| plant gain A | 4.826 ± 0.090 (ideal 5.447) |
| **`RD_MAX_THR_YAW_R` R_opt** | **1.134 ± 0.021**, range 1.086–1.155 |

Reproduce with:

```sh
# NOTE: pyulog lives in the Homebrew interpreter, NOT in the ros-replay env
#       (ros-replay runs the TESTS; it cannot run the tool).
/opt/homebrew/bin/python3 tools/analyze_bag_ulog.py \
    --sweep --ulog-dir ~/Documents/QGroundControl\ Daily/Logs \
    --emit-params rec.params

# after each A/B, the acceptance check:
/opt/homebrew/bin/python3 tools/analyze_bag_ulog.py \
    --ulog <new.ulg> --json post.json
# then read post.json: plant.G must be 1.00 +- 0.02
```

The tool is committed at **`fceb96b`** in `PX4_DXP` (`tools/analyze_bag_ulog.py` +
`tools/test_analyze_bag_ulog.py`, 53 tests) and is now on the Jetson too (@`2cdf78b`).
`--json` emits the whole result set — reconstruction verdicts, plant fit, dead-band
occupancy, geometry, recommendations with their confidence — so the acceptance check can be
read programmatically instead of parsed out of the text report.

The sweep keeps only the **most recent parameter configuration** (9 runs) and sets aside
24 runs flown with 6 older configs — a gain is a property of the config it flew with, so
mixing them produces a meaningless aggregate.

⚠ `1.134` is a **fitted mean with a real spread** (±0.021). Treat the first A/B as
confirming the direction and magnitude, not as a final value. The acceptance criterion is
unchanged and is what actually decides: **post-change steady gain 1.00 ± 0.02.**

### ✅ MEASURED IN THE FIELD 2026-08-05 — 1.140 ± 0.013, scrub caveat CLOSED

11 fresh ULogs + 12 bags, same day, post-remount, all at `RD_MAX_THR_YAW_R` = 0.95.
The raw `--sweep` aggregate (G 1.067 ± 0.080, R_opt 1.035 ± 0.155) is **NOT actionable** —
it averages runs where the measurement is invalid. Filter on **`RO_YAW_RATE_LIM`
saturation** and the data is perfectly bimodal:

| | n | G | R_opt | A cross-check | wheel/gyro |
|---|---|---|---|---|---|
| **clean (0 % sat)** | 7 | **1.1200 ± 0.0086** | **1.140 ± 0.013** | ≤4 % | 0.999–1.016 |
| saturated (8–29 %) | 4 | 0.9731 ± 0.0711 | 0.705–0.968 | **26–61 %** | 0.887–1.076 |

Once the limiter clips the setpoint, measured/commanded is the **limiter**, not the
feedforward, so G reads low. Always filter on `§9 RO_YAW_RATE_LIM ... hit X%` before
quoting G; the tool's own A cross-check (§10) independently flags the same four runs.

⇒ **1.140 ± 0.013 (n=7, today) vs 1.134 ± 0.021 (n=9, yesterday) — within 0.3σ.**
Two separate days, separate flights, same answer. **Applied: 0.95 → 1.14.**

**The scrub caveat is CLOSED — a scalar is correct, no gain schedule needed:**

```
0.50 m/s (n=3):  G 1.1213   R_opt 1.141
0.35 m/s (n=4):  G 1.1191   R_opt 1.139     difference 0.20 % of G
```

`A` is not turn-rate dependent in the unsaturated regime.

### 6c. `RO_YAW_RATE_LIM` — a measurement FILTER, not a failure cause

⚠ **Earlier claim RETRACTED.** The pre block showed saturation correlating 4/4 with
failures, which read as causal. The post block **saturates on 9 of 9 runs and every one
completed**, with tracking improving. Saturation was a *symptom* of a rover already off
track and correcting hard, not the mechanism.

It stays essential for **measurement**: above 14.7° heading error the setpoint is clipped
at 22 °/s, and measured/commanded then reports the LIMITER, not the feedforward. Filter on
it before quoting any gain. **Do not raise `RO_YAW_RATE_LIM`** — it was deliberately
lowered 90 → 22 in the retune that fixed physical repeatability, and tracking is now
*better* with it binding ~20 % of the time.

### 6d. POST-CHANGE VERIFICATION 08-05 — derived from the firmware source

Re-derived independently of `analyze_bag_ulog.py`, transcribing `RoverControl.cpp
rateControl()` and `DifferentialRateControl.cpp` at v1.16.2:

```
u = clamp( clamp(adj·WT/2R, ±1) + Kp·(adj − ω) + I , ±1 )
```
`math::interpolate` CLAMPS (`Functions.hpp`); `pid_yaw_rate_integral` is added **raw**,
not I-scaled. **Reconstruction vs logged `normalized_speed_diff`: RMS 0.00000.**

| | PRE R=0.95 | POST R=1.14 |
|---|---|---|
| usable runs | 7 of 11 (sat<0.5 %) | 9 of 9 |
| **plant gain A** | **4.843 ± 0.063** | **4.888 ± 0.214** |
| steady gain G | 1.1206 ± 0.0084 | **1.0346 ± 0.0246** |
| R_opt = A·WT/2 | 1.138 ± 0.015 | 1.148 ± 0.050 |

⭐ **`A` is INVARIANT across the change** — a physical drivetrain property that must not
move when a control gain does. It didn't, while `G` moved as designed. That is the check
proving the identification measures the plant and not itself.

Inverse-variance pooled **A = 4.847 ⇒ R_opt = 1.139 ⇒ the applied 1.14 is correct to
0.1 %. KEEP IT.**

⚠ Gate (`G` = 1.00 ± 0.02) **marginally missed**: 1.0346 ± 0.0246; over-rotation 12.1 % →
3.5 %. The A-derived model predicts 0.999 at R = 1.14, so the direct `G` fit and the A
prediction disagree by ~3.5 % — **unexplained**. Integral wind-up was the obvious
candidate and is ruled out (I-share 17.9 % → 5.8 %). Suspect scatter in a through-origin
fit on 35–87 surviving samples vs 111–186 pre.
🚫 **Do not chase it with 1.14 × 1.035 ≈ 1.18** — that is the same closed-loop error that
produced the discarded 1.06. `R_opt = A·WT/2` contains no `G`.

**Only 2 of 880 params differed between blocks** (`RD_MAX_THR_YAW_R`, `COM_FLIGHT_UUID`),
confirmed by diffing the two ULog snapshots. Do that before attributing any A/B result.

### 6b. `RO_MAX_THR_SPEED` — HOLD, do not change today

An earlier version of the tool recommended `1.28 → 1.089` (single run) / `1.130 ± 0.044`
(cross-run), on the argument that `A_ideal = 2K/WT`, so an overstated full-throttle speed
makes the plant look 11% lossy — one calibration error appearing in both loops.

**That conflicts with a closed field measurement:** full throttle was measured at
**≈1.28 m/s** (prereq CLOSED 2026-08-01), which is where the current value came from.

**RESOLVED 2026-08-05 — the field measurement wins, and the tool now says so itself.**
Measured on log_15: the fit spans throttle **0.081–0.291, i.e. 29% of the range**, with a
real friction intercept, and extrapolates to **1.170 m/s** at full throttle. A slope fitted
over the bottom third of the range with a genuine offset is not evidence about the top.
`analyze_bag_ulog.py` now records the throttle span and **WITHHOLDS the recommendation
unless the fit reaches ≥0.60 throttle**, printing `WITHHELD (extrapolation)`; the value is
excluded from `--emit-params`. Two paired tests pin this.

**Why the yaw number survives on the same data:** the friction intercept is **common-mode**
— it affects both wheels equally, so it **cancels in the wheel-speed *difference*** that
generates yaw. The yaw path therefore depends only on the local *slope*, which is what is
measured here. That is why `R_opt` is firm while `RO_MAX_THR_SPEED` is not, and it is now
printed as the explanation rather than left as a coincidence.

Reasons to hold:
- It contradicts a direct measurement that was deliberately closed.
- It is **not** a cheap A/B: `RO_MAX_THR_SPEED` feeds the throttle slew (`RO_ACCEL_LIM`/K)
  and the normalised infeasibility clamp across the entire speed range.
- `R_opt` is fitted from data and **does not depend on it**, so holding costs step 1 nothing.

**Cheap way to close it today (16:00 slot, optional):** a full-throttle straight run in
MANUAL is a ~10 s test and gives the top of the range directly, which is exactly the
evidence the fit lacks. That is a *measurement*, not a tuning change — it settles §6b
without touching the param.

**v1.16.2 trap:** the PX4 guide tells you to adjust `RO_YAW_RATE_CORR`. **That param does
not exist in v1.16.2** — it is v1.17+. On our firmware the FF knob is `RD_MAX_THR_YAW_R`
(FF = ψ̇·wt/2 / `RD_MAX_THR_YAW_R`, verified in v1.16.2 `RoverControl.cpp rateControl()`).
Same inverted law as `RO_MAX_THR_SPEED`: **raise it to turn less.**

Note the guide's starting heuristic ("half the full-throttle ground speed") gives
1.28/2 = 0.64, well below our 0.95. The fitted **1.134** (§6a) is authoritative — the
heuristic is only a seed.

**Dead knobs — do not spend field time:** `EKF2_HEAD_NOISE` is a **no-op** (binds
`mag_heading_noise`, `EKF2.cpp:132`). `EKF2_GPS_YAW_OFF` is **dead code** (the driver's
`GPS_YAW_OFFSET` is the working knob). `RO_SPEED_P`/`RO_SPEED_I`/`RO_ACCEL_LIM` **never
act in OFFBOARD** — `speed_body_x_setpoint` is identically zero; speed is open-loop
feedforward with a real −0.05 m/s rolling-resistance intercept.

---

## 7. WENC — stays OFF today

`EKF2_WENC_CTRL = 0` on the vehicle (**re-verified live 08-05**). **Do not enable it today.**
⚠ `CLAUDE.md` still documents `EKF2_WENC_CTRL = 1` — that row is **stale and must be fixed.**

- 07-27 A/B, straight 3.07 m line: **WENC=1 walks left 1.27–1.51 cm/m, ends 3–4.5 cm off**,
  persistent and one-sided. WENC=0 error is *transient* — an 8.4 cm start error decays to 0.5 cm.
- The filter **hides it**: WENC=1 self-reported 1.29 cm RMS vs **3.55 cm true** (2.75×
  understatement). You lose the ability to see the problem in telemetry.
- Mechanism: body-frame velocity fusion pulls v_NED onto the *estimated* heading at
  `v·sin(ψ_err)`. At ψ_err = 1.1° that is **1.92 cm/m** — measured 1.77–2.09.

**WENC converts heading error into position error.** Heading is precisely what is broken
today. Enabling it before F2 lands takes the worst term and hard-couples it into position.

WENC becomes worth re-testing only after: **A1** (else it fuses nothing) → **A9** (else the
EKF-GSF yaw rescue can never fire — `fuseBodyFrameVelocity` refreshes the *global*
`_time_last_hor_vel_fuse`) → **F2 + heading bias measured** → **A12 params**
(`LAT_N`→0.35, `GATE`→5). A1 ships today; the rest do not.

It is a **margin** tool, not an accuracy tool: at 5 Hz RTK there is 200 ms between fixes —
10 cm of travel at 0.5 m/s with no position update. That is what WENC is for, and the
audit calls it required for a *durable* 1 cm. Not for reaching 1 cm.

---

## 8. Open items in memory — full ledger

### Firmware, not shipping today
| ID | Item | Why open |
|---|---|---|
| **F7** | `s_variance_m_s` variance-vs-σ units bug; silently disables `EKF2_REQ_SACC` | **submodule** — overlay can't reach it (verified `.gitmodules:13`) |
| **F8** | `-m config` to pin 30/50 Hz; today 30 Hz exists only because MAVROS asks | optional, new ROMFS overlay |
| **C1** | `in_air` permanently false → heading dropout clears `yaw_align` → `vehicle_global_position` **stops** | coupled to `mission_block`; not a last-day change |
| **C2** | NMEA driver restart on 500 ms quiet → multi-second `sensor_gps` blackouts | submodule |
| **A5** | WENC lever arm `EKF2_WENC_POS_*` | WENC off |
| **A6–A11** | no-slip constraint, slip detector, per-wheel calib, fault path, per-axis gating | WENC off |
| **B** | Manual-mode steering has no slew (throttle does) | separate workstream |
| — | Cold-boot RoboClaw death (boot-retry reverted, run#23) | open since 05-22 |

### Structural — no param or small patch fixes these
- **2.8° crab is uncorrectable inside PX4 today.** Velocity OFFBOARD derives yaw from
  `atan2(vel)` only (`DifferentialVelControl.cpp:141-166`) — the companion has **zero**
  independent heading authority. Needs an `RD_CRAB_OFF` param added to the bearing, or
  attitude+velocity offboard. This is the largest single remaining term (`e_ss = Ld·tanβ ≈ 1.7 cm`).
- **Velocity OFFBOARD discards `yawspeed`** — `/rpp/yaw_rate_body` never reaches the rate
  loop; `yaw_rate_feedback_gain` is inert. Only `body_rate` mode consumes it.
- **Ld trap:** raising lookahead fixes swing (+14° PM) but **doubles** the crab offset.
  Biases first, then Ld.
- **Circles at 0.5 m/s saturate the yaw envelope** (0.45 rad/s clamp hit on 49–57% of ticks
  at R=1.5 m). Needs a κ→yaw-rate speed cap, companion-side. **Keep circles at 0.35.**

### Companion — deployed but unproven
- **Stop-latch** (`8ee84cd`, deployed @`2cdf78b`, `stop_latch_enabled` default **False**) —
  **A/B is on today's field track, 12:45.** Replay says the `…200723` PRE_CORNER strand is
  eliminated and good runs are byte-identical; the field decides the default.
  ⚠ Watch: rest point must land **inside** the corner-advance tolerance. If the rover strands
  latched-at-zero 2–3 cm short, the 0.10 nudge is not firing — knob is `stop_latch_capture_dist_m`.
- **Pivot-release pair** — `RD_TRANS_TRN_DRV` 2°→1° **and** companion tol 3°→2° (must stay
  ~1° above the firmware stop angle; equal = deadlock, 6/6 on 07-30). Judge on release
  **xtrack**, not exit angle: the 08-03 A/B halved exit error and marked 57% worse.
- Not today: D3 latency bias (`pose_latency_bias_s=0`, should be ~0.03–0.05), D4 crab
  integrator, D6 `use_feedforward_yaw_rate` A/B, D10 simplifier tol 0.01 → 0.003.

---

## 9. Acceptance criteria

| Check | Pass |
|---|---|
| Pivot sign | <1 cm wander during in-place 180° pivot |
| `RD_MAX_THR_YAW_R` 0.95 → 1.14 | ⚠️ **marginal** — G 1.0346 ± 0.0246 vs gate 1.00 ± 0.02. KEEP: A-invariance + R_opt = 1.139 carry it (§6d) |
| **Day gate on p95** | ❌ **NOT met** — p95 3.0–3.6 cm, max ~5 cm. RMS spec met, tail spec not. |
| Stop-latch | no creep past stops; rest inside corner-advance tolerance; corner dead time < previous 2–6 s |
| CI build | green; artifact sha256 + `board_id` recorded |
| Flash sanity (params unchanged) | behaviour identical to the pre-flash 1.134 pair — flash is neutral |
| F2 effect | yaw wobble ±4° → **~±1°**; EKF−receiver σ 1.45–2.88° → <1° |
| **Day gate** | **0.5 m/s straight ≤2 cm RMS + physical tape check** (budget 4 runs) |

### Measured result of the day — FULL-MISSION, 9 completed runs per block

⚠ **Report `analysis.tracking.overall`, not `marking_only` and NEVER
`geometry.xtrack_vs_planned`.** The latter's sample count collapses to n = 12–54 on some
runs (manufacturing 0.08–0.46 cm scores against n = 1576 on others) and produced a bogus
"−50 %" headline. Always print `n` beside an xtrack figure.

| | R = 0.95 | R = 1.14 | Δ |
|---|---|---|---|
| **full-mission RMS** | **1.91 cm** | **1.69 cm** | −12 % |
| median / worst run | 1.96 / 2.64 | 1.65 / 2.08 | −16 / −21 % |
| spread across runs | 1.45–2.64 | 1.19–2.08 | tighter |
| p95 (mean) | 3.45 | 3.00 | −13 % |
| **max excursion** (mean / worst) | 4.66 / 5.96 | **3.80 / 4.95** | −18 % |
| marking-only RMS | 1.89 | 1.66 | −12 % |
| aborts + runaways | 3 + 1 | **0** | — |
| worst coast | 29.8 cm | 2.4 cm | — |

**The tail is the remaining problem, not the RMS.** p95 sits at 3.0–3.6 cm and single
excursions still reach ~5 cm. **The 1–2 cm whole-mission spec is met on RMS but NOT on
p95.** F2 is the next lever — it targets exactly the noise-excited swing that produces
those excursions.

**Reference point:** square is already **1.01–1.17 cm** across seven runs at 0.35, circle
**1.55 cm**. The 1–2 cm target is met at 0.35 today. The job is holding it at **0.5**.

**1 cm has zero margin** — the physical floor of a 1.5 cm RTK receiver at 5 Hz is ~0.9 cm.
1–2 cm durable is the honest spec.

---

## 10. Rollback

Every commit is independent and revertable. If the flash regresses:
1. `EKF2_GPS_YAW_N` → 0.1 restores stock estimator behaviour **without reflashing**
   (this is why the default is 0.1).
2. `RD_MAX_THR_YAW_R` → 0.95.
3. Full rollback: reflash the `06309e41a7` artifact from CI run #27.

Companion-side: `stop_latch_enabled false` reverts the stop behaviour with no restart of
anything, and it is already the default.

⚠ Firmware built locally vs in CI is functionally equivalent but **not bit-identical**
(different toolchains). **Flash CI artifacts only.**

---

## Sources

- [PX4 Rate Tuning (rover)](https://docs.px4.io/main/en/config_rover/rate_tuning)
- [PX4 Configuration/Tuning — Differential Rover](https://docs.px4.io/v1.16/en/config_rover/differential.html)
- [PX4 Attitude Tuning (rover)](https://docs.px4.io/main/en/config_rover/attitude_tuning)
- [PX4 Using the ECL EKF (v1.16)](https://docs.px4.io/v1.16/en/advanced_config/tuning_the_ecl_ekf.html)
