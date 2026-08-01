# Bug Closure Checklist

**Compiled:** 2026-07-22 · **Rev:** 2026-08-01 session close (register audited against 07-25→08-01 field evidence + today's 15 bags)
**Tree:** `Upgrade_speed` @ `cfbf5f3` (= `baseline_master`) · **⚠ NOT yet on the Jetson — see D6**
**Method:** an item is only closed by evidence, never by "the code looks right." Three bugs once
shipped because a test's ground truth mirrored the bug it was testing.

> **This revision removes the full text of closed items** (ledger below keeps the pointer) and
> registers the 2026-08-01 findings as the **D-series**. The B-series physical-measurement
> cluster is now the oldest open work on the vehicle.

---

## CLOSED LEDGER (evidence-closed; full text in git history of this file)

| Item | What it was | Closed by |
|---|---|---|
| A1 | `source_file` empty → §8 dead | `e3939e3`, Jetson-verified 07-22 |
| A2 | `must_hit` lost on non-staged loads | `21a8b05`; field step OBE — flow is staged-only by rule since 07-29 |
| A4 | `outcome` lied on partial runs | `5f820be` `traversal` block; populated in every bundle since (15/15 today) |
| A3 | EKF resets unnoticed | jump-guard rework `a1df503` + `test_ekf_reset_compensation.py`; induced-reset field demo never run (accepted) |
| A5 | no `/spray/session_config` subscription | Spray DASH/B0 transport landed (`d2b1b9e` lineage) |
| A7 | cross-session origin drift | `GPS_SURVEYED` staging re-derives from surveyed anchors + stale-origin fix `779d9f0`; literal SET_GPS_GLOBAL_ORIGIN pinning never done, no longer carries the error |
| A9 | must_hit lost in multi-entity joins | fixed 07-23 + 3 regression tests; field step folds into the next multi-entity survey |
| survey-tol | analyser tolerance hardcoded | `7d77565`; **UI wiring still open → moved to F3** |
| C1 | placement non-determinism | closed 07-22 bit-identical; re-proven 08-01 (0.000 mm shape residual) |
| C2 | survey CSV never field-driven | CSV missions driven 07-27+; live harness 12/12 on 08-01 |
| C7 | endpoint rests 0.9–2.0 cm short | **obsolete** — endpoint architecture rebuilt (`3e7e300`/`64c096b`): paint end 0.1–1.7 cm, resting deliberately in the run-out |
| C8 | per-run wander RMS 1.1–1.8 | lookahead retune `cfbf5f3`: total paint RMS 0.66–0.98 @0.6 m/s, ripple ±1 cm |
| B2 | wheel-scale asymmetry fused with confidence | **moot** — `EKF2_WENC_CTRL=0` since 08-01 evening; premise ("noise 0.1") was also retracted (live was 0.35) |

---

## Priority 0 — Deploy gate (blocks everything else)

### ☐ D6 — `cfbf5f3` defaults have never executed on the vehicle
- **State:** lookahead/speed defaults (`min_lookahead_dist 0.35`, `lookahead_time 1.0`,
  `mission_speed 0.5`) promoted in node + server registry + CLAUDE.md; Jetson went offline
  before deploy. Jetson is still on the deleted-locally fix branch @`1cb229f` with runtime params.
- **Hazard until deployed:** any `rpp-pipeline` restart reverts the live node to OLD defaults
  (0.52 / 1.6 / 0.70) — the validated behaviour silently disappears.
- **Verify:** Jetson `git fetch && git checkout Upgrade_speed` → restart services (NTRIP ritual:
  `POST /api/rtk/ntrip/start` + re-login) → `make jetson-tests` (482 src suite — **not run on
  this commit**) → one smoke mission; bag `debug[13]/[15]/[38]` reads 0.35 / 1.0 / 0.5 **without
  any runtime set**.
- **Closes when:** the smoke bag proves defaults-as-defaults and the src suite is green on `cfbf5f3`.

---

## Priority 1 — The physical measurement cluster (oldest open work; tape gates all three)

### ☐ B5 — Both-direction tape / physical re-survey — **4th day owed, the deciding measurement**
- Survey the painted marks (RS3, averaging 10–30 s/point) or run the tape **both directions**.
- **Why it decides:** 08-01 signed-bias decode — NE-line runs lean right (4/4, −0.6…−1.8 cm),
  SW-line runs lean left-of-zero on average; both lean the **same SE ground side** ⇒ a
  ground-frame disturbance (slope / antenna lean; roll 2–3° measured constant) that the
  controller's xtrack is structurally half-blind to. No log can substitute.
- **Closes when:** paint-vs-design is quantified in both directions and the offset is attributed
  (body-fixed lever arm vs ground-frame lean vs placement).

### ☐ B3 — Antenna lever arm (`EKF2_GPS_POS_Y = 0` is an assertion, not a measurement)
- Blocked by B5's numbers. Physically measure primary-antenna offset from centreline; enter or
  confirm Y=0. The ±2.2 cm heading-flip signature (8/8 runs, 07-31 global decode) is the
  standing evidence that something body-fixed is real.

### ☐ B1 — Heading offset measured, not assumed — **downgraded**
- The 180° is correct in principle; the motivating "5 of 6 drove left" is stale (08-01: no stable
  one-sided bias). The <0.5° stationary residual test remains undone; do it opportunistically
  when parked on the surveyed line during B5.

### ☐ B4 — Nozzle offset (both offsets still 0.0)
- Every cm-level number to date describes the **antenna**. Measure nozzle forward + lateral vs
  antenna; §3.6 xtrack-gate fix is **mandatory before any non-zero offset**.
- Feeds the spray-node tuning session (F-series companion: A6 plan Rev 4).

---

## Priority 2 — D-series: registered 2026-08-01 (today's bags)

### ☐ D1 — Pivot walk 0.6–4.8 cm — largest unaddressed physical error term
- "In-place" pivots translate (tonight: up to 4.8 cm, consistent −4.2 dN on the 20:5x set;
  4.57 cm in the 165426 error budget). Architectural: PX4 derives heading from the
  velocity-vector bearing — rotation is bought with translation; yaw-only does not turn this
  rover (pivot A/B 08-01: tightening produced a 23 s stall, walk unchanged).
- **Mitigated, not fixed:** at 0.6 m/s the speed-scaled lookahead converges the walk inside the
  0.5 m pre-extension (spray-ON 0.33–1.14 cm from the stake). Extension-less missions still eat
  it whole.
- **Closes when:** either compensated (feed-forward of the known walk into the approach stop
  target) or formally accepted with the extension prerequisite documented.

### ☐ D2 — Approach-stop scatter 2–4 cm, direction-dependent
- Same mission, same tolerance: stop error 1.25–3.76 cm across 8 approaches, direction varies
  with where the approach came from. Combines with D1 to set the paint-entry error.
- **Closes when:** characterised across ≥10 approaches from ≥3 directions, or made irrelevant by
  D1's compensation.

### ☐ D3 — Terminal deceleration happens inside the paint
- Braking zone (`approach_velocity_scaling_dist` 0.9 m floor) > 0.5 m aft extension ⇒ decel
  starts ~0.4 m before paint end; with L=0.35 config this wiggles −2…−3.7 cm at 3.76–3.80 m
  while spraying (2 of 3 runs @0.35; smaller at 0.6). **1.0 m extensions rejected by operator.**
- **Candidate levers (untested):** speed-derived shrink of the 0.9 m floor at low mission_speed;
  decouple mark-end decel from run-out length.
- **Closes when:** last-0.5 m painted xtrack ≤ 1.5 cm without lengthening the extension.

### ☐ D4 — `xy_goal_tolerance` is global; 0.01 deadlocks run boundaries
- 0.01 gave a 0.5–0.9 cm approach stop (excellent) but the paints-nowhere pre-extension run uses
  the same tolerance at its boundary → 50 s hunting at the paint start, no paint, e-stop
  (bag 200257). A tighter approach needs a **per-leg tolerance** (code change in
  `_goal_tol_effective` / run-advance check). Until then 0.02 stands.
- **Closes when:** per-leg tolerance lands with a named A/B, or the idea is formally dropped.

### ☐ D5 — App param-push silently reverts runtime params between missions
- Observed live: `xy_goal_tolerance` runtime set at 20:00 was back to registry default by the
  20:05 mission, no restart involved (server registry push wins). Runtime A/Bs are only
  trustworthy through bag `debug[13]/[15]/[38]` as-run readback.
- **Closes when:** the push is documented + surfaced (log line or API echo of what was pushed),
  or params get a provenance/lock mechanism.

### ☐ D7 — 0.8 m/s rung (declared max) untested
- Accel to 0.8 needs 0.91 m ≈ 2× the pre-extension ⇒ paint opens ~0.66 m/s still accelerating;
  entry cost expected but unquantified. Throttle fine (62% of 1.28 calibration).
- **Closes when:** 3 runs @0.8 decoded; keep-or-cap decision recorded against the 0.6 baseline
  (0.66–0.98 cm RMS).

### ☐ C5 — segment↔smooth seam (carried; still unverifiable)
- 08-01 curve bag's only profile flip is at path load (xtrack NaN) — no mid-drive seam
  exercised. Needs a mission whose profile flips while driving.

### ☐ C6 — Spray pivot-state gate: near-closed
- No pivot overspray in any of today's 15 decoded runs; valve opened at cruise in every
  extension run. Formal closure wants one deliberate pivot-adjacent spray check.

### ☐ C3 / C4 — POINT snapping on a dense real drawing / §8 on a fresh file-sourced bundle
- Both blocked on missions that carry a source file — today's point-missions stage with
  `source_file: null`, so §8 legitimately skips. Fold into the next DXF/CSV field day.

---

## Priority 3 — Server / app / infra (F-series + carried)

### ☐ A11 — `mark=0` transit points come back painted (HIGH, from 07-2x register)
### ☐ A8 — CPU executor mismatch (`MultiThreadedExecutor` + exclusive group = zero parallelism)
### ☐ A6 — Spray plan Rev 4 (contract stale vs `6523a84`) — **prerequisite for spray-node tuning**
### ☐ F1 — App: projected-CS CSV export → 8 points at Null Island with warnings identical to a good file
### ☐ F2 — App: `#1`-style point names silently deleted on import
### ☐ F3 — App: `survey_tolerance_m` not sent (analyser runs on unattributed default)
### ☐ F4 — Spray start-delay vs speed A/B — only meaningful on extension-less missions now
  (with extensions the valve opens at cruise; 165426 showed stationary opening without them)

### ☐ F5 — px4-dxp restart takes ~11 s; ~6–7 s of it is script overhead, not MAVROS
- **Decomposition (from `px4_start_service.sh` structure; journal-verify before fixing):**
  real MAVROS/FCU work ≈ 4–5 s; the rest is two unconditional `sleep 1`s (post-pkill
  line ~199, post-free_port line ~130), a 1 Hz ready-poll that **spawns a fresh
  Python+rclpy process per tick** (~0.5–1.5 s each on the Orin), `ros2 param set` via the
  stale-after-restart CLI daemon in `apply_gcs_heartbeat`, a 1 s-tick flag poll, and one
  more cold rclpy spawn for FCU validation.
- **Why it matters:** the restart drops MAVROS + cascades rpp-pipeline; every second of it
  is OFFBOARD/QGC-bridge outage, and the restart-during-mission hazard window scales with it.
- **Fix (ranked):** (1) one persistent readiness helper — extend
  `tools/ros2_mavros_health.py` to wait-for-node → retype heartbeat via rclpy param client
  → confirm `connected`, single process, 0.1 s internal polling; (2) make both fixed sleeps
  conditional on having actually killed/freed something; (3) 0.1 s flag-poll ticks;
  (4) `sd_notify` READY + `Type=notify` so dependents start at true readiness (also
  unlocks the disabled `WatchdogSec`). Expected ~11 s → ~4–5 s.
- **Verify:** `journalctl -u px4-dxp` timestamps, before vs after, 3 restarts each,
  rover disarmed. Numbers above are structural estimates until then.
- **Closes when:** measured restart ≤ 6 s with FCU connected and all dependents healthy.

---

## Standing rules for any verification run

1. **Record coverage** — partial-run vertex misses are meaningless at the uncovered end.
2. **Measure pose vs raw `/path`** — never `/rpp/debug[0]` alone, never `/spray/debug[5]`.
3. **Decompose A (plan vs truth) / B (driven vs plan)** before blaming a subsystem.
4. **Repeat 3×; conclude at 10×.** Bimodal = marginal stability, never "noise".
5. **Extensions on** for anything judging paint quality; extension-less runs eat D1+D2 whole.
6. **GPSRAW is 2–8 cm off `global` while moving** (identical parked) — never reference it for
   moving events; never gate on `eph`.
7. **As-run params come from the bag** (`debug[11..46]`), never the manifest (`rpp_params`
   off-by-2) and never from memory of what was set (D5).

## Known-good yardstick (2026-08-01, `stg_a456f3a7`, 0.6 m/s, defaults-equivalent config)

```
paint RMS                 0.66 – 0.98 cm      (3/3 PASS, short 3.3 m line)
paint entry offset        ≤ 1.3 cm            (extension converges the pivot walk)
spray-ON vs stake         0.33 – 1.14 cm      (1 outlier 4.63 with a −1.1 entry)
spray-OFF vs stake        0.76 – 2.04 cm
endpoint resting          6.6 – 8.2 cm into the run-out (by design)
pivot walk                0.8 – 4.8 cm        (D1 — absorbed by the extension at 0.6)
curve (n=1)               RMS 1.56, max 3.18
```

A regression is any run materially outside these **at the new defaults**.
