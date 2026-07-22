# Bug Closure Checklist

**Compiled:** 2026-07-22 · **Tree:** `Upgrade_Spray` @ `dd166b3` (Jetson matches) · 669 tests pass
**Companion doc:** `docs/OPEN_BUGS.pdf` (the register) · **Method:** skill `analyse-missions`

Each item states what must be **true**, how to **prove** it, and what result **closes** it.
An item is only closed by evidence, never by "the code looks right" — this session produced
three bugs that shipped because a test's ground truth mirrored the bug it was testing.

---

## Priority 0 — Do before the next run batch

These three are small, and all of them currently corrupt the data you collect. Running more
missions before they land means re-analysing everything afterwards.

### ☐ A1 — `source_file` populates, §8 runs automatically
- **Cause:** `routes/path.py:1179` writes `"source": result["source"]` (a filename **string**);
  `bag_autorecord.py:412` reads `source.get("filepath")` (expects a **dict**).
- **Fix:** write the dict, or resolve the filename against `server/missions/` on read.
- **Verify:**
  1. Stage a georef mission → open the staged JSON → `metadata.source` resolves to a real path.
  2. Drive it → `manifest.json` has non-empty `staged_mission.source_file`.
  3. `python3 tools/analyze_mission.py <bundle>` → **§8 prints a verdict, not "unavailable"**.
- **Closes when:** §8 reports per-vertex absolute misses on a real bundle without hand-work.

### ☐ A2 — `must_hit` reaches non-staged missions
- **Cause:** only `routes/path.py:1279` passes `must_hit=`. Three call sites do not:
  `mission_loading.py:100`, `routes/mission.py:62`, `sockets/events.py:119`.
- **Verify:** load a georef DXF through the **non-staged** route, then
  `ros2 topic echo /path --once` → some points carry `position.z` with **bit 1 set** (z = 2 or 3).
- **Closes when:** a non-staged load publishes a non-zero must-hit count.
- ⚠ Add a test that asserts it, or this silently regresses.

### ☐ A4 — `outcome` reflects actual traversal
- **Cause:** `bag_autorecord.py:550-560` marks INCOMPLETE only on a missing `recorder_end`;
  path coverage is never checked.
- **Evidence it is wrong:** runs covering **24/64** and **76/86** waypoints both reported COMPLETE.
- **Verify:** add coverage (fraction of `/path` points the pose came within 25 cm of) to the
  manifest; deliberately abort a run at ~50% → `outcome` is **not** COMPLETE.
- **Closes when:** a partial run is machine-detectable without opening the bag.

---

## Priority 1 — The largest real error

### ☐ B1 — Dual-antenna heading offset is MEASURED, not assumed
- **Current:** `EKF2_GPS_YAW_OFF = 180.0`, `GPS_YAW_OFFSET = 180.0`. 180 is a round number —
  correct in principle (antennas mounted reversed) but almost certainly not measured.
- **Why it matters:** `lateral offset ≈ lookahead × sin(heading error)`. At the 0.52 m lookahead,
  **2 cm needs only ~2.2°**. Measured: 5 of 6 runs drove left, B spread 3.74 cm.
- **Test (stationary, no driving):**
  1. Park the rover **on the surveyed line, aligned along it**, stationary, RTK FIXED.
  2. True bearing from the surveyed lat/lon:
     `bearing = atan2(ΔE, ΔN)` using v0 → v3 of `tes_cross_line`.
  3. Read the reported heading (`/mavros/imu/data` yaw, or `vehicle_attitude`).
  4. `residual = reported − true`.
- **Closes when:** `|residual| < 0.5°`, after correcting `EKF2_GPS_YAW_OFF` by the residual.
- **Re-verify after the change:** repeat the 3 EXT runs; **B (driven vs own plan) mean should
  collapse toward zero and stop being one-sided.**

### ☐ B2 — Wheel scale symmetry
- **Current:** one `RBCLW_COUNTS_REV = 148000` for both wheels; `RD_WHEEL_TRACK = 0.470`.
  `EKF2_WENC_NOISE` / `_LAT_N` were tightened 0.35 → **0.1**, so the filter now trusts the
  encoder ~3.5× more — a wheel mismatch is fused in with confidence rather than rejected.
- **The test that separates B1 from B2 cheaply:** drive the same line **forward, then backward**.
  - A **heading** offset **flips sign** with direction.
  - A **wheel-scale** asymmetry **does not**.
- **Also verify:** measure both wheel diameters; confirm `RD_WHEEL_TRACK` against the physical
  centre-to-centre distance.
- **Closes when:** forward/backward bias signs are explained, and any residual is < 0.5 cm.

### ☐ B3 — Antenna lever arm is real
- **Current:** `EKF2_GPS_POS_X/Y/Z = 0 / 0 / −0.4`. **Y = 0 asserts the antenna is laterally centred.**
- **Verify:** physically measure primary-antenna offset from the vehicle centreline and from the
  reference point; enter the true values.
- **Closes when:** measured values are in QGC, or Y = 0 is confirmed correct by measurement.

---

## Priority 2 — Close the paint question

### ☐ B4 — Nozzle offset measured
- **Current:** `nozzle_forward_offset_m` / `nozzle_lateral_offset_m` declared, **never measured**.
- **Consequence:** every number produced so far describes where the **antenna** went.
- **Verify:** measure nozzle position relative to the GNSS antenna (forward + lateral); enter it;
  then re-run the spray boundary test.
- ⚠ `SPRAY_NOZZLE_OFFSET_PLAN.md` §3.6 — the spray-node xtrack gate fix is **mandatory before any
  non-zero offset**, or an offset ≥ 10 cm blocks all spraying.
- **Closes when:** a painted line's measured position matches the plan within the tracking budget.

### ☐ B5 — Physical re-survey
- **Verify:** after a marking run, survey the painted marks with the RS3 **using averaging**
  (the existing survey CSV shows `Samples=1`, 1.7 cm single-epoch RMS — average 10–30 s/point).
  Compare against the design in the same CRS.
- **Closes when:** paint-vs-design is quantified. **This is the only way to close error budget 4;
  no log can substitute.**

### ☐ Spray start-delay vs nozzle-offset A/B
- **Symptom:** spray starts late, stops on time. That asymmetry **rules out nozzle offset**
  (which shifts both ends equally) and points at `solenoid_open_delay_s = 0.10`.
- **Verify:** drive the square at **0.35 m/s**, then **0.15 m/s**. Delay error **scales with speed**;
  a nozzle offset does not.
- **Closes when:** the two causes are separated and `solenoid_open_delay_s` is set from the data.

---

## Priority 3 — Confirm what shipped

### ☐ C1 — Placement determinism (`dd166b3`) proven on hardware
- **Claim:** the same staged mission now publishes a **bit-identical** path every load.
- **Verify (≈1 minute, no driving):**
  1. Confirm the origin is present: `EKF local-frame origin received` in the rover-server log.
  2. Load the same staged mission **twice**; capture `/path` each time.
  3. Diff point-by-point.
- **Closes when:** **max point-to-point difference = 0.000 cm** across all waypoints.
- **Reference:** before the fix, the same file published paths 0.39–1.53 cm apart.

### ☐ C4 — §8 auto-runs → blocked by A1.
### ☐ C2 — Survey CSV ingest driven in the field (all missions to date were DXF).
### ☐ C3 — POINT-layer control points on a **dense real** drawing (only a synthetic 41-vertex case so far).
### ☐ C5 — segment→smooth ~5 cm seam
- May already be fixed as a side effect of `64c12ff`.
- **Verify:** plot `/rpp/debug[0]` (xtrack) against `/rpp/debug[40]` (profile code). If no step
  coincides with the 1↔2 flip, **close it**.
### ☐ C6 — Spray pivot-state gate (`6523a84`) field-verified (replay-verified 157→53 only).
### ☐ C7 — Endpoint under-run: rover rests **0.9–2.0 cm short** every run; decide whether the spec needs it closed.
### ☐ C8 — Per-run wander: residual **RMS 1.1–1.8 cm** after removing the mean offset. Control tuning, not offset.

---

## Priority 4 — Unblock spray modes

### ☐ A6 — Spray plan **Rev 4** written
`6523a84` invalidated §5, §7.2 and §7.3 (min-speed gate removed; the node now *does* subscribe to
RPP state). **Must land before any Phase C code** — implementing against a stale contract is how
`16480d9` became a false fix.

### ☐ A5 — `/spray/session_config` subscription exists (Phase B0)
Node comment at `spray_controller_node.py:444` confirms there is none, so **no mode can reach the
node**. Blocks dash and point entirely.
- **Verify:** publish each of the 3 configs → `/spray/status.mode` reflects it; continuous
  behaviour byte-identical to today; mission clear publishes a **cleared** config (TRANSIENT_LOCAL
  means a restart would otherwise resurrect stale geometry).

---

## Priority 5 — Known, scoped, not urgent

### ☐ A3 — Consume `xy_reset_counter` / `delta_xy`
Zero references in `src/` or `server/`. The origin is fixed, but the vehicle estimate can jump
against it mid-mission and nothing notices. **Verify:** an induced EKF reset appears in telemetry
and in the mission report.

### ☐ A7 — Cross-session origin repeatability
Origin re-derived at each EKF restart from a fresh first fix → one degE7 step (≤1.11 cm) per
session. Needs `SET_GPS_GLOBAL_ORIGIN` pinning at boot. **Verify:** reboot, reload, compare — paths
identical across the reboot.

### ☐ A8 — CPU contention (executor mismatch)
`MultiThreadedExecutor` + `MutuallyExclusiveCallbackGroup` = zero parallelism, 50–65% of one core.
Arm/disarm safety blocks a naive swap.

---

## Standing rules for any verification run

1. **Record coverage.** A partial run's per-vertex misses are meaningless at the uncovered end.
2. **Measure pose vs raw `/path`** — never `/rpp/debug[0]` (error vs the *conditioned* path,
   structurally hides vertex deletion) and never `/spray/debug[5]` (nozzle xtrack).
3. **Decompose every offset** into `A` (plan vs surveyed truth) and `B` (driven vs own plan)
   before blaming a subsystem. Their spreads say which one is at fault.
4. **Repeat 3×.** A systematic error repeats in sign and magnitude; noise does not.
5. **Run extensions.** Entry error lands on the dry run-up instead of the painted line
   (3.66 cm → 0.42 cm at v0, measured).
6. **Re-stage after any projection change.** `e483d53` shifted projected north 0.62%.

## Known-good yardstick (2026-07-22, `tes_cross_line`, 5 runs)

```
plan vs surveyed truth    0.26 – 1.13 cm   (identical on all 4 vertices = pure translation)
run-to-run plan spread    ≤ 1.53 cm
driven vs plan (bias)     −2.00 … +1.74 cm     ← dominant term
vertex hits, extensions   0.33 – 0.88 cm
vertex hits, bare         0.67 – 3.66 cm       ← v0 worst, no run-up
endpoint rest             0.9 – 2.0 cm short; overshoot never > +0.4 cm
```

A regression is any run materially outside these.
