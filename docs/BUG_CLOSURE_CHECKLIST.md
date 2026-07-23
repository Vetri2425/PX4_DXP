# Bug Closure Checklist

**Compiled:** 2026-07-22 · **Updated:** 2026-07-23 (source-verified)
**Tree:** `Upgrade_Spray` · 701 tests pass
**Companion doc:** `docs/OPEN_BUGS.md` → `.pdf` (the register) · **Method:** skill `analyse-missions`

> **Source verification 2026-07-23:** three parallel agents read every claim against the tree.
> All claimed fixes (A1, A2, A4, survey-tolerance, C1–C6) confirmed landed; all open bugs
> (A3, A5, A6, A7, A8, A9, B1–B4) confirmed present. Corrections: **A9** lives in
> `shape_grouping.py::_merge_chain`, not `engine.py`; **B4** offsets are applied (default 0.0),
> not dead; **C3** POINT matching is layer-agnostic. A worried "§8 key-name bug" was chased down
> and **dismissed** — the staging writer (`routes/path.py:1210`) and reader
> (`bag_autorecord.py:443`) agree on the `metadata` key; §8 is genuinely unblocked.

> **Status this cycle:** A1, A2, A4 and the survey-tolerance item are **CODE-COMPLETE,
> pushed and deployed** (`e3939e3`, `21a8b05`, `5f820be`, `7d77565`). Each was verified
> in-env on the Jetson against real bags — but **none has been through a field run**.
> Verified-on-hardware is not the same as field-proven; the ☑ boxes below say which.

Each item states what must be **true**, how to **prove** it, and what result **closes** it.
An item is only closed by evidence, never by "the code looks right" — this session produced
three bugs that shipped because a test's ground truth mirrored the bug it was testing.

---

## Priority 0 — Do before the next run batch  ✅ ALL LANDED

These three were small, and all of them corrupted the data you collect. They are now
deployed, so the next run batch records clean provenance.

### ☑ A1 — `source_file` populates, §8 runs automatically — **FIXED `e3939e3`, DEPLOYED**
- **Cause:** `routes/path.py:1179` writes `"source": result["source"]` (a filename **string**);
  `bag_autorecord.py:412` reads `source.get("filepath")` (expects a **dict**).
- **Fix:** write the dict, or resolve the filename against `server/missions/` on read.
- **Verify:**
  1. Stage a georef mission → open the staged JSON → `metadata.source` resolves to a real path.
  2. Drive it → `manifest.json` has non-empty `staged_mission.source_file`.
  3. `python3 tools/analyze_mission.py <bundle>` → **§8 prints a verdict, not "unavailable"**.
- **Closes when:** §8 reports per-vertex absolute misses on a real bundle without hand-work.
- **DONE:** the fix was neither of the two options above. `PathEngine.plan_file()` already
  built the exact dict the recorder wanted (`path_engine/engine.py:366`) — the stager was
  reaching for the wrong field. `metadata.source` keeps its string shape for the frontend;
  the dict rides alongside as `metadata.source_detail`. The reader accepts either **and**
  resolves a legacy bare filename against `server/missions/`, so bundles staged by the old
  server still analyse.
- **Verified on the Jetson 2026-07-22:** staged `tes_cross_line.dxf` →
  `source_file = /home/flash/PX4_DXP/server/missions/tes_cross_line.dxf` (exists) → **§8 gate: RUNS**.
- ⚠ **Not retroactive for closed bundles.** The reader fix only helps where the staged JSON
  still exists; the 07-22 manifests already on disk keep their empty `source_file`.

### ☑ A2 — `must_hit` reaches non-staged missions — **FIXED `21a8b05`, DEPLOYED**
- **Cause:** only `routes/path.py:1279` passes `must_hit=`. Three call sites do not:
  `mission_loading.py:100`, `routes/mission.py:62`, `sockets/events.py:119`.
- **Verify:** load a georef DXF through the **non-staged** route, then
  `ros2 topic echo /path --once` → some points carry `position.z` with **bit 1 set** (z = 2 or 3).
- **Closes when:** a non-staged load publishes a non-zero must-hit count.
- **DONE:** provenance now rides in the preview beside `spray`, so `must_hit_for_path` reads
  the same cached `PathPreviewResponse` that `spray_flags_for_path` already reads — no extra
  planning pass. It returns `None`, not an all-False list, when provenance cannot be trusted:
  `None` means "unknown" and degrades to geometry-only simplification, whereas all-False is a
  positive claim that no point is a vertex and *looks* like real provenance.
- **The test you asked for exists** — and it is structural, scanning every
  `offboard_ctrl.load_path` call site, because the socket handler cannot be invoked in-process
  and a future load route would otherwise drop provenance again silently.
- **Verified on the Jetson 2026-07-22:** `/api/path/tes_cross_line.dxf/preview` → 64 points,
  **4 must-hit**, and the non-staged route produces `must_hit` identical to the staged route's
  point for point.
- ☐ **Field step still open:** confirm `/path` carries `z = 3` on a live non-staged load.

### ☑ A4 — `outcome` reflects actual traversal — **FIXED `5f820be`, DEPLOYED**
- **Cause:** `bag_autorecord.py:550-560` marks INCOMPLETE only on a missing `recorder_end`;
  path coverage is never checked.
- **Evidence it is wrong:** runs covering **24/64** and **76/86** waypoints both reported COMPLETE.
- **Verify:** add coverage (fraction of `/path` points the pose came within 25 cm of) to the
  manifest; deliberately abort a run at ~50% → `outcome` is **not** COMPLETE.
- **Closes when:** a partial run is machine-detectable without opening the bag.
- **DONE, as a separate `traversal` block** rather than by overloading `outcome.status`:
  bag integrity and mission success are different facts and stay separately queryable.
  `outcome` now carries a `means` field saying what it actually asserts. Coverage is
  distance-to-path, deliberately **not** cross-track — a rover stopped dead on the line has
  zero xtrack for the whole mission it never drove.
- **The field data corrected the design.** Both partial runs covered the **END** of the path,
  not the beginning, so the abort-vs-skip split written first called them "not stopped early"
  and said nothing more. Classification is now a `shape`: `FULL` / `STOPPED_EARLY` /
  `STARTED_LATE` / `MIDDLE_ONLY` / `INTERIOR_GAP` / `NONE`.
- **Ruled out** a late-starting recorder as the cause: `POLL_S=0.2` bounds that at ~5 cm of
  driving, while the unreached leads are **1.95 m** and **0.48 m**.
- **Verified on the Jetson 2026-07-22**, reproducing both known-bad runs unprompted:

  | bundle | outcome | traversal | covered | shape |
  |---|---|---|---|---|
  | 162537 | COMPLETE | COMPLETE | 64/64 100% | FULL |
  | 162652 | COMPLETE | **PARTIAL** | 24/64 38% | STARTED_LATE |
  | 162107 | COMPLETE | COMPLETE | 86/86 100% | FULL |
  | 162241 | COMPLETE | **MOSTLY** | 76/86 88% | STARTED_LATE |
  | 162351 | COMPLETE | COMPLETE | 86/86 100% | FULL |
  | 162800 | COMPLETE | COMPLETE | 63/63 100% | FULL |

  All six report `mission_end_reason = "mission completed"` — **the reason field lies too.**
- ☐ **One path not yet exercised:** the recorder writes `traversal.status = "PENDING"` at
  mission stop. That code only runs on a real mission; first drive confirms it.

### ☑ survey_tolerance_m is operator-set — **FIXED `7d77565`, DEPLOYED**
- **Was:** the constant `SURVEY_TOL_CM = 2.5` in `analyze_mission.py` decided whether §7 FAILs.
- **Now:** `PathPlanRequest.survey_tolerance_m` (0 < x ≤ 1 m) → staged → manifest → analyser.
  Precedence `--survey-tol-cm` > staged > default. **Every report states the tolerance AND
  where it came from**, including when nothing was dropped — a PASS at an unattributed
  threshold is not evidence either.
- **Rejected:** auto-deriving it from the survey CSV's `Lateral RMS`. Any multiplier that keeps
  the known 3.43 cm vertex classified as INTENT has to be reverse-engineered from the answer
  we already believe — the same trap as a test whose ground truth mirrors the bug.
- **No controller change needed:** RPP already exposes `segment_simplify_max_offset_m`, and its
  authoritative retention test is `must_hit` provenance, not a tolerance.
- ☐ **Not yet wired in the UI** — the mobile frontend does not send the field, so real missions
  still report `[built-in default — not set for this survey]`.

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
- **Current:** `nozzle_forward_offset_m` / `nozzle_lateral_offset_m` declared **and applied**
  (`spray_controller_node.py:178-196` + `:798-804`, real body-frame transform), but both
  **default to 0.0** — the transform runs, it just shifts by zero. [src-verified 07-23]
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

### ☑ C1 — Placement determinism (`dd166b3`) proven on hardware — **CLOSED 2026-07-22**
- **Claim:** the same staged mission now publishes a **bit-identical** path every load.
- **Verify (≈1 minute, no driving):**
  1. Confirm the origin is present: `EKF local-frame origin received` in the rover-server log.
  2. Load the same staged mission **twice**; capture `/path` each time.
  3. Diff point-by-point.
- **Closes when:** **max point-to-point difference = 0.000 cm** across all waypoints.
- **Reference:** before the fix, the same file published paths 0.39–1.53 cm apart.
- **RESULT (Jetson, 2026-07-22, no driving):** staged `tes_cross_line.dxf`, loaded it twice
  via `POST /api/path/load-to-controller`, diffed `/api/mission/loaded-path`:

  ```
  load 1 : 64 waypoints, 40 sampled     max point-to-point difference : 0.0000 cm
  load 2 : 64 waypoints, 40 sampled     mean                          : 0.0000 cm
  ```

  **Bit-identical. C1 closed.**
- ⚠ **Scope:** this proves *within-session* determinism only, and compares the 40 sampled
  waypoints the summary returns (head 20 + tail 20 of 64) — enough, because a placement
  error shifts every point uniformly. **Cross-session repeatability is A7 and remains open:**
  the origin is re-derived at each EKF restart from a fresh first fix, so one degE7
  quantisation step (≤1.11 cm) of offset per session is still possible.
- Controller state was cleared afterwards; rover left idle/disarmed as found.

### ☐ C4 — §8 auto-runs → **A1 unblocked**; gate verified RUNS on the Jetson, but has not yet auto-run on a freshly recorded bundle. Needs one drive.
### ☐ C2 — Survey CSV ingest driven in the field (all missions to date were DXF).
### ☐ C3 — POINT control points on a **dense real** drawing. [src-verified 07-23] The parser is **layer-agnostic** (`dxf_parser.py:702`, snaps any POINT within 1 cm of a vertex — not keyed to a "Points" layer). The 41-vertex case is a commit-message demo, not a committed test; real coverage is a 4-vertex case in `test_core.py:91-112`.
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

### ☑ A9 — must_hit survives multi-entity joins — **FIXED 2026-07-23**  [src-verified 07-23]
- **Cause:** `path_engine/optimizers/shape_grouping.py::_merge_chain` (~`:206-213`) copies the merged
  composite's `vertex_indices` from the **`head`** chain member only; the other joined segments'
  vertices are discarded. A square from 4 LINE entities keeps only its first segment's 2 endpoints
  as must-hit → **2 of 4 corners**. `64c12ff` fixed vertex-drop in RPP conditioning but never touched
  this file, so multi-entity shapes are still under-marked upstream of it.
- **Verify:** plan a 4-LINE square → assert `sum(must_hit) == 4`. Currently pinned at the buggy value
  by `server/test_staged_endpoints.py:408-425` (comment: "a separate engine defect").
- **Closes when:** every source-geometry corner of a multi-entity shape carries `must_hit=True`, and
  the pinning test is flipped to assert the correct count.
- **DONE:** `_merge_chain` and `decompose_line_chain_to_edges` now remap `vertex_indices` (and
  `control_indices`) into the composite/edge index space, over-preserving when provenance is absent.
  A 4-LINE square and an unequal-sided rectangle both keep 4/4 corners. 3 regression tests added to
  `test_vertex_provenance.py`, proven to fail on pre-fix source (`corner (2,2) not flagged`).
  **406 path_engine + 168 server tests pass.** ☐ field-unverified — confirm on a real multi-entity survey.

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
