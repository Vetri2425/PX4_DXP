# PX4_DXP — Open Bug Register

3WD marking rover · branch `Upgrade_Spray` @ **`7d77565`** (Jetson matches) · **701 tests pass**
Compiled 2026-07-22 from six field runs of `tes_cross_line`; **updated 2026-07-22 post-deploy**.

> This markdown is the **source**. `OPEN_BUGS.pdf` is generated from it —
> edit here, then regenerate. The PDF was previously hand-maintained with no source,
> which is why it drifted.

**Severity** is impact on the marking result, not effort.
**Verified in-env** ≠ **field-proven**. The register says which.

---

## A — CODE LEVEL

| # | Bug | Location / parameter | Evidence | Sev | Status |
|---|---|---|---|---|---|
| **A1** | `source_file` never populates → §8 ABSOLUTE ACCURACY can never run. Two different things named `source`: the stager writes a filename **string**, the reader expects a **dict**. | `routes/path.py:1179` wrote `"source": result["source"]`; `bag_autorecord.py:412` read `source.get("filepath")` | §8 reported `unavailable` on all 6 bundles; the entire absolute analysis had to be done by hand. | HIGH | ✅ **FIXED `e3939e3`** · deployed · Jetson-verified: §8 gate **RUNS** |
| **A2** | `must_hit` lost on every non-staged load. Only the staged path passed it, so vertex provenance never reached those missions. | `mission_loading.py:100`, `routes/mission.py:62`, `sockets/events.py:119` (vs `routes/path.py:1279`) | `mission_20260722_161014` published `must_hit = 0` on a 48-pt path. | HIGH | ✅ **FIXED `21a8b05`** · deployed · Jetson: preview 64 pts / **4 must-hit**; matches staged route point-for-point. ☐ live `/path z=3` unconfirmed |
| **A3** | EKF position resets ignored. Origin is fixed, but the vehicle estimate can jump against it mid-mission and nothing notices. | `xy_reset_counter`, `delta_xy` — zero references in `src/` or `server/` | Source-verified in `ekf_helper.cpp`: GNSS resets project through the origin and increment the counter. `dd166b3` does not cover this. | HIGH | ☐ **OPEN** |
| **A4** | `outcome == COMPLETE` on partial runs. Only a missing `recorder_end` marked INCOMPLETE; path coverage was never checked. | `bag_autorecord.py:550-560` | Two runs covered 24/64 and 76/86 waypoints — both reported COMPLETE. | HIGH | ✅ **FIXED `5f820be`** · deployed · reproduced both bad runs unprompted. ☐ `PENDING` marker needs one real mission |
| **A5** | No `/spray/session_config` subscription. No mode can reach the spray node; dash and point remain schema-only. | `spray_controller_node.py:444` (the node's own comment) | Blocks Spray V2 Phase C and D entirely. | HIGH | ☐ **OPEN — top code blocker** |
| **A6** | Spray plan Rev 3 stale vs shipped code. §5/§7.2/§7.3 describe a speed gate `6523a84` removed, and deny an RPP coupling that now exists. | `SPRAY_CONTROLLER_V2_PLAN.md` vs `spray_controller_node.py:877` | Implementing Phase C against it repeats the `16480d9` false-fix pattern. | HIGH | ☐ **OPEN — do before A5** |
| **A7** | Cross-session origin repeatability. Origin re-derived at each EKF restart from a fresh first fix; needs `SET_GPS_GLOBAL_ORIGIN` pinning at boot. | not implemented anywhere | One degE7 quantisation step (≤1.11 cm) of offset **per session**. Within a session, identical. | MED | ☐ **OPEN** — see C1: within-session is now proven, cross-session is not |
| **A8** | CPU contention — executor mismatch. `MultiThreadedExecutor` with `MutuallyExclusiveCallbackGroup` gives zero parallelism. | `ros_node.py` executor setup | 50–65% of one core. Arm/disarm safety blocks a naive swap. | MED | ☐ **OPEN** |
| **A9** | `must_hit` under-marked at entity junctions. A square built from 4 LINE entities flags only **2 of its 4 corners**; independent of corner smoothing. | `path_engine/engine.py:1039-1124` (merge step / junction dedup) | Found while verifying A2. Reproduces identically on the staged route, so it predates `21a8b05`. If provenance is under-marked, RPP can still simplify real surveyed corners away — `64c12ff` would be quietly incomplete. | HIGH | ☐ **NEW, OPEN** |

### Survey tolerance — closed this cycle
`SURVEY_TOL_CM = 2.5` was a constant in `analyze_mission.py` that decided whether §7 FAILs.
Now `PathPlanRequest.survey_tolerance_m` → staged → manifest → analyser, precedence
`--survey-tol-cm` > staged > default, with the source printed in every report.
✅ **`7d77565`**, deployed. ☐ Not yet sent by the mobile frontend.

---

## B — CALIBRATION / HARDWARE (QGC parameters)

**None of these are code. All need the rover and a tape measure.**

| # | Bug | Location / parameter | Evidence | Sev | Status |
|---|---|---|---|---|---|
| **B1** | **Dual-antenna heading bias — largest open error.** A constant heading error makes pure pursuit settle on a line *parallel* to the path. | `EKF2_GPS_YAW_OFF` = 180.0 (a round number — assumed, not measured), `GPS_YAW_OFFSET` = 180.0 | 5 of 6 runs drove **left**; B spread 3.74 cm, means −0.47 to −2.00 cm. `offset ≈ lookahead × sin(err)`: **2 cm needs only ~2.2°** at 0.52 m lookahead. | HIGH | ☐ **OPEN — do this first** |
| **B2** | Differential wheel asymmetry. One counts-per-rev value for two wheels that may not match; same one-sided signature as B1. | `RBCLW_COUNTS_REV` = 148000, `RD_WHEEL_TRACK` = 0.470 | Indistinguishable from B1 in the data. **The cheap separator:** drive the same line forward then backward — a heading offset flips sign, a wheel-scale error does not. | MED | ☐ **OPEN** |
| **B3** | GNSS antenna lever arm. Antenna off the vehicle centreline gives a fixed lateral offset; roll amplifies it. | `EKF2_GPS_POS_X/Y/Z` = 0 / 0 / −0.4 — **Y=0 asserts it is centred** | Reference rig: 1.934 m pole, 1.7° tilt = 5.7 cm lateral if uncompensated. | MED | ☐ **OPEN** |
| **B4** | Nozzle offset uncalibrated. Every measurement to date describes where the **antenna** went, not where paint went. | `nozzle_forward_offset_m`, `nozzle_lateral_offset_m` — declared, unmeasured | Blocks error budget 4. ⚠ `SPRAY_NOZZLE_OFFSET_PLAN.md` §3.6 xtrack-gate fix is **mandatory before any non-zero offset**. | HIGH | ☐ **OPEN** |
| **B5** | Physical re-survey never done. No log can answer where the paint landed. | process gap — Emlid RS3 | Survey CSV shows `Samples=1`, 1.7 cm single-epoch RMS. **Use averaging (10–30 s/point) when re-surveying.** | HIGH | ☐ **OPEN** |

---

## C — SHIPPED BUT UNVERIFIED (code-proven, field evidence pending)

| # | Item | Evidence | Sev | Status |
|---|---|---|---|---|
| **C1** | Placement determinism (`dd166b3`) | Two loads of one staged mission on the Jetson, 2026-07-22: **max point-to-point difference 0.0000 cm** across 40 sampled of 64 waypoints. Was 0.39–1.53 cm before the fix. | HIGH | ✅ **CLOSED on hardware** (within-session only — cross-session is A7) |
| **C2** | Survey CSV ingest (`211244b`) | unit tests only; never run in the field — all missions to date were DXF | MED | ☐ open |
| **C3** | POINT-layer control points | synthetic road case only; never exercised on a dense real drawing | MED | ☐ open |
| **C4** | §8 absolute accuracy auto-runs | was blocked by A1 — **unblocked**; gate verified RUNS on the Jetson, but has not yet auto-run on a freshly recorded bundle | HIGH | ☐ open (needs one drive) |
| **C5** | segment→smooth ~5 cm seam | visual only; may already be fixed by `64c12ff`. Check `/rpp/debug[0]` vs `[40]` at the profile flip. | MED | ☐ open |
| **C6** | Spray pivot-state gate (`6523a84`) | replay-verified 157→53; field-unverified | MED | ☐ open |
| **C7** | Endpoint under-run | rover rests **0.9–2.0 cm short** every run; overshoot never above +0.4 cm | MED | ☐ measured, no fix attempted |
| **C8** | Per-run wander | residual **RMS 1.1–1.8 cm** between runs after removing the mean offset | MED | ☐ measured, untouched |

---

## Recommended order

1. **B1 heading test** — largest error, **no driving needed**, one QGC parameter.
2. ~~A1, A2, A4~~ — ✅ done and deployed; the next run batch records clean provenance.
3. ~~C1 two-load determinism~~ — ✅ closed, 0.0000 cm.
4. **B4 + B5** — the only route to knowing where paint lands.
5. **A6 → A5** — unblocks spray modes (write Rev 4 *before* touching Phase C code).
6. **A9** — under-marked junction vertices; may mean `64c12ff` is incomplete.

---

## Field-proven this cycle

- **Vertex deletion** (`64c12ff`) — 64→4 conditioned points, was 64→2.
- **Georef north-scale** (`e483d53`) — 2.3255 m against a 2.3255 m WGS84 geodesic, residual 0.000 mm.

**Tracking is sound** at 0.33–0.88 cm per surveyed vertex with extensions.
**The remaining error is offset, not tracking** — which is why B1–B3 sit above every code item.

---

## Standing caution

Three bugs shipped in the 2026-07-22 cycle because a test's ground truth **mirrored the bug**:
bare-vertex input bypassing densification; haversine using the same wrong radius as the
projection; NavSatFix decoded by tests that bypassed the CDR reader. Be sceptical of any test
claiming to validate against "truth" — and check a new test **fails** before the fix lands.
