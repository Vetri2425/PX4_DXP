# Field Position & Plan Error Analysis — 2026-07-13 (evening bags)

**Branch:** `baseline_master` @ `ea47ec8` · **Bags:** `bags/13-07-2026/` (10 pulled; 7 completed missions analysed)
**Scope:** cross-track, start, drift, stop, placement/plan errors. Trustworthy numbers only — the analyser
coast/pivot FAILs are a metric bug (see `docs/BUG_analyze_mission_metrics.md`), not rover error.

> **One-line verdict:** tracking, stops, and entry convergence are **production-grade** (all sub-2 cm steady-state,
> 1.1–1.7 cm stops). The two real position/plan items to watch are **corner-transient overshoot** and
> **ref-point survey scale repeatability (~±2%)** — neither is a controller bug.

---

## 1. Cross-track error — steady-state marking (controller active-segment)

| Mission | Marking RMS | p95 | max | Bias (L/R) |
|---|---|---|---|---|
| Line 18:12 | 0.52 cm | 0.80 | 0.86 | −0.28 (63/22) |
| Line 18:13 | 0.94 | 1.36 | 4.47 | −0.00 (47/42) |
| sq2x2 18:14 | 1.16 | 2.70 | 3.58 | −0.25 (26/26) |
| sq2x2 18:16 | 0.74 | 1.05 | 4.51 | +0.07 (21/24) |
| stg 8c6290de (GPS) | 1.23 | 2.22 | 4.33 | −0.29 (36/15) |
| stg 37888b3e (GPS) | 1.02 | 2.47 | 3.05 | +0.04 (22/24) |
| stg 3ed56676 (GPS) | 0.98 | 2.18 | 4.74 | +0.11 (14/28) |

**Result:** every mission **sub-2 cm RMS**, p95 ≤ 2.7 cm. Bias is near-zero (±0.3 cm) and L/R-balanced →
**no systematic cross-track offset**, GPS-surveyed and LOCAL alike. Straight-line marking is the tightest
(0.5–0.9 cm). ✅

## 2. Cross-track peak — corner transient (real, expected)

Independent perpendicular-to-full-path measurement peaks at **~30 cm momentarily at the pivot apex** on every
shape, decaying immediately back to sub-2 cm on the next straight. This is the rover swinging wide through the
turn, i.e. the **corner-overshoot / arc-flow behaviour** — it is the *only* place position error leaves the
sub-2 cm band, and it ties directly to the open **P1 (arc doesn't flow)** work. Not a defect; the target of P1.

## 3. Start error & two-phase entry convergence

| Mission | Start metric | Value |
|---|---|---|
| stg 8c6290de | converge to first marked wp | **0.5 cm** |
| stg 37888b3e | " | **1.5 cm** |
| stg 3ed56676 | " | **1.7 cm** |
| Line 18:12 / 18:13 | closest approach to planned start | 6.2 / 4.2 cm |

**Two-phase GPS entry converges to the first marked point within ~1.5 cm** from arbitrary start positions
(placement offsets up to +3.09 m N — see §6). This is the entry feature working as designed. ✅

**Measurement caveat (not a rover error):** `sq2x2 18:16` shows a 49.6 cm "start error" — this is an
**open-square plan** where `path[0]=(0.42,−0.11)` is *not* where the rover began (it started at the
`(0.41,1.98)` corner and RPP picked up the nearest waypoint). Same family as the analyser bug: measuring
against `path[0]` is wrong when the rover legitimately starts elsewhere on the path.

## 4. Drift — none

Cross-track **does not grow over the run**: steady-state RMS in the last third of marking is ≤ 1.3 cm on every
mission (Line 0.7, squares 0.6–0.7, staged 0.5–1.2). Apparent first-third inflation in the raw sweep is entry/
first-corner transient, not drift. Bias stable at ±0.3 cm start-to-finish. No EKF jumps, 0 RTK-degraded,
0 pose-stale across all bags. ✅

## 5. Stop error (the trustworthy metric)

| Mission | Endpoint resting | Corner closest (worst) |
|---|---|---|
| Line 18:12 / 18:13 | 1.1 / 1.6 cm | — |
| sq2x2 18:14 / 18:16 | 1.6 / 4.8 cm | 0.1–2.5 cm |
| stg 8c6290de | 1.6 cm | 0.1–1.4 cm |
| stg 37888b3e | 1.7 cm | 0.1–2.5 cm |
| stg 3ed56676 | 1.7 cm | 1.2–1.7 cm |

**Final stops rest 1.1–1.7 cm from the planned endpoint** (one 4.8 cm outlier on sq2x2 18:16); corner closest-
approach 0.1–2.5 cm. Completion latch (`_hold_at_completion`) holding — coast at the *true* final approach is
≤ 1 cm. ✅ (The "282/335 cm coast" verdicts are the analyser bug, not real overshoot.)

## 6. Placement & plan fidelity

Survey placement offset (live-EKF correction `P_live = P + L − R_anchor`) and ref-point affine per staging:

| Mission | Placement offset | rot | **scale** |
|---|---|---|---|
| stg 8c6290de | +0.386 N −0.285 E | −0.09° | 0.9970 |
| stg 37888b3e | +3.068 N +0.187 E | 4.03° | 1.0008 |
| stg 3ed56676 | +3.086 N +0.187 E | 4.03° | **0.9780** |

**Real plan-side item to watch — ref-point scale repeatability.** Across today's stagings the fitted affine
**scale ranged 0.966–1.0025** (§ other stagings), i.e. **~±2–3 %**. On stg_3ed56676 the plan was scaled to
**0.978 (−2.2 %)** — a 2 m edge is placed as ~1.956 m (≈ 4.4 cm geometric error over 2 m). This is GPS noise in
the surveyed ref points propagating through the affine fit; it is a **plan-geometry error independent of
tracking** (the rover tracks the *placed* path to sub-2 cm — but the placed path itself can be ~2 % off true
scale). Consider: more ref points to average out fit noise, or clamping scale toward 1.0 when the physical rig
is known-rigid.

---

## What needs resolving (ranked)
1. **[BUG] analyser coast/pivot metrics** — `docs/BUG_analyze_mission_metrics.md` (P2). Blocks trusting any
   further shape analysis. Mac-side, low-risk.
2. **[P1] corner-transient overshoot / arc flow** — the only place position error exceeds sub-2 cm (§2).
   Diagnosis-first, no knob retune (invariants I2/I4).
3. **[watch] ref-point survey scale ±2–3 %** (§6) — plan-geometry accuracy, not tracking. Quantify over more
   stagings before acting.

## What is NOT broken
Steady-state cross-track (sub-2 cm), zero systematic bias/drift, two-phase entry convergence (~1.5 cm), final
stops (1.1–1.7 cm), health (0 OFFBOARD drops / EKF jumps / RTK-degraded). The controller is behaving to spec.
