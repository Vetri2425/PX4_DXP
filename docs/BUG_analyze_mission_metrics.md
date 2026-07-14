# BUG — `analyze_mission.py` coast-past & pivot-window metrics false-fail good missions

**Status:** OPEN · **Severity:** Medium (analyzer-only — does NOT affect the rover) · **Filed:** 2026-07-13
**Component:** `tools/analyze_mission.py` (Mac/Jetson post-mission analyser) · **Branch:** `baseline_master` @ `ea47ec8`
**Handoff ref:** this is item **P2** in `docs/SESSION_CONTINUATION_2026_07_13.md`.

## Summary
The post-mission analyser reports **FAIL** on missions the rover executed correctly. Two metrics are
mis-measured: (1) **coast-past** is not bounded to the final approach, and (2) the **pivot window** is not
gated to a single corner. The rover's real behaviour on the affected bags is production-grade (sub-2 cm
tracking, 1.4–1.7 cm final stops). **No rover/controller defect — this is an analyser measurement bug.**

## Evidence (bags in `bags/13-07-2026/`)

| Bag | Analyser verdict | Reality (trustworthy fields) |
|---|---|---|
| `square_2x2.dxf_..._181444` | FAIL — coast-past **282.3 cm** | endpoint `resting 1.6 cm`, tracking RMS 0.84 cm |
| `stg_37888b3e_..._183355` | FAIL — coast-past **282.2 cm** | `resting 1.7 cm`, RMS 1.04 cm |
| `stg_8c6290de_..._182423` | FAIL — coast **335.6 cm** + `reverse-flip True` | `resting 1.6 cm`; the "flip" pivot0 reports `turn 585.1°` |

## Root cause

### 1. Coast-past is unbounded (measures the wrong departure)
`analyze_stops` measures how far the rover travelled past the endpoint, but does not restrict the window to
the **final** approach. Two ways it mis-fires:
- **Two-phase staged entry:** the rover *starts* on the entry leg ~2.8 m from the endpoint. That initial
  distance is counted as "coast." → prints `coast 282 cm` while `resting` is 1.7 cm.
- **Closed/near-closed shapes (square):** `path[0] ≈ path[-1]`, so the rover is ~2.8 m from the endpoint at
  mission start and mid-run; the unbounded window catches that pass-by.

Constants involved: `COAST_MAX_CM=15.0`, `DEPART_M=0.30` (analyze_mission.py:42,45). `DEPART_M` is too small
and the coast window is not re-based to the last real departure.

### 2. Pivot window merges multiple corners
`_pivot_windows` / `analyze_pivots` accumulate turning across the entry + several corners into one window.
`stg_8c6290de` `pivot0: turn 585.1° … min-fwd -0.08 flip=True` — a 585° single pivot is physically
impossible; it is the entry + first corners merged. The slightly-negative `min-fwd` inside that bogus window
trips the reverse-flip gate (`FWD_EPS=-0.02`, analyze_mission.py:48). The mission's **other 9 pivots are all
clean** (`min-fwd 0.021`, `flip=False`, settle <1.5°).

## Impact
- Good production missions are labelled FAIL → real regressions will be masked going forward.
- Specifically corrupts the **arc-flow (P1)** investigation, which relies on this analyser.

## Fix (both changes are Mac-side, pure-stdlib, no rover, no field test)
1. **Coast-past:** measure coast only on the FINAL approach — after the rover last departed the endpoint by
   `> 0.5 m`. Re-base the window to that last departure, not the whole trajectory.
2. **Pivot window:** gate each pivot to one real `CORNER_ALIGN` (segment-state) transition, not accumulated
   heading change. Reject/segment any window whose net turn exceeds ~200°.

## Invariant (trust this number until fixed)
**Final-stop = direct `pose → endpoint` distance** (the report `resting` field) is correct: 1.4–1.7 cm on all
affected bags. Distrust `coast-past` and `reverse-flip` on two-phase-entry and closed/curved missions until
the two fixes above land. One change → re-run the 3 evidence bags → confirm they flip to PASS.
