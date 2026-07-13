# End-to-End Field-Test Audit — Staged GPS-Surveyed Mission

**Date:** 2026-07-13 14:31 IST
**Branch:** `test/colinear-fix` @ `c7164c5`
**Method:** read-only audit, 3 parallel auditors (server / controller / frontend). No files changed.

## Verdict

- 🟡 **CONDITIONAL GO** for a *supervised* DXF-surveyed field test
- 🔴 **NO-GO** for an autonomous arbitrary-start production run

The pipeline is sound and contract-correct end-to-end **except the runtime entry** (getting from an arbitrary start to the first waypoint), which is emergent rather than engineered. Everything from the first waypoint onward — the D0/D3-proven core — is GO.

## Per-stage results

| # | Stage | Verdict | Basis |
|---|---|---|---|
| 1 | Frontend alignment (DXF ref-points → anchor) | 🟢 GO | Operator supplies surveyed lat/lon per ref point (not device GPS) — the survey anchor the server wants; server derives `origin_gps` from the fit |
| 2 | Staging contract | 🟢 GO | saveOrder→spray→verify→plan-and-stage→load-to-controller; server derives `GPS_SURVEYED` from `origin_gps` presence; fields match |
| 3 | GPS anchor → local frame (`P_live=P+L−R_anchor`) | 🟢 GO | Strongest stage — Karney geodesic; fully fail-closed on RTK/pose/global/skew; typed 422; unit-test-locked |
| 4 | Load + start | 🟢 GO | Placement resolved *before* arming (422 not masked by STALE); `GPS_SURVEYED ⊕ auto_origin` enforced; verified-start sends `auto_origin:false` |
| 5 | Verify loaded-path | 🟢 GO | App checks mission_id + is_staged + protected + placement_mode + waypoints; all present server-side |
| 6 | Tracking (segment/smooth dispatch) | 🟢 GO | Line→segment, arc→smooth; per-run, coherent |
| 7 | Corner stop + pivot + D3 latch | 🟢 GO | Body-axis brake, measured-stop dwell, ±75° cone, angle-aware timeout, completion latch — D0/D3-validated |
| 8 | Extensions PRE/MARK/AFT + collinear | 🟢 GO | Momentum kept across collinear spray boundaries, stops at real corners |
| 9 | **Runtime entry (arbitrary start → wp0)** | 🔴 **NO-GO (prod)** / supervised-only | No entry state machine; run 0 gets no pre-align; twist bridge velocity-only; entry emergent (forward-cone clamp + PX4 spot-turn) |

## Blockers (why full-autonomous is NO-GO)

1. **Runtime entry is unbuilt** (D1/D2/D4 of `RUNTIME_ENTRY_VELOCITY_PLAN.md`). Run 0 begins normal tracking immediately at accel-ramped speed; forward-cone clamp prevents reverse-flip/deadlock, but the entry transient (cross-track, overshoot, launch speed after the spot-turn) is uncontrolled and geometry-dependent.
2. **Spray not force-OFF during entry** by the controller. If run 0 is a bare MARK span, the sprayer fires while the rover is still arcing onto the line. Spray-OFF-during-entry currently depends entirely on the planner supplying a spray-off PRE/transit lead-in.

## Conditions for a SUPERVISED GO (all required)

1. **DXF ref-point alignment only.** 🔴 Hard NO-GO on the CSV "GPS point mission" path — `/api/path/parse-point-gps-csv` and its staging fields don't exist on this server; it silently misbehaves.
2. **Start via the verified-staged path** (`auto_origin:false`). Do **not** use the app's "Force Start" override for a surveyed mission — it can send `auto_origin:true`/`path_name` and revert the mission to raw LOCAL_NED.
3. **Neutralize the entry gap:** start the rover near wp0 with modest heading error, *or* have the planner emit a spray-off transit/PRE lead-in as run 0 so the first MARK begins only after the rover is on-line.
4. **Operator on the kill switch** through the initial spot-turn and first corner.
5. Confirm **RTK FIXED** at start (placement fails closed otherwise — safe, just won't start).

## Bottom line

The mission engine is field-ready; the on-ramp isn't. Tracking, stops, pivots, extensions, GPS placement, and the frontend/staging contract are GO and largely test-locked. The single thing between "supervised GO" and "autonomous GO" is the **runtime entry** — the `RUNTIME_ENTRY_VELOCITY_PLAN.md` work (D0/D3 done; D1/D2/D4 pending).

## Auditor detail (for the record)

- **Server (GO, conditional):** geodesic conversion + fail-closed gates are the strongest stage; the only operational pin is empty-body start for staged missions — *mitigated* by the frontend, which sends `{mission_id, auto_origin:false}` (no `path_name`) on the verified path. Residual risk only via "Force Start".
- **Controller (GO tracking / NO-GO entry):** areas 1–3 + 8 coherent and baseline-matched; area 9 (entry) has no state machine — run 0 gets no `_run_align_pending`, `_run_alignment_hold` never fires for the first run, twist bridge velocity-only. Entry is emergent forward-cone-clamp + PX4 spot-turn: safe (no reverse-flip/deadlock) but uncontrolled and not spray-safe if run 0 is MARK.
- **Frontend (GO for DXF surveyed):** alignment→stage→verify→start complete and contract-correct; `auto_origin:false` forced. NO-GO on CSV point missions (endpoints absent on baseline).

## Follow-up work items

- **D1** two-phase entry publish (server) · **D2** run-0 pre-align (controller) · **D4** `ENTRY` MissionState + controller-enforced spray-OFF during entry. These lift stage 9 from NO-GO to GO and remove supervised-condition 3.
