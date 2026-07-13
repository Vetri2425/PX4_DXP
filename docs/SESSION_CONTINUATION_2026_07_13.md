# Session Continuation — Runtime Entry + Recorder + Field Validation

**Date:** 2026-07-13 · **Branch:** `test/colinear-fix` · **HEAD:** `dc08ab5` (all pushed to origin, all deployed to Jetson)
**Read first:** `docs/RUNTIME_ENTRY_VELOCITY_PLAN.md`, `docs/E2E_FIELD_AUDIT.md`, `docs/BAG_RECORDER_PRODUCTION_PLAN.md`. Memory index at `~/.claude/.../memory/MEMORY.md`.
**Ethos:** verify in the code on THIS branch — do not trust stale notes/other branches. Invariants I1–I4 below are non-negotiable.

---

## 1. Status snapshot (what is DONE, deployed, field-proven)

| Item | Commit | Deployed | Field-validated |
|---|---|---|---|
| **D0** large-yaw pivot bench | (bench) | — | ✅ 169.6° dead-stop spot-turn, no reverse-flip |
| **D3** completion latch (`_hold_at_completion`) | 6e31904 | ✅ rpp-pipeline | ✅ coast 9.9 cm vs 108 cm |
| **Recorder** production-grade + `analyze_mission.py` | c7164c5 | ✅ bag-autorecord | ✅ auto-captures every mission → bundle + report |
| **D2** run-0 pre-align (gated `entry_prealign_enabled`) | 019e22d | ✅ rpp-pipeline | ✅ clean 89–176° entry pivots |
| **D1** two-phase entry + **D4** `ENTRY` MissionState | dc08ab5 | ✅ rover-server | ✅ line + square from far + degenerate starts |
| **E2E audit** | 019e22d | — | supervised-GO → now lifted for line/square |

**Deployed state on Jetson (192.168.1.102):** all services active at `dc08ab5`. `git pull` is clean.
**⚠ Runtime param that RESETS on rpp-pipeline restart:** `entry_prealign_enabled=true` must be re-set after any rpp-pipeline restart (it is declared `False`). Set it before any surveyed test:
`ros2 param set /rpp_controller entry_prealign_enabled true`

---

## 2. Field results (6 missions, 2026-07-13) — see `docs/` A4 report + bags

Bags: Jetson `~/bags_jet/`, mirrored to Mac `~/Vetri/rover_bags/missions_20260713/`.

| # | Shape | Placement | Start dist | Entry turn | Xtrack RMS/max | Heading-err | Final stop | Verdict |
|---|---|---|---|---|---|---|---|---|
| 1 | Line | Auto-origin | 0.15 m | ~0° | 0.61 / 1.03 cm | 0.66° | 1.5 cm | 🟢 |
| 2 | Line | GPS | 2.05 m | 176.5° | 1.04 / 2.66 cm | 0.64° | 1.4 cm | 🟢 |
| 3 | Line | GPS (skip) | 0.03 m | 2.4° | 0.61 / 1.39 cm | 0.53° | 1.3 cm | 🟢 |
| 4 | Square | GPS | 1.98 m | 88.8° | 1.00 / 2.45 cm | 0.63° | 1.8 cm | 🟢 |
| 5 | Square | GPS (skip) | 0.03 m | 124.4° | 1.06 / 3.03 cm | 0.64° | 1.7 cm | 🟢 |
| 6 | Arc | GPS | 3.28 m | ~64° | 1.28 / 4.54 cm | 0.64° | 1.6 cm | 🟡 flow |

**Line + Square = production-grade, field-validated.** Sub-2 cm xtrack, 0.6° heading, clean stops ≤1.8 cm, two-phase GPS entry proven. Speed steady (0.33±0.05), no speed/steering oscillation. Square corners: closest 0.9–1.2 cm, coast 0, pivots 88–124° clean.
**Arc = accuracy PASS, flow OPEN** (below).

---

## 3. Open work items (prioritized)

### P1 — Arc doesn't flow (the top technical item)
Arc #6 tracks accurately (1.28 cm RMS) but was executed **~68% stop-pivot** (segment-state share: ALIGN 44% + PRE_CORNER 24%, TRACK only 27%), with 27–29 cm overshoot at 3 curve corners. Profile code showed **both segment(1) and smooth(2)** were used. An arc should *flow*, not stop-pivot.
**Investigate:** how `sct_1.5m` gets classified (`_classify_auto_profile`) and run-split (`_split_run_at_corners`, `_smooth_corners`) — is the curve being treated as segment corners instead of the smooth profile, or is `corner_smooth_radius` not reaching it? Confirm from a bag whether the arc runs were profile 1 or 2 and where the ALIGN time came from. Do NOT retune corner knobs to mask it (I2/I4).

### P2 — `analyze_mission.py` metric false-fails (cosmetic, fix before more shapes)
The endpoint/corner **coast-past** metric mis-fires on (a) two-phase entry (rover STARTS near the endpoint → counts the entry-leg drive-away as coast; e.g. line/square "200–281 cm FAIL" while resting was 1.3–1.8 cm) and (b) smooth curves (flow-through counted as coast; `pivot0=537° flip=True` is bogus). **Fix:** measure coast only on the FINAL approach (after the rover last departed the endpoint by >0.5 m); gate pivot-window detection to real CORNER_ALIGN transitions, not accumulated arc turning. The **final-stop = direct pose→endpoint distance** is the reliable number.

### P3 — Circle mission (not yet run)
Same flow as arc (smooth, closed loop). Run after P1 so the arc flow fix carries over.

### P4 — `entry_prealign_enabled` persistence
Currently a runtime param defaulting False (resets on restart). After enough field A/B, either flip the declared default to True or set it via the rpp launch/config so surveyed missions get the entry pivot without a manual `ros2 param set`. Until then, **re-set it every rpp-pipeline restart**.

### P5 — Deferred from earlier
- Server: optional guard rejecting `path_name`/`auto_origin=true` on `/api/mission/start` when a staged/GPS_SURVEYED mission is resident (frontend "Force Start" footgun; audit gap 1). Frontend already avoids it on the verified path.
- Recorder: leftover July-10 bundles were marked INCOMPLETE (one-time, harmless).

---

## 4. Workflow to continue (test → pull → analyze)

**Deploy a code change** (per CLAUDE.md restart matrix):
- `src/*.py` → `sudo systemctl restart rpp-pipeline` (re-set `entry_prealign_enabled` after!)
- `server/**` → `sudo systemctl restart rover-server`
- `*.service` → `daemon-reload` first
- Check rover **disarmed** before any rpp-pipeline restart (it gaps the OFFBOARD heartbeat → failsafe if armed).

**Run a surveyed field test** (operator-gated — arming drives a live rover):
1. RTK **FIXED** (placement fails closed otherwise). `entry_prealign_enabled=true`, `tracking_profile=auto`.
2. Stage via the app's **DXF ref-point alignment** flow, **per-line extensions on** (spray-off PRE lead-in). **Not** CSV point missions (endpoints absent), **not** "Force Start".
3. Verified-staged start (`auto_origin:false`). Rover drives entry leg → stops on first point → pivots → tracks → stops on final point.
4. Operator on the kill switch (two-phase entry is field-new but proven for line/square).

**Pull + analyze a mission bag:**
```
# newest bundles on Jetson
ssh flash@192.168.1.102 'ls -dt ~/bags_jet/*/ | head'
scp -r flash@192.168.1.102:~/bags_jet/<bundle> ~/Vetri/rover_bags/missions_20260713/
python3 tools/analyze_mission.py ~/Vetri/rover_bags/missions_20260713/<bundle>/bag   # note the /bag subdir
```
`analyze_mission.py` runs on the Mac (pure-stdlib sqlite reader) or Jetson. Report sections: tracking / stops / pivots / speed / spray / health / config / verdict. **Trust the final-stop = direct distance; distrust coast-past on two-phase/curve until P2.**

---

## 5. Invariants (never violate)

- **I1** — stop/brake is body-axis longitudinal only (`_corner_brake_velocity`). No off-nose recenter vector (that oscillated the recenter branch).
- **I2** — keep `segment_corner_acceptance_radius=0.05`. (0.02 under-braked into pivots.)
- **I3** — stops confirmed by *measured* speed+yaw-rate dwell (`_corner_stop_satisfied`), fresh-telemetry-only timeout.
- **I4** — one change, one A/B, one field bag. No bundling, no corner-knob retunes, no cherry-picking July branches. Baseline controller pin stays `cd44884`.

## 6. Key references
- Plans: `RUNTIME_ENTRY_VELOCITY_PLAN.md` (D1–D4, status), `E2E_FIELD_AUDIT.md`, `BAG_RECORDER_PRODUCTION_PLAN.md`.
- Code: entry two-phase = `server/offboard_controller.py` `start_async` + `advance_entry_to_marking`; phase transition = `server/main.py` telemetry loop; run-0 pre-align = `src/rpp_controller_node.py` `_apply_run` (`entry_prealign_enabled`); completion = `_hold_at_completion`.
- Tools: `tools/analyze_mission.py`, `tools/bag_autorecord.py`, `tools/d0_entry_pivot/` (D0/D3 benches).
- Node names: RPP = `/rpp_controller` (not `_node`); FCU params live via `ros2 param get /mavros/param <ID>` (not in the bag).

## 7. One-line state
**Velocity-mode runtime entry (D1–D4) is built, deployed, and field-validated production-grade for line + square from arbitrary GPS-surveyed start positions; arc tracks accurately but doesn't flow (P1); analyzer coast metric needs the final-approach fix (P2); circle still to run (P3).**
