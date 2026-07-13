# Implementation Plan — Runtime Entry + Whole Mission on Velocity OFFBOARD

**Status:** PRIMARY plan for runtime entry. Supersedes `OFFBOARD_POSITION_MODE_PLAN.md` and the position-first framing in `MISSION_ALIGNMENT_RUNTIME_ENTRY_PLAN.md` §5. Those remain as the **documented fallback** (see §9).
**Base branch:** `test/colinear-fix` (controller pin `cd44884`). Frozen corner knobs stay frozen.
**Date:** 2026-07-13 (rev 2 — D1 re-specified as two-phase publish after review found the flag-split hole)
**Prereqs done:** Epic 1 (placement hygiene / scale gate), Epic 2 (live EKF placement `P_live = P + L − R_anchor`), Auth. Do not reopen.
**Status gate:** CONDITIONAL GO. Do **D0 first**. Do not code D1–D4 until D0 PASSES. D1 uses the two-phase model (§4) — the naive "prepend one run" does **not** work under `_split_runs_by_flag` (see §4 D1 note).

---

## 0. The decision, in one paragraph

The whole mission — runtime entry, marking, corners, endpoint stop — can run on a **single continuous velocity-OFFBOARD stream**, reusing primitives already field-validated on this branch. This is chosen over the firmware position-mode plan because the **pivot** in both approaches bottoms out in the identical firmware spot-turn FSM (`DifferentialVelControl`, overlay build) — so position mode offers **no structural gain on the pivot** while adding three risks velocity mode does not have: a ~500 ms mask-switch lag, unproven OFFBOARD position ingestion (G1 still OPEN), and a firmware change (Phase C) for line-direction yaw.

**Honest scope of that claim (rev 2):** "same FSM" is true for the **pivot only**. The **stop** is a real tradeoff, not a wash — position mode gets a firmware decelerate-to-`NAV_ACC_RAD` profile; velocity mode relies on the companion's active-brake timing (`_corner_brake_velocity` + measured-stop dwell). Velocity is the right primary **while G1-OFFBOARD is OPEN**, but it is "lower net risk," not "risk-free" or "position mode is worthless."

---

## 1. The decisive technical finding (why this is possible)

Read the baseline's own corner-pivot comment against the position plan's source audit:

- **Velocity mode** — [`src/rpp_controller_node.py:2449-2470`](../src/rpp_controller_node.py#L2449): *"PX4 DifferentialVelControl in OFFBOARD **velocity** mode… the firmware's native state machine sees a large heading error (>`RD_TRANS_DRV_TRN`), enters SPOT_TURNING (zero forward throttle), rotates in place the short way; once aligned (<`RD_TRANS_TRN_DRV`) transitions to DRIVING."*
- **Position mode** — `OFFBOARD_POSITION_MODE_PLAN.md` §2: the pivot is *"overlay `DifferentialVelControl.cpp:172-176` spot-turn FSM."*

**Same FSM.** `goToPositionMode` computes a pure-pursuit bearing and feeds `DifferentialVelControl`; the companion computes a bearing and feeds `DifferentialVelControl` via the velocity vector. The in-place spot-turn (zero throttle >40°, short-way via `wrap_pi`, no reverse-flip) is one piece of code, invoked identically in both modes. The position plan's "angle-independent clean pivot" is therefore **not unique to position mode** — and the plan itself concedes (§2 rev-2) the pivot is threshold-gated at 40° and *arcs* below that, in **both** modes.

Consequence: on the **pivot**, the velocity path carries less risk — it never switches masks (no §7 500 ms lag), needs no `rover_position_setpoint` ingestion proof, and needs no firmware. It does **not** inherit position mode's firmware decel-to-stop; that stop quality is D0/D3's job to prove on the companion side.

---

## 2. Phase-by-phase status (evidence, not hope)

| Phase | Velocity-mode status | Source |
|---|---|---|
| Marking (line / arc) | **Proven** ≤2 cm (arc 1.46 / lshape 0.90 / square 0.87 / U-turn 1.06 cm) | 06-15 bags |
| Corner stop + pivot (≤~120°) | **Proven** — segment profile; square 90° + triangle corners field-validated | `_control_segment_profile` |
| Completion stop | **1 known bug** — bare-zero setpoint coasts (1.08 m past on Line_2m) | fix exists: `277d3dc` |
| Entry: drive-to-point + stop | **Reuses proven** run-boundary stop | `_hold_before_run_advance` |
| Entry: **large arbitrary-yaw pivot from dead stop** | **The one unproven regime** — same primitive at >120° | `_corner_pivot_velocity` |

Exactly **one** unproven regime remains, and it is the same one the position plan's G3 is also unproven on. Velocity reaches it by extending a primitive already validated at 90–120°; position reaches it through an unproven firmware ingestion path.

---

## 3. Architecture — the entry is a separate spray-OFF velocity **publish** (two-phase)

The runtime entry is **not a new subsystem** and needs **no RPP conditioning change**. It is a self-contained single-segment velocity path, published *before* the marking path, orchestrated by the server.

```
 PLACE (Epic 2 — done):  marking points → live EKF frame
      │
 PHASE 1 — ENTRY publish:   path_entry = [ live_pose , entry_target ]   spray OFF
      RPP tracks the single FINAL segment → approach-decel →
      _hold_at_completion (D3) active-brakes to a CONFIRMED stop at entry_target → DONE
      rover holds zero-velocity; twist streamer keeps the 50 Hz OFFBOARD heartbeat alive
      │
 server detects DONE-settled at entry_target  (ENTRY state: DONE ≠ mission-complete)
      │
 PHASE 2 — MARKING publish:  path_mark = placed marking runs   UNCHANGED, frozen knobs
      run 0 pre-align (D2) pivots in place to face the first line
        └ _corner_pivot_velocity + _run_alignment_hold release gates (proven)
      → track → MARK → corners → completion (baseline RPP)
```

**Why two publishes, not one prepended run** (the review's D1 finding): `_split_runs_by_flag` ([`:1030`](../src/rpp_controller_node.py#L1030)) splits runs **only** on spray-flag transitions. The entry leg (OFF) and the first PRE (OFF, extensions) are flag-homogeneous, so a single published path **merges them into one run** — there is no run boundary at `entry_target`, and `_hold_before_run_advance` never fires there. And even a genuine boundary is **conditional**: `_hold_before_run_advance` skips the stop when the next run is collinear (`_next_run_requires_alignment`, [`:1440`](../src/rpp_controller_node.py#L1440)). A separate Phase-1 publish gives `entry_target` an **unconditional final-segment stop** and keeps `_split_runs_by_flag` / conditioning **frozen** (I4). The Phase-1→Phase-2 handoff is **velocity→velocity** — no mask switch, so the position plan's §7 500 ms lag does not apply.

*Rejected alternative:* editing `_split_runs_by_flag` to force a break at `entry_target`. That touches a core conditioning function that every shape depends on — an I4 violation for a problem the two-phase publish solves server-side.

`entry_target` = `placed_wps[0]` (extensions off) or the **first PRE** of the first mark span (extensions on) — the same target the position plan's D3/§5.3 defines, delivered as a **velocity path**, never a densified path and never a position setpoint.

| Concern | Owner |
|---|---|
| Free-space drive to entry_target + confirmed stop | RPP final-segment profile + `_hold_at_completion` (D3), on `path_entry` |
| Detect arrival, publish marking path | Server (DONE-settle watcher in ENTRY state) |
| Pivot to face the first marking line | Run-0 pre-align (D2) at the start of `path_mark` |
| Mark track + corners + endpoint stop | Baseline RPP (unchanged) |
| Spray forced OFF through the entry | `path_entry` flags all OFF + MissionState `ENTRY` gate |

**Dependency interlock (rev 2):** D2 and D3 are **not optional add-ons** — the two-phase model makes them load-bearing. The Phase-1 entry stop *is* a completion, so it needs the **D3** latch or it coasts past `entry_target` (the 1.08 m bag). The line pivot now happens at the Phase-2 **run-0** start, so it needs **D2** or run 0 merely arcs onto the line. Land order therefore becomes D0 → D3 → D1 → D2 → D4 (D3 before D1, since D1's entry stop depends on it).

---

## 4. Work breakdown (function-level, isolated deliverables)

Each item lands **alone** with its own before/after bag. Order: **D0 (verify) → D3 → D1 → D2 → D4** (D3 before D1 because D1's entry stop depends on the completion latch).

### D0 — Make-or-break bench (verify the ONE unknown FIRST) — 0 code
Settle the only real risk before building anything. See §5.

### D3 — RPP: re-land the completion latch (277d3dc) *(isolated; lands before D1)*
- A *correct* fix for a real deterministic bug: PX4 velocity-OFFBOARD coasts on a bare zero setpoint, so the rover drifts past `xy_goal_tolerance`, the goal check goes false, tracking resumes, and it drove 1.08 m past the goal (bag 2026-07-10_20-07 Line_2m).
- Re-land `_hold_at_completion` + `_completion_stop_pending` **alone** — active-brake to rest, latch DONE only on `_completion_settle_satisfied()`. Cleared per-run in `_apply_run` and per-path in `_path_cb`, never from the tracking fall-through.
- **Take nothing else** from `fix/runtime-entry-stop` (no `corner_stop_hold_s`, no param retunes, no densification).
- **Why first:** the Phase-1 entry stop (D1) *is* a completion. Without this latch it coasts past `entry_target`, and D1 cannot land cleanly.

### D1 — Server: two-phase entry publish  *(most of the work; pure Python, unit-testable)*
- `entry_target_from_plan(placed_wps, spray_flags, extensions_enabled) → (n, e)` — first WP or first PRE.
- In `OffboardController.start_async` (`server/offboard_controller.py`), after placement resolves the marking points, run the two-phase sequence (§3): **Phase 1** publish `path_entry = [live_pose, entry_target]` (flags all `False`); enter `ENTRY`; watch for RPP DONE-settled at `entry_target`; **Phase 2** publish the marking path and transition to MARKING. `live_pose` = current `(pos_n, pos_e)` from `get_state()`.
- **Do not** rely on prepending the entry as a run inside one path — it merges with the first PRE under `_split_runs_by_flag` (§3). Two publishes is the mechanism.
- **Do not densify the entry leg.** It is a 2-point straight segment; RPP's lookahead + final-segment decel tracks and stops it directly. (No companion entry-Path anti-pattern — I4.)
- Expose on start response: `entry_target_ned`, `placement_translation`, `placement_mode`, `entry_phase`.
- Tests: entry_target selection (ext on/off); Phase-1 path is 2 points, spray-OFF; DONE-settle → Phase-2 publish of the full marking path; ENTRY does not auto-complete the mission on Phase-1 DONE.

### D2 — RPP: run-0 pre-align  *(1 isolated control-path change; required, not optional)*
- Today `_apply_run` sets `_run_align_pending` only when a `prev_run` exists ([`:1340`](../src/rpp_controller_node.py#L1340)) — so **run 0 never pivots in place**; it arcs onto its first heading via the forward-cone clamp. In the two-phase model the pivot onto the marking line happens at the **start of the Phase-2 path (run 0)**, so this gap must be closed or the rover arcs onto the line from the entry stop.
- Add: for run 0, set `_run_align_pending = True` (and `_run_align_turn_rad`) when initial `|yaw_err|` to the first segment exceeds `segment_corner_threshold_deg`. Reuse `_corner_pivot_velocity` + the exact `_run_alignment_hold` release gates — **no new pivot code**.
- Gate behind a param (`entry_prealign_enabled`, default matches whatever D0 proves). A/B against arc-on.

### D4 — Server: single `ENTRY` MissionState *(blast-radius table, §7)*
- One new state, `ENTRY`, spanning the entry run. Far simpler than the position plan's `ENTRY_POSITION` + `HANDOFF` pair because **there is no mask switch and no handoff** — it is one continuous velocity stream that flows into MARKING when RPP crosses into run1.

---

## 5. Make-or-break bench (D0) — the only real unknown

**Goal:** confirm `_corner_pivot_velocity` executes a large arbitrary-yaw spot-turn from a dead stop cleanly, in velocity OFFBOARD, before any build work.

**Setup:** place `entry_target` (or a synthetic 2-point path) so the rover must spot-turn **~150–180°** at the stop point to face the next segment. Auto-origin LOCAL_NED is fine; no surveyed start needed for D0.

**Record:** `/mavros/local_position/pose`, `/rpp/velocity_ned`, `/mavros/setpoint_raw/local`, `/rpp/segment_debug`, `/mavros/state`.

**PASS bar:**
- Rover brakes to a confirmed stop (measured speed < `segment_stop_speed_threshold`, dwell held) **before** the pivot begins.
- Spot-turn completes with **no reverse-flip** (fwd_component stays > 0 throughout), **no oscillation**, monotonic heading convergence.
- Settles within `segment_heading_tolerance_deg` and releases into TRACK.

**PASS → build D3/D1/D2/D4; velocity mode is the whole-mission answer.**
**FAIL** (velocity spot-turn genuinely cannot do >120° from dead stop) → scope position mode for the **entry pivot only** (never corners, never whole mission) per §9. This is the exact fallback the position plan's own §12 concedes.

D0 is a **hard gate, not a formality**. Field-validated 90° tracking corners are *not* evidence that a ~180° spot-turn from a dead stop at an entry seam will be clean — that regime is genuinely untested. No build work starts until this bag PASSES.

---

## 6. The four invariants — guardrails that encode why the other branches failed

These are hard rules and PR-review gates. Each maps to a concrete field failure on a discarded branch.

| # | Invariant | Failure it prevents |
|---|---|---|
| **I1** | **Body-axis brake only.** Every stop/brake command is longitudinal along the nose (`_corner_brake_velocity`). Never an off-nose "recenter/servo-to-point" vector. | `feat/entry-pivot-recenter` added `_corner_hold_velocity` (off-nose servo) → recenter fought the pivot through PX4's bearing-from-velocity heading → **oscillation** (24 commits, multiple reverts). |
| **I2** | **Acceptance radius stays 5 cm** (`segment_corner_acceptance_radius = 0.05`). The pivot arms only after the brake has room to null residual speed. | `fix/corner-stop-production` retuned 0.05→0.02 → under-braked into the pivot → the very "still rolling" problem the recenter branch then chased. |
| **I3** | **Stop confirmed by *measured* speed + yaw-rate dwell**, fresh-telemetry-only timeout ([`:3248`](../src/rpp_controller_node.py#L3248)). Never advance to a pivot on position alone. | Fresh-telemetry-above-threshold must never time out into a pivot — that started pivots from a drifting rover. |
| **I4** | **One change, one A/B, one bag.** Each deliverable lands alone and is field-measured against the frozen baseline. | Bundling ~11 interacting changes + a mid-branch self-revert (`451d0cb` "restore validated defaults") made the ref branch un-attributable and "misled agent reasoning." |

---

## 7. `MissionState` blast radius (mandatory — do not silent-add the enum)

| Consumer | `ENTRY` |
|---|---|
| Spray auto | **OFF / deny** (entire entry run is spray-OFF) |
| `/load-to-controller` | **409** (busy) |
| Password change | **409** |
| E-stop / abort | **allowed** (same stop-path / MANUAL / disarm) |
| Telemetry `mission_state` | show `ENTRY` |
| Bridge health | treat as active mission |
| Auto-complete (RPP DONE) | **ignore** until the stream reaches run1 (MARKING) |
| Entry timeout / max-distance | safety abort (existing watchdog extended to `ENTRY`) |

No `HANDOFF` state exists — there is no mask switch to guard.

---

## 8. Staged rollout

1. **D0 bench** — large-yaw pivot PASS/FAIL recorded in this doc.
2. **D3 completion latch** — before/after Line_2m bag; confirm ≤ `xy_goal_tolerance`, no coast-past.
3. **D1 two-phase entry publish** — LOCAL_NED auto-origin square; confirm Phase-1 stop at entry_target, DONE-settle → Phase-2 publish, pivot onto side 1, unchanged mark xtrack.
4. **D2 run-0 pre-align** — A/B: arc-on vs pre-align, adversarial initial heading.
5. **D4 MissionState** — spray-deny / 409 / telemetry integration tests.
6. **GPS_SURVEYED end-to-end** — place → entry → stop at PRE/wp0 → pivot → mark; spray OFF in entry; watch `POSE_GLOBAL_MAX_SKEW_MS` on first live surveyed start.
7. **Shape campaign** — L / square / triangle / arc; auto-bag; compare to validated-day metrics.

---

## 9. Fallback (only if D0 FAILS)

If the velocity spot-turn cannot do >120° from a dead stop, adopt `OFFBOARD_POSITION_MODE_PLAN.md` **scoped to the entry pivot only**:
- Prove **G1-OFFBOARD** first (sole publisher through the twist bridge; `rover_position_setpoint` populated; clean 5 cm stop). Never bolt a second publisher on the setpoint topic.
- Position mode owns **only** the entry transit gap; corners and marking stay velocity RPP.
- Handle the ~500 ms mask-switch lag (§7 of that plan) inside a spray-OFF buffer.
- **Never** use position mode for the whole mission or for mark corners.

The whole-mission velocity path (this doc) is primary precisely because it avoids all of the above unless the tail-angle pivot forces it.

---

## 10. Acceptance criteria

- [ ] **D0:** large-yaw (~150–180°) velocity spot-turn from dead stop — clean, no reverse-flip, no oscillation, settles in tolerance (bag in this doc).
- [ ] **D3:** completion latch — rover stops ≤ `xy_goal_tolerance`, no coast-past, on the Line_2m repro.
- [ ] **D1:** two-phase publish — Phase-1 stop at entry_target, DONE-settle → Phase-2 marking publish, pivot onto first line, mark xtrack class = baseline on square/line; no densified entry path on `/path`; ENTRY does not auto-complete on Phase-1 DONE.
- [ ] **D2:** run-0 pre-align — adversarial initial heading arrives on-line cleanly; A/B recorded.
- [ ] **D4:** `ENTRY` state — spray denied, `/load` 409, telemetry shows `ENTRY`, abort/estop mid-entry safe.
- [ ] LOCAL_NED auto-origin square still works with no surveyed entry.
- [ ] **Corner knobs unchanged** (`slowdown=0.50`, `brake_cap=0.08`, acceptance `0.05`, no `corner_stop_hold_s`). Invariants I1–I4 upheld in every PR.

---

## 11. One-line summary

**Place (done) → publish a spray-OFF entry path, stop at entry_target (completion latch), then publish the marking path and pivot onto the line (run-0 pre-align) → mark on baseline RPP; two velocity publishes, no mask switch, no firmware, no densified entry path, no `_split_runs_by_flag` change; verify the large-yaw entry pivot once (D0) before any build.**
