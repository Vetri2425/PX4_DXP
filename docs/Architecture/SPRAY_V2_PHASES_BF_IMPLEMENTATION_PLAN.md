# Spray V2 — Phases B–F Implementation Plan

**Written:** 2026-07-22 · **Branch:** `Upgrade_Spray` @ `64c12ff`
**Inputs:** `SPRAY_CONTROLLER_V2_PLAN.md` (Rev 3, 2026-07-15) + a read-only survey of the code as it stands today.
**Precondition:** the 9 georef runs pass and line-following is signed off. Nothing here starts before that.

---

## ▶ NEXT SESSION — start here

**State:** `Upgrade_Spray` @ `51f0ab4` (pushed). 626 tests pass. **NOTHING IS DEPLOYED** — Jetson has been down since 07-22 and is now **4 commits behind**:

```
51f0ab4  docs (this plan + 2 research docs)
211244b  survey CSV ingest + POINT-layer control points
e483d53  georef north-scale fix           <-- changes every geo mission by 0.62 %
64c12ff  vertex-drop fix (DP + provenance)
de943e4  §7 geometry-fidelity report      <-- the tool needed to read the runs
```

**Do in this order:**
1. **Deploy + verify.** Pull, restart `rpp-pipeline` *and* `rover-server` (these touch `src/`, `server/` and `path_engine/`). Run `src/test_corner_absorb_pivot.py` + `test_smoke_rpp_controller.py` **in-env** — no `src/` test has ever executed anywhere (no rclpy on the Mac).
2. **Re-stage every georeferenced mission.** `e483d53` changes projected north by 0.62 %; anything staged before it is that much long. Do not reuse old staged plans.
3. **Bench check before driving.** Stage `tes_cross_line`, confirm `/rpp/conditioned_path` keeps all 4 vertices. If it still comes out at 2 points, stop — nothing else matters.
4. **9 georef runs.** 3× `tes_cross_line` (the real test) + 3× `test_line_2` (straight control) + 3× a new long-tangent drawing (15–30 m, 2–3 deflections <2° = the road case). Measure pose vs **raw `/path`** — never `/rpp/debug[0]` (error vs the conditioned path, structurally hides this bug) and never `/spray/debug[5]` (nozzle xtrack).
5. **Then** this document: ~~plan Rev 4 (§3 divergence)~~ ✅ **done 2026-07-23** → **B0** (transport). *(Still owed: operator's per-segment-dash answer — §7.2.)*

**Smaller items now open from the ingest work:** CSV quality gating (`require_fix` / `max_lateral_rms_m`) exists in the parser but no API route sets it; a grid-only CSV needs explicit operator CRS confirmation rather than a magnitude guess; the DXF `Codes` MTEXT is still unread (POINTs cover the DXF case for now).

**Two open items not covered here:** the segment→smooth ~5 cm seam (visual-only; may already be fixed as a side effect of `64c12ff` — check `/rpp/debug[0]` vs `[40]` at the profile flip) and the spray start-delay vs nozzle-offset A/B (square @0.35 then @0.15 m/s).

> ⚠ **Session lesson worth carrying:** two bugs shipped because a test's ground truth mirrored the bug — `test_7_musthit_points_preserved` fed bare vertices and bypassed densification, and the georef test used haversine with the same wrong radius as the projection. Be sceptical of any geometry test that claims to validate against "truth."

---

## 0. Executive summary

Phase A landed cleanly and the §9 infra fix shipped with it. **Dash and point modes have complete schema validation and zero behavioural implementation** — and, more importantly, *no transport exists to deliver a mode to the node at all*. The node's own comment says so:

> `spray_controller_node.py:444` — "there is no /spray/session_config subscription yet (that lands with dash/point modes)"

So the first work item is not a mode. It is the missing pipe.

Second finding, and the one that changes the plan: **`6523a84` (speed gate → pivot-state gate) invalidated three specific pieces of the V2 plan's mode design.** The plan assumes a `min_spray_speed_mps` gate that no longer gates, and assumes the spray node does not subscribe to RPP state — it now does. Section 3 below itemises what has to be re-decided before Phase C is written.

---

## 1. Phase map — plan vs. code as it stands

| Phase | Plan says | Code today | Verdict |
|---|---|---|---|
| **A** — FSM + telemetry + §9 infra | `spray_fsm.py`, `spray_session_config.py`, node refactor, `rpp_start.sh` split | `spray_fsm.py` (7 states, ack/timeout/RECOVERY backoff), `spray_session_config.py` (all 3 modes parse), `spray_status.py` (typed, `Optional[float]`, no inf), `rpp_start.sh` critical/aux split with 2s→30s backoff | ✅ **DONE** — field parity run still pending |
| **B** — RTK/GPS gate (§7.6) | Subscribe `/mavros/gpsstatus/gps1/raw`, threshold + asymmetric hysteresis | Status dataclass *has* `gps_fix_ok` / `gps_fix_name`; **no GPSRAW subscription exists**; no threshold, no hysteresis | ❌ **NOT STARTED** — status fields are placeholders |
| **B0** — *(new, not in plan)* session-config transport | Implied by §3 but never given its own phase | **ABSENT.** No `/spray/session_config` subscription, no publisher, no API route. `mode` is hardcoded `"continuous"` (`:1200`), `mode_state` always `{}` (`:1215`) | ❌ **BLOCKS C AND D** |
| **C** — Dash | `spray_modes.py` dash arc-length toggle math | Schema only (`DashConfig` parses, round-trips, 46 schema tests). Zero references outside `spray_session_config.py` + its tests | ❌ **NOT STARTED** |
| **D** — Point | Node FSM + coordinate ingest + **planner-side hold contract** | Schema only (`PointsModeConfig`). Nothing in node, FSM, `path_engine/`, or API | ❌ **NOT STARTED, two-sided** |
| **E** — Speed-proportional flow | `spray_flow_model.py` | File absent. Actuator only ever commands `on_value`/`off_value` | ❌ **NOT STARTED** |
| **F** — Field validation | Dry-run then live per mode | — | — |

**Test coverage reality:** 29 tests in `test_spray_controller_v2.py`, all continuous. 46 in `test_spray_session_config.py`, all schema-level. **Zero behavioural tests touch dash or point.**

---

## 2. The three blockers, in dependency order

### B1. No transport (blocks C and D absolutely)

There is no way to tell the node it is in dash or point mode. The only mission-geometry channel is `/path`, whose `position.z` is a 2-bit field now **fully consumed** (bit0 spray, bit1 must-hit as of `64c12ff`). Dash distances and point coordinate lists cannot ride there.

`server/routes/spray.py` publishes only `/spray/manual` (Bool). `spray_params.py` exposes a curated 13-key ROS-param surface with no mode key. `SprayTestRequest` has no `mode` field.

### B2. Arc-length substrate is not dash-ready (blocks C)

`_build_path_model` (`:138`) computes a static `cumulative_s` table — monotonic by construction, fine. But live position comes from `_project_onto_path` (`:199`), which does a **fresh global nearest-segment search every tick** with no memory of the previous segment. On retrace or self-overlapping geometry `s` can jump backward or forward, which would double-fire or skip dash toggles.

Plan §7.2 already specifies the fix (monotonic `s_dash`, jump rejection, `jump_reject_accept_after` recovery). **None of it exists** — grep for "wedge"/"monotonic" returns nothing.

### B3. The pivot gate suppresses stationary spray (blocks D)

`_pivot_is_active()` (`:884`) returns True when `/rpp/segment_debug[1] == CORNER_ALIGN (3)`, and `_auto_safety_status` (`:934`) hard-blocks: `return False, "pivoting in place"`.

A point-mode dwell **stops the rover and sprays**. Nothing in the code distinguishes "stopped to pivot" from "stopped to dwell." Left alone, the gate suppresses every dot.

---

## 3. ⚠ Plan-vs-code divergence — re-decide before writing Phase C

The V2 plan is Rev 3, dated **2026-07-15**. `6523a84` ("speed no longer gates on/off; pivot state does") landed **2026-07-18**. Three plan sections are now describing a system that no longer exists:

| Plan text | Reality | Consequence |
|---|---|---|
| §5: min-speed gate is **mode-conditional** — swap `speed >= min_spray_speed_mps` for `speed <= point_arrival_max_speed_mps` in point mode | `min_spray_speed_mps` is still *declared* (`:393`) but **is not a gate**. Code comment `:930`: *"speed is intentionally NOT compared against a minimum here… slow means thin (flow control), never off."* | The prescribed swap is a no-op. The real point-mode blocker is the **pivot gate** (B3), which §5 never contemplated. **§5 must be rewritten.** |
| §7.2: corner-crossing detected "**purely from the node's own velocity** — `speed_mps < min_spray_speed_mps` — with **no dependency on any RPP substate topic**; the spray node does not subscribe to RPP state today and V2 deliberately keeps it that way" | The node **does** subscribe to `/rpp/segment_debug` (`_segment_debug_cb`, `:877`) and gates on it | The stated design constraint is already violated by shipped code. Dash's deferred-actuation rule must be re-specified against the pivot state, which is *better* (a discrete state can't dither — that was the whole point of `6523a84`) but it is **not what the plan says**. |
| §7.3 point mode: "a dwell sprays at a standstill by definition" | True, and the pivot gate blocks exactly that | Point mode needs an explicit gate exemption that the plan does not specify. |

**Recommendation:** amend the plan (a Rev 4 touching §5, §7.2, §7.3) *before* writing Phase C code. Roughly a half-day of writing, and it prevents implementing against a stale contract — which is precisely how `16480d9` became a false fix.

> ✅ **DONE 2026-07-23 — `SPRAY_CONTROLLER_V2_PLAN.md` is now Rev 4.** §5 rewrites the gate table around the discrete pivot-state gate (min-speed struck through as a non-gate); §7.2 re-specs dash corner-deferral against `/rpp/segment_debug[1]==CORNER_ALIGN` and records the **open per-segment-dash decision**; §7.3 adds the scoped, node-side point-mode pivot-gate exemption. Phase C may now be written against Rev 4. The one thing still owed the operator is the per-segment-dash answer (does the dash pattern reset per surveyed line, or run continuously across the mission).

**Good news in the divergence:** the pivot-state gate is a *stronger* substrate than the velocity test the plan assumed. §7.2's whole concern was that a dash toggle mid-pivot must flip phase (arc-length doesn't care about time spent stationary) while deferring actuation. A discrete `CORNER_ALIGN` state expresses that more cleanly than a speed threshold that dithers at the frozen corner-crawl speeds (0.08 / 0.08 / 0.03).

---

## 4. Implementation plan

### Phase B0 — Session-config transport `[S]` · *new, blocks everything*

**Deliverables**
1. Node: subscribe `/spray/session_config` on **RELIABLE + TRANSIENT_LOCAL** (mirror the existing `_path_qos()` pattern). Parse via the existing `parse_session_config`. `schema_version` mismatch → log + keep last-known-good (fail static).
2. Node: `_publish_status` reports the **real** mode instead of hardcoded `"continuous"` (`:1200`), and populates `mode_state` (`:1215`) instead of `{}`.
3. Server: publish a `SpraySessionConfig` on mission load.
4. Server: **mission clear must publish a cleared config**, not just stop publishing — TRANSIENT_LOCAL means a restarted node re-latches the last message and would silently resurrect stale geometry (plan §3, explicit).
5. API: mode selectable. Extend the staged-mission payload rather than inventing a new route.

**Contract preserved:** the node remains the *only* parser, computes its own fingerprint, never validates a caller-supplied one. That was defect #3 and the cause of the 409 storm in `spray_param_contract_and_degraded_load`.

**Gate:** bench — publish each of the 3 configs, confirm `/spray/status.mode` reflects it and continuous behaviour is byte-identical to today. Clear the mission, restart the spray node, confirm it does **not** re-latch stale geometry.

---

### Phase B — RTK/GPS fix-quality gate `[S]`

Fully specified in plan §7.6 and it needs no amendment.

**Deliverables:** subscribe `/mavros/gpsstatus/gps1/raw`; param `spray_min_fix_type` (default 6 = RTK_FIXED); `gps_fix_timeout_s` (default 2.0, looser than the 0.5 s pose/velocity gates because GPSRAW is slower); **asymmetric hysteresis** — fail instantly on bad-or-stale, re-open only after `gps_recover_hold_s` (1.0 s) continuously good. Populate the existing `gps_fix_ok` / `gps_fix_name` status fields. Staleness is a *distinct* reason string from bad fix.

**Gate:** bench with simulated fix-type flapping incl. a stale-GPSRAW case; then a field RTK dropout during a live mark run.

*Order note:* B is independent of B0 and could run either side of it. Doing it here keeps the gate stack in one working set, since C and D both extend it.

---

### Phase C — Dash mode `[M]`

**Prerequisite:** plan Rev 4 (§3 above) + B0.

**Deliverables**
1. `src/spray_modes.py` (new) — pure functions, no rclpy.
2. **Monotonic arc-length tracker** — the real work. Cursor-based projection seeded from the last segment index rather than a global search; `s_dash = max(s_dash_prev, projection.s)`; reject single-tick jumps > `speed × dt × jump_tolerance_factor` (3.0); **recovery rule** — accept after `jump_reject_accept_after` (5) consecutive self-consistent rejects, or the guard wedges permanently on legitimate relocation.
3. Dash toggle math on cumulative mission arc-length, continuous across corners and entity boundaries. Boundaries generated dynamically from `s_at_last_toggle + {on,off}_distance_m`, then consumed by the **existing** boundary-lead machinery so solenoid compensation is shared with continuous mode, not duplicated.
4. **Arming criterion:** dash metering stays unarmed until `xtrack_error_m <= max_xtrack_error_m`, reusing the §5 threshold — no new param. Before that, `geometry_desired = False` (the entry transit must not meter).
5. **Corner deferral, re-specified against the pivot state:** phase still flips at the correct `s` while stationary; actuator command deferred until the pivot gate re-opens. Neither leak paint at a standstill nor drift the pattern.

**Tests** (`src/test_spray_dash_v2.py`): corner mid-dash; dash shorter than one leg; dash spanning 3+ legs; jump rejection recovers after N self-consistent ticks and stays frozen for noise; unarmed until xtrack satisfied.

**Gate:** bench on a synthetic multi-leg path with a corner mid-dash **before any field run** (plan §8, explicit).

---

### Phase D — Point mode `[L]` · *two-sided*

**Prerequisite:** B0 + a decision on B3.

**D-node**
1. Point FSM in `spray_modes.py`: `TRANSIT → ARRIVING → HOLDING → DWELLING → ADVANCING`.
2. Arrival = distance ≤ tolerance **and** (heading within tolerance, or `heading_tolerance_deg: null` for position-only) **and** speed ≤ `point_arrival_max_speed_mps`, held continuously for `arrival_settle_s`. A blip resets the timer — no partial credit.
3. Advance waits for `OFF_CONFIRMED`, not merely for `dwell_s` to elapse — so a retrying OFF cannot let the rover move off a still-spraying dot.
4. **Pivot-gate exemption (B3).** Recommended: exempt when `mode == "point"` **and** the rover is within `arrival_tolerance_m` of the active target. This keeps the fix **node-side** and avoids touching the frozen RPP controller. Do *not* simply disable the gate in point mode — that re-opens corner leakage on the transits between dots.
5. **Unreachable-target watchdog** with `point_arrival_timeout_s` (60 s): skip, log, surface in telemetry, never silently count as sprayed. **Index-synchronised** — `target_index` derives from proximity over remaining coordinates, never a free-running counter, or spray watches point *i+1* while the rover sits at *i*.

**D-planner** (`path_engine` / server — the half that is easy to forget)
6. Emit a per-point **hold** at each coordinate, reusing the field-proven final-segment stop machinery (`segment_endpoint_approach_speed`, stop-dwell). Hold duration is computed from the same `SpraySessionConfig` dwell params so planner and node cannot drift.
7. Without this the rover drives *through* the points, ARRIVING never satisfies, and every point times out. **This is the single most likely way Phase D fails.**

**Synergy worth exploiting:** `PlannedPath.must_hit` (landed `64c12ff`) is already a per-waypoint "declared point" flag travelling end-to-end. Point mode's coordinate list and the must-hit set are the same data. This is also where **Layer 3** (DXF `Points`-layer → must-hit) stops being redundant: your georef drawings already carry POINT entities on every vertex, which is precisely a point-mode target list.

**Tests** (`src/test_spray_point_v2.py`): settle-reset-on-blip; dwell + OFF-confirm before advance; position-only vs heading-gated; empty and 1-point lists; watchdog skip does not advance ahead of the rover.

**Gate:** bench on a 3+ point synthetic mission before field.

---

### Phase E — Speed-proportional flow `[M]`

`src/spray_flow_model.py` — duty scales linearly between `min_flow_value` and `on_value` with speed, slew-limited so the pump is never step-commanded. Layered **on top of** the FSM, active only while `ON_CONFIRMED` — no new safety states. Our actuator already accepts a continuously variable command, so unlike classic ag PWM there is nothing to simulate.

Runs **after** modes are stable, so calibration reflects real scenarios (straight continuous, dash toggling, and the corrected antenna path if the nozzle-offset plan has landed) rather than a synthetic sweep.

---

### Phase F — Field validation

Dry-run (no paint) first on **every** mode — existing operator rule — then live paint per mode, including a deliberate speed-varying pass to confirm line density stays visually constant. RTK-dropout live test if not covered in B.

---

## 5. Recommended order, and the one real choice

```
Plan Rev 4 (§5/§7.2/§7.3)  →  B0  →  B  →  C (dash)  →  D (point)  →  E  →  F
                                            └── or D before C, see below
```

**Dash before point** is the lower-risk default: node-only, no planner work, no gate exemption, and the arc-length cursor it builds is reusable machinery.

**But point may be worth more to you.** For road pre-marking, points-only *is* the product — a pre-marking robot lays reference marks for a following striping crew; it does not paint the finished line. Point mode also consumes the must-hit data you just landed.

The honest trade: point delivers more product value, dash is the safer engineering step. If road pre-marking is the near-term goal, run **D before C** and accept the two-sided coordination cost.

## 6. Risks

| Risk | Mitigation |
|---|---|
| Implementing Phase C against the stale plan | Rev 4 first — non-negotiable. This is how `16480d9` became a false fix |
| D-planner half forgotten → every point times out | Treat D as two tickets, planner one first; bench a 3-point mission before any node work is called done |
| Monotonic-`s` guard wedges | The recovery rule is specified; make it an explicit test, not a code comment |
| Pivot-gate exemption too broad → corner leakage in point mode | Scope the exemption to "within `arrival_tolerance_m` of the active target," never mode-wide |
| Per-segment dash patterns (road: 3 m+6 m vs 6 m+3 m by section) | Out of scope here. `position.z` is full at 2 bits; this needs per-segment config in `PathSegment.metadata` (the dict exists, carries only `geometry_type` today). Design Phase C's config to carry a pattern **per segment** rather than one global pair — cheap now, expensive later |
| Regression in continuous mode | B0 gate is byte-identical continuous behaviour; 29 existing tests must stay green |

## 7. Reference — do not reuse

`a62cab2` ("three spray modes… with mission-bound config", 2026-06-22) implements all three modes on `feat/point-mission-gps-flow` and other branches. It was **rejected for reuse** ("real bugs"), and its point mode is a *server-side async orchestrator* — architecturally opposite to V2's node-owned single source of truth. Read it for the dash arc-length math; do not lift it.
