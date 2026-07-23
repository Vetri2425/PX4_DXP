# RPP Progress + Spray Handshake — Implementation Task List

Companion to [`RPP_PROGRESS_HANDSHAKE_DESIGN.md`](RPP_PROGRESS_HANDSHAKE_DESIGN.md).
Target branch: `Upgrade_Spray` · Frozen-controller rule: **every controller change ships default-OFF behind a named A/B; default-OFF must be byte-for-byte the frozen path.**

## Legend
- `[ ]` todo · `[~]` in progress · `[x]` done
- **Node/file** targets are the primary edit sites; verify against tree before editing.
- Each phase has a **Definition of Done (DoD)** = the gate to merge that phase to `baseline_master`.
- Ordering is a dependency chain: G0 → G1 → {G2 ∥ G3} → G4 → G5 → G6. G2 and G3 are independent and can run in parallel after G1.

---

## G0 — Scaffolding & contracts (no behavior, prerequisite for all)

Purpose: land the shared vocabulary and topic plumbing so every later phase is additive.

- [x] **G0.1** ~~Decide placement~~ **RESOLVED: separate `MissionPhase` enum (values 0–11 per design §3) on a separate `/rpp/progress` topic** — NOT appended to `SegmentStateCode`, NOT on `/rpp/segment_debug`. Rationale: tracking-state and mission-phase are orthogonal co-occurring axes (one enum slot can't hold both); separation makes the frozen `/rpp/segment_debug` contract structurally untouched. Define the enum in a shared module importable by both nodes.
- [x] **G0.2** JSON payloads + (de)serialize for the 4 channels — `src/mission_progress.py` (`ProgressMsg`, `MilestoneMsg`, `PointDoneMsg`, `AdvanceMsg`; strict-out, lenient-in, NaN→null). `std_msgs/String` JSON, no custom-msg build step.
- [x] **G0.3** QoS specs centralized as plain data in `src/mission_progress.py` (`PROGRESS_QOS` BEST_EFFORT/1, `MILESTONE_QOS`/`POINT_DONE_QOS` RELIABLE/10, `ADVANCE_QOS` RELIABLE/1, all VOLATILE). rclpy adapter `src/mission_progress_ros.py:qos_profile()` maps spec→`QoSProfile` (on-robot only; keeps the pure module Mac-testable).
- [x] **G0.4** All new params declared, default = frozen behavior, unread. RPP (`rpp_controller_node.py` after point-hold group): `progress_publish_enabled=False`, `point_precise_stop_enabled=False`, `point_arrival_tolerance_m=0.02`, `precise_stop_mode=feedforward`, `precise_stop_creep_speed=0.05`, `precise_stop_max_s=8.0`, `point_execution_mode=auto`, `manual_wait_timeout_s=0.0`, `point_hold_max_s=10.0`. Spray (`spray_controller_node.py` after pivot-gate group): `consume_rpp_progress=False`, `progress_timeout_s=0.3`.
- [x] **G0.5** `src/test_mission_progress.py` — enum values frozen, milestone names, QoS specs, 4 round-trips, NaN/Inf→null, lenient-parse-never-raises, out-of-range→IDLE. **15/15 pass on Mac; full pure suite 210/210.**

**DoD:** enum + schemas + QoS + params exist and import cleanly; all params default to frozen behavior; nothing publishes/consumes yet. ✅ Contract module + tests green on Mac (210/210); both nodes AST-parse; param decls behavior-neutral. ✅ **Deployed + in-env frozen regression green on Jetson (2026-07-23).** **G0 COMPLETE.**

---

## G1 — Progress publication (RPP additive, pure observability)

Purpose: RPP announces phase + distance-to-boundary. No consumer. Zero behavior change.

- [x] **G1.1** Phase classifier landed as a **pure module** `src/progress_classifier.py` (`classify()`, `next_mark_boundary()`, `segment_mark_flags()`, `milestones_for()`). Node gathers primitives (seg_idx, projection `t`, `along_s`, spray flags, must-hit ranks, speed) and calls it — projection only, no new geometry. Both boundary regimes: MARK↔non-MARK transitions (continuous/dash) and next must-hit point (point).
- [x] **G1.2** `/rpp/progress` published every control tick behind `progress_publish_enabled`, via a thin wrapper: `_control_loop` → `_control_loop_impl` (frozen body, byte-for-byte) → gated, **exception-isolated** `_publish_progress_tick` (an observability bug can never crash control).
- [x] **G1.3** `/rpp/milestone` emitted once per phase/point edge with monotonic `seq` (`_emit_progress` + `milestones_for`): MARK_START/STOP, PRE_START, AFT_STOP, AT_POINT, DWELL_DONE_RPP, REACHED_END.
- [x] **G1.4** `stopped` (measured-speed < `segment_stop_speed_threshold`) and signed `xtrack_m` populated from the same projection the tick used (tracking pose = `_last_pos` − `_ekf_reset_offset`).
- [x] **G1.5** `src/test_progress_classifier.py` — phase table (continuous/extensions/point), `dist_to_next_boundary` math, milestone edges. **18 tests; full pure suite 228/228 on Mac.**
- [ ] **G1.6** Bench in-env (Jetson): `src/test_progress_publication.py` written (progress ON → emits + MARK_TRACKING + MARK_START milestone + monotonic seq; progress OFF → silent, control still drives). **Needs Jetson run** (rclpy-only).

**DoD:** with flag ON, progress/milestone streams correct on bench; with flag OFF, controller output byte-for-byte frozen. ✅ Mac: classifier + contract green (228/228); wrapper keeps impl untouched; new param `progress_approach_dist_m` (0.30) + fields all read only under the flag. ⏳ **Remaining:** Jetson run of `test_progress_publication.py` + frozen regression (smoke + segment_stop + corner_pivot) with the flag OFF.

---

## G2 — Spray boundary sourcing (spray additive, A/B)

Purpose: kill dual-projection drift for moving marks by sourcing the boundary from RPP.

- [x] **G2.1** `spray_controller_node.py` subscribes to `/rpp/progress` (`_rpp_progress_cb`); behind `consume_rpp_progress`, `_make_spray_decision` sources `dist_to_next_boundary_m` + `next_boundary` for the continuous lead math (`solenoid_open_delay_s`, `on/off_overspray_margin_m`) — **same equations, single source**. Mapping in `_rpp_kind_for` (MARK_START→TRANSIT_TO_MARK, MARK_END/terminal-REACHED_END→MARK_TO_TRANSIT). The xtrack safety gate stays on the **local** projection (independent, per §12).
- [x] **G2.2** Staleness fallback in `_rpp_boundary_inputs`: no progress within `progress_timeout_s` (or flag off / non-continuous mode / no message yet) → `(None, "", inf)` → `_make_spray_decision` uses the local `/path` projection **byte-for-byte** for that tick. Source switches (rpp↔path↔off) logged once, rate-limited (`_note_rpp_source`).
- [x] **G2.3** ~~DashMeter resets at region edges~~ **DECLINED — conflicts with a locked operator decision.** `DashMeter` is *continuous across mission, does NOT reset at entity/corner boundaries* (operator-confirmed 2026-07-23, [[spray_dash_b0_landed]]). Resetting at RPP envelope edges would contradict that. So G2 boundary-sourcing is scoped to **continuous mode only**; dash keeps local arc-length metering (`_rpp_boundary_inputs` returns off for dash). Point uses the handshake (G4), not boundary sourcing. If region-reset is ever wanted it needs a **separate operator A/B** that revisits the locked decision.
- [x] **G2.4** `test_spray_rpp_boundary.py` (12 tests): `_rpp_kind_for` mapping; RPP-sourced lead == local-projection lead on a clean stream (on_early/off_early); RPP overrides a disagreeing local projection; xtrack gate stays independent; node gate + fresh/stale/no-message/disabled/non-continuous fallback.
- [ ] **G2.5** Bench replay (Jetson): same mission, `consume_rpp_progress` OFF vs ON — valve-fire count/positions comparable, no new dither. **Needs in-env run.**

**DoD:** OFF = today byte-for-byte; ON produces equivalent-or-tighter boundary timing on bench; field A/B deferred to G6. Do **not** merge as new default until G6 field-proves ≥ frozen. ✅ Mac: 12 new tests + all 95 spray-node tests green (refactor behavior-preserving; the OFF path reuses the exact pre-G2 lead block). ⏳ **Remaining:** G2.5 bench replay in-env.

---

## G3 — Precise stop at the point (RPP, frozen A/B) — parallel to G2

Purpose: land the 2 cm along-track stop primitive for must-hit points only.

- [x] **G3.1** **Option A feed-forward decel** in pure `precise_stop.py` (`feedforward_trigger_distance` = `max(acceptance, v²/2a)`, `feedforward_brake_speed` = `√(2ad)` capped). `_point_hold_tick` engages the hold at the trigger distance (not the fixed 0.10 m radius) and `_precise_stop_ready` publishes the decel profile toward the coordinate so `v→0` at the point. Decel is `precise_stop_decel_m_s2` (new param, default 0.30); the profile cap is the engagement speed (floored at creep).
- [x] **G3.2** Gated behind `point_precise_stop_enabled` (default False → the frozen brake-when-near path runs byte-for-byte; the `elif not _corner_stop_satisfied()` branch + dwell block untouched). Used **only** for must-hit points under the point-hold overlay; `_corner_stop_satisfied` / corner stops unchanged. Double-gated (also needs `point_hold_enabled`).
- [x] **G3.3** **Option B low-speed servo** (`precise_stop_mode=servo`): after a coarse stop, `_precise_stop_ready` creeps at `precise_stop_creep_speed` (`servo_speed`, along-track sign) until `|residual| ≤ point_arrival_tolerance_m` (0.02). Feed-forward is the coarse phase before the physical stop; servo closes the residual.
- [x] **G3.4** Timeout backstop: after the coarse stop, if the servo can't reach tolerance within `precise_stop_max_s`, accept the best position, warn (rate-limited), dwell — never wedge (`_point_servo_start_ns` timer; reset on release / new point / new mission).
- [x] **G3.5** `test_precise_stop.py` (18 tests): trigger distance over `v`/`a` (v² scaling + floor), decel profile (cap, monotonic, zero-at-point), residual sign convention, servo law + arrival boundary, and a 50 Hz integration that converges to the point **without overshoot**.
- [ ] **G3.6** `test_precise_stop_node.py` written (in-env): freeze (precise OFF == frozen acceptance radius; overlay off = no-op), feed-forward engages beyond the acceptance radius + decelerates, dwell held until at-point **and** stopped, servo creep, `precise_stop_max_s=0` timeout-accept, release-once. **Needs Jetson run** (rclpy). Field: measure along-track rest error.
- [ ] **Prereq (parallel, hard):** nozzle-from-antenna offset measured + applied so the *dot* lands on the point, not the antenna (design §7, open-Q #4). Tracked as its own task — blocks G6 meaning, not G3 code.

**DoD:** OFF = frozen brake-when-near byte-for-byte; ON hits tolerance on bench with feed-forward, servo available as tighter fallback; field accuracy A/B deferred to G6. ✅ Mac: 18 pure tests green; both nodes AST-parse; precise path triple-gated + OFF path structurally the pre-G3 branch. ⏳ **Remaining:** Jetson run of `test_precise_stop_node.py` + frozen regression (smoke + segment_stop + corner_pivot + `test_point_hold_rpp` with precise OFF).

---

## G4 — Point handshake (both nodes) — needs G1 + G3

Purpose: replace the fixed-timer point coordination with `AT_POINT → dwell → point_done → advance` (auto).

- [x] **G4.1** RPP handshake release gated behind **`point_handshake_enabled`** (new param, default OFF). On the confirmed stop the point-hold enters the dwell and holds zero velocity; the `AT_POINT i` milestone is emitted at the dwell edge (G1 `milestones_for`, DWELL_HOLD entry). OFF ⇒ the frozen fixed `point_hold_s` timer runs byte-for-byte (the handshake branch is a pure `if point_handshake_enabled:` around it).
- [x] **G4.2** Spray fires the dwell FSM only on the RPP proof: `PointMeter.update(require_arrival_gate=…)` makes RPP the arrival authority (self-arrival guessing ignored, design §5) — gate True iff `/rpp/milestone AT_POINT i` for the meter's target. On the OFF-confirmed dwell-complete (`PointUpdate.completed_index`), the node publishes `/spray/point_done i` (`seq`, `reason=dwell_complete`).
- [x] **G4.3** RPP auto-advance: `_point_handshake_ready` releases the hold the tick `/spray/point_done i` matches the held point's rank with a fresh seq (seq-after-arm), so the point-hold releases → tracking drives to `i+1`.
- [x] **G4.4** Backstops: RPP advances after `point_hold_max_s` if `point_done` never arrives (warn; in manual it still holds for the operator, not skips). Spray re-syncs from the cached `/rpp/progress.phase ∈ {AT_POINT, DWELL_HOLD}` at the target index when the RELIABLE `AT_POINT` milestone is missed (`_point_handshake_gate` fallback).
- [x] **G4.5** `point_execution_mode` plumbing: `PUT /api/paths/{name}/spray-mode/point` now applies `req.point_execution_mode` (auto|manual) to the RPP via `set_rpp_param_async("point_execution_mode", …)` at load (design §8 option (a)). Behaviour-inert until `point_handshake_enabled`.
- [x] **G4.6** Unit tests: pure `test_point_handshake.py` (gate replaces self-arrival, completed_index once/OFF-confirmed/skip-safe, classifier WAIT_OPERATOR + DWELL_DONE edge). In-env `test_point_handshake_rpp.py` (auto release on point_done, wrong/stale point_done ignored, `point_hold_max_s` backstop) + `test_point_handshake_spray.py` (milestone intake, gate, re-sync, point_done payload/seq).
- [ ] **G4.7** Bench in-env (Jetson): full auto point sequence over a multi-point `/path`; assert milestone/point_done ordering + `/api/spray/status` phase. **Needs in-env run.**

**DoD:** auto point missions run the handshake on bench; with point mode OFF, frozen point behavior unchanged; regression green. ✅ Mac: full pure suite green (326, +10 handshake); both nodes + server AST-parse; the OFF path is structurally the pre-G4 fixed-timer branch (PointMeter gate defaults to None = frozen self-arrival). ⏳ **Remaining:** G4.7 bench run + frozen regression (smoke + segment_stop + corner_pivot + `test_point_hold_rpp` with `point_handshake_enabled` OFF).

---

## G5 — Manual gate (RPP + server + app) — needs G4

Purpose: `WAIT_OPERATOR` + operator-driven advance.

- [x] **G5.1** RPP: `point_execution_mode=manual` → after the spray `point_done` proof (or the phase-1 backstop) the hold enters `WAIT_OPERATOR` (`_point_wait_start_ns`; reported on `/rpp/progress` via classifier phase 10) and releases only on `/point/advance` with `expect_index == held rank` (fresh receive-count guards stale double-taps). `manual_wait_timeout_s` (0 = wait forever) optionally advances.
- [x] **G5.2** Server: `POST /api/spray/point/advance {expect_index}` → `ros_node.publish_point_advance` → `/point/advance {advance:true, expect_index}` (RELIABLE VOLATILE depth 1, never TRANSIENT_LOCAL). In `server/routes/spray.py`.
- [x] **G5.3** Server: `/api/spray/status.point` now carries `rpp_phase_name`, `rpp_point_index`, `wait_operator` — mirrored from the spray node's `/spray/status` (which reads the cached `/rpp/progress`) for button visibility + the `expect_index`.
- [~] **G5.4** Frontend: **backend contract complete & ready** — poll `/api/spray/status`; when `point.wait_operator` show "Next point"; `POST /api/spray/point/advance {expect_index: point.rpp_point_index}`; disable while the POST is in-flight (double-tap guard, also enforced server/RPP-side by `expect_index`). Lives in the separate mobile repo (`Three_Wheel_v2`, branch `plan-editor`) — **not committed here**; integrate there.
- [x] **G5.5** Unit tests: manual sequencer + `expect_index` wrong/stale rejection + wait-forever/timeout in `test_point_handshake_rpp.py` (`test_manual_waits_for_operator_then_advances`, `test_manual_backstop_still_waits_for_operator`, `test_manual_wait_timeout_advances`).
- [ ] **G5.6** Bench/round-trip (Jetson): button → server → `/point/advance` → RPP advances the correct point. **Needs in-env run.**

**DoD:** manual point mode round-trips button→advance on bench; auto mode unaffected; default execution mode stays `auto`. ✅ Mac: manual sequencer green; server routes AST-parse; default `point_execution_mode=auto` + `point_handshake_enabled=OFF` keep the frozen path. ⏳ **Remaining:** G5.4 frontend integration (separate repo) + G5.6 bench round-trip.

---

## G6 — Field validation (operator-run at rover)

Purpose: A/B every flag at the rover; only after this does a flag become the new default / merge to `baseline_master`.

- [ ] **G6.1** Precise stop accuracy: RTK truth vs commanded point, feed-forward vs servo vs frozen brake-when-near.
- [ ] **G6.2** Continuous/dash boundary accuracy: `consume_rpp_progress` ON vs OFF on the same mission — verify ≥ frozen anticipation (no regression of moving-mark lead).
- [ ] **G6.3** Handshake latency + dwell timing on a real point mission (auto).
- [ ] **G6.4** Manual button round-trip in the field.
- [ ] **G6.5** Nozzle offset applied + verified (prereq from G3) so the dot lands on the point.
- [ ] **G6.6** Record bags per [`analyse-missions`](../../.claude/skills/analyse-missions/SKILL.md); measure pose vs **raw `/path`**, never `/rpp/debug[0]`.

**DoD per flag:** field A/B shows ≥ frozen; flag flipped to new default and phase merged to `baseline_master`. Any flag that doesn't beat frozen stays default-OFF.

---

## Cross-cutting invariants (every phase)
- Default-OFF path must be **byte-for-byte** the frozen controller — assert with the frozen regression smoke, `segment_stop`, and `corner_pivot` suites before every merge.
- No `TRANSIENT_LOCAL` on command topics (advance/point_done) — a restart must not replay a stale command.
- Proof **gates**, never **replaces**: spray keeps its `/path` geometry brain and falls back on stale progress. No hard dependency RPP→spray for the valve.
- `/rpp/segment_debug` frozen contract untouched — new state lives on `/rpp/progress`.

## Suggested commit granularity
One phase = one reviewable slice; within a phase, split RPP vs spray vs server/app into separate commits so an A/B can bisect cleanly. Land G0 as its own commit first.
