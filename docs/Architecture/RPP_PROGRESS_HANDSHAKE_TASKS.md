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

- [ ] **G2.1** In `spray_controller_node.py`, subscribe to `/rpp/progress`; behind `consume_rpp_progress`, feed `dist_to_next_boundary_m` + `next_boundary` into the existing lead math (`solenoid_open_delay_s`, `on/off_overspray_margin_m`) — same equations, single source.
- [ ] **G2.2** Staleness fallback: if last progress msg older than `progress_timeout_s`, revert to local `/path` projection **for that tick** (proof augments, never strands the valve). Log fallback transitions (rate-limited).
- [ ] **G2.3** Dash: RPP supplies the MARK-region envelope; `DashMeter` keeps doing the ON/OFF sub-pattern inside it (RPP does not dash-meter). Ensure DashMeter resets at region edges from the envelope.
- [ ] **G2.4** Unit tests: boundary-sourced lead vs local-projection lead agree within tolerance on a clean stream; fallback-on-stale path.
- [ ] **G2.5** Bench replay: same mission, `consume_rpp_progress` OFF vs ON — valve-fire count/positions comparable, no new dither.

**DoD:** OFF = today byte-for-byte; ON produces equivalent-or-tighter boundary timing on bench; field A/B deferred to G6. Do **not** merge as new default until G6 field-proves ≥ frozen.

---

## G3 — Precise stop at the point (RPP, frozen A/B) — parallel to G2

Purpose: land the 2 cm along-track stop primitive for must-hit points only.

- [ ] **G3.1** Implement **Option A feed-forward decel** (design §7): trigger braking at `d = v²/2a` before the point (`a` bounded by `segment_brake_velocity_cap_m_s`) so `v→0` at the coordinate. Reuse the brake primitive with a computed trigger point instead of the fixed 10 cm radius.
- [ ] **G3.2** Gate behind `point_precise_stop_enabled` (default False → today's brake-when-near via `_point_hold_tick`). New primitive is used **only** for must-hit points under point mode; corner stops keep `_corner_stop_satisfied` unchanged.
- [ ] **G3.3** Implement **Option B low-speed servo** as fallback (`precise_stop_mode=servo`): after coarse stop, creep at `precise_stop_creep_speed` with along-track corrections until `|err| ≤ point_arrival_tolerance_m` (0.02), bounded by `precise_stop_max_s`.
- [ ] **G3.4** Timeout backstop: if precise stop can't reach tolerance within `precise_stop_max_s`, rest at best position and warn (do not wedge).
- [ ] **G3.5** Unit tests: feed-forward trigger-distance math over a range of `v`/`a`; servo convergence + timeout.
- [ ] **G3.6** Bench in-env: drive to a placed point, measure along-track rest error, confirm timeout path.
- [ ] **Prereq (parallel, hard):** nozzle-from-antenna offset measured + applied so the *dot* lands on the point, not the antenna (design §7, open-Q #4). Tracked as its own task — blocks G6 meaning, not G3 code.

**DoD:** OFF = frozen brake-when-near byte-for-byte; ON hits tolerance on bench with feed-forward, servo available as tighter fallback; field accuracy A/B deferred to G6.

---

## G4 — Point handshake (both nodes) — needs G1 + G3

Purpose: replace the fixed-timer point coordination with `AT_POINT → dwell → point_done → advance` (auto).

- [ ] **G4.1** RPP: on confirmed precise-stop at point `i`, set phase `AT_POINT`, emit milestone `AT_POINT i`, enter `DWELL_HOLD` (hold zero velocity).
- [ ] **G4.2** Spray: fire dwell FSM **only** on `/rpp/milestone AT_POINT i` (settle ≈ 0 since RPP guarantees the stop); on confirmed actuator OFF, publish `/spray/point_done i` with `seq` + `reason=dwell_complete`.
- [ ] **G4.3** RPP auto-advance: on `/spray/point_done i`, transition `TRANSIT → i+1`.
- [ ] **G4.4** Backstops: RPP advances after `point_hold_max_s` if `point_done` never arrives (warn); spray re-syncs from `/rpp/progress.phase==AT_POINT` on a `seq` gap (missed milestone).
- [ ] **G4.5** `point_execution_mode` plumbing (auto path): carry through `PathPlanRequest → staged spray_session → RPP param at load` (design §8 option (a), server sets it on load).
- [ ] **G4.6** Unit tests: handshake sequencer (auto), `seq`-gap re-sync, `point_hold_max_s` backstop.
- [ ] **G4.7** Bench in-env: full auto point sequence over a multi-point `/path`; assert milestone/point_done ordering + `/api/spray/status` phase.

**DoD:** auto point missions run the handshake on bench; with point mode OFF, frozen point behavior unchanged; regression green.

---

## G5 — Manual gate (RPP + server + app) — needs G4

Purpose: `WAIT_OPERATOR` + operator-driven advance.

- [ ] **G5.1** RPP: when `point_execution_mode=manual`, after `point_done` enter `WAIT_OPERATOR`; advance only on `/point/advance` with matching `expect_index`. Optional `manual_wait_timeout_s` (0 = wait forever).
- [ ] **G5.2** Server: `POST /api/spray/point/advance` → publishes `/point/advance {advance, expect_index}`. Wire in `server/routes/path.py` where `point_execution_mode` currently dead-ends.
- [ ] **G5.3** Server: surface `phase == WAIT_OPERATOR` + current `point_index` in `/api/spray/status` (mode/point block already exists @ `0cd284a`) for button visibility.
- [ ] **G5.4** Frontend: show "Next point" button while `/api/spray/status.phase == WAIT_OPERATOR`; POST advance with `expect_index`; guard double-taps.
- [ ] **G5.5** Unit tests: manual sequencer, `expect_index` rejects wrong/stale point, wait-forever vs timeout.
- [ ] **G5.6** Bench/round-trip: button → server → `/point/advance` → RPP advances the correct point.

**DoD:** manual point mode round-trips button→advance on bench; auto mode unaffected; default execution mode stays `auto`.

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
