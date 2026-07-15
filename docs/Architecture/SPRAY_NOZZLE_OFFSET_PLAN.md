# Nozzle-Offset-From-Antenna — Implementation Plan

Status: PLANNING — not started. Companion doc to
`docs/Architecture/SPRAY_CONTROLLER_V2_PLAN.md`, but this is a **path_engine
/ geometry problem, not a spray-controller problem** — different subsystem,
different owner code, kept as its own plan deliberately.
Rev 2 (2026-07-15): robustness pass — committed the offset-before-staging
ordering (resolved a §3.1/§3.4 contradiction), pinned the sign convention
to reuse `_nozzle_position_ned` instead of re-deriving it, documented the
corner-smoothing exactness limit + moved the §4 property test to the
post-smoothing path so it actually catches it, worked through the
no-double-application guarantee, and added degenerate short-segment tests.
Recommendation (Solution A, don't touch RPP) unchanged.
Rev 3 (2026-07-15): verification-pass fixes + parameter lifecycle —
corrected §3.4's broken "spray controller unchanged" claim (the nozzle
xtrack gate breaks against the offset path — §3.6), corrected where corner
smoothing actually runs (inside the frozen RPP node, not path_engine —
§3.3/§4), stated the direction-reversal invariant explicitly (§3.1),
added the point-mode cross-plan gap (§5), added a miter-limit note
(§3.3), and added §3.7: the offset params are fingerprinted plan-time
hardware constants with a paint-line calibration procedure, not runtime
tuning knobs. Recommendation (Solution A) unchanged.

## 1. The problem, precisely

The rover's GPS antenna (the point PX4/EKF actually reports as `local_position`)
is what the RPP tracking controller steers along the target line. The spray
nozzle is mounted at a fixed rigid offset from that antenna — some
`nozzle_forward_offset_m` (along the rover's nose) and `nozzle_lateral_offset_m`
(to the rover's left/right). Today's spray controller already computes where
the nozzle *physically is* (`_nozzle_position_ned` in
`src/spray_controller_node.py`) — but **only to decide spray on/off timing**,
never to correct where the rover drives. The RPP controller has zero
knowledge that a nozzle exists; it drives the antenna exactly onto the
surveyed/DXF geometry.

Consequence: if the nozzle has any lateral offset, the antenna follows the
drawn line perfectly, but the **paint lands parallel-shifted** by the
lateral offset distance — every line comes out correct in shape but wrong
in position by a constant sideways amount. This gets worse at corners,
because the offset is defined in the rover's *body frame* (it rotates with
heading), not the world frame — so the required sideways correction on the
antenna's path changes direction every time the rover turns, it isn't a
single constant shift applied once to the whole mission.

## 2. Two candidate solutions, and which one this plan recommends

### Solution A — Planner-side geometry offset (RECOMMENDED)

Before a mission's path ever reaches the RPP controller, shift the target
geometry itself so that when the **antenna** follows the shifted path, the
**nozzle** (rigidly offset from the antenna) ends up exactly on the
*original* surveyed/DXF geometry. This is a one-time, static coordinate
transform done in `path_engine`, at plan-generation time. The RPP
controller never finds out a nozzle exists — it just tracks a different
line than the one drawn, and gets the physics right by construction.

### Solution B — Path-tracking-side offset compensation

Modify the live RPP control loop so its cross-track/heading error is
computed against the *nozzle's* projected position (already available —
this project computes exactly that today, just for spray timing) instead
of the antenna's. The controller would then be steering to minimize
nozzle error directly, in real time.

### Why A, not B

1. **A hard project rule blocks B outright.** `CLAUDE.md`, current status
   section: *"Controller + tuning CLOSED at this baseline... Do not
   re-open arc PID/lookahead unless a regression appears."* Solution B is
   exactly that — it reaches into the live tracking-error computation of a
   controller that is explicitly frozen because it took months of
   validated tuning (`cd44884` collinear-momentum baseline, BUG-T1/T2/T3
   lineage, sub-2cm RMS shape tracking) to get right. Reopening it for an
   unrelated feature risks the thing this project has worked hardest to
   protect.
2. **They are mathematically the same fix, applied in different places —
   so there's no accuracy trade-off, only a risk trade-off.** The rover
   only has one thing it can actually steer: the antenna's position/heading.
   Whether you compute "where should the antenna be so the nozzle lands
   here" once, offline, in the planner (A), or every control tick, live,
   inside the tracking loop (B), you are solving the identical geometric
   equation. A does it once, in a stateless, unit-testable function, with
   its output fully visible before the rover ever moves. B does it
   continuously, inside code that already has closed/frozen tuning
   depending on its exact current error signal.
3. **The rover's mount is rigid, which is the easy case industrially.**
   Real GPS-guidance systems distinguish exactly this: implements
   *rigidly* bolted to the vehicle are controlled as "a rigidly-connected
   location on the vehicle" via a static configured offset entered once at
   setup; only *towed/articulated* implements (which trail, lag, and swing
   independently of the tractor, especially in turns) need a live
   kinematic hitch-tracking system to correct for that independent motion
   ([USPTO — Farm implement guidance method and apparatus](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7054731);
   [USPTO — Adjustment of vehicle-implement trajectories to compensate for lateral implement offset](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/6804587);
   [ResearchGate — Implement lateral position accuracy under RTK-GPS tractor guidance](https://researchgate.net/publication/229351588_Implement_lateral_position_accuracy_under_RTK-GPS_tractor_guidance)).
   Our nozzle has no hitch, no articulation, no independent degree of
   freedom — it moves exactly as a fixed point on the chassis moves. That
   is precisely the case industry solves with a static offset, i.e.
   Solution A, not the dynamic case that would justify B.
4. **This is also just where the work already belongs.** `path_engine` /
   trajectory planning is explicitly listed as this project's active-focus
   area #1 (`CLAUDE.md`); the tracking controller is explicitly listed as
   closed. Solution A fits the codebase's own stated boundaries; B fights
   them.

**Verdict: build Solution A. Do not touch the RPP controller for this.**

## 3. Solution A — detailed design

### 3.1 Where it runs

New pipeline step in `path_engine`, applied to the planned geometry
**after** DXF parsing / entity ordering / TSP but **before** the existing
corner-smoothing and per-line PRE/MARK/AFT extension staging that RPP
already consumes today. This ordering matters: everything downstream of
this step (corner smoothing, `segment_corner_threshold_deg` pivot logic,
per-line extension densification) is proven and field-validated — it
should keep operating exactly as it does today, just on already-offset
input coordinates, so none of that machinery needs to know an offset
happened.

**Direction-reversal invariant (hard rule, was only implied before):** a
lateral offset flips sides when travel direction reverses — any step that
reverses a segment's traversal *after* the offset is applied paints at
**2× the offset on the wrong side**. The offset must therefore run after
**every** step that can change traversal direction: entity ordering, TSP,
**or-opt, duplicate-geometry dedupe, and retrace handling** (`7c4b03e` —
these run in this branch's planner and were not named in earlier revs),
and closed-loop orientation choice. Downstream of the offset step, no
stage may reverse a segment — this is asserted, not assumed (a cheap
plan-time check: per-segment travel bearing recorded at offset time must
match the staged output's bearing). Ordering vs spray latency
compensation (`apply_spray_latency_compensation`): offset (lateral) runs
**before** latency compensation (longitudinal boundary shift); the two
commute on straights but not exactly at corners, so the order is pinned
here rather than left to chance — latency comp operates on the
already-offset antenna arc-length, matching how the spray node will
measure it at runtime.

**Ordering committed (resolves an earlier ambiguity in this doc):** the
offset is applied to the **raw mark geometry, before extension staging** —
NOT to the finished PRE/MARK/AFT set. Offsetting one clean mark line
(a single perpendicular per segment) is simpler and less error-prone than
offsetting an already-staged PRE/MARK/AFT/connector chain, and it lets the
existing extension planner build PRE/MARK/AFT on top of the offset mark
exactly as it does today. §3.4's transit-routing then follows for free
because the extension planner emits its PRE waypoint from the (already
offset) mark start. Earlier text implying the transform is applied "to the
PRE/MARK/AFT set as a whole" is superseded by this.

### 3.2 The core transform

For a straight segment from point `A` to point `B` (unit direction
`d = (B - A) / |B - A|`, right-hand perpendicular `n = rotate90(d)`), the
antenna's target point is:

```
antenna_target = original_point - forward_offset_m * d - lateral_offset_m * n
```

(sign convention: chosen so that when the antenna sits at `antenna_target`
with heading `d`, the nozzle — offset `+forward` along `d` and `+lateral`
along `n` from the antenna, matching the existing `_nozzle_position_ned`
body-frame convention already in `spray_controller_node.py` — lands
exactly on `original_point`.)

**Do NOT hand-re-derive the signs — reuse the code.** `_nozzle_position_ned`
already encodes the exact body-frame convention (`lateral` positive to
rover-right: `nozzle = pose + forward·[cos,sin]ψ + lateral·[-sin,cos]ψ`).
A single sign flip here paints every line at **2× the offset on the wrong
side**. The offset transform must therefore be defined as the algebraic
**inverse of `_nozzle_position_ned`** and, wherever practical, literally
import and call that function (or a shared helper factored out of it) so
there is exactly one place the sign convention lives — the same
single-source-of-truth principle §3.5 applies to the offset *values*,
applied here to the offset *math*. The heading `ψ` used per segment is the
segment's own travel bearing (`atan2(d_e, d_n)` in NED).

For the common case on this rover (nozzle mounted at the chassis
centerline, offset only sideways — `forward_offset_m ≈ 0`), this reduces
to the simple, well-understood case from ag guidance: **a parallel offset
line**, shifted perpendicular to travel direction by `lateral_offset_m`.
This is the majority case and should be validated first.

### 3.3 Corner handling

Offsetting a *polyline* (not a single straight line) by a constant
perpendicular distance is the same problem CNC/laser-cutting systems solve
as "cutter/tool-radius compensation" (G41/G42) and vector graphics engines
solve as "stroke offsetting" — a well-known, well-documented class of
algorithm, not something being invented here:

- **Outer corners** (the offset is on the outside of the turn): the two
  adjacent offset segments pull apart at the vertex and need a join — a
  simple **miter join** (extend both offset lines to their intersection
  point) is sufficient at the corner angles this project already handles
  (the rover's own `segment_corner_threshold_deg`/`corner_smooth_radius_m`
  machinery downstream will further smooth whatever vertex this step
  produces, so the join here doesn't need to be sophisticated).
  **Miter limit:** the miter point extends `offset / sin(θ/2)` from the
  vertex — unbounded as the corner sharpens (≈5.7× the offset at 20°).
  Standard practice applies: if miter length exceeds
  `miter_limit × offset` (default 4.0, the SVG/PostScript convention),
  fall back to a **bevel** (connect the two offset endpoints directly).
  This makes the §4 acute-corner unit test well-defined instead of
  implementation-dependent.
- **Inner corners** (the offset is on the inside of the turn): the two
  adjacent offset segments can cross/overlap before reaching the vertex.
  This must be detected (segment-segment intersection test) and trimmed —
  standard "corner trimming" in cutter-compensation terms. Left untrimmed,
  this would command the antenna through a small self-intersecting loop
  right at the corner.
- Both cases are local (only affect the geometry within roughly
  `lateral_offset_m` of a vertex) — this is a small, bounded, testable
  piece of computational geometry, not a rewrite of path generation.

**Exactness caveat — corner smoothing degrades the offset (know this
before trusting corners):** the offset is computed on the *pre-smoothed*
polyline, where each segment has a constant heading, so a constant
perpendicular offset is exactly correct. But downstream corner smoothing
rounds every corner into an arc, and through that arc the rover's heading
rotates *continuously* — so the body-frame nozzle **sweeps** and the
constant perpendicular offset is exact only on the straight portions, with
a residual error through the smoothed corner proportional to
`lateral_offset_m × corner_curvature`. For a small lateral offset this is
sub-cm and negligible; for a large offset through a tight corner it is not.
This is *why* the property test in §4 must be evaluated on the FINAL,
post-smoothing antenna path (see §4), not on the offset function's raw
output — otherwise the test validates a stage that the smoother later
perturbs. If a specific rover's offset is large enough that measured corner
error exceeds tolerance, the fix is a corner-local correction, not
re-opening the smoother; but for the expected small offsets this is a
documented known-limit, not a blocker.

**Where smoothing actually runs (corrected in Rev 3 — earlier revs
implied a path_engine stage):** production corner smoothing is
`_smooth_corners` **inside the frozen RPP controller node**
(`src/rpp_controller_node.py`, param `corner_smooth_radius_m`, baseline
default 0.5). A planner-side `corner_smooth_radius_m` also exists
(`server/models.py`) but **defaults to 0.0 (disabled)** — production
paths are smoothed RPP-side, after `/path` is published. Consequence for
§4: the post-smoothing property test cannot import the RPP node's method
directly (it lives on a `Node` subclass in an rclpy-importing file — not
importable in the Mac dev env, and the controller file is CLOSED). The
test therefore uses a **read-only reimplementation of the same inscribed-
arc rule** (radius + arc-points params copied from the frozen defaults),
validated once against a recorded `/path`→smoothed-path pair from a bench
run so the reimplementation is proven equivalent before it's trusted.
The controller file itself is not touched, honoring the freeze.

### 3.4 What gets offset vs what doesn't

Only geometry that will actually be **sprayed** needs nozzle-accurate
placement — transit-only connector segments don't put paint anywhere, so
they don't need the transform applied to their interior. What they *do*
need is to end at the correct **offset entry point** of the next marked
run (the per-line PRE-extension waypoint, in this project's existing
per-line PRE/MARK/AFT terminology) — i.e., the transit path routes the
antenna to wherever the antenna needs to be for the nozzle to start
exactly on the mark's true start point, not to the mark's true start point
itself. This falls out naturally because the offset is applied to the raw mark
before staging (§3.1), so the extension planner builds its PRE/MARK/AFT
on the already-offset mark and transit segments are simply routed to the
(now-offset) PRE waypoint like today.

**No double-application (worked through — the one subtle risk):** after
this lands, the line fed to RPP is the *antenna* path (offset). The spray
controller *still* projects the **nozzle** position onto that path for its
on/off timing. The **arc-length/timing part** composes correctly and does
NOT double-count: the nozzle passes a true mark boundary exactly when its
projection passes the corresponding offset-path boundary, so spray fires
at the true mark edges. The requirement this imposes: the offset must be
applied to the geometry **exactly once, in path_engine**, and the spray
controller must keep using `nozzle_*_offset_m` **only** for timing
projection, never re-shift the geometry a second time. Stated here so an
implementer doesn't "helpfully" offset again in the controller.
**However — "spray controller unchanged" is NOT fully true; the xtrack
gate breaks. See §3.6, which is a mandatory part of this plan.**

### 3.5 New parameters

- `nozzle_forward_offset_m`, `nozzle_lateral_offset_m` — already exist as
  spray-controller params (used today only for spray timing); this plan
  reads the **same two values** in `path_engine` so there is exactly one
  place the physical offset is measured and entered, not two configs that
  can drift (same "single source of truth" principle as the V2 controller
  plan). Needs a real bench measurement of the physical mount before
  either value is anything but a placeholder. Lifecycle — how a change to
  these values is managed without desyncing anything — is §3.7.
- `miter_limit` (default 4.0) — §3.3 bevel fallback threshold. Geometry
  constant, not a tuning knob.
- No other new params — this is a geometry function, not a tunable
  control loop.

### 3.6 The spray-node xtrack gate MUST be fixed with this plan (found in
verification — §3.4's earlier "controller unchanged" claim was wrong)

Both RPP **and the spray node** consume the same `/path` topic
(`rpp_controller_node.py` and `spray_controller_node.py` both subscribe
`"/path"`). After Solution A, `/path` carries the *offset antenna* path —
but `_make_spray_decision` forces `safety_ok=False` when the **nozzle's**
projection distance exceeds `max_xtrack_error_m` (default **0.10 m**).
With the offset live, a *perfectly tracking* rover has its nozzle sitting
on the ORIGINAL geometry — i.e. exactly `|lateral_offset_m|` away from
`/path` — so any lateral offset ≥ 10 cm **permanently blocks spray** with
a bogus "xtrack error" reason, and smaller offsets silently eat the error
budget. (A Swozi/TR10-style side mount is 20–40 cm: for that class of
rover this gate as-is means zero paint, ever.)

Fix (smallest correct change, stays inside the spray node): the xtrack
gate compares the **expected nozzle-to-path distance** instead of raw
distance — gate on `abs(projection.xtrack_error_m − |lateral_offset_m|) >
max_xtrack_error_m` when forward offset is negligible, or in general on
the distance between the *actual* nozzle position and the *expected*
nozzle position (antenna's path foot-point pushed through
`_nozzle_position_ned` at the segment heading). Arc-length boundary
timing keeps using the nozzle projection exactly as today (§3.4 —
that part composes correctly). No new topic, no second geometry
publication, no RPP change. A unit test in §4 covers: offset = 0 →
behavior identical to today; offset = 0.3 m + perfect tracking → gate
open; offset = 0.3 m + genuine 0.15 m tracking error → gate closed.

### 3.7 Parameter lifecycle — offsets are fingerprinted plan-time
hardware constants, NOT runtime tuning knobs

Today both offset params are live-settable at any moment via the server
spray-params API (`server/routes/spray_params.py`) and, like all ROS
params, reset on a `rpp-pipeline` restart unless persisted. Once the
offset shapes *planned geometry*, a live change after planning silently
desyncs three things at once: the staged path, the spray timing, and the
physical truth. Rules:

1. **Measured hardware geometry, one persistent home.** Both values live
   in one persisted config on the Jetson (`rover_geometry` section,
   delivered by the same persisted-config mechanism the project already
   uses for service drop-ins — not a POST that evaporates on restart).
   path_engine and the spray node both read this one source. It changes
   only when hardware physically moves.
2. **Baked into the plan fingerprint.** path_engine records the offset
   pair used into the plan metadata and includes it in the existing
   path-fingerprint hash. At mission start the server compares live
   config vs the plan's baked values — mismatch → 409 "plan was generated
   with a different nozzle offset, re-plan required." A param change can
   therefore never half-apply: it either forces a replan or does nothing.
3. **Validated at ingest.** Sanity bounds (|offset| ≤ 1.0 m, finite)
   rejected at config load — a typo cannot plan a path meters off survey.
4. **Mount changes are named events.** If the rover genuinely swaps
   between center-mount and side-mount, keep two measured presets in the
   config and select one; each selection still flows through rule 2. No
   UI slider — it's a wrench-and-tape-measure change.
5. **Calibrate by painting, not only by tape measure.** Drive one
   straight surveyed line with spray on; the measured lateral
   displacement of the paint from the surveyed line IS the residual
   `nozzle_lateral_offset_m` error (sign = side), and paint start/stop
   overshoot feeds the forward value. This captures antenna phase-center
   error a ruler can't. Re-run after any mount change (extends the §6
   Phase 1 measurement step — tape measure first, paint line confirms).

Zero-offset degeneracy (the current rover: nozzle mounted directly under
the antenna): both params 0.0 → the transform is the identity, the
antenna path IS the DXF geometry, and this entire plan is a no-op by
construction — which is exactly why landing it behind offsets-default-0
cannot regress the current setup.

## 4. Testing plan

- **Unit tests, synthetic geometry:** straight line (parallel-offset
  correctness), right-angle corner with offset on the outside of the turn,
  right-angle corner with offset on the inside of the turn (trim
  correctness — the case most likely to be buggy), acute corner, obtuse
  corner, a closed shape (square/circle) to confirm the offset loop closes
  without a seam gap, and a **short segment relative to the offset**
  (`lateral_offset_m` comparable to or larger than a segment length, and
  the same interacting with densification order) to exercise the
  degenerate inner-corner trim where an offset segment can be fully
  consumed.
- **Property test (evaluated on the POST-SMOOTHING antenna path, not the
  raw offset output):** for any input polyline + offset, re-deriving the
  nozzle's position from the transformed antenna path — using
  `_nozzle_position_ned` itself as the forward transform (the same function
  the offset inverts, §3.2), at the heading each antenna segment actually
  has after `smooth_corners` has run — must reproduce the original geometry
  within a small numerical tolerance. Running this on the *final* geometry
  is what makes it catch the corner-smoothing residual error flagged in
  §3.3; a version run on the pre-smooth offset output would pass while the
  real painted corner is off. This is the actual correctness criterion, not
  "does the offset line look right."
- **Rev 3 additions:** the §3.6 xtrack-gate tests (offset 0 → identical to
  today; offset 0.3 m + perfect tracking → gate open; offset 0.3 m + real
  0.15 m error → gate closed); a direction-reversal assertion test (§3.1
  invariant — recorded per-segment bearing at offset time matches staged
  output, and a deliberately reversed segment is caught); a §3.7
  fingerprint test (plan staged with offset A, config changed to offset B,
  mission start → 409); miter-limit fallback test (acute corner beyond
  `miter_limit` produces a bevel, not a spike); and a one-time equivalence
  check of the §3.3 smoothing reimplementation against a recorded
  bench `/path`→smoothed pair.
- **Bench validation:** run a synthetic mission through the full
  `path_engine → RPP` pipeline in sim/bench and confirm the *antenna's*
  reported path differs from the DXF geometry by exactly the expected
  offset pattern (including at corners) before ever testing on paint.
- **Field validation:** paint a known test shape, physically measure the
  painted line's position against the surveyed target (this project
  already tracks sub-2cm RMS on shape tracking — the offset-corrected
  paint should land within the same tolerance band of true geometry, not
  just be internally self-consistent).

## 5. Interplay with existing systems (read before starting)

- **GPS-surveyed placement/alignment** (`gps_alignment_placement_bug`,
  `epic12_placement_landed` in project memory) computes where the mission
  origin sits relative to survey points — that's an antenna-referenced
  alignment today and should stay antenna-referenced; this offset plan
  operates entirely inside path_engine's *mission-local* geometry, after
  placement, so the two systems don't need to know about each other.
- **Per-line extension staging** (`per_line_extensions_v2`) — this plan
  explicitly reuses that existing PRE/MARK/AFT structure as the unit the
  offset transform is applied to, rather than introducing a new geometry
  concept.
- **Corner-smoothing / stop-pivot controller** (`cd44884` baseline,
  CLOSED) — explicitly not modified; consumes offset geometry unchanged,
  exactly as it consumes today's un-offset geometry.
- **Point/Coordinate mode** (`SPRAY_CONTROLLER_V2_PLAN.md` §7.3) —
  **cross-plan gap, owned HERE as of Rev 3:** point mode paints wherever
  the *nozzle* is during a dwell, and with `heading_tolerance_deg: null`
  (position-only arrival, legal in the V2 plan) the heading at the dot is
  unconstrained — so with a non-zero offset the paint lands *anywhere on a
  circle of radius |offset|* around the target. Rule: when the configured
  offset is non-zero, point-mode planning must either (a) offset the hold
  target by the offset vector rotated to the **planned approach heading**
  and require heading-gated arrival at that heading, or (b) refuse
  `heading_tolerance_deg: null` at config validation with a clear error.
  (a) is the real fix; (b) is the guard until (a) lands. V2 plan Phase D
  must not ship point mode with non-zero offset without one of the two.
- **Speed-proportional flow rate** (`SPRAY_CONTROLLER_V2_PLAN.md` §7.5) —
  once this offset plan lands, the flow-rate bench calibration in that
  plan's Phase E should be re-run against real (offset-corrected) mark
  geometry rather than a synthetic bench line, since the corrected antenna
  path changes speed profile through corners slightly. Already noted as a
  dependency in that plan's rollout.

## 6. Rollout

1. **Phase 1 — measure.** Physically measure `nozzle_forward_offset_m` /
   `nozzle_lateral_offset_m` on the real rover. Everything below is
   meaningless without accurate numbers here.
2. **Phase 2 — offset-curve function, unit-tested only.** Build the
   transform + corner trim/join as a pure, ROS-free function in
   `path_engine`, validated entirely by the synthetic geometry tests in
   §4 — no hardware involved yet.
3. **Phase 3 — bench/sim integration.** Wire it into the real
   `path_engine → RPP` pipeline behind a config flag (default off), run
   the bench validation in §4, confirm downstream corner-smoothing still
   behaves identically on offset input. **Lands together with the §3.6
   spray-node xtrack-gate fix and the §3.7 persisted-config +
   plan-fingerprint lifecycle** — the offset must never be enableable
   without both (a >10 cm offset with today's gate sprays nothing, and an
   unfingerprinted offset can silently desync staged plans).
4. **Phase 4 — field validation.** Dry-run first (per this project's
   standing rule for anything touching mark geometry), then a real paint
   test on a known shape, measuring actual paint position against survey.
5. **Phase 5 — flip default on**, once field-validated, and note the
   dependency for `SPRAY_CONTROLLER_V2_PLAN.md` Phase E (flow-rate
   calibration should happen after this, not before).

## 7. Open question for the operator

The exact physical offset (§3.5) hasn't been measured yet — this plan is
correct regardless of the number, but no phase past Phase 1 can start
until it's known. If the nozzle is mounted at (or very close to) the
antenna's position today, this entire plan is low-priority; if it's
mounted meaningfully off to one side, it's the reason painted lines have
likely been landing slightly off-target this whole time and should be
prioritized ahead of the flow-rate work in the companion plan.
