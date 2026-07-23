# Spray Controller Architecture V2 — Implementation Plan

Status (2026-07-23): Phase A **LANDED + deployed** (FSM, typed telemetry,
config schema for all 3 modes, `rpp_start.sh` isolation). Phases B0/B/C/D/E/F
**not started** — see `SPRAY_V2_PHASES_BF_IMPLEMENTATION_PLAN.md` for the
build order. Branch: `Upgrade_Spray`. This doc is the source design; the
B–F doc is the sequenced work plan that consumes it.
Rev 2 (2026-07-15): robustness pass — filled the previously-dangling RTK
gate spec (§7.6), removed the unstated RPP-substate dependency in dash
mode + added a monotonic-`s` guard (§7.2), added a point-mode
unreachable-target watchdog (§7.3), and pinned flow-rate saturation /
slew-init semantics (§7.5). No architectural decisions changed.
Rev 3 (2026-07-15): verification-pass fixes — resolved the point-mode ↔
min-speed-gate contradiction (§5), specified the mission-side point-mode
contract + fixed the watchdog-skip desync (§7.3), added the missing
`DISABLED` exit transition (§4), added a recovery rule to the
monotonic-`s` jump rejection (§7.2), defined the dash-start "on path
proper" criterion (§7.2), pinned flow-update ack semantics (§7.5), and
specified mission-clear config semantics (§3). No architectural
decisions changed.
Rev 4 (2026-07-23): **code-reconciliation pass — the only Rev to change a
design decision.** Rev 3 predates `6523a84` (2026-07-18, "speed no longer
gates on/off; pivot state does"). Three sections described a rover that no
longer exists and are corrected here, so Phase C is not written against a
stale contract (the exact way `16480d9` became a false fix):
  • §5 — the min-speed floor is **no longer a gate** (slow → thin, handled
    by §7.5 flow control, never off). The real mode-relevant gate is now
    the discrete **pivot-state gate** (`/rpp/segment_debug[1]==CORNER_ALIGN`).
    The Rev 3 "swap the speed gate per mode" instruction was a no-op and is
    removed; the gate table is rewritten around the pivot state.
  • §7.2 — dash corner-deferral is re-specified against the **pivot state**,
    not a velocity threshold. Rev 3's claim that "the node does not subscribe
    to RPP state and V2 keeps it that way" is factually wrong against shipped
    code (`_segment_debug_cb`) — and the discrete state is the *better*
    substrate (a state can't dither at the 0.08/0.08/0.03 corner-crawl speeds
    the way a speed threshold does; that was the whole point of `6523a84`).
  • §7.3 — point-mode dwell sprays at a standstill, which the pivot gate now
    blocks. A scoped, **node-side** gate exemption is specified (frozen RPP
    untouched). Rev 3 assumed no such exemption was needed.
  Also records one **open road-marking decision** (per-segment dash pattern)
  in §7.2 rather than silently locking continuous-across-corners metering.
  No other architectural decision changed; §§1–4, §6, §7.1, §7.4–7.6, 8–13
  stand as Rev 3.
Author context: written from scratch against this branch's current controller
(`src/spray_controller_node.py`, 1051 lines, Mode-1-only). `main`'s spray V2
(`35a1518` Task 17/18/19 + `a62cab2` three-mode work) is used as a **feature
reference only** — its code is not reused. Several of its defects are
documented below as things this design structurally avoids, not bugs to
port and re-fix.

Decisions locked in for this plan (confirmed by operator 2026-07-15):
1. **Node-owned single source of truth.** No parallel server-side copy of
   spray state/config that can drift from the node. Server sends a
   versioned config blob at mission load and reads telemetry back; it does
   not maintain its own spray FSM.
2. **Three modes:** Continuous, Interval/Dash (arc-length metered, crosses
   corners), Point/Coordinate (dwell-based). Dash cycle does **not** reset
   at entity/corner boundaries — metering is continuous mission arc-length.
3. **Safety scope:** add a real GPS/RTK fix-quality gate now (hardware
   already exists — see `src/rpp_controller_node.py` P0.3). Obstacle-sensing
   integration is a documented future hook only; no sensor exists yet, so
   no obstacle logic is built or exposed as live params in this pass.
4. **Speed-proportional flow rate** (added 2026-07-15): the sprayer no
   longer commands a fixed full-flow value whenever geometry says "on" —
   commanded flow scales with rover ground speed so paint-per-meter of
   line stays constant instead of gapping at speed and pooling when slow.
   Design in §7.5, grounded in how industrial ag PWM rate-controllers
   solve the identical problem (see sources at the end of §7.5).

---

## 1. Why not reuse `main`'s implementation

Concrete, sourced defects found in `main`'s history that this design avoids
structurally, not by patching:

| # | Defect on `main` | Root cause | How V2 avoids it |
|---|---|---|---|
| 1 | Continuous-mode crash loop → 11s OFFBOARD blackouts → 1.5m corner drift (`533d83c` regression, memory: `spray_continuous_pipeline_crash`) | `distance_to_boundary_m=inf` serialized via `json.dumps(allow_nan=False)`; `rpp_start.sh` counted spray failures toward the **global** node-failure threshold, so a spray crash restarted the whole pipeline | §6 Telemetry contract sanitizes non-finite values at the boundary by construction (typed status model, not raw floats); §9 makes spray-node failure isolation an explicit infra requirement, verified independent of node code quality |
| 2 | Undeclared-param 409s (`e3aae88`, memory: `spray_param_contract_and_degraded_load`) | Serializer (`spray_config.py`) emitted 57 keys; node declared 42; two independently-maintained lists drifted | §3 Config model: one versioned schema, one file, node parses it directly — no separate serializer/declaration lists to keep in sync |
| 3 | Degraded-load fingerprint 409 (same commit) | Fingerprint computed over "real" flags but passed through even when flags were zeroed for a degraded load | §3: node recomputes its own path fingerprint locally from whatever geometry it actually received; never trusts a caller-supplied fingerprint |
| 4 | 15 GPS/obstacle params declared but never read — "safety gap" the previous author explicitly flagged (memory: `spray_param_contract_and_degraded_load`) | Params added for forward-compat, gating never implemented | §7 GPS gate is implemented, not just declared. Obstacle params are **not added at all** until there's a sensor to back them — no inert safety-looking params |
| 5 | CPU-spin executor bug (`83c9bb7` revert) | Multi-threaded executor on a node with no genuine parallel I/O need | §10: single-threaded executor by default, called out explicitly in the design so it isn't "fixed forward" into a regression again |
| 6 | Manual override could read as optimistic ON before Task 17/18/19 hardened it | State tracked as scattered booleans (`_commanded`, `accepted_on`, pending flags) across a 2000-line file, easy to leave a path unswept | §4 formalizes the actuator FSM as an explicit `enum.Enum` + transition table, unit-testable in isolation from ROS |

---

## 2. Scope

**In scope:** actuator command state machine, all safety gates (existing +
new RTK gate), three controller modes, speed-proportional flow rate,
config load path, telemetry contract, startup reconciliation, infra
isolation fix, test plan, rollout phases.

**Out of scope (explicitly deferred, not silently dropped):**
- Obstacle-avoidance integration — no sensor exists; revisit when one is
  specified. §7.4 documents the hook point so it's a clean add later.
- Physical spray-flow feedback (flow sensor / current sense) — today's
  actuator is command-truthful (trusts MAVROS ack), not physically
  verified. Stays that way; `physical_feedback_supported=False` remains
  accurate. Flag as a hardware backlog item, not a V2 deliverable.
- Server-side mission-bound spray-mode persistence UI/endpoints beyond the
  minimal load-time blob — can be layered on top later without touching
  the node's FSM.

---

## 3. Config model — single source of truth

Today's node already takes ~30 individually-declared ROS params for
Mode-1 behavior. V2 keeps per-actuator/backend params as ROS params
(rarely change, bench-tunable) but introduces **one versioned mission
config message** for everything that varies per mission (mode, boundaries,
dash distances, point list):

```
SpraySessionConfig:
  schema_version: int          # bump on any breaking field change
  mode: "continuous" | "dash" | "point"
  points: [(n: float, e: float), ...]   # NED, only for continuous/dash geometry
  flags:  [bool, ...]                    # per-point mark/transit, continuous/dash
  dash:                                   # present only if mode == "dash"
    on_distance_m: float
    off_distance_m: float
    start_state: "on" | "off"
  points_mode:                            # present only if mode == "point"
    coordinates: [(n: float, e: float), ...]
    arrival_tolerance_m: float
    heading_tolerance_deg: float | null    # null = position-only arrival
    arrival_settle_s: float
    dwell_s: float
```

Delivery: published once per mission load on a **RELIABLE, TRANSIENT_LOCAL**
topic `/spray/session_config` (same durability class as `/path` today), so
a late-joining/restarted spray node picks up the current mission
automatically — mirrors the existing `_path_qos()` pattern already in the
codebase.

Rules that eliminate defects #2/#3 above by construction:
- The node is the **only** parser of this schema. There is no second
  "declare these params on the node" step — nothing to drift.
- The node computes its own path fingerprint (hash of `points`+`flags`)
  the moment it receives a config, purely for its own change-detection
  (has the mission actually changed since the last tick). It never
  receives, trusts, or is asked to validate a caller-supplied fingerprint.
- Degraded loads (spray unavailable, flags zeroed) are just "mode=continuous
  with all-False flags" — a config the node accepts and behaves on exactly
  as it would any other all-transit mission. No separate degraded code path
  to keep consistent with the healthy one.
- `schema_version` mismatch → node logs and ignores the config, staying on
  its last-known-good one (fail static, not fail open).
- **Mission clear must publish, not just stop publishing.** Because the
  topic is TRANSIENT_LOCAL, a restarted node re-latches the last message —
  so `POST /api/mission/clear` (and any mission-teardown path) publishes
  an explicit cleared config (`mode=continuous`, empty `points`/`flags`)
  rather than leaving the old mission latched. Otherwise a spray-node
  restart after a clear silently resurrects stale geometry.

---

## 4. Actuator command state machine

Replace the current scattered-boolean tracking (`_commanded`,
`_off_confirmed`, `_cmd_seq`, retry timers) with an explicit FSM. The
existing sequence-id anti-stale-reply mechanism (`_cmd_seq`) and the
"never optimistically claim ON" invariant are correct and are kept —
they're just promoted from implicit to a formal, unit-testable transition
table.

```
States:
  OFF_UNCONFIRMED   — startup / just booted; actuator's real state unknown
  OFF_CONFIRMED     — MAVROS acked an OFF for the current cmd_seq
  ON_PENDING        — ON dispatched, awaiting MAVROS ack
  ON_CONFIRMED      — MAVROS acked ON for the current cmd_seq
  OFF_PENDING       — OFF dispatched (from ON_CONFIRMED or ON_PENDING), awaiting ack
  RECOVERY          — an OFF command was rejected/failed; retrying on backoff
  DISABLED          — spray_enabled=False or a fail-safe fired; ON is refused
                        at the FSM boundary regardless of caller intent

Transitions (event → new state):
  boot                                   → OFF_UNCONFIRMED
  OFF_UNCONFIRMED  + send(OFF)           → OFF_PENDING
  * (any state)    + safety_loss         → OFF_PENDING   (force=True, bypass backoff)
  * (any state)    + spray_disabled      → DISABLED, then OFF_PENDING
  DISABLED         + spray_re_enabled    → OFF_UNCONFIRMED  (re-confirm OFF before
                                            any ON is possible again — never jump
                                            straight to OFF_CONFIRMED on re-enable)
  OFF_CONFIRMED    + desired(ON) & safety_ok → ON_PENDING
  ON_PENDING       + ack(success)        → ON_CONFIRMED
  ON_PENDING       + ack(fail/timeout)   → OFF_PENDING     (never latch a failed ON as ON)
  ON_PENDING       + desired(OFF)        → OFF_PENDING     (caller changed its mind
                                            before the ON ack landed; stale ON ack
                                            is then ignored via cmd_seq)
  ON_CONFIRMED     + desired(OFF)        → OFF_PENDING
  OFF_PENDING      + ack(success)        → OFF_CONFIRMED
  OFF_PENDING      + ack(fail/timeout)   → RECOVERY
  RECOVERY         + backoff_elapsed     → OFF_PENDING     (retry)
  RECOVERY         + ack(success)        → OFF_CONFIRMED
```

Invariants (enforced in code, asserted in tests):
- **`spraying == True` is only ever published when state == `ON_CONFIRMED`.**
  No optimistic-ON window exists anywhere, including the instant a command
  is dispatched (this closes the exact class of bug Task 17/18/19 was
  built to fix on `main`, but makes it structurally impossible instead of
  behaviorally patched).
- Every dispatched command carries the state machine's monotonic `cmd_seq`;
  a MAVROS reply is applied only if `seq == current cmd_seq` (kept from
  today's node — correct as-is).
- `RECOVERY` retry backoff: `min(0.5 * 2^attempt, 5.0)` seconds, reset to
  `0.5s` the moment a config/mode change or safety-loss edge occurs. (An
  improvement over today's flat `500ms` retry — bounded exponential
  backoff avoids hammering MAVROS if the command service is genuinely
  degraded, while a fresh event still gets a fast first retry.)
- `DISABLED` is reachable from every other state and is checked **before**
  any ON dispatch, not just as a periodic watchdog sweep — closes any
  theoretical race between "gate flips closed" and "in-flight ON send".
- Startup always begins in `OFF_UNCONFIRMED` and must reach `OFF_CONFIRMED`
  before the FSM will ever accept an `ON` transition — formalizes today's
  "drive OFF at boot, don't trust believed state" behavior, which is
  correct and is kept.

This FSM is implemented as a plain Python class with no ROS/rclpy
dependency (`SpraySafetyStateMachine` — pure `state`, `handle(event) ->
(new_state, side_effects)`), so it is fully unit-testable (§8) without a
ROS runtime, unlike today's logic which is entangled with the `Node`
subclass.

---

## 5. Safety gate stack

All gates are AND-ed; any failing gate forces `desired(OFF)` regardless of
mode logic. Ordered by how often each actually trips in practice (fail
fast on the cheap/common ones):

| Gate | Formula / condition | Source |
|---|---|---|
| Master enable | `spray_enabled == True` | existing, keep |
| Armed | `armed == True` | existing, keep |
| Flight mode | `mode == "OFFBOARD"` unless `manual_active` (manual only needs armed — cmd 187/183 accepted in any armed mode) | existing, keep |
| Pose freshness | `age(pose) <= pose_timeout_s` (default 0.5s) | existing, keep |
| Velocity freshness | `age(velocity) <= velocity_timeout_s` (default 0.5s) | existing, keep |
| Cross-track error | `xtrack_error_m <= max_xtrack_error_m` (default 0.10m) | existing, keep |
| ~~Min spray speed~~ (**REMOVED as a gate — Rev 4**) | Rev 3 gated `speed_mps >= min_spray_speed_mps`. `6523a84` deleted this: slow no longer means OFF, it means **thin**, which §7.5 flow control handles. `min_spray_speed_mps` is still *declared* but is **not** consulted for on/off. The Rev 3 "swap per mode for point" instruction is therefore a no-op and is dropped. Kept in the table struck-through so no future reader re-adds it thinking it was an omission. | superseded |
| **Pivot-state gate (the real mode-relevant gate — Rev 4)** | `/rpp/segment_debug[1] != CORNER_ALIGN`. When the driving controller reports it is pivoting in place, moving-mode spray (continuous/dash) is held OFF — a discrete state that cannot dither at the frozen corner-crawl speeds (0.08/0.08/0.03), which a speed threshold would. **Point mode requires a scoped exemption to this gate (§7.3) — a dwell sprays while stopped.** | existing (`_segment_debug_cb`), formalized |
| **RTK/GPS fix quality (NEW)** | see §7.6 | new |
| Session config loaded | mode-appropriate config present (path model for continuous/dash, coordinate list for point) | existing (as "path not loaded"), generalized to all 3 modes |

Each gate reports a single `(ok: bool, reason: str)` in a priority-ordered
check — the **first** failing gate's reason is what gets published to
`/spray/status.safety_reason`, so operators always see the actual blocking
cause instead of a generic "safety blocked."

---

## 6. Telemetry / status contract

Root cause of defect #1 (`main`'s crash loop) was an untyped float
(`inf`) reaching `json.dumps(allow_nan=False)`. V2 status is a typed
dataclass, converted at the publish boundary with an explicit sanitizer —
not "add a recursive inf-scrubber and hope every future field goes through
it" (which is what the `main` fix (`spray_runtime_protocol.py`) had to
retrofit after the fact):

```python
@dataclass(frozen=True)
class SpraySessionStatus:
    schema_version: int
    mode: str
    fsm_state: str                       # one of the §4 state names
    spraying: bool                       # true only if fsm_state == ON_CONFIRMED
    desired: bool
    manual_active: bool
    safety_ok: bool
    safety_reason: str
    distance_to_boundary_m: Optional[float]   # None, never inf — capped/omitted at source
    gps_fix_ok: bool
    gps_fix_name: str
    xtrack_error_m: Optional[float]
    mode_state: dict                     # small mode-specific fields, see §7.2/§7.3
```

Rule: **no field is ever assigned `float("inf")`/`float("nan")`.**
Distance-to-boundary is `None` when there is no next boundary (e.g.
continuous-mode mission with zero transitions), not `inf` — the type
system (`Optional[float]`) makes the "no value" case explicit at the
producer instead of relying on a downstream serializer to catch it.
`json.dumps(..., allow_nan=False)` stays on as a backstop, but the
dataclass makes it unreachable in normal operation rather than the only
line of defense.

---

## 7. Controller modes

Shared machinery for all three modes: the existing `_build_path_model` /
`_project_onto_path` / cumulative arc-length (`cumulative_s`) functions in
today's node are correct and reused as-is (they're pure functions, already
well-isolated — no rewrite needed there).

### 7.1 Mode 1 — Continuous (existing, carried forward)

Unchanged from today's distance-aware logic:
- `on_lead_m = speed_mps * solenoid_open_delay_s + on_overspray_margin_m`
- `off_lead_m = max(0, speed_mps * solenoid_close_delay_s - off_overspray_margin_m)`
- Fires `ON` when `distance_to_next_boundary_m <= on_lead_m` approaching a
  `TRANSIT_TO_MARK` boundary; fires `OFF` similarly on `MARK_TO_TRANSIT`.

This logic is field-validated on this branch already (per project memory)
— V2 does not touch it, only relocates it under the new FSM/config model.

### 7.2 Mode 2 — Interval / Dash

State (`mode_state` for dash): `{s_at_last_toggle: float, phase: "on"|"off"}`.

Metering is **cumulative mission arc-length** (`cumulative_s`, continuous
across corners and entity boundaries — locked decision #2). Formula, run
every tick once pose is projected onto the path:

```
s = projection.s                          # current arc-length position
elapsed = s - s_at_last_toggle
target = on_distance_m  if phase == "on"  else off_distance_m

if elapsed >= target - on_lead_m (when phase would flip to "on")   → flip phase, s_at_last_toggle = s
if elapsed >= target - off_lead_m (when phase would flip to "off") → flip phase, s_at_last_toggle = s

geometry_desired = (phase == "on")
```

The same `on_lead_m`/`off_lead_m` solenoid-compensation formulas from §7.1
apply at each dash toggle boundary, not just at entity boundaries — a dash
toggle **is** a boundary in this model (`SprayBoundary` list is generated
dynamically from `s_at_last_toggle + {on,off}_distance_m` instead of
statically from flag changes, but consumed by the same boundary-lead-time
machinery as continuous mode).

Explicit corner-crossing rule (**re-specified in Rev 4 against the pivot
state**): if `elapsed` crosses `target` while the rover is mid-pivot at a
corner, the **phase still flips** at the correct `s` (arc-length doesn't
care about time spent stationary), but the *actuator command* is deferred
until the pivot ends — so a dash segment can span a full stop-and-pivot
without either (a) leaking paint at a standstill or (b) losing metering
accuracy.

Rev 4 change — how "mid-pivot" is detected. Rev 3 said detect it "purely
from the node's own velocity (`speed_mps < min_spray_speed_mps`), with no
dependency on any RPP substate topic." That is **wrong against shipped
code**: the node already subscribes to `/rpp/segment_debug`
(`_segment_debug_cb`) and `6523a84` made the pivot *state*, not speed, the
authority. Dash deferral therefore keys off the same **discrete pivot-state
gate** as §5: defer the actuator command while
`/rpp/segment_debug[1] == CORNER_ALIGN`, resume when it clears. This is
strictly better than the velocity test Rev 3 assumed — a discrete state
cannot chatter, whereas a speed threshold dithers against the frozen
corner-crawl speeds (0.08 / 0.08 / 0.03) and would flip the solenoid on and
off through the crawl. The `s`/phase bookkeeping is unchanged; only the
"is it safe to actuate right now" signal is the pivot state, not a speed
compare. The inter-node coupling Rev 3 wanted to avoid **already exists and
is load-bearing** — V2 uses it rather than pretending it away.

**OPEN road-marking decision (Rev 4 — decide before Phase C config is
frozen): does the dash pattern reset per line segment, or run continuously
across the whole mission?** Locked decision #2 says continuous mission
arc-length (a 6-on/3-off pattern flows *through* a corner and does not
restart at each new line). For road pre-marking the operator may instead
want the pattern to **restart at each surveyed line** (so every line begins
with a full 6 m dash, not a partial one), and may even want a **different
pattern per line** (section A = 6/3, section B = 3/6). Rev 3's single global
`{on,off}_distance_m` pair cannot express that. This is left OPEN, not
silently locked:
  • If continuous-across-mission is acceptable → Rev 3's global pair stands,
    Phase C is simplest.
  • If per-line reset / per-line pattern is wanted → Phase C config must
    carry the pattern **per segment** (natural home: `PathSegment.metadata`,
    the dict that today carries only `geometry_type`), and the toggle math
    resets `s_at_last_toggle` at each segment boundary instead of never.
    Cheap to design in now, expensive to retrofit later.
  Recommendation: confirm with the operator which road-marking behaviour is
  required and design Phase C's config for the per-segment case even if the
  first shipped pattern is a single global pair — the substrate cost is one
  dict field.

**Monotonic-`s` guard (self-overlapping paths):** `_project_onto_path`
returns the *nearest*-segment arc-length, so on a retrace/self-overlapping
run (figure-8, or the retrace geometry this branch already handles) `s` can
jump backward or forward between ticks, which would double-fire or skip
dash toggles. Dash metering therefore does **not** consume raw
`projection.s`; it integrates a monotonic `s_dash = max(s_dash_prev,
projection.s)` clamped to non-decreasing, and rejects any single-tick jump
larger than `speed_mps * dt * jump_tolerance_factor` (default 3.0) as a
projection glitch rather than real travel. Continuous mode's on/off
boundaries are static so they're immune to this; only dash's dynamic
boundary generation needs the guard.

**Jump-rejection recovery rule (the guard must not wedge):** a rejected
jump does not advance `s_dash`, so if the projection has *legitimately*
moved (retrace geometry snapping to a later pass, a resumed run) every
subsequent tick's delta vs the frozen `s_dash` also exceeds tolerance and
dash metering would freeze permanently. Therefore: if
`jump_reject_accept_after` (default 5) **consecutive** ticks are rejected
and their raw `projection.s` values agree with each other to within
`speed_mps * dt * jump_tolerance_factor` (i.e. it's a stable new position,
not noise), the new `s` is accepted as real travel, `s_dash` snaps to it,
and the event is logged + surfaced in `mode_state`. A one-tick glitch is
still rejected; a persistent, self-consistent relocation always recovers
within ~`jump_reject_accept_after` ticks.

**Dash-start criterion ("on the path proper" made computable):**
`_project_onto_path` always returns a nearest point, including during the
entry transit, so "first tick the pose projects onto the path proper"
needs an explicit trigger: dash metering arms on the first tick where
`xtrack_error_m <= max_xtrack_error_m` (reusing the existing §5 gate
threshold, no new param). Before that tick, `s_at_last_toggle` and
`s_dash` remain uninitialized and `geometry_desired = False`.

Mission-start initialization: `s_at_last_toggle = 0.0`, `phase =
dash.start_state`. Note the mission may begin with a runtime **entry
transit** to wp0 (D1–D4, project memory) before arc-length 0 is reached;
dash metering starts from the first tick the pose projects onto the path
proper, not from the entry-transit leg (which carries no dash geometry).

### 7.3 Mode 3 — Point / Coordinate

State (`mode_state` for point): `{target_index: int, phase: "transit" |
"arriving" | "holding" | "dwelling" | "advancing"}`.

FSM per point (index `i`, target `(n_i, e_i)`):

```
TRANSIT   : rover navigating toward target_index (RPP's job, not spray's —
            spray just watches). geometry_desired = False.
ARRIVING  : distance_to_target_m = hypot(pose_n - n_i, pose_e - e_i)
            arrived = (distance_to_target_m <= arrival_tolerance_m)
                       and (heading_tolerance_deg is None or |yaw_err| <= heading_tolerance_deg)
                       and (speed_mps <= point_arrival_max_speed_mps)
            if arrived: start settle timer
            if arrived continuously for >= arrival_settle_s: → HOLDING
            else if arrived flag drops before settle completes: reset timer (no partial credit)
HOLDING   : geometry_desired = True (spray ON); start dwell timer at entry
DWELLING  : remain HOLDING's ON state until dwell_s elapsed, then
            geometry_desired = False; wait for FSM §4 to reach OFF_CONFIRMED
ADVANCING : target_index += 1; if target_index == len(coordinates): mode
            complete (geometry_desired = False, report mission-mode-done);
            else target_index's point → TRANSIT
```

**Mission-side contract (required — spray "just watching" is not enough
on its own):** the spray node observes arrival, but *something must make
the rover actually stop and hold* at each coordinate for at least
`arrival_settle_s + dwell_s + OFF-confirm margin`. A point-mode mission is
therefore planned as a path whose waypoints are the coordinate list with a
**per-point hold** at each target — reusing the controller's existing
final-segment stop behavior (`segment_endpoint_approach_speed` /
stop-dwell machinery, already field-proven) rather than any new RPP state.
Hold duration is mission-plan data (path_engine/server side), computed
from the same `SpraySessionConfig` dwell params so the two cannot drift.
Without this contract the rover drives *through* the points, ARRIVING
never satisfies, and every point falsely times out. This is a path_engine/
server deliverable of Phase D, called out here so Phase D is scoped as
two-sided (planner + spray node), not spray-node-only.

Notes:
- **Pivot-gate exemption (NEW — Rev 4, the point-mode blocker Rev 3
  missed).** §5's pivot-state gate holds moving-mode spray OFF while the
  driving controller reports `CORNER_ALIGN`. But a point dwell **sprays at a
  standstill by definition** — and nothing in the shipped code distinguishes
  "stopped to pivot" from "stopped to dwell," so left alone the gate
  suppresses *every dot*. Point mode therefore needs a scoped exemption:
  the pivot gate is bypassed **only when** `mode == "point"` **and** the
  rover is within `arrival_tolerance_m` of the active target coordinate (i.e.
  the FSM is in `HOLDING`/`DWELLING`). Two hard constraints on the exemption:
  (1) it is **node-side only** — the frozen RPP controller is not touched;
  (2) it is **never mode-wide** — on the transit legs *between* dots the
  gate stays fully active, or corner leakage returns on those transits. This
  is the one gate change point mode requires; everything else in this FSM is
  unchanged from Rev 3.
- `arrival_settle_s` guards against a momentary tolerance-satisfying blip
  (GPS jitter) triggering a spray at the wrong spot — matches the existing
  RPP corner-align settle-time pattern (`segment_align_settle_s`) already
  proven on this branch, reused here for consistency rather than inventing
  a new tuning knob class.
- Point mode explicitly waits for `OFF_CONFIRMED` (§4) before advancing,
  not just for `dwell_s` to elapse — so a failed/retrying OFF command
  cannot cause the rover to move off a still-spraying point.
- `heading_tolerance_deg: null` is a legal config (position-only arrival)
  for point patterns where orientation at the dot doesn't matter — kept
  optional rather than mandatory to avoid over-constraining CSV-only point
  lists that carry no heading data.
- **Unreachable-target watchdog (new — closes an infinite-stall hole):**
  `ARRIVING` cannot block forever. If a point is not reached within
  `point_arrival_timeout_s` (default 60s, param) — e.g. the RTK gate is
  holding the rover, an obstacle blocks it, or the tolerance is
  physically unsatisfiable — the point is **skipped** (logged with reason,
  `geometry_desired` stays False so no paint is laid), the FSM advances via
  `ADVANCING`, and the skip is surfaced in telemetry. This mirrors the
  server's existing mission-skip pattern rather than inventing new
  operator semantics, and prevents a single bad coordinate from wedging the
  whole point mission. A skipped point never silently counts as sprayed.
  **Desync guard:** a watchdog skip must not advance spray's
  `target_index` unilaterally while the rover is still navigating to the
  skipped point — that would leave spray watching point `i+1` while the
  rover arrives at point `i`. Skip is therefore **index-synchronized**:
  spray's `target_index` always tracks which hold the *rover* is at/next,
  derived from proximity (`argmin` over remaining coordinates within the
  path model), not from a free-running counter. The timeout marks point
  `i` as `skipped` in telemetry and refuses to spray at it if the rover
  does eventually arrive late — it does not advance the pointer ahead of
  the rover.

### 7.4 Obstacle-integration hook (documented, not implemented)

Per locked decision #3, no obstacle params or logic are added. The one
forward-looking commitment: the §5 gate table is a literal ordered list a
future `obstacle_clear` gate can be inserted into without restructuring —
called out here so the *next* person doesn't have to re-derive that the
gate stack is meant to be extensible.

---

### 7.5 Speed-Proportional Flow Rate (cross-mode)

**Problem:** today's actuator only ever commands `on_value` (full flow) or
`off_value` (fully closed). Line density is therefore proportional to
*time spent over a point*, which is inversely proportional to speed — a
speed-up mid-line thins the paint toward gaps, a slow-down pools it. This
is a real, well-solved problem in industrial spraying (precision-ag rate
controllers), not a novel one — see grounding at the end of this section.

**Design — decoupled from the safety FSM, not a new FSM state.** The §4
actuator state machine still only answers a binary question: *is spraying
allowed right now* (`OFF_CONFIRMED` / `ON_CONFIRMED` / …). Layered on top,
active only while `state == ON_CONFIRMED`, a small stateless-per-tick
**flow modulator** answers a second, independent question: *how much
flow, right now*. This mirrors how real PWM rate-controllers work too —
they still have a binary boom/section on-off state, with continuous duty
modulation running underneath it (see sources below). Keeping these two
concerns separate means the safety-critical FSM in §4 is untouched by this
feature — no new safety states, no new failure modes in the part of the
system that has to be provably correct.

**Hardware reality check (why this is simpler for us than classic ag PWM):**
classic ag PWM pulses a binary solenoid at 10–20 Hz to fake an analog rate
on a constant-pressure line, because those nozzles are true open/shut
valves. Our actuator (`actuator_backend=mavlink_actuator`, normalized
-1..1 into a DC pump, or `mavlink_servo_pwm`, direct microseconds) already
accepts a **continuously variable** command — `on_value`/`on_pwm_us` are
just the two endpoints of a range that was previously never used in
between. So V2 does not need to simulate PWM pulsing at all: it commands
a direct analog throttle value between a calibrated minimum and
`on_value`, recomputed every reassert tick. Simpler mechanism, same
outcome (flow ∝ speed).

**Formula (continuous/dash modes — any tick where the rover is moving):**

```
duty(t) = clamp(
    min_flow_value + (on_value - min_flow_value) * (speed_mps / rated_marking_speed_mps),
    min_flow_value,
    on_value,
)
# then slew-rate limited so the pump is never step-commanded:
duty_cmd(t) = duty(t-1) + clamp(duty(t) - duty(t-1), -max_slew_per_s * dt, +max_slew_per_s * dt)
```

New params: `rated_marking_speed_mps` (the speed at which `on_value`
produces the correct calibrated line — default = this project's validated
baseline marking speed, 0.35 m/s, tunable), `min_flow_value` (lowest
usable pump command before the physical flow stops being reliable —
bench-calibrated, prevents commanding a value so low the pump stalls or
the line clogs), `max_slew_per_s` (caps how fast the commanded value can
change per second, protecting the pump from being step-commanded on every
speed fluctuation).

**Saturation semantics (important operational consequence):** the `clamp`
caps `duty` at `on_value` once `speed_mps >= rated_marking_speed_mps` —
above rated speed the pump is already at full flow and **cannot scale
further, so lines thin out**. Therefore `rated_marking_speed_mps` must be
set at the **top of the expected operating speed range, not its middle** —
so that full flow is reached only at max speed and there is always headroom
to scale up. Setting it mid-range guarantees under-paint on every
above-rated stretch. The §7.5 calibration procedure below is updated to say
this explicitly.

**Slew-limiter initial condition (avoids an undefined `duty(t-1)`):** on
the `OFF_CONFIRMED → ON_CONFIRMED` edge the slew filter's previous value is
seeded to `min_flow_value` (not the stale pre-OFF value and not `on_value`),
so flow always ramps up from the safe floor rather than stepping or
resuming a stale command. `dt` in the slew formula is the **actual measured
tick interval** (monotonic-clock delta), not a nominal constant, so a
slow/variable reassert tick can't let the limiter over- or under-shoot.

**Mode-specific override — Point mode is NOT speed-scaled.** Point/dwell
spraying (§7.3) happens at zero forward speed by definition — the formula
above would floor to `min_flow_value` for the entire dwell, which is
wrong; a dot needs its own tuned flow. Point mode instead uses a fixed,
separately-calibrated `point_dwell_flow_value` param, unaffected by this
section's formula.

**Safety boundary:** the flow modulator can only ever move the commanded
value *within* `[min_flow_value, on_value]` while the FSM already says
`ON_CONFIRMED` — it has no path to keep spray on longer, turn it on
earlier, or override any §5 gate. If a gate trips, §4's FSM drives to
`OFF_PENDING` exactly as before and the flow modulator's output is moot.

**Flow-update ack semantics (distinct from ON/OFF acks — must not feed
the §4 table):** flow-value updates ride the existing reassert tick
(`reassert_hz`, default 2 Hz) through the same `_cmd_seq`-stamped command
path, but a *flow update* is not an *ON command* in FSM terms. A
failed/timed-out ack on a mid-ON flow update does **not** map to
`ack(fail) → OFF_PENDING` — otherwise a single dropped ack at 2 Hz kills
spray mid-line for no safety reason (the pump is still running at its
last-acked value; nothing unsafe has happened). Instead: log, keep the
last-acked value as `duty(t-1)` for the slew filter, and retry next tick.
Only `flow_ack_fail_limit` (default 3) **consecutive** flow-ack failures
escalate to the FSM as `ack(fail)` — at that point the command channel is
genuinely degraded and driving to `OFF_PENDING` is the correct fail-safe.
ON/OFF transition commands keep their strict single-ack §4 semantics
unchanged.

**Telemetry:** `SpraySessionStatus.mode_state` gains
`commanded_flow_value: float` and `flow_source: "speed_scaled" |
"point_fixed" | "n/a"` for field debugging (was this line thin because of
a gate, or because of the flow formula — must be visible, not inferred).

**Calibration procedure (required before field use, bench step):** paint
3 short test lines at fixed representative speeds (e.g. 0.15, 0.35,
0.6 m/s) with `duty` locked at `on_value` for each, measure line
width/opacity, then set `rated_marking_speed_mps` to the **fastest speed
the mission will actually run at** (per the saturation note above — full
flow must land at the top of the range so nothing above it under-paints),
and `min_flow_value` to the lowest value that still produces an unbroken
line at the slowest expected speed. Re-run after any nozzle/pump hardware
change.

**Grounding — this is standard practice, not a novel idea:** ground-speed-
proportional flow via duty-cycle modulation is exactly how modern precision-
ag sprayers hold application rate constant despite speed changes — TeeJet
DynaJet, Raven Hawkeye, Capstan PinPoint/SharpShooter, Case Aim Command,
John Deere ExactApply, WEEDit Quadro and Agrifac StrictSprayPlus all use
this pattern (PWM duty cycle tied to ground speed, keeping *pressure*
constant and *duty cycle* variable, which is what keeps spray pattern
quality uniform across a speed range) [Sprayers 101](https://sprayers101.com/pwm/),
[TeeJet — What is PWM?](https://www.teejet.com/teejet-news/tj_what-is-pwm),
[CAES Field Report — PWM Technology for Agricultural Sprayers](https://fieldreport.caes.uga.edu/publications/C1277/pulse-width-modulation-technology-for-agricultural-sprayers/).

### 7.6 RTK / GPS fix-quality gate (the §5 "NEW" gate, fully specified)

This is the only genuinely new safety gate in V2, so it is specified here
in full rather than left as a pointer. It mirrors `rpp_controller_node.py`'s
P0.3 gate (fix_type source, topic) but is spray-scoped.

**Source:** subscribe `/mavros/gpsstatus/gps1/raw` (`GPSRAW`), read
`fix_type`. RTK codes: `6 = RTK_FIXED`, `5 = RTK_FLOAT`, `<5` = worse.

**Threshold (param `spray_min_fix_type`, default 6):** spraying requires
`fix_type >= spray_min_fix_type`. Default demands RTK_FIXED — the same bar
RPP uses to drive. Operators can lower to 5 for RTK_FLOAT sites, but the
default is the strict one (fail-safe).

**Staleness is a distinct failure from bad fix.** A missing GPSRAW message
is NOT "fix ok, just quiet" — treat `age(last GPSRAW) > gps_fix_timeout_s`
(default 2.0s, deliberately looser than the 0.5s pose/velocity gates since
GPSRAW publishes slower) as gate-FAIL with reason `"gps stale"`, separate
from `"gps fix N < required M"`. Never let a stale-but-last-was-good reading
hold the gate open.

**Hysteresis (anti-flap):** RTK fix-type flaps at the 5↔6 boundary in the
field. Raw gating on every sample would chatter the spray on/off at a
boundary crossing. Two asymmetric edge timers:
- **Drop (spray-protecting, fast):** the instant a sample is below
  threshold OR stale, the gate fails immediately — no debounce on the
  unsafe edge (a real dropout must stop paint now).
- **Recover (spray-enabling, slow):** the gate only re-opens after fix has
  been continuously `>= threshold` for `gps_recover_hold_s` (default 1.0s).
  A single good sample after a dropout does not re-enable spray.

This asymmetry (immediate-off, delayed-on) is the standard safety-gate
shape and matches the RPP `segment_align_settle_s` "must hold before
trusting it" pattern already proven on this branch.

**Telemetry:** `SpraySessionStatus.gps_fix_ok` / `gps_fix_name` already
carry this (§6); `safety_reason` surfaces the specific cause
(`"gps fix 5 < required 6"` vs `"gps stale"`) via the §5 first-failing-gate
rule.

## 8. Testing plan

Mirrors the "verify controller patches in-env on Jetson, not by static
review" lesson already in project memory (`collinear_spray_boundary_slowdown`)
— unit tests below are necessary but not sufficient; §9 covers the field
step.

- **FSM unit tests** (`src/test_spray_fsm.py`, no ROS import needed):
  full transition table coverage, including the two invariants from §4
  (`spraying` never true outside `ON_CONFIRMED`; stale-seq replies ignored)
  as explicit property-style tests generating random event sequences.
- **Dash-mode unit tests** (`src/test_spray_dash_v2.py`): arc-length
  toggle math on synthetic paths including a corner mid-dash (the §7.2
  deferred-actuation case), a dash shorter than one leg, a dash spanning
  3+ legs.
- **Point-mode unit tests** (`src/test_spray_point_v2.py`): arrival
  settle-reset-on-blip, dwell + OFF-confirm gating before advance,
  position-only vs heading-gated arrival, empty/1-point coordinate lists.
- **Gate-stack tests**: each gate independently forces OFF; first-failing-
  reason ordering; RTK gate hysteresis (§7.6 formula) under a flapping
  fix-type sequence, including a stale-GPSRAW (no message) case.
- **Telemetry sanitization test**: construct a status with a `None`
  distance-to-boundary and assert the JSON round-trips without touching
  `allow_nan` at all (i.e. there's never a `nan`/`inf` in the dataclass to
  begin with — a regression test for defect #1's exact failure mode).
- **Config schema test**: `schema_version` mismatch is ignored, not
  crashed on; malformed config (mismatched points/flags length) is
  rejected without touching the FSM.
- **Rev 3 regression tests** (folded into the suites above, listed here so
  none is silently dropped): `DISABLED + spray_re_enabled →
  OFF_UNCONFIRMED` (never straight to OFF_CONFIRMED); point mode sprays at
  standstill / continuous+dash still refuse below `min_spray_speed_mps`
  (the §5 mode-conditional swap); monotonic-`s` jump rejection *recovers*
  after `jump_reject_accept_after` self-consistent ticks and stays frozen
  for noisy ones; dash metering stays unarmed until
  `xtrack <= max_xtrack_error_m`; watchdog skip does not advance
  `target_index` ahead of the rover (index-sync test); a single flow-ack
  failure does not leave `ON_CONFIRMED`, `flow_ack_fail_limit` consecutive
  ones do; cleared config (empty points) unloads a previously-latched
  mission.
- **Flow modulator unit tests** (`src/test_spray_flow_model.py`): duty
  scales linearly with speed between `min_flow_value` and `on_value`;
  clamps correctly above `rated_marking_speed_mps` and below the min gate
  speed; slew-rate limiter caps a step speed change to `max_slew_per_s`;
  point mode always returns `point_dwell_flow_value` regardless of speed
  input; flow modulator never produces a value when FSM state isn't
  `ON_CONFIRMED` (safety-boundary test).

Run `src/` spray tests separately from `server/` (existing project
gotcha, memory: `spray_actuator_safety_architecture` — module-name
collisions if combined).

---

## 9. Infra: failure isolation (independent of node rewrite, do first)

`rpp_start.sh` on this branch **still** routes `spray_controller` through
the same global `record_fail` / `MAX_FAILS_IN_WINDOW=5` threshold as the
OFFBOARD-critical nodes (`twist_to_setpoint`, `rpp_controller`) — confirmed
by reading the current script; the isolation fix from the
`spray_continuous_pipeline_crash` postmortem was written but never landed
here. This is a cheap, high-value fix that should ship **before** the V2
node rewrite, independent of it:

- Split `is_critical_node` (`twist_to_setpoint`, `rpp_controller`) vs
  auxiliary (`spray_controller`, `xtrack_logger`).
- Only critical-node failures call `record_fail` / can trip a full
  pipeline restart.
- Auxiliary nodes get their own isolated respawn with backoff
  (2s → 30s), and while `spray_controller` is down the actuator fails
  safe OFF (already true structurally — MAVROS just stops receiving
  commands, PX4 doesn't hold a stale ON).

This means: even in the worst case where the V2 node has a bug that makes
it crash, the blast radius is "no spray for N seconds while it respawns,"
never "OFFBOARD heartbeat drops and the rover coasts through a corner" —
the exact failure chain that caused the 2026-06-25 field incident.

---

## 10. Executor / threading

Keep `SingleThreadedExecutor` (implicit default via `rclpy.spin(node)`,
as today). Defect #5 above is `main`'s only reason to ever consider
multi-threading this node, and that reason doesn't exist here — the spray
node has no blocking I/O that benefits from parallelism; `ReentrantCallbackGroup`
is used today only so a manual-override callback isn't blocked behind a
slow tick, which doesn't require a multi-threaded executor. No change
planned; called out so it isn't "improved" into a regression later.

---

## 11. File layout

Deliberately flatter than `main`'s ~15-file spread (a direct consequence
of decision #1 — no server-side parallel state to house):

```
src/
  spray_fsm.py            # NEW — pure SpraySafetyStateMachine (§4), no rclpy import
  spray_modes.py          # NEW — pure functions: dash toggle math (§7.2),
                           #        point arrival/dwell FSM (§7.3)
  spray_flow_model.py     # NEW — pure speed→duty formula + slew limiter (§7.5)
  spray_session_config.py # NEW — schema dataclass + parse/validate (§3)
  spray_controller_node.py# MODIFIED — ROS glue only: subscriptions, timers,
                           #            wires §4/§7 modules together, publishes
                           #            §6 status. Path-projection helpers
                           #            (_build_path_model etc.) stay, moved
                           #            into spray_modes.py as shared geometry.
  test_spray_fsm.py            # NEW
  test_spray_modes.py          # NEW (dash + point unit tests)
  test_spray_session_config.py # NEW

server/routes/spray.py    # MODIFIED — minimal: publish SpraySessionConfig
                           #            on mission load; existing manual
                           #            on/off/enable/test endpoints largely
                           #            unchanged (they already talk to the
                           #            node via /spray/manual, not a
                           #            server-side FSM)
rpp_start.sh               # MODIFIED — §9 critical/auxiliary node split
```

No `server/spray_safety.py`, `spray_mode_store.py`, `spray_mission_config.py`,
`spray_startup_reconciliation.py`, `spray_runtime_protocol.py` equivalents —
their responsibilities are absorbed into the node-owned modules above or
eliminated by the config model in §3.

---

## 12. Rollout phases

Each phase is independently field-testable before the next starts —
mirrors how Mode 1 already shipped incrementally on this branch.

1. **Phase A — FSM + telemetry foundation.** Land `spray_fsm.py`,
   `spray_session_config.py`, refactor `spray_controller_node.py` onto
   them for **Mode 1 only** (behavior-preserving — same continuous-mode
   logic, new internals). Land the §9 `rpp_start.sh` isolation fix in the
   same phase (unrelated code, but this is the natural checkpoint). Dry-run
   bench validate, then one field run confirming Mode 1 parity with today.
2. **Phase B — RTK/GPS gate.** Add the §7.6 fix-quality gate to the node,
   subscribe `/mavros/gpsstatus/gps1/raw` directly (mirrors
   `rpp_controller_node.py`'s existing pattern — no new dependency).
   Bench-test with simulated fix-type flapping, then field-validate an RTK
   dropout during a live mark run forces OFF and recovers correctly.
3. **Phase C — Dash mode.** Land `spray_modes.py` dash logic + config
   support. Bench-validate on a synthetic multi-leg path with a corner
   mid-dash (§7.2's deferred-actuation case) before any field run.
4. **Phase D — Point mode.** Land point FSM + coordinate-list ingest.
   Bench-validate arrival/dwell/advance on a 3+ point synthetic mission.
5. **Phase E — Speed-proportional flow.** Land `spray_flow_model.py` +
   reassert-tick integration (§7.5). Run the bench calibration procedure
   at 3+ speeds *after* modes are stable, since calibration should reflect
   real mark scenarios (straight-line continuous, dash toggling, and — if
   the nozzle-offset plan (see companion doc) has landed by this point —
   the corrected antenna path) rather than a synthetic bench-only speed
   sweep.
6. **Phase F — Field validation pass.** Dry-run (no paint) first on every
   mode per the existing operator rule, then live-paint field runs per
   mode (including a deliberate speed-varying pass to confirm line density
   stays visually constant), then the RTK-dropout live test if not already
   covered in Phase B.

No phase is scoped to land silently — each ends with an explicit
bench-then-field checkpoint, consistent with how every prior spray change
on this project has been validated (memory: state changes must be
verified in-env on Jetson, not by static review).

---

## 13. Open questions for a later pass (not blocking this plan)

- Server-side per-mission spray-mode persistence (so a saved mission
  remembers its mode/dash-distances across reloads) — §2 scoped this out;
  straightforward to add later as a thin wrapper that just re-publishes
  the same `SpraySessionConfig` on mission load from a stored sidecar file.
- Obstacle-sensing gate (§7.4) — hook point exists, no implementation
  until a sensor is specified.
- Physical spray-flow feedback — would upgrade `physical_feedback_supported`
  from `False` to `True` and let `force_spray_off_confirmed`-equivalent
  logic require a real physical OFF read, not just a command ack. Hardware
  dependency, not a software gap.
