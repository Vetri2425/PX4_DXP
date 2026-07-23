# RPP Progress Authority + Spray Handshake — Design Doc

Status: **DESIGN / not implemented** · Target branch: `Upgrade_Spray` · Frozen-controller rule applies (every controller change ships default-OFF behind a named A/B).

Author intent (operator, 2026-07-23): *"RPP owns where-am-I-in-the-mission and announces it; the spray controller handles the valve with **proof**, not blindly. Same principle for point, continuous, and dash. For point mode: RPP stops at the point (2 cm), says 'I reached', spray dwells, says 'done', then RPP either auto-continues or — in manual mode — waits for a frontend command before driving to the next point."*

---

## 1. Why (the problem today)

The two nodes each decide independently from `/path`, with **no signal between them**:

- **Dual projection / drift.** Both `rpp_controller` and `spray_controller` project the pose onto `/path` on their own. They can disagree (frame, 100 ms timing skew, xtrack). Nothing reconciles them.
- **No handshake.** Point mode is *time-coordinated*: RPP brakes near a point and holds a **fixed** `point_hold_s`; spray independently guesses arrival. Neither says "reached" or "done." (`rpp_controller_node.py:_point_hold_tick`, `spray_controller_node.py` PointMeter.)
- **No 2 cm stop.** `point_hold_acceptance_m` = 0.10 is a **trigger radius**, and the corner-brake stops *wherever it coasts* — not a servo onto the coordinate.
- **No manual mode.** The app's `point_execution_mode: auto|manual` dead-ends in the acknowledge-only route (`server/routes/path.py`) — it never reaches the RPP.
- **Blind spray at stops.** Where the spray node's geometry guess is weakest (a stop), there is no confirmation that the rover is actually *on* the point.

What already exists and points the way: **the spray node already consumes an RPP state as a gate.** It subscribes to `/rpp/segment_debug` and reads `CORNER_ALIGN` (=3) to hold spray OFF during a pivot (`spray_controller_node.py:697`, `_SEGMENT_STATE_CORNER_ALIGN`). This design **generalizes that one pattern** into a full progress channel.

---

## 2. Principle

> **RPP is the single authority on mission progress and announces it. Spray owns valve *timing* and reacts to RPP's progress as *proof* — layered on top of its own geometry as a fallback, never as a blind single point of failure.**

Two rules that make it robust rather than a regression:

1. **Announce distance-to-boundary, not just arrival.** Moving marks (continuous/dash) need the valve to *lead* the boundary by `solenoid_open_delay × speed`. A bare "I reached the mark start" event fires the valve **late**. The progress channel therefore carries the **metric distance to the next boundary**, so spray keeps its speed-compensated lead off an authoritative source.
2. **Proof gates; it does not replace.** Spray keeps its `/path` geometry brain. RPP progress *confirms* stops (point/corner handshake) and *sources* boundary distance. If the progress channel is stale, spray degrades to today's geometry — no hard dependency.

Division of labor stays clean:

| Concern | Owner |
|---|---|
| Where am I / which phase / distance to next boundary / confirmed stop | **RPP** |
| When the valve opens/closes, actuator lead, speed→flow | **Spray** |
| Auto-continue vs wait-for-operator | **RPP** (gate) + **server/frontend** (command) |

---

## 3. State machine (RPP)

Extend the existing `SegmentStateCode` (`INACTIVE, TRACK_SEGMENT, PRE_CORNER_SLOWDOWN, CORNER_ALIGN, DONE, CORNER_STOP`) with a **marking-progress** enum used across all modes. This is a *superset* — the current codes keep their meaning and numeric values; new codes are appended.

```
MissionPhase (new, published on /rpp/progress):
  IDLE            0   no path / no pose
  TRANSIT         1   driving a no-spray connector (spray must be OFF)
  PRE_EXT         2   inside a PRE extension leg (approaching a mark, spray OFF)
  APPROACH_MARK   3   within lead distance of a MARK start (spray may lead ON)
  MARK_TRACKING   4   inside a MARK region, tracking the line
  MARK_END        5   within lead distance of the MARK end (spray may lead OFF)
  AFT_EXT         6   inside an AFT extension leg (spray OFF)
  APPROACH_POINT  7   point mode: closing on must-hit point i
  AT_POINT        8   point mode: precise-stopped & confirmed on point i (HANDSHAKE)
  DWELL_HOLD      9   point mode: holding while spray dwells point i
  WAIT_OPERATOR  10   point mode (manual): dwell done, waiting for /point/advance
  REACHED_END    11   final waypoint reached, mission complete
```

Per-mode phase paths:

- **Continuous:** `TRANSIT → APPROACH_MARK → MARK_TRACKING → MARK_END → TRANSIT → …`
- **With extensions:** `TRANSIT → PRE_EXT → APPROACH_MARK → MARK_TRACKING → MARK_END → AFT_EXT → TRANSIT → …`
- **Dash:** identical phase path to continuous; the ON/OFF *within* `MARK_TRACKING` is still the spray node's `DashMeter` (RPP does not dash-meter).
- **Point:** `TRANSIT → APPROACH_POINT → AT_POINT → DWELL_HOLD → (auto: TRANSIT to i+1 | manual: WAIT_OPERATOR → TRANSIT to i+1) → … → REACHED_END`

Boundary source: MARK/PRE/MARK/AFT boundaries are already encoded on `/path` (spray-flag `bit0` transitions + the planner's per-line extension segments). RPP computes which region it is in and the along-path distance to the next transition — it does not need new geometry, only to *project and announce*.

---

## 4. Topic contracts

Three channels. JSON `std_msgs/String` payloads (matches the `/spray/status` precedent — extensible, no custom-msg build step).

### 4.1 `/rpp/progress` — continuous state (RPP → spray), 50 Hz, BEST_EFFORT depth 1
For anticipation; loss-tolerant because it republishes every tick.
```json
{
  "phase": 4,                       // MissionPhase
  "phase_name": "MARK_TRACKING",
  "segment_index": 12,              // index into the mark/transit run list
  "point_index": -1,                // point mode: current must-hit target, else -1
  "dist_to_next_boundary_m": 0.34,  // along-path metres to the next phase transition
  "next_boundary": "MARK_END",      // what the next transition is
  "speed_mps": 0.35,
  "stopped": false,                 // confirmed physically stopped (corner/point)
  "xtrack_m": 0.011                 // signed cross-track, for observability
}
```

### 4.2 `/rpp/milestone` — discrete events (RPP → spray/server), RELIABLE, depth 10, VOLATILE
For the handshake; must never be dropped. Published once per transition.
```json
{ "event": "AT_POINT", "index": 2, "seq": 41, "stamp_ns": 173... }
// events: MARK_START, MARK_STOP, PRE_START, AFT_STOP, AT_POINT, DWELL_DONE_RPP, REACHED_END
```
`seq` is monotonic so a consumer can detect a gap and re-sync from `/rpp/progress`.

### 4.3 `/spray/point_done` — completion (spray → RPP), RELIABLE, depth 10, VOLATILE
Spray tells RPP it finished a point so RPP may advance.
```json
{ "point_index": 2, "done": true, "seq": 17, "reason": "dwell_complete" }
```

### 4.4 `/point/advance` — operator command (server → RPP), RELIABLE, depth 1, VOLATILE
Manual-mode "next point". Published by the server on a frontend button press.
```json
{ "advance": true, "expect_index": 2 }   // expect_index guards against stale double-taps
```

QoS rationale mirrors existing choices: high-rate telemetry = BEST_EFFORT (like `/rpp/segment_debug`); commands/events that must arrive = RELIABLE VOLATILE (like `/spray/manual`, never TRANSIENT_LOCAL so a restart can't replay a stale "advance").

---

## 5. Spray consumption rules (per mode)

Gated by a single spray param `consume_rpp_progress` (default **False** → byte-for-byte today's behavior).

- **Continuous / dash (moving marks):**
  - Use `/rpp/progress.dist_to_next_boundary_m` + `next_boundary` as the *authoritative* boundary for the existing lead math (`solenoid_open_delay_s`, `on/off_overspray_margin_m`) instead of the spray node's own `/path` projection. Same equations, single source → no dual-projection drift.
  - Dash: RPP gives the MARK-region envelope; the `DashMeter` still does the ON/OFF sub-pattern inside it.
  - **Fallback:** if `/rpp/progress` is stale (> `progress_timeout_s`), revert to the local `/path` projection for that tick. Proof augments; it never strands the valve.
- **Point (stops):**
  - Ignore self-arrival guessing. Spray fires **only** after `/rpp/milestone AT_POINT i`. Then run the existing dwell FSM (settle already guaranteed by the RPP stop → settle can be ~0), and on confirmed actuator OFF publish `/spray/point_done i`.
  - This is the "with proof, not blindly" case: the rover is *confirmed on the point* before a drop is sprayed.

---

## 6. The handshake (point mode)

```
RPP                                  SPRAY                         SERVER/FRONTEND
 │ drive path → APPROACH_POINT(i)     │                             │
 │ PRECISE_STOP(i) (≤2 cm, §7)        │                             │
 │ confirm stopped                    │                             │
 │ ── milestone AT_POINT i ─────────► │ dwell point i               │
 │ phase=DWELL_HOLD, hold zero        │ (settle≈0, dwell_s, OFF)    │
 │                                    │ ── /spray/point_done i ───► │
 │ ◄──────────────────────────────── │                             │
 │ mode?                              │                             │
 │  auto  → TRANSIT to i+1            │                             │
 │  manual→ WAIT_OPERATOR ────────────────────────────────────────►│ show "Next point" button
 │         ◄─── /point/advance ───────────────────────────────────│ operator taps
 │         → TRANSIT to i+1           │                             │
 └── … → REACHED_END                  │                             │
```

Backstops (robustness):
- **Spray never says done:** RPP holds at most `point_hold_max_s` (cap), then advances with a logged warning. Never wedges the mission.
- **Missed `AT_POINT`:** RELIABLE topic + `seq` gap detection; spray re-syncs from `/rpp/progress.phase==AT_POINT`.
- **`/point/advance` lost / operator away (manual):** RPP holds indefinitely (safe) — optional `manual_wait_timeout_s` (0 = wait forever) for unattended runs.
- **Double-tap advance:** `expect_index` rejects a command for the wrong point.

---

## 7. Precise stop at the point (2 cm)

Today's `_point_hold_tick` brakes when within `point_hold_acceptance_m` (10 cm) and rests wherever it coasts. To hit **2 cm** in the *along-track* axis (cross-track is already ~2 cm from tracking):

- **Option A — feed-forward decel (recommended first).** From current speed `v` and the brake decel `a` (bounded by `segment_brake_velocity_cap_m_s`), begin braking at distance `d = v²/2a` **before** the point so `v→0` *at* the coordinate. Open-loop, no new closed loop, reuses the brake primitive with a computed trigger point instead of a fixed radius.
- **Option B — low-speed position servo (fallback if A misses in field).** After a coarse stop, creep toward the point at `precise_stop_creep_speed` (e.g. 0.05 m/s) with small along-track corrections until `|error| ≤ point_arrival_tolerance_m` (0.02), bounded by `precise_stop_max_s`. Closed-loop, tighter, slower.

Both gated by `point_precise_stop_enabled` (default False → today's brake-when-near). `point_arrival_tolerance_m` default **0.02**. Nozzle-from-antenna offset (currently 0 / unmeasured) must be applied so the *dot*, not the antenna, lands on the point — tracked as a prerequisite, not part of this doc.

Consistency note: this is a **new primitive** used only for must-hit points under point mode; corner stops keep the proven `_corner_stop_satisfied` path unchanged.

---

## 8. Auto vs manual execution

- Carry `point_execution_mode` (`auto|manual`) through the pipeline it currently dies in: `PathPlanRequest → spray_session (staged) → /rpp/…`. Two options for delivery to RPP: (a) RPP param `point_execution_mode` set at load, or (b) a field on the point session — **(a) is simpler and matches how RPP already takes params**; the server sets it on load.
- **Auto:** RPP advances on `/spray/point_done` (or the `point_hold_max_s` backstop).
- **Manual:** RPP enters `WAIT_OPERATOR` after `point_done`, advances only on `/point/advance`. Server exposes `POST /api/spray/point/advance`; frontend shows a "Next point" button while `/rpp/progress.phase == WAIT_OPERATOR`. The existing `/api/spray/status` (now carrying `mode` + point block) already surfaces the phase for the button's visibility.

---

## 9. What's already there vs. new

| Piece | Status |
|---|---|
| RPP publishes a state enum (`/rpp/segment_debug`) | **Exists** — extend to `/rpp/progress` |
| Spray consumes an RPP state as a gate (pivot) | **Exists** — generalize |
| `/path` carries MARK/PRE/AFT + must-hit (bit0/bit1) | **Exists** — RPP projects & announces |
| Point coords from placed `/path` must-hit | **Exists** (`67ef615`) |
| `/api/spray/status` reports mode + point config | **Exists** (`0cd284a`) |
| `MissionPhase` enum + `/rpp/progress` + `/rpp/milestone` | New |
| `/spray/point_done`, `/point/advance` | New |
| Precise 2 cm stop primitive | New |
| Manual gate + `point_execution_mode` plumbing + `/api/spray/point/advance` + button | New |
| Spray `consume_rpp_progress` boundary sourcing | New |

---

## 10. Parameters (all default to today's behavior)

RPP (`rpp_controller`):
- `progress_publish_enabled` (False) — publish `/rpp/progress` + `/rpp/milestone`
- `point_precise_stop_enabled` (False), `point_arrival_tolerance_m` (0.02)
- `precise_stop_mode` (`feedforward|servo`), `precise_stop_creep_speed` (0.05), `precise_stop_max_s` (8.0)
- `point_execution_mode` (`auto`), `manual_wait_timeout_s` (0.0 = forever)
- `point_hold_max_s` (backstop cap)

Spray (`spray_controller`):
- `consume_rpp_progress` (False), `progress_timeout_s` (0.3 → fallback to `/path`)

Server:
- stage `point_execution_mode`; `POST /api/spray/point/advance` → `/point/advance`

---

## 11. Phasing (each phase independently shippable, default-OFF)

- **G1 — Progress publication.** RPP publishes `/rpp/progress` + `/rpp/milestone` behind `progress_publish_enabled`. No consumer yet. Pure observability; zero behavior change. *(RPP additive)*
- **G2 — Spray boundary sourcing.** Spray consumes progress for continuous/dash lead behind `consume_rpp_progress`, with `/path` fallback. A/B vs today's projection on the same mission. *(spray additive)*
- **G3 — Precise stop.** `point_precise_stop_enabled` feed-forward decel; A/B stop-accuracy vs brake-when-near. *(RPP, frozen A/B)*
- **G4 — Point handshake.** `AT_POINT → dwell → point_done → advance` (auto). Replaces the fixed-timer coordination. *(both nodes)*
- **G5 — Manual gate.** `WAIT_OPERATOR` + `/point/advance` + server route + frontend button. *(RPP + server + app)*
- **G6 — Field validation.** A/B every flag at the rover; measure stop accuracy, boundary accuracy, handshake latency.

Each phase merges to `baseline_master` only after its own A/B; the frozen continuous path stays the fallback until G2 is field-proven at least as good.

---

## 12. Robustness / failure matrix

| Failure | Behavior |
|---|---|
| `/rpp/progress` stale | spray falls back to `/path` geometry (no strand) |
| `/rpp/milestone AT_POINT` dropped | RELIABLE + `seq` gap → re-sync from progress phase |
| `/spray/point_done` never arrives | RPP advances after `point_hold_max_s`, warns |
| `/point/advance` lost (manual) | RPP holds (safe); optional `manual_wait_timeout_s` |
| RPP progress glitch/wrong phase | spray fallback + xtrack gate still independent |
| Precise stop can't reach 2 cm | `precise_stop_max_s` timeout → dwell at best position, warn |
| Node restart mid-mission | VOLATILE command topics never replay stale advance; progress re-latches from live state |

---

## 13. Test plan

- **Unit (pure/stubbed, Mac):** phase-transition table; `dist_to_next_boundary` math; feed-forward decel trigger distance; handshake sequencer (auto + manual); `seq`-gap re-sync; fallback-on-stale.
- **Bench in-env (Jetson, disarmed):** publish a placed `/path`, drive the state machine by injecting pose, assert `/rpp/progress`/`/rpp/milestone`/`/spray/point_done` sequencing and `/api/spray/status` phase; confirm default-OFF = byte-for-byte frozen (regression smoke + segment_stop + corner_pivot green).
- **Field A/B (G6):** stop accuracy (RTK truth vs commanded point), continuous boundary accuracy G2-on vs G2-off, handshake dwell timing, manual button round-trip.

---

## 14. Open questions

1. ~~`MissionPhase` numeric values — append after `CORNER_STOP=5` in one shared enum, or a separate enum on a separate topic?~~ **RESOLVED 2026-07-23: separate `MissionPhase` enum on a separate `/rpp/progress` topic.** Decisive reason — tracking-state (`SegmentStateCode`) and mission-phase are **orthogonal, co-occurring** axes (e.g. `MARK_TRACKING ∧ PRE_CORNER_SLOWDOWN` hold simultaneously); one enum has one slot and cannot encode both without losing information. Separation also makes the frozen contract *structural*: `/rpp/segment_debug` is provably untouched because it is never edited. The only cost — two channels skewing — is neutralized by design (progress is BEST_EFFORT, republished every 50 Hz tick; correctness-critical events ride the RELIABLE `/rpp/milestone`).
2. Feed-forward vs servo as the G3 default — start feed-forward; keep servo as the tighter fallback.
3. Manual mode: per-mission or global param? (Leaning: staged per-mission, applied as an RPP param at load.)
4. Nozzle offset: must be measured before 2 cm point marking is meaningful — separate task, hard prerequisite for G6.
5. Does dash need MARK-region envelope events, or is arc-length self-sufficient? (Leaning: keep DashMeter; RPP only supplies the MARK envelope so dash resets correctly at region edges.)

---

## 15. Verdict recap

The operator's flow is **consistent** (it generalizes the shipped pivot-gate pattern) and **robust in this form** — authoritative progress that carries *distance-to-boundary*, used by spray as a *confirming gate layered over its own geometry*, with the handshake reserved for stops. It would be **not** robust as a naive "RPP fires discrete arrival events, spray blindly obeys," because that regresses the field-proven moving-mark anticipation. This design keeps the anticipation, adds the handshake, adds the 2 cm stop and the manual gate, and ships entirely behind default-OFF A/B flags.
