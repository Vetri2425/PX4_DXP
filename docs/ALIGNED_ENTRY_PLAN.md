# Aligned Runtime Entry — Plan (E-series)

**Status:** **DISABLED IN THE FIELD 2026-08-03 AFTER AN ACCIDENT — do not re-enable
without the §8 redesign.** E1 implemented 08-02 (`099a9a2` tree, server suite 405
green); first field run 08-03 (bag `stg_553075d4_132652`): geometry executed exactly
as designed — and that was the problem.

## 8. FIELD FAILURE 2026-08-03 — design flaw, not an implementation bug

The rover was parked **down-line of the mission start** (on the mark's side). The
staging detour therefore ran **past wp0 into ground beyond the line start** — ground
the old 2-pt entry never enters. The operator (correctly) read "it missed the stop
point", e-stopped 1 s before the staging corner stop (bag shows dist-to-corner
0.001 m at e-stop), and the rover had a **small accident in the unplanned corridor**.

Root design flaw: §5 risk 1 claimed the staging corridor is "the same ground the
rover was about to drive anyway." **False when parked down-line** — exactly the
anti-parallel case the feature was built for. The maneuver needs ground clearance
the system cannot see and the operator was never asked about.

Disabled in source by default (`ROVER_ENTRY_STAGING` defaults to `0`) and also
disabled by the existing systemd drop-in
`/etc/systemd/system/rover-server.service.d/entry-staging.conf`. Code stays in
tree for controlled testing; re-enable only with an explicit
`ROVER_ENTRY_STAGING=1` override after the plan's field gate passes.

**Re-enable requires ALL of:**
1. App draws the full entry route (including the staging detour) BEFORE start, and
   the operator explicitly confirms it — the detour must never be a surprise.
2. A conservative auto-guard: staging only when the detour corridor stays within
   ground the mission already traverses (e.g. within the staged path's bounding
   hull + margin); otherwise fall back to the plain chord and accept the pivot.
3. E3 gates re-run from scratch.
**Owner surface:** `server/offboard_controller.py::start_async` ONLY (frontend trajectory,
path engine, and RPP are untouched)
**Attacks:** register **D1** (pivot walk at the mission start) and **D2** (approach-stop
scatter) at their **origin**, instead of compensating downstream.
**Supersedes nothing.** `RUNTIME_ENTRY_VELOCITY_PLAN.md` (D0–D3) remains the base — this
is a geometry upgrade to its phase-1 path, reusing every mechanism D1–D3 landed.

---

## 1. Problem (verified on 12 With_EXT bags, 2026-08-01)

The phase-1 entry path is `[live_pose, staged_path[0]]` — a straight chord whose direction
is "wherever the rover parked → mission start". It ignores the mark's own direction.

Measured consequences (independent decode, all 12 bags):

| Fact | Value |
|---|---|
| Heading error at phase-1 start (no entry pre-align) | 59–159° |
| Entry-chord tracking (honest, perpendicular) | RMS 0.96–2.20 cm, max 3.1–5.7 cm — fine |
| Entry stop vs target | 0.6–2.8 cm, almost always short — fine |
| Handoff error entry-end → mark[0] | 0.0000 cm, 12/12 — fine |
| **Arrival heading vs mark direction** | **~140–180° wrong** |
| **Phase-2 pre-align pivot at the mission start** | **7.3–12.7 s** |
| **Pivot walk dragged onto the start point** | **0.6–4.8 cm (D1)** |

The entry *tracking* is not the problem. The entry *ends pointing the wrong way at the
single most accuracy-critical point of the mission*, and the forced ~180° pivot there
seeds the paint-entry error that the 0.5 m PRE then has to absorb.

(Related, refuted during verification: third-party claim of 11–18 cm entry cross-track
peaks was an along-track/cross-track mixing error; true peaks are 3–6 cm.)

## 2. Design — 3 points instead of 2

```
today:    parked ───────────────────────────► mark[0]   (arrive ~180° wrong → pivot HERE)

planned:  parked ─────────► staging ─══════► mark[0]    (last leg collinear with the mark)
                             │  d ≈ 1.2 m behind mark[0], along −u
                             └─ any sharp turn happens HERE, in free space
u = unit(staged_path[1] − staged_path[0])   — the PRE/mark direction, already in hand
staging = staged_path[0] − d·u
```

- **Entry publish becomes** `[live, staging, mark[0]]`, `spray=[F,F,F]`, `must_hit=[F,F,F]`.
- If the turn at `staging` is ≥ `segment_corner_threshold_deg` (45), RPP runs its **existing**
  corner stop-pivot there — 1.2 m out, where a 2–4 cm walk lands in free space and the
  aligned final leg converges it before arrival (convergence length ≈ 2–3·Ld ≈ 0.7–1.0 m
  at the new lookahead config — the leg is sized to cover it).
- Rover arrives at `mark[0]` **already collinear with the mark** → phase-2 D2 pre-align sees
  a few degrees instead of ~180° → no pivot walk at the start point.
- Everything else is reuse: `publish_path` (any point count), D3 completion latch,
  DONE detection, `advance_entry_to_marking`, the stashed marking path.

### Degenerate cases (all decided at build time, ~5 lines)

| Case | Behaviour |
|---|---|
| `dist(live, mark[0]) ≤ ENTRY_SKIP_DIST_M` | unchanged: no entry leg at all (existing skip) |
| Rover already within ~20° of aligned AND roughly behind `mark[0]` | 2-pt entry as today (staging adds nothing) |
| `dist(live, staging) < ~0.5 m` | drop `live→staging` leg; entry = `[live, mark[0]]` but live is near the aligned corridor anyway |
| Turn at staging < 45° | no pivot — RPP drives it as a smooth kink (already supported) |

### Parameters (server-side, must land in BOTH node config and `routes/rpp_params.py`-style registry if exposed)

| Param | Default | Meaning |
|---|---|---|
| `entry_staging_dist_m` | 1.2 | how far behind the mission start the staging point sits (≥ 2–3·Ld) |
| `entry_staging_enabled` | true | field off-switch, same convention as `endpoint_approach_run_remaining` |

## 3. What does NOT change

- Frontend trajectory / staged mission JSON — byte-identical.
- Path engine, extensions, spray flags, must-hit semantics.
- RPP controller — zero changes; the 3-pt path exercises only field-proven code paths
  (multi-segment transit run, corner stop-pivot, completion latch).
- The two-phase state machine (`ENTRY → advance_entry_to_marking → marking`).

## 4. Phases & gates

**E1 — Implement + unit tests (Mac, ~1–2 h).**
Build the 3-pt list in `start_async`; degenerate guards; log line states staging geometry.
Tests: staging placement math (all four quadrants of parked position), each degenerate
case, flags/must_hit lengths, off-switch. Server suite stays green.

**E2 — Bench check on the Jetson (no driving, ~10 min).**
Stage the standard line from a parked pose behind/left/right of the start; assert the
published `/path` has 3 points with the staging point at `mark[0] − 1.2·u`; assert entry
skip still fires when parked on the start.

**E3 — Field A/B (n=3 vs n=3, same mission, same speed 0.5/0.6, ~30 min).**

| Gate | Today (12-bag baseline) | PASS |
|---|---|---|
| E3.1 phase-2 pre-align pivot angle | ~140–180° | **≤ 30°** |
| E3.2 phase-2 pre-align duration | 7.3–12.7 s | **≤ 3 s** |
| E3.3 pivot walk at mark[0] | 0.6–4.8 cm | **≤ 1 cm** |
| E3.4 spray-ON vs surveyed stake | 0.33–4.63 cm | **≤ 1.5 cm every run** |
| E3.5 mark RMS @0.6 | 0.66–0.98 cm | **≤ 1.0 cm, no regression** |
| E3.6 total entry+align time | entry + 7–13 s pivot | not worse than today |

Rollback = `entry_staging_enabled=false` (behaviour bit-identical to today), or revert the
single commit.

## 5. Risks (named, none structural)

1. **Staging point lands somewhere inconvenient** (obstacle, wet paint from a previous
   line). There is no obstacle map; the operator sees the entry leg in the app already.
   Mitigation: `d` small (1.2 m), off-switch, and the staging point is always on the
   mark's own approach corridor — the same ground the rover was about to drive anyway.
2. **Corner stop at staging adds a stop** (~3–5 s). It *replaces* the 7–13 s pivot at the
   start point — net time is expected to improve (E3.6 guards it).
3. **`_split_runs_by_flag` interaction** — entry is one all-OFF run with an interior
   vertex; the vertex is a corner, not a run boundary. E1 unit test pins this.
4. **Anti-parallel parking** (rover parked exactly on the far side): turn at staging
   approaches 180° — the pivot walk happens there, in free space, by design. Not a risk,
   the point of the design; noted so nobody "fixes" it.

## 6. Why not the alternatives

- **Pre-align on phase-1 start only** (pivot to the chord before driving): removes the
  59–159° initial swing but the rover still *arrives* ~180° wrong — the critical pivot
  stays at the critical point. Worth adding later as polish, not as the fix.
- **Densify the entry chord**: changes nothing — the defect is where the line ends and
  which way it points, not its point count.
- **Dubins/arc entry planner**: strictly more code for the same arrival state; the
  segment profile + corner pivot already implements the only maneuver needed.

## 7. Effort

Core change ~10 lines + ~60 lines of tests. One bench session. One 30-min field A/B.
Everything downstream (D3 latch, advance, spray safety) is already field-proven.
