# Field Checklist — Dash Run, 2026-07-29

**Branch under test:** `prod/audit-stage3b` @ `36352a4` (deployed, cold-boot verified)
**What this proves:** 10 spray/control fixes that have never painted a line.
**Window:** ~16:20 → 18:30

**Roles.** You are at the rover with the e-stop. I drive the API from the laptop and watch telemetry.
**I will not arm or command motion without you saying so, for each run.** Say "start" for each one.

---

## Mission requirement — pick the right file

The mission MUST have **at least two separate marked runs with a transit connector between them**
(two parallel lines, or a shape with disconnected edges). Without a connector, R5 is untestable.

Ideal: **two parallel straight lines, 3–5 m each, ≥1 m apart**, dash mode.
Straight lines make the failures measurable with a tape; curves hide them.

---

## GO / NO-GO gates — all four before any motion

| # | Gate | How | Abort if |
|---|---|---|---|
| 1 | **RTK FIXED** | Start NTRIP from the app (`/api/rtk/ntrip/start`). I'll confirm `fix_type=6`. | Stays FLOAT/3D — accuracy claims are meaningless below FIXED |
| 2 | **App login** | Log in with the Jul-11 password. Telemetry must stream. | Client can't auth — I roll auth back, we continue, and the app dev gets a bug |
| 3 | **EKF origin** | I check `/api/health/origin` | `INCONSISTENT`, or delta > 5 cm |
| 4 | **Spray disarmed-safe** | Valve closed, tank state known | Valve reads ON at rest |

---

## Phase 1 — DRY RUN (no paint in the tank) ⚠️ mandatory first

Project rule: the first live run after a spray-path change is dry. Ten fixes changed that path today.

Run the full mission with an empty tank. I watch `/spray/state` transitions and log them.

**Pass =** valve opens on marked runs, **stays shut across the connector**, closes at the end.
**If the valve opens on the connector, STOP.** That is R5 failing and paint would be on the ground.

---

## Phase 2 — WET RUN #1

Fill, run the same mission. You watch the ground; I watch telemetry.

| Check | Fix | What good looks like |
|---|---|---|
| Paint starts at/just before the line start | **B2** | ≤ ~2 cm before. **Late is a fail** — that was the old bug |
| Connectors dry | **R5** | zero paint between runs |
| Dashes regular | **R4** | on/off lengths match the configured pattern |
| Valve closes at the end | **R6** | no puddle past the last station |
| Accel at start | **R2** | smooth; no jerk or crawl in the first 2 m |

**Measure and write down:** distance from line start to first paint, and the first three dash
on/off lengths. Tape, not eyeball.

---

## Phase 3 — WET RUN #2 — ⭐ THE CRITICAL TEST

**Re-run the same mission. Do NOT move the rover between runs. Do NOT re-stage.**

This is the only way to test R4 + B1, and it is the single most important measurement today.

**Pass =** run #2's dashes land **on top of** run #1's.
**Fail =** they are offset by any visible amount → the geometry anchor is not working.

If they overprint, R4 and B1 are proven and the dash mode is genuinely repeatable — which is what a
customer re-marking a road actually needs.

---

## Phase 4 — STOP-SHORT (drop this first if time is tight)

Start the mission, then stop it partway down a marked run.

**Pass =** valve closes within ~5 cm. **Fail =** it keeps spraying → R6's terminal shutoff isn't
firing on the stop path.

---

## Phase 5 — Bags + verdict (I do this)

Pull bags, run the mission-analysis method, and report against production-grade criteria:
cross-track RMS, paint-start offset, dash pitch consistency run-to-run, connector cleanliness,
terminal overrun.

---

## What I'm watching live, and what makes me call a stop

| Signal | Stop if |
|---|---|
| `xtrack_m` | > 5 cm sustained |
| `rpp_state` | `STALE`, `RTK_WAIT` or `JUMP_SKIP` mid-run |
| `safety_reason` | anything unexpected — I'll read it to you verbatim |
| `/spray/state` | ON during a connector or after mission end |
| `gps_fix` | drops below RTK_FIXED |

**Expect more spray refusals than you're used to** — the gate went 10 cm → 3 cm today. A refusal on a
good fix is the product working. Refusals *constantly* on a good fix means 3 cm is too tight and we
tune it — that's a data outcome, not a bug.

---

## If time runs short

Priority order. **Phase 1 and Phase 3 are the must-haves.**

1. Dry run (safety, non-negotiable)
2. Wet #1 + Wet #2 overprint ⭐
3. Stop-short
4. P1 DXF pass — only if a suitable CAD file is to hand

---

## Not being tested today

- ~~`POST /api/path/plan-trajectory`~~ — **field-validated 2026-07-28**, no longer outstanding
- Point missions — bug A11 still open
- Joystick — re-enabled 16:35; J3 axis-mapping bench test still owed (see below)
