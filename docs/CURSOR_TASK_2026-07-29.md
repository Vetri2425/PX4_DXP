# CURSOR EXECUTION TASK — Production-Readiness Fixes (2026-07-29)

You are working on a **real autonomous ground vehicle that carries a paint sprayer**. Code you write
here drives a 100 kg rover in OFFBOARD mode. There is no simulator in this loop. Treat every change to
`src/` as safety-relevant.

**Your job:** investigate, verify, and implement the fix list below. **Do not deploy. Do not push. Do
not touch the Jetson.** A separate reviewer handles review and deployment.

---

## 0. Read these first, in this order

1. `docs/PRODUCTION_READINESS_AUDIT_2026-07-29.md` — the audit you are executing. Every finding ID
   (P1, S1, R3…) below is defined there with rationale.
2. `CLAUDE.md` — project hard rules. **Non-negotiable.**
3. `docs/PROJECT_MATURITY_AUDIT_2026-07-26.md` — prior audit, for context on what is already known.

---

## 1. Hard constraints — violating any of these fails the task

| Rule | Why |
|---|---|
| **Branch off `baseline_master`; never commit to it** | It is the trusted, field-validated baseline. Use `git checkout -b prod/audit-2026-07-29`. |
| **Never `git push`** | The reviewer pushes after review. |
| **Never SSH to `192.168.1.102`, never run `deploy.sh`, never restart a service** | Deployment is out of scope and unsafe from here. |
| **Never edit PX4 firmware or push FCU params** | QGC on the Mac is the source of truth for params. |
| **Never propose an ArduRover solution** | ArduRover is abandoned on this project. |
| **Do not "improve" `PX4_EARTH_RADIUS_M = 6371000.0`** in `path_engine/ned.py` | It is a frame definition matched bit-for-bit to PX4's EKF, not a physical radius. |
| **All 1 220 off-robot tests must pass after every commit** | Command in §3. A red suite means stop and fix, not proceed. |

### DO NOT TOUCH — these are closed and field-validated

- `path_engine/planners/arc_chain.py`, `extensions.py`, `corner_fillet.py`, `smooth.py`
- `path_engine/optimizers/` (both files)
- `src/twist_to_setpoint_node.py` — holds the 50 Hz PX4 OFFBOARD heartbeat contract
- `rpp_controller_node._project_onto_path` (~line 2867) — the windowed O(1) search is correct
- Any RPP tuning parameter value (`segment_*`, `corner_*`, `max_yaw_rate_body`, `a_lat_max`)
- The `run-out = 0.10` floor at `path_engine/engine.py:~1284` — it was set to 0.05 once and caused a
  terminal heading flip in the field. The comment explains why. Leave it.

If a fix below appears to require touching one of these, **stop and write down why** in your report
instead of doing it.

---

## 2. Environment

```bash
# Mac dev. No ROS2 here — rclpy is unavailable and that is expected.
PY=.venv-dev/bin/python

# path_engine — pure, no PYTHONPATH needed
$PY -m pytest path_engine/tests -q                     # expect: 475 passed

# server — needs itself on the path
cd server && PYTHONPATH="$PWD:$PWD/.." ../$PY -m pytest . -q     # expect: 370 passed

# src — 15 files need rclpy and CANNOT run here. Ignore exactly these:
cd src && PYTHONPATH="$PWD:$PWD/.." ../$PY -m pytest . -q \
  --ignore=offboard_test.py --ignore=spin_in_place_test.py \
  --ignore=test_completion_stop.py --ignore=test_corner_absorb_pivot.py \
  --ignore=test_corner_stop_brake.py --ignore=test_ekf_reset_compensation.py \
  --ignore=test_endpoint_approach.py --ignore=test_entry_prealign.py \
  --ignore=test_point_handshake_rpp.py --ignore=test_point_handshake_spray.py \
  --ignore=test_point_hold_rpp.py --ignore=test_precise_stop_node.py \
  --ignore=test_progress_publication.py --ignore=test_smoke_rpp_controller.py \
  --ignore=test_spray_flag_conditioning.py                       # expect: 375 passed
```

**Those 15 ignored files are the safety-critical ones** (corner pivot, stop brake, endpoint approach,
point handshake, spray flag conditioning). You cannot run them. Therefore: **if a change you make
could plausibly affect one of them, say so explicitly in your report** so the reviewer runs it on the
robot. Do not guess.

---

## 3. Method — mandatory, in stages

Work in **four stages**. Each stage ends with: tests green → one commit → a written report block.
**Do not start a stage before the previous one is committed and green.**

Commit message format:
```
<stage>: <finding IDs> — <what>

<why, one paragraph>

Findings: R5, R4
Tests: path_engine 475, server 370, src 375 — all pass
Field-verification owed: <yes/no + what>
```

---

## STAGE 0 — Verify before you change anything

**The audit's line numbers may have drifted. Verify every one. Report drift; do not silently adapt.**

For each finding, open the cited location and confirm the described code is actually there. Produce a
table:

| ID | Cited location | Found at | Code matches description? |
|---|---|---|---|

**If any finding does not reproduce, STOP and report it.** Do not implement a fix for a problem you
could not confirm. A finding that has already been fixed is a good outcome — record it and move on.

Findings to verify:

- **R3** `src/spray_controller_node.py:644` — `declare_parameter("max_xtrack_error_m", 0.10)`;
  enforced at `:446` (`if projection.xtrack_error_m > max_xtrack_error_m: safety_ok = False`)
- **R4** `src/spray_modes.py:135` — `self.s_dash = raw_s; self.s_at_last_toggle = raw_s` on arm
- **R5** `src/spray_controller_node.py:445` sets `geometry_desired = projection.current_flag`;
  `:466` overwrites it with `du.geometry_desired`; `:535` is
  `desired = bool(geometry_desired and safety_ok)` with no AND against `current_flag`
- **R6** the B4 terminal-shutoff block at `:521` is inside the `else:` opened at `:472`
  (continuous-only), so the dash branch at `:452-471` never reaches it
- **S1** `rover-server.service:9` is `Type=simple`, no `WatchdogSec=` anywhere in the file;
  `server/main.py:201` sends `READY=1`, `:578` sends `WATCHDOG=1`
- **S2** `server/main.py:323` `_emit_authenticated` awaits `sio.emit` with no timeout;
  `:338` `_telemetry_loop` also carries the E-stop at `:521`
- **S3** `rover-server.service:49` `Environment=ROVER_DISABLE_AUTH=1`
- **S4** `server/routes/path.py:958` and `:979` — unbounded `await file.read()`
- **R1** `server/ros_node.py:253` — one `MutuallyExclusiveCallbackGroup` shared by ~10 subscriptions;
  `/mavros/state` correctly has its own at `:254`
- **R2** `src/rpp_controller_node.py:3335` and `:3915` — `max_accel / self.CONTROL_HZ`;
  `:4524` `rclpy.spin(node)`
- **P1** `path_engine/engine.py:822` — rounding-bucket dedup key

Commit Stage 0 as a report only if you changed nothing; otherwise fold it into Stage 1's report.

---

## STAGE 1 — Ship-blockers (highest value, do these first)

### R5 — Dash paints TRANSIT connectors *(1 line)*

`src/spray_controller_node.py:~466`

```python
geometry_desired = du.geometry_desired and projection.current_flag
```

The meter must keep integrating `s` through the transit (so the pattern phase does not drift — that is
the locked "continuous across mission" decision), but the valve must stay shut. **Add a test** proving
a dash session over a path with a TRANSIT run never returns `desired=True` inside that run.

### R6 — Dash has no terminal shutoff *(indentation)*

Hoist the B4 block at `:521` out of the `else:` at `:472` so it runs for both continuous and dash. Its
own comment says it "fires independently of any boundary/lead", so it is already written to be
mode-agnostic. **Add a test:** dash session, rover stops within `terminal_off_epsilon_m` of the last
station on an "on" phase → `desired` goes False.

### R4 — Dash pattern anchor is non-deterministic *(~10 lines)*

`src/spray_modes.py` — `DashMeter` currently anchors `s_at_last_toggle` to whatever `raw_s` is on the
first `xtrack_ok` tick, so the whole pattern shifts between runs of the same mission.

Add an optional `anchor_s: float | None = None` constructor arg. On arm:

```python
self.s_dash = raw_s
self.s_at_last_toggle = self._anchor_s if self._anchor_s is not None else raw_s
```

The node passes the **first `MARK_START` station** from the path model. `None` preserves today's
behaviour exactly, so every existing dash test stays valid.

**Test the property, not the implementation:** arm the same meter twice at two different `raw_s`
values with the same `anchor_s`, and assert the toggle stations are identical.

### R3 — xtrack gate is 5× looser than the ±2 cm spec

Two parts.

**(a)** `src/spray_controller_node.py:644` — default `0.10` → **`0.03`**.

**(b)** Make it settable per mission via `SpraySessionConfig`.

> ⚠️ **DEPLOY TRAP — read this before touching `spray_session_config.py`.**
> `src/spray_session_config.py:220` matches `schema_version` **exactly** and raises `ConfigSchemaError`
> on mismatch, and the node's documented contract on a config it cannot parse is **fail static — keep
> the last-known-good mode.** So bumping `SCHEMA_VERSION` means a new server talking to a
> not-yet-updated node causes the node to **silently retain the previous mission's spray mode.** That is
> a mid-mission wrong-paint failure with no error surfaced.
>
> **Therefore: do NOT bump `SCHEMA_VERSION`.** Add `max_xtrack_error_m` as an **additive optional
> top-level key** that defaults to `None` when absent, and have the node fall back to its ROS param
> when it is `None`. Then old-node/new-server and new-node/old-server both work with no lockstep
> deploy.
>
> **First, write a test proving the current parser tolerates an unknown top-level key.** If it does
> not, stop and report — the additive approach is unavailable and this needs a design decision.

Expect more spray refusals after this change. **That refusal is the product working** — do not widen
the gate to make a test pass.

### S1 — systemd watchdog is dead code *(2 lines)*

`rover-server.service`: `Type=simple` → `Type=notify`, and add `WatchdogSec=15`.

`server/main.py` already sends `READY=1` and `WATCHDOG=1` every ~3 s, so no Python change is needed.

The dependency is safe: **`sdnotify==0.3.2` is already pinned at `server/requirements.txt:9`** (verified
2026-07-29). This matters because if that import ever fails, `_sd_notifier` is `None` (`main.py:60-66`),
the notifications silently never send, and `Type=notify` would make systemd **kill the service at
startup timeout**. Re-confirm the line is still there before you make the unit-file change; if it has
gone, restore it rather than reverting to `Type=simple`.

### S3 — auth disabled in production *(1 line)*

Delete `Environment=ROVER_DISABLE_AUTH=1` from `rover-server.service:49`.

Then **verify the bootstrap path**: read `server/auth.py` and `docs/ROVER_LOCAL_AUTH.md` and confirm
what happens on a fresh boot with no password configured. If enabling auth locks the operator out of
their own rover with no recovery, **do not make this change** — report the blocker instead. Getting
locked out in the field is worse than the vulnerability.

---

## STAGE 2 — Structural

### S2 — bound the emit; split the safety watchdog

`server/main.py`. Two changes:

1. `_emit_authenticated` (`:323`) — wrap each emit:
   `await asyncio.wait_for(sio.emit(event, data, to=sid), timeout=0.5)`. On timeout, log at warning
   and continue to the next SID — never abandon the tick.
2. Extract the E-stop watchdog (`:485-564`) into its **own** `asyncio.Task`. It must call only
   `ros_node.get_state()` and `emergency_handler.estop_async()` and **must never touch Socket.IO**.
   The `safety_abort` event emit stays in the telemetry loop, fed by a flag or queue.

Rationale: today a phone leaving WiFi with a full TCP buffer can stall the emit await, and the E-stop
watchdog stops running while the rover drives. Keep both tasks cancelled cleanly in the shutdown path.

**Test:** a stalled emit (mock that never resolves) must not prevent the watchdog task from firing an
estop.

### R1 — callback groups

`server/ros_node.py`. Add two groups alongside the existing `_state_sub_group`:

```python
self._pose_group = MutuallyExclusiveCallbackGroup()   # /mavros/local_position/pose
self._rpp_group  = MutuallyExclusiveCallbackGroup()   # /rpp/debug
```

**Do not change `num_threads`, and do not touch `_svc_group`** — the `ReentrantCallbackGroup` for
service clients is what guarantees arm/disarm responsiveness. The precedent and the reasoning are
already written at `:247-256` for `/mavros/state`; follow it.

`src/rpp_controller_node.py`: give `_path_cb` its own `MutuallyExclusiveCallbackGroup` and change
`main()` (`:4524`) from `rclpy.spin(node)` to a `MultiThreadedExecutor(num_threads=2)`.

**Flag for on-robot verification:** this changes concurrency on the live control node. Mark it clearly
in your report — the reviewer must run the 15 rclpy tests plus a drive test before this reaches a
mission.

### R2 — measured `dt` in the accel ramps

`src/rpp_controller_node.py:3335` and `:3915`. Replace the assumed `1/CONTROL_HZ` with a clock-measured
delta, clamped so a long stall cannot produce a huge step:

```python
now = self.get_clock().now()
dt = min(0.1, (now - self._last_tick).nanoseconds * 1e-9) if self._last_tick else 1.0 / self.CONTROL_HZ
self._last_tick = now
```

Use `self.get_clock()` (**not** `time.monotonic()`) — the codebase deliberately uses ROS time so bag
replay works; there is a note about this at `src/mission_runner_node.py:420`.

**This changes vehicle acceleration behaviour. Mark it as field-verification-owed.**

### R7 — make the 15 rclpy tests runnable on demand

Add a `Makefile` target (or `tools/run_jetson_tests.sh`) that runs the full `src/` suite **with** rclpy,
writes a JSON result file, and returns non-zero on failure. **Write the runner only — do not execute it
against the Jetson.** The reviewer wires up how it gets invoked.

---

## STAGE 3 — Hardening

### P1 — distance-based dedup

`path_engine/engine.py:~822`. Replace the rounding-bucket key with an explicit endpoint-distance test
over MARK segments (O(n²) is fine — n is tens). Two segments are duplicates iff their endpoint pairs
match within `tol` in either direction **and** their lengths match within `tol`.

Add a `allow_duplicate_drop: bool = True` engine arg. When `False`, a detected duplicate raises instead
of dropping. Keep `duplicate_stats` in `planning_metadata` exactly as it is — the app reads it.

**Tests:** (a) two lines 4 cm apart with equal length are **both kept**; (b) two truly coincident lines
straddling a 5 cm bucket boundary are correctly deduped.

### S4 — size caps on two routes

`server/routes/path.py:958` and `:979`. Copy the exact pattern already used at `:995` and `:1036`
(`read(MAX_UPLOAD_BYTES + 1)` → 413). Do not invent a new pattern.

### S5 — planning concurrency bound

A module-level `asyncio.Semaphore(2)` in `server/routes/path.py` around the `plan_segments` /
`path_mgr.plan_path` calls. Keep it inside the existing `wait_for` so a queued request still times out
cleanly rather than hanging.

### P2 — extract the pipeline steps *(do this LAST, and only if Stages 0–3 are green and committed)*

`path_engine/engine.py:532-1394`. `_plan_from_segments` is 860 lines / 12 order-dependent passes.

Extract each numbered step to a module-level `def _step_N_<name>(segments, cfg, ...) -> list[PathSegment]`
with the bodies moved **byte-for-byte** — no logic edits, no "while I'm here" cleanups. `_plan_from_segments`
becomes a ~30-line ordered pipeline.

**Acceptance is binary: all 475 path_engine tests pass with zero modifications to any test file.**
If you find yourself editing a test to make this pass, you changed behaviour — revert and try again.

**Preserve every explanatory comment.** Those comments encode field failures that cost real money
("Step 3 withholds transits *because* Step 4…", the d82317d 180°-double-back, the E1–E5 defects). Losing
them is worse than not doing the refactor. If a comment explains an ordering constraint, it moves with
the step **and** gets echoed at the call site.

If this turns out to be riskier than it looks, **stop and leave it undone.** It is the lowest-value item
on the list. A half-finished refactor of this method is far worse than none.

---

## 4. Report format — produce this at the end

```markdown
# Cursor Execution Report — 2026-07-29

## Stage 0 — verification
<the drift table; findings that did not reproduce>

## Implemented
| ID | Files | Commit | Tests added | Field-verification owed? |

## NOT implemented, and why
<blockers, design decisions needed, things I judged too risky>

## Requires on-robot verification before it drives
<explicit list — R1, R2, R3 at minimum>

## Test results
path_engine: N passed | server: N passed | src (non-rclpy): N passed
Could not run: 15 rclpy files — <which of my changes could affect them>

## Open questions for the reviewer
```

---

## 5. Judgement calls — read this before you start

You will hit things this document did not anticipate. When you do:

- **Prefer stopping and reporting over guessing.** An unimplemented item with a clear explanation is a
  good outcome. A wrong implementation of a spray gate is a ruined road.
- **Never widen a safety gate to make a test pass.** If a test fails after R3, the test encoded the old
  loose tolerance — fix the test, and say so.
- **Never delete a comment you do not understand.** In this repo the comments are the field logbook.
- **If a fix requires touching the DO-NOT-TOUCH list, stop.** Report why. There is a reason each entry
  is on it.
- The audit is a starting point, not scripture. **If you find something worse than what is listed here,
  say so** — that is more valuable than completing the list.
