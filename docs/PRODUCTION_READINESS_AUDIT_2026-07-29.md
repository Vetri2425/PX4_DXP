# Production-Readiness Code Audit — 2026-07-29

**Scope:** `path_engine/` (7 524 lines) · `server/` (13 860) · `src/` ROS2 nodes (12 754) · systemd units
**Branch:** `baseline_master` @ `6fe6c76`
**Question asked:** *"The pre-line product is almost at end. We make this one a robust product, no more prototype. Honest score out of 100."*
**Method:** full read of every non-test source file in the three layers, plus every test suite executed.

> This audit scores the **code**. It does not score field verification — that is tracked separately in
> [`MERGE_WAIVER_2026-07-27.md`](MERGE_WAIVER_2026-07-27.md) and remains the larger debt.
> Dash mode has never painted a line.
> **CORRECTED 2026-07-29 by the operator:** `POST /api/path/plan-trajectory` **IS field-validated**
> (run 2026-07-28). Earlier statements in this document and in project memory calling it
> "never used in a field run" were wrong.

---

# ✅ CLOSURE ADDENDUM — 2026-07-29, end of day

**All 14 findings resolved: 13 fixed, 1 refused with mechanical evidence.** Plus 8 more found during
execution, all closed. Branch **`prod/audit-stage3b` @ `65a930f`**, deployed to the Jetson,
cold-boot verified. `baseline_master` untouched at `6fe6c76`.

## Score: 71 → **82 / 100**

| Layer | Was | Now | What moved |
|---|---|---|---|
| `path_engine` | 82 | **86** | P1 dedup fixed (better than specced), P3 property suite added, 475 → 487 tests. P2 still undone — but now *understood*, with a prerequisite, rather than unexamined debt. |
| `server` | 74 | **87** | Every finding closed: S1 S2 S3 S4 S5 + S1-a, plus an auth-bootstrap capability that was not in the audit. |
| `src` (ROS2) | 68 | **85** | R1 R2 R3 R4 R5 R6 R7 + B1 B2 N2 all closed. RPP stays single-threaded **by decision**, with the blocker documented. |
| Pipeline / ops | 60 | **68** | The rclpy suite now runs and is green (430/0, exit 0) — but nothing runs it automatically, and we *discovered* dependency management is broken on the Jetson (see below). |

Same weighting: path_engine 25 % · server 25 % · ROS2 30 % · ops 20 %.

## ⚠️ Read this before believing the number

**The code is better. The *proof* is worse.** This morning the rover ran field-proven baseline code.
It now runs ~20 changes, **none of which has painted a line or driven a metre**. On a
"would I ship this today" basis, confidence is *lower* than it was at 71 — and stays lower until the
dash run happens. The score measures the code; it does not measure whether the rover still marks a
straight line.

**A clean dash run is worth more than every remaining item on this list combined.**

## What is still open

| Item | State |
|---|---|
| **Dash field run** | The whole point. Ten fixes live, zero verified. |
| **P1 field pass** | Only Stage 3 item that changes planner output — any mission with CAD geometry in the 1–5 cm band now plans differently. |
| **App login test** | Auth enforced today; no real client has touched it. |
| **P2** | Refused with AST evidence — see below. Reopening requires a golden-plan snapshot harness first. |
| **CI** | The runner exists and is green; nothing invokes it automatically. |
| **Jetson pip is broken** | `setuptools 80.9.0` in `~/.local` shadows the system one and calls `canonicalize_version(strip_trailing_zero=…)`, which apt's `packaging 21.3` rejects. **Every sdist install on that box fails.** `requirements.txt` is therefore fiction — `sdnotify` was pinned yet absent. Use apt, or fix the pair (risky: ROS2 shares it). |
| **Seam fold** | `group_join_tol_m` snaps a junction vertex up to 5 cm — 2.5× the marking spec. Correct for its design case, a real loss for overlapping collinear strokes. Documented + pinned, not fixed. |
| **Frontend rotation screen** | `must_change_password` handling — separate repo. Blocks the *next* headless site, not this rover. |

## Closure log

| ID | Resolution | Commit |
|---|---|---|
| R5 | dash respects TRANSIT flag | `6dd247d` |
| R4 | dash grid anchored to first MARK_START, not arrival | `6dd247d` |
| R6 | terminal shutoff hoisted out of the continuous-only branch | `6dd247d` |
| B1 | meter rebuilt on every `/path` — fixes the GPS_SURVEYED two-phase entry | `d030154` |
| B2 | solenoid lead restored at mark boundaries via a shared helper | `d030154` |
| R3 | xtrack gate 0.10 → 0.03 + per-mission override, no `SCHEMA_VERSION` bump | `c4fdcde` |
| S1 | `Type=notify` + `WatchdogSec=15` — **cold-boot verified**, READY in 6 s vs 60 s | `0cf6de7`, `48e9178` |
| S1-a | heartbeat must fire in ROS-degraded mode | `abaf626` |
| S2 | emit timeout + E-stop split to its own task | `2aeaa5d` |
| R1 | **single-reference mission mailbox** — supersedes the audit's wrong ask | `99f992e` |
| R2 | measured `dt` in both accel ramps | `6076d99` |
| R7 | Jetson test runner — **430/0, exit 0, first ever green** | `3227e5c`, `b9965d0` |
| S3 | operator auth **enforced**; joystick manual drive disabled | `9f42559` |
| — | auth bootstrap: default password + server-enforced rotation | `0b572ea` |
| N2 | clean SIGTERM across four nodes — zero tracebacks on restart *and* cold boot | `9975c2d` |
| R2-clamp | `dt` floored at 0.0 | `5b50efc` |
| S4 | size caps on both point-CSV routes | `0fabb46` |
| S5 | `Semaphore(2)` around the five heavy planner offloads | `7f23f1b` |
| P1 | dedup: distance test at a **decoupled 1 cm**, not 5 cm rounding buckets | `c22b144` |
| P3 | hypothesis property suite — proven to kill the pre-P1 engine | `2b597ed` |
| P2 | **REFUSED** with AST evidence | — |

## Three times this audit's proposed fix was worse than the bug

Worth recording, because the pattern held every time:

1. **R1** — "make `_apply_run` atomic" was *wrong*, not incomplete. `_path_cb` writes **16 more**
   tracked fields outside it, including `_ekf_reset_offset`, whose staleness is a silent metre-class
   placement error. The audit's fix would have closed the item on paper while leaving something worse.
2. **P1** — "endpoint-distance test within tol" would have made the 4 cm-parallel-line deletion
   **deterministic**; the rounding buckets only did it sometimes. `tol` was `mark_spacing` = 5 cm and
   the failure case was 4 cm.
3. **P2** — the proposed `_step_N(segments, cfg) -> list[PathSegment]` signature is provably wrong for
   ≥8 of 12 passes: 18 values cross step boundaries, 8 stats dicts converge on the epilogue. With a
   context object the payoff *inverts* — ordering coupling becomes invisible where today it is at
   least prose-documented.

**Lesson for this codebase: verify the coupling before restructuring.** Mechanical extraction — write-set
diffing, AST analysis — beat argument every time.

---

## Original verdict, 2026-07-29 morning — 71 / 100

*(kept as the historical record; superseded by the addendum above)*

| Layer | Score | One-line verdict |
|---|---|---|
| `path_engine` | **82** | Genuinely production-grade. Pure, deterministic, well-tested, numerically correct. |
| `server` | **74** | Async discipline is right. Auth is shipped **off** and the systemd watchdog is **inert**. |
| `src` (ROS2) | **68** | Sound control code undermined by executors configured for zero parallelism, and three dash defects. |
| Pipeline / ops | **60** | No CI, no lint, no pinned deps; the safety-critical tests run nowhere. |

Weighting: path_engine 25 % · server 25 % · ROS2 30 % · ops 20 %.

**The code is well past prototype.** What separates it from a product is (a) four real defects in the
dash/continuous spray path, (b) the supervision + auth layer, (c) any mechanism at all for knowing it
still works.

---

## Evidence — tests actually executed, not read

```
path_engine/tests    475 passed   0.65 s
server/              370 passed   3.47 s
src/  (non-rclpy)    375 passed   0.19 s
                    ─────────────
                    1 220 passing off-robot

src/  (rclpy)         15 files    CANNOT RUN off-Jetson — and nothing runs them on it
```

Test-to-source line ratio: path_engine 0.95 : 1 · server 0.53 : 1 · src 0.69 : 1.

**Static hygiene:** zero bare `except:`. Zero `except Exception: pass`. Zero un-timed network calls.
Every CPU-bound or filesystem call in the routes goes through `asyncio.to_thread` wrapped in
`asyncio.wait_for` with a named 504. This is not typical of a prototype and should be said plainly.

---

## Deltas since the 2026-07-26 maturity audit

Two gaps recorded three days ago are now **CLOSED** and are re-verified here:

| 07-26 finding | Status 07-29 | Evidence |
|---|---|---|
| "DXF plan runs ON the event loop (3 sites)" | **CLOSED** | 16 `asyncio.to_thread` sites in `routes/path.py`; `plan_segments` (:2456) and `path_mgr.plan_path` (:1991) both wrapped with `wait_for`. No planning call reaches the loop. |
| "`/plan-and-stage` silently drops `close_shape` + `fit_arcs*`" | **CLOSED** | Both routes now forward the full param set; the in-code comment at `routes/path.py:~2019` documents the fix. |

One gap is **still open, unchanged, and is now the top ship-blocker**:

| 07-26 finding | Status 07-29 |
|---|---|
| "Dash mode overwrites `projection.current_flag` — paints across transit legs; no terminal shutoff" | **STILL OPEN.** Confirmed at `spray_controller_node.py:466` / `:521`. Scored 35 % then; unchanged now. |

This audit adds three dash findings the 07-26 pass did not name: pattern-anchor
non-determinism (R4), the xtrack-gate calibration argument (R3), and the systemd
watchdog being inert (S1).

---

# Layer 1 — `path_engine` · 82 / 100

The strongest code in the repository, and the part that is finished.

**What is right, and must not be touched:**

- [`ned.py`](../path_engine/ned.py) is an exact port of PX4 `MapProjection::project()` with the WGS84
  trap documented and fenced off (`geodesic_ground_distance_m` is the only sanctioned WGS84 use).
- [`dxf_to_ned_affine`](../path_engine/ned.py#L211) locks scale to 1.0 with a written argument for
  *why* a free-scale fit hides errors: a 2-point similarity fit has zero residual by construction, so
  RMSE reads ~0 no matter how wrong the size is. `estimate_fit_scale` keeps the free value as a
  diagnostic only.
- Field lessons are encoded as prose next to the code that carries them (the run-out 0.10 restore at
  `engine.py:1265-1284`, the `_retraces` collinearity test at `:1001`, the E1–E5 extension defects).
  This is the repository's real asset.
- The arc fit, extension geometry, corner fillet, shape grouping and the optimizer are all
  **closed**. Do not reopen them.

## P1 — [MEDIUM] Duplicate-geometry dedup uses rounding buckets, not a tolerance test

**`path_engine/engine.py:822`**

```python
tol = max(self.mark_spacing, 1e-3)                       # 0.05 m
a = (round(seg.points[0][0] / tol), round(seg.points[0][1] / tol))
key = (min(a, b), max(a, b), length)
if key in seen:                                          # → segment DROPPED
```

Rounding is not proximity.

- **False negative:** two genuinely coincident lines whose endpoints straddle a 5 cm bucket boundary
  hash differently, both survive, and the rover paints the line twice with a 180° reversal between
  passes — the exact failure this block was written to prevent (`sct_1.5m.DXF`, handles 64/67).
- **False positive:** two *distinct* parallel lines 4 cm apart with equal length collide, and one is
  **deleted from the mission** with only a `log.warning`. For road pre-line work with tight parallel
  geometry (stop-bar hatching, double centre lines) this is silent geometry loss.

**Fix:** replace the hash with an O(n²) endpoint-distance test over MARK segments only — *n* is tens,
not thousands — and promote the drop from `log.warning` to a hard error unless the caller passed
`allow_duplicate_drop=True`. `duplicate_stats` already rides in `planning_metadata`; surface it in the
app's plan-confirmation screen.

## P2 — [MEDIUM] `_plan_from_segments` is an 860-line method with 12 order-dependent passes

**`path_engine/engine.py:532-1394`**

Every pass mutates `segments` in place, and the ordering constraints exist only as prose:

> *"Step 3 withholds the optimizer's transit links **because** Step 4 is about to wrap each mark in
> PRE/AFT…"* · *"fillet runs AFTER the arc fit **so that** a genuinely surveyed arc is recovered
> first…"* · *"close_shape runs after arc fit **so** the closing edge is a straight chord…"*

It is correct today and the tests pin the behaviour. It is also the highest-risk thing in the
codebase to modify — and the point-mission upgrade will modify it.

**Fix — the only refactor sanctioned by this audit, and it is not a rewrite:** extract each numbered
step to a module-level `def _step_N(segments, cfg) -> list[PathSegment]` with the bodies moved
**byte-for-byte**, leaving `_plan_from_segments` as a ~30-line pipeline list. Ordering becomes data
you can assert on. **Do this before the point work, not after.**

## P3 — [LOW] No property/fuzz test on the geometry invariants

475 tests, all example-based. The invariants worth randomising are cheap and high-value:

- spacing ≤ `mark_spacing` everywhere
- `len(spray_flags) == len(merged_waypoints) == len(must_hit)`
- no NaN/Inf in any output coordinate
- total length within ε of the sum of per-segment lengths
- **no MARK point present in the input is absent from the output**

≈40 lines with `hypothesis`.

---

# Layer 2 — `server` · 74 / 100

**What is right:** [`control_arbiter.py`](../server/control_arbiter.py) holds its lock only across the
ownership *claim*, never across a caller's slow body, with a written explanation of why in-flight-only
ownership makes the mission-stranding bug unrepresentable. [`_await_ros_future`](../server/ros_node.py#L966)
correctly handles the late-result race `asyncio.wait_for` creates against rclpy futures. Dead-process
detection (`_rpp_debug_recv_time`, `_state_recv_time`) is thorough and reasoned. Graceful degradation
without MAVROS is deliberate and documented in the unit file.

## S1 — [CRITICAL] The systemd watchdog is dead code

**`rover-server.service:9`** is `Type=simple` with **no `WatchdogSec=`**.
**`server/main.py:201`** sends `READY=1`; **`server/main.py:578`** sends `WATCHDOG=1` every ~3 s.

With `Type=simple`, systemd ignores both notifications. **The heartbeat goes nowhere.** The code
exists, runs, costs cycles, and supervises nothing.

```ini
# rover-server.service
Type=notify
WatchdogSec=15
```

## S2 — [CRITICAL] The entire safety-abort path is one unsupervised asyncio task

**`server/main.py:338`** — `_telemetry_loop` runs *all* of:

1. telemetry push (10 Hz)
2. mission auto-completion (`RUNNING → COMPLETED`)
3. ENTRY → RUNNING advance
4. **the pose-stale / RPP-dead / FCU-disconnected E-stop** (`main.py:521`)

It catches `Exception` per iteration with exponential back-off, so a *raising* bug is survivable.
What is **not** survivable is the loop blocking forever inside an `await`.
[`_emit_authenticated`](../server/main.py#L323) awaits `sio.emit(...)` per SID **with no timeout**. A
phone that walks out of WiFi range with a full TCP send buffer stalls that await — and when it stalls,
the E-stop watchdog stops running while the rover is driving.

Nothing detects this today: no external heartbeat (S1), no restart, only
`finally: log.info("telemetry loop exited")`.

```python
# 1. bound every emit
await asyncio.wait_for(sio.emit(event, data, to=sid), timeout=0.5)

# 2. move the safety watchdog into its own task — get_state() + estop only,
#    never touching Socket.IO. Then S1's WatchdogSec supervises what remains.
```

## S3 — [MEDIUM — CORRECTED 2026-07-29] Auth is disabled in the shipped production unit

> ⚠️ **CORRECTION.** This finding originally read *"bound to `0.0.0.0:5001` — anyone on the site WiFi
> can `POST /api/mission/start`."* **That is wrong.** Disabling auth also forces a loopback bind:
> `server/run.sh:16` (`ROVER_DISABLE_AUTH=1 → HOST=127.0.0.1`) and `server/config.py:309`
> (`DEFAULT_HOST = "127.0.0.1"`, plus CORS narrowed to localhost). `FASTAPI_HOST` is **not** set in
> `rover-server.service`, so as committed the server is **loopback-only** and the unauthenticated-LAN
> attack described above does not hold. Severity drops HIGH → MEDIUM. Original claim retracted.

**`rover-server.service:49`** — `Environment=ROVER_DISABLE_AUTH=1`.

A complete auth stack exists and is tested: PBKDF2 at 260 000 iterations, machine tokens, session TTL
([`server/auth.py`](../server/auth.py), 515 lines). Production bypasses all of it — but the same flag
also confines the server to `127.0.0.1`.

**The two are coupled, and that inverts the fix.** Deleting the line does not merely "turn auth on" —
it simultaneously flips the bind to `0.0.0.0` and CORS to `*`, exposing the API to the LAN **for the
first time**. And there is **no bootstrap route**: `POST /api/auth/login` returns 503 when no password
is configured, `change-password` requires an existing session, and the first password can only be set
host-side via `server/rover_auth_cli.py setup` (interactive) followed by a **restart**.

So removing the flag before bootstrapping a password yields a LAN-exposed server that nobody can log
into, recoverable only by SSH.

**This is a deploy-time sequence, not a code change:**

1. On the Jetson: `python3 server/rover_auth_cli.py setup` (interactive, min 8 chars) → writes the
   PBKDF2 hash to `config/rover_password.json`.
2. Remove `Environment=ROVER_DISABLE_AUTH=1`.
3. `./deploy.sh` (daemon-reload) and restart `rover-server`.
4. Verify the boot log reads `auth: loaded operator_configured=True`, then confirm
   `POST /api/auth/login` returns a token before letting the operator app near it.

**Open question that only the Jetson can answer:** if a systemd drop-in on the device sets
`FASTAPI_HOST=0.0.0.0`, the original HIGH finding stands in full — unauthenticated **and**
LAN-reachable. Check `systemctl cat rover-server` on the device before accepting the MEDIUM rating.

## S4 — [MEDIUM] Two upload routes have no size cap, unlike every other one

**`server/routes/path.py:958`** (`/parse-point-csv`) and **`:979`** (`/parse-point-gps-csv`) do
`content = (await file.read())` — unbounded.

`/upload` (`:995`) and `/parse-dxf` (`:1036`) both correctly read `MAX_UPLOAD_BYTES + 1` and raise 413.
The **inconsistency is the bug** — the correct pattern already exists two functions away. Two-line fix.

## S5 — [LOW] No concurrency bound on planning

`asyncio.to_thread` uses the default executor (32 threads) while `rover-server.service` sets
`CPUQuota=200%`. Several concurrent plan requests each get a slice and **all** hit the 15 s timeout,
instead of one succeeding. An `asyncio.Semaphore(2)` around the plan calls turns total failure into a
queue.

---

# Layer 3 — `src` (ROS2) · 68 / 100

**What is right:** `rpp-pipeline.service` sets `CPUSchedulingPolicy=fifo`, priority 80, and
`CPUAffinity=4` pinned to a Jetson Orin *performance* core — that is better than most robotics shops
ship. [`_project_onto_path`](../src/rpp_controller_node.py#L2867) is a windowed O(1) search with a
correct full-scan fallback on hint invalidation.
[`twist_to_setpoint_node`](../src/twist_to_setpoint_node.py) holds the 50 Hz OFFBOARD heartbeat
contract independently, with a fail-closed zero-velocity default and NaN rejection on input. The spray
FSM confirms actuator acks rather than assuming them. **None of that should change.**

## 3a — Threading, executors, latency

### R1 — [HIGH] Both executors are configured for zero parallelism

**`server/ros_node.py:253`** — `MultiThreadedExecutor(num_threads=4)`, but **ten** subscriptions share
a single `MutuallyExclusiveCallbackGroup`: pose, battery, global_pos, gp_origin, gps_raw, `/rpp/debug`,
`/rpp/velocity_ned`, `/spray/state`, `/spray/manual_state`, `/spray/status`. They serialise on one
thread; the 4 threads buy nothing. A `json.loads` in `_cb_spray_status` (`:757`) delays the pose
callback.

`/mavros/state` was already given its own group with a correct written rationale (starvation cascading
into joystick rejections). **Apply the same reasoning to the two other hot paths:**

```python
self._pose_group = MutuallyExclusiveCallbackGroup()   # /mavros/local_position/pose
self._rpp_group  = MutuallyExclusiveCallbackGroup()   # /rpp/debug
```

**`src/rpp_controller_node.py:4524`** is worse — plain `rclpy.spin(node)`, default callback group, so
`_path_cb`, `_pose_cb`, `_vel_cb` and the 50 Hz `_control_loop` are strictly serial on one thread.

*(Supersedes and makes concrete the open item in memory `cpu_contention_executor_mismatch`.)*

### R2 — [HIGH] The 50 Hz control loop assumes a fixed dt that a long `_path_cb` breaks

**`src/rpp_controller_node.py:3335`** and **`:3915`**

```python
speed = min(speed, self._last_speed_cmd + max_accel / self.CONTROL_HZ)   # dt assumed = 20 ms
```

Meanwhile [`_path_cb`](../src/rpp_controller_node.py#L795) → `_apply_run` runs resample +
corner-smooth + DP-simplify over the whole run **inside the same single-threaded executor**. For the
stated primary product — ~1 km of road pre-line at 5 cm spacing, ≈20 000 points — that is hundreds of
milliseconds of pure Python blocking the control timer.

The next tick then applies **one** 20 ms accel step to cover ~200 ms of real elapsed time. The velocity
ramp under-commands by an order of magnitude exactly at mission start, which is where the entry and
first-line geometry are decided.

FIFO/80 + core pinning fix *scheduler* jitter. They do not fix a self-inflicted long callback.

```python
now = self.get_clock().now()
dt = min(0.1, (now - self._last_tick).nanoseconds * 1e-9) if self._last_tick else 1 / self.CONTROL_HZ
self._last_tick = now
speed = min(speed, self._last_speed_cmd + max_accel * dt)
```

…plus move `_path_cb` to its own callback group under `MultiThreadedExecutor(2)` so path conditioning
cannot pre-empt the control tick.

## 3b — Continuous mode

### R3 — [HIGH] The spray xtrack gate is 5× looser than the accuracy claim

**`src/spray_controller_node.py:644`** — `max_xtrack_error_m = 0.10`, enforced at **`:446`**.

The gate is **not missing** (correcting an earlier project-memory note) — it is calibrated an order of
magnitude wide. The product claim is ±2 cm marking; the valve is permitted to open anywhere within
10 cm of the line.

This **is** the standing field observation *"valve opens at 1.6–6.6 cm control xtrack"*: every one of
those values sits inside the gate, so the gate never fired. For a marking product this is the single
most important number in the repository, and it is not derived from the accuracy specification.

**Fix:** `0.10 → 0.03`, and move it from a node parameter into `SpraySessionConfig` — line width and
tolerance are *mission* properties, not node properties. Expect more refusals at first; **that refusal
is the product working.**

## 3c — Dash mode · three structural defects

[`DashMeter`](../src/spray_modes.py#L37) itself is well written: monotonic clamping, jump rejection
with a bounded recovery path, drift-free ideal-grid toggling, and a documented rationale for
continuous-across-mission metering. **All three defects are in how the node wires it in, not in the
meter.**

### R4 — [HIGH] Dash pattern placement is non-deterministic between runs

**`src/spray_modes.py:135`**

```python
self.armed = True
self.s_dash = raw_s
self.s_at_last_toggle = raw_s      # ← the pattern anchor
```

The meter arms on the first tick where `xtrack_ok` — and `xtrack_ok` means *"within 10 cm"* (R3). The
dash grid origin is therefore set by **where the rover happened to converge onto the line**, not by the
mission geometry.

Run the same mission twice, enter the path 30 cm further along, and **every dash in the mission shifts
30 cm.** That is a repeatability defect in the product's defining output — a customer re-marking a road
cannot lay new dashes over the old ones.

**Fix:** anchor to a geometry-defined station. Pass the first `MARK_START` boundary `s` from the path
model into `DashMeter` at construction and seed `s_at_last_toggle` with it. Dash placement then becomes
a pure function of the plan, and two runs of one mission are identical.

### R5 — [HIGH] Dash mode discards the MARK/TRANSIT flag — it paints the connectors

**`src/spray_controller_node.py:445`** sets `geometry_desired = projection.current_flag`; **`:466`**
overwrites it with `du.geometry_desired`. `current_flag` never re-enters —
**`:535`** is `desired = bool(geometry_desired and safety_ok)` with no AND against the plan's spray
flag.

A dash session's path model is the full published `/path`:
[`build_session_config`](../server/spray_session_builder.py#L56) applies no geometry filter. So on any
multi-run mission the meter keeps metering across TRANSIT connectors and PRE/AFT run-outs, and **paints
dashes on ground the plan marked spray-OFF.**

*"Continuous across mission"* (the locked operator decision, 2026-07-23) means the **pattern phase**
does not reset at boundaries. It does not mean the valve ignores the transit flag.

```python
geometry_desired = du.geometry_desired and projection.current_flag
```

The meter keeps integrating `s` through the transit — so the phase still does not drift — but the valve
stays shut. That is exactly the §7.2 requirement.

### R6 — [MEDIUM] Dash mode has no terminal shutoff

The B4 terminal-off guard at **`src/spray_controller_node.py:521`** sits inside the `else:` branch
opened at **`:472`** — the **continuous-only** path. The dash branch (`:452-471`) never reaches it.

At mission end, if the rover stops on an "on" phase, only the `active_timeout_s` staleness watchdog
closes the valve. That is a puddle at the end of every dash mission.

**Fix:** hoist the terminal block out of the `else:` so it runs for both modes. It is already written to
be independent of the boundary/lead math ("fires independently of any boundary/lead").

## 3d — Ops

### R7 — [HIGH] The 15 safety-critical test files run nowhere

`test_corner_pivot` · `test_corner_stop_brake` · `test_completion_stop` · `test_endpoint_approach` ·
`test_entry_prealign` · `test_ekf_reset_compensation` · `test_point_handshake_rpp` ·
`test_point_handshake_spray` · `test_point_hold_rpp` · `test_precise_stop_node` ·
`test_progress_publication` · `test_smoke_rpp_controller` · `test_spray_flag_conditioning` ·
`offboard_test` · `spin_in_place_test`

All require `rclpy`; none run on the Mac; and there is **no CI anywhere in the repo** — no
`.github/`, no `pyproject.toml`, no lint config, no pinned dependency set.

These are precisely the tests that guard behaviours already regressed once in the field (the run-out
0.05 terminal-heading flip; the collinear spray-boundary momentum fix).

**Fix:** this does not need GitHub Actions. A `make test-jetson` run over SSH plus a git pre-push hook,
or a systemd `OnCalendar` timer on the Jetson that runs them nightly and writes a status file the
server exposes at `/api/health/tests`. It needs *something*.

---

# Fix order

### Ship-blockers — before calling it a product (≈1 day)

| # | ID | Change | Size |
|---|---|---|---|
| 1 | **R5** | Dash paints connectors — `and projection.current_flag` | 1 line |
| 2 | **R4** | Dash anchor non-determinism — seed from first MARK station | ~10 lines |
| 3 | **R3** | xtrack gate `0.10 → 0.03`, moved into `SpraySessionConfig` | ~15 lines |
| 4 | **R6** | Hoist terminal shutoff out of the continuous branch | indentation |
| 5 | **S1** | `Type=notify` + `WatchdogSec=15` | 2 lines |
| 6 | **S3** | Delete `ROVER_DISABLE_AUTH=1` | 1 line |

### Structural — next (≈2 days)

| # | ID | Change |
|---|---|---|
| 7 | **S2** | Timeout every Socket.IO emit; split the safety watchdog into its own task |
| 8 | **R1** | Three callback groups in `ros_node.py`; `MultiThreadedExecutor(2)` + own group for `_path_cb` |
| 9 | **R2** | Measured `dt` in both accel ramps |
| 10 | **R7** | Automated Jetson test run + `/api/health/tests` |

### Hardening — then (≈1 day)

| # | ID | Change |
|---|---|---|
| 11 | **P1** | Distance-based dedup; hard error by default |
| 12 | **S4** | Size caps on the two point-CSV routes |
| 13 | **P2** | Extract the 12 pipeline steps — **before** the point work |
| 14 | **S5 / P3** | Planning semaphore; geometry property tests |

**Fourteen items, ≈4 days. Nothing here is a rewrite. The architecture is right.**

---

# THE NEXT STEP

> **Fix the four dash defects (R3–R6) and re-run the dash mission in the field.
> That is the next step, and it is the only one.**

Dash is the named primary mode. As shipped it will paint the connectors, place its pattern differently
on every run, keep the valve open past the end, and open it 10 cm off-line. **None of these surface as
errors** — the mission reports success and the paint is wrong.

All four are one-file fixes in `spray_controller_node.py` / `spray_modes.py`, and no amount of the rest
of the list substitutes for one clean dash run on the ground.

Everything else above is real, but it is what you do *while waiting for weather*.

---

# Final note — point-mission upgrade path

The point mode is **further along than dash mode**, which is not the usual reading of it. Both halves
the design called for actually exist:

| Half | Status | Location |
|---|---|---|
| Node-side FSM | **Complete and correct** | [`PointMeter`](../src/spray_modes.py#L228) |
| Controller-side per-point hold | **Implemented** — the docstring still calls it "owed" | `rpp_controller_node.py:1857` `_point_hold_tick`, `:1798` `_point_handshake_ready` |
| G4 arrival handshake | **Implemented** | `/rpp/milestone AT_POINT` |
| G5 manual advance | **Implemented** | `/point/advance`, `ros_node.py:784` |
| CSV ingest | **Correct** | [`point_ingest.py`](../src/point_ingest.py) — 308 tests |

`PointMeter` bakes in the right safety rules and they are worth recording: ADVANCE waits for
`off_confirmed` rather than merely for `dwell_s` to elapse (a retrying OFF can never let the mission
leave a still-spraying dot); ARRIVING gives **no partial credit** on a settle blip; the unreachable-target
watchdog skips a bad coordinate, logs it, and **never counts it as sprayed**; `exempt_pivot` is narrow —
dwelling within tolerance of the *active* dot only, never mode-wide.

**The upgrade path is therefore short, and it is three things, in order:**

### 1. Close bug A11 first

The survey-CSV fallback in the staging path swallows bare point-CSV parse errors and drops `dwell_s`
and `mark`, so `mark=0` transit points come back painted. Everything above is correct and none of it
helps if the flags never reach the node.

This is a **routing** bug in the load path, not a bug in `point_ingest` — which is exactly why 308
`point_ingest` tests pass while the field behaviour is wrong.
See memory `bug_a11_point_csv_validation_swallowed`.

### 2. Point mode inherits R3, and it matters more there

A dot is sprayed at a standstill with a separately-calibrated `point_dwell_flow_value`, but arrival is
governed by `arrival_tolerance_m` — a **different** number from the xtrack gate. Make the two
consistent and derive both from the marking specification, or a dot lands within tolerance of the
wrong place.

### 3. Do P2 (extract the pipeline steps) *before* adding point geometry to the engine

Point missions bypass most of `_plan_from_segments` — they do not want densification, extensions,
corner smoothing, or the TSP in its current form. Threading a fourth mode through an 860-line method by
adding conditionals is how it becomes unmaintainable. Extract first; point then becomes a short
pipeline list of its own steps that reuses only the parts it wants.

> **Point mode does not need new control code.** It needs the load path fixed, the tolerances unified
> with the marking spec, and the engine's step ordering made explicit before it grows a fourth consumer.

---

## Cross-references

- [`PROJECT_MATURITY_AUDIT_2026-07-26.md`](PROJECT_MATURITY_AUDIT_2026-07-26.md) — commercial-envelope
  audit (~65 %). Complementary: that one scores the product, this one scores the code.
- [`MERGE_WAIVER_2026-07-27.md`](MERGE_WAIVER_2026-07-27.md) — field-verification debt.
- [`OPEN_BUGS.md`](OPEN_BUGS.md) — A11 and the B-series.
- [`Architecture/SPRAY_CONTROLLER_V2_PLAN.md`](Architecture/) — §7.2 dash, §7.3 point.
