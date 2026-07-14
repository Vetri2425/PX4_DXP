# Production Hardening Audit — Comms, Geometry, Controller, GNSS

**Date:** 2026-07-14
**Branch:** `baseline_master` @ `ea47ec8`
**Method:** 4 parallel read-only audits (Sonnet agents) over `server/**`, `path_engine/**`, `src/**`, `PX4_params/**`
**Status:** Findings only — **no code changed**
**Companion doc:** `01_COMMERCIAL_BENCHMARK_AND_UPGRADE_PATH.md` (market benchmark)

> **Goal this audit serves:** stabilize the current version — no crashes, no stale/hung telemetry —
> and make the plan→trajectory chain dimensionally trustworthy.
> **Acceptance criterion (operator's words):** *"If I give a Square DXF with 2 m, the trajectory
> must be 2 m — reliably, production-grade."*

---

## 0. Headline

| # | Finding | Severity | Status |
|---|---|---|---|
| **A** | Alignment transform **solves scale as a free parameter** → a 2 m square is accepted anywhere in **1.5–2.5 m** | 🔴 **Breaks the acceptance criterion** | Confirmed, code-read |
| **B** | **E-stop crashes on first call** after boot and reports failure *even though the rover stopped* | 🔴 Safety-critical | Confirmed, code-read |
| **C** | `rpp_controller` death is **invisible** — telemetry freezes at last-healthy values, watchdog trusts data written by the dead process | 🔴 Operator misled | Confirmed, code-read |
| **D** | Mid-mission crash-restart may **re-run the mission from run 0** and drive back over marked ground | 🔴 **If confirmed, worst field risk here** | ⚠ **UNVERIFIED — verify first** |
| **E** | Two **one-line param crashes** reachable from `ros2 param set` (`max_linear_decel=0` → ZeroDivisionError) | 🟠 Crash-loop | Confirmed, code-read |
| **F** | Densification bug from June **still live** in `shape_grouping` → alternating 5 cm/10 cm spacing | 🟠 Violates spray-metering invariant | **Confirmed by executing the code** |
| **G** | UM982 **dual-antenna heading already fused** (`EKF2_GPS_CTRL=15`) | ✅ Good news | Confirmed, param dump |

**Note on G:** this **overturns** the top recommendation of doc `01`. The "46% accuracy win" from
enabling dual-antenna heading is **already banked**. Doc `01` has been corrected.

---

## 1. ✅ What is genuinely production-grade already

Do not "fix" these. They work, and the audits verified them in code.

### Controller / OFFBOARD

- **Process isolation is the real safety net.** Production runs `rpp_start.sh:82-150` (NOT
  `launch/rpp_pipeline.launch.py`, which self-describes as a convenience wrapper). Each node is a
  **separate OS process**, with a bash watchdog restarting any dead one in 2 s
  (`rpp_start.sh:139-147`). An `rpp_controller_node` crash **cannot** kill `twist_to_setpoint_node`.
- **`twist_to_setpoint_node` is a proper safety boundary:**
  - rejects **non-finite (NaN/Inf) velocity** on `/rpp/velocity_ned` and drops it
    (`twist_to_setpoint_node.py:179-187`) — this is exactly the `inf`-leak that caused the historical
    crash-loop, killed at the boundary;
  - detects input staleness (`input_max_age_s=0.2 s`) and **streams zero velocity**
    (`:215-230`) — so if RPP dies, **OFFBOARD setpoints keep flowing**, the ≥2 Hz / <0.5 s PX4
    contract holds, and the rover safe-stops in ~200 ms instead of dropping to failsafe.
- **Numeric guards are consistently correct**: `acos` domain-clamped (`rpp_controller_node.py:2010-2012`),
  every segment-length division guarded (`:1853-1856`, `:2270-2272`, `:2000-2003`), lookahead-walk
  and curvature divisions guarded (`:2144`, `:2329`, `:2982-2984`, `:3161`).
- **Watchdogs are angle-aware, not naive timeouts**: `_pivot_timeout_budget` scales with corner
  angle (`:3407-3424`); `_corner_stop_satisfied` has a bounded stale-velocity fallback
  (`_CORNER_STOP_MAX_HOLD_S=2.0`) so a dead `/velocity_local` can't deadlock CORNER_STOP forever
  (`:3355-3405`).
- **Firmware-aware forward-cone clamping** (`:3239-3297`) — field-validated BUG-T3 fix for PX4's
  reverse-flip.
- **`spray_controller_node` fails closed** — `shutdown_off()` in a `finally` block (`:655-666`).

### Server / comms

- **NaN/Inf never reaches the client** — `_sanitize()` (`server/main.py:274-283`) applied to every
  `telemetry` and `mission_status` emit (`:354,364`).
- **MAVROS crash detection is real.** `/mavros/state` is TRANSIENT_LOCAL, so a dead MAVROS leaves a
  cached `connected=True` forever. Overridden in **both** code paths via `_state_recv_time` /
  `_MAVROS_STATE_TIMEOUT_S=2.0` (`server/ros_node.py:493-496`, `:524-527`, `:202`).
- **ROS service calls never block the event loop** — `_call_async` → `_service_ready_async`
  (fail-fast) → `_await_ros_future` with `asyncio.wait_for`; the late-result race after timeout is
  explicitly guarded (`server/ros_node.py:546-802`, `:599-604`).
- **Telemetry loop self-heals** with capped exponential backoff, correctly re-raising
  `CancelledError` (`server/main.py:458-466`).
- **No orphaned `create_task` handles** — every background task keeps a reference.
- **RTK subprocess lifecycle fully async + supervised** (`server/rtk_manager.py:114-219`).
- **Auth is sound** — PBKDF2-HMAC + `hmac.compare_digest`, atomic fsynced writes
  (`server/auth.py:98-116`, `:150-165`).

### Geometry

- **Units handled correctly and completely** — full `$INSUNITS` table
  (`path_engine/parsers/dxf_parser.py:52-69`), applied at parse time to every coordinate. **A 2 m
  square authored as 2000 mm becomes exactly 2.000 m.**
- **No UTM anywhere — no grid-convergence / scale-factor error.** Geodesy uses GeographicLib's
  **Karney inverse geodesic** straight to a local NED delta (`path_engine/ned.py:22-54`). There is no
  projected CRS in the loop at all. *(This retires the "0.4 m/km UTM scale" concern raised in doc `01`
  — it does not apply to this design. Good architectural call.)*
- **Surveyed live-placement is provably rigid** — translation only, no rotation, no scale
  (`server/mission_placement.py:121-146`).
- **1-ref-point and GPS-origin alignment paths force `scale = 1.0` explicitly**
  (`path_engine/engine.py:501`, `:521`).
- **float64 throughout** the planning stack; `nav_msgs/Path` fields are float64 → `/path` introduces
  **no** truncation.
- `densify_line` **forces exact endpoints** regardless of remainder (`straight_line.py:46-48`).
- Arc/circle/bulge discretization is **chord-error bounded (5 mm)** with exact start/end
  (`arc_curve.py:116-129`).
- **TSP only reverses point lists** — never mutates coordinates (`segment_order.py:22-41`).
- **PRE/AFT extensions never mutate MARK geometry** (`extensions.py:461-469`).
- **Staged reload does not re-apply alignment** → no double-scaling (`routes/path.py:1054-1082`).

### GNSS

- **Dual-antenna heading is fused.** `EKF2_GPS_CTRL = 15` (bit 3 = dual-antenna heading;
  default 7 = off) — `PX4_params/12-06-2026/init.params:251`. Deliberately tuned: `7` → `11` → `15`
  by 2026-06-08, stable. Corroborated by `GPS_1_CONFIG=101` (TELEM1), `SER_TEL1_BAUD=230400`
  (UM982's mandated rate), `GPS_1_PROTOCOL=6` (NMEA), `GPS_YAW_OFFSET=180` / `EKF2_GPS_YAW_OFF=180`.

---

## 2. 🔴 A — Alignment solves SCALE as a free parameter (**breaks the 2 m criterion**)

**This is the single defect that directly violates the acceptance criterion.**

`path_engine/ned.py:57-134` (`dxf_to_ned_affine`) is a least-squares **similarity** fit that solves
scale, rotation and translation *together*:

```python
scale = math.hypot(a, b)          # ned.py:116  ← scale is a FREE PARAMETER
```

This is Umeyama/Procrustes **with scale enabled**, not scale-locked Kabsch. It runs whenever **≥2 DXF
ref points + ≥2 GPS ref points** are supplied (`engine.py:462-478`), and the solved scale multiplies
**every** segment point (`engine.py:538`).

### Why the existing guards do not catch it

The only defense is `_assert_alignment_scale` (`server/routes/path.py:355-378`) with
**`SCALE_FIT_TOLERANCE = 0.25`** (`server/config.py:90`).

> **Anything in `[0.75, 1.25]` passes silently. A 2 m square is accepted anywhere from 1.5 m to 2.5 m.**

And the RMSE gate (`RMSE_MAX = 0.05`, `config.py:85`) **cannot help**. The code already admits this
in its own comment (`routes/path.py:360-361`):

> *"A 2-point fit is exactly determined so its RMSE is ~0 and the RMSE gate cannot catch this —
> this scale gate is the only defense."*

With an exactly-determined 2-point fit, **scale error is driven purely by RTK/survey noise in those
two reference points.** A few cm of noise on a short baseline is a percent-level scale error, applied
to every dimension of the drawing.

### Why cross-track RMS will never reveal this

Cross-track error measures distance **to the path**. It says nothing about whether the path is the
**right size**. A stretched path tracked perfectly reports **beautiful sub-2 cm RMS** while painting
the wrong square. **Our existing accuracy metric is structurally blind to this defect.**

### Fix

**Lock scale to 1.0.** A DXF is dimensionally authoritative; the survey should only ever tell us
*where* and *which way* — never *how big*.

1. Use **scale-locked Kabsch/Umeyama** (rotation + translation only) for the ≥2-ref-point path.
2. Keep the fitted scale as a **reported diagnostic** — if it deviates from 1.0 by more than a few
   ×10⁻³, that is a *survey quality alarm*, not something to silently absorb into the geometry.
3. Tighten `SCALE_FIT_TOLERANCE` from `0.25` to something physically defensible (e.g. `0.01`) for as
   long as free scale remains reachable at all.
4. Add a **validator invariant**: planned segment lengths must match source-DXF lengths within a
   tight tolerance. See §7.

### ⚠ Open question — is this live or latent?

The audit **could not determine** whether the field workflow actually uses the ≥2-ref-point
alignment path, or whether operators exclusively use the GPS-surveyed `mission_placement.py` flow
(which is **safe by construction** — translation only).

> **If the mobile app exposes "align with 2+ reference points," this is a LIVE field hazard and is
> P0. If operators only ever use surveyed placement, it is latent — still fix it, but it is not on
> fire.** Requires an operator answer or a check of the Expo/RN frontend
> (`/Users/dyx_a1/Vetri/temp/Three_Wheel_v2`, out of this repo).

---

## 3. 🔴 B — E-stop crashes on first call and *lies about it*

`server/emergency.py:71-72`:

```python
if self._controller is not None:
    async with self._controller._lock:      # ← _lock is None until lazily created
        self._controller.state = MissionState.ABORTED
```

`OffboardController._lock` is `None` until `_lifecycle_lock()` lazily creates it
(`server/offboard_controller.py:88-95`), and that lazy-init only ever runs from
`start_async` / `stop_async` / `abort_async` / `clear_mission_async`. `emergency.py` reaches for the
**raw attribute** instead.

### Failure scenario

Fresh boot → operator's **first** action is an e-stop (a completely plausible pre-flight safety
check) → steps 1–3 **succeed against the real FCU** (stop-path published, MANUAL set, disarmed) →
then `async with None:` raises `AttributeError`, uncaught → REST `/api/vehicle/estop` returns **500**,
and the Socket.IO handler **never emits `estop_result`**.

> **The rover actually stopped. The operator is told the e-stop failed.**
>
> That is the worst possible direction for this error to point: an operator who believes the e-stop
> did not take will escalate — physically approach the rover, or hit something else.

**No test covers this path** — `estop` / `emergency` appear nowhere in the server test suites.

### Fix

Call `self._controller._lifecycle_lock()` instead of touching `_lock` raw, **and** wrap the
state-update in `try/except` so a state-machine hiccup can never suppress the e-stop *response*.

### Related — e-stop can also be queued behind a DXF parse

`server/routes/path.py:747` calls `parse_dxf(fpath)` **directly inside an async handler**, not
wrapped in `asyncio.to_thread` — even though the *same parse* is correctly offloaded 286 lines
earlier at `routes/path.py:461`. Uploads are capped at 5 MiB; a complex CAD parse blocks the single
event loop, so the 10 Hz telemetry loop misses ticks **and Socket.IO `emergency_stop` requests queue
behind the parse.**

**Fix:** wrap line 747 in `asyncio.to_thread(...)`. One line.

---

## 4. 🔴 C — Controller death is invisible; the watchdog trusts a dead process

`_cb_rpp_debug` (`server/ros_node.py:411-432`) writes `xtrack_m`, `rpp_state`, `speed_m_s` and —
critically — `pose_age_ms` straight from the RPP node's **self-reported** `data[6]`, with **no
independent receive-time tracking**. This is inconsistent with pose / global-pos / GPS-fix, which all
*do* track `_recv_time` (`server/ros_node.py:199-201`).

### Failure scenario

`rpp_controller` dies while in `TRACKING` (code 1 — **not** in `RPP_UNHEALTHY_CODES`,
`server/config.py:67`) with a low last-reported `pose_age_ms`.

Every downstream check now **freezes at "healthy"**. The watchdog at `server/main.py:408-412`:

```python
code in RPP_UNHEALTHY_CODES or pose_age > POSE_STALE_MS or connected is False
```

…never trips — **because it is evaluating staleness using data the dead process last wrote.**

The operator app then displays internally contradictory telemetry:

```
mode: MANUAL          ← live, from /mavros/state (PX4 failsafe fired correctly)
rpp_state: TRACKING   ← frozen, from a dead process
pose_age_ms: 12       ← frozen, from a dead process
```

At a glance that reads as **"still autonomously driving."**

**Fix:** add `_rpp_debug_recv_time` (mirror the existing pattern), expose `rpp_debug_age_ms` in
`get_state()` / telemetry, and fold it into the `unhealthy` check at `server/main.py:408-412`.

### Companion — freshness is computed but never streamed

`local_pose_age_ms`, `global_position_age_ms`, `gps_fix_age_ms`, `pose_global_skew_ms` are **already
computed** (`server/ros_node.py:473-491`) but consumed **only** as a one-time pre-mission placement
gate. None appear in the 100 ms telemetry payload (`server/main.py:325-353`), in `TelemetryData`
(`server/models.py:77-109`), or in `GET /api/telemetry/latest`.

**Scenario:** RTK/NTRIP drops mid-mission. Local EKF pose keeps updating from wheel odometry, so
`pose_age_ms` / `rpp_state` stay "healthy" — but `lat`/`lon`/`gps_fix_name`/`hrms` are now stale with
**no age exposed**. The operator cannot distinguish *"RTK_FIXED now"* from *"was RTK_FIXED five
minutes ago."*

*(Partial mitigation exists: RPP sets `rpp_state=RTK_WAIT` (4, unhealthy) on fix degradation — but
that safety net depends on RPP being **alive**, which is exactly what finding C says we cannot
detect.)*

**Fix:** add the four already-computed fields to the telemetry model. Low effort, high value.

---

## 5. 🔴 D — **UNVERIFIED, VERIFY FIRST:** does a crash-restart re-run the mission from run 0?

`_path_cb` **always** calls `self._apply_run(0)` (`src/rpp_controller_node.py:733`). There is **no
persisted `run_idx` / progress state anywhere in the node.** `/path` is subscribed **TRANSIENT_LOCAL**
(`:549-554`, `:582`).

**If** the production `/path` publisher (the FastAPI server) uses transient-local durability and
publishes once per mission, then:

> mid-mission crash → `rpp_start.sh` restarts the node in ~2 s → node re-subscribes → **receives the
> same latched Path** → `_apply_run(0)` → **starts tracking run 0 from the rover's current physical
> position** → potentially **drives back across already-marked ground.**

That converts *"the node restarts safely"* (finding: it does) into *"the rover unexpectedly drives
back through the mission"* — which is a **bigger field risk than any individual crash in this
document.**

The auditing agent could not confirm this because the publisher is server-side, outside its assigned
scope. **This is statically checkable and must be settled before the next field run.**

**Verify:** the QoS durability of the server-side `/path` publisher, and whether the server
re-publishes on RPP restart.

---

## 6. 🟠 E — Two one-line param crashes reachable from `ros2 param set`

**There are ZERO `ParameterDescriptor` range constraints across all five nodes** (verified: zero hits).
Every `declare_parameter` accepts whatever a live `ros2 param set` sends.

| Param | Site | Bad value | Result |
|---|---|---|---|
| `max_linear_decel` | `rpp_controller_node.py:2768-2771` (**every tick, every profile**), also `:2670-2678` | `0.0` | `ZeroDivisionError` → node dies → crash-loops every ~2 s |
| `a_lat_max` | `rpp_controller_node.py:3043-3044` — `sqrt(a_lat_max / kappa_speed)` | negative | `ValueError` (domain). Guard only checks `kappa_speed > 1e-9`, never the radicand's sign |

**Compounding:** **no node has `try/except` in ANY timer or subscriber callback** — `main()` catches
only `KeyboardInterrupt` (`rpp_controller_node.py:3652-3670`; same in all 5 nodes). Any uncaught
exception propagates out of `rclpy.spin()` and **kills the process.**

### Fix (surgical, zero tracking-behaviour change)

1. **Clamp at read:**
   ```python
   max_decel = max(1e-3, float(self.get_parameter("max_linear_decel").value))
   a_lat_max = max(0.0,  float(self.get_parameter("a_lat_max").value))
   ```
2. **Wrap each node's timer/subscriber callback body** in a top-level
   `try/except Exception: log + safe-stop`.

> This converts *"unhandled exception kills the process"* → *"one bad tick logs a traceback and the
> next tick retries"* — a **categorically different failure mode** for a field robot, and the single
> highest-leverage stability change across all five files.

### Latent — unbounded recursion

`_control_segment_profile` **calls itself** on corner acceptance (`:2481-2484`) rather than looping.
O(1) per cycle on sane paths, but a degenerate planner output with many consecutive
sub-`segment_corner_acceptance_radius` (0.05 m) segments would recurse once per segment in a single
tick, with **no depth cap**. Convert to a `while` loop or cap depth at `len(self._path)`.

---

## 7. 🟠 F — The June densification fix did NOT cover `shape_grouping` (**reproduced by execution**)

The auditing agent **ran the actual code** — two 2 m LINE edges of a square, densified at 5 cm, then
grouped. Result:

> **Alternating 5 cm / 10 cm spacing.** Every other interior waypoint silently dropped.

**Cause:** `_merge_chain` (`path_engine/optimizers/shape_grouping.py:174-222`) drops any point within
`tol` of the previously emitted point. `group_join_tol_m` defaults to **`0.05`** (`engine.py:160`) —
**numerically identical to `mark_spacing = 0.05`** (`engine.py:143`, never overridden by the server).
So legitimate 5 cm densified points look like duplicates and get culled.

This is the **same `tol == mark_spacing` pattern** recorded as fixed at `aac121d` (2026-06-20) —
**that fix did not cover this code path.** `_merge_chain` has **no unit test**.

**Trigger:** only when a shape is authored as **multiple connected LINE entities** rather than one
closed LWPOLYLINE — a **very common CAD export style**.

**Impact — read carefully:**
- ✅ Total length is preserved **exactly** (4.0 m for two 2 m edges).
- ✅ The shared corner vertex is **not** displaced.
- ❌ **Does NOT resize the square** — this is *not* a dimensional defect.
- ❌ **Does violate the ≤5 cm spacing invariant** → matters for **spray metering**, not dimensions.

**Fix:** decouple the tolerances — `group_join_tol_m` must be **strictly smaller** than
`mark_spacing` (e.g. `0.005`), and add a `_merge_chain` unit test asserting uniform spacing.

### Validator has no net for any of this

`path_engine/validator.py` checks counts, bounding box (only warns > 1000 m — far too coarse to catch
a 25% scale error on a 2 m shape), turn radius, max-gap (upper bound 0.5 m — the 10 cm gaps pass
silently), and self-intersections.

> **There is NO check comparing planned geometry against source-DXF dimensions, and NO
> lower-bound/uniformity check on waypoint spacing.** Defects **A** and **F** have no automated
> detection net.

**Add two invariants:**
1. **Dimensional:** every planned segment's length matches its source-DXF length within ~1 mm.
   *(This is the automated form of the operator's "2 m must be 2 m" criterion — it should be a test,
   not a hope.)*
2. **Spacing uniformity:** all MARK intervals within `[0.5·spacing, 1.0·spacing]`.

---

## 8. Untested critical paths

- 🔴 **`twist_to_setpoint_node` has ZERO real test coverage.** `test_p05_yaw_setpoint.py`
  **re-implements** the yaw formula in the test file rather than importing the node — so the NaN/Inf
  rejection (`:179-187`), the staleness failsafe (`:215-230`), and `type_mask` selection (`:271-287`)
  are **never actually executed by any test.**
  > **The node that owns the entire OFFBOARD heartbeat contract is our least-tested node.**
- **No test feeds a NaN pose** from `/mavros/local_position/pose` into `_control_loop`. Since NaN
  comparisons are always `False`, the jump-detection guard (`:2876`) would **silently not trigger** —
  it relies entirely on `twist_to_setpoint`'s downstream `isfinite` check. Untested end-to-end.
- **No test for the `max_linear_decel` / `a_lat_max` crash paths** (§6). A one-line regression test
  would have caught both.
- **No test for e-stop** anywhere in the server suites (§3).
- **No test for `_merge_chain` / `group_join_tol_m`** (§7).
- **No test for the closed-loop seam + stale-hint combination** (jump-skip then resume on a circle).

---

## 9. Prioritized plan

> **Sequencing principle: stabilize, then trust the geometry, then improve tracking, then chase
> speed.** Do not tune tracking on a stack that can crash-loop or silently mis-scale — you will be
> tuning against noise.

### Phase 0 — Verify before touching anything (hours)

| | Action |
|---|---|
| **0.1** | **Settle finding D.** Check the server-side `/path` publisher QoS durability + republish-on-restart. If a crash really re-runs run 0, **this is the top priority in the document.** |
| **0.2** | **Answer the §2 open question:** does the field workflow use ≥2-ref-point alignment, or only surveyed placement? This decides whether **A** is live or latent. |
| **0.3** | Confirm GNSS heading lock in the field (read-only): QGC → MAVLink Console → `listener estimator_status_flags -n 1` → want `cs_gnss_yaw: True`, `cs_gnss_yaw_fault: False`. |

### Phase 1 — Stability (small, surgical, no behaviour change)

| | Action | Ref |
|---|---|---|
| **1.1** | Fix e-stop `_lock` → `_lifecycle_lock()` + try/except | §3 |
| **1.2** | `asyncio.to_thread` the DXF parse at `routes/path.py:747` | §3 |
| **1.3** | Clamp `max_linear_decel` and `a_lat_max` at their read sites | §6 |
| **1.4** | Wrap every node's timer/subscriber callback in `try/except → log + safe-stop` | §6 |
| **1.5** | Add `_rpp_debug_recv_time` + `rpp_debug_age_ms`; fold into the unhealthy check | §4 |
| **1.6** | Stream the 4 already-computed freshness fields in telemetry | §4 |
| **1.7** | Per-sid `try/except` in `_emit_authenticated` (one bad sid currently drops the tick for **all** clients) | comms audit |

### Phase 2 — Geometry trust (**the 2 m criterion**)

| | Action | Ref |
|---|---|---|
| **2.1** | **Scale-lock the alignment fit** (Kabsch, rotation+translation only); report residual scale as a survey-quality alarm | §2 |
| **2.2** | Tighten `SCALE_FIT_TOLERANCE` 0.25 → 0.01 while free scale remains reachable | §2 |
| **2.3** | Decouple `group_join_tol_m` (0.05 → 0.005) from `mark_spacing` | §7 |
| **2.4** | **Add the two validator invariants**: dimensional (segment length vs DXF, ~1 mm) + spacing uniformity | §7 |
| **2.5** | Add the golden test: **2 m square DXF in → assert every side is 2.000 m ± 1 mm, all intervals ≤ 5 cm** — the operator's criterion, as CI | §7 |

### Phase 3 — Test the untested

| | Action | Ref |
|---|---|---|
| **3.1** | Real tests for `twist_to_setpoint_node` (NaN rejection, staleness failsafe, `type_mask`) — **import the node, don't re-implement it** | §8 |
| **3.2** | NaN-pose end-to-end test through `_control_loop` | §8 |
| **3.3** | Param-crash regression tests; e-stop test; `_merge_chain` test | §8 |

### Phase 4 — Tracking quality (**RPP only — no MPC**)

| | Action |
|---|---|
| **4.1** | **Unify the lookahead law.** Today: `ld_gain·v + xt_ld_gain·|e⊥|` (`:2975`) with a curvature **floor** bolted on separately (`max(l_d, 0.35/κ)`, `:2982-2984`) → it **snaps** at the floor instead of anticipating. Move to `l_d = clamp(k_v·v + k_κ/√κ_preview + k_e·|e⊥|, l_min, l_max)` using the **already-computed** `preview_curvature` / `_max_preview_curvature` infrastructure. Additive, only widens existing floor logic, trivially A/B-testable. **Low risk, real gain.** |
| **4.2** | Regression test for the closed-loop seam + stale-hint case. |
| **4.3** | **Do NOT touch `yaw_rate_feedback_gain`** — it is a documented NO-OP in velocity mode. |

### Phase 5 — Speed (only after 0–4 are green)

Then, and only then, take up **SPD-T1** (1.0 m/s line / 0.6 m/s arc) from doc `01`. Prereq stands:
verify RoboClaw top speed vs `RO_MAX_THR_SPEED = 0.9`.

---

## 10. Explicitly NOT recommended

- ❌ **Do not replace RPP with MPC.** Operator decision; RPP stays.
- ❌ **Do not enable dual-antenna heading** — already enabled (`EKF2_GPS_CTRL=15`).
- ❌ **Do not chase the arc/smooth structural floor.** It is a velocity-mode OFFBOARD limitation
  (PX4 discards `yawspeed`); no RPP-side change closes it. Production correctly routes all tracking
  through segment/stop-pivot.
- ❌ **Do not worry about UTM scale distortion.** There is no projected CRS in the pipeline.

---

## 11. Doc-hygiene follow-ups

- `docs/Architecture/FINAL_ARCHITECTURE.md:70` and
  `docs/Researches/COMMERCIAL_ROVER_RESEARCH/Tasks/T4_sensor_fusion.md:57` both state the UM982 is on
  **`/dev/ttyUSB0`**. The param dumps prove **TELEM1** (`GPS_1_CONFIG=101`). The `ttyUSB0` claim looks
  miscopied from the *proposed* uXRCE-DDS bridge in `docs/Researches/Hybride_Archi_Decision.md`.
  **Fix both — they will mislead anyone debugging GNSS.**
- `CLAUDE.md` calls `PX4_DXP_Tracker.xlsx` the "Full FCU set." **It is not** — it omits
  `EKF2_GPS_CTRL`, `GPS_1_CONFIG`, `GPS_YAW_OFFSET`, `EKF2_GPS_YAW_OFF`. The raw
  `PX4_params/*/*.params` dumps are the real source of truth. Update the pointer.
