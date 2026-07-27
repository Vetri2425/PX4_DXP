# PX4_DXP — Open Bug Register

3WD marking rover · branch `Upgrade_Spray` · **701 tests pass**
Compiled 2026-07-22 from six field runs of `tes_cross_line`; **updated 2026-07-22 post-deploy**.
**2026-07-27:** added **A11** (bare lat/lon point-CSV validation swallowed by the survey fallback) —
source-verified and reproduced, fix deferred to after the Monday 2026-07-28 field session.
Added **A12** (FCU parameter capture emits `None` for all 13 tracked params) — found during the
WENC session, **blocks the real WENC A/B**; full report `docs/WENC_AB_2026-07-27.md`.
**Source-verified 2026-07-23** (3 parallel agents, every item read against the tree): all claimed
fixes confirmed landed; all open bugs confirmed present. Three corrections applied below —
A9 location, B4 classification, C3 layer scope — flagged inline as ⟲ **[src-verified 07-23]**.

> This markdown is the **source**. `OPEN_BUGS.pdf` is generated from it —
> edit here, then regenerate. The PDF was previously hand-maintained with no source,
> which is why it drifted.

**Severity** is impact on the marking result, not effort.
**Verified in-env** ≠ **field-proven**. The register says which.

---

## A — CODE LEVEL

| # | Bug | Location / parameter | Evidence | Sev | Status |
|---|---|---|---|---|---|
| **A1** | `source_file` never populates → §8 ABSOLUTE ACCURACY can never run. Two different things named `source`: the stager writes a filename **string**, the reader expects a **dict**. | `routes/path.py:1179` wrote `"source": result["source"]`; `bag_autorecord.py:412` read `source.get("filepath")` | §8 reported `unavailable` on all 6 bundles; the entire absolute analysis had to be done by hand. | HIGH | ✅ **FIXED `e3939e3`** · deployed · Jetson-verified: §8 gate **RUNS** |
| **A2** | `must_hit` lost on every non-staged load. Only the staged path passed it, so vertex provenance never reached those missions. | `mission_loading.py:100`, `routes/mission.py:62`, `sockets/events.py:119` (vs `routes/path.py:1279`) | `mission_20260722_161014` published `must_hit = 0` on a 48-pt path. | HIGH | ✅ **FIXED `21a8b05`** · deployed · Jetson: preview 64 pts / **4 must-hit**; matches staged route point-for-point. ☐ live `/path z=3` unconfirmed |
| **A3** | EKF position resets ignored. Origin is fixed, but the vehicle estimate can jump against it mid-mission and nothing notices. | `xy_reset_counter`, `delta_xy` — zero references in `src/` or `server/` | Source-verified in `ekf_helper.cpp`: GNSS resets project through the origin and increment the counter. `dd166b3` does not cover this. | HIGH | ◐ **FIX STAGED — gated, default OFF.** `rpp_controller_node.py` `ekf_reset_compensation` (+`ekf_reset_max_absorb_m=0.30`): the existing P0.2 jump IS the reset delta (MAVROS can't supply `delta_xy` — `odometry` plugin denylisted), absorbed into `_ekf_reset_offset` and subtracted from the tracking pose so net xtrack ≈ 0 instead of a kink. OFF = byte-for-byte frozen P0.2. Touches frozen RPP → **needs a named A/B run**. Test: `src/test_ekf_reset_compensation.py` (rclpy-gated, runs in-env only). ✅ committed + deployed (default OFF, byte-for-byte frozen until opted in) · ☐ **needs a named A/B run to enable** · ☐ field-unverified |
| **A4** | `outcome == COMPLETE` on partial runs. Only a missing `recorder_end` marked INCOMPLETE; path coverage was never checked. | `bag_autorecord.py:550-560` | Two runs covered 24/64 and 76/86 waypoints — both reported COMPLETE. | HIGH | ✅ **FIXED `5f820be`** · deployed · reproduced both bad runs unprompted. ☐ `PENDING` marker needs one real mission |
| **A5** | No `/spray/session_config` subscription. No mode can reach the spray node; dash and point remain schema-only. | `spray_controller_node.py` | Blocks Spray V2 Phase C and D entirely. | HIGH | ✅ **CLOSED 2026-07-23 @`0545fa1` — B0 transport built + deployed.** Node subscribes (RELIABLE+TRANSIENT_LOCAL, fail-static); server publishes on load + explicit cleared on clear; `PathPlanRequest.spray_mode` staged. Bench-verified: live node switches continuous↔dash, topic 1-pub/1-sub. ☐ point mode still Phase D (reserved, not emitted). |
| **A6** | Spray plan Rev 3 stale vs shipped code. §5/§7.2/§7.3 describe a speed gate `6523a84` removed, and deny an RPP coupling that now exists. | `SPRAY_CONTROLLER_V2_PLAN.md` vs `spray_controller_node.py:877` | Implementing Phase C against it repeats the `16480d9` false-fix pattern. | HIGH | ✅ **CLOSED 2026-07-23 — plan now Rev 4** (§5 gate table around pivot-state gate; §7.2 dash deferral vs `CORNER_ALIGN` + open per-segment-dash decision; §7.3 node-side point pivot-gate exemption). ☐ one operator decision still owed: per-segment dash reset vs continuous-across-mission. |
| **A7** | Cross-session origin repeatability. Origin re-derived at each EKF restart from a fresh first fix; needs `SET_GPS_GLOBAL_ORIGIN` pinning at boot. | not implemented anywhere | One degE7 quantisation step (≤1.11 cm) of offset **per session**. Within a session, identical. | MED | ☐ **OPEN** — see C1: within-session is now proven, cross-session is not |
| **A8** | CPU contention — executor mismatch. `MultiThreadedExecutor` with `MutuallyExclusiveCallbackGroup` gives zero parallelism. | `ros_node.py` executor setup | 50–65% of one core. Arm/disarm safety blocks a naive swap. | MED | ☐ **OPEN** |
| **A9** | `must_hit` under-marked at entity junctions. A square built from 4 LINE entities flags only **2 of its 4 corners**; independent of corner smoothing. | ⟲ **[src-verified 07-23]** actual defect is `path_engine/optimizers/shape_grouping.py::_merge_chain` (~`:206-213`): the merged composite copies `vertex_indices` from the **`head`** chain member only, so the other segments' corners are dropped, while `points` is the full concatenation. The `engine.py:1039-1124` "merge/dedup" block cited earlier is **downstream** — it only consumes the already-incomplete provenance. | Found while verifying A2. Reproduces identically on the staged route, so it predates `21a8b05`. **`64c12ff` never touched `shape_grouping.py` → the vertex-drop fix is quietly incomplete for multi-entity shapes.** Pinned by `server/test_staged_endpoints.py:408-425` (test comments call it "a separate engine defect"). | HIGH | ✅ **FIXED 2026-07-23** — `_merge_chain` + `decompose_line_chain_to_edges` now remap `vertex_indices`/`control_indices` into the composite/edge index space. 4-LINE square keeps **4/4** corners. 3 regression tests in `test_vertex_provenance.py` (proven to fail on pre-fix source). 406 path_engine + 168 server tests pass. ☐ field-unverified |
| **A12** | **FCU parameter capture records `None` for every tracked parameter — every bag ever recorded has NO parameter provenance.** A param A/B cannot be proven from its own bags: which side of the test a run belongs to rests entirely on the operator's word and the folder name. | `as_run_config.fcu_params.values` in `manifest.json` — all **13** declared keys are `null`: `EKF2_WENC_CTRL`, `RO_YAW_P`, `RO_YAW_RATE_LIM`, `RO_MAX_THR_SPEED`, `RD_TRANS_TRN_ARM`, `RD_TRANS_ARM_TRN`, `RBCLW_COUNTS_REV`, `NAV_ACC_RAD`, `COM_OF_LOSS_T`, `PWM_AUX_FUNC1`, `PWM_AUX_MIN1`, `PWM_AUX_MAX1`, `PWM_AUX_DIS1`. The schema slot exists and is emitted; only the fetch is unwired. | Found 2026-07-27 during the WENC session (`docs/WENC_AB_2026-07-27.md`). Verified `null` on both blocks and every one of the 7 bundles in `bags/27_07_2026/`. The session's `EKF2_WENC_CTRL=1` vs `=0` labelling is therefore unverifiable from the data — the FCU reboot between blocks IS provable (`gp_origin` moved 4.76 m) but the parameter value is not. | HIGH | ☐ **OPEN — blocks the real WENC A/B.** Fix: populate the values via MAVROS `/mavros/param/get` (or the param plugin's cached map) at recorder start, after MAVROS is up; record fetch failures explicitly rather than emitting `null`, so a missing value is visibly missing instead of silently absent. Must land before the arc-block A/B or that test inherits the same hole. |
| **A11** | **Bare lat/lon point-CSV validation is unreachable — every rejection is silently swallowed and the file is re-parsed with `dwell_s` and `mark` DISCARDED.** A row the parser correctly refuses comes back as an accepted mission with different data. Worst case: points explicitly declared `mark=0` (transit, no paint) are returned `mark=True` and **will be painted**. | `point_ingest.py:349-365` — `parse_point_gps_csv_text` wraps the bare parse in `try: … except ValueError as bare_err:` and falls through to `_survey_latlon_rows(text)` (`:302-324`), which hard-codes `dwell_s=None, mark=True` for every point (`:322`). The fallback **always succeeds on these files**: `survey_csv.py:68-69` lists `"lat"`/`"lon"` as survey aliases, so a bare `lat,lon,dwell_s,mark` header satisfies `_header_map_from_line` (`:93-107`) too. `bare_err` is therefore never re-raised. The per-row validation at `:242-254` and the mark-aware policy at `:133-149` are correct — they are simply thrown away. | Found 2026-07-27 while generating a test CSV from `tes_cross_line_2.dxf`. Reproduced on four inputs, all **ACCEPTED** when they must be rejected: `dwell_s=0.0,mark=1` → `2.0` (violates `:142` "must be > 0 when mark=true"); `dwell_s=999` → `2.0` (violates the 60 s `max_dwell_s`); `dwell_s=abc` → `2.0` (not a number); `dwell_s=abc,mark=0` → **`mark=True`, dwell 2.0** (transit points become painted points). A valid file with `mark=0` parses correctly, which proves the bare path is live and only the error branch is broken. | HIGH | ☐ **OPEN — deferred by operator 2026-07-27 to after the Monday field session.** Fix (not applied): make the fallback conditional on *shape*, not on failure — only try `_survey_latlon_rows` when the header is **not** a bare `lat,lon…` header, and re-raise `bare_err` otherwise. A validation error must never be answered with different data. Needs regression tests for all four rows above. |
| **A10** | NTRIP subprocess logs a spurious `[ERROR] NTRIP error: [Errno 9] Bad file descriptor — reconnect #N` on every `rover-server` restart, and bumps `_reconnect_count`. Cosmetic — the socket lifecycle is correct; it is a **teardown artifact**, never a mid-run RTK fault. | `ntrip_rtcm_node.py`: on SIGTERM, `_handle_signal` (`:531-537`) → `request_stop()` → `_close_active_socket()` (`:173`) closes the fd **from the signal thread** while the `_run` worker is blocked in `sock.recv(4096)` (`:449`, guarded only for `socket.timeout` at `:450`). The `OSError(EBADF)` falls through to the generic handler (`:477-484`) which logs at ERROR + increments the counter, then `:502` `if self._stop_event.is_set(): break` exits cleanly. **Node is a child of `rover-server`, not `px4-dxp`** — spawned/terminated by `RTKManager` (`server/rtk_manager.py:53`, `:185`), which is why a `rover-server` restart cycles it. `_close_active_socket()` is reachable **only** via `request_stop()` (signal handler / `destroy_node`), so this EBADF can occur only on the stop path. | Observed 2026-07-23 right after the joystick `rover-server` restart; the fresh node reconnected normally. RTK health was independently unaffected. GGA-send races log a different message (`"Failed to send GGA"`, caught at `:285`); real network drops give ECONNRESET/ETIMEDOUT/gaierror, not EBADF. | LOW | ☐ **OPEN — cosmetic.** Fix (not applied): in the `except` at `ntrip_rtcm_node.py:477`, when `self._stop_event.is_set()` log at INFO/DEBUG and skip the `_reconnect_count` bump — or catch `OSError` around the `recv` and treat it as a clean break while stopping. |

### Survey tolerance — closed this cycle
`SURVEY_TOL_CM = 2.5` was a constant in `analyze_mission.py` that decided whether §7 FAILs.
Now `PathPlanRequest.survey_tolerance_m` → staged → manifest → analyser, precedence
`--survey-tol-cm` > staged > default, with the source printed in every report.
✅ **`7d77565`**, deployed. ☐ Not yet sent by the mobile frontend.

---

## B — CALIBRATION / HARDWARE (QGC parameters)

**None of these are code. All need the rover and a tape measure.**

| # | Bug | Location / parameter | Evidence | Sev | Status |
|---|---|---|---|---|---|
| **B1** | **Dual-antenna heading bias — largest open error.** A constant heading error makes pure pursuit settle on a line *parallel* to the path. | `EKF2_GPS_YAW_OFF` = 180.0 (a round number — assumed, not measured), `GPS_YAW_OFFSET` = 180.0 | 5 of 6 runs drove **left**; B spread 3.74 cm, means −0.47 to −2.00 cm. `offset ≈ lookahead × sin(err)`: **2 cm needs only ~2.2°** at 0.52 m lookahead. | HIGH | ☐ **OPEN — do this first** |
| **B2** | Differential wheel asymmetry. One counts-per-rev value for two wheels that may not match; same one-sided signature as B1. | `RBCLW_COUNTS_REV` = 148000, `RD_WHEEL_TRACK` = 0.470 | Indistinguishable from B1 in the data. **The cheap separator:** drive the same line forward then backward — a heading offset flips sign, a wheel-scale error does not. | MED | ☐ **OPEN** |
| **B3** | GNSS antenna lever arm. Antenna off the vehicle centreline gives a fixed lateral offset; roll amplifies it. | `EKF2_GPS_POS_X/Y/Z` = 0 / 0 / −0.4 — **Y=0 asserts it is centred** | Reference rig: 1.934 m pole, 1.7° tilt = 5.7 cm lateral if uncompensated. | MED | ☐ **OPEN** |
| **B4** | Nozzle offset uncalibrated. Every measurement to date describes where the **antenna** went, not where paint went. | ⟲ **[src-verified 07-23]** `nozzle_forward_offset_m` / `nozzle_lateral_offset_m` are **not dead code** — they are declared at `spray_controller_node.py:372-373` and **applied** via a real body-frame rotation (`_nozzle_position_ned`, `:178-196`) feeding the spray decision at `:798-804`. Both just **default to 0.0** (never measured). | Blocks error budget 4. ⚠ `SPRAY_NOZZLE_OFFSET_PLAN.md` §3.6 xtrack-gate fix is **mandatory before any non-zero offset**: the gate at `spray_controller_node.py:292-297` compares **raw** xtrack against `max_xtrack_error_m` (0.10 m) with **no offset correction**, so any lateral offset ≥ 10 cm permanently blocks spraying. | HIGH | ☐ **OPEN** |
| **B5** | Physical re-survey never done. No log can answer where the paint landed. | process gap — Emlid RS3 | Survey CSV shows `Samples=1`, 1.7 cm single-epoch RMS. **Use averaging (10–30 s/point) when re-surveying.** | HIGH | ☐ **OPEN** |

---

## C — SHIPPED BUT UNVERIFIED (code-proven, field evidence pending)

| # | Item | Evidence | Sev | Status |
|---|---|---|---|---|
| **C1** | Placement determinism (`dd166b3`) | Two loads of one staged mission on the Jetson, 2026-07-22: **max point-to-point difference 0.0000 cm** across 40 sampled of 64 waypoints. Was 0.39–1.53 cm before the fix. | HIGH | ✅ **CLOSED on hardware** (within-session only — cross-session is A7) |
| **C2** | Survey CSV ingest (`211244b`) | unit tests only; never run in the field — all missions to date were DXF | MED | ☐ open |
| **C3** | POINT-layer control points | ⟲ **[src-verified 07-23]** the parser is **layer-agnostic** — `dxf_parser.py:702-782` (`_annotate_control_points`) matches **any** POINT entity within `CONTROL_SNAP_M=0.01` of a polyline vertex, not specifically a "Points" layer. The 41-vertex/3-control case is a **commit-message demo, not a committed test**; real coverage is a 4-vertex case in `test_core.py:91-112`. Synthetic only; never on a dense real drawing. | MED | ☐ open |
| **C4** | §8 absolute accuracy auto-runs | was blocked by A1 — **unblocked**; gate verified RUNS on the Jetson, but has not yet auto-run on a freshly recorded bundle | HIGH | ☐ open (needs one drive) |
| **C5** | segment→smooth ~5 cm seam | visual only; may already be fixed by `64c12ff`. Check `/rpp/debug[0]` vs `[40]` at the profile flip. | MED | ☐ open |
| **C6** | Spray pivot-state gate (`6523a84`) | replay-verified 157→53; field-unverified | MED | ☐ open |
| **C7** | Endpoint under-run | rover rests **0.9–2.0 cm short** every run; overshoot never above +0.4 cm | MED | ☐ measured, no fix attempted |
| **C8** | Per-run wander | residual **RMS 1.1–1.8 cm** between runs after removing the mean offset | MED | ☐ measured, untouched |
| **C9** | Closed-loop endpoint projection snap-back | `_project_onto_path` (`rpp_controller_node.py`) is a **global-closest** search with **no monotonic-progress window**. On a closed shape whose start/end vertex is coincident, near the finish the nozzle can sit closer to the *first* leg than the last, so the reference point snaps back to s≈0 (dist-to-goal jumps back to full length). Spray can latch ON past the finish and the controller chases the wrong segment. **Distinct from A3** (there the estimate steps under a fixed path; here the reference point jumps across a seam) but the same "invisible kink" family, and related to the C5 segment→smooth seam. | narrow, geometry-dependent, **PRE-EXISTING**; demonstrated in analysis of a closed all-MARK loop, not yet seen in a field bag | MED | ☐ open — fix = deferred **monotonic-progress window** in `_project_onto_path`; touches frozen RPP → needs A/B |

---

## Recommended order

1. **B1 heading test** — largest error, **no driving needed**, one QGC parameter.
2. ~~A1, A2, A4~~ — ✅ done and deployed; the next run batch records clean provenance.
3. ~~C1 two-load determinism~~ — ✅ closed, 0.0000 cm.
4. **B4 + B5** — the only route to knowing where paint lands.
5. **A6 → A5** — unblocks spray modes (write Rev 4 *before* touching Phase C code).
6. ~~**A9** — under-marked junction vertices~~ — ✅ **FIXED 2026-07-23** (`_merge_chain`/`decompose_line_chain_to_edges` provenance remap); completes what `64c12ff` left undone for multi-entity shapes.

---

## Field-proven this cycle

- **Vertex deletion** (`64c12ff`) — 64→4 conditioned points, was 64→2.
- **Georef north-scale** (`e483d53`) — 2.3255 m against a 2.3255 m WGS84 geodesic, residual 0.000 mm.

**Tracking is sound** at 0.33–0.88 cm per surveyed vertex with extensions.
**The remaining error is offset, not tracking** — which is why B1–B3 sit above every code item.

---

## Standing caution

Three bugs shipped in the 2026-07-22 cycle because a test's ground truth **mirrored the bug**:
bare-vertex input bypassing densification; haversine using the same wrong radius as the
projection; NavSatFix decoded by tests that bypassed the CDR reader. Be sceptical of any test
claiming to validate against "truth" — and check a new test **fails** before the fix lands.
