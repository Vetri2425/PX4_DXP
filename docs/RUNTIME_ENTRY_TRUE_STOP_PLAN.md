# Runtime-Entry TRUE-STOP Plan (stop-first, field-test-ready)

**HEAD:** `5f35c7a` (`feat/entry-pivot-recenter`)
**Scope:** companion RPP only (`src/rpp_controller_node.py`). No PX4/QGC/firmware. No ArduRover.
**Goal:** make the runtime-entry stop park like the segment intra-corner stop, then advance. Pivot is observation-only tomorrow.

---

## A. Problem statement

Run 0 of a `GPS_SURVEYED` mission is a `smooth`, `runtime_entry=True` acquisition leg. At the entry→MARK boundary the stop is served by **two** divergent paths, both wrong:

1. **Pure-zero branch** (`_entry_pure_stop_hold`, `rpp_controller_node.py:1948`): once inside `segment_entry_pure_stop_dist_m` (0.10 m) it commands **exactly (0,0)**. PX4 velocity-OFFBOARD **coasts on a zero setpoint** (no brake), and certification needs `≤0.005 m/s` (inside the RTK/EKF noise floor) held 1.0 s with **no stale fallback** → non-convergent deadlock.
2. **Generic branch** (`_corner_stop_satisfied`, `:5003`, gate `0.08 m/s`/`0.30 s`): certifies while the rover still creeps at **5–7 cm/s**, then `_advance_run(pre_stopped=True)` → `_run_alignment_hold` pivots from a moving rover → walks tens of cm past wp0.

The segment intra-corner stop works because it **actively brakes** (`_corner_brake_velocity`) to a genuine halt before certifying. Entry stop does neither cleanly.

**Non-goals (confirmed clean, do not reopen):** the t=0 init turn (forward-cone pure-pursuit, ~few cm slide); the post-stop profile switch (SMOOTH run0 → SEGMENT/SMOOTH next leg via `_advance_run`/`_run_alignment_hold`); the pivot/recenter geometry itself.

---

## B. Target behaviour — acceptance criteria (bag-measurable)

"True stop like segment," for the **runtime-entry→MARK boundary only**:

1. **No coast, no creep-drive inside the stop window.** Once `pos_error ≤ segment_entry_true_stop_dist_m` (0.10 m), the command is an **active brake opposing motion** (`_corner_brake_velocity`, cap 0.18 m/s) while `meas_speed > segment_stop_speed_threshold`, then a low-capped `_corner_hold_velocity` for the final cm. Never `(0,0)` while still moving; never `_smooth_capture_velocity` drive-to-point inside the window.
2. **STOP_CERT only when parked:** measured speed `≤ segment_entry_stop_speed_m_s` (**default 0.03 m/s = 3 cm/s**) **AND** yaw-rate `≤ segment_stop_yaw_rate_threshold` (0.05 rad/s) held continuously `≥ segment_stop_dwell_s` (**0.30 s**), **AND** position `≤ corner_position_tolerance_m` (0.02 m). Stale-velocity fallback caps the hold at `_CORNER_STOP_MAX_HOLD_S` (2.0 s) so a quiet topic cannot deadlock.
   - *Why 3 cm/s, not 1 cm/s:* the audit measured this rover's RTK/EKF noise floor at 0.5–2 cm/s; a 1 cm/s continuous-dwell gate flickers and reproduces the 0.005 deadlock. 3 cm/s sits just above the floor, equals the codebase's existing at-rest definition (`MISSION_COMPLETE_REST_SPEED_M_S=0.03`), and is **reachable because the rover is actively braked** through it (unlike the pure-zero coast). Fallback: if bags show walk-past at 3 cm/s, tighten this one param to 0.02.
3. **Position at cert ≤ 2 cm** from wp0; overshoot after the first ≤5 cm approach **≤ 5 cm** before cert (active brake bounds this).
4. **No ALIGN / no `_advance_run`** until (2)+(3) hold — already structural: `_advance_run(pre_stopped=True)` fires only on the `_corner_stop_satisfied` True branch.
5. **After cert:** run advances; next-leg profile = SEGMENT/SMOOTH via existing `_advance_run`/`_run_alignment_hold`. **Unchanged** (no ordering fix needed — `pre_stopped=True` carries `_corner_stop_complete`).

**Verify from bags** (`/rpp/stop_debug`, `/rpp/debug`, `/rpp/segment_debug`, pose):
- `stop_debug.phase`: HOLDING(1) → STOP_CERTIFIED(2), never BLOCKED(9) looping; `run=0` throughout HOLD, `run=1` at first ALIGN.
- `stop_debug.pos_error` at STOP_CERTIFIED ≤ 0.02; min pos_error ≥ (peak − 0.05).
- `debug[3]` (speed) and `velocity_local` speed at the STOP_CERTIFIED cycle ≤ 0.03.
- `debug[7]` state APPROACH/TRACKING during HOLD; `segment_debug` state = CORNER_STOP(5) during HOLD, CORNER_ALIGN(3) only after cert.

---

## C. Design — exact code plan

### Decision (one paragraph)
**Replace** the pure-zero entry stop with **segment true-stop parity**: route `RUNTIME_ENTRY_TO_MARK` through the *same* generic tail of `_hold_before_run_advance` that the working smooth RUN_BOUNDARY uses (active `_corner_brake_velocity` → low-cap `_corner_hold_velocity`, already present at `:1874–1910` and already active for run 0 because run 0 is `smooth`), and **certify on a parked speed gate** via the existing `_corner_stop_satisfied` with a single new threshold `segment_entry_stop_speed_m_s` (default 0.03). Delete the pure-zero branch, its latch, its helper, and its three dead params. No new state machine, no param matrix — one new gate whose default *is* the intended production behaviour, plus reuse of three field-validated helpers.

### Functions changed
1. **`_hold_before_run_advance`** (`:1748`)
   - Delete the pure-stop latch + dispatch block (`:1813–1827`): the `entry_pure_stop_dist` read, the `_entry_pure_stop_latched = True` set, the `if … _entry_pure_stop_latched: return self._entry_pure_stop_hold(...)`, and the trailing `self._entry_pure_stop_since = None`.
   - At the cert call (`:1829`), pass a parked-speed override for the entry boundary:
     ```python
     entry_cert = stop_reason == StopReason.RUNTIME_ENTRY_TO_MARK
     speed_override = (
         float(self.get_parameter("segment_entry_stop_speed_m_s").value)
         if entry_cert else None
     )
     if self._corner_stop_satisfied(position_ok=position_ok,
                                    speed_threshold_override=speed_override):
     ```
   - The HOLD tail (`:1860–1946`) is **unchanged** and now serves entry: run 0 is `smooth` so `_active_tracking_profile != "segment"` is already True → the true-stop active-brake block (`:1874`) runs for entry automatically.
2. **`_corner_stop_satisfied`** (`:5003`) — add optional override:
   ```python
   def _corner_stop_satisfied(self, *, position_ok=True, speed_threshold_override=None):
       ...
       speed_thresh = (speed_threshold_override
                       if speed_threshold_override is not None
                       else float(self.get_parameter("segment_stop_speed_threshold").value))
   ```
   Everything else (yaw-rate gate, 0.30 s dwell, 2 s stale fallback) unchanged.
3. **Delete** `_entry_pure_stop_hold` (`:1948–2036`) entirely.
4. **Delete** state vars `_entry_pure_stop_since`, `_entry_pure_stop_latched` (`:695–699`) and their resets in `_reset_corner_pivot_state` (`:4953–4954`).

### Params
- **STAY (now govern entry too, no change):** `segment_entry_true_stop_dist_m` (0.10, active-brake window), `segment_boundary_corner_handoff_m` (0.05), `corner_position_tolerance_m` (0.02), `segment_stop_yaw_rate_threshold` (0.05), `segment_stop_dwell_s` (0.30), `segment_brake_velocity_cap_m_s` (0.18).
- **NEW (one):** `segment_entry_stop_speed_m_s = 0.03` — parked cert gate for the entry boundary. Default = production true-stop. Declared next to the retired params (`:590`).
- **RETIRED (delete declarations `:579-592` for the pure-stop trio):** `segment_entry_pure_stop_dist_m`, `segment_entry_pure_stop_speed_m_s`, `segment_entry_pure_stop_dwell_s`. (If a field param file still pushes them, PX4 param-set to an undeclared name is a no-op warn, not a crash — safe. Note the retirement in the deploy checklist.)

### Pure-stop / true-stop resolution (no half-modes)
- **Pure-stop:** REPLACED (branch + latch + helper + 3 params deleted).
- **True-stop brake (`segment_entry_true_stop_dist_m`):** KEPT and now the entry stop's sole hold mechanism; cert extended with the parked gate. One clean mode.

### Enable / rollback knobs (no new flag)
- Behaviour is enabled by the existing `segment_entry_true_stop_dist_m > 0` (default 0.10). Set 0 → reverts to pure `_smooth_capture_velocity` capture.
- `segment_entry_stop_speed_m_s = 0.08` → reverts entry cert to the segment default (loose) without touching code.
- Full rollback = `git revert` of the single implementing commit.

---

## D. Implementation steps (PR-sized, ordered)

1. **Add param** `segment_entry_stop_speed_m_s = 0.03` with doc comment at `:590`; **delete** the three `segment_entry_pure_stop_*` declarations. *(declare block)*
2. **`_corner_stop_satisfied`**: add `speed_threshold_override=None` kwarg + the two-line threshold selection. *(`:5003`)*
3. **`_hold_before_run_advance`**: delete the pure-stop latch/dispatch block (`:1813–1827`); add the `entry_cert`/`speed_override` computation and pass `speed_threshold_override` into the `_corner_stop_satisfied` call (`:1829`). Leave the HOLD tail untouched. *(`:1748`)*
4. **Delete** `_entry_pure_stop_hold` (`:1948–2036`).
5. **Delete** `_entry_pure_stop_since` / `_entry_pure_stop_latched` declarations (`:695–699`) and resets (`:4953–4954`). Grep for any other reference and remove.
6. **Tests** (section below): rewrite `src/test_entry_pure_stop.py` → `test_entry_true_stop.py`; update `test_run_boundary_pivot.py` if it asserts entry pure-zero.
7. Run `python3 -m pytest src/test_entry_true_stop.py src/test_run_boundary_pivot.py src/test_rpp_entry_transit.py src/test_mission_start_align.py -q` (venv + PYTHONPATH per jetson_deploy_flow). Green before deploy.

Expected diff: ~1 param swap, ~3-line signature change, ~15-line branch deletion, ~90-line method deletion, ~4-line state cleanup. Net negative LOC.

---

## E. Field test protocol (STOP-FIRST — pass gate is stop only)

1. **Mission:** `GPS_SURVEYED`, entry distance **> 0.5 m**, next leg **segment** (2×2 square or a line) so SMOOTH→SEGMENT is visible in `segment_debug`.
2. **Start deliberately mis-headed** (90°–180°) so the entry leg is real.
3. **Bag** (autorecord or manual): `/rpp/stop_debug /rpp/debug /rpp/segment_debug /mavros/local_position/pose /mavros/local_position/velocity_local`.
4. **STOP PASS (fail-closed, all must hold):**
   - measured speed at STOP_CERTIFIED cycle ≤ **0.03 m/s**;
   - `stop_debug.pos_error` at cert ≤ **0.02 m**;
   - no post-closest growth > **0.05 m** before cert;
   - time from first `pos_error ≤ 0.05 m` to STOP_CERTIFIED ≤ **2.5 s** (active-brake + 0.30 s dwell + margin; longer ⇒ deadlock suspicion).
5. **If STOP FAIL:** halt campaign; debug stop only from bags (cmd speed `debug[3]`, measured `velocity_local`, `segment_debug` state, `stop_debug.pos_error` timeline). **Do not** tune pivot.
6. **If STOP PASS:** record pivot outcome as observation only:
   - pivot heading certs, pos stays ≲10 cm → note "stop done; pivot next-upgrade if needed";
   - pivot heading OK but pos walks → **separate ticket** after stop signed off. Do not expand scope.
7. **Runs:** minimum **4** — {square 0.6 m entry, square 1.5 m entry, line 1.0 m entry mis-headed 180°, circle 1.0 m entry (SMOOTH→SMOOTH next leg, checks the gate on a non-segment successor)}. 5th optional: very short entry (~0.3 m, criterion-F edge).

---

## F. Risks & rollback

- **Circle / smooth next-leg:** entry cert now parks harder; verify the SMOOTH→SMOOTH case (run 4) still advances (fallback: stale cap fires at 2 s). Mitigation: the parked gate + stale fallback are the same machinery segment corners already ship.
- **Very short entry (<3 cm):** `pos_error ≤ true_stop_dist` may be true from the first cycle; `_corner_brake_velocity` returns 0 below `stop_speed`, `_corner_hold_velocity` servos — no coast. Covered by run 5.
- **Collinear / non-alignment boundaries:** untouched — gated by `_next_run_requires_alignment()` before any of this.
- **3 cm/s too loose (walk-past persists):** tighten `segment_entry_stop_speed_m_s` → 0.02 (one param, no redeploy of logic).
- **Rollback:** `segment_entry_true_stop_dist_m=0` (revert to capture) or `segment_entry_stop_speed_m_s=0.08` (revert cert), else `git revert` the one commit. No param archaeology.

---

## G. Deliverable summary

**1. Design decision:** Replace pure-zero entry stop with segment true-stop parity (active brake + corner-hold, already in the generic tail) and certify on a parked gate via existing `_corner_stop_satisfied` + one new threshold `segment_entry_stop_speed_m_s=0.03`. Delete the pure-stop branch/latch/helper/3 params. One mode, one new knob (default = production).

**2. Entry-stop pseudocode:**
```
_hold_before_run_advance(pos, yaw, dist):
  if last run: return False
  if not next_run_requires_alignment(): advance_run(); return True
  latch stop_pending; compute stop_pt, prev_pt, heading_err, pos_error, position_ok
  stop_reason = RUNTIME_ENTRY_TO_MARK if entry_boundary else RUN_BOUNDARY

  # CERT — parked gate for entry, segment gate otherwise
  speed_override = segment_entry_stop_speed_m_s if entry else None
  if corner_stop_satisfied(position_ok, speed_threshold_override=speed_override):
      make+validate stop_certificate; publish STOP_CERTIFIED
      run_boundary_stop_pending=False; advance_run(pre_stopped=True); return True

  # HOLD — active brake then servo (identical for entry + smooth boundary)
  segment_state=CORNER_STOP
  meas = speed(vel_ned) if vel_fresh else 0
  if profile!="segment" and true_stop_dist>0 and pos_error<=true_stop_dist:
      brake = corner_brake_velocity(yaw) if meas>stop_speed
              else corner_hold_velocity(..., cap=stop_speed)
  elif profile!="segment" and pos_error>handoff: brake = smooth_capture_velocity(...)
  else: brake = corner_hold_velocity(...)
  publish_velocity(brake); publish_yaw_rate(0); publish HOLDING; return True

corner_stop_satisfied(position_ok, speed_threshold_override=None):
  speed_thresh = speed_threshold_override or param(segment_stop_speed_threshold)
  # position_ok AND speed<thresh AND yaw_rate<thresh, continuous >= dwell(0.30s)
  # stale velocity -> cap at _CORNER_STOP_MAX_HOLD_S(2.0s)
```

**3. File/function checklist:** `src/rpp_controller_node.py` — `_hold_before_run_advance:1748` (edit), `_corner_stop_satisfied:5003` (signature), `_entry_pure_stop_hold:1948` (delete), param block `:590` (swap), state `:695`/reset `:4953` (delete).

**4. Tests:** `test_entry_true_stop.py` (new, replaces `test_entry_pure_stop.py`):
- `test_entry_stop_active_brakes_not_zero` — inside true_stop window, meas>stop_speed ⇒ published vel nonzero, longitudinal, opposes motion (not (0,0)).
- `test_entry_stop_certifies_only_when_parked` — meas=0.05 ⇒ no advance; meas≤0.03 + pos≤0.02 held 0.30 s ⇒ `_advance_run(pre_stopped=True)` fires once.
- `test_entry_stop_blocks_above_position_tol` — pos_error=0.04, slow ⇒ no cert.
- `test_entry_stop_stale_velocity_fallback` — stale vel + held≥2 s ⇒ cert (no deadlock).
- `test_entry_pure_stop_removed` — `_entry_pure_stop_hold`/latch absent (regression guard).
- Update `test_run_boundary_pivot.py` any entry pure-zero assertion.

**5. Field checklist:** section E — 4–5 runs, stop-first pass gate (≤0.03 m/s, ≤0.02 m pos, ≤0.05 m overshoot, ≤2.5 s to cert), pivot observation-only.

**6. OUT OF SCOPE (do not touch):** t=0 init turn / forward-cone pure-pursuit; `_run_alignment_hold` pivot + `segment_entry_pivot_recenter` geometry; post-cert profile switch / `_advance_run` ordering; spray; all PX4/QGC/FCU params; ArduRover; arc/smooth PID + lookahead tuning; segment intra-corner stop (reference, unchanged).
