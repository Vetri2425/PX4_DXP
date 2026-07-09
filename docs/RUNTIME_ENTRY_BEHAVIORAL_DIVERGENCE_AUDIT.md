# Runtime-Entry Failure Root Cause Audit (Behavioral Equivalence)

**HEAD:** `5f35c7a` (`feat/entry-pivot-recenter`)
**Methodology:** Cycle-by-cycle behavioral diff vs. validated reference transition. No architecture comparison, no redesign.

---

## Executive Summary

**First behavioral divergence:** Phase 3 (Stop behavior), in `_entry_pure_stop_hold` — the very first controller cycle after `pos_error <= segment_entry_pure_stop_dist_m` (0.10 m) at a Runtime-Entry boundary.

At that cycle the two paths emit **different velocity commands**:

| Transition | First stop-zone velocity command |
|---|---|
| **RUN_BOUNDARY** (working) | `_corner_hold_velocity` / `_corner_brake_velocity` / `_smooth_capture_velocity` — a **nonzero** servo command toward the stop point |
| **RUNTIME_ENTRY_TO_MARK** (failing) | `_publish_zero` — **exactly (0, 0)**, no servo, no brake, no capture |

This single branch split is the root cause. Everything downstream — the certification deadlock, the dwell-timer reset, the absence of a stale-velocity fallback — is a *consequence* of this divergence, not an independent bug.

**Why the working transition succeeds:** it actively brakes the rover to a halt (`_corner_brake_velocity` opposes measured motion), then certifies on a **loose** speed gate (`segment_stop_speed_threshold` = 0.08 m/s) with a **stale-velocity fallback** (`_CORNER_STOP_MAX_HOLD_S` = 2.0 s).

**Why Runtime-Entry fails:** PX4 in OFFBOARD velocity mode **coasts on a zero setpoint** — it does not brake. With the pure-zero command and **no active brake**, the rover must coast down to `segment_entry_pure_stop_speed_m_s` (0.005 m/s — 16× tighter than the working path) purely through mechanical drag. EKF/RTK noise keeps measured speed fluctuating at 0.005–0.02 m/s, so the 1.0 s continuous dwell (`segment_entry_pure_stop_dwell_s`) repeatedly resets and the certification **never completes**. The pivot never fires.

---

## 1. Behavioral Timeline — Working Transition (RUN_BOUNDARY)

Reference: `Transit → MARK` or `Extension → Connector`. Active run is **smooth** (Runtime-Entry legs are also smooth, so this is the apples-to-apples reference).

| Cycle (rel.) | State | Function | vel (vn, ve) | speed | Stop-cert | Align-cert | Notes |
|---|---|---|---|---|---|---|---|
| t−2.0 m | TRACKING | `_control` (smooth RPP) | along-path, ~mission_speed | ~0.35 | — | — | Pure-pursuit lookahead |
| … approach scaling begins ~1.5 m out … | | | | decel | | | `approach_velocity_scaling_dist` |
| t−0.50 m | TRACKING→capture | `_hold_before_run_advance` (capture latch) | `_smooth_capture_velocity` toward stop_pt | 0.08→0.03 | — | — | `segment_boundary_capture_radius_m` latches `_run_boundary_stop_pending` |
| t−0.10 m | CORNER_STOP | `_hold_before_run_advance` → corner_hold/brake | `_corner_hold_velocity` (tangent servo) or `_corner_brake_velocity` (body-axis brake) | ≤0.18 cap | — | — | **Nonzero command toward stop point** |
| t≈stop | CORNER_STOP | `_corner_stop_satisfied` | nonzero brake/hold until speed<0.08 & yawrate<0.05 | <0.08 | arming | — | Active braking drives speed down |
| +0.30 s dwell | HOLDING→STOP_CERTIFIED | `_make_stop_certificate` | 0 | 0 | **created** | — | `segment_stop_dwell_s` passed |
| next cycle | `_advance_run(pre_stopped=True)` | — | — | — | cert carried | — | Run swaps; `_corner_stop_complete=True` carried |
| pivot cycles | CORNER_ALIGN | `_run_alignment_hold` → `_corner_pivot_velocity` | corner_speed @ exit heading | ~0.08 | — | arming | ±75° forward cone |
| recenter cycles | CORNER_ALIGN | `_run_alignment_hold` → `_corner_hold_velocity` (recenter) | capped servo toward run0 | ≤0.08 | — | arming | `segment_entry_pivot_recenter` |
| +0.20 s settle | ALIGN_CERTIFIED | `_make_alignment_certificate` | 0 | 0 | — | **created** | `segment_align_settle_s` |
| release | TRACK_SEGMENT | `_reset_corner_pivot_state` → `_control_segment_profile` | along new run | ramp | — | — | Tracking resumes |

---

## 2. Behavioral Timeline — Runtime-Entry (RUNTIME_ENTRY_TO_MARK)

Active run is the entry-OFF leg (smooth), next run is the first MARK run.

| Cycle (rel.) | State | Function | vel (vn, ve) | speed | Stop-cert | Align-cert | Notes |
|---|---|---|---|---|---|---|---|
| t−2.0 m | TRACKING | `_control` (smooth RPP) | along-path | ~0.35 | — | — | Identical to working |
| t−0.50 m | TRACKING→capture | `_hold_before_run_advance` (capture latch) | `_smooth_capture_velocity` | 0.08→0.03 | — | — | **Identical** to working |
| t−0.10 m | **CORNER_STOP** | `_hold_before_run_advance` → **`_entry_pure_stop_hold`** | **(0, 0)** via `_publish_zero` | **0.0** | — | — | **🔴 DIVERGENCE: latch sets, pure-zero branch taken** |
| +0.02 s | CORNER_STOP | `_entry_pure_stop_hold` | (0, 0) | 0.0 cmd / ~0.02 meas | — | — | `speed_ok = meas <= 0.005` → False |
| +0.5 s | CORNER_STOP | `_entry_pure_stop_hold` | (0, 0) | coasting down | — | — | Rover coasts; no active brake |
| +1.0 s | CORNER_STOP | `_entry_pure_stop_hold` | (0, 0) | ~0.008 (noise floor) | — | — | `speed_ok` flickers; dwell resets |
| +5.0 s | CORNER_STOP | `_entry_pure_stop_hold` | (0, 0) | ~0.008 | — | — | **Deadlock**: dwell never completes 1.0 s continuously |
| … (pivot never fires) … | | | | | | | |

---

## 3. Side-by-Side Comparison Per Phase

### Phase 1 — Approach
**Identical.** Both use the smooth-RPP `_control` path with the same lookahead, speed profile (`approach_velocity_scaling_dist`=1.5 m), and slowdown. Capture latches at `segment_boundary_capture_radius_m`=0.50 m via the same code in `_control_loop`. ✅ No divergence.

### Phase 2 — Stop Initiation
**Identical trigger.** Both enter `_hold_before_run_advance` when `dist_to_goal <= capture_r` and `_next_run_requires_alignment()`. Both compute `stop_reason` and `pos_error` the same way. ✅ No divergence yet.

### Phase 3 — Stop Behavior (🔴 FIRST DIVERGENCE)
**This is where they split.** In `_hold_before_run_advance` (line ~1813):

```python
if stop_reason == StopReason.RUNTIME_ENTRY_TO_MARK and self._entry_pure_stop_latched:
    return self._entry_pure_stop_hold(...)   # ← Runtime-Entry ONLY
# else: generic corner_brake / corner_hold / smooth_capture branch  ← working path
```

| | RUN_BOUNDARY (working) | RUNTIME_ENTRY_TO_MARK |
|---|---|---|
| Branch | generic `_hold_before_run_advance` tail | `_entry_pure_stop_hold` |
| Velocity command | `_corner_hold_velocity` / `_corner_brake_velocity` / `_smooth_capture_velocity` | **`_publish_zero` (exactly 0,0)** |
| Active braking? | **Yes** (`_corner_brake_velocity` opposes motion when `meas_speed > stop_speed`) | **No** (zero command only) |
| Servo toward point? | **Yes** (tangent-frame hold or fixed-bearing capture) | **No** |

The published velocity streams are **NOT identical**. This is the earliest cycle where the two paths produce different `(vn, ve)`.

### Phase 4 — Stop Certification (🔴 consequence of Phase 3)
| Gate | RUN_BOUNDARY (`_corner_stop_satisfied`) | RUNTIME_ENTRY (`_entry_pure_stop_hold`) |
|---|---|---|
| Position gate | `corner_position_tolerance_m` = 0.02 m | `corner_position_tolerance_m` = 0.02 m (same) |
| Speed gate | `segment_stop_speed_threshold` = **0.08 m/s** | `segment_entry_pure_stop_speed_m_s` = **0.005 m/s** (16× tighter) |
| Yaw-rate gate | `segment_stop_yaw_rate_threshold` = 0.05 rad/s | **None** (speed only) |
| Dwell | `segment_stop_dwell_s` = **0.30 s** | `segment_entry_pure_stop_dwell_s` = **1.0 s** (3.3× longer) |
| Stale-velocity fallback | **Yes** — `_CORNER_STOP_MAX_HOLD_S` = 2.0 s cap | **No** — `measured_speed = None` → `speed_ok = False` → dwell resets forever |
| Active braking to reach gate | **Yes** | **No** |

The working path's certification is reachable *because the controller actively brakes the rover to <0.08 m/s*. The Runtime-Entry path's certification is unreachable because (a) the rover is never actively braked and (b) the 0.005 m/s threshold sits inside the EKF noise floor.

### Phase 5 — Run Transition
Code path is identical (`_advance_run(pre_stopped=True)`), but **Runtime-Entry never reaches it** because Phase 4 deadlocks. For the working path, the run swap, segment reset, path-index reset, and certificate carry (`pre_stopped=True` → `_corner_stop_complete=True`) all proceed correctly. ✅ No independent bug here.

### Phase 6 — Pivot
Code path is identical (`_run_alignment_hold` → `_corner_pivot_velocity`, same ±75° cone, same `segment_min_corner_speed`, same damping). Inputs to `_corner_pivot_velocity` (`yaw_ned`, `heading_err`, `corner_speed`) are computed the same way. **Runtime-Entry never reaches it.** ✅ No independent bug.

### Phase 7 — Recenter
Identical (`segment_entry_pivot_recenter` gates the same `_corner_hold_velocity` recenter for both `RUN_BOUNDARY` and `RUNTIME_ENTRY_TO_MARK`). **Runtime-Entry never reaches it.** ✅ No independent bug.

### Phase 8 — Release
Identical release gates (`heading_ok`, `yaw_rate_ok`, `speed_ok`, `position_release_ok`, `align_settle_s`). **Runtime-Entry never reaches it.** ✅ No independent bug.

### Phase 9 — Tracking Restart
Identical (`_reset_corner_pivot_state` → normal `_control_segment_profile` / `_control`). **Runtime-Entry never reaches it.** ✅ No independent bug.

---

## 4. First Behavioral Divergence

**Phase:** 3 (Stop behavior)
**Cycle:** The first control cycle in `_hold_before_run_advance` after `_entry_pure_stop_latched` is set (i.e., the first cycle where `pos_error <= segment_entry_pure_stop_dist_m` = 0.10 m at a Runtime-Entry boundary).
**What changes:** Runtime-Entry calls `_entry_pure_stop_hold` → `_publish_zero` (0, 0). The working path calls `_corner_hold_velocity` / `_corner_brake_velocity` / `_smooth_capture_velocity` (nonzero servo/brake).

---

## 5. Exact Source File

`src/rpp_controller_node.py`

---

## 6. Exact Function

`_hold_before_run_advance` (dispatch) and `_entry_pure_stop_hold` (the divergent branch).

---

## 7. Exact Lines

- **Dispatch (the split):** `_hold_before_run_advance`, the block
  ```python
  if stop_reason == StopReason.RUNTIME_ENTRY_TO_MARK and self._entry_pure_stop_latched:
      return self._entry_pure_stop_hold(...)
  ```
  (~lines 1822–1825)
- **Divergent command:** `_entry_pure_stop_hold`, the unconditional
  ```python
  self._publish_zero(StateCode.APPROACH, ...)
  ```
  (~line 1977, executed *before* the dwell/certify check)
- **Unreachable cert gate:** same function,
  ```python
  speed_limit = float(self.get_parameter("segment_entry_pure_stop_speed_m_s").value)  # 0.005
  ...
  speed_ok = measured_speed is not None and measured_speed <= speed_limit
  ```
  (~lines 1990–1998)
- **Deadlock on stale velocity:** same function,
  ```python
  else:
      self._entry_pure_stop_since = None   # resets dwell; no fallback cap
  ```
  (~line 2024)

---

## 8. Exact Variables Responsible

| Variable | Where | Role in failure |
|---|---|---|
| `stop_reason == StopReason.RUNTIME_ENTRY_TO_MARK` | `_hold_before_run_advance` dispatch | Routes into the pure-zero branch instead of the generic brake/hold/capture branch |
| `self._entry_pure_stop_latched` | `_hold_before_run_advance` | Once set, *only* `_entry_pure_stop_hold` may run — permanently excludes the brake/hold/capture branches |
| `segment_entry_pure_stop_speed_m_s` (=0.005) | `_entry_pure_stop_hold` cert gate | 16× tighter than the working path's 0.08; sits inside EKF noise floor |
| `segment_entry_pure_stop_dwell_s` (=1.0) | `_entry_pure_stop_hold` dwell | 3.3× longer than working path's 0.30 s; combined with the tight speed gate, never completes |
| `measured_speed` (None when velocity stale) | `_entry_pure_stop_hold` | Forces `speed_ok=False` and dwell reset with **no** `_CORNER_STOP_MAX_HOLD_S` fallback |
| `_publish_zero(...)` (vs `_corner_brake_velocity`) | `_entry_pure_stop_hold` | No active brake → PX4 coasts → measured speed never drops reliably below 0.005 |

---

## 9. Why the Working Transition Succeeds

The generic `_hold_before_run_advance` tail (RUN_BOUNDARY / smooth run boundary):
1. **Actively brakes** the rover via `_corner_brake_velocity` (a longitudinal command opposing measured body-forward motion, capped at 0.18 m/s) whenever `meas_speed > segment_stop_speed_threshold`.
2. **Servos toward the point** via `_corner_hold_velocity` (tangent-frame PD) or `_smooth_capture_velocity` (fixed-bearing decel) so the rover lands inside `corner_position_tolerance_m`.
3. Certifies on `_corner_stop_satisfied`, which gates on **0.08 m/s** (reachable because the controller itself drives the speed down) + yaw-rate 0.05 rad/s, dwell **0.30 s**.
4. Provides a **stale-velocity fallback** (`_CORNER_STOP_MAX_HOLD_S` = 2.0 s) so a quiet velocity topic cannot deadlock the mission.

The combination (active braking + loose gate + fallback) is what makes the working path convergent.

---

## 10. Why Runtime-Entry Fails

`_entry_pure_stop_hold` (RUNTIME_ENTRY_TO_MARK only):
1. **Commands exactly zero** — `_publish_zero` runs unconditionally at the top of the function, before any certify check. PX4 in OFFBOARD velocity mode **coasts on a zero setpoint** (the codebase documents this repeatedly, e.g. the `segment_brake_velocity_cap_m_s` param comment: "PX4 velocity-OFFBOARD does not brake on a zero setpoint — it coasts").
2. **No active brake, no servo, no capture** is ever permitted once `_entry_pure_stop_latched` is set — the latch is explicit and only clears on STOP_CERTIFIED + `_advance_run` (which never happens) or mission reset.
3. The rover must therefore coast down to `segment_entry_pure_stop_speed_m_s` = **0.005 m/s** purely through mechanical drag.
4. EKF/RTK noise floor on a 3WD rover keeps measured speed fluctuating at roughly 0.005–0.02 m/s even when physically stopped — the param's own doc admits "measured speed will essentially never read exactly 0.000".
5. The 1.0 s **continuous** dwell (`segment_entry_pure_stop_dwell_s`) resets on **any** cycle where `meas_speed > 0.005` or velocity is stale. With noise-driven flicker, the dwell timer never completes a full 1.0 s window.
6. Unlike `_corner_stop_satisfied`, there is **no stale-velocity fallback cap** — `measured_speed = None` ⇒ `speed_ok = False` ⇒ `_entry_pure_stop_since = None` indefinitely.

Result: the StopCertificate is never created → `_advance_run(pre_stopped=True)` is never called → the run never swaps → the pivot in `_run_alignment_hold` never runs → the first MARK run is never tracked. The rover sits at the boundary emitting zero velocity forever.

---

## 11. Failure Stage Classification

**Primary stage: STOP (Phase 3) + CERTIFICATE (Phase 4) interaction.**

- Phase 3 is the *first* divergence (the pure-zero command vs. active brake/servo).
- Phase 4 is where the divergence becomes *fatal* (the 0.005 m/s gate + 1.0 s dwell + no fallback makes the certification non-convergent).

Phases 5–9 (run swap, pivot, recenter, release, tracking restart) are **never reached** — they are not independent failure sites. The failure is *not* in the approach, the pivot, the recenter, the release, or the tracking restart; those are all byte-for-byte identical to the working path.

---

## 12. Root Cause

**Single root cause:** `RUNTIME_ENTRY_TO_MARK` is routed into a dedicated `_entry_pure_stop_hold` branch that (a) forbids the active braking / position servo the working path uses to converge, and (b) certifies on a speed threshold (`segment_entry_pure_stop_speed_m_s` = 0.005 m/s) that sits inside the EKF/RTK noise floor, with a dwell (1.0 s) and no stale-velocity fallback that make it structurally non-convergent.

If multiple independent bugs must be ranked, they are all sub-defects of this one branch:

1. **(Highest causal impact)** The pure-zero command with no active brake. PX4 coasts; the rover is never driven to the speed gate. (`_publish_zero` at the top of `_entry_pure_stop_hold`.)
2. **(High)** The 0.005 m/s certification threshold is below the EKF noise floor. Even if the rover were physically stopped, measured speed flicker prevents a continuous 1.0 s dwell. (`segment_entry_pure_stop_speed_m_s` gate.)
3. **(Medium)** No stale-velocity fallback. `_corner_stop_satisfied` caps at `_CORNER_STOP_MAX_HOLD_S` = 2.0 s when velocity is stale; `_entry_pure_stop_hold` resets the dwell forever. (`measured_speed = None` → `speed_ok = False`.)
4. **(Lowest, derivative)** The 1.0 s dwell (vs 0.30 s) compounds bugs #2 and #3 but is not independently fatal.

**The working `RUN_BOUNDARY` transition succeeds because it uses the generic tail of `_hold_before_run_advance`, which actively brakes the rover and certifies on a 0.08 m/s gate with a stale-data fallback. Runtime-Entry fails because it is diverted into `_entry_pure_stop_hold`, which does neither.**

No fix is proposed. The objective — identifying the precise behavioral divergence — is complete.