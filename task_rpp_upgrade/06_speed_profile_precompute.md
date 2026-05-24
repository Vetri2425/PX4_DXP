# 06 — Precomputed speed profile (lateral-acceleration-bounded)

**Agent:** GLM (4.5 or 5.1)
**Estimated diff:** +200 lines (1 new module, controller wiring)
**Depends on:** 03 (needs `SplinePath.kappa(s)`)
**Blocks:** 11 (dynamic speed multiplies on top of this)

## Goal

Today the predictive κ regulation (P1.1) reactively scales speed at the
current lookahead. Replace it with a *precomputed* speed profile `v_ref(s)`
that anticipates every corner along the entire path, satisfying:

- Lateral acceleration cap: `v² · |κ(s)| ≤ a_lat_max`
- Longitudinal accel cap: `|dv/ds · v| ≤ a_long_max` (both accel + brake)
- Max speed: `v(s) ≤ v_max`

The profile is solved once at path-receipt time, in ~10 ms for typical
paths. The controller then just looks up `v_ref(s_la)` at each cycle.

This eliminates the "I started braking too late" failure mode that drives
corner overshoot.

## Algorithm — forward / backward pass

Standard time-optimal-on-fixed-path technique:

1. Compute the curvature-limited speed at every sample:
   `v_kappa(s) = sqrt(a_lat_max / max(|κ(s)|, ε))`, clamped to `v_max`.
2. Forward pass: enforce acceleration limit going forward from `s=0`.
   `v[i+1] = min(v_kappa[i+1], sqrt(v[i]² + 2·a_long_max·Δs))`
3. Backward pass: enforce deceleration limit going backward from `s=end`.
   `v[i-1] = min(v[i-1], sqrt(v[i]² + 2·a_long_max·Δs))`
4. Result is `v_ref(s)` = the lower envelope of all three constraints.

Optionally smooth jerk by passing v_ref through a small low-pass before
return.

## Files to read first

- `src/rpp_controller_node.py` — `_max_preview_curvature` (P1.1) and the
  speed-output block.
- `path_geometry.py::SplinePath` (task 03).

## Scope

### A. New module `src/speed_profile.py`

A pure-math module. Function:

```
solve_speed_profile(
    kappa_array,       # 1D array, sampled at Δs spacing
    ds,                # sample spacing (m)
    v_max,             # m/s
    a_lat_max,         # m/s²
    a_long_max,        # m/s²
    v_start,           # m/s — initial condition (usually 0)
    v_end,             # m/s — terminal condition (usually 0)
    jerk_filter_alpha, # 0..1, 0 = off
) -> v_array
```

Returns a 1D array same shape as `kappa_array`.

### B. Wiring in controller

In `_path_cb`:

```
self._speed_profile = solve_speed_profile(
    self._spline.kappa_samples,
    ds=0.01,
    v_max=cruise_speed,
    a_lat_max=0.25 * 9.81,  # = ATC_TURN_MAX_G in QGC
    a_long_max=0.5,
    v_start=0.0,
    v_end=0.0,
    jerk_filter_alpha=0.1,
)
```

In control loop, replace the existing predictive-κ speed scaling with:

```
v_cmd = self._speed_profile[clamp(round(s_la / 0.01), 0, len-1)]
```

### C. Parameters

- `rpp_enable_speed_profile` (bool, default False).
- `rpp_a_lat_max_m_s2` (float, default 2.45 ≈ 0.25 g).
- `rpp_a_long_max_m_s2` (float, default 0.5).
- `rpp_speed_profile_jerk_alpha` (float, default 0.1).

When disabled, fall through to P1.1 predictive-κ logic. When enabled, P1.1
becomes a safety belt only (a runtime check that `v_cmd ≤ sqrt(a_lat / κ_actual)`
with a 1.2× margin — if it isn't, log a warning and clamp).

### D. Tests

`tests/test_speed_profile.py`:

- Straight line of all-zero κ → v_ref = v_max everywhere except first and
  last segments (accel / decel).
- Step-curvature track (straight, then sharp turn, then straight) → v_ref
  shows: cruise, anticipatory decel to corner-limited speed, hold through
  corner, re-accel.
- v_start = 0, v_end = 0 verified.
- Bounded accel: max(|dv|) per Δs sample ≤ a_long·Δs/v + small numerical
  slack.

### E. Diagnostics

Add `v_ref` and `v_cmd` to `/rpp/debug` (extend array if needed). This
makes "rover went too fast into a corner" diagnosable from a bag.

## Out of scope

- Time-optimal solutions with vehicle dynamics (mass, motor torque limits).
  We're at low speed; kinematic constraints are sufficient.
- Energy-optimal profiles (battery life).
- Online re-planning. One-shot at path receipt.

## Acceptance criteria

- [ ] All unit tests pass.
- [ ] On 2 m square baseline run with profile on: rover slows to <0.25 m/s
      before each corner and re-accelerates after — visible in bag.
- [ ] Max xtrack at corners drops to < 5 cm (combined with task 03 spline,
      should reach the < 3 cm target).
- [ ] Profile solve time < 50 ms for a 100 m path on Jetson.
- [ ] No latency increase on the 50 Hz control loop (the lookup is O(1)).

## Notes for the agent

- Reference for forward/backward pass: Pfeiffer & Johanni (1987),
  "Concept for manipulator trajectory planning." Same idea, fixed path.
- The jerk filter is a single-pole IIR; don't go higher-order — it
  introduces phase lag that shows up as late braking.
- a_lat_max should match the PX4 `ATC_TURN_MAX_G` parameter so the
  feedforward and the PX4 inner-loop agree about what's safe. Mismatch
  causes the rover to enter a corner thinking "I can do this", then PX4
  brakes harder than the profile expected.
