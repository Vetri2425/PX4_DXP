# 04 — Path curvature feedforward

**Agent:** Haiku 4.5
**Estimated diff:** +40 lines
**Depends on:** 03 (needs `PathGeometry.kappa(s)`)
**Blocks:** —

## Goal

Currently the controller derives steering curvature from the geometric
Pure-Pursuit formula `κ_pp = 2·sin(α) / L_d`, which lags the real path
curvature by ~`L_d / v` seconds. Add a feedforward term using the known
path curvature at the lookahead point.

**RPP remains the primary steering law.** This adds a feedforward correction
to the existing RPP output — it does NOT replace RPP with a different
controller.

## Math

Steering curvature command:

```
κ_cmd = κ_pp + k_ff · κ_path(s_la)
```

where:
- `κ_pp` is the existing geometric Pure-Pursuit curvature (unchanged).
- `κ_path(s_la)` is the path curvature at the lookahead arc-length (from
  PathGeometry, task 03).
- `k_ff ∈ [0, 1]` is a blend. Start at 0.5, tune to 1.0 if stable.

For body-rate output (P3.1 yaw-rate):

```
ω_ff = κ_path(s_la) · v_cmd      (already in P3.1 — verify it now uses
                                    PathGeometry κ, not polyline-derived)
```

## Files to read first

- `src/rpp_controller_node.py` — control loop (lines 977-1300). Find where
  `kappa = ...` is set for the velocity output.
- `src/path_geometry.py` (from task 03).

## Scope

### A. Parameter

`rpp_kappa_feedforward_gain` (float, default 0.0 → off). Range [0, 1.5].

### B. Wiring

In the control loop, after the lookahead point is chosen, fetch
`κ_path = self._path_geom.kappa(s_la)`. Add `k_ff · κ_path` to the curvature
used downstream.

For body-rate path (P3.1): replace any polyline-derived κ in the yaw-rate
calc with `self._path_geom.kappa(s_la)`.

### C. Sign convention

The existing code uses signed κ where right-of-path is +. Verify PathGeometry
returns the same sign convention by running the unit test from task 03's
half-circle on a known direction. Fix the sign in `PathGeometry.kappa()` if
the test reveals a flip — do NOT add ad-hoc sign flips in the controller.

### D. Diagnostics

Add `kappa_path_la` to `/rpp/debug` array (extend it to 10 elements if
needed). This makes the FF visible offline.

## Out of scope

- Auto-tuning `k_ff` (manual gain).
- Per-segment FF gain.
- Removing the geometric Pure-Pursuit term (we keep both as belt + braces).

## Acceptance criteria

- [ ] On a constant-radius arc, with `k_ff = 1.0`, the steady-state heading
      error drops at least 50 % vs `k_ff = 0`.
- [ ] On straights (κ_path = 0), behaviour is unchanged regardless of `k_ff`.
- [ ] No oscillation introduced at `k_ff = 1.0`.
- [ ] Unit test: `κ_path` from PathGeometry at midpoint of unit circle equals
      1.0 ± 0.05, with correct sign.

## Notes for the agent

- This is a small, focused change. Resist the urge to also tune Pure-Pursuit
  gains in the same PR.
- The signed convention slot in the existing debug array is element [4]
  (κ at lookahead vehicle-relative). Add a separate element for path-frame
  κ rather than overwriting it.