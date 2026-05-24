# 05 — Stanley xtrack blend in small-error regime

**Agent:** GLM (4.5 or 5.1)
**Estimated diff:** +60 lines
**Depends on:** 03 (needs path tangent for heading reference)
**Blocks:** —

## Goal

Pure Pursuit has a known weakness: with small xtrack, its steering target
oscillates because α (heading-to-lookahead) is dominated by noise. The
Stanley controller, designed for the DARPA Grand Challenge, drives xtrack
to zero asymptotically with no oscillation but is poor for re-acquisition
from far off-path.

Blend them: Stanley in the small-error regime, Pure Pursuit in the large-
error regime. Result: best of both.

## Math

Stanley steering law:

```
δ_stanley = ψ_e + atan2(k_st · e_⊥, v + ε)
```

where:
- `ψ_e = ψ_path(s_proj) - ψ_vehicle` (signed heading error wrt path tangent
  at the foot of perpendicular).
- `e_⊥` is signed cross-track.
- `k_st` is the Stanley gain (start at 1.0).
- `ε` is a small softening constant (0.05 m/s) so the formula is well-
  defined at v ≈ 0.
- The arctan caps the cross-track contribution, preventing over-steer when
  far off-path.

Pure Pursuit steering law (existing):

```
δ_pp = atan2(2·L·sin(α), L_d)     (for a bicycle; for our diff-drive we use
                                     ω = κ·v which has the same effect)
```

Blend:

```
w = exp(-(e_⊥ / e_blend)²)        # gaussian, w → 1 at e=0, w → 0 at e=2·e_blend
δ_cmd = w · δ_stanley + (1 - w) · δ_pp
```

`e_blend` ≈ 0.08 m so the crossover is around 5 cm xtrack.

## Files to read first

- `src/rpp_controller_node.py` — control loop, the block that converts α
  and L_d to a yaw target.
- `path_geometry.py::SplinePath::tangent(s)`.

## Scope

### A. Parameters

- `rpp_enable_stanley_blend` (bool, default False).
- `rpp_stanley_gain` (float, default 1.0).
- `rpp_stanley_v_soft_m_s` (float, default 0.05).
- `rpp_blend_xtrack_m` (float, default 0.08).

### B. Wiring

After projection: compute `ψ_path_at_foot = self._spline.tangent(s_proj)`
heading.

Compute `δ_stanley` per the formula. Compute the existing Pure Pursuit
yaw target as `δ_pp`. Blend using the Gaussian weight.

Replace the existing yaw target output with the blended `δ_cmd`. If
`rpp_enable_stanley_blend=False`, output the existing `δ_pp` exactly
(regression safety).

### C. Frame discipline

All angles in NED radians, range `[-π, π]`. Wrap explicitly after each
operation. The `ψ_e` and `e_⊥` signs must agree (both right-of-path =
positive); add a unit test for that.

### D. Tests

`tests/test_stanley_blend.py`:

- Straight path, vehicle at xtrack = 0, ψ_e = 0 → δ_cmd = 0.
- Straight path, vehicle at xtrack = +10 cm, ψ_e = 0, v = 0.4 → δ_stanley ≈
  atan2(1.0 · 0.1, 0.45) ≈ 12.5°, pointing back toward path (negative δ in
  NED-CW convention).
- Curve, vehicle on-path, ψ_e = 0 → δ_stanley = 0 (Stanley does NOT add
  curvature on-path; that's what FF in task 04 does).
- Blend weight: at e=0 → w=1 (pure Stanley); at e=0.16 m → w≈0.018 (almost
  pure PP).

## Out of scope

- Tuning `k_st` adaptively (manual).
- Stanley with predictive horizon — that's MPC, not Stanley.

## Acceptance criteria

- [ ] On straight 5 m path, with blend on, the steady-state xtrack
      standard deviation halves vs blend-off.
- [ ] On 2 m square corners, with blend on, no worse than blend-off
      (Stanley shouldn't help here because xtrack > e_blend).
- [ ] Stanley alone (set `e_blend → ∞`) does NOT cause divergence at large
      xtrack on the half-circle re-entry test.
- [ ] All unit tests pass.

## Notes for the agent

- Reference: Hoffmann, Tomlin, Montemerlo, Thrun. *Autonomous Automobile
  Trajectory Tracking for Off-Road Driving: Controller Design, Experimental
  Validation and Racing.* ACC 2007 — the original Stanley paper.
- Stanley was designed for an Ackermann car; for differential-drive we use
  the same steering angle as a desired heading and let the lower-level
  control handle wheel speeds. The math is identical.
