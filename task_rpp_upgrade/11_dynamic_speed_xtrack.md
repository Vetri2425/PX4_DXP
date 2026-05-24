# 11 — Dynamic speed regulation by |xtrack|

**Agent:** Gemma 3 1B
**Estimated diff:** +20 lines
**Depends on:** 06 (multiplies on top of the precomputed profile)
**Blocks:** —

## Goal

When the rover is far off-path (e.g. after a disturbance, after a manual
re-position, after a path swap), the high inherited `v_ref` from the speed
profile causes a hot re-acquisition with overshoot. Scale `v_ref` down
when `|e_⊥|` is large.

## Math

Smooth speed multiplier in `[v_min_frac, 1]`:

```
g(|e|) = clamp(1 - |e| / e_max, v_min_frac, 1)
v_out = v_ref · g(|e|)
```

- `e_max` = 0.30 m → at 30 cm off-path, multiplier = `v_min_frac`.
- `v_min_frac` = 0.25 → minimum speed = 25 % of nominal (don't stop, that
  delays re-acquisition).
- Linear is fine; the speed profile already handles the
  curvature-anticipatory shape.

## Files to read first

- `src/rpp_controller_node.py` — after task 06's `v_cmd =
  self._speed_profile[...]`.

## Scope

### A. Parameters

- `rpp_enable_dynamic_speed` (bool, default False).
- `rpp_dynamic_speed_e_max_m` (float, default 0.30).
- `rpp_dynamic_speed_v_min_frac` (float, default 0.25).

### B. Wiring

After `v_cmd` is computed from the speed profile, multiply by `g(|e|)`.
Pass through unchanged when the feature is off.

### C. Diagnostics

Log the multiplier to `/rpp/debug` so the offline trace shows when it kicks
in.

### D. Tests

`tests/test_dynamic_speed.py`:

- |e| = 0 → g = 1 → v unchanged.
- |e| = 0.30 → g = v_min_frac = 0.25 → v = 0.25·v_ref.
- |e| > 0.30 → g still v_min_frac (clamp).

## Out of scope

- Adaptive `e_max` based on path curvature.
- Stopping when |e| > some threshold (too aggressive; keep going slowly to
  re-acquire).

## Acceptance criteria

- [ ] Drop the rover at 25 cm xtrack on a straight path → re-acquires in
      < 1.5 s with no overshoot beyond ±2 cm.
- [ ] No regression on on-path tracking (g = 1 when |e| ≈ 0).
- [ ] Multiplier visible in `/rpp/debug` during a manual disturbance test.

## Notes for the agent

- This complements task 07 (I-term) and task 12 (bias observer): both
  expect small steady xtrack to converge over time. If `g` is too
  aggressive at small |e|, the I-term will integrate slower because the
  rover moves less per cycle. Keep `e_max ≥ 0.20 m` to avoid that
  coupling.
