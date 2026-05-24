# 07 — Cross-track integral term (kill steady bias)

**Agent:** Haiku 4.5
**Estimated diff:** +25 lines
**Depends on:** 03 (for clean projection s parameter)
**Blocks:** —

## Goal

Pure Pursuit and Stanley both lack an integral term against `e_⊥`. A small
constant disturbance (compass bias, wheel diameter mismatch, mounting
offset) produces a constant non-zero xtrack that neither controller will
remove. Add a slow I-term that integrates xtrack over arc length and
biases the yaw setpoint.

## Math

```
i_e ← i_e + e_⊥ · ds                  # integrate over arc length, not time
i_e ← clamp(i_e, -i_max, +i_max)      # anti-windup
δ_bias = k_i · i_e
ψ_cmd ← ψ_cmd - δ_bias                # subtract because positive e_⊥ = right of path
```

Integrate vs arc length (not time) so the term is speed-invariant. A 1 cm
xtrack contributes the same regardless of how fast you're crawling.

Reset `i_e ← 0` when:
- Path is reloaded.
- Rover transitions IDLE → TRACKING.
- `|e_⊥|` exceeds an "off-path" threshold (default 0.20 m): the controller
  is in re-acquisition, not steady tracking — windup would hurt.

## Files to read first

- `src/rpp_controller_node.py` — control loop, after yaw command is
  computed.

## Scope

### A. Parameters

- `rpp_enable_xtrack_integral` (bool, default False).
- `rpp_xtrack_ki` (float, default 0.3). Units: rad / (m·m).
- `rpp_xtrack_i_max_rad` (float, default 0.05 ≈ 3°). Anti-windup cap.
- `rpp_xtrack_i_offpath_reset_m` (float, default 0.20). Reset threshold.

### B. State

`self._xtrack_integral_rad = 0.0`. Reset in the conditions above.

### C. Wiring

After `ψ_cmd` is computed (post-Stanley blend if enabled), apply the bias
subtraction. Always log `i_e` and `δ_bias` to `/rpp/debug` for offline
verification.

### D. Tests

`tests/test_xtrack_integral.py`:

- Constant xtrack +5 cm, ds = 0.4 m/cycle × 50 cycles → i_e = 10 m·cm = 0.1
  → δ_bias = 0.3 · 0.1 = 0.03 rad ≈ 1.7°. Verify clamp at 3°.
- Reset on |e| > 0.20 → i_e = 0.
- Sign: positive xtrack (right of path) → negative δ_bias (turn left to
  return).

## Out of scope

- Adaptive Ki (manual gain).
- Separate I-term per heading vs xtrack.

## Acceptance criteria

- [ ] On straight 5 m path with bias on, the steady-state mean xtrack drops
      to within ±5 mm (vs ±1-2 cm with I-term off).
- [ ] No corner-overshoot increase on the 2 m square (the windup-reset on
      off-path entry prevents this).
- [ ] Unit tests pass.

## Notes for the agent

- Per-cycle ds: use `v_cmd · dt` where dt is the control-loop period
  (1/50 s by default). Cheaper than re-computing arc length.
- The reset on off-path entry is the most important detail — without it,
  the rover does an aggressive correction after re-acquisition.
