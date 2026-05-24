# 09 — κ low-pass filter + flip yaw-FF default to ON

**Agent:** Gemma 3 1B
**Estimated diff:** +15 lines
**Depends on:** 03 (spline κ), 04 (κ FF wired)
**Blocks:** —

## Goal

Two tiny, mechanical changes that smooth steering output:

1. Pass the path curvature used for FF through a single-pole low-pass to
   reduce jerk at spline-sample boundaries.
2. Flip the P3.1 yaw-rate feedforward parameter default from OFF to ON.
   P3.1 has been hardware-validated; default-OFF was a safety choice for
   initial release. Now it's just a wart.

## Math (low-pass)

Single-pole IIR:

```
κ_lpf ← α · κ_lpf + (1 - α) · κ_raw
```

`α ∈ [0, 1)`. Default 0.3 → cutoff ≈ 18 Hz at 50 Hz loop. Aggressive
enough to kill sample-boundary discontinuities, gentle enough not to add
visible lag.

Initialize `κ_lpf = 0.0`. Reset to 0 on path receipt and on IDLE → TRACKING.

## Files to read first

- `src/rpp_controller_node.py` — find where the κ used for FF is consumed
  (after task 04 lands). Find the P3.1 enable parameter.

## Scope

### A. Parameters

- `rpp_kappa_lpf_alpha` (float, default 0.3). Range [0, 0.95].
- The existing P3.1 enable parameter (whatever its name is, check the
  code) — change its default from False to True. Do NOT remove the
  parameter; leave it as a kill-switch.

### B. State

`self._kappa_lpf = 0.0`. Reset on path receipt.

### C. Wiring

Before using `κ_path` in any FF term, apply the LPF. Do not LPF the κ used
for the speed profile lookup — that's already smoothed by the precompute.

### D. No new tests required

The unit tests from tasks 03 and 04 cover correctness. This is a
parameter-level tweak.

## Out of scope

- Higher-order filters.
- Adaptive cutoff.
- Removing the P3.1 parameter entirely.

## Acceptance criteria

- [ ] On any path, the yaw-rate setpoint trace (capture from a bag) shows
      no high-frequency spikes that aren't present in the path itself.
- [ ] Default behaviour with all task 03/04/09 features on shows smooth
      corner traversal in the bag.
- [ ] P3.1 yaw FF is on by default and the rover behaves identically to
      previous tests where it was explicitly enabled.

## Notes for the agent

- This is the smallest task in the folder. Don't expand its scope.
- α = 0.3 was chosen because it's the same value the existing pose
  extrapolation uses for its IIR — kept consistent on purpose.
