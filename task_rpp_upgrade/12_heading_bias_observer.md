# 12 — Heading-bias online observer (DEFERRED)

**Status:** **DEFERRED** — The UM982 dual-antenna RTK GNSS provides true
heading directly from the two-antenna baseline. No magnetometer bias to
estimate. Implement this task ONLY if dual-antenna heading becomes
unavailable (e.g., RTK FLOAT or NO-FIX fallback).

**Original spec preserved below for reference.**

---

**Agent:** GLM (4.5 or 5.1)
**Estimated diff:** +150 lines (1 new module + control-loop hook)
**Depends on:** 03 (path tangent), 07 (xtrack I-term context — observer is a sibling, not a replacement)
**Blocks:** —

## Goal

The integral term in task 07 catches steady xtrack bias but takes time
(integrating arc length) and is reset on off-path entry. An *observer*
that tracks the heading bias `b̂` directly converges faster on long runs
and survives re-acquisition events.

**Why DEFERRED:** Our UM982 dual-antenna RTK gives true heading from the
antenna baseline — no compass involved, no magnetic declination or
hard-iron offset. The xtrack bias on straights (1-3 cm in log 59) is well
within tolerance without a heading-bias correction. This task should be
revived ONLY if:

1. Dual-antenna heading drops out (RTK FLOAT/NO-FIX) and we fall back to
   GPS COG + compass, OR
2. Field testing shows a consistent lateral bias > 2 cm that the I-term
   (task 07) doesn't converge within 5 m.

## Math

Single-state Kalman-flavour observer:

```
state:   b̂ — estimate of (heading_measured - heading_true) in radians
input:   e_⊥ (signed xtrack), v_path (longitudinal projection of velocity),
         ψ_e (heading error wrt path tangent)
```

When the rover is on a straight section (`|κ_path|` < 0.05) and moving
forward (`v_path` > 0.1):

```
e_⊥_rate ≈ v_path · sin(ψ_e + b)              # small-angle: ≈ v_path · (ψ_e + b)
```

If `ψ_e_measured = ψ_path - ψ_vehicle_measured = (ψ_path - ψ_vehicle_true) - b
            = ψ_e_true - b`, then:

```
e_⊥_rate = v_path · (ψ_e_measured + b - b) = v_path · ψ_e_measured
```

— bias cancels out in the rate. But the *steady state* xtrack accumulates
proportional to the bias times the integration window. So observe the
running mean:

```
ε ← x · (1 - α) · e_⊥ + α · ε        # IIR low-pass on xtrack itself
b̂ ← b̂ + k_obs · (ε - 0) · sign(v_path)
b̂ ← clamp(b̂, -b_max, +b_max)
```

with `α = 0.99`, `k_obs = 0.01`, `b_max = 0.10 rad ≈ 5.7°`.

Apply correction:

```
ψ_vehicle_corrected = ψ_vehicle_measured - b̂
ψ_e_corrected = ψ_path - ψ_vehicle_corrected
```

## Files to read first

- `src/rpp_controller_node.py` — heading reads from `_pose_cb`.
- The Stanley blend (task 05) and κ FF (task 04) for where to apply the
  correction.

## Scope

### A. New module `src/heading_bias_observer.py`

Pure-math class:

```
HeadingBiasObserver(
    alpha=0.99,
    k_obs=0.01,
    b_max=0.10,
    straight_kappa_threshold=0.05,
    min_v=0.10,
)
  .update(e_perp, kappa_path_local, v_path) -> b_hat
  .reset()
```

Observer only updates when `|κ_path|` < threshold AND `v_path` > min_v.

### B. Wiring

In the control loop, after projection and before the steering target:

```
if rpp_enable_heading_bias_observer:
    b_hat = self._bias_obs.update(signed_xtrack, kappa_path, v_path)
    yaw_vehicle_corrected = wrap(yaw_vehicle - b_hat)
else:
    yaw_vehicle_corrected = yaw_vehicle
```

### C. Parameters

- `rpp_enable_heading_bias_observer` (bool, default False).
- `rpp_bias_alpha` (float, default 0.99).
- `rpp_bias_k_obs` (float, default 0.01).
- `rpp_bias_b_max_rad` (float, default 0.10).

### D. Tests

`tests/test_heading_bias_observer.py`:

- Inject a constant bias of 0.05 rad on a straight path → observer
  converges to b̂ ≈ 0.05 within 5 s.
- On a curved path → observer holds previous estimate (no update).
- v = 0 → no update.
- Bias > b_max → clamps.

## Acceptance criteria

- [ ] On a 5 m straight run with a deliberately offset compass (manually
      add 3° via `MAG_ROT_*`), observer converges to b̂ ≈ 3° within 8 s.
- [ ] Mean steady xtrack drops to within ±5 mm by 10 s.
- [ ] On the 2 m square, no regression: bias holds during corners,
      updates only on straights.

## Notes for the agent

- Reference: same principle as the wind-bias observer in fixed-wing
  autopilots. Single-state, slow, observability-gated.
- This observer assumes a constant-in-time bias. If the bias is actually
  yaw-rate-proportional (gyroscope scale error), the observer will track
  it incorrectly during turns. The active-region gate prevents that
  failure mode.