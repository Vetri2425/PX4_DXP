# 03 (DEPRECATED) — Cubic-spline path smoothing

> **DEPRECATED 2026-05-24.** This spec is rejected for the marking-rover
> application. Interpolating cubic splines through waypoints overshoot at
> sharp corners (Runge-like oscillation), which would round the corners
> our rover is paid to paint sharply.
>
> **Use `03_path_geometry_and_stanley_tracking.md` instead.** That spec
> uses arc-length parameterisation of the polyline (no interpolation),
> windowed Menger curvature for κ(s), and a HARD-corner pivot sub-state
> for 90° turns. It also folds in tasks 04 and 05.

---

# (Original spec below, kept for historical context — DO NOT IMPLEMENT)



**Agent:** GLM (4.5 or 5.1)
**Estimated diff:** +300 lines (1 new module, controller edit), -50 lines
**Depends on:** 02 (baseline)
**Blocks:** 04 (needs κ(s)), 06 (needs v(s)), 12 (needs ψ_path(s))

## Goal

Replace the polyline path representation with a C¹ cubic-spline,
arc-length-parameterised, with smooth κ(s) and ψ_path(s) lookups. This kills
the dominant source of xtrack error: vertex-induced curvature spikes at
mission waypoint corners.

## Mathematical contract

Given input waypoints `P = [(n_0, e_0), ..., (n_{N-1}, e_{N-1})]`:

1. Optional pre-smoothing (corner-rounding by inscribed arcs of radius
   `r_corner`, like P1.3). Use this for marking paths where the path **must
   pass through** every waypoint exactly — skip for "soft" missions.
2. Fit a natural cubic spline `(N(t), E(t))` parameterised by chord
   parameter `t ∈ [0, T]`.
3. Compute arc length `s(t) = ∫₀ᵗ √(N'(τ)² + E'(τ)²) dτ` numerically (cumulative
   Simpson's rule).
4. Build inverse table `t(s)` via dense sampling + monotone interpolation.
5. Expose lookups:
   - `pos(s) → (n, e)`
   - `tangent(s) → (n̂, ê)`, then `ψ_path(s) = atan2(ê, n̂)` (NED yaw)
   - `kappa(s) = (N'·E'' - E'·N'') / (N'² + E'²)^(3/2)`
6. Sampling resolution for tables: **1 cm of arc length**. For a 20 m path
   that's a 2000-element array per channel — trivially fast.

## Files to read first

- `src/rpp_controller_node.py` — current `_resample_path`, `_smooth_corners`,
  `_project_onto_path`. Lines 571-870.
- `Test_mission/mission_half_circle_180.waypoints` — a densified arc; good
  sanity-check input.

## Scope

### A. New module `src/path_geometry.py`

A pure-math module (no rclpy). Class `SplinePath`:

- Constructor: takes list of `(n, e)` and optional smoothing params.
- Methods: `pos(s)`, `tangent(s)`, `kappa(s)`, `project(n, e, hint_s=None)
  → (s, signed_xtrack)`.
- `project` uses Newton iteration from `hint_s` (replaces the segment
  walker but takes a 1D scalar hint instead of a segment index).
- Total length `self.length_m`.

### B. Wire into `rpp_controller_node.py`

Replace `_path_cb` body: build `SplinePath` once on path receipt. Cache it
on `self._spline`.

Replace `_project_onto_path` callsite: use `self._spline.project(pos_n,
pos_e, hint_s=self._last_s)` and store `self._last_s` for the next cycle.

Lookahead block: instead of walking forward by `L_d` over polyline segments,
compute `s_la = self._last_s + L_d` and call `self._spline.pos(s_la)`.

Predictive κ block (P1.1): replace Menger-curvature over polyline samples
with `max(self._spline.kappa(s_la + i * Δs) for i in range(n_preview))`.

### C. Parameter

`rpp_enable_spline_smoothing` (bool, default **False** initially). Flip
default to **True** after baseline-vs-on hardware A/B confirms <5 cm corner
xtrack.

Keep a `rpp_pre_smooth_corner_radius_m` param. Default 0.0 (off) — spline
alone covers most of the benefit for densified inputs.

### D. Tests

`tests/test_spline_path.py`:

- Straight line of 10 points → projection on midpoint = (5, 0), xtrack = 0,
  κ everywhere = 0, total length = 9 (if spacing 1 m).
- Unit circle of 20 points → κ ≈ 1.0 everywhere (within 5 %), ψ_path
  monotonically advances.
- Projection convergence: any test point within 5 cm of the path converges
  to <1 mm in ≤ 3 Newton iterations.
- Densified arc from `Test_mission/mission_half_circle_180.waypoints` →
  spline matches input within 1 mm RMS.

## Out of scope

- B-splines, Bézier, NURBS. Cubic natural spline is sufficient and avoids
  knot tuning.
- Online re-fitting (dynamic obstacle avoidance). One-shot fit on path
  receipt only.
- Path with self-intersections — undefined behaviour.

## Acceptance criteria

- [ ] All unit tests pass.
- [ ] With `rpp_enable_spline_smoothing=True`, the 2 m square baseline run
      shows **max xtrack at corners < 5 cm** (vs 9.4 cm baseline) and no
      worse on straights.
- [ ] With `rpp_enable_spline_smoothing=False`, the 2 m square baseline run
      reproduces the previous 9.4 cm corner result within ±1 cm
      (regression-free fallback).
- [ ] Newton projection never diverges over a 30 s run (no NaN, no infinite
      loop).
- [ ] CPU on Jetson stays under 5 % single-core during a 30 s run.

## Notes for the agent

- For natural cubic spline coefficients, use scipy's `CubicSpline` with
  `bc_type='natural'` if scipy is already a dep; otherwise hand-roll the
  Thomas-algorithm tridiagonal solve (≤ 50 lines).
- Cumulative arc length: trapezoidal is enough; Simpson is overkill for
  rover speeds. The integration error budget is 1 mm per metre.
- Newton step for projection:
  `s_{k+1} = s_k - ((p - pos(s_k)) · tangent(s_k)) / |tangent(s_k)|²`.
  Clamp to `[0, length_m]`.
- Comment in code: cite Nav2 RPP for comparison
  (`https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller`).
