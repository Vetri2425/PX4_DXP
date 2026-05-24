# 03 — Path geometry module (arc-length + κ lookup, NO spline interpolation)

**Agent:** GLM (4.5 or 5.1)
**Estimated diff:** +250 lines (1 new module, controller wiring for projection + κ)
**Depends on:** 02 (baseline)
**Blocks:** 04 (κ FF), 05 (Stanley blend), 06 (speed profile), 11 (dynamic speed)

## 0. Application context

This rover paints **road markings** and **sports field lines**. The path
fed to RPP is the **physical line on the ground that will be marked**.

Hard constraints:
- **Waypoints are sacred.** A 90° corner in the input stays 90°. No
  interpolating splines, no Bézier rounding — the path passes through
  every waypoint exactly.
- **The trajectory planner owns feasibility.** It decides pivot-turn
  insertion, pen-up/down, and curve densification. RPP trusts its path.
- **Heading IS the output.** The marker pen is offset from centre — heading
  drift = paint drift, even at zero xtrack.

## 1. Goal

Give the controller a proper arc-length-parameterised path with clean κ(s)
lookups, replacing the polyline-segment projection. This is the **geometry
foundation** that all subsequent upgrades (κ FF, Stanley blend, speed
profile) build on.

**RPP remains the primary steering law.** This task adds geometry only —
no change to the steering target computation. Stanley blend comes in task 05.

## 2. What this task solves

The current polyline representation causes:
- Vertex-induced κ spikes at corners → predictive regulation over-decelerates.
- Segment-walking projection is O(N) worst case, no arc-length for speed
  profile lookup.
- No path-tangent heading reference (needed by Stanley blend in task 05).

## 3. Design — `src/path_geometry.py` (new module)

Pure-math, no rclpy. Class `PathGeometry`:

```python
PathGeometry(points, kappa_window_m=0.20, kappa_lpf_alpha=0.3,
             soft_corner_deg=10.0, hard_corner_deg=45.0)
  .length_m                            # total arc length
  .pos(s)        -> (n, e)             # linear interp on polyline
  .tangent(s)    -> (n̂, ê)             # finite diff on segment
  .yaw_path(s)   -> ψ in NED radians   # = atan2(ê, n̂)
  .kappa(s)      -> κ                  # windowed Menger (see below)
  .vertex_type(i) -> {SMOOTH, SOFT, HARD, ENDPOINT}
  .next_hard_corner_s(s_from) -> s | None
  .project(n, e, hint_s=None) -> (s, signed_xtrack)
```

### κ computation (windowed Menger)

For a sample at arc length `s`:

1. Collect all vertices within `kappa_window_m / 2` on each side of `s`
   (≥ 3 vertices needed; if fewer, return κ = 0).
2. For each interior vertex `v_i`, compute Menger curvature from
   `(v_{i-1}, v_i, v_{i+1})`:
   `κ_i = 4·area(triangle) / (|v_{i-1}v_i| · |v_iv_{i+1}| · |v_{i-1}v_{i+1}|)`
3. Sign by cross-product direction (right-of-path = +).
4. Return arc-length-weighted average of `κ_i` over the window.
5. Run a single-pole IIR over s with `α = kappa_lpf_alpha` to remove
   discrete-sample noise.

Result: pure straights return κ ≈ 0. Densified arcs return κ ≈ true arc
curvature. Hard corners return a single high-κ blip the width of one window.

**NO interpolation between waypoints.** The polyline stays as the planner
gave it. This module adds arc-length indexing and curvature — it does not
modify the path shape.

### Vertex classification

For each interior vertex `v_i`:

```
α_i = signed turn angle between (v_{i-1} → v_i) and (v_i → v_{i+1})
type = SMOOTH if |α_i| < soft_corner_deg
       SOFT   if soft_corner_deg ≤ |α_i| < hard_corner_deg
       HARD   if |α_i| ≥ hard_corner_deg
```

HARD vertex type is used by the pivot sub-state (task 14). SMOOTH and SOFT
are tracked identically by the controller.

## 4. Wiring into `rpp_controller_node.py`

- Replace `_path_cb` body: build `PathGeometry` once on path receipt. Cache
  as `self._path_geom`.
- Replace `_project_onto_path`: use
  `self._path_geom.project(pos_n, pos_e, hint_s=self._last_s)` and store
  `self._last_s` for the next cycle.
- Predictive κ block (P1.1): replace polyline-derived κ with
  `max(self._path_geom.kappa(s_la + i * Δs) for i in range(n_preview))`.
- **No change to the steering target computation.** RPP lookahead + α →
  velocity vector remains unchanged. The geometry module only improves the
  projection and κ inputs.

## 5. Parameters

| Param | Default | Range | Meaning |
|---|---|---|---|
| `rpp_enable_geometry_v2` | False | bool | master switch for PathGeometry |
| `rpp_kappa_window_m` | 0.20 | [0.05, 1.0] | Menger window |
| `rpp_kappa_lpf_alpha` | 0.30 | [0.0, 0.95] | κ LPF |
| `rpp_soft_corner_deg` | 10.0 | [5, 30] | SMOOTH/SOFT boundary |
| `rpp_hard_corner_deg` | 45.0 | [30, 90] | SOFT/HARD boundary |

All default-OFF. Flip to True only after baseline-vs-on hardware A/B.

## 6. Files to read first

- `src/rpp_controller_node.py` — `_path_cb`, `_project_onto_path`,
  `_predictive_kappa_scan`, control loop.
- `Test_mission/mission_half_circle_180.waypoints` — densified arc, tests κ.
- `Test_mission/mission_straight_5m.waypoints` — pure straight, tests κ = 0.
- `Test_mission/mission_square.waypoints` — HARD corners.

## 7. Tests

`tests/test_path_geometry.py`:

- Straight line of 10 points → κ = 0 everywhere within 1e-6.
- Densified arc R=1.5 m, 100 points → κ = 0.667 ± 0.03 within the arc,
  sign correct.
- Square corners → vertex_type = HARD for all 4 corners.
- 45° kink → vertex_type = SOFT.
- Projection: any point within 5 cm of the path converges within 3
  Newton-like steps.

## 8. Acceptance criteria (hardware A/B on bench)

Baseline (task 02) captured for all test paths. Then with geometry v2 on:

- [ ] **Straight 5 m**: no regression — xtrack within ±1 cm of baseline.
- [ ] **Half-circle 180°**: κ lookup returns stable values, no vertex spikes.
- [ ] **2 m square**: vertex_type flags all 4 corners as HARD.
- [ ] All `tests/test_path_geometry.py` pass.
- [ ] CPU on Jetson < 3 % single-core during a 30 s run.

## 9. Out of scope

- Interpolating splines of any kind (cubic, Bézier, NURBS). They round
  corners and deviate from waypoints — wrong for marking.
- Steering law changes (Stanley blend → task 05, pivot → task 14).
- Speed profile (task 06).
- Online path re-planning. One-shot build on path receipt.

## 10. References

- Coulter. *Implementation of the Pure Pursuit Path Tracking Algorithm.*
  CMU-RI-TR-92-01, 1992. (Pure Pursuit baseline we're building on.)
- Nav2 RPP controller:
  https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html
- Menger curvature for discrete curves:
  https://en.wikipedia.org/wiki/Menger_curvature