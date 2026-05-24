# 03 — Path geometry + Stanley heading tracking (REPLACES old 03/04/05)

**Agent:** GLM (4.5 or 5.1)
**Estimated diff:** +450 lines (1 new module, 1 controller rewrite of the steering block)
**Depends on:** 02 (baseline)
**Blocks:** 06 (needs the new path geometry API), 11 (heading-aware speed reg)
**Supersedes:** old `03_path_smoothing_cubic_spline.md`, `04_path_kappa_feedforward.md`, `05_stanley_xtrack_blend.md` (all three are folded into this single, application-honest spec).

---

## 0. Application context (read this before anything else)

This rover paints **road markings** and **sports field lines**. The path
fed to RPP is the **physical line on the ground that will be marked**.

Implications that change everything:

- **Waypoints are sacred.** A 90° corner in the input is a 90° corner the
  customer paid for. We do NOT round corners by deviating from waypoints.
  No interpolating splines, no Bézier shortcuts.
- **The upstream trajectory planner owns feasibility.** It receives DXF /
  CSV via the frontend, decides where to insert pivot-turns, where to lift
  the pen (separate channel), and where to densify curves. RPP TRUSTS the
  path it receives is already feasible — it must not second-guess by
  smoothing it.
- **Heading IS the output.** The marker pen is mounted offset from the
  rover's centre of rotation. If the rover's heading drifts from the path
  tangent, the painted line drifts the same amount, even at zero xtrack.
  Heading control is as important as cross-track.
- **Speed is the deviation amplifier.** At 0.4 m/s, a 0.1 s controller lag
  is 4 cm of drift. Speed must be regulated to keep the rover within the
  controller's bandwidth.

## 1. Goal

Drive xtrack on straights to **1-2 cm steady** and corner overshoot to
**2-5 cm peak**. No compromise on path fidelity — the rover passes through
every input waypoint within `goal_radius_m`.

## 2. What's wrong with the current controller

Three coupled problems, in order of contribution to the 9.4 cm corner
xtrack baseline:

1. **Pure-Pursuit heading is derived, not commanded.** The current
   controller outputs a velocity vector (`v_n, v_e`); twist_to_setpoint
   derives yaw via `atan2(v_n, v_e)`. The rover's *physical* heading
   tracks this derived target with lag — the marker pen wobbles even when
   the rover trajectory is correct.
2. **κ(s) is computed from polyline geometry.** Vertex spikes at corners
   make the predictive κ regulation (P1.1) react late and over-decelerate.
3. **No path-tangent heading reference.** Stanley-style control (yaw
   target = path tangent + xtrack correction) is the proven solution for
   tight tracking on smooth paths. Currently absent.

## 3. Design (one controller, three blocks)

```
┌──────────────────────────────────────────────────────┐
│  Block A: PathGeometry (new module, path_geometry.py)│
│   • arc-length parameterisation of input polyline    │
│   • per-vertex angle classification:                 │
│       SMOOTH (<10°), SOFT (10°-45°), HARD (>45°)     │
│   • lookups: pos(s), tangent(s), kappa(s)            │
│   • κ(s) from windowed Menger over 5 neighbour       │
│     segments, arc-length LPF                         │
│   • NO interpolation between waypoints — the path    │
│     stays as the planner gave it                     │
└──────────────────────────────────────────────────────┘
                          │
                          ▼
┌──────────────────────────────────────────────────────┐
│  Block B: Heading-first steering                     │
│   • on SMOOTH + SOFT regions:                        │
│       ψ_cmd = ψ_path(s_proj)                         │
│              - atan2(k_st · xtrack, v + v_soft)      │
│              - k_hb · b̂   (bias correction, task 12) │
│              - k_i  · ∫xtrack ds (task 07)           │
│       v_cmd = v_ref(s_la)   (task 06 speed profile)  │
│              · g_xtrack(|e|)                         │
│              · g_heading(|ψ_e|)   ← NEW              │
│   • on HARD corner approach:                         │
│       enter PIVOT state at d=approach_pivot_m        │
│       v_cmd = 0                                      │
│       ψ_cmd = ψ_next_segment                         │
│       wait until |ψ_e| < 3° AND |ω| < 5°/s           │
│       resume normal tracking                         │
└──────────────────────────────────────────────────────┘
                          │
                          ▼
┌──────────────────────────────────────────────────────┐
│  Block C: Output                                     │
│   • publish /rpp/velocity_ned   (v_cmd · ψ̂_cmd)      │
│   • publish /rpp/yaw_rate_body  (ω_ff = κ · v_cmd)   │
│   • twist_to_setpoint already converts to ENU and    │
│     uses explicit yaw — no change needed there       │
└──────────────────────────────────────────────────────┘
```

## 4. Block-by-block contract

### Block A — `src/path_geometry.py` (new module)

Pure-math, no rclpy. Class `PathGeometry`:

```
PathGeometry(points, kappa_window_m=0.20, kappa_lpf_alpha=0.3,
             soft_corner_deg=10.0, hard_corner_deg=45.0)
  .length_m                            # total arc length
  .pos(s)        -> (n, e)             # linear interp on polyline
  .tangent(s)    -> (n̂, ê)             # finite diff on segment
  .yaw_path(s)   -> ψ in NED radians   # = atan2(ê, n̂)
  .kappa(s)      -> κ                  # see § κ computation below
  .vertex_type(i) -> {SMOOTH, SOFT, HARD, ENDPOINT}
  .next_hard_corner_s(s_from) -> s | None
  .project(n, e, hint_s=None) -> (s, signed_xtrack)
```

#### κ computation (the careful bit)

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

Result: pure straights return κ ≈ 0 (no Runge oscillation). Densified
arcs return κ ≈ true arc curvature. Hard corners return a single
high-κ blip the width of one window — and they're flagged via
`vertex_type` so the controller can pivot-turn instead of trying to
follow.

#### Vertex classification

For each interior vertex `v_i`:

```
α_i = signed turn angle between (v_{i-1} → v_i) and (v_i → v_{i+1})
type = SMOOTH if |α_i| < soft_corner_deg
       SOFT   if soft_corner_deg ≤ |α_i| < hard_corner_deg
       HARD   if |α_i| ≥ hard_corner_deg
```

Endpoints are always `ENDPOINT`. The controller treats SMOOTH and SOFT
identically; HARD triggers the pivot sub-state.

### Block B — Heading-first steering (rewrites the steering portion of `rpp_controller_node.py`)

#### B.1 — Steering target on SMOOTH/SOFT

```
ψ_tangent  = self._path.yaw_path(s_proj)
e_perp     = signed_xtrack
v          = max(v_cmd_last, v_soft)              # avoid divide-by-zero
δ_stanley  = atan2(k_st · e_perp, v)
ψ_cmd_raw  = wrap_pi(ψ_tangent - δ_stanley
                    - k_hb · self._bias_hat       # task 12, if enabled
                    - k_i · self._xt_integral)    # task 07, if enabled
```

`k_st` default 1.2. `v_soft` default 0.1 m/s. Wrap to `[-π, π]`.

This is the Stanley law — proven on DARPA Grand Challenge, lower steady
xtrack than Pure Pursuit, no lookahead-distance tuning. Critically:
**ψ_tangent is the heading we'd hold if we were perfectly on path**, so
the rover stays parallel to the line. Pure Pursuit's lookahead heading
drifts ahead of the tangent on curves — that's why our marker pen
swings on corners.

#### B.2 — Yaw rate feedforward (always on)

```
ω_ff = self._path.kappa(s_la) · v_cmd     # rad/s
```

Published to `/rpp/yaw_rate_body`. Removes the lag between commanded
heading change and actual rotation on smooth curves.

#### B.3 — Speed regulation

Three multiplicative regulators:

```
v_profile  = self._speed_profile[clamp(s_la / ds, 0, N-1)]  # task 06
g_xtrack   = clamp(1 - |e_perp| / e_max_xtrack, v_min_frac, 1)
g_heading  = clamp(1 - |ψ_e| / ψ_max_err,       v_min_frac, 1)
v_cmd      = v_profile · g_xtrack · g_heading
```

`e_max_xtrack` = 0.10 m → at 10 cm off-path, speed drops to `v_min_frac`.
`ψ_max_err` = 15° → at 15° heading misalignment, speed drops to
`v_min_frac` = 0.25.

The `g_heading` regulator is the new piece. Without it, the rover can
exit a corner with the right *position* but wrong *heading*, and lay a
diagonal slash of paint instead of continuing along the line.

#### B.4 — Hard-corner pivot sub-state

State machine extension:

```
states: STALE, IDLE, TRACKING, PIVOT_APPROACH, PIVOT_TURN, APPROACH, DONE
```

Transitions:
- TRACKING → PIVOT_APPROACH when `dist_to_next_hard_corner < approach_pivot_m`
  (default 0.30 m).
- PIVOT_APPROACH: continue normal tracking but cap `v_cmd ≤ 0.10 m/s`
  (gradual decel before the corner; smooth taper).
- PIVOT_APPROACH → PIVOT_TURN when reaching the corner vertex
  (`dist_to_vertex < 0.05 m`).
- PIVOT_TURN: `v_cmd = 0`; `ψ_cmd = ψ_next_segment`; publish via
  `/rpp/yaw_rate_body` for crisper rotation than yaw-target-only.
- PIVOT_TURN → TRACKING when `|ψ_e| < 3°` AND `|ω_measured| < 5°/s`
  (rotation settled). Re-acquire path with `s_proj` reset to the corner
  vertex.

PIVOT_TURN time-cap: 4 s. If exceeded, log warning and resume tracking
anyway (avoid infinite hang).

### Block C — Output

No changes to `twist_to_setpoint_node.py`. It already accepts NED
velocity + (now) explicit yaw via the velocity-vector derivation. We
publish `(v_cmd, ψ_cmd)` decomposed as `(v_n, v_e) = v_cmd · (cos ψ, sin ψ)`
(NED, ψ measured from North CW).

For body-rate path (P3.1): publish `ω_ff` to `/rpp/yaw_rate_body` always
when the feature is on; downstream node forwards to PX4 if PX4 is
configured for body-rate OFFBOARD.

## 5. Parameters

All default-OFF until baseline-vs-on hardware A/B passes.

| Param | Default | Range | Meaning |
|---|---|---|---|
| `rpp_enable_geometry_v2` | False | bool | master switch for Block A+B |
| `rpp_kappa_window_m` | 0.20 | [0.05, 1.0] | Menger window |
| `rpp_kappa_lpf_alpha` | 0.30 | [0.0, 0.95] | κ LPF |
| `rpp_soft_corner_deg` | 10.0 | [5, 30] | SMOOTH/SOFT boundary |
| `rpp_hard_corner_deg` | 45.0 | [30, 90] | SOFT/HARD boundary |
| `rpp_stanley_gain` | 1.2 | [0.5, 3.0] | Stanley k_st |
| `rpp_stanley_v_soft_m_s` | 0.10 | [0.01, 0.30] | speed softener |
| `rpp_yaw_ff_enable` | True | bool | curvature feedforward |
| `rpp_e_max_xtrack_m` | 0.10 | [0.05, 0.50] | speed-vs-xtrack roll-off |
| `rpp_psi_max_err_deg` | 15.0 | [5, 45] | speed-vs-heading roll-off |
| `rpp_v_min_frac` | 0.25 | [0.05, 1.0] | speed floor as fraction of profile |
| `rpp_pivot_approach_m` | 0.30 | [0.10, 1.0] | distance to begin slowdown |
| `rpp_pivot_settle_deg` | 3.0 | [1, 10] | heading tolerance to exit PIVOT |
| `rpp_pivot_settle_rate_dps` | 5.0 | [1, 20] | yaw-rate tolerance to exit PIVOT |
| `rpp_pivot_timeout_s` | 4.0 | [1, 10] | safety cap |

## 6. Files to read first

- `src/rpp_controller_node.py` — lines 977-1300 (control loop), 1338-1373 (debug publish).
- `src/twist_to_setpoint_node.py` — confirm explicit-yaw path is live (post-`fd91d0c`).
- `Test_mission/mission_square.waypoints` — 4-corner test that this design targets.
- `Test_mission/mission_half_circle_180.waypoints` — densified arc, tests κ computation.
- `Test_mission/mission_straight_5m.waypoints` — pure straight, tests xtrack noise floor.

## 7. Tests

`tests/test_path_geometry.py`:
- Straight line of 10 points → κ = 0 everywhere within 1e-6.
- Densified arc R=1.5 m, 100 points → κ = 0.667 ± 0.03 within the
  arc, sign correct.
- Square corners → vertex_type = HARD for all 4 corners.
- 45° kink → vertex_type = SOFT.
- Projection: any point within 5 cm of the path converges within 3
  Newton-like steps.

`tests/test_stanley_steering.py`:
- On straight, e=0, ψ_e=0 → ψ_cmd = ψ_tangent.
- On straight, e=+5 cm, v=0.4 → δ_stanley = atan2(1.2·0.05, 0.4) ≈ 8.5°
  toward path.
- On curve, on-path → ψ_cmd = ψ_tangent (FF handles curvature, not
  Stanley).
- Sign agreement: positive xtrack → negative δ → turn left in NED.

`tests/test_pivot_state.py`:
- HARD corner ahead → state transitions on cue.
- Settle conditions enforce both heading AND rate.
- Timeout fires after 4 s.

## 8. Acceptance criteria (hardware A/B on bench)

Baseline (task 02) captured for all three test paths. Then with this
upgrade on:

- [ ] **Straight 5 m**: mean |xtrack| < 1 cm, max |xtrack| < 2 cm.
- [ ] **Half-circle 180°**: mean |xtrack| < 2 cm, max |xtrack| < 3 cm.
- [ ] **2 m square**: max |xtrack| < 5 cm at corners (sharp 90°
      remains hard; pivot-turn gives us the tight approach), mean on
      straights < 1.5 cm.
- [ ] **Endpoint precision**: distance from final pose to last waypoint
      < 3 cm on all paths.
- [ ] **No new oscillation** on any path (verify yaw-rate trace).
- [ ] **PIVOT_TURN never times out** on any test path (settle works).
- [ ] All `tests/test_*.py` pass.
- [ ] CPU on Jetson < 5 % single-core during a 30 s run.

If targets are missed by a factor > 1.5×, the upgrade is a regression —
revert and open a follow-up task to investigate.

## 9. Out of scope

- Pen-up / pen-down control. That's a separate channel owned by the
  trajectory planner.
- DXF parsing / mission decomposition. Trajectory planner.
- Online path re-planning. One-shot path on receipt.
- Heading-at-goal control (orient to face a specific direction at goal).
  The natural fall-out of Block B is that the rover ends pointing along
  the last segment's tangent, which is what marking applications want
  99 % of the time.
- Pose-extrapolation (P2.4) interaction with PIVOT: disable extrapolation
  inside PIVOT_TURN (extrapolating zero-velocity pose forward by dt is
  pointless and risks false settle).

## 10. References

- Hoffmann, Tomlin, Montemerlo, Thrun. *Autonomous Automobile Trajectory
  Tracking for Off-Road Driving: Controller Design, Experimental
  Validation and Racing.* ACC 2007. (Stanley controller — the heading-
  first design we're adopting.)
- Coulter. *Implementation of the Pure Pursuit Path Tracking Algorithm.*
  CMU-RI-TR-92-01, 1992. (Why we're NOT using Pure Pursuit as primary.)
- Nav2 controller comparison: their `RegulatedPurePursuit` is what we're
  beating; their `MPPIController` is the long-game upgrade if we ever
  want adaptive obstacle avoidance.
  https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html

## 11. Migration note

This spec **supersedes** the old `03_path_smoothing_cubic_spline.md`
(rejected: interpolating splines overshoot at marking-application
corners), `04_path_kappa_feedforward.md` (folded into Block B.2), and
`05_stanley_xtrack_blend.md` (folded into Block B.1 as the *primary*
steering, not a blend).

The old files now carry a deprecation header pointing here. Do not
implement them; implement this one.
