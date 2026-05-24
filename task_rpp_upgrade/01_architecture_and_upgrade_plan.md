# RPP Controller — Architecture & Upgrade Plan

This is the master design doc for the upgrade. Every task in this folder
plugs into one of the labelled blocks below. If a task contradicts this
plan, fix the plan first, then the task.

## 1. Current pipeline (post-`fd91d0c`)

```
                      (path from mission_runner / path_publisher)
                                    │
                                    ▼
                          ┌────────────────────┐
                          │  Path conditioning  │  P1.3 — opt-in resample + corner smooth
                          │  (in-node)          │
                          └─────────┬──────────┘
                                    │ conditioned path (NED)
                                    │
                          ┌─────────▼──────────┐
   /mavros/pose ──ENU→NED─►  Projection         │  P1.4 — segment-hint, O(1) steady
   /mavros/vel  ──────────►  + latency extrap   │  P2.4 — pose + v·Δt
                          └─────────┬──────────┘
                                    │ (foot_n, foot_e, signed_xtrack, seg_idx)
                                    │
                          ┌─────────▼──────────┐
                          │  Lookahead          │  P0.1 — closed-loop L_d on v
                          │  selection          │  P1.2 — + k_e · |xtrack|
                          └─────────┬──────────┘
                                    │ (LA point, α heading error)
                                    │
                          ┌─────────▼──────────┐
                          │  Predictive κ       │  P1.1 — worst κ over preview N
                          │  scan               │
                          └─────────┬──────────┘
                                    │ κ_preview, κ_steer
                                    │
                          ┌─────────▼──────────┐
                          │  Velocity output    │
                          │  v ∝ f(κ_preview)   │
                          │  yaw_target = α     │
                          └─────────┬──────────┘
                                    │ (v_n, v_e)
                                    ▼
                       /rpp/velocity_ned (NED)
                                    │
                                    ▼
                       twist_to_setpoint_node (50 Hz)
                                    │ NED→ENU
                                    ▼
                       /mavros/setpoint_raw/local (ENU)
                                    │
                                    ▼
                       PX4 OFFBOARD → DiffOffboardMode → RoboClaw
```

## 2. Where the error comes from (baseline 9.4 cm at corners)

Decomposed:

| Source | Estimated contribution | Affected by upgrades |
|---|---|---|
| Vertex-induced κ spikes at square corners | 5-7 cm | 03 (spline), 06 (speed profile) |
| Pure-pursuit lateral overshoot in turns | 2-3 cm | 04 (κ FF), 05 (Stanley blend) |
| Pipeline latency (pose age, command transport) | 0.5-1 cm | 10 (latency LA) |
| Compass / mount bias | 0.5-1 cm steady | 07 (I-term), 12 (bias observer) |
| Final-approach overshoot at goal | 1-2 cm endpoint | 08 (approach PI) |

The first row dominates. Path smoothing is the single highest-impact upgrade
and should land first.

## 3. Target pipeline (post-upgrade)

```
                      (raw path from upstream)
                                    │
                                    ▼
                          ┌────────────────────┐
                          │  Path conditioning  │  (always-on)
                          │  • cubic spline     │  ◄── upgrade 03
                          │  • arc-length       │       (replaces P1.3 resample)
                          │    re-parameterise  │
                          └─────────┬──────────┘
                                    │ s(p), κ(s), ψ_path(s)
                                    │
                                    ▼
                          ┌────────────────────┐
                          │  Speed profile      │  ◄── upgrade 06
                          │  (offline solve)    │       v(s) s.t.  v² · κ ≤ a_lat_max
                          │                     │              and |dv/ds| ≤ accel_max
                          └─────────┬──────────┘
                                    │ v_ref(s)
                                    │
   /mavros/pose ──ENU→NED─►┌────────▼──────────┐
   /mavros/vel  ──────────►│  Projection +     │  (existing + latency LA  ◄── upgrade 10)
                          │  latency extrap   │
                          └─────────┬──────────┘
                                    │ s, xtrack
                                    │
                          ┌─────────▼──────────┐
                          │  Steering target    │
                          │  yaw = blend(       │  ◄── upgrade 05
                          │    pure-pursuit α,  │       Stanley near small e_⊥
                          │    Stanley θ_e +    │
                          │    atan(k·e/v)      │
                          │  )                  │
                          └─────────┬──────────┘
                                    │ ψ_cmd
                                    │
                          ┌─────────▼──────────┐
                          │  Curvature FF       │  ◄── upgrade 04, 09
                          │  ω_ff = κ_path · v  │       always-on yaw FF
                          │  low-pass κ         │
                          └─────────┬──────────┘
                                    │
                          ┌─────────▼──────────┐
                          │  Outer xtrack I     │  ◄── upgrade 07
                          │  + heading bias obs │  ◄── upgrade 12
                          └─────────┬──────────┘
                                    │
                          ┌─────────▼──────────┐
                          │  Goal-approach PI   │  ◄── upgrade 08
                          │  (activates @       │
                          │   d_to_goal < 30cm) │
                          └─────────┬──────────┘
                                    │
                          ┌─────────▼──────────┐
                          │  Speed shaping      │  ◄── upgrade 11
                          │  v = v_ref · g(|e|) │
                          └─────────┬──────────┘
                                    │
                                    ▼
                       /rpp/velocity_ned + /rpp/yaw_rate_body
```

## 4. Block contracts (do not break)

- **Path conditioning input**: any `nav_msgs/Path` in LOCAL_NED with ≥ 2
  points and ≥ 5 cm spacing. Output: arc-length-parameterised cubic spline
  + lookup `κ(s)`, `ψ_path(s)`. Sampling resolution: 1 cm.
- **Speed profile**: precomputed at path-receipt time. Stored as a 1D array
  indexed by arc-length sample. No per-cycle re-solve.
- **Projection**: returns `(s, signed_xtrack, foot_n, foot_e, ψ_path)`. The
  s parameter is the global arc length, not a segment index.
- **Steering output**: still a NED velocity vector. The yaw-rate channel is
  the optional body-rate path (P3.1, becomes default after upgrade 09).
- **Frame**: NED everywhere in this node. ENU at the MAVROS boundary only.

## 5. Default-off discipline

Every upgrade lands behind a parameter `rpp_enable_<feature>` (bool, default
False) UNTIL the upgrade has been validated on hardware with a baseline-vs-on
A/B. Then the default flips to True and a follow-up commit removes the dead
"if False" branch.

This lets us:
- Bisect regressions per-feature.
- Run the old behaviour for comparison without redeploying old code.
- Land risky changes incrementally without holding back safer ones.

## 6. Test path matrix (use for every upgrade validation)

| Path | File | Why |
|---|---|---|
| 2 m square | `Test_mission/mission_square.waypoints` | Baseline corner stress |
| Half circle R=1.5 m, 180° | `Test_mission/mission_half_circle.waypoints` | Constant-curvature, tests κ FF |
| Densified arc R=1.5 m | `Test_mission/mission_half_circle_180.waypoints` (Karney) | Smooth path, isolates controller from path discretisation |
| Straight 5 m | (create if missing) | Reference for steady-state xtrack noise floor |
| S-curve | (create) | Sign-flipping κ — tests bias observer + I-term |

All compared at the same nominal speed (`WP_SPEED` / `cruise_speed` = 0.4 m/s).

## 7. Where each upgrade goes (file map)

| Upgrade | File(s) touched | Lines (est.) |
|---|---|---|
| 03 Spline smoothing | `src/rpp_controller_node.py` (path callback), new `src/path_geometry.py` | +300 / -50 |
| 04 κ FF | `src/rpp_controller_node.py` (control loop) | +40 |
| 05 Stanley blend | `src/rpp_controller_node.py` (steering target) | +60 |
| 06 Speed profile | new `src/speed_profile.py` + path callback | +200 |
| 07 xtrack I | `src/rpp_controller_node.py` (control loop) | +25 |
| 08 Goal approach | `src/rpp_controller_node.py` (state machine) | +80 |
| 09 κ LPF + yaw FF default | `src/rpp_controller_node.py` | +15 |
| 10 Latency LA | `src/rpp_controller_node.py` (lookahead block) | +20 |
| 11 Dynamic speed | `src/rpp_controller_node.py` (output stage) | +20 |
| 12 Heading bias obs | new `src/heading_bias_observer.py` + control loop | +150 |
| 13 Benchmark harness | new `tools/benchmark_rpp.py` | +250 |

Total: ~1100 lines added, mostly behind flags.

## 8. What we keep from the existing code

- Projection with segment hint (P1.4) — feeds the new spline as initial guess.
- Pose latency extrapolation (P2.4) — kept verbatim; upgrade 10 stacks on top.
- RTK fix gate (P0.3) — non-negotiable safety.
- Jump detection (P0.2) — kept.

Predictive κ (P1.1) is *partially* superseded by upgrade 06's pre-computed
speed profile, but stays on as a runtime safety belt against path-receipt
race conditions and dynamic re-routes (when we eventually support those).
