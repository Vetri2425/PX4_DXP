# RPP Controller — Architecture & Upgrade Plan

This is the master design doc for the upgrade. Every task in this folder
plugs into one of the labelled blocks below. If a task contradicts this
plan, fix the plan first, then the task.

> **2026-05-24 revision 2:** RPP is the PRIMARY steering controller.
> Stanley is a BLEND supplement for the small-xtrack regime only (< 8 cm).
> All upgrades are incremental additions to the existing RPP node — no
> rewrite. Pivot-turn for HARD corners is a separate task (14). Task 12
> (heading-bias observer) is DEFERRED because UM982 dual-antenna provides
> true heading directly. Old task 03 (cubic spline) is DEPRECATED —
> replaced by polyline + Menger κ (no interpolation between waypoints).

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
                          │  yaw_target = α       │  ◄── RPP (Pure Pursuit) IS PRIMARY
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
| Vertex-induced κ spikes at square corners | 5-7 cm | 03 (geometry), 14 (pivot) |
| Pure-pursuit lateral overshoot in turns | 2-3 cm | 04 (κ FF), 05 (Stanley blend) |
| Pipeline latency (pose age, command transport) | 0.5-1 cm | 10 (latency LA) |
| Compass / mount bias | 0.5-1 cm steady | 07 (I-term); 12 (observer, DEFERRED) |
| Final-approach overshoot at goal | 1-2 cm endpoint | 08 (approach PI) |

The first row dominates. Path geometry (task 03) and pivot-turn (task 14)
are the highest-impact upgrades.

## 3. Target pipeline (post-upgrade)

```
                      (raw path from upstream)
                                    │
                                    ▼
                          ┌────────────────────┐
                          │  PathGeometry       │  ◄── upgrade 03
                          │  • polyline (sacred) │      NO interpolation
                          │  • arc-length param  │      replaces P1.3 resample
                          │  • κ(s), ψ_path(s)  │
                          │  • vertex classify   │      HARD → pivot (task 14)
                          └─────────┬──────────┘
                                    │ s, κ(s), ψ_path(s), vertex_type
                                    │
                          ┌─────────▼──────────┐
                          │  Speed profile      │  ◄── upgrade 06
                          │  (offline solve)    │       v(s) s.t.  v² · κ ≤ a_lat_max
                          │                     │              and |dv/ds| ≤ accel_max
                          └─────────┬──────────┘
                                    │ v_ref(s)
                                    │
   /mavros/pose ──ENU→NED─►┌────────▼──────────┐
   /mavros/vel  ──────────►│  Projection +     │  (existing + latency LA  ◄── upgrade 10)
                          │  latency extrap   │      uses PathGeometry.project()
                          └─────────┬──────────┘
                                    │ s, xtrack, ψ_path
                                    │
                          ┌─────────▼──────────┐
                          │  Steering target    │
                          │                     │
                          │  PRIMARY: RPP       │  ◄── existing Pure Pursuit
                          │    α → yaw_target   │      (unchanged for large xtrack)
                          │                     │
                          │  BLEND: Stanley     │  ◄── upgrade 05
                          │    w·δ_stanley +     │      activates at small xtrack
                          │    (1-w)·δ_pp        │      w = exp(-(e/e_blend)²)
                          │                     │
                          │  + Curvature FF     │  ◄── upgrade 04
                          │    κ_cmd += k_ff·κ   │
                          │                     │
                          │  + xtrack I-term    │  ◄── upgrade 07
                          │  + goal-approach PI  │  ◄── upgrade 08
                          └─────────┬──────────┘
                                    │ ψ_cmd
                                    │
                     ┌──────────────┼──────────────┐
                     │  HARD corner ahead?           │  ◄── upgrade 14
                     │  YES → PIVOT_APPROACH then    │
                     │        PIVOT_TURN (v=0,       │
                     │        ψ_cmd=ψ_next_segment)  │
                     │  NO  → continue normal track  │
                     └──────────────┼──────────────┘
                                    │
                          ┌─────────▼──────────┐
                          │  Speed shaping      │  ◄── upgrade 11
                          │  v = v_ref · g(|e|) │
                          │              · g(ψ_e)│
                          └─────────┬──────────┘
                                    │
                                    ▼
                       /rpp/velocity_ned + /rpp/yaw_rate_body
```

## 4. Block contracts (do not break)

- **PathGeometry input**: any `nav_msgs/Path` in LOCAL_NED with ≥ 2
  points and ≥ 5 cm spacing. Output: arc-length-parameterised polyline
  + lookup `κ(s)`, `ψ_path(s)`, `vertex_type(i)`. NO interpolation
  between waypoints — the path stays as the planner gave it.
- **Speed profile**: precomputed at path-receipt time. Stored as a 1D array
  indexed by arc-length sample. No per-cycle re-solve.
- **Projection**: returns `(s, signed_xtrack, foot_n, foot_e, ψ_path)`. The
  s parameter is the global arc length, not a segment index.
- **Steering output**: still a NED velocity vector. The yaw-rate channel is
  the optional body-rate path (P3.1, becomes default after upgrade 09).
- **RPP remains primary**: Stanley blend only activates at small xtrack
  (< e_blend ≈ 8 cm). When `rpp_enable_stanley_blend = False`, output is
  bit-for-bit identical to current RPP.
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
| Straight 5 m | `Test_mission/mission_straight_5m.waypoints` | Reference for steady-state xtrack noise floor |
| S-curve | (create) | Sign-flipping κ — tests bias observer + I-term |

All compared at the same nominal speed (`WP_SPEED` / `cruise_speed` = 0.4 m/s).

## 7. Where each upgrade goes (file map)

| Upgrade | File(s) touched | Lines (est.) |
|---|---|---|
| 03 PathGeometry | `src/rpp_controller_node.py` (path callback), new `src/path_geometry.py` | +250 / -30 |
| 04 κ FF | `src/rpp_controller_node.py` (control loop) | +40 |
| 05 Stanley blend | `src/rpp_controller_node.py` (steering target) | +60 |
| 06 Speed profile | new `src/speed_profile.py` + path callback | +200 |
| 07 xtrack I | `src/rpp_controller_node.py` (control loop) | +25 |
| 08 Goal approach | `src/rpp_controller_node.py` (state machine) | +80 |
| 09 κ LPF + yaw FF default | `src/rpp_controller_node.py` | +15 |
| 10 Latency LA | `src/rpp_controller_node.py` (lookahead block) | +20 |
| 11 Dynamic speed | `src/rpp_controller_node.py` (output stage) | +20 |
| 12 Heading bias obs | DEFERRED — UM982 dual-antenna provides true heading | — |
| 13 Benchmark harness | `tools/benchmark_rpp.py` | +250 |
| 14 Pivot-turn | `src/rpp_controller_node.py` (state machine) | +120 |

Total: ~860 lines added, mostly behind flags.

## 8. What we keep from the existing code

- **Projection with segment hint (P1.4)** — feeds PathGeometry.project() as
  initial guess for Newton iteration.
- **Pose latency extrapolation (P2.4)** — kept verbatim; upgrade 10 stacks on top.
- **RTK fix gate (P0.3)** — non-negotiable safety.
- **Jump detection (P0.2)** — kept.
- **Pure Pursuit steering** — RPP remains the PRIMARY steering law. Stanley
  blend (task 05) is a small-xtrack supplement only.
- **Predictive κ (P1.1)** — partially superseded by PathGeometry (task 03),
  but stays on as a runtime safety belt against path-receipt race conditions.

## 9. Why RPP stays primary

The architecture decision is RPP-first because:

1. **RPP is proven.** Hardware-validated on 2 m square (log 59): 1-3 cm
   xtrack on straights, 9.4 cm peak at corners. It works.
2. **Stanley is weak at re-acquisition.** From large xtrack (> 15 cm),
   Stanley's arctan correction saturates and it struggles to converge.
   RPP's lookahead naturally handles re-acquisition.
3. **Blend is the right hybrid.** Stanley excels at small xtrack (stable,
   zero-noise convergence). RPP excels at large xtrack (fast convergence).
   The Gaussian blend `w = exp(-(e/e_blend)²)` gives the best of both.
4. **The marking application needs both.** Straights need Stanley's tight
   convergence. Corner re-entry needs RPP's lookahead. Neither alone wins.

This is NOT a migration to Stanley. RPP is the primary controller. Stanley
is a targeted supplement for the regime where it's strongest.