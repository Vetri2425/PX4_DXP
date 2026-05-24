# 13 — Benchmark harness vs Nav2 RPP

**Agent:** Haiku 4.5
**Estimated diff:** +250 lines (1 new tool, 1 README)
**Depends on:** 02 (benchmark script), every other task that lands
**Blocks:** —

## Goal

Produce a single document with a head-to-head, numbers-on-the-table
comparison: our controller vs Nav2 RPP, on the same paths, with the same
hardware. Outcome: either we're ahead and we publish, or we're not and we
know exactly which task to revisit.

## Methodology

### A. Paths

Same matrix as `01_architecture_and_upgrade_plan.md` § 6:

1. 2 m square
2. Half circle (Karney, R = 1.5 m)
3. Straight 5 m
4. S-curve (new — create as task 02 prerequisite)

All input as `nav_msgs/Path` topic published once per run. Same path file
fed into both controllers. Both controllers run on the same Jetson with
the same MAVROS / PX4 stack.

### B. Configurations

Run each path 3 times under each configuration, take median to defeat
single-run noise.

| Config | Description |
|---|---|
| **Baseline** | Our controller, all upgrade flags OFF |
| **Ours**     | Our controller, all upgrade flags ON (03-12) |
| **Nav2**     | Nav2's `nav2_regulated_pure_pursuit_controller` plugin, default tuning |
| **Nav2-tuned** | Same plugin, tuned to best-effort for these paths |

Nav2 controller runs via a small bridge node that subscribes to our
`/path`, publishes to our `/rpp/velocity_ned` topic shape. Bridge is in
`tools/nav2_rpp_bridge/` (new package).

### C. Metrics (per run)

| Metric | Definition |
|---|---|
| `xtrack_max_cm` | max |e_⊥| over the run |
| `xtrack_rms_cm` | RMS |e_⊥| over the run |
| `xtrack_straight_mean_cm` | mean |e_⊥| on straight subsegments (κ < 0.1) |
| `xtrack_corner_max_cm` | max |e_⊥| where κ > 1.0 over a 1 m window |
| `endpoint_err_cm` | distance from final pose to last waypoint |
| `time_s` | total mission time (lower = faster, but only valid if xtrack is comparable) |
| `heading_err_rms_deg` | RMS heading error wrt path tangent |
| `jerk_rms` | jerk in commanded yaw rate (proxy for smoothness) |

### D. Deliverable

`tools/benchmark_results.md`:

```
# RPP Controller Benchmark — 2026-XX-XX

| Path | Config | xtrack_max_cm | xtrack_rms_cm | endpoint_err_cm | time_s | notes |
|---|---|---|---|---|---|---|
| square 2m | Baseline | 9.4 | 3.6 | ... | ... | ... |
| square 2m | Ours     | 2.8 | 1.1 | ... | ... | ... |
| square 2m | Nav2     | 8.7 | 3.1 | ... | ... | ... |
...
```

Plus PNG plots: xtrack-vs-arclength overlay for each path × all configs.

## Files to read first

- `tools/benchmark_rpp.py` (task 02).
- Nav2 RPP plugin source:
  https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller

## Scope

### A. Nav2 bridge

A minimal launcher that:

- Loads the Nav2 RPP plugin (`nav2_regulated_pure_pursuit_controller`).
- Subscribes to our `/path` topic, forwards as Nav2 Path.
- Subscribes to `/mavros/local_position/pose`.
- Publishes Nav2 cmd_vel → translates to `/rpp/velocity_ned` shape.
- Disables our `rpp_controller_node` for the duration (param toggle).

### B. Runner

`tools/run_benchmark.sh`:

- Takes a path name and a config name.
- Sets the right params (load YAML per config).
- Starts a rosbag.
- Triggers the mission via FastAPI.
- Waits for DONE.
- Stops the bag.
- Runs `benchmark_rpp.py` on the bag.
- Appends the result row to `tools/benchmark_results.md`.

### C. Plotting

Use matplotlib. One PNG per path, all configs overlaid, xtrack-vs-arclength
on Y, signed.

## Out of scope

- Wind-tunnel-style controlled disturbance injection. Save for v2.
- Cross-platform Nav2 builds (Jetson only).
- Benchmarking against other controllers (TEB, MPPI). Just Nav2 for now.

## Acceptance criteria

- [ ] All 4 paths × 4 configs run end-to-end with the harness.
- [ ] Results table populated.
- [ ] PNG plots committed.
- [ ] **Goal: our controller wins on `xtrack_corner_max_cm` and
      `endpoint_err_cm` by ≥ 2× on at least 3 of the 4 paths vs Nav2.**
      If not, raise issues for the underperforming dimensions and link
      back to specific upgrade tasks for follow-up.

## Notes for the agent

- Nav2 needs a costmap to plan, but for pure controller-only comparison
  you can publish a synthetic empty costmap and feed the path directly
  to the controller. Look at `nav2_controller`'s test fixtures for the
  minimal stand-alone config.
- Run benchmarks on the same day — overnight temperature swings change
  the IMU bias and skew comparisons.
