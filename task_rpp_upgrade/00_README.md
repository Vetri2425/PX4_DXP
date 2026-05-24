# task_rpp_upgrade — RPP Controller Upgrade Work Queue

Self-contained agent tasks to evolve the current Regulated Pure Pursuit (RPP)
controller into a precision tracker that beats Nav2 RPP on cross-track error
for the 3WD marking rover.

## Baseline (2026-05-23, log 59)

2 m × 2 m square mission, RPP params from `Final_Best`:

| Metric | Value | Where |
|---|---|---|
| Max xtrack | **9.4 cm** | corner overshoot |
| Mean xtrack (straights) | 1-3 cm | between corners |
| RMS xtrack | 3.6 cm | full lap |
| Yaw mean err | 10.7° (before fix) → ~3° (after `fd91d0c`) | turns |
| Mission completion | 100 % | — |

## Targets (post-upgrade)

| Metric | Target | Stretch |
|---|---|---|
| Max xtrack (corners) | < 3 cm | < 2 cm |
| Mean xtrack (straights) | < 1 cm | < 0.5 cm |
| RMS xtrack | < 1.5 cm | < 1 cm |
| Goal endpoint precision | < 2 cm | < 1 cm |
| Yaw err (steady) | < 2° | < 1° |

## How we beat Nav2 RPP

Nav2's RPP is a strong open-source baseline but has fixed limitations:
- **Reactive curvature regulation only.** It scales speed by *current*
  curvature at the lookahead point — no preview.
- **No path smoothing.** Inherits the input polyline's vertex-induced
  curvature spikes verbatim.
- **No latency compensation.** Assumes zero pipeline delay.
- **No xtrack integral term.** Steady bias from compass / mounting offset
  becomes a steady xtrack offset.
- **No goal-approach controller.** Pure Pursuit famously overshoots the
  final waypoint.

Our upgrades hit each of those, in order of expected impact:

| # | Upgrade | Expected xtrack reduction | Agent |
|---|---|---|---|
| 03 | Cubic-spline path smoothing | **5-7 cm at corners** | GLM |
| 04 | Path-κ feedforward | 2-3 cm transient | Haiku |
| 05 | Stanley xtrack blend (small-e regime) | 1-2 cm steady | GLM |
| 06 | Precomputed speed profile (a_lat-bounded) | 2-3 cm at corners | GLM |
| 07 | xtrack integral term | bias-elimination, ~0.5-1 cm | Haiku |
| 08 | Goal-approach PI (last 30 cm) | 1-2 cm at endpoint | Haiku |
| 09 | κ low-pass + always-on yaw FF | smoothness, no xtrack but reduces jerk | Gemma |
| 10 | Latency-compensated lookahead | 0.5-1 cm | Gemma |
| 11 | Dynamic speed regulation by `|xtrack|` | safety, recovery shaping | Gemma |
| 12 | Heading-bias online observer | bias-elimination over minutes | GLM |

A `13_benchmark_harness_vs_nav2.md` task closes the loop: head-to-head
quantitative comparison on the same paths.

Read `01_architecture_and_upgrade_plan.md` next — it explains the controller
block diagram and where each upgrade plugs in.

## How to use this folder

Same conventions as `task_frontend/00_README.md`:

1. Pick a task `02_*` and up.
2. Read the agent recommendation and the files listed under `Files to read
   first`.
3. Implement only what's in `Scope`. Anything in `Out of scope` is for
   another task.
4. **Always capture a baseline first (task 02) before attempting xtrack
   improvements.** You cannot prove a delta without a before-number.
5. Update params + test files together — never code-only.
6. Move done files to `task_rpp_upgrade/done/`.

## Agent capability legend

Same as `task_frontend/`. RPP code is in Python (rclpy); GLM handles ROS2 and
multi-method controller changes, Haiku handles bounded single-method
features, Gemma handles single-line parameter / formula edits.

## Hard rules for every task

- **Default-OFF for every new feature.** Keep a parameter that flips it on.
  We need to be able to A/B compare on hardware, and bisect regressions.
- **Never break the baseline path.** Existing 2 m square must still
  complete with the new code in "all flags off" mode.
- **Every upgrade ships with a unit test.** Pure math (no ROS) where
  possible. The test file lives next to the source: `tests/test_<feature>.py`.
- **No tuning by feel on hardware.** Use the test paths in
  `Test_mission/`. Capture rosbags. Compute xtrack offline from the bag.
- **Cite the source.** If you reference Nav2, Stanley, MPC, or a paper, add
  a one-line comment with the URL or DOI so the next reader can verify.
- **Do not edit `twist_to_setpoint_node.py`** unless the task explicitly
  says so. That node owns the OFFBOARD heartbeat and was hardware-validated
  on 05-23.
- **Frame discipline:** `rpp_controller_node` works in NED internally and
  outputs NED velocity. ENU lives only at the MAVROS boundary
  (`twist_to_setpoint_node`). Never mix.
