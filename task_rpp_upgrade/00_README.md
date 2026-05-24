# task_rpp_upgrade — RPP Controller Upgrade Work Queue

Self-contained agent tasks to evolve the current Regulated Pure Pursuit (RPP)
controller into a precision tracker that beats Nav2 RPP on cross-track error
for the 3WD marking rover.

## Application context

This rover paints **road markings** and **sports field lines**. Path comes
from a DXF / CSV upload via the frontend → trajectory planner → RPP. The
path IS the line that will be painted on the ground. Implications:

- Waypoints are sacred (no smoothing that deviates from them).
- 90° corners must stay 90° (handled via pivot-turn, not by rounding).
- Heading is as important as position (the marker pen is offset from
  centre of rotation — heading drift = paint drift).
- Speed must be regulated to keep the rover within controller bandwidth.

The trajectory planner upstream is responsible for kinodynamic feasibility
(densifying arcs, inserting pivot waypoints, pen-up/down). RPP trusts the
path it receives.

## Core architecture decision

**RPP is the primary steering controller.** It stays as primary throughout
all upgrades. Stanley is a BLEND supplement for small-xtrack regime only
(< 8 cm). Pivot-turn handles HARD 90° corners. No controller migration.
No rewrite. All upgrades are incremental additions to the existing
`rpp_controller_node.py`.

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

Calibrated for the **road / sports-field marking** application — the input
path IS the painted line, so waypoints are sacred and 90° corners must stay
90°. We achieve sharp corners via a pivot-turn sub-state at flagged HARD
vertices, not by rounding the path.

| Metric | Target | Stretch |
|---|---|---|
| Mean xtrack (straights) | 1-2 cm | < 1 cm |
| Max xtrack (smooth curves, R ≥ 1 m) | 2-3 cm | < 2 cm |
| Max xtrack (HARD 90° corners, post-pivot) | 2-5 cm | < 3 cm |
| RMS xtrack | < 1.5 cm | < 1 cm |
| Goal endpoint precision | < 3 cm | < 2 cm |
| Heading err (steady, on straights) | < 2° | < 1° |
| Heading err (post-pivot exit) | < 3° | < 2° |

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
| 03 | **PathGeometry** (arc-length + κ lookup, NO spline) | **5-7 cm corners** (removes vertex spikes) | GLM |
| 04 | **Path-κ feedforward** | **1-2 cm steady** (reduces heading lag on curves) | Haiku |
| 05 | **Stanley xtrack blend** (small-e regime ONLY, RPP stays primary) | **1-2 cm steady** (eliminates small-xtrack oscillation) | GLM |
| 06 | Precomputed speed profile (a_lat-bounded) | 2-3 cm at corners | GLM |
| 07 | xtrack integral term | bias-elimination, ~0.5-1 cm | Haiku |
| 08 | Goal-approach PI (last 30 cm) | 1-2 cm at endpoint | Haiku |
| 09 | κ low-pass + always-on yaw FF | smoothness, no xtrack but reduces jerk | Gemma |
| 10 | Latency-compensated lookahead | 0.5-1 cm | Gemma |
| 11 | Dynamic speed regulation by `|xtrack|` and `|ψ_e|` | safety, recovery shaping | Gemma |
| 12 | ~~Heading-bias online observer~~ | **DEFERRED** — UM982 dual-antenna provides true heading | — |
| 13 | Benchmark harness vs Nav2 | validation | GLM |
| 14 | **Pivot-turn for HARD 90° corners** | **2-5 cm corners** (stop, spot-turn, resume) | GLM |

~~04 (κ FF)~~ and ~~05 (Stanley blend)~~ are **no longer folded into 03**.
They are separate, incremental tasks that build on the PathGeometry module
from 03. Old task 03 (cubic spline) is **DEPRECATED** — replaced by
polyline + Menger κ (no interpolation between waypoints).

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