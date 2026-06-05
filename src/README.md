# RPP Controller Pipeline — Operator Guide

This directory contains the Phase 2 OFFBOARD path-following pipeline for the
3WD marking rover. Five nodes + one launch file. Designed for ±1-2 cm marking
accuracy with PX4 v1.16.2 + RoboClaw QPPS closed-loop wheel control + UM982 RTK.

```
path_publisher  ─→ /path
                       ↓
         rpp_controller ─→ /rpp/velocity_ned
                        └→ /rpp/yaw_rate_body
                       ↓                    ↘
       twist_to_setpoint                    xtrack_logger ─→ CSV
                       ↓
       /mavros/setpoint_raw/local
                       ↓
              MAVROS2 → PX4 v1.16.2 (DifferentialVelControl)
                       ↓
              RoboClaw (QPPS, closed-loop encoder PID)
                       ↓
                     Motors

         mission_runner ←── /mavros/state, /rpp/debug
                ↓ services
         /mavros/set_mode, /mavros/cmd/arming
```

## Files

| File | Purpose | Topics |
|---|---|---|
| `rpp_controller_node.py` | Regulated Pure Pursuit math; outputs NED velocity vector, diagnostics, and optional yaw-rate feedforward | sub: `/path`, `/mavros/local_position/pose`, `/mavros/local_position/velocity_local`, `/mavros/gpsstatus/gps1/raw`<br>pub: `/rpp/velocity_ned`, `/rpp/yaw_rate_body`, `/rpp/debug` |
| `twist_to_setpoint_node.py` | OFFBOARD heartbeat; bridges to MAVROS at 50 Hz with velocity + explicit yaw, and yaw-rate when fresh | sub: `/rpp/velocity_ned`, `/rpp/yaw_rate_body`<br>pub: `/mavros/setpoint_raw/local` |
| `path_publisher_node.py` | Hardcoded test paths in NED | pub: `/path` (TRANSIENT_LOCAL) |
| `xtrack_logger_node.py` | Time-aligned CSV of every tuning signal | sub: `/path`, `/mavros/local_position/pose`, `/rpp/debug`, `/rpp/velocity_ned`, `/mavros/setpoint_raw/local`<br>output: `/tmp/rpp_<path>_<ts>.csv` |
| `mission_runner_node.py` | Drives OFFBOARD lifecycle (pre-stream → mode → arm → wait DONE → disarm) | sub: `/mavros/state`, `/rpp/debug`<br>srv: `/mavros/set_mode`, `/mavros/cmd/arming` |
| `launch/rpp_pipeline.launch.py` | Brings up everything in the right order | — |
| `offboard_test.py` | Pre-Phase-2 standalone OFFBOARD test (kept for regression) | — |

## Running it

### SITL bring-up (recommended first)

```bash
# Terminal 1: PX4 SITL with differential rover (Gazebo Harmonic)
cd ~/PX4-Autopilot
make px4_sitl gz_r1_rover

# Terminal 2: MAVROS bridge for SITL (or rely on px4-dxp.service if testing on-rover)
ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14580

# Terminal 3: bring up the pipeline (manual mission start)
cd ~/PX4_DXP
ros2 launch src/launch/rpp_pipeline.launch.py path_name:=straight_5m

# Terminal 4: monitor RPP debug
ros2 topic echo /rpp/debug

# Once you see RPP outputting non-zero velocity, run the mission separately:
ros2 run --prefix "python3" src/mission_runner_node.py
```

### Hardware bring-up (after SITL passes)

The Jetson already runs `px4-dxp.service` (MAVROS + NTRIP). Just launch the
pipeline:

```bash
# On Jetson
cd ~/PX4_DXP
ros2 launch src/launch/rpp_pipeline.launch.py \
    path_name:=straight_5m \
    auto_run:=true

# In another shell, watch live xtrack:
ros2 topic echo /rpp/debug
```

`auto_run:=true` is **only** safe on hardware with the rover positioned in a
clear test area and an RC E-stop ready.

### Dry-run telemetry capture (no arming)

```bash
ros2 launch src/launch/rpp_pipeline.launch.py \
    path_name:=arc_quarter_1m5 \
    auto_run:=true \
    dry_run:=true
```

In dry-run, mission_runner walks through every phase but skips `set_mode` and
`arming` calls. Useful for capturing the RPP→twist→MAVROS pipeline without
moving the rover.

## Test paths

| Name | Description | Tests |
|---|---|---|
| `straight_5m` | 5 m straight north, 50 cm point spacing | Cross-track stability on straight |
| `arc_quarter_1m5` | Quarter circle, R=1.5 m, NE turn | Arc tracking, curvature regulation |
| `lshape_2x2` | 2 m N then 2 m E (90° corner) | PX4 spot-turn FSM, corner behaviour |

Goal acceptance: rover within `xy_goal_tolerance` (default 2 cm) of the final
waypoint for `done_settle_s` (default 1 s).

## Tuning entry points

**Current recommended defaults for Phase 2 corner/arc marking**:

- `mission_speed=0.35`
- `min_lookahead_dist=0.52`
- `lookahead_time=1.6`
- `a_lat_max=0.3`
- `preview_curvature_n=4`
- `xtrack_lookahead_gain=0.05`
- `path_resample_spacing_m=0.08`
- `corner_smooth_radius_m=0.5`
- `corner_smooth_arc_pts=6`
- `use_feedforward_yaw_rate=true`
- `yaw_rate_feedback_gain=0.0`
- `max_yaw_rate_body=0.45`

Order matters — change one parameter at a time, capture a CSV, plot, repeat.

| Symptom | Try |
|---|---|
| Wobbles on straight line | Increase `min_lookahead_dist` or reduce `mission_speed` |
| Cuts arcs/corners significantly | Lower `a_lat_max`, increase `corner_smooth_radius_m`, or increase `corner_smooth_arc_pts` |
| Corner yaw overshoots | Lower `yaw_rate_feedback_gain`; current mainline default is pure feedforward (`0.0`) |
| Overshoots goal | Decrease `approach_velocity_scaling_dist` (0.6 → 0.4) |
| Stops short of goal | Decrease `xy_goal_tolerance` (0.02 → 0.01); also check P4 `p4_zero_vel_threshold` |
| Velocity step ringing on hardware | Reduce PX4 `RO_SPEED_P` from 0.5 → 0.2-0.3 (post-QPPS plant is much stiffer) |
| Spot-turn doesn't trigger on L-shape | Reduce PX4 `RD_TRANS_DRV_TRN` toward 25° |
| Spot-turn lingers | Reduce PX4 `RD_TRANS_TRN_DRV` toward 3° |

All RPP-side tunables are ROS2 params on `rpp_controller_node` and adjustable
at runtime:

```bash
ros2 param set /rpp_controller min_lookahead_dist 0.40
```

PX4-side params (`RO_*`, `RD_*`, `PP_*`) are flashed via QGC.

## Frame conventions (avoid the most common bugs)

- All paths are in **LOCAL_NED**: `pose.position.x = North`, `pose.position.y = East`.
- `path.header.frame_id = "local_ned"` is enforced by `rpp_controller_node`.
- MAVROS `/mavros/local_position/pose` is **ENU** per REP-103. The RPP node
  swaps axes on read.
- `/rpp/velocity_ned` is **NED** (`vector.x = vN`, `vector.y = vE`).
- The MAVROS PositionTarget output uses `coordinate_frame = 1` (LOCAL_NED).
- `twist_to_setpoint_node` converts the RPP velocity to MAVROS ENU fields:
  `velocity.x = v_e`, `velocity.y = v_n`, `velocity.z = -v_d`.
- `twist_to_setpoint_node` computes explicit ENU yaw from the velocity
  direction and holds the previous yaw below 1 cm/s.
- `/rpp/yaw_rate_body` is NED/body yaw-rate feedforward. When it is fresh and
  nonzero, `twist_to_setpoint_node` sends type_mask `455`; otherwise it sends
  velocity + yaw with yaw-rate ignored (`2503`).

## Diagnostics

`/rpp/debug` (`std_msgs/Float32MultiArray`) emits 39 floats every control
cycle. Indices `[0..7]` are the stable legacy runtime fields, `[8..10]`
add tuning diagnostics, and `[11..38]` snapshot the active controller params.

| Idx | Field | Units |
|---|---|---|
| 0 | cross_track_error_signed | m (`+` = right of path) |
| 1 | heading_error | rad |
| 2 | lookahead_dist | m |
| 3 | speed_cmd | m/s |
| 4 | curvature κ | 1/m |
| 5 | dist_to_goal | m |
| 6 | pose_age | ms |
| 7 | state_code | -1=STALE, 0=IDLE, 1=TRACKING, 2=APPROACH, 3=DONE, 4=RTK_WAIT, 5=JUMP_SKIP |
| 8 | l_d_raw | m |
| 9 | kappa_speed | 1/m |
| 10 | yaw_rate_cmd | rad/s |
| 11..38 | param snapshot | see `rpp_controller_node.py` header |

`xtrack_logger_node` writes a CSV with the runtime diagnostics plus rover pose,
closest path point, RPP velocity, yaw-rate command, and final MAVROS setpoint.

## Safety & failsafes

- **Pose staleness:** RPP emits `(0, 0, 0)` if pose hasn't arrived in 200 ms.
  Rover will hold position; OFFBOARD stays alive.
- **Pose extrapolation:** `use_imu_extrapolation=true` enables velocity-based
  extrapolation from `/mavros/local_position/velocity_local`. The parameter
  name is legacy; the current code does not integrate IMU acceleration.
- **Input staleness:** twist_to_setpoint emits `(0, 0, 0)` if `/rpp/velocity_ned`
  hasn't arrived in 200 ms. Independent layer of protection.
- **Mission timeout:** `mission_runner` aborts and disarms after 300 s
  (configurable via `mission_timeout_s`).
- **External OFFBOARD exit:** if RC override or failsafe drops OFFBOARD,
  `mission_runner` detects the mode change and exits without disarming
  (operator now has control).
- **Ctrl+C on mission_runner:** disarms and reverts to MANUAL on shutdown.
- **PX4 OFFBOARD failsafe:** if streaming gaps exceed 500 ms (`COM_OF_LOSS_T`),
  PX4 drops OFFBOARD on its own — the streamer's zero-velocity heartbeat
  prevents this.
- **Emergency stop path:** RPP intentionally ignores empty `Path` messages.
  E-stop should publish a single-point path at the current position, then
  switch MANUAL / disarm through the server safety path.

## What's NOT in this pipeline

By design, these are deferred to later phases or out of scope:

- **No Nav2 stack** — single-purpose controller, no costmaps or behavior trees
- **No global planner** — paths come pre-computed (Phase 3: DXF parser)
- **No obstacle avoidance** — marking has no obstacles
- **No Stanley / MPC fallback** — only RPP. Add only if RPP misses ±2 cm after
  RTK validation.
- **No spline smoothing** — RPP handles polylines fine. Phase 3 adds DXF→spline.
- **No reverse path support** — RPP outputs forward velocity only. PX4's P3
  patch handles momentary reverse during spot-turns; not exposed at path level.

## Verification checklist before declaring Phase 2 done

- [ ] All 6 files compile clean (already passing)
- [ ] SITL: straight_5m completes with CSV cross-track < 5 cm RMS
- [ ] SITL: arc_quarter_1m5 completes with CSV cross-track < 5 cm RMS
- [ ] SITL: lshape_2x2 completes; spot-turn visible in `rover_velocity_status`
- [ ] Hardware: RTK fix > 95% of run time (orthogonal to controller — but gates accuracy)
- [ ] Hardware: straight_5m at 0.4 m/s with cross-track < 3 cm RMS on RTK
- [ ] Hardware: arc_quarter_1m5 with cross-track < 3 cm RMS
- [ ] Hardware: `RO_SPEED_P` re-tuned for QPPS plant (no ringing on velocity steps)
- [ ] P3 (reverse) validation: command brief reverse, observe no 180° spin
- [ ] P4 (heading hold) validation: command zero velocity on slope, drift < 1 cm/s
