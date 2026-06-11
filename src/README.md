# RPP Controller Pipeline — Operator Guide

**Version:** 2.0 (FastAPI Server Integrated & Sprint 2 Geometry Upgrades)  
**Last Updated:** 2026-06-06

This directory contains the Phase 2 OFFBOARD path-following pipeline for the 3WD marking rover. It consists of five core runtime nodes, three validation test suites, and one central launch file. The system is designed for ±1–2 cm marking accuracy with PX4 v1.16.2, RoboClaw QPPS closed-loop wheel control, and Holybro UM982 RTK GPS.

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

---

## 📁 Files

### Core ROS2 Nodes

| File | Purpose | Key Topics / Services |
| :--- | :--- | :--- |
| [`rpp_controller_node.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/src/rpp_controller_node.py) | **Regulated Pure Pursuit Controller**:<br>• Tracks path waypoints and translates ENU poses to NED.<br>• Computes lookahead targets, worst-case preview curvatures, and lateral acceleration speed limits.<br>• Publishes relative NED velocity vectors and body yaw-rate feedforward signals. | **Subscribes to**:<br>• `/path` (`nav_msgs/Path`) - target waypoints<br>• `/mavros/local_position/pose` (`geometry_msgs/PoseStamped`) - ENU pose<br>• `/mavros/local_position/velocity_local` (`geometry_msgs/TwistStamped`) - ENU velocity<br>• `/mavros/gpsstatus/gps1/raw` (`mavros_msgs/GPSRAW`) - RTK fix gate<br><br>**Publishes to**:<br>• `/rpp/velocity_ned` (`geometry_msgs/Vector3Stamped`) - velocity setpoint<br>• `/rpp/yaw_rate_body` (`std_msgs/Float32`) - body yaw rate<br>• `/rpp/debug` (`std_msgs/Float32MultiArray`) - 39 performance metrics |
| [`twist_to_setpoint_node.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/src/twist_to_setpoint_node.py) | **MAVROS Heartbeat Translator**:<br>• Subscribes to RPP velocity and body yaw-rate inputs at 50 Hz.<br>• Translates signals to MAVROS ENU coordinates and publishes them as raw setpoints.<br>• Computes explicit ENU yaw from velocity direction and freezes heading below 1 cm/s (P4/P0.5). | **Subscribes to**:<br>• `/rpp/velocity_ned`<br>• `/rpp/yaw_rate_body`<br><br>**Publishes to**:<br>• `/mavros/setpoint_raw/local` (`mavros_msgs/PositionTarget`) |
| [`path_publisher_node.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/src/path_publisher_node.py) | **Test Path Publisher**:<br>• Publishes hardcoded test trajectories (`straight_5m`, `arc_quarter_1m5`, `lshape_2x2`, `square_2x2`, etc.) in the `local_ned` frame. | **Publishes to**:<br>• `/path` (TRANSIENT_LOCAL) |
| [`xtrack_logger_node.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/src/xtrack_logger_node.py) | **Tuning CSV Logger**:<br>• Aggregates time-aligned telemetry (pose, speed, cross-track error, yaw-rates) from various topics at 20 Hz.<br>• Logs values to a CSV file (`/tmp/rpp_<path>_<ts>.csv`) for spreadsheet/pandas analysis. | **Subscribes to**:<br>• `/path`, `/mavros/local_position/pose`, `/rpp/debug`, `/rpp/velocity_ned`, `/mavros/setpoint_raw/local` |
| [`mission_runner_node.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/src/mission_runner_node.py) | **Autonomous Offboard Manager**:<br>• Orchestrates the full automatic mission sequence: pre-streaming -> arming -> switching to OFFBOARD -> waiting for DONE status -> disarming. | **Subscribes to**:<br>• `/mavros/state`, `/rpp/debug`<br><br>**Calls Services**:<br>• `/mavros/cmd/arming`, `/mavros/set_mode` |

### Pipeline Launch

| File | Purpose |
| :--- | :--- |
| [`launch/rpp_pipeline.launch.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/src/launch/rpp_pipeline.launch.py) | **Central Launch Orchestrator**:<br>• Initializes and configures the ROS2 nodes in the correct startup order.<br>• Accepts tuning overrides (`min_lookahead_dist`, `max_linear_vel`, `mission_speed`, etc.) to tweak properties at runtime. |

### Diagnostic & Verification Tests

| File | Purpose |
| :--- | :--- |
| [`test_smoke_rpp_controller.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/src/test_smoke_rpp_controller.py) | **Node Runtime Smoke Test**:<br>• Instantiates the controller, mocks subscriber data, and executes `_control_loop` to ensure execution completes without crashes. |
| [`test_sprint2_geometry.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/src/test_sprint2_geometry.py) | **Geometry Solver Offline Tests**:<br>• Solves linear resampling, corner smoothing arcs, lookahead projections, and Menger curvature math offline without requiring a running ROS2 framework. |
| [`test_p05_yaw_setpoint.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/src/test_p05_yaw_setpoint.py) | **Explicit Yaw Tests**:<br>• Verifies explicit yaw calculation formulas, swaps, and 1 cm/s heading lock freeze thresholds. |
| [`offboard_test.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/src/offboard_test.py) | **Legacy Standalone Test Node**:<br>• A basic pre-Phase-2 offboard publisher kept to verify baseline low-level compatibility. |

---

## 🏃 Running the Pipeline

### SITL Simulation (Gazebo Harmonic)

1. **Terminal 1: Launch PX4 SITL Rover**
   ```bash
   cd ~/PX4-Autopilot
   make px4_sitl gz_r1_rover
   ```

2. **Terminal 2: Launch MAVROS SITL Bridge**
   ```bash
   ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14580
   ```

3. **Terminal 3: Launch RPP Node Pipeline**
   ```bash
   cd ~/PX4_DXP
   ros2 launch src/launch/rpp_pipeline.launch.py path_name:=straight_5m
   ```

4. **Terminal 4: Start the Automated Mission Runner**
   ```bash
   ros2 run --prefix "python3" src/mission_runner_node.py
   ```

### Hardware Deployment

Ensure the MAVROS companion services are active on the Jetson Orin before launching the pipeline:

```bash
# Verify companion bridge state
systemctl status px4-dxp.service

# Launch the pipeline on hardware with auto-run enabled
ros2 launch src/launch/rpp_pipeline.launch.py path_name:=straight_5m auto_run:=true
```

> [!WARNING]
> Only enable `auto_run:=true` on hardware if the rover is positioned in a cleared test area and an operator is holding a physical RC E-stop switch.

### Dry-Run Telemetry Capturing (No Arming)

For safety validation, run a dry-run mission:

```bash
ros2 launch src/launch/rpp_pipeline.launch.py path_name:=arc_quarter_1m5 auto_run:=true dry_run:=true
```

This runs the RPP and translator pipelines without publishing arming commands, allowing log captures of simulated target paths.

---

## 📐 Frame Conventions

* **Local Path**: Described in **LOCAL_NED** (`pose.position.x = North`, `pose.position.y = East`). The frame header must match `"local_ned"`.
* **MAVROS Pose**: Published in **ENU** (`x = East`, `y = North`) per REP-103. The controller swaps these coordinates on read.
* **RPP Velocity Output**: Published to `/rpp/velocity_ned` in **NED** (`x = vN`, `y = vE`).
* **Heartbeat Translator Output**: Published in MAVROS ENU coordinates (`velocity.x = vE`, `velocity.y = vN`, `velocity.z = -vD`).
* **Feedforward Yaw Rate**: Published on `/rpp/yaw_rate_body` in body frame (rad/s). If active and fresh, the type mask is `455` (velocity + explicit yaw + yaw-rate). Otherwise, it sends velocity + explicit yaw under mask `2503` and ignores yaw-rate.

---

## 📈 Parameter Tuning & Diagnostics

### Telemetry Debug Format (`/rpp/debug`)

The node publishes a 47-field diagnostic payload. Indices `0..7` are stable for legacy consumers; newer fields are append-only.

| Index | Field | Description / Units |
| :--- | :--- | :--- |
| `0` | `cross_track_error_signed` | Cross-track deviation in metres (+ = right of path) |
| `1` | `heading_error` | Angle offset between current heading and lookahead (rad) |
| `2` | `lookahead_dist` | Current lookahead horizon distance in metres |
| `3` | `speed_cmd` | Output target speed command in m/s |
| `4` | `curvature_kappa` | Instantaneous steering curvature $\kappa$ ($1/m$) |
| `5` | `dist_to_goal` | Metres remaining to final path waypoint |
| `6` | `pose_age` | Latency age of the pose message (ms) |
| `7` | `state_code` | Status code: `-1` (stale), `0` (idle), `1` (tracking), `2` (approach), `3` (done), `4` (RTK wait), `5` (jump skip) |
| `8` | `l_d_raw` | Target lookahead distance calculated before bounding clamp (m) |
| `9` | `kappa_speed` | Curvature value used to scale velocity limits ($1/m$) |
| `10` | `yaw_rate_cmd` | Computed body-rate yaw command (rad/s) |
| `11..38` | `params` | Snapshot of active controller parameters (for post-run parsing) |
| `39` | `spray_active` | Phase 3 MARK/TRANSIT state: `1.0` = MARK, `0.0` = TRANSIT/OFF |
| `40` | `tracking_profile_code` | `0` auto/unknown, `1` segment, `2` smooth |
| `41..46` | `segment_profile_params` | Segment-profile tuning snapshot: corner threshold, slowdown distance, min corner speed, acceptance radius, heading tolerance, yaw-rate gain |

### Real-Time Parameter Adjustment

ROS2 parameters can be set dynamically during execution:

```bash
# Example: Lower the lookahead floor to 40 cm for tight tracks
ros2 param set /rpp_controller min_lookahead_dist 0.40
```

Tuning guidance is summarized in the table below:

| Symptom | Adjustment |
| :--- | :--- |
| **Straight-line oscillation** | Increase `min_lookahead_dist` or reduce `mission_speed`. |
| **Corner-cutting** | Reduce `a_lat_max`, increase `corner_smooth_radius_m` or `corner_smooth_arc_pts`. |
| **Yaw overshoot at corners** | Decrease `yaw_rate_feedback_gain` (rely on feedforward). |
| **Over-shooting goal endpoint** | Reduce `approach_velocity_scaling_dist` (e.g. from `0.6` to `0.4`). |
| **Stopping short of goal** | Decrease `xy_goal_tolerance` (e.g. from `0.02` to `0.01`). |

---

## 🛡 Safety & Failsafe Guards

* **Pose Loss (Staleness)**: Commands an emergency-stop `(0,0,0)` if no MAVROS pose is received within `pose_max_age_s` (default `0.5 s`) to keep the OFFBOARD stream active without tripping failsafes.
* **Pose Extrapolation (Latency Closure)**: If `use_imu_extrapolation=true` is enabled, predicts pose forward by `v_ned * pose_age` to handle message latency.
* **Input Staleness**: `twist_to_setpoint_node` immediately commands zero velocity if RPP stops publishing for `200 ms`.
* **External Interrupts**: The `mission_runner` immediately releases controller lock and yields to MANUAL mode control if the operator triggers an RC override.
