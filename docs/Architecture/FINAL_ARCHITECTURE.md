# DYX 3WD Marking Rover - Final Architecture

**Version:** 2.0  
**Date:** 2026-06-03  
**Status:** Phase 2 active; baseline stable; corner tuning in progress  
**Reviews incorporated:** ChatGPT, Grok, Claude, Haiku 4.5, GLM engineering review  
**Hardware:** CubeOrangePlus, Jetson Orin, Sabertooth 2x32, UM982 dual-antenna RTK

Deep review history lives in `D:\Vetri\3WD_GCS\architecture\PX4_ROS2\` (12 files).  
This document records the current built stack, the decisions that are still active, and the next tuning loop.

**Current snapshot**
- Phase 1 complete
- Phase 1.5 complete
- Phase 2 OFFBOARD stack built and running
- Path engine v1.0 landed for DXF / CSV / QGC mission planning
- FastAPI control plane and mobile frontend are built
- Next step is to reduce corner cross-track error from the current baseline to <= 5 cm

See the active tuning plan in `docs/superpowers/plans/2026-06-02-corner-xtrack-reduction.md`.

---

## 1. Architecture Decisions

### 1.1 Communication Bridge: MAVROS2

| Option | Verdict | Reason |
|---|---|---|
| uXRCE-DDS | Rejected | reconnect bugs, rover offboard bugs, no native QGC path, no RTK injection path |
| **MAVROS2** | **Selected** | working today, native QGC, RC fallback, NTRIP RTK support, single bridge means fewer race conditions |

Typical latency is 20-35 ms. The current RPP lookahead and speed regulation absorb that delay at marking speeds.

**Current state:** `px4-dxp.service` runs MAVROS2 on `/dev/ttyACM0` at 921600 baud with NTRIP injection and watchdog hardening.

### 1.2 Localization: PX4 EKF2 Primary, Jetson Fusion Deferred

| Option | Verdict |
|---|---|
| **PX4 EKF2 only** | **Current default** - proven enough for the straight-line accuracy target |
| Jetson only | Rejected - CPU spikes make dead-reckoning drift too risky |
| robot_localization fusion | Deferred - add only if RTK gaps or arc error justify it |

```text
UM982 RTK -> PX4 EKF2 (50 Hz) -> /mavros/local_position/pose
CubeOrangePlus IMU -----------^

Jetson robot_localization (20 Hz) <- wheel odometry (AMT102 via STM32 bridge)
                                  <- NHC virtual sensor (future)
                                  -> /odom -> RPP controller
```

**Current state:** RTK injection is running through MAVROS2. Jetson-side fusion is not the default path yet.

### 1.3 IMU: CubeOrangePlus Built-In

| Option | Verdict |
|---|---|
| **CubeOrangePlus ICM42688P** | **Selected** - enough for the current accuracy budget |
| External IMU | Rejected - too much cost for too little gain |

### 1.4 GNSS: UM982 Dual-Antenna RTK

| Option | Verdict |
|---|---|
| Two single-antenna modules | Rejected - more wiring, more failure points |
| **UM982 dual-antenna** | **Selected** - compass-free heading with one module |

**Current state:** UM982 is connected on `/dev/ttyUSB0`. NTRIP corrections are injected into `/mavros/gps_rtk/send_rtcm`.

### 1.5 Path Tracking: RPP Primary, MPC Deferred

| Option | Verdict |
|---|---|
| Pure RPP | **Selected** - current active path follower |
| Pure MPC | Deferred - not needed for the current field target |
| RPP + MPC hybrid | Deferred - keep it as a fallback idea, not the mainline architecture |

The current controller is a Jetson-side Regulated Pure Pursuit stack with:
- velocity-vector OFFBOARD output
- predictive curvature regulation
- adaptive lookahead
- path conditioning for DXF / CSV / QGC input
- RTK fix gating
- EKF jump detection
- optional yaw-rate feedforward

**Current state:** This is the built and running Phase 2 path-following pipeline.

### 1.6 Spray Control: Jetson GPIO Direct

| Option | Verdict |
|---|---|
| uXRCE-DDS aux command | Rejected - latency and dependency risk |
| MAVROS2 relay | Rejected - slower than direct hardware control |
| **Jetson GPIO -> transistor -> relay** | **Selected** - simplest and fastest path |

**Current state:** not yet built. Still the right architecture for Phase 3 because the hardware path stays independent of the flight controller.

### 1.7 Plan B: Jetson Direct Motor Control

If PX4 offboard ever stops being reliable, the fallback is direct Jetson motor control without GPL contamination.

```text
Jetson ROS2
-> path tracking
-> motor mixing
-> PCA9685
-> Sabertooth 2x32
```

This is a fallback only. It is not the primary plan.

### 1.8 NHC: Non-Holonomic Constraint

The NHC publisher is still a useful future upgrade:
- lateral velocity forced to zero
- vertical velocity forced to zero
- helps dead reckoning during GPS gaps

**Current state:** deferred until localization fusion is actually needed.

---

## 2. Current System Architecture

```text
OPERATOR SIDE
  React Native frontend
    -> FastAPI backend
      -> ROS2 services / telemetry / mission control

JETSON ORIN
  path_engine
    -> path_publisher_node
    -> mission_runner_node
    -> rpp_controller_node
    -> twist_to_setpoint_node
    -> MAVROS2
    -> PX4 v1.16.2
    -> Sabertooth 2x32
    -> motors

  xtrack_logger_node
    -> CSV logs for tuning

  ntrip_rtcm_node
    -> MAVROS2 RTK injection
```

### What is built right now
- `path_engine` handles DXF / CSV / QGC mission planning
- `path_publisher_node` publishes the path into ROS2
- `rpp_controller_node` computes the velocity vector and diagnostics
- `twist_to_setpoint_node` keeps OFFBOARD alive and forwards the command to MAVROS2
- `mission_runner_node` owns arm / mode / finish state handling
- `xtrack_logger_node` captures tuning data
- FastAPI exposes mission, path, params, telemetry, and emergency-stop control
- The React Native frontend sits on top of the FastAPI control plane

---

## 3. Current Control State

### PX4 baseline parameters

These remain the stable rover-side values that keep AUTO and rover control sane:

```text
RO_YAW_RATE_P = 0.5
RO_YAW_RATE_I = 0.3
RO_YAW_RATE_LIM = 30.0
RO_SPEED_P = 0.5
RO_SPEED_I = 0.1
RO_YAW_P = 1.0
RO_MAX_THR_SPEED = 1.5
NAV_ACC_RAD = 0.1
MIS_YAW_ERR = 25.0
RD_TRANS_DRV_TRN = 0.52
RD_TRANS_TRN_DRV = 0.0873
RD_WHEEL_TRACK = 0.47
RD_TANK_MODE = 1
GPS_YAW_OFFSET = 180.0
CA_R_REV = 3
```

These are the FCU-side guardrails. The interesting tuning now mostly lives in ROS2.

### RPP / ROS2 defaults

These are the current controller-side defaults in `src/rpp_controller_node.py` and the launch file:

| Param | Value | Notes |
|---|---|---|
| `max_linear_vel` | `0.8` | hardware ceiling |
| `mission_speed` | `0.35` | operator-facing job speed |
| `min_linear_vel` | `0.15` | floor |
| `min_lookahead_dist` | `0.52` | current arc-safe floor |
| `max_lookahead_dist` | `1.0` | upper clamp |
| `lookahead_time` | `1.6` | velocity scaling |
| `a_lat_max` | `0.3` | lateral accel limit |
| `preview_curvature_n` | `4` | predictive curvature window |
| `xtrack_lookahead_gain` | `0.05` | xtrack-adaptive lookahead |
| `path_resample_spacing_m` | `0.08` | path densification |
| `corner_smooth_radius_m` | `0.5` | corner smoothing |
| `corner_smooth_arc_pts` | `6` | corner discretisation |
| `use_imu_extrapolation` | `false` | off by default |
| `use_feedforward_yaw_rate` | `true` | enabled |
| `yaw_rate_feedback_gain` | `0.0` | current mainline default |
| `max_yaw_rate_body` | `0.45` | clamp |
| `max_linear_accel` | `0.35` | ramp-up control |
| `max_linear_decel` | `0.5` | braking derivation |

### Latest tuning direction

The latest commit tightened the arc defaults, and the active tuning loop is now:
- lower corner xtrack
- keep straight-line accuracy unchanged
- only add fused odom if the field data proves we need it

---

## 4. Hardware Bill of Materials

| Component | Model | Status | Cost (INR) |
|---|---|---|---|
| Autopilot | CubeOrangePlus | Owned, running | - |
| Motor driver | Sabertooth 2x32 | Owned, running | - |
| Companion | Jetson Orin | Owned, running | - |
| GNSS | Holybro UM982 | Owned, running | - |
| Wheel encoders | AMT102 (1024 CPR) x 2 | Needed for fusion | 6,000-10,000 |
| Encoder bridge | STM32F103 | Needed for fusion | 500 |
| Spray driver | 2N2222 + relay + diode | Needed for Phase 3 | 200 |
| E-stop | NC relay + button | Needed for Phase 3 | 300 |
| Plan B servo driver | PCA9685 | Fallback only | 500-800 |

**Total new hardware:** about INR 7,000-12,000

---

## 5. Software Stack & Licensing

| Layer | Component | License | Status |
|---|---|---|---|
| Core framework | ROS2 Humble | Apache 2.0 | Running |
| Bridge | MAVROS2 | BSD | Running |
| Motor control | PX4 v1.16.2 | BSD | Running |
| Path planning | path_engine | MIT / project code | Running |
| Path tracking | RPP controller stack | project code | Running |
| Backend API | FastAPI server | MIT / project code | Running |
| Frontend | React Native dashboard | MIT / project code | Running |
| Localization fusion | robot_localization | BSD | Deferred |
| Spray control | GPIO relay path | project code | Deferred |
| NHC publisher | virtual sensor | project code | Deferred |
| Encoder bridge | STM32F103 firmware | MIT | Deferred |

**No GPL-3 code in the active path.**

---

## 6. What We Have Achieved

- Phase 1 PX4 baseline is complete and AUTO mode works on straight missions
- Phase 1.5 hardening is complete: NTRIP watchdog, service hardening, deployment workflow, and runtime hygiene are in place
- MAVROS2 is confirmed as the single bridge for the active architecture
- The OFFBOARD stack is built end to end
- `path_engine` v1.0 is landed and handles real mission inputs
- The Jetson-side RPP controller is built with predictive curvature, adaptive lookahead, and diagnostics
- The mission runner, setpoint streamer, logger, and path publisher are all in place
- FastAPI control plane is built for mission and telemetry operations
- The frontend is built on top of that API
- The architecture no longer depends on a speculative rewrite; it now has a working mainline

The most recent work is focused on tuning, not re-architecting:
- expand observability
- reduce corner xtrack
- preserve straight-line stability
- keep RTK and OFFBOARD reliable

---

## 7. Next Plan

This is the practical next sequence, aligned with the active tuning plan:

### Step 1 - Establish the baseline on the current branch
- run the square_2x2 path
- record `/rpp/debug`, pose, and state
- compare against the 9.4 cm corner baseline

### Step 2 - Tune the main corner knobs
- sweep `yaw_rate_feedback_gain`
- if needed reduce `a_lat_max`
- if needed increase `corner_smooth_arc_pts`

### Step 3 - Decide whether fusion is actually needed
- if corner xtrack still misses the target, enable `robot_localization`
- otherwise keep the stack simpler and stay on PX4 EKF2 only

### Step 4 - Move to field mission integration
- connect DXF mission input through `path_engine`
- validate spray timing
- validate E-stop behavior
- run a full draw mission end to end

### Step 5 - Production hardening
- run longer-duration field tests
- validate failover paths
- finalize customer-facing docs once the tuning numbers stop moving

---

## 8. Review Decision Record

| Topic | Final |
|---|---|
| Bridge | MAVROS2 |
| Localization | PX4 EKF2 primary |
| IMU | CubeOrangePlus built-in |
| GNSS | UM982 dual-antenna RTK |
| Path tracking | Jetson-side RPP |
| Spray | Jetson GPIO relay |
| MPC | Deferred |
| Plan B | Jetson direct motor control |
| NHC | Deferred |

