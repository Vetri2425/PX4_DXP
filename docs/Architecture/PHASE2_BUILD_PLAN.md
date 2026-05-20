# Phase 2 Build Plan — OFFBOARD Path Following

**Version:** 1.0
**Date:** 2026-05-20
**Stack:** PX4 v1.16.2 EKF2 + MAVROS2 + ROS2 Humble
**Bridge:** MAVROS2 only (DDS shelved — see MAVROS2_ONLY_DECISION.md)

---

## Target System Diagram (Final Build)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          JETSON ORIN (192.168.1.102)                      │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │ ROS2 Humble                                                     │    │
│  │                                                                 │    │
│  │  ┌──────────────┐    ┌───────────────┐    ┌──────────────────┐  │    │
│  │  │ DXF Parser   │───>│ Path Planner  │───>│ RPP Controller   │  │    │
│  │  │ (laptop)     │    │ (arc+straight) │    │ (pure pursuit)    │  │    │
│  │  └──────────────┘    └───────────────┘    └────────┬─────────┘  │    │
│  │                                                      │          │    │
│  │                                    /mavros/setpoint_raw/local    │    │
│  │                                    (PositionTarget @ 50Hz)      │    │
│  │                                                      │          │    │
│  │  ┌──────────────┐    ┌───────────────┐               │          │    │
│  │  │ Spray Node   │    │ NTRIP Node    │               │          │    │
│  │  │ (GPIO→relay) │    │ (RTCM inject) │               │          │    │
│  │  └──────────────┘    └───────────────┘               │          │    │
│  └──────────────────────────────────────────────────────┼──────────┘    │
│                                                         │              │
│  MAVROS2 Node ──────────────────────────────────────────┤              │
│  ├─ /mavros/local_position/pose    (EKF2 position IN)   │              │
│  ├─ /mavros/global_position/pose   (GPS position IN)    │              │
│  ├─ /mavros/state                   (mode/armed IN)      │              │
│  ├─ /mavros/setpoint_raw/local      (setpoint OUT)      │              │
│  ├─ /mavros/cmd/arming              (arm service)        │              │
│  └─ /mavros/set_mode               (OFFBOARD service)    │              │
│                                                         │              │
│  /dev/ttyACM0 ◄──────── USB ─────────────────────────────┤              │
└──────────────────────────────────────────────────────────┼──────────────┘
                                                           │
                    USB (921600 baud)                        │
                                                           ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                       CUBEORANGEPLUS (PX4 v1.16.2)                        │
│                                                                          │
│  ┌─────────┐    ┌──────────┐    ┌────────────────────┐    ┌──────────┐ │
│  │ UM982    │───>│ EKF2     │───>│ DifferentialPosCtrl │───>│ Motor    │ │
│  │ RTK DA   │    │ (50Hz)   │    │ + DifferentialVelC  │    │ Mixing   │ │
│  │ (TELEM1) │    │          │    │ (P3+P4 patched)     │    │ + IK     │ │
│  └─────────┘    └──────────┘    └────────────────────┘    └────┬─────┘ │
│       ↑                ↑                                         │       │
│  IMU (3×ICM42688P)    NTRIP RTCM                                 │       │
│  (internal)           (via Jetson)                               │       │
│                                                                   ▼       │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │ Sabertooth 2x32 ── Left Motor ── Right Motor ── Front Caster   │    │
│  └─────────────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────────────┘

Localization stack (CURRENT — sufficient for ±3cm):
  UM982 RTK (10Hz) + CubeOrangePlus IMU (50Hz) → PX4 EKF2 → /mavros/local_position/pose

Future optimization (NOT needed now):
  AMT102 encoders → STM32F103 bridge → Jetson robot_localization ← NHC
  (Adds: GPS-outage resilience, 5-15% arc improvement)
```

---

## Session Build Plan

### Session 1 — Flash Patched Firmware + Set Params
**Where:** Laptop (QGC) + physical rover
**Duration:** ~30 min
**Milestone:** Patched firmware running, AUTO still works

| Step | Action | Verify |
|---|---|---|
| 1 | Download CI artifact from https://github.com/Vetri2425/PX4-Autopilot/actions | `.px4` file in hand |
| 2 | Flash to CubeOrangePlus via QGC | QGC shows correct firmware version |
| 3 | Set 13 FCU params (table below) | `param show` confirms each |
| 4 | Run AUTO straight-line mission | Same 1.2-2.1cm xtrack as before |
| 5 | Verify RD_TANK_MODE=0 works for manual | Rover responds to RC sticks |

**Param table (set ALL before any OFFBOARD test):**

| # | Param | Value | Reason |
|---|---|---|---|
| 1 | COM_OBL_RC_ACT | 5 | Hold mode = safe auto-stop |
| 2 | COM_OF_LOSS_T | 0.2 | 8cm max coast @ 0.4 m/s |
| 3 | COM_RCL_EXCEPT | 4 | Allow OFFBOARD without RC |
| 4 | RD_TANK_MODE | 0 | OFFBOARD mode (not tank) |
| 5 | RO_SPEED_LIM | 0.5 | 25% headroom over nominal |
| 6 | RO_MAX_THR_SPEED | 0.5 | Must match RO_SPEED_LIM |
| 7 | RO_ACCEL_LIM | 0.4 | 1s to cruise |
| 8 | RO_DECEL_LIM | 0.8 | 0.5s stop |
| 9 | RO_JERK_LIM | 5.0 | Smooth profile |
| 10 | PP_LOOKAHD_MIN | 0.1 | Min pure-pursuit lookahead |
| 11 | PP_LOOKAHD_MAX | 1.0 | Max pure-pursuit lookahead |
| 12 | PP_LOOKAHD_GAIN | 0.5 | Lookahead = 0.5 × speed |
| 13 | NAV_ACC_RAD | 0.10 | WP switch radius |

---

### Session 2 — OFFBOARD Hello World
**Where:** Jetson
**Duration:** ~2 hours
**Milestone:** Rover moves 1m forward via OFFBOARD, stops cleanly

| Step | Action | Verify |
|---|---|---|
| 1 | Create `~/PX4_DXP/src/offboard_test.py` | File exists |
| 2 | Stream position setpoints 50Hz for 0.5s | PX4 accepts stream |
| 3 | Call arm + set_mode OFFBOARD | `/mavros/state` shows OFFBOARD |
| 4 | Publish forward setpoint for 2s | **Wheels off ground (bench test)** |
| 5 | Publish zero-velocity setpoint | P4 fix: no North-snap rotation |
| 6 | Disarm | Clean stop |
| 7 | Capture rosbag | Record state + pose + setpoint |

**Node structure:**
```
offboard_test.py
  ├─ Publisher:  /mavros/setpoint_raw/local  (PositionTarget, 50Hz)
  ├─ Service:    /mavros/cmd/arming           (CommandBool)
  ├─ Service:    /mavros/set_mode             (SetMode, "OFFBOARD")
  └─ Subscriber: /mavros/state               (monitor mode/armed)
```

---

### Session 3 — Velocity OFFBOARD Test
**Where:** Jetson
**Duration:** ~2 hours
**Milestone:** Forward, reverse, stop all work correctly

| Step | Action | Verify |
|---|---|---|
| 1 | Write `velocity_test.py` using `/mavros/setpoint_velocity/cmd_vel` | File exists |
| 2 | Test forward (linear.x = +0.3) | Rover moves forward |
| 3 | Test reverse (linear.x = -0.3) | **P3 validation: backward motion, no 180° spin** |
| 4 | Test stop (linear.x = 0) | **P4 validation: heading holds, no North-snap** |
| 5 | Measure MAVROS2 latency | `ros2 topic hz` setpoint vs pose |
| 6 | On ground test | Wheels on ground, rover moves |

---

### Session 4 — Straight-Line Path Follower
**Where:** Jetson
**Duration:** ~3 hours
**Milestone:** Rover follows 5m straight line via OFFBOARD position setpoints

| Step | Action | Verify |
|---|---|---|
| 1 | Write `path_follower_node.py` | File exists |
| 2 | Hardcode 5m straight-line NED waypoints (1m spacing) | Path loaded |
| 3 | Stream position setpoints, advance on NAV_ACC_RAD | Rover follows line |
| 4 | Measure cross-track error from rosbag | Compare vs AUTO 1.2cm baseline |
| 5 | Test at 0.4 m/s | Accuracy within ±3cm |

**Node structure:**
```
path_follower_node.py
  ├─ Publisher:   /mavros/setpoint_raw/local  (PositionTarget, 50Hz)
  ├─ Subscriber: /mavros/local_position/pose  (current position)
  ├─ Subscriber: /mavros/state                 (mode/armed monitor)
  ├─ Service:    /mavros/cmd/arming
  ├─ Service:    /mavros/set_mode
  └─ Param:      waypoints (NED list), speed, advance_radius
```

**Position-only setpoint (type_mask):**
```python
IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW | IGNORE_YAW_RATE
# = 4039 (bits: only PX and PY set)
```

---

### Session 5 — Arc Path Follower
**Where:** Jetson
**Duration:** ~4 hours
**Milestone:** Rover follows 1.5m radius arc with <3cm cross-track error

| Step | Action | Verify |
|---|---|---|
| 1 | Write `arc_generator.py` — NED arc waypoints from center+radius+angles | Dense waypoints at 5cm spacing |
| 2 | Feed arc waypoints into path_follower_node | Rover follows arc |
| 3 | Test 60°, 90°, 180° arcs at R=1.5m | Shape accuracy |
| 4 | Capture rosbag, compute cross-track | RMS < 3cm |
| 5 | Compare with AUTO densified-WP method | OFFBOARD should be smoother |

**Arc generation (Karney geodesic, reuse from mission_generator.py):**
```
Input:  center_lat, center_lon, radius_m, start_angle, end_angle, density=0.05m
Output: list of (north, east) in NED frame relative to home
```

---

### Session 6 — RPP Controller (Jetson-Side Pure Pursuit)
**Where:** Jetson
**Duration:** ~4 hours
**Milestone:** Velocity-mode pure pursuit with smoother arc tracking

| Step | Action | Verify |
|---|---|---|
| 1 | Write `rpp_controller_node.py` | File exists |
| 2 | Subscribe to `/mavros/local_position/pose` | Current position at 50Hz |
| 3 | Compute lookahead point on reference path | Pure pursuit math |
| 4 | Publish velocity setpoint (speed + bearing) | `/mavros/setpoint_velocity/cmd_vel` |
| 5 | Test same arcs as Session 5 | Compare accuracy vs position-mode |
| 6 | Tune PP gains | PP_LOOKAHD_GAIN, MIN, MAX |

**Why RPP on Jetson vs PX4's built-in pure pursuit:**
- PX4 pure pursuit only works in position-mode (setpoint_raw/local)
- Velocity-mode gives direct speed+heading control → lower latency
- Jetson RPP can switch between position and velocity mode per segment
- Better for production: Jetson owns the path intelligence

---

### Session 7 — DXF → Path Pipeline
**Where:** Laptop generates, Jetson executes
**Duration:** ~4 hours
**Milestone:** Load a DXF file, rover draws the shape

| Step | Action | Verify |
|---|---|---|
| 1 | Enhance `mission_generator.py` to output NED waypoint list | JSON or CSV format |
| 2 | Write `mission_loader_node.py` on Jetson | Reads NED path file |
| 3 | `mission_loader` → `rpp_controller` → OFFBOARD | Full pipeline |
| 4 | scp NED path from laptop to Jetson | File transfer works |
| 5 | Test: DXF with straight + arc + straight | Full shape drawn |

---

### Session 8 — Spray Integration
**Where:** Jetson
**Duration:** ~3 hours
**Milestone:** Spray triggers on path, stops on transition

| Step | Action | Verify |
|---|---|---|
| 1 | Wire Jetson GPIO → 2N2222 → relay → solenoid | Circuit complete |
| 2 | Write `spray_control_node.py` | GPIO on/off via ROS2 topic |
| 3 | Tie spray to path segments (spray on-line, off on transition) | Synchronized |
| 4 | Lag compensation: trigger early by (lag_ms × speed) | Even marks |
| 5 | E-stop NC relay test | Physical button kills spray |

---

### Session 9 — Full Drawing Mission (Field Test)
**Where:** Field
**Duration:** ~4 hours
**Milestone:** End-to-end DXF → path → drive → spray → mark on ground

| Step | Action | Verify |
|---|---|---|
| 1 | Load production DXF (parking lot lines or similar) | Realistic test |
| 2 | Full autonomous run: OFFBOARD → drive → spray | Marks appear on ground |
| 3 | Measure marking accuracy vs plan | Tape measure or survey |
| 4 | Rosbag full capture | Complete log for analysis |
| 5 | Test E-stop during mission | Safe shutdown works |

---

### Session 10 — Production Hardening
**Where:** Jetson
**Duration:** ~4 hours
**Milestone:** System survives failures gracefully

| Step | Action | Verify |
|---|---|---|
| 1 | Watchdog node: kill OFFBOARD if FCU heartbeat >500ms lost | Auto-fallback to Hold mode |
| 2 | Graceful shutdown on ROS2 node exit | Publish zero velocity, disarm |
| 3 | Systemd integration: OFFBOARD node in px4-dxp.service | Survives reboot |
| 4 | Log rotation for rosbag | Don't fill disk |
| 5 | NHC virtual sensor (lateral_vel=0 publisher) | Optional EKF improvement |

---

## Localization: Why EKF2 Alone Is Sufficient

**Current (Phase 2):**
```
UM982 RTK (10Hz) + CubeOrangePlus IMU (50Hz) → PX4 EKF2 → /mavros/local_position/pose
```

**Proven accuracy:** 1.2-2.1cm cross-track on straight lines (AUTO mode tests, May 13-19).
**Error budget:** RTK ±1.5cm (dominant) + gyro drift ±2.6cm = ±3.0cm RSS. Within 3cm budget.

**When to add Jetson fusion (robot_localization):**
- GPS dropouts in your test field (bridges, buildings)
- Cross-track error >3cm on arcs that EKF2 can't handle
- Need sub-centimeter accuracy

**Future fusion stack (Phase 3-4, only if needed):**
```
UM982 RTK ────────────→ PX4 EKF2 ────────→ MAVROS2 ──┐
AMT102 encoders → STM32F103 → Jetson UART ─────────→ robot_localization ←──┤ → /odom → RPP
NHC virtual sensor (50 lines Python, lateral_vel=0) ─────────────────────┘
```

---

## What NOT to Build (Deferred)

| Item | Why Deferred | Revisit When |
|---|---|---|
| Jetson robot_localization | EKF2 alone hits ±3cm target | GPS dropouts observed in field |
| AMT102 encoder STM32 bridge | No accuracy gap to fill yet | EKF2 drift on arcs >3cm |
| MPC controller | RPP sufficient for >0.33m radius arcs | Tight corners <0.33m radius needed |
| DDS/uXRCE-DDS | 4+ blocking bugs, MAVROS2 sufficient | MAVROS2 fails latency test (>40ms) |
| P1 PosControl patch | Stock v1.16.2 PosControl gates output behind flag_armed (free disarm guard) | Stale setpoint after OFFBOARD exit observed in testing |