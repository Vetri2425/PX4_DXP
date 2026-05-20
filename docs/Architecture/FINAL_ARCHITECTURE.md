# DYX 3WD Marking Rover — Final Architecture

**Version:** 1.0  
**Date:** 2026-05-20  
**Status:** Phase 1.5 complete, Phase 2 next  
**Reviews incorporated:** ChatGPT, Grok, Claude, Haiku 4.5, GLM engineering review  
**Hardware:** CubeOrangePlus, Jetson Orin, Sabertooth 2x32, UM982 dual-antenna RTK  

Deep review history lives in `D:\Vetri\3WD_GCS\architecture\PX4_ROS2\` (12 files).  
This document records **final decisions and current built state only.**

---

## 1. Architecture Decisions

### 1.1 — Communication Bridge: MAVROS2

| Option | Verdict | Reason |
|---|---|---|
| uXRCE-DDS | Rejected | 2-8% frame drops under thermal stress, no auto-reconnect after agent restart, no native QGC support |
| **MAVROS2** | **Selected** | <1% frame drop, native QGC, proven on ground vehicles, RC failsafe fallback |

Typical latency: 20-35ms. Worst-case: 80ms under CPU spike. Decision gate: if >40ms consistently, evaluate uXRCE-DDS with workarounds.

Fallback: RC override → PX4 RTL. GPIO pulse from Jetson triggers RC failsafe.

**Current state:** Running as `px4-dxp.service`. `/dev/ttyACM0` @ 921600 baud, GCS `udp-b://:14550@`. Production-hardened with watchdogs.

### 1.2 — Localization: PX4 EKF2 Primary, Jetson Fusion Deferred

| Option | Verdict |
|---|---|
| **PX4 EKF2 only** | **Phase 2 default** — proven ±1.2-2.1cm xtrack, hits ±3cm budget |
| Jetson only | Rejected — CPU spikes cause dead-reckoning divergence |
| Dual-layer (robot_localization) | Deferred to Phase 3-4 — add only if GPS dropouts or arc error >3cm |

```
UM982 RTK (dual-antenna) → PX4 EKF2 (50Hz) → /mavros/global_position
CubeOrangePlus IMU ───────→ PX4 EKF2 ─────────→ /mavros/imu/data
                                                    ↓
Jetson robot_localization (20Hz) ←──────────────────┘
    ↑ wheel odometry (AMT102 via STM32 bridge)
    ↑ NHC virtual sensor (lateral_vel=0, vertical_vel=0)
    ↓
/odom (smooth, 20Hz) → RPP/MPC controller
```

Graceful degradation: if Jetson EKF fails, PX4 position still works (less smooth). If PX4 EKF fails, Jetson degrades to dead-reckoning (~30s).

**Current state:** PX4 EKF2 running with RTK injection via NTRIP node. Jetson robot_localization NOT yet built (deferred — EKF2 alone hits ±3cm target).

### 1.3 — IMU: CubeOrangePlus Built-In

| Option | Verdict |
|---|---|
| **CubeOrangePlus ICM42688P** | **Selected** — vibration-isolated, temperature-controlled, sufficient for ±3cm |
| External IMU (LORD 3DM-GX5-45) | Rejected — ₹3-5L cost for ±0.7cm improvement |

Error budget: RTK ±1.5cm (dominant) + gyro drift ±2.6cm + wheel encoder ±1.0cm = ±3.3cm RSS. Upgrading IMU improves to ±2.1cm for ₹3-5L. Not worth it.

### 1.4 — GNSS: UM982 Dual-Antenna RTK

| Option | Verdict |
|---|---|
| Two HERE3+ modules | Rejected — two modules, wiring complexity, ₹60K |
| **UM982 dual-antenna** | **Selected** — one module, two antennas, compass-free heading, ₹25-35K |

PX4 params: `EKF2_GPS_CTRL = 8` (bit 3, enable dual-antenna yaw), `GPS_YAW_OFFSET = 0`.  
Antenna baseline: 50cm+ → heading error ≈ ±1.1°.

**Current state:** UM982 connected on `/dev/ttyUSB0`. NTRIP RTK corrections injected via `ntrip_rtcm_node.py` → `/mavros/gps_rtk/send_rtcm`.

### 1.5 — Path Tracking: RPP + MPC Hybrid

| Option | Verdict |
|---|---|
| Pure RPP | Rejected — overshoots on tight curves (<0.3m radius) |
| Pure MPC | Rejected — 10ms solve time, unnecessary on straights |
| **RPP primary + MPC fallback** | **Selected** |

RPP handles 95% of path (straights + gentle curves). MPC only for tight corners (curvature > 3.0 m⁻¹, radius < 0.33m). Hybrid cuts MPC usage by ~70%.

MPC solver: OSQP (not CVXPY). Linearized bicycle model, QP formulation, ~5ms solve on Jetson Orin. Falls back to RPP if QP infeasible.

**Current state:** Not built. Phase 2 deliverable.

### 1.6 — Spray Control: Jetson GPIO Direct

| Option | Verdict |
|---|---|
| uXRCE-DDS aux command | Rejected — 5-15ms latency, DDS dependency |
| MAVROS2 relay | Rejected — 40-80ms latency |
| **Jetson GPIO → transistor → relay** | **Selected** — 5-10ms latency, zero autopilot dependency |

Circuit: Jetson GPIO 3.3V → 1kΩ → 2N2222 NPN base → collector drives 12V relay → solenoid. Flyback diode (1N4007).  
E-Stop: NC relay in series with 12V solenoid line. Hardware path independent of software.  
Spray timing: event-driven, tied to trajectory position. Lag compensation: trigger at (target_distance - lag_distance).

**Current state:** Not built. Phase 2/3 deliverable.

### 1.7 — MPC Solver: OSQP

Original proposal used CVXPY+ECOS which cannot handle non-convex constraints (`cp.tan()`, `find_closest_point()` inside optimization loop). Corrected to OSQP with linearized bicycle model.

```python
# Linearize at current state
A = [[1, 0, -v*sin(θ)*dt],
     [0, 1,  v*cos(θ)*dt],
     [0, 0,  1]]
B = [[cos(θ)*dt, 0],
     [sin(θ)*dt, 0],
     [0,          dt/L]]
# Solve QP: minimize ||x_pred - x_desired||² + λ||u||²
# OSQP ~5ms on Jetson Orin
```

Fallback: if OSQP infeasible, fall back to RPP with minimum lookahead.

### 1.8 — Plan B: Jetson Direct Motor Control (No Autopilot, No GPL)

If PX4 Rover offboard proves unreliable, fallback is NOT ArduRover (GPL-3 blocks commercial sale).

```
Jetson Orin ROS2
├─ Path tracking (RPP/MPC) → cmd_vel
├─ Motor mixing node:
│   left_PWM  = (v - ω*track/2) / v_max
│   right_PWM = (v + ω*track/2) / v_max
└─ PCA9685 I2C servo driver → Sabertooth 2x32 ESCs
```

Failsafe: software watchdog + RC receiver on Jetson GPIO + physical E-Stop button → NC relay cuts 12V.  
Licensing: PCA9685 driver = MIT. ROS2 nodes = proprietary. Zero GPL.  
Transition time: ~1 week. PCA9685 cost: ₹500-800.

### 1.9 — NHC (Non-Holonomic Constraint)

Virtual sensor publisher in ROS2, ~50 lines of Python. Publish TwistWithCovarianceStamped at 20Hz with lateral velocity (y) = 0, vertical velocity (z) = 0, covariance 0.001 on y/z.

Benefit: 5-15% improvement on arcs, 12-34% during GPS outages. Zero hardware cost.

---

## 2. Complete System Architecture (Current + Planned)

```
┌──────────────────────────────────────────────────────────────────┐
│                        JETSON ORIN (Compute)                      │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │ ROS2 Humble (DDS middleware)                               │  │
│  │                                                            │  │
│  │  [DXF Parser] → [Path Generator] → [Spline Engine]       │  │   ← Phase 2
│  │                                    ↓                       │  │
│  │                          [Smoothing Filter]                │  │   ← Phase 2
│  │                          (Quintic polynomial)               │  │
│  │                                    ↓                       │  │
│  │  [Mission Manager] → [RPP / MPC Controller] → /cmd_vel    │  │   ← Phase 2
│  │                                    ↓                       │  │
│  │                          [Spray Controller] → GPIO          │  │   ← Phase 3
│  │                                                            │  │
│  │  [robot_localization EKF] ← /mavros/* (PX4 position)      │  │   ← Phase 2
│  │       ↑                    ← /wheel_odometry (STM32 bridge) │  │   ← Phase 2
│  │       ↑                    ← /nhc_constraint (virtual)      │  │   ← Phase 2
│  │       ↓                                                   │  │
│  │  /odom (smooth, 20Hz) → RPP/MPC                           │  │
│  │                                                            │  │
│  │  [ntrip_rtcm_node] → /mavros/gps_rtk/send_rtcm            │  │   ← RUNNING
│  └────────────────────────────────────────────────────────────┘  │
│                              ↓                                  │
│              USB (/dev/ttyACM0) 921600 (MAVROS2)                 │
│              Jetson GPIO (spray, E-Stop)                         │
└──────────────────────────────────────────────────────────────────┘
                               ↓
         ┌────────────────────────────────────────┐
         │        CUBEORANGEPLUS (PX4 v1.16.2)     │
         │                                        │
         │  [PX4 EKF2] ← UM982 RTK (dual-antenna) │  ← RUNNING
         │  [PX4 EKF2] ← ICM42688P IMU            │  ← RUNNING
         │  [Offboard Mode] ← /cmd_vel via MAVROS  │  ← Phase 2
         │  [AUTO Mode] ← Mission waypoints       │  ← WORKING
         │  [Motor Rate PIDs] → Sabertooth 2x32    │  ← RUNNING
         │  [Failsafe: RTL, geofence, E-Stop]      │  ← RUNNING
         │  [RC Override] ← RC receiver             │  ← RUNNING
         └────────────────────────────────────────┘
```

---

## 3. Current PX4 Parameters on FCU

```
RO_YAW_RATE_P = 0.5     RO_YAW_RATE_I = 0.3     RO_YAW_RATE_LIM = 30.0
RO_SPEED_P = 0.5        RO_SPEED_I = 0.1
RO_YAW_P = 1.0           RO_MAX_THR_SPEED = 1.5
NAV_ACC_RAD = 0.1         MIS_YAW_ERR = 25.0
PP_LOOKAHD_GAIN = 1.0     PP_LOOKAHD_MIN = 1.0   PP_LOOKAHD_MAX = 5.0
RD_TRANS_DRV_TRN = 0.52   RD_TRANS_TRN_DRV = 0.0873
RD_WHEEL_TRACK = 0.47     RD_TANK_MODE = 1
GPS_YAW_OFFSET = 180.0    CA_R_REV = 3
```

All 6 firmware bugs fixed. AUTO mode working with straight-line missions.

---

## 4. Hardware Bill of Materials

| Component | Model | Status | Cost (₹) |
|---|---|---|---|
| Autopilot | CubeOrangePlus | Owned, running | — |
| Motor driver | Sabertooth 2x32 | Owned, running | — |
| Companion | Jetson Orin | Owned, running | — |
| GNSS | Holybro UM982 | Owned, running | — |
| Wheel encoders | AMT102 (1024 CPR) × 2 | **Needed** | 6,000-10,000 |
| Encoder bridge | STM32F103 | **Needed** | 500 |
| Spray driver | 2N2222 + 1N4007 + 12V relay | **Needed** | 200 |
| E-Stop | NC relay + hardware button | **Needed** | 300 |
| Plan B servo driver | PCA9685 | **Needed** (fallback only) | 500-800 |
| **Total new hardware** | | | **₹7,000-12,000** |

---

## 5. Software Stack & Licensing

| Layer | Component | License | Status |
|---|---|---|---|
| Core framework | ROS2 Humble | Apache 2.0 | Running |
| Bridge | MAVROS2 | BSD | Running |
| Motor control | PX4 v1.16.2 | BSD | Running |
| Navigation | Nav2 (RPP) | Apache 2.0 | Phase 2 |
| Localization | robot_localization | BSD | Phase 2 |
| MPC solver | OSQP | Apache 2.0 | Phase 2 |
| Geometry engine | Custom (spline) | Proprietary | Phase 2 |
| Path tracking | RPP + OSQP MPC | Proprietary | Phase 2 |
| Spray control | Event-driven GPIO | Proprietary | Phase 3 |
| NHC publisher | Virtual sensor | Proprietary | Phase 2 |
| NTRIP client | ntrip_rtcm_node.py | Proprietary | Running |
| Encoder bridge | STM32F103 firmware | MIT | Phase 2 |

**No GPL-3 code.** PX4 = BSD, ROS2 = Apache 2.0, custom = proprietary.

---

## 6. Phase Plan

### Phase 1 — PX4 Baseline (COMPLETE)
- PX4 v1.16.2 flashed, 6 firmware bugs fixed
- AUTO mode tuned, straight-line missions working
- MAVROS2 bridge running as systemd service
- NTRIP RTK injection running

### Phase 1.5 — Production Hardening (COMPLETE)
- All runtime bugs fixed (20+ issues from audit + Kiro review)
- CRC-24Q validation on RTCM frames
- NTRIP watchdog with exponential backoff
- systemd unit hardened (BindsTo, ProtectSystem=strict)
- deploy.sh symlink-based deployment workflow
- Docs: MAVROS vs DDS comparison, Pure DDS architecture

### Phase 2 — ROS2 Offboard Control (NEXT)
- Setpoint streamer node (velocity → OFFBOARD mode)
- Straight-line velocity control via `/mavros/setpoint_velocity/cmd_vel`
- Arc controller (pure pursuit → MPC)
- robot_localization EKF integration
- NHC virtual sensor publisher
- Wheel encoder bridge (STM32 → UART → Jetson)
- OFFBOARD mode: arm → stream setpoints → mode switch

### Phase 3 — Drawing Integration
- DXF parser → path generator → spline engine
- Spray controller (Jetson GPIO → relay → solenoid)
- Spray timing with lag compensation
- E-Stop hardware integration
- Full mission: path → tracking → spray → done

### Phase 4 — Production Hardening
- Failsafe testing (RTL, geofence, E-Stop)
- Plan B validation (PCA9685 direct motor control)
- 10+ hour continuous operation test
- Customer documentation
- Accuracy certification (±2-3cm arcs)

---

## 7. Competitive Positioning

| Metric | DYX (PX4+ROS2) | Trimble 60 | Raven 2 | TinyMobileRobots |
|---|---|---|---|---|
| Arc accuracy | ±3-5cm (RTK+RPP) | ±2cm | ±5cm | ±1-2cm |
| Spline support | G2 continuous | Limited | No | Yes |
| Spray timing | ±50ms (Jetson GPIO) | ±100ms | N/A | Proprietary |
| Price target | ₹25-35L ($30-42K) | ₹1.2Cr ($150K+) | ₹1.6Cr ($200K+) | ₹80L-1Cr ($100K+) |
| License | Proprietary (no GPL) | Proprietary | Proprietary | Proprietary |

---

## 8. Review Decision Record

| Topic | ChatGPT | Grok | Claude | Final |
|---|---|---|---|---|
| Bridge | Not addressed | uXRCE-DDS (w/ caveats) | MAVROS2 | **MAVROS2** |
| Localization | Jetson-only | PX4-only | Hybrid | **Dual-layer** |
| IMU | Not addressed | PX4 fused | PX4 fused + raw fallback | **CubeOrangePlus built-in** |
| Spray | Bypass PX4 | uXRCE-DDS acceptable | Jetson GPIO | **Jetson GPIO** |
| MPC solver | Not addressed | Not addressed | CVXPY+ECOS (flawed) | **OSQP** |
| RTK module | Not addressed | Not addressed | Not specified | **UM982 dual-antenna** |
| NHC | Not addressed | Not addressed | Not addressed | **Virtual sensor** |
| Plan B | Not addressed | Not addressed | ArduRover GUIDED (GPL) | **Jetson → PCA9685** |