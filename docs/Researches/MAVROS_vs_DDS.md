# MAVROS2 vs uXRCE-DDS — Comparison for 3WD Marking Rover

**Date:** 2026-05-19 | **Scope:** PX4 v1.16.2 + CubeOrangePlus + Jetson Orin

---

## Architecture Overview

### MAVROS2 (Current Setup)

```
Jetson Orin                    CubeOrangePlus
┌──────────┐   MAVLink v2     ┌──────────┐
│ MAVROS2  │ ←──────────────→│ PX4      │
│ (ROS2)   │  /dev/ttyACM0   │ v1.16.2  │
│          │  USB @ 921600    │          │
└──────────┘                  └──────────┘
     ↕ ROS2 topics
  Arc Controller
  NTRIP node
  QGC UDP bridge (14550)
```

- Translates MAVLink messages ↔ ROS2 topics
- Every message goes through: uORB → MAVLink serialization → serial → MAVLink deserialization → ROS2 message
- Added overhead at every hop

### uXRCE-DDS (Native PX4 Bridge)

```
Jetson Orin                    CubeOrangePlus
┌──────────┐   XRCE-DDS       ┌──────────┐
│ DDS Agent│ ←──────────────→│ DDS      │
│ (ROS2)   │  serial or UDP   │ Client   │
│          │                  │ (in PX4) │
└──────────┘                  └──────────┘
     ↕ ROS2 topics (native px4_msgs)
  Arc Controller
  NTRIP node
```

- Direct uORB → DDS mapping, no MAVLink translation
- PX4 firmware includes DDS client by default
- Agent runs on companion as a proxy to the DDS network

---

## Comparison Table

| Aspect | MAVROS2 | uXRCE-DDS |
|---|---|---|
| **Protocol** | MAVLink v2 → ROS2 translation | Direct DDS (XRCE-DDS) |
| **One-way latency** | 3–5 ms (serialize + serial + deserialize) | <1 ms (direct CDR) |
| **Max reliable rate** | ~50 Hz (MAVLink constrained) | 100 Hz native (uORB rate) |
| **Message types** | MAVLink types (info loss in translation) | Native `px4_msgs` (1:1 uORB mapping) |
| **Rover setpoints** | Generic `TrajectorySetpoint` only | `RoverSpeedSetpoint` + `RoverSteeringSetpoint` (rover-specific) |
| **Resource usage** | Heavy (full ROS2 env + plugin system) | Medium (DDS agent only) |
| **Network stability** | Good (MAVLink handles packet loss gracefully) | Vulnerable on WiFi (ACKNACK storms on loss) — stable on wired serial |
| **QGC bridge** | Built-in (`gcs_url:=udp-b://:14550@`) | Separate — needs MAVLink on another port |
| **NTRIP/RTK injection** | `/mavros/gps_rtk/send_rtcm` → MAVLink GPS_RTCM_DATA | NOT in default `dds_topics.yaml` (needs Patch 7) |
| **PX4 ROS2 Interface Lib** | Not supported (needs DDS topics) | Full support — `RoverSpeedSteeringSetpointType` etc. |
| **PX4 parameter access** | Full (via QGC + MAVLink PARAM) | Limited (`VehicleCommand` service, no param read/write) |
| **Firmware flash** | QGC via MAVROS2 UDP | QGC via separate MAVLink connection |
| **Agent version** | N/A | Must use v2.4.2 (Humble) — v3.x incompatible |
| **QoS compatibility** | ROS2 defaults work | PX4 uses TRANSIENT_LOCAL + BEST_EFFORT (incompatible with ROS2 defaults) |
| **Maturity for rovers** | Proven, Phase 1 worked | New in v1.16+, rover setpoints marked "experimental" |

---

## Latency Impact on Sub-cm Accuracy

At 0.5 m/s rover speed, 1 cm position error accumulates in 20 ms.

| Control Rate | Loop Period | Latency | Computation Budget | Verdict |
|---|---|---|---|---|
| MAVROS2 @ 50 Hz | 20 ms | 3–5 ms | 10–14 ms | Tight but workable |
| MAVROS2 @ 100 Hz | 10 ms | 3–5 ms | 0–4 ms | Insufficient — latency eats budget |
| DDS @ 100 Hz | 10 ms | <1 ms | 8+ ms | Comfortable margin |

At 100 Hz, MAVROS2's latency consumes 30–50% of the loop period. DDS at <1 ms leaves 80%+ for path controller computation.

---

## Rover Setpoint Types

### Via MAVROS2: TrajectorySetpoint Only

```
/mavros/setpoint_raw/local  →  TrajectorySetpoint (position, velocity, yaw in NED)
```

- Copter-first design — no steering concept
- Must compute NED position/velocity from rover kinematics
- No validation — bad setpoints accepted silently

### Via DDS: Rover-Specific Setpoints

| Topic | Type | Purpose |
|---|---|---|
| `/fmu/in/rover_speed_setpoint` | RoverSpeedSetpoint | Forward speed (m/s) |
| `/fmu/in/rover_steering_setpoint` | RoverSteeringSetpoint | Normalized steering [-1, 1] |
| `/fmu/in/rover_position_setpoint` | RoverPositionSetpoint | Position target |
| `/fmu/in/rover_throttle_setpoint` | RoverThrottleSetpoint | Direct throttle |
| `/fmu/in/rover_rate_setpoint` | RoverRateSetpoint | Yaw rate |
| `/fmu/in/rover_attitude_setpoint` | RoverAttitudeSetpoint | Heading target |

**For differential drive:** `RoverSpeedSetpoint` + `RoverSteeringSetpoint` is the natural pair — exactly what a differential kinematic model outputs.

### PX4 ROS2 Interface Library (DDS Only)

```cpp
#include <px4_ros2/control/setpoint_types/experimental/rover/speed_steering.hpp>

// RoverSpeedSteeringSetpointType — purpose-built for differential drive
speed_steering_setpoint_.update(
    /*speed_body_x=*/0.5f,            // m/s forward
    /*normalized_steering_setpoint=*/0.3f  // [-1,1] speed diff L/R
);
```

- Automatically sets `OffboardControlMode` flags
- Validates inputs before they reach PX4
- Can register a custom drive mode (safer than raw OFFBOARD)
- **Not available via MAVROS2**

---

## DDS Topics Available (v1.16+)

### Publications (PX4 → ROS2, `/fmu/out/`)

| Topic | Rate | Purpose |
|---|---|---|
| `vehicle_odometry` | 100 Hz | Position, velocity, attitude, variances |
| `vehicle_attitude` | 50 Hz | Quaternion attitude |
| `vehicle_local_position` | 50 Hz | Detailed local position |
| `vehicle_gps_position` | 50 Hz | Raw GPS (SensorGps) |
| `vehicle_status` | 5 Hz | Armed, mode, failsafe |
| `battery_status` | 1 Hz | Voltage |
| `vehicle_land_detected` | 5 Hz | Landed state |
| `failsafe_flags` | 5 Hz | Failsafe indicators |
| `estimator_status_flags` | 5 Hz | EKF status |

### Subscriptions (ROS2 → PX4, `/fmu/in/`)

| Topic | Purpose |
|---|---|
| `rover_speed_setpoint` | Forward speed |
| `rover_steering_setpoint` | Normalized steering |
| `rover_position_setpoint` | Position target |
| `rover_throttle_setpoint` | Direct throttle |
| `rover_rate_setpoint` | Yaw rate |
| `vehicle_command` | Arm, mode switch |
| `offboard_control_mode` | Control mode flags |
| `trajectory_setpoint` | Legacy setpoint (copter-style) |

### NOT in DDS (MAVLink/MAVROS2 Only)

| Function | Impact |
|---|---|
| RTK/RTCM injection (`GpsInjectData`) | Sub-cm accuracy impossible without workaround |
| PX4 parameter read/write | No live param tuning from ROS2 |
| QGC connection | No live monitoring without separate MAVLink |

---

## Network Stability Warning

uXRCE-DDS uses RTPS protocol which generates Heartbeat/ACKNACK packets. On packet loss:
- MAVLink: Retransmits only lost packets, recovers gracefully
- DDS: ACKNACK storms can cascade, causing 1–2 second communication blackouts

**Mitigation:** Use wired serial (not WiFi) for DDS. Our setup uses USB serial — safe.

---

## Known DDS Bugs (Fixed in v1.16.2)

| Bug | Fix | Our Status |
|---|---|---|
| RTT too high on serial (#22286) | Fixed v1.14+ | Not affected |
| Timesync convergence delay (#22382) | Fixed Apr 2024 | Not affected |
| Client not receiving at 57600 baud (#22323) | Use 921600 | Not affected |
| GPS time sync conflict (#22463) | Disabled by default | Set `UXRCE_DDS_SYNCT=0` |

---

## Recommendation

**Phase 2: Hybrid (MAVROS2 + DDS)**
- MAVROS2 stays on USB for QGC + NTRIP (works today)
- Add uXRCE-DDS on TELEM2 for 100 Hz control loop + rover setpoints
- Requires 1 serial wire from CubeOrange+ TELEM2 to Jetson

**Phase 3/4: Pure DDS + ESP8266**
- Remove MAVROS2, DDS on USB only
- ESP8266 on TELEM1 for wireless QGC ($5 module)
- Patch 7 (add `GpsInjectData` to `dds_topics.yaml`) for NTRIP via DDS

---

## Sources

- [PX4 uXRCE-DDS Documentation](https://docs.px4.io/main/en/middleware/uxrce_dds)
- [PX4 dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml)
- [PX4 Rover API](https://docs.px4.io/main/en/flight_modes_rover/api)
- [PX4 ROS2 Interface Library](https://auterion.github.io/px4-ros2-interface-lib/group__setpoint__types__rover.html)
- [RoverSpeedSteeringSetpointType API](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1RoverSpeedSteeringSetpointType.html)
- [MAVSDK vs MAVROS vs uXRCE-DDS Comparison](https://quad-drone-lab.co.kr/px4-mavsdk-c-programming-episode-11-complete-comparison-of-mavsdk-vs-mavros-vs-uxrce-dds/)
- [RTCM via DDS Issue #212](https://github.com/PX4/px4_ros_com/issues/212)
- [PX4 Offboard delay #16290](https://github.com/PX4/PX4-Autopilot/issues/16290)