# Pure DDS Architecture (No MAVROS2) — 3WD Marking Rover

**Date:** 2026-05-19 | **Scope:** Replace MAVROS2 entirely with uXRCE-DDS

---

## Architecture

```
┌──────────────────────────────────────────────────────┐
│                   CubeOrangePlus                       │
│  PX4 v1.16.2 + 7 patches (add GpsInjectData to DDS)  │
│                                                        │
│  USB      ─── uXRCE-DDS client (UXRCE_DDS_CFG = USB) │
│  TELEM1   ─── MAVLink → ESP8266 WiFi (MAV_0_CONFIG)  │
│  TELEM2   ─── (unused or GPS2)                       │
└──────────────────┬───────────────────┬────────────────┘
                   │ USB               │ WiFi (~50m)
┌──────────────────┴──────┐   ┌────────┴───────────────┐
│      Jetson Orin         │   │    Laptop QGC           │
│                          │   │  Connects to ESP8266    │
│  /dev/ttyACM0 ← USB     │   │  AP (PixRacer SSID)     │
│  uXRCE-DDS Agent v2.4.2 │   │  Port 14550 UDP        │
│                          │   │                         │
│  /fmu/out/vehicle_       │   │  - Param tuning        │
│   odometry (100Hz)       │   │  - Live monitoring      │
│  /fmu/in/rover_speed_    │   │  - Firmware flash       │
│   setpoint               │   └─────────────────────────┘
│  /fmu/in/rover_steering_ │
│   setpoint               │
│  /fmu/in/vehicle_rtcm_   │  ← Patch 7
│   inject (NTRIP)         │
│                          │
│  ┌─────────────────────┐│
│  │ Arc Controller      ││
│  │ (RoverSpeedSteering)││
│  │ ↑ Path Generator    ││
│  └─────────────────────┘│
│                          │
│  ntrip_node.py →         │
│  /fmu/in/vehicle_rtcm_   │
│   inject                 │
└──────────────────────────┘
```

---

## What Changes from Current Setup

| Component | Current (MAVROS2) | Pure DDS |
|---|---|---|
| USB connection | MAVROS2 (MAVLink) | uXRCE-DDS client |
| QGC path | Jetson MAVROS2 UDP bridge (14550) | ESP8266 WiFi on TELEM1 → laptop directly |
| NTRIP RTK | `/mavros/gps_rtk/send_rtcm` | `/fmu/in/vehicle_rtcm_inject` (needs Patch 7) |
| Control topics | `/mavros/setpoint_raw/local` | `/fmu/in/rover_speed_setpoint` + `rover_steering_setpoint` |
| Odometry | `/mavros/local_position/pose` | `/fmu/out/vehicle_odometry` (100 Hz) |
| Arm/mode | `/mavros/cmd/arming`, `/mavros/set_mode` | `/fmu/in/vehicle_command` (ROS2 service) |
| PX4 params | QGC via MAVROS2 | QGC via ESP8266 WiFi |
| systemd service | `px4-dxp.service` (MAVROS2) | `px4-dds.service` (DDS agent) |
| Plugin denylist | `px4_pluginlists_rover.yaml` | Not needed (DDS has no plugins) |
| Agent version | N/A | v2.4.2 (Humble) — v3.x incompatible |

---

## Three Gaps and Their Solutions

### Gap 1: NTRIP RTK Injection

**Problem:** `GpsInjectData` is NOT in default `dds_topics.yaml`. RTK corrections cannot reach PX4 via DDS.

**Solution: Patch 7 — Add `vehicle_rtcm_inject` to `dds_topics.yaml`**

Edit `src/modules/uxrce_dds_client/dds_topics.yaml`, add under `subscriptions`:
```yaml
- topic: /fmu/in/vehicle_rtcm_inject
  type: px4_msgs::msg::GpsInjectData
```

Then rebuild firmware. NTRIP node publishes to `/fmu/in/vehicle_rtcm_inject` instead of `/mavros/gps_rtk/send_rtcm`.

**Effort:** Medium (one YAML line + firmware rebuild + reflash)
**Risk:** Low (well-defined uORB topic, just exposing it to DDS)

### Gap 2: QGC Connection

**Problem:** DDS doesn't speak MAVLink. QGC needs MAVLink to show telemetry, tune params, flash firmware.

**Solution: ESP8266 WiFi on TELEM1**

| Item | Detail |
|---|---|
| Module | ESP8266 (ESP-01 or NodeMCU, ~$5) |
| Firmware | [MAVESP8266 v1.2.2](https://hamishwillee.github.io/PX4-user_guide/en/telemetry/esp8266_wifi_module.html) |
| Wiring | TELEM1 TX → ESP8266 RX, TELEM1 RX → ESP8266 TX, GND → GND, 3.3V → VCC |
| PX4 params | `MAV_0_CONFIG = TELEM1`, `SER_TEL1_BAUD = 921600` |
| QGC connect | WiFi AP "PixRacer" (pw: `pixracer`), UDP port 14550 |
| Range | ~50m open field |

**Effort:** Low ($5 module + 4 wires)
**Risk:** Low (proven setup, used in hundreds of drone builds)

### Gap 3: PX4 Parameter Access

**Problem:** DDS `VehicleCommand` service can arm/disarm/switch modes, but cannot read/write arbitrary PX4 parameters.

**Solution:** Solved by Gap 2 solution. QGC via ESP8266 provides full param editor.

---

## Required PX4 Parameter Changes

```bash
# DDS on USB (replaces MAVROS2)
UXRCE_DDS_CFG = 0              # USB port (0 = USB CDCACM)

# MAVLink on TELEM1 for ESP8266/QGC
MAV_0_CONFIG = 101             # TELEM1 (was USB)
SER_TEL1_BAUD = 921600         # Match ESP8266

# Disable MAVLink on TELEM2 (prevent conflict)
MAV_1_CONFIG = 0               # Disabled

# Disable DDS time sync (GPS conflict risk)
UXRCE_DDS_SYNCT = 0

# DDS domain ID (must match ROS_DOMAIN_ID on Jetson)
UXRCE_DDS_DOM_ID = 0
```

**Note:** `UXRCE_DDS_CFG = 0` for USB may need verification — check PX4 serial port mapping for CubeOrange+. Some builds use numeric IDs rather than symbolic names.

---

## Jetson Software Setup

### 1. Install uXRCE-DDS Agent v2.4.2

```bash
# Build from source (v2.4.2 for Humble compatibility)
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 2. New systemd service: `px4-dds.service`

```ini
[Unit]
Description=PX4 DDS Bridge - uXRCE-DDS Agent (CubeOrangePlus)
After=network.target network-online.target
Wants=network-online.target

[Service]
Type=simple
User=flash
Group=flash
WorkingDirectory=/home/flash/PX4_DXP
ExecStart=/usr/local/bin/MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600
Restart=always
RestartSec=5

Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=0

StandardOutput=journal
StandardError=journal
SyslogIdentifier=px4-dds

[Install]
WantedBy=multi-user.target
```

### 3. NTRIP Node Changes

Current:
```python
self.pub = self.create_publisher(RTCM, "/mavros/gps_rtk/send_rtcm", 10)
```

New:
```python
from px4_msgs.msg import GpsInjectData

self.pub = self.create_publisher(
    GpsInjectData,
    "/fmu/in/vehicle_rtcm_inject",
    1  # Queue size 1 — RTCM corrections are time-critical, drop old
)
```

Message format changes — `GpsInjectData` has different fields than `RTCM`:
```python
msg = GpsInjectData()
msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
msg.len = len(rtcm_data)
msg.data = list(rtcm_data) + [0] * (180 - len(rtcm_data))  # Fixed 180-byte array
```

### 4. Control Node Template

```python
#!/usr/bin/env python3
"""Arc controller via DDS — uses RoverSpeedSetpoint + RoverSteeringSetpoint"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleOdometry, RoverSpeedSetpoint, RoverSteeringSetpoint
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus

# PX4 publishers use BEST_EFFORT + TRANSIENT_LOCAL — ROS2 defaults are incompatible
PX4_SUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=5
)

class ArcController(Node):
    def __init__(self):
        super().__init__("arc_controller")

        # Subscribe to vehicle state
        self.odom_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry",
            self.odom_callback, PX4_SUB_QOS)

        self.status_sub = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status",
            self.status_callback, PX4_SUB_QOS)

        # Publishers for control
        self.speed_pub = self.create_publisher(
            RoverSpeedSetpoint, "/fmu/in/rover_speed_setpoint", 1)
        self.steering_pub = self.create_publisher(
            RoverSteeringSetpoint, "/fmu/in/rover_steering_setpoint", 1)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 1)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 1)

        # 100 Hz control loop
        self.timer = self.create_timer(0.01, self.control_loop)

    def control_loop(self):
        # Must publish offboard control mode at same rate
        mode_msg = OffboardControlMode()
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        mode_msg.speed = True
        mode_msg.steering = True
        self.offboard_pub.publish(mode_msg)

        # Publish speed + steering setpoints
        speed_msg = RoverSpeedSetpoint()
        speed_msg.timestamp = mode_msg.timestamp
        speed_msg.speed_body_x = self.target_speed  # m/s forward

        steering_msg = RoverSteeringSetpoint()
        steering_msg.timestamp = mode_msg.timestamp
        steering_msg.normalized_speed_diff = self.target_steering  # [-1, 1]

        self.speed_pub.publish(speed_msg)
        self.steering_pub.publish(steering_msg)
```

---

## Migration Path (4 Steps, Each Reversible)

### Step 1: Add TELEM2 Wire + Test DDS Alongside MAVROS2
- Wire CubeOrange+ TELEM2 TX/RX/GND to Jetson UART or USB-serial
- Set `UXRCE_DDS_CFG=102`, `SER_TEL2_BAUD=921600`, `MAV_1_CONFIG=0`
- Install uXRCE-DDS Agent v2.4.2 on Jetson
- Verify: `ros2 topic echo /fmu/out/vehicle_odometry` shows data at 100 Hz
- **MAVROS2 unchanged, still active on USB**

### Step 2: Build Arc Controller Using DDS Topics
- Create `RoverSpeedSetpoint` + `RoverSteeringSetpoint` publisher node
- Use `OffboardControlMode` with `speed=True`, `steering=True`
- Test straight-line drive via DDS while MAVROS2 still provides QGC/NTRIP
- **Both bridges running, control via DDS only**

### Step 3: Add ESP8266 on TELEM1 + Patch 7 (NTRIP via DDS)
- Wire ESP8266 to TELEM1, flash MAVESP8266 firmware
- Set `MAV_0_CONFIG=101` (TELEM1) for WiFi QGC
- Add `GpsInjectData` to `dds_topics.yaml` (Patch 7), rebuild + reflash
- Modify NTRIP node to publish to `/fmu/in/vehicle_rtcm_inject`
- **All functions now available via DDS + ESP8266**

### Step 4: Remove MAVROS2
- Stop `px4-dxp.service`
- Set `UXRCE_DDS_CFG=0` (USB) — move DDS to USB, free TELEM2
- Create new `px4-dds.service` for DDS agent only
- Verify all functions: control, NTRIP, QGC via ESP8266
- **Pure DDS architecture achieved**

---

## Risks and Mitigations

| Risk | Severity | Mitigation |
|---|---|---|
| DDS agent v3.x incompatibility | High | Pin agent to v2.4.2, document version in service file |
| QoS mismatch causes no data flow | High | Use `BEST_EFFORT` + `TRANSIENT_LOCAL` on all subscribers |
| ESP8266 WiFi range insufficient | Medium | ESP32 alternative (~$10) for longer range; or use wired Ethernet |
| Patch 7 breaks DDS bridge | Medium | Test in SITL first; `GpsInjectData` is a simple subscription, low risk |
| Single point of failure (DDS only) | Medium | ESP8266 provides independent monitoring channel for diagnostics |
| Rover setpoints marked "experimental" | Low | API stable since Aug 2025 (PR #140), library v2.1.0 released |
| DDS ACKNACK storms on packet loss | Low | Wired serial eliminates this risk entirely |
| NTRIP node message format change | Low | `GpsInjectData` has fixed 180-byte array, well-documented |

---

## What You Lose vs What You Gain

### You Lose
- MAVROS2 plugin ecosystem (image, vibration, etc.) — not needed for rover
- QGC via Jetson network (must use ESP8266 WiFi instead)
- Simultaneous QGC + ROS2 control on same laptop network segment
- Easy param access from ROS2 (no `ros2 param` for PX4 params)

### You Gain
- 100 Hz native control loop (vs 50 Hz MAVLink-limited)
- Rover-specific setpoints (`RoverSpeedSetpoint` + `RoverSteeringSetpoint`)
- PX4 ROS2 Interface Library access (`RoverSpeedSteeringSetpointType`)
- <1 ms latency (vs 3–5 ms MAVROS2)
- Simpler systemd service (1 agent vs MAVROS2 + plugins)
- No MAVLink message size limits (DDS has larger MTU)
- Direct `VehicleCommand` ROS2 service for arm/mode (vs MAVROS simulated services)
- Wireless QGC via ESP8266 (no cable to laptop during field work)

---

## Sources

- [PX4 uXRCE-DDS Documentation](https://docs.px4.io/main/en/middleware/uxrce_dds)
- [PX4 dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml)
- [RTCM via DDS Issue #212](https://github.com/PX4/px4_ros_com/issues/212)
- [PX4 ESP8266 WiFi Module Guide](https://hamishwillee.github.io/PX4-user_guide/en/telemetry/esp8266_wifi_module.html)
- [PX4 WiFi Telemetry](https://docs.px4.io/v1.16/en/telemetry/telemetry_wifi)
- [PX4 ROS2 Interface Library](https://github.com/Auterion/px4-ros2-interface-lib)
- [RoverSpeedSteeringSetpointType API](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1RoverSpeedSteeringSetpointType.html)
- [PR #140 — Rover setpoints + example](https://github.com/Auterion/px4-ros2-interface-lib/pull/140)
- [uXRCE-DDS RTT Bug #22286](https://github.com/PX4/PX4-Autopilot/issues/22286)
- [Micro XRCE-DDS Agent v2.4.2](https://github.com/eProsima/Micro-XRCE-DDS-Agent/tree/v2.4.2)