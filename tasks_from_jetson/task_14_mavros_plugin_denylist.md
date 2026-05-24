# Task 14 — Unused MAVROS Plugins Still Active

**Priority:** LOW
**File:** `px4_pluginlists_rover.yaml`
**Lines:** 1–11

---

## Problem

The current denylist removes 6 plugins, but several more load by default that have no function on a ground rover:

```yaml
# Current denylist:
plugin_denylist:
  - image_pub
  - vibration
  - distance_sensor
  - rangefinder
  - wheel_odometry
  - odometry
```

## Plugins That Should Also Be Denied

These can be confirmed by running `ros2 node info /mavros` and checking which topics are being advertised — each unused plugin adds topics, subscribers, and MAVLink message handlers:

| Plugin | Why it can be denied |
|--------|---------------------|
| `fake_gps` | Injects simulated GPS. Never used on real hardware. |
| `hil` | Hardware-In-the-Loop simulation only. On real rover = dead weight. |
| `landing_target` | Copter/VTOL precision landing. Not a rover concept. |
| `mocap_pose_estimate` | Motion capture pose injection. No mocap system present. |
| `vision_pose_estimate` | Visual odometry injection. No camera/VIO pipeline. |
| `setpoint_accel` | Acceleration setpoints. Phase 2 uses `setpoint_raw/local` and `setpoint_velocity/cmd_vel`. |
| `safety_area` | Publishes fence/safety zone markers. No visualiser consuming this. |

## Impact of Leaving Them Active

Each active plugin:
- Registers MAVLink message handlers (scanned on every incoming message)
- Advertises ROS2 topics (adds to DDS graph, costs discovery bandwidth)
- Consumes ~1–5MB RSS each

On a Jetson this is not a memory crisis (current RSS is 282MB vs 3GB limit). But extra plugins increase the MAVLink message dispatch latency and make `ros2 topic list` noisier — harder to spot legitimate topics.

## Required Fix (do not apply — analysis only)

```yaml
/mavros/mavros_node:
  ros__parameters:
    plugin_denylist:
      - image_pub
      - vibration
      - distance_sensor
      - rangefinder
      - wheel_odometry
      - odometry
      - fake_gps
      - hil
      - landing_target
      - mocap_pose_estimate
      - vision_pose_estimate
      - setpoint_accel
      - safety_area
```

Before adding any plugin to the denylist, verify Phase 2 doesn't use it. For `setpoint_accel` — Phase 2 uses `setpoint_raw` (position target with type_mask) and `setpoint_velocity`, neither of which is `setpoint_accel`.

---

**Depends on:** None
**Blocks:** Nothing
