# Quick Reference — Current RPP Yaw, Timing, and Pose-Latency Path

This note replaces the older P0.5/P2.5/P2.4 checklist. The current mainline no
longer publishes `/rpp/yaw_setpoint_ned`.

## Current Runtime Contract

### Explicit yaw in `twist_to_setpoint_node`

`twist_to_setpoint_node.py` computes explicit ENU yaw directly from
`/rpp/velocity_ned`:

```text
yaw_ned = atan2(v_e, v_n)
yaw_enu = atan2(v_n, v_e)
```

When speed is below 1 cm/s, it holds the previous yaw so stops do not snap to
North. There is no `use_explicit_yaw` parameter anymore; yaw is always included
in the MAVROS `PositionTarget`.

### Feedforward yaw-rate

`rpp_controller_node.py` publishes `/rpp/yaw_rate_body` as:

```text
yaw_rate_body = kappa * speed + yaw_rate_feedback_gain * heading_error
```

Current defaults:

```text
use_feedforward_yaw_rate = true
yaw_rate_feedback_gain = 0.0
max_yaw_rate_body = 0.45
```

`twist_to_setpoint_node.py` sends type_mask `455` when yaw-rate is fresh and
nonzero. If yaw-rate is zero or stale, it sends type_mask `2503` so PX4 ignores
yaw-rate while still receiving velocity + yaw.

### Velocity-based pose extrapolation

`use_imu_extrapolation` is still the parameter name, but the implementation is
velocity-based. RPP subscribes to `/mavros/local_position/velocity_local`, swaps
MAVROS ENU velocity to NED, and projects the pose forward by `v * pose_age`.
Raw IMU acceleration is intentionally not used because gravity leakage is larger
than the useful correction at marking speeds.

Enable for testing:

```bash
ros2 param set /rpp_controller use_imu_extrapolation true
```

Debug log to watch:

```bash
ros2 run rpp_controller_node --ros-args --log-level debug
# Look for "P2.4 v-extrapolation" messages.
```

## Parameter Summary

| Parameter | Node | Default | Notes |
|---|---|---:|---|
| `use_imu_extrapolation` | `rpp_controller` | `false` | Enables velocity-based pose extrapolation |
| `imu_max_extrap_age_s` | `rpp_controller` | `0.10` | Extra pose-age budget when extrapolating |
| `use_feedforward_yaw_rate` | `rpp_controller` | `true` | Publishes `/rpp/yaw_rate_body` |
| `yaw_rate_feedback_gain` | `rpp_controller` | `0.0` | Feedback term; current mainline uses pure feedforward |
| `max_yaw_rate_body` | `rpp_controller` | `0.45` | Clamp, rad/s |

## Testing

```bash
cd ~/PX4_DXP
python3 src/test_p05_yaw_setpoint.py -v
python3 src/test_sprint2_geometry.py
python3 src/test_smoke_rpp_controller.py
```

`test_smoke_rpp_controller.py` needs ROS2 Python packages (`rclpy`,
`geometry_msgs`, `mavros_msgs`) and is intended for the Jetson/ROS2 environment.

## Troubleshooting

### Yaw-rate not active

```bash
ros2 topic echo /rpp/yaw_rate_body
ros2 topic echo /mavros/setpoint_raw/local | grep -E "type_mask|yaw_rate"
```

Expect type_mask `455` only while `/rpp/yaw_rate_body` is fresh and nonzero.
Expect type_mask `2503` while stopped or when yaw-rate is stale.

### Pose extrapolation not active

```bash
ros2 param get /rpp_controller use_imu_extrapolation
ros2 topic echo /mavros/local_position/velocity_local --once
```

If velocity is missing or older than `imu_max_extrap_age_s`, RPP falls back to
the raw MAVROS pose.

## Files Involved

```text
src/rpp_controller_node.py
  - publishes /rpp/velocity_ned, /rpp/yaw_rate_body, /rpp/debug
  - performs velocity-based pose extrapolation

src/twist_to_setpoint_node.py
  - computes explicit yaw from velocity
  - conditionally forwards yaw-rate feedforward

src/test_p05_yaw_setpoint.py
  - validates yaw math, velocity ENU/NED swap, and type masks
```
