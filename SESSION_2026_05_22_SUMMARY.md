# RPP Upgrade Path — Session 2026-05-22 Summary

## Objective

Capture the Phase C RPP runtime changes that closed pose latency and added yaw
feedforward support. This note has been corrected to match current code, not
the short-lived intermediate `/rpp/yaw_setpoint_ned` design.

## Current Runtime Reality

### Explicit Yaw

`/rpp/yaw_setpoint_ned` has been removed.

`twist_to_setpoint_node.py` computes explicit yaw directly from fresh
`/rpp/velocity_ned`:

```text
yaw_NED = atan2(v_east, v_north)
yaw_ENU = pi/2 - yaw_NED = atan2(v_north, v_east)
```

When speed is below 1 cm/s, the node holds the last yaw to avoid `atan2(0,0)`.
The PositionTarget mask is `2503` when yaw_rate is ignored.

### Feedforward Yaw Rate

RPP publishes `/rpp/yaw_rate_body` as body yaw rate in NED convention, CW
positive. `twist_to_setpoint_node.py` includes it only when the value is fresh
and non-zero:

| Condition | PositionTarget mask | Meaning |
|---|---:|---|
| fresh non-zero `/rpp/yaw_rate_body` | `455` | velocity + yaw + yaw_rate |
| stale/zero yaw rate | `2503` | velocity + yaw, yaw_rate ignored |

### Pose Extrapolation

The parameter is still named `use_imu_extrapolation`, but the implementation is
velocity-based. RPP subscribes to `/mavros/local_position/velocity_local`,
converts ENU velocity to NED, and extrapolates pose with `dp = v * dt` bounded
by `imu_max_extrap_age_s`. Debug logs use `P2.4 v-extrapolation`.

## Debug Array

`/rpp/debug` is now a 39-field `Float32MultiArray`:

- `[0..7]`: stable runtime fields used by legacy consumers
- `[8]`: `l_d_raw_m`
- `[9]`: `kappa_speed`
- `[10]`: `yaw_rate_cmd_rad_s`
- `[11..38]`: active RPP parameter snapshot for every bag sample

State code `[7]` values:

| Code | Name | Meaning |
|---:|---|---|
| -1 | STALE | pose/input stale |
| 0 | IDLE | no active path |
| 1 | TRACKING | following path |
| 2 | APPROACH | final approach |
| 3 | DONE | goal reached |
| 4 | RTK_WAIT | GPS fix below RTK_FIXED |
| 5 | JUMP_SKIP | one-cycle EKF/position-jump skip |

## Safety Correction

RPP ignores empty `Path` messages. E-stop and soft-stop paths must publish a
single-point path at the rover's current NED position, then the server switches
MANUAL and disarms for e-stop.

## Useful Verification

```bash
ros2 topic echo /rpp/debug --once
ros2 topic echo /rpp/yaw_rate_body
ros2 topic echo /mavros/setpoint_raw/local --once
ros2 param get /rpp_controller use_imu_extrapolation
```

## Status

- Current validated RPP params include `max_yaw_rate_body=0.45`,
  `a_lat_max=0.3`, and `corner_smooth_radius_m=0.5`.
- P2.4 is velocity-based despite the legacy parameter name.
- P3.1 yaw-rate feedforward is conditional at the MAVROS bridge.
- Historical P0.5 unit-test/deployment notes referencing
  `/rpp/yaw_setpoint_ned` are obsolete.
