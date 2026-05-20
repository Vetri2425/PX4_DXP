# OFFBOARD Action Plan — Ranked by Safety Priority

Companion to:
- `OFFBOARD_AUDIT_FINDINGS.md` — full findings with file:line evidence
- `OFFBOARD_PATCHES.md` — concrete patch specs (P1–P4)

Do these in order. Items 1–4 are blocking; do not attempt a live OFFBOARD test until all
four are done. Items 5–7 are for accuracy and robustness. Items 8–9 are Phase 2 / DDS prep.

---

## #1 — Set safe failsafe params before any OFFBOARD attempt  
**Priority: BLOCKING — do this before any armed test**  
**Effort: 5 minutes in QGC**

The defaults will let the rover coast 40 cm on signal loss and fall back to a mode that
doesn't work for an RC-less rover.

| Parameter | Set to | Was | Reason |
|---|---|---|---|
| `COM_OF_LOSS_T` | `0.2` | `1.0` | Max coast = 8 cm @ 0.4 m/s |
| `COM_OBL_RC_ACT` | `5` | `0` | Hold_mode = safe auto-stop, no RC needed |
| `COM_RCL_EXCEPT` | `4` (bit 2) | `0` | Don't cascade to RC-loss failsafe during OFFBOARD |
| `COM_ARM_WO_GPS` | `0` | varies | Require valid GPS to arm — marking rover needs position |
| `RO_SPEED_LIM` | `0.5` | varies | Absolute cap; 25% over nominal 0.4 m/s |
| `RO_MAX_THR_SPEED` | `0.5` | varies | Must match RO_SPEED_LIM or feedforward is wrong |
| `RO_ACCEL_LIM` | `0.4` | varies | 1 s to reach cruise |
| `RO_DECEL_LIM` | `0.8` | varies | 0.5 s stop, 2× accel for safety |
| `RO_JERK_LIM` | `5.0` | varies | Smooth speed profile through pure pursuit |
| `PP_LOOKAHD_MIN` | `0.1` | varies | Minimum pure-pursuit look-ahead 10 cm |
| `PP_LOOKAHD_MAX` | `1.0` | varies | Max look-ahead 1 m at cruise |
| `PP_LOOKAHD_GAIN` | `0.5` | varies | Look-ahead = 0.5 × speed (0.2 m at 0.4 m/s) |
| `RD_TANK_MODE` | `0` | `0` | Keep 0 for OFFBOARD; only flip to 1 for joystick debug |

Save all to EEPROM / SD card. Verify with `param show COM_OF_LOSS_T` etc. in MAVLink shell.

---

## #2 — Patch DifferentialVelControl: fix zero-velocity and sign bugs (P3+P4)  
**Priority: BLOCKING — every OFFBOARD stop misbehaves without this**  
**Effort: 30 min code + rebuild**

The patch is in `OFFBOARD_PATCHES.md` → P3+P4 combined block.

File to edit in the fork:
```
src/modules/rover_differential/DifferentialVelControl/DifferentialVelControl.cpp
```

Function: `DifferentialVelControl::generateVelocitySetpoint()`

Replace the speed+bearing calculation with the combined block that:
1. Holds current yaw when velocity magnitude < 0.01 m/s (fixes North-snap on stop).
2. Computes signed speed by projecting NED velocity onto body-forward axis (fixes can't-reverse).

**Extend `build_rover.yml` to copy this file:**

```yaml
- name: Apply rover patches
  run: |
    cp fork_patches/.../RoverDifferential.cpp  src/modules/rover_differential/RoverDifferential.cpp
    cp fork_patches/.../RoverDifferential.hpp  src/modules/rover_differential/RoverDifferential.hpp
    cp fork_patches/.../module.yaml            src/modules/rover_differential/module.yaml
    cp fork_patches/.../RoverLandDetector.cpp  src/modules/land_detector/RoverLandDetector.cpp
    cp fork_patches/.../mission_block.cpp      src/modules/navigator/mission_block.cpp
    # NEW — add these two lines:
    cp fork_patches/.../DifferentialVelControl.cpp \
       src/modules/rover_differential/DifferentialVelControl/DifferentialVelControl.cpp
    cp fork_patches/.../DifferentialPosControl.cpp \
       src/modules/rover_differential/DifferentialPosControl/DifferentialPosControl.cpp
    cp fork_patches/.../DifferentialPosControl.hpp \
       src/modules/rover_differential/DifferentialPosControl/DifferentialPosControl.hpp
```

Push fork → CI builds → download and flash `.px4` from Actions artifact.

---

## #3 — Patch DifferentialPosControl: invalidate cached target on OFFBOARD exit (P1)  
**Priority: BLOCKING — closes #18346 latent runaway**  
**Effort: 30 min code + included in same rebuild as #2**

File to edit in the fork:
```
src/modules/rover_differential/DifferentialPosControl/DifferentialPosControl.cpp
src/modules/rover_differential/DifferentialPosControl/DifferentialPosControl.hpp
```

Add `bool _was_offboard{false}` to the .hpp private section.

In `.cpp`, inside `DifferentialPosControl::updatePosControl()`, right after
`updateSubscriptions()`, insert the edge-detect + NaN-reset block from `OFFBOARD_PATCHES.md`
→ P1. Also add the disarm-guard NaN reset in the else-branch.

These files are included in the new `build_rover.yml` `cp` commands added in #2.

---

## #4 — Patch RoverDifferential::Run(): add was_armed safe-stop guard (P2)  
**Priority: High — prevents motor-linger on disarm, matches Ackermann/Mecanum pattern**  
**Effort: 20 min code + same rebuild**

File to edit in the fork (already copied by CI):
```
src/modules/rover_differential/RoverDifferential.cpp
src/modules/rover_differential/RoverDifferential.hpp
```

Add `bool _was_armed{false}` to .hpp.

In `RoverDifferential::Run()`, wrap the `generateActuatorSetpoint()` call with the
`_was_armed` guard from `OFFBOARD_PATCHES.md` → P2. On disarm, publish a zero actuator_motors
and reset the slew-rate.

This keeps parity with the Ackermann / Mecanum module pattern and closes the one-cycle
linger.

---

## #5 — Verify Hold_mode actually stops the rover in SITL  
**Priority: High — confirm failsafe behaviour before field test**  
**Effort: 1-2 hours**

Steps:
1. Build firmware with patches from #2-#4.
2. Flash to SITL (`make px4_sitl gazebo-classic_rover` or use `make px4_sitl jmavsim`).
   Alternatively, if SITL rover is not configured, test on bench with motors disconnected.
3. Sequence:
   a. Arm, enter OFFBOARD, publish a constant velocity setpoint.
   b. Kill setpoint stream from MAVROS2 side. Wait for `COM_OF_LOSS_T = 0.2s`.
   c. Verify nav_state transitions to AUTO_LOITER (`NAVIGATION_STATE_AUTO_LOITER = 5`).
   d. Verify `differential_velocity_setpoint.speed` drops to 0 within 1 cycle.
   e. Re-publish setpoints. Verify nav_state returns to OFFBOARD.
4. Repeat with position-mode setpoints.
5. Repeat with `COM_OBL_RC_ACT=7` (Disarm) to check disarm path as backup.

Success criterion: speed reaches 0 within 300 ms of setpoint loss. No North-snap rotation.

---

## #6 — Validate ±3cm arc accuracy with position-mode OFFBOARD  
**Priority: High — accuracy gate for production**  
**Effort: 2-4 hours field test**

Recommended OFFBOARD setpoint strategy:

```python
# ROS2 node — publish position setpoints along a precomputed arc
# waypoint spacing = 5 cm (every 0.05 m along the arc)
# setpoint type: position only (type_mask = ignore vx/vy/vz/ax/ay/az/yaw/yaw_rate)

msg = PositionTarget()
msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
msg.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                 PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                 PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                 PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
msg.position.x = waypoint_n   # NED north
msg.position.y = waypoint_e   # NED east
msg.position.z = 0.0          # not used by rover
```

Publish rate: 50 Hz. Advance waypoint index whenever rover passes within `NAV_ACC_RAD`
(default 0.5m → reduce to 0.1m for marking accuracy).

Set `NAV_ACC_RAD = 0.10` for the marking rover. This is the waypoint-switch radius inside
`DifferentialPosControl::goToPositionMode()`.

Logging: record `rover_position_setpoint.position_ned` vs `vehicle_local_position.xy`
in ulog. Cross-track error = perpendicular distance from the arc line.

Success criterion: RMS cross-track error < 3 cm at 0.4 m/s on a 1 m radius arc.

---

## #7 — MAVROS2 OFFBOARD node minimum implementation  
**Priority: Medium — needed for Phase 1 production**  
**Effort: 1-2 days**

Minimum safe OFFBOARD ROS2 node pattern:

```python
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from rclpy.qos import QoSProfile, ReliabilityPolicy

class OffboardNode(Node):
    STREAM_HZ = 50  # must stay above 1 / COM_OF_LOSS_T (1/0.2 = 5 Hz) with margin

    def _publish_heartbeat(self):
        """Publish last waypoint to keep OFFBOARD heartbeat alive."""
        msg = self._current_setpoint()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.sp_pub.publish(msg)

    def _engage_offboard(self):
        """Stream for 0.5s then switch mode."""
        # PX4 requires setpoints streaming BEFORE mode switch
        for _ in range(int(0.5 * self.STREAM_HZ)):
            self._publish_heartbeat()
            time.sleep(1.0 / self.STREAM_HZ)
        self.set_mode_client.call(SetMode.Request(custom_mode='OFFBOARD'))

    def _emergency_stop(self):
        """Publish zero velocity then wait for failsafe."""
        stop = PositionTarget()
        stop.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        stop.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                          PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
                          PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                          PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
        stop.velocity.x = 0.0
        stop.velocity.y = 0.0
        stop.velocity.z = 0.0
        for _ in range(5):
            stop.header.stamp = self.get_clock().now().to_msg()
            self.sp_pub.publish(stop)
            time.sleep(0.02)
        # Then go silent — COM_OF_LOSS_T takes over
```

Key behaviours to implement:
- Always pre-stream before `SET_MODE OFFBOARD`.
- Keep a 50 Hz background timer that republishes last setpoint even if the mission is paused.
- On any exception / ROS2 node shutdown: publish 5× zero-velocity setpoints, then `disarm()`.
- Subscribe to `mavros/state` and monitor `nav_state != OFFBOARD` — if unexpected mode change,
  log and do not re-engage automatically.
- Watchdog: if last acknowledgement from FCU is >500ms old, trigger emergency stop.

---

## #8 — (Phase 2) Add DifferentialOffboardMode to the build pipeline  
**Priority: Low — future architecture migration**  
**Effort: 1-2 days**

Once Ackermann-style refactor is desired (nav_state dispatch, proper reset(), stopVehicle()):

1. The new architecture's `DifferentialOffboardMode.cpp` (in fork's DriveModes folder) already
   handles position, velocity, attitude, and body_rate in an `else if` chain. It needs P3+P4
   merged in for correctness.
2. `RoverDifferential.cpp` needs to become the Ackermann-style Run() with nav_state switch
   and `_was_armed` guard. Currently the v1.16.2 base RoverDifferential.cpp doesn't have this;
   the 2025 refactored one does (but it's dead code in the build).
3. CI workflow needs to `cp` the full new architecture instead of just the two files.

This is a non-trivial rebuild; do it as a separate branch, validate in SITL first.

---

## #9 — (Phase 2) DDS graduation gate for OFFBOARD  
**Priority: Low — DDS transition**

From `Hybride_Archi_Decision.md` Gate 2:

- [ ] Confirm `rover_speed_setpoint` + `rover_steering_setpoint` path works end-to-end with
      the 2025 refactored architecture (the DDS path bypasses `trajectory_setpoint` entirely
      and publishes directly to rover-specific topics).
- [ ] Verify failsafe (#18346) is resolved in the new architecture before enabling DDS control.
      The 2025 `DifferentialOffboardMode.cpp` has the same absence of timestamp validation and
      same `velocity_ned.norm()` sign bug — it needs P3+P4 before it is safe.
- [ ] The DDS `rover_speed_setpoint` / `rover_steering_setpoint` path does NOT go through
      `DifferentialPosControl` or `DifferentialVelControl`. It bypasses the OFFBOARD pure
      pursuit logic entirely. You own the speed controller on the ROS2 side.

---

## Summary — Ranked Action List

| # | Action | Severity | Blocking | Effort |
|---|--------|----------|----------|--------|
| 1 | Set `COM_OBL_RC_ACT=5`, `COM_OF_LOSS_T=0.2`, other params | Critical | Yes | 5 min |
| 2 | Patch VelControl: zero-velocity heading + speed sign (P3+P4) | High | Yes | 30 min |
| 3 | Patch PosControl: invalidate cached target on OFFBOARD exit (P1) | High | Yes | 30 min |
| 4 | Patch RoverDifferential: was_armed safe-stop on disarm (P2) | High | Yes | 20 min |
| 5 | Extend build_rover.yml to copy VelControl + PosControl + .hpp | Required | Yes | 15 min |
| 6 | SITL / bench test: verify Hold_mode stops rover within 300 ms | High | Yes | 2 h |
| 7 | Field test: ±3cm arc accuracy with position-mode OFFBOARD | High | Yes | 4 h |
| 8 | Write production OFFBOARD ROS2 node with watchdog + e-stop | Medium | No | 2 days |
| 9 | (Phase 2) DDS graduation: verify new architecture in SITL first | Low | No | 2 days |
