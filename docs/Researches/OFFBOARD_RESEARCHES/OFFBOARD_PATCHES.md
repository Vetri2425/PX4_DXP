# OFFBOARD Patch Specifications

Four targeted patches for the v1.16.2 rover_differential OFFBOARD path.
Each patch is specified as "file → function → what to add/change and why".
No full diffs are written here — that's for `OFFBOARD_ACTION_PLAN.md` task tracking.

All paths are relative to the upstream v1.16.2 checkout that the CI workflow uses.
Files you copy from the fork must include these changes before being uploaded to the fork.

---

## P1 — Reset cached position setpoint on OFFBOARD exit

**Priority:** Critical — directly closes the #18346 runaway window.

**Root cause recap.**
`DifferentialPosControl::generatePositionSetpoint()` writes a finite `rover_position_setpoint`
during OFFBOARD. When OFFBOARD ends, nothing clears it. A subsequent mode transition that
reaches `goToPositionMode()` finds a valid cached target and drives toward it.

**File:**
`src/modules/rover_differential/DifferentialPosControl/DifferentialPosControl.cpp`

**Function:**
`DifferentialPosControl::updatePosControl()` — the top-level function called every 10 ms.

**What to track.**
Add a member variable `bool _was_offboard{false}` to `DifferentialPosControl.hpp`.

**What to add inside `updatePosControl()`, right after `updateSubscriptions()`:**

```cpp
// Detect OFFBOARD falling edge and invalidate the cached position setpoint
const bool offboard_active = _vehicle_control_mode.flag_control_position_enabled
                             && _vehicle_control_mode.flag_control_offboard_enabled;

if (_was_offboard && !offboard_active) {
    // OFFBOARD just ended — erase cached target so goToPositionMode() is inert
    _rover_position_setpoint.position_ned[0] = NAN;
    _rover_position_setpoint.position_ned[1] = NAN;
    _curr_wp_ned     = Vector2f{NAN, NAN};
    _prev_wp_ned     = Vector2f{NAN, NAN};
    _course_control  = false;
}
_was_offboard = offboard_active;
```

**Why this is the right location.**
`updateSubscriptions()` just refreshed `_vehicle_control_mode` from uORB. The check runs
before `generatePositionSetpoint()` or `generateVelocitySetpoint()` so neither function sees
stale data in the same tick.

**Side effects.**
On POSCTL fallback, `goToPositionMode()` will now find NaN and skip (its `distance_to_target`
will be NaN → the `if (distance > acceptance_radius)` check fails → function returns without
publishing). POSCTL falls through to `manualPositionMode()` which is the right behaviour for
a joystick-less rover (reads stick → all zeros → zero-speed setpoint → stop).

**Also add the equivalent invalidation on disarm:**

```cpp
} else if (!_vehicle_control_mode.flag_armed) {
    _rover_position_setpoint.position_ned[0] = NAN;
    _rover_position_setpoint.position_ned[1] = NAN;
}
```

Place this inside the outer `if (flag_control_position_enabled && flag_armed)` else-branch
that already exists implicitly (i.e. when the function returns early).

---

## P2 — Safe-stop on mode change in RoverDifferential::Run()

**Priority:** High — closes the missing `was_armed` guard vs. Ackermann/Mecanum, but less
critical than initially stated because sub-controllers already self-reset their PIDs.

**Root cause recap.**
`RoverDifferential::Run()` never calls `stopVehicle()` or publishes a zero actuator command
on disarm. When the vehicle disarms or when OFFBOARD exits, the last published `actuator_motors`
values linger for one more 10 ms cycle before the `flag_armed` gate blocks the setpoint path.
On Ackermann and Mecanum, a separate `_was_armed` guard calls `stopVehicle()` and `reset()`
immediately. This is a structural pattern difference, not a safety gap (the sub-controllers
already reset their own PIDs in their else-branches), but it's cleaner to match the pattern.

**File:**
`src/modules/rover_differential/RoverDifferential.cpp` — **this is the file you patch in the fork**.

**Function:**
`RoverDifferential::Run()`

**Add to `RoverDifferential.hpp` (private members):**

```cpp
bool _was_armed{false};
bool _was_offboard{false};
```

**Replace the existing `Run()` body's control-mode and actuator sections with:**

```cpp
void RoverDifferential::Run()
{
    if (_parameter_update_sub.updated()) {
        updateParams();
    }

    const hrt_abstime timestamp_prev = _timestamp;
    _timestamp = hrt_absolute_time();
    _dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

    _differential_pos_control.updatePosControl();
    _differential_vel_control.updateVelControl();
    _differential_att_control.updateAttControl();
    _differential_rate_control.updateRateControl();

    if (_vehicle_control_mode_sub.updated()) {
        _vehicle_control_mode_sub.copy(&_vehicle_control_mode);
    }

    const bool full_manual_mode_enabled =
        _vehicle_control_mode.flag_control_manual_enabled
        && !_vehicle_control_mode.flag_control_position_enabled
        && !_vehicle_control_mode.flag_control_attitude_enabled
        && !_vehicle_control_mode.flag_control_rates_enabled;

    if (full_manual_mode_enabled) {
        generateSteeringAndThrottleSetpoint();
    }

    if (_vehicle_control_mode.flag_armed) {
        _was_armed = true;
        generateActuatorSetpoint();

    } else if (_was_armed) {
        // Just disarmed — publish a zero actuator command to cut motors cleanly
        actuator_motors_s actuator_motors{};
        actuator_motors.reversible_flags = _param_r_rev.get();
        actuator_motors.timestamp = _timestamp;
        actuator_motors.control[0] = 0.f;
        actuator_motors.control[1] = 0.f;
        _actuator_motors_pub.publish(actuator_motors);
        // Reset slew-rate so next arm starts from zero
        _throttle_body_x_setpoint.setForcedValue(0.f);
        _was_armed = false;
    }
}
```

**Note on IK sign.** The existing patched `computeInverseKinematics()` already has the
correct left/right sign for Sabertooth wiring (`control[0]=throttle+diff,
control[1]=throttle-diff`). Keep that unchanged.

---

## P3 — Fix velocity sign for reverse motion

**Priority:** Medium (required if the mission plan ever needs the rover to back up).

**Root cause recap.**
`differential_velocity_setpoint.speed = velocity_in_local_frame.norm()` — always positive.
Reverse motion is silently discarded. The rover then tries to spot-turn 180° and go forward,
which takes 1-3 seconds and destroys arc continuity.

**File:**
`src/modules/rover_differential/DifferentialVelControl/DifferentialVelControl.cpp`

**Function:**
`DifferentialVelControl::generateVelocitySetpoint()`

**Replace the speed/bearing calculation block:**

```cpp
// BEFORE (v1.16.2 stock):
differential_velocity_setpoint.speed   = velocity_in_local_frame.norm();
differential_velocity_setpoint.bearing = atan2f(velocity_in_local_frame(1),
                                                velocity_in_local_frame(0));

// AFTER (signed speed, correct reverse):
//
// Project the NED velocity into the body frame using the current yaw.
// Forward component is body-x (cos(yaw), sin(yaw)).
// The bearing (heading the rover should face) is always the forward direction of the
// NED vector, but we sign speed as negative when the rover is commanded to go
// backward (NED vector points behind current heading).
const float bearing = atan2f(velocity_in_local_frame(1), velocity_in_local_frame(0));
const float mag     = velocity_in_local_frame.norm();
// dot product of NED velocity with current body-x unit vector
const float fwd_component = velocity_in_local_frame(0) * cosf(_vehicle_yaw)
                          + velocity_in_local_frame(1) * sinf(_vehicle_yaw);
// sign: positive if velocity is generally forward, negative if backward
const float speed_sign = (fwd_component >= 0.f) ? 1.f : -1.f;
differential_velocity_setpoint.speed   = speed_sign * mag;
differential_velocity_setpoint.bearing = bearing;
```

**Downstream impact.**
`DifferentialVelControl::generateAttitudeAndThrottleSetpoint()` already passes
`_differential_velocity_setpoint.speed` through `constrain(speed, -RO_SPEED_LIM,
RO_SPEED_LIM)` and `RoverControl::speedControl()` which handles negative speeds via the slew
rate and PID. `DifferentialRateControl` and `DifferentialAttControl` are heading-only
controllers that handle any sign of speed. No further changes needed.

**MAVROS2 convention.**
`cmd_vel.linear.x` negative = reverse. MAVROS2 setpoint_velocity plugin converts that
directly to `target_local_ned.vx` which is already body-aligned in `MAV_FRAME_BODY_NED` or
NED-aligned in `MAV_FRAME_LOCAL_NED`. Either works with this patch.

---

## P4 — Fix heading at zero velocity (North-snap bug)

**Priority:** High (visible misbehaviour on every commanded stop).

**Root cause recap.**
`atan2f(0, 0) = 0` → bearing = 0 rad = North. AttControl then yaws the rover to face North
before stopping. Manifests as a spurious rotation on every zero-velocity setpoint.

**File:**
`src/modules/rover_differential/DifferentialVelControl/DifferentialVelControl.cpp`

**Function:**
`DifferentialVelControl::generateVelocitySetpoint()`

**Replace the publication block:**

```cpp
// BEFORE:
differential_velocity_setpoint.bearing = atan2f(velocity_in_local_frame(1),
                                                velocity_in_local_frame(0));

// AFTER:
constexpr float ZERO_VEL_THRESHOLD = 0.01f; // m/s — below this, freeze heading
if (velocity_in_local_frame.norm() < ZERO_VEL_THRESHOLD) {
    // Velocity is commanded to zero — hold current heading, don't snap to North
    differential_velocity_setpoint.speed   = 0.f;
    differential_velocity_setpoint.bearing = _vehicle_yaw; // keep current yaw
} else {
    differential_velocity_setpoint.speed   = velocity_in_local_frame.norm();
    differential_velocity_setpoint.bearing = atan2f(velocity_in_local_frame(1),
                                                    velocity_in_local_frame(0));
}
```

**Note.** If you also apply P3, combine both into a single rewrite of the block:

```cpp
constexpr float ZERO_VEL_THRESHOLD = 0.01f;
const float mag = velocity_in_local_frame.norm();

if (mag < ZERO_VEL_THRESHOLD) {
    differential_velocity_setpoint.speed   = 0.f;
    differential_velocity_setpoint.bearing = _vehicle_yaw;
} else {
    const float bearing = atan2f(velocity_in_local_frame(1),
                                 velocity_in_local_frame(0));
    const float fwd_component = velocity_in_local_frame(0) * cosf(_vehicle_yaw)
                              + velocity_in_local_frame(1) * sinf(_vehicle_yaw);
    const float speed_sign = (fwd_component >= 0.f) ? 1.f : -1.f;
    differential_velocity_setpoint.speed   = speed_sign * mag;
    differential_velocity_setpoint.bearing = bearing;
}
```

This combined rewrite fixes both P3 and P4 in one block — the most practical approach.

---

## Fork Patch Delivery

These patches must be in the files you upload to the `Vetri2425/PX4-Autopilot` fork so the
CI workflow picks them up on the next `cp` run:

| File in fork | Patches applied |
|---|---|
| `src/modules/rover_differential/RoverDifferential.cpp` | P2 |
| `src/modules/rover_differential/RoverDifferential.hpp` | P2 (add `_was_armed`, `_was_offboard`) |
| `src/modules/rover_differential/DifferentialVelControl/DifferentialVelControl.cpp` | P3 + P4 combined |
| `src/modules/rover_differential/DifferentialPosControl/DifferentialPosControl.cpp` | P1 |
| `src/modules/rover_differential/DifferentialPosControl/DifferentialPosControl.hpp` | P1 (add `_was_offboard`) |

The CI workflow currently only `cp`s `RoverDifferential.cpp/.hpp` and `module.yaml`.
You must extend `build_rover.yml` to also copy the VelControl and PosControl files.
See the exact `cp` commands needed in `OFFBOARD_ACTION_PLAN.md` item #3.
