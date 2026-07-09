# PX4 Rover Mission Mode vs Companion RPP: Waypoint Stop & Spot-Turn Behaviour

This document traces the exact firmware source logic for PX4 v1.16.x rover mission-mode waypoint stopping and spot-turn behavior, then compares it with the companion RPP (`rpp_controller_node.py`) implementation.

---

## Table of Contents

1. [PX4 Firmware Pipeline Overview](#1-px4-firmware-pipeline-overview)
2. [Mission Waypoint Acceptance (Navigator)](#2-mission-waypoint-acceptance-navigator)
3. [Auto Mode: Waypoint-to-Position Setpoint](#3-auto-mode-waypoint-to-position-setpoint)
4. [Position Control: The DrivingState State Machine](#4-position-control-the-drivingstate-state-machine)
5. [Pure Pursuit Guidance](#5-pure-pursuit-guidance)
6. [Attitude Control (Yaw PID)](#6-attitude-control-yaw-pid)
7. [Rate Control (Differential Steering)](#7-rate-control-differential-steering)
8. [Speed Control](#8-speed-control)
9. [Spot-Turn Summary (Firmware)](#9-spot-turn-summary-firmware)
10. [RPP Companion Implementation](#10-rpp-companion-implementation)
11. [Key Differences](#11-key-differences)

---

## 1. PX4 Firmware Pipeline Overview

The PX4 rover differential module runs at **100 Hz** (`ScheduleOnInterval(10_ms)`) and chains these stages every cycle:

```
DifferentialPosControl::updatePosControl()    // mission/position -> speed+yaw setpoint
    |
    v
DifferentialVelControl::updateVelControl()    // speed setpoint -> throttle; heading gate
    |
    v
DifferentialAttControl::updateAttControl()    // yaw setpoint -> yaw rate setpoint (PID)
    |
    v
DifferentialRateControl::updateRateControl()  // yaw rate -> wheel speed diff (PID)
    |
    v
RoverDifferential::generateActuatorSetpoint() // inverse kinematics -> left/right motor
```

**Source:** `src/modules/rover_differential/RoverDifferential.cpp` `Run()`

```cpp
void RoverDifferential::Run()
{
    // ...
    _differential_pos_control.updatePosControl();
    _differential_vel_control.updateVelControl();
    _differential_att_control.updateAttControl();
    _differential_rate_control.updateRateControl();
    // ...
    generateActuatorSetpoint();
}
```

---

## 2. Mission Waypoint Acceptance (Navigator)

**Source:** `src/modules/navigator/mission_block.cpp` -- `is_mission_item_reached_or_completed()`

### Acceptance Logic for Rovers

For rovers (`VEHICLE_TYPE_ROVER`), waypoint acceptance is **purely distance-based** in the XY plane:

```cpp
if (dist_xy >= 0.0f && dist_xy <= _navigator->get_acceptance_radius()) {
    _waypoint_position_reached = true;
}
```

- `NAV_ACC_RAD` (default **10.0 m**, min 0.05 m) is the acceptance radius
- Rovers do **not** check yaw/heading at waypoints (unlike multicopter):
  ```cpp
  // consider yaw reached for non-rotary wing vehicles (such as fixed-wing)
  if (_navigator->get_vstatus()->vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
      _waypoint_yaw_reached = true;  // always true for rovers
  }
  ```

### Key Observation

PX4 mission mode has **no concept of "stopping at a waypoint to pivot"**. The rover drives straight through each waypoint. Whether it stops or slows depends entirely on the geometry of the **next** waypoint (the transition angle), computed downstream in auto mode.

---

## 3. Auto Mode: Waypoint-to-Position Setpoint

**Source:** `src/modules/rover_differential/DifferentialDriveModes/DifferentialAutoMode/DifferentialAutoMode.cpp`

### `autoControl()` converts the global waypoint triplet to a local `rover_position_setpoint`:

```cpp
void DifferentialAutoMode::autoControl()
{
    // ... project prev/curr/next waypoints to NED ...

    float waypoint_transition_angle = RoverControl::calcWaypointTransitionAngle(
        prev_wp_ned, curr_wp_ned, next_wp_ned);

    float cruising_speed = /* from mission or RO_SPEED_LIM */;

    rover_position_setpoint.arrival_speed = arrivalSpeed(
        cruising_speed, waypoint_transition_angle,
        _param_ro_speed_limit.get(),
        _param_rd_trans_drv_trn.get(),
        _param_ro_speed_red.get(), curr_wp_type);
}
```

### `arrivalSpeed()` -- The core decision: stop vs. slow-through

```cpp
float DifferentialAutoMode::arrivalSpeed(...)
{
    // Upcoming stop — arrival speed = 0
    if (!PX4_ISFINITE(waypoint_transition_angle)
        || waypoint_transition_angle < M_PI_F - trans_drv_trn  // hard turn
        || curr_wp_type == SETPOINT_TYPE_LAND
        || curr_wp_type == SETPOINT_TYPE_IDLE) {
        return 0.f;  // FULL STOP at this waypoint
    }

    // Smooth turn — reduce speed proportionally to turn angle
    if (speed_red > FLT_EPSILON) {
        float speed_reduction = math::constrain(
            speed_red * math::interpolate(M_PI_F - waypoint_transition_angle,
                                          0.f, M_PI_F, 0.f, 1.f), 0.f, 1.f);
        return max_speed * (1.f - speed_reduction);
    }

    return cruising_speed;
}
```

### `calcWaypointTransitionAngle()` -- Interior angle at the waypoint vertex

**Source:** `src/lib/rover_control/RoverControl.cpp`

```cpp
float calcWaypointTransitionAngle(Vector2f &prev_wp, Vector2f &curr_wp, Vector2f &next_wp)
{
    Vector2f curr_to_next = next_wp - curr_wp;
    Vector2f curr_to_prev = prev_wp - curr_wp;
    // ...
    float cosin = curr_to_prev.unit_or_zero() * curr_to_next.unit_or_zero();
    return acosf(math::constrain(cosin, -1.f, 1.f));
}
```

- Returns angle in `[0, pi]`
- `pi` = straight line (no turn)
- Small angle = sharp U-turn

### Stop Condition

```
arrival_speed == 0  (FULL STOP) when:
    transition_angle < (pi - RD_TRANS_DRV_TRN)
    i.e. the heading change is > RD_TRANS_DRV_TRN (default 0.1745 rad = ~10 degrees)
```

So the rover **stops fully at a waypoint** if the next leg requires a heading change greater than `RD_TRANS_DRV_TRN`.

---

## 4. Position Control: The DrivingState State Machine

**Source:** `src/modules/rover_differential/DifferentialPosControl/DifferentialPosControl.cpp`

### `updatePosControl()` -- Core logic

This is where the firmware decides between **driving** and **spot-turning**:

```cpp
void DifferentialPosControl::updatePosControl()
{
    // ... get target_waypoint_ned, start_ned, cruising_speed, arrival_speed ...

    float distance_to_target = (_target_waypoint_ned - _curr_pos_ned).norm();

    // If arrival_speed > 0 (smooth turn), shift the effective target outward
    if (_arrival_speed > FLT_EPSILON) {
        distance_to_target -= _param_nav_acc_rad.get();
    }

    if (distance_to_target > _param_nav_acc_rad.get() || _arrival_speed > FLT_EPSILON) {

        // Compute deceleration-limited speed for approach
        float speed_setpoint = _cruising_speed;
        if (_param_ro_decel_limit.get() > FLT_EPSILON && _param_ro_jerk_limit.get() > FLT_EPSILON) {
            speed_setpoint = math::min(
                math::trajectory::computeMaxSpeedFromDistance(
                    _param_ro_jerk_limit.get(),
                    _param_ro_decel_limit.get(),
                    distance_to_target,
                    fabsf(_arrival_speed)),
                _cruising_speed);
        }

        // Pure pursuit: compute target bearing
        float yaw_setpoint = PurePursuit::calcTargetBearing(
            ..., _target_waypoint_ned, _start_ned, _curr_pos_ned, fabsf(speed_setpoint));

        float heading_error = matrix::wrap_pi(yaw_setpoint - _vehicle_yaw);

        // === STATE MACHINE: DRIVING <-> SPOT_TURNING ===

        // Transition: DRIVING -> SPOT_TURNING
        if (_current_state == DrivingState::DRIVING
            && fabsf(heading_error) > _param_rd_trans_drv_trn.get()) {
            _current_state = DrivingState::SPOT_TURNING;
        }
        // Transition: SPOT_TURNING -> DRIVING
        else if (_current_state == DrivingState::SPOT_TURNING
                 && fabsf(heading_error) < _param_rd_trans_trn_drv.get()) {
            _current_state = DrivingState::DRIVING;
        }

        // === ACT ON STATE ===

        if (_current_state == DrivingState::SPOT_TURNING) {
            speed_setpoint = 0.f;  // ZERO forward speed during spot turn
        } else if (_param_ro_speed_red.get() > FLT_EPSILON) {
            // Driving: reduce speed proportional to heading error
            float speed_reduction = math::constrain(
                _param_ro_speed_red.get() * math::interpolate(
                    fabsf(heading_error), 0.f, M_PI_F, 0.f, 1.f), 0.f, 1.f);
            float max_speed = math::constrain(
                _param_ro_max_thr_speed.get() * (1.f - speed_reduction),
                0.f, _param_ro_max_thr_speed.get());
            speed_setpoint = math::constrain(speed_setpoint, -max_speed, max_speed);
        }

        // Publish speed_setpoint and yaw_setpoint downstream
        rover_speed_setpoint.speed_body_x = speed_setpoint;
        rover_attitude_setpoint.yaw_setpoint = yaw_setpoint;

    } else {
        // At waypoint, within acceptance radius with zero arrival speed
        rover_speed_setpoint.speed_body_x = 0.f;  // STOP
        rover_attitude_setpoint.yaw_setpoint = _vehicle_yaw;  // HOLD current heading

        if (!_stopped && fabsf(_vehicle_speed) < FLT_EPSILON) {
            _stopped = true;
            _target_waypoint_ned = _curr_pos_ned;
        }
    }
}
```

### The `DrivingState` Enum

```cpp
enum class DrivingState {
    SPOT_TURNING,  // Vehicle is turning on the spot (speed = 0)
    DRIVING        // Vehicle is driving forward
};
```

### Hysteresis Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `RD_TRANS_DRV_TRN` | 0.1745 rad (~10 degrees) | DRIVING -> SPOT_TURNING threshold |
| `RD_TRANS_TRN_DRV` | 0.0873 rad (~5 degrees) | SPOT_TURNING -> DRIVING threshold |

The hysteresis prevents oscillation: enter spot-turn at >10 deg heading error, exit only when <5 deg.

---

## 5. Pure Pursuit Guidance

**Source:** `src/lib/pure_pursuit/PurePursuit.cpp`

```cpp
float PurePursuit::calcTargetBearing(
    pure_pursuit_status_s &status,
    const float lookahead_gain,     // PP_LOOKAHD_GAIN
    const float lookahead_max,      // PP_LOOKAHD_MAX
    const float lookahead_min,      // PP_LOOKAHD_MIN
    const Vector2f &curr_wp_ned,
    const Vector2f &prev_wp_ned,
    const Vector2f &curr_pos_ned,
    const float vehicle_speed)
{
    float lookahead_distance = math::constrain(
        lookahead_gain * fabsf(vehicle_speed),
        lookahead_min, lookahead_max);

    // ... computes crosstrack error, finds intersection point ...

    // Three cases:
    // 1. Close to waypoint: target bearing directly to waypoint
    // 2. Crosstrack > lookahead: target closest point on path
    // 3. Regular: target intersection of lookahead circle with path line

    return target_bearing;  // desired heading in rad (NED)
}
```

This is classic pure pursuit. The lookahead distance scales with speed (`v * k`), clamped between min/max.

---

## 6. Attitude Control (Yaw PID)

**Source:** `src/modules/rover_differential/DifferentialAttControl/DifferentialAttControl.cpp`

```cpp
void DifferentialAttControl::updateAttControl()
{
    // ...
    // PID: yaw_error -> yaw_rate_setpoint
    _pid_yaw.setGains(_param_ro_yaw_p.get(), 0.f, 0.f);  // P-only (no I, no D)
    _pid_yaw.setIntegralLimit(_max_yaw_rate);
    _pid_yaw.setOutputLimit(_max_yaw_rate);

    // Slew rate limits yaw setpoint change rate
    _adjusted_yaw_setpoint.setSlewRate(_max_yaw_rate);  // = RO_YAW_RATE_LIM in rad/s

    float yaw_rate_setpoint = RoverControl::attitudeControl(
        _adjusted_yaw_setpoint, _pid_yaw, _max_yaw_rate,
        _vehicle_yaw, _yaw_setpoint, dt);
}
```

### `RoverControl::attitudeControl()` -- The P-controller with slew rate

**Source:** `src/lib/rover_control/RoverControl.cpp`

```cpp
float attitudeControl(SlewRateYaw<float> &adjusted_yaw_setpoint, PID &pid_yaw,
                      const float yaw_slew_rate, float vehicle_yaw,
                      float yaw_setpoint, const float dt)
{
    // Apply yaw slew rate (limits how fast yaw setpoint can change)
    adjusted_yaw_setpoint.setSlewRate(yaw_slew_rate);
    adjusted_yaw_setpoint.update(yaw_setpoint, dt);

    // P-controller: error = desired_yaw - actual_yaw
    pid_yaw.setSetpoint(wrap_pi(adjusted_yaw_setpoint.getState() - vehicle_yaw));
    return pid_yaw.update(0.f, dt);  // returns yaw_rate_setpoint
}
```

- **P-only** controller: `yaw_rate_setpoint = RO_YAW_P * heading_error`
- Output clamped to `RO_YAW_RATE_LIM` (deg/s -> rad/s)
- Yaw setpoint itself is slew-rate limited (prevents instantaneous heading jumps)

---

## 7. Rate Control (Differential Steering)

**Source:** `src/modules/rover_differential/DifferentialRateControl/DifferentialRateControl.cpp`

```cpp
void DifferentialRateControl::updateRateControl()
{
    // PID: yaw_rate_error -> normalized_speed_diff
    _pid_yaw_rate.setGains(_param_ro_yaw_rate_p.get(), _param_ro_yaw_rate_i.get(), 0.f);

    float speed_diff_normalized = RoverControl::rateControl(
        _adjusted_yaw_rate_setpoint, _pid_yaw_rate,
        yaw_rate_setpoint, _vehicle_yaw_rate,
        _param_ro_max_thr_speed.get(),
        _param_ro_yaw_rate_corr.get(),
        _param_ro_yaw_accel_limit.get() * M_DEG_TO_RAD_F,
        _param_ro_yaw_decel_limit.get() * M_DEG_TO_RAD_F,
        _param_rd_wheel_track.get(), dt);

    rover_steering_setpoint.normalized_steering_setpoint = speed_diff_normalized;
}
```

### `RoverControl::rateControl()` -- Feedforward + PID

```cpp
float rateControl(...)
{
    // Apply accel/decel slew rate to yaw rate setpoint
    adjusted_yaw_rate_setpoint.setSlewRate(max_yaw_accel);
    adjusted_yaw_rate_setpoint.update(yaw_rate_setpoint, dt);

    // Feedforward: yaw_rate -> speed difference
    // speed_diff = yaw_rate * wheel_track / 2
    float speed_diff = (adjusted_yaw_rate_setpoint.getState() * wheel_track / 2.f) * yaw_rate_corr;
    float speed_diff_normalized = math::interpolate(speed_diff, -max_thr_speed, max_thr_speed, -1.f, 1.f);

    // PID feedback
    speed_diff_normalized += pid_yaw_rate.update(vehicle_yaw_rate, dt);

    return math::constrain(speed_diff_normalized, -1.f, 1.f);
}
```

### Inverse Kinematics (final actuator output)

**Source:** `RoverDifferential.cpp` -- `computeInverseKinematics()`

```cpp
Vector2f RoverDifferential::computeInverseKinematics(float throttle_body_x, float speed_diff_normalized)
{
    // Prioritize yaw if total exceeds 1.0
    if (fabsf(throttle_body_x) + fabsf(speed_diff_normalized) > 1.0f) {
        float excess = fabsf(max_motor_command - 1.0f);
        throttle_body_x -= sign(throttle_body_x) * excess;
    }
    // control[0] = left motor, control[1] = right motor
    return {throttle_body_x + speed_diff_normalized,
            throttle_body_x - speed_diff_normalized};
}
```

During spot turn (`throttle_body_x = 0`): left = `+diff`, right = `-diff` (wheels spin opposite).

---

## 8. Speed Control

**Source:** `src/modules/rover_differential/DifferentialVelControl/DifferentialVelControl.cpp`

```cpp
void DifferentialVelControl::generateAttitudeAndThrottleSetpoint()
{
    // Read desired speed and bearing
    float speed_body_x_setpoint = _differential_velocity_setpoint.speed;
    float bearing = _differential_velocity_setpoint.bearing;

    // === SAME STATE MACHINE AS POSITION CONTROL ===
    float heading_error = wrap_pi(bearing - _vehicle_yaw);
    if (_current_state == DRIVING && fabsf(heading_error) > _param_rd_trans_drv_trn.get()) {
        _current_state = SPOT_TURNING;
    } else if (_current_state == SPOT_TURNING && fabsf(heading_error) < _param_rd_trans_trn_drv.get()) {
        _current_state = DRIVING;
    }

    float speed_body_x_setpoint = 0.f;
    if (_current_state == DRIVING) {
        speed_body_x_setpoint = constrain(_diff_vel_setpoint.speed, ...);
        // ... adjust for steering saturation ...
    }

    // Speed PID -> throttle
    float throttle = RoverControl::speedControl(
        _speed_setpoint, _pid_speed, speed_body_x_setpoint, _vehicle_speed_body_x,
        _param_ro_accel_limit.get(), _param_ro_decel_limit.get(),
        _param_ro_max_thr_speed.get(), _dt);
}
```

`RoverControl::speedControl()` uses feedforward (`speed / max_thr_speed`) + PID (`RO_SPEED_P`, `RO_SPEED_I`) with accel/decel slew rate limiting.

---

## 9. Spot-Turn Summary (Firmware)

The firmware spot-turn is a **two-layer state machine** with no explicit "pivot then drive" sequence:

```
While driving toward waypoint:
    1. Pure Pursuit computes yaw_setpoint (bearing to lookahead point)
    2. heading_error = yaw_setpoint - vehicle_yaw

    3. If |heading_error| > RD_TRANS_DRV_TRN (~10 deg):
        -> Switch to SPOT_TURNING
        -> speed_setpoint = 0 (forward throttle zeroed)
        -> Yaw PID drives yaw_rate_setpoint = RO_YAW_P * heading_error
        -> Rate PID drives wheel speed differential (opposite spin)
        -> Rover rotates in place

    4. If |heading_error| < RD_TRANS_TRN_DRV (~5 deg):
        -> Switch back to DRIVING
        -> speed_setpoint restored (cruising or deceleration-limited)
        -> Rover drives forward along path
```

### Critical properties of firmware spot-turn:

1. **Seamless/reactive**: No distinct "stop phase" then "pivot phase". The state machine reacts to heading error every cycle (100 Hz).
2. **Hysteresis**: 10 deg to enter, 5 deg to exit -- prevents oscillation.
3. **Speed = 0 during turn**: Forward throttle is zeroed, but differential yaw rate is still active.
4. **No position hold during turn**: The rover does not try to hold a specific position while spot-turning. It just zeroes forward speed and rotates.
5. **Arrival speed = 0 means stop**: When `arrivalSpeed()` returns 0, the position controller commands `speed=0, yaw=hold` once within acceptance radius.
6. **Yaw slew rate**: The yaw setpoint itself is rate-limited (`RO_YAW_RATE_LIM`), so large heading changes are applied as a ramp, not a step.

---

## 10. RPP Companion Implementation

**Source:** `src/rpp_controller_node.py`

### Architecture

The RPP runs on the Jetson companion at **50 Hz** and publishes NED velocity commands via MAVROS `/mavros/setpoint_velocity/cmd_vel`. It does NOT use PX4 mission mode at all -- it uses OFFBOARD velocity mode.

### Segment State Machine

```python
class SegmentStateCode(IntEnum):
    INACTIVE = 0
    TRACK_SEGMENT = 1         # Normal RPP tracking along a straight segment
    PRE_CORNER_SLOWDOWN = 2   # Decelerating toward a corner
    CORNER_ALIGN = 3          # Pivoting in place toward next segment heading
    DONE = 4                  # Mission complete
    CORNER_STOP = 5           # Holding zero velocity until physically stopped
```

### Corner Handling (Stop-and-Pivot Sequence)

The RPP uses an explicit **multi-phase stop-and-pivot** at hard corners:

```
Phase 1: PRE_CORNER_SLOWDOWN
    - When dist_to_corner < segment_slowdown_dist AND corner_angle >= threshold
    - Speed = max(endpoint_approach_speed, max_v * (dist/slowdown))

Phase 2: CORNER_STOP (within acceptance radius)
    - Active braking: _corner_hold_velocity() commands small velocity toward
      the corner point + damping from measured EKF velocity
    - Must satisfy _corner_stop_satisfied():
        * position_ok (inside corner_position_tolerance_m)
        * measured speed < segment_stop_speed_threshold
        * yaw rate < segment_stop_yaw_rate_threshold
        * All conditions held for segment_stop_dwell_s
    - Issues a StopCertificate (proven physical stop)

Phase 3: CORNER_ALIGN (pivot in place)
    - Commands _corner_pivot_velocity():
        * Small velocity vector (corner_speed ~0.08 m/s) pointed at exit heading
        * Clamped to +/- 75 deg forward cone (prevents PX4 reverse-flip: BUG-T3)
        * Damped as heading_err shrinks inside segment_pivot_damp_start_deg
    - Release gates (ALL must pass):
        * heading_err < segment_heading_tolerance_deg (default 2 deg)
        * yaw rate < segment_stop_yaw_rate_threshold
        * measured speed < segment_align_speed_threshold
        * Held for segment_align_settle_s (default 0.10 s)
    - Issues an AlignmentCertificate

Phase 4: TRACK_SEGMENT (resume)
    - Segment index incremented, normal RPP lookahead tracking resumes
```

### Run-Boundary Pivot

Between "runs" (entities like spray-flagged sections), the RPP uses `_run_alignment_hold()`:

- Same stop-certify-then-pivot sequence
- Triggered by heading change >= `segment_corner_threshold_deg` between runs
- Also triggered by runtime entry-to-mark boundaries (spray transitions)

### Final Endpoint Stop

Uses `_completion_settle_satisfied()`:
- Position within `xy_goal_tolerance`
- Measured speed < threshold for `segment_stop_dwell_s`
- Issues `FinalStopCertificate` -> `DONE` state

### Certificate System

The RPP implements a **certificate-based proven stop system**:

```python
@dataclass
class StopCertificate:
    reason: StopReason           # INTRA_RUN_CORNER, RUN_BOUNDARY, etc.
    position_error_m: float
    measured_speed_m_s: float
    yaw_rate_rad_s: float
    dwell_s: float               # how long conditions held
    valid: bool

@dataclass
class AlignmentCertificate:
    target_heading_rad: float
    heading_error_rad: float
    # ... same proven-stop fields

@dataclass
class FinalStopCertificate:
    # ... same proven-stop fields
```

Each certificate is validated against the current target position/segment before release.

### Firmware-Aware Pivot (Key Hack)

Since PX4 OFFBOARD velocity mode derives heading from velocity-vector bearing (not from yaw_rate field), the RPP commands a **small velocity vector** at the exit heading instead of a yaw rate:

```python
def _corner_pivot_velocity(self, yaw_ned, heading_err, corner_speed):
    max_offset = self._CORNER_MAX_BEARING_OFFSET_RAD  # ~75 deg
    step = self._clamp(heading_err, -max_offset, max_offset)
    cmd_bearing = yaw_ned + step

    # Damp speed as heading converges (prevents overshoot)
    damp_start = radians(segment_pivot_damp_start_deg)
    scale = clamp(abs(heading_err) / damp_start, 0.0, 1.0)
    corner_speed = max(damp_floor, corner_speed * scale)

    return corner_speed * cos(cmd_bearing), corner_speed * sin(cmd_bearing)
```

This makes PX4's firmware see a large heading error -> enter `SPOT_TURNING` (speed=0) -> rotate the short way -> exit `SPOT_TURNING` when aligned.

---

## 11. Key Differences

| Aspect | PX4 Firmware Mission Mode | RPP Companion (OFFBOARD) |
|--------|--------------------------|--------------------------|
| **Where logic runs** | FCU firmware (CubeOrangePlus) | Jetson companion (Python, 50 Hz) |
| **Control mode** | AUTO mission (position setpoints) | OFFBOARD velocity (TwistStamped) |
| **Path representation** | Discrete waypoints (prev/curr/next triplet) | Dense polyline path (segment-by-segment) |
| **Guidance algorithm** | Pure Pursuit (circle-line intersection) | RPP (lookahead point on segment, with crosstrack lookahead gain) |
| **Waypoint acceptance** | Distance to waypoint <= NAV_ACC_RAD (default 10m) | Per-segment: dist to corner <= segment_corner_acceptance_radius |
| **Stop at waypoint decision** | `arrivalSpeed() == 0` when transition angle > RD_TRANS_DRV_TRN (~10 deg) | Always stops at hard corners (>= segment_corner_threshold_deg) |
| **Spot-turn trigger** | Heading error > RD_TRANS_DRV_TRN (~10 deg), reactive every cycle | Explicit phase transition after physical stop certified |
| **Spot-turn execution** | Yaw PID (P-only) -> yaw_rate -> wheel diff. Forward speed = 0. | Velocity-vector hack: small velocity at exit bearing, clamped to forward cone |
| **Stop confirmation** | `_stopped` flag when `fabsf(_vehicle_speed) < FLT_EPSILON` (simple) | Multi-condition certificate: position + speed + yaw-rate + dwell time |
| **Position hold during pivot** | None (just zeroes speed, lets physics settle) | Active `_corner_hold_velocity()` braking toward exact corner point |
| **Hysteresis** | 10 deg enter / 5 deg exit (configurable params) | Fixed heading tolerance (2 deg) + settle dwell (0.10 s) |
| **Yaw rate control** | Direct PID on yaw rate with feedforward (wheel_track geometry) | No direct yaw rate (inert in OFFBOARD velocity mode) |
| **Speed during corner approach** | Deceleration profile from jerk/decel limits OR speed reduction by heading error | Linear slowdown: `speed = max_v * (dist/slowdown_dist)` |
| **Smooth corner handling** | arrival_speed > 0, no stop, speed reduced by RO_SPEED_RED | Collinear/tangent junctions (sub-threshold angle) pass through without stopping |
| **State machine** | 2-state: DRIVING / SPOT_TURNING (hysteresis) | 5-state: TRACK / PRE_CORNER_SLOWDOWN / CORNER_STOP / CORNER_ALIGN / DONE |
| **Certificate/audit trail** | None | StopCertificate, AlignmentCertificate, FinalStopCertificate with telemetry |
| **Pivot timeout** | None (reactive, stays in SPOT_TURNING until heading converges) | Angle-aware watchdog: `budget = spinup_margin + angle/nominal_rate` |
| **Forward cone protection** | N/A (firmware handles reverse natively) | `_clamp_velocity_to_forward_cone()` prevents BUG-T3 wrong-direction turn |
| **Path conditioning** | N/A (waypoints are raw) | Run splitting, connector absorption, collinear merging, corner detection |

### Why the RPP is more complex

1. **OFFBOARD velocity mode limitation**: PX4's firmware ignores `yaw_rate` in velocity OFFBOARD. The RPP must fake a pivot by commanding a velocity vector, which requires the forward-cone clamp and damping logic.

2. **Precision marking requirement**: The rover paints lines, so sub-2cm accuracy at corners is required. PX4 mission mode with 10m acceptance radius is far too coarse.

3. **Physical stop certification**: The RPP must prove (via certificates backed by fresh velocity telemetry) that the rover is truly stopped before pivoting. PX4 just checks `speed ~= 0`.

4. **Spray coordination**: The RPP coordinates spray on/off with path segments, requiring explicit run boundaries and stop-pivot at transitions.

5. **Multiple path profiles**: The RPP supports `segment` (stop-pivot), `smooth` (continuous pure pursuit), and `auto` (per-entity split) profiles. PX4 mission mode only has the waypoint-triplet model.
