Now I have all the data I need. Let me compose the full audit.

---

# PX4 v1.16.2 Rover Differential OFFBOARD Safety Audit

## A. Safety-Critical

### 1. #18346 Root Cause — **CONFIRMED**

**Finding:** The stale-setpoint propagation chain is real and runs through five modules.

The full data path after OFFBOARD → Position failsafe:

1. **`offboardCheck.cpp:56-57`** — After `COM_OF_LOSS_T` expires, sets `offboard_control_signal_lost = true`
2. **`failsafe.cpp:696`** — `checkModeFallback()` calls `fromOffboardLossActParam()` → `Action::FallbackPosCtrl`, `user_intended_mode = NAVIGATION_STATE_POSCTL`
3. **`framework.cpp:675`** — `modeFromAction()` maps `FallbackPosCtrl` → `NAVIGATION_STATE_POSCTL`
4. **`control_mode.cpp:75-83`** — POSCTL sets `flag_control_position_enabled`, `flag_control_velocity_enabled`, `flag_control_attitude_enabled`, `flag_control_rates_enabled` all = **true**
5. **`RoverDifferential.cpp:76-79`** — All four sub-controllers run **unconditionally every 100Hz cycle**:
   ```cpp
   _differential_pos_control.updatePosControl();
   _differential_vel_control.updateVelControl();
   _differential_att_control.updateAttControl();
   _differential_rate_control.updateRateControl();
   ```

6. **`DifferentialPosControl.cpp:145-158`** — `updateSubscriptions()` only sets `_target_waypoint_ned` when `_rover_position_setpoint_sub.updated()` returns true. After offboard exits, **no one publishes this topic**, so the stale waypoint persists. The `_stopped` flag was set to `false` on the last offboard position update (line 158). Since the rover hasn't reached the stale waypoint yet, `distance_to_target > _param_nav_acc_rad`, and the position controller enters its driving branch (line 66) and publishes **non-zero `rover_speed_setpoint`** and **non-current `rover_attitude_setpoint`** toward the stale waypoint.

7. **`DifferentialSpeedControl.cpp:66-74`** — Picks up the freshly published `rover_speed_setpoint` from step 6, computes throttle via PID + slew rate, publishes `rover_throttle_setpoint` with **non-zero `throttle_body_x`**.

8. **`RoverDifferential.cpp:114-118`** — `generateActuatorSetpoint()` sees the updated `rover_throttle_setpoint`, runs throttleControl + IK → motor commands → **rover drives toward stale offboard waypoint**.

**Critical gap:** There is **zero** mode-transition reset anywhere. No sub-controller checks `_vehicle_control_mode` to zero its internal setpoint state. The only mode check in the entire `RoverDifferential` is `full_manual_mode_enabled` (line 82), which gates only the RC stick path.

**Severity:** **Critical** — runaway potential on every offboard loss event with default `COM_OBL_RC_ACT=0`.

**Action:** Must patch. See Q13.

---

### 2. Safest `COM_OBL_RC_ACT` for Ground Rover

**Finding:** The enum in [failsafe.h:249-257](/D:/Vetri/3WD_GCS/PX4-Autopilot/src/modules/commander/failsafe/failsafe.h:249) defines eight values. I analyzed each:

| Value | Mode | Maps to | Rover-safe? |
|-------|------|---------|-------------|
| 0 | Position_mode | `NAVIGATION_STATE_POSCTL` | **No** — triggers #18346 runaway |
| 1 | Altitude_mode | `NAVIGATION_STATE_ALTCTL` | N/A for rover |
| 2 | Stabilized | `NAVIGATION_STATE_STAB` | Falls through if position invalid; no position driving but no active braking either |
| 3 | Return_mode | `NAVIGATION_STATE_AUTO_RTL` | Requires mission — rover may drive to home |
| 4 | Land_mode | `NAVIGATION_STATE_AUTO_LAND` | Not meaningful for rover |
| 5 | Hold_mode | `NAVIGATION_STATE_AUTO_LOITER` | **Likely safe** — auto mode uses navigator, which should command hold-at-current-position. The auto mode publishes a new `rover_position_setpoint` at current position, overwriting any stale offboard value. |
| 6 | Terminate | `NAVIGATION_STATE_TERMINATION` | Kills all controllers, requires reboot |
| 7 | Disarm | N/A (action, not mode) | Stops rover dead, requires re-arm |

**Recommendation flow:**
1. **For production before patch:** `COM_OBL_RC_ACT=7` (Disarm). Stops dead. Operator re-arms after investigation. This is the only option that **guarantees** the rover cannot drive on stale setpoints.
2. **For production after #18346 patch:** `COM_OBL_RC_ACT=5` (Hold). Rover holds position, operator can resume.
3. **If GPS is lost:** The cascading failsafe at [failsafe.cpp:703-707] checks if RC is also lost. If both offboard + RC lost → falls through to `NAV_RCL_ACT`. Set `NAV_RCL_ACT` appropriately.

**Severity:** **Critical** — wrong choice here means the rover drives away on failsafe.

**Action:** Set `COM_OBL_RC_ACT=7` for all pre-patch testing. Document as temporary until #18346 is patched.

---

### 3. Mode Transition Setpoint Reset — **CONFIRMED ABSENT**

**Finding:** I traced every path in `RoverDifferential::Run()` and all sub-controllers. There is no code anywhere that:
- Zeros `_speed_setpoint` in `DifferentialSpeedControl` on mode change
- Zeros `_yaw_setpoint` in `DifferentialAttControl` on mode change
- Invalidates `_target_waypoint_ned` in `DifferentialPosControl` on mode change
- Resets `_yaw_rate_setpoint` in `DifferentialRateControl` on mode change
- Checks `_vehicle_control_mode.flag_control_offboard_enabled` in any sub-controller

The `RoverDifferential` class subscribes to `vehicle_control_mode` (line 108 of `.hpp`) and updates it (line 78 of `.cpp`), but **never passes it to any sub-controller** or uses it for resetting. The mode is only used for the manual mode RC path.

**Severity:** **Critical** — this is the architectural gap behind #18346.

**Action:** Must patch. See Q13.

---

### 4. `COM_OF_LOSS_T` Window (Default 1.0s)

**Finding:** At [offboardCheck.cpp:56-57]:
```cpp
bool data_is_recent = hrt_absolute_time() < offboard_control_mode.timestamp
    + static_cast<hrt_abstime>(_param_com_of_loss_t.get() * 1_s);
```

At 0.4 m/s, the default 1.0s window means **40cm of uncontrolled motion** before failsafe even triggers. This matters because:
- The failsafe detection is based on `offboard_control_mode` timestamp, not `trajectory_setpoint` timestamp
- `offboard_control_mode` is published alongside `trajectory_setpoint` in `mavlink_receiver.cpp:1138-1142`
- Both age at the same rate

**Can shorten?** Yes. Parameter `COM_OF_LOSS_T` can be set as low as 0.0s in theory, but practical minimum depends on GCS update rate:
- MAVROS2 at 50Hz → 20ms nominal interval → 0.1s gives 5 missed messages before trigger
- At 0.1s: ~4cm of motion — aggressive but feasible with reliable telemetry
- Recommended: **0.3s** (~12cm) as a balanced starting point

**Severity:** **High** — 40cm is significant for a ±3cm accuracy system. Combined with #18346, the 1.0s default means the rover gets a full second of stale-command driving plus however long it takes for position mode to "arrive" at the stale waypoint.

**Action:** Set `COM_OF_LOSS_T=0.3` as starting value. Tune downward to 0.15s if telemetry link proves reliable.

---

## B. Control Path Correctness

### 5. Velocity Mode Sign Bug — **CONFIRMED**

**Finding:** [DifferentialOffboardMode.cpp:76]:
```cpp
rover_speed_setpoint.speed_body_x = velocity_ned.norm();
```

`norm()` is always ≥0. The rover can never reverse in velocity offboard mode. For a marking rover that needs to back up at arc endpoints or reposition, this is a **showstopper**.

The `rover_speed_setpoint.speed_body_x` field is documented as body-frame speed — positive forward, negative backward. But `velocity_ned` is in NED frame (MAVROS2 default). The sign is lost in the `norm()` operation.

**Correct implementation:** Project NED velocity onto body x-axis using current vehicle yaw:
```cpp
speed_body_x = velocity_ned(0) * cosf(yaw) + velocity_ned(1) * sinf(yaw);
```
This requires subscribing to `vehicle_attitude` in `DifferentialOffboardMode`. The sign naturally handles forward/reverse.

**Severity:** **High** — prevents any reverse motion in velocity mode. Workaround exists (use position mode for reversing maneuvers) but is awkward for arc endpoint handling.

**Action:** Must patch. See Q14.

---

### 6. Heading at Zero Velocity — **CONFIRMED**

**Finding:** [DifferentialOffboardMode.cpp:80]:
```cpp
rover_attitude_setpoint.yaw_setpoint = atan2f(velocity_ned(1), velocity_ned(0));
```

When velocity is (0,0), `atan2f(0,0)` returns 0.0 (North). This means **every time the rover stops**, the attitude controller receives `yaw_setpoint = 0` and tries to turn the rover to face North. For a marking rover, this would create unwanted pivot arcs at every stop.

The `DifferentialPosControl` handles this correctly at [DifferentialPosControl.cpp:85-89] — when the rover reaches the acceptance radius, it publishes `yaw_setpoint = _vehicle_yaw` (hold current heading). The offboard velocity mode should follow the same pattern.

**Severity:** **High** — causes unintended yaw motion at every stop event. For a marking rover, this means the paint line gets a hook or arc at every endpoint.

**Action:** Must patch. See Q15.

---

### 7. Position vs Velocity Mode for Arc Following

**Finding:** Both modes can work, but have different tradeoffs for ±3cm accuracy:

| Aspect | Position Mode | Velocity Mode |
|--------|--------------|---------------|
| Who plans the path | PX4's pure pursuit (RPP) | GCS (our code) |
| Setpoint type | Target waypoint + cruising speed | Speed + heading |
| Arc handling | RPP interpolates between waypoints | GCS computes continuous curvature |
| Accuracy ceiling | Limited by RPP lookahead params | Limited by GCS path planner quality |
| GPS dependency | Requires continuous GPS | Velocity can use fused odometry |
| Safety (#18346) | **Vulnerable** — stale waypoint persists | Stale velocity → speed controller continues at last speed |

**Recommendation: Velocity mode** for arc following. The GCS can compute smooth velocity profiles with correct curvature, and the stale-setpoint path in velocity mode is less dangerous (rover just maintains last speed instead of driving toward a distant waypoint). But both the sign bug (Q5) and heading-at-zero bug (Q6) must be fixed first.

**Severity:** **Medium** — architectural choice, not a code bug.

**Action:** Use velocity mode. Fix Q5 and Q6 before testing.

---

### 8. `else if` Chain Limitation

**Finding:** [DifferentialOffboardMode.cpp:62-91] uses an `if/else if` chain. Only one control dimension activates at a time.

**Impact:** For velocity mode, this is **not a problem** — the velocity branch publishes **both** `rover_speed_setpoint` (speed) and `rover_attitude_setpoint` (heading), giving simultaneous speed + heading control. For position mode, the position branch publishes only `rover_position_setpoint`, and the `DifferentialPosControl` internally generates both speed and heading via pure pursuit.

**Severity:** **Low** — the `else if` chain design is intentional and works correctly for rover applications where velocity mode provides the needed dual control.

**Action:** Accept. No patch needed.

---

## C. Integration with Our Architecture

### 9. MAVROS2 `cmd_vel` → Offboard Mode

**Finding:** MAVROS2's `setpoint_velocity/cmd_vel` plugin converts `TwistStamped` → `SET_POSITION_TARGET_LOCAL_NED` with:
- `type_mask`: X_IGNORE, Y_IGNORE, Z_IGNORE, YAW_IGNORE, YAW_RATE_IGNORE all set; only VX/VY active
- This goes through [mavlink_receiver.cpp:1037] → `fill_offboard_control_mode()` at [mavlink_receiver.cpp:1025]: since velocity fields are not NaN, `ocm.velocity = true`. Position fields are NaN → `ocm.position = false`.

Result: activates the `else if (offboard_control_mode.velocity)` branch in `DifferentialOffboardMode.cpp:73`.

**Severity:** **Low** — informational. Correct mapping, no bug.

**Action:** Use this for velocity-mode arc following.

---

### 10. MAVROS2 `setpoint_raw/local` → Offboard Mode

**Finding:** MAVROS2's `setpoint_raw/local` (PositionTarget) publishes as `SET_POSITION_TARGET_LOCAL_NED` with type_mask controlling which fields are set. If position fields (x, y) are not ignored, `fill_offboard_control_mode()` sets `ocm.position = true`.

This activates the position branch of `DifferentialOffboardMode`. **Can be used for position-based arc following** but inherits the #18346 vulnerability.

**Severity:** **Low** — informational for architecture planning.

**Action:** Consider for position-based waypoint following (not recommended for continuous arcs due to #18346).

---

### 11. Rate Mismatch (50Hz → 100Hz)

**Finding:** In `DifferentialOffboardMode::offboardControl()`:
```cpp
_offboard_control_mode_sub.copy(&offboard_control_mode);
_trajectory_setpoint_sub.copy(&trajectory_setpoint);
```

Uses `copy()`, not `update()`. On 100Hz cycles where no new `trajectory_setpoint` arrived from the 50Hz GCS, `copy()` returns the **last published value** without checking freshness. The offboard controller re-publishes rover setpoints from stale (but recent) data **on every cycle**.

This means:
- Rover controllers always have data to work with (no skipped cycles)
- Between GCS updates, the rover uses the last known setpoint (effectively a zero-order hold)
- At 50Hz GCS → 100Hz rover, each setpoint is used for ~2 rover cycles

**Severity:** **Low** — expected behavior and acceptable for 2× rate mismatch.

**Action:** Accept. If GCS rate drops below 20Hz, the stale-data issue becomes more concerning. Monitor GCS update rate in telemetry.

---

### 12. Required PX4 Params Before First OFFBOARD Test

| Param | Recommended Value | Reason |
|-------|-------------------|--------|
| `COM_OF_LOSS_T` | **0.3** (default 1.0) | Shorter timeout = less uncontrolled motion |
| `COM_OBL_RC_ACT` | **7** (Disarm) pre-patch; **5** (Hold) post-patch | Prevents #18346 runaway |
| `NAV_RCL_ACT` | **7** (Disarm) or **5** (Hold) | Cascading failsafe if RC also lost |
| `COM_ARM_WO_GPS` | **1** (if GPS not used for initial tests) | Allows arm without GPS |
| `RO_SPEED_LIM` | **0.5** (m/s, slightly above 0.4 target) | Speed ceiling |
| `RO_MAX_THR_SPEED` | **0.5** (m/s at full throttle) | Feedforward calibration |
| `RO_ACCEL_LIM` | **0.3** (m/s²) | Smooth acceleration for paint quality |
| `RO_DECEL_LIM` | **0.5** (m/s²) | Faster decel for safety |
| `RO_SPEED_P` | Tune via step response | Speed PID proportional |
| `RO_SPEED_TH` | **0.02** (m/s) | Zero-speed threshold |
| `RO_YAW_P` | Tune via step response | Heading PID proportional |
| `RO_YAW_RATE_LIMIT` | **60** (deg/s) | Max turn rate |
| `RO_YAW_RATE_TH` | **0.5** (deg/s) | Zero-yaw-rate threshold |
| `RD_WHEEL_TRACK` | Measure actual rover | IK accuracy |
| `PP_LOOKAHD_GAIN` | 0.8 | If using position mode |
| `PP_LOOKAHD_MAX` | 3.0 (m) | If using position mode |
| `PP_LOOKAHD_MIN` | 0.5 (m) | If using position mode |
| `NAV_ACC_RAD` | **0.05** (5cm) | Waypoint acceptance |

---

## D. Patches We May Need

### 13. Fix #18346 — Mode Transition Setpoint Reset

**Location:** `DifferentialPosControl.cpp`, new method `resetSetpoints()` called from `RoverDifferential::Run()`.

**Approach:**
1. Add `vehicle_control_mode` subscription to `DifferentialPosControl`, `DifferentialSpeedControl`, and `DifferentialAttControl`
2. When `flag_control_offboard_enabled` transitions true→false:
   - `DifferentialPosControl`: reset `_target_waypoint_ned = Vector2f(NAN, NAN)`, `_stopped = true`
   - `DifferentialSpeedControl`: set `_speed_setpoint = NAN` so `PX4_ISFINITE` guard at line 66 prevents throttle output
   - `DifferentialAttControl`: set `_yaw_setpoint = NAN` so `PX4_ISFINITE` guard at line 79 prevents yaw rate output
3. Alternatively, a lighter approach: In `RoverDifferential::Run()`, detect mode change and publish explicit zero setpoints to each rover topic (`rover_speed_setpoint.speed_body_x = 0`, `rover_attitude_setpoint.yaw_setpoint = _vehicle_yaw`, etc.)

**Patch location:** `DifferentialPosControl.cpp` (add mode tracking), `DifferentialSpeedControl.cpp` (same), `DifferentialAttControl.cpp` (same), or `RoverDifferential.cpp` (centralized reset).

---

### 14. Fix Velocity Sign Bug

**Location:** [DifferentialOffboardMode.cpp:76]

**Approach:** Replace `velocity_ned.norm()` with signed projection onto body x-axis. This requires current vehicle yaw — add `vehicle_attitude_s` subscription to `DifferentialOffboardMode`.

```cpp
// Instead of: speed_body_x = velocity_ned.norm();
const float yaw = /* from vehicle_attitude */;
speed_body_x = velocity_ned(0) * cosf(yaw) + velocity_ned(1) * sinf(yaw);
```

When the GCS sends velocity in `MAV_FRAME_BODY_NED`, the mavlink receiver already handles the rotation at [mavlink_receiver.cpp:1078-1081]. But for `MAV_FRAME_LOCAL_NED` (MAVROS2 default), the offboard mode must do the projection.

**Patch location:** `DifferentialOffboardMode.hpp` (add subscription), `DifferentialOffboardMode.cpp:76` (change computation).

---

### 15. Fix Heading-at-Zero-Velocity

**Location:** [DifferentialOffboardMode.cpp:78-82]

**Approach:** Add velocity threshold check. When `velocity_ned.norm() < _param_ro_speed_th` or both components are near zero:
- Option A: Skip publishing `rover_attitude_setpoint` entirely (attitude controller retains last `_yaw_setpoint` which is already NAN-initialized, so no yaw rate is published)
- Option B: Publish `yaw_setpoint = _vehicle_yaw` (hold current heading, same as `DifferentialPosControl.cpp:85`)

Option B is cleaner and explicit.

```cpp
if (velocity_ned.norm() > _param_ro_speed_th.get()) {
    rover_attitude_setpoint.yaw_setpoint = atan2f(velocity_ned(1), velocity_ned(0));
} else {
    rover_attitude_setpoint.yaw_setpoint = _vehicle_yaw; // hold current heading
}
```

**Patch location:** `DifferentialOffboardMode.hpp` (add `vehicle_attitude` sub + `_param_ro_speed_th`), `DifferentialOffboardMode.cpp:78-82` (add guard).

---

## Ranked Action List

**P0 — Ship blockers (safety):**
1. **Set `COM_OBL_RC_ACT=7` (Disarm)** — immediately, before any offboard test. This is the only param that guarantees the rover stops if offboard signal is lost
2. **Set `COM_OF_LOSS_T=0.3`** — reduce the uncontrolled window from 1.0s to 0.3s
3. **Patch #18346** (Q13) — reset setpoints on offboard→position mode transition. Without this, `COM_OBL_RC_ACT=5` (Hold) cannot be safely used

**P1 — Functional blockers (can't complete missions):**
4. **Patch velocity sign bug** (Q14) — rover must reverse at arc endpoints
5. **Patch heading-at-zero-velocity** (Q15) — rover must not pivot at stops

**P2 — Before production:**
6. Switch `COM_OBL_RC_ACT` from 7→5 (Hold) once #18346 patch is verified
7. Set `NAV_RCL_ACT` to 7 or 5 (match offboard loss policy for RC-cascade case)
8. Configure all rover-specific params (Q12 list)
9. Tune speed/heading PIDs with the actual rover hardware
10. Verify the full failsafe chain end-to-end: kill GCS mid-mission, confirm rover stops within tolerance