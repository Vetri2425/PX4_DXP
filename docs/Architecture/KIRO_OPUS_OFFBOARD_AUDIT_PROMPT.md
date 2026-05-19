# Kiro Opus Prompt — PX4 Rover OFFBOARD Code Audit

---

## Context

You are auditing the PX4 v1.16.2 rover differential OFFBOARD control path for a **commercial marking rover** (3WD differential drive, 0.4 m/s, target ±3cm arc accuracy). The rover draws paint lines on construction sites. A runaway or failure to stop is a **safety incident**, not just a bug.

We are using **MAVROS2 only** (no DDS). Setpoints arrive via MAVLink `SET_POSITION_TARGET_LOCAL_NED` → `mavlink_receiver` → uORB → rover controllers.

**Key open issue:** [PX4 #18346](https://github.com/PX4/PX4-Autopilot/issues/18346) — "Rover offboard lost: default failsafe continues on last setpoint." Open since Oct 2021, stale, no fix, no assignee.

---

## Source Files to Audit (all from `D:\Vetri\3WD_GCS\PX4-Autopilot\`)

### Offboard Mode Entry/Exit
1. `src/modules/commander/Commander.cpp` (2491 lines) — `offboardControlCheck()`, mode switch logic
2. `src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp` (102 lines) — `COM_OF_LOSS_T` timeout, signal loss detection
3. `src/modules/commander/failsafe/failsafe.cpp` (599 lines) — `checkModeFallback()`, `fromOffboardLossActParam()`
4. `src/modules/commander/failsafe/failsafe.h` — `offboard_loss_failsafe_mode` enum (Position/Altitude/Stabilized/RTL/Land/Hold/Terminate/Disarm)
5. `src/modules/commander/ModeUtil/control_mode.cpp` — `NAVIGATION_STATE_OFFBOARD` flag mapping

### Rover Differential Offboard
6. `src/modules/rover_differential/DifferentialDriveModes/DifferentialOffboardMode/DifferentialOffboardMode.cpp` (85 lines) — **THE core file**: translates `trajectory_setpoint` → rover setpoints
7. `src/modules/rover_differential/DifferentialDriveModes/DifferentialOffboardMode/DifferentialOffboardMode.hpp` (79 lines) — subscriptions/publications

### Rover Controllers (downstream of offboard)
8. `src/modules/rover_differential/RoverDifferential.cpp` (220 lines) — main Run() loop at 100Hz, calls all sub-controllers
9. `src/modules/rover_differential/DifferentialPosControl/DifferentialPosControl.cpp` (182 lines) — pure pursuit position controller
10. `src/modules/rover_differential/DifferentialSpeedControl/DifferentialSpeedControl.cpp` (158 lines) — speed PID + slew rate
11. `src/modules/rover_differential/DifferentialAttControl/DifferentialAttControl.cpp` — heading PID
12. `src/modules/rover_differential/DifferentialRateControl/DifferentialRateControl.cpp` — yaw rate PID
13. `src/modules/rover_differential/DifferentialActControl/DifferentialActControl.cpp` — actuator mixing + IK

### MAVLink Receiver (upstream of offboard)
14. `src/modules/mavlink/mavlink_receiver.cpp` (2872 lines) — `fill_offboard_control_mode()`, `SET_POSITION_TARGET_LOCAL_NED` handler

### Shared Library
15. `src/lib/rover_control/RoverControl.cpp` (217 lines) — throttle/speed/attitude/rate control utilities

---

## What I Found (Pre-Audit Summary for Context)

### Failsafe System (appears well-designed)
- `COM_OF_LOSS_T` (default 1.0s) — timeout before failsafe triggers
- `COM_OBL_RC_ACT` — 8 fallback options (Position/Altitude/Stabilized/RTL/Land/Hold/Terminate/Disarm)
- `offboardCheck.cpp` sets `offboard_control_signal_lost = true` when setpoints are stale > `COM_OF_LOSS_T`
- `checkModeFallback()` in `failsafe.cpp` triggers `COM_OBL_RC_ACT` action on signal loss
- If RC also lost, cascades to `NAV_RCL_ACT` fallback

### Issue #18346 — The Runaway Bug
- **Symptom:** When offboard setpoints stop, failsafe triggers, rover enters Position mode — but **continues at last setpoint instead of stopping**
- **Likely root cause:** Position mode controller inherits the last `trajectory_setpoint` or `rover_position_setpoint` from the offboard mode. The position setpoint wasn't zeroed on mode transition.
- In `DifferentialPosControl.cpp`, line 167: `_target_waypoint_ned` is only updated when `_rover_position_setpoint_sub.updated()` fires. If offboard mode published a stale position setpoint before exiting, the position controller keeps driving toward it.

### DifferentialOffboardMode.cpp — Code Quality Issues
1. **No timestamp validation** — blindly copies `trajectory_setpoint` without checking if data is fresh
2. **No NaN guard on velocity** — `atan2f(velocity_ned(1), velocity_ned(0))` at line 80 crashes if both components are NaN/zero (heading = 0, always faces North)
3. **Speed is always positive** — `velocity_ned.norm()` at line 76 loses direction sign. Rover can't reverse in velocity offboard mode.
4. **No timeout/watchdog** — offboard mode itself doesn't check setpoint age; relies entirely on commander's `offboardControlCheck()` which has a 1.0s default window
5. **`else if` chain** — can't do simultaneous position+attitude or velocity+rate. Only one control dimension at a time.

---

## Audit Questions (Answer ALL)

### A. Safety-Critical

1. **#18346 root cause:** Trace the exact data path when OFFBOARD → Position mode failsafe fires. Does `rover_position_setpoint` retain the last offboard value? Does `DifferentialPosControl` start driving toward it? Is there a mode-transition reset in `RoverDifferential::Run()`?

2. **What is the safest `COM_OBL_RC_ACT` value for a ground rover?** Position mode (0) is the default but triggers #18346. Hold mode (5) might not exist for rovers. Disarm (7) stops the rover dead but requires re-arm. Terminate (6) kills everything. What actually works for a rover?

3. **Is there any code in `RoverDifferential::Run()` or the sub-controllers that resets setpoints on mode change?** Search for any `_vehicle_control_mode` flag check that zeros `_speed_setpoint`, `_rover_throttle_setpoint`, or `_rover_steering_setpoint` when OFFBOARD exits.

4. **What happens during the `COM_OF_LOSS_T` window (default 1.0s)?** The rover continues at the last setpoint for a full second before failsafe triggers. For a 0.4 m/s rover, that's 40cm of uncontrolled motion. Is there a way to shorten this?

### B. Control Path Correctness

5. **Velocity mode sign bug:** `DifferentialOffboardMode.cpp:76` uses `velocity_ned.norm()` which is always ≥0. The rover cannot reverse. For a marking rover that needs to back up at arc endpoints, is this a showstopper? What's the correct implementation?

6. **Heading from velocity:** `atan2f(vy, vx)` gives heading = 0 when vx=vy=0 (zero velocity). This means stopping the rover publishes yaw=0 (North). Does the attitude controller then try to turn the rover to face North every time it stops?

7. **Position mode vs velocity mode:** For arc following with RPP controller, which offboard mode makes more sense? Position (send target waypoints) or velocity (send speed+heading)? What are the tradeoffs for ±3cm accuracy?

8. **`else if` chain limitation:** Offboard mode can only activate one control dimension (position OR velocity OR attitude OR body_rate). For a rover, we need simultaneous speed + heading control. Does this mean we MUST use velocity mode (which publishes both `rover_speed_setpoint` and `rover_attitude_setpoint`)? Or can position mode work for arcs?

### C. Integration with Our Architecture

9. **MAVROS2 `setpoint_velocity/cmd_vel` (TwistStamped) → what offboard mode does it activate?** Trace from MAVROS plugin through MAVLink message to `fill_offboard_control_mode()`. Does it set `velocity=true` and nothing else?

10. **MAVROS2 `setpoint_raw/local` (PositionTarget) → what offboard mode?** Can we use this for position-based arc following instead of velocity?

11. **Rate mismatch:** MAVROS2 publishes at 50Hz max. `RoverDifferential` runs at 100Hz. What does the offboard mode do on cycles where no new `trajectory_setpoint` arrives? Does it re-publish the stale value, or skip?

12. **What PX4 params must we set before first OFFBOARD test?** List all safety-critical params: `COM_OF_LOSS_T`, `COM_OBL_RC_ACT`, `COM_ARM_WO_GPS`, any rover-specific offboard params.

### D. Patches We May Need

13. **If #18346 is confirmed as a missing reset on mode transition, draft the fix location and approach.** Which file, which function, what to zero. Don't write the patch — just specify where and what.

14. **If velocity sign bug is confirmed, draft the fix.** `speed_body_x` should preserve direction sign. How to derive it from NED velocity?

15. **If heading-at-zero-velocity bug is confirmed, draft the fix.** When velocity is zero, the attitude setpoint should be the current vehicle heading, not North.

---

## Output Format

For each question, answer:
- **Finding:** What you found in the code (cite file:line)
- **Severity:** Critical / High / Medium / Low
- **Action:** What we must do (patch, param change, workaround, or accept)
- **Patch location:** If a fix is needed, which file and function

End with a **ranked action list** — what to fix or configure first, in order of safety priority.