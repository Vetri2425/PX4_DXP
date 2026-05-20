# PX4 Rover OFFBOARD Audit — Findings

Subject: PX4 v1.16.2 rover differential, MAVROS2-only path, marking rover (3WD, 0.4 m/s, ±3cm).
Audited tree: `D:\Vetri\3WD_GCS\PX4-Autopilot` (Vetri2425 fork) and stock `v1.16.2` tag.

---

## 0. Build-Tree Reality Check (read this first)

The fork's working copy contains TWO incompatible architectures stacked on top of each other:

- New 2025 architecture under `src/modules/rover_differential/`:
  `DifferentialAct/Att/Pos/Rate/SpeedControl/`,
  `DifferentialDriveModes/Differential{Auto,Manual,Offboard}Mode/`.
- Old v1.16.2 architecture base files at the same level: `RoverDifferential.cpp/.hpp` (which
  `#include "DifferentialVelControl/DifferentialVelControl.hpp"` — a folder that does not
  exist in the fork's tree).

The CI workflow `.github/workflows/build_rover.yml` resolves this by:

1. Checking out **stock PX4 `v1.16.2`** from `PX4/PX4-Autopilot`.
2. Copying only `RoverDifferential.cpp/.hpp` + `module.yaml` + `RoverLandDetector.cpp` +
   `mission_block.cpp` from the fork on top.
3. Running `make cubepilot_cubeorangeplus_rover`.

So the actual flying binary is **stock v1.16.2 rover_differential**, with two patches:
- RD_TANK_MODE bypass in `RoverDifferential::generateActuatorSetpoint()`.
- IK sign swap in `computeInverseKinematics()` (left/right swapped to match Sabertooth).

**Implication:** the audit prompt's `DifferentialOffboardMode.cpp` is dead code in this build.
The real OFFBOARD handler is `src/modules/rover_differential/DifferentialVelControl/`
and `src/modules/rover_differential/DifferentialPosControl/` from v1.16.2. All file:line
citations below refer to those v1.16.2 files. Where I could not read the file directly
through the workspace, I read it via `git show v1.16.2:<path>` from the fork repo.

Fork commit referenced: `de704a0a` (`fix(rover_differential): retarget RD_TANK_MODE patch
to v1.16.2 source tree`) — confirms the dual-tree structure is intentional patch-overlay.

---

## A. Safety-Critical Findings

### A1. #18346 root cause — actual data path on OFFBOARD-position → POSCTL failsafe

**Finding.** Partial root cause confirmed; full runaway is mode-dependent.

OFFBOARD-position path in v1.16.2:

```
trajectory_setpoint  (mavlink_receiver:1109)
       │
       ▼
DifferentialPosControl::generatePositionSetpoint()    ← only runs when offboard+position
       │  publishes rover_position_setpoint (cached forever, never invalidated)
       ▼
DifferentialPosControl::generateVelocitySetpoint()    ← runs whenever flag_control_position_enabled
       │
       ├── flag_control_manual_enabled && flag_control_position_enabled  → manualPositionMode()
       ├── flag_control_auto_enabled                                     → autoPositionMode()
       └── else → reads cached _rover_position_setpoint                  → goToPositionMode()
```

`DifferentialPosControl.hpp:159` declares `rover_position_setpoint_s _rover_position_setpoint{}`
as a class member that is initialized to `position_ned = {NAN, NAN}`. After any OFFBOARD-position
session, `generatePositionSetpoint()` (`DifferentialPosControl.cpp:111-122`) overwrites it with
finite values. **There is no code path that resets it back to NaN when OFFBOARD ends.** It
also stays cached across mode transitions because `DifferentialPosControl` is a class
instance owned by `RoverDifferential`, never destroyed.

What happens on OFFBOARD-position loss with default `COM_OBL_RC_ACT=0` (Position_mode):

- Commander forces nav_state to `POSCTL`.
- `control_mode.cpp:74-83` sets `flag_control_manual_enabled=true` AND
  `flag_control_position_enabled=true` for POSCTL.
- In `DifferentialPosControl::generateVelocitySetpoint()` the **first** `if` branch matches
  (manual && position) → `manualPositionMode()` runs, NOT `goToPositionMode()`.
- `manualPositionMode()` reads `manual_control_setpoint` directly from joystick/RC, not from
  the cached position setpoint.

So in POSCTL fallback, the cached setpoint is **not** used. The runaway #18346 describes is
actually driven by two other things in v1.16.2:

1. `manual_control_setpoint` is the active source. If RC was active alongside MAVLink and the
   joystick is non-zero at the moment of failover, the rover follows the joystick.
   **More dangerous:** if no joystick is connected at all (typical for a tethered MAVROS-only
   rover), `manual_control_setpoint` was never published. `_manual_control_setpoint_sub.copy()`
   then leaves the local struct zero-initialized → `throttle=0, roll=0` → first branch
   `speed_body_x_setpoint < FLT_EPSILON` → "turn on spot" with `speed=0, bearing=current yaw`
   → effectively a stop. So no joystick = safe stop in POSCTL.
2. The **`COM_OF_LOSS_T` blind window** (default 1.0s). During this window OFFBOARD is still
   active, the last `trajectory_setpoint` is still being acted on, and the rover does not
   know setpoints are stale. At 0.4 m/s = 40 cm of uncontrolled travel. See A4.

The cached `_rover_position_setpoint` only becomes a runaway if a future code path or
operator-induced mode transition lands in `goToPositionMode()` directly (e.g. user-intended
mode is anything that has `flag_control_position_enabled=true && !flag_control_manual_enabled
&& !flag_control_auto_enabled`). On v1.16.2 that combination doesn't naturally exist for a
ground rover, but **any future change that flips `flag_control_manual_enabled=false` while
position is enabled re-introduces the bug.** It is latent.

**Severity.** High (latent runaway, real 40 cm coast in window).
**Action.** Patch `DifferentialPosControl::generateVelocitySetpoint()` and
`generatePositionSetpoint()` to invalidate `_rover_position_setpoint` to NaN whenever
`flag_control_offboard_enabled` transitions from true to false. See `OFFBOARD_PATCHES.md` P1.

---

### A2. Safest `COM_OBL_RC_ACT` value for a ground rover

**Finding.** Enum from `failsafe.h:76-86`:

```
0 = Position_mode  (default)
1 = Altitude_mode
2 = Stabilized
3 = Return_mode  (RTL)
4 = Land_mode
5 = Hold_mode    (AUTO_LOITER)
6 = Terminate
7 = Disarm
```

Mapping to navigation states is in `failsafe.cpp:284-323`. For a ground rover:

- **0 Position_mode → POSCTL.** v1.16.2 PosControl in POSCTL goes through `manualPositionMode()`
  which reads `manual_control_setpoint`. With no RC and no joystick (MAVROS2-only rover), local
  struct is zero, rover stops. With RC active and joystick non-zero, rover follows stick.
  Acceptable only if RC is reliable.
- **1 Altitude_mode, 2 Stabilized.** Designed for multirotors. On a rover these set
  `flag_control_attitude_enabled` only, no position/velocity. `DifferentialAttControl` runs
  but `_yaw_setpoint` is NaN until something publishes it → effectively idle. Not safe-stop,
  not runaway. Untested for rovers, avoid.
- **3 Return_mode (RTL).** Auto-mission path. `DifferentialPosControl::autoPositionMode()`
  reads `position_setpoint_triplet` published by navigator. RTL on a rover requires a valid
  home position, valid local position estimate, and a path back. For a marking rover mid-arc,
  this means it drives back to the takeoff point cutting across whatever is in the way. **Not
  safe** for a rover painting a customer site.
- **4 Land_mode.** Maps to `NAVIGATION_STATE_AUTO_LAND`. Useless for rovers — no descent.
  Land_detector is always-landed for rovers in this fork (per `RoverLandDetector.cpp` patch),
  so behaviour is undefined. Avoid.
- **5 Hold_mode → AUTO_LOITER.** PosControl `autoPositionMode()`, but with no waypoint
  triplet active, `_curr_wp_ned` is NaN → `auto_stop` branch fires (`!_next_wp_ned.isAllFinite()`)
  and publishes `speed=0, bearing=_vehicle_yaw`. **This is a safe stop.** Available on rovers.
- **6 Terminate.** Sets `flag_control_termination_enabled` only, all controllers idle, motors
  go to zero through actuator armed-but-no-setpoint path. Effectively a hard stop, but the
  rover stays armed and cannot be re-engaged without rebooting commander. Use only if Hold
  is unreachable.
- **7 Disarm.** Cuts motor PWM immediately. Rover coasts to a stop on its own friction
  (no regen braking). Then requires re-arm and re-mission. Cleanest fail-safe stop semantics
  for a marking rover but **the most expensive to recover from in the field**.

**Severity.** Critical (default value (0) is wrong for an RC-less rover).
**Action.** Set `COM_OBL_RC_ACT=5` (Hold_mode) for a marking rover. It produces a clean
auto-loiter stop using PosControl's `auto_stop` branch and does not require RC, joystick, or
home position. Falls through to RTL/Disarm cascade only if also low battery. See action plan
for the full param set.

---

### A3. Mode-transition resets in `RoverDifferential::Run()` and sub-controllers

**Finding.** `RoverDifferential::Run()` (`RoverDifferential.cpp:60-89`) does NOT track
`_was_armed`, does NOT call any `reset()`, does NOT call any `stopVehicle()`. It just runs
all four sub-controllers every 10ms.

Compare to the sibling architectures in the same repo:

- `rover_ackermann/RoverAckermann.cpp:79-94` does call `reset()` on mode change AND
  `_ackermann_act_control.stopVehicle()` on disarm-after-armed.
- `rover_mecanum/RoverMecanum.cpp:79-94` is identical.

This is a structural deficiency in v1.16.2 differential. However, the sub-controllers
self-compensate more than initially claimed:

- `DifferentialAttControl.cpp:78-89`: when `flag_control_attitude_enabled=false`, the `else`
  branch resets `_pid_yaw.resetIntegral()` and `_adjusted_yaw_setpoint`. Good — exits
  attitude-mode safely.
- `DifferentialRateControl.cpp:76-85`: when `flag_control_rates_enabled=false`, the `else`
  branch resets `_pid_yaw_rate.resetIntegral()`. Good — exits rate-mode safely.
- `DifferentialVelControl.cpp:74-77`: when `flag_control_velocity_enabled=false` OR
  `flag_armed=false`, the `else` branch resets `_pid_speed.resetIntegral()` and
  `_speed_setpoint.setForcedValue(0.f)`. Good — exits velocity-mode safely.
- `DifferentialPosControl::updatePosControl()` (`DifferentialPosControl.cpp:53-66`): runs only
  when `flag_control_position_enabled && flag_armed`. When position is disabled the function
  returns early. **But it does NOT reset `_rover_position_setpoint`, `_curr_wp_ned`, or
  `_course_control` state.** The cache survives across mode transitions. **This is the only
  sub-controller with a state-reset gap.**

**Severity.** High (narrower than initially stated — only PosControl is affected).
**Action.** Patch `DifferentialPosControl::updatePosControl()` to detect `flag_control_offboard_enabled`
falling edge and invalidate `_rover_position_setpoint` to NaN. Also add a `_was_armed` guard
in `RoverDifferential::Run()` to publish zero actuator_motors on disarm for cleaner motor cut.
See P1 and P2.

---

### A4. The `COM_OF_LOSS_T` blind window

**Finding.** `offboardCheck.cpp:39-41`:

```cpp
bool data_is_recent = hrt_absolute_time() < offboard_control_mode.timestamp
                      + static_cast<hrt_abstime>(_param_com_of_loss_t.get() * 1_s);
```

Default `COM_OF_LOSS_T = 1.0s`. For 1.0s after the last `offboard_control_mode` publish, the
flag `offboard_control_signal_lost` stays false. Failsafe does not trigger. The downstream
PosControl/VelControl keep acting on the last `trajectory_setpoint` because their
subscriptions still report fresh data.

At 0.4 m/s rover cruise, 1.0s = 40 cm of uncontrolled travel. For ±3cm arc accuracy that's
already 10× the tolerance. For obstacle margin on a construction site it can be a collision.

The minimum value `COM_OF_LOSS_T` accepts is generally 0.0s but PX4 enforces a documented
floor of ~0.05s; below ~50 ms commander loop noise and OFFBOARD heartbeat jitter cause
spurious failsafe at the publishing rate.

**Severity.** Critical for a 40 cm tolerance rover.
**Action.** Set `COM_OF_LOSS_T = 0.2s` (200 ms). At 0.4 m/s = 8 cm coast worst-case. Below
that, expect false trips. Combine with a 50 Hz minimum publish rate from MAVROS2 (every 20 ms,
10× safety margin against the 200 ms timeout). See action plan.

---

## B. Control-Path Correctness

### B1. Velocity sign bug — `DifferentialVelControl.cpp:120-121`

```cpp
const Vector2f velocity_in_local_frame(trajectory_setpoint.velocity[0],
                                       trajectory_setpoint.velocity[1]);
if (offboard_vel_control && velocity_in_local_frame.isAllFinite()) {
    differential_velocity_setpoint_s differential_velocity_setpoint{};
    differential_velocity_setpoint.timestamp = _timestamp;
    differential_velocity_setpoint.speed = velocity_in_local_frame.norm();
    differential_velocity_setpoint.bearing = atan2f(velocity_in_local_frame(1),
                                                    velocity_in_local_frame(0));
```

**Finding.** `velocity_in_local_frame.norm()` is always ≥ 0. Information about reverse
direction is encoded in `bearing` (`atan2f` covers full ±π). Downstream
`DifferentialPosControl::generateVelocitySetpoint` (when called from VelControl chain) does
respect bearing direction, but PosControl's `goToPositionMode()` uses pure pursuit which
assumes forward motion. Forcing reverse via opposite NED velocity works only because the
rover then "turns around" via the heading-error → SPOT_TURNING transition in
`generateAttitudeAndThrottleSetpoint()` (`DifferentialVelControl.cpp:140-148`). That spot turn
takes 1-3 seconds and breaks any arc continuity.

For a marking rover that needs to back up at arc endpoints (lift the brush, reverse 5 cm to
touch the line again), the spot-turn-and-drive-forward semantics is wrong. You want a true
negative speed.

**Severity.** Medium for arc marking, High if reverse motion is a planned mission step.
**Action.** Patch `DifferentialVelControl::generateVelocitySetpoint()` to compute a signed
`speed_body_x` by projecting NED velocity into the body frame instead of taking the norm.
See P3 in `OFFBOARD_PATCHES.md`.

---

### B2. Heading at zero velocity — `DifferentialVelControl.cpp:121`

```cpp
differential_velocity_setpoint.bearing = atan2f(velocity_in_local_frame(1),
                                                velocity_in_local_frame(0));
```

**Finding.** `atan2f(0, 0) = 0` per IEEE 754 / glibc convention. So a `(0, 0)` velocity
setpoint publishes `bearing = 0` = North.

Downstream effect (`DifferentialVelControl.cpp:131-148`):

- `_rover_attitude_setpoint_pub.publish(...)` with `yaw_setpoint = 0`.
- `DifferentialAttControl` sees `yaw_setpoint = 0`, `_vehicle_yaw = ψ_actual`. Heading error
  = `wrap_pi(0 - ψ)`. If ψ ≠ 0, the rover yaws toward North.
- `_current_state` may transition to `SPOT_TURNING` (line 140) because `|heading_error| >
  RD_TRANS_DRV_TRN`. Then `speed_body_x_setpoint = 0` (line 152). So the rover sits and yaws
  toward North, then stops.

Net result: every time you send a zero-velocity hover-stop in OFFBOARD, the rover rotates
to face North before stopping. Visible bug, often misread as "drift on stop".

**Severity.** High (visible misbehaviour, mishandles common stop case).
**Action.** Add a magnitude check; when velocity magnitude is below a threshold, set bearing
to current vehicle yaw. See P4.

---

### B3. Position vs velocity offboard mode for ±3cm arc

**Finding.** Trade-offs:

- **Position mode** (set `position[0..1]`, leave velocity NaN). Each setpoint is a target
  waypoint. PosControl runs pure pursuit (`DifferentialPosControl::goToPositionMode`) with
  `RO_SPEED_LIM` cruising speed and `RO_DECEL_LIM`/`RO_JERK_LIM` braking. Look-ahead is
  `PP_LOOKAHD_GAIN * speed`, clamped to `[PP_LOOKAHD_MIN, PP_LOOKAHD_MAX]`. For arc
  following, you publish points spaced by ~look-ahead distance (≈ 1× look-ahead so the rover
  sees the next point before reaching the current). Errors converge to the cross-track error
  of pure pursuit, which scales with look-ahead² / arc-radius.
- **Velocity mode** (set `velocity[0..1]`, leave position NaN). Every setpoint is a heading
  + speed. The arc is described as a sequence of `(speed, bearing)` over time. Rover yaws to
  heading then accelerates. With `RD_TRANS_DRV_TRN` threshold the rover toggles between
  drive and spot-turn each cycle if heading changes are sharp.

For ±3cm on a 1m radius arc at 0.4 m/s:
- Position mode with `PP_LOOKAHD_MIN ≈ 0.1m` and dense waypoints (every 5 cm) gives natural
  smoothing and bounded cross-track. Recommended.
- Velocity mode at 50 Hz with 8 ms latency sees 3.2 mm of motion per cycle; achievable in
  theory but the spot-turn threshold (`RD_TRANS_DRV_TRN` default 30°) makes any sharp
  heading change a stop-and-yaw event.

**Severity.** N/A (architectural choice).
**Action.** Use **position mode** (`SET_POSITION_TARGET_LOCAL_NED` with position only, all
other type_mask bits set to ignore). Publish waypoints every 0.05–0.10 m along the arc.
Reserve velocity mode for straight-line transit between marking segments where direction is
constant.

---

### B4. The `else if` chain — single-dimension offboard

**Finding.** `DifferentialVelControl.cpp:118` checks
`offboard_control_mode.velocity && !offboard_control_mode.position`. So velocity-only is the
trigger. But the actual control flow per cycle in v1.16.2 is:

```
PosControl  always runs  → if offboard+position: publish rover_position_setpoint then enter
                              goToPositionMode → publishes differential_velocity_setpoint
                              (speed + bearing)
VelControl  always runs  → if offboard+velocity (no position): publish differential_velocity_setpoint
                              from trajectory_setpoint;
                              else: regenerate attitude+throttle from existing setpoint
                              (cached or freshly published by PosControl)
AttControl  always runs  → consume rover_attitude_setpoint, publish rover_steering_setpoint
RateControl always runs  → consume rate setpoint OR steering setpoint, publish actuator path
```

So PX4 v1.16.2 already produces both speed and heading regardless of which OFFBOARD field you
populate. You don't need simultaneous position+attitude — position alone gives bearing
through pure pursuit, velocity alone gives bearing through atan2(vy,vx).

**Severity.** Low (the "limitation" doesn't apply in v1.16.2).
**Action.** None. Use either position or velocity, not both.

---

## C. Integration with MAVROS2

### C1. `setpoint_velocity/cmd_vel` (TwistStamped)

**Finding.** MAVROS2's `setpoint_velocity` plugin publishes `SET_POSITION_TARGET_LOCAL_NED`
with `coordinate_frame = MAV_FRAME_LOCAL_NED` and only velocity components valid (position +
acceleration bits set in `type_mask`).

`mavlink_receiver.cpp:1027-1145` decodes it. The relevant block for offboard_control_mode at
`mavlink_receiver.cpp:1107-1110`:

```cpp
offboard_control_mode_s ocm{};
ocm.position = !matrix::Vector3f(setpoint.position).isAllNan();
ocm.velocity = !matrix::Vector3f(setpoint.velocity).isAllNan();
ocm.acceleration = !matrix::Vector3f(setpoint.acceleration).isAllNan();
```

For a velocity-only TwistStamped: `ocm.position=false, ocm.velocity=true,
ocm.acceleration=false`. The publish at `mavlink_receiver.cpp:1126` then sets
`flag_control_velocity_enabled=true` (via `control_mode.cpp:135`). PosControl skips offboard
position branch. VelControl `generateVelocitySetpoint` runs.

So **TwistStamped → velocity offboard mode** is correct end-to-end. Subject to B1 (sign) and
B2 (zero-velocity heading).

**Severity.** Low.
**Action.** None for the wiring. Use `setpoint_velocity/cmd_vel_unstamped` for ROS2 TwistStamped
input. Set the body-frame variant if you want body-relative velocity (the MAVLink receiver at
`mavlink_receiver.cpp:1056-1098` handles `MAV_FRAME_BODY_NED` rotation).

---

### C2. `setpoint_raw/local` (PositionTarget)

**Finding.** Same `SET_POSITION_TARGET_LOCAL_NED` MAVLink message, but exposed in MAVROS2 as
the raw PositionTarget message where you control `type_mask` directly. Position-only
configuration: `type_mask = IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY |
IGNORE_AFZ | IGNORE_YAW | IGNORE_YAW_RATE`. Then `setpoint.position` is finite, all velocity
NaN, all acceleration NaN.

In the receiver, that produces `ocm.position=true, ocm.velocity=false`. Path goes through
`DifferentialPosControl::generatePositionSetpoint` → `goToPositionMode`. This is the path
recommended in B3.

One important detail at `mavlink_receiver.cpp:1129-1133`:

```cpp
if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
    // only publish setpoint once in OFFBOARD
    setpoint.timestamp = hrt_absolute_time();
    _trajectory_setpoint_pub.publish(setpoint);
}
```

The `trajectory_setpoint` topic is **only** updated while the vehicle is already in OFFBOARD
nav_state. The `offboard_control_mode` topic is published unconditionally (line 1126). This
means: streaming setpoints does NOT auto-engage OFFBOARD; you still have to send a
SET_MODE → OFFBOARD command. Streaming alone keeps the heartbeat alive once OFFBOARD is
active.

**Severity.** Low.
**Action.** Use `setpoint_raw/local` with position-only `type_mask` for arc marking.
Engage OFFBOARD via `mavros_msgs/srv/SetMode` with `custom_mode="OFFBOARD"` after at least
one second of streaming setpoints.

---

### C3. Rate mismatch — MAVROS2 50 Hz vs RoverDifferential 100 Hz

**Finding.** `RoverDifferential::Run()` is scheduled on `ScheduleOnInterval(10_ms)`
(`RoverDifferential.cpp:48`) → 100 Hz. MAVROS2 publishes at the rate the user asks (default
30-50 Hz for `setpoint_raw/local`).

Each sub-controller calls `_xxx_setpoint_sub.updated()` or `.copy()`:
- `.updated()` returns true once per new message.
- `.copy()` always returns the most recent value (or zeroed struct if never published).

`DifferentialPosControl::generatePositionSetpoint` (`DifferentialPosControl.cpp:111-121`)
reads `trajectory_setpoint` via `.copy()` every call regardless of update flag. So on cycles
where MAVROS2 has not published, PosControl re-publishes the **same** `rover_position_setpoint`
to its own topic, every 10 ms. The local position controller then keeps acting.

This is fine for steady-state arc following, but for stop transitions: if MAVROS2 publishes
"stop here" once and then goes silent, PosControl keeps re-publishing the same target → rover
overshoots and oscillates around it (pure pursuit converges but the integrator never sees a
new target).

**Severity.** Medium.
**Action.** Either keep MAVROS2 publishing at ≥ 20 Hz continuously (reuse last setpoint if
no new plan), or publish a `stopped` sentinel. The standard ROS2 OFFBOARD pattern is "always
keep the last setpoint warm at ≥ 2 Hz" — `COM_OF_LOSS_T` requires it.

---

### C4. Required PX4 params for first OFFBOARD test

Listed in the action plan (`OFFBOARD_ACTION_PLAN.md`). Summary:

- `COM_OF_LOSS_T` = 0.2 (was 1.0)
- `COM_OBL_RC_ACT` = 5 (Hold_mode, was 0 Position_mode)
- `COM_RCL_EXCEPT` = bit 2 (allow OFFBOARD without RC) — necessary for headless rover
- `COM_ARM_WO_GPS` = 0 (require GPS for arming, marking rover needs absolute position)
- `EKF2_HGT_REF` = 1 or 3 (GPS height; rover doesn't have a baro that's useful)
- `RO_SPEED_LIM` = 0.5 (max 0.5 m/s, leave 25% margin over 0.4 m/s cruise)
- `RO_ACCEL_LIM` = 0.4 m/s² (1 second to cruise)
- `RO_DECEL_LIM` = 0.8 m/s² (twice accel for safety stop ~ 0.5s)
- `RO_JERK_LIM` = 5.0 m/s³ (smooth profile)
- `RO_MAX_THR_SPEED` = 0.5 m/s (matches RO_SPEED_LIM at full throttle)
- `RD_TRANS_DRV_TRN` = 30° (default, threshold to enter spot-turn)
- `RD_TRANS_TRN_DRV` = 5° (default, threshold to exit spot-turn)
- `RD_TANK_MODE` = 0 for OFFBOARD (only set to 1 for joystick tank-mode debugging)

---

## Cross-Reference

- `OFFBOARD_PATCHES.md` — concrete patch sketches for findings A1, A3, B1, B2.
- `OFFBOARD_ACTION_PLAN.md` — ranked action list, full param set, MAVROS2 client patterns.
