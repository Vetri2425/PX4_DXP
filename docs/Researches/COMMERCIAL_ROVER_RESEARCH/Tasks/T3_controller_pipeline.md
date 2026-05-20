# T3: Controller Pipeline — Trajectory to OFFBOARD Commands

## What to Research

What is the complete control pipeline from a trajectory (list of target positions/velocities) to the actual OFFBOARD setpoint commands that MAVROS2 sends to PX4? What controllers are needed and in what order?

## Specific Questions

1. **What controllers exist between trajectory and motors?**
   - Path following controller (pure pursuit, Stanley, MPC)
   - Speed controller (velocity profile tracking)
   - Heading controller (yaw tracking)
   - Low-level motor controller (PX4 internal: differential drive mixing)
   - Which of these run on Jetson vs PX4?

2. **Pure Pursuit for differential drive:**
   - Standard pure pursuit: computes steering angle for Ackermann
   - How to adapt pure pursuit for differential drive (no steering angle, only left/right wheel speeds)?
   - What is the lookahead distance for 0.3-0.4 m/s and ±2cm accuracy?
   - How does pure pursuit handle: straight lines, arcs, sharp corners, U-turns?

3. **Stanley controller vs Pure Pursuit vs MPC:**
   - Pros/cons of each for differential drive marking rover
   - Which is simplest to implement and still achieve ±2cm?
   - Which is used by commercial marking robots?

4. **What does PX4's internal controller do?**
   - PX4 rover has its own position/velocity controller (RoverPositionControl)
   - In OFFBOARD mode, does PX4's internal controller run, or is it bypassed?
   - If we send position setpoints: does PX4 compute the velocity + heading internally?
   - If we send velocity setpoints: does PX4 just do motor mixing?
   - What about sending yaw + forward speed directly?

5. **Setpoint type selection:**
   - Position setpoints (`type_mask=3580`): PX4 does position control internally
   - Velocity setpoints (`type_mask=3527`): Jetson does trajectory following, PX4 does motor mixing
   - Which gives better accuracy for arc following?
   - Can we combine: position for station-keeping, velocity for transit?

6. **What is the RPP (Rover Pure Pursuit) controller?**
   - Is this a PX4 internal module or a ROS2 node?
   - How does PX4's `RoverPositionControl` relate to pure pursuit?
   - Should we implement pure pursuit on Jetson (ROS2 node) or use PX4's internal controller?

7. **Control loop timing:**
   - 50Hz setpoint stream → how fast must the controller compute?
   - What latency is acceptable between position measurement and setpoint output?
   - Does MAVROS2 add latency? How much?

## Our Context

- OFFBOARD mode: Jetson sends setpoints to `/mavros/setpoint_raw/local` at 50Hz
- All setpoints in FRAME_LOCAL_NED (1) — body offset frame (9) rejected by PX4
- For velocity: body→NED transform needed (heading from EKF2 via `/mavros/local_position/pose`)
- PX4 has internal RoverPositionControl + DifferentialVelControl
- P3 (reverse) and P4 (heading hold) patches in firmware but not validated yet
- Current accuracy: unknown (no RTK for testing)

## Deliverable

1. Control pipeline diagram: Trajectory → [Controller(s)] → Setpoint → PX4 → Motors
2. Which controllers run on Jetson vs PX4 (clear boundary)
3. Recommended controller for Phase 2 (simplest that achieves ±2cm)
4. Setpoint type recommendation (position vs velocity vs hybrid)
5. RPP controller explanation and how it fits our architecture