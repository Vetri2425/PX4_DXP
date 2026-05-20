**Control Pipeline: Trajectory → OFFBOARD Setpoints → PX4 Motors (Differential Drive Rover)**

**TL;DR**  
PX4 (v1.15+/main, especially v1.16+ rover firmware) handles low-level differential mixing and velocity/position control internally. For custom dense trajectories (marking paths with arcs/corners), the **recommended pipeline** runs a simple geometric **Pure Pursuit** (or Regulated Pure Pursuit) controller on the Jetson (ROS2 node). It outputs velocity setpoints (forward speed + yaw rate, transformed to NED) published at 50 Hz via MAVROS `/mavros/setpoint_raw/local` (type_mask for velocity). PX4 bypasses its high-level guidance and performs motor mixing only. This is simplest for ±2 cm at 0.3–0.4 m/s. Position setpoints let PX4 run its internal pure-pursuit-based guidance (good for sparse waypoints/station-keeping); velocity gives tighter external control for arcs. MAVROS2 (MAVLink) adds minor latency; 50 Hz is sufficient with <20–50 ms total loop delay.

### 1. Full Control Pipeline & Controllers (Jetson vs PX4 Boundary)

**High-level flow (your context):**
1. **Trajectory** (Jetson): List of (x, y, heading, speed) points or spline + velocity profile.
2. **Path-following controller** (Jetson ROS2 node — **recommended**): Pure Pursuit / Stanley / MPC → desired body-frame v_forward and ω (yaw rate) or heading.
3. **Setpoint publisher** (Jetson): Transform to NED (using EKF2 heading from `/mavros/local_position/pose`), publish `mavros_msgs/PositionTarget` (or ROS2 equivalent) at **50 Hz** to `/mavros/setpoint_raw/local`.
4. **MAVROS → MAVLink**: `SET_POSITION_TARGET_LOCAL_NED` (FRAME_LOCAL_NED = 1).
5. **PX4 OFFBOARD**:
   - If **velocity setpoint** (recommended for transit/arcs): PX4 skips high-level position/guidance → `DifferentialVelControl` (or equivalent in new differential module) → motor mixing (left/right wheel speeds from v and ω).
   - If **position setpoint**: PX4 runs full internal controller (pure pursuit guidance → velocity/heading → mixing).
6. **PX4 low-level**: Actuator mixing → ESC/motors (differential: v_l = v – (ω · track/2), v_r = v + (ω · track/2)).

**Clear boundary**:
- **Jetson (external)**: Trajectory generation, path following (Pure Pursuit etc.), velocity/heading computation, body→NED transform, 50 Hz streaming + OffboardControlMode heartbeat (ROS2) or MAVLink proof-of-life.
- **PX4 (internal)**: Velocity/attitude/rate mixing, motor allocation, safety limits, EKF2 odometry. In new v1.16+ differential rover module, a shared **pure pursuit guidance library** is used for position/mission modes.

**Deprecated/legacy note**: Old `RoverPositionControl` (pre-v1.16) used L1/pure-pursuit-like logic and had limited OFFBOARD velocity support (added ~2019). New architecture (separate Ackermann/Differential/Mecanum modules + rover setpoints) is cleaner.

**Rover-specific OFFBOARD setpoints (PX4 main / ROS2 native, preferred long-term)**: Use `RoverSpeedSetpoint` + `RoverSteeringSetpoint` / `RoverRateSetpoint` (or `RoverPositionSetpoint`) with `OffboardControlMode`. Hierarchy: highest valid setpoint wins and generates lower ones. MAVROS `setpoint_raw` still works via conversion to `TrajectorySetpoint`.

### 2. Pure Pursuit for Differential Drive

**Standard pure pursuit** (Ackermann): Lookahead point on path at distance L; α = angle error; steering δ = atan(2 L sin(α) / L²) or curvature κ = 2 sin(α)/L.

**Adaptation for differential drive** (no steering angle):
- Compute lookahead point P_ld on path (L ahead of closest point or along path arc length).
- α = heading error to P_ld.
- Desired curvature κ = 2 sin(α) / L.
- Forward speed v from profile (or constant 0.3–0.4 m/s).
- Yaw rate ω = v · κ.
- Wheel speeds: v_left = v – (ω · d/2), v_right = v + (ω · d/2) where d = track width (RD_WHEEL_TRACK param in PX4).
- Publish as velocity setpoint (NED vx, vy derived from v/heading, or body + yaw_rate).

**Lookahead distance for 0.3–0.4 m/s & ±2 cm**:
- Velocity-scaled: L = k · v (k ≈ 1–2 s) with min L ≈ 0.15–0.25 m, max ≈ 0.8–1.0 m.
- At 0.35 m/s → L ≈ 0.35–0.7 m typical starting point. Tune smaller for tighter tracking (risk of oscillation); larger for smoothness (risk of corner cutting).
- Use **Regulated Pure Pursuit** (Nav2 style): adaptive L + rotation-to-path/ goal logic + oscillation damping. Excellent for slow precise work.

**Behavior**:
- **Straight lines**: Excellent (follows exactly if tuned).
- **Arcs/curves**: Good approximation; smaller L = tighter radius tracking.
- **Sharp corners**: Cuts corners unless L small or path densely sampled/spline-smoothed; may need hybrid (slow + small L or switch to Stanley near corners).
- **U-turns**: Possible; use small L or explicit rotate-in-place (if supported) + forward.

Many sources confirm this works well on differential rovers/skid-steer at low speeds.

### 3. Pure Pursuit vs Stanley vs MPC for Differential Marking Rover

| Controller | Pros | Cons | ±2 cm Feasibility (0.3–0.4 m/s) | Implementation Complexity | Commercial Use |
|------------|------|------|---------------------------------|---------------------------|---------------|
| **Pure Pursuit** (geometric) | Simple, stable, smooth outputs, easy to tune (mainly L), low compute | Cuts corners at speed, sensitive to L tuning | Good–Excellent (tune L + add regulation) | Very low (ROS2 node, ~100 lines) | Very common (agriculture, lawn robots, UGVs) |
| **Stanley** | Excellent cross-track error, uses heading + lateral error | Can be aggressive/oscillatory, more tuning (gains + softening) | Excellent | Low–Medium | Common in DARPA-era + modern AVs; good for precision |
| **MPC** | Optimal, handles constraints (vel/acc/jerk), predictive, multi-objective | Heavy compute/tuning (model, weights, horizon), overkill for slow rover | Best (if tuned) | High | Used in high-end commercial (some marking/AGVs) but rare for simple rovers |

**Recommendation for Phase 2 (simplest ±2 cm)**: **Pure Pursuit (or Regulated Pure Pursuit)** on Jetson. Sufficient for marking accuracy at low speed with good odometry/RTK. Add Stanley if cross-track >2 cm persists. MPC only if you need obstacle avoidance or complex optimization later.

**Sources for comparison**: Multiple robotics papers + Nav2/ROS implementations show Pure Pursuit sufficient for <5 cm at low speed; Stanley edges it on lateral error; MPC wins on complex scenarios but not justified here.

### 4. What PX4 Internal Controller Does in OFFBOARD

- **Position setpoints** (`type_mask` ignore velocity/acc): PX4 runs full `RoverPositionControl` / new differential position module → pure pursuit guidance (lookahead circle intersection) → velocity/heading setpoint → vel controller → mixer.
- **Velocity setpoints** (recommended): PX4 largely bypasses guidance → directly to velocity controller / `DifferentialVelControl` → motor mixing (left/right from v + ω). Yaw often derived from velocity vector direction (for Ackermann/diff; yaw/yaw_rate ignored in some MAVLink paths).
- **Yaw + forward speed**: Supported in some combinations; PX4 derives commands accordingly.
- New v1.16+ rover modules use shared pure pursuit library for consistency across modes.

**RPP (Rover Pure Pursuit) controller**: This refers to PX4’s internal pure pursuit guidance (used in legacy `RoverPositionControl` and new differential module’s position/auto modes). It is **not** a standalone ROS2 node — it is firmware-internal. For your custom dense trajectories (not sparse mission waypoints), implement on Jetson and send velocity setpoints. Use PX4 internal only for simple position holding or mission mode.

### 5. Setpoint Type Recommendation

- **Velocity setpoints (type_mask ≈ 3527 or velocity-only flags)**: **Best for arc/transit following**. Jetson owns the trajectory logic → PX4 just mixes. Highest accuracy/control for your use case.
- **Position setpoints (type_mask ≈ 3580)**: Good for station-keeping or when you want PX4 to handle simple following. PX4 pure pursuit runs internally.
- **Hybrid (recommended)**: Velocity for moving along path; switch to position (or zero-velocity) at key points/station-keeping. Or use new `RoverSpeedSetpoint` + `RoverSteeringSetpoint` (if migrating to native ROS2 px4_msgs).
- **Accuracy for arcs**: Velocity (external controller) > Position (PX4 internal). Body-frame velocity + yaw_rate often cleanest for diff drive.

**MAVROS note**: Use `setpoint_raw/local` with correct `type_mask` (ignore unwanted fields) and `FRAME_LOCAL_NED`. Body velocity requires heading transform. 50 Hz is standard and well-supported.

### 6. Control Loop Timing & Latency

- **50 Hz setpoint stream**: Ideal. Controller should compute at ≥50 Hz (ideally 100 Hz internal).
- **Acceptable latency**: <20–50 ms total (position measurement → setpoint output). Pure Pursuit is fast (<1 ms).
- **MAVROS2/MAVLink overhead**: Typically 5–20 ms; monitor with `rostopic hz` and PX4 logs (`mavlink status`). Use high-quality link (USB/UART).
- Failsafe: PX4 requires ~2 Hz proof-of-life (OffboardControlMode or MAVLink stream); drops to failsafe after `COM_OF_LOSS_T`.

### 7. Actionable Recommendations for Phase 2

1. **Implement on Jetson**: ROS2 node with Regulated Pure Pursuit (adapt from Nav2 or simple geometric version). Output `geometry_msgs/Twist` or direct `PositionTarget` (velocity fields + yaw_rate).
2. **Setpoint choice**: Velocity-first (type_mask velocity). Publish continuously at 50 Hz once in OFFBOARD.
3. **Tuning priorities**: Lookahead L (start 0.3–0.5 m), wheel track (PX4 param), speed profile (smooth accel/decel), odometry quality (RTK strongly recommended for ±2 cm).
4. **Validation**: SITL first (Gazebo rover), then hardware with motion capture or RTK. Log cross-track error.
5. **Future-proof**: Migrate toward native ROS2 `Rover*Setpoint` + `OffboardControlMode` (px4_ros_com / microXRCE) for cleaner hierarchy.
6. **Accuracy enablers**: RTK GPS or visual odometry, low speed, path smoothing (splines), possibly add small Stanley correction term.

**Sources** (key ones; full cross-referenced from PX4 docs, GitHub PRs/issues, robotics papers, Nav2, community discussions 2022–2025):
- PX4 Offboard docs (main) — rover setpoints & hierarchy.
- PX4 v1.16 release notes & differential rover pages.
- GitHub PRs: #13225 (velocity OFFBOARD), #25074 (rover setpoints DDS), rover module refactors.
- Pure Pursuit literature: MathWorks, Nav2 Regulated Pure Pursuit, multiple IEEE/ arXiv papers on differential adaptation & lookahead tuning.
- Community: PX4 Discuss threads on rover OFFBOARD, MAVROS setpoint_raw behavior.

This pipeline gives you full external trajectory control while leveraging PX4’s robust low-level differential mixing. Pure Pursuit on Jetson + velocity setpoints is the pragmatic, simplest path to ±2 cm marking accuracy. Let me know if you need code skeletons, parameter examples, or deeper dives into any part (e.g., exact type_mask values or ROS2 implementation).