# In-Depth Technical Report: Controller Pipeline Architecture from Trajectory to OFFBOARD Commands for Differential Drive UGVs

## Control Pipeline Topology from Trajectory to Motors

Autonomous path-following for a differential drive Unmanned Ground Vehicle (UGV) requires a distributed, multi-rate control pipeline. This pipeline bridges the high-level, computationally intensive motion planning on the companion computer with the deterministic, real-time actuator control on the flight controller.

The logical and physical data flow spans multiple coordinate systems, communication interfaces, and control loops to translate a list of target coordinates into physical wheel velocities.

The sequence of operations is structured as follows:

```
  Global Trajectory (Waypoints in map/odom frame)
         │
         ▼
  Regulated Pure Pursuit (RPP) Controller (50 Hz)
         │  (Computes body-relative command: v_x, ω_z)
         ▼
  Frame Transformer (50 Hz)
         │  (Reads EKF2 heading; projects body velocities to Local NED)
         ▼
  MAVROS2 Node (50 Hz)
         │  (Serializes NED commands into MAVLink SET_POSITION_TARGET_LOCAL_NED)
         └───► ───┐
                                                   │
┌──────────────────────────────────────────────────┘
│
▼
  uORB Topic: OffboardControlMode & TrajectorySetpoint
         │
         ▼
  DifferentialVelControl Module (200+ Hz)
         │  (Bypasses RoverPositionControl; Projects NED velocities back to body;
         │   Resolves differential kinematics to compute target wheel speeds: v_L, v_R)
         ▼
  Closed-Loop Wheel Speed Controller (200+ Hz)
         │  (Executes high-frequency PID control; compares targets to DShot RPM)
         ▼
  Actuator Mixer / DShot Driver
         │  (Generates individual motor ESC telemetry/control frames)
         ▼
  Physical Brushless DC Motors & Wheels
```

## Allocation of Controllers Across Jetson and PX4 Autopilot

The division of control tasks between the NVIDIA Jetson and the PX4 Autopilot is governed by the need for low-latency, deterministic execution of safety-critical loops, and the high-throughput processing required for spatial mapping, obstacle costmaps, and non-linear trajectory planning.

The boundary is physically defined by the serial connection carrying MAVLink or XRCE-DDS messages, and logically defined by the OffboardControlMode flags that dictate which PX4 modules remain active.

| Controller Layer | Executing Platform | Run Frequency | Computational Complexity | Inputs | Outputs | Primary Function |
|---|---|---|---|---|---|---|
| Path-Following Controller (e.g., RPP) | Jetson (ROS2 Nav2) | 50 Hz | Moderate to High (Iterates over local costmaps and path points) | Waypoint array, EKF2 position, local costmap | Body-frame linear velocity ($v_x$), angular velocity ($\omega_z$) | Computes immediate geometric steering commands to track the reference path. |
| Velocity Profiler (Speed Controller) | Jetson (ROS2 Nav2) | 50 Hz | Low (Evaluates geometric constraint equations) | Local path curvature, proximity to obstacle inflation layers | Regulated linear velocity ($v_{regulated}$) | Scales forward velocity to prevent overshoot on curves and avoid obstacle collisions. |
| Heading Controller (Yaw Tracking) | Jetson (ROS2 Nav2) | 50 Hz | Low (Proportional controller) | Current heading, lookahead point orientation | Target angular velocity ($\omega_{regulated}$) | Aligns the vehicle body with the local tangent of the path. |
| Low-Level Motor Controller (DifferentialVelControl) | PX4 Autopilot | 200+ Hz | Minimal (Simple arithmetic and closed-loop PID) | Local NED velocities, wheel encoder/DShot RPM feedback | Normalized individual wheel motor commands | Translates inertial velocity commands into target wheel speeds and regulates them via closed-loop PID. |
| Differential Drive Actuator Mixer | PX4 Autopilot | 200+ Hz | Negligible (Direct pin mapping and scaling) | Normalized wheel motor commands, physical vehicle geometry parameters | Individual ESC command signals (DShot/PWM duty cycles) | Resolves normalized commands into individual physical ESC signals. |

## Kinematics and Adaptation of Pure Pursuit for Differential Drive Platforms

### Kinematic Adaptations

The classical Pure Pursuit formulation is mathematically derived using a kinematic bicycle model designed for Ackermann-steered vehicles. It calculates a steering angle command $\delta$ to target a lookahead point $G = (x_g, y_g)$ located at a lookahead distance $l_d$ in the vehicle's local coordinate frame:

$$\delta = \arctan\left(\frac{2 L \sin\alpha}{l_d}\right)$$

where $L$ represents the vehicle's physical wheelbase, and $\alpha$ is the angle between the vehicle's current heading vector and the lookahead vector, defined as:

$$\sin\alpha = \frac{y_g}{l_d}$$

For a differential drive robot, there is no physical steering joint or steering angle $\delta$. Instead, the vehicle's heading is modified by generating a difference in rotational velocity between the left and right wheels. To adapt the Pure Pursuit algorithm, the target path curvature $K$ must be extracted directly. Curvature is mathematically defined as the inverse of the radius $R$ of the target circular arc joining the robot's current position to the lookahead point:

$$K = \frac{1}{R} = \frac{2 \sin\alpha}{l_d} = \frac{2 y_g}{l_d^2}$$

Given a desired linear forward velocity $v$ (commanded by the trajectory velocity profiler), the required angular velocity $\omega_z$ (yaw rate) about the robot's center of rotation is computed by projecting this curvature onto the linear speed:

$$\omega_z = v \cdot K = \frac{2 v y_g}{l_d^2}$$

Using the differential drive kinematic model, these body-relative commands ($v, \omega_z$) are mapped to individual target wheel velocities for the left ($v_L$) and right ($v_R$) wheels using the physical wheel track width $W$ (defined in the PX4 firmware via the parameter RD_WHEEL_TRACK):

$$v_L = v - \frac{\omega_z W}{2} = v \left(1 - \frac{y_g W}{l_d^2}\right)$$

$$v_R = v + \frac{\omega_z W}{2} = v \left(1 + \frac{y_g W}{l_d^2}\right)$$

### Lookahead Distance Optimization for Highly Accurate Low-Speed Tracking

To achieve the targeted $\pm 2\text{ cm}$ cross-track accuracy at slow operating speeds of $0.3\text{ m/s}$ to $0.4\text{ m/s}$, the lookahead distance $l_d$ must be dynamically scaled.

If $l_d$ is kept constant, a major trade-off occurs. If $l_d$ is too small (e.g., $< 0.2\text{ m}$), the controller becomes highly sensitive to high-frequency noise from the EKF2 position estimator, leading to limit cycle oscillations and rapid motor chatter. If $l_d$ is too large (e.g., $> 0.8\text{ m}$), the tracking accuracy degrades, causing the vehicle to cut corners and fail the $\pm 2\text{ cm}$ requirement.

To resolve this, the lookahead distance must be dynamically scaled as a function of the instantaneous linear velocity $v$, bounded by strict minimum and maximum thresholds:

$$l_d = \max\left(l_{d,\min}, \min\left(l_{d,\max}, v \cdot t_{lookahead}\right)\right)$$

At $0.3\text{ m/s} \le v \le 0.4\text{ m/s}$, the following parameter values are mathematically optimal:

- **$l_{d,\min} = 0.30\text{ m}$**: Prevents the lookahead point from falling inside the robot's physical frame and acting as a high-gain noise amplifier when starting from a standstill or moving slowly.
- **$l_{d,\max} = 0.60\text{ m}$**: Limits the lookahead distance when running at maximum speed, forcing the robot to adhere tightly to curved segments.
- **$t_{lookahead} = 1.2\text{ s}$**: Acts as the lookahead gain. At $0.3\text{ m/s}$, the un-saturated lookahead is $0.3 \cdot 1.2 = 0.36\text{ m}$. At $0.4\text{ m/s}$, it is $0.48\text{ m}$. These distances fall safely within the optimized tracking range.

### Pure Pursuit Primitive Execution Analysis

**Straight Lines:** On linear path segments, the cross-track error $y_g$ is minimal. The lookahead distance scales toward its upper range ($0.45\text{ m} - 0.48\text{ m}$), smoothing out minor localization errors and tracking the line with high stability and zero oscillation.

**Constant-Radius Arcs:** When tracking a smooth curve, Pure Pursuit exhibits a systematic steady-state lateral tracking offset. Because the algorithm is always targeting a point ahead, the robot "cuts" the arc, tracking a circle of slightly larger radius. For a path of radius $R_{path}$ and a lookahead distance $l_d$, the steady-state cross-track error $e_{ss}$ is approximated by:

$$e_{ss} \approx R_{path} - \sqrt{R_{path}^2 - \frac{l_d^2}{4}}$$

At $R_{path} = 2.0\text{ m}$ and $l_d = 0.36\text{ m}$, $e_{ss} \approx 0.008\text{ m}$ ($0.8\text{ cm}$), which remains within the $\pm 2\text{ cm}$ tracking envelope.

**Sharp Corners:** Traditional Pure Pursuit cannot track sharp, discontinuous corners without significant overshoot. To maintain the $\pm 2\text{ cm}$ threshold, the companion computer's trajectory manager must recognize sharp corners in the global path and transition the controller to a "spot turn" state. When heading error $\theta_e$ exceeds a critical threshold (e.g., $30^\circ$, configured via RD_TRANS_DRV_TRN), the robot halts linear translation ($v = 0$), rotates on its center of gravity until the heading aligns with the next path segment (under a threshold configured via RD_TRANS_TRN_DRV), and then resumes forward motion.

**U-Turns:** A U-turn is handled as a localized extreme-curvature corner. The velocity-scaled lookahead controller reduces linear speed to its minimum allowed value ($0.15\text{ m/s}$), causing the lookahead distance to compress to $l_{d,\min} = 0.30\text{ m}$. This tightens the turning radius of the tracking arc. If the curve is too tight, the spot-turn transition triggers, forcing a clean, zero-radius rotation to prevent lateral sliding and turf damage.

## Comparative Analysis of Path Tracking Controllers

Selecting the correct path tracking controller requires balancing tracking accuracy, low-speed stability, computational cost, and ease of integration. The table below presents a comparative analysis of three prominent GNC control algorithms adjusted for a differential drive marking rover:

| Evaluation Criterion | Pure Pursuit (Standard / Velocity-Scaled) | Stanley Controller | Model Predictive Control (MPC) |
|---|---|---|---|
| Mathematical Control Law | $\omega_z = \frac{2 v y_g}{l_d^2}$ | $\omega_z = v \left[ \frac{\theta_e + \arctan\left(\frac{k e}{v + \epsilon}\right)}{L_{virtual}} \right]$ | $\min_{u} \sum_{t=0}^{N} \|x_t - x_{sp}\|_Q^2 + \|u_t\|$ subject to kinematic state equations |
| Computational Footprint | Negligible ($< 0.1\text{ ms}$ per iteration; easily runs at $> 1000\text{ Hz}$) | Negligible ($< 0.1\text{ ms}$ per iteration; easily runs at $> 1000\text{ Hz}$) | Moderate to High ($2\text{ ms} - 15\text{ ms}$ depending on horizon length $N$ and solver optimization) |
| Curvature Sensitivity | Low. Tends to cut corners and truncate high-curvature segments. | High. Explicitly incorporates path heading and matches path curvature. | Excellent. Explicitly models and anticipates future path curvature changes. |
| Standstill / Low-Speed Stability | High. Bounded by $l_{d,\min}$, preventing division-by-zero singularities. | Poor. As $v \to 0$, the feedback gain approaches infinity, causing extreme steering jitter. | High. Handles zero speed constraints natively within the optimization solver. |
| Sensitivity to Localization Noise | Low. The lookahead distance acts as a physical low-pass filter on high-frequency pose jitter. | High. Direct coupling of lateral error $e$ causes localized EKF2 position jitter to translate to motor chatter. | Moderate. State estimation noise must be mitigated via state observers or Kalman filtering. |
| Implementation Complexity | Minimal (Trivial geometric equations; $< 100$ lines of code). | Low to Moderate (Requires calculation of closest-point projection and virtual wheelbase). | Extremely High (Requires QP/IP solver, dynamic system identification, and complex tuning). |
| Suitability for $\pm 2\text{ cm}$ Marking | High (When paired with Regulated Pure Pursuit speed-scaling heuristics). | Moderate (High accuracy on arcs, but low-speed noise susceptibility causes uneven paint lines). | Optimal (Technically achieves sub-centimeter tracking but overkill for Phase 2). |

### Qualitative Control Trade-Offs

The Stanley Controller provides superior tracking accuracy on continuous curves because it measures lateral error at the front axle and steering is driven to actively align the vehicle body with the local tangent of the path. However, when adapted to a differential drive vehicle, the steering angle $\delta$ must be mapped back to yaw rate using a "virtual wheelbase" $L_{virtual}$.

At low speeds ($0.3\text{ m/s}$), the Stanley formulation becomes numerically sensitive. Even with a softening parameter $\epsilon$, localization noise from a standard non-RTK EKF2 state estimator causes the heading error and lateral error terms to generate rapid, alternating steering commands. On a lawn or running track, this rapid chattering of the differential motors causes mechanical stress, high power consumption, and physical "weaving" of the marking line.

Model Predictive Control (MPC) represents the theoretical peak of tracking performance. It solves a constrained optimization problem at every time step, allowing the rover to slow down before entering a turn, explicitly preventing wheel slip on wet grass, and compensating for mechanical actuator delay.

The primary drawback is its complexity. Accurate MPC requires a high-fidelity dynamic model of the rover, including tire-soil interaction coefficients that change between dry dirt, wet grass, and asphalt. If the model is inaccurate, MPC can easily become unstable or execute erratic movements.

Pure Pursuit is highly robust. Because it targets a lookahead point ahead of the vehicle, it behaves like a physical tether pulling the vehicle along the path. This geometric formulation acts as a natural low-pass filter, smoothing out high-frequency localization drift and sensor noise.

The primary historical limitation—corner cutting—is successfully resolved by the Regulated Pure Pursuit (RPP) variant. By dynamically slowing down the robot's forward progress when path curvature increases, the tracking arc tightens, allowing the robot to negotiate curves with minimal overshoot and maintain the strict $\pm 2\text{ cm}$ limit.

### Commercial Systems Context & Phase 2 Recommendation

In the commercial field-marking and turf-printing industry, robots prioritizes reliability, deterministic safety, and ease of field calibration. Most of these commercial systems deploy optimized variations of Regulated Pure Pursuit or Adaptive Pure Pursuit on their companion computers, coupled with high-accuracy RTK-GPS and wheel encoder fusion on the low-level flight controller. MPC is occasionally utilized in ultra-high-speed agricultural platforms, but is generally avoided in lightweight field markers due to the difficulty of real-time parameter tuning in changing soil conditions.

**Recommendation for Phase 2:** The development team should implement the Regulated Pure Pursuit (RPP) Controller running on the Jetson companion computer. It provides the simplest mathematical structure that is mathematically capable of achieving the $\pm 2\text{ cm}$ target at $0.3-0.4\text{ m/s}$, is highly robust to EKF2 localization noise, and eliminates the instability risks associated with low-speed Stanley controllers or poorly modeled MPC algorithms.

## PX4 Autopilot Internal Controller Behavior in OFFBOARD Mode

The PX4 flight controller architecture was significantly refactored in version 1.16. The old monolithic RoverPositionControl module (originally derived from fixed-wing L1 guidance) has been deprecated. PX4 now uses modular, drive-specific modules for Ackermann, differential, and mecanum rovers. For differential vehicles, the primary internal tasks are divided between high-level path navigation and low-level wheel speed control.

When operating in OFFBOARD mode, the active state of PX4's internal control loops is dictated entirely by the structure of the incoming ROS2 / MAVLink setpoints and the configured fields in the OffboardControlMode message:

### 1. Position Setpoints (type_mask = 3580)

When receiving position setpoints (valid $x, y, z$ coordinates, with velocity and acceleration fields typically set to NaN or used as optional feedforward terms), PX4 retains full control over the trajectory generation and steering kinematics.

**Internal Pipeline:** The incoming position targets populate the internal uORB TrajectorySetpoint topic. PX4's high-level position controller module remains active. It intercepts these coordinates and uses its internal pure pursuit guidance library (governed by parameters like PP_LOOKAHD_GAIN, PP_LOOKAHD_MIN, and PP_LOOKAHD_MAX) to compute a target bearing and local heading error.

**Result:** This computed heading error is fed into a PID heading controller to generate a target yaw rate, while the distance to the waypoint governs the speed profile. The high-level path-following loops on the Jetson are completely bypassed. PX4 computes the velocity and heading commands internally, then passes them to DifferentialVelControl.

### 2. Velocity Setpoints (type_mask = 3527)

When receiving velocity setpoints (position fields set to NaN, velocities $v_N, v_E, v_D$ and/or yaw/yawspeed populated), PX4's high-level position and steering controllers are completely bypassed.

**Internal Pipeline:** PX4 acts strictly as a slave velocity-tracking regulator. The incoming local NED velocity commands bypass RoverPositionControl and are routed directly to the DifferentialVelControl module.

**Result:** The controller projects the NED velocities back into the robot's local body frame, extracts the forward speed target ($v_{body,x}$) and yaw rate target ($\omega_z$), and immediately resolves them into individual left and right target wheel speeds based on the physical track width RD_WHEEL_TRACK. These targets are regulated at high frequency (200+ Hz) using internal PID loops (tuned via RD_SPEED_P and RD_SPEED_I) and feedforward terms (tuned via RO_MAX_THR_SPEED), outputting normalized thrust values directly to the motor mixer.

### 3. Direct Yaw and Forward Speed Control via TrajectorySetpoint

Under legacy PX4 firmware, sending velocity commands to a rover in offboard mode was notoriously unreliable. Because the TrajectorySetpoint.msg was structurally hardcoded for multicopters, the rover position control task did not subscribe to or parse it correctly, resulting in zero actuator output despite active offboard mode.

This severe limitation was resolved in the main development branch by PR #24318. This pull request added native offboard trajectory setpoint support specifically for differential rovers.

Under this updated architecture, the companion computer streams TrajectorySetpoint.msg over the XRCE-DDS bridge. By setting the position array to NaN and populating the linear velocity array ($v_N, v_E, v_D$) alongside a valid yaw or yawspeed target, the low-level differential controller parses these fields directly, allowing seamless steering without the risk of actuator dropouts.

### Coordinate Transformation and Frame Rejection

A critical operational constraint of PX4's offboard mode is that it strictly rejects body-relative offset frames (such as frame 9, MAV_FRAME_BODY_NED) for velocity setpoints in rovers. All incoming commands must be mapped to the inertial Local NED frame (frame 1, MAV_FRAME_LOCAL_NED).

Because the Jetson-based path-following controller naturally computes guidance commands in the UGV's body-relative frame (where linear forward velocity is $v_x$ and lateral velocity $v_y = 0$), the companion computer must perform an active coordinate rotation prior to publishing to MAVROS.

The instantaneous yaw heading $\theta$ of the rover is extracted from EKF2 by subscribing to the /mavros/local_position/pose topic. The body velocity vector is rotated into the Local NED frame using the standard 2D rotation matrix:

$$\begin{bmatrix} v_N \\ v_E \end{bmatrix} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} v_{body, x} \\ 0 \end{bmatrix}$$

$$v_D = 0$$

These rotated coordinates, along with the desired yaw and yawspeed targets, are populated into the TrajectorySetpoint message and streamed over the MAVROS channel.

## The RPP Controller and Architectural Integration

### Clarifying the "RPP" Abbreviation

Within this specific system architecture, the term "RPP" must be clearly contextualized, as it can refer to two distinct implementations across the ROS2 and PX4 domains:

- **Regulated Pure Pursuit (RPP) Controller (ROS2 / Jetson):** This is a highly robust local path-tracking controller executed as a plugin on the companion computer. It implements advanced speed regulation based on local costmaps, path curvature, and goal-approach deceleration. It acts as the "brain" of the path-following pipeline.
- **Rover Pure Pursuit (RPP) Guidance (PX4 Autopilot):** This refers to the geometric pure pursuit path-following algorithms compiled internally within PX4's flight stack. It is utilized by PX4 during autonomous Waypoint Missions or when Offboard Position Setpoints are commanded, using parameters such as PP_LOOKAHD_GAIN and PP_LOOKAHD_MIN.

To prevent structural conflicts, the system architecture must enforce a strict separation of concerns. High-level geometric path following should be executed via the ROS2 Regulated Pure Pursuit (RPP) controller on the Jetson, while the internal PX4 pure pursuit guidance is bypassed entirely by utilizing offboard velocity setpoints. This design yields massive architectural advantages:

```
┌──────────────────────────────┐                       ┌──────────────────────────────┐
│ ROS2 Nav2 RPP Controller     │                       │ DifferentialVelControl       │
│ • Reads local costmaps       │  Velocity Setpoints   │ • Closed-loop wheel control  │
│ • Curvature-based slowdown   ├──────────────────────►│ • 200+ Hz PID tracking       │
│ • Handles reverse & heading  │  (Local NED frame)    │ • Standardized motor mixing  │
│ • Highly customizable        │                       │ • Hardwired safety checks    │
└──────────────────────────────┘                       └──────────────────────────────┘
        [ Jetson Companion Computer ]                           [ PX4 Autopilot ]
```

### Architectural Implications of the P3 and P4 Firmware Patches

The rover's firmware contains two critical, unvalidated patches:

- **P3 (Reverse Driving):** This patch addresses the kinematic inversion required when a differential drive robot travels backward. In reverse motion, standard feedback gains for steering and heading alignment must be mathematically inverted; otherwise, the lateral correction creates a positive feedback loop, causing the UGV to spin uncontrollably on its axis.

- **P4 (Heading Hold):** This patch implements a low-speed heading stabilization loop within the flight controller. When driving straight at slow speeds ($0.3\text{ m/s}$), minor physical perturbations (such as uneven grass, loose soil, or asymmetric wheel slippage) cause the robot to drift. A high-rate heading-hold loop uses the EKF2 gyroscopic yaw estimate to continuously apply micro-corrections to the differential motor speeds, forcing a perfectly straight trajectory.

Because these firmware patches are currently unvalidated, and the development team lacks RTK-GPS hardware for testing, running the path-following controller directly on the PX4 flight controller via position setpoints represents a severe risk. If the high-level guidance loop is closed inside PX4, debugging a tracking error becomes extremely difficult, as the developer cannot easily isolate whether a failure originates in the EKF2 state estimator, the legacy RoverPositionControl guidance logic, the P3 reverse kinematics, or the P4 heading-hold loops.

By running the Regulated Pure Pursuit (RPP) controller on the Jetson and transmitting low-level velocity commands, the control loops are isolated.

- **Reverse Path Validation:** The Jetson can execute backward maneuvers by systematically commanding a negative linear velocity ($v_x < 0$) while utilizing its own stable coordinate frame, allowing the development team to isolate and validate the P3 reverse patch on PX4 progressively.
- **Heading Hold Validation:** The Jetson can command a pure linear velocity vector ($v_x = 0.3\text{ m/s}, \omega_z = 0$). By observing the vehicle's physical deviation on grass without RTK, the team can determine whether PX4's internal P4 heading-hold loop is actively rejecting physical slip disturbances or if the patch requires parameter tuning.

## Control Loop Timing and Latency Budget

### Timing Budget of the 50 Hz Control Loop

To ensure smooth, continuous motion without triggering PX4's offboard timeout failsafes, the companion computer must maintain a deterministic publishing rate of $50\text{ Hz}$ ($20\text{ ms}$ time budget per cycle).

A detailed breakdown of the maximum allowable processing and transit times within this $20\text{ ms}$ budget is presented below:

| Pipeline Stage | Processing Task | Target Duration | Budget Allocation (%) | Potential Latency Sources |
|---|---|---|---|---|
| Stage 1: State Estimation | EKF2 position estimation and uORB publishing | $1.5\text{ ms}$ | 7.5% | Sensor processing delay, IMU/encoder hardware integration lag. |
| Stage 2: Transport (Upstream) | Serial/DDS transmission of EKF2 odometry to Jetson | $2.5\text{ ms}$ | 12.5% | High serial bus load, queue bottlenecks on the XRCE-DDS bridge. |
| Stage 3: Local Planner | ROS2 Regulated Pure Pursuit algorithm computation | $1.0\text{ ms}$ | 5.0% | Heavy local costmap updates, high waypoint search-ring densities. |
| Stage 4: Frame Transform | 2D body-to-Local-NED coordinate rotation | $0.2\text{ ms}$ | 1.0% | Thread scheduling latency within the ROS2 executor stack. |
| Stage 5: Transport (Downstream) | MAVROS serialization and transmission of TrajectorySetpoint | $2.5\text{ ms}$ | 12.5% | Serial driver queuing, UART baud rate saturation (e.g., 921600 baud limits). |
| Stage 6: PX4 Execution | DifferentialVelControl and actuator allocation processing | $1.0\text{ ms}$ | 5.0% | Interrupt handling overhead, high uORB message subscription rates. |
| Stage 7: Margin Buffer | Unallocated slack to absorb OS scheduling and network jitter | $11.3\text{ ms}$ | 56.5% | Context switching on non-real-time Linux kernels (Jetson Ubuntu). |

If the Jetson's Ubuntu operating system experiences a scheduling spike and the total loop time exceeds $500\text{ ms}$, PX4's commander module will immediately trigger a loss-of-offboard failsafe (COM_OF_LOSS_T), disengaging autonomous control. Maintaining a lean, highly optimized $50\text{ Hz}$ stream provides a $25\times$ packet margin, ensuring that occasional delays never jeopardize vehicle safety.

### The ~100-150 ms Latency Phenomenon

Developers frequently observe a delay of $\sim 100-150\text{ ms}$ between the moment a ROS2 node publishes a velocity setpoint and the physical acceleration of the vehicle.

Profiling of the communication stack reveals that the physical transmission link is not the primary cause of this delay. ROS2 to MAVROS publish-to-subscribe latency is $\le 1\text{ ms}$, serial transmission takes $\sim 1.5\text{ ms}$, and the MAVROS timesync status confirms a round-trip time (RTT) of only $4-15\text{ ms}$.

Instead, this latency is a composite physical and algorithmic phenomenon:

- **EKF2 Fusion Window Delay ($\sim 30-50\text{ ms}$):** The EKF2 estimator fuses incoming sensor measurements (such as wheel encoders, IMU, and optical flow) through a historical buffer window to output stable local position estimates. This buffering delay is mathematically necessary to resolve state estimation covariance but introduces a phase lag in the high-level path-following loop.
- **Actuator Deadbands and Motor Inertia ($\sim 50-80\text{ ms}$):** Electric motors and ESCs require time to overcome physical rotor inertia and mechanical static friction.
- **PX4 Inner-Loop Filtering ($\sim 20-30\text{ ms}$):** Low-pass filters are applied to the velocity setpoints and control outputs to prevent high-frequency chatter from damaging the physical gearboxes.

A detailed diagnostic and mitigation table for timing-related anomalies in offboard setups is presented below:

| Timing Anomaly | Observed Symptom | Primary Root Cause | Corrective Action / Mitigation |
|---|---|---|---|
| Serial Queue Backups | High control latency ($\sim 500-1000\text{ ms}$), visible lagging in vehicle response, eventual offboard timeout. | Clogging the serial line by streaming high-frequency commands ($>100\text{ Hz}$) alongside dense telemetry. | Limit the offboard publishing rate to $50\text{ Hz}$. Increase UART baud rate to $921600$ or switch to a high-speed Ethernet connection. |
| EKF2 Phase Lag | Vehicle overshoots straight-line paths and oscillates around the center line. | The high-level controller reacts to delayed state estimates, causing the steering corrections to fall out of phase. | Implement a simple velocity-predictor in the Jetson path follower to project the vehicle's pose forward by the estimated EKF2 lag. |
| Motor Deadband Sluggishness | Minor steering commands fail to spin the wheels, causing the vehicle to drift before correcting aggressively. | Physical friction in the motor gearboxes prevents rotation at low throttle inputs. | Implement low-level deadband compensation on the Jetson by adding a static feedforward offset to any non-zero velocity command. |
| Offboard Heartbeat Loss | Frequent, abrupt drops from offboard mode back to manual or position hold. | Jitter in the Jetson's non-real-time OS causes setpoint packet intervals to exceed $500\text{ ms}$. | Assign real-time thread priority (SCHED_FIFO) to the ROS2 MAVROS and path-following nodes on the Jetson. |

## Architectural Implementation Summary

To ensure successful deployment during the Phase 2 testing window, the engineering team must execute the following implementation plan:

1. **Configure PX4 v1.16+:** Select the Generic Rover Differential airframe to activate the modular, redesigned differential drive control allocation loops. Ensure the parameter RD_WHEEL_TRACK is calibrated to the millimeter.

2. **Establish high-speed, low-latency offboard velocity control:** Stream OffboardControlMode and TrajectorySetpoint messages from the Jetson at a rock-solid $50\text{ Hz}$. Banish the legacy, buggy position control loops and utilize velocity setpoints to bypass PX4's internal high-level guidance. This isolates the unvalidated P3 and P4 firmware patches, allowing clean, sandboxed testing.

3. **Perform body-to-local NED frame conversions on the Jetson:** Ensure all velocity commands are actively rotated using the EKF2 heading from /mavros/local_position/pose before publishing to MAVROS, as PX4 will reject body-relative commands.

4. **Deploy Nav2 Regulated Pure Pursuit (RPP) on the Jetson:** Standardize on the RPP plugin for path following, utilizing its dynamic lookahead ($l_{d,\min} = 0.30\text{ m}$, $l_{d,\max} = 0.60\text{ m}$, $t_{lookahead} = 1.2\text{ s}$) and curvature-based velocity scaling to safely achieve $\pm 2\text{ cm}$ relative tracking accuracy at operating speeds of $0.3-0.4\text{ m/s}$ on grass. Ensure a safe manual override remains active by setting COM_RC_OVERRIDE = 1.
