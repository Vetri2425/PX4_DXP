# Control Pipeline (Trajectory → Commands)

The **high-level trajectory** is first processed on the Jetson (in ROS2).  For example, a list of GPS/local waypoints or a generated path is fed into a path‐following controller (Pure Pursuit, Stanley, MPC, etc.).  This controller computes intermediate targets (either next positions or velocity vectors) along the path.  A speed (velocity) profile may also be applied to obey acceleration limits.  The Jetson then publishes setpoints (at 50 Hz) via MAVROS to PX4’s OFFBOARD interface. 

On the PX4 side, the **RoverPositionControl** module handles incoming setpoints.  In **position mode** (sending `SET_POSITION_TARGET_LOCAL_NED` with X,Y positions), PX4 runs its built-in L1 pure‐pursuit controller: it takes the current and previous waypoint and computes a desired yaw and throttle.  In effect, PX4’s controller “pursues” a lookahead point on the line connecting waypoints to generate a turn angle【93†L2141-L2144】.  In **velocity mode** (sending only velocity setpoints), PX4 instead executes a simple velocity controller: it PID‐tracks the commanded forward speed and computes yaw = `atan2(vy, vx)` (turning to face the velocity vector)【104†L7-L10】.  In both cases the outputs are throttle and yaw demand.  Finally, PX4’s low-level differential‐drive mixer converts these demands into left/right wheel speeds (motor commands). 

In summary: Jetson (ROS2) runs the **path-following** and (if desired) *speed* or *heading* controller to generate position/velocity setpoints. PX4 runs the internal **RoverPositionControl** (pure pursuit + velocity PID) or velocity control, and the **motor mixer** to drive the wheels. 

# Pure Pursuit on a Differential-Drive Robot

**Standard Pure Pursuit** (for a car) computes a steering angle from a lookahead circle intersection.  For a differential drive, we do something equivalent: compute the curvature from the lookahead point, then translate it to wheel speeds via inverse kinematics.  In practice, one computes an angular velocity $\omega = v \,\kappa$ (where $v$ is forward speed and $\kappa$ curvature), and then sets 
```
v_left  = v − (b/2)·ω,  
v_right = v + (b/2)·ω, 
``` 
with $b$ = wheelbase.  This steers the robot on the desired circle.  (Equivalently, PX4’s control computes a desired heading `atan2(vy,vx)` and scales it to a yaw command【104†L7-L10】.) 

**Lookahead distance:**  A typical choice is on the order of the vehicle’s speed.  As one MATLAB example notes, “the look-ahead distance should be larger than the desired linear velocity for smooth path” and using 0.3 m at 0.6 m/s is effective【96†L148-L155】.  In Nav2’s regulated pure pursuit, they use ~0.6 m at 0.5 m/s (min 0.3, max 0.9)【58†L402-L408】.  For our 0.3–0.4 m/s speed, a lookahead of roughly 0.3–0.5 m is reasonable (start around 1×speed).  A larger lookahead smooths motion (less oscillation) but cuts corners; a smaller lookahead tightens tracking but can cause oscillations【96†L148-L155】【58†L480-L484】.  Tuning rules are: if you see wobble, **increase** lookahead; if you lag the path, **decrease** it【58†L480-L484】. 

**Behavior:**  On straight segments and gentle arcs, pure pursuit will smoothly follow the path.  At sharp corners or U-turns, pure pursuit may “cut” or under-turn, since it follows a circular arc.  In extreme cases (tight U-turns) it may overshoot and need corrective oscillation.  Indeed, pure pursuit *cannot* perfectly trace a path with finite lookahead – sharp corners degrade performance【60†L201-L204】.  However, it is very simple to implement and works well for gently curving paths.

# Stanley vs Pure Pursuit vs MPC

- **Pure Pursuit:**  Geometric, easy to implement and tune (just lookahead length).  It “pursues” a forward point and tends to be smooth【103†L61-L66】.  Its drawback is corner-cutting or oscillation on sharp turns (lack of heading error correction)【103†L61-L66】【60†L201-L204】.  

- **Stanley Controller:**  A geometric controller that explicitly uses both cross-track and heading error【103†L68-L74】.  It can give tighter tracking: e.g. one user saw lateral error *halved* by switching from pure pursuit to Stanley on a diff-drive robot【69†L153-L160】.  In theory, a well-tuned Stanley will not cut corners but may overshoot turns, and it requires a good heading estimate and tuning of gains【103†L68-L74】【69†L153-L160】.  It is somewhat more complex than pure pursuit because of the extra heading term.  

- **MPC (Model Predictive Control):**  Uses a dynamic model to optimize future control actions.  It generally offers the best tracking accuracy and handles dynamic constraints, but requires much more computation and a vehicle model and solver.  For a slow marking robot (0.3 m/s) MPC may be overkill.  It has been used in high-end systems (e.g. autonomous cars) where ±cm accuracy is critical, but implementing a real-time MPC on Jetson is substantially more complex.  

**Recommendation for ±2 cm accuracy:**  A carefully tuned pure pursuit or Stanley controller should suffice at our speeds. Pure Pursuit is the simplest (only a lookahead and desired speed), but may require smaller lookahead and low speed for 2 cm precision. Stanley can give better steady-state error but needs accurate heading feedback. MPC would likely meet the spec but is “hardcore”. Most commercial marking or farm robots appear to use variants of pure-pursuit or simple geometric controllers (often combined with precise GNSS); explicit use of MPC in ag robots is rare. In practice, start with pure pursuit (or Nav2’s RPP) and tune it; if that fails, a tuned Stanley is the next step.  

# PX4’s Internal Rover Controllers (RoverPositionControl)

PX4’s firmware includes a **RoverPositionControl** module for navigation.  This module runs continuously (unless in MANUAL) and executes either position or velocity control based on the incoming setpoint.  In OFFBOARD mode, **PX4 still runs these controllers internally** – they are not bypassed.  In detail:

- **Position setpoints (`SET_POSITION_TARGET_LOCAL_NED` with X,Y):** RoverPositionControl’s `control_position()` is invoked【93†L2141-L2144】.  It uses an L1 (pure-pursuit) algorithm on the path segment to compute the desired yaw setpoint, and a speed PID to achieve the commanded speed【93†L2141-L2144】.  The code (shown above) navigates from the previous waypoint to the current waypoint, sets throttle to a mission speed, then computes a turning radius and scales it to a yaw control effort【76†L561-L569】.  In short, PX4 takes your target position and drives to it with its internal pure-pursuit + PID yaw controller.  

- **Velocity setpoints (`SET_POSITION_TARGET_LOCAL_NED` with VX, VY):** RoverPositionControl’s `control_velocity()` is used【76†L611-L619】.  This takes the desired velocity vector, runs a PID on speed (throttle) to match the forward speed, and computes the yaw command as `atan2(vy,vx)` scaled by the max turn angle【104†L7-L10】.  Thus PX4 will drive at the commanded velocity and point towards it.  In this mode, PX4 effectively does only *speed control and yaw mixing* – it does not do any path “pursuit.” 

- **Yaw + throttle (attitude) setpoints:** If you send a `SET_ATTITUDE_TARGET` message (attitude quaternion + thrust), PX4 will extract only the yaw component and treat thrust as forward speed【102†L2094-L2096】.  PX4’s rover attitude controller then drives that yaw and thrust.  In other words, sending an attitude command lets you directly command the heading and speed (and PX4 will mix that to wheels). 

In short: _If you send positions_, PX4 computes the velocity and heading internally (RoverPositionControl). _If you send velocities_, PX4 only runs a velocity controller and does the motor mixing. PX4’s internal controllers are always active in OFFBOARD – you cannot completely bypass them (unless you go into manual). 

# Setpoint Type: Position vs. Velocity

- **Position setpoints** (`type_mask=3580`, i.e. specify X,Y): PX4 will perform **position control** internally【93†L2141-L2144】.  It uses its pure-pursuit path follower and speed/YAW PID to reach the target, closing the loop on position error. This usually yields tighter path tracking (any drift is corrected by the onboard controller). It’s well-suited for waypoint navigation or when you want PX4 to autonomously steer and stop at each target.

- **Velocity setpoints** (`type_mask=3527`, i.e. specify VX,VY): PX4 will treat them as direct velocity commands. The Jetson must generate a continuous stream of velocity vectors (and optionally yaw) along the trajectory. PX4 will then just try to hold that speed and rotate towards the velocity direction【104†L7-L10】. This is effectively more open-loop and may drift if not perfectly fed, but it offloads path-following entirely to the Jetson. 

Which is more accurate for arcs? In general, position-mode setpoints can be more accurate for curved paths because the controller continually corrects path error【93†L2141-L2144】. Pure velocity commands may cause slight cutting or drift unless the Jetson’s controller constantly adjusts yaw. 

**Hybrid strategy:**  A reasonable approach is to use position commands for precise tasks (e.g. stopping or holding position), and velocity commands for smooth transit. For example, one could send fixed positions at turns or the ends of lines (so PX4 will settle exactly there), and stream velocity setpoints during straight-line motion. This combines the strengths of both modes.

# RPP (“Regulated Pure Pursuit”) Controller

“RPP” refers to **Regulated Pure Pursuit**, a variant of pure-pursuit used in ROS Navigation2 (Nav2)【48†L107-L115】.  It is **not** a PX4-internal module; rather it is a ROS2 path-following plugin. PX4’s built-in `RoverPositionControl` already implements a pure-pursuit (L1) algorithm internally【93†L2141-L2144】. In other words, PX4 **has its own pure-pursuit yaw controller**, so if you use PX4’s position mode you effectively get that behavior “for free.” 

If you wanted to run pure pursuit on the Jetson instead, you could use Nav2’s RPP plugin or a custom ROS2 node. However, since PX4 already handles path following in firmware, it is often simplest to let PX4 do it. Implementing RPP on the Jetson would duplicate PX4’s efforts unless you need some special feature. In our architecture, you could either send position setpoints (and rely on PX4’s internal pure-pursuit), or send raw path points to a ROS2 pure-pursuit node (RPP) on the Jetson, then feed the output to PX4 as velocity commands. Either works, but using PX4’s controller means fewer moving parts. 

# Control Loop Timing and Latency

At a 50 Hz setpoint rate (20 ms period), the Jetson-side controller must compute new commands well within that time (typically in a few milliseconds) to maintain real-time behavior. In practice, dedicating <5 ms for computation per cycle is advisable, leaving margin for communication and scheduling. Sensor-to-actuator latency should ideally be small – on the order of tens of milliseconds. 

MAVROS2/MicroRTPS adds only modest delay at these rates. In fact, ROS1+MAVROS has been demonstrated handling 200 Hz offboard loops with 100 Hz feedback【91†L134-L142】. At 50 Hz, the latency through MAVROS2 or MicroRTPS to PX4 should be only a few milliseconds (well under 10 ms) on a good network. The PX4 side processes global/local position at ~50 Hz internally (see `orb_set_interval` in RoverPositionControl)【71†L815-L822】. In summary, a 50 Hz pipeline is feasible: the Jetson controller should run faster than 20 ms per loop, and any added communication delay is small. 

**Sources:** PX4 Rover docs and code【93†L2141-L2144】【102†L2094-L2096】, PX4 Offboard modes【33†L2052-L2060】【102†L2094-L2096】, Nav2/Pure-Pursuit tuning guides【96†L148-L155】【58†L402-L408】, path-tracking literature【60†L201-L204】【103†L61-L74】, and community experiments【91†L134-L142】【69†L153-L160】.