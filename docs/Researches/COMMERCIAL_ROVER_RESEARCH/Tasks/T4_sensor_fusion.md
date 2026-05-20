# T4: Position Feedback & Sensor Fusion

## What to Research

How does the rover know where it is? What sensors provide position and heading, how are they fused, and what accuracy can we achieve? What is the feedback loop that closes the controller?

## Specific Questions

1. **Current position sources:**
   - PX4 EKF2 fuses: IMU + GPS (UM982 single-point) + GPS heading (UM982 dual-antenna)
   - What accuracy does EKF2 provide for position? For heading?
   - Is EKF2 output published on `/mavros/local_position/pose`?
   - What coordinate frame? What latency?

2. **RTK corrections:**
   - Our UM982 receives NTRIP RTCM corrections → RTK Fixed mode
   - What accuracy does RTK Fixed provide? (typical: ±1-2cm horizontal, ±3-5cm vertical)
   - What is RTK Float vs RTK Fixed? How long does convergence take?
   - What happens when NTRIP drops? How fast does fix degrade?

3. **Dual-antenna heading:**
   - UM982 dual-antenna provides heading from carrier phase difference
   - What heading accuracy? (typical: ±0.5° with 1m baseline)
   - Our baseline is ~0.47m (wheel track) — what accuracy does that give?
   - How long does heading initialization take after power-on?
   - What happens when one antenna loses sky view?

4. **Do we need additional sensors?**
   - Wheel encoders / odometry: does PX4 support this for rover?
   - IMU integration: is the built-in IMU in CubeOrangePlus sufficient?
   - Visual odometry: needed? Feasible on Jetson Orin?
   - LiDAR: for obstacle detection? Or just position?

5. **robot_localization (EKF/UKF on ROS2):**
   - Should we fuse additional sensors on Jetson using `robot_localization`?
   - Or is PX4 EKF2 sufficient?
   - What's the latency difference: PX4 EKF2 vs Jetson-side fusion?
   - When is Jetson-side fusion needed? (Phase 3?)

6. **Position feedback for controller:**
   - The controller needs: current position (x,y), current heading (yaw), current speed
   - Where does each come from?
   - `/mavros/local_position/pose`: position + orientation (from EKF2)
   - `/mavros/local_position/velocity_body`: speed (from EKF2)
   - What is the total latency from sensor measurement to setpoint output?
   - Is the feedback fast enough for 50Hz control at 0.4 m/s?

7. **Accuracy budget:**
   - For ±2cm lateral accuracy at 0.4 m/s:
   - How much error comes from position measurement?
   - How much from controller tracking?
   - How much from actuator delay?
   - What is the error breakdown?

## Our Context

- UM982 dual-antenna RTK GNSS on `/dev/ttyUSB0`
- NTRIP corrections injected via `/mavros/gps_rtk/send_rtcm`
- CubeOrangePlus IMU (built-in)
- PX4 EKF2 fuses IMU + GPS + dual-antenna heading
- Currently no wheel encoders, no visual odometry
- `/mavros/local_position/pose` publishes at ~30-50Hz (depends on EKF2 output rate)

## Deliverable

1. Sensor fusion architecture: what runs on PX4 vs Jetson
2. Accuracy estimate for each sensor source
3. Latency estimate end-to-end
4. Whether additional sensors are needed for ±2cm target
5. Decision: PX4 EKF2 only, or add robot_localization on Jetson?