# T5: RPP (Rover Pure Pursuit) & Arc Following Controller

## What to Research

What is the RPP commander? How does pure pursuit work for differential drive? How do we follow arcs and complex shapes with ±2cm accuracy?

## Specific Questions

1. **What is RPP (Rover Pure Pursuit)?**
   - Is RPP a PX4 internal module, a ROS2 package, or a custom controller?
   - PX4 has `RoverPositionControl` — is this the same as RPP?
   - Does PX4's internal pure pursuit work in OFFBOARD mode?
   - Or do we need to implement pure pursuit as a ROS2 node on Jetson?

2. **Pure Pursuit algorithm for differential drive:**
   - Standard PP computes steering angle for Ackermann steering
   - For differential drive: no steering angle. Instead compute left/right wheel speeds
   - How to convert PP output (curvature) to differential wheel speeds?
   - Formula: `ω = v / R` where R = turn radius from PP, ω = yaw rate, v = forward speed
   - Then: `left_speed = v - ω * L/2`, `right_speed = v + ω * L/2` where L = wheel track
   - Is this correct? What about the minimum turning radius?

3. **Lookahead distance tuning:**
   - Standard: `L_d = K * v` (proportional to speed)
   - For 0.3 m/s: what K value? (typical: 0.5-2.0)
   - Too small: oscillates, too large: cuts corners
   - For ±2cm on straight lines: minimum lookahead?
   - For ±2cm on arcs (R=1.5m): minimum lookahead?
   - Variable lookahead: decrease near corners, increase on straights?

4. **Arc following specifically:**
   - Pure pursuit follows a lookahead point on the path
   - For a circular arc: the lookahead point is always on the circle
   - Does PP naturally follow arcs, or does it cut inside/outside?
   - What is the cross-track error for a circle of radius R=1.5m at v=0.3 m/s?
   - Is there a better controller than PP for arcs? (MPC, LQR, Stanley)

5. **Corner handling:**
   - What happens at a sharp corner (e.g., 90° turn in a rectangle)?
   - PP tends to cut corners — how to handle?
   - Slow down before corners? Switch to position mode at corners?
   - "Slow-in, fast-out" speed profile at corners?

6. **Segment transitions:**
   - Line → arc: how to transition smoothly?
   - Arc → line: how to transition smoothly?
   - Arc → arc (different radii): how to transition?
   - Should the controller know about upcoming segments (preview)?

7. **Comparison with alternatives:**
   - Pure Pursuit vs Stanley controller for differential drive
   - Pure Pursuit vs MPC (Model Predictive Control)
   - Pure Pursuit vs LQR (Linear Quadratic Regulator)
   - Which is used by commercial marking robots?
   - What does the GNSS/INS paper recommend (MPC with 46% lateral improvement)?

## Our Context

- Differential drive: wheelbase 470mm, min turn radius ~235mm
- Speed: 0.3-0.4 m/s for marking
- Accuracy target: ±2cm lateral
- OFFBOARD mode: Jetson sends 50Hz setpoints
- Must follow: straight lines, arcs (R≥0.5m), circles, connected shapes
- PX4's internal position controller exists but we don't know if it uses PP

## Deliverable

1. RPP definition and how it fits our architecture
2. Pure Pursuit implementation plan for differential drive
3. Lookahead tuning guidelines for our speed/accuracy targets
4. Arc following accuracy estimate
5. Corner handling strategy
6. Comparison: PP vs Stanley vs MPC — recommendation for our rover