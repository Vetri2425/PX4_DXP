# Pure Pursuit Algorithm in PX4 — Detailed Explanation

## Overview

**Pure Pursuit** is a path-following algorithm used in PX4 rovers to guide the vehicle along a path defined by waypoints. It's a **geometric steering control** method that calculates the desired heading (yaw) the rover should follow to track the path.

The algorithm was originally developed by **R. C. Coulter (1992)** at Carnegie Mellon University and is widely used in autonomous vehicles because it's:
- **Simple** to implement
- **Robust** to disturbances
- **Computationally efficient**
- **Naturally stable** for most vehicle types

---

## The Core Concept: Lookahead Circle

Pure Pursuit works by:

1. **Drawing a circle** around the vehicle with radius = **lookahead distance**
2. **Finding where this circle intersects** the path (line segment between previous and current waypoint)
3. **Steering toward the intersection point** closer to the current waypoint

```
                    C (Current Waypoint)
                   /
                __/__
              /  /    \
             /  /      \
            |  /  V     |  ← Lookahead circle (radius = lookahead_distance)
             \/        /
             /\ _____ /
            /
           P (Previous Waypoint)

    N (0 rad)
        ^
        |
        | D
(-1.5708 rad) <----- ⨂ -----> E (1.5708 rad)
        |
        |
        ⌄
    (+- 3.14159 rad)
```

**Key insight:** The lookahead distance is **proportional to vehicle speed**:
```
lookahead_distance = speed * lookahead_gain
```

This means:
- **Fast vehicles** look further ahead (smoother turns)
- **Slow vehicles** look closer (tighter tracking)
- **Stationary vehicles** have minimal lookahead (can't steer)

---

## PX4 Implementation Details

### Function Signature
```cpp
float calcTargetBearing(
    pure_pursuit_status_s &pure_pursuit_status,
    float lookahead_gain,           // Tuning parameter (typically 0.5–2.0)
    float lookahead_max,            // Maximum lookahead distance [m]
    float lookahead_min,            // Minimum lookahead distance [m]
    const Vector2f &curr_wp_ned,    // Current waypoint (North/East) [m]
    const Vector2f &prev_wp_ned,    // Previous waypoint (North/East) [m]
    const Vector2f &curr_pos_ned,   // Vehicle position (North/East) [m]
    float vehicle_speed             // Vehicle ground speed [m/s]
);
```

### Return Value
- **Target bearing** (yaw angle in radians) the vehicle should steer toward
- **NAN** if inputs are invalid

### Algorithm Steps (from PurePursuit.cpp)

#### Step 1: Calculate Lookahead Distance
```cpp
const float lookahead_distance = math::constrain(
    lookahead_gain * fabsf(vehicle_speed),  // Scale by speed
    lookahead_min,                          // Clamp to minimum
    lookahead_max                           // Clamp to maximum
);
```

#### Step 2: Project Vehicle Position onto Path
```cpp
// Vector from previous waypoint to current waypoint
const Vector2f prev_wp_to_curr_wp = curr_wp_ned - prev_wp_ned;

// Vector from previous waypoint to vehicle
const Vector2f prev_wp_to_curr_pos = curr_pos_ned - prev_wp_ned;

// Project vehicle position onto the path line
const Vector2f position_along_path = 
    (prev_wp_to_curr_pos * prev_wp_to_curr_wp_u) * prev_wp_to_curr_wp_u;

// Shortest vector from vehicle to path (perpendicular)
const Vector2f curr_pos_to_path = position_along_path - prev_wp_to_curr_pos;

// Signed crosstrack error (positive = left of path, negative = right)
const float crosstrack_error = sign(...) * curr_pos_to_path.norm();
```

#### Step 3: Handle Special Cases

**Case 1: Vehicle is closer to waypoint than lookahead distance**
```cpp
if (curr_pos_to_curr_wp.norm() < lookahead_distance) {
    target_bearing = bearing_to_curr_waypoint;  // Just steer to waypoint
}
```

**Case 2: Path is outside lookahead circle (no intersection)**
```cpp
else if (fabsf(crosstrack_error) > lookahead_distance) {
    // Steer to closest point on path (fallback)
    target_bearing = bearing_to_closest_point_on_path;
}
```

**Case 3: Normal Pure Pursuit (intersection exists)**
```cpp
else {
    // Calculate distance along path to intersection point
    const float line_extension = sqrt(
        lookahead_distance² - crosstrack_error²
    );
    
    // Find intersection point
    const Vector2f intersection_point = 
        position_along_path + line_extension * prev_wp_to_curr_wp_u;
    
    // Steer toward intersection
    target_bearing = atan2(intersection_point.y, intersection_point.x);
}
```

---

## How Your Rover Uses Pure Pursuit

### In DifferentialPosControl.cpp

Your rover's position controller calls Pure Pursuit like this:

```cpp
// Line 99-102 in DifferentialPosControl.cpp
const float yaw_setpoint = PurePursuit::calcTargetBearing(
    pure_pursuit_status,
    _param_pp_lookahd_gain.get(),      // PP_LOOKAHD_GAIN parameter
    _param_pp_lookahd_max.get(),       // PP_LOOKAHD_MAX parameter
    _param_pp_lookahd_min.get(),       // PP_LOOKAHD_MIN parameter
    _target_waypoint_ned,              // Current waypoint from OFFBOARD
    _start_ned,                        // Previous waypoint
    _curr_pos_ned,                     // GPS position (NED frame)
    fabsf(speed_setpoint)              // Desired speed
);
```

### Control Flow

```
OFFBOARD Setpoint (from Jetson)
    ↓
DifferentialPosControl::updatePosControl()
    ↓
Pure Pursuit calcTargetBearing()
    ↓
yaw_setpoint (desired heading)
    ↓
Heading Error = yaw_setpoint - vehicle_yaw
    ↓
DifferentialRateControl (yaw rate controller)
    ↓
Inverse Kinematics (IK)
    ↓
Motor PWM outputs (left & right)
```

### Key Parameters

| Parameter | Default | Range | Purpose |
|-----------|---------|-------|---------|
| `PP_LOOKAHD_GAIN` | 1.0 | 0.1–5.0 | Scales lookahead with speed |
| `PP_LOOKAHD_MAX` | 10.0 m | 1–50 m | Maximum lookahead distance |
| `PP_LOOKAHD_MIN` | 1.0 m | 0.1–5 m | Minimum lookahead distance |
| `NAV_ACC_RAD` | 0.5 m | 0.1–5 m | Waypoint acceptance radius |

---

## Why You're Developing RPP (Reactive Pure Pursuit) on Jetson

### Limitations of Stock Pure Pursuit in PX4

1. **No obstacle avoidance** — Pure Pursuit blindly follows waypoints; it can't detect or avoid obstacles
2. **No dynamic replanning** — If the path becomes blocked, the rover gets stuck
3. **No sensor fusion** — Only uses GPS; doesn't incorporate camera, LiDAR, or other sensors
4. **Fixed lookahead** — Can't adapt to terrain or environmental changes
5. **No real-time adaptation** — Parameters are static; can't adjust on-the-fly

### What RPP Adds

**Reactive Pure Pursuit** enhances the algorithm by:

1. **Real-time obstacle detection** (camera/LiDAR on Jetson)
2. **Dynamic path replanning** (if obstacle detected, generate new waypoints)
3. **Adaptive lookahead** (increase lookahead on open terrain, decrease near obstacles)
4. **Sensor fusion** (combine GPS, vision, IMU for better localization)
5. **Reactive steering** (adjust heading based on local environment, not just global path)

### Architecture

```
Jetson Orin (Companion Computer)
    ├── Camera/LiDAR input
    ├── RPP Algorithm
    │   ├── Obstacle detection
    │   ├── Path replanning
    │   └── Adaptive lookahead
    └── MAVROS2 Bridge
        ↓
        OFFBOARD Setpoint (waypoint + speed)
        ↓
PX4 on CubeOrangePlus
    ├── Pure Pursuit (calcTargetBearing)
    ├── Rate Control
    └── Motor Output
```

### Why Jetson, Not PX4?

- **PX4 is real-time** — Can't afford expensive vision processing
- **Jetson is powerful** — Can run OpenCV, neural networks, SLAM
- **Separation of concerns** — PX4 handles low-level control; Jetson handles high-level planning
- **Modularity** — Can swap algorithms without reflashing firmware

---

## Pure Pursuit Behavior Examples

### Example 1: Straight Line
```
P -------- C
    V

Vehicle is on path, heading error = 0
→ Pure Pursuit returns bearing toward C
→ Rover drives straight
```

### Example 2: Left Turn
```
    C
   /
  /
 /
P
    V

Vehicle is right of path, heading error > 0
→ Pure Pursuit returns bearing toward intersection point (left)
→ Rover steers left
```

### Example 3: Tight Turn (Low Speed)
```
    C
   /
  /
 /
P
V (close to path)

Low speed → small lookahead_distance
→ Intersection point is close to vehicle
→ Tight steering angle
→ Sharp turn
```

### Example 4: Gentle Turn (High Speed)
```
        C
       /
      /
     /
    /
   /
  /
 P
V (far from path)

High speed → large lookahead_distance
→ Intersection point is far ahead
→ Shallow steering angle
→ Gentle turn
```

---

## Mathematical Foundation

### Lookahead Circle Intersection

Given:
- Vehicle at position **V**
- Lookahead circle radius **L**
- Path line from **P** to **C**

Find: Intersection point **I**

**Solution:**
1. Project V onto line PC → point **Q**
2. Distance from V to Q = **d** (crosstrack error)
3. If **d > L**: No intersection (fallback to closest point)
4. If **d ≤ L**: Distance along path from Q to I = √(L² - d²)
5. Intersection point: **I = Q + √(L² - d²) × (unit vector along PC)**

### Bearing Calculation
```cpp
target_bearing = atan2(I.y - V.y, I.x - V.x)
```

---

## Tuning Guidelines for Your Rover

### Lookahead Gain (`PP_LOOKAHD_GAIN`)
- **Too low** (< 0.5): Rover oscillates, overshoots turns
- **Too high** (> 2.0): Rover cuts corners, misses waypoints
- **Recommended**: 1.0–1.5 for differential drive rovers

### Lookahead Min/Max
- **Min** (1.0 m): Prevents steering deadlock at low speeds
- **Max** (10.0 m): Prevents excessive lookahead at high speeds
- **Ratio**: Max/Min should be 5–10×

### Acceptance Radius (`NAV_ACC_RAD`)
- **Too small** (< 0.3 m): Rover overshoots, oscillates around waypoint
- **Too large** (> 2.0 m): Rover accepts waypoint too early, cuts path
- **Recommended**: 0.5–1.0 m for marking operations

---

## Debugging Pure Pursuit Issues

### Check These Logs
```
pure_pursuit_status.csv:
  - lookahead_distance: Should scale with speed
  - crosstrack_error: Should decrease as rover approaches path
  - target_bearing: Should point toward waypoint
  - distance_to_waypoint: Should decrease monotonically

rover_attitude_setpoint.csv:
  - yaw_setpoint: Should match target_bearing from Pure Pursuit

vehicle_local_position.csv:
  - body_vx: Should be positive (forward) in AUTO mode
  - heading: Should track yaw_setpoint
```

### Common Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| Rover oscillates left/right | Lookahead gain too low | Increase `PP_LOOKAHD_GAIN` |
| Rover cuts corners | Lookahead gain too high | Decrease `PP_LOOKAHD_GAIN` |
| Rover never reaches waypoint | Acceptance radius too large | Decrease `NAV_ACC_RAD` |
| Rover overshoots waypoint | Acceptance radius too small | Increase `NAV_ACC_RAD` |
| Rover stuck at low speed | Lookahead min too high | Decrease `PP_LOOKAHD_MIN` |

---

## Summary

**Pure Pursuit** is the **geometric steering algorithm** that:
1. Calculates a lookahead circle around your rover
2. Finds where the circle intersects the path
3. Steers toward that intersection point
4. Naturally adapts to speed (faster = gentler turns)

**Your RPP on Jetson** will enhance this by:
1. Adding obstacle detection
2. Dynamically replanning paths
3. Adapting lookahead based on environment
4. Fusing multiple sensors

This separation allows PX4 to focus on **low-level control** while Jetson handles **high-level planning**.

---

## References

- **Original Paper:** Coulter, R. C. (1992). Implementation of the Pure Pursuit Path Tracking Algorithm. CMU-RI-TR-92-01.
- **PX4 Source:** `src/lib/pure_pursuit/PurePursuit.cpp`
- **Your Rover:** `src/modules/rover_differential/DifferentialPosControl/DifferentialPosControl.cpp`
