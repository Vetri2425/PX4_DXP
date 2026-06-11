# PX4 Rover Square Tracking Analysis  
**Bag:** `arc_fix_04_20260609_174210_0.db3`  
**Controller reviewed:** `rpp_controller_node(1).py`  
**Generated:** 2026-06-10  
**Topic:** Why current parameters work better for arc/circle but fail on square, and how to fix it.

---

## 1. Executive Summary

The rover did not fail because of a single bad PX4 parameter. The main issue is that the current RPP controller and parameter set are tuned for **smooth continuous-curvature paths** such as arcs and circles, but the test path was a **2 m × 2 m square**, which has sharp 90° corners.

From the bag:

- Desired path: **2 m × 2 m closed square**
- Desired path length: **8.000 m**
- Actual traveled distance after final path publish: **~8.820 m**
- Mean cross-track error after final path publish: **~0.112 m**
- Max cross-track error: **~0.450 m**
- Final distance from nearest path point: **~0.130 m**

From the controller:

- `corner_smooth_radius_m = 0.5`
- `min_lookahead_dist = 0.52`
- `lookahead_time = 1.6`
- `mission_speed = 0.35`
- `max_yaw_rate_body = 0.45`
- `path_resample_spacing_m = 0.08`
- `use_feedforward_yaw_rate = True`

The most important finding is:

> The controller is not tracking the raw square exactly. With `corner_smooth_radius_m = 0.5`, the square is internally converted into a rounded square. Therefore, the rover cutting corners is expected behavior.

The observed peak error of about **0.45 m** is very close to the configured **0.5 m corner smoothing radius**, so a large part of the measured square error is explained by the controller deliberately smoothing the corner.

---

## 2. Bag Evidence

### 2.1 Important Topics Found

| Topic | Type | Count | Use |
|---|---:|---:|---|
| `/path` | `nav_msgs/msg/Path` | 2 | Desired path |
| `/rpp/debug` | `std_msgs/msg/Float32MultiArray` | 1951 | RPP internal state and parameter snapshot |
| `/rpp/velocity_ned` | `geometry_msgs/msg/Vector3Stamped` | 1951 | RPP velocity command |
| `/rpp/yaw_rate_body` | `std_msgs/msg/Float32` | 1951 | RPP yaw-rate command |
| `/mavros/setpoint_raw/local` | `mavros_msgs/msg/PositionTarget` | 1951 | PX4 offboard setpoint |
| `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | 1142 | Actual rover pose |
| `/mavros/state` | `mavros_msgs/msg/State` | 48 | Arm/mode state |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 76 | GPS/MAVROS health |

---

## 3. Frame Convention Finding

The desired path and actual pose are not in the same frame by default.

### Desired path

`/path` is in:

```text
frame_id = local_ned
x = North
y = East
z = Down
```

### Actual pose

`/mavros/local_position/pose` is MAVROS local pose in ENU:

```text
x = East
y = North
z = Up
```

So actual pose must be converted before comparison:

```text
actual_ned.North = pose_enu.y
actual_ned.East  = pose_enu.x
actual_ned.Down  = -pose_enu.z
```

This conversion was applied for the analysis.

---

## 4. Desired Path Found From Bag

The final `/path` message contained:

```text
frame: local_ned
points: 53
path length: 8.000 m
shape: 2 m × 2 m closed square
```

### Square corners in NED

| Corner | North m | East m |
|---:|---:|---:|
| Start / close point | 4.4505 | -3.2684 |
| Corner 1 | 6.4505 | -3.2684 |
| Corner 2 | 6.4505 | -1.2684 |
| Corner 3 | 4.4505 | -1.2684 |
| End / close point | 4.4505 | -3.2684 |

### Desired movement

```text
Start
→ North 2 m
→ East 2 m
→ South 2 m
→ West 2 m
→ close
```

There was also an earlier single-point `/path`:

```text
N = 4.7427
E = -2.9821
```

That appears to be a temporary or initial path publish, not the actual mission path.

---

## 5. Actual Traveled Path Found From Bag

Raw `/mavros/local_position/pose`:

```text
samples: 1142
duration: 38.066 s
raw ENU traveled distance: 8.860 m
```

Converted to NED:

```text
start NED:
N = 4.4109
E = -3.2727
D = 7.0582

after final path publish:
N = 4.4507
E = -3.2684
D = 7.0760

end NED:
N = 4.5745
E = -2.9999
D = 7.2104
```

Actual distance traveled after final path publish:

```text
8.820 m
```

Desired path length:

```text
8.000 m
```

Extra travel:

```text
~0.820 m
```

This extra distance is consistent with cornering, smoothing, and final settling.

---

## 6. Cross-Track Error Result

Using actual NED path against the raw desired square polyline:

| Metric | Value |
|---|---:|
| X-track min | ~0.000 m |
| X-track mean, full log | ~0.103 m |
| X-track max, full log | ~0.450 m |
| X-track mean, after final path publish | ~0.112 m |
| X-track max, after final path publish | ~0.450 m |
| Final distance from nearest path point | ~0.130 m |

### Interpretation

This is not yet a 1–2 cm tracking system for square corners.

However, the **0.45 m peak error is not surprising** because the controller was configured with:

```text
corner_smooth_radius_m = 0.5 m
```

So the controller likely tried to follow a rounded internal path, while the evaluation compared actual motion against the raw square.

---

## 7. Vehicle State Summary

From `/mavros/state`:

```text
MANUAL → OFFBOARD → OFFBOARD/disarmed
```

Transitions observed:

```text
armed=True,  mode=MANUAL
armed=True,  mode=OFFBOARD
armed=False, mode=OFFBOARD
```

Diagnostics looked healthy:

```text
GPS fix type: 6
satellites: 36–37
EPH: 0.40 m
EPV: 0.80 m
MAVROS connected: yes
vehicle type: Ground rover
PX4 mode: MANUAL then OFFBOARD
```

So this does not look like a GPS or MAVROS connectivity failure.

---

## 8. RPP Debug Parameter Snapshot

From `/rpp/debug`, the active parameter values were:

```text
debug[11] max_linear_vel                  = 0.8000
debug[12] min_linear_vel                  = 0.1500
debug[13] min_lookahead_dist              = 0.5200
debug[14] max_lookahead_dist              = 1.0000
debug[15] lookahead_time                  = 1.6000
debug[16] a_lat_max                       = 0.3000
debug[17] regulated_min_speed             = 0.3000
debug[18] xy_goal_tolerance               = 0.0200
debug[19] min_goal_travel_m               = 0.5000
debug[20] approach_velocity_scaling_dist  = 0.6000
debug[21] min_approach_linear_velocity    = 0.1000
debug[22] p4_zero_vel_threshold           = 0.0200
debug[23] pose_max_age_s                  = 0.5000
debug[24] ekf_jump_threshold_m            = 0.0500
debug[25] require_rtk_fix                 = 1.0000
debug[26] preview_curvature_n             = 4.0000
debug[27] xtrack_lookahead_gain           = 0.0500
debug[28] path_resample_spacing_m         = 0.0800
debug[29] corner_smooth_radius_m          = 0.5000
debug[30] corner_smooth_arc_pts           = 6.0000
debug[31] use_imu_extrapolation           = 0.0000
debug[32] imu_max_extrap_age_s            = 0.1000
debug[33] use_feedforward_yaw_rate        = 1.0000
debug[34] yaw_rate_feedback_gain          = 0.0000
debug[35] max_yaw_rate_body               = 0.4500
debug[36] max_linear_accel                = 0.3500
debug[37] max_linear_decel                = 0.5000
debug[38] mission_speed                   = 0.3500
```

Useful command observations:

```text
target / max linear speed observed: ~0.350 m/s
mean RPP speed command: ~0.289 m/s
yaw-rate command clamp: ±0.450 rad/s
lookahead-like value range: about 0.336 to 0.583 m
```

---

## 9. RPP Controller Code Findings

The uploaded controller node confirms the same parameter design.

### 9.1 RPP subscribes to path in LOCAL_NED

The controller comments state:

```text
Path poses are in LOCAL_NED.
MAVROS pose is ENU and converted to NED on read.
All math after pose input is NED.
```

This part is correct.

### 9.2 Controller outputs NED velocity vector

The controller publishes:

```text
/rpp/velocity_ned
```

with:

```text
vector.x = v_north
vector.y = v_east
vector.z = 0
```

This is expected.

### 9.3 Controller publishes feedforward yaw rate

The controller publishes:

```text
/rpp/yaw_rate_body
```

using:

```python
yaw_rate_ff = kappa * speed
yaw_rate_fb = yaw_rate_feedback_gain * theta_e
yaw_rate_body = yaw_rate_ff + yaw_rate_fb
yaw_rate_body = clamp(yaw_rate_body, -max_yaw_rate_body, +max_yaw_rate_body)
```

Since:

```text
yaw_rate_feedback_gain = 0.0
```

the yaw-rate command is pure feedforward:

```text
yaw_rate_body = kappa × speed
```

This is good for arcs and circles, but not enough for sharp 90° square corners unless the path is handled as a special sharp-corner mode.

### 9.4 Biggest code-level finding: corner smoothing is active

The controller declares:

```python
self.declare_parameter("corner_smooth_radius_m", 0.5)
```

And in `_path_cb()`:

```python
if corner_r > 0.0 and len(cond_pts) >= 3:
    cond_pts, cond_flags = self._smooth_corners(...)
```

Therefore:

```text
Raw square path → internally smoothed rounded-square path
```

This is the primary reason square tracking visually cuts the corners.

---

## 10. Why Current Parameters Work for Arc and Circle

Arc and circle paths have smooth curvature.

For a circle:

```text
curvature κ = 1 / radius
```

For a normal arc:

```text
curvature changes slowly or remains constant
```

This is exactly what RPP likes.

The controller can:

1. Find a stable lookahead point.
2. Compute curvature to that lookahead.
3. Apply lateral acceleration speed limiting.
4. Command `yaw_rate = κ × v`.
5. Keep moving smoothly without discontinuities.

For arc/circle:

```text
lookahead target moves smoothly
velocity vector changes smoothly
yaw-rate feedforward is meaningful
PX4 receives continuous commands
rover motion is stable
```

That is why your current parameter set works better for arcs/circles.

---

## 11. Why Square Fails With Same Parameters

A square has discontinuous curvature.

At each 90° corner:

```text
straight segment curvature = 0
corner curvature = very high / discontinuous
next straight curvature = 0
```

Pure pursuit does not naturally stop at a corner and rotate. It looks ahead along the path. Near the corner, the lookahead point moves onto the next side, so the vector from rover to lookahead becomes diagonal.

The current control law effectively says:

```python
v_n = speed * unit_to_lookahead_n
v_e = speed * unit_to_lookahead_e
```

So at a square corner, the rover is commanded diagonally across the corner.

That causes:

```text
corner cutting
large cross-track spike
final end offset
extra travel distance
less square-like shape
```

---

## 12. Key Root Causes

### Root Cause 1 — The square is being internally rounded

```text
corner_smooth_radius_m = 0.5 m
```

For a 2 m side, this is large. It consumes 25% of each side length.

This alone explains a large part of the observed corner cutting.

### Root Cause 2 — Lookahead is too large for 2 m square sides

```text
min_lookahead_dist = 0.52 m
lookahead_time = 1.6
```

On a 2 m side, 0.52 m lookahead is large. Near corners, the lookahead target enters the next segment early.

### Root Cause 3 — RPP is continuous, but square requires discrete corner behavior

For sharp square tracking, the rover should not always continuously chase a point. It should:

```text
track straight segment
slow before corner
reach corner tolerance
align to next segment
then continue
```

This is not the same behavior as arc/circle tracking.

### Root Cause 4 — Yaw-rate cap may limit turn authority

```text
max_yaw_rate_body = 0.45 rad/s
```

At 0.35 m/s, a 0.45 rad/s yaw rate corresponds to an equivalent turn radius:

```text
R = v / ω = 0.35 / 0.45 = 0.78 m
```

That is okay for smooth arcs, but for tighter square corner transitions it can still be slow, especially if the rover must rotate quickly into the next side.

### Root Cause 5 — Evaluation compared actual path to raw square

The controller likely tracked a rounded internal square, but the analysis compared actual path against the original raw square. So the reported error is partly real tracking error and partly expected geometric difference.

---

## 13. Immediate Fix Strategy

Use separate parameter profiles for:

1. Arc / circle / smooth paths
2. Square / rectangle / sharp polyline paths

Do not use the exact same parameter set for both.

---

## 14. Immediate Square Parameter Set

For the next square test, use:

```bash
ros2 param set /rpp_controller corner_smooth_radius_m 0.15
ros2 param set /rpp_controller min_lookahead_dist 0.25
ros2 param set /rpp_controller max_lookahead_dist 0.60
ros2 param set /rpp_controller lookahead_time 1.0
ros2 param set /rpp_controller mission_speed 0.25
ros2 param set /rpp_controller max_yaw_rate_body 0.65
ros2 param set /rpp_controller a_lat_max 0.25
ros2 param set /rpp_controller preview_curvature_n 3
ros2 param set /rpp_controller xtrack_lookahead_gain 0.0
```

Expected result:

```text
less corner cutting
lower peak xtrack error
slower but tighter path
more square-like actual path
less diagonal shortcut behavior
```

---

## 15. Aggressive Square Test Profile

For debugging only, test with zero smoothing:

```bash
ros2 param set /rpp_controller corner_smooth_radius_m 0.0
ros2 param set /rpp_controller min_lookahead_dist 0.20
ros2 param set /rpp_controller max_lookahead_dist 0.50
ros2 param set /rpp_controller lookahead_time 0.8
ros2 param set /rpp_controller mission_speed 0.20
ros2 param set /rpp_controller max_yaw_rate_body 0.75
ros2 param set /rpp_controller a_lat_max 0.20
ros2 param set /rpp_controller xtrack_lookahead_gain 0.0
```

This should show whether the main problem is corner smoothing or downstream PX4 yaw/turn response.

Important:

> A real rover cannot physically follow a perfect mathematical 90° corner at speed. With zero smoothing, the system may need a stop-turn-go behavior to be truly accurate.

---

## 16. Keep Current Profile for Arc/Circle

For arc and circle, your current profile is reasonable:

```bash
ros2 param set /rpp_controller corner_smooth_radius_m 0.5
ros2 param set /rpp_controller min_lookahead_dist 0.52
ros2 param set /rpp_controller max_lookahead_dist 1.0
ros2 param set /rpp_controller lookahead_time 1.6
ros2 param set /rpp_controller mission_speed 0.35
ros2 param set /rpp_controller max_yaw_rate_body 0.45
ros2 param set /rpp_controller a_lat_max 0.30
ros2 param set /rpp_controller preview_curvature_n 4
```

This is smoother and safer for continuous curvature paths.

---

## 17. Production-Grade Fix Strategy

The correct production fix is not just parameter tuning.

The controller should support two modes.

---

## 18. Mode A — Smooth RPP Mode

Use this for:

```text
arc
circle
smooth spline
large-radius DXF curve
rounded polyline
```

Characteristics:

```text
continuous velocity
continuous yaw-rate
curvature speed regulation
corner smoothing allowed
feedforward yaw rate useful
```

Recommended behavior:

```text
Use current RPP design.
Keep corner smoothing enabled.
Use larger lookahead.
Use normal mission speed.
```

---

## 19. Mode B — Sharp-Corner Segment Mode

Use this for:

```text
square
rectangle
field boundary
polyline with hard corners
marking paths requiring sharp geometry
```

The controller should switch to a state machine:

```text
TRACK_SEGMENT
→ PRE_CORNER_SLOWDOWN
→ CORNER_ALIGN
→ NEXT_SEGMENT
→ DONE
```

### State 1 — TRACK_SEGMENT

Follow only the current line segment.

Do not let the lookahead point jump deeply into the next segment.

Use line projection and signed cross-track control.

### State 2 — PRE_CORNER_SLOWDOWN

When distance to corner is below threshold:

```text
corner_slowdown_dist = 0.4 to 0.7 m
```

reduce speed:

```text
speed = 0.10 to 0.20 m/s
```

### State 3 — CORNER_ALIGN

At the corner:

```text
if distance_to_corner < corner_acceptance_radius:
    stop or near-stop
    align heading to next segment
```

Acceptance radius:

```text
0.03 to 0.08 m
```

Heading threshold:

```text
5° to 10°
```

### State 4 — NEXT_SEGMENT

After heading aligns:

```text
advance segment index
resume forward tracking
```

---

## 20. Recommended Controller Design Change

Add path mode detection or explicit path mode metadata.

Example:

```python
if path_type in ["ARC", "CIRCLE", "CURVE", "ROUNDED_POLYLINE"]:
    controller_mode = "SMOOTH_RPP"
elif path_type in ["SQUARE", "RECTANGLE", "POLYLINE_SHARP"]:
    controller_mode = "SHARP_CORNER_SEGMENT"
```

If explicit metadata is not available, infer from path geometry:

```python
for each interior vertex:
    angle = angle_between(prev_segment, next_segment)

if angle_change > sharp_corner_threshold_deg:
    sharp_corner_count += 1

if sharp_corner_count > 0:
    use sharp-corner mode
```

Suggested threshold:

```text
sharp_corner_threshold_deg = 45°
```

For square, each corner is 90°, so it should use sharp-corner mode.

---

## 21. Important Implementation Rule

Do not blindly smooth every path.

Current behavior:

```python
if corner_smooth_radius_m > 0:
    smooth all corners
```

Better behavior:

```python
if mode == "SMOOTH_RPP":
    apply corner smoothing
else:
    preserve raw sharp corners
```

This prevents a square from being unintentionally converted into a rounded square.

---

## 22. RPP-Level Code Fixes

### Fix 1 — Add controller profile parameter

Add:

```python
self.declare_parameter("tracking_profile", "smooth")
```

Allowed values:

```text
smooth
sharp
auto
```

Behavior:

```python
if tracking_profile == "smooth":
    use current RPP behavior

if tracking_profile == "sharp":
    disable corner smoothing and use segment/corner state machine

if tracking_profile == "auto":
    detect path geometry and choose mode
```

---

### Fix 2 — Publish internal conditioned path for debugging

Currently, analysis used raw `/path`, but RPP internally conditions it.

Add a debug topic:

```text
/rpp/conditioned_path
```

Publish the actual path after:

```text
resampling
corner smoothing
mark/transit conditioning
```

Then future analysis can compare:

```text
actual path vs raw desired path
actual path vs RPP conditioned target path
```

This will clearly separate:

```text
planner geometry error
controller tracking error
vehicle/PX4 response error
```

---

### Fix 3 — Add corner debug states

Extend `/rpp/debug` or add `/rpp/corner_debug`:

```text
current_segment_index
distance_to_segment_end
distance_to_corner
corner_angle_deg
corner_mode_active
target_heading_to_next_segment
heading_error_to_next_segment
corner_state_code
```

This will make square debugging much easier.

---

### Fix 4 — Do not allow lookahead to shortcut corners in sharp mode

For sharp mode:

```text
lookahead should remain on current segment
until the rover reaches corner acceptance radius
```

Only after corner acceptance should the target move to the next segment.

This prevents diagonal corner cutting.

---

### Fix 5 — Add corner velocity law

Use:

```text
v_corner = clamp(k * distance_to_corner, min_speed, mission_speed)
```

Example:

```python
if distance_to_corner < corner_slowdown_dist:
    speed = max(min_corner_speed, mission_speed * distance_to_corner / corner_slowdown_dist)
```

Suggested values:

```text
corner_slowdown_dist = 0.50 m
min_corner_speed = 0.08 m/s
corner_acceptance_radius = 0.05 m
```

---

## 23. PX4 / Downstream Checks Needed

To fully verify the downstream side, review these files next:

```text
1. twist_to_setpoint_node.py
2. PX4 rover parameter dump
3. mission/path generation code
4. RPP launch YAML
5. PX4 rover attitude/rate/differential controller files if using a fork
```

Most important is:

```text
twist_to_setpoint_node.py
```

because it determines whether:

```text
/rpp/velocity_ned
/rpp/yaw_rate_body
```

are correctly forwarded into:

```text
/mavros/setpoint_raw/local
```

Also verify PX4 parameters:

```text
RO_YAW_P
RO_YAW_RATE_LIM
RD_TRANS_DRV_TRN
RD_TRANS_TRN_DRV
RD_MAX_THR_YAW_R
RD_YAW_RATE_P
RD_WHEEL_TRACK
RD_SPEED_P
RD_SPEED_I
```

---

## 24. Test Plan

### Test 1 — Repeat square with current params

Purpose:

```text
baseline confirmation
```

Expected:

```text
corner cutting similar to current result
peak error around smoothing radius scale
```

---

### Test 2 — Square with reduced smoothing

Use:

```bash
ros2 param set /rpp_controller corner_smooth_radius_m 0.15
ros2 param set /rpp_controller min_lookahead_dist 0.25
ros2 param set /rpp_controller mission_speed 0.25
```

Expected:

```text
actual path should become more square-like
peak error should drop
```

---

### Test 3 — Square with zero smoothing

Use:

```bash
ros2 param set /rpp_controller corner_smooth_radius_m 0.0
ros2 param set /rpp_controller min_lookahead_dist 0.20
ros2 param set /rpp_controller mission_speed 0.20
```

Expected:

```text
more accurate straight sides
possible jerk or hesitation at corners
if PX4 cannot rotate quickly, corner overshoot may remain
```

---

### Test 4 — Arc/circle with current params

Purpose:

```text
confirm smooth profile remains good
```

Expected:

```text
arc/circle should remain stable
low oscillation
smooth yaw-rate
```

---

### Test 5 — Compare against RPP conditioned path

After adding `/rpp/conditioned_path`, evaluate:

```text
actual vs raw planner path
actual vs RPP conditioned path
```

This will tell whether the error comes from:

```text
intentional smoothing
controller tracking
PX4/vehicle dynamics
```

---

## 25. Expected Results After Fix

With only parameter tuning:

```text
Mean xtrack may improve from ~0.11 m to ~0.04–0.08 m
Peak xtrack may improve from ~0.45 m to ~0.10–0.20 m
```

With proper sharp-corner segment mode:

```text
Mean xtrack can approach 1–3 cm on straight sides
Corner error depends on stop/turn accuracy and RTK quality
Peak corner error can be kept below ~5–10 cm with good tuning
```

For true 1–2 cm square marking, the rover likely needs:

```text
RTK fixed
low speed
good wheel calibration
no corner smoothing for square
segment-level tracking
corner stop/align behavior
careful PX4 yaw/turn tuning
```

---

## 26. Final Diagnosis

Current result:

```text
Arc/circle: good
Square: poor corner fidelity
```

Reason:

```text
The current RPP profile is optimized for smooth paths.
Square requires sharp-corner logic.
The controller also smooths the square with 0.5 m radius, so corner cutting is expected.
```

Failure split estimate:

```text
70% path/controller geometry behavior
20% yaw-rate / PX4 heading response limit
10% tuning
```

Fastest near-term fix:

```text
Use a separate square parameter profile:
smaller smoothing
smaller lookahead
slower mission speed
higher yaw-rate allowance
```

Correct production fix:

```text
Add dual-mode controller:
1. Smooth RPP mode for arcs/circles
2. Sharp-corner segment mode for squares/rectangles/polylines
```

---

## 27. Recommended Next Engineering Action

### Immediate

Run square again with:

```bash
ros2 param set /rpp_controller corner_smooth_radius_m 0.15
ros2 param set /rpp_controller min_lookahead_dist 0.25
ros2 param set /rpp_controller max_lookahead_dist 0.60
ros2 param set /rpp_controller lookahead_time 1.0
ros2 param set /rpp_controller mission_speed 0.25
ros2 param set /rpp_controller max_yaw_rate_body 0.65
ros2 param set /rpp_controller a_lat_max 0.25
ros2 param set /rpp_controller preview_curvature_n 3
ros2 param set /rpp_controller xtrack_lookahead_gain 0.0
```

### Then

Upload:

```text
twist_to_setpoint_node.py
PX4 parameter dump
launch YAML for RPP
path generation / mission load code
```

### Production

Implement:

```text
tracking_profile = smooth | sharp | auto
/rpp/conditioned_path publisher
sharp-corner segment state machine
corner slowdown + corner align behavior
```

---

## 28. Related Generated Artifacts

- `actual_vs_desired_path_clean.png`
- `path_points.csv`
- `traveled_path.csv`
- `traveled_path_ned.csv`

Use the clean plot to visually confirm:

```text
desired square path
actual rover path
corner cutting
start/end offset
xtrack summary
```
