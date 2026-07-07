# PX4 Rover Stop And Waypoint Transition Logic

Date: 2026-07-07

Reference firmware tree:

- `/Users/dyx_a1/Vetri/PX4-Autopilot`
- Observed commit: `4152220472`
- Note: that firmware tree has local rover differential edits, so this document separates the general PX4 logic from local custom behavior where relevant.

This document is only a reference for how PX4 rover mission logic works internally. It does not change RPP, MAVROS, server behavior, FCU parameters, or PX4 firmware.

## Short Answer

PX4 rover stopping is not based on one emergency-style stop command at the corner.

PX4 works by combining:

1. Mission triplet context: previous, current, next waypoint.
2. A per-waypoint `arrival_speed` decision.
3. Pure Pursuit line tracking from previous waypoint to current waypoint.
4. Distance-based deceleration so the rover can physically reach the requested arrival speed.
5. Acceptance-radius mission advance.
6. Zero-speed hold behavior after the stop condition is reached.

So the important PX4 idea is:

```text
track the line -> slow before the point -> arrive with target speed -> accept/hold/advance
```

Not:

```text
drive fast to the corner -> send one stop command at the corner
```

## Main PX4 Data Model

PX4 Navigator publishes a mission setpoint triplet:

- `previous`
- `current`
- `next`

This triplet gives the rover controller enough context to know:

- where the current segment starts
- where the current target point is
- whether there is a next segment after the point
- whether the rover should stop or transition through the point

The rover-specific setpoint is `RoverPositionSetpoint`:

Reference:

- `/Users/dyx_a1/Vetri/PX4-Autopilot/msg/RoverPositionSetpoint.msg`

Fields:

```text
position_ned   target position
start_ned      line start position
cruising_speed normal travel speed
arrival_speed  speed desired at target
yaw            mecanum-only travel yaw
```

The key fields for stopping are:

```text
position_ned
start_ned
cruising_speed
arrival_speed
```

If `arrival_speed = 0`, the rover should approach the target and stop.

If `arrival_speed > 0`, the rover may continue through the waypoint.

## Mission/Navigator Layer

Navigator owns mission progress.

It decides when a mission item is reached and when to advance to the next one.

Important reference files:

- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/navigator/mission.cpp`
- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/navigator/mission_base.cpp`
- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/navigator/mission_block.cpp`
- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/navigator/navigator_main.cpp`

For normal mission items, Navigator marks the waypoint reached when the vehicle is inside the acceptance radius.

The relevant behavior is in:

```text
MissionBlock::is_mission_item_reached_or_completed()
```

For normal waypoint position acceptance:

```text
dist_xy <= acceptance_radius
```

For rovers, yaw is accepted automatically after position is reached, because rover waypoint acceptance is mainly position-based.

Important point:

PX4 mission acceptance is not exact zero-distance logic. It is acceptance-radius logic.

The acceptance radius comes from:

- `NAV_ACC_RAD`
- sometimes controller-reported acceptance radius

In `Navigator::get_acceptance_radius()`, PX4 uses `NAV_ACC_RAD` and, for fixed-wing/rover, can take the max of that and controller acceptance.

## Auto Mode Layer

The rover auto mode converts Navigator's mission triplet into a rover position setpoint.

Reference:

- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/rover_differential/DifferentialDriveModes/DifferentialAutoMode/DifferentialAutoMode.cpp`

The main flow:

1. Read `position_setpoint_triplet`.
2. Convert global waypoint coordinates to local NED.
3. Compute:
   - current waypoint NED
   - previous waypoint NED
   - next waypoint NED
4. Calculate the waypoint transition angle.
5. Decide `arrival_speed`.
6. Publish `rover_position_setpoint`.

The published rover position setpoint contains:

```text
position_ned = current waypoint
start_ned = previous waypoint
cruising_speed = mission/default speed
arrival_speed = chosen target speed at current waypoint
```

## Arrival Speed Decision

The important function is:

```text
DifferentialAutoMode::arrivalSpeed()
```

PX4 sets `arrival_speed = 0` when:

- there is no valid next waypoint
- the transition angle requires a stop
- the current waypoint type is land
- the current waypoint type is idle

PX4 allows non-zero arrival speed when:

- a valid next waypoint exists
- the transition can be driven through
- transition-speed reduction still leaves a positive speed

Simplified:

```text
if no safe transition:
    arrival_speed = 0
else:
    arrival_speed = reduced_or_cruising_speed
```

This is one of the most important PX4 references.

PX4 decides stop vs transition before the vehicle reaches the waypoint. It does not wait until the corner to discover it must stop.

## Pure Pursuit Line Tracking

PX4 Pure Pursuit tracks the segment from previous waypoint to current waypoint.

Reference:

- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/lib/pure_pursuit/PurePursuit.cpp`
- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/lib/pure_pursuit/PurePursuit.hpp`

Function:

```text
PurePursuit::calcTargetBearing()
```

Inputs include:

```text
current waypoint
previous waypoint
current rover position
vehicle speed
lookahead parameters
```

The controller computes:

- lookahead distance
- rover projection onto the path segment
- crosstrack error
- bearing to current waypoint
- bearing to the lookahead/intersection point

Normal case:

```text
aim at lookahead point on the previous-current segment
```

Fallback cases:

- If close to the waypoint, aim directly at the waypoint.
- If crosstrack is larger than lookahead, aim toward the closest path point.
- If behind the previous waypoint extension, aim toward previous.
- If past/ahead of the current waypoint extension, aim toward current.

Important point:

PX4 does not simply point at the final waypoint for the whole segment. It follows the line geometry.

## Distance-Based Deceleration

The rover position controller is where PX4 makes stopping physically achievable.

Reference:

- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/rover_differential/DifferentialPosControl/DifferentialPosControl.cpp`

The controller computes:

```text
distance_to_target = norm(target_waypoint - current_position)
```

If `arrival_speed > 0`, it shifts the target distance toward the acceptance edge. That allows smoother through-waypoint behavior.

Then it caps the speed using:

```text
math::trajectory::computeMaxSpeedFromDistance(
    jerk_limit,
    decel_limit,
    distance_to_target,
    arrival_speed
)
```

Reference:

- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/lib/mathlib/math/TrajMath.hpp`

This means:

```text
allowed_speed = maximum speed that can still brake to arrival_speed by the target
```

If `arrival_speed = 0`, the controller reduces speed early enough to stop near the target.

This is the strongest reference behavior for proper corner stopping.

## Spot-Turn And Heading Error Logic

For differential rover, PX4 has a simple driving state machine:

- `DRIVING`
- `SPOT_TURNING`

If heading error is too large, it enters spot-turn mode and sets forward speed to zero while turning.

Relevant parameters:

- `RD_TRANS_DRV_TRN`
- `RD_TRANS_TRN_DRV`

Reference:

- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/rover_differential/DifferentialPosControl/DifferentialPosControl.cpp`
- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/rover_differential/module.yaml`

Simplified:

```text
if driving and heading error is too large:
    stop forward motion and spot turn

if spot turning and heading error becomes small:
    resume driving
```

This prevents the rover from pushing forward while pointed badly away from the desired bearing.

## Speed Reduction By Heading Error

When the rover is driving, PX4 can reduce speed based on heading/course error.

Relevant parameter:

- `RO_SPEED_RED`

Reference:

- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/lib/rover_control/rovercontrol_params.yaml`

Concept:

```text
larger heading error -> lower allowed speed
```

This reduces overshoot and improves stability while tracking.

## Low-Level Speed And Actuator Path

The rover position controller publishes:

- `rover_speed_setpoint`
- `rover_attitude_setpoint`

Then lower controllers convert those into:

- throttle setpoint
- steering/yaw-rate setpoint
- actuator motor outputs

Reference files:

- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/rover_differential/DifferentialVelControl/DifferentialVelControl.cpp`
- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/rover_differential/DifferentialAttControl/DifferentialAttControl.cpp`
- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/rover_differential/DifferentialRateControl/DifferentialRateControl.cpp`
- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/rover_differential/RoverDifferential.cpp`
- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/lib/rover_control/RoverControl.cpp`

Important behavior:

- Speed setpoints are slew-limited.
- Deceleration limits are applied.
- Yaw/rate setpoints are slew-limited.
- Final motor commands are constrained.

This means the stop behavior is distributed through multiple layers, not only the mission layer.

## Waypoint Reached And Mission Advance

Navigator checks whether the mission item is reached.

Reference:

- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/navigator/mission_block.cpp`
- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/navigator/mission_base.cpp`

Main sequence:

```text
if mission item reached:
    set seq_reached

    if autocontinue:
        advance mission
        set next mission items
```

If the waypoint has hold time, Navigator waits until the hold time is complete before advancing.

This hold logic uses:

```text
time_inside
_time_wp_reached
```

Important point:

Navigator advances based on reached/completed logic. It does not itself solve exact physical stopping. The controller must make the stop physically happen.

## Stop/Hold Behavior

In the rover position controller, when the rover is inside the stop region and the target is a stop target, it publishes zero speed.

Reference:

- `/Users/dyx_a1/Vetri/PX4-Autopilot/src/modules/rover_differential/DifferentialPosControl/DifferentialPosControl.cpp`

General behavior:

```text
if close enough to target and arrival_speed is zero:
    publish speed_body_x = 0
    hold yaw/current attitude behavior
```

In the local PX4 tree inspected here, there are additional local edits for loiter/current-position stop handling. Those appear in `git diff` and should be treated as local firmware work, not necessarily upstream PX4 behavior.

The local custom behavior includes:

- treating loiter as current-position zero-speed rover setpoint
- publishing additional zero rate/steering setpoints in a zero-speed stop case
- caching current position as target after full stop

Those custom pieces may be useful references, but they are not the core generic PX4 mission model.

## Important Parameters

Mission/acceptance:

```text
NAV_ACC_RAD
```

Pure Pursuit:

```text
PP_LOOKAHD_GAIN
PP_LOOKAHD_MIN
PP_LOOKAHD_MAX
```

Rover speed/deceleration:

```text
RO_SPEED_LIM
RO_ACCEL_LIM
RO_DECEL_LIM
RO_JERK_LIM
RO_SPEED_TH
RO_SPEED_RED
```

Differential rover spot-turn transition:

```text
RD_TRANS_DRV_TRN
RD_TRANS_TRN_DRV
```

## What PX4 Does Well

PX4 separates the job cleanly:

```text
Navigator:
    mission state, reached/advance, previous/current/next

Auto mode:
    converts mission triplet to rover setpoint
    decides arrival_speed

Position controller:
    pure pursuit line tracking
    speed cap from remaining stopping distance
    stop/hold command

Lower controllers:
    speed/yaw tracking
    acceleration/deceleration limits
    motor output constraints
```

That separation is why PX4 can stop or transition more predictably.

## Practical Takeaway

The working PX4 logic is:

```text
previous/current/next waypoint context
    -> decide arrival_speed
    -> pure pursuit tracks previous-current line
    -> decel cap based on distance remaining
    -> reach waypoint inside acceptance radius
    -> hold or advance depending on mission item
```

PX4 does not require exact zero-distance mission acceptance.

The exact stop quality comes from the controller layer:

```text
arrival_speed = 0
+ stopping-distance speed cap
+ zero-speed hold behavior
```

That is the correct PX4 reference model.

## Current Companion Rover Logic

This section describes the current companion-side rover behavior in this workspace:

- `/Users/dyx_a1/Vetri/PX4_DXP/src/rpp_controller_node.py`
- `/Users/dyx_a1/Vetri/PX4_DXP/src/rpp_path_conditioning.py`

Honest flag:

This is the current workspace code, not proof that the Jetson/PX4 vehicle has already executed it correctly in the field. Some of this logic is newly patched and still needs a real mission bag to prove the physical rover obeys it at corners and run ends.

Another important distinction:

PX4 internal rover AUTO mode uses `RoverPositionSetpoint` with `arrival_speed`.

The companion RPP stack does not use that PX4 `arrival_speed` contract directly. It sends OFFBOARD velocity vectors through MAVROS. So the companion must implement its own version of:

```text
approach shaping
position gate
stop dwell
hold/recover command
run advance
```

That is why the companion logic is more explicit around corner stop and run transitions.

## Current Path Intake

When a new `nav_msgs/Path` arrives, `_path_cb()` validates it and builds a run queue.

The path is converted into `(north, east)` tuples, and spray state is encoded from `pose.position.z`:

```text
z > 0.5 -> spray/mark point
z <= 0.5 -> transit/off point
```

The controller then decides how to track the geometry:

```text
tracking_profile = auto | segment | smooth
```

Current behavior:

- `segment` is used for straight lines, polylines, polygons, and hard-corner shapes.
- `smooth` is used for arcs, circles, splines, and point-leg style densified straight legs.
- `auto` classifies each run separately.

Reference:

- `_path_cb()`
- `_classify_auto_profile()`
- `_split_runs_by_flag()`
- `_split_run_at_corners()`
- `_merge_collinear_runs()`
- `split_leading_entry_transit()`

## Current Run Model

The companion does not track the entire mission path as one single object in auto mode.

It splits the path into runs:

```text
run 0 -> active path
run 1 -> next active path
run 2 -> next active path
...
```

Each run stores:

```text
poses
flags
profile
length
cum_s
closed
runtime_entry
```

The active run becomes `self._path`.

That means all tracking logic only sees the current run. When the run ends, the controller either:

- advances immediately if the next run is collinear/aligned, or
- stops at the boundary, pivots/alignment-holds, then starts the next run.

Honest flag:

This run model is not the same as PX4 Navigator's mission triplet. It is a companion-side substitute. It can work, but only if run splitting preserves the true physical stop/transition points.

## Runtime Entry Transit

Runtime GPS entry can prepend an OFF acquisition leg before the first mission waypoint.

The helper `split_leading_entry_transit()` separates that leading entry transit so smoothing does not accidentally erase or bypass waypoint 0.

Current behavior:

- If runtime entry is detected, the entry leg becomes its own run.
- That entry run is preserved even if it is shorter than the normal 5 cm sliver filter.
- The entry-to-mark boundary forces alignment if it transitions into a marked run.

Honest flag:

This fixes the class of bug where a tiny runtime entry sliver could be dropped and the rover could hot-start into the mark geometry. It still assumes the incoming path encoding correctly marks runtime entry.

## Sliver And Connector Handling

The controller has two different cleanup mechanisms:

1. Connector absorption
2. Degenerate run dropping

Connector absorption:

Very short connector segments between two real hard corners can be collapsed into one apex. This is meant to prevent an 8 cm connector from becoming its own fake pivot target.

Degenerate run dropping:

After runs are built, runs shorter than 5 cm are dropped, except runtime entry runs.

Honest flag:

This is a practical field fix, not a general computational-geometry proof. It protects known planner artifacts, but if a real mission intentionally contains a sub-5 cm mark entity, the code can treat it as a sliver unless it is protected by runtime-entry handling or upstream semantics.

## Segment Profile: Straight Lines, Legs, Polylines, Polygons

Segment profile tracks one segment at a time.

The active segment is:

```text
self._path[self._segment_idx] -> self._path[self._segment_idx + 1]
```

The controller:

1. Projects current rover position onto the active segment.
2. Computes distance along the segment to the segment end.
3. Chooses a lookahead point on the current segment only.
4. Commands a velocity vector toward that lookahead.
5. Slows near hard corners or final endpoints.
6. Stops/pivots only when needed.

This is intentionally different from smooth RPP, where lookahead can walk across multiple path samples.

Why:

For hard-corner marking, allowing lookahead to jump around the corner would cut the corner and smear the mark. Segment mode keeps the rover on the current side until the corner state machine allows transition.

## Segment Start Behavior

At mission start or new run start, the controller may need to face the first segment before tracking.

Current behavior:

- Run 0 defers the alignment decision until the first fresh pose is available.
- Later runs compare previous run exit heading to new run entry heading.
- If the heading change is below `segment_corner_threshold_deg`, it continues without a stop/pivot.
- If the heading change is hard, `_run_alignment_hold()` handles stop/pivot/settle before normal tracking.

Honest flag:

The companion does not push PX4 into a native AUTO waypoint state. It relies on PX4 OFFBOARD velocity behavior. The pivot is created by commanding a small velocity vector in the desired bearing, so PX4's differential rover logic sees a heading target from velocity bearing.

## Segment Hard-Corner Stop

For a hard corner inside a segment run, current logic enters the corner state when:

```text
not final_segment
and distance_to_corner <= segment_corner_acceptance_radius
```

Default acceptance radius:

```text
segment_corner_acceptance_radius = 0.05 m
```

But release from stop is stricter:

```text
corner_position_tolerance_m = 0.02 m
```

So the rover can begin braking around 5 cm from the corner, but it should not release the stop/pivot gate unless it is within 2 cm of the actual corner point.

The stop confirmation requires:

```text
position_ok == True
linear speed < segment_stop_speed_threshold
yaw rate < segment_stop_yaw_rate_threshold
held continuously for segment_stop_dwell_s
```

Default values:

```text
segment_stop_speed_threshold = 0.02 m/s
segment_stop_yaw_rate_threshold = 0.05 rad/s
segment_stop_dwell_s = 0.30 s
```

Honest flag:

This is the new correct direction. It fixes the old "velocity-only dwell" weakness by adding position certification. But the exact 2 cm number is aggressive and must be validated against RTK/EKF noise, latency, and drivetrain behavior.

## Segment Corner Hold Command

When the rover is not yet certified stopped at the corner, the companion does not only publish zero velocity.

It calls `_corner_hold_velocity()`.

Current behavior:

- If already inside the 2 cm position gate, use body-axis braking to bleed residual speed.
- If outside the gate, command a bounded velocity back toward the stop point.
- Decompose error into along-track and cross-track components.
- Apply along/cross gains.
- Damp with fresh EKF velocity when available.
- Cap the command with `segment_brake_velocity_cap_m_s`.

Default cap:

```text
segment_brake_velocity_cap_m_s = 0.18 m/s
```

Honest flag:

This is stronger than zero velocity, because zero velocity can let PX4 coast. But it is also an integration risk: the command is an arbitrary NED velocity vector near a corner, and PX4 differential rover derives heading from that velocity vector. The code tries to manage that risk with bounded caps and separate pivot logic, but the final truth is a bag from the rover.

## Segment Corner Pivot And Release

After the stop is certified, the controller pivots toward the next segment.

Important detail:

The rover is not pivoted by sending only yaw rate.

The code comments say PX4 rover differential OFFBOARD velocity mode derives heading from the velocity vector. So the companion commands a small velocity vector aimed at the next segment heading.

Current pivot speed:

```text
max(0.05, segment_min_corner_speed)
```

The velocity bearing is clamped into a forward cone so PX4 does not interpret the command as reverse and turn the wrong way.

The controller only releases the pivot when:

```text
corner stop complete
position still OK
heading error inside tolerance
yaw rate settled
linear speed settled, when velocity is fresh
settled for segment_align_settle_s
```

Default values:

```text
segment_heading_tolerance_deg = 2.0
segment_pivot_release_max_deg = 3.0
segment_align_settle_s = 0.20 s
segment_align_speed_threshold = 0.02 m/s
```

Honest flag:

The pivot release is now conservative. That is good for marking precision, but it can expose real-world deadlocks if velocity telemetry is noisy/stale or PX4 never settles yaw rate below threshold. There is a bounded stale-velocity fallback, but fresh telemetry saying "still moving" does not time out into release.

## Run Boundary Stop

A run boundary is the end of one entity/transit and the start of another.

When the active run reaches its final point and another run exists, `_hold_before_run_advance()` decides whether to stop or pass through.

Current behavior:

- If the next run does not require alignment, advance immediately.
- If the next run requires alignment, stop at the current run's final point.
- Use the same 2 cm corner position gate.
- Use the same velocity/yaw-rate dwell.
- Carry the stop confirmation into the next run so the new run can pivot without a duplicate stop.

This is the current answer to:

```text
how does the rover stop at a line end / leg end / segment end?
```

It should stop at the run endpoint before switching into a differently-headed next run.

Honest flag:

If the next run is classified as aligned/collinear, the code intentionally does not stop. That is correct for PRE/MARK/AFT on the same line, but wrong if upstream geometry incorrectly hides a real corner as collinear.

## Final Mission End

When the active run reaches its final point and there is no next run, the controller does not immediately publish DONE based only on position.

Current behavior:

1. It checks:

```text
dist_to_goal <= xy_goal_tolerance
path_travel_m >= min_travel
```

2. Then `_completion_settle_satisfied()` requires physical settle:

```text
fresh velocity below segment_stop_speed_threshold
for segment_stop_dwell_s
```

3. If velocity telemetry is stale, a bounded fallback allows DONE after the stale cap.

4. Until settle passes, it publishes zero velocity with `StateCode.APPROACH`.

5. Only after settle passes does it publish `StateCode.DONE`.

Honest flag:

This is a good fix for "position reached but rover still coasting." However, it still depends on `/velocity_local` being accurate enough. If velocity is biased low, DONE can be early. If velocity is noisy high, DONE can be delayed.

## Segment Final Endpoint Approach

For final segment endpoints, the controller now slows before the endpoint.

Current speed approach logic:

```text
approach_d = max(
    approach_velocity_scaling_dist,
    max_v^2 / (2 * max_linear_decel) + 0.10
)

if distance_to_endpoint < approach_d:
    speed = min(speed, max(segment_endpoint_approach_speed, max_v * scale))
```

Default values:

```text
approach_velocity_scaling_dist = 1.5 m
segment_endpoint_approach_speed = 0.03 m/s
max_linear_decel = 0.5 m/s^2
```

This is the companion-side equivalent of PX4's "slow before the point" behavior, but it is simpler than PX4's jerk-aware formula.

Honest flag:

This is not exactly PX4 `computeMaxSpeedFromDistance()`. It is a linear ramp plus braking-distance minimum. It is likely better than the old full-speed-to-end behavior, but it is less mathematically complete than PX4's jerk/decel planner.

## Smooth Profile: Arcs, Circles, Splines, Point Legs

Smooth profile uses the existing RPP path-following logic.

Current behavior:

1. Project rover onto the path.
2. Compute adaptive lookahead.
3. Walk along path samples to find lookahead point.
4. Compute curvature from vehicle to lookahead.
5. Regulate speed by predicted curvature and lateral acceleration.
6. Apply approach scaling near the run end.
7. Publish NED velocity vector and optional feedforward yaw rate.

Smooth profile is used for:

- arcs
- circles
- continuous-curvature paths
- densified point-mode straight legs

For open smooth runs, approach scaling is based on Euclidean distance to final waypoint, after enough path travel.

For closed smooth runs, approach scaling is based on remaining along-loop distance:

```text
remaining = run_length - path_travel_m
```

This prevents a circle from immediately slowing down just because first and last waypoints are near each other.

Honest flag:

Smooth profile still uses RPP lookahead and curvature regulation; it does not have a hard 2 cm corner stop unless the run ends and a run-boundary stop is required. That is correct for arcs/circles that should roll smoothly, but it is not the right mode for sharp mark corners.

## Point Or Point-Leg Behavior

The code distinguishes between a true one-pose run and a densified point-leg.

A one-pose segment run:

```text
len(self._path) < 2
```

Current behavior:

- If another run exists, advance immediately.
- If no next run exists, publish DONE.

A runtime/densified point leg:

- Can be classified as smooth.
- Can be resampled.
- Is tracked like a smooth straight leg.

Honest flag:

If the mission requires "drive to this point and physically stop there", the safest representation is not a single-point path. A single-point run has no segment direction and current segment logic can advance immediately. It should be represented as a leg with start and target, or handled by an explicit point-stop mode.

## Current Start-To-Stop Flow Summary

For a hard-corner segment mission, the current companion flow is:

```text
path received
    -> split/condition into runs
    -> apply run 0
    -> if start heading is wrong, align before driving
    -> track current segment
    -> slow before hard corner or final endpoint
    -> enter CORNER_STOP near the corner
    -> hold/recover exact corner point
    -> require 2 cm position gate + velocity/yaw-rate dwell
    -> pivot toward next segment
    -> require heading/velocity/yaw-rate settle
    -> advance segment or run
    -> repeat
    -> final endpoint reached
    -> require physical completion settle
    -> publish DONE
```

For a smooth arc/circle/point-leg run, the current flow is:

```text
path received
    -> classify as smooth
    -> optionally smooth/resample
    -> project onto path
    -> choose lookahead along path
    -> regulate speed by curvature
    -> apply end approach scaling
    -> at run end, stop/advance or settle DONE
```

For a collinear PRE/MARK/AFT line:

```text
path received
    -> split by spray flags
    -> merge collinear runs
    -> preserve spray flags
    -> track as one continuous straight motion
    -> no forced stop at spray-only boundary
```

## Honest Overall Verdict

The current companion code is much closer to the PX4 reference model than the earlier version.

Good:

- It now has explicit run context.
- It distinguishes segment vs smooth geometry.
- It slows before segment endpoints.
- It has a physical stop dwell.
- It now has a position gate at the actual corner/run boundary.
- It actively holds/recovers the stop point instead of trusting zero velocity.
- It avoids releasing a pivot while fresh telemetry says the rover is still moving.
- It delays final DONE until physical settle.

Still not fully PX4-equivalent:

- It does not use PX4's native `RoverPositionSetpoint.arrival_speed`.
- Its final endpoint speed shaping is linear, not PX4's jerk-aware planner.
- Corner hold sends NED velocity vectors near the stop point, which must be verified against PX4's heading/reverse behavior.
- A single-point run is not a true physical point-stop primitive.
- Classification errors upstream can still choose the wrong stop/transition behavior.
- The 2 cm position gate is only as good as pose latency, RTK quality, and local-frame consistency.

Production flag:

This code is structurally in the right direction, but the correct next proof is not another static argument. The proof is a Jetson/PX4 run with bags showing:

```text
corner position error
actual speed
yaw rate
RPP segment state
published velocity vector
run/segment index
DONE timing
```

Only that confirms whether the rover truly starts, tracks, stops at the point/leg/segment end, pivots, and transitions as intended.

## Exact Failure In Our Braking/Stop Logic

This is the honest failure statement.

The failure was not only "the rover did not stop at the corner."

The deeper failure was:

```text
the companion did not have a production-grade stop contract
```

It had tracking logic, speed scaling, corner logic, and run-advance logic, but the boundary between those behaviors was not a strict certified state machine.

In production terms, the controller must prove three different facts before it is allowed to transition:

```text
1. I am at the stop point.
2. I am physically stopped.
3. I am aligned for the next motion.
```

The old logic did not prove all three facts at the right time.

## Failure 1: Zero Velocity Was Treated Like Braking

The companion published zero velocity at stop moments.

That is not the same as braking.

On this rover/PX4 path, a zero velocity setpoint can let the rover coast, especially if it reaches the corner with residual speed. The rover can continue moving after the companion has already entered the "stop" phase.

Bad assumption:

```text
publish v = 0
    -> rover is stopped
```

Correct production assumption:

```text
publish v = 0
    -> requested stop only
    -> must verify actual speed and position before transition
```

Production fix:

The stop state must actively control the residual motion.

It needs one of these:

```text
preferred: controller-owned position hold at the stop point
acceptable: bounded reverse/brake velocity with position recovery
not enough: blind zero velocity
```

The current patched code moved toward this with `_corner_hold_velocity()`, but the production-grade design should formalize this as a state output, not an incidental helper call.

## Failure 2: Arrival Speed Was Too High

The rover sometimes reached the stop point too fast.

That means the stop state began after the physical system was already carrying too much momentum.

A corner stop cannot be accurate if the rover arrives at the corner with more kinetic energy than the hold/brake command can remove within the available distance.

Bad behavior:

```text
track segment at cruise speed
    -> enter corner acceptance radius
    -> command stop/brake
    -> rover coasts past corner
```

Correct behavior:

```text
compute remaining stopping distance before the corner
    -> reduce speed early
    -> enter stop gate already near stop speed
    -> certify stop
```

PX4 does this with `arrival_speed` and `computeMaxSpeedFromDistance()`.

Current companion code has a simpler ramp:

```text
approach_d = max(configured distance, v^2 / (2a) + margin)
speed scales down linearly inside approach_d
```

Honest flag:

That is better than the earlier behavior, but it is still less production-grade than a jerk/deceleration-aware arrival-speed planner.

Production fix:

Use an explicit arrival-speed planner per target:

```text
target.arrival_speed = 0      for stop points
target.arrival_speed > 0      for rolling transitions

v_allowed = f(distance_remaining, arrival_speed, decel_limit, jerk_limit)
commanded_speed = min(cruise_speed, v_allowed, curvature_limit)
```

For the companion, this can be implemented without PX4 AUTO by computing the same concept before publishing OFFBOARD velocity.

## Failure 3: Position Was Not A Release Gate

The old corner stop logic could confirm a stop using velocity and dwell even if the rover was not exactly at the intended corner point.

That is the core corner marking failure.

Bad release contract:

```text
speed is low
yaw rate is low
dwell time passed
    -> release corner
```

Correct release contract:

```text
distance_to_stop_point <= position_tolerance
speed is low
yaw rate is low
dwell time passed
    -> release corner
```

Current patched code adds:

```text
corner_position_tolerance_m = 0.02
```

and passes `position_ok` into `_corner_stop_satisfied()`.

Honest flag:

The logic is now correct in structure, but production validation must prove that 2 cm is stable with actual RTK, EKF latency, local-frame transforms, wheel slip, and MAVROS timing.

Production fix:

Make the position gate a first-class stop certificate:

```text
StopCertificate:
    target_id
    target_ned
    position_error_m
    along_error_m
    cross_error_m
    speed_m_s
    yaw_rate_rad_s
    stable_since
    certified_at
```

No transition may happen without a valid certificate.

## Failure 4: Timeout Could Replace Physical Truth

A timeout is useful for avoiding deadlock when telemetry disappears.

But a timeout must never override fresh evidence that the rover is still moving.

Old risky behavior:

```text
corner stop has waited too long
    -> release/pivot even if rover is still drifting
```

Correct behavior:

```text
if velocity telemetry is fresh and says moving:
    do not release

if velocity telemetry is stale:
    bounded fallback may release or fault, depending on mission policy
```

Current patched code moves in this direction:

- fresh velocity above threshold never releases due to timeout
- stale velocity can use a bounded fallback

Honest production flag:

For industrial marking, stale telemetry should probably not silently proceed at a precision corner. The safer production policy is:

```text
stale velocity during precision stop
    -> hold for bounded time
    -> mission CONTROL_FAULT
    -> do not mark the next segment as cleanly executed
```

Proceeding on stale data is useful during development, but it is not ideal production behavior for high-precision marking.

## Failure 5: Corner Stop And Pivot Were Not Separate Enough

A sharp corner has two separate operations:

```text
1. stop at the corner point
2. rotate/align for the next leg
```

Those must be separate states.

The failure occurs when the controller blends them:

```text
arrive near corner
    -> start turning
    -> residual forward motion carries rover past the point
```

Correct contract:

```text
TRACK_SEGMENT
    -> APPROACH_STOP
    -> STOP_HOLD
    -> STOP_CERTIFIED
    -> PIVOT_ALIGN
    -> ALIGN_CERTIFIED
    -> TRACK_NEXT
```

The current code now has `CORNER_STOP` and `CORNER_ALIGN`, which is the right direction.

Production fix:

Make the transition impossible in code unless a stop certificate exists.

In other words:

```text
CORNER_ALIGN may not run unless STOP_CERTIFIED was entered for the same target_id
```

## Failure 6: Run Boundary Was Treated Like Ordinary Goal Completion

A run boundary is not just "goal reached."

It can be:

```text
straight continuation
hard corner
spray-only boundary
runtime entry boundary
final mission end
```

Each has different stop requirements.

Bad design:

```text
distance_to_goal <= tolerance
    -> advance or done
```

Correct design:

```text
distance_to_goal <= tolerance
    -> classify boundary
    -> choose stop/roll/align/done policy
```

The current code does this partly with:

- `_next_run_requires_alignment()`
- `_hold_before_run_advance()`
- `_runtime_entry_to_mark_boundary()`
- `_merge_collinear_runs()`

Honest flag:

This is still distributed across helpers. Production-grade logic should centralize this into an explicit boundary classifier.

Production fix:

Create a boundary decision object:

```text
BoundaryDecision:
    boundary_type:
        FINAL_MISSION_END
        COLLINEAR_CONTINUATION
        SPRAY_ONLY_CONTINUATION
        HARD_CORNER
        RUNTIME_ENTRY_TO_MARK
        PROFILE_SWITCH
        POINT_STOP
    stop_required: bool
    align_required: bool
    target_stop_point: NED point
    target_exit_heading: rad
    arrival_speed: m/s
    tolerance: m
```

Then the FSM consumes the decision.

## Failure 7: There Was No Explicit Fault State

A production controller must distinguish:

```text
normal stop still settling
temporary telemetry stale
control fault
mission complete
emergency stop
```

Without explicit fault states, the system can only keep waiting, silently time out, or incorrectly advance.

Current code has safer gating, but the industrial-grade design should add explicit control faults.

Examples:

```text
STOP_POSITION_FAULT
STOP_VELOCITY_FAULT
PIVOT_ALIGNMENT_FAULT
POSE_STALE_FAULT
VELOCITY_STALE_FAULT
BOUNDARY_CLASSIFICATION_FAULT
```

These are not emergency stops by default. They are mission-control faults:

```text
stop motion
hold current point if possible
report precise reason
prevent marking next segment as valid
wait for operator/server decision
```

## Failure 8: The Controller Did Not Produce Enough Stop Evidence

For production marking, "state changed to DONE" is not enough.

The bag must prove why the controller believed it was allowed to advance.

Required evidence:

```text
target stop point N/E
current position N/E
position error
along error
cross error
actual speed
yaw rate
velocity freshness
pose freshness
stop dwell elapsed
state name
target heading
heading error
alignment dwell elapsed
run index
segment index
boundary type
published velocity command
published yaw-rate command
fault reason, if any
```

Honest flag:

The current debug topics are useful, but production certification would benefit from a dedicated stop/FSM diagnostic message or richer debug vector fields.

## Production-Grade FSM Proposal

This is the clean industrial-grade FSM the rover should use.

It is written as a design contract, not as an applied code patch.

## FSM Principles

Every state must define:

```text
entry action
continuous output command
exit guard
timeout/fault policy
diagnostic evidence
```

No state should "fall through" based on convenience.

Every transition should have a named reason:

```text
BOUNDARY_ROLL_THROUGH
APPROACH_ZONE_ENTERED
STOP_POINT_CAPTURED
STOP_CERTIFIED
ALIGN_CERTIFIED
FINAL_SETTLED
CONTROL_FAULT
```

## FSM State List

Recommended top-level states:

```text
NO_PATH
WAIT_POSE
WAIT_RTK
LOAD_RUN
START_ALIGN_DECIDE
RUN_ALIGN_STOP
RUN_ALIGN_PIVOT
RUN_ALIGN_SETTLE
TRACK_SEGMENT
TRACK_SMOOTH
APPROACH_STOP
STOP_HOLD
STOP_CERTIFY
PIVOT_ALIGN
PIVOT_SETTLE
RUN_ADVANCE
COMPLETION_STOP
COMPLETION_SETTLE
DONE
CONTROL_FAULT
SAFETY_ABORT
```

Some states can share implementation, but they should remain separate in the state model because they have different meanings.

## State: NO_PATH

Meaning:

No usable path is loaded.

Output:

```text
velocity = 0
yaw_rate = 0
```

Exit guard:

```text
valid path received
```

Fault policy:

None.

Diagnostic:

```text
state = NO_PATH
path_id = none
```

## State: WAIT_POSE

Meaning:

Path exists, but pose is unavailable or stale.

Output:

```text
velocity = 0
yaw_rate = 0
```

Exit guard:

```text
fresh pose available
```

Fault policy:

If mission is active and pose remains stale beyond production threshold:

```text
CONTROL_FAULT or SAFETY_ABORT
```

depending on server policy.

## State: WAIT_RTK

Meaning:

RTK-fixed requirement is enabled and GPS fix is not sufficient.

Output:

```text
velocity = 0
yaw_rate = 0
```

Exit guard:

```text
fix_type >= RTK_FIXED
```

Fault policy:

If RTK drops during marking:

```text
CONTROL_FAULT: RTK_LOST_DURING_MARK
```

Industrial note:

Do not silently continue marking precision lines without RTK if the job requires RTK.

## State: LOAD_RUN

Meaning:

Select active run, compute geometry, classify profile, reset per-run state.

Entry action:

```text
active_run = runs[run_idx]
path_progress = 0
segment_idx = 0
clear stop certificate
clear alignment certificate
```

Exit guard:

```text
run loaded and geometry valid
```

Fault policy:

```text
CONTROL_FAULT: INVALID_RUN_GEOMETRY
```

if run length, point count, or frame data is invalid for its intended type.

## State: START_ALIGN_DECIDE

Meaning:

Decide whether the rover must align before starting this run.

Inputs:

```text
current yaw
first segment heading
previous run exit heading, if any
boundary decision
```

Exit:

```text
if no alignment needed:
    TRACK_SEGMENT or TRACK_SMOOTH
else:
    RUN_ALIGN_STOP
```

Honest flag:

This state should not depend only on geometry. It must also account for runtime entry into MARK, because a mark entry can require a deliberate stop/align even if geometry looks simple.

## State: TRACK_SEGMENT

Meaning:

Track one hard segment from A to B.

Continuous command:

```text
project rover onto A->B
choose lookahead on A->B only
compute speed cap
publish velocity toward lookahead
```

Speed limit should be:

```text
speed = min(
    mission_speed,
    curvature_limit,
    arrival_speed_limit_to_target,
    accel_ramp_limit
)
```

Exit guards:

```text
if boundary approach zone entered and stop required:
    APPROACH_STOP

if segment endpoint reached and roll-through allowed:
    RUN_ADVANCE or next segment

if final point reached:
    COMPLETION_STOP
```

Fault policy:

```text
CONTROL_FAULT: XTRACK_TOO_HIGH
CONTROL_FAULT: SEGMENT_OVERSHOT_WITHOUT_STOP
CONTROL_FAULT: POSE_JUMP
```

Production note:

Tracking must never advance to the next segment solely because the lookahead crossed the corner.

## State: TRACK_SMOOTH

Meaning:

Track smooth geometry such as arc/circle/spline.

Continuous command:

```text
project onto path
compute lookahead along path
compute curvature
limit speed by lateral acceleration
limit speed by end arrival policy
publish velocity vector
```

Exit guards:

```text
smooth run end reached and next boundary is roll-through:
    RUN_ADVANCE

smooth run end reached and hard boundary follows:
    APPROACH_STOP / STOP_HOLD

final smooth end reached:
    COMPLETION_STOP
```

Fault policy:

```text
CONTROL_FAULT: LOST_PATH_PROJECTION
CONTROL_FAULT: CLOSED_LOOP_PROGRESS_INVALID
```

## State: APPROACH_STOP

Meaning:

The rover is near a stop-required target and must arrive with controlled residual speed.

Entry:

```text
target_stop_point fixed
arrival_speed = 0
stop_reason fixed
```

Continuous command:

```text
remaining = distance or along-track distance to stop point
speed_allowed = stopping_speed_limit(remaining, final_speed=0)
publish tracking velocity with speed_allowed
```

Exit guard:

```text
distance_to_stop_point <= stop_capture_radius
```

Suggested distinction:

```text
stop_capture_radius = 0.05 m
stop_certification_radius = 0.02 m
```

Fault policy:

If rover passes the stop point by more than a strict overshoot limit:

```text
CONTROL_FAULT: STOP_POINT_OVERSHOT
```

Honest production note:

This is where the current code is still weaker than PX4. It uses a linear ramp; production should use a stopping-speed function derived from measured decel/jerk capability.

## State: STOP_HOLD

Meaning:

The rover is at/near the stop target and must be actively held or recovered to that point.

Continuous command:

```text
if position_error > certification_radius:
    command bounded velocity toward stop point
else:
    command braking/zero hold

yaw_rate = 0
```

Exit guard:

```text
position_error <= certification_radius
actual_speed <= stop_speed_threshold
abs(yaw_rate) <= stop_yaw_rate_threshold
```

Then transition to `STOP_CERTIFY`.

Fault policy:

```text
if position_error grows:
    CONTROL_FAULT: STOP_HOLD_DIVERGING

if cannot enter certification radius within timeout:
    CONTROL_FAULT: STOP_POSITION_NOT_REACHED

if speed does not settle within timeout:
    CONTROL_FAULT: STOP_SPEED_NOT_SETTLED
```

Production note:

Timeout should not release the mission into pivot. Timeout should create a fault unless the mission explicitly allows degraded continuation.

## State: STOP_CERTIFY

Meaning:

The rover is inside the stop gate and appears physically stopped. Now prove it stays true for dwell time.

Continuous command:

```text
hold stop point
yaw_rate = 0
```

Exit guard:

```text
position_error <= certification_radius
speed <= stop_speed_threshold
yaw_rate <= stop_yaw_rate_threshold
for stop_dwell_s continuously
```

Output:

```text
StopCertificate created
```

Transition:

```text
if next boundary needs heading change:
    PIVOT_ALIGN
elif next boundary rolls through:
    RUN_ADVANCE
elif final:
    COMPLETION_SETTLE
```

Critical rule:

```text
any violation resets dwell
```

## State: PIVOT_ALIGN

Meaning:

The rover is stopped at the point and must turn toward the next leg.

Precondition:

```text
valid StopCertificate for this same target point
```

Continuous command:

```text
command firmware-aware pivot vector toward target heading
yaw_rate command = 0 unless body-rate mode is proven active for this rover
```

Exit guard:

```text
abs(heading_error) <= heading_tolerance
```

Then transition to `PIVOT_SETTLE`.

Fault policy:

```text
CONTROL_FAULT: PIVOT_NOT_PROGRESSING
CONTROL_FAULT: PIVOT_TIMEOUT
```

Production note:

If the rover drifts outside stop position tolerance during pivot, go back to `STOP_HOLD`, not forward to tracking.

## State: PIVOT_SETTLE

Meaning:

Heading is inside tolerance; now prove the rover is not still spinning or sliding.

Continuous command:

```text
hold/recover stop point
yaw_rate = 0
```

Exit guard:

```text
position_error <= certification_radius
heading_error <= release_heading_tolerance
speed <= align_speed_threshold
yaw_rate <= yaw_rate_threshold
for align_settle_s continuously
```

Output:

```text
AlignmentCertificate created
```

Transition:

```text
RUN_ADVANCE or TRACK_NEXT_SEGMENT
```

Critical rule:

No transition to tracking without both:

```text
StopCertificate
AlignmentCertificate
```

when the boundary is a hard corner.

## State: RUN_ADVANCE

Meaning:

Switch from one run/segment to the next.

Entry action:

```text
record completed run id
load next run
clear old tracking state
preserve valid stop certificate only if it applies to the boundary point
```

Exit:

```text
LOAD_RUN or START_ALIGN_DECIDE
```

Fault policy:

```text
CONTROL_FAULT: RUN_ADVANCE_WITHOUT_CERTIFICATE
```

if the boundary required a stop but no certificate exists.

## State: COMPLETION_STOP

Meaning:

The final mission endpoint has been reached. Do not report DONE yet.

Continuous command:

```text
hold final point
yaw_rate = 0
```

Exit:

```text
COMPLETION_SETTLE
```

## State: COMPLETION_SETTLE

Meaning:

Prove the rover is physically stopped before publishing mission DONE.

Exit guard:

```text
final position inside tolerance
speed below threshold
yaw_rate below threshold
stable for dwell
```

Output:

```text
FinalStopCertificate
StateCode.DONE
```

Fault policy:

```text
CONTROL_FAULT: FINAL_STOP_NOT_SETTLED
```

if the rover cannot settle within production timeout.

## State: DONE

Meaning:

Mission completed and final physical stop is certified.

Output:

```text
velocity = 0
yaw_rate = 0
DONE diagnostic
final certificate retained
```

Important:

`DONE` must mean:

```text
the mission is geometrically complete
and the rover is physically settled
```

not just:

```text
the rover passed near the last waypoint
```

## State: CONTROL_FAULT

Meaning:

Mission cannot continue with production confidence.

Output:

```text
velocity = 0 or hold-current-position command
yaw_rate = 0
fault diagnostic
mission remains incomplete
```

Fault examples:

```text
STOP_POINT_OVERSHOT
STOP_POSITION_NOT_REACHED
STOP_SPEED_NOT_SETTLED
PIVOT_TIMEOUT
PIVOT_NOT_PROGRESSING
POSE_STALE
VELOCITY_STALE
RTK_LOST
RUN_GEOMETRY_INVALID
BOUNDARY_CLASSIFICATION_INVALID
```

Industrial rule:

Do not label these as user emergency stops.

They are controller or mission execution faults.

The server should show them differently from:

```text
operator pressed emergency stop
```

## State: SAFETY_ABORT

Meaning:

Safety layer requires stopping the rover now.

Examples:

```text
FCU disconnected
pose stream dead
OFFBOARD unhealthy
server watchdog
operator emergency stop
```

Output:

```text
stop/hold command
mission terminal reason:
    safety_abort
    gps_safety_abort
    emergency_stop
```

Industrial rule:

Emergency stop by user and automatic safety abort must be distinguishable in logs.

## Production Stop Target Object

The FSM should operate on an explicit target object:

```text
StopTarget:
    id
    reason
    point_ned
    inbound_heading
    outbound_heading
    inbound_run_id
    outbound_run_id
    stop_capture_radius_m
    stop_cert_radius_m
    arrival_speed_m_s
    max_approach_speed_m_s
    stop_timeout_s
    allow_stale_velocity_fallback
```

Reasons:

```text
HARD_CORNER
RUN_BOUNDARY
RUNTIME_ENTRY_TO_MARK
FINAL_ENDPOINT
POINT_STOP
SAFETY_HOLD
```

This prevents ambiguous logic like:

```text
we are near some endpoint, maybe advance
```

The FSM should always know:

```text
which point
why stopping
what proves completion
what happens next
```

## Production Boundary Classifier

Before entering a boundary, classify it once.

Inputs:

```text
current run profile
next run profile
current endpoint
next start point
inbound heading
outbound heading
spray flags
runtime entry marker
mission item type
```

Output:

```text
BoundaryDecision
```

Example decisions:

```text
COLLINEAR_CONTINUATION:
    stop_required = false
    align_required = false
    arrival_speed = cruise

SPRAY_FLAG_ONLY_BOUNDARY:
    stop_required = false
    align_required = false
    arrival_speed = cruise

HARD_CORNER:
    stop_required = true
    align_required = true
    arrival_speed = 0

RUNTIME_ENTRY_TO_MARK:
    stop_required = true
    align_required = true
    arrival_speed = 0

FINAL_ENDPOINT:
    stop_required = true
    align_required = false
    arrival_speed = 0

SMOOTH_TO_SMOOTH_TANGENT:
    stop_required = false
    align_required = false
    arrival_speed = cruise_or_limited
```

Production rule:

The tracking controller should not infer boundary policy on the fly from scattered checks. It should consume this decision.

## Production Speed Planner

The speed planner should produce one number:

```text
speed_command_m_s
```

from explicit constraints:

```text
mission_speed
hardware_speed_limit
curvature_speed_limit
arrival_speed_limit
acceleration_limit
operator_precision_limit
```

For stop targets:

```text
arrival_speed = 0
distance_remaining = along-track distance to target
speed_arrival_limit = stopping_speed(distance_remaining, final_speed=0)
```

Recommended formula:

Use the PX4-style jerk/deceleration model or a measured drivetrain stop table.

Minimum production version:

```text
brake_distance = v^2 / (2 * measured_decel) + latency_margin + slope_margin
v_allowed = sqrt(2 * measured_decel * max(0, distance_remaining - cert_radius))
```

Better version:

```text
v_allowed = computeMaxSpeedFromDistance(
    jerk_limit,
    decel_limit,
    braking_distance,
    final_speed
)
```

Industrial note:

The deceleration limit must come from the actual rover on turf/concrete/field surface, not only from code assumptions.

## Production Position Hold Controller

The stop hold controller should be a real small controller, not just a helper.

Inputs:

```text
target point
current position
current velocity
heading
mode: recover / hold / brake
```

Outputs:

```text
velocity_ned
yaw_rate
```

Rules:

```text
if outside cert radius:
    drive toward target with bounded speed

if inside cert radius but moving:
    brake measured motion

if inside cert radius and stopped:
    command zero / hold
```

Limits:

```text
max_hold_speed
max_reverse_component
max_lateral_component
forward-cone constraint, if PX4 velocity-bearing mode is active
```

Honest flag:

Because PX4 differential rover derives desired heading from velocity bearing, arbitrary sideways hold vectors can create unexpected heading behavior. Production hold should either:

```text
1. use a position-capable setpoint interface, or
2. explicitly constrain hold velocity to firmware-safe bearings, or
3. prove with bags that the current NED hold vectors do not cause reverse/heading failures.
```

## Production Telemetry Requirements

A production FSM must publish enough evidence to debug every transition.

At minimum:

```text
fsm_state
fsm_state_elapsed_s
transition_reason
boundary_type
run_idx
segment_idx
target_stop_n
target_stop_e
position_error_m
along_error_m
cross_error_m
actual_speed_m_s
yaw_rate_rad_s
pose_age_ms
velocity_age_ms
heading_error_deg
stop_dwell_s
align_dwell_s
command_v_n
command_v_e
command_yaw_rate
stop_certificate_valid
alignment_certificate_valid
fault_code
```

Production rule:

Every mission bag should allow this question to be answered:

```text
why did the rover transition right here?
```

without guessing from indirect topics.

## Production Fault Policy

Development fallback:

```text
velocity stale for 2 s
    -> proceed to avoid deadlock
```

Production precision policy:

```text
velocity stale during stop certification
    -> hold for short grace
    -> CONTROL_FAULT
    -> do not proceed into marking
```

Recommended production defaults:

```text
stale velocity during TRACK:
    controlled stop or safety abort, depending on pose quality

stale velocity during STOP_HOLD:
    hold command, then CONTROL_FAULT

stale velocity during PIVOT_SETTLE:
    hold command, then CONTROL_FAULT

fresh velocity says moving:
    never timeout-release

position error outside cert gate:
    never timeout-release
```

## Minimal Production Acceptance Criteria

Before calling this industrial-grade, prove these with bags:

1. Straight line final endpoint:

```text
does not pass endpoint by more than tolerance
DONE only after speed dwell
```

2. 90 degree hard corner:

```text
stops within 2 cm
speed < threshold before pivot
pivots after stop certificate
enters next leg with heading error within release tolerance
```

3. Runtime entry to mark:

```text
entry run is preserved
stop/alignment occurs before MARK if required
MARK does not hot-start with wrong heading
```

4. Collinear PRE/MARK/AFT:

```text
does not force unnecessary stop at spray flag boundary
spray flag changes without speed dip
```

5. Circle/closed loop:

```text
does not declare DONE at seam start
travels required loop fraction
slows only near real completion
```

6. Telemetry stale cases:

```text
fresh moving never timeout-releases
stale velocity creates CONTROL_FAULT or documented degraded behavior
operator can see exact reason
```

7. Overshoot fault:

```text
if rover passes stop point beyond allowed overshoot
mission faults instead of pretending corner was clean
```

## Clean Production Architecture Summary

The industrial-grade version should look like this:

```text
Path/Run Builder
    -> Boundary Classifier
        -> StopTarget / BoundaryDecision
            -> FSM
                -> Speed Planner
                -> Path Tracker
                -> Stop Hold Controller
                -> Pivot Controller
                -> Certificate Publisher
                -> Fault Reporter
```

The most important change is conceptual:

```text
tracking code should not decide mission truth casually
```

Instead:

```text
only the FSM can advance mission truth
and only with certificates
```

## Final Honest Recommendation

For real production marking, the next implementation should not be another small patch around corner release.

It should be a formal FSM refactor around stop certificates.

Minimum production-grade target:

```text
hard corner cannot pivot without StopCertificate
next leg cannot track without AlignmentCertificate
DONE cannot publish without FinalStopCertificate
timeouts create CONTROL_FAULT, not silent success
every transition emits evidence
```

That is the difference between "works in one bag" and "industrial-grade rover behavior."

## Cursor Review Audit Addendum

Date: 2026-07-07

This section records the read-only review of this document and my audit of that review against the current workspace.

No code changes are implied by this section.

## Review Verdict

The Cursor review is mostly correct.

It correctly identifies that this document is three things in one:

```text
PX4 reference
current companion as-built audit
production FSM proposal
```

That is useful, but it creates one risk:

```text
a reader can mistake proposed production FSM behavior for behavior already implemented
```

That risk is real.

The document must be read with this split:

```text
PX4 reference:
    how PX4 native rover AUTO is structured

Current companion logic:
    what this workspace currently does

Failure/FSM sections:
    what failed and what production-grade design should become
```

## What The Review Got Right

The review correctly confirms these points:

1. The PX4 reference section is directionally accurate.

PX4 native rover mission behavior uses:

```text
Navigator triplet
arrival_speed
RoverPositionSetpoint
Pure Pursuit line tracking
distance-based speed cap
acceptance-radius mission advance
```

2. The core companion difference is correct.

The companion stack does not use PX4 native rover `arrival_speed` control directly.

It sends OFFBOARD setpoints through MAVROS, so the companion has to implement its own stop contract:

```text
approach shaping
position gate
physical stop dwell
hold/recover
run advance
```

3. The current companion section matches the code.

The review correctly maps document claims to code for:

```text
_runs
_apply_run()
_advance_run()
split_leading_entry_transit()
_corner_hold_velocity()
_corner_stop_satisfied(position_ok=...)
_completion_settle_satisfied()
segment_endpoint_approach_speed
corner_position_tolerance_m
```

4. The honest flags are valid.

The review agrees that these are real gaps:

```text
new patch still needs field bags
2 cm gate is aggressive
hold vectors may interact with PX4 heading/reverse behavior
linear ramp is weaker than PX4 jerk-aware planning
single-point run is not a true point-stop primitive
classification errors can still cause wrong stop/transition behavior
```

5. The production FSM section is design only.

The review correctly says these are not implemented yet:

```text
StopCertificate
AlignmentCertificate
BoundaryDecision
StopTarget
CONTROL_FAULT FSM state
APPROACH_STOP / STOP_HOLD / STOP_CERTIFY as real states
PX4-style computeMaxSpeedFromDistance planner
dedicated stop/FSM diagnostic topic
```

## Important Nuance: Not Pure Velocity-Only

The review uses the phrase "velocity-only OFFBOARD."

That is directionally right for the stop discussion, but technically it needs nuance.

Current `twist_to_setpoint_node.py` ignores position:

```text
IGNORE_PX
IGNORE_PY
IGNORE_PZ
```

So PX4 is not receiving a local position target for the corner stop.

However, the current bridge can send:

```text
velocity
explicit yaw
yaw_rate feedforward
```

with type mask:

```text
455
```

So the precise statement is:

```text
the companion is position-masked OFFBOARD, not necessarily yaw/yaw-rate-masked OFFBOARD
```

Why this matters:

The stop failure is not because yaw/yaw-rate are always absent.

The stop failure is because the companion does not hand PX4 a position target with an `arrival_speed = 0` contract. The companion must prove and hold the stop point itself.

## What The Review Slightly Understates

The review says the document is useful but long.

That is true, but the deeper issue is not length alone.

The deeper issue is:

```text
state of implementation is mixed with target architecture
```

For engineering use, this document now needs section labels that stay mentally sharp:

```text
REFERENCE
AS-BUILT
FAILURE
PROPOSED
NOT IMPLEMENTED
FIELD-PROOF REQUIRED
```

Without that separation, a future reader could assume the FSM/certificate design is already live.

## Current Code Status After Review

As of this audit, the current companion code has:

```text
2 cm corner position release gate
fresh velocity/yaw-rate dwell
no timeout release while fresh telemetry says moving
active stop-point hold/recover command
final DONE physical settle
runtime entry preservation
linear endpoint approach ramp
```

The current companion code does not yet have:

```text
formal StopCertificate object
formal AlignmentCertificate object
central BoundaryDecision classifier
explicit CONTROL_FAULT state machine
production stale-telemetry fault policy
jerk-aware arrival speed planner
separate corner hold velocity cap
dedicated stop evidence topic
```

## Most Important Review Finding

The most important finding from the review is this:

```text
the document describes a correct production direction,
but the rover is not production-proven yet
```

The current patch may fix the corner-stop class of bug structurally, but production status requires field evidence.

Required bag proof:

```text
RPP state enters CORNER_STOP at the intended target
position error enters and stays within tolerance
actual speed falls below threshold
yaw rate falls below threshold
dwell completes
CORNER_ALIGN starts only after stop certification
heading settles before next leg tracks
final DONE happens only after physical settle
```

## Document Gaps Confirmed By Review

The review points out several document gaps that are fair.

### Gap 1: Missing Bag Numbers

This document describes failure modes, but it does not embed the strongest measured values from the mission bags.

The missing evidence includes values such as:

```text
arrival speed near failed corner
actual drift distance
command vector direction during stop
time missed before dwell completion
terminal reason mapping
```

This document should eventually link the failure section to the actual bag-forensics output.

### Gap 2: Layer A Is Not Bold Enough

The document says the current code uses a linear ramp and not PX4 `computeMaxSpeedFromDistance()`.

That is accurate.

But the unimplemented item should be called out more sharply:

```text
Kinematic/jerk-aware arrival planner is proposed, not implemented.
```

Current code still uses:

```text
linear approach ramp
braking-distance minimum
```

not:

```text
full arrival-speed planner
```

### Gap 3: Fault Policy Is Proposed

The document recommends:

```text
CONTROL_FAULT instead of silent timeout success
```

That is not live yet.

Current code still has bounded stale-velocity fallback in some places.

Production should change stale precision-stop behavior from:

```text
fallback proceed
```

to:

```text
hold briefly
then CONTROL_FAULT
```

unless the operator explicitly accepts degraded behavior.

### Gap 4: Hold Cap Is Shared With Brake Cap

The review correctly notes:

```text
_corner_hold_velocity() uses segment_brake_velocity_cap_m_s
```

There is no separate:

```text
corner_hold_velocity_cap_m_s
```

That means the same 0.18 m/s cap is doing two jobs:

```text
active braking cap
position recovery/hold cap
```

Production should separate these because hold/recover near a 2 cm gate may need a lower, more precise cap than braking residual travel speed.

## Updated Honest Assessment

After reading the review and checking the code again, the honest status is:

```text
PX4 reference section:
    useful and directionally accurate

current companion section:
    matches the current workspace

failure analysis:
    aligned with the known corner-stop failure

production FSM:
    good target architecture, not implemented

industrial readiness:
    not proven
```

Current code is better described as:

```text
patched toward stop certification
```

not:

```text
industrial-grade stop FSM
```

## Next Documentation Part

The next useful documentation section should be a compact implementation delta:

```text
Current Behavior
Proposed Production Behavior
Missing Code
Field Proof Required
```

That section should make it impossible to confuse:

```text
what runs today
```

with:

```text
what the FSM design says should run later
```

Recommended next section title:

```text
Current Vs Proposed Production Delta
```

Recommended table columns:

```text
Area
Current code
Production target
Missing implementation
Proof required
```

This would turn the document from a long design narrative into an actionable build checklist.

## FLAG-01 Addendum: Runtime Path Identity Vs Stop-Only Testing

Date: 2026-07-07

Detailed note:

- `docs/after_stop_Fix.md`

### Verdict

```text
DOES NOT BLOCK STOP-ONLY
BLOCKS FULL PIPELINE ONLY
```

FLAG-01 is real:

```text
staged mission fingerprint can describe staged waypoints + spray_flags,
while GPS_SURVEYED runtime placement and runtime-entry geometry can change
the actual published /path.
```

If `/path/identity` still publishes the old staged fingerprint, then
`/rpp/conditioned_path_identity` can carry a fingerprint that does not honestly
describe the runtime geometry. The spray controller may then reject the mission
or stay OFF because the conditioned geometry does not match the configured
staged fingerprint.

That is mainly a full pipeline blocker:

```text
DXF -> GPS_SURVEYED placement -> runtime path -> spray binding -> paint output
```

It is not, by itself, a blocker for Phase 1.1 stop-certificate dry-run testing.

RPP stop certificates are based on the actual runtime geometry and telemetry:

```text
actual /path
actual /rpp/conditioned_path
position
velocity
yaw rate
heading error
dwell timing
```

They are not certified from the staged fingerprint.

### Required Constraint

For Phase 1.1 stop-only field validation, do not use staged DXF geometry as the
sole truth source. Validate stop targets and transitions against:

```text
/path
/rpp/conditioned_path
/rpp/stop_debug
/rpp/segment_debug
```

Preferred clean test:

```text
LOCAL_NED path
no runtime-entry rewrite
spray disabled or not required
require_stop_certificates=true
```

Acceptable dry-run test:

```text
GPS_SURVEYED/runtime-entry allowed
spray not required
bag analysis uses actual runtime path geometry
```

Do not claim full production validation until FLAG-01 is fixed and a bag proves
the full DXF/GPS_SURVEYED/spray identity chain.
