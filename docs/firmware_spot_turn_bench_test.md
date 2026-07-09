# Firmware spot-turn bench test

**Question it answers:** does the flashed PX4 (`rover_differential`, v1.16.2) **rotate in
place** (SPOT_TURNING, zero forward throttle, wheels counter-rotate) when the companion
sends a velocity vector whose bearing is >10° off the nose — or does it **drive/arc**?

This single result decides the whole runtime-entry pivot fix:
- **Spins in place** → the code is already right; drop the companion ±75° forward-cone
  clamp for the pivot and let PX4 spot-turn. No recenter tuning needed.
- **Drives/arcs** → the design's premise ("±75° cone → PX4 SPOT_TURNING") is false on this
  build → reflash the spot-turn firmware, or go geometric (aligned run-in / accept a small
  entry error and let the PRE extension absorb it).

Every companion-side fix we tried (recenter, cap, true-stop) failed for the same reason:
the rover *drives* instead of spot-turning, so any big-angle pivot arcs. This test confirms
that at the firmware level.

---

## SAFETY FIRST
- **Do Phase 1 with the wheels OFF THE GROUND** (rover on blocks/stands, wheels free). It
  arms and commands motion — on blocks there is zero runaway risk and you can watch pure
  rotation.
- Have the **RC / e-stop ready** the entire time.
- **No mission running** (rover idle) so only the test setpoint drives `cmd_vel` — otherwise
  rover-server/rpp-pipeline will fight for the setpoint. Do NOT stop `px4-dxp.service`
  (it carries the QGC bridge).
- Disarm immediately after each observation.

---

## Phase 0 — Does the firmware even have the spot-turn params? (no motion)
In QGC → Parameters, search:
- `RD_TRANS_DRV_TRN`  (drive→turn threshold, default 0.1745 rad = 10°)
- `RD_TRANS_TRN_DRV`  (turn→drive hysteresis, default 0.0873 rad = 5°)

- **Params MISSING** → flashed firmware predates the spot-turn state machine → answer is
  already "reflash needed." Skip the drive test.
- **Params present** → note their values, continue to Phase 1.

---

## Phase 1 — Wheels-off-ground spot-turn test (the decider)

1. Rover on blocks, wheels free. Note the rover's physical heading (say it faces **North**).

2. From the Jetson (`ssh flash@192.168.1.102`, `source` the ROS2 env), start streaming a
   velocity vector pointing ~90° off the nose. The companion twist is ENU (x=East, y=North),
   so to command **East** (90° off a North-facing rover) at 0.10 m/s:
   ```bash
   ros2 topic pub -r 10 /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped \
     '{header: {frame_id: "map"}, twist: {linear: {x: 0.10, y: 0.0, z: 0.0}}}'
   ```
   Leave this running in one terminal (it must stream ≥2 Hz BEFORE OFFBOARD).

3. In a second terminal, start a ulog (or just watch QGC MAVLink Inspector), then arm +
   OFFBOARD:
   ```bash
   ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'
   ros2 service call /mavros/set_mode  mavros_msgs/srv/SetMode  '{custom_mode: "OFFBOARD"}'
   ```

4. **Observe for ~5 s**, then disarm:
   ```bash
   ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: false}'
   ```
   Stop the `ros2 topic pub`.

### What to watch
- **Wheels (physical):** do the two sides **counter-rotate** (one forward, one reverse =
  spin in place), or do **both drive forward** (= it would translate/arc)?
- **Throttle telemetry** — QGC → MAVLink Inspector, or read the ulog afterward:
  - `rover_throttle_setpoint.throttle_body_x` — **≈ 0** during the turn = SPOT_TURNING;
    **> 0** = DRIVING.
  - `rover_velocity_status.adjusted_speed_body_x_setpoint` vs `measured_speed_body_x`.

---

## Interpretation → next action

| Observation | Meaning | Next move |
|---|---|---|
| Wheels counter-rotate, `throttle_body_x ≈ 0` | **PX4 spot-turns** | Companion fix: stop clamping the pivot bearing to ±75°; send the raw bearing-to-target (>10° off nose) so PX4 spot-turns. Entry pivot then works with **no recenter**. |
| Both wheels drive, `throttle_body_x > 0`, rover would translate | **PX4 drives, no spot-turn** | Firmware side: reflash a build where the spot-turn engages (verify `RD_TRANS_*`), OR go geometric — a short aligned run-in / accept ~5 cm entry error and let the PRE extension absorb it. Companion recenter is a dead end. |

---

## Phase 2 — On-ground confirm (only if Phase 1 spun in place)
Clear flat area, e-stop in hand. Repeat the same command with wheels on the ground. Confirm
it rotates roughly in place (small translation) rather than arcing away. Then the companion
change (drop the ±75° clamp for the pivot) can be trusted for a field run.
