# G1-OFFBOARD Bench Procedure — prove firmware ingests an OFFBOARD position go-to

**Gate:** G1-OFFBOARD (`MISSION_ALIGNMENT_RUNTIME_ENTRY_PLAN.md` §5.0). Blocks Epic 3 field ship.
**Base:** `test/colinear-fix` (controller pin `cd44884` — do not touch).
**Runtime:** Jetson `192.168.1.102` (`flash`), FCU CubeOrangePlus PX4 v1.16.2.
**Est. time:** ~15 min. **Requires a human watching the rover.**

AUTO logs (`log_23`, `log_24`) prove PosControl stop quality *under Navigator only*. They do **not** prove OFFBOARD position ingestion. This bench is the only thing that does.

---

## 0. Pass bar — score TWO things separately

The two failure modes route to different contingencies, so do not collapse them into "did it stop near the target."

| # | Question | Evidence (FCU ULog) | If NO |
|---|---|---|---|
| **R — Routing** | Did the OFFBOARD position setpoint reach rover PosControl? | `rover_position_setpoint` topic present **and populated** (finite x/y); `offboard_control_mode.position == true`; `vehicle_control_mode.flag_control_offboard_enabled == true` | **C4** — OFFBOARD position no-ops on this firmware. Epic 3 D4 architecture collapses; fall back to velocity chase / disable firmware entry. |
| **Q — Quality** | Did it converge and stop cleanly? | `vehicle_local_position` reaches target, speed → 0, holds, **no hunt/oscillation**; stop error ≤ 5 cm | **C1/C2** — routing works but tune arrival: keep `NAV_ACC_RAD=0.05`, let RPP clean up in PRE, or companion arrival radius 5 cm. **Not** a hard fail. |

**GO to build Epic 3 lifecycle** requires **R = PASS**. Q shapes contingency, not go/no-go.

> Routing caveat: if `rover_position_setpoint` is simply *absent* from the ULog, first confirm it's in the logged topic set (`SDLOG_PROFILE`) before concluding C4 — absence-from-log ≠ absence-from-firmware. Populated-but-ignored (present, finite, but the rover doesn't move toward it) **is** a true C4.

---

## 1. Preconditions

- [ ] Rover on stands **or** ≥2 m clear space ahead — it will **arm and drive to absolute local point (N=1.0, E=0.0)**. From an off-heading start it will turn first (that's the point).
- [ ] RTK **FIXED** (surveyed placement not required for this bench; a valid EKF local pose is). Confirm: `ros2 topic echo /mavros/local_position/pose --once`.
- [ ] Spray disabled / hardware off — this bench must not actuate spray.
- [ ] E-stop physically in reach.
- [ ] FCU params at baseline (`RO_YAW_P=1.5`, `RO_JERK_LIM`/`RO_DECEL_LIM`/`RO_SPEED_LIM` > 0, `NAV_ACC_RAD=0.05`). Do **not** push params from Jetson — verify in QGC.

**Off-heading setup (recommended):** park the rover so local point `(1.0, 0.0)` is ~40–90° off its nose. That exercises turn-then-converge, mirroring a real entry, without the reverse-flip regime. Save a full reverse (~160°) start for the G3 large-yaw handoff bag, not this routing bench.

---

## 2. Free the setpoint topic (mutex)

`twist_to_setpoint_node` streams `/mavros/setpoint_raw/local` at 50 Hz whenever `rpp-pipeline` runs. Two publishers on that topic = garbage. Stop **only** `rpp-pipeline` — `px4-dxp` (MAVROS + QGC bridge + RTK) stays up.

```bash
ssh flash@192.168.1.102
sudo systemctl stop rpp-pipeline          # does NOT drop MAVROS (px4-dxp keeps it)

# confirm the topic is now silent (Ctrl-C after a few seconds — expect NO messages):
ros2 topic hz /mavros/setpoint_raw/local
pgrep -af twist_to_setpoint               # expect: nothing
```

Do **not** stop `px4-dxp` (carries the QGC telemetry bridge and MAVROS). Leave `rover-server` alone — it does not publish setpoints; just don't start a mission from the app during the bench.

---

## 3. Start the bag (separate terminal on Jetson)

Capture the companion-side truth alongside the FCU ULog.

```bash
TS=$(date +%Y%m%d_%H%M%S)
ros2 bag record -o ~/bags/g1_offboard_${TS} \
  /mavros/local_position/pose \
  /mavros/setpoint_raw/local \
  /mavros/setpoint_raw/target_local \
  /mavros/state
```

- `setpoint_raw/local` = what the companion **sent**.
- `setpoint_raw/target_local` = MAVROS echo of what PX4 **accepted** (sanity that the message wasn't rejected at the MAVLink layer).
- `state` = mode/armed transitions.

---

## 4. Run the test node

Default mode is `position` (drives to local `(1.0, 0.0)`). The node self-sequences: 1 s preflight stream → OFFBOARD → arm → drive → hold 2 s → stop → disarm → back to MANUAL. It already streams ≥2 Hz before the OFFBOARD request and disarms on any unexpected mode change or Ctrl-C.

```bash
cd ~/PX4_DXP
# primary form (direct, most robust — ROS env is sourced by the login shell):
python3 src/offboard_test.py --ros-args -p mode:=position -p forward_dist:=1.0

# equivalent if the package build exposes it:
#   ros2 run px4_dxp offboard_test.py --ros-args -p mode:=position
```

Watch the console. Expected happy path:
```
FCU connected ... Position estimate: YES
Step 2: Requesting OFFBOARD mode... Mode switch to OFFBOARD: sent
Waiting for OFFBOARD mode confirmation...      # must actually reach OFFBOARD
Step 3: Arming... Arm: success
Step 4: Driving forward 1.0m...                # rover turns toward (1,0) then drives
Step 5: Holding position for 2s...
=== POSITION MODE TEST COMPLETE ===
```

**Abort triggers (any of):** rover heads the wrong way, overshoots hard, oscillates, or you're unsure → **Ctrl-C** (node stops + disarms) and/or physical E-stop. A failed run is still data — bag it and note what happened.

Common stumbles:
- `Failed to enter OFFBOARD` / `Mode did not switch` → a setpoint stream gap or a second publisher still alive (re-check §2), or FCU rejected OFFBOARD (pre-arm). This is **not** an ingestion result — fix and re-run.
- `Arm: DENIED` → GPS/pre-arm. RTK FIXED should clear it; do not set `COM_ARM_WO_GPS` for a real drive test.

---

## 5. Restore runtime immediately after

```bash
sudo systemctl start rpp-pipeline
pgrep -af twist_to_setpoint                 # expect: running again
ros2 topic hz /mavros/setpoint_raw/local    # expect: ~50 Hz again (Ctrl-C)
```

---

## 6. Pull and score the FCU ULog (authoritative)

The `.ulg` is FCU-side, downloaded via QGC to the Mac daily-logs dir. After the run, the **newest** `.ulg` is this bench:

```
/Users/dyx_a1/Documents/QGroundControl Daily/Logs/<DD-MM-YYYY>/
```

Inspect for the four routing signals + the quality trace (pyulog / Flight Review / PlotJuggler):

```bash
# on the Mac, with pyulog installed:
ulog_info   "<newest>.ulg"          # does the topic list include rover_position_setpoint?
ulog_messages "<newest>.ulg"
# then dump / plot:
#   rover_position_setpoint            -> R: present + finite x,y  (HARD BAR)
#   offboard_control_mode.position     -> R: true during the leg
#   vehicle_control_mode.flag_control_offboard_enabled -> R: true
#   trajectory_setpoint.position       -> R: finite (the streamed target)
#   vehicle_local_position (x,y,vx,vy) -> Q: converges to (1,0), speed->0, no hunt
```

Cross-check against the ROS bag: `/mavros/setpoint_raw/local` should show a steady position-masked target, and `/mavros/state` should show `OFFBOARD` + `armed` across the drive.

---

## 7. Record the result

Write the verdict back into `MISSION_ALIGNMENT_RUNTIME_ENTRY_PLAN.md`:
- §5.0 "How to close G1-OFFBOARD" → PASS/FAIL + ULog filename.
- §6 gate table → G1-OFFBOARD row status.

Template:
```
G1-OFFBOARD <PASS|FAIL> — <YYYY-MM-DD>, ULog <name>.ulg
  R (routing):  <PASS|FAIL>  rover_position_setpoint <populated|absent|present-but-ignored>
  Q (quality):  <PASS|C1|C2> stop err <X> cm, hunt <none|observed>
  -> <GO build Epic 3 lifecycle | C4 disable firmware entry>
```

Then update the memory file `epic12_placement_landed_2026_07_11.md` (Epic 3 gate status).

---

## Safety recap
- Only `rpp-pipeline` is stopped; `px4-dxp` (MAVROS/QGC/RTK) stays up.
- Human watching; E-stop in reach; spray off.
- Ctrl-C = clean stop + disarm; node also disarms on any unexpected mode change.
- Do not push FCU params from Jetson. Do not touch the frozen `cd44884` controller.
- One publisher on `/mavros/setpoint_raw/local` at all times.
