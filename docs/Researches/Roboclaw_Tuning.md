I have the official BasicMicro procedure. Now let me give you the full systematic tuning guide tailored to your setup. (Content rephrased from the BasicMicro Motion Studio docs for compliance with licensing.)

---

# 🎯 Complete Tuning Procedure: 3WD Marking Rover with RoboClaw + PX4

## Goal
Match left and right wheel speeds so the rover drives perfectly straight, follows arcs cleanly, and holds speed on slopes/grass.

---

## 📋 Phase 0 — Prerequisites (do these once, then forget)

| Item | Status | Where to check |
|---|---|---|
| New firmware flashed | Required | Run `7a229fe9` build artifact through QGC |
| RoboClaw connected to a Cube TELEM port | Required | Wired by you |
| `RBCLW_SER_CFG` set to that TELEM port | Required | QGC → Parameters |
| Encoder wires to ENC1/ENC2 on RoboClaw | Required | Hardware |
| USB cable from PC to RoboClaw (separate from Cube) | Required | For Motion Studio |
| **Wheels lifted off the ground** for PHASE 1 & 2 | ⚠️ Critical for safety | Block the rover on stands |

> ⚠️ **Throughout phases 1 and 2, the rover wheels must be off the ground.** You will be running motors at 100% with no load control. If wheels are on the ground, the rover will lurch forward at full speed.

---

## 🔧 Phase 1 — Bench setup with BasicMicro Motion Studio

This phase happens on the **PC connected to the RoboClaw via USB**, with the Cube powered OFF (or RoboClaw serial cable disconnected from Cube).

### Step 1.1 — Install Motion Studio
- Download: https://www.basicmicro.com/downloads
- Install on Windows. Plug RoboClaw USB → Motion Studio detects it.

### Step 1.2 — Verify motor wiring direction
Per the official procedure (rephrased):

1. Open **PWM Settings** (left menu).
2. Slide **Motor 1** slider up → motor should spin **forward**.
3. Slide **Motor 2** slider up → motor should spin **forward**.
4. If a motor spins backward, **power down the RoboClaw** and physically swap the two motor wires (M1A↔M1B or M2A↔M2B). Repeat the test.

> Why: PX4 expects "positive command = forward". If wiring is wrong, your closed loop will run away in the wrong direction.

### Step 1.3 — Verify encoder direction
Still in **PWM Settings**:

1. Slide Motor 1 slider up (forward).
2. Watch the **M1 Encoder** count box at the top — it should **increase**.
3. Repeat for Motor 2 / M2 Encoder.
4. If a count goes *down* during forward motion, power down and swap the encoder A/B wires on the ENC1 or ENC2 header.

> Why: If the encoder reads "going backward" while the motor goes forward, the velocity PID will compensate wrongly and slam the motor at full power in the wrong direction.

### Step 1.4 — Find the maximum QPPS for each motor
Still in **PWM Settings**:

1. Run **Motor 1** at 100% (slider all the way up). Hold for ~3 seconds.
2. Read the **M1 Speed** value at the top of the window. Write it down. Example: `8240`.
3. Drop slider to 0. Wait for motor to stop.
4. Run **Motor 2** at 100%. Read **M2 Speed**. Write it down. Example: `8155`.
5. Drop slider back to 0.

> These values are the maximum QPPS each motor can physically achieve. They will likely be slightly different — that's normal.

> 💡 **Use a slightly conservative number for tuning.** Take 90% of the lower of the two values. Example:
> - M1 max = 8240, M2 max = 8155
> - Lower = 8155 → 90% = **7340** ← this is your `RBCLW_QPPS_MAX`
>
> Why 90%: leaves headroom. If the rover encounters a small uphill or wind, the controller can ask for "100%" and the RoboClaw still has a margin to push harder. If you set QPPS_MAX = the actual peak, the motor saturates and closed-loop control is lost when fully commanded.

### Step 1.5 — Run velocity autotune

1. Click **Velocity Settings** in the left menu.
2. In the **QPSS** field for Motor 1, type the value you wrote down (e.g., `8240` — the actual measured peak, not the 90% number).
3. Click **Tune M1**. The motor will spin back-and-forth for ~30 seconds while RoboClaw figures out the PID.
4. When it stops, the **P, I, D** boxes are filled in automatically.
5. Repeat: enter M2's QPSS, click **Tune M2**.

### Step 1.6 — Test the tuning
Still in Velocity Settings:

1. Move the **Motor 1** slider up and down slowly. Motor should respond quickly with **no chatter, no vibration, no buzzing**.
2. Use the **graph** at the top: set channel 1 to "Motor1 setpoint", channel 2 to "Motor1 velocity". Move the slider — both lines should track each other closely with little gap.
3. If you hear chatter/vibration: lower P and I by 20% each (keep ratio similar) until smooth.
4. If there's a constant gap between setpoint and velocity: increase I by 20%.
5. Repeat for Motor 2.

### Step 1.7 — SAVE settings to the RoboClaw
**This is the step everyone forgets.**

1. Top menu → **Device** → **Write Settings**.
2. RoboClaw blinks. Settings are now in NVRAM and will persist after power cycle.

> If you skip this, your tuning vanishes the moment you unplug USB power.

### Step 1.8 — Disconnect Motion Studio
Close Motion Studio, unplug USB cable from RoboClaw. From now on, the RoboClaw is controlled by the Cube via the TELEM serial port.

---

## ⚙️ Phase 2 — PX4 parameter setup (in QGC)

Cube powered, USB to PC, QGroundControl open.

### Step 2.1 — Set the RoboClaw parameters
QGC → Vehicle Setup → Parameters → search "RBCLW":

```
RBCLW_SER_CFG     = <your TELEM port, e.g. TELEM2>
RBCLW_ADDRESS     = 128         (default, matches RoboClaw default)
RBCLW_COUNTS_REV  = <encoder counts per wheel revolution>
RBCLW_QPPS_MAX    = 7340        ← from Phase 1.4 (the 90% value)
```

> `RBCLW_COUNTS_REV` you find in the encoder datasheet × 4 (quadrature). Example: 300 CPR encoder = 1200 counts/rev.

### Step 2.2 — Set the rover-side parameters (still in QGC)

```
CA_R_REV          = 3           ← MUST stay 3 (bidirectional PWM)
RD_TANK_MODE      = 1           ← keeps your manual paddle control
RO_YAW_RATE_P     = 0.5         ← outer loop, start conservative
RO_YAW_RATE_I     = 0.3
NAV_ACC_RAD       = 0.5
GPS_YAW_OFFSET    = 180.0       (verify by driving north — adjust to 0 if heading shows ~180)
```

### Step 2.3 — Save and reboot
- QGC → Tools (gear icon) → **Reboot Vehicle**

---

## 🚗 Phase 3 — On-ground straight-line verification

Now wheels go BACK on the ground. Rover armed in MANUAL mode first.

### Step 3.1 — MANUAL mode straight-line test
1. Arm in MANUAL.
2. Push both paddles equally forward.
3. Rover should drive **straight** for 5 m.
4. **Drift LEFT?** → right wheel is faster than left → reduce M2 QPPS_MAX, OR see Step 3.3.
5. **Drift RIGHT?** → left wheel is faster than right → reduce M1 QPPS_MAX, OR see Step 3.3.

> Even small differences in tire diameter (~1 mm) cause drift. Mechanical fix first (matched tires, matched pressure if pneumatic), then electrical.

### Step 3.2 — Check for closed-loop "stickiness"
At very low paddle inputs, motor should still try to move. If it deadbands (motor doesn't even twitch until paddle is 20% in), the inner PID may have too much I-clamping. Re-enter Motion Studio, raise I slightly.

### Step 3.3 — If straight-line drift persists (per-motor QPPS_MAX needed)
Right now we have one global `RBCLW_QPPS_MAX`. If your motors differ enough that they can't be matched mechanically, we need to split this into `RBCLW_QPPS_M1_MAX` and `RBCLW_QPPS_M2_MAX`. Tell me and I'll patch it — about 10 lines.

Workaround until then: tune the *slower* motor's QPPS_MAX so it just barely reaches its peak, and accept the faster motor will run slightly under its peak. Drift will minimize but you lose ~5% top speed.

---

## 🎯 Phase 4 — AUTO MISSION verification

### Step 4.1 — Run `mission_square.waypoints`
- Should drive 4 straight legs, each visibly straighter than before.
- Corners should be cleaner spot turns.
- Look at the painted/marked line — uniform width?

### Step 4.2 — Run `mission_half_circle_60.waypoints`
- Arc should look like an arc, not a wobbly polygon.
- The inner-wheel-slow / outer-wheel-fast ratio is now ENFORCED by the closed loop, so circles should be much rounder than before.

### Step 4.3 — Slope test (if you have one)
- Drive across a 5° slope at 0.5 m/s.
- The rover should hold both wheel speeds even on the slope.
- Compare line uniformity to the old open-loop build — this is the most dramatic improvement.

### Step 4.4 — Log review
Pull the `.ulg`. Check these signals in PlotJuggler:

| Signal | What you want to see |
|---|---|
| `actuator_motors.control[0]` (left) | Tracks `actuator_motors.control[1]` (right) on straight legs |
| `vehicle_local_position.body_vx` | Stays near commanded forward speed during slopes |
| `wheel_encoders.wheel_speed[0,1]` | Both wheels at the same value during straights |

---

## 🔁 Phase 5 — Iterate (Mission Planner video method)

Per your existing `tech.md` discipline:

1. Make ONE parameter change (start with `RBCLW_QPPS_MAX`, then per-motor tune).
2. Record a Mission Planner screen video of the test run.
3. Extract frames every 2 s with OpenCV.
4. Crop header (left 22%, top 20%) to read pidachieved/piddesired Min/Max/Mean.
5. Conclude → next change → repeat.

---

## 🚨 Common problems & fixes

| Symptom | Cause | Fix |
|---|---|---|
| Rover doesn't move at all | `RBCLW_QPPS_MAX = 0` | Set the value from Phase 1.4 |
| Rover oscillates / vibrates / buzzes | Inner PID over-aggressive | Motion Studio → Velocity Settings → lower P and I by 20% → Save → reboot |
| Constant speed error (commands 100% but only reaches 80%) | QPPS_MAX too high | Lower QPPS_MAX by 10% |
| Drift one direction in MANUAL | Tire/motor mismatch | Mechanical match first; if persistent, request per-motor split patch |
| Drift only in AUTO, not MANUAL | Outer loop (`RO_YAW_RATE_P`) too low/high | Adjust ±20% in QGC and retest |
| Straight in MANUAL, but wobbly in AUTO at low speed | Inner loop deadband | Increase I in Motion Studio |
| Fine in low speed, oscillates at high speed | Inner P too high for high-speed regime | Slightly lower P |
| Rover suddenly runs away at full speed | Encoder direction wrong! | EMERGENCY DISARM. Re-do Phase 1.3. |

---

## 📚 Source

Procedure compiled and rephrased from BasicMicro's official guide: [Auto Tuning with Motion Studio](https://www.basicmicro.com/Auto-Tuning-with-Motion-Studio_b_51.html). Original content paraphrased for licensing compliance.

---

## ✅ Quick checklist (print this)

```
Phase 1 — RoboClaw bench (wheels OFF ground):
  [ ] 1.1 Motion Studio installed, RoboClaw detected
  [ ] 1.2 Both motors spin forward when slider goes up
  [ ] 1.3 Both encoders count UP when motor spins forward
  [ ] 1.4 Recorded M1_max_QPPS = ____  M2_max_QPPS = ____
  [ ] 1.5 Velocity autotuned both M1 and M2
  [ ] 1.6 Smooth response, graph tracks setpoint
  [ ] 1.7 Device → Write Settings ← CRITICAL

Phase 2 — PX4 params (QGC):
  [ ] 2.1 RBCLW_SER_CFG, ADDRESS, COUNTS_REV, QPPS_MAX set
  [ ] 2.2 CA_R_REV=3, RD_TANK_MODE=1, RO_YAW_RATE_P=0.5
  [ ] 2.3 param save + reboot

Phase 3 — Ground (wheels ON ground):
  [ ] 3.1 MANUAL straight-line drift < 5 cm over 5 m
  [ ] 3.2 No deadband at low paddle input
  [ ] 3.3 If drift > 5 cm, decide: mechanical fix or per-motor patch

Phase 4 — Mission tests:
  [ ] 4.1 Square mission — straight legs, clean corners
  [ ] 4.2 Half-circle — clean arc shape
  [ ] 4.3 Slope test — no slowdown
  [ ] 4.4 Log review — wheel speeds match during straights
```

That's the complete procedure. Start with Phase 1 (bench, wheels off, USB to RoboClaw). The Motion Studio autotune is genuinely the highest-leverage step — get that right and the rest is mostly verification.