# Spray / Marking Servo — Hardware, FCU Params & Endpoint Testing

**Status:** Phase 3 software deployed (commits `eb77785` → `00a11e1`). Hardware wiring and
QGC configuration are the remaining operator steps before field use.
**Plan reference:** `docs/superpowers/plans/2026-06-10-spray-servo-integration.md`

---

## 1. System overview

```
Planner spray_flags ─► /path (flag in pose.position.z)
                          │
        rpp_controller_node (50 Hz)
          flags propagated through corner-smoothing/resampling,
          MARK segment iff flags[i] AND flags[i+1]
                          │
                          ├─► /rpp/debug[39]  (spray_active, telemetry)
                          └─► /spray/active   (Bool, 50 Hz)
                          ▼
        spray_controller_node                ◄── /spray/manual  (server override)
          debounce → edge-fire → 2 Hz re-assert
          watchdog + fail-safes (see §5)
                          │
                          ├─► /mavros/cmd/command  CommandLong 187 (DO_SET_ACTUATOR)
                          ├─► /spray/state         (actual commanded, → server "spraying")
                          └─► /spray/manual_state  (override truth, → server)
                          ▼
        PX4 mixer: Peripheral via Actuator Set 1 (function 301) → AUX pin → driver → solenoid
```

`MAV_CMD_DO_SET_ACTUATOR` (187): `param1` carries the Set-1 value (**+1.0 = ON → MAX
PWM, −1.0 = OFF → MIN PWM**), `param2..6` are `NaN` (leave other sets unchanged),
`param7 = 0` (required index). Verified against the fork's
`FunctionActuatorSet.hpp` / `output_functions.yaml`.

> The `rpp-pipeline` systemd unit runs **`rpp_start.sh`**, not the launch file.
> `spray_controller_node` is started/watchdogged there.

---

## 2. Hardware integration

### 2.1 On/off solenoid (recommended for marking paint)

```
CubeOrangePlus AUX5 (signal) ──► logic-level MOSFET gate / opto relay IN
Solenoid supply (+) ──► solenoid (+)
Solenoid (−) ──► MOSFET drain / relay COM-NO ──► supply (−)
Flyback diode ACROSS the solenoid coil (cathode → +)
FMU GND ◄──── common ground ────► driver GND
```

Rules:
- **Never power the solenoid from the FMU servo rail** — own supply, flyback diode mandatory.
- Common ground between FMU and driver is required for the PWM signal to be read.
- Keep the AUX signal wire short and away from the RoboClaw motor leads (PWM noise).
- Driver must interpret the PWM: use a PWM-switch module (e.g. RC relay, threshold
  ~1500 µs) or an RC-PWM-input MOSFET board. A bare MOSFET does not decode PWM.

### 2.2 Proportional servo valve (alternative)

AUX5 signal → servo signal directly (servo powered from a BEC on the rail, not the FMU's
regulator). ON/OFF map to the two PWM endpoints; intermediate values give partial flow.
Software is identical — only QGC MIN/MAX endpoints differ.

---

## 3. QGC parameter setup (Mac only — never push params from the Jetson)

QGC → Vehicle Setup → **Actuators**:

| Setting | Value | Purpose |
|---|---|---|
| AUX5 function | **Peripheral via Actuator Set 1** (= function 301) | Routes `DO_SET_ACTUATOR param1` to AUX5 |
| AUX5 **MIN** | 1000 µs | Spray **OFF** endpoint (value −1.0) |
| AUX5 **MAX** | 2000 µs | Spray **ON** endpoint (value +1.0) |
| AUX5 **DISARMED** | **1000 µs (OFF)** | **Hardware backstop — disarm always kills spray, every mode** |
| AUX5 **FAILSAFE** | 1000 µs (OFF) | Link-loss failsafe → spray off |

Notes:
- A different AUX pin is fine — only the *function* assignment matters. A different
  Actuator Set (2..6) requires `actuator_set_index` param on `spray_controller_node`.
- Tune MIN/MAX to the actual driver thresholds; invert with the channel REV flag if
  the driver is active-low.
- Verified firmware behaviour: when **disarmed**, the mixer holds DISARMED PWM on all
  channels regardless of commands; when **armed**, `DO_SET_ACTUATOR` drives the pin in
  **any flight mode** (OFFBOARD is only a software-layer gate, see §5).
- Record the chosen pin/set in `~/PX4_DXP/config/` notes.

---

## 4. Endpoint reference & testing

All control endpoints require the `X-Rover-Token` header
(`TOKEN=$(ssh flash@192.168.1.102 cat '~/.rover_token')`). Base URL
`http://192.168.1.102:5001` (or `localhost:5001` on the Jetson).

### 4.1 `GET /api/spray/status`

```bash
curl -s -H "X-Rover-Token: $TOKEN" http://192.168.1.102:5001/api/spray/status
# {"spraying": false, "spray_active_desired": false, "manual_override": false}
```

| Field | Meaning |
|---|---|
| `spraying` | Actual commanded solenoid state (`/spray/state` from the node) |
| `spray_active_desired` | What the RPP MARK logic wants right now (`/rpp/debug[39]`) |
| `manual_override` | A manual test/override is currently holding (node truth, not server guess) |

### 4.2 `POST /api/spray/test`

```bash
# Manual ON for 3 s (default duration), auto-off after
curl -s -X POST -H "X-Rover-Token: $TOKEN" -H "Content-Type: application/json" \
  -d '{"on": true}' http://192.168.1.102:5001/api/spray/test
# {"manual": true, "duration_s": 3.0}

# Explicit duration (clamped to 10 s max)
curl -s ... -d '{"on": true, "duration_s": 2.5}' .../api/spray/test

# Cancel immediately (always allowed, even mid-mission)
curl -s ... -d '{"on": false}' .../api/spray/test
# {"manual": false}
```

| Response | Condition |
|---|---|
| `200 {"manual": true, "duration_s": N}` | Override published; server auto-off scheduled |
| `409` "blocked while a mission is RUNNING" | Manual ON refused during missions |
| `409` "requires an armed FCU" | Disarmed — AUX holds DISARMED PWM anyway |
| `400` | `duration_s` not a positive finite number |
| `503` | ROS bridge not up |

Auto-off layers (all independent): server timer (≤10 s) → node
`manual_override_timeout_s` (10 s hard expiry) → disarm/mode fail-safes → FCU DISARMED PWM.

### 4.3 Telemetry (frontend binding)

`GET /api/telemetry/latest` and the 10 Hz WebSocket stream carry:
- `spraying: bool`
- `marking_state: "marking" | "transit" | "off"` — `off` = not armed or mission not
  RUNNING; `marking` = running + spraying; `transit` = running, spray off.

### 4.4 Bench test procedure (wheels off ground)

1. **FCU-level** (proves QGC config + wiring, bypasses all rover software):
   ```bash
   # ARMED required (any mode). Watch AUX5 with a scope/servo tester/LED.
   ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
     "{command: 187, param1: 1.0, param2: .nan, param3: .nan, param4: .nan, \
       param5: .nan, param6: .nan, param7: 0.0}"     # → MAX (ON)
   # param1: -1.0 → MIN (OFF). Disarm → pin returns to DISARMED PWM.
   ```
2. **Endpoint-level** (proves the full server → node → FCU chain):
   arm, then `POST /api/spray/test {"on": true, "duration_s": 3}` — solenoid ON,
   auto-OFF after 3 s. Poll `/api/spray/status` during the window:
   `spraying=true, manual_override=true`.
3. **Fail-safe checks** (each must drop spray OFF immediately):
   - Disarm mid-test (QGC or RC).
   - `{"on": false}` mid-test.
   - `ssh flash@192.168.1.102 'pkill -f spray_controller_node'` mid-test — watchdog in
     `rpp_start.sh` restarts it; pin falls to OFF (node shutdown sends OFF; restarted
     node never replays the stale override — volatile QoS).
4. **Mission-path check:** load a path with mixed MARK/TRANSIT flags, run it, and
   verify `marking_state` flips at segment boundaries; confirm
   `POST /spray/test {"on": true}` returns 409 while RUNNING.
5. **Latency calibration:** `ros2 bag record /spray/active /spray/state
   /mavros/local_position/pose`, measure boundary-cross → actuation (include the
   production `debounce_samples` setting), feed the result into the planner's
   spray-latency compensation (`apply_spray_latency_compensation`). Re-measure after
   changing debounce or mission speed.

---

## 5. Safety model (layered, top wins)

| Layer | Mechanism |
|---|---|
| **Firmware** | DISARMED/FAILSAFE PWM = OFF — disarm kills spray in every mode, no software needed |
| **Node fail-safes** | Disarm or mode ≠ OFFBOARD (`require_offboard`, default true) → force OFF and **clear any manual override**; node shutdown sends a final OFF |
| **Node watchdogs** | `/spray/active` stale > `active_timeout_s` (0.5 s) → auto desire OFF (manual has its own clock); manual expiry `manual_override_timeout_s` (10 s) — an override can never latch |
| **Server gates** | Manual ON refused while mission RUNNING or disarmed; duration clamped ≤ 10 s with auto-off task |
| **Data fail-safe** | Unknown/legacy paths (no z-flags) read as all-OFF at the controller; flag length mismatches force OFF |

`spray_controller_node` parameters (set via `rpp_start.sh` args if needed):
`actuator_set_index` (1), `on_value` (1.0), `off_value` (−1.0), `debounce_samples` (3),
`reassert_hz` (2.0), `require_offboard` (true), `active_timeout_s` (0.5),
`manual_override_timeout_s` (10.0), `command_service` (`/mavros/cmd/command`).

---

## 6. Troubleshooting

| Symptom | Check |
|---|---|
| Pin never moves | Armed? AUX function = 301 in QGC? `pgrep -f spray_controller_node` on Jetson? `journalctl -u rpp-pipeline | grep spray` |
| Moves on `ros2 service call` but not via endpoint | Mode gate: endpoint path enforces OFFBOARD unless `require_offboard:=false`; check node log "manual spray ON rejected" |
| ON but drops after ~0.5 s during a mission | `/spray/active` stale — is `rpp_controller_node` alive? |
| ON but drops after 10 s on bench | Expected — node-side manual expiry; re-issue the test |
| `manual_override` stays false after POST 200 | `/spray/manual_state` not reaching server — check both nodes share `ROS_DOMAIN_ID=0` |
| Spray inverted | Swap MIN/MAX in QGC or set `on_value`/`off_value` to ∓1.0 |
