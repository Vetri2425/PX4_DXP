# Virtual Joystick — Second-Stage Architecture Audit

**Date:** 2026-06-25  
**Status tags:** CONFIRMED (traced to exact code), INFERRED (logical derivation from code), PROPOSED (design decision), UNRESOLVED (requires Jetson verification)  
**Hardware:** NO-GO until Jetson bench + controlled field gates pass.

---

## 1. Executive Verdict

### Selected production architecture

**MAVLink `MANUAL_CONTROL` via mode switch to MANUAL.**  
Body-frame semantics are exact. No dual-publisher conflict. No world-frame rotation required. All joystick behaviors — forward, reverse, arc steering, stationary tank-turn, dead-man stop — map naturally.

### Fallback

If the MAVROS2 `manual_control` plugin is confirmed absent: send MAVLink `MANUAL_CONTROL` frames via a pymavlink UDP gateway node connecting to the MAVROS GCS bridge (localhost:14550). Same MAVLink frame, different delivery socket.

### Rejected architectures

**OFFBOARD velocity mode for joystick — REJECTED.** Two fatal flaws confirmed by code:

1. `DifferentialVelControl::generateVelocitySetpoint()` (overlay, lines 114–154) freezes heading at `|v| < 0.01 m/s` (`ZERO_VEL_THRESHOLD`). Stationary turns are structurally impossible.
2. At `RD_TRANS_DRV_TRN` (default 45°), the state machine enters `SPOT_TURNING`: sets speed to zero, pivots first, then drives. Any steering input > 45° off current heading during motion stops the rover and pivots instead of arcing. This is not joystick semantics.

The previous audit's `/rpp/velocity_ned` dual-publisher design is also rejected. A global-frame velocity topic owned by two publishers depends on message timing, not a mux.

### GO / CONDITIONAL GO / NO-GO

**CONDITIONAL GO** for software implementation.  
**NO-GO** for hardware until all gates in Section 11 pass.

### Remaining blockers before implementation

1. MAVROS2 `manual_control` plugin availability must be confirmed on Jetson (`ros2 topic list | grep manual_control`). If absent, pymavlink gateway is the delivery path.
2. `COM_RC_IN_MODE = 2` is confirmed in all stored param files — no change needed.
3. Server watchdog rate must be tighter than 500 ms to guarantee no RC_LOSS gap (see Section 9).
4. SITL test with MAVLink MANUAL_CONTROL required before first hardware session.

---

## 2. MAVLink MANUAL_CONTROL Evidence

### 2a. Handler in PX4 firmware

**CONFIRMED** — `PX4-Autopilot/src/modules/mavlink/mavlink_receiver.cpp`, lines 2132–2179.

```
void MavlinkReceiver::handle_message_manual_control(mavlink_message_t *msg)
```

Full field mapping (code-confirmed):

| MAVLink field | Range check | PX4 internal field | Normalised range |
|---|---|---|---|
| `x` | `[-1000, 1000]` | `manual_control_setpoint.pitch` | `x / 1000.f` → `[-1.0, 1.0]` |
| `y` | `[-1000, 1000]` | `manual_control_setpoint.roll` | `y / 1000.f` → `[-1.0, 1.0]` |
| `z` | `[0, 1000]` | `manual_control_setpoint.throttle` | `(z / 500.f) - 1.f` → `[-1.0, 1.0]` |
| `r` | `[-1000, 1000]` | `manual_control_setpoint.yaw` | `r / 1000.f` → `[-1.0, 1.0]` |
| `buttons` | — | `.buttons` | pass-through |
| `aux1..6` | `[-1000, 1000]`, gated by `enabled_extensions` bits 2–7 | `.aux1..6` | `/1000.f` |

Out-of-range values are **silently dropped** — the corresponding field is not written (`if (math::isInRange(...))` guards, line 2144).

**Throttle convention — CONFIRMED:**
- z = 0 → throttle = −1.0 (full reverse)
- z = 500 → throttle = 0.0 (neutral/stop)
- z = 1000 → throttle = +1.0 (full forward)

**Steering axis — CONFIRMED:**
`y` → `manual_control_setpoint.roll` → `rover_steering_setpoint.normalized_speed_diff`  
`RoverDifferential::generateSteeringAndThrottleSetpoint()` maps `roll` directly to `normalized_speed_diff` (line 115). Positive `y` = right turn (left wheel faster).

**Timestamp and source — CONFIRMED:**
```cpp
manual_control_setpoint.data_source = SOURCE_MAVLINK_0 + _mavlink.get_instance_id();
manual_control_setpoint.timestamp = manual_control_setpoint.timestamp_sample = hrt_absolute_time();
manual_control_setpoint.valid = true;
_manual_control_input_pub.publish(manual_control_setpoint);
```
Published to `manual_control_input` uORB topic (not `manual_control_setpoint` — the selector does that).

**PX4 freshness timeout — CONFIRMED:**  
`COM_RC_LOSS_T = 0.5 s` (confirmed from `PX4_DXP/PX4_params/12-06-2026/init.params`).  
`ManualControlSelector::isInputValid()` (line 66): `sample_newer_than_timeout = now < input.timestamp_sample + _timeout`. Messages older than 500 ms invalidate the setpoint → RC_LOSS failsafe triggers.

**COM_RC_IN_MODE = 2 — CONFIRMED from stored production params:**  
Value 2 = `RcOrMavlinkWithFallback` (ManualControlSelector.cpp, line 80):
```cpp
case RcInMode::RcOrMavlinkWithFallback:
    match = (input.data_source == _setpoint.data_source) || !_setpoint.valid;
    break;
```
MAVLink `MANUAL_CONTROL` is accepted. No parameter change required.

**Required PX4 navigation mode — CONFIRMED:**  
`RoverDifferential::Run()` (line 81–87): `generateSteeringAndThrottleSetpoint()` is only called when `full_manual_mode_enabled`:
```cpp
const bool full_manual_mode_enabled = flag_control_manual_enabled
    && !flag_control_position_enabled
    && !flag_control_attitude_enabled
    && !flag_control_rates_enabled;
```
This evaluates to true only in MANUAL mode. ACRO, STAB, POSITION set additional flags and do not call `generateSteeringAndThrottleSetpoint()` directly.

**Arming/disarming — CONFIRMED:**  
`ManualControl::processSwitches()` (line 183–184):
```cpp
if (_selector.setpoint().valid
    && _selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_RC) {
```
Switch-based arming executes **only when data_source == SOURCE_RC**. MAVLink MANUAL_CONTROL sets `SOURCE_MAVLINK_0`. **MAVLink source cannot arm or disarm via switch logic.** Explicit `arm_async()` service call is required.

**Stationary turn support — CONFIRMED:**  
`RoverDifferential::computeInverseKinematics()` (line 166–179):
```cpp
return Vector2f(throttle_body_x + speed_diff_normalized,
                throttle_body_x - speed_diff_normalized);
```
With `throttle_body_x = 0`, `speed_diff_normalized = r`: left motor = `+r`, right = `-r`. This is a genuine tank-turn with zero forward motion. Natural stationary pivoting is supported in MANUAL mode.

**Behaviour when messages stop — CONFIRMED via ManualControlSelector:**  
At 500 ms with no new MANUAL_CONTROL: `valid = false`, `_instance = -1`. `ManualControl::processInput()` takes the else branch — resets stick differentials, publishes invalid setpoint. RC_LOSS failsafe triggers based on `COM_RCL_EXCEPT`:
- `COM_RCL_EXCEPT = 4` (bit 2 = OFFBOARD) → OFFBOARD mode is exempt from RC_LOSS
- MANUAL mode is NOT in the exception list → RC_LOSS failsafe will fire in MANUAL

**This is critical**: If the operator is in MANUAL mode with joystick active, a 500 ms gap in MANUAL_CONTROL messages triggers the RC_LOSS failsafe. Server watchdog must prevent this. See Section 9 for timing.

---

## 3. Installed MAVROS Capability

### 3a. MAVROS launch and version

**CONFIRMED from `PX4_DXP/px4_start_service.sh` line 111:**
```bash
ros2 launch mavros node.launch \
    fcu_url:=/dev/serial/by-id/usb-CubePilot_CubeOrange+_0-if00:921600 \
    gcs_url:=udp://:14550@ \
    pluginlists_yaml:=.../px4_pluginlists_rover.yaml \
    config_yaml:=/opt/ros/humble/share/mavros/launch/px4_config.yaml
```
MAVROS2 on ROS2 Humble. GCS bridge: MAVROS listens on UDP port 14550; any MAVLink client connecting here gets full bidirectional MAVLink forwarding.

### 3b. Plugin denylist

**CONFIRMED from `PX4_DXP/px4_pluginlists_rover.yaml`:**  
Neither `manual_control` nor `rc_override` nor `rc_io` appears in the denylist. All three are therefore loaded by default (if present in the MAVROS2 Humble package).

### 3c. MAVROS2 manual_control plugin

**INFERRED from MAVROS2 source and package structure:**  
MAVROS2 (Humble) ships a `manual_control` plugin that:
- Subscribes `/mavros/manual_control/send` (incoming command from companion)
- Publishes `/mavros/manual_control/control` (incoming MANUAL_CONTROL from GCS, not needed here)
- Message type: `mavros_msgs/msg/ManualControl` with fields `x`, `y`, `z`, `r`, `buttons`

The plugin is listed in `mavros_plugins.xml` under standard MAVROS2 installation.

**UNRESOLVED — requires Jetson verification:**
```bash
# On Jetson:
ros2 topic list | grep manual_control
ros2 topic info /mavros/manual_control/send
```
If `/mavros/manual_control/send` does not appear, use pymavlink gateway (Option B below).

### 3d. Delivery options ranked

**Option A (preferred):** MAVROS2 `manual_control` plugin  
Publish `mavros_msgs/ManualControl` to `/mavros/manual_control/send`. MAVROS wraps and sends MAVLink `MANUAL_CONTROL` to FCU.

**Option B (fallback):** pymavlink UDP gateway  
Small Python node on Jetson: `mavutil.mavlink_connection('udpin:0.0.0.0:14551')` listening for joystick commands, sends MANUAL_CONTROL frames via UDP to `localhost:14550` (MAVROS GCS bridge). MAVROS forwards to FCU via serial. No MAVROS2 plugin dependency.

**Option C (not recommended):** MAVROS `rc_io` plugin / `/mavros/rc/override`  
This sends `RC_CHANNELS_OVERRIDE`, NOT `MANUAL_CONTROL`. PX4 processes these via `handle_message_rc_channels_override()` → `input_rc` → `rc_update()` → `manual_control_input` with a source tag that may allow switch-based arming. Semantics differ from physical RC in edge cases. **Do not use unless proven equivalent by code trace of `rc_update` RC_CHANNELS_OVERRIDE handling.**

### 3e. Conclusion

**Conclusion 1 (primary, INFERRED):** MAVROS2 directly supports sending `MANUAL_CONTROL` via `/mavros/manual_control/send`. Subject to Jetson `ros2 topic list` confirmation.  
**Conclusion 4 (fallback, CONFIRMED path):** A small pymavlink gateway is sufficient if the plugin is absent or has message-type issues. `localhost:14550` is accessible on Jetson with zero configuration changes.

---

## 4. Behaviour Comparison

### Candidates

- **Physical RC**: SBUS → rc_update → SOURCE_RC → DifferentialManualMode (full manual)
- **Candidate A — MANUAL_CONTROL**: pymavlink/MAVROS2 → SOURCE_MAVLINK_0 → full_manual_mode (same mixer)
- **Candidate B — OFFBOARD velocity**: TwistToSetpointNode → DifferentialVelControl (NED velocity)

### 4a. Joystick action table

| Action | Physical RC | Candidate A (MANUAL_CONTROL) | Candidate B (OFFBOARD velocity) |
|--------|-------------|-------------------------------|----------------------------------|
| Forward | throttle → throttle_body_x → IK | z→throttle → throttle_body_x → IK **IDENTICAL** | NED velocity derived from throttle×cos(yaw), throttle×sin(yaw) **DIFFERENT** |
| Reverse | negative throttle → IK (CA_R_REV) | IDENTICAL | bearing flipped 180° in DifferentialVelControl (CONFIRMED) — works but via different mechanism |
| Steer left while moving | roll → speed_diff → IK → left wheel slower | IDENTICAL | bearing = atan2(v_north with left offset, v_east) — only small corrections work; >45° triggers SPOT_TURN stop-first **BROKEN for large steer** |
| Steer right while moving | IDENTICAL to left | IDENTICAL | Same broken behavior at large steer angles |
| Steer left while stopped | roll→speed_diff, throttle=0 → left=-r, right=+r (tank turn) | IDENTICAL | |v|<0.01 → heading freeze, no turn **BROKEN** |
| Steer right while stopped | IDENTICAL | IDENTICAL | **BROKEN** |
| Sudden neutral | zero throttle + zero roll → motors stop, SlewRate decel | IDENTICAL (SlewRate applies) | zero NED velocity → heading freeze → decel via SlewRate |
| Wi-Fi loss | No effect (physical RC active) | 500 ms gap → RC_LOSS failsafe fires (MANUAL mode not exempt) | 500 ms OFFBOARD gap → PX4 exits OFFBOARD to failsafe |
| Server process crash | No effect | last MANUAL_CONTROL timestamp expires → RC_LOSS in 500 ms | TwistToSetpointNode stops → OFFBOARD gap → failsafe in 500 ms |
| ROS node crash (joystick node) | No effect | MANUAL_CONTROL stream stops → RC_LOSS in 500 ms | `/rpp/velocity_ned` silent → TwistToSetpointNode streams zeros → OFFBOARD preserved, rover decelerates |
| MAVROS loss | No effect | MAVROS serial link dead → no frame reaches FCU → RC_LOSS in 500 ms | MAVROS dead → setpoint_raw/local not received by FCU → OFFBOARD gap → failsafe |
| PX4 mode change (external) | RC source in MANUAL naturally | Manual control stream still active; if mode goes to OFFBOARD while joystick active, motors controlled by OFFBOARD setpoint — **conflict** | OFFBOARD setpoint remains active; mode change would conflict differently |
| FCU disconnection | Physical RC bypasses Jetson | MAVROS state_recv_time check detects → server watchdog → release joystick | MAVROS state_recv_time check → safety watchdog in main.py → estop |

### 4b. Failure mode comparison summary

| Failure | Candidate A outcome | Candidate B outcome | Better |
|---------|--------------------|--------------------|--------|
| Wi-Fi loss | RC_LOSS → configurable failsafe | OFFBOARD gap → failsafe | A (configurable action) |
| Server crash | RC_LOSS in 500 ms | rover decelerates, OFFBOARD stays live | B (safer decel) |
| Node crash | RC_LOSS in 500 ms | safe decel | B |
| MAVROS loss | RC_LOSS | OFFBOARD gap failsafe | Equal |
| Stationary turn | Native tank turn | **Broken** | **A wins decisively** |
| Arc steering | Direct IK | Broken at >45° | **A wins decisively** |

**Candidate A is selected** because the steering-while-moving and stationary-turn breakages in Candidate B are fundamental to the v1.16.2 DifferentialVelControl state machine — they cannot be fixed from the companion without firmware changes.

Candidate A's weaker failure isolation (server crash → RC_LOSS vs decel) is mitigated by the server-side 300 ms watchdog sending zero commands before the 500 ms PX4 timeout.

---

## 5. Final Control Architecture

### 5a. Mode as the arbiter

The PX4 **navigation mode** is the single arbitration boundary:
- **OFFBOARD** = TwistToSetpointNode + RPP own the control path. Joystick MUST NOT be active.
- **MANUAL** = MANUAL_CONTROL frames own the control path. Missions MUST NOT be started.

No additional mux node is required. The mode transition IS the mux. TwistToSetpointNode continues streaming in the background at 50 Hz in both modes — in MANUAL mode PX4 ignores these frames, but the OFFBOARD heartbeat contract is maintained, so switching back to OFFBOARD is instantaneous.

### 5b. ASCII flow diagram

```
React Native App
    │
    │  Socket.IO (over Wi-Fi)
    │  joystick_acquire {auth, lease_request}
    │  joystick_command {lease_id, seq, deadman, throttle, steering}
    │  joystick_release {lease_id}
    ▼
PX4_DXP/server/sockets/events.py
    │  on_joystick_acquire   → JoystickController.acquire(sid)
    │  on_joystick_command   → JoystickController.handle_cmd(sid, cmd)
    │  on_joystick_release   → JoystickController.release(sid)
    ▼
PX4_DXP/server/joystick_controller.py  (NEW)
    JoystickController
    ├── validate + clamp (throttle ∈ [-1,1], steering ∈ [-1,1])
    ├── replay / sequence / dead-man checks
    ├── 300 ms server watchdog (asyncio task)
    │     → zero command if stale, release if 2 s
    ├── on acquire: call ros_node.set_mode_async("MANUAL")
    │     guard: OffboardController.state must be IDLE/COMPLETED/ABORTED
    ├── on release: send zero → ros_node.set_mode_async("OFFBOARD")
    │     (TwistToSetpointNode maintained heartbeat → instant switch)
    └── on MAVROS disconnect: force release → estop path
    │
    ▼
PX4_DXP/server/manual_control_gateway.py  (NEW ROS2 node or async method)
    ManualControlGateway
    ├── publish_manual_control(throttle, steering)
    │     → publishes mavros_msgs/ManualControl to /mavros/manual_control/send
    │         x = 0 (pitch/forward — not used for differential rover, throttle is z)
    │         y = steering * 1000   (roll → normalized_speed_diff)
    │         z = (throttle + 1) * 500  (throttle → z in [0,1000])
    │         r = 0 (yaw — not used in differential direct mode)
    │     OR (fallback): sends MANUAL_CONTROL via pymavlink UDP to localhost:14550
    ├── 20 Hz keep-alive timer (when joystick active)
    │     → re-publishes last command if no new command within 50 ms
    │     → publishes zero if no command received (JoystickController watchdog
    │        fires first at 300 ms; this 20 Hz ensures PX4 never sees a gap)
    └── safe shutdown: publish zero on node destroy
    │
    ▼
MAVROS2 (px4-dxp.service)
    manual_control plugin → MAVLink MANUAL_CONTROL (sysid=1, compid=1)
    │
    OR
    │
pymavlink UDP gateway → localhost:14550 → MAVROS GCS bridge → FCU serial
    │
    ▼
PX4 mavlink_receiver.cpp:2132
handle_message_manual_control()
    │  x/1000→pitch, y/1000→roll, (z/500-1)→throttle, r/1000→yaw
    │  data_source = SOURCE_MAVLINK_0 + instance_id
    │  valid = true
    ▼
ManualControlSelector (COM_RC_IN_MODE=2: accepts MAVLink)
    │  timeout = COM_RC_LOSS_T = 500 ms
    ▼
manual_control_setpoint uORB
    ▼
RoverDifferential::generateSteeringAndThrottleSetpoint()  (100 Hz)
    │  roll → rover_steering_setpoint.normalized_speed_diff
    │  throttle → rover_throttle_setpoint.throttle_body_x
    ▼
RoverDifferential::generateActuatorSetpoint()
    │  SlewRate (RO_ACCEL_LIM / RO_DECEL_LIM)
    │  computeInverseKinematics(throttle, speed_diff)
    │    left  = throttle + speed_diff
    │    right = throttle - speed_diff
    ▼
actuator_motors uORB → Roboclaw UART (opcodes 35/36)
    ▼
Motors

── Background (always running, both modes) ──────────────────────────────────
RPP + TwistToSetpointNode:
    /rpp/velocity_ned (RPP output) → TwistToSetpointNode → 50 Hz
    → /mavros/setpoint_raw/local (PositionTarget)
    → MAVROS → PX4
    [In MANUAL mode: PX4 ignores these setpoints but OFFBOARD heartbeat
     is maintained → switch to OFFBOARD is instantaneous on mission start]
```

### 5c. Single-publisher invariant

There is only one publisher on `/mavros/manual_control/send` at any time: `ManualControlGateway`. It publishes zeros when idle. The arbitration is the PX4 mode — not a topic-level mux — so the dual-publisher problem does not arise. TwistToSetpointNode publishes on a different MAVROS topic (`setpoint_raw/local`) and there is never a conflict.

---

## 6. Command and Lease Protocol

### 6a. Event schemas

**`joystick_acquire` (client → server)**
```json
{
  "type": "joystick_acquire",
  "auth": "<shared-secret-token>",
  "session_id": "client-generated-uuid-v4",
  "client_monotonic_ms": 1234567
}
```

**Server → client on success:**
```json
{
  "type": "joystick_acquired",
  "lease_id": "server-uuid-v4",
  "lease_expires_ms": 30000,
  "max_throttle": 0.35,
  "max_steering": 1.0,
  "command_rate_hz": 10,
  "server_monotonic_ms": 1234600
}
```

**`joystick_command` (client → server, 10 Hz nominal)**
```json
{
  "type": "joystick_command",
  "auth": "<token>",
  "session_id": "client-uuid",
  "lease_id": "server-uuid",
  "sequence": 145,
  "client_monotonic_ms": 3892231,
  "deadman": true,
  "throttle": 0.35,
  "steering": -0.20
}
```

**`joystick_release` (client → server)**
```json
{
  "type": "joystick_release",
  "auth": "<token>",
  "session_id": "client-uuid",
  "lease_id": "server-uuid"
}
```

**`joystick_state` (server → all clients, in telemetry tick)**
```json
{
  "joystick_active": true,
  "joystick_lease_id": "server-uuid",
  "joystick_session_id": "client-uuid",
  "joystick_last_cmd_age_ms": 45,
  "joystick_throttle": 0.35,
  "joystick_steering": -0.20
}
```

**Rejection/error events (server → requesting client)**
```json
{
  "type": "joystick_error",
  "code": "mission_active | not_owner | lease_expired | stale_timestamp | 
           out_of_order | replay | nan_value | out_of_range | auth_failed | 
           mode_unavailable | not_armed",
  "message": "<human-readable>"
}
```

### 6b. Validation rules (server-side, applied to every `joystick_command`)

| Field | Validation | Rejection code |
|-------|-----------|----------------|
| `auth` | `secrets.compare_digest()` against token file | `auth_failed` |
| `session_id` | must match current owner session_id | `not_owner` |
| `lease_id` | must match current lease_id | `not_owner` |
| `sequence` | must be > last accepted sequence (per session) | `out_of_order` or `replay` |
| `client_monotonic_ms` | must be ≥ last accepted client_monotonic_ms | `replay` |
| `client_monotonic_ms` | must be ≤ server_receive_time + 2000 ms | `stale_timestamp` |
| `deadman` | must be `true` | zero-command override (not hard reject — treat as throttle=0, steering=0) |
| `throttle` | `math.isfinite(v)` and `|v| ≤ 1.0` | `nan_value` or `out_of_range` |
| `steering` | `math.isfinite(v)` and `|v| ≤ 1.0` | `nan_value` or `out_of_range` |
| Command rate | reject if < 80 ms since last accepted (server-side rate cap: 12.5 Hz max) | `rate_exceeded` |

**Dead-man rule:** If `deadman == false` (or field absent), treat as zero command and do NOT reject. Allows intentional stop via dead-man release. Log the event.

**Replay protection:** Both `sequence` (monotonic counter) and `client_monotonic_ms` (client clock) must advance. Both checks required — client clock alone can repeat across reconnects; sequence alone can be set to max.

**Reconnect behaviour:** New `joystick_acquire` issues a new `lease_id`. Old lease_id commands rejected with `not_owner`. Reconnecting client within the 2 s watchdog window still gets `not_owner` on commands because lease_id changed — it must re-acquire.

**Lease expiry (PROPOSED, 30 s):** Lease expires if no valid command received for 30 s even if watchdog hasn't fired (belt-and-suspenders). Server sends `joystick_released` with code `lease_expired`.

---

## 7. Atomic Mission/Joystick Arbitration

### 7a. Existing lock structure

**CONFIRMED from code:**

1. `OffboardController._lifecycle_lock()` — `asyncio.Lock()`, lazy-init, one per server process. Guards all lifecycle state transitions: `start_async`, `stop_async`, `abort_async`, `clear_mission_async`. **This is the single authoritative lifecycle lock.**

2. `mission_loading._load_lock` — module-level `asyncio.Lock()`. Guards path loading (IO + state validation). Separate from lifecycle lock. Called inside `load_path_for_controller()`.

3. No Point/Dash/Survey mission evidence of a separate lifecycle lock — `OffboardController.start_async()` is the entry point for all mission modes (line 337 onwards, same `_lifecycle_lock()`).

**INFERRED:** Socket.IO and REST routes both call into `start_async()` which acquires `_lifecycle_lock()`. They share the same boundary.

### 7b. Proposed joystick lock

**PROPOSED:** Add a module-level (or main.py-scoped) `asyncio.Lock` named `_control_mode_lock` that serialises any operation that changes the PX4 navigation mode. Both `JoystickController.acquire()` and `OffboardController.start_async()` must hold this lock while performing the mode transition.

```python
# server/control_arbitration.py  (new file)
import asyncio

_control_mode_lock: asyncio.Lock | None = None

def get_control_mode_lock() -> asyncio.Lock:
    global _control_mode_lock
    if _control_mode_lock is None:
        _control_mode_lock = asyncio.Lock()
    return _control_mode_lock
```

**Lock acquisition order (must be consistent to avoid deadlock):**
1. Acquire `_control_mode_lock` first (guards mode transitions)
2. Acquire `_lifecycle_lock()` second (guards controller state)
3. Never hold both in reverse order

For joystick acquire:
```python
async with get_control_mode_lock():
    if offboard_ctrl.state in MISSION_ACTIVE_STATES:
        raise JoystickAcquireConflict("mission_active")
    joystick_ctrl.set_owner(sid, lease_id)
    await ros_node.set_mode_async("MANUAL")
```

For mission start (extend OffboardController.start_async):
```python
async with self._lifecycle_lock():            # existing
    async with get_control_mode_lock():       # NEW — inner
        if joystick_ctrl.is_active:
            return False, "joystick_active"
        # ... existing mode switch to OFFBOARD
```

### 7c. Concurrent case coverage

| Concurrent scenario | Outcome |
|---------------------|---------|
| mission_start + joystick_acquire at same time | First to acquire `_control_mode_lock` wins; second waits, then checks state and rejects |
| joystick_release + mission_start at same time | Release clears owner and switches OFFBOARD; mission_start then succeeds |
| mission_abort during joystick_acquire | abort holds `_lifecycle_lock` but not `_control_mode_lock`; joystick_acquire acquires `_control_mode_lock` then checks `offboard_ctrl.state` — ABORTED → allowed |
| socket disconnect during mode transition | `on_disconnect` checks `owner_sid == disconnected_sid`; if mid-acquire, the acquire coroutine checks `owner_sid` at end; disconnect sets owner=None; acquire self-cancels (see note) |
| server restart with PX4 still armed in MANUAL | On startup: `JoystickController` init state = INACTIVE; server broadcasts `joystick_released`; client must re-acquire; PX4 in MANUAL with zero commands until joystick reconnects |
| reconnecting client with new SID | New SID → new `joystick_acquire` → new `lease_id`; old lease_id rejected; clean handover |

**Note on mid-acquire disconnect:** The `acquire()` coroutine must check that the requesting SID still matches after the async `set_mode_async()` call returns (since the socket may have disconnected while the mode switch was in flight). If mismatch, call `release()` immediately.

### 7d. State machine

```
                    joystick_acquire
      ┌─────────────────────────────────────────────┐
      │  (guard: ctrl.state not in MISSION_ACTIVE)  │
      ▼                                             │
IDLE ──────────────────────────────────────────► JOYSTICK_ACTIVE
  │                                                  │
  │  mission_start                                   │  joystick_release
  │  (guard: joystick_ctrl.is_active == False)       │  socket disconnect
  ▼                                                  │  watchdog 2 s
MISSION_ACTIVE ◄─────────────────────────────────────┘
  │
  │  mission_stop / mission_abort / auto-complete
  ▼
IDLE

INVARIANT: MISSION_ACTIVE and JOYSTICK_ACTIVE are mutually exclusive.
           PX4 mode in OFFBOARD ↔ MISSION_ACTIVE or IDLE-after-mission.
           PX4 mode in MANUAL   ↔ JOYSTICK_ACTIVE.
```

---

## 8. Watchdog and Failsafe Design

### 8a. Layer table

| Layer | Component | Trigger | Action | Soft/Hard |
|-------|-----------|---------|--------|-----------|
| Frontend | React Native | Dead-man button released | Emit zero `joystick_command` + `joystick_release` immediately | Soft stop |
| Frontend | React Native | App goes to background | `AppState` change → emit `joystick_release` | Soft stop |
| Frontend | React Native | Socket disconnect | Re-connect within 5 s → re-acquire; else: server watchdog fires | Reconnect |
| Frontend | React Native | No joystick_acquired ack | Retry `joystick_acquire`; do not send commands | — |
| Server | JoystickController._watchdog | Last valid cmd age > 300 ms | Publish zero command (throttle=0, steering=0) | Soft stop |
| Server | JoystickController._watchdog | Last valid cmd age > 2000 ms | `release_control()` → mode OFFBOARD | Release lease |
| Server | on_disconnect | Owner SID disconnects | `release_control()` immediately | Release lease |
| Server | main.py safety loop | joystick active + MAVROS connected=False > 1 s | `estop_async()` | Hard E-stop |
| Server | main.py safety loop | joystick active + FCU not connected > 1 s | `release_control()` + `estop_async()` | Hard E-stop |
| ROS gateway | ManualControlGateway | No new command within 50 ms (20 Hz keep-alive) | Re-publish last command | — |
| ROS gateway | ManualControlGateway | Node shutdown/SIGTERM | Publish zero once | Soft stop |
| PX4 | ManualControlSelector | No valid MANUAL_CONTROL for > COM_RC_LOSS_T (500 ms) | RC_LOSS failsafe (MANUAL mode not exempt) | PX4 failsafe |
| PX4 | Commander | RC_LOSS in MANUAL | Default failsafe action (HOLD or LAND, depending on COM_RCL_ACT) | Configurable |
| PX4 | DifferentialVelControl / SlewRate | Throttle drops to zero | Motor power ramps down via RO_DECEL_LIM | Hardware decel |

### 8b. Stop gradations

| Event | Stop type | Mode after | Armed? |
|-------|-----------|-----------|--------|
| Dead-man release | Soft stop: zero command | MANUAL → OFFBOARD | Yes |
| Joystick release (explicit) | Soft stop: zero command, mode switch | OFFBOARD | Yes |
| Watchdog timeout (300 ms) | Soft stop: zero command | MANUAL (still held) | Yes |
| Watchdog release (2000 ms) | Mode release: switch to OFFBOARD | OFFBOARD | Yes |
| Server MAVROS watchdog | Hard E-stop: stop_path + MANUAL + disarm | MANUAL | No |
| E-stop button pressed | Hard E-stop (existing EmergencyHandler) | MANUAL | No |
| PX4 RC_LOSS | PX4-controlled failsafe action | depends on COM_RCL_ACT | configurable |

**Do not disarm for brief network jitter.** The server watchdog at 300 ms sends zero commands; only the 2000 ms release switches mode. MAVROS loss and FCU disconnect watchdog trigger estop (disarm) because motor safety requires it. Network jitter alone does not disarm.

### 8c. E-stop path (unchanged, CONFIRMED)

`EmergencyHandler.estop_async()` (existing, confirmed working from CLAUDE.md):
1. `publish_stop_path()` → RPP zero vel
2. `set_mode_async("MANUAL")` → exit OFFBOARD
3. `arm_async(False)` → disarm
4. `controller.state = ABORTED`

When called from joystick path, also call `joystick_ctrl.force_release()` before step 3.

---

## 9. Timing Validation

### 9a. Confirmed timings (from code and stored params)

| Parameter | Value | Source |
|-----------|-------|--------|
| COM_RC_LOSS_T | 500 ms | PX4_params/12-06-2026/init.params (CONFIRMED) |
| COM_RC_IN_MODE | 2 (RcOrMavlinkWithFallback) | Same file (CONFIRMED) |
| COM_RCL_EXCEPT | 4 (OFFBOARD exempt only) | Same file (CONFIRMED) |
| TwistToSetpointNode rate | 50 Hz (20 ms period) | src/twist_to_setpoint_node.py (CONFIRMED) |
| TwistToSetpointNode input staleness | 200 ms | `input_max_age_s=0.2` (CONFIRMED) |
| MAVROS state timeout (RosBridgeNode) | 2000 ms | `_MAVROS_STATE_TIMEOUT_S = 2.0` (CONFIRMED) |
| OFFBOARD gap before failsafe | 500 ms | PX4 documentation + OFFBOARD contract comment in twist_to_setpoint_node.py (CONFIRMED) |
| PX4 ManualControl scheduler interval | 200 ms | `ScheduleDelayed(200_ms)` in ManualControl.cpp:84 (CONFIRMED — timeout detection heartbeat) |

### 9b. Proposed initial timing values

| Parameter | Proposed value | Rationale |
|-----------|---------------|-----------|
| Frontend command rate | 10 Hz (100 ms period) | Sufficient margin over 500 ms PX4 timeout; low enough for React Native scheduler jitter |
| Server rate cap | 12.5 Hz max (reject if < 80 ms since last) | Prevent spam; still well under 500 ms timeout |
| Server stale command → zero | 300 ms | 200 ms margin before 500 ms PX4 timeout; absorbs Wi-Fi jitter + RN scheduler |
| Server stale command → release | 2000 ms | Long enough for brief Wi-Fi dropout without full release |
| ManualControlGateway keep-alive | 20 Hz (50 ms period) | Re-publishes last command every 50 ms; ensures MAVROS/PX4 never see a gap |
| ManualControlGateway zero-flush on release | 1 frame | Immediate zero before mode switch |
| Mode switch timeout (MANUAL→OFFBOARD) | 3 s (existing set_mode_async timeout) | From existing OffboardController flow |
| Lease expiry (no command) | 30 s | Belt-and-suspenders over 2 s watchdog |
| Joystick acquire mode-switch max wait | 3 s | Beyond this → return error, do not proceed |

### 9c. Requires Jetson bench measurement

| Parameter | Why measure | Expected range |
|-----------|------------|----------------|
| Socket.IO round-trip latency (Wi-Fi) | Confirm < 100 ms | 5–50 ms typical |
| MAVROS manual_control topic latency (Jetson → FCU) | Confirm < 50 ms | 5–20 ms |
| Mode switch latency (set_mode_async) | Confirm < 500 ms | 100–300 ms typical |
| React Native command emit jitter | Set margin for 300 ms watchdog | < 50 ms typical |

---

## 10. Test Plan

### T1 — Unit: JoystickController state machine (no hardware)

| ID | Test | Expected |
|----|------|----------|
| T1.1 | acquire() when offboard_ctrl.state=RUNNING | Rejected: `mission_active` |
| T1.2 | acquire() when state=IDLE | Accepted; lease_id issued; mode switch called |
| T1.3 | handle_cmd with wrong lease_id | Rejected: `not_owner` |
| T1.4 | handle_cmd with old sequence number | Rejected: `out_of_order` |
| T1.5 | handle_cmd with replay (same seq twice) | Second rejected: `replay` |
| T1.6 | handle_cmd with throttle=2.0 | Rejected: `out_of_range` |
| T1.7 | handle_cmd with throttle=NaN | Rejected: `nan_value` |
| T1.8 | handle_cmd with deadman=false | Accepted; zero command forwarded |
| T1.9 | No cmd for 350 ms | Watchdog sends zero command; lease retained |
| T1.10 | No cmd for 2100 ms | Watchdog releases; mode switch to OFFBOARD called |
| T1.11 | mission_start while joystick active | Rejected by control_mode_lock check |
| T1.12 | joystick_acquire while mission RUNNING | Rejected |
| T1.13 | Socket disconnect (owner SID) | Immediate release; mode switch called |
| T1.14 | Socket disconnect (non-owner SID) | No effect on joystick state |
| T1.15 | estop during joystick active | estop called; joystick released; mode MANUAL; disarmed |

### T2 — Unit: ManualControlGateway MAVLink field encoding

| ID | Test | Expected on MAVLink MANUAL_CONTROL |
|----|------|-------------------------------------|
| T2.1 | throttle=1.0, steering=0.0 | z=1000, y=0 |
| T2.2 | throttle=-1.0, steering=0.0 | z=0, y=0 |
| T2.3 | throttle=0.0, steering=0.0 | z=500, y=0 |
| T2.4 | throttle=0.5, steering=1.0 | z=750, y=1000 |
| T2.5 | throttle=0.0, steering=-0.5 | z=500, y=-500 |
| T2.6 | keep-alive timer fires (no new cmd) | Re-publishes last frame; no gap > 50 ms |
| T2.7 | Node shutdown | Zero frame published (z=500, y=0) |

### T3 — Unit: Forward/reverse semantics (CONFIRMED code trace)

| ID | Test | Expected PX4 behavior |
|----|------|----------------------|
| T3.1 | throttle=0.5, steering=0 → z=750, y=0 | manual_control.throttle=0.5 → throttle_body_x=0.5 → IK(0.5,0)=(0.5,0.5) → both motors forward |
| T3.2 | throttle=-0.5, steering=0 → z=250, y=0 | throttle_body_x=-0.5 → IK(-0.5,0)=(-0.5,-0.5) → both motors reverse |
| T3.3 | throttle=0, steering=0.5 → z=500, y=500 | throttle=0, speed_diff=0.5 → IK(0,0.5)=(0.5,-0.5) → left forward, right reverse → right turn |
| T3.4 | throttle=0.5, steering=0.5 → z=750, y=500 | IK(0.5,0.5): max_cmd=1.0, no clamping; left=1.0, right=0.0 → forward-right arc |
| T3.5 | throttle=0.5, steering=1.0 → z=750, y=1000 | max_cmd=1.5 > 1.0; excess=0.5; throttle adjusted → prioritize yaw |

### T4 — Integration: Socket.IO → PX4 mode (Jetson, rover disarmed, no motion)

| ID | Test | Expected |
|----|------|----------|
| T4.1 | joystick_acquire without arm | Rejected: `not_armed` |
| T4.2 | arm + joystick_acquire | Mode switches to MANUAL; joystick_acquired event |
| T4.3 | joystick_command with valid cmd | PX4 telemetry shows MANUAL mode; `ros2 topic echo /mavros/state --once` shows mode=Manual |
| T4.4 | Confirm /mavros/manual_control/send exists | `ros2 topic list | grep manual_control` → topic present |
| T4.5 | joystick_release | Mode switches to OFFBOARD (TwistToSetpointNode maintaining heartbeat); joystick_released event |
| T4.6 | mission_start after joystick_release | Mission starts in OFFBOARD cleanly |
| T4.7 | mission_start during joystick active | Rejected: `joystick_active` |
| T4.8 | joystick_acquire during mission running | Rejected: `mission_active` |
| T4.9 | No command for 2100 ms | Mode automatically switches to OFFBOARD; joystick_released event |
| T4.10 | Socket disconnect (owner) | Mode switches to OFFBOARD; joystick_released |
| T4.11 | E-stop during joystick | Disarmed; mode MANUAL; joystick released |

### T5 — PX4 mode + motor semantics (Jetson, wheels blocked)

| ID | Test | Expected |
|----|------|----------|
| T5.1 | Forward (throttle=0.3) | `/mavros/setpoint_raw/local` topic NOT used; PX4 manual control path active; speed_m_s in telemetry responds |
| T5.2 | Reverse (throttle=-0.3) | Telemetry shows negative speed or reverse motion |
| T5.3 | Stationary turn (throttle=0, steering=0.5) | No heading freeze (MANUAL mode); wheels turning in opposite directions |
| T5.4 | 50 ms latency after cmd stop | ManualControlGateway keep-alive re-publishes last cmd within 50 ms |
| T5.5 | Server watchdog zero (300 ms no cmd) | PX4 receives zero throttle within 300 ms; motors stop |

### T6 — SITL validation (before any hardware)

Run PX4 SITL (`make px4_sitl gz_rover_diff` or equivalent) with MAVLink MANUAL_CONTROL fed from the server stack.

| ID | Test | Expected |
|----|------|----------|
| T6.1 | Full forward-reverse sequence | SITL rover moves forward/reverse |
| T6.2 | Arc steering while moving | SITL rover arcs (verify no SPOT_TURN interruption in MANUAL mode) |
| T6.3 | Tank turn (stationary) | SITL rover pivots without forward motion |
| T6.4 | Wi-Fi loss simulation (drop Socket.IO) | Server watchdog 300 ms → zero; 2000 ms → OFFBOARD |
| T6.5 | Mission → joystick → mission handover | Clean mode switch sequence verified in SITL |
| T6.6 | estop mid-drive | SITL rover stops and disarms |

### T7 — Jetson bench (wheels off ground or blocked)

Run T4 + T5 sequence with rover fully assembled but not able to roll. Verify MAVLink timing on hardware FCU.

| ID | Test | Expected |
|----|------|----------|
| T7.1 | Measure Socket.IO latency (10 samples) | < 100 ms |
| T7.2 | Measure mode switch latency (MANUAL→OFFBOARD) | < 500 ms |
| T7.3 | Confirm no RC_LOSS fires during 60 s joystick session | PX4 logs show no RC_LOSS event |
| T7.4 | Confirm RC_LOSS fires at 550 ms gap (induced) | PX4 log shows RC_LOSS; verify failsafe action |

### T8 — Controlled field validation (≤ 0.2 m/s, spotter)

| ID | Test | Expected |
|----|------|----------|
| T8.1 | Forward at throttle=0.3 | Rover moves forward, decelerates on release |
| T8.2 | Reverse at throttle=-0.3 | Clean reverse |
| T8.3 | Arc left/right while moving | Natural arc turn, no abrupt stop-pivot |
| T8.4 | Stationary tank turn | Pivots in place |
| T8.5 | Dead-man release mid-motion | Stops within 1 s |
| T8.6 | Socket disconnect at speed | Watchdog zero; stop within 300 ms + decel |
| T8.7 | Joystick → stop → mission → return → joystick | Full handover cycle |
| T8.8 | E-stop button | Stop + disarm < 3 s |

---

## 11. File-by-File Implementation Plan

### File 1: `server/control_arbitration.py` — NEW

```
Purpose:    Module-level asyncio.Lock for mode transitions
Classes:    get_control_mode_lock() → asyncio.Lock
            MISSION_ACTIVE_STATES = {ARMING, SWITCHING_OFFBOARD, RUNNING, STOPPING}
Locking:    Acquired FIRST (outer); lifecycle lock acquired second (inner)
Tests:      T1.11, T1.12
```

### File 2: `server/joystick_controller.py` — NEW

```
Purpose:    Lease lifecycle, validation, server-side watchdog, zero-on-release
Classes:    JoystickController
Interface:
    async acquire(sid, lease_id, session_id) → (bool, str)
        Checks: armed, not mission_active (control_mode_lock)
        Calls:  ros_node.set_mode_async("MANUAL")
        Sets:   _owner_sid, _lease_id, _session_id, _last_seq, starts watchdog
    
    def handle_cmd(sid, cmd: JoystickCmd) → (bool, str)
        Validates: ownership, seq, replay, deadman, ranges
        Calls:  gateway.publish_manual_control(throttle, steering)
        Updates: _last_cmd_time, _last_seq, _last_mono
    
    async release(sid=None, reason="explicit")
        Sends zero, calls ros_node.set_mode_async("OFFBOARD")
        Cancels watchdog, clears owner

    async force_release()
        Same as release(None), ignores owner check

    _watchdog_loop()  async task
        Every 100 ms: check _last_cmd_time
        > 300 ms: gateway.publish_zero()
        > 2000 ms: await release(reason="watchdog_timeout")

    @property is_active / owner_sid / lease_id

Locking:    _control_mode_lock acquired in acquire() and release()
Tests:      T1.1–T1.15
```

### File 3: `server/manual_control_gateway.py` — NEW

```
Purpose:    Single publisher to /mavros/manual_control/send; 20 Hz keep-alive
Classes:    ManualControlGateway (ROS2 Node mixin or standalone node)
Interface:
    publish_manual_control(throttle: float, steering: float)
        Converts: z = int((throttle + 1.0) * 500), y = int(steering * 1000)
        Publishes: mavros_msgs/ManualControl to /mavros/manual_control/send
        Stores: _last_throttle, _last_steering
    
    publish_zero()
        Calls publish_manual_control(0.0, 0.0)
    
    _keepalive_cb()  (20 Hz ROS2 timer, active only when joystick_active=True)
        Re-publishes _last_throttle, _last_steering

    on_destroy() / shutdown_hook()
        Publishes zero frame

Fallback:  If /mavros/manual_control/send unavailable, pymavlink UDP to localhost:14550
           __init__ probes topic; if absent after 5 s, falls back to pymavlink path
           pymavlink target: udp:localhost:14550

Locking:    No lock needed — single caller (JoystickController.handle_cmd is single-threaded
            in asyncio; gateway publish is synchronous ROS2 call)
Tests:      T2.1–T2.7
```

### File 4: `server/ros_node.py` — EXTEND (minimal)

```
Changes:
  - Import mavros_msgs/ManualControl in optional block (alongside State, CommandBool)
  - No new publishers needed — ManualControlGateway owns that publisher
  - No changes to existing publishers/subscribers
```

### File 5: `server/sockets/events.py` — EXTEND register_handlers()

```
Add three handlers inside register_handlers(sio):
    on_joystick_acquire(sid, data)
        auth check → joystick_ctrl.acquire(sid, ...)
        emit joystick_acquired or joystick_error

    on_joystick_command(sid, data)
        auth check → validate schema → joystick_ctrl.handle_cmd(sid, cmd)
        emit joystick_error on rejection only (do not ack every cmd at 10 Hz)

    on_joystick_release(sid, data)
        auth check → joystick_ctrl.release(sid)
        emit joystick_released

Extend on_disconnect(sid):
    if joystick_ctrl and joystick_ctrl.owner_sid == sid:
        await joystick_ctrl.release(sid, reason="disconnect")
```

### File 6: `server/main.py` — EXTEND

```
Changes:
  - Import JoystickController, ManualControlGateway
  - Add globals: joystick_ctrl, manual_gateway
  - Lifespan: init ManualControlGateway, then JoystickController(ros_node, offboard_ctrl, manual_gateway)
  - Safety watchdog (_telemetry_loop):
      if joystick_ctrl.is_active and not telemetry.connected:
          await joystick_ctrl.force_release()
          await emergency_handler.estop_async()
  - Telemetry payload: add joystick_active, joystick_last_cmd_age_ms fields
```

### File 7: `server/models.py` — EXTEND

```
Add:
  class JoystickCmd(BaseModel):
      type: Literal["joystick_command"]
      auth: str
      session_id: str
      lease_id: str
      sequence: int
      client_monotonic_ms: int
      deadman: bool = False
      throttle: float = Field(..., ge=-1.0, le=1.0)
      steering: float = Field(..., ge=-1.0, le=1.0)

  class JoystickAcquireRequest(BaseModel):
      auth: str
      session_id: str
      client_monotonic_ms: int
```

### File 8: `server/offboard_controller.py` — EXTEND start_async (one guard)

```
Change:
    Inside start_async(), after _lifecycle_lock() is acquired, add:
        from control_arbitration import get_control_mode_lock
        async with get_control_mode_lock():
            if joystick_ctrl is not None and joystick_ctrl.is_active:
                return False, "joystick_active"
            # ... existing OFFBOARD sequence
```

---

## 12. Final Implementation Gates

### Gate 1 — Software implementation
- [ ] All 8 files above created/extended
- [ ] Unit tests T1.1–T1.15, T2.1–T2.7 pass (`python -m pytest server/test_joystick*.py`)
- [ ] No new asyncio lock ordering violations (review: always control_mode_lock → lifecycle_lock)
- [ ] Auth validation tests: wrong token rejected, stale timestamp rejected, replay rejected

### Gate 2 — MAVROS topic verification (Jetson, service running)
- [ ] `ros2 topic list | grep manual_control` confirms `/mavros/manual_control/send` present
- [ ] `ros2 topic echo /mavros/manual_control/send` shows messages when gateway publishes
- [ ] If absent: pymavlink fallback confirmed working via `tcpdump` on UDP port 14550

### Gate 3 — SITL (T6.1–T6.6 all pass)
- [ ] PX4 SITL with rover model accepts MAVLink MANUAL_CONTROL from server stack
- [ ] Forward/reverse/stationary-turn confirmed in simulation
- [ ] Wi-Fi loss simulation: zero command within 300 ms
- [ ] Mission→joystick→mission handover: clean mode switches

### Gate 4 — Jetson bench (wheels blocked, rover not rolling)
- [ ] T4.1–T4.11 all pass (Socket.IO → mode switch → PX4 manual mode)
- [ ] T5.1–T5.5 all pass (motor encoder feedback confirms zero on stop)
- [ ] T7.1–T7.4 timing measurements pass (latency < 100 ms, no spurious RC_LOSS)
- [ ] 60 s continuous joystick session: no RC_LOSS, no spurious disarm

### Gate 5 — Low-speed field testing (≤ 0.2 m/s, spotter)
- [ ] T8.1–T8.8 all pass
- [ ] Video recording of stationary tank turn and arc steering

### Gate 6 — Production deployment (≤ 0.35 m/s validated speed)
- [ ] All previous gates passed
- [ ] `JOYSTICK_MAX_SPEED` raised to 0.35 m/s (matching validated mission speed)
- [ ] React Native app UI reviewed for dead-man button placement and visibility
- [ ] Operator training: joystick does not arm; must arm first via arm event

---

*All CONFIRMED tags above are grounded in direct file inspection. INFERRED tags are logical derivations from confirmed code. PROPOSED tags are design decisions requiring sign-off. UNRESOLVED tags require Jetson runtime verification before Gate 2 can close. Hardware deployment is NO-GO until Gate 4 passes.*
