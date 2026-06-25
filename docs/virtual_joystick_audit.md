# Virtual Joystick — Production Architecture Audit

**Date:** 2026-06-25  
**Scope:** Backend transport, auth, ROS2 nodes, MAVROS/PX4 interface, control-mode arbitration, watchdogs, failsafes, tests  
**Status tags used:** CONFIRMED (traced to code), INFERRED (logical from code), PROPOSED (design decision), UNRESOLVED (needs verification)

---

## 1. Executive Verdict

**CONDITIONAL GO** — the hardware and software foundations are sufficient to implement a safe virtual joystick. All injection points are proven in code. No PX4 firmware changes are required. The recommended design reuses the existing OFFBOARD pipeline and 50 Hz heartbeat, adding a thin gateway layer.

Go criteria before hardware deployment:
1. Software-only bench validation (joystick commands reach RoboClaw without hardware motion) — **NO-GO until done**
2. Controlled indoor field test at <0.2 m/s with E-stop within arm's reach — **NO-GO until done**
3. All arbitration invariants pass automated test suite — **NO-GO until done**

Blocking risks that must be resolved before shipping: control-ownership race (joystick active → mission start concurrency), Socket.IO heartbeat loss without client reconnect, and the absence of any joystick-related auth scope extension.

---

## 2. Physical RC Evidence Map

| # | Stage | Repo / File | Function / Symbol | Input | Output | Safety behaviour |
|---|-------|-------------|-------------------|-------|--------|-----------------|
| 1 | RC hardware → PX4 driver | PX4 / `drivers/rc_input/` | `RCInput` driver | SBUS/PPM/DSM signal | `input_rc` uORB topic | Hardware signal loss → `RC_LOSS` failsafe |
| 2 | Raw RC → normalised | PX4 / `modules/rc_update/RCUpdate.cpp` | `update()` | `input_rc` | `manual_control_input` (SOURCE_RC) | Channel range check; signal must be fresh within COM_RC_LOSS_T |
| 3 | MAVLink path (CONFIRMED alternative) | PX4 / `modules/mavlink/mavlink_receiver.cpp:2132–2179` | `handle_message_manual_control()` | MAVLink MANUAL_CONTROL msg | `manual_control_input` (SOURCE_MAVLINK_0+instance) | x,y,r ∈ [-1000,1000]; z ∈ [0,1000] enforced; `valid=true` set |
| 4 | Source selection | PX4 / `modules/manual_control/ManualControl.cpp` | `processInput()` + `ManualControlSelector` | Up to MAX_MANUAL_INPUT_COUNT `manual_control_input` subs | `manual_control_setpoint` uORB | COM_RC_IN_MODE selects RC-only / MAVLink-only / both; staleness timeout = COM_RC_LOSS_T |
| 5 | RC switch guard | PX4 / `modules/manual_control/ManualControl.cpp` | `processSwitches()` | `manual_control_setpoint.data_source` | Arm/disarm/mode-switch commands | **Only executes if data_source == SOURCE_RC** — MAVLink source CANNOT trigger switch-based arming (CONFIRMED) |
| 6 | Manual mode dispatcher | PX4 / `modules/rover_differential/RoverDifferential.cpp` | `generateSteeringAndThrottleSetpoint()` | `manual_control_setpoint` | `rover_steering_setpoint`, `rover_throttle_setpoint` | Only active when `vehicle_control_mode.flag_control_manual_enabled` |
| 7 | Manual mode axes | PX4 / `DifferentialManualMode/DifferentialManualMode.cpp` | `manual()` | `.roll`→steering diff, `.throttle`→throttle_body_x | uORB steering/throttle setpoints | SlewRate applied in `generateActuatorSetpoint()` |
| 8 | IK mixer | PX4 / `RoverDifferential.cpp` | `computeInverseKinematics(throttle, speed_diff)` | Throttle ∈ [-1,1], diff ∈ [-1,1] | `Vector2f(left, right)` motor commands | If \|left\|>\|right\|>1: yaw priority, throttle reduced |
| 9 | Motor driver | PX4 / `drivers/roboclaw/Roboclaw.cpp` | QPPS velocity opcodes 35/36 | Normalised motor commands | UART → RoboClaw | Disarm guard: zero command on disarm |

CONFIRMED: Physical RC and MAVLink MANUAL_CONTROL share identical downstream paths from `manual_control_input` onward. The only difference is the `data_source` tag which prevents MAVLink from triggering RC-switch-based arming.

---

## 3. Current Rover Movement Pipeline (End-to-End)

### 3a. Mission/OFFBOARD path (current production mode)

```
React Native app
    │  Socket.IO "mission_start" {path_name, auth}
    ▼
PX4_DXP/server/sockets/events.py  on_mission_start()
    │  calls start_mission_for_controller()
    ▼
PX4_DXP/server/offboard_controller.py  OffboardController.start_async()
    │  asyncio.Lock (_lifecycle_lock) — prevents concurrent lifecycle
    │  1. publish_path() → /rpp/cmd_path (nav_msgs/Path, NED, spray flags in z)
    │  2. arm_async() → /mavros/cmd/arming
    │  3. wait SETPOINT_STREAM_GRACE_S
    │  4. RPP health check
    │  5. set_mode_async("OFFBOARD") → /mavros/set_mode
    ▼
PX4_DXP/src/rpp/  (RPP controller, runs in rpp-pipeline.service)
    │  Subscribes /rpp/cmd_path, /mavros/local_position/pose
    │  Outputs /rpp/velocity_ned (Vector3Stamped, NED m/s)
    │         /rpp/yaw_rate_body (Float32, rad/s)
    ▼
PX4_DXP/src/twist_to_setpoint_node.py  TwistToSetpointNode (50 Hz)
    │  Subscribes /rpp/velocity_ned, /rpp/yaw_rate_body
    │  Converts NED→ENU: msg.velocity.x=v_e, .y=v_n, .z=-v_d
    │  Publishes /mavros/setpoint_raw/local (PositionTarget, 50 Hz)
    │  type_mask=455 (vel+yaw+yaw_rate) or 2503 (vel+yaw)
    │  Staleness guard: input_max_age_s=0.2 → streams zeros when stale
    ▼
MAVROS (px4-dxp.service)
    │  setpoint_raw plugin → MAVLink SET_POSITION_TARGET_LOCAL_NED
    ▼
PX4 firmware
    │  DifferentialOffboardMode::offboardControl() — velocity branch
    │  DifferentialVelControl::generateVelocitySetpoint()
    │    bearing = atan2(vE, vN); if |v|<0.01→speed=0, heading freeze
    │    reverse detection: bearing flipped 180° to avoid spot-turn
    │  DifferentialVelControl::generateAttitudeAndThrottleSetpoint()
    │    DRIVING / SPOT_TURNING states; heading error gate
    │  RoverControl::speedControl() PID
    ▼
RoverDifferential::generateActuatorSetpoint()
    │  SlewRate: RO_ACCEL_LIM / RO_DECEL_LIM
    │  computeInverseKinematics()
    ▼
Roboclaw UART (opcodes 35/36, QPPS velocity)
    ▼
Motors
```

### 3b. OFFBOARD heartbeat contract (CONFIRMED)

TwistToSetpointNode maintains the 50 Hz OFFBOARD heartbeat unconditionally. When `/rpp/velocity_ned` goes stale (>200 ms), it streams zero-velocity PositionTarget — rover decelerates under PX4 speed control, OFFBOARD mode is preserved. A complete node crash would stop publishing → PX4 detects OFFBOARD dropout (>500 ms gap) → failsafe.

### 3c. E-stop path (CONFIRMED)

`EmergencyHandler.estop_async()`:
1. `publish_stop_path()` → single-point path at current NED → RPP outputs zero vel → TwistToSetpointNode streams zeros
2. `set_mode_async("MANUAL")` → PX4 exits OFFBOARD
3. `arm_async(False)` → disarm
4. `controller.state = ABORTED`

---

## 4. Injection-Option Comparison

### Option A — MAVLink MANUAL_CONTROL (via MAVROS rc_override or direct MAVLink)

**Path:** Server → MAVROS `/mavros/rc/override` → MAVROS RC override plugin → PX4 `manual_control_input` (SOURCE_MAVLINK)  
**PX4 code:** `mavlink_receiver.cpp:2132` `handle_message_manual_control()`  
**Rover moves when:** vehicle_control_mode.flag_control_manual_enabled (MANUAL/STABILIZED/POSITION modes)

| Pros | Cons |
|------|------|
| Does not require OFFBOARD mode | Requires mode switch from OFFBOARD → MANUAL before joystick works |
| Independent of RPP/TwistToSetpointNode | Must manage mode switching explicitly |
| rc_override plugin already available (not in denylist) | OFFBOARD heartbeat gap during mode switch → PX4 failsafe unless heartbeat bridged |
| Direct velocity/steering control | MAVLink switch arm/disarm blocked (SOURCE_MAVLINK) — must use service calls |
| No 50 Hz setpoint stream needed | Cannot coexist with active mission in OFFBOARD |
| | Axes are stick-style (roll/throttle), not velocity; joystick UX maps less naturally |

**Assessment:** Viable but forces mode-switching complexity. Every joystick session requires OFFBOARD→MANUAL and back. During mode transition the OFFBOARD heartbeat must be bridged to avoid failsafe. Not recommended as primary path.

---

### Option B — Direct `/mavros/setpoint_velocity/cmd_vel` (TwistStamped)

**Path:** New ROS2 publisher → `/mavros/setpoint_velocity/cmd_vel` → MAVROS setpoint_velocity plugin → PX4 `trajectory_setpoint` velocity  
**Rover moves when:** OFFBOARD mode active, `offboard_control_mode.velocity=true`

| Pros | Cons |
|------|------|
| Clean velocity semantics (m/s forward, rad/s yaw) | setpoint_velocity plugin is NOT explicitly in denylist but its behavior with rover differential needs verification |
| Standard MAVROS interface | Requires its own 50 Hz heartbeat — would compete with TwistToSetpointNode's current heartbeat on `/mavros/setpoint_raw/local` |
| ENU frame (intuitive) | Two simultaneous OFFBOARD setpoint sources would create undefined arbitration in PX4 |

**Assessment:** Rejected. Competing with TwistToSetpointNode creates undefined PX4 behaviour. Would need TwistToSetpointNode disabled during joystick, which breaks the heartbeat contract.

---

### Option C — Inject into `/rpp/velocity_ned` (TwistToSetpointNode input) [RECOMMENDED]

**Path:** New `JoystickGatewayNode` publishes to `/rpp/velocity_ned` → TwistToSetpointNode converts and streams at 50 Hz → same OFFBOARD path as missions  
**Rover moves when:** OFFBOARD active, rover armed — same pre-conditions as mission mode

| Pros | Cons |
|------|------|
| Reuses existing TwistToSetpointNode heartbeat — no second heartbeat needed | Must suppress RPP during joystick (RPP also publishes `/rpp/velocity_ned`) |
| TwistToSetpointNode's 200 ms staleness guard is a free ROS-layer watchdog | Requires RPP be paused/idled when joystick active |
| No PX4 mode changes required — stays in OFFBOARD | Joystick active requires OFFBOARD mode to already be armed |
| EKF2, speed limits, SlewRate all apply — rover cannot be commanded faster than RO_SPEED_LIM | |
| E-stop path unchanged — works identically | |
| NED vector injection is clean: forward=+N, right=+E | |

**Assessment:** Recommended primary approach. The RPP suppression concern is resolved by the existing RPP idle state — when no path is loaded or the mission is stopped, RPP outputs zero on `/rpp/velocity_ned`. A mux/priority signal from JoystickGatewayNode suppresses RPP publication while joystick is active.

---

### Option D — New standalone OFFBOARD publisher (bypasses TwistToSetpointNode entirely)

**Path:** JoystickGatewayNode publishes directly to `/mavros/setpoint_raw/local` at 50 Hz  
**Rover moves when:** OFFBOARD active

| Pros | Cons |
|------|------|
| Full direct control | Must own entire 50 Hz heartbeat responsibility |
| No dependency on TwistToSetpointNode | Heartbeat gap on node crash → PX4 failsafe (no fallback) |
| | Duplicates TwistToSetpointNode's NED→ENU conversion and type_mask logic |
| | Two publishers on `/mavros/setpoint_raw/local` if node coexists with TwistToSetpointNode |

**Assessment:** More fragile than Option C. Not recommended unless TwistToSetpointNode architecture changes.

---

### Summary table

| Option | Mode required | Heartbeat owner | RPP interaction | Recommendation |
|--------|--------------|-----------------|-----------------|----------------|
| A (MAVLink MANUAL_CONTROL) | MANUAL | N/A | Incompatible | Fallback only |
| B (cmd_vel) | OFFBOARD | New + TwistToSetpoint conflict | Incompatible | Rejected |
| C (velocity_ned injection) | OFFBOARD | TwistToSetpointNode (unchanged) | Must idle RPP | **Primary** |
| D (direct setpoint_raw) | OFFBOARD | New node | Must disable TwistToSetpointNode | Not recommended |

---

## 5. Proposed Architecture

### 5a. ASCII flow diagram

```
React Native App
    │
    │  Socket.IO (authenticated, X-Rover-Token / auth dict)
    │  Events:
    │    joystick_start   {auth}
    │    joystick_cmd     {auth, vx:float, vy:float, omega:float}
    │    joystick_stop    {auth}
    │    emergency_stop   {auth}  (existing)
    ▼
PX4_DXP/server/sockets/events.py
    register_handlers()
    ┌─────────────────────────────────────────────────────┐
    │  on_joystick_start(sid, data)                        │
    │    _auth_ok(data) → reject if fail                   │
    │    JoystickController.acquire_control(sid)           │
    │      checks: no mission RUNNING, rover armed,        │
    │              OFFBOARD active                         │
    │    → emit "joystick_acquired" or "joystick_error"    │
    │                                                      │
    │  on_joystick_cmd(sid, data)                          │
    │    _auth_ok(data) → reject if fail                   │
    │    JoystickController.handle_cmd(sid, vx, vy, omega) │
    │      owner check: only owner sid accepted            │
    │      clamp: |vx|,|vy| ≤ JOYSTICK_MAX_SPEED          │
    │             |omega| ≤ JOYSTICK_MAX_YAW_RATE          │
    │      stamp and forward to JoystickGatewayNode        │
    │                                                      │
    │  on_joystick_stop(sid, data)                         │
    │    JoystickController.release_control(sid)           │
    │    → publish zero vel → gateway publishes stop       │
    └─────────────────────────────────────────────────────┘
    │
    ▼
PX4_DXP/server/joystick_controller.py  (new)
    JoystickController
    ├── _owner_sid: str | None
    ├── _last_cmd_time: float
    ├── _watchdog_task: asyncio.Task
    │     polls every 100 ms; if _last_cmd_time > JOYSTICK_TIMEOUT_S (0.5 s):
    │       publish zero → _publish_stop_velocity()
    │       if > JOYSTICK_RELEASE_S (2.0 s): release_control()
    ├── acquire_control(sid) → checks OffboardController.state
    ├── handle_cmd(sid, vx, vy, omega)
    │     stamps command; calls ros_node.publish_joystick_velocity(vx, vy, omega)
    └── release_control(sid)
          calls ros_node.publish_joystick_velocity(0, 0, 0)
          signals JoystickGatewayNode to idle
    │
    ▼
PX4_DXP/server/ros_node.py  RosBridgeNode  (extended)
    New publisher: self._joy_vel_pub
        topic:  /joystick/velocity_ned   (geometry_msgs/Vector3Stamped)
        frame:  "map" (NED)
    publish_joystick_velocity(vx_north, vy_east, omega_rad_s)
        builds Vector3Stamped, stamps, publishes
    │
    ▼
PX4_DXP/src/joystick_gateway_node.py  (new ROS2 node, runs in rpp-pipeline.service)
    JoystickGatewayNode
    ├── Subscribes /joystick/velocity_ned (Vector3Stamped)
    │     staleness: 200 ms (same as TwistToSetpointNode input_max_age_s)
    ├── Subscribes /rpp/active (Bool or inferred from /rpp/state)
    ├── When joystick active:
    │     Publishes /rpp/velocity_ned at 20 Hz (NED)
    │     Sets /joystick/active → True (Bool, latched)
    │     Signals RPP to idle (publish empty path to /rpp/cmd_path OR
    │           via /joystick/active flag that RPP checks before publishing)
    ├── When joystick inactive / stale:
    │     Publishes /rpp/velocity_ned zero once → TwistToSetpointNode streams zeros
    │     Sets /joystick/active → False
    │     RPP resumes normal operation
    └── E-stop passthrough: if /emergency_stop received → zero immediately
    │
    ▼
TwistToSetpointNode (existing, UNCHANGED)
    /rpp/velocity_ned (now written by either RPP or JoystickGatewayNode)
    50 Hz → /mavros/setpoint_raw/local
    │
    ▼
MAVROS → PX4 → RoboClaw → Motors
```

### 5b. RPP suppression mechanism

When `JoystickGatewayNode` sets `/joystick/active=True`, RPP must not publish to `/rpp/velocity_ned`. Two acceptable implementations (choose one during implementation):

**Option C1 (preferred):** RPP subscribes `/joystick/active` (Bool, latched). When True, RPP skips publishing on `/rpp/velocity_ned` but continues its internal loop. Zero-velocity is published by JoystickGatewayNode. RPP resumes on False.

**Option C2 (alternative):** `JoystickGatewayNode` publishes an empty path to `/rpp/cmd_path` when joystick starts. RPP enters DONE/IDLE state and stops publishing velocity. Joystick stop restores whatever path was previously active — requires path save/restore logic.

Option C1 is preferred because it has no side effect on the RPP path state machine.

---

## 6. Mission and Mode Arbitration — State-Transition Table

### Control ownership states

```
IDLE            → No active mission, no joystick
MISSION_ACTIVE  → OffboardController.state ∈ {ARMING,SWITCHING_OFFBOARD,RUNNING}
JOYSTICK_ACTIVE → JoystickController._owner_sid is not None
```

**Invariant (must be enforced in code):** `MISSION_ACTIVE` and `JOYSTICK_ACTIVE` are mutually exclusive. Only one can hold `OFFBOARD` setpoint ownership at a time.

### State transitions

| Current state | Event | Action | Next state | Guard |
|---------------|-------|--------|------------|-------|
| IDLE | joystick_start | arm_async() if not armed; set_mode("OFFBOARD") if not in OFFBOARD; acquire_control() | JOYSTICK_ACTIVE | Rover must be connectable; MAVROS connected |
| IDLE | mission_start | OffboardController.start_async() (existing) | MISSION_ACTIVE | — |
| JOYSTICK_ACTIVE | joystick_stop | publish zero; release_control() | IDLE | — |
| JOYSTICK_ACTIVE | emergency_stop | estop_async() (existing) | IDLE | — |
| JOYSTICK_ACTIVE | joystick timeout (500 ms no cmd) | publish zero; soft hold | JOYSTICK_ACTIVE (held) | JoystickController watchdog |
| JOYSTICK_ACTIVE (held, >2 s) | watchdog fires | release_control(); mode remains OFFBOARD/armed | IDLE | — |
| JOYSTICK_ACTIVE | mission_start | **REJECT** — emit joystick_error | JOYSTICK_ACTIVE (no change) | Ownership check: owner_sid not None |
| MISSION_ACTIVE | joystick_start | **REJECT** — emit joystick_error | MISSION_ACTIVE (no change) | OffboardController.state ∈ MISSION_STATES |
| MISSION_ACTIVE | mission_stop/abort | stop_active_mission() (existing) | IDLE | — |
| MISSION_ACTIVE | joystick_start (after stop completes) | acquire_control() | JOYSTICK_ACTIVE | Must wait for IDLE |
| ANY | socket disconnect | if owner_sid == disconnected_sid: release_control() | IDLE (if was JOYSTICK_ACTIVE) | on_disconnect callback |
| ANY | MAVROS connected=False | JoystickController.release_control(); estop if JOYSTICK_ACTIVE | IDLE | Safety watchdog in main.py |

### Mode arbitration rules (PX4 side)

- Joystick requires OFFBOARD mode (same as mission). The arming/mode flow is identical.
- Joystick does NOT arm/disarm by itself via RC switch commands — must use explicit `arm_async()` service call.
- If rover is already armed and in OFFBOARD (e.g. just finished a mission in IDLE): joystick_start skips arm and mode steps.
- If rover is disarmed/MANUAL: joystick_start must arm and set OFFBOARD — same pre-flight sequence as mission but without a path.

**UNRESOLVED:** Should joystick_start arm the rover automatically? Recommend: NO for first release. Require operator to arm via existing `arm` socket event first, then joystick_start acquires control. Reduces surprise arming from accidental joystick events.

---

## 7. File-by-File Implementation Plan

All changes are in `PX4_DXP/`. No PX4 firmware changes required.

---

### File 1: `server/joystick_controller.py` — NEW FILE

```
Purpose: Single-owner joystick state machine + server-side watchdog
Key classes: JoystickController
```

**Interface:**
```python
class JoystickController:
    # Constants (tune after hardware validation)
    JOYSTICK_TIMEOUT_S     = 0.5   # no cmd → publish zero
    JOYSTICK_RELEASE_S     = 2.0   # zero held this long → release
    JOYSTICK_MAX_SPEED     = 0.35  # m/s (match current validated speed)
    JOYSTICK_MAX_YAW_RATE  = 0.45  # rad/s (match validated max_yaw_rate_body)

    def __init__(self, ros_node, offboard_ctrl): ...

    async def acquire_control(self, sid: str) -> tuple[bool, str]:
        # Reject if offboard_ctrl.state in MISSION_STATES
        # Set _owner_sid, start _watchdog_task
        # Publish /joystick/active = True via ros_node

    def handle_cmd(self, sid: str, vx: float, vy: float, omega: float) -> bool:
        # Returns False if sid != _owner_sid
        # Clamp inputs
        # Update _last_cmd_time
        # Call ros_node.publish_joystick_velocity(vx, vy, omega)

    async def release_control(self, sid: str | None = None):
        # sid=None → force release (estop path)
        # Publish zero velocity
        # Publish /joystick/active = False
        # Cancel _watchdog_task
        # Clear _owner_sid

    async def _watchdog_loop(self):
        # Every 100 ms: check _last_cmd_time
        # If age > JOYSTICK_TIMEOUT_S: publish zero
        # If age > JOYSTICK_RELEASE_S: release_control()

    @property
    def is_active(self) -> bool: ...

    @property
    def owner_sid(self) -> str | None: ...
```

**MISSION_STATES:** `{MissionState.ARMING, MissionState.SWITCHING_OFFBOARD, MissionState.RUNNING, MissionState.STOPPING}`

---

### File 2: `server/ros_node.py` — EXTEND RosBridgeNode

Add publisher and publish method. Minimal diff.

```python
# In __init__, after existing publishers:
self._joy_vel_pub = self.create_publisher(
    Vector3Stamped, "/joystick/velocity_ned", 10
)
self._joy_active_pub = self.create_publisher(
    Bool, "/joystick/active", qos_profile_latched  # transient_local, depth=1
)

def publish_joystick_velocity(
    self, vx_north: float, vy_east: float, omega: float = 0.0
) -> None:
    """Publish joystick velocity in NED frame.
    vx_north: forward speed m/s (+N)
    vy_east:  lateral speed m/s (+E)
    omega: yaw rate rad/s (not used by TwistToSetpointNode directly;
           reserved for future body_rate mode)
    """
    msg = Vector3Stamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = "map"
    msg.vector.x = vx_north
    msg.vector.y = vy_east
    msg.vector.z = 0.0  # no vertical component for ground rover
    self._joy_vel_pub.publish(msg)

def publish_joystick_active(self, active: bool) -> None:
    msg = Bool()
    msg.data = active
    self._joy_active_pub.publish(msg)
```

**Note:** `omega` (yaw rate) is included in the method signature for future use. For the first release, TwistToSetpointNode's NED→ENU velocity-bearing conversion handles implicit yaw; explicit yaw_rate injection requires type_mask change in TwistToSetpointNode and is deferred (see Section 10, Risk R4).

---

### File 3: `src/joystick_gateway_node.py` — NEW ROS2 NODE

```python
"""
JoystickGatewayNode

Subscribes:
  /joystick/velocity_ned  (geometry_msgs/Vector3Stamped, from FastAPI bridge)
  /joystick/active        (std_msgs/Bool, latched)

Publishes:
  /rpp/velocity_ned       (geometry_msgs/Vector3Stamped, NED)
    → only when joystick active AND input fresh (< STALE_S)
    → zero when joystick active but stale (triggers TwistToSetpointNode zeros)
    → silent (no publish) when joystick inactive (lets RPP own the topic)

Parameters:
  stale_s       (float, default 0.2)   — input staleness timeout
  max_speed     (float, default 0.35)  — hard clamp, m/s
  max_yaw_rate  (float, default 0.45)  — future use

Node lifecycle:
  - Runs in same process as rpp-pipeline (or as standalone node added to px4-dxp.service)
  - If node crashes: /rpp/velocity_ned goes silent (no joystick, no RPP)
    TwistToSetpointNode streams zeros → rover decelerates → OFFBOARD preserved
    (safe failure mode)
"""

class JoystickGatewayNode(Node):
    def __init__(self):
        super().__init__("joystick_gateway")
        # Subscribers
        self._joy_vel_sub = self.create_subscription(
            Vector3Stamped, "/joystick/velocity_ned",
            self._on_joy_vel, 10
        )
        self._joy_active_sub = self.create_subscription(
            Bool, "/joystick/active",
            self._on_joy_active,
            qos_profile_latched  # must match publisher QoS
        )
        # Publisher — competes with RPP on this topic; RPP must check /joystick/active
        self._vel_pub = self.create_publisher(
            Vector3Stamped, "/rpp/velocity_ned", 10
        )
        # State
        self._active = False
        self._last_vel: Vector3Stamped | None = None
        self._last_vel_time = 0.0
        self._stale_s = self.declare_parameter("stale_s", 0.2).value
        self._max_speed = self.declare_parameter("max_speed", 0.35).value
        # 20 Hz publish timer (only when active)
        self._timer = self.create_timer(0.05, self._timer_cb)

    def _on_joy_active(self, msg: Bool):
        self._active = msg.data
        if not self._active:
            # Publish one zero to flush TwistToSetpointNode
            self._publish_zero()

    def _on_joy_vel(self, msg: Vector3Stamped):
        # Clamp
        spd = math.sqrt(msg.vector.x**2 + msg.vector.y**2)
        if spd > self._max_speed and spd > 0:
            scale = self._max_speed / spd
            msg.vector.x *= scale
            msg.vector.y *= scale
        self._last_vel = msg
        self._last_vel_time = time.monotonic()

    def _timer_cb(self):
        if not self._active:
            return
        age = time.monotonic() - self._last_vel_time
        if age > self._stale_s:
            self._publish_zero()
        else:
            self._vel_pub.publish(self._last_vel)

    def _publish_zero(self):
        z = Vector3Stamped()
        z.header.stamp = self.get_clock().now().to_msg()
        z.header.frame_id = "map"
        self._vel_pub.publish(z)
```

**RPP modification required:** RPP's velocity publisher must check `/joystick/active` (latched Bool subscriber) and skip publishing to `/rpp/velocity_ned` when True. This is a ≤10-line change in the RPP node's publish callback.

---

### File 4: `server/sockets/events.py` — EXTEND register_handlers()

Add three new Socket.IO event handlers inside `register_handlers(sio)`:

```python
@sio.on("joystick_start")
async def on_joystick_start(sid, data=None):
    from main import joystick_ctrl
    if not _auth_ok(data):
        return await _emit_unauth(sio, sid)
    if joystick_ctrl is None:
        return await sio.emit("joystick_error", {"reason": "not_initialised"}, to=sid)
    ok, reason = await joystick_ctrl.acquire_control(sid)
    if ok:
        await sio.emit("joystick_acquired", {"sid": sid}, to=sid)
    else:
        await sio.emit("joystick_error", {"reason": reason}, to=sid)

@sio.on("joystick_cmd")
async def on_joystick_cmd(sid, data=None):
    from main import joystick_ctrl
    if not _auth_ok(data):
        return await _emit_unauth(sio, sid)
    if joystick_ctrl is None:
        return
    if not isinstance(data, dict):
        return
    vx    = float(data.get("vx", 0.0))
    vy    = float(data.get("vy", 0.0))
    omega = float(data.get("omega", 0.0))
    ok = joystick_ctrl.handle_cmd(sid, vx, vy, omega)
    if not ok:
        await sio.emit("joystick_error", {"reason": "not_owner"}, to=sid)

@sio.on("joystick_stop")
async def on_joystick_stop(sid, data=None):
    from main import joystick_ctrl
    if not _auth_ok(data):
        return await _emit_unauth(sio, sid)
    if joystick_ctrl is None:
        return
    await joystick_ctrl.release_control(sid)
    await sio.emit("joystick_released", {}, to=sid)
```

**Extend `disconnect` handler:**
```python
@sio.event
async def disconnect(sid):
    from main import activity_log, joystick_ctrl
    activity_log.append(...)
    # Release joystick if owner disconnects
    if joystick_ctrl is not None and joystick_ctrl.owner_sid == sid:
        await joystick_ctrl.release_control(sid)
```

---

### File 5: `server/main.py` — EXTEND lifespan + expose joystick_ctrl global

```python
# Add to globals section (near offboard_ctrl, path_mgr etc.):
joystick_ctrl: JoystickController | None = None

# Add to lifespan initialisation block (after OffboardController init):
from joystick_controller import JoystickController
joystick_ctrl = JoystickController(ros_node=ros_node, offboard_ctrl=offboard_ctrl)

# Add to safety watchdog (_telemetry_loop) — if joystick active and connected=False:
if joystick_ctrl is not None and joystick_ctrl.is_active:
    if not telemetry.connected:
        await joystick_ctrl.release_control()
        await emergency_handler.estop_async()
```

---

### File 6: `server/models.py` — EXTEND (optional but recommended)

Add joystick-related models for type safety:

```python
class JoystickCmd(BaseModel):
    vx:    float = Field(0.0, ge=-1.0, le=1.0)   # m/s, NED north
    vy:    float = Field(0.0, ge=-1.0, le=1.0)   # m/s, NED east
    omega: float = Field(0.0, ge=-1.0, le=1.0)   # rad/s
    auth:  str | None = None

class JoystickState(str, Enum):
    IDLE    = "idle"
    ACTIVE  = "active"
    HELD    = "held"   # active but stale, publishing zero
```

---

### File 7: RPP node (existing, path TBD) — MINOR EDIT

Add subscriber for `/joystick/active` (latched Bool). In the RPP publish callback:

```python
if self._joystick_active:
    return   # skip publishing /rpp/velocity_ned; JoystickGatewayNode owns it
```

This is the **only** change needed in RPP. The RPP path state machine, controller logic, and all existing behaviour are unchanged.

---

### File 8: `px4_start_service.sh` or `rpp-pipeline.service` — ADD JoystickGatewayNode

If `rpp-pipeline.service` launches nodes via a launch file or exec script, add `joystick_gateway_node` to the same process/launch:

```bash
ros2 run px4_dxp joystick_gateway_node &
```

Or in a Python launch file:
```python
Node(package="px4_dxp", executable="joystick_gateway_node", name="joystick_gateway"),
```

Restart scope: changes to `joystick_gateway_node.py` → restart `rpp-pipeline.service` (not `px4-dxp.service` — no MAVROS drop).

---

## 8. Test Plan

### T1 — Unit: JoystickController state machine (no hardware)

| Test | Input | Expected |
|------|-------|----------|
| T1.1 | acquire_control() when offboard_ctrl.state=RUNNING | Returns (False, "mission_active") |
| T1.2 | acquire_control() when offboard_ctrl.state=IDLE | Returns (True, "ok") |
| T1.3 | handle_cmd(wrong_sid, ...) | Returns False |
| T1.4 | handle_cmd(owner_sid, 0.5, 0.0, 0.0) | Clamped to max_speed; publish called |
| T1.5 | handle_cmd(owner_sid, 10.0, 0.0, 0.0) | Clamped to JOYSTICK_MAX_SPEED (0.35) |
| T1.6 | No cmd for 0.6 s (watchdog fires) | Zero published; state=HELD |
| T1.7 | No cmd for 2.1 s (watchdog fires) | release_control() called; owner_sid=None |
| T1.8 | release_control() while not owner | No error; no state change |
| T1.9 | mission_start while joystick active | mission_start sees JOYSTICK_ACTIVE; REJECT |
| T1.10 | joystick_start while mission RUNNING | Returns (False, "mission_active") |

### T2 — Unit: JoystickGatewayNode (ROS2 mock)

| Test | Input | Expected on /rpp/velocity_ned |
|------|-------|-------------------------------|
| T2.1 | /joystick/active=False | No publish |
| T2.2 | /joystick/active=True; vel=(0.2,0,0) | Publishes 0.2,0,0 at 20 Hz |
| T2.3 | active=True; input stale >200 ms | Publishes 0,0,0 |
| T2.4 | active=True; vel=(2.0,0,0) | Clamped to max_speed=0.35 |
| T2.5 | active=True → False | One zero publish then silent |

### T3 — Integration: Socket.IO → ROS topic (Jetson, no motor power)

```bash
# From Mac or Jetson
python3 tools/test_joystick.py --host localhost --cmd '{"vx":0.1,"vy":0.0}'
# Verify with:
ros2 topic echo /joystick/velocity_ned --once
ros2 topic echo /rpp/velocity_ned --once   # should show 0.1,0,0 when active
```

| Test | Steps | Expected |
|------|-------|----------|
| T3.1 | joystick_start without arm | Reject (rover not armed) |
| T3.2 | arm + joystick_start | joystick_acquired; /joystick/active=True |
| T3.3 | joystick_cmd {vx:0.1} | /rpp/velocity_ned shows 0.1,0,0 |
| T3.4 | joystick_cmd from different SID | joystick_error "not_owner" |
| T3.5 | socket disconnect | /joystick/active→False; /rpp/velocity_ned silent |
| T3.6 | mission_start while joystick active | 409/joystick_error |
| T3.7 | joystick_start while mission running | joystick_error "mission_active" |
| T3.8 | joystick_stop | /joystick/active→False; released event |

### T4 — Integration: PX4 OFFBOARD reachability (Jetson, wheels blocked/motor-off)

| Test | Steps | Expected |
|------|-------|----------|
| T4.1 | arm + OFFBOARD + joystick_cmd {vx:0.1} | PX4 reports velocity setpoint in OFFBOARD; no motion (wheels blocked) |
| T4.2 | joystick active + pull ethernet | MAVROS connected=False → watchdog releases joystick → estop |
| T4.3 | joystick active + kill rpp-pipeline | /rpp/velocity_ned silent → TwistToSetpointNode streams zeros |
| T4.4 | joystick active + stop cmd stream for 2 s | Watchdog releases; /joystick/active=False |

### T5 — Hardware validation (controlled field, <0.2 m/s, spotter present)

| Test | Steps | Expected |
|------|-------|----------|
| T5.1 | Forward vx=0.1 m/s | Rover moves forward at ~0.1 m/s |
| T5.2 | Stop cmd mid-motion | Rover decelerates under SlewRate; stops |
| T5.3 | Reverse vx=-0.1 m/s | DifferentialVelControl reverse bearing flip; rover moves reverse |
| T5.4 | Emergency stop during motion | Rover stops, disarms within ~3 s |
| T5.5 | Socket disconnect at speed | Joystick released; rover stops |
| T5.6 | Mission start + complete + joystick_start | Joystick acquires after mission IDLE |

---

## 9. Configuration and Parameters

### 9a. New server-side constants (joystick_controller.py)

| Constant | Value | Rationale |
|----------|-------|-----------|
| `JOYSTICK_TIMEOUT_S` | 0.5 s | Matches PX4 OFFBOARD timeout (0.5 s gap → failsafe); server acts before PX4 |
| `JOYSTICK_RELEASE_S` | 2.0 s | Enough time for brief network glitch without full release |
| `JOYSTICK_MAX_SPEED` | 0.35 m/s | Matches current validated production speed; raise after SPD-T1 |
| `JOYSTICK_MAX_YAW_RATE` | 0.45 rad/s | Matches validated max_yaw_rate_body |
| `WATCHDOG_POLL_S` | 0.1 s | 10 Hz poll; low CPU cost |

### 9b. New ROS2 node parameters (joystick_gateway_node.py)

| Parameter | Default | Rationale |
|-----------|---------|-----------|
| `stale_s` | 0.2 s | Matches TwistToSetpointNode input_max_age_s |
| `max_speed` | 0.35 m/s | Server-side clamp mirrored in ROS layer |
| `publish_hz` | 20 Hz | Half TwistToSetpointNode rate; sufficient heartbeat contribution |

### 9c. PX4 FCU parameters — NO CHANGES REQUIRED

All existing validated parameters apply. RO_SPEED_LIM enforces speed ceiling inside PX4 independent of joystick. RO_ACCEL_LIM / RO_DECEL_LIM enforce SlewRate.

### 9d. Socket.IO events contract

| Event (client→server) | Payload fields | Auth required |
|-----------------------|---------------|---------------|
| `joystick_start` | `{auth: str}` | Yes |
| `joystick_cmd` | `{auth: str, vx: float, vy: float, omega: float}` | Yes |
| `joystick_stop` | `{auth: str}` | Yes |

| Event (server→client) | Payload | When |
|-----------------------|---------|------|
| `joystick_acquired` | `{sid: str}` | On successful acquire |
| `joystick_released` | `{}` | On successful release |
| `joystick_error` | `{reason: str}` | On any rejection |
| `telemetry` | existing TelemetryData + `joystick_active: bool` | Every 100 ms (add field) |

### 9e. Telemetry extension

Add `joystick_active: bool` to `TelemetryData` in `server/models.py`. Populate in `_telemetry_loop()` via `joystick_ctrl.is_active`. React Native app uses this to show joystick state independently of Socket.IO event sequence.

---

## 10. Risks and Unresolved Questions

| ID | Risk / Question | Severity | Mitigation |
|----|-----------------|----------|------------|
| R1 | **Race: joystick_start concurrently with mission_start** | HIGH | `_lifecycle_lock()` in OffboardController serialises mission ops. JoystickController.acquire_control() must also check under the same lock or its own mutex. Unresolved: determine if a single shared asyncio.Lock covers both, or if separate locks with consistent acquisition order are needed. |
| R2 | **Socket.IO reconnect after network drop — joystick ownership left dangling** | HIGH | Watchdog fires JOYSTICK_RELEASE_S (2 s) after last cmd. Reconnecting client gets new SID → must joystick_start again. Old SID ownership is cleared by watchdog. This is the intended behaviour but must be tested. |
| R3 | **RPP publishes to /rpp/velocity_ned simultaneously with JoystickGatewayNode** | MEDIUM | Topic has two publishers → latest message wins at TwistToSetpointNode. If /joystick/active latched message is delivered to RPP with any delay, there is a brief window where both publish. Mitigation: JoystickGatewayNode publishes zero first, waits 50 ms (one RPP cycle), then publishes joystick velocity. Or: RPP checks the flag synchronously before each publish. |
| R4 | **Yaw rate not forwarded to PX4 in current TwistToSetpointNode** | LOW | TwistToSetpointNode uses velocity bearing (atan2) for implicit yaw control. Explicit omega from joystick cmd is ignored in first release. For tank-turn behaviour, body_rate OFFBOARD mode is needed — requires type_mask change in TwistToSetpointNode. Deferred to v2. |
| R5 | **MAVROS state TRANSIENT_LOCAL gives stale connected=True** | MEDIUM | ros_node.py already overrides with `_state_recv_time` 2 s timeout. Joystick watchdog in main.py must also consult this, not raw `connected` flag. CONFIRMED behaviour in ros_node.py. |
| R6 | **joystick_start when rover is disarmed / not in OFFBOARD** | MEDIUM | First release: require rover to be pre-armed and pre-OFFBOARD (operator arms via existing `arm` event, then sets OFFBOARD via `set_mode` event, then joystick_start). Alternative: joystick_start triggers full arm+OFFBOARD sequence automatically (like mission start). Recommend explicit operator sequence for safety. UNRESOLVED: pick one approach and document in UX. |
| R7 | **Speed limits at higher speeds (SPD-T1 backlog)** | LOW | JOYSTICK_MAX_SPEED=0.35 mirrors mission speed. When SPD-T1 is validated, raise constant. No architectural change needed. |
| R8 | **rpp-pipeline.service restart scope for JoystickGatewayNode** | LOW | JoystickGatewayNode runs in rpp-pipeline.service. If the service is restarted mid-joystick-session, /joystick/active goes stale → TwistToSetpointNode streams zeros → safe. But /joystick/active latched publisher is lost on restart — JoystickController must re-publish on reconnect. |
| R9 | **COM_RC_IN_MODE value on production FCU** | UNRESOLVED | If COM_RC_IN_MODE = RC_ONLY, MAVLink MANUAL_CONTROL messages are ignored (affects Option A fallback). Current value unknown. Verify via QGC before implementing Option A fallback. |
| R10 | **JoystickGatewayNode publish rate vs TwistToSetpointNode heartbeat** | LOW | JoystickGateway publishes at 20 Hz; TwistToSetpointNode reads at 50 Hz. TwistToSetpointNode will repeat the last value at 50 Hz until it goes stale (200 ms = 4 publish cycles). This is acceptable. |

---

## 11. Ordered Implementation Phases

### Phase J1 — Server-side skeleton (no ROS, no hardware)

1. Create `server/joystick_controller.py` with mock `ros_node` stub
2. Extend `server/sockets/events.py` with `joystick_start`, `joystick_cmd`, `joystick_stop` handlers
3. Extend `server/main.py` with `joystick_ctrl` global + lifespan init
4. Run `python3 -m pytest tests/test_joystick_controller.py` — all T1 tests pass
5. Manual Socket.IO test: connect from Mac, verify event flow, verify auth rejection

**Gate:** T1.1–T1.10 all pass. No hardware needed.

---

### Phase J2 — ROS2 node + topic wiring (Jetson, no motor power)

1. Create `src/joystick_gateway_node.py`
2. Extend `server/ros_node.py` with `publish_joystick_velocity()` and `publish_joystick_active()`
3. Add JoystickGatewayNode to rpp-pipeline launch
4. Apply RPP `/joystick/active` check (≤10-line change)
5. Restart `rpp-pipeline.service`
6. Run T2 and T3 tests

Verify on Jetson:
```bash
ros2 topic echo /joystick/velocity_ned
ros2 topic echo /rpp/velocity_ned
ros2 topic echo /joystick/active
```

**Gate:** T2.1–T2.5, T3.1–T3.8 all pass. Topics wired correctly. RPP silent when joystick active.

---

### Phase J3 — PX4 OFFBOARD reachability (Jetson, wheels blocked/props removed)

1. Arm rover via existing `arm` socket event
2. Set OFFBOARD mode via existing `set_mode` event
3. joystick_start → joystick_cmd {vx:0.1}
4. Confirm PX4 telemetry shows velocity setpoint change
5. Run T4.1–T4.4

Verify PX4 side:
```bash
ros2 topic echo /mavros/setpoint_raw/local --once
# Should show velocity (0.1 m/s north → ENU: x=0, y=0.1)
ssh flash@192.168.1.102 'cd ~/PX4_DXP && python3 tools/capture_telemetry.py -n 5 --host localhost'
# Watch speed_m_s respond
```

**Gate:** PX4 accepts velocity setpoints via joystick path. Watchdog fires correctly. E-stop works. No unexpected mode changes.

---

### Phase J4 — Controlled hardware validation (field, spotter)

1. Set JOYSTICK_MAX_SPEED = 0.1 m/s for first hardware run
2. Run T5.1–T5.6 in order
3. Verify E-stop, disconnect, and watchdog behaviour with rover moving
4. Raise JOYSTICK_MAX_SPEED to 0.2 m/s, repeat
5. Raise to 0.35 m/s (production max), repeat

**Gate:** All T5 tests pass at 0.35 m/s. E-stop latency < 1 s. No unexpected disarms.

---

### Phase J5 — React Native frontend integration

1. Implement virtual joystick UI (two-thumb or single-thumb with directional pad)
2. Emit `joystick_cmd` at 10 Hz when thumb held (app-side rate limit)
3. Emit `joystick_stop` on thumb release and on app background
4. Display `joystick_active` from telemetry stream
5. Block mission start UI when `joystick_active=true`
6. End-to-end test: full drive session, mission handoff, E-stop from app

**Rate recommendation:** App emits at 10 Hz. Server watchdog is 500 ms = 5 missed packets before zero. TwistToSetpointNode stale is 200 ms = 2 missed packets. App must emit at >5 Hz minimum; 10 Hz provides comfortable margin.

---

### Phase J6 — Hardening and optional yaw rate (deferred)

1. Add explicit yaw rate support: change TwistToSetpointNode type_mask to include yaw_rate when `/rpp/yaw_rate_body` has joystick source. Or add separate joystick yaw_rate publisher.
2. Persistent ownership token: allow reconnecting client to reclaim joystick session within grace period using session token (not SID)
3. Telemetry: add `joystick_cmd_age_ms` to TelemetryData for frontend latency display
4. Rate-limit `joystick_cmd` events server-side (discard if age < 80 ms) to protect against chatty clients

---

*End of audit. All CONFIRMED tags are grounded in code traced during investigation. PROPOSED items require implementation decision. UNRESOLVED items require follow-up before Phase J3 gate.*
