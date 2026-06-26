# React Native Virtual Joystick V2 — Frontend Wiring Guide

**Status:** Implementation-ready. All fields and events verified against backend commit
`e0e3debef91726309b6bac02570aa8208b78fd9f` and current React Native frontend code.

**Target frontend:** `DXF_Three_Wheel` (Expo SDK 54, TypeScript, React Navigation)
**Backend:** `PX4_DXP/server` (FastAPI + Socket.IO + asyncio)
**Jetson IP:** `192.168.1.102:5001`

---

## Table of Contents

1. [Backend Contract Summary](#1-backend-contract-summary)
2. [Frontend Architecture](#2-frontend-architecture)
3. [UI and State Machine](#3-ui-and-state-machine)
4. [File-by-File Wiring Plan](#4-file-by-file-wiring-plan)
5. [TypeScript Reference Examples](#5-typescript-reference-examples)
6. [Safety and Cleanup Rules](#6-safety-and-cleanup-rules)
7. [Test Plan](#7-test-plan)
8. [Open Questions](#8-open-questions)
9. [Final Frontend Implementation Checklist](#9-final-frontend-implementation-checklist)

---

## 1. Backend Contract Summary

All events verified against `server/sockets/events.py`, `server/joystick_controller.py`,
`server/control_arbiter.py`, `server/manual_control_gateway.py`, `server/config.py`,
`server/auth.py`, `server/models.py`, `server/main.py`.

### 1.1 Authentication

Every joystick Socket.IO event MUST include `"auth"` in the event data payload:

```json
{ "auth": "<shared-secret-token>", ... }
```

- Source: `server/auth.py:75-81` — `check_socket_token(token)` validates via `secrets.compare_digest`
- Token lives at `~/.rover_token` on Jetson
- Auth disabled when `ROVER_DISABLE_AUTH=1`
- Rejection: server emits `socket_error` event with `{"reason": "unauthorised"}` to requesting sid
- Source: `server/sockets/events.py:23-30`

### 1.2 Events — Exact Payloads

#### `joystick_acquire` (client → server)

**Required fields** (from `server/joystick_controller.py:122-128`, `_required_str` at 413-417):

| Field | Type | Validation |
|-------|------|------------|
| `auth` | string | Must match shared secret |
| `session_id` | string | Client-generated UUID (required, non-empty) |
| `client_monotonic_ms` | int | Client monotonic timestamp for ordering (from model `JoystickAcquireRequest`, `server/models.py:58-61`) |

Note: The Pydantic model `JoystickAcquireRequest` defines `client_monotonic_ms` as `int` but
`joystick_controller.acquire()` only reads `session_id` from the dict via `_required_str()`.
The `client_monotonic_ms` field in the model is present but not validated at acquire time
by the controller — only `session_id` is used. Include it to match the model schema.

**Success response** — server emits `joystick_acquired` to requesting sid (from
`server/joystick_controller.py:174-183`):

```json
{
  "type": "joystick_acquired",
  "lease_id": "32-char-hex-server-uuid",
  "state": "active",
  "command_rate_hz": 20.0,
  "server_stop_timeout_ms": 300,
  "gateway_stop_timeout_ms": 400,
  "max_throttle": 0.15,
  "max_steering": 0.50
}
```

- `command_rate_hz` = `1.0 / _min_command_interval_s` — use this as the target command send rate
- `server_stop_timeout_ms` = `JOYSTICK_SERVER_STOP_TIMEOUT_S * 1000` (default 300 ms)
- `gateway_stop_timeout_ms` = `JOYSTICK_GATEWAY_STALE_TIMEOUT_S * 1000` (default 400 ms)
- `max_throttle` = `JOYSTICK_MAX_ABS_THROTTLE` (default 0.15)
- `max_steering` = `JOYSTICK_MAX_ABS_STEERING` (default 0.50)

**Freeze these values from the acquire response.** The backend clamps throttle/steering to these
limits — apply them to the joystick UI max range.

**Error response** — server emits `joystick_error` to requesting sid (from
`server/sockets/events.py:33-36`):

```json
{
  "type": "joystick_error",
  "code": "<error_code>",
  "message": "<human-readable>"
}
```

#### `joystick_command` (client → server)

**Required fields** (from `server/joystick_controller.py:420-438` — `_parse_command`):

```json
{
  "auth": "<token>",
  "session_id": "<client-uuid>",
  "lease_id": "<server-uuid-from-acquire>",
  "sequence": 145,
  "client_monotonic_ms": 3892231,
  "deadman": true,
  "throttle": 0.35,
  "steering": -0.20
}
```

| Field | Type | Validation |
|-------|------|------------|
| `auth` | string | Token check |
| `session_id` | string | Must match owner session_id |
| `lease_id` | string | Must match current lease_id |
| `sequence` | int | Must be strictly > last accepted sequence |
| `client_monotonic_ms` | int | Must be ≥ last accepted |
| `deadman` | bool | If false → throttle/steering forced to 0.0, state → HELD |
| `throttle` | float | ∈ [-1.0, 1.0], must be finite |
| `steering` | float | ∈ [-1.0, 1.0], must be finite |

**Success:** Commands do NOT receive individual ack events. Success is silent. If the command
is accepted, no event is emitted back (confirmed from `server/sockets/events.py:133-145` —
only errors emit).

**Error:** `joystick_error` event (same format as above).

**State transition on deadman=false:** The backend transitions to `HELD` state (from
`server/joystick_controller.py:204-208`). In HELD state, commands are still accepted but
with throttle=steering=0 forced. The next `deadman=true` command transitions back to `ACTIVE`.

#### `joystick_release` (client → server)

**Required fields** (from `server/sockets/events.py:147-166`,
`server/joystick_controller.py:229-253`):

```json
{
  "auth": "<token>",
  "session_id": "<client-uuid>",
  "lease_id": "<server-uuid>"
}
```

**Success response** — server emits `joystick_released` to all clients (broadcast):

```json
{
  "type": "joystick_released",
  "state": "inactive",
  "reason": "explicit"
}
```

Possible release reasons from backend: `"explicit"`, `"disconnect"`, `"lease_timeout"`,
`"fcu_disconnected"`, `"estop"`, `"shutdown"`, `"forced"`, `"acquire_failed"`,
`"server_timeout_neutral"`.

**Note:** `joystick_release` handler in events.py does NOT emit to `to=sid` — it emits
broadcast. But `joystick_ctrl.release()` returns the result dict, which is what gets
emitted. Check `server/sockets/events.py:166`: `await sio.emit("joystick_released", result)` —
no `to=sid`, so this is a broadcast.

### 1.3 Disconnect Behaviour

From `server/sockets/events.py:48-58`:

When a socket disconnects, if that sid owns the joystick lease:
1. `joystick_ctrl.release(sid, reason="disconnect")` is called
2. `joystick_released` event is broadcast

### 1.4 Telemetry Payload (Joystick Fields)

From `server/main.py:393-394` and `server/joystick_controller.py:278-297` — the telemetry
push loop (10 Hz) merges `joystick_ctrl.snapshot()` into the telemetry event.

Joystick-related telemetry fields:

| Field | Type | Authoritative? | Notes |
|-------|------|----------------|-------|
| `joystick_state` | `"inactive"`\|`"acquiring"`\|`"active"`\|`"held"`\|`"releasing"` | ✅ Authoritative state | Backend truth |
| `joystick_active` | bool | ✅ Authoritative | `state ∈ {ACTIVE, HELD}` |
| `joystick_owner_present` | bool | ✅ Authoritative | `session_id is not None` |
| `joystick_has_lease` | bool | ✅ Authoritative | `lease_id is not None` |
| `joystick_last_valid_cmd_age_ms` | float\|null | Diagnostic | `null` when no cmd received |
| `joystick_deadman` | bool | Diagnostic | Last deadman state |
| `joystick_commanded_throttle` | float | Diagnostic | Last scaled value |
| `joystick_commanded_steering` | float | Diagnostic | Last scaled value |
| `joystick_stop_reason` | string\|null | ✅ Authoritative | `"deadman_released"`, `"server_timeout_neutral"`, etc. |
| `control_owner` | `"idle"`\|... | ✅ Authoritative | From ControlArbiter |
| `joystick_owned` | bool | ✅ Authoritative | True when arbiter owner is joystick |
| `gateway_active` | bool | Diagnostic | Gateway worker active |
| `gateway_command_age_ms` | float\|null | Diagnostic | Age of last accepted command at gateway |
| `gateway_last_send_age_ms` | float\|null | Diagnostic | Age of last published MANUAL_CONTROL frame |
| `gateway_last_frame` | `{x,y,z,r,buttons}` | Diagnostic | Last MAVLink frame contents |
| `gateway_last_sent_neutral` | bool | Diagnostic | Last sent was neutral |
| `transport` | `"mavros"`\|`"pymavlink"` | Diagnostic | Active transport |
| `transport_healthy` | bool | Diagnostic | Transport health check |
| `transport_error` | string | Diagnostic | Health failure reason |
| `armed` | bool | ✅ Authoritative | From PX4 state |
| `mode` | string | ✅ Authoritative | PX4 mode (expect `"MANUAL"` when active) |
| `connected` | bool | ✅ Authoritative | FCU connection |

**Usage in frontend:**
- `joystick_state` — use for UI state display, reconciliation after reconnect
- `joystick_active` — gate for mission start disable
- `joystick_has_lease` — verify local lease is still valid
- `joystick_stop_reason` — show operator why joystick stopped
- `joystick_last_valid_cmd_age_ms` — display command age in UI
- `control_owner` — cross-reference with local state
- `transport_healthy` — show transport health indicator
- `armed` / `mode` — show FCU state in joystick panel

### 1.5 Error Codes (Every Backend Error)

From `server/joystick_controller.py` (class `JoystickError`), `server/control_arbiter.py`
(class `ControlArbiterError`), and the Socket.IO handler layer:

| Code | Trigger | Frontend Action |
|------|---------|-----------------|
| `manual_control_disabled` | `JOYSTICK_MANUAL_ENABLED=0` | Show "Manual control disabled by deployment". Do not retry. |
| `malformed` | Missing/invalid field type | Show "Invalid request format". Fix payload. |
| `mode_unavailable` | FCU not ready, MANUAL switch failed, not confirmed, or not in MANUAL | Show "MANUAL mode unavailable". Retry acquire. Clear lease. |
| `fcu_disconnected` | FCU not connected | Show "FCU not connected". Retry acquire later. Clear lease. |
| `not_armed` | Vehicle not armed | Show "Vehicle must be armed first". Retry after arm. Clear lease. |
| `not_owner` | Wrong SID/session_id/lease_id | Show "Not the joystick owner". Clear local lease. Re-acquire required. |
| `mission_active` | Mission is running or transitioning | Show "Mission is active — cannot acquire joystick". Wait for mission end. |
| `joystick_active` | Joystick already owned by another client | Show "Joystick is already in use". Wait. |
| `acquire_cancelled` | Acquire cancelled mid-transition | Show "Acquire cancelled". Retry acquire. Clear lease. |
| `unavailable` | Joystick controller not initialized | Show "Joystick unavailable". Retry later. |
| `lease_inactive` | State not ACTIVE/HELD when command sent | Clear local lease. Stop command timer. Re-acquire required. |
| `transport_unavailable` | MAVROS/pymavlink transport unhealthy | Show "Manual control transport unhealthy". Stop sending. Wait for health. |
| `out_of_order` | Sequence not strictly increasing | Show "Command rejected: out of order". Check sequence counter. |
| `replay` | `client_monotonic_ms` moved backward | Show "Command rejected: replay detected". Reset monotonic clock. |
| `rate_exceeded` | Sending faster than configured rate | Throttle sender. Check command rate. |
| `nan_value` | Non-finite throttle or steering | Fix math. Force neutral locally. |
| `out_of_range` | Values outside [-1.0, 1.0] | Clamp values in client before sending. |
| `auth_failed` | (via `socket_error`) Invalid/missing token | Show "Authentication failed". Do not retry without fixing token. |
| `acquire_failed` | (in arbiter, during exception rollback) | Show "Acquire failed". Retry acquire. |

### 1.6 Configuration Defaults (Backend Truth)

From `server/config.py:129-156` (all configurable via environment variables):

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `JOYSTICK_MANUAL_ENABLED` | `"0"` (disabled) | Must be `"1"` in deployment |
| `JOYSTICK_COMMAND_RATE_HZ` | `20.0` | Frontend should send at this rate |
| `JOYSTICK_GATEWAY_RATE_HZ` | `50.0` | Gateway publishes MANUAL_CONTROL at this rate |
| `JOYSTICK_SERVER_STOP_TIMEOUT_S` | `0.30` | Server zeroes command if >300ms since last valid |
| `JOYSTICK_GATEWAY_STALE_TIMEOUT_S` | `0.40` | Gateway goes neutral if >400ms since last receive |
| `JOYSTICK_LEASE_REVOKE_TIMEOUT_S` | `2.0` | Server revokes lease if >2s since last valid command |
| `JOYSTICK_LEASE_EXPIRY_S` | `30.0` | Server expires lease if >30s since last valid command |
| `JOYSTICK_NEUTRAL_PRESTREAM_S` | `0.20` | Neutral pre-stream before mode switch during acquire |
| `JOYSTICK_MODE_CONFIRM_TIMEOUT_S` | `3.0` | Max wait for MANUAL mode confirmation |
| `JOYSTICK_MAX_ABS_THROTTLE` | `0.15` | Max throttle clamp (acquire response reports this) |
| `JOYSTICK_MAX_ABS_STEERING` | `0.50` | Max steering clamp (acquire response reports this) |

---

## 2. Frontend Architecture

### 2.1 Current State

The React Native frontend (`DXF_Three_Wheel`) is a single-component monolith in
`App.tsx` (~10K lines) with:

- **Socket.IO client:** `socket.io-client` v4.8.3, connected as a single shared connection
- **Connection:** `io(target, { transports: ["websocket"], timeout: 20000, forceNew: true })`
- **Socket stored:** `const [socket, setSocket] = useState<Socket | null>(null)`
- **Telemetry:** Listens on `telemetry` event, merges into `telemetrySnapshot` state
- **Mission status:** Listens on `mission_status` event
- **Arm:** REST `POST /api/arm` (not Socket.IO)
- **Set mode:** REST `POST /api/set_mode` (not Socket.IO)
- **E-stop:** REST `POST /api/estop` (not Socket.IO)
- **No AppState handling:** No `react-native` `AppState` listener
- **No joystick:** No virtual joystick UI or state at all

### 2.2 Integration Strategy

**Use the existing shared Socket.IO connection.** The backend's Socket.IO server handles
all events (mission + joystick) on the same connection. Register joystick event listeners
alongside the existing `telemetry` and `mission_status` listeners.

**Auth token:** The backend checks `data.auth` on every control event. The frontend must
either:
- Have the operator enter the token at connect time, or
- Read it from a stored config, or
- Use the REST auth header pattern (if the backend also supports `X-Rover-Token` in Socket.IO)

Currently, the frontend does NOT send `auth` on any Socket.IO event — only REST calls go
through the `X-Rover-Token` header (if auth is enabled). **The joystick wiring MUST include
`auth` in every Socket.IO joystick event payload.**

**Shared infrastructure to preserve:**
- `socket` state variable — already holds the connection
- `telemetrySnapshot` — extend to include new joystick fields
- `systemHealth` — extend with joystick state
- `apiBaseUrl` — already set on connect

### 2.3 Recommended Patterns

Do NOT introduce Redux, Zustand, or any new state management library. The existing
`useState` + `useRef` + `useCallback` pattern in App.tsx is sufficient. For joystick,
add a lightweight service module and a custom hook.

---

## 3. UI and State Machine

### 3.1 Frontend States

```
DISABLED        — initial state, manual control disabled by deployment or no socket
DISCONNECTED    — socket not connected
AVAILABLE       — socket connected, FCU connected, armed, no mission active, ready to acquire
ACQUIRING       — acquire sent, waiting for response
ACTIVE          — lease held, deadman pressed, sending commands
HELD            — lease held, deadman released, sending neutral commands
RELEASING       — explicit release sent, waiting for response
BLOCKED_BY_MISSION — mission active, cannot acquire
ERROR           — unrecoverable error
```

### 3.2 State Transitions

| From | Event | To | Action |
|------|-------|----|--------|
| DISABLED | Socket connects + `control_owner=idle` | AVAILABLE | Enable acquire button |
| DISABLED | `joystick_acquired` telemetry from another client | DISABLED | Show "in use by other operator" |
| AVAILABLE | Mission starts (telemetry) | BLOCKED_BY_MISSION | Disable acquire, show reason |
| AVAILABLE | Socket disconnects | DISCONNECTED | Clear state |
| AVAILABLE | FCU disconnects (`connected=false`) | DISABLED | Disable acquire |
| AVAILABLE | Vehicle disarms | DISABLED | Disable acquire |
| AVAILABLE | User presses acquire | ACQUIRING | Send `joystick_acquire` |
| ACQUIRING | `joystick_acquired` received | ACTIVE | Store lease, start deadman listener |
| ACQUIRING | `joystick_error` received | AVAILABLE | Show error, allow retry |
| ACQUIRING | Socket disconnects | DISCONNECTED | Clear pending acquire |
| ACTIVE | Deadman pressed (`deadman=true`) | ACTIVE | Send commands with deadman=true |
| ACTIVE | Deadman released (`deadman=false`) | HELD | Send one neutral+deadman=false frame, keep lease |
| HELD | Deadman pressed again | ACTIVE | Resume sending non-zero commands |
| ACTIVE/HELD | Command timeout (>2s no response) | ERROR | Clear lease, stop sender |
| ACTIVE/HELD | Explicit release | RELEASING | Stop sender, send release |
| RELEASING | `joystick_released` received | AVAILABLE | Clear local lease, enable acquire |
| ACTIVE/HELD/ACQUIRING/RELEASING | Socket disconnect | DISCONNECTED | Clear lease, stop sender, force neutral |
| ACTIVE/HELD/ACQUIRING | App background | DISCONNECTED | Send neutral, send release, stop sender |
| ACTIVE/HELD | `joystick_stop_reason` appears (server timeout) | ERROR | Show reason, clear local lease |
| ACTIVE/HELD/ACQUIRING | E-stop pressed | DISABLED | Send release (best-effort), clear lease |
| ACTIVE/HELD | Backend telemetry shows `joystick_state=inactive` | AVAILABLE | Clear local lease (lease revoked by backend) |
| BLOCKED_BY_MISSION | Mission completes (telemetry) | AVAILABLE | Enable acquire |
| DISCONNECTED | Socket reconnects | AVAILABLE | Reconcile with telemetry, do NOT auto-acquire |
| ERROR | User dismisses error | AVAILABLE | Clear error, allow fresh acquire |
| App foreground (from background) | Telemetry received | Reconcile | Do NOT auto-acquire. Wait for explicit user action. |

### 3.3 Joystick UI Requirements

#### Axis Conventions

```
throttle:  -1.0 = full reverse
            0.0 = neutral/stop
           +1.0 = full forward

steering:  -1.0 = full left
            0.0 = straight
           +1.0 = full right
```

These are **normalized operator intent** values. The backend owns all physical scaling.
The frontend MUST NOT apply physical speed scaling, PWM conversion, or velocity mapping.

#### UI Model

**Dead-man control (hold-to-drive):**

A dedicated press-and-hold dead-man button. The operator must keep this pressed
for any non-zero throttle to reach the rover.

Behaviour:

| Action | Result |
|--------|--------|
| Screen opens | No motion. Neutral intent. |
| Acquire received | No motion. Must press dead-man first. |
| Dead-man pressed | Commands flow. Throttle/steering values sent with `deadman=true`. |
| Dead-man released | Immediate `deadman=false` frame sent. Throttle/steering forced to 0.0 locally. |
| Joystick thumb released | Returns to neutral (0.0, 0.0). Next command sends neutral. |
| Touch ends on joystick | Same as thumb release — return to neutral. |

**Joystick thumb behaviour:**

- Return-to-center on release
- Visual neutral position clearly marked
- Optional: slight haptic feedback at neutral crossing
- Dead zone: 2-3% of axis range (configurable, e.g. `|value| < 0.03` → clamp to 0.0)
- Optional response curve: `value = sign(value) * |value|^exponent` (default exponent 1.0 = linear, 1.5 = softer center, 2.0 = very soft center). Apply curve after dead zone.

**Forward/reverse mapping:**

- Thumb up from center → positive throttle (forward)
- Thumb down from center → negative throttle (reverse)
- Thumb left from center → negative steering (turn left)
- Thumb right from center → positive steering (turn right)

**Composite joystick:**

A single thumb pad controlling both throttle (vertical axis) and steering (horizontal axis)
simultaneously. OR separate slider/thumb controls for each axis. The single dual-axis pad
is recommended for one-thumb operation.

#### Max Range Clamping

On `joystick_acquired` response, read `max_throttle` and `max_steering`. Clamp the
joystick output to these ranges before sending:

```typescript
const clampedThrottle = Math.max(-maxThrottle, Math.min(maxThrottle, rawThrottle));
const clampedSteering = Math.max(-maxSteering, Math.min(maxSteering, rawSteering));
```

These values represent the backend's current physical limits. Display them to the operator
in the joystick HUD (e.g. "Throttle range: ±15%", "Steering range: ±50%").

---

## 4. File-by-File Wiring Plan

### Current Project Structure

```
DXF_Three_Wheel/
├── App.tsx                          (monolith, ~10K lines)
├── src/
│   ├── api/
│   │   ├── missionApi.ts            (REST mission calls)
│   │   ├── missionContract.ts       (mission validation)
│   │   └── pathApi.ts               (REST path calls)
│   ├── types/
│   │   └── plan.ts                  (TelemetrySnapshot, PlanLine types)
│   ├── components/
│   │   ├── BoundaryEditor.tsx
│   │   ├── GeometryViewport.tsx
│   │   ├── LeftSidebar.tsx
│   │   ├── MapView.tsx
│   │   └── TopHeader.tsx
│   └── screens/
│       ├── HomeScreen.tsx
│       ├── PlanScreen.tsx
│       └── TemplatesPage.tsx
```

### Files to Create

#### File 1: `src/types/joystick.ts` — NEW

**Responsibility:** All TypeScript types for joystick protocol.

**Public interface:**
```typescript
export interface JoystickAcquireRequest { ... }
export interface JoystickAcquiredResponse { ... }
export interface JoystickCommandRequest { ... }
export interface JoystickReleaseRequest { ... }
export interface JoystickReleasedResponse { ... }
export interface JoystickErrorEvent { ... }
export interface JoystickTelemetryFields { ... }
export type JoystickState = "inactive" | "acquiring" | "active" | "held" | "releasing";
export type FrontendJoystickState = "DISABLED" | "DISCONNECTED" | "AVAILABLE" | ...;
export type JoystickErrorCode = "manual_control_disabled" | "malformed" | ...;
```

#### File 2: `src/services/roverSocket.ts` — MODIFY (extract from App.tsx)

**Responsibility:** Socket.IO connection management and event bus. Currently embedded in
App.tsx. Extract to a service that exposes the shared socket instance and provides
type-safe event registration.

**Important state:**
- Socket instance (singleton)
- Connection status
- Auth token

**Existing pattern:** `io(target, { transports: ["websocket"], timeout: 20000, forceNew: true })`
— preserve this. Add auth token attachment for joystick events.

**If extraction is too invasive:** Add joystick event handling inline in App.tsx, following
the exact same pattern used for `telemetry` and `mission_status` listeners (lines 930-1026).
This avoids restructuring.

#### File 3: `src/hooks/useVirtualJoystick.ts` — NEW

**Responsibility:** Encapsulates all joystick state, acquire/command/release flow,
fixed-rate sender, dead-man tracking, sequence counter, and cleanup.

**Public interface:**
```typescript
export function useVirtualJoystick(socket: Socket | null, authToken: string) {
  return {
    // State
    state: FrontendJoystickState,
    error: JoystickErrorEvent | null,
    leaseId: string | null,
    sessionId: string | null,
    maxThrottle: number,
    maxSteering: number,
    commandRateHz: number,

    // Actions
    acquire: () => Promise<void>,
    release: () => Promise<void>,
    setIntent: (throttle: number, steering: number) => void,
    setDeadman: (pressed: boolean) => void,

    // Telemetry fields (from parent)
    lastCmdAgeMs: number | null,
    stopReason: string | null,
  };
}
```

**Important state (all in-memory only, never persisted):**
- `sessionId: string` — generated once per app launch (UUID v4)
- `leaseId: string | null` — from acquire response, cleared on release/error/disconnect
- `sequence: number` — reset to 0 on each new acquire, incremented per transmitted command
- `deadman: boolean` — current deadman button state
- `latestThrottle: number` — latest normalized throttle intent (useRef)
- `latestSteering: number` — latest normalized steering intent (useRef)
- `commandTimer: NodeJS.Timeout | null` — fixed-rate sender interval

**Event subscriptions:**
- `joystick_acquired` — handle acquire success
- `joystick_error` — handle all joystick errors
- `joystick_released` — handle release broadcast
- `telemetry` — reconcile state from backend
- Socket `disconnect` — cleanup

**Cleanup logic:**
- On unmount: send release, stop timer, clear lease
- On socket disconnect: stop timer, clear lease, mark DISCONNECTED
- On app background: send release, stop timer, clear lease

#### File 4: `src/components/VirtualJoystick.tsx` — NEW

**Responsibility:** Dual-axis thumb joystick UI using PanResponder or
react-native-gesture-handler.

**Props:**
```typescript
interface VirtualJoystickProps {
  onIntentChange: (throttle: number, steering: number) => void;
  disabled: boolean;
  maxThrottle: number;
  maxSteering: number;
  deadZone?: number;
}
```

**Behaviour:**
- Pan gesture tracks thumb position relative to joystick center
- Vertical component → throttle, horizontal → steering
- On release: call `onIntentChange(0, 0)` (return to neutral)
- Apply dead zone before callback
- Visual: two concentric circles (outer = boundary, inner = dead zone)
- Thumb dot follows touch, clamped to outer boundary
- Thumb dot color changes when disabled

#### File 5: `src/components/DeadmanButton.tsx` — NEW

**Responsibility:** Press-and-hold dead-man control button.

**Props:**
```typescript
interface DeadmanButtonProps {
  onPress: () => void;
  onRelease: () => void;
  disabled: boolean;
  active: boolean;
}
```

**Behaviour:**
- `onPressIn` → call `onPress`
- `onPressOut` → call `onRelease`
- Visual: pulsing border when active (optional), solid color when pressed
- Clear label: "HOLD TO DRIVE"
- Red color when pressed, grey when released

#### File 6: `src/screens/ManualControlScreen.tsx` — NEW

**Responsibility:** Screen containing joystick, dead-man button, telemetry HUD,
and control buttons (acquire, release, arm, mode).

**Layout:**
```
┌──────────────────────────┐
│   Manual Control         │  ← Header with connection state
├──────────────────────────┤
│ [Armed] [MANUAL] [20Hz]  │  ← Status bar
│ Cmd age: 45ms            │
├──────────────────────────┤
│       ┌─────────┐        │
│       │         │        │
│       │  JOYSTICK│        │  ← VirtualJoystick
│       │         │        │
│       └─────────┘        │
├──────────────────────────┤
│  Throttle: +0.12         │  ← Numeric readouts
│  Steering: -0.08         │
├──────────────────────────┤
│  [═══ HOLD TO DRIVE ═══] │  ← DeadmanButton
├──────────────────────────┤
│  [Acquire]  [Release]    │  ← Control buttons
│  [E-STOP]                │
└──────────────────────────┘
```

**Navigation:** Add to bottom tabs or as a modal screen from the "Swozi" tab.

#### File 7: `src/utils/joystickMath.ts` — NEW

**Responsibility:** Pure math functions for axis clamping, dead zone, response curve.

**Public interface:**
```typescript
export function applyDeadZone(value: number, threshold: number): number;
export function applyResponseCurve(value: number, exponent: number): number;
export function clampAxis(value: number, min: number, max: number): number;
export function processAxis(raw: number, deadZone: number, curve: number, min: number, max: number): number;
```

#### File 8: `src/types/plan.ts` — MODIFY

**Responsibility:** Extend `TelemetrySnapshot` with joystick telemetry fields.

Add:
```typescript
export interface TelemetrySnapshot {
  // ... existing fields ...
  joystick_state?: string | null;
  joystick_active?: boolean | null;
  joystick_owner_present?: boolean | null;
  joystick_has_lease?: boolean | null;
  joystick_last_valid_cmd_age_ms?: number | null;
  joystick_deadman?: boolean | null;
  joystick_commanded_throttle?: number | null;
  joystick_commanded_steering?: number | null;
  joystick_stop_reason?: string | null;
  control_owner?: string | null;
  joystick_owned?: boolean | null;
  gateway_active?: boolean | null;
  gateway_command_age_ms?: number | null;
  gateway_last_send_age_ms?: number | null;
  gateway_last_frame?: { x: number; y: number; z: number; r: number; buttons: number } | null;
  gateway_last_sent_neutral?: boolean | null;
  transport?: string | null;
  transport_healthy?: boolean | null;
  transport_error?: string | null;
}
```

#### File 9: `App.tsx` — MODIFY (integration points)

**Changes needed:**

1. **Import hook:** `import { useVirtualJoystick } from "./src/hooks/useVirtualJoystick";`

2. **Add auth token state:**
```typescript
const [authToken] = useState<string>(() => {
  // Read from stored config or environment
  // Fallback: empty string (will cause auth_failed if backend requires auth)
  return "";
});
```

3. **Call hook at top level:**
```typescript
const joystick = useVirtualJoystick(socket, authToken);
```

4. **Register joystick event listeners** (alongside existing telemetry/mission_status listeners,
   around App.tsx line 926-1026):
```typescript
nextSocket.on("joystick_acquired", (data) => { ... });
nextSocket.on("joystick_error", (data) => { ... });
nextSocket.on("joystick_released", (data) => { ... });
```

5. **Extend telemetry handler** to pass joystick fields to the hook.

6. **Add AppState listener** (new addition, currently absent):
```typescript
import { AppState } from "react-native";

useEffect(() => {
  const subscription = AppState.addEventListener("change", (nextAppState) => {
    if (nextAppState === "background" || nextAppState === "inactive") {
      joystick.handleBackground();
    } else if (nextAppState === "active") {
      joystick.handleForeground();
    }
  });
  return () => subscription.remove();
}, [joystick]);
```

7. **Add navigation route** for ManualControlScreen.

8. **Mission start guard:** Disable mission start button when `joystick.joystickActive` is
   true (from telemetry `joystick_active` field).

---

## 5. TypeScript Reference Examples

### 5.1 Type Definitions

```typescript
// src/types/joystick.ts

export type JoystickErrorCode =
  | "manual_control_disabled"
  | "malformed"
  | "mode_unavailable"
  | "fcu_disconnected"
  | "not_armed"
  | "not_owner"
  | "mission_active"
  | "joystick_active"
  | "acquire_cancelled"
  | "unavailable"
  | "lease_inactive"
  | "transport_unavailable"
  | "out_of_order"
  | "replay"
  | "rate_exceeded"
  | "nan_value"
  | "out_of_range";

export interface JoystickAcquireRequest {
  auth: string;
  session_id: string;
  client_monotonic_ms: number;
}

export interface JoystickAcquiredResponse {
  type: "joystick_acquired";
  lease_id: string;
  state: "active";
  command_rate_hz: number;
  server_stop_timeout_ms: number;
  gateway_stop_timeout_ms: number;
  max_throttle: number;
  max_steering: number;
}

export interface JoystickCommandRequest {
  auth: string;
  session_id: string;
  lease_id: string;
  sequence: number;
  client_monotonic_ms: number;
  deadman: boolean;
  throttle: number;
  steering: number;
}

export interface JoystickReleaseRequest {
  auth: string;
  session_id: string;
  lease_id: string;
}

export interface JoystickReleasedResponse {
  type: "joystick_released";
  state: "inactive";
  reason: string;
}

export interface JoystickErrorEvent {
  type: "joystick_error";
  code: JoystickErrorCode;
  message: string;
}

export type FrontendJoystickState =
  | "DISABLED"
  | "DISCONNECTED"
  | "AVAILABLE"
  | "ACQUIRING"
  | "ACTIVE"
  | "HELD"
  | "RELEASING"
  | "BLOCKED_BY_MISSION"
  | "ERROR";

export interface JoystickTelemetryFields {
  joystick_state?: string | null;
  joystick_active?: boolean | null;
  joystick_owner_present?: boolean | null;
  joystick_has_lease?: boolean | null;
  joystick_last_valid_cmd_age_ms?: number | null;
  joystick_deadman?: boolean | null;
  joystick_commanded_throttle?: number | null;
  joystick_commanded_steering?: number | null;
  joystick_stop_reason?: string | null;
  control_owner?: string | null;
  joystick_owned?: boolean | null;
  gateway_active?: boolean | null;
  gateway_command_age_ms?: number | null;
  gateway_last_send_age_ms?: number | null;
  transport_healthy?: boolean | null;
}
```

### 5.2 Socket Event Registration and Cleanup

```typescript
// Inside connect handler (or useEffect) in App.tsx — register alongside
// existing telemetry + mission_status listeners

// Called once when socket connects:
function registerJoystickListeners(socket: Socket): () => void {
  const onAcquired = (data: unknown) => {
    if (data && typeof data === "object") {
      joystickRef.current?.onAcquired(data as JoystickAcquiredResponse);
    }
  };

  const onError = (data: unknown) => {
    if (data && typeof data === "object") {
      const err = data as JoystickErrorEvent;
      joystickRef.current?.onError(err);
      console.warn(`[JOYSTICK] Error: ${err.code} — ${err.message}`);
    }
  };

  const onReleased = (data: unknown) => {
    if (data && typeof data === "object") {
      joystickRef.current?.onReleased(data as JoystickReleasedResponse);
    }
  };

  const onDisconnect = (_reason: string) => {
    joystickRef.current?.onSocketDisconnect();
  };

  socket.on("joystick_acquired", onAcquired);
  socket.on("joystick_error", onError);
  socket.on("joystick_released", onReleased);
  socket.on("disconnect", onDisconnect);

  // Return cleanup function
  return () => {
    socket.off("joystick_acquired", onAcquired);
    socket.off("joystick_error", onError);
    socket.off("joystick_released", onReleased);
    socket.off("disconnect", onDisconnect);
  };
}
```

### 5.3 Acquire Flow

```typescript
// src/hooks/useVirtualJoystick.ts

async function acquire(): Promise<void> {
  if (!socketRef.current || !socketRef.current.connected) {
    setError({ type: "joystick_error", code: "unavailable",
                message: "Socket not connected" });
    return;
  }

  setState("ACQUIRING");
  setError(null);

  const payload: JoystickAcquireRequest = {
    auth: authTokenRef.current,
    session_id: sessionIdRef.current,
    client_monotonic_ms: Date.now(),
  };

  socketRef.current.emit("joystick_acquire", payload);
  // Response comes via 'joystick_acquired' or 'joystick_error' event
}

function onAcquired(data: JoystickAcquiredResponse): void {
  leaseIdRef.current = data.lease_id;
  sequenceRef.current = 0;           // Reset sequence per new lease
  maxThrottleRef.current = data.max_throttle;
  maxSteeringRef.current = data.max_steering;
  commandRateHzRef.current = data.command_rate_hz;

  setLeaseId(data.lease_id);
  setMaxThrottle(data.max_throttle);
  setMaxSteering(data.max_steering);
  setCommandRateHz(data.command_rate_hz);
  setState("ACTIVE");  // Ready, but deadman not pressed yet
}
```

### 5.4 Fixed-Rate Command Sender

```typescript
// src/hooks/useVirtualJoystick.ts

const latestThrottleRef = useRef<number>(0);
const latestSteeringRef = useRef<number>(0);
const deadmanRef = useRef<boolean>(false);
const commandTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);
const sequenceRef = useRef<number>(0);
const socketRef = useRef<Socket | null>(null);
const startMonotonicMs = useRef<number>(Date.now());

function startCommandSender(rateHz: number): void {
  stopCommandSender();

  const intervalMs = Math.round(1000 / rateHz);

  commandTimerRef.current = setInterval(() => {
    const sock = socketRef.current;
    if (!sock || !sock.connected) return;

    const lease = leaseIdRef.current;
    const session = sessionIdRef.current;
    if (!lease || !session) return;

    const seq = ++sequenceRef.current;  // Pre-increment: 1, 2, 3, ...

    const payload: JoystickCommandRequest = {
      auth: authTokenRef.current,
      session_id: session,
      lease_id: lease,
      sequence: seq,
      client_monotonic_ms: Date.now() - startMonotonicMs.current,
      deadman: deadmanRef.current,
      throttle: deadmanRef.current ? latestThrottleRef.current : 0.0,
      steering: deadmanRef.current ? latestSteeringRef.current : 0.0,
    };

    sock.emit("joystick_command", payload);
  }, intervalMs);
}

function stopCommandSender(): void {
  if (commandTimerRef.current !== null) {
    clearInterval(commandTimerRef.current);
    commandTimerRef.current = null;
  }
}

// Called by UI on every joystick movement:
function setIntent(throttle: number, steering: number): void {
  latestThrottleRef.current = throttle;
  latestSteeringRef.current = steering;
}

// Called by UI on dead-man press/release:
function setDeadman(pressed: boolean): void {
  deadmanRef.current = pressed;
  if (!pressed) {
    // Immediately send one deadman=false + neutral frame
    sendImmediateNeutralDeadmanFrame();
    setState("HELD");
  } else {
    setState("ACTIVE");
  }
}

function sendImmediateNeutralDeadmanFrame(): void {
  const sock = socketRef.current;
  const lease = leaseIdRef.current;
  const session = sessionIdRef.current;
  if (!sock || !sock.connected || !lease || !session) return;

  const seq = ++sequenceRef.current;
  const payload: JoystickCommandRequest = {
    auth: authTokenRef.current,
    session_id: session,
    lease_id: lease,
    sequence: seq,
    client_monotonic_ms: Date.now() - startMonotonicMs.current,
    deadman: false,
    throttle: 0.0,
    steering: 0.0,
  };
  sock.emit("joystick_command", payload);
}
```

### 5.5 AppState Background Release

```typescript
// src/hooks/useVirtualJoystick.ts

import { AppState, AppStateStatus } from "react-native";

function useAppStateHandler(
  handleBackground: () => void,
  handleForeground: () => void,
): void {
  useEffect(() => {
    const subscription = AppState.addEventListener(
      "change",
      (nextAppState: AppStateStatus) => {
        if (nextAppState === "background" || nextAppState === "inactive") {
          handleBackground();
        } else if (nextAppState === "active") {
          handleForeground();
        }
      },
    );
    return () => {
      subscription.remove();
    };
  }, [handleBackground, handleForeground]);
}

function handleBackground(): void {
  // 1. Force neutral intent
  latestThrottleRef.current = 0;
  latestSteeringRef.current = 0;
  deadmanRef.current = false;
  setState("DISCONNECTED");

  // 2. Send deadman=false neutral frame if connected
  sendImmediateNeutralDeadmanFrame();

  // 3. Send release if leased
  if (leaseIdRef.current && socketRef.current?.connected) {
    const releasePayload: JoystickReleaseRequest = {
      auth: authTokenRef.current,
      session_id: sessionIdRef.current,
      lease_id: leaseIdRef.current,
    };
    socketRef.current.emit("joystick_release", releasePayload);
  }

  // 4. Stop command timer
  stopCommandSender();

  // 5. Clear local lease
  leaseIdRef.current = null;
  sequenceRef.current = 0;
  setLeaseId(null);
  setError(null);
  // State remains DISCONNECTED
}

function handleForeground(): void {
  // Do NOT auto-resume. Wait for telemetry reconciliation.
  // Operator must explicitly press acquire again.
  // State stays DISCONNECTED until user action.
}
```

### 5.6 Socket Disconnect Cleanup

```typescript
function onSocketDisconnect(): void {
  // Stop sending immediately
  stopCommandSender();

  // Force neutral locally
  latestThrottleRef.current = 0;
  latestSteeringRef.current = 0;
  deadmanRef.current = false;

  // Clear local lease — it's invalid on reconnect
  leaseIdRef.current = null;
  sequenceRef.current = 0;
  setLeaseId(null);
  setState("DISCONNECTED");
  setError({
    type: "joystick_error",
    code: "unavailable",
    message: "Socket disconnected — re-acquire required",
  });
}
```

### 5.7 Telemetry Reconciliation

```typescript
function reconcileTelemetry(telem: JoystickTelemetryFields): void {
  // Update diagnostic fields for display
  setLastCmdAgeMs(telem.joystick_last_valid_cmd_age_ms ?? null);
  setStopReason(telem.joystick_stop_reason ?? null);

  // Authoritative state check: if backend says inactive but we think we're active,
  // the lease was revoked server-side. Clear local state.
  if (
    telem.joystick_state === "inactive" &&
    (stateRef.current === "ACTIVE" || stateRef.current === "HELD" ||
     stateRef.current === "ACQUIRING" || stateRef.current === "RELEASING")
  ) {
    console.warn(
      "[JOYSTICK] Backend state is inactive but local state is",
      stateRef.current,
      "— clearing local lease. Stop reason:",
      telem.joystick_stop_reason,
    );

    stopCommandSender();
    leaseIdRef.current = null;
    sequenceRef.current = 0;
    latestThrottleRef.current = 0;
    latestSteeringRef.current = 0;
    deadmanRef.current = false;
    setLeaseId(null);
    setState("AVAILABLE");
    setStopReason(telem.joystick_stop_reason ?? "lease_revoked_by_server");
  }

  // Mission arbitration: block acquire if mission is active
  if (telem.control_owner === "mission") {
    setState("BLOCKED_BY_MISSION");
  }

  // If state was BLOCKED_BY_MISSION and mission ended, revert to AVAILABLE
  if (
    telem.control_owner === "idle" &&
    stateRef.current === "BLOCKED_BY_MISSION"
  ) {
    setState("AVAILABLE");
  }
}
```

### 5.8 E-Stop Interaction

```typescript
// E-stop is available separate from joystick — it's a REST call
// When the user presses E-stop while joystick is active:

async function handleEStop(): Promise<void> {
  // 1. Force neutral locally first (don't wait for HTTP)
  latestThrottleRef.current = 0;
  latestSteeringRef.current = 0;
  deadmanRef.current = false;
  stopCommandSender();

  // 2. Best-effort send release if connected
  if (leaseIdRef.current && socketRef.current?.connected) {
    socketRef.current.emit("joystick_release", {
      auth: authTokenRef.current,
      session_id: sessionIdRef.current,
      lease_id: leaseIdRef.current,
    } as JoystickReleaseRequest);
  }

  // 3. Clear local lease immediately
  leaseIdRef.current = null;
  sequenceRef.current = 0;
  setLeaseId(null);

  // 4. Send E-stop via REST (existing estopVehicle function)
  await estopVehicle();

  // 5. Mark DISABLED — must re-acquire after E-stop
  setState("DISABLED");
}
```

### 5.9 Error Handling

```typescript
const ERROR_MESSAGES: Record<JoystickErrorCode, string> = {
  manual_control_disabled: "Manual control is disabled by deployment configuration",
  malformed: "Invalid request format — check payload",
  mode_unavailable: "MANUAL mode unavailable — check FCU state",
  fcu_disconnected: "Flight controller not connected",
  not_armed: "Vehicle must be armed before acquiring joystick",
  not_owner: "Session or lease mismatch — re-acquire required",
  mission_active: "Mission is active — cannot acquire joystick",
  joystick_active: "Joystick already in use by another client",
  acquire_cancelled: "Acquire was cancelled — retry",
  unavailable: "Joystick controller not available",
  lease_inactive: "Lease is not active — re-acquire required",
  transport_unavailable: "Manual control transport unhealthy",
  out_of_order: "Command sequence error — resetting",
  replay: "Command replay detected — resetting",
  rate_exceeded: "Command rate exceeded — throttling",
  nan_value: "Invalid axis value (NaN/Infinity)",
  out_of_range: "Axis value outside [-1, 1] range",
};

function onError(err: JoystickErrorEvent): void {
  const shouldClearLease: JoystickErrorCode[] = [
    "not_owner", "lease_inactive", "mode_unavailable",
    "fcu_disconnected", "not_armed", "unavailable",
    "acquire_cancelled",
  ];
  const shouldStopSender: JoystickErrorCode[] = [
    "lease_inactive", "not_owner", "mode_unavailable",
    "fcu_disconnected", "transport_unavailable",
    "acquire_cancelled",
  ];
  const requiresReacquire: JoystickErrorCode[] = [
    "not_owner", "lease_inactive", "acquire_cancelled",
  ];
  const shouldForceNeutral: JoystickErrorCode[] = [
    "nan_value", "out_of_range", "transport_unavailable",
  ];

  if (shouldClearLease.includes(err.code)) {
    leaseIdRef.current = null;
    sequenceRef.current = 0;
    setLeaseId(null);
  }
  if (shouldStopSender.includes(err.code)) {
    stopCommandSender();
  }
  if (shouldForceNeutral.includes(err.code)) {
    latestThrottleRef.current = 0;
    latestSteeringRef.current = 0;
  }
  if (requiresReacquire.includes(err.code)) {
    setState("AVAILABLE");
  }

  setError(err);

  // Show user-facing message
  showToast(
    `Joystick Error`,
    err.message || ERROR_MESSAGES[err.code] || err.code,
    "error",
  );
}
```

### 5.10 Joystick Math Utilities

```typescript
// src/utils/joystickMath.ts

/**
 * Apply dead zone: values within ±threshold are clamped to 0.
 * Values outside are remapped to fill [0, 1] proportionally.
 */
export function applyDeadZone(value: number, threshold: number): number {
  if (Math.abs(value) < threshold) return 0;
  const sign = value > 0 ? 1 : -1;
  return sign * (Math.abs(value) - threshold) / (1 - threshold);
}

/**
 * Apply exponential response curve for finer control near center.
 * exponent=1.0 is linear; 2.0 gives more precision near zero.
 */
export function applyResponseCurve(value: number, exponent: number): number {
  const sign = value > 0 ? 1 : -1;
  return sign * Math.pow(Math.abs(value), exponent);
}

/**
 * Clamp axis to [min, max] range.
 */
export function clampAxis(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

/**
 * Full axis processing pipeline:
 * 1. Dead zone
 * 2. Response curve
 * 3. Clamp to configured limits
 */
export function processAxis(
  raw: number,
  deadZone: number,
  curveExponent: number,
  maxLimit: number,
): number {
  const withDeadZone = applyDeadZone(raw, deadZone);
  const withCurve = applyResponseCurve(withDeadZone, curveExponent);
  return clampAxis(withCurve, -maxLimit, maxLimit);
}
```

---

## 6. Safety and Cleanup Rules

### 6.1 Dead-Man

- Separate dedicated press-and-hold button — not part of the joystick thumb area
- Must be continuously pressed for any non-zero command to flow
- Release → immediate `deadman=false` frame + forced local neutral
- Visual: pulse animation when active, prominent "HOLD TO DRIVE" label

### 6.2 Disconnect

- Socket `disconnect` event → stop command timer, clear local lease, force neutral
- Do NOT attempt to send release on disconnect — there's no connection
- State → DISCONNECTED
- Show "Connection lost — re-acquire required" banner

### 6.3 Background

- `AppState` change to `"background"` or `"inactive"` → full cleanup sequence
- Send neutral deadman=false frame (best-effort)
- Send release (best-effort)
- Stop command timer
- Clear local lease
- State → DISCONNECTED

### 6.4 Foreground (Reconnect)

- Do NOT auto-acquire
- Wait for telemetry to reconcile
- Require explicit user action to acquire
- If backend telemetry shows joystick still active under old lease, show "another client
  owns joystick — re-acquire to take over"

### 6.5 Screen Unmount

- Same as background: full cleanup
- `useEffect` cleanup function in the hook handles this

### 6.6 E-Stop

- Immediately force local neutral
- Stop command timer
- Clear lease (best-effort release if connected)
- Send REST E-stop
- State → DISABLED
- E-stop button remains available regardless of joystick state

### 6.7 Lease Non-Persistence

- `lease_id` stored only in `useRef` (not `useState` that might persist)
- Never written to AsyncStorage, never survives app restart
- Generated `session_id` stored in ref, regenerated on app launch
- `sequence` reset to 0 on each new acquire

### 6.8 Command Rules Checklist

- [x] No commands without valid socket connection
- [x] No commands without valid lease_id
- [x] No commands without deadman pressed (commanded intent forced to 0)
- [x] No commands while app is in background
- [x] No commands after release or error that clears the lease
- [x] Sequence must increment by 1 per transmitted frame
- [x] `client_monotonic_ms` must be monotonically increasing per session
- [x] Throttle/steering must be finite and within [-1.0, 1.0]
- [x] Send at the rate reported in acquire response `command_rate_hz`
- [x] Thumb release on joystick returns to neutral (0, 0)
- [x] Dead-man release sends immediate neutral + deadman=false frame

### 6.9 Mission Arbitration Rules

- [x] Do not allow acquire while `mission_state` is in `{LOADING, ARMING, SWITCHING_OFFBOARD,
  RUNNING, STOPPING, DISARMING}` — gate on `control_owner !== "mission"` from telemetry
- [x] Do not allow mission start while joystick state is ACTIVE/HELD/ACQUIRING/RELEASING
- [x] Do not assume release succeeded until `joystick_state === "inactive"` in telemetry
- [x] Show mode, armed state, FCU connection, transport health in joystick HUD
- [x] Show `joystick_stop_reason` when joystick becomes inactive
- [x] Show `joystick_last_valid_cmd_age_ms` as "command age"

---

## 7. Test Plan

### 7.1 Unit Tests

Run with `jest` or `vitest` in the React Native project.

**Joystick math** (`src/utils/joystickMath.ts`):

| ID | Test | Expected |
|----|------|----------|
| M1 | `applyDeadZone(0.02, 0.03)` | `0` (within dead zone) |
| M2 | `applyDeadZone(-0.02, 0.03)` | `0` (negative within dead zone) |
| M3 | `applyDeadZone(0.5, 0.03)` | `~0.485` (remapped) |
| M4 | `applyDeadZone(1.0, 0.03)` | `1.0` (exact boundary) |
| M5 | `applyResponseCurve(0.5, 2.0)` | `0.25` (square) |
| M6 | `applyResponseCurve(-0.5, 2.0)` | `-0.25` (sign preserved) |
| M7 | `clampAxis(0.25, -0.15, 0.15)` | `0.15` (clamp high) |
| M8 | `clampAxis(-0.8, -0.5, 0.5)` | `-0.5` (clamp low) |
| M9 | `processAxis(0.5, 0.03, 1.0, 0.15)` | `0.15` (full pipeline) |

**Sequence handling** (`useVirtualJoystick` in isolation):

| ID | Test | Expected |
|----|------|----------|
| S1 | Acquire → sequence starts at 0 | `sequenceRef.current === 0` |
| S2 | First command → sequence becomes 1 | After one `setInterval` tick |
| S3 | Two commands → sequence is 2 | Monotonic increment |
| S4 | New acquire → sequence resets to 0 | Fresh lease |
| S5 | No command without lease | Sender does nothing when `leaseIdRef.current === null` |
| S6 | No command without deadman | Throttle/steering forced to 0 |

**Error handling:**

| ID | Test | Expected |
|----|------|----------|
| E1 | `not_owner` error → clear lease | `leaseIdRef.current` becomes null |
| E2 | `lease_inactive` error → stop sender | Timer cleared |
| E3 | `out_of_range` error → force neutral | `latestThrottleRef.current` becomes 0 |
| E4 | `mission_active` error on acquire → state stays AVAILABLE | Acquire rejected |

**State machine:**

| ID | Test | Expected |
|----|------|----------|
| ST1 | Initial state with no socket | `DISABLED` |
| ST2 | Socket connected + healthy telemetry | `AVAILABLE` |
| ST3 | Acquire sent | `ACQUIRING` |
| ST4 | `joystick_acquired` received | `ACTIVE` |
| ST5 | Deadman released during ACTIVE | `HELD` |
| ST6 | Deadman pressed during HELD | `ACTIVE` |
| ST7 | Release sent | `RELEASING` |
| ST8 | `joystick_released` received | `AVAILABLE` |
| ST9 | Mission active telemetry | `BLOCKED_BY_MISSION` |
| ST10 | Socket disconnect during ACTIVE | `DISCONNECTED` |
| ST11 | Error with `not_owner` code | `AVAILABLE` (requires re-acquire) |

### 7.2 Socket Integration Tests

Run against Jetson backend with `ROVER_JOYSTICK_MANUAL_ENABLED=1` and
`ROVER_DISABLE_AUTH=1` (for testing without token):

| ID | Test | Procedure | Expected |
|----|------|-----------|----------|
| I1 | Acquire success | Connect → arm → send `joystick_acquire` | Receive `joystick_acquired` with `lease_id`, `max_throttle`, `max_steering` |
| I2 | Acquire rejected — not armed | Connect → send `joystick_acquire` (disarmed) | Receive `joystick_error` with code `not_armed` |
| I3 | Acquire rejected — mission active | Start mission → send `joystick_acquire` | Receive `joystick_error` with code `mission_active` |
| I4 | Valid command accepted | Acquire → send valid `joystick_command` | No error event (silent success) |
| I5 | Wrong lease_id rejected | Acquire → send command with wrong lease_id | Receive `joystick_error` with code `not_owner` |
| I6 | Deadman false → HELD state | Acquire → send command with `deadman=false` | Telemetry shows `joystick_state: "held"` |
| I7 | Release clears local state | Acquire → send release → receive released | Backend telemetry shows `joystick_state: "inactive"` |
| I8 | Backend lease timeout → revocation | Acquire → stop sending → wait 2+ seconds | Telemetry shows `joystick_state: "inactive"`, `joystick_stop_reason: "lease_timeout"` |
| I9 | Reconnect requires fresh acquire | Can't send commands after disconnect/reconnect | `not_owner` error for old lease_id |
| I10 | E-stop clears joystick | Acquire → send REST E-stop | Telemetry shows `joystick_state: "inactive"`, `joystick_stop_reason: "estop"` |

### 7.3 Jetson Bench Verification

**Precondition:** Rover powered on, motors depowered or wheels off ground, FCU connected.

| ID | Test | Expected |
|----|------|----------|
| B1 | Acquire → mode switches to MANUAL | `ros2 topic echo /mavros/state` shows `mode: Manual` |
| B2 | Neutral throttle (0.0) → no wheel command | Wheels do not move |
| B3 | Forward (throttle=+0.15) → forward rotation | Wheels rotate forward |
| B4 | Reverse (throttle=-0.15) → reverse rotation | Wheels rotate backward |
| B5 | Steering right (steering=+0.5) → right turn | Left wheel faster than right |
| B6 | Steering left (steering=-0.5) → left turn | Right wheel faster than left |
| B7 | Stationary turn (throttle=0, steering=0.5) | Wheels rotate opposite directions |
| B8 | Thumb release → neutral | Wheels stop |
| B9 | Dead-man release → immediate neutral | Wheels stop, telemetry shows `joystick_deadman: false` |
| B10 | App background → release | Backend telemetry shows `joystick_state: "inactive"` |
| B11 | Socket disconnect → release | Backend telemetry shows `joystick_state: "inactive"` |
| B12 | No command for >300ms → server zeroes | Wheels stop, backend still in ACTIVE |
| B13 | No command for >2s → lease revoked | `joystick_released` event fired |
| B14 | E-stop mid-session → disarm + neutral | Mode → MANUAL, armed → false |
| B15 | Mission start blocked during active | `mission_start` rejected |
| B16 | 60-second continuous session | No RC_LOSS, no spurious mode changes |

---

## 8. Open Questions

1. **Auth token delivery to frontend:** The backend uses a shared-secret file
   (`~/.rover_token`). The frontend currently does not send `auth` in Socket.IO events
   (only REST uses `X-Rover-Token`). The frontend must obtain the token. Options: (a) manual
   entry in settings, (b) hardcoded for dev, (c) copied from Jetson. This needs a decision.

2. **Navigation integration:** The app uses a single `Page` state variable with a sidebar
   menu (not React Navigation stack). The joystick screen could be added as a new `Page`
   option (e.g., `"manual"`) or as a modal. The existing pattern of `page` state and
   conditional rendering in App.tsx suggests adding a new page.

3. **E-stop via Socket.IO vs REST:** The backend supports both `emergency_stop` Socket.IO
   event handler AND `POST /api/estop` REST endpoint. The Socket.IO handler calls the
   same `EmergencyHandler.estop_async()`. For the frontend, REST is already wired and working
   — continue using REST. The Socket.IO path is also available but is redundant.

4. **`client_monotonic_ms` in acquire request:** The Pydantic model `JoystickAcquireRequest`
   includes this field but the backend controller only uses `session_id`. The field should
   be sent for schema compliance but is not validated at acquire time. Clarify whether this
   is intentional or should be removed from the model.

5. **React Native Animated vs Reanimated:** The project already includes
   `react-native-reanimated` (v4.1.7) in dependencies — use Reanimated worklets for smooth
   60fps joystick rendering. If performance issues arise with PanResponder, switch to
   Gesture Handler's `Gesture.Pan()`.

6. **Socket.IO reconnect behavior:** The backend disconnects old sid and releases joystick
   on disconnect. The frontend Socket.IO client auto-reconnects. On reconnect, a new
   `joystick_acquire` with a fresh `session_id` is required.

7. **Token rotation:** The backend auto-generates a token if `~/.rover_token` is missing.
   There's no mechanism for the frontend to discover this token. Consider adding a startup
   log line that the operator can read from the Jetson console.

---

## 9. Final Frontend Implementation Checklist

### Phase 1 — Types and math (no backend dependency)

- [ ] **1. Create `src/types/joystick.ts`** — all joystick protocol types
- [ ] **2. Create `src/utils/joystickMath.ts`** — axis processing functions
- [ ] **3. Extend `src/types/plan.ts`** — add joystick telemetry fields to `TelemetrySnapshot`
- [ ] **4. Write unit tests for joystickMath.ts** (M1-M9 from test plan)

### Phase 2 — Hook and socket listeners (no UI)

- [ ] **5. Create `src/hooks/useVirtualJoystick.ts`** — full state machine, acquire/command/release flow, fixed-rate sender, dead-man tracking, AppState handling, telemetry reconciliation
- [ ] **6. Register joystick event listeners in App.tsx** — alongside existing telemetry listeners
- [ ] **7. Add auth token state/config** — decide how the frontend gets the shared secret
- [ ] **8. Add AppState listener** in App.tsx (currently absent from the entire app)
- [ ] **9. Wire telemetry fields into `telemetrySnapshot`** — extend the merge logic
- [ ] **10. Write unit tests for state machine transitions** (ST1-ST11 from test plan)

### Phase 3 — UI components

- [ ] **11. Create `src/components/VirtualJoystick.tsx`** — dual-axis thumb pad with PanResponder or Gesture Handler
- [ ] **12. Create `src/components/DeadmanButton.tsx`** — press-and-hold dead-man button
- [ ] **13. Create `src/screens/ManualControlScreen.tsx`** — full manual control screen layout
- [ ] **14. Add navigation route** — new page for manual control in the sidebar menu

### Phase 4 — Integration and arbitration

- [ ] **15. Mission start guard** — disable start when `joystick_active === true` in telemetry
- [ ] **16. Joystick acquire guard** — disable acquire when `control_owner === "mission"` in
  telemetry
- [ ] **17. Wire E-stop interaction** — force-neutral + clear lease on E-stop press
- [ ] **18. Wire arm/disarm guard** — show arm requirement before acquire

### Phase 5 — Socket integration tests (Jetson, auth disabled)

- [ ] **19. Run I1-I10 integration tests** against Jetson backend
- [ ] **20. Verify error codes match backend** — check all error code handling
- [ ] **21. Test reconnect flow** — disconnect + reconnect + re-acquire

### Phase 6 — Jetson bench verification (motors depowered)

- [ ] **22. Run B1-B16 bench tests** with wheels blocked/off-ground
- [ ] **23. Verify no RC_LOSS during 60-second session**
- [ ] **24. Verify mode transitions** (MANUAL acquire, OFFBOARD on release)
- [ ] **25. Measure Socket.IO latency** — must be < 100ms

### Phase 7 — Field validation (low speed, spotter)

- [ ] **26. Controlled field test at ≤0.2 m/s with spotter**
- [ ] **27. Validate forward/reverse/steering/stationary-turn**
- [ ] **28. Validate dead-man release stops rover within 1s**
- [ ] **29. Validate E-stop full sequence**
- [ ] **30. Validate mission ↔ joystick handover**

---


zzion Commands

Run these on Jetson to verify backend state during development:

```bash
# Check if manual control is enabled
journalctl -u rover-server.service --no-pager -n 20 | grep -i manual

# Watch joystick state in telemetry
curl -s http://localhost:5001/api/ping

# Check MAVROS manual_control topic
ros2 topic list | grep manual_control

# Watch PX4 mode
ros2 topic echo /mavros/state --once | grep mode

# Check joystick config values
curl -s http://localhost:5001/api/telemetry | jq '{joystick_state, joystick_active, control_owner, armed, mode}'
```

## Appendix B: Backend Deployment Variables

Set these on Jetson in the rover-server systemd drop-in:

```bash
ROVER_JOYSTICK_MANUAL_ENABLED=1    # Required: enable manual control
ROVER_JOYSTICK_COMMAND_RATE_HZ=20  # Command rate (frontend sends at this rate)
ROVER_JOYSTICK_MAX_ABS_THROTTLE=0.15  # Max throttle (0.15 = 15%)
ROVER_JOYSTICK_MAX_ABS_STEERING=0.50  # Max steering (0.50 = 50%)
# Auth: ensure ROVER_DISABLE_AUTH=1 for dev, or provision ~/.rover_token