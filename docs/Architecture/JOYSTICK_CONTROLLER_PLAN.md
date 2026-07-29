# Joystick Controller (Virtual Joystick / Manual Control) — Implementation Plan

> **Status:** PLAN ONLY — reference design. No logic is proposed for blind merge.
> **Target branch:** `Upgrade_Spray` (this doc lives here).
> **Scope:** the **full manual-control stack** — server `JoystickController` +
> `ManualControlGateway` + `ControlArbiter` + Socket.IO wiring + config + telemetry, plus the
> React-Native client contract for `Three_Wheel_v2`.
> **Injection route (operator-selected):** **Option A — MAVLink `MANUAL_CONTROL`** (PX4 switched
> to `MANUAL` mode). See §3 for why this route carries firmware prerequisites that gate the whole
> feature.
>
> **Explicitly OUT of scope:** runtime stop / hold / recenter, RPP pivot / true-stop, spray. This
> plan is *only* the joystick controller.
>
> **Design intent:** the existing joystick code (on `feat/entry-pivot-recenter` /
> `fix/runtime-entry-stop`) is a strong reference but **not robust enough to cherry-pick**. This
> plan keeps its proven patterns, names its real defects, and specifies a hardened build for
> `Upgrade_Spray`.

---

## 0. TL;DR

Add operator-driven manual driving to `Upgrade_Spray`: an authenticated Socket.IO lease lets one
client stream throttle/steering while holding a dead-man, PX4 is put in `MANUAL` mode, and the
server relays `MANUAL_CONTROL` MAVLink frames at a fixed rate with a layered watchdog that goes
neutral and revokes the lease on any staleness. Mission and joystick are mutually exclusive via a
single-owner arbiter.

**Two hard prerequisites gate the whole feature (Option A):**
1. **`COM_RC_IN_MODE`** on the production FCU must permit MAVLink `MANUAL_CONTROL` (not `RC_ONLY`).
   If it is RC-only, PX4 silently ignores every frame we send. *(Audit R9 — UNRESOLVED.)*
2. **The `MANUAL_CONTROL` axis mapping must match what `rover_differential` actually reads.** The
   reference gateway encodes **steering into `y` (roll)** and throttle into `z`; a differential
   rover in MANUAL normally takes **throttle on `z` and steering/turn on `r` (yaw)**. This is a
   likely mapping bug — verify on the bench before trusting any drive. *(New finding — §7.1.)*

Until both are verified on the bench (props off), the feature cannot leave Phase J3.

---

## 1. Reference sources (what this plan is built from)

| Source | Location | Role |
|---|---|---|
| `JoystickController` | `feat/entry-pivot-recenter:server/joystick_controller.py` (18 KB) | lease FSM, dead-man, watchdog, validation |
| `ManualControlGateway` | `…:server/manual_control_gateway.py` (12 KB) | MANUAL_CONTROL transport + 50 Hz sender thread + stale watchdog |
| `ControlArbiter` | `…:server/control_arbiter.py` (7.5 KB) | single-owner mission/joystick mutual exclusion |
| Socket.IO handlers | `…:server/sockets/events.py` | `joystick_acquire/command/release`, disconnect release |
| Config knobs | `…:server/config.py` (`JOYSTICK_*`) | rate/timeout/limit params + load-time invariant checks |
| Frontend wiring guide | `…:docs/REACT_NATIVE_JOYSTICK_V2_WIRING_GUIDE.md` (1592 ln) | full client↔server contract |
| Architecture audit | `…:docs/virtual_joystick_audit.md` | route comparison (A/B/C/D), risk register R1–R10 |
| Client app (target) | `Three_Wheel_v2` @ `CSV-alignment` — `src/hooks/useVirtualJoystick.ts`, `components/ManualJoystick.tsx`, `DeadmanButton.tsx`, `utils/joystick*` | the PX4_DXP operator app |
| Client app (secondary ref) | `DYX_GCS_V` @ `refactor` — `src/utils/joystick*`, `components/manual/*`, `types/px4/joystick.ts` | a second, independent client of the same protocol |

**Current `Upgrade_Spray` (HEAD `1193a43`) has none of the server joystick stack.** It has clean
integration anchors: `server/main.py` globals (`ros_node`, `offboard_ctrl`, `emergency_handler`,
populated in `lifespan`), `server/ros_node.py` (`RosBridgeNode`), `server/emergency.py`
(`estop_async`), and `server/sockets/events.py`.

---

## 2. Definitions & the lease state machine

**Ownership FSM (server, `JoystickController._state`):**

```
INACTIVE ──acquire()──▶ ACQUIRING ──MANUAL confirmed + lease minted──▶ ACTIVE
   ▲                        │                                            │  ▲
   │                        └── acquire fail ──▶ INACTIVE                │  │ deadman=true
   │                                                       deadman=false │  │
   │                                                                     ▼  │
   └────────── release()/watchdog/estop ◀── RELEASING ◀── release ──── HELD
```

- **ACTIVE** — lease held, dead-man asserted, real throttle/steering flow.
- **HELD** — lease held, dead-man released → throttle/steering forced to 0 (lease *not* dropped).
- **RELEASING** — explicit release in flight.
- The controller is the **single source of truth**; the client reconciles to it via telemetry.

**Mutual exclusion (`ControlArbiter.ControlOwner`):** `IDLE / MISSION / JOYSTICK_ACQUIRING /
JOYSTICK_ACTIVE / JOYSTICK_HELD / RELEASING`. Invariant: **MISSION and JOYSTICK ownership are never
simultaneous** — one lock, checked at both mission-start and joystick-acquire.

---

## 3. Route decision — Option A (MANUAL_CONTROL), with prerequisites

The audit compared four injection routes. Operator selected **Option A**. Recorded here for the
record, including the trade-off being accepted:

| Option | PX4 mode | Heartbeat owner | RPP | Note |
|---|---|---|---|---|
| **A — MAVLink `MANUAL_CONTROL`** ✅ selected | **MANUAL** | gateway thread @ 50 Hz | bypassed (mode leaves OFFBOARD) | as-implemented; firmware-dependent |
| C — velocity into `/rpp/velocity_ned` | OFFBOARD | `twist_to_setpoint` (unchanged) | idled via flag | audit's "primary"; not chosen |

**What choosing A commits us to:**
- A **real PX4 mode switch** `OFFBOARD/AUTO → MANUAL` on acquire, and back on release. The controller
  already does this (`set_mode_async("MANUAL")` + confirm within `JOYSTICK_MODE_CONFIRM_TIMEOUT_S`).
- The rover is driven by the **MAVLink `MANUAL_CONTROL` manual pipeline**, not the OFFBOARD/RPP
  pipeline. RPP/`twist_to_setpoint` are inert while manual is active.
- **Two firmware unknowns become hard gates** (§0): `COM_RC_IN_MODE` acceptance (R9) and the axis
  mapping (§7.1). Neither is negotiable — if PX4 ignores the frames or maps the wrong axis, the
  operator gets either no motion or wrong-direction motion.

> If bench testing (Phase J3) shows PX4 does **not** drive correctly from `MANUAL_CONTROL` on this
> `rover_differential` airframe, the fallback is to re-open the Option C decision — but that is a
> separate plan, not a silent pivot.

---

## 4. Backend architecture (full stack, hardened)

### 4.1 `ManualControlGateway` + transport (`server/manual_control_gateway.py`)

- **Transport abstraction** `ManualControlTransport` with two impls:
  - `MavrosManualControlTransport` → publishes `mavros_msgs/msg/ManualControl` on
    `/mavros/manual_control/send`. Casts x/y/z/r to `float32` (rclpy rejects int on float field),
    tracks consecutive publish errors, health-gates on subscriber presence
    (`JOYSTICK_MAVROS_REQUIRE_SUBSCRIBER`) and error count (`…PUBLISH_ERROR_LIMIT`).
  - `PymavlinkManualControlTransport` → direct `manual_control_send` (fallback).
- **Frame encoding** `encode_manual_control(throttle, steering)`:
  `x=0, y=steering*1000 [-1000,1000], z=(throttle+1)*500 [0,1000], r=0, buttons=0`.
  **Neutral frame = `(0,0,500,0,0)`.**
  - ⚠ **§7.1 finding:** steering in `y`, `r=0`. For `rover_differential`, turn is normally `r`
    (yaw). This mapping **must be bench-verified/corrected** before Phase J4.
- **Fixed-rate sender thread** (`_run`, `JOYSTICK_GATEWAY_RATE_HZ = 50`): a **dedicated thread**,
  not the asyncio loop, so a stalled command handler can never repeat the last nonzero frame
  forever. If the last accepted command is older than `JOYSTICK_GATEWAY_STALE_TIMEOUT_S`, it sends
  **neutral** instead. This is the innermost safety layer — keep it exactly.
- `activate_neutral()` / `wait_neutral_barrier()` — streams neutral **before** the mode switch
  (satisfies "stream setpoints before requesting the mode" hygiene). `deactivate_neutral()` flushes
  3 neutral frames on teardown.

### 4.2 `JoystickController` (`server/joystick_controller.py`)

Lease lifecycle, all guarded:
- **`acquire(sid, data)`** under `arbiter.joystick_acquire()`: reject if manual disabled, if a
  mission is active, or if already owned. Sequence: check FCU ready (`connected` + `armed`) → check
  transport healthy → `gateway.activate_neutral()` → `wait_neutral_barrier(NEUTRAL_PRESTREAM_S)` →
  `set_mode_async("MANUAL")` → confirm MANUAL within `MODE_CONFIRM_TIMEOUT_S` → re-check the
  acquire wasn't cancelled → mint `lease_id` (uuid4 hex) → `ACTIVE` → start watchdog. Returns
  `{lease_id, command_rate_hz, server_stop_timeout_ms, gateway_stop_timeout_ms, max_throttle,
  max_steering}` — the client **freezes these limits**.
- **`handle_command(sid, data)`** (synchronous, on the socket handler): validate owner
  (sid+session+lease), state ∈ {ACTIVE,HELD}, transport healthy, **still in MANUAL mode**,
  **strictly increasing `sequence`**, **non-decreasing `client_monotonic_ms`** (replay guard),
  finite + in-range values, rate not exceeded. Clamp to `MAX_ABS_THROTTLE/STEERING`. If
  `deadman=false` → force zeros + `HELD`; else `ACTIVE`. Forward to `gateway.accept_command()`.
  Commands are **not** individually acked (success is silent; only errors emit).
- **`release(... reason, force)`** — owner-checked unless `force`; `RELEASING` →
  `gateway.deactivate_neutral()` → `_clear_local`. Broadcast `joystick_released`.
- **`emergency_neutralize(reason)`** — synchronous hard neutral + full state clear, for the e-stop
  path.
- **Watchdog** (`_watchdog_loop`, 50 ms tick): if last-valid-command age >
  `SERVER_STOP_TIMEOUT_S` → push neutral; if > `LEASE_REVOKE_TIMEOUT_S` (or `LEASE_EXPIRY_S`) →
  `release(force=True, reason="lease_timeout")`. Crash-safe: any exception forces neutral.
- **`snapshot()`** merges arbiter + gateway + joystick state into telemetry (§5).

### 4.3 `ControlArbiter` (`server/control_arbiter.py`)

Single `asyncio.Lock` guarding ownership. `mission_start()` rejects while joystick-owned;
`joystick_acquire()` rejects while a mission is active (`MISSION_ACTIVE_STATES`). This is the
enforcement point for the mutual-exclusion invariant.

> **Hardening (do not port the smell):** the reference uses `contextvars` re-entry
> (`clear_reentry_context`/`reset_reentry_context`) to survive nested `async with` — a sign the
> call graph re-enters the lock. On `Upgrade_Spray`, design the call graph **flat** (handler →
> arbiter → gateway) so no re-entry shim is needed. If a nested acquire ever appears, fix the call
> graph, don't reintroduce the shim. (§7.4)

### 4.4 Socket.IO wiring (`server/sockets/events.py`)

Handlers: `joystick_acquire` → `ctrl.acquire`; `joystick_command` → `ctrl.handle_command`
(errors → `joystick_error` to sid); `joystick_release` → `ctrl.release` (broadcast
`joystick_released`); `disconnect` → if the sid owns the lease, `ctrl.release(reason="disconnect")`.
**Every event payload carries `auth` in the body** (not a header) — validate before touching the
controller. E-stop (`emergency_stop` / `POST /api/estop`) must call
`ctrl.emergency_neutralize()` in addition to the existing stop-path.

### 4.5 Config & the timeout-ordering invariant (`server/config.py`)

All `JOYSTICK_*` env-configurable, **validated at load** — the ordering below is the safety chain
and `config.py` refuses to start if violated:

```
SERVER_STOP_TIMEOUT_S (0.30)  <  GATEWAY_STALE_TIMEOUT_S (0.40)
                              <  PX4_RC_LOSS_S (0.50)
                              <  LEASE_REVOKE_TIMEOUT_S (2.0)
```

Meaning: server zeros the command first (0.30), the gateway independently goes neutral (0.40), PX4's
own RC-loss failsafe is the backstop (0.50), and only after 2 s is the lease revoked. **Keep this
invariant and its load-time check.**

| Param | Default | Meaning |
|---|---|---|
| `JOYSTICK_MANUAL_ENABLED` | `0` (off) | must be `1` to enable at all |
| `JOYSTICK_MANUAL_TRANSPORT` | `mavros` | `mavros` \| `pymavlink` |
| `JOYSTICK_COMMAND_RATE_HZ` | `20.0` | advertised client send rate |
| `JOYSTICK_GATEWAY_RATE_HZ` | `50.0` | gateway MANUAL_CONTROL publish rate |
| `JOYSTICK_SERVER_STOP_TIMEOUT_S` | `0.30` | server → neutral |
| `JOYSTICK_GATEWAY_STALE_TIMEOUT_S` | `0.40` | gateway → neutral |
| `JOYSTICK_PX4_RC_LOSS_S` | `0.50` | must stay < PX4's failsafe |
| `JOYSTICK_LEASE_REVOKE_TIMEOUT_S` | `2.0` | revoke lease |
| `JOYSTICK_LEASE_EXPIRY_S` | `30.0` | hard expiry |
| `JOYSTICK_NEUTRAL_PRESTREAM_S` | `0.20` | neutral before mode switch |
| `JOYSTICK_MODE_CONFIRM_TIMEOUT_S` | `3.0` | max wait for MANUAL confirm |
| `JOYSTICK_MAX_ABS_THROTTLE` | `0.15`–`0.35`* | throttle clamp (advertised to client) |
| `JOYSTICK_MAX_ABS_STEERING` | `0.20`–`0.50`* | steering clamp (advertised to client) |

\* reference defaults drifted between the code (`0.35`/`0.20`) and the wiring guide (`0.15`/`0.50`).
**Pin explicit conservative values for first field runs** (recommend `0.10 / 0.20`) and raise only
after validation. Do not inherit whichever default happens to be in the file.

---

## 5. Telemetry contract (server → client, folded into the 10 Hz telemetry push)

Authoritative (client reconciles to these): `joystick_state`, `joystick_active`,
`joystick_owner_present`, `joystick_has_lease`, `joystick_stop_reason`, `control_owner`,
`joystick_owned`, plus FCU `armed` / `mode` / `connected`.
Diagnostic: `joystick_last_valid_cmd_age_ms`, `joystick_deadman`,
`joystick_commanded_throttle/steering`, `gateway_active`, `gateway_command_age_ms`,
`gateway_last_send_age_ms`, `gateway_last_frame{x,y,z,r,buttons}`, `gateway_last_sent_neutral`,
`transport`, `transport_healthy`, `transport_error`.

**These fields are load-bearing for client safety** — the client detects lease loss and mission
takeover *only* through telemetry (`joystick_state=="inactive"`, `control_owner=="mission"`). If
telemetry stops carrying them, the client cannot recover. Treat them as part of the contract.

---

## 6. Client contract (both apps) — what the backend must satisfy

### 6.1 Events & payloads (identical across `Three_Wheel_v2` and `DYX_GCS_V`)

- **`joystick_acquire`** → `{ auth, session_id, client_monotonic_ms }`
- **`joystick_command`** → `{ auth, session_id, lease_id, sequence, client_monotonic_ms, deadman,
  throttle, steering }` — `sequence` starts at 1, strictly increasing; `client_monotonic_ms` =
  `floor(performance.now())`, non-decreasing; throttle/steering ∈ [−1,1], forced 0 when
  `deadman=false`.
- **`joystick_release`** → `{ auth, session_id, lease_id }`
- Server → client: **`joystick_acquired`** (carries `command_rate_hz, max_throttle, max_steering,
  server_stop_timeout_ms, gateway_stop_timeout_ms`), **`joystick_error`** `{type,code,message}`,
  **`joystick_released`** (broadcast) `{state,reason,lease_id?}`.
- Auth on **every** event body; a bad token → `socket_error {reason:"unauthorised"}`.

### 6.2 Client timing the backend must tolerate

- **Client floors its send interval at 55 ms (~18 Hz)** via `safeCommandIntervalMs = max(55,
  ceil(1000/hz)+5)`, *even when the server advertises 20 Hz*. Backend `SERVER_STOP_TIMEOUT_S`
  (0.30) and rate limit must comfortably accommodate ~55 ms + jitter — **do not set the rate limit
  to reject 18 Hz**, and do not set stop timeouts below ~3× the client interval.
- Client is a **single self-rescheduling heartbeat** reading latest refs (gesture frames at
  60–120 Hz only mutate refs, they don't emit). Backend sees a steady stream, not per-input bursts.
- **Out-of-band "urgent neutral"** commands (`deadman=false, 0, 0`) can arrive any time (on
  release/blur/e-stop/deadman-off) — backend must accept them out of cadence.
- Client gives up locally after **`RELEASE_CONFIRM_TIMEOUT_MS = 1000`** if no `joystick_released`,
  and after **`ACQUIRE_TIMEOUT_MS = 3800`** if no `joystick_acquired`. Backend acquire (mode
  confirm ≤ 3.0 s) fits inside 3.8 s — **keep it that way** or the client self-cancels mid-acquire.

### 6.3 Dead-man models **differ between the two apps** — backend must treat `deadman` as an opaque authoritative boolean

- **`Three_Wheel_v2`**: has a dedicated **`DeadmanButton` ("HOLD TO DRIVE")** *and* intent-derived
  deadman (`hasLease && stick-off-center`). Either can assert it.
- **`DYX_GCS_V`**: **no** deadman button — deadman is **purely stick-deflection**
  (`hasLease && !centered`). Centering the stick is the release.

  → The backend must not assume a particular UI gesture. It consumes the `deadman` boolean as sent;
  `deadman=false` ⇒ forced zeros + `HELD`, regardless of how the client derived it. This is already
  how `handle_command` behaves — **preserve it**; do not add UI assumptions server-side.

### 6.4 Legacy path to avoid

`Three_Wheel_v2/src/api/vehicleApi.ts` also has a **REST `POST /api/manual_control`** `{forward,
yaw}` — **no auth, no lease, no sequencing**. This is a legacy fire-and-forget path. **Do not
implement or rely on it** for the hardened feature; it bypasses every safety layer. If it must
exist for compatibility, gate it behind `JOYSTICK_MANUAL_ENABLED` and the arbiter too, or leave it
unimplemented (404).

---

## 7. Robustness gaps → fixes (the "not robust" answer)

| # | Weakness | Where | Fix in this plan |
|---|---|---|---|
| ~~**7.1**~~ | ✅ **CLOSED — steering IS `y`, `r=0`.** The "turn is normally `r`" hypothesis was tested and **did not hold in the field**. | `manual_control_gateway.encode_manual_control` | Done: encoder carries the FIELD-confirmed mapping (PX4 v1.16.2 `rover_differential`), golden-frame tests pin it (`test_manual_control_gateway.py:55,61`). **No longer a gate.** |
| **7.2** | **`COM_RC_IN_MODE`** may be `RC_ONLY` → PX4 silently ignores all frames | FCU param (QGC) | Verify via QGC (Mac is source of truth) that MAVLink stick input is enabled. **HARD gate before J3.** (Audit R9) |
| **7.3** | **Acquire↔mission-start race** (concurrent) | arbiter + offboard lifecycle | Both must acquire the **same** lock (or strictly ordered locks). Add a concurrency test that fires both simultaneously. (Audit R1) |
| **7.4** | **`contextvars` re-entry shim** in arbiter — hides a re-entrant call graph | `control_arbiter.py` | Design flat call graph; drop the shim. (§4.3) |
| **7.5** | **Reconnect leaves ownership dangling** | controller + socket | Rely on watchdog `LEASE_REVOKE` (2 s) + disconnect handler; new SID must re-acquire with fresh `session_id`. Test reconnect explicitly. (Audit R2) |
| **7.6** | **Stale `connected=True`** (TRANSIENT_LOCAL) | ros_node/state | Watchdog & acquire must consult `_state_recv_time` (2 s override), not raw `connected`. (Audit R5, matches CLAUDE.md MAVROS crash-detect rule) |
| **7.7** | **MANUAL mode not re-verified per command** could drift if PX4 exits MANUAL | `handle_command` | `_check_manual_mode()` on every command (reference already does — keep). On mode loss → error `mode_unavailable`, force neutral. |
| **7.8** | **Throttle/steering default limits inconsistent** (0.35/0.20 vs 0.15/0.50) | config | Pin explicit conservative values; never inherit ambient default. (§4.5) |
| **7.9** | **Motors drive the instant deadman asserts in MANUAL** | whole feature | First field run `MAX_ABS_THROTTLE=0.10`, spotter, props/wheels-blocked bench first. E-stop reachable at all times. |
| **7.10** | **Legacy REST `/api/manual_control`** bypasses all safety | vehicleApi | Do not implement, or gate identically. (§6.4) |

---

## 8. Phased rollout (J1–J6)

Adapted for the MANUAL_CONTROL route. Each phase gates the next; `JOYSTICK_MANUAL_ENABLED=0` until J3.

- **J1 — Server skeleton (no ROS, no hardware).** Port hardened `JoystickController`,
  `ManualControlGateway`, `ControlArbiter`, config (with invariant checks), socket handlers,
  `main.py` global + lifespan init. Unit tests (state machine, validation, watchdog, arbiter
  mutual-exclusion) green. **Gate:** all unit tests pass; config refuses illegal timeout ordering.
- **J2 — Transport wiring (Jetson, no motor power).** MAVROS `ManualControl` publisher created;
  verify `/mavros/manual_control/send` has a subscriber and frames publish. `ros2 topic echo` the
  topic; confirm neutral = `(0,0,500,0,0)`. **Gate:** frames flow, watchdog goes neutral on stale.
- **J3 — PX4 reachability (Jetson, wheels blocked / props off).** ⚠ **Verify §7.2 `COM_RC_IN_MODE`
  first.** Arm → acquire → MANUAL confirmed → send small throttle → observe PX4 manual-control
  setpoint / actuator response. **Verify §7.1 axis mapping here.** **Gate:** PX4 responds; correct
  axis drives correct wheels; watchdog + e-stop confirmed; no unexpected disarms.
- **J4 — Controlled hardware (field, spotter, `MAX_ABS_THROTTLE=0.10`).** Full drive, dead-man
  release stops, disconnect stops, background stops, lease-timeout revokes. Raise limits stepwise
  only after each passes. **Gate:** e-stop latency < 1 s at every step.
- **J5 — `Three_Wheel_v2` frontend integration.** Wire `useVirtualJoystick` to the shared socket
  with `auth` on every event; add joystick fields to telemetry type; block mission-start UI when
  `joystick_active`; block acquire when `control_owner=="mission"`. End-to-end drive + handoff +
  e-stop from app. **Gate:** client reconciles all lease-loss/takeover paths via telemetry.
- **J6 — Hardening (deferred).** Explicit yaw-rate (if §7.1 needs `r`), reconnect grace token,
  server-side chatty-client rate discard, richer diagnostics.

---

## 9. Test plan

- **Unit (server):** FSM transitions; owner/sequence/replay/range/rate validation; deadman→HELD
  forcing zeros; watchdog neutral@0.30 & revoke@2.0; arbiter rejects acquire-during-mission and
  mission-during-acquire; config load-time invariant rejection; **golden MANUAL_CONTROL frame test
  (§7.1)**; neutral frame = `(0,0,500,0,0)`.
- **Concurrency:** simultaneous `joystick_acquire` + `mission_start` — exactly one wins (R1).
- **Bench (Jetson, props off):** topic publish + subscriber health; stale→neutral; MANUAL confirm;
  axis mapping; e-stop neutralize. In-env only — controller patches must be verified on the Jetson
  (`ROS_DOMAIN_ID` per host), never static-review-only.
- **Field (spotter):** drive, dead-man release, disconnect, app-background, lease-timeout, e-stop,
  mission handoff — at each speed step.
- **Client:** reuse `joystickAcquireRecovery.test.ts` expectations (mission_active→BLOCKED,
  joystick_active→AVAILABLE, telemetry-inactive recovery, foreign-lease reject).

---

## 10. Do-NOT list

- **Do not** cherry-pick the reference files wholesale — reimplement hardened per §7.
- **Do not** enable `JOYSTICK_MANUAL_ENABLED=1` before J3 (firmware gates §7.1/§7.2 pass).
- **Do not** trust raw `connected` — use `_state_recv_time` (§7.6).
- **Do not** weaken the timeout-ordering invariant (§4.5).
- **Do not** set the server rate limit to reject the client's ~18 Hz floor (§6.2).
- **Do not** assume a UI deadman gesture server-side — `deadman` is an opaque boolean (§6.3).
- **Do not** implement the unauthenticated REST `/api/manual_control` as-is (§6.4).
- **Do not** push FCU params from Jetson — verify `COM_RC_IN_MODE` in QGC on the Mac.
- **Do not** silently fall back to Option C if A fails on the bench — that's a separate decision.

---

## 11. Open questions

1. **Auth-token delivery to the client.** Server checks `data.auth` on every socket event; the
   client currently sends `auth` only if it has the token. How does the operator app obtain it
   (settings entry / stored config / Jetson console line)? (Wiring-guide OQ1/OQ7.)
2. **§7.1 axis semantics** — confirm from PX4 `rover_differential` MANUAL mapping docs / bench
   whether turn is `r` (yaw) or `y` (roll), and whether throttle wants `z` in [0,1000] or
   [−1000,1000].
3. **Acquire sequencing UX** — require operator to arm + (implicitly) accept MANUAL, or have
   acquire drive arm+mode automatically? Recommend explicit for safety. (Audit R6.)
4. **`client_monotonic_ms` in acquire** — declared in the model but only `session_id` is read.
   Keep for schema parity or drop? (Wiring-guide OQ4.)
5. **Two client deadman models** (§6.3) — is that intentional product divergence, or should
   `Three_Wheel_v2` standardize on the hold-button? Backend is agnostic either way.

---

## 12. Reference index

| Concept | Branch @ sha | File:symbol |
|---|---|---|
| Lease FSM / dead-man / watchdog | `feat/entry-pivot-recenter` @ `deb8cab` | `server/joystick_controller.py:JoystickController` |
| MANUAL_CONTROL transport + gateway | same | `server/manual_control_gateway.py` (`encode_manual_control`, `ManualControlGateway._run`) |
| Single-owner arbitration | same | `server/control_arbiter.py:ControlArbiter` |
| Socket handlers | same | `server/sockets/events.py` (`joystick_acquire/command/release`) |
| Config + invariant checks | same | `server/config.py` (`JOYSTICK_*`, `validate`) |
| Full client↔server contract | same | `docs/REACT_NATIVE_JOYSTICK_V2_WIRING_GUIDE.md` |
| Route comparison + risk register | same | `docs/virtual_joystick_audit.md` (§4, §10 R1–R10, §11 phases) |
| Client app (target) | `Three_Wheel_v2` @ `CSV-alignment` | `src/hooks/useVirtualJoystick.ts`, `src/components/{ManualJoystick,DeadmanButton}.tsx`, `src/utils/joystick{CommandScheduler,FrontendSafety,Math}.ts`, `src/types/joystick.ts` |
| Client app (secondary) | `DYX_GCS_V` @ `refactor` | `src/utils/joystick*`, `src/components/manual/{ManualJoystick,ManualDrivePanel}.tsx`, `src/types/px4/joystick.ts` |
| Integration anchors (target) | `Upgrade_Spray` @ `1193a43` | `server/main.py` (globals+lifespan), `server/ros_node.py:RosBridgeNode`, `server/emergency.py:estop_async`, `server/sockets/events.py` |

*End of plan.*
