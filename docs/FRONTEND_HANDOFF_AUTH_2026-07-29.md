# Frontend Handoff — Auth is now ENFORCED (2026-07-29)

**To:** Three_Wheel_v2 (Expo / React Native operator app) developer
**From:** rover backend
**Backend ref:** branch `prod/audit-2026-07-29` @ `9f42559`, deployed to Jetson `192.168.1.102`

---

## ⚠️ BREAKING — read this first

**Operator authentication was OFF from 2026-07-11 until today. It is now ON.**

Every API request except the two public health endpoints now returns **401** without a valid token.
Socket.IO now **refuses the connection** without one. If the app is not already sending
`X-Rover-Token`, it will fail completely against this rover.

This is a deliberate security fix, not a regression. Backend rollback exists but will not be used
except in an emergency.

---

## 1. What the app must do

### 1.1 Log in

```
POST /api/auth/login
Content-Type: application/json

{ "password": "<operator password>" }
```

**200 response:**

```json
{
  "token": "…",
  "session_id": "…",
  "expires_at": "2026-07-30T00:55:00Z",
  "ttl_s": 43200,
  "must_change_password": false
}
```

| Status | Meaning | App behaviour |
|---|---|---|
| 200 | success | store `token`, proceed |
| 401 | `{"detail":"Invalid password"}` | wrong password — re-prompt |
| 503 | operator password not configured | show "rover not set up" — see §3 |

Session TTL is **12 h** (`ttl_s`). Persist the token; do not re-login on every launch.

### 1.2 Send the token on EVERY request

```
X-Rover-Token: <token>
```

Exact header name — `config.py:191`, `TOKEN_HEADER_NAME = "X-Rover-Token"`.

### 1.3 Send the token on the Socket.IO handshake

The connect handler reads it from the `auth` payload (`sockets/events.py`):

```js
io(url, { auth: { token } })      // preferred
io(url, { auth: token })          // also accepted (bare string)
```

**Without it the server raises `ConnectionRefusedError("unauthorised")` and the socket never
connects.** There is no telemetry at all until this is done — this is the single most likely
cause of "app shows nothing" after this change.

### 1.4 Handle 401 globally

A 401 on any call means the session expired or was revoked (a password change revokes all other
sessions). Clear the stored token, drop back to the login screen, reconnect the socket after
re-login. Do not retry-loop on 401.

---

## 2. Public endpoints — no token needed

| Endpoint | Use |
|---|---|
| `GET /api/ping` | reachability / "is the rover there" |
| `GET /api/healthz` | health probe |

Use `/api/ping` for the connection indicator, **not** a protected route — otherwise a logged-out
app looks like an offline rover.

UDP discovery beacon (port 5002) already carries `auth_required: true` — use it to decide whether
to show the login screen before the first request.

---

## 3. NEW — forced password rotation (`must_change_password`)

A rover that has never been configured now boots with a **documented default password** so a new
headless site can be set up from the app with no laptop and no SSH. That default **must** be
rotated, and the backend enforces it — this is not advisory.

### Behaviour while the default is in force

- `POST /api/auth/login` with the default **succeeds** and returns `must_change_password: true`
- **every other endpoint returns 403:**
  ```json
  { "detail": { "code": "password_change_required",
                "message": "Default bootstrap password must be changed before operating the rover" } }
  ```
- **Socket.IO connect is refused** — no telemetry either
- only `POST /api/auth/change-password` and `POST /api/auth/logout` work

### What the app must build

On a successful login with `must_change_password: true`, go **straight to a mandatory
change-password screen**. Do not show the dashboard, do not attempt the socket connection, do not
offer a skip. Everything is 403 until it is done, so any other screen will just render errors.

```
POST /api/auth/change-password
X-Rover-Token: <token from the default login>

{ "current_password": "<the default>", "new_password": "<min 8 chars>" }
```

**200 response:**

```json
{ "token": "…", "session_id": "…", "expires_at": "…", "ttl_s": 43200, "revoked_sessions": 2 }
```

Replace the stored token with the **new** one — the old session is revoked. Then connect the socket
and proceed normally.

| Status | Meaning | App behaviour |
|---|---|---|
| 200 | rotated | swap token, proceed |
| 401 | current password wrong | re-prompt |
| 409 | blocked — a mission is active | tell the operator to stop the mission first |
| 422 | new password < 8 chars | validate client-side first |

**Ask the backend team for the current default value.** It is deliberately not written in this doc
and there is **no unauthenticated endpoint that exposes it** — telling any caller "this rover is on
its default password" would hand an attacker the exact string to try. The app learns the state only
from a *successful* login.

> **Not urgent for the current rover.** `192.168.1.102` already has a real operator password from
> 2026-07-11, so `must_change_password` is `false` there and this screen never triggers. It is
> required before the **next new site** can be set up without a laptop.

---

## 4. Also changed today — worth knowing, no app work required

### 4.1 Spray will refuse more often. This is correct.

The cross-track gate tightened from **10 cm → 3 cm** to match the ±2 cm marking spec. The rover now
refuses to spray in situations it previously allowed.

**Do not treat this as an error state.** Surface the reason string verbatim — it now ends in
`(param)` or `(mission)` so the operator can tell which knob refused:

```
xtrack error 0.047m > 0.030m (param)
```

`(param)` = the node's ROS default · `(mission)` = a per-mission override sent with the session
config. If the app currently shows a generic "spray failed", it will now show it much more often and
the operator will have no idea why.

### 4.2 `safety_abort` still arrives — unchanged contract

The E-stop watchdog moved into its own backend task, but the `safety_abort` Socket.IO event and its
payload (`reason`, `pose_age_ms`, `rpp_debug_age_ms`, `rpp_state`, `rpp_state_name`, `connected`)
are unchanged. No app change needed. Mentioned only so it is not a surprise if you are reading the
backend diff.

### 4.3 Manual joystick control is DISABLED on this rover

`ROVER_JOYSTICK_MANUAL_ENABLED` is now `0` (it had been enabled against the documented firmware
gate). Joystick acquire will reject with `manual_control_disabled`. If the app has a joystick
screen, it should show that state cleanly rather than hanging on acquire. **Do not build around
re-enabling it** — that is a backend/firmware decision pending the §8 J3 bench gates.

### 4.4 RTK stays operator-started

Unchanged, but confirming: RTK is **not** automatic. The operator starts it from the app:

```
POST /api/rtk/ntrip/start      POST /api/rtk/stop      GET /api/rtk/status
```

Until it is started the rover sits at `3D_FIX` and the controller correctly refuses to drive
(`GPS fix_type=3 (need 6=RTK_FIXED)`). If the app does not make the RTK state obvious, an operator
will read "refusing to drive" as a fault.

---

## 5. Test checklist before you call it done

- [ ] Fresh install, no stored token → login screen, not a crash
- [ ] Login → token stored → dashboard loads → **socket connects and telemetry streams**
- [ ] Kill the token (edit storage) → next call 401 → app returns to login, no retry loop
- [ ] App relaunch inside 12 h → still logged in, no re-prompt
- [ ] Wrong password → clear message, no lockout of the UI
- [ ] `/api/ping` used for the connectivity indicator, not a protected route
- [ ] Mandatory rotation screen fires on `must_change_password: true` and blocks everything else
- [ ] After rotation, the **new** token is stored and the socket reconnects with it
- [ ] Spray refusal reason string is displayed verbatim, including the `(param)` / `(mission)` suffix

---

## 6. If the app is broken and you need the rover working now

Tell the backend team. Auth can be turned back off in one command on the Jetson while you fix the
client — the previous configuration is backed up at
`~/deploy_backups/50-auth-disabled.conf.bak-20260729`.

**Do not work around auth in the client** (hardcoded tokens, skipping the header, disabling the
socket auth). Ask for the rollback instead.

---

## Questions → backend team

- the current bootstrap default password value (§3)
- whether a given site wants a per-mission `max_xtrack_error_m` override (§4.1)
- anything in §5 that fails
