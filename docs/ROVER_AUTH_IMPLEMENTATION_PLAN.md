# Implementation Plan — Rover Local Auth (Client Communication)

**Status:** Epics A1–A4 implemented on `test/colinear-fix` (2026-07-11). Client cutover (A5) pending.  
**Date:** 2026-07-11  
**Base branch:** `test/colinear-fix` @ `cb2d6d5` (Epics 1–2 placement already on tree)  
**Auth reference (audit only):** `fix/runtime-entry-stop` **branch tip** `277d3dc` (RPP completion-latch commit — contains auth, is **not** itself an auth commit). Do **not** blind-merge the tip.  
**Primary commits on ref (auth-only):** `e9df43e` (feat), `4146aec` (disable bypass), `84aaadb` (Socket.IO honor disable)  
**Canonical ops doc:** `docs/ROVER_LOCAL_AUTH.md`  
**Plan audit:** Opus verified all load-bearing claims (2026-07-11). **GO** — implemented.

### Audit resolutions
- Open Q3: baseline **`GET /api/mission/loaded-path` exists** — machine scope shipped in v1.
- Tip `277d3dc` is audit anchor only; ported auth ideas/files, not the completion-latch RPP diff.

### Implementation progress
- **A1–A4 done:** password/session/`auth.py`, `/api/auth/*`, route cutover (telemetry protected; mission status/loaded-path/activity scoped), Socket.IO connect auth + authenticated emits, CLI, gitignore, bag_autorecord machine token path, beacon `auth_required`.
- **A5 pending:** mobile login + Jetson one-time `rover_auth_cli.py setup`.
---

## 0. Goal

Give the mobile / browser client a **stable, human-login auth layer** so it can:

1. Discover the rover and know auth is required.
2. **Log in** with an operator password.
3. Call protected REST with a short-lived **session token**.
4. Open Socket.IO once with that token and receive telemetry / send control.
5. Log out / change password safely.
6. Keep bag auto-record working with a **separate read-only machine token**.

Without this, the client cannot speak the protocol the July app already expects, and baseline’s static `~/.rover_token` model is awkward for multi-device operators.

---

## 1. Locked decisions (from reference audit)

| # | Decision |
|---|---|
| A1 | **Operator password** (PBKDF2-HMAC-SHA256) → **random session token**. Never store plaintext password or raw session tokens on disk. |
| A2 | Wire header stays **`X-Rover-Token`** (client already uses this name). Session token value replaces the old static shared secret. |
| A3 | Socket.IO authenticates **once at connect** (`auth: { token }`). After that, trust SID membership — **not** a password on every event. |
| A4 | **Do not auto-generate** an operator password. First boot = CLI setup (`rover_auth_cli.py setup`). |
| A5 | **Do not migrate** `~/.rover_token` / `config/rover_token` into a password. One-time Jetson setup; document cutover. |
| A6 | Machine token for **bag-autorecord only**, scoped allowlist: `mission:status`, `mission:loaded-path`, `activity:read`. No control / spray / RTK / params / password APIs. |
| A7 | Keep **`/api/ping`** and **`/api/healthz`** (and bridge health) **public**. Protect telemetry snapshot + activity for authenticated parties (operator **or** machine where scoped). |
| A8 | Dev bypass: honor disable flag on **both** REST and Socket.IO connect (ref bugfix `84aaadb`). Prefer accepting **both** env names during cutover: `ROVER_DISABLE_AUTH` (baseline) and `ROVER_AUTH_DISABLED` (ref). |
| A9 | Port **auth module + routes + tests + Jetson CLI + docs** only. Do **not** drag July mission/entry/joystick/point-mission code. If a ref auth call site needs `joystick_ctrl` and baseline has no joystick, gate on mission state alone. |
| A10 | Password change blocked while mission is active; rotate requester session; revoke other operator sessions; disconnect their sockets. |

---

## 2. Baseline vs reference (verified)

### 2.1 Baseline today (`test/colinear-fix`)

| Piece | Behavior |
|---|---|
| `server/auth.py` | Single shared secret from `~/.rover_token`; **auto-creates** if missing |
| REST | `Depends(require_token)` on mission/path/vehicle/spray/params/rtk… |
| Telemetry REST | **`/api/telemetry/latest` is open** (no auth) |
| System | `/ping`, `/healthz`, activity — open |
| Socket.IO | Per-event `data.auth` checked via `check_socket_token` |
| Login API | **None** |
| Machine tokens | **None** |
| Disable | `ROVER_DISABLE_AUTH=1` |

### 2.2 Reference (`fix/runtime-entry-stop` @ `277d3dc`) — keep these *ideas*

| Piece | Behavior |
|---|---|
| `server/auth.py` (~515 LOC) | Password hash file + in-memory sessions (SHA-256 token ids) + machine token registry |
| `server/routes/auth.py` | `POST /login`, `/logout`, `/change-password` |
| `server/rover_auth_cli.py` | `setup`, `reset-password`, `create-machine-token` |
| Socket.IO | `bind_socket_sid` on connect / `unbind` on disconnect; `socket_authenticated(sid)` |
| Telemetry push | `_emit_authenticated` → only SIDs in `authenticated_sids()` |
| Scoped deps | `require_operator_or_machine("mission:status"|…)` |
| Tests | `server/test_auth.py` (hash, login/revoke, machine scopes, password rotate) |
| Doc | `docs/ROVER_LOCAL_AUTH.md` |

### 2.3 Explicit non-goals (v1 on baseline)

- Blind `git checkout` of July `server/auth.py` without rewiring baseline call sites.
- Porting joystick / point-mission / spray-mode routers that do not exist on this tree.
- OAuth / cloud identity / multi-tenant accounts.
- Persisting operator sessions across `rover-server` restarts (ref is in-memory; keep that — clients re-login).
- Keeping auto-generated static `~/.rover_token` as a parallel operator path (cut over cleanly).

---

## 3. Client contract (what the app must do)

```
┌──────────────┐   UDP/beacon (auth_required=true)    ┌──────────────┐
│ Mobile / Web │ ───────────────────────────────────► │ rover-server │
│    client    │                                      │              │
│              │  POST /api/auth/login {password}     │              │
│              │ ◄──── { token, session_id, expires } │              │
│              │                                      │              │
│              │  REST: X-Rover-Token: <session>      │              │
│              │  Socket.IO connect auth: { token }   │              │
│              │ ◄──── telemetry @ ~10 Hz             │              │
└──────────────┘                                      └──────────────┘
```

| Transport | Credential | Notes |
|---|---|---|
| REST control / plan / spray / … | `X-Rover-Token: <session>` | Same header name as baseline |
| Socket.IO | connect `auth: { token: <session> }` | No per-event password after connect |
| Bag auto-record | machine token file | Only allowlisted GETs |
| Public | `/api/ping`, `/api/healthz`, `/api/health/bridge` | Discovery / probes |

**Breaking change vs baseline static token:** old clients that paste `~/.rover_token` forever must switch to login → session. Coordinate mobile app release with Jetson deploy.

---

## 4. Architecture (server)

```
config/
  rover_password.json          # PBKDF2 record only (0600, gitignored)
  rover_machine_tokens.json    # token_id + scopes (0600, gitignored); raw token NEVER stored
  bag_autorecord.token         # raw machine token for service (0600, gitignored)

server/auth.py                 # hash, sessions, machine validate, deps, socket bind
server/routes/auth.py          # login / logout / change-password
server/rover_auth_cli.py       # Jetson first-setup CLI
server/test_auth.py

Wiring:
  main.init_auth()
  include_router(auth_router)
  sockets: bind on connect; _auth_ok(sid)
  telemetry loop: emit only to authenticated_sids()
  mission status / loaded-path / activity: require_operator_or_machine(scope)
  everything else control-ish: require_token (= operator session)
```

### 4.1 Session model (from ref — re-implement, don’t paste blindly)

- Login → `secrets.token_urlsafe(32)`; store only `sha256(token)` as `token_id`.
- TTL default **12 h** (`ROVER_SESSION_TTL_S`).
- Logout / password-change / expiry → revoke; disconnect bound SIDs.
- `AUTH_DISABLED` → REST + Socket.IO both bypass (same `_BYPASS_CONTEXT`).

### 4.2 Machine token scopes (exact allowlist)

| Scope | Endpoint(s) on baseline |
|---|---|
| `mission:status` | `GET /api/mission/status` |
| `mission:loaded-path` | `GET /api/mission/loaded-path` (if present; else skip until route exists) |
| `activity:read` | `GET /api/activity` (+ related activity list if any) |

Any other route stays operator-only.

---

## 5. Work breakdown

### Epic A1 — Core auth module + config + gitignore
1. Add config knobs: `AUTH_PASSWORD_FILE`, `AUTH_MACHINE_TOKENS_FILE`, `AUTH_SESSION_TTL_S`, `AUTH_PBKDF2_ITERATIONS`, unified `AUTH_DISABLED` (read both env names).
2. Replace `server/auth.py` with password/session/machine design (re-derived from ref; keep public names `require_token`, `init_auth`).
3. Gitignore password / machine / bag token files; **never** commit real secrets (ref’s sample JSON is structure-only — regenerate on Jetson).
4. Unit tests ported/adapted from `test_auth.py` (hash, login, revoke, machine scope deny, password rotate).

### Epic A2 — HTTP API + route dependency cutover
1. Add `server/routes/auth.py`; mount at `/api/auth/*`.
2. Change-password: block if `offboard_ctrl.state` not in idle/completed/aborted/error; **omit joystick check** unless joystick exists on this branch.
3. Switch telemetry router to `require_token` (operator). Decide explicitly: machine tokens do **not** get full telemetry (ref: telemetry is operator-only).
4. Apply `require_operator_or_machine(...)` only on the three allowlisted GETs.
5. Leave ping/health public.
6. Remove reliance on auto-created `~/.rover_token` for operator auth.

### Epic A3 — Socket.IO + telemetry push
1. Connect: `bind_socket_sid(sid, token)` or refuse.
2. Disconnect: `unbind_socket_sid`.
3. Control handlers: `_auth_ok(sid)` instead of per-payload static token.
4. Telemetry / mission_status pushes: `_emit_authenticated` → `authenticated_sids()` only.
5. Password-change / logout: disconnect revoked SIDs (`auth_revoked` event optional but useful for client UX).

### Epic A4 — Jetson ops + bag-autorecord
1. Add `rover_auth_cli.py`.
2. Document setup in `docs/ROVER_LOCAL_AUTH.md` (copy ideas from ref; paths relative to this repo).
3. Wire `bag-autorecord` to read `config/bag_autorecord.token` and send `X-Rover-Token`.
4. `deploy.sh` / service notes: restart `rover-server` (+ bag-autorecord) after setup; **do not** restart `px4-dxp` for auth-only.

### Epic A5 — Client + field cutover
1. Mobile: login screen → store session → REST header + Socket.IO auth.
2. Handle 401 → re-login; handle `auth_revoked` → clear session.
3. Jetson one-time: `setup` + `create-machine-token`; confirm old `~/.rover_token` unused.
4. Beacon: ensure `auth_required` reflects reality when password configured / not disabled.

### Suggested order
`A1 → A2 → A3 → tests green → A4 on Jetson → A5 client`.  
Do **not** mix this PR with Epic 3 firmware entry or RPP changes.

---

## 6. Migration / cutover checklist (Jetson)

```bash
cd ~/PX4_DXP
# after code deploy
python3 server/rover_auth_cli.py setup
python3 server/rover_auth_cli.py create-machine-token --name bag-autorecord
# paste printed token into config/bag_autorecord.token (mode 0600)
./deploy.sh   # if unit files changed
sudo systemctl restart rover-server
sudo systemctl restart bag-autorecord
```

| Before | After |
|---|---|
| Client holds forever-token from `~/.rover_token` | Client logs in; holds session TTL |
| Socket sends `auth` on every event | Socket connects with session once |
| Bag uses shared operator token (if any) | Bag uses scoped machine token |

---

## 7. Acceptance criteria

**Server**
- [ ] No password file → login returns 503; control routes 401; CLI can create file.
- [ ] Wrong password → 401; right password → token; token works on `X-Rover-Token`.
- [ ] Logout / expiry → subsequent REST + socket fail.
- [ ] Machine token can hit only allowlisted GETs; denied on `/api/mission/start`, spray, path plan, etc.
- [ ] `AUTH_DISABLED` / `ROVER_DISABLE_AUTH` bypasses REST **and** Socket.IO connect.
- [ ] Password change mid-mission → 409; when allowed → other sessions disconnected.
- [ ] `pytest server/test_auth.py` + existing route suites green.

**Client**
- [ ] Login → telemetry stream → arm/start still work end-to-end on LAN.
- [ ] Cold start without token cannot control vehicle.

**Ops**
- [ ] Password/machine files mode `0600`, gitignored.
- [ ] Auth-only deploy does not restart `px4-dxp`.

---

## 8. Risks & contingencies

| Risk | Mitigation |
|---|---|
| Mobile app still expects static token | Ship app login in same window as Jetson auth deploy; temporary `AUTH_DISABLED` only on isolated bench |
| In-memory sessions lost on `rover-server` restart | Expected; client re-logins; document |
| Telemetry was open on baseline; locking it breaks scrapers | Update `capture_telemetry.py` / any curl recipes to login or use disable on bench |
| Accidental paste of July mission code with auth | Review PR file list: auth/config/sockets/main/routes only |
| Env name mismatch (`ROVER_DISABLE_AUTH` vs `ROVER_AUTH_DISABLED`) | Read both; document one canonical going forward |

---

## 9. Open questions (resolve before / during A5)

1. Should `/api/telemetry/latest` stay briefly open during cutover, or hard-cut with app release?
2. Session TTL: keep 12 h, or shorter for field tablets shared across operators?
3. ~~Does baseline `GET /api/mission/loaded-path` exist yet?~~ **Resolved (Opus):** yes — machine scope included in v1.
4. Beacon publisher: who sets `auth_required` — always true once password file exists?

---

## 10. One-line summary

**Replace baseline’s forever shared-secret with ref-style password login → session token (same `X-Rover-Token` header), connect-time Socket.IO auth, and a scoped machine token for bag-autorecord — re-derive on `test/colinear-fix`, do not blind-merge July stacks.**
