# Frontend ↔ Backend ↔ ROS2 — System Architecture

This is the wiring contract every task in this folder must respect. If a task
file conflicts with this document, the architecture wins — fix the task spec
first, then implement.

## 1. Topology

```
+--------------------+        Wi-Fi LAN        +--------------------------+
|  Operator device   |  <===================>  |   Jetson Orin (rover)    |
|  (phone / laptop)  |                         |   192.168.1.102          |
+--------------------+                         +--------------------------+
| React 18 SPA       |                         | FastAPI (5001)           |
| (Babel-in-browser) |                         |   |                      |
|  +--------------+  |   Socket.IO (10 Hz)     |   +- Socket.IO server    |
|  | store.jsx    |<-+----------------------------+ |  telemetry, status  |
|  | api.jsx      |<-+   REST /api/* (cmds)    +-->|                      |
|  | screens/*    |  |                         |   +- REST routes        |
|  +--------------+  |   UDP 5002 (discovery)  |       arm/mode/estop/   |
|                    |<-+-------------------------+   mission/path/param |
+--------------------+                         |   |                      |
                                               | rclpy node (bg thread)   |
                                               |   |                      |
                                               | ROS2 topics + services   |
                                               |   |                      |
                                               | RPP pipeline nodes       |
                                               | (path_publisher,         |
                                               |  rpp_controller,         |
                                               |  twist_to_setpoint,      |
                                               |  mission_runner,         |
                                               |  xtrack_logger)          |
                                               |   |                      |
                                               | MAVROS → PX4 → RoboClaw  |
                                               +--------------------------+
```

## 2. Transport rules

- **Telemetry → Socket.IO only.** Never poll `/api/telemetry/latest` from a
  timer. Use the existing `onTelemetry` handler in store.jsx. REST `GET
  /api/telemetry/latest` exists only for a one-shot fetch after reconnect.
- **Commands → REST.** REST is idempotent, traceable in DevTools, and FastAPI
  generates OpenAPI docs. Socket.IO command events (`arm`, `set_mode`, …) exist
  as a backup transport but the frontend should prefer REST. Wire results back
  via the Socket.IO `arm_result` / `mode_result` events — don't only trust the
  REST response.
- **File uploads → REST multipart.** Use `fetch` with `FormData`. Do not base64
  inline into a Socket.IO event.
- **Discovery → UDP beacon on 5002.** Only the discovery picker reads this.
  Everything else uses the user-chosen base URL.

## 3. State ownership

| Concern | Owns | Reader |
|---|---|---|
| Connection state (socket open / closed / authed) | `api.jsx` | `store.jsx` via `backendConnected` |
| Telemetry stream | `store.jsx` (`t.*`) | every screen via `useRover()` |
| Mission lifecycle | backend (`offboard_controller.py`) | mirrored to `store.jsx` via `mission_status` and `mission_completed` |
| Path catalog | backend (`path_manager.py`) | fetched via REST, cached in `store.gallery` |
| PX4 params | PX4 FCU → backend via MAVROS | fetched via REST `/api/params`, cached in `store.px4Params` |
| Logs | backend file tail | streamed via Socket.IO `log` event (to be added in task 07) |
| Drawing state (waypoints, DXF, gallery) | `store.jsx` (client-side until upload) | screens |
| Tweaks (typeface, density, HUD toggles) | `tweaks-panel.jsx` + `app.jsx` | `app.jsx` data attributes |

**Key rule:** the *backend is the source of truth* for anything physical
(armed, mode, position, params). The frontend may show its intent immediately
(optimistic update) but must reconcile to the next Socket.IO event within 200 ms.

## 4. Telemetry mapping (backend → store.t)

The backend `ros_node.get_state()` publishes this shape (see
`server/ros_node.py` and `server/models.py`). The mapping in
`store.jsx::onTelemetry` is the contract — only extend it, never rename:

| Backend field | `t.*` field | Notes |
|---|---|---|
| `connected` | `connected` | MAVROS link, override after 2 s timeout |
| `armed` | `armed` | |
| `mode` | `mode` | "MANUAL" / "OFFBOARD" — display map: MANUAL→Manual, OFFBOARD→Mission/Draw |
| `battery_v` | `voltage` | |
| `battery_pct` | `battery` | 0-100 |
| `pos_n`, `pos_e` | not in `t` directly | passed to map overlay via separate store key |
| `heading_ned_deg` | `heading`, `yaw` | degrees, 0=North CW |
| `speed_m_s` | `speed` | |
| `rpp_state` | (to be added) `rppState` | int 0..3 → name via lookup |
| `gps_fix` | `fix` | int → "NO_FIX/2D/3D/3D_DGPS/RTK_FLOAT/RTK_FIXED" |
| `gps_sat` | `sats` | |
| `lat`, `lon`, `alt` | `lat`, `lon`, `alt` | for map |

Anything not in that list (`current`, `temp`, `hdop`, `rssi`, `motor[]`,
`rosDomain`, `nodesAlive`, `hz`) is **NOT in backend yet**. Leave the mock
value or request a backend extension via a new task.

## 5. Mock fallback contract

`store.jsx` has a mock tick that runs *only* when `backendConnected === false`.
Every screen must work with the mock. The rule:

- If `backendConnected`, the screen reads from real `t.*` plus any real backend
  fetches (paths, params, logs).
- If `!backendConnected`, the screen falls back to seeded mock data
  (`SEED_GALLERY`, `SEED_PX4_PARAMS`, `SEED_LOGS`, etc.). It must not throw, not
  show "loading…" forever, not show an error toast.

A small "offline / mock" badge belongs in the status bar (task 03).

## 6. Error display

- Socket-level errors → `backendError` (already wired) → banner.
- Per-command errors (e.g. arm rejected because not in OFFBOARD) → toast in
  the calling screen. Do not propagate to global `backendError` — that field
  is reserved for connection / safety-abort events.
- Safety abort (`safety_abort` event) → modal that requires explicit
  acknowledgement. After ack, `clearEStop()` re-enables UI but stays disarmed.

## 7. Auth

- Token comes from `localStorage["rover_token"]` (already wired).
- Set the token via the settings panel (task 03). If empty + backend has auth
  enabled, the first REST call returns 401 → surface via `backendError`.
- Dev mode: backend runs with `ROVER_DISABLE_AUTH=1`. No token needed. The
  frontend should still send the header if present — backend ignores it.

## 8. Network base URL

Currently hard-coded to `http://localhost:5001` in `api.jsx`. Task 10
(discovery picker) lifts this into `localStorage["rover_base_url"]` and a
settings UI. Until task 10 lands, all other tasks may continue to assume
localhost.

## 9. Adding a new feature — the loop

1. Confirm the backend endpoint / event exists (`server/routes/*` or
   `server/sockets/events.py`). If not, stop and write a backend-extension
   task — do not silently add Python.
2. Add the call to `api.jsx` (REST wrapper or Socket.IO handler).
3. Wire it through `store.jsx` if it's shared state, or call directly from
   the screen if it's screen-local.
4. Keep the mock fallback alive.
5. Verify in DevTools network tab that the call fires once per user action,
   not in a loop.
