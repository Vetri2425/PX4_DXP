# PX4 DXP — Frontend / Backend Integration Architecture
## Dev & Verification Flow Only (Not Production)

---

## 1. Overview

The frontend (`front-end/`) is a React 18 SPA served statically via Babel in-browser.
The backend (`server/`) is a FastAPI + Socket.IO + ROS2 (rclpy) service.

This architecture wires them together so the same UI you already built drives the real rover backend, with graceful fallback to mock data when the backend is unreachable.

---

## 2. Communication Patterns

| Direction | Protocol | Use Case | Rate |
|-----------|----------|----------|------|
| Server ? Client | Socket.IO `telemetry` event | Live telemetry (battery, pose, RPP state, GPS) | 10 Hz |
| Server ? Client | Socket.IO `mission_status` | Mission lifecycle updates | 10 Hz |
| Server ? Client | Socket.IO `mission_completed` / `safety_abort` | One-shot events | On trigger |
| Client ? Server | REST POST `/api/arm` | Arm / disarm | On user action |
| Client ? Server | REST POST `/api/set_mode` | Set PX4 mode (MANUAL / OFFBOARD) | On user action |
| Client ? Server | REST POST `/api/estop` | Emergency stop | On user action |
| Client ? Server | REST POST `/api/mission/*` | Load / start / stop / abort | On user action |
| Client ? Server | Socket.IO `arm`, `set_mode`, `mission_*` | Alternative realtime commands | On user action |

### Why two transports?
- **Socket.IO** for telemetry: low latency, push-based, handles reconnect automatically.
- **REST** for commands: idempotent, easy to debug with `curl`, FastAPI generates OpenAPI docs.

---

## 3. Data Flow

```
+-------------------------------------------------------------+
¦                    PX4 DXP System                           ¦
+-------------------------------------------------------------¦
¦                                                             ¦
¦  +--------------+      Socket.IO (10Hz)     +------------+ ¦
¦  ¦   React      ¦ ?----------------------- ¦ FastAPI    ¦ ¦
¦  ¦   Frontend   ¦                            ¦ + SocketIO ¦ ¦
¦  ¦              ¦      REST POST /api/*       ¦            ¦ ¦
¦  ¦  +--------+ ¦ ------------------------? ¦            ¦ ¦
¦  ¦  ¦ api.jsx¦ ¦                            ¦  +------+  ¦ ¦
¦  ¦  ¦ store  ¦ ¦                            ¦  ¦ROS2  ¦  ¦ ¦
¦  ¦  ¦ tweaks ¦ ¦                            ¦  ¦Node  ¦  ¦ ¦
¦  ¦  +--------+ ¦                            ¦  +------+  ¦ ¦
¦  +--------------+                            ¦     ¦      ¦ ¦
¦                                              ¦  MAVROS    ¦ ¦
¦                                              ¦     ¦      ¦ ¦
¦                                              ¦  PX4 FCU   ¦ ¦
¦                                              +------------+ ¦
¦                                                             ¦
+-------------------------------------------------------------+
```

---

## 4. Auth Strategy (Dev Mode)

The backend uses token auth (`X-Rover-Token` header / Socket.IO `auth.token`).

For **dev flow only**:
```bash
export ROVER_DISABLE_AUTH=1   # backend bypasses all auth checks
```

If you want auth:
```bash
# Backend generates ~/.rover_token on first start
# Frontend stores it in localStorage and sends with every request
```

---

## 5. File Changes

| File | Action | Purpose |
|------|--------|---------|
| `front-end/PX4 DXP.html` | **Edit** | Add `socket.io-client` CDN script |
| `front-end/lib/api.jsx` | **Create** | REST + Socket.IO client wrapper |
| `front-end/lib/store.jsx` | **Edit** | Wire real backend; keep mock fallback |
| `front-end/screens/home.jsx` | **Edit** | Connect buttons to API actions |
| `front-end/screens/drive.jsx` | **Edit** | Wire arm/setMode to API |
| `front-end/screens/map.jsx` | **Edit** | Wire mission controls to API |

---

## 6. Running the Dev Setup

### Terminal 1 — Backend
```bash
cd D:\Vetri\3WD_GCS\PX4_DXP\server
pip install -r requirements.txt
$env:ROVER_DISABLE_AUTH="1"
uvicorn main:app --reload --host 0.0.0.0 --port 5001
```

### Terminal 2 — Frontend (static file server)
```bash
cd D:\Vetri\3WD_GCS\PX4_DXP\front-end
python -m http.server 3000
# Then open http://localhost:3000
```

### Or just open the HTML file directly
Since the frontend is static and CORS is enabled (`*` on backend), you can open `PX4 DXP.html` directly in a browser. The API calls will cross-origin to `http://localhost:5001`.

---

## 7. Fallback Behavior

When the backend is unreachable:
- `store.jsx` detects disconnect after 3 seconds
- Mock telemetry interval resumes automatically
- All UI controls still work for layout verification
- A small "offline / mock" badge appears in the status bar

This is intentional for dev flow — you can tweak CSS and layout without needing the rover powered on.

---

## 8. API Surface Quick Reference

### REST Endpoints
```
GET  /api/telemetry/latest          ? TelemetryData
GET  /api/mission/status           ? MissionStatus
POST /api/arm                      ? ArmRequest { arm: bool }
POST /api/set_mode                 ? ModeRequest { mode: "MANUAL" | "OFFBOARD" }
POST /api/estop                    ? EstopResponse
POST /api/mission/load             ? MissionLoadRequest { path_name?, mission_file? }
POST /api/mission/start            ? MissionStartRequest
POST /api/mission/stop             ? {}
POST /api/mission/abort            ? {}
```

### Socket.IO Server ? Client Events
```
telemetry          ? { battery, voltage, pos_n, pos_e, heading, ... }
mission_status     ? { state, rpp_state, rpp_state_name, ... }
mission_completed  ? { state, name }
safety_abort       ? { reason, pose_age_ms, rpp_state }
rover_disconnected ? {}
arm_result         ? { success, arm, message }
mode_result        ? { success, mode, message }
```

### Socket.IO Client ? Server Events
```
arm                ? { arm: bool, auth? }
set_mode           ? { mode: string, auth? }
emergency_stop     ? { auth? }
mission_load       ? { path_name, auth? }
mission_start      ? { auth? }
mission_stop       ? { auth? }
mission_abort      ? { auth? }
```

---

## 9. Telemetry Mapping (Backend ? Frontend)

| Backend Field (`ros_node.get_state()`) | Frontend Store (`t.*`) |
|----------------------------------------|--------------------------|
| `connected` | `connected` |
| `armed` | `armed` |
| `mode` | `mode` |
| `battery_v` / `battery_pct` | `voltage` / `battery` |
| `pos_n`, `pos_e` | used for map overlay |
| `heading_ned_deg` | `heading`, `yaw` |
| `speed_m_s` | `speed` |
| `rpp_state` | RPP state pill |
| `gps_fix`, `gps_sat` | `fix`, `sats` |
| `lat`, `lon`, `alt` | GPS readout |

---

## 10. Safety Notes

- The backend already has safety abort on pose-stale (`POSE_STALE_MS = 500ms`, grace `1s`).
- Frontend estop button calls `/api/estop` which triggers: stop-path ? MANUAL ? disarm.
- Mission start is guarded: no start without loaded path, no start while RUNNING.
- **This integration is for bench testing only.** Do not run motors unattended.
