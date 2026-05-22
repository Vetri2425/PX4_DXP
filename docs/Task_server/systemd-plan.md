# Systemd Deployment Plan — Drawing Rover Server + RPP Pipeline

**Date:** 2026-05-21
**Status:** Plan (ready to implement)

---

## 1. Current State on Jetson

| Service | What it does | Status |
|---------|-------------|--------|
| `px4-dxp.service` | MAVROS + NTRIP (via `px4_start_service.sh`) | ✅ Running, auto-restarts |
| `ntrip.service` | (Not separate — NTRIP is embedded inside `px4-dxp.service` as a watchdog subprocess) | N/A |
| FastAPI server | `server/run.sh` → uvicorn | ❌ Not managed by systemd |
| RPP pipeline | `src/launch/rpp_pipeline.launch.py` → 4 ROS2 nodes | ❌ Not managed by systemd |

---

## 2. How NRP_ROS `start_service.sh` Works (Lessons Learned)

The legacy script is a **monolithic process manager** that:
1. Sources ROS2 Humble
2. Kills stale processes, frees ports
3. Starts rosbridge_server (waits for `/rosbridge_websocket` node)
4. Starts MAVROS under a watchdog loop (auto-restarts on crash)
5. Starts GPS altitude corrector, telemetry node, power monitor
6. Starts the Python backend server (waits for port 5001)
7. `wait` on the backend PID — when it dies, the whole service dies

**Key patterns:**
- Single bash script owns everything — one `systemctl restart` brings up the whole stack
- Internal watchdog loops for crash-prone processes (MAVROS)
- `check_ros_node()` polling for readiness
- `wait_for_port()` for the backend server
- Cleanup via trap + CHILD_PIDS array
- All output goes to journald via `StandardOutput=journal`

**What works well:** simple, one service to manage, one log stream.
**What doesn't:** if the backend crashes, MAVROS dies too. No independent restart. No health checks.

---

## 3. How Our `px4_start_service.sh` Works

Same pattern as NRP_ROS but leaner:
1. Sources ROS2 Humble
2. Verifies `/dev/ttyACM0` exists (CubeOrangePlus USB)
3. Kills stale MAVROS/NTRIP processes
4. Starts `mavros_watchdog` (background loop — restarts MAVROS on crash)
5. Waits for MAVROS to be ready (35s timeout)
6. Validates FCU connection via `/mavros/state`
7. Starts `ntrip_watchdog` (background loop — restarts NTRIP on crash)
8. `wait` on both watchdog PIDs

**What's missing:** the RPP pipeline nodes and the FastAPI server are not started here.

---

## 4. Architecture Decision: Two New Services

Rather than stuffing everything into `px4-dxp.service` (NRP_ROS monolith pattern), we split into **three independent services** with explicit ordering:

```
px4-dxp.service          (existing — MAVROS + NTRIP)
       ↓ After=
rpp-pipeline.service     (NEW — twist_to_setpoint + rpp_controller + xtrack_logger)
       ↓ After=
rover-server.service     (NEW — FastAPI + Socket.IO backend)
```

**Why split:**
- MAVROS can restart independently without killing the server
- The server can restart independently without killing RPP
- RPP nodes are stateless — they reconnect to MAVROS topics automatically
- The server has its own health endpoint (`/api/healthz`) for monitoring
- Each service has its own journal log stream (`journalctl -u rover-server`)
- Failure isolation: a server bug doesn't take down the setpoint stream

**Ordering rationale:**
- `px4-dxp` must be up first (MAVROS provides topics)
- `rpp-pipeline` must be up second (twist_to_setpoint streams zeros for OFFBOARD pre-stream)
- `rover-server` comes last (it reads from RPP topics and calls MAVROS services)

---

## 5. Implementation Plan

### 5.1 Create `rpp_start.sh` (RPP pipeline startup script)

```
~/PX4_DXP/rpp_start.sh
```

Starts only the three always-on RPP nodes (NOT path_publisher, NOT mission_runner — those are server-driven now):
1. `twist_to_setpoint_node.py` — must start first (OFFBOARD heartbeat)
2. `rpp_controller_node.py` — path follower
3. `xtrack_logger_node.py` — telemetry CSV capture

Does NOT start `path_publisher_node.py` (the server publishes paths via `/path` topic).
Does NOT start `mission_runner_node.py` (the server owns OFFBOARD lifecycle).

Watchdog pattern: if any node dies, restart it. If all three die within 30s, exit (let systemd restart the whole service).

### 5.2 Create `rpp-pipeline.service`

```ini
[Unit]
Description=RPP Controller Pipeline (twist_to_setpoint + rpp_controller + xtrack_logger)
After=px4-dxp.service
Wants=px4-dxp.service
PartOf=px4-dxp.service

[Service]
Type=exec
User=flash
Group=flash
WorkingDirectory=/home/flash/PX4_DXP
ExecStart=/home/flash/PX4_DXP/rpp_start.sh
Restart=on-failure
RestartSec=5
...
```

`PartOf=px4-dxp.service` means: if MAVROS is stopped, RPP is stopped too.

### 5.3 Create `rover-server.service`

```ini
[Unit]
Description=Drawing Rover FastAPI Server
After=rpp-pipeline.service
Wants=rpp-pipeline.service

[Service]
Type=exec
User=flash
Group=flash
WorkingDirectory=/home/flash/PX4_DXP/server
ExecStart=/home/flash/PX4_DXP/server/run.sh
Restart=on-failure
RestartSec=5
WatchdogSec=30
...
```

The server's telemetry loop can send `sd_notify("WATCHDOG=1")` every 10s to prove liveness.

### 5.4 Update `deploy.sh`

Add symlinks for the two new service files + `systemctl daemon-reload`.

### 5.5 Update `server/run.sh`

Add `sd_notify` support (via `systemd-python` or a simple `socat` call) so `WatchdogSec=` works.

---

## 6. Startup Sequence (Full)

```
Boot
  ↓
network.target
  ↓
dev-ttyACM0.device (CubeOrangePlus USB detected)
  ↓
px4-dxp.service starts
  → MAVROS connects to FCU
  → NTRIP connects to RTK base
  → /mavros/state, /mavros/local_position/pose, etc. become available
  ↓ (After=px4-dxp.service)
rpp-pipeline.service starts
  → twist_to_setpoint_node streams zeros at 50 Hz (OFFBOARD pre-stream ready)
  → rpp_controller_node waits for /path (publishes zero velocity → IDLE)
  → xtrack_logger_node waits for data
  ↓ (After=rpp-pipeline.service)
rover-server.service starts
  → FastAPI + Socket.IO on port 5001
  → Subscribes to /rpp/debug, /mavros/state, etc.
  → Beacon broadcasts on UDP 5002
  → /api/healthz returns all-green
  ↓
Frontend connects via Socket.IO
  → Operator loads path, starts mission
  → Server publishes /path → RPP tracks → twist_to_setpoint streams velocity → PX4 drives motors
```

---

## 7. File Deliverables

| File | Purpose |
|------|---------|
| `PX4_DXP/rpp_start.sh` | RPP pipeline startup script (3 nodes + watchdog) |
| `PX4_DXP/rpp-pipeline.service` | systemd unit for RPP pipeline |
| `PX4_DXP/rover-server.service` | systemd unit for FastAPI server |
| `PX4_DXP/server/run.sh` | Updated with sd_notify support |
| `PX4_DXP/deploy.sh` | Updated to symlink new services |

---

## 8. Verification After Deployment

```bash
# On Jetson:
sudo systemctl daemon-reload
sudo systemctl enable rpp-pipeline.service rover-server.service

# Start the full stack:
sudo systemctl start px4-dxp.service
# (rpp-pipeline and rover-server start automatically via After= ordering)

# Verify:
systemctl status px4-dxp rpp-pipeline rover-server
journalctl -u rover-server -f   # watch server logs
curl http://localhost:5001/api/healthz
curl http://localhost:5001/api/paths
```

---

## 9. Rollback

If anything goes wrong:
```bash
sudo systemctl stop rover-server rpp-pipeline
# px4-dxp continues running (MAVROS + NTRIP unaffected)
# Manual testing: run nodes by hand
```
