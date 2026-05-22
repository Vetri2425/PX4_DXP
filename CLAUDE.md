# 3WD Marking Rover — Jetson Companion (Runtime Brain)

This is the **Jetson-side** CLAUDE.md. It lives in `~/PX4_DXP/` on the Jetson Orin at `192.168.1.102`. Scope = **runtime, ROS2, MAVROS2, on-device debugging**. Architecture/firmware/tuning lives on the laptop (see "Cross-Machine Workflow" below).

## Who this Claude is

- **Role:** Runtime engineer on the rover. Builds and runs ROS2 nodes, manages MAVROS2, captures bags, debugs live telemetry.
- **Not your job:** Editing PX4 firmware patches, generating mission `.waypoints` files, running GitHub Actions, analyzing logs offline at length. Those belong to **Laptop Claude** at `D:\Vetri\3WD_GCS\`.

## Hardware on this machine

| Item | Value |
|---|---|
| Host | Jetson Orin, hostname `flash`, user `flash` |
| OS | Ubuntu (Tegra 5.15.148), aarch64 |
| IP | `192.168.1.102` (eno1) |
| ROS2 | Humble (`/opt/ros/humble/`) |
| FCU link | `/dev/ttyACM0` @ 921600 (CubeOrangePlus, PX4 v1.16.2) |
| RTK link | UM982 on TELEM1 (CubeOrangePlus) — NTRIP injects RTCM via MAVROS |
| ROS_DOMAIN_ID | 0 (default) |

## Post-Pull Workflow (IMPORTANT)

After every `git pull`, follow these steps in order:

```bash
cd ~/PX4_DXP
git pull                          # 1. Get latest code

./deploy.sh                       # 2. Sync system files (symlinks + env)
                                  #    First run: prompts for NTRIP credentials
                                  #    Later runs: skips if already set up

# 3. Restart service to pick up changes
sudo systemctl restart px4-dxp.service

# 4. Verify
systemctl status px4-dxp.service   # should show "active (running)"
ros2 topic echo /mavros/state --once  # should show "connected: true"
```

**What `deploy.sh` does:**
- Symlinks `px4-dxp.service` → `/etc/systemd/system/` (future `git pull` auto-updates)
- Symlinks `ntrip.logrotate` → `/etc/logrotate.d/` (future `git pull` auto-updates)
- Creates NTRIP env file at `~/PX4_DXP/config/ntrip.env` (prompts once, skips if exists)
- Reloads systemd daemon + enables service

**Why symlinks?** Once deployed, `git pull` updates the repo files in-place. Since systemd and logrotate read through the symlink, **no re-deploy needed for content changes** — just restart the service. Only re-run `deploy.sh` if you add NEW files or change the service definition.

## Running services (do NOT casually restart)

### `px4-dxp.service` — MAVROS2 bridge
- Source: `~/PX4_DXP/px4-dxp.service` (symlinked to `/etc/systemd/system/`)
- Launcher: `~/PX4_DXP/px4_start_service.sh`
- NTRIP node: `~/PX4_DXP/ntrip_rtcm_node.py` (launched by start script)
- Config: `~/PX4_DXP/px4_pluginlists_rover.yaml`
- Log rotation: `~/PX4_DXP/ntrip.logrotate` (symlinked to `/etc/logrotate.d/`)
- Credentials: `~/PX4_DXP/config/ntrip.env` (gitignored — never committed)

**Status checks:**
```bash
systemctl status px4-dxp.service
ros2 topic list | grep mavros
ros2 topic echo /mavros/state --once
```

**Restart only when you have a reason:**
```bash
sudo systemctl restart px4-dxp.service
```

## Project folder layout on Jetson

```
~/PX4_DXP/                          ← THIS WORKSPACE (git repo)
  ├── CLAUDE.md                      ← this file
  ├── deploy.sh                      ← post-pull deployment script
  ├── px4_start_service.sh           ← service launcher
  ├── px4_pluginlists_rover.yaml     ← MAVROS plugin denylist
  ├── ntrip_rtcm_node.py             ← NTRIP RTK injector
  ├── px4-dxp.service                ← systemd unit (symlinked to system)
  ├── ntrip.logrotate                ← log rotation (symlinked to system)
  ├── config/                        ← local config (gitignored secrets)
  │   └── ntrip.env                  ← NTRIP credentials (gitignored, never committed)
  ├── docs/                          ← architecture docs
  │   ├── MAVROS_vs_DDS.md
  │   ├── Pure_DDS.md
  │   └── Progress/PROGRESS.md       ← running project log
  ├── src/                           ← ROS2 nodes (Phase 2)
  │   ├── rpp_controller_node.py     ← RPP controller (NED velocity output)
  │   ├── twist_to_setpoint_node.py  ← 50Hz PositionTarget streamer
  │   ├── path_publisher_node.py      ← test path publisher (hardcoded + QGC/CSV)
  │   ├── xtrack_logger_node.py      ← 20Hz CSV logger (18 columns)
  │   ├── mission_runner_node.py      ← OFFBOARD lifecycle state machine
  │   └── launch/rpp_pipeline.launch.py ← ordered startup
  ├── params/                        ← PX4 parameter files
  │   ├── Param_with_Roboclaw.params ← full param set (RBCLW_QPPS_MAX=0, must set!)
  │   └── README.md                  ← param file docs + safety table
  ├── server/                        ← FastAPI backend (Phase 2)
  │   ├── main.py                    ← FastAPI app + Socket.IO + telemetry loop
  │   ├── ros_node.py                ← rclpy bridge (subscribers + async service calls)
  │   ├── offboard_controller.py     ← OFFBOARD lifecycle (arm → OFFBOARD → run → stop)
  │   ├── path_manager.py            ← path loading (6 built-in + QGC + CSV)
  │   ├── rpp_status.py              ← RPP debug decoder + done-settle
  │   ├── emergency.py               ← e-stop: stop-path + MANUAL + disarm
  │   ├── beacon.py                  ← UDP discovery broadcast
  │   ├── auth.py                    ← shared-secret token auth
  │   ├── config.py, models.py       ← constants + Pydantic models
  │   ├── logging_setup.py           ← structured logging
  │   ├── routes/                    ← 6 REST route modules
  │   ├── sockets/events.py          ← Socket.IO event handlers
  │   ├── missions/                  ← uploaded .waypoints/.csv files (gitignored)
  │   ├── requirements.txt           ← pip dependencies
  │   ├── run.sh                     ← startup script (source ROS2 + uvicorn)
  │   └── ARCHITECTURE.md            ← full build specification
  └── (legacy files outside repo)

~/circle_drive.py, half_circle.py, square_drive.py, d_shape.py,
~/u_turn.py, u_turn_simple.py, test_arc.py, spin360.py, spin360_1.py
                                    ← legacy test scripts (review before reuse)
~/ardupilot/                        ← ardupilot tree (NOT used; ArduRover abandoned)
~/.kiro/steering/                   ← AI context files (laptop maintains these)
```

## Key design decisions

1. **All runtime files live inside `~/PX4_DXP/`** — `git pull` updates everything. No scattered files outside the repo.
2. **System files are symlinked, not copied** — `deploy.sh` creates symlinks so git changes propagate automatically. Just restart the service.
3. **NTRIP node is inside the repo** — old location `~/ntrip_rtcm_node.py` is dead. The start script references `$SCRIPT_DIR/ntrip_rtcm_node.py`.

## FastAPI Backend Server

**Location:** `~/PX4_DXP/server/`
**Port:** 5001 (default, configurable via `FASTAPI_PORT` env var)
**Discovery:** UDP broadcast on port 5002 every 2s

**Architecture:** Frontend (React/Vue) → FastAPI + Socket.IO → rclpy node → ROS2 topics/services → MAVROS → PX4

**Running the server:**
```bash
cd ~/PX4_DXP/server && bash run.sh       # production (requires token)
ROVER_DISABLE_AUTH=1 bash run.sh          # dev mode (no auth)
```

**API endpoints:** See `server/ARCHITECTURE.md` for full specification.
**Key endpoints:** `/api/ping`, `/api/arm`, `/api/set_mode`, `/api/estop`, `/api/mission/start`, `/api/mission/stop`, `/api/mission/status`, `/api/paths`, `/api/path/upload`, `/api/path/publish`, `/api/telemetry/latest`

**Socket.IO events:** `telemetry` (10Hz), `mission_status` (10Hz), `arm`, `set_mode`, `emergency_stop`, `mission_load/start/stop/abort`

**Auth:** Shared-secret token at `~/.rover_token` (auto-generated). Set `ROVER_DISABLE_AUTH=1` for LAN-only dev.

**Critical implementation details:**
- **Pure rclpy** — no roslibpy, no CLI fallback. Server runs on same Jetson as ROS2 nodes.
- **Async service calls** — `arm_async()`, `set_mode_async()` use `call_async` + `add_done_callback`, never block FastAPI event loop.
- **Stop-path, not empty Path** — RPP node ignores empty Path (early return). E-stop publishes single point at rover's current position instead.
- **ENU→NED conversion** — MAVROS pose is ENU frame. Server converts: `yaw_NED = π/2 - yaw_ENU`, `pos_n = pose.y`, `pos_e = pose.x`.
- **MAVROS process-crash detection** — TRANSIENT_LOCAL keeps last message with `connected=True` even after MAVROS dies. Server tracks `_state_recv_time` and overrides `connected=False` after 2s timeout.

## Phase 2 plan (active — see laptop `project_architecture_decision.md`)

**Goal:** Jetson becomes the brain. Publishes setpoints via MAVROS2 OFFBOARD; PX4 only does EKF + motor mix + safety.

**OFFBOARD rules (PX4-specific, easy to forget):**
1. You **MUST** stream setpoints at ≥2 Hz **BEFORE** requesting OFFBOARD mode, or PX4 rejects the mode switch.
2. If the stream drops for >0.5s once in OFFBOARD, PX4 exits to failsafe.
3. Never use `/mavros/set_mode` with `GUIDED` for PX4 — that name is ArduPilot. PX4 wants `OFFBOARD`.
4. For velocity control: `/mavros/setpoint_velocity/cmd_vel` (TwistStamped).
5. For position/path control: `/mavros/setpoint_raw/local` (PositionTarget) — preferred for arc following.

**Milestones:**
1. Verify MAVROS bridge → `ros2 topic echo /mavros/state` shows `connected: true`.
2. Build setpoint streamer node in `~/PX4_DXP/src/` — straight-line velocity first.
3. Arm + switch to OFFBOARD via service calls. Confirm motion.
4. Pure-pursuit arc controller node (later: MPC). Source path comes from laptop-generated waypoint files.
5. Capture rosbag of every test → push to laptop for analysis.

## Cross-Machine Workflow

| Task | Where it happens |
|---|---|
| Firmware patches, GitHub Actions builds | Laptop only |
| QGC param tuning, mission `.waypoints` generation | Laptop only |
| Architecture decisions, deep log analysis, memory system | Laptop only |
| ROS2 node code, launch files, OFFBOARD logic | **Jetson primary**, laptop can review via git |
| Running tests, capturing rosbags, restarting services | Jetson only |

**Bridge = git.** Push from whichever machine edits, pull on the other before touching. **One writer at a time.** Don't both edit the same file in the same session.

**Memory is per-machine.** Each Claude has its own `~/.claude/`. Don't try to sync. Cross-reference laptop memory files by name (e.g. "see laptop's `project_px4_migration.md` for firmware status") — never paste their contents here.

## Hard rules

- **Do not edit PX4 firmware** on Jetson. Patches live on laptop only (`D:\Vetri\3WD_GCS\PX4-Autopilot\`).
- **Do not stop `px4-dxp.service` without warning the user.** It carries the QGC bridge they're watching.
- **Do not disable RTK** (`ntrip_rtcm_node.py`) — survey accuracy depends on it.
- **Do not push to the FCU param storage** from Jetson without explicit instruction. QGC on laptop is the param source of truth.
- **ArduRover is abandoned.** `~/ardupilot/` is dead weight — do not propose ArduRover solutions.
- When unsure about firmware behavior, params, or build flow — **ask the user to check with Laptop Claude**.

## Quick commands

```bash
# Verify bridge alive
ros2 topic echo /mavros/state --once

# Watch live position
ros2 topic echo /mavros/local_position/pose

# Watch RTK fix status
ros2 topic echo /mavros/gpsstatus/gps1/raw --once

# Arm (only when safe — wheels off the ground or in safe zone)
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# OFFBOARD mode (only AFTER streaming setpoints for >1s)
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"

# Capture a bag
ros2 bag record /mavros/local_position/pose /mavros/setpoint_raw/local /mavros/state -o ~/bags/$(date +%Y%m%d_%H%M%S)

# View service logs
journalctl -u px4-dxp.service -f

# Post-pull deploy
cd ~/PX4_DXP && ./deploy.sh --restart

# Start FastAPI server (manual)
cd ~/PX4_DXP/server && bash run.sh

# Start FastAPI server (dev mode, no auth)
ROVER_DISABLE_AUTH=1 cd ~/PX4_DXP/server && bash run.sh

# Test server health
curl http://localhost:5001/api/ping

# Watch RPP debug
ros2 topic echo /rpp/debug --once

# Watch velocity output
ros2 topic echo /rpp/velocity_ned --once
```

## Useful pointers

- MAVROS pluginlist: `~/PX4_DXP/px4_pluginlists_rover.yaml`
- MAVROS PX4 config: `/opt/ros/humble/share/mavros/launch/px4_config.yaml`
- Service log: `journalctl -u px4-dxp.service -f`
- NTRIP credentials: `~/PX4_DXP/config/ntrip.env` (gitignored)
- QGC UDP port: 14550 (laptop QGC connects here)
- Date format in memory/files: `YYYY-MM-DD`