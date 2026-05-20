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
| RTK link | `/dev/ttyUSB0` (UM982 dual-antenna) |
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
  │   └── Pure_DDS.md
  └── (future) src/, launch/         ← Phase 2 ROS2 nodes

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
```

## Useful pointers

- MAVROS pluginlist: `~/PX4_DXP/px4_pluginlists_rover.yaml`
- MAVROS PX4 config: `/opt/ros/humble/share/mavros/launch/px4_config.yaml`
- Service log: `journalctl -u px4-dxp.service -f`
- NTRIP credentials: `~/PX4_DXP/config/ntrip.env` (gitignored)
- QGC UDP port: 14550 (laptop QGC connects here)
- Date format in memory/files: `YYYY-MM-DD`