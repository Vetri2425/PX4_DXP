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

## Running services (do NOT casually restart)

### `px4-dxp.service` — MAVROS2 bridge
- File: `/etc/systemd/system/px4-dxp.service`
- Launcher: `~/PX4_DXP/px4_start_service.sh`
- Brings up `mavros_node` with:
  - `fcu_url:=/dev/ttyACM0:921600`
  - `gcs_url:=udp-b://:14550@` (QGC bridge)
  - `pluginlists:=~/PX4_DXP/px4_pluginlists_rover.yaml`
- Also launches `~/ntrip_rtcm_node.py` → publishes to `/mavros/gps_rtk/send_rtcm`

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
~/PX4_DXP/                       ← THIS WORKSPACE (source folder)
  ├── px4_start_service.sh       ← service launcher (do not break)
  ├── px4_pluginlists_rover.yaml ← MAVROS plugin allowlist
  ├── CLAUDE.md                  ← this file
  └── (future) src/, launch/, scripts/ ← Phase 2 ROS2 nodes go here

~/ntrip_rtcm_node.py             ← RTK injector (referenced by service)
~/circle_drive.py, half_circle.py, square_drive.py, d_shape.py,
~/u_turn.py, u_turn_simple.py, test_arc.py, spin360.py, spin360_1.py
                                 ← legacy test scripts (review before reuse)
~/ardupilot/                     ← ardupilot tree (NOT used; ArduRover abandoned 2026-05-18)
~/.kiro/steering/                ← AI context files (laptop maintains these)
```

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
| systemd service edits | Jetson only |

**Bridge = git.** When `~/PX4_DXP/` becomes a git repo, push from whichever machine edits, pull on the other before touching. **One writer at a time.** Don't both edit the same file in the same session.

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
```

## Useful pointers

- MAVROS pluginlist: `~/PX4_DXP/px4_pluginlists_rover.yaml`
- MAVROS PX4 config: `/opt/ros/humble/share/mavros/launch/px4_config.yaml`
- Service log: `journalctl -u px4-dxp.service -f`
- QGC UDP port: 14550 (laptop QGC connects here)
- Date format in memory/files: `YYYY-MM-DD`
