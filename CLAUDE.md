# 3WD Marking Rover — Jetson Companion (Runtime Brain)

Scope: **runtime, ROS2, MAVROS2, on-device debugging** on Jetson Orin at `192.168.1.102`.
Not your job: PX4 firmware, `.waypoints` generation, GitHub Actions, offline log analysis — those belong to Laptop Claude at `D:\Vetri\3WD_GCS\`.

## Hardware

| Item | Value |
|---|---|
| IP | `192.168.1.102` (eno1), user `flash` |
| OS | Ubuntu aarch64, ROS2 Humble (`/opt/ros/humble/`) |
| FCU | `/dev/ttyACM0` @ 921600 (CubeOrangePlus, PX4 v1.16.2) |
| RTK | UM982 on TELEM1 — NTRIP injects RTCM via MAVROS |
| ROS_DOMAIN_ID | 0 |

## After git pull

```bash
git pull
./deploy.sh          # sync symlinks + env (prompts for NTRIP once, skips after)
sudo systemctl restart rpp-pipeline   # or px4-dxp if bridge files changed
```

**Restart the narrowest service:**

| Changed | Restart | Drops MAVROS? |
|---|---|---|
| `src/*.py` | `sudo systemctl restart rpp-pipeline` | No (~2s) |
| `server/**` | `sudo systemctl restart rover-server` | No (~2s) |
| `px4_start_service.sh`, pluginlist, NTRIP | `sudo systemctl restart px4-dxp` | Yes (~11s) |
| `*.service` / new files | `./deploy.sh` (daemon-reload + symlinks) | — |

`rpp-pipeline PartOf=px4-dxp` — px4-dxp restart cascades down; rpp-pipeline restart does not cascade up.
Re-run `deploy.sh` only when adding NEW files or changing `.service` definitions.

## FastAPI server (`server/`, port 5001)

```bash
cd ~/PX4_DXP/server && bash run.sh     # production (token auth)
ROVER_DISABLE_AUTH=1 bash run.sh        # dev mode
curl http://localhost:5001/api/ping     # health check
```

Discovery: UDP broadcast port 5002. Full API spec: `server/ARCHITECTURE.md`.

**Critical impl rules:**
- **Stop-path, not empty Path** — RPP ignores empty Path (early return). E-stop publishes current position as single-point path.
- **Async service calls** — `arm_async()`, `set_mode_async()` use `call_async` + `add_done_callback`, never block FastAPI event loop.
- **ENU→NED (RPP input):** `yaw_NED = π/2 - yaw_ENU`, `pos_n = pose.y`, `pos_e = pose.x`
- **NED→ENU (twist_to_setpoint output):** `vel.x = v_e` (East), `vel.y = v_n` (North), `vel.z = -v_d` (Up)
- **MAVROS crash detection:** TRANSIENT_LOCAL keeps stale `connected=True`. Server overrides after 2s via `_state_recv_time`.

## OFFBOARD rules (PX4-specific, easy to forget)

1. Stream setpoints at ≥2 Hz **before** requesting OFFBOARD or PX4 rejects the mode switch.
2. Stream gap >0.5s once in OFFBOARD → PX4 exits to failsafe.
3. Use `OFFBOARD` not `GUIDED` — GUIDED is ArduPilot naming.
4. Velocity: `/mavros/setpoint_velocity/cmd_vel` (TwistStamped)
5. Path/arc: `/mavros/setpoint_raw/local` (PositionTarget)

**Current status:** 2m×2m square validated (log 59, 2026-05-23). Max xtrack 9.4cm corners, 1-3cm straights.
**Next:** RPP corner tuning, safety params on FCU, RTK P4 validation.

## Cross-machine workflow

| Task | Where |
|---|---|
| Firmware patches, QGC params, waypoint gen, log analysis | Laptop only |
| ROS2 nodes, launch files, OFFBOARD logic | **Jetson primary**, laptop reviews via git |
| Running tests, rosbags, restarting services | Jetson only |

Bridge = git. One writer at a time. Memory is per-machine — cross-reference laptop memory by filename only.

## Hard rules

- **Do not edit PX4 firmware on Jetson.** Patches live on laptop only.
- **Do not stop `px4-dxp.service` without warning** — carries the QGC bridge.
- **Do not disable RTK** (`ntrip_rtcm_node.py`) — survey accuracy depends on it.
- **Do not push FCU params from Jetson** — QGC on laptop is source of truth.
- **ArduRover is abandoned.** Do not propose ArduRover solutions.
- Unsure about firmware/params/build? Ask user to check with Laptop Claude.

## Quick reference

```bash
ros2 topic echo /mavros/state --once           # bridge alive?
ros2 topic echo /mavros/local_position/pose    # live position
journalctl -u px4-dxp.service -f              # service log
ros2 bag record /mavros/local_position/pose /mavros/setpoint_raw/local /mavros/state -o ~/bags/$(date +%Y%m%d_%H%M%S)
```

- NTRIP credentials: `~/PX4_DXP/config/ntrip.env` (gitignored)
- MAVROS pluginlist: `~/PX4_DXP/px4_pluginlists_rover.yaml`
- QGC UDP port: 14550
