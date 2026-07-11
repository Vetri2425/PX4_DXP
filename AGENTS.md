# 3WD Marking Rover — Jetson Companion

Scope: runtime, ROS2, MAVROS2, on-device debugging on Jetson Orin `192.168.1.102`.
Not your job: PX4 firmware, waypoint gen, log analysis — those belong to Laptop Codex at `D:\Vetri\3WD_GCS\`.

## Hardware

| Item | Value |
|---|---|
| IP | `192.168.1.102` (eno1), user `flash` |
| OS | Ubuntu aarch64, ROS2 Humble |
| FCU | `/dev/ttyACM0` @ 921600 (CubeOrangePlus, PX4 v1.16.2) |
| RTK | UM982 on TELEM1 — NTRIP via MAVROS |

## Service restart (narrowest scope)

| Changed | Restart | Drops MAVROS? |
|---|---|---|
| `src/*.py` | `sudo systemctl restart rpp-pipeline` | No (~2s) |
| `server/**` | `sudo systemctl restart rover-server` | No (~2s) |
| `px4_start_service.sh`, pluginlist, NTRIP | `sudo systemctl restart px4-dxp` | Yes (~11s) |
| `*.service` / new files | `./deploy.sh` (daemon-reload) | — |

`rpp-pipeline PartOf=px4-dxp` — px4-dxp restart cascades down; not up.

## Critical impl rules

- **E-stop:** publish current pos as single-point path — RPP ignores empty Path
- **Async only:** `arm_async()`, `set_mode_async()` use `call_async` + `add_done_callback`
- **ENU→NED (RPP input):** `yaw_NED = π/2 - yaw_ENU`, `pos_n = pose.y`, `pos_e = pose.x`
- **NED→ENU (twist output):** `vel.x = v_e`, `vel.y = v_n`, `vel.z = -v_d`
- **MAVROS crash detect:** TRANSIENT_LOCAL keeps stale `connected=True`; server overrides after 2s via `_state_recv_time`

## OFFBOARD rules

1. Stream setpoints ≥2 Hz **before** requesting OFFBOARD or PX4 rejects it
2. Gap >0.5s in OFFBOARD → PX4 exits to failsafe
3. Use `OFFBOARD` not `GUIDED`
4. Velocity: `/mavros/setpoint_velocity/cmd_vel` (TwistStamped)
5. Path/arc: `/mavros/setpoint_raw/local` (PositionTarget)

## Current status (2026-07-11) — TRUSTED BASELINE

- **Pin:** branch `test/colinear-fix` @ **`3b7841a`** (controller = **`cd44884`**). All further work starts here.
- Phase 2 OFFBOARD + FastAPI running; production tracking = **segment / stop-pivot**
- Corner ≤5cm MET (06-12/06-15). Collinear PRE/MARK/AFT momentum fix in tree (`cd44884`).
- Frozen corner knobs: `segment_slowdown_dist=0.50`, `segment_brake_velocity_cap_m_s=0.08`, PRE_CORNER floor `segment_min_corner_speed=0.08`
- Spray: PX4 AUX1 cmd 187 (not GPIO). robot_localization superseded by EKF2 wheel-encoder fusion.
- Detail: `CLAUDE.md` Current status + agent memory `baseline_colinear_fix_2026_07_11`

## Hard rules

- Do not edit PX4 firmware on Jetson
- Do not stop `px4-dxp.service` without warning — carries QGC bridge
- Do not disable RTK (`ntrip_rtcm_node.py`)
- Do not push FCU params from Jetson — QGC on laptop is source of truth
- ArduRover is abandoned — do not propose ArduRover solutions

## Quick reference

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose
journalctl -u px4-dxp.service -f
ros2 bag record /mavros/local_position/pose /mavros/setpoint_raw/local /mavros/state -o ~/bags/$(date +%Y%m%d_%H%M%S)
```

- NTRIP creds: `~/PX4_DXP/config/ntrip.env` (gitignored)
- MAVROS pluginlist: `~/PX4_DXP/px4_pluginlists_rover.yaml`
- FastAPI: port 5001 — `curl http://localhost:5001/api/ping`
- QGC UDP: 14550 | ROS_DOMAIN_ID: 0
