# 3WD Marking Rover — Jetson Companion

Scope: runtime, ROS2, MAVROS2, on-device debugging on Jetson Orin `192.168.1.102`.
Not your job: PX4 firmware, waypoint gen, log analysis — those live on Mac GCS at `/Users/dyx_a1/Vetri/3WD_GCS_transfer/3WD_GCS/`.

> **Cross-project memory:** `/Users/dyx_a1/Vetri/PX4-Autopilot/.claude/memory/integration.md`
> Load this when working on firmware↔companion integration, OFFBOARD interface, coordinate conventions, or open issues spanning both projects.

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

## GCS Machine (Mac)

| Item | Value |
|---|---|
| Host | MacBook Air, user `dyx_a1` |
| GCS path | `/Users/dyx_a1/Vetri/3WD_GCS_transfer/3WD_GCS/` |
| SSH to Jetson | `ssh flash@192.168.1.102` |
| QGC | QGroundControl on macOS |

## Current status (2026-06-06)

- Phase 2 OFFBOARD stack running; FastAPI + mobile frontend built
- Arc tuning at arc_fix_28; arc_fix_16 validated 1.5m arc at **2.57cm median xtrack**
- Active goal: corner xtrack **≤5cm** (plan: `docs/superpowers/plans/2026-06-02-corner-xtrack-reduction.md`)
- Validated RPP params: `max_yaw_rate_body=0.45`, `a_lat_max=0.3`, `corner_smooth_radius_m=0.5`
- Phase 3 (spray GPIO) and robot_localization fusion: not yet built

## Hard rules

- Do not edit PX4 firmware on Jetson
- Do not stop `px4-dxp.service` without warning — carries QGC bridge
- Do not disable RTK (`ntrip_rtcm_node.py`)
- Do not push FCU params from Jetson — QGC on Mac is source of truth
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

## Telemetry debugging

Use `tools/capture_telemetry.py` to inspect live WebSocket telemetry — prefer this over `curl /api/telemetry/latest` when you need multiple samples or want to watch values change.

```bash
# From Mac — single snapshot
ssh flash@192.168.1.102 'cd ~/PX4_DXP && python3 tools/capture_telemetry.py -n 1 --host localhost'

# From Mac — 5 samples (one per 100ms tick at 10 Hz)
ssh flash@192.168.1.102 'cd ~/PX4_DXP && python3 tools/capture_telemetry.py -n 5 --host localhost'

# From Mac — continuous stream until Ctrl-C
ssh flash@192.168.1.102 'cd ~/PX4_DXP && python3 tools/capture_telemetry.py -n 0 --host localhost'

# Filter a specific field (e.g. GPS accuracy)
ssh flash@192.168.1.102 'cd ~/PX4_DXP && python3 tools/capture_telemetry.py -n 5 --host localhost 2>/dev/null' \
  | python3 -c "import sys,json; [print(json.loads(l)['gps_fix_name'], json.loads(l)['hrms'], json.loads(l)['vrms']) for l in sys.stdin]"
```

Output is NDJSON (one JSON object per line). Fields: all `TelemetryData` fields + `_captured_at` (UTC ISO-8601). NaN → null.
