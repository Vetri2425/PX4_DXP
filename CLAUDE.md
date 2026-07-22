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

## Current status (2026-07-11) — TRUSTED BASELINE

> **Pin:** branch `test/colinear-fix` @ **`3b7841a`** (`coliner test fix`). Controller code = **`cd44884`** (collinear spray-boundary momentum). All further companion work starts from this tree. July mainline stop/pivot param doublings, `corner_stop_hold_s`, completion latch, and twist reverse-yaw-hold are **not** in this branch — do not reintroduce without a named A/B.

- Phase 2 OFFBOARD stack running; FastAPI + mobile frontend built
- **Controller + tuning CLOSED** at this baseline. Production tracking = **segment / stop-pivot**. Do not re-open arc PID/lookahead unless a regression appears.
- **Lineage still included:** BUG-T3 `510be9b` / BUG-T2 `1af51ac` / BUG-T1 `036f116` (validated 2026-06-15) + collinear momentum `cd44884` (2026-06-19).
- **Tracking @0.35 m/s — shapes sub-2cm RMS (06-15 bags):** arc 1.46 / lshape 0.90 / square 0.87 / U-turn 1.06 cm.
- **Arc (smooth RPP) — structural floor, DEFERRED:** velocity OFFBOARD discards `trajectory_setpoint.yawspeed`; pure-P → following err `≈ ω/RO_YAW_P`. `RO_YAW_P=1.5`. Companion `yaw_rate_feedback_gain` is a NO-OP in velocity mode.
- **Frozen RPP corner-stop defaults (this tree):** `segment_slowdown_dist=0.50`, `segment_brake_velocity_cap_m_s=0.08`, `segment_min_corner_speed=0.08` (PRE_CORNER floor), `segment_endpoint_approach_speed=0.03` (final-segment only), `segment_heading_tolerance_deg=2.0`, `segment_stop_yaw_rate_threshold=0.05`, `segment_align_settle_s=0.20`, `segment_stop_dwell_s=0.30`. Also: `max_yaw_rate_body=0.45`, `a_lat_max=0.3`, `corner_smooth_radius_m=0.5`. **FCU params — verified from the QGC export `PX4_params/22-07-2026/` (880 params), NOT from memory.** Full set in that file; `PX4_DXP_Tracker.xlsx` → "PX4 FCU Params" is stale.

| Group | Param | Value | Note |
|---|---|---|---|
| Yaw | `RO_YAW_P` | 1.5 | pure-P; I=D=0 in firmware — structural following-error floor |
| | `RO_YAW_RATE_P` | 0.13 | was 0.17 |
| | `RO_YAW_RATE_LIM` | **22** | was 90; **CLAUDE.md previously said 30 — wrong** |
| | `RO_YAW_ACCEL_LIM` / `RO_YAW_DECEL_LIM` | 15 / 18 | was 25 / 34 |
| Speed | `RO_MAX_THR_SPEED` | **0.96** | **previously documented as 0.9 — wrong** |
| Heading | `EKF2_GPS_YAW_OFF` | **180.0** | dual antenna mounted REVERSED. A round number — assumed, not measured. See open bug B1 |
| | `GPS_YAW_OFFSET` | **180.0** | driver-level twin of the above |
| Antenna | `EKF2_GPS_POS_X/Y/Z` | 0 / 0 / −0.4 | **Y=0 asserts the antenna is on the centreline** — verify physically (bug B3) |
| Wheels | `RBCLW_COUNTS_REV` | 148000 | one value for BOTH wheels (bug B2) |
| | `RBCLW_QPPS_MAX` | 182655 | |
| | `RD_WHEEL_TRACK` | 0.470 | |
| | `EKF2_WENC_RAD` | 0.1524 | 6 in |
| Encoder fusion | `EKF2_WENC_CTRL` | 1 | enabled |
| | `EKF2_WENC_NOISE` / `_LAT_N` | 0.1 / 0.1 | was 0.35 — **3.5× more trust in the encoder** |
| | `EKF2_WENC_GATE` | 3 | PX4 default is 5.0 SD — tighter here |
| Cornering | `RD_TRANS_DRV_TRN` / `_TRN_DRV` | 0.70 / 0.0349 rad | drive→turn 40°, turn→drive 2° |
| | `NAV_ACC_RAD` | 0.05 | |
| Failsafe | `NAV_RCL_ACT` / `NAV_DLL_ACT` | 6 / 6 | **Disarm** on RC / datalink loss |
| | `COM_RC_IN_MODE` | 2 | |

`PP_LOOKAHD_*` (0.7 / 0.6 / 2.0) exist but are **unused** — they drive PX4's own AUTO-mission pure pursuit, which this rover does not use (companion RPP over OFFBOARD velocity).
- **FUTURE — SPD-T1 (backlog):** 1.0 m/s line / 0.6 m/s arc. Prereq: verify RoboClaw top speed vs `RO_MAX_THR_SPEED=0.96`.
- Tracking profiles live: `tracking_profile=auto|segment|smooth`.
- Phase 3 spray: **live on this tree** — `spray_controller_node.py` → PX4 AUX1 via cmd 187; `on_value=1.0` / `off_value=-1.0` (normalized). QGC: `PWM_AUX_FUNC1=301`, `PWM_AUX_MIN1=0`, `PWM_AUX_MAX1=15000` (verified 2026-07-22; raised from 3000 for flow — see spray PWM strength note), `PWM_AUX_DIS1=0`. Manual: `POST /api/spray/test`.
- Plan doc on branch: `docs/OFFBOARD_POSITION_MODE_PLAN.md` (future position-mode stop architecture — not implemented in controller yet).
- robot_localization fusion: not pursued (EKF2 wheel-encoder fusion supersedes).

### Active focus (from this baseline)
1. **Path engine + trajectory planning** — mission/path generation, segment splitting, corner handling
2. **CRS / coordinate handling** — CRS + geodesic conversion for path import
3. **Spray control logic** — flag conditioning, timing, safety gates end-to-end
4. **Full-pipeline validation** — CAD/DXF → path → mission → drive → spray
5. Optional: evaluate OFFBOARD position-mode plan without regressing `cd44884` stop/pivot

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
