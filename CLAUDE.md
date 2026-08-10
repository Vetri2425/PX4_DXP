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

| Changed | Restart | Drops MAVROS? | Drops RTK? |
|---|---|---|---|
| `src/*.py` | `sudo systemctl restart rpp-pipeline` | No (~2s) | No |
| `server/**` | `sudo systemctl restart rover-server` | No (~2s) | **YES — see below** |
| `px4_start_service.sh`, pluginlist, NTRIP | `sudo systemctl restart px4-dxp` | Yes (~11s) | Yes |
| `*.service` / new files | `./deploy.sh` (daemon-reload) | — | — |

`rpp-pipeline PartOf=px4-dxp` — px4-dxp restart cascades down; not up.

> ⚠ **Restarting `rover-server` KILLS the NTRIP stream.** The RTK client is a
> *child process* of that service (`server/rtk_manager.py` spawns
> `ntrip_rtcm_node.py`), so any `server/**` deploy silently drops the rover to
> FLOAT with **no warning in the app**. Restart it immediately after:
> `POST /api/rtk/ntrip/start` (caster `caster.emlid.com:2101`, mountpoint + creds
> in `config/ntrip.env`). Session tokens are in-memory too — log in *after* the
> restart or the call returns "Invalid or missing rover session".
> `ntrip.service` is a **dead end**: it needs RTKLIB `str2str` and
> `~/ntrip_stream.sh`, neither of which exists on this Jetson.

> ⚠ **Check `armed` + mission state before restarting `rpp-pipeline`.** On
> 2026-08-01 a restart was issued while the rover was mid-run in OFFBOARD at
> 0.70 m/s; it aborted the run. Dropping setpoints in OFFBOARD is exactly what
> trips the failsafe, and `NAV_RCL_ACT`/`NAV_DLL_ACT` are both **Disarm**.

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
- **Frozen RPP corner-stop defaults (this tree):** `segment_slowdown_dist=0.50`, `segment_brake_velocity_cap_m_s=0.08`, `segment_min_corner_speed=0.08` (PRE_CORNER floor), `segment_endpoint_approach_speed=0.03` (final-segment only), `segment_heading_tolerance_deg=2.0`, `segment_stop_yaw_rate_threshold=0.05`, `segment_align_settle_s=0.20`, `segment_stop_dwell_s=0.30`. Also: `max_yaw_rate_body=0.45`, `a_lat_max=0.3`, `corner_smooth_radius_m=0.5`. **FCU params — the table below was re-verified against the LIVE FCU on 2026-08-01** (`python3 tools/quick_params.py`, ~1.5 s for 40 params; MAVROS2 exposes every FCU param as a ROS param on `/mavros/param`). Every row matched except the two `EKF2_WENC_*` rows, which were wrong — **read the FCU, never this table from memory.** Latest QGC export `params/31_07_2026_6_08pm.params` agrees with the live values; `PX4_params/22-07-2026/` and `PX4_DXP_Tracker.xlsx` → "PX4 FCU Params" are both stale.

| Group | Param | Value | Note |
|---|---|---|---|
| Yaw | `RO_YAW_P` | 1.5 | pure-P; I=D=0 in firmware — structural following-error floor |
| | `RO_YAW_RATE_P` | 0.13 | was 0.17 |
| | `RO_YAW_RATE_LIM` | **22** | was 90; **CLAUDE.md previously said 30 — wrong** |
| | `RO_YAW_ACCEL_LIM` / `RO_YAW_DECEL_LIM` | 15 / 18 | was 25 / 34 |
| Speed | `RO_MAX_THR_SPEED` | **1.28** | **CALIBRATION, not a limit** — full-throttle speed. `throttle ≈ v_des / this`, so understating it makes every command too large. Was 0.96 ⇒ **+33 % overspeed** (0.35 cmd → 0.467 measured); set to 1.28 in QGC 2026-08-01 and the error closed to **±6 %**. You RAISE it to go slower. `RO_SPEED_LIM` is NOT this knob — it caps the *setpoint* and never binds while `mission_speed` is below it |
| Heading | `GPS_YAW_OFFSET` | **180.0** | **the ACTIVE knob** (audit 2026-08-03): driver subtracts it, exactly cancelling the hardcoded +180° master-front flip in `nmea.cpp` → net raw UM982 baseline heading (antennas mounted REVERSED). Round number — assumed, not measured (bug B1) |
| | `EKF2_GPS_YAW_OFF` | **180.0** | **DEAD CODE on this setup** — EKF2 only applies it when the driver's `heading_offset` is NaN, and the driver always publishes a finite one. Changing it does nothing; tune `GPS_YAW_OFFSET` instead |
| Antenna | `EKF2_GPS_POS_X/Y/Z` | 0 / 0 / −0.4 | **Y=0 asserts the antenna is on the centreline** — verify physically (bug B3) |
| Wheels | `RBCLW_COUNTS_REV` | 148000 | one value for BOTH wheels (bug B2) |
| | `RBCLW_QPPS_MAX` | 182655 | |
| | `RD_WHEEL_TRACK` | 0.470 | |
| | `EKF2_WENC_RAD` | 0.1524 | 6 in |
| Encoder fusion | `EKF2_WENC_CTRL` | 1 | enabled |
| | `EKF2_WENC_NOISE` / `_LAT_N` | **0.35 / 0.35** | **previously documented as 0.1/0.1 — wrong.** The "3.5× more trust in the encoder" claim built on that number is RETRACTED: the retune is not on the vehicle |
| | `EKF2_WENC_GATE` | **5.0** | **previously documented as 3 — wrong.** 5.0 is the PX4 default, so the gate is **NOT** tightened here |
| Cornering | `RD_TRANS_DRV_TRN` / `_TRN_DRV` | 0.70 / 0.0349 rad | drive→turn 40°, turn→drive 2° |
| | `NAV_ACC_RAD` | 0.05 | |
| Failsafe | `NAV_RCL_ACT` / `NAV_DLL_ACT` | 6 / 6 | **Disarm** on RC / datalink loss |
| | `COM_RC_IN_MODE` | 2 | |

`PP_LOOKAHD_*` (0.7 / 0.6 / 2.0) exist but are **unused** — they drive PX4's own AUTO-mission pure pursuit, which this rover does not use (companion RPP over OFFBOARD velocity).
- **FUTURE — SPD-T1 (backlog):** 1.0 m/s line / 0.6 m/s arc. Prereq closed 2026-08-01: measured full-throttle speed is **~1.28 m/s** (`RO_MAX_THR_SPEED` now 1.28), so 1.0 m/s is within reach of the drivetrain.
- Tracking profiles live: `tracking_profile=auto|segment|smooth`.
- Phase 3 spray: **live on this tree** — `spray_controller_node.py` → PX4 AUX1 via cmd 187; `on_value=1.0` / `off_value=-1.0` (normalized). QGC: `PWM_AUX_FUNC1=301`, `PWM_AUX_MIN1=0`, `PWM_AUX_MAX1=15000` (verified 2026-07-22; raised from 3000 for flow — see spray PWM strength note), `PWM_AUX_DIS1=0`. Manual: `POST /api/spray/test`.
- Plan doc on branch: `docs/OFFBOARD_POSITION_MODE_PLAN.md` (future position-mode stop architecture — not implemented in controller yet).
- robot_localization fusion: not pursued (EKF2 wheel-encoder fusion supersedes).
- **GNSS/manual-mode/encoder firmware audit 2026-08-03:** full findings + patch backlog in `docs/FIRMWARE_PENDING_PATCHES.md`. Highlights: heading dropout **stops `/mavros/global_position/*` entirely** (always-landed patch clears yaw-align — §C1); `gps2/raw`, `gps_rtk/rtk_baseline`, `gp_origin` **never publish on this setup — normal, not a fault** (`gp_origin` needs MAV_CMD_REQUEST_MESSAGE 49); NTRIP has **no autostart** after a rover-server restart; manual-mode steering has **no smoothing and no param can add it** (`RO_YAW_EXPO`-family consumed only by dead code). ⚠ That audit's "deployed tree = `~/px4-rover-build`" instruction is **WRONG and superseded** — see *Firmware provenance* below.

### Firmware provenance (corrected 2026-08-09 — read before any firmware claim)

| what | where |
|---|---|
| **Sources** | `/Users/dyx_a1/Vetri/PX4-Autopilot`, branch `main` — real fork history |
| **Flashed artifact** | `PX4_Firmware/wenc-final_1d82e616/cubeorangeplus_rover_firmware/cubepilot_cubeorangeplus_rover.px4` — fork `1d82e616f8`, built 2026-08-08 12:20, base v1.16.2 (`54f0455f`). Flashed 2026-08-08 afternoon; live for `first_run`–`fourth_run` (14:13–19:27) |
| **Build archive** | `PX4-Autopilot/PX4_Firmware/` — **only `gnss-yaw_015c6748` (08-09, F2 v2, not yet flashed) is present locally as of 2026-08-09 evening.** The wenc-era build dirs (`wenc-logging_06309e41`, `wenc-leverarm_255453d9`, `wenc-leverarm+logfix_9641ea9a`, `wenc-final_1d82e616` — the one actually live on the vehicle) are **no longer on disk**, deleted or replaced at some point after `wenc-final_1d82e616` was flashed; still live on GitHub Actions as CI artifacts if needed (`gh run list --repo Vetri2425/PX4-Autopilot --workflow=build_rover.yml`), just not downloaded locally |
| **What is in the binary** | stock **PX4 v1.16.2** + the explicit `cp` list in `.github/workflows/build_rover.yml` |
| ⛔ **Never audit** | `~/px4-rover-build` — detached Apr-2026 HEAD, dirty worktree, contains **none** of the fork commits; its `git log`/`blame` describe only the base and lie about every overlaid file |

**The rule is two-sided.** A file **on** the `cp` list (EKF2 + `wheel_encoder_fusion.cpp`, `DifferentialVelControl/`, `RoverDifferential.*`, RoverLandDetector, roboclaw, `logged_topics.cpp`, …) → read the **fork**. A file **not** on the list (`rover_differential/CMakeLists.txt`, `output_predictor.*`, gps/mavlink/sensors) → the binary has **stock v1.16.2**, so reading the fork gives a post-v1.18 file that never shipped. Open `build_rover.yml` **first** and decide which side the file is on. This trap already produced a confident-but-wrong "`DifferentialVelControl` isn't compiled" — the base CMakeLists **does** build it, and it is flying.

**`ver_sw` can never identify a fork build** — CI copies fork files onto a v1.16.2 checkout without committing, so the `.px4` always records the base hash `54f0455f`; every past build (including `fw_heading_stability_20260805`) records the *identical* hash. A flash is undetectable from `ver_sw` or FCU params; the only discriminators are the `.px4`'s own `image_size` / `build_time`, and the artifact directory name — preserve that naming. The `EKF2_GPS_YAW_N` build (a deliberate attempt to cut the swing) **backfired — tightened gate ⇒ rejected pivot innovations ⇒ 8–9 s yaw blackouts — and was reverted 2026-08-05** to `wenc-logging_06309e41`.

**2026-08-07→08-08 batch, closes backlog A4+A5 (`docs/FIRMWARE_PENDING_PATCHES.md`):**
- `255453d967` — EKF2 WENC IMU lever-arm correction (**A5**): the body-frame velocity measurement is now corrected for `ω × imu_pos_body` before fusion, matching the pattern `updateGnssVel()`/EV already use. Root cause of the pivot-position-walks-in-a-circle bug (radius ≈ `EKF2_IMU_POS_X`).
- `9641ea9a99` + `53940ead83` — logger fix so `wheel_encoders`/`estimator_aid_src_wheel_encoder` actually log from a cold boot (prerequisite for field-verifying the above) + pin the aid-source instance count (was reserving 7 dead subscription slots).
- `1d82e616f8` — RoboClaw driver now timestamps the encoder read *before* the UART transactions instead of after (**A4**), closing a window of up to ~264 ms of unbounded jitter in the timestamp fed to EKF2's WENC fusion buffer.
- **Field-verified 2026-08-09** via pivot-window drift-radius analysis (`estimator_local_position` during near-stationary high-yaw-rate windows) across all 34 ulogs in `PX4_Logs/WENC_FIX_PATCHES_AUG_08/{first,second,third,fourth}_run/`: median pivot wobble radius **1.52 cm → 0.50 cm**, median net walk **2.02 cm → 0.83 cm** vs the 2026-08-05 pre-flash baseline (`PX4_Logs/pre_run/`, `PX4_Logs/post_run/`), n=44 pre / n=105 post pivot windows, same methodology both sides.

**2026-08-09, F2 v2 — code landed + build + replay verified, NOT YET FLASHED (`docs/FIRMWARE_PENDING_PATCHES.md` §E1):**
- `015c67484e` + comment-correction `a6e9e12e2a` on `main` — adds `EKF2_GPS_YAW_N` (binds the previously-hardcoded `gnss_heading_noise`) and a new `EKF2_GPS_YAW_G` (absolute innovation floor, rad — force-accepts a gate-rejected update if its innovation is still under the floor; one-way, can only rescue, never discard). Both default to stock behaviour. Fixes the design flaw that killed F2 v1 (`45f576bd`, reverted 08-05): the accept/reject gate is `EKF2_HDG_GATE·√(P+R)`, so lowering `R` (`EKF2_GPS_YAW_N`) for accuracy also narrowed the outlier window, which rejected 17.5%/24.3% of heading updates during pivots and force-reset yaw on the 7 s aiding timeout — the field wobble.
- `gnss_yaw_control.cpp` and a new `params_gnss_yaw.yaml` had to be re-anchored onto stock v1.16.2 and newly added to `build_rover.yml`'s overlay list — **neither was ever on it before** (v1 didn't need to touch `gnss_yaw_control.cpp`; its `params_gnss.yaml` overlay commit lives on an abandoned branch that was never merged into `main`). `params_gnss.yaml` itself was deliberately left untouched (missing `EKF2_GPS_DELAY`/`EKF2_GPS_POS_*` on current `main` relative to v1.16.2 — the new params got their own yaml instead, following the `params_wheel_encoder.yaml` precedent).
- **CI build `31306374824`: success.** Local artifact `PX4_Firmware/gnss-yaw_015c6748/cubeorangeplus_rover_firmware/cubepilot_cubeorangeplus_rover.px4`.
- **Verified via replay, artifacts at `/Users/dyx_a1/Vetri/f2v2_replay/`:** neutrality — `estimator_aid_src_gnss_yaw` at default params is decision-identical (0 diff on `fused`/`innovation_rejected`) across 6 logs (4× 08-08 + both 08-05 failure recordings); continuous-value jitter ≤0.17° is reproducible replay-start noise, never flips a decision. 120-cell sweep (`sweep.csv`, 5×`EKF2_GPS_YAW_N` × 4×`EKF2_GPS_YAW_G` × 6 logs) confirms the fix and finds **one non-monotonic dead cell**: `YAW_N=0.05, YAW_G=0.175` fails on the hardest log (`log312`: 2.49% rejected, 5.04 s longest run, 1 `yaw_align` drop) while both `YAW_N=0.1` (gate wide enough unaided) and `YAW_N≤0.03` (floor rescues cleanly) pass — that log's peak rejected innovation at `YAW_N=0.05` sits at 13.07°, just above the 10° floor; at `YAW_N≤0.03` tighter tracking keeps innovations smaller so the same floor catches nearly everything.
- **Field-protocol correction: skip `EKF2_GPS_YAW_N=0.05` entirely, go straight to `YAW_N=0.013, YAW_G=0.175`** (0.03/0.02/0.013 are all equally clean in the sweep; 0.013 is the value where the UM982's own reported accuracy finally wins the `fmaxf` floor).
- **Not flashed yet.** Remaining items are non-code: tape-measure `EKF2_IMU_POS_X`, test outdoors with a real 3D fix (replay is open-loop, can't validate live GNSS), don't change `EKF2_WENC_CTRL` in the same A/B window.

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
