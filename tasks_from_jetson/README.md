# PX4_DXP Task List — Phase 1 Stabilisation

Audit: 2026-05-19. Last reconciled: 2026-05-20 after laptop Claude push (ef54460→55a7482).

---

## Status Overview

| # | Issue | Priority | Status |
|---|-------|----------|--------|
| [01](task_01_ntrip_watchdog.md) | NTRIP node has no watchdog | CRITICAL | ✅ DONE (pull) |
| [02](task_02_rtcm_frame_parsing.md) | RTCM frame boundaries not respected | CRITICAL | ✅ DONE (pull) |
| [03](task_03_credentials_plaintext.md) | NTRIP credentials hardcoded | HIGH | ✅ DONE (pull) |
| [04](task_04_pkill_wrong_pattern.md) | pkill pattern wrong | HIGH | ✅ DONE (pull) |
| [05](task_05_restrict_realtime.md) | `RestrictRealtime=yes` blocks MAVROS scheduler | HIGH | ❌ OPEN |
| [06](task_06_start_limit_burst.md) | Burst limit too tight for USB flap | MEDIUM | ❌ OPEN |
| [07](task_07_protect_home_misconfiguration.md) | ProtectHome defeated by ReadWritePaths | MEDIUM | ✅ DONE (pull) |
| [08](task_08_redundant_polling_loops.md) | Dual competing ros2 node list loops | MEDIUM | ❌ OPEN |
| [09](task_09_ntrip_shutdown_race.md) | Publisher use-after-free on shutdown | MEDIUM | ✅ DONE (pull) |
| [10](task_10_ntrip_boot_dns_backoff.md) | Fixed 5s reconnect floods journal on boot | MEDIUM | ✅ DONE (pull) |
| [11](task_11_ntrip_telemetry_logging.md) | No RTCM frame/byte counter | MEDIUM | ✅ PARTIAL (pull) |
| [12](task_12_set_e_cleanup_inconsistency.md) | set -euo pipefail vs blanket \|\| true | LOW | ❌ OPEN |
| [13](task_13_cpu_quota.md) | CPUQuota=200% too low for burst | LOW | ✅ DONE (pull → 400%) |
| [14](task_14_mavros_plugin_denylist.md) | Unused MAVROS plugins active | LOW | ✅ PARTIAL (pull) |
| [15](task_15_ntrip_move_into_repo.md) | ntrip_rtcm_node.py outside repo | LOW | ✅ DONE (pull) |

**New issues found in the pulled code — added below:**

| # | Issue | Priority | Status |
|---|-------|----------|--------|
| [16](task_16_bindsto_burst_conflict.md) | `BindsTo=dev-ttyACM0.device` + burst limit race | HIGH | ❌ OPEN |
| [17](task_17_ntrip_log_split.md) | NTRIP logs split between /tmp/ntrip.log and journald | LOW | ❌ OPEN |
| [18](task_18_deploy_root_owned_env.md) | deploy.sh creates NTRIP env file as root — flash can't update | LOW | ❌ OPEN |
| [19](task_19_ntrip_mountpt_default.md) | NTRIP_MOUNTPT hardcoded default left in code | LOW | ❌ OPEN |

---

## What the Pull Fixed (summary)

**Tasks 01, 02, 03, 04, 07, 09, 10, 13, 15 — fully resolved:**

- `ntrip_watchdog()` function added — mirrors mavros_watchdog pattern exactly
- RTCM3 parser with full CRC-24Q validation, frame scanning, reserved bits handling
- Credentials moved to `~/.config/ntrip/env` via `EnvironmentFile=` in service
- `pkill` patterns fixed to `"mavros.*node.launch"` and `"ntrip_rtcm_node"`
- `ProtectHome=read-only` now paired with specific subdirs (`.ros`, `PX4_DXP`) not all of `~`
- `_stop_event` threading.Event added; `destroy_node()` joins thread cleanly
- Exponential backoff on reconnect (5s → 60s cap) with interruptible `stop_event.wait()`
- `CPUQuota` raised to 400%
- `ntrip_rtcm_node.py` now in repo, referenced via `$SCRIPT_DIR`

**Tasks 11, 14 — partial:**

- Task 11: 30s health monitor added (staleness detection) but no per-interval frame/byte counter
- Task 14: Added `hil`, `actuator_control`, `3dr_radio`, `ftp` to denylist. Still missing: `fake_gps`, `landing_target`, `mocap_pose_estimate`, `vision_pose_estimate`, `setpoint_accel`, `safety_area`

---

## Remaining Open Tasks (do in this order)

**Round 1 — Before next field test:**
1. **Task 16** — `BindsTo` + burst limit (new, high — can lock out service on USB flap)
2. **Task 05** — Remove `RestrictRealtime=yes` (1 line, service restart)
3. **Task 06** — Raise `StartLimitBurst` to 10 + `StartLimitInterval` to 300

**Round 2 — Housekeeping:**
4. Task 14 (partial) — Add remaining 6 plugins to denylist
5. Task 11 (partial) — Add frame/byte counter to telemetry log
6. Task 08 — Remove redundant outer polling loop from main script
7. Task 18 — Fix deploy.sh to create env file as flash:flash not root

**Round 3 — Code quality:**
8. Task 12 — set -e / || true cleanup
9. Task 17 — Consolidate NTRIP logging to journald only
10. Task 19 — Move NTRIP_MOUNTPT to required env var
