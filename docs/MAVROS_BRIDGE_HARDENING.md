# MAVROS Bridge — Production Hardening Design

**Status:** Phase 1 implemented in-tree; Phases 2–4 remain proposed.
**Scope:** The ROS2/DDS communication layer between MAVROS, `rover-server`, and the RPP pipeline on the Jetson.
**Author:** generated 2026-06-06, after the DDS shared-memory incident.

---

## 1. Why this matters

The MAVROS bridge is the single most critical runtime layer. Every safety-relevant
signal and command rides it:

- **Telemetry in:** `/mavros/state` (FCU link, mode, armed), `/mavros/local_position/pose`, battery, GPS.
- **Control out:** `/mavros/cmd/arming`, `/mavros/set_mode`, `/mavros/setpoint_raw/local`, `/path`.
- **RPP link:** `/rpp_controller/*` param services, `/rpp/debug`, `/rpp/velocity_ned`.

If this layer silently degrades, the operator sees a connected UI but the rover is
flying blind. **A frozen-but-"alive" bridge is the worst failure class** — worse than
a clean crash, because nothing alarms.

---

## 2. Incident summary (root cause)

**Observed:** After ungraceful process kills during a deploy recovery, `mavros_node`
kept running but `/mavros/state` had `Publisher count: 0`; `rover-server` reported
`fcu_connected: False`; telemetry frozen. Individual service restarts did not fix it.
A reboot did.

**Root cause:** ROS2 Humble's default RMW is **eProsima Fast DDS** (`rmw_fastrtps_cpp`).
On a single host it uses a **shared-memory transport** with lock files under
`/dev/shm/fastrtps_*`. These are released only on **graceful** shutdown. A `SIGKILL`
(or a crash, or systemd hitting `TimeoutStopSec` → SIGKILL under `KillMode=mixed`)
leaves **stale locked segments**. New/restarting participants then fail with
`open_and_lock_file failed` and cannot (re)establish the topic — hence `Publisher count: 0`.

**Contributing factors:**
- Nothing cleans stale `/dev/shm/fastrtps_*` on startup.
- `pkill -f "mavros|rpp|..."` self-matches the operator's own shell command line and
  kills the SSH session mid-recovery (observed: exit 255). Manual recovery is itself fragile.
- No automated detection/recovery of a frozen bridge.

---

## 3. Current state (measured)

| Aspect | Value |
|---|---|
| RMW | `rmw_fastrtps_cpp` (default; `RMW_IMPLEMENTATION` unset). CycloneDDS not installed. |
| Transport | Fast DDS default = shared memory (`/dev/shm/fastrtps_*`) + UDP. |
| `px4-dxp.service` | `Restart=always`, `KillMode=mixed`, `TimeoutStopSec=30`, `After=…dev-ttyACM0.device` |
| `rpp-pipeline.service` | `PartOf=px4-dxp`, `After/Wants=px4-dxp`, `Restart=on-failure`, `TimeoutStopSec=15` |
| `rover-server.service` | `After/Wants=rpp-pipeline`, `Restart=on-failure`, `TimeoutStopSec=15` |
| Drop-ins (Jetson-only) | `10-pythonpath`, `20-host` (`FASTAPI_HOST=0.0.0.0`), `30-cors` (`ROVER_CORS_ORIGINS=*`) |

systemd ordering/restart is already reasonable. The **only** fragile element is the
Fast DDS shared-memory lock-file mechanism.

---

## 4. Failure-mode analysis

| # | Failure | Trigger | Current handling | Gap |
|---|---|---|---|---|
| F1 | Stale `/dev/shm/fastrtps_*` jams the bus | Ungraceful death (crash / SIGKILL / stop-timeout) | none | **critical** |
| F2 | systemd SIGKILLs on stop | stop exceeds `TimeoutStopSec` | `KillMode=mixed` (causes F1) | high |
| F3 | `rpp-pipeline` doesn't auto-start after px4-dxp `start` | `PartOf` propagates stop/restart, not start | none | medium |
| F4 | `rover-server` holds a stale MAVROS link after MAVROS restart | TRANSIENT_LOCAL latch | `_state_recv_time` override (2s) | low (mostly handled) |
| F5 | Frozen bridge undetected | F1/serial loss | none (manual only) | high |
| F6 | Operator recovery kills own SSH session | broad `pkill -f` self-match | none | medium |

---

## 5. Proposed hardening (phased)

### Phase 1 — Eliminate the shared-memory failure mode  ⭐ root fix, low risk

**1a. Disable Fast DDS shared-memory transport (use localhost UDP).**
Provide a Fast DDS XML profile that forces UDPv4-only, and point every ROS process at it:

```xml
<!-- config/fastdds_no_shm.xml -->
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_only</transport_id>
        <type>UDPv4</type>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="udp_only_participant" is_default_profile="true">
      <rtps>
        <userTransports><transport_id>udp_only</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
```

Set in each service env (drop-ins / launch scripts):
```
FASTRTPS_DEFAULT_PROFILES_FILE=/home/flash/PX4_DXP/config/fastdds_no_shm.xml
```

- **Effect:** no `/dev/shm/fastrtps_*` lock files ever created → F1 becomes *impossible*,
  even on a mid-run crash. Loopback UDP is well within budget for our message rates/sizes.
- **Risk:** low. Reversible by unsetting the env var.
- **Trade-off:** marginally higher localhost latency vs SHM — negligible at 10–50 Hz, small payloads.

**1b. Stale-shm sweep on startup (belt-and-suspenders).**
`ExecStartPre` on `px4-dxp` (the first node in the graph):
```
ExecStartPre=/bin/sh -c 'rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true'
```
Harmless if 1a is active; covers any non-profiled stray process.

### Phase 2 — Guarantee graceful shutdown + coherent startup

- **F2:** confirm `ExecStop`/signal handlers tear down all children before `TimeoutStopSec`.
  `rpp_start.sh` already traps TERM/INT and force-kills stragglers after a grace period; mirror
  that rigor in `px4_start_service.sh`. Raise `TimeoutStopSec` only if a clean stop genuinely needs more.
- **F3:** make a px4-dxp recovery bring the whole stack back. Either add `BindsTo=`+`After=` so
  `rpp-pipeline`/`rover-server` follow px4-dxp start, or an `ExecStartPost` that starts them.
  (Keep `rover-server` after a real MAVROS-ready gate.)

### Phase 3 — Self-healing bridge watchdog

Extend `rover-server`'s existing pose-stale watchdog to monitor **MAVROS link liveness**:
- Track `/mavros/state` receipt freshness (already have `_state_recv_time`) + publisher presence.
- On sustained loss (> N s) → **graceful** `systemctl restart px4-dxp` (never `kill -9`),
  with **crash-loop backoff** (max K restarts per window, then stop and alarm) and an
  `activity_log` ERROR + Socket.IO alert so the operator/UI sees it.
- Expose `GET /api/health/bridge`: `{fcu_connected, state_age_ms, pose_age_ms, mavros_pub_present, last_recovery_ts, recovery_count}`.

### Phase 4 (optional) — Standardize RMW on CycloneDDS

Many production ROS2 robots run **CycloneDDS** for transport robustness/simplicity.
- `sudo apt install ros-humble-rmw-cyclonedds-cpp`; set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  in every service env; provide a CycloneDDS config (e.g. loopback/iface pinning).
- **Requires full revalidation** with MAVROS + all nodes (discovery, services, QoS).
- Only pursue if Phases 1–3 prove insufficient. Phase 1 already removes the observed failure.

---

## 6. Recommendation

Do **Phase 1** now — it eliminates the exact failure with minimal risk and is reversible.
Add **Phases 2–3** to reach genuine self-healing field-grade. Defer **Phase 4** unless needed.

Also adopt the operator-safety rule regardless of phase: **never `kill -9` ROS nodes; use
`systemctl stop`.** Avoid broad `pkill -f` patterns (they self-match the SSH command).
A `tools/recover_bridge.sh` helper (graceful stop → shm sweep → ordered start → verify)
would make manual recovery safe and one-command.

---

## 7. Validation plan (per phase, on the Jetson)

1. **Baseline:** `ros2 topic info /mavros/state` → `Publisher count: 1`; `curl /api/mission/status` → `fcu_connected:true`, fresh `pose_age_ms`.
2. **Phase 1 proof:** after applying, confirm `ls /dev/shm | grep fastrtps` stays empty during normal run; then deliberately `kill -9 mavros_node` and confirm the watchdog/systemd restart **recovers cleanly with no manual shm sweep** (this is the regression test for F1).
3. **No-flap check:** count `"starting MAVROS"` in `journalctl -u px4-dxp` over 2 min → exactly 1.
4. **Phase 3 proof:** simulate a frozen link (block FCU heartbeat); confirm auto graceful recovery + backoff + alert; confirm it does NOT flap when the FCU is genuinely absent.
5. **Full stack:** all three services `active`, 0 tracebacks, `/api/ping` 200, mobile/QGC reconnect.

## 8. Rollback

- Phase 1: unset `FASTRTPS_DEFAULT_PROFILES_FILE` (revert to SHM) + remove `ExecStartPre`; `daemon-reload` + restart.
- Phases 2–4: revert the respective commits / drop-ins; reboot restores the systemd baseline.

---

## 9. Open questions for the operator

1. Message-rate ceiling expected at peak (to confirm UDP-loopback headroom)? Current peak is the 50 Hz setpoint stream — comfortably fine.
2. Acceptable auto-recovery aggressiveness (max restarts/window before it stops and alarms)?
3. Is a brief MAVROS drop (~11 s, px4-dxp restart) ever unacceptable mid-mission, or should recovery wait for an idle/safe state first?
