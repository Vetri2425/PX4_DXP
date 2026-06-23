# Forensic Architecture Audit: NTRIP RTK Injection Reliability (PX4_DXP)

**Task 03.1 — Read-Only Audit**
**Date: 2026-06-22**
**Ref: main branch (83c9bb7f)**

---

## Architecture Diagram (Live Data Path)

```
NTRIP Caster (Internet)
    │ TCP socket (:2101)
    ▼
ntrip_rtcm_node.py (ROS2 node, daemon thread)
    │ ┌─ RTCM3 preamble detection (0xD3)
    │ ├─ Frame length validation
    │ ├─ CRC-24Q verification
    │ └─ Publish to /mavros/gps_rtk/send_rtcm
    ▼
MAVROS gps_rtk plugin
    │ MAVLink GPS_RTCM_DATA message
    ▼
PX4 FCU (/dev/ttyACM0 @ 921600)
    │ Serial
    ▼
UM982 GPS (on TELEM1)
    │ RTCM corrections applied
    ▼
EKF2
    │ Fix type propagated
    ▼
mavros/gpsstatus/gps1/raw (GPSRAW.fix_type)
    │   → /mavros/global_position/raw/fix (NavSatFix)
    ▼
rpp_controller_node.py (P0.3 gate: require_rtk_fix=True)
    │ fix_type==6 → RTK_FIXED → drive allowed
    │ fix_type<6  → StateCode.RTK_WAIT → zero velocity published
    ▼
twist_to_setpoint_node → PX4 OFFBOARD → RoboClaw → Motors
```

---

## Files Responsible

| Layer | File | Function | Role |
|---|---|---|---|
| **NTRIP Client** | `ntrip_rtcm_node.py` (549 lines) | `NtripNode` class | Connects TCP, validates RTCM3, publishes to MAVROS |
| **RTCM Parser** | `ntrip_rtcm_node.py` | `_parse_rtcm_frames()` static method | Preamble scan, length extraction, CRC-24Q check |
| **Reconnection** | `ntrip_rtcm_node.py` | `_run()` main loop | Exponential backoff reconnect (5s→60s max) |
| **GGA Back-feed** | `ntrip_rtcm_node.py` | `_send_gga()`, `_format_gga()` | 10s timer sending GGA to caster |
| **Health** | `ntrip_rtcm_node.py` | `_check_health()`, `_write_status()` | 30s health timer, status file writer |
| **Process Lifecycle** | `server/rtk_manager.py` (281 lines) | `AsyncRTKManager` | async subprocess create/terminate/watch |
| **API Routes** | `server/routes/rtk.py` (116 lines) | `start_ntrip()`, `stop_all()`, `rtk_status()` | REST endpoints for RTK control |
| **Server Entry** | `server/main.py` (485 lines) | `lifespan()` | Creates `AsyncRTKManager`, stops on shutdown |
| **Offboard Controller** | `server/offboard_controller.py` (844 lines) | `start_async()` | Blocks mission start if RPP=RTK_WAIT |
| **RPP Controller** | `src/rpp_controller_node.py` (3581 lines) | `_control_loop()`, `_gps_cb()` | P0.3 RTK gate: zero velocity when fix_type<6 |
| **Safety Watchdog** | `server/main.py` | `_telemetry_loop()` | RPP unhealthy codes (incl. RTK_WAIT) trigger estop after grace |
| **MAVROS Bridge** | `server/ros_node.py` (1242 lines) | `get_state()` | `connected=False` override after 2s state timeout |
| **MAVROS Watchdog** | `px4_start_service.sh` (229 lines) | `mavros_watchdog()` | Restarts MAVROS if process dies |
| **Service: px4-dxp** | `px4-dxp.service` | — | systemd unit, `Restart=always`, `PartOf` hierarchy |
| **Service: rpp-pipeline** | `rpp-pipeline.service` | — | `PartOf=px4-dxp.service` |
| **Service: rover-server** | `rover-server.service` | — | Independent restart, no PartOf |
| **Deploy** | `deploy.sh` | — | Symlinks service files, daemon-reload |

---

## Process Lifecycle Ownership

### Who starts NTRIP?
The FastAPI server creates `AsyncRTKManager` in its lifespan. A user (or operator) calls `POST /api/rtk/ntrip/start` with host/port/mountpoint/credentials. The manager starts `ntrip_rtcm_node.py` as a subprocess with `asyncio.create_subprocess_exec`.

### Who stops NTRIP?
- User calls `POST /api/rtk/stop` (calls `stop_all()`)
- Server shutdown: `lifespan()` cleanup calls `await rtk_manager.stop_all()`
- The subprocess exits on its own on connection failure + reconnection exhaustion (but there IS no exhaustion — infinite loop)

### Subprocess creation
`asyncio.create_subprocess_exec()` with stdin PIPE for password. **No process group isolation** — the child inherits the server's process group. `kill()` is used on timeout.

### Child process monitoring
`_watch_process()` is an asyncio task that awaits `process.wait()`. On exit, it clears internal state. **No automatic restart** — the manager transitions to `idle` and stays there.

### systemd control
- NTRIP is NOT directly controlled by systemd (it's a FastAPI-managed subprocess)
- systemd controls `rover-server.service` which hosts the FastAPI app
- If `rover-server` restarts, NTRIP process is orphaned (no process group isolation) → `_clear_process_locked()` runs but the actual child process may survive
- If `px4-dxp` or `rpp-pipeline` restarts, the FastAPI server survives (no PartOf) → NTRIP child survives

### FastAPI control of NTRIP
Yes — exclusively. All start/stop/status goes through FastAPI routes.

### Rover reboot restores NTRIP session?
**No.** After reboot:
1. systemd starts `px4-dxp.service` (MAVROS)
2. `rpp-pipeline.service` starts (after MAVROS)
3. `rover-server.service` starts (after pipeline, no wait)
4. Server starts → `AsyncRTKManager` in `idle` state
5. **NTRIP does NOT auto-start** — operator must call `POST /api/rtk/ntrip/start`

### Persistent or temporary?
**Temporary.** NTRIP session does not survive server restart, process crash, or rover reboot.

---

## Connection Failure Behavior

| Failure | Current Behavior | Safe? |
|---|---|---|
| **Internet disconnects** | TCP socket timeout (30s) → reconnect loop with exponential backoff (5s→60s) | ⚠️ Partial (30s detection gap) |
| **WiFi drops** | Same as internet disconnect — socket timeout 30s | ⚠️ Same gap |
| **Ethernet cable disconnects** | TCP half-open: socket timeout detection only (30s) on next recv(). Phase 1: OS may take 10-60s to declare ESTABLISHED dead. | ❌ Long detection time |
| **Hotspot disappears** | TCP socket timeout 30s → reconnect | ⚠️ 30s detection gap |
| **NTRIP caster closes socket** | `recv()` returns empty bytes → immediate reconnect (no backoff advance) | ✅ Fast detection |
| **Caster auth fails** | `_connect()` raises `ConnectionError("Caster rejected: ...")` → reconnect loop | ✅ Fast |
| **Corrupted RTCM** | CRC-24Q mismatch → frame discarded, preamble scan continues at next byte | ✅ Fast |
| **Zero bytes from caster** | Socket stays open, 30s health check sees no data, logs warning, stays connected | ❌ No reconnect trigger |
| **DNS resolution fails** | `socket.connect()` raises `gaierror` → caught by outer `except Exception` → reconnect loop | ⚠️ Correct but retry wastes backoff |

### Zero-bytes scenario detail
The `_check_health()` timer runs every 30s and checks if data was received. It ONLY logs a warning — it does NOT trigger a reconnect. If the caster sends keepalive bytes without valid RTCM frames, the parser discards them but `_byte_count` still increments, so health check sees "data" and stays satisfied.

---

## Reconnection Architecture

- **Retry loop exists?** Yes — `_run()` infinite `while not self._stop_event.is_set()` 
- **Backoff strategy?** Yes — exponential: `min(5 * 2^attempt, 60)` seconds
- **Reconnect delay configurable?** No — hardcoded in `_run()`:
  ```python
  backoff = min(5 * (2 ** attempt), 60)
  ```
- **Infinite retry or fixed count?** Infinite — no retry limit
- **Connection state machine?** No — simple try/except/retry. States: `connected`, `connecting`, `error`, `reconnecting`, `streaming`, `starting`, `stopping` (from status file)
- **Child process crash recovery?** No — if `ntrip_rtcm_node.py` process dies, `_watch_process` clears state. The FastAPI server stays idle. **No automatic restart of the subprocess.**

### Internet drops for 20 seconds during mission:
1. Socket timeout after 30s (10s longer than the outage — it would actually survive the 20s drop → still connected when internet returns!)
2. **Actually: a 20s internet drop would NOT trigger a timeout** (socket timeout is 30s). The socket would either:
   - Stay in TCP keepalive state (no OS-level keepalive configured)
   - Eventually get a TCP RST when the network recovers
3. If internet returns before 30s socket timeout: **no disruption** — data resumes
4. If internet returns after 30s: reconnect triggers, takes 5s (first attempt) to 60s (later attempts)
5. During reconnect: no RTCM → GPS degrades from FIXED → FLOAT → 3D over ~30-60s

**Key gap: no keepalive configured.** TCP keepalive (SO_KEEPALIVE) is not set. The socket timeout is 30s for data recv. During the 20s internet drop, the process survives and automatically resumes when internet returns.

---

## RTCM Stream Validation

- **Preamble detection?** Yes: `buf[i] != _RTCM3_PREAMBLE` (0xD3)
- **Frame length validation?** Yes: reads bits 6-15 of the 2-byte length field:
  ```python
  msg_len = length_field & 0x03FF
  ```
- **CRC-24Q verification?** Yes: full implementation with precomputed lookup table:
  ```python
  computed_crc = _rtcm3_crc(frame, payload_len)
  ```
- **Corrupted frame rejection?** Yes: CRC mismatch discards frame, advances 1 byte:
  ```python
  if computed_crc != expected_crc:
      i += 1  # skip this preamble, scan for next
      continue
  ```
- **Malformed frame recovery?** Yes: any byte that isn't 0xD3 advances the scan pointer by 1
- **Partial frame buffering?** Yes: incomplete frames stay in `buf` for next recv:
  ```python
  if i + total_frame > len(buf):
      break  # wait for more data
  ```
- **Binary stream resynchronization?** Yes: scan-by-scan — if a 0xD3 appears mid-payload of a previous frame, the parser may falsely detect a frame at that offset, but CRC will fail and it skips.

### Can corrupted RTCM reach PX4?
**No.** Every frame passes CRC-24Q validation before publish. The parser is robust and follows the RTCM3 spec correctly.

**One edge case:** if a corrupted frame has a valid CRC by coincidence (1 in 2^24 ≈ 0.000006% probability). Effectively impossible.

---

## MAVROS Injection Reliability

- **Publisher QoS:** RELIABLE with depth=10, VOLATILE durability
  ```python
  rtcm_qos = QoSProfile(
      depth=10,
      reliability=ReliabilityPolicy.RELIABLE,
      durability=DurabilityPolicy.VOLATILE,
  )
  ```
- **Queue depth:** 10
- **gps_rtk plugin assumptions:** MAVROS `gps_rtk` plugin subscribes with RELIABLE QoS and expects individual RTCM frames. Each frame is published as a separate RTCM message. Queue depth 10 is adequate for typical RTCM rates (1-5 Hz).
- **Behavior if MAVROS temporarily disconnects:** RELIABLE publisher will queue up to 10 messages. If MAVROS reconnects quickly, messages are delivered. If disconnected longer, past 10 messages are dropped.
- **Behavior if FCU heartbeat disappears:** MAVROS may still be running (just waiting). RELIABLE QoS keeps publishing — messages go nowhere useful.
- **Publish exceptions handled?** No try/except around `self.pub.publish(msg)` — an exception here would propagate up to the main loop, breaking the connection and triggering reconnect.

### If /mavros restarts during mission:
1. NTRIP node survives (separate ROS2 node)
2. MAVROS restarts → its subscribers are destroyed and recreated
3. NTRIP node's publisher remains valid (DDS handles late-joining subscribers)
4. **Publish would recover automatically** — DDS discovery between nodes takes ~1-2s
5. RTCM frames published during the gap are lost (queue depth 10)

**Minor risk:** if MAVROS restart exceeds ~3 RTCM frames (at 5 Hz = 2s window), frame loss occurs. At typical RTCM rates (1-2 Hz), queue depth 10 covers ~5-10s of gap.

---

## GPS Fix Transition Safety

### Mission start safety
- `offboard_controller.py` `start_async()` checks `rpp_code in RPP_UNHEALTHY_CODES` first
- If RPP is in RTK_WAIT, start is **blocked** with message:
  ```
  "start: RPP RTK_WAIT — GPS fix < RTK_FIXED. Wait for fix..."
  ```
- This is a hard guard — mission cannot start without RTK FIXED

### In-mission degradation timeline
- NTRIP stops → RTCM ceases
- PX4 coast timer (EKF2_GPS_DELAY ≈ 250ms) expires
- GPS receiver (UM982) holds RTK fix for ~10-30s without corrections (depends on satellite geometry, ionospheric activity)
- fix_type drops: 6 → 5 (RTK_FLOAT) → 3 (3D FIX) over ~30-60s
- `rpp_controller_node.py` detects `fix_type < 6` at 50 Hz → immediately enters RTK_WAIT state
- Mission continues with zero velocity (RPP publishes zero)
- `server/main.py` telemetry loop detects `rpp_state=4 (RTK_WAIT)` in `RPP_UNHEALTHY_CODES`
- After `SAFETY_STALE_GRACE_S` (~2-5s estimated, needs verification from `config.py`), `emergency_handler.estop_async()` triggers
- Estop: stop-path + MANUAL mode + disarm

### RTK_WAIT detection latency
- RPP detects fix_type change within 1 control cycle (20ms at 50 Hz)
- Zero velocity published immediately
- **This is excellent — <20ms from fix degradation to zero-velocity command**

### Emergency stop trigger latency
- Server telemetry loop runs at TELEMETRY_HZ (configurable, likely 10 Hz)
- After first RTK_WAIT detection, `stale_since` is set
- After `SAFETY_STALE_GRACE_S` seconds, estop fires

---

## Hidden Reliability Weaknesses

### CRITICAL

| # | Weakness | File | Line(s) | Description |
|---|---|---|---|---|
| C1 | **No heartbeat monitoring** | `ntrip_rtcm_node.py` | — | No periodic "are you alive" check to caster. Relies entirely on passive data reception. |
| C2 | **Socket keepalive not configured** | `ntrip_rtcm_node.py` | 335 | `socket.socket()` created without `SO_KEEPALIVE`. Half-open TCP connections undetected for OS-dependent periods (often 2 hours+). |
| C3 | **No reconnect supervisor for child process** | `server/rtk_manager.py` | 200-207 | `_watch_process` clears state on exit but does NOT restart the subprocess. Systemd does not manage NTRIP subprocess. |
| C4 | **No stream timeout detection → reconnect** | `ntrip_rtcm_node.py` | 297-316 | `_check_health()` logs warnings but does NOT initiate socket close/reconnect when data stops flowing. Stale stream can persist forever. |

### HIGH

| # | Weakness | File | Line(s) | Description |
|---|---|---|---|---|
| H1 | **30s socket timeout** | `ntrip_rtcm_node.py` | 366 | `s.settimeout(30)` after connect. A 20s internet drop is invisible; a 31s drop wastes 30s + 5s reconnect = 35s total. |
| H2 | **No RTCM rate monitoring** | `ntrip_rtcm_node.py` | — | No expected frame rate check. A caster that sends 1 frame/hour passes health check. |
| H3 | **No last valid RTCM timestamp in connection logic** | `ntrip_rtcm_node.py` | 438-510 | `_run()` reconnect uses socket timeout only. Does NOT proactively reconnect if no valid frames for N seconds. |
| H4 | **No process group isolation for child process** | `server/rtk_manager.py` | 135-139 | `create_subprocess_exec` creates child in same process group. Server crash/kill can orphan the child. |
| H5 | **No auto-restart on server restart** | `server/main.py` | 153 | `AsyncRTKManager()` created fresh in `idle` state. NTRIP must be manually restarted after server restart. |
| H6 | **NTRIP not in systemd dependency chain** | `rover-server.service` | — | NTRIP is a subprocess of FastAPI. No systemd oversight. No restart policy. |

### MEDIUM

| # | Weakness | File | Line(s) | Description |
|---|---|---|---|---|
| M1 | **Frame publish not exception-wrapped** | `ntrip_rtcm_node.py` | 466 | `self.pub.publish(msg)` can throw (e.g., if DDS buffer full). Would propagate to main loop and trigger reconnect. |
| M2 | **GGA uses placeholder values** | `ntrip_rtcm_node.py` | 260-261 | `num_sats = 8`, `hdop = 1.0` — hardcoded. Caster may use these for client quality metrics. |
| M3 | **Status file written every 1s** | `ntrip_rtcm_node.py` | 146 | 1Hz write to tempfile + os.replace. Low impact but unnecessary I/O. |
| M4 | **No ICY header body scan** | `ntrip_rtcm_node.py` | 354-358 | `ICY 200 OK` detection handles minimal header but doesn't scan for `\r\n\r\n` in ICY response — relies on first-line-only for some casters. |
| M5 | **No bandwidth/jitter monitoring** | `ntrip_rtcm_node.py` | — | Not logged or exposed in status. |

### LOW

| # | Weakness | File | Line(s) | Description |
|---|---|---|---|---|
| L1 | `last_error` overwritten on reconnect attempt | `ntrip_rtcm_node.py` | 481 | Previous error info lost when new error overwrites it. |
| L2 | No DNS pre-resolution | `ntrip_rtcm_node.py` | 338 | `s.connect((NTRIP_HOST, NTRIP_PORT))` resolves on connect. If DNS changes IP mid-session, next connect uses new IP. |

---

## Telemetry Visibility

### `GET /api/rtk/status` response:

| Field | Exposed? | Notes |
|---|---|---|
| `mode` | ✅ | `"ntrip"`, `"lora"`, `"idle"` |
| `state` | ✅ | Via `source_state` from child status file |
| `connected` | ✅ | Boolean, reflects socket state |
| `reconnecting` | ❌ | Not exposed — `source_state` may show `"reconnecting"` |
| `bytes/sec` | ❌ | Not available in REST endpoint (but `_check_health` logs per-30s) |
| `RTCM rate` | ❌ | Not computed or exposed |
| `last valid packet age` | ✅ | `last_frame_age_s` — time since last RTCM frame |
| `caster latency` | ❌ | Not measured |
| `authentication failure` | ⚠️ | Visible in `last_error` if auth fails |
| `DNS failure` | ⚠️ | Visible in `last_error` if DNS fails |
| `socket timeout` | ⚠️ | Visible in logs only |

### Can operator know RTK pipeline health before fix drops?
**Partially.** The operator can:
1. Check `GET /api/rtk/status` → `connected=True`, `last_frame_age_s=N`
2. If `last_frame_age_s` climbs above ~10s, the `healthy` flag becomes `False`
3. Watch `gps_fix` in telemetry stream
4. See `rpp_state=4 (RTK_WAIT)` in telemetry

**Gap:** No proactive alerting. No per-second byte rate to detect degradation before total loss. No trend analysis.

---

## Production Mission Reliability Scenario

### Scenario: Hotspot disconnects for 30 seconds

| Time (s) | Event | Detected? | Action |
|---|---|---|---|
| T+0 | Hotspot drops | — | Socket remains open (TCP state machine waits for OS timeout) |
| T+0 to T+10 | No data arrives | `_check_health` logs "No RTCM data" at T+30s | None — socket not closed |
| T+30 | Socket timeout fires | ✅ | Reconnect triggered |
| T+30 | NTRIP process detects zero bytes after wait | ✅ | `except socket.timeout:` → break inner loop → reconnect |
| T+30 | Backoff: 1st attempt = 5s | ⚠️ | 5s wait, then attempt connect |
| T+30 to T+40 | RTCM stream stops | — | UM982 holds RTK FIXED for ~10-30s |
| T+40 to T+70 | fix_type drops 6→5→3 | ✅ RPP detects at 50Hz | RPP enters RTK_WAIT (zero velocity) |
| T+35 | Reconnect attempt #1 | May fail (no internet) | Backoff to 10s |
| T+45 | Reconnect attempt #2 | May fail (no internet) | Backoff to 20s |
| T+55 | Hotspot returns (T+30s elapsed) | — | — |
| T+55 to T+75 | Backoff #3 (40s) or #4 (60s) | — | Process is in backoff wait |
| T+65 to T+95 | Reconnect succeeds | ✅ | RTCM resumes |
| T+65 to T+95 | GPS re-acquires RTK FIXED | ✅ | RPP detects fix_type=6 → resumes velocity |
| T+65+grace | RTK_WAIT clears | ✅ | Mission resumes (if estop hasn't fired) |

### Estop timeline
- RPP enters RTK_WAIT at ~T+40 to T+70
- Server telemetry detects unhealthy at same time
- After `SAFETY_STALE_GRACE_S = 1.0s` (confirmed from `server/config.py` line 123), estop fires
- Estop: stop-path + MANUAL + disarm

**Critical finding:** `SAFETY_STALE_GRACE_S = 1.0s`. The estop WILL fire within 1 second of RTK_WAIT detection, which means:
- For a 30s internet drop: estop fires ~40-70s before internet returns
- For a 5s internet drop: estop still fires (RTK degrades in 10-30s, which is already past 1s grace)
- **Every internet outage that causes RTK degradation will trigger a full mission abort and disarm**
- Recovery cannot happen mid-mission without manual restart
- RTK degrades within 10-30s of correction loss
- RPP enters RTK_WAIT immediately
- Estop fires after grace period

**Mission abort is likely even for a short internet outage.**

### Process survival
The NTRIP process survives the 30s drop. The socket timeout triggers correctly. The reconnect loop eventually succeeds. There is no crash, no exit.

### Reconnect automation
The process automatically reconnects. Operator intervention is NOT required — the infinite retry loop handles it.

---

## Network Resilience Audit

- **Socket keepalive enabled?** **No.** `SO_KEEPALIVE` is not set on the socket.
- **Timeout configured?** Yes: 10s for initial connect, 30s for data receive.
- **Reconnect interval:** `min(5 * 2^attempt, 60)` seconds — 5, 10, 20, 40, 60, 60, 60, ...
- **ICY/HTTP response parsing:** Yes — handles both HTTP/1.0 and ICY headers. Checks for "200" in response. Detection of auth failure is `"200" not in header` which could false-negative if response is multi-line.
- **Stale socket detection:** Only via 30s recv timeout. No application-level keepalive.
- **Half-open TCP handling:** **Not handled.** A dead TCP socket (where both ends think the other closed but the network drops the RST) will block indefinitely on `recv()` until the 30s timeout fires. Without keepalive, the OS may take hours to detect half-open state.

### Can dead TCP socket remain undetected forever?
**Yes.** Without `SO_KEEPALIVE` and with a 30s recv timeout, a half-open TCP connection where the network silently drops packets:
- `recv()` blocks until timeout (30s) → "Socket timeout" → reconnect
- But if the caster sends periodic keepalive bytes (many casters do at 5-30s intervals), the socket stays alive indefinitely with zero valid RTCM frames
- `_check_health()` sees `_byte_count > 0` (from keepalive bytes) → doesn't trigger reconnect
- **Caster sending keepalive but no RTCM = permanent stale connection**

---

## Comparison With Production Requirements

| Requirement | Current Status | Score |
|---|---|---|
| User manually starts NTRIP | ✅ Yes — via API | **PASS** |
| Session survives temporary internet loss | ⚠️ Partial — survives socket timeout (30s), but no keepalive | **PARTIAL** |
| Automatic reconnect forever | ✅ Yes — infinite retry loop | **PASS** |
| Reconnect without operator intervention | ✅ Yes — fully automatic | **PASS** |
| Bad RTCM never reaches PX4 | ✅ Yes — CRC-24Q validation | **PASS** |
| Stale stream detected immediately | ❌ No — 30s health check does NOT trigger reconnect | **FAIL** |
| Mission only aborts when RTK actually degrades | ✅ Yes — fix_type<6 triggers RTK_WAIT | **PASS** |
| Transport failure separated from navigation failure | ⚠️ Partial — RPP distinguishes RTK_WAIT vs STALE vs JUMP_SKIP, but NTRIP transport status is not directly connected to mission abort logic | **PARTIAL** |

### Architecture Score: **Production Risky**

---

## Failure Matrix

| Failure Mode | Detection | Detection Time | Action | Auto-Recover | Safe? |
|---|---|---|---|---|---|
| **WiFi/Ethernet lost** | Socket timeout | 30s | Reconnect backoff | ✅ Yes | ⚠️ Slow detection |
| **DNS resolution failure** | `socket.connect()` exception | Immediate | Reconnect backoff | ✅ Yes | ✅ Fast |
| **Caster closes socket** | `recv()` returns empty | Immediate | Reconnect backoff | ✅ Yes | ✅ Fast |
| **Caster auth failure** | Response lacks "200" | Immediate | Reconnect backoff | ✅ Yes | ✅ Fast |
| **Corrupted RTCM** | CRC-24Q mismatch | Per-frame | Frame discarded | ✅ Yes | ✅ Safe |
| **Caster sends zero bytes** | Health check warning | 30s+ | Logs only | ❌ No | ❌ Stale stream |
| **Caster sends keepalive, no RTCM** | Health check sees bytes | Not detected | None | ❌ No | ❌ Hidden failure |
| **Half-open TCP** | Socket timeout | 30s | Reconnect backoff | ✅ Yes | ⚠️ Long detection |
| **MAVROS process death** | `px4_start_service.sh` watchdog | ~30s | Restart MAVROS | ✅ Yes | ✅ Safe |
| **NTRIP process crash** | `_watch_process` callback | ~1s | State cleared, idle | ❌ No restart | ❌ Needs manual |
| **Server restart** | — | — | NTRIP stopped, idle | ❌ No restart | ❌ Needs manual |
| **Rover reboot** | — | — | All services restart, NTRIP idle | ❌ No restart | ❌ Needs manual |
| **FCU disconnect** | MAVROS state timeout | 2s | `connected=False` | ✅ Yes | ✅ Safe |
| **RTK fix degradation** | `_gps_cb()` (GPSRAW) | <20ms | Zero velocity | ✅ On re-fix | ✅ Fast |
| **GGA socket failure** | Exception on sendall | 10s timer | Logs, suppressed after 3 fails | ⚠️ Warns only | ⚠️ Minor |

---

## Production Weaknesses (Ranked)

### 🔴 CRITICAL (Must fix before long mission)

| Rank | Issue | Impact |
|---|---|---|
| C1 | **No SO_KEEPALIVE on NTRIP socket** | Half-open TCP connections undetected for hours. Cannot trust connected=True during network partition. |
| C3 | **No auto-restart on NTRIP process crash** | If `ntrip_rtcm_node.py` crashes or is killed, RTK injection stops permanently until operator manually restarts via API. Mission loses RTK. |
| C2 | **No stream timeout action** | `_check_health()` detects no-data but does NOT close/reconnect. Stale connection with caster sending keepalive bytes = permanent stall. |

### 🟠 HIGH (Should fix for production)

| Rank | Issue | Impact |
|---|---|---|
| H1 | **No systemd supervision of NTRIP** | Subprocess lifecycle tied to FastAPI only. Server restart or crash orphans or kills the NTRIP child. |
| H2 | **30s socket timeout too long** | Every internet blip >30s causes observable GPS degradation. 15s or less preferred. |
| H4 | **No auto-start NTRIP on server boot** | After any restart, RTK dead until operator action. Forgets current session config. |
| H5 | **`SAFETY_STALE_GRACE_S` vs RTK coast time mismatch** | Estop likely fires before internet recovers. Mission aborts for short drops. |
| H6 | **No supervisor heartbeat to caster** | Cannot proactively detect caster failure. Relies on passive data reception. |

### 🟡 MEDIUM (Fix when convenient)

| Rank | Issue | Impact |
|---|---|---|
| M1 | **No publish exception handling** | Single publish failure triggers full reconnect + backoff. |
| M2 | **No last-good-frame-time utilization** | Reconnect could be triggered earlier by checking frame timestamp. |
| M3 | **GGA quality reporting incomplete** | Hardcoded sat count and HDOP may affect caster's client quota. |

### 🔵 LOW (Nice to have)

| Rank | Issue | Impact |
|---|---|---|
| L1 | Status endpoint lacks byte rate | Reduced observability of stream health |
| L2 | No ICY body scan | May miss some caster response headers |
| L3 | Auth error overwrites previous error | Debugging sequence harder |

---

## Final Verdict

# ⚠️ Production Risky

**Not Unsafe** — the core NTRIP → RTCM → MAVROS → PX4 path works correctly. RTCM validation is robust. Reconnection is automatic. RTK gating in RPP is precise (<20ms detection). Mission abort on degradation is safe.

**But not Production Ready** — three critical issues make long autonomous missions unreliable:

1. **No TCP keepalive** — the connection health indicator (`connected=True`) can lie during network partitions. A dead socket looks alive until the 30s recv timeout fires.
2. **No child process auto-restart** — if the NTRIP process dies mid-mission (OOM, segfault, signal), the manager transitions to idle permanently. RTK injection silently stops. The mission continues (until fix degrades) but cannot recover.
3. **No action on stream timeout** — the 30s health timer warns but does not act. A caster that sends keepalive bytes without RTCM frames creates a permanently stale connection that looks healthy.

### Key Strengths (that must NOT be regressed):
- ✅ RTCM3 CRC-24Q validation before every publish — no corrupted corrections reach PX4
- ✅ Exponential backoff reconnect with infinite retries
- ✅ 50 Hz RTK fix monitoring with immediate zero-velocity on fix loss
- ✅ Clear separation between transport failure and navigation failure (RPP distinguishable codes)
- ✅ MAVROS process death detection via 2s state timeout in `ros_node.py`
- ✅ RELIABLE QoS on RTCM publisher with depth=10
- ✅ Manual start prevents unintended NTRIP connections

### Recommended minimum fixes:
1. Add `SO_KEEPALIVE` with 10s idle, 3 probes, 5s interval to NTRIP socket
2. Make `_check_health()` close and reconnect when no valid RTCM frames arrive for >30s
3. Add auto-restart capability to `AsyncRTKManager` (or wrap in systemd-managed service)
4. Reduce initial socket timeout from 30s to 10-15s
5. Add last-valid-frame timestamp to reconnect trigger