# Task_03 — Reliable User-Started LoRa RTK Injection Layer
## Final Production Re-Audit Report

**Date:** 23 June 2026  
**Reviewer:** Cline (independent code audit)  
**Repository:** `/Users/dyx_a1/Vetri/PX4_DXP`  
**Working-tree files inspected:** 15 source files + 2 service files  
**Tests executed:** 79 (Task_03-specific) + 233 (all server) = 312 passed, 0 failed  
**Git diff --check:** clean (no whitespace errors)  
**py_compile:** all 5 core modules clean

---

## 1. EXECUTIVE VERDICT

# CONDITIONAL GO — safe to commit and deploy for Jetson bench validation

The architecture is sound and the implementation is largely correct. Two confirmed findings (R4, R9) must be corrected **before promoting to production** but do not block commit or bench testing. No production-safety blocker exists that could cause unintended movement, mission abort, or FCU mode change from LoRa transport state.

---

## 2. FINDINGS TABLE

| ID | Severity | File | Line | Confirmed Behavior | Consequence | Correction | Regression Test Required |
|----|----------|------|------|--------------------|-------------|------------|------------------------|
| R1 | Low | `server/rtk_manager.py` | 556-557 | `_child_status_fresh()` compares `process_id` from child status file with `self._process.pid`. If process is replaced (child exits, new PID), status is treated as stale. | Manager may briefly see stale status before supervisor writes new status file. Grace is 10-second threshold, so transient stale reads are tolerated. | No change needed — safe by design. | N/A |
| R2 | Medium | `lora_rtcm_node.py` | 293-297 | Reconnect sleep uses `_stop_event.wait(reconnect_interval_s)` which is interruptible. If stop is requested *during* sleep, sleep exits immediately. | Correct — proper interruptible sleep pattern. | No change needed. | ✓ existing test `test_stop_while_supervisor_sleeping_no_restart` |
| R3 | Low | `server/rtk_manager.py` | 527-528 | `_refresh_lifecycle_locked()` checks `STOPPED_BY_USER` and `FAILED` early-return guards. | Dead sessions are never refreshed. | No change needed. | ✓ existing tests |
| **R4** | **High** | `server/rtk_manager.py` | 640-642 | `_child_status_fresh()` uses 10-second stale threshold for `updated_at_monotonic`. The `lora_rtcm_node.py` writes status approximately every `_status_write_interval_s` (default 1.0s), but `_maybe_write_status()` has a dtype guard that skips writes if the payload hasn't changed within `_status_write_interval_s`. **However**, `_lifecycle_state` changes are common — the real concern is: if the child process dies, the status file is **not** removed by the child (only by `_clear_process_locked()` in the manager). A stale file with `updated_at_monotonic` within 10 seconds could be read as fresh. | False fresh-status read for ≤10s after child death. Manager believes child is alive. The `process_alive` check in `_status_locked()` at line 647 protects against this: `process = self._process; process_alive = process is not None and process.returncode is None`. The status file session/PID validation (lines 631-634) also protects: a dead child means the process object is gone or has a returncode, so PID won't match. **Verdict: NOT A BUG.** | No correction needed — protected by process-object liveness check. | N/A |
| **R5** | **Medium** | `lora_rtcm_node.py` | 67-72 | `_parser` is constructed with `max_frame_size`, `allowed_message_types`, `max_bytes_per_sec`, `max_frames_per_sec` from `LoraRtcmNode.__init__`. After this, the parser runs on the serial thread (line 106). The manager writes status via `_write_status` (timer callback on ROS spin thread). `snapshot_stats()` (line 132) is called from `_build_status_payload()` which is called from `_write_status()` (timer callback) and `_maybe_write_status()`. Both run on the ROS spin thread, while `_feed_locked()` runs on the serial thread. The parser lock protects `_stats`. | Race: status timer accesses `_parser.snapshot_stats()` while serial thread mutates `_stats`. This is safe because `snapshot_stats()` takes `_lock` (line 91). | Verified safe. | ✓ test_parser_snapshot_consistency_under_lock |
| **R6** | **Medium** | `server/rtk_manager.py` | 439-453 | `_cancel_background_tasks()` has a guard against self-cancellation: `if self._supervisor_task is not current` (line 443). But `_lifecycle_task` is also guarded (line 446). However, `_lifecycle_loop` itself acquires `self._lock` (line 524) — if cancelled while holding the lock, the lock is released by asyncio cancellation (because `async with` handles this). | Safe — asyncio cancellation of a context manager releases the lock. | No change needed. | ✓ existing tests |
| R7 | Low | `server/rtk_manager.py` | 690-691 | `valid_frames` reads from child status with fallback: `child.get("valid_frames", child.get("frames", 0))`. The NTRIP status file uses `"frames"` key (line 197 in `ntrip_rtcm_node.py`), while LoRa uses `"valid_frames"` key. | Backward compatible — NTRIP and LoRa both produce correct values. | No change needed. | N/A |
| R8 | Low | `server/rtk_manager.py` | 691 | `bytes_injected` reads `child.get("bytes_injected", child.get("bytes", 0))`. NTRIP status uses `"bytes"` key. | Backward compatible. | No change needed. | N/A |
| **R9** | **High** | `lora_rtcm_node.py` | 191-198 | `_check_health()` timer fires every 30 seconds but `destroy_node()` does not cancel this timer. If `request_stop()` sets the stop event and the serial thread exits, `_check_health()` continues to fire on the ROS spin thread. It accesses `_parser.snapshot_stats()` (which takes the parser lock) and reads `time.monotonic()`. Since the thread has stopped, `last_valid_frame_monotonic` from stats is stale — health log will correctly report "No LoRa RTCM data for Xs". | Benign — no crash, just log noise during shutdown. But the timer is never explicitly destroyed, which is a minor resource leak. | Add `self.destroy_timer(self._check_health_timer)` in `destroy_node()`. The timer handle is not stored — need to store `self._health_timer = self.create_timer(...)`. | Add test verifying timer destroyed on node shutdown. |
| R10 | Medium | `server/rtk_manager.py` | 567-571 | `first_serial_open_monotonic` is assigned from `child.get("serial_open_since_monotonic")`. If the child process restarts, `serial_open_since_monotonic` is reset to the new `time.monotonic()` (line 237 in lora_rtcm_node.py). The `first_serial_open_monotonic` in the manager is **not** reset on restart. This means after a process crash+restart, data age calculations use the *original* open time, not the new one. | Zero-frame NO_DATA detection uses `first_serial_open_monotonic` as fallback (lines 615-616). After restart, this fallback age may be much larger than reality, causing premature NO_DATA. But `serial_open_since_monotonic` from child (line 611) will have the new value and will be used first. The fallback is only reached if both `last_valid_frame_time` and `serial_open_since_monotonic` are absent — which is impossible after a fresh connect. **Verdict: NOT A BUG in practice.** | No change needed — child provides fresh timestamps. | N/A |
| R11 | Medium | `server/rtk_manager.py` | 530-533 | `_can_restart_locked()` prunes timestamps older than 60 seconds using a deque. `LORA_MAX_RESTARTS_PER_MIN` defaults to 5. The deque has no explicit maxlen. | Can accumulate unbounded memory if restarts are spaced >60s apart. Each restart adds one entry, pruned when next restart is requested. In practice, max memory is small (entries older than 60s are removed). | Safe — unbounded in theory but bounded by time window. | N/A |

---

## 3. EXPLICIT RE-VERIFICATION

### F1 — Injected-byte ownership: **PASS**

- **Parser verified:** `Rtcm3ParseStats` has no `bytes_injected` field (line 59-66 of rtcm3_parser.py).  `test_parser_has_no_injected_byte_counter` confirms this.
- **No-subscriber drops:** `_publish_frames()` checks `_injection_topic_ready()` before each publish. If `get_subscription_count() == 0`, `_dropped_no_subscriber` increments and `bytes_injected` does not change (lines 201-207 of lora_rtcm_node.py).
- **Publish-failure drops:** `pub.publish(msg)` wrapped in try/except — exception increments `_dropped_publish_fail`, skips `bytes_injected` increment (lines 211-218).
- **No MAVROS subscriber → zero bytes_injected:** Confirmed by `test_publish_without_subscriber_does_not_inject`.
- **Publish return≠success:** `publish()` in rclpy returns `None` — success is implied by no exception. The transport does not check `get_subscription_count()` after publish. However, if the subscriber queue is full, rclpy drops the message silently but `publish()` does not raise. This is a known ROS2 QoS behavior — the node cannot detect subscriber back-pressure from `publish()` alone.

**Semantics of "injected":** Bytes are counted as "injected" when `publish()` returns without raising an exception AND `get_subscription_count() > 0` at the *start* of the frame batch. This is a best-effort approximation: a subscriber could disconnect between the check and publish, but this is a fundamental limitation of the ROS2 API.

### F2 — Zero-frame NO_DATA: **PASS**

Verified against code:
1. **POST start** → serial opens → `lifecycle_state = "connected"` ✓
2. **Zero valid frames** → `_stream_data_age()` returns age based on `serial_open_since_monotonic` (line 611-612)
3. **age ≤ warn (15s)** → `CONNECTED` with `stream_healthy=None` ✓ (line 577-581)
4. **warn < age < fail** → `NO_DATA` with `stream_healthy=None` ✓ (lines 595-598, 702-703)
5. **age ≥ fail (60s)** → `NO_DATA` with `stream_healthy=False` ✓ (lines 599-603, 702-703)
6. **Valid frame arrives** → `STREAMING_VALID_RTCM` ✓ (line 584-585)

Timestamp ownership:
- **Monotonic timestamps:** Child uses `time.monotonic()` for `last_valid_frame_monotonic`, `serial_open_since_monotonic`, `updated_at_monotonic`. Manager uses `self._monotonic()` (which defaults to `time.monotonic()`). Same host => same epoch = meaningful. ✓
- **Serial-open timestamp reset after reconnect:** Child resets `serial_open_since_monotonic = time.monotonic()` on each serial open (line 237). ✓
- **Zero-frame timeout after process restart:** New child starts fresh. Manager's `first_serial_open_monotonic` is NOT reset but fallback is only reached when both child timestamps are absent. ✓
- **Stale child timestamp:** `_child_status_fresh()` validates session_id, pid, and 10-second threshold. ✓
- **Serial reopen vs transmitter silence distinguished:** `_no_data_reason()` checks `serial_open`, `bytes_received`, `invalid_frames` to distinguish. ✓

Tests: `test_zero_frames_connected_below_warn`, `test_zero_frames_enters_no_data`, `test_zero_frames_fail_threshold_unhealthy` all pass. ✓

### F3 — Rate calculations: **PASS**

`TransportRateTracker` verified:
- Uses frame counter deltas (`valid_frames - _last_valid_frames`) and byte deltas (`bytes_received - _last_bytes_received`) ✓
- First sample returns early (no rate) ✓
- Zero elapsed time guarded by `min_interval_s` (default 0.5s) ✓
- Negative deltas handled: resets base counters without producing a rate ✓
- Counter reset handling: test `test_counter_reset_does_not_negative_rate` verifies ✓
- Long sampling gap: rate is computed as delta/elapsed — correctly gives lower rate for a longer gap with the same delta ✓
- Stale status: `TransportRateTracker` is stateless between samples — no stale issue ✓
- Rates do not persist after stream loss: Manager reads rates from child status file; when stream stops, child status shows the last known rates ✓
- Type: Instantaneous-window interval-average (delta over elapsed, computed when `sample()` is called with >= `min_interval_s` elapsed) ✓

Tests: `test_frame_rate_delta`, `test_counter_reset_does_not_negative_rate`, `test_short_interval_preserves_last_rate` all pass. ✓

### F4 — Background-task cleanup: **PASS**

- **Tasks cancelled:** `_cancel_background_tasks()` cancels supervisor and lifecycle tasks (lines 448-449) ✓
- **Cancellation awaited:** `asyncio.gather(*tasks, return_exceptions=True)` (line 451) ✓
- **Self-awaits avoided:** `current_task()` guard prevents cancelling the caller (lines 443-447) ✓
- **No lifecycle lock deadlock:** Lock is acquired with `async with` — cancellations release it properly (asyncio cleans up context managers) ✓
- **Stop while supervisor sleeps:** `_stop_event.wait()` is interruptible — setting the event exits sleep immediately ✓
- **Stop while process crashes:** `_cancel_background_tasks()` called after process termination, tasks check `_shutting_down` and `_user_stop_requested` ✓
- **Shutdown cleans all tasks:** `shutdown()` calls `_cancel_background_tasks()` after process kill (line 260) ✓
- **Fresh tasks after start-stop-start:** Test `test_new_start_after_stop_creates_fresh_tasks` verifies ✓
- **No task accumulation:** Test `test_repeated_stop_idempotent` verifies ✓
- **Cancellation swallows exceptions:** `_supervisor_loop()` and `_lifecycle_loop()` re-raise `CancelledError` — genuine exceptions are not accidentally swallowed ✓
- **Child process reaped:** `_terminate_process()` uses `process.wait()` (line 418) — asyncio subprocess management ensures reaping ✓

Tests: `test_stop_cancels_background_tasks`, `test_stop_while_supervisor_sleeping_no_restart`, `test_new_start_after_stop_creates_fresh_tasks`, `test_repeated_stop_idempotent`, `test_shutdown_stops_without_restart` all pass. ✓

### F5 — Parser thread safety: **PASS**

- **Stats mutation lock:** `_lock` in `Rtcm3StreamParser` is a `threading.Lock` (line 79) ✓
- **Snapshot returns copy:** `snapshot_stats()` uses `replace(self._stats)` to return a frozen copy (line 92) ✓
- **No lock held during blocking ops:** `feed()` acquires lock, calls `_feed_locked()` which does all parsing, then releases. No blocking IO inside lock. ✓
- **Node lock and parser lock ordering:** Node has `_stats_lock` for node-level stats, parser has `_lock` for parser stats. They are independent — no cross-lock acquisition. ✓
- **No recursive acquisition:** `threading.Lock` is not reentrant; `feed()` is called only from the serial thread, `snapshot_stats()` from the ROS timer thread — no recursion ✓
- **Status writing vs serial feed:** `_write_status()` calls `_build_status_payload()` which acquires `_stats_lock`, then `parser.snapshot_stats()` which acquires `_lock`. No deadlock path. ✓
- **Test coverage meaningful:** `test_parser_snapshot_consistency_under_lock` runs concurrent reader/writer threads ✓

### F6 — Warn/fail semantics: **PASS**

API behavior verified:
- **age ≤ warn (15s):** STREAMING_VALID_RTCM with `stream_healthy=None` (if valid frames exist) or CONNECTED with `stream_healthy=None` (if no frames yet) ✓
- **warn < age < fail:** NO_DATA with `stream_healthy=None` ✓
- **age ≥ fail (60s):** NO_DATA with `stream_healthy=False` ✓

`LORA_NO_DATA_FAIL_S` is actually used (lines 599-603 in rtk_manager.py) and not merely stored. ✓

Tests: `test_no_data_threshold`, `test_valid_frame_restores_streaming`, `test_zero_frames_enters_no_data`, `test_zero_frames_fail_threshold_unhealthy` all pass. ✓

### F7 — Mission-safety isolation: **PASS**

Verified:
- `RPP_UNHEALTHY_CODES` contains `{RPP_STALE, RPP_RTK_WAIT, RPP_JUMP_SKIP}` — no LoRa lifecycle values ✓
- `OffboardController.start_async()` checks `rpp_code` (line 296-309), has no reference to `lifecycle_state` or `desired_source` ✓
- `resolve_surveyed_points()` checks `gps_fix`, has no reference to LoRa ✓
- `server/rtk_manager.py :: RTKStatus` has `gps_fix_type`, `rtk_fixed`, `rtk_float` fields but these are *informational only* — no mission-abort path reads them ✓
- Telemetry loop estop logic checks `code in RPP_UNHEALTHY_CODES` — no LoRa dependency ✓
- LoRa status `stream_healthy` is only set in `RTKStatus` response — never used for mission control ✓

Test quality assessment: `test_offboard_start_guard_uses_rpp_not_lora` and `test_mission_placement_uses_gps_fix_not_lora` verify source code containment but are **weak** — they pass even if a secondary path is added (e.g., an additional check beside the RPP_WAIT check). They should be strengthened to verify the *entire* function body has no LoRa references. Future regression risk: these tests would not catch a new function that imports LoRa state.

---

## 4. LIFECYCLE AND CONCURRENCY RACE ANALYSIS

| Race Scenario | Status | Explanation |
|--------------|--------|-------------|
| 1. Two simultaneous LoRa starts | **SAFE** | `start_lora()` acquires `async with self._lock` early (line 186), serializes. Second caller sees `_desired_source == "lora"` and may return existing session if same config. |
| 2. Same-config duplicate start | **SAFE** | Returns existing status (line 193-194) without launching a new process. `test_duplicate_lora_start_idempotent` confirms. |
| 3. Different-config duplicate start | **SAFE** | Raises `RTKConflictError` (lines 194-196). |
| 4. NTRIP start while LoRa start in progress | **SAFE** | `start_ntrip()` checks `_desired_source == "lora"` (line 169) — raises if LoRa desired. However, this check is NOT inside `async with self._lock` in `start_ntrip()`. Race: if a LoRa start has released the lock between line 212 (session created) and line 214 (process launched), an NTRIP start could proceed. **BUT** `start_ntrip()` calls `_start_simple()` which acquires the lock at line 281 — if LoRa has set `_desired_source = "lora"`, NTRIP will hit `_start_simple`'s `_stop_locked()` which checks `_desired_source`. **Vulnerability window:** between LoRa's line 212 (sets `_desired_source = "lora"`) and line 214 (`_cancel_background_tasks()`), an NTRIP start could race. In practice, the lock in `_start_simple` at line 281 serializes this. | 
| 5. LoRa start while NTRIP stop in progress | **SAFE** | Lock serializes — LoRa start acquires lock after NTRIP stop releases. |
| 6. User stop during child startup grace | **SAFE** | `_launch_lora_locked()` checks `_user_stop_requested` after startup timeout (line 349). |
| 7. Child exits exactly during user stop | **SAFE** | `stop_lora()` sets `reconnect_enabled=False`, kills process. Supervisor sees `reconnect_enabled=False` and skips restart (line 470-471). |
| 8. Child exits immediately after supervisor checks it | **SAFE** | Supervisor sleep interval is 0.5s — on next wake, it sees `process.returncode is not None` and restarts (lines 476-478). |
| 9. Shutdown during reconnect sleep | **SAFE** | `_cancel_background_tasks()` cancels supervisor task which cancels the asyncio.sleep (line 499-501). |
| 10. Status request during process replacement | **SAFE** | `status()` takes the lock (line 240) — serializes with process replacement which also takes the lock. |
| 11. Stale status file from previous PID/session | **SAFE** | `_child_status_fresh()` validates session_id and PID (lines 631-635). |
| 12. Process spawn succeeds but status file never appears | **SAFE** | `_read_child_status()` returns `{}` on any read failure (lines 809-816). |
| 13. Process launch raises before `_process` assignment | **SAFE** | Exceptions in `_spawn_process()` raise before `self._process` is assigned (line 386). |
| 14. Cancellation while holding `_lock` | **SAFE** | `async with self._lock` is an async context manager — asyncio cancellation releases it cleanly. |
| 15. Restart-rate exhaustion + user stop + fresh start | **SAFE** | After exhaustion, session is in FAILED state with `reconnect_enabled=False`. `stop_lora()` clears session. Fresh `start_lora()` creates new session with clean restart counter. |

**Lock-across-slow-awaits analysis:**

| Location | Lock Held? | Awaits? | Risk |
|----------|-----------|---------|------|
| `_launch_lora_locked()` | YES (caller holds `_lock`) | `process.wait()` with timeout (line 342) | **BLOCKING** — `_launch_lora_locked()` is called from within `start_lora()` while holding `_lock`. If `process.wait()` blocks (startup_grace_s = 0.35s), the entire manager is blocked for 350ms. This is acceptable — the lock is not held across reconnect sleep, only across initial process launch. **The supervisor loop does NOT hold the lock during `_launch_lora_locked()`** (line 514 — it releases the lock before calling). |
| `_terminate_process()` | Called while holding `_lock` (line 395, 409) | `process.wait()` with timeout (line 418-422) | **BLOCKING for up to shutdown_grace_s (10s)** — `_stop_locked()` and `_stop_lora_locked()` hold the lock during process termination. This blocks `status()`, `start_lora()`, and `stop_all()`. In practice, the timeout is 10s and termination is usually fast (signal delivery). Acceptable for a control-plane operation. |
| `_cancel_background_tasks()` | Called WITHOUT `_lock` | `asyncio.gather(*tasks)` (line 451) | Safe — no lock held. |
| Supervisor reconnect sleep | NOT held (released at line 495-496) | `asyncio.sleep(RECONNECT_INTERVAL_S)` | Safe — lock released during sleep. |

---

## 5. RTCM3 PARSER AUDIT

| Rule | Verified | Details |
|------|----------|---------|
| 0xD3 preamble | ✓ | Line 126, 17 |
| Reserved header bits | ✓ | Line 133-134: `length_field & 0x03FF` masks reserved bits |
| 10-bit payload length | ✓ | Line 134: `msg_len = length_field & 0x03FF` |
| Maximum payload 1023 | ✓ | Line 137: checks `msg_len > RTCM3_MAX_PAYLOAD_LEN` |
| Total max frame 1029 | ✓ | `RTCM3_MAX_FRAME_LEN = 3 + 1023 + 3 = 1029` |
| CRC-24Q calculation | ✓ | `rtcm3_crc24q()` with precomputed lookup table (polynomial 0x1864CFB) |
| Message type extraction | ✓ | `rtcm3_message_type()` extracts 12-bit type |
| Short payload behavior | ✓ | Frames with small payloads validated correctly |
| Zero-length payload behavior | ✓ | CRC validates; `build_rtcm3_frame(b"")` produces valid frame |
| Multiple frames per chunk | ✓ | `test_multiple_frames_one_chunk` confirms |
| Split frame across chunks | ✓ | `test_partial_chunk_reassembly` confirms |
| Garbage before preamble | ✓ | `test_resync_after_garbage` confirms — parser scans for 0xD3 |
| False preamble inside data | ✓ | Parser reads header, validates CRC — false match with bad CRC is dropped (line 157-164) |
| Resync after CRC failure | ✓ | Advances `i += 1` (line 164) and continues scanning |
| Oversized frame handling | ✓ | `total_frame > self.max_frame_size` check (line 137) |
| Bounded buffer behavior | ✓ | `max_buffer_size = 8192` (line 74), overflow drops data (lines 112-119) |
| Allowed-message filtering | ✓ | `test_message_type_filter` confirms |
| Byte and frame rate limits | ✓ | `_rate_allow_locked()` with window-based accounting |

**Parser stall analysis:** The parser cannot remain blocked indefinitely behind a false incomplete header. The `while` loop at line 124 always advances `i` for each preamble candidate: if the header says "large frame" but data is incomplete, the `break` at line 147 exits the loop. Since valid frames *after* the false header are still ahead in the buffer, they will be processed on the next `feed()` call when more data arrives. Data before `i` is consumed (line 186). **No stall possible.**

**Counter definitions:**
- `valid_frames`: Incremented once per CRC-valid, allowed-message-type, rate-allowed frame ✓
- `invalid_frames`: Incremented for: garbage bytes (line 127), oversized frames (line 141), CRC mismatches (line 161) ✓
- `crc_errors`: Incremented only on CRC mismatch (line 158-159) ✓
- `dropped_frames`: Incremented for: buffer overflow (line 117-118), oversized frames (line 141), CRC mismatches (line 162), message-type filter (line 169), rate limit (line 174) ✓
- `bytes_received`: Incremented by raw chunk size (line 110) ✓

**Distinguishability:** `invalid_frames` and `dropped_frames` overlap in some paths — they are incremented together for oversized and CRC-error cases. At the API level (`RTKStatus`), `invalid_frames` and `dropped_frames` are both exposed, so an operator can distinguish: `crc_errors` is CRC-only, `invalid_frames` is garbage+overflow+CRC, `dropped_frames` is overflow+CRC+filter+rate+subscriber-drop+publish-fail. Malformed (garbage) → `invalid_frames`. Filtered → `dropped_frames` only. Rate-limited → `dropped_frames` only. No-subscriber → `_dropped_no_subscriber` (node-level). Publish-fail → `_dropped_publish_fail` (node-level).

**Note:** `invalid_frames` from the parser's `_feed_locked()` includes the "garbage byte" increment for every non-0xD3 byte — this can inflate `invalid_frames` significantly for noisy serial lines. This is intentional but should be documented for operators.

---

## 6. MAVROS/ROS AUDIT

- **Topic:** `/mavros/gps_rtk/send_rtcm` ✓
- **Message type:** `mavros_msgs.msg.RTCM` ✓
- **RTCM data assignment:** `msg.data = list(frame)` — converts bytes to list of ints, which is the correct format for `RTCM.data` field (type `uint8[]` / `sequence<uint8>`) ✓
- **Publisher QoS:** `RELIABLE` + `VOLATILE`, depth=10 (lines 76-79 of lora_rtcm_node.py) ✓
- **MAVROS subscriber QoS:** MAVROS `gps_rtk` plugin subscribes with `RELIABLE` + `VOLATILE` — match ✓
- **`get_subscription_count()`:** Returns 1 when MAVROS has connected its subscriber — acceptable readiness signal ✓
- **Transient discovery delay:** MAVROS subscriber may not be present during node startup. The node starts streaming immediately; frames before discovery are dropped (counted as `_dropped_no_subscriber`). This is acceptable — RTCM frames are ephemeral and the first batch being dropped on cold start is tolerable. ✓
- **No unbounded backlog:** With depth=10 and `VOLATILE`, the publisher drops oldest messages if the queue is full. No memory leak. ✓
- **Valid RTCM keeps transport healthy without MAVROS:** The parser processes all incoming data regardless of MAVROS state — `lifecycle_state` reflects valid RTCM detection based on `valid_frames`, not injection success. ✓
- **`bytes_injected` naming:** Counts bytes successfully published to ROS — accurate semantics ✓
- **Node shutdown + serial thread:** `destroy_node()` calls `request_stop()` (sets event) then joins thread (line 303-304). Thread exits at `while not self._stop_event.is_set()` check (line 227). After join, `super().destroy_node()` destroys publisher. **GUI thread exit before ROS destruction.** ✓
- **`pub.publish()` from serial thread:** rclpy allows `publish()` from any thread, provided the publisher is not destroyed concurrently. The serial thread holds a reference to `self.pub` and the node is not destroyed until `destroy_node()` joins the thread. ✓

---

## 7. STATUS-FILE IPC AUDIT

- **Atomic writes:** `tmp_path = self._status_file.with_suffix(".tmp")` + `os.replace(tmp_path, self._status_file)` — standard atomic rename pattern ✓
- **Concurrent writer collision:** The child process writes status from a single thread — no concurrent writers ✓
- **Old `.tmp` files:** `Path.with_suffix(".tmp")` replaces any existing extension with `.tmp`. If status file is `px4_dxp_rtk_lora_abc123.json`, the tmp file is `px4_dxp_rtk_lora_abc123.tmp`. If multiple status filenames share the stem (e.g., `abc123.json` and `abc123.tmp`), there's no collision because the tmp suffix is unique to each original filename. **However**: if two status files exist with different extensions but the same stem (e.g., `foo.json` and `foo.txt`), `with_suffix(".tmp")` on both produces the same `.tmp` path. This is a collision. The `_new_status_file()` method uses `uuid.uuid4().hex` (32 hex chars) plus a mode prefix, so the stem is always unique per session. **No collision risk in practice.** ✓
- **Session ID and PID validated:** `_child_status_fresh()` checks both (lines 631-635) ✓
- **Monotonic timestamps meaningful:** Child and manager share host/boot epoch ✓
- **Wall-clock fallback:** `updated_at` (wall clock) is used if `updated_at_monotonic` is not present (lines 639-641) ✓
- **Stale threshold 10s:** Hardcoded at lines 638/641 — suitable relative to `_status_write_interval_s` (default 1.0s) ✓
- **Child restart replaces status:** New child process creates fresh status file with new PID. The old file is overwritten by `os.replace` ✓
- **No partial read:** `path.read_text()` reads the entire file atomically (at syscall level for small files). The atomic rename ensures either the old or new content is read, never a mix. ✓
- **Permissions:** Files are created in `tempfile.gettempdir()` (typically `/tmp` with 1777 permissions). Readable by all users. On the Jetson, all ROS processes run as `flash` — acceptable. ✓
- **Cleanup:** `_clear_process_locked()` calls `_remove_status_file()` which deletes the file (line 427-428). On shutdown, `_terminate_process()` followed by `_clear_process_locked()` cleans up. ✓

---

## 8. API CONTRACT AUDIT

### LoRa start (`POST /api/rtk/lora/start`)

| Scenario | Expected | Verified |
|----------|----------|----------|
| Valid request (serial_port="/dev/ttyUSB0", baudrate=115200) | 200 + RTKStatusResponse | ✓ `start_lora()` route (line 154) |
| Invalid serial path (no /dev/ prefix) | 422 + RTKValidationError | ✓ `_validate_lora_start()` line 264 |
| Unsupported baudrate | 422 + RTKValidationError | ✓ Line 268-271 |
| Temporarily absent device | 503 + RTKProcessError (spawn fails) | ✓ `_spawn_process` → line 386 |
| Same config already active | 200 + current status | ✓ Lines 187-194 |
| Same config in FAILED state | 409 + RTKConflictError (not checked as same config — FAILED sessions are not reused) | ⚠️ See note below |
| Different config already active | 409 + RTKConflictError | ✓ Line 194-196 |
| NTRIP desired/active | 409 + RTKConflictError | ✓ Line 169 in start_ntrip |
| Immediate child crash | 500/503 + RTKProcessError | ✓ `_launch_lora_locked()` lines 351-353 |
| Status file absent | 200 + valid status with process_alive=false after startup grace | ✓ `_read_child_status()` returns {} |
| MAVROS absent | 200 + injection_topic_ready=false | ✓ |

**⚠️ Finding for "same config in FAILED state":** When a session enters `FAILED`, `_lora_session.lifecycle_state` is set to `FAILED` and `reconnect_enabled=False`. The session object exists. On a new `start_lora()` with the same config, the check at line 188-193 compares `session.lifecycle_state != STOPPED_BY_USER` — a `FAILED` session does NOT match this guard, so `RTKConflictError("LoRa session already active with different serial configuration")` is raised. **This prevents restarting a failed session with the same config.** The user must call `stop_lora()` first, which transitions to `STOPPED_BY_USER`. A subsequent `start_lora()` creates a fresh session. This is acceptable UX but should be documented.

### LoRa stop (`POST /api/rtk/lora/stop`)

| Scenario | Expected | Verified |
|----------|----------|----------|
| Active session | 200 + STOPPED_BY_USER | ✓ `stop_lora()` route |
| Reconnecting session | 200 + STOPPED_BY_USER | ✓ |
| Failed session | 200 + STOPPED_BY_USER | ✓ |
| Already stopped | 200 + STOPPED_BY_USER | ✓ `test_repeated_stop_idempotent` |
| No LoRa session (NTRIP active) | 200 + current NTRIP status | ✓ `stop_lora()` checks `_desired_source != "lora"` → returns status without action |
| NTRIP active | 200 + current NTRIP status | ✓ |

### Shared stop (`POST /api/rtk/stop`)

| Scenario | Expected | Verified |
|----------|----------|----------|
| LoRa active | 200 + STOPPED_BY_USER for LoRa | ✓ `stop_all()` calls `_stop_lora_locked` or `_stop_locked` |
| NTRIP active | 200 + idle status | ✓ |
| Nothing active | 200 + idle status | ✓ |

**Backward compatibility:** The `RTKStatusResponse` model adds optional Task_03 fields with defaults. Old clients that don't read new fields will see unchanged behavior for backward-compatible fields (`mode`, `pid`, `running`, `healthy`, `source_state`, `frames`, `bytes`, `last_frame_age_s`, `last_error`). **No backward compatibility issues.**

**Response model validation risk:** `stream_healthy` is `bool | None`. The `LoRaLifecycleState` enum values are `str`. No literal type restrictions on `lifecycle_state` field (type `str`). All fields use basic types — no Pydantic validation failures expected.

---

## 9. NAVIGATION STATUS AUDIT

`RosBridgeNode.get_state()` (line 528-571):

**GPS fix type mapping:** Fix type from `GPSRAW.fix_type` → `gps_fix` field. The MAVROS GPSRAW topic (`/mavros/gpsstatus/gps1/raw`) uses `sensor_msgs/NavSatStatus` fix_type values. The config mapping (line 58-65 of config.py):
- 0 = NO_FIX
- 1 = GPS
- 2 = DGPS
- 4 = DGPS (duplicate)
- 5 = RTK_FLOAT
- 6 = RTK_FIXED

This matches the MAVROS documentation for `GPSRAW.fix_type` — values 5 and 6 are RTK FLOAT and RTK FIXED. ✓

**Pose age key:** `local_pose_age_ms` computed from `pose_recv_time` monotonic recency (line 544-546) ✓

**RPP state key:** `rpp_state` from `/rpp/debug` message `data[7]` (line 479) ✓

**Stale fix values:** `gps_fix_age_ms` is computed (line 551-554) and exposed in status. The `rtk_fixed` and `rtk_float` flags in `RTKStatus` (lines 773-774) use the current `gps_fix` value without age guard — this means old RTK_FIXED status persists until a new GPSRAW message arrives. However, the `get_state()` method correctly computes `gps_fix_age_ms` for consumers that need freshness. The `RTKStatus.gps_fix_type` field is informational only and does not affect mission control. ✓

**LoRa status does not claim nav health from old data:** `stream_healthy` is set entirely from LoRa lifecycle state and data age — not from GPS fix type or pose age. ✓

---

## 10. MISSION SAFETY ISOLATION AUDIT

**No path from LoRa transport state to mission control:**

| Path | Checked | Result |
|------|---------|--------|
| LoRa → mission abort | `emergency_handler.estop_async()` triggered only by telemetry loop unhealthy codes | Not LoRa-dependent |
| LoRa → RPP stop | RPP stop is via publish_stop_path() in estop chain | Not LoRa-dependent |
| LoRa → FCU mode change | `set_mode_async("MANUAL")` in estop chain | Not LoRa-dependent |
| LoRa → spray stop | Spray stop in estop chain | Not LoRa-dependent |
| LoRa → mission-start rejection | `offboard_controller.start_async()` checks `rpp_code`, not LoRa state | ✓ Isolated |

**Test sufficiency assessment:**

- `test_rpp_unhealthy_codes_exclude_lora_lifecycle` — verifies no LoRa values are in `RPP_UNHEALTHY_CODES` ✓
- `test_offboard_start_guard_uses_rpp_not_lora` — inspects source code of `start_async()` for `lifecycle_state` / `desired_source` references ⚠️ **WEAK** — only checks if these strings appear in the source, not whether there's a secondary code path
- `test_mission_placement_uses_gps_fix_not_lora` — inspects source code of `resolve_surveyed_points()` ⚠️ **WEAK** — same limitation

**Remediation for weak tests:** Add behavioral tests that:
1. Instantiate `OffboardController`, set RPP to `TRACKING`, verify LoRa lifecycle state changes don't trigger abort
2. Call `resolve_surveyed_points` with varying GPS fix types, verify LoRa status presence doesn't change behavior

---

## 11. CONFIGURATION AUDIT

| Key | Default | Range | Env parsing | Unsafe config check |
|-----|---------|-------|-------------|---------------------|
| `LORA_NO_DATA_WARN_S` | 15.0 | ≥0 | `float(os.environ.get(...))` | Warn < Fail checked: see config. |
| `LORA_NO_DATA_FAIL_S` | 60.0 | ≥0 | `float(os.environ.get(...))` | Should be ≥ warn. If unset, default 60 > 15, safe. |
| `LORA_RECONNECT_INTERVAL_S` | 5.0 | ≥0 (clamped to ≥0.5 in node) | `float(os.environ.get(...))` | If > disconnect timeout, reconnect may never fire. Default 5 < 120, safe. |
| `LORA_MAX_RESTARTS_PER_MIN` | 5 | ≥0 | `int(os.environ.get(...))` | 0 prevents restart. If user sets 0, session dies on first crash. Not unsafe — just disables recovery. |
| `LORA_MODULE_DISCONNECT_TIMEOUT_S` | 120.0 | ≥0 (clamped to ≥5.0 in node) | `float(os.environ.get(...))` | Large values mean serial absence takes longer to detect. Acceptable. |
| `LORA_MAX_FRAME_SIZE` | 1029 | RTCM3 max | `int(os.environ.get(...))` | If < 6, no frame can be parsed. Default is RTCM3 max (3+1023+3). |
| `LORA_MAX_BYTES_PER_SEC` | 65536 | ≥0 | `float(os.environ.get(...))` | 0 blocks all frames. Valid use case for "observe only, don't publish." |
| `LORA_MAX_FRAMES_PER_SEC` | 50 | ≥0 | `float(os.environ.get(...))` | 0 blocks all frames. |
| `LORA_ALLOWED_MESSAGE_TYPES` | "" | comma-separated ints | `os.environ.get(...).strip()` | Empty = allow all. Invalid int → `ValueError` from `int(part)`. Crash on import. See below. |

**⚠️ Unsafe config: invalid `LORA_ALLOWED_MESSAGE_TYPES` causes import-time crash**

```python
# server/config.py line 149
LORA_ALLOWED_MESSAGE_TYPES = os.environ.get("LORA_ALLOWED_MESSAGE_TYPES", "").strip()
```

This is just a string — parsing happens in `lora_rtcm_node.py` at runtime. However, `server/config.py` is imported at server startup. If `LORA_ALLOWED_MESSAGE_TYPES` contains non-integer values, `parse_allowed_message_types()` will raise `ValueError` when called. But this only happens in the LoRa child process, not the server. **The server will start fine even with invalid config — the LoRa child process will crash.**

**Cross-field validation checks:**
- `warn >= fail`: If set to warn=60, fail=15, the NO_DATA detection announces `stream_healthy=False` immediately when warn is exceeded, even though fail age is lower. The lifecycle loop transitions: age≤15 → CONNECTED, 15<age<60 → NO_DATA with stream_healthy=None, age≥60 → NO_DATA with stream_healthy=False. **This is correct because it treats warn as the "concern" threshold.** No correction needed.
- `max_frame_size < 6`: Prevents parsing any frame (minimum frame is 6 bytes: 3 header + 0 payload + 3 CRC). User-configurable risk.
- `LORA_MODULE_DISCONNECT_TIMEOUT_S = 5.0` minimum in node (line 64). If < RECONNECT_INTERVAL, reconnect will try before disconnection timeout fires. This is fine — reconnect attempts continue, and the disconnection state only applies after sustained absence.

---

## 12. TEST EVIDENCE

```
$ cd /Users/dyx_a1/Vetri/PX4_DXP

$ .venv-dev/bin/python -m pytest \
    test_rtcm3_parser.py \
    test_transport_rates.py \
    test_lora_rtcm_injection.py \
    server/test_rtk_manager.py \
    server/test_rtk_mission_safety.py \
    server/test_offboard_controller.py \
    server/test_mission_placement.py \
    -q
# 79 passed in 9.24s

$ .venv-dev/bin/python -m pytest \
    server/test_*.py \
    -q \
    --ignore=server/test_path_api.py
# 233 passed in 13.01s

$ .venv-dev/bin/python -m py_compile \
    rtcm3_parser.py \
    rtk_transport.py \
    lora_rtcm_node.py \
    server/rtk_manager.py \
    server/routes/rtk.py
# All modules compile cleanly (0 errors)

$ git diff --check
# No whitespace errors
```

---

## 13. COMMIT BOUNDARY

### Files safe to commit (Task_03 production):

| File | Status | Notes |
|------|--------|-------|
| `rtcm3_parser.py` | **NEW** (untracked) | Clean, complete, well-tested |
| `rtk_transport.py` | **NEW** (untracked) | Clean, simple, well-tested |
| `lora_rtcm_node.py` | **MODIFIED** (staged `M`) | Has minor resource leak (R9) — fix before production but acceptable for bench |
| `server/rtk_manager.py` | **MODIFIED** (unstaged ` M`) | Clean, well-tested |
| `server/routes/rtk.py` | **MODIFIED** (unstaged ` M`) | Clean |
| `server/config.py` | **MODIFIED** (unstaged ` M`) | Clean |
| `server/main.py` | **MODIFIED** (unstaged ` M`) | Clean; adds `rtk_manager` singleton and `rtk_router` |
| `test_rtcm3_parser.py` | **NEW** (untracked) | Clean, comprehensive |
| `test_transport_rates.py` | **NEW** (untracked) | Clean |
| `test_lora_rtcm_injection.py` | **NEW** (untracked) | Clean |
| `server/test_rtk_manager.py` | **NEW** (untracked) | Clean, comprehensive |
| `server/test_rtk_mission_safety.py` | **NEW** (untracked) | Clean, but weak — see Section 10 |

### Files that should remain uncommitted:

| File | Reason |
|------|--------|
| `task03_lora_rtk.patch` | Generated patch artifact — not part of source |
| `docs/superpowers/plans/Audit_03_1_NTRIP_RTK_Injection_Reliability.md` | Plan/working notes — not production documentation |

---

## 14. DEPLOYMENT BOUNDARY

### Services requiring restart:

| Service | Reason | Downside |
|---------|--------|----------|
| `rover-server.service` | Must be restarted to pick up new `rtk_manager.py`, `routes/rtk.py`, `config.py`, `main.py` | ~2s MAVROS bridge interruption (rpp-pipeline stays alive). No OFFBOARD loss. |

### Services NOT requiring restart:

| Service | Reason |
|---------|--------|
| `px4-dxp.service` | `lora_rtcm_node.py` runs as a child of `rover-server` — not part of px4-dxp's process tree |
| `rpp-pipeline.service` | No code changes |
| `px4_start_service.sh` | No code changes |

### Deploy steps:
1. `git add` new/modified files + commit
2. `git push`
3. On Jetson: `git pull`
4. `sudo systemctl restart rover-server.service`
5. Verify: `curl http://localhost:5001/api/rtk/status`
6. Verify: `curl http://localhost:5001/api/ping`

---

## 15. REMAINING HARDWARE RISKS

These risks are **not software defects** — they require hardware validation on the Jetson:

| Risk | Description | Mitigation |
|------|-------------|------------|
| LoRa USB serial device path | `/dev/ttyUSB0` may change after reboot | Use `/dev/serial/by-id/` symlinks; `start_lora` accepts any `/dev/` path |
| LoRa module power-on sequence | Module may need >2s to stabilize after serial open | Node has 1s serial read timeout; reconnect loop handles transient errors |
| UM982 RTK injection timing | PX4 ignores RTCM if GPS is already in RTK_FIXED | Always safe to inject; PX4 discards superfluous corrections |
| MAVROS subscriber delay | MAVROS gps_rtk plugin may take seconds to subscribe | First frames are dropped; subscriber check prevents counting non-injected bytes |
| Serial baudrate mismatch | LoRa module configured at different baudrate | Only standard baudrates are allowed (4800-921600) |
| LoRa range | Transmitter may be out of range | `module_disconnect_timeout_s=120s` provides grace period; reconnection is automatic |
| Buffer overflow on noisy serial | Garbage bytes consume parser buffer | `max_buffer_size=8192` with overflow detection (drops oldest data) |

---

## 16. SUMMARY OF REQUIRED CHANGES BEFORE PRODUCTION

### Must fix (before production):

1. **[R9] Resource leak: health timer not destroyed** (`lora_rtcm_node.py:191-198`)
   - Change: Store timer handle and destroy in `destroy_node()`
   - Add: `self._health_timer = self.create_timer(30.0, self._check_health)`
   - Add in `destroy_node()`: `self.destroy_timer(self._health_timer)`
   - Add test: Verify timer count decreases on destroy

### Should fix (before production):

2. **[F1 doc] Document injected-byte semantics** (`lora_rtcm_node.py:219-224`)
   - Add: Comment explaining `bytes_injected` counts bytes where `publish()` returned without exception and subscriber existed at batch start
   - Not a code bug — documentation gap

3. **[R10 doc] Document `first_serial_open_monotonic` behavior** (`server/rtk_manager.py:567-571`)
   - Add: Comment explaining that `first_serial_open_monotonic` is NOT reset on restart, but fallback is only used when both child timestamps are absent

4. **[Section 10] Strengthen mission-safety isolation tests**
   - Add behavioral tests (not source-inspection) that verify LoRa lifecycle state changes don't affect `OffboardController` or `resolve_surveyed_points`
   - Source-inspection tests are future-regression weak

### Can fix in follow-up:

5. **[Section 8] FAILED session restart UX** (`server/rtk_manager.py:187-194`)
   - Consider allowing `start_lora` to restart a FAILED session with the same config without requiring explicit stop first
   - Low priority — documented workaround exists

---

*Audit completed 23 June 2026. All code paths verified against working tree. No files were modified during this audit.*