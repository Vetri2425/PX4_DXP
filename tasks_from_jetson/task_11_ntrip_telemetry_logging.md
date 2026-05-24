# Task 11 — No RTCM Message Rate or Byte Counter Logging

**Priority:** MEDIUM
**File:** `~/ntrip_rtcm_node.py`

---

## Problem

The NTRIP node has no observability beyond error messages. There is no way to know from the journal whether:
- RTCM corrections are flowing at the expected rate
- Which RTCM message types are being received (1074, 1084, 1094, etc.)
- How many frames per interval (should be ~1 Hz per constellation)
- Total bytes transferred (confirms stream is active, not just connected)

In field operation, RTK degradation from "Fix" to "Float" is the symptom. The cause could be:
- NTRIP caster connected but sending 0 bytes (keepalive stall)
- RTCM messages arriving but frame parsing dropping them (Task 02)
- MAVROS not injecting (separate issue)
- PX4 rejecting corrections (PX4-side issue)

Without a frame counter, it is impossible to tell which layer failed from the Jetson side.

## Current Logging State

```python
# Only these print statements exist:
print(f"[ntrip] Starting: {NTRIP_HOST}:{NTRIP_PORT}/{NTRIP_MOUNTPT}", flush=True)
print(f"[ntrip] Connected OK — streaming RTCM3", flush=True)
print("[ntrip] Socket timeout, reconnecting...", flush=True)
print("[ntrip] Stream ended, reconnecting...", flush=True)
print(f"[ntrip] Error: {e} — retry in {RECONNECT_SEC}s", flush=True)
```

No periodic status. No counters. No message type identification.

## Required Fix (do not apply — analysis only)

After fixing frame parsing (Task 02), add a periodic stats log:

```python
# Add to NtripNode.__init__:
self._frame_count = 0
self._byte_count = 0
self._last_stats_time = time.time()
STATS_INTERVAL = 60  # seconds

# After publishing each frame in _run():
self._frame_count += 1
self._byte_count += len(frame)

now = time.time()
if now - self._last_stats_time >= STATS_INTERVAL:
    self.get_logger().info(
        f"RTCM stats: {self._frame_count} frames, "
        f"{self._byte_count} bytes in last {STATS_INTERVAL}s"
    )
    self._frame_count = 0
    self._byte_count = 0
    self._last_stats_time = now
```

Expected healthy output: `RTCM stats: 180 frames, 45000 bytes in last 60s` (3 constellations × 1 Hz × 60s ≈ 180 frames).

If you see `0 frames` with no errors, the caster is connected but sending nothing — a caster-side issue.
If you see frames but no RTK fix, the problem is downstream of this node (MAVROS injection or PX4 parameter).

---

**Depends on:** Task 02 (frame parsing — stats are per-frame, not per recv())
**Blocks:** Nothing, but critical for field diagnostics
