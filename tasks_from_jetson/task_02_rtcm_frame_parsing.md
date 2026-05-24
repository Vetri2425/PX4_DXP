# Task 02 — RTCM Frame Boundaries Not Respected

**Priority:** CRITICAL
**File:** `~/ntrip_rtcm_node.py`
**Lines:** 66–80

---

## Problem

The node publishes the raw bytes from each `sock.recv()` call directly as a single MAVROS RTCM message, without parsing RTCM3 frame boundaries:

```python
# ntrip_rtcm_node.py:66-80
while rclpy.ok():
    try:
        chunk = sock.recv(4096)
    except socket.timeout:
        ...
    if not chunk:
        ...
    buf += chunk
    msg = RTCM()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.data = bytes(buf)
    self.pub.publish(msg)
    buf = b""
```

RTCM3 is a framed binary protocol. Each frame is structured as:

```
0xD3 | 6-bit reserved + 10-bit length | payload | 24-bit CRC
```

A single `recv(4096)` call can return:
- **Multiple complete frames** — MAVROS injects only the first; the rest are silently dropped
- **A partial frame** (TCP can fragment at any byte) — MAVROS receives a malformed message and discards it
- **Leftover bytes from a previous frame concatenated with the start of a new one** — corrupt injection

The current code handles none of these cases. Every publish is a gamble on whether TCP happened to deliver exactly one frame at a time.

## Why It Matters

RTCM MSM4/MSM7 messages (the type Emlid casters send) arrive at 1 Hz for each constellation. At 921600 baud with PX4's RTCM injection, missing or malformed corrections directly cause:
- RTK float instead of RTK fix
- Fix-to-float cycling under marginal conditions
- Increased time-to-first-fix after reconnect

This is the most likely root cause of intermittent RTK degradation during operation, even when the NTRIP connection is healthy and the log shows no errors.

## Evidence in Codebase

The `leftover` variable in `_connect()` (lines 42–50) shows awareness of partial reads during the HTTP handshake — the developer knows TCP can fragment. But the same logic is not applied to the RTCM stream itself.

```python
# _connect() correctly handles handshake leftover:
if b"\r\n\r\n" in resp:
    header, leftover = resp.split(b"\r\n\r\n", 1)
    ...
return s, leftover   # passed back as initial buf
```

The leftover handling is correct. The frame parsing for the stream is what's missing.

## Required Fix (do not apply — analysis only)

Parse the stream into complete RTCM3 frames before publishing. Each frame starts with `0xD3`:

```python
def _extract_frames(buf: bytes) -> tuple[list[bytes], bytes]:
    """Split buf into complete RTCM3 frames + unconsumed remainder."""
    frames = []
    while len(buf) >= 3:
        if buf[0] != 0xD3:
            # Lost sync — scan forward for next 0xD3
            idx = buf.find(b'\xd3', 1)
            if idx == -1:
                buf = b""
                break
            buf = buf[idx:]
            continue
        # 10-bit length in bytes 1-2 (bits 13-22 of the 3-byte header)
        length = ((buf[1] & 0x03) << 8) | buf[2]
        frame_size = 3 + length + 3  # header + payload + CRC
        if len(buf) < frame_size:
            break  # incomplete frame — wait for more data
        frames.append(buf[:frame_size])
        buf = buf[frame_size:]
    return frames, buf

# In _run():
while rclpy.ok():
    chunk = sock.recv(4096)
    ...
    buf += chunk
    frames, buf = _extract_frames(buf)
    for frame in frames:
        msg = RTCM()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = bytes(frame)
        self.pub.publish(msg)
```

Note: CRC validation can be added later. Frame boundary parsing alone is the critical fix.

---

**Depends on:** None
**Blocks:** Nothing, but fix this before field testing Phase 2 arc runs
