# Task 10 — NTRIP Reconnect Interval Too Short for Boot DNS Failure

**Priority:** MEDIUM
**File:** `~/ntrip_rtcm_node.py`
**Lines:** 11, 62–84

---

## Problem

```python
# ntrip_rtcm_node.py:11
RECONNECT_SEC = 5
```

The NTRIP node starts at service launch, which happens after `network-online.target`. However, `network-online.target` only guarantees an IP address is assigned — not that DNS is resolving. On the Tegra with NetworkManager, `caster.emlid.com` DNS resolution can fail for 15–30 seconds after `network-online.target` fires, while `systemd-resolved` finishes its initial cache population.

During that window:
- `socket.connect((NTRIP_HOST, NTRIP_PORT))` raises `socket.gaierror: [Errno -3] Temporary failure in name resolution`
- The `except Exception as e` catches it, logs `[ntrip] Error: ... retry in 5s`
- After 5s, tries again — fails again
- This repeats 3–6 times before DNS is ready

The journal fills with error lines during every boot, making it harder to spot genuine runtime errors later. More importantly, RTCM corrections don't start flowing until ~30 seconds after boot instead of immediately.

## Evidence in Codebase

The service `After=network-online.target` (service line 5) only guarantees network availability, not DNS. The `socket.settimeout(10)` in `_connect()` (line 29) is for the connection timeout, not DNS resolution — DNS resolution happens in `socket.connect()` before the timeout applies.

## Required Fix (do not apply — analysis only)

Implement exponential backoff with a cap:

```python
RECONNECT_MIN_SEC = 5
RECONNECT_MAX_SEC = 60

# In _run():
retry_delay = RECONNECT_MIN_SEC
while rclpy.ok() and not self._stop.is_set():
    try:
        sock, buf = self._connect()
        retry_delay = RECONNECT_MIN_SEC  # reset on successful connect
        while rclpy.ok() and not self._stop.is_set():
            # ... normal stream loop
    except socket.gaierror as e:
        self.get_logger().warning(f"DNS failure: {e} — retry in {retry_delay}s")
        time.sleep(retry_delay)
        retry_delay = min(retry_delay * 2, RECONNECT_MAX_SEC)
    except Exception as e:
        self.get_logger().error(f"NTRIP error: {e} — retry in {retry_delay}s")
        time.sleep(retry_delay)
        retry_delay = min(retry_delay * 2, RECONNECT_MAX_SEC)
```

DNS failures back off to 60s max. Stream errors (which are runtime failures, not boot transients) reset the delay on reconnect.

Also note: switching from `print()` to `self.get_logger()` (shown above) is the correct ROS2 pattern — it routes through the ROS2 logging infrastructure and appears in `ros2 topic echo /rosout`.

---

**Depends on:** Task 09 (adds `self._stop` event needed for the loop condition)
**Blocks:** Nothing
