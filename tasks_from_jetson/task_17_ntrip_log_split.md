# Task 17 — NTRIP Logs Split Between /tmp/ntrip.log and journald

**Priority:** LOW
**File:** `px4_start_service.sh`
**Line:** 134

---

## Problem

The NTRIP watchdog starts the node with stdout/stderr redirected to a file:

```bash
# px4_start_service.sh:134
python3 "$NTRIP_SCRIPT" >> /tmp/ntrip.log 2>&1 &
```

However, `ntrip_rtcm_node.py` now uses `self.get_logger()` for all operational output:
- `self.get_logger().info("Connected — streaming RTCM3")`
- `self.get_logger().warn("Socket timeout, reconnecting...")`
- `self.get_logger().error(f"NTRIP error: {e} — reconnect #{...}")`

ROS2 `get_logger()` output routes through the ROS2 logging infrastructure, which (inside a systemd service with `StandardOutput=journal`) sends to journald. It does **not** write to stdout in the normal sense.

So the logging is now split:
- **journald** (`journalctl -u px4-dxp.service`): all `get_logger()` output — the operational logs
- **/tmp/ntrip.log**: ROS2 pre-init output (e.g., `[INFO] [rcl]: ...`), C extension tracebacks, and any `print()` that slipped through — almost nothing useful

When debugging, a developer needs to check two places. The `/tmp/ntrip.log` accumulates silently and the log rotation in the start script only triggers at >10MB.

## Required Fix (do not apply — analysis only)

Remove the redirect — let the NTRIP process write to the same journald stream as everything else:

```bash
# ntrip_watchdog():
python3 "$NTRIP_SCRIPT" &
ntrip_pid=$!
```

All output (both `get_logger()` via ROS2 and any remaining `print()`) goes to journald under the `px4-dxp` identifier. Filter NTRIP logs with:

```bash
journalctl -u px4-dxp.service -g "ntrip_rtcm_node"
```

Also remove the log-rotation block (lines 177–180) in the start script — it becomes unnecessary.

---

**Depends on:** None
**Blocks:** Nothing
