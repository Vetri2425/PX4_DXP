# Task 08 — Redundant Competing MAVROS Readiness Polling Loops

**Priority:** MEDIUM
**File:** `px4_start_service.sh`
**Lines:** 67–79 (inside watchdog) and 135–146 (main script)

---

## Problem

Two separate loops both poll `ros2 node list` for `/mavros` simultaneously after service start:

**Loop A — inside `mavros_watchdog()` (lines 67–79):**
```bash
local ready=0
for i in {1..30}; do
    if check_ros_node "/mavros"; then
        ready=1
        break
    fi
    ...
    sleep 1
done
```

**Loop B — in main script body (lines 135–146):**
```bash
mavros_ready=0
for i in {1..35}; do
    if check_ros_node "/mavros"; then
        mavros_ready=1
        break
    fi
    ...
    sleep 1
done
```

Both call `check_ros_node()` which runs `ros2 node list 2>/dev/null | grep -q "$1"`.

## Impact

`ros2 node list` invokes the ros2 CLI daemon. Two concurrent calls compete for the daemon's response. On the Jetson Tegra, the ROS2 daemon takes 1–3s to respond on cold start. Two callers double this load, causing:

- Each loop to see slower responses → longer time-to-ready detection
- Occasional false negatives (grep sees empty output mid-query) → loop increments unnecessarily
- Spurious `"Waiting for /mavros node... (N/30)"` log messages from both loops interleaved

Loop B in the main script is entirely redundant — the watchdog already performs readiness detection. The watchdog's `log "Watchdog: MAVROS ready (PID $mavros_pid)"` signals that MAVROS is up. The main script only needs to wait for that signal, not re-check independently.

## Evidence in Codebase

`check_ros_node()` is defined at line 38:
```bash
check_ros_node() {
    ros2 node list 2>/dev/null | grep -q "$1"
}
```

It's called in both loops with `"/mavros"`. There is no synchronisation mechanism (no named pipe, no flag file, no signal) between the watchdog and the main script — they poll independently.

## Required Fix (do not apply — analysis only)

Remove Loop B from the main script. Instead, use a simple flag-file or a signal from the watchdog to the parent:

```bash
# In watchdog, after MAVROS is confirmed ready:
touch /tmp/mavros_ready

# In main script, wait for the flag instead of re-polling:
log "Waiting for MAVROS watchdog to signal ready..."
for i in {1..60}; do
    [[ -f /tmp/mavros_ready ]] && break
    sleep 1
done
[[ -f /tmp/mavros_ready ]] || { log "ERROR: MAVROS did not become ready in time"; exit 1; }
```

Clean up `/tmp/mavros_ready` in the cleanup trap.

---

**Depends on:** None
**Blocks:** Nothing
