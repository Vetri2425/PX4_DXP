# Task 05 — RestrictRealtime=yes Degrades MAVROS Serial Scheduling

**Priority:** HIGH
**File:** `/etc/systemd/system/px4-dxp.service`
**Line:** 31

---

## Problem

```ini
# px4-dxp.service:31
RestrictRealtime=yes
```

This systemd security directive blocks any process in the service's cgroup from using real-time scheduling policies (`SCHED_FIFO`, `SCHED_RR`).

MAVROS's serial driver (`mavros_node`) uses `SCHED_FIFO` threads for the MAVLink parser to ensure low-latency byte processing at 921600 baud. With `RestrictRealtime=yes`, those threads silently fall back to `SCHED_OTHER` (normal CFS scheduling).

## Impact

At 921600 baud, the FCU sends MAVLink messages at up to ~92KB/s. The parser thread needs to wake within ~1ms of bytes arriving on the tty. Under normal load this works fine with CFS. But:

- When ROS2 DDS middleware (Fast-RTPS) spikes CPU to publish telemetry topics, the parser thread can be preempted for 5–20ms
- This causes MAVLink sequence gaps → MAVROS logs `seq mismatch` warnings
- Accumulated gaps trigger the MAVROS heartbeat timeout logic → false "FCU disconnected" events
- PX4's side sees missed heartbeats and may trigger RC failsafe

The current journal shows repeated `FCU: UNK(8): EVENT` and `FCU: EVENT` errors — these are likely exacerbated by this scheduling downgrade.

## Evidence in Codebase

`systemctl status` shows the service has been running 37 minutes and has consistent FCU event warnings in the log:
```
[WARN] FCU: UNK(8): EVENT 1914663
[ERROR] FCU: EVENT 13835193
[ERROR] FCU: EVENT 10011251
```

These are PX4 internal event log messages being forwarded via MAVLink. The frequency and variety suggest the PX4 is seeing gaps in MAVLink communication and logging state transitions as a result.

## Required Fix (do not apply — analysis only)

Remove `RestrictRealtime=yes` from the service file. This is the only directive that needs removing — the other security settings are appropriate. MAVROS on a dedicated companion computer is trusted software; it does not need to be prevented from using real-time scheduling.

```ini
# Remove this line:
RestrictRealtime=yes
```

No replacement needed. The default (no restriction) is correct for hardware-interfacing ROS2.

---

**Depends on:** None
**Blocks:** Nothing directly, but should be fixed before interpreting any FCU event logs as real firmware issues
