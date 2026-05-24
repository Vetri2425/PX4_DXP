# Task 06 — StartLimitBurst Too Tight for USB Reconnect Scenario

**Priority:** MEDIUM
**File:** `/etc/systemd/system/px4-dxp.service`
**Lines:** 19–21

---

## Problem

```ini
# px4-dxp.service:19-21
Restart=on-failure
RestartSec=10
StartLimitInterval=120
StartLimitBurst=5
```

This allows 5 restart attempts within any 120-second window. If the service fails more than 5 times in 120 seconds, systemd enters `failed` state and stops retrying. **Manual `systemctl reset-failed && systemctl start px4-dxp` is required to recover.**

## When This Triggers

The most common scenario is CubeOrangePlus power cycling or PX4 rebooting during parameter save:

1. PX4 reboots → `/dev/ttyACM0` disappears
2. `px4_start_service.sh` exits (FCU device check at line 102 fails fast)
3. systemd restarts after 10s — `/dev/ttyACM0` may still not be back (USB re-enumeration takes 5–15s on Tegra)
4. Script exits again immediately
5. Repeat 3–4 more times in rapid succession → burst limit hit → service enters `failed`

The rover is now headless — no MAVROS, no QGC bridge, no RTK — and no one knows unless they SSH in.

A secondary scenario: any crash that happens before `mavros_watchdog` fully starts (e.g., ROS2 setup.bash not found) will also cause rapid repeated failures.

## Required Fix (do not apply — analysis only)

Increase the burst window to absorb USB flap:

```ini
RestartSec=15
StartLimitInterval=300
StartLimitBurst=10
```

This allows 10 restarts in 5 minutes (one every 30s on average) before giving up. Given that each restart already has a 15s delay and MAVROS takes 30–60s to start, 10 bursts covers a 5–10 minute hardware recovery window without ever triggering falsely on a genuine crash loop.

Optionally add `OnFailure=` to notify via a simple wall message or log:
```ini
OnFailure=px4-dxp-failure-notify.service
```

---

**Depends on:** None
**Blocks:** Nothing, but important for unattended field operation
