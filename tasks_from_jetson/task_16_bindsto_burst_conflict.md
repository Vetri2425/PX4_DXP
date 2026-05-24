# Task 16 — BindsTo=dev-ttyACM0.device + StartLimitBurst Race

**Priority:** HIGH
**File:** `px4-dxp.service`
**Lines:** 3–5, 17–18

---

## Problem

The new service file adds:

```ini
After=network.target network-online.target dev-ttyACM0.device
BindsTo=dev-ttyACM0.device
```

`BindsTo=dev-ttyACM0.device` makes the service lifecycle tightly coupled to the USB device unit. When the CubeOrangePlus USB device disappears (even briefly), systemd **stops** the service. When it reappears, systemd **starts** it again.

This is layered on top of an unchanged burst limit:

```ini
StartLimitInterval=120
StartLimitBurst=5
```

## The Conflict

The MAVROS watchdog inside `px4_start_service.sh` already handles FCU reconnect gracefully — it detects MAVROS exit, sleeps briefly, and restarts MAVROS. The watchdog is designed for exactly this scenario.

`BindsTo` adds a *second*, redundant restart mechanism that operates at the systemd level rather than the script level:

- **FCU reboots for param save** (takes 5–15s): USB disappears → systemd stops service → USB reappears → systemd starts service. Each cycle = one burst count.
- **Bouncy USB connection**: Same scenario, faster. Three or four bounces in 120 seconds → burst limit exhausted → service enters `failed` state, stops retrying.

The internal watchdog would have handled all of these without consuming burst count or triggering service restarts.

## Why BindsTo Was Added

Likely to ensure the service doesn't start without the FCU present. This is already handled by:
```bash
# px4_start_service.sh:148-151
if [[ ! -c "$FCU_DEVICE" ]]; then
    log "ERROR: $FCU_DEVICE not found — is CubeOrangePlus plugged in via USB?"
    exit 1
fi
```

So the explicit fast-fail in the script makes `BindsTo` redundant for startup. And for runtime, `BindsTo` is actively harmful because it bypasses the watchdog's graceful restart logic.

## Required Fix (do not apply — analysis only)

**Option A (recommended):** Remove `BindsTo` and keep the script's fast-fail. Leave `After=dev-ttyACM0.device` if you want ordering (waits for device before starting), but without `BindsTo`.

```ini
After=network.target network-online.target dev-ttyACM0.device
Wants=network-online.target
# Remove: BindsTo=dev-ttyACM0.device
```

**Option B:** Keep `BindsTo` but also raise the burst limit significantly (Task 06) to absorb the additional restart pressure:

```ini
StartLimitInterval=300
StartLimitBurst=20
```

Option A is cleaner — the watchdog is the right layer for handling FCU reconnect.

---

**Depends on:** None
**Blocks:** Nothing, but must be evaluated before Task 06 (burst limit) — the right fix for 06 depends on whether BindsTo stays or goes
