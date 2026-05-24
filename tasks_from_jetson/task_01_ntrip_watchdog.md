# Task 01 — NTRIP Node Has No Watchdog / Restart Loop

**Priority:** CRITICAL
**File:** `px4_start_service.sh`
**Lines:** 155–157

---

## Problem

The NTRIP RTK injector is started with `nohup` and its PID is tracked in `CHILD_PIDS`, but only for cleanup on service exit. There is no watchdog, no restart loop, no health check.

```bash
# px4_start_service.sh:155-157
nohup python3 /home/flash/ntrip_rtcm_node.py >> /tmp/ntrip.log 2>&1 &
CHILD_PIDS+=($!)
sleep 2
```

If the NTRIP node crashes (caster drops connection, DNS error, Python exception, network hiccup), it dies silently. The bridge keeps running, MAVROS keeps running, everything looks alive — but RTK corrections stop flowing. PX4 degrades from RTK-Fix to single-point GPS without any alert.

## Why It Matters

RTK accuracy is the entire point of this rover. A silent RTK loss during a marking run means the marks are placed incorrectly with no indication that anything went wrong. This has already happened at least once (hence the NTRIP reconnect logic inside the Python node itself). But if the Python process itself dies, the internal reconnect loop is gone too.

Compare this to MAVROS — it has a full watchdog function (lines 50–95 of the same script) with:
- Auto-restart on exit
- 30-iteration readiness check per restart
- Logged restart events

The NTRIP node has none of this.

## Evidence in Codebase

`ntrip_rtcm_node.py` can exit on:
- `ConnectionError` raised in `_connect()` — caught by the thread's `except Exception`, thread sleeps and retries. But this is the *thread*, not the *process*. If `rclpy.spin()` is interrupted for any reason, `main()` falls through, `rclpy.shutdown()` is called, and the process exits — killing the thread too.
- Any unhandled exception outside the `try/except` in `_run()`.
- Signals (SIGTERM from the shell cleanup, stray SIGPIPE from a broken socket).

## Required Fix (do not apply — analysis only)

Mirror the `mavros_watchdog()` pattern for NTRIP:

```bash
ntrip_watchdog() {
    local ntrip_pid=""
    _ntrip_wd_cleanup() {
        [[ -n "$ntrip_pid" ]] && kill "$ntrip_pid" 2>/dev/null || true
        exit 0
    }
    trap '_ntrip_wd_cleanup' TERM INT

    while true; do
        log "NTRIP watchdog: starting ntrip_rtcm_node..."
        python3 /home/flash/ntrip_rtcm_node.py >> /tmp/ntrip.log 2>&1 &
        ntrip_pid=$!
        wait "$ntrip_pid" 2>/dev/null || true
        log "NTRIP watchdog: node exited — restarting in 10s..."
        ntrip_pid=""
        sleep 10
    done
}
```

Start it after MAVROS is confirmed ready, add its PID to `CHILD_PIDS`.

---

**Depends on:** None
**Blocks:** Task 12 (boot DNS backoff should be implemented before the watchdog restart interval is finalised)
