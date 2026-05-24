# Task 12 — set -euo pipefail Undermined by Blanket || true Suppression

**Priority:** LOW
**File:** `px4_start_service.sh`
**Lines:** 8, 23–28, 116–119

---

## Problem

The script declares strict error handling at the top:

```bash
# px4_start_service.sh:8
set -euo pipefail
```

But then suppresses errors on nearly every command in cleanup and setup paths:

```bash
# px4_start_service.sh:23-28 (cleanup function)
cleanup() {
    for pid in "${CHILD_PIDS[@]:-}"; do
        if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true      # ← suppressed
            wait "$pid" 2>/dev/null || true      # ← suppressed
        fi
    done
}

# px4_start_service.sh:115-119
ros2 daemon stop >/dev/null 2>&1 || true         # ← suppressed
pkill -f "mavros px4.launch" 2>/dev/null || true # ← suppressed (also wrong pattern — see Task 04)
sleep 1
```

## Why This Is a Problem

`set -e` + `|| true` on every cleanup command means:
1. `set -e` only protects the critical path (FCU device check, source setup.bash)
2. All pre-flight cleanup and post-flight teardown are effectively running with `set +e`
3. A silent failure in cleanup (e.g., `lsof` not installed, `pkill` binary missing) will not be caught or logged

More concretely: if `ros2 daemon stop` hangs (it can, on a busy system), it blocks the entire startup sequence. With `2>/dev/null || true`, there's no log entry, no timeout — it just hangs silently.

## Required Fix (do not apply — analysis only)

Bracket cleanup/prep sections explicitly:

```bash
# For genuinely optional operations, use set +e locally:
set +e
ros2 daemon stop 2>/dev/null
pkill -f "mavros node.launch" 2>/dev/null
pkill -f "mavros_node" 2>/dev/null
set -e

# For operations with a real timeout concern, add timeout:
timeout 5 ros2 daemon stop 2>/dev/null || log "WARNING: ros2 daemon stop timed out"
```

This preserves the intent of `set -e` for the critical path while being explicit about where errors are intentionally ignored.

---

**Depends on:** None
**Blocks:** Nothing (code quality, not runtime correctness)
