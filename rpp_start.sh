#!/bin/bash
# RPP Pipeline startup — runs the four always-on controller nodes.
#
# Nodes started:
#   1. twist_to_setpoint_node.py  — 50 Hz OFFBOARD heartbeat (must start first)
#   2. rpp_controller_node.py     — Regulated Pure Pursuit path follower
#   3. spray_controller_node.py   — MARK actuator via MAV_CMD_DO_SET_ACTUATOR
#   4. xtrack_logger_node.py      — CSV telemetry capture for tuning
#
# NOT started here (server-driven):
#   - path_publisher_node.py      — server publishes /path directly
#   - mission_runner_node.py      — server owns OFFBOARD lifecycle
#
# Watchdog: if any node dies, it is restarted. If all nodes die within
# FAIL_WINDOW seconds, the script exits (systemd restarts the whole service).

set -euo pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC_DIR="${SCRIPT_DIR}/src"

# Timing
NODE_RESTART_DELAY=2
FAIL_WINDOW=30
MAX_FAILS_IN_WINDOW=5

log() { echo "[rpp_pipeline] $(date '+%H:%M:%S') $*"; }

# ── Source ROS2 ───────────────────────────────────────────────────────────────
if [[ ! -f "$ROS_SETUP" ]]; then
    log "ERROR: ROS2 setup not found at $ROS_SETUP"
    exit 1
fi
set +u; source "$ROS_SETUP"; set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# ── Cleanup ───────────────────────────────────────────────────────────────────
declare -A NODE_PIDS=()
FAIL_TIMES=()

SHUTTING_DOWN=0

cleanup() {
    SHUTTING_DOWN=1
    log "Shutting down RPP pipeline..."
    # Phase 1: polite SIGTERM to every node.
    for name in "${!NODE_PIDS[@]}"; do
        local pid="${NODE_PIDS[$name]:-}"
        [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null || true
    done
    # Phase 2: brief grace, then force-kill any straggler. We do NOT `wait`
    # on the nodes: an rclpy node blocked in a spin C-call may not honour
    # SIGTERM promptly, and waiting on it is what made systemd hit
    # TimeoutStopSec and SIGKILL the whole service after 15 s (Result:
    # timeout). These are stateless setpoint/logger nodes — a hard kill on
    # shutdown is safe and the next start resumes the zero-velocity heartbeat.
    sleep 0.5
    for name in "${!NODE_PIDS[@]}"; do
        local pid="${NODE_PIDS[$name]:-}"
        [[ -n "$pid" ]] && kill -KILL "$pid" 2>/dev/null || true
    done
}

# handle_exit runs cleanup and then EXITS. The previous code trapped `cleanup`
# directly with no exit, so on SIGTERM the trap cleaned up but execution
# resumed in the watchdog loop below, which promptly restarted the nodes —
# the script never terminated and systemd killed it on timeout. Exiting here
# is what makes `systemctl stop/restart rpp-pipeline` fast (sub-second).
handle_exit() {
    local code="$1"
    trap - EXIT INT TERM
    cleanup
    exit "$code"
}

trap 'handle_exit 143' TERM
trap 'handle_exit 130' INT
trap 'handle_exit $?' EXIT

# ── Node launcher ─────────────────────────────────────────────────────────────
start_node() {
    local name="$1"
    local script="$2"
    log "Starting $name..."
    python3 "$script" &
    NODE_PIDS["$name"]=$!
    log "$name started (PID ${NODE_PIDS[$name]})"
}

record_fail() {
    local now
    now=$(date +%s)
    FAIL_TIMES+=("$now")
    # Trim old entries outside the window
    local cutoff=$((now - FAIL_WINDOW))
    local new_times=()
    for t in "${FAIL_TIMES[@]}"; do
        if [[ "$t" -ge "$cutoff" ]]; then
            new_times+=("$t")
        fi
    done
    FAIL_TIMES=("${new_times[@]}")
    if [[ ${#FAIL_TIMES[@]} -ge $MAX_FAILS_IN_WINDOW ]]; then
        log "ERROR: $MAX_FAILS_IN_WINDOW failures in ${FAIL_WINDOW}s — giving up (systemd will restart)"
        exit 1
    fi
}

# ── Kill stale instances ──────────────────────────────────────────────────────
pkill -f "twist_to_setpoint_node" 2>/dev/null || true
pkill -f "rpp_controller_node" 2>/dev/null || true
pkill -f "spray_controller_node" 2>/dev/null || true
pkill -f "xtrack_logger_node" 2>/dev/null || true
sleep 1

# ── Start nodes in order ──────────────────────────────────────────────────────
log "====================================================="
log " RPP Pipeline Starting"
log " Nodes: twist_to_setpoint, rpp_controller, spray_controller, xtrack_logger"
log "====================================================="

start_node "twist_to_setpoint" "${SRC_DIR}/twist_to_setpoint_node.py"
start_node "rpp_controller" "${SRC_DIR}/rpp_controller_node.py"
start_node "spray_controller" "${SRC_DIR}/spray_controller_node.py"
start_node "xtrack_logger" "${SRC_DIR}/xtrack_logger_node.py"

log "All RPP nodes started. Entering watchdog loop..."

# ── Watchdog loop ─────────────────────────────────────────────────────────────
while true; do
    sleep 2
    # If a shutdown signal arrived during the sleep, stop — never resurrect
    # nodes that cleanup() is tearing down.
    [[ "$SHUTTING_DOWN" -eq 1 ]] && break
    for name in "twist_to_setpoint" "rpp_controller" "spray_controller" "xtrack_logger"; do
        local_pid="${NODE_PIDS[$name]:-}"
        if [[ -z "$local_pid" ]] || ! kill -0 "$local_pid" 2>/dev/null; then
            log "WARNING: $name died — restarting in ${NODE_RESTART_DELAY}s..."
            record_fail
            sleep "$NODE_RESTART_DELAY"
            case "$name" in
                twist_to_setpoint) start_node "$name" "${SRC_DIR}/twist_to_setpoint_node.py" ;;
                rpp_controller)    start_node "$name" "${SRC_DIR}/rpp_controller_node.py" ;;
                spray_controller)  start_node "$name" "${SRC_DIR}/spray_controller_node.py" ;;
                xtrack_logger)     start_node "$name" "${SRC_DIR}/xtrack_logger_node.py" ;;
            esac
        fi
    done
done
