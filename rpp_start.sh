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
# Watchdog: if any node dies, it is restarted.
#
# CRITICAL vs AUXILIARY (Spray Controller V2 plan, §9 — infra failure
# isolation, do first): twist_to_setpoint + rpp_controller are the
# OFFBOARD-critical nodes — losing them for long is unsafe, so their
# deaths still count toward the global FAIL_WINDOW/MAX_FAILS_IN_WINDOW
# threshold and can trip a full-pipeline restart (systemd restarts the
# whole service). spray_controller + xtrack_logger are auxiliary: they
# get their own isolated per-node backoff restart and NEVER count toward
# the global threshold or trigger a full-pipeline exit. See
# is_critical_node()/aux backoff below for the rationale (2026-06-25
# spray crash-loop incident).

set -euo pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC_DIR="${SCRIPT_DIR}/src"

# Timing
NODE_RESTART_DELAY=2
FAIL_WINDOW=30
MAX_FAILS_IN_WINDOW=5

# Auxiliary-node isolated backoff (§9): starts at 2s, doubles per
# consecutive death, capped at 30s. Resets back to 2s once a node has
# stayed up at least AUX_RESET_AFTER_S since its last (re)start.
AUX_BACKOFF_CAP_S=30
AUX_RESET_AFTER_S=60

log() { echo "[rpp_pipeline] $(date '+%H:%M:%S') $*"; }

# mono_s: MONOTONIC seconds since boot (from /proc/uptime). Used for all
# watchdog/backoff timing so an RTK/NTP wall-clock STEP (routine on this
# rover when GPS time sync lands) can't stall or misfire a restart. `date
# +%s` (wall clock) would jump backward/forward on such a step and either
# freeze an auxiliary node's scheduled restart or collapse its backoff.
mono_s() { local u _; read -r u _ < /proc/uptime; echo "${u%.*}"; }

# ── Source ROS2 ───────────────────────────────────────────────────────────────
if [[ ! -f "$ROS_SETUP" ]]; then
    log "ERROR: ROS2 setup not found at $ROS_SETUP"
    exit 1
fi
set +u; source "$ROS_SETUP"; set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# ── Cleanup ───────────────────────────────────────────────────────────────────
declare -A NODE_PIDS=()
declare -A NODE_START_TIME=()
declare -A AUX_FAIL_COUNT=()
# Scheduled earliest-restart epoch for a dead auxiliary node (0/unset = not
# scheduled). The watchdog polls this each 2s iteration instead of blocking
# on a long sleep — see the auxiliary branch for why that matters.
declare -A AUX_NEXT_RESTART=()
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
    NODE_START_TIME["$name"]=$(mono_s)
    log "$name started (PID ${NODE_PIDS[$name]})"
}

# ── Node classification: critical vs auxiliary (§9, Spray Controller V2 plan) ─
# Postmortem (2026-06-25, memory: spray_continuous_pipeline_crash): a
# continuous-mode spray_controller crash-loop (inf distance-to-boundary
# serialization bug) tripped the GLOBAL record_fail/MAX_FAILS_IN_WINDOW
# threshold shared with twist_to_setpoint/rpp_controller, so the whole
# pipeline gave up and systemd restarted it — an ~11s OFFBOARD blackout
# that produced a 1.5m corner drift on a real run. The crash was in an
# auxiliary node; it should never have been able to tear down the
# OFFBOARD-critical nodes.
#
# CRITICAL: twist_to_setpoint (OFFBOARD heartbeat), rpp_controller (path
# tracking). Losing either for long is unsafe — deaths still call
# record_fail and can trip the full-pipeline exit 1 (systemd restart),
# unchanged from prior behavior.
#
# AUXILIARY: spray_controller, xtrack_logger. Neither is in the OFFBOARD
# setpoint loop. While spray_controller is down the actuator fails safe
# OFF by construction: MAVROS simply stops receiving
# MAV_CMD_DO_SET_ACTUATOR commands, and PX4 does not hold a stale ON
# state absent a repeated command — so isolating its failures from the
# critical watchdog is safe. Auxiliary deaths NEVER call record_fail and
# NEVER trip exit 1; they respawn on their own per-node exponential
# backoff (2s -> AUX_BACKOFF_CAP_S), tracked in AUX_FAIL_COUNT.
is_critical_node() {
    case "$1" in
        twist_to_setpoint|rpp_controller) return 0 ;;
        *) return 1 ;;
    esac
}

# aux_backoff_delay: consecutive-fail count -> capped exponential delay.
# count=1 -> 2s, 2 -> 4s, 3 -> 8s, 4 -> 16s, 5+ -> capped at
# AUX_BACKOFF_CAP_S (30s).
aux_backoff_delay() {
    local count="$1"
    local delay=$(( 2 ** count ))
    if (( delay > AUX_BACKOFF_CAP_S )); then
        delay=$AUX_BACKOFF_CAP_S
    fi
    echo "$delay"
}

record_fail() {
    local now
    now=$(mono_s)
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
            if is_critical_node "$name"; then
                # Critical path: unchanged from prior behavior — counts
                # toward the global failure window and can give up the
                # whole pipeline (record_fail may exit 1).
                log "WARNING: $name (critical) died — restarting in ${NODE_RESTART_DELAY}s..."
                record_fail
                sleep "$NODE_RESTART_DELAY"
                case "$name" in
                    twist_to_setpoint) start_node "$name" "${SRC_DIR}/twist_to_setpoint_node.py" ;;
                    rpp_controller)    start_node "$name" "${SRC_DIR}/rpp_controller_node.py" ;;
                esac
            else
                # Auxiliary path: isolated, NON-BLOCKING per-node backoff.
                # Never calls record_fail, never counts toward
                # MAX_FAILS_IN_WINDOW, never trips exit 1 — a crash-looping
                # spray_controller/xtrack_logger cannot take down OFFBOARD.
                #
                # It must NOT `sleep` for the backoff here. A multi-second
                # blocking sleep in this single-threaded loop would (a) delay
                # detection+restart of a CRITICAL node that dies during the
                # window (twist_to_setpoint's heartbeat gap → PX4 failsafe),
                # and (b) if `systemctl stop/restart` arrives mid-sleep and
                # the sleep exceeds TimeoutStopSec, systemd SIGKILLs the whole
                # cgroup before handle_exit/cleanup can run — the exact
                # failure the trap machinery exists to prevent. Instead we
                # schedule a restart epoch and let the 2s loop poll it, so no
                # sleep in this script ever exceeds 2s.
                now_ts=$(mono_s)
                next_ts="${AUX_NEXT_RESTART[$name]:-0}"
                if (( next_ts == 0 )); then
                    # Fresh death: escalate the backoff and schedule; the
                    # restart itself happens on a later iteration once the
                    # scheduled epoch passes (matching the old "wait, then
                    # restart" ordering, minus the blocking sleep).
                    started_ts="${NODE_START_TIME[$name]:-$now_ts}"
                    if (( now_ts - started_ts >= AUX_RESET_AFTER_S )); then
                        AUX_FAIL_COUNT["$name"]=0
                    fi
                    AUX_FAIL_COUNT["$name"]=$(( ${AUX_FAIL_COUNT[$name]:-0} + 1 ))
                    # Clamp the counter itself (not just the delay) so
                    # 2**count can't grow unbounded across a long crash-loop
                    # that never earns a reset.
                    if (( AUX_FAIL_COUNT[$name] > 10 )); then
                        AUX_FAIL_COUNT["$name"]=10
                    fi
                    aux_delay=$(aux_backoff_delay "${AUX_FAIL_COUNT[$name]}")
                    AUX_NEXT_RESTART["$name"]=$(( now_ts + aux_delay ))
                    log "WARNING: $name (auxiliary) died — isolated restart scheduled in ${aux_delay}s (consecutive fail #${AUX_FAIL_COUNT[$name]}, not counted toward pipeline MAX_FAILS_IN_WINDOW)"
                elif (( now_ts >= next_ts )); then
                    # Backoff elapsed: restart now, clear the schedule so the
                    # next death is treated as fresh.
                    AUX_NEXT_RESTART["$name"]=0
                    case "$name" in
                        spray_controller) start_node "$name" "${SRC_DIR}/spray_controller_node.py" ;;
                        xtrack_logger)    start_node "$name" "${SRC_DIR}/xtrack_logger_node.py" ;;
                    esac
                fi
                # else: still within the backoff window — do nothing, the
                # next 2s iteration re-checks. No blocking sleep.
            fi
        fi
    done
done
