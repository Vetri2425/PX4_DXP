#!/bin/bash

# PX4 MAVROS bridge — CubeOrangePlus via /dev/ttyACM0
# QGC connects via directed UDP (no telemetry radio needed):
#   In QGC: Comm Links → Add → UDP → Port 14550 → connect
#   MAVROS sends directed packets to LAPTOP_IP, not broadcast.

set -euo pipefail

FCU_DEVICE="/dev/ttyACM0"
FCU_BAUD="921600"
GCS_UDP_PORT="14550"
LAPTOP_IP="192.168.1.103"
JETSON_IP="192.168.1.102"
ROS_SETUP="/opt/ros/humble/setup.bash"

# Timing constants
# MAVROS_READY_TIMEOUT is the main-body wait for the flag file;
# it must be > MAVROS_READY_WAIT (the watchdog's per-start attempt limit)
# so the main body always gives the watchdog at least one full attempt.
MAVROS_READY_TIMEOUT=35
MAVROS_READY_WAIT=30
NTRIP_READY_WAIT=2
MAVROS_RESTART_DELAY=3
NTRIP_RESTART_DELAY=3
MAVROS_FAIL_DELAY=5

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NTRIP_SCRIPT="${SCRIPT_DIR}/ntrip_rtcm_node.py"
MAVROS_READY_FLAG="/tmp/px4_mavros_ready"

declare -a CHILD_PIDS=()
MAVROS_WATCHDOG_PID=""
NTRIP_WATCHDOG_PID=""

log() { echo "[px4_service] $(date '+%H:%M:%S') $*"; }

cleanup() {
    log "Cleaning up child processes..."
    rm -f "$MAVROS_READY_FLAG"
    for pid in "${CHILD_PIDS[@]:-}"; do
        if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
}

handle_exit() {
    local code=$1
    trap - EXIT INT TERM
    cleanup
    exit "$code"
}

check_ros_node() {
    # timeout 5: prevents an indefinitely-hung ROS2 daemon from blocking
    # the watchdog restart loop for up to MAVROS_READY_WAIT × ∞ seconds.
    timeout 5 ros2 node list 2>/dev/null | grep -q "$1"
}

free_port() {
    local port=$1
    if lsof -i ":$port" >/dev/null 2>&1; then
        log "Port $port busy — freeing gracefully..."
        lsof -ti ":$port" | xargs -r kill -TERM 2>/dev/null || true
        sleep 2
        if lsof -i ":$port" >/dev/null 2>&1; then
            log "Port $port still busy — force killing..."
            lsof -ti ":$port" | xargs -r kill -9 2>/dev/null || true
        fi
    fi
}

mavros_watchdog() {
    local mavros_pid=""

    _wd_cleanup() {
        [[ -n "$mavros_pid" ]] && kill "$mavros_pid" 2>/dev/null || true
        exit 0
    }
    trap '_wd_cleanup' TERM INT

    while true; do
        log "Watchdog: starting MAVROS (PX4)..."
        # Flush stale ROS2 daemon entries so check_ros_node doesn't
        # return true from a dead /mavros node left over from the crash.
        timeout 5 ros2 daemon stop >/dev/null 2>&1 || true
        free_port 14550
        sleep 1

        ros2 launch mavros node.launch \
            fcu_url:=${FCU_DEVICE}:${FCU_BAUD} \
            gcs_url:=udp://@${LAPTOP_IP}:${GCS_UDP_PORT} \
            pluginlists_yaml:=${SCRIPT_DIR}/px4_pluginlists_rover.yaml \
            config_yaml:=/opt/ros/humble/share/mavros/launch/px4_config.yaml \
            fcu_protocol:=v2.0 \
            tgt_system:=1 \
            tgt_component:=1 \
            log_output:=screen \
            respawn_mavros:=false &
        mavros_pid=$!

        local ready=0
        for i in $(seq 1 "$MAVROS_READY_WAIT"); do
            if check_ros_node "/mavros"; then
                ready=1
                break
            fi
            if ! kill -0 "$mavros_pid" 2>/dev/null; then
                log "Watchdog: MAVROS exited before node appeared"
                break
            fi
            log "Waiting for /mavros node... ($i/$MAVROS_READY_WAIT)"
            sleep 1
        done

        if [[ "$ready" -eq 1 ]]; then
            log "Watchdog: MAVROS ready (PID $mavros_pid)"
            touch "$MAVROS_READY_FLAG"
            wait "$mavros_pid" 2>/dev/null || true
            log "Watchdog: MAVROS exited — restarting in ${MAVROS_RESTART_DELAY}s..."
            mavros_pid=""
            sleep "$MAVROS_RESTART_DELAY"
        else
            log "Watchdog: MAVROS failed to start — retrying in ${MAVROS_FAIL_DELAY}s..."
            kill "$mavros_pid" 2>/dev/null || true
            wait "$mavros_pid" 2>/dev/null || true
            mavros_pid=""
            sleep "$MAVROS_FAIL_DELAY"
        fi
    done
}

ntrip_watchdog() {
    local ntrip_pid=""

    _ntrip_cleanup() {
        [[ -n "$ntrip_pid" ]] && kill "$ntrip_pid" 2>/dev/null || true
        exit 0
    }
    trap '_ntrip_cleanup' TERM INT

    while true; do
        log "Watchdog: starting NTRIP RTK client..."
        python3 "$NTRIP_SCRIPT" &
        ntrip_pid=$!
        wait "$ntrip_pid" 2>/dev/null || true
        log "Watchdog: NTRIP exited — restarting in ${NTRIP_RESTART_DELAY}s..."
        ntrip_pid=""
        sleep "$NTRIP_RESTART_DELAY"
    done
}

trap 'handle_exit $?' EXIT
trap 'handle_exit 130' INT
trap 'handle_exit 143' TERM

# Verify FCU device
if [[ ! -c "$FCU_DEVICE" ]]; then
    log "ERROR: $FCU_DEVICE not found — is CubeOrangePlus plugged in via USB?"
    exit 1
fi
log "FCU device found: $FCU_DEVICE (CubeOrangePlus PX4)"

# Source ROS 2
if [[ ! -f "$ROS_SETUP" ]]; then
    log "ERROR: ROS 2 setup not found at $ROS_SETUP"
    exit 1
fi
set +u; source "$ROS_SETUP"; set -u

# Stop daemon and kill stale processes — all best-effort, explicitly bracketed.
# `set -e` would abort the script if any of these fail, which is wrong here
# (they're cleanup ops; non-zero exit is expected when nothing is running).
set +e
timeout 5 ros2 daemon stop >/dev/null 2>&1
pkill -f "mavros.*node.launch" 2>/dev/null
pkill -f "ntrip_rtcm_node" 2>/dev/null
set -e
sleep 1

log "====================================================="
log " PX4 MAVROS UDP Bridge Starting"
log " FCU : $FCU_DEVICE @ ${FCU_BAUD} baud"
log " QGC : UDP directed to $LAPTOP_IP:$GCS_UDP_PORT"
log " QGC setup: Comm Links → Add → UDP → Port $GCS_UDP_PORT"
log " Or QGC auto-discovers on same LAN (no config needed)"
log "====================================================="

rm -f "$MAVROS_READY_FLAG"

mavros_watchdog &
MAVROS_WATCHDOG_PID=$!
CHILD_PIDS+=("$MAVROS_WATCHDOG_PID")

log "Waiting for MAVROS to initialise..."
mavros_ready=0
for i in $(seq 1 "$MAVROS_READY_TIMEOUT"); do
    if [[ -f "$MAVROS_READY_FLAG" ]]; then
        mavros_ready=1
        rm -f "$MAVROS_READY_FLAG"
        break
    fi
    if ! kill -0 "$MAVROS_WATCHDOG_PID" 2>/dev/null; then
        log "ERROR: MAVROS watchdog died unexpectedly"
        exit 1
    fi
    log "Waiting... ($i/$MAVROS_READY_TIMEOUT)"
    sleep 1
done

if [[ "$mavros_ready" -eq 0 ]]; then
    log "ERROR: MAVROS did not come up in time"
    exit 1
fi

log "MAVROS is READY"

# Validate FCU connection
# `timeout 10` is the outer guard; --timeout is NOT a valid ros2 topic echo
# flag in Humble — the outer timeout provides the same protection cleanly.
if timeout 10 ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "connected: true"; then
    log "FCU connected via MAVROS"
else
    log "WARNING: MAVROS node exists but FCU may not be connected — check serial link"
fi

log "Starting NTRIP RTK client..."
if [[ -z "${NTRIP_USER:-}" ]] || [[ -z "${NTRIP_PASS:-}" ]] || [[ -z "${NTRIP_MOUNTPT:-}" ]]; then
    log "WARNING: NTRIP_USER, NTRIP_PASS, or NTRIP_MOUNTPT env vars not set — NTRIP will crash-loop"
    log "Run deploy.sh to create config/ntrip.env"
fi
ntrip_watchdog &
NTRIP_WATCHDOG_PID=$!
CHILD_PIDS+=("$NTRIP_WATCHDOG_PID")

sleep "$NTRIP_READY_WAIT"
if ! kill -0 "$NTRIP_WATCHDOG_PID" 2>/dev/null; then
    log "WARNING: NTRIP watchdog exited immediately — check: journalctl -u px4-dxp.service -n 50"
fi

log "Active ROS nodes:"
timeout 10 ros2 node list 2>/dev/null || true
log "=== Bridge running. QGC → UDP → ${JETSON_IP}:${GCS_UDP_PORT} ==="

wait "$MAVROS_WATCHDOG_PID" "$NTRIP_WATCHDOG_PID"
