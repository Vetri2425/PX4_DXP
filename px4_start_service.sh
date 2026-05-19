#!/bin/bash

# PX4 MAVROS bridge — CubeOrangePlus via /dev/ttyACM0
# QGC connects via UDP (no telemetry radio needed):
#   In QGC: Comm Links → UDP → Port 14550 → connect
#   Or QGC auto-discovers via UDP broadcast on the LAN

set -euo pipefail

FCU_DEVICE="/dev/ttyACM0"
FCU_BAUD="921600"
GCS_UDP_PORT="14550"
JETSON_IP="192.168.1.102"
ROS_SETUP="/opt/ros/humble/setup.bash"

declare -a CHILD_PIDS=()
MAVROS_WATCHDOG_PID=""

log() { echo "[px4_service] $(date '+%H:%M:%S') $*"; }

cleanup() {
    log "Cleaning up child processes..."
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
    ros2 node list 2>/dev/null | grep -q "$1"
}

free_port() {
    local port=$1
    if lsof -i ":$port" >/dev/null 2>&1; then
        log "Port $port busy — freeing..."
        lsof -ti ":$port" | xargs -r kill -9 || true
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
        free_port 14550
        sleep 1

        ros2 launch mavros node.launch             fcu_url:=${FCU_DEVICE}:${FCU_BAUD}             gcs_url:=udp-b://:${GCS_UDP_PORT}@             pluginlists_yaml:=/home/flash/PX4_DXP/px4_pluginlists_rover.yaml             config_yaml:=/opt/ros/humble/share/mavros/launch/px4_config.yaml             fcu_protocol:=v2.0             tgt_system:=1             tgt_component:=1             log_output:=screen             respawn_mavros:=false &
        mavros_pid=$!

        local ready=0
        for i in {1..30}; do
            if check_ros_node "/mavros"; then
                ready=1
                break
            fi
            if ! kill -0 "$mavros_pid" 2>/dev/null; then
                log "Watchdog: MAVROS exited before node appeared"
                break
            fi
            log "Waiting for /mavros node... ($i/30)"
            sleep 1
        done

        if [[ "$ready" -eq 1 ]]; then
            log "Watchdog: MAVROS ready (PID $mavros_pid)"
            wait "$mavros_pid" 2>/dev/null || true
            log "Watchdog: MAVROS exited — restarting in 3s..."
            mavros_pid=""
            sleep 3
        else
            log "Watchdog: MAVROS failed to start — retrying in 5s..."
            kill "$mavros_pid" 2>/dev/null || true
            wait "$mavros_pid" 2>/dev/null || true
            mavros_pid=""
            sleep 5
        fi
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

ros2 daemon stop >/dev/null 2>&1 || true

# Kill any stale MAVROS instances
pkill -f "mavros px4.launch" 2>/dev/null || true
sleep 1

log "====================================================="
log " PX4 MAVROS UDP Bridge Starting"
log " FCU : $FCU_DEVICE @ ${FCU_BAUD} baud"
log " QGC : UDP broadcast port $GCS_UDP_PORT"
log " QGC setup: Comm Links → Add → UDP → Port $GCS_UDP_PORT"
log " Or QGC auto-discovers on same LAN (no config needed)"
log "====================================================="

mavros_watchdog &
MAVROS_WATCHDOG_PID=$!
CHILD_PIDS+=("$MAVROS_WATCHDOG_PID")

log "Waiting for MAVROS to initialise..."
mavros_ready=0
for i in {1..35}; do
    if check_ros_node "/mavros"; then
        mavros_ready=1
        break
    fi
    if ! kill -0 "$MAVROS_WATCHDOG_PID" 2>/dev/null; then
        log "ERROR: MAVROS watchdog died unexpectedly"
        exit 1
    fi
    log "Waiting... ($i/35)"
    sleep 1
done

if [[ "$mavros_ready" -eq 0 ]]; then
    log "ERROR: MAVROS did not come up in time"
    exit 1
fi

log "MAVROS is READY"
log "Starting NTRIP RTK client..."
nohup python3 /home/flash/ntrip_rtcm_node.py >> /tmp/ntrip.log 2>&1 &
CHILD_PIDS+=($!)
sleep 2

log "Active ROS nodes:"
ros2 node list || true
log "=== Bridge running. QGC → UDP → ${JETSON_IP}:${GCS_UDP_PORT} ==="

wait "$MAVROS_WATCHDOG_PID"
