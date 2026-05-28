#!/usr/bin/env bash
# Capture ROS2/MAVROS diagnostics for OFFBOARD arming failures.
#
# Usage, terminal 1:
#   cd ~/PX4_DXP
#   ros2 launch src/launch/rpp_pipeline.launch.py path_name:=arc_half_1m5 auto_run:=true auto_origin:=true
#
# Usage, terminal 2:
#   cd ~/PX4_DXP
#   bash tools/capture_offboard_diag.sh
#
# Then run the printed commands in QGC MAVLink Console while terminal 1 is
# switching to OFFBOARD / attempting to arm.

set -u

DURATION_S="${1:-45}"
INTERVAL_S="${INTERVAL_S:-1}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="/tmp/offboard_diag_${STAMP}"
LOG="${OUT_DIR}/capture.log"
BAG_DIR="${OUT_DIR}/rosbag"

mkdir -p "$OUT_DIR"

log() {
    local line
    line="[$(date '+%H:%M:%S')] $*"
    echo "$line" | tee -a "$LOG"
}

section() {
    {
        echo
        echo "===== $* ====="
        date '+%Y-%m-%d %H:%M:%S.%3N %Z'
    } | tee -a "$LOG"
}

run_capture() {
    local name="$1"
    shift
    section "$name"
    "$@" >>"$LOG" 2>&1
}

source_ros() {
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        # shellcheck disable=SC1091
        set +u  # ROS setup scripts reference unbound vars internally
        source /opt/ros/humble/setup.bash
        set -u
    else
        log "WARN: /opt/ros/humble/setup.bash not found"
    fi
    export ROS_DOMAIN_ID
}

topic_once() {
    local topic="$1"
    local seconds="${2:-3}"
    echo "--- $topic ---" >>"$LOG"
    timeout "$seconds" ros2 topic echo "$topic" --once >>"$LOG" 2>&1 || true
}

topic_hz() {
    local topic="$1"
    local seconds="${2:-5}"
    echo "--- hz $topic (${seconds}s) ---" >>"$LOG"
    timeout "$seconds" ros2 topic hz "$topic" >>"$LOG" 2>&1 || true
}

print_qgc_commands() {
    cat <<'EOF' | tee "${OUT_DIR}/qgc_console_commands.txt" | tee -a "$LOG"

===== RUN THESE IN QGC MAVLINK CONSOLE DURING THE ARM FAILURE =====

commander check
listener vehicle_status 1
listener failsafe_flags 1
listener offboard_control_mode 1
listener vehicle_local_position 1
listener vehicle_attitude 1
listener sensor_combined 1
commander check

Copy the output into the chat after this script finishes.

EOF
}

snapshot_static() {
    run_capture "HOST" bash -lc 'hostname; uname -a; date; id; pwd'
    run_capture "ENV" bash -lc 'env | sort | grep -E "ROS_|RMW_|FASTRTPS|CYCLONE|DDS|PX4|MAV" || true'
    run_capture "SYSTEMD RPP" bash -lc 'systemctl is-active rpp-pipeline.service 2>/dev/null; systemctl status rpp-pipeline.service --no-pager -l 2>/dev/null | sed -n "1,80p" || true'
    run_capture "SYSTEMD PX4-DXP" bash -lc 'systemctl is-active px4-dxp.service 2>/dev/null; systemctl status px4-dxp.service --no-pager -l 2>/dev/null | sed -n "1,80p" || true'
    run_capture "PROCESS MATCHES" bash -lc 'pgrep -af "twist_to_setpoint|rpp_controller|xtrack_logger|path_publisher|mission_runner|mavros|rpp_start|rpp_pipeline" || true'
    run_capture "ROS NODE DUPLICATES" bash -lc 'ros2 node list 2>&1 | sort | uniq -c'
    run_capture "ROS TOPIC LIST FILTERED" bash -lc 'ros2 topic list 2>&1 | grep -E "mavros|rpp|path|parameter_events" || true'
}

snapshot_topics() {
    section "ONE-SHOT TOPIC SNAPSHOTS"
    topic_once /mavros/state 4
    topic_once /mavros/statustext 2
    topic_once /mavros/local_position/pose 4
    topic_once /mavros/local_position/velocity_local 4
    topic_once /mavros/gpsstatus/gps1/raw 4
    topic_once /rpp/debug 4
    topic_once /rpp/velocity_ned 4
    topic_once /rpp/yaw_rate_body 2
    topic_once /mavros/setpoint_raw/local 4

    section "TOPIC RATES"
    topic_hz /mavros/setpoint_raw/local 6
    topic_hz /rpp/velocity_ned 6
    topic_hz /rpp/debug 6
    topic_hz /mavros/local_position/pose 6
    topic_hz /mavros/local_position/velocity_local 6
}

start_bag() {
    section "ROS BAG START"
    if ! command -v ros2 >/dev/null 2>&1; then
        log "WARN: ros2 command unavailable, skipping bag"
        return
    fi

    ros2 bag record \
        /mavros/state \
        /mavros/statustext \
        /mavros/local_position/pose \
        /mavros/local_position/velocity_local \
        /mavros/gpsstatus/gps1/raw \
        /mavros/setpoint_raw/local \
        /rpp/debug \
        /rpp/velocity_ned \
        /rpp/yaw_rate_body \
        -o "$BAG_DIR" >>"$LOG" 2>&1 &
    BAG_PID=$!
    log "ros2 bag record started pid=${BAG_PID}, output=${BAG_DIR}"
}

stop_bag() {
    if [[ "${BAG_PID:-}" != "" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
        section "ROS BAG STOP"
        kill -INT "$BAG_PID" 2>/dev/null || true
        wait "$BAG_PID" 2>/dev/null || true
    fi
}

watch_loop() {
    section "LIVE WATCH LOOP (${DURATION_S}s)"
    local end
    end=$((SECONDS + DURATION_S))

    while [[ "$SECONDS" -lt "$end" ]]; do
        {
            echo
            echo "--- tick $(date '+%H:%M:%S.%3N') ---"
            echo "[nodes]"
            ros2 node list 2>&1 | sort | uniq -c
            echo "[mavros state]"
            timeout 1 ros2 topic echo /mavros/state --once 2>&1 | grep -E "connected:|armed:|mode:" || true
            echo "[rpp debug data]"
            timeout 1 ros2 topic echo /rpp/debug --once 2>&1 | grep -E "data:|---" || true
            echo "[setpoint]"
            timeout 1 ros2 topic echo /mavros/setpoint_raw/local --once 2>&1 | grep -E "coordinate_frame:|type_mask:|velocity:|yaw:|yaw_rate:|---" || true
        } >>"$LOG"
        sleep "$INTERVAL_S"
    done
}

main() {
    source_ros
    log "Writing diagnostics to $OUT_DIR"
    print_qgc_commands
    snapshot_static
    start_bag
    watch_loop
    snapshot_topics
    stop_bag
    run_capture "RECENT JOURNAL RPP/PX4" bash -lc 'journalctl --since "10 minutes ago" --no-pager 2>/dev/null | grep -E "rpp-pipeline|px4-dxp|mission_runner|rpp_controller|twist_to_setpoint|mavros|RTPS_TRANSPORT_SHM|open_and_lock_file" | tail -300 || true'
    log "DONE"
    log "Main log: $LOG"
    log "ROS bag:  $BAG_DIR"
    log "QGC commands saved at: ${OUT_DIR}/qgc_console_commands.txt"
}

trap stop_bag EXIT INT TERM
main "$@"
