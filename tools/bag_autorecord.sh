#!/usr/bin/env bash
# Wrapper for the auto rosbag recorder: source ROS2, then exec the daemon.
set -euo pipefail

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# Where to save bags (override via systemd Environment=BAGS_DIR=...)
export BAGS_DIR="${BAGS_DIR:-$HOME/bags_jet}"
mkdir -p "$BAGS_DIR"

exec python3 "$(dirname "$0")/bag_autorecord.py"
