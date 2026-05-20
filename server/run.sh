#!/bin/bash
# Drawing Rover FastAPI backend — launch script
set -euo pipefail

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source the DXP workspace if built locally
# source ~/PX4_DXP/install/setup.bash

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export FASTAPI_PORT=${FASTAPI_PORT:-5001}

cd "$(dirname "$0")"

exec python3 -m uvicorn main:app \
    --host 0.0.0.0 \
    --port "$FASTAPI_PORT" \
    --log-level info
