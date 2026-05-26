#!/bin/bash
# Drawing Rover FastAPI backend — launch script
# Called by rover-server.service (systemd Type=notify + WatchdogSec=30)
set -euo pipefail

# Pre-define so ROS2 chain never trips on it with set -u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export FASTAPI_PORT=${FASTAPI_PORT:-5001}

cd "$(dirname "$0")"

# Install pip dependencies if missing (first run after deploy)
if ! python3 -c "import fastapi" 2>/dev/null; then
    echo "[run.sh] Installing pip dependencies..."
    pip3 install --user -r requirements.txt
fi

# Notify systemd that we're ready (Type=notify)
# The actual READY=1 is sent from Python after lifespan completes.
# sd_notify from bash is a fallback if sdnotify Python package is missing.

exec python3 -m uvicorn main:app \
    --host 0.0.0.0 \
    --port "$FASTAPI_PORT" \
    --log-level info
