#!/bin/bash
# deploy.sh — Sync PX4_DXP repo files to system locations on Jetson
# Run after: git pull
# Usage:   cd ~/PX4_DXP && ./deploy.sh [--restart]
#
# What it does:
#   1. Symlinks systemd service → /etc/systemd/system/
#   2. Reloads systemd daemon
#   3. With --restart: restarts px4-dxp.service
#
# Symlinks mean future `git pull` updates are live immediately —
# no re-deploy needed for file content changes. Only re-run this
# if you add NEW files or change the service definition.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RESTART=false

if [[ "${1:-}" == "--restart" ]]; then
    RESTART=true
fi

log() { echo "[deploy] $*"; }

# ── 1. Systemd service ──────────────────────────────────────────────
SERVICE_SRC="${SCRIPT_DIR}/px4-dxp.service"
SERVICE_DST="/etc/systemd/system/px4-dxp.service"

if [[ -L "$SERVICE_DST" ]]; then
    CURRENT_TARGET=$(readlink -f "$SERVICE_DST")
    if [[ "$CURRENT_TARGET" == "$SERVICE_SRC" ]]; then
        log "systemd: symlink already correct → ${SERVICE_SRC}"
    else
        log "systemd: updating symlink ${SERVICE_DST} → ${SERVICE_SRC}"
        sudo ln -sf "$SERVICE_SRC" "$SERVICE_DST"
    fi
elif [[ -f "$SERVICE_DST" ]]; then
    log "systemd: replacing file with symlink ${SERVICE_DST} → ${SERVICE_SRC}"
    sudo mv "$SERVICE_DST" "${SERVICE_DST}.bak"
    sudo ln -s "$SERVICE_SRC" "$SERVICE_DST"
else
    log "systemd: creating symlink ${SERVICE_DST} → ${SERVICE_SRC}"
    sudo ln -s "$SERVICE_SRC" "$SERVICE_DST"
fi

# ── 1b. RPP pipeline service ───────────────────────────────────────
_deploy_service() {
    local src="$1" dst="$2" name="$3"
    if [[ -L "$dst" ]]; then
        local current
        current=$(readlink -f "$dst")
        if [[ "$current" == "$src" ]]; then
            log "systemd: $name symlink already correct"
        else
            log "systemd: updating $name symlink"
            sudo ln -sf "$src" "$dst"
        fi
    elif [[ -f "$dst" ]]; then
        sudo mv "$dst" "${dst}.bak"
        sudo ln -s "$src" "$dst"
    else
        log "systemd: creating $name symlink"
        sudo ln -s "$src" "$dst"
    fi
}

_deploy_service "${SCRIPT_DIR}/rpp-pipeline.service" \
    "/etc/systemd/system/rpp-pipeline.service" "rpp-pipeline"

_deploy_service "${SCRIPT_DIR}/rover-server.service" \
    "/etc/systemd/system/rover-server.service" "rover-server"

_deploy_service "${SCRIPT_DIR}/bag-autorecord.service" \
    "/etc/systemd/system/bag-autorecord.service" "bag-autorecord"

# Auto-bag output folder (pulled from the Mac; outside the read-only repo)
mkdir -p "${HOME}/bags_jet"
log "bags: ${HOME}/bags_jet ready"

# Make startup scripts executable
chmod +x "${SCRIPT_DIR}/rpp_start.sh" 2>/dev/null || true
chmod +x "${SCRIPT_DIR}/server/run.sh" 2>/dev/null || true
chmod +x "${SCRIPT_DIR}/server/rover_auth_cli.py" 2>/dev/null || true
chmod +x "${SCRIPT_DIR}/tools/bag_autorecord.sh" 2>/dev/null || true

# ── 2. Keep manual ROS shells on the same Fast DDS profile ─────────
DDS_PROFILE_EXPORT="export FASTRTPS_DEFAULT_PROFILES_FILE=${SCRIPT_DIR}/config/fastdds_no_shm.xml"

_ensure_shell_export() {
    local shell_file="$1"
    touch "$shell_file"
    if grep -qxF "$DDS_PROFILE_EXPORT" "$shell_file"; then
        log "shell: FASTRTPS_DEFAULT_PROFILES_FILE already configured in ${shell_file}"
    elif grep -q "^export FASTRTPS_DEFAULT_PROFILES_FILE=" "$shell_file"; then
        sed -i.bak "s#^export FASTRTPS_DEFAULT_PROFILES_FILE=.*#${DDS_PROFILE_EXPORT}#" "$shell_file"
        log "shell: updated FASTRTPS_DEFAULT_PROFILES_FILE in ${shell_file}"
    else
        {
            echo ""
            echo "# PX4_DXP: disable Fast DDS shared memory for rover ROS diagnostics."
            echo "$DDS_PROFILE_EXPORT"
        } >> "$shell_file"
        log "shell: added FASTRTPS_DEFAULT_PROFILES_FILE to ${shell_file}"
    fi
}

_ensure_shell_export "${HOME}/.bashrc"
_ensure_shell_export "${HOME}/.profile"

# ── 3. Reload systemd ──────────────────────────────────────────────
sudo systemctl daemon-reload
log "systemd: daemon reloaded"

# ── 4. Enable service (if not already) ─────────────────────────────
if systemctl is-enabled px4-dxp.service >/dev/null 2>&1; then
    log "systemd: px4-dxp already enabled"
else
    sudo systemctl enable px4-dxp.service
    log "systemd: px4-dxp enabled"
fi

if systemctl is-enabled rpp-pipeline.service >/dev/null 2>&1; then
    log "systemd: rpp-pipeline already enabled"
else
    sudo systemctl enable rpp-pipeline.service
    log "systemd: rpp-pipeline enabled"
fi

if systemctl is-enabled rover-server.service >/dev/null 2>&1; then
    log "systemd: rover-server already enabled"
else
    sudo systemctl enable rover-server.service
    log "systemd: rover-server enabled"
fi

if systemctl is-enabled bag-autorecord.service >/dev/null 2>&1; then
    log "systemd: bag-autorecord already enabled"
else
    sudo systemctl enable bag-autorecord.service
    log "systemd: bag-autorecord enabled"
fi

# ── 8. Restart (optional) ──────────────────────────────────────────
if $RESTART; then
    log "Restarting all services..."
    sudo systemctl restart px4-dxp.service
    sleep 3
    sudo systemctl restart rpp-pipeline.service
    sleep 2
    sudo systemctl restart rover-server.service
    sleep 3
    log ""
    log "Service status:"
    for svc in px4-dxp rpp-pipeline rover-server; do
        if systemctl is-active "${svc}.service" >/dev/null 2>&1; then
            log "  ✓ $svc is ACTIVE"
        else
            log "  ✗ $svc is NOT active — check: journalctl -u ${svc}.service -n 50"
        fi
    done
else
    log ""
    log "Files deployed. To restart all services now, run:"
    log "  sudo systemctl restart px4-dxp rpp-pipeline rover-server"
    log ""
    log "Or re-run with --restart:"
    log "  ./deploy.sh --restart"
fi

log "Done."
