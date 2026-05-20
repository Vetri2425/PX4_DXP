#!/bin/bash
# deploy.sh — Sync PX4_DXP repo files to system locations on Jetson
# Run after: git pull
# Usage:   cd ~/PX4_DXP && ./deploy.sh [--restart]
#
# What it does:
#   1. Symlinks systemd service → /etc/systemd/system/
#   2. Symlinks logrotate config → /etc/logrotate.d/
#   3. Creates NTRIP env file if missing (prompts for credentials)
#   4. Reloads systemd daemon
#   5. With --restart: restarts px4-dxp.service
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

# ── 2. Logrotate config ────────────────────────────────────────────
LOGROTATE_SRC="${SCRIPT_DIR}/ntrip.logrotate"
LOGROTATE_DST="/etc/logrotate.d/ntrip"

if [[ -L "$LOGROTATE_DST" ]]; then
    CURRENT_TARGET=$(readlink -f "$LOGROTATE_DST")
    if [[ "$CURRENT_TARGET" == "$LOGROTATE_SRC" ]]; then
        log "logrotate: symlink already correct → ${LOGROTATE_SRC}"
    else
        log "logrotate: updating symlink ${LOGROTATE_DST} → ${LOGROTATE_SRC}"
        sudo ln -sf "$LOGROTATE_SRC" "$LOGROTATE_DST"
    fi
elif [[ -f "$LOGROTATE_DST" ]]; then
    log "logrotate: replacing file with symlink ${LOGROTATE_DST} → ${LOGROTATE_SRC}"
    sudo mv "$LOGROTATE_DST" "${LOGROTATE_DST}.bak"
    sudo ln -s "$LOGROTATE_SRC" "$LOGROTATE_DST"
else
    log "logrotate: creating symlink ${LOGROTATE_DST} → ${LOGROTATE_SRC}"
    sudo ln -s "$LOGROTATE_SRC" "$LOGROTATE_DST"
fi

# ── 3. NTRIP credentials ───────────────────────────────────────────
NTRIP_ENV="${SCRIPT_DIR}/config/ntrip.env"

if [[ -f "$NTRIP_ENV" ]]; then
    log "ntrip: env file exists at ${NTRIP_ENV}"
    # Patch existing file if NTRIP_MOUNTPT is missing
    if ! grep -q "^NTRIP_MOUNTPT=" "$NTRIP_ENV"; then
        echo ""
        echo "  NTRIP_MOUNTPT not found in ${NTRIP_ENV} — adding it now."
        read -rp "  NTRIP_MOUNTPT (e.g. MP23960a): " ntrip_mountpt
        echo "NTRIP_MOUNTPT=${ntrip_mountpt}" >> "$NTRIP_ENV"
        log "ntrip: NTRIP_MOUNTPT added to ${NTRIP_ENV}"
    fi
else
    log "ntrip: env file NOT found — creating it now"
    mkdir -p "$(dirname "$NTRIP_ENV")"

    echo ""
    echo "  NTRIP credentials required for RTK injection."
    echo "  Stored at ${NTRIP_ENV} (gitignored, flash-owned, mode 600)."
    echo ""
    read -rp "  NTRIP_USER: " ntrip_user
    read -rp "  NTRIP_PASS: " ntrip_pass
    read -rp "  NTRIP_MOUNTPT (e.g. MP23960a): " ntrip_mountpt

    printf "NTRIP_USER=%s\nNTRIP_PASS=%s\nNTRIP_MOUNTPT=%s\n" \
        "$ntrip_user" "$ntrip_pass" "$ntrip_mountpt" > "$NTRIP_ENV"
    chmod 600 "$NTRIP_ENV"
    log "ntrip: env file created at ${NTRIP_ENV} (mode 600, owner flash)"
fi

# ── 4. Ensure service references env file ──────────────────────────
# The service file has EnvironmentFile commented out by default.
# Check if it's uncommented; if not, warn the user.
if grep -q "^EnvironmentFile=" "$SERVICE_SRC" 2>/dev/null; then
    log "systemd: EnvironmentFile is active in service"
elif grep -q "^# EnvironmentFile=" "$SERVICE_SRC" 2>/dev/null; then
    log "WARNING: EnvironmentFile is commented out in service file"
    log "  NTRIP credentials won't be loaded until you uncomment it."
    log "  Edit ${SERVICE_SRC} and uncomment the EnvironmentFile line."
fi

# ── 5. Reload systemd ──────────────────────────────────────────────
sudo systemctl daemon-reload
log "systemd: daemon reloaded"

# ── 6. Enable service (if not already) ─────────────────────────────
if systemctl is-enabled px4-dxp.service >/dev/null 2>&1; then
    log "systemd: service already enabled"
else
    sudo systemctl enable px4-dxp.service
    log "systemd: service enabled"
fi

# ── 7. Restart (optional) ──────────────────────────────────────────
if $RESTART; then
    log "Restarting px4-dxp.service..."
    sudo systemctl restart px4-dxp.service
    sleep 3
    if systemctl is-active px4-dxp.service >/dev/null 2>&1; then
        log "Service is ACTIVE"
    else
        log "WARNING: Service not active — check: journalctl -u px4-dxp.service -n 50"
    fi
else
    log ""
    log "Files deployed. To restart the service now, run:"
    log "  sudo systemctl restart px4-dxp.service"
    log ""
    log "Or re-run with --restart:"
    log "  ./deploy.sh --restart"
fi

log "Done."