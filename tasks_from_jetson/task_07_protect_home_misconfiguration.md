# Task 07 — ProtectHome Misconfiguration (Security Hardening Defeated)

**Priority:** MEDIUM
**File:** `/etc/systemd/system/px4-dxp.service`
**Lines:** 27–28

---

## Problem

```ini
# px4-dxp.service:27-28
ProtectHome=read-only
ReadWritePaths=/home/flash /tmp /var/tmp /home/flash/.ros
```

`ProtectHome=read-only` is intended to make the home directory read-only. But `ReadWritePaths=/home/flash` immediately re-grants full write access to the entire home directory. The two directives cancel each other out — `ProtectHome=read-only` does nothing.

This means the service (which runs as `flash`) has full write access to all of `/home/flash`, including:
- `~/.ssh/` — SSH authorized keys
- `~/.claude/` — Claude memory/config
- Any credentials or config files in home

For a service that only needs write access to specific subdirectories, this is overly permissive.

## What the Service Actually Needs Write Access To

From reading `px4_start_service.sh` and `ntrip_rtcm_node.py`:

| Path | Why write access needed |
|------|------------------------|
| `/tmp` | `ros2 launch` writes `/tmp/launch_params_*` temp files |
| `/var/tmp` | systemd default for some runtime state |
| `/home/flash/.ros` | ROS2 log files (`~/.ros/log/`) |
| `/home/flash/bags/` | If rosbag recording is started (Phase 2) |
| `/tmp/ntrip.log` | NTRIP node log (already under `/tmp`) |

It does **not** need write access to the rest of `/home/flash`.

## Required Fix (do not apply — analysis only)

```ini
ProtectHome=yes
ReadWritePaths=/home/flash/.ros /tmp /var/tmp
```

`ProtectHome=yes` (not `read-only`) makes home completely inaccessible except for paths explicitly listed in `ReadWritePaths`. Add `/home/flash/bags` when rosbag recording is implemented.

Note: The service script itself is at `/home/flash/PX4_DXP/px4_start_service.sh` — read access to this path is implicitly granted since it's the `ExecStart` binary. `ProtectHome=yes` still allows reading the script, just not writing.

---

**Depends on:** None
**Blocks:** Nothing
