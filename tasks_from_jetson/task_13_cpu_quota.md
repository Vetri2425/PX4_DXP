# Task 13 — CPUQuota=200% Can Throttle MAVROS Under Burst Load

**Priority:** LOW
**File:** `/etc/systemd/system/px4-dxp.service`
**Line:** 39

---

## Problem

```ini
# px4-dxp.service:39
CPUQuota=200%
```

On the Jetson Orin Nano Super (6-core ARM Cortex-A78), `200%` = 2 full cores. The service cgroup is throttled if the aggregate CPU usage of all processes exceeds 2 cores in any scheduling period (100ms by default in Linux cgroups v2).

## What Runs Under This Cgroup

From `systemctl status`, the service cgroup contains:
- `px4_start_service.sh` (bash — minimal)
- `ros2 daemon` (Python — DDS discovery overhead)
- `ros2 launch mavros node.launch` (Python launch wrapper)
- `mavros_node` (C++ — serial parser, MAVLink decoder, ROS2 publisher)
- `ntrip_rtcm_node.py` (Python — NTRIP socket + ROS2 publisher)

At steady state, total CPU is well under 200%. The risk is burst scenarios:

- **MAVROS topic burst:** When PX4 sends a flood of MAVLink messages (e.g., after arming, during mode transitions), `mavros_node` CPU spikes for 50–200ms
- **Phase 2 OFFBOARD:** Adding setpoint publisher nodes and arc controller will push steady-state CPU higher
- **rosbag recording:** `ros2 bag record` is CPU-intensive; if run inside this service (which it currently is not, but may be), it can consume a full core

If throttling hits during a MAVLink burst, the parser thread falls behind, messages queue up in the serial buffer (limited depth on the kernel TTY driver), and bytes are eventually dropped — causing the same symptoms as `RestrictRealtime=yes` (Task 05).

## Required Fix (do not apply — analysis only)

For Phase 1 (current state), `CPUQuota=200%` is probably fine. Flag for review when Phase 2 nodes are added. Either:

- Raise to `CPUQuota=400%` to give MAVROS + controller nodes headroom, or
- Remove `CPUQuota` entirely — the Jetson is a dedicated rover computer with no competing workloads

If `CPUQuota` is kept for protection against runaway processes, pair it with `CPUAccounting=yes` and monitor via `systemd-cgtop` during Phase 2 integration testing.

---

**Depends on:** None — review again after Phase 2 nodes are added
**Blocks:** Nothing currently
