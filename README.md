# PX4_DXP — 3WD Marking Rover (Jetson Runtime)

Runtime workspace on the Jetson Orin companion computer for the DYX Autonomous 3WD marking rover.

- **FCU:** CubeOrangePlus running PX4 v1.16.2 (custom rover build, fork: [Vetri2425/PX4-Autopilot](https://github.com/Vetri2425/PX4-Autopilot))
- **Bridge:** MAVROS2 over `/dev/ttyACM0` @ 921600
- **RTK:** Holybro UM982 dual-antenna with NTRIP injection
- **ROS2:** Humble on Ubuntu (Tegra)
- **Role:** Phase 2 ROS2 OFFBOARD arc controller (replaces PX4 AUTO densified-waypoint method)

## Contents

| File | Purpose |
|---|---|
| `px4_start_service.sh` | systemd launcher for MAVROS2 + NTRIP |
| `px4_pluginlists_rover.yaml` | MAVROS plugin allowlist |
| `CLAUDE.md` | Context file for Claude Code (runtime brain scope) |

## Architecture

See [Architecture Decision](https://github.com/Vetri2425/PX4-Autopilot) — laptop side owns firmware patches, this side owns ROS2 runtime.

## Service

```bash
systemctl status px4-dxp.service
journalctl -u px4-dxp.service -f
```

## License

TBD
