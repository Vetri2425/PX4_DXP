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

## Changelog

**2026-05-25 — path_engine v1.0 (Phases 1-4):** Added complete path planning subsystem for DXF/CSV/QGC mission files. Phase 1: core data models (SegmentType, PathSegment, PlannedPath, DXFEntity), parsers (ezdxf-based DXF with LINE/POINT/SPLINE, enhanced 6-col CSV with backward-compatible 2-col, QGC .waypoints via Karney geodesic), straight-line densification at 5cm MARK/15cm TRANSIT spacing. Phase 2: curvature-adaptive arc/circle discretization using chord-error (sagitta) method, LWPOLYLINE bulge-to-arc conversion (positive=CCW, negative=CW per DXF standard), ELLIPSE via ezdxf make_path+flattening. Phase 3: nearest-neighbor TSP segment ordering with endpoint reversal (enters segment from whichever end is closest), TRANSIT segment insertion between MARK segments at 0.5m/s, spray latency compensation (3.5cm lead-in, 3.5mm lead-out for 0.35m/s marking speed). Phase 4: ROS2 integration — path_publisher_node extended for DXF/CSV via PathEngine with start_position from MAVROS pose, /dyx/spray_cmd (Bool, edge-triggered from spray_flags), /dyx/mission/progress (Float32, 1Hz), FastAPI endpoints (/api/path/parse-dxf, /api/path/plan with ref_points and start_position), CLI standalone entry point. 97/97 tests. Bug fix: spray.py lead-out compensation sign error corrected. Dependencies: ezdxf, geographiclib (existing). scipy NOT required.

## Service

```bash
systemctl status px4-dxp.service
journalctl -u px4-dxp.service -f
```

## License

TBD
