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
| `px4_start_service.sh` | systemd launcher/watchdog for MAVROS2 |
| `ntrip_rtcm_node.py` | supervised NTRIP client and RTCM3 injector |
| `px4_pluginlists_rover.yaml` | MAVROS plugin allowlist |
| `CLAUDE.md` | Context file for Claude Code (runtime brain scope) |

## Architecture

See [Architecture Decision](https://github.com/Vetri2425/PX4-Autopilot) — laptop side owns firmware patches, this side owns ROS2 runtime.

## Changelog

**2026-08-20 — RTK/NTRIP reliability:** `rover-server` restores NTRIP from
`config/ntrip.env` on startup and supervises child exits. The RPP and AUTO
spray gates now require a fresh RTK_FIXED sample with known horizontal
accuracy ≤10 cm, then one continuous second of good quality before recovery.
See `docs/RTK_NTRIP_OPERATIONS.md` for setup and field checks.

**2026-08-20 — spray safety hardening:** `spray_controller_node.py` continuously leases ON authority to a separate `spray_safety_watchdog_node.py`. Missing, false, malformed, or stale leases independently drive AUX OFF; the controller also refuses ON without a fresh reciprocal watchdog heartbeat. `rpp_start.sh` supervises both without restarting the OFFBOARD-critical drive nodes. Physical valve feedback and hardware fail-closed validation remain required.

**2026-05-25 — path_engine v1.0 (Phases 1-4):** Added complete path planning subsystem for DXF/CSV/QGC mission files. Phase 1: core data models (SegmentType, PathSegment, PlannedPath, DXFEntity), parsers (ezdxf-based DXF with LINE/POINT/SPLINE, enhanced 6-col CSV with backward-compatible 2-col, QGC .waypoints via Karney geodesic), straight-line densification at 5cm MARK/15cm TRANSIT spacing. Phase 2: curvature-adaptive arc/circle discretization using chord-error (sagitta) method, LWPOLYLINE bulge-to-arc conversion (positive=CCW, negative=CW per DXF standard), ELLIPSE via ezdxf make_path+flattening. Phase 3: nearest-neighbor TSP segment ordering with endpoint reversal, TRANSIT segment insertion, spray latency compensation, per-entity spray overrides, and `spray_flags` parallel to `merged_waypoints`. Phase 4/current runtime: planned paths publish flags via `/path` `pose.position.z`; RPP emits `/spray/active`; FastAPI exposes path planning, staging, load-to-controller, and spray endpoints. Dependencies: ezdxf, geographiclib (existing). scipy NOT required.

## Service

```bash
systemctl status px4-dxp.service
journalctl -u px4-dxp.service -f
```

## License

TBD
