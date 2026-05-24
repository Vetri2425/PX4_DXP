# 02 — Baseline capture procedure (run first, run again after every upgrade)

**Agent:** Haiku 4.5
**Estimated diff:** ~250 lines (new tool + procedure doc), no controller changes
**Depends on:** none
**Blocks:** every other task (you must have a baseline to compare against)

## Goal

A repeatable script + rosbag procedure that produces a single CSV of
xtrack-vs-time and a one-line summary for every test path. Used both for
the initial baseline (before any upgrade) and as the gate at the end of
every upgrade.

## Files to read first

- `src/xtrack_logger_node.py` — existing 20 Hz logger.
- `src/rpp_controller_node.py` — `/rpp/debug` topic format (the first
  element is signed xtrack).
- `Test_mission/` — the .waypoints files listed in
  `01_architecture_and_upgrade_plan.md` § 6.

## Scope

### A. Create `tools/benchmark_rpp.py`

A single Python script (rclpy not required) that:

1. Reads a rosbag (`mcap` or `db3`) recorded during a mission run.
2. Extracts `/rpp/debug` (Float32MultiArray, 8+ elements) into a pandas
   DataFrame keyed by header.stamp.
3. Extracts `/mavros/local_position/pose` into the same frame.
4. Computes per-sample:
   - signed xtrack (already in `/rpp/debug[0]`)
   - heading err (already in `/rpp/debug[1]`)
   - lookahead (already in `/rpp/debug[2]`)
   - state code (already in `/rpp/debug[7]`)
5. Emits a CSV: `t, xtrack_m, heading_err_rad, lookahead_m, state_code,
   pos_n, pos_e, speed_m_s`.
6. Emits a one-line summary:
   ```
   path=square | n=1234 | t=23.4s | xtrack max=0.094 mean=0.011 rms=0.036
   | heading_err rms_deg=2.1 | endpoint_err_m=0.018
   ```
7. Optionally saves a PNG: xtrack-vs-time + xtrack-vs-arclength.

### B. Document the run procedure

A markdown sibling `tools/benchmark_rpp.md`:

- Pre-flight: how to put the rover at the path start, verify RTK FIXED,
  verify which params are loaded.
- Recording: `ros2 bag record /rpp/debug /mavros/local_position/pose
  /mavros/state /mavros/setpoint_raw/local /rpp/velocity_ned -o <name>`.
- Run the mission via the FastAPI server.
- Stop the bag after `state=DONE` (3 in `/rpp/debug[7]`).
- Analyse: `python tools/benchmark_rpp.py --bag <name> --path square
  --params Final_Best --out baseline/`.

### C. Standard baseline directory layout

```
baseline/
  2026-05-24_square_Final_Best/
    bag/                      # rosbag
    debug.csv                 # decoded
    summary.txt               # one-line
    xtrack.png                # plot
    notes.md                  # human notes
```

### D. Capture the initial set

Before any code change, capture baselines for at least:

- 2 m square — `Final_Best` params
- Half circle (Karney, 180°) — `Final_Best` params
- Straight 5 m — `Final_Best` params (create the mission file if missing —
  10 waypoints at 0.5 m spacing in a straight line)

## Out of scope

- Live xtrack display (that's task 08 in `task_frontend/`).
- Modifying any controller code.
- Sub-cm pose ground truth (we trust RTK FIXED HDOP < 0.7 as our reference).

## Acceptance criteria

- [ ] `tools/benchmark_rpp.py` runs against a real bag, produces CSV +
      summary + PNG without errors.
- [ ] Summary one-liner matches the canonical format above.
- [ ] Three baselines captured and committed under `baseline/`.
- [ ] Procedure doc readable by someone who has never run this before.

## Notes for the agent

- Pandas + rosbag2 reading on Jetson: prefer `rosbags` (third-party, pip
  installable) over the official `rosbag2_py` bindings — much simpler.
- The endpoint error in summary: distance from the final pose sample to
  the last waypoint of the input mission.
- Do not check rosbags into git. The `baseline/<run>/bag/` directories are
  gitignored.
