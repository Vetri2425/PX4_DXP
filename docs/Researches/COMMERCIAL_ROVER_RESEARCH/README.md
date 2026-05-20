# Commercial Marking Rover — Research Tasks

**Purpose:** Define every component needed to build a reliable commercial drawing/marking rover. Each task file is self-contained and can be given to ChatGPT, Grok, Gemini, or any AI for research. Synthesize outputs into final architecture documentation before writing any code.

**Our rover:** 3WD differential drive (2 drive wheels + front caster), wheelbase 470mm, PX4 v1.16.2 on CubeOrangePlus, Jetson Orin companion computer, UM982 dual-antenna RTK GNSS, ROS2 Humble + MAVROS2 OFFBOARD mode.

**Current state:** Basic OFFBOARD control works (position + velocity setpoints streaming at 50Hz). Need full pipeline from "mission file" to "wheel commands" before continuing.

## Task Files

| # | File | Topic | Status |
|---|------|-------|--------|
| T1 | `T1_mission_formats.md` | Mission/file formats used by commercial marking rovers | TODO |
| T2 | `T2_trajectory_planning.md` | Pattern/trajectory planning from CAD, DXF, waypoints | TODO |
| T3 | `T3_controller_pipeline.md` | Trajectory → controller → OFFBOARD command flow | TODO |
| T4 | `T4_sensor_fusion.md` | Position feedback, sensor fusion, heading sources | TODO |
| T5 | `T5_rpp_arc_controller.md` | RPP (Rover Pure Pursuit) commander for arc/shape following | TODO |
| T6 | `T6_full_system_architecture.md` | End-to-end system: mission → trajectory → controller → motors → feedback | TODO |

## How to Use

1. Copy each task file content and paste into ChatGPT / Grok / Gemini
2. Collect all responses
3. Synthesize into architecture documentation in `docs/Architecture/`
4. Then start coding with clear goals

## Our Constraints

- **Hardware:** Jetson Orin, CubeOrangePlus (PX4 v1.16.2), UM982 dual-antenna RTK, 2WD differential
- **Software:** ROS2 Humble, MAVROS2, Python 3
- **OFFBOARD mode:** Jetson sends setpoints at 50Hz via `/mavros/setpoint_raw/local`
- **Coordinate frames:** All setpoints in FRAME_LOCAL_NED (1). PX4 rejects FRAME_BODY_OFFSET_NED (9). Body→NED transform done in ROS2 node.
- **Accuracy target:** ±2cm lateral for road marking, ±1cm for sports field marking
- **Speed target:** 0.3-0.4 m/s marking speed
- **Open firmware bugs:** P3 (reverse motion) not working in OFFBOARD, P4 (heading hold) not validated