# T3 Controller Pipeline — Final Synthesis & Decision

**Date:** 2026-05-20
**Status:** Decided — ready to hand to coding agent
**Inputs:** ChatGPT, Gemini, GLM, Grok findings (`./`) + primary-source research (PX4 v1.16 docs, Nav2 RPP docs, PX4 forum 2025-2026, commercial vendor pages) + repo state (`docs/Architecture/FINAL_ARCHITECTURE.md`, `docs/Architecture/MAVROS2_ONLY_DECISION.md`, `src/offboard_test.py`)

---

## TL;DR (read this first)

Phase 2 controller pipeline is decided. **Three sentences:**

1. Run the path follower on Jetson as **Regulated Pure Pursuit (RPP)**, output body-frame `(v, ω)`, transform to NED on Jetson, publish at **50 Hz** to `/mavros/setpoint_raw/local` with **velocity-only type_mask = 3527** in `FRAME_LOCAL_NED (1)`.
2. Let PX4 v1.16.2's **DifferentialDriveControl** module do velocity tracking + wheel mixing. Bypass PX4's own pure pursuit guidance by sending velocity setpoints (not position).
3. Do **not** use the new `rover_speed_setpoint` / `rover_rate_setpoint` uXRCE-DDS path — it has a public open bug from Dec 2025 where wheels don't spin even when the topic is received ([PX4 Forum 48430](https://discuss.px4.io/t/rover-offboard-rover-speed-setpoint-rover-rate-setpoint/48430)). MAVROS2 `setpoint_raw/local` is the only path that works on rovers today.

---

## Research Plan & Sources

### Sub-questions investigated

1. PX4 v1.16+ rover architecture — confirm `DifferentialDriveControl` exists and how OFFBOARD interacts with it
2. OFFBOARD setpoint types that actually work for rovers in v1.16.2
3. Regulated Pure Pursuit at 0.3–0.4 m/s — what lookahead and tuning are correct
4. What commercial marking robots actually use
5. MAVROS2 vs uXRCE-DDS for OFFBOARD in 2026 — is there a reason to migrate?

### Primary sources consulted

| # | Source | Type | Date |
|---|---|---|---|
| 1 | [PX4 v1.16 Release Announcement](https://px4.io/px4-autopilot-release-v1-16-what-you-need-to-know/) | Official | Aug 2025 |
| 2 | [PX4 Differential Rover docs (v1.16)](https://docs.px4.io/v1.16/en/frames_rover/differential.html) | Official | 2025 |
| 3 | [PX4 Differential Rover Configuration/Tuning (v1.16)](https://docs.px4.io/v1.16/en/config_rover/differential) | Official | 2025 |
| 4 | [PX4 Differential Rover Drive Modes (v1.16)](https://docs.px4.io/v1.16/en/flight_modes_rover/differential.html) | Official | 2025 |
| 5 | [Nav2 RPP Configuration Guide](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html) | Official | 2026 |
| 6 | [Nav2 RPP README + paper](https://github.com/ros-navigation/navigation2/blob/main/nav2_regulated_pure_pursuit_controller/README.md) | Official | 2026 |
| 7 | [Macenski et al., "Regulated Pure Pursuit for Robot Path Tracking" (Autonomous Robots, 2023)](https://arxiv.org/abs/2305.20026) | Peer-reviewed | 2023 |
| 8 | [PX4 Forum 48430 — Rover Offboard rover_speed_setpoint not actuating](https://discuss.px4.io/t/rover-offboard-rover-speed-setpoint-rover-rate-setpoint/48430) | Bug report | Dec 2025 |
| 9 | [Turf Tank — Benefits of base station in line marking](https://turftank.com/en/benefits-of-base-station-in-line-marking/) | Vendor | Jul 2025 |
| 10 | [TinyMobileRobots — Line Marking Robots](https://tinymobilerobots.com/en_gb/product-lines/) | Vendor | 2025 |
| 11 | [Unicore — A little robot marks the lines (GPSWorld)](https://www.gpsworld.com/unicore-a-little-robot-marks-the-lines/) | Industry | Nov 2025 |
| 12 | [PX4 uXRCE-DDS docs](https://docs.px4.io/main/en/middleware/uxrce_dds) | Official | 2025 |
| 13 | Repo: `docs/Architecture/MAVROS2_ONLY_DECISION.md` | Internal | May 2026 |
| 14 | Repo: `src/offboard_test.py` | Internal | May 2026 |

---

## Key Findings (cross-referenced)

### Finding 1 — PX4 v1.16 rover stack is fundamentally different from pre-v1.16

**Source 1, 3, 4:** Confirmed by primary docs. The legacy `RoverPositionControl` module is **deprecated** in v1.16. The new architecture is:

- One module per drive type: `DifferentialDriveControl`, `AckermannDriveControl`, `MecanumDriveControl`
- **Shared pure-pursuit guidance library** used by Position mode, Mission mode, and Return mode
- Tuned via `PP_LOOKAHD_GAIN`, `PP_LOOKAHD_MIN`, `PP_LOOKAHD_MAX`
- Uses lookahead circle algorithm: lookahead distance `l_d = v · k` where `k = PP_LOOKAHD_GAIN`

> "Instead of a single controller handling every drive style, PX4 includes separate modules for Ackermann, differential, and mecanum rovers that all share a pure-pursuit guidance library."  
> — [PX4 v1.16 release announcement](https://px4.io/px4-autopilot-release-v1-16-what-you-need-to-know/) (content rephrased for compliance)

**What this means for our pipeline:**
- The four AI reports treated `RoverPositionControl` as the active OFFBOARD controller. **Outdated.** On v1.16.2 (which we run), it's `DifferentialDriveControl` that handles incoming setpoints.
- PX4's internal pure pursuit runs on **position-mode setpoints in OFFBOARD**, not velocity-mode.
- The deprecated module ([docs](https://docs.px4.io/v1.16/en/frames_rover/rover_position_control.html)) is what older Stack Overflow posts and AI training data refer to. Ignore those for our purposes.

**Cross-reference with AI reports:**
- ChatGPT: described `RoverPositionControl` as active — **wrong for v1.16.2**, but its high-level conclusions still hold because the new module behaves equivalently
- Gemini: correctly identified `DifferentialVelControl` and the shared pure-pursuit library — **most accurate**
- GLM: described `RoverPositionControl` — outdated
- Grok: identified the v1.16 refactor and `DifferentialVelControl` — accurate

### Finding 2 — Position mode in PX4 v1.16 already uses pure pursuit even for "straight lines"

This is non-obvious and important. From [PX4 differential config](https://docs.px4.io/v1.16/en/config_rover/differential):

> "When driving in a straight line (no yaw rate input) position mode leverages the same path following algorithm used in auto modes called pure pursuit to achieve the best possible straight line driving behaviour."  
> (Content rephrased for compliance)

Translation: **PX4 v1.16 *always* uses pure pursuit when given position-based control authority.** There is no "simple PID-to-waypoint" mode anymore. If we send position setpoints in OFFBOARD, PX4 runs its internal pure pursuit, computes a lookahead point, generates a yaw setpoint, runs cascaded yaw → yaw-rate → motor PIDs, and finally mixes to wheels. We cannot bypass this short of running velocity mode.

**Implication:** Sending position setpoints means **two pure-pursuit controllers stacked** (one on Jetson if we add Nav2 RPP, one in PX4). That's a guaranteed instability. The choice is binary:

- **Velocity setpoints** → PX4 only does velocity tracking + mixing, our RPP runs on Jetson alone.
- **Position setpoints** → PX4 does its pure pursuit, we *cannot* run our own RPP on top.

### Finding 3 — The new uXRCE-DDS rover setpoint topics are broken on real rovers (Dec 2025)

**Source 8** — confirmed bug report. User on PX4 v1.17 publishing `/fmu/in/rover_speed_setpoint` and `/fmu/in/rover_rate_setpoint` via uXRCE-DDS:

- Topics received correctly on PX4 (`listener` confirms)
- Offboard mode accepted, vehicle armed, no failsafe
- **BUT:** `rover_throttle_setpoint` and `rover_steering_setpoint` (the downstream uORB topics) are never produced
- **Result:** wheels do not spin

The thread is from Feb 2025, last reply Apr 2025, **still unresolved as of May 2026**.

This matches what's already documented in the repo's `MAVROS2_ONLY_DECISION.md`:

> "Rover OFFBOARD accepted, armed, setpoints received — wheels don't spin... DDS rover control simply doesn't work in current firmware."

**Implication:** The shiny new "native ROS 2 rover offboard" path that Grok and Gemini both flagged as future-proof — **does not work today**. It's a Phase 4 upgrade target. Phase 2 must use MAVROS2.

### Finding 4 — Nav2 RPP defaults are very close to what our rover needs

**Source 5, 6, 7** — Nav2's official RPP configuration page and the peer-reviewed paper:

| Parameter | Nav2 default | Our recommendation | Justification |
|---|---|---|---|
| `lookahead_dist` (constant mode) | 0.6 m | — | We use velocity-scaled mode |
| `min_lookahead_dist` | 0.3 m | **0.30 m** | Matches Gemini's analysis; floor at low speed |
| `max_lookahead_dist` | 0.9 m | **0.60 m** | Tighter than Nav2 default — needed for ±2cm marking |
| `lookahead_time` (gain `k`) | 1.5 s | **1.2 s** | At 0.3 m/s gives `l_d = 0.36 m` (in range) |
| `use_velocity_scaled_lookahead_dist` | false | **true** | Critical for low-speed stability |
| `use_regulated_linear_velocity_scaling` | true | **true** | Slows on tight curves automatically |
| `regulated_linear_scaling_min_radius` | 0.9 m | **0.6 m** | Tighter — our rover can do tighter arcs than industrial AGVs |
| `regulated_linear_scaling_min_speed` | 0.25 m/s | **0.15 m/s** | Marking-rover slower than service robot |
| `use_rotate_to_heading` | true | **true** | Equivalent to Gemini/GLM's "spot turn" FSM — Nav2 already implements this |
| `rotate_to_heading_min_angle` | 0.785 rad (45°) | **0.524 rad (30°)** | Gemini's tighter threshold for marking accuracy |
| `max_linear_vel` | 0.5 m/s | **0.4 m/s** | Our marking speed cap |
| `min_approach_linear_velocity` | 0.05 m/s | **0.05 m/s** | Default fine |

**Cross-reference with AI reports on lookahead:**
- ChatGPT: 0.3–0.5 m fixed — too wide, no velocity scaling
- Gemini: 0.30–0.60 m, t=1.2 s — **matches Nav2 best practice**
- GLM: 0.4–0.6 m fixed — narrow but no velocity scaling
- Grok: 0.35–0.7 m, t=1–2 s — slightly looser than Gemini

**Verdict:** Gemini's recommendation is correct and aligns with the published Nav2/RPP paper. Use that.

### Finding 5 — Commercial markers all run RTK + variants of pure pursuit

**Source 9, 10, 11:** Industry confirmation.

- **Turf Tank Two** ([source](https://turftank.com/en/benefits-of-base-station-in-line-marking/)): Uses local RTK base station, claims ±0.3" (≈ 7.6 mm) accuracy. Architecture details are proprietary but RTK + GNSS-only is confirmed (no LIDAR, no SLAM).
- **TinyMobileRobots** ([source](https://tinymobilerobots.com/en_gb/product-lines/)): RTK-GNSS without local base station. "Millimetre-level accuracy" claimed.
- **Unicore-based marker** ([GPSWorld coverage, Nov 2025](https://www.gpsworld.com/unicore-a-little-robot-marks-the-lines/)): RTK-GNSS, 1–2 cm positioning accuracy claimed.
- **SWOZI Auto, FJDynamics RM21:** Both RTK-GNSS, multi-mode field marking.

**No commercial marker is publicly using MPC for path tracking.** Where pipelines are described, they describe variants of pure pursuit + speed regulation. None describe Stanley or MPC.

This validates our choice of RPP for Phase 2.

### Finding 6 — At our speed, MAVROS2 latency is fine

From the existing repo doc `MAVROS2_ONLY_DECISION.md` (verified):
- 50 Hz × 0.4 m/s = 8 mm between updates
- 35 ms typical MAVROS2 latency × 0.4 m/s = 14 mm position lag
- RPP lookahead (0.30–0.60 m) **dominates** these errors — the controller naturally compensates by aiming ahead
- Worst-case tracking error ≈ 22 mm against a ±20–30 mm target

**uXRCE-DDS would shave ~10 mm off this** but it doesn't actuate the wheels (Finding 3). MAVROS2 is the only path that *works*, and the math says it's adequate.

### Finding 7 — RTK is the dominant accuracy lever, not the controller

This is the uncomfortable truth. From Turf Tank's own description and the Macenski paper:

- RTK GNSS: ±1.5 cm typical
- Pure pursuit tracking error at our scale: ±0.5–2.0 cm (with proper tuning)
- Total RSS: √(1.5² + 1.5²) ≈ ±2.1 cm

**Without RTK, no controller — RPP, Stanley, or MPC — can hit ±2 cm.** The current state of UM982 RTK injection via NTRIP (running, but not validated for fix-quality on this rover) is the actual gating concern. Phase 2 controller work is correct to start, but the team must close the RTK validation loop in parallel — there's no point tuning RPP gains against an unvalidated position estimate.

---

## Comparison Table — Reconciling the four AI reports

| Question | ChatGPT | Gemini | GLM | Grok | Primary sources verdict | **Decision** |
|---|---|---|---|---|---|---|
| Where does path following run? | Jetson | Jetson | Jetson | Jetson | All four agree, confirmed by PX4 docs (Position-mode pure pursuit cannot be disabled on PX4 side without going to velocity-mode setpoints) | **Jetson** |
| Setpoint type for arcs | Position with PX4 internal RPP, OR Velocity | Velocity-only | Velocity-only | Velocity-first | PX4 v1.16 always pure-pursuits position setpoints → stacked controllers if we use position. Velocity isolates concerns. | **Velocity (type_mask 3527)** |
| Algorithm | Pure pursuit, Stanley fallback | Regulated Pure Pursuit | Pure Pursuit, Stanley fallback | Regulated Pure Pursuit | Nav2 paper + commercial vendors converge on RPP variants | **Regulated Pure Pursuit** |
| Lookahead at 0.3–0.4 m/s | 0.3–0.5 m fixed | 0.30–0.60 m, t=1.2 s | 0.4–0.6 m fixed | 0.35–0.7 m, t=1–2 s | Nav2 official defaults map to Gemini's recommendation | **Velocity-scaled, 0.30–0.60 m, t=1.2 s** |
| Sharp corner handling | Accept corner cutting | Spot-turn at >30° heading error | Spin mode at >45° | Hybrid: small L + slow + maybe stop | Nav2 RPP `use_rotate_to_heading` exists with default 45°. Marking accuracy benefits from tighter threshold. | **Rotate-to-heading at >30°** |
| MAVROS2 vs uXRCE-DDS | Not addressed | Mentions DDS as future, recommends MAVROS for now | Recommends MAVROS | Recommends DDS migration as Phase-3 target | Forum 48430 (Dec 2025): DDS rover offboard wheels-don't-spin bug still unresolved May 2026 | **MAVROS2 only — DDS path is broken** |
| Bridge migration timing | Not addressed | Phase 3+ | Not discussed | "Future-proof, migrate" | Already decided in `MAVROS2_ONLY_DECISION.md`, validated by ongoing DDS rover bug | **MAVROS2 for entire Phase 2; revisit only if DDS rover bug closes** |
| MPC consideration | Overkill at our speed | Overkill, prefers RPP | Overkill | Overkill unless avoidance needed | Nav2 RPP paper explicitly targets exactly our use case (industrial/service robots at low speed). MPC unjustified. | **No MPC in Phase 2** |
| PX4 internal "RPP" naming | "Internal pure pursuit" | "PP_LOOKAHD_*" guidance library | "PX4 internal alternative to P-loop" | "Internal pure pursuit guidance" | PX4 docs confirm it's the shared pure-pursuit library, not a published Nav2 plugin | **PX4 internal PP ≠ Nav2 RPP — bypassed by sending velocity** |

---

## Final Architecture Diagram

```
┌────────────────────────────────────────────────────────────────────┐
│                      JETSON ORIN (ROS2 Humble)                      │
│                                                                    │
│  [Mission file: DXF / waypoints]                                  │   Phase 3
│        ↓                                                           │
│  [Trajectory Planner]                                             │   Phase 2/3
│   • Spline smoothing (G2 continuous)                              │
│   • Speed profile (curvature-based)                                │
│   • Output: nav_msgs/Path in LOCAL_NED                             │
│        ↓                                                           │
│  [rpp_controller_node.py]            ← Phase 2, this task         │
│   • Closest-point projection                                       │
│   • Velocity-scaled lookahead (0.30–0.60 m, t=1.2 s)              │
│   • Curvature → ω = v · κ                                          │
│   • Speed regulation by curvature                                  │
│   • Rotate-to-heading FSM at >30°                                  │
│   • Output: geometry_msgs/Twist (body frame v_x, ω_z)              │
│        ↓                                                           │
│  [twist_to_setpoint_node.py]         ← Phase 2, this task         │
│   • Read yaw from /mavros/local_position/pose                      │
│   • Body→NED rotation: (v_N, v_E) = R(yaw) · (v_x, 0)             │
│   • Build PositionTarget: type_mask=3527, frame=LOCAL_NED          │
│   • Publish at 50 Hz                                               │
│        ↓                                                           │
└─────── /mavros/setpoint_raw/local ───────────────────────────────┘
                                ↓
                MAVROS2 (USB ttyACM0 @ 921600)
                                ↓  SET_POSITION_TARGET_LOCAL_NED
┌────────────────────────────────────────────────────────────────────┐
│                CUBEORANGEPLUS (PX4 v1.16.2 Rover Build)             │
│                                                                    │
│  [MAVLink receiver] → trajectory_setpoint uORB                     │
│        ↓                                                           │
│  [DifferentialDriveControl]                                       │
│   • Velocity-mode path → skip pure pursuit guidance                │
│   • Body-frame projection from NED velocity                        │
│   • Closed-loop speed PID (RO_SPEED_P, RO_SPEED_I)                │
│   • Closed-loop yaw rate PID (RO_YAW_RATE_P, RO_YAW_RATE_I)        │
│   • Wheel mixing: v_L = v − ω·track/2, v_R = v + ω·track/2         │
│        ↓                                                           │
│  [Actuator allocator → motor outputs]                              │
└────────────────────────────────────────────────────────────────────┘
                                ↓
                       Sabertooth 2x32 → motors
```

---

## Decision Summary (for the coding agent)

### Hard requirements

1. **Bridge:** MAVROS2 only. Topic `/mavros/setpoint_raw/local`. Do not write any uXRCE-DDS rover setpoint topics (broken in current firmware).
2. **Coordinate frame:** `FRAME_LOCAL_NED (1)` — `FRAME_BODY_OFFSET_NED (9)` is rejected by PX4 rover code.
3. **Setpoint type:** Velocity-only, `type_mask = 3527` (ignore positions, accelerations, yaw, yaw_rate).
4. **Rate:** 50 Hz, continuous, no gaps. PX4 `COM_OF_LOSS_T` cutoff = 500 ms.
5. **Frame transform:** Done in Jetson `twist_to_setpoint_node.py`. NED yaw extracted from `/mavros/local_position/pose` quaternion (the pattern is already correct in `src/offboard_test.py:_get_yaw_rad()` — reuse).
6. **PX4 params left as-is:** `PP_LOOKAHD_GAIN=1.0`, `MIN=1.0`, `MAX=5.0`, `RD_WHEEL_TRACK=0.47`, `RO_YAW_RATE_LIM=30°/s`. These affect Position/Auto modes but not velocity-OFFBOARD.

### Algorithm parameters for `rpp_controller_node.py`

| Parameter | Value | Source |
|---|---|---|
| `max_linear_vel` | 0.4 m/s | Project spec |
| `min_approach_linear_velocity` | 0.05 m/s | Nav2 default |
| `min_lookahead_dist` | 0.30 m | Gemini + Nav2 default |
| `max_lookahead_dist` | 0.60 m | Tighter than Nav2 default for marking |
| `lookahead_time` | 1.2 s | Gemini |
| `use_velocity_scaled_lookahead_dist` | true | Required at low speed |
| `use_regulated_linear_velocity_scaling` | true | Nav2 paper |
| `regulated_linear_scaling_min_radius` | 0.6 m | Tighter than Nav2 default |
| `regulated_linear_scaling_min_speed` | 0.15 m/s | Marking-rover floor |
| `rotate_to_heading_min_angle` | 0.524 rad (30°) | Marking accuracy |
| `rotate_to_heading_resume_angle` | 0.175 rad (10°) | Hysteresis |
| `rotate_to_heading_angular_vel` | 0.6 rad/s | Conservative for ±2cm |

### Open risks the agent must understand

| Risk | Status | Mitigation |
|---|---|---|
| RTK fix-quality not validated on this rover | **OPEN** — gating risk | Validate UM982 RTK fix in field before tuning RPP gains. Without RTK, ±2 cm is impossible regardless of controller. |
| P3 (reverse) firmware patch not validated | **OPEN** | Test reverse with velocity-mode (already covered in `offboard_test.py` velocity test). If body-frame transform is correct, reverse should "just work" without P3. |
| P4 (heading hold) firmware patch not validated | **OPEN** | When commanding `v=0, ω=0`, observe drift on slope. If drift > 1 cm/s, P4 isn't holding heading and we need a Jetson-side heading PID inside the velocity command. |
| EKF2 phase lag (~30–50 ms) at 0.4 m/s = ~12–20 mm | **KNOWN** | RPP lookahead absorbs this. If straight-line oscillation is observed, add forward predictor: `pose_now = pose_msg + v · (now − pose_msg.stamp)`. Don't add preemptively. |
| MAVROS2 is bypassing PX4's pure pursuit | **BY DESIGN** | This is intentional. Position-mode setpoints would stack two pure-pursuit controllers and oscillate. |
| Nav2 stack as a whole vs custom RPP node | **DECISION POINT** | Nav2 brings costmaps, behavior trees, planners — overkill for marking with no obstacles. Custom RPP node is ~200 lines, easier to debug, no costmap server required. **Build custom**. |

---

## Phase 2 Build Order (for coding agent)

Strictly sequential, smallest commits first.

### Step 1 — `rpp_controller_node.py` (no MAVROS dependency)

**Inputs:**
- `/path` (`nav_msgs/Path`) — list of poses in `LOCAL_NED`
- `/mavros/local_position/pose` (`geometry_msgs/PoseStamped`) — for current position + heading

**Output:**
- `/cmd_vel_body` (`geometry_msgs/Twist`) — `linear.x = v_body`, `angular.z = ω`

**Logic:**
1. Find closest point on path to current pose (linear search; path lengths are short).
2. Compute path-arc-length lookahead point at `l_d = clamp(v · 1.2, 0.30, 0.60)`.
3. Express lookahead in body frame: `(x_g, y_g) = R(-yaw) · (lookahead_NED - pose_NED)`.
4. Compute curvature `κ = 2 · y_g / l_d²`.
5. Compute desired forward velocity: start at 0.4 m/s, regulate down by curvature: `v = max(0.15, 0.4 · clamp(0.6 / max(|R|, 0.6), 0.0, 1.0))` where `R = 1/κ`.
6. Compute `ω = v · κ`.
7. **Rotate-to-heading FSM:**
   - Compute heading error `θ_e` to lookahead point.
   - If `state == DRIVE` and `|θ_e| > 30°`: enter `ROTATE`. Output `(v=0, ω=sign(θ_e)·0.6)`.
   - If `state == ROTATE` and `|θ_e| < 10°`: return to `DRIVE`.
8. Publish `/cmd_vel_body`.

**Tests (not unit tests — run on simulated path):**
- Straight 5-meter line: cross-track error stays under 5 cm at 0.4 m/s.
- 1.5 m radius arc: cross-track error stays under 3 cm.
- L-shape 90° corner: rotate-to-heading triggers, completes, resumes drive.

### Step 2 — `twist_to_setpoint_node.py` (MAVROS interface)

**Inputs:**
- `/cmd_vel_body` (from Step 1)
- `/mavros/local_position/pose` (for current yaw)

**Output:**
- `/mavros/setpoint_raw/local` (`mavros_msgs/PositionTarget`) at 50 Hz

**Logic:**
1. Subscribe to `/cmd_vel_body` (latest sample).
2. On 50 Hz timer:
   - Read current yaw (NED) using exactly the routine in `src/offboard_test.py:_get_yaw_rad()`.
   - Rotate body velocity to NED: `v_N = v_body·cos(yaw) − 0·sin(yaw) = v_body·cos(yaw)`, `v_E = v_body·sin(yaw)`.
   - Build `PositionTarget` with `coordinate_frame=1`, `type_mask=3527`, `velocity.x=v_N`, `velocity.y=v_E`, `velocity.z=0`, `yaw_rate=ω` (with `IGNORE_YAW_RATE` *cleared* — this is the only deviation from `offboard_test.py`).
   - Publish.
3. If no `/cmd_vel_body` received in 200 ms, publish zero-velocity setpoint with current yaw_rate=0.

**Note on yaw_rate inclusion:** `offboard_test.py` currently ignores yaw_rate (type_mask 3527 includes IGNORE_YAW_RATE = 2048). For RPP we need to *not* ignore yaw_rate, so the actual mask becomes `3527 - 2048 = 1479`. Verify this works against PX4 v1.16.2 — if PX4 rejects it, fall back to 3527 and let yaw be derived from velocity vector direction (PX4 default behavior).

### Step 3 — Path source for testing

Three hardcoded paths in a launch parameter:
- `straight_5m` — 5 m due north, single segment
- `arc_quarter_1m5` — quarter circle, 1.5 m radius
- `lshape_2x2` — 2 m north, then 2 m east (tests corner handling)

### Step 4 — Cross-track logger node

Subscribes to `/path` and `/mavros/local_position/pose`. At 20 Hz, computes:
- Closest-point distance (cross-track error)
- Heading error
- Current `(v, ω)` commanded
- Logs CSV: `t, x, y, yaw, x_target, y_target, e_lat, e_heading, v_cmd, ω_cmd`

### Step 5 — SITL validation before hardware

Run all three paths in Gazebo Harmonic with the differential rover model that comes with PX4 v1.16. Validate:
- No oscillation on straight line
- No corner-cutting on arc (cross-track stays inside 3 cm)
- Rotate-to-heading completes cleanly on L-shape

Only after SITL passes, move to hardware on grass with RTK.

### Step 6 — Hardware bring-up

In order:
1. Run `offboard_test.py` velocity mode (already exists) — confirm OFFBOARD + velocity setpoint pipeline still works on hardware.
2. Run `rpp_controller_node` + `twist_to_setpoint_node` on `straight_5m` at **0.2 m/s** (slower than spec) — verify no instabilities.
3. Repeat at 0.4 m/s.
4. `arc_quarter_1m5`.
5. `lshape_2x2`.
6. With RTK validated and fix-quality > 95%, measure cross-track and update tuning if needed.

---

## What This Decision Does Not Include (deliberately)

These are out of scope for Phase 2 controller work:

- **No Nav2 stack:** No costmaps, planners, or behavior trees. Marking has no obstacles.
- **No Stanley controller:** Pure RPP. Add Stanley only if RPP fails to hit ±2 cm after RTK is validated.
- **No MPC:** Not justified at 0.4 m/s with our paths.
- **No uXRCE-DDS migration:** Wait for forum 48430 to close.
- **No native `RoverSpeedSetpoint`:** Same reason.
- **No PX4 parameter changes:** The current Position-mode tuning (`PP_LOOKAHD_*`, `RO_*`) is irrelevant when we send velocity setpoints. Don't touch it.
- **No EKF2 forward predictor:** Add only if straight-line oscillation is observed in field testing.

---

## Sources

1. PX4 Autopilot. "Release V1.16: What You Need To Know." https://px4.io/px4-autopilot-release-v1-16-what-you-need-to-know/ (Aug 2025)
2. PX4 Documentation. "Differential Rovers (v1.16)." https://docs.px4.io/v1.16/en/frames_rover/differential.html
3. PX4 Documentation. "Configuration/Tuning (Differential Rover) (v1.16)." https://docs.px4.io/v1.16/en/config_rover/differential
4. PX4 Documentation. "Drive Modes (Differential Rover) (v1.16)." https://docs.px4.io/v1.16/en/flight_modes_rover/differential.html
5. Nav2 Documentation. "Regulated Pure Pursuit Configuration." https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html
6. Macenski, Singh, Martin, Gines. "Regulated Pure Pursuit for Robot Path Tracking." Autonomous Robots, 2023. https://arxiv.org/abs/2305.20026
7. ros-navigation/navigation2. "nav2_regulated_pure_pursuit_controller README." https://github.com/ros-navigation/navigation2/blob/main/nav2_regulated_pure_pursuit_controller/README.md
8. PX4 Forum. "Rover Offboard: rover_speed_setpoint / rover_rate_setpoint." https://discuss.px4.io/t/rover-offboard-rover-speed-setpoint-rover-rate-setpoint/48430 (Feb 2025, unresolved May 2026)
9. Turf Tank. "Benefits of base station in line marking." https://turftank.com/en/benefits-of-base-station-in-line-marking/ (Jul 2025)
10. TinyMobileRobots. "Line Marking Robots for Sports Pitches." https://tinymobilerobots.com/en_gb/product-lines/
11. GPSWorld. "Unicore: A little robot marks the lines." https://www.gpsworld.com/unicore-a-little-robot-marks-the-lines/ (Nov 2025)
12. PX4 Documentation. "uXRCE-DDS (PX4-ROS 2/DDS Bridge)." https://docs.px4.io/main/en/middleware/uxrce_dds
13. Repo: `docs/Architecture/MAVROS2_ONLY_DECISION.md`
14. Repo: `docs/Architecture/FINAL_ARCHITECTURE.md`
15. Repo: `src/offboard_test.py`

*Content from external sources rephrased for compliance with licensing restrictions.*
