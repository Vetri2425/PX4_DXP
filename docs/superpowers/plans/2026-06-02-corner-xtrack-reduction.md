# Corner Xtrack Reduction — RPP Upgrade Path Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce corner cross-track error from the validated baseline of 9.4cm (log 59) to ≤5cm using mechanisms already present in the codebase, then build the robot_localization fusion upgrade path as Phase 2.

**Architecture:** Extend existing Regulated Pure Pursuit (RPP) — no MPC, no architecture replacement. The controller already has feedforward yaw rate (P3.1, κ·v), predictive curvature regulation (P1.1), and corner path smoothing (P1.3). Sprint 1 validates and tunes these. Sprint 2 adds `robot_localization` wheel-odom fusion for smoother pose input.

**Tech Stack:** Python 3.10, ROS2 Humble, MAVROS2, PX4 v1.16.2 OFFBOARD, UM982 RTK, Jetson Orin. Params live in rpp_pipeline.launch.py + runtime `ros2 param set`. Bags analyzed on laptop.

**Ownership model:**
- Laptop (this session): param file edits, analysis scripts, plan/launch edits, this document.
- Jetson: service restarts, field runs, rosbag capture. SSH: `ssh flash@192.168.1.102`.

**Baseline (log 59, 2026-05-23):** max xtrack 9.4cm corners, 1-3cm straights. That older run used P3.1 yaw-rate feedforward with `yaw_rate_feedback_gain=1.2`. Current mainline defaults to pure feedforward (`yaw_rate_feedback_gain=0.0`) with `max_yaw_rate_body=0.45`.

---

## Diagnostic: Why 9.4cm at corners?

The three mechanisms at play and their failure modes:

| Mechanism | Param | Current value | What goes wrong at corners |
|---|---|---|---|
| **P3.1 FF yaw rate** (κ·v) | `yaw_rate_feedback_gain` | 0.0 current mainline; 1.2 in old log 59 baseline | If k_ψ·θ_e over-commands, the rover yaws past tangent → oscillates out the back of the corner |
| **P1.1 Predictive κ** | `preview_curvature_n` | 4 | If speed doesn't drop early enough, lateral slip puts rover outside the arc |
| **P1.3 Corner smoothing** | `corner_smooth_radius_m` | 0.5m | Smoothed arc may still demand curvature beyond what a_lat_max constrains the speed to |
| **P4.1 Lateral accel** | `a_lat_max` | 0.3 m/s² | At R=0.5m (κ=2), speed floor 0.3m/s gives v²/R = 0.18 m/s² — under budget, but barely |

Corner xtrack is dominated by **yaw lag** (P3.1 k_ψ too high → overshoot) or **speed too high into corner** (P1.1 preview window too short or a_lat_max too generous).

---

## Sprint 1 — Tune existing RPP to ≤5cm corners (field-testable in hours)

**Entry condition:** Current code on Jetson has the 39-field `/rpp/debug` layout, `/rpp/yaw_rate_body`, and current `twist_to_setpoint_node.py` yaw handling deployed.

### Task 1: Deploy and verify the current RPP runtime

**Files:**
- `src/rpp_controller_node.py`
- `src/twist_to_setpoint_node.py`
- `server/config.py` / server telemetry consumers

- [ ] **Step 1.1 — Verify working tree / commit state on laptop**

```bash
cd D:/Vetri/3WD_GCS/PX4_DXP
git status --short
```

- [ ] **Step 1.2 — Push and deploy on Jetson**

```bash
# Jetson:
cd ~/PX4_DXP && git pull
sudo systemctl restart rpp-pipeline
journalctl -u rpp-pipeline -n 20 --no-pager
```

Expected: `RPP controller started` log line visible within 3s.

- [ ] **Step 1.3 — Verify debug array size**

```bash
# Jetson:
ros2 topic echo /rpp/debug --once | grep -E "size|stride"
```

Expected: `size: 39`, `stride: 39`.

- [ ] **Step 1.4 — Verify yaw-rate path**

```bash
ros2 topic echo /rpp/yaw_rate_body --once
ros2 topic echo /mavros/setpoint_raw/local --once | grep -E "type_mask|yaw|yaw_rate"
```

Expected: type_mask `2503` while stopped or zero yaw-rate; type_mask `455` while tracking a curve with fresh nonzero yaw-rate.

---

### Task 2: Baseline corner bag — capture before tuning

Capture a bag with the current params so Sprint 1 tuning has a clean before/after comparison.

**IMPORTANT:** The 9.4cm baseline (log 59) came from a `square_2x2` run — 90° corners smoothed to R=0.5m arcs by corner_smooth_radius_m. Use `square_2x2` for all baseline and validation runs, not `arc_quarter_1m5`. The quarter-arc path is a single R=1.5m gentle arc: it never exercises the sharp-corner machinery and cannot be compared to the log 59 number.

- [ ] **Step 2.1 — Run square_2x2 with current params, capture bag**

```bash
# Jetson (two terminals):
# Terminal 1 — start bag:
ros2 bag record /rpp/debug /mavros/local_position/pose /mavros/state \
    -o ~/bags/corner_baseline_$(date +%Y%m%d_%H%M%S)

# Terminal 2 — launch (manual mission start):
cd ~/PX4_DXP
ros2 launch src/launch/rpp_pipeline.launch.py path_name:=square_2x2 \
    use_feedforward_yaw_rate:=true yaw_rate_feedback_gain:=0.0 log_level:=debug
```

- [ ] **Step 2.2 — Extract corner xtrack from bag (laptop)**

```bash
# Laptop — after scp of bag:
python3 - <<'EOF'
import sqlite3, glob, struct, math

bag_path = "~/bags/corner_baseline_*"   # adjust to actual path
# Parse /rpp/debug[0] (xtrack) and [7] (state_code) from the bag's .db3
# State TRACKING=1, APPROACH=2; filter to corner segments where |xtrack| is largest
EOF
```

Simpler: use `ros2 bag play` + `ros2 topic echo /rpp/debug` on Jetson and read [0] field.

Max |debug[0]| during state=TRACKING is your corner xtrack number.

---

### Task 3: Sweep yaw_rate_feedback_gain — find the square-corner optimum

**Context from git history:** the 9.4cm log 59 baseline was at k_ψ=1.2. Current mainline default is k_ψ=0.0, so the first field run should establish a new square-corner baseline before adding feedback back in.

Observation model: watch `debug[10]` (yaw_rate_cmd_rad_s) and `debug[0]` (xtrack_m) simultaneously. At corners, if the rover cuts inside the arc = gain too high (overshoot into interior). If it swings wide and corrects late = gain too low (insufficient correction).

- [ ] **Step 3.1 — Test k_ψ = 0.0 (current mainline)**

```bash
# Jetson — while RPP is running square_2x2:
ros2 param set /rpp_controller yaw_rate_feedback_gain 0.0
```

Record peak |debug[0]| at corner. This is the new current-code baseline.

- [ ] **Step 3.2 — Test k_ψ = 0.6 (moderate feedback)**

```bash
ros2 param set /rpp_controller yaw_rate_feedback_gain 0.6
```

Record peak |debug[0]|. Compare to 0.0 and the old 1.2 baseline.

- [ ] **Step 3.3 — Test k_ψ = 0.3**

```bash
ros2 param set /rpp_controller yaw_rate_feedback_gain 0.3
```

- [ ] **Step 3.4 — Record winning value, update launch file with the best**

```python
# Use runtime param set for field tuning. If a nonzero value clearly wins,
# update src/rpp_controller_node.py default or pass it through launch/systemd.
```

Commit: `git commit -m "tune(rpp): yaw_rate_feedback_gain X.X — best corner xtrack on square_2x2"`

---

### Task 4: Tighten predictive curvature preview window (P1.1)

`preview_curvature_n=4` looks ahead 4×L_d. At L_d=0.8m and 0.5m/s that's 3.2m — farther than a typical corner entry. But if `a_lat_max` is too generous, the regulated speed doesn't slow enough.

- [ ] **Step 4.1 — Lower a_lat_max to force earlier speed reduction**

```bash
# Default is 0.3 m/s². Try 0.2:
ros2 param set /rpp_controller a_lat_max 0.2
```

At R=0.5m (κ=2): `v = sqrt(0.2/2) = 0.316 m/s` (was 0.387 m/s). This is a 18% speed reduction at corner entry, which should directly reduce lateral displacement.

Floor speed `regulated_linear_scaling_min_speed=0.3` is still respected.

- [ ] **Step 4.2 — Cross-check with corner_smooth_radius_m**

At R=0.5m corner smoothing, κ_max = 1/0.5 = 2.0. With a_lat_max=0.2: v_corner = 0.316 m/s. Lateral accel = v²·κ = 0.1 × 2.0 = 0.2 m/s² = exactly on budget. ✓

If corner xtrack improves but straight speed is impacted unacceptably, try a_lat_max=0.25 as a compromise.

**Note:** `a_lat_max` is NOT in the launch file's forward list (only `min_lookahead_dist`, `max_lookahead_dist` etc. are forwarded). It can only be set via `ros2 param set` at runtime or by adding a new `DeclareLaunchArgument` entry. Runtime `ros2 param set` is sufficient for tuning — no restart needed.

- [ ] **Step 4.3 — Record best a_lat_max, add to param file**

```bash
# Laptop: update Param/14-05-2026/First_Best.param or create a new dated param file
# Document: a_lat_max=X.X, corner_xtrack_cm=Y.Y (from bag)
```

---

### Task 5: Validate corner xtrack ≤5cm — go/no-go gate

- [ ] **Step 5.1 — Run square_2x2 with tuned params, capture validation bag**

```bash
# Jetson:
ros2 bag record /rpp/debug /mavros/local_position/pose /mavros/state \
    -o ~/bags/corner_tuned_$(date +%Y%m%d_%H%M%S)
```

- [ ] **Step 5.2 — Check max |debug[0]| during TRACKING state**

Pass criterion: `max(|xtrack|) < 0.05 m` during corner traversal.

- [ ] **Step 5.3 — Commit tuned params**

```bash
git add src/launch/rpp_pipeline.launch.py
git commit -m "tune(rpp): corner xtrack <5cm — a_lat_max=X.X, k_psi=X.X"
```

**If xtrack is still ≥5cm:** Proceed to Task 6 (corner path smoothing audit) before declaring Phase 2.

---

### Task 6 (conditional): Corner smoothing radius audit

Only needed if Task 5 gate fails.

- [ ] **Step 6.1 — Check corner_smooth_arc_pts is sufficient**

At `corner_smooth_radius_m=0.5` and `arc_pts=6`, the arc is discretized into 6 points. The chord error between adjacent points is `r·(1 - cos(π/n)) ≈ 0.5 × 0.021 = 1 cm` for n=6 — acceptable but at the edge. Try `arc_pts=10` to smooth the discretization artifact.

```bash
ros2 param set /rpp_controller corner_smooth_arc_pts 10
```

- [ ] **Step 6.2 — Check path_resample_spacing_m gives enough κ samples**

`path_resample_spacing_m=0.08` at corner_r=0.5 gives ~39 points per quarter arc — sufficient for predictive κ. If your DXF waypoints are sparse (>0.5m apart), resample will densify them. Verify with the path conditioning log:

```bash
journalctl -u rpp-pipeline -n 30 | grep "Path conditioned"
```

---

## Sprint 2 — robot_localization wheel-odom fusion (Phase 3, deferred pre-condition)

**Entry condition:** Sprint 1 complete (corner xtrack ≤5cm with RTK-only). STM32 encoder bridge to Jetson must be physically wired and publishing `/wheel_odom` before this sprint begins.

**Why deferred:** EKF2 alone already hits the ±3cm budget on straights (log 59). robot_localization adds value only if RTK dropout occurs (>5s gap), arc error exceeds 3cm despite Sprint 1 tuning, or high-speed (>1.0 m/s) operation is needed where GPS latency becomes the dominant xtrack driver.

**Architecture (per your diagram):**

```
UM982 RTK → PX4 EKF2 (50Hz) → /mavros/local_position/pose (ENU)
CubeOrangePlus IMU → PX4 EKF2 ─────────────────────────────────┐
                                                                  ↓
AMT102 encoder → STM32 bridge → /wheel_odom (nav_msgs/Odometry)
NHC virtual sensor (lateral_vel=0)                               ↓
                              → Jetson robot_localization (20Hz) → /odom
                                                                  ↓
                                                         RPP uses /odom
```

### Task 7: Build the robot_localization node

**Files (Jetson-side):**
- Create: `~/PX4_DXP/src/localization_node.py`
- Modify: `~/PX4_DXP/config/robot_localization.yaml`
- Modify: `~/PX4_DXP/src/launch/rpp_pipeline.launch.py` (add localization_node)
- Modify: `~/PX4_DXP/src/rpp_controller_node.py` (add `/odom` subscriber, keep `/mavros/local_position/pose` as fallback)

**Dependencies (Jetson):**
```bash
sudo apt install ros-humble-robot-localization
```

- [ ] **Step 7.1 — Create robot_localization config**

```yaml
# ~/PX4_DXP/config/robot_localization.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 20.0
    sensor_timeout: 0.1
    two_d_mode: true           # rover is ground vehicle
    publish_tf: false          # do not fight MAVROS TF
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: /mavros/local_position/pose    # EKF2 position (high accuracy, lower freq)
    odom0_config: [true,  true,  false,  # x, y, z position
                   false, false, true,   # roll, pitch, yaw
                   false, false, false,  # vx, vy, vz
                   false, false, false,  # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 5

    odom1: /wheel_odom                    # Encoder odometry (smooth, high-freq)
    odom1_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,  # vx, vy only (NHC: vy=0 always)
                   false, false, false,
                   false, false, false]
    odom1_differential: false
    odom1_relative: true
    odom1_queue_size: 10
```

- [ ] **Step 7.2 — Add /odom subscriber to rpp_controller_node as opt-in fallback**

In `src/rpp_controller_node.py`, add a parameter `use_fused_odom` (default False) and a second pose subscriber on `/odom`. When `use_fused_odom=True`, prefer `/odom` over `/mavros/local_position/pose` for the projection step. Keep the MAVROS subscriber always active for the EKF jump guard.

```python
# In __init__, after existing subscribers:
self.declare_parameter("use_fused_odom", False)
from nav_msgs.msg import Odometry
self.create_subscription(Odometry, "/odom", self._odom_cb, be_qos)
self._fused_pose: PoseStamped | None = None
self._fused_recv_time: RclTime | None = None
```

```python
def _odom_cb(self, msg):
    """Accept fused /odom pose when use_fused_odom=True."""
    if not self.get_parameter("use_fused_odom").value:
        return
    ps = PoseStamped()
    ps.header = msg.header
    ps.pose = msg.pose.pose
    self._fused_pose = ps
    self._fused_recv_time = self.get_clock().now()
```

In `_control_loop`, replace the single `pose_for_projection = self._pose` line:

```python
if self.get_parameter("use_fused_odom").value and self._fused_pose is not None:
    fused_age = (self.get_clock().now() - self._fused_recv_time).nanoseconds * 1e-9
    if fused_age < max_age_s:
        pose_for_projection = self._fused_pose
    # else fall through to MAVROS pose (already set as default above)
```

- [ ] **Step 7.3 — Add localization_node to launch file**

```python
# src/launch/rpp_pipeline.launch.py — in _build():
loc_proc = ExecuteProcess(
    cmd=["ros2", "run", "robot_localization", "ekf_node",
         "--ros-args", "--params-file",
         os.path.join(src_dir, "..", "config", "robot_localization.yaml")],
    name="ekf_localization",
    output="screen",
)
actions.insert(0, loc_proc)   # start before RPP
```

- [ ] **Step 7.4 — Validate /odom is publishing and plausible**

```bash
# Jetson:
ros2 topic echo /odom --once
ros2 topic hz /odom   # expect ~20 Hz
```

Compare `/odom` position to `/mavros/local_position/pose` position — they should agree within 5cm during static (rover stationary).

- [ ] **Step 7.5 — Enable fused odom, run arc_quarter_1m5, capture bag**

```bash
ros2 param set /rpp_controller use_fused_odom true
```

Compare corner xtrack bag vs Sprint 1 baseline. Pass criterion: no regression on straights AND corner xtrack not worse than Sprint 1.

- [ ] **Step 7.6 — Commit robot_localization integration**

```bash
git add config/robot_localization.yaml src/launch/rpp_pipeline.launch.py src/rpp_controller_node.py
git commit -m "feat(loc): add robot_localization wheel-odom fusion, opt-in via use_fused_odom param"
```

---

## Official Documentation References

These are the canonical sources for each mechanism:

| Mechanism | Source |
|---|---|
| Nav2 Regulated Pure Pursuit | `https://nav2.ros.org/configuration/packages/configuring-regulated-pp.html` — lateral accel constraint, adaptive lookahead, predictive curvature are all validated Nav2 patterns |
| PX4 OFFBOARD velocity mode | PX4 dev docs > Offboard Control > `SET_POSITION_TARGET_LOCAL_NED` type_mask=455 |
| robot_localization EKF | `http://docs.ros.org/en/humble/p/robot_localization/` — `ekf_node`, sensor fusion config, `two_d_mode` |
| RTK/EKF2 accuracy | PX4 EKF2 tuning guide: `EKF2_GPS_POS_X/Y` lever arm, `EKF2_GPS_V_GATE` velocity innovation gate |
| NHC (Non-Holonomic Constraint) | Georgy et al. 2009 — add lateral velocity = 0 as a virtual sensor; improves dead-reckoning on ground vehicles by 40-60% per published benchmarks |

---

## Xtrack Kill Priority Summary

| Priority | Change | Estimated xtrack improvement | Effort | Field-testable? |
|---|---|---|---|---|
| **1** | Lower `yaw_rate_feedback_gain` from 1.2 → 0.3–0.6 (arc_fix_18 ran at 1.2, produced 9.4cm) | Likely −2 to −5cm at corners | 20 min | Yes, no restart |
| **2** | Reduce `a_lat_max` from 0.3 → 0.2 | −1 to −2cm at corners (physics-grounded) | 5 min | Yes, no restart |
| **3** | Increase `corner_smooth_arc_pts` 6→10 | −0.5cm chord error | 5 min | Restart needed |
| **4** | robot_localization fused odom | −0 to −1cm (smoother pose input) | 2–4 days | SITL first |

**First milestone (done in one field session):** Tasks 1–5. Expected result: corner xtrack ≤5cm. Zero new code, pure param tuning with one debug-array commit already written.
