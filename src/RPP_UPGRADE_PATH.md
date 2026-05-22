# RPP Controller Upgrade Path

**Goal:** Make `rpp_controller_node.py` measurably superior to PX4 v1.16.2 + 6-patch overlay's AUTO Pure Pursuit on the 3WD marking rover, targeting sub-cm cross-track tracking on R ≥ 1 m paths at 0.4 m/s.

**Current state (baseline):** Working RPP geometry (segment projection, arc-length lookahead, curvature regulation, approach scaling) wrapped in a 50 Hz Python ROS2 node, output as NED velocity vector, consumed by PX4 via MAVROS → `DifferentialOffboardMode` → unsigned `velocity_ned.norm()`.

**Bottom-line gap:** ~110 ms end-to-end latency vs AUTO's ~12 ms. At 0.4 m/s that is 4 cm of head-start that the better geometry cannot recover.

---

## Why RPP is *algorithmically* better than PX4 PP

| Capability | PX4 PP | RPP (current) | Notes |
|---|---|---|---|
| Closest-point projection | Active segment only | All segments | RPP wins on dense polylines |
| Lookahead method | Straight chord | Arc-length walk | RPP eliminates corner-cut error `R(1−cos(L_d/2R))` |
| Curvature speed regulation | None | Linear scale by `R/R_min` | RPP wins on tight curves |
| Goal precision | Acceptance radius (binary) | Linear approach ramp | RPP wins on stops |
| EKF reset handling | Yes | No | PX4 wins on RTK glitches |
| Loop rate | 250 Hz | 50 Hz | PX4 wins on motion discretization |
| Pose source rate | 250 Hz internal | 10 Hz via MAVROS | PX4 wins on staleness |
| End-to-end latency | ~12 ms | ~110 ms | PX4 wins on tracking |

**Verdict:** RPP's *algorithm* is superior. RPP's *implementation* is not. Closing the implementation gaps is the work.

---

## Definition of "RPP superior to PX4 PP"

RPP is superior once **all seven** of these are true:

1. End-to-end latency < 25 ms (currently ~110 ms)
2. Path is C2-continuous, no piecewise-linear curvature
3. Speed regulation peeks ahead (predictive κ, not reactive)
4. Lookahead adapts to cross-track error
5. Signed reverse motion works on hardware
6. EKF reset is detected and handled
7. Pose updates at ≥ 100 Hz (not 10 Hz MAVROS)

Without all seven, AUTO with tuned `PP_LOOKAHD_MIN = 0.20` beats RPP on cross-track. With all seven, RPP wins by an estimated 30–50% on R ≤ 1 m and matches AUTO on straights (both EKF-noise-bound).

---

## Priority Matrix

Effort estimates assume single developer, no field-testing time. ROI = expected XTE improvement / effort hours.

| # | Item | Effort | Expected XTE win | Touches firmware? |
|---|---|---|---|---|
| **P0 — Correctness gaps (do first)** | | | | |
| 0.1 | Closed-loop `L_d` using last commanded `v` | 30 min | 5–10 mm at corners | No |
| 0.2 | EKF reset detection | 2 h | safety / failure-mode fix | No |
| 0.3 | RTK FIX gate (refuse non-zero v unless fix_type=6) | 1 h | precondition | No |
| 0.4 | Signed reverse — add P3+P4 patches to 6-patch overlay | 2 h + flash | unblocks reverse modes | **Yes** (patches 7+8) |
| 0.5 | Output explicit `yaw_setpoint` instead of relying on `atan2(vE, vN)` | 4 h | smoother corners, removes a coupling | No |
| **P1 — Geometry upgrades that put RPP clearly ahead** | | | | |
| 1.1 | Predictive curvature regulation (peek N lookaheads) | 1 day | 5–15 mm on tight curves | No |
| 1.2 | Adaptive lookahead `L_d = k_v·v + k_e·|e_⊥|` | 4 h | 5 mm during reacquisition | No |
| 1.3 | Bezier / Dubins-G2 path smoothing on receipt | 2 days | 5–10 mm everywhere on curves | No |
| 1.4 | Use the (declared but unused) `_closest_seg_hint` | 1 h | CPU only | No |
| **P2 — Latency closure (the real fight)** | | | | |
| 2.1 | C++ rclcpp port of RPP node | 3–5 days | enables 250 Hz | No |
| 2.2 | Bump loop rate to 250 Hz | bundled with 2.1 | 6–8 mm at 0.4 m/s | No |
| 2.3 | Direct uXRCE-DDS to PX4, bypass MAVROS | 1–2 days | **30–40 mm latency cut** | Config only |
| 2.4 | IMU-based pose extrapolation (alt to 2.3) | 1 day | 20–30 mm | No |
| 2.5 | RT scheduling on Jetson (`chrt -f 80`, core pin) | 1 h | jitter ±20 ms → ±2 ms | No |
| **P3 — Vehicle-model upgrades (beat textbook RPP)** | | | | |
| 3.1 | Feedforward `ω_ff = κ·v` via OFFBOARD body-rate | 1 day | 10 mm at corners | No |
| 3.2 | Slip calibration: identify `α` on R=1 m arc | half-day field + 2 h code | 5–15 mm on arcs | No |
| 3.3 | Linear MPC inner loop (acados / osqp-eigen) | 2–3 weeks | 5–10 mm at the limit | No |

---

## P0 — Correctness gaps (3–4 hours of work, no firmware change except 0.4)

### 0.1 Closed-loop lookahead distance

**Current code** (`rpp_controller_node.py`, `_control_loop`):

```python
v_for_ld = max(min_v, max_v * 0.5)  # conservative initial estimate
l_d = self._clamp(v_for_ld * ld_gain, l_min, l_max)
```

`L_d` is therefore constant in time. The `lookahead_time` parameter is dead.

**Fix:** persist last commanded speed, reuse next cycle.

```python
# __init__:
self._last_speed_cmd = 0.0

# _control_loop, before lookahead:
v_for_ld = max(min_v, self._last_speed_cmd if self._last_speed_cmd > 0 else max_v * 0.5)
l_d = self._clamp(v_for_ld * ld_gain, l_min, l_max)

# at end of _control_loop, after computing speed:
self._last_speed_cmd = speed
```

**Why it matters:** at start-up and after any approach-zone slow-down, `L_d` is too long, which inflates `e_ss = L_d²/(2R)`. The fix gives ~6× reduction in steady-state error during slow-speed segments.

### 0.2 EKF reset detection

**Symptom we want to avoid:** RTK FLOAT → RTK FIX transition causes a position jump of 5–30 cm. RPP currently treats this as a tracking error and slams the controller.

**Fix:** subscribe to `/mavros/local_position/odom` (PX4 forwards `xy_reset_counter` here), or detect via velocity-bounded jump:

```python
# threshold: physically impossible motion
max_dt_motion = max_v * dt + 3 * sigma_pos  # ≈ 0.4*0.02 + 3*0.01 = 0.038 m
if self._last_pos is not None:
    jump = math.hypot(pos_n - self._last_pos[0], pos_e - self._last_pos[1])
    if jump > max_dt_motion:
        self.get_logger().warn(f"Position jump {jump*100:.1f} cm — pausing one cycle")
        self._last_pos = (pos_n, pos_e)
        self._publish_zero(StateCode.STALE, ...)
        return
self._last_pos = (pos_n, pos_e)
```

### 0.3 RTK fix gate

Without RTK FIX (fix_type = 6), GPS noise is 5–15 cm CEP. No controller can give you sub-cm tracking at that noise floor. Refuse to drive.

```python
# Subscriber:
self._gps_fix_type = 0
self.create_subscription(GPSRAW, "/mavros/gpsstatus/gps1/raw", self._gps_cb, be_qos)

def _gps_cb(self, msg):
    self._gps_fix_type = msg.fix_type

# In _control_loop, after pose freshness:
require_rtk = self.get_parameter("require_rtk_fix").value
if require_rtk and self._gps_fix_type < 6:
    self.get_logger().warn(
        f"GPS fix_type={self._gps_fix_type} (<6 RTK_FIX) — refusing to drive",
        throttle_duration_sec=2.0)
    self._publish_zero(StateCode.STALE, ...)
    return
```

Default `require_rtk_fix = true` for marking; expose as parameter for SITL / non-RTK testing.

### 0.4 Signed reverse — patches 7 and 8

The deployed `DifferentialOffboardMode.cpp` (v1.16.2):

```cpp
} else if (offboard_control_mode.velocity) {
    rover_speed_setpoint.speed_body_x = velocity_ned.norm();   // unsigned!
    rover_attitude_setpoint.yaw_setpoint = atan2f(velocity_ned(1), velocity_ned(0));
}
```

`(-0.4, 0)` becomes speed = +0.4, bearing = π → SPOT_TURN, not reverse. The fix already exists in the fork's `DifferentialVelControl.cpp` as the P3+P4 patches. They are **not** in the 6-patch overlay.

**Action:**
- Add P3 (signed body-x projection) and P4 (zero-velocity heading hold) to the 6-patch overlay.
- Update `.github/workflows/build_rover.yml` with two new `cp` lines.
- Update `.kiro/steering/structure.md` "The Six Patch Files" → "The Eight Patch Files".
- Update `.kiro/steering/bug-registry.md` with this gap.

If reverse is not needed in production for the marking application, this can be deferred. If it is, do it now.

### 0.5 Output explicit yaw_setpoint

The current contract leans on PX4's `atan2(vE, vN)` heading derivation. This couples your geometry to the FSM in a way you can't see or tune from the Jetson.

**Alternative output:** use type_mask that includes both velocity and yaw.

```python
# twist_to_setpoint_node.py
TYPE_MASK_VEL_AND_YAW = (
    IGNORE_PX | IGNORE_PY | IGNORE_PZ
    | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
    | IGNORE_YAW_RATE
)  # yaw is *not* ignored

# RPP node now outputs (v_n, v_e, yaw_target_ned)
```

In the RPP node, smooth the yaw setpoint with a slew limit (e.g. 90 °/s) to prevent SPOT_TURN trips on path discontinuities. This gives RPP authority over when the FSM activates.

---

## P1 — Geometry upgrades (the real RPP advantage)

### 1.1 Predictive curvature regulation

**Current:** speed regulated by *current* `κ` at the lookahead point.

**Better:** evaluate `κ` at multiple lookahead distances, take the worst.

```python
def _peek_max_curvature(self, seg_idx, foot_n, foot_e, l_d, n_previews=3):
    kappa_max = 0.0
    for k in range(1, n_previews + 1):
        lh_n, lh_e, _ = self._get_lookahead_point(seg_idx, foot_n, foot_e, k * l_d)
        dn, de = lh_n - foot_n, lh_e - foot_e
        # use foot heading instead of vehicle yaw for path-intrinsic curvature
        # ... (project onto path-tangent frame and compute κ)
        kappa_max = max(kappa_max, abs(kappa_at_k))
    return kappa_max
```

**Impact:** rover *anticipates* the corner and starts slowing before it gets there, instead of slowing as it enters. Smoother throttle profile, no late-braking, no overshoot. This is the single biggest geometric improvement over textbook Pure Pursuit and is what Nav2's RPP does.

### 1.2 Adaptive lookahead based on cross-track error

$$
L_d = \mathrm{clamp}(k_v \cdot v + k_e \cdot |e_\perp|,\ L_{\min},\ L_{\max})
$$

**Tuning:** `k_v = lookahead_time = 1.2` (existing), `k_e = 1.0`.

**Behavior:** on path → short `L_d` → tight tracking. Off path → longer `L_d` → smooth re-acquisition without overshoot. Provably stable (Park 2020). PX4 has nothing like this.

### 1.3 Bezier / Dubins-G2 path smoothing

Polylines have piecewise-constant curvature: zero on straights, infinite at vertices. Predictive κ regulator (1.1) reads infinity at every vertex and slams the speed. The fix is to smooth the path once, on receipt:

```python
def _path_cb(self, msg: Path):
    raw = list(msg.poses)
    smoothed = self._smooth_path_g2(raw, corner_radius=0.3)
    self._path = smoothed
    # ...
```

`_smooth_path_g2` replaces each three-vertex window with a quintic Bezier or Dubins arc satisfying G2 continuity. Cost: O(n) on receipt, zero per cycle.

After smoothing, your κ is bounded everywhere, your foot-of-perpendicular projection becomes well-defined, and your predictive regulator reads sane numbers.

### 1.4 Use the segment search hint

Already declared, never used:

```python
self._closest_seg_hint = 0
```

Replace the full-O(n) search with a windowed search:

```python
def _project_onto_path(self, pos_n, pos_e):
    n_pts = len(self._path)
    # bound search around the previous closest segment
    lo = max(0, self._closest_seg_hint - 2)
    hi = min(n_pts - 1, self._closest_seg_hint + 4)
    # ... search only [lo, hi)
    self._closest_seg_hint = best_seg_idx
    return ...
```

For typical motion this is O(1). For path discontinuities (re-plan, big jump), reset the hint to 0.

---

## P2 — Latency closure (until done, geometry can't manifest)

### Current latency breakdown

```
EKF2 → MAVROS pose publish (10 Hz, ENU)         ≈ 50 ms
     → rpp_controller_node 50 Hz                ≈ 20 ms
     → twist_to_setpoint_node 50 Hz             ≈ 20 ms
     → MAVROS forward to FMU                    ≈ 10 ms
     → DifferentialOffboardMode at 250 Hz       ≈  4 ms
     → AttControl + ActControl                  ≈  8 ms
     ────────────────────────────────────────────────────
     Total                                      ≈ 110 ms
```

At 0.4 m/s, 110 ms = 4.4 cm of motion. Hard floor on tracking.

### Target

```
PX4 uXRCE-DDS → Jetson rpp_node (250 Hz, C++)   ≈  4 ms
     → trajectory_setpoint via uXRCE-DDS         ≈  4 ms
     → DifferentialOffboardMode 250 Hz           ≈  4 ms
     → Att + Act                                 ≈  8 ms
     ────────────────────────────────────────────────────
     Total                                      ≈ 20 ms
```

20 ms = 8 mm of motion at 0.4 m/s. RPP geometry now beats AUTO's 12 ms because the geometric advantage at corners is > 12 mm.

### 2.1 / 2.2 — C++ port at 250 Hz

Move `rpp_controller_node.py` → `rpp_controller_node.cpp` as an `rclcpp::Component`. Same algorithm, deterministic timing, no GIL. Combined with 2.3 below, ROS2 timer + DDS gives reliable 250 Hz on Jetson Orin.

Use `Eigen::Vector2d` instead of tuples; vectorise projection loop (SIMD on Jetson). Path smoothing (1.3) goes here too — Boost.Geometry or a hand-rolled Bezier solver.

### 2.3 Direct uXRCE-DDS — biggest single win

PX4 1.16 includes `uxrce_dds_client`. Configuration:

**On PX4 (NSH):**
```
uxrce_dds_client start -t udp -h 192.168.1.102 -p 8888
```

Or persistent via `extras.txt` on SD card.

**On Jetson:**
- Install `Micro-XRCE-DDS-Agent` (`sudo snap install micro-xrce-dds-agent`).
- Run agent: `MicroXRCEAgent udp4 -p 8888`.
- ROS2 node subscribes to `/fmu/out/vehicle_local_position` (250 Hz) and publishes `/fmu/in/trajectory_setpoint`.
- Topic mapping is auto-generated from PX4's `dds_topics.yaml`.

**Wins:**
- Pose at 250 Hz native, not 10 Hz MAVROS.
- No MAVLink serialisation hop.
- Saves ~40 ms total.
- Removes the entire `twist_to_setpoint_node.py` bridge.

**Cost:** one-time setup, plus replacement of all MAVROS topic strings in the RPP node.

### 2.4 IMU-based pose extrapolation (fallback if 2.3 unavailable)

If uXRCE-DDS is blocked (network policy, dependency hell), keep MAVROS for now and dead-reckon the pose between updates:

```python
self.create_subscription(Imu, "/mavros/imu/data", self._imu_cb, be_qos)

def _imu_cb(self, msg):
    self._latest_accel = (msg.linear_acceleration.x, ...)

# in _control_loop, between the last pose and now:
dt = (now - self._pose_recv_time)
pos_n_hat = pos_n + v_n * dt + 0.5 * a_n * dt**2
# use pos_*_hat for projection
```

Reset on each fresh pose. Closes ~30 ms of stale-pose lag.

### 2.5 Real-time scheduling on Jetson

```bash
sudo chrt -f 80 ros2 run px4_dxp rpp_controller_node
sudo taskset -c 4 ros2 run px4_dxp rpp_controller_node
sudo cpufreq-set -c 4 -g performance
```

Or in `px4-dxp.service`:

```ini
[Service]
CPUSchedulingPolicy=fifo
CPUSchedulingPriority=80
CPUAffinity=4
```

Drops timer jitter from ±20 ms to ±2 ms.

---

## P3 — Vehicle-model upgrades (past textbook RPP)

### 3.1 Feedforward yaw rate

With `κ` and `v` known, the yaw rate the rover *should* be turning is `ω_ff = κ · v`. Send it directly:

```python
# Switch type_mask to include yaw_rate
# Use OFFBOARD body_rate mode: offboard_control_mode.body_rate = true
```

Now `DifferentialOffboardMode` routes through the body-rate branch:

```cpp
} else if (offboard_control_mode.body_rate) {
    rover_rate_setpoint.yaw_rate_setpoint = trajectory_setpoint.yawspeed;
}
```

Bypasses the heading PID and SPOT_TURN FSM entirely. Now you only fight rate-tracking dynamics, not heading-error dynamics. This is what removes the visible pivoting at moderate corners.

Pair with a small heading-feedback term to prevent drift: `ω = κ·v + k_ψ·(ψ_target − ψ_actual)`.

### 3.2 Slip calibration

Differential drive with caster slips. The kinematic radius `R_kin = L/(2δ)` is wrong by 5–15%. Identify on a calibration arc:

1. Drive a commanded R = 1 m circle in MANUAL with logged motor commands.
2. Fit the actual circle from `vehicle_local_position`.
3. Compute `α = R_actual / R_commanded`.
4. Apply `R_eff = α · R_kin` in the IK.

Expose as `RD_SLIP_ALPHA` parameter (firmware) or `slip_alpha` (RPP). Re-calibrate on tyre changes.

### 3.3 Linear MPC inner loop

For sub-cm on R < 1 m at 0.3 m/s, even pre-smoothed RPP saturates around 1.5 cm cross-track. To break that floor, you need a model-based controller.

Five-step linear MPC over a unicycle error model:

$$
\begin{bmatrix} \dot e_x \\ \dot e_y \\ \dot e_\psi \end{bmatrix} =
\begin{bmatrix} -1 & 0 & 0 \\ 0 & 0 & v_{ref} \\ 0 & 0 & 0 \end{bmatrix}
\begin{bmatrix} e_x \\ e_y \\ e_\psi \end{bmatrix} +
\begin{bmatrix} 1 & 0 \\ 0 & 0 \\ 0 & 1 \end{bmatrix}
\begin{bmatrix} \tilde v \\ \tilde \omega \end{bmatrix}
$$

Cost: tracking + control effort + slew limits. Solver: `acados` (auto-generated C from Python), 100–500 µs solve time on Jetson Orin. Use it as the inner loop, RPP becomes the path-projection layer that supplies `(p_ref, ψ_ref, v_ref, κ_ref)`.

This is genuinely beyond what PX4 ships. It is also a 2–3 week project. Defer until P0–P2 are done and measured.

---

## Sequencing — recommended order of operations

### Sprint 1 (1 day) — correctness + measurement baseline
- P0.1, P0.2, P0.3, P1.4 → all in `rpp_controller_node.py`
- Capture baseline: 10 m straight, R = 1 m circle, 2 m square at 0.4 m/s, RTK FIX. Log XTE histograms.

### Sprint 2 (3–5 days) — geometry advantage
- P1.1 (predictive κ)
- P1.2 (adaptive `L_d`)
- P1.3 (Bezier smoothing)
- Re-run baseline. Expect 10–20 mm improvement on R = 1 m, no change on straights.

### Sprint 3 (1 week) — close the latency gap
- P2.3 (uXRCE-DDS direct) **or** P2.4 (IMU extrapolation)
- P2.5 (RT scheduling)
- Re-run baseline. Expect 30–40 mm improvement at corners.

### Sprint 4 (1 week) — C++ port + 250 Hz
- P2.1 + P2.2
- Re-run baseline. Expect a further 5–10 mm at corners; jitter improvement everywhere.

### Sprint 5 (3 days) — vehicle model
- P3.1 (feedforward ω) + P3.2 (slip α)
- Re-run baseline. Expect 5–15 mm on arcs; smoother time-domain profile.

### Sprint 6 (optional, 2–3 weeks) — MPC
- P3.3
- Only if Sprint 5 leaves > 1 cm on R < 1 m circles.

### Reverse-motion track (parallel, 0.5 day)
- P0.4 (P3+P4 firmware patches)
- Only if reverse is needed for the marking application.

### Yaw-authority track (parallel, half-day)
- P0.5 (explicit `yaw_setpoint` output)
- Pre-requisite for P3.1 if going via attitude mode rather than body-rate mode.

---

## Validation — definition of done

After each sprint, re-run the standard test set with PX4 ULog + ROS2 bag time-synced by GPS timestamp:

| Test | Path | Speed | Pass criterion |
|---|---|---|---|
| 1 | 10 m straight | 0.4 m/s | XTE max < 2 cm |
| 2 | R = 5 m circle (full) | 0.4 m/s | XTE RMS < 1 cm |
| 3 | R = 1 m circle (full) | 0.2 m/s | XTE RMS < 2 cm |
| 4 | 2 m square, 90° | 0.3 m/s | corner overshoot < 3 cm |
| 5 | 1 m slalom, 5 gates | 0.3 m/s | XTE max < 2 cm |
| 6 | 50 stops | 0.2 m/s | final pos 95th %ile < 1.5 cm |

RPP is "superior to PX4 PP" once it beats the AUTO baseline (recommended-tuned, see `next-session.md`) on at least 5 of the 6 tests with statistical significance over 5 runs.

---

## Open questions to decide before starting

1. **Reverse motion in production?** Drives the priority of P0.4. Default: yes (skid plate, retreat-from-obstacle scenarios). If no, defer.
2. **uXRCE-DDS feasible on the Jetson?** Drives the choice between P2.3 and P2.4. Check network policy and snapd availability.
3. **Acceptable latency-vs-stability trade?** Drives the loop-rate target in Sprint 4. 250 Hz is aspirational; 100 Hz is achievable today.
4. **MPC budget?** Drives whether Sprint 6 gets greenlit. Skip if Sprint 5 result is acceptable.

---

## Notes for future sessions

- **Do not** revert the existing RPP design back to PX4 AUTO during upgrades. Once we commit to OFFBOARD as the production surface, the upgrade investments compound. Switching mid-stream loses the path-smoothing and predictive-κ work.
- **Do** keep AUTO as the fallback. The startup script should be able to switch to AUTO MISSION on RPP node failure, with the same waypoint set.
- **Always** apply parameter changes one at a time, with screen-recorded telemetry, per the param-tuning discipline rule. This applies to RPP parameters as much as to PX4 ones.
- **Always** check the `RoverLandDetector` ↔ `mission_block` interaction before adding any new firmware patch — that is the canonical example of patch coupling.

---

**Document version:** 1.0
**Created:** 2026-05-22
**Owner:** Vetri (firmware/control)
**Source review:** based on `PurePursuit.cpp`, `DifferentialPosControl.cpp`, `DifferentialOffboardMode.cpp`, `DifferentialVelControl.cpp`, `rpp_controller_node.py`, `twist_to_setpoint_node.py` as of 2026-05-22.
