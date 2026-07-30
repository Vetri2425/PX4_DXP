#!/usr/bin/env python3
"""Regulated Pure Pursuit (RPP) controller node for 3WD marking rover.

Architecture position:
  Mission/DXF → Trajectory Planner → [THIS NODE] → twist_to_setpoint_node → MAVROS2 → PX4
                                                                              ↓
                                                                   DifferentialVelControl
                                                                  (NED → body, speed PID,
                                                                   spot-turn FSM, mixing)
                                                                              ↓
                                                                       RoboClaw QPPS

What this node does
-------------------
- Subscribes to a path of waypoints in LOCAL_NED.
- Subscribes to /mavros/local_position/pose (ENU; converted to NED on read).
- Subscribes to /mavros/gpsstatus/gps1/raw for RTK fix gating (P0.3).
- Computes a Regulated Pure Pursuit (RPP) lookahead point on the path.
- Outputs a NED velocity *vector* (not body-frame Twist).

Sprint 1 upgrades vs baseline
------------------------------
  P0.1  Closed-loop L_d: uses last commanded speed → lookahead_time param
        is now live (was dead constant before).
  P0.2  EKF / RTK jump detection: position jumps > physically-possible motion
        pause one control cycle and do not inject a spike into the controller.
  P0.3  RTK FIX gate: refuses to command non-zero velocity unless GPS
        fix_type = 6 (RTK_FIXED). Gated by `require_rtk_fix` parameter so
        SITL / non-RTK testing still works.
  P1.4  Segment search hint (_closest_seg_hint): projection search starts from
        the previous closest segment instead of i=0 every cycle. O(1) in
        steady state instead of O(n).

Sprint 2 upgrades vs Sprint 1
------------------------------
  P1.1  Predictive curvature regulation: speed scaling now uses the worst κ
        across N preview points along the path (path-intrinsic Menger
        curvature), not just at the lookahead point. The rover anticipates
        corners and slows BEFORE entering them. This is the single biggest
        geometric advantage over textbook Pure Pursuit.
  P1.2  Adaptive lookahead: L_d = clamp(k_v·v + k_e·|e_⊥|, L_min, L_max).
        On-path → tight lookahead → tight tracking. Off-path → longer
        lookahead → smooth re-acquisition without overshoot.
  P1.3  Path conditioning on receipt (both opt-in, default OFF for marking):
        - path_resample_spacing_m > 0: linear resample to uniform spacing.
          Preserves geometry exactly; densifies sparse polylines so
          predictive κ has more samples.
        - corner_smooth_radius_m > 0: replace interior vertices with
          inscribed circular arcs of the given radius. Bounds path
          curvature at κ_max = 1/r. Skips vertices where adjacent segments
          are too short to support the radius.

Phase D / P4.1–P4.2 — dynamics-aware speed control
-----------------------------------------------------
  P4.1  Lateral acceleration constraint: speed = min(max_v, sqrt(a_lat_max/|κ|)),
        floored at regulated_linear_scaling_min_speed. Replaces the old linear
        R/min_radius scaling with the physically correct form. At a_lat_max=0.3:
        straight→1.0 m/s, R=1m→0.55 m/s, R=0.5m→0.39 m/s, R=0.3m→0.30 m/s.
        Tune a_lat_max; the old regulated_linear_scaling_min_radius is removed.
  P4.2  Mission speed — single operator knob (ros2 param set mission_speed X.X).
        max_linear_vel is the hardware ceiling (never touch per-job).
        mission_speed is what you set per job: 1.0 for roads, 0.4 for fields.
        approach_velocity_scaling_dist and ekf_jump_threshold_m auto-derive from
        mission_speed at runtime (physics: d=v²/2a, thr=v/Hz+σ_RTK). The
        configured param values act as floors — never silently undersized.

Phase C / P3.1 — opt-in upgrades (default OFF for backward compat)
-----------------------------------------------------------------------
  P0.5  REMOVED 2026-05-23: yaw is now computed in twist_to_setpoint_node from
        the velocity vector (atan2(v_n, v_e)). No separate /rpp/yaw_setpoint_ned
        topic needed. RPP still computes yaw_target internally for _last_yaw_cmd
        state, but no longer publishes it.
  P2.4  Velocity-based pose extrapolation (latency closure): when on,
        dead-reckon `pose_for_projection = pose + vel_ned · pose_age` using
        /mavros/local_position/velocity_local (already EKF-clean and gravity-
        compensated). The pose freshness budget extends by
        imu_max_extrap_age_s so a 50-150 ms MAVROS gap stays usable instead
        of tripping STALE. We deliberately omit the 0.5·a·dt² term: it's
        sub-mm at typical bench accelerations and pulling raw IMU `a` would
        leak gravity bias through any non-zero pitch/roll.
  P3.1  Feedforward yaw rate: publishes /rpp/yaw_rate_body (Float32) with
        ω = κ·v + k_ψ·θ_e for body-rate OFFBOARD mode. Bypasses PX4 spot-turn
        FSM for smoother corners. Requires twist_to_setpoint_node to forward
        the rate.

What this node does NOT do
--------------------------
- Does NOT (by default) compute angular velocity ω. PX4 v1.16+ ignores
  yawspeed in the OFFBOARD velocity branch and derives target yaw from
  atan2(vE, vN) of the velocity vector. P3.1 publishes ω opt-in for the
  body-rate path; the velocity path is unchanged.
- Does NOT implement rotate-to-heading. PX4's spot-turn FSM does this
  automatically; tune RD_TRANS_DRV_TRN (≈30°) and RD_TRANS_TRN_DRV (≈5°).
- Does NOT do body→NED rotation of pose. Output is already in NED.

Output contract
---------------
Topic:  /rpp/velocity_ned   (geometry_msgs/Vector3Stamped)
        header.stamp     = now
        header.frame_id  = "local_ned"
        vector.x         = v_north  (m/s, NED North)
        vector.y         = v_east   (m/s, NED East)
        vector.z         = 0.0

When the path is complete, the velocity vector is exactly (0, 0, 0) and yaw is frozen.
PX4's P4 patch detects |v| < 1 cm/s and freezes heading instead of snapping to North.

When pose is stale or missing, an emergency-stop (0, 0, 0) is published at
50 Hz so OFFBOARD does not drop (COM_OF_LOSS_T = 500 ms).

Diagnostics
-----------
Topic:  /rpp/debug   (std_msgs/Float32MultiArray, layout encoded below)
        [0]  cross_track_error_m  (signed: + = right of path)
        [1]  heading_error_rad    (to lookahead, body frame)
        [2]  lookahead_dist_m     (actual rover→lookahead Euclidean)
        [3]  speed_cmd_m_s
        [4]  curvature_kappa      (steering κ at lookahead, vehicle-relative)
        [5]  dist_to_goal_m
        [6]  pose_age_ms
        [7]  state_code           (see StateCode below; backward compatible)
        [8]  l_d_raw_m            (B1: requested Ld before clamp; saturation visible)
        [9]  kappa_speed          (B1: worst preview κ used for speed scaling)
        [10] yaw_rate_cmd_rad_s   (P3.1: final clamped body yaw rate cmd; 0 if FF disabled)
        [11] max_linear_vel       (param snapshot — hardware ceiling m/s)
        [12] min_linear_vel       (param)
        [13] min_lookahead_dist   (param)
        [14] max_lookahead_dist   (param)
        [15] lookahead_time       (param)
        [16] a_lat_max            (param — m/s²)
        [17] regulated_linear_scaling_min_speed (param)
        [18] xy_goal_tolerance    (param — m)
        [19] min_goal_travel_m    (param — m)
        [20] approach_velocity_scaling_dist (param — m)
        [21] min_approach_linear_velocity (param — m/s)
        [22] p4_zero_vel_threshold (param — m/s)
        [23] pose_max_age_s       (param — s)
        [24] ekf_jump_threshold_m (param — m)
        [25] require_rtk_fix      (param — 0/1 boolean)
        [26] preview_curvature_n  (param — integer count)
        [27] xtrack_lookahead_gain (param)
        [28] path_resample_spacing_m (param — m)
        [29] corner_smooth_radius_m  (param — m)
        [30] corner_smooth_arc_pts   (param — integer count)
        [31] use_imu_extrapolation   (param — 0/1 boolean)
        [32] imu_max_extrap_age_s    (param — s)
        [33] use_feedforward_yaw_rate (param — 0/1 boolean)
        [34] yaw_rate_feedback_gain   (param)
        [35] max_yaw_rate_body        (param — rad/s)
        [36] max_linear_accel         (param — m/s²)
        [37] max_linear_decel         (param — m/s²)
        [38] mission_speed            (param — m/s)
        [39] spray_active             (Phase 3: 1.0 MARK, 0.0 TRANSIT/OFF)
        [40] tracking_profile_code    (0 auto/unknown, 1 segment, 2 smooth)
        [41] segment_corner_threshold_deg
        [42] segment_slowdown_dist
        [43] segment_min_corner_speed
        [44] segment_corner_acceptance_radius
        [45] segment_heading_tolerance_deg
        [46] segment_yaw_rate_gain
Layout is append-only: indices [0..7] keep their meaning forever. Consumers
that only read [0..7] continue to work.

Frame conventions
-----------------
- Path poses are in LOCAL_NED (x = North, y = East, z = Down).
- /mavros/local_position/pose is in ENU (x = East, y = North, z = Up) per
  MAVROS REP-103. We swap x↔y on read to get NED.
- Yaw is converted from ENU quaternion to NED yaw on read.
- All math after the pose-input boundary is NED.
"""

from __future__ import annotations

import math
from enum import IntEnum
from typing import NamedTuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time as RclTime
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import GPSRAW          # P0.3 RTK fix gate
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension, Float32, String

import mission_progress as mp
import precise_stop as pstop
import progress_classifier as pc
from mission_progress import (
    AdvanceMsg,
    MilestoneMsg,
    MissionPhase,
    PointDoneMsg,
    ProgressMsg,
)
from mission_progress_ros import qos_profile


# ---------------------------------------------------------------------------
# Diagnostic state codes (published in /rpp/debug index 7)
# ---------------------------------------------------------------------------
class StateCode(IntEnum):
    STALE = -1      # pose is stale (timeout); emergency stop
    IDLE = 0        # no path or no pose yet
    TRACKING = 1    # normal RPP tracking
    APPROACH = 2    # within approach_dist of goal; speed scaling active
    DONE = 3        # within goal_tolerance; outputting zero velocity
    # B2: distinct codes for the two "controller is publishing zero for a
    # specific reason" branches. All consumers (server/main, server/offboard
    # controller, mission_runner) treat these as STALE-equivalent (no-drive,
    # safety-abort eligible after grace) but the rpp_state_name surfaces
    # the actual reason in telemetry.
    RTK_WAIT = 4    # GPS fix < RTK_FIXED; refusing to drive (P0.3 gate)
    JUMP_SKIP = 5   # one-cycle skip due to position jump (P0.2 EKF guard)


class SegmentStateCode(IntEnum):
    INACTIVE = 0
    TRACK_SEGMENT = 1
    PRE_CORNER_SLOWDOWN = 2
    CORNER_ALIGN = 3
    DONE = 4
    # Stop-and-spin corner execution: zero velocity is held at the corner
    # point until the rover is PHYSICALLY stopped (actual speed below
    # segment_stop_speed_threshold for segment_stop_dwell_s), and only then
    # does CORNER_ALIGN pivot toward the next heading. Prevents approach
    # momentum from carrying the rover past the corner during the pivot.
    CORNER_STOP = 5


# ---------------------------------------------------------------------------
# Atomic mission handoff (_path_cb → control thread)
# ---------------------------------------------------------------------------
class _PendingMission(NamedTuple):
    """One fully-conditioned mission, staged by _path_cb and installed by
    _drain_pending_mission.

    The handoff is a SINGLE reference assignment to `_pending_mission`
    (atomic under the GIL); every actual field write happens in
    _install_mission/_apply_run on the thread that runs the control tick.
    Treat instances as immutable: nothing may mutate `runs` after staging.
    """
    runs: list
    must_hit_keys: frozenset
    stamp: object          # builtin_interfaces.msg.Time
    frame_id: str


# ---------------------------------------------------------------------------
# RPP Controller Node
# ---------------------------------------------------------------------------
class RPPControllerNode(Node):
    """Regulated Pure Pursuit controller — publishes NED velocity at 50 Hz."""

    CONTROL_HZ = 50  # publish rate (Hz) — must match twist_to_setpoint_node

    def __init__(self):
        super().__init__("rpp_controller")

        # ------------------------------------------------------------------
        # Parameters (all tunable at launch / runtime via ros2 param)
        # ------------------------------------------------------------------
        # RPP geometry
        # max_linear_vel is the rover's true hardware ceiling.
        # arc_fix_18 validated: 0.8 with a_lat_max=0.3 constrains arc speed to
        # sqrt(0.3/kappa). For R=1.5m kappa=0.667 → 0.671 m/s effective speed.
        # Tune mission_speed per job to further cap; this is the hw ceiling.
        self.declare_parameter("max_linear_vel",                      0.8)
        self.declare_parameter("min_linear_vel",                      0.15)
        # 1.5 m arc baseline:
        # k_path = 1/1.5 = 0.667 1/m, curvature floor below enforces
        # Ld >= 0.35 / k_path = 0.525 m. Keep the explicit minimum aligned
        # with that floor so the debugged Ld is stable and predictable.
        self.declare_parameter("min_lookahead_dist",                  0.52)
        self.declare_parameter("max_lookahead_dist",                  1.0)
        self.declare_parameter("lookahead_time",                      1.6)

        # Curvature regulation — lateral acceleration constraint (P4.1)
        # v_lat_limit = sqrt(a_lat_max / |kappa|); physically correct form.
        # Replaces the old linear R/min_radius scaling.
        # At a_lat_max=0.3: R=1m→0.55m/s, R=0.5m→0.39m/s, R=0.3m→0.30m/s.
        self.declare_parameter("a_lat_max",                           0.3)   # m/s²
        self.declare_parameter("regulated_linear_scaling_min_speed",  0.3)

        # Goal handling
        self.declare_parameter("xy_goal_tolerance",                   0.02)   # 2 cm
        # Minimum distance the rover must have traveled along the path before
        # the goal check activates. Prevents DONE on closed-loop paths where
        # the rover starts at the final waypoint. Set to 0 to disable.
        self.declare_parameter("min_goal_travel_m",                   0.5)    # m
        # Closed-loop completion guard. A circle (or any closed entity) has
        # first ≈ last waypoint, so Euclidean dist-to-goal is small from the
        # very start; the min_goal_travel_m cap (0.5·len) lets it declare DONE
        # after barely moving. For runs whose endpoints close within
        # close_loop_threshold_m AND whose length exceeds close_loop_min_len_m,
        # require closed_loop_min_travel_frac of the full run length to be
        # traversed before DONE is permitted.
        # 15 cm: must exceed one waypoint spacing — the planner's junction
        # de-dup drops a closed run's coincident seam point, so RPP sees the
        # endpoints ~one spacing apart rather than exactly coincident — while
        # staying far below any genuinely-open shape's endpoint gap.
        self.declare_parameter("close_loop_threshold_m",              0.15)   # endpoint gap → "closed"
        self.declare_parameter("close_loop_min_len_m",                1.0)    # only guard runs longer than this
        self.declare_parameter("closed_loop_min_travel_frac",         0.9)    # fraction of circumference required
        self.declare_parameter("approach_velocity_scaling_dist",      0.6)    # m
        self.declare_parameter("min_approach_linear_velocity",        0.1)
        self.declare_parameter("p4_zero_vel_threshold",               0.02)   # m/s; floor speed below this to exactly 0 to trigger PX4 P4

        # Safety
        self.declare_parameter("pose_max_age_s",                      0.5)    # 500 ms staleness threshold (aligned with COM_OF_LOSS_T)
        self.declare_parameter("path_frame_id",                       "local_ned")

        # P0.2 — EKF / position-jump detection
        # Physically impossible jump threshold: v_max * dt + 3 * sigma_pos.
        # At 50 Hz, dt=0.02 s; v_max=0.4 m/s; sigma_pos≈1 cm RTK.
        # Default 0.05 m (5 cm) — adjust upward if EKF resets during RTK
        # acquisition cause false triggers.
        self.declare_parameter("ekf_jump_threshold_m",                0.05)

        # A3 — EKF reset compensation (default OFF; named A/B switch)
        # When an EKF position reset fires (GNSS re-lock, RTK FLOAT→FIXED),
        # PX4 teleports its own estimate and shifts its internal setpoints by
        # `delta_xy` so tracking is continuous. That `delta_xy`/`xy_reset_counter`
        # is NOT reachable over our MAVROS config (the `odometry` plugin — the
        # only MAVLink carrier of `reset_counter` — is denylisted). So the jump
        # itself, already flagged by the P0.2 guard, is used as the reset delta:
        # it is absorbed into a running offset and subtracted from the tracking
        # pose, holding the path relationship continuous instead of commanding a
        # lurch back onto the line (which paints a kink at every RTK re-lock).
        #   FALSE → exact frozen P0.2 behavior (skip one cycle, no compensation).
        #   TRUE  → absorb the jump, drive through, keep net cross-track ≈ 0.
        # This is the A3 opt-in; leave FALSE until a named A/B run validates it.
        self.declare_parameter("ekf_reset_compensation",              False)
        # Guard: only absorb jumps up to this magnitude. A larger single jump is
        # more likely a bad-estimate glitch than a clean RTK reset; absorbing it
        # would bake a bogus offset into the whole mission frame, so we fall back
        # to the baseline skip-one-cycle behavior above this cap.
        self.declare_parameter("ekf_reset_max_absorb_m",             0.30)

        # P0.3 — RTK FIX gate
        # fix_type = 6 → RTK_FIXED.  Set false for SITL or non-RTK testing.
        self.declare_parameter("require_rtk_fix",                     True)

        # P1.1 — Predictive curvature regulation
        # Number of look-ahead probe points used to find the worst κ in front
        # of the rover. Each probe is at k * L_d arc length, k = 1..N.
        # Speed is regulated by max(|κ|) over the previews, not just the
        # one at L_d. 3 previews is the Nav2 default and a good compromise.
        # Set 1 to disable (matches baseline RPP).
        self.declare_parameter("preview_curvature_n",                 4)

        # P1.2 — Adaptive lookahead based on cross-track error
        # L_d = clamp(lookahead_time * v + xtrack_lookahead_gain * |e_⊥|, L_min, L_max)
        # Set 0.0 to disable the cross-track term (pure velocity-scaled).
        # 1.0 means a 10 cm cross-track adds 10 cm of lookahead.
        self.declare_parameter("xtrack_lookahead_gain",               0.05)

        # B2 (2026-07-25) — smooth-profile lateral correction. The smooth RPP
        # had NO signed lateral term anywhere in the control law (the only use
        # of signed_xtrack was abs()'d into the lookahead length), so with
        # PX4's pure-P yaw loop a steady offset on an arc was structurally
        # permanent: measured 5.1-5.4 cm INSIDE the R=2.4 m curve in all three
        # 2026-07-25 field runs. smooth_lateral_gain rotates the commanded
        # velocity bearing by −gain·signed_xtrack (xtrack + = right of path,
        # NED bearing + = clockwise, so a rover right of path steers left),
        # clamped to ±smooth_lateral_max_deg. Steady-state model: the offset
        # shrinks by 1/(1 + L·gain). 0.0 restores the pre-B2 frozen behaviour.
        # Default 1.5 FIELD-VALIDATED 2026-07-25 (runs 182055 + 190115 —
        # marking RMS 5.13 → 1.00/1.41 cm; crossover k·v ≈ 0.5 rad/s at
        # 0.35 m/s, 3× inside RO_YAW_P=1.5).
        self.declare_parameter("smooth_lateral_gain",                 1.5)
        self.declare_parameter("smooth_lateral_max_deg",              8.0)
        # B2 root cause 2 — the curvature lookahead floor l_d ≥ coeff/κ
        # (= coeff·R) was hardcoded 0.35 and BINDS on gentle arcs (R=2.43 m →
        # 0.85 m lookahead vs 0.56 m velocity-scaled), and the inside-cut
        # scales ~L². 0.35 restores the pre-B2 frozen behaviour. Default 0.20
        # FIELD-VALIDATED with the gain above (floor 0.49 m < raw 0.56 m, so
        # the velocity-scaled lookahead wins; the IDLE-fallback retry at
        # l_min still protects the lookahead walk).
        self.declare_parameter("smooth_curvature_ld_coeff",           0.20)

        # P5.1 — arc inside-cut cap (see the CAP block in the smooth tracker).
        # Pure pursuit cuts inside an arc by e ≈ L²·κ/8, speed-independent. Cap
        # the lookahead at sqrt(8·e_target/κ) so that cut stays bounded.
        #
        # Default 0.005 (5 mm) FIELD-VALIDATED 2026-07-30 on the curve mission
        # (3 runs, ~14:00): valve-gated marking RMS 1.51 → 1.23/1.24/1.30 cm,
        # spread 0.07 cm; the inside-cut signature +1.17 → −0.39 cm (nulled);
        # full coverage in ONE spray interval instead of two. It caps l_d at
        # ≈0.305 m for the measured κ=0.43 (R≈2.4 m).
        #
        # 0.0 restores the pre-P5.1 geometry exactly — keep that as the A/B arm
        # if a regression appears. Only the SMOOTH profile is affected; segment
        # mode is untouched, so straight-line and stop/pivot behaviour on the
        # frozen controller is unchanged by this default.
        #
        # ⚠ NOT yet regression-tested on the 2x2 square (the shape that came
        # back partial at a globally-forced L=0.30). The cap is curvature-
        # conditional, so it should not bind on the square's straights — but
        # that is reasoning, not a measurement.
        self.declare_parameter("smooth_max_arc_cut_m",                0.005)
        # Hard floor on the capped lookahead, so an over-tight cap on a sharp
        # arc cannot collapse l_d toward zero and degenerate the lookahead walk
        # (the IDLE-fallback path). 0.25 m is below every value tested and well
        # clear of the 0.12-0.21 m band that went unstable on 2026-07-29.
        self.declare_parameter("smooth_min_arc_ld_m",                 0.25)
        # Half-baseline for path curvature estimation, in metres of arc length.
        # MUST stay ≥ ~0.10 m: three-point curvature on a 4-10 cm densified
        # path is noise-dominated (measured 0.626 at 0.05 m vs 0.42-0.43 from
        # 0.10 m up on the same arc). 0.0 restores the legacy adjacent-vertex
        # estimate — do not use it for anything that scales a control quantity.
        self.declare_parameter("curvature_baseline_m",                0.15)

        # P1.3 — Path conditioning on receipt
        # path_resample_spacing_m: if > 0, linearly resample the path to this
        #   uniform spacing on receipt. Densifies sparse polylines so the
        #   predictive κ regulator has enough samples. Geometry is preserved
        #   exactly (straight segments stay straight). 0.0 disables.
        # corner_smooth_radius_m: if > 0, replace interior vertices with
        #   inscribed arcs of this radius. Bounds path κ at 1/r. Vertices
        #   whose adjacent segments are shorter than r are left as sharp
        #   corners with a warning. 0.0 disables.
        # corner_smooth_arc_pts: number of points used to discretise each
        #   inscribed arc (only used when corner_smooth_radius_m > 0).
        self.declare_parameter("path_resample_spacing_m",             0.08)
        self.declare_parameter("corner_smooth_radius_m",              0.5)
        self.declare_parameter("corner_smooth_arc_pts",               6)

        # Tracking profile:
        #   auto    — classify each received path as segment or smooth
        #   segment — straight/polyline/polygon tracker with corner align
        #   smooth  — existing RPP with optional corner smoothing
        # `sharp` is accepted as a runtime alias for `segment`.
        self.declare_parameter("tracking_profile",                    "auto")
        self.declare_parameter("segment_corner_threshold_deg",         45.0)
        # Max turn at a vertex that the segment-mode lookahead may look
        # THROUGH instead of clipping at. See _segment_lookahead_point for the
        # full rationale and the 2026-07-30 field evidence.
        #
        # 5.0 matches the `collinear_tol_deg` the simplifier already uses to
        # decide what counts as a straight run, so this crosses exactly the
        # vertices that survived simplification for a non-geometric reason
        # (spray-flag boundaries, declared must-hit points) and never a corner.
        #
        # 0.0 restores the pre-fix clip-at-segment-end geometry EXACTLY — that
        # is the A/B arm if a regression appears. Set it as a default, not at
        # runtime: runtime params on this node are lost on restart.
        self.declare_parameter("segment_lookahead_cross_collinear_deg",  5.0)
        # Segment-mode simplification keeps a "collinear" vertex anyway if it
        # sits more than this far off the straight run (metric Douglas-Peucker
        # test). Stops near-straight must-hit points (a few cm off, only ~3 deg)
        # from being dropped so the rover actually tracks through them. 1 cm is
        # below the sub-2 cm tracking floor, so real drawn points survive while
        # exactly-collinear densification samples (0 cm off) are still removed.
        # 0 = old angle-only behaviour.
        self.declare_parameter("segment_simplify_max_offset_m",        0.01)
        # D2 (runtime entry): when true, run 0 pivots in place to its first
        # heading before tracking (spray OFF via _run_alignment_hold) instead of
        # the emergent forward-cone arc at tracking speed.
        #
        # DEFAULT FLIPPED False -> True, 2026-07-29, on field evidence. The A/B
        # is done and this side won.
        #
        # The `or run["flags"][0]` clause below already forces a pre-align when
        # run 0 starts on a MARK — but a GPS_SURVEYED two-phase entry publishes
        # the ENTRY leg first with spray_flags [False, False], so that clause
        # does NOT fire there and the rover drove the entry leg from whatever
        # heading it was parked at. It then reached the first point crooked and
        # never recovered.
        #
        # Measured, same missions, same code, same session:
        #   OFF : 2.01 cm and 10.58 cm on ONE mission run twice; the bad run
        #         reported "none — no CORNER_STOP->ALIGN->TRACK pivots" and
        #         carried a -2.67 cm bias (55 % left) the whole line.
        #   ON  : 0.96 / 1.65 / 2.15 / 2.17 cm across four runs, every one with
        #         a pivot, the two repeats of one mission agreeing to 0.02 cm.
        #
        # It was left OFF as a D2 A/B flag and never flipped after the A/B
        # concluded. Because it is a declare_parameter default with nothing
        # overriding it in rpp_start.sh or the launch file, a runtime
        # `ros2 param set` silently reverted on every rpp-pipeline restart —
        # which is why one session produced both 2 cm and 10 cm runs with no
        # log line explaining the difference.
        #
        # See docs/RUNTIME_ENTRY_VELOCITY_PLAN.md §4 D2.
        self.declare_parameter("entry_prealign_enabled",              True)
        self.declare_parameter("segment_slowdown_dist",               0.50)
        self.declare_parameter("segment_min_corner_speed",             0.08)
        # Final-segment (run-endpoint) goal-approach floor. A per-line PRE/AFT
        # run ends AT a corner, so the rover must arrive slow enough for active
        # braking to stop it within the corner point. The old endpoint floor was
        # min_approach_linear_velocity (0.10 m/s); the speed loop overshot to
        # ~0.13 and braking (0.08 cap) coasted 3-4.6 cm past the corner, smearing
        # MARK entry to ~4.8 cm. A dedicated, lower floor here — separate from the
        # WITHIN-run corner floor (segment_min_corner_speed) and the smooth/arc
        # floor (min_approach_linear_velocity) — drops run-endpoint arrival to
        # ~0.05 m/s (floor + overshoot) so drift is <1 cm, without touching the
        # non-extension square's within-run corners or arc approaches.
        self.declare_parameter("segment_endpoint_approach_speed",      0.03)   # m/s
        self.declare_parameter("segment_corner_acceptance_radius",     0.05)
        # Pivot exit tolerance. MUST sit strictly ABOVE the firmware's
        # RD_TRANS_TRN_DRV stop angle (2.0° as flown): the firmware stops
        # turning at ITS threshold, so an equal companion tolerance deadlocks —
        # PX4 quits at exactly the angle RPP still requires it to cross.
        # 2026-07-30 bags: 6/6 pivots stalled 8-14 s pinned at ~2.5°, escaping
        # only on EKF yaw noise (P0-1). 3.0° accepts before the firmware quits.
        self.declare_parameter("segment_heading_tolerance_deg",        3.0)
        self.declare_parameter("segment_yaw_rate_gain",                1.5)
        # CORNER_STOP: confirm the rover is physically stopped at the corner
        # before pivoting. Both linear speed AND yaw-rate (from velocity_local)
        # must be below their thresholds for segment_stop_dwell_s. The 2 s
        # stale-data cap prevents a deadlock if the velocity topic goes quiet;
        # it never overrides fresh evidence that the rover is still moving.
        self.declare_parameter("segment_stop_speed_threshold",         0.02)   # m/s
        self.declare_parameter("segment_stop_yaw_rate_threshold",      0.05)   # rad/s (~2.9 deg/s)
        self.declare_parameter("segment_stop_dwell_s",                 0.30)   # s
        # ── Phase D point-hold A/B (FROZEN CONTROLLER — default OFF) ─────────
        # When enabled, the rover brakes to a confirmed stop at each must-hit
        # waypoint and HOLDS for point_hold_s (long enough for the spray node's
        # arrival-settle + dwell + OFF-confirm), then continues — so point-mode
        # dots are actually painted. Reuses the corner-brake/stop primitives.
        # OFF = byte-for-byte the frozen baseline (the overlay early-returns).
        # This is a named A/B: enable only for a point-mission run at the rover.
        self.declare_parameter("point_hold_enabled",                   False)
        self.declare_parameter("point_hold_s",                         2.0)    # s
        self.declare_parameter("point_hold_acceptance_m",              0.10)   # m
        # ── RPP progress + spray handshake (design RPP_PROGRESS_HANDSHAKE) ───
        # G0: declared now, defaults = frozen behavior, NOTHING reads them yet.
        # Each is wired + A/B'd in a later phase (G1/G3/G4/G5). See
        # docs/Architecture/RPP_PROGRESS_HANDSHAKE_TASKS.md.
        # G1 — publish /rpp/progress + /rpp/milestone (pure observability):
        self.declare_parameter("progress_publish_enabled",            False)
        self.declare_parameter("progress_approach_dist_m",            0.30)   # m — APPROACH_/MARK_END band width
        # G3 — precise 2 cm stop at must-hit points (point mode only):
        self.declare_parameter("point_precise_stop_enabled",          False)
        self.declare_parameter("point_arrival_tolerance_m",           0.02)   # m
        self.declare_parameter("precise_stop_mode",                   "feedforward")  # feedforward|servo
        self.declare_parameter("precise_stop_decel_m_s2",             0.30)   # m/s² (feedforward profile)
        self.declare_parameter("precise_stop_creep_speed",            0.05)   # m/s (servo)
        self.declare_parameter("precise_stop_max_s",                  8.0)    # s
        # G4/G5 — point handshake + auto/manual advance:
        # point_handshake_enabled (default OFF) swaps the Phase-D fixed point_hold_s
        # timer for the RPP↔spray handshake: RPP holds the dwell until the spray
        # node confirms /spray/point_done (auto), or — in manual mode — until the
        # operator's /point/advance. point_hold_max_s is the never-wedge backstop.
        # OFF ⇒ the frozen fixed-timer point-hold runs byte-for-byte. The handshake
        # also needs progress_publish_enabled (for the AT_POINT milestone spray
        # reacts to); the backstop makes any misconfig safe.
        self.declare_parameter("point_handshake_enabled",           False)
        self.declare_parameter("point_execution_mode",               "auto")  # auto|manual
        self.declare_parameter("manual_wait_timeout_s",              0.0)     # s (0 = wait forever)
        self.declare_parameter("point_hold_max_s",                   10.0)    # s backstop cap
        # Active braking at a corner stop. PX4 velocity-OFFBOARD does not brake
        # on a zero setpoint — it coasts — so a rover that reaches the corner
        # still at ~0.1-0.16 m/s drifts 2-3 cm before the dwell confirms, and
        # then pivots from the wrong point. When velocity data is fresh and the
        # rover is still above the stop threshold, command a small velocity
        # opposing its motion (capped here) to actively decelerate. 0 disables.
        self.declare_parameter("segment_brake_velocity_cap_m_s",       0.08)   # m/s
        # CORNER_ALIGN exit: heading error AND yaw-rate (AND, when fresh, linear
        # speed) must be within tolerance for segment_align_settle_s before the
        # state machine advances. Prevents premature exit while the rover is
        # still spinning or drifting.
        self.declare_parameter("segment_align_settle_s",               0.20)   # s
        self.declare_parameter("segment_align_speed_threshold",        0.02)   # m/s (release gate)
        # Pivot watchdog: after this long, relax the heading tolerance but
        # still require yaw-rate settling. Never launch onto the next line
        # merely because the timer expired while the rover is still turning.
        self.declare_parameter("segment_turn_timeout_s",               5.0)    # s
        # Timeout release band. MUST be strictly wider than
        # segment_heading_tolerance_deg or the watchdog is a no-op:
        # max(tol, timeout_tol) relaxes nothing when both are equal — which is
        # exactly how every 2026-07-30 pivot stall escaped only on EKF drift.
        self.declare_parameter("segment_timeout_heading_tolerance_deg", 4.0)   # deg
        # Angle-aware pivot watchdog (Part B): a fixed timeout is wrong for a
        # corner whose magnitude varies. The rover spot-turns at a roughly
        # constant rate, so the budget scales with the corner angle:
        #   budget = spinup_margin + corner_angle_rad / nominal_pivot_rate
        # clamped to [segment_turn_timeout_s (min), segment_pivot_timeout_max_s].
        # A 120° corner at 0.40 rad/s + 1.0 s spin-up ⇒ ~6.2 s, vs the old
        # flat 5.0 s that released a 120° pivot ~20–30° short.
        self.declare_parameter("segment_pivot_spinup_margin_s",        1.0)    # s
        self.declare_parameter("segment_nominal_pivot_rate_rad_s",     0.40)   # rad/s (field-observed)
        self.declare_parameter("segment_pivot_timeout_max_s",          9.0)    # s safety clamp
        # Hard release gate: never launch onto the next leg while the heading
        # error to that leg exceeds this, regardless of timeout. Backstops the
        # relaxed timeout band so a mis-set tolerance cannot release a large
        # residual error into forward TRACK acceleration. MUST sit above
        # segment_timeout_heading_tolerance_deg — this cap is applied with
        # min(), so a cap at/below the timeout band silently re-disables the
        # watchdog (the 2026-07-30 stall had 3.0 capping a 2.0/2.0 pair).
        self.declare_parameter("segment_pivot_release_max_deg",        5.0)    # deg
        # Connector absorption (Part A): adjacent apex waypoints can leave a
        # sub-threshold "connector" segment (e.g. 8 cm) between two real legs.
        # If it survives into run-splitting it becomes its own pivot target and
        # the remaining leg turn falls below segment_corner_threshold_deg, so
        # the real leg is entered un-pivoted. Collapse any segment shorter than
        # this that is bracketed by two genuine corners into a single corner
        # vertex before splitting. Set to 0 to disable.
        self.declare_parameter("connector_absorb_m",                   0.20)   # m
        self.declare_parameter("connector_min_corner_deg",             20.0)   # deg (bracket gate)

        # P2.4 — Velocity-based pose extrapolation (latency closure)
        # When enabled, dead-reckon the pose forward by `vel_ned * pose_age`
        # to close the gap between when MAVROS published the pose and when
        # the controller is about to use it. We also extend the pose-age
        # acceptance window by `imu_max_extrap_age_s` so a 50-150 ms MAVROS
        # latency stays usable instead of tripping STALE.
        # Backwards compat: default off.
        self.declare_parameter("use_imu_extrapolation",               False)
        # Cap on how far past pose_max_age_s we'll trust extrapolation.
        # 0.10 s + the existing 0.20 s pose_max_age = 300 ms total budget.
        self.declare_parameter("imu_max_extrap_age_s",                0.10)

        # P3.1 — Feedforward yaw rate via body-rate mode
        # When enabled, RPP computes ω_ff = κ·v and sends it directly to PX4
        # via OFFBOARD body-rate mode instead of relying on heading PID.
        # Bypasses spot-turn FSM, smoother corners, better rate tracking.
        # Requires twist_to_setpoint_node to support body-rate output.
        self.declare_parameter("use_feedforward_yaw_rate",            True)
        # For the 1.5 m arc, start with pure kappa*v feedforward. Outer yaw
        # feedback had been over-commanding yaw rate in arc_fix_28.
        self.declare_parameter("yaw_rate_feedback_gain",              0.0)
        # Clamp on body yaw rate. Match PX4 RO_YAW_RATE_LIM (deg/s) converted
        # to rad/s so RPP doesn't request more than PX4 will honor.
        # 0.45 rad/s ≈ 25.8°/s — validated baseline (arc_fix_16, 2026-06-03):
        # with the PX4 yaw feedforward enabled (RD_MAX_THR_YAW_R=0.95) the arc
        # peaks at ~0.34 rad/s, so 0.45 leaves headroom and never saturates.
        # Set 0.0 to disable.
        self.declare_parameter("max_yaw_rate_body",                   0.45)

        # Acceleration ramp (P0 polish): cap how fast `speed` can RAMP UP
        # cycle-to-cycle. Prevents motor jerk on mission start and after a
        # stop/re-plan. Decel is intentionally NOT limited — the P4 floor
        # relies on instantaneous step-to-zero to trigger PX4 P4 yaw freeze
        # at the goal. Set to 0.0 to disable.
        self.declare_parameter("max_linear_accel",                    0.35)  # m/s²

        # P4.2 — Mission speed (operator-facing, set per job via ros2 param set)
        # This is the single knob the operator touches. It is capped by
        # max_linear_vel (hardware ceiling). Dependent params — approach distance
        # and EKF jump threshold — are derived from this value at runtime so the
        # operator never has to touch them.
        # Roads/large fields: 1.0 m/s  |  Sports fields/tight marking: 0.3–0.5 m/s
        self.declare_parameter("mission_speed",                       0.35)  # m/s

        # P4.2 — Deceleration limit used ONLY for braking-distance derivation.
        # Separate from max_linear_accel because the accel ramp is one-way
        # (decel is unbounded in the control loop by design — P4 goal freeze).
        # This param tells the approach-zone calculator how quickly the rover
        # can realistically stop. Default matches max_linear_accel.
        self.declare_parameter("max_linear_decel",                    0.5)  # m/s²

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._path: list[PoseStamped] = []
        self._spray_flags: list[bool] = []
        self._active_tracking_profile: str = "smooth"
        # Per-entity run queue (see _split_runs_by_flag / _apply_run)
        self._runs: list[dict] = []
        # Mission mailbox: _path_cb stages a _PendingMission here as ONE
        # reference; _drain_pending_mission (control thread) installs it.
        # `is` comparison against _installed_mission makes the steady-state
        # tick cost one attribute load + one identity check — no allocation,
        # no lock, and no clear-write that could race a fresher staging.
        self._pending_mission: _PendingMission | None = None
        self._installed_mission: _PendingMission | None = None
        # Quantised (n,e) keys of points the planner flagged as source geometry.
        # Empty = no provenance on this path (legacy publisher) → simplification
        # falls back to the geometric tests alone.
        self._must_hit_keys: frozenset[tuple[int, int]] = frozenset()
        # Phase D point-hold A/B (default OFF). Which must-hit points have had
        # their dwell, the one currently being dwelled, and its dwell start.
        # Byte-for-byte inert unless point_hold_enabled is set.
        self._point_hold_done_keys: set = set()
        self._point_hold_active_key = None
        self._point_hold_start_ns = None
        # G3 precise-stop servo (default OFF; only under point_precise_stop_enabled
        # + precise_stop_mode=servo). Wall-clock start of the creep phase for the
        # timeout backstop; None while not servoing. _point_approach_speed is the
        # speed captured when the hold engaged — the kinematic cap for the
        # feed-forward decel profile.
        self._point_servo_start_ns = None
        self._point_approach_speed = 0.0
        # G4/G5 — point handshake (default OFF; only under point_handshake_enabled).
        # Latest /spray/point_done seen (index + monotonic seq); the seq snapshot
        # taken when the current hold armed, so only a point_done that arrived
        # AFTER arming can release it. _point_spray_done_this_hold latches the
        # dwell-complete proof so a later point_done for another point cannot
        # un-latch it mid-wait. _point_wait_start_ns marks the manual WAIT_OPERATOR
        # phase; the /point/advance index + monotonic count gate the release.
        self._point_done_index: int = -1
        self._point_done_seq: int = -1
        self._point_done_seq_at_arm: int = -1
        self._point_spray_done_this_hold: bool = False
        self._point_wait_start_ns = None
        self._advance_index: int = -1
        self._advance_count: int = 0
        self._advance_count_at_wait: int = 0
        # G1 — mission-progress publication (default OFF, observability only).
        # must-hit vertices of the active run in path order + a key→rank map, so
        # the progress classifier can name the point currently held. Rebuilt in
        # _apply_run. All read only under progress_publish_enabled.
        self._musthit_vertices: list[int] = []
        self._musthit_rank_by_key: dict = {}
        self._progress_seq: int = 0
        self._progress_last_phase: MissionPhase = MissionPhase.IDLE
        self._progress_last_point: int = -1
        self._run_idx: int = 0
        self._run_align_pending: bool = False
        # Latched while a completed run is physically stopping before the
        # controller is allowed to switch to the next, differently-headed run.
        self._run_boundary_stop_pending: bool = False
        # D3: terminal twin of the run-boundary latch. Latched while the rover
        # brakes to a confirmed physical stop at the FINAL waypoint before DONE
        # is published. PX4 velocity-OFFBOARD coasts on a bare zero setpoint, so
        # without this the rover drifts past xy_goal_tolerance, the goal check
        # flips false, control falls through to tracking, and it drives away
        # (bag 2026-07-10_20-07 Line_2m: reached at 4 mm, ran 1.08 m past).
        self._completion_stop_pending: bool = False
        self._segment_idx: int = 0
        self._segment_state: SegmentStateCode = SegmentStateCode.INACTIVE
        # CORNER_STOP / pivot-watchdog state (shared by the segment corner and
        # run-transition pivots — only one corner is active at a time).
        self._corner_stop_entered: RclTime | None = None      # when CORNER_STOP began
        self._corner_stop_settle_since: RclTime | None = None # speed+yaw-rate both OK since
        self._corner_stop_complete: bool = False               # stop confirmed; pivoting now
        self._pivot_started: RclTime | None = None             # when CORNER_ALIGN actuation began
        self._pivot_timeout_warned: bool = False
        self._pivot_turn_angle_rad: float = 0.0                # corner magnitude this pivot must cover
        self._align_settle_since: RclTime | None = None        # heading+yaw-rate both OK since
        self._run_align_turn_rad: float = 0.0                  # run-transition corner magnitude (for budget)
        self._last_segment_debug: tuple[float, ...] = (
            0.0, 0.0, 0.0, float("nan"), float("nan"),
            float("nan"), float("nan"), float("nan"), 0.0, 0.0,
        )
        self._pose: PoseStamped | None = None
        self._pose_recv_time: RclTime | None = None
        self._path_done = False
        self._path_travel_m: float = 0.0   # monotonic along-path progress on active run
        self._path_s: list[float] = []     # cumulative arc length for active run

        # P1.4 — segment search hint: start projection from previous best seg
        self._closest_seg_hint: int = 0
        # Track last filtered speed for curvature-aware lookahead smoothing
        self._filtered_speed: float = 0.0
        # P1.4 (Sprint 2 fixup) — full-scan flag: forces O(n) projection on
        # the first cycle after a path reset OR an EKF jump, then sticks to
        # the windowed O(1) search. Without this, a re-plan that places the
        # rover mid-path causes 2-3 cycles of wrong-direction velocity (~1.6
        # to 2.4 cm of bad motion at 0.4 m/s) — outside the 2 cm goal spec.
        self._hint_valid: bool = False

        # P0.1 — closed-loop L_d: persist last commanded speed
        self._last_speed_cmd: float = 0.0
        # R2 — measured control-tick dt for accel ramps (ROS clock). None until
        # the first tick after a path load so that tick uses 1/CONTROL_HZ.
        self._last_tick: RclTime | None = None
        self._tick_dt: float = 1.0 / self.CONTROL_HZ

        # P0.5 — explicit yaw_setpoint: persist last commanded yaw for freeze
        self._last_yaw_cmd: float = 0.0

        # P2.4 — Velocity-based pose extrapolation (latency closure).
        # We dead-reckon the pose forward by `vel_ned * dt_pose_age` to
        # close the MAVROS pose latency gap. We use velocity (not
        # acceleration) because at v=0.4 m/s the v·dt term is ~30× larger
        # than 0.5·a·dt² and is gravity-clean (PX4 EKF compensated).
        # `_latest_vel_ned` holds the latest NED velocity from
        # /mavros/local_position/velocity_local. MAVROS publishes it in ENU;
        # we swap x↔y like for pose in _vel_cb().
        self._latest_vel_ned: tuple[float, float] = (0.0, 0.0)
        self._latest_vel_time: RclTime | None = None
        self._latest_yaw_rate_ned: float = 0.0   # EKF yaw-rate, NED CW+ (rad/s)

        # P0.2 — EKF jump detection: last accepted NED position
        self._last_pos: tuple[float, float] | None = None

        # A3 — EKF reset compensation: cumulative offset (NED, m) absorbed from
        # position jumps this mission, and a count for post-hoc correlation.
        # Both stay zero unless `ekf_reset_compensation` is enabled. Subtracted
        # from the tracking pose so absorbed resets do not appear as cross-track.
        self._ekf_reset_offset: tuple[float, float] = (0.0, 0.0)
        self._ekf_reset_count: int = 0

        # P0.3 — RTK fix tracking
        self._gps_fix_type: int = 0  # 0 = no fix; 6 = RTK_FIXED

        # ------------------------------------------------------------------
        # QoS profiles
        # ------------------------------------------------------------------
        be_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._vel_pub = self.create_publisher(
            Vector3Stamped, "/rpp/velocity_ned", be_qos
        )
        self._dbg_pub = self.create_publisher(
            Float32MultiArray, "/rpp/debug", be_qos
        )
        self._segment_dbg_pub = self.create_publisher(
            Float32MultiArray, "/rpp/segment_debug", be_qos
        )
        self._conditioned_path_pub = self.create_publisher(
            Path, "/rpp/conditioned_path", path_qos
        )
        # P3.1: optional yaw rate (body-rate mode) publisher
        self._yaw_rate_pub = self.create_publisher(
            Float32, "/rpp/yaw_rate_body", be_qos
        )
        self._spray_active_pub = self.create_publisher(
            Bool, "/spray/active", be_qos
        )
        # G1 — mission-progress channel (design §4.1/§4.2). Separate topics +
        # separate enum from /rpp/segment_debug, whose frozen contract is left
        # untouched. Nothing publishes here unless progress_publish_enabled.
        self._progress_pub = self.create_publisher(
            String, mp.TOPIC_PROGRESS, qos_profile(mp.PROGRESS_QOS)
        )
        self._milestone_pub = self.create_publisher(
            String, mp.TOPIC_MILESTONE, qos_profile(mp.MILESTONE_QOS)
        )

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(Path, "/path", self._path_cb, path_qos)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, be_qos
        )
        # P0.3 — RTK fix gate: track GPS fix quality
        self.create_subscription(
            GPSRAW, "/mavros/gpsstatus/gps1/raw", self._gps_cb, be_qos
        )
        # P2.4 — Velocity for pose extrapolation (latency closure).
        # `/mavros/local_position/velocity_local` (geometry_msgs/TwistStamped)
        # is in ENU (linear.x=East, linear.y=North); we swap to NED on read.
        # PX4-EKF-compensated, so it's already gravity-clean — first-order
        # `pos + v·dt` dead-reckon is the dominant term and avoids the
        # gravity bias that comes with raw IMU acceleration.
        from geometry_msgs.msg import TwistStamped
        self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_local",
            self._vel_cb, be_qos,
        )
        # G4/G5 — point handshake command channels (design §4.3/§4.4). RELIABLE
        # VOLATILE (never TRANSIENT_LOCAL, so a restart cannot replay a stale
        # "done"/"advance"). Read-only unless point_handshake_enabled; the
        # callbacks only cache the latest message, so the frozen controller is
        # untouched when the flag is OFF.
        self.create_subscription(
            String, mp.TOPIC_POINT_DONE, self._point_done_cb,
            qos_profile(mp.POINT_DONE_QOS),
        )
        self.create_subscription(
            String, mp.TOPIC_ADVANCE, self._advance_cb,
            qos_profile(mp.ADVANCE_QOS),
        )

        # ------------------------------------------------------------------
        # 50 Hz control timer
        # ------------------------------------------------------------------
        self._timer = self.create_timer(1.0 / self.CONTROL_HZ, self._control_loop)

        # P0.2 fixup — surface incompatible threshold/velocity combinations.
        # The default ekf_jump_threshold_m=0.05 assumes max_linear_vel<=1.5 m/s.
        # Bump in either direction without bumping the other → false-positive
        # jump-skips that look like an EKF problem but are just expected motion.
        self._check_threshold_compat()

        self.get_logger().info(
            "RPP controller started "
            "(Sprint 1: P0.1 Ld, P0.2 EKF, P0.3 RTK, P1.4 hint; "
            "Sprint 2: P1.1 pred-κ, P1.2 adapt-Ld, P1.3 cond; "
            "Phase B: B1 dbg10, B2 RTK_WAIT/JUMP_SKIP, B3 1-pass walk) — "
            "output: /rpp/velocity_ned (NED, Vector3Stamped). "
            "Waiting for /path and /mavros/local_position/pose."
        )

    # ==================================================================
    # Subscriber callbacks
    # ==================================================================
    def _path_cb(self, msg: Path):
        """Validate frame, accept new path, reset state."""
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path — ignoring")
            return

        expected = self.get_parameter("path_frame_id").value
        if msg.header.frame_id and msg.header.frame_id != expected:
            self.get_logger().error(
                f"Path frame_id {msg.header.frame_id!r} != expected {expected!r}; "
                f"rejecting path. (Set 'path_frame_id' param to match planner.)"
            )
            return

        # P1.3 — Path conditioning (linear resample + corner smoothing).
        # Operates on (north, east) tuples to keep the geometry code simple,
        # then converts back to PoseStamped at the end.
        raw_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        # position.z is a bitfield: bit0 = spray ON, bit1 = must-hit vertex
        # (source geometry, not densification fill). MUST bit-test, not `> 0.5`
        # — a spray-OFF must-hit point encodes as 2.0.
        _z = [int(round(p.pose.position.z)) for p in msg.poses]
        raw_flags = [bool(z & 1) for z in _z]
        # Provenance travels by coordinate, not by index: run splitting reorders
        # and re-groups points but never MOVES them, so a quantised (n,e) key
        # survives the whole conditioning pipeline intact.
        # Local until install: _path_cb writes NO tracked state directly —
        # everything lands atomically via _install_mission.
        must_hit_keys = frozenset(
            self._pt_key(p) for p, z in zip(raw_pts, _z) if z & 2
        )
        n_raw = len(raw_pts)

        resample_dx = float(self.get_parameter("path_resample_spacing_m").value)
        corner_r    = float(self.get_parameter("corner_smooth_radius_m").value)
        arc_pts     = int(self.get_parameter("corner_smooth_arc_pts").value)
        requested = self._normalize_tracking_profile(
            self.get_parameter("tracking_profile").value
        )
        threshold = float(
            self.get_parameter("segment_corner_threshold_deg").value
        )

        # Auto profile resolves PER RUN, not per path: a DXF mission mixes
        # line entities (segment tracking) and arc/circle entities (smooth
        # RPP) in one Path. Spray-flag transitions are the entity boundaries
        # the planner gives us, so each contiguous same-flag run is
        # classified and conditioned independently, then the runs are
        # tracked in sequence (see _apply_run/_advance_run). A forced
        # profile keeps the whole path as a single run.
        if requested == "auto":
            connector_m = float(self.get_parameter("connector_absorb_m").value)
            connector_min_corner = float(
                self.get_parameter("connector_min_corner_deg").value
            )
            raw_runs = [
                sub
                for run in self._split_runs_by_flag(raw_pts, raw_flags)
                for sub in self._split_run_at_corners(
                    *self._absorb_short_connectors(
                        *run, threshold, connector_m, connector_min_corner
                    ),
                    threshold,
                )
            ]
            # PRE(OFF) -> MARK(ON) -> AFT(OFF) is one straight motion pass.
            # Preserve the per-point spray flags, but fuse collinear flag runs
            # so spray transitions do not create endpoint slowdown/reacquisition.
            raw_runs = self._merge_collinear_runs(raw_runs, threshold)
        else:
            raw_runs = [(raw_pts, raw_flags)]

        stamp = msg.header.stamp
        runs: list[dict] = []
        for run_pts, run_flags in raw_runs:
            profile = (
                requested if requested != "auto"
                else self._classify_auto_profile(run_pts, threshold)
            )
            if profile == "segment":
                c_pts, c_flags = self._simplify_path_for_profile(
                    run_pts, run_flags,
                    max_offset_m=float(
                        self.get_parameter("segment_simplify_max_offset_m").value
                    ),
                    must_hit_keys=must_hit_keys,
                )
            else:
                c_pts, c_flags = run_pts, run_flags
                if corner_r > 0.0 and len(c_pts) >= 3:
                    c_pts, c_flags = self._smooth_corners(
                        c_pts, corner_r, max(2, arc_pts), c_flags
                    )
                if resample_dx > 0.0 and len(c_pts) >= 2:
                    c_pts, c_flags = self._resample_path(
                        c_pts, resample_dx, c_flags
                    )
            runs.append({
                "poses": self._build_poses(
                    c_pts, c_flags, stamp, expected, must_hit_keys
                ),
                "flags": list(c_flags),
                "profile": profile,
                "length": self._pts_length(c_pts),
                "cum_s": self._pts_cumulative_lengths(c_pts),
                "closed": self._is_closed_run(c_pts),
            })

        # Drop degenerate slivers (e.g. the ~3 cm reversed stub that spray
        # compensation folds back at a mark start): each would command a
        # pointless stop + double 180° pivot. The next run starts within
        # goal tolerance of the dropped geometry, so nothing is lost.
        if len(runs) > 1:
            kept = [r for r in runs if r["length"] >= 0.05]
            if kept and len(kept) < len(runs):
                self.get_logger().info(
                    f"Dropped {len(runs) - len(kept)} sliver run(s) < 5 cm"
                )
                runs = kept

        # Hand the whole mission over as ONE reference assignment; every real
        # field write happens in _install_mission on the control thread. The
        # inline drain below is what the serial executor already guarantees
        # (install completes before the next timer tick), so behaviour today
        # is byte-identical. A future MultiThreadedExecutor drops ONLY the
        # inline call and installs at tick-top instead — the reverted MT
        # experiment died exactly on the alternative: an e-stop 1-point path
        # swapped in under a tick whose _closest_seg_hint sat deep inside a
        # ~20k-point path → IndexError inside the control timer mid-e-stop.
        self._pending_mission = _PendingMission(
            runs=runs, must_hit_keys=must_hit_keys,
            stamp=stamp, frame_id=expected,
        )
        self._drain_pending_mission()

        n_cond = sum(len(r["poses"]) for r in runs)
        n_seg_runs = sum(1 for r in runs if r["profile"] == "segment")
        first = runs[0]["poses"][0].pose.position
        last = runs[-1]["poses"][-1].pose.position
        self.get_logger().info(
            f"Path accepted: {n_raw} → {n_cond} waypoints in {len(runs)} "
            f"run(s) ({n_seg_runs} segment, {len(runs) - n_seg_runs} smooth; "
            f"requested={requested}, resample={resample_dx:.2f}m, "
            f"corner_r={corner_r:.2f}m), "
            f"first=({first.x:.2f}N, {first.y:.2f}E), "
            f"last=({last.x:.2f}N, {last.y:.2f}E)"
        )

    def _pose_cb(self, msg: PoseStamped):
        """Store latest pose. Frame conversion happens at use-site."""
        self._pose = msg
        self._pose_recv_time = self.get_clock().now()

    # P0.3 — RTK fix gate callback
    def _gps_cb(self, msg: GPSRAW):
        """Track GPS fix type. fix_type=6 → RTK_FIXED (required for marking)."""
        prev = self._gps_fix_type
        self._gps_fix_type = msg.fix_type
        if prev != msg.fix_type:
            fix_names = {0: "NO_FIX", 1: "NO_FIX", 2: "2D", 3: "3D",
                         4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"}
            self.get_logger().info(
                f"GPS fix changed: {fix_names.get(prev,'?')} → "
                f"{fix_names.get(msg.fix_type,'?')} (fix_type={msg.fix_type})"
            )

    # P2.4 — Velocity callback for pose extrapolation
    def _vel_cb(self, msg):
        """Track latest NED linear velocity and yaw-rate from MAVROS (EKF-clean).

        MAVROS publishes `/mavros/local_position/velocity_local` in ENU:
          msg.twist.linear.x  = East,  msg.twist.linear.y = North
          msg.twist.angular.z = yaw-rate ENU (CCW+)
        We swap linear x↔y to NED. ENU angular.z (CCW+) negates to NED (CW+).
        """
        v_north = msg.twist.linear.y    # ENU y → NED x
        v_east = msg.twist.linear.x     # ENU x → NED y
        self._latest_vel_ned = (v_north, v_east)
        self._latest_yaw_rate_ned = -msg.twist.angular.z   # ENU CCW+ → NED CW+
        self._latest_vel_time = self.get_clock().now()

    # ==================================================================
    # Boot-time parameter sanity check
    # ==================================================================
    def _check_threshold_compat(self):
        """Warn if ekf_jump_threshold_m is too tight for max_linear_vel.

        Per-cycle physical max motion = max_v / control_hz, plus ~3σ_RTK ≈ 3 cm.
        If the threshold is below that, every cycle of fast driving will trip
        the jump guard and the rover will refuse to drive. Surfacing this at
        boot prevents a 20-minute "why won't it move" debug session.
        """
        # P4.2: ekf_jump_threshold is auto-derived each cycle as
        # max(param, mission_speed/Hz + 0.03), so the manual param is
        # a floor. Log the effective threshold at current mission_speed.
        mission_v = float(self.get_parameter("mission_speed").value)
        hw_max_v = float(self.get_parameter("max_linear_vel").value)
        max_v = min(hw_max_v, mission_v)
        jump_thr_param = float(self.get_parameter("ekf_jump_threshold_m").value)
        jump_thr_derived = max_v / self.CONTROL_HZ + 0.03
        jump_thr_eff = max(jump_thr_param, jump_thr_derived)
        self.get_logger().info(
            f"P4.2 jump threshold: param={jump_thr_param:.3f}m, "
            f"derived={jump_thr_derived:.3f}m (at mission_speed={max_v:.2f}m/s), "
            f"effective={jump_thr_eff:.3f}m"
        )

        # P4.1 lateral-accel constraint info: log the effective speed at
        # representative radii so the operator can verify tuning at boot.
        a_lat = float(self.get_parameter("a_lat_max").value)
        min_curv_v = float(self.get_parameter("regulated_linear_scaling_min_speed").value)
        if a_lat > 0.0:
            # kappa = 1/R; v_lat = sqrt(a_lat / kappa) = sqrt(a_lat * R)
            v_r1  = max(min_curv_v, min(max_v, math.sqrt(a_lat * 1.0)))
            v_r05 = max(min_curv_v, min(max_v, math.sqrt(a_lat * 0.5)))
            v_r03 = max(min_curv_v, min(max_v, math.sqrt(a_lat * 0.3)))
            self.get_logger().info(
                f"P4.1 lat-accel: a_lat_max={a_lat:.2f} m/s² → "
                f"R=1.0m:{v_r1:.2f}m/s  R=0.5m:{v_r05:.2f}m/s  R=0.3m:{v_r03:.2f}m/s  "
                f"straight:{max_v:.2f}m/s  floor:{min_curv_v:.2f}m/s"
            )

        # Min-approach vs P4-floor invariant: the P4 floor must be BELOW the
        # approach floor, otherwise the rover hard-zeros throughout approach
        # instead of only at the goal — destroying smooth deceleration.
        p4_floor = float(self.get_parameter("p4_zero_vel_threshold").value)
        approach_v = float(self.get_parameter("min_approach_linear_velocity").value)
        if p4_floor >= approach_v:
            self.get_logger().warn(
                f"p4_zero_vel_threshold={p4_floor:.3f} >= "
                f"min_approach_linear_velocity={approach_v:.3f}. Rover will "
                f"abruptly zero throughout the approach zone, not just at goal. "
                f"Set p4_zero_vel_threshold strictly less than "
                f"min_approach_linear_velocity (e.g. 0.02 vs 0.05)."
            )

        # Accel ramp diagnostic: if max_linear_accel is so high that one
        # cycle covers (max_v - min_v), the limiter is effectively a no-op.
        accel = float(self.get_parameter("max_linear_accel").value)
        min_v = float(self.get_parameter("min_linear_vel").value)
        if accel > 0.0 and accel / self.CONTROL_HZ > (max_v - min_v):
            self.get_logger().warn(
                f"max_linear_accel={accel:.2f} m/s² allows full speed-up "
                f"({max_v - min_v:.2f} m/s span) in one {1000/self.CONTROL_HZ:.0f} ms cycle. "
                f"Limiter is effectively disabled. Set lower (e.g. 0.5) "
                f"or use 0.0 to disable explicitly."
            )

    # ==================================================================
    # Frame conversion helpers
    # ==================================================================
    @staticmethod
    def _enu_pose_to_ned(pose_stamped: PoseStamped) -> tuple[float, float, float]:
        """Convert MAVROS ENU pose → NED (north, east, yaw_ned).

        MAVROS REP-103: pose.position is ENU (x=East, y=North, z=Up).
        Quaternion encodes ENU yaw (0=East, CCW positive).
        Returns (north, east, yaw_ned) where yaw_ned is 0=North, CW positive.
        """
        # Position: ENU x=East,y=North → NED x=North,y=East
        north = pose_stamped.pose.position.y
        east = pose_stamped.pose.position.x

        # Yaw: extract ENU yaw from quaternion, convert to NED
        q = pose_stamped.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_enu = math.atan2(siny_cosp, cosy_cosp)
        yaw_ned = math.pi / 2.0 - yaw_enu
        yaw_ned = (yaw_ned + math.pi) % (2 * math.pi) - math.pi
        return north, east, yaw_ned

    # ==================================================================
    # Geometry helpers
    # ==================================================================
    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    @staticmethod
    def _dist(ax: float, ay: float, bx: float, by: float) -> float:
        return math.hypot(ax - bx, ay - by)

    @staticmethod
    def _angle_wrap(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _profile_code(profile: str) -> float:
        if profile == "segment":
            return 1.0
        if profile == "smooth":
            return 2.0
        return 0.0

    @staticmethod
    def _normalize_tracking_profile(value: object) -> str:
        profile = str(value).strip().lower()
        if profile == "sharp":
            return "segment"
        if profile in ("auto", "segment", "smooth"):
            return profile
        return "auto"

    @staticmethod
    def _segment_heading(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.atan2(b[1] - a[1], b[0] - a[0])

    @classmethod
    def _heading_delta(cls, h0: float, h1: float) -> float:
        return abs(cls._angle_wrap(h1 - h0))

    @staticmethod
    def _perp_dist(
        p: tuple[float, float],
        a: tuple[float, float],
        b: tuple[float, float],
    ) -> float:
        """Perpendicular distance of point p from the infinite line a->b."""
        dx, dy = b[0] - a[0], b[1] - a[1]
        h = math.hypot(dx, dy)
        if h < 1e-9:
            return math.hypot(p[0] - a[0], p[1] - a[1])
        return abs(dx * (a[1] - p[1]) - dy * (a[0] - p[0])) / h

    @staticmethod
    def _pt_key(p: tuple[float, float]) -> tuple[int, int]:
        """Quantise a point to a 1 mm grid for identity lookups.

        Conditioning reorders and regroups points but never moves them, so a
        millimetre-quantised key is a stable identity across the pipeline.
        """
        return (int(round(p[0] * 1000.0)), int(round(p[1] * 1000.0)))

    @classmethod
    def _dp_mark_keep(
        cls,
        pts: list[tuple[float, float]],
        eps: float,
        lo: int,
        hi: int,
        keep: list[bool],
    ) -> None:
        """Douglas-Peucker: mark indices in (lo, hi) needed to stay within eps.

        Iterative (explicit stack) rather than recursive — a road alignment can
        carry thousands of points and Python's recursion limit is 1000.
        """
        stack = [(lo, hi)]
        while stack:
            a_i, b_i = stack.pop()
            if b_i <= a_i + 1:
                continue
            a, b = pts[a_i], pts[b_i]
            d_max, i_max = 0.0, -1
            for i in range(a_i + 1, b_i):
                d = cls._perp_dist(pts[i], a, b)
                if d > d_max:
                    d_max, i_max = d, i
            if i_max >= 0 and d_max > eps:
                keep[i_max] = True
                stack.append((a_i, i_max))
                stack.append((i_max, b_i))

    @classmethod
    def _simplify_path_for_profile(
        cls,
        pts: list[tuple[float, float]],
        flags: list[bool] | None = None,
        collinear_tol_deg: float = 5.0,
        max_offset_m: float = 0.0,
        must_hit_keys: frozenset[tuple[int, int]] | None = None,
    ) -> tuple[list[tuple[float, float]], list[bool]]:
        """Remove duplicate and same-heading vertices while preserving corners.

        This is used for segment-mode decisions and tracking. Generated
        squares/rectangles often arrive as many collinear samples per side;
        segment mode needs the side endpoints, not every resampled point.
        Flag changes are preserved so mark/transit boundaries are not erased.

        A point is retained when ANY of these hold:

        1. it is an endpoint, or a spray-flag boundary;
        2. `must_hit_keys` marks it as source geometry (a CAD/survey vertex
           rather than densification fill) — the authoritative test;
        3. its heading change from the last retained point exceeds
           `collinear_tol_deg` — the corner test;
        4. `max_offset_m > 0` and dropping it would push the retained polyline
           further than that from the original — a true Douglas-Peucker test.

        Rule 4 replaces a guard that measured each candidate against the NEXT
        RAW SAMPLE instead of the span being collapsed. On a path densified at
        `mark_spacing` (5 cm) that measured ~2.4 mm at a vertex sitting 3-4 cm
        off the retained chord, so it never fired and surveyed vertices were
        silently deleted — the rover then drove a straight chord and missed
        them by that offset. Measuring against the collapsing span is what
        `tools/analyze_mission.py:_perp_from_span` already does.

        Rule 2 is what actually settles it. A tolerance alone cannot separate
        survey noise from intent — a 3.4 cm real bend and 3.4 cm of RTK noise
        are numerically identical. Provenance can: densification fill carries no
        intent and may be dropped freely, a source vertex is operator intent and
        is never dropped. If a survey is noisy the resulting shape is noisy, and
        that is a survey-quality problem to surface, not one for the controller
        to silently smooth away.

        When `max_offset_m <= 0` and `must_hit_keys` is empty this reduces
        exactly to the original angle-only behaviour, which the classification
        and run-merge call sites depend on.
        """
        if flags is None or len(flags) != len(pts):
            flags = [False] * len(pts)
        if not pts:
            return [], []

        clean_pts: list[tuple[float, float]] = [pts[0]]
        clean_flags: list[bool] = [bool(flags[0])]
        for pt, flag in zip(pts[1:], flags[1:]):
            if math.hypot(pt[0] - clean_pts[-1][0], pt[1] - clean_pts[-1][1]) < 1e-6:
                clean_flags[-1] = bool(clean_flags[-1] or flag)
                continue
            clean_pts.append(pt)
            clean_flags.append(bool(flag))

        n = len(clean_pts)
        if n < 3:
            return clean_pts, clean_flags

        tol = math.radians(collinear_tol_deg)
        mh = must_hit_keys or frozenset()

        # Pass 1 — angle / flag / must-hit anchors. The heading test measures
        # from the last RETAINED point (not the previous raw one) so gradual
        # curvature accumulates; classification relies on that exact behaviour.
        keep = [False] * n
        keep[0] = True
        keep[n - 1] = True
        last_kept = 0
        for i in range(1, n - 1):
            this_pt = clean_pts[i]
            h0 = cls._segment_heading(clean_pts[last_kept], this_pt)
            h1 = cls._segment_heading(this_pt, clean_pts[i + 1])
            heading_change = cls._heading_delta(h0, h1)
            flag_boundary = (
                clean_flags[i - 1] != clean_flags[i]
                or clean_flags[i] != clean_flags[i + 1]
            )
            must = cls._pt_key(this_pt) in mh
            if heading_change <= tol and not flag_boundary and not must:
                continue
            keep[i] = True
            last_kept = i

        # Pass 2 — geometric fidelity between anchors. Only runs when a
        # tolerance is set, so angle-only callers are bit-for-bit unchanged.
        if max_offset_m > 0.0:
            anchors = [i for i, k in enumerate(keep) if k]
            for a_i, b_i in zip(anchors, anchors[1:]):
                cls._dp_mark_keep(clean_pts, max_offset_m, a_i, b_i, keep)

        out_pts = [clean_pts[i] for i in range(n) if keep[i]]
        out_flags = [clean_flags[i] for i in range(n) if keep[i]]
        return out_pts, out_flags

    @classmethod
    def _classify_auto_profile(
        cls,
        pts: list[tuple[float, float]],
        threshold_deg: float,
    ) -> str:
        """Classify raw path geometry as segment or smooth.

        Segment covers straight lines and hard-corner polylines. Smooth covers
        continuous-curvature paths where heading changes accumulate gradually.
        """
        simple, _ = cls._simplify_path_for_profile(pts, collinear_tol_deg=5.0)
        if len(simple) <= 2:
            return "segment"

        headings: list[float] = []
        for a, b in zip(simple[:-1], simple[1:]):
            if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-6:
                headings.append(cls._segment_heading(a, b))
        if len(headings) <= 1:
            return "segment"

        threshold = math.radians(max(0.0, threshold_deg))
        deltas = [cls._heading_delta(a, b) for a, b in zip(headings[:-1], headings[1:])]
        if not deltas:
            return "segment"
        if max(deltas) >= threshold:
            return "segment"

        # Smooth requires SUSTAINED turning: several distinct turning
        # vertices accumulating real heading change, the signature of a
        # discretized arc/circle/spline. One or two shallow bends — a cut
        # corner from waypoint sampling, a slight kink between chained
        # lines — must stay segment.
        turning = [d for d in deltas if d > math.radians(2.0)]
        if len(turning) >= 3 and sum(turning) > math.radians(20.0):
            return "smooth"
        return "segment"

    # ------------------------------------------------------------------
    # Per-entity run management (auto profile)
    #
    # A mission Path is split into "runs" at spray-flag transitions —
    # the planner toggles the flag at every entity↔transit boundary, so
    # each run is one entity (or one transit hop). Runs are classified
    # and conditioned independently in _path_cb, then tracked one at a
    # time: the active run IS self._path, so every existing single-
    # profile code path works unchanged. Completion checks call
    # _advance_run() instead of declaring DONE; DONE is only published
    # after the last run, which is what the server's mission-complete
    # settling logic watches for.
    # ------------------------------------------------------------------
    @staticmethod
    def _pts_length(pts: list[tuple[float, float]]) -> float:
        return sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(pts[:-1], pts[1:])
        )

    @classmethod
    def _pts_cumulative_lengths(cls, pts: list[tuple[float, float]]) -> list[float]:
        """Cumulative arc length at each waypoint."""
        out = [0.0]
        for a, b in zip(pts[:-1], pts[1:]):
            out.append(out[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        return out

    @staticmethod
    def _split_runs_by_flag(
        pts: list[tuple[float, float]], flags: list[bool]
    ) -> list[tuple[list[tuple[float, float]], list[bool]]]:
        """Split a path into contiguous same-flag runs (entity boundaries).

        Each run after the first is prepended with the previous run's last
        point so consecutive runs share their boundary vertex and the
        concatenated geometry stays gap-free. The prepended point takes the
        run's own flag so each run stays flag-homogeneous.
        """
        groups: list[tuple[list[tuple[float, float]], list[bool]]] = []
        start = 0
        for i in range(1, len(pts)):
            if flags[i] != flags[i - 1]:
                groups.append((list(pts[start:i]), list(flags[start:i])))
                start = i
        groups.append((list(pts[start:]), list(flags[start:])))

        out: list[tuple[list[tuple[float, float]], list[bool]]] = []
        for k, (rp, rf) in enumerate(groups):
            if k > 0:
                rp = [groups[k - 1][0][-1]] + rp
                rf = [rf[0]] + rf
            out.append((rp, rf))
        return out

    @classmethod
    def _runs_collinear(
        cls,
        prev_pts: list[tuple[float, float]],
        next_pts: list[tuple[float, float]],
        threshold_deg: float,
    ) -> bool:
        """Return True when two contiguous runs continue in the same direction."""
        if not prev_pts or not next_pts:
            return False
        gap = math.hypot(
            next_pts[0][0] - prev_pts[-1][0],
            next_pts[0][1] - prev_pts[-1][1],
        )
        if gap > 0.05:
            return False
        sp, _ = cls._simplify_path_for_profile(prev_pts)
        sn, _ = cls._simplify_path_for_profile(next_pts)
        if len(sp) < 2 or len(sn) < 2:
            return False
        h0 = cls._segment_heading(sp[-2], sp[-1])
        h1 = cls._segment_heading(sn[0], sn[1])
        return math.degrees(cls._heading_delta(h0, h1)) < threshold_deg

    @classmethod
    def _merge_collinear_runs(
        cls,
        runs: list[tuple[list[tuple[float, float]], list[bool]]],
        threshold_deg: float,
    ) -> list[tuple[list[tuple[float, float]], list[bool]]]:
        """Fuse collinear PRE/MARK/AFT flag runs without changing spray flags."""
        if len(runs) < 2:
            return [(list(pts), list(flags)) for pts, flags in runs]

        merged: list[tuple[list[tuple[float, float]], list[bool]]] = [
            (list(runs[0][0]), list(runs[0][1]))
        ]
        for pts, flags in runs[1:]:
            prev_pts, prev_flags = merged[-1]
            same_profile = (
                cls._classify_auto_profile(prev_pts, threshold_deg)
                == cls._classify_auto_profile(pts, threshold_deg)
            )
            if same_profile and cls._runs_collinear(prev_pts, pts, threshold_deg):
                start = 0
                if math.hypot(
                    pts[0][0] - prev_pts[-1][0],
                    pts[0][1] - prev_pts[-1][1],
                ) < 1e-6:
                    start = 1
                prev_pts.extend(pts[start:])
                prev_flags.extend(flags[start:])
            else:
                merged.append((list(pts), list(flags)))
        return merged

    @classmethod
    def _simplify_with_indices(
        cls,
        pts: list[tuple[float, float]],
        flags: list[bool],
        collinear_tol_deg: float = 5.0,
    ) -> tuple[list[tuple[float, float]], list[bool], list[int]]:
        """Like _simplify_path_for_profile but also returns, for each kept
        vertex, its index in the *input* `pts`. Used by connector absorption to
        map simplified corner vertices back to the raw run for splicing.

        Duplicate (coincident) points are folded into the surviving vertex, so
        the returned indices point at the first occurrence of each kept vertex.
        """
        if not pts:
            return [], [], []
        # Fold coincident points (mirror _simplify_path_for_profile), tracking
        # the original index of each survivor.
        clean_pts: list[tuple[float, float]] = [pts[0]]
        clean_flags: list[bool] = [bool(flags[0])]
        clean_idx: list[int] = [0]
        for j in range(1, len(pts)):
            pt, flag = pts[j], flags[j]
            if math.hypot(pt[0] - clean_pts[-1][0], pt[1] - clean_pts[-1][1]) < 1e-6:
                clean_flags[-1] = bool(clean_flags[-1] or flag)
                continue
            clean_pts.append(pt)
            clean_flags.append(bool(flag))
            clean_idx.append(j)

        if len(clean_pts) < 3:
            return clean_pts, clean_flags, clean_idx

        tol = math.radians(collinear_tol_deg)
        out_pts = [clean_pts[0]]
        out_flags = [clean_flags[0]]
        out_idx = [clean_idx[0]]
        for i in range(1, len(clean_pts) - 1):
            h0 = cls._segment_heading(out_pts[-1], clean_pts[i])
            h1 = cls._segment_heading(clean_pts[i], clean_pts[i + 1])
            heading_change = cls._heading_delta(h0, h1)
            flag_boundary = (
                clean_flags[i - 1] != clean_flags[i]
                or clean_flags[i] != clean_flags[i + 1]
            )
            if heading_change <= tol and not flag_boundary:
                continue
            out_pts.append(clean_pts[i])
            out_flags.append(clean_flags[i])
            out_idx.append(clean_idx[i])
        out_pts.append(clean_pts[-1])
        out_flags.append(clean_flags[-1])
        out_idx.append(clean_idx[-1])
        return out_pts, out_flags, out_idx

    @staticmethod
    def _line_intersection(
        a: tuple[float, float],
        b: tuple[float, float],
        c: tuple[float, float],
        d: tuple[float, float],
    ) -> tuple[float, float] | None:
        """Intersection of infinite line (a→b) with infinite line (c→d).

        Returns None when the directions are near-parallel (no stable
        intersection). Used to recover the true corner apex two doubled
        waypoints approximate.
        """
        d1x, d1y = b[0] - a[0], b[1] - a[1]
        d2x, d2y = d[0] - c[0], d[1] - c[1]
        denom = d1x * d2y - d1y * d2x
        if abs(denom) < 1e-9:
            return None
        t = ((c[0] - a[0]) * d2y - (c[1] - a[1]) * d2x) / denom
        return (a[0] + t * d1x, a[1] + t * d1y)

    @classmethod
    def _absorb_short_connectors(
        cls,
        pts: list[tuple[float, float]],
        flags: list[bool],
        threshold_deg: float,
        connector_absorb_m: float,
        min_corner_deg: float = 20.0,
    ) -> tuple[list[tuple[float, float]], list[bool]]:
        """Collapse sub-threshold connector segments before run splitting.

        Adjacent apex waypoints in planner output can leave a very short
        segment (e.g. 8 cm) between two real legs — a "connector". Left intact
        it survives run-splitting as its own pivot target, and the remaining
        leg turn then falls below segment_corner_threshold_deg, so the real leg
        is entered un-pivoted (the triangle-apex-2 failure).

        A segment between simplified corner vertices V[i]→V[i+1] is absorbed
        when ALL hold:
          * its length < connector_absorb_m, and
          * it is *interior* (a real leg exists on each side), and
          * both ends are genuine corners: the bend at V[i] (incoming leg vs
            connector) AND at V[i+1] (connector vs outgoing leg) each exceed
            min_corner_deg.
        The two endpoints are replaced by the *intersection of the two adjacent
        leg lines* — the true apex the doubled waypoints approximate — so both
        legs keep their original headings (a plain midpoint sits off the leg
        lines when the connector is lateral, bending the corner and leaving a
        residual stub). If the legs are near-parallel or the intersection lands
        implausibly far away, the midpoint is used as a safe fallback. The merge
        vertex inherits the connector's flag, so mark/transit semantics hold.

        The dual-corner gate is what protects a *real* short MARK stroke: a
        deliberate short entity is not bracketed by two hard corners (it is
        collinear-ish with, or flag-bounded from, its neighbours), so it is left
        untouched. Arc/circle samples (gradual <min_corner_deg bends) are also
        immune. connector_absorb_m ≤ 0 disables the pass.
        """
        if connector_absorb_m <= 0.0 or len(pts) < 4:
            return list(pts), list(flags)

        verts, vflags, vidx = cls._simplify_with_indices(pts, flags)
        if len(verts) < 4:
            return list(pts), list(flags)

        min_corner = math.radians(min_corner_deg)
        # Collect absorb actions as (raw_start_idx, raw_end_idx, midpoint, flag).
        # Evaluate on the *original* simplified geometry; apply right-to-left so
        # raw indices stay valid. Interior requires i in [1 .. len-3].
        actions: list[tuple[int, int, tuple[float, float], bool]] = []
        i = 1
        while i <= len(verts) - 3:
            a, b, c, d = verts[i - 1], verts[i], verts[i + 1], verts[i + 2]
            seg_len = math.hypot(c[0] - b[0], c[1] - b[1])
            if seg_len < connector_absorb_m:
                bend_in = cls._heading_delta(
                    cls._segment_heading(a, b), cls._segment_heading(b, c)
                )
                bend_out = cls._heading_delta(
                    cls._segment_heading(b, c), cls._segment_heading(c, d)
                )
                if bend_in >= min_corner and bend_out >= min_corner:
                    mid = ((b[0] + c[0]) * 0.5, (b[1] + c[1]) * 0.5)
                    apex = cls._line_intersection(a, b, c, d)
                    # Use the leg-line intersection unless it is missing
                    # (near-parallel legs) or implausibly far from the connector
                    # — then fall back to the midpoint.
                    if apex is not None and math.hypot(
                        apex[0] - mid[0], apex[1] - mid[1]
                    ) <= max(0.5, 5.0 * seg_len):
                        merge = apex
                    else:
                        merge = mid
                    actions.append((vidx[i], vidx[i + 1], merge, bool(vflags[i])))
                    i += 2   # skip past the absorbed connector
                    continue
            i += 1

        if not actions:
            return list(pts), list(flags)

        out_pts = list(pts)
        out_flags = list(flags)
        for raw_a, raw_b, mid, flag in reversed(actions):
            out_pts[raw_a:raw_b + 1] = [mid]
            out_flags[raw_a:raw_b + 1] = [flag]
        return out_pts, out_flags

    @classmethod
    def _split_run_at_corners(
        cls,
        pts: list[tuple[float, float]],
        flags: list[bool],
        threshold_deg: float,
    ) -> list[tuple[list[tuple[float, float]], list[bool]]]:
        """Sub-split one run at hard corners so each piece is geometrically
        homogeneous.

        Real planner output is not entity-clean: consecutive mark entities
        chain without a transit between them, and the bridge from a transit
        into an entity can reverse direction (~180°). A single hard corner
        like that would otherwise flip a whole circle run to the segment
        profile. Splitting at hard corners yields pure pieces — straight
        sides classify segment, arcs/circles classify smooth — and the
        corner itself becomes a run transition, where _run_alignment_hold
        pivots the rover before the next piece starts.
        """
        simple, _ = cls._simplify_path_for_profile(pts)
        if len(simple) < 3:
            return [(pts, flags)]

        corner_pts: list[tuple[float, float]] = []
        for i in range(1, len(simple) - 1):
            h0 = cls._segment_heading(simple[i - 1], simple[i])
            h1 = cls._segment_heading(simple[i], simple[i + 1])
            if math.degrees(cls._heading_delta(h0, h1)) >= threshold_deg:
                corner_pts.append(simple[i])
        if not corner_pts:
            return [(pts, flags)]

        # Simplification preserves the original point tuples, so corner
        # vertices can be located in the raw run by exact equality, in order.
        splits: list[int] = []
        ci = 0
        for idx, p in enumerate(pts):
            if ci < len(corner_pts) and p == corner_pts[ci]:
                splits.append(idx)
                ci += 1

        out: list[tuple[list[tuple[float, float]], list[bool]]] = []
        start = 0
        for s in splits:
            if s > start:
                out.append((pts[start:s + 1], flags[start:s + 1]))
                start = s
        if start < len(pts) - 1:
            out.append((pts[start:], flags[start:]))
        return out or [(pts, flags)]

    @staticmethod
    def _build_poses(
        pts: list[tuple[float, float]],
        flags: list[bool],
        stamp,
        frame_id: str,
        must_hit_keys: frozenset[tuple[int, int]] | None = None,
    ) -> list[PoseStamped]:
        """Build conditioned poses, re-emitting the position.z bitfield.

        bit0 = spray ON, bit1 = must-hit. Carrying provenance through to
        /rpp/conditioned_path is what lets analyze_mission's geometry-fidelity
        report tell "this vertex was dropped" from "this was only fill".
        """
        mh = must_hit_keys or frozenset()
        poses: list[PoseStamped] = []
        for (n, e), flag in zip(pts, flags):
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = frame_id
            ps.pose.position.x = float(n)
            ps.pose.position.y = float(e)
            must = (int(round(n * 1000.0)), int(round(e * 1000.0))) in mh
            ps.pose.position.z = float((1 if flag else 0) | (2 if must else 0))
            ps.pose.orientation.w = 1.0
            poses.append(ps)
        return poses

    def _drain_pending_mission(self) -> None:
        """Install the latest staged mission unless it is already active.

        Steady-state cost (called every control tick): one attribute load and
        one identity compare — no allocation, no lock (a lock here would risk
        priority inversion against the FIFO-80 control thread). The mailbox is
        never cleared; tracking the installed object by identity means a
        fresher mission staged mid-drain simply installs on the next tick
        (last-writer-wins, same as today's serial execution).
        """
        pm = self._pending_mission
        if pm is None or pm is self._installed_mission:
            return
        # Mark installed BEFORE installing: if _install_mission ever raises,
        # the broken mission must not be retried at 50 Hz.
        self._installed_mission = pm
        self._install_mission(pm)

    def _install_mission(self, pm: _PendingMission) -> None:
        """Make a staged mission the actively tracked one.

        Every write to mission/run/cursor state lives here or in _apply_run —
        _path_cb only stages. Under the serial executor this runs inline from
        _path_cb (identical to the historical behaviour); under a future
        multi-threaded executor it runs only at the top of the control tick,
        so the control loop can never observe a half-installed mission.
        """
        self._must_hit_keys = pm.must_hit_keys
        # A fresh mission re-arms every point-hold dwell (Phase D A/B).
        self._point_hold_done_keys = set()
        self._point_hold_active_key = None
        self._point_hold_start_ns = None
        self._point_servo_start_ns = None
        self._point_approach_speed = 0.0
        # G4/G5 — re-arm the point handshake for the new mission.
        self._point_spray_done_this_hold = False
        self._point_wait_start_ns = None
        self._point_done_seq_at_arm = self._point_done_seq
        self._advance_count_at_wait = self._advance_count

        self._runs = pm.runs
        # Mission-level resets; per-run state is reset inside _apply_run.
        # P0.1 — reset last speed so L_d bootstraps cleanly on new path
        self._last_speed_cmd = 0.0
        # R2 — first tick after path load must use nominal 1/CONTROL_HZ, not a
        # dt that spans the whole conditioning/load wall time.
        self._last_tick = None
        # P0.2 — reset jump guard; first pose on new path is always "valid"
        self._last_pos = None
        # A3 — start each mission with a clean reset-offset frame. The path is
        # re-anchored to the origin on every load, so any offset accumulated on
        # a prior mission must not carry over.
        self._ekf_reset_offset = (0.0, 0.0)
        self._ekf_reset_count = 0
        self._apply_run(0)
        self._publish_conditioned_path(pm.stamp, pm.frame_id)

    def _apply_run(self, idx: int, *, pre_stopped: bool = False) -> None:
        """Make run `idx` the actively tracked path; reset per-run state."""
        prev_run = self._runs[idx - 1] if idx > 0 else None
        run = self._runs[idx]
        self._run_align_pending = False
        self._run_align_turn_rad = 0.0
        if prev_run and len(prev_run["poses"]) > 1 and len(run["poses"]) > 1:
            # Pivot before this run only if the heading steps by a hard corner.
            # Connector absorption (_absorb_short_connectors) runs in
            # conditioning, so prev_run's exit heading is already the true
            # incoming leg heading — no look-through workaround needed here.
            p0 = prev_run["poses"][-2].pose.position
            p1 = prev_run["poses"][-1].pose.position
            h0 = math.atan2(p1.y - p0.y, p1.x - p0.x)
            n0 = run["poses"][0].pose.position
            n1 = run["poses"][1].pose.position
            h1 = math.atan2(n1.y - n0.y, n1.x - n0.x)
            threshold = math.radians(float(self.get_parameter("segment_corner_threshold_deg").value))
            turn = abs(self._heading_delta(h0, h1))
            if turn >= threshold:
                self._run_align_pending = True
                self._run_align_turn_rad = turn   # angle-aware pivot budget
        elif idx == 0 and len(run["poses"]) > 1 and (
            bool(self.get_parameter("entry_prealign_enabled").value)
            # Run 0 starts on a MARK: the rover MUST be on the first-segment
            # heading before spray is allowed on, or it paints while arcing onto
            # line. Force the pre-align regardless of the param in that case.
            or bool(run.get("flags") and run["flags"][0])
        ):
            # D2 — runtime-entry pre-align. Run 0 has no prev_run, so the block
            # above is skipped and the rover would ARC onto the first heading at
            # tracking speed (E2E audit gap 9). When enabled, pivot in place to
            # the first-segment heading first — spray is forced OFF throughout
            # by _run_alignment_hold. The live heading error is computed each
            # cycle there (and releases immediately if already aligned), so no
            # pose is needed here; budget the pivot watchdog for the worst case.
            self._run_align_pending = True
            self._run_align_turn_rad = math.pi
        self._reset_corner_pivot_state()
        # A hard run boundary is stopped before _advance_run(). Carry that
        # confirmation into the new run so _run_alignment_hold pivots directly
        # instead of running a duplicate CORNER_STOP after the switch.
        if pre_stopped and self._run_align_pending:
            self._corner_stop_complete = True
        self._run_boundary_stop_pending = False
        self._run_idx = idx
        self._path = run["poses"]
        self._path_s = list(run.get("cum_s", []))
        self._spray_flags = list(run["flags"])
        # G1 — cache this run's must-hit vertices (path order) + key→rank map for
        # the progress classifier. Behaviour-neutral: only read when
        # progress_publish_enabled. No-op cost is a single pass at run switch.
        self._musthit_vertices = []
        self._musthit_rank_by_key = {}
        if self._must_hit_keys:
            for i, ps in enumerate(self._path):
                p = ps.pose.position
                key = self._pt_key((p.x, p.y))
                if key in self._must_hit_keys:
                    self._musthit_rank_by_key[key] = len(self._musthit_vertices)
                    self._musthit_vertices.append(i)
        self._active_tracking_profile = run["profile"]
        self._segment_idx = 0
        self._segment_state = (
            SegmentStateCode.TRACK_SEGMENT
            if run["profile"] == "segment" and len(self._path) >= 2
            else SegmentStateCode.INACTIVE
        )
        self._path_done = False
        self._completion_stop_pending = False   # D3: clear terminal-stop latch per run/path
        self._path_travel_m = 0.0   # reset along-path progress per run
        # P1.4 — reset hint so search starts from beginning of the run
        self._closest_seg_hint = 0
        # Closed loops have first ≈ last. A full nearest-segment scan at the loop
        # seam can snap to the final segment and collapse lookahead/progress
        # before the circle is traced. New runs start at waypoint 0, so seed
        # closed loops there and let the normal projection window advance.
        self._hint_valid = bool(run.get("closed"))
        self._last_segment_debug = (
            self._profile_code(run["profile"]),
            float(self._segment_state.value),
            0.0,
            float("nan"), float("nan"), float("nan"),
            float("nan"), float("nan"), 0.0, 0.0,
        )

    def _advance_run(self, *, pre_stopped: bool = False) -> bool:
        """Switch to the next run, if any. False means mission complete."""
        if self._run_idx + 1 >= len(self._runs):
            return False
        self._apply_run(self._run_idx + 1, pre_stopped=pre_stopped)
        run = self._runs[self._run_idx]
        self.get_logger().info(
            f"Run {self._run_idx + 1}/{len(self._runs)} started "
            f"(profile={run['profile']}, {len(run['poses'])} waypoints, "
            f"{run['length']:.2f} m)"
        )
        return True

    def _next_run_turn(self) -> float:
        """Absolute heading change from the active run into the next run."""
        if self._run_idx + 1 >= len(self._runs):
            return 0.0
        current = self._runs[self._run_idx]["poses"]
        following = self._runs[self._run_idx + 1]["poses"]
        if len(current) < 2 or len(following) < 2:
            return 0.0
        a0, a1 = current[-2].pose.position, current[-1].pose.position
        b0, b1 = following[0].pose.position, following[1].pose.position
        h0 = math.atan2(a1.y - a0.y, a1.x - a0.x)
        h1 = math.atan2(b1.y - b0.y, b1.x - b0.x)
        return abs(self._angle_wrap(h1 - h0))

    def _next_run_requires_alignment(self) -> bool:
        threshold = math.radians(
            float(self.get_parameter("segment_corner_threshold_deg").value)
        )
        return self._next_run_turn() >= threshold

    def _point_done_cb(self, msg: String) -> None:
        """G4: cache the spray node's /spray/point_done (lenient parse)."""
        pd = PointDoneMsg.from_json(msg.data)
        if pd.done and pd.point_index >= 0:
            self._point_done_index = pd.point_index
            self._point_done_seq = pd.seq

    def _advance_cb(self, msg: String) -> None:
        """G5: cache the operator's /point/advance (lenient parse).

        A monotonic receive-count (not a wire seq — AdvanceMsg carries none) lets
        the WAIT_OPERATOR gate accept only a command that arrived AFTER the wait
        began, so a stale double-tap for an already-advanced point is ignored.
        """
        adv = AdvanceMsg.from_json(msg.data)
        if adv.advance:
            self._advance_index = adv.expect_index
            self._advance_count += 1

    def _point_handshake_ready(self, target_key, dwell_start_ns: int) -> bool:
        """G4/G5: decide when a handshake point-dwell may release. Default OFF.

        Phase 1 (both modes) — wait for the spray node's dwell-complete proof
        (`/spray/point_done` for this point's rank), or the point_hold_max_s
        backstop (advance anyway, warn — never wedge the mission).
        Phase 2 (manual only) — after the proof, hold in WAIT_OPERATOR until the
        operator's `/point/advance` with a matching expect_index, or the optional
        manual_wait_timeout_s.

        Only ever called under point_handshake_enabled, so the frozen fixed-timer
        point-hold path is untouched.
        """
        rank = self._musthit_rank_by_key.get(target_key, -1)
        now_ns = self.get_clock().now().nanoseconds

        # Phase 1 — the spray node's dwell-complete proof (or the backstop).
        if not self._point_spray_done_this_hold:
            fresh = (
                self._point_done_index == rank
                and self._point_done_seq > self._point_done_seq_at_arm
            )
            if fresh:
                self._point_spray_done_this_hold = True
            else:
                max_s = float(self.get_parameter("point_hold_max_s").value)
                if max_s > 0.0 and (now_ns - dwell_start_ns) * 1e-9 >= max_s:
                    self.get_logger().warn(
                        f"point handshake: no /spray/point_done for point {rank} "
                        f"within {max_s:.1f}s — proceeding (backstop)",
                        throttle_duration_sec=5.0,
                    )
                    self._point_spray_done_this_hold = True  # don't wedge
                else:
                    return False   # keep holding (phase DWELL_HOLD)

        # Proof in hand. Auto advances immediately.
        if str(self.get_parameter("point_execution_mode").value) != "manual":
            return True

        # Phase 2 (manual) — WAIT_OPERATOR until /point/advance for this point.
        if self._point_wait_start_ns is None:
            self._point_wait_start_ns = now_ns
            self._advance_count_at_wait = self._advance_count
            self.get_logger().info(
                f"point {rank}: dwell complete — waiting for operator advance"
            )
        if (self._advance_count > self._advance_count_at_wait
                and self._advance_index == rank):
            return True
        timeout = float(self.get_parameter("manual_wait_timeout_s").value)
        if timeout > 0.0 and (now_ns - self._point_wait_start_ns) * 1e-9 >= timeout:
            self.get_logger().warn(
                f"point {rank}: manual wait timeout ({timeout:.1f}s) — advancing",
                throttle_duration_sec=5.0,
            )
            return True
        return False   # keep holding (phase WAIT_OPERATOR)

    def _point_hold_tick(
        self,
        pos_n: float,
        pos_e: float,
        yaw_ned: float,
        pose_age_s: float,
        dist_to_goal: float,
    ) -> bool:
        """Phase D A/B: brake+hold at each must-hit point for its mark dwell.

        Returns True when this control cycle was fully handled (braking or
        dwelling at a point); False to let normal tracking run. Entirely gated
        by point_hold_enabled — when off it returns False on the first line, so
        the control loop is the byte-for-byte frozen baseline.

        Reuses the proven corner-brake / corner-stop-confirm primitives, so a
        point hold decelerates and confirms a physical stop exactly the way a
        run-boundary corner stop does, then holds zero for point_hold_s. The
        hold is sized (by the operator) to cover the spray node's
        arrival-settle + dwell + OFF-confirm, so the dot is painted while the
        rover is stopped here. Points already dwelled are remembered per
        mission (_point_hold_done_keys), so each is held exactly once.
        """
        if not bool(self.get_parameter("point_hold_enabled").value):
            return False
        if not self._must_hit_keys:
            return False

        acceptance = float(self.get_parameter("point_hold_acceptance_m").value)
        # G3 precise stop (default OFF → trigger == acceptance, byte-for-byte).
        # When enabled, engage the hold earlier so the feed-forward decel can
        # bring v→0 exactly at the point: trigger = max(acceptance, v²/2a).
        precise = bool(self.get_parameter("point_precise_stop_enabled").value)
        approach_speed = (
            math.hypot(self._latest_vel_ned[0], self._latest_vel_ned[1])
            if self._vel_is_fresh()
            else 0.0
        )
        trigger = acceptance
        if precise:
            trigger = pstop.feedforward_trigger_distance(
                approach_speed,
                float(self.get_parameter("precise_stop_decel_m_s2").value),
                acceptance,
            )
        target_key = None
        target_pos = None
        best = trigger
        for ps in self._path:
            p = ps.pose.position
            key = self._pt_key((p.x, p.y))
            if key not in self._must_hit_keys or key in self._point_hold_done_keys:
                continue
            d = self._dist(pos_n, pos_e, p.x, p.y)
            if d <= best:
                best = d
                target_key = key
                target_pos = (p.x, p.y)
        if target_key is None:
            # Not near any un-dwelled must-hit point → normal tracking.
            self._point_hold_active_key = None
            self._point_hold_start_ns = None
            return False

        # Entering a hold at a new point: re-arm the stop-confirm primitive once.
        if self._point_hold_active_key != target_key:
            self._point_hold_active_key = target_key
            self._point_hold_start_ns = None
            self._reset_corner_pivot_state()
            # G4/G5 — re-arm the handshake for this point (behaviour-neutral when
            # point_handshake_enabled is OFF; only read by _point_handshake_ready).
            self._point_done_seq_at_arm = self._point_done_seq
            self._point_spray_done_this_hold = False
            self._point_wait_start_ns = None
            if precise:
                # Capture the engagement speed as the feed-forward decel cap and
                # re-arm the servo timeout. Floor at creep so a slow entry still
                # crawls the last few cm in (G3).
                self._point_servo_start_ns = None
                self._point_approach_speed = max(
                    approach_speed,
                    float(self.get_parameter("precise_stop_creep_speed").value),
                )

        # Approach gate. Precise mode (G3, default OFF) decelerates onto the
        # coordinate and requires BOTH a confirmed physical stop AND the
        # along-track residual within tolerance before dwelling; the frozen path
        # requires only the corner stop-confirm. _precise_stop_ready publishes
        # its own approach setpoint while returning False.
        if precise:
            if not self._precise_stop_ready(
                target_pos, pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal
            ):
                return True
        elif not self._corner_stop_satisfied():
            # Not stopped yet — brake toward zero (same primitive as a corner).
            self._segment_state = SegmentStateCode.CORNER_STOP
            self._last_speed_cmd = 0.0
            brake_n, brake_e = self._corner_brake_velocity(yaw_ned)
            self._publish_velocity(brake_n, brake_e)
            self._publish_yaw_rate(0.0)
            self._publish_debug(
                cross_track=0.0, heading_err=0.0, lookahead=dist_to_goal,
                speed=math.hypot(brake_n, brake_e), kappa=0.0,
                dist_goal=dist_to_goal, pose_age_ms=pose_age_s * 1000.0,
                state=StateCode.TRACKING, l_d_raw=float("nan"),
                kappa_speed=0.0, yaw_rate=0.0, spray_active=False,
            )
            return True

        # Confirmed stopped — dwell.
        now_ns = self.get_clock().now().nanoseconds
        if self._point_hold_start_ns is None:
            self._point_hold_start_ns = now_ns
            self.get_logger().info(f"point hold: dwelling at must-hit {target_key}")

        # G4/G5 point handshake (default OFF). When enabled, the dwell releases on
        # the spray node's /spray/point_done (auto) — or, in manual mode, after the
        # operator's /point/advance — instead of the fixed point_hold_s timer, with
        # point_hold_max_s as a never-wedge backstop. Default OFF ⇒ the frozen
        # fixed-timer block in the `else` below runs byte-for-byte.
        if bool(self.get_parameter("point_handshake_enabled").value):
            if not self._point_handshake_ready(target_key, self._point_hold_start_ns):
                self._segment_state = SegmentStateCode.CORNER_STOP
                self._last_speed_cmd = 0.0
                self._publish_velocity(0.0, 0.0)
                self._publish_yaw_rate(0.0)
                self._publish_debug(
                    cross_track=0.0, heading_err=0.0, lookahead=dist_to_goal,
                    speed=0.0, kappa=0.0, dist_goal=dist_to_goal,
                    pose_age_ms=pose_age_s * 1000.0, state=StateCode.TRACKING,
                    l_d_raw=float("nan"), kappa_speed=0.0, yaw_rate=0.0,
                    spray_active=False,
                )
                return True
            elapsed_s = (now_ns - self._point_hold_start_ns) * 1e-9
        else:
            hold_s = float(self.get_parameter("point_hold_s").value)
            elapsed_s = (now_ns - self._point_hold_start_ns) * 1e-9
            if elapsed_s < hold_s:
                self._segment_state = SegmentStateCode.CORNER_STOP
                self._last_speed_cmd = 0.0
                self._publish_velocity(0.0, 0.0)
                self._publish_yaw_rate(0.0)
                self._publish_debug(
                    cross_track=0.0, heading_err=0.0, lookahead=dist_to_goal,
                    speed=0.0, kappa=0.0, dist_goal=dist_to_goal,
                    pose_age_ms=pose_age_s * 1000.0, state=StateCode.TRACKING,
                    l_d_raw=float("nan"), kappa_speed=0.0, yaw_rate=0.0,
                    spray_active=False,
                )
                return True

        # Dwell complete — mark done and release to normal tracking this cycle.
        self._point_hold_done_keys.add(target_key)
        self._point_hold_active_key = None
        self._point_hold_start_ns = None
        self._point_servo_start_ns = None
        self._point_wait_start_ns = None
        self._reset_corner_pivot_state()
        self.get_logger().info(
            f"point hold: released {target_key} after {elapsed_s:.1f}s "
            f"({len(self._point_hold_done_keys)}/{len(self._must_hit_keys)} done)"
        )
        return False

    def _precise_stop_ready(
        self,
        target_pos: tuple[float, float],
        pos_n: float,
        pos_e: float,
        yaw_ned: float,
        pose_age_s: float,
        dist_to_goal: float,
    ) -> bool:
        """G3 precise 2 cm stop onto a must-hit point (design §7). Default OFF.

        Returns True when the rover is confirmed stopped AND within
        point_arrival_tolerance_m of the point along-track (→ the caller
        dwells). While False it has already published this tick's approach
        setpoint:

          * feedforward (default): the kinematic decel profile v = √(2·a·d)
            toward the coordinate — open-loop, no new closed loop.
          * servo: after a coarse stop, creep at precise_stop_creep_speed until
            within tolerance, bounded by precise_stop_max_s (timeout → accept the
            best position and dwell; never wedge).

        Pure math lives in precise_stop.py; this method is the ROS glue. Called
        only under point_precise_stop_enabled, so the frozen brake-when-near path
        is untouched.
        """
        tn, te = target_pos
        residual = pstop.along_track_residual(pos_n, pos_e, tn, te, yaw_ned)
        tol = float(self.get_parameter("point_arrival_tolerance_m").value)
        stopped = self._corner_stop_satisfied()
        mode = str(self.get_parameter("precise_stop_mode").value)

        if pstop.reached(residual, tol) and stopped:
            self._point_servo_start_ns = None
            return True

        self._segment_state = SegmentStateCode.CORNER_STOP
        self._last_speed_cmd = 0.0

        if mode == "servo" and stopped:
            # Coarse stop reached but off the mark → creep, bounded by a timeout.
            now_ns = self.get_clock().now().nanoseconds
            if self._point_servo_start_ns is None:
                self._point_servo_start_ns = now_ns
            max_s = float(self.get_parameter("precise_stop_max_s").value)
            if (now_ns - self._point_servo_start_ns) * 1e-9 >= max_s:
                self.get_logger().warn(
                    f"precise stop: servo timeout ({residual * 100.0:.1f} cm "
                    f"residual) — dwelling at best position",
                    throttle_duration_sec=5.0,
                )
                self._point_servo_start_ns = None
                return True
            creep = float(self.get_parameter("precise_stop_creep_speed").value)
            v_cmd = pstop.servo_speed(residual, creep, tol)
        else:
            # Feed-forward decel profile (feedforward mode, or servo's coarse
            # phase before the physical stop is confirmed).
            decel = float(self.get_parameter("precise_stop_decel_m_s2").value)
            profile = pstop.feedforward_brake_speed(
                abs(residual), decel, self._point_approach_speed
            )
            v_cmd = math.copysign(profile, residual) if residual != 0.0 else 0.0

        vel_n, vel_e = v_cmd * math.cos(yaw_ned), v_cmd * math.sin(yaw_ned)
        self._publish_velocity(vel_n, vel_e)
        self._publish_yaw_rate(0.0)
        self._publish_debug(
            cross_track=0.0, heading_err=0.0, lookahead=dist_to_goal,
            speed=abs(v_cmd), kappa=0.0, dist_goal=dist_to_goal,
            pose_age_ms=pose_age_s * 1000.0, state=StateCode.TRACKING,
            l_d_raw=float("nan"), kappa_speed=0.0, yaw_rate=0.0,
            spray_active=False,
        )
        return False

    def _hold_before_run_advance(
        self,
        pos_n: float,
        pos_e: float,
        yaw_ned: float,
        pose_age_s: float,
        dist_to_goal: float,
    ) -> bool:
        """Physically stop at a hard run boundary, then advance exactly once.

        Returns True when the control cycle has been fully handled. Collinear
        transitions advance immediately; hard transitions latch CORNER_STOP
        until the actual speed/yaw-rate dwell passes.
        """
        if self._run_idx + 1 >= len(self._runs):
            return False
        if not self._next_run_requires_alignment():
            self._advance_run()
            return True

        if not self._run_boundary_stop_pending:
            self._reset_corner_pivot_state()
            self._run_boundary_stop_pending = True

        next_poses = self._runs[self._run_idx + 1]["poses"]
        n0, n1 = next_poses[0].pose.position, next_poses[1].pose.position
        target_heading = math.atan2(n1.y - n0.y, n1.x - n0.x)
        heading_err = self._angle_wrap(target_heading - yaw_ned)

        if self._corner_stop_satisfied():
            self._run_boundary_stop_pending = False
            self._advance_run(pre_stopped=True)
            return True

        self._segment_state = SegmentStateCode.CORNER_STOP
        self._last_speed_cmd = 0.0
        brake_n, brake_e = self._corner_brake_velocity(yaw_ned)
        brake_speed = math.hypot(brake_n, brake_e)
        self._publish_velocity(brake_n, brake_e)
        self._publish_yaw_rate(0.0)
        self._publish_debug(
            cross_track=0.0,
            heading_err=heading_err,
            lookahead=dist_to_goal,
            speed=brake_speed,
            kappa=0.0,
            dist_goal=dist_to_goal,
            pose_age_ms=pose_age_s * 1000.0,
            state=StateCode.TRACKING,
            l_d_raw=float("nan"),
            kappa_speed=0.0,
            yaw_rate=0.0,
            spray_active=False,
        )
        self._publish_segment_debug(
            SegmentStateCode.CORNER_STOP,
            max(0, len(self._path) - 2),
            0.0,
            dist_to_goal,
            math.degrees(self._next_run_turn()),
            target_heading,
            heading_err,
            0.0,
        )
        return True

    def _hold_at_completion(
        self,
        pos_n: float,
        pos_e: float,
        yaw_ned: float,
        pose_age_s: float,
        dist_to_goal: float,
    ) -> None:
        """Brake to a confirmed physical stop at the final waypoint, then DONE.

        The terminal twin of _hold_before_run_advance, built from the SAME two
        primitives the run-boundary stop and the D0 bench both proved:
        _corner_brake_velocity (body-axis brake only — invariant I1, never an
        off-nose recenter vector) and _corner_stop_satisfied (measured
        speed+yaw-rate dwell — invariant I3). It deliberately does NOT introduce
        a parallel completion-settle mechanism (the ref branch's
        _completion_settle_satisfied); it reuses the corner-stop confirm the
        rest of the controller already trusts.

        Why it exists: PX4 velocity-OFFBOARD coasts on a bare zero setpoint.
        The old completion published zero the instant dist_to_goal <=
        xy_goal_tolerance, the rover drifted past, the goal check flipped false,
        control fell through to tracking, and it accelerated away from a goal it
        had already reached (bag 2026-07-10_20-07 Line_2m: 1.08 m run-past).

        Latch on first entry, brake actively, and publish DONE only once the
        rover is confirmed stopped. The latch is cleared per run/path in
        _apply_run and is checked BEFORE the goal test in _control_loop, so a
        coast can never route control back to tracking.
        """
        if not self._completion_stop_pending:
            self._reset_corner_pivot_state()
            self._completion_stop_pending = True

        if self._corner_stop_satisfied():
            goal_tol = float(self.get_parameter("xy_goal_tolerance").value)
            self.get_logger().info(
                f"Path complete — settled {dist_to_goal * 100:.1f} cm from the "
                f"final point (tol={goal_tol * 100:.1f} cm)"
            )
            self._path_done = True
            self._segment_state = SegmentStateCode.DONE
            self._publish_zero(
                StateCode.DONE,
                pose_age_ms=pose_age_s * 1000.0,
                dist_to_goal=dist_to_goal,
            )
            return

        # Not yet stopped: active body-axis brake, exactly as the run-boundary
        # stop. Publish CORNER_STOP on segment_debug and a non-DONE tracking
        # state on /rpp/debug so the server's mission-complete watcher does not
        # latch DONE until the rover is physically stopped. Spray forced OFF.
        self._segment_state = SegmentStateCode.CORNER_STOP
        self._last_speed_cmd = 0.0
        brake_n, brake_e = self._corner_brake_velocity(yaw_ned)
        self._publish_velocity(brake_n, brake_e)
        self._publish_yaw_rate(0.0)
        self._publish_debug(
            cross_track=0.0,
            heading_err=0.0,
            lookahead=dist_to_goal,
            speed=math.hypot(brake_n, brake_e),
            kappa=0.0,
            dist_goal=dist_to_goal,
            pose_age_ms=pose_age_s * 1000.0,
            state=StateCode.TRACKING,
            l_d_raw=float("nan"),
            kappa_speed=0.0,
            yaw_rate=0.0,
            spray_active=False,
        )
        self._publish_segment_debug(
            SegmentStateCode.CORNER_STOP,
            max(0, len(self._path) - 2),
            0.0,
            dist_to_goal,
            float("nan"),
            float("nan"),
            float("nan"),
            0.0,
        )

    def _is_closed_run(self, pts: list[tuple[float, float]]) -> bool:
        """True when a run is a closed loop (e.g. a circle entity).

        A closed run has its first and last waypoint within
        close_loop_threshold_m, AND a total length above close_loop_min_len_m
        (so a short stub that happens to fold back is not mistaken for a loop).
        Closed runs need a circumference-based completion guard — see
        _run_min_travel — because their endpoints coincide, defeating the
        Euclidean dist-to-goal check.
        """
        if len(pts) < 3:
            return False
        thr = float(self.get_parameter("close_loop_threshold_m").value)
        min_len = float(self.get_parameter("close_loop_min_len_m").value)
        if self._pts_length(pts) < min_len:
            return False
        gap = math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1])
        return gap <= thr

    def _run_min_travel(self) -> float:
        """Along-path travel required before the active run may declare DONE.

        Open run: min_goal_travel_m, capped at half the run length. The param
        guards against instant-DONE on missions where the rover starts near
        the goal; the cap keeps a short entity/transit hop from deadlocking
        (its half-length can be below the param).

        Closed run (circle, closed polyline): first ≈ last waypoint, so the
        Euclidean dist-to-goal is small from the very start and the open-path
        cap (0.5·len) lets the loop "complete" after ~half a metre without
        tracing it. Require closed_loop_min_travel_frac of the FULL run length
        instead, forcing the rover to traverse nearly the whole circumference.
        """
        min_travel = float(self.get_parameter("min_goal_travel_m").value)
        if not self._runs:
            return min_travel
        run = self._runs[self._run_idx]
        length = float(run["length"])
        if run.get("closed"):
            frac = float(self.get_parameter("closed_loop_min_travel_frac").value)
            return frac * length
        return min(min_travel, 0.5 * length)

    def _path_progress_at(self, seg_idx: int, t: float) -> float:
        """Return along-run arc length for a segment projection."""
        if not self._path_s or len(self._path_s) != len(self._path):
            return 0.0
        if len(self._path_s) == 1:
            return 0.0
        i = max(0, min(int(seg_idx), len(self._path_s) - 2))
        alpha = self._clamp(float(t), 0.0, 1.0)
        return self._path_s[i] + alpha * (self._path_s[i + 1] - self._path_s[i])

    def _update_path_progress(self, seg_idx: int, t: float) -> None:
        """Advance monotonic along-path progress; never count local spin as travel."""
        self._path_travel_m = max(self._path_travel_m, self._path_progress_at(seg_idx, t))

    def _run_alignment_hold(
        self, pos_n: float, pos_e: float, yaw_ned: float, pose_age_s: float
    ) -> bool:
        """Pivot in place toward the new run's initial heading.

        Returns True while stopping, pivoting, or settling;
        False once aligned (or alignment is not applicable), letting the
        normal control flow proceed. Already-aligned transitions (e.g. a
        transit continuing straight into an entity) pass through with no
        stop. Like segment CORNER_ALIGN, the firmware-aware velocity-vector
        pivot is used here; use_feedforward_yaw_rate is intentionally ignored.
        """
        if not self._run_align_pending:
            return False
        if len(self._path) < 2:
            self._run_align_pending = False
            return False
        a = self._path[0].pose.position
        b = self._path[1].pose.position
        if self._dist(a.x, a.y, b.x, b.y) < 1e-6:
            self._run_align_pending = False
            return False

        target_heading = math.atan2(b.y - a.y, b.x - a.x)
        heading_err = self._angle_wrap(target_heading - yaw_ned)
        heading_tol = math.radians(
            float(self.get_parameter("segment_heading_tolerance_deg").value)
        )
        timed_out = self._corner_stop_complete and self._pivot_timed_out(
            self._run_align_turn_rad
        )
        yaw_rate_tol = float(self.get_parameter("segment_stop_yaw_rate_threshold").value)
        align_settle_s = float(self.get_parameter("segment_align_settle_s").value)
        now_align = self.get_clock().now()
        release_heading_tol = heading_tol
        if timed_out:
            release_heading_tol = max(
                heading_tol,
                math.radians(float(self.get_parameter(
                    "segment_timeout_heading_tolerance_deg"
                ).value)),
            )
        # Hard release gate: never launch onto the next leg while still grossly
        # mis-headed, even if the timeout band relaxed. Backstops a mis-set
        # relaxed tolerance from accelerating a large residual into TRACK.
        release_heading_tol = min(
            release_heading_tol,
            math.radians(float(self.get_parameter("segment_pivot_release_max_deg").value)),
        )
        heading_ok = abs(heading_err) <= release_heading_tol
        vel_fresh = self._vel_is_fresh()
        # Fresh telemetry is mandatory during normal alignment. If the
        # velocity topic disappears, the angle-aware pivot watchdog is the
        # bounded fallback: pose heading is still fresh (enforced by the outer
        # control loop), so allow the settle dwell after the timeout instead of
        # deadlocking CORNER_ALIGN forever on an unavailable velocity sample.
        yaw_rate_ok = (
            abs(self._latest_yaw_rate_ned) < yaw_rate_tol
            if vel_fresh else timed_out
        )
        speed_ok = self._align_speed_ok() if vel_fresh else timed_out
        if self._corner_stop_complete and heading_ok and yaw_rate_ok and speed_ok:
            if self._align_settle_since is None:
                self._align_settle_since = now_align
            settled = (now_align - self._align_settle_since).nanoseconds * 1e-9 >= align_settle_s
        else:
            self._align_settle_since = None
            settled = False
        if settled:
            self._run_align_pending = False
            self._last_speed_cmd = 0.0
            self._reset_corner_pivot_state()
            return False

        final = self._path[-1].pose.position
        dist_to_goal = self._dist(pos_n, pos_e, final.x, final.y)

        # As soon as heading enters the release band, remove the pivot vector
        # and physically settle. Continuing to command corner_speed here would
        # keep linear speed above the release threshold until the watchdog.
        if self._corner_stop_complete and heading_ok:
            self._last_speed_cmd = 0.0
            brake_n, brake_e = self._corner_brake_velocity(yaw_ned)
            self._publish_velocity(brake_n, brake_e)
            self._publish_yaw_rate(0.0)
            self._publish_debug(
                cross_track=0.0,
                heading_err=heading_err,
                lookahead=float("nan"),
                speed=math.hypot(brake_n, brake_e),
                kappa=0.0,
                dist_goal=dist_to_goal,
                pose_age_ms=pose_age_s * 1000.0,
                state=StateCode.TRACKING,
                l_d_raw=float("nan"),
                kappa_speed=0.0,
                yaw_rate=0.0,
                spray_active=False,
            )
            self._publish_segment_debug(
                SegmentStateCode.CORNER_ALIGN, 0, float("nan"), float("nan"),
                float("nan"), target_heading, heading_err, 0.0,
            )
            return True

        # Stop-and-spin: hold zero velocity at the corner until the rover is
        # physically stopped (approach momentum gone), THEN pivot. Without
        # this the residual ~0.09 m/s arrival speed carries the rover past
        # the corner point during the first part of the turn.
        if not self._corner_stop_complete:
            if not self._corner_stop_satisfied():
                # Active braking: PX4 coasts on a zero setpoint, so command a
                # small velocity opposing the rover's motion to truly stop it at
                # the corner point before pivoting. (0,0) when already stopped or
                # velocity is stale.
                self._last_speed_cmd = 0.0
                brake_n, brake_e = self._corner_brake_velocity(yaw_ned)
                self._publish_velocity(brake_n, brake_e)
                self._publish_yaw_rate(0.0)
                self._publish_debug(
                    cross_track=0.0,
                    heading_err=heading_err,
                    lookahead=float("nan"),
                    speed=math.hypot(brake_n, brake_e),
                    kappa=0.0,
                    dist_goal=dist_to_goal,
                    pose_age_ms=pose_age_s * 1000.0,
                    state=StateCode.TRACKING,
                    l_d_raw=float("nan"),
                    kappa_speed=0.0,
                    yaw_rate=0.0,
                    spray_active=False,
                )
                self._publish_segment_debug(
                    SegmentStateCode.CORNER_STOP, 0, float("nan"), float("nan"),
                    float("nan"), target_heading, heading_err, 0.0,
                )
                return True
            self._corner_stop_complete = True

        # Firmware-aware pivot (see segment CORNER_ALIGN for the full
        # rationale): the differential rover turns by chasing the velocity-
        # vector bearing, not the MAVROS yaw_rate field, and freezes heading
        # below 0.01 m/s. Command a small velocity vector at the run's initial
        # heading (forward-cone clamped) so the firmware spot-turns in place
        # the short way to it, then rolls out.
        corner_speed = max(
            0.05, float(self.get_parameter("segment_min_corner_speed").value)
        )
        v_n, v_e = self._corner_pivot_velocity(yaw_ned, heading_err, corner_speed)

        self._last_speed_cmd = corner_speed
        self._publish_velocity(v_n, v_e)
        self._publish_yaw_rate(0.0)
        self._publish_debug(
            cross_track=0.0,
            heading_err=heading_err,
            lookahead=float("nan"),
            speed=corner_speed,
            kappa=0.0,
            dist_goal=dist_to_goal,
            pose_age_ms=pose_age_s * 1000.0,
            state=StateCode.TRACKING,
            l_d_raw=float("nan"),
            kappa_speed=0.0,
            yaw_rate=0.0,
            spray_active=False,   # spray only once tracking the run begins
        )
        self._publish_segment_debug(
            SegmentStateCode.CORNER_ALIGN, 0, float("nan"), float("nan"),
            float("nan"), target_heading, heading_err, 0.0,
        )
        return True

    def _segment_angle_deg(self, idx: int) -> float:
        if idx < 0 or idx + 2 >= len(self._path):
            return float("nan")
        a = self._path[idx].pose.position
        b = self._path[idx + 1].pose.position
        c = self._path[idx + 2].pose.position
        h0 = math.atan2(b.y - a.y, b.x - a.x)
        h1 = math.atan2(c.y - b.y, c.x - b.x)
        return math.degrees(self._heading_delta(h0, h1))

    def _segment_lookahead_point(
        self,
        seg_idx: int,
        foot_n: float,
        foot_e: float,
        l_d: float,
        max_junction_deg: float,
    ) -> tuple[float, float]:
        """Place the lookahead point `l_d` ahead of the foot along the path,
        crossing a vertex only while the turn there is collinear to within
        `max_junction_deg`. Stops at the first real corner, or at path end.

        Why this exists — segment mode used to clip the lookahead at the
        current segment's end:

            lookahead_along = min(max(0.0, dist_to_end_along), l_d)

        so `l_d` was silently overridden by whatever the segment happened to
        be, and decayed to ~0 as the rover approached each vertex. Note it
        bypassed `min_lookahead_dist` (0.52) entirely — the floor could not
        protect it.

        That is fine at a real corner, where the rover is about to stop and
        pivot anyway and MUST NOT steer toward geometry past the turn. It is
        wrong at a COLLINEAR vertex, which is not a corner at all.
        `_simplify_path_for_profile` deletes collinear fill, so the collinear
        vertices that survive into segment mode are exactly the deliberate
        anchors: spray-flag boundaries and declared must-hit points. Clipping
        there truncates the lookahead for no geometric reason.

        Field evidence 2026-07-30, bag stg_95d11205_..._154238 (tes_cross_line_2,
        0.5 m extensions). The extension leg is split 0.375 m + 0.125 m by the
        flag-boundary anchor, so the lookahead ran 0.286 -> 0.187 -> 0.054 m
        across the approach and then jumped to 0.560 m on the marked line.
        Pure-pursuit steering gain goes as 1/L^2, so the rover hit the line
        under roughly a hundredfold gain spike, overshot from -2.2 cm to
        +5.2 cm, and did not recover until s = 1.4 m — well inside the paint.
        The valve correctly shut at 5.1 cm and the line came out in two pieces.
        Its sibling run 154444 entered without the crossing rate and stayed
        under 1.4 cm.

        A LONGER extension does not fix this: extra length arrives as more
        short legs, and the truncation is per-segment. This is the same root
        cause as the 2026-07-29 must-hit failure (short segments capping the
        lookahead into marginal steering stability) — third instance.

        `max_junction_deg <= 0.0` restores the pre-fix geometry exactly and is
        the A/B arm. Reduction is exact, not approximate: with the walk
        disabled, or when `l_d` fits inside the current segment, or at a
        non-collinear junction, this returns precisely what the old two-line
        form returned. Only the collinear-crossing case is new.

        ⚠ FIELD-UNVERIFIED at the time of writing. Real corners are untouched
        by construction (they exceed the threshold and terminate the walk), so
        the blast radius is paths carrying flag/must-hit anchors on a straight
        run — i.e. any marked line with extensions.
        """
        n_pts = len(self._path)
        if n_pts < 2:
            p = self._path[0].pose.position
            return p.x, p.y

        remaining = max(0.0, l_d)
        cur = max(0, min(seg_idx, n_pts - 2))
        pn, pe = foot_n, foot_e

        while cur + 1 < n_pts:
            b = self._path[cur + 1].pose.position
            seg_rem = self._dist(pn, pe, b.x, b.y)
            if seg_rem >= remaining:
                if seg_rem <= 1e-9:
                    return b.x, b.y
                f = remaining / seg_rem
                return pn + (b.x - pn) * f, pe + (b.y - pe) * f

            # The lookahead outruns this segment. Crossing the vertex at
            # cur+1 is only legitimate when it is not a corner.
            if max_junction_deg <= 0.0:
                return b.x, b.y
            angle = self._segment_angle_deg(cur)
            # NaN means cur+1 is the last vertex — nothing left to cross into.
            if not (abs(angle) <= max_junction_deg):
                return b.x, b.y

            remaining -= seg_rem
            pn, pe = b.x, b.y
            cur += 1

        last = self._path[n_pts - 1].pose.position
        return last.x, last.y

    def _project_onto_segment(self, pos_n: float, pos_e: float, seg_idx: int):
        n_pts = len(self._path)
        if n_pts == 1:
            wp = self._path[0].pose.position
            d = self._dist(pos_n, pos_e, wp.x, wp.y)
            return 0.0, wp.x, wp.y, 0.0, d

        seg_idx = max(0, min(seg_idx, n_pts - 2))
        a = self._path[seg_idx].pose.position
        b = self._path[seg_idx + 1].pose.position
        dx = b.x - a.x
        dy = b.y - a.y
        seg_sq = dx * dx + dy * dy
        if seg_sq < 1e-12:
            d = self._dist(pos_n, pos_e, a.x, a.y)
            return 0.0, a.x, a.y, 0.0, d

        t_raw = ((pos_n - a.x) * dx + (pos_e - a.y) * dy) / seg_sq
        t = self._clamp(t_raw, 0.0, 1.0)
        foot_n = a.x + t * dx
        foot_e = a.y + t * dy
        seg_len = math.sqrt(seg_sq)

        # Signed perpendicular offset from this segment's INFINITE line.
        #
        # The 2D cross product of the segment direction with (pos − foot)
        # annihilates any along-track component, so cross_z / seg_len is the
        # exact perpendicular distance even when t was clamped and `foot` is
        # a vertex rather than the true perpendicular foot. For 0 < t < 1 it
        # is identical to the old copysign(|pos − foot|, cross_z) — this is a
        # no-op in the segment interior.
        #
        # It is NOT a no-op at the ends, which is the point. The old form used
        # |pos − foot| as the magnitude, so a clamped t made the ALONG-track
        # gap to the vertex masquerade as cross-track. That fires on every
        # segment handover: seg_idx advances one control cycle before the
        # rover physically reaches the vertex, t_raw goes negative, and for
        # exactly one cycle the reported cross-track jumps to the remaining
        # along-track distance.
        #
        # Field evidence 2026-07-30, bags stg_1bda4c36_..._151006 and
        # stg_bbb8e1a5_..._152342 (tes_cross_line_2, 0.5 m extensions, RTK
        # fixed, segment profile). Four spikes per run at the four segment
        # boundaries: reported −4.88 / −4.44 / +4.85 / +4.65 cm while the true
        # offset from the surveyed line was −0.34 / −0.15 / +0.44 / +0.12 cm.
        # The magnitude matched the along-track gap every time — at 151006's
        # exit, dist_to_end_along was 0.068 m one sample before the switch, so
        # at 0.35 m/s and 0.1 s/cycle ~0.033 m remained, reported as 3.42 cm.
        # Those paths are perfectly COLLINEAR, so no genuine cross-track can
        # exist at a handover at all.
        #
        # `t`, `foot_*` and `dist_to_end_along` deliberately stay clamped: the
        # segment state machine and the lookahead placement (which walks
        # forward from `foot`) both require a point ON the segment.
        cross_z = dx * (pos_e - foot_e) - dy * (pos_n - foot_n)
        signed_e = cross_z / seg_len
        dist_to_end_along = (1.0 - t) * seg_len
        return t, foot_n, foot_e, signed_e, dist_to_end_along

    def _path_curvature_at(self, seg_idx: int, baseline_m: float = 0.0) -> float:
        """Estimate path curvature at the projection foot (Menger curvature).

        Returns 1/m curvature (0.0 for straight lines, >0 for curves).

        `baseline_m` is the HALF-baseline: the neighbours are walked outward
        until they are at least this far along the path from the centre point.
        0.0 keeps the legacy adjacent-vertex behaviour.

        ⚠ Adjacent vertices are NOT safe for anything quantitative. Conditioned
        paths arrive densified at 4-10 cm, and a three-point circle on that
        spacing is dominated by coordinate noise: on the 2026-07-30 curve bags
        it reported kappa_max 1.251 (R = 0.80 m) where the true geometry was a
        near-constant R = 2.3-2.4 m arc — a 3x error that produced a completely
        wrong diagnosis (an imagined yaw-rate saturation) until a field test
        disproved it. Convergence measured on that path: half-baseline 0.05 m
        -> 0.626, 0.10 -> 0.417, 0.20 -> 0.450, 0.30 -> 0.430, 0.50 -> 0.422.
        It is stable from ~0.10 m up, so any consumer that scales a control
        quantity by curvature must pass a baseline of at least that.
        """
        n_pts = len(self._path)
        if n_pts < 3:
            return 0.0
        i1 = max(0, min(seg_idx, n_pts - 1))
        i0, i2 = max(0, i1 - 1), min(n_pts - 1, i1 + 1)
        if baseline_m > 0.0:
            p1 = self._path[i1].pose.position
            # Walk outward on arc length, not index, so the baseline is
            # independent of how finely the path was densified.
            d = 0.0
            while i0 > 0 and d < baseline_m:
                a = self._path[i0].pose.position
                b = self._path[i0 + 1].pose.position
                d += math.hypot(b.x - a.x, b.y - a.y)
                i0 -= 1
            d = 0.0
            while i2 < n_pts - 1 and d < baseline_m:
                a = self._path[i2].pose.position
                b = self._path[i2 - 1].pose.position
                d += math.hypot(b.x - a.x, b.y - a.y)
                i2 += 1
            del p1
        if i2 - i0 < 2:
            return 0.0
        a = self._path[i0].pose.position
        b = self._path[i1].pose.position
        c = self._path[i2].pose.position
        ab = math.hypot(b.x - a.x, b.y - a.y)
        bc = math.hypot(c.x - b.x, c.y - b.y)
        ca = math.hypot(a.x - c.x, a.y - c.y)
        if ab < 1e-6 or bc < 1e-6 or ca < 1e-6:
            return 0.0
        area2 = abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))
        return (2.0 * area2) / (ab * bc * ca)

    # ==================================================================
    # P1.3 — Path conditioning helpers
    # ==================================================================
    @staticmethod
    def _resample_path(pts: list[tuple[float, float]],
                       spacing: float,
                       flags: list[bool] | None = None
                       ) -> list[tuple[float, float]] | tuple[list[tuple[float, float]], list[bool]]:
        """Linearly resample a polyline to uniform spacing.

        The first and last points are kept exactly; intermediate samples are
        placed every `spacing` metres along the cumulative arc length.
        Geometry is preserved (straight segments stay straight).
        """
        carry_flags = flags is not None
        if flags is None:
            flags = [False] * len(pts)
        elif len(flags) != len(pts):
            flags = [False] * len(pts)

        if len(pts) < 2 or spacing <= 0.0:
            out_pts = list(pts)
            out_flags = list(flags)
            return (out_pts, out_flags) if carry_flags else out_pts

        # Cumulative arc length per vertex
        cum = [0.0]
        for i in range(1, len(pts)):
            cum.append(cum[-1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                            pts[i][1] - pts[i - 1][1]))
        total = cum[-1]
        if total < spacing:
            out_pts = [pts[0], pts[-1]]
            out_flags = [bool(flags[0]), bool(flags[-1])]
            return (out_pts, out_flags) if carry_flags else out_pts

        n_samples = max(2, int(math.ceil(total / spacing)) + 1)
        out: list[tuple[float, float]] = []
        out_flags: list[bool] = []
        seg = 0
        for k in range(n_samples):
            target = (k / (n_samples - 1)) * total
            # Advance segment pointer
            while seg + 1 < len(cum) - 1 and cum[seg + 1] < target:
                seg += 1
            seg_len = cum[seg + 1] - cum[seg]
            if seg_len < 1e-12:
                out.append(pts[seg])
                out_flags.append(bool(flags[seg]))
                continue
            t = (target - cum[seg]) / seg_len
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            n = pts[seg][0] + t * (pts[seg + 1][0] - pts[seg][0])
            e = pts[seg][1] + t * (pts[seg + 1][1] - pts[seg][1])
            out.append((n, e))
            if k == 0:
                out_flags.append(bool(flags[0]))
            elif k == n_samples - 1:
                out_flags.append(bool(flags[-1]))
            else:
                out_flags.append(bool(flags[seg] and flags[seg + 1]))
        # Force exact endpoints
        out[0] = pts[0]
        out[-1] = pts[-1]
        out_flags[0] = bool(flags[0])
        out_flags[-1] = bool(flags[-1])
        return (out, out_flags) if carry_flags else out

    def _smooth_corners(self, pts: list[tuple[float, float]],
                        radius: float,
                        arc_pts: int,
                        flags: list[bool] | None = None
                        ) -> list[tuple[float, float]] | tuple[list[tuple[float, float]], list[bool]]:
        """Replace each interior vertex with an inscribed circular arc.

        Bounds path curvature at κ_max = 1/radius.

        For each interior vertex P with neighbours A (before) and B (after),
        compute the inscribed arc tangent to AP and PB at distance d from P.
        d = radius / tan(theta/2), where theta is the interior angle.
        Vertices where d > 0.45 * min(|AP|, |PB|) are skipped (segments too
        short to support the arc) and a warning is logged.
        Endpoints are always kept.
        """
        carry_flags = flags is not None
        if flags is None:
            flags = [False] * len(pts)
        elif len(flags) != len(pts):
            flags = [False] * len(pts)

        n = len(pts)
        if n < 3 or radius <= 0.0:
            out_pts = list(pts)
            out_flags = list(flags)
            return (out_pts, out_flags) if carry_flags else out_pts

        out: list[tuple[float, float]] = [pts[0]]
        out_flags: list[bool] = [bool(flags[0])]
        skipped = 0
        for i in range(1, n - 1):
            ax, ay = pts[i - 1]
            px, py = pts[i]
            bx, by = pts[i + 1]
            v1n, v1e = ax - px, ay - py   # P→A direction (incoming reversed)
            v2n, v2e = bx - px, by - py   # P→B direction
            l1 = math.hypot(v1n, v1e)
            l2 = math.hypot(v2n, v2e)
            if l1 < 1e-9 or l2 < 1e-9:
                continue

            # Unit vectors away from P
            u1n, u1e = v1n / l1, v1e / l1
            u2n, u2e = v2n / l2, v2e / l2

            # Half angle of the bend: cos(theta) = u1·u2
            dot = u1n * u2n + u1e * u2e
            dot = max(-1.0, min(1.0, dot))
            theta = math.acos(dot)             # interior angle, 0..pi
            if theta < 1e-3 or math.pi - theta < 1e-3:
                # Nearly collinear; no smoothing needed, no chord taken
                out.append(pts[i])
                out_flags.append(bool(flags[i]))
                continue

            # Tangent length from P to the arc start
            d = radius / math.tan(theta / 2.0)
            if d > 0.45 * min(l1, l2):
                # Segments too short for this radius — keep sharp corner
                skipped += 1
                out.append(pts[i])
                out_flags.append(bool(flags[i]))
                continue

            # Arc start (toward A) and end (toward B)
            sa_n = px + d * u1n
            sa_e = py + d * u1e
            sb_n = px + d * u2n
            sb_e = py + d * u2e

            # Arc centre is at distance R from P along the bisector,
            # on the inside of the bend. Bisector direction = (u1+u2)/|u1+u2|.
            bx_n = u1n + u2n
            bx_e = u1e + u2e
            bl = math.hypot(bx_n, bx_e)
            if bl < 1e-9:
                out.append(pts[i])
                out_flags.append(bool(flags[i]))
                continue
            bx_n /= bl
            bx_e /= bl
            # Distance from P to centre along the bisector:
            # |PC| = R / sin(theta/2)
            pc = radius / math.sin(theta / 2.0)
            cx_n = px + pc * bx_n
            cx_e = py + pc * bx_e

            # Sweep angle equals (pi - theta), going from sa to sb around C.
            # Determine sweep sign from cross product of (C→sa) × (C→sb).
            r1n = sa_n - cx_n
            r1e = sa_e - cx_e
            r2n = sb_n - cx_n
            r2e = sb_e - cx_e
            ang1 = math.atan2(r1e, r1n)
            ang2 = math.atan2(r2e, r2n)
            cross_z = r1n * r2e - r1e * r2n
            sweep = ang2 - ang1
            if cross_z >= 0:
                if sweep < 0:
                    sweep += 2.0 * math.pi
            else:
                if sweep > 0:
                    sweep -= 2.0 * math.pi

            # Discretise the arc
            arc_flag = bool(flags[i - 1] and flags[i] and flags[i + 1])
            out.append((sa_n, sa_e))
            out_flags.append(arc_flag)
            for k in range(1, arc_pts):
                a = ang1 + sweep * (k / arc_pts)
                out.append((cx_n + radius * math.cos(a),
                            cx_e + radius * math.sin(a)))
                out_flags.append(arc_flag)
            out.append((sb_n, sb_e))
            out_flags.append(arc_flag)

        out.append(pts[-1])
        out_flags.append(bool(flags[-1]))
        if skipped > 0:
            self.get_logger().warn(
                f"corner_smooth: skipped {skipped} vertices — "
                f"adjacent segments shorter than {radius:.2f} m allows. "
                f"Reduce corner_smooth_radius_m or densify the path."
            )
        return (out, out_flags) if carry_flags else out

    # ==================================================================
    # P1.1 — Predictive curvature (path-intrinsic Menger curvature)
    # ==================================================================
    def _walk_path_samples(self, seg_idx: int,
                           foot_n: float, foot_e: float,
                           targets: list[float]
                           ) -> list[tuple[float, float, bool]]:
        """Single-pass walk: emit (n, e, hit_end) for each cumulative arc
        length in `targets` (must be sorted ascending).

        B3 perf: O(path_length + len(targets)) total instead of
        O(path_length * len(targets)) — replaces N independent walks from
        the projection foot with one shared walk.
        """
        out: list[tuple[float, float, bool]] = []
        if not targets:
            return out

        n_pts = len(self._path)
        if n_pts == 0:
            return [(foot_n, foot_e, True) for _ in targets]
        if n_pts == 1:
            wp = self._path[0].pose.position
            return [(wp.x, wp.y, True) for _ in targets]

        # Initial sub-segment: from foot to end of seg_idx
        if seg_idx + 1 < n_pts:
            end_n = self._path[seg_idx + 1].pose.position.x
            end_e = self._path[seg_idx + 1].pose.position.y
        else:
            end_n = self._path[seg_idx].pose.position.x
            end_e = self._path[seg_idx].pose.position.y

        prev_n, prev_e = foot_n, foot_e
        next_n, next_e = end_n, end_e
        arc = 0.0
        i = seg_idx + 1
        t_idx = 0
        finished = False

        while t_idx < len(targets):
            target = targets[t_idx]
            seg_len = self._dist(prev_n, prev_e, next_n, next_e)
            if finished:
                # Path exhausted; clamp remaining targets to final waypoint
                final = self._path[-1].pose.position
                while t_idx < len(targets):
                    out.append((final.x, final.y, True))
                    t_idx += 1
                break

            if arc + seg_len >= target:
                # Interpolate inside current sub-segment
                remaining = target - arc
                ratio = remaining / seg_len if seg_len > 1e-9 else 1.0
                ratio = 0.0 if ratio < 0.0 else (1.0 if ratio > 1.0 else ratio)
                lh_n = prev_n + ratio * (next_n - prev_n)
                lh_e = prev_e + ratio * (next_e - prev_e)
                out.append((lh_n, lh_e, False))
                t_idx += 1
                # Loop back without advancing — next target may be in same seg
                continue

            # Advance to next sub-segment
            arc += seg_len
            i += 1
            if i >= n_pts:
                finished = True
                continue
            prev_n, prev_e = next_n, next_e
            next_n = self._path[i].pose.position.x
            next_e = self._path[i].pose.position.y

        return out

    def _max_preview_curvature(self, seg_idx: int,
                               foot_n: float, foot_e: float,
                               l_d: float, n_previews: int) -> float:
        """Return the worst |κ| at N preview points along the path ahead.

        Path-intrinsic Menger curvature, computed from three samples per
        preview at distances (k-0.5)L_d, k*L_d, (k+0.5)L_d, k=1..N.
        Independent of the rover's current pose.

        B3: Uses `_walk_path_samples` to do a single forward walk through
        the path geometry instead of N independent walks (was O(P*N), now
        O(P+N) where P is the path length).
        """
        if n_previews <= 1 or l_d <= 0.0:
            return 0.0

        # Build sorted target list: per preview k, three samples at
        # (k-0.5)L_d, k*L_d, (k+0.5)L_d. Already monotonic in k.
        half = 0.5 * l_d
        targets: list[float] = []
        for k in range(1, n_previews + 1):
            centre = k * l_d
            targets.append(max(0.05, centre - half))  # never sample at foot
            targets.append(centre)
            targets.append(centre + half)
        # `targets` is already sorted as long as 0.05 < l_d (true after the
        # gate above for any sensible l_d), so no sort needed.

        samples = self._walk_path_samples(seg_idx, foot_n, foot_e, targets)
        if len(samples) != len(targets):
            return 0.0  # shouldn't happen, but defensive

        kappa_max = 0.0
        for k in range(n_previews):
            p_a = samples[3 * k + 0]
            p_b = samples[3 * k + 1]
            p_c = samples[3 * k + 2]
            # If the middle and last sample both ran off the end of the
            # path, this preview adds no information — and further previews
            # are even further out, so stop.
            if p_b[2] and p_c[2]:
                break
            kab = math.hypot(p_b[0] - p_a[0], p_b[1] - p_a[1])
            kbc = math.hypot(p_c[0] - p_b[0], p_c[1] - p_b[1])
            kca = math.hypot(p_a[0] - p_c[0], p_a[1] - p_c[1])
            if kab < 1e-6 or kbc < 1e-6 or kca < 1e-6:
                continue
            area2 = abs((p_b[0] - p_a[0]) * (p_c[1] - p_a[1])
                        - (p_b[1] - p_a[1]) * (p_c[0] - p_a[0]))
            kappa = (2.0 * area2) / (kab * kbc * kca)
            if kappa > kappa_max:
                kappa_max = kappa
        return kappa_max

    def _project_onto_path(self, pos_n: float, pos_e: float):
        """Find closest point on the path as a *segment projection*.

        P1.4: Uses _closest_seg_hint to start the search from the previous
        best segment instead of i=0. In steady state this is O(1) — only 6
        segments are checked. On path discontinuities (re-plan, big jump)
        the hint is already reset to 0 in _path_cb.

        Returns:
          (seg_idx, t, foot_n, foot_e, signed_cross_track_m)
          seg_idx is the segment starting at path[seg_idx].
          t is the segment parameter [0, 1].
          (foot_n, foot_e) is the projection point in NED.
          signed_cross_track_m: + means rover is to the *right* of the path
                                heading direction (NED, viewed from above).
        """
        n_pts = len(self._path)
        if n_pts == 1:
            wp = self._path[0].pose.position
            d = self._dist(pos_n, pos_e, wp.x, wp.y)
            return 0, 0.0, wp.x, wp.y, d  # sign undefined for single point

        # P1.4: windowed search centred on the previous closest segment.
        # Window: [hint-2, hint+4) — wide enough to handle 0.4 m/s at 50 Hz
        # (0.008 m per cycle; a 25 cm segment takes ~30 cycles to traverse).
        # On the very first cycle after a path reset or EKF jump
        # (_hint_valid=False), do a full O(n) scan so we lock onto the correct
        # segment immediately. After that, windowed search is O(1) in steady
        # state.
        if not self._hint_valid:
            lo, hi = 0, n_pts - 1
        else:
            lo = max(0, self._closest_seg_hint - 2)
            hi = min(n_pts - 1, self._closest_seg_hint + 4)
            # Widen to full scan when window is too narrow (short paths)
            if hi - lo < 3:
                lo, hi = 0, n_pts - 1

        best = (lo, 0.0,
                self._path[lo].pose.position.x,
                self._path[lo].pose.position.y,
                float("inf"), 0.0)
        # best = (seg_idx, t, foot_n, foot_e, dist, signed_e)

        for i in range(lo, hi):
            ax = self._path[i].pose.position.x
            ay = self._path[i].pose.position.y
            bx = self._path[i + 1].pose.position.x
            by = self._path[i + 1].pose.position.y
            dx = bx - ax
            dy = by - ay
            seg_sq = dx * dx + dy * dy
            if seg_sq < 1e-12:
                continue

            t_raw = ((pos_n - ax) * dx + (pos_e - ay) * dy) / seg_sq
            t = self._clamp(t_raw, 0.0, 1.0)
            foot_n = ax + t * dx
            foot_e = ay + t * dy
            d = self._dist(pos_n, pos_e, foot_n, foot_e)

            if d < best[4]:
                # Signed cross-track via 2D cross product
                # path direction (dx, dy); error vector (pos − foot)
                # cross_z = dx * (pos_e − foot_e) − dy * (pos_n − foot_n)
                # In NED top-down view, +cross_z = rover is to the right of
                # the path heading.
                cross_z = dx * (pos_e - foot_e) - dy * (pos_n - foot_n)
                seg_len = math.sqrt(seg_sq)
                signed_e = math.copysign(d, cross_z) if seg_len > 0 else 0.0
                best = (i, t, foot_n, foot_e, d, signed_e)

        # P1.4: persist the winning segment for next cycle. If the windowed
        # search found nothing useful (every segment in the window was
        # zero-length), invalidate the hint so the next cycle full-scans
        # to recover.
        if best[4] == float("inf"):
            self._hint_valid = False
        else:
            self._closest_seg_hint = best[0]
            self._hint_valid = True
        return best[0], best[1], best[2], best[3], best[5]

    def _get_lookahead_point(
        self, seg_idx: int, foot_n: float, foot_e: float, l_d: float
    ) -> tuple[float, float, bool]:
        """Walk along the path from (foot_n, foot_e) on segment seg_idx until
        accumulated arc length ≥ l_d.

        Returns (lh_n, lh_e, hit_end). hit_end is True when the path is
        shorter than l_d from the foot.
        """
        n_pts = len(self._path)
        # First sub-segment: from foot to end of seg_idx
        end_n = self._path[seg_idx + 1].pose.position.x if seg_idx + 1 < n_pts \
            else self._path[seg_idx].pose.position.x
        end_e = self._path[seg_idx + 1].pose.position.y if seg_idx + 1 < n_pts \
            else self._path[seg_idx].pose.position.y

        prev_n, prev_e = foot_n, foot_e
        next_n, next_e = end_n, end_e
        arc = 0.0

        # Iterate from current segment to end of path
        i = seg_idx + 1
        while True:
            seg_len = self._dist(prev_n, prev_e, next_n, next_e)
            if arc + seg_len >= l_d:
                # Interpolate exactly l_d into this sub-segment
                remaining = l_d - arc
                ratio = remaining / seg_len if seg_len > 1e-9 else 1.0
                lh_n = prev_n + ratio * (next_n - prev_n)
                lh_e = prev_e + ratio * (next_e - prev_e)
                return lh_n, lh_e, False
            arc += seg_len
            i += 1
            if i >= n_pts:
                # Off the end of the path — return final waypoint
                final = self._path[-1].pose.position
                return final.x, final.y, True
            prev_n, prev_e = next_n, next_e
            next_n = self._path[i].pose.position.x
            next_e = self._path[i].pose.position.y

    def _control_segment_profile(
        self,
        pos_n: float,
        pos_e: float,
        yaw_ned: float,
        pose_age_s: float,
        dist_to_goal: float,
    ) -> None:
        """Track straight/polyline/polygon paths one segment at a time."""
        n_pts = len(self._path)
        if n_pts < 2:
            # Single-point run (e.g. a POINT entity): advance immediately;
            # the next control cycle tracks the new run.
            if self._advance_run():
                return
            self._segment_state = SegmentStateCode.DONE
            self._path_done = True
            self._publish_zero(
                StateCode.DONE,
                pose_age_ms=pose_age_s * 1000.0,
                dist_to_goal=dist_to_goal,
            )
            self._publish_segment_debug(
                self._segment_state, 0, float("nan"), dist_to_goal,
                float("nan"), float("nan"), float("nan"), 0.0,
            )
            return

        while self._segment_idx < n_pts - 2:
            a = self._path[self._segment_idx].pose.position
            b = self._path[self._segment_idx + 1].pose.position
            if self._dist(a.x, a.y, b.x, b.y) >= 1e-6:
                break
            self._segment_idx += 1

        self._segment_idx = max(0, min(self._segment_idx, n_pts - 2))
        seg_idx = self._segment_idx
        a = self._path[seg_idx].pose.position
        b = self._path[seg_idx + 1].pose.position
        seg_len = self._dist(a.x, a.y, b.x, b.y)
        if seg_len < 1e-6:
            self._publish_zero(
                StateCode.IDLE,
                pose_age_ms=pose_age_s * 1000.0,
                dist_to_goal=dist_to_goal,
            )
            return

        final_segment = seg_idx >= n_pts - 2
        t, foot_n, foot_e, signed_xtrack, dist_to_end_along = self._project_onto_segment(
            pos_n, pos_e, seg_idx
        )
        self._update_path_progress(seg_idx, t)
        dist_to_corner = self._dist(pos_n, pos_e, b.x, b.y)
        corner_angle = self._segment_angle_deg(seg_idx)
        spray_active = self._segment_spray_active(seg_idx)

        goal_tol = float(self.get_parameter("xy_goal_tolerance").value)
        min_travel = self._run_min_travel()
        if final_segment and dist_to_corner <= goal_tol and self._path_travel_m >= min_travel:
            # Stop before switching across a real heading change. Collinear
            # spray transitions were already merged during path conditioning.
            if self._run_idx + 1 < len(self._runs):
                self._hold_before_run_advance(
                    pos_n, pos_e, yaw_ned, pose_age_s, dist_to_corner
                )
                return
            # D3: route the final-run completion through the single stop handler
            # so the rover brakes to a confirmed stop instead of coasting on a
            # bare zero setpoint.
            self._hold_at_completion(
                pos_n, pos_e, yaw_ned, pose_age_s, dist_to_corner
            )
            return

        acceptance = float(self.get_parameter("segment_corner_acceptance_radius").value)
        heading_tol = math.radians(
            float(self.get_parameter("segment_heading_tolerance_deg").value)
        )
        yaw_gain = float(self.get_parameter("segment_yaw_rate_gain").value)
        use_ff_yaw_rate = bool(self.get_parameter("use_feedforward_yaw_rate").value)
        max_yr = float(self.get_parameter("max_yaw_rate_body").value)

        if not final_segment and dist_to_corner <= acceptance:
            path_corner_deg = abs(self._segment_angle_deg(seg_idx))
            threshold_deg = float(self.get_parameter("segment_corner_threshold_deg").value)
            c = self._path[seg_idx + 2].pose.position
            target_heading = math.atan2(c.y - b.y, c.x - b.x)
            heading_err = self._angle_wrap(target_heading - yaw_ned)
            timed_out = self._corner_stop_complete and self._pivot_timed_out(
                math.radians(path_corner_deg)
            )
            yaw_rate_tol = float(self.get_parameter("segment_stop_yaw_rate_threshold").value)
            align_settle_s = float(self.get_parameter("segment_align_settle_s").value)
            now_align = self.get_clock().now()
            release_heading_tol = heading_tol
            if timed_out:
                release_heading_tol = max(
                    heading_tol,
                    math.radians(float(self.get_parameter(
                        "segment_timeout_heading_tolerance_deg"
                    ).value)),
                )
            release_heading_tol = min(
                release_heading_tol,
                math.radians(float(self.get_parameter("segment_pivot_release_max_deg").value)),
            )
            heading_ok = abs(heading_err) <= release_heading_tol
            vel_fresh = self._vel_is_fresh()
            yaw_rate_ok = (
                abs(self._latest_yaw_rate_ned) < yaw_rate_tol
                if vel_fresh else timed_out
            )
            speed_ok = self._align_speed_ok() if vel_fresh else timed_out
            if self._corner_stop_complete and heading_ok and yaw_rate_ok and speed_ok:
                if self._align_settle_since is None:
                    self._align_settle_since = now_align
                settled = (now_align - self._align_settle_since).nanoseconds * 1e-9 >= align_settle_s
            else:
                self._align_settle_since = None
                settled = False
            # Advance immediately for geometrically tangent junctions; otherwise
            # require the corner-stop settle/timeout gate for hard corners.
            if path_corner_deg < threshold_deg or settled:
                self._segment_idx += 1
                self._segment_state = SegmentStateCode.TRACK_SEGMENT
                # Collinear (sub-threshold) junctions keep momentum so the rover
                # flows through a spray-only PRE/MARK/AFT boundary instead of
                # dipping to ~0 and ramping back up (field: 0.08-0.11 m/s dips at
                # PRE->MARK / MARK->AFT). Only a real corner (reached here via the
                # stop/align settle gate) zeroes speed for the pivot.
                if path_corner_deg >= threshold_deg:
                    self._last_speed_cmd = 0.0
                self._reset_corner_pivot_state()
                self._publish_segment_debug(
                    self._segment_state, self._segment_idx, float("nan"),
                    dist_to_corner, corner_angle, target_heading, heading_err, 0.0,
                )
                self._control_segment_profile(
                    pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal
                )
                return

            # Once heading is inside the release band, stop driving the pivot
            # vector and actively settle before advancing. Otherwise the
            # corner-speed command itself prevents the speed gate from passing.
            if self._corner_stop_complete and heading_ok:
                self._last_speed_cmd = 0.0
                brake_n, brake_e = self._corner_brake_velocity(yaw_ned)
                self._publish_velocity(brake_n, brake_e)
                self._publish_yaw_rate(0.0)
                self._publish_debug(
                    cross_track=signed_xtrack,
                    heading_err=heading_err,
                    lookahead=dist_to_corner,
                    speed=math.hypot(brake_n, brake_e),
                    kappa=0.0,
                    dist_goal=dist_to_goal,
                    pose_age_ms=pose_age_s * 1000.0,
                    state=StateCode.TRACKING,
                    l_d_raw=float("nan"),
                    kappa_speed=0.0,
                    yaw_rate=0.0,
                    spray_active=spray_active,
                )
                self._publish_segment_debug(
                    SegmentStateCode.CORNER_ALIGN, seg_idx,
                    dist_to_end_along, dist_to_corner, corner_angle,
                    target_heading, heading_err, 0.0,
                )
                return

            # Stop-and-spin: confirm the rover is physically stopped at the
            # corner before pivoting (see _run_alignment_hold for the twin).
            if not self._corner_stop_complete:
                if not self._corner_stop_satisfied():
                    self._segment_state = SegmentStateCode.CORNER_STOP
                    # Active braking (see _run_alignment_hold twin): drive a
                    # small velocity opposing the rover's motion so it physically
                    # stops at the corner instead of coasting past it.
                    self._last_speed_cmd = 0.0
                    brake_n, brake_e = self._corner_brake_velocity(yaw_ned)
                    self._publish_velocity(brake_n, brake_e)
                    self._publish_yaw_rate(0.0)
                    self._publish_debug(
                        cross_track=signed_xtrack,
                        heading_err=heading_err,
                        lookahead=dist_to_corner,
                        speed=math.hypot(brake_n, brake_e),
                        kappa=0.0,
                        dist_goal=dist_to_goal,
                        pose_age_ms=pose_age_s * 1000.0,
                        state=StateCode.TRACKING,
                        l_d_raw=float("nan"),
                        kappa_speed=0.0,
                        yaw_rate=0.0,
                        spray_active=spray_active,
                    )
                    self._publish_segment_debug(
                        self._segment_state, seg_idx, dist_to_end_along,
                        dist_to_corner, corner_angle, target_heading,
                        heading_err, 0.0,
                    )
                    return
                self._corner_stop_complete = True

            self._segment_state = SegmentStateCode.CORNER_ALIGN
            # Corner pivot — firmware-aware actuation.
            #
            # PX4 rover_differential (DifferentialVelControl) in OFFBOARD
            # velocity mode derives heading SOLELY from the velocity-vector
            # bearing = atan2(vE, vN); it ignores the MAVROS yaw and yaw_rate
            # fields entirely. When |v| < 0.01 m/s it holds the current
            # heading. So publishing (0 vel, yaw_rate) — the old behaviour —
            # made the firmware freeze heading and discard the yaw_rate, which
            # deadlocked the mission at the first corner (square bag
            # 20260611_170539: 0.7° of yaw change at the corner, never
            # advanced to side 2).
            #
            # Instead, command a small velocity VECTOR aimed at the exit
            # heading (but kept inside the forward cone — see
            # _corner_pivot_velocity). The firmware's native state machine
            # sees a large heading error (>RD_TRANS_DRV_TRN, 10°), enters
            # SPOT_TURNING (zero forward throttle), and rotates in place the
            # short way; once aligned (<RD_TRANS_TRN_DRV, 5°) it transitions
            # to DRIVING and rolls out along side 2. Magnitude only sets the
            # post-turn drive-out speed and must clear the firmware's 0.01 m/s
            # freeze threshold with margin. yaw_rate is left zero — it is
            # inert on this rover and only muddies the setpoint mask.
            corner_speed = max(
                0.05, float(self.get_parameter("segment_min_corner_speed").value)
            )
            v_n, v_e = self._corner_pivot_velocity(yaw_ned, heading_err, corner_speed)
            self._last_speed_cmd = corner_speed
            self._publish_velocity(v_n, v_e)
            self._publish_yaw_rate(0.0)
            self._publish_debug(
                cross_track=signed_xtrack,
                heading_err=heading_err,
                lookahead=dist_to_corner,
                speed=corner_speed,
                kappa=0.0,
                dist_goal=dist_to_goal,
                pose_age_ms=pose_age_s * 1000.0,
                state=StateCode.TRACKING,
                l_d_raw=float("nan"),
                kappa_speed=0.0,
                yaw_rate=0.0,
                spray_active=spray_active,
            )
            self._publish_segment_debug(
                self._segment_state, seg_idx, dist_to_end_along, dist_to_corner,
                corner_angle, target_heading, heading_err, 0.0,
            )
            return

        hw_max_v = float(self.get_parameter("max_linear_vel").value)
        mission_v = float(self.get_parameter("mission_speed").value)
        max_v = min(hw_max_v, mission_v)
        min_v = float(self.get_parameter("min_linear_vel").value)
        l_min = float(self.get_parameter("min_lookahead_dist").value)
        l_max = float(self.get_parameter("max_lookahead_dist").value)
        ld_gain = float(self.get_parameter("lookahead_time").value)
        xt_ld_gain = float(self.get_parameter("xtrack_lookahead_gain").value)

        v_for_ld = max(min_v, self._last_speed_cmd if self._last_speed_cmd > 0.0
                       else max_v * 0.5)
        v_for_ld = 0.7 * v_for_ld + 0.3 * max_v
        l_d_raw = ld_gain * v_for_ld + xt_ld_gain * abs(signed_xtrack)
        l_d = self._clamp(l_d_raw, l_min, l_max)

        lh_n, lh_e = self._segment_lookahead_point(
            seg_idx, foot_n, foot_e, l_d,
            float(self.get_parameter("segment_lookahead_cross_collinear_deg").value),
        )

        dn = lh_n - pos_n
        de = lh_e - pos_e
        x_body = dn * math.cos(yaw_ned) + de * math.sin(yaw_ned)
        y_body = -dn * math.sin(yaw_ned) + de * math.cos(yaw_ned)
        l_actual = math.hypot(x_body, y_body)
        if l_actual < 1e-6:
            dn = b.x - pos_n
            de = b.y - pos_e
            x_body = dn * math.cos(yaw_ned) + de * math.sin(yaw_ned)
            y_body = -dn * math.sin(yaw_ned) + de * math.cos(yaw_ned)
            l_actual = math.hypot(x_body, y_body)
            if l_actual < 1e-6:
                self._publish_zero(
                    StateCode.IDLE,
                    pose_age_ms=pose_age_s * 1000.0,
                    dist_to_goal=dist_to_goal,
                )
                return

        theta_e = math.atan2(y_body, x_body)
        speed = max_v
        slowdown = float(self.get_parameter("segment_slowdown_dist").value)
        min_corner_speed = float(self.get_parameter("segment_min_corner_speed").value)
        self._segment_state = SegmentStateCode.TRACK_SEGMENT
        corner_threshold_deg = float(
            self.get_parameter("segment_corner_threshold_deg").value
        )
        if (
            not final_segment
            and slowdown > 1e-6
            and dist_to_corner < slowdown
            and math.isfinite(corner_angle)
            and abs(corner_angle) >= corner_threshold_deg
        ):
            scale = self._clamp(dist_to_corner / slowdown, 0.0, 1.0)
            speed = max(min_corner_speed, max_v * scale)
            self._segment_state = SegmentStateCode.PRE_CORNER_SLOWDOWN

        # Final-segment goal-approach deceleration.
        # The corner slowdown above is gated to NON-final segments, so a
        # straight line used to drive at full speed into its endpoint B and
        # only zero velocity once within xy_goal_tolerance — arriving fast, it
        # coasted PAST B. Mirror the smooth/arc profile's approach scaling
        # (see _control) so the rover brakes to a low speed before B and stops
        # ON the point. This also tightens run-to-run corner overshoot, since
        # a run endpoint is a final segment too.
        if final_segment:
            max_decel = float(self.get_parameter("max_linear_decel").value)
            # Run-endpoint floor (see param decl): lower than the smooth/arc
            # min_approach_linear_velocity so the rover arrives slow enough for
            # active braking to stop ON the corner point, not 4 cm past it.
            approach_v = float(self.get_parameter("segment_endpoint_approach_speed").value)
            approach_d = max(
                float(self.get_parameter("approach_velocity_scaling_dist").value),
                (max_v * max_v) / (2.0 * max_decel) + 0.10,
            )
            if dist_to_corner < approach_d:
                scale = self._clamp(dist_to_corner / approach_d, 0.0, 1.0)
                speed = min(speed, max(approach_v, max_v * scale))
                self._segment_state = SegmentStateCode.PRE_CORNER_SLOWDOWN

        max_accel = float(self.get_parameter("max_linear_accel").value)
        speed_before_accel = speed
        if max_accel > 0.0:
            speed = min(speed, self._last_speed_cmd + max_accel * self._tick_dt)

        p4_floor = float(self.get_parameter("p4_zero_vel_threshold").value)
        if speed < p4_floor and speed_before_accel < p4_floor and self._last_speed_cmd > 0.0:
            speed = 0.0
        self._last_speed_cmd = speed

        yaw_rate_body = yaw_gain * theta_e if use_ff_yaw_rate else 0.0
        if max_yr > 0.0:
            yaw_rate_body = self._clamp(yaw_rate_body, -max_yr, max_yr)

        unit_n = dn / l_actual
        unit_e = de / l_actual
        v_n = speed * unit_n
        v_e = speed * unit_e
        # BUG-T3 fix: clamp velocity bearing into forward cone so PX4
        # reverse-detection never flips the turn, even on the first segment
        # (idx==0) where _run_alignment_hold is skipped.
        v_n, v_e = self._clamp_velocity_to_forward_cone(v_n, v_e, yaw_ned, speed)
        speed_mag = math.hypot(v_n, v_e)
        if speed_mag > 0.01:
            self._last_yaw_cmd = math.atan2(v_e, v_n)

        self._publish_velocity(v_n, v_e)
        self._publish_yaw_rate(yaw_rate_body)
        self._publish_debug(
            cross_track=signed_xtrack,
            heading_err=theta_e,
            lookahead=l_actual,
            speed=speed,
            kappa=0.0,
            dist_goal=dist_to_goal,
            pose_age_ms=pose_age_s * 1000.0,
            state=StateCode.TRACKING,
            l_d_raw=l_d_raw,
            kappa_speed=0.0,
            yaw_rate=yaw_rate_body,
            spray_active=spray_active,
        )
        self._publish_segment_debug(
            self._segment_state, seg_idx, dist_to_end_along, dist_to_corner,
            corner_angle, math.atan2(b.y - a.y, b.x - a.x), theta_e,
            yaw_rate_body,
        )

        self.get_logger().debug(
            f"[SEGMENT/{self._segment_state.name}] seg={seg_idx} "
            f"xtrack={signed_xtrack * 100:+.2f}cm dcorner={dist_to_corner:.2f}m "
            f"ld={l_actual:.2f}m v=({v_n:+.3f},{v_e:+.3f})m/s "
            f"speed={speed:.3f} θe={math.degrees(theta_e):+.1f}° "
            f"yaw_rate={yaw_rate_body:+.3f}rad/s"
        )

    # ==================================================================
    # Main control loop (50 Hz)
    # ==================================================================
    def _control_loop(self):
        """Timer entry: run the frozen control tick, then (gated) publish
        mission progress.

        The control tick is byte-for-byte `_control_loop_impl`. Progress
        publication is fully behind `progress_publish_enabled` AND wrapped so a
        bug in the observability path can never take down the controller — if
        it raises, the drive command for this tick has already been published
        by the impl, and we only lose one progress sample.
        """
        # Atomic mission handoff: install any mission staged by _path_cb
        # BEFORE dt is measured, so the R2 "_last_tick = None" reset inside
        # _install_mission gives this very tick the nominal 1/CONTROL_HZ dt.
        # No-op (one identity compare) on every tick without a fresh mission,
        # and today also on ticks WITH one — _path_cb drains inline under the
        # serial executor. This call is the future MT-executor install point.
        self._drain_pending_mission()

        # R2 — measured dt once per tick (ROS clock, not wall time) so segment
        # and smooth accel ramps agree. Clamp at 0.1 s: without it a long stall
        # (blocked timer, debugger, load spike) produces one huge accel step.
        # Floor at 0.0: a backward ROS-clock step would otherwise give negative
        # dt, and `max_accel * dt` then LOWERS the speed ceiling below
        # _last_speed_cmd — one spurious decel tick. dt=0 just holds the
        # previous ceiling (dt is only ever a multiplier in the two ramps).
        now = self.get_clock().now()
        if self._last_tick is None:
            self._tick_dt = 1.0 / self.CONTROL_HZ
        else:
            self._tick_dt = max(0.0, min(0.1, (now - self._last_tick).nanoseconds * 1e-9))
        self._last_tick = now

        self._control_loop_impl()
        if bool(self.get_parameter("progress_publish_enabled").value):
            try:
                self._publish_progress_tick()
            except Exception as exc:  # never let observability crash control
                self.get_logger().warn(
                    f"progress publish failed: {exc}", throttle_duration_sec=2.0
                )

    # ------------------------------------------------------------------
    # G1 — mission-progress publication (design §3/§4). Gated, additive.
    # ------------------------------------------------------------------
    def _measured_speed(self) -> float:
        """Latest measured ground speed (m/s), 0 if no fresh velocity sample."""
        if self._latest_vel_time is None:
            return 0.0
        age = (self.get_clock().now() - self._latest_vel_time).nanoseconds * 1e-9
        if age >= 0.5:
            return 0.0
        v_n, v_e = self._latest_vel_ned
        return math.hypot(v_n, v_e)

    def _point_progress_inputs(self) -> dict:
        """Point-mode phase inputs for the classifier, from the point-hold state.

        Reuses the SAME point-hold state the Phase-D overlay maintains
        (_point_hold_active_key/_start_ns/_done_keys) — G1 only *reports* it.
        """
        point_mode = (
            bool(self.get_parameter("point_hold_enabled").value)
            and bool(self._must_hit_keys)
        )
        if not point_mode:
            return {"point_mode": False}
        active_key = self._point_hold_active_key
        done = self._point_hold_done_keys
        all_done = len(done) >= len(self._must_hit_keys)
        if active_key is not None:
            rank = self._musthit_rank_by_key.get(active_key, -1)
        else:
            # closing on the next un-dwelled must-hit point, in path order
            rank = -1
            for r, vtx in enumerate(self._musthit_vertices):
                p = self._path[vtx].pose.position
                if self._pt_key((p.x, p.y)) not in done:
                    rank = r
                    break
        vertex = self._musthit_vertices[rank] if 0 <= rank < len(self._musthit_vertices) else -1
        return {
            "point_mode": True,
            "point_active": active_key is not None,
            "point_dwelling": self._point_hold_start_ns is not None,
            # G5 — manual gate: the RPP is holding after dwell for the operator's
            # /point/advance. Reported so /rpp/progress (→ /spray/status → the app)
            # can surface WAIT_OPERATOR for the "Next point" button. None unless a
            # manual handshake is actually waiting.
            "point_wait_operator": self._point_wait_start_ns is not None,
            "point_target_rank": rank,
            "point_target_vertex": vertex,
            "all_points_done": all_done,
        }

    def _publish_progress_tick(self) -> None:
        """Classify this tick and publish /rpp/progress (+ milestone edges)."""
        if not self._path or self._last_pos is None:
            self._emit_progress(ProgressMsg(
                phase=MissionPhase.REACHED_END if self._path_done else MissionPhase.IDLE
            ))
            return

        # Tracking pose = raw last pose minus any absorbed EKF-reset offset —
        # exactly what the control tick projected with this cycle.
        pos_n = self._last_pos[0] - self._ekf_reset_offset[0]
        pos_e = self._last_pos[1] - self._ekf_reset_offset[1]
        seg_idx = max(0, min(self._segment_idx, max(0, len(self._path) - 2)))
        t, _fn, _fe, signed_xtrack, _de = self._project_onto_segment(pos_n, pos_e, seg_idx)
        along_s = self._path_progress_at(seg_idx, t)
        speed = self._measured_speed()
        stop_thr = float(self.get_parameter("segment_stop_speed_threshold").value)

        pt = self._point_progress_inputs()
        msg = pc.classify(
            has_path=True, path_done=self._path_done,
            spray_flags=self._spray_flags, cum_s=self._path_s,
            seg_idx=seg_idx, proj_t=t, along_s=along_s,
            signed_xtrack=signed_xtrack, speed=speed, stopped=speed < stop_thr,
            approach_dist_m=float(self.get_parameter("progress_approach_dist_m").value),
            **pt,
        )
        self._emit_progress(msg)

    def _emit_progress(self, msg: ProgressMsg) -> None:
        """Publish a ProgressMsg + any milestone events on the phase/point edge."""
        s = String()
        s.data = msg.to_json()
        self._progress_pub.publish(s)

        events = pc.milestones_for(
            self._progress_last_phase, msg.phase,
            self._progress_last_point, msg.point_index,
        )
        for event in events:
            self._progress_seq += 1
            idx = msg.point_index if msg.point_index >= 0 else msg.segment_index
            m = String()
            m.data = MilestoneMsg(
                event=event, seq=self._progress_seq, index=idx,
                stamp_ns=self.get_clock().now().nanoseconds,
            ).to_json()
            self._milestone_pub.publish(m)
        self._progress_last_phase = msg.phase
        self._progress_last_point = msg.point_index

    def _control_loop_impl(self):
        """Compute and publish NED velocity vector."""
        # ---- Read parameters (allows runtime tuning) ----
        hw_max_v    = self.get_parameter("max_linear_vel").value           # hardware ceiling
        mission_v   = self.get_parameter("mission_speed").value           # P4.2 operator knob
        max_v       = min(hw_max_v, mission_v)                            # effective ceiling
        min_v       = self.get_parameter("min_linear_vel").value
        l_min       = self.get_parameter("min_lookahead_dist").value
        l_max       = self.get_parameter("max_lookahead_dist").value
        ld_gain     = self.get_parameter("lookahead_time").value
        a_lat_max   = self.get_parameter("a_lat_max").value               # P4.1
        min_curv_v  = self.get_parameter("regulated_linear_scaling_min_speed").value
        goal_tol    = self.get_parameter("xy_goal_tolerance").value
        approach_v  = self.get_parameter("min_approach_linear_velocity").value
        p4_floor    = self.get_parameter("p4_zero_vel_threshold").value
        max_age_s   = self.get_parameter("pose_max_age_s").value
        req_rtk     = self.get_parameter("require_rtk_fix").value         # P0.3
        n_preview   = int(self.get_parameter("preview_curvature_n").value)  # P1.1
        xt_ld_gain  = self.get_parameter("xtrack_lookahead_gain").value   # P1.2

        # P4.2 — Derive speed-dependent params from mission_speed at runtime.
        # Operator only sets mission_speed; these follow automatically.

        # Braking distance: d = v² / (2·a_decel) + 0.10m safety margin.
        # max(param, derived) so the configured value acts as a minimum floor.
        max_decel   = self.get_parameter("max_linear_decel").value
        approach_d  = max(
            self.get_parameter("approach_velocity_scaling_dist").value,
            (max_v * max_v) / (2.0 * max_decel) + 0.10,
        )

        # EKF jump threshold: per-cycle physical max = mission_speed / Hz + 3σ_RTK.
        # max(param, derived) keeps the manual param as a hard floor.
        jump_thr    = max(                                                # P0.2 + P4.2
            self.get_parameter("ekf_jump_threshold_m").value,
            max_v / self.CONTROL_HZ + 0.03,
        )

        # ---- Pose freshness check ----
        # P2.4 fixup: when use_imu_extrapolation is on, allow `pose_age` up to
        # `max_age_s + imu_max_extrap_age_s` and dead-reckon the residual
        # using the latest velocity. Without this expansion the extrapolation
        # benefit was theoretical — a stale pose still tripped STALE before
        # we got a chance to extrapolate it.
        if self._pose is None or self._pose_recv_time is None:
            self._publish_zero(StateCode.IDLE, pose_age_ms=float("nan"))
            return

        use_extrap = self.get_parameter("use_imu_extrapolation").value
        extrap_horizon = float(self.get_parameter("imu_max_extrap_age_s").value)
        effective_max_age = max_age_s + (extrap_horizon if use_extrap else 0.0)

        pose_age_s = (self.get_clock().now() - self._pose_recv_time).nanoseconds * 1e-9
        if pose_age_s > effective_max_age:
            self.get_logger().warn(
                f"Stale pose ({pose_age_s * 1000:.0f} ms > "
                f"{effective_max_age * 1000:.0f} ms) — emergency stop",
                throttle_duration_sec=1.0,
            )
            self._publish_zero(StateCode.STALE, pose_age_ms=pose_age_s * 1000)
            return

        # P2.4 — Velocity-based pose extrapolation (latency closure)
        # If enabled and we have a fresh velocity sample, project the pose
        # forward by the full pose_age using `pos + v·dt`. This is gravity-
        # clean (PX4-EKF compensated) and the dominant correction term —
        # at v=0.4 m/s and dt=50 ms that's 2 cm of latency closure.
        # We deliberately skip the 0.5·a·dt² term: at typical bench accel
        # 0.5 m/s² it contributes <1 mm and pulling a in introduces gravity
        # bias from imperfect roll/pitch attitude.
        pose_for_projection = self._pose
        if use_extrap and self._latest_vel_time is not None:
            vel_age_s = (self.get_clock().now() - self._latest_vel_time).nanoseconds * 1e-9
            # Only trust velocity if it's at least as fresh as the pose
            # (otherwise we'd be applying a stale velocity to a stale pose).
            if vel_age_s < extrap_horizon:
                v_n, v_e = self._latest_vel_ned
                dt = pose_age_s
                d_n = v_n * dt
                d_e = v_e * dt

                pose_for_projection = PoseStamped()
                pose_for_projection.header = self._pose.header
                # self._pose is MAVROS ENU: x=East, y=North.
                # _latest_vel_ned is NED: v_n=North, v_e=East.
                pose_for_projection.pose.position.x = self._pose.pose.position.x + d_e
                pose_for_projection.pose.position.y = self._pose.pose.position.y + d_n
                pose_for_projection.pose.position.z = self._pose.pose.position.z
                pose_for_projection.pose.orientation = self._pose.pose.orientation

                self.get_logger().debug(
                    f"P2.4 v-extrapolation: pose_age={pose_age_s*1000:.1f}ms, "
                    f"v_ned=({v_n:+.2f},{v_e:+.2f}) m/s, "
                    f"Δned=({d_n*100:+.2f},{d_e*100:+.2f}) cm"
                )

        # ---- P0.3: RTK FIX gate ----
        if req_rtk and self._gps_fix_type < 6:
            self.get_logger().warn(
                f"GPS fix_type={self._gps_fix_type} (need 6=RTK_FIXED) — "
                "refusing to drive. Set require_rtk_fix:=false for SITL.",
                throttle_duration_sec=2.0,
            )
            # B2: emit RTK_WAIT (4) so observers can distinguish "no GPS fix"
            # from "no pose stream" (which stays as STALE/-1).
            self._publish_zero(StateCode.RTK_WAIT, pose_age_ms=pose_age_s * 1000)
            return

        # ---- Path readiness check ----
        if not self._path:
            self._publish_zero(StateCode.IDLE, pose_age_ms=pose_age_s * 1000)
            return

        # ---- Already done? Keep heartbeat at zero. ----
        if self._path_done:
            self._publish_zero(StateCode.DONE, pose_age_ms=pose_age_s * 1000)
            return

        # ---- Pose in NED ----
        # Single quaternion extraction: yaw is captured here and reused at
        # the body-frame κ computation below. Earlier versions called
        # _enu_pose_to_ned twice per cycle; that's now consolidated.
        pos_n, pos_e, yaw_ned = self._enu_pose_to_ned(pose_for_projection)

        # ---- P0.2 / A3: EKF / position-jump detection ----
        # If the pose jumps further than is physically possible in one control
        # cycle (max_v * dt + 3σ_pos), it's an EKF reset or RTK acquisition
        # artefact — the rover did not physically move, only its estimate did.
        #
        # Two responses, selected by `ekf_reset_compensation`:
        #   • OFF (frozen P0.2 baseline): skip this cycle, do NOT update the
        #     controller. _last_pos still advances to the post-jump position so
        #     only one cycle is skipped per event — but the path reference does
        #     NOT move, so subsequent cycles see the reset as cross-track error
        #     and steer to close it (A3: paints a kink at the reset).
        #   • ON (A3): absorb the jump vector into `_ekf_reset_offset` and drive
        #     through. The offset is subtracted from the tracking pose below, so
        #     the path relationship stays continuous and net cross-track ≈ 0 —
        #     PX4's own setpoint-shift convention, done companion-side because
        #     MAVROS does not expose delta_xy/xy_reset_counter here.
        comp_enabled = self.get_parameter("ekf_reset_compensation").value
        max_absorb = float(self.get_parameter("ekf_reset_max_absorb_m").value)
        if self._last_pos is not None:
            d_n = pos_n - self._last_pos[0]
            d_e = pos_e - self._last_pos[1]
            jump_m = math.hypot(d_n, d_e)
            if jump_m > jump_thr:
                # A3 compensation: only for jumps within the absorb cap. A
                # larger jump is treated as a suspect estimate, not a clean
                # reset, and falls through to the baseline skip below.
                if comp_enabled and jump_m <= max_absorb:
                    self._ekf_reset_offset = (
                        self._ekf_reset_offset[0] + d_n,
                        self._ekf_reset_offset[1] + d_e,
                    )
                    self._ekf_reset_count += 1
                    # A3 post-hoc flag: this WARN is the only in-log marker that
                    # a reset landed here, so a later kink/offset in the painted
                    # line can be correlated to it. delta_xy is unavailable over
                    # MAVROS, so the absorbed jump vector is logged in its place.
                    self.get_logger().warn(
                        f"A3 EKF reset absorbed: jump {jump_m * 100:.1f} cm "
                        f"(Δ={d_n * 100:+.1f},{d_e * 100:+.1f} cm) → cumulative "
                        f"offset ({self._ekf_reset_offset[0] * 100:+.1f},"
                        f"{self._ekf_reset_offset[1] * 100:+.1f}) cm "
                        f"[n={self._ekf_reset_count}] — tracking frame held",
                        throttle_duration_sec=0.5,
                    )
                    # Frame is continuous (offset applied below), so the segment
                    # hint stays valid and we drive on — no skip, no kink.
                    # _last_pos advances at the shared assignment just below.
                else:
                    self.get_logger().warn(
                        f"Position jump {jump_m * 100:.1f} cm > threshold "
                        f"{jump_thr * 100:.1f} cm — skipping cycle (EKF reset?)",
                        throttle_duration_sec=0.5,
                    )
                    self._last_pos = (pos_n, pos_e)
                    # Reset segment hint: after a jump we can't trust the old index
                    self._closest_seg_hint = 0
                    # P1.4 fixup — force full scan next cycle so we relocate the
                    # rover's true segment instead of crawling a window forward
                    # from a stale hint.
                    self._hint_valid = False
                    # B2: emit JUMP_SKIP (5) so observers see the cause-of-pause.
                    # Server watchdog and offboard controller treat it the same
                    # as STALE (RPP_UNHEALTHY_CODES) — same response, more info.
                    self._publish_zero(StateCode.JUMP_SKIP, pose_age_ms=pose_age_s * 1000)
                    return
        self._last_pos = (pos_n, pos_e)

        # ---- A3: apply the accumulated EKF-reset offset to the tracking pose ----
        # No-op when compensation is off (offset stays (0,0)). When on, every
        # absorbed reset shifts the pose we track with, so the path reference
        # effectively moves with the estimate and the painted line stays smooth.
        # _last_pos above intentionally holds the RAW pose so jump detection
        # keeps working in the estimator's own frame.
        pos_n -= self._ekf_reset_offset[0]
        pos_e -= self._ekf_reset_offset[1]

        # ---- Run-transition alignment (per-entity profile switching) ----
        # After advancing to a new run, pivot toward its initial heading
        # before tracking it. No-op when already aligned.
        if self._run_alignment_hold(pos_n, pos_e, yaw_ned, pose_age_s):
            return

        smooth_projection = None
        if self._active_tracking_profile != "segment":
            smooth_projection = self._project_onto_path(pos_n, pos_e)
            self._update_path_progress(smooth_projection[0], smooth_projection[1])

        # ---- Goal check ----
        # Skip until the rover has traveled min_goal_travel_m along the path.
        # Prevents immediate DONE on closed-loop paths where the rover starts
        # at the final waypoint (e.g., square_2x2 with auto_origin).
        min_travel = self._run_min_travel()
        final = self._path[-1].pose.position
        dist_to_goal = self._dist(pos_n, pos_e, final.x, final.y)
        # Phase D point-hold overlay (default OFF — frozen until enabled). Dwell
        # at each undone must-hit waypoint before the run-boundary/goal logic so
        # every surveyed point (incl. corners and the final point) gets its mark
        # dwell. Byte-for-byte no-op when point_hold_enabled is False.
        if self._point_hold_tick(pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal):
            return
        if self._run_boundary_stop_pending:
            self._hold_before_run_advance(
                pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal
            )
            return
        # D3: once the final-waypoint stop is latched, hold it BEFORE the goal
        # test — a coast back above xy_goal_tolerance must not flip control into
        # tracking and drive away.
        if self._completion_stop_pending:
            self._hold_at_completion(
                pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal
            )
            return
        if dist_to_goal <= goal_tol and self._path_travel_m >= min_travel:
            # End of the active run: advance to the next run (per-entity
            # profile switching). The next 20 ms cycle pivots via
            # _run_alignment_hold if needed, then tracks the new run. DONE
            # is only published after the last run — the server's mission-
            # complete settling watches for it.
            if self._run_idx + 1 < len(self._runs):
                self._hold_before_run_advance(
                    pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal
                )
                return
            # D3: final waypoint reached — brake to a confirmed physical stop
            # before DONE instead of publishing a bare zero that PX4 coasts on.
            self._hold_at_completion(
                pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal
            )
            return

        if self._active_tracking_profile == "segment":
            self._control_segment_profile(
                pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal
            )
            return

        # ---- Step 1: Closest-point projection (segment, not vertex) ----
        # P1.4: _project_onto_path uses _closest_seg_hint internally.
        if smooth_projection is None:
            seg_idx, t, foot_n, foot_e, signed_xtrack = self._project_onto_path(
                pos_n, pos_e
            )
            self._update_path_progress(seg_idx, t)
        else:
            seg_idx, t, foot_n, foot_e, signed_xtrack = smooth_projection
        spray_active = self._segment_spray_active(seg_idx)

        # ---- Step 2: P0.1 + P1.2 — Closed-loop, xtrack-adaptive lookahead ----
        # P0.1: use last commanded speed (lookahead_time param is now live).
        # P1.2: add k_e · |xtrack| so off-path the lookahead extends and the
        #       rover re-acquires smoothly instead of cutting back hard.
        # Bootstrap: when _last_speed_cmd is 0 (first cycle, post-reset, or
        # post-stop) the inner expression is max_v * 0.5; the outer max() with
        # min_v only kicks in if the last commanded speed dropped below it
        # (e.g. just exited approach scaling on a tight corner).
        # Fix 2: low-pass filter v_for_ld (70/30 blend) to prevent 1-step
        # limit-cycle oscillation between lookahead distance and curvature.
        v_for_ld = max(min_v, self._last_speed_cmd if self._last_speed_cmd > 0.0
                       else max_v * 0.5)
        v_for_ld = 0.7 * v_for_ld + 0.3 * max_v
        l_d_raw = ld_gain * v_for_ld + xt_ld_gain * abs(signed_xtrack)
        l_d = self._clamp(l_d_raw, l_min, l_max)

        # Curvature-aware lookahead on arcs. Robust baseline (NOT adjacent
        # vertices) — see _path_curvature_at for why that distinction is
        # load-bearing.
        kappa_path = self._path_curvature_at(
            seg_idx, baseline_m=float(self.get_parameter("curvature_baseline_m").value)
        )

        # P5.1 (CAP): pure pursuit geometrically cuts INSIDE an arc by
        # e ≈ L²/(8R) = L²·κ/8 — no speed term, which is why halving
        # mission_speed on 2026-07-30 changed the curve error by nothing while
        # the lookahead sweep changed it by 3x. Bound that cut instead:
        #
        #     e ≤ e_target  ⇒  L ≤ sqrt(8·e_target/κ)
        #
        # Field-anchored: κ=0.43, e_target=0.005 ⇒ L ≤ 0.305 m. Measured at
        # L=0.30 on that arc: inside-cut went +1.15 cm → −0.40 cm and marking
        # RMS 2.31 → 1.34 cm (3/3 runs in spec, spread 0.09 cm), no ringing.
        # L=1.00 gave +3.80 cm, ratio 3.30 vs L² prediction 3.10.
        #
        # SMOOTH-PROFILE ONLY, by design. Segment mode keeps the long lookahead
        # so straights and ≥45° corners are untouched — a short lookahead there
        # is the 2026-07-29 instability regime (lookahead truncated to
        # 0.12-0.21 m gave bimodal 1.4/8 cm runs). Two `2x2 square` runs at a
        # global L=0.30 came back partial, which is why this is conditional.
        #
        # The cap REPLACES the Fix-1 floor rather than combining with it. They
        # are irreconcilable: floor = coeff/κ ∝ 1/κ, cap = sqrt(8e/κ) ∝ 1/√κ,
        # so the floor always wins as κ→0 — at the field-tested κ=0.43 it is
        # 0.465 m against a 0.305 m cap and would erase the effect entirely.
        # That is exactly why the validated field config also set
        # smooth_curvature_ld_coeff=0.0. The floor is also mis-shaped for its
        # own stated purpose (stopping the lookahead point landing on the
        # rover): coeff/κ = coeff·R GROWS on gentle arcs, where there is no
        # degeneracy risk at all. Degeneracy is guarded instead by the absolute
        # smooth_min_arc_ld_m floor plus the existing IDLE-fallback retry.
        ld_coeff = float(self.get_parameter("smooth_curvature_ld_coeff").value)
        cut_target = float(self.get_parameter("smooth_max_arc_cut_m").value)
        if kappa_path > 1e-6 and cut_target > 0.0:
            l_cap = math.sqrt(8.0 * cut_target / kappa_path)
            l_d = max(min(l_d, l_cap),
                      float(self.get_parameter("smooth_min_arc_ld_m").value))
        elif kappa_path > 1e-6 and ld_coeff > 0.0:
            # Fix 1 (legacy FLOOR), unchanged when the cap is off: ensure l_d
            # spans a fraction of the radius so the lookahead walk reliably
            # reaches past the foot. Applied after the [l_min, l_max] clamp and
            # may exceed max_lookahead_dist.
            l_d = max(l_d, ld_coeff / kappa_path)

        # ---- Step 3: Lookahead point (NED), then body-frame for κ ----
        lh_n, lh_e, hit_end = self._get_lookahead_point(seg_idx, foot_n, foot_e, l_d)

        # Body-frame y-component for curvature math (yaw_ned was already
        # extracted at the pose-in-NED step above; do NOT call
        # _enu_pose_to_ned again here).
        dn = lh_n - pos_n
        de = lh_e - pos_e
        # NED → body (NED yaw is CW+, North=0).
        #   x_body =  dn * cos(yaw) + de * sin(yaw)
        #   y_body = -dn * sin(yaw) + de * cos(yaw)
        # In our body convention y_body+ = right (FRD); RPP curvature uses
        # this directly. We do NOT publish ω, so the FRD-vs-FLU distinction
        # is purely internal to the κ computation.
        x_body = dn * math.cos(yaw_ned) + de * math.sin(yaw_ned)
        y_body = -dn * math.sin(yaw_ned) + de * math.cos(yaw_ned)
        l_actual = math.hypot(x_body, y_body)

        if l_actual < 1e-6:
            # Lookahead landed on top of us — retry with min_lookahead_dist
            # instead of publishing zero. Without this fallback, a short
            # adaptive lookahead on a curved path triggers IDLE every other
            # cycle, producing stop-start motion.
            lh_n, lh_e, hit_end = self._get_lookahead_point(
                seg_idx, foot_n, foot_e, l_min)
            dn = lh_n - pos_n
            de = lh_e - pos_e
            x_body = dn * math.cos(yaw_ned) + de * math.sin(yaw_ned)
            y_body = -dn * math.sin(yaw_ned) + de * math.cos(yaw_ned)
            l_actual = math.hypot(x_body, y_body)
            if l_actual < 1e-6:
                self._publish_zero(StateCode.IDLE, pose_age_ms=pose_age_s * 1000,
                                   dist_to_goal=dist_to_goal)
                return

        # ---- Step 4: Curvature ----
        kappa = (2.0 * y_body) / (l_actual * l_actual)

        # Heading error to lookahead in body frame (signed; for diagnostics)
        theta_e = math.atan2(y_body, x_body)

        # ---- Step 5: P1.1 — Predictive curvature-regulated speed ----
        # Steering still uses kappa (vehicle-relative κ at the lookahead).
        # Speed regulation now uses the WORST κ across N preview points
        # along the path ahead (path-intrinsic Menger). This anticipates
        # corners — the rover slows BEFORE entering them, not as it enters.
        # If preview_curvature_n <= 1 this falls back to baseline behaviour.
        if n_preview > 1:
            kappa_speed = self._max_preview_curvature(seg_idx, foot_n, foot_e,
                                                      l_d, n_preview)
        else:
            kappa_speed = abs(kappa)

        # P4.1 — Lateral acceleration constraint: v ≤ sqrt(a_lat_max / |κ|).
        # Physically correct form of the curvature speed limit. Replaces the
        # old linear R/min_radius scaling which underestimated speed at large
        # radii and was not grounded in vehicle dynamics.
        if kappa_speed > 1e-9:
            v_lat_limit = math.sqrt(a_lat_max / kappa_speed)
            speed = self._clamp(min(max_v, v_lat_limit), min_curv_v, max_v)
        else:
            speed = max_v

        # ---- Step 6: Approach scaling near goal ----
        # Open run: gate approach scaling behind min travel so a mission that
        # starts near its goal does not throttle from cycle 0.
        #
        # Closed run (circle): dist_to_goal is the Euclidean distance to the
        # FINAL waypoint, which ≈ the start (seam), so it stays < approach_d the
        # whole time the rover is near the seam — flooring speed to approach_v
        # before the loop is ever traced. Field bag 20260613_200921: the rover
        # reached the circle entry, then crept at ~3 cm/s for 118 s and never
        # went around (open line→arc→line U-turns trace fine precisely because
        # their goal is a full arc-length away and never triggers this). So for
        # closed runs scale on the REMAINING along-loop distance
        # (run_length − path_travel): full speed around the loop, decelerate
        # only in the final approach_d metres back to the seam.
        state_code = StateCode.TRACKING
        run_closed = bool(self._runs and self._runs[self._run_idx].get("closed"))
        if run_closed:
            run_len = float(self._runs[self._run_idx]["length"])
            remaining = max(0.0, run_len - self._path_travel_m)
            if remaining < approach_d:
                scale = self._clamp(remaining / approach_d, 0.0, 1.0)
                speed = min(speed, max(approach_v, speed * scale))
                state_code = StateCode.APPROACH
        elif dist_to_goal < approach_d and self._path_travel_m >= approach_d:
            # Linearly scale speed from full → approach_v as dist → 0
            scale = self._clamp(dist_to_goal / approach_d, 0.0, 1.0)
            approach_speed = max(approach_v, speed * scale)
            speed = min(speed, approach_speed)
            state_code = StateCode.APPROACH

        # ---- Step 6.5: Accel-UP ramp (mission-start motor-jerk guard) ----
        # Cap how fast `speed` can RAMP UP relative to the previous cycle.
        # Decel is deliberately unbounded: the P4 floor relies on a clean
        # step-to-zero at the goal, and a symmetric decel limiter would
        # cause goal overshoot beyond the 2 cm xy_goal_tolerance.
        # Uses self._tick_dt from _control_loop (same value as segment ramp).
        speed_before_accel = speed
        max_accel = self.get_parameter("max_linear_accel").value
        if max_accel > 0.0:
            speed = min(speed, self._last_speed_cmd + max_accel * self._tick_dt)

        # ---- Step 7: P4 floor — exact zero below threshold for clean stop ----
        # Apply the floor only when the intended target speed is below the
        # floor. During normal ramp-up, the accel-limited speed can be below
        # the floor for a few cycles; zeroing that value creates a permanent
        # 0 -> delta_up -> 0 deadlock.
        if speed < p4_floor and speed_before_accel < p4_floor and self._last_speed_cmd > 0.0:
            speed = 0.0

        # ---- P0.1: persist commanded speed for next cycle's L_d ----
        self._last_speed_cmd = speed

        # ---- P3.1 — Feedforward yaw rate (body-rate mode) ----
        # Must run AFTER all speed modifications (approach scaling, accel ramp, P4 floor)
        # so that yaw_rate_ff = κ·v uses the same speed that will actually be commanded.
        # Computing before approach scaling caused 4× over-command during deceleration.
        use_ff_yaw_rate = self.get_parameter("use_feedforward_yaw_rate").value
        if use_ff_yaw_rate:
            yaw_rate_ff = kappa * speed  # feedforward: κ·v (speed is now fully resolved)
            yaw_rate_fb = self.get_parameter("yaw_rate_feedback_gain").value * theta_e
            yaw_rate_body = yaw_rate_ff + yaw_rate_fb
            max_yr = self.get_parameter("max_yaw_rate_body").value
            if max_yr > 0.0:
                yaw_rate_body = self._clamp(yaw_rate_body, -max_yr, max_yr)
        else:
            yaw_rate_body = 0.0

        # ---- Step 8: Build NED velocity vector ----
        # Direction: unit vector from rover to lookahead point, in NED.
        # PX4 computes target_yaw = atan2(vE, vN) and aligns the rover with
        # that direction via its internal heading PID + spot-turn FSM.
        # P0.5: if enabled in twist_to_setpoint_node, we also publish an
        # explicit yaw setpoint that gives RPP authority over heading.
        unit_n = dn / l_actual if l_actual > 1e-9 else 0.0
        unit_e = de / l_actual if l_actual > 1e-9 else 0.0
        v_n = speed * unit_n
        v_e = speed * unit_e

        # B2 — signed lateral correction (default OFF: gain 0.0 keeps the
        # frozen controller byte-for-byte). Rotate the commanded bearing
        # toward the path: signed_xtrack + = right of path and NED bearing +
        # = clockwise (rightward), so the correction is −gain·xtrack. Without
        # this the smooth profile has no lateral feedback at all and PX4's
        # pure-P yaw loop parks the rover at a permanent offset on arcs.
        lat_gain = float(self.get_parameter("smooth_lateral_gain").value)
        if lat_gain > 0.0 and speed > 1e-6:
            max_corr = math.radians(
                float(self.get_parameter("smooth_lateral_max_deg").value))
            delta = self._clamp(-lat_gain * signed_xtrack, -max_corr, max_corr)
            bearing = math.atan2(v_e, v_n) + delta
            v_n = speed * math.cos(bearing)
            v_e = speed * math.sin(bearing)

        # BUG-T3 fix: clamp velocity bearing into forward cone so PX4
        # reverse-detection never flips the turn, even on the first run
        # (idx==0) where _run_alignment_hold is skipped.
        v_n, v_e = self._clamp_velocity_to_forward_cone(v_n, v_e, yaw_ned, speed)

        # P0.5: compute target yaw (NED: 0=North, CW+).
        # When |v| < 1 cm/s, freeze at last commanded yaw to avoid snapping
        # to North on stop (matches PX4 P4 patch behavior).
        speed_mag = math.hypot(v_n, v_e)
        if speed_mag > 0.01:
            yaw_target_ned = math.atan2(v_e, v_n)
        else:
            yaw_target_ned = self._last_yaw_cmd
        self._last_yaw_cmd = yaw_target_ned

        # ---- Publish ----
        self._publish_velocity(v_n, v_e)
        self._publish_yaw_rate(yaw_rate_body)  # P3.1

        # ---- Diagnostics ----
        self._publish_debug(
            cross_track=signed_xtrack,
            heading_err=theta_e,
            lookahead=l_actual,
            speed=speed,
            kappa=kappa,
            dist_goal=dist_to_goal,
            pose_age_ms=pose_age_s * 1000,
            state=state_code,
            l_d_raw=l_d_raw,                   # B1
            kappa_speed=kappa_speed,           # B1
            yaw_rate=yaw_rate_body,            # P3.1
            spray_active=spray_active,
        )

        # B3(a): the smooth profile must ALSO publish /rpp/segment_debug with a
        # TRACK_SEGMENT edge. Without this the smooth branch only ever emits
        # segment_debug via the SEGMENT profile / pivot paths, so the LAST state
        # on the wire before a smooth MARK run is the run-boundary pivot's
        # CORNER_ALIGN — which the spray node's pivot gate then latches for the
        # full segment_state_timeout_s, holding spray OFF for ~1 s (9.5-9.9 cm of
        # unpainted line at the start of every smooth run — field bug B3). It
        # also gives the spray B5 tracking-seen gate a real "run has started"
        # edge for BOTH profiles. Diagnostics ONLY: it publishes after the
        # velocity/yaw commands and does not mutate _segment_state (which the
        # SEGMENT profile owns), so no control path is affected. Corner-specific
        # fields carry NaN-free placeholders (this is a straight-tracking edge).
        self._publish_segment_debug(
            SegmentStateCode.TRACK_SEGMENT,
            seg_idx,
            dist_to_goal,       # [3] dist_to_segment_end (goal-relative here)
            dist_to_goal,       # [4] dist_to_corner (no corner in smooth)
            0.0,                # [5] corner angle — none
            yaw_target_ned,     # [6] target heading NED
            theta_e,            # [7] heading error
            yaw_rate_body,      # [8] yaw-rate command
        )

        r_eff = (1.0 / kappa_speed) if kappa_speed > 1e-9 else float("inf")
        self.get_logger().debug(
            f"[{state_code.name}] xtrack={signed_xtrack * 100:+.2f}cm "
            f"ld={l_actual:.2f}m(req={l_d:.2f}) κ={kappa:+.3f} κ_pred={kappa_speed:.3f} "
            f"R={r_eff if r_eff != float('inf') else -1:.2f}m "
            f"v=({v_n:+.3f},{v_e:+.3f})m/s speed={speed:.3f} "
            f"θe={math.degrees(theta_e):+.1f}° dgoal={dist_to_goal * 100:.1f}cm "
            f"hint={self._closest_seg_hint} fix={self._gps_fix_type} "
            f"hit_end={hit_end}"
        )

    # ==================================================================
    # Publishers
    # ==================================================================
    def _publish_conditioned_path(self, stamp, frame_id: str):
        """Publish the full conditioned mission (all runs concatenated).

        Runs after the first share their boundary vertex with the previous
        run, so the duplicate first pose is skipped to keep the published
        geometry clean for bag-based comparison.
        """
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.poses = []
        run_pose_lists = (
            [run["poses"] for run in self._runs] if self._runs
            else [self._path]
        )
        for k, poses in enumerate(run_pose_lists):
            for src in (poses[1:] if k > 0 else poses):
                ps = PoseStamped()
                ps.header.stamp = stamp
                ps.header.frame_id = frame_id
                ps.pose = src.pose
                msg.poses.append(ps)
        self._conditioned_path_pub.publish(msg)

    def _publish_segment_debug(
        self,
        state: SegmentStateCode,
        seg_idx: int,
        dist_to_segment_end: float,
        dist_to_corner: float,
        corner_angle_deg: float,
        target_heading_ned: float,
        heading_error_rad: float,
        yaw_rate_body: float,
    ):
        msg = Float32MultiArray()
        msg.layout.dim.append(
            MultiArrayDimension(label="rpp_segment_debug", size=10, stride=10)
        )
        msg.data = [
            self._profile_code(self._active_tracking_profile),  # [0] profile: 1 segment, 2 smooth
            float(state.value),                                 # [1] segment state
            float(seg_idx),                                     # [2] current segment index
            float(dist_to_segment_end),                         # [3] along-track distance to segment end
            float(dist_to_corner),                              # [4] Euclidean distance to corner/end
            float(corner_angle_deg),                            # [5] next-corner angle
            float(target_heading_ned),                          # [6] target heading, NED CW+
            float(heading_error_rad),                           # [7] target-current heading error
            float(yaw_rate_body),                               # [8] yaw-rate command (RPP FF, 0 during pivot)
            float(self._latest_yaw_rate_ned),                   # [9] actual yaw-rate NED (EKF, rad/s)
        ]
        self._last_segment_debug = tuple(msg.data)
        self._segment_dbg_pub.publish(msg)

    # Max bearing offset from the nose during a corner pivot. Beyond ~90° the
    # PX4 rover_differential reverse-detection (fwd_component < 0) flips the
    # command to reverse + opposite bearing, spot-turning the rover the WRONG
    # way into the 180° heading singularity where it deadlocks (observed in
    # bag square_cornerfix_20260611_174508: target +90°, rover ran to -90°).
    # Clamping the commanded velocity bearing into the forward cone keeps
    # fwd_component > 0, so the firmware spot-turns the short way, forward,
    # and the bearing converges to the exit heading as the rover rotates.
    _CORNER_MAX_BEARING_OFFSET_RAD = math.radians(75.0)

    def _corner_pivot_velocity(
        self, yaw_ned: float, heading_err: float, corner_speed: float
    ) -> tuple[float, float]:
        """NED velocity vector for a corner/alignment pivot.

        Points at the exit heading, but no more than ±75° off the current nose
        so PX4's reverse-detection never flips the turn. heading_err is the
        wrapped (target_heading - yaw_ned).
        """
        step = self._clamp(
            heading_err,
            -self._CORNER_MAX_BEARING_OFFSET_RAD,
            self._CORNER_MAX_BEARING_OFFSET_RAD,
        )
        cmd_bearing = yaw_ned + step
        return corner_speed * math.cos(cmd_bearing), corner_speed * math.sin(cmd_bearing)

    def _clamp_velocity_to_forward_cone(
        self,
        v_n: float,
        v_e: float,
        yaw_ned: float,
        speed: float,
    ) -> tuple[float, float]:
        """Prevent PX4 rover reverse-flip by keeping velocity bearing in forward cone.

        PX4 DifferentialVelControl derives desired heading from the velocity
        vector bearing = atan2(vE, vN). If the velocity vector is >90° from the
        rover nose, the forward projection fwd_component = v_n*cos(yaw) +
        v_e*sin(yaw) goes negative, and PX4 may choose reverse + 180° heading —
        spot-turning the rover the WRONG way (BUG-T3).

        This helper clamps the commanded velocity bearing into the same ±75°
        forward cone used by corner pivots (_corner_pivot_velocity), keeping
        fwd_component > 0 so the firmware spot-turns the short way, forward.

        No-op (returns (v_n, v_e) unchanged) when:
          - speed is zero
          - velocity magnitude is near-zero
          - the raw bearing is already inside the ±75° cone
        """
        if speed <= 1e-6:
            return v_n, v_e
        mag = math.hypot(v_n, v_e)
        if mag <= 1e-9:
            return v_n, v_e
        bearing = math.atan2(v_e, v_n)  # NED bearing: atan2(E, N); 0=North, CW+
        heading_err = self._angle_wrap(bearing - yaw_ned)
        if abs(heading_err) <= self._CORNER_MAX_BEARING_OFFSET_RAD:
            return v_n, v_e
        step = self._clamp(
            heading_err,
            -self._CORNER_MAX_BEARING_OFFSET_RAD,
            self._CORNER_MAX_BEARING_OFFSET_RAD,
        )
        cmd_bearing = yaw_ned + step
        # Preserve requested speed (the intended command magnitude).
        cmd_speed = speed
        return cmd_speed * math.cos(cmd_bearing), cmd_speed * math.sin(cmd_bearing)

    # Cap used only when velocity_local is stale. Fresh evidence that the rover
    # is moving always keeps CORNER_STOP active.
    _CORNER_STOP_MAX_HOLD_S = 2.0      # stale-velocity fallback cap

    def _vel_is_fresh(self) -> bool:
        """True when /velocity_local has arrived within the last 0.3 s."""
        if self._latest_vel_time is None:
            return False
        return (self.get_clock().now() - self._latest_vel_time).nanoseconds * 1e-9 < 0.3

    def _corner_brake_velocity(self, yaw_ned: float) -> tuple[float, float]:
        """Longitudinal NED command opposing body-forward motion.

        The command is exactly forward or reverse along the current body heading,
        never an arbitrary off-axis vector. This keeps PX4's reverse selection
        unambiguous and does not enter the BUG-T3 wrong-turn region. Returns zero
        for stale data, lateral-dominant motion, disabled braking, or an already
        stopped rover.
        """
        if not self._vel_is_fresh():
            return (0.0, 0.0)
        cap = float(self.get_parameter("segment_brake_velocity_cap_m_s").value)
        if cap <= 0.0:
            return (0.0, 0.0)
        v_n, v_e = self._latest_vel_ned
        speed = math.hypot(v_n, v_e)
        thresh = float(self.get_parameter("segment_stop_speed_threshold").value)
        if speed < thresh:
            return (0.0, 0.0)
        fwd_n, fwd_e = math.cos(yaw_ned), math.sin(yaw_ned)
        v_forward = v_n * fwd_n + v_e * fwd_e
        # Do not invent a longitudinal reverse command for sideways EKF motion.
        if abs(v_forward) < thresh or abs(v_forward) < 0.5 * speed:
            return (0.0, 0.0)
        mag = min(cap, abs(v_forward))
        sign = -1.0 if v_forward > 0.0 else 1.0
        return (sign * mag * fwd_n, sign * mag * fwd_e)

    def _align_speed_ok(self) -> bool:
        """Require fresh velocity and low linear speed for alignment release."""
        if not self._vel_is_fresh():
            return False
        v_n, v_e = self._latest_vel_ned
        return math.hypot(v_n, v_e) < float(
            self.get_parameter("segment_align_speed_threshold").value
        )

    def _reset_corner_pivot_state(self):
        self._corner_stop_entered = None
        self._corner_stop_settle_since = None
        self._corner_stop_complete = False
        self._pivot_started = None
        self._pivot_timeout_warned = False
        self._pivot_turn_angle_rad = 0.0
        self._align_settle_since = None

    def _corner_stop_satisfied(self) -> bool:
        """True once the rover is confirmed physically stopped at the corner.

        Confirmation = actual ground speed AND yaw-rate (both from velocity_local)
        below their thresholds continuously for segment_stop_dwell_s.

        Timeout policy (per-line extension fix): the rover is actively braked
        toward zero by the caller, so a FRESH velocity that is still above the
        stop threshold must NOT be allowed to time out into a pivot — that was
        the old bug where the 2 s cap fired while the rover was still drifting at
        ~0.14 m/s and the pivot started from the wrong point. The
        _CORNER_STOP_MAX_HOLD_S cap fires only when velocity data is STALE
        (cannot confirm the stop). Fresh telemetry above the threshold never
        advances into a pivot.
        """
        now = self.get_clock().now()
        if self._corner_stop_entered is None:
            self._corner_stop_entered = now

        speed_thresh = float(self.get_parameter("segment_stop_speed_threshold").value)
        yaw_rate_thresh = float(self.get_parameter("segment_stop_yaw_rate_threshold").value)
        dwell = float(self.get_parameter("segment_stop_dwell_s").value)

        fresh = self._vel_is_fresh()
        held = (now - self._corner_stop_entered).nanoseconds * 1e-9
        if not fresh:
            self._corner_stop_settle_since = None
            if held >= self._CORNER_STOP_MAX_HOLD_S:
                self.get_logger().warn(
                    "CORNER_STOP cap reached with STALE velocity — proceeding to pivot",
                    throttle_duration_sec=5.0,
                )
                return True
            return False

        v_n, v_e = self._latest_vel_ned
        speed_ok = math.hypot(v_n, v_e) < speed_thresh
        yaw_rate_ok = abs(self._latest_yaw_rate_ned) < yaw_rate_thresh

        both_ok = speed_ok and yaw_rate_ok
        if both_ok:
            if dwell <= 0.0:
                return True
            if self._corner_stop_settle_since is None:
                self._corner_stop_settle_since = now
            elif (now - self._corner_stop_settle_since).nanoseconds * 1e-9 >= dwell:
                return True
        else:
            self._corner_stop_settle_since = None  # any violation resets dwell

        return False

    def _pivot_timeout_budget(self) -> float:
        """Angle-aware pivot watchdog budget (seconds).

        The rover spot-turns at a roughly constant rate, so a 120° corner needs
        far longer than a 30° one. Budget = spinup_margin + angle/rate, floored
        at the legacy segment_turn_timeout_s (so small corners are unchanged)
        and clamped to segment_pivot_timeout_max_s. The corner magnitude is
        captured at pivot start (self._pivot_turn_angle_rad)."""
        base = float(self.get_parameter("segment_turn_timeout_s").value)
        rate = float(self.get_parameter("segment_nominal_pivot_rate_rad_s").value)
        margin = float(self.get_parameter("segment_pivot_spinup_margin_s").value)
        max_s = float(self.get_parameter("segment_pivot_timeout_max_s").value)
        angle = max(0.0, float(self._pivot_turn_angle_rad))
        budget = margin + (angle / rate if rate > 1e-6 else 0.0)
        budget = max(budget, base)          # never below the legacy floor
        if max_s > 0.0:
            budget = min(budget, max_s)     # safety clamp
        return budget

    def _pivot_timed_out(self, turn_angle_rad: float | None = None) -> bool:
        """Watchdog for the in-place pivot: True once CORNER_ALIGN has run
        longer than the angle-aware budget (_pivot_timeout_budget). Callers may
        then use the timeout heading tolerance and normal release gates. If
        velocity telemetry is stale, callers may use the timed-out state as a
        bounded fallback rather than deadlocking. `turn_angle_rad` (the corner
        magnitude) is captured on the first call to size the budget."""
        now = self.get_clock().now()
        if self._pivot_started is None:
            self._pivot_started = now
            if turn_angle_rad is not None and math.isfinite(turn_angle_rad):
                self._pivot_turn_angle_rad = abs(float(turn_angle_rad))
            return False
        timeout = self._pivot_timeout_budget()
        if timeout <= 0.0:
            return False
        if (now - self._pivot_started).nanoseconds * 1e-9 < timeout:
            return False
        if not self._pivot_timeout_warned:
            self._pivot_timeout_warned = True
            relaxed_tol = float(self.get_parameter(
                "segment_timeout_heading_tolerance_deg"
            ).value)
            self.get_logger().warn(
                f"Corner pivot exceeded {timeout:.1f}s "
                f"(angle≈{math.degrees(self._pivot_turn_angle_rad):.0f}°) — using "
                f"timeout heading tolerance {relaxed_tol:.1f} deg; still waiting for "
                "the alignment release gates"
            )
        return True

    def _publish_velocity(self, v_n: float, v_e: float):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "local_ned"
        msg.vector.x = float(v_n)
        msg.vector.y = float(v_e)
        msg.vector.z = 0.0
        self._vel_pub.publish(msg)

    # P3.1: publish feedforward yaw rate (body frame, rad/s)
    def _publish_yaw_rate(self, yaw_rate_body: float):
        msg = Float32()
        msg.data = float(yaw_rate_body)
        self._yaw_rate_pub.publish(msg)

    def _publish_zero(
        self,
        state: StateCode,
        pose_age_ms: float = float("nan"),
        dist_to_goal: float = float("nan"),
    ):
        """Publish (0, 0, 0) and a diagnostic. Used for IDLE/DONE/STALE/RTK_WAIT/JUMP_SKIP."""
        self._publish_velocity(0.0, 0.0)
        self._publish_yaw_rate(0.0)  # P3.1: zero yaw rate on stop
        # R8 fix: reset commanded-speed memory so that after a pause
        # (STALE / RTK_WAIT / JUMP_SKIP) the accel ramp restarts from 0
        # instead of resuming from the pre-pause speed and bypassing the
        # motor-start jerk protection.
        self._last_speed_cmd = 0.0
        self._publish_debug(
            cross_track=float("nan"),
            heading_err=float("nan"),
            lookahead=float("nan"),
            speed=0.0,
            kappa=float("nan"),
            dist_goal=dist_to_goal,
            pose_age_ms=pose_age_ms,
            state=state,
            l_d_raw=float("nan"),       # B1
            kappa_speed=float("nan"),   # B1
            yaw_rate=0.0,               # P3.1
            spray_active=False,
        )
        if self._active_tracking_profile == "segment":
            seg_idx = max(0, min(self._segment_idx, max(0, len(self._path) - 2)))
            dbg_state = self._segment_state
            if state == StateCode.DONE:
                dbg_state = SegmentStateCode.DONE
            self._publish_segment_debug(
                dbg_state,
                seg_idx,
                float("nan"),
                dist_to_goal,
                self._segment_angle_deg(seg_idx) if len(self._path) >= 3 else float("nan"),
                float("nan"),
                float("nan"),
                0.0,
            )

    def _segment_spray_active(self, seg_idx: int) -> bool:
        """Return True iff the currently tracked conditioned segment is MARK."""
        if self._path_done or len(self._path) < 2:
            return False
        if len(self._spray_flags) != len(self._path):
            return False
        seg = max(0, min(seg_idx, len(self._path) - 2))
        return bool(self._spray_flags[seg] and self._spray_flags[seg + 1])

    def _publish_spray_active(self, active: bool):
        msg = Bool()
        msg.data = bool(active)
        self._spray_active_pub.publish(msg)

    def _publish_debug(
        self,
        cross_track: float,
        heading_err: float,
        lookahead: float,
        speed: float,
        kappa: float,
        dist_goal: float,
        pose_age_ms: float,
        state: StateCode,
        l_d_raw: float = float("nan"),       # B1: requested Ld before clamp
        kappa_speed: float = float("nan"),   # B1: predictive κ used for speed
        yaw_rate: float = 0.0,               # P3.1: final clamped body yaw rate cmd
        spray_active: bool = False,
    ):
        """Publish /rpp/debug Float32MultiArray.

        Indices [0..10]: runtime tracking state (backward-compatible with
        existing consumers that only read [0..7]).
        Indices [11..38]: snapshot of all tunable RPP parameters. Every bag
        message is self-contained — you can replay and correlate parameter
        values with tracking performance without needing a separate param dump.
        Index [39]: spray_active.
        Indices [40..46]: active tracking profile and segment-mode params.
        """
        self._publish_spray_active(spray_active)
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label="rpp_debug",
                                                  size=47, stride=47))

        # ---- Snapshot all parameters once for this cycle ----
        p_max_linear_vel = float(self.get_parameter("max_linear_vel").value)
        p_min_linear_vel = float(self.get_parameter("min_linear_vel").value)
        p_min_lookahead = float(self.get_parameter("min_lookahead_dist").value)
        p_max_lookahead = float(self.get_parameter("max_lookahead_dist").value)
        p_lookahead_time = float(self.get_parameter("lookahead_time").value)
        p_a_lat_max = float(self.get_parameter("a_lat_max").value)
        p_reg_scale_min = float(self.get_parameter("regulated_linear_scaling_min_speed").value)
        p_goal_tol = float(self.get_parameter("xy_goal_tolerance").value)
        p_min_goal_travel = float(self.get_parameter("min_goal_travel_m").value)
        p_approach_dist = float(self.get_parameter("approach_velocity_scaling_dist").value)
        p_min_approach_v = float(self.get_parameter("min_approach_linear_velocity").value)
        p_p4_floor = float(self.get_parameter("p4_zero_vel_threshold").value)
        p_pose_max_age = float(self.get_parameter("pose_max_age_s").value)
        p_ekf_jump = float(self.get_parameter("ekf_jump_threshold_m").value)
        p_req_rtk = 1.0 if self.get_parameter("require_rtk_fix").value else 0.0
        p_preview_n = float(int(self.get_parameter("preview_curvature_n").value))
        p_xt_ld_gain = float(self.get_parameter("xtrack_lookahead_gain").value)
        p_resample = float(self.get_parameter("path_resample_spacing_m").value)
        p_corner_r = float(self.get_parameter("corner_smooth_radius_m").value)
        p_corner_pts = float(int(self.get_parameter("corner_smooth_arc_pts").value))
        p_use_extrap = 1.0 if self.get_parameter("use_imu_extrapolation").value else 0.0
        p_extrap_age = float(self.get_parameter("imu_max_extrap_age_s").value)
        p_ff_yaw = 1.0 if self.get_parameter("use_feedforward_yaw_rate").value else 0.0
        p_yr_fb_gain = float(self.get_parameter("yaw_rate_feedback_gain").value)
        p_max_yr = float(self.get_parameter("max_yaw_rate_body").value)
        p_max_accel = float(self.get_parameter("max_linear_accel").value)
        p_max_decel = float(self.get_parameter("max_linear_decel").value)
        p_mission_speed = float(self.get_parameter("mission_speed").value)
        p_profile_code = self._profile_code(self._active_tracking_profile)
        p_segment_corner_threshold = float(self.get_parameter("segment_corner_threshold_deg").value)
        p_segment_slowdown = float(self.get_parameter("segment_slowdown_dist").value)
        p_segment_min_speed = float(self.get_parameter("segment_min_corner_speed").value)
        p_segment_acceptance = float(self.get_parameter("segment_corner_acceptance_radius").value)
        p_segment_heading_tol = float(self.get_parameter("segment_heading_tolerance_deg").value)
        p_segment_yaw_gain = float(self.get_parameter("segment_yaw_rate_gain").value)

        msg.data = [
            float(cross_track),        # [0]  cross_track_error_m, signed
            float(heading_err),        # [1]  heading_error_rad
            float(lookahead),          # [2]  lookahead_dist_m (actual)
            float(speed),              # [3]  speed_cmd_m_s
            float(kappa),              # [4]  curvature_kappa (steering)
            float(dist_goal),          # [5]  dist_to_goal_m
            float(pose_age_ms),        # [6]  pose_age_ms
            float(state.value),        # [7]  state_code
            float(l_d_raw),            # [8]  l_d_raw_m       (B1)
            float(kappa_speed),        # [9]  kappa_speed     (B1)
            float(yaw_rate),           # [10] yaw_rate_cmd_rad_s (P3.1)
            p_max_linear_vel,          # [11] max_linear_vel
            p_min_linear_vel,          # [12] min_linear_vel
            p_min_lookahead,           # [13] min_lookahead_dist
            p_max_lookahead,           # [14] max_lookahead_dist
            p_lookahead_time,          # [15] lookahead_time
            p_a_lat_max,               # [16] a_lat_max
            p_reg_scale_min,           # [17] regulated_linear_scaling_min_speed
            p_goal_tol,                # [18] xy_goal_tolerance
            p_min_goal_travel,         # [19] min_goal_travel_m
            p_approach_dist,           # [20] approach_velocity_scaling_dist
            p_min_approach_v,          # [21] min_approach_linear_velocity
            p_p4_floor,                # [22] p4_zero_vel_threshold
            p_pose_max_age,            # [23] pose_max_age_s
            p_ekf_jump,                # [24] ekf_jump_threshold_m
            p_req_rtk,                 # [25] require_rtk_fix
            p_preview_n,               # [26] preview_curvature_n
            p_xt_ld_gain,              # [27] xtrack_lookahead_gain
            p_resample,                # [28] path_resample_spacing_m
            p_corner_r,                # [29] corner_smooth_radius_m
            p_corner_pts,              # [30] corner_smooth_arc_pts
            p_use_extrap,              # [31] use_imu_extrapolation
            p_extrap_age,              # [32] imu_max_extrap_age_s
            p_ff_yaw,                  # [33] use_feedforward_yaw_rate
            p_yr_fb_gain,              # [34] yaw_rate_feedback_gain
            p_max_yr,                  # [35] max_yaw_rate_body
            p_max_accel,               # [36] max_linear_accel
            p_max_decel,               # [37] max_linear_decel
            p_mission_speed,           # [38] mission_speed
            1.0 if spray_active else 0.0,  # [39] spray_active
            p_profile_code,            # [40] tracking_profile_code
            p_segment_corner_threshold,  # [41] segment_corner_threshold_deg
            p_segment_slowdown,        # [42] segment_slowdown_dist
            p_segment_min_speed,       # [43] segment_min_corner_speed
            p_segment_acceptance,      # [44] segment_corner_acceptance_radius
            p_segment_heading_tol,     # [45] segment_heading_tolerance_deg
            p_segment_yaw_gain,        # [46] segment_yaw_rate_gain
        ]
        self._dbg_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = None
    try:
        node = RPPControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is the normal SIGTERM path under systemd —
        # a traceback here on every service stop is noise, not a crash.
        pass
    finally:
        if node:
            # Last-gasp zero velocity on the way out — best-effort,
            # twist_to_setpoint_node will continue heartbeats with its own zero.
            try:
                node._publish_velocity(0.0, 0.0)
                node._publish_spray_active(False)
            except Exception:
                pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
