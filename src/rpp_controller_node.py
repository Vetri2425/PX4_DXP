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

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import GPSRAW          # P0.3 RTK fix gate
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension, Float32


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
        self.declare_parameter("segment_slowdown_dist",               0.50)
        self.declare_parameter("segment_min_corner_speed",             0.08)
        self.declare_parameter("segment_corner_acceptance_radius",     0.05)
        self.declare_parameter("segment_heading_tolerance_deg",        7.5)
        self.declare_parameter("segment_yaw_rate_gain",                1.5)

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
        self._run_idx: int = 0
        self._run_align_pending: bool = False
        self._segment_idx: int = 0
        self._segment_state: SegmentStateCode = SegmentStateCode.INACTIVE
        self._last_segment_debug: tuple[float, ...] = (
            0.0, 0.0, 0.0, float("nan"), float("nan"),
            float("nan"), float("nan"), float("nan"), 0.0,
        )
        self._pose: PoseStamped | None = None
        self._pose_recv_time: RclTime | None = None
        self._path_done = False
        self._path_travel_m: float = 0.0   # cumulative distance traveled along path

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

        # P0.2 — EKF jump detection: last accepted NED position
        self._last_pos: tuple[float, float] | None = None

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
        raw_flags = [p.pose.position.z > 0.5 for p in msg.poses]
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
            raw_runs = [
                sub
                for run in self._split_runs_by_flag(raw_pts, raw_flags)
                for sub in self._split_run_at_corners(*run, threshold)
            ]
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
                    run_pts, run_flags
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
                "poses": self._build_poses(c_pts, c_flags, stamp, expected),
                "flags": list(c_flags),
                "profile": profile,
                "length": self._pts_length(c_pts),
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

        self._runs = runs
        # Mission-level resets; per-run state is reset inside _apply_run.
        # P0.1 — reset last speed so L_d bootstraps cleanly on new path
        self._last_speed_cmd = 0.0
        # P0.2 — reset jump guard; first pose on new path is always "valid"
        self._last_pos = None
        self._apply_run(0)
        self._publish_conditioned_path(stamp, expected)

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
        """Track latest NED linear velocity from MAVROS (already EKF-clean).

        MAVROS publishes `/mavros/local_position/velocity_local` in ENU:
          msg.twist.linear.x = East,  msg.twist.linear.y = North
        We swap to NED (x=North, y=East) the same way we do for pose.
        """
        v_north = msg.twist.linear.y    # ENU y → NED x
        v_east = msg.twist.linear.x     # ENU x → NED y
        self._latest_vel_ned = (v_north, v_east)
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

    @classmethod
    def _simplify_path_for_profile(
        cls,
        pts: list[tuple[float, float]],
        flags: list[bool] | None = None,
        collinear_tol_deg: float = 5.0,
    ) -> tuple[list[tuple[float, float]], list[bool]]:
        """Remove duplicate and same-heading vertices while preserving corners.

        This is used for segment-mode decisions and tracking. Generated
        squares/rectangles often arrive as many collinear samples per side;
        segment mode needs the side endpoints, not every resampled point.
        Flag changes are preserved so mark/transit boundaries are not erased.
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

        if len(clean_pts) < 3:
            return clean_pts, clean_flags

        tol = math.radians(collinear_tol_deg)
        out_pts: list[tuple[float, float]] = [clean_pts[0]]
        out_flags: list[bool] = [clean_flags[0]]
        for i in range(1, len(clean_pts) - 1):
            prev_pt = out_pts[-1]
            this_pt = clean_pts[i]
            next_pt = clean_pts[i + 1]
            h0 = cls._segment_heading(prev_pt, this_pt)
            h1 = cls._segment_heading(this_pt, next_pt)
            heading_change = cls._heading_delta(h0, h1)
            flag_boundary = clean_flags[i - 1] != clean_flags[i] or clean_flags[i] != clean_flags[i + 1]
            if heading_change <= tol and not flag_boundary:
                continue
            out_pts.append(this_pt)
            out_flags.append(clean_flags[i])

        out_pts.append(clean_pts[-1])
        out_flags.append(clean_flags[-1])
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
    ) -> list[PoseStamped]:
        poses: list[PoseStamped] = []
        for (n, e), flag in zip(pts, flags):
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = frame_id
            ps.pose.position.x = float(n)
            ps.pose.position.y = float(e)
            ps.pose.position.z = 1.0 if flag else 0.0
            ps.pose.orientation.w = 1.0
            poses.append(ps)
        return poses

    def _apply_run(self, idx: int) -> None:
        """Make run `idx` the actively tracked path; reset per-run state."""
        run = self._runs[idx]
        # Runs after the first may start at a hard corner (sub-split) or a
        # transit↔entity boundary: pivot toward the run's initial heading
        # before tracking it (see _run_alignment_hold). The first run keeps
        # the existing drive-from-anywhere start behavior.
        self._run_align_pending = idx > 0
        self._run_idx = idx
        self._path = run["poses"]
        self._spray_flags = list(run["flags"])
        self._active_tracking_profile = run["profile"]
        self._segment_idx = 0
        self._segment_state = (
            SegmentStateCode.TRACK_SEGMENT
            if run["profile"] == "segment" and len(self._path) >= 2
            else SegmentStateCode.INACTIVE
        )
        self._path_done = False
        self._path_travel_m = 0.0   # reset travel distance per run
        # P1.4 — reset hint so search starts from beginning of the run
        self._closest_seg_hint = 0
        self._hint_valid = False
        self._last_segment_debug = (
            self._profile_code(run["profile"]),
            float(self._segment_state.value),
            0.0,
            float("nan"), float("nan"), float("nan"),
            float("nan"), float("nan"), 0.0,
        )

    def _advance_run(self) -> bool:
        """Switch to the next run, if any. False means mission complete."""
        if self._run_idx + 1 >= len(self._runs):
            return False
        self._apply_run(self._run_idx + 1)
        run = self._runs[self._run_idx]
        self.get_logger().info(
            f"Run {self._run_idx + 1}/{len(self._runs)} started "
            f"(profile={run['profile']}, {len(run['poses'])} waypoints, "
            f"{run['length']:.2f} m)"
        )
        return True

    def _run_min_travel(self) -> float:
        """min_goal_travel_m capped at half the active run length.

        The param guards against instant-DONE on closed-loop missions; a
        single run is one entity or transit hop and can be far shorter
        than the param, which would otherwise deadlock run completion.
        """
        min_travel = float(self.get_parameter("min_goal_travel_m").value)
        if not self._runs:
            return min_travel
        return min(min_travel, 0.5 * self._runs[self._run_idx]["length"])

    def _run_alignment_hold(
        self, pos_n: float, pos_e: float, yaw_ned: float, pose_age_s: float
    ) -> bool:
        """Pivot in place toward the new run's initial heading.

        Returns True while holding (zero velocity + yaw rate published);
        False once aligned (or alignment is not applicable), letting the
        normal control flow proceed. Already-aligned transitions (e.g. a
        transit continuing straight into an entity) pass through with no
        stop. Like segment CORNER_ALIGN, yaw rate is the only actuation
        here, so use_feedforward_yaw_rate is intentionally not consulted.
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
        if abs(heading_err) <= heading_tol:
            self._run_align_pending = False
            return False

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

        final = self._path[-1].pose.position
        dist_to_goal = self._dist(pos_n, pos_e, final.x, final.y)
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
        d = self._dist(pos_n, pos_e, foot_n, foot_e)
        cross_z = dx * (pos_e - foot_e) - dy * (pos_n - foot_n)
        signed_e = math.copysign(d, cross_z) if d > 0.0 else 0.0
        seg_len = math.sqrt(seg_sq)
        dist_to_end_along = (1.0 - t) * seg_len
        return t, foot_n, foot_e, signed_e, dist_to_end_along

    def _path_curvature_at(self, seg_idx: int) -> float:
        """Estimate path curvature at the projection foot using Menger
        curvature of three consecutive path vertices centred on seg_idx.

        Returns 1/m curvature (0.0 for straight lines, >0 for curves).
        Used to enforce a curvature-adequate minimum lookahead on arcs.
        """
        n_pts = len(self._path)
        if n_pts < 3:
            return 0.0
        i0 = max(0, seg_idx - 1)
        i1 = seg_idx
        i2 = min(n_pts - 1, seg_idx + 1)
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
        dist_to_corner = self._dist(pos_n, pos_e, b.x, b.y)
        corner_angle = self._segment_angle_deg(seg_idx)
        spray_active = self._segment_spray_active(seg_idx)

        goal_tol = float(self.get_parameter("xy_goal_tolerance").value)
        min_travel = self._run_min_travel()
        if final_segment and dist_to_corner <= goal_tol and self._path_travel_m >= min_travel:
            # End of run: advance and return — the next 20 ms cycle runs
            # _run_alignment_hold (pivot if needed), then the new run's
            # profile takes over.
            if self._advance_run():
                return
            self.get_logger().info(
                f"Segment path complete — within {dist_to_corner * 100:.1f} cm "
                f"of final point (tol={goal_tol * 100:.1f} cm)"
            )
            self._path_done = True
            self._segment_state = SegmentStateCode.DONE
            self._publish_zero(
                StateCode.DONE,
                pose_age_ms=pose_age_s * 1000.0,
                dist_to_goal=dist_to_corner,
            )
            self._publish_segment_debug(
                self._segment_state, seg_idx, dist_to_end_along, dist_to_corner,
                corner_angle, float("nan"), float("nan"), 0.0,
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
            c = self._path[seg_idx + 2].pose.position
            target_heading = math.atan2(c.y - b.y, c.x - b.x)
            heading_err = self._angle_wrap(target_heading - yaw_ned)
            if abs(heading_err) <= heading_tol:
                self._segment_idx += 1
                self._segment_state = SegmentStateCode.TRACK_SEGMENT
                self._publish_segment_debug(
                    self._segment_state, self._segment_idx, float("nan"),
                    dist_to_corner, corner_angle, target_heading, heading_err, 0.0,
                )
                self._control_segment_profile(
                    pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal
                )
                return

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

        dir_n = (b.x - a.x) / seg_len
        dir_e = (b.y - a.y) / seg_len
        lookahead_along = min(max(0.0, dist_to_end_along), l_d)
        if lookahead_along <= 1e-6:
            lh_n, lh_e = b.x, b.y
        else:
            lh_n = foot_n + dir_n * lookahead_along
            lh_e = foot_e + dir_e * lookahead_along

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
        if not final_segment and slowdown > 1e-6 and dist_to_corner < slowdown:
            scale = self._clamp(dist_to_corner / slowdown, 0.0, 1.0)
            speed = max(min_corner_speed, max_v * scale)
            self._segment_state = SegmentStateCode.PRE_CORNER_SLOWDOWN

        max_accel = float(self.get_parameter("max_linear_accel").value)
        speed_before_accel = speed
        if max_accel > 0.0:
            speed = min(speed, self._last_speed_cmd + max_accel / self.CONTROL_HZ)

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

        # ---- P0.2: EKF / position-jump detection ----
        # If the pose jumps further than is physically possible in one control
        # cycle (max_v * dt + 3σ_pos), it's an EKF reset or RTK acquisition
        # artefact. Skip this cycle and do NOT update the controller.
        # We still update _last_pos so the next cycle compares against the
        # new (post-jump) position — only one cycle is skipped per event.
        if self._last_pos is not None:
            jump_m = math.hypot(pos_n - self._last_pos[0],
                                pos_e - self._last_pos[1])
            if jump_m > jump_thr:
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
        # Accumulate path travel distance for min_goal_travel_m gate.
        # Must happen BEFORE _last_pos update so delta is non-zero.
        if self._last_pos is not None:
            dx = pos_n - self._last_pos[0]
            dy = pos_e - self._last_pos[1]
            self._path_travel_m += math.hypot(dx, dy)

        self._last_pos = (pos_n, pos_e)

        # ---- Run-transition alignment (per-entity profile switching) ----
        # After advancing to a new run, pivot toward its initial heading
        # before tracking it. No-op when already aligned.
        if self._run_alignment_hold(pos_n, pos_e, yaw_ned, pose_age_s):
            return

        # ---- Goal check ----
        # Skip until the rover has traveled min_goal_travel_m along the path.
        # Prevents immediate DONE on closed-loop paths where the rover starts
        # at the final waypoint (e.g., square_2x2 with auto_origin).
        min_travel = self._run_min_travel()
        final = self._path[-1].pose.position
        dist_to_goal = self._dist(pos_n, pos_e, final.x, final.y)
        if dist_to_goal <= goal_tol and self._path_travel_m >= min_travel:
            # End of the active run: advance to the next run (per-entity
            # profile switching). The next 20 ms cycle pivots via
            # _run_alignment_hold if needed, then tracks the new run. DONE
            # is only published after the last run — the server's mission-
            # complete settling watches for it.
            if self._advance_run():
                return
            else:
                self.get_logger().info(
                    f"Path complete — within {dist_to_goal * 100:.1f} cm of goal "
                    f"(tol={goal_tol * 100:.1f} cm)"
                )
                self._path_done = True
                self._publish_zero(StateCode.DONE, pose_age_ms=pose_age_s * 1000,
                                   dist_to_goal=dist_to_goal)
                return

        if self._active_tracking_profile == "segment":
            self._control_segment_profile(
                pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal
            )
            return

        # ---- Step 1: Closest-point projection (segment, not vertex) ----
        # P1.4: _project_onto_path uses _closest_seg_hint internally.
        seg_idx, t, foot_n, foot_e, signed_xtrack = self._project_onto_path(
            pos_n, pos_e
        )
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

        # Fix 1: curvature-aware minimum lookahead — on arcs, ensure l_d
        # spans at least 1/3 of the radius so the lookahead walk reliably
        # reaches past the foot. Without this, short lookaheads on tight
        # arcs can land at the rover position, triggering the IDLE path.
        kappa_path = self._path_curvature_at(seg_idx)
        if kappa_path > 1e-6:
            l_d = max(l_d, 0.35 / kappa_path)

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
        # FIX: Gate approach scaling behind min_goal_travel_m.  Without this,
        # closed-loop paths (e.g. square_2x2) where the start IS the goal
        # immediately trigger approach scaling, floor speed to approach_v
        # (0.05 m/s), and the rover can never accelerate — it sits there
        # yawing in place at 5 cm/s forever.
        state_code = StateCode.TRACKING
        if dist_to_goal < approach_d and self._path_travel_m >= approach_d:
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
        speed_before_accel = speed
        max_accel = self.get_parameter("max_linear_accel").value
        if max_accel > 0.0:
            delta_up = max_accel / self.CONTROL_HZ
            speed = min(speed, self._last_speed_cmd + delta_up)

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
            MultiArrayDimension(label="rpp_segment_debug", size=9, stride=9)
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
            float(yaw_rate_body),                               # [8] yaw-rate command
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
    except KeyboardInterrupt:
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
