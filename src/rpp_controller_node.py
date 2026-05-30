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

import math
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import GPSRAW          # P0.3 RTK fix gate
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Float32


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
        self.declare_parameter("max_linear_vel",                      1.0)
        self.declare_parameter("min_linear_vel",                      0.15)
        self.declare_parameter("min_lookahead_dist",                  0.5)
        self.declare_parameter("max_lookahead_dist",                  1.5)
        self.declare_parameter("lookahead_time",                      1.5)

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
        self.declare_parameter("pose_max_age_s",                      0.5)    # 200 ms staleness threshold

        # Pose convergence gate: refuse to track until pose has been fresher
        # than pose_converge_age_ms continuously for pose_converge_time_s.
        # Prevents tracking against an EKF that hasn't settled yet (common at
        # startup after a static→moving transition).
        self.declare_parameter("pose_converge_age_ms",                 50.0)   # ms
        self.declare_parameter("pose_converge_time_s",                 1.0)    # s
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
        self.declare_parameter("xtrack_lookahead_gain",               0.3)

        # P1.5 — L_d low-pass filter.
        # Smooths step changes in L_d caused by kappa_path alternating between
        # 0 (collinear triplet at segment boundary) and 1/R (arc interior).
        # l_d_lpf_alpha=0 disables (raw L_d, original behaviour).
        # Recommended: 0.7 for arcs — one L_d time-constant ≈ DT/(1-α) = 0.07 s.
        # Set higher (0.85-0.90) for very smooth marking at cost of slower
        # xtrack recovery.  Never set ≥ 1.0 (filter becomes non-causal).
        self.declare_parameter("l_d_lpf_alpha",                       0.0)

        # Fix 1 — curvature-aware lookahead minimum.
        # On arcs, ensures l_d >= curvature_ld_factor / kappa_path so the
        # lookahead walk produces a κ close to 1/R.
        # Official Nav2 RPP uses no such term; L_d is purely velocity-scaled
        # and max_lookahead_dist is a hard ceiling (navigation2 source:
        # regulated_pure_pursuit_controller.cpp, getLookAheadDistance()).
        # Here the result is bounded by max_lookahead_dist to honour that
        # contract. Set 0.0 to match Nav2 behaviour exactly (pure velocity-
        # scaled lookahead). 0.75 was the original calibrated value;
        # 0.45 is recommended for R ≥ 1.5 m paths where the 0.75 value
        # forces L_d = 1.125 m and reduces xtrack correction gain 3.5×.
        self.declare_parameter("curvature_ld_factor",                 0.75)

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
        # corner_smooth_min_bend_deg: minimum bend angle at a vertex before
        #   smoothing is applied.  Vertices with a smaller bend are kept as-is.
        #   Default 10 deg prevents the smoother from inserting spurious high-κ
        #   arcs at the shallow bends of a polygon-arc path (e.g. 3.83 deg/step
        #   for arc_half_1m5), which would otherwise cause L_d oscillation and
        #   visible "spot turns" in the kappa command. Set 0.0 to restore the
        #   original behaviour (smooth every non-collinear vertex).
        self.declare_parameter("path_resample_spacing_m",             0.08)
        self.declare_parameter("corner_smooth_radius_m",              0.5)
        self.declare_parameter("corner_smooth_arc_pts",               6)
        self.declare_parameter("corner_smooth_min_bend_deg",          10.0)

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
        self.declare_parameter("yaw_rate_feedback_gain",              0.0)  # 0=pure FF; tune up once sign confirmed
        # Clamp on body yaw rate. Match PX4 RO_YAW_RATE_LIM (deg/s) converted
        # to rad/s so RPP doesn't request more than PX4 will honor.
        # 0.5 rad/s ≈ 28.6°/s — safe default. Set 0.0 to disable.
        self.declare_parameter("max_yaw_rate_body",                   1.0)

        # Acceleration ramp (P0 polish): cap how fast `speed` can RAMP UP
        # cycle-to-cycle. Prevents motor jerk on mission start and after a
        # stop/re-plan. Decel is intentionally NOT limited — the P4 floor
        # relies on instantaneous step-to-zero to trigger PX4 P4 yaw freeze
        # at the goal. Set to 0.0 to disable.
        self.declare_parameter("max_linear_accel",                    0.5)  # m/s²

        # P4.2 — Mission speed (operator-facing, set per job via ros2 param set)
        # This is the single knob the operator touches. It is capped by
        # max_linear_vel (hardware ceiling). Dependent params — approach distance
        # and EKF jump threshold — are derived from this value at runtime so the
        # operator never has to touch them.
        # Roads/large fields: 1.0 m/s  |  Sports fields/tight marking: 0.3–0.5 m/s
        self.declare_parameter("mission_speed",                       0.4)  # m/s

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
        self._pose: PoseStamped | None = None
        self._pose_recv_time: RclTime | None = None
        self._path_done = False
        self._path_travel_m: float = 0.0   # cumulative distance traveled along path

        # P1.4 — segment search hint: start projection from previous best seg
        self._closest_seg_hint: int = 0
        # Pre-computed nominal path curvature — median of interior-segment Menger
        # curvatures. Used as a floor in the curvature-aware lookahead minimum
        # to prevent edge segments (first/last few) from collapsing kappa_path
        # to ~0 and dropping the curvature minimum mid-arc.
        self._path_nominal_kappa: float = 0.0
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

        # P1.5 — L_d low-pass filter state.  Prevents sudden kappa spikes when
        # kappa_path jumps between path segments (Menger curvature of collinear
        # triplets returns 0, dropping L_d from the curvature minimum back to the
        # velocity floor every few waypoints).  Filter is reset to the first
        # computed L_d on each new path so there is no warm-up transient.
        self._l_d_filtered: float = 0.0
        self._l_d_filter_init: bool = False

        # P0.5 — explicit yaw_setpoint: persist last commanded yaw for freeze
        self._last_yaw_cmd: float = 0.0

        # P2.4 — Velocity-based pose extrapolation (latency closure).
        # We dead-reckon the pose forward by `vel_ned * dt_pose_age` to
        # close the MAVROS pose latency gap. We use velocity (not
        # acceleration) because at v=0.4 m/s the v·dt term is ~30× larger
        # than 0.5·a·dt² and is gravity-clean (PX4 EKF compensated).
        # `_latest_vel_ned` holds the latest NED velocity from
        # /mavros/local_position/velocity_local (already in ENU/NED at the
        # MAVROS boundary; we swap x↔y like for pose).
        self._latest_vel_ned: tuple[float, float] = (0.0, 0.0)
        self._latest_vel_time: RclTime | None = None

        # P0.2 — EKF jump detection: last accepted NED position
        self._last_pos: tuple[float, float] | None = None

        # P0.3 — RTK fix tracking
        self._gps_fix_type: int = 0  # 0 = no fix; 6 = RTK_FIXED

        # Pose convergence guard: prevent tracking from starting against a
        # stale/diverged EKF estimate.  Tracks the first time pose_age dropped
        # below the convergence threshold; tracking is gated until at least
        # `pose_converge_time_s` seconds have elapsed since that moment.
        self._pose_first_good_t: float = float("inf")

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
        # P3.1: optional yaw rate (body-rate mode) publisher
        self._yaw_rate_pub = self.create_publisher(
            Float32, "/rpp/yaw_rate_body", be_qos
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

        # Guard: if path content is identical to the existing path, skip
        # the full state reset to avoid unnecessary _path_travel_m = 0,
        # which causes IDLE↔TRACKING flicker when TRANSIENT_LOCAL QoS
        # re-delivers the same path message.
        if self._path and len(msg.poses) == len(self._path):
            same = True
            for new_ps, old_ps in zip(msg.poses, self._path):
                if (abs(new_ps.pose.position.x - old_ps.pose.position.x) > 1e-4
                        or abs(new_ps.pose.position.y - old_ps.pose.position.y) > 1e-4):
                    same = False
                    break
            if same:
                return

        # P1.3 — Path conditioning (linear resample + corner smoothing).
        # Operates on (north, east) tuples to keep the geometry code simple,
        # then converts back to PoseStamped at the end.
        raw_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        n_raw = len(raw_pts)

        resample_dx   = float(self.get_parameter("path_resample_spacing_m").value)
        corner_r      = float(self.get_parameter("corner_smooth_radius_m").value)
        arc_pts       = int(self.get_parameter("corner_smooth_arc_pts").value)
        min_bend_rad  = math.radians(
            float(self.get_parameter("corner_smooth_min_bend_deg").value))

        cond_pts = raw_pts
        if corner_r > 0.0 and len(cond_pts) >= 3:
            cond_pts = self._smooth_corners(
                cond_pts, corner_r, max(2, arc_pts), min_bend_rad)
        if resample_dx > 0.0 and len(cond_pts) >= 2:
            cond_pts = self._resample_path(cond_pts, resample_dx)

        # Build PoseStamped list in the same NED frame
        stamp = msg.header.stamp
        new_path: list[PoseStamped] = []
        for n, e in cond_pts:
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = expected
            ps.pose.position.x = float(n)
            ps.pose.position.y = float(e)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            new_path.append(ps)

        self._path = new_path
        self._path_done = False
        self._path_travel_m = 0.0   # reset travel distance on new path
        # P1.4 — reset hint so search starts from beginning of new path
        self._closest_seg_hint = 0
        # Pre-compute nominal path curvature (median of interior-segment Menger
        # curvatures). Skip first 2 and last 2 segments which have edge effects.
        _kappas = sorted(
            self._path_curvature_at(i) for i in range(2, len(new_path) - 2))
        self._path_nominal_kappa = _kappas[len(_kappas) // 2] if _kappas else 0.0
        # P1.4 fixup — force full scan on first projection after re-plan
        self._hint_valid = False
        # P0.1 — reset last speed so L_d bootstraps cleanly on new path
        self._last_speed_cmd = 0.0
        # P1.5 — reset L_d filter; first cycle will seed it to avoid transient
        self._l_d_filter_init = False
        # P0.2 — reset jump guard; first pose on new path is always "valid"
        self._last_pos = None
        # Reset pose convergence timer for the new path
        self._pose_first_good_t = float("inf")

        first = self._path[0].pose.position
        last = self._path[-1].pose.position
        if len(self._path) != n_raw:
            self.get_logger().info(
                f"Path conditioned: {n_raw} → {len(self._path)} waypoints "
                f"(resample={resample_dx:.2f}m, corner_r={corner_r:.2f}m), "
                f"first=({first.x:.2f}N, {first.y:.2f}E), "
                f"last=({last.x:.2f}N, {last.y:.2f}E)"
            )
        else:
            self.get_logger().info(
                f"Path accepted: {len(self._path)} waypoints, "
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
                       spacing: float) -> list[tuple[float, float]]:
        """Linearly resample a polyline to uniform spacing.

        The first and last points are kept exactly; intermediate samples are
        placed every `spacing` metres along the cumulative arc length.
        Geometry is preserved (straight segments stay straight).
        """
        if len(pts) < 2 or spacing <= 0.0:
            return list(pts)

        # Cumulative arc length per vertex
        cum = [0.0]
        for i in range(1, len(pts)):
            cum.append(cum[-1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                            pts[i][1] - pts[i - 1][1]))
        total = cum[-1]
        if total < spacing:
            return [pts[0], pts[-1]]

        n_samples = max(2, int(math.ceil(total / spacing)) + 1)
        out: list[tuple[float, float]] = []
        seg = 0
        for k in range(n_samples):
            target = (k / (n_samples - 1)) * total
            # Advance segment pointer
            while seg + 1 < len(cum) - 1 and cum[seg + 1] < target:
                seg += 1
            seg_len = cum[seg + 1] - cum[seg]
            if seg_len < 1e-12:
                out.append(pts[seg])
                continue
            t = (target - cum[seg]) / seg_len
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            n = pts[seg][0] + t * (pts[seg + 1][0] - pts[seg][0])
            e = pts[seg][1] + t * (pts[seg + 1][1] - pts[seg][1])
            out.append((n, e))
        # Force exact endpoints
        out[0] = pts[0]
        out[-1] = pts[-1]
        return out

    def _smooth_corners(self, pts: list[tuple[float, float]],
                        radius: float,
                        arc_pts: int,
                        min_bend_rad: float = 0.0) -> list[tuple[float, float]]:
        """Replace each interior vertex with an inscribed circular arc.

        Bounds path curvature at κ_max = 1/radius.

        For each interior vertex P with neighbours A (before) and B (after),
        compute the inscribed arc tangent to AP and PB at distance d from P.
        d = radius / tan(theta/2), where theta is the interior angle.
        Vertices where d > 0.45 * min(|AP|, |PB|) are skipped (segments too
        short to support the arc) and a warning is logged.
        Vertices whose bend angle (pi - theta) is below min_bend_rad are kept
        as-is — this prevents inserting spurious high-κ arcs at the shallow
        bends of a polygon-arc path, which would cause L_d oscillation.
        Endpoints are always kept.
        """
        n = len(pts)
        if n < 3 or radius <= 0.0:
            return list(pts)

        out: list[tuple[float, float]] = [pts[0]]
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
            bend  = math.pi - theta            # actual turn angle, 0..pi
            if theta < 1e-3 or bend < 1e-3:
                # Nearly collinear or full U-turn; keep vertex as-is
                out.append(pts[i])
                continue
            if min_bend_rad > 0.0 and bend < min_bend_rad:
                # Bend too shallow — preserve vertex, do not insert arc
                out.append(pts[i])
                continue

            # Tangent length from P to the arc start
            d = radius / math.tan(theta / 2.0)
            if d > 0.45 * min(l1, l2):
                # Segments too short for this radius — keep sharp corner
                skipped += 1
                out.append(pts[i])
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
            out.append((sa_n, sa_e))
            for k in range(1, arc_pts):
                a = ang1 + sweep * (k / arc_pts)
                out.append((cx_n + radius * math.cos(a),
                            cx_e + radius * math.sin(a)))
            out.append((sb_n, sb_e))

        out.append(pts[-1])
        if skipped > 0:
            self.get_logger().warn(
                f"corner_smooth: skipped {skipped} vertices — "
                f"adjacent segments shorter than {radius:.2f} m allows. "
                f"Reduce corner_smooth_radius_m or densify the path."
            )
        return out

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

        # P1.4: forward-only windowed search from the previous closest segment.
        # Window: [hint, hint+4) — no backward search to prevent foot-jitter on
        # arcs where adjacent segments are nearly equidistant and the foot
        # would otherwise alternate segments every cycle (see arc_fix_08 analysis).
        # At 0.4 m/s and 50 Hz the rover moves 0.008 m/cycle; hint+4 covers
        # every reachable segment without ever looking behind.
        # On the very first cycle after a path reset or EKF jump
        # (_hint_valid=False), do a full O(n) scan so we lock onto the correct
        # segment immediately. After that, the forward-only window is O(1).
        if not self._hint_valid:
            lo, hi = 0, n_pts - 1
        else:
            lo = self._closest_seg_hint  # no backward search — prevents foot-jitter on arcs
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
                dx = v_n * dt
                dy = v_e * dt

                pose_for_projection = PoseStamped()
                pose_for_projection.header = self._pose.header
                pose_for_projection.pose.position.x = self._pose.pose.position.x + dx
                pose_for_projection.pose.position.y = self._pose.pose.position.y + dy
                pose_for_projection.pose.position.z = self._pose.pose.position.z
                pose_for_projection.pose.orientation = self._pose.pose.orientation

                self.get_logger().debug(
                    f"P2.4 v-extrapolation: pose_age={pose_age_s*1000:.1f}ms, "
                    f"v_ned=({v_n:+.2f},{v_e:+.2f}) m/s, "
                    f"Δpos=({dx*100:+.2f},{dy*100:+.2f}) cm"
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

        # ---- Pose convergence guard ----
        # Gate tracking until the EKF pose has been consistently fresh for at
        # least 1 second, preventing the controller from tracking against a
        # stale/diverged pose that reports CTE = 0.8 m and 50° heading error.
        converge_thr_ms = float(self.get_parameter("pose_converge_age_ms").value)
        converge_time_s = float(self.get_parameter("pose_converge_time_s").value)
        pose_age_ms_val = pose_age_s * 1000.0
        if pose_age_ms_val < converge_thr_ms:
            if self._pose_first_good_t == float("inf"):
                self._pose_first_good_t = (
                    self.get_clock().now().nanoseconds * 1e-9)
                self.get_logger().info(
                    f"Pose converging — age={pose_age_ms_val:.1f}ms < "
                    f"{converge_thr_ms:.0f}ms threshold; stabilising for "
                    f"{converge_time_s:.1f}s before tracking"
                )
        elif pose_age_ms_val > converge_thr_ms * 3.0:
            self._pose_first_good_t = float("inf")

        if (self._pose_first_good_t != float("inf")
                and (self.get_clock().now().nanoseconds * 1e-9
                     - self._pose_first_good_t) < converge_time_s):
            self._publish_zero(StateCode.IDLE, pose_age_ms=pose_age_ms_val)
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

        # ---- Goal check ----
        # Skip until the rover has traveled min_goal_travel_m along the path.
        # Prevents immediate DONE on closed-loop paths where the rover starts
        # at the final waypoint (e.g., square_2x2 with auto_origin).
        min_travel = self.get_parameter("min_goal_travel_m").value
        final = self._path[-1].pose.position
        dist_to_goal = self._dist(pos_n, pos_e, final.x, final.y)
        if dist_to_goal <= goal_tol and self._path_travel_m >= min_travel:
            self.get_logger().info(
                f"Path complete — within {dist_to_goal * 100:.1f} cm of goal "
                f"(tol={goal_tol * 100:.1f} cm)"
            )
            self._path_done = True
            self._publish_zero(StateCode.DONE, pose_age_ms=pose_age_s * 1000,
                               dist_to_goal=dist_to_goal)
            return

        # ---- Step 1: Closest-point projection (segment, not vertex) ----
        # P1.4: _project_onto_path uses _closest_seg_hint internally.
        seg_idx, t, foot_n, foot_e, signed_xtrack = self._project_onto_path(
            pos_n, pos_e
        )

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
        # spans at least curvature_ld_factor / kappa_path so the lookahead
        # walk produces κ ≈ 1/R.  Result is clamped to l_max so that
        # max_lookahead_dist remains a hard ceiling (per Nav2 RPP contract).
        # Set curvature_ld_factor=0.0 to disable (pure velocity-scaled L_d).
        curv_ld_factor = self.get_parameter("curvature_ld_factor").value
        kappa_path = self._path_curvature_at(seg_idx)
        # Fix 1a: Floor kappa_path to prevent edge-segment collapse. The
        # Menger curvature at the first/last few segments returns ~0 or
        # underestimates the true arc curvature, causing the curvature
        # minimum to drop out or oscillate at arc boundaries (~4 segments
        # per end = "7-8 transitions" across a semicircle).
        if self._path_nominal_kappa > 1e-6:
            kappa_path = max(kappa_path, self._path_nominal_kappa * 0.5)
        if curv_ld_factor > 0.0 and kappa_path > 1e-6:
            l_d = min(l_max, max(l_d, curv_ld_factor / kappa_path))

        # Fix 4a: hard-floor lookahead in the approach zone to prevent κ
        # collapse.  As dist_goal → 0, v_path → 0 → l_d → 0 → κ = 2·y_body/lh²
        # spikes to ±20 because pure pursuit breaks down at sub-cm lookaheads.
        if dist_to_goal < approach_d and self._path_travel_m >= approach_d:
            l_d = max(l_d, l_min)

        # P1.5 — L_d low-pass filter.  Applied after all clamps so the filter
        # smooths the final value sent to the lookahead walk.  Seeded on the
        # first cycle of a new path to avoid a ramp-up transient.
        lpf_alpha = float(self.get_parameter("l_d_lpf_alpha").value)
        if lpf_alpha > 0.0:
            if not self._l_d_filter_init:
                self._l_d_filtered = l_d
                self._l_d_filter_init = True
            self._l_d_filtered = lpf_alpha * self._l_d_filtered + (1.0 - lpf_alpha) * l_d
            l_d = self._l_d_filtered

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

        # Fix 4b: clamp κ to a physically meaningful maximum so the goal
        # approach doesn't produce κ = ±20 when the lookahead collapses
        # below 2 cm.  κ_max = 3 / max(l_actual, l_min) bounds the yaw
        # rate to something the vehicle can actually achieve.
        kappa = self._clamp(
            kappa,
            -3.0 / max(l_actual, l_min),
            +3.0 / max(l_actual, l_min))

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
        max_accel = self.get_parameter("max_linear_accel").value
        if max_accel > 0.0:
            delta_up = max_accel / self.CONTROL_HZ
            speed = min(speed, self._last_speed_cmd + delta_up)

        # ---- Step 7: P4 floor — exact zero below threshold for clean stop ----
        # Skip during initial ramp-up from standstill, otherwise the accel
        # ramp (0.01 m/s per cycle at default 0.5 m/s²) can never exceed the
        # floor (0.02 m/s) and the rover is permanently stuck at zero.
        if speed < p4_floor and self._last_speed_cmd > 0.0:
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
        )

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
    ):
        """Publish /rpp/debug Float32MultiArray.

        Layout is append-only — indices [0..7] retained verbatim from the
        original 8-field schema so existing consumers (xtrack_logger,
        mission_runner, server/ros_node, server/main, server/rpp_status)
        continue to work without changes. New B1 fields are at [8..9],
        P3.1 field at [10].
        """
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label="rpp_debug",
                                                  size=11, stride=11))
        msg.data = [
            float(cross_track),     # [0]  cross_track_error_m, signed
            float(heading_err),     # [1]  heading_error_rad
            float(lookahead),       # [2]  lookahead_dist_m (actual)
            float(speed),           # [3]  speed_cmd_m_s
            float(kappa),           # [4]  curvature_kappa (steering)
            float(dist_goal),       # [5]  dist_to_goal_m
            float(pose_age_ms),     # [6]  pose_age_ms
            float(state.value),     # [7]  state_code
            float(l_d_raw),         # [8]  l_d_raw_m       (B1)
            float(kappa_speed),     # [9]  kappa_speed     (B1)
            float(yaw_rate),        # [10] yaw_rate_cmd_rad_s (P3.1)
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
            except Exception:
                pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
