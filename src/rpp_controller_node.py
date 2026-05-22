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

Phase C / P0.5 / P3.1 — opt-in upgrades (default OFF for backward compat)
------------------------------------------------------------------------
  P0.5  Explicit yaw setpoint: publishes /rpp/yaw_setpoint_ned (Float32) so
        twist_to_setpoint_node can include yaw in the PositionTarget setpoint.
        Gives RPP authority over heading instead of relying on PX4's
        atan2(vE, vN) derivation. When |v| < 1 cm/s, yaw freezes at the last
        commanded heading (matches PX4 P4 patch behavior — no North snap).
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

Topic:  /rpp/yaw_setpoint_ned   (std_msgs/Float32)  [P0.5]
        data             = target yaw (radians, NED convention)
                           Derived from velocity vector: atan2(v_e, v_n).
                           When |v| < 1 cm/s, yaw is frozen at last commanded value.
                           Allows twist_to_setpoint_node to include yaw in PositionTarget
                           and gives RPP authority over heading instead of relying on
                           PX4's atan2(vE, vN) derivation.

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
        self.declare_parameter("max_linear_vel",                      0.4)
        self.declare_parameter("min_linear_vel",                      0.15)
        self.declare_parameter("min_lookahead_dist",                  0.30)
        self.declare_parameter("max_lookahead_dist",                  0.60)
        self.declare_parameter("lookahead_time",                      1.2)

        # Curvature regulation
        self.declare_parameter("regulated_linear_scaling_min_radius", 0.6)
        self.declare_parameter("regulated_linear_scaling_min_speed",  0.15)

        # Goal handling
        self.declare_parameter("xy_goal_tolerance",                   0.02)   # 2 cm
        self.declare_parameter("approach_velocity_scaling_dist",      0.6)    # m
        self.declare_parameter("min_approach_linear_velocity",        0.05)
        self.declare_parameter("p4_zero_vel_threshold",               0.02)   # m/s; floor speed below this to exactly 0 to trigger PX4 P4

        # Safety
        self.declare_parameter("pose_max_age_s",                      0.2)    # 200 ms staleness threshold
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
        self.declare_parameter("preview_curvature_n",                 3)

        # P1.2 — Adaptive lookahead based on cross-track error
        # L_d = clamp(lookahead_time * v + xtrack_lookahead_gain * |e_⊥|, L_min, L_max)
        # Set 0.0 to disable the cross-track term (pure velocity-scaled).
        # 1.0 means a 10 cm cross-track adds 10 cm of lookahead.
        self.declare_parameter("xtrack_lookahead_gain",               1.0)

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
        self.declare_parameter("path_resample_spacing_m",             0.0)
        self.declare_parameter("corner_smooth_radius_m",              0.0)
        self.declare_parameter("corner_smooth_arc_pts",               6)

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
        self.declare_parameter("use_feedforward_yaw_rate",            False)
        self.declare_parameter("yaw_rate_feedback_gain",              0.5)  # heading error feedback

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._path: list[PoseStamped] = []
        self._pose: PoseStamped | None = None
        self._pose_recv_time: RclTime | None = None
        self._path_done = False

        # P1.4 — segment search hint: start projection from previous best seg
        self._closest_seg_hint: int = 0
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
        # /mavros/local_position/velocity_local (already in ENU/NED at the
        # MAVROS boundary; we swap x↔y like for pose).
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
        # P0.5: explicit yaw setpoint (NED, radians).  When use_explicit_yaw=True
        # in twist_to_setpoint_node, this gives RPP authority over heading
        # instead of relying on PX4's atan2(vE, vN) derivation.
        self._yaw_pub = self.create_publisher(
            Float32, "/rpp/yaw_setpoint_ned", be_qos
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

        # P1.3 — Path conditioning (linear resample + corner smoothing).
        # Operates on (north, east) tuples to keep the geometry code simple,
        # then converts back to PoseStamped at the end.
        raw_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        n_raw = len(raw_pts)

        resample_dx = float(self.get_parameter("path_resample_spacing_m").value)
        corner_r    = float(self.get_parameter("corner_smooth_radius_m").value)
        arc_pts     = int(self.get_parameter("corner_smooth_arc_pts").value)

        cond_pts = raw_pts
        if corner_r > 0.0 and len(cond_pts) >= 3:
            cond_pts = self._smooth_corners(cond_pts, corner_r, max(2, arc_pts))
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
        # P1.4 — reset hint so search starts from beginning of new path
        self._closest_seg_hint = 0
        # P1.4 fixup — force full scan on first projection after re-plan
        self._hint_valid = False
        # P0.1 — reset last speed so L_d bootstraps cleanly on new path
        self._last_speed_cmd = 0.0
        # P0.2 — reset jump guard; first pose on new path is always "valid"
        self._last_pos = None

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
        max_v = float(self.get_parameter("max_linear_vel").value)
        jump_thr = float(self.get_parameter("ekf_jump_threshold_m").value)
        recommended = max_v / self.CONTROL_HZ + 0.03
        if recommended > jump_thr:
            self.get_logger().warn(
                f"ekf_jump_threshold_m={jump_thr:.3f} m is too tight for "
                f"max_linear_vel={max_v:.2f} m/s at {self.CONTROL_HZ} Hz. "
                f"Recommend ekf_jump_threshold_m >= {recommended:.3f} m, "
                f"or you'll see false-positive JUMP_SKIPs during normal motion."
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
                        arc_pts: int) -> list[tuple[float, float]]:
        """Replace each interior vertex with an inscribed circular arc.

        Bounds path curvature at κ_max = 1/radius.

        For each interior vertex P with neighbours A (before) and B (after),
        compute the inscribed arc tangent to AP and PB at distance d from P.
        d = radius / tan(theta/2), where theta is the interior angle.
        Vertices where d > 0.45 * min(|AP|, |PB|) are skipped (segments too
        short to support the arc) and a warning is logged.
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
            if theta < 1e-3 or math.pi - theta < 1e-3:
                # Nearly collinear; no smoothing needed, no chord taken
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

    # ==================================================================
    # Main control loop (50 Hz)
    # ==================================================================
    def _control_loop(self):
        """Compute and publish NED velocity vector."""
        # ---- Read parameters (allows runtime tuning) ----
        max_v       = self.get_parameter("max_linear_vel").value
        min_v       = self.get_parameter("min_linear_vel").value
        l_min       = self.get_parameter("min_lookahead_dist").value
        l_max       = self.get_parameter("max_lookahead_dist").value
        ld_gain     = self.get_parameter("lookahead_time").value
        min_radius  = self.get_parameter("regulated_linear_scaling_min_radius").value
        min_curv_v  = self.get_parameter("regulated_linear_scaling_min_speed").value
        goal_tol    = self.get_parameter("xy_goal_tolerance").value
        approach_d  = self.get_parameter("approach_velocity_scaling_dist").value
        approach_v  = self.get_parameter("min_approach_linear_velocity").value
        p4_floor    = self.get_parameter("p4_zero_vel_threshold").value
        max_age_s   = self.get_parameter("pose_max_age_s").value
        jump_thr    = self.get_parameter("ekf_jump_threshold_m").value   # P0.2
        req_rtk     = self.get_parameter("require_rtk_fix").value        # P0.3
        n_preview   = int(self.get_parameter("preview_curvature_n").value)  # P1.1
        xt_ld_gain  = self.get_parameter("xtrack_lookahead_gain").value  # P1.2

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

        # ---- Path readiness check ----
        if not self._path:
            self._publish_zero(StateCode.IDLE, pose_age_ms=pose_age_s * 1000)
            return

        # ---- Already done? Keep heartbeat at zero. ----
        if self._path_done:
            self._publish_zero(StateCode.DONE, pose_age_ms=pose_age_s * 1000)
            return

        # ---- Pose in NED ----
        pos_n, pos_e, _yaw = self._enu_pose_to_ned(pose_for_projection)
        # Note: yaw_ned is computed but NOT used in the velocity-vector design.
        # PX4 derives target yaw from atan2(vE, vN) in DifferentialOffboardMode.

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
        self._last_pos = (pos_n, pos_e)

        # ---- Goal check ----
        final = self._path[-1].pose.position
        dist_to_goal = self._dist(pos_n, pos_e, final.x, final.y)
        if dist_to_goal <= goal_tol:
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
        v_for_ld = max(min_v, self._last_speed_cmd if self._last_speed_cmd > 0.0
                       else max_v * 0.5)
        l_d_raw = ld_gain * v_for_ld + xt_ld_gain * abs(signed_xtrack)
        l_d = self._clamp(l_d_raw, l_min, l_max)

        # ---- Step 3: Lookahead point (NED), then body-frame for κ ----
        lh_n, lh_e, hit_end = self._get_lookahead_point(seg_idx, foot_n, foot_e, l_d)

        # Body-frame y-component for curvature math (need yaw here)
        _, _, yaw_ned = self._enu_pose_to_ned(pose_for_projection)
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
            # Lookahead landed on top of us; just hold position
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
            kappa_speed = max(abs(kappa),
                              self._max_preview_curvature(seg_idx, foot_n, foot_e,
                                                          l_d, n_preview))
        else:
            kappa_speed = abs(kappa)

        if kappa_speed > 1e-9:
            radius = 1.0 / kappa_speed
            speed_scale = self._clamp(radius / min_radius, 0.0, 1.0)
            speed = max(min_curv_v, max_v * speed_scale)
        else:
            radius = float("inf")
            speed = max_v

        # P3.1 — Feedforward yaw rate (body-rate mode)
        # ω_ff = κ·v (feedforward from path curvature and speed)
        # Plus small heading-error feedback to prevent drift.
        use_ff_yaw_rate = self.get_parameter("use_feedforward_yaw_rate").value
        if use_ff_yaw_rate:
            yaw_rate_ff = kappa * speed  # feedforward: κ·v
            yaw_rate_fb = self.get_parameter("yaw_rate_feedback_gain").value * theta_e
            yaw_rate_body = yaw_rate_ff + yaw_rate_fb
        else:
            yaw_rate_body = 0.0

        # ---- Step 6: Approach scaling near goal ----
        state_code = StateCode.TRACKING
        if dist_to_goal < approach_d:
            # Linearly scale speed from full → approach_v as dist → 0
            scale = self._clamp(dist_to_goal / approach_d, 0.0, 1.0)
            approach_speed = max(approach_v, speed * scale)
            speed = min(speed, approach_speed)
            state_code = StateCode.APPROACH

        # ---- Step 7: P4 floor — exact zero below threshold for clean stop ----
        if speed < p4_floor:
            speed = 0.0

        # ---- P0.1: persist commanded speed for next cycle's L_d ----
        self._last_speed_cmd = speed

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
        self._publish_yaw(yaw_target_ned)  # P0.5
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
            l_d_raw=l_d_raw,           # B1
            kappa_speed=kappa_speed,   # B1
        )

        self.get_logger().debug(
            f"[{state_code.name}] xtrack={signed_xtrack * 100:+.2f}cm "
            f"ld={l_actual:.2f}m(req={l_d:.2f}) κ={kappa:+.3f} κ_pred={kappa_speed:.3f} "
            f"R={radius if radius != float('inf') else -1:.2f}m "
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

    # P0.5: publish explicit yaw setpoint (NED, radians)
    def _publish_yaw(self, yaw_ned: float):
        msg = Float32()
        msg.data = float(yaw_ned)
        self._yaw_pub.publish(msg)

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
        # P0.5: freeze yaw at last commanded heading (do NOT snap to North).
        self._publish_yaw(self._last_yaw_cmd)
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
    ):
        """Publish /rpp/debug Float32MultiArray.

        Layout is append-only — indices [0..7] retained verbatim from the
        original 8-field schema so existing consumers (xtrack_logger,
        mission_runner, server/ros_node, server/main, server/rpp_status)
        continue to work without changes. New B1 fields are at [8..9].
        """
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label="rpp_debug",
                                                  size=10, stride=10))
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
