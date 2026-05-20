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
- Computes a Regulated Pure Pursuit (RPP) lookahead point on the path.
- Outputs a NED velocity *vector* (not body-frame Twist).

What this node does NOT do
--------------------------
- Does NOT compute angular velocity ω. PX4 v1.16+ ignores yawspeed in the
  OFFBOARD velocity branch. PX4 derives target yaw from atan2(vE, vN) of the
  velocity vector and runs its own heading PID + spot-turn FSM
  (RD_TRANS_DRV_TRN / RD_TRANS_TRN_DRV).
- Does NOT implement rotate-to-heading. PX4's spot-turn FSM does this
  automatically; tune RD_TRANS_DRV_TRN (≈30°) and RD_TRANS_TRN_DRV (≈5°).
- Does NOT do body→NED rotation. Output is already in NED.

Output contract
---------------
Topic:  /rpp/velocity_ned   (geometry_msgs/Vector3Stamped)
        header.stamp     = now
        header.frame_id  = "local_ned"
        vector.x         = v_north  (m/s, NED North)
        vector.y         = v_east   (m/s, NED East)
        vector.z         = 0.0

When the path is complete, the velocity vector is exactly (0, 0, 0). PX4's
P4 patch detects |v| < 1 cm/s and freezes heading instead of snapping to North.

When pose is stale or missing, an emergency-stop (0, 0, 0) is published at
50 Hz so OFFBOARD does not drop (COM_OF_LOSS_T = 500 ms).

Diagnostics
-----------
Topic:  /rpp/debug   (std_msgs/Float32MultiArray, layout encoded below)
        [0]  cross_track_error_m  (signed: + = right of path)
        [1]  heading_error_rad    (to lookahead, body frame)
        [2]  lookahead_dist_m
        [3]  speed_cmd_m_s
        [4]  curvature_kappa
        [5]  dist_to_goal_m
        [6]  pose_age_ms
        [7]  state_code           (0=idle, 1=tracking, 2=approach, 3=done, -1=stale)

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
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


# ---------------------------------------------------------------------------
# Diagnostic state codes (published in /rpp/debug index 7)
# ---------------------------------------------------------------------------
class StateCode(IntEnum):
    STALE = -1     # pose is stale; emergency stop
    IDLE = 0       # no path or no pose yet
    TRACKING = 1   # normal RPP tracking
    APPROACH = 2   # within approach_dist of goal; speed scaling active
    DONE = 3       # within goal_tolerance; outputting zero velocity


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

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._path: list[PoseStamped] = []
        self._pose: PoseStamped | None = None
        self._pose_recv_time: RclTime | None = None
        self._path_done = False
        self._closest_seg_hint = 0   # for monotonic search optimisation

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

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(Path, "/path", self._path_cb, path_qos)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, be_qos
        )

        # ------------------------------------------------------------------
        # 50 Hz control timer
        # ------------------------------------------------------------------
        self._timer = self.create_timer(1.0 / self.CONTROL_HZ, self._control_loop)

        self.get_logger().info(
            "RPP controller started — output: /rpp/velocity_ned (NED frame, "
            "Vector3Stamped). Waiting for /path and /mavros/local_position/pose."
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

        self._path = list(msg.poses)
        self._path_done = False
        self._closest_seg_hint = 0

        first = self._path[0].pose.position
        last = self._path[-1].pose.position
        self.get_logger().info(
            f"Path accepted: {len(self._path)} waypoints, "
            f"first=({first.x:.2f}N, {first.y:.2f}E), "
            f"last=({last.x:.2f}N, {last.y:.2f}E)"
        )

    def _pose_cb(self, msg: PoseStamped):
        """Store latest pose. Frame conversion happens at use-site."""
        self._pose = msg
        self._pose_recv_time = self.get_clock().now()

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

    def _project_onto_path(self, pos_n: float, pos_e: float):
        """Find closest point on the path as a *segment projection*.

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

        best = (0, 0.0, self._path[0].pose.position.x, self._path[0].pose.position.y, float("inf"), 0.0)
        # best = (seg_idx, t, foot_n, foot_e, dist, signed_e)

        for i in range(n_pts - 1):
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

        # ---- Pose freshness check ----
        if self._pose is None or self._pose_recv_time is None:
            self._publish_zero(StateCode.IDLE, pose_age_ms=float("nan"))
            return

        pose_age_s = (self.get_clock().now() - self._pose_recv_time).nanoseconds * 1e-9
        if pose_age_s > max_age_s:
            self.get_logger().warn(
                f"Stale pose ({pose_age_s * 1000:.0f} ms > "
                f"{max_age_s * 1000:.0f} ms) — emergency stop",
                throttle_duration_sec=1.0,
            )
            self._publish_zero(StateCode.STALE, pose_age_ms=pose_age_s * 1000)
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
        pos_n, pos_e, _yaw = self._enu_pose_to_ned(self._pose)
        # Note: yaw_ned is computed but NOT used in the velocity-vector design.
        # PX4 derives target yaw from atan2(vE, vN) in DifferentialOffboardMode.

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
        seg_idx, t, foot_n, foot_e, signed_xtrack = self._project_onto_path(
            pos_n, pos_e
        )

        # ---- Step 2: Velocity-scaled lookahead ----
        # Use the just-computed speed estimate for stability. We start at
        # min_v floor since the last published v_current isn't tracked here
        # (kept simple - the lookahead converges in a few cycles).
        v_for_ld = max(min_v, max_v * 0.5)  # conservative initial estimate
        l_d = self._clamp(v_for_ld * ld_gain, l_min, l_max)

        # ---- Step 3: Lookahead point (NED), then body-frame for κ ----
        lh_n, lh_e, hit_end = self._get_lookahead_point(seg_idx, foot_n, foot_e, l_d)

        # Body-frame y-component for curvature math (need yaw here)
        _, _, yaw_ned = self._enu_pose_to_ned(self._pose)
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

        # ---- Step 5: Curvature-regulated speed ----
        if abs(kappa) > 1e-9:
            radius = abs(1.0 / kappa)
            speed_scale = self._clamp(radius / min_radius, 0.0, 1.0)
            speed = max(min_curv_v, max_v * speed_scale)
        else:
            radius = float("inf")
            speed = max_v

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

        # ---- Step 8: Build NED velocity vector ----
        # Direction: unit vector from rover to lookahead point, in NED.
        # PX4 computes target_yaw = atan2(vE, vN) and aligns the rover with
        # that direction via its internal heading PID + spot-turn FSM.
        unit_n = dn / l_actual if l_actual > 1e-9 else 0.0
        unit_e = de / l_actual if l_actual > 1e-9 else 0.0
        v_n = speed * unit_n
        v_e = speed * unit_e

        # ---- Publish ----
        self._publish_velocity(v_n, v_e)

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
        )

        self.get_logger().debug(
            f"[{state_code.name}] xtrack={signed_xtrack * 100:+.2f}cm "
            f"ld={l_actual:.2f}m κ={kappa:+.3f} R={radius if radius != float('inf') else -1:.2f}m "
            f"v=({v_n:+.3f},{v_e:+.3f})m/s speed={speed:.3f} "
            f"θe={math.degrees(theta_e):+.1f}° dgoal={dist_to_goal * 100:.1f}cm "
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

    def _publish_zero(
        self,
        state: StateCode,
        pose_age_ms: float = float("nan"),
        dist_to_goal: float = float("nan"),
    ):
        """Publish (0, 0, 0) and a diagnostic. Used for IDLE/DONE/STALE."""
        self._publish_velocity(0.0, 0.0)
        self._publish_debug(
            cross_track=float("nan"),
            heading_err=float("nan"),
            lookahead=float("nan"),
            speed=0.0,
            kappa=float("nan"),
            dist_goal=dist_to_goal,
            pose_age_ms=pose_age_ms,
            state=state,
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
    ):
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label="rpp_debug", size=8, stride=8))
        msg.data = [
            float(cross_track),
            float(heading_err),
            float(lookahead),
            float(speed),
            float(kappa),
            float(dist_goal),
            float(pose_age_ms),
            float(state.value),
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
