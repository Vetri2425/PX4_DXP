#!/usr/bin/env python3
"""Cross-track error logger — captures RPP performance to CSV for offline analysis.

Pipeline position:
  /path                ──┐
  /mavros/local_position/pose ──┤
  /rpp/debug                  ──┼──> [THIS NODE] → /tmp/rpp_<path_name>_<ts>.csv
  /rpp/velocity_ned           ──┤
  /mavros/setpoint_raw/local  ──┘

Why a logger instead of just rosbag
-----------------------------------
rosbag captures everything (great for post-mortem) but isn't structured for
spreadsheet analysis. This node emits a single time-aligned CSV with all the
key tuning signals on one row per timestamp, ready to drop into pandas / Excel.

Use rosbag in addition to this for full-fidelity capture.

CSV columns
-----------
  t_s                  Time since node start (seconds)
  pose_n               Rover NED north (m)
  pose_e               Rover NED east  (m)
  pose_yaw_ned_deg     Rover NED yaw   (degrees)
  pose_age_ms          Pose freshness from /rpp/debug
  closest_n            Closest point on path, NED north (m)
  closest_e            Closest point on path, NED east  (m)
  xtrack_signed_cm     Signed cross-track (+ = right of path)  [from /rpp/debug]
  heading_err_deg      Heading error to lookahead             [from /rpp/debug]
  lookahead_m          Velocity-scaled lookahead              [from /rpp/debug]
  speed_cmd_m_s        Commanded speed                        [from /rpp/debug]
  curvature_kappa      Path curvature                         [from /rpp/debug]
  dist_to_goal_m       Distance to final waypoint             [from /rpp/debug]
  rpp_state            -1=stale, 0=idle, 1=tracking, 2=approach, 3=done
  v_ned_n_m_s          Velocity setpoint North (from /rpp/velocity_ned)
  v_ned_e_m_s          Velocity setpoint East
  mavros_v_n_m_s       Final MAVROS setpoint vN (from /mavros/setpoint_raw/local)
  mavros_v_e_m_s       Final MAVROS setpoint vE

Output path
-----------
  /tmp/rpp_<path_name_or_unknown>_<YYYYMMDD_HHMMSS>.csv
  Override with `output_path` parameter.
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray


class XTrackLoggerNode(Node):
    """Time-aligns RPP signals and writes them to a CSV row at LOG_HZ."""

    LOG_HZ = 20  # 50 ms per row — coarser than control loop, plenty for analysis

    def __init__(self):
        super().__init__("xtrack_logger")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("output_path", "")  # empty → auto-generate
        self.declare_parameter("path_name_hint", "unknown")

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._path: Path | None = None
        self._pose: PoseStamped | None = None
        self._debug: Float32MultiArray | None = None
        self._vel_ned: Vector3Stamped | None = None
        self._mavros_sp: PositionTarget | None = None

        self._t0 = self.get_clock().now()

        # ------------------------------------------------------------------
        # CSV file
        # ------------------------------------------------------------------
        out = self.get_parameter("output_path").value
        if not out:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            hint = self.get_parameter("path_name_hint").value
            out = f"/tmp/rpp_{hint}_{ts}.csv"

        # On Windows /tmp won't exist, use cwd
        if out.startswith("/tmp/") and os.name == "nt":
            out = os.path.join(os.getcwd(), os.path.basename(out))

        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)

        self._csv_path = out
        self._csv_file = open(out, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow([
            "t_s",
            "pose_n", "pose_e", "pose_yaw_ned_deg", "pose_age_ms",
            "closest_n", "closest_e",
            "xtrack_signed_cm", "heading_err_deg",
            "lookahead_m", "speed_cmd_m_s", "curvature_kappa",
            "dist_to_goal_m", "rpp_state",
            "v_ned_n_m_s", "v_ned_e_m_s",
            "mavros_v_n_m_s", "mavros_v_e_m_s",
        ])
        self._csv_file.flush()

        self.get_logger().info(f"xtrack_logger writing to {out}")

        # ------------------------------------------------------------------
        # QoS
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
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(Path, "/path", lambda m: setattr(self, "_path", m), path_qos)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            lambda m: setattr(self, "_pose", m), be_qos
        )
        self.create_subscription(
            Float32MultiArray, "/rpp/debug",
            lambda m: setattr(self, "_debug", m), be_qos
        )
        self.create_subscription(
            Vector3Stamped, "/rpp/velocity_ned",
            lambda m: setattr(self, "_vel_ned", m), be_qos
        )
        self.create_subscription(
            PositionTarget, "/mavros/setpoint_raw/local",
            lambda m: setattr(self, "_mavros_sp", m), be_qos
        )

        # ------------------------------------------------------------------
        # Logging timer
        # ------------------------------------------------------------------
        self._timer = self.create_timer(1.0 / self.LOG_HZ, self._log_row)

    # ==================================================================
    # Logging
    # ==================================================================
    def _log_row(self):
        if self._pose is None:
            return  # wait for at least pose

        now = self.get_clock().now()
        t_s = (now - self._t0).nanoseconds * 1e-9

        # --- Pose in NED ---
        # MAVROS local_position is ENU; swap axes for NED
        pose_n = self._pose.pose.position.y
        pose_e = self._pose.pose.position.x

        q = self._pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_enu = math.atan2(siny_cosp, cosy_cosp)
        yaw_ned = (math.pi / 2.0 - yaw_enu + math.pi) % (2 * math.pi) - math.pi

        # --- Closest point on path (segment projection) ---
        closest_n = closest_e = float("nan")
        if self._path and len(self._path.poses) >= 2:
            best_d = float("inf")
            for i in range(len(self._path.poses) - 1):
                ax = self._path.poses[i].pose.position.x
                ay = self._path.poses[i].pose.position.y
                bx = self._path.poses[i + 1].pose.position.x
                by = self._path.poses[i + 1].pose.position.y
                dx, dy = bx - ax, by - ay
                seg_sq = dx * dx + dy * dy
                if seg_sq < 1e-12:
                    continue
                t = max(0.0, min(1.0, ((pose_n - ax) * dx + (pose_e - ay) * dy) / seg_sq))
                fn = ax + t * dx
                fe = ay + t * dy
                d = math.hypot(pose_n - fn, pose_e - fe)
                if d < best_d:
                    best_d = d
                    closest_n, closest_e = fn, fe

        # --- /rpp/debug fields ---
        dbg = self._debug.data if self._debug and len(self._debug.data) >= 8 else [float("nan")] * 8
        xtrack_signed_cm = dbg[0] * 100.0 if math.isfinite(dbg[0]) else float("nan")
        heading_err_deg = math.degrees(dbg[1]) if math.isfinite(dbg[1]) else float("nan")
        lookahead = dbg[2]
        speed_cmd = dbg[3]
        kappa = dbg[4]
        dist_goal = dbg[5]
        pose_age_ms = dbg[6]
        rpp_state = int(dbg[7]) if math.isfinite(dbg[7]) else -99

        # --- /rpp/velocity_ned ---
        if self._vel_ned:
            v_ned_n = self._vel_ned.vector.x
            v_ned_e = self._vel_ned.vector.y
        else:
            v_ned_n = v_ned_e = float("nan")

        # --- /mavros/setpoint_raw/local ---
        if self._mavros_sp:
            mv_n = self._mavros_sp.velocity.x
            mv_e = self._mavros_sp.velocity.y
        else:
            mv_n = mv_e = float("nan")

        self._csv.writerow([
            f"{t_s:.3f}",
            f"{pose_n:.4f}", f"{pose_e:.4f}",
            f"{math.degrees(yaw_ned):.2f}", f"{pose_age_ms:.1f}",
            f"{closest_n:.4f}", f"{closest_e:.4f}",
            f"{xtrack_signed_cm:.3f}", f"{heading_err_deg:.2f}",
            f"{lookahead:.3f}", f"{speed_cmd:.3f}", f"{kappa:.4f}",
            f"{dist_goal:.4f}", rpp_state,
            f"{v_ned_n:.4f}", f"{v_ned_e:.4f}",
            f"{mv_n:.4f}", f"{mv_e:.4f}",
        ])

        # Flush every ~1s so partial logs survive a Ctrl-C
        if int(t_s * self.LOG_HZ) % self.LOG_HZ == 0:
            self._csv_file.flush()

    def destroy_node(self):
        try:
            self._csv_file.flush()
            self._csv_file.close()
            self.get_logger().info(f"xtrack CSV closed: {self._csv_path}")
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = XTrackLoggerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
