#!/usr/bin/env python3
"""Straight 2 m line publisher — D3 completion-stop field test.

Isolates the final-waypoint stop (no corner): A = live pose, B = A + 2 m north,
spray OFF. Drives to B and must STOP ON B — the D3 fix. Before D3 the rover
coasted past B on a bare zero setpoint (bag 2026-07-10 Line_2m, 1.08 m run-past).

Same RPP /path contract + latched QoS as publish_hairpin.py. Causes NO motion
until ARMED + OFFBOARD. Ctrl-C to clear /path.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# NED offsets (d_north, d_east): straight 2 m north.
OFFSETS = [(0.0, 0.0), (2.0, 0.0)]


class LinePublisher(Node):
    def __init__(self):
        super().__init__("d3_line_publisher")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        pose_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(Path, "/path", latched)
        self._published = False
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, pose_qos
        )
        self._warned = 0
        self.create_timer(2.0, self._nag)
        self.get_logger().info("d3_line_publisher: waiting for pose to anchor origin…")

    def _nag(self):
        if not self._published:
            self._warned += 1
            self.get_logger().warn(f"still no pose after {self._warned * 2}s")

    def _pose_cb(self, msg: PoseStamped):
        if self._published:
            return
        origin_n = float(msg.pose.position.y)   # ENU y = North
        origin_e = float(msg.pose.position.x)   # ENU x = East
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "local_ned"
        for dn, de in OFFSETS:
            ps = PoseStamped()
            ps.header.stamp = path.header.stamp
            ps.header.frame_id = "local_ned"
            ps.pose.position.x = origin_n + dn   # North
            ps.pose.position.y = origin_e + de   # East
            ps.pose.position.z = 0.0             # spray OFF
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._pub.publish(path)
        self._published = True
        a, b = (p.pose.position for p in path.poses)
        self.get_logger().info(
            "Published straight 2 m line on /path (spray OFF, latched): "
            f"A=({a.x:.3f}N,{a.y:.3f}E) B=({b.x:.3f}N,{b.y:.3f}E). "
            "Must STOP on B. NO motion until ARMED + OFFBOARD."
        )


def main():
    rclpy.init()
    node = LinePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
