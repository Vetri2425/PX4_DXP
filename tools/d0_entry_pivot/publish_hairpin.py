#!/usr/bin/env python3
"""D0 hairpin path publisher — dependency-free, spray OFF, exact RPP /path contract.

Why this instead of path_publisher_node.py: that node routes every .csv/.dxf
through path_engine (absent on the Jetson) and defaults spray ON. For the D0
bench we want a plain NED hairpin, spray OFF, no planner.

Publishes a 3-point hairpin on /path anchored at the rover's LIVE pose:
    A = live pose
    B = A + 2.0 m north      (drive out, then brake to a dead stop)
    C = A + 0.4 m east        (~169 deg hairpin exit — the make-or-break turn)

RPP /path contract (verified against rpp_controller_node._path_cb):
    position.x = North, position.y = East, position.z = spray flag (0 = OFF),
    frame_id = "local_ned", QoS = RELIABLE + TRANSIENT_LOCAL (latched).
ENU pose -> NED origin: n = pose.y, e = pose.x.

Causes NO motion by itself. RPP only drives when the FCU is ARMED *and* in
OFFBOARD. Leave this running (it holds the latched /path); Ctrl-C to clear.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# Hairpin offsets in NED metres (d_north, d_east) — matches hairpin.csv
OFFSETS = [(0.0, 0.0), (2.0, 0.0), (0.0, 0.4)]


class HairpinPublisher(Node):
    def __init__(self):
        super().__init__("d0_hairpin_publisher")
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
        self.get_logger().info(
            "d0_hairpin_publisher: waiting for /mavros/local_position/pose to anchor origin…"
        )

    def _nag(self):
        if not self._published:
            self._warned += 1
            self.get_logger().warn(
                f"still no pose after {self._warned * 2}s — is MAVROS up and local_position valid?"
            )

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
            ps.pose.position.z = 0.0             # spray OFF (transit test)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._pub.publish(path)
        self._published = True
        a, b, c = (p.pose.position for p in path.poses)
        self.get_logger().info(
            "Published hairpin on /path (spray OFF, latched, held): "
            f"A=({a.x:.3f}N,{a.y:.3f}E) B=({b.x:.3f}N,{b.y:.3f}E) C=({c.x:.3f}N,{c.y:.3f}E). "
            "NO motion until ARMED + OFFBOARD. Ctrl-C to clear /path."
        )


def main():
    rclpy.init()
    node = HairpinPublisher()
    try:
        rclpy.spin(node)   # stay alive to serve the latched /path
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
