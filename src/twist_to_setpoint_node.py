#!/usr/bin/env python3
"""NED velocity vector → MAVROS PositionTarget streamer.

Pipeline position:
  rpp_controller_node → /rpp/velocity_ned → [THIS NODE] → /mavros/setpoint_raw/local → MAVROS → PX4

Why this node exists separately from rpp_controller_node
--------------------------------------------------------
Separation of concerns:
  - RPP node owns *path geometry* (lookahead, curvature, speed regulation).
  - This node owns the *PX4 OFFBOARD heartbeat contract* (50 Hz, COM_OF_LOSS_T,
    type_mask, frame, fail-safe zero-velocity on input loss).

If RPP crashes, this node keeps the OFFBOARD heartbeat alive with zero velocity
so PX4 doesn't drop OFFBOARD and trigger failsafe RTL.

Output contract
---------------
  Topic:  /mavros/setpoint_raw/local   (mavros_msgs/PositionTarget)
  Rate:   50 Hz, continuous (never gaps; PX4 drops OFFBOARD after 500 ms gap)
  Frame:  FRAME_LOCAL_NED (1)
  Mask:   3527 (velocity-only, ignore positions, accelerations, yaw, yaw_rate)
          PX4 v1.16+ DifferentialOffboardMode derives target yaw from
          atan2(vE, vN) of the velocity vector — yawspeed is ignored anyway.

Frame discipline
----------------
Input is *already* in NED (Vector3Stamped from rpp_controller_node, header
frame_id="local_ned"). No body→NED rotation needed in this node.

Stale-input behaviour
---------------------
  - Before first velocity received: stream (0,0,0) so OFFBOARD can be entered
    cleanly. PX4 P4 patch detects |v| < 1cm/s and freezes heading.
  - After first velocity received but stale > input_max_age_s: stream (0,0,0)
    and warn at 1 Hz. Rover holds position, OFFBOARD stays live.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import PositionTarget


# ---------------------------------------------------------------------------
# PositionTarget type_mask constants (MAVLink SET_POSITION_TARGET_LOCAL_NED)
# ---------------------------------------------------------------------------
FRAME_LOCAL_NED = 1

IGNORE_PX = 1
IGNORE_PY = 2
IGNORE_PZ = 4
IGNORE_VX = 8
IGNORE_VY = 16
IGNORE_VZ = 32
IGNORE_AFX = 64
IGNORE_AFY = 128
IGNORE_AFZ = 256
IGNORE_YAW = 1024
IGNORE_YAW_RATE = 2048

# Velocity-only: send vN, vE, vD; ignore everything else.
# PX4 OFFBOARD velocity branch derives yaw from atan2(vE, vN) regardless of
# the IGNORE_YAW bit, so this mask gives us the full velocity-driven path
# follower behaviour without having to manage yaw on the Jetson side.
TYPE_MASK_VELOCITY = (
    IGNORE_PX | IGNORE_PY | IGNORE_PZ
    | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
    | IGNORE_YAW | IGNORE_YAW_RATE
)  # = 3527


class TwistToSetpointNode(Node):
    """Bridges /rpp/velocity_ned to /mavros/setpoint_raw/local at 50 Hz."""

    STREAM_HZ = 50

    def __init__(self):
        super().__init__("twist_to_setpoint")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("input_max_age_s", 0.2)   # 200 ms input staleness
        self.declare_parameter("expected_input_frame", "local_ned")

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._latest_vel: Vector3Stamped | None = None
        self._latest_recv_time = None
        self._published_count = 0
        self._stale_warn_count = 0

        # ------------------------------------------------------------------
        # QoS — match offboard_test.py for compatibility with PX4 setpoint loop
        # ------------------------------------------------------------------
        be_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ------------------------------------------------------------------
        # Publishers / Subscribers
        # ------------------------------------------------------------------
        self._sp_pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", be_qos
        )
        self.create_subscription(
            Vector3Stamped, "/rpp/velocity_ned", self._vel_cb, be_qos
        )

        # ------------------------------------------------------------------
        # 50 Hz stream timer
        # ------------------------------------------------------------------
        self._timer = self.create_timer(1.0 / self.STREAM_HZ, self._stream_cb)

        self.get_logger().info(
            f"twist_to_setpoint started — streaming /mavros/setpoint_raw/local "
            f"at {self.STREAM_HZ} Hz (type_mask={TYPE_MASK_VELOCITY}, "
            f"frame=LOCAL_NED). Source: /rpp/velocity_ned."
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _vel_cb(self, msg: Vector3Stamped):
        expected = self.get_parameter("expected_input_frame").value
        if msg.header.frame_id and msg.header.frame_id != expected:
            self.get_logger().warn(
                f"Velocity frame_id {msg.header.frame_id!r} != expected {expected!r}; "
                f"using anyway but check rpp_controller_node configuration",
                throttle_duration_sec=5.0,
            )

        # Sanity checks — reject NaN/Inf
        if not (math.isfinite(msg.vector.x) and math.isfinite(msg.vector.y)
                and math.isfinite(msg.vector.z)):
            self.get_logger().warn(
                f"Non-finite velocity received "
                f"({msg.vector.x}, {msg.vector.y}, {msg.vector.z}) — ignoring",
                throttle_duration_sec=1.0,
            )
            return

        self._latest_vel = msg
        self._latest_recv_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # 50 Hz stream
    # ------------------------------------------------------------------
    def _stream_cb(self):
        max_age = self.get_parameter("input_max_age_s").value
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""  # PX4 ignores; coordinate_frame is what matters
        msg.coordinate_frame = FRAME_LOCAL_NED
        msg.type_mask = TYPE_MASK_VELOCITY

        # Default: zero velocity (safe fail-stop)
        v_n = 0.0
        v_e = 0.0
        v_d = 0.0
        source = "zero"

        if self._latest_vel is not None and self._latest_recv_time is not None:
            age_s = (self.get_clock().now() - self._latest_recv_time).nanoseconds * 1e-9
            if age_s <= max_age:
                v_n = float(self._latest_vel.vector.x)
                v_e = float(self._latest_vel.vector.y)
                v_d = float(self._latest_vel.vector.z)
                source = "rpp"
            else:
                source = "stale"
                self._stale_warn_count += 1
                # Warn at most once per second
                if self._stale_warn_count % self.STREAM_HZ == 0:
                    self.get_logger().warn(
                        f"Input stale ({age_s * 1000:.0f} ms > "
                        f"{max_age * 1000:.0f} ms) — streaming zero velocity"
                    )

        msg.velocity.x = v_n
        msg.velocity.y = v_e
        msg.velocity.z = v_d

        # Position, acceleration, yaw, yaw_rate are ignored by mask but set
        # to safe values to avoid uninitialised-memory paranoia.
        msg.position.x = 0.0
        msg.position.y = 0.0
        msg.position.z = 0.0
        msg.acceleration_or_force.x = 0.0
        msg.acceleration_or_force.y = 0.0
        msg.acceleration_or_force.z = 0.0
        msg.yaw = 0.0
        msg.yaw_rate = 0.0

        self._sp_pub.publish(msg)
        self._published_count += 1

        # Heartbeat log every 5 seconds
        if self._published_count % (self.STREAM_HZ * 5) == 0:
            self.get_logger().debug(
                f"streaming [{source}] v=({v_n:+.3f},{v_e:+.3f},{v_d:+.3f}) m/s "
                f"published={self._published_count}"
            )


def main():
    rclpy.init()
    node = None
    try:
        node = TwistToSetpointNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
