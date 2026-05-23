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
frame_id="local_ned"). Output to MAVROS must be in ENU (REP-103):
x=East, y=North, z=Up. We swap N↔E and negate z on output.

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
from std_msgs.msg import Float32


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

# P0.5 — Velocity + explicit yaw: send vN, vE, vD, yaw; ignore everything else.
# This gives RPP authority over heading instead of relying on PX4's
# atan2(vE, vN) derivation. Useful for P3.1 (feedforward ω) and smoother
# corner transitions.
TYPE_MASK_VELOCITY_AND_YAW = (
    IGNORE_PX | IGNORE_PY | IGNORE_PZ
    | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
    | IGNORE_YAW_RATE
)  # = 1535 (yaw is NOT ignored)


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
        # P0.5 — enable explicit yaw_setpoint in PositionTarget
        self.declare_parameter("use_explicit_yaw", False)  # default: velocity-only (backward compat)
        self.declare_parameter("yaw_slew_rate_rad_s", 1.57)  # 90 deg/s default

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._latest_vel: Vector3Stamped | None = None
        self._latest_yaw: float | None = None  # P0.5: track latest yaw setpoint
        self._latest_recv_time = None
        self._last_yaw_cmd: float = 0.0  # P0.5: track last published yaw for slew limiting
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
        # P0.5: optional yaw setpoint from RPP node (when use_explicit_yaw=true)
        self.create_subscription(
            Float32, "/rpp/yaw_setpoint_ned", self._yaw_cb, be_qos
        )

        # ------------------------------------------------------------------
        # 50 Hz stream timer
        # ------------------------------------------------------------------
        self._timer = self.create_timer(1.0 / self.STREAM_HZ, self._stream_cb)

        self.get_logger().info(
            f"twist_to_setpoint started — streaming /mavros/setpoint_raw/local "
            f"at {self.STREAM_HZ} Hz (frame=LOCAL_NED). Source: /rpp/velocity_ned. "
            f"P0.5: use_explicit_yaw={self.get_parameter('use_explicit_yaw').value} "
            f"(type_mask={TYPE_MASK_VELOCITY_AND_YAW if self.get_parameter('use_explicit_yaw').value else TYPE_MASK_VELOCITY})."
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

    # P0.5: yaw setpoint callback (optional, only used if use_explicit_yaw=true)
    def _yaw_cb(self, msg: Float32):
        """Track latest yaw setpoint from RPP node (NED, radians)."""
        if math.isfinite(msg.data):
            self._latest_yaw = float(msg.data)
        else:
            self.get_logger().warn(
                f"Non-finite yaw received ({msg.data}) — ignoring",
                throttle_duration_sec=1.0,
            )

    # ------------------------------------------------------------------
    # 50 Hz stream
    # ------------------------------------------------------------------
    def _stream_cb(self):
        max_age = self.get_parameter("input_max_age_s").value
        use_explicit_yaw = self.get_parameter("use_explicit_yaw").value
        yaw_slew = self.get_parameter("yaw_slew_rate_rad_s").value
        
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""  # PX4 ignores; coordinate_frame is what matters
        msg.coordinate_frame = FRAME_LOCAL_NED
        msg.type_mask = TYPE_MASK_VELOCITY_AND_YAW if use_explicit_yaw else TYPE_MASK_VELOCITY

        # Default: zero velocity (safe fail-stop)
        v_n = 0.0
        v_e = 0.0
        v_d = 0.0
        yaw_ned = 0.0
        source = "zero"

        if self._latest_vel is not None and self._latest_recv_time is not None:
            age_s = (self.get_clock().now() - self._latest_recv_time).nanoseconds * 1e-9
            if age_s <= max_age:
                v_n = float(self._latest_vel.vector.x)
                v_e = float(self._latest_vel.vector.y)
                v_d = float(self._latest_vel.vector.z)
                source = "rpp"
                # P0.5: if explicit yaw is enabled, use the latest yaw setpoint
                # (with slew limiting to prevent sharp heading snaps).
                if use_explicit_yaw and self._latest_yaw is not None:
                    yaw_ned = self._slew_yaw(self._last_yaw_cmd, self._latest_yaw, yaw_slew)
                else:
                    yaw_ned = 0.0
            else:
                source = "stale"
                self._stale_warn_count += 1
                # Warn at most once per second
                if self._stale_warn_count % self.STREAM_HZ == 0:
                    self.get_logger().warn(
                        f"Input stale ({age_s * 1000:.0f} ms > "
                        f"{max_age * 1000:.0f} ms) — streaming zero velocity"
                    )

        # MAVROS PositionTarget uses ENU convention (REP-103):
        #   x = East, y = North, z = Up
        # Our RPP controller outputs NED: v_n = North, v_e = East.
        # Swap N↔E and negate z to convert NED → ENU.
        msg.velocity.x = v_e       # ENU x = East  (was NED y)
        msg.velocity.y = v_n       # ENU y = North (was NED x)
        msg.velocity.z = -v_d      # ENU z = Up    (negate NED Down)
        msg.yaw = yaw_ned

        # Position, acceleration, yaw_rate are ignored by mask but set
        # to safe values to avoid uninitialised-memory paranoia.
        msg.position.x = 0.0
        msg.position.y = 0.0
        msg.position.z = 0.0
        msg.acceleration_or_force.x = 0.0
        msg.acceleration_or_force.y = 0.0
        msg.acceleration_or_force.z = 0.0
        msg.yaw_rate = 0.0

        self._sp_pub.publish(msg)
        self._last_yaw_cmd = yaw_ned  # P0.5: track for next cycle's slew limiting
        self._published_count += 1

        # Heartbeat log every 5 seconds
        if self._published_count % (self.STREAM_HZ * 5) == 0:
            yaw_str = f"yaw={yaw_ned:.3f}rad" if use_explicit_yaw else "yaw=auto"
            self.get_logger().debug(
                f"streaming [{source}] v=({v_n:+.3f},{v_e:+.3f},{v_d:+.3f}) m/s "
                f"{yaw_str} published={self._published_count}"
            )

    @staticmethod
    def _slew_yaw(current_yaw: float, target_yaw: float, slew_rate: float) -> float:
        """Apply slew limiting to yaw setpoint to prevent sharp heading snaps.
        
        Wraps the error to [-π, π] and limits the rate of change.
        dt = 1/50 Hz = 0.02 s.
        """
        dt = 1.0 / 50.0  # 50 Hz stream rate
        max_delta = slew_rate * dt
        
        # Wrap error to [-π, π]
        error = target_yaw - current_yaw
        error = (error + math.pi) % (2 * math.pi) - math.pi
        
        # Clamp to max_delta
        delta = max(-max_delta, min(max_delta, error))
        return current_yaw + delta


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
