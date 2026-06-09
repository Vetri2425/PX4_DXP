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

Output contract
---------------
  Topic:  /mavros/setpoint_raw/local   (mavros_msgs/PositionTarget)
  Rate:   50 Hz, continuous (never gaps; PX4 drops OFFBOARD after 500 ms gap)
  Frame:  FRAME_LOCAL_NED (1)
  Mask:   455 (velocity + explicit yaw + yaw_rate feedforward; ignore positions, accelerations)
          Yaw is computed from velocity direction: yaw_ENU = atan2(v_n, v_e).
          yaw_rate = yaw_rate_body (LOCAL_NED pass-through, NED CW+) from /rpp/yaw_rate_body.
          Feedforward κ·v eliminates arc outside-drift structural bias caused by
          yaw controller phase lag on continuous curves.

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
)  # = 2503 (yaw is NOT ignored)

# P3.1 — Velocity + yaw + yaw_rate feedforward: send vN, vE, vD, yaw, yaw_rate.
# Adds continuous curvature feedforward (κ·v from RPP) so PX4's yaw controller
# tracks the arc tangent without phase lag → eliminates outside-drift structural bias.
# 455 = 2503 - 2048 = IGNORE_YAW_RATE removed from TYPE_MASK_VELOCITY_AND_YAW.
TYPE_MASK_VEL_YAW_YAWRATE = (
    IGNORE_PX | IGNORE_PY | IGNORE_PZ
    | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
)  # = 455 (velocity + yaw + yaw_rate all active)


class TwistToSetpointNode(Node):
    """Bridges /rpp/velocity_ned to /mavros/setpoint_raw/local at 50 Hz."""

    STREAM_HZ = 50

    def __init__(self):
        super().__init__("twist_to_setpoint")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("input_max_age_s", 0.2)   # 200 ms input staleness
        # T1 fix: dedicated, wider staleness window for yaw_rate feedforward.
        # Using input_max_age_s for yaw_rate caused a one-cycle FF dropout
        # (type_mask 455→2503 flip) when the RPP timer and this node's timer
        # drifted by one executor scheduling slot under CPU load. The yaw_rate
        # topic is published in the same RPP cycle as velocity, so if velocity
        # is fresh, a yaw_rate up to ~1.5× older is still the matching sample.
        self.declare_parameter("yaw_rate_max_age_s", 0.3)
        self.declare_parameter("expected_input_frame", "local_ned")
        # Explicit yaw computed from velocity direction (always on since 2026-05-23).
        # PX4 leaves trajectory_setpoint.yaw=NaN without explicit yaw, causing
        # yaw tracking lag on turns.

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._latest_vel: Vector3Stamped | None = None
        self._latest_recv_time = None
        self._latest_yaw_rate_body: float = 0.0   # NED CW+ rad/s from RPP
        self._yaw_rate_recv_time = None
        self._last_yaw_cmd: float = 0.0  # Track last yaw for zero-speed hold
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
        self.create_subscription(
            Float32, "/rpp/yaw_rate_body", self._yaw_rate_cb, be_qos
        )

        # ------------------------------------------------------------------
        # 50 Hz stream timer
        # ------------------------------------------------------------------
        self._timer = self.create_timer(1.0 / self.STREAM_HZ, self._stream_cb)

        self.get_logger().info(
            f"twist_to_setpoint started — streaming /mavros/setpoint_raw/local "
            f"at {self.STREAM_HZ} Hz (frame=LOCAL_NED). Sources: /rpp/velocity_ned + "
            f"/rpp/yaw_rate_body. Yaw+yaw_rate feedforward active "
            f"(type_mask={TYPE_MASK_VEL_YAW_YAWRATE})."
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

    def _yaw_rate_cb(self, msg: Float32):
        if math.isfinite(msg.data):
            self._latest_yaw_rate_body = msg.data
            self._yaw_rate_recv_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # 50 Hz stream
    # ------------------------------------------------------------------
    def _stream_cb(self):
        max_age = self.get_parameter("input_max_age_s").value

        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""  # PX4 ignores; coordinate_frame is what matters
        msg.coordinate_frame = FRAME_LOCAL_NED
        # type_mask set dynamically below based on whether yaw_rate FF is active

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

        # MAVROS PositionTarget uses ENU convention (REP-103):
        #   x = East, y = North, z = Up
        # Our RPP controller outputs NED: v_n = North, v_e = East.
        # Swap N↔E and negate z to convert NED → ENU.
        msg.velocity.x = v_e       # ENU x = East  (was NED y)
        msg.velocity.y = v_n       # ENU y = North (was NED x)
        msg.velocity.z = -v_d      # ENU z = Up    (negate NED Down)

        # Compute explicit yaw from velocity direction.
        # PX4's DifferentialOffboardMode leaves yaw=NaN in trajectory_setpoint
        # when IGNORE_YAW is set, and its internal atan2(vE,vN) derivation lags
        # on turns (yaw rate capped at 30°/s). Publishing an explicit yaw gives
        # the yaw controller a direct target instead of a derived one.
        #
        # NED yaw = atan2(v_east, v_north), where 0=North, CW+.
        # MAVROS expects ENU yaw in PositionTarget: yaw_ENU = π/2 - yaw_NED.
        # For a velocity vector: yaw_NED = atan2(v_e, v_n),
        # so yaw_ENU = π/2 - atan2(v_e, v_n) = atan2(v_n, v_e).
        speed = math.hypot(v_n, v_e)
        if speed > 0.01:
            yaw_enu = math.atan2(v_n, v_e)  # ENU: 0=East, CCW+
        else:
            # Below 1 cm/s — hold last known heading to avoid atan2(0,0) noise.
            # P4 zero-vel freeze prevents actual motion, so this is just for
            # the yaw setpoint continuity.
            yaw_enu = self._last_yaw_cmd
        msg.yaw = yaw_enu

        # Position and acceleration: ignored by mask, set to safe values.
        msg.position.x = 0.0
        msg.position.y = 0.0
        msg.position.z = 0.0
        msg.acceleration_or_force.x = 0.0
        msg.acceleration_or_force.y = 0.0
        msg.acceleration_or_force.z = 0.0

        # Yaw_rate feedforward from RPP (NED CW+, body frame).
        # MAVROS LOCAL_NED passes yaw_rate through without negation, so send
        # the NED value directly (positive = CW = right turn).
        # Dynamic type_mask: when yaw_rate FF is active use 455 (send yaw_rate);
        # when FF is zero/stale use 2503 (ignore yaw_rate, let PX4 derive it
        # from velocity direction). Sending explicit 0 with mask=455 would
        # command PX4 to hold zero turn rate, blocking arc tracking.
        yaw_rate_age = float("inf")
        if self._yaw_rate_recv_time is not None:
            yaw_rate_age = (self.get_clock().now() - self._yaw_rate_recv_time).nanoseconds * 1e-9
        # T1 fix: compare against the wider yaw_rate_max_age_s window (not
        # input_max_age_s) so one slot of executor timer drift cannot drop
        # the κ·v feedforward for a cycle mid-arc.
        yaw_rate_max_age = self.get_parameter("yaw_rate_max_age_s").value
        if source == "rpp" and yaw_rate_age <= yaw_rate_max_age and abs(self._latest_yaw_rate_body) > 1e-4:
            msg.yaw_rate = self._latest_yaw_rate_body    # NED CW+ passed through directly
            msg.type_mask = TYPE_MASK_VEL_YAW_YAWRATE   # 455: vel + yaw + yaw_rate
        else:
            msg.yaw_rate = 0.0
            msg.type_mask = TYPE_MASK_VELOCITY_AND_YAW  # 2503: vel + yaw, ignore yaw_rate

        self._sp_pub.publish(msg)
        self._last_yaw_cmd = yaw_enu  # Track for next cycle's zero-speed hold
        self._published_count += 1

        # Heartbeat log every 5 seconds
        if self._published_count % (self.STREAM_HZ * 5) == 0:
            self.get_logger().debug(
                f"streaming [{source}] v=({v_n:+.3f},{v_e:+.3f},{v_d:+.3f}) m/s "
                f"yaw_enu={yaw_enu:.3f}rad yaw_rate={msg.yaw_rate:+.3f}rad/s "
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
