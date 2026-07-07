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
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension


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
    SEGMENT_STATE_TRACK = 1
    SEGMENT_STATE_CORNER_STOP = 5
    HEADING_MODE_FROM_VELOCITY = 0.0
    HEADING_MODE_HOLD_LAST = 1.0
    HEADING_MODE_ZERO_HOLD = 2.0
    SOURCE_ZERO = 0.0
    SOURCE_RPP = 1.0
    SOURCE_STALE = 2.0

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
        # Universal braking guard: when RPP commands a reverse longitudinal
        # vector, keep heading continuous instead of turning that vector into
        # a 180-degree explicit yaw request. The segment state catches normal
        # CORNER_STOP; the angle fallback catches endpoint/run settle braking
        # or future profiles that reuse the same reverse-brake primitive.
        self.declare_parameter("hold_yaw_in_corner_stop", True)
        self.declare_parameter("reverse_brake_yaw_hold_enabled", True)
        self.declare_parameter("reverse_brake_yaw_hold_angle_deg", 100.0)
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
        self._last_motion_yaw_valid: bool = False
        self._latest_segment_state: int | None = None
        self._segment_state_recv_time = None
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
        self.create_subscription(
            Float32MultiArray, "/rpp/segment_debug", self._segment_debug_cb, be_qos
        )
        self._bridge_dbg_pub = self.create_publisher(
            Float32MultiArray, "/rpp/setpoint_bridge_debug", be_qos
        )

        # ------------------------------------------------------------------
        # 50 Hz stream timer
        # ------------------------------------------------------------------
        self._timer = self.create_timer(1.0 / self.STREAM_HZ, self._stream_cb)

        self.get_logger().info(
            f"twist_to_setpoint started — streaming /mavros/setpoint_raw/local "
            f"at {self.STREAM_HZ} Hz (frame=LOCAL_NED). Sources: /rpp/velocity_ned + "
            f"/rpp/yaw_rate_body + /rpp/segment_debug. Yaw+yaw_rate feedforward active "
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

    def _segment_debug_cb(self, msg: Float32MultiArray):
        # /rpp/segment_debug[1] is SegmentStateCode. Keep this optional: smooth
        # or future profiles may not publish it, so the geometry fallback below
        # still protects reverse-brake commands.
        if len(msg.data) > 1 and math.isfinite(msg.data[1]):
            self._latest_segment_state = int(round(float(msg.data[1])))
            self._segment_state_recv_time = self.get_clock().now()

    @staticmethod
    def _angle_wrap(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

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
        source_code = self.SOURCE_ZERO
        input_age_s = float("inf")

        if self._latest_vel is not None and self._latest_recv_time is not None:
            input_age_s = (self.get_clock().now() - self._latest_recv_time).nanoseconds * 1e-9
            if input_age_s <= max_age:
                v_n = float(self._latest_vel.vector.x)
                v_e = float(self._latest_vel.vector.y)
                v_d = float(self._latest_vel.vector.z)
                source = "rpp"
                source_code = self.SOURCE_RPP
            else:
                source = "stale"
                source_code = self.SOURCE_STALE
                self._stale_warn_count += 1
                # Warn at most once per second
                if self._stale_warn_count % self.STREAM_HZ == 0:
                    self.get_logger().warn(
                        f"Input stale ({input_age_s * 1000:.0f} ms > "
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
        heading_mode = self.HEADING_MODE_ZERO_HOLD
        segment_state = self._latest_segment_state
        segment_state_age_s = float("inf")
        if self._segment_state_recv_time is not None:
            segment_state_age_s = (
                self.get_clock().now() - self._segment_state_recv_time
            ).nanoseconds * 1e-9
        segment_state_fresh = segment_state_age_s <= max_age
        in_corner_stop = (
            bool(self.get_parameter("hold_yaw_in_corner_stop").value)
            and segment_state_fresh
            and segment_state == self.SEGMENT_STATE_CORNER_STOP
        )
        # Forward tracking (RPP SegmentStateCode.TRACK_SEGMENT) always commands a
        # forward-cone velocity — never a reverse brake — so the velocity bearing
        # IS the intended travel heading. This gate is essential after a
        # RUNTIME_ENTRY→MARK (or any run-boundary) pivot: on release the new MARK
        # heading can be ~180° from the previous entry-leg heading held in
        # _last_yaw_cmd, which would otherwise latch reverse_hold forever
        # (2026-07-07 14:26 bag: rover drove ~0.6 m backward on MARK with spray
        # ON because the bridge kept HOLD_LAST yaw at the stale entry heading).
        forward_tracking = (
            segment_state_fresh
            and segment_state == self.SEGMENT_STATE_TRACK
        )

        if speed > 0.01:
            velocity_yaw_enu = math.atan2(v_n, v_e)  # ENU: 0=East, CCW+
            reverse_hold = False
            if bool(self.get_parameter("reverse_brake_yaw_hold_enabled").value):
                hold_angle = math.radians(
                    float(self.get_parameter("reverse_brake_yaw_hold_angle_deg").value)
                )
                reverse_hold = (
                    source == "rpp"
                    and self._last_motion_yaw_valid
                    and not forward_tracking
                    and abs(self._angle_wrap(velocity_yaw_enu - self._last_yaw_cmd)) >= hold_angle
                )

            if in_corner_stop or reverse_hold:
                # The nonzero vector is a braking actuator request, not a new
                # travel-bearing contract. Keep the last commanded heading so
                # reverse braking does not become "face backward".
                yaw_enu = self._last_yaw_cmd
                heading_mode = self.HEADING_MODE_HOLD_LAST
            else:
                yaw_enu = velocity_yaw_enu
                heading_mode = self.HEADING_MODE_FROM_VELOCITY
        else:
            # Below 1 cm/s — hold last known heading to avoid atan2(0,0) noise.
            # P4 zero-vel freeze prevents actual motion, so this is just for
            # the yaw setpoint continuity.
            yaw_enu = self._last_yaw_cmd
            heading_mode = self.HEADING_MODE_ZERO_HOLD
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
        self._publish_bridge_debug(
            source_code=source_code,
            input_age_s=input_age_s,
            segment_state=float(segment_state) if segment_state is not None else float("nan"),
            segment_state_age_s=segment_state_age_s,
            heading_mode=heading_mode,
            v_n=v_n,
            v_e=v_e,
            v_d=v_d,
            yaw_enu=yaw_enu,
            yaw_rate=msg.yaw_rate,
            type_mask=float(msg.type_mask),
            speed=speed,
        )
        self._last_yaw_cmd = yaw_enu  # Track for next cycle's zero-speed hold
        if heading_mode == self.HEADING_MODE_FROM_VELOCITY:
            self._last_motion_yaw_valid = True
        self._published_count += 1

        # Heartbeat log every 5 seconds
        if self._published_count % (self.STREAM_HZ * 5) == 0:
            self.get_logger().debug(
                f"streaming [{source}] v=({v_n:+.3f},{v_e:+.3f},{v_d:+.3f}) m/s "
                f"yaw_enu={yaw_enu:.3f}rad yaw_rate={msg.yaw_rate:+.3f}rad/s "
                f"published={self._published_count}"
            )

    def _publish_bridge_debug(
        self,
        *,
        source_code: float,
        input_age_s: float,
        segment_state: float,
        segment_state_age_s: float,
        heading_mode: float,
        v_n: float,
        v_e: float,
        v_d: float,
        yaw_enu: float,
        yaw_rate: float,
        type_mask: float,
        speed: float,
    ) -> None:
        msg = Float32MultiArray()
        msg.layout.dim.append(
            MultiArrayDimension(label="rpp_setpoint_bridge_debug", size=13, stride=13)
        )
        msg.data = [
            float(self._published_count),              # [0] bridge publish sequence
            float(source_code),                        # [1] 0 zero, 1 rpp, 2 stale
            float(input_age_s * 1000.0) if math.isfinite(input_age_s) else -1.0,
                                                        # [2] /rpp/velocity_ned age ms
            float(segment_state),                      # [3] latest SegmentStateCode
            float(segment_state_age_s * 1000.0) if math.isfinite(segment_state_age_s) else -1.0,
                                                        # [4] /rpp/segment_debug age ms
            float(heading_mode),                       # [5] 0 vector, 1 hold, 2 zero-hold
            float(v_n),                                # [6] input velocity north m/s
            float(v_e),                                # [7] input velocity east m/s
            float(v_d),                                # [8] input velocity down m/s
            float(yaw_enu),                            # [9] published ENU yaw rad
            float(yaw_rate),                           # [10] published yaw_rate rad/s
            float(type_mask),                          # [11] PositionTarget type_mask
            float(speed),                              # [12] horizontal speed m/s
        ]
        self._bridge_dbg_pub.publish(msg)

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
