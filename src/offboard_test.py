#!/usr/bin/env python3
"""OFFBOARD test node for 3WD marking rover.

Covers Phase 2 Sessions 2-3:
  Session 2 — Position mode: 1m forward, hold, disarm
  Session 3 — Velocity mode: forward, reverse, stop, heading hold

Usage:
  # Session 2: position-mode 1m forward
  ros2 run px4_dxp offboard_test.py

  # Session 3: velocity-mode test
  ros2 run px4_dxp offboard_test.py --ros-args -p mode:=velocity

Safety:
  - Streams setpoints for 1s BEFORE requesting OFFBOARD (PX4 requirement)
  - Monitors /mavros/state — disarms on unexpected mode change
  - Publishes zero-velocity stop before disarm
  - Ctrl+C triggers clean shutdown: stop → disarm

PX4 OFFBOARD sequence (MUST follow this order):
  1. Start streaming setpoints at >=2Hz (we use 50Hz)
  2. Wait >=1s of continuous streaming
  3. Switch to OFFBOARD mode via /mavros/set_mode
  4. Arm via /mavros/cmd/arming
  5. Continue streaming setpoints without interruption

Important: PX4 rover rejects FRAME_BODY_OFFSET_NED (9). All setpoints
use FRAME_LOCAL_NED (1). Velocity commands are transformed from body frame
(forward/right) to NED (north/east) using the current heading.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mavros_msgs.msg import PositionTarget, State, ExtendedState, StatusText
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped


# ---------------------------------------------------------------------------
# PositionTarget type_mask constants (MAVLink SET_POSITION_TARGET_LOCAL_NED)
# ---------------------------------------------------------------------------
FRAME_LOCAL_NED = 1
# FRAME_BODY_OFFSET_NED = 9  # REJECTED by PX4 rover — use FRAME_LOCAL_NED + heading transform

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

# Position-only setpoint: send position.x (North), position.y (East)
# Ignores: PZ, all velocities, all accelerations, yaw, yaw_rate
TYPE_MASK_POSITION = (
    IGNORE_PZ | IGNORE_VX | IGNORE_VY | IGNORE_VZ
    | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
    | IGNORE_YAW | IGNORE_YAW_RATE
)  # = 3580

# Velocity-only setpoint: send velocity.x (North), velocity.y (East)
# Ignores: all positions, all accelerations, yaw (derived from velocity dir), yaw_rate
TYPE_MASK_VELOCITY = (
    IGNORE_PX | IGNORE_PY | IGNORE_PZ
    | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
    | IGNORE_YAW | IGNORE_YAW_RATE
)  # = 3527

# Stop setpoint: zero velocity, hold current heading
# Same as velocity-only but with vx=vy=vz=0
TYPE_MASK_STOP = TYPE_MASK_VELOCITY


class OffboardTestNode(Node):
    """OFFBOARD test node with position and velocity modes."""

    STREAM_HZ = 50       # setpoint publish rate (must be > 5Hz for COM_OF_LOSS_T=0.2)
    PREFLIGHT_S = 1.0    # seconds of streaming before OFFBOARD switch
    FORWARD_DIST = 1.0   # meters to drive forward (position mode)
    FORWARD_SPEED = 0.3  # m/s forward speed (velocity mode)
    HOLD_TIME = 2.0      # seconds to hold position/velocity after reaching target
    STOP_SETTLE_S = 0.5  # seconds to stream zero-velocity before disarm

    def __init__(self):
        super().__init__("offboard_test")

        # --- Parameters ---
        self.declare_parameter("mode", "position")  # "position" or "velocity"
        self.declare_parameter("forward_dist", self.FORWARD_DIST)
        self.declare_parameter("forward_speed", self.FORWARD_SPEED)

        self.mode = self.get_parameter("mode").value
        self.target_dist = self.get_parameter("forward_dist").value
        self.target_speed = self.get_parameter("forward_speed").value

        # --- State ---
        self.current_state = State()
        self.current_pose = None  # PoseStamped from /mavros/local_position/pose
        self.extended_state = None  # ExtendedState from /mavros/extended_state
        self.offboard_engaged = False
        self.mission_done = False
        self.phase = "preflight"  # preflight → stream → arm → run → stop → disarm

        # --- QoS profiles ---
        # MAVROS setpoint: BEST_EFFORT + VOLATILE (PX4 expects frequent updates)
        sp_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # MAVROS state: RELIABLE (important state changes)
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        # --- Publishers ---
        self.sp_pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", sp_qos
        )

        # --- Subscribers ---
        self.state_sub = self.create_subscription(
            State, "/mavros/state", self._state_cb, state_qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, sp_qos
        )
        self.ext_state_sub = self.create_subscription(
            ExtendedState, "/mavros/extended_state", self._ext_state_cb, state_qos
        )
        self.statustext_sub = self.create_subscription(
            StatusText, "/mavros/statustext", self._statustext_cb, state_qos
        )

        # --- Service clients ---
        self.set_mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self.arming_cli = self.create_client(CommandBool, "/mavros/cmd/arming")

        # --- Timer: 50Hz setpoint stream ---
        self.stream_timer = self.create_timer(1.0 / self.STREAM_HZ, self._stream_cb)

        # --- Wait for services ---
        self.get_logger().info(
            f"OFFBOARD test node started (mode={self.mode}, "
            f"dist={self.target_dist}m, speed={self.target_speed}m/s)"
        )
        self.get_logger().info("Waiting for MAVROS services...")
        self.set_mode_cli.wait_for_service(timeout_sec=10.0)
        self.arming_cli.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Services available.")

        # --- Wait for FCU connection ---
        self.get_logger().info("Waiting for FCU connection...")
        start = time.time()
        while not self.current_state.connected and (time.time() - start) < 30.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.current_state.connected:
            self.get_logger().error("FCU not connected after 30s — aborting")
            self._shutdown()
            return
        self.get_logger().info(
            f"FCU connected (mode={self.current_state.mode}, "
            f"armed={self.current_state.armed})"
        )

        # --- Check position estimate ---
        self.get_logger().info("Checking position estimate...")
        self._spin_for(2.0)  # Let ExtendedState arrive
        has_position = self.current_pose is not None
        self.get_logger().info(
            f"Position estimate: {'YES' if has_position else 'NO — may need COM_ARM_WO_GPS=1'}"
        )
        if not has_position:
            self.get_logger().warn(
                "No position estimate! If arming fails, set COM_ARM_WO_GPS=1 in QGC params"
            )

        # --- Check arm-readiness from ExtendedState ---
        if self.extended_state is not None:
            ls = self.extended_state.landed_state
            ls_names = {0: "UNDEFINED", 1: "ON_GROUND", 2: "IN_AIR", 3: "TAKEOFF"}
            self.get_logger().info(f"Landed state: {ls_names.get(ls, ls)}")
        else:
            self.get_logger().warn("No ExtendedState received — GPS/arming status unknown")

        # --- Reset to MANUAL for clean start ---
        if self.current_state.mode not in ("MANUAL", "CMODE(393216)"):
            self.get_logger().info(
                f"FCU in {self.current_state.mode} — resetting to MANUAL first"
            )
            self._set_mode("MANUAL")
            self._spin_for(1.0)

        # --- Start mission sequence ---
        self._run_mission()

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _state_cb(self, msg: State):
        prev_mode = self.current_state.mode
        prev_armed = self.current_state.armed
        self.current_state = msg

        if self.offboard_engaged:
            if msg.mode != "OFFBOARD":
                self.get_logger().warn(
                    f"Mode changed from OFFBOARD to {msg.mode} — disengaging"
                )
                self.offboard_engaged = False
                self.mission_done = True
            if prev_armed and not msg.armed:
                self.get_logger().info("Rover disarmed — ending mission")
                self.offboard_engaged = False
                self.mission_done = True

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _ext_state_cb(self, msg: ExtendedState):
        self.extended_state = msg

    def _statustext_cb(self, msg: StatusText):
        severity = {0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERR", 4: "WARN", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}
        tag = severity.get(msg.severity, f"?{msg.severity}")
        self.get_logger().info(f"[FCU {tag}] {msg.text}")

    # ------------------------------------------------------------------
    # Setpoint generators
    # ------------------------------------------------------------------
    def _make_position_setpoint(self, north: float, east: float) -> PositionTarget:
        """Create a position-only setpoint in NED frame."""
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = FRAME_LOCAL_NED
        msg.type_mask = TYPE_MASK_POSITION
        msg.position.x = north   # NED North
        msg.position.y = east    # NED East
        msg.position.z = 0.0     # NED Down (irrelevant for rover)
        return msg

    def _get_yaw_rad(self) -> float:
        """Get current heading in radians from pose quaternion (NED convention)."""
        if self.current_pose is None:
            return 0.0
        q = self.current_pose.pose.orientation
        # Quaternion to yaw (NED): yaw=0 is North, positive clockwise
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _make_velocity_setpoint(self, vx: float, vy: float) -> PositionTarget:
        """Create a velocity-only setpoint, transforming body frame to NED.

        PX4 rover rejects FRAME_BODY_OFFSET_NED (9). We must use FRAME_LOCAL_NED (1)
        and transform body-frame velocities ourselves using the current heading.

        Body frame:  vx = forward (+) / reverse (-), vy = right (+) / left (-)
        NED frame:   north = vx*cos(yaw) - vy*sin(yaw), east = vx*sin(yaw) + vy*cos(yaw)
        """
        yaw = self._get_yaw_rad()
        north_vel = vx * math.cos(yaw) - vy * math.sin(yaw)
        east_vel = vx * math.sin(yaw) + vy * math.cos(yaw)

        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = FRAME_LOCAL_NED
        msg.type_mask = TYPE_MASK_VELOCITY
        msg.velocity.x = north_vel  # NED North m/s
        msg.velocity.y = east_vel   # NED East m/s
        msg.velocity.z = 0.0
        return msg

    def _make_stop_setpoint(self) -> PositionTarget:
        """Create a zero-velocity stop setpoint (P4 holds heading via P4 patch)."""
        return self._make_velocity_setpoint(0.0, 0.0)

    def _make_hold_setpoint(self) -> PositionTarget:
        """Hold current position (re-publish last position target)."""
        if self.current_pose is not None:
            return self._make_position_setpoint(
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
            )
        # Fallback: hold at origin
        return self._make_position_setpoint(0.0, 0.0)

    # ------------------------------------------------------------------
    # 50Hz stream callback
    # ------------------------------------------------------------------
    def _stream_cb(self):
        """Publish setpoints at 50Hz. Content depends on mission phase."""
        if self.mission_done:
            return

        if self.phase == "preflight":
            # Before OFFBOARD: stream hold-at-origin to satisfy PX4 pre-stream requirement
            msg = self._make_position_setpoint(0.0, 0.0)
            self.sp_pub.publish(msg)

        elif self.phase == "run_position":
            # During position-mode mission: stream target position
            msg = self._make_position_setpoint(self.target_dist, 0.0)
            self.sp_pub.publish(msg)

        elif self.phase == "run_velocity_forward":
            msg = self._make_velocity_setpoint(self.target_speed, 0.0)
            self.sp_pub.publish(msg)

        elif self.phase == "run_velocity_reverse":
            msg = self._make_velocity_setpoint(-self.target_speed, 0.0)
            self.sp_pub.publish(msg)

        elif self.phase == "run_velocity_stop":
            msg = self._make_stop_setpoint()
            self.sp_pub.publish(msg)

        elif self.phase == "hold":
            msg = self._make_hold_setpoint()
            self.sp_pub.publish(msg)

        elif self.phase == "stop":
            msg = self._make_stop_setpoint()
            self.sp_pub.publish(msg)

    # ------------------------------------------------------------------
    # Mission sequence
    # ------------------------------------------------------------------
    def _run_mission(self):
        """Execute the full OFFBOARD mission sequence."""
        if self.mode == "position":
            self._run_position_mission()
        elif self.mode == "velocity":
            self._run_velocity_mission()
        else:
            self.get_logger().error(f"Unknown mode: {self.mode}")

    def _run_position_mission(self):
        """Session 2: Drive 1m forward in OFFBOARD position mode.

        Sequence:
        1. Stream hold-at-origin for 1s (preflight)
        2. Switch to OFFBOARD mode
        3. Arm
        4. Stream forward position for 3s (or until near target)
        5. Hold position for 2s
        6. Stop (zero velocity)
        7. Disarm
        """
        self.get_logger().info("=== POSITION MODE: 1m forward test ===")

        # Step 1: Preflight — stream for 1s
        self.get_logger().info("Step 1: Streaming setpoints for 1s (preflight)...")
        self.phase = "preflight"
        self._spin_for(self.PREFLIGHT_S)

        # Step 2: Switch to OFFBOARD
        self.get_logger().info("Step 2: Requesting OFFBOARD mode...")
        result = self._set_mode("OFFBOARD")
        if not result:
            self.get_logger().error("Failed to enter OFFBOARD mode — aborting")
            self._shutdown()
            return

        # Step 2b: Wait for mode to actually change
        self.get_logger().info("Waiting for OFFBOARD mode confirmation...")
        deadline = time.time() + 5.0
        while self.current_state.mode != "OFFBOARD" and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_state.mode != "OFFBOARD":
            self.get_logger().error(
                f"Mode did not switch to OFFBOARD (stuck at {self.current_state.mode}) — aborting"
            )
            self._shutdown()
            return
        self.offboard_engaged = True

        # Step 3: Arm
        self.get_logger().info("Step 3: Arming...")
        result = self._arm(True)
        if not result:
            self.get_logger().error("Failed to arm — aborting")
            self._set_mode("MANUAL")
            self._shutdown()
            return

        # Step 4: Drive forward
        self.get_logger().info(f"Step 4: Driving forward {self.target_dist}m...")
        self.phase = "run_position"
        drive_time = max(self.target_dist / self.target_speed * 1.5, 3.0)  # generous timeout
        self._spin_for(drive_time)

        # Step 5: Hold position
        self.get_logger().info("Step 5: Holding position for 2s...")
        self.phase = "hold"
        self._spin_for(self.HOLD_TIME)

        # Step 6: Stop
        self.get_logger().info("Step 6: Stopping...")
        self.phase = "stop"
        self._spin_for(self.STOP_SETTLE_S)

        # Step 7: Disarm
        self.get_logger().info("Step 7: Disarming...")
        self._arm(False)
        self.offboard_engaged = False

        self.get_logger().info("=== POSITION MODE TEST COMPLETE ===")
        self.mission_done = True
        self._shutdown()

    def _run_velocity_mission(self):
        """Session 3: Velocity-mode OFFBOARD test — forward, reverse, stop, heading hold.

        Uses FRAME_LOCAL_NED with heading-aware transform for velocity commands:
          vx=+0.3 → forward at 0.3 m/s (NED transform uses current yaw)
          vx=-0.3 → reverse at 0.3 m/s (NED transform uses current yaw)
          vx=0    → stop, P4 holds current heading (no North-snap)

        Sequence:
        1. Stream zero-velocity for 1s (preflight)
        2. Switch to OFFBOARD mode
        3. Arm
        4. Forward 0.3 m/s for 3s
        5. Stop (heading hold test — P4 validation)
        6. Reverse -0.3 m/s for 3s (P3 validation — backward without 180° spin)
        7. Stop again (heading hold — P4 validation)
        8. Hold for 2s
        9. Disarm
        """
        self.get_logger().info("=== VELOCITY MODE: forward/reverse/stop test ===")

        # Step 1: Preflight — stream zero-velocity for 1s
        self.get_logger().info("Step 1: Streaming zero-velocity for 1s (preflight)...")
        self.phase = "run_velocity_stop"
        self._spin_for(self.PREFLIGHT_S)

        # Step 2: Switch to OFFBOARD
        self.get_logger().info("Step 2: Requesting OFFBOARD mode...")
        result = self._set_mode("OFFBOARD")
        if not result:
            self.get_logger().error("Failed to enter OFFBOARD mode — aborting")
            self._shutdown()
            return

        # Step 2b: Wait for mode to actually change
        self.get_logger().info("Waiting for OFFBOARD mode confirmation...")
        deadline = time.time() + 5.0
        while self.current_state.mode != "OFFBOARD" and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_state.mode != "OFFBOARD":
            self.get_logger().error(
                f"Mode did not switch to OFFBOARD (stuck at {self.current_state.mode}) — aborting"
            )
            self._shutdown()
            return
        self.offboard_engaged = True

        # Step 3: Arm
        self.get_logger().info("Step 3: Arming...")
        result = self._arm(True)
        if not result:
            self.get_logger().error("Failed to arm — aborting")
            self._set_mode("MANUAL")
            self._shutdown()
            return

        # Step 4: Forward
        self.get_logger().info(f"Step 4: Forward {self.target_speed} m/s for 3s...")
        self.phase = "run_velocity_forward"
        self._spin_for(3.0)

        # Step 5: Stop (heading hold — P4 validation)
        self.get_logger().info("Step 5: Stop (heading hold test)...")
        self.phase = "run_velocity_stop"
        self._spin_for(2.0)

        # Step 6: Reverse (P3 validation — backward without 180° spin)
        self.get_logger().info(f"Step 6: Reverse {-self.target_speed} m/s for 3s (NED transform)...")
        self.phase = "run_velocity_reverse"
        self._spin_for(3.0)

        # Step 7: Stop again
        self.get_logger().info("Step 7: Stop (heading hold test)...")
        self.phase = "run_velocity_stop"
        self._spin_for(2.0)

        # Step 8: Hold
        self.get_logger().info("Step 8: Hold position for 2s...")
        self.phase = "hold"
        self._spin_for(self.HOLD_TIME)

        # Step 9: Stop + Disarm
        self.get_logger().info("Step 9: Stopping and disarming...")
        self.phase = "stop"
        self._spin_for(self.STOP_SETTLE_S)
        self._arm(False)
        self.offboard_engaged = False

        self.get_logger().info("=== VELOCITY MODE TEST COMPLETE ===")
        self.mission_done = True
        self._shutdown()

    # ------------------------------------------------------------------
    # Service calls
    # ------------------------------------------------------------------
    def _set_mode(self, mode: str) -> bool:
        """Switch PX4 flight mode via /mavros/set_mode."""
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            result = future.result()
            if result and result.mode_sent:
                self.get_logger().info(f"Mode switch to {mode}: sent")
                return True
            else:
                self.get_logger().error(f"Mode switch to {mode}: rejected ({result})")
                return False
        else:
            self.get_logger().error(f"Mode switch to {mode}: timeout")
            return False

    def _arm(self, arm: bool) -> bool:
        """Arm or disarm via /mavros/cmd/arming."""
        req = CommandBool.Request()
        req.value = arm
        future = self.arming_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            result = future.result()
            if result and result.success:
                self.get_logger().info(f"{'Arm' if arm else 'Disarm'}: success")
                return True
            else:
                reason = getattr(result, 'result', None)
                self.get_logger().error(
                    f"{'Arm' if arm else 'Disarm'}: DENIED (result={reason}). "
                    f"Likely causes: no GPS fix (set COM_ARM_WO_GPS=1) or pre-arm checks failed"
                )
                return False
        else:
            self.get_logger().error(f"{'Arm' if arm else 'Disarm'}: timeout")
            return False

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _spin_for(self, seconds: float):
        """Spin ROS2 for the given duration (non-blocking for timers)."""
        deadline = time.time() + seconds
        while time.time() < deadline and not self.mission_done:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _shutdown(self):
        """Clean shutdown: stop streaming, disarm if needed."""
        self.get_logger().info("Shutting down...")
        self.stream_timer.cancel()

        # Disarm if still armed
        if self.current_state.armed:
            self.get_logger().warn("Still armed at shutdown — disarming")
            # Publish stop setpoints before disarm
            for _ in range(10):
                msg = self._make_stop_setpoint()
                msg.header.stamp = self.get_clock().now().to_msg()
                self.sp_pub.publish(msg)
                time.sleep(0.02)
            self._arm(False)

        # Switch back to MANUAL (not HOLD — rover doesn't have HOLD mode)
        if self.offboard_engaged:
            self._set_mode("MANUAL")


def main():
    rclpy.init()
    node = None
    try:
        node = OffboardTestNode()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception: {e}")
        raise
    finally:
        if node:
            node._shutdown()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()