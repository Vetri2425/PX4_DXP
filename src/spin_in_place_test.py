#!/usr/bin/env python3
"""Spin-in-place capability test for the 3WD differential rover.

Goal
----
Find out whether the rover can SPIN IN PLACE a full 360° and stop at the
*exact* start heading (0° error). If it can, the same rate-limited /
decelerating spin profile this script uses can be ported into
rpp_controller_node's corner pivot to kill the ~20° heading overshoot that
currently bows out the post-corner legs (bags 12-06-2026).

Why this command shape
----------------------
PX4 rover_differential (DifferentialVelControl, OFFBOARD velocity mode)
derives heading SOLELY from the velocity-vector bearing atan2(vE, vN) and
ignores the MAVROS yaw / yaw_rate fields. When |v| < 0.01 m/s it freezes
heading. So the ONLY way to make it turn is to publish a velocity VECTOR
pointing where we want the nose; the firmware spot-turns toward it (zero
forward throttle while heading error > RD_TRANS_DRV_TRN). This is NOT a
"formal" forward/path command — it is a pure spin-in-place primitive.

The current RPP pivot lets the firmware turn at its own max rate (~62°/s in
the bags) and overshoots ~20°. This script instead commands the *target*
bearing as a rate-limited sweep that DECELERATES into the final heading, so
the firmware tracks a gentle trapezoidal yaw profile and (hopefully) arrives
with little residual rate. We measure the result.

Method
------
1. Standard OFFBOARD bring-up (stream → OFFBOARD → arm), copied from
   offboard_test.py conventions (FRAME_LOCAL_NED, 50 Hz, async services).
2. Record yaw0.
3. Spin: each cycle, command bearing = yaw0 + direction * min(rotated + lead,
   total), where `lead` keeps the firmware spot-turning and is ramped DOWN
   over the last `decel_deg` so the rover eases into the final heading.
   Magnitude is a small fixed `spin_speed` (only sets bearing; firmware
   applies zero forward throttle while spot-turning).
4. Once the full rotation is reached, command ZERO velocity and let it settle.
5. Report: total rotation achieved, peak yaw rate, overshoot past target, and
   final heading error vs yaw0. PASS if |final error| <= pass_tol_deg.

Usage (on the Jetson, MAVROS up):
  ros2 run px4_dxp spin_in_place_test.py
  # or with params:
  ros2 run px4_dxp spin_in_place_test.py --ros-args \
      -p spin_deg:=360.0 -p direction:=cw -p spin_speed:=0.08 \
      -p lead_max_deg:=25.0 -p lead_min_deg:=4.0 -p decel_deg:=45.0

Safety:
  - Streams zero-velocity >=1 s before OFFBOARD (PX4 requirement).
  - Aborts if mode leaves OFFBOARD or the rover disarms unexpectedly.
  - Hard time cap on the spin; stop → disarm → MANUAL on exit / Ctrl-C.
  - This spins in place only; it never commands sustained forward travel.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mavros_msgs.msg import PositionTarget, State, StatusText
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped


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

# Velocity + explicit yaw (matches twist_to_setpoint_node so results transfer
# 1:1 to the real RPP pipeline). yaw is inert on the differential rover but we
# send it for parity. mask 2503.
TYPE_MASK_VEL_YAW = (
    IGNORE_PX | IGNORE_PY | IGNORE_PZ
    | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
    | IGNORE_YAW_RATE
)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class SpinInPlaceTest(Node):

    STREAM_HZ = 50
    PREFLIGHT_S = 1.2
    SETTLE_S = 3.0          # zero-velocity settle after the spin, then measure
    STOP_SETTLE_S = 0.5

    def __init__(self):
        super().__init__("spin_in_place_test")

        # ---- Parameters ----
        self.declare_parameter("spin_deg", 360.0)       # total rotation
        self.declare_parameter("direction", "cw")        # cw (NED +) or ccw (-)
        self.declare_parameter("spin_speed", 0.08)       # m/s vector magnitude
        self.declare_parameter("lead_max_deg", 25.0)     # bearing lead while spinning
        self.declare_parameter("lead_min_deg", 4.0)      # lead at the very end
        self.declare_parameter("decel_deg", 45.0)        # ramp lead down over last N deg
        self.declare_parameter("pass_tol_deg", 3.0)      # verdict tolerance
        self.declare_parameter("max_spin_time_s", 40.0)  # watchdog

        self.spin_deg = float(self.get_parameter("spin_deg").value)
        self.direction = 1.0 if str(self.get_parameter("direction").value).lower() == "cw" else -1.0
        self.spin_speed = float(self.get_parameter("spin_speed").value)
        self.lead_max = math.radians(float(self.get_parameter("lead_max_deg").value))
        self.lead_min = math.radians(float(self.get_parameter("lead_min_deg").value))
        self.decel = math.radians(float(self.get_parameter("decel_deg").value))
        self.pass_tol = float(self.get_parameter("pass_tol_deg").value)
        self.max_spin_time = float(self.get_parameter("max_spin_time_s").value)
        self.total = math.radians(self.spin_deg)

        # ---- State ----
        self.current_state = State()
        self.current_pose = None
        self.offboard_engaged = False
        self.mission_done = False
        self.phase = "preflight"     # preflight | spin | settle | stop
        self.yaw0 = None
        self._prev_yaw = None
        self.rotated = 0.0           # signed accumulated rotation (rad)
        self.peak_rate = 0.0         # deg/s
        self.peak_rotated = 0.0      # max |rotated| reached (rad) — for overshoot
        self._spin_start_t = None
        self._last_log = 0.0

        # ---- QoS ----
        sp_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)

        self.sp_pub = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", sp_qos)
        self.create_subscription(State, "/mavros/state", self._state_cb, state_qos)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._pose_cb, sp_qos)
        self.create_subscription(StatusText, "/mavros/statustext", self._statustext_cb, state_qos)
        self.set_mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self.arming_cli = self.create_client(CommandBool, "/mavros/cmd/arming")

        self.stream_timer = self.create_timer(1.0 / self.STREAM_HZ, self._stream_cb)

        self.get_logger().info(
            f"spin_in_place_test: spin {self.spin_deg:.0f}° "
            f"{'CW' if self.direction > 0 else 'CCW'}, speed={self.spin_speed} m/s, "
            f"lead {math.degrees(self.lead_max):.0f}→{math.degrees(self.lead_min):.0f}° "
            f"over last {math.degrees(self.decel):.0f}°"
        )
        self.set_mode_cli.wait_for_service(timeout_sec=10.0)
        self.arming_cli.wait_for_service(timeout_sec=10.0)

        self.get_logger().info("Waiting for FCU connection...")
        t = time.time()
        while not self.current_state.connected and (time.time() - t) < 30.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.current_state.connected:
            self.get_logger().error("FCU not connected — aborting")
            self._shutdown()
            return

        self._spin_for(2.0)
        if self.current_pose is None:
            self.get_logger().error("No /mavros/local_position/pose — aborting")
            self._shutdown()
            return

        if self.current_state.mode not in ("MANUAL", "CMODE(393216)"):
            self._set_mode("MANUAL")
            self._spin_for(1.0)

        self._run()

    # ------------------------------------------------------------------
    def _state_cb(self, msg: State):
        prev_armed = self.current_state.armed
        self.current_state = msg
        if self.offboard_engaged:
            if msg.mode != "OFFBOARD":
                self.get_logger().warn(f"Mode left OFFBOARD ({msg.mode}) — aborting")
                self.offboard_engaged = False
                self.mission_done = True
            if prev_armed and not msg.armed:
                self.get_logger().warn("Disarmed unexpectedly — aborting")
                self.offboard_engaged = False
                self.mission_done = True

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _statustext_cb(self, msg: StatusText):
        sev = {0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERR", 4: "WARN",
               5: "NOTICE", 6: "INFO", 7: "DEBUG"}.get(msg.severity, "?")
        self.get_logger().info(f"[FCU {sev}] {msg.text}")

    # ------------------------------------------------------------------
    def _yaw_ned(self) -> float:
        q = self.current_pose.pose.orientation
        yaw_enu = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return wrap_pi(math.pi / 2.0 - yaw_enu)

    def _make_setpoint(self, bearing_ned: float, speed: float) -> PositionTarget:
        """Velocity vector at `bearing_ned` (NED), magnitude `speed`, + explicit
        yaw = bearing (ENU). Matches twist_to_setpoint_node output exactly."""
        v_n = speed * math.cos(bearing_ned)
        v_e = speed * math.sin(bearing_ned)
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = FRAME_LOCAL_NED
        msg.type_mask = TYPE_MASK_VEL_YAW
        msg.velocity.x = v_e        # ENU East
        msg.velocity.y = v_n        # ENU North
        msg.velocity.z = 0.0
        msg.yaw = wrap_pi(math.pi / 2.0 - bearing_ned)   # ENU yaw (inert on diff rover)
        return msg

    def _make_stop(self) -> PositionTarget:
        # Zero velocity; firmware holds current heading (no North-snap).
        hold = self._yaw_ned() if self.current_pose is not None else 0.0
        return self._make_setpoint(hold, 0.0)

    # ------------------------------------------------------------------
    def _stream_cb(self):
        if self.mission_done:
            return
        if self.phase == "preflight":
            self.sp_pub.publish(self._make_stop())
            return
        if self.phase in ("settle", "stop"):
            self.sp_pub.publish(self._make_stop())
            return
        if self.phase != "spin":
            return

        # ---- live spin control ----
        yaw = self._yaw_ned()
        if self._prev_yaw is not None:
            d = wrap_pi(yaw - self._prev_yaw)
            self.rotated += d
            now = time.time()
            dt = now - (self._last_rate_t if hasattr(self, "_last_rate_t") else now)
            if dt > 0:
                rate = abs(math.degrees(d) / dt)
                self.peak_rate = max(self.peak_rate, rate)
            self._last_rate_t = now
        self._prev_yaw = yaw
        self.peak_rotated = max(self.peak_rotated, abs(self.rotated))

        signed_done = self.direction * self.rotated   # progress toward +total
        remaining = self.total - signed_done

        # lead ramps down over the last `decel` rad so the firmware decelerates
        if remaining < self.decel:
            frac = max(0.0, remaining / self.decel)
            lead = self.lead_min + (self.lead_max - self.lead_min) * frac
        else:
            lead = self.lead_max

        # commanded bearing = yaw0 + direction * min(progress+lead, total)
        cmd_progress = min(signed_done + lead, self.total)
        bearing = wrap_pi(self.yaw0 + self.direction * cmd_progress)
        self.sp_pub.publish(self._make_setpoint(bearing, self.spin_speed))

        # progress log at ~2 Hz
        if time.time() - self._last_log > 0.5:
            self._last_log = time.time()
            self.get_logger().info(
                f"spin: rotated={math.degrees(signed_done):6.1f}/{self.spin_deg:.0f}°  "
                f"err_to_final={math.degrees(remaining):6.1f}°  "
                f"rate≈{self.peak_rate:4.0f}°/s(peak)  lead={math.degrees(lead):.0f}°"
            )

        # done when the full rotation is reached
        if signed_done >= self.total:
            self.get_logger().info("Reached target rotation — settling.")
            self.phase = "settle"

        # watchdog
        if self._spin_start_t and (time.time() - self._spin_start_t) > self.max_spin_time:
            self.get_logger().warn("Spin time cap reached — settling.")
            self.phase = "settle"

    # ------------------------------------------------------------------
    def _run(self):
        self.get_logger().info("=== SPIN-IN-PLACE TEST ===")
        self.phase = "preflight"
        self._spin_for(self.PREFLIGHT_S)

        if not self._set_mode("OFFBOARD"):
            self._shutdown(); return
        deadline = time.time() + 5.0
        while self.current_state.mode != "OFFBOARD" and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_state.mode != "OFFBOARD":
            self.get_logger().error(f"OFFBOARD not engaged ({self.current_state.mode})")
            self._shutdown(); return
        self.offboard_engaged = True

        if not self._arm(True):
            self._set_mode("MANUAL"); self._shutdown(); return

        # latch start heading
        self.yaw0 = self._yaw_ned()
        self._prev_yaw = self.yaw0
        self.rotated = 0.0
        self.peak_rotated = 0.0
        self._spin_start_t = time.time()
        self.get_logger().info(f"Start heading yaw0 = {math.degrees(self.yaw0):+.2f}° NED")

        self.phase = "spin"
        # spin runs in the stream callback; wait until it flips to settle
        while self.phase == "spin" and not self.mission_done:
            rclpy.spin_once(self, timeout_sec=0.05)

        # settle at zero velocity, then measure
        self.phase = "settle"
        self._spin_for(self.SETTLE_S)
        self._report()

        self.phase = "stop"
        self._spin_for(self.STOP_SETTLE_S)
        self._arm(False)
        self.offboard_engaged = False
        self.mission_done = True
        self._shutdown()

    def _report(self):
        if self.current_pose is None or self.yaw0 is None:
            self.get_logger().error("No pose — cannot measure.")
            return
        final_yaw = self._yaw_ned()
        final_err = math.degrees(wrap_pi(final_yaw - self.yaw0))
        total_rot = math.degrees(self.direction * self.rotated)
        overshoot = math.degrees(self.peak_rotated) - self.spin_deg
        verdict = "PASS" if abs(final_err) <= self.pass_tol else "FAIL"
        self.get_logger().info("================ SPIN RESULT ================")
        self.get_logger().info(f"  commanded rotation : {self.spin_deg:.1f}° "
                               f"{'CW' if self.direction > 0 else 'CCW'}")
        self.get_logger().info(f"  total rotated      : {total_rot:.1f}°")
        self.get_logger().info(f"  peak |yaw rate|    : {self.peak_rate:.0f}°/s")
        self.get_logger().info(f"  overshoot past 360 : {overshoot:+.1f}°")
        self.get_logger().info(f"  FINAL HEADING ERROR: {final_err:+.2f}°  (vs start)")
        self.get_logger().info(f"  VERDICT            : {verdict} "
                               f"(tol ±{self.pass_tol:.1f}°)")
        self.get_logger().info("=============================================")
        if verdict == "PASS":
            self.get_logger().info(
                "Rover CAN spin in place accurately with this profile — port "
                "the rate-limited/decelerating lead into rpp_controller's pivot."
            )
        else:
            self.get_logger().info(
                "Overshoot remains — try smaller lead_max_deg, larger decel_deg, "
                "or lower the FCU yaw-rate/accel limits in QGC, then re-run."
            )

    # ------------------------------------------------------------------
    def _set_mode(self, mode: str) -> bool:
        req = SetMode.Request(); req.custom_mode = mode
        fut = self.set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        ok = fut.done() and fut.result() and fut.result().mode_sent
        self.get_logger().info(f"set_mode {mode}: {'sent' if ok else 'FAILED'}")
        return bool(ok)

    def _arm(self, arm: bool) -> bool:
        req = CommandBool.Request(); req.value = arm
        fut = self.arming_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        ok = fut.done() and fut.result() and fut.result().success
        self.get_logger().info(f"{'arm' if arm else 'disarm'}: {'ok' if ok else 'DENIED'}")
        return bool(ok)

    def _spin_for(self, seconds: float):
        deadline = time.time() + seconds
        while time.time() < deadline and not self.mission_done:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _shutdown(self):
        self.get_logger().info("Shutting down (stop → disarm → MANUAL)...")
        try:
            self.stream_timer.cancel()
        except Exception:
            pass
        if self.current_state.armed:
            for _ in range(10):
                m = self._make_stop()
                m.header.stamp = self.get_clock().now().to_msg()
                self.sp_pub.publish(m)
                time.sleep(0.02)
            self._arm(False)
        if self.offboard_engaged:
            self._set_mode("MANUAL")


def main():
    rclpy.init()
    node = None
    try:
        node = SpinInPlaceTest()
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node._shutdown()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
