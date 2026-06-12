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
derives heading PRIMARILY from the velocity-vector bearing atan2(vE, vN). When
|v| < 0.01 m/s it freezes heading. So to make it turn we publish a velocity
VECTOR pointing where we want the nose; the firmware spot-turns toward it
(zero forward throttle while heading error > RD_TRANS_DRV_TRN). As a
feedforward we ALSO send the matching yaw_rate (type_mask 455, exactly like
twist_to_setpoint_node) so PX4's yaw controller tracks the sweep without phase
lag instead of deriving everything from the vector bearing.

Smooth profile (matches rpp_controller_node, fixes the old jerky version)
-------------------------------------------------------------------------
The previous version led the *measured* heading by a CONSTANT offset
(`yaw0 + direction*(rotated + lead)`). Working the algebra through, the
commanded bearing always sat a fixed `lead` ahead of the live heading, so PX4
saw a constant large heading error and spot-turned at its own max rate (~62°/s)
the entire spin — a bang-bang turn, not a controlled one — and the residual
lead caused overshoot at the end.

This version mirrors rpp_controller_node._corner_pivot_velocity instead:
  - A time-parameterized TRAPEZOIDAL yaw sweep advances an internal target
    heading from yaw0 to yaw0 + direction*total: ramp the rate up at
    `yaw_accel`, cruise at `yaw_rate_max`, then ramp DOWN so it arrives with
    ~zero rate (classic accel/cruise/decel profile).
  - The commanded bearing is CLOSED-LOOP on the live heading:
        cmd_bearing = yaw + clamp(target_heading - yaw, ±cone)
    so the firmware's perceived error — and thus its spot-turn rate — decays
    smoothly to zero as the rover catches the (now stationary) target. No
    constant max-rate turn, no overshoot.
  - The sweep's instantaneous angular rate is sent as a yaw_rate feedforward
    (NED CW+) so PX4 bypasses pure atan2 heading derivation lag.
  - The velocity magnitude is ramped up from 0 at `mag_accel` to avoid the
    step-input torque jerk at spin start.

Method
------
1. Standard OFFBOARD bring-up (stream → OFFBOARD → arm), copied from
   offboard_test.py conventions (FRAME_LOCAL_NED, 50 Hz, async services).
2. Record yaw0.
3. Spin: trapezoidal target sweep + closed-loop clamped bearing + yaw_rate
   feedforward + magnitude ramp (see above).
4. Once the sweep reaches the full rotation and the measured heading has
   settled, command ZERO velocity and let it settle.
5. Report: total rotation achieved, peak yaw rate, overshoot past target, and
   final heading error vs yaw0. PASS if |final error| <= pass_tol_deg.

Usage (on the Jetson, MAVROS up):
  ros2 run px4_dxp spin_in_place_test.py
  # or with params:
  ros2 run px4_dxp spin_in_place_test.py --ros-args \
      -p spin_deg:=360.0 -p direction:=cw -p spin_speed:=0.08 \
      -p yaw_rate_max_deg:=25.0 -p yaw_accel_deg:=40.0 -p bearing_cone_deg:=75.0

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

# Velocity + explicit yaw, yaw_rate IGNORED. mask 2503. Used when the yaw_rate
# feedforward is zero/inactive (e.g. preflight, settle, stop) — matches
# twist_to_setpoint_node's TYPE_MASK_VELOCITY_AND_YAW.
TYPE_MASK_VEL_YAW = (
    IGNORE_PX | IGNORE_PY | IGNORE_PZ
    | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
    | IGNORE_YAW_RATE
)

# Velocity + explicit yaw + yaw_rate feedforward. mask 455. This is the smooth
# path twist_to_setpoint_node uses (TYPE_MASK_VEL_YAW_YAWRATE): PX4's yaw
# controller tracks the commanded rate directly instead of deriving it from the
# velocity-vector bearing, eliminating the phase lag / oscillation that made
# the constant-lead version jerky.
TYPE_MASK_VEL_YAW_YAWRATE = (
    IGNORE_PX | IGNORE_PY | IGNORE_PZ
    | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
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
        self.declare_parameter("spin_speed", 0.08)       # m/s vector magnitude (cruise)
        # Trapezoidal yaw-sweep profile (replaces the old constant-lead scheme).
        # yaw_rate_max matches rpp_controller_node max_yaw_rate_body (0.45 rad/s
        # ≈ 25.8°/s validated). yaw_accel sets how fast the sweep ramps in/out.
        self.declare_parameter("yaw_rate_max_deg", 25.0)  # deg/s cruise yaw rate
        self.declare_parameter("yaw_accel_deg", 40.0)     # deg/s² sweep ramp
        # Closed-loop bearing cone: command no more than ±this off the live
        # nose, so PX4's reverse-detection never flips the turn. Matches
        # rpp_controller_node._CORNER_MAX_BEARING_OFFSET_RAD (75°).
        self.declare_parameter("bearing_cone_deg", 75.0)
        # Velocity magnitude ramp (m/s²) so the spin start is not a step input.
        self.declare_parameter("mag_accel", 0.35)
        self.declare_parameter("settle_tol_deg", 2.0)    # measured-heading settle band
        self.declare_parameter("pass_tol_deg", 3.0)      # verdict tolerance
        self.declare_parameter("max_spin_time_s", 40.0)  # watchdog

        self.spin_deg = float(self.get_parameter("spin_deg").value)
        self.direction = 1.0 if str(self.get_parameter("direction").value).lower() == "cw" else -1.0
        self.spin_speed = float(self.get_parameter("spin_speed").value)
        self.yaw_rate_max = math.radians(float(self.get_parameter("yaw_rate_max_deg").value))
        self.yaw_accel = math.radians(float(self.get_parameter("yaw_accel_deg").value))
        self.bearing_cone = math.radians(float(self.get_parameter("bearing_cone_deg").value))
        self.mag_accel = float(self.get_parameter("mag_accel").value)
        self.settle_tol = math.radians(float(self.get_parameter("settle_tol_deg").value))
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
        # Trapezoidal sweep state
        self._target_progress = 0.0  # commanded swept angle so far (rad, 0..total)
        self._sweep_rate = 0.0       # current sweep angular rate (rad/s, >=0)
        self._mag = 0.0              # current ramped velocity magnitude (m/s)
        self._last_sweep_t = None    # wall clock of previous sweep update

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
            f"{'CW' if self.direction > 0 else 'CCW'}, cruise={self.spin_speed} m/s, "
            f"yaw_rate_max={math.degrees(self.yaw_rate_max):.0f}°/s, "
            f"yaw_accel={math.degrees(self.yaw_accel):.0f}°/s², "
            f"cone=±{math.degrees(self.bearing_cone):.0f}° (trapezoidal sweep + yaw_rate FF)"
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

    def _make_setpoint(self, bearing_ned: float, speed: float,
                       yaw_rate_ned: float = 0.0) -> PositionTarget:
        """Velocity vector at `bearing_ned` (NED), magnitude `speed`, + explicit
        yaw = bearing (ENU), + optional yaw_rate feedforward (NED CW+).

        Matches twist_to_setpoint_node output: when |yaw_rate| > 1e-4 we send
        type_mask 455 (velocity + yaw + yaw_rate) so PX4's yaw controller tracks
        the commanded rate directly; otherwise 2503 (velocity + yaw, ignore
        yaw_rate) so PX4 derives heading from the vector bearing."""
        v_n = speed * math.cos(bearing_ned)
        v_e = speed * math.sin(bearing_ned)
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = FRAME_LOCAL_NED
        msg.velocity.x = v_e        # ENU East
        msg.velocity.y = v_n        # ENU North
        msg.velocity.z = 0.0
        msg.yaw = wrap_pi(math.pi / 2.0 - bearing_ned)   # ENU yaw
        # MAVROS LOCAL_NED passes yaw_rate through without negation (NED CW+).
        if abs(yaw_rate_ned) > 1e-4:
            msg.yaw_rate = yaw_rate_ned
            msg.type_mask = TYPE_MASK_VEL_YAW_YAWRATE   # 455
        else:
            msg.yaw_rate = 0.0
            msg.type_mask = TYPE_MASK_VEL_YAW           # 2503
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

        # ---- live spin control (trapezoidal sweep + closed-loop bearing) ----
        yaw = self._yaw_ned()
        now = time.time()
        if self._prev_yaw is not None:
            d = wrap_pi(yaw - self._prev_yaw)
            self.rotated += d
            dt_rate = now - (self._last_rate_t if hasattr(self, "_last_rate_t") else now)
            if dt_rate > 0:
                rate = abs(math.degrees(d) / dt_rate)
                self.peak_rate = max(self.peak_rate, rate)
            self._last_rate_t = now
        self._prev_yaw = yaw
        self.peak_rotated = max(self.peak_rotated, abs(self.rotated))

        # dt for the sweep/magnitude integrators
        dt = now - (self._last_sweep_t if self._last_sweep_t is not None else now)
        self._last_sweep_t = now
        dt = max(0.0, min(dt, 0.1))   # guard against scheduling hiccups

        # Trapezoidal target-heading sweep: ramp the sweep rate up at yaw_accel,
        # cruise at yaw_rate_max, then ramp DOWN over the braking distance so the
        # target arrives at `total` with ~zero rate. The commanded error then
        # decays to zero → no overshoot, unlike the old constant-lead scheme.
        remaining = self.total - self._target_progress
        # distance needed to bleed the current rate to zero at yaw_accel
        brake_dist = (self._sweep_rate * self._sweep_rate) / (2.0 * self.yaw_accel) if self.yaw_accel > 0 else 0.0
        if remaining <= brake_dist:
            self._sweep_rate = max(0.0, self._sweep_rate - self.yaw_accel * dt)
        else:
            self._sweep_rate = min(self.yaw_rate_max, self._sweep_rate + self.yaw_accel * dt)
        self._target_progress = min(self.total, self._target_progress + self._sweep_rate * dt)

        # Ramp the velocity magnitude up from 0 (no step input at spin start).
        self._mag = min(self.spin_speed, self._mag + self.mag_accel * dt)

        # Closed-loop bearing: command the live heading plus the clamped error
        # to the instantaneous target (mirrors _corner_pivot_velocity). The
        # firmware's perceived error shrinks to zero as it catches the target.
        target_heading = wrap_pi(self.yaw0 + self.direction * self._target_progress)
        heading_err = wrap_pi(target_heading - yaw)
        step = max(-self.bearing_cone, min(self.bearing_cone, heading_err))
        bearing = wrap_pi(yaw + step)

        # yaw_rate feedforward = signed sweep rate (NED CW+), matching the
        # twist_to_setpoint_node mask-455 path. Zero near the end so the mask
        # falls back to vector-derived heading for the final settle.
        yaw_rate_ff = self.direction * self._sweep_rate
        self.sp_pub.publish(self._make_setpoint(bearing, self._mag, yaw_rate_ff))

        signed_done = self.direction * self.rotated
        # progress log at ~2 Hz
        if now - self._last_log > 0.5:
            self._last_log = now
            self.get_logger().info(
                f"spin: rotated={math.degrees(signed_done):6.1f}/{self.spin_deg:.0f}°  "
                f"target={math.degrees(self._target_progress):6.1f}°  "
                f"err={math.degrees(heading_err):+5.1f}°  "
                f"sweep_rate={math.degrees(self._sweep_rate):4.0f}°/s  "
                f"rate≈{self.peak_rate:4.0f}°/s(peak)"
            )

        # Done when the sweep has reached the full rotation AND the measured
        # heading has settled onto the final target (closed-loop completion).
        final_err = abs(wrap_pi(yaw - wrap_pi(self.yaw0 + self.direction * self.total)))
        if self._target_progress >= self.total - 1e-6 and final_err <= self.settle_tol:
            self.get_logger().info("Reached target rotation and settled — stopping.")
            self.phase = "settle"

        # watchdog
        if self._spin_start_t and (now - self._spin_start_t) > self.max_spin_time:
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
        self._target_progress = 0.0
        self._sweep_rate = 0.0
        self._mag = 0.0
        self._last_sweep_t = None
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
                "Overshoot remains — try a lower yaw_rate_max_deg, a gentler "
                "yaw_accel_deg, or lower the FCU yaw-rate/accel limits in QGC, "
                "then re-run."
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
