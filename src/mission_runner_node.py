#!/usr/bin/env python3
"""Mission runner — orchestrates pre-stream, OFFBOARD switch, arming, completion, disarm.

Pipeline position:
  [THIS NODE]  ←──  /rpp/debug   (state == DONE → mission complete)
       │
       ↓ service calls
  /mavros/set_mode      → switch PX4 between MANUAL and OFFBOARD
  /mavros/cmd/arming    → arm / disarm

This is the operator-facing entry point. It assumes:
  - twist_to_setpoint_node is already streaming /mavros/setpoint_raw/local
    (with zeros if no path / no RPP output)
  - rpp_controller_node is running and will publish a non-zero velocity once
    a /path arrives
  - path_publisher_node will be triggered separately (or already published)

Sequence executed by this node
------------------------------
  1. Wait for FCU connection (/mavros/state).connected == true
  2. Wait for setpoint stream confirmation (just give twist_to_setpoint 1 second)
  3. Switch to OFFBOARD via /mavros/set_mode
  4. Wait for mode change confirmation
  5. Arm via /mavros/cmd/arming
  6. Monitor /rpp/debug — when state_code == DONE (3) for `done_settle_s`,
     the mission is considered complete
  7. Disarm
  8. Switch back to MANUAL
  9. Exit

Safety
------
  - If FCU disconnects during the mission, immediately disarm and exit
  - If OFFBOARD mode changes externally (e.g. RC override), gracefully exit
    without disarming (operator is in control)
  - Ctrl+C triggers clean shutdown: stop streaming → disarm → MANUAL → exit
  - Total mission timeout (default 300 s = 5 min) — disarm if exceeded

Usage
-----
  ros2 run ... mission_runner --ros-args -p mission_timeout_s:=120.0
"""

import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Float32MultiArray


class MissionPhase(Enum):
    INIT = "INIT"
    WAIT_FCU = "WAIT_FCU"
    WAIT_STREAM = "WAIT_STREAM"
    SWITCH_OFFBOARD = "SWITCH_OFFBOARD"
    ARM = "ARM"
    RUNNING = "RUNNING"
    DONE = "DONE"
    DISARM = "DISARM"
    EXIT_MANUAL = "EXIT_MANUAL"
    FINISHED = "FINISHED"
    ABORTED = "ABORTED"


# RPP state codes from rpp_controller_node (kept in sync; see StateCode enum there)
RPP_STATE_DONE = 3
RPP_STATE_STALE = -1


class MissionRunnerNode(Node):
    """Drives the PX4 OFFBOARD lifecycle for a single path mission."""

    def __init__(self):
        super().__init__("mission_runner")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("mission_timeout_s",  300.0)  # 5 min default
        self.declare_parameter("done_settle_s",       1.0)   # state==DONE held for N s
        self.declare_parameter("stream_warmup_s",     1.5)   # stream before OFFBOARD
        self.declare_parameter("mode_switch_timeout_s", 5.0)
        self.declare_parameter("dry_run",            False)  # if true, never actually arms

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._phase = MissionPhase.INIT
        self._fcu_state: State | None = None
        self._rpp_debug: Float32MultiArray | None = None
        self._mission_t0 = self.get_clock().now()
        self._done_t0: float | None = None
        self._was_offboard = False  # track external mode changes

        # ------------------------------------------------------------------
        # QoS
        # ------------------------------------------------------------------
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        be_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(State, "/mavros/state", self._state_cb, state_qos)
        self.create_subscription(Float32MultiArray, "/rpp/debug", self._debug_cb, be_qos)

        # ------------------------------------------------------------------
        # Service clients
        # ------------------------------------------------------------------
        self._set_mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self._arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")

        # ------------------------------------------------------------------
        # Phase tick — 5 Hz state machine
        # ------------------------------------------------------------------
        self._tick = self.create_timer(0.2, self._phase_tick)

        dry = self.get_parameter("dry_run").value
        self.get_logger().info(
            f"mission_runner started ({'DRY RUN' if dry else 'LIVE'}). "
            f"Waiting for FCU. Phase: {self._phase.name}"
        )

    # ==================================================================
    # Subscriber callbacks
    # ==================================================================
    def _state_cb(self, msg: State):
        prev = self._fcu_state
        self._fcu_state = msg

        # External OFFBOARD exit detection (e.g. RC override, failsafe)
        if self._phase == MissionPhase.RUNNING:
            if self._was_offboard and msg.mode != "OFFBOARD":
                self.get_logger().warn(
                    f"OFFBOARD exited externally (mode={msg.mode!r}, "
                    f"armed={msg.armed}) — aborting mission"
                )
                self._phase = MissionPhase.ABORTED
            if prev and prev.armed and not msg.armed:
                self.get_logger().warn("Disarmed externally — aborting mission")
                self._phase = MissionPhase.ABORTED

    def _debug_cb(self, msg: Float32MultiArray):
        self._rpp_debug = msg

    # ==================================================================
    # State machine
    # ==================================================================
    def _phase_tick(self):
        # Global mission timeout
        elapsed = (self.get_clock().now() - self._mission_t0).nanoseconds * 1e-9
        timeout = self.get_parameter("mission_timeout_s").value
        if elapsed > timeout and self._phase not in (
            MissionPhase.DISARM, MissionPhase.EXIT_MANUAL,
            MissionPhase.FINISHED, MissionPhase.ABORTED, MissionPhase.INIT,
        ):
            self.get_logger().error(
                f"Mission timeout ({elapsed:.1f}s > {timeout:.1f}s) — aborting"
            )
            self._phase = MissionPhase.ABORTED

        # ----- Phase dispatch -----
        if self._phase == MissionPhase.INIT:
            self._phase = MissionPhase.WAIT_FCU

        elif self._phase == MissionPhase.WAIT_FCU:
            if self._fcu_state and self._fcu_state.connected:
                self.get_logger().info(
                    f"FCU connected (mode={self._fcu_state.mode}, "
                    f"armed={self._fcu_state.armed})"
                )
                self._phase_t0 = self.get_clock().now()
                self._phase = MissionPhase.WAIT_STREAM

        elif self._phase == MissionPhase.WAIT_STREAM:
            warmup = self.get_parameter("stream_warmup_s").value
            t_in_phase = (self.get_clock().now() - self._phase_t0).nanoseconds * 1e-9
            if t_in_phase >= warmup:
                self.get_logger().info(
                    f"Stream warmup complete ({warmup}s) — switching to OFFBOARD"
                )
                self._phase = MissionPhase.SWITCH_OFFBOARD

        elif self._phase == MissionPhase.SWITCH_OFFBOARD:
            if self.get_parameter("dry_run").value:
                self.get_logger().info("DRY RUN: skipping OFFBOARD switch")
                self._phase = MissionPhase.RUNNING
                self._was_offboard = True
                return
            ok = self._set_mode("OFFBOARD")
            if ok:
                # Wait for mode confirmation
                deadline = time.time() + self.get_parameter("mode_switch_timeout_s").value
                while time.time() < deadline:
                    if self._fcu_state and self._fcu_state.mode == "OFFBOARD":
                        self.get_logger().info("OFFBOARD confirmed")
                        self._was_offboard = True
                        self._phase = MissionPhase.ARM
                        return
                    rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().error(
                    f"OFFBOARD not confirmed (still {self._fcu_state.mode if self._fcu_state else 'unknown'}) — aborting"
                )
                self._phase = MissionPhase.ABORTED
            else:
                self.get_logger().error("OFFBOARD switch rejected — aborting")
                self._phase = MissionPhase.ABORTED

        elif self._phase == MissionPhase.ARM:
            if self.get_parameter("dry_run").value:
                self.get_logger().info("DRY RUN: skipping arm")
                self._phase = MissionPhase.RUNNING
                return
            ok = self._arm(True)
            if ok:
                self.get_logger().info("Armed — mission running")
                self._phase = MissionPhase.RUNNING
            else:
                self.get_logger().error("Arm rejected — aborting (set COM_ARM_WO_GPS=1 if no fix)")
                self._phase = MissionPhase.EXIT_MANUAL  # back to MANUAL without disarm

        elif self._phase == MissionPhase.RUNNING:
            # Watch /rpp/debug for DONE state
            if self._rpp_debug and len(self._rpp_debug.data) >= 8:
                state_code = int(self._rpp_debug.data[7])
                now_s = (self.get_clock().now() - self._mission_t0).nanoseconds * 1e-9

                if state_code == RPP_STATE_DONE:
                    if self._done_t0 is None:
                        self._done_t0 = now_s
                        self.get_logger().info("RPP reports DONE — settling...")
                    elif (now_s - self._done_t0) >= self.get_parameter("done_settle_s").value:
                        self.get_logger().info(
                            f"DONE settled for {self.get_parameter('done_settle_s').value}s — "
                            f"mission complete in {now_s:.1f}s"
                        )
                        self._phase = MissionPhase.DISARM
                else:
                    self._done_t0 = None  # reset if state changed

                if state_code == RPP_STATE_STALE:
                    self.get_logger().warn(
                        "RPP reports STALE pose — controller will emit zeros, "
                        "rover will hold position. Check MAVROS pose stream.",
                        throttle_duration_sec=5.0,
                    )

        elif self._phase == MissionPhase.DISARM:
            if self.get_parameter("dry_run").value:
                self.get_logger().info("DRY RUN: skipping disarm")
                self._phase = MissionPhase.EXIT_MANUAL
                return
            self._arm(False)  # best effort
            self._phase = MissionPhase.EXIT_MANUAL

        elif self._phase == MissionPhase.EXIT_MANUAL:
            if self.get_parameter("dry_run").value:
                self.get_logger().info("DRY RUN: skipping mode revert")
                self._phase = MissionPhase.FINISHED
                return
            self._set_mode("MANUAL")
            self._phase = MissionPhase.FINISHED

        elif self._phase == MissionPhase.ABORTED:
            self.get_logger().error("Mission aborted — disarming and reverting to MANUAL")
            if not self.get_parameter("dry_run").value:
                self._arm(False)
                self._set_mode("MANUAL")
            self._phase = MissionPhase.FINISHED

        elif self._phase == MissionPhase.FINISHED:
            self.get_logger().info("Mission finished — shutting down node")
            self._tick.cancel()
            # Trigger a clean shutdown
            rclpy.try_shutdown()

    # ==================================================================
    # Service helpers
    # ==================================================================
    def _set_mode(self, mode: str) -> bool:
        if not self._set_mode_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/mavros/set_mode unavailable")
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        future = self._set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.done() and future.result():
            r = future.result()
            ok = bool(r.mode_sent)
            self.get_logger().info(f"set_mode {mode}: {'sent' if ok else 'rejected'}")
            return ok
        self.get_logger().error(f"set_mode {mode}: timeout")
        return False

    def _arm(self, value: bool) -> bool:
        if not self._arm_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/mavros/cmd/arming unavailable")
            return False
        req = CommandBool.Request()
        req.value = value
        future = self._arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.done() and future.result():
            r = future.result()
            ok = bool(r.success)
            self.get_logger().info(f"{'arm' if value else 'disarm'}: {'success' if ok else 'denied'}")
            return ok
        self.get_logger().error(f"{'arm' if value else 'disarm'}: timeout")
        return False


def main():
    rclpy.init()
    node = None
    try:
        node = MissionRunnerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Ctrl+C — disarming and reverting to MANUAL")
            try:
                node._arm(False)
                node._set_mode("MANUAL")
            except Exception:
                pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
