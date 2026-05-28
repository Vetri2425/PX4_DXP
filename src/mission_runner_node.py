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

Service call design
-------------------
  All MAVROS service calls use call_async() + future polling in the 5 Hz tick.
  This avoids the deadlock that spin_until_future_complete() causes when called
  from within a timer callback (both share the same MutuallyExclusiveCallbackGroup).

Usage
-----
  ros2 run ... mission_runner --ros-args -p mission_timeout_s:=120.0
"""

import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Float32MultiArray


class MissionPhase(Enum):
    INIT = "INIT"
    WAIT_FCU = "WAIT_FCU"
    WAIT_STREAM = "WAIT_STREAM"
    SWITCH_OFFBOARD = "SWITCH_OFFBOARD"
    CONFIRM_OFFBOARD = "CONFIRM_OFFBOARD"
    WAIT_OFFBOARD_STATE = "WAIT_OFFBOARD_STATE"
    WAIT_POSITION = "WAIT_POSITION"   # wait for EKF position valid (RPP TRACKING)
    ARM = "ARM"
    CONFIRM_ARM = "CONFIRM_ARM"
    RUNNING = "RUNNING"
    DONE = "DONE"
    DISARM = "DISARM"
    CONFIRM_DISARM = "CONFIRM_DISARM"
    EXIT_MANUAL = "EXIT_MANUAL"
    CONFIRM_MANUAL = "CONFIRM_MANUAL"
    FINISHED = "FINISHED"
    ABORTED = "ABORTED"


# RPP state codes from rpp_controller_node (kept in sync; see StateCode enum there)
RPP_STATE_DONE = 3
RPP_STATE_STALE = -1
# B2: additional non-driving codes — same response as STALE (keep RUNNING,
# warn at a throttled rate; the controller is publishing zero velocity for
# a documented reason and the server-level watchdog handles eventual abort).
RPP_STATE_RTK_WAIT = 4
RPP_STATE_JUMP_SKIP = 5
RPP_STATE_NONDRIVING = (RPP_STATE_STALE, RPP_STATE_RTK_WAIT, RPP_STATE_JUMP_SKIP)

SERVICE_TIMEOUT_S = 5.0


class MissionRunnerNode(Node):
    """Drives the PX4 OFFBOARD lifecycle for a single path mission."""

    def __init__(self):
        super().__init__("mission_runner")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("mission_timeout_s",  300.0)  # 5 min default
        self.declare_parameter("done_settle_s",       1.0)   # state==DONE held for N s
        self.declare_parameter("stream_warmup_s",     0.5)   # stream before OFFBOARD
        self.declare_parameter("mode_switch_timeout_s", 5.0)
        self.declare_parameter("dry_run",            False)  # if true, never actually arms
        self.declare_parameter("post_offboard_settle_s", 1.0)  # min wait after OFFBOARD before arm
        self.declare_parameter("arm_max_retries",        3)    # retry arm this many times
        self.declare_parameter("arm_retry_delay_s",      5.0)  # seconds between retries

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._phase = MissionPhase.INIT
        self._fcu_state: State | None = None
        self._rpp_debug: Float32MultiArray | None = None
        self._mission_t0 = self.get_clock().now()
        self._done_t0: float | None = None
        self._was_offboard = False  # track external mode changes
        self._pending_future = None   # active service call future
        self._future_t0 = None        # timestamp when future was created
        self._running_t0: float | None = None  # wall time when RUNNING entered
        self._arm_attempts = 0        # number of arm attempts made
        self._arm_retry_t0: float | None = None  # wall time of last arm rejection

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
        # Callback groups — Reentrant for service clients so call_async
        # futures can complete while the timer tick is running.
        # ------------------------------------------------------------------
        svc_group = ReentrantCallbackGroup()

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(State, "/mavros/state", self._state_cb, state_qos)
        self.create_subscription(Float32MultiArray, "/rpp/debug", self._debug_cb, be_qos)
        self.create_subscription(StatusText, "/mavros/statustext", self._statustext_cb, be_qos)

        # ------------------------------------------------------------------
        # Service clients (ReentrantCallbackGroup to avoid deadlock)
        # ------------------------------------------------------------------
        self._set_mode_cli = self.create_client(
            SetMode, "/mavros/set_mode", callback_group=svc_group)
        self._arm_cli = self.create_client(
            CommandBool, "/mavros/cmd/arming", callback_group=svc_group)

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

        # Log every mode/arm transition (helps diagnose which failsafe fired)
        if prev and (prev.mode != msg.mode or prev.armed != msg.armed):
            elapsed = (self.get_clock().now() - self._mission_t0).nanoseconds * 1e-9
            hint = ""
            if msg.mode == "AUTO.LAND" and msg.armed:
                hint = " <-- FAILSAFE (RC loss or OFFBOARD loss — check COM_RCL_EXCEPT)"
            elif msg.mode == "AUTO.RTL":
                hint = " <-- FAILSAFE (RTL triggered)"
            self.get_logger().warn(
                f"FCU state: {prev.mode}/{prev.armed} -> {msg.mode}/{msg.armed} "
                f"at t={elapsed:.2f}s{hint}"
            )

        # External OFFBOARD exit detection (e.g. RC override, failsafe)
        if self._phase == MissionPhase.RUNNING:
            if self._was_offboard and msg.mode != "OFFBOARD":
                run_s = (time.time() - self._running_t0) if self._running_t0 else -1
                self.get_logger().warn(
                    f"OFFBOARD exited externally (mode={msg.mode!r}, "
                    f"armed={msg.armed}, ran_for={run_s:.1f}s) — aborting mission"
                )
                self._phase = MissionPhase.ABORTED
            if prev and prev.armed and not msg.armed:
                self.get_logger().warn("Disarmed externally — aborting mission")
                self._phase = MissionPhase.ABORTED

    def _debug_cb(self, msg: Float32MultiArray):
        self._rpp_debug = msg

    def _statustext_cb(self, msg: StatusText):
        """Log PX4 statustext during any active phase — shows arm rejections and failsafe reasons."""
        _quiet = (MissionPhase.INIT, MissionPhase.WAIT_FCU, MissionPhase.WAIT_STREAM,
                  MissionPhase.FINISHED)
        if self._phase not in _quiet:
            elapsed = (self.get_clock().now() - self._mission_t0).nanoseconds * 1e-9
            self.get_logger().warn(
                f"PX4 [{msg.severity}] t={elapsed:.2f}s [{self._phase.name}]: {msg.text}"
            )

    # ==================================================================
    # Non-blocking service call helpers
    # ==================================================================
    def _call_set_mode(self, mode: str) -> bool:
        """Initiate set_mode call. Returns True if call was sent, False if service unavailable."""
        if not self._set_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/mavros/set_mode unavailable")
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        self._pending_future = self._set_mode_cli.call_async(req)
        self._future_t0 = self.get_clock().now()
        return True

    def _call_arm(self, value: bool) -> bool:
        """Initiate arm/disarm call. Returns True if call was sent, False if service unavailable."""
        if not self._arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/mavros/cmd/arming unavailable")
            return False
        req = CommandBool.Request()
        req.value = value
        self._pending_future = self._arm_cli.call_async(req)
        self._future_t0 = self.get_clock().now()
        return True

    def _future_done(self) -> bool:
        """Check if the pending service call future has completed."""
        return self._pending_future is not None and self._pending_future.done()

    def _future_timed_out(self) -> bool:
        """Check if the pending service call has exceeded SERVICE_TIMEOUT_S."""
        if self._future_t0 is None:
            return False
        elapsed = (self.get_clock().now() - self._future_t0).nanoseconds * 1e-9
        return elapsed > SERVICE_TIMEOUT_S

    def _clear_future(self):
        self._pending_future = None
        self._future_t0 = None

    # ==================================================================
    # State machine
    # ==================================================================
    def _phase_tick(self):
        # Global mission timeout
        elapsed = (self.get_clock().now() - self._mission_t0).nanoseconds * 1e-9
        timeout = self.get_parameter("mission_timeout_s").value
        if elapsed > timeout and self._phase not in (
            MissionPhase.DISARM, MissionPhase.CONFIRM_DISARM,
            MissionPhase.EXIT_MANUAL, MissionPhase.CONFIRM_MANUAL,
            MissionPhase.FINISHED, MissionPhase.ABORTED, MissionPhase.INIT,
            MissionPhase.WAIT_OFFBOARD_STATE,
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
            sent = self._call_set_mode("OFFBOARD")
            if sent:
                self._phase = MissionPhase.CONFIRM_OFFBOARD
            else:
                self.get_logger().error("OFFBOARD switch — service unavailable, aborting")
                self._phase = MissionPhase.ABORTED

        elif self._phase == MissionPhase.CONFIRM_OFFBOARD:
            if self._future_done():
                r = self._pending_future.result()
                self._clear_future()
                ok = bool(r.mode_sent) if r else False
                if ok:
                    self.get_logger().info("set_mode OFFBOARD: sent")
                else:
                    self.get_logger().warn(f"set_mode OFFBOARD: rejected ({r})")
                # Wait for FCU state to reflect OFFBOARD regardless
                self._phase_t0 = self.get_clock().now()
                self._phase = MissionPhase.WAIT_OFFBOARD_STATE
            elif self._future_timed_out():
                self._clear_future()
                self.get_logger().error("set_mode OFFBOARD: timeout")
                self._phase = MissionPhase.ABORTED

        elif self._phase == MissionPhase.WAIT_OFFBOARD_STATE:
            # FCU state callback updates self._fcu_state; check if OFFBOARD
            if self._fcu_state and self._fcu_state.mode == "OFFBOARD":
                self.get_logger().info("OFFBOARD confirmed")
                self._was_offboard = True
                self._phase_t0 = self.get_clock().now()
                self._phase = MissionPhase.WAIT_POSITION

        elif self._phase == MissionPhase.WAIT_POSITION:
            # Wait until (a) RPP reports TRACKING (state=1) and (b) post_offboard_settle_s
            # has elapsed. The settle guard prevents arm rejection when EKF is still
            # stabilising after OFFBOARD switch (common on second run without reboot).
            rpp_state = int(self._rpp_debug.data[7]) if (
                self._rpp_debug and len(self._rpp_debug.data) >= 8) else -99
            wait_s = (self.get_clock().now() - self._phase_t0).nanoseconds * 1e-9
            settle = self.get_parameter("post_offboard_settle_s").value
            pos_timeout = self.get_parameter("mode_switch_timeout_s").value

            if rpp_state == 1 and wait_s >= settle:
                self.get_logger().info(
                    f"EKF position valid — RPP TRACKING, settled {wait_s:.1f}s — arming")
                self._phase = MissionPhase.ARM
            elif rpp_state == 1:
                self.get_logger().info(
                    f"RPP TRACKING — EKF settling ({wait_s:.1f}s/{settle:.1f}s)...",
                    throttle_duration_sec=1.0,
                )
            elif wait_s > pos_timeout:
                self.get_logger().error(
                    f"EKF position not valid after {pos_timeout:.0f}s "
                    f"(RPP state={rpp_state}) — aborting. "
                    f"Check EKF2_REQ_GPS_H (should be 1.0, not 10.0)."
                )
                self._phase = MissionPhase.ABORTED
            else:
                self.get_logger().info(
                    f"Waiting for EKF position (RPP state={rpp_state}, t={wait_s:.1f}s)...",
                    throttle_duration_sec=3.0,
                )

        elif self._phase == MissionPhase.ARM:
            if self.get_parameter("dry_run").value:
                self.get_logger().info("DRY RUN: skipping arm")
                self._phase = MissionPhase.RUNNING
                return
            sent = self._call_arm(True)
            if sent:
                self._phase = MissionPhase.CONFIRM_ARM
            else:
                self.get_logger().error("Arm — service unavailable, aborting")
                self._phase = MissionPhase.EXIT_MANUAL

        elif self._phase == MissionPhase.CONFIRM_ARM:
            # If we're in a retry backoff, just wait
            if self._arm_retry_t0 is not None:
                retry_delay = self.get_parameter("arm_retry_delay_s").value
                waited = time.time() - self._arm_retry_t0
                if waited >= retry_delay:
                    self._arm_retry_t0 = None
                    self.get_logger().info(
                        f"Arm retry attempt {self._arm_attempts + 1}/"
                        f"{self.get_parameter('arm_max_retries').value}..."
                    )
                    self._phase = MissionPhase.ARM
                return

            if self._future_done():
                r = self._pending_future.result()
                self._clear_future()
                ok = bool(r.success) if r else False
                if ok:
                    self.get_logger().info("Armed — mission running")
                    self._running_t0 = time.time()
                    self._phase = MissionPhase.RUNNING
                else:
                    self._arm_attempts += 1
                    max_retries = self.get_parameter("arm_max_retries").value
                    retry_delay = self.get_parameter("arm_retry_delay_s").value
                    if self._arm_attempts < max_retries:
                        self.get_logger().warn(
                            f"Arm rejected (attempt {self._arm_attempts}/{max_retries}) — "
                            f"retrying in {retry_delay:.0f}s. "
                            f"See PX4 statustext above for reason."
                        )
                        self._arm_retry_t0 = time.time()
                        # stay in CONFIRM_ARM — retry timer will flip back to ARM
                    else:
                        self.get_logger().error(
                            f"Arm rejected after {self._arm_attempts} attempts — aborting. "
                            f"See PX4 statustext above for exact reason."
                        )
                        self._phase = MissionPhase.EXIT_MANUAL
            elif self._future_timed_out():
                self._clear_future()
                self.get_logger().error("Arm: timeout")
                self._phase = MissionPhase.EXIT_MANUAL

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
                elif state_code == RPP_STATE_RTK_WAIT:
                    self.get_logger().warn(
                        "RPP reports RTK_WAIT — GPS fix below RTK_FIXED. "
                        "Controller refuses to drive until fix=6 OR you set "
                        "require_rtk_fix:=false. Check NTRIP stream.",
                        throttle_duration_sec=5.0,
                    )
                elif state_code == RPP_STATE_JUMP_SKIP:
                    self.get_logger().warn(
                        "RPP reports JUMP_SKIP — position jump > "
                        "ekf_jump_threshold_m. One-cycle skip; if persistent "
                        "the EKF is resetting (RTK acquisition?) or the "
                        "threshold is too tight for current speed.",
                        throttle_duration_sec=2.0,
                    )

        elif self._phase == MissionPhase.DISARM:
            if self.get_parameter("dry_run").value:
                self.get_logger().info("DRY RUN: skipping disarm")
                self._phase = MissionPhase.EXIT_MANUAL
                return
            sent = self._call_arm(False)
            if sent:
                self._phase = MissionPhase.CONFIRM_DISARM
            else:
                # Best effort — move on
                self._phase = MissionPhase.EXIT_MANUAL

        elif self._phase == MissionPhase.CONFIRM_DISARM:
            if self._future_done():
                self._clear_future()
                self._phase = MissionPhase.EXIT_MANUAL
            elif self._future_timed_out():
                self._clear_future()
                self.get_logger().warn("Disarm: timeout (continuing to MANUAL)")
                self._phase = MissionPhase.EXIT_MANUAL

        elif self._phase == MissionPhase.EXIT_MANUAL:
            if self.get_parameter("dry_run").value:
                self.get_logger().info("DRY RUN: skipping mode revert")
                self._phase = MissionPhase.FINISHED
                return
            sent = self._call_set_mode("MANUAL")
            if sent:
                self._phase = MissionPhase.CONFIRM_MANUAL
            else:
                self._phase = MissionPhase.FINISHED

        elif self._phase == MissionPhase.CONFIRM_MANUAL:
            if self._future_done():
                self._clear_future()
                self._phase = MissionPhase.FINISHED
            elif self._future_timed_out():
                self._clear_future()
                self.get_logger().warn("set_mode MANUAL: timeout")
                self._phase = MissionPhase.FINISHED

        elif self._phase == MissionPhase.ABORTED:
            self.get_logger().error("Mission aborted — disarming and reverting to MANUAL")
            if not self.get_parameter("dry_run").value:
                self._call_arm(False)
                self._call_set_mode("MANUAL")
            self._phase = MissionPhase.FINISHED

        elif self._phase == MissionPhase.FINISHED:
            self.get_logger().info("Mission finished — shutting down node")
            self._tick.cancel()
            rclpy.try_shutdown()


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
                if node._arm_cli.service_is_ready():
                    req = CommandBool.Request()
                    req.value = False
                    node._arm_cli.call_async(req)
                if node._set_mode_cli.service_is_ready():
                    req = SetMode.Request()
                    req.custom_mode = "MANUAL"
                    node._set_mode_cli.call_async(req)
            except Exception:
                pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()