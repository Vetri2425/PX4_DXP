#!/usr/bin/env python3
"""Pivot-walk sweep — how far does the rover TRANSLATE while pivoting N degrees?

WHY
---
CORNER_ALIGN does not command a yaw rate. It publishes a VELOCITY VECTOR of
magnitude `segment_min_corner_speed` (0.08 m/s) aimed up to +-75 deg off the
nose, and lets PX4's rover_differential derive heading from the vector bearing
(src/rpp_controller_node.py:3697-3703, _corner_pivot_velocity:4706).

PX4 spot-turns while the heading error exceeds RD_TRANS_DRV_TRN (40 deg) and
switches to DRIVING below RD_TRANS_TRN_DRV (2 deg). Any translation during that
sequence shows up as lateral "walk" at the mark entry. Field bags on 2026-08-03
measured 3.36 cm of walk, but every one of them turned 135-182 deg -- far too
narrow a span to reveal how walk scales with angle.

This sweep drives pure pivots from 45 to 360 deg and measures the displacement
of each, using the SAME command law as the controller.

SAFETY
------
* Publishes ZERO velocity until you ARM and select OFFBOARD yourself.
  The script never arms, never changes mode, never disarms.
* Aborts and zeroes the setpoint if: displacement exceeds ABORT_DISPLACEMENT_M,
  the vehicle leaves OFFBOARD, disarms, or a pivot exceeds its time budget.
* Zeroes velocity on every exit path including Ctrl-C.
* Rover spins in place: a ~1 m clear radius is enough.

CONFLICTING PUBLISHER
---------------------
rpp-pipeline also publishes /mavros/setpoint_velocity/cmd_vel. Stop it first or
the two fight:
    sudo systemctl stop rpp-pipeline      # check `armed` is false first
    ...run this sweep...
    sudo systemctl start rpp-pipeline

USAGE
    python3 tools/pivot_walk_sweep.py                 # full 10-angle ladder
    python3 tools/pivot_walk_sweep.py --angles 45,90  # subset
    python3 tools/pivot_walk_sweep.py --out /tmp/x.json
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State

# --- must mirror the controller ------------------------------------------
CORNER_SPEED_MS = 0.08          # max(0.05, segment_min_corner_speed)
MAX_BEARING_OFFSET_RAD = math.radians(75.0)   # _CORNER_MAX_BEARING_OFFSET_RAD
RELEASE_TOL_DEG = 3.0           # segment_heading_tolerance_deg
NOMINAL_RATE_RAD_S = 0.40       # segment_nominal_pivot_rate_rad_s
SPINUP_MARGIN_S = 1.0           # segment_pivot_spinup_margin_s

# --- sweep ----------------------------------------------------------------
DEFAULT_ANGLES = [45, 75, 90, 135, 160, 180, 210, 240, 270, 360]
RATE_HZ = 20.0
SETTLE_S = 2.5                  # zero-velocity dwell after each pivot
GAP_S = 3.0                     # pause between pivots
TIMEOUT_MARGIN = 2.0            # x the nominal budget before abort
ABORT_DISPLACEMENT_M = 0.50     # any pivot moving this far is a fault


def yaw_enu_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def corner_pivot_velocity(yaw_ned: float, step: float) -> tuple[float, float]:
    """Verbatim _corner_pivot_velocity, with `step` already clamped."""
    cmd_bearing = yaw_ned + step
    return (CORNER_SPEED_MS * math.cos(cmd_bearing),
            CORNER_SPEED_MS * math.sin(cmd_bearing))


class PivotSweep(Node):
    def __init__(self, angles: list[float], out_path: str):
        super().__init__("pivot_walk_sweep")
        self.angles = angles
        self.out_path = out_path
        self.results: list[dict] = []
        self.aborted: str | None = None

        pose_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST)
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               history=HistoryPolicy.KEEP_LAST)

        self._vel_pub = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._pose_cb, pose_qos)
        self.create_subscription(State, "/mavros/state", self._state_cb, state_qos)

        self.pose = None          # (t, east, north, yaw_ned)
        self.state = None
        self._cmd = (0.0, 0.0)    # (v_n, v_e) NED
        # stream at RATE_HZ from the very first tick: PX4 requires >=2 Hz
        # setpoints BEFORE it will accept OFFBOARD.
        self.create_timer(1.0 / RATE_HZ, self._tick)

    # ---- callbacks -------------------------------------------------------
    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        yaw_enu = yaw_enu_from_quat(msg.pose.orientation)
        self.pose = (time.time(), float(p.x), float(p.y),
                     wrap(math.pi / 2.0 - yaw_enu))     # ENU -> NED yaw

    def _state_cb(self, msg: State):
        self.state = msg

    def _tick(self):
        v_n, v_e = self._cmd
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x = v_e        # NED -> ENU
        m.twist.linear.y = v_n
        m.twist.linear.z = 0.0
        self._vel_pub.publish(m)

    # ---- helpers ---------------------------------------------------------
    def hold(self, seconds: float):
        self._cmd = (0.0, 0.0)
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self) -> bool:
        self.get_logger().info(
            "Streaming ZERO velocity. ARM and select OFFBOARD when the area is "
            "clear. Ctrl-C aborts.")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose and self.state and self.state.armed \
                    and self.state.mode == "OFFBOARD":
                self.get_logger().info("ARMED + OFFBOARD detected — starting in 3 s")
                self.hold(3.0)
                return True
        return False

    def healthy(self) -> str | None:
        if not self.state or not self.state.armed:
            return "disarmed"
        if self.state.mode != "OFFBOARD":
            return f"left OFFBOARD (mode={self.state.mode})"
        if not self.pose or (time.time() - self.pose[0]) > 0.5:
            return "pose stale"
        return None

    # ---- one pivot -------------------------------------------------------
    def run_pivot(self, target_deg: float) -> dict:
        target = math.radians(target_deg)
        sign = 1.0 if target >= 0 else -1.0
        budget = SPINUP_MARGIN_S + abs(target) / NOMINAL_RATE_RAD_S
        deadline = time.time() + budget * TIMEOUT_MARGIN

        t0, e0, n0, yaw0 = self.pose
        accum = 0.0
        prev_yaw = yaw0
        started = time.time()

        while rclpy.ok():
            why = self.healthy()
            if why:
                self.aborted = why
                break
            # accumulate UNWRAPPED rotation so >180 deg targets keep one direction
            accum += wrap(self.pose[3] - prev_yaw)
            prev_yaw = self.pose[3]
            remaining = target - accum
            if abs(remaining) <= math.radians(RELEASE_TOL_DEG):
                break
            if time.time() > deadline:
                self.aborted = f"timeout after {budget*TIMEOUT_MARGIN:.1f}s"
                break
            moved = math.hypot(self.pose[1] - e0, self.pose[2] - n0)
            if moved > ABORT_DISPLACEMENT_M:
                self.aborted = f"displacement {moved:.2f} m exceeded limit"
                break
            step = max(-MAX_BEARING_OFFSET_RAD,
                       min(MAX_BEARING_OFFSET_RAD, remaining))
            self._cmd = corner_pivot_velocity(self.pose[3], step)
            rclpy.spin_once(self, timeout_sec=0.02)

        dur = time.time() - started
        self._cmd = (0.0, 0.0)
        self.hold(SETTLE_S)

        _, e1, n1, yaw1 = self.pose
        d_e, d_n = e1 - e0, n1 - n0
        # decompose into the frame of the STARTING heading (NED yaw)
        fwd = d_n * math.cos(yaw0) + d_e * math.sin(yaw0)
        lat = -d_n * math.sin(yaw0) + d_e * math.cos(yaw0)   # + = RIGHT
        return {
            "target_deg": target_deg,
            "achieved_deg": math.degrees(accum),
            "duration_s": round(dur, 2),
            "walk_total_cm": round(100 * math.hypot(d_e, d_n), 2),
            "walk_fwd_cm": round(100 * fwd, 2),
            "walk_lat_cm": round(100 * lat, 2),
            "rate_deg_s": round(math.degrees(accum) / dur, 1) if dur > 0 else None,
            "aborted": self.aborted,
        }

    # ---- sweep -----------------------------------------------------------
    def run(self):
        if not self.wait_ready():
            return
        for i, ang in enumerate(self.angles, 1):
            if self.aborted:
                break
            self.get_logger().info(f"[{i}/{len(self.angles)}] pivot {ang:.0f} deg")
            r = self.run_pivot(ang)
            self.results.append(r)
            self.get_logger().info(
                f"    achieved {r['achieved_deg']:+.1f} deg in {r['duration_s']}s  "
                f"walk {r['walk_total_cm']} cm "
                f"(fwd {r['walk_fwd_cm']}, lat {r['walk_lat_cm']})")
            if self.aborted:
                self.get_logger().error(f"ABORTED: {self.aborted}")
                break
            self.hold(GAP_S)
        self._cmd = (0.0, 0.0)
        self.report()

    def report(self):
        if not self.results:
            print("\nno results")
            return
        print("\n" + "=" * 78)
        print("PIVOT WALK SWEEP   (walk_lat: + = RIGHT of the starting heading)")
        print("=" * 78)
        print(f"{'target':>8}{'achieved':>10}{'secs':>7}{'deg/s':>8}"
              f"{'walk_cm':>9}{'fwd_cm':>9}{'lat_cm':>9}")
        for r in self.results:
            print(f"{r['target_deg']:>8.0f}{r['achieved_deg']:>10.1f}"
                  f"{r['duration_s']:>7.1f}{r['rate_deg_s'] or 0:>8.1f}"
                  f"{r['walk_total_cm']:>9.2f}{r['walk_fwd_cm']:>9.2f}"
                  f"{r['walk_lat_cm']:>9.2f}")
        ok = [r for r in self.results if not r["aborted"]]
        if len(ok) >= 3:
            import statistics
            xs = [r["target_deg"] for r in ok]
            ys = [r["walk_total_cm"] for r in ok]
            mx, my = statistics.mean(xs), statistics.mean(ys)
            den = sum((x - mx) ** 2 for x in xs)
            if den > 0:
                slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den
                print(f"\n  walk vs angle: {slope:+.4f} cm/deg, "
                      f"intercept {my - slope*mx:+.2f} cm")
                print("  linear in angle  => translation accrues WHILE turning")
                print("  flat             => walk is a fixed per-pivot offset")
                print("  ~2r*sin(ang/2)   => antenna lever arm, not translation")
        Path(self.out_path).write_text(json.dumps(self.results, indent=2))
        print(f"\n  wrote {self.out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--angles", default=None,
                    help="comma list, e.g. 45,90,180 (default: full ladder)")
    ap.add_argument("--out", default="/tmp/pivot_walk_sweep.json")
    args = ap.parse_args()
    angles = ([float(a) for a in args.angles.split(",")]
              if args.angles else DEFAULT_ANGLES)

    rclpy.init()
    node = PivotSweep(angles, args.out)

    def bail(*_):
        node._cmd = (0.0, 0.0)
        for _ in range(10):
            node._tick()
            time.sleep(0.05)
        node.get_logger().warn("interrupted — zero velocity published")
        node.report()
        rclpy.try_shutdown()
        sys.exit(1)

    signal.signal(signal.SIGINT, bail)
    try:
        node.run()
    finally:
        node._cmd = (0.0, 0.0)
        for _ in range(10):
            node._tick()
            time.sleep(0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
