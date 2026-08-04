#!/usr/bin/env python3
"""Offline replay of a recorded run through the REAL RPPControllerNode.

Built 2026-08-04 to chase the 2x2-square stall in bag stg_1e79599d: the segment
index oscillated (2 -> 5 -> 3 -> 2 -> 5), `dist_to_end_along` sat pinned at
0.000 for ~20 s while the rover moved, `heading_err` froze near -137 deg, and
the endpoint approach ramp clamped speed to 0.030 m/s until the run was
e-stopped at ~36 % coverage. A straight line never shows it.

This feeds the bag's OWN `/path` and pose sequence into a live node, one
control tick per recorded pose, and dumps the internal state the bag cannot
see: `_segment_idx`, the projection parameter `t`, `_path_travel_m`,
`_run_idx`, and the run-remaining measure. It then reports the FIRST tick where
the segment index moves backward, which is where the state machine breaks.

No rover and no ROS graph needed, but rclpy must import — run it on the Jetson.

    python3 tools/replay_segment_stall.py bags/<bundle>            # full trace
    python3 tools/replay_segment_stall.py bags/<bundle> --from 18 --to 50
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))


def _load_am():
    spec = importlib.util.spec_from_file_location(
        "am", os.path.join(_HERE, "analyze_mission.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _enu_pose(north, east, yaw_ned):
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = east          # MAVROS pose is ENU
    msg.pose.position.y = north
    msg.pose.position.z = 0.0
    yaw_enu = math.pi / 2.0 - yaw_ned
    half = yaw_enu / 2.0
    msg.pose.orientation.w = math.cos(half)
    msg.pose.orientation.z = math.sin(half)
    return msg


class _Cap:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)

    @property
    def last(self):
        return self.messages[-1] if self.messages else None


def replay(bundle, t_from=None, t_to=None, print_every=1):
    am = _load_am()
    bag, _ = am._find_bag_dir(bundle)
    s = am.collect(bag)

    # The RUN path (longest /path message) plus its spray/must-hit bitfield.
    runs = sorted(s.paths, key=lambda x: -len(x[1]))
    if not runs:
        print("no /path in bundle"); return 1
    _t, pts = runs[0]
    # Velocity is a REQUIRED input: the corner-stop / advance logic reads
    # _measured_speed(), so a replay without it sits in TRACK_SEGMENT forever
    # and never reproduces the real corner behaviour.
    vels = []
    for _tp, _m, _ts in am.read_bag(bag):
        if _tp == "/mavros/local_position/velocity_local":
            vels.append((_ts, _m["lx"], _m["ly"]))
    z = s.path_z or [0.0] * len(pts)
    print("bundle : %s" % bundle)
    print("path   : %d points, along-length %.3f m"
          % (len(pts), sum(math.hypot(pts[i + 1][0] - pts[i][0],
                                      pts[i + 1][1] - pts[i][1])
                           for i in range(len(pts) - 1))))
    print("poses  : %d samples over %.1f s   velocity samples: %d"
          % (len(s.pose), s.pose[-1][0] - s.pose[0][0], len(vels)))

    import rclpy
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    try:
        from rpp_controller_node import RPPControllerNode
        from nav_msgs.msg import Path
        from mavros_msgs.msg import GPSRAW
        from test_smoke_rpp_controller import _make_path_pose
        from geometry_msgs.msg import TwistStamped
        import bisect as _bisect

        node = RPPControllerNode()
        for attr in ("_vel_pub", "_yaw_rate_pub", "_dbg_pub", "_segment_dbg_pub",
                     "_conditioned_path_pub", "_spray_active_pub",
                     "_progress_pub", "_milestone_pub"):
            if hasattr(node, attr):
                setattr(node, attr, _Cap())

        path_msg = Path()
        path_msg.header.frame_id = "local_ned"
        path_msg.header.stamp = node.get_clock().now().to_msg()
        path_msg.poses = [
            _make_path_pose(n, e, mark=bool(int(round(zz)) & 1))
            for (n, e), zz in zip(pts, z)
        ]
        # keep the must-hit bit too — conditioning behaviour depends on it
        for ps, zz in zip(path_msg.poses, z):
            ps.pose.position.z = float(int(round(zz)))
        node._path_cb(path_msg)

        gps = GPSRAW(); gps.fix_type = 6
        node._gps_cb(gps)

        print("installed: %d runs, active run %d, conditioned path %d pts"
              % (len(getattr(node, "_runs", []) or []),
                 getattr(node, "_run_idx", -1), len(node._path)))
        print()

        t0 = s.pose[0][0]
        hdr = ("%8s %6s %5s %5s %8s %10s %10s %10s"
               % ("t", "state", "run", "seg", "proj_t", "travel_m", "remain_m", "dist_end"))
        print(hdr); print("-" * len(hdr))

        prev_seg = None
        prev_run = None
        run_switches = []
        first_backward = None
        rows = 0
        for i in range(len(s.pose)):
            t, n, e, yaw = s.pose[i]
            rel = t - t0
            if t_from is not None and rel < t_from:
                continue
            if t_to is not None and rel > t_to:
                break
            if vels:
                j = _bisect.bisect_left([v[0] for v in vels], t)
                j = max(0, min(j, len(vels) - 1))
                tw = TwistStamped()
                tw.twist.linear.x = float(vels[j][1])   # ENU East
                tw.twist.linear.y = float(vels[j][2])   # ENU North
                node._vel_cb(tw)
            node._pose_cb(_enu_pose(n, e, yaw))
            try:
                node._control_loop()
            except Exception as exc:                     # noqa: BLE001
                print("tick raised %s: %s" % (type(exc).__name__, exc))
                raise

            seg = getattr(node, "_segment_idx", -1)
            run_i = getattr(node, "_run_idx", -1)
            travel = getattr(node, "_path_travel_m", float("nan"))
            remain = node._run_remaining_along()
            proj_t = dist_end = float("nan")
            if len(node._path) >= 2:
                pt, _fn, _fe, _xt, dea = node._project_onto_segment(n, e, seg)
                proj_t, dist_end = pt, dea

            if prev_seg is not None and seg < prev_seg and first_backward is None:
                first_backward = (rel, prev_seg, seg)
            prev_seg = seg
            if prev_run is not None and run_i != prev_run:
                run_switches.append((rel, prev_run, run_i))
            prev_run = run_i

            if i % print_every == 0 and rows < 500:
                print("%8.2f %6d %5d %5d %8.3f %10.3f %10s %10.3f"
                      % (rel, int(getattr(node, "_segment_state", -1)), run_i, seg,
                         proj_t, travel,
                         "None" if remain is None else "%.3f" % remain, dist_end))
                rows += 1

        print()
        print("RUN SWITCHES: %s" % (
            ", ".join("t=%.1f %d->%d" % x for x in run_switches) or "NONE"))
        print("final run %s seg %s state %s" % (
            getattr(node, "_run_idx", -1), getattr(node, "_segment_idx", -1),
            int(getattr(node, "_segment_state", -1))))
        if first_backward:
            print("FIRST BACKWARD segment index: t=%.2f s  %d -> %d"
                  % first_backward)
        else:
            print("segment index never moved backward in this window")
        return 0
    finally:
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bundle")
    ap.add_argument("--from", dest="t_from", type=float, default=None)
    ap.add_argument("--to", dest="t_to", type=float, default=None)
    ap.add_argument("--print-every", type=int, default=1,
                    help="decimate PRINTING only; every pose is always ticked, "
                         "because skipping poses makes the position delta trip "
                         "the EKF jump guard and every cycle is discarded")
    a = ap.parse_args()
    raise SystemExit(replay(a.bundle, a.t_from, a.t_to, max(1, a.print_every)))


if __name__ == "__main__":
    main()
