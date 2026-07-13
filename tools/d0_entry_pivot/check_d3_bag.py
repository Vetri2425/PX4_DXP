#!/usr/bin/env python3
"""D3 completion-stop bag checker — did the rover STOP on the endpoint?

Reads a rosbag2 bag from the straight-line run (publish_line.py) and reports the
coast-past. Regression target: bag 2026-07-10 Line_2m drove 1.08 m past the goal
because a bare zero setpoint coasts. With D3 the rover should stop on the point.

    python3 tools/d0_entry_pivot/check_d3_bag.py ~/bags/d3_YYYYMMDD_HHMMSS

Record the bag with (at least):
    /path  /mavros/local_position/pose  /rpp/segment_debug  /rpp/debug

Signals: B = final pose on /path (position.x=N, y=E). Rover pose is ENU
(x=E, y=N) -> NED (n=y, e=x). Metric = distance(rover, B) over time.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

APPROACH_M = 0.10        # "arrived" when within this of B
FINAL_STOP_MAX_M = 0.12  # resting distance from B must be <= this
COAST_MAX_M = 0.15       # max distance past B AFTER first arrival must be <= this
S_DONE = 4


def _load_bag(uri: str):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:  # pragma: no cover
        sys.exit(f"ERROR: ROS2 python libs missing ({e}). Run on the Jetson.")
    storage_id = "sqlite3"
    meta = os.path.join(uri, "metadata.yaml")
    if os.path.isfile(meta):
        with open(meta) as f:
            if "mcap" in f.read():
                storage_id = "mcap"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
                rosbag2_py.ConverterOptions("", ""))
    msgcls = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic in msgcls:
            yield topic, deserialize_message(data, msgcls[topic])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    args = ap.parse_args()
    if not os.path.isdir(args.bag):
        sys.exit(f"ERROR: not a bag directory: {args.bag}")

    goal_b = None               # (n, e) final path point
    rover = []                  # [(n, e)] in log order
    saw_done = False

    for topic, m in _load_bag(args.bag):
        if topic == "/path" and m.poses:
            last = m.poses[-1].pose.position
            goal_b = (last.x, last.y)          # /path is NED direct
        elif topic == "/mavros/local_position/pose":
            rover.append((m.pose.position.y, m.pose.position.x))   # ENU->NED
        elif topic == "/rpp/segment_debug" and len(m.data) >= 2:
            if int(round(m.data[1])) == S_DONE:
                saw_done = True

    if goal_b is None:
        sys.exit("ERROR: no /path in bag — record /path so the endpoint B is known")
    if not rover:
        sys.exit("ERROR: no /mavros/local_position/pose in bag")

    dists = [math.hypot(n - goal_b[0], e - goal_b[1]) for (n, e) in rover]
    i_arrive = next((i for i, d in enumerate(dists) if d <= APPROACH_M), None)
    min_d = min(dists)
    final_d = dists[-1]
    coast = max(dists[i_arrive:]) if i_arrive is not None else float("nan")

    print(f"\nD3 completion-stop check — {args.bag}")
    print(f"  endpoint B (NED): ({goal_b[0]:.3f}N, {goal_b[1]:.3f}E)   pose samples: {len(rover)}")
    print(f"  closest approach to B : {min_d*100:.1f} cm")
    print(f"  resting distance at end: {final_d*100:.1f} cm")
    if i_arrive is not None:
        print(f"  max distance AFTER arrival (coast-past): {coast*100:.1f} cm")
    print(f"  reached DONE state: {saw_done}")

    checks = []
    checks.append(("arrived", i_arrive is not None,
                   f"came within {APPROACH_M*100:.0f} cm of B" if i_arrive is not None
                   else f"never got within {APPROACH_M*100:.0f} cm of B"))
    if i_arrive is not None:
        checks.append(("no-coast-past", coast <= COAST_MAX_M,
                       f"max {coast*100:.1f} cm past B (<= {COAST_MAX_M*100:.0f} → "
                       f"{'stopped on point' if coast <= COAST_MAX_M else 'DROVE AWAY'})"))
    checks.append(("stopped-on-point", final_d <= FINAL_STOP_MAX_M,
                   f"resting {final_d*100:.1f} cm from B (<= {FINAL_STOP_MAX_M*100:.0f} → "
                   f"{'ok' if final_d <= FINAL_STOP_MAX_M else 'NOT ON POINT'})"))
    checks.append(("done-latched", saw_done, "segment_debug reached DONE" if saw_done
                   else "never latched DONE"))

    print()
    hard_fail = False
    for name, ok, detail in checks:
        if not ok:
            hard_fail = True
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:18s} {detail}")
    verdict = "FAIL" if hard_fail else "PASS"
    print(f"\n  ===> D3 {verdict} <===")
    print("  PASS → completion latch confirmed in-field; D1 (two-phase entry) unblocked.\n")
    return 1 if hard_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
