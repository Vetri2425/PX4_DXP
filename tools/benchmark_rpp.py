#!/usr/bin/env python3
"""
benchmark_rpp.py — RPP baseline / regression analysis tool.

Reads a rosbag (mcap or sqlite3) recorded during a mission run, decodes
/rpp/debug + /mavros/local_position/pose (+ optional /mavros/global_position/global),
and produces:

  - <out>/debug.csv         per-sample decoded data
  - <out>/summary.txt       one-line summary
  - <out>/xtrack.png        xtrack vs time + xtrack vs arclength

Summary one-liner format (canonical — do not change):
    path=square | n=1234 | t=23.4s | xtrack max=0.094 mean=0.011 rms=0.036 | heading_err rms_deg=2.1 | endpoint_err_m=0.018

Topics consumed
---------------
  /rpp/debug                        std_msgs/Float32MultiArray (10 floats; [8..9] optional)
                                     [0] cross_track_error_m  (+ = right of path)
                                     [1] heading_error_rad
                                     [2] lookahead_dist_m
                                     [3] speed_cmd_m_s
                                     [4] curvature_kappa
                                     [5] dist_to_goal_m
                                     [6] pose_age_ms
                                     [7] state_code  (StateCode IntEnum)
                                     [8] l_d_raw_m            (optional)
                                     [9] kappa_speed          (optional)
  /mavros/local_position/pose       ENU (x=East, y=North, z=Up). Swapped to NED on read.
  /mavros/global_position/global    optional, used only for endpoint_err if --mission absent

Frame discipline
----------------
Internally everything is NED (north/east), matching rpp_controller_node.

Endpoint error
--------------
If --mission <file.waypoints> is supplied, endpoint_err_m is the great-circle
distance from the final GPS fix (or, if no GPS topic recorded, NaN) to the
last waypoint of the mission. If --mission is absent, prints 'endpoint_err_m=n/a'.

Self-test
---------
    python tools/benchmark_rpp.py --self-test
prints a canonical summary line built from synthetic data and exits 0.

Dependencies (install on the analysis machine):
    pip install rosbags pandas matplotlib numpy
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple


# ---------------------------------------------------------------------------
# StateCode — must match src/rpp_controller_node.py (~line 137)
# ---------------------------------------------------------------------------
class StateCode(IntEnum):
    STALE = -1
    IDLE = 0
    TRACKING = 1
    APPROACH = 2
    DONE = 3
    RTK_WAIT = 4
    JUMP_SKIP = 5


# /rpp/debug indices (stable [0..7], optional [8..9])
IDX_XTRACK = 0
IDX_HEADING_ERR = 1
IDX_LOOKAHEAD = 2
IDX_SPEED_CMD = 3
IDX_KAPPA = 4
IDX_DIST_GOAL = 5
IDX_POSE_AGE = 6
IDX_STATE = 7
IDX_LD_RAW = 8
IDX_KAPPA_SPEED = 9

DEBUG_TOPIC = "/rpp/debug"
POSE_TOPIC = "/mavros/local_position/pose"
GLOBAL_TOPIC = "/mavros/global_position/global"


# ---------------------------------------------------------------------------
# Pure-data records (no ROS dependency)
# ---------------------------------------------------------------------------
@dataclass
class DebugSample:
    t: float                # bag receive time, seconds since first message
    xtrack_m: float
    heading_err_rad: float
    lookahead_m: float
    speed_cmd_m_s: float
    kappa: float
    dist_goal_m: float
    pose_age_ms: float
    state_code: int
    l_d_raw_m: float        # NaN if absent
    kappa_speed: float      # NaN if absent


@dataclass
class PoseSample:
    t: float                # bag receive time, seconds
    pos_n_m: float          # NED north (= ENU y)
    pos_e_m: float          # NED east  (= ENU x)


@dataclass
class GlobalSample:
    t: float
    lat: float
    lon: float


# ---------------------------------------------------------------------------
# Parsing helpers (testable without rosbags)
# ---------------------------------------------------------------------------
def decode_debug_array(data: Sequence[float], t: float) -> DebugSample:
    """Decode one /rpp/debug payload. Tolerates 8-element (old) and 10-element (new)."""
    n = len(data)
    if n < 8:
        raise ValueError(f"/rpp/debug payload has {n} elements (need >= 8)")
    nan = float("nan")
    return DebugSample(
        t=t,
        xtrack_m=float(data[IDX_XTRACK]),
        heading_err_rad=float(data[IDX_HEADING_ERR]),
        lookahead_m=float(data[IDX_LOOKAHEAD]),
        speed_cmd_m_s=float(data[IDX_SPEED_CMD]),
        kappa=float(data[IDX_KAPPA]),
        dist_goal_m=float(data[IDX_DIST_GOAL]),
        pose_age_ms=float(data[IDX_POSE_AGE]),
        state_code=int(data[IDX_STATE]),
        l_d_raw_m=float(data[IDX_LD_RAW]) if n > IDX_LD_RAW else nan,
        kappa_speed=float(data[IDX_KAPPA_SPEED]) if n > IDX_KAPPA_SPEED else nan,
    )


def decode_pose_enu_to_ned(x_enu: float, y_enu: float, t: float) -> PoseSample:
    """ENU pose -> NED. North = ENU y, East = ENU x."""
    return PoseSample(t=t, pos_n_m=float(y_enu), pos_e_m=float(x_enu))


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters."""
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def read_mission_waypoints(path: Path) -> List[Tuple[float, float]]:
    """Parse a QGC WPL 110 .waypoints file. Returns list of (lat, lon) for NAV_WAYPOINT rows.
    Skips the home/datum row (index 0)."""
    wps: List[Tuple[float, float]] = []
    with open(path) as f:
        lines = f.readlines()
    if not lines or not lines[0].startswith("QGC WPL"):
        raise ValueError(f"{path} not a QGC WPL file")
    for line in lines[1:]:
        line = line.strip()
        if not line:
            continue
        parts = line.split("\t")
        if len(parts) < 12:
            continue
        seq = int(parts[0])
        is_current = int(parts[1])
        # frame col may shift between formats; lat/lon are columns 8 and 9
        # in the QGC WPL 110 layout: seq, current, frame, command, p1-p4, lat, lon, alt, autocont
        try:
            lat = float(parts[8])
            lon = float(parts[9])
        except (ValueError, IndexError):
            continue
        # Skip home/datum row (seq==0, current==1)
        if seq == 0 and is_current == 1:
            continue
        wps.append((lat, lon))
    return wps


def compute_summary(
    debug: List[DebugSample],
    poses: List[PoseSample],
    globals_: List[GlobalSample],
    path_name: str,
    mission_last_wp: Optional[Tuple[float, float]],
) -> str:
    """Build canonical one-line summary."""
    if not debug:
        return f"path={path_name} | n=0 | t=0.0s | xtrack max=n/a mean=n/a rms=n/a | heading_err rms_deg=n/a | endpoint_err_m=n/a"

    n = len(debug)
    t0 = debug[0].t
    t1 = debug[-1].t
    t_dur = t1 - t0

    xs = [d.xtrack_m for d in debug]
    abs_xs = [abs(x) for x in xs]
    x_max = max(abs_xs)
    x_mean = sum(abs_xs) / n
    x_rms = math.sqrt(sum(x * x for x in xs) / n)

    he = [d.heading_err_rad for d in debug]
    he_rms_rad = math.sqrt(sum(h * h for h in he) / n)
    he_rms_deg = math.degrees(he_rms_rad)

    # endpoint_err: prefer GPS final fix vs mission last WP
    endpoint_str = "n/a"
    if mission_last_wp is not None:
        last_lat, last_lon = mission_last_wp
        if globals_:
            g = globals_[-1]
            d_m = haversine_m(g.lat, g.lon, last_lat, last_lon)
            endpoint_str = f"{d_m:.3f}"
        # else: no GPS topic in bag — leave n/a (local frame -> lat/lon would need datum)

    return (
        f"path={path_name} | n={n} | t={t_dur:.1f}s | "
        f"xtrack max={x_max:.3f} mean={x_mean:.3f} rms={x_rms:.3f} | "
        f"heading_err rms_deg={he_rms_deg:.1f} | "
        f"endpoint_err_m={endpoint_str}"
    )


def merge_pose_into_csv(debug: List[DebugSample], poses: List[PoseSample]) -> List[dict]:
    """Nearest-neighbor align poses to debug timestamps. Returns list of CSV rows."""
    rows: List[dict] = []
    if not poses:
        for d in debug:
            rows.append({
                "t": d.t,
                "xtrack_m": d.xtrack_m,
                "heading_err_rad": d.heading_err_rad,
                "lookahead_m": d.lookahead_m,
                "state_code": d.state_code,
                "pos_n": float("nan"),
                "pos_e": float("nan"),
                "speed_cmd_m_s": d.speed_cmd_m_s,
            })
        return rows

    # Pose timestamps sorted; do two-pointer nearest
    p_ts = [p.t for p in poses]
    j = 0
    for d in debug:
        # advance j to closest
        while j + 1 < len(p_ts) and abs(p_ts[j + 1] - d.t) <= abs(p_ts[j] - d.t):
            j += 1
        p = poses[j]
        rows.append({
            "t": d.t,
            "xtrack_m": d.xtrack_m,
            "heading_err_rad": d.heading_err_rad,
            "lookahead_m": d.lookahead_m,
            "state_code": d.state_code,
            "pos_n": p.pos_n_m,
            "pos_e": p.pos_e_m,
            "speed_cmd_m_s": d.speed_cmd_m_s,
        })
    return rows


# ---------------------------------------------------------------------------
# Rosbag reading (uses `rosbags` library)
# ---------------------------------------------------------------------------
def read_bag(bag_path: Path) -> Tuple[List[DebugSample], List[PoseSample], List[GlobalSample]]:
    """Read a rosbag2 (mcap or sqlite3). Requires `rosbags` package."""
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as e:
        raise RuntimeError(
            "rosbags package not installed. Run: pip install rosbags pandas matplotlib numpy"
        ) from e

    debug: List[DebugSample] = []
    poses: List[PoseSample] = []
    globals_: List[GlobalSample] = []

    with AnyReader([bag_path]) as reader:
        t0_ns: Optional[int] = None
        conns_debug = [c for c in reader.connections if c.topic == DEBUG_TOPIC]
        conns_pose = [c for c in reader.connections if c.topic == POSE_TOPIC]
        conns_glob = [c for c in reader.connections if c.topic == GLOBAL_TOPIC]
        wanted = conns_debug + conns_pose + conns_glob
        if not conns_debug:
            print(
                f"[warn] bag has no {DEBUG_TOPIC} — was rpp_controller_node running?",
                file=sys.stderr,
            )

        for conn, timestamp, raw in reader.messages(connections=wanted):
            if t0_ns is None:
                t0_ns = timestamp
            t = (timestamp - t0_ns) / 1e9
            msg = reader.deserialize(raw, conn.msgtype)

            if conn.topic == DEBUG_TOPIC:
                try:
                    debug.append(decode_debug_array(list(msg.data), t))
                except ValueError as e:
                    print(f"[warn] skipping debug msg at t={t:.3f}: {e}", file=sys.stderr)
            elif conn.topic == POSE_TOPIC:
                p = msg.pose.position
                poses.append(decode_pose_enu_to_ned(p.x, p.y, t))
            elif conn.topic == GLOBAL_TOPIC:
                globals_.append(GlobalSample(t=t, lat=float(msg.latitude), lon=float(msg.longitude)))

    return debug, poses, globals_


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------
def write_csv(rows: List[dict], out_path: Path) -> None:
    import csv
    fields = ["t", "xtrack_m", "heading_err_rad", "lookahead_m", "state_code",
              "pos_n", "pos_e", "speed_cmd_m_s"]
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow(row)


def write_plot(debug: List[DebugSample], poses: List[PoseSample], out_path: Path) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[warn] matplotlib not installed — skipping plot", file=sys.stderr)
        return

    if not debug:
        return

    ts = [d.t for d in debug]
    xs = [d.xtrack_m for d in debug]

    # arclength from pose stream (cumulative N/E distance)
    arclen = []
    if poses:
        cum = 0.0
        prev = poses[0]
        s = {prev.t: 0.0}
        for p in poses[1:]:
            cum += math.hypot(p.pos_n_m - prev.pos_n_m, p.pos_e_m - prev.pos_e_m)
            s[p.t] = cum
            prev = p
        # map debug t -> nearest pose t -> s
        p_ts = sorted(s.keys())
        j = 0
        for d in debug:
            while j + 1 < len(p_ts) and abs(p_ts[j + 1] - d.t) <= abs(p_ts[j] - d.t):
                j += 1
            arclen.append(s[p_ts[j]])

    fig, axes = plt.subplots(2 if arclen else 1, 1, figsize=(10, 6 if arclen else 4))
    ax1 = axes[0] if arclen else axes
    ax1.plot(ts, xs, linewidth=1.0)
    ax1.axhline(0, color="k", linewidth=0.5)
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("xtrack (m, + = right)")
    ax1.set_title("xtrack vs time")
    ax1.grid(True, alpha=0.3)

    if arclen:
        ax2 = axes[1]
        ax2.plot(arclen, xs, linewidth=1.0)
        ax2.axhline(0, color="k", linewidth=0.5)
        ax2.set_xlabel("arclength (m)")
        ax2.set_ylabel("xtrack (m, + = right)")
        ax2.set_title("xtrack vs arclength")
        ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=100)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------
def self_test() -> int:
    """Synthesise data, exercise parser+summary, print canonical line."""
    # Build 100 samples of a square-ish run: xtrack oscillates ±5cm, peaks 9cm at "corners"
    import random
    random.seed(0)
    debug: List[DebugSample] = []
    for i in range(100):
        t = i * 0.05
        # base sine + corner spikes at i=25,50,75
        x = 0.02 * math.sin(i * 0.3) + (0.07 if i in (25, 50, 75) else 0.0)
        d_list = [
            x,                     # 0 xtrack
            math.radians(2.0),     # 1 heading_err
            0.4,                   # 2 lookahead
            0.4,                   # 3 speed_cmd
            0.5,                   # 4 kappa
            10.0 - i * 0.1,        # 5 dist_goal
            5.0,                   # 6 pose_age_ms
            int(StateCode.TRACKING),  # 7 state
            0.4,                   # 8 l_d_raw
            0.4,                   # 9 kappa_speed
        ]
        debug.append(decode_debug_array(d_list, t))

    # Test 8-element tolerance
    short = decode_debug_array([0.01, 0.0, 0.4, 0.4, 0.5, 10.0, 5.0, 1], 0.0)
    assert math.isnan(short.l_d_raw_m), "l_d_raw_m must be NaN for 8-elem payload"
    assert math.isnan(short.kappa_speed), "kappa_speed must be NaN for 8-elem payload"

    poses = [PoseSample(t=i * 0.05, pos_n_m=i * 0.02, pos_e_m=0.0) for i in range(100)]
    globals_: List[GlobalSample] = []  # no GPS in self-test

    # With mission but no GPS -> n/a
    summary_no_gps = compute_summary(debug, poses, globals_, "square",
                                     mission_last_wp=(13.07208272, 80.26194903))
    # With GPS final fix near last WP -> ~0
    globals_ = [GlobalSample(t=4.95, lat=13.07208270, lon=80.26194900)]
    summary_with_gps = compute_summary(debug, poses, globals_, "square",
                                       mission_last_wp=(13.07208272, 80.26194903))

    print("[self-test] 8-element payload tolerated: OK")
    print("[self-test] no-GPS summary:")
    print("    " + summary_no_gps)
    print("[self-test] with-GPS summary:")
    print("    " + summary_with_gps)

    # Validate canonical structure
    for s in (summary_no_gps, summary_with_gps):
        for needle in ("path=", "| n=", "| t=", "s | xtrack max=", " mean=", " rms=",
                       "| heading_err rms_deg=", "| endpoint_err_m="):
            assert needle in s, f"summary missing '{needle}': {s}"

    # Test CSV row build
    rows = merge_pose_into_csv(debug, poses)
    assert len(rows) == 100, f"expected 100 CSV rows, got {len(rows)}"
    assert "speed_cmd_m_s" in rows[0], "csv missing speed_cmd_m_s column"

    # Test mission parsing on the new straight 5m file
    repo_root = Path(__file__).resolve().parent.parent
    mission = repo_root / "Test_mission" / "mission_straight_5m.waypoints"
    if mission.exists():
        wps = read_mission_waypoints(mission)
        assert len(wps) == 11, f"expected 11 WPs, got {len(wps)}"
        print(f"[self-test] mission parse OK: {len(wps)} waypoints")

    print("[self-test] PASS")
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="RPP baseline / regression analysis")
    ap.add_argument("--bag", type=Path, help="path to rosbag2 directory (mcap or db3)")
    ap.add_argument("--path", default="unknown", help="logical path name for summary (e.g. square)")
    ap.add_argument("--params", default="unknown", help="param set name (for the notes only)")
    ap.add_argument("--out", type=Path, default=Path("baseline"),
                    help="output directory (will be created)")
    ap.add_argument("--mission", type=Path, default=None,
                    help="optional .waypoints file for endpoint_err computation")
    ap.add_argument("--self-test", action="store_true", help="run synthetic self-test and exit")
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test()

    if args.bag is None:
        ap.error("--bag is required (or use --self-test)")

    bag = args.bag.resolve()
    if not bag.exists():
        print(f"[err] bag path does not exist: {bag}", file=sys.stderr)
        return 2

    debug, poses, globals_ = read_bag(bag)

    mission_last_wp: Optional[Tuple[float, float]] = None
    if args.mission is not None:
        if not args.mission.exists():
            print(f"[err] mission file not found: {args.mission}", file=sys.stderr)
            return 2
        wps = read_mission_waypoints(args.mission)
        if wps:
            mission_last_wp = wps[-1]
        else:
            print(f"[warn] mission file has no NAV_WAYPOINT rows: {args.mission}",
                  file=sys.stderr)

    args.out.mkdir(parents=True, exist_ok=True)
    rows = merge_pose_into_csv(debug, poses)
    write_csv(rows, args.out / "debug.csv")
    summary = compute_summary(debug, poses, globals_, args.path, mission_last_wp)
    (args.out / "summary.txt").write_text(summary + "\n")
    write_plot(debug, poses, args.out / "xtrack.png")

    print(summary)
    print(f"[out] {args.out}/debug.csv  ({len(rows)} rows)")
    print(f"[out] {args.out}/summary.txt")
    print(f"[out] {args.out}/xtrack.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
