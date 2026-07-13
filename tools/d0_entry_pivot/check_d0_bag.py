#!/usr/bin/env python3
"""D0 make-or-break bag checker — runtime-entry pivot gate.

Reads a rosbag2 bag from the D0 hairpin run (tools/d0_entry_pivot/hairpin.csv)
and prints PASS/FAIL on each bar from RUNBOOK.md §5. Run it on the Jetson
(has ROS2 Humble + rosbag2_py). Read-only; it never touches the robot.

    python3 tools/d0_entry_pivot/check_d0_bag.py ~/bags/d0_20260713_120000

The one question it answers: did velocity-OFFBOARD spot-turn ~169° from a DEAD
STOP cleanly — stop first, no reverse-flip, no oscillation, settle in tolerance?

Signals used
------------
  /rpp/segment_debug            Float32MultiArray  [1]=segment_state [7]=heading_err(rad)
  /rpp/velocity_ned             Vector3Stamped     commanded NED velocity (x=N, y=E)
  /mavros/local_position/pose   PoseStamped        quaternion -> ENU yaw -> NED yaw
  /mavros/local_position/velocity_local  TwistStamped  measured ground speed

SegmentStateCode: INACTIVE=0 TRACK_SEGMENT=1 PRE_CORNER_SLOWDOWN=2
                  CORNER_ALIGN=3 DONE=4 CORNER_STOP=5
"""
from __future__ import annotations

import argparse
import math
import os
import sys

# --- Pass thresholds (defaults; loosened slightly vs live params for EKF noise) ---
STOP_SPEED_PASS = 0.05        # m/s — min measured speed during CORNER_STOP must dip below this
FWD_EPS = -0.02               # m/s — commanded forward component during ALIGN must stay >= this
OSC_BAND_RAD = math.radians(8.0)   # a |heading_err| rise bigger than this counts as an oscillation
SETTLE_TOL_RAD = math.radians(3.0) # final |heading_err| at ALIGN release must be <= this (param is 2°)
TURNING_BAND_RAD = math.radians(5.0)  # below this |heading_err| the turn is DONE and the align-settle
                                      # brake (an intentional body-axis REVERSE) is expected — it is
                                      # NOT a reverse-flip, so exclude those samples from CHECK 2.
TURN_MIN_DEG, TURN_MAX_DEG = 140.0, 200.0   # actual pose-yaw sweep expected (~169°)

S_TRACK, S_ALIGN, S_STOP = 1, 3, 5


def _load_bag(uri: str):
    """Yield (topic, msg) in log order using rosbag2_py + message deserialization."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:  # pragma: no cover
        sys.exit(f"ERROR: ROS2 python libs missing ({e}). Run this on the Jetson.")

    storage_id = "sqlite3"
    meta = os.path.join(uri, "metadata.yaml")
    if os.path.isfile(meta):
        with open(meta) as f:
            if "mcap" in f.read():
                storage_id = "mcap"

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    typemap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgcls = {name: get_message(t) for name, t in typemap.items()}
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic in msgcls:
            yield topic, deserialize_message(data, msgcls[topic])


def _yaw_ned_from_pose(msg) -> float:
    """PoseStamped quaternion -> ENU yaw -> NED yaw (NED CW+, 0=North)."""
    q = msg.pose.orientation
    yaw_enu = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return math.pi / 2.0 - yaw_enu


def _nearest(series, t):
    """series = sorted list of (t, value); return value nearest time t (or None)."""
    if not series:
        return None
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    best = series[lo]
    if lo > 0 and abs(series[lo - 1][0] - t) < abs(best[0] - t):
        best = series[lo - 1]
    return best[1]


def _angle_wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="path to the rosbag2 directory")
    args = ap.parse_args()
    if not os.path.isdir(args.bag):
        sys.exit(f"ERROR: not a bag directory: {args.bag}")

    seg = []      # (t, state, heading_err_rad)
    velcmd = []   # (t, v_n, v_e)
    poseyaw = []  # (t, yaw_ned)
    measspd = []  # (t, speed)

    for topic, m in _load_bag(args.bag):
        if topic == "/rpp/segment_debug" and len(m.data) >= 8:
            t = len(seg)  # log-order index as time proxy (topic is 1/cycle)
            seg.append((t, int(round(m.data[1])), float(m.data[7])))
        elif topic == "/rpp/velocity_ned":
            velcmd.append((_hstamp(m), float(m.vector.x), float(m.vector.y)))
        elif topic == "/mavros/local_position/pose":
            poseyaw.append((_hstamp(m), _yaw_ned_from_pose(m)))
        elif topic == "/mavros/local_position/velocity_local":
            measspd.append((_hstamp(m), math.hypot(m.twist.linear.x, m.twist.linear.y)))

    if not seg:
        sys.exit("ERROR: no /rpp/segment_debug in bag — was tracking_profile=segment set?")

    # --- Locate the pivot window: first STOP -> first ALIGN after it -> first TRACK release ---
    def first(state, start=0):
        for i in range(start, len(seg)):
            if seg[i][1] == state:
                return i
        return None

    i_stop = first(S_STOP)
    i_align = first(S_ALIGN, (i_stop or 0) + 1) if i_stop is not None else None
    i_rel = first(S_TRACK, (i_align or 0) + 1) if i_align is not None else None
    if i_rel is None:
        i_rel = len(seg) - 1  # ran to end of bag still aligning

    poseyaw.sort(); measspd.sort(); velcmd.sort()
    # map segment_debug indices -> wallclock via header-stamped neighbours is not
    # needed: we sampled pose/vel by their own stamps and correlate by state window
    # using the segment_debug ORDER; for cross-topic lookups we use the fraction
    # of the pivot window (robust to slightly different rates).
    def win_frac_time(idx, series):
        if not series:
            return None
        frac = idx / max(1, len(seg) - 1)
        return series[min(len(series) - 1, int(frac * (len(series) - 1)))]

    results = []

    # CHECK 1 — stop before turn
    if i_stop is None:
        results.append(("stop-before-turn", False,
                        "segment never entered CORNER_STOP(5) — no braked stop at the corner"))
    elif i_align is None:
        results.append(("stop-before-turn", False,
                        "entered CORNER_STOP but never reached CORNER_ALIGN(3) — pivot never started"))
    else:
        # min measured speed during [stop..align] should dip below STOP_SPEED_PASS
        spd_win = [v for (t, v) in measspd] if measspd else []
        if spd_win:
            lo = int((i_stop / max(1, len(seg) - 1)) * (len(spd_win) - 1))
            hi = int((i_align / max(1, len(seg) - 1)) * (len(spd_win) - 1))
            seg_spd = spd_win[min(lo, hi):max(lo, hi) + 1] or spd_win
            mn = min(seg_spd)
            ok = mn < STOP_SPEED_PASS
            results.append(("stop-before-turn", ok,
                            f"CORNER_STOP before CORNER_ALIGN; min measured speed in stop window "
                            f"{mn:.3f} m/s (< {STOP_SPEED_PASS} → {'ok' if ok else 'STILL MOVING'})"))
        else:
            results.append(("stop-before-turn", True,
                            "CORNER_STOP precedes CORNER_ALIGN (no velocity_local topic to confirm speed)"))

    # CHECK 2 — no reverse-flip WHILE ACTIVELY TURNING.
    # A real BUG-T3 flip drives the pivot vector behind the nose while the
    # heading is still wrong. The align-settle phase ALSO commands a body-axis
    # reverse (the _corner_brake_velocity brake) once the heading is achieved —
    # that is intentional, not a flip. So only evaluate samples where the rover
    # is still actively turning (|heading_err| > TURNING_BAND); exclude the brake.
    if i_align is not None and velcmd and poseyaw:
        min_fwd = math.inf
        min_fwd_herr = None
        for idx in range(i_align, i_rel + 1):
            if abs(seg[idx][2]) <= TURNING_BAND_RAD:
                continue  # turn done — align-settle brake (intentional reverse), skip
            frac = idx / max(1, len(seg) - 1)
            vc = velcmd[min(len(velcmd) - 1, int(frac * (len(velcmd) - 1)))]
            yw = poseyaw[min(len(poseyaw) - 1, int(frac * (len(poseyaw) - 1)))]
            fwd = vc[1] * math.cos(yw[1]) + vc[2] * math.sin(yw[1])
            if fwd < min_fwd:
                min_fwd, min_fwd_herr = fwd, seg[idx][2]
        if min_fwd is math.inf:
            results.append(("no-reverse-flip", None,
                            "no active-turning samples in ALIGN (all braking) — skipped"))
        else:
            ok = min_fwd >= FWD_EPS
            results.append(("no-reverse-flip", ok,
                            f"min forward-component while actively turning {min_fwd:+.3f} m/s "
                            f"(at heading_err {math.degrees(min_fwd_herr):+.1f}°; >= {FWD_EPS} → "
                            f"{'ok' if ok else 'REVERSE-FLIP'}); align-settle brake excluded"))
    else:
        results.append(("no-reverse-flip", None, "insufficient velocity_ned/pose data — skipped"))

    # CHECK 3 — no oscillation: |heading_err| should not rise repeatedly during ALIGN
    if i_align is not None:
        herr = [abs(seg[i][2]) for i in range(i_align, i_rel + 1)]
        rises, max_rise, prev = 0, 0.0, herr[0] if herr else 0.0
        run_min = prev
        for h in herr[1:]:
            if h > run_min + OSC_BAND_RAD:
                rises += 1
                max_rise = max(max_rise, h - run_min)
                run_min = h
            else:
                run_min = min(run_min, h)
        ok = rises == 0
        results.append(("no-oscillation", ok,
                        f"{rises} rise(s) of |heading_err| > {math.degrees(OSC_BAND_RAD):.0f}° during ALIGN "
                        f"(max rise {math.degrees(max_rise):.1f}° → {'monotonic' if ok else 'OSCILLATES'})"))
    else:
        results.append(("no-oscillation", None, "no ALIGN phase — skipped"))

    # CHECK 4 — settles: final |heading_err| at release <= SETTLE_TOL
    if i_align is not None:
        final_err = abs(seg[i_rel][2]) if seg[i_rel][1] == S_TRACK else abs(seg[i_rel - 1][2])
        ok = final_err <= SETTLE_TOL_RAD
        results.append(("settles", ok,
                        f"|heading_err| at release {math.degrees(final_err):.2f}° "
                        f"(<= {math.degrees(SETTLE_TOL_RAD):.0f}° → {'ok' if ok else 'NOT SETTLED'})"))
    else:
        results.append(("settles", None, "no ALIGN phase — skipped"))

    # CHECK 5 — turn magnitude: pose yaw swept ~169° the short way
    if i_stop is not None and poseyaw:
        lo = int((i_stop / max(1, len(seg) - 1)) * (len(poseyaw) - 1))
        hi = int((i_rel / max(1, len(seg) - 1)) * (len(poseyaw) - 1))
        yaws = [y for (_t, y) in poseyaw[min(lo, hi):max(lo, hi) + 1]]
        swept = 0.0
        for a, b in zip(yaws, yaws[1:]):
            swept += _angle_wrap(b - a)
        deg = abs(math.degrees(swept))
        ok = TURN_MIN_DEG <= deg <= TURN_MAX_DEG
        results.append(("turn-magnitude", ok,
                        f"pose-yaw swept {deg:.1f}° "
                        f"(expect {TURN_MIN_DEG:.0f}-{TURN_MAX_DEG:.0f}° → {'ok' if ok else 'OFF-TARGET'})"))
    else:
        results.append(("turn-magnitude", None, "no pose data — skipped"))

    # --- Report ---
    print(f"\nD0 pivot check — {args.bag}")
    print(f"  segment_debug msgs: {len(seg)}   "
          f"STOP@{i_stop}  ALIGN@{i_align}  RELEASE@{i_rel}\n")
    hard_fail = False
    for name, ok, detail in results:
        tag = "PASS" if ok else ("WARN" if ok is None else "FAIL")
        if ok is False:
            hard_fail = True
        print(f"  [{tag}] {name:18s} {detail}")

    verdict = "FAIL" if hard_fail else "PASS"
    print(f"\n  ===> D0 {verdict} <===")
    print("  PASS → unblock D3→D1→D2→D4.   FAIL → scoped position-mode fallback (plan §9).\n")
    return 1 if hard_fail else 0


def _hstamp(m) -> float:
    """Header stamp in seconds (falls back to 0 if header missing)."""
    try:
        s = m.header.stamp
        return s.sec + s.nanosec * 1e-9
    except AttributeError:
        return 0.0


if __name__ == "__main__":
    raise SystemExit(main())
