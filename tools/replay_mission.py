#!/usr/bin/env python3
"""Universal offline replay of a recorded mission through the REAL RPPControllerNode.

Supersedes the single-purpose replay_segment_stall.py (kept for --trace work).
Works for EVERY mission shape this rover runs — lines, there-and-back collinear
lines, squares/segment missions, circles/smooth curves, multi-run installs,
staged runtime-entry missions — and turns a bag into a verdict:

  * feeds EVERY /path message at its recorded time (staged missions publish
    several; a 1-point Path is the live e-stop sentinel and is fed as such)
  * feeds every pose, velocity and GPSRAW sample in bag order, one control
    tick per pose (skipping poses trips the EKF jump guard — never decimate)
  * captures what the node COMMANDS (velocity + /rpp/debug) tick by tick
  * prints a phase timeline: path installs, run switches, segment-state
    transitions, corner events
  * runs anomaly detectors tuned to the two OPPOSITE stop bugs on record:
      STRAND        cmd stuck inside PX4's RO_SPEED_TH dead-band (0<cmd<0.10)
                    while the rover is not moving and distance remains
      NO-STOP       a run boundary / corner stop crossed without the measured
                    speed ever reaching a stop (runtime-entry pre-start bug)
    plus corner run-through, backward segment index, state oscillation and
    lookahead collapse
  * fidelity check: replayed speed command vs the bag's own /rpp/debug[3],
    so you know the replay actually reproduced the field run before you
    trust any conclusion from it

Runs anywhere rclpy imports: the Jetson, or the Mac `ros-replay` micromamba
env (RoboStack Humble):

  MAMBA_ROOT_PREFIX=~/mamba /opt/homebrew/opt/micromamba/bin/micromamba run \
      -n ros-replay python3 tools/replay_mission.py bags/<bundle>

  --param name:=value   override any controller param (repeatable) — test a
                        candidate fix against a recorded failure, offline
  --no-auto-params      do not copy the bag's as-run param snapshot
                        (/rpp/debug slots) onto the node
  --csv out.csv         dump the per-tick record for plotting
  --from/--to           replay a time window (seconds from first pose)
  --print-every N       decimate PRINTING only (every pose is always ticked)

Open-loop caveat: the replay drives the node with the RECORDED poses, so it
proves mechanisms (what the node commanded and why), never remedies — a fix
that changes the commands would have changed the poses too.
"""

from __future__ import annotations

import argparse
import bisect
import csv as _csv
import importlib.util
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))

# PX4 speed dead-band: commands in (0, RO_SPEED_TH) do not turn the wheels.
RO_SPEED_TH = 0.10
STOPPED_V = 0.03          # measured speed below this = "stopped"
NOSTOP_V = 0.08           # min speed around a stop event above this = no-stop
STRAND_MIN_S = 2.0        # dead-band dwell before we call it a strand
SEG_STATE_NAMES = {0: "INACTIVE", 1: "TRACK", 2: "PRE_CORNER", 3: "ALIGN",
                   4: "DONE", 5: "CORNER_STOP"}

# /rpp/debug param-snapshot slots -> declared parameter names (subset that
# maps 1:1; used to restore the AS-RUN configuration onto the replay node).
DBG_PARAM_SLOTS = {
    11: "max_linear_vel", 12: "min_linear_vel", 13: "min_lookahead_dist",
    14: "max_lookahead_dist", 15: "lookahead_time", 16: "a_lat_max",
    20: "approach_velocity_scaling_dist", 35: "max_yaw_rate_body",
}


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
    msg.pose.position.x = east
    msg.pose.position.y = north
    msg.pose.position.z = 0.0
    yaw_enu = math.pi / 2.0 - yaw_ned
    half = yaw_enu / 2.0
    msg.pose.orientation.w = math.cos(half)
    msg.pose.orientation.z = math.sin(half)
    return msg


class _BagClock:
    """Node-clock stand-in driven by BAG time.

    The controller's accel ramps (`_tick_dt`), alignment settle timers,
    pose-age checks and yaw-rate stillness gates all read
    `self.get_clock().now()`. A replay that ticks at wall speed makes those
    read ~2 ms per tick instead of the bag's ~33 ms — the speed ramp then
    never rises (cmd pinned near zero) and `_run_alignment_hold` never
    settles. Stepping this clock to each event's bag timestamp restores the
    node's real time base.
    """

    def __init__(self):
        self._ns = 0

    def step_to(self, t_epoch_s):
        self._ns = int(t_epoch_s * 1e9)

    def now(self):
        from rclpy.time import Time
        return Time(nanoseconds=self._ns)


class _Cap:
    """Publisher stand-in that records (tick_time, msg)."""

    def __init__(self):
        self.messages = []
        self.now = 0.0

    def publish(self, msg):
        self.messages.append((self.now, msg))

    @property
    def last(self):
        return self.messages[-1][1] if self.messages else None


def _collect_inputs(am, bag):
    """Merge every replay input into one time-ordered event list."""
    events = []          # (t, kind, payload)
    for topic, m, t in am.read_bag(bag):
        if topic == "/mavros/local_position/pose":
            events.append((t, "pose", m))
        elif topic == "/mavros/local_position/velocity_local":
            events.append((t, "vel", (m["lx"], m["ly"])))
        elif topic == "/path":
            pts = [(p[0], p[1], (p[2] if len(p) > 2 else 0.0))
                   for p in m["poses"]]
            events.append((t, "path", pts))
        elif topic.endswith("gps1/raw"):
            events.append((t, "gps", m.get("fix_type", 6)))
    events.sort(key=lambda e: e[0])
    return events


def replay(bundle, t_from=None, t_to=None, print_every=10,
           param_overrides=(), auto_params=True, csv_out=None):
    am = _load_am()
    bag, _manifest = am._find_bag_dir(bundle)
    s = am.collect(bag)
    if not s.pose:
        print("no poses in bundle"); return 1
    events = _collect_inputs(am, bag)
    n_paths = sum(1 for e in events if e[1] == "path")
    print("bundle : %s" % bundle)
    print("inputs : %d events (%d /path msgs, %d poses, %d vel, %d gps)"
          % (len(events), n_paths,
             sum(1 for e in events if e[1] == "pose"),
             sum(1 for e in events if e[1] == "vel"),
             sum(1 for e in events if e[1] == "gps")))

    # as-run params from the bag's own debug stream (median of each slot)
    as_run = {}
    if auto_params and s.rpp:
        mid = s.rpp[len(s.rpp) // 2][1]
        for slot, name in DBG_PARAM_SLOTS.items():
            if len(mid) > slot and mid[slot] == mid[slot]:
                as_run[name] = float(mid[slot])
        cmds = [r[1][3] for r in s.rpp if len(r[1]) > 3 and r[1][3] == r[1][3]]
        if cmds:
            as_run["mission_speed"] = round(max(cmds), 2)   # heuristic
    # bag's own commanded speed, for the fidelity check
    bag_cmd = [(r[0], r[1][3]) for r in s.rpp
               if len(r[1]) > 3 and r[1][3] == r[1][3]]
    bag_cmd_t = [b[0] for b in bag_cmd]

    import rclpy
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    try:
        from rclpy.parameter import Parameter
        from rpp_controller_node import RPPControllerNode
        from nav_msgs.msg import Path
        from mavros_msgs.msg import GPSRAW
        from geometry_msgs.msg import TwistStamped
        from test_smoke_rpp_controller import _make_path_pose

        node = RPPControllerNode()
        clock = _BagClock()
        clock.step_to(events[0][0] if events else 0.0)
        node.get_clock = lambda: clock          # bag-time base for the node
        caps = {}
        for attr in ("_vel_pub", "_yaw_rate_pub", "_dbg_pub",
                     "_segment_dbg_pub", "_conditioned_path_pub",
                     "_spray_active_pub", "_progress_pub", "_milestone_pub"):
            if hasattr(node, attr):
                caps[attr] = _Cap()
                setattr(node, attr, caps[attr])

        applied = {}
        for name, val in as_run.items():
            try:
                node.set_parameters([Parameter(name, value=val)])
                applied[name] = val
            except Exception:
                pass
        for ov in param_overrides:
            name, _, raw = ov.partition(":=")
            val = (raw.lower() == "true") if raw.lower() in ("true", "false") \
                else (float(raw) if any(c in raw for c in ".0123456789") else raw)
            node.set_parameters([Parameter(name.strip(), value=val)])
            applied[name.strip()] = val
        if applied:
            print("params : %s" % ", ".join(
                "%s=%s" % kv for kv in sorted(applied.items())))

        gps0 = GPSRAW(); gps0.fix_type = 6
        node._gps_cb(gps0)

        t0 = s.pose[0][0]
        rows = []            # per-tick record
        timeline = []        # (t_rel, text)
        prev = {"run": None, "seg": None, "state": None}
        first_backward = None
        state_flips = 0
        printed = 0
        tick = 0

        def note(rel, text):
            timeline.append((rel, text))

        for (t, kind, payload) in events:
            rel = t - t0
            if t_to is not None and rel > t_to:
                break
            clock.step_to(t)
            if kind == "path":
                mark_bits = [int(round(z)) for (_n, _e, z) in payload]
                msg = Path()
                msg.header.frame_id = "local_ned"
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.poses = [_make_path_pose(n, e, mark=bool(int(round(z)) & 1))
                             for (n, e, z) in payload]
                for ps, z in zip(msg.poses, mark_bits):
                    ps.pose.position.z = float(z)
                node._path_cb(msg)
                kind_txt = ("E-STOP sentinel" if len(payload) == 1 else
                            "path %d pts (%d runs installed, active run %s)"
                            % (len(payload),
                               len(getattr(node, "_runs", []) or []),
                               getattr(node, "_run_idx", -1)))
                note(rel, "PATH   " + kind_txt)
                continue
            if kind == "vel":
                tw = TwistStamped()
                tw.twist.linear.x = float(payload[0])
                tw.twist.linear.y = float(payload[1])
                node._vel_cb(tw)
                continue
            if kind == "gps":
                g = GPSRAW(); g.fix_type = int(payload)
                node._gps_cb(g)
                continue

            # pose -> one control tick
            if t_from is not None and rel < t_from:
                continue
            # read_bag gives the raw msg dict; derive NED exactly like collect()
            n, e = payload["y"], payload["x"]
            yaw = am._yaw_ned_from_quat(payload["qx"], payload["qy"],
                                        payload["qz"], payload["qw"])
            for c in caps.values():
                c.now = rel
            node._pose_cb(_enu_pose(n, e, yaw))
            node._control_loop()
            tick += 1

            run_i = getattr(node, "_run_idx", -1)
            seg = getattr(node, "_segment_idx", -1)
            st = int(getattr(node, "_segment_state", -1))
            remain = node._run_remaining_along()
            tw = caps["_vel_pub"].last if "_vel_pub" in caps else None
            if tw is None:
                cmd_v = float("nan")
            elif hasattr(tw, "vector"):          # Vector3Stamped (/rpp/velocity_ned)
                cmd_v = math.hypot(tw.vector.x, tw.vector.y)
            else:                                 # TwistStamped fallback
                cmd_v = math.hypot(tw.twist.linear.x, tw.twist.linear.y)
            dbg = caps["_dbg_pub"].last if "_dbg_pub" in caps else None
            data = list(getattr(dbg, "data", []) or [])
            xt = data[0] if len(data) > 0 else float("nan")
            ld = data[2] if len(data) > 2 else float("nan")
            meas = getattr(node, "_measured_speed", None)
            meas_v = meas() if callable(meas) else float("nan")

            rows.append(dict(t=rel, run=run_i, seg=seg, state=st,
                             cmd_v=cmd_v, meas_v=meas_v, xtrack=xt, ld=ld,
                             remain=(float("nan") if remain is None else remain),
                             n=n, e=e))

            if prev["run"] is not None and run_i != prev["run"]:
                note(rel, "RUN    %s -> %s" % (prev["run"], run_i))
            if prev["state"] is not None and st != prev["state"]:
                note(rel, "STATE  %s -> %s"
                     % (SEG_STATE_NAMES.get(prev["state"], prev["state"]),
                        SEG_STATE_NAMES.get(st, st)))
                state_flips += 1
            if prev["seg"] is not None and seg < prev["seg"] \
                    and run_i == prev["run"] and first_backward is None:
                first_backward = (rel, prev["seg"], seg)
            prev.update(run=run_i, seg=seg, state=st)

            if tick % print_every == 0 and printed < 400:
                print("%8.2f run%-2s seg%-3s %-11s cmd=%.3f meas=%.3f "
                      "xt=%+.3f Ld=%.2f rem=%s"
                      % (rel, run_i, seg, SEG_STATE_NAMES.get(st, st),
                         cmd_v, meas_v, xt, ld,
                         "-" if remain is None else "%.3f" % remain))
                printed += 1

        # ------------------------------------------------------------------
        print("\n=== TIMELINE ===")
        for rel, text in timeline:
            print("  %8.2f  %s" % (rel, text))

        print("\n=== DETECTORS ===")
        issues = []

        # 1. STRAND: dead-band command while stopped and distance remains
        start = None
        for r in rows:
            in_band = (1e-3 < r["cmd_v"] < RO_SPEED_TH
                       and r["meas_v"] < STOPPED_V
                       and (r["remain"] != r["remain"] or r["remain"] > 0.05))
            if in_band and start is None:
                start = r
            elif not in_band and start is not None:
                dur = r["t"] - start["t"]
                if dur >= STRAND_MIN_S:
                    issues.append(
                        "STRAND  t=%.1f..%.1f (%.1fs) cmd=%.3f<RO_SPEED_TH "
                        "meas=%.3f remain=%.3f state=%s"
                        % (start["t"], r["t"], dur, start["cmd_v"],
                           start["meas_v"], start["remain"],
                           SEG_STATE_NAMES.get(start["state"])))
                start = None
        if start is not None and rows and rows[-1]["t"] - start["t"] >= STRAND_MIN_S:
            issues.append(
                "STRAND  t=%.1f..end (%.1fs) cmd=%.3f<RO_SPEED_TH meas=%.3f "
                "remain=%.3f state=%s  [never recovered]"
                % (start["t"], rows[-1]["t"] - start["t"], start["cmd_v"],
                   start["meas_v"], start["remain"],
                   SEG_STATE_NAMES.get(start["state"])))

        # 2. NO-STOP at run switches and corner stops
        ts = [r["t"] for r in rows]
        def min_meas(t_lo, t_hi):
            i0 = bisect.bisect_left(ts, t_lo); i1 = bisect.bisect_right(ts, t_hi)
            win = rows[i0:i1]
            return min((r["meas_v"] for r in win), default=float("nan"))
        for rel, text in timeline:
            if text.startswith("RUN "):
                mv = min_meas(rel - 1.2, rel + 1.2)
                if mv == mv and mv > NOSTOP_V:
                    issues.append(
                        "NO-STOP run switch at t=%.1f: min measured speed "
                        "%.3f m/s in +/-1.2 s — rover never stopped "
                        "(pre-start/no-stop bug signature)" % (rel, mv))
        in5 = None
        for r in rows:
            if r["state"] == 5 and in5 is None:
                in5 = r["t"]
            elif r["state"] != 5 and in5 is not None:
                mv = min_meas(in5, r["t"])
                if mv == mv and mv > STOPPED_V + 0.02:
                    issues.append(
                        "CORNER RUN-THROUGH t=%.1f..%.1f: CORNER_STOP held "
                        "but min measured speed %.3f m/s" % (in5, r["t"], mv))
                in5 = None

        # 3. structure
        if first_backward:
            issues.append("BACKWARD SEG INDEX t=%.2f  %d -> %d (corner "
                          "deadlock signature)" % first_backward)
        if state_flips > 8 * max(1, n_paths):
            issues.append("STATE OSCILLATION: %d segment-state transitions "
                          "(expect a handful per run)" % state_flips)

        # 4. lookahead collapse
        min_ld = applied.get("min_lookahead_dist") or 0.35
        low = [r for r in rows if r["ld"] == r["ld"] and 0 < r["ld"] < 0.5 * min_ld]
        if len(low) > 15:
            issues.append("LOOKAHEAD COLLAPSE: Ld < %.2f (half the floor) on "
                          "%d ticks, first at t=%.1f"
                          % (0.5 * min_ld, len(low), low[0]["t"]))

        # 5. fidelity vs the bag's own commands
        if bag_cmd:
            deltas = []
            for r in rows:
                j = bisect.bisect_left(bag_cmd_t, r["t"] + t0)
                j = max(0, min(j, len(bag_cmd) - 1))
                if abs(bag_cmd[j][0] - (r["t"] + t0)) < 0.06 and r["cmd_v"] == r["cmd_v"]:
                    deltas.append(r["cmd_v"] - bag_cmd[j][1])
            if deltas:
                rms = math.sqrt(sum(d * d for d in deltas) / len(deltas))
                mx = max(abs(d) for d in deltas)
                print("fidelity: replay cmd vs bag /rpp/debug[3]: "
                      "rms=%.3f max=%.3f m/s over %d matched ticks"
                      % (rms, mx, len(deltas)))
                if rms > 0.05:
                    issues.append("FIDELITY: replay diverges from the field "
                                  "run (cmd rms delta %.3f m/s) — as-run "
                                  "params differ; conclusions unsafe" % rms)

        if issues:
            for msg in issues:
                print("  !! %s" % msg)
        else:
            print("  clean — no anomalies detected")

        # coverage + end state
        moved = [r for r in rows if r["meas_v"] == r["meas_v"] and r["meas_v"] > 0.05]
        print("\nend    : run %s seg %s state %s | ticks %d | moving %.0f%% "
              "of ticks | final remain %s"
              % (prev["run"], prev["seg"],
                 SEG_STATE_NAMES.get(prev["state"], prev["state"]), tick,
                 100.0 * len(moved) / max(1, len(rows)),
                 "-" if not rows or rows[-1]["remain"] != rows[-1]["remain"]
                 else "%.3f" % rows[-1]["remain"]))

        if csv_out:
            with open(csv_out, "w", newline="") as fh:
                w = _csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
            print("csv    : %s (%d rows)" % (csv_out, len(rows)))
        return 0
    finally:
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bundle")
    ap.add_argument("--from", dest="t_from", type=float, default=None)
    ap.add_argument("--to", dest="t_to", type=float, default=None)
    ap.add_argument("--print-every", type=int, default=10,
                    help="decimate PRINTING only; every pose is always ticked")
    ap.add_argument("--param", action="append", default=[],
                    metavar="name:=value",
                    help="override a controller parameter (repeatable) — "
                         "test a candidate fix against a recorded failure")
    ap.add_argument("--no-auto-params", action="store_true",
                    help="do not restore the bag's as-run param snapshot")
    ap.add_argument("--csv", dest="csv_out", default=None)
    a = ap.parse_args()
    raise SystemExit(replay(a.bundle, a.t_from, a.t_to,
                            max(1, a.print_every), a.param,
                            not a.no_auto_params, a.csv_out))


if __name__ == "__main__":
    main()
