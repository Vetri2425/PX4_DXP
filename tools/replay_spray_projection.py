#!/usr/bin/env python3
"""Offline replay of the spray path projection, with a fidelity gate.

WHY A FIDELITY GATE. A replay that does not reproduce the recorded behaviour
proves nothing about a candidate fix. The first attempt at this harness walked
the 30 Hz pose track instead of the 48 Hz nozzle feed the node actually uses,
and scored a run 330 cm late that the bag shows painting 2 cm early. The
control arm was wrong, so the treatment arm was meaningless.

This harness therefore replays the EXACT input the node fed the projector --
the nozzle position recorded in /spray/debug[2],[3] -- threads prev_s the way
the node does (reset on every /path, stored back every tick), and checks the
recomputed station against the recorded station in /spray/debug[4]. Only if
that residual is small is the A arm trusted; the candidate fix is then run on
the same input and the two compared.

  A arm  gate 0 deg  == shipped behaviour == must reproduce /spray/debug[4]
  B arm  gate N deg  == candidate         == judged on where the MARK flag fires

/spray/debug slot map (continuous mode, from decide()):
  [0] model present   [1] speed   [2] nozzle_n   [3] nozzle_e
  [4] projection.s    [5] xtrack  [6] current_flag (raw geometry MARK flag)
  [7] boundary.s      [8] dist_to_boundary        [9] geometry_desired
  [10] safety_ok      [11] desired

Usage
-----
    PYTHONPATH=src python3 tools/replay_spray_projection.py <bundle> [...]
    PYTHONPATH=src python3 tools/replay_spray_projection.py --gate 90 bags/x/*/

Needs the spray node importable (PYTHONPATH=src) but NOT rclpy-free: run it in
the ros-replay env, e.g.
    PYTHONPATH=src micromamba run -p ~/mamba/envs/ros-replay python tools/...
"""
from __future__ import annotations

import argparse
import bisect
import glob
import importlib.util
import math
import os
import sys

FIDELITY_TOL_M = 0.01          # A arm must track the recorded station to 1 cm


def _load(path: str, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


_here = os.path.dirname(os.path.abspath(__file__))
_root = os.path.dirname(_here)
am = _load(os.path.join(_root, "tools", "analyze_mission.py"), "_am")
sc = _load(os.path.join(_root, "src", "spray_controller_node.py"), "_sc")


class Events:
    """Time-ordered stream of the three inputs the projector depends on."""

    def __init__(self, bag):
        self.paths: list[tuple[float, list]] = []
        self.debug: list[tuple[float, list]] = []
        self.pose: list[tuple[float, float]] = []      # (t, yaw_ned)
        for topic, msg, t in am.read_bag(bag):
            if topic == "/path":
                self.paths.append((t, msg["poses"]))
            elif topic == "/spray/debug":
                self.debug.append((t, msg["data"]))
            elif topic == "/mavros/local_position/pose":
                pass
        self.paths.sort(key=lambda x: x[0])
        self.debug.sort(key=lambda x: x[0])
        s = am.collect(bag)
        self.pose = sorted((p[0], p[3]) for p in s.pose)
        self._pt = [p[0] for p in self.pose]

    def yaw_at(self, t: float) -> float:
        """Nearest recorded yaw. Pose is 30 Hz, debug 48 Hz, so nearest is
        within ~17 ms -- under 0.5 deg even at full pivot rate."""
        if not self.pose:
            return float("nan")
        i = bisect.bisect_left(self._pt, t)
        cands = [j for j in (i - 1, i) if 0 <= j < len(self.pose)]
        return min(cands, key=lambda j: abs(self._pt[j] - t)) and self.pose[
            min(cands, key=lambda j: abs(self._pt[j] - t))
        ][1]


def _model_from_path(poses) -> tuple:
    pts = [(p[0], p[1]) for p in poses]
    flags = [bool(int(round(p[2])) & 1) for p in poses]
    model = sc._build_path_model(pts, flags)
    s0 = next(
        (b.s for b in model.boundaries if b.kind == sc.TRANSIT_TO_MARK), None
    )
    return model, s0


def replay(bundle: str, gate_deg: float, win=(0.5, 2.0, 1.0)):
    bag, _ = am._find_bag_dir(bundle)
    ev = Events(bag)
    if not ev.debug or not ev.paths:
        return None
    cos_gate = sc._direction_gate_cos(gate_deg)
    back, fwd, reacq = win

    model = None
    s0 = None
    prev_s = None
    first_flag_s = None
    resid: list[float] = []
    flag_mismatch = 0
    n = 0
    pi = 0
    for t, d in ev.debug:
        # Install any /path that became current at or before this tick, and
        # reset the station exactly as _path_cb does.
        while pi < len(ev.paths) and ev.paths[pi][0] <= t:
            model, s0 = _model_from_path(ev.paths[pi][1])
            prev_s = None
            first_flag_s = None
            pi += 1
        if model is None or len(d) < 7:
            continue
        nz_n, nz_e = d[2], d[3]
        if not (math.isfinite(nz_n) and math.isfinite(nz_e)):
            continue
        pr = sc._project_onto_path(
            model, nz_n, nz_e,
            prev_s=prev_s, window_back_m=back, window_fwd_m=fwd,
            reacquire_dist_m=reacq,
            heading_rad=ev.yaw_at(t), direction_gate_cos=cos_gate,
        )
        if pr is None:
            continue
        prev_s = pr.s
        n += 1
        if math.isfinite(d[4]):
            resid.append(abs(pr.s - d[4]))
            if bool(pr.current_flag) != bool(d[6] > 0.5):
                flag_mismatch += 1
        if first_flag_s is None and pr.current_flag:
            first_flag_s = pr.s
    if not n:
        return None
    rms = math.sqrt(sum(r * r for r in resid) / len(resid)) if resid else float("nan")
    return dict(
        n=n, s0=s0, first_flag_s=first_flag_s,
        err_cm=(first_flag_s - s0) * 100
        if (first_flag_s is not None and s0 is not None) else float("nan"),
        fid_rms=rms, fid_max=max(resid) if resid else float("nan"),
        flag_mismatch_pct=100.0 * flag_mismatch / len(resid) if resid else float("nan"),
        recorded_first_flag_cm=None,
    )


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bundles", nargs="+")
    ap.add_argument("--gate", type=float, default=90.0,
                    help="candidate direction gate half-angle, deg (default 90)")
    a = ap.parse_args(argv)

    items = []
    for pat in a.bundles:
        items.extend(sorted(glob.glob(pat)) if any(c in pat for c in "*?[") else [pat])

    print(f"{'run':<8}{'fidRMS':>9}{'fidMax':>9}{'flagErr%':>10}"
          f"{'A_late_cm':>11}{'B_late_cm':>11}{'verdict':>10}")
    print("-" * 68)
    bad_fid = 0
    rows = []
    for b in items:
        try:
            A = replay(b, 0.0)
            B = replay(b, a.gate)
        except Exception as exc:                       # noqa: BLE001
            print(f"{os.path.basename(b.rstrip('/'))[-6:]:<8} SKIP {exc}")
            continue
        if A is None or B is None:
            continue
        lbl = os.path.basename(b.rstrip("/"))[-6:]
        ok = A["fid_rms"] <= FIDELITY_TOL_M and A["flag_mismatch_pct"] < 1.0
        if not ok:
            bad_fid += 1
        print(f"{lbl:<8}{A['fid_rms']*100:>9.3f}{A['fid_max']*100:>9.2f}"
              f"{A['flag_mismatch_pct']:>10.2f}{A['err_cm']:>11.1f}"
              f"{B['err_cm']:>11.1f}{'ok' if ok else 'UNFAITHFUL':>10}")
        rows.append((lbl, ok, A["err_cm"], B["err_cm"]))

    print()
    if bad_fid:
        print(f"*** {bad_fid} of {len(rows)} runs FAILED the fidelity gate "
              f"(A arm must track /spray/debug[4] to {FIDELITY_TOL_M*100:.0f} cm "
              f"and agree on the MARK flag).")
        print("*** The B arm on those runs proves NOTHING. Fix the harness first.")
        return 1
    good = [r for r in rows if r[1]]
    lateA = [r[2] for r in good if not math.isnan(r[2])]
    lateB = [r[3] for r in good if not math.isnan(r[3])]
    print(f"fidelity OK on {len(good)}/{len(rows)} runs")
    if lateA:
        print(f"  A (shipped)   worst {max(lateA):+.1f} cm   "
              f">15 cm late: {sum(1 for x in lateA if x > 15)}")
        print(f"  B (gate {a.gate:.0f} deg) worst {max(lateB):+.1f} cm   "
              f">15 cm late: {sum(1 for x in lateB if x > 15)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
