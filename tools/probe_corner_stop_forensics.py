#!/usr/bin/env python3
"""Forensic probe: corner-stop behavior from mission-debug bags."""
from __future__ import annotations

import json
import math
import sqlite3
import sys
from pathlib import Path

import numpy as np
from rosbags.typesys import Stores, get_typestore

SEG = {0: "INACTIVE", 1: "TRACK", 2: "PRE_SLOW", 3: "ALIGN", 4: "DONE", 5: "CORNER_STOP"}
RPP = {-1: "STALE", 0: "IDLE", 1: "TRACKING", 2: "RTK_WAIT", 3: "DONE", 4: "JUMP_SKIP"}


def load_bag(bundle: Path):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    db = list(bundle.rglob("*.db3"))[0]
    con = sqlite3.connect(db)
    topics = {r[1]: (r[0], r[2]) for r in con.execute("select id,name,type from topics")}

    def read(topic):
        if topic not in topics:
            return []
        tid, typ = topics[topic][0], topics[topic][1]
        return [
            (t * 1e-9, ts.deserialize_cdr(d, typ))
            for t, d in con.execute(
                "select timestamp,data from messages where topic_id=? order by timestamp", (tid,)
            )
        ]

    return {
        "pose": read("/mavros/local_position/pose"),
        "vel": read("/mavros/local_position/velocity_local"),
        "rpp": read("/rpp/debug"),
        "seg": read("/rpp/segment_debug"),
        "path": read("/path"),
        "path_id": read("/path/identity"),
    }


def mission_meta(bundle: Path) -> dict:
    out = {}
    for name in ("manifest.json", "mission/staged.json", "mission/loaded_path.json", "mission/source.json"):
        p = bundle / name
        if p.exists():
            out[name] = json.loads(p.read_text())
    ext = bundle / "mission/sidecars/.square_2x2.dxf.extensions.json"
    if ext.exists():
        out["extensions"] = json.loads(ext.read_text())
    return out


def speed_at(pose_rows, vel_rows, t):
    # nearest velocity sample
    if not vel_rows:
        return float("nan")
    times = np.array([r[0] for r in vel_rows])
    i = int(np.argmin(np.abs(times - t)))
    v = vel_rows[i][1].twist.linear
    return math.hypot(v.x, v.y)


def analyze_corner_stops(data: dict, t_mission_start: float | None = None) -> list[dict]:
    seg = data["seg"]
    if not seg:
        return []
    events = []
    prev_state = None
    in_stop = False
    stop_start = None
    stop_samples = []

    for t, msg in seg:
        st = int(msg.data[1])
        if prev_state != 5 and st == 5:
            in_stop = True
            stop_start = t
            stop_samples = []
        if in_stop and st == 5:
            stop_samples.append({
                "t": t - (t_mission_start or seg[0][0]),
                "dist_seg_end_m": msg.data[3],
                "dist_corner_m": msg.data[4],
                "seg_idx": int(msg.data[2]),
                "heading_err_deg": math.degrees(msg.data[7]),
                "yaw_rate_cmd": msg.data[8],
                "yaw_rate_act": msg.data[9],
                "speed_mps": speed_at(data["pose"], data["vel"], t),
            })
        if prev_state == 5 and st != 5:
            in_stop = False
            if stop_samples:
                mn = min(stop_samples, key=lambda s: s["dist_corner_m"])
                events.append({
                    "seg_idx": stop_samples[0]["seg_idx"],
                    "duration_s": float(stop_samples[-1]["t"] - stop_samples[0]["t"]),
                    "min_dist_corner_m": float(mn["dist_corner_m"]),
                    "min_dist_corner_at_s": float(mn["t"]),
                    "min_speed_mps": float(min(s["speed_mps"] for s in stop_samples)),
                    "final_speed_mps": float(stop_samples[-1]["speed_mps"]),
                    "exit_state": SEG.get(st, st),
                    "samples": len(stop_samples),
                    "first_dist_corner_m": float(stop_samples[0]["dist_corner_m"]),
                })
        prev_state = st
    return events


def first_stale(data: dict, t0: float) -> dict | None:
    for t, msg in data["rpp"]:
        if int(msg.data[7]) == -1:
            return {"t_s": t - t0, "xtrack_cm": msg.data[0] * 100, "speed_cmd": msg.data[3]}
    return None


def path_publishes(data: dict) -> list[dict]:
    out = []
    for i, (t, msg) in enumerate(data["path"]):
        n = len(msg.poses)
        if i == 0 or n != out[-1]["n"]:
            out.append({"idx": i, "t": t, "n": n})
    return out[:8]


def probe(bundle: Path) -> dict:
    meta = mission_meta(bundle)
    manifest = meta.get("manifest.json", {})
    data = load_bag(bundle)
    t0 = data["seg"][0][0] if data["seg"] else (data["rpp"][0][0] if data["rpp"] else 0)
    corners = analyze_corner_stops(data, t0)
    stale = first_stale(data, t0)

    # tracking quality during first TRACK segment
    xtk = [m.data[0] * 100 for _, m in data["rpp"] if int(m.data[7]) == 1]
    xtk_arr = np.array(xtk) if xtk else np.array([])

    staged = meta.get("mission/staged.json", {})
    loaded = meta.get("mission/loaded_path.json", {})
    placement = manifest.get("placement", {})

    return {
        "bag": bundle.name,
        "stop_reason": manifest.get("outcome", {}).get("stop_reason"),
        "commit": manifest.get("software", {}).get("git_commit", "")[:7],
        "entry_distance_m": placement.get("entry_distance_m"),
        "entry_transit": placement.get("entry_transit_added"),
        "extensions": meta.get("extensions"),
        "point_execution_mode": staged.get("point_execution_mode"),
        "point_leg_trajectory_mode": staged.get("point_leg_trajectory_mode"),
        "num_waypoints": loaded.get("num_waypoints"),
        "num_transit": loaded.get("num_transit"),
        "path_publishes": path_publishes(data),
        "corner_stops": corners[:6],
        "first_stale": stale,
        "xtrack_tracking_cm_rms": float(np.sqrt(np.mean(xtk_arr**2))) if xtk_arr.size else None,
        "xtrack_tracking_cm_max": float(np.max(np.abs(xtk_arr))) if xtk_arr.size else None,
    }


def main():
    bags = sys.argv[1:]
    if not bags:
        print("usage: probe_corner_stop_forensics.py <bundle_dir> ...")
        return 1
    for b in bags:
        p = Path(b)
        r = probe(p)
        print(json.dumps(r, indent=2))
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
