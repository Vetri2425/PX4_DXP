#!/usr/bin/env python3
"""Quantify CORNER_STOP drift vs MARK entry cross-track for per-line extension bags."""

from __future__ import annotations

import json
import math
import sqlite3
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

SEGMENT_STATES = {
    0: "INACTIVE",
    1: "TRACK_SEGMENT",
    2: "PRE_CORNER_SLOWDOWN",
    3: "CORNER_ALIGN",
    4: "DONE",
    5: "CORNER_STOP",
}


class CDR:
    def __init__(self, raw: bytes):
        self.raw = bytes(raw)
        self.base = 4
        self.off = 4

    def align(self, size: int) -> None:
        rel = self.off - self.base
        self.off = self.base + ((rel + size - 1) // size) * size

    def unpack(self, fmt: str, size: int, alignment: int | None = None):
        self.align(alignment or size)
        value = struct.unpack_from("<" + fmt, self.raw, self.off)[0]
        self.off += size
        return value

    def u8(self):
        return self.unpack("B", 1)

    def u32(self):
        return self.unpack("I", 4)

    def i32(self):
        return self.unpack("i", 4)

    def f32(self):
        return self.unpack("f", 4)

    def f64(self):
        return self.unpack("d", 8)

    def string(self) -> str:
        n = self.u32()
        value = self.raw[self.off : self.off + n].rstrip(b"\0").decode("utf-8", "replace")
        self.off += n
        return value

    def header(self) -> tuple[int, int, str]:
        return self.i32(), self.u32(), self.string()


def parse_multi(raw: bytes) -> list[float]:
    c = CDR(raw)
    dim_n = c.u32()
    for _ in range(dim_n):
        c.string()
        c.u32()
        c.u32()
    c.u32()
    n = c.u32()
    return [c.f32() for _ in range(n)]


def parse_path(raw: bytes) -> list[dict]:
    c = CDR(raw)
    c.header()
    count = c.u32()
    points = []
    for idx in range(count):
        c.header()
        n, e, z = c.f64(), c.f64(), c.f64()
        c.f64(), c.f64(), c.f64(), c.f64()
        points.append({"idx": idx, "n": n, "e": e, "z": z, "spray": z > 0.5})
    return points


def parse_pose(raw: bytes) -> dict:
    c = CDR(raw)
    c.header()
    e, n, u = c.f64(), c.f64(), c.f64()
    qx, qy, qz, qw = c.f64(), c.f64(), c.f64(), c.f64()
    yaw_enu = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return {"n": n, "e": e, "yaw_ned": wrap(math.pi / 2 - yaw_enu)}


def parse_twist(raw: bytes) -> dict:
    c = CDR(raw)
    c.header()
    x, y, z = c.f64(), c.f64(), c.f64()
    c.f64(), c.f64(), c.f64()
    return {"speed": math.hypot(x, y), "body_x": x, "body_y": y}


def parse_bool(raw: bytes) -> dict:
    return {"value": bool(CDR(raw).u8())}


def wrap(value: float) -> float:
    return (value + math.pi) % (2 * math.pi) - math.pi


def heading(a: dict, b: dict) -> float:
    return math.atan2(b["e"] - a["e"], b["n"] - a["n"])


def distance(a: dict, b: dict) -> float:
    return math.hypot(b["n"] - a["n"], b["e"] - a["e"])


def runs_by_flag(points: list[dict]) -> list[dict]:
    runs = []
    start = 0
    for i in range(1, len(points) + 1):
        if i == len(points) or points[i]["spray"] != points[start]["spray"]:
            pts = points[start:i]
            runs.append(
                {
                    "run": len(runs),
                    "spray": pts[0]["spray"],
                    "points": pts,
                    "length": sum(distance(a, b) for a, b in zip(pts[:-1], pts[1:])),
                }
            )
            start = i
    return runs


def mark_lines(points: list[dict]) -> list[dict]:
    flag_runs = runs_by_flag(points)
    mark_runs = [r for r in flag_runs if r["spray"]]
    lines = []
    for li, mark in enumerate(mark_runs, 1):
        mpts = mark["points"]
        h = heading(mpts[0], mpts[-1])
        before = flag_runs[flag_runs.index(mark) - 1] if flag_runs.index(mark) > 0 else None
        pre_start = before["points"][0] if before else mpts[0]
        lines.append(
            {
                "line": li,
                "mark_start": mpts[0],
                "mark_end": mpts[-1],
                "pre_start": pre_start,
                "heading_rad": h,
                "mark_length_m": mark["length"],
            }
        )
    return lines


def load_topic(con, topic, parser) -> pd.DataFrame:
    topic_id = con.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()[0]
    records = []
    for ts, raw in con.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_id,),
    ):
        record = {"ts_ns": int(ts)}
        record.update(parser(raw))
        records.append(record)
    return pd.DataFrame(records)


def nearest_indices(source_ns: np.ndarray, target_ns: np.ndarray) -> np.ndarray:
    idx = np.searchsorted(source_ns, target_ns)
    idx = np.clip(idx, 1, len(source_ns) - 1)
    left = idx - 1
    choose_left = target_ns - source_ns[left] <= source_ns[idx] - target_ns
    return np.where(choose_left, left, idx)


def line_metrics(df: pd.DataFrame, line: dict) -> pd.DataFrame:
    h = line["heading_rad"]
    dn = df["n"] - line["mark_start"]["n"]
    de = df["e"] - line["mark_start"]["e"]
    out = df.copy()
    out["along_m"] = dn * math.cos(h) + de * math.sin(h)
    out["xtrack_m"] = -dn * math.sin(h) + de * math.cos(h)
    return out


def pos_error(row, target: dict) -> float:
    return math.hypot(row.n - target["n"], row.e - target["e"])


def along_from_pre(row, line: dict) -> float:
    return (row.n - line["pre_start"]["n"]) * math.cos(line["heading_rad"]) + (
        row.e - line["pre_start"]["e"]
    ) * math.sin(line["heading_rad"])


def analyze_bag(bag_dir: Path) -> dict:
    db_files = list(bag_dir.glob("*.db3"))
    if not db_files:
        raise FileNotFoundError(f"No .db3 in {bag_dir}")
    bag = db_files[0]
    con = sqlite3.connect(bag)
    raw_path = con.execute(
        "SELECT data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name='/path')"
    ).fetchone()[0]
    points = parse_path(raw_path)
    lines = mark_lines(points)

    debug = load_topic(con, "/rpp/debug", lambda raw: {f"d{i}": v for i, v in enumerate(parse_multi(raw))})
    seg = load_topic(
        con,
        "/rpp/segment_debug",
        lambda raw: {f"s{i}": v for i, v in enumerate(parse_multi(raw))},
    )
    pose = load_topic(con, "/mavros/local_position/pose", parse_pose)
    vel_local = load_topic(con, "/mavros/local_position/velocity_local", parse_twist)
    vel_body = load_topic(con, "/mavros/local_position/velocity_body", parse_twist)
    spray = load_topic(con, "/spray/active", parse_bool)
    con.close()

    t0 = int(min(debug.ts_ns.min(), pose.ts_ns.min()))
    master = debug.rename(
        columns={"d0": "rpp_xtrack", "d3": "speed_cmd", "d10": "yaw_rate_cmd"}
    ).copy()
    joins = [
        (
            seg,
            {
                "s1": "segment_state",
                "s2": "segment_idx",
                "s7": "segment_heading_err",
                "s9": "actual_yaw_rate",
            },
        ),
        (pose, {"n": "n", "e": "e", "yaw_ned": "yaw_ned"}),
        (vel_local, {"speed": "actual_speed"}),
        (vel_body, {"body_x": "body_forward_speed", "body_y": "body_lateral_speed"}),
        (spray, {"value": "spray_active"}),
    ]
    target = master.ts_ns.to_numpy()
    for source, mapping in joins:
        idx = nearest_indices(source.ts_ns.to_numpy(), target)
        for src, dst in mapping.items():
            master[dst] = source.iloc[idx][src].to_numpy()
    master["segment_state_name"] = master.segment_state.round().astype(int).map(SEGMENT_STATES)

    spray_edges = master.spray_active.astype(int).diff().fillna(master.spray_active.astype(int))
    on_idx = list(master.index[spray_edges == 1])
    off_idx = list(master.index[spray_edges == -1])
    if master.spray_active.iloc[-1]:
        off_idx.append(master.index[-1])

    corners = []
    for line in lines:
        oi = on_idx[line["line"] - 1]
        search_start = 0 if line["line"] == 1 else on_idx[line["line"] - 2]
        pre = master.loc[search_start:oi - 1]
        stop_rows = pre[pre.segment_state_name == "CORNER_STOP"]
        if stop_rows.empty:
            continue
        episodes = (stop_rows.index.to_series().diff().fillna(1) > 1).cumsum()
        ep = stop_rows.loc[episodes[episodes == episodes.iloc[-1]].index]

        stop_start = ep.iloc[0]
        stop_end = ep.iloc[-1]
        entry_pos = {"n": float(stop_start.n), "e": float(stop_start.e)}
        stopped_mask = ep.actual_speed < 0.02
        if stopped_mask.any():
            first_stopped = ep.loc[stopped_mask.idxmax()]
            stop_time_s = float((first_stopped.ts_ns - stop_start.ts_ns) / 1e9)
            coast_to_stop_cm = pos_error(first_stopped, entry_pos) * 100
        else:
            first_stopped = stop_end
            stop_time_s = float((stop_end.ts_ns - stop_start.ts_ns) / 1e9)
            coast_to_stop_cm = pos_error(stop_end, entry_pos) * 100

        total_drift_cm = pos_error(stop_end, entry_pos) * 100
        arrival_speed = float(stop_start.actual_speed)
        arrival_body_fwd = float(stop_start.body_forward_speed)

        later_off = [x for x in off_idx if x > oi]
        ei = later_off[0] - 1 if later_off else master.index[-1]
        mark_window = line_metrics(master.loc[oi:ei], line)
        if mark_window.empty:
            continue
        spray_on = mark_window.iloc[0]
        peak_idx = mark_window.xtrack_m.abs().idxmax()
        peak = mark_window.loc[peak_idx]

        align_rows = pre[pre.segment_state_name == "CORNER_ALIGN"]
        align_exit_err_deg = math.nan
        if not align_rows.empty:
            last_align = align_rows.iloc[-1]
            align_exit_err_deg = float(math.degrees(last_align.segment_heading_err))

        body_after_02 = mark_window[mark_window.along_m >= 0.2]
        body_rms_cm = (
            float(math.sqrt(np.mean(body_after_02.xtrack_m**2)) * 100) if len(body_after_02) else math.nan
        )

        corners.append(
            {
                "corner": line["line"],
                "arrival_speed_mps": arrival_speed,
                "arrival_body_fwd_mps": arrival_body_fwd,
                "stop_time_to_002_s": stop_time_s,
                "coast_to_stop_cm": coast_to_stop_cm,
                "total_stop_drift_cm": total_drift_cm,
                "stop_progress_past_pre_start_cm": along_from_pre(stop_end, line) * 100,
                "mark_entry_xtrack_cm": float(abs(spray_on.xtrack_m) * 100),
                "mark_entry_signed_xtrack_cm": float(spray_on.xtrack_m * 100),
                "mark_peak_xtrack_cm": float(abs(peak.xtrack_m) * 100),
                "mark_peak_zone": "entry" if peak.along_m < line["mark_length_m"] / 3 else "other",
                "mark_body_after_02_rms_cm": body_rms_cm,
                "pivot_exit_heading_err_deg": align_exit_err_deg,
                "entry_equals_drift_delta_cm": float(
                    abs(abs(spray_on.xtrack_m) * 100 - total_drift_cm)
                ),
            }
        )

    corner_df = pd.DataFrame(corners)
    all_mark = []
    for line, oi in zip(lines, on_idx):
        later_off = [x for x in off_idx if x > oi]
        ei = later_off[0] - 1 if later_off else master.index[-1]
        w = line_metrics(master.loc[oi:ei], line)
        all_mark.append(w)
    if all_mark:
        mark_all = pd.concat(all_mark)
        mission = {
            "mark_rms_cm": float(math.sqrt(np.mean(mark_all.xtrack_m**2)) * 100),
            "mark_within_2cm_pct": float((mark_all.xtrack_m.abs() <= 0.02).mean() * 100),
            "mark_median_abs_cm": float(mark_all.xtrack_m.abs().median() * 100),
        }
    else:
        mission = {}

    return {
        "bag": str(bag),
        "bag_name": bag_dir.name,
        "extension": len(lines) > 1,
        "corners": corners,
        "corner_summary": {
            "n": len(corners),
            "arrival_speed_mean": float(corner_df.arrival_speed_mps.mean()) if len(corner_df) else math.nan,
            "arrival_speed_max": float(corner_df.arrival_speed_mps.max()) if len(corner_df) else math.nan,
            "coast_to_stop_mean_cm": float(corner_df.coast_to_stop_cm.mean()) if len(corner_df) else math.nan,
            "total_drift_mean_cm": float(corner_df.total_stop_drift_cm.mean()) if len(corner_df) else math.nan,
            "mark_entry_mean_cm": float(corner_df.mark_entry_xtrack_cm.mean()) if len(corner_df) else math.nan,
            "mark_entry_max_cm": float(corner_df.mark_entry_xtrack_cm.max()) if len(corner_df) else math.nan,
            "entry_drift_delta_mean_cm": float(corner_df.entry_equals_drift_delta_cm.mean())
            if len(corner_df)
            else math.nan,
            "body_after_02_rms_mean_cm": float(corner_df.mark_body_after_02_rms_cm.mean())
            if len(corner_df)
            else math.nan,
            "pivot_exit_err_max_deg": float(corner_df.pivot_exit_heading_err_deg.abs().max())
            if len(corner_df)
            else math.nan,
        },
        "mission": mission,
    }


def model_stop_drift(v0: float, brake_cap: float, stop_thresh: float = 0.02) -> tuple[float, float]:
    """Constant opposing brake: dv/dt ≈ brake_cap, drift ≈ integral v dt."""
    if v0 <= stop_thresh:
        return 0.0, 0.0
    t_stop = (v0 - stop_thresh) / brake_cap
    # v(t) = v0 - brake_cap*t → distance = v0*t - 0.5*brake_cap*t^2
    drift = v0 * t_stop - 0.5 * brake_cap * t_stop**2
    return t_stop, drift * 100


def model_arrival_speed(
    mission_speed: float = 0.35,
    slowdown_dist: float = 0.50,
    min_corner_speed: float = 0.08,
    min_approach_v: float = 0.10,
    approach_dist: float = 0.60,
    dist_at_corner: float = 0.0,
) -> float:
    """Final-segment endpoint speed from current segment slowdown + approach logic."""
    scale_slow = 1.0 if slowdown_dist <= 0 else max(0.0, min(1.0, dist_at_corner / slowdown_dist))
    speed = max(min_corner_speed, mission_speed * scale_slow)
    scale_app = 1.0 if approach_dist <= 0 else max(0.0, min(1.0, dist_at_corner / approach_dist))
    speed = min(speed, max(min_approach_v, mission_speed * scale_app))
    return speed


def main() -> None:
    bags = sys.argv[1:] or [
        "/Users/dyx_a1/Vetri/PX4_DXP/bags/18-06-2026/Extension_Fix /square_2x2.dxf_20260618_200436",
        "/Users/dyx_a1/Vetri/PX4_DXP/bags/18-06-2026/Extension_Fix /square_2x2.dxf_20260618_200121",
        "/Users/dyx_a1/Vetri/PX4_DXP/bags/18-06-2026/Extension_Fix /square_2x2.dxf_20260618_200716",
        "/Users/dyx_a1/Vetri/PX4_DXP/bags/18-06-2026/square_2x2.dxf_20260618_174704",
    ]

    results = []
    for bag_path in bags:
        p = Path(bag_path)
        r = analyze_bag(p)
        results.append(r)
        print(f"\n=== {r['bag_name']} ===")
        print(json.dumps(r["corner_summary"], indent=2))
        print(json.dumps(r["mission"], indent=2))
        for c in r["corners"]:
            print(
                f"  C{c['corner']}: arr={c['arrival_speed_mps']:.3f} m/s  "
                f"coast={c['coast_to_stop_cm']:.2f} cm  drift={c['total_stop_drift_cm']:.2f} cm  "
                f"entry_xt={c['mark_entry_xtrack_cm']:.2f} cm  peak={c['mark_peak_xtrack_cm']:.2f} cm  "
                f"body_rms@0.2m={c['mark_body_after_02_rms_cm']:.2f} cm  "
                f"pivot_exit={c['pivot_exit_heading_err_deg']:+.2f}°"
            )

    print("\n=== LEVER MODEL (v0 from measured mean 0.13 m/s unless noted) ===")
    v0 = 0.13
    brake = 0.08
    t, drift = model_stop_drift(v0, brake)
    print(f"Baseline: v0={v0}, brake_cap={brake} → t={t:.2f}s drift={drift:.2f} cm")

    scenarios = [
        ("A: min_corner 0.03", 0.10, 0.08),
        ("B: slowdown 0.8 (v0≈0.10)", 0.10, 0.08),
        ("C: brake_cap 0.15", 0.13, 0.15),
        ("D: v0 0.05 pre-stop", 0.05, 0.08),
        ("C+D: brake 0.15 + v0 0.05", 0.05, 0.15),
        ("A+C: min_corner→v0 0.08 + brake 0.15", 0.08, 0.15),
    ]
    for name, v, cap in scenarios:
        t, d = model_stop_drift(v, cap)
        print(f"  {name}: v0={v:.2f} cap={cap:.2f} → t={t:.2f}s drift={d:.2f} cm")

    print("\n=== ARRIVAL SPEED AT RUN END (dist=0) ===")
    for label, kwargs in [
        ("current", {}),
        ("A min_corner=0.03", {"min_corner_speed": 0.03}),
        ("B slowdown=0.8 @dist=0", {"dist_at_corner": 0.0, "slowdown_dist": 0.8}),
        ("lower min_approach=0.05", {"min_approach_v": 0.05}),
        ("A+C min_corner=0.03 + min_approach=0.05", {"min_corner_speed": 0.03, "min_approach_v": 0.05}),
    ]:
        v = model_arrival_speed(**kwargs)
        t, d = model_stop_drift(v, 0.08)
        t15, d15 = model_stop_drift(v, 0.15)
        print(f"  {label}: endpoint v={v:.3f} m/s  drift@0.08={d:.2f}cm  drift@0.15={d15:.2f}cm")

    out = Path("/Users/dyx_a1/Vetri/PX4_DXP/analysis/corner_entry_drift.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)


if __name__ == "__main__":
    main()