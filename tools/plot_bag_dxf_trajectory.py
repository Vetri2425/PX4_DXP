#!/usr/bin/env python3
"""Plot bag trajectory with DXF mark lines vs extension/approach path."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sqlite3
import struct
import sys
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT))

import extract_rosbag_direct as E  # noqa: E402
from path_engine.engine import PathEngine  # noqa: E402
from path_engine.parsers.dxf_parser import parse_dxf  # noqa: E402


def parse_full_path(data: bytes) -> list[tuple[float, float]]:
    """Decode nav_msgs/Path poses to ENU (east, north)."""
    off = E.skip_cdr_header(data, 0)
    _, off = E.read_int32(data, off)
    _, off = E.read_uint32(data, off)
    _, off = E.read_string(data, off)
    n, off = E.read_uint32(data, off)
    marks = [m.start() for m in re.finditer(b"local_ned", data)][1:]
    pts: list[tuple[float, float]] = []
    for mark in marks[:n]:
        end = (mark + 10 + 3) & ~3
        north, east, _ = struct.unpack_from("<ddd", data, end)
        pts.append((east, north))
    return pts


def load_bag(bag_dir: Path) -> tuple[list[tuple[float, float]], list[tuple[float, float, float]]]:
    db3 = next(bag_dir.glob("*.db3"))
    con = sqlite3.connect(db3)
    topics = {name: tid for tid, name, _ in con.execute("select id,name,type from topics")}

    path_row = con.execute(
        "select data from messages where topic_id=? order by timestamp limit 1",
        (topics["/path"],),
    ).fetchone()
    path = parse_full_path(path_row[0])

    poses: list[tuple[float, float, float]] = []
    for ts, data in con.execute(
        "select timestamp,data from messages where topic_id=? order by timestamp",
        (topics["/mavros/local_position/pose"],),
    ):
        msg = E.parse_PoseStamped(data)
        poses.append((ts * 1e-9, msg["position_x"], msg["position_y"]))
    return path, poses


def seg_dist(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def geometric_xtrack(
    poses: list[tuple[float, float, float]],
    path: list[tuple[float, float]],
) -> np.ndarray:
    poly = path
    out = np.empty(len(poses))
    for i, (_, east, north) in enumerate(poses):
        out[i] = (
            min(
                seg_dist(east, north, poly[j][0], poly[j][1], poly[j + 1][0], poly[j + 1][1])
                for j in range(len(poly) - 1)
            )
            * 100.0
        )
    return out


def find_approach_split(path: list[tuple[float, float]], thresh_m: float = 1.0) -> int:
    for i in range(len(path) - 1):
        d = math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        if d > thresh_m:
            return i
    return 0


def load_dxf_plan_roles(dxf_path: Path, ext_cfg: dict) -> tuple[list[tuple[float, float]], list[str]]:
    sidecar_ext = dxf_path.parent / f".{dxf_path.name}.extensions.json"
    sidecar_ent = dxf_path.parent / f".{dxf_path.name}.entities.json"

    overrides: dict[str, bool] = {}
    if sidecar_ent.exists():
        with open(sidecar_ent, encoding="utf-8") as f:
            overrides = {
                k: bool(v.get("is_mark", True))
                for k, v in json.load(f).get("overrides", {}).items()
            }

    enabled = bool(ext_cfg.get("enabled", False))
    pre_m = float(ext_cfg.get("pre_extension_m", 0.5))
    aft_m = float(ext_cfg.get("aft_extension_m", 0.5))

    entities = parse_dxf(str(dxf_path))
    for ent in entities:
        if ent.entity_id in overrides:
            ent.is_mark_override = overrides[ent.entity_id]

    engine = PathEngine(
        enable_path_extensions=enabled,
        pre_extension_m=pre_m,
        aft_extension_m=aft_m,
    )
    plan = engine.plan_dxf_entities(entities, start_position=(0.0, 0.0))

    waypoints: list[tuple[float, float]] = []
    roles: list[str] = []
    for seg in plan.segments:
        role = seg.metadata.get("extension_role") or (
            "dxf_mark" if seg.segment_type.name == "MARK" else "transit"
        )
        if seg.metadata.get("extension_connector"):
            role = "extension_connector"
        elif role == "transit" and "start" in seg.source_entity:
            role = "approach_transit"
        elif role in ("pre", "aft"):
            role = f"extension_{role}"

        for pt in seg.points:
            enu = (pt[1], pt[0])
            if waypoints:
                last = waypoints[-1]
                if (
                    math.hypot(enu[0] - last[0], enu[1] - last[1]) < 0.01
                    and roles[-1] == role
                ):
                    continue
            waypoints.append(enu)
            roles.append(role)
    return waypoints, roles


def classify_bag_waypoints(
    bag_path: list[tuple[float, float]],
    plan_waypoints: list[tuple[float, float]],
    plan_roles: list[str],
) -> list[str]:
    """Label each bag waypoint as approach, extension, or DXF mark."""
    split = find_approach_split(bag_path)
    approach_end = min(split + 2, len(bag_path))
    mark_bag = bag_path[approach_end:]
    # Include both ends of the long origin→field transit segment.
    labels = ["approach"] * approach_end

    if not mark_bag:
        return labels

    best = (1e9, 0, 0.0, 0.0, 0)
    for off in range(max(1, len(mark_bag) - len(plan_waypoints) + 1)):
        n = min(len(plan_waypoints), len(mark_bag) - off)
        if n < 20:
            break
        de = sum(mark_bag[off + i][0] - plan_waypoints[i][0] for i in range(n)) / n
        dn = sum(mark_bag[off + i][1] - plan_waypoints[i][1] for i in range(n)) / n
        errs = [
            math.hypot(
                mark_bag[off + i][0] - (plan_waypoints[i][0] + de),
                mark_bag[off + i][1] - (plan_waypoints[i][1] + dn),
            )
            for i in range(n)
        ]
        mean_err = sum(errs) / len(errs)
        if mean_err < best[0]:
            best = (mean_err, off, de, dn, n)

    _, off, de, dn, n = best
    mark_labels = ["dxf_mark"] * len(mark_bag)
    for i in range(n):
        role = plan_roles[i]
        if role in ("approach_transit", "extension_connector", "extension_pre", "extension_aft", "transit"):
            mark_labels[off + i] = "extension_path"
        elif role == "dxf_mark":
            mark_labels[off + i] = "dxf_mark"
        else:
            mark_labels[off + i] = "extension_path"

    return labels + mark_labels


def plot_segments(ax, pts, labels, style_map):
    if len(pts) < 2:
        return
    current = labels[0]
    start = 0
    for i in range(1, len(labels)):
        if labels[i] != current:
            _plot_labeled_segment(ax, pts[start:i], current, style_map)
            start = i
            current = labels[i]
    _plot_labeled_segment(ax, pts[start:], current, style_map)


def _plot_labeled_segment(ax, segment, label, style_map):
    if len(segment) < 1:
        return
    style = style_map[label]
    xs = [p[0] for p in segment]
    ys = [p[1] for p in segment]
    if len(segment) == 1:
        ax.plot(xs, ys, **style["point"], label=style["label"])
        return
    ax.plot(xs, ys, **style["line"], label=style["label"])
    ax.plot(xs, ys, **style["point"])


def annotate_corners(ax, pts, labels, every: int = 25):
    for i, (east, north) in enumerate(pts):
        if i % every != 0 and i not in (0, len(pts) - 1):
            continue
        if labels[i] == "approach" and i not in (0, 1):
            continue
        ax.annotate(
            str(i),
            (east, north),
            textcoords="offset points",
            xytext=(4, 4),
            fontsize=7,
            color="#333333",
        )


def plot_bag(
    bag_dir: Path,
    dxf_path: Path,
    out_path: Path,
    title_suffix: str = "",
) -> dict:
    bag_path, poses = load_bag(bag_dir)
    xtrack_cm = geometric_xtrack(poses, bag_path)

    ext_cfg_path = dxf_path.parent / f".{dxf_path.name}.extensions.json"
    ext_cfg = {"enabled": True, "pre_extension_m": 0.5, "aft_extension_m": 0.5}
    if ext_cfg_path.exists():
        with open(ext_cfg_path, encoding="utf-8") as f:
            ext_cfg.update(json.load(f))

    plan_pts, plan_roles = load_dxf_plan_roles(dxf_path, ext_cfg)
    labels = classify_bag_waypoints(bag_path, plan_pts, plan_roles)
    counts = Counter(labels)

    path_len = sum(
        math.hypot(bag_path[i + 1][0] - bag_path[i][0], bag_path[i + 1][1] - bag_path[i][1])
        for i in range(len(bag_path) - 1)
    )
    duration_s = poses[-1][0] - poses[0][0] if poses else 0.0

    style_map = {
        "approach": {
            "label": "Approach / origin transit",
            "line": {"color": "#E67E22", "linewidth": 2.5, "linestyle": "--", "zorder": 3},
            "point": {"color": "#E67E22", "marker": "o", "markersize": 5, "linestyle": "None", "zorder": 4},
        },
        "extension_path": {
            "label": "Extension path (pre/aft/connector)",
            "line": {"color": "#9B59B6", "linewidth": 2.0, "linestyle": "-.", "zorder": 3},
            "point": {"color": "#9B59B6", "marker": "s", "markersize": 3, "linestyle": "None", "zorder": 4},
        },
        "dxf_mark": {
            "label": "DXF lines (mark)",
            "line": {"color": "#2E86DE", "linewidth": 2.0, "zorder": 3},
            "point": {"color": "#2E86DE", "marker": ".", "markersize": 4, "linestyle": "None", "zorder": 4},
        },
    }

    fig, ax = plt.subplots(figsize=(12, 10))

    handles = []
    seen = set()
    for label in ("approach", "extension_path", "dxf_mark"):
        idxs = [i for i, lb in enumerate(labels) if lb == label]
        if not idxs:
            continue
        chunks = []
        start = idxs[0]
        prev = idxs[0]
        for i in idxs[1:] + [None]:
            if i is not None and i == prev + 1:
                prev = i
                continue
            chunks.append((start, prev + 1))
            if i is not None:
                start = i
                prev = i
        for s, e in chunks:
            seg = bag_path[s:e]
            style = style_map[label]
            xs = [p[0] for p in seg]
            ys = [p[1] for p in seg]
            line_label = style["label"] if label not in seen else None
            ax.plot(xs, ys, **style["line"], label=line_label)
            ax.plot(xs, ys, **style["point"])
            seen.add(label)

    actual_e = [p[1] for p in poses]
    actual_n = [p[2] for p in poses]
    sc = ax.scatter(
        actual_e,
        actual_n,
        c=xtrack_cm,
        cmap="RdYlGn_r",
        s=8,
        vmin=0,
        vmax=max(5.0, float(np.percentile(xtrack_cm, 95))),
        label="Actual trajectory",
        zorder=2,
        alpha=0.85,
    )
    plt.colorbar(sc, ax=ax, label="Cross-track error (cm)", shrink=0.85)

    ax.plot(bag_path[0][0], bag_path[0][1], "s", color="#27AE60", markersize=12, label="Start", zorder=6)
    ax.plot(bag_path[-1][0], bag_path[-1][1], "X", color="#C0392B", markersize=12, label="End", zorder=6)
    annotate_corners(ax, bag_path, labels)

    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    name = bag_dir.name
    suffix = f" — {title_suffix}" if title_suffix else ""
    ax.set_title(
        f"{dxf_path.name}{suffix}\n"
        f"{len(bag_path)} waypoints, {path_len:.2f} m planned, {duration_s:.0f} s bag"
    )
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)

    return {
        "bag": name,
        "waypoints": len(bag_path),
        "path_len_m": path_len,
        "duration_s": duration_s,
        "xtrack_mean_cm": float(np.mean(xtrack_cm)),
        "xtrack_rms_cm": float(np.sqrt(np.mean(xtrack_cm**2))),
        "xtrack_max_cm": float(np.max(xtrack_cm)),
        "labels": dict(counts),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", help="ROS2 bag directories")
    parser.add_argument("--dxf", default=str(ROOT / "server/missions/square_2x2.dxf"))
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args()

    dxf_path = Path(args.dxf)
    results = []
    for bag_arg in args.bags:
        bag_dir = Path(bag_arg)
        out_dir = Path(args.out_dir) if args.out_dir else bag_dir.parent / "waypoint_plots"
        stamp = bag_dir.name.rsplit("_", 2)[-2] + "_" + bag_dir.name.rsplit("_", 1)[-1]
        out_file = out_dir / f"trajectory_{stamp}.png"
        results.append(plot_bag(bag_dir, dxf_path, out_file, title_suffix=stamp))

    if len(results) > 1:
        fig, axes = plt.subplots(1, 2, figsize=(18, 8))
        for ax, bag_arg in zip(axes, args.bags):
            img = plt.imread(out_dir / f"trajectory_{Path(bag_arg).name.rsplit('_', 2)[-2]}_{Path(bag_arg).name.rsplit('_', 1)[-1]}.png")
            ax.imshow(img)
            ax.axis("off")
            ax.set_title(Path(bag_arg).name, fontsize=10)
        fig.tight_layout()
        combo = out_dir / "trajectory_both.png"
        fig.savefig(combo, dpi=120, bbox_inches="tight")
        plt.close(fig)
        print(f"saved {combo}")

    for r in results:
        print(
            f"{r['bag']}: {r['waypoints']} wp, {r['path_len_m']:.2f}m, "
            f"xtrack rms={r['xtrack_rms_cm']:.2f}cm max={r['xtrack_max_cm']:.2f}cm, "
            f"labels={r['labels']}, saved trajectory plot"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())