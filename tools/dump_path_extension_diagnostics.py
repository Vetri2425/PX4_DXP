#!/usr/bin/env python3
"""Dump DXF path geometry with and without drive extensions.

Example:
    python3 tools/dump_path_extension_diagnostics.py server/missions/square_2x2.dxf
"""

from __future__ import annotations

import argparse
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from path_engine.core import SegmentType
from path_engine.engine import PathEngine


def _dist(a, b) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _tangent_start(points):
    for i in range(1, len(points)):
        d = _dist(points[0], points[i])
        if d > 1e-9:
            return ((points[i][0] - points[0][0]) / d, (points[i][1] - points[0][1]) / d)
    return None


def _tangent_end(points):
    for i in range(len(points) - 2, -1, -1):
        d = _dist(points[i], points[-1])
        if d > 1e-9:
            return ((points[-1][0] - points[i][0]) / d, (points[-1][1] - points[i][1]) / d)
    return None


def _fmt_pt(pt) -> str:
    return f"({pt[0]:.4f},{pt[1]:.4f})"


def _fmt_vec(vec) -> str:
    if vec is None:
        return "(nan,nan)"
    return f"({vec[0]:.4f},{vec[1]:.4f})"


def _role(seg) -> str:
    if seg.metadata.get("extension_role") == "pre":
        return "pre"
    if seg.metadata.get("extension_role") == "aft":
        return "aft"
    if seg.metadata.get("extension_connector"):
        return "connector"
    return "mark" if seg.segment_type == SegmentType.MARK else "transit"


def _dump_plan(label: str, plan) -> None:
    print(f"\n## {label}")
    print(
        "idx\tid\ttype\trole\tsource\tparent\tfirst\tlast\t"
        "start_tangent\tend_tangent\tpoints\tlength_m"
    )
    for i, seg in enumerate(plan.segments):
        parent = seg.metadata.get("parent_source_entity", "")
        print(
            f"{i}\t{seg.segment_id}\t{seg.segment_type.name}\t{_role(seg)}\t"
            f"{seg.source_entity}\t{parent}\t{_fmt_pt(seg.points[0])}\t"
            f"{_fmt_pt(seg.points[-1])}\t{_fmt_vec(_tangent_start(seg.points))}\t"
            f"{_fmt_vec(_tangent_end(seg.points))}\t{len(seg.points)}\t{seg.length:.4f}"
        )

    print("\n# discontinuities")
    print("idx\tfrom\tto\tdistance_m\theading_jump_deg")
    found = False
    for i, (a, b) in enumerate(zip(plan.segments, plan.segments[1:])):
        gap = _dist(a.points[-1], b.points[0])
        ta = _tangent_end(a.points)
        tb = _tangent_start(b.points)
        jump = float("nan")
        if ta is not None and tb is not None:
            dot = max(-1.0, min(1.0, ta[0] * tb[0] + ta[1] * tb[1]))
            jump = math.degrees(math.acos(dot))
        if gap > 0.01 or (not math.isnan(jump) and jump > 30.0):
            found = True
            print(f"{i}\t{a.source_entity}\t{b.source_entity}\t{gap:.4f}\t{jump:.1f}")
    if not found:
        print("none")

    print("\n# merged path")
    print(
        f"waypoints={len(plan.merged_waypoints)} mark_length_m={plan.total_mark_length:.4f} "
        f"transit_length_m={plan.total_transit_length:.4f}"
    )


def _make_engine(args, enable_extensions: bool) -> PathEngine:
    return PathEngine(
        mark_spacing=args.mark_spacing,
        transit_spacing=args.transit_spacing,
        optimize_order=not args.no_optimize,
        compensate_spray=args.compensate_spray,
        enable_path_extensions=enable_extensions,
        pre_extension_m=args.pre,
        aft_extension_m=args.aft,
        corner_smooth_radius_m=args.corner_smooth_radius,
        corner_smooth_arc_pts=args.corner_smooth_arc_pts,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dxf", help="DXF mission file")
    parser.add_argument("--pre", type=float, default=0.5)
    parser.add_argument("--aft", type=float, default=0.5)
    parser.add_argument("--mark-spacing", type=float, default=0.05)
    parser.add_argument("--transit-spacing", type=float, default=0.15)
    parser.add_argument("--corner-smooth-radius", type=float, default=0.0)
    parser.add_argument("--corner-smooth-arc-pts", type=int, default=6)
    parser.add_argument("--no-optimize", action="store_true")
    parser.add_argument("--compensate-spray", action="store_true")
    args = parser.parse_args()

    no_ext = _make_engine(args, enable_extensions=False).plan_file(args.dxf)
    with_ext = _make_engine(args, enable_extensions=True).plan_file(args.dxf)

    _dump_plan("no extensions", no_ext)
    _dump_plan("with extensions", with_ext)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
