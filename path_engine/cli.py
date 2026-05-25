"""Command-line interface for path_engine — standalone testing without ROS2 or FastAPI.

Usage:
    python -m path_engine.cli plan soccer_field.dxf
    python -m path_engine.cli plan mission.waypoints --origin 13.072 80.262
    python -m path_engine.cli info path.csv
    python -m path_engine.cli parse-dxf soccer_field.dxf
"""

from __future__ import annotations

import argparse
import json
import sys

from .core import SegmentType
from .engine import PathEngine


def _format_segments(segments) -> list[dict]:
    """Format segments for JSON output."""
    result = []
    for seg in segments:
        result.append({
            "type": "MARK" if seg.segment_type == SegmentType.MARK else "TRANSIT",
            "points": seg.points,
            "speed": seg.speed,
            "segment_id": seg.segment_id,
            "source": seg.source_entity,
            "length_m": round(seg.length, 3),
        })
    return result


def cmd_plan(args):
    """Run the full planning pipeline on a file."""
    engine = PathEngine(
        mark_spacing=args.mark_spacing,
        transit_spacing=args.transit_spacing,
        marking_speed=args.mark_speed,
        transit_speed=args.transit_speed,
    )

    origin = (0.0, 0.0)
    if args.origin:
        parts = args.origin.split(",")
        if len(parts) == 2:
            origin = (float(parts[0]), float(parts[1]))

    start_position = None
    if args.start_position:
        parts = args.start_position.split(",")
        if len(parts) == 2:
            start_position = (float(parts[0]), float(parts[1]))

    plan = engine.plan_file(
        args.filepath,
        origin=origin,
        start_position=start_position,
    )

    if args.output:
        # Write waypoints as CSV
        with open(args.output, "w") as f:
            f.write("north_m,east_m,spray_on\n")
            for pt, flag in zip(plan.merged_waypoints, plan.spray_flags):
                f.write(f"{pt[0]:.4f},{pt[1]:.4f},{1 if flag else 0}\n")
        print(f"Wrote {plan.num_waypoints} waypoints to {args.output}")
    else:
        # Print summary
        print(f"Planned path: {plan.num_waypoints} waypoints")
        print(f"  Segments: {len(plan.segments)}")
        print(f"  MARK length: {plan.total_mark_length:.2f} m")
        print(f"  TRANSIT length: {plan.total_transit_length:.2f} m")
        print(f"  Total length: {plan.total_length:.2f} m")
        if plan.merged_waypoints:
            first = plan.merged_waypoints[0]
            last = plan.merged_waypoints[-1]
            print(f"  First point: ({first[0]:.3f}N, {first[1]:.3f}E)")
            print(f"  Last point:  ({last[0]:.3f}N, {last[1]:.3f}E)")

    if args.json:
        output = {
            "num_waypoints": plan.num_waypoints,
            "total_mark_length": round(plan.total_mark_length, 3),
            "total_transit_length": round(plan.total_transit_length, 3),
            "segments": _format_segments(plan.segments),
        }
        print(json.dumps(output, indent=2))


def cmd_parse_dxf(args):
    """Parse a DXF file and show entity list."""
    from .parsers.dxf_parser import parse_dxf

    entities = parse_dxf(args.filepath, unit_scale=args.unit_scale)

    print(f"DXF file: {args.filepath}")
    print(f"Entities found: {len(entities)}")
    print()

    type_counts: dict[str, int] = {}
    for ent in entities:
        type_counts[ent.entity_type] = type_counts.get(ent.entity_type, 0) + 1

    for etype, count in sorted(type_counts.items()):
        print(f"  {etype}: {count}")

    print()
    for ent in entities[:20]:  # Show first 20
        is_mark = "MARK" if ent.is_mark() else "TRANSIT"
        print(f"  [{is_mark}] {ent.entity_type} layer={ent.layer!r} id={ent.entity_id}")
        if ent.entity_type == "LINE":
            s = ent.geometry["start"]
            e = ent.geometry["end"]
            length = ((s[0]-e[0])**2 + (s[1]-e[1])**2)**0.5
            print(f"         ({s[0]:.3f},{s[1]:.3f}) → ({e[0]:.3f},{e[1]:.3f}) len={length:.3f}m")
        elif ent.entity_type == "CIRCLE":
            c = ent.geometry["center"]
            r = ent.geometry["radius"]
            print(f"         center=({c[0]:.3f},{c[1]:.3f}) r={r:.3f}m")

    if len(entities) > 20:
        print(f"  ... and {len(entities) - 20} more")


def cmd_info(args):
    """Show info about a mission file without planning."""
    from .parsers import load_mission_file, load_mission_segments

    pts = load_mission_file(args.filepath)
    segments = load_mission_segments(args.filepath)

    print(f"File: {args.filepath}")
    print(f"  Flat waypoints: {len(pts)}")
    print(f"  Segments: {len(segments)}")
    for i, seg in enumerate(segments):
        stype = "MARK" if seg.segment_type == SegmentType.MARK else "TRANSIT"
        print(f"    [{i}] {stype} {seg.source_entity}: {len(seg.points)} pts, "
              f"{seg.length:.3f}m @ {seg.speed:.2f}m/s")


def main():
    parser = argparse.ArgumentParser(description="DYX Path Planning Engine CLI")
    subparsers = parser.add_subparsers(dest="command", required=True)

    # plan
    plan_parser = subparsers.add_parser("plan", help="Plan a path from file")
    plan_parser.add_argument("filepath", help="Path to .dxf, .csv, or .waypoints file")
    plan_parser.add_argument("--origin", help="NED origin as 'north,east' (e.g. '1.5,0.3')")
    plan_parser.add_argument("--start-position", help="Rover start position as 'north,east' for TSP optimization")
    plan_parser.add_argument("--output", "-o", help="Write waypoints to CSV file")
    plan_parser.add_argument("--json", action="store_true", help="Print JSON summary")
    plan_parser.add_argument("--mark-spacing", type=float, default=0.05, help="MARK waypoint spacing (m)")
    plan_parser.add_argument("--transit-spacing", type=float, default=0.15, help="TRANSIT waypoint spacing (m)")
    plan_parser.add_argument("--mark-speed", type=float, default=0.35, help="MARK speed (m/s)")
    plan_parser.add_argument("--transit-speed", type=float, default=0.50, help="TRANSIT speed (m/s)")
    plan_parser.set_defaults(func=cmd_plan)

    # parse-dxf
    dxf_parser = subparsers.add_parser("parse-dxf", help="Parse DXF and show entities")
    dxf_parser.add_argument("filepath", help="Path to .dxf file")
    dxf_parser.add_argument("--unit-scale", type=float, default=None, help="Metres per DXF unit (auto-detect if omitted)")
    dxf_parser.set_defaults(func=cmd_parse_dxf)

    # info
    info_parser = subparsers.add_parser("info", help="Show info about a mission file")
    info_parser.add_argument("filepath", help="Path to mission file")
    info_parser.set_defaults(func=cmd_info)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()