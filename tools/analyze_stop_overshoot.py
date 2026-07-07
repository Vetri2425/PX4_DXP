#!/usr/bin/env python3
"""Analyze stop overshoot for each event across 10 bags.

Outputs per-bag tables and a cross-bag ranking of worst 3 overshoot events.
"""
from __future__ import annotations

import json
import math
import sqlite3
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from rosbags.typesys import Stores, get_typestore


@dataclass
class StopEvent:
    """A distinct stop event."""
    reason: str
    target_n: float
    target_e: float
    certified_error_cm: float
    max_overshoot_cm: float
    max_overshoot_at_s: float
    has_reverse_recovery: bool
    settle_duration_s: float
    phase_at_cert: str
    event_start_t: float
    cert_t: float


PHASE_MAP = {
    0: "INACTIVE", 1: "HOLDING", 2: "STOP_CERTIFIED",
    3: "ALIGNING", 4: "ALIGN_CERTIFIED", 5: "FINAL_CERTIFIED", 6: "RELEASED", 7: "BLOCKED"
}
REASON_MAP = {
    0: "UNKNOWN", 1: "INTRA_RUN_CORNER", 2: "RUN_BOUNDARY",
    3: "RUNTIME_ENTRY_TO_MARK", 4: "FINAL_ENDPOINT"
}


def load_bag(bundle: Path) -> dict:
    """Load stop_debug and pose topics from .db3."""
    ts = get_typestore(Stores.ROS2_HUMBLE)
    db = list(bundle.rglob("*.db3"))
    if not db:
        print(f"  WARNING: no .db3 in {bundle}", file=sys.stderr)
        return {"stop_debug": [], "pose": []}

    db = db[0]
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
        "stop_debug": read("/rpp/stop_debug"),
        "pose": read("/mavros/local_position/pose"),
    }


def enu_to_ned(pose_stamped_msg):
    """Convert ENU pose to NED."""
    # PoseStamped has pose.position
    # pos_n = pose.y (North = ENU North)
    # pos_e = pose.x (East = ENU East)
    # pos_d = -pose.z (Down = -ENU Up)
    pos = pose_stamped_msg.pose.position
    return pos.y, pos.x, -pos.z


def distance_to_target(rover_n, rover_e, target_n, target_e):
    """Euclidean distance in NED plane (N, E)."""
    return math.sqrt((rover_n - target_n)**2 + (rover_e - target_e)**2)


def analyze_stop_events(stop_debug, pose_data) -> list[StopEvent]:
    """
    Parse stop_debug messages, group into distinct events, and compute overshoot.

    stop_debug messages are Float32MultiArray with:
      [0] phase (0-7)
      [1] reason (1-4)
      [2] run_idx
      [3] seg_idx
      [4] target_n (NED frame, meters)
      [5] target_e (NED frame, meters)
      [6] position_error (meters)
      [7] measured_speed (m/s)
      [8] measured_yaw_rate (rad/s)
      [9] heading_error (rad)
      [10] dwell_time (s)
      [11] velocity_fresh (flag)
      [12] certificate_source
      [13] stop_cert_valid
      [14] align_cert_valid
      [15] final_cert_valid
      [16-19] unused
    """
    if not stop_debug:
        return []

    # Build a map: (reason, target_n, target_e) -> list of (t, phase, error_m, raw_msg)
    events_by_target = {}
    for t, msg in stop_debug:
        data = msg.data
        phase = int(data[0])
        reason = int(data[1])
        target_n = float(data[4])
        target_e = float(data[5])
        error_m = float(data[6])

        key = (reason, round(target_n, 4), round(target_e, 4))
        if key not in events_by_target:
            events_by_target[key] = []
        events_by_target[key].append((t, phase, error_m, msg))

    # Build pose map: t -> (rover_n, rover_e)
    pose_map = {}
    for t, pose_msg in pose_data:
        rover_n, rover_e, _ = enu_to_ned(pose_msg)
        pose_map[t] = (rover_n, rover_e)

    # Analyze each event
    results = []
    for (reason, target_n, target_e), messages in events_by_target.items():
        if not messages:
            continue

        # Find the window: first HOLDING (phase=1) to first *_CERTIFIED
        first_holding_idx = None
        cert_idx = None
        cert_phase = None

        for i, (t, phase, error_m, msg) in enumerate(messages):
            if first_holding_idx is None and phase in (1, 3):  # HOLDING or ALIGNING
                first_holding_idx = i
            if cert_idx is None and phase in (2, 4, 5):  # STOP_CERTIFIED, ALIGN_CERTIFIED, FINAL_CERTIFIED
                cert_idx = i
                cert_phase = phase

        # If no complete lifecycle, skip
        if first_holding_idx is None or cert_idx is None:
            continue

        event_start_t = messages[first_holding_idx][0]
        event_end_t = messages[cert_idx][0]
        cert_error_m = messages[cert_idx][2]

        # Gather pose samples in the window [event_start_t, event_end_t]
        poses_in_window = []
        for t, (rover_n, rover_e) in pose_map.items():
            if event_start_t <= t <= event_end_t:
                dist_m = distance_to_target(rover_n, rover_e, target_n, target_e)
                poses_in_window.append((t, dist_m))

        if not poses_in_window:
            # No pose samples in this window; use the error from stop_debug
            max_overshoot_m = cert_error_m
            has_reverse = False
            max_overshoot_at_s = 0
        else:
            poses_in_window.sort(key=lambda x: x[0])

            # Find max distance and detect reverse/recovery
            max_dist_m = max(d for _, d in poses_in_window)
            max_overshoot_at_s = poses_in_window[np.argmax([d for _, d in poses_in_window])][0] - event_start_t

            # Detect reverse recovery: does distance increase and then decrease?
            dists = [d for _, d in poses_in_window]
            if len(dists) > 1:
                # Look for a local max followed by decrease to final
                has_reverse = False
                for i in range(len(dists) - 1):
                    if dists[i] < dists[i+1]:
                        # Found an increase; check if it decreases later
                        for j in range(i+1, len(dists)):
                            if dists[j] < dists[i]:
                                has_reverse = True
                                break
                        if has_reverse:
                            break
            else:
                has_reverse = False

            max_overshoot_m = max_dist_m

        settle_duration_s = event_end_t - event_start_t

        results.append(StopEvent(
            reason=REASON_MAP.get(reason, f"UNKNOWN({reason})"),
            target_n=target_n,
            target_e=target_e,
            certified_error_cm=cert_error_m * 100,
            max_overshoot_cm=max_overshoot_m * 100,
            max_overshoot_at_s=max_overshoot_at_s,
            has_reverse_recovery=has_reverse,
            settle_duration_s=settle_duration_s,
            phase_at_cert=PHASE_MAP.get(cert_phase, f"UNKNOWN({cert_phase})"),
            event_start_t=event_start_t,
            cert_t=event_end_t,
        ))

    return results


def format_mission_name(bundle_name: str) -> str:
    """Extract mission shape from bundle name."""
    # e.g., "2026-07-07_18-12-25.264_IST_Line_2m.DXF_stg_..."
    if "_Line_" in bundle_name:
        return "Line_2m"
    elif "_L_2m" in bundle_name:
        return "L_2m"
    elif "_square_" in bundle_name:
        return "square_2m"
    elif "_circle_" in bundle_name:
        return "circle_3m"
    else:
        return bundle_name[:30]


def main():
    bag_dirs = [
        "bags/7-7-2026/new/new_01/2026-07-07_18-12-25.264_IST_Line_2m.DXF_stg_a9b9bc1d_1783428133",
        "bags/7-7-2026/new/new_01/2026-07-07_18-13-30.500_IST_Line_2m.DXF_stg_a9b9bc1d_1783428133",
        "bags/7-7-2026/new/new_01/2026-07-07_18-14-50.097_IST_Line_2m.DXF_stg_a9b9bc1d_1783428133",
        "bags/7-7-2026/new/new_01/2026-07-07_18-18-07.060_IST_L_2m.DXF_stg_8e7306b9_1783428479",
        "bags/7-7-2026/new/new_01/2026-07-07_18-19-44.818_IST_L_2m.DXF_stg_8e7306b9_1783428479",
        "bags/7-7-2026/new/new_01/2026-07-07_18-22-21.226_IST_square_2m.DXF_stg_9c1ef887_1783428731",
        "bags/7-7-2026/new/new_01/2026-07-07_18-24-55.579_IST_square_2m.DXF_stg_9c1ef887_1783428731",
        "bags/7-7-2026/new/new_01/2026-07-07_18-28-59.237_IST_circle_3m.DXF_stg_2d20b16e_1783429128",
        "bags/7-7-2026/new/new_01/2026-07-07_18-34-11.060_IST_circle_3m.DXF_stg_2d20b16e_1783429128",
        "bags/7-7-2026/new/new_01/2026-07-07_18-36-16.583_IST_circle_3m.DXF_stg_2d20b16e_1783429128",
    ]

    root = Path("/Users/dyx_a1/Vetri/PX4_DXP")
    all_events = []

    for bag_path_rel in bag_dirs:
        bundle = root / bag_path_rel
        mission_name = format_mission_name(bundle.name)

        print(f"\n{'='*80}")
        print(f"MISSION: {mission_name}")
        print(f"BUNDLE: {bundle.name}")
        print(f"{'='*80}")

        if not bundle.exists():
            print(f"  ERROR: {bundle} does not exist")
            continue

        data = load_bag(bundle)
        if not data["stop_debug"]:
            print("  WARNING: /rpp/stop_debug topic not found or empty")
            continue
        if not data["pose"]:
            print("  WARNING: /mavros/local_position/pose topic not found or empty")
            continue

        events = analyze_stop_events(data["stop_debug"], data["pose"])

        if not events:
            print("  No stop events detected in this mission")
            continue

        print(f"\nStop Events Table ({len(events)} events):")
        print("-" * 160)
        print(
            f"{'Reason':<20} {'Target(N,E)':<30} "
            f"{'Cert Error(cm)':<15} {'Max Overshoot(cm)':<18} "
            f"{'Reverse?':<10} {'Settle(s)':<10}"
        )
        print("-" * 160)

        for event in events:
            target_str = f"({event.target_n:.2f}, {event.target_e:.2f})"
            reverse_str = "YES" if event.has_reverse_recovery else "NO"
            print(
                f"{event.reason:<20} {target_str:<30} "
                f"{event.certified_error_cm:>13.2f} {event.max_overshoot_cm:>16.2f} "
                f"{reverse_str:<10} {event.settle_duration_s:>8.2f}"
            )
            all_events.append((mission_name, event))

        print()

    # Cross-bag ranking: worst 3 overshoots
    print("\n" + "="*80)
    print("CROSS-BAG RANKING: Top 3 Worst Overshoots")
    print("="*80)

    sorted_events = sorted(all_events, key=lambda x: x[1].max_overshoot_cm, reverse=True)

    for rank, (mission, event) in enumerate(sorted_events[:3], 1):
        print(
            f"\n{rank}. {mission} | {event.reason}\n"
            f"   Target: N={event.target_n:.2f}, E={event.target_e:.2f}\n"
            f"   Max Overshoot: {event.max_overshoot_cm:.2f} cm\n"
            f"   Certified Error: {event.certified_error_cm:.2f} cm\n"
            f"   Reverse/Recovery: {'YES' if event.has_reverse_recovery else 'NO'}\n"
            f"   Settle Duration: {event.settle_duration_s:.2f} s"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
