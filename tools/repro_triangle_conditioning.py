#!/usr/bin/env python3
"""Offline reproduction of path conditioning against the exact bag /path.

Feeds the recorded triangle /path through RPPControllerNode._path_cb and prints
the resulting run list (length, heading, profile), the corner angle and pivot
target at each run transition, and the angle-aware pivot-timeout budget.

Run on a ROS2-sourced host:
    PYTHONPATH=src python3 -X utf8 tools/repro_triangle_conditioning.py /tmp/triangle_path.json
"""
import json
import math
import sys

import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def heading_deg(p0, p1):
    return math.degrees(math.atan2(p1.y - p0.y, p1.x - p0.x))


def main():
    path_json = sys.argv[1] if len(sys.argv) > 1 else "/tmp/triangle_path.json"
    pts = json.load(open(path_json))

    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    try:
        from rpp_controller_node import RPPControllerNode
        node = RPPControllerNode()

        msg = Path()
        msg.header.frame_id = node.get_parameter("path_frame_id").value
        msg.header.stamp = node.get_clock().now().to_msg()
        for n, e, z in pts:
            ps = PoseStamped()
            ps.header.frame_id = msg.header.frame_id
            ps.pose.position.x = float(n)
            ps.pose.position.y = float(e)
            ps.pose.position.z = float(z)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        node._path_cb(msg)
        runs = node._runs

        rate = float(node.get_parameter("segment_nominal_pivot_rate_rad_s").value)
        margin = float(node.get_parameter("segment_pivot_spinup_margin_s").value)
        base = float(node.get_parameter("segment_turn_timeout_s").value)
        max_s = float(node.get_parameter("segment_pivot_timeout_max_s").value)
        thr = float(node.get_parameter("segment_corner_threshold_deg").value)

        def budget(angle_rad):
            b = max(margin + angle_rad / rate, base)
            return min(b, max_s)

        def lead_seg(poses):
            """(heading°, first-seg length m) — the pivot target & whether the
            run leads with a tiny connector stub."""
            if len(poses) < 2:
                return float("nan"), 0.0
            a, b = poses[0].pose.position, poses[1].pose.position
            return heading_deg(a, b), math.hypot(b.x - a.x, b.y - a.y)

        print(f"\n=== CONDITIONED RUN LIST  ({len(runs)} runs) ===")
        print(f"{'run':>3} {'profile':8} {'len_m':>7} {'head°':>7} {'lead_m':>7} "
              f"{'corner°':>8} {'pivot_tgt°':>10} {'timeout_s':>9}")
        leads = []
        prev_exit = None
        for i, r in enumerate(runs):
            poses = r["poses"]
            h_in, seg = lead_seg(poses)
            leads.append((h_in, seg))
            corner = pivot_tgt = tbud = float("nan")
            if prev_exit is not None and not math.isnan(h_in):
                corner = math.degrees(RPPControllerNode._heading_delta(
                    math.radians(prev_exit), math.radians(h_in)))
                if corner >= thr:
                    pivot_tgt = h_in
                    tbud = budget(math.radians(corner))
            print(f"{i:>3} {r['profile']:8} {r['length']:7.3f} {h_in:7.1f} {seg:7.3f} "
                  f"{corner:8.1f} {pivot_tgt:10.1f} {tbud:9.2f}")
            prev_exit = heading_deg(poses[-2].pose.position, poses[-1].pose.position) if len(poses) >= 2 else prev_exit

        # Apex-2: connector now appears neither as its own run nor as a tiny
        # leading segment; the run after Leg2 must pivot toward ~-150°.
        tiny_leads = [seg for _h, seg in leads if seg < 0.15]
        print("\n=== APEX-2 VALIDATION ===")
        print(f"tiny (<15cm) connector leading segments remaining: {len(tiny_leads)} (want 0)")
        ok = False
        prev_exit = None
        for i, r in enumerate(runs):
            poses = r["poses"]
            if len(poses) < 2:
                continue
            h_in, seg = lead_seg(poses)
            if prev_exit is not None:
                corner = math.degrees(RPPControllerNode._heading_delta(
                    math.radians(prev_exit), math.radians(h_in)))
                if abs(((h_in + 180) % 360) - 180 - (-150.0)) <= 5.0 and corner >= thr and seg >= 0.5:
                    print(f"Leg3 run {i}: corner={corner:.1f}° pivot_target={h_in:.1f}° "
                          f"lead_seg={seg:.3f}m timeout={budget(math.radians(corner)):.2f}s")
                    ok = True
            prev_exit = heading_deg(poses[-2].pose.position, poses[-1].pose.position)
        print(f"apex-2 = one run transition, ~120° corner, -150° pivot target, no stub: {ok}")

        node.destroy_node()
        return 0 if (len(tiny_leads) == 0 and ok) else 1
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
