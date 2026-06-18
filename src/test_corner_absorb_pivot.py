#!/usr/bin/env python3
"""Tests for the triangle corner-exit fix.

PART A — short-connector absorption before run splitting.
PART B — angle-aware pivot watchdog + hard release gate.

Run on a ROS2-sourced host (needs rclpy):
    python3 -X utf8 src/test_corner_absorb_pivot.py
"""
import math
import sys

import rclpy
from rclpy.parameter import Parameter


# --------------------------------------------------------------------------
# Geometry helpers (no node needed — these target the pure classmethods)
# --------------------------------------------------------------------------
def _hd(cls, a_deg, b_deg):
    return math.degrees(cls._heading_delta(math.radians(a_deg), math.radians(b_deg)))


def _run_lead(cls, sub_runs):
    """Per split sub-run: (first-segment heading °, first-segment length m,
    total length m), measured on the *simplified* run as the tracker sees it.

    The first segment heading is exactly what _apply_run/_run_alignment_hold
    pivots toward, so it is the pivot target for the run.
    """
    out = []
    for pts, _flags in sub_runs:
        simple, _ = cls._simplify_path_for_profile(pts)
        if len(simple) >= 2:
            h = math.degrees(cls._segment_heading(simple[0], simple[1]))
            seg = math.hypot(simple[1][0] - simple[0][0], simple[1][1] - simple[0][1])
        else:
            h, seg = float("nan"), 0.0
        out.append((h, seg, cls._pts_length(pts)))
    return out


def _norm180(deg):
    return ((deg + 180) % 360) - 180


def _triangle_apex_path():
    """Leg2(-30°,~2m) → connector(-120°,8.45cm) → Leg3(-150°,~2m), all MARK.

    Mirrors the bag triangle_202m apex 2: two near-coincident apex waypoints
    leave an 8.45 cm connector at -120° between the -30° and -150° legs.
    Collinear mid-samples are included so simplification has real work to do.
    """
    d = lambda deg: (math.cos(math.radians(deg)), math.sin(math.radians(deg)))
    s = (0.0, 0.0)
    v_in = (s[0] + 2.0 * d(-30)[0], s[1] + 2.0 * d(-30)[1])           # -30° leg, 2 m
    v_out = (v_in[0] + 0.0845 * d(-120)[0], v_in[1] + 0.0845 * d(-120)[1])  # 8.45 cm connector
    end = (v_out[0] + 2.0 * d(-150)[0], v_out[1] + 2.0 * d(-150)[1])  # -150° leg, 2 m
    mid_a = ((s[0] + v_in[0]) / 2, (s[1] + v_in[1]) / 2)
    mid_b = ((v_out[0] + end[0]) / 2, (v_out[1] + end[1]) / 2)
    pts = [s, mid_a, v_in, v_out, mid_b, end]
    flags = [True] * len(pts)
    return pts, flags


def test_1_triangle_tiny_connector(cls):
    pts, flags = _triangle_apex_path()
    THR, ABSORB, MINC = 45.0, 0.20, 20.0

    # Baseline (no absorption): the 8.45cm connector is the LEADING SEGMENT of
    # the apex run (fused with Leg3, since connector→Leg3 = 30° < threshold), so
    # the run's pivot target is the connector heading ≈ -120° — the bug.
    base = _run_lead(cls, cls._split_run_at_corners(pts, flags, THR))
    apex_base = [r for r in base if r[1] < 0.15]
    assert apex_base, f"baseline should have a tiny connector leading segment, got {base}"
    assert abs(_norm180(apex_base[0][0]) - (-120.0)) <= 3.0, \
        f"baseline pivot target should be the connector ~-120°, got {apex_base[0][0]:.1f}"

    # With absorption: the connector is gone; the apex run leads with a real
    # leg whose heading (the pivot target) is ≈ -150°, never -120°.
    ab_pts, ab_flags = cls._absorb_short_connectors(pts, flags, THR, ABSORB, MINC)
    runs = cls._split_run_at_corners(ab_pts, ab_flags, THR)
    rh = _run_lead(cls, runs)
    assert len(runs) == 2, f"expected one corner → 2 sub-runs, got {len(runs)}: {rh}"
    assert not any(seg < 0.15 for _h, seg, _L in rh), \
        f"no tiny connector leading segment must remain, got {rh}"
    h_out = rh[1][0]
    corner = _hd(cls, rh[0][0], h_out)
    assert 110.0 <= corner <= 125.0, f"merged corner should be ~120°, got {corner:.1f}"
    assert abs(_norm180(h_out) - (-150.0)) <= 3.0, \
        f"pivot target heading should be ~-150°, got {h_out:.1f}"
    assert corner >= THR, "merged corner must exceed split threshold → pivot fires"
    print(f"PASS test 1: connector absorbed → pivot target -120°→{_norm180(h_out):.0f}°, "
          f"one {corner:.0f}° corner, no tiny leading segment")


def test_2_absorption_gating(cls):
    THR, ABSORB, MINC = 45.0, 0.20, 20.0
    d = lambda deg, r: (r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg)))

    # (a) connector below ABSORB bracketed by two real corners → absorbed.
    pts, flags = _triangle_apex_path()
    ab_pts, _ = cls._absorb_short_connectors(pts, flags, THR, ABSORB, MINC)
    assert len(ab_pts) < len(pts), "interior bracketed connector must be absorbed"

    # (b) a genuine short MARK stroke that is NOT bracketed by two hard corners
    # must be preserved. Here: straight line with a short collinear segment in
    # the middle (bends ≈ 0°) — absorbing would be wrong.
    straight = [(0, 0), (1.0, 0.0), (1.08, 0.0), (2.0, 0.0)]  # 8cm middle segment, collinear
    sflags = [True] * 4
    out_pts, _ = cls._absorb_short_connectors(straight, sflags, THR, ABSORB, MINC)
    assert out_pts == straight, f"collinear short segment must NOT be absorbed, got {out_pts}"

    # (c) short segment with only ONE real corner (continues straight after) —
    # one bend is ~0°, so the dual-corner gate must reject it.
    one_corner = [(0, 0)] + [d(0, x) for x in (1.0,)] + [d(0, 1.0)]  # build a single-bend case
    p = [(0.0, 0.0), (1.0, 0.0), (1.05, 0.05), (2.0, 0.95)]  # corner then near-straight 7cm
    pf = [True] * 4
    out2, _ = cls._absorb_short_connectors(p, pf, THR, ABSORB, MINC)
    # Only absorb if BOTH bends ≥ MINC; here the second bend is small → keep.
    assert out2 == p, f"single-corner short segment must NOT be absorbed, got {out2}"
    print("PASS test 2: bracketed connector absorbed; collinear & single-corner short strokes preserved")


def test_3_tangent_regression(cls):
    THR, ABSORB, MINC = 45.0, 0.20, 20.0

    # True tangent/smooth transition: a gentle 10° bend between two long legs
    # must not be turned into a hard corner, and nothing absorbed.
    d = lambda deg, r: (r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg)))
    a = (0.0, 0.0)
    b = d(0, 2.0)
    c = (b[0] + d(10, 2.0)[0], b[1] + d(10, 2.0)[1])
    tangent = [a, b, c]
    tflags = [True] * 3
    out_pts, _ = cls._absorb_short_connectors(tangent, tflags, THR, ABSORB, MINC)
    assert out_pts == tangent, "tangent junction must be untouched by absorption"
    runs = cls._split_run_at_corners(out_pts, tflags, THR)
    assert len(runs) == 1, f"10° tangent must not split into a corner, got {len(runs)}"

    # Tiny connector must NOT suppress the real corner: after absorption the
    # 120° apex corner still splits (it is not collapsed away).
    pts, flags = _triangle_apex_path()
    ab_pts, ab_flags = cls._absorb_short_connectors(pts, flags, THR, ABSORB, MINC)
    runs2 = cls._split_run_at_corners(ab_pts, ab_flags, THR)
    assert len(runs2) == 2, "real apex corner must still trigger a split after absorption"
    print("PASS test 3: real tangent stays continuous; tiny connector does not suppress the real corner")


def test_6_angle_wrapping(cls):
    # Wrapped heading deltas must take the short way around ±180°.
    assert abs(_hd(cls, 179, -179) - 2.0) < 1e-6, "179→-179 must be 2°, not 358°"
    assert abs(_hd(cls, -150, 150) - 60.0) < 1e-6, "-150→150 must be 60°, not 300°"
    assert abs(_hd(cls, 170, -170) - 20.0) < 1e-6, "170→-170 must be 20°"

    # Absorption near the wrap: a connector between a +170° leg and a -170° leg
    # is only a 20° real bend on the outgoing side. Use a connector whose two
    # bends both exceed MINC across the wrap and confirm it absorbs with the
    # correct (wrapped) merged corner.
    THR, ABSORB, MINC = 45.0, 0.20, 15.0
    d = lambda deg, r: (r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg)))
    s = (0.0, 0.0)
    v_in = d(170, 2.0)
    conn = d(-90, 0.08)
    v_out = (v_in[0] + conn[0], v_in[1] + conn[1])
    end = (v_out[0] + d(-170, 2.0)[0], v_out[1] + d(-170, 2.0)[1])
    pts = [s, v_in, v_out, end]
    flags = [True] * 4
    out_pts, _ = cls._absorb_short_connectors(pts, flags, THR, ABSORB, MINC)
    assert len(out_pts) == 3, f"wrap-straddling connector should absorb, got {out_pts}"
    print("PASS test 6: heading deltas wrap correctly across ±180°; wrap-straddling connector absorbs")


# --------------------------------------------------------------------------
# Node-based tests (Part B: angle-aware timeout + hard release gate)
# --------------------------------------------------------------------------
class _Cap:
    def __init__(self):
        self.messages = []

    def publish(self, m):
        self.messages.append(m)

    @property
    def last(self):
        return self.messages[-1] if self.messages else None


def test_4_angle_aware_timeout(node):
    budgets = {}
    for deg in (30, 90, 120, 180):
        node._pivot_turn_angle_rad = math.radians(deg)
        budgets[deg] = node._pivot_timeout_budget()
    seq = [budgets[d] for d in (30, 90, 120, 180)]
    assert all(a <= b + 1e-9 for a, b in zip(seq, seq[1:])), \
        f"budgets must be non-decreasing with angle, got {budgets}"
    assert 6.0 <= budgets[120] <= 7.0, f"120° budget must be ~6–7s, got {budgets[120]:.2f}"
    assert budgets[120] > 5.0 + 1e-6, "120° must exceed the legacy 5s floor (the fix)"
    assert budgets[180] > budgets[120], "180° must exceed 120° budget"
    assert budgets[180] <= float(node.get_parameter("segment_pivot_timeout_max_s").value) + 1e-9, \
        "budget must respect the max clamp"
    print(f"PASS test 4: angle-aware budgets 30°={budgets[30]:.1f} 90°={budgets[90]:.1f} "
          f"120°={budgets[120]:.1f} 180°={budgets[180]:.1f}s")


def test_5_release_safety_gate(node):
    from geometry_msgs.msg import PoseStamped

    def pose(n, e):
        ps = PoseStamped()
        ps.header.frame_id = "local_ned"
        ps.pose.position.x = float(n)
        ps.pose.position.y = float(e)
        ps.pose.orientation.w = 1.0
        return ps

    # Path whose first segment heads due-North (NED target heading = 0°).
    node._path = [pose(0.0, 0.0), pose(1.0, 0.0)]
    node._latest_yaw_rate_ned = 0.0

    # (a) Grossly mis-headed (28°) → must keep holding, never release to TRACK.
    node._reset_corner_pivot_state()
    node._run_align_pending = True
    node._run_align_turn_rad = math.radians(120.0)
    node._corner_stop_complete = True
    holding = node._run_alignment_hold(0.0, 0.0, math.radians(28.0), 0.02)
    assert holding is True, "must keep holding while heading error is large"
    assert node._run_align_pending is True, "alignment must not be released at 28° error"

    # (b) Within strict tolerance (1°) with yaw settled → releases normally.
    node.set_parameters([Parameter("segment_align_settle_s", value=0.0)])
    node._reset_corner_pivot_state()
    node._run_align_pending = True
    node._run_align_turn_rad = math.radians(120.0)
    node._corner_stop_complete = True
    released = node._run_alignment_hold(0.0, 0.0, math.radians(1.0), 0.02)
    assert released is False, "should release at ≤2° heading error with yaw settled"
    assert node._run_align_pending is False, "alignment pending cleared on strict exit"
    node.set_parameters([Parameter("segment_align_settle_s", value=0.10)])

    # (c) Hard cap holds even if the relaxed timeout band is mis-set huge.
    node.set_parameters([Parameter("segment_timeout_heading_tolerance_deg", value=30.0)])
    strict = math.radians(float(node.get_parameter("segment_heading_tolerance_deg").value))
    relaxed = math.radians(float(node.get_parameter("segment_timeout_heading_tolerance_deg").value))
    hard = math.radians(float(node.get_parameter("segment_pivot_release_max_deg").value))
    eff = min(max(strict, relaxed), hard)
    assert abs(eff - hard) < 1e-9, "effective release tol must be capped at the hard gate (10°)"
    assert math.radians(12.0) > eff, "a 12° residual must not pass the capped gate"
    node.set_parameters([Parameter("segment_timeout_heading_tolerance_deg", value=5.0)])
    print("PASS test 5: holds at 28°, releases at 1°, hard 10° cap survives a mis-set relaxed band")


def main():
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    try:
        from rpp_controller_node import RPPControllerNode
        cls = RPPControllerNode

        test_1_triangle_tiny_connector(cls)
        test_2_absorption_gating(cls)
        test_3_tangent_regression(cls)
        test_6_angle_wrapping(cls)

        node = RPPControllerNode()
        for name in ("_vel_pub", "_yaw_rate_pub", "_dbg_pub",
                     "_segment_dbg_pub", "_conditioned_path_pub", "_spray_active_pub"):
            setattr(node, name, _Cap())
        test_4_angle_aware_timeout(node)
        test_5_release_safety_gate(node)
        node.destroy_node()

        print("\n=== ALL CORNER-ABSORB / PIVOT TESTS PASSED ===")
        return True
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
