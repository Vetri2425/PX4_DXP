#!/usr/bin/env python3
"""D15 endpoint lookahead extension (in-env, needs rclpy).

The bug (measured 2026-08-04, bags stg_8644443e 12:53/12:54 + ULog log_5/6):
`_segment_lookahead_point` clipped the aim point at the FINAL vertex, so the
aim point stopped moving while the rover kept closing on it. The ACTUAL
lookahead distance decayed 0.452 -> 0.217 -> 0.062 m (run2: 0.024 m) through
the terminal braking of a STRAIGHT 2-point transit hop. Pure-pursuit steering
gain goes as 1/L, so 1 cm of cross-track at L=0.024 m commands ~23 deg of
course change — and the ULog shows the yaw setpoint swinging -17.8/-18.2 deg
with only 3.9-4.5 % steering authority used.

`min_lookahead_dist` cannot fix it: past the final vertex there is no path
left to walk, so the floor is unreachable. Raising it 0.35 -> 0.45 made the
arrival walk worse.

Load-bearing invariants:
  * when l_d fits inside the path, the extension is never reached — output is
    byte-identical to the old behaviour;
  * a REAL corner still clips (we must not steer past a turn);
  * with extend_past_end=False the old geometry is reproduced exactly.

Run:  python3 -m pytest -q test_endpoint_lookahead.py   (Jetson only — rclpy)
"""

import math

import rclpy

from test_smoke_rpp_controller import _make_path_pose


def _node(path):
    from rpp_controller_node import RPPControllerNode

    node = RPPControllerNode()
    node._path = [_make_path_pose(n, e) for (n, e) in path]
    return node


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def test_extends_past_final_vertex_holding_l_d():
    """The whole point: L_actual stays ~l_d instead of decaying to zero."""
    rclpy.init()
    try:
        # 1 m due-north path; rover 0.9 m along it, so only 0.1 m of path left.
        node = _node([(0.0, 0.0), (1.0, 0.0)])
        l_d = 0.45
        lh = node._segment_lookahead_point(0, 0.9, 0.0, l_d, 5.0, True)
        # Aim point must sit l_d ahead of the foot, i.e. 0.35 m PAST the end.
        assert abs(lh[0] - 1.35) < 1e-9, lh
        assert abs(lh[1] - 0.0) < 1e-9, lh
        assert abs(_dist(lh, (0.9, 0.0)) - l_d) < 1e-9
    finally:
        rclpy.shutdown()


def test_old_behaviour_collapses_without_the_fix():
    """Documents the bug: pinned aim point, lookahead decays toward zero."""
    rclpy.init()
    try:
        node = _node([(0.0, 0.0), (1.0, 0.0)])
        for foot, expect_L in ((0.55, 0.45), (0.90, 0.10), (0.98, 0.02)):
            old = node._segment_lookahead_point(0, foot, 0.0, 0.45, 5.0, False)
            assert abs(old[0] - 1.0) < 1e-9          # pinned to the endpoint
            assert abs(_dist(old, (foot, 0.0)) - expect_L) < 1e-9
            new = node._segment_lookahead_point(0, foot, 0.0, 0.45, 5.0, True)
            assert abs(_dist(new, (foot, 0.0)) - 0.45) < 1e-9
    finally:
        rclpy.shutdown()


def test_no_change_when_l_d_fits_inside_the_path():
    """Reduction is exact away from the endpoint — not an approximation."""
    rclpy.init()
    try:
        node = _node([(0.0, 0.0), (5.0, 0.0)])
        on = node._segment_lookahead_point(0, 1.0, 0.0, 0.45, 5.0, True)
        off = node._segment_lookahead_point(0, 1.0, 0.0, 0.45, 5.0, False)
        assert on == off
        assert abs(on[0] - 1.45) < 1e-12
    finally:
        rclpy.shutdown()


def test_real_corner_still_clips():
    """We must NOT steer past a turn — only the path END is extended."""
    rclpy.init()
    try:
        # north 1 m, then hard 90 deg east: the vertex at (1,0) is a corner.
        node = _node([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
        lh = node._segment_lookahead_point(0, 0.9, 0.0, 0.45, 5.0, True)
        assert abs(lh[0] - 1.0) < 1e-9 and abs(lh[1] - 0.0) < 1e-9, lh
    finally:
        rclpy.shutdown()


def test_collinear_vertex_still_crossed_then_end_extended():
    """Collinear anchors are walked through, AND the far end still extends.

    Path is a straight 1 m run split by a collinear anchor at 0.5 m (the
    spray-boundary/must-hit case). From foot 0.2 m an l_d of 1.0 m must cross
    the anchor and then run 0.2 m past the endpoint — exercising both fixes in
    one walk.
    """
    rclpy.init()
    try:
        node = _node([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)])
        lh = node._segment_lookahead_point(0, 0.2, 0.0, 1.0, 5.0, True)
        assert abs(lh[0] - 1.2) < 1e-9, lh
        assert abs(_dist(lh, (0.2, 0.0)) - 1.0) < 1e-9
    finally:
        rclpy.shutdown()


def test_degenerate_foot_on_endpoint_uses_segment_direction():
    """Rover exactly on the endpoint: bearing must stay defined, not NaN."""
    rclpy.init()
    try:
        node = _node([(0.0, 0.0), (1.0, 0.0)])
        lh = node._segment_lookahead_point(0, 1.0, 0.0, 0.45, 5.0, True)
        assert abs(lh[0] - 1.45) < 1e-9 and abs(lh[1] - 0.0) < 1e-9, lh
        assert all(math.isfinite(c) for c in lh)
    finally:
        rclpy.shutdown()


def test_extension_follows_a_diagonal_final_bearing():
    """Direction comes from the final segment, not from an axis assumption."""
    rclpy.init()
    try:
        node = _node([(0.0, 0.0), (1.0, 1.0)])               # 45 deg
        foot = (0.9, 0.9)
        lh = node._segment_lookahead_point(0, foot[0], foot[1], 0.45, 5.0, True)
        assert abs(_dist(lh, foot) - 0.45) < 1e-9
        assert abs((lh[0] - foot[0]) - (lh[1] - foot[1])) < 1e-9   # still 45 deg
    finally:
        rclpy.shutdown()
