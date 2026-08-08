#!/usr/bin/env python3
"""Segment-mode lookahead must look THROUGH a collinear vertex, not clip at it.

The bug (2026-07-30): segment mode placed its lookahead with

    lookahead_along = min(max(0.0, dist_to_end_along), l_d)

so `l_d` was overridden by whatever the current segment happened to be, and
decayed toward 0 as the rover approached each vertex — bypassing
`min_lookahead_dist` (0.52) entirely.

Correct at a real corner (the rover is about to stop and pivot and must not
steer past the turn). Wrong at a COLLINEAR vertex, which is not a corner.
`_simplify_path_for_profile` deletes collinear fill, so the collinear vertices
surviving into segment mode are precisely the deliberate anchors: spray-flag
boundaries and declared must-hit points.

Field evidence — bag stg_95d11205_..._154238 (tes_cross_line_2, 0.5 m
extensions). The extension leg splits 0.375 + 0.125 at the flag-boundary
anchor, so the lookahead ran 0.286 -> 0.187 -> 0.054 m across the approach and
jumped to 0.560 m on the marked line. Steering gain goes as 1/L^2, so the
rover hit the line under a ~100x gain spike, overshot -2.2 -> +5.2 cm, and did
not recover until s = 1.4 m. Valve shut at 5.1 cm; the line painted in two
pieces. Sibling run 154444, entering without the crossing rate, stayed under
1.4 cm.

⚠ Bind the REAL method. Do not re-implement the walk here — this repo has
shipped bugs whose tests mirrored the defect they were meant to catch.

Run:  python3 -m pytest src/test_segment_lookahead_collinear.py
"""
import math
import unittest

# Installs the ROS stubs (or defers to a real rclpy) and imports the node.
import test_curvature_lookahead_cap as _boot
from test_segment_projection_handover import FIELD_PATH, _PS, _unit

RPP = _boot.RPP


class _Holder:
    _segment_lookahead_point = RPP._segment_lookahead_point
    _corner_clip = RPP._corner_clip
    _segment_angle_deg = RPP._segment_angle_deg
    _dist = staticmethod(RPP._dist)
    _heading_delta = classmethod(RPP._heading_delta.__func__)
    _angle_wrap = staticmethod(RPP._angle_wrap)

    def __init__(self, pts):
        self._path = [_PS(x, y) for x, y in pts]


def _legacy(h, seg_idx, foot_n, foot_e, l_d):
    """The exact pre-fix two-line form, for reduction checks."""
    a = h._path[seg_idx].pose.position
    b = h._path[seg_idx + 1].pose.position
    seg_len = math.hypot(b.x - a.x, b.y - a.y)
    dir_n, dir_e = (b.x - a.x) / seg_len, (b.y - a.y) / seg_len
    dist_to_end = math.hypot(b.x - foot_n, b.y - foot_e)
    along = min(max(0.0, dist_to_end), l_d)
    if along <= 1e-6:
        return b.x, b.y
    return foot_n + dir_n * along, foot_e + dir_e * along


# A right-angle path: 3 m north, then 3 m east. Junction at vertex 1 is 90deg.
CORNER_PATH = [(0.0, 0.0), (3.0, 0.0), (3.0, 3.0)]

L_D = 0.56          # the design lookahead these runs used
TOL_DEG = 5.0       # the shipped default


def _foot_short_of_vertex(path, seg_idx, gap):
    """Point on segment `seg_idx`, `gap` metres before its end vertex."""
    a, b = path[seg_idx], path[seg_idx + 1]
    un, ue, L = _unit(a, b)
    return b[0] - un * gap, b[1] - ue * gap


class TestTheFieldRegression(unittest.TestCase):
    """154238's approach: lookahead collapsed to 0.054 m on the lead-in."""

    def test_lookahead_no_longer_collapses_on_the_extension_leg(self):
        h = _Holder(FIELD_PATH)
        # 0.054 m from the end of the lead-in — the measured collapse point.
        fn, fe = _foot_short_of_vertex(FIELD_PATH, 1, 0.054)
        lh = h._segment_lookahead_point(1, fn, fe, L_D, TOL_DEG)
        reached = math.hypot(lh[0] - fn, lh[1] - fe)
        self.assertAlmostEqual(reached, L_D, places=6)

    def test_the_old_form_really_did_collapse_there(self):
        """Sensitivity guard — if this stops failing, the test above is blind."""
        h = _Holder(FIELD_PATH)
        fn, fe = _foot_short_of_vertex(FIELD_PATH, 1, 0.054)
        lh = _legacy(h, 1, fn, fe, L_D)
        self.assertAlmostEqual(math.hypot(lh[0] - fn, lh[1] - fe), 0.054, places=6)

    def test_lookahead_holds_across_the_whole_approach(self):
        """Sweep the lead-in: the design lookahead must never be truncated."""
        h = _Holder(FIELD_PATH)
        for seg_idx in (0, 1):
            a, b = FIELD_PATH[seg_idx], FIELD_PATH[seg_idx + 1]
            un, ue, L = _unit(a, b)
            for frac in (0.0, 0.25, 0.5, 0.75, 0.99):
                fn, fe = a[0] + un * L * frac, a[1] + ue * L * frac
                lh = h._segment_lookahead_point(seg_idx, fn, fe, L_D, TOL_DEG)
                got = math.hypot(lh[0] - fn, lh[1] - fe)
                with self.subTest(seg=seg_idx, frac=frac):
                    self.assertAlmostEqual(got, L_D, places=6)

    def test_lookahead_point_stays_on_the_path(self):
        """Crossing a vertex must not shortcut the corner — path is straight
        here, so the point must lie on the line to within float noise."""
        h = _Holder(FIELD_PATH)
        un, ue, _ = _unit(FIELD_PATH[0], FIELD_PATH[-1])
        a = FIELD_PATH[0]
        fn, fe = _foot_short_of_vertex(FIELD_PATH, 1, 0.054)
        lh = h._segment_lookahead_point(1, fn, fe, L_D, TOL_DEG)
        off = (lh[0] - a[0]) * ue - (lh[1] - a[1]) * un
        # FIELD_PATH is the bag's conditioned path rounded to millimetres, so
        # its vertices are collinear only to ~0.1 mm. 0.2 mm is the tightest
        # honest bound the input supports; a corner shortcut would be cm-scale.
        self.assertLess(abs(off), 2e-4)


class TestRealCornersAreUntouched(unittest.TestCase):
    def test_walk_stops_at_a_90_degree_corner(self):
        h = _Holder(CORNER_PATH)
        fn, fe = _foot_short_of_vertex(CORNER_PATH, 0, 0.10)
        lh = h._segment_lookahead_point(0, fn, fe, L_D, TOL_DEG)
        self.assertAlmostEqual(lh[0], 3.0, places=9)
        self.assertAlmostEqual(lh[1], 0.0, places=9)
        # i.e. exactly what the old form did.
        self.assertEqual(lh, _legacy(h, 0, fn, fe, L_D))

    def test_corner_at_exactly_the_threshold_is_crossed_but_beyond_is_not(self):
        for deg, crosses in ((4.9, True), (5.1, False)):
            r = math.radians(deg)
            path = [(0.0, 0.0), (1.0, 0.0),
                    (1.0 + math.cos(r), math.sin(r))]
            h = _Holder(path)
            fn, fe = _foot_short_of_vertex(path, 0, 0.05)
            lh = h._segment_lookahead_point(0, fn, fe, 0.5, TOL_DEG)
            reached = math.hypot(lh[0] - fn, lh[1] - fe)
            with self.subTest(deg=deg):
                if crosses:
                    self.assertGreater(reached, 0.4)
                else:
                    self.assertAlmostEqual(reached, 0.05, places=9)


class TestExactReduction(unittest.TestCase):
    """threshold 0.0 must reproduce the pre-fix geometry bit-for-bit."""

    def test_zero_threshold_matches_legacy_everywhere(self):
        # extend_past_end=False: this asserts the exact reduction of the
        # COLLINEAR-crossing A/B arm. The D15 endpoint extension (2026-08-04)
        # is a separate feature with its own flag and deliberately does NOT
        # reduce to legacy at the path end — that clipping was the bug.
        for path in (FIELD_PATH, CORNER_PATH):
            h = _Holder(path)
            for seg_idx in range(len(path) - 1):
                a, b = path[seg_idx], path[seg_idx + 1]
                un, ue, L = _unit(a, b)
                for frac in (0.0, 0.1, 0.5, 0.9, 1.0):
                    for l_d in (0.05, 0.3, 0.56, 2.0):
                        fn, fe = a[0] + un * L * frac, a[1] + ue * L * frac
                        got = h._segment_lookahead_point(
                            seg_idx, fn, fe, l_d, 0.0, False)
                        want = _legacy(h, seg_idx, fn, fe, l_d)
                        with self.subTest(seg=seg_idx, frac=frac, l_d=l_d):
                            self.assertAlmostEqual(got[0], want[0], places=9)
                            self.assertAlmostEqual(got[1], want[1], places=9)

    def test_matches_legacy_when_l_d_fits_inside_the_segment(self):
        """No crossing needed -> the walk must be a no-op, threshold or not."""
        h = _Holder(FIELD_PATH)
        a, b = FIELD_PATH[2], FIELD_PATH[3]          # the 3.069 m marked line
        un, ue, L = _unit(a, b)
        for frac in (0.0, 0.25, 0.5):
            fn, fe = a[0] + un * L * frac, a[1] + ue * L * frac
            got = h._segment_lookahead_point(2, fn, fe, L_D, TOL_DEG)
            want = _legacy(h, 2, fn, fe, L_D)
            self.assertAlmostEqual(got[0], want[0], places=9)
            self.assertAlmostEqual(got[1], want[1], places=9)


class TestTerminationAndDegenerateInput(unittest.TestCase):
    def test_walk_stops_at_path_end(self):
        """Pre-D15 behaviour, retained under the A/B arm only.

        Pinning the aim point at the last vertex is what made the effective
        lookahead decay to 0.024-0.062 m during terminal braking (measured
        2026-08-04) and blew up the steering gain. With the fix ON the aim
        point is extended along the final bearing instead; see
        test_endpoint_lookahead.py.
        """
        h = _Holder(FIELD_PATH)
        last = FIELD_PATH[-1]
        lh = h._segment_lookahead_point(4, last[0], last[1], 50.0, TOL_DEG, False)
        self.assertAlmostEqual(lh[0], last[0], places=9)
        self.assertAlmostEqual(lh[1], last[1], places=9)

    def test_walk_extends_past_path_end_when_enabled(self):
        """D15: the aim point must stay l_d away, not collapse onto the rover."""
        h = _Holder(FIELD_PATH)
        last = FIELD_PATH[-1]
        prev = FIELD_PATH[-2]
        un, ue, _L = _unit(prev, last)
        lh = h._segment_lookahead_point(4, last[0], last[1], 0.45, TOL_DEG, True)
        self.assertAlmostEqual(lh[0], last[0] + un * 0.45, places=9)
        self.assertAlmostEqual(lh[1], last[1] + ue * 0.45, places=9)

    def test_single_point_path_returns_that_point(self):
        h = _Holder([(1.0, 2.0)])
        self.assertEqual(h._segment_lookahead_point(0, 9.0, 9.0, L_D, TOL_DEG),
                         (1.0, 2.0))

    def test_zero_lookahead_returns_the_foot(self):
        h = _Holder(FIELD_PATH)
        a = FIELD_PATH[2]
        lh = h._segment_lookahead_point(2, a[0], a[1], 0.0, TOL_DEG)
        self.assertAlmostEqual(lh[0], a[0], places=9)
        self.assertAlmostEqual(lh[1], a[1], places=9)

    def test_out_of_range_seg_idx_is_clamped_not_crashed(self):
        h = _Holder(FIELD_PATH)
        for si in (-5, 99):
            lh = h._segment_lookahead_point(si, FIELD_PATH[0][0], FIELD_PATH[0][1],
                                            L_D, TOL_DEG)
            self.assertEqual(len(lh), 2)


class TestDefaultIsPinned(unittest.TestCase):
    def test_shipped_default_is_5_degrees(self):
        src = open(_boot.rc.__file__).read()
        self.assertIn(
            'declare_parameter("segment_lookahead_cross_collinear_deg",  5.0)', src)
        # The tracker must go through the walk, not the old inline clip.
        tracker = src.split("def _control_segment_profile", 1)[1]
        self.assertIn("self._segment_lookahead_point(", tracker)
        self.assertNotIn("lookahead_along = min(", tracker)


if __name__ == "__main__":
    unittest.main()
