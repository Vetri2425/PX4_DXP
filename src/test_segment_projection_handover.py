#!/usr/bin/env python3
"""`_project_onto_segment` must report PERPENDICULAR offset, not along-track.

The bug (found 2026-07-30, fixed same day): the reported cross-track spiked to
~5 cm for exactly one control cycle at every segment handover, on a path that
was perfectly straight. `seg_idx` advances one cycle before the rover reaches
the vertex, so `t_raw` goes negative, `t` clamps to 0, `foot` snaps back to the
segment's START vertex, and the old magnitude `|pos - foot|` became the
remaining ALONG-TRACK gap reported as cross-track.

Field evidence — bags/30_07_2026/stg_1bda4c36_..._151006 and
stg_bbb8e1a5_..._152342 (tes_cross_line_2, 0.5 m extensions, RTK fixed,
segment profile). Four spikes per run at the four segment boundaries:
reported -4.88 / -4.44 / +4.85 / +4.65 cm, true offset from the surveyed line
-0.34 / -0.15 / +0.44 / +0.12 cm. Those conditioned paths are COLLINEAR, so a
genuine cross-track at a handover is geometrically impossible.

⚠ These tests bind the REAL method off RPPControllerNode. Do not "simplify"
them into a local re-implementation of the projection: this repo has already
shipped three bugs whose tests mirrored the defect they were meant to catch.

Run:  python3 -m pytest src/test_segment_projection_handover.py
"""
import math
import unittest

# Importing this module installs the ROS stubs (or defers to a real rclpy) and
# imports rpp_controller_node. Reused deliberately rather than copied: it
# encodes a non-obvious real-vs-stub detection that only misbehaves when the
# suites run TOGETHER. See its docstring.
import test_curvature_lookahead_cap as _boot

RPP = _boot.RPP


class _Pt:
    def __init__(self, x, y):
        self.x, self.y = x, y


class _Pose:
    def __init__(self, x, y):
        self.position = _Pt(x, y)


class _PS:
    def __init__(self, x, y):
        self.pose = _Pose(x, y)


class _Holder:
    """Just enough of the node for _project_onto_segment."""

    _project_onto_segment = RPP._project_onto_segment
    # staticmethod on the node — rewrap, or the class-attribute assignment
    # turns them back into instance methods and injects a bogus `self`.
    _clamp = staticmethod(RPP._clamp)
    _dist = staticmethod(RPP._dist)

    def __init__(self, pts):
        self._path = [_PS(x, y) for x, y in pts]


# The real conditioned path from both field bags, as (north, east) — which is
# how nav_msgs Path stores it here (position.x = north, y = east).
FIELD_PATH = [
    (10.436, -1.376),   # lead-in start
    (10.648, -1.067),   # lead-in stub vertex
    (10.719, -0.964),   # MARK start   (must-hit)
    (12.457, 1.565),    # MARK end     (must-hit)
    (12.528, 1.668),    # run-out stub vertex
    (12.740, 1.977),    # run-out end
]


def _unit(a, b):
    dn, de = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dn, de)
    return dn / L, de / L, L


class TestFieldPathIsCollinear(unittest.TestCase):
    def test_every_vertex_lies_on_one_straight_line(self):
        """If this fails the rest of the file proves nothing."""
        un, ue, _ = _unit(FIELD_PATH[0], FIELD_PATH[-1])
        a = FIELD_PATH[0]
        for p in FIELD_PATH[1:-1]:
            rn, re = p[0] - a[0], p[1] - a[1]
            self.assertLess(abs(rn * ue - re * un), 1e-3,
                            f"{p} is off the line — path is not collinear")


class TestHandoverSpike(unittest.TestCase):
    """The regression itself: index advanced, rover not yet at the vertex."""

    def _short_of_vertex(self, seg_idx, gap_m, lateral_m=0.0):
        """Place the rover `gap_m` BEFORE the start vertex of `seg_idx`,
        offset `lateral_m` to the right of path heading — i.e. exactly the
        state on the cycle where seg_idx has advanced early."""
        h = _Holder(FIELD_PATH)
        a, b = FIELD_PATH[seg_idx], FIELD_PATH[seg_idx + 1]
        un, ue, _ = _unit(a, b)
        pos_n = a[0] - un * gap_m + ue * lateral_m
        pos_e = a[1] - ue * gap_m - un * lateral_m
        return h._project_onto_segment(pos_n, pos_e, seg_idx)

    def test_collinear_handover_reports_zero_crosstrack(self):
        """THE BUG. 3.3 cm short of the vertex on a straight path -> ~0 cm."""
        for seg_idx in range(len(FIELD_PATH) - 1):
            with self.subTest(seg_idx=seg_idx):
                _t, _fn, _fe, xt, _de = self._short_of_vertex(seg_idx, 0.033)
                self.assertLess(abs(xt), 1e-6,
                                f"seg {seg_idx}: reported {xt*100:+.2f} cm on a "
                                f"collinear path — the along-track gap leaked in")

    def test_field_exit_spike_magnitude_is_gone(self):
        """151006's exit handover: 0.033 m remaining was reported as 3.42 cm."""
        _t, _fn, _fe, xt, _de = self._short_of_vertex(4, 0.033)
        self.assertLess(abs(xt) * 100, 0.01)

    def test_the_old_formula_would_still_fail_this(self):
        """Pin that the test is actually sensitive to the defect.

        Recomputes the pre-fix expression from the same inputs; if this ever
        stops failing, the test above has gone blind.
        """
        seg_idx, gap = 4, 0.033
        h = _Holder(FIELD_PATH)
        a, b = FIELD_PATH[seg_idx], FIELD_PATH[seg_idx + 1]
        un, ue, _ = _unit(a, b)
        pos_n, pos_e = a[0] - un * gap, a[1] - ue * gap
        # Old behaviour: t clamps to 0, foot == a, magnitude = |pos - a|.
        old = math.hypot(pos_n - a[0], pos_e - a[1])
        self.assertAlmostEqual(old, gap, places=6)
        self.assertGreater(old * 100, 3.0, "the old form did report >3 cm here")

    def test_genuine_lateral_offset_survives_the_clamp(self):
        """A real 4 cm offset while 3 cm short reports 4 cm — not hypot(4,3)=5."""
        _t, _fn, _fe, xt, _de = self._short_of_vertex(2, 0.03, lateral_m=0.04)
        self.assertAlmostEqual(abs(xt), 0.04, places=6)


class TestInteriorIsUnchanged(unittest.TestCase):
    """For 0 < t < 1 the new form must equal the old one bit-for-bit-ish."""

    def test_matches_legacy_copysign_form_in_the_interior(self):
        h = _Holder(FIELD_PATH)
        seg_idx = 2                       # the 3.069 m marked line
        a, b = FIELD_PATH[seg_idx], FIELD_PATH[seg_idx + 1]
        un, ue, L = _unit(a, b)
        for frac in (0.05, 0.25, 0.5, 0.75, 0.95):
            for lat in (-0.05, -0.012, 0.0, 0.012, 0.05):
                s = frac * L
                pos_n = a[0] + un * s + ue * lat
                pos_e = a[1] + ue * s - un * lat
                t, fn, fe, xt, _de = h._project_onto_segment(pos_n, pos_e, seg_idx)
                dn, de = b[0] - a[0], b[1] - a[1]
                d = math.hypot(pos_n - fn, pos_e - fe)
                cross_z = dn * (pos_e - fe) - de * (pos_n - fn)
                legacy = math.copysign(d, cross_z) if d > 0.0 else 0.0
                self.assertGreater(t, 0.0)
                self.assertLess(t, 1.0)
                self.assertAlmostEqual(xt, legacy, places=9)

    def test_sign_convention_positive_is_right_of_heading(self):
        """NED top-down: + means the rover is to the RIGHT of path heading."""
        h = _Holder(FIELD_PATH)
        a, b = FIELD_PATH[2], FIELD_PATH[3]
        un, ue, L = _unit(a, b)
        mid_n, mid_e = a[0] + un * L / 2, a[1] + ue * L / 2
        # "Right" of heading (un, ue) in NED is (-ue, +un): for a due-north
        # heading (1, 0) that is (0, 1) — pure east. Which is right. Getting
        # this backwards is easy, hence the due-north check below.
        right = h._project_onto_segment(mid_n - ue * 0.05, mid_e + un * 0.05, 2)
        left = h._project_onto_segment(mid_n + ue * 0.05, mid_e - un * 0.05, 2)
        self.assertGreater(right[3], 0.0)
        self.assertLess(left[3], 0.0)
        self.assertAlmostEqual(right[3], 0.05, places=9)
        self.assertAlmostEqual(left[3], -0.05, places=9)

    def test_due_north_heading_right_is_east(self):
        """Anchor the sign convention on a case with no room for confusion."""
        h = _Holder([(0.0, 0.0), (10.0, 0.0)])          # straight north
        east = h._project_onto_segment(5.0, +0.05, 0)   # 5 cm east of the line
        west = h._project_onto_segment(5.0, -0.05, 0)
        self.assertAlmostEqual(east[3], +0.05, places=9)
        self.assertAlmostEqual(west[3], -0.05, places=9)


class TestStateMachineContractUnchanged(unittest.TestCase):
    """`t`, `foot` and `dist_to_end_along` must stay CLAMPED to the segment."""

    def test_t_and_dist_to_end_still_clamp_before_the_start(self):
        h = _Holder(FIELD_PATH)
        a, b = FIELD_PATH[2], FIELD_PATH[3]
        un, ue, L = _unit(a, b)
        t, fn, fe, _xt, de = h._project_onto_segment(
            a[0] - un * 0.10, a[1] - ue * 0.10, 2)
        self.assertEqual(t, 0.0)
        self.assertAlmostEqual(de, L, places=9)
        self.assertAlmostEqual(fn, a[0], places=9)
        self.assertAlmostEqual(fe, a[1], places=9)

    def test_t_and_dist_to_end_still_clamp_past_the_end(self):
        h = _Holder(FIELD_PATH)
        a, b = FIELD_PATH[2], FIELD_PATH[3]
        un, ue, _L = _unit(a, b)
        t, fn, fe, xt, de = h._project_onto_segment(
            b[0] + un * 0.10, b[1] + ue * 0.10, 2)
        self.assertEqual(t, 1.0)
        self.assertAlmostEqual(de, 0.0, places=9)
        self.assertAlmostEqual(fn, b[0], places=9)
        self.assertAlmostEqual(fe, b[1], places=9)
        # Overshoot is along-track too — it must not read as cross-track.
        self.assertLess(abs(xt), 1e-9)

    def test_degenerate_and_single_point_paths_still_return_zero(self):
        single = _Holder([(1.0, 2.0)])
        self.assertEqual(single._project_onto_segment(5.0, 5.0, 0)[3], 0.0)
        dup = _Holder([(1.0, 2.0), (1.0, 2.0)])
        self.assertEqual(dup._project_onto_segment(5.0, 5.0, 0)[3], 0.0)


class TestSourceIsPinned(unittest.TestCase):
    def test_the_along_track_leak_cannot_come_back(self):
        src = open(_boot.rc.__file__).read()
        body = src.split("def _project_onto_segment", 1)[1].split("\n    def ", 1)[0]
        self.assertIn("signed_e = cross_z / seg_len", body)
        self.assertNotIn("math.copysign(d, cross_z)", body)


if __name__ == "__main__":
    unittest.main()
