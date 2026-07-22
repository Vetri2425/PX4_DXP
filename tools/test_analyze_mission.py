#!/usr/bin/env python3
"""Regression tests for analyze_mission's CDR parsers and pure metric helpers.

No bag or ROS needed — hand-built CDR buffers validate the little-endian XCDR1
reader (esp. member alignment), and synthetic series validate the stat / corner
/ edge helpers.

Run:  python3 tools/test_analyze_mission.py
"""
import math
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze_mission as am  # noqa: E402

_HDR = b"\x00\x01\x00\x00"  # CDR_LE encapsulation header


class TestCDR(unittest.TestCase):
    def test_float32multiarray_roundtrip(self):
        vals = [0.5, -1.25, 3.0, 42.0]
        buf = _HDR
        buf += struct.pack("<I", 0)          # dim_len
        buf += struct.pack("<I", 0)          # data_offset
        buf += struct.pack("<I", len(vals))  # data_len
        for v in vals:
            buf += struct.pack("<f", v)
        out = am._p_f32ma(buf)["data"]
        self.assertEqual(len(out), len(vals))
        for a, b in zip(out, vals):
            self.assertAlmostEqual(a, b, places=5)

    def test_posestamped_alignment(self):
        # header + string frame_id then 7×f64 — exercises 8-byte alignment.
        buf = _HDR
        buf += struct.pack("<i", 12)         # sec
        buf += struct.pack("<I", 340000000)  # nsec
        fid = b"map\x00"                      # length includes null
        buf += struct.pack("<I", len(fid)) + fid
        # pad to 8-byte boundary (relative to payload start) before first f64
        rel = len(buf) - 4
        buf += b"\x00" * ((-rel) % 8)
        pose = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)  # x,y,z, qx,qy,qz,qw
        for v in pose:
            buf += struct.pack("<d", v)
        out = am._p_pose(buf)
        self.assertAlmostEqual(out["x"], 1.0)
        self.assertAlmostEqual(out["y"], 2.0)
        self.assertAlmostEqual(out["qw"], 1.0)
        # qw=1 → yaw_enu 0 → yaw_ned = +pi/2
        self.assertAlmostEqual(am._yaw_ned_from_quat(out["qx"], out["qy"], out["qz"], out["qw"]),
                               math.pi / 2.0, places=6)

    def test_state_no_header(self):
        buf = _HDR
        buf += bytes([1, 1, 0, 0])           # connected, armed, guided, manual
        mode = b"OFFBOARD\x00"
        buf += struct.pack("<I", len(mode)) + mode
        buf += bytes([4])                    # system_status
        out = am._p_state(buf)
        self.assertTrue(out["connected"])
        self.assertTrue(out["armed"])
        self.assertEqual(out["mode"], "OFFBOARD")

    def test_bool_and_float32(self):
        self.assertTrue(am._p_bool(_HDR + b"\x01")["data"])
        self.assertFalse(am._p_bool(_HDR + b"\x00")["data"])
        self.assertAlmostEqual(am._p_f32(_HDR + struct.pack("<f", 0.45))["data"], 0.45, places=5)


class TestStats(unittest.TestCase):
    def test_pctl(self):
        v = [1, 2, 3, 4, 5]
        self.assertEqual(am._pctl(v, 0.5), 3)
        self.assertEqual(am._pctl(v, 0.0), 1)
        self.assertEqual(am._pctl(v, 1.0), 5)

    def test_stat_block(self):
        b = am._stat_block([0.01, -0.01, 0.02, -0.02])  # meters
        self.assertEqual(b["n"], 4)
        self.assertAlmostEqual(b["rms_cm"], 1.58, places=1)
        self.assertEqual(b["max_cm"], 2.0)
        self.assertEqual(b["left_frac"], 0.5)


class TestCornerAndEdges(unittest.TestCase):
    def test_stop_vertices_line(self):
        verts = [(0, 0), (1, 0), (2, 0)]  # straight — only endpoint is a stop
        self.assertEqual(am._stop_vertices(verts), [2])

    def test_stop_vertices_square(self):
        verts = [(0, 0), (2, 0), (2, 2), (0, 2), (0, 0)]  # 3 interior 90° corners + endpoint
        self.assertEqual(am._stop_vertices(verts), [1, 2, 3, 4])

    def test_stop_vertices_collapses_densified(self):
        # a densified straight leg then a real 90° corner
        verts = [(0, 0), (0.5, 0), (1, 0), (1.5, 0), (2, 0), (2, 1), (2, 2)]
        self.assertEqual(am._stop_vertices(verts), [4, 6])  # corner at idx4, endpoint idx6

    def test_edges(self):
        series = [(0.0, False), (0.1, True), (0.2, True), (0.3, False), (0.4, True)]
        self.assertEqual(am._edges(series),
                         [(0.0, "off"), (0.1, "on"), (0.3, "off"), (0.4, "on")])


def _densify(verts, step=0.05):
    """Resample a polyline at *step* — mirrors path_engine mark_spacing=0.05."""
    out = [verts[0]]
    for a, b in zip(verts[:-1], verts[1:]):
        n = max(1, int(round(math.dist(a, b) / step)))
        for i in range(1, n + 1):
            out.append((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n))
    return out


class TestGeometryFidelity(unittest.TestCase):
    """§9 — catches a conditioned path that no longer holds the surveyed geometry.

    Built from the 2026-07-18 field case (tes_cross_line.dxf): a 4-point survey
    whose interior vertices bend the line 1.45°/2.79° and sit 3.4/4.4 cm off the
    end-to-end chord. The controller dropped both, tracked a 2-point straight
    line, and the old report still said PASS because §1 reads the controller's
    error against its OWN conditioned path.
    """

    # the real projected vertices, local ENU metres (georef.py output)
    V = [(-0.8602, -1.2172), (-0.2858, -0.4419), (0.2663, 0.3443), (0.8797, 1.3148)]

    def _series(self, conditioned):
        s = am.Series()
        planned = _densify(self.V)
        s.paths = [(0.0, planned)]
        s.cond_paths = [(0.0, conditioned)]
        # rover drives exactly the conditioned path — a perfect follower
        s.pose = [(i * 0.1, n, e, 0.0) for i, (n, e) in enumerate(_densify(conditioned))]
        return s

    def test_dropped_vertices_are_reported_and_fail(self):
        # what RPP actually published: the bare end-to-end chord
        g = am.analyze_geometry_fidelity(self._series([self.V[0], self.V[-1]]))
        self.assertTrue(g["available"])
        self.assertEqual(g["dropped_total"], 2)
        self.assertEqual(g["dropped_above_tolerance"], 2)
        self.assertEqual(g["verdict"], "FAIL")
        devs = sorted(d["deviation_cm"] for r in g["runs"] for d in r["dropped"])
        # the chord deviations computed straight from the DXF: 3.43 and 4.41 cm
        self.assertAlmostEqual(devs[0], 3.43, delta=0.05)
        self.assertAlmostEqual(devs[1], 4.41, delta=0.05)

    def test_retained_vertices_pass(self):
        # control: conditioning kept every vertex (test_line_2.dxf behaviour)
        g = am.analyze_geometry_fidelity(self._series(list(self.V)))
        self.assertEqual(g["dropped_total"], 0)
        self.assertEqual(g["verdict"], "PASS")

    def test_deviation_below_tolerance_warns_not_fails(self):
        # survey noise, not intent: a vertex 1 cm off must NOT fail the mission
        v = [(0.0, 0.0), (1.0, 0.01), (2.0, 0.0)]
        s = am.Series()
        s.paths = [(0.0, _densify(v))]
        s.cond_paths = [(0.0, [v[0], v[-1]])]
        s.pose = [(i * 0.1, n, e, 0.0) for i, (n, e) in enumerate(_densify([v[0], v[-1]]))]
        g = am.analyze_geometry_fidelity(s, survey_tol_cm=2.5)
        self.assertEqual(g["dropped_total"], 1)
        self.assertEqual(g["dropped_above_tolerance"], 0)
        self.assertEqual(g["verdict"], "WARN")

    def test_unavailable_without_conditioned_path_geom(self):
        s = am.Series()
        s.paths = [(0.0, _densify(self.V))]
        g = am.analyze_geometry_fidelity(s)
        self.assertFalse(g["available"])
        self.assertIn("conditioned_path", g["reason"])


class TestStopsCoastRegression(unittest.TestCase):
    """The 2026-07-18 false-FAIL: `coast-past 416.9cm` on a 4.17 m mission whose
    rover rested 1.1 cm from its endpoint. Two independent causes."""

    def test_path_selection_prefers_the_mark_run(self):
        # A staged mission publishes: transit hop, mark run, 1-pt endpoint marker.
        # Taking the LAST left s.path as the single endpoint point.
        s = am.Series()
        for pts in ([(0.0, 0.0), (1.0, 0.0)],
                    _densify([(1.0, 0.0), (3.0, 0.0)]),
                    [(3.0, 0.0)]):
            if pts:
                s.paths.append((0.0, pts))
                if s.path is None or len(pts) > len(s.path):
                    s.path = pts
        self.assertEqual(len(s.path), 41)          # the mark run, not the 1-pt marker
        self.assertNotEqual(s.path, [(3.0, 0.0)])

    def _there_and_back(self):
        """Rover parked beside its own endpoint, drives 3 m out, comes back and stops.

        The old metric anchored to the FIRST moment the rover was within 10 cm of
        the endpoint — t=0 here — so the 'coast' became the whole 3 m drive.
        """
        s = am.Series()
        s.path = _densify([(0.0, 0.0), (3.0, 0.0)])
        out = [(x / 100.0, 0.0) for x in range(0, 301, 5)]     # 0 -> 3 m
        back = [(3.0, 0.0)] * 20                                # settle at the end
        track = [(0.02, 0.0)] * 10 + out + back                 # starts 2 cm from (0,0)
        s.pose = [(i * 0.1, n, e, 0.0) for i, (n, e) in enumerate(track)]
        s.seg = []
        return s

    def test_endpoint_coast_is_not_the_mission_length(self):
        st = am.analyze_stops(self._there_and_back())
        self.assertTrue(st["available"])
        ep = next(e for e in st["stops"] if e["is_endpoint"])
        self.assertLess(ep["coast_past_cm"], am.COAST_MAX_CM)
        self.assertLess(st["worst_coast_cm"], 15.0)      # was 305.7 on the real bag
        self.assertLess(ep["resting_cm"], 5.0)
        self.assertEqual(st["verdict"], "PASS")

    def test_endpoint_resting_gate_is_wired(self):
        # FINAL_STOP_MAX_CM was declared but never used — a rover that halts far
        # from its endpoint must now fail.
        # Rover reaches the endpoint, then creeps 14 cm past it and rests there.
        # 14 cm is under COAST_MAX_CM (15) but over FINAL_STOP_MAX_CM (12), so only
        # the resting gate can catch it — which is the point of wiring it.
        s = self._there_and_back()
        creep = [(3.0 + x / 100.0, 0.0) for x in range(0, 15)] + [(3.14, 0.0)] * 10
        base = [(n, e) for (_t, n, e, _y) in s.pose][:-20]
        track = base + creep
        s.pose = [(i * 0.1, n, e, 0.0) for i, (n, e) in enumerate(track)]
        st = am.analyze_stops(s)
        self.assertGreater(st["endpoint_resting_cm"], am.FINAL_STOP_MAX_CM)
        self.assertLessEqual(st["worst_coast_cm"], am.COAST_MAX_CM)
        self.assertEqual(st["verdict"], "FAIL")

    def test_meaningless_euclidean_metric_is_gone(self):
        st = am.analyze_stops(self._there_and_back())
        for e in st["stops"]:
            self.assertNotIn("max_dist_after_arrival_cm", e)


if __name__ == "__main__":
    unittest.main(verbosity=2)


# ── §8 ABSOLUTE ACCURACY ─────────────────────────────────────────────────────
# The one budget §1 and §7 structurally cannot see: both work in the local
# frame, so a wrong anchor shifts the whole mission on the ground while every
# local metric still reports centimetres.

_LAT0, _LON0 = 13.07208106, 80.26195346
_LAT1, _LON1 = 13.07206010, 80.26195184   # 2.3255 m from the first, real survey


def _mps(lat):
    return am._metres_per_degree(lat)


def _series_at(offset_n_m=0.0, offset_e_m=0.0):
    """A rover that drives the two surveyed points, optionally displaced.

    `offset_*` simulates a PLACEMENT error: the local path and the driven pose
    agree perfectly (so §1 is clean), but the global fixes are shifted.
    """
    s = am.Series()
    mn, me = _mps(_LAT0)
    # local frame: point 0 at (0,0), point 1 at its true offset
    dn = (_LAT1 - _LAT0) * mn
    de = (_LON1 - _LON0) * me
    s.path = [(0.0, 0.0), (dn, de)]
    s.path_z = [3, 3]                      # spray ON + must-hit
    for k, (n, e) in enumerate(s.path):
        t = 10.0 + k
        s.pose.append((t, n, e, 0.0))
        lat = _LAT0 + (n + offset_n_m) / mn
        lon = _LON0 + (e + offset_e_m) / me
        s.global_fix.append((t, lat, lon, 0.0))
    return s


def _manifest(tmp_path):
    csv = tmp_path / "survey.csv"
    csv.write_text(
        "Name,Code,Latitude,Longitude\n"
        f"1,L_1,{_LAT0},{_LON0}\n"
        f"2,L_1,{_LAT1},{_LON1}\n"
    )
    return {"staged_mission": {"source_file": str(csv)}}


def test_absolute_passes_when_the_rover_is_where_the_survey_says(tmp_path):
    out = am.analyze_absolute(_series_at(), _manifest(tmp_path))
    assert out["available"], out.get("reason")
    assert out["verdict"] == "PASS"
    assert out["max_cm"] < 1.0
    assert out["bias_cm"] < 1.0


def test_absolute_catches_a_placement_shift_local_metrics_cannot(tmp_path):
    """40 cm north shift: every local metric is perfect, §8 must still fail."""
    out = am.analyze_absolute(_series_at(offset_n_m=0.40), _manifest(tmp_path))
    assert out["available"], out.get("reason")
    assert out["verdict"] == "FAIL"
    assert 39.0 < out["bias_cm"] < 41.0
    assert out["bias_n_cm"] > 39.0
    assert out["scatter_cm"] < 1.0, "a rigid shift must show as bias, not scatter"
    assert any("PLACEMENT" in n for n in out["notes"])


def test_absolute_distinguishes_scatter_from_bias(tmp_path):
    """Equal-and-opposite errors = noise, not placement: bias small, scatter large."""
    s = _series_at()
    mn, me = _mps(_LAT0)
    s.global_fix = []
    for k, (n, e) in enumerate(s.path):
        sign = 1.0 if k == 0 else -1.0
        s.global_fix.append((10.0 + k, _LAT0 + (n + sign * 0.20) / mn,
                             _LON0 + e / me, 0.0))
    out = am.analyze_absolute(s, _manifest(tmp_path))
    assert out["bias_cm"] < 1.0, "opposite errors must cancel in the bias"
    assert out["scatter_cm"] > 15.0
    assert out["verdict"] in ("WARN", "FAIL")   # still flagged by per-point miss


def test_absolute_unavailable_without_global_fixes(tmp_path):
    s = _series_at()
    s.global_fix = []
    out = am.analyze_absolute(s, _manifest(tmp_path))
    assert not out["available"]
    assert "global_position" in out["reason"]


def test_absolute_refuses_to_guess_without_independent_ground_truth():
    """No source file => no check. It must NOT fall back to the mission's own
    anchor, which would be circular and detect nothing."""
    out = am.analyze_absolute(_series_at(), {"staged_mission": {}})
    assert not out["available"]
    assert "ground truth" in out["reason"]


def test_absolute_reports_mismatched_counts_rather_than_pairing_blindly(tmp_path):
    m = _manifest(tmp_path)
    s = _series_at()
    s.path = s.path + [(9.0, 9.0)]
    s.path_z = [3, 3, 3]
    out = am.analyze_absolute(s, m)
    assert not out["available"]
    assert "cannot pair" in out["reason"]


def test_geodesic_matches_the_field_survey():
    d = am._geodesic_m(_LAT0, _LON0, _LAT1, _LON1)
    assert abs(d - 2.3255) < 0.001
