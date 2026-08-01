#!/usr/bin/env python3
"""Regression tests for analyze_mission's CDR parsers and pure metric helpers.

No bag or ROS needed — hand-built CDR buffers validate the little-endian XCDR1
reader (esp. member alignment), and synthetic series validate the stat / corner
/ edge helpers.

Run:  python3 -m pytest tools/test_analyze_mission.py     ← full suite (51)
      python3 tools/test_analyze_mission.py               ← unittest classes only (23)

The file is a hybrid: unittest.TestCase classes plus bare pytest-style
`def test_*` functions. `unittest.main()` cannot see the latter, so **pytest is
the authoritative runner**. (Until 2026-07-30 the `__main__` block also sat
mid-file, so the script path ran 18 of 51 and still printed OK.)
"""
import json
import math
import os
import struct
import sys
import unittest

import pytest

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


# ── §12 GEO OVERLAY ──────────────────────────────────────────────────────────
# Renders surveyed / commanded-/path / driven into lat/lon and writes a map
# overlay. Needs the EKF origin (gp_origin) to geo-reference /path.

def test_geo_overlay_writes_three_layers_and_placement(tmp_path):
    s = _series_at()
    s.ekf_origin = (_LAT0, _LON0)                 # local (0,0) == this origin
    out = am.analyze_geo(s, _manifest(tmp_path), str(tmp_path))
    assert out["available"], out.get("reason")
    assert out["n_surveyed"] == 2 and out["n_commanded"] == 2 and out["n_driven"] == 2
    # commanded = geodesic inverse of the (0,0)->truth path → sits ON the survey
    assert out["placement_mean_cm"] < 2.0
    gj = json.loads((tmp_path / "geo_overlay.geojson").read_text())
    layers = {f["properties"]["layer"] for f in gj["features"]}
    assert layers == {"surveyed", "commanded_path", "driven"}
    # GeoJSON coordinates are [lon, lat] — sanity-check the ordering.
    surveyed = next(f for f in gj["features"] if f["properties"]["layer"] == "surveyed")
    lon0, lat0 = surveyed["geometry"]["coordinates"][0]
    assert abs(lat0 - _LAT0) < 1e-6 and abs(lon0 - _LON0) < 1e-6
    assert (tmp_path / "geo_overlay.csv").is_file()


def test_geo_overlay_without_ekf_origin_still_exports_driven(tmp_path):
    s = _series_at()
    s.ekf_origin = None                           # no gp_origin captured
    out = am.analyze_geo(s, _manifest(tmp_path), str(tmp_path))
    assert out["available"]                       # surveyed + driven still export
    assert out["n_commanded"] == 0
    assert "no EKF origin" in out["reason"]


def test_geo_overlay_placement_responds_to_a_wrong_origin(tmp_path):
    """A 30 cm-shifted EKF origin makes commanded-geo miss the survey by ~30 cm."""
    s = _series_at()
    mn, _me = _mps(_LAT0)
    s.ekf_origin = (_LAT0 + 0.30 / mn, _LON0)     # origin 30 cm north
    out = am.analyze_geo(s, _manifest(tmp_path), str(tmp_path))
    assert out["placement_mean_cm"] > 25.0


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


# ── §9 TRAVERSAL ───────────────────────────────────────────────────────────────
# A4: manifest.outcome.status is hardcoded COMPLETE by the recorder whenever it
# shuts down cleanly, so a run that aborted at 40% is indistinguishable from a
# full one. These pin the coverage measure that tells them apart.

def _straight_path(n=40, step=1.0):
    """A 40 m straight line, one point per metre.

    Spacing is deliberately wider than COVERAGE_RADIUS_M. At 25 cm spacing the
    point one past the stop sits exactly on the radius and counts as reached,
    so an abort at 50% measures 52.5% — a real property of the metric, but it
    would make these assertions about tie-breaking rather than about coverage.
    """
    return [(i * step, 0.0) for i in range(n)]


def _series_driving(path, fraction=1.0, skip=None):
    """Rover follows `path` exactly, for `fraction` of it. `skip` = (lo, hi)
    index range it never visits (but it does come back afterwards)."""
    s = am.Series()
    s.path = list(path)
    s.paths = [(0.0, list(path))]
    stop_at = int(len(path) * fraction)
    driven = []
    for i, (n, e) in enumerate(path[:stop_at]):
        if skip and skip[0] <= i < skip[1]:
            continue
        driven.append((n, e))
    s.pose = [(i * 0.1, n, e, 0.0) for i, (n, e) in enumerate(driven)]
    return s


def test_full_run_is_complete():
    t = am.analyze_traversal(_series_driving(_straight_path()))
    assert t["available"]
    assert t["status"] == "COMPLETE"
    assert t["coverage"] == 1.0
    assert t["verdict"] == "PASS"
    assert t["shape"] == "FULL"
    assert t["missing_leading"] == t["missing_trailing"] == t["missing_interior"] == 0


def test_abort_partway_is_detected_and_fails():
    t = am.analyze_traversal(_series_driving(_straight_path(), fraction=0.375))
    assert t["status"] == "PARTIAL"
    assert t["verdict"] == "FAIL"
    assert t["points_covered"] == 15 and t["points_total"] == 40
    assert t["shape"] == "STOPPED_EARLY"
    assert t["last_covered_index"] == 14
    assert t["missing_trailing"] == 25 and t["missing_leading"] == 0


def test_never_drove_the_start_is_not_called_stopping_early():
    """The real 2026-07-22 bag: 24/64 covered, but they were the LAST 24 — the
    rover never reached the beginning. A two-way abort/skip split called this
    'not stopped early' and said nothing more, which is why `shape` exists."""
    path = _straight_path()
    s = am.Series()
    s.path = list(path)
    s.paths = [(0.0, list(path))]
    s.pose = [(i * 0.1, n, e, 0.0) for i, (n, e) in enumerate(path[25:])]
    t = am.analyze_traversal(s)
    assert t["shape"] == "STARTED_LATE"
    assert t["stopped_early"] is False
    assert t["missing_leading"] == 25 and t["missing_trailing"] == 0
    assert t["first_covered_index"] == 25
    assert t["last_covered_index"] == 39


def test_a_gap_mid_path_is_not_reported_as_stopping_early():
    """Skipping geometry and coming back is a different failure from an abort."""
    t = am.analyze_traversal(_series_driving(_straight_path(), skip=(10, 20)))
    assert t["status"] != "COMPLETE"
    assert t["shape"] == "INTERIOR_GAP"
    assert t["stopped_early"] is False
    assert t["missing_interior"] == 10
    assert t["last_covered_index"] == 39   # it did come back


def test_middle_only_run():
    path = _straight_path()
    s = am.Series()
    s.path = list(path)
    s.paths = [(0.0, list(path))]
    s.pose = [(i * 0.1, n, e, 0.0) for i, (n, e) in enumerate(path[10:30])]
    t = am.analyze_traversal(s)
    assert t["shape"] == "MIDDLE_ONLY"
    assert t["missing_leading"] == 10 and t["missing_trailing"] == 10


def test_rover_nowhere_near_the_path():
    path = _straight_path()
    s = am.Series()
    s.path = list(path)
    s.paths = [(0.0, list(path))]
    s.pose = [(i * 0.1, 500.0, 500.0, 0.0) for i in range(10)]
    t = am.analyze_traversal(s)
    assert t["shape"] == "NONE"
    assert t["points_covered"] == 0 and t["coverage"] == 0.0
    assert t["verdict"] == "FAIL"


def test_coverage_uses_distance_not_crosstrack():
    """A rover stopped dead ON the line has zero xtrack for the rest of the
    mission it never drove — that is exactly the run this must catch."""
    t = am.analyze_traversal(_series_driving(_straight_path(), fraction=0.5))
    assert t["coverage"] == 0.5


def test_radius_is_honoured():
    path = _straight_path(n=4)
    s = am.Series()
    s.path = list(path)
    s.paths = [(0.0, list(path))]
    # rover passes 40 cm to the side of every point
    s.pose = [(i * 0.1, n, e + 0.40, 0.0) for i, (n, e) in enumerate(path)]
    assert am.analyze_traversal(s, radius_m=0.25)["points_covered"] == 0
    assert am.analyze_traversal(s, radius_m=0.50)["points_covered"] == 4


def test_grid_bucketing_matches_bruteforce_on_a_2d_shape():
    """The spatial grid is an optimisation; it must not change the answer.
    Checked against the naive all-pairs scan it replaced."""
    path = [(math.cos(i / 20.0) * 3.0, math.sin(i / 20.0) * 3.0) for i in range(126)]
    s = am.Series()
    s.path = path
    s.paths = [(0.0, path)]
    s.pose = [(i * 0.1, n + 0.05, e - 0.03, 0.0)
              for i, (n, e) in enumerate(path) if i % 3]
    r = 0.25
    brute = sum(
        1 for (pn, pe) in path
        if any(math.hypot(p[1] - pn, p[2] - pe) <= r for p in s.pose)
    )
    assert am.analyze_traversal(s, radius_m=r)["points_covered"] == brute


def test_no_path_or_no_pose_is_unavailable_not_zero_coverage():
    """Unavailable must never be reported as 0% — that would fail every bag
    recorded before /path was captured."""
    empty = am.Series()
    assert am.analyze_traversal(empty)["available"] is False
    s = am.Series()
    s.path = _straight_path()
    s.paths = [(0.0, s.path)]
    assert am.analyze_traversal(s)["available"] is False


def test_manifest_writeback_adds_only_traversal(tmp_path):
    import json as _json
    mpath = tmp_path / "manifest.json"
    original = {"identity": {"mission_id": "m1"},
                "outcome": {"status": "COMPLETE", "recorder_end": {"utc": "2026"}}}
    mpath.write_text(_json.dumps(original))

    out = am._write_traversal_to_manifest(str(tmp_path), {"status": "PARTIAL",
                                                          "coverage": 0.375})
    assert out == str(mpath)
    got = _json.loads(mpath.read_text())
    assert got["traversal"]["status"] == "PARTIAL"
    assert got["outcome"] == original["outcome"]   # untouched
    assert got["identity"] == original["identity"]


def test_manifest_writeback_is_silent_when_there_is_no_manifest(tmp_path):
    """A bare rosbag dir is a valid input; it just has nowhere to record this."""
    assert am._write_traversal_to_manifest(str(tmp_path), {"status": "COMPLETE"}) is None


# ── survey tolerance is the SURVEY's property, not the analyser's ──────────────
# It decides whether §7 FAILs a run, so where the number came from is part of
# the verdict. A threshold nobody chose cannot justify either a PASS or a FAIL.

def test_default_when_nothing_is_set():
    tol, src = am.resolve_survey_tol_cm({})
    assert tol == am.SURVEY_TOL_CM
    assert "not set" in src


def test_operator_value_staged_with_the_mission_wins():
    tol, src = am.resolve_survey_tol_cm(
        {"staged_mission": {"survey_tolerance_m": 0.008}})
    assert tol == 0.8
    assert "operator-set" in src


def test_cli_overrides_the_staged_value():
    tol, src = am.resolve_survey_tol_cm(
        {"staged_mission": {"survey_tolerance_m": 0.008}}, cli_cm=4.0)
    assert tol == 4.0
    assert src == "--survey-tol-cm"


def test_unusable_staged_values_fall_back_and_say_so():
    """Silently judging a run by a number nobody chose is the failure mode."""
    for bad in ("abc", -1.0, 0.0, 5.0):   # non-numeric, negative, zero, an absurd 5 m
        tol, src = am.resolve_survey_tol_cm(
            {"staged_mission": {"survey_tolerance_m": bad}})
        assert tol == am.SURVEY_TOL_CM, bad
        assert "default" in src and str(bad) in src, (bad, src)


def test_tolerance_actually_changes_the_intent_verdict():
    """The whole point: the same geometry judged by two surveys' precision.

    A vertex 3.43 cm off the chord is INTENT at the 2.5 cm default, but is
    within noise for a sloppier survey that declared 5 cm.
    """
    V = TestGeometryFidelity.V
    s = am.Series()
    s.paths = [(0.0, _densify(V))]
    s.cond_paths = [(0.0, [V[0], V[-1]])]
    s.pose = [(i * 0.1, n, e, 0.0) for i, (n, e) in enumerate(_densify([V[0], V[-1]]))]

    strict = am.analyze_geometry_fidelity(s, survey_tol_cm=2.5)
    loose = am.analyze_geometry_fidelity(s, survey_tol_cm=5.0)

    assert strict["dropped_total"] == loose["dropped_total"] == 2   # same geometry
    assert strict["dropped_above_tolerance"] == 2 and strict["verdict"] == "FAIL"
    assert loose["dropped_above_tolerance"] == 0 and loose["verdict"] == "WARN"


def test_source_is_reported_in_the_result():
    s = am.Series()
    s.paths = [(0.0, _densify(TestGeometryFidelity.V))]
    s.cond_paths = [(0.0, list(TestGeometryFidelity.V))]
    s.pose = [(i * 0.1, n, e, 0.0)
              for i, (n, e) in enumerate(_densify(TestGeometryFidelity.V))]
    g = am.analyze_geometry_fidelity(s, survey_tol_cm=3.0, survey_tol_source="unit test")
    assert g["survey_tol_cm"] == 3.0
    assert g["survey_tol_source"] == "unit test"


# ---------------------------------------------------------------------------
# marking_only must be gated on the VALVE, not on the RPP's geometric intent.
#
# Field 2026-07-30, bag stg_1cad4f00_..._132302: gating on /spray/active
# reported marking RMS 4.05 cm / max 9.70 (FAIL) where the valve-gated truth
# was 1.33 cm. /spray/active is `_spray_flags[seg] and _spray_flags[seg+1]` —
# the path's INTENT — and led the valve by 1.8 s, so the window swallowed the
# dry recovery from a pivot that released 17.47° off heading. There was no
# coverage of analyze_tracking at all, which is how it shipped.

def _tracking_series(rpp_xt, active_win, state_win, dt=0.1):
    """rpp_xt = [xtrack_m]; *_win = (t_on, t_off) for that boolean signal."""
    s = am.Series()
    s.rpp = [(i * dt, [x] + [0.0] * 10) for i, x in enumerate(rpp_xt)]
    n = len(rpp_xt)
    for win, dest in ((active_win, "spray_active"), (state_win, "spray_state")):
        if win is None:
            continue
        on, off = win
        getattr(s, dest).extend(
            (i * dt, on <= i * dt < off) for i in range(n)
        )
    return s


class TestMarkingGate(unittest.TestCase):
    # 0.0-1.0 s: 10 cm excursion (dry approach). 1.0-2.0 s: 1 cm (painted).
    XT = [0.10] * 10 + [0.01] * 10

    def test_gates_on_spray_state_not_active(self):
        s = _tracking_series(self.XT, active_win=(0.0, 2.0), state_win=(1.0, 2.0))
        t = am.analyze_tracking(s)
        self.assertEqual(t["marking_gate"], "spray_state")
        self.assertTrue(t["marking_gate_is_valve"])
        # Painted span only → 1 cm, not the 10 cm dry excursion.
        self.assertAlmostEqual(t["marking_only"]["rms_cm"], 1.0, places=1)
        self.assertEqual(t["verdict"], "PASS")

    def test_dry_approach_is_reported_separately(self):
        s = _tracking_series(self.XT, active_win=(0.0, 2.0), state_win=(1.0, 2.0))
        t = am.analyze_tracking(s)
        self.assertIsNotNone(t["approach_dry"])
        self.assertAlmostEqual(t["approach_dry"]["rms_cm"], 10.0, places=1)
        # ...and never folded into the paint verdict.
        self.assertEqual(t["verdict_basis"], "marking_only")

    def test_intent_gating_would_have_inflated_it(self):
        """Pins the magnitude of the bug this fix removes."""
        s = _tracking_series(self.XT, active_win=(0.0, 2.0), state_win=None)
        t = am.analyze_tracking(s)
        self.assertEqual(t["marking_gate"], "spray_active")
        self.assertFalse(t["marking_gate_is_valve"])   # flagged in the report
        # Both spans averaged: sqrt((10²+1²)/2) ≈ 7.1 cm — the over-report.
        self.assertGreater(t["marking_only"]["rms_cm"], 5.0)

    def test_falls_back_through_the_chain_and_says_so(self):
        s = _tracking_series(self.XT, active_win=None, state_win=None)
        s.spray_commanded = [(i * 0.1, 1.0 <= i * 0.1 < 2.0) for i in range(20)]
        t = am.analyze_tracking(s)
        self.assertEqual(t["marking_gate"], "spray_commanded")
        self.assertTrue(t["marking_gate_is_valve"])
        self.assertAlmostEqual(t["marking_only"]["rms_cm"], 1.0, places=1)

    def test_no_spray_signal_falls_back_to_overall(self):
        s = _tracking_series(self.XT, active_win=None, state_win=None)
        t = am.analyze_tracking(s)
        self.assertIsNone(t.get("marking_only"))
        self.assertEqual(t["verdict_basis"], "overall")


# Must stay LAST: unittest.main() only sees classes already defined when it
# runs. It previously sat mid-file, so `python3 tools/test_analyze_mission.py`
# executed 18 of 51 tests and reported OK — every test below that point (§8
# absolute accuracy, geometry fidelity, the marking gate) was dead unless the
# suite happened to be run under pytest.
if __name__ == "__main__":
    unittest.main(verbosity=2)


# ── 2026-08-01: the two defects that made the analyser lie in BOTH directions ──
#
# Bag stg_9ecf2985 (2026-08-01 15:47) overshot its endpoint by 54.6 cm and this
# analyser reported "worst coast-past 0.0cm ... verdict PASS", while grading the
# whole run WARN on nine OFFBOARD "drops" that never happened. The 51 tests above
# all passed against that code, so neither defect had any coverage.

def _overshoot_series(overshoot_m):
    """Rover drives a 3 m north line, passes the endpoint, rests `overshoot_m`
    past it — the exact shape that made the endpoint search run off the end."""
    path = [(i * 0.1, 0.0) for i in range(31)]          # 0 .. 3.0 m north
    s = am.Series()
    s.path = list(path)
    s.paths = [(0.0, list(path))]
    driven = [(i * 0.02, 0.0) for i in range(int(3.0 / 0.02) + 1)]
    n = 3.0
    while n < 3.0 + overshoot_m:
        n += 0.02
        driven.append((n, 0.0))
    driven += [(3.0 + overshoot_m, 0.0)] * 25           # comes to rest, stays there
    s.pose = [(i * 0.05, nn, ee, 0.0) for i, (nn, ee) in enumerate(driven)]
    s.seg = [(0.0, am.S_DONE, 0.0)]
    return s


def test_large_overshoot_is_measured_not_dropped():
    """THE REGRESSION. Resting beyond DEPART_M must not delete the measurement.

    The endpoint arrival used to anchor on "the last sample beyond DEPART_M";
    when the rover RESTS past that, the final sample matched, the search index
    ran off the end, and `continue` dropped the stop. Pinned well beyond
    DEPART_M so a re-introduction cannot hide.
    """
    st = am.analyze_stops(_overshoot_series(0.55))
    assert st["count"] == 1, f"stop was dropped: {st}"
    assert st["endpoint_resting_cm"] == pytest.approx(55.0, abs=1.5)
    assert st["worst_coast_cm"] == pytest.approx(55.0, abs=1.5), \
        "coast must not be capped at DEPART_M on the endpoint"
    assert st["verdict"] == "FAIL"


def test_unmeasurable_stop_is_never_a_pass():
    """A run that never reaches the point must read UNAVAILABLE, not PASS.

    With no stops, worst_coast stays 0.0 and resting is None, so the verdict
    expression produced a confident PASS out of an empty list.
    """
    path = [(i * 0.1, 0.0) for i in range(31)]
    s = am.Series()
    s.path = list(path); s.paths = [(0.0, list(path))]
    s.pose = [(i * 0.05, i * 0.02, 0.0, 0.0) for i in range(50)]   # stops at 1 m of 3
    st = am.analyze_stops(s)
    assert st["count"] == 0
    assert st["verdict"] == "UNAVAILABLE", "empty measurement must not grade PASS"
    assert st["endpoint_measured"] is False
    assert st["unmeasured"], "the skipped stop must be reported, not silent"


def test_offboard_drops_count_transitions_not_samples():
    """MAVROS republishes latched State; only a real edge is a drop.

    Nine duplicate MANUAL samples BEFORE the rover ever entered OFFBOARD scored
    nine 'drops' and were the sole reason a run graded WARN.
    """
    s = am.Series()
    s.state = ([(0.30 + i * 0.001, "MANUAL", True) for i in range(9)]
               + [(0.32, "OFFBOARD", True)]
               + [(1.0 + i, "OFFBOARD", True) for i in range(27)])
    h = am.analyze_health(s)
    assert h["offboard_drops"] == 0, f"phantom drops: {h['events']}"
    assert h["verdict"] == "PASS"


def test_a_real_offboard_drop_is_still_caught():
    """The fix must not blind the detector to the thing it exists for."""
    s = am.Series()
    s.state = ([(float(i), "OFFBOARD", True) for i in range(10)]
               + [(10.0 + i, "HOLD", True) for i in range(5)]      # ONE real edge
               + [(20.0 + i, "OFFBOARD", True) for i in range(5)])
    h = am.analyze_health(s)
    assert h["offboard_drops"] == 1, f"expected exactly one edge, got {h['events']}"
    assert h["verdict"] == "WARN"


def test_mode_change_while_disarmed_is_not_a_drop():
    """Leaving OFFBOARD disarmed is the operator, not a failsafe."""
    s = am.Series()
    s.state = [(0.0, "OFFBOARD", False), (1.0, "MANUAL", False), (2.0, "MANUAL", False)]
    assert am.analyze_health(s)["offboard_drops"] == 0
