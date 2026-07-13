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


if __name__ == "__main__":
    unittest.main(verbosity=2)
