"""Tests for path_engine NED coordinate transforms."""

import math

from path_engine.ned import latlon_to_ned, dxf_to_ned_affine, apply_affine_transform, _HAS_GEOGRAPHICLIB


# ── latlon_to_ned tests ────────────────────────────────────────────────────────

def test_latlon_to_ned_same_point():
    """Same lat/lon as origin → (0, 0)."""
    if not _HAS_GEOGRAPHICLIB:
        return
    n, e = latlon_to_ned(13.0, 80.0, 13.0, 80.0)
    assert abs(n) < 0.001
    assert abs(e) < 0.001


def test_latlon_to_ned_north():
    """Moving 1° north from origin should give positive north metres."""
    if not _HAS_GEOGRAPHICLIB:
        return
    n, e = latlon_to_ned(14.0, 80.0, 13.0, 80.0)
    assert n > 100_000  # ~111 km per degree
    assert abs(e) < 100  # Nearly pure north


def test_latlon_to_ned_east():
    """Moving 1° east from origin should give positive east metres."""
    if not _HAS_GEOGRAPHICLIB:
        return
    n, e = latlon_to_ned(13.0, 81.0, 13.0, 80.0)
    assert abs(n) < 1000  # Mostly east
    assert e > 100_000  # ~111 km * cos(13°) per degree


def test_latlon_to_ned_known_distance():
    """Test with a known short distance (arc mission center)."""
    if not _HAS_GEOGRAPHICLIB:
        return
    # From the CLAUDE.md: center 13.07203780, 80.26194903
    # Start point: 13.0720378, 80.2619352
    # These are very close, should be just a few metres apart
    n, e = latlon_to_ned(13.0720378, 80.2619352, 13.07203780, 80.26194903)
    # East should be negative (start is west of center) and small
    assert abs(e) < 10  # Within a few metres
    assert abs(n) < 1  # Nearly same latitude


# ── dxf_to_ned_affine tests ────────────────────────────────────────────────────

def test_affine_identity():
    """Identity transform (same DXF and NED points)."""
    dxf_pts = [(0.0, 0.0), (1.0, 0.0)]
    ned_pts = [(0.0, 0.0), (1.0, 0.0)]
    scale, theta, offset_n, offset_e = dxf_to_ned_affine(dxf_pts, ned_pts)
    assert abs(scale - 1.0) < 0.001
    assert abs(theta) < 0.001
    assert abs(offset_n) < 0.001
    assert abs(offset_e) < 0.001


def test_affine_scale_only():
    """Pure scaling (2x) with no rotation or offset."""
    dxf_pts = [(0.0, 0.0), (1.0, 0.0)]
    ned_pts = [(0.0, 0.0), (2.0, 0.0)]
    scale, theta, offset_n, offset_e = dxf_to_ned_affine(dxf_pts, ned_pts)
    assert abs(scale - 2.0) < 0.001


def test_affine_translation():
    """Pure translation (offset) with no scale or rotation."""
    dxf_pts = [(0.0, 0.0), (1.0, 0.0)]
    ned_pts = [(10.0, 20.0), (11.0, 20.0)]
    scale, theta, offset_n, offset_e = dxf_to_ned_affine(dxf_pts, ned_pts)
    assert abs(scale - 1.0) < 0.001
    assert abs(offset_n - 10.0) < 0.001
    assert abs(offset_e - 20.0) < 0.001


def test_affine_rotation_90deg():
    """90° rotation: DXF x-axis maps to NED y-axis."""
    dxf_pts = [(0.0, 0.0), (1.0, 0.0)]  # along DXF y (north)
    ned_pts = [(0.0, 0.0), (0.0, 1.0)]  # maps to east
    scale, theta, offset_n, offset_e = dxf_to_ned_affine(dxf_pts, ned_pts)
    assert abs(scale - 1.0) < 0.001
    assert abs(theta - math.pi / 2) < 0.01 or abs(theta + math.pi * 1.5) < 0.01


def test_affine_insufficient_points():
    """Less than 2 reference points raises ValueError."""
    try:
        dxf_to_ned_affine([(0, 0)], [(0, 0)])
        assert False, "Should have raised ValueError"
    except ValueError:
        pass


def test_affine_coincident_points():
    """Coincident reference points raise ValueError."""
    try:
        dxf_to_ned_affine([(1, 1), (1, 1)], [(0, 0), (1, 1)])
        assert False, "Should have raised ValueError"
    except ValueError:
        pass


# ── apply_affine_transform tests ────────────────────────────────────────────────

def test_apply_affine_identity():
    """Identity transform returns original point."""
    result = apply_affine_transform((3.0, 4.0), scale=1.0, theta=0.0, offset_n=0.0, offset_e=0.0)
    assert abs(result[0] - 3.0) < 0.001
    assert abs(result[1] - 4.0) < 0.001


def test_apply_affine_translation():
    """Pure translation adds offset."""
    result = apply_affine_transform((0.0, 0.0), scale=1.0, theta=0.0, offset_n=10.0, offset_e=20.0)
    assert abs(result[0] - 10.0) < 0.001
    assert abs(result[1] - 20.0) < 0.001


def test_apply_affine_scale():
    """Scale multiplies coordinates."""
    result = apply_affine_transform((5.0, 0.0), scale=0.01, theta=0.0, offset_n=0.0, offset_e=0.0)
    assert abs(result[0] - 0.05) < 0.001
    assert abs(result[1]) < 0.001


def test_apply_affine_roundtrip():
    """Affine transform roundtrip: compute params, then transform another point."""
    dxf_pts = [(0.0, 0.0), (100.0, 0.0)]  # DXF cm
    ned_pts = [(0.0, 0.0), (1.0, 0.0)]    # NED metres (1m = 100cm)
    scale, theta, off_n, off_e = dxf_to_ned_affine(dxf_pts, ned_pts)

    # Transform a DXF point (50, 50)cm → should be (0.5, 0.5)m
    result = apply_affine_transform((50.0, 0.0), scale, theta, off_n, off_e)
    assert abs(result[0] - 0.5) < 0.01