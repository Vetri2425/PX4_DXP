"""Tests for path_engine NED coordinate transforms."""

import math

from path_engine.ned import (
    latlon_to_ned,
    dxf_to_ned_affine,
    apply_affine_transform,
    estimate_fit_scale,
    _HAS_GEOGRAPHICLIB,
)


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
    scale, theta, offset_n, offset_e, res, rmse = dxf_to_ned_affine(dxf_pts, ned_pts)
    assert abs(scale - 1.0) < 0.001
    assert abs(theta) < 0.001
    assert abs(offset_n) < 0.001
    assert abs(offset_e) < 0.001
    assert all(r < 0.001 for r in res)
    assert rmse < 0.001


def test_affine_never_scales_geometry():
    """Ref points implying 2x MUST NOT scale the geometry — the DXF owns the size.

    Previously this asserted the opposite (scale==2.0). That was the defect: a
    free-scale fit absorbs any survey disagreement into `scale` and silently
    resizes the drawing, so a 2 m square could be painted at 1.5-2.5 m and still
    pass every gate. The fit is now rigid.
    """
    dxf_pts = [(0.0, 0.0), (1.0, 0.0)]
    ned_pts = [(0.0, 0.0), (2.0, 0.0)]  # survey says these points are 2 m apart
    scale, theta, offset_n, offset_e, res, rmse = dxf_to_ned_affine(dxf_pts, ned_pts)

    assert scale == 1.0, "applied scale must be pinned to unity"
    # The disagreement is now REPORTED rather than absorbed: a rigid 2-point fit has
    # one residual DOF and it is exactly the baseline-length mismatch. Under the old
    # free-scale fit this rmse was ~0, which is why the RMSE gate never caught it.
    assert rmse > 0.4, f"baseline mismatch must surface as rmse, got {rmse}"

    # ...and the free-scale value is still available as a diagnostic.
    assert abs(estimate_fit_scale(dxf_pts, ned_pts) - 2.0) < 0.001


def test_affine_scale_only_legacy_optin():
    """lock_scale=False restores the old free-scale fit (diagnostics/tests only)."""
    dxf_pts = [(0.0, 0.0), (1.0, 0.0)]
    ned_pts = [(0.0, 0.0), (2.0, 0.0)]
    scale, _, _, _, res, rmse = dxf_to_ned_affine(dxf_pts, ned_pts, lock_scale=False)
    assert abs(scale - 2.0) < 0.001
    assert all(r < 0.001 for r in res)
    assert rmse < 0.001  # exactly this blindness is why lock_scale defaults to True


def test_affine_translation():
    """Pure translation (offset) with no scale or rotation."""
    dxf_pts = [(0.0, 0.0), (1.0, 0.0)]
    ned_pts = [(10.0, 20.0), (11.0, 20.0)]
    scale, theta, offset_n, offset_e, res, rmse = dxf_to_ned_affine(dxf_pts, ned_pts)
    assert abs(scale - 1.0) < 0.001
    assert abs(offset_n - 10.0) < 0.001
    assert abs(offset_e - 20.0) < 0.001
    assert all(r < 0.001 for r in res)
    assert rmse < 0.001


def test_affine_rotation_90deg():
    """90° rotation: DXF x-axis maps to NED y-axis."""
    dxf_pts = [(0.0, 0.0), (1.0, 0.0)]  # along DXF y (north)
    ned_pts = [(0.0, 0.0), (0.0, 1.0)]  # maps to east
    scale, theta, offset_n, offset_e, res, rmse = dxf_to_ned_affine(dxf_pts, ned_pts)
    assert abs(scale - 1.0) < 0.001
    assert abs(theta - math.pi / 2) < 0.01 or abs(theta + math.pi * 1.5) < 0.01
    assert all(r < 0.001 for r in res)
    assert rmse < 0.001


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


def test_affine_multipoint_least_squares():
    """Least-squares alignment over N=4 noisy points — rigid (translation only here).

    The transform under test is scale=1, rotation=0, translation=(5, 5), with ~1 cm
    of noise on each target. This is the realistic surveyed case: the operator's ref
    points carry noise, and the fit must average it into rotation/translation without
    ever resizing the drawing.
    """
    dxf_pts = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    ref_pts = [
        (5.01, 5.0),    # target (5, 5)   + (0.01, 0)
        (15.0, 4.99),   # target (15, 5)  + (0, -0.01)
        (14.99, 15.0),  # target (15, 15) + (-0.01, 0)
        (5.0, 15.01),   # target (5, 15)  + (0, 0.01)
    ]
    scale, theta, off_n, off_e, res, rmse = dxf_to_ned_affine(dxf_pts, ref_pts)
    assert scale == 1.0
    assert abs(theta) < 0.01
    assert abs(off_n - 5.0) < 0.05
    assert abs(off_e - 5.0) < 0.05
    assert all(r < 0.02 for r in res)
    assert rmse < 0.02


def test_affine_multipoint_does_not_absorb_scale_error():
    """N=4 ref points implying a uniformly 2x-larger figure must NOT resize the DXF.

    Over-determined fits are not immune: least-squares happily reports scale=2.0 for
    a consistently-scaled point set. Locking scale means the geometry survives and the
    disagreement lands in RMSE, where the RMSE gate can reject it.
    """
    dxf_pts = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    ref_pts = [(5.0, 5.0), (25.0, 5.0), (25.0, 25.0), (5.0, 25.0)]  # 2x the DXF

    scale, _, _, _, _, rmse = dxf_to_ned_affine(dxf_pts, ref_pts)
    assert scale == 1.0
    assert rmse > 1.0, "a 2x survey/drawing disagreement must be loudly reported"

    assert abs(estimate_fit_scale(dxf_pts, ref_pts) - 2.0) < 0.01


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
    scale, theta, off_n, off_e, res, rmse = dxf_to_ned_affine(dxf_pts, ned_pts)

    # Transform a DXF point (50, 50)cm → should be (0.5, 0.5)m
    result = apply_affine_transform((50.0, 0.0), scale, theta, off_n, off_e)
    assert abs(result[0] - 0.5) < 0.01