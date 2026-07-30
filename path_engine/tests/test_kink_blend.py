"""kink_blend unit tests — the G1 biarc blend for arc-fit joint kinks.

The reference defect: curve_6_points-1 (2026-07-30) carried −9.98°/+4.04°/−9.99°
single-vertex tangent jumps at surveyed stakes. These tests pin the pass's
contract: qualifying kinks get spread into G1 arcs, the line never moves more
than the cap, out-of-band turns and straights are untouched, and control
(must-hit) provenance survives within the cap.
"""

import math

import pytest

from path_engine.planners.kink_blend import blend_kinks


def _turns_deg(pts):
    out = []
    for i in range(1, len(pts) - 1):
        v1 = (pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        v2 = (pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        if math.hypot(*v1) < 1e-12 or math.hypot(*v2) < 1e-12:
            out.append(0.0)
            continue
        out.append(math.degrees(math.atan2(v1[0] * v2[1] - v1[1] * v2[0],
                                           v1[0] * v2[0] + v1[1] * v2[1])))
    return out


def _dist_to_polyline(p, poly):
    best = float("inf")
    for j in range(len(poly) - 1):
        a, b = poly[j], poly[j + 1]
        dn, de = b[0] - a[0], b[1] - a[1]
        s2 = dn * dn + de * de
        if s2 < 1e-18:
            continue
        t = max(0.0, min(1.0, ((p[0] - a[0]) * dn + (p[1] - a[1]) * de) / s2))
        best = min(best, math.hypot(p[0] - (a[0] + t * dn), p[1] - (a[1] + t * de)))
    return best


def _polyline(headings_deg, step=0.15):
    """Build a polyline from per-step headings (degrees)."""
    pts = [(0.0, 0.0)]
    for h in headings_deg:
        r = math.radians(h)
        pts.append((pts[-1][0] + step * math.cos(r), pts[-1][1] + step * math.sin(r)))
    return pts


def _kinked_line(kink_deg, leg_pts=8, step=0.15):
    """Two straight legs meeting at one vertex with `kink_deg` of turn."""
    return _polyline([0.0] * leg_pts + [kink_deg] * leg_pts, step)


def test_straight_line_untouched():
    pts = _polyline([0.0] * 20)
    out, ctrl, report = blend_kinks(pts, None, max_dev_m=0.01)
    assert out == pts
    assert report == []
    assert ctrl == list(range(len(pts)))


def test_ten_degree_kink_blended_within_cap():
    pts = _kinked_line(10.0)
    apex = pts[8]
    out, ctrl, report = blend_kinks(pts, None, max_dev_m=0.01)
    assert len(report) == 1 and report[0]["blended"]
    # tangent discontinuity is spread: no emitted vertex turns like the kink did
    assert max(abs(t) for t in _turns_deg(out)) < 4.0
    # the surveyed apex stays within the cap of the new line
    assert _dist_to_polyline(apex, out) <= 0.0105
    # endpoints are untouched
    assert out[0] == pts[0] and out[-1] == pts[-1]


def test_small_turn_below_floor_untouched():
    pts = _kinked_line(2.5)
    out, ctrl, report = blend_kinks(pts, None, max_dev_m=0.01)
    assert out == pts
    assert report == []


def test_real_corner_above_ceiling_untouched():
    pts = _kinked_line(90.0)
    out, ctrl, report = blend_kinks(pts, None, max_dev_m=0.01)
    assert out == pts
    assert report == []


def test_control_provenance_survives_within_cap():
    pts = _kinked_line(10.0)
    apex_idx = 8
    ctrl_in = [0, apex_idx, len(pts) - 1]
    out, ctrl, report = blend_kinks(pts, ctrl_in, max_dev_m=0.01)
    assert report[0]["blended"]
    assert 0 in ctrl and (len(out) - 1) in ctrl
    # some control point sits within the cap of the original apex
    apex = pts[apex_idx]
    assert min(math.dist(out[c], apex) for c in ctrl) <= 0.0105
    assert ctrl == sorted(set(ctrl))
    assert all(0 <= c < len(out) for c in ctrl)


def test_arc_to_arc_joint_like_the_field_case():
    """Two R=3 m arcs joined with a 10° tangent jump — corner_fillet refuses
    this shape (no straight baseline); kink_blend must handle it."""
    pts = [(3.0 * math.sin(a), 3.0 * (1 - math.cos(a)))
           for a in [i * 0.05 for i in range(13)]]  # arc 1, ~1.8 m
    # tangent at the end of arc 1, then jump it by +10° and continue a new arc
    end_a = 12 * 0.05
    heading = end_a + math.radians(10.0)
    cx, cy = pts[-1]
    for i in range(1, 13):
        a = heading + i * 0.05
        # incremental chords of a second R=3 arc starting at the jumped heading
        cx += 0.15 * math.cos(a)
        cy += 0.15 * math.sin(a)
        pts.append((cx, cy))
    turns_before = _turns_deg(pts)
    assert max(abs(t) for t in turns_before) > 9.0  # the joint is a real kink
    out, ctrl, report = blend_kinks(pts, None, max_dev_m=0.01)
    blended = [r for r in report if r["blended"]]
    assert blended, f"joint not blended: {report}"
    assert max(abs(t) for t in _turns_deg(out)) < 5.0
    # every replaced original point stays within the cap
    assert all(_dist_to_polyline(p, out) <= 0.0105 for p in pts)


def test_uniform_arc_discretization_untouched():
    """A coarsely sampled smooth arc turns 4.5° at EVERY vertex — over any
    absolute threshold, but with zero excess over its neighbours. This is the
    plan-trajectory contract case: dense fitted arcs must pass through
    byte-identical (regression for test_fitted_arc_is_not_re_fitted...)."""
    r = 5.0
    arc = [(r * math.cos(t * math.pi / 40), r * math.sin(t * math.pi / 40))
           for t in range(21)]
    out, ctrl, report = blend_kinks(arc, None, max_dev_m=0.01)
    assert out == arc
    assert not any(e.get("blended") for e in report)


def test_reverse_jog_amid_uniform_turns_blended():
    """The stake-10 defect: a +4° vertex amid uniform −2.7° turns — small in
    absolute terms, a ~6.7° tangent discontinuity in context."""
    heads = []
    h = 0.0
    for i in range(24):
        h += 4.0 if i == 12 else -2.7
        heads.append(h)
    pts = _polyline(heads)
    out, ctrl, report = blend_kinks(pts, None, max_dev_m=0.01)
    blended = [e for e in report if e["blended"]]
    assert blended, f"reverse jog not blended: {report}"
    assert all(_dist_to_polyline(p, out) <= 0.0105 for p in pts)


def test_zero_cap_is_a_noop():
    pts = _kinked_line(10.0)
    out, ctrl, report = blend_kinks(pts, None, max_dev_m=0.0)
    assert out == pts and report == []


def test_engine_default_off_and_validation():
    from path_engine.engine import PathEngine
    with pytest.raises(ValueError):
        PathEngine(blend_kinks_max_dev_m=-0.1)
    assert PathEngine().blend_kinks_max_dev_m == 0.0
