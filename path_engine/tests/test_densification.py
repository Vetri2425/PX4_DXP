"""Regression tests for waypoint densification of extension geometry.

Guards the production invariant after the densification fix:

    All non-zero waypoint intervals in PRE, MARK, AFT, and extension connectors
    must be <= 0.05 m, while geometry, ordering, spray states, and total path
    length remain exact.

Root causes these tests pin down:
  * PRE/AFT were densified twice (Step 4 + Step 5b) — float noise re-split clean
    5 cm intervals into 2.5 cm ones. Fixed: single owner (Step 5b) + float guard.
  * MARK edges arrived from _merge_chain with ~0.10 m gaps (corner dedup tol ==
    mark_spacing) and were never re-densified. Fixed: re-densify edges in Step 4.
  * Connectors used transit_spacing (0.15) -> 0.1414 m intervals. Fixed: the
    extension_connector flag now routes them to mark_spacing.
"""

from __future__ import annotations

import math
import os

from path_engine.core import PathSegment, SegmentType
from path_engine.engine import PathEngine
from path_engine.planners.straight_line import densify_line
from path_engine.parsers.dxf_parser import parse_dxf, entities_to_segments

_SQUARE_DXF = os.path.join(
    os.path.dirname(__file__), "..", "..", "Simple Demo", "square_2x2.dxf"
)
_MAX = 0.050001  # 5 cm + float-guard epsilon


def _ivs(pts):
    return [
        math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        for i in range(1, len(pts))
    ]


def _nz(pts):
    return [v for v in _ivs(pts) if v > 1e-9]


def _polylen(pts):
    return sum(_ivs(pts))


def _production_plan():
    ents = parse_dxf(_SQUARE_DXF)
    segs = entities_to_segments(ents)
    eng = PathEngine(
        mark_spacing=0.05,
        transit_spacing=0.15,
        compensate_spray=False,
        enable_path_extensions=True,
        pre_extension_m=0.5,
        aft_extension_m=0.5,
        per_line_extensions=True,
        optimize_order=False,
    )
    return eng.plan_segments(segs)


def _by_role(plan):
    pre, mark, aft, conn = [], [], [], []
    for s in plan.segments:
        role = s.metadata.get("extension_role")
        if role == "pre":
            pre.append(s)
        elif role == "aft":
            aft.append(s)
        elif s.metadata.get("extension_connector"):
            conn.append(s)
        elif s.segment_type == SegmentType.MARK:
            mark.append(s)
    return pre, mark, aft, conn


# ── Test 1 — PRE exact single-pass density ───────────────────────────────────

def test_pre_exact_single_pass_density():
    pre, *_ = _by_role(_production_plan())
    assert len(pre) == 4
    for s in pre:
        assert abs(_polylen(s.points) - 0.5) < 1e-6
        assert len(s.points) == 11
        assert len(s.points) - 1 == 10
        for v in _nz(s.points):
            assert abs(v - 0.05) < 1e-6


# ── Test 2 — AFT exact single-pass density ───────────────────────────────────

def test_aft_exact_single_pass_density():
    _, _, aft, _ = _by_role(_production_plan())
    assert len(aft) == 4
    for s in aft:
        assert abs(_polylen(s.points) - 0.5) < 1e-6
        assert len(s.points) == 11
        for v in _nz(s.points):
            assert abs(v - 0.05) < 1e-6


# ── Test 3 — MARK exact density (no 10 cm gaps) ──────────────────────────────

def test_mark_exact_density_no_gaps():
    _, mark, _, _ = _by_role(_production_plan())
    assert len(mark) == 4
    for s in mark:
        assert abs(_polylen(s.points) - 2.0) < 1e-6
        assert len(s.points) == 41
        ivs = _nz(s.points)
        assert max(ivs) <= _MAX
        # no interval anywhere near the old 0.10 m gap
        assert max(ivs) < 0.06


# ── Test 4 — diagonal connector maximum spacing ──────────────────────────────

def test_connector_max_spacing():
    _, _, _, conn = _by_role(_production_plan())
    assert len(conn) == 3
    diag = math.hypot(0.5, 0.5)
    for s in conn:
        assert len(s.points) == 16
        assert len(s.points) - 1 == 15
        assert max(_nz(s.points)) <= _MAX
        # polyline length equals straight endpoint distance (still a straight seg)
        assert abs(_polylen(s.points) - diag) < 1e-6
        assert abs(math.hypot(
            s.points[-1][0] - s.points[0][0],
            s.points[-1][1] - s.points[0][1],
        ) - diag) < 1e-6


# ── Test 5 — geometry preservation / total length ────────────────────────────

def test_geometry_and_total_length_preserved():
    plan = _production_plan()
    pre, mark, aft, conn = _by_role(plan)
    assert abs(sum(_polylen(s.points) for s in pre) - 2.0) < 1e-6
    assert abs(sum(_polylen(s.points) for s in mark) - 8.0) < 1e-6
    assert abs(sum(_polylen(s.points) for s in aft) - 2.0) < 1e-6
    diag = math.hypot(0.5, 0.5)
    for s in conn:
        assert abs(_polylen(s.points) - diag) < 1e-6

    structured = sum(_polylen(s.points) for s in plan.segments)
    expected = 4 * 3.0 + 3 * diag
    assert abs(structured - expected) < 1e-6

    flattened = _polylen(plan.merged_waypoints)
    assert abs(flattened - structured) < 1e-6


# ── Test 6 — spray-boundary zero-distance transitions survive ────────────────

def test_spray_boundary_duplicates_survive():
    plan = _production_plan()
    wp = plan.merged_waypoints
    flags = plan.spray_flags
    transitions = 0
    for i in range(1, len(wp)):
        d = math.hypot(wp[i][0] - wp[i - 1][0], wp[i][1] - wp[i - 1][1])
        if d < 1e-6 and flags[i] != flags[i - 1]:
            transitions += 1
    # 4 sides x (PRE end/MARK start OFF->ON, MARK end/AFT start ON->OFF)
    assert transitions == 8


# ── Test 7 — generic (non-extension) transit keeps transit_spacing ───────────

def test_generic_transit_unaffected():
    eng = PathEngine(
        mark_spacing=0.05,
        transit_spacing=0.15,
        compensate_spray=False,
        enable_path_extensions=False,
        optimize_order=False,
    )
    # Two disjoint MARK lines: the flattening inserts an implicit transit move,
    # but the explicit TRANSIT entity here exercises generic densification.
    plan = eng.plan_segments([
        PathSegment(
            segment_type=SegmentType.TRANSIT,
            points=[(0.0, 0.0), (0.0, 1.0)],
            speed=0.5,
            source_entity="TRANSIT_GENERIC",
            metadata={},
        ),
    ])
    transit = [s for s in plan.segments if s.segment_type == SegmentType.TRANSIT][0]
    ivs = _nz(transit.points)
    # spaced at transit_spacing (0.15), NOT mark_spacing (0.05)
    assert max(ivs) > 0.05 + 1e-3
    assert max(ivs) <= 0.15 + 1e-6


# ── Test 8 — float-guard subdivision behaviour ───────────────────────────────

def test_float_guard_no_spurious_split():
    # length == spacing with float noise must NOT split into two half-intervals
    pts = densify_line((0.0, 0.0), (0.05000000000000004, 0.0), 0.05)
    assert len(pts) == 2
    # a genuinely longer segment still subdivides
    pts2 = densify_line((0.0, 0.0), (0.1, 0.0), 0.05)
    assert len(pts2) == 3
    assert max(_nz(pts2)) <= _MAX
    # exact-divisible 0.5 m -> 11 points, all 0.05
    pts3 = densify_line((0.0, 0.0), (0.5, 0.0), 0.05)
    assert len(pts3) == 11
    for v in _nz(pts3):
        assert abs(v - 0.05) < 1e-6


# ── Test 9 — spray-compensation ownership unchanged ──────────────────────────

def test_production_compensation_disabled_marks_exact():
    plan = _production_plan()
    _, mark, _, _ = _by_role(plan)
    for s in mark:
        assert abs(_polylen(s.points) - 2.0) < 1e-6
