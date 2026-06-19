"""Production geometry contract: planner preserves CAD MARK endpoints.

Planner owns geometry (exact PRE/MARK/AFT lengths).
Runtime spray_controller owns latency anticipation.
"""

from __future__ import annotations

import math
import os

import pytest

from path_engine.core import PathSegment, SegmentType
from path_engine.engine import PathEngine
from path_engine.parsers.dxf_parser import parse_dxf, entities_to_segments

_TOL = 1e-6
_SQUARE_DXF = os.path.join(
    os.path.dirname(__file__), "..", "..", "Simple Demo", "square_2x2.dxf"
)


def _hypot_seg(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _polyline_length(pts: list[tuple[float, float]]) -> float:
    return sum(_hypot_seg(pts[i - 1], pts[i]) for i in range(1, len(pts)))


def _endpoint_distance(pts: list[tuple[float, float]]) -> float:
    return _hypot_seg(pts[0], pts[-1])


def _production_engine(**overrides) -> PathEngine:
    """Settings matching production plan/stage/load (geometry exact)."""
    params = dict(
        mark_spacing=0.05,
        compensate_spray=False,
        enable_path_extensions=True,
        pre_extension_m=0.5,
        aft_extension_m=0.5,
        per_line_extensions=True,
        optimize_order=False,
    )
    params.update(overrides)
    return PathEngine(**params)


def _mark_segments(plan) -> list[PathSegment]:
    return [
        s for s in plan.segments
        if s.segment_type == SegmentType.MARK
        and s.metadata.get("extension_role") not in ("pre", "aft")
    ]


def _extension_segments(plan, role: str) -> list[PathSegment]:
    return [
        s for s in plan.segments
        if s.metadata.get("extension_role") == role
    ]


# ── Test 1: exact MARK geometry ──────────────────────────────────────────────

def test_production_mark_exact_2m_line():
    engine = _production_engine()
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (2.0, 0.0)],
        speed=0.35,
        source_entity="LINE_T1",
        metadata={"geometry_type": "LINE", "line_like": True},
    )
    plan = engine.plan_segments([seg])
    marks = _mark_segments(plan)
    assert len(marks) == 1
    pts = marks[0].points
    assert abs(_endpoint_distance(pts) - 2.0) < _TOL
    assert abs(_polyline_length(pts) - 2.0) < _TOL


# ── Test 2: exact PRE/MARK/AFT extensions ────────────────────────────────────

def test_production_pre_mark_aft_lengths():
    engine = _production_engine()
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (2.0, 0.0)],
        speed=0.35,
        source_entity="LINE_T2",
        metadata={"geometry_type": "LINE", "line_like": True},
    )
    plan = engine.plan_segments([seg])
    pres = _extension_segments(plan, "pre")
    marks = _mark_segments(plan)
    afts = _extension_segments(plan, "aft")
    assert len(pres) == 1 and len(marks) == 1 and len(afts) == 1
    assert abs(_polyline_length(pres[0].points) - 0.5) < _TOL
    assert abs(_endpoint_distance(marks[0].points) - 2.0) < _TOL
    assert abs(_polyline_length(marks[0].points) - 2.0) < _TOL
    assert abs(_polyline_length(afts[0].points) - 0.5) < _TOL


# ── Test 3: four-side square ─────────────────────────────────────────────────

@pytest.mark.skipif(not os.path.isfile(_SQUARE_DXF), reason="square_2x2.dxf missing")
def test_production_square_four_mark_sides_exact_2m():
    entities = parse_dxf(_SQUARE_DXF)
    segments = entities_to_segments(entities)
    engine = _production_engine()
    plan = engine.plan_segments(segments)
    marks = _mark_segments(plan)
    assert len(marks) == 4
    for mark in marks:
        pts = mark.points
        assert abs(_endpoint_distance(pts) - 2.0) < _TOL, mark.source_entity
        assert abs(_polyline_length(pts) - 2.0) < _TOL, mark.source_entity


# ── Test 4: legacy geometric compensation still works ──────────────────────────

def test_legacy_compensate_spray_true_produces_2_0315m():
    engine = PathEngine(
        compensate_spray=True,
        enable_path_extensions=False,
        optimize_order=False,
        mark_spacing=0.05,
    )
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (2.0, 0.0)],
        speed=0.35,
    )
    plan = engine.plan_segments([seg])
    marks = _mark_segments(plan)
    pts = marks[0].points
    expected = 2.0 + 0.10 * 0.35 - 0.01 * 0.35  # 2.0315
    assert abs(_endpoint_distance(pts) - expected) < 1e-4
    assert abs(_polyline_length(pts) - expected) < 1e-4


# ── Test 5: spray-transition preservation ──────────────────────────────────────

def test_production_spray_boundary_points_preserved():
    engine = _production_engine()
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (2.0, 0.0)],
        speed=0.35,
        source_entity="LINE_T5",
        metadata={"geometry_type": "LINE", "line_like": True},
    )
    plan = engine.plan_segments([seg])
    boundary_pairs = [
        (a, fa, b, fb)
        for (a, fa), (b, fb) in zip(
            zip(plan.merged_waypoints, plan.spray_flags),
            zip(plan.merged_waypoints[1:], plan.spray_flags[1:]),
        )
        if _hypot_seg(a, b) < 1e-9 and fa != fb
    ]
    assert len(boundary_pairs) == 2


# ── Test 6: production defaults do not leak compensation ─────────────────────

def _repo_root() -> str:
    return os.path.join(os.path.dirname(__file__), "..", "..")


def test_path_plan_request_default_compensate_spray_false():
    models_path = os.path.join(_repo_root(), "server", "models.py")
    with open(models_path) as fh:
        source = fh.read()
    assert "compensate_spray: bool = False" in source


def test_path_manager_plan_path_default_compensate_spray_false():
    pm_path = os.path.join(_repo_root(), "server", "path_manager.py")
    with open(pm_path) as fh:
        source = fh.read()
    assert 'kwargs.pop("compensate_spray", False)' in source
    assert "compensate_spray=False" in source


# ── Test 7: controller distance-aware compensation remains enabled ───────────

def test_spray_controller_distance_aware_default_enabled():
    sc_path = os.path.join(_repo_root(), "src", "spray_controller_node.py")
    with open(sc_path) as fh:
        source = fh.read()
    assert 'declare_parameter("use_distance_aware_spray", True)' in source