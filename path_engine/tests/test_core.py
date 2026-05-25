"""Tests for path_engine core data models."""

import math
from path_engine.core import SegmentType, PathSegment, DXFEntity, PlannedPath


def test_segment_type_values():
    assert SegmentType.MARK == 0
    assert SegmentType.TRANSIT == 1
    assert SegmentType.MARK.value == 0
    assert SegmentType.TRANSIT.value == 1


def test_path_segment_length():
    # Straight line: (0,0) → (3,0) = 3.0m
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
        speed=0.35,
    )
    assert abs(seg.length - 3.0) < 0.001


def test_path_segment_length_l_shape():
    # L-shape: (0,0) → (2,0) → (2,2) = 4.0m
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)],
        speed=0.35,
    )
    assert abs(seg.length - 4.0) < 0.001


def test_path_segment_length_single_point():
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(1.0, 2.0)],
        speed=0.35,
    )
    assert seg.length == 0.0


def test_path_segment_length_empty():
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[],
        speed=0.35,
    )
    assert seg.length == 0.0


def test_dxf_entity_is_mark_default():
    ent = DXFEntity(entity_type="LINE", layer="DRAWING")
    assert ent.is_mark() is True


def test_dxf_entity_is_mark_transit_layer():
    ent = DXFEntity(entity_type="LINE", layer="TRANSIT")
    assert ent.is_mark() is False


def test_dxf_entity_is_mark_travel_layer():
    ent = DXFEntity(entity_type="LINE", layer="TRAVEL_LINES")
    assert ent.is_mark() is False


def test_dxf_entity_is_mark_custom_mapping():
    ent = DXFEntity(entity_type="LINE", layer="OUTLINE")
    # Default: no match in mapping → still MARK
    assert ent.is_mark(layer_mapping={"TRANSIT": "transit"}) is True
    # With mapping that matches:
    assert ent.is_mark(layer_mapping={"OUTLINE": "transit"}) is False
    # Ignore:
    assert ent.is_mark(layer_mapping={"OUTLINE": "ignore"}) is True  # "ignore" treated as non-transit


def test_planned_path_defaults():
    plan = PlannedPath()
    assert plan.num_waypoints == 0
    assert plan.total_length == 0.0
    assert plan.segments == []
    assert plan.merged_waypoints == []
    assert plan.spray_flags == []


def test_planned_path_with_segments():
    seg1 = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0, 0), (1, 0), (2, 0)],
        speed=0.35,
    )
    seg2 = PathSegment(
        segment_type=SegmentType.TRANSIT,
        points=[(2, 0), (2, 1)],
        speed=0.50,
    )
    plan = PlannedPath(
        segments=[seg1, seg2],
        merged_waypoints=[(0, 0), (1, 0), (2, 0), (2, 1)],
        spray_flags=[True, True, True, False],
        total_mark_length=2.0,
        total_transit_length=1.0,
    )
    assert plan.num_waypoints == 4
    assert abs(plan.total_length - 3.0) < 0.001
    assert plan.total_mark_length == 2.0
    assert plan.total_transit_length == 1.0