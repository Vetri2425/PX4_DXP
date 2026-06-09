"""Unit tests for PathValidator."""

import math
import pytest
from path_engine.core import PlannedPath
from path_engine.validator import PathValidationError, PathValidator


def test_validator_empty_path():
    validator = PathValidator()
    plan = PlannedPath()
    warnings = validator.validate(plan)
    assert len(warnings) == 1
    assert "no waypoints" in warnings[0]


def test_validator_safe_path():
    validator = PathValidator()
    # Straight line, 10m long
    pts = [(i * 0.5, 0.0) for i in range(21)]
    plan = PlannedPath(merged_waypoints=pts)
    warnings = validator.validate(plan)
    assert len(warnings) == 0


def test_validator_bbox_warning():
    validator = PathValidator(max_bbox_size_m=500.0)
    pts = [(0.0, 0.0), (1000.0, 0.0)]
    plan = PlannedPath(merged_waypoints=pts)
    warnings = validator.validate(plan)
    assert len(warnings) > 0
    assert "bounding box" in warnings[0]


def test_validator_gap_warning():
    validator = PathValidator(max_gap_m=1.0)
    pts = [(0.0, 0.0), (0.5, 0.0), (10.0, 0.0)]  # 9.5m gap
    plan = PlannedPath(merged_waypoints=pts)
    warnings = validator.validate(plan)
    assert len(warnings) > 0
    assert "gap" in warnings[0]


def test_validator_turn_radius_warning():
    validator = PathValidator(min_turn_radius_m=1.0)
    # 90-degree corner at (1.0, 0.0) with intermediate spacing:
    # (0, 0) -> (1.0, 0.0) -> (1.0, 1.0)
    pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    plan = PlannedPath(merged_waypoints=pts)
    warnings = validator.validate(plan)
    assert len(warnings) > 0
    assert "radius" in warnings[0]


def test_validator_self_intersection():
    validator = PathValidator()
    # Figure 8 or self crossing line
    # (0,0) -> (2,2) -> (2,0) -> (0,2)
    pts = [(0.0, 0.0), (2.0, 2.0), (2.0, 0.0), (0.0, 2.0)]
    plan = PlannedPath(merged_waypoints=pts)
    warnings = validator.validate(plan)
    assert len(warnings) > 0
    assert any("intersects" in w for w in warnings)


def test_validator_hard_fails_waypoint_explosion():
    validator = PathValidator(max_waypoints=10)
    plan = PlannedPath(merged_waypoints=[(i * 0.1, 0.0) for i in range(11)])

    warnings, errors = validator.validate_detailed(plan)
    assert warnings == []
    assert len(errors) == 1
    assert "Too many waypoints" in errors[0]

    with pytest.raises(PathValidationError):
        validator.validate_or_raise(plan)


def test_validator_warns_near_waypoint_limit():
    validator = PathValidator(max_waypoints=10)
    plan = PlannedPath(merged_waypoints=[(float(i), 0.0) for i in range(9)])

    warnings = validator.validate_or_raise(plan)
    assert any("High waypoint count" in w for w in warnings)
