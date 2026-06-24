#!/usr/bin/env python3
"""Unit tests for point-leg trajectory helpers."""

from __future__ import annotations

import math

from point_leg_trajectory import (
    PointLegTrajectoryMode,
    build_point_leg_path,
    densify_point_leg,
    interior_spacing_stats,
    is_collinear_straight_leg,
    predict_rpp_conditioning,
    simplify_collinear_path,
)


def test_two_point_mode_preserves_endpoints():
    start = (0.0, 0.0)
    end = (5.0, 0.0)
    pts = build_point_leg_path(start, end, mode=PointLegTrajectoryMode.TWO_POINT)
    assert pts == [start, end]
    profile, conditioned = predict_rpp_conditioning(pts, runtime_entry=True)
    assert profile == "segment"
    assert conditioned == [start, end]


def test_densified_spacing_on_long_leg():
    start = (0.0, 0.0)
    end = (5.0, 0.0)
    spacing = 0.08
    pts = build_point_leg_path(
        start, end, mode=PointLegTrajectoryMode.DENSIFIED, spacing_m=spacing
    )
    assert pts[0] == start
    assert pts[-1] == end
    assert len(pts) > 2
    lo, hi = interior_spacing_stats(pts)
    assert lo >= spacing - 0.01
    assert hi <= spacing + 0.01


def test_short_leg_stays_two_points():
    start = (0.0, 0.0)
    end = (0.04, 0.0)
    pts = densify_point_leg(start, end, 0.08)
    assert len(pts) == 2
    assert pts[0] == start
    assert pts[-1] == end


def test_collinear_densified_predicts_smooth_resample():
    start = (0.0, 0.0)
    end = (4.0, 0.0)
    published = build_point_leg_path(
        start, end, mode=PointLegTrajectoryMode.DENSIFIED, spacing_m=0.08
    )
    profile, conditioned = predict_rpp_conditioning(
        published, runtime_entry=True, resample_spacing_m=0.08
    )
    assert profile == "smooth"
    assert conditioned[0] == start
    assert conditioned[-1] == end
    assert len(conditioned) >= len(published) - 2
    lo, hi = interior_spacing_stats(conditioned)
    assert lo >= 0.07
    assert hi <= 0.09


def test_segment_simplify_collapses_collinear_intermediates():
    start = (0.0, 0.0)
    end = (3.0, 0.0)
    dense = build_point_leg_path(
        start, end, mode=PointLegTrajectoryMode.DENSIFIED, spacing_m=0.5
    )
    simplified = simplify_collinear_path(dense)
    assert len(simplified) == 2
    assert simplified[0] == start
    assert simplified[-1] == end


def test_non_collinear_not_straight_leg():
    pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert is_collinear_straight_leg(pts) is False


def test_invalid_mode_rejected():
    try:
        PointLegTrajectoryMode.parse("arc")
        assert False, "expected ValueError"
    except ValueError as exc:
        assert "two_point" in str(exc)


def main():
    test_two_point_mode_preserves_endpoints()
    test_densified_spacing_on_long_leg()
    test_short_leg_stays_two_points()
    test_collinear_densified_predicts_smooth_resample()
    test_segment_simplify_collapses_collinear_intermediates()
    test_non_collinear_not_straight_leg()
    test_invalid_mode_rejected()
    print("PASS")


if __name__ == "__main__":
    main()