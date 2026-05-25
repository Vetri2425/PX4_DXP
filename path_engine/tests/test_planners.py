"""Tests for path_engine planners (straight_line + arc_curve)."""

import math

from path_engine.core import PathSegment, SegmentType
from path_engine.planners.straight_line import densify_line, densify_segment
from path_engine.planners.arc_curve import (
    arc_waypoints,
    densify_circle,
    densify_arc_from_dxf,
    densify_lwpolyline_bulge,
)


def test_densify_line_basic():
    """5m line at 0.5m spacing → 11 points."""
    pts = densify_line((0, 0), (5, 0), spacing=0.5)
    assert len(pts) == 11
    assert pts[0] == (0.0, 0.0)
    assert pts[-1] == (5.0, 0.0)
    # Check intermediate point spacing
    for i in range(1, len(pts)):
        d = math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
        assert abs(d - 0.5) < 0.01, f"Spacing at {i}: {d}"


def test_densify_line_diagonal():
    """Diagonal line preserves length."""
    start = (0.0, 0.0)
    end = (3.0, 4.0)  # 5m diagonal
    pts = densify_line(start, end, spacing=0.5)
    assert pts[0] == start
    assert pts[-1] == end
    # Total length should be ~5m
    total = sum(
        math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
        for i in range(1, len(pts))
    )
    assert abs(total - 5.0) < 0.05


def test_densify_line_endpoints_preserved():
    """Start and end points must be exactly the input values."""
    start = (1.23, 4.56)
    end = (7.89, 0.12)
    pts = densify_line(start, end, spacing=0.3)
    assert pts[0] == start
    assert pts[-1] == end


def test_densify_line_zero_length():
    """Zero-length line returns single point."""
    pts = densify_line((1.0, 2.0), (1.0, 2.0), spacing=0.05)
    assert len(pts) == 1
    assert pts[0] == (1.0, 2.0)


def test_densify_line_short_line():
    """Line shorter than spacing gets at least 2 points (start + end)."""
    pts = densify_line((0, 0), (0.01, 0.01), spacing=0.5)
    assert len(pts) >= 2
    assert pts[0] == (0, 0)
    assert pts[-1] == (0.01, 0.01)


def test_densify_line_spacing_accuracy():
    """All interior points should be within 1% of target spacing."""
    length = 2.0
    spacing = 0.05
    pts = densify_line((0, 0), (length, 0), spacing=spacing)
    for i in range(1, len(pts) - 1):
        d = abs(pts[i][0] - pts[i-1][0])
        assert abs(d - spacing) < spacing * 0.01, f"Point {i}: spacing={d}"


def test_densify_segment_mark():
    """MARK segment uses mark_spacing (default 5cm)."""
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (1.0, 0.0)],
        speed=0.35,
    )
    result = densify_segment(seg, mark_spacing=0.05, transit_spacing=0.15)
    assert result.segment_type == SegmentType.MARK
    assert result.speed == 0.35
    assert len(result.points) > 2  # Should be densified
    # Check spacing ≈ 0.05
    for i in range(1, len(result.points)):
        d = math.hypot(
            result.points[i][0] - result.points[i-1][0],
            result.points[i][1] - result.points[i-1][1],
        )
        assert abs(d - 0.05) < 0.01, f"MARK spacing at {i}: {d}"


def test_densify_segment_transit():
    """TRANSIT segment uses transit_spacing (default 15cm)."""
    seg = PathSegment(
        segment_type=SegmentType.TRANSIT,
        points=[(0.0, 0.0), (3.0, 0.0)],
        speed=0.50,
    )
    result = densify_segment(seg, mark_spacing=0.05, transit_spacing=0.15)
    assert result.segment_type == SegmentType.TRANSIT
    assert result.speed == 0.50
    # Check spacing ≈ 0.15
    for i in range(1, len(result.points)):
        d = abs(result.points[i][0] - result.points[i-1][0])
        assert abs(d - 0.15) < 0.01, f"TRANSIT spacing at {i}: {d}"


def test_densify_segment_single_point_passthrough():
    """Single-point segment (POINT entity) passes through unchanged."""
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(1.5, 2.5)],
        speed=0.35,
    )
    result = densify_segment(seg)
    assert result.points == [(1.5, 2.5)]


def test_densify_segment_preserves_metadata():
    """Densification preserves segment_id and source_entity."""
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0, 0), (1, 0)],
        speed=0.35,
        segment_id=42,
        source_entity="LINE_E042",
    )
    result = densify_segment(seg)
    assert result.segment_id == 42
    assert result.source_entity == "LINE_E042"


def test_densify_segment_multi_point():
    """Multi-point segment (L-shape) densifies each sub-segment."""
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (2.0, 0.0), (2.0, 1.5)],
        speed=0.35,
    )
    result = densify_segment(seg, mark_spacing=0.25)
    assert len(result.points) > len(seg.points)
    # Check that junction point (2,0) is preserved
    junction_found = any(
        abs(p[0] - 2.0) < 0.001 and abs(p[1]) < 0.001
        for p in result.points
    )
    assert junction_found, "Junction point should be preserved"


# ── Arc/circle discretization tests ────────────────────────────────────────────

TOLERANCE = 0.005  # 5mm position tolerance


def test_arc_quarter_circle():
    """Quarter circle R=1.5m from 0° to 90° CCW."""
    pts = arc_waypoints((0, 0), 1.5, 0, 90, chord_error=0.005)
    assert len(pts) >= 2
    # Start point: center + r*cos(0) east, r*sin(0) north → (0, 1.5) (north=0, east=1.5)
    # Wait — at angle 0: north = 0 + 1.5*sin(0) = 0, east = 0 + 1.5*cos(0) = 1.5
    assert abs(pts[0][0]) < TOLERANCE, f"Start north: {pts[0][0]}"
    assert abs(pts[0][1] - 1.5) < TOLERANCE, f"Start east: {pts[0][1]}"
    # End point: at 90°, north = 1.5*sin(90°) = 1.5, east = 1.5*cos(90°) = 0
    assert abs(pts[-1][0] - 1.5) < TOLERANCE, f"End north: {pts[-1][0]}"
    assert abs(pts[-1][1]) < TOLERANCE, f"End east: {pts[-1][1]}"


def test_arc_full_circle():
    """Full circle via densify_circle returns closed loop."""
    pts = densify_circle((0, 0), 1.0, chord_error=0.005)
    assert len(pts) >= 10
    # Last point should approximately match first
    assert abs(pts[-1][0] - pts[0][0]) < TOLERANCE
    assert abs(pts[-1][1] - pts[0][1]) < TOLERANCE


def test_arc_chord_error_bound():
    """Verify all points are within chord_error of the true arc."""
    center = (2.0, 3.0)
    radius = 0.5
    chord_error = 0.005
    pts = arc_waypoints(center, radius, 0, 270, chord_error=chord_error)

    # For each point, verify it's on the circle (distance from center ≈ radius)
    for pt in pts:
        d = math.hypot(pt[0] - center[0], pt[1] - center[1])
        assert abs(d - radius) < 0.01, f"Point off circle: d={d}, r={radius}"


def test_arc_spacing_bounds():
    """Verify waypoint spacing is within [min_spacing, max_spacing]."""
    radius = 2.0
    pts = arc_waypoints(
        (0, 0), radius, 0, 180,
        chord_error=0.005, min_spacing=0.02, max_spacing=0.10,
    )
    for i in range(1, len(pts)):
        d = math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
        # Spacing should be within bounds (allow small tolerance for endpoints)
        assert d <= 0.15, f"Spacing too large at {i}: {d}"
        assert d >= 0.005, f"Spacing too small at {i}: {d}"


def test_arc_zero_radius():
    """Zero radius returns single point."""
    pts = arc_waypoints((1.0, 2.0), 0.0, 0, 90)
    assert len(pts) == 1
    assert pts[0] == (1.0, 2.0)


def test_arc_small_radius():
    """Very small radius still produces points on the circle."""
    pts = arc_waypoints((0, 0), 0.05, 0, 360, chord_error=0.001)
    # All points should be near the circle
    for pt in pts:
        d = math.hypot(pt[0], pt[1])
        assert abs(d - 0.05) < 0.01


def test_arc_ccw_sweep():
    """CCW arc from 0° to 270° covers 3/4 of the circle."""
    pts = arc_waypoints((0, 0), 1.0, 0, 270, direction="CCW")
    # Start should be at angle 0 (east): north=0, east=1
    assert abs(pts[0][0]) < TOLERANCE
    assert abs(pts[0][1] - 1.0) < TOLERANCE


def test_arc_cw_sweep():
    """CW arc from 90° to 0° covers 90° clockwise."""
    pts = arc_waypoints((0, 0), 1.0, 90, 0, direction="CW")
    # Start at 90° (north): north=1, east=0
    assert abs(pts[0][0] - 1.0) < TOLERANCE
    assert abs(pts[0][1]) < TOLERANCE
    # End at 0° (east): north=0, east=1
    assert abs(pts[-1][0]) < TOLERANCE
    assert abs(pts[-1][1] - 1.0) < TOLERANCE


def test_arc_dxf_wrapping():
    """DXF arc from 350° to 10° wraps CCW through 360° (20° arc)."""
    pts = densify_arc_from_dxf((0, 0), 1.0, 350, 10)
    assert len(pts) >= 2
    # Should be a short arc, not a 350° arc
    # Check total arc length < 0.5m (20° of R=1m ≈ 0.35m)
    total_len = sum(
        math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
        for i in range(1, len(pts))
    )
    assert total_len < 0.5, f"Arc too long for 20° wrap: {total_len:.3f}m"


def test_circle_circumference():
    """Full circle circumference should be approximately 2πr."""
    radius = 1.5
    pts = densify_circle((0, 0), radius, chord_error=0.005)
    total_len = sum(
        math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
        for i in range(1, len(pts))
    )
    expected = 2 * math.pi * radius
    assert abs(total_len - expected) < 0.05, f"Circle len={total_len:.3f}, expected={expected:.3f}"


def test_circle_point_on_circle():
    """All points from densify_circle should lie on the circle."""
    center = (3.0, 4.0)
    radius = 2.0
    pts = densify_circle(center, radius, chord_error=0.005)
    for pt in pts:
        d = math.hypot(pt[0] - center[0], pt[1] - center[1])
        assert abs(d - radius) < TOLERANCE, f"Point off circle: d={d}"


# ── LWPOLYLINE bulge tests ────────────────────────────────────────────────────

def test_lwpolyline_straight_no_bulge():
    """LWPOLYLINE with all zero bulges → straight polyline."""
    vertices = [(0, 0), (2, 0), (2, 2)]
    bulges = [0.0, 0.0, 0.0]
    pts = densify_lwpolyline_bulge(vertices, bulges, closed=False)
    assert len(pts) >= 3
    assert abs(pts[0][0]) < TOLERANCE and abs(pts[0][1]) < TOLERANCE
    assert abs(pts[-1][0] - 2.0) < TOLERANCE and abs(pts[-1][1] - 2.0) < TOLERANCE


def test_lwpolyline_bulge_ccw():
    """Positive bulge creates CCW arc between vertices."""
    # Two vertices with a bulge of 0.5 (CCW arc)
    vertices = [(0.0, 0.0), (2.0, 0.0)]
    bulges = [0.5, 0.0]
    pts = densify_lwpolyline_bulge(vertices, bulges, closed=False)
    assert len(pts) >= 3  # More than 2 = arc was discretized
    # Start and end should match input vertices
    assert abs(pts[0][0]) < TOLERANCE and abs(pts[0][1]) < TOLERANCE
    assert abs(pts[-1][0] - 2.0) < TOLERANCE and abs(pts[-1][1]) < TOLERANCE


def test_lwpolyline_bulge_cw():
    """Negative bulge creates CW arc between vertices."""
    vertices = [(0.0, 0.0), (2.0, 0.0)]
    bulges = [-0.5, 0.0]
    pts = densify_lwpolyline_bulge(vertices, bulges, closed=False)
    assert len(pts) >= 3
    # Start and end should match
    assert abs(pts[0][0]) < TOLERANCE
    assert abs(pts[-1][0] - 2.0) < TOLERANCE


def test_lwpolyline_closed():
    """Closed LWPOLYLINE connects last vertex back to first."""
    # Square with no bulges
    vertices = [(0, 0), (1, 0), (1, 1), (0, 1)]
    bulges = [0.0, 0.0, 0.0, 0.0]
    pts = densify_lwpolyline_bulge(vertices, bulges, closed=True)
    # Last point should approximately match first
    assert abs(pts[-1][0] - pts[0][0]) < TOLERANCE
    assert abs(pts[-1][1] - pts[0][1]) < TOLERANCE


def test_lwpolyline_empty():
    """Empty vertices list returns empty."""
    pts = densify_lwpolyline_bulge([], [], closed=False)
    assert pts == []


def test_lwpolyline_mixed_line_and_arc():
    """LWPOLYLINE with some bulge and some straight segments."""
    # Start with straight, then arc, then straight
    vertices = [(0, 0), (2, 0), (4, 0), (4, 2)]
    bulges = [0.0, 0.3, 0.0, 0.0]
    pts = densify_lwpolyline_bulge(vertices, bulges, closed=False)
    assert len(pts) >= 4  # At least the 4 vertices
    # Start at (0,0)
    assert abs(pts[0][0]) < TOLERANCE and abs(pts[0][1]) < TOLERANCE
    # End at (4,2)
    assert abs(pts[-1][0] - 4.0) < TOLERANCE
    assert abs(pts[-1][1] - 2.0) < TOLERANCE