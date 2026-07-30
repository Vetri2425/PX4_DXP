"""Tests for path_engine.planners.extensions — drive extension logic.

Stage 3 scope: LINE and line-like LWPOLYLINE only.
ARC/CIRCLE/SPLINE/ELLIPSE/POINT are explicitly tested to confirm they are
NOT extended (deferred to Stage 7).

All coordinate pairs are (north_m, east_m) tuples in NED frame.
"""

from __future__ import annotations

import math
import pytest

from path_engine.core import PathSegment, SegmentType
from path_engine.planners.extensions import (
    _distance,
    _is_closed_run,
    _is_line_like_segment,
    _offset_point,
    _unit_vector,
    decompose_line_chain_to_edges,
    split_mark_segment_with_extensions,
)
from path_engine.optimizers.shape_grouping import group_connected_segments
from path_engine.engine import PathEngine


# ---------------------------------------------------------------------------
# Helper geometry tests
# ---------------------------------------------------------------------------

class TestUnitVector:
    def test_horizontal(self):
        uv = _unit_vector((0.0, 0.0), (5.0, 0.0))
        assert uv is not None
        assert abs(uv[0] - 1.0) < 1e-9
        assert abs(uv[1]) < 1e-9

    def test_vertical(self):
        uv = _unit_vector((0.0, 0.0), (0.0, 3.0))
        assert uv is not None
        assert abs(uv[0]) < 1e-9
        assert abs(uv[1] - 1.0) < 1e-9

    def test_coincident_returns_none(self):
        assert _unit_vector((1.0, 2.0), (1.0, 2.0)) is None

    def test_diagonal_normalised(self):
        uv = _unit_vector((0.0, 0.0), (3.0, 4.0))
        assert uv is not None
        assert abs(math.hypot(uv[0], uv[1]) - 1.0) < 1e-9


class TestOffsetPoint:
    def test_forward(self):
        p = _offset_point((0.0, 0.0), (1.0, 0.0), 0.5)
        assert abs(p[0] - 0.5) < 1e-9
        assert abs(p[1]) < 1e-9

    def test_backward(self):
        p = _offset_point((0.0, 0.0), (1.0, 0.0), -0.5)
        assert abs(p[0] - (-0.5)) < 1e-9
        assert abs(p[1]) < 1e-9


class TestIsLineLike:
    def test_line(self):
        seg = PathSegment(SegmentType.MARK, [(0, 0), (1, 0)], source_entity="LINE_E001")
        assert _is_line_like_segment(seg) is True

    def test_lwpolyline(self):
        seg = PathSegment(SegmentType.MARK, [(0, 0), (1, 0)], source_entity="LWPOLYLINE_42")
        assert _is_line_like_segment(seg) is True

    def test_polyline(self):
        seg = PathSegment(SegmentType.MARK, [(0, 0), (1, 0)], source_entity="POLYLINE_99")
        assert _is_line_like_segment(seg) is True

    def test_arc_excluded(self):
        seg = PathSegment(SegmentType.MARK, [(0, 0), (1, 0)], source_entity="ARC_A1")
        assert _is_line_like_segment(seg) is False

    def test_circle_excluded(self):
        seg = PathSegment(SegmentType.MARK, [(0, 0), (1, 0)], source_entity="CIRCLE_C1")
        assert _is_line_like_segment(seg) is False

    def test_spline_excluded(self):
        seg = PathSegment(SegmentType.MARK, [(0, 0), (1, 0)], source_entity="SPLINE_S1")
        assert _is_line_like_segment(seg) is False

    def test_ellipse_excluded(self):
        seg = PathSegment(SegmentType.MARK, [(0, 0), (1, 0)], source_entity="ELLIPSE_E1")
        assert _is_line_like_segment(seg) is False

    def test_point_excluded(self):
        seg = PathSegment(SegmentType.MARK, [(0, 0), (1, 0)], source_entity="POINT_P1")
        assert _is_line_like_segment(seg) is False

    def test_empty_source_excluded(self):
        seg = PathSegment(SegmentType.MARK, [(0, 0), (1, 0)], source_entity="")
        assert _is_line_like_segment(seg) is False

    def test_line_chain_metadata(self):
        seg = PathSegment(
            SegmentType.MARK,
            [(0, 0), (1, 0)],
            source_entity="group:anything",
            metadata={"geometry_type": "LINE_CHAIN", "grouped_from": ["LINE_E001"]},
        )
        assert _is_line_like_segment(seg) is True

    def test_grouped_line_sources(self):
        seg = PathSegment(
            SegmentType.MARK,
            [(0, 0), (1, 0)],
            source_entity="group:LINE_E001+LINE_E002",
        )
        assert _is_line_like_segment(seg) is True

    def test_grouped_line_source_with_count_suffix(self):
        seg = PathSegment(
            SegmentType.MARK,
            [(0, 0), (1, 0)],
            source_entity="group:LINE_E001+1",
        )
        assert _is_line_like_segment(seg) is True

    def test_grouped_arc_sources_excluded(self):
        seg = PathSegment(
            SegmentType.MARK,
            [(0, 0), (1, 0)],
            source_entity="group:ARC_A1+ARC_A2",
        )
        assert _is_line_like_segment(seg) is False

    def test_line_geometry_metadata(self):
        # dxf_parser now tags every LINE with geometry_type — classification is
        # by metadata, independent of the label.
        seg = PathSegment(
            SegmentType.MARK,
            [(0, 0), (1, 0)],
            source_entity="anything_at_all",
            metadata={"geometry_type": "LINE"},
        )
        assert _is_line_like_segment(seg) is True

    def test_curved_metadata_hard_excludes_line_like_label(self):
        # A curved geometry_type must win even when the label looks line-like —
        # this is the guard that keeps the smooth/segment profile split intact.
        seg = PathSegment(
            SegmentType.MARK,
            [(0, 0), (1, 0)],
            source_entity="LINE_DECOY",
            metadata={"geometry_type": "ARC"},
        )
        assert _is_line_like_segment(seg) is False

    def test_bulge_polyline_metadata_excluded(self):
        seg = PathSegment(
            SegmentType.MARK,
            [(0, 0), (1, 0)],
            source_entity="LWPOLYLINE_7",
            metadata={"geometry_type": "LWPOLYLINE_BULGE"},
        )
        assert _is_line_like_segment(seg) is False


class TestGroupedLineLikeExtensions:
    @pytest.mark.parametrize("prefix", ["LINE", "LWPOLYLINE", "POLYLINE"])
    def test_grouped_line_like_chain_gets_pre_mark_aft(self, prefix):
        segs = [
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[(0.0, 0.0), (1.0, 0.0)],
                speed=0.35,
                source_entity=f"{prefix}_E001",
            ),
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[(1.0, 0.0), (2.0, 0.0)],
                speed=0.35,
                source_entity=f"{prefix}_E002",
            ),
        ]

        grouped = group_connected_segments(segs)
        assert len(grouped) == 1
        assert grouped[0].source_entity.startswith(f"group:{prefix}_E001+")
        assert grouped[0].metadata["grouped_from"] == [
            f"{prefix}_E001",
            f"{prefix}_E002",
        ]

        result = split_mark_segment_with_extensions(
            grouped[0], pre_extension_m=0.5, aft_extension_m=0.5, transit_speed=0.50
        )

        assert [s.segment_type for s in result] == [
            SegmentType.TRANSIT,
            SegmentType.MARK,
            SegmentType.TRANSIT,
        ]
        assert result[0].points == [(-0.5, 0.0), (0.0, 0.0)]
        assert result[1].points == [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        assert result[2].points == [(2.0, 0.0), (2.5, 0.0)]
        assert result[0].metadata["parent_source_entity"] == grouped[0].source_entity
        assert result[2].metadata["parent_source_entity"] == grouped[0].source_entity

    def test_grouped_unknown_mark_chain_does_not_become_extendable(self):
        segs = [
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[(0.0, 0.0), (1.0, 0.0)],
                source_entity="A",
            ),
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[(1.0, 0.0), (2.0, 0.0)],
                source_entity="B",
            ),
        ]

        grouped = group_connected_segments(segs)
        assert len(grouped) == 1
        assert grouped[0].metadata["grouped_from"] == ["A", "B"]
        assert "line_like" not in grouped[0].metadata
        assert "geometry_type" not in grouped[0].metadata

        result = split_mark_segment_with_extensions(
            grouped[0], pre_extension_m=0.5, aft_extension_m=0.5, transit_speed=0.50
        )

        assert len(result) == 1
        assert result[0].segment_type == SegmentType.MARK
        assert result[0].points == [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]


class TestClosedRunExtensions:
    """Closed loops (square / triangle / closed polyline) get no linear
    PRE/AFT run-up — endpoints coincide, so an extension would stub into the
    shape. Open runs are unaffected."""

    def _square_chain(self):
        # Four connected lines forming a closed 2x2 square loop.
        edges = [
            ((0.0, 0.0), (2.0, 0.0)),
            ((2.0, 0.0), (2.0, 2.0)),
            ((2.0, 2.0), (0.0, 2.0)),
            ((0.0, 2.0), (0.0, 0.0)),
        ]
        segs = [
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[a, b],
                speed=0.35,
                source_entity=f"LINE_E{i:03d}",
            )
            for i, (a, b) in enumerate(edges)
        ]
        grouped = group_connected_segments(segs)
        assert len(grouped) == 1
        return grouped[0]

    def test_is_closed_run_detects_loop(self):
        chain = self._square_chain()
        assert _is_closed_run(chain.points) is True

    def test_is_closed_run_rejects_open_line(self):
        assert _is_closed_run([(0.0, 0.0), (2.0, 0.0)]) is False

    def test_is_closed_run_rejects_tiny_loop(self):
        # Endpoints coincide but perimeter < 1.0 m → not a real shape.
        tiny = [(0.0, 0.0), (0.1, 0.0), (0.1, 0.1), (0.0, 0.0)]
        assert _is_closed_run(tiny) is False

    def test_closed_square_gets_no_extensions(self):
        chain = self._square_chain()
        result = split_mark_segment_with_extensions(
            chain, pre_extension_m=0.5, aft_extension_m=0.5, transit_speed=0.50
        )
        # Suppressed: a single MARK, no TRANSIT lead-in/out.
        assert len(result) == 1
        assert result[0].segment_type == SegmentType.MARK
        assert result[0].points == chain.points

    def test_open_chain_still_gets_extensions(self):
        # Control: an OPEN line-like chain still receives the full triplet.
        segs = [
            PathSegment(SegmentType.MARK, [(0.0, 0.0), (1.0, 0.0)],
                        speed=0.35, source_entity="LINE_E001"),
            PathSegment(SegmentType.MARK, [(1.0, 0.0), (2.0, 0.0)],
                        speed=0.35, source_entity="LINE_E002"),
        ]
        grouped = group_connected_segments(segs)
        result = split_mark_segment_with_extensions(
            grouped[0], pre_extension_m=0.5, aft_extension_m=0.5, transit_speed=0.50
        )
        assert [s.segment_type for s in result] == [
            SegmentType.TRANSIT, SegmentType.MARK, SegmentType.TRANSIT,
        ]


# ---------------------------------------------------------------------------
# Stage 3 Test 1 — Simple horizontal line A→B
# ---------------------------------------------------------------------------

class TestHorizontalLine:
    """LINE from (0,0) to (10,0) with pre=0.5, aft=0.5."""

    def setup_method(self):
        self.seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 0.0), (10.0, 0.0)],
            speed=0.35,
            segment_id=1,
            source_entity="LINE_E001",
        )
        self.result = split_mark_segment_with_extensions(
            self.seg, pre_extension_m=0.5, aft_extension_m=0.5, transit_speed=0.50
        )

    def test_returns_three_segments(self):
        assert len(self.result) == 3

    def test_pre_is_transit(self):
        assert self.result[0].segment_type == SegmentType.TRANSIT

    def test_mark_is_mark(self):
        assert self.result[1].segment_type == SegmentType.MARK

    def test_aft_is_transit(self):
        assert self.result[2].segment_type == SegmentType.TRANSIT

    def test_pre_start_point(self):
        # PRE goes from (-0.5, 0) to (0, 0)
        pre_start = self.result[0].points[0]
        assert abs(pre_start[0] - (-0.5)) < 1e-6
        assert abs(pre_start[1]) < 1e-6

    def test_pre_end_equals_original_start(self):
        assert self.result[0].points[-1] == (0.0, 0.0)

    def test_mark_points_unchanged(self):
        assert self.result[1].points == [(0.0, 0.0), (10.0, 0.0)]

    def test_aft_start_equals_original_end(self):
        assert self.result[2].points[0] == (10.0, 0.0)

    def test_aft_end_point(self):
        # AFT goes from (10, 0) to (10.5, 0)
        aft_end = self.result[2].points[-1]
        assert abs(aft_end[0] - 10.5) < 1e-6
        assert abs(aft_end[1]) < 1e-6

    def test_original_not_mutated(self):
        # Original segment points must be intact after extension
        assert self.seg.points == [(0.0, 0.0), (10.0, 0.0)]

    def test_source_entity_labels(self):
        assert self.result[0].source_entity == "LINE_E001:pre"
        assert self.result[1].source_entity == "LINE_E001"
        assert self.result[2].source_entity == "LINE_E001:aft"

    def test_segment_ids_preserved(self):
        for s in self.result:
            assert s.segment_id == 1

    def test_transit_speed(self):
        assert self.result[0].speed == 0.50
        assert self.result[2].speed == 0.50

    def test_mark_speed_preserved(self):
        assert self.result[1].speed == 0.35


# ---------------------------------------------------------------------------
# Stage 3 Test 2 — Vertical line (north direction)
# ---------------------------------------------------------------------------

class TestVerticalLine:
    """LINE from (0,0) to (0,10) — eastward travel."""

    def setup_method(self):
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 0.0), (0.0, 10.0)],
            speed=0.35,
            source_entity="LINE_E002",
        )
        self.result = split_mark_segment_with_extensions(
            seg, pre_extension_m=0.5, aft_extension_m=0.5, transit_speed=0.50
        )

    def test_pre_start(self):
        # Direction is +east; PRE goes south (negative east)
        pre_start = self.result[0].points[0]
        assert abs(pre_start[0]) < 1e-6        # north unchanged
        assert abs(pre_start[1] - (-0.5)) < 1e-6  # east = -0.5

    def test_aft_end(self):
        aft_end = self.result[2].points[-1]
        assert abs(aft_end[0]) < 1e-6         # north unchanged
        assert abs(aft_end[1] - 10.5) < 1e-6  # east = 10.5


# ---------------------------------------------------------------------------
# Stage 3 Test 3 — Reverse line (east→west)
# ---------------------------------------------------------------------------

class TestReverseLine:
    """LINE from (10,0) to (0,0) — westward travel."""

    def setup_method(self):
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(10.0, 0.0), (0.0, 0.0)],
            speed=0.35,
            source_entity="LINE_E003",
        )
        self.result = split_mark_segment_with_extensions(
            seg, pre_extension_m=0.5, aft_extension_m=0.5, transit_speed=0.50
        )

    def test_pre_start(self):
        # Direction is -north; PRE steps backwards → +north → (10.5, 0)
        pre_start = self.result[0].points[0]
        assert abs(pre_start[0] - 10.5) < 1e-6
        assert abs(pre_start[1]) < 1e-6

    def test_aft_end(self):
        # AFT continues westward from (0,0) → (-0.5, 0)
        aft_end = self.result[2].points[-1]
        assert abs(aft_end[0] - (-0.5)) < 1e-6
        assert abs(aft_end[1]) < 1e-6


# ---------------------------------------------------------------------------
# Stage 3 Test 4 — Zero extension lengths (one or both)
# ---------------------------------------------------------------------------

class TestZeroExtensions:
    def _seg(self):
        return PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 0.0), (5.0, 0.0)],
            speed=0.35,
            source_entity="LINE_E004",
        )

    def test_both_zero_returns_one_segment(self):
        result = split_mark_segment_with_extensions(
            self._seg(), pre_extension_m=0.0, aft_extension_m=0.0, transit_speed=0.50
        )
        assert len(result) == 1
        assert result[0].segment_type == SegmentType.MARK

    def test_pre_zero_only(self):
        result = split_mark_segment_with_extensions(
            self._seg(), pre_extension_m=0.0, aft_extension_m=0.5, transit_speed=0.50
        )
        assert len(result) == 2
        assert result[0].segment_type == SegmentType.MARK
        assert result[1].segment_type == SegmentType.TRANSIT

    def test_aft_zero_only(self):
        result = split_mark_segment_with_extensions(
            self._seg(), pre_extension_m=0.5, aft_extension_m=0.0, transit_speed=0.50
        )
        assert len(result) == 2
        assert result[0].segment_type == SegmentType.TRANSIT
        assert result[1].segment_type == SegmentType.MARK


# ---------------------------------------------------------------------------
# Stage 3 Test 5 — Guard cases (TRANSIT, non-line-like, short segments)
# ---------------------------------------------------------------------------

class TestGuards:
    def test_transit_passthrough(self):
        """TRANSIT segments are returned as-is (copy)."""
        seg = PathSegment(
            segment_type=SegmentType.TRANSIT,
            points=[(0.0, 0.0), (5.0, 0.0)],
            speed=0.50,
            source_entity="LINE_E005",
        )
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        assert len(result) == 1
        assert result[0].segment_type == SegmentType.TRANSIT
        assert result[0].points == [(0.0, 0.0), (5.0, 0.0)]

    def test_transit_not_mutated(self):
        seg = PathSegment(SegmentType.TRANSIT, [(0, 0), (5, 0)],
                          source_entity="transit:1")
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        # The result is a copy; original unchanged
        assert seg.points == [(0, 0), (5, 0)]

    def test_arc_not_extended(self):
        """ARC segments must NOT be extended in Stage 3."""
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 0.0), (5.0, 5.0), (10.0, 0.0)],
            speed=0.35,
            source_entity="ARC_A1",
        )
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        assert len(result) == 1
        assert result[0].segment_type == SegmentType.MARK
        assert result[0].source_entity == "ARC_A1"

    def test_circle_not_extended(self):
        """CIRCLE segments must NOT be extended in Stage 3."""
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0), (1.0, 0.0)],
            speed=0.35,
            source_entity="CIRCLE_C1",
        )
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        assert len(result) == 1
        assert result[0].segment_type == SegmentType.MARK

    def test_single_point_passthrough(self):
        """Segments with < 2 points cannot define a direction — pass through."""
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(3.0, 4.0)],
            speed=0.35,
            source_entity="LINE_E006",
        )
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        assert len(result) == 1
        assert result[0].points == [(3.0, 4.0)]

    def test_empty_source_entity_passthrough(self):
        """Empty source_entity is not line-like — pass through."""
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 0.0), (5.0, 0.0)],
            speed=0.35,
            source_entity="",
        )
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        assert len(result) == 1
        assert result[0].segment_type == SegmentType.MARK


# ---------------------------------------------------------------------------
# Stage 3 Test 6 — Spray flags through full engine pipeline
# ---------------------------------------------------------------------------

class TestSprayFlagsThroughEngine:
    """Verify that spray flags are OFF on pre/aft and ON only on MARK geometry."""

    def _make_engine(self) -> PathEngine:
        return PathEngine(
            enable_path_extensions=True,
            pre_extension_m=0.5,
            aft_extension_m=0.5,
            optimize_order=False,      # disable optimizer for deterministic order
            compensate_spray=False,    # disable spray comp to keep points clean
        )

    def _make_seg(self) -> PathSegment:
        return PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 0.0), (10.0, 0.0)],
            speed=0.35,
            segment_id=1,
            source_entity="LINE_E010",
        )

    def test_spray_off_on_pre(self):
        plan = self._make_engine().plan_segments([self._make_seg()])
        # First point(s) are PRE extension — spray must be OFF
        assert plan.spray_flags[0] is False

    def test_spray_on_for_mark_region(self):
        plan = self._make_engine().plan_segments([self._make_seg()])
        # Find a flag that is True — must exist
        assert any(f is True for f in plan.spray_flags)

    def test_spray_off_on_aft(self):
        plan = self._make_engine().plan_segments([self._make_seg()])
        # Last point(s) are AFT extension — spray must be OFF
        assert plan.spray_flags[-1] is False

    def test_spray_flag_pattern(self):
        """Pattern must be: False(s) → True(s) → False(s)."""
        plan = self._make_engine().plan_segments([self._make_seg()])
        flags = plan.spray_flags
        # No True flag should appear before the first False→True transition
        # and no True flag after the last True→False transition.
        first_true = next(i for i, f in enumerate(flags) if f)
        last_true  = len(flags) - 1 - next(i for i, f in enumerate(reversed(flags)) if f)
        # All flags before first_true must be False
        assert all(not f for f in flags[:first_true])
        # All flags after last_true must be False
        assert all(not f for f in flags[last_true + 1:])

    def test_mark_length_is_original_only(self):
        """total_mark_length must equal only the original 10m segment."""
        plan = self._make_engine().plan_segments([self._make_seg()])
        assert abs(plan.total_mark_length - 10.0) < 0.01


class TestExtensionStitchingThroughEngine:
    """Regression coverage for CAD line chains split into per-edge extensions."""

    def _square_edges(self):
        corners = [
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (0.0, 0.0),
        ]
        return [
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[corners[i], corners[i + 1]],
                speed=0.35,
                source_entity=f"LINE_E{i}",
                metadata={"geometry_type": "LINE"},
            )
            for i in range(4)
        ]

    def test_square_without_extensions_preserves_cad_topology(self):
        engine = PathEngine(
            enable_path_extensions=False,
            optimize_order=False,
            compensate_spray=False,
            mark_spacing=2.0,
        )
        plan = engine.plan_segments(self._square_edges())

        assert [s.segment_type for s in plan.segments] == [SegmentType.MARK]
        assert plan.merged_waypoints == [
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (0.0, 0.0),
        ]

    def test_square_extensions_vertex_anchored_no_corner_spurs(self):
        """Vertex-anchored policy: a CLOSED square has no free end, so enabling
        extensions must NOT inject per-corner run-ups or diagonal AFT->PRE
        connector spurs. The grouped chain is recognised as a closed run and
        extensions are suppressed — output matches the no-extension topology."""
        engine = PathEngine(
            enable_path_extensions=True,
            pre_extension_m=0.5,
            aft_extension_m=0.5,
            optimize_order=False,
            compensate_spray=False,
            mark_spacing=2.0,
            transit_spacing=0.25,
        )
        plan = engine.plan_segments(self._square_edges())

        # No diagonal stitch connectors, and no out-and-back spurs.
        connectors = [
            s for s in plan.segments
            if s.metadata.get("extension_connector") is True
        ]
        assert connectors == []

        # The closed square collapses to a single clean MARK chain — identical
        # topology to enable_path_extensions=False.
        assert [s.segment_type for s in plan.segments] == [SegmentType.MARK]
        assert plan.merged_waypoints == [
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (0.0, 0.0),
        ]

        # No >100° heading reversal anywhere (the spur signature was 135°).
        wps = plan.merged_waypoints
        for i in range(1, len(wps) - 1):
            ax, ay = wps[i][0] - wps[i - 1][0], wps[i][1] - wps[i - 1][1]
            bx, by = wps[i + 1][0] - wps[i][0], wps[i + 1][1] - wps[i][1]
            la, lb = math.hypot(ax, ay), math.hypot(bx, by)
            if la < 1e-9 or lb < 1e-9:
                continue
            dot = max(-1.0, min(1.0, (ax * bx + ay * by) / (la * lb)))
            assert math.degrees(math.acos(dot)) <= 100.0

    def test_open_chain_extends_only_true_ends_dense(self):
        """Vertex-anchored policy on an OPEN L-chain: a run-up is added at the
        chain start and a run-out at the chain end (the two true open ends), the
        internal corner is left clean (no spur), and the run-ups are sampled at
        MARK spacing so they track as tightly as the mark line."""
        engine = PathEngine(
            enable_path_extensions=True,
            pre_extension_m=0.5,
            aft_extension_m=0.5,
            optimize_order=False,
            compensate_spray=False,
            mark_spacing=0.05,
            transit_spacing=0.5,
        )
        # Open L: (0,0)->(2,0) then (2,0)->(2,2). One internal 90° corner.
        plan = engine.plan_segments([
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[(0.0, 0.0), (2.0, 0.0)],
                speed=0.35,
                source_entity="LINE_L0",
                metadata={"geometry_type": "LINE"},
            ),
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[(2.0, 0.0), (2.0, 2.0)],
                speed=0.35,
                source_entity="LINE_L1",
                metadata={"geometry_type": "LINE"},
            ),
        ])

        roles = [s.metadata.get("extension_role") for s in plan.segments]
        assert roles.count("pre") == 1
        assert roles.count("aft") == 1
        # No diagonal stitch connectors at the internal corner.
        assert not any(
            s.metadata.get("extension_connector") is True for s in plan.segments
        )

        pre = next(s for s in plan.segments if s.metadata.get("extension_role") == "pre")
        aft = next(s for s in plan.segments if s.metadata.get("extension_role") == "aft")
        # Run-up colinear with first edge, ending at chain start (0,0).
        assert pre.points[-1] == (0.0, 0.0)
        assert abs(pre.points[0][0] - (-0.5)) < 1e-9 and abs(pre.points[0][1]) < 1e-9
        # Run-out colinear with last edge, starting at chain end (2,2).
        assert aft.points[0] == (2.0, 2.0)
        assert abs(aft.points[-1][0] - 2.0) < 1e-9 and abs(aft.points[-1][1] - 2.5) < 1e-9
        # Run-ups densified at MARK spacing (0.05): 0.5 m -> ~10 intervals.
        assert len(pre.points) >= 10
        assert len(aft.points) >= 10

    def test_spray_compensation_keeps_extensions_continuous(self):
        engine = PathEngine(
            enable_path_extensions=True,
            pre_extension_m=0.5,
            aft_extension_m=0.5,
            optimize_order=False,
            compensate_spray=True,
            mark_spacing=0.5,
            transit_spacing=0.25,
        )
        plan = engine.plan_segments([
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[(0.0, 0.0), (2.0, 0.0)],
                speed=0.35,
                source_entity="LINE_E0",
                metadata={"geometry_type": "LINE"},
            )
        ])

        for prev, curr in zip(plan.segments, plan.segments[1:]):
            assert _distance(prev.points[-1], curr.points[0]) < 1e-9

        # Coincident boundary waypoints must survive when their spray state
        # changes, otherwise lead-in/lead-out compensation is lost at flattening.
        boundary_pairs = [
            (a, fa, b, fb)
            for (a, fa), (b, fb) in zip(
                zip(plan.merged_waypoints, plan.spray_flags),
                zip(plan.merged_waypoints[1:], plan.spray_flags[1:]),
            )
            if _distance(a, b) < 1e-9 and fa != fb
        ]
        assert len(boundary_pairs) == 2


# ---------------------------------------------------------------------------
# Stage 3 Test 7 — Disabled mode: behavior identical to pre-extension code
# ---------------------------------------------------------------------------

class TestDisabledMode:
    """With enable_path_extensions=False, output must match old behavior."""

    def _seg(self):
        return PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 0.0), (10.0, 0.0)],
            speed=0.35,
            segment_id=1,
            source_entity="LINE_E020",
        )

    def test_disabled_no_extension_segments(self):
        engine = PathEngine(
            enable_path_extensions=False,
            optimize_order=False,
            compensate_spray=False,
        )
        plan = engine.plan_segments([self._seg()])
        # No per-line PRE/AFT extension segments are injected, but the engine
        # appends a terminal TRANSIT run-out so a mission that ends on MARK still
        # has a MARK->TRANSIT boundary for the spray node (otherwise spray
        # latches ON at the endpoint). That boundary is a duplicated coincident
        # vertex, so the last TWO flags are False — see the BOUNDARY TERMINATOR
        # block in engine.py.
        assert all(f is True for f in plan.spray_flags[:-2])
        assert plan.spray_flags[-2] is False
        assert plan.spray_flags[-1] is False

    def test_disabled_same_waypoint_count_as_legacy(self):
        """Disabled mode must produce exactly the same output as a vanilla engine."""
        legacy = PathEngine(optimize_order=False, compensate_spray=False)
        new_off = PathEngine(
            enable_path_extensions=False,
            optimize_order=False,
            compensate_spray=False,
        )
        seg = self._seg()
        plan_legacy = legacy.plan_segments([seg])
        plan_new    = new_off.plan_segments([seg])
        assert plan_legacy.merged_waypoints == plan_new.merged_waypoints
        assert plan_legacy.spray_flags == plan_new.spray_flags


# ---------------------------------------------------------------------------
# Stage 3 Test 8 — PathEngine config validation
# ---------------------------------------------------------------------------

class TestEngineConfigValidation:
    def test_negative_pre_raises(self):
        with pytest.raises(ValueError, match="pre_extension_m"):
            PathEngine(pre_extension_m=-0.1)

    def test_negative_aft_raises(self):
        with pytest.raises(ValueError, match="aft_extension_m"):
            PathEngine(aft_extension_m=-0.1)

    def test_zero_pre_is_valid(self):
        engine = PathEngine(pre_extension_m=0.0, aft_extension_m=0.0)
        assert engine.pre_extension_m == 0.0

    def test_default_disabled(self):
        """Default engine must have extensions disabled for safe rollout."""
        engine = PathEngine()
        assert engine.enable_path_extensions is False


# ---------------------------------------------------------------------------
# Stage 3 Test 9 — LWPOLYLINE line-like profile
# ---------------------------------------------------------------------------

class TestLWPolylineExtension:
    """Multi-point polyline should also get PRE/AFT extensions."""

    def test_lwpolyline_three_points(self):
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)],
            speed=0.35,
            source_entity="LWPOLYLINE_E030",
        )
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        assert len(result) == 3
        # PRE direction from (0,0)→(5,0) is +north, so pre_start = (-0.5, 0)
        pre_start = result[0].points[0]
        assert abs(pre_start[0] - (-0.5)) < 1e-6
        assert abs(pre_start[1]) < 1e-6
        # AFT direction from (5,0)→(10,0) is +north, so aft_end = (10.5, 0)
        aft_end = result[2].points[-1]
        assert abs(aft_end[0] - 10.5) < 1e-6
        assert abs(aft_end[1]) < 1e-6

    def test_lwpolyline_angled(self):
        """L-shaped polyline: direction at end differs from start."""
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)],
            speed=0.35,
            source_entity="LWPOLYLINE_E031",
        )
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        assert len(result) == 3
        # PRE: start direction is +north from (0,0)→(5,0), pre_start = (-0.5, 0)
        pre_start = result[0].points[0]
        assert abs(pre_start[0] - (-0.5)) < 1e-6
        # AFT: end direction is +east from (5,0)→(5,5), aft_end = (5, 5.5)
        aft_end = result[2].points[-1]
        assert abs(aft_end[0] - 5.0) < 1e-6
        assert abs(aft_end[1] - 5.5) < 1e-6


# ---------------------------------------------------------------------------
# Stage 7 — ARC / CIRCLE extension using metadata tangents
# ---------------------------------------------------------------------------

TOL = 1e-4   # 0.1 mm tolerance for all Stage 7 geometry checks


def _make_arc_seg(
    start_angle_deg: float,
    end_angle_deg: float,
    radius: float = 1.0,
    center: tuple = (0.0, 0.0),
    seg_id: int = 1,
) -> PathSegment:
    """Build a minimal ARC PathSegment with correct metadata (mirrors dxf_parser)."""
    from path_engine.planners.arc_curve import arc_waypoints
    pts = arc_waypoints(center, radius, start_angle_deg, end_angle_deg,
                        chord_error=0.001, direction="CCW")
    a_start = math.radians(start_angle_deg)
    a_end   = math.radians(end_angle_deg)
    return PathSegment(
        segment_type=SegmentType.MARK,
        points=pts,
        speed=0.35,
        segment_id=seg_id,
        source_entity="ARC_T1",
        metadata={
            "geometry_type": "ARC",
            "start_tangent": (math.cos(a_start), -math.sin(a_start)),
            "end_tangent":   (math.cos(a_end),   -math.sin(a_end)),
            "direction": "CCW",
        },
    )


def _make_circle_seg(radius: float = 1.0, center: tuple = (0.0, 0.0)) -> PathSegment:
    """Build a minimal CIRCLE PathSegment with correct metadata (mirrors dxf_parser)."""
    from path_engine.planners.arc_curve import densify_circle
    pts = densify_circle(center, radius, chord_error=0.001)
    return PathSegment(
        segment_type=SegmentType.MARK,
        points=pts,
        speed=0.35,
        source_entity="CIRCLE_T1",
        metadata={
            "geometry_type": "CIRCLE",
            "start_tangent": (1.0, 0.0),
            "end_tangent":   (1.0, 0.0),
            "direction": "CCW",
        },
    )


class TestArcExtension_0to90:
    """ARC 0deg to 90deg CCW r=1 center=(0,0).

    Geometry:
      start at 0deg  = East point  = (north=0, east=1)
      end   at 90deg = North point = (north=1, east=0)
      CCW tangent at  0deg: (cos  0, -sin  0) = (+1,  0) heading North
      CCW tangent at 90deg: (cos 90, -sin 90) = ( 0, -1) heading West

    PRE: step backward (South) from East point  -> (-0.5, 1)
    AFT: step forward  (West)  from North point -> (1,  -0.5)
    """

    def setup_method(self):
        self.seg = _make_arc_seg(0, 90, radius=1.0)
        self.result = split_mark_segment_with_extensions(
            self.seg, pre_extension_m=0.5, aft_extension_m=0.5, transit_speed=0.50
        )

    def test_three_segments(self):
        assert len(self.result) == 3

    def test_types(self):
        assert self.result[0].segment_type == SegmentType.TRANSIT
        assert self.result[1].segment_type == SegmentType.MARK
        assert self.result[2].segment_type == SegmentType.TRANSIT

    def test_pre_start_south_of_east_point(self):
        pre_start = self.result[0].points[0]
        assert abs(pre_start[0] - (-0.5)) < TOL, f"north={pre_start[0]}"
        assert abs(pre_start[1] - 1.0)   < TOL, f"east={pre_start[1]}"

    def test_pre_end_equals_arc_start(self):
        arc_start = self.seg.points[0]
        pre_end = self.result[0].points[-1]
        assert abs(pre_end[0] - arc_start[0]) < TOL
        assert abs(pre_end[1] - arc_start[1]) < TOL

    def test_aft_end_west_of_north_point(self):
        aft_end = self.result[2].points[-1]
        assert abs(aft_end[0] - 1.0)    < TOL, f"north={aft_end[0]}"
        assert abs(aft_end[1] - (-0.5)) < TOL, f"east={aft_end[1]}"

    def test_mark_metadata_preserved(self):
        m = self.result[1].metadata
        assert "start_tangent" in m
        assert "end_tangent"   in m
        assert m["geometry_type"] == "ARC"

    def test_pre_extension_role_metadata(self):
        m = self.result[0].metadata
        assert m.get("extension_role") == "pre"
        assert m.get("parent_source_entity") == "ARC_T1"

    def test_aft_extension_role_metadata(self):
        m = self.result[2].metadata
        assert m.get("extension_role") == "aft"
        assert m.get("parent_source_entity") == "ARC_T1"

    def test_original_not_mutated(self):
        assert self.seg.metadata["geometry_type"] == "ARC"
        assert len(self.seg.points) > 0


class TestArcExtension_0to180:
    """ARC 0deg to 180deg CCW r=1 center=(0,0) - Stage 7A correction test.

    Geometry:
      start at   0deg = East point = (north=0, east=+1)
      end   at 180deg = West point = (north=0, east=-1)
      CCW tangent at   0deg: (cos   0, -sin   0) = (+1,  0) North
      CCW tangent at 180deg: (cos 180, -sin 180) = (-1,  0) South

    PRE: step backward (South) from East point -> (-0.5, +1)
    AFT: step forward  (South) from West point -> (-0.5, -1)

    Stage 7A audit table listed AFT as East side of West point - WRONG.
    Correct: AFT is SOUTH of the West endpoint, as confirmed here.
    """

    def setup_method(self):
        self.seg = _make_arc_seg(0, 180, radius=1.0)
        self.result = split_mark_segment_with_extensions(
            self.seg, pre_extension_m=0.5, aft_extension_m=0.5, transit_speed=0.50
        )

    def test_pre_south_of_east_point(self):
        pre_start = self.result[0].points[0]
        assert abs(pre_start[0] - (-0.5)) < TOL, f"Expected north=-0.5, got {pre_start[0]}"
        assert abs(pre_start[1] - 1.0)   < TOL, f"Expected east=+1.0, got {pre_start[1]}"

    def test_aft_south_of_west_point(self):
        """Verifies the Stage 7A correction: AFT is SOUTH of the West endpoint."""
        aft_end = self.result[2].points[-1]
        assert abs(aft_end[0] - (-0.5)) < TOL, f"Expected north=-0.5, got {aft_end[0]}"
        assert abs(aft_end[1] - (-1.0)) < TOL, f"Expected east=-1.0, got {aft_end[1]}"


class TestCircleExtension:
    """CIRCLE r=1 center=(0,0) starts at 0deg=East point, travels CCW.

    Geometry:
      start/end at East point = (north=0, east=+1)
      CCW tangent at 0deg: (cos 0, -sin 0) = (+1, 0) heading North

    PRE: step backward (South) from East point -> (north=-0.5, east=+1)
    AFT: step forward  (North) from East point -> (north=+0.5, east=+1)
    """

    def setup_method(self):
        self.seg = _make_circle_seg(radius=1.0)
        self.result = split_mark_segment_with_extensions(
            self.seg, pre_extension_m=0.5, aft_extension_m=0.5, transit_speed=0.50
        )

    def test_three_segments(self):
        assert len(self.result) == 3

    def test_pre_south_of_east_point(self):
        pre_start = self.result[0].points[0]
        assert abs(pre_start[0] - (-0.5)) < TOL, f"north={pre_start[0]}"
        assert abs(pre_start[1] - 1.0)   < TOL, f"east={pre_start[1]}"

    def test_aft_north_of_east_point(self):
        aft_end = self.result[2].points[-1]
        assert abs(aft_end[0] - 0.5) < TOL, f"north={aft_end[0]}"
        assert abs(aft_end[1] - 1.0) < TOL, f"east={aft_end[1]}"

    def test_circle_metadata_preserved(self):
        m = self.result[1].metadata
        assert m.get("geometry_type") == "CIRCLE"

    def test_spray_flag_pattern_through_engine(self):
        """PRE=False, CIRCLE=True, AFT=False through full engine pipeline."""
        engine = PathEngine(
            enable_path_extensions=True,
            pre_extension_m=0.5,
            aft_extension_m=0.5,
            optimize_order=False,
            compensate_spray=False,
        )
        plan = engine.plan_segments([self.seg])
        flags = plan.spray_flags
        assert flags[0]  is False, "First flag (PRE) must be False"
        assert flags[-1] is False, "Last flag (AFT) must be False"
        assert any(f is True for f in flags), "Must have True (CIRCLE MARK) flags"
        first_true = next(i for i, f in enumerate(flags) if f)
        last_true  = len(flags) - 1 - next(i for i, f in enumerate(reversed(flags)) if f)
        assert all(not f for f in flags[:first_true])
        assert all(f     for f in flags[first_true:last_true + 1])
        assert all(not f for f in flags[last_true + 1:])


class TestArcWithoutMetadata:
    """ARC PathSegment without metadata (not from dxf_parser).
    Must NOT be extended — safe fallback for manually-built segments.
    """

    def test_arc_no_metadata_not_extended(self):
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 1.0), (0.5, 0.866), (1.0, 0.0)],
            speed=0.35,
            source_entity="ARC_A999",
        )
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        assert len(result) == 1
        assert result[0].source_entity == "ARC_A999"

    def test_circle_no_metadata_not_extended(self):
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 1.0), (1.0, 0.0), (0.0, -1.0), (-1.0, 0.0), (0.0, 1.0)],
            speed=0.35,
            source_entity="CIRCLE_C999",
        )
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        assert len(result) == 1


class TestOptimizerReversalMetadata:
    """Optimizer reversal must swap and negate start_tangent/end_tangent.

    ARC 0deg to 90deg CCW (r=1):
      original start_tangent = (+1,  0)  North at East point
      original end_tangent   = ( 0, -1)  West at North point

    After reversal (rover enters from North point, travels CW to East):
      new start_tangent = -old end_tangent   = ( 0, +1)  East at North point
      new end_tangent   = -old start_tangent = (-1,  0)  South at East point
    """

    def setup_method(self):
        from path_engine.optimizers.segment_order import optimize_segment_order
        arc = _make_arc_seg(0, 90, radius=1.0, seg_id=1)
        # Start optimizer at arc end (North point ~(1,0)) to force reversal
        arc_end = arc.points[-1]
        ordered = optimize_segment_order([arc], start_position=arc_end)
        self.rev = next(s for s in ordered if s.segment_type == SegmentType.MARK)

    def test_reversal_flagged(self):
        assert self.rev.metadata.get("reversed") is True

    def test_new_start_tangent(self):
        """new start_tangent = -old end_tangent = (0, +1)."""
        st = self.rev.metadata["start_tangent"]
        assert abs(st[0] - 0.0) < TOL, f"north={st[0]}"
        assert abs(st[1] - 1.0) < TOL, f"east={st[1]}"

    def test_new_end_tangent(self):
        """new end_tangent = -old start_tangent = (-1, 0)."""
        et = self.rev.metadata["end_tangent"]
        assert abs(et[0] - (-1.0)) < TOL, f"north={et[0]}"
        assert abs(et[1] - 0.0)    < TOL, f"east={et[1]}"

    def test_reversed_arc_pre_west_of_north_point(self):
        """PRE of reversed arc: step West from North point -> (1, -0.5)."""
        result = split_mark_segment_with_extensions(self.rev, 0.5, 0.5, 0.50)
        assert len(result) == 3
        pre_start = result[0].points[0]
        assert abs(pre_start[0] - 1.0)    < TOL, f"north={pre_start[0]}"
        assert abs(pre_start[1] - (-0.5)) < TOL, f"east={pre_start[1]}"

    def test_reversed_arc_aft_south_of_east_point(self):
        """AFT of reversed arc: step South from East point -> (-0.5, 1)."""
        result = split_mark_segment_with_extensions(self.rev, 0.5, 0.5, 0.50)
        aft_end = result[2].points[-1]
        assert abs(aft_end[0] - (-0.5)) < TOL, f"north={aft_end[0]}"
        assert abs(aft_end[1] - 1.0)    < TOL, f"east={aft_end[1]}"


class TestMetadataPropagationChain:
    """Metadata must survive every pipeline stage."""

    def test_metadata_survives_densify(self):
        from path_engine.planners.straight_line import densify_segment
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 0.0), (1.0, 0.0)],
            speed=0.35,
            source_entity="LINE_META1",
            metadata={"start_tangent": (1.0, 0.0), "end_tangent": (1.0, 0.0),
                      "geometry_type": "ARC"},
        )
        result = densify_segment(seg, mark_spacing=0.05)
        assert "start_tangent" in result.metadata
        assert result.metadata["geometry_type"] == "ARC"

    def test_metadata_survives_spray_compensation(self):
        from path_engine.spray import apply_spray_latency_compensation
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 0.0), (1.0, 0.0)],
            speed=0.35,
            source_entity="LINE_META2",
            metadata={"geometry_type": "ARC", "start_tangent": (1.0, 0.0),
                      "end_tangent": (1.0, 0.0)},
        )
        result = apply_spray_latency_compensation(seg)
        assert result.metadata.get("geometry_type") == "ARC"

    def test_mark_copy_carries_metadata_by_value(self):
        """MARK output of split must be a metadata copy, not same object."""
        seg = _make_arc_seg(0, 90, radius=1.0)
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        mark_out = result[1]
        assert mark_out.metadata is not seg.metadata, "must be a copy"
        assert mark_out.metadata["geometry_type"] == "ARC"

    def test_pre_transit_has_extension_role_only(self):
        """PRE transit has extension_role but NOT geometry_type."""
        seg = _make_arc_seg(0, 90, radius=1.0)
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        pre_meta = result[0].metadata
        assert pre_meta.get("extension_role") == "pre"
        assert "geometry_type" not in pre_meta

    def test_aft_transit_has_extension_role_only(self):
        """AFT transit has extension_role but NOT geometry_type."""
        seg = _make_arc_seg(0, 90, radius=1.0)
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        aft_meta = result[2].metadata
        assert aft_meta.get("extension_role") == "aft"
        assert "geometry_type" not in aft_meta

    def test_single_point_passthrough_preserves_metadata(self):
        """Single-point passthrough guard must still copy metadata."""
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(1.0, 2.0)],
            speed=0.35,
            source_entity="ARC_SINGLE",
            metadata={"geometry_type": "ARC"},
        )
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        assert result[0].metadata.get("geometry_type") == "ARC"

    def test_transit_passthrough_preserves_metadata(self):
        """TRANSIT guard must still copy metadata."""
        seg = PathSegment(
            segment_type=SegmentType.TRANSIT,
            points=[(0.0, 0.0), (5.0, 0.0)],
            speed=0.50,
            source_entity="transit:1",
            metadata={"some_key": "some_value"},
        )
        result = split_mark_segment_with_extensions(seg, 0.5, 0.5, 0.50)
        assert result[0].metadata.get("some_key") == "some_value"


class TestPerLineExtensions:
    """Per-line mode: every CAD line is an independent PRE→MARK→AFT pass, even on
    a closed square. Decomposition + suppress_closed_loops=False bypass the
    connectivity policy. Legacy mode (default) stays byte-identical."""

    def _square_chain(self):
        edges = [
            ((0.0, 0.0), (2.0, 0.0)),
            ((2.0, 0.0), (2.0, 2.0)),
            ((2.0, 2.0), (0.0, 2.0)),
            ((0.0, 2.0), (0.0, 0.0)),
        ]
        segs = [
            PathSegment(SegmentType.MARK, [a, b], speed=0.35,
                        source_entity=f"LINE_E{i:03d}")
            for i, (a, b) in enumerate(edges)
        ]
        grouped = group_connected_segments(segs)
        assert len(grouped) == 1
        return grouped[0]

    # ── decomposition ──────────────────────────────────────────────────────
    def test_decompose_square_into_four_edges(self):
        chain = self._square_chain()
        edges = decompose_line_chain_to_edges(chain)
        assert len(edges) == 4
        for e in edges:
            assert e.segment_type == SegmentType.MARK
            assert len(e.points) >= 2
            # each edge is a straight, OPEN line (start != end)
            assert _is_closed_run(e.points) is False

    def test_decompose_single_line_is_unchanged(self):
        seg = PathSegment(SegmentType.MARK, [(0.0, 0.0), (2.0, 0.0)],
                          speed=0.35, source_entity="LINE_E001")
        assert decompose_line_chain_to_edges(seg) == [seg]

    def test_decompose_keeps_curves_whole(self):
        # An arc-like segment must NOT be split — it keeps tangent extensions.
        seg = PathSegment(
            SegmentType.MARK,
            [(0.0, 0.0), (0.5, 0.3), (1.0, 0.8), (1.4, 1.4)],
            speed=0.35, source_entity="ARC_A1",
            metadata={"geometry_type": "ARC"},
        )
        assert decompose_line_chain_to_edges(seg) == [seg]

    # ── suppress_closed_loops flag ─────────────────────────────────────────
    def test_closed_square_extends_per_line_when_unsuppressed(self):
        # The decomposed edges are open, so each gets a full PRE/MARK/AFT triplet
        # even though the parent shape was closed.
        chain = self._square_chain()
        for edge in decompose_line_chain_to_edges(chain):
            parts = split_mark_segment_with_extensions(
                edge, 0.5, 0.5, 0.50, suppress_closed_loops=False
            )
            roles = [p.metadata.get("extension_role") for p in parts]
            assert roles == ["pre", None, "aft"]
            assert [p.segment_type for p in parts] == [
                SegmentType.TRANSIT, SegmentType.MARK, SegmentType.TRANSIT
            ]

    def test_suppress_default_still_blocks_closed_square(self):
        # Legacy guard intact: the whole closed chain is still suppressed.
        chain = self._square_chain()
        result = split_mark_segment_with_extensions(chain, 0.5, 0.5, 0.50)
        assert len(result) == 1 and result[0].segment_type == SegmentType.MARK

    # ── full engine plan ───────────────────────────────────────────────────
    def _square_segs(self):
        edges = [
            ((0.0, 0.0), (2.0, 0.0)), ((2.0, 0.0), (2.0, 2.0)),
            ((2.0, 2.0), (0.0, 2.0)), ((0.0, 2.0), (0.0, 0.0)),
        ]
        return [
            PathSegment(SegmentType.MARK, [a, b], speed=0.35,
                        source_entity=f"LINE_E{i:03d}",
                        metadata={"geometry_type": "LINE", "line_like": True})
            for i, (a, b) in enumerate(edges)
        ]

    def test_engine_per_line_square_gives_four_passes(self):
        """per_line means what it says: every CAD line is an INDEPENDENT PRE/MARK/AFT.

        Each side is approached already settled on-line and up to speed, marked dead
        straight, and exited — rather than the rover pivoting through the corner
        mid-spray. That is the whole point of the mode.
        """
        eng = PathEngine(enable_path_extensions=True, pre_extension_m=0.5,
                         aft_extension_m=0.5, per_line_extensions=True,
                         compensate_spray=False, optimize_order=False)
        plan = eng.plan_segments(self._square_segs())
        pre = sum(1 for s in plan.segments if s.metadata.get("extension_role") == "pre")
        aft = sum(1 for s in plan.segments if s.metadata.get("extension_role") == "aft")
        mark = sum(1 for s in plan.segments
                   if s.segment_type == SegmentType.MARK
                   and not s.metadata.get("extension_role"))
        assert (pre, mark, aft) == (4, 4, 4)

    def test_engine_per_line_never_doubles_back_on_its_own_aft(self):
        """The cost of per-line is travel between lines — never a RETRACE.

        Consecutive edges no longer touch, so the rover drives out along edge N's AFT,
        turns, and comes back to edge N+1's PRE. Those connectors must be routed AFTER
        extension so they span AFT-tip -> next-PRE-start directly. Routing them BEFORE
        (the old Step 3 behaviour) left them wired to the ORIGINAL mark endpoints, so
        the rover had to drive 180 deg back over its own AFT to reach the stale link —
        cancelling the run-out entirely. That is the d82317d field failure.

        A pivot at a corner is fine (it is spray-off, and stop-pivot handles it). Driving
        back over ground it just covered is not.
        """
        eng = PathEngine(enable_path_extensions=True, pre_extension_m=0.5,
                         aft_extension_m=0.5, per_line_extensions=True,
                         compensate_spray=False, optimize_order=True)
        wps = eng.plan_segments(self._square_segs()).merged_waypoints
        for i in range(1, len(wps) - 1):
            ax, ay = wps[i][0] - wps[i - 1][0], wps[i][1] - wps[i - 1][1]
            bx, by = wps[i + 1][0] - wps[i][0], wps[i + 1][1] - wps[i][1]
            la, lb = math.hypot(ax, ay), math.hypot(bx, by)
            if la < 1e-9 or lb < 1e-9:
                continue
            dot = max(-1.0, min(1.0, (ax * bx + ay * by) / (la * lb)))
            ang = math.degrees(math.acos(dot))
            assert ang < 170.0, f"near-reversal of {ang:.0f} deg at {wps[i]} — retracing"

    def test_engine_per_line_spray_flags_off_on_off(self):
        """4 lines → exactly 8 spray toggles (off→on then on→off per line)."""
        eng = PathEngine(enable_path_extensions=True, per_line_extensions=True,
                         compensate_spray=False, optimize_order=False)
        plan = eng.plan_segments(self._square_segs())
        toggles = sum(1 for a, b in zip(plan.spray_flags, plan.spray_flags[1:]) if a != b)
        assert toggles == 8
        assert any(plan.spray_flags) and not all(plan.spray_flags)
        # Spray OFF on the leading run-up and the trailing run-out.
        assert plan.spray_flags[0] is False
        assert plan.spray_flags[-1] is False

    def test_engine_per_line_densifies_runups_at_mark_spacing(self):
        eng = PathEngine(enable_path_extensions=True, pre_extension_m=0.5,
                         aft_extension_m=0.5, per_line_extensions=True,
                         mark_spacing=0.05, compensate_spray=False,
                         optimize_order=False)
        plan = eng.plan_segments(self._square_segs())
        for s in plan.segments:
            if s.metadata.get("extension_role") in ("pre", "aft"):
                # 0.5 m at 0.05 m spacing → ~11 points, definitely not a 2-pt jump.
                assert len(s.points) >= 8

    def test_engine_legacy_mode_unchanged(self):
        # per_line OFF → closed square still suppressed (one MARK, no extensions).
        eng = PathEngine(enable_path_extensions=True, per_line_extensions=False,
                         compensate_spray=False, optimize_order=False)
        plan = eng.plan_segments(self._square_segs())
        assert not any(s.metadata.get("extension_role") for s in plan.segments)


class TestInterEntityTransitRouting:
    """The transit between two entities must leave from the AFT tip and arrive at
    the next PRE start — never from the original mark endpoints.

    The bug: `optimize_segment_order` generated its TRANSIT links in Step 3, wiring
    the ORIGINAL mark endpoints together. Step 4 then wrapped each mark in PRE/AFT,
    which left those links connected to nothing, and the connector pass could only
    reconcile them by driving the rover BACKWARDS over its own AFT:

        circle MARK ends (1.00, 1.95)
        AFT out to       (1.50, 1.95)
        <<< 180 deg
        back to          (1.00, 1.95)   <- retraces the AFT, cancelling it
        transit to       (0.00, 0.00)   <- the stale link, from the ORIGINAL endpoint
        overshoot to     (0.00,-0.50)
        <<< 180 deg
        back to          (0.00, 0.00)   <- that was the "PRE"
        square MARK begins

    Two pointless 180 deg reversals per transition, and the run-up/run-out exactly
    cancelled — the rover still entered every spray boundary from a standstill, which
    is the one thing extensions exist to prevent.

    Fix: Step 3 withholds its transit links when extensions are on, so the connector
    pass runs AFT-tip -> next-PRE-start directly.
    """

    @staticmethod
    def _two_separate_lines() -> list[PathSegment]:
        # Two parallel lines, far apart: the rover must deadhead between them.
        return [
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[(0.0, 0.0), (2.0, 0.0)],
                speed=0.35,
                source_entity="LINE_A",
                metadata={"geometry_type": "LINE", "line_like": True},
            ),
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[(0.0, 5.0), (2.0, 5.0)],
                speed=0.35,
                source_entity="LINE_B",
                metadata={"geometry_type": "LINE", "line_like": True},
            ),
        ]

    def _plan(self):
        eng = PathEngine(
            enable_path_extensions=True,
            pre_extension_m=0.5,
            aft_extension_m=0.5,
            per_line_extensions=False,
            compensate_spray=False,
            optimize_order=True,
            mark_spacing=0.05,
            transit_spacing=0.15,
        )
        return eng.plan_segments(self._two_separate_lines())

    def test_no_reversal_between_entities(self):
        """No near-180 deg turn anywhere: the rover must not double back on itself."""
        wps = self._plan().merged_waypoints
        for i in range(1, len(wps) - 1):
            ax, ay = wps[i][0] - wps[i - 1][0], wps[i][1] - wps[i - 1][1]
            bx, by = wps[i + 1][0] - wps[i][0], wps[i + 1][1] - wps[i][1]
            la, lb = math.hypot(ax, ay), math.hypot(bx, by)
            if la < 1e-9 or lb < 1e-9:
                continue  # zero-length step = deliberate spray-boundary duplicate
            dot = max(-1.0, min(1.0, (ax * bx + ay * by) / (la * lb)))
            ang = math.degrees(math.acos(dot))
            assert ang < 170.0, (
                f"near-reversal of {ang:.1f} deg at {wps[i]} — the rover is doubling "
                "back on itself (stale transit link routed before extensions)"
            )

    def test_transit_leaves_from_aft_tip_not_the_mark_endpoint(self):
        """The connector must start where the AFT ends, not where the MARK ended."""
        plan = self._plan()
        segs = plan.segments
        aft_idx = next(i for i, s in enumerate(segs)
                       if s.metadata.get("extension_role") == "aft")
        aft = segs[aft_idx]
        # The AFT is not the last segment: travel to the next run follows it.
        assert aft_idx < len(segs) - 1
        nxt = segs[aft_idx + 1]
        assert nxt.segment_type == SegmentType.TRANSIT
        # Contiguous: the connector begins exactly at the AFT tip.
        assert math.dist(nxt.points[0], aft.points[-1]) < 1e-6, (
            f"transit starts at {nxt.points[0]} but the AFT ends at {aft.points[-1]} — "
            "the rover would have to drive back over its own run-out to reach it"
        )

    def test_transit_arrives_at_the_pre_start(self):
        """...and it must hand off to the next run's PRE, not overshoot past it."""
        plan = self._plan()
        segs = plan.segments
        pre_idxs = [i for i, s in enumerate(segs)
                    if s.metadata.get("extension_role") == "pre"]
        # The second run's PRE is preceded by the inter-run transit.
        second_pre = pre_idxs[-1]
        assert second_pre > 0
        prev = segs[second_pre - 1]
        assert prev.segment_type == SegmentType.TRANSIT
        assert math.dist(prev.points[-1], segs[second_pre].points[0]) < 1e-6

    def test_path_is_continuous(self):
        """Every segment starts where the previous one ended — no teleports."""
        segs = self._plan().segments
        for a, b in zip(segs, segs[1:]):
            assert math.dist(a.points[-1], b.points[0]) < 1e-6, (
                f"discontinuity: {a.source_entity} ends {a.points[-1]}, "
                f"{b.source_entity} starts {b.points[0]}"
            )


class TestDensifyIdempotence:
    """densify must be idempotent: extensions get densified when built AND again in
    the re-densify pass, and TRANSIT connectors can pass through more than once.

    Without a float guard, an interval that is *exactly* `spacing` accumulates float
    error to 0.050000000000000003, so ceil(length/spacing) == 2 and every 5 cm step is
    split into two 2.5 cm steps. On a per-line star this produced 328 spurious gaps.
    """

    def test_densify_line_is_idempotent(self):
        from path_engine.planners.straight_line import densify_line

        pts = densify_line((0.0, 0.0), (0.5, 0.0), 0.05)
        again = []
        for i in range(len(pts) - 1):
            chunk = densify_line(pts[i], pts[i + 1], 0.05)
            again.extend(chunk[1:] if again else chunk)
        assert len(again) == len(pts), (
            f"re-densifying split {len(pts)} points into {len(again)} — "
            "every interval was halved"
        )

    def test_densify_segment_is_idempotent(self):
        from path_engine.planners.straight_line import densify_segment

        seg = PathSegment(
            segment_type=SegmentType.TRANSIT,
            points=[(0.0, -0.5), (0.0, 0.0)],
            speed=0.5,
            source_entity="x:pre",
            metadata={"extension_role": "pre"},
        )
        once = densify_segment(seg, 0.05, 0.05)
        twice = densify_segment(once, 0.05, 0.05)
        assert len(twice.points) == len(once.points)
        gaps = [math.dist(twice.points[i], twice.points[i + 1])
                for i in range(len(twice.points) - 1)]
        assert all(abs(g - 0.05) < 1e-9 for g in gaps), f"non-uniform spacing: {set(gaps)}"

    def test_densify_still_honours_spacing(self):
        """The guard must not under-densify: every interval stays <= spacing."""
        from path_engine.planners.straight_line import densify_line

        for length in (0.05, 0.10, 0.12, 0.5, 2.0, 1.0 / 3):
            pts = densify_line((0.0, 0.0), (length, 0.0), 0.05)
            gaps = [math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
            assert max(gaps) <= 0.05 + 1e-9, f"length={length}: max gap {max(gaps)}"
            assert abs(pts[-1][0] - length) < 1e-12, "endpoint must be exact"


class TestSprayCompensationAtContiguousMarkJunctions:
    """Spray latency compensation belongs at REAL spray boundaries only.

    Where two MARK segments actually touch, the rover sprays straight through the
    junction — the spray never toggles there, so it is not a boundary. Compensating it
    anyway pulls spray OFF 3.5 mm *before* the junction and back ON 3.5 cm *after* it,
    and the planner then stitches a spray-off detour around the gap it just created:
    an unpainted notch in the middle of continuous marked geometry.

    (This is separate from per-line mode, where consecutive edges are deliberately
    separated by PRE/AFT and every edge end IS a real boundary — compensation must
    still fire there. See test_compensation_still_fires_at_real_boundaries.)
    """

    @staticmethod
    def _touching_lines() -> list[PathSegment]:
        # An L: two MARK segments sharing the vertex (2,0). Grouping is disabled below,
        # so they stay as two segments that touch — the case under test.
        return [
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[(0.0, 0.0), (2.0, 0.0)],
                speed=0.35,
                source_entity="LINE_0",
                metadata={"geometry_type": "LINE", "line_like": True},
            ),
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[(2.0, 0.0), (2.0, 2.0)],
                speed=0.35,
                source_entity="LINE_1",
                metadata={"geometry_type": "LINE", "line_like": True},
            ),
        ]

    def _plan(self):
        eng = PathEngine(
            enable_path_extensions=False,
            compensate_spray=True,
            optimize_order=False,
            group_shapes=False,      # keep them as two touching MARK segments
            mark_spacing=0.05,
            transit_spacing=0.15,
        )
        return eng.plan_segments(self._touching_lines())

    def test_spray_stays_on_across_a_touching_mark_junction(self):
        plan = self._plan()
        wps, flags = plan.merged_waypoints, plan.spray_flags
        near = [i for i, p in enumerate(wps) if math.hypot(p[0] - 2.0, p[1]) < 0.10]
        assert near, "shared vertex missing from the path"
        assert all(flags[i] for i in near), (
            "spray drops OFF within 10 cm of the shared vertex — unpainted notch in "
            "the middle of continuous marked geometry"
        )

    def test_no_detour_stitched_around_the_junction(self):
        assert not any(s.metadata.get("extension_connector")
                       for s in self._plan().segments)

    def test_compensation_still_fires_at_real_boundaries(self):
        """The fix must not disable compensation where it is genuinely needed.

        Per-line mode separates every edge with PRE/AFT, so all four of a square's edge
        ends are real OFF->ON / ON->OFF boundaries and must each be compensated.
        """
        eng = PathEngine(
            enable_path_extensions=True,
            pre_extension_m=0.5,
            aft_extension_m=0.5,
            per_line_extensions=True,
            compensate_spray=True,
            optimize_order=False,
            mark_spacing=0.05,
        )
        c = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
        square = [
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[c[i], c[i + 1]],
                speed=0.35,
                source_entity=f"LINE_{i}",
                metadata={"geometry_type": "LINE", "line_like": True},
            )
            for i in range(4)
        ]
        plan = eng.plan_segments(square)

        lead_in = 0.10 * 0.35    # 3.5 cm — solenoid open time
        lead_out = 0.01 * 0.35   # 3.5 mm — solenoid close time
        marks = [s for s in plan.segments
                 if s.segment_type == SegmentType.MARK
                 and not s.metadata.get("extension_role")]
        assert len(marks) == 4
        for m in marks:
            # Each 2.0 m edge is lengthened by the lead-in and shortened by the lead-out.
            length = sum(math.dist(m.points[i], m.points[i + 1])
                         for i in range(len(m.points) - 1))
            assert abs(length - (2.0 + lead_in - lead_out)) < 1e-3, (
                f"edge is {length:.4f} m — compensation not applied at this boundary"
            )


class TestExtensionSafetyLimits:
    """E1/E2/E5: an extension must not be longer than the line it serves, must not be
    laid down across geometry the rover paints, and must be sampled as tightly as a
    mark line.

    All three were found by planning the 14 sample DXFs and measuring the result:
    docs/upgrade_path/03_EXTENSION_GEOMETRY_DEFECTS.md
    """

    @staticmethod
    def _line(a, b, src="LINE_X") -> PathSegment:
        return PathSegment(
            segment_type=SegmentType.MARK,
            points=[a, b],
            speed=0.35,
            source_entity=src,
            metadata={"geometry_type": "LINE", "line_like": True},
        )

    # ── E2: proportion ──────────────────────────────────────────────────────

    def test_short_line_gets_no_extension(self):
        """A 10 cm mark cannot benefit from a 50 cm run-up.

        star_3x3m's two 10 cm crosshair lines were each given 0.465 m of PRE + 0.503 m
        of AFT — 7.4x the line they serve. The rover drove nearly a metre to paint a
        hand's width, twice.
        """
        eng = PathEngine(enable_path_extensions=True, pre_extension_m=0.5,
                         aft_extension_m=0.5, per_line_extensions=True,
                         compensate_spray=False, optimize_order=False,
                         extension_min_line_length_m=0.30)
        plan = eng.plan_segments([self._line((0.0, 0.0), (0.10, 0.0))])
        assert not any(s.metadata.get("extension_role") for s in plan.segments)

    def test_extension_capped_at_a_fraction_of_the_line(self):
        """A 0.6 m line with max_line_fraction=0.5 gets at most 0.3 m of run-up."""
        eng = PathEngine(enable_path_extensions=True, pre_extension_m=0.5,
                         aft_extension_m=0.5, per_line_extensions=True,
                         compensate_spray=False, optimize_order=False,
                         mark_spacing=0.05,
                         extension_max_line_fraction=0.5,
                         extension_min_line_length_m=0.30)
        plan = eng.plan_segments([self._line((0.0, 0.0), (0.6, 0.0))])
        for s in plan.segments:
            if s.metadata.get("extension_role") in ("pre", "aft"):
                length = sum(math.dist(s.points[i], s.points[i + 1])
                             for i in range(len(s.points) - 1))
                assert length <= 0.3 + 1e-6, f"extension {length:.3f} m exceeds 0.5x line"
                assert s.metadata.get("extension_clamped") is True

    def test_long_line_keeps_the_full_requested_extension(self):
        """The cap must not shorten extensions that were already reasonable."""
        eng = PathEngine(enable_path_extensions=True, pre_extension_m=0.5,
                         aft_extension_m=0.5, per_line_extensions=True,
                         compensate_spray=False, optimize_order=False,
                         mark_spacing=0.05)
        plan = eng.plan_segments([self._line((0.0, 0.0), (3.0, 0.0))])
        roles = [s for s in plan.segments
                 if s.metadata.get("extension_role") in ("pre", "aft")]
        assert len(roles) == 2
        for s in roles:
            length = sum(math.dist(s.points[i], s.points[i + 1])
                         for i in range(len(s.points) - 1))
            assert abs(length - 0.5) < 1e-6
            assert not s.metadata.get("extension_clamped")

    # ── E1: collision ───────────────────────────────────────────────────────

    def test_extension_stops_short_of_marked_geometry(self):
        """The run-out must not be driven across a line the rover paints.

        star_3x3m's tips touch the enclosing square, so six of its run-ups/run-outs were
        laid straight across the square's edges. The extension is now ray-cast against
        the rest of the drawing and shortened (or dropped) before it reaches one.
        """
        # A 2 m line pointing east, with a wall of paint 0.30 m past its end.
        line = self._line((0.0, 0.0), (0.0, 2.0), "LINE_A")
        wall = self._line((-1.0, 2.30), (1.0, 2.30), "LINE_WALL")

        eng = PathEngine(enable_path_extensions=True, pre_extension_m=0.5,
                         aft_extension_m=0.5, per_line_extensions=True,
                         compensate_spray=False, optimize_order=False,
                         mark_spacing=0.05, group_shapes=False,
                         extension_obstacle_clearance_m=0.10)
        plan = eng.plan_segments([line, wall])

        aft = next((s for s in plan.segments
                    if s.metadata.get("extension_role") == "aft"
                    and s.metadata.get("parent_source_entity") == "LINE_A"), None)
        if aft is not None:
            # Reaches at most (0.30 - 0.10 clearance) = 0.20 m, so it never touches 2.30.
            assert max(p[1] for p in aft.points) <= 2.30 - 0.10 + 1e-6, (
                "run-out crosses the wall of paint it was supposed to stop short of"
            )

    def test_extension_unobstructed_is_left_alone(self):
        """Nothing in the way → full requested length. No false clamping."""
        line = self._line((0.0, 0.0), (0.0, 2.0), "LINE_A")
        far = self._line((10.0, 0.0), (10.0, 2.0), "LINE_FAR")
        eng = PathEngine(enable_path_extensions=True, pre_extension_m=0.5,
                         aft_extension_m=0.5, per_line_extensions=True,
                         compensate_spray=False, optimize_order=False,
                         mark_spacing=0.05, group_shapes=False)
        plan = eng.plan_segments([line, far])
        aft = next(s for s in plan.segments
                   if s.metadata.get("extension_role") == "aft"
                   and s.metadata.get("parent_source_entity") == "LINE_A")
        length = sum(math.dist(aft.points[i], aft.points[i + 1])
                     for i in range(len(aft.points) - 1))
        assert abs(length - 0.5) < 1e-6

    # ── E5: connector sampling ──────────────────────────────────────────────

    def test_connectors_are_sampled_at_mark_spacing(self):
        """Inter-run connectors sit between two ~135 deg pivots — the hardest point on
        the path to track. They were left at transit_spacing (0.15 m), i.e. the sparsest
        sampling exactly where the rover most needs waypoints.
        """
        eng = PathEngine(enable_path_extensions=True, pre_extension_m=0.5,
                         aft_extension_m=0.5, per_line_extensions=True,
                         compensate_spray=False, optimize_order=True,
                         mark_spacing=0.05, transit_spacing=0.15)
        c = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
        square = [self._line(c[i], c[i + 1], f"LINE_{i}") for i in range(4)]
        plan = eng.plan_segments(square)

        conns = [s for s in plan.segments if s.metadata.get("extension_connector")]
        assert conns, "expected inter-run connectors on a per-line square"
        for s in conns:
            gaps = [math.dist(s.points[i], s.points[i + 1])
                    for i in range(len(s.points) - 1)]
            assert max(gaps) <= 0.05 + 1e-9, (
                f"connector sampled at {max(gaps) * 100:.1f} cm — should be mark_spacing"
            )

    # ── E4: reporting ───────────────────────────────────────────────────────

    def test_swept_footprint_is_reported(self):
        """A 2 m square with 0.5 m extensions sweeps a bigger area than it marks, and
        the operator must be told — on a bounded site that is an out-of-bounds excursion.
        """
        eng = PathEngine(enable_path_extensions=True, pre_extension_m=0.5,
                         aft_extension_m=0.5, per_line_extensions=True,
                         compensate_spray=False, optimize_order=True, mark_spacing=0.05)
        c = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
        plan = eng.plan_segments([self._line(c[i], c[i + 1], f"LINE_{i}")
                                  for i in range(4)])
        rep = plan.planning_metadata.get("extensions")
        assert rep is not None
        assert rep["max_overshoot_m"] > 0.2
        assert rep["swept_bbox"]["width_m"] > rep["marked_bbox"]["width_m"]

        from path_engine.validator import PathValidator
        warnings = PathValidator().validate(plan)
        assert any("beyond the marked geometry" in w for w in warnings)


class TestSprayBoundaryIsTheCadBoundary:
    """The plan must carry the TRUE geometry: spray ON where the CAD line starts, OFF
    where it ends. No planner-side lead.

    The planner used to shift the MARK boundary 3.5 cm early to compensate for solenoid
    open time. But spray_controller_node.py:276 ALREADY does that, at runtime, from the
    rover's ACTUAL speed:

        on_lead = speed_mps * solenoid_open_delay_s + on_overspray_margin_m
                = 0.35 * 0.10 + 0.02  =  5.5 cm at marking speed

    Doing it in both places meant paint started ~9 cm before the CAD line — double
    compensated. The controller is the right place for it: its lead stays correct when
    the profile slows into a corner, or when marking_speed changes. The planner's is a
    static shift that assumes the rover is always at marking_speed.
    """

    @staticmethod
    def _square() -> list[PathSegment]:
        c = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
        return [
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[c[i], c[i + 1]],
                speed=0.35,
                source_entity=f"LINE_{i}",
                metadata={"geometry_type": "LINE", "line_like": True},
            )
            for i in range(4)
        ]

    def test_compensation_is_off_by_default(self):
        assert PathEngine().compensate_spray is False

    def test_mark_length_equals_the_cad_line_exactly(self):
        eng = PathEngine(enable_path_extensions=True, per_line_extensions=True,
                         pre_extension_m=0.5, aft_extension_m=0.5,
                         optimize_order=False, mark_spacing=0.05)
        plan = eng.plan_segments(self._square())
        marks = [s for s in plan.segments if s.segment_type == SegmentType.MARK]
        assert len(marks) == 4
        for m in marks:
            length = sum(math.dist(m.points[i], m.points[i + 1])
                         for i in range(len(m.points) - 1))
            assert abs(length - 2.0) < 1e-9, (
                f"MARK is {length:.4f} m, not the CAD line's 2.000 m — the planner is "
                "still shifting the spray boundary"
            )

    def test_spray_toggles_exactly_on_the_cad_corners(self):
        eng = PathEngine(enable_path_extensions=True, per_line_extensions=True,
                         pre_extension_m=0.5, aft_extension_m=0.5,
                         optimize_order=True, mark_spacing=0.05)
        plan = eng.plan_segments(self._square())
        wps, flags = plan.merged_waypoints, plan.spray_flags
        corners = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
        for i in range(len(flags) - 1):
            if flags[i] == flags[i + 1]:
                continue
            p = wps[i + 1] if not flags[i] else wps[i]
            assert min(math.dist(p, c) for c in corners) < 1e-9, (
                f"spray toggles at {p}, which is not a CAD corner"
            )


class TestPaintAwareRouting:
    """E3: the transit between two runs must not drive over paint already on the ground.

    The connector is a straight shot from one run's run-out to the next run's run-up, and
    nothing stopped it crossing finished lines — the wheels go through wet paint.

    The fix is ORDERING, not rerouting, and it turns on one observation: crossing a line
    that has not been painted YET is free. star_3x3m's crosshairs sit inside both the
    star and the square, so some connector must cross that geometry — but if the
    crosshairs are marked first, there is no paint there to cross. Penalising only
    already-laid paint lets the optimizer find that ordering by itself.
    """

    @staticmethod
    def _crossings(plan) -> int:
        """Count spray-OFF segments that drive over a mark laid EARLIER in the run.

        Tested on the segments, not on the merged 5 cm polyline: a connector is a
        straight line, so its two endpoints describe it exactly, and a chunk-relative
        crossing test can miss a hit that lands on a chunk boundary.
        """
        def hit(p1, p2, p3, p4) -> bool:
            d = (p2[0] - p1[0]) * (p4[1] - p3[1]) - (p2[1] - p1[1]) * (p4[0] - p3[0])
            if abs(d) < 1e-12:
                return False
            t = ((p3[0] - p1[0]) * (p4[1] - p3[1]) - (p3[1] - p1[1]) * (p4[0] - p3[0])) / d
            u = ((p3[0] - p1[0]) * (p2[1] - p1[1]) - (p3[1] - p1[1]) * (p2[0] - p1[0])) / d
            return 1e-6 < t < 1 - 1e-6 and 1e-6 < u < 1 - 1e-6

        n = 0
        painted: list[PathSegment] = []
        for seg in plan.segments:
            if seg.segment_type == SegmentType.MARK:
                painted.append(seg)
                continue
            if len(seg.points) < 2 or not painted:
                continue
            a, b = seg.points[0], seg.points[-1]     # the move, end to end
            for m in painted:
                if any(hit(a, b, m.points[k], m.points[k + 1])
                       for k in range(len(m.points) - 1)):
                    n += 1
                    break
        return n

    @staticmethod
    def _enclosed_line_in_a_box() -> list[PathSegment]:
        """A short line inside a closed 3 m box. To mark the line the rover must enter
        the box; to mark the box it must leave. Ordering decides whether it crosses
        wet paint: mark the inner line FIRST and there is no paint to cross.
        """
        c = [(0.0, 0.0), (3.0, 0.0), (3.0, 3.0), (0.0, 3.0), (0.0, 0.0)]
        box = [
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[c[i], c[i + 1]],
                speed=0.35,
                source_entity=f"BOX_{i}",
                metadata={"geometry_type": "LINE", "line_like": True},
            )
            for i in range(4)
        ]
        inner = PathSegment(
            segment_type=SegmentType.MARK,
            points=[(1.2, 1.5), (1.8, 1.5)],
            speed=0.35,
            source_entity="LINE_INNER",
            metadata={"geometry_type": "LINE", "line_like": True},
        )
        return box + [inner]

    def _plan(self, avoid: bool):
        eng = PathEngine(
            enable_path_extensions=True, per_line_extensions=True,
            pre_extension_m=0.5, aft_extension_m=0.5,
            optimize_order=True, mark_spacing=0.05, transit_spacing=0.15,
            group_shapes=True, avoid_wet_paint=avoid,
        )
        return eng.plan_segments(self._enclosed_line_in_a_box())

    def test_enclosed_line_is_marked_before_the_shape_enclosing_it(self):
        plan = self._plan(avoid=True)
        order = [s.source_entity for s in plan.segments
                 if s.segment_type == SegmentType.MARK]
        inner = next(i for i, s in enumerate(order) if "INNER" in str(s))
        assert inner == 0, (
            f"the enclosed line is marked {inner + 1}th — it must go FIRST, or the "
            f"rover drives over the box to reach it. Order: {order}"
        )

    def test_no_transit_crosses_wet_paint(self):
        assert self._crossings(self._plan(avoid=True)) == 0

    def test_the_guard_is_what_prevents_it(self):
        """Without the penalty the same geometry DOES cross wet paint — proving the test
        is measuring the fix and not a coincidence of this shape."""
        assert self._crossings(self._plan(avoid=False)) > 0


class TestDuplicateGeometry:
    """CAD files contain lines drawn on top of each other.

    sct_1.5m.DXF has the same (0,0)->(1.5,0) LINE twice (handles 64 and 67) — drawn once,
    copied, never noticed. Left in, the planner faithfully paints it TWICE: mark it, run
    out past the end, reverse 180 deg, and mark it again backwards. Double-thick line,
    wasted time, and the reverse-flip the differential rover handles worst.
    """

    def test_coincident_line_is_dropped(self):
        line = PathSegment(
            segment_type=SegmentType.MARK, points=[(0.0, 0.0), (0.0, 2.0)], speed=0.35,
            source_entity="LINE_A", metadata={"geometry_type": "LINE", "line_like": True},
        )
        clone = PathSegment(
            segment_type=SegmentType.MARK, points=[(0.0, 0.0), (0.0, 2.0)], speed=0.35,
            source_entity="LINE_B", metadata={"geometry_type": "LINE", "line_like": True},
        )
        eng = PathEngine(optimize_order=False, mark_spacing=0.05, group_shapes=False)
        plan = eng.plan_segments([line, clone])
        marks = [s for s in plan.segments if s.segment_type == SegmentType.MARK]
        assert len(marks) == 1
        assert plan.planning_metadata["duplicate_geometry"]["removed"] == 1

    def test_reversed_duplicate_is_also_dropped(self):
        """The same line drawn backwards is still the same line."""
        line = PathSegment(
            segment_type=SegmentType.MARK, points=[(0.0, 0.0), (0.0, 2.0)], speed=0.35,
            source_entity="LINE_A", metadata={"geometry_type": "LINE", "line_like": True},
        )
        flipped = PathSegment(
            segment_type=SegmentType.MARK, points=[(0.0, 2.0), (0.0, 0.0)], speed=0.35,
            source_entity="LINE_B", metadata={"geometry_type": "LINE", "line_like": True},
        )
        eng = PathEngine(optimize_order=False, mark_spacing=0.05, group_shapes=False)
        plan = eng.plan_segments([line, flipped])
        assert sum(1 for s in plan.segments
                   if s.segment_type == SegmentType.MARK) == 1

    def test_distinct_lines_are_kept(self):
        a = PathSegment(
            segment_type=SegmentType.MARK, points=[(0.0, 0.0), (0.0, 2.0)], speed=0.35,
            source_entity="LINE_A", metadata={"geometry_type": "LINE", "line_like": True},
        )
        b = PathSegment(
            segment_type=SegmentType.MARK, points=[(5.0, 0.0), (5.0, 2.0)], speed=0.35,
            source_entity="LINE_B", metadata={"geometry_type": "LINE", "line_like": True},
        )
        eng = PathEngine(optimize_order=False, mark_spacing=0.05, group_shapes=False)
        plan = eng.plan_segments([a, b])
        assert sum(1 for s in plan.segments
                   if s.segment_type == SegmentType.MARK) == 2
        assert plan.planning_metadata["duplicate_geometry"]["removed"] == 0

    def test_operator_is_warned(self):
        line = PathSegment(
            segment_type=SegmentType.MARK, points=[(0.0, 0.0), (0.0, 2.0)], speed=0.35,
            source_entity="LINE_A", metadata={"geometry_type": "LINE", "line_like": True},
        )
        clone = PathSegment(
            segment_type=SegmentType.MARK, points=[(0.0, 0.0), (0.0, 2.0)], speed=0.35,
            source_entity="LINE_B", metadata={"geometry_type": "LINE", "line_like": True},
        )
        from path_engine.validator import PathValidator
        eng = PathEngine(optimize_order=False, mark_spacing=0.05, group_shapes=False)
        warnings = PathValidator().validate(eng.plan_segments([line, clone]))
        assert any("duplicate line" in w for w in warnings)

    # ── P1 (audit 2026-07-29): rounding buckets replaced by a distance test ──

    def _eng(self, **kw):
        kw.setdefault("optimize_order", False)
        kw.setdefault("mark_spacing", 0.05)
        kw.setdefault("group_shapes", False)
        return PathEngine(**kw)

    @staticmethod
    def _mark(pts, name):
        return PathSegment(
            segment_type=SegmentType.MARK, points=pts, speed=0.35,
            source_entity=name, metadata={"geometry_type": "LINE", "line_like": True},
        )

    def test_bucket_straddling_duplicate_is_dropped(self):
        """The old hash's false NEGATIVE: two coincident lines straddling a
        rounding-bucket edge hashed apart and BOTH got painted — double-thick
        line plus a 180° reversal, the failure the pass was written to stop.

        With tol=0.05, round(x/tol) flips between 0.0249 and 0.0251. The two
        lines here are 0.2 mm apart — physically the same stroke."""
        a = self._mark([(0.0249, 0.0), (0.0249, 2.0)], "LINE_A")
        b = self._mark([(0.0251, 0.0), (0.0251, 2.0)], "LINE_B")
        plan = self._eng().plan_segments([a, b])
        marks = [s for s in plan.segments if s.segment_type == SegmentType.MARK]
        assert len(marks) == 1, "coincident lines straddling the old bucket edge must dedup"
        assert plan.planning_metadata["duplicate_geometry"]["removed"] == 1

    def test_close_parallel_lines_are_both_kept(self):
        """The old hash's false POSITIVE: two DISTINCT parallel lines ~4 cm
        apart with equal length could land in one bucket and one was silently
        DELETED — data loss on tight road pre-line geometry (double centre
        lines, stop-bar hatching). 4 cm is intent, not a duplicate."""
        a = self._mark([(0.0, 0.0), (0.0, 2.0)], "LINE_A")
        b = self._mark([(0.04, 0.0), (0.04, 2.0)], "LINE_B")
        plan = self._eng().plan_segments([a, b])
        marks = [s for s in plan.segments if s.segment_type == SegmentType.MARK]
        assert len(marks) == 2, "distinct 4 cm-apart parallel lines must BOTH survive"
        assert plan.planning_metadata["duplicate_geometry"]["removed"] == 0

    def test_mirrored_arc_same_endpoints_is_kept(self):
        """Same endpoints, same length, opposite bulge — different geometry.
        The endpoint+length signature alone (old and new) cannot tell these
        apart; the arc-length-midpoint term must."""
        import math as _m
        n = 16
        up = [( _m.sin(_m.pi * i / n), 1.0 - _m.cos(_m.pi * i / n)) for i in range(n + 1)]
        down = [( _m.sin(_m.pi * i / n), -(1.0 - _m.cos(_m.pi * i / n))) for i in range(n + 1)]
        plan = self._eng().plan_segments(
            [self._mark(up, "ARC_UP"), self._mark(down, "ARC_DOWN")]
        )
        marks = [s for s in plan.segments if s.segment_type == SegmentType.MARK]
        assert len(marks) == 2, "mirrored arcs are distinct geometry, not duplicates"

    def test_allow_duplicate_drop_false_refuses(self):
        a = self._mark([(0.0, 0.0), (0.0, 2.0)], "LINE_A")
        b = self._mark([(0.0, 0.0), (0.0, 2.0)], "LINE_B")
        eng = self._eng(allow_duplicate_drop=False)
        with pytest.raises(ValueError, match="LINE_A"):
            eng.plan_segments([a, b])

    def test_allow_duplicate_drop_false_passes_clean_geometry(self):
        a = self._mark([(0.0, 0.0), (0.0, 2.0)], "LINE_A")
        b = self._mark([(5.0, 0.0), (5.0, 2.0)], "LINE_B")
        plan = self._eng(allow_duplicate_drop=False).plan_segments([a, b])
        assert plan.planning_metadata["duplicate_geometry"]["removed"] == 0


class TestRetraceSuppression:
    """A run-out must never be followed by a connector that drives straight back over it.

    Two runs meeting end-to-start is normally fine — a square's corners meet that way and
    the extensions leave PERPENDICULAR to each other, giving a wide turn. But when the two
    runs are COLLINEAR at the junction, the run-out leaves along the very line the run-in
    approaches on, and the connector can only double back:

        sct_1.5m: edge1 ends at (1.5, 3.0); CIRCLE_63 starts there, tangentially
          AFT  (1.5,3.0) -> (1.0,3.0)
          CONN (1.0,3.0) -> (2.0,3.0)    retraces it
          PRE  (2.0,3.0) -> (1.5,3.0)    retraces again

    The test is on DIRECTION, not on a shared point: perpendicular -> keep, collinear -> drop.
    """

    def test_collinear_junction_drops_the_extensions(self):
        """Two collinear segments end-to-end: no run-out/run-in between them."""
        a = PathSegment(
            segment_type=SegmentType.MARK, points=[(0.0, 0.0), (0.0, 2.0)], speed=0.35,
            source_entity="LINE_A", metadata={"geometry_type": "LINE", "line_like": True},
        )
        b = PathSegment(
            segment_type=SegmentType.MARK, points=[(0.0, 2.0), (0.0, 4.0)], speed=0.35,
            source_entity="LINE_B", metadata={"geometry_type": "LINE", "line_like": True},
        )
        eng = PathEngine(enable_path_extensions=True, per_line_extensions=True,
                         pre_extension_m=0.5, aft_extension_m=0.5,
                         optimize_order=False, mark_spacing=0.05, group_shapes=False)
        plan = eng.plan_segments([a, b])
        roles = [s.metadata.get("extension_role") for s in plan.segments]
        assert roles.count("pre") == 1     # only the very start of the run
        assert roles.count("aft") == 1     # only the very end
        wps = plan.merged_waypoints
        for i in range(1, len(wps) - 1):
            ax, ay = wps[i][0] - wps[i - 1][0], wps[i][1] - wps[i - 1][1]
            bx, by = wps[i + 1][0] - wps[i][0], wps[i + 1][1] - wps[i][1]
            la, lb = math.hypot(ax, ay), math.hypot(bx, by)
            if la < 1e-9 or lb < 1e-9:
                continue
            dot = max(-1.0, min(1.0, (ax * bx + ay * by) / (la * lb)))
            assert math.degrees(math.acos(dot)) < 170.0, "the rover retraces itself"

    def test_perpendicular_corner_keeps_per_line(self):
        """A square's corner is NOT a retrace — every edge must keep its own pass.

        This is the guard against 'fixing' retraces by quietly deleting per-line, which
        is what a naive shared-endpoint test does.
        """
        c = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
        square = [
            PathSegment(
                segment_type=SegmentType.MARK, points=[c[i], c[i + 1]], speed=0.35,
                source_entity=f"LINE_{i}",
                metadata={"geometry_type": "LINE", "line_like": True},
            )
            for i in range(4)
        ]
        eng = PathEngine(enable_path_extensions=True, per_line_extensions=True,
                         pre_extension_m=0.5, aft_extension_m=0.5,
                         optimize_order=True, mark_spacing=0.05)
        plan = eng.plan_segments(square)
        roles = [s.metadata.get("extension_role") for s in plan.segments]
        marks = [s for s in plan.segments if s.segment_type == SegmentType.MARK]
        assert (roles.count("pre"), len(marks), roles.count("aft")) == (4, 4, 4), (
            "per-line was silently disabled at the square's corners"
        )
