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
        # All spray flags must be True (pure MARK, no TRANSIT extensions)
        assert all(f is True for f in plan.spray_flags)

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
