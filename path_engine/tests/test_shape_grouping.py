"""Tests for shape grouping (connected MARK primitive chaining)."""

import math

from path_engine.core import PathSegment, SegmentType
from path_engine.optimizers.shape_grouping import group_connected_segments


def _line(a, b, source="LINE", **meta):
    return PathSegment(
        segment_type=SegmentType.MARK,
        points=[a, b],
        source_entity=source,
        metadata=dict(meta),
    )


def test_disconnected_lines_pass_through():
    """Two lines that share no endpoint stay as two separate segments."""
    segs = [_line((0, 0), (1, 0), "A"), _line((5, 5), (6, 5), "B")]
    out = group_connected_segments(segs)
    assert len(out) == 2
    assert out[0].source_entity == "A"
    assert out[1].source_entity == "B"


def test_single_segment_noop():
    """A lone chainable segment is returned untouched."""
    segs = [_line((0, 0), (1, 0), "A")]
    out = group_connected_segments(segs)
    assert out == segs


def test_open_chain_merged_in_order():
    """Three connected lines chain end-to-end into one composite run."""
    segs = [
        _line((0, 0), (1, 0), "A"),
        _line((1, 0), (2, 0), "B"),
        _line((2, 0), (3, 0), "C"),
    ]
    out = group_connected_segments(segs)
    assert len(out) == 1
    comp = out[0]
    assert comp.segment_type == SegmentType.MARK
    assert comp.points[0] == (0, 0)
    assert comp.points[-1] == (3, 0)
    assert comp.metadata["grouped_from"] == ["A", "B", "C"]


def test_chain_reverses_segment_to_connect():
    """A reversed primitive is flipped so the chain stays continuous."""
    segs = [
        _line((0, 0), (1, 0), "A"),
        _line((2, 0), (1, 0), "B"),  # stored end-first
    ]
    out = group_connected_segments(segs)
    assert len(out) == 1
    assert out[0].points[0] == (0, 0)
    assert out[0].points[-1] == (2, 0)


def test_closed_square_chains_to_one_loop():
    """Four edges of a square chain into a single closed run."""
    segs = [
        _line((0, 0), (1, 0), "A"),
        _line((1, 0), (1, 1), "B"),
        _line((1, 1), (0, 1), "C"),
        _line((0, 1), (0, 0), "D"),
    ]
    out = group_connected_segments(segs)
    assert len(out) == 1
    comp = out[0]
    # First ≈ last → closed loop preserved for the controller's closed guard.
    d = math.hypot(comp.points[0][0] - comp.points[-1][0],
                   comp.points[0][1] - comp.points[-1][1])
    assert d < 1e-6
    assert len(comp.metadata["grouped_from"]) == 4


def test_curved_mark_not_absorbed():
    """A circle segment is never merged into an adjacent line chain."""
    circle = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0, 0), (1, 1), (2, 0), (1, -1), (0, 0)],
        source_entity="CIRCLE_1",
        metadata={"geometry_type": "CIRCLE"},
    )
    line = _line((0, 0), (-1, 0), "A")
    out = group_connected_segments([circle, line])
    assert circle in out
    assert line in out


def test_transit_not_grouped():
    """TRANSIT segments are left alone even if endpoints touch."""
    t = PathSegment(
        segment_type=SegmentType.TRANSIT,
        points=[(0, 0), (1, 0)],
        source_entity="T",
    )
    line = _line((1, 0), (2, 0), "A")
    out = group_connected_segments([t, line])
    assert t in out
    # The lone line stays a single segment (nothing else chainable).
    assert any(s.source_entity == "A" for s in out)


def test_two_shapes_sharing_edge_stay_continuous():
    """Triangle + square sharing one edge → continuous chains, no interleave."""
    # Shared edge is the vertical segment from (0,0) to (0,1).
    triangle = [
        _line((0, 0), (0, 1), "SHARED"),   # shared edge
        _line((0, 1), (-1, 0.5), "T1"),
        _line((-1, 0.5), (0, 0), "T2"),
    ]
    square = [
        _line((0, 0), (1, 0), "S1"),
        _line((1, 0), (1, 1), "S2"),
        _line((1, 1), (0, 1), "S3"),
    ]
    out = group_connected_segments(triangle + square)
    # All six edges form one connected component (joined at the shared edge),
    # so they collapse into chain(s) — far fewer than 6 loose primitives — and
    # every emitted run is geometrically continuous.
    assert len(out) < 6
    for seg in out:
        for i in range(1, len(seg.points)):
            # No teleport jumps within a chain.
            d = math.hypot(seg.points[i][0] - seg.points[i - 1][0],
                           seg.points[i][1] - seg.points[i - 1][1])
            assert d < 2.0
    # No primitive is lost: total distinct sources preserved.
    sources = set()
    for seg in out:
        sources.update(seg.metadata.get("grouped_from", [seg.source_entity]))
    assert sources == {"SHARED", "T1", "T2", "S1", "S2", "S3"}


def test_input_order_preserved_for_mixed():
    """A transit between two line groups keeps its relative position."""
    segs = [
        _line((0, 0), (1, 0), "A"),
        _line((1, 0), (2, 0), "B"),
        PathSegment(segment_type=SegmentType.TRANSIT,
                    points=[(2, 0), (5, 0)], source_entity="T"),
        _line((5, 0), (6, 0), "C"),
    ]
    out = group_connected_segments(segs)
    types = [s.segment_type for s in out]
    # group(A+B), TRANSIT, C
    assert types == [SegmentType.MARK, SegmentType.TRANSIT, SegmentType.MARK]
