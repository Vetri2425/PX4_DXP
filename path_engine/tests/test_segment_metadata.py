"""Tests for runtime segment order metadata in plan_path() response.

Covers every requirement from the task spec:
1. runtime_segment_index is 0-based and sequential.
2. runtime_sequence is 1-based and sequential.
3. MARK extension emits pre_transit → mark → aft_transit in that sequence.
4. parent_entity_id links :pre and :aft back to the parent handle.
5. order_source = "saved_entity_order" when saved sidecar exists.
6. order_source = "optimizer" when no saved order and optimizer enabled.
7. order_source = "parser_order" when no saved order and optimizer disabled.
8. is_extension = True only for pre/aft extension TRANSITs.
9. Backward-compatible fields (type, speed, source, length_m) still present.
10. Builtin path branch also emits the new fields.
"""

from __future__ import annotations

import os
import sys

import pytest

# Make server/ importable
sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "server"))
)

from path_engine.core import DXFEntity, PathSegment, SegmentType
from path_engine.entity_order import apply_entity_order


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_line_entity(entity_id: str, north: float = 0.0) -> DXFEntity:
    return DXFEntity(
        entity_type="LINE",
        layer="0",
        entity_id=entity_id,
        geometry={"start": (north, 0.0), "end": (north + 1.0, 0.0)},
    )


# ── Helper unit tests ─────────────────────────────────────────────────────────

class TestSegmentHelpers:
    """Unit-test the three static helper methods directly."""

    @pytest.fixture()
    def path_mgr(self, tmp_path):
        from path_manager import PathManager
        return PathManager(str(tmp_path / "missions"))

    def _mark_seg(self, source="LINE_1A3", meta=None):
        return PathSegment(
            segment_type=SegmentType.MARK,
            points=[(0.0, 0.0), (1.0, 0.0)],
            source_entity=source,
            metadata=meta or {},
        )

    def _transit_seg(self, source="transit:1", meta=None):
        return PathSegment(
            segment_type=SegmentType.TRANSIT,
            points=[(1.0, 0.0), (2.0, 0.0)],
            source_entity=source,
            metadata=meta or {},
        )

    # _segment_role
    def test_role_mark(self, path_mgr):
        assert path_mgr._segment_role(self._mark_seg()) == "mark"

    def test_role_transit(self, path_mgr):
        assert path_mgr._segment_role(self._transit_seg()) == "transit"

    def test_role_pre_transit(self, path_mgr):
        seg = self._transit_seg("LINE_1A3:pre", meta={"extension_role": "pre"})
        assert path_mgr._segment_role(seg) == "pre_transit"

    def test_role_aft_transit(self, path_mgr):
        seg = self._transit_seg("LINE_1A3:aft", meta={"extension_role": "aft"})
        assert path_mgr._segment_role(seg) == "aft_transit"

    # _parent_source_entity
    def test_parent_source_plain_mark(self, path_mgr):
        assert path_mgr._parent_source_entity(self._mark_seg("LINE_1A3")) == "LINE_1A3"

    def test_parent_source_pre(self, path_mgr):
        seg = self._transit_seg(
            "LINE_1A3:pre",
            meta={"extension_role": "pre", "parent_source_entity": "LINE_1A3"},
        )
        assert path_mgr._parent_source_entity(seg) == "LINE_1A3"

    def test_parent_source_aft(self, path_mgr):
        seg = self._transit_seg(
            "LINE_1A3:aft",
            meta={"extension_role": "aft", "parent_source_entity": "LINE_1A3"},
        )
        assert path_mgr._parent_source_entity(seg) == "LINE_1A3"

    def test_parent_source_no_metadata_strips_suffix(self, path_mgr):
        # No parent_source_entity in metadata → strip :pre suffix from source
        seg = self._transit_seg("ARC_2B4:pre", meta={"extension_role": "pre"})
        assert path_mgr._parent_source_entity(seg) == "ARC_2B4"

    def test_parent_source_synthetic_transit(self, path_mgr):
        seg = self._transit_seg("transit:3")
        assert path_mgr._parent_source_entity(seg) == "transit"

    # _extract_entity_id
    def test_extract_id_line(self, path_mgr):
        assert path_mgr._extract_entity_id("LINE_1A3") == "1A3"

    def test_extract_id_arc(self, path_mgr):
        assert path_mgr._extract_entity_id("ARC_2B4") == "2B4"

    def test_extract_id_circle(self, path_mgr):
        assert path_mgr._extract_entity_id("CIRCLE_3C5") == "3C5"

    def test_extract_id_lwpolyline(self, path_mgr):
        assert path_mgr._extract_entity_id("LWPOLYLINE_4D6") == "4D6"

    def test_extract_id_transit_returns_none(self, path_mgr):
        assert path_mgr._extract_entity_id("transit:1") is None

    def test_extract_id_no_underscore_returns_none(self, path_mgr):
        # Source with no underscore at all → None
        assert path_mgr._extract_entity_id("UNKNOWNHANDLE") is None

    def test_extract_id_builtin_returns_none(self, path_mgr):
        # Colon-containing synthetics → None (builtin, transit, group)
        assert path_mgr._extract_entity_id("builtin:square_2x2") is None
        assert path_mgr._extract_entity_id("transit:start") is None
        assert path_mgr._extract_entity_id("group:LINE_1A3+LINE_2B4") is None


# ── PathManager integration tests ─────────────────────────────────────────────

class TestPlanPathSegmentMetadata:
    """End-to-end: plan_path() result segments[] carry all required metadata."""

    @pytest.fixture()
    def missions_dir(self, tmp_path):
        return str(tmp_path / "missions")

    @pytest.fixture()
    def path_mgr(self, missions_dir):
        from path_manager import PathManager
        return PathManager(missions_dir)

    @pytest.fixture()
    def dxf_two_lines(self, missions_dir):
        """Two LINE entities in a DXF: A at north=0, B at north=10."""
        pytest.importorskip("ezdxf")
        import ezdxf
        os.makedirs(missions_dir, exist_ok=True)
        doc = ezdxf.new("R2010")
        doc.header["$INSUNITS"] = 6  # metres
        msp = doc.modelspace()
        msp.add_line((0, 0, 0), (0, 1, 0), dxfattribs={"layer": "0"})   # entity A
        msp.add_line((0, 10, 0), (0, 11, 0), dxfattribs={"layer": "0"}) # entity B
        fpath = os.path.join(missions_dir, "two_lines.dxf")
        doc.saveas(fpath)
        return "two_lines.dxf"

    def _ids(self, path_mgr, missions_dir, filename):
        """Parser-order entity IDs."""
        fpath = os.path.join(missions_dir, filename)
        return [e.entity_id for e in path_mgr.parse_dxf(fpath)]

    # ── 1. runtime_segment_index is 0-based and sequential ────────────────────

    def test_runtime_segment_index_sequential(self, path_mgr, dxf_two_lines):
        result = path_mgr.plan_path(dxf_two_lines, optimize=True)
        segs = result["segments"]
        assert len(segs) >= 1
        for expected_idx, seg in enumerate(segs):
            assert seg["runtime_segment_index"] == expected_idx, (
                f"seg[{expected_idx}] has runtime_segment_index={seg['runtime_segment_index']}"
            )

    # ── 2. runtime_sequence is 1-based and sequential ─────────────────────────

    def test_runtime_sequence_sequential(self, path_mgr, dxf_two_lines):
        result = path_mgr.plan_path(dxf_two_lines, optimize=True)
        segs = result["segments"]
        for expected_seq, seg in enumerate(segs, start=1):
            assert seg["runtime_sequence"] == expected_seq, (
                f"seg[{expected_seq-1}] has runtime_sequence={seg['runtime_sequence']}"
            )

    def test_runtime_sequence_equals_index_plus_one(self, path_mgr, dxf_two_lines):
        result = path_mgr.plan_path(dxf_two_lines, optimize=True)
        for seg in result["segments"]:
            assert seg["runtime_sequence"] == seg["runtime_segment_index"] + 1

    # ── 3. Extension triplet: pre_transit → mark → aft_transit ────────────────

    def test_extension_triplet_roles(self, path_mgr, missions_dir, dxf_two_lines):
        # Enable extensions via the sidecar
        path_mgr.save_extension_config("two_lines.dxf", True, 0.3, 0.3)
        result = path_mgr.plan_path(dxf_two_lines, optimize=False)
        segs = result["segments"]
        roles = [s["segment_role"] for s in segs]
        # Each MARK entity expands to pre_transit + mark + aft_transit
        assert "pre_transit" in roles
        assert "mark" in roles
        assert "aft_transit" in roles

    def test_extension_triplet_order(self, path_mgr, missions_dir, dxf_two_lines):
        """pre_transit always comes immediately before mark, aft after."""
        path_mgr.save_extension_config("two_lines.dxf", True, 0.3, 0.3)
        result = path_mgr.plan_path(dxf_two_lines, optimize=False)
        segs = result["segments"]
        roles = [s["segment_role"] for s in segs]
        # Find first mark index
        mark_indices = [i for i, r in enumerate(roles) if r == "mark"]
        assert len(mark_indices) >= 1
        for mi in mark_indices:
            if mi > 0:
                assert roles[mi - 1] in ("pre_transit", "transit"), (
                    f"Expected pre_transit or transit before mark at {mi}, got {roles[mi-1]}"
                )
            if mi < len(roles) - 1:
                assert roles[mi + 1] in ("aft_transit", "transit"), (
                    f"Expected aft_transit or transit after mark at {mi}, got {roles[mi+1]}"
                )

    def test_extension_sequences_are_consecutive(self, path_mgr, missions_dir, dxf_two_lines):
        """pre_transit, mark, aft_transit for same entity have consecutive sequence numbers."""
        path_mgr.save_extension_config("two_lines.dxf", True, 0.3, 0.3)
        result = path_mgr.plan_path(dxf_two_lines, optimize=False)
        segs = result["segments"]
        # Group by parent_entity_id and check the extension triplet is adjacent
        from collections import defaultdict
        by_parent: dict[str, list] = defaultdict(list)
        for seg in segs:
            pid = seg.get("parent_entity_id")
            if pid:
                by_parent[pid].append(seg)
        for pid, group in by_parent.items():
            seqs = [s["runtime_sequence"] for s in group]
            # Sequences must be consecutive integers
            seqs_sorted = sorted(seqs)
            expected = list(range(seqs_sorted[0], seqs_sorted[0] + len(seqs_sorted)))
            assert seqs_sorted == expected, (
                f"Entity {pid} segments not consecutive: {seqs}"
            )

    # ── 4. parent_entity_id links :pre and :aft back to parent handle ──────────

    def test_pre_parent_entity_id_matches_mark(self, path_mgr, missions_dir, dxf_two_lines):
        path_mgr.save_extension_config("two_lines.dxf", True, 0.3, 0.3)
        result = path_mgr.plan_path(dxf_two_lines, optimize=False)
        segs = result["segments"]
        pre_segs = [s for s in segs if s["segment_role"] == "pre_transit"]
        mark_segs = [s for s in segs if s["segment_role"] == "mark"]
        assert len(pre_segs) > 0
        assert len(mark_segs) > 0
        # Every pre_transit must share parent_entity_id with a mark
        mark_parent_ids = {s["parent_entity_id"] for s in mark_segs}
        for pre in pre_segs:
            assert pre["parent_entity_id"] in mark_parent_ids, (
                f"pre_transit parent_entity_id {pre['parent_entity_id']!r} "
                f"not found in mark parents {mark_parent_ids}"
            )

    def test_aft_parent_entity_id_matches_mark(self, path_mgr, missions_dir, dxf_two_lines):
        path_mgr.save_extension_config("two_lines.dxf", True, 0.3, 0.3)
        result = path_mgr.plan_path(dxf_two_lines, optimize=False)
        segs = result["segments"]
        aft_segs = [s for s in segs if s["segment_role"] == "aft_transit"]
        mark_segs = [s for s in segs if s["segment_role"] == "mark"]
        mark_parent_ids = {s["parent_entity_id"] for s in mark_segs}
        for aft in aft_segs:
            assert aft["parent_entity_id"] in mark_parent_ids

    def test_plain_mark_parent_entity_id_not_none(self, path_mgr, dxf_two_lines):
        """A plain MARK segment has a non-null parent_entity_id."""
        result = path_mgr.plan_path(dxf_two_lines, optimize=False)
        mark_segs = [s for s in result["segments"] if s["segment_role"] == "mark"]
        assert len(mark_segs) > 0
        for seg in mark_segs:
            assert seg["parent_entity_id"] is not None

    def test_optimizer_transit_parent_entity_id_is_none(self, path_mgr, dxf_two_lines):
        """Optimizer-inserted TRANSITs (transit:N) have parent_entity_id=None."""
        result = path_mgr.plan_path(dxf_two_lines, optimize=True)
        transit_segs = [s for s in result["segments"] if s["segment_role"] == "transit"]
        # At least one inter-entity transit should exist between 2 MARK segments
        for seg in transit_segs:
            src = seg["source"]
            if src.startswith("transit:"):
                assert seg["parent_entity_id"] is None, (
                    f"Synthetic transit source {src!r} should have null parent_entity_id"
                )

    # ── 5/6/7. order_source ───────────────────────────────────────────────────

    def test_order_source_saved_entity_order(
        self, path_mgr, missions_dir, dxf_two_lines
    ):
        """Saved sidecar → every segment carries order_source='saved_entity_order'."""
        ids = self._ids(path_mgr, missions_dir, dxf_two_lines)
        path_mgr.save_entity_order("two_lines.dxf", list(reversed(ids)))
        result = path_mgr.plan_path("two_lines.dxf", optimize=True)
        for seg in result["segments"]:
            assert seg["order_source"] == "saved_entity_order", (
                f"Expected saved_entity_order, got {seg['order_source']!r} "
                f"for segment {seg['source']!r}"
            )

    def test_order_source_optimizer(self, path_mgr, dxf_two_lines):
        """No saved order + optimize=True → order_source='optimizer'."""
        result = path_mgr.plan_path(dxf_two_lines, optimize=True)
        for seg in result["segments"]:
            assert seg["order_source"] == "optimizer", (
                f"Expected optimizer, got {seg['order_source']!r}"
            )

    def test_order_source_parser_order(self, path_mgr, dxf_two_lines):
        """No saved order + optimize=False → order_source='parser_order'."""
        result = path_mgr.plan_path(dxf_two_lines, optimize=False)
        for seg in result["segments"]:
            assert seg["order_source"] == "parser_order", (
                f"Expected parser_order, got {seg['order_source']!r}"
            )

    # ── 8. is_extension ───────────────────────────────────────────────────────

    def test_is_extension_false_for_mark(self, path_mgr, dxf_two_lines):
        result = path_mgr.plan_path(dxf_two_lines, optimize=False)
        for seg in result["segments"]:
            if seg["segment_role"] == "mark":
                assert seg["is_extension"] is False

    def test_is_extension_false_for_optimizer_transit(self, path_mgr, dxf_two_lines):
        result = path_mgr.plan_path(dxf_two_lines, optimize=True)
        for seg in result["segments"]:
            if seg["segment_role"] == "transit":
                assert seg["is_extension"] is False

    def test_is_extension_true_for_pre_aft(
        self, path_mgr, missions_dir, dxf_two_lines
    ):
        path_mgr.save_extension_config("two_lines.dxf", True, 0.3, 0.3)
        result = path_mgr.plan_path(dxf_two_lines, optimize=False)
        for seg in result["segments"]:
            if seg["segment_role"] in ("pre_transit", "aft_transit"):
                assert seg["is_extension"] is True, (
                    f"Expected is_extension=True for {seg['segment_role']}"
                )

    # ── 9. Backward compatibility ─────────────────────────────────────────────

    def test_backward_compat_fields_present(self, path_mgr, dxf_two_lines):
        """Legacy fields type, speed, source, length_m must still be present."""
        result = path_mgr.plan_path(dxf_two_lines, optimize=True)
        for seg in result["segments"]:
            assert "type" in seg
            assert "speed" in seg
            assert "source" in seg
            assert "length_m" in seg

    def test_type_field_values(self, path_mgr, dxf_two_lines):
        result = path_mgr.plan_path(dxf_two_lines, optimize=True)
        for seg in result["segments"]:
            assert seg["type"] in ("MARK", "TRANSIT")

    def test_speed_is_positive(self, path_mgr, dxf_two_lines):
        result = path_mgr.plan_path(dxf_two_lines, optimize=True)
        for seg in result["segments"]:
            assert seg["speed"] > 0

    def test_length_m_non_negative(self, path_mgr, dxf_two_lines):
        result = path_mgr.plan_path(dxf_two_lines, optimize=True)
        for seg in result["segments"]:
            assert seg["length_m"] >= 0.0


# ── Builtin path branch ───────────────────────────────────────────────────────

class TestBuiltinSegmentMetadata:
    """Builtin path branch of plan_path() also emits the new metadata fields."""

    @pytest.fixture()
    def path_mgr(self, tmp_path):
        from path_manager import PathManager
        return PathManager(str(tmp_path / "missions"))

    def test_builtin_new_fields_present(self, path_mgr):
        result = path_mgr.plan_path("straight_5m")
        assert len(result["segments"]) == 1
        seg = result["segments"][0]
        assert seg["runtime_segment_index"] == 0
        assert seg["runtime_sequence"] == 1
        assert seg["segment_role"] == "mark"
        assert seg["is_extension"] is False
        assert seg["order_source"] == "parser_order"
        assert seg["parent_entity_id"] is None

    def test_builtin_backward_compat(self, path_mgr):
        result = path_mgr.plan_path("square_2x2")
        seg = result["segments"][0]
        assert seg["type"] == "MARK"
        assert "speed" in seg
        assert "source" in seg
        assert "length_m" in seg
