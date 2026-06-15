"""Tests for entity order integration.

Covers:
- apply_entity_order() shared helper (saved_order vs parser order)
- Saved entity order affects GET /entities preview order_index
- Saved entity order affects plan output segment order
- Optimizer is skipped when saved order exists
- Optimizer still runs when no saved order exists
- New DXF entities not in sidecar are appended at end
- Removed entity IDs in sidecar are silently ignored
"""

from __future__ import annotations

import json
import os
import tempfile

import pytest

from path_engine.core import DXFEntity, PathSegment, SegmentType
from path_engine.entity_order import apply_entity_order
from path_engine.engine import PathEngine


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_entity(entity_id: str, north_start: float = 0.0) -> DXFEntity:
    """Create a minimal LINE DXFEntity at a given north offset."""
    return DXFEntity(
        entity_type="LINE",
        layer="0",
        entity_id=entity_id,
        geometry={
            "start": (north_start, 0.0),
            "end": (north_start + 1.0, 0.0),
        },
    )


def _entities(*ids_and_north) -> list[DXFEntity]:
    """Build entities from (id, north_start) pairs."""
    return [_make_entity(eid, n) for eid, n in ids_and_north]


# ── Unit tests: apply_entity_order() ─────────────────────────────────────────

class TestApplyEntityOrder:
    """Unit tests for the shared apply_entity_order() helper."""

    def test_empty_saved_order_returns_parser_order(self):
        entities = _entities(("A", 0.0), ("B", 5.0), ("C", 10.0))
        result = apply_entity_order(entities, [])
        assert [e.entity_id for e in result] == ["A", "B", "C"]

    def test_saved_order_reorders_entities(self):
        entities = _entities(("A", 0.0), ("B", 5.0), ("C", 10.0))
        result = apply_entity_order(entities, ["C", "A", "B"])
        assert [e.entity_id for e in result] == ["C", "A", "B"]

    def test_new_entities_not_in_sidecar_appended_at_end(self):
        """Entity D exists in DXF but not in saved_order → appended last."""
        entities = _entities(("A", 0.0), ("B", 5.0), ("D", 20.0))
        result = apply_entity_order(entities, ["B", "A"])
        assert [e.entity_id for e in result] == ["B", "A", "D"]

    def test_removed_sidecar_ids_silently_ignored(self):
        """Entity X is in saved_order but no longer in DXF → ignored."""
        entities = _entities(("A", 0.0), ("B", 5.0))
        result = apply_entity_order(entities, ["X", "B", "A"])
        assert [e.entity_id for e in result] == ["B", "A"]

    def test_no_duplicates_emitted(self):
        """Even with a malformed sidecar, each entity appears exactly once."""
        entities = _entities(("A", 0.0), ("B", 5.0))
        # saved_order has A listed twice — second occurrence must be dropped
        result = apply_entity_order(entities, ["A", "A", "B"])
        assert [e.entity_id for e in result] == ["A", "B"]

    def test_partial_order_appends_remainder(self):
        """Sidecar covers only some entities; rest are appended in parser order."""
        entities = _entities(("A", 0.0), ("B", 5.0), ("C", 10.0), ("D", 15.0))
        result = apply_entity_order(entities, ["C", "A"])
        assert [e.entity_id for e in result] == ["C", "A", "B", "D"]

    def test_returns_new_list_does_not_mutate_input(self):
        entities = _entities(("A", 0.0), ("B", 5.0))
        original_ids = [e.entity_id for e in entities]
        result = apply_entity_order(entities, ["B", "A"])
        assert [e.entity_id for e in entities] == original_ids  # unchanged
        assert result is not entities  # different list object

    def test_all_entities_missing_from_dxf_returns_empty(self):
        """All saved IDs stale — no entities to append, result is empty."""
        entities = []
        result = apply_entity_order(entities, ["X", "Y"])
        assert result == []

    def test_single_entity_no_reorder(self):
        entities = _entities(("A", 0.0))
        result = apply_entity_order(entities, ["A"])
        assert [e.entity_id for e in result] == ["A"]


# ── Integration: saved order affects plan output order ─────────────────────────

class TestEntityOrderInPlanning:
    """Saved entity order must survive the full parse → plan pipeline."""

    def _make_two_line_entities(self) -> tuple[DXFEntity, DXFEntity]:
        """Two LINE entities at distinct north positions."""
        ent_a = _make_entity("A", north_start=0.0)   # LINE from (0,0)→(1,0)
        ent_b = _make_entity("B", north_start=10.0)  # LINE from (10,0)→(11,0)
        return ent_a, ent_b

    def test_parser_order_optimizer_can_change_order(self):
        """Without saved order, optimizer is free to reorder."""
        ent_a, ent_b = self._make_two_line_entities()
        engine = PathEngine(optimize_order=True, compensate_spray=False)
        # Start near B → optimizer should visit B first
        plan = engine.plan_dxf_entities(
            [ent_a, ent_b],
            start_position=(9.5, 0.0),
        )
        mark_segs = [s for s in plan.segments if s.segment_type == SegmentType.MARK]
        assert len(mark_segs) == 2
        # B's north is 10.0; first mark should be near 10
        first_north = mark_segs[0].points[0][0]
        assert abs(first_north - 10.0) < 1.5, (
            f"Expected B first (north ~10), got {first_north}"
        )

    def test_saved_order_a_b_preserves_a_first(self):
        """apply_entity_order([B, A], ['A', 'B']) → A before B in plan."""
        ent_a, ent_b = self._make_two_line_entities()
        # Feed B first to engine but with saved order requesting A→B
        entities_reordered = apply_entity_order([ent_b, ent_a], ["A", "B"])
        engine = PathEngine(
            optimize_order=False,  # caller is responsible for disabling
            compensate_spray=False,
        )
        plan = engine.plan_dxf_entities(entities_reordered)
        mark_segs = [s for s in plan.segments if s.segment_type == SegmentType.MARK]
        assert len(mark_segs) == 2
        first_north = mark_segs[0].points[0][0]
        assert abs(first_north - 0.0) < 1.5, (
            f"Expected A first (north ~0), got {first_north}"
        )

    def test_saved_order_b_a_preserves_b_first(self):
        """Saved order ['B', 'A'] → B is executed before A regardless of start."""
        ent_a, ent_b = self._make_two_line_entities()
        entities_reordered = apply_entity_order([ent_a, ent_b], ["B", "A"])
        engine = PathEngine(optimize_order=False, compensate_spray=False)
        plan = engine.plan_dxf_entities(entities_reordered)
        mark_segs = [s for s in plan.segments if s.segment_type == SegmentType.MARK]
        first_north = mark_segs[0].points[0][0]
        assert abs(first_north - 10.0) < 1.5, (
            f"Expected B first (north ~10), got {first_north}"
        )

    def test_entity_to_segment_expansion_preserves_adjacency(self):
        """PRE TRANSIT + MARK + AFT TRANSIT for an entity stay adjacent and ordered.

        With path extensions, each MARK entity expands into up to 3 segments:
        pre-TRANSIT, MARK, aft-TRANSIT. These three must remain adjacent and
        must not be reordered relative to each other.
        """
        ent = _make_entity("A", north_start=5.0)
        engine = PathEngine(
            optimize_order=False,
            compensate_spray=False,
            enable_path_extensions=True,
            pre_extension_m=0.3,
            aft_extension_m=0.3,
        )
        plan = engine.plan_dxf_entities([ent])
        segs = plan.segments
        # With a single entity and extensions there should be:
        #   TRANSIT (pre) → MARK → TRANSIT (aft)
        types = [s.segment_type for s in segs]
        assert SegmentType.MARK in types
        # Verify MARK is surrounded by TRANSITs (or at boundaries)
        mark_idx = types.index(SegmentType.MARK)
        if mark_idx > 0:
            assert types[mark_idx - 1] == SegmentType.TRANSIT
        if mark_idx < len(types) - 1:
            assert types[mark_idx + 1] == SegmentType.TRANSIT


# ── Integration: optimizer gating ─────────────────────────────────────────────

class TestOptimizerGating:
    """Optimizer must be suppressed iff a saved entity order exists."""

    def _two_distant_mark_segments(self) -> list[PathSegment]:
        return [
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[(0.0, 0.0), (1.0, 0.0)],
                speed=0.35,
                source_entity="seg_A",
            ),
            PathSegment(
                segment_type=SegmentType.MARK,
                points=[(50.0, 0.0), (51.0, 0.0)],
                speed=0.35,
                source_entity="seg_B",
            ),
        ]

    def test_optimizer_runs_without_saved_order(self):
        """No saved order → optimization metadata shows nearest_neighbor."""
        engine = PathEngine(
            optimize_order=True,
            compensate_spray=False,
            use_two_opt=False,  # keep deterministic
        )
        segs = self._two_distant_mark_segments()
        plan = engine.plan_segments(segs, start_position=(50.5, 0.0))
        opt = plan.planning_metadata.get("optimization", {})
        assert opt.get("method") == "nearest_neighbor", (
            f"Expected nearest_neighbor, got {opt.get('method')!r}"
        )
        # With start near B, B should be first
        mark_segs = [s for s in plan.segments if s.segment_type == SegmentType.MARK]
        assert abs(mark_segs[0].points[0][0] - 50.0) < 1.0

    def test_optimizer_disabled_with_saved_order(self):
        """Saved order → optimize_order=False → 'disabled' in metadata."""
        engine = PathEngine(
            optimize_order=False,  # simulate what plan_path does with saved order
            compensate_spray=False,
        )
        segs = self._two_distant_mark_segments()
        plan = engine.plan_segments(segs, start_position=(50.5, 0.0))
        opt = plan.planning_metadata.get("optimization", {})
        assert opt.get("method") == "disabled", (
            f"Expected disabled, got {opt.get('method')!r}"
        )
        # Without optimization, A comes first (parser order)
        mark_segs = [s for s in plan.segments if s.segment_type == SegmentType.MARK]
        assert abs(mark_segs[0].points[0][0] - 0.0) < 1.0


# ── Integration: PathManager sidecar → plan pipeline ──────────────────────────

class TestPathManagerEntityOrder:
    """End-to-end: PathManager reads sidecar and suppresses optimizer."""

    @pytest.fixture()
    def missions_dir(self, tmp_path):
        return str(tmp_path / "missions")

    @pytest.fixture()
    def path_mgr(self, missions_dir):
        # Import inside fixture to avoid top-level server dependency
        import sys
        server_dir = os.path.join(
            os.path.dirname(__file__), "..", "..", "server"
        )
        sys.path.insert(0, os.path.abspath(server_dir))
        from path_manager import PathManager
        return PathManager(missions_dir)

    @pytest.fixture()
    def dxf_file(self, missions_dir):
        """Write a two-line DXF (A northward, B southward) to missions_dir."""
        pytest.importorskip("ezdxf")
        import ezdxf

        os.makedirs(missions_dir, exist_ok=True)
        doc = ezdxf.new("R2010")
        doc.header["$INSUNITS"] = 6  # metres
        msp = doc.modelspace()
        # Entity A: north 0→1
        msp.add_line((0, 0, 0), (0, 1, 0), dxfattribs={"layer": "0"})
        # Entity B: north 10→11
        msp.add_line((0, 10, 0), (0, 11, 0), dxfattribs={"layer": "0"})
        fpath = os.path.join(missions_dir, "test.dxf")
        doc.saveas(fpath)
        return "test.dxf"

    def _get_entity_ids(self, path_mgr, missions_dir, filename):
        """Helper: parse DXF and return entity IDs in parser order."""
        fpath = os.path.join(missions_dir, filename)
        entities = path_mgr.parse_dxf(fpath)
        return [e.entity_id for e in entities]

    def test_no_sidecar_optimizer_runs(self, path_mgr, missions_dir, dxf_file):
        """Without a sidecar, plan_path uses optimizer (optimize=True)."""
        result = path_mgr.plan_path(
            dxf_file,
            optimize=True,
            start_position=(9.5, 10.5),  # near B
        )
        entity_order_meta = result.get("planning_metadata", {}).get("entity_order")
        assert entity_order_meta is None, (
            "entity_order metadata should not appear when no sidecar exists"
        )
        opt = result.get("planning_metadata", {}).get("optimization", {})
        assert opt.get("method") != "disabled"

    def test_saved_order_disables_optimizer(self, path_mgr, missions_dir, dxf_file):
        """Sidecar present → plan_path sets optimize_order=False."""
        ids = self._get_entity_ids(path_mgr, missions_dir, dxf_file)
        assert len(ids) == 2, f"Expected 2 entities, got {len(ids)}: {ids}"

        # Save order reversed
        path_mgr.save_entity_order(dxf_file, list(reversed(ids)))

        result = path_mgr.plan_path(dxf_file, optimize=True)
        # planning_metadata should record that entity_order was applied
        entity_order_meta = result["planning_metadata"].get("entity_order", {})
        assert entity_order_meta.get("optimizer_skipped") is True
        assert entity_order_meta.get("num_ids") == 2

    def test_saved_order_b_a_plan_visits_b_first(self, path_mgr, missions_dir, dxf_file):
        """Saved order [B, A] → first MARK segment in plan starts near north=10."""
        ids = self._get_entity_ids(path_mgr, missions_dir, dxf_file)
        # Determine which ID maps to B (north ~10) vs A (north ~0)
        fpath = os.path.join(missions_dir, dxf_file)
        entities = path_mgr.parse_dxf(fpath)
        id_map = {e.entity_id: e for e in entities}
        # A is the entity whose start north < 5
        id_a = next(
            eid for eid in ids
            if id_map[eid].geometry["start"][0] < 5.0
        )
        id_b = next(
            eid for eid in ids
            if id_map[eid].geometry["start"][0] >= 5.0
        )
        # Save [B, A]
        path_mgr.save_entity_order(dxf_file, [id_b, id_a])

        result = path_mgr.plan_path(dxf_file, optimize=True)
        # First MARK segment should start near north=10 (B)
        mark_segs = [s for s in result["segments"] if s["type"] == "MARK"]
        assert len(mark_segs) >= 1
        # merged_waypoints[0] should be near B's north
        first_wp = result["merged_waypoints"][0]
        assert abs(first_wp[0] - 10.0) < 1.5, (
            f"Expected first waypoint near north=10 (B), got {first_wp[0]}"
        )

    def test_saved_order_a_b_plan_visits_a_first(self, path_mgr, missions_dir, dxf_file):
        """Saved order [A, B] → first MARK segment starts near north=0."""
        ids = self._get_entity_ids(path_mgr, missions_dir, dxf_file)
        fpath = os.path.join(missions_dir, dxf_file)
        entities = path_mgr.parse_dxf(fpath)
        id_map = {e.entity_id: e for e in entities}
        id_a = next(eid for eid in ids if id_map[eid].geometry["start"][0] < 5.0)
        id_b = next(eid for eid in ids if id_map[eid].geometry["start"][0] >= 5.0)
        path_mgr.save_entity_order(dxf_file, [id_a, id_b])

        # start_position near B — without saved order optimizer would pick B
        # With saved order, A must still come first
        result = path_mgr.plan_path(
            dxf_file,
            optimize=True,
            start_position=(9.5, 10.5),
        )
        first_wp = result["merged_waypoints"][0]
        assert abs(first_wp[0] - 0.0) < 1.5, (
            f"Saved order A→B: expected first waypoint near north=0, got {first_wp[0]}"
        )

    def test_preview_path_uses_saved_order(self, path_mgr, missions_dir, dxf_file):
        """preview_path() also suppresses optimizer when saved order exists."""
        ids = self._get_entity_ids(path_mgr, missions_dir, dxf_file)
        fpath = os.path.join(missions_dir, dxf_file)
        entities = path_mgr.parse_dxf(fpath)
        id_map = {e.entity_id: e for e in entities}
        id_a = next(eid for eid in ids if id_map[eid].geometry["start"][0] < 5.0)
        id_b = next(eid for eid in ids if id_map[eid].geometry["start"][0] >= 5.0)
        path_mgr.save_entity_order(dxf_file, [id_b, id_a])

        preview = path_mgr.preview_path(dxf_file)
        # First waypoint must be near B (north ~10)
        assert len(preview.waypoints) > 0
        first_north = preview.waypoints[0].north
        assert abs(first_north - 10.0) < 1.5, (
            f"preview first waypoint should be near B (north=10), got {first_north}"
        )
