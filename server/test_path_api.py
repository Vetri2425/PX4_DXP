import math
import os
import sys

# Ensure server directory is in python path
sys.path.insert(0, os.path.dirname(__file__))

import pytest
from fastapi import HTTPException
from types import SimpleNamespace
from models import MissionState, PathPlanRequest, PathPreviewResponse, RefPoint
from routes.path import (
    get_path_extensions,
    path_entities,
    plan_path,
    preview_path,
    save_path_entity_overrides,
    save_path_extensions,
    update_entity_order,
)
import main
from path_manager import PathManager


def test_path_manager_preview_returns_bounds_and_local_ned_points(tmp_path):
    mission_file = tmp_path / "line.csv"
    mission_file.write_text("0,0\n1.5,-0.25\n2.0,0.75\n", encoding="utf-8")

    mgr = PathManager(str(tmp_path))
    preview = mgr.preview_path("line.csv")

    assert preview.name == "line.csv"
    assert preview.frame == "local_ned"
    assert preview.num_points == 3
    assert preview.bounds is not None
    assert preview.bounds.north_min == 0.0
    assert preview.bounds.north_max == 2.0
    assert preview.bounds.east_min == -0.25
    assert preview.bounds.east_max == 0.75
    assert preview.waypoints[1].north == 1.5
    assert preview.waypoints[1].east == -0.25
    assert all(pt.spray is True for pt in preview.waypoints)


def test_path_manager_preview_survey_csv_real_must_hit_and_geo_origin(tmp_path):
    """A named-header survey CSV previews via the planner: only surveyed vertices
    are must-hit (not every densified point), and the WGS84 origin is exposed."""
    mission_file = tmp_path / "survey.csv"
    lat0, lon0, mn, me = 13.0721, 80.2620, 110900.0, 108400.0
    rows = ["Name,Code,Latitude,Longitude"]
    for i, (n, e) in enumerate([(0, 0), (1, 0), (2, 0), (2, 2)]):
        rows.append(f"{i + 1},L_1,{lat0 + n / mn:.8f},{lon0 + e / me:.8f}")
    mission_file.write_text("\n".join(rows) + "\n", encoding="utf-8")

    preview = PathManager(str(tmp_path)).preview_path("survey.csv")

    must_hit = [pt.must_hit for pt in preview.waypoints]
    assert sum(must_hit) == 4, "exactly the four surveyed vertices"
    assert not all(must_hit), "densified fill is not must-hit"
    assert preview.num_points > 4, "the chain was densified"
    assert preview.geo_origin is not None
    assert abs(preview.geo_origin[0] - lat0) < 0.01


def _count_big_kinks(pts, deg=5.0):
    import math
    big = 0
    for i in range(1, len(pts) - 1):
        v1 = (pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        v2 = (pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        m1 = math.hypot(*v1); m2 = math.hypot(*v2)
        if m1 < 1e-9 or m2 < 1e-9:
            continue
        d = (v1[0] * v2[0] + v1[1] * v2[1]) / (m1 * m2)
        if math.degrees(math.acos(max(-1.0, min(1.0, d)))) > deg:
            big += 1
    return big


def test_survey_csv_arc_previews_as_a_true_arc_not_chords(tmp_path):
    """A surveyed curve auto-arc-fits in the preview (was straight chords)."""
    import math
    from path_engine.parsers.georef import metres_per_degree
    lat0, lon0 = 13.0721, 80.2620
    mn, me = metres_per_degree(lat0)
    rows = ["Name,Code,Latitude,Longitude"]
    for i in range(7):  # 90° arc, radius 5 m, 7 surveyed points
        a = math.radians(90 * i / 6)
        n, e = 5.0 * math.sin(a), 5.0 * math.cos(a)
        rows.append(f"{i + 1},L_1,{lat0 + n / mn:.8f},{lon0 + e / me:.8f}")
    (tmp_path / "arc.csv").write_text("\n".join(rows) + "\n", encoding="utf-8")

    pts = [(w.north, w.east)
           for w in PathManager(str(tmp_path)).preview_path("arc.csv").waypoints]
    # A true arc bends by a fraction of a degree per densified step: no big kinks.
    # Straight chords between the 7 vertices would show ~5 big (>5°) kinks.
    assert _count_big_kinks(pts) == 0


def test_survey_csv_square_stays_square_in_preview(tmp_path):
    """Auto arc-fit must NOT round a square — corner-split keeps sharp corners."""
    import math
    from path_engine.parsers.georef import metres_per_degree
    lat0, lon0 = 13.0721, 80.2620
    mn, me = metres_per_degree(lat0)
    rows = ["Name,Code,Latitude,Longitude"]
    for i, (n, e) in enumerate([(0, 0), (2, 0), (2, 2), (0, 2), (0, 0)], start=1):
        rows.append(f"{i},L_1,{lat0 + n / mn:.8f},{lon0 + e / me:.8f}")
    (tmp_path / "square.csv").write_text("\n".join(rows) + "\n", encoding="utf-8")

    pts = [(w.north, w.east)
           for w in PathManager(str(tmp_path)).preview_path("square.csv").waypoints]
    # A closed square keeps its three interior 90° corners as sharp kinks.
    assert _count_big_kinks(pts) == 3


def test_path_manager_preview_legacy_ned_csv_has_no_geo_origin(tmp_path):
    """A headerless NED CSV is an explicit point list — no geo origin, all vertices."""
    mission_file = tmp_path / "line.csv"
    mission_file.write_text("0,0\n1.5,-0.25\n2.0,0.75\n", encoding="utf-8")

    preview = PathManager(str(tmp_path)).preview_path("line.csv")

    assert preview.geo_origin is None
    assert all(pt.must_hit for pt in preview.waypoints)


def test_path_manager_preview_preserves_dxf_spray_flags(tmp_path, monkeypatch):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakeEngine:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

        def plan_file(self, filepath):
            assert filepath == str(mission_file)
            return SimpleNamespace(
                merged_waypoints=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
                spray_flags=[True, False, True],
            )

    import path_engine

    monkeypatch.setattr(path_engine, "PathEngine", FakeEngine)

    mgr = PathManager(str(tmp_path))
    preview = mgr.preview_path("field.dxf")

    assert [pt.spray for pt in preview.waypoints] == [True, False, True]


def test_path_manager_preview_caches_uploaded_file_result(tmp_path, monkeypatch):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")
    calls = {"count": 0}

    class FakeEngine:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

        def plan_file(self, filepath):
            calls["count"] += 1
            return SimpleNamespace(
                merged_waypoints=[(0.0, 0.0), (1.0, 0.0)],
                spray_flags=[True, True],
            )

    import path_engine

    monkeypatch.setattr(path_engine, "PathEngine", FakeEngine)

    mgr = PathManager(str(tmp_path))
    first = mgr.preview_path("field.dxf")
    second = mgr.preview_path("field.dxf")

    assert calls["count"] == 1
    assert first is second


def test_path_manager_clear_entity_overrides_invalidates_preview_cache(tmp_path):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    mgr = PathManager(str(tmp_path))
    fpath = str(mission_file)
    mgr._preview_cache[fpath] = (1, 2, object())

    mgr.clear_entity_overrides("field.dxf")

    assert fpath not in mgr._preview_cache


@pytest.mark.anyio
async def test_preview_api_returns_path_preview(monkeypatch):
    class FakePathManager:
        def preview_path(self, name):
            return PathPreviewResponse(
                name=name,
                num_points=2,
                bounds={
                    "north_min": 0.0,
                    "north_max": 1.0,
                    "east_min": 0.0,
                    "east_max": 0.5,
                },
                waypoints=[
                    {"north": 0.0, "east": 0.0, "spray": True},
                    {"north": 1.0, "east": 0.5, "spray": True},
                ],
            )

    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await preview_path("square_2x2")

    assert data.name == "square_2x2"
    assert data.frame == "local_ned"
    assert data.num_points == 2
    assert data.bounds.north_max == 1.0
    assert data.waypoints[1].east == 0.5


@pytest.mark.anyio
async def test_preview_api_missing_path_is_404(monkeypatch):
    class FakePathManager:
        def preview_path(self, name):
            raise FileNotFoundError(f"Path not found: {name!r}")

    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    with pytest.raises(HTTPException) as exc:
        await preview_path("missing.csv")

    assert exc.value.status_code == 404
    assert "missing.csv" in exc.value.detail


@pytest.mark.anyio
async def test_entities_api_returns_line_preview_points(tmp_path, monkeypatch):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            assert filepath == str(mission_file)
            return [
                SimpleNamespace(
                    entity_id="A1",
                    entity_type="LINE",
                    layer="MARKING",
                    color=7,
                    geometry={
                        "start": (0.0, 0.0),
                        "end": (1.0, 2.0),
                    },
                    is_mark=lambda: True,
                )
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {
                "enabled": False,
                "pre_extension_m": 0.5,
                "aft_extension_m": 0.5,
            }

        def load_entity_order(self, filename):
            return []

    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("field.dxf")

    assert data.name == "field.dxf"
    assert data.frame == "local_ned"
    assert data.num_entities == 1
    assert data.bounds.north_max == 1.0
    assert data.bounds.east_max == 2.0
    ent = data.entities[0]
    assert ent.entity_id == "A1"
    assert ent.entity_type == "LINE"
    assert ent.length_m == pytest.approx(2.236, abs=0.001)
    assert [pt.model_dump() for pt in ent.preview_points] == [
        {"north": 0.0, "east": 0.0},
        {"north": 1.0, "east": 2.0},
    ]


def test_path_manager_saves_entity_overrides(tmp_path):
    from path_engine.core import DXFEntity

    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")
    mgr = PathManager(str(tmp_path))
    mgr.parse_dxf = lambda *args, **kwargs: [
        DXFEntity(entity_type="LINE", layer="MARKINGS", entity_id="A1")
    ]

    saved = mgr.save_entity_overrides("field.dxf", {"A1": False})

    assert saved == 1
    assert mgr.load_entity_overrides("field.dxf") == {"A1": False}


@pytest.mark.anyio
async def test_entities_api_applies_saved_mark_override(tmp_path, monkeypatch):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            assert filepath == str(mission_file)
            return [
                SimpleNamespace(
                    entity_id="A1",
                    entity_type="LINE",
                    layer="MARKINGS",
                    color=7,
                    geometry={
                        "start": (0.0, 0.0),
                        "end": (1.0, 0.0),
                    },
                    is_mark=lambda: True,
                )
            ]

        def load_entity_overrides(self, filename):
            return {"A1": False}

        def load_extension_config(self, filename):
            return {
                "enabled": False,
                "pre_extension_m": 0.5,
                "aft_extension_m": 0.5,
            }

        def load_entity_order(self, filename):
            return []

    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("field.dxf")

    assert data.entities[0].default_is_mark is True
    assert data.entities[0].is_mark is False


@pytest.mark.anyio
async def test_entities_api_includes_extension_preview(tmp_path, monkeypatch):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            assert filepath == str(mission_file)
            return [
                SimpleNamespace(
                    entity_id="A1",
                    entity_type="LINE",
                    layer="MARKINGS",
                    color=7,
                    geometry={
                        "start": (0.0, 0.0),
                        "end": (1.0, 0.0),
                    },
                    is_mark=lambda: True,
                )
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {
                "enabled": True,
                "pre_extension_m": 0.5,
                "aft_extension_m": 0.25,
            }

        def load_entity_order(self, filename):
            return []

    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("field.dxf")

    ext = data.entities[0].extension_preview
    assert data.extension_config.enabled is True
    assert ext.enabled is True
    assert ext.pre_points[0].north == -0.5
    assert ext.aft_points[-1].north == 1.25
    assert data.bounds.north_min == -0.5
    assert data.bounds.north_max == 1.25


@pytest.mark.anyio
async def test_entities_api_uses_dense_spline_points_for_extension_preview(tmp_path, monkeypatch):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            assert filepath == str(mission_file)
            return [
                SimpleNamespace(
                    entity_id="S1",
                    entity_type="SPLINE",
                    layer="MARKINGS",
                    color=7,
                    geometry={"vertices": [(0.0, 0.0), (1.0, 0.0), (1.0, 10.0)]},
                    is_mark=lambda: True,
                )
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {
                "enabled": True,
                "pre_extension_m": 0.5,
                "aft_extension_m": 0.25,
            }

        def load_entity_order(self, filename):
            return []

    import routes.path as path_route

    def fake_preview_tuples(ent, max_points=200):
        if max_points == 10000:
            return [(0.0, 0.0), (1.0, 0.0), (1.0, 10.0)]
        return [(0.0, 0.0), (0.0, 10.0)]

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(path_route, "_entity_preview_tuples", fake_preview_tuples)
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("field.dxf")

    ext = data.entities[0].extension_preview
    assert [pt.model_dump() for pt in data.entities[0].preview_points] == [
        {"north": 0.0, "east": 0.0},
        {"north": 0.0, "east": 10.0},
    ]
    assert ext.pre_points[0].north == pytest.approx(-0.5)
    assert ext.pre_points[0].east == pytest.approx(0.0)
    assert ext.aft_points[-1].north == pytest.approx(1.0)
    assert ext.aft_points[-1].east == pytest.approx(10.25)


@pytest.mark.anyio
async def test_entities_api_suppresses_extension_preview_on_closed_chain(tmp_path, monkeypatch):
    """per_line=False only. A closed square (4 edges meeting corner-to-corner)
    has no free end, so NO entity may show a run-up — matching the planner's
    vertex-anchored chain-ends policy (suppress_closed_loops=True). Internal
    corners are junctions, not open ends.

    per_line=True is the opposite contract and is covered by
    test_entities_api_per_line_extends_every_side_of_closed_square."""
    mission_file = tmp_path / "square.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    corners = [(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [
                SimpleNamespace(
                    entity_id=f"E{i}",
                    entity_type="LINE",
                    layer="MARKINGS",
                    color=7,
                    geometry={"start": corners[i], "end": corners[i + 1]},
                    is_mark=lambda: True,
                )
                for i in range(4)
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            # Explicit: this test pins the per_line=False chain-ends policy.
            return {"enabled": True, "pre_extension_m": 0.5, "aft_extension_m": 0.5,
                    "per_line": False}

        def load_entity_order(self, filename):
            return []

    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("square.dxf")

    assert data.extension_config.enabled is True
    for ent in data.entities:
        assert ent.extension_preview.enabled is False, ent.entity_id
        assert ent.extension_preview.pre_points == []
        assert ent.extension_preview.aft_points == []


@pytest.mark.anyio
async def test_entities_api_extension_preview_only_at_open_ends_of_chain(tmp_path, monkeypatch):
    """Open L-chain (two edges sharing one corner): the run-up appears only at
    the two outer ends, never at the shared internal corner."""
    mission_file = tmp_path / "lshape.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [
                SimpleNamespace(
                    entity_id="L0", entity_type="LINE", layer="M", color=7,
                    geometry={"start": (0.0, 0.0), "end": (2.0, 0.0)},
                    is_mark=lambda: True,
                ),
                SimpleNamespace(
                    entity_id="L1", entity_type="LINE", layer="M", color=7,
                    geometry={"start": (2.0, 0.0), "end": (2.0, 2.0)},
                    is_mark=lambda: True,
                ),
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            # Explicit: this test pins the per_line=False chain-ends policy.
            return {"enabled": True, "pre_extension_m": 0.5, "aft_extension_m": 0.5,
                    "per_line": False}

        def load_entity_order(self, filename):
            return []

    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("lshape.dxf")
    by_id = {e.entity_id: e.extension_preview for e in data.entities}

    # L0: free start at (0,0) -> PRE only; shared end at corner -> no AFT.
    assert by_id["L0"].pre_points != [] and by_id["L0"].aft_points == []
    # L1: shared start at corner -> no PRE; free end at (2,2) -> AFT only.
    assert by_id["L1"].pre_points == [] and by_id["L1"].aft_points != []


def _fake_square_mgr(per_line: bool, corners=None):
    """4 LINE entities drawn corner-to-corner as a closed square."""
    corners = corners or [(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [
                SimpleNamespace(
                    entity_id=f"E{i}", entity_type="LINE", layer="M", color=7,
                    geometry={"start": corners[i], "end": corners[i + 1]},
                    is_mark=lambda: True,
                )
                for i in range(4)
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {"enabled": True, "pre_extension_m": 0.5, "aft_extension_m": 0.5,
                    "per_line": per_line}

        def load_entity_order(self, filename):
            return []

    return FakePathManager()


@pytest.mark.anyio
async def test_entities_api_per_line_extends_every_side_of_closed_square(tmp_path, monkeypatch):
    """per_line=True: every side of a CLOSED square gets its own PRE and AFT.

    Regression for the preview<->plan divergence: the preview applied the
    chain-ends freeness gate unconditionally and reported enabled=False on all
    four sides, while the planner (decompose_line_chain_to_edges +
    suppress_closed_loops=False) really does emit 4 PRE + 4 AFT here — see
    path_engine test_engine_per_line_square_gives_four_passes. A square's
    corners are perpendicular, so none of them is a collinear retrace.
    """
    (tmp_path / "square.dxf").write_text("0\nEOF\n", encoding="utf-8")
    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", _fake_square_mgr(per_line=True))

    data = await path_entities("square.dxf")

    assert data.extension_config.per_line is True
    for ent in data.entities:
        assert ent.extension_preview.enabled is True, ent.entity_id
        assert ent.extension_preview.pre_points != [], ent.entity_id
        assert ent.extension_preview.aft_points != [], ent.entity_id
        assert ent.extension_preview.pre_length_m == 0.5
        assert ent.extension_preview.aft_length_m == 0.5

    # Matches the planner's (pre, aft) == (4, 4) on this same shape.
    assert sum(1 for x in data.extensions if x.role == "pre") == 4
    assert sum(1 for x in data.extensions if x.role == "aft") == 4


def _fake_closed_polyline_mgr(per_line: bool, closed: bool = True):
    """ONE closed LWPOLYLINE square — the test_1.dxf shape (a single polyline,
    not 4 separate LINEs). Previously the preview treated this as one self-closed
    run and produced 0 extensions, while the plan splits it into 4 sides."""
    verts = [(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0)]

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [SimpleNamespace(
                entity_id="PL0", entity_type="LWPOLYLINE", layer="Lines", color=7,
                geometry={"vertices": verts, "bulges": [0.0] * len(verts),
                          "closed": closed},
                is_mark=lambda: True,
            )]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {"enabled": True, "pre_extension_m": 0.5, "aft_extension_m": 0.5,
                    "per_line": per_line}

        def load_entity_order(self, filename):
            return []

    return FakePathManager()


@pytest.mark.anyio
async def test_entity_order_excludes_survey_points(tmp_path, monkeypatch):
    """A DXF that is one LWPOLYLINE + survey POINTs must accept an order of just
    the polyline — POINTs are not drivable and the client never orders them.

    Regression for the georeferenced-square load failure: the order contract
    required EVERY parsed entity id (incl. 5 POINTs), so a valid order of the
    single polyline was rejected as 'Missing entity IDs'.
    """
    from path_engine.core import DXFEntity
    from models import EntityOrderUpdateRequest
    import routes.path as path_route

    (tmp_path / "geo.dxf").write_text("0\nEOF\n", encoding="utf-8")
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))

    saved = {}

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [
                DXFEntity(entity_type="LWPOLYLINE", layer="Lines", entity_id="PL0",
                          geometry={"vertices": [(0.0, 0.0), (2.0, 0.0)]}),
                DXFEntity(entity_type="POINT", layer="Points", entity_id="P1",
                          geometry={"position": (0.0, 0.0)}),
                DXFEntity(entity_type="POINT", layer="Points", entity_id="P2",
                          geometry={"position": (2.0, 0.0)}),
            ]

        def save_entity_order(self, filename, order):
            saved["order"] = list(order)

    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    # Ordering just the polyline must succeed — the two POINTs are excluded.
    resp = await update_entity_order("geo.dxf", EntityOrderUpdateRequest(entity_order=["PL0"]))
    assert resp.entity_order == ["PL0"]
    assert saved["order"] == ["PL0"]


@pytest.mark.anyio
async def test_entity_order_rejects_point_id(tmp_path, monkeypatch):
    """A POINT id is not orderable, so posting it is 'unknown', not accepted."""
    from path_engine.core import DXFEntity
    from models import EntityOrderUpdateRequest
    import routes.path as path_route

    (tmp_path / "geo.dxf").write_text("0\nEOF\n", encoding="utf-8")
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [
                DXFEntity(entity_type="LWPOLYLINE", layer="Lines", entity_id="PL0",
                          geometry={"vertices": [(0.0, 0.0), (2.0, 0.0)]}),
                DXFEntity(entity_type="POINT", layer="Points", entity_id="P1",
                          geometry={"position": (0.0, 0.0)}),
            ]

        def save_entity_order(self, filename, order):
            pass

    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    with pytest.raises(HTTPException) as exc:
        await update_entity_order("geo.dxf", EntityOrderUpdateRequest(entity_order=["PL0", "P1"]))
    assert exc.value.status_code == 422
    assert "Unknown" in str(exc.value.detail)


def _fake_geographic_mgr():
    """One polyline whose entities carry a geo_origin — i.e. parse_dxf detected
    lat/lon and georef projected + stamped the WGS84 origin (as it does for a
    real georeferenced DXF)."""
    verts = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]

    class FakePathManager:
        def parse_dxf(self, filepath):
            ent = SimpleNamespace(
                entity_id="PL0", entity_type="LWPOLYLINE", layer="Lines", color=7,
                geometry={"vertices": verts, "bulges": [0.0] * 4, "closed": True},
                is_mark=lambda: True, geo_origin=(13.072071, 80.261949))
            return [ent]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {"enabled": False, "pre_extension_m": 0.5, "aft_extension_m": 0.5,
                    "per_line": True}

        def load_entity_order(self, filename):
            return []

    return FakePathManager()


@pytest.mark.anyio
async def test_entities_api_exposes_geographic_signal(tmp_path, monkeypatch):
    """A georeferenced DXF reports is_geographic + geo_origin so the client can
    skip manual alignment and stage straight to geo_origin."""
    (tmp_path / "geo.dxf").write_text("0\nEOF\n", encoding="utf-8")
    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", _fake_geographic_mgr())

    data = await path_entities("geo.dxf")
    assert data.is_geographic is True
    assert data.geo_origin == [13.072071, 80.261949]


@pytest.mark.anyio
async def test_entities_api_metric_dxf_not_geographic(tmp_path, monkeypatch):
    """A plain metric DXF (no geo_origin) reports is_geographic False / None."""
    (tmp_path / "sq.dxf").write_text("0\nEOF\n", encoding="utf-8")
    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", _fake_closed_polyline_mgr(per_line=True))

    data = await path_entities("sq.dxf")
    assert data.is_geographic is False
    assert data.geo_origin is None


@pytest.mark.anyio
async def test_entities_api_per_line_closed_polyline_matches_plan(tmp_path, monkeypatch):
    """A single CLOSED LWPOLYLINE square must preview the same 4 PRE + 4 AFT the
    plan drives — not 0.

    This is the preview<->plan divergence the earlier per_line fix did NOT cover:
    it handled a square drawn as 4 separate LINEs, but a square drawn as ONE
    closed polyline hit the self-closed 'no free end' branch and got nothing,
    while engine.py (decompose_line_chain_to_edges, gated on per_line) really
    splits it into 4 sides and extends each. Cross-checked against the planner
    functions directly below.
    """
    (tmp_path / "poly.dxf").write_text("0\nEOF\n", encoding="utf-8")
    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", _fake_closed_polyline_mgr(per_line=True))

    data = await path_entities("poly.dxf")

    n_pre = sum(1 for x in data.extensions if x.role == "pre")
    n_aft = sum(1 for x in data.extensions if x.role == "aft")
    assert (n_pre, n_aft) == (4, 4), f"preview gave {(n_pre, n_aft)}, plan drives (4, 4)"
    # All four runs belong to the one polyline, disambiguated by edge_index.
    assert {x.entity_id for x in data.extensions} == {"PL0"}
    assert {x.edge_index for x in data.extensions if x.role == "pre"} == {0, 1, 2, 3}


@pytest.mark.anyio
async def test_entities_api_per_line_closed_polyline_equals_planner(tmp_path, monkeypatch):
    """Definitive: preview extension count == what the real planner emits.

    Runs decompose_line_chain_to_edges + split_mark_segment_with_extensions on
    the identical geometry and asserts the preview produced the same number of
    PRE/AFT runs. If the planner's edge-splitting ever changes, this fails.
    """
    from path_engine.core import PathSegment, SegmentType
    from path_engine.planners.extensions import (
        decompose_line_chain_to_edges, split_mark_segment_with_extensions,
    )
    verts = [(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]
    seg = PathSegment(segment_type=SegmentType.MARK, points=verts, speed=0.35,
                      source_entity="PL0", metadata={"geometry_type": "LWPOLYLINE"})
    plan_pre = plan_aft = 0
    for edge in decompose_line_chain_to_edges(seg):
        parts = split_mark_segment_with_extensions(
            edge, pre_extension_m=0.5, aft_extension_m=0.5, transit_speed=0.2,
            suppress_closed_loops=False)
        plan_pre += sum(1 for p in parts if p.metadata.get("extension_role") == "pre")
        plan_aft += sum(1 for p in parts if p.metadata.get("extension_role") == "aft")

    (tmp_path / "poly.dxf").write_text("0\nEOF\n", encoding="utf-8")
    import routes.path as path_route
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", _fake_closed_polyline_mgr(per_line=True))
    data = await path_entities("poly.dxf")

    prev_pre = sum(1 for x in data.extensions if x.role == "pre")
    prev_aft = sum(1 for x in data.extensions if x.role == "aft")
    assert (prev_pre, prev_aft) == (plan_pre, plan_aft), \
        f"preview {(prev_pre, prev_aft)} != plan {(plan_pre, plan_aft)}"


@pytest.mark.anyio
async def test_entities_api_per_line_closed_polyline_grows_corner_connectors(tmp_path, monkeypatch):
    """The sides of a split polyline get their between-side connectors.

    Regression for 'the connector is missing': a single polyline was one
    connector-endpoint and produced none. Per-edge endpoints give one connector
    per corner traversed (3 for a 4-side square painted in sequence), each
    spanning AFT-tip -> next PRE-start = sqrt(0.5^2 + 0.5^2) = 0.707 m.
    """
    (tmp_path / "poly.dxf").write_text("0\nEOF\n", encoding="utf-8")
    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", _fake_closed_polyline_mgr(per_line=True))

    data = await path_entities("poly.dxf")

    assert len(data.transit_preview) == 3, [t.length_m for t in data.transit_preview]
    for t in data.transit_preview:
        assert t.from_entity_id == "PL0" and t.to_entity_id == "PL0"
        assert abs(t.length_m - 0.707) < 0.01, t.length_m


@pytest.mark.anyio
async def test_entities_api_chain_ends_closed_polyline_gets_no_extension(tmp_path, monkeypatch):
    """per_line=False (chain-ends): a closed polyline still gets NO extension —
    a closed loop has no true open end, and the planner does not decompose in
    this mode. Guards against the fix leaking into chain-ends behaviour."""
    (tmp_path / "poly.dxf").write_text("0\nEOF\n", encoding="utf-8")
    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", _fake_closed_polyline_mgr(per_line=False))

    data = await path_entities("poly.dxf")
    assert data.extensions == []
    assert all(not e.extension_preview.enabled for e in data.entities)


@pytest.mark.anyio
async def test_entities_api_per_line_run_up_geometry_is_outward_and_collinear(tmp_path, monkeypatch):
    """A PRE run-up must approach ALONG the line from outside it, not sideways.

    E0 runs (0,0)->(0,2) (north const, east +). Its PRE therefore starts 0.5 m
    BEFORE (0,0) on the same line — i.e. at east=-0.5 — and ends at the vertex.
    """
    (tmp_path / "square.dxf").write_text("0\nEOF\n", encoding="utf-8")
    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", _fake_square_mgr(per_line=True))

    data = await path_entities("square.dxf")
    pre = next(x for x in data.extensions if x.entity_id == "E0" and x.role == "pre")

    assert (pre.points[0].north, round(pre.points[0].east, 6)) == (0.0, -0.5)
    assert (pre.points[-1].north, pre.points[-1].east) == (0.0, 0.0)
    assert pre.length_m == 0.5


@pytest.mark.anyio
async def test_entities_api_per_line_suppresses_collinear_retrace_junction(tmp_path, monkeypatch):
    """per_line=True still drops a run-up at a COLLINEAR junction.

    Two edges continuing straight through a shared point are a retrace: the AFT
    would run out along the very line the next PRE runs back in on, so the
    connector doubles back over both (the d82317d field failure). The planner's
    _touches() is a direction test, so the preview must be one too — a shared
    point alone is not enough to block, but a straight-through one is.
    """
    (tmp_path / "split.dxf").write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [  # one 4 m line drawn as two collinear 2 m halves
                SimpleNamespace(
                    entity_id="C0", entity_type="LINE", layer="M", color=7,
                    geometry={"start": (0.0, 0.0), "end": (0.0, 2.0)},
                    is_mark=lambda: True,
                ),
                SimpleNamespace(
                    entity_id="C1", entity_type="LINE", layer="M", color=7,
                    geometry={"start": (0.0, 2.0), "end": (0.0, 4.0)},
                    is_mark=lambda: True,
                ),
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {"enabled": True, "pre_extension_m": 0.5, "aft_extension_m": 0.5,
                    "per_line": True}

        def load_entity_order(self, filename):
            return []

    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("split.dxf")
    by_id = {e.entity_id: e.extension_preview for e in data.entities}

    # Outer ends stay free; the collinear seam at (0,2) is suppressed both sides.
    assert by_id["C0"].pre_points != [] and by_id["C0"].aft_points == []
    assert by_id["C1"].pre_points == [] and by_id["C1"].aft_points != []


@pytest.mark.anyio
async def test_entities_api_extensions_layer_mirrors_per_entity_preview(tmp_path, monkeypatch):
    """The named `extensions` layer is a flattening, never a second source of truth."""
    (tmp_path / "square.dxf").write_text("0\nEOF\n", encoding="utf-8")
    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", _fake_square_mgr(per_line=True))

    data = await path_entities("square.dxf")

    flat = {(x.entity_id, x.role): [(p.north, p.east) for p in x.points]
            for x in data.extensions}
    expected = {}
    for ent in data.entities:
        ep = ent.extension_preview
        if ep.pre_points:
            expected[(ent.entity_id, "pre")] = [(p.north, p.east) for p in ep.pre_points]
        if ep.aft_points:
            expected[(ent.entity_id, "aft")] = [(p.north, p.east) for p in ep.aft_points]
    assert flat == expected


@pytest.mark.anyio
async def test_entities_api_extensions_layer_empty_when_chain_ends_suppress(tmp_path, monkeypatch):
    """per_line=False on a closed square: no previews, so no `extensions` layer."""
    (tmp_path / "square.dxf").write_text("0\nEOF\n", encoding="utf-8")
    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", _fake_square_mgr(per_line=False))

    data = await path_entities("square.dxf")
    assert data.extensions == []


@pytest.mark.anyio
async def test_entities_api_connectors_span_aft_tip_to_next_pre_start(tmp_path, monkeypatch):
    """With per-line extensions on, travel runs AFT-tip -> next PRE-start.

    The planner routes connectors only AFTER extension, so a connector pinned to
    the mark vertex would overlap the run-up it should start from and
    under-report the distance driven. Square in DXF order 0..3:
      E0 (0,0)->(0,2) exits its AFT at (0,2.5)
      E1 (0,2)->(2,2) enters its PRE at (-0.5,2)
    so the connector is (0,2.5) -> (-0.5,2), NOT the vertex (0,2) -> (0,2).
    """
    (tmp_path / "square.dxf").write_text("0\nEOF\n", encoding="utf-8")
    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", _fake_square_mgr(per_line=True))

    data = await path_entities("square.dxf")
    t0 = data.transit_preview[0]

    assert (t0.from_entity_id, t0.to_entity_id) == ("E0", "E1")
    assert (t0.points[0].north, t0.points[0].east) == (0.0, 2.5)   # E0 AFT tip
    assert (t0.points[-1].north, round(t0.points[-1].east, 6)) == (-0.5, 2.0)  # E1 PRE start
    assert t0.length_m == round(math.hypot(0.5, 0.5), 3) == 0.707


@pytest.mark.anyio
async def test_entities_api_per_line_corners_are_not_dropped_as_zero_length(tmp_path, monkeypatch):
    """Every corner of a per-line square has a REAL connector.

    Pre-fix the preview measured vertex->vertex, got 0, and silently skipped it —
    hiding travel the rover genuinely drives. Once each edge runs off its own
    end the edges no longer touch, so all 3 in-order junctions are ~0.707 m.
    """
    (tmp_path / "square.dxf").write_text("0\nEOF\n", encoding="utf-8")
    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", _fake_square_mgr(per_line=True))

    data = await path_entities("square.dxf")

    assert len(data.transit_preview) == 3
    for t in data.transit_preview:
        assert t.length_m == 0.707, (t.from_entity_id, t.to_entity_id, t.length_m)


@pytest.mark.anyio
async def test_entities_api_connectors_use_mark_endpoints_when_no_extensions(tmp_path, monkeypatch):
    """No extensions -> connectors are unchanged: vertex to vertex, touching
    corners still collapse to nothing. Guards the fallback path."""
    (tmp_path / "square.dxf").write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            corners = [(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]
            return [
                SimpleNamespace(
                    entity_id=f"E{i}", entity_type="LINE", layer="M", color=7,
                    geometry={"start": corners[i], "end": corners[i + 1]},
                    is_mark=lambda: True,
                )
                for i in range(4)
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {"enabled": False, "pre_extension_m": 0.5, "aft_extension_m": 0.5,
                    "per_line": True}

        def load_entity_order(self, filename):
            return []

    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("square.dxf")

    assert data.extensions == []
    # Consecutive edges share their vertices, so every connector is zero-length
    # and correctly dropped.
    assert data.transit_preview == []


@pytest.mark.anyio
async def test_entities_api_extension_bounds_include_run_ups(tmp_path, monkeypatch):
    """Bounds must cover the run-ups, or a client clips them off-canvas.

    per_line square 0..2 with 0.5 m extensions -> -0.5..2.5 on both axes.
    """
    (tmp_path / "square.dxf").write_text("0\nEOF\n", encoding="utf-8")
    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", _fake_square_mgr(per_line=True))

    data = await path_entities("square.dxf")
    assert data.bounds.north_min == -0.5 and data.bounds.north_max == 2.5
    assert data.bounds.east_min == -0.5 and data.bounds.east_max == 2.5


@pytest.mark.anyio
async def test_entities_api_includes_entity_to_entity_transit_preview(tmp_path, monkeypatch):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            assert filepath == str(mission_file)
            return [
                SimpleNamespace(
                    entity_id="A1",
                    entity_type="LINE",
                    layer="MARKINGS",
                    color=7,
                    geometry={
                        "start": (0.0, 0.0),
                        "end": (1.0, 0.0),
                    },
                    is_mark=lambda: True,
                ),
                SimpleNamespace(
                    entity_id="A2",
                    entity_type="LINE",
                    layer="MARKINGS",
                    color=7,
                    geometry={
                        "start": (1.0, 2.0),
                        "end": (2.0, 2.0),
                    },
                    is_mark=lambda: True,
                ),
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {
                "enabled": False,
                "pre_extension_m": 0.5,
                "aft_extension_m": 0.5,
            }

        def load_entity_order(self, filename):
            return []

    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("field.dxf")

    assert len(data.transit_preview) == 1
    transit = data.transit_preview[0]
    assert transit.from_entity_id == "A1"
    assert transit.to_entity_id == "A2"
    assert transit.length_m == 2.0
    assert [pt.model_dump() for pt in transit.points] == [
        {"north": 1.0, "east": 0.0},
        {"north": 1.0, "east": 2.0},
    ]


@pytest.mark.anyio
async def test_transit_preview_skips_pointless_mark_entity_without_breaking_chain(
    tmp_path, monkeypatch
):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    def line(entity_id, start, end):
        return SimpleNamespace(
            entity_id=entity_id,
            entity_type="LINE",
            layer="MARKINGS",
            color=7,
            geometry={"start": start, "end": end},
            is_mark=lambda: True,
        )

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [
                line("A1", (0.0, 0.0), (1.0, 0.0)),
                # MARK entity with no drawable preview points (unsupported
                # geometry type) — must not break the transit chain.
                SimpleNamespace(
                    entity_id="A2",
                    entity_type="SOLID",
                    layer="MARKINGS",
                    color=7,
                    geometry={},
                    is_mark=lambda: True,
                ),
                line("A3", (3.0, 0.0), (4.0, 0.0)),
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {
                "enabled": False,
                "pre_extension_m": 0.5,
                "aft_extension_m": 0.5,
            }

        def load_entity_order(self, filename):
            return []

    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("field.dxf")

    # A1 and A3 connect across the degenerate A2, like the planner would.
    assert [(t.from_entity_id, t.to_entity_id) for t in data.transit_preview] == [
        ("A1", "A3"),
    ]
    assert data.transit_preview[0].length_m == 2.0


@pytest.mark.anyio
async def test_save_entities_api_persists_overrides(monkeypatch):
    from models import DXFEntityOverridesRequest, EntityMarkOverride

    captured = {}

    class FakePathManager:
        def save_entity_overrides(self, name, overrides):
            captured["name"] = name
            captured["overrides"] = overrides
            return len(overrides)

    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await save_path_entity_overrides(
        "field.dxf",
        DXFEntityOverridesRequest(
            overrides=[EntityMarkOverride(entity_id="A1", is_mark=False)]
        ),
    )

    assert data.name == "field.dxf"
    assert data.num_overrides == 1
    assert captured["overrides"] == {"A1": False}


def test_path_manager_saves_extension_config(tmp_path):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    mgr = PathManager(str(tmp_path))
    saved = mgr.save_extension_config("field.dxf", True, 0.4, 0.6)

    # per_line omitted (None) on a fresh file resolves to the default, now True
    # (per-entity extensions on every edge/corner).
    assert saved == {
        "enabled": True,
        "pre_extension_m": 0.4,
        "aft_extension_m": 0.6,
        "per_line": True,
    }
    assert mgr.load_extension_config("field.dxf") == saved

    # per-line round-trips through save/load
    saved_pl = mgr.save_extension_config("field.dxf", True, 0.5, 0.5, per_line=True)
    assert saved_pl["per_line"] is True
    assert mgr.load_extension_config("field.dxf")["per_line"] is True

    # sticky: omitting per_line (None) preserves the saved value, doesn't reset it
    saved_omit = mgr.save_extension_config("field.dxf", True, 0.3, 0.3)
    assert saved_omit["per_line"] is True
    # explicit False still turns it off
    saved_off = mgr.save_extension_config("field.dxf", True, 0.3, 0.3, per_line=False)
    assert saved_off["per_line"] is False


def test_plan_preflight_rejects_oversized_mission(tmp_path):
    """A drawing that densifies past the waypoint cap fails fast with the
    actionable message (before the slow full plan), and honours coarser spacing
    so the operator can follow the message's own advice."""
    ezdxf = pytest.importorskip("ezdxf")
    doc = ezdxf.new()
    doc.header["$INSUNITS"] = 6  # metres
    doc.modelspace().add_line((0, 0), (600, 0))  # 600 m marking line
    (tmp_path / "long.dxf").write_bytes(b"")  # placeholder so saveas path exists
    doc.saveas(str(tmp_path / "long.dxf"))

    mgr = PathManager(str(tmp_path))
    # 5 cm spacing → ~12000 waypoints. The cap is 100k (raised so a 2.4 km road
    # survey can load at all), so pin an explicit lower cap here — the point of
    # this test is the preflight guard, not the value of the default.
    with pytest.raises(ValueError, match="Too many waypoints"):
        mgr.plan_path("long.dxf", summary_only=True, max_waypoints=10000)
    # Coarser 20 cm spacing → ~3000 waypoints → the guard does not fire.
    result = mgr.plan_path("long.dxf", summary_only=True, line_spacing=0.20,
                           max_waypoints=10000)
    assert result["num_waypoints"] > 0
    # And the raised default lets the 12000-waypoint mission through.
    assert mgr.plan_path("long.dxf", summary_only=True)["num_waypoints"] > 10000


@pytest.mark.anyio
async def test_extension_config_api_round_trip(tmp_path, monkeypatch):
    from models import PathExtensionConfig

    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    mgr = PathManager(str(tmp_path))

    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", mgr)

    saved = await save_path_extensions(
        "field.dxf",
        PathExtensionConfig(
            enabled=True,
            pre_extension_m=0.4,
            aft_extension_m=0.6,
        ),
    )
    loaded = await get_path_extensions("field.dxf")

    assert saved.name == "field.dxf"
    assert saved.enabled is True
    assert loaded.pre_extension_m == 0.4
    assert loaded.aft_extension_m == 0.6


def test_path_manager_plan_applies_entity_overrides(tmp_path, monkeypatch):
    from path_engine.core import DXFEntity, PathSegment, SegmentType

    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")
    captured = {}

    class FakeEngine:
        def __init__(self, **kwargs):
            pass

        def plan_dxf_entities(self, entities, **kwargs):
            captured["is_mark"] = [ent.is_mark() for ent in entities]
            return SimpleNamespace(
                num_waypoints=2,
                segments=[
                    PathSegment(
                        segment_type=SegmentType.TRANSIT,
                        points=[(0.0, 0.0), (1.0, 0.0)],
                    )
                ],
                total_mark_length=0.0,
                total_transit_length=1.0,
                total_length=1.0,
                alignment_metadata={},
                planning_metadata={},
                merged_waypoints=[(0.0, 0.0), (1.0, 0.0)],
                spray_flags=[False, False],
            )

    class FakeValidator:
        def __init__(self, *args, **kwargs):
            pass

        def validate_or_raise(self, plan):
            return []

    import path_engine.engine as engine_module
    import path_engine.validator as validator_module

    monkeypatch.setattr(engine_module, "PathEngine", FakeEngine)
    monkeypatch.setattr(validator_module, "PathValidator", FakeValidator)

    mgr = PathManager(str(tmp_path))
    mgr.parse_dxf = lambda *args, **kwargs: [
        DXFEntity(entity_type="LINE", layer="MARKINGS", entity_id="A1")
    ]
    mgr.save_entity_overrides("field.dxf", {"A1": False})

    result = mgr.plan_path("field.dxf")

    assert captured["is_mark"] == [False]
    assert result["spray_flags"] == [False, False]
    assert result["planning_metadata"]["entity_overrides"]["num_overrides"] == 1


def test_entity_override_never_beats_ignore_classification():
    from path_engine.core import DXFEntity

    ent = DXFEntity(entity_type="LINE", layer="MARKINGS", entity_id="A1")
    ent.is_mark_override = True

    # Override decides MARK/TRANSIT when the entity is plannable...
    assert ent.classify() == "mark"
    ent.is_mark_override = False
    assert ent.classify() == "transit"
    assert ent.is_mark() is False

    # ...but an 'ignore' layer mapping always wins: ignored entities must
    # never be planned, overridden or not.
    ent.is_mark_override = True
    assert ent.classify({"MARKINGS": "ignore"}) == "ignore"


def test_entity_extension_directions_use_analytic_arc_tangents():
    from path_engine.core import DXFEntity
    from path_engine.planners.extensions import entity_extension_directions

    # Quarter arc 0°→90°. CCW tangent at θ in (north, east) is (cos θ, -sin θ):
    # preview directions must come from the same formula the planner uses,
    # not from finite differences of densified preview points.
    ent = DXFEntity(
        entity_type="ARC",
        layer="MARKINGS",
        entity_id="A1",
        geometry={"center": (0.0, 0.0), "radius": 1.0,
                  "start_angle": 0.0, "end_angle": 90.0},
    )
    dirs = entity_extension_directions(ent, [])
    assert dirs is not None
    start_dir, end_dir = dirs
    assert start_dir == pytest.approx((1.0, 0.0))
    assert end_dir == pytest.approx((0.0, -1.0))

    # POINT entities get no extension, matching the planner guard.
    point = DXFEntity(entity_type="POINT", layer="MARKINGS", entity_id="P1",
                      geometry={"position": (0.0, 0.0)})
    assert entity_extension_directions(point, [(0.0, 0.0), (0.0, 0.0)]) is None


def test_parse_dxf_cache_returns_pristine_copies(tmp_path, monkeypatch):
    from path_engine.core import DXFEntity
    import path_engine.parsers.dxf_parser as dxf_parser_module

    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")
    calls = {"n": 0}

    def fake_parse(filepath, unit_scale=None, layer_mapping=None):
        calls["n"] += 1
        return [DXFEntity(entity_type="LINE", layer="MARKINGS", entity_id="A1")]

    monkeypatch.setattr(dxf_parser_module, "parse_dxf", fake_parse)

    mgr = PathManager(str(tmp_path))
    first = mgr.parse_dxf(str(mission_file))
    PathManager.apply_entity_overrides(first, {"A1": False})
    second = mgr.parse_dxf(str(mission_file))

    assert calls["n"] == 1  # second call served from cache
    # Override stamped on the first copy must not leak into later parses.
    assert first[0].is_mark_override is False
    assert second[0].is_mark_override is None


def test_path_plan_request_extension_defaults_are_safe():
    req = PathPlanRequest(source="soccer_field_penalty_area.dxf")

    assert req.enable_path_extensions is False
    assert req.pre_extension_m == 0.5
    assert req.aft_extension_m == 0.5


def test_path_plan_request_rejects_negative_extensions():
    from pydantic import ValidationError

    for field in ("pre_extension_m", "aft_extension_m"):
        with pytest.raises(ValidationError):
            PathPlanRequest(source="soccer_field_penalty_area.dxf", **{field: -0.1})


@pytest.mark.anyio
async def test_plan_api_does_not_pass_extension_flags(monkeypatch):
    captured = {}

    class FakePathManager:
        def plan_path(self, source, summary_only=False, **kwargs):
            captured["source"] = source
            captured["summary_only"] = summary_only
            captured["kwargs"] = kwargs
            return {
                "source": source,
                "num_waypoints": 2,
                "num_segments": 1,
                "mark_length_m": 1.0,
                "transit_length_m": 0.0,
                "total_length_m": 1.0,
                "segments": [],
                "merged_waypoints": [(0.0, 0.0), (1.0, 0.0)],
                "spray_flags": [True, True],
                "alignment_metadata": {},
                "warnings": [],
            }

    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    req = PathPlanRequest(
        source="soccer_field_penalty_area.dxf",
        enable_path_extensions=True,
        pre_extension_m=0.25,
        aft_extension_m=0.75,
    )

    data = await plan_path(req)

    assert data.source == "soccer_field_penalty_area.dxf"
    assert "enable_path_extensions" not in captured["kwargs"]
    assert "pre_extension_m" not in captured["kwargs"]
    assert "aft_extension_m" not in captured["kwargs"]
    # Old clients setting the deprecated fields must be told they were ignored.
    assert any("deprecated" in w.lower() for w in (data.warnings or []))


def test_path_manager_passes_extension_flags_to_engine(tmp_path, monkeypatch):
    mission_file = tmp_path / "line.csv"
    mission_file.write_text("north,east\n0,0\n1,0\n", encoding="utf-8")
    captured = {}

    class FakeEngine:
        def __init__(self, **kwargs):
            captured["engine_kwargs"] = kwargs

        def plan_file(self, filepath, **kwargs):
            captured["plan_filepath"] = filepath
            captured["plan_kwargs"] = kwargs
            return SimpleNamespace(
                num_waypoints=2,
                segments=[],
                total_mark_length=1.0,
                total_transit_length=0.0,
                total_length=1.0,
                alignment_metadata={},
                merged_waypoints=[(0.0, 0.0), (1.0, 0.0)],
                spray_flags=[True, True],
            )

    class FakeValidator:
        def __init__(self, *args, **kwargs):
            pass

        def validate(self, plan):
            return []

        def validate_or_raise(self, plan):
            return []

    import path_engine.engine as engine_module
    import path_engine.validator as validator_module

    monkeypatch.setattr(engine_module, "PathEngine", FakeEngine)
    monkeypatch.setattr(validator_module, "PathValidator", FakeValidator)

    mgr = PathManager(str(tmp_path))
    result = mgr.plan_path(
        "line.csv",
        enable_path_extensions=True,
        pre_extension_m=0.25,
        aft_extension_m=0.75,
    )

    assert result["source"] == "line.csv"
    assert captured["engine_kwargs"]["enable_path_extensions"] is True
    assert captured["engine_kwargs"]["pre_extension_m"] == 0.25
    assert captured["engine_kwargs"]["aft_extension_m"] == 0.75

@pytest.mark.anyio
async def test_plan_api_dxf_ref_points():
    if main.path_mgr is None:
        main.path_mgr = PathManager(main.MISSION_DIR)

    req = PathPlanRequest(
        source="soccer_field_penalty_area.dxf",
        include_waypoints=True,
        line_spacing=0.1,
        transit_spacing=0.3,
        marking_speed=0.4,
        transit_speed=0.6,
        close_loop=True,
        ref_points=[
            RefPoint(dxf_x=0.0, dxf_y=0.0, lat=13.0, lon=80.0),
            # 13.00009039 is exactly 10.000 m north of 13.0 (WGS84 geodesic), matching
            # the 10.0-unit DXF baseline. The old value (13.0001) is 11.063 m — an 10.6%
            # disagreement with the drawing. That used to "pass" only because the fit
            # absorbed it into scale and stretched the geometry 10.6% while reporting
            # rmse~0. The fit is rigid now, so inconsistent refs are correctly rejected
            # by the RMSE gate and the fixture has to be physically real.
            RefPoint(dxf_x=10.0, dxf_y=0.0, lat=13.0000903910, lon=80.0),
        ],
        origin_gps=[13.072066, 80.261956]
    )
    
    data = await plan_path(req)
    
    assert data.source == "soccer_field_penalty_area.dxf"
    assert data.num_waypoints > 0
    assert data.num_segments > 0
    assert len(data.merged_waypoints) > 0
    assert len(data.spray_flags) > 0
    assert data.alignment_metadata is not None
    assert data.warnings is not None
    
    meta = data.alignment_metadata
    assert meta["method"] == "least_squares"
    assert "scale" in meta
    assert "rmse" in meta
    assert "residuals" in meta
    assert len(meta["residuals"]) == 2

@pytest.mark.anyio
async def test_plan_api_dxf_simple_rotation():
    if main.path_mgr is None:
        main.path_mgr = PathManager(main.MISSION_DIR)

    req = PathPlanRequest(
        source="soccer_field_penalty_area.dxf",
        include_waypoints=True,
        line_spacing=0.1,
        transit_spacing=0.3,
        marking_speed=0.4,
        transit_speed=0.6,
        close_loop=False,
        rotation_deg=45.0,
        origin_gps=[13.072066, 80.261956]
    )
    
    data = await plan_path(req)
    
    assert data.source == "soccer_field_penalty_area.dxf"
    assert data.num_waypoints > 0
    assert data.num_segments > 0
    assert len(data.merged_waypoints) > 0
    assert len(data.spray_flags) > 0
    assert data.alignment_metadata is not None
    assert data.alignment_metadata["method"] == "gps_origin"
    assert data.alignment_metadata["rotation_deg"] == 45.0

@pytest.mark.anyio
async def test_plan_api_single_point_heading():
    if main.path_mgr is None:
        main.path_mgr = PathManager(main.MISSION_DIR)

    # Gap B: one ref point + heading is now a valid alignment mode (was a
    # silent fall-back to gps_origin about (0,0)).
    req = PathPlanRequest(
        source="soccer_field_penalty_area.dxf",
        include_waypoints=True,
        line_spacing=0.1,
        transit_spacing=0.3,
        marking_speed=0.4,
        transit_speed=0.6,
        rotation_deg=30.0,
        ref_points=[
            RefPoint(dxf_x=5.0, dxf_y=5.0, lat=13.0001, lon=80.0001),
        ],
        origin_gps=[13.072066, 80.261956]
    )

    data = await plan_path(req)
    meta = data.alignment_metadata
    assert meta["method"] == "single_point_heading"
    assert meta["rotation_deg"] == 30.0
    assert meta["scale"] == 1.0
    # Clicked point is offset from origin_gps, so translation must be non-zero.
    assert meta["offset_n"] != 0.0 or meta["offset_e"] != 0.0
    assert meta["rmse"] == 0.0

@pytest.mark.anyio
async def test_plan_api_coincident_ref_points():
    if main.path_mgr is None:
        main.path_mgr = PathManager(main.MISSION_DIR)

    # Two coincident ref points should fail inside dxf_to_ned_affine and raise 422 HTTP exception
    req = PathPlanRequest(
        source="soccer_field_penalty_area.dxf",
        include_waypoints=True,
        line_spacing=0.1,
        transit_spacing=0.3,
        marking_speed=0.4,
        transit_speed=0.6,
        ref_points=[
            RefPoint(dxf_x=0.0, dxf_y=0.0, lat=13.0, lon=80.0),
            RefPoint(dxf_x=0.0, dxf_y=0.0, lat=13.0, lon=80.0),
        ],
        origin_gps=[13.072066, 80.261956]
    )
    
    with pytest.raises(HTTPException) as exc:
        await plan_path(req)
    assert exc.value.status_code == 422
    assert "coincident" in exc.value.detail


# ── Gap A: unit-scale frame consistency ───────────────────────────────────────

def test_affine_scale_is_unity_when_ref_points_share_metric_frame():
    """Gap A regression — affine math contract.

    The engine no longer applies unit scaling to ref points: they arrive pre-scaled
    in local-NED metres from the /entities preview. Metric-frame refs → fitted scale
    ≈ 1.0; raw cm vs metric NED → grossly wrong scale.

    Note this now exercises ``estimate_fit_scale``, the free-scale *diagnostic*. The
    applied transform (``dxf_to_ned_affine``) is rigid and always reports scale=1.0,
    so it can no longer be used to detect a unit mismatch — that is exactly the point
    of locking it (geometry is never resized to fit bad refs). The diagnostic is what
    ``_assert_alignment_scale`` gates on.
    """
    from path_engine.ned import dxf_to_ned_affine, estimate_fit_scale

    unit_scale = 0.01  # cm → m
    # Two ref points 1000 DXF units (= 10 m) apart along DXF-x.
    raw_dxf = [(0.0, 0.0), (0.0, 1000.0)]  # stored as (dxf_y, dxf_x)
    ned = [(0.0, 0.0), (0.0, 10.0)]        # 10 m east

    # Unit mismatch: raw cm points vs metric NED → diagnostic scale ~0.01.
    raw_scale = estimate_fit_scale(raw_dxf, ned)
    assert abs(raw_scale - 1.0) > 0.5  # demonstrably off → gate rejects

    # Correct: scale ref points into metres first.
    metric_dxf = [(p[0] * unit_scale, p[1] * unit_scale) for p in raw_dxf]
    fixed_scale = estimate_fit_scale(metric_dxf, ned)
    assert abs(fixed_scale - 1.0) < 1e-6

    # And regardless of refs, the APPLIED transform never rescales geometry.
    assert dxf_to_ned_affine(raw_dxf, ned)[0] == 1.0
    assert dxf_to_ned_affine(metric_dxf, ned)[0] == 1.0


# ── Gap D: RMSE quality gate ───────────────────────────────────────────────────

@pytest.mark.anyio
async def test_plan_api_rmse_gate_rejects_high_residual(monkeypatch):
    """Gap D: alignment RMSE above RMSE_MAX returns 422 and stages nothing."""
    from config import RMSE_MAX

    class FakePathManager:
        def plan_path(self, source, summary_only=False, **kwargs):
            return {
                "source": source,
                "num_waypoints": 2,
                "num_segments": 1,
                "mark_length_m": 1.0,
                "transit_length_m": 0.0,
                "total_length_m": 1.0,
                "segments": [],
                "merged_waypoints": [(0.0, 0.0), (1.0, 0.0)],
                "spray_flags": [True, True],
                "alignment_metadata": {
                    "method": "least_squares",
                    "rmse": RMSE_MAX + 0.10,
                    "scale": 1.0,
                    "origin_gps": (13.072066, 80.261956),
                },
                "warnings": [],
            }

    monkeypatch.setattr(main, "path_mgr", FakePathManager())
    req = PathPlanRequest(source="soccer_field_penalty_area.dxf")

    with pytest.raises(HTTPException) as exc:
        await plan_path(req)
    assert exc.value.status_code == 422
    assert "rmse" in exc.value.detail.lower()


def _fake_pm_with_alignment(meta: dict):
    """Path-manager stub returning a fixed alignment_metadata for gate tests."""
    class FakePathManager:
        def plan_path(self, source, summary_only=False, **kwargs):
            return {
                "source": source,
                "num_waypoints": 2,
                "num_segments": 1,
                "mark_length_m": 1.0,
                "transit_length_m": 0.0,
                "total_length_m": 1.0,
                "segments": [],
                "merged_waypoints": [(0.0, 0.0), (1.0, 0.0)],
                "spray_flags": [True, True],
                "alignment_metadata": meta,
                "warnings": [],
            }
    return FakePathManager()


@pytest.mark.anyio
async def test_plan_api_scale_gate_rejects_double_scaled_refpoints(monkeypatch):
    """Unit/frame mismatch → scale ~100; RMSE≈0 so only the scale gate rejects."""
    meta = {
        "method": "least_squares",
        "scale": 100.0,
        "rmse": 0.0,
        "origin_gps": (13.072066, 80.261956),
    }
    monkeypatch.setattr(main, "path_mgr", _fake_pm_with_alignment(meta))
    req = PathPlanRequest(source="soccer_field_penalty_area.dxf")

    with pytest.raises(HTTPException) as exc:
        await plan_path(req)
    assert exc.value.status_code == 422
    assert "scale" in exc.value.detail.lower()


@pytest.mark.anyio
async def test_plan_api_scale_gate_accepts_unity_scale(monkeypatch):
    """Healthy fit (scale≈1.0) passes the scale gate."""
    meta = {
        "method": "least_squares",
        "scale": 1.002,
        "rmse": 0.0,
        "origin_gps": (13.072066, 80.261956),
    }
    monkeypatch.setattr(main, "path_mgr", _fake_pm_with_alignment(meta))
    req = PathPlanRequest(source="soccer_field_penalty_area.dxf", include_waypoints=False)

    result = await plan_path(req)
    assert result is not None


@pytest.mark.anyio
async def test_plan_api_rejects_placeholder_origin_gps(monkeypatch):
    """Known GCS fixture (13.0, 80.0) must not stage a surveyed mission."""
    meta = {
        "method": "least_squares",
        "scale": 1.0,
        "rmse": 0.0,
        "origin_gps": (13.0, 80.0),
    }
    monkeypatch.setattr(main, "path_mgr", _fake_pm_with_alignment(meta))
    req = PathPlanRequest(source="soccer_field_penalty_area.dxf", include_waypoints=False)

    with pytest.raises(HTTPException) as exc:
        await plan_path(req)
    assert exc.value.status_code == 422
    assert "placeholder" in exc.value.detail.lower()


def test_engine_does_not_rescale_metric_ref_points():
    """Engine: metric refs feed affine unchanged; ref_unit_scale removed."""
    import inspect
    import math
    from path_engine.engine import PathEngine
    from path_engine.core import PathSegment, SegmentType

    sig = inspect.signature(PathEngine._plan_from_segments)
    assert "ref_unit_scale" not in sig.parameters

    engine = PathEngine()
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (0.0, 10.0)],
        speed=0.4,
        segment_id="seg0",
        source_entity="LINE_0",
    )
    ref_dxf = [(0.0, 0.0), (0.0, 10.0)]
    ref_gps = [(13.0, 80.0), (13.0, 80.0 + 10.0 / (111320.0 * math.cos(math.radians(13.0))))]

    plan = engine._plan_from_segments(
        [seg],
        origin_gps=(13.072066, 80.261956),
        ref_points_dxf=ref_dxf,
        ref_points_gps=ref_gps,
    )
    scale = (plan.alignment_metadata or {}).get("scale")
    assert scale is not None
    assert abs(scale - 1.0) < 0.02


# ── Gaps C & E: staging + load-to-controller round-trip ────────────────────────

@pytest.mark.anyio
async def test_plan_then_load_to_controller_round_trip(monkeypatch, tmp_path):
    """Gaps C/E: plan stages the aligned mission; load-to-controller pushes the
    identical waypoints to the controller and forwards the GPS anchor."""
    import routes.path as path_routes
    from models import LoadMissionRequest

    staging = tmp_path / "staging"
    monkeypatch.setattr(path_routes, "STAGING_DIR", str(staging))

    waypoints = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]

    class FakePathManager:
        def plan_path(self, source, summary_only=False, **kwargs):
            return {
                "source": source,
                "num_waypoints": len(waypoints),
                "num_segments": 1,
                "mark_length_m": 2.0,
                "transit_length_m": 0.0,
                "total_length_m": 2.0,
                "segments": [],
                "merged_waypoints": list(waypoints),
                "spray_flags": [True, True, True],
                "alignment_metadata": {
                    "method": "least_squares",
                    "rmse": 0.004,
                    "rotation_deg": 12.0,
                    "scale": 1.0,
                    "origin_gps": (13.072066, 80.261956),
                },
                "warnings": [],
            }

    class FakeController:
        def __init__(self):
            self.loaded = None
            self.state = MissionState.IDLE

        def load_path(self, points, name=None, spray_flags=None, **kwargs):
            self.loaded = (list(points), name, spray_flags, kwargs)

    class _OriginNode:
        """Healthy EKF-origin verdict. /load-to-controller fails closed on a
        surveyed mission without one, so the test has to declare which world
        it is in rather than inherit whatever main.ros_node happens to be."""

        def get_origin_health(self):
            return {"status": "OK", "trusted": True, "delta_m": 0.019,
                    "detail": "origin agrees", "threshold_m": 0.30}

    fake_ctrl = FakeController()
    monkeypatch.setattr(main, "path_mgr", FakePathManager())
    monkeypatch.setattr(main, "offboard_ctrl", fake_ctrl)
    monkeypatch.setattr(main, "ros_node", _OriginNode(), raising=False)

    req = PathPlanRequest(source="soccer_field_penalty_area.dxf")
    data = await plan_path(req)

    assert data.mission_summary is not None
    mid = data.mission_summary.mission_id
    assert data.mission_summary.estimated_paint_l > 0
    assert data.mission_summary.estimated_runtime_s > 0
    assert (staging / f"{mid}.json").is_file()

    resp = await path_routes.load_mission_to_controller(LoadMissionRequest(mission_id=mid))
    assert resp["status"] == "success"
    assert resp["num_waypoints"] == len(waypoints)
    assert resp["anchor_loaded"] is True
    # Controller received the exact aligned waypoints.
    assert fake_ctrl.loaded[0] == waypoints
    assert fake_ctrl.loaded[3]["placement_mode"] == "GPS_SURVEYED"
    assert fake_ctrl.loaded[3]["origin_gps"] == (13.072066, 80.261956)
    assert fake_ctrl.loaded[3]["is_staged"] is True
    assert resp["placement_mode"] == "GPS_SURVEYED"
    import routes.path as path_routes
    from models import LoadMissionRequest

    monkeypatch.setattr(path_routes, "STAGING_DIR", str(tmp_path / "staging"))

    class FakeController:
        state = MissionState.IDLE

        def load_path(self, points, name=None, spray_flags=None):
            pass

    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    with pytest.raises(HTTPException) as exc:
        await path_routes.load_mission_to_controller(
            LoadMissionRequest(mission_id="stg_does_not_exist")
        )
    assert exc.value.status_code == 404


@pytest.mark.anyio
async def test_load_to_controller_rejects_while_running(monkeypatch, tmp_path):
    """Field-safety: loading a new mission while one is RUNNING returns 409
    and never reads the staged artifact."""
    import routes.path as path_routes
    from models import LoadMissionRequest

    monkeypatch.setattr(path_routes, "STAGING_DIR", str(tmp_path / "staging"))

    class FakeController:
        state = MissionState.RUNNING

        def load_path(self, points, name=None, spray_flags=None):
            raise AssertionError("load_path must not be called while RUNNING")

    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    with pytest.raises(HTTPException) as exc:
        await path_routes.load_mission_to_controller(
            LoadMissionRequest(mission_id="stg_anything")
        )
    assert exc.value.status_code == 409


# ── Extension-aware auto-origin (server flow) ───────────────────────────────

def _write_line_dxf(path):
    """Author a single LINE heading north (DXF (0,0)->(0,5)) in metres.

    The DXF parser maps DXF x->east, y->north, so a DXF segment ending at
    (0,5) heads due north in NED. $INSUNITS=6 makes unit_scale 1.0.
    """
    import ezdxf

    doc = ezdxf.new()
    doc.header["$INSUNITS"] = 6  # metres → unit_scale 1.0
    msp = doc.modelspace()
    msp.add_line((0.0, 0.0), (0.0, 5.0))
    doc.saveas(str(path))


def test_load_path_auto_origin_anchors_pre_point_at_rover(tmp_path):
    """With extensions enabled + auto_origin, the executed path's first point
    (the PRE run-up point) lands on the rover pose, and disabling auto_origin
    keeps the historical drawing-origin anchoring (PRE point behind rover)."""
    try:
        import ezdxf  # noqa: F401
    except ImportError:
        pytest.skip("ezdxf not installed")

    dxf = tmp_path / "line.dxf"
    _write_line_dxf(dxf)

    mgr = PathManager(str(tmp_path))
    mgr.save_extension_config("line.dxf", True, 0.5, 0.5)

    rover = (10.0, 20.0)

    # auto_origin=True → anchor the first driven waypoint (PRE point) at rover.
    pts_anchored = mgr.load_path("line.dxf", origin=rover, start_position=rover, auto_origin=True)
    assert len(pts_anchored) >= 2
    assert abs(pts_anchored[0][0] - rover[0]) < 1e-3
    assert abs(pts_anchored[0][1] - rover[1]) < 1e-3

    # auto_origin=False → drawing-origin anchoring: PRE point is 0.5 m behind
    # the rover (south) since the DXF line starts heading north from (0,0).
    pts_drawing = mgr.load_path("line.dxf", origin=rover, start_position=rover, auto_origin=False)
    assert abs(pts_drawing[0][0] - (rover[0] - 0.5)) < 1e-3

    # Same geometry, just translated: identical waypoint count.
    assert len(pts_anchored) == len(pts_drawing)


def test_load_path_executes_with_extension_config(tmp_path):
    """Regression for the prerequisite fix: the executed path honors the saved
    extension config (PRE/AFT legs present), not a bare default engine."""
    try:
        import ezdxf  # noqa: F401
    except ImportError:
        pytest.skip("ezdxf not installed")

    dxf = tmp_path / "line.dxf"
    _write_line_dxf(dxf)
    mgr = PathManager(str(tmp_path))

    # Extensions OFF → no run-up; line starts at (0,0) (modulo a small spray
    # lead-in of ~0.035 m from latency compensation).
    mgr.save_extension_config("line.dxf", False, 0.5, 0.5)
    pts_off = mgr.load_path("line.dxf")
    assert abs(pts_off[0][0]) < 0.1  # near drawing origin (spray lead-in only)

    # Extensions ON → PRE leg prepended; path now starts 0.5 m before (0,0).
    mgr.save_extension_config("line.dxf", True, 0.5, 0.5)
    pts_on = mgr.load_path("line.dxf")
    assert pts_on[0][0] < -0.4  # PRE run-up point south of origin
    assert len(pts_on) > len(pts_off)


# ── Entity ordering tests ────────────────────────────────────────────────────

def _make_line_entity(entity_id: str, is_mark_callable=None):
    """Build a SimpleNamespace mimicking DXFEntity for ordering tests."""
    if is_mark_callable is None:
        is_mark_callable = lambda: True
    return SimpleNamespace(
        entity_id=entity_id,
        entity_type="LINE",
        layer="MARKINGS",
        color=7,
        geometry={"start": (0.0, 0.0), "end": (1.0, 0.0)},
        is_mark=is_mark_callable,
        classify=lambda: "mark",
    )


def test_apply_entity_order_empty_saved_order_returns_parser_order():
    from routes.path import _apply_entity_order

    entities = [_make_line_entity("A1"), _make_line_entity("A2"), _make_line_entity("A3")]
    ordered = _apply_entity_order(entities, [])
    assert [e.entity_id for e in ordered] == ["A1", "A2", "A3"]


def test_apply_entity_order_reorders():
    from routes.path import _apply_entity_order

    entities = [_make_line_entity("A1"), _make_line_entity("A2"), _make_line_entity("A3")]
    ordered = _apply_entity_order(entities, ["A3", "A1", "A2"])
    assert [e.entity_id for e in ordered] == ["A3", "A1", "A2"]


def test_apply_entity_order_appends_new_entities_at_end():
    from routes.path import _apply_entity_order

    entities = [_make_line_entity("A1"), _make_line_entity("A2"), _make_line_entity("A3")]
    ordered = _apply_entity_order(entities, ["A3"])
    assert [e.entity_id for e in ordered] == ["A3", "A1", "A2"]


def test_apply_entity_order_ignores_missing_saved_ids():
    from routes.path import _apply_entity_order

    entities = [_make_line_entity("A1"), _make_line_entity("A2")]
    ordered = _apply_entity_order(entities, ["A3", "A1"])
    assert [e.entity_id for e in ordered] == ["A1", "A2"]


def test_apply_entity_order_no_duplicates():
    from routes.path import _apply_entity_order

    entities = [_make_line_entity("A1"), _make_line_entity("A2")]
    ordered = _apply_entity_order(entities, ["A1", "A2", "A1"])
    assert [e.entity_id for e in ordered] == ["A1", "A2"]


def test_path_manager_entity_order_round_trip(tmp_path):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")
    mgr = PathManager(str(tmp_path))

    # Mock parse_dxf to return known entities
    mgr.parse_dxf = lambda *args, **kwargs: [
        _make_line_entity("A1"),
        _make_line_entity("A2"),
        _make_line_entity("A3"),
    ]

    # No saved order yet
    assert mgr.load_entity_order("field.dxf") == []

    # Save order
    mgr.save_entity_order("field.dxf", ["A3", "A1", "A2"])
    loaded = mgr.load_entity_order("field.dxf")
    assert loaded == ["A3", "A1", "A2"]

    # Overwrite
    mgr.save_entity_order("field.dxf", ["A2", "A3", "A1"])
    loaded = mgr.load_entity_order("field.dxf")
    assert loaded == ["A2", "A3", "A1"]


def test_path_manager_entity_order_cleared_on_delete(tmp_path):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")
    mgr = PathManager(str(tmp_path))
    mgr.parse_dxf = lambda *args, **kwargs: [_make_line_entity("A1")]

    mgr.save_entity_order("field.dxf", ["A1"])
    assert mgr.load_entity_order("field.dxf") == ["A1"]

    mgr.delete_file("field.dxf")
    assert mgr.load_entity_order("field.dxf") == []


def test_path_manager_entity_order_cleared_on_upload(tmp_path):
    from path_manager import validate_upload

    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")
    mgr = PathManager(str(tmp_path))
    mgr.parse_dxf = lambda *args, **kwargs: [_make_line_entity("A1")]

    mgr.save_entity_order("field.dxf", ["A1"])
    assert mgr.load_entity_order("field.dxf") == ["A1"]

    # Upload a new version of the same file (via save_uploaded)
    mgr.save_uploaded("field.dxf", b"0\nEOF\n")
    assert mgr.load_entity_order("field.dxf") == []


def test_path_manager_load_entity_order_handles_malformed_json(tmp_path):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")
    mgr = PathManager(str(tmp_path))

    # Write malformed sidecar
    sidecar = tmp_path / ".field.dxf.entity_order.json"
    sidecar.write_text("not valid json", encoding="utf-8")

    assert mgr.load_entity_order("field.dxf") == []


def test_path_manager_load_entity_order_handles_invalid_structure(tmp_path):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")
    mgr = PathManager(str(tmp_path))

    sidecar = tmp_path / ".field.dxf.entity_order.json"
    sidecar.write_text('{"entity_order": {"A1": 1}}', encoding="utf-8")

    assert mgr.load_entity_order("field.dxf") == []


@pytest.mark.anyio
async def test_entities_api_get_returns_parser_order_with_order_index(tmp_path, monkeypatch):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [
                _make_line_entity("A1"),
                _make_line_entity("A2"),
                _make_line_entity("A3"),
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {"enabled": False, "pre_extension_m": 0.5, "aft_extension_m": 0.5}

        def load_entity_order(self, filename):
            return []

    import routes.path as path_route
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("field.dxf")

    assert data.num_entities == 3
    assert [e.entity_id for e in data.entities] == ["A1", "A2", "A3"]
    assert [e.order_index for e in data.entities] == [0, 1, 2]


@pytest.mark.anyio
async def test_entities_api_get_returns_saved_order(tmp_path, monkeypatch):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [
                _make_line_entity("A1"),
                _make_line_entity("A2"),
                _make_line_entity("A3"),
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {"enabled": False, "pre_extension_m": 0.5, "aft_extension_m": 0.5}

        def load_entity_order(self, filename):
            return ["A3", "A1", "A2"]

    import routes.path as path_route
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("field.dxf")

    assert [e.entity_id for e in data.entities] == ["A3", "A1", "A2"]
    assert [e.order_index for e in data.entities] == [0, 1, 2]


@pytest.mark.anyio
async def test_entities_api_transit_follows_reordered_mark_entities(tmp_path, monkeypatch):
    """Transit preview must connect MARK entities in reordered sequence."""
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    def line(entity_id, start, end):
        return SimpleNamespace(
            entity_id=entity_id,
            entity_type="LINE",
            layer="MARKINGS",
            color=7,
            geometry={"start": start, "end": end},
            is_mark=lambda: True,
        )

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [
                line("A", (0.0, 0.0), (1.0, 0.0)),   # first in parser order
                line("B", (2.0, 0.0), (3.0, 0.0)),   # second
                line("C", (4.0, 0.0), (5.0, 0.0)),   # third
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {"enabled": False, "pre_extension_m": 0.5, "aft_extension_m": 0.5}

        def load_entity_order(self, filename):
            return ["C", "A", "B"]  # saved order: C first

    import routes.path as path_route
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("field.dxf")

    # Entity order must be C, A, B
    assert [e.entity_id for e in data.entities] == ["C", "A", "B"]

    # Transit must connect C→A and A→B (the consecutively ordered MARK entities)
    assert len(data.transit_preview) == 2
    assert data.transit_preview[0].from_entity_id == "C"
    assert data.transit_preview[0].to_entity_id == "A"
    assert data.transit_preview[1].from_entity_id == "A"
    assert data.transit_preview[1].to_entity_id == "B"


@pytest.mark.anyio
async def test_entities_api_new_entities_appended_after_saved_order(tmp_path, monkeypatch):
    """DXF changes: saved order has old ID, DXF has extra entity not in saved order."""
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [
                _make_line_entity("B"),  # new entity not in saved order
                _make_line_entity("A"),  # in saved order
                _make_line_entity("C"),  # new entity not in saved order
            ]

        def load_entity_overrides(self, filename):
            return {}

        def load_extension_config(self, filename):
            return {"enabled": False, "pre_extension_m": 0.5, "aft_extension_m": 0.5}

        def load_entity_order(self, filename):
            return ["A", "OLD"]  # OLD no longer exists

    import routes.path as path_route
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("field.dxf")

    # A first (from saved order), then B, C appended in parser order
    assert [e.entity_id for e in data.entities] == ["A", "B", "C"]


@pytest.mark.anyio
async def test_update_entity_order_endpoint_saves_full_order(tmp_path, monkeypatch):
    from models import EntityOrderUpdateRequest, EntityOrderUpdateResponse

    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    saved = None

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [_make_line_entity("A1"), _make_line_entity("A2")]

        def save_entity_order(self, filename, entity_order):
            nonlocal saved
            saved = (filename, entity_order)

    import routes.path as path_route
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    from routes.path import update_entity_order

    req = EntityOrderUpdateRequest(entity_order=["A2", "A1"])
    resp = await update_entity_order("field.dxf", req)

    assert saved == ("field.dxf", ["A2", "A1"])
    assert resp.name == "field.dxf"
    assert resp.num_entities == 2
    assert resp.entity_order == ["A2", "A1"]


@pytest.mark.anyio
async def test_update_entity_order_rejects_duplicate_ids(tmp_path, monkeypatch):
    from models import EntityOrderUpdateRequest

    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [_make_line_entity("A1"), _make_line_entity("A2")]

    import routes.path as path_route
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    from routes.path import update_entity_order

    req = EntityOrderUpdateRequest(entity_order=["A1", "A2", "A1"])
    with pytest.raises(HTTPException) as exc:
        await update_entity_order("field.dxf", req)
    assert exc.value.status_code == 422
    assert "Duplicate" in exc.value.detail


@pytest.mark.anyio
async def test_update_entity_order_rejects_unknown_ids(tmp_path, monkeypatch):
    from models import EntityOrderUpdateRequest

    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [_make_line_entity("A1"), _make_line_entity("A2")]

    import routes.path as path_route
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    from routes.path import update_entity_order

    req = EntityOrderUpdateRequest(entity_order=["A1", "UNKNOWN"])
    with pytest.raises(HTTPException) as exc:
        await update_entity_order("field.dxf", req)
    assert exc.value.status_code == 422
    assert "Unknown" in exc.value.detail


@pytest.mark.anyio
async def test_update_entity_order_rejects_missing_ids(tmp_path, monkeypatch):
    from models import EntityOrderUpdateRequest

    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            return [_make_line_entity("A1"), _make_line_entity("A2")]

    import routes.path as path_route
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    from routes.path import update_entity_order

    req = EntityOrderUpdateRequest(entity_order=["A1"])
    with pytest.raises(HTTPException) as exc:
        await update_entity_order("field.dxf", req)
    assert exc.value.status_code == 422
    assert "Missing" in exc.value.detail


@pytest.mark.anyio
async def test_update_entity_order_rejects_non_dxf(tmp_path, monkeypatch):
    from models import EntityOrderUpdateRequest

    csv_file = tmp_path / "field.csv"
    csv_file.write_text("0,0\n", encoding="utf-8")

    import routes.path as path_route
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))

    from routes.path import update_entity_order

    req = EntityOrderUpdateRequest(entity_order=["A1"])
    with pytest.raises(HTTPException) as exc:
        await update_entity_order("field.csv", req)
    assert exc.value.status_code == 415


@pytest.mark.anyio
async def test_update_entity_order_rejects_missing_path(tmp_path, monkeypatch):
    from models import EntityOrderUpdateRequest

    import routes.path as path_route
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))

    from routes.path import update_entity_order

    req = EntityOrderUpdateRequest(entity_order=["A1"])
    with pytest.raises(HTTPException) as exc:
        await update_entity_order("nonexistent.dxf", req)
    assert exc.value.status_code == 404


def test_preview_spray_flags_match_executed_path_with_extensions(tmp_path):
    """Preview spray-flag length matches the executed path so the controller
    does not fall back to spray-OFF on a length mismatch."""
    try:
        import ezdxf  # noqa: F401
    except ImportError:
        pytest.skip("ezdxf not installed")

    dxf = tmp_path / "line.dxf"
    _write_line_dxf(dxf)
    mgr = PathManager(str(tmp_path))
    mgr.save_extension_config("line.dxf", True, 0.5, 0.5)

    preview = mgr.preview_path("line.dxf")
    executed = mgr.load_path("line.dxf")
    assert preview.num_points == len(executed)
    # PRE/AFT extensions are TRANSIT → spray OFF at the ends.
    assert preview.waypoints[0].spray is False
    assert preview.waypoints[-1].spray is False


# ── Survey control points in the preview ──────────────────────────────────────
# The pipeline works in local NED and previously emitted ONLY that, so a client
# could not draw the raw surveyed shots: the arc fit and corner fillet
# deliberately move geometry off the measurements, and re-projecting NED back to
# lat/lon goes through the client's own projection rather than the source data.

_SURVEY_HDR = ("Name,Code,Code description,Easting,Northing,Elevation,Longitude,"
               "Latitude,Lateral RMS,Solution status,Samples,PDOP,CS name")


def _survey_row(name, code, lon, lat):
    return (f"{name},{code},Point,1204636.0,1243756.0,9.9,{lon:.8f},{lat:.8f},"
            f"0.017,FIX,1,1.8,WGS 84 / Tamil Nadu + EGM96 height")


def _write_survey(tmp_path, n=6, name="curve.csv"):
    """A gently curving chain of n surveyed shots ~0.6 m apart."""
    import math as _m
    lat0, lon0 = 13.07206142, 80.26193876
    deg = 1.0 / 111320.0
    rows = []
    for i in range(n):
        a = _m.radians(i * 12.0)
        rows.append(_survey_row(i + 1, "P",
                                lon0 + (0.6 * i) * deg * 0.9,
                                lat0 + (0.6 * (1 - _m.cos(a))) * deg))
    (tmp_path / name).write_text("\n".join([_SURVEY_HDR, *rows]) + "\n")
    return name


def test_preview_emits_control_points_with_source_latlon(tmp_path):
    name = _write_survey(tmp_path)
    mgr = PathManager(str(tmp_path))
    res = mgr.preview_path(name)

    assert len(res.control_points) == 6
    assert all(c.lat is not None and c.lon is not None for c in res.control_points)
    # Exactly the values in the file, not re-projected from NED.
    import csv as _csv
    with open(tmp_path / name) as f:
        src = list(_csv.DictReader(f))
    assert [c.lat for c in res.control_points] == [float(r["Latitude"]) for r in src]
    assert [c.lon for c in res.control_points] == [float(r["Longitude"]) for r in src]
    assert [c.name for c in res.control_points] == [r["Name"] for r in src]
    assert all(c.code == "P" for c in res.control_points)


def test_control_points_share_the_waypoint_ned_frame(tmp_path):
    """north/east must be directly overlayable on the waypoints — i.e. the same
    projection about the same geo_origin the preview reports."""
    from path_engine.parsers.georef import metres_per_degree

    name = _write_survey(tmp_path)
    res = PathManager(str(tmp_path)).preview_path(name)
    lat0, lon0 = res.geo_origin
    mdeg_n, mdeg_e = metres_per_degree(lat0)
    for c in res.control_points:
        assert abs((c.lat - lat0) * mdeg_n - c.north) < 1e-6
        assert abs((c.lon - lon0) * mdeg_e - c.east) < 1e-6


def test_control_points_are_the_raw_shots_not_the_fitted_path(tmp_path):
    """They must NOT move when the fitted geometry does — that is what makes
    them useful as a separate map layer."""
    name = _write_survey(tmp_path)
    mgr = PathManager(str(tmp_path))
    before = mgr.preview_path(name).control_points

    mgr.save_line_config(name, 5.0)          # fillet on: waypoints change
    mgr._preview_cache.clear()
    after = mgr.preview_path(name)
    assert [(c.north, c.east, c.lat, c.lon) for c in after.control_points] == \
           [(c.north, c.east, c.lat, c.lon) for c in before]


def test_non_survey_sources_emit_no_control_points(tmp_path):
    mgr = PathManager(str(tmp_path))
    assert mgr.preview_path("builtin:square_2x2").control_points == []
    # Legacy headerless NED CSV has no surveyed provenance either.
    (tmp_path / "legacy.csv").write_text("0,0\n1,0\n2,0\n")
    assert mgr.preview_path("legacy.csv").control_points == []


def test_grid_only_survey_csv_emits_control_points_without_latlon(tmp_path):
    """A Northing/Easting export has no geographic anchor; the points still come
    through so they can be drawn, but lat/lon is null rather than invented."""
    hdr = "Name,Code,Northing,Easting"
    rows = [f"{i+1},L_1,{1243756.0 + i * 0.6:.3f},{1204636.0 + i * 0.2:.3f}"
            for i in range(4)]
    (tmp_path / "grid.csv").write_text("\n".join([hdr, *rows]) + "\n")
    res = PathManager(str(tmp_path)).preview_path("grid.csv")
    assert len(res.control_points) == 4
    assert res.geo_origin is None
    assert all(c.lat is None and c.lon is None for c in res.control_points)
    assert all(c.name is not None for c in res.control_points)


# ── preview / plan / load must produce the SAME geometry for a survey CSV ─────
# The arc fit is automatic for a survey CSV, and preview and load both apply it.
# PathPlanRequest.fit_arcs used to default to False, and /api/path/plan forwards
# it unconditionally — so an ordinary plan call sent an EXPLICIT "off" and staged
# straight chords for the very file the map was drawing as arcs. None now means
# "not specified"; only a caller who asks gets an override.

def _curving_survey(tmp_path, name="sweep.csv"):
    """A survey CSV whose points lie on a real arc, so fitting is observable."""
    import math as _m
    hdr = ("Name,Code,Code description,Easting,Northing,Elevation,Longitude,"
           "Latitude,Lateral RMS,Solution status,Samples,PDOP,CS name")
    lat0, lon0, deg = 13.07206142, 80.26193876, 1.0 / 111320.0
    rows = []
    for i in range(12):
        a = _m.radians(i * 8.0)
        rows.append(
            f"{i+1},P,Point,1204636.0,1243756.0,9.9,"
            f"{lon0 + 6.0 * _m.sin(a) * deg:.8f},"
            f"{lat0 + 6.0 * (1 - _m.cos(a)) * deg:.8f},"
            f"0.017,FIX,1,1.8,WGS 84 / Tamil Nadu + EGM96 height"
        )
    (tmp_path / name).write_text("\n".join([hdr, *rows]) + "\n")
    return name


def test_plan_matches_preview_for_a_survey_csv(tmp_path):
    """The WYSIWYG contract: what the map draws is what gets planned."""
    name = _curving_survey(tmp_path)
    mgr = PathManager(str(tmp_path))
    preview = [(w.north, w.east) for w in mgr.preview_path(name).waypoints]
    planned = mgr.plan_path(name, summary_only=False)["merged_waypoints"]
    assert planned == preview


def test_plan_defaults_do_not_disable_the_survey_arc_fit(tmp_path):
    """Regression: forwarding the request model's unset arc-fit fields (all None)
    must be identical to not passing them at all."""
    name = _curving_survey(tmp_path)
    mgr = PathManager(str(tmp_path))
    bare = mgr.plan_path(name, summary_only=False)["merged_waypoints"]
    forwarded = mgr.plan_path(
        name, summary_only=False,
        fit_arcs=None, fit_arcs_rms_m=None,
        fit_arcs_corner_deg=None, fit_arcs_max_dev_m=None,
    )["merged_waypoints"]
    assert forwarded == bare
    # And the fit really is doing something, or the assertion above is vacuous.
    chords = mgr.plan_path(name, summary_only=False, fit_arcs=False)["merged_waypoints"]
    assert chords != bare


def test_explicit_fit_arcs_false_still_wins(tmp_path):
    """None means 'unset', but a caller who genuinely wants chords still gets them."""
    name = _curving_survey(tmp_path)
    mgr = PathManager(str(tmp_path))
    off = mgr.plan_path(name, summary_only=False, fit_arcs=False)["merged_waypoints"]
    on = mgr.plan_path(name, summary_only=False, fit_arcs=True)["merged_waypoints"]
    assert off != on


def test_plan_honours_the_line_config_sidecar_like_preview_does(tmp_path):
    """fit_arcs_max_dev_m defaulted to 0.15 in the request model and was always
    forwarded, overriding the per-file sidecar that preview and load read."""
    name = _curving_survey(tmp_path)
    mgr = PathManager(str(tmp_path))
    mgr.save_line_config(name, 0.0, fit_arcs_max_dev_m=0.002)   # too tight to fit
    mgr._preview_cache.clear()
    preview = [(w.north, w.east) for w in mgr.preview_path(name).waypoints]
    planned = mgr.plan_path(name, summary_only=False)["merged_waypoints"]
    assert planned == preview
