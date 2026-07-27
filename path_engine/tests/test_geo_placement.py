"""A georeferenced DXF auto-places at its own coordinates — no alignment survey.

georef projects lat/lon to local ENU metres centred on the DXF's geo_origin;
this checks that planning then defaults GPS placement to that same origin, so the
mission round-trips to the exact lat/lon it was drawn at.
"""
import math

from path_engine.core import DXFEntity
from path_engine.engine import PathEngine


def _geo_square_entity():
    """A closed lat/lon square, then georef-projected to metres (as parse_dxf
    would leave it): local coords + geo_origin stamped."""
    from path_engine.parsers.georef import detect_and_project
    verts = [(13.07206392, 80.26194124), (13.07208325, 80.26194283),
             (13.07208256, 80.26196034), (13.07206129, 80.26195891)]
    ent = DXFEntity(entity_type="LWPOLYLINE", layer="Lines", entity_id="PL0",
                    geometry={"vertices": list(verts), "bulges": [0.0] * 4,
                              "closed": True}, unit_scale=1.0)
    origin = detect_and_project([ent])
    return ent, origin


def test_geo_origin_defaults_when_no_explicit_gps():
    ent, origin = _geo_square_entity()
    assert PathEngine._geo_origin_gps([ent], None, None) == origin


def test_explicit_origin_gps_wins():
    ent, _ = _geo_square_entity()
    explicit = (10.0, 20.0)
    assert PathEngine._geo_origin_gps([ent], explicit, None) == explicit


def test_ref_points_suppress_geo_default():
    ent, _ = _geo_square_entity()
    # ref-point alignment given → do NOT override with geo_origin
    assert PathEngine._geo_origin_gps([ent], None, [(1.0, 2.0), (3.0, 4.0)]) is None


def test_metric_dxf_has_no_geo_default():
    metric = DXFEntity(entity_type="LWPOLYLINE", layer="Lines", entity_id="M0",
                       geometry={"vertices": [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)],
                                 "bulges": [0.0] * 3, "closed": True}, unit_scale=1.0)
    assert PathEngine._geo_origin_gps([metric], None, None) is None


def test_planned_mission_carries_geo_origin_and_roundtrips():
    ent, origin = _geo_square_entity()
    plan = PathEngine().plan_dxf_entities([ent])
    meta = plan.alignment_metadata or {}
    assert meta.get("method") == "gps_origin"
    assert meta.get("origin_gps") == origin

    # local (0,0) sits at the geo_origin; a waypoint's true lat/lon is
    # origin + its ENU offset. Reconstruct and confirm it lands back inside the
    # original lat/lon square (~2 m across).
    R = 6378137.0
    lat0, lon0 = origin
    lats, lons = [], []
    for n, e in plan.merged_waypoints:
        lats.append(lat0 + math.degrees(n / R))
        lons.append(lon0 + math.degrees(e / (R * math.cos(math.radians(lat0)))))
    assert 13.0720 < min(lats) and max(lats) < 13.0721, (min(lats), max(lats))
    assert 80.2619 < min(lons) and max(lons) < 80.2620, (min(lons), max(lons))
