"""Georeferenced-DXF detection + projection to local ENU metres."""
import math

from path_engine.core import DXFEntity
from path_engine.parsers.georef import (
    detect_and_project,
    looks_geographic,
)

# The real test_1.dxf square (lat, lon), a ~2 m square near Chennai.
GEO_SQUARE = [
    (13.07206392, 80.26194124),
    (13.07208325, 80.26194283),
    (13.07208256, 80.26196034),
    (13.07206129, 80.26195891),
]


def _poly(verts, etype="LWPOLYLINE"):
    return DXFEntity(entity_type=etype, layer="Lines", entity_id="PL0",
                     geometry={"vertices": list(verts), "bulges": [0.0] * len(verts),
                               "closed": True}, unit_scale=1.0)


# --- detection -------------------------------------------------------------

def test_detects_latlon_square():
    ok, reason = looks_geographic(GEO_SQUARE)
    assert ok, reason


def test_metric_square_near_origin_not_geographic():
    ok, _ = looks_geographic([(0, 0), (0, 2), (2, 2), (2, 0)])
    assert not ok


def test_metric_square_far_from_origin_not_geographic():
    # A 5 m square placed at (50, 50) m — coords in lat/lon RANGE but extent
    # is metric-sized, so the offset/extent ratio stays low.
    ok, _ = looks_geographic([(50, 50), (50, 55), (55, 55), (55, 50)])
    assert not ok


def test_projected_utm_metres_not_geographic():
    # UTM-style coordinates (hundreds of thousands of metres) are out of range.
    ok, _ = looks_geographic([(4467000.0, 300000.0), (4467002.0, 300002.0)])
    assert not ok


# --- projection ------------------------------------------------------------

def test_projects_latlon_square_to_metres():
    ent = _poly(GEO_SQUARE)
    origin = detect_and_project([ent])
    assert origin is not None
    lat0, lon0 = origin
    assert 13.0 < lat0 < 13.1 and 80.2 < lon0 < 80.3
    assert ent.geo_origin == origin

    verts = ent.geometry["vertices"]
    # Sides come back ~2 m (survey was hand-drawn, so allow slack).
    sides = [math.dist(verts[i], verts[(i + 1) % len(verts)]) for i in range(len(verts))]
    for s in sides:
        assert 1.8 < s < 2.5, sides
    # Centred on the origin: coordinates are single-digit metres, not degrees.
    assert all(abs(c) < 5.0 for v in verts for c in v)


def test_metric_dxf_untouched():
    verts = [(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0)]
    ent = _poly(verts)
    origin = detect_and_project([ent])
    assert origin is None
    assert ent.geometry["vertices"] == verts  # byte-for-byte unchanged
    assert ent.geo_origin is None


def test_projects_line_point_circle_keys():
    line = DXFEntity(entity_type="LINE", layer="L", entity_id="L0",
                     geometry={"start": GEO_SQUARE[0], "end": GEO_SQUARE[1]}, unit_scale=1.0)
    point = DXFEntity(entity_type="POINT", layer="P", entity_id="P0",
                      geometry={"position": GEO_SQUARE[2]}, unit_scale=1.0)
    circle = DXFEntity(entity_type="CIRCLE", layer="C", entity_id="C0",
                       geometry={"center": GEO_SQUARE[0], "radius": 0.00002}, unit_scale=1.0)
    origin = detect_and_project([line, point, circle])
    assert origin is not None
    # All absolute coords now single-digit metres.
    assert all(abs(c) < 5.0 for c in line.geometry["start"])
    assert all(abs(c) < 5.0 for c in point.geometry["position"])
    assert all(abs(c) < 5.0 for c in circle.geometry["center"])
    # Radius scaled from degrees to metres (0.00002 deg ~ 2.2 m).
    assert 1.5 < circle.geometry["radius"] < 3.0


def test_side_lengths_match_haversine_ground_truth():
    """Projected side lengths match the true great-circle distances."""
    ent = _poly(GEO_SQUARE)
    detect_and_project([ent])
    proj = ent.geometry["vertices"]

    def haversine(a, b):
        R = 6378137.0
        dlat = math.radians(b[0] - a[0])
        dlon = math.radians(b[1] - a[1])
        la1, la2 = math.radians(a[0]), math.radians(b[0])
        h = math.sin(dlat / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin(dlon / 2) ** 2
        return 2 * R * math.asin(math.sqrt(h))

    for i in range(len(GEO_SQUARE)):
        truth = haversine(GEO_SQUARE[i], GEO_SQUARE[(i + 1) % len(GEO_SQUARE)])
        got = math.dist(proj[i], proj[(i + 1) % len(proj)])
        assert abs(got - truth) < 0.001, (i, got, truth)  # sub-mm
