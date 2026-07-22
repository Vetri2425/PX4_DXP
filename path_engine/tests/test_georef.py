"""Georeferenced-DXF detection + projection to local ENU metres."""
import math

from path_engine.core import DXFEntity
from path_engine.parsers.georef import (
    detect_and_project,
    looks_geographic,
    metres_per_degree,
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


def _vincenty_inverse(a_pt, b_pt):
    """WGS84 ellipsoidal geodesic distance, metres. Independent ground truth.

    Deliberately NOT the local-radii formula the implementation uses — this is
    Vincenty's iterative inverse solution, a different formulation, so it cannot
    reproduce an error in the code under test.

    This replaced a haversine reference that used the semi-major axis as a
    SPHERICAL radius. That reference shared the exact wrong assumption as the
    pre-2026-07-22 projection code (semi-major axis on the north axis), so the
    test passed while the projection carried a +0.62 % north scale error.
    A test whose ground truth mirrors the implementation's bug proves nothing.
    """
    a = 6378137.0
    f = 1.0 / 298.257223563
    b = (1.0 - f) * a
    lat1, lon1 = math.radians(a_pt[0]), math.radians(a_pt[1])
    lat2, lon2 = math.radians(b_pt[0]), math.radians(b_pt[1])
    L = lon2 - lon1
    U1 = math.atan((1 - f) * math.tan(lat1))
    U2 = math.atan((1 - f) * math.tan(lat2))
    sinU1, cosU1 = math.sin(U1), math.cos(U1)
    sinU2, cosU2 = math.sin(U2), math.cos(U2)
    lam = L
    for _ in range(200):
        sin_lam, cos_lam = math.sin(lam), math.cos(lam)
        sin_sigma = math.hypot(cosU2 * sin_lam,
                               cosU1 * sinU2 - sinU1 * cosU2 * cos_lam)
        if sin_sigma == 0:
            return 0.0  # coincident
        cos_sigma = sinU1 * sinU2 + cosU1 * cosU2 * cos_lam
        sigma = math.atan2(sin_sigma, cos_sigma)
        sin_alpha = cosU1 * cosU2 * sin_lam / sin_sigma
        cos2_alpha = 1 - sin_alpha ** 2
        cos_2sigma_m = (cos_sigma - 2 * sinU1 * sinU2 / cos2_alpha) if cos2_alpha != 0 else 0.0
        C = f / 16 * cos2_alpha * (4 + f * (4 - 3 * cos2_alpha))
        lam_prev = lam
        lam = L + (1 - C) * f * sin_alpha * (
            sigma + C * sin_sigma * (cos_2sigma_m + C * cos_sigma *
                                     (-1 + 2 * cos_2sigma_m ** 2)))
        if abs(lam - lam_prev) < 1e-12:
            break
    u2 = cos2_alpha * (a * a - b * b) / (b * b)
    A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)))
    B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)))
    d_sigma = B * sin_sigma * (cos_2sigma_m + B / 4 * (
        cos_sigma * (-1 + 2 * cos_2sigma_m ** 2)
        - B / 6 * cos_2sigma_m * (-3 + 4 * sin_sigma ** 2) * (-1 + 4 * cos_2sigma_m ** 2)))
    return b * A * (sigma - d_sigma)


def test_side_lengths_match_ellipsoidal_geodesic_ground_truth():
    """Projected side lengths match true WGS84 geodesic distances."""
    ent = _poly(GEO_SQUARE)
    detect_and_project([ent])
    proj = ent.geometry["vertices"]

    for i in range(len(GEO_SQUARE)):
        truth = _vincenty_inverse(GEO_SQUARE[i], GEO_SQUARE[(i + 1) % len(GEO_SQUARE)])
        got = math.dist(proj[i], proj[(i + 1) % len(proj)])
        assert abs(got - truth) < 0.001, (i, got, truth)  # sub-mm


def test_north_axis_uses_meridional_radius_not_semi_major_axis():
    """Regression: the north scale must be M(lat), not the semi-major axis.

    Using `a` for north is a +0.62 % scale error at 13 deg latitude — invisible
    on a 2 m square, +62 cm over a 100 m road tangent.
    """
    mdeg_n, mdeg_e = metres_per_degree(13.07207058)

    # Textbook WGS84 values at the equator.
    eq_n, eq_e = metres_per_degree(0.0)
    assert abs(eq_n - 110574.0) < 1.0, eq_n
    assert abs(eq_e - 111319.5) < 1.0, eq_e

    # The wrong value the code used to use.
    wrong = math.radians(1.0) * 6378137.0
    assert abs(mdeg_n - wrong) > 600.0, "north scale still uses the semi-major axis"

    # North grows toward the poles, east shrinks to zero.
    assert metres_per_degree(60.0)[0] > mdeg_n
    assert metres_per_degree(89.9)[1] < 300.0


def test_matches_field_survey_ground_truth():
    """Ground truth from a real RTK survey of a line we have driven.

    Emlid Reach RS3, RTK FIX, 2026-07-18, points 3 and 4 of test_line_2
    (code L_2). The surveyor's own projected grid (WGS 84 / Tamil Nadu) reads
    2.3264 m for this line; the true geodesic is 2.3255 m. The pre-fix code
    produced 2.3399 m.
    """
    P3 = (13.07208106, 80.26195346)
    P4 = (13.07206010, 80.26195184)
    ent = _poly([P3, P4])
    detect_and_project([ent])
    got = math.dist(*ent.geometry["vertices"])

    truth = _vincenty_inverse(P3, P4)
    assert abs(got - truth) < 0.0005, (got, truth)
    assert abs(got - 2.3255) < 0.002, got          # matches the survey
    assert abs(got - 2.3399) > 0.010, "reproduced the pre-fix scale error"
