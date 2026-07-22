"""Survey-export CSV ingest (named-header GNSS point files)."""

import math

import pytest

from path_engine.core import PathSegment, SegmentType
from path_engine.engine import PathEngine
from path_engine.parsers.survey_csv import looks_like_survey_csv, read_survey_csv

# Two real surveyed points from test_line_2.csv (Emlid Reach RS3, RTK FIX,
# 2026-07-18, code L_2). The full export has 42 columns; these are the ones
# that carry meaning. True WGS84 geodesic between them is 2.3255 m.
_HEADER = ("Name,Code,Code description,Easting,Northing,Elevation,Longitude,"
           "Latitude,Lateral RMS,Solution status,Samples,PDOP,CS name")
_ROW3 = ("3,L_2,Line,1204637.765,1243758.291,9.890,80.26195346,13.07208106,"
         "0.017,FIX,1,1.3,WGS 84 / Tamil Nadu + EGM96 height")
_ROW4 = ("4,L_2,Line,1204637.606,1243755.970,9.886,80.26195184,13.07206010,"
         "0.017,FIX,1,1.7,WGS 84 / Tamil Nadu + EGM96 height")

TRUE_GEODESIC_M = 2.3255


def _write(tmp_path, *lines, name="survey.csv"):
    p = tmp_path / name
    p.write_text("\n".join(lines) + "\n")
    return str(p)


# --- detection --------------------------------------------------------------

def test_detects_named_header_survey_csv(tmp_path):
    assert looks_like_survey_csv(_write(tmp_path, _HEADER, _ROW3, _ROW4))


def test_legacy_headerless_ned_csv_is_not_a_survey_csv(tmp_path):
    """The old 2-/6-column NED format must keep its existing code path."""
    legacy = _write(tmp_path, "0.0,0.0", "1.0,0.0", "1.0,1.0", name="legacy.csv")
    assert not looks_like_survey_csv(legacy)


def test_single_axis_header_is_not_enough(tmp_path):
    """A header naming only one axis is ambiguous — reject rather than guess."""
    f = _write(tmp_path, "Name,Northing,Elevation", "1,1243758.291,9.89")
    assert not looks_like_survey_csv(f)


# --- geometry ---------------------------------------------------------------

def test_latlon_is_preferred_and_matches_the_geodesic(tmp_path):
    """Lat/lon -> local ENU gives true ground distance, which is what the EKF frame is."""
    res = read_survey_csv(_write(tmp_path, _HEADER, _ROW3, _ROW4))

    assert res.coordinate_source == "latlon"
    assert len(res.segments) == 1
    assert abs(res.segments[0].length - TRUE_GEODESIC_M) < 0.001


def test_grid_fallback_when_no_latlon_and_it_warns(tmp_path):
    """Grid coords carry the projection scale factor, so they are a fallback."""
    header = "Name,Code,Easting,Northing"
    f = _write(tmp_path, header, "3,L_2,1204637.765,1243758.291",
               "4,L_2,1204637.606,1243755.970")
    res = read_survey_csv(f)

    assert res.coordinate_source == "grid"
    assert res.geo_origin is None
    assert any("scale factor" in w for w in res.warnings)
    # The Tamil Nadu grid reads +0.042% long vs ground on this line.
    assert res.segments[0].length > TRUE_GEODESIC_M
    assert abs(res.segments[0].length - 2.3264) < 0.001


def test_geo_origin_is_the_survey_centroid(tmp_path):
    res = read_survey_csv(_write(tmp_path, _HEADER, _ROW3, _ROW4))
    lat0, lon0 = res.geo_origin
    assert abs(lat0 - (13.07208106 + 13.07206010) / 2) < 1e-9
    assert abs(lon0 - (80.26195346 + 80.26195184) / 2) < 1e-9


# --- structure: Code groups, Name orders ------------------------------------

def test_code_column_groups_points_into_separate_lines(tmp_path):
    """`Code` is the field-to-finish feature code: it says which line a point is on."""
    f = _write(
        tmp_path,
        "Name,Code,Latitude,Longitude",
        "1,L_1,13.07206390,80.26194110",
        "2,L_1,13.07206906,80.26194825",
        "1,L_2,13.07208106,80.26195346",
        "2,L_2,13.07206010,80.26195184",
    )
    res = read_survey_csv(f)

    assert len(res.segments) == 2
    assert [s.metadata["survey_code"] for s in res.segments] == ["L_1", "L_2"]
    assert all(len(s.points) == 2 for s in res.segments)


def test_name_column_orders_points_numerically_not_lexically(tmp_path):
    """Point 10 must come after point 9, not after point 1."""
    rows = ["Name,Code,Latitude,Longitude"]
    for i in (1, 10, 2):
        rows.append(f"{i},L_1,13.0720{600 + i:04d},80.2619{4000 + i:04d}")
    res = read_survey_csv(_write(tmp_path, *rows))

    assert res.segments[0].metadata["survey_names"] == ["1", "2", "10"]


def test_rows_out_of_file_order_are_still_sorted_by_name(tmp_path):
    res = read_survey_csv(_write(tmp_path, _HEADER, _ROW4, _ROW3))  # 4 before 3
    assert res.segments[0].metadata["survey_names"] == ["3", "4"]


def test_missing_code_column_yields_one_segment_in_file_order(tmp_path):
    f = _write(tmp_path, "Name,Latitude,Longitude",
               "1,13.07206390,80.26194110", "2,13.07206906,80.26194825")
    res = read_survey_csv(f)
    assert len(res.segments) == 1
    assert res.segments[0].metadata["survey_code"] is None


# --- every surveyed point is a declared control point -----------------------

def test_every_surveyed_point_is_a_control_point(tmp_path):
    res = read_survey_csv(_write(tmp_path, _HEADER, _ROW3, _ROW4))
    assert res.segments[0].metadata["control_indices"] == [0, 1]


def test_control_declaration_survives_the_full_planner(tmp_path):
    res = read_survey_csv(_write(tmp_path, _HEADER, _ROW3, _ROW4))
    plan = PathEngine(mark_spacing=0.05, optimize_order=False).plan_segments(res.segments)

    assert len(plan.merged_waypoints) > 40, "should densify"
    assert sum(plan.must_hit) == 2, "exactly the two surveyed points"
    flagged = [p for p, m in zip(plan.merged_waypoints, plan.must_hit) if m]
    assert math.dist(flagged[0], flagged[1]) == pytest.approx(TRUE_GEODESIC_M, abs=0.001)


# --- quality columns are surfaced, never silently dropped -------------------

def test_single_epoch_points_are_warned_about(tmp_path):
    res = read_survey_csv(_write(tmp_path, _HEADER, _ROW3, _ROW4))
    assert any("single-epoch" in w for w in res.warnings)


def test_non_fix_points_are_kept_but_warned_by_default(tmp_path):
    row = _ROW4.replace(",FIX,", ",FLOAT,")
    res = read_survey_csv(_write(tmp_path, _HEADER, _ROW3, row))
    assert len(res.segments[0].points) == 2
    assert any("not RTK FIX" in w for w in res.warnings)


def test_require_fix_drops_non_fix_points(tmp_path):
    row = _ROW4.replace(",FIX,", ",FLOAT,")
    res = read_survey_csv(_write(tmp_path, _HEADER, _ROW3, row), require_fix=True)
    assert res.point_count == 1          # points IMPORTED, i.e. after filtering
    assert len(res.segments[0].points) == 1
    assert any("dropped 1 point" in w for w in res.warnings)


def test_require_fix_raises_rather_than_returning_an_empty_plan(tmp_path):
    a = _ROW3.replace(",FIX,", ",SINGLE,")
    b = _ROW4.replace(",FIX,", ",FLOAT,")
    with pytest.raises(ValueError, match="no points left"):
        read_survey_csv(_write(tmp_path, _HEADER, a, b), require_fix=True)


def test_lateral_rms_threshold_warns(tmp_path):
    res = read_survey_csv(_write(tmp_path, _HEADER, _ROW3, _ROW4),
                          max_lateral_rms_m=0.010)
    assert any("lateral RMS" in w for w in res.warnings)


def test_cs_name_is_captured(tmp_path):
    res = read_survey_csv(_write(tmp_path, _HEADER, _ROW3, _ROW4))
    assert res.cs_name == "WGS 84 / Tamil Nadu + EGM96 height"


# --- failure modes ----------------------------------------------------------

def test_rejects_a_file_with_no_usable_coordinates(tmp_path):
    f = _write(tmp_path, "Name,Code,Latitude,Longitude", "1,L_1,,", "2,L_1,,")
    with pytest.raises(ValueError, match="no rows with usable coordinates"):
        read_survey_csv(f)


def test_rejects_a_headerless_file(tmp_path):
    with pytest.raises(ValueError, match="no survey CSV header"):
        read_survey_csv(_write(tmp_path, "0.0,0.0", "1.0,0.0"))


def test_single_point_code_is_kept_and_warned(tmp_path):
    f = _write(tmp_path, "Name,Code,Latitude,Longitude",
               "1,L_1,13.07206390,80.26194110",
               "1,L_9,13.07208106,80.26195346")
    res = read_survey_csv(f)
    assert len(res.segments) == 2
    assert any("only 1 point" in w for w in res.warnings)
