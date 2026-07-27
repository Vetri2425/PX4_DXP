#!/usr/bin/env python3
"""Unit tests for point mission ingest."""

from __future__ import annotations

import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from point_ingest import (
    GPS_SURVEYED_FRAME,
    gps_point_mission_parse_payload,
    parse_dxf_point_entities,
    parse_point_csv_text,
    parse_point_gps_csv_text,
    points_from_staged_dict,
    points_to_staged_dict,
)


def test_csv_two_column_legacy():
    pts = parse_point_csv_text("1.0,2.0\n3.0,4.0\n")
    assert len(pts) == 2
    assert pts[0].north_m == 1.0
    assert pts[0].east_m == 2.0
    assert pts[0].dwell_s == 2.0
    assert pts[0].mark is True


def test_csv_with_dwell_legacy():
    pts = parse_point_csv_text("1.0,2.0,3.5\n")
    assert pts[0].dwell_s == 3.5
    assert pts[0].mark is True


def test_csv_header_two_column():
    pts = parse_point_csv_text("north,east\n1.0,2.0\n3.0,4.0\n")
    assert len(pts) == 2
    assert pts[0].dwell_s == 2.0
    assert pts[0].mark is True


def test_csv_header_with_dwell():
    pts = parse_point_csv_text("north,east,dwell_s\n1.0,2.0,3.5\n")
    assert pts[0].dwell_s == 3.5


def test_csv_header_with_mark_false():
    pts = parse_point_csv_text(
        "north,east,dwell_s,mark\n1.0,2.0,0.0,false\n2.0,3.0,1.5,true\n"
    )
    assert pts[0].mark is False
    assert pts[0].dwell_s == 0.0
    assert pts[1].mark is True
    assert pts[1].dwell_s == 1.5


def test_csv_mark_defaults_true_without_column():
    pts = parse_point_csv_text("north,east,dwell_s\n1.0,2.0,2.5\n")
    assert pts[0].mark is True


def test_staged_dict_round_trip_includes_mark():
    pts = parse_point_csv_text("north,east,dwell_s,mark\n1.0,2.0,2.0,false\n")
    staged = points_to_staged_dict(pts)
    assert staged[0]["mark"] is False
    restored = points_from_staged_dict(staged)
    assert restored[0].mark is False


def test_unknown_header_columns_rejected():
    try:
        parse_point_csv_text("north,east,foo\n1,2,3\n")
        assert False
    except ValueError as exc:
        assert "unknown CSV header columns" in str(exc)


def test_headerless_extra_column_rejected():
    try:
        parse_point_csv_text("1.0,2.0,3.0,false\n")
        assert False
    except ValueError as exc:
        assert "too many columns for headerless CSV" in str(exc)


def test_malformed_mark_rejected():
    try:
        parse_point_csv_text("north,east,dwell_s,mark\n1.0,2.0,2.0,maybe\n")
        assert False
    except ValueError as exc:
        assert "mark must be a boolean" in str(exc)


def test_malformed_row_rejected():
    try:
        parse_point_csv_text("bad,row\n")
        assert False
    except ValueError:
        pass


def test_empty_file_rejected():
    try:
        parse_point_csv_text("# only comments\n")
        assert False
    except ValueError:
        pass


def test_zero_dwell_mark_true_rejected():
    try:
        parse_point_csv_text("1.0,2.0,0.0\n")
        assert False
    except ValueError as exc:
        assert "dwell_s must be > 0 when mark=true" in str(exc)


def test_zero_dwell_mark_false_allowed_with_header():
    pts = parse_point_csv_text("north,east,dwell_s,mark\n1.0,2.0,0.0,false\n")
    assert pts[0].dwell_s == 0.0
    assert pts[0].mark is False


def test_negative_dwell_rejected():
    try:
        parse_point_csv_text("1.0,2.0,-1.0\n")
        assert False
    except ValueError as exc:
        assert "dwell_s must be > 0 when mark=true" in str(exc)


def test_non_finite_dwell_rejected():
    try:
        parse_point_csv_text("1.0,2.0,nan\n")
        assert False
    except ValueError as exc:
        assert "dwell_s must be finite" in str(exc)


def test_max_dwell_exceeded_rejected():
    try:
        parse_point_csv_text("1.0,2.0,61.0\n", max_dwell_s=60.0)
        assert False
    except ValueError as exc:
        assert "exceeds maximum 60.0" in str(exc)


def test_default_dwell_above_max_rejected():
    try:
        parse_point_csv_text("1.0,2.0\n", default_dwell_s=70.0, max_dwell_s=60.0)
        assert False
    except ValueError as exc:
        assert "exceeds maximum 60.0" in str(exc)


def test_dxf_point_entities():
    ent = types.SimpleNamespace(
        entity_type="POINT",
        entity_id="p1",
        geometry={"position": (1.5, 2.5)},
    )
    pts = parse_dxf_point_entities([ent], default_dwell_s=1.5)
    assert len(pts) == 1
    assert pts[0].dwell_s == 1.5
    assert pts[0].mark is True


def test_gps_csv_valid_two_points():
    csv_text = (
        "lat,lon,dwell_s,mark\n"
        "13.0,80.0,2.0,true\n"
        "13.00005,80.00005,3.0,true\n"
    )
    parsed = parse_point_gps_csv_text(csv_text)
    assert parsed.anchor_lat == 13.0
    assert parsed.anchor_lon == 80.0
    assert len(parsed.points) == 2
    assert abs(parsed.points[0].north_m) < 1e-6
    assert abs(parsed.points[0].east_m) < 1e-6
    assert math.hypot(parsed.points[1].north_m, parsed.points[1].east_m) > 0.5


def test_gps_csv_payload_uses_gps_surveyed_frame():
    parsed = parse_point_gps_csv_text("lat,lon\n13.0,80.0\n")
    payload = gps_point_mission_parse_payload(parsed)
    assert payload["point_source_frame"] == GPS_SURVEYED_FRAME
    assert payload["anchor"] == {"lat": 13.0, "lon": 80.0}
    assert payload["num_points"] == 1


def test_gps_csv_default_dwell_and_mark():
    parsed = parse_point_gps_csv_text("lat,lon\n13.0,80.0\n")
    assert parsed.points[0].dwell_s == 2.0
    assert parsed.points[0].mark is True


def test_gps_csv_mark_column_without_dwell():
    parsed = parse_point_gps_csv_text("lat,lon,mark\n13.0,80.0,false\n")
    assert parsed.points[0].mark is False
    assert parsed.points[0].dwell_s == 2.0


def test_gps_csv_missing_lat_lon_rejected():
    try:
        parse_point_gps_csv_text("lat,lon\n13.0\n")
        assert False
    except ValueError as exc:
        assert "expected 2 column(s)" in str(exc)


def test_gps_csv_invalid_lat_rejected():
    try:
        parse_point_gps_csv_text("lat,lon\n91.0,80.0\n")
        assert False
    except ValueError as exc:
        assert "lat must be within [-90, 90]" in str(exc)


def test_gps_csv_invalid_lon_rejected():
    try:
        parse_point_gps_csv_text("lat,lon\n13.0,181.0\n")
        assert False
    except ValueError as exc:
        assert "lon must be within [-180, 180]" in str(exc)


def test_gps_csv_requires_header():
    try:
        parse_point_gps_csv_text("13.0,80.0\n")
        assert False
    except ValueError as exc:
        assert "expected lat,lon CSV header" in str(exc)


# ── survey-export CSV (Emlid/Trimble named header) → point mission ─────────────

# The real 2x2_square.csv shape: Name,Code,…,Longitude,Latitude,… (lon BEFORE lat),
# 4 RTK-FIX corners of a ~2 m square at the Chennai site. Trimmed columns; DictReader
# maps by name so position is irrelevant as long as the header names match.
_EMLID_2X2 = (
    "Name,Code,Code description,Easting,Northing,Longitude,Latitude,Solution status,Samples,Lateral RMS\n"
    "1,P,Point,1204636.447,1243756.378,80.26194119,13.07206386,FIX,26,0.017\n"
    "2,P,Point,1204636.581,1243758.386,80.26194256,13.07208200,FIX,26,0.017\n"
    "3,P,Point,1204638.580,1243758.261,80.26196097,13.07208073,FIX,26,0.017\n"
    "4,P,Point,1204638.491,1243756.249,80.26196002,13.07206256,FIX,26,0.017\n"
)


def test_emlid_survey_export_parses_as_point_mission():
    parsed = parse_point_gps_csv_text(_EMLID_2X2)
    assert len(parsed.points) == 4
    # First Name-ordered point is the anchor.
    assert parsed.anchor_lat == 13.07206386
    assert parsed.anchor_lon == 80.26194119
    assert abs(parsed.points[0].north_m) < 1e-6 and abs(parsed.points[0].east_m) < 1e-6
    # Every surveyed point is a mark with the default dwell.
    assert all(p.mark is True for p in parsed.points)
    assert all(p.dwell_s == 2.0 for p in parsed.points)
    # P2 is ~2 m NORTH of P1 (lat increased), barely any east.
    assert parsed.points[1].north_m > 1.5
    assert abs(parsed.points[1].east_m) < 0.5


def test_emlid_export_ordered_by_name_not_file_order():
    shuffled = "\n".join(
        [_EMLID_2X2.splitlines()[0]] + list(reversed(_EMLID_2X2.splitlines()[1:]))
    ) + "\n"
    parsed = parse_point_gps_csv_text(shuffled)
    # Anchor is still Name==1 despite the rows being reversed in the file.
    assert parsed.anchor_lat == 13.07206386


def test_survey_alias_name_latitude_longitude():
    # arc_survey.csv shape: name,latitude,longitude (spelled out, lat then lon).
    text = "name,latitude,longitude\n1,13.07206,80.261941\n2,13.0720781,80.261941\n"
    parsed = parse_point_gps_csv_text(text)
    assert len(parsed.points) == 2
    assert parsed.anchor_lat == 13.07206


def test_emlid_payload_is_gps_surveyed():
    payload = gps_point_mission_parse_payload(parse_point_gps_csv_text(_EMLID_2X2))
    assert payload["point_source_frame"] == GPS_SURVEYED_FRAME
    assert payload["num_points"] == 4
    assert payload["anchor"] == {"lat": 13.07206386, "lon": 80.26194119}


def test_bare_latlon_still_takes_precedence_over_survey():
    # A bare lat,lon,dwell,mark file must go through the bare parser (keeping
    # dwell + mark), NOT the survey fallback (which forces mark=True/default dwell).
    parsed = parse_point_gps_csv_text("lat,lon,dwell_s,mark\n13.0,80.0,5.0,false\n")
    assert parsed.points[0].dwell_s == 5.0
    assert parsed.points[0].mark is False


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print(f"ok {test.__name__}")
    print("PASS")


if __name__ == "__main__":
    main()