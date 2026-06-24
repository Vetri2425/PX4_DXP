#!/usr/bin/env python3
"""Unit tests for point mission ingest."""

from __future__ import annotations

import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from point_ingest import (
    parse_dxf_point_entities,
    parse_point_csv_text,
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


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print(f"ok {test.__name__}")
    print("PASS")


if __name__ == "__main__":
    main()