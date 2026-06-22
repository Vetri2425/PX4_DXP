#!/usr/bin/env python3
"""Unit tests for point mission ingest."""

from __future__ import annotations

import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from point_ingest import parse_dxf_point_entities, parse_point_csv_text


def test_csv_two_column():
    pts = parse_point_csv_text("1.0,2.0\n3.0,4.0\n")
    assert len(pts) == 2
    assert pts[0].north_m == 1.0
    assert pts[0].east_m == 2.0
    assert pts[0].dwell_s == 2.0


def test_csv_with_dwell():
    pts = parse_point_csv_text("1.0,2.0,3.5\n")
    assert pts[0].dwell_s == 3.5


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


def test_dxf_point_entities():
    ent = types.SimpleNamespace(
        entity_type="POINT",
        entity_id="p1",
        geometry={"position": (1.5, 2.5)},
    )
    pts = parse_dxf_point_entities([ent], default_dwell_s=1.5)
    assert len(pts) == 1
    assert pts[0].dwell_s == 1.5


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print(f"ok {test.__name__}")
    print("PASS")


if __name__ == "__main__":
    main()