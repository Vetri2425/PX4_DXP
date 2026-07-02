#!/usr/bin/env python3
"""Unit tests for pure RPP path-conditioning helpers."""

from __future__ import annotations

from rpp_path_conditioning import split_leading_entry_transit


def test_split_legacy_two_point_runtime_entry():
    pts = [(0.0, -0.5), (0.0, 0.0), (0.0, 0.0), (0.0, 1.0)]
    flags = [False, False, True, True]

    entry, profile_pts, profile_flags = split_leading_entry_transit(
        pts, flags, marked=True
    )

    assert entry == ([(0.0, -0.5), (0.0, 0.0)], [False, False])
    assert profile_pts == [(0.0, 0.0), (0.0, 1.0)]
    assert profile_flags == [True, True]


def test_split_densified_runtime_entry_duplicate_at_end():
    pts = [
        (1.406, -0.926),
        (1.419, -0.883),
        (1.431, -0.839),
        (1.443, -0.796),
        (1.456, -0.752),
        (1.468, -0.709),
        (1.480, -0.665),
        (1.480, -0.665),
        (1.480, -0.616),
    ]
    flags = [False, False, False, False, False, False, False, True, True]

    entry, profile_pts, profile_flags = split_leading_entry_transit(
        pts, flags, marked=True
    )

    assert entry == (pts[:7], [False] * 7)
    assert profile_pts == pts[7:]
    assert profile_flags == [True, True]


def test_no_split_without_runtime_marker():
    pts = [(0.0, -0.5), (0.0, 0.0), (0.0, 0.0), (0.0, 1.0)]
    flags = [False, False, True, True]

    entry, profile_pts, profile_flags = split_leading_entry_transit(
        pts, flags, marked=False
    )

    assert entry is None
    assert profile_pts == pts
    assert profile_flags == flags


def main():
    test_split_legacy_two_point_runtime_entry()
    test_split_densified_runtime_entry_duplicate_at_end()
    test_no_split_without_runtime_marker()
    print("PASS")


if __name__ == "__main__":
    main()
