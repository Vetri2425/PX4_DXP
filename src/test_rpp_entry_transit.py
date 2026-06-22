"""ROS-free tests for the RPP runtime entry boundary."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rpp_path_conditioning import split_leading_entry_transit


def test_mark_first_entry_is_split_before_profile_conditioning():
    points = [(-2.0, 1.0), (0.0, 0.0), (0.0, 0.0), (1.0, 0.0)]
    flags = [False, False, True, True]

    entry, mission, mission_flags = split_leading_entry_transit(
        points, flags, marked=True
    )

    assert entry == ([(-2.0, 1.0), (0.0, 0.0)], [False, False])
    assert mission == [(0.0, 0.0), (1.0, 0.0)]
    assert mission_flags == [True, True]


def test_forced_smooth_receives_separate_entry_and_original_mission():
    points = [(-2.0, 1.0), (0.0, 0.0), (0.0, 0.0), (0.0, 2.0)]
    flags = [False, False, True, True]

    entry, smooth_points, smooth_flags = split_leading_entry_transit(
        points, flags, marked=True
    )

    assert entry[0][-1] == smooth_points[0]
    assert entry[1] == [False, False]
    assert smooth_points == points[2:]
    assert smooth_flags == flags[2:]


def test_pre_first_runtime_entry_is_also_split_before_smoothing():
    points = [(-2.0, 0.0), (-0.5, 0.0), (-0.5, 0.0), (0.0, 0.0), (1.0, 0.0)]
    flags = [False, False, False, False, True]

    entry, mission_points, mission_flags = split_leading_entry_transit(
        points, flags, marked=True
    )

    assert entry == ([(-2.0, 0.0), (-0.5, 0.0)], [False, False])
    assert mission_points == points[2:]
    assert mission_flags == flags[2:]


def test_unmarked_legacy_duplicate_is_unchanged():
    points = [(-2.0, 0.0), (0.0, 0.0), (0.0, 0.0), (1.0, 0.0)]
    flags = [False, False, True, True]

    entry, unchanged_points, unchanged_flags = split_leading_entry_transit(
        points, flags, marked=False
    )

    assert entry is None
    assert unchanged_points == points
    assert unchanged_flags == flags
