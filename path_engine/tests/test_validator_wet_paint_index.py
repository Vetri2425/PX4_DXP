"""The wet-paint check is spatially indexed; it must still agree with all-pairs.

The original implementation tested every spray-OFF segment against every earlier
sprayed one — O(n^2). On the 2.4 km roads survey (48560 waypoints) that was 67.7
million segment tests, 35 s on a laptop and 100 s on the Jetson, and it was the
single dominant cost of planning the mission. It is now grid-indexed.

Speed is worthless if the answer changed, so these tests pin the new result
against a brute-force reference implementation of the ORIGINAL algorithm.
"""

import random

import pytest

from path_engine.core import PlannedPath
from path_engine.validator import PathValidator


def _brute_force(wp, fl, validator):
    """The original all-pairs algorithm, verbatim, as the reference."""
    crossings = []
    for i in range(len(wp) - 1):
        if fl[i]:
            continue
        for j in range(i):
            if not fl[j]:
                continue
            if validator._seg_cross(wp[i], wp[i + 1], wp[j], wp[j + 1]):
                crossings.append(wp[i])
                break
    return crossings


def _indexed(wp, fl, validator):
    """Run the real check and recover the crossing points from its warning."""
    warnings = []
    plan = PlannedPath(segments=[], merged_waypoints=list(wp), spray_flags=list(fl))
    validator._check_drives_over_wet_paint(plan, warnings)
    if not warnings:
        return 0
    text = warnings[0]
    return int(text.split("painted lines at ")[1].split(" point")[0])


@pytest.fixture
def validator():
    return PathValidator()


def _grid_mission(n_lines=6, pitch=2.0, length=10.0, step=0.25):
    """Parallel painted lines, each reached by a transit that crosses the others."""
    wp, fl = [], []
    for k in range(n_lines):
        e = k * pitch
        # transit in from a shared corridor off to one side — crosses earlier paint
        wp.append((-4.0, -6.0)); fl.append(False)
        wp.append((length / 2, e)); fl.append(False)
        m = int(length / step)
        for i in range(m + 1):
            wp.append((i * step, e)); fl.append(True)
    return wp, fl


def test_indexed_matches_all_pairs_on_a_crossing_mission(validator):
    wp, fl = _grid_mission()
    expected = _brute_force(wp, fl, validator)
    assert len(expected) > 0, "fixture must actually produce crossings"
    assert _indexed(wp, fl, validator) == len(expected)


def test_indexed_matches_all_pairs_when_nothing_crosses(validator):
    """A clean mission must stay clean — no false positives from the grid."""
    wp, fl = [], []
    for k in range(6):
        for i in range(40):
            wp.append((i * 0.25, k * 2.0)); fl.append(True)
        wp.append((0.0, k * 2.0 + 2.0)); fl.append(False)   # retrace, no crossing
    assert _brute_force(wp, fl, validator) == []
    assert _indexed(wp, fl, validator) == 0


@pytest.mark.parametrize("seed", [1, 2, 3, 4, 5])
def test_indexed_matches_all_pairs_on_random_paths(validator, seed):
    """Random geometry, including transits long enough to span many grid cells —
    the case the index has to fall back to a full scan for."""
    rng = random.Random(seed)
    wp, fl = [], []
    for _ in range(140):
        wp.append((rng.uniform(-60, 60), rng.uniform(-60, 60)))
        fl.append(rng.random() < 0.6)
    assert _indexed(wp, fl, validator) == len(_brute_force(wp, fl, validator))


def test_indexed_handles_a_transit_spanning_the_whole_site(validator):
    """A single 500 m transit covers far more than the 64-cell cap, so it takes
    the oversized path on both the insert and the lookup side."""
    wp, fl = [], []
    for i in range(60):                       # a painted line near the origin
        wp.append((i * 0.25, 0.0)); fl.append(True)
    wp.append((7.5, -250.0)); fl.append(False)   # huge transit crossing it
    wp.append((7.5, 250.0)); fl.append(False)
    for i in range(60):
        wp.append((i * 0.25, 5.0)); fl.append(True)
    assert _indexed(wp, fl, validator) == len(_brute_force(wp, fl, validator))


def test_oversized_painted_segment_is_still_checked(validator):
    """A LONG painted segment goes in the oversized list, not the grid; a later
    transit must still be found crossing it."""
    wp = [(0.0, -200.0), (0.0, 200.0),        # 400 m painted line
          (-50.0, 10.0), (50.0, 10.0)]        # transit crossing it
    fl = [True, True, False, False]
    assert _indexed(wp, fl, validator) == len(_brute_force(wp, fl, validator)) > 0
