#!/usr/bin/env python3
"""Spray path-projection continuity (2026-08-04).

THE BUG. A run `/path` can carry the approach leg and the marked leg in ONE
message. On an out-and-back line the two legs sit centimetres apart, and
`_project_onto_path` used to re-scan every segment each tick with no memory,
taking the globally nearest. With the rover's own cross-track (1-3 cm)
comparable to the leg spacing, "nearest" flips between legs.

Measured on bag stg_dir_test/stg_d8a4f2ad (110 pts, 8.52 m, direction reversal
at segment 29, MARK vertices 34..104 = s 4.721..8.024): the reported station
teleported between s=1.67 and s=7.54 while the rover was still on the approach
leg, and the MARK flag did not settle true until s=5.10 — 41 cm past its own
boundary. Same mechanism produced 1-3 cm paint gaps mid-mark (4-8 valve edges
where there should be 2) and one run that never sprayed at all.

Load-bearing invariants:
  * zero windows reproduce the pre-fix global search EXACTLY (A/B arm);
  * a fresh path (prev_s=None) always acquires globally;
  * a genuine relocalisation still re-acquires (window is abandoned when
    nothing in it explains where we are).

Run:  python3 -m pytest -q test_spray_projection_continuity.py   (Jetson only)
"""

import math

from spray_controller_node import _build_path_model, _project_onto_path

WB, WF, RE = 0.5, 2.0, 1.0          # the shipped defaults


def _out_and_back(sep_m=0.04, mark_from=10):
    """SW 4 m then NE 4 m, legs `sep_m` apart. MARK on the RETURN leg only."""
    pts, flg = [], []
    for k in range(41):                       # outbound, transit
        pts.append((-0.1 * k, 0.0)); flg.append(False)
    for k in range(41):                       # return, MARK after mark_from
        pts.append((-4.0 + 0.1 * k, sep_m)); flg.append(k >= mark_from)
    return _build_path_model(pts, flg), 41 + mark_from


def test_global_scan_picks_the_wrong_leg():
    """Documents the bug: nearer the OUTBOUND leg => flag reads TRANSIT."""
    m, mark_i = _out_and_back()
    mark_s = m.cumulative_s[mark_i]
    # Physically on the return (MARK) leg, 2 cm past the boundary, but sitting
    # 1.5 cm off toward the outbound leg — i.e. nearer the wrong one.
    n, e = -4.0 + 0.1 * 10 + 0.02, 0.015
    got = _project_onto_path(m, n, e)
    assert got.current_flag is False, "expected the pre-fix wrong-leg latch"
    assert got.s < mark_s, got.s


def test_window_keeps_it_on_the_right_leg():
    """The fix: with the previous station known, the MARK leg wins."""
    m, mark_i = _out_and_back()
    mark_s = m.cumulative_s[mark_i]
    n, e = -4.0 + 0.1 * 10 + 0.02, 0.015
    got = _project_onto_path(m, n, e, prev_s=mark_s - 0.05,
                             window_back_m=WB, window_fwd_m=WF,
                             reacquire_dist_m=RE)
    assert got.current_flag is True, got
    assert got.s >= mark_s, got.s


def test_zero_window_is_exactly_the_old_behaviour():
    """A/B arm must be a byte-identical reduction, not an approximation."""
    m, _ = _out_and_back()
    for k in range(0, 80, 3):
        n = -4.0 + k * 0.1
        for e in (0.0, 0.015, 0.04, -0.2):
            a = _project_onto_path(m, n, e)
            b = _project_onto_path(m, n, e, prev_s=1.0,
                                   window_back_m=0.0, window_fwd_m=0.0)
            assert a == b, (n, e, a, b)


def test_fresh_path_acquires_globally():
    """prev_s=None must never restrict the search."""
    m, _ = _out_and_back()
    n, e = -1.0, 0.0
    assert _project_onto_path(m, n, e) == _project_onto_path(
        m, n, e, prev_s=None, window_back_m=WB, window_fwd_m=WF,
        reacquire_dist_m=RE)


def test_relocalisation_reacquires():
    """A real jump must not stay latched to a stale window."""
    m, _ = _out_and_back()
    # 5 m off the path, previous station claims s=7.0. Nothing in the window
    # explains this, so the global scan must take over.
    got = _project_onto_path(m, -2.0, 5.0, prev_s=7.0,
                             window_back_m=WB, window_fwd_m=WF,
                             reacquire_dist_m=RE)
    ref = _project_onto_path(m, -2.0, 5.0)
    assert got.s == ref.s and got.segment_index == ref.segment_index


def test_window_survives_normal_advance():
    """Consecutive ticks at marking speed stay inside the window."""
    m, mark_i = _out_and_back()
    s = m.cumulative_s[mark_i] - 0.30
    prev = s
    for step in range(60):                      # 60 ticks x 7 mm = 42 cm
        n = -4.0 + 0.1 * 10 - 0.30 + step * 0.007
        got = _project_onto_path(m, n, 0.015, prev_s=prev,
                                 window_back_m=WB, window_fwd_m=WF,
                                 reacquire_dist_m=RE)
        assert got.s >= prev - 0.05, (step, got.s, prev)   # no backward teleport
        prev = got.s
    assert prev > m.cumulative_s[mark_i], prev


def test_no_backward_teleport_across_the_reversal():
    """The exact failure signature: station must not jump legs mid-mark."""
    m, mark_i = _out_and_back()
    prev = m.cumulative_s[mark_i] + 0.10
    for step in range(40):
        n = -4.0 + 0.1 * 10 + 0.10 + step * 0.01
        got = _project_onto_path(m, n, 0.015, prev_s=prev,
                                 window_back_m=WB, window_fwd_m=WF,
                                 reacquire_dist_m=RE)
        # the bug moved s by metres in one tick; allow only physical motion
        assert abs(got.s - prev) < 0.5, (step, prev, got.s)
        assert got.current_flag is True, (step, got.s)
        prev = got.s
