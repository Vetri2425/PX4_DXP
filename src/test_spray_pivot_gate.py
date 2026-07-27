#!/usr/bin/env python3
"""Spray on/off must be decided by GEOMETRY, never by speed.

Regression cover for the 2026-07-17 field bags: a bare `speed < 0.05` safety
gate collided with the frozen RPP corner speeds (brake cap 0.08, min corner
0.08, endpoint approach 0.03). The rover crawls across that threshold by
design, so the gate dithered -- 157 valve transitions where the geometry asked
for 25. These tests pin the replacement: speed has no authority over on/off,
and only an explicit RPP pivot state suppresses spray.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_spray_manual_override import make_node  # noqa: E402
from spray_controller_node import _SEGMENT_STATE_CORNER_ALIGN  # noqa: E402


class _Msg:
    def __init__(self, data):
        self.data = data


def _armed_node():
    node = make_node()
    node._path_model = object()  # non-None: "path loaded"
    return node


def _safety(node, speed):
    return node._auto_safety_status(
        pose_fresh=True, speed=speed, velocity_fresh=True
    )


# --- speed must not gate ---------------------------------------------------


def test_speed_below_old_gate_still_sprays():
    """The exact speeds that chattered in the field must now pass."""
    node = _armed_node()
    # every one of these is below the old min_spray_speed_mps=0.05
    for speed in (0.0, 0.003, 0.03, 0.033, 0.044, 0.049):
        ok, reason = _safety(node, speed)
        assert ok, f"speed {speed} was blocked: {reason!r}"


def test_endpoint_approach_speed_sprays():
    """segment_endpoint_approach_speed=0.03 was structurally unsprayable."""
    ok, reason = _safety(_armed_node(), 0.03)
    assert ok, reason


def test_min_spray_speed_param_is_inert():
    """Param stays declared for the server's contract, but must not gate."""
    node = _armed_node()
    node._params["min_spray_speed_mps"].value = 0.50  # absurdly high
    ok, _ = _safety(node, 0.01)
    assert ok, "min_spray_speed_mps must no longer have on/off authority"


def test_no_chatter_across_the_old_threshold():
    """Sweeping through 0.05 must not change the decision even once."""
    node = _armed_node()
    speeds = [0.033, 0.053, 0.037, 0.051, 0.044, 0.075]  # measured, run 160753
    decisions = [_safety(node, s)[0] for s in speeds]
    assert all(decisions), decisions
    assert len(set(decisions)) == 1, "decision flipped with speed alone"


# --- pivot gate ------------------------------------------------------------


def test_pivot_suppresses_spray():
    node = _armed_node()
    node._segment_debug_cb(_Msg([1.0, float(_SEGMENT_STATE_CORNER_ALIGN)]))
    ok, reason = _safety(node, 0.30)
    assert not ok
    assert reason == "pivoting in place"


def test_corner_stop_does_not_suppress_spray():
    """CORNER_STOP(5) still coasts ~2cm of real line -- must keep painting."""
    node = _armed_node()
    node._segment_debug_cb(_Msg([1.0, 5.0]))
    ok, _ = _safety(node, 0.05)
    assert ok, "CORNER_STOP must not suppress; only CORNER_ALIGN does"


def test_tracking_state_does_not_suppress():
    node = _armed_node()
    node._segment_debug_cb(_Msg([1.0, 1.0]))  # TRACK_SEGMENT
    assert _safety(node, 0.30)[0]


def test_pivot_gate_can_be_disabled():
    node = _armed_node()
    node._params["spray_off_during_pivot"].value = False
    node._segment_debug_cb(_Msg([1.0, float(_SEGMENT_STATE_CORNER_ALIGN)]))
    assert _safety(node, 0.30)[0]


def test_absent_segment_state_is_permissive():
    """An RPP that never publishes the topic must not kill spray."""
    node = _armed_node()
    assert node._segment_state is None
    assert _safety(node, 0.30)[0]


def test_stale_segment_state_is_permissive():
    """B3(b) NEW spec: a message-free CORNER_ALIGN must FAIL OPEN once stale.

    The old spec (this test used to assert it) held spray off for the full
    segment_state_timeout_s after the last message — a 1.0 s / ~10 cm unpainted
    gap at the start of every smooth MARK run, because the smooth profile went
    silent right after a run-boundary CORNER_ALIGN. The gate must now clear the
    latched state on the first stale tick, releasing spray and staying released
    until a genuinely fresh message arrives.
    """
    node = _armed_node()
    node._segment_debug_cb(_Msg([1.0, float(_SEGMENT_STATE_CORNER_ALIGN)]))
    assert not _safety(node, 0.30)[0]  # fresh -> suppressed
    node._clock.ns += 1_100_000_000  # just past segment_state_timeout_s=1.0
    assert _safety(node, 0.30)[0], "stale pivot state must fail open"
    # New spec: the stale state is CLEARED, not merely ignored for this tick.
    assert node._segment_state is None
    assert node._segment_state_recv_time is None


def test_fresh_tracking_message_releases_pivot_gate_promptly():
    """B3(a): the RPP now publishes a TRACK_SEGMENT edge in the smooth profile,
    so a real tracking message releases the gate immediately — without waiting
    out segment_state_timeout_s. This is what closes the ~1 s / ~10 cm gap; the
    spray node needs no timeout at all once a fresh non-pivot edge is on the
    wire."""
    node = _armed_node()
    node._segment_debug_cb(_Msg([1.0, float(_SEGMENT_STATE_CORNER_ALIGN)]))
    assert not _safety(node, 0.30)[0]  # pivoting -> suppressed
    node._clock.ns += 20_000_000  # 20 ms later (one 50 Hz RPP tick)
    node._segment_debug_cb(_Msg([1.0, 1.0]))  # TRACK_SEGMENT arrives
    assert _safety(node, 0.30)[0], "a fresh tracking edge must release the gate at once"


def test_short_message_ignored():
    node = _armed_node()
    node._segment_debug_cb(_Msg([1.0]))  # malformed, no state field
    assert node._segment_state is None
    assert _safety(node, 0.30)[0]


# --- contract with the RPP enum -------------------------------------------


def test_corner_align_code_matches_rpp_enum():
    """This node duplicates the RPP's enum value; pin them together.

    If the RPP renumbers SegmentStateCode, the spray node would silently
    suppress on the wrong state (or never suppress). Fail loudly here instead.
    """
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "rpp_controller_node.py")
    with open(src) as fh:
        text = fh.read()
    assert f"CORNER_ALIGN = {_SEGMENT_STATE_CORNER_ALIGN}" in text, (
        "spray_controller_node._SEGMENT_STATE_CORNER_ALIGN is out of sync with "
        "rpp_controller_node.SegmentStateCode.CORNER_ALIGN"
    )
