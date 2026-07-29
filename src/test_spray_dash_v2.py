#!/usr/bin/env python3
"""Phase C — DashMeter unit tests (plan §8). Pure, no rclpy — runs on the Mac.

Covers the §8 dash requirements: toggle math on a straight run, a dash shorter
than one leg, a dash spanning multiple legs (continuous across corners), the
monotonic-`s` jump rejection AND its recovery, and the xtrack arming gate.

The rover ticks at ~50 Hz, so real per-tick arc-length steps are millimetres.
`drive()` walks `s` in fine increments (each well inside the jump-tolerance
window) exactly as the live control loop would, instead of teleporting `s`.

Run:  python3 -m pytest -q test_spray_dash_v2.py
"""

from spray_modes import DashMeter


def drive(meter, s_start, s_end, *, step=0.02, speed=1.0, dt=1.0,
          xtrack_ok=True, on_lead=0.0, off_lead=0.0, record=False):
    """Walk arc-length s_start→s_end in `step` increments, ticking each one.

    speed*dt*factor(3) = 3.0 m tolerance ≫ step, so every tick is accepted as
    real travel (the jump guard only rejects genuine teleports). Returns the
    last DashUpdate, or a list of (s, DashUpdate) when record=True.
    """
    out = []
    n = max(1, int(round((s_end - s_start) / step)))
    last = None
    for i in range(n + 1):
        s = s_start + i * step
        last = meter.update(s, speed, dt, xtrack_ok, on_lead, off_lead)
        if record:
            out.append((s, last))
    return out if record else last


def test_straight_run_toggles_on_grid():
    """6-on/3-off: ON [0,6), OFF [6,9), ON [9,15), OFF [15,18)... (period 9)."""
    m = DashMeter(on_distance_m=6.0, off_distance_m=3.0, start_state="on")
    samples = drive(m, 0.0, 20.0, record=True)
    for s, u in samples:
        cyc = s % 9.0
        expect_on = cyc < 6.0 - 1e-6
        assert u.geometry_desired == expect_on, (
            f"s={s:.3f} (cyc {cyc:.3f}): expected {'ON' if expect_on else 'OFF'}, got {u.phase}"
        )


def test_start_state_off():
    """start_state='off': OFF [0,3), ON [3,9), OFF [9,12)..."""
    m = DashMeter(on_distance_m=6.0, off_distance_m=3.0, start_state="off")
    assert drive(m, 0.0, 2.9).geometry_desired is False
    assert drive(m, 2.9, 3.2).geometry_desired is True     # into ON at 3
    assert drive(m, 3.2, 9.2).geometry_desired is False    # back OFF at 9


def test_dash_shorter_than_leg():
    """Tiny 1-on/0.5-off pattern flips many times within a single 5 m leg."""
    m = DashMeter(on_distance_m=1.0, off_distance_m=0.5, start_state="on")
    samples = drive(m, 0.0, 5.0, record=True)
    phases = [u.phase for _, u in samples]
    flips = sum(1 for a, b in zip(phases, phases[1:]) if a != b)
    # Period 1.5 m over 5 m → ~3 full cycles → ~6 flips.
    assert flips >= 5, f"expected repeated toggling within one leg, got {flips}"


def test_spans_multiple_legs_continuous_across_corners():
    """Arc-length is continuous across corners — the meter never sees a corner.

    A corner at s=5 is invisible to arc-length: the toggle must still land on
    the 6 m grid boundary, mid-'leg' (locked decision #2).
    """
    m = DashMeter(on_distance_m=6.0, off_distance_m=3.0, start_state="on")
    assert drive(m, 0.0, 5.9).geometry_desired is True    # still first 6 m ON
    assert drive(m, 5.9, 6.1).geometry_desired is False   # crossed 6 m → OFF


def test_unarmed_until_xtrack_ok():
    """Metering stays disarmed (never sprays) until the pose is on-path."""
    m = DashMeter(6.0, 3.0, "on")
    u0 = m.update(0.0, 1.0, 1.0, xtrack_ok=False)   # off-path entry transit
    assert u0.armed is False and u0.geometry_desired is False
    u1 = m.update(2.0, 1.0, 1.0, xtrack_ok=False)   # still off-path, s ignored
    assert u1.armed is False
    # Arrive on-path at s=2: arming anchors HERE, so ON runs from 2 for 6 m.
    u2 = m.update(2.0, 1.0, 1.0, xtrack_ok=True)
    assert u2.armed is True and u2.geometry_desired is True
    assert drive(m, 2.0, 7.9).geometry_desired is True    # 2..8 ON
    assert drive(m, 7.9, 8.1).geometry_desired is False   # boundary at 8


def test_backward_jump_rejected_holds_pattern():
    """A one-tick backward glitch is rejected; s_dash and phase hold."""
    m = DashMeter(6.0, 3.0, "on")
    drive(m, 0.0, 3.0)                       # s_dash≈3, phase on
    u = m.update(0.2, 0.35, 0.1, True)       # backward glitch (tol 0.105 m)
    assert u.jump_rejected is True
    assert abs(u.s_dash - 3.0) < 1e-6        # frozen, did not follow the glitch
    assert u.phase == "on"


def test_forward_glitch_rejected_then_recovers():
    """A single huge forward step is rejected; a persistent one recovers.

    At speed 0.35, dt 0.1 → tolerance 0.105 m. A +5 m step is a teleport:
    rejected once, but accepted after `jump_reject_accept_after` (5)
    self-consistent repeats so the guard cannot wedge on a real relocation.
    """
    m = DashMeter(6.0, 3.0, "on")
    drive(m, 0.0, 1.0)                       # s_dash≈1
    u = m.update(6.0, 0.35, 0.1, True)       # +5 m — glitch, rejected
    assert u.jump_rejected is True
    assert abs(u.s_dash - 1.0) < 1e-6
    last = None
    for _ in range(4):                       # 4 more self-consistent → 5 total
        last = m.update(6.0, 0.35, 0.1, True)
    assert last.recovered is True
    assert abs(last.s_dash - 6.0) < 1e-6     # snapped to the new position
    # Re-anchored: a fresh ON dash starts here, not a burst of catch-up flips.
    assert last.phase == "on" and last.geometry_desired is True


def test_lead_flips_command_early():
    """With an on_lead, the OFF->ON command fires `lead` metres before the grid."""
    m = DashMeter(on_distance_m=6.0, off_distance_m=3.0, start_state="off")
    # OFF [0,3); the upcoming flip to ON uses on_lead=0.5, so it fires at 2.5.
    u = drive(m, 0.0, 2.6, on_lead=0.5, off_lead=0.5)
    assert u.geometry_desired is True        # 2.6 > 3.0-0.5 → already ON


def test_bad_config_rejected():
    for bad in [(0.0, 3.0), (6.0, -1.0)]:
        try:
            DashMeter(*bad)
            assert False, f"expected ValueError for {bad}"
        except ValueError:
            pass
    try:
        DashMeter(6.0, 3.0, "sometimes")
        assert False, "expected ValueError for bad start_state"
    except ValueError:
        pass


def test_anchor_s_toggle_stations_independent_of_arm_raw_s():
    """R4: same geometry anchor + two arm-time raw_s → identical toggle stations.

    Customer-visible: re-marking a road must lay new dashes over the old ones,
    regardless of where on the entry the meter first saw xtrack_ok.
    """
    anchor = 2.0

    def toggle_stations(arm_raw_s: float) -> list[float]:
        m = DashMeter(6.0, 3.0, "on", anchor_s=anchor)
        m.update(arm_raw_s, 1.0, 1.0, xtrack_ok=True)
        stations: list[float] = []
        last = m.s_at_last_toggle
        for _s, u in drive(m, arm_raw_s, 25.0, record=True):
            if abs(u.s_at_last_toggle - last) > 1e-9:
                stations.append(u.s_at_last_toggle)
                last = u.s_at_last_toggle
        return stations

    a = toggle_stations(0.5)
    b = toggle_stations(1.7)
    assert a == b
    # Geometry grid from anchor=2: on 6 → 8, off 3 → 11, on 6 → 17, off 3 → 20.
    assert a == [8.0, 11.0, 17.0, 20.0]


def test_anchor_s_none_preserves_arm_at_raw_s():
    """anchor_s=None (default) still anchors the pattern at the arm-time raw_s."""
    m = DashMeter(6.0, 3.0, "on")  # no anchor_s
    m.update(2.0, 1.0, 1.0, xtrack_ok=True)
    assert abs(m.s_at_last_toggle - 2.0) < 1e-9
    assert drive(m, 2.0, 7.9).geometry_desired is True
    assert drive(m, 7.9, 8.1).geometry_desired is False


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("ALL DASH TESTS PASSED")
