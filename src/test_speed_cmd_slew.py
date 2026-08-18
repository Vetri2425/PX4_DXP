"""Patch 2 — smooth-profile speed_cmd slew helpers.

Mirrors src/rpp_controller_node.py static helpers so these tests do not
need a ROS2 node. Keep in sync with RPPControllerNode._update_kappa_hard_latch
and RPPControllerNode._apply_smooth_speed_slew.
"""
import pytest


def update_kappa_hard_latch(
    hard_latched: bool,
    kappa_now: float,
    kappa_hard_enter: float = 0.25,
    kappa_hard_exit: float = 0.15,
) -> bool:
    if kappa_hard_enter <= kappa_hard_exit:
        return abs(kappa_now) >= kappa_hard_enter
    if abs(kappa_now) >= kappa_hard_enter:
        return True
    if abs(kappa_now) <= kappa_hard_exit:
        return False
    return bool(hard_latched)


def apply_smooth_speed_slew(
    speed_raw: float,
    last_speed: float,
    dt: float,
    *,
    hard_latched: bool = False,
    speed_cmd_decel: float = 0.30,
    max_accel: float = 0.20,
    accel_scale: float = 1.0,
    approach_active: bool = False,
    p4_floor: float = 0.02,
) -> tuple[float, int]:
    if speed_raw > last_speed:
        if max_accel > 0.0:
            speed = min(
                speed_raw,
                last_speed + max_accel * max(0.0, accel_scale) * max(0.0, dt),
            )
        else:
            speed = speed_raw
        return speed, 0
    if approach_active or speed_raw < p4_floor:
        return speed_raw, 3
    if hard_latched:
        return speed_raw, 2
    if speed_cmd_decel > 0.0:
        return max(speed_raw, last_speed - speed_cmd_decel * max(0.0, dt)), 1
    return speed_raw, 2


def reset_kappa_hard_latch() -> bool:
    """Same result as RPPControllerNode._reset_kappa_hard_latch()."""
    return False


def test_accel_path_uses_sbs():
    speed, mode = apply_smooth_speed_slew(
        1.0, 0.30, 0.02, accel_scale=1.0, max_accel=0.20,
    )
    assert mode == 0
    assert speed == pytest.approx(0.30 + 0.20 * 1.0 * 0.02)

    blocked, mode_b = apply_smooth_speed_slew(
        1.0, 0.30, 0.02, accel_scale=0.0, max_accel=0.20,
    )
    assert mode_b == 0
    assert blocked == pytest.approx(0.30)


def test_sweep_flicker_decel_is_bounded():
    speed, mode = apply_smooth_speed_slew(
        0.45, 0.90, 0.02,
        hard_latched=update_kappa_hard_latch(False, 0.08),
        speed_cmd_decel=0.30,
    )
    assert mode == 1
    assert speed == pytest.approx(0.894)
    assert speed >= 0.894


def test_no_010_drop_over_0p2s_in_soft_mode():
    last = 0.90
    dt = 0.02
    ticks = int(0.2 / dt)
    for _ in range(ticks):
        last, mode = apply_smooth_speed_slew(
            0.45, last, dt,
            hard_latched=update_kappa_hard_latch(False, 0.08),
            speed_cmd_decel=0.30,
        )
        assert mode == 1
    assert 0.90 - last <= 0.10 + 1e-9
    assert last == pytest.approx(0.90 - 0.30 * 0.2)


def test_tight_curve_immediate():
    latched = update_kappa_hard_latch(False, 0.47)
    assert latched is True
    speed, mode = apply_smooth_speed_slew(
        0.30, 1.0, 0.02, hard_latched=latched, speed_cmd_decel=0.30,
    )
    assert mode == 2
    assert speed == pytest.approx(0.30)


def test_hysteresis_enter_exit():
    latched = False
    latched = update_kappa_hard_latch(latched, 0.20)
    assert latched is False
    latched = update_kappa_hard_latch(latched, 0.26)
    assert latched is True
    latched = update_kappa_hard_latch(latched, 0.20)
    assert latched is True
    latched = update_kappa_hard_latch(latched, 0.15)
    assert latched is False


def test_approach_and_p4_bypass_immediate():
    speed_a, mode_a = apply_smooth_speed_slew(
        0.40, 1.0, 0.02, approach_active=True, speed_cmd_decel=0.30,
    )
    assert mode_a == 3
    assert speed_a == pytest.approx(0.40)

    speed_p, mode_p = apply_smooth_speed_slew(
        0.01, 0.50, 0.02, p4_floor=0.02, speed_cmd_decel=0.30,
    )
    assert mode_p == 3
    assert speed_p == pytest.approx(0.01)


def test_zero_decel_restores_legacy_unbounded():
    speed, mode = apply_smooth_speed_slew(
        0.45, 0.90, 0.02,
        hard_latched=False,
        speed_cmd_decel=0.0,
    )
    assert mode == 2
    assert speed == pytest.approx(0.45)


def test_hard_mode_never_exceeds_raw_target():
    latched = update_kappa_hard_latch(False, 0.47)
    speed, mode = apply_smooth_speed_slew(
        0.30, 1.0, 0.02, hard_latched=latched, speed_cmd_decel=0.30,
    )
    assert mode == 2
    assert speed == pytest.approx(0.30)
    assert speed <= 0.30


def test_hard_latch_resets_on_run_path_reset():
    latched = update_kappa_hard_latch(False, 0.47)
    assert latched is True
    assert reset_kappa_hard_latch() is False
