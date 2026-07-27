"""ControlArbiter mutual-exclusion unit tests (plan phase J1).

Covers both directions of the single-owner invariant (plan §2, §7.3): a
mission cannot start while the joystick owns control, and the joystick
cannot acquire while a mission is active — including transitions still in
flight (ACQUIRING / a MISSION claim not yet finalised).
"""
from __future__ import annotations

import asyncio
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from control_arbiter import (
    ControlArbiter,
    ControlArbiterError,
    ControlOwner,
    get_control_arbiter,
    reset_control_arbiter_for_tests,
)
from models import MissionState


class FakeOffboardCtrl:
    def __init__(self, state=MissionState.IDLE):
        self.state = state


def run(coro):
    return asyncio.run(coro)


def test_starts_idle():
    arbiter = ControlArbiter()
    assert arbiter.owner == ControlOwner.IDLE
    assert arbiter.joystick_owned is False


def test_mission_start_claims_and_releases_to_idle():
    arbiter = ControlArbiter()
    offboard = FakeOffboardCtrl(state=MissionState.IDLE)

    async def scenario():
        async with arbiter.mission_start(offboard):
            assert arbiter.owner == ControlOwner.MISSION
        assert arbiter.owner == ControlOwner.IDLE

    run(scenario())


def test_mission_start_relinquishes_on_exit_even_if_state_active():
    """owner==MISSION is in-flight-only: it does NOT persist for the mission's
    runtime. The running mission is guarded by offboard_ctrl.state, not by a
    sticky owner. Relinquishing on every exit is what prevents stranding when a
    mission ends outside this bracket (mark_completed, abort, disarm)."""
    arbiter = ControlArbiter()
    offboard = FakeOffboardCtrl(state=MissionState.RUNNING)

    async def scenario():
        async with arbiter.mission_start(offboard):
            assert arbiter.owner == ControlOwner.MISSION
        assert arbiter.owner == ControlOwner.IDLE

    run(scenario())


def test_joystick_can_acquire_after_mission_completes_outside_bracket():
    """Regression: a mission that completes naturally (mark_completed in the
    telemetry loop) leaves state COMPLETED without re-entering mission_start.
    owner must not be stranded at MISSION, so a joystick acquire must succeed."""
    arbiter = ControlArbiter()
    offboard = FakeOffboardCtrl(state=MissionState.RUNNING)

    async def scenario():
        # Mission ran and exited the start bracket while still RUNNING…
        async with arbiter.mission_start(offboard):
            pass
        # …then completed in the telemetry loop, outside any bracket.
        offboard.state = MissionState.COMPLETED
        await arbiter.begin_joystick_acquire(offboard)

    run(scenario())
    assert arbiter.owner == ControlOwner.JOYSTICK_ACQUIRING


def test_joystick_acquire_rejected_while_mission_state_active():
    arbiter = ControlArbiter()
    offboard = FakeOffboardCtrl(state=MissionState.RUNNING)
    with pytest.raises(ControlArbiterError) as exc:
        run(arbiter.begin_joystick_acquire(offboard))
    assert exc.value.code == "mission_active"
    assert arbiter.owner == ControlOwner.IDLE  # rejected claim leaves owner untouched


def test_joystick_acquire_rejected_while_mission_transition_in_flight():
    """Even after mission_start()'s claim, before the mission enters an
    ACTIVE state string, the MISSION owner itself blocks a joystick claim."""
    arbiter = ControlArbiter()
    offboard = FakeOffboardCtrl(state=MissionState.IDLE)

    async def scenario():
        async with arbiter.mission_start(offboard):
            with pytest.raises(ControlArbiterError) as exc:
                await arbiter.begin_joystick_acquire(offboard)
            assert exc.value.code == "mission_active"

    run(scenario())


def test_mission_start_rejected_while_joystick_owns_control():
    arbiter = ControlArbiter()
    offboard = FakeOffboardCtrl(state=MissionState.IDLE)
    run(arbiter.begin_joystick_acquire(offboard))
    arbiter.mark_joystick_active("sess1", "lease1")

    async def try_mission():
        async with arbiter.mission_start(offboard):
            pass

    with pytest.raises(ControlArbiterError) as exc:
        run(try_mission())
    assert exc.value.code == "joystick_active"
    assert arbiter.owner == ControlOwner.JOYSTICK_ACTIVE  # unaffected by rejected claim


def test_mission_start_rejected_while_joystick_still_acquiring():
    """ACQUIRING (not yet ACTIVE) must already block a concurrent mission
    start — this is the concurrent-acquire race in plan §7.3 / audit R1."""
    arbiter = ControlArbiter()
    offboard = FakeOffboardCtrl(state=MissionState.IDLE)
    run(arbiter.begin_joystick_acquire(offboard))  # claims ACQUIRING, no follow-up

    async def try_mission():
        async with arbiter.mission_start(offboard):
            pass

    with pytest.raises(ControlArbiterError) as exc:
        run(try_mission())
    assert exc.value.code == "joystick_active"


def test_second_joystick_acquire_rejected_while_first_owns():
    arbiter = ControlArbiter()
    offboard = FakeOffboardCtrl(state=MissionState.IDLE)
    run(arbiter.begin_joystick_acquire(offboard))
    arbiter.mark_joystick_active("sess1", "lease1")
    with pytest.raises(ControlArbiterError) as exc:
        run(arbiter.begin_joystick_acquire(offboard))
    assert exc.value.code == "joystick_active"


def test_clear_joystick_reverts_to_idle_and_allows_new_acquire():
    arbiter = ControlArbiter()
    offboard = FakeOffboardCtrl(state=MissionState.IDLE)
    run(arbiter.begin_joystick_acquire(offboard))
    arbiter.mark_joystick_active("sess1", "lease1")
    arbiter.clear_joystick(reason="test")
    assert arbiter.owner == ControlOwner.IDLE
    # A fresh acquire must now succeed without raising.
    run(arbiter.begin_joystick_acquire(offboard))
    assert arbiter.owner == ControlOwner.JOYSTICK_ACQUIRING


def test_mark_held_and_releasing_transitions():
    arbiter = ControlArbiter()
    offboard = FakeOffboardCtrl(state=MissionState.IDLE)
    run(arbiter.begin_joystick_acquire(offboard))
    arbiter.mark_joystick_active("sess1", "lease1")
    arbiter.mark_joystick_held()
    assert arbiter.owner == ControlOwner.JOYSTICK_HELD
    assert arbiter.joystick_owned is True
    arbiter.mark_releasing()
    assert arbiter.owner == ControlOwner.RELEASING
    assert arbiter.joystick_owned is True


def test_ensure_mission_motion_allowed_rejects_when_joystick_owns():
    arbiter = ControlArbiter()
    offboard = FakeOffboardCtrl(state=MissionState.IDLE)
    run(arbiter.begin_joystick_acquire(offboard))
    arbiter.mark_joystick_active("sess1", "lease1")
    with pytest.raises(ControlArbiterError) as exc:
        run(arbiter.ensure_mission_motion_allowed(offboard))
    assert exc.value.code == "joystick_active"


def test_ensure_mission_motion_allowed_rejects_mid_transition_states():
    arbiter = ControlArbiter()
    for state in (MissionState.ARMING, MissionState.SWITCHING_OFFBOARD, MissionState.STOPPING):
        offboard = FakeOffboardCtrl(state=state)
        with pytest.raises(ControlArbiterError) as exc:
            run(arbiter.ensure_mission_motion_allowed(offboard))
        assert exc.value.code == "mission_transition"


def test_ensure_mission_motion_allowed_passes_when_idle():
    arbiter = ControlArbiter()
    offboard = FakeOffboardCtrl(state=MissionState.RUNNING)
    run(arbiter.ensure_mission_motion_allowed(offboard))  # no raise


def test_snapshot_fields():
    arbiter = ControlArbiter()
    offboard = FakeOffboardCtrl(state=MissionState.IDLE)
    run(arbiter.begin_joystick_acquire(offboard))
    arbiter.mark_joystick_active("sess1", "lease1")
    snap = arbiter.snapshot()
    assert snap["control_owner"] == "joystick_active"
    assert snap["joystick_owned"] is True
    assert snap["joystick_owner_present"] is True
    assert snap["joystick_has_lease"] is True


def test_module_singleton_get_and_reset():
    a = get_control_arbiter()
    b = get_control_arbiter()
    assert a is b
    c = reset_control_arbiter_for_tests()
    assert c is not a
    assert get_control_arbiter() is c
