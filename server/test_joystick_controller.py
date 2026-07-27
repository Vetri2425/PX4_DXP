"""JoystickController lease FSM unit tests (plan phase J1).

No ROS, no hardware — fakes stand in for RosBridgeNode / OffboardController /
ManualControlGateway. Covers: acquire/release FSM, owner/sequence/replay/
range/rate validation, deadman→HELD forcing zeros, and the watchdog's
neutral-then-revoke behaviour. JOYSTICK_MANUAL_ENABLED itself is never
touched here — these tests instantiate JoystickController directly with
manual_enabled=True, independent of the deployment default (config stays 0
until phase J3's firmware gates pass).
"""
from __future__ import annotations

import asyncio
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from control_arbiter import ControlArbiter, ControlArbiterError, ControlOwner
from joystick_controller import JoystickController, JoystickError, JoystickState
from models import MissionState


class FakeRosNode:
    def __init__(self, *, connected=True, armed=True, mode="OFFBOARD", arm_ok=True):
        self._state = {"connected": connected, "armed": armed, "mode": mode}
        self.mode_calls: list[str] = []
        self.arm_calls: list[bool] = []
        self._arm_ok = arm_ok

    def get_state(self):
        return dict(self._state)

    async def set_mode_async(self, mode):
        self.mode_calls.append(mode)
        self._state["mode"] = mode
        return True, ""

    async def arm_async(self, arm):
        self.arm_calls.append(bool(arm))
        if not self._arm_ok:
            return False, "fake arm rejected"
        self._state["armed"] = bool(arm)
        return True, ""


class FakeOffboardCtrl:
    def __init__(self, state=MissionState.IDLE):
        self.state = state


class FakeGateway:
    def __init__(self, *, healthy=True):
        self._healthy = healthy
        self.commands: list[tuple[float, float]] = []
        self.neutral_sends = 0
        self.activated = False
        self.deactivated = False

    def is_healthy(self):
        return self._healthy

    def health_reason(self):
        return "" if self._healthy else "fake transport down"

    def activate_neutral(self):
        self.activated = True

    def deactivate_neutral(self):
        self.deactivated = True

    def wait_neutral_barrier(self, duration_s):
        pass

    def accept_command(self, throttle, steering):
        self.commands.append((throttle, steering))

    def send_neutral(self, *, refresh=False):
        self.neutral_sends += 1

    def snapshot(self):
        return {"transport": "fake", "transport_healthy": self._healthy}


def run(coro):
    return asyncio.run(coro)


def _controller(**overrides):
    node = overrides.pop("node", None) or FakeRosNode()
    offboard = overrides.pop("offboard", None) or FakeOffboardCtrl()
    gateway = overrides.pop("gateway", None) or FakeGateway()
    arbiter = overrides.pop("arbiter", None) or ControlArbiter()
    overrides.setdefault("manual_enabled", True)
    ctrl = JoystickController(node, offboard, gateway, arbiter=arbiter, **overrides)
    return ctrl, node, offboard, gateway, arbiter


def _cmd(lease_id, session_id="s1", sequence=1, mono=1000, deadman=True,
         throttle=0.5, steering=0.1):
    return {
        "session_id": session_id,
        "lease_id": lease_id,
        "sequence": sequence,
        "client_monotonic_ms": mono,
        "deadman": deadman,
        "throttle": throttle,
        "steering": steering,
    }


def _active_controller(**overrides):
    ctrl, node, offboard, gateway, arbiter = _controller(**overrides)
    acquired = run(ctrl.acquire("sid1", {"session_id": "s1"}))
    return ctrl, node, gateway, arbiter, acquired["lease_id"]


# ── Acquire / FSM ────────────────────────────────────────────────────────────


def test_acquire_disabled_rejects():
    ctrl, *_ = _controller(manual_enabled=False)
    with pytest.raises(JoystickError) as exc:
        run(ctrl.acquire("sid1", {"session_id": "s1"}))
    assert exc.value.code == "manual_control_disabled"


def test_acquire_requires_session_id():
    ctrl, *_ = _controller()
    with pytest.raises(JoystickError) as exc:
        run(ctrl.acquire("sid1", {}))
    assert exc.value.code == "malformed"


def test_acquire_auto_arms_disarmed_fcu():
    node = FakeRosNode(armed=False)
    ctrl, node, offboard, gateway, arbiter = _controller(node=node)
    result = run(ctrl.acquire("sid1", {"session_id": "s1"}))
    assert result["type"] == "joystick_acquired"
    assert node.arm_calls == [True]
    assert node.get_state()["armed"] is True
    # arm must happen only after MANUAL is requested (neutral already streaming)
    assert node.mode_calls == ["MANUAL"]
    run(ctrl.force_release())


def test_acquire_requires_armed_fcu_when_auto_arm_disabled():
    node = FakeRosNode(armed=False)
    ctrl, node, offboard, gateway, arbiter = _controller(
        node=node, auto_arm_enabled=False
    )
    with pytest.raises(JoystickError) as exc:
        run(ctrl.acquire("sid1", {"session_id": "s1"}))
    assert exc.value.code == "not_armed"
    assert node.arm_calls == []
    assert ctrl.snapshot()["joystick_state"] == JoystickState.INACTIVE.value
    assert arbiter.owner == ControlOwner.IDLE  # claim rolled back, not stuck ACQUIRING


def test_acquire_arm_rejection_rolls_back():
    node = FakeRosNode(armed=False, arm_ok=False)
    ctrl, node, offboard, gateway, arbiter = _controller(node=node)
    with pytest.raises(JoystickError) as exc:
        run(ctrl.acquire("sid1", {"session_id": "s1"}))
    assert exc.value.code == "arm_failed"
    assert ctrl.snapshot()["joystick_state"] == JoystickState.INACTIVE.value
    assert arbiter.owner == ControlOwner.IDLE
    assert gateway.deactivated is True


def test_acquire_arm_timeout_disarms_back():
    class NeverArmsNode(FakeRosNode):
        async def arm_async(self, arm):
            self.arm_calls.append(bool(arm))
            return True, ""  # accepted but armed state never appears

    node = NeverArmsNode(armed=False)
    ctrl, node, offboard, gateway, arbiter = _controller(
        node=node, arm_confirm_timeout_s=0.1
    )
    with pytest.raises(JoystickError) as exc:
        run(ctrl.acquire("sid1", {"session_id": "s1"}))
    assert exc.value.code == "arm_failed"
    # best-effort disarm after we initiated the arm
    assert node.arm_calls == [True, False]
    assert arbiter.owner == ControlOwner.IDLE


def test_acquire_requires_connected_fcu():
    node = FakeRosNode(connected=False)
    ctrl, *_ = _controller(node=node)
    with pytest.raises(JoystickError) as exc:
        run(ctrl.acquire("sid1", {"session_id": "s1"}))
    assert exc.value.code == "fcu_disconnected"


def test_acquire_requires_healthy_transport():
    gateway = FakeGateway(healthy=False)
    ctrl, *_ = _controller(gateway=gateway)
    with pytest.raises(JoystickError) as exc:
        run(ctrl.acquire("sid1", {"session_id": "s1"}))
    assert exc.value.code == "transport_unavailable"


def test_acquire_success_switches_manual_and_activates():
    ctrl, node, offboard, gateway, arbiter = _controller()
    result = run(ctrl.acquire("sid1", {"session_id": "s1"}))
    try:
        assert result["type"] == "joystick_acquired"
        assert ctrl.snapshot()["joystick_state"] == JoystickState.ACTIVE.value
        assert node.mode_calls == ["MANUAL"]
        assert gateway.activated is True
        assert arbiter.owner == ControlOwner.JOYSTICK_ACTIVE
    finally:
        run(ctrl.release("sid1", force=True))


def test_acquire_rejects_while_mission_active():
    offboard = FakeOffboardCtrl(state=MissionState.RUNNING)
    ctrl, *_ = _controller(offboard=offboard)
    with pytest.raises(ControlArbiterError) as exc:
        run(ctrl.acquire("sid1", {"session_id": "s1"}))
    assert exc.value.code == "mission_active"


def test_second_acquire_rejected_while_first_active():
    ctrl, node, offboard, gateway, arbiter = _controller()
    run(ctrl.acquire("sid1", {"session_id": "s1"}))
    try:
        with pytest.raises(ControlArbiterError) as exc:
            run(ctrl.acquire("sid2", {"session_id": "s2"}))
        assert exc.value.code == "joystick_active"
    finally:
        run(ctrl.release("sid1", force=True))


def test_mission_start_rejected_while_joystick_active():
    ctrl, node, offboard, gateway, arbiter = _controller()
    run(ctrl.acquire("sid1", {"session_id": "s1"}))
    try:
        async def try_mission():
            async with arbiter.mission_start(offboard):
                pass

        with pytest.raises(ControlArbiterError) as exc:
            run(try_mission())
        assert exc.value.code == "joystick_active"
    finally:
        run(ctrl.release("sid1", force=True))


# ── handle_command validation ───────────────────────────────────────────────


def test_handle_command_wrong_sid_rejected():
    ctrl, node, gateway, arbiter, lease_id = _active_controller()
    try:
        with pytest.raises(JoystickError) as exc:
            ctrl.handle_command("wrong_sid", _cmd(lease_id))
        assert exc.value.code == "not_owner"
    finally:
        run(ctrl.release("sid1", force=True))


def test_handle_command_wrong_session_rejected():
    ctrl, node, gateway, arbiter, lease_id = _active_controller()
    try:
        with pytest.raises(JoystickError) as exc:
            ctrl.handle_command("sid1", _cmd(lease_id, session_id="bogus"))
        assert exc.value.code == "not_owner"
    finally:
        run(ctrl.release("sid1", force=True))


def test_handle_command_wrong_lease_rejected():
    ctrl, node, gateway, arbiter, lease_id = _active_controller()
    try:
        with pytest.raises(JoystickError) as exc:
            ctrl.handle_command("sid1", _cmd("bogus-lease"))
        assert exc.value.code == "not_owner"
    finally:
        run(ctrl.release("sid1", force=True))


def test_handle_command_sequence_must_strictly_increase():
    ctrl, node, gateway, arbiter, lease_id = _active_controller()
    try:
        ctrl.handle_command("sid1", _cmd(lease_id, sequence=1))
        with pytest.raises(JoystickError) as exc:
            ctrl.handle_command("sid1", _cmd(lease_id, sequence=1))
        assert exc.value.code == "out_of_order"
        with pytest.raises(JoystickError) as exc2:
            ctrl.handle_command("sid1", _cmd(lease_id, sequence=0))
        assert exc2.value.code == "out_of_order"
    finally:
        run(ctrl.release("sid1", force=True))


def test_handle_command_client_monotonic_replay_rejected():
    ctrl, node, gateway, arbiter, lease_id = _active_controller()
    try:
        ctrl.handle_command("sid1", _cmd(lease_id, sequence=1, mono=1000))
        with pytest.raises(JoystickError) as exc:
            ctrl.handle_command("sid1", _cmd(lease_id, sequence=2, mono=500))
        assert exc.value.code == "replay"
    finally:
        run(ctrl.release("sid1", force=True))


def test_handle_command_out_of_range_rejected():
    ctrl, node, gateway, arbiter, lease_id = _active_controller()
    try:
        with pytest.raises(JoystickError) as exc:
            ctrl.handle_command("sid1", _cmd(lease_id, throttle=1.5))
        assert exc.value.code == "out_of_range"
    finally:
        run(ctrl.release("sid1", force=True))


def test_handle_command_nan_rejected():
    ctrl, node, gateway, arbiter, lease_id = _active_controller()
    try:
        with pytest.raises(JoystickError) as exc:
            ctrl.handle_command("sid1", _cmd(lease_id, throttle=float("nan")))
        assert exc.value.code == "nan_value"
    finally:
        run(ctrl.release("sid1", force=True))


def test_handle_command_clamps_to_configured_limits():
    ctrl, node, gateway, arbiter, lease_id = _active_controller(
        max_abs_throttle=0.1, max_abs_steering=0.2
    )
    try:
        result = ctrl.handle_command("sid1", _cmd(lease_id, throttle=1.0, steering=-1.0))
        assert result["throttle"] == pytest.approx(0.1)
        assert result["steering"] == pytest.approx(-0.2)
        assert gateway.commands[-1] == (pytest.approx(0.1), pytest.approx(-0.2))
    finally:
        run(ctrl.release("sid1", force=True))


def test_handle_command_deadman_false_forces_zeros_and_held():
    ctrl, node, gateway, arbiter, lease_id = _active_controller()
    try:
        result = ctrl.handle_command(
            "sid1", _cmd(lease_id, deadman=False, throttle=0.5, steering=0.5)
        )
        assert result["throttle"] == 0.0
        assert result["steering"] == 0.0
        assert result["state"] == JoystickState.HELD.value
        assert ctrl.snapshot()["joystick_state"] == JoystickState.HELD.value
        assert arbiter.owner == ControlOwner.JOYSTICK_HELD
    finally:
        run(ctrl.release("sid1", force=True))


def test_handle_command_deadman_reasserted_returns_to_active():
    ctrl, node, gateway, arbiter, lease_id = _active_controller()
    try:
        ctrl.handle_command("sid1", _cmd(lease_id, sequence=1, deadman=False))
        assert ctrl.snapshot()["joystick_state"] == JoystickState.HELD.value
        ctrl.handle_command("sid1", _cmd(lease_id, sequence=2, deadman=True))
        assert ctrl.snapshot()["joystick_state"] == JoystickState.ACTIVE.value
        assert arbiter.owner == ControlOwner.JOYSTICK_ACTIVE
    finally:
        run(ctrl.release("sid1", force=True))


def test_handle_command_rejected_when_not_in_manual_mode():
    ctrl, node, gateway, arbiter, lease_id = _active_controller()
    try:
        node._state["mode"] = "OFFBOARD"
        with pytest.raises(JoystickError) as exc:
            ctrl.handle_command("sid1", _cmd(lease_id))
        assert exc.value.code == "mode_unavailable"
    finally:
        node._state["mode"] = "MANUAL"
        run(ctrl.release("sid1", force=True))


def test_handle_command_transport_unhealthy_rejected():
    ctrl, node, gateway, arbiter, lease_id = _active_controller()
    try:
        gateway._healthy = False
        with pytest.raises(JoystickError) as exc:
            ctrl.handle_command("sid1", _cmd(lease_id))
        assert exc.value.code == "transport_unavailable"
    finally:
        gateway._healthy = True
        run(ctrl.release("sid1", force=True))


def test_handle_command_rejected_before_lease_active():
    ctrl, *_ = _controller()
    with pytest.raises(JoystickError) as exc:
        ctrl.handle_command("sid1", _cmd("some-lease"))
    assert exc.value.code == "not_owner"


# ── release / arbiter cleanup ───────────────────────────────────────────────


def test_release_by_owner_clears_state_and_arbiter():
    ctrl, node, offboard, gateway, arbiter = _controller()
    run(ctrl.acquire("sid1", {"session_id": "s1"}))
    result = run(ctrl.release("sid1"))
    assert result["type"] == "joystick_released"
    assert ctrl.snapshot()["joystick_state"] == JoystickState.INACTIVE.value
    assert arbiter.owner == ControlOwner.IDLE
    assert gateway.deactivated is True


def test_release_by_non_owner_rejected():
    ctrl, node, offboard, gateway, arbiter = _controller()
    run(ctrl.acquire("sid1", {"session_id": "s1"}))
    try:
        with pytest.raises(JoystickError) as exc:
            run(ctrl.release("intruder"))
        assert exc.value.code == "not_owner"
    finally:
        run(ctrl.release("sid1", force=True))


def test_release_frees_arbiter_for_a_new_acquire():
    ctrl, node, offboard, gateway, arbiter = _controller()
    run(ctrl.acquire("sid1", {"session_id": "s1"}))
    run(ctrl.release("sid1"))
    result = run(ctrl.acquire("sid2", {"session_id": "s2"}))  # must not raise
    assert result["type"] == "joystick_acquired"
    run(ctrl.release("sid2", force=True))


def test_emergency_neutralize_clears_arbiter_ownership():
    """Reference gap fix: e-stop must not strand the arbiter at
    JOYSTICK_ACTIVE — the reference left the arbiter owner untouched here,
    which would block every future mission start forever.

    emergency_neutralize() is called synchronously from
    EmergencyHandler.estop_async() on the server's single long-lived event
    loop (the same loop that owns the joystick watchdog task), so acquire and
    emergency_neutralize run inside one asyncio.run() here to match that —
    not two separate ones, which would tear the loop down between calls and
    orphan the watchdog task's cancellation.
    """
    ctrl, node, offboard, gateway, arbiter = _controller()

    async def scenario():
        await ctrl.acquire("sid1", {"session_id": "s1"})
        ctrl.emergency_neutralize(reason="estop")

    run(scenario())
    assert ctrl.snapshot()["joystick_state"] == JoystickState.INACTIVE.value
    assert arbiter.owner == ControlOwner.IDLE
    assert gateway.neutral_sends >= 1


# ── Watchdog ─────────────────────────────────────────────────────────────────


def test_watchdog_forces_neutral_after_server_stop_timeout():
    ctrl, node, offboard, gateway, arbiter = _controller(
        server_stop_timeout_s=0.05, lease_revoke_timeout_s=5.0, lease_expiry_s=30.0,
    )

    async def scenario():
        await ctrl.acquire("sid1", {"session_id": "s1"})
        await asyncio.sleep(0.15)
        assert gateway.neutral_sends >= 1
        assert ctrl.snapshot()["joystick_stop_reason"] == "server_timeout_neutral"
        assert ctrl.snapshot()["joystick_state"] == JoystickState.ACTIVE.value  # not revoked yet
        await ctrl.release("sid1", force=True)

    run(scenario())


def test_watchdog_revokes_lease_after_revoke_timeout():
    ctrl, node, offboard, gateway, arbiter = _controller(
        server_stop_timeout_s=0.05, lease_revoke_timeout_s=0.15, lease_expiry_s=30.0,
    )

    async def scenario():
        await ctrl.acquire("sid1", {"session_id": "s1"})
        await asyncio.sleep(0.35)
        assert ctrl.snapshot()["joystick_state"] == JoystickState.INACTIVE.value
        assert ctrl.snapshot()["joystick_stop_reason"] == "lease_timeout"
        assert arbiter.owner == ControlOwner.IDLE

    run(scenario())


def test_watchdog_stops_when_lease_released_explicitly():
    ctrl, node, offboard, gateway, arbiter = _controller(
        server_stop_timeout_s=5.0, lease_revoke_timeout_s=10.0, lease_expiry_s=30.0,
    )

    async def scenario():
        await ctrl.acquire("sid1", {"session_id": "s1"})
        await ctrl.release("sid1")
        neutral_sends_after_release = gateway.neutral_sends
        await asyncio.sleep(0.1)
        # No further watchdog-driven neutral sends once released/cancelled.
        assert gateway.neutral_sends == neutral_sends_after_release

    run(scenario())
