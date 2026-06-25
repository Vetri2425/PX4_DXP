import asyncio
import os
import sys
from collections import deque

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from control_arbiter import ControlArbiter, ControlOwner, reset_control_arbiter_for_tests
from emergency import EmergencyHandler
from joystick_controller import JoystickController, JoystickError, JoystickState
from manual_control_gateway import ManualControlGateway, NEUTRAL_FRAME
from models import MissionState
from offboard_controller import OffboardController


@pytest.fixture(autouse=True)
def clean_global_arbiter():
    reset_control_arbiter_for_tests()
    yield
    reset_control_arbiter_for_tests()


class FakeTransport:
    name = "fake"

    def __init__(self, healthy=True):
        self.healthy = healthy
        self.frames = []

    def is_healthy(self):
        return self.healthy

    def health_reason(self):
        return "" if self.healthy else "fake unhealthy"

    def send_frame(self, frame):
        self.frames.append(frame)

    def shutdown(self):
        pass


class FakeRos:
    def __init__(self, *, connected=True, armed=True, mode="MANUAL"):
        self.state = {"connected": connected, "armed": armed, "mode": mode}
        self.calls = []

    def get_state(self):
        return dict(self.state)

    async def set_mode_async(self, mode):
        self.calls.append(("set_mode", mode))
        self.state["mode"] = mode
        return True, ""


def make_controller(
    *,
    ros=None,
    transport=None,
    arbiter=None,
    manual_enabled=True,
    command_rate_hz=1000000000.0,
    mode_confirm_timeout_s=3.0,
):
    ros = ros or FakeRos()
    transport = transport or FakeTransport()
    gateway = ManualControlGateway(transport, rate_hz=1000.0, stale_timeout_s=0.05)
    offboard = type("Offboard", (), {"state": MissionState.IDLE})()
    ctrl = JoystickController(
        ros,
        offboard,
        gateway,
        arbiter=arbiter or ControlArbiter(),
        manual_enabled=manual_enabled,
        neutral_prestream_s=0.0,
        mode_confirm_timeout_s=mode_confirm_timeout_s,
        command_rate_hz=command_rate_hz,
        server_stop_timeout_s=0.03,
        lease_revoke_timeout_s=0.12,
        gateway_stale_timeout_s=0.05,
    )
    return ctrl, gateway, transport, ros, offboard


def run(coro):
    return asyncio.run(coro)


async def acquire(ctrl, sid="sid", session_id="session"):
    return await ctrl.acquire(
        sid,
        {
            "session_id": session_id,
            "client_monotonic_ms": 1,
        },
    )


def command(lease_id, **overrides):
    data = {
        "session_id": "session",
        "lease_id": lease_id,
        "sequence": 1,
        "client_monotonic_ms": 10,
        "deadman": True,
        "throttle": 0.8,
        "steering": -0.8,
    }
    data.update(overrides)
    return data


def test_acquire_rejects_when_manual_authorization_disabled():
    ctrl, *_ = make_controller(manual_enabled=False)
    with pytest.raises(JoystickError) as exc:
        run(acquire(ctrl))
    assert exc.value.code == "manual_control_disabled"


def test_acquire_rejects_when_transport_unhealthy():
    ctrl, *_ = make_controller(transport=FakeTransport(healthy=False))
    with pytest.raises(JoystickError) as exc:
        run(acquire(ctrl))
    assert exc.value.code == "transport_unavailable"


def test_acquire_streams_neutral_before_manual_and_issues_lease_after_confirmed_mode():
    ctrl, gateway, transport, ros, _ = make_controller(ros=FakeRos(mode="POSCTL"))
    result = run(acquire(ctrl))
    assert result["lease_id"]
    assert ros.calls == [("set_mode", "MANUAL")]
    assert transport.frames[0] == NEUTRAL_FRAME
    assert ctrl.is_active is True


def test_valid_command_clamps_to_configured_manual_limits():
    ctrl, gateway, _, _, _ = make_controller()
    lease = run(acquire(ctrl))["lease_id"]
    result = ctrl.handle_command("sid", command(lease))
    assert result["throttle"] == 0.15
    assert result["steering"] == -0.5
    assert gateway.snapshot()["gateway_last_frame"]["z"] == 575
    assert gateway.snapshot()["gateway_last_frame"]["y"] == -500


def test_rejected_command_does_not_refresh_watchdogs_or_last_command():
    ctrl, gateway, _, _, _ = make_controller()
    lease = run(acquire(ctrl))["lease_id"]
    ctrl.handle_command("sid", command(lease))
    before = ctrl.snapshot()
    with pytest.raises(JoystickError) as exc:
        ctrl.handle_command("sid", command(lease, sequence=2, throttle=2.0))
    after = ctrl.snapshot()
    assert exc.value.code == "out_of_range"
    assert after["joystick_commanded_throttle"] == before["joystick_commanded_throttle"]
    assert after["joystick_commanded_steering"] == before["joystick_commanded_steering"]
    assert gateway.snapshot()["gateway_last_frame"]["z"] == 575


def test_duplicate_sequence_rejected_before_refresh():
    ctrl, _, _, _, _ = make_controller()
    lease = run(acquire(ctrl))["lease_id"]
    ctrl.handle_command("sid", command(lease))
    with pytest.raises(JoystickError) as exc:
        ctrl.handle_command("sid", command(lease, deadman=False))
    assert exc.value.code == "out_of_order"
    assert ctrl.snapshot()["joystick_deadman"] is True


def test_wrong_sid_session_or_lease_rejected():
    ctrl, _, _, _, _ = make_controller()
    lease = run(acquire(ctrl))["lease_id"]
    with pytest.raises(JoystickError) as exc:
        ctrl.handle_command("other-sid", command(lease))
    assert exc.value.code == "not_owner"
    with pytest.raises(JoystickError) as exc:
        ctrl.handle_command("sid", command(lease, session_id="other-session"))
    assert exc.value.code == "not_owner"
    with pytest.raises(JoystickError) as exc:
        ctrl.handle_command("sid", command("other-lease"))
    assert exc.value.code == "not_owner"


def test_command_rejected_if_mode_leaves_manual():
    ctrl, _, _, ros, _ = make_controller()
    lease = run(acquire(ctrl))["lease_id"]
    ros.state["mode"] = "OFFBOARD"
    with pytest.raises(JoystickError) as exc:
        ctrl.handle_command("sid", command(lease))
    assert exc.value.code == "mode_unavailable"


def test_deadman_false_enters_held_and_newer_true_resumes():
    ctrl, gateway, _, _, _ = make_controller()
    lease = run(acquire(ctrl))["lease_id"]
    ctrl.handle_command("sid", command(lease, deadman=False))
    assert ctrl.snapshot()["joystick_state"] == "held"
    assert gateway.snapshot()["gateway_last_frame"] == {
        "x": 0,
        "y": 0,
        "z": 500,
        "r": 0,
        "buttons": 0,
    }
    ctrl.handle_command("sid", command(lease, sequence=2, client_monotonic_ms=11, deadman=True))
    assert ctrl.snapshot()["joystick_state"] == "active"


def test_release_stays_manual_and_neutral_without_offboard_switch():
    ctrl, gateway, transport, ros, _ = make_controller()
    lease = run(acquire(ctrl))["lease_id"]
    ctrl.handle_command("sid", command(lease))
    result = run(ctrl.release("sid", session_id="session", lease_id=lease))
    assert result["reason"] == "explicit"
    assert ros.calls == [("set_mode", "MANUAL")]
    assert transport.frames[-1] == NEUTRAL_FRAME
    assert ros.state["mode"] == "MANUAL"
    assert ctrl.is_active is False


def test_server_watchdog_neutralizes_then_revokes_lease():
    ctrl, gateway, transport, _, _ = make_controller()

    async def scenario():
        lease = (await acquire(ctrl))["lease_id"]
        ctrl.handle_command("sid", command(lease))
        await asyncio.sleep(0.16)
        return ctrl.snapshot()

    snap = run(scenario())
    assert snap["joystick_active"] is False
    assert snap["joystick_stop_reason"] == "lease_timeout"
    assert transport.frames[-1] == NEUTRAL_FRAME


def test_mission_start_rejected_while_joystick_active():
    arbiter = reset_control_arbiter_for_tests()
    arbiter.mark_joystick_active("session", "lease")
    ros = FakeRos(mode="MANUAL")
    offboard = OffboardController(ros, deque())
    offboard.load_path([(0.0, 0.0), (1.0, 0.0)], name="test")
    ok, message = run(offboard.start_async())
    assert ok is False
    assert "joystick owns manual control" in message
    assert ros.calls == []


def test_estop_clears_mission_arbiter_owner():
    arbiter = reset_control_arbiter_for_tests()
    arbiter._owner = ControlOwner.MISSION

    class FakeOffboard:
        _lock = asyncio.Lock()
        state = MissionState.RUNNING

    class FakeRos:
        def publish_stop_path(self):
            return (0.0, 0.0)

        async def set_mode_async(self, mode):
            return True, ""

        async def arm_async(self, arm):
            return True, ""

    handler = EmergencyHandler(FakeRos(), FakeOffboard(), deque())
    run(handler.estop_async())
    assert arbiter.owner == ControlOwner.IDLE


def test_acquire_rejects_if_interrupted_before_lease():
    ctrl, gateway, transport, _, _ = make_controller(ros=FakeRos(mode="POSCTL"))

    async def interrupt_after_mode_confirm(self, mode, timeout_s):
        self._owner_sid = None
        self._state = JoystickState.INACTIVE
        return True

    ctrl._wait_for_mode = interrupt_after_mode_confirm.__get__(ctrl, JoystickController)

    with pytest.raises(JoystickError) as exc:
        run(acquire(ctrl, sid="sid-a"))
    assert exc.value.code == "acquire_cancelled"
    assert ctrl.lease_id is None
    assert transport.frames[-1] == NEUTRAL_FRAME


def test_same_task_nested_hold_remains_reentrant():
    arbiter = ControlArbiter()

    async def scenario():
        async with arbiter.hold():
            async with arbiter.hold():
                return "nested"

    assert run(asyncio.wait_for(scenario(), timeout=0.2)) == "nested"


def test_child_task_created_inside_hold_cannot_bypass_lock():
    arbiter = ControlArbiter()
    entered = asyncio.Event()

    async def child():
        async with arbiter.hold():
            entered.set()

    async def scenario():
        async with arbiter.hold():
            task = asyncio.create_task(child())
            await asyncio.sleep(0.05)
            assert entered.is_set() is False
        await asyncio.wait_for(entered.wait(), timeout=0.2)
        await task

    run(scenario())


def test_watchdog_task_does_not_inherit_arbiter_context_and_blocks_on_real_lock():
    arbiter = ControlArbiter()
    ctrl, _, _, _, _ = make_controller(arbiter=arbiter)

    async def scenario():
        await acquire(ctrl)
        lock_released = asyncio.Event()

        async def blocker():
            async with arbiter.hold():
                await asyncio.sleep(0.18)
            lock_released.set()

        blocking_task = asyncio.create_task(blocker())
        await asyncio.sleep(0.15)
        assert ctrl.lease_id is not None
        assert ctrl.snapshot()["control_owner"] == ControlOwner.JOYSTICK_ACTIVE.value
        await asyncio.wait_for(lock_released.wait(), timeout=0.3)
        await blocking_task
        for _ in range(10):
            if ctrl.lease_id is None:
                break
            await asyncio.sleep(0.02)
        assert ctrl.lease_id is None
        assert ctrl.snapshot()["control_owner"] == ControlOwner.IDLE.value

    run(scenario())


def test_mission_and_joystick_race_has_one_winner_without_deadlock():
    arbiter = ControlArbiter()
    offboard = type("Offboard", (), {"state": MissionState.IDLE})()
    results = []

    async def joystick_task():
        async with arbiter.joystick_acquire(offboard):
            arbiter.mark_joystick_active("session", "lease")
            results.append("joystick")
            await asyncio.sleep(0.02)

    async def mission_task():
        await asyncio.sleep(0)
        try:
            async with arbiter.mission_start(offboard):
                results.append("mission")
        except Exception as exc:
            results.append(getattr(exc, "code", "error"))

    async def run_race():
        await asyncio.wait_for(asyncio.gather(joystick_task(), mission_task()), 0.5)

    run(run_race())
    assert results.count("joystick") == 1
    assert "joystick_active" in results


def test_snapshot_redacts_owner_session_id():
    ctrl, _, _, _, _ = make_controller()
    run(acquire(ctrl, session_id="private-session"))
    snap = ctrl.snapshot()
    assert "joystick_owner_session_id" not in snap
    assert snap["joystick_owner_present"] is True


def test_estop_starts_physical_action_while_acquire_waits_for_manual(monkeypatch):
    import main

    class BlockingRos(FakeRos):
        def __init__(self):
            super().__init__(mode="POSCTL")
            self.first_set_mode_started = asyncio.Event()
            self.first_set_mode_release = asyncio.Event()
            self.stop_path_called = asyncio.Event()
            self.disarm_called = asyncio.Event()
            self.stop_path_at = None
            self.disarm_at = None

        async def set_mode_async(self, mode):
            self.calls.append(("set_mode", mode))
            if len([call for call in self.calls if call[0] == "set_mode"]) == 1:
                self.first_set_mode_started.set()
                await self.first_set_mode_release.wait()
            self.state["mode"] = mode
            return True, ""

        def publish_stop_path(self):
            self.stop_path_at = asyncio.get_running_loop().time()
            self.calls.append(("publish_stop_path",))
            self.stop_path_called.set()
            return (0.0, 0.0)

        async def arm_async(self, arm):
            self.disarm_at = asyncio.get_running_loop().time()
            self.calls.append(("arm", arm))
            self.disarm_called.set()
            return True, ""

    class FakeOffboard:
        _lock = asyncio.Lock()
        state = MissionState.RUNNING

    async def scenario():
        ros = BlockingRos()
        ctrl, _, _, _, _ = make_controller(ros=ros, mode_confirm_timeout_s=1.0)
        monkeypatch.setattr(main, "joystick_ctrl", ctrl, raising=False)
        handler = EmergencyHandler(ros, FakeOffboard(), deque())
        acquire_task = asyncio.create_task(acquire(ctrl))
        await asyncio.wait_for(ros.first_set_mode_started.wait(), timeout=0.2)
        started_at = asyncio.get_running_loop().time()
        estop_task = asyncio.create_task(handler.estop_async())
        await asyncio.wait_for(ros.stop_path_called.wait(), timeout=0.1)
        await asyncio.wait_for(ros.disarm_called.wait(), timeout=0.1)
        assert ros.stop_path_at - started_at < 0.1
        assert ros.disarm_at - started_at < 0.1
        ros.first_set_mode_release.set()
        await estop_task
        with pytest.raises(JoystickError) as exc:
            await acquire_task
        assert exc.value.code == "acquire_cancelled"
        assert ctrl.lease_id is None

    run(scenario())
