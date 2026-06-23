"""Unit tests for AsyncRTKManager LoRa lifecycle and supervision."""

from __future__ import annotations

import asyncio
import json
import os
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from config import LORA_MAX_RESTARTS_PER_MIN, LORA_NO_DATA_FAIL_S, LORA_NO_DATA_WARN_S
from rtk_manager import (
    AsyncRTKManager,
    LoRaLifecycleState,
    RTKConflictError,
    RTKProcessError,
    RTKValidationError,
)


class FakeClock:
    def __init__(self, start: float = 1000.0):
        self.t = start

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class FakeProcess:
    def __init__(self, pid: int = 4242):
        self.pid = pid
        self.returncode = None
        self._waiters: list[asyncio.Future] = []
        self.terminated = False
        self.killed = False
        self.stdin = None

    def terminate(self) -> None:
        self.terminated = True
        self.returncode = 0
        for fut in self._waiters:
            if not fut.done():
                fut.set_result(0)

    def kill(self) -> None:
        self.killed = True
        self.returncode = -9
        for fut in self._waiters:
            if not fut.done():
                fut.set_result(-9)

    async def wait(self):
        fut = asyncio.get_running_loop().create_future()
        self._waiters.append(fut)
        if self.returncode is not None:
            fut.set_result(self.returncode)
        return await fut

    def exit(self, code: int = 1) -> None:
        self.returncode = code
        for fut in self._waiters:
            if not fut.done():
                fut.set_result(code)


@pytest.fixture
def repo_root(tmp_path):
    lora = tmp_path / "lora_rtcm_node.py"
    lora.write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    ntrip = tmp_path / "ntrip_rtcm_node.py"
    ntrip.write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    return tmp_path


def _write_child_status(
    path: Path,
    session_id: str,
    pid: int,
    *,
    clock: FakeClock | None = None,
    **overrides,
) -> None:
    now = clock() if clock is not None else time.monotonic()
    payload = {
        "session_id": session_id,
        "process_id": pid,
        "mode": "lora",
        "lifecycle_state": "streaming",
        "state": "streaming",
        "serial_open": True,
        "connected": True,
        "valid_frames": 10,
        "invalid_frames": 0,
        "crc_errors": 0,
        "dropped_frames": 0,
        "bytes_received": 1000,
        "bytes_injected": 900,
        "last_valid_frame_time": now,
        "serial_open_since_monotonic": now,
        "valid_frame_rate_hz": None,
        "bytes_per_sec": None,
        "injection_topic_ready": True,
        "updated_at_monotonic": now,
    }
    payload.update(overrides)
    path.write_text(json.dumps(payload), encoding="utf-8")


@pytest.mark.anyio
async def test_lora_start_sets_desired_source(repo_root, monkeypatch):
    clock = FakeClock()
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    status = await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    assert status.desired_source == "lora"
    assert status.serial_port == "/dev/ttyUSB0"
    assert status.baudrate == 115200
    assert status.lifecycle_state == LoRaLifecycleState.STARTING.value


@pytest.mark.anyio
async def test_user_stop_clears_desired_and_disables_restart(repo_root, monkeypatch):
    clock = FakeClock()
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)

    status = await manager.stop_lora()
    assert status.desired_source is None
    assert status.lifecycle_state == LoRaLifecycleState.STOPPED_BY_USER.value
    assert status.stop_reason == "user_stop"
    assert proc.terminated or proc.killed


@pytest.mark.anyio
async def test_process_crash_restarts_and_preserves_session(repo_root, monkeypatch):
    import rtk_manager as rtk_manager_module

    monkeypatch.setattr(rtk_manager_module, "LORA_RECONNECT_INTERVAL_S", 0.1)
    clock = FakeClock()
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
        startup_grace_s=0.01,
    )
    procs: list[FakeProcess] = []

    async def fake_exec(*_cmd, **_kwargs):
        p = FakeProcess(pid=5000 + len(procs))
        procs.append(p)
        return p

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    session_id = manager._lora_session.session_id  # noqa: SLF001

    procs[0].exit(1)
    await asyncio.sleep(1.2)
    status = await manager.status()
    assert status.desired_source == "lora"
    assert status.restart_count >= 1
    assert status.lifecycle_state in {
        LoRaLifecycleState.RECONNECTING.value,
        LoRaLifecycleState.STARTING.value,
        LoRaLifecycleState.CONNECTED.value,
    }
    assert len(procs) >= 2
    assert manager._lora_session.session_id == session_id  # noqa: SLF001


@pytest.mark.anyio
async def test_restart_rate_exhaustion_enters_failed(repo_root, monkeypatch):
    clock = FakeClock()
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
        startup_grace_s=0.01,
    )
    proc = FakeProcess(pid=6000)

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)

    session = manager._lora_session  # noqa: SLF001
    for _ in range(LORA_MAX_RESTARTS_PER_MIN):
        session.restart_timestamps.append(clock())
        clock.advance(0.1)
    proc.exit(1)
    manager._process = None  # noqa: SLF001
    async with manager._lock:  # noqa: SLF001
        if not manager._can_restart_locked(session):  # noqa: SLF001
            session.lifecycle_state = LoRaLifecycleState.FAILED
            session.transport_reason = "restart_rate_exhausted"
    status = await manager.status()
    assert status.lifecycle_state == LoRaLifecycleState.FAILED.value
    assert status.transport_reason == "restart_rate_exhausted"


@pytest.mark.anyio
async def test_no_data_threshold(repo_root, monkeypatch):
    clock = FakeClock(2000.0)
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    status_path = manager._status_file  # noqa: SLF001
    stale_last_valid = clock() - LORA_NO_DATA_WARN_S - 1.0
    _write_child_status(
        status_path,
        manager._lora_session.session_id,  # noqa: SLF001
        proc.pid,
        clock=clock,
        last_valid_frame_time=stale_last_valid,
        lifecycle_state="connected",
    )
    clock.advance(LORA_NO_DATA_WARN_S + 2.0)
    _write_child_status(
        status_path,
        manager._lora_session.session_id,  # noqa: SLF001
        proc.pid,
        clock=clock,
        last_valid_frame_time=stale_last_valid,
        lifecycle_state="connected",
    )
    status = await manager.status()
    assert status.lifecycle_state == LoRaLifecycleState.NO_DATA.value


@pytest.mark.anyio
async def test_valid_frame_restores_streaming(repo_root, monkeypatch):
    clock = FakeClock(3000.0)
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    session = manager._lora_session  # noqa: SLF001
    session.lifecycle_state = LoRaLifecycleState.NO_DATA
    _write_child_status(
        manager._status_file,  # noqa: SLF001
        session.session_id,
        proc.pid,
        clock=clock,
        last_valid_frame_time=clock(),
        lifecycle_state="streaming",
    )
    status = await manager.status()
    assert status.lifecycle_state == LoRaLifecycleState.STREAMING_VALID_RTCM.value


@pytest.mark.anyio
async def test_ntrip_lora_mutual_exclusion(repo_root, monkeypatch):
    clock = FakeClock()
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)

    with pytest.raises(RTKConflictError):
        await manager.start_ntrip(
            host="caster",
            port=2101,
            mountpoint="MP",
            user="u",
            password="p",
        )


@pytest.mark.anyio
async def test_duplicate_lora_start_idempotent(repo_root, monkeypatch):
    clock = FakeClock()
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()
    calls = {"n": 0}

    async def fake_exec(*_cmd, **_kwargs):
        calls["n"] += 1
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    first = await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    second = await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    assert calls["n"] == 1
    assert second.desired_source == "lora"
    assert first.session_started_at == second.session_started_at


@pytest.mark.anyio
async def test_stale_child_status_ignored(repo_root, monkeypatch):
    clock = FakeClock(4000.0)
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    _write_child_status(
        manager._status_file,  # noqa: SLF001
        "stale-session",
        proc.pid,
        valid_frames=9999,
        last_valid_frame_time=clock(),
    )
    status = await manager.status()
    assert status.valid_frames == 0


@pytest.mark.anyio
async def test_invalid_serial_port_rejected():
    manager = AsyncRTKManager()
    with pytest.raises(RTKValidationError):
        await manager.start_lora(serial_port="ttyUSB0", baudrate=115200)


@pytest.mark.anyio
async def test_zero_frames_connected_below_warn(repo_root, monkeypatch):
    clock = FakeClock(5000.0)
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    open_since = clock()
    _write_child_status(
        manager._status_file,  # noqa: SLF001
        manager._lora_session.session_id,  # noqa: SLF001
        proc.pid,
        clock=clock,
        valid_frames=0,
        bytes_received=0,
        last_valid_frame_time=None,
        serial_open_since_monotonic=open_since,
        lifecycle_state="connected",
    )
    status = await manager.status()
    assert status.lifecycle_state == LoRaLifecycleState.CONNECTED.value
    assert status.stream_healthy is None
    assert status.serial_open is True


@pytest.mark.anyio
async def test_zero_frames_enters_no_data(repo_root, monkeypatch):
    clock = FakeClock(6000.0)
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    open_since = clock() - LORA_NO_DATA_WARN_S - 2.0
    _write_child_status(
        manager._status_file,  # noqa: SLF001
        manager._lora_session.session_id,  # noqa: SLF001
        proc.pid,
        clock=clock,
        valid_frames=0,
        bytes_received=0,
        last_valid_frame_time=None,
        serial_open_since_monotonic=open_since,
        lifecycle_state="connected",
    )
    status = await manager.status()
    assert status.lifecycle_state == LoRaLifecycleState.NO_DATA.value
    assert status.stream_healthy is None


@pytest.mark.anyio
async def test_zero_frames_fail_threshold_unhealthy(repo_root, monkeypatch):
    clock = FakeClock(7000.0)
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    open_since = clock() - LORA_NO_DATA_FAIL_S - 1.0
    _write_child_status(
        manager._status_file,  # noqa: SLF001
        manager._lora_session.session_id,  # noqa: SLF001
        proc.pid,
        clock=clock,
        valid_frames=0,
        bytes_received=0,
        last_valid_frame_time=None,
        serial_open_since_monotonic=open_since,
        lifecycle_state="connected",
    )
    status = await manager.status()
    assert status.lifecycle_state == LoRaLifecycleState.NO_DATA.value
    assert status.stream_healthy is False


@pytest.mark.anyio
async def test_invalid_only_stream_reason(repo_root, monkeypatch):
    clock = FakeClock(7100.0)
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    open_since = clock() - LORA_NO_DATA_WARN_S - 1.0
    _write_child_status(
        manager._status_file,  # noqa: SLF001
        manager._lora_session.session_id,  # noqa: SLF001
        proc.pid,
        clock=clock,
        valid_frames=0,
        invalid_frames=12,
        bytes_received=400,
        last_valid_frame_time=None,
        serial_open_since_monotonic=open_since,
        lifecycle_state="connected",
    )
    status = await manager.status()
    assert status.transport_reason == "invalid_stream_only"


@pytest.mark.anyio
async def test_mavros_unavailable_does_not_force_transmitter_silent(repo_root, monkeypatch):
    clock = FakeClock(7200.0)
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    _write_child_status(
        manager._status_file,  # noqa: SLF001
        manager._lora_session.session_id,  # noqa: SLF001
        proc.pid,
        clock=clock,
        valid_frames=5,
        bytes_injected=0,
        injection_topic_ready=False,
        lifecycle_state="streaming",
    )
    status = await manager.status()
    assert status.lifecycle_state == LoRaLifecycleState.STREAMING_VALID_RTCM.value
    assert status.transport_reason is None


@pytest.mark.anyio
async def test_child_rates_consumed_by_manager(repo_root, monkeypatch):
    clock = FakeClock()
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    _write_child_status(
        manager._status_file,  # noqa: SLF001
        manager._lora_session.session_id,  # noqa: SLF001
        proc.pid,
        clock=clock,
        valid_frame_rate_hz=5.0,
        bytes_per_sec=500.0,
    )
    status = await manager.status()
    assert status.valid_frame_rate_hz == 5.0
    assert status.bytes_per_sec == 500.0


def _pending_rtk_tasks() -> list[asyncio.Task]:
    return [
        t
        for t in asyncio.all_tasks()
        if t.get_name() in {"rtk-supervisor", "rtk-lifecycle"}
    ]


@pytest.mark.anyio
async def test_stop_cancels_background_tasks(repo_root, monkeypatch):
    clock = FakeClock()
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    assert manager._supervisor_task is not None  # noqa: SLF001
    await manager.stop_lora()
    assert manager._supervisor_task is None  # noqa: SLF001
    assert manager._lifecycle_task is None  # noqa: SLF001
    assert _pending_rtk_tasks() == []


@pytest.mark.anyio
async def test_stop_while_supervisor_sleeping_no_restart(repo_root, monkeypatch):
    import rtk_manager as rtk_manager_module

    monkeypatch.setattr(rtk_manager_module, "LORA_RECONNECT_INTERVAL_S", 5.0)
    clock = FakeClock()
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
        startup_grace_s=0.01,
    )
    calls = {"n": 0}

    async def fake_exec(*_cmd, **_kwargs):
        calls["n"] += 1
        p = FakeProcess(pid=7000 + calls["n"])
        if calls["n"] == 1:
            return p
        raise AssertionError("supervisor resurrected child after stop")

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    manager._process.exit(1)  # noqa: SLF001
    await asyncio.sleep(0.6)
    await manager.stop_lora()
    await asyncio.sleep(0.2)
    assert calls["n"] == 1


@pytest.mark.anyio
async def test_new_start_after_stop_creates_fresh_tasks(repo_root, monkeypatch):
    clock = FakeClock()
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    procs: list[FakeProcess] = []

    async def fake_exec(*_cmd, **_kwargs):
        p = FakeProcess(pid=8000 + len(procs))
        procs.append(p)
        return p

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    first_supervisor = manager._supervisor_task  # noqa: SLF001
    await manager.stop_lora()
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    second_supervisor = manager._supervisor_task  # noqa: SLF001
    assert first_supervisor is not second_supervisor
    assert not second_supervisor.done()


@pytest.mark.anyio
async def test_repeated_stop_idempotent(repo_root, monkeypatch):
    clock = FakeClock()
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()

    async def fake_exec(*_cmd, **_kwargs):
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    first = await manager.stop_lora()
    second = await manager.stop_lora()
    assert first.lifecycle_state == LoRaLifecycleState.STOPPED_BY_USER.value
    assert second.lifecycle_state == LoRaLifecycleState.STOPPED_BY_USER.value


@pytest.mark.anyio
async def test_shutdown_stops_without_restart(repo_root, monkeypatch):
    clock = FakeClock()
    manager = AsyncRTKManager(
        lora_script=repo_root / "lora_rtcm_node.py",
        ntrip_script=repo_root / "ntrip_rtcm_node.py",
        clock=clock,
    )
    proc = FakeProcess()
    calls = {"n": 0}

    async def fake_exec(*_cmd, **_kwargs):
        calls["n"] += 1
        return proc

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)
    await manager.start_lora(serial_port="/dev/ttyUSB0", baudrate=115200)
    await manager.shutdown()
    proc.exit(1)
    await asyncio.sleep(0.6)
    assert calls["n"] == 1
    status = await manager.status()
    assert status.desired_source is None
    assert manager._supervisor_task is None  # noqa: SLF001
    assert _pending_rtk_tasks() == []