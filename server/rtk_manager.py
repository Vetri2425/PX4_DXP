"""Async subprocess orchestration for RTK correction streams."""

from __future__ import annotations

import asyncio
import json
import logging
import os
import sys
import tempfile
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Literal

from config import (
    LORA_ALLOWED_MESSAGE_TYPES,
    LORA_MAX_BYTES_PER_SEC,
    LORA_MAX_FRAME_SIZE,
    LORA_MAX_FRAMES_PER_SEC,
    LORA_MAX_RESTARTS_PER_MIN,
    LORA_MODULE_DISCONNECT_TIMEOUT_S,
    LORA_NO_DATA_FAIL_S,
    LORA_NO_DATA_WARN_S,
    LORA_RECONNECT_INTERVAL_S,
    RPP_RTK_WAIT,
)

RTKSource = Literal["ntrip", "lora"]
RTKMode = Literal["ntrip", "lora", "idle"]


class LoRaLifecycleState(str, Enum):
    IDLE = "IDLE"
    STARTING = "STARTING"
    CONNECTED = "CONNECTED"
    STREAMING_VALID_RTCM = "STREAMING_VALID_RTCM"
    RECONNECTING = "RECONNECTING"
    NO_DATA = "NO_DATA"
    MODULE_DISCONNECTED = "MODULE_DISCONNECTED"
    FAILED = "FAILED"
    STOPPED_BY_USER = "STOPPED_BY_USER"


LORA_BAUDRATES = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600}


@dataclass
class RTKStatus:
    # Backward-compatible fields
    mode: RTKMode
    pid: int | None
    running: bool
    healthy: bool
    source_state: str
    frames: int
    bytes: int
    last_frame_age_s: float | None
    last_error: str | None

    # Task_03 fields
    active_source: RTKSource | None = None
    desired_source: RTKSource | None = None
    lifecycle_state: str = LoRaLifecycleState.IDLE.value
    serial_port: str | None = None
    baudrate: int | None = None
    session_started_at: str | None = None
    process_alive: bool = False
    serial_open: bool = False
    reconnecting: bool = False
    restart_count: int = 0
    valid_frames: int = 0
    invalid_frames: int = 0
    crc_error_count: int = 0
    dropped_frames: int = 0
    bytes_received: int = 0
    bytes_injected: int = 0
    last_valid_rtcm_age_s: float | None = None
    valid_frame_rate_hz: float | None = None
    bytes_per_sec: float | None = None
    injection_topic_ready: bool | None = None
    stream_healthy: bool | None = None
    transport_reason: str | None = None
    stop_reason: str | None = None
    gps_fix_type: int | None = None
    rtk_fixed: bool | None = None
    rtk_float: bool | None = None
    pose_age_s: float | None = None
    rpp_rtk_wait: bool | None = None


class RTKProcessError(RuntimeError):
    """Raised when an RTK subprocess cannot be started cleanly."""


class RTKConflictError(RuntimeError):
    """Raised when a conflicting RTK source/session is already active."""


class RTKValidationError(ValueError):
    """Raised when RTK start parameters are invalid."""


@dataclass
class _LoraSession:
    session_id: str
    serial_port: str
    baudrate: int
    started_at: str
    reconnect_enabled: bool = True
    restart_count: int = 0
    stop_reason: str | None = None
    lifecycle_state: LoRaLifecycleState = LoRaLifecycleState.STARTING
    transport_reason: str | None = None
    last_error: str | None = None
    no_data_since: float | None = None
    stream_unhealthy_since: float | None = None
    first_serial_open_monotonic: float | None = None
    restart_timestamps: deque[float] = field(default_factory=deque)


class AsyncRTKManager:
    """Owns user-requested RTK injection sessions with LoRa self-healing."""

    def __init__(
        self,
        *,
        ntrip_script: Path | None = None,
        lora_script: Path | None = None,
        python_executable: str | None = None,
        startup_grace_s: float = 0.35,
        shutdown_grace_s: float = 10.0,
        navigation_provider: Callable[[], dict[str, Any]] | None = None,
        clock: Callable[[], float] | None = None,
    ) -> None:
        repo_root = Path(__file__).resolve().parents[1]
        self._ntrip_script = ntrip_script or (repo_root / "ntrip_rtcm_node.py")
        self._lora_script = lora_script or (repo_root / "lora_rtcm_node.py")
        self._python = python_executable or sys.executable or "python3"
        self._startup_grace_s = startup_grace_s
        self._shutdown_grace_s = shutdown_grace_s
        self._navigation_provider = navigation_provider
        self._monotonic = clock or time.monotonic

        self._lock = asyncio.Lock()
        self._process: asyncio.subprocess.Process | None = None
        self._mode: RTKMode = "idle"
        self._desired_source: RTKSource | None = None
        self._lora_session: _LoraSession | None = None
        self._status_file: Path | None = None
        self._supervisor_task: asyncio.Task | None = None
        self._lifecycle_task: asyncio.Task | None = None
        self._user_stop_requested = False
        self._shutting_down = False
        self._log = logging.getLogger("server.rtk_manager")

    async def start_ntrip(
        self,
        *,
        host: str,
        port: int,
        mountpoint: str,
        user: str,
        password: str,
    ) -> RTKStatus:
        if self._desired_source == "lora":
            raise RTKConflictError("LoRa RTK session is active or desired; stop it first")
        args = [
            "--host",
            host,
            "--port",
            str(port),
            "--mountpoint",
            mountpoint,
            "--user",
            user,
            "--pass-stdin",
        ]
        return await self._start_simple("ntrip", self._ntrip_script, args, stdin_payload=f"{password}\n")

    async def start_lora(self, *, baudrate: int, serial_port: str) -> RTKStatus:
        self._validate_lora_start(serial_port, baudrate)
        async with self._lock:
            if self._desired_source == "lora" and self._lora_session is not None:
                if (
                    self._lora_session.serial_port == serial_port
                    and self._lora_session.baudrate == baudrate
                    and self._lora_session.lifecycle_state != LoRaLifecycleState.STOPPED_BY_USER
                ):
                    return self._status_locked()
                raise RTKConflictError(
                    "LoRa session already active with different serial configuration"
                )
            if self._desired_source == "ntrip":
                await self._stop_locked(reason="switching to LoRa")
            elif self._process is not None:
                await self._stop_locked(reason="switching to LoRa")

            self._user_stop_requested = False
            session = _LoraSession(
                session_id=uuid.uuid4().hex,
                serial_port=serial_port,
                baudrate=baudrate,
                started_at=self._iso_utc_now(),
                lifecycle_state=LoRaLifecycleState.STARTING,
            )
            self._lora_session = session
            self._desired_source = "lora"
        await self._cancel_background_tasks()
        async with self._lock:
            await self._launch_lora_locked(session)
            self._ensure_background_tasks_locked()
            return self._status_locked()

    async def stop_lora(self, *, reason: str = "user_stop") -> RTKStatus:
        async with self._lock:
            if self._desired_source != "lora":
                if self._lora_session and self._lora_session.lifecycle_state == LoRaLifecycleState.STOPPED_BY_USER:
                    return self._status_locked()
                return self._status_locked()
            await self._stop_lora_locked(reason=reason)
        await self._cancel_background_tasks()
        async with self._lock:
            return self._status_locked()

    async def stop_all(self) -> RTKStatus:
        async with self._lock:
            if self._desired_source == "lora":
                await self._stop_lora_locked(reason="user_stop")
            else:
                await self._stop_locked(reason="user_stop")
        await self._cancel_background_tasks()
        async with self._lock:
            return self._status_locked()

    async def status(self) -> RTKStatus:
        async with self._lock:
            self._refresh_lifecycle_locked()
            if self._process is not None and self._process.returncode is not None:
                if self._desired_source == "lora" and not self._user_stop_requested:
                    # Unexpected exit; supervisor handles restart.
                    self._process = None
                else:
                    self._clear_process_locked()
            return self._status_locked()

    async def shutdown(self) -> None:
        self._shutting_down = True
        async with self._lock:
            self._user_stop_requested = True
            if self._lora_session is not None:
                self._lora_session.reconnect_enabled = False
            if self._desired_source == "lora":
                await self._stop_lora_locked(reason="shutdown")
            else:
                await self._stop_locked(reason="shutdown")
        await self._cancel_background_tasks()

    def _validate_lora_start(self, serial_port: str, baudrate: int) -> None:
        port = serial_port.strip()
        if not port.startswith("/dev/"):
            raise RTKValidationError("serial_port must be an absolute /dev/ path")
        if ".." in port:
            raise RTKValidationError("serial_port must not contain '..'")
        if baudrate not in LORA_BAUDRATES:
            raise RTKValidationError(
                f"baudrate {baudrate} not in allowed set: {sorted(LORA_BAUDRATES)}"
            )

    async def _start_simple(
        self,
        mode: Literal["ntrip", "lora"],
        script: Path,
        args: list[str],
        *,
        stdin_payload: str | None = None,
    ) -> RTKStatus:
        async with self._lock:
            if not script.exists():
                raise RTKProcessError(f"{mode} script not found: {script}")
            await self._stop_locked(reason=f"starting {mode}")
            self._desired_source = mode
            self._lora_session = None
            self._user_stop_requested = False
            status_file = self._new_status_file(mode)
            args = [*args, "--status-file", str(status_file)]
            process = await self._spawn_process(script, args, stdin_payload=stdin_payload)
            self._process = process
            self._mode = mode
            self._status_file = status_file
            try:
                await asyncio.wait_for(process.wait(), timeout=self._startup_grace_s)
            except asyncio.TimeoutError:
                return self._status_locked()
            rc = process.returncode
            self._clear_process_locked()
            self._desired_source = None
            raise RTKProcessError(f"{mode} RTK subprocess exited immediately with code {rc}")

    async def _launch_lora_locked(self, session: _LoraSession) -> None:
        if not self._lora_script.exists():
            session.lifecycle_state = LoRaLifecycleState.FAILED
            session.last_error = f"lora script not found: {self._lora_script}"
            raise RTKProcessError(session.last_error)

        status_file = self._new_status_file("lora")
        args = [
            "--baudrate",
            str(session.baudrate),
            "--serial-port",
            session.serial_port,
            "--status-file",
            str(status_file),
            "--session-id",
            session.session_id,
            "--reconnect-interval-s",
            str(LORA_RECONNECT_INTERVAL_S),
            "--module-disconnect-timeout-s",
            str(LORA_MODULE_DISCONNECT_TIMEOUT_S),
            "--max-frame-size",
            str(LORA_MAX_FRAME_SIZE),
            "--max-bytes-per-sec",
            str(LORA_MAX_BYTES_PER_SEC),
            "--max-frames-per-sec",
            str(LORA_MAX_FRAMES_PER_SEC),
        ]
        if LORA_ALLOWED_MESSAGE_TYPES:
            args.extend(["--allowed-message-types", LORA_ALLOWED_MESSAGE_TYPES])

        process = await self._spawn_process(self._lora_script, args)
        self._process = process
        self._mode = "lora"
        self._status_file = status_file
        session.lifecycle_state = LoRaLifecycleState.STARTING
        session.transport_reason = None
        session.last_error = None

        try:
            await asyncio.wait_for(process.wait(), timeout=self._startup_grace_s)
        except asyncio.TimeoutError:
            return

        rc = process.returncode
        self._process = None
        self._mode = "idle"
        if self._user_stop_requested:
            return
        session.lifecycle_state = LoRaLifecycleState.FAILED
        session.last_error = f"lora subprocess exited immediately with code {rc}"
        raise RTKProcessError(session.last_error)

    async def _spawn_process(
        self,
        script: Path,
        args: list[str],
        *,
        stdin_payload: str | None = None,
    ) -> asyncio.subprocess.Process:
        safe_args = self._redact_args(args)
        cmd = [self._python, str(script), *args]
        self._log.info("starting RTK subprocess: %s %s", self._python, safe_args)
        process = None
        try:
            process = await asyncio.create_subprocess_exec(
                *cmd,
                cwd=str(script.parent),
                stdin=asyncio.subprocess.PIPE if stdin_payload is not None else asyncio.subprocess.DEVNULL,
            )
            if stdin_payload is not None:
                assert process.stdin is not None
                process.stdin.write(stdin_payload.encode())
                await process.stdin.drain()
                process.stdin.close()
            return process
        except Exception as exc:
            if process is not None:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
                await process.wait()
            self._log.exception("failed to start RTK subprocess")
            raise RTKProcessError(f"failed to start RTK subprocess: {exc}") from exc

    async def _stop_locked(self, *, reason: str) -> None:
        self._user_stop_requested = True
        self._desired_source = None
        process = self._process
        if process is None:
            self._clear_process_locked()
            return
        await self._terminate_process(process)
        self._clear_process_locked()

    async def _stop_lora_locked(self, *, reason: str) -> None:
        self._user_stop_requested = True
        session = self._lora_session
        if session is not None:
            session.reconnect_enabled = False
            session.stop_reason = reason
            session.lifecycle_state = LoRaLifecycleState.STOPPED_BY_USER
            session.transport_reason = reason
        self._desired_source = None
        process = self._process
        if process is not None:
            await self._terminate_process(process)
        self._clear_process_locked()
        # Keep session object for STOPPED_BY_USER status until a new start.

    async def _terminate_process(self, process: asyncio.subprocess.Process) -> None:
        pid = process.pid
        if process.returncode is None:
            process.terminate()
            try:
                await asyncio.wait_for(process.wait(), timeout=self._shutdown_grace_s)
            except asyncio.TimeoutError:
                self._log.warning("RTK subprocess pid=%s did not exit; killing", pid)
                process.kill()
                await process.wait()

    def _clear_process_locked(self) -> None:
        self._process = None
        self._mode = "idle"
        if self._status_file is not None:
            self._remove_status_file(self._status_file)
        self._status_file = None

    def _ensure_background_tasks_locked(self) -> None:
        if self._shutting_down:
            return
        if self._supervisor_task is None or self._supervisor_task.done():
            self._supervisor_task = asyncio.create_task(self._supervisor_loop(), name="rtk-supervisor")
        if self._lifecycle_task is None or self._lifecycle_task.done():
            self._lifecycle_task = asyncio.create_task(self._lifecycle_loop(), name="rtk-lifecycle")

    async def _cancel_background_tasks(self) -> None:
        tasks: list[asyncio.Task] = []
        current = asyncio.current_task()
        if self._supervisor_task is not None and not self._supervisor_task.done():
            if self._supervisor_task is not current:
                tasks.append(self._supervisor_task)
        if self._lifecycle_task is not None and not self._lifecycle_task.done():
            if self._lifecycle_task is not current:
                tasks.append(self._lifecycle_task)
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        self._supervisor_task = None
        self._lifecycle_task = None

    async def _supervisor_loop(self) -> None:
        try:
            await self._supervisor_loop_body()
        except asyncio.CancelledError:
            raise

    async def _supervisor_loop_body(self) -> None:
        while not self._shutting_down:
            await asyncio.sleep(0.5)
            should_restart = False
            async with self._lock:
                if self._desired_source != "lora" or self._lora_session is None:
                    continue
                session = self._lora_session
                if (
                    not session.reconnect_enabled
                    or self._user_stop_requested
                    or session.lifecycle_state == LoRaLifecycleState.FAILED
                ):
                    continue
                process = self._process
                if process is not None and process.returncode is None:
                    continue
                if process is not None and process.returncode is not None:
                    self._process = None
                    self._mode = "idle"
                if not self._can_restart_locked(session):
                    session.lifecycle_state = LoRaLifecycleState.FAILED
                    session.transport_reason = "restart_rate_exhausted"
                    session.last_error = (
                        f"exceeded {LORA_MAX_RESTARTS_PER_MIN} restarts per minute"
                    )
                    session.reconnect_enabled = False
                    continue
                session.restart_count += 1
                session.restart_timestamps.append(self._monotonic())
                session.lifecycle_state = LoRaLifecycleState.RECONNECTING
                session.transport_reason = "process_restart"
                should_restart = True

            if not should_restart:
                continue

            try:
                await asyncio.sleep(LORA_RECONNECT_INTERVAL_S)
            except asyncio.CancelledError:
                return
            async with self._lock:
                if (
                    self._desired_source != "lora"
                    or self._lora_session is None
                    or not self._lora_session.reconnect_enabled
                    or self._user_stop_requested
                ):
                    continue
                session = self._lora_session
                if session.lifecycle_state == LoRaLifecycleState.FAILED:
                    continue
                try:
                    await self._launch_lora_locked(session)
                except RTKProcessError as exc:
                    session.lifecycle_state = LoRaLifecycleState.FAILED
                    session.last_error = str(exc)
                    session.transport_reason = "startup_failure"

    async def _lifecycle_loop(self) -> None:
        try:
            while not self._shutting_down:
                await asyncio.sleep(0.5)
                async with self._lock:
                    self._refresh_lifecycle_locked()
        except asyncio.CancelledError:
            raise

    def _can_restart_locked(self, session: _LoraSession) -> bool:
        now = self._monotonic()
        while session.restart_timestamps and now - session.restart_timestamps[0] > 60.0:
            session.restart_timestamps.popleft()
        return len(session.restart_timestamps) < LORA_MAX_RESTARTS_PER_MIN

    def _refresh_lifecycle_locked(self) -> None:
        session = self._lora_session
        if session is None:
            return
        if session.lifecycle_state == LoRaLifecycleState.STOPPED_BY_USER:
            return
        if session.lifecycle_state == LoRaLifecycleState.FAILED:
            return
        if self._desired_source != "lora":
            return

        child = self._read_child_status()
        if child and not self._child_status_fresh(child, session):
            child = {}

        process_alive = self._process is not None and self._process.returncode is None
        child_state = str(child.get("lifecycle_state") or child.get("state") or "").lower()

        if child_state == "module_disconnected":
            session.lifecycle_state = LoRaLifecycleState.MODULE_DISCONNECTED
            session.transport_reason = "serial_module_disconnected"
            session.last_error = child.get("last_error")
            return

        if child_state in {"reconnecting", "error"} or (
            not process_alive and session.reconnect_enabled and not self._user_stop_requested
        ):
            session.lifecycle_state = LoRaLifecycleState.RECONNECTING
            session.transport_reason = child.get("transport_reason") or "reconnecting"
            session.last_error = child.get("last_error")
            return

        # Child serial_open_since_monotonic is authoritative after each connect/
        # reconnect. first_serial_open_monotonic is a manager-side fallback only
        # when the child timestamp is temporarily unavailable; stale child status
        # is rejected via session_id, process_id, and updated_at freshness checks.
        if child.get("serial_open") and session.first_serial_open_monotonic is None:
            open_since = child.get("serial_open_since_monotonic")
            session.first_serial_open_monotonic = (
                float(open_since) if isinstance(open_since, (int, float)) else self._monotonic()
            )

        age = self._stream_data_age(child, session)
        has_valid_frames = int(child.get("valid_frames", 0) or 0) > 0

        if age is None:
            if process_alive and child.get("serial_open"):
                session.lifecycle_state = LoRaLifecycleState.CONNECTED
            elif process_alive:
                session.lifecycle_state = LoRaLifecycleState.STARTING
            return

        if age <= LORA_NO_DATA_WARN_S:
            if has_valid_frames:
                session.lifecycle_state = LoRaLifecycleState.STREAMING_VALID_RTCM
            elif child.get("serial_open"):
                session.lifecycle_state = LoRaLifecycleState.CONNECTED
            else:
                session.lifecycle_state = LoRaLifecycleState.STARTING
            session.transport_reason = None
            session.stream_unhealthy_since = None
            session.no_data_since = None
            return

        if session.no_data_since is None:
            session.no_data_since = self._monotonic()
        session.lifecycle_state = LoRaLifecycleState.NO_DATA
        session.transport_reason = self._no_data_reason(child)
        if age >= LORA_NO_DATA_FAIL_S:
            if session.stream_unhealthy_since is None:
                session.stream_unhealthy_since = self._monotonic()
        else:
            session.stream_unhealthy_since = None

    def _stream_data_age(self, child: dict[str, Any], session: _LoraSession) -> float | None:
        """Age of the correction stream for no-data policy.

        Prefer last valid RTCM time when frames exist. Otherwise use child
        ``serial_open_since_monotonic`` (authoritative). The manager's
        ``first_serial_open_monotonic`` is only a fallback when child timestamps
        are temporarily missing; stale status files are dropped by
        :meth:`_child_status_fresh`.
        """
        now = self._monotonic()
        last_valid = child.get("last_valid_frame_time")
        if isinstance(last_valid, (int, float)):
            return max(0.0, now - float(last_valid))

        open_since = child.get("serial_open_since_monotonic")
        if isinstance(open_since, (int, float)) and child.get("serial_open"):
            return max(0.0, now - float(open_since))

        if session.first_serial_open_monotonic is not None and child.get("serial_open"):
            return max(0.0, now - session.first_serial_open_monotonic)

        return None

    @staticmethod
    def _no_data_reason(child: dict[str, Any]) -> str:
        if not child.get("serial_open", False):
            return "serial_disconnected"
        if child.get("valid_frames", 0) == 0 and child.get("invalid_frames", 0) > 0:
            return "invalid_stream_only"
        if child.get("valid_frames", 0) == 0 and child.get("bytes_received", 0) == 0:
            return "transmitter_silent"
        return "transmitter_silent"

    def _child_status_fresh(self, child: dict[str, Any], session: _LoraSession) -> bool:
        if child.get("session_id") != session.session_id:
            return False
        pid = child.get("process_id")
        if pid is not None and self._process is not None and pid != self._process.pid:
            return False
        updated = child.get("updated_at_monotonic")
        if isinstance(updated, (int, float)):
            return (self._monotonic() - float(updated)) <= 10.0
        updated_wall = child.get("updated_at")
        if isinstance(updated_wall, (int, float)):
            return (time.time() - float(updated_wall)) <= 10.0
        return True

    def _status_locked(self) -> RTKStatus:
        process = self._process
        process_alive = process is not None and process.returncode is None
        child = self._read_child_status() if process_alive else {}
        session = self._lora_session

        if session and child and not self._child_status_fresh(child, session):
            child = {}

        desired = self._desired_source
        active: RTKSource | None = None
        lifecycle = LoRaLifecycleState.IDLE.value
        if session is not None and session.lifecycle_state == LoRaLifecycleState.STOPPED_BY_USER:
            lifecycle = session.lifecycle_state.value
        elif desired == "lora" and session is not None:
            lifecycle = session.lifecycle_state.value
            if process_alive and session.lifecycle_state not in {
                LoRaLifecycleState.RECONNECTING,
                LoRaLifecycleState.FAILED,
                LoRaLifecycleState.STOPPED_BY_USER,
            }:
                active = "lora"
            elif process_alive and child.get("serial_open"):
                active = "lora"
        elif desired == "ntrip" and process_alive:
            active = "ntrip"
            lifecycle = str(child.get("state") or "running")

        if int(child.get("valid_frames", 0) or 0) > 0:
            last_valid_mono = child.get("last_valid_frame_time")
            last_valid_age = (
                max(0.0, self._monotonic() - float(last_valid_mono))
                if isinstance(last_valid_mono, (int, float))
                else None
            )
        elif session is not None:
            last_valid_age = self._stream_data_age(child, session)
        else:
            last_valid_age = None
        last_frame_wall = child.get("last_frame_time")
        last_frame_age_s = (
            max(0.0, time.time() - float(last_frame_wall))
            if isinstance(last_frame_wall, (int, float))
            else last_valid_age
        )

        valid_frames = int(child.get("valid_frames", child.get("frames", 0)) or 0)
        bytes_injected = int(child.get("bytes_injected", child.get("bytes", 0)) or 0)
        reconnecting = lifecycle in {
            LoRaLifecycleState.RECONNECTING.value,
            LoRaLifecycleState.MODULE_DISCONNECTED.value,
        }
        stream_healthy = None
        transport_reason = None
        if desired == "lora" and session is not None:
            transport_reason = session.transport_reason
            if lifecycle == LoRaLifecycleState.STREAMING_VALID_RTCM.value:
                stream_healthy = True
            elif lifecycle == LoRaLifecycleState.NO_DATA.value:
                stream_healthy = False if session.stream_unhealthy_since is not None else None
            elif lifecycle in {
                LoRaLifecycleState.FAILED.value,
                LoRaLifecycleState.MODULE_DISCONNECTED.value,
                LoRaLifecycleState.STOPPED_BY_USER.value,
            }:
                stream_healthy = False
            elif lifecycle in {LoRaLifecycleState.STARTING.value, LoRaLifecycleState.CONNECTED.value}:
                stream_healthy = None

        valid_rate = child.get("valid_frame_rate_hz")
        if valid_rate is not None:
            valid_rate = float(valid_rate)
        bytes_rate = child.get("bytes_per_sec")
        if bytes_rate is not None:
            bytes_rate = float(bytes_rate)

        nav = self._navigation_provider() if self._navigation_provider else {}
        gps_fix = nav.get("gps_fix")
        pose_age_ms = nav.get("local_pose_age_ms")
        rpp_state = nav.get("rpp_state")

        running = process_alive or (
            desired == "lora"
            and session is not None
            and session.lifecycle_state not in {
                LoRaLifecycleState.STOPPED_BY_USER,
                LoRaLifecycleState.IDLE,
            }
            and session.reconnect_enabled
        )
        healthy = bool(
            lifecycle == LoRaLifecycleState.STREAMING_VALID_RTCM.value
            or (desired == "ntrip" and process_alive and last_frame_age_s is not None and last_frame_age_s <= 10.0)
        )

        return RTKStatus(
            mode=self._mode if running else "idle",
            pid=process.pid if process_alive else None,
            running=running,
            healthy=healthy,
            source_state=str(child.get("state") or lifecycle.lower()),
            frames=valid_frames,
            bytes=bytes_injected,
            last_frame_age_s=last_frame_age_s,
            last_error=child.get("last_error") or (session.last_error if session else None),
            active_source=active,
            desired_source=desired,
            lifecycle_state=lifecycle,
            serial_port=session.serial_port if session else None,
            baudrate=session.baudrate if session else None,
            session_started_at=session.started_at if session else None,
            process_alive=process_alive,
            serial_open=bool(child.get("serial_open", False)),
            reconnecting=reconnecting,
            restart_count=session.restart_count if session else 0,
            valid_frames=valid_frames,
            invalid_frames=int(child.get("invalid_frames", 0) or 0),
            crc_error_count=int(child.get("crc_errors", 0) or 0),
            dropped_frames=int(child.get("dropped_frames", 0) or 0),
            bytes_received=int(child.get("bytes_received", 0) or 0),
            bytes_injected=bytes_injected,
            last_valid_rtcm_age_s=last_valid_age,
            valid_frame_rate_hz=valid_rate,
            bytes_per_sec=bytes_rate,
            injection_topic_ready=child.get("injection_topic_ready"),
            stream_healthy=stream_healthy,
            transport_reason=transport_reason,
            stop_reason=session.stop_reason if session else None,
            gps_fix_type=int(gps_fix) if gps_fix is not None else None,
            rtk_fixed=gps_fix == 6 if gps_fix is not None else None,
            rtk_float=gps_fix == 5 if gps_fix is not None else None,
            pose_age_s=(float(pose_age_ms) / 1000.0) if pose_age_ms is not None else None,
            rpp_rtk_wait=rpp_state == RPP_RTK_WAIT if rpp_state is not None else None,
        )

    @staticmethod
    def _iso_utc_now() -> str:
        return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

    @staticmethod
    def _redact_args(args: list[str]) -> list[str]:
        redacted = list(args)
        for i, arg in enumerate(redacted[:-1]):
            if arg in {"--pass", "--password"}:
                redacted[i + 1] = "***"
        return redacted

    @staticmethod
    def _new_status_file(mode: RTKMode) -> Path:
        name = f"px4_dxp_rtk_{mode}_{uuid.uuid4().hex}.json"
        return Path(tempfile.gettempdir()) / name

    @staticmethod
    def _remove_status_file(path: Path) -> None:
        try:
            path.unlink(missing_ok=True)
        except Exception:
            logging.getLogger("server.rtk_manager").warning(
                "failed to remove RTK status file %s", path, exc_info=True
            )

    def _read_child_status(self) -> dict[str, Any]:
        path = self._status_file
        if path is None:
            return {}
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return {}
        except json.JSONDecodeError:
            return {}
        except Exception:
            self._log.warning("failed to read RTK status file %s", path, exc_info=True)
            return {}