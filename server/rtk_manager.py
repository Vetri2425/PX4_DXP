"""Async subprocess orchestration for RTK correction streams."""

from __future__ import annotations

import asyncio
import json
import logging
import os
import signal
import sys
import tempfile
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Literal

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from rtk_transport import redact_rtk_secrets

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
    NTRIP_AUTH_EXIT_CODE,
    NTRIP_CONNECT_TIMEOUT_S,
    NTRIP_MAX_RESTARTS_PER_MIN,
    NTRIP_NO_RTCM_RECONNECT_S,
    NTRIP_NO_RTCM_WARN_S,
    NTRIP_PUBLISH_ERROR_UNHEALTHY_THRESHOLD,
    NTRIP_RECONNECT_INITIAL_S,
    NTRIP_RECONNECT_JITTER_FRAC,
    NTRIP_RECONNECT_MAX_S,
    NTRIP_RECV_TIMEOUT_S,
    NTRIP_RESTART_COOLDOWN_S,
    NTRIP_SUPERVISOR_RESTART_DELAY_S,
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


class NtripLifecycleState(str, Enum):
    IDLE = "IDLE"
    STARTING = "STARTING"
    CONNECTING = "CONNECTING"
    CONNECTED = "CONNECTED"
    STREAMING_VALID_RTCM = "STREAMING_VALID_RTCM"
    RECONNECTING = "RECONNECTING"
    NO_VALID_RTCM = "NO_VALID_RTCM"
    AUTH_FAILED = "AUTH_FAILED"
    DNS_FAILED = "DNS_FAILED"
    CASTER_UNREACHABLE = "CASTER_UNREACHABLE"
    PROCESS_CRASHED = "PROCESS_CRASHED"
    FAILED = "FAILED"
    STOPPED_BY_USER = "STOPPED_BY_USER"


_NTRIP_CHILD_STATE_MAP = {
    "starting": NtripLifecycleState.STARTING,
    "connecting": NtripLifecycleState.CONNECTING,
    "connected": NtripLifecycleState.CONNECTED,
    "streaming_valid_rtcm": NtripLifecycleState.STREAMING_VALID_RTCM,
    "streaming": NtripLifecycleState.STREAMING_VALID_RTCM,
    "reconnecting": NtripLifecycleState.RECONNECTING,
    "no_valid_rtcm": NtripLifecycleState.NO_VALID_RTCM,
    "auth_failed": NtripLifecycleState.AUTH_FAILED,
    "dns_failed": NtripLifecycleState.DNS_FAILED,
    "caster_unreachable": NtripLifecycleState.CASTER_UNREACHABLE,
    "mount_not_found": NtripLifecycleState.CASTER_UNREACHABLE,
    "protocol_error": NtripLifecycleState.RECONNECTING,
    "stopping": NtripLifecycleState.STOPPED_BY_USER,
}


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
    user_requested: bool | None = None
    host: str | None = None
    port: int | None = None
    mountpoint: str | None = None
    username: str | None = None
    connected: bool | None = None
    last_exit_code: int | None = None
    last_process_error: str | None = None
    publish_error_count: int = 0
    injection_healthy: bool | None = None
    valid_rtcm_bytes: int = 0
    frames_published: int = 0
    invalid_scan_events: int = 0
    dropped_complete_frames: int = 0


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


@dataclass
class _NtripSession:
    session_id: str
    host: str
    port: int
    mountpoint: str
    username: str
    password: str
    started_at: str
    user_requested: bool = True
    reconnect_enabled: bool = True
    restart_count: int = 0
    stop_reason: str | None = None
    lifecycle_state: NtripLifecycleState = NtripLifecycleState.STARTING
    transport_reason: str | None = None
    last_error: str | None = None
    last_exit_code: int | None = None
    last_process_error: str | None = None
    restart_timestamps: deque[float] = field(default_factory=deque)
    restart_cooldown_until: float | None = None


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
        self._ntrip_session: _NtripSession | None = None
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
        host = host.strip()
        mountpoint = mountpoint.strip()
        user = user.strip()
        if not host or not mountpoint or not user:
            raise RTKValidationError("host, mountpoint, and user are required")
        async with self._lock:
            if self._desired_source == "lora":
                raise RTKConflictError("LoRa RTK session is active or desired; stop it first")
            if self._desired_source == "ntrip" and self._ntrip_session is not None:
                session = self._ntrip_session
                if (
                    session.host == host
                    and session.port == port
                    and session.mountpoint == mountpoint
                    and session.username == user
                    and session.lifecycle_state != NtripLifecycleState.STOPPED_BY_USER
                    and session.lifecycle_state != NtripLifecycleState.AUTH_FAILED
                ):
                    return self._status_locked()
            if self._desired_source == "lora":
                await self._stop_lora_locked(reason="switching to NTRIP")
            elif self._process is not None:
                await self._stop_ntrip_locked(reason="starting ntrip")

            self._user_stop_requested = False
            session = _NtripSession(
                session_id=uuid.uuid4().hex,
                host=host,
                port=port,
                mountpoint=mountpoint,
                username=user,
                password=password,
                started_at=self._iso_utc_now(),
                lifecycle_state=NtripLifecycleState.STARTING,
                user_requested=True,
            )
            self._ntrip_session = session
            self._lora_session = None
            self._desired_source = "ntrip"
        await self._cancel_background_tasks()
        async with self._lock:
            await self._launch_ntrip_locked(session, password=password)
            self._ensure_background_tasks_locked()
            return self._status_locked()

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
                await self._stop_ntrip_locked(reason="switching to LoRa")
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
            elif self._desired_source == "ntrip":
                await self._stop_ntrip_locked(reason="user_stop")
            else:
                await self._stop_locked(reason="user_stop")
        await self._cancel_background_tasks()
        async with self._lock:
            return self._status_locked()

    async def status(self) -> RTKStatus:
        async with self._lock:
            self._refresh_lifecycle_locked()
            if self._process is not None and self._process.returncode is not None:
                rc = self._process.returncode
                if (
                    self._desired_source == "lora"
                    and not self._user_stop_requested
                ):
                    self._process = None
                elif (
                    self._desired_source == "ntrip"
                    and self._ntrip_session is not None
                    and not self._user_stop_requested
                ):
                    session = self._ntrip_session
                    if not self._handle_ntrip_exit_locked(session, rc):
                        session.lifecycle_state = NtripLifecycleState.PROCESS_CRASHED
                        session.transport_reason = "process_crashed"
                    self._process = None
                    self._mode = "idle"
                else:
                    self._clear_process_locked()
            return self._status_locked()

    async def shutdown(self) -> None:
        self._shutting_down = True
        async with self._lock:
            self._user_stop_requested = True
            if self._lora_session is not None:
                self._lora_session.reconnect_enabled = False
            if self._ntrip_session is not None:
                self._ntrip_session.reconnect_enabled = False
            if self._desired_source == "lora":
                await self._stop_lora_locked(reason="shutdown")
            elif self._desired_source == "ntrip":
                await self._stop_ntrip_locked(reason="shutdown")
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

    async def _launch_ntrip_locked(self, session: _NtripSession, *, password: str) -> None:
        if not self._ntrip_script.exists():
            session.lifecycle_state = NtripLifecycleState.FAILED
            session.last_error = f"ntrip script not found: {self._ntrip_script}"
            raise RTKProcessError(session.last_error)

        status_file = self._new_status_file("ntrip")
        args = [
            "--host",
            session.host,
            "--port",
            str(session.port),
            "--mountpoint",
            session.mountpoint,
            "--user",
            session.username,
            "--pass-stdin",
            "--status-file",
            str(status_file),
            "--session-id",
            session.session_id,
            "--connect-timeout-s",
            str(NTRIP_CONNECT_TIMEOUT_S),
            "--recv-timeout-s",
            str(NTRIP_RECV_TIMEOUT_S),
            "--no-rtcm-warn-s",
            str(NTRIP_NO_RTCM_WARN_S),
            "--no-rtcm-reconnect-s",
            str(NTRIP_NO_RTCM_RECONNECT_S),
            "--reconnect-initial-s",
            str(NTRIP_RECONNECT_INITIAL_S),
            "--reconnect-max-s",
            str(NTRIP_RECONNECT_MAX_S),
            "--reconnect-jitter-frac",
            str(NTRIP_RECONNECT_JITTER_FRAC),
            "--publish-error-unhealthy-threshold",
            str(NTRIP_PUBLISH_ERROR_UNHEALTHY_THRESHOLD),
        ]
        process = await self._spawn_process(
            self._ntrip_script,
            args,
            stdin_payload=f"{password}\n",
        )
        self._process = process
        self._mode = "ntrip"
        self._status_file = status_file
        session.lifecycle_state = NtripLifecycleState.STARTING
        session.transport_reason = None
        session.last_error = None
        session.last_exit_code = None
        session.last_process_error = None

        try:
            await asyncio.wait_for(process.wait(), timeout=self._startup_grace_s)
        except asyncio.TimeoutError:
            return

        rc = process.returncode
        self._process = None
        self._mode = "idle"
        if self._user_stop_requested:
            return
        session.last_exit_code = rc
        if rc == NTRIP_AUTH_EXIT_CODE:
            session.lifecycle_state = NtripLifecycleState.AUTH_FAILED
            session.reconnect_enabled = False
            session.transport_reason = "auth_failed"
            session.last_process_error = "NTRIP authentication rejected"
            raise RTKProcessError(session.last_process_error)
        session.lifecycle_state = NtripLifecycleState.FAILED
        session.last_error = f"ntrip subprocess exited immediately with code {rc}"
        session.last_process_error = session.last_error
        raise RTKProcessError(session.last_error)

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
            spawn_kwargs: dict[str, Any] = {
                "cwd": str(script.parent),
                "stdin": (
                    asyncio.subprocess.PIPE
                    if stdin_payload is not None
                    else asyncio.subprocess.DEVNULL
                ),
            }
            if sys.platform != "win32":
                spawn_kwargs["start_new_session"] = True
            process = await asyncio.create_subprocess_exec(*cmd, **spawn_kwargs)
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

    async def _stop_ntrip_locked(self, *, reason: str) -> None:
        self._user_stop_requested = True
        session = self._ntrip_session
        if session is not None:
            session.user_requested = False
            session.reconnect_enabled = False
            session.password = ""
            session.stop_reason = reason
            session.lifecycle_state = NtripLifecycleState.STOPPED_BY_USER
            session.transport_reason = reason
        self._desired_source = None
        process = self._process
        if process is not None:
            await self._terminate_process(process)
        self._clear_process_locked()

    async def _terminate_process(self, process: asyncio.subprocess.Process) -> None:
        pid = process.pid
        if process.returncode is None:
            if sys.platform != "win32" and pid is not None:
                try:
                    os.killpg(os.getpgid(pid), signal.SIGTERM)
                except ProcessLookupError:
                    process.terminate()
            else:
                process.terminate()
            try:
                await asyncio.wait_for(process.wait(), timeout=self._shutdown_grace_s)
            except asyncio.TimeoutError:
                self._log.warning("RTK subprocess pid=%s did not exit; killing", pid)
                if sys.platform != "win32" and pid is not None:
                    try:
                        os.killpg(os.getpgid(pid), signal.SIGKILL)
                    except ProcessLookupError:
                        process.kill()
                else:
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
            restart_source: RTKSource | None = None
            async with self._lock:
                if self._desired_source == "lora" and self._lora_session is not None:
                    session = self._lora_session
                    if (
                        not session.reconnect_enabled
                        or self._user_stop_requested
                        or session.lifecycle_state == LoRaLifecycleState.FAILED
                    ):
                        pass
                    else:
                        process = self._process
                        if process is not None and process.returncode is None:
                            pass
                        else:
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
                            else:
                                session.restart_count += 1
                                session.restart_timestamps.append(self._monotonic())
                                session.lifecycle_state = LoRaLifecycleState.RECONNECTING
                                session.transport_reason = "process_restart"
                                should_restart = True
                                restart_source = "lora"
                elif self._desired_source == "ntrip" and self._ntrip_session is not None:
                    session = self._ntrip_session
                    if (
                        not session.reconnect_enabled
                        or not session.user_requested
                        or self._user_stop_requested
                        or session.lifecycle_state == NtripLifecycleState.AUTH_FAILED
                    ):
                        pass
                    else:
                        process = self._process
                        if (
                            session.lifecycle_state == NtripLifecycleState.FAILED
                            and session.transport_reason == "restart_throttled"
                            and session.restart_cooldown_until is not None
                            and self._monotonic() >= session.restart_cooldown_until
                        ):
                            session.restart_cooldown_until = None
                            session.lifecycle_state = NtripLifecycleState.RECONNECTING
                            session.transport_reason = "restart_cooldown_complete"
                            should_restart = True
                            restart_source = "ntrip"
                        elif process is not None and process.returncode is None:
                            pass
                        elif session.lifecycle_state == NtripLifecycleState.FAILED:
                            pass
                        elif process is None or process.returncode is not None:
                            # Reap an unexpected child exit here so a confirmed
                            # auth rejection becomes terminal AUTH_FAILED without
                            # depending on /api/rtk/status being polled.
                            if process is not None and process.returncode is not None:
                                if self._handle_ntrip_exit_locked(
                                    session, process.returncode
                                ):
                                    self._process = None
                                    self._mode = "idle"
                                    continue  # auth rejection: never restart
                            if self._can_restart_ntrip_locked(session):
                                session.restart_count += 1
                                session.restart_timestamps.append(self._monotonic())
                                session.lifecycle_state = NtripLifecycleState.RECONNECTING
                                session.transport_reason = "process_restart"
                                should_restart = True
                                restart_source = "ntrip"

            if not should_restart or restart_source is None:
                continue

            delay = (
                LORA_RECONNECT_INTERVAL_S
                if restart_source == "lora"
                else NTRIP_SUPERVISOR_RESTART_DELAY_S
            )
            try:
                await asyncio.sleep(delay)
            except asyncio.CancelledError:
                return
            async with self._lock:
                if self._user_stop_requested or self._shutting_down:
                    continue
                if restart_source == "lora":
                    if (
                        self._desired_source != "lora"
                        or self._lora_session is None
                        or not self._lora_session.reconnect_enabled
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
                elif restart_source == "ntrip":
                    if (
                        self._desired_source != "ntrip"
                        or self._ntrip_session is None
                        or not self._ntrip_session.reconnect_enabled
                        or not self._ntrip_session.user_requested
                    ):
                        continue
                    session = self._ntrip_session
                    if session.lifecycle_state == NtripLifecycleState.AUTH_FAILED:
                        continue
                    if session.lifecycle_state == NtripLifecycleState.FAILED:
                        if session.transport_reason != "restart_throttled":
                            continue
                    if not session.password:
                        session.lifecycle_state = NtripLifecycleState.FAILED
                        session.last_error = "NTRIP restart unavailable after stop"
                        session.transport_reason = "restart_requires_operator"
                        session.reconnect_enabled = False
                        continue
                    try:
                        await self._launch_ntrip_locked(session, password=session.password)
                    except RTKProcessError as exc:
                        if session.lifecycle_state == NtripLifecycleState.AUTH_FAILED:
                            session.reconnect_enabled = False
                        else:
                            session.lifecycle_state = NtripLifecycleState.FAILED
                        session.last_error = redact_rtk_secrets(str(exc))
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

    def _handle_ntrip_exit_locked(self, session: _NtripSession, rc: int | None) -> bool:
        """Record an unexpected NTRIP child exit; detect terminal auth rejection.

        Shared by :meth:`status` and the supervisor loop so AUTH_FAILED is
        reached without depending on ``/api/rtk/status`` polling. Returns True
        only for a confirmed authentication rejection (``NTRIP_AUTH_EXIT_CODE``):
        the session moves to AUTH_FAILED, reconnect is disabled, and the caller
        must NOT restart. Returns False for any other exit (caller may restart).
        Does not touch restart counters — terminal auth never counts as a
        restart attempt.
        """
        session.last_exit_code = rc
        session.last_process_error = redact_rtk_secrets(
            session.last_error or f"process exited with code {rc}"
        )
        if rc == NTRIP_AUTH_EXIT_CODE:
            session.lifecycle_state = NtripLifecycleState.AUTH_FAILED
            session.reconnect_enabled = False
            session.transport_reason = "auth_failed"
            return True
        return False

    def _can_restart_ntrip_locked(self, session: _NtripSession) -> bool:
        now = self._monotonic()
        if session.restart_cooldown_until is not None and now < session.restart_cooldown_until:
            return False
        if session.restart_cooldown_until is not None and now >= session.restart_cooldown_until:
            session.restart_cooldown_until = None
        while session.restart_timestamps and now - session.restart_timestamps[0] > 60.0:
            session.restart_timestamps.popleft()
        if len(session.restart_timestamps) >= NTRIP_MAX_RESTARTS_PER_MIN:
            session.restart_cooldown_until = now + NTRIP_RESTART_COOLDOWN_S
            session.lifecycle_state = NtripLifecycleState.FAILED
            session.transport_reason = "restart_throttled"
            session.last_error = (
                f"exceeded {NTRIP_MAX_RESTARTS_PER_MIN} restarts per minute; "
                f"cooling down for {NTRIP_RESTART_COOLDOWN_S:.0f}s"
            )
            return False
        return True

    def _refresh_lifecycle_locked(self) -> None:
        if self._desired_source == "lora":
            self._refresh_lora_lifecycle_locked()
        elif self._desired_source == "ntrip":
            self._refresh_ntrip_lifecycle_locked()

    def _refresh_lora_lifecycle_locked(self) -> None:
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

    def _child_status_fresh_ntrip(self, child: dict[str, Any], session: _NtripSession) -> bool:
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

    def _refresh_ntrip_lifecycle_locked(self) -> None:
        session = self._ntrip_session
        if session is None:
            return
        if session.lifecycle_state in {
            NtripLifecycleState.STOPPED_BY_USER,
            NtripLifecycleState.AUTH_FAILED,
        }:
            return
        if session.lifecycle_state == NtripLifecycleState.FAILED:
            return
        if self._desired_source != "ntrip":
            return

        child = self._read_child_status()
        if child and not self._child_status_fresh_ntrip(child, session):
            child = {}

        process_alive = self._process is not None and self._process.returncode is None
        if not process_alive and session.lifecycle_state not in {
            NtripLifecycleState.PROCESS_CRASHED,
            NtripLifecycleState.RECONNECTING,
        }:
            return
        if not child:
            if process_alive:
                session.lifecycle_state = NtripLifecycleState.STARTING
            return

        child_state = str(child.get("lifecycle_state") or child.get("state") or "").lower()
        mapped = _NTRIP_CHILD_STATE_MAP.get(child_state)
        if mapped is not None and session.lifecycle_state not in {
            NtripLifecycleState.PROCESS_CRASHED,
            NtripLifecycleState.FAILED,
        }:
            session.lifecycle_state = mapped
        err = child.get("last_error")
        if err:
            session.last_error = redact_rtk_secrets(str(err))
        if child.get("transport_reason"):
            session.transport_reason = str(child.get("transport_reason"))

    def _status_locked(self) -> RTKStatus:
        process = self._process
        process_alive = process is not None and process.returncode is None
        child = self._read_child_status() if (process_alive or self._status_file) else {}
        lora_session = self._lora_session
        ntrip_session = self._ntrip_session

        if lora_session and child and not self._child_status_fresh(child, lora_session):
            child = {}
        if ntrip_session and child and not self._child_status_fresh_ntrip(child, ntrip_session):
            child = {}

        desired = self._desired_source
        active: RTKSource | None = None
        lifecycle = LoRaLifecycleState.IDLE.value
        reconnecting = False
        stream_healthy: bool | None = None
        transport_reason: str | None = None
        stop_reason: str | None = None
        restart_count = 0
        user_requested: bool | None = None
        host = port = mountpoint = username = None
        connected: bool | None = None
        last_exit_code: int | None = None
        last_process_error: str | None = None
        session_started_at: str | None = None
        serial_port: str | None = None
        baudrate: int | None = None

        if lora_session is not None and lora_session.lifecycle_state == LoRaLifecycleState.STOPPED_BY_USER:
            lifecycle = lora_session.lifecycle_state.value
            stop_reason = lora_session.stop_reason
            restart_count = lora_session.restart_count
        elif desired == "lora" and lora_session is not None:
            lifecycle = lora_session.lifecycle_state.value
            transport_reason = lora_session.transport_reason
            stop_reason = lora_session.stop_reason
            restart_count = lora_session.restart_count
            session_started_at = lora_session.started_at
            serial_port = lora_session.serial_port
            baudrate = lora_session.baudrate
            if process_alive and lora_session.lifecycle_state not in {
                LoRaLifecycleState.RECONNECTING,
                LoRaLifecycleState.FAILED,
                LoRaLifecycleState.STOPPED_BY_USER,
            }:
                active = "lora"
            elif process_alive and child.get("serial_open"):
                active = "lora"
            reconnecting = lifecycle in {
                LoRaLifecycleState.RECONNECTING.value,
                LoRaLifecycleState.MODULE_DISCONNECTED.value,
            }
            if lifecycle == LoRaLifecycleState.STREAMING_VALID_RTCM.value:
                stream_healthy = True
            elif lifecycle == LoRaLifecycleState.NO_DATA.value:
                stream_healthy = (
                    False if lora_session.stream_unhealthy_since is not None else None
                )
            elif lifecycle in {
                LoRaLifecycleState.FAILED.value,
                LoRaLifecycleState.MODULE_DISCONNECTED.value,
                LoRaLifecycleState.STOPPED_BY_USER.value,
            }:
                stream_healthy = False
        elif ntrip_session is not None and ntrip_session.lifecycle_state in {
            NtripLifecycleState.STOPPED_BY_USER,
            NtripLifecycleState.AUTH_FAILED,
        }:
            lifecycle = ntrip_session.lifecycle_state.value
            stop_reason = ntrip_session.stop_reason
            restart_count = ntrip_session.restart_count
            user_requested = ntrip_session.user_requested
            host = ntrip_session.host
            port = ntrip_session.port
            mountpoint = ntrip_session.mountpoint
            username = ntrip_session.username
            last_exit_code = ntrip_session.last_exit_code
            last_process_error = redact_rtk_secrets(ntrip_session.last_process_error)
            session_started_at = ntrip_session.started_at
            transport_reason = ntrip_session.transport_reason
            stream_healthy = False if lifecycle == NtripLifecycleState.AUTH_FAILED.value else None
        elif desired == "ntrip" and ntrip_session is not None:
            lifecycle = ntrip_session.lifecycle_state.value
            transport_reason = ntrip_session.transport_reason
            stop_reason = ntrip_session.stop_reason
            restart_count = ntrip_session.restart_count
            user_requested = ntrip_session.user_requested
            host = ntrip_session.host
            port = ntrip_session.port
            mountpoint = ntrip_session.mountpoint
            username = ntrip_session.username
            last_exit_code = ntrip_session.last_exit_code
            last_process_error = redact_rtk_secrets(ntrip_session.last_process_error)
            session_started_at = ntrip_session.started_at
            connected = bool(child.get("connected", False)) if child else False
            if process_alive and ntrip_session.lifecycle_state not in {
                NtripLifecycleState.FAILED,
                NtripLifecycleState.STOPPED_BY_USER,
                NtripLifecycleState.AUTH_FAILED,
                NtripLifecycleState.PROCESS_CRASHED,
            }:
                active = "ntrip"
            reconnecting = lifecycle in {
                NtripLifecycleState.RECONNECTING.value,
                NtripLifecycleState.CONNECTING.value,
                NtripLifecycleState.NO_VALID_RTCM.value,
                NtripLifecycleState.DNS_FAILED.value,
                NtripLifecycleState.CASTER_UNREACHABLE.value,
                NtripLifecycleState.PROCESS_CRASHED.value,
            } or bool(child.get("reconnecting", False))
            if child.get("stream_healthy") is not None:
                stream_healthy = bool(child.get("stream_healthy"))
            elif lifecycle == NtripLifecycleState.STREAMING_VALID_RTCM.value:
                stream_healthy = True
            elif lifecycle in {
                NtripLifecycleState.AUTH_FAILED.value,
                NtripLifecycleState.FAILED.value,
            }:
                stream_healthy = False

        last_valid_age = child.get("last_valid_rtcm_age_s")
        if isinstance(last_valid_age, (int, float)):
            last_valid_age = max(0.0, float(last_valid_age))
        elif desired == "lora" and lora_session is not None:
            last_valid_age = self._stream_data_age(child, lora_session)
        else:
            last_valid_age = None

        last_frame_age_s = last_valid_age
        valid_frames = int(child.get("valid_frames", child.get("frames", 0)) or 0)
        bytes_injected = int(child.get("bytes_injected", child.get("bytes", 0)) or 0)
        invalid_scan = int(
            child.get("invalid_scan_events", child.get("invalid_frames", 0)) or 0
        )
        dropped_complete = int(
            child.get("dropped_complete_frames", child.get("dropped_frames", 0)) or 0
        )
        publish_error_count = int(child.get("publish_error_count", 0) or 0)
        injection_healthy = child.get("injection_healthy")
        if injection_healthy is not None:
            injection_healthy = bool(injection_healthy)

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

        session_error = None
        if lora_session is not None:
            session_error = lora_session.last_error
        elif ntrip_session is not None:
            session_error = ntrip_session.last_error

        running = process_alive or (
            desired == "lora"
            and lora_session is not None
            and lora_session.lifecycle_state not in {
                LoRaLifecycleState.STOPPED_BY_USER,
                LoRaLifecycleState.IDLE,
            }
            and lora_session.reconnect_enabled
        ) or (
            desired == "ntrip"
            and ntrip_session is not None
            and ntrip_session.user_requested
            and ntrip_session.lifecycle_state not in {
                NtripLifecycleState.STOPPED_BY_USER,
                NtripLifecycleState.AUTH_FAILED,
            }
            and (
                process_alive
                or ntrip_session.reconnect_enabled
                or ntrip_session.lifecycle_state
                in {
                    NtripLifecycleState.PROCESS_CRASHED,
                    NtripLifecycleState.RECONNECTING,
                    NtripLifecycleState.FAILED,
                }
            )
        )

        healthy = bool(
            lifecycle == LoRaLifecycleState.STREAMING_VALID_RTCM.value
            or lifecycle == NtripLifecycleState.STREAMING_VALID_RTCM.value
            or (
                desired == "ntrip"
                and stream_healthy is True
                and last_valid_age is not None
                and last_valid_age <= NTRIP_NO_RTCM_WARN_S
            )
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
            last_error=redact_rtk_secrets(child.get("last_error") or session_error),
            active_source=active,
            desired_source=desired,
            lifecycle_state=lifecycle,
            serial_port=serial_port,
            baudrate=baudrate,
            session_started_at=session_started_at,
            process_alive=process_alive,
            serial_open=bool(child.get("serial_open", False)),
            reconnecting=reconnecting,
            restart_count=restart_count,
            valid_frames=valid_frames,
            invalid_frames=invalid_scan,
            crc_error_count=int(child.get("crc_errors", 0) or 0),
            dropped_frames=dropped_complete,
            bytes_received=int(child.get("bytes_received", 0) or 0),
            bytes_injected=bytes_injected,
            last_valid_rtcm_age_s=last_valid_age,
            valid_frame_rate_hz=valid_rate,
            bytes_per_sec=bytes_rate,
            injection_topic_ready=child.get("injection_topic_ready"),
            stream_healthy=stream_healthy,
            transport_reason=transport_reason,
            stop_reason=stop_reason,
            gps_fix_type=int(gps_fix) if gps_fix is not None else None,
            rtk_fixed=gps_fix == 6 if gps_fix is not None else None,
            rtk_float=gps_fix == 5 if gps_fix is not None else None,
            pose_age_s=(float(pose_age_ms) / 1000.0) if pose_age_ms is not None else None,
            rpp_rtk_wait=rpp_state == RPP_RTK_WAIT if rpp_state is not None else None,
            user_requested=user_requested,
            host=host,
            port=port,
            mountpoint=mountpoint,
            username=username,
            connected=connected,
            last_exit_code=last_exit_code,
            last_process_error=last_process_error,
            publish_error_count=publish_error_count,
            injection_healthy=injection_healthy,
            valid_rtcm_bytes=int(child.get("valid_rtcm_bytes", 0) or 0),
            frames_published=int(child.get("frames_published", 0) or 0),
            invalid_scan_events=invalid_scan,
            dropped_complete_frames=dropped_complete,
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