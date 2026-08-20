"""Async subprocess orchestration for RTK correction streams."""

from __future__ import annotations

import asyncio
import json
import logging
import os
import tempfile
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

RTKMode = Literal["ntrip", "lora", "idle"]


class RTKProcessError(RuntimeError):
    """Raised when RTK configuration or a child process is invalid."""


@dataclass(frozen=True)
class NtripConfig:
    host: str
    port: int
    mountpoint: str
    user: str
    password: str


def load_ntrip_config(path: str | Path) -> NtripConfig:
    """Load the gitignored NTRIP env file without executing shell code.

    Accepted keys match ``tools/debug_ntrip_caster.py``. Values may be bare or
    wrapped in matching single/double quotes; ``export KEY=...`` is accepted.
    Passwords are never included in validation errors or logs.
    """
    config_path = Path(path)
    try:
        text = config_path.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise RTKProcessError(f"NTRIP config not found: {config_path}") from exc
    except OSError as exc:
        raise RTKProcessError(f"cannot read NTRIP config {config_path}: {exc}") from exc

    values: dict[str, str] = {}
    accepted = {
        "NTRIP_HOST",
        "NTRIP_PORT",
        "NTRIP_MOUNTPT",
        "NTRIP_USER",
        "NTRIP_PASS",
    }
    for line_no, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].lstrip()
        if "=" not in line:
            raise RTKProcessError(
                f"invalid NTRIP config line {line_no}: expected KEY=value"
            )
        key, value = line.split("=", 1)
        key = key.strip()
        if key not in accepted:
            continue
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
            value = value[1:-1]
        if "\x00" in value or "\n" in value or "\r" in value:
            raise RTKProcessError(f"invalid value for {key} in NTRIP config")
        values[key] = value

    missing = [key for key in accepted if not values.get(key)]
    if missing:
        raise RTKProcessError(
            "NTRIP config missing required keys: " + ", ".join(sorted(missing))
        )
    try:
        port = int(values["NTRIP_PORT"])
    except ValueError as exc:
        raise RTKProcessError("NTRIP_PORT must be an integer") from exc
    if not 1 <= port <= 65535:
        raise RTKProcessError("NTRIP_PORT must be in 1..65535")
    host = values["NTRIP_HOST"]
    if any(char.isspace() for char in host) or "://" in host:
        raise RTKProcessError("NTRIP_HOST must be a hostname or IP without a URL scheme")
    mountpoint = values["NTRIP_MOUNTPT"].lstrip("/")
    if not mountpoint or any(char.isspace() for char in mountpoint):
        raise RTKProcessError("NTRIP_MOUNTPT must be a non-empty path component")

    # Do not reject an existing field deployment solely on permissions, but
    # surface the secret exposure loudly. deploy/preflight can enforce 0600.
    try:
        if os.stat(config_path).st_mode & 0o077:
            logging.getLogger("server.rtk_manager").warning(
                "NTRIP config %s is group/world accessible; set mode 0600",
                config_path,
            )
    except OSError:
        pass

    return NtripConfig(
        host=host,
        port=port,
        mountpoint=mountpoint,
        user=values["NTRIP_USER"],
        password=values["NTRIP_PASS"],
    )


@dataclass(frozen=True)
class RTKStatus:
    mode: RTKMode
    pid: int | None
    running: bool
    healthy: bool
    source_state: str
    frames: int
    bytes: int
    last_frame_age_s: float | None
    last_error: str | None
    desired_mode: RTKMode
    supervisor_restarts: int
    active_profile_id: str | None
    active_profile_revision: int | None


class AsyncRTKManager:
    """Owns the active RTK injection subprocess.

    Only one RTK source may publish RTCM frames into MAVROS at a time. Starting
    a new source first stops the current child process under the same lock.
    """

    def __init__(
        self,
        *,
        ntrip_script: Path | None = None,
        lora_script: Path | None = None,
        python_executable: str | None = None,
        startup_grace_s: float = 0.35,
        shutdown_grace_s: float = 10.0,
    ) -> None:
        repo_root = Path(__file__).resolve().parents[1]
        self._ntrip_script = ntrip_script or (repo_root / "ntrip_rtcm_node.py")
        self._lora_script = lora_script or (repo_root / "lora_rtcm_node.py")
        self._python = python_executable or sys.executable or "python3"
        self._startup_grace_s = startup_grace_s
        self._shutdown_grace_s = shutdown_grace_s

        self._lock = asyncio.Lock()
        self._process: asyncio.subprocess.Process | None = None
        self._mode: RTKMode = "idle"
        self._watch_task: asyncio.Task | None = None
        self._status_file: Path | None = None
        self._log = logging.getLogger("server.rtk_manager")
        self._desired_mode: RTKMode = "idle"
        self._desired_ntrip: NtripConfig | None = None
        self._restart_task: asyncio.Task | None = None
        self._restart_attempt = 0
        self._supervisor_restarts = 0
        self._last_supervisor_error: str | None = None
        self._active_profile_id: str | None = None
        self._active_profile_revision: int | None = None

    async def start_ntrip(
        self,
        *,
        host: str,
        port: int,
        mountpoint: str,
        user: str,
        password: str,
    ) -> RTKStatus:
        return await self.start_ntrip_config(
            NtripConfig(
                host=host,
                port=port,
                mountpoint=mountpoint.lstrip("/"),
                user=user,
                password=password,
            )
        )

    async def start_ntrip_config(self, config: NtripConfig) -> RTKStatus:
        """Start NTRIP and retain its config for supervised child restarts."""
        return await self.start_ntrip_profile(config)

    async def start_ntrip_profile(
        self,
        config: NtripConfig,
        *,
        profile_id: str | None = None,
        profile_revision: int | None = None,
    ) -> RTKStatus:
        """Start NTRIP and retain both config and applied profile identity."""
        async with self._lock:
            self._cancel_restart_locked()
            self._desired_mode = "ntrip"
            self._desired_ntrip = config
            self._active_profile_id = profile_id
            self._active_profile_revision = profile_revision
            self._restart_attempt = 0
            self._last_supervisor_error = None
            try:
                return await self._start_ntrip_locked(config)
            except Exception:
                self._desired_mode = "idle"
                self._desired_ntrip = None
                self._active_profile_id = None
                self._active_profile_revision = None
                raise

    async def start_lora(self, *, baudrate: int, serial_port: str) -> RTKStatus:
        args = [
            "--baudrate",
            str(baudrate),
            "--serial-port",
            serial_port,
        ]
        async with self._lock:
            self._cancel_restart_locked()
            self._desired_mode = "lora"
            self._desired_ntrip = None
            self._active_profile_id = None
            self._active_profile_revision = None
            self._restart_attempt = 0
            self._last_supervisor_error = None
            try:
                return await self._start_locked("lora", self._lora_script, args)
            except Exception:
                self._desired_mode = "idle"
                raise

    async def mark_ntrip_unavailable(self, error: str) -> RTKStatus:
        """Expose an autostart configuration failure through the status API."""
        async with self._lock:
            self._cancel_restart_locked()
            self._desired_mode = "ntrip"
            self._desired_ntrip = None
            self._active_profile_id = None
            self._active_profile_revision = None
            self._last_supervisor_error = str(error)
            await self._stop_locked()
            return self._status_locked()

    async def stop_all(self) -> RTKStatus:
        """Stop any active RTK child and return the resulting idle status."""
        async with self._lock:
            self._desired_mode = "idle"
            self._desired_ntrip = None
            self._active_profile_id = None
            self._active_profile_revision = None
            self._restart_attempt = 0
            self._last_supervisor_error = None
            self._cancel_restart_locked()
            await self._stop_locked()
            return self._status_locked()

    async def status(self) -> RTKStatus:
        async with self._lock:
            if self._process is not None and self._process.returncode is not None:
                self._last_supervisor_error = (
                    f"{self._mode} subprocess exited with code {self._process.returncode}"
                )
                self._clear_process_locked()
                self._schedule_ntrip_restart_locked()
            return self._status_locked()

    async def _start_ntrip_locked(self, config: NtripConfig) -> RTKStatus:
        args = [
            "--host",
            config.host,
            "--port",
            str(config.port),
            "--mountpoint",
            config.mountpoint,
            "--user",
            config.user,
            "--pass-stdin",
        ]
        return await self._start_locked(
            "ntrip",
            self._ntrip_script,
            args,
            stdin_payload=f"{config.password}\n",
        )

    async def _start_locked(
        self,
        mode: Literal["ntrip", "lora"],
        script: Path,
        args: list[str],
        stdin_payload: str | None = None,
    ) -> RTKStatus:
        """Start one child. Caller must hold ``self._lock``."""
        if not script.exists():
            raise RTKProcessError(f"{mode} script not found: {script}")

        await self._stop_locked()

        status_file = self._new_status_file(mode)
        args = [*args, "--status-file", str(status_file)]
        safe_args = self._redact_args(args)
        cmd = [self._python, str(script), *args]
        self._log.info("starting %s RTK subprocess: %s %s", mode, self._python, safe_args)

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
        except Exception as exc:
            if process is not None:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
                await process.wait()
            self._log.exception("failed to start %s RTK subprocess", mode)
            self._mode = "idle"
            self._process = None
            self._remove_status_file(status_file)
            raise RTKProcessError(f"failed to start {mode} RTK subprocess: {exc}") from exc

        self._process = process
        self._mode = mode
        self._status_file = status_file
        self._watch_task = asyncio.create_task(
            self._watch_process(process, mode), name=f"rtk-{mode}-watch"
        )

        try:
            await asyncio.wait_for(process.wait(), timeout=self._startup_grace_s)
        except asyncio.TimeoutError:
            return self._status_locked()

        rc = process.returncode
        self._clear_process_locked()
        raise RTKProcessError(f"{mode} RTK subprocess exited immediately with code {rc}")

    async def _stop_locked(self) -> None:
        process = self._process
        if process is None:
            self._clear_process_locked()
            return

        mode = self._mode
        pid = process.pid
        self._log.info("stopping %s RTK subprocess pid=%s", mode, pid)

        if process.returncode is None:
            process.terminate()
            try:
                await asyncio.wait_for(process.wait(), timeout=self._shutdown_grace_s)
            except asyncio.TimeoutError:
                self._log.warning(
                    "%s RTK subprocess pid=%s did not exit after %.1fs; killing",
                    mode,
                    pid,
                    self._shutdown_grace_s,
                )
                process.kill()
                await process.wait()

        self._clear_process_locked()

    async def _watch_process(
        self, process: asyncio.subprocess.Process, mode: RTKMode
    ) -> None:
        rc = await process.wait()
        async with self._lock:
            if self._process is process:
                self._log.warning("%s RTK subprocess pid=%s exited rc=%s", mode, process.pid, rc)
                self._last_supervisor_error = f"{mode} subprocess exited with code {rc}"
                self._clear_process_locked()
                self._schedule_ntrip_restart_locked()

    def _cancel_restart_locked(self) -> None:
        task = self._restart_task
        if task is not None and not task.done() and task is not asyncio.current_task():
            task.cancel()
        self._restart_task = None

    def _schedule_ntrip_restart_locked(self) -> None:
        if (
            self._desired_mode != "ntrip"
            or self._desired_ntrip is None
            or self._process is not None
            or (self._restart_task is not None and not self._restart_task.done())
        ):
            return
        delay = min(30.0, 2.0 ** min(self._restart_attempt, 5))
        self._restart_attempt += 1
        self._log.warning("restarting NTRIP subprocess in %.1fs", delay)
        self._restart_task = asyncio.create_task(
            self._restart_ntrip_after(delay), name="rtk-ntrip-restart"
        )

    async def _restart_ntrip_after(self, delay_s: float) -> None:
        try:
            await asyncio.sleep(delay_s)
            async with self._lock:
                if asyncio.current_task() is self._restart_task:
                    self._restart_task = None
                config = self._desired_ntrip
                if self._desired_mode != "ntrip" or config is None or self._process is not None:
                    return
                try:
                    await self._start_ntrip_locked(config)
                except Exception as exc:
                    self._last_supervisor_error = str(exc)
                    self._log.error("supervised NTRIP restart failed: %s", exc)
                    self._schedule_ntrip_restart_locked()
                else:
                    self._supervisor_restarts += 1
                    self._restart_attempt = 0
                    self._last_supervisor_error = None
                    self._log.info("supervised NTRIP restart succeeded")
        except asyncio.CancelledError:
            raise

    def _clear_process_locked(self) -> None:
        self._process = None
        self._mode = "idle"
        if self._status_file is not None:
            self._remove_status_file(self._status_file)
        self._status_file = None
        task = self._watch_task
        if task is not None and not task.done() and task is not asyncio.current_task():
            task.cancel()
        self._watch_task = None

    # Max age (s) of the newest health anchor before the stream is unhealthy.
    _HEALTHY_MAX_AGE_S = 10.0

    @classmethod
    def _health_anchor_age_s(cls, child_status: dict[str, Any], now: float) -> float | None:
        """Age since the newest of: last valid RTCM frame, or the current
        connection's establishment (``connected_since``).

        ``last_frame_time`` in the child's status file survives reconnects, so
        judging health on it alone marks every fresh reconnect after a
        >_HEALTHY_MAX_AGE_S outage unhealthy before the new connection has any
        chance to stream (NTRIP VRS casters may not send frames until the
        first GGA back-feed, up to 10 s after handshake). Anchoring on the
        newest of the two gives each new connection one honest grace window,
        after which missing frames correctly turn the stream unhealthy.
        Children predating ``connected_since`` degrade to the old behaviour.
        """
        anchors = [
            float(t)
            for t in (
                child_status.get("last_frame_time"),
                child_status.get("connected_since"),
            )
            if isinstance(t, (int, float)) and not isinstance(t, bool)
        ]
        if not anchors:
            return None
        return max(0.0, now - max(anchors))

    def _status_locked(self) -> RTKStatus:
        process = self._process
        running = process is not None and process.returncode is None
        child_status = self._read_child_status() if running else {}
        now = time.time()
        last_frame_time = child_status.get("last_frame_time")
        # Honest frame age for display; health is judged on the anchor age.
        last_frame_age_s = (
            max(0.0, now - float(last_frame_time))
            if isinstance(last_frame_time, (int, float))
            else None
        )
        anchor_age_s = self._health_anchor_age_s(child_status, now)
        if running:
            fallback_state = "running"
        elif self._desired_mode == "ntrip" and self._restart_task is not None:
            fallback_state = "restarting"
        elif self._desired_mode == "ntrip":
            fallback_state = "unavailable"
        else:
            fallback_state = "idle"
        source_state = str(child_status.get("state") or fallback_state)
        healthy = bool(
            running
            and child_status.get("connected", False)
            and anchor_age_s is not None
            and anchor_age_s <= self._HEALTHY_MAX_AGE_S
        )
        return RTKStatus(
            mode=self._mode if running else "idle",
            pid=process.pid if running else None,
            running=running,
            healthy=healthy,
            source_state=source_state,
            frames=int(child_status.get("frames", 0) or 0),
            bytes=int(child_status.get("bytes", 0) or 0),
            last_frame_age_s=last_frame_age_s,
            last_error=child_status.get("last_error") or self._last_supervisor_error,
            desired_mode=self._desired_mode,
            supervisor_restarts=self._supervisor_restarts,
            active_profile_id=self._active_profile_id,
            active_profile_revision=self._active_profile_revision,
        )

    @staticmethod
    def _redact_args(args: list[str]) -> list[str]:
        redacted = list(args)
        for i, arg in enumerate(redacted[:-1]):
            if arg == "--pass":
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
        except Exception:
            self._log.warning("failed to read RTK status file %s", path, exc_info=True)
            return {}
