"""Async subprocess orchestration for RTK correction streams."""

from __future__ import annotations

import asyncio
import json
import logging
import tempfile
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

RTKMode = Literal["ntrip", "lora", "idle"]


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


class RTKProcessError(RuntimeError):
    """Raised when an RTK subprocess cannot be started cleanly."""


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
        shutdown_grace_s: float = 5.0,
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

    async def start_ntrip(
        self,
        *,
        host: str,
        port: int,
        mountpoint: str,
        user: str,
        password: str,
    ) -> RTKStatus:
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
        return await self._start(
            "ntrip",
            self._ntrip_script,
            args,
            stdin_payload=f"{password}\n",
        )

    async def start_lora(self, *, baudrate: int, serial_port: str) -> RTKStatus:
        args = [
            "--baudrate",
            str(baudrate),
            "--serial-port",
            serial_port,
        ]
        return await self._start("lora", self._lora_script, args)

    async def stop_all(self) -> RTKStatus:
        """Stop any active RTK child and return the resulting idle status."""
        async with self._lock:
            await self._stop_locked()
            return self._status_locked()

    async def status(self) -> RTKStatus:
        async with self._lock:
            if self._process is not None and self._process.returncode is not None:
                self._clear_process_locked()
            return self._status_locked()

    async def _start(
        self,
        mode: Literal["ntrip", "lora"],
        script: Path,
        args: list[str],
        stdin_payload: str | None = None,
    ) -> RTKStatus:
        async with self._lock:
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
                self._clear_process_locked()

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

    def _status_locked(self) -> RTKStatus:
        process = self._process
        running = process is not None and process.returncode is None
        child_status = self._read_child_status() if running else {}
        last_frame_time = child_status.get("last_frame_time")
        last_frame_age_s = (
            max(0.0, time.time() - float(last_frame_time))
            if isinstance(last_frame_time, (int, float))
            else None
        )
        source_state = str(child_status.get("state") or ("running" if running else "idle"))
        healthy = bool(
            running
            and child_status.get("connected", False)
            and last_frame_age_s is not None
            and last_frame_age_s <= 10.0
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
            last_error=child_status.get("last_error"),
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
