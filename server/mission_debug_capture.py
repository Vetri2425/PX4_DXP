"""Lightweight server side of the mission-debug recorder handshake."""
from __future__ import annotations

import asyncio
import datetime
import json
import os
import uuid
from pathlib import Path
from typing import Any

from config import (
    MISSION_CAPTURE_ACK_TIMEOUT_S,
    MISSION_CAPTURE_REQUIRED,
    MISSION_DEBUG_CONTROL_DIR,
    STAGING_DIR,
)
from logging_setup import get_logger

log = get_logger("server.mission_capture")


class CaptureUnavailable(RuntimeError):
    """Raised when capture is required but the recorder cannot become ready."""


def _utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, sort_keys=True, separators=(",", ":"))
        stream.flush()
        os.fsync(stream.fileno())
    os.chmod(tmp, 0o600)
    os.replace(tmp, path)


class MissionDebugCoordinator:
    """Create capture requests and publish small lifecycle update files."""

    def __init__(
        self,
        control_dir: str = MISSION_DEBUG_CONTROL_DIR,
        *,
        required: bool = MISSION_CAPTURE_REQUIRED,
        ack_timeout_s: float = MISSION_CAPTURE_ACK_TIMEOUT_S,
    ) -> None:
        self.root = Path(control_dir)
        self.required = bool(required)
        self.ack_timeout_s = float(ack_timeout_s)
        self.active_capture_id: str | None = None
        self.last_status: dict[str, Any] = {
            "state": "idle",
            "required": self.required,
        }

    def _path(self, group: str, capture_id: str) -> Path:
        return self.root / group / f"{capture_id}.json"

    async def begin_capture(
        self,
        controller,
        *,
        start_request: dict[str, Any],
        transport: str,
    ) -> str | None:
        summary = controller.loaded_path_summary()
        capture_id = str(uuid.uuid4())
        mission_id = summary.get("mission_id")
        staged_path = None
        if summary.get("is_staged") and mission_id:
            staged_path = str(Path(STAGING_DIR) / f"{os.path.basename(str(mission_id))}.json")

        request = {
            "protocol_version": 1,
            "capture_id": capture_id,
            "requested_at_utc": _utc_now(),
            "source_name": summary.get("source_name") or summary.get("name") or "mission",
            "loaded_mission_id": mission_id,
            "placement_mode": summary.get("placement_mode"),
            "origin_gps": summary.get("origin_gps"),
            "staged_artifact_path": staged_path,
            "loaded_path": summary,
            "start_request": {**start_request, "transport": transport},
        }
        request_path = self._path("requests", capture_id)
        ack_path = self._path("acks", capture_id)

        try:
            await asyncio.to_thread(_atomic_json, request_path, request)
            deadline = asyncio.get_running_loop().time() + self.ack_timeout_s
            while asyncio.get_running_loop().time() < deadline:
                try:
                    ack = await asyncio.to_thread(self._read_json, ack_path)
                except FileNotFoundError:
                    await asyncio.sleep(0.05)
                    continue
                if ack.get("ready"):
                    self.active_capture_id = capture_id
                    self.last_status = {**ack, "state": "ready", "required": self.required}
                    return capture_id
                raise CaptureUnavailable(ack.get("error") or "recorder rejected capture")
            raise CaptureUnavailable(
                f"mission-debug recorder did not acknowledge within {self.ack_timeout_s:.1f}s"
            )
        except Exception as exc:
            try:
                await asyncio.to_thread(
                    _atomic_json,
                    self._path("cancelled", capture_id),
                    {
                        "capture_id": capture_id,
                        "cancelled_at_utc": _utc_now(),
                        "reason": str(exc),
                    },
                )
            except Exception:
                log.exception("failed to cancel capture request %s", capture_id)
            self.last_status = {
                "state": "unavailable",
                "required": self.required,
                "capture_id": capture_id,
                "error": str(exc),
            }
            if self.required:
                if isinstance(exc, CaptureUnavailable):
                    raise
                raise CaptureUnavailable(str(exc)) from exc
            log.warning("mission capture unavailable; continuing by policy: %s", exc)
            return None

    @staticmethod
    def _read_json(path: Path) -> dict[str, Any]:
        with open(path, encoding="utf-8") as stream:
            data = json.load(stream)
        if not isinstance(data, dict):
            raise ValueError(f"invalid capture JSON object: {path}")
        return data

    def record_placement(self, capture_id: str | None, placement: dict[str, Any]) -> None:
        if capture_id is None:
            return
        # A debug-sidecar write must never propagate into mission control flow:
        # this runs inside the pre-publish hook on the lifecycle-locked start
        # path. Swallow and log any I/O error so a full disk cannot abort start.
        try:
            _atomic_json(
                self._path("placement", capture_id),
                {"capture_id": capture_id, "recorded_at_utc": _utc_now(), **placement},
            )
        except Exception:
            log.exception("record_placement failed for %s (capture unaffected)", capture_id)

    def record_start_result(
        self, capture_id: str | None, *, success: bool, state: str, message: str
    ) -> None:
        if capture_id is None:
            return
        try:
            _atomic_json(
                self._path("start-results", capture_id),
                {
                    "capture_id": capture_id,
                    "recorded_at_utc": _utc_now(),
                    "success": bool(success),
                    "state": state,
                    "message": message,
                },
            )
        except Exception:
            log.exception("record_start_result failed for %s", capture_id)
        if not success:
            self.record_terminal(capture_id, "start_failed", state=state, details=message)

    def record_terminal(
        self,
        capture_id: str | None,
        reason: str,
        *,
        state: str,
        details: Any = None,
    ) -> None:
        capture_id = capture_id or self.active_capture_id
        if capture_id is None:
            return
        try:
            _atomic_json(
                self._path("terminal", capture_id),
                {
                    "capture_id": capture_id,
                    "terminal_at_utc": _utc_now(),
                    "reason": reason,
                    "state": state,
                    "details": details,
                },
            )
        except Exception:
            log.exception("record_terminal failed for %s", capture_id)
        self.last_status = {
            **self.last_status,
            "state": "terminal_pending",
            "terminal_reason": reason,
        }
        # Clear the active id once a terminal is emitted so a later event with no
        # running mission (e.g. an idle e-stop) cannot be mis-attributed to a
        # finalized capture or leave an orphan terminal file.
        if capture_id == self.active_capture_id:
            self.active_capture_id = None

    def get_status(self) -> dict[str, Any]:
        status_path = self.root / "status.json"
        try:
            recorder_status = self._read_json(status_path)
        except (OSError, ValueError, json.JSONDecodeError):
            recorder_status = {}
        return {**self.last_status, **recorder_status, "required": self.required}
