"""Asynchronous startup reconciliation for residual spray/dwell state.

After ``rover-server`` restarts the point orchestrator begins IDLE with no run
token, but the spray node may still hold an active dwell or command-layer ON
state. This module reads fresh runtime status once the ROS bridge is up,
forces dwell cancel + confirmed OFF when residual activity is detected, and
blocks mission load/start until reconciliation completes.
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass, field
from typing import Any, Callable

from logging_setup import get_logger
from spray_safety import force_spray_off_confirmed

log = get_logger("server.spray_startup")


_STATUS_PROBE_FIELDS = (
    "active_dwell",
    "commanded_on",
    "confirmed_off",
    "status_stale",
)


def _truthy(status: dict[str, Any], name: str, actuator: dict[str, Any]) -> bool:
    return bool(status.get(name, actuator.get(name, False)))


def _read_runtime_status(ros_node) -> tuple[dict[str, Any] | None, str | None]:
    if ros_node is None or not hasattr(ros_node, "get_spray_runtime_status"):
        return None, "spray runtime status unavailable"
    try:
        status = dict(ros_node.get_spray_runtime_status())
    except Exception as exc:
        return None, f"spray runtime status failed: {exc}"
    return status, None


def _status_malformed(status: dict[str, Any]) -> str | None:
    actuator = status.get("actuator")
    if not isinstance(actuator, dict):
        actuator = {}
    for field_name in _STATUS_PROBE_FIELDS:
        value = status.get(field_name, actuator.get(field_name))
        if value is None:
            return f"missing required field {field_name!r}"
        if field_name != "status_stale" and not isinstance(value, bool):
            if field_name in status and not isinstance(status[field_name], bool):
                return f"field {field_name!r} must be bool"
    return None


def indicates_residual_spray_activity(status: dict[str, Any] | None) -> tuple[bool, str]:
    """Return whether fresh status suggests possible residual spray activity."""
    if status is None:
        return True, "spray runtime status unavailable"
    malformed = _status_malformed(status)
    if malformed:
        return True, malformed
    if bool(status.get("status_stale", True)):
        return True, "spray runtime status stale"

    actuator = status.get("actuator")
    if not isinstance(actuator, dict):
        actuator = {}

    accepted_on = _truthy(status, "accepted_command_on", actuator)
    pending = _truthy(status, "pending_command", actuator)
    pending_on = status.get("pending_command_on", actuator.get("pending_on"))
    pending_on_active = pending and pending_on is not False

    reasons: list[str] = []
    if _truthy(status, "active_dwell", actuator):
        reasons.append("active_dwell")
    if _truthy(status, "commanded_on", actuator):
        reasons.append("commanded_on")
    if accepted_on:
        reasons.append("accepted_on")
    if pending_on_active:
        reasons.append("pending_on")
    if not _truthy(status, "confirmed_off", actuator):
        reasons.append("confirmed_off not true")

    if reasons:
        return True, ", ".join(reasons)
    return False, ""


@dataclass
class SprayStartupReconciliationState:
    complete: bool = False
    in_progress: bool = False
    recovery_required: bool = False
    reason: str = ""
    dwell_cancel_result: dict[str, Any] | None = None
    spray_off_result: dict[str, Any] | None = None
    residual_detected: bool = False
    started_at_monotonic_s: float = 0.0
    finished_at_monotonic_s: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        return {
            "complete": self.complete,
            "in_progress": self.in_progress,
            "recovery_required": self.recovery_required,
            "reason": self.reason,
            "dwell_cancel_result": self.dwell_cancel_result,
            "spray_off_result": self.spray_off_result,
            "residual_detected": self.residual_detected,
            "started_at_monotonic_s": self.started_at_monotonic_s,
            "finished_at_monotonic_s": self.finished_at_monotonic_s,
        }


class SprayStartupReconciliation:
    _BRIDGE_WAIT_S = 5.0
    _BRIDGE_POLL_S = 0.1
    _OFF_TIMEOUT_S = 5.0

    def __init__(self) -> None:
        self._state = SprayStartupReconciliationState()
        self._ready = asyncio.Event()
        self._task: asyncio.Task | None = None

    @property
    def state(self) -> SprayStartupReconciliationState:
        return self._state

    def is_ready(self) -> bool:
        return (
            self._state.complete
            and not self._state.in_progress
            and not self._state.recovery_required
        )

    def blocks_mission_operations(self) -> bool:
        return not self.is_ready()

    def block_reason(self) -> str | None:
        if self._state.recovery_required:
            return (
                self._state.reason
                or "spray startup reconciliation requires operator recovery"
            )
        if self._state.in_progress or not self._state.complete:
            return "spray startup reconciliation in progress"
        return None

    def get_status(self) -> dict[str, Any]:
        return self._state.as_dict()

    async def wait_until_ready(self, timeout_s: float | None = None) -> bool:
        if self.is_ready():
            return True
        if timeout_s is None:
            await self._ready.wait()
            return self.is_ready()
        try:
            await asyncio.wait_for(self._ready.wait(), timeout=timeout_s)
        except asyncio.TimeoutError:
            return False
        return self.is_ready()

    def mark_bridge_unavailable(
        self,
        reason: str = "spray runtime unavailable during startup reconciliation",
    ) -> None:
        """ROS/spray bridge absent at startup — fail closed until recovery."""
        self._state = SprayStartupReconciliationState(
            complete=False,
            in_progress=False,
            recovery_required=True,
            reason=reason,
            finished_at_monotonic_s=time.monotonic(),
        )
        self._ready.clear()

    def start(self, ros_node, *, record: Callable[[str, str], None] | None = None) -> asyncio.Task:
        if (
            self._task is not None
            and not self._task.done()
            and not self._state.recovery_required
        ):
            return self._task
        self._ready.clear()
        self._state = SprayStartupReconciliationState(
            in_progress=True,
            started_at_monotonic_s=time.monotonic(),
        )
        self._task = asyncio.create_task(
            self._run(ros_node, record=record),
            name="spray-startup-reconciliation",
        )
        return self._task

    async def _wait_for_bridge(self, ros_node) -> bool:
        deadline = time.monotonic() + self._BRIDGE_WAIT_S
        while time.monotonic() < deadline:
            if ros_node is not None and hasattr(ros_node, "get_spray_runtime_status"):
                return True
            await asyncio.sleep(self._BRIDGE_POLL_S)
        return ros_node is not None and hasattr(ros_node, "get_spray_runtime_status")

    async def _cancel_dwell(self, ros_node) -> dict[str, Any]:
        if ros_node is None or not hasattr(ros_node, "cancel_spray_dwell_async"):
            return {"success": False, "message": "dwell cancel unavailable"}
        try:
            ok, message = await asyncio.wait_for(
                ros_node.cancel_spray_dwell_async(),
                timeout=self._OFF_TIMEOUT_S,
            )
            return {"success": bool(ok), "message": message or ""}
        except asyncio.TimeoutError:
            return {"success": False, "message": "dwell cancel timed out", "timeout": True}
        except Exception as exc:
            return {"success": False, "message": str(exc)}

    async def _run(
        self,
        ros_node,
        *,
        record: Callable[[str, str], None] | None = None,
    ) -> SprayStartupReconciliationState:
        def _log(level: str, message: str) -> None:
            if record is not None:
                record(level, message)
            getattr(log, level if level in ("info", "warning", "error") else "info")(message)

        try:
            bridge_ready = await self._wait_for_bridge(ros_node)
            if not bridge_ready:
                self._state.recovery_required = True
                self._state.reason = (
                    "spray runtime unavailable during startup reconciliation"
                )
                _log("error", self._state.reason)
                return self._state

            status, read_error = _read_runtime_status(ros_node)
            if status is None:
                self._state.recovery_required = True
                self._state.reason = read_error or (
                    "spray runtime unavailable during startup reconciliation"
                )
                _log("error", self._state.reason)
                return self._state

            residual, residual_reason = indicates_residual_spray_activity(status)
            self._state.residual_detected = residual

            if not residual:
                _log("info", "spray startup reconciliation: no residual activity detected")
                return self._state

            _log(
                "warning",
                f"spray startup reconciliation: residual activity detected ({residual_reason})",
            )

            dwell_cancel = await self._cancel_dwell(ros_node)
            self._state.dwell_cancel_result = dwell_cancel
            if hasattr(ros_node, "publish_spray_manual"):
                try:
                    ros_node.publish_spray_manual(False)
                except Exception as exc:
                    _log("warning", f"startup reconciliation manual OFF publish failed: {exc}")

            spray_off = await force_spray_off_confirmed(
                ros_node,
                timeout_s=self._OFF_TIMEOUT_S,
            )
            self._state.spray_off_result = spray_off.as_dict()

            verify_status, verify_error = _read_runtime_status(ros_node)
            still_residual, still_reason = indicates_residual_spray_activity(verify_status)
            off_confirmed = bool(spray_off.success)

            if not off_confirmed or still_residual:
                self._state.recovery_required = True
                parts = []
                if not off_confirmed:
                    parts.append(spray_off.failure_reason or spray_off.message)
                if still_residual:
                    parts.append(
                        verify_error or still_reason or "residual spray activity persists"
                    )
                self._state.reason = "; ".join(p for p in parts if p)
                _log(
                    "error",
                    "spray startup reconciliation failed: "
                    f"{self._state.reason}",
                )
            else:
                _log("info", "spray startup reconciliation: residual dwell cancelled and OFF confirmed")
        finally:
            self._state.in_progress = False
            self._state.finished_at_monotonic_s = time.monotonic()
            if self._state.recovery_required:
                self._state.complete = False
                self._ready.clear()
            else:
                self._state.complete = True
                self._ready.set()
        return self._state