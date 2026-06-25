"""Authoritative mission/joystick ownership arbitration."""
from __future__ import annotations

import asyncio
import contextvars
from contextlib import asynccontextmanager
from enum import Enum
from typing import Any, AsyncIterator, NamedTuple

from models import MissionState


class ControlArbiterError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


class ControlOwner(str, Enum):
    IDLE = "idle"
    MISSION = "mission"
    JOYSTICK_ACQUIRING = "joystick_acquiring"
    JOYSTICK_ACTIVE = "joystick_active"
    JOYSTICK_HELD = "joystick_held"
    RELEASING = "releasing"


MISSION_ACTIVE_STATES = {
    MissionState.LOADING,
    MissionState.ARMING,
    MissionState.SWITCHING_OFFBOARD,
    MissionState.RUNNING,
    MissionState.STOPPING,
    MissionState.DISARMING,
}

class _ArbiterReentry(NamedTuple):
    depth: int
    task: asyncio.Task | None


_NO_REENTRY = _ArbiterReentry(0, None)
_IN_ARBITER: contextvars.ContextVar[_ArbiterReentry] = contextvars.ContextVar(
    "control_arbiter_reentry",
    default=_NO_REENTRY,
)


def clear_reentry_context():
    """Clear inherited arbiter re-entry for independently scheduled tasks."""
    return _IN_ARBITER.set(_NO_REENTRY)


def reset_reentry_context(token) -> None:
    _IN_ARBITER.reset(token)


class ControlArbiter:
    """Single lock for control ownership and mode-transition decisions."""

    def __init__(self) -> None:
        self._lock: asyncio.Lock | None = None
        self._owner = ControlOwner.IDLE
        self._joystick_session_id: str | None = None
        self._joystick_lease_id: str | None = None
        self._stop_reason: str | None = None

    def _control_lock(self) -> asyncio.Lock:
        if self._lock is None:
            self._lock = asyncio.Lock()
        return self._lock

    @property
    def owner(self) -> ControlOwner:
        return self._owner

    @property
    def joystick_owned(self) -> bool:
        return self._owner in {
            ControlOwner.JOYSTICK_ACQUIRING,
            ControlOwner.JOYSTICK_ACTIVE,
            ControlOwner.JOYSTICK_HELD,
            ControlOwner.RELEASING,
        }

    @asynccontextmanager
    async def mission_start(self, offboard_ctrl: Any) -> AsyncIterator[None]:
        async with self._locked():
            if self.joystick_owned:
                raise ControlArbiterError(
                    "joystick_active",
                    "mission start rejected: joystick owns manual control",
                )
            self._owner = ControlOwner.MISSION
            try:
                yield
            finally:
                state = getattr(offboard_ctrl, "state", None)
                if state not in MISSION_ACTIVE_STATES:
                    self._owner = ControlOwner.IDLE

    @asynccontextmanager
    async def joystick_acquire(self, offboard_ctrl: Any) -> AsyncIterator[None]:
        async with self._locked():
            self._reject_if_mission_active(offboard_ctrl)
            if self.joystick_owned:
                raise ControlArbiterError(
                    "joystick_active",
                    "joystick control is already owned",
                )
            self._owner = ControlOwner.JOYSTICK_ACQUIRING
            try:
                yield
            except Exception:
                self.clear_joystick(reason="acquire_failed")
                raise

    async def ensure_mission_motion_allowed(self, offboard_ctrl: Any) -> None:
        async with self._locked():
            if self.joystick_owned:
                raise ControlArbiterError(
                    "joystick_active",
                    "mission motion rejected: joystick owns manual control",
                )
            self._reject_if_mission_start_conflict(offboard_ctrl)

    async def ensure_offboard_endpoint_allowed(self) -> None:
        async with self._locked():
            raise ControlArbiterError(
                "offboard_requires_mission_start",
                "OFFBOARD transitions must go through mission start",
            )

    def mark_joystick_active(self, session_id: str, lease_id: str) -> None:
        self._owner = ControlOwner.JOYSTICK_ACTIVE
        self._joystick_session_id = session_id
        self._joystick_lease_id = lease_id
        self._stop_reason = None

    def mark_joystick_held(self) -> None:
        if self.joystick_owned:
            self._owner = ControlOwner.JOYSTICK_HELD

    def mark_releasing(self) -> None:
        if self.joystick_owned:
            self._owner = ControlOwner.RELEASING

    def clear_joystick(self, *, reason: str) -> None:
        self._owner = ControlOwner.IDLE
        self._joystick_session_id = None
        self._joystick_lease_id = None
        self._stop_reason = reason

    def mark_idle_if_not_joystick(self) -> None:
        if not self.joystick_owned:
            self._owner = ControlOwner.IDLE

    def snapshot(self) -> dict[str, Any]:
        return {
            "control_owner": self._owner.value,
            "joystick_owned": self.joystick_owned,
            "joystick_owner_present": self._joystick_session_id is not None,
            "joystick_has_lease": self._joystick_lease_id is not None,
            "joystick_stop_reason": self._stop_reason,
        }

    @asynccontextmanager
    async def hold(self) -> AsyncIterator[None]:
        """Serialize arbiter mutations (re-entrant within one task)."""
        async with self._locked():
            yield

    @asynccontextmanager
    async def _locked(self) -> AsyncIterator[None]:
        current = asyncio.current_task()
        reentry = _IN_ARBITER.get()
        if reentry.depth > 0 and reentry.task is current:
            token = _IN_ARBITER.set(_ArbiterReentry(reentry.depth + 1, current))
            try:
                yield
            finally:
                _IN_ARBITER.reset(token)
            return
        async with self._control_lock():
            token = _IN_ARBITER.set(_ArbiterReentry(1, current))
            try:
                yield
            finally:
                _IN_ARBITER.reset(token)

    def _reject_if_mission_active(self, offboard_ctrl: Any) -> None:
        state = getattr(offboard_ctrl, "state", None)
        if state in MISSION_ACTIVE_STATES:
            value = getattr(state, "value", str(state))
            raise ControlArbiterError(
                "mission_active",
                f"joystick acquire rejected: mission state is {value}",
            )
        if self._owner == ControlOwner.MISSION:
            raise ControlArbiterError(
                "mission_active",
                "joystick acquire rejected: mission transition in progress",
            )

    def _reject_if_mission_start_conflict(self, offboard_ctrl: Any) -> None:
        state = getattr(offboard_ctrl, "state", None)
        if state in {MissionState.ARMING, MissionState.SWITCHING_OFFBOARD, MissionState.STOPPING}:
            value = getattr(state, "value", str(state))
            raise ControlArbiterError(
                "mission_transition",
                f"mission motion rejected: controller state is {value}",
            )


_ARBITER: ControlArbiter | None = None


def get_control_arbiter() -> ControlArbiter:
    global _ARBITER
    if _ARBITER is None:
        _ARBITER = ControlArbiter()
    return _ARBITER


def reset_control_arbiter_for_tests() -> ControlArbiter:
    global _ARBITER
    _ARBITER = ControlArbiter()
    return _ARBITER
