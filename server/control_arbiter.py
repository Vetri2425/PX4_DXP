"""Authoritative mission/joystick ownership arbitration.

Single-owner invariant: MISSION and JOYSTICK ownership are never simultaneous.
The lock is held only across the short ownership-*claim* step, never across a
caller's slow body (FCU handshake, mode-switch confirm, ROS I/O). Nothing here
ever re-enters `_control_lock()` from within a body it is still holding, so —
unlike the `feat/entry-pivot-recenter` reference this was ported from — no
contextvars re-entry shim is needed. If a future change ever needs a locked
call from inside another locked call's body, that is a sign the call graph
grew a nested acquire; fix the call graph instead of reintroducing the shim
(plan docs/Architecture/JOYSTICK_CONTROLLER_PLAN.md §7.4).
"""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from enum import Enum
from typing import Any, AsyncIterator

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
    MissionState.ENTRY,
    MissionState.RUNNING,
    MissionState.STOPPING,
    MissionState.DISARMING,
}


class ControlArbiter:
    """Single lock for control ownership transitions."""

    def __init__(self) -> None:
        self._lock: asyncio.Lock | None = None
        self._owner = ControlOwner.IDLE
        self._joystick_session_id: str | None = None
        self._joystick_lease_id: str | None = None
        self._stop_reason: str | None = None

    def _control_lock(self) -> asyncio.Lock:
        # Lazily created: bound to whichever event loop first calls a locked
        # method, mirroring OffboardController._lifecycle_lock().
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

    # ── Mission side ─────────────────────────────────────────────────────────

    @asynccontextmanager
    async def mission_start(self, offboard_ctrl: Any) -> AsyncIterator[None]:
        """Bracket one mission lifecycle call (start/stop) against the joystick.

        The lock is held only to make the claim; the caller's body (arm,
        mode-switch, ROS I/O) runs unlocked. `owner == MISSION` means strictly
        "a mission lifecycle call is in flight right now" — nothing more. It is
        deliberately NOT sticky for the mission's runtime: whether a mission is
        actually driving is derived from `offboard_ctrl.state`
        (MISSION_ACTIVE_STATES), which is what `begin_joystick_acquire` checks.

        On exit this bracket ALWAYS relinquishes (unless the joystick somehow
        owns, which the entry guard forbids). This is the load-bearing
        anti-stranding property: a mission can leave its active state OUTSIDE
        this bracket — `OffboardController.mark_completed()` and
        `advance_entry_to_marking()` run in the telemetry loop, abort/disarm run
        in their own calls — none of which touch the arbiter. If `owner` stayed
        MISSION across the running phase, a naturally-completed mission would
        strand it at MISSION forever and permanently reject every future
        joystick acquire (via `_reject_if_mission_active`). Keeping ownership
        in-flight-only makes that class of bug unrepresentable.

        `offboard_ctrl` is unused here now (the guard reads state at claim time)
        but kept in the signature for call-site symmetry and future use.
        """
        async with self._control_lock():
            if self.joystick_owned:
                raise ControlArbiterError(
                    "joystick_active",
                    "mission start rejected: joystick owns manual control",
                )
            self._owner = ControlOwner.MISSION
        try:
            yield
        finally:
            if not self.joystick_owned:
                self._owner = ControlOwner.IDLE

    async def ensure_mission_motion_allowed(self, offboard_ctrl: Any) -> None:
        """Reject mission motion if the joystick owns control or a mission
        state transition is racing this call."""
        async with self._control_lock():
            if self.joystick_owned:
                raise ControlArbiterError(
                    "joystick_active",
                    "mission motion rejected: joystick owns manual control",
                )
            state = getattr(offboard_ctrl, "state", None)
            if state in {
                MissionState.ARMING,
                MissionState.SWITCHING_OFFBOARD,
                MissionState.STOPPING,
            }:
                value = getattr(state, "value", str(state))
                raise ControlArbiterError(
                    "mission_transition",
                    f"mission motion rejected: controller state is {value}",
                )

    # ── Joystick side ────────────────────────────────────────────────────────

    async def begin_joystick_acquire(self, offboard_ctrl: Any) -> None:
        """Claim JOYSTICK_ACQUIRING ownership. Raises if a mission is active
        or the joystick is already owned by someone else.

        Callers must follow up with `mark_joystick_active()` on success or
        `clear_joystick()` on failure — both are plain unlocked mutators,
        safe because only the task that won this claim mutates joystick state
        until it either activates or clears.
        """
        async with self._control_lock():
            self._reject_if_mission_active(offboard_ctrl)
            if self.joystick_owned:
                raise ControlArbiterError(
                    "joystick_active",
                    "joystick control is already owned",
                )
            self._owner = ControlOwner.JOYSTICK_ACQUIRING
            self._stop_reason = None

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

    # ── Introspection ────────────────────────────────────────────────────────

    def snapshot(self) -> dict[str, Any]:
        return {
            "control_owner": self._owner.value,
            "joystick_owned": self.joystick_owned,
            "joystick_owner_present": self._joystick_session_id is not None,
            "joystick_has_lease": self._joystick_lease_id is not None,
            "joystick_stop_reason": self._stop_reason,
        }


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
