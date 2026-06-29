"""Mission operation coordination — priority, preemption, and stale tokens."""

from __future__ import annotations

import asyncio
import threading
import time
from dataclasses import dataclass
from enum import Enum


class MissionOperation(str, Enum):
    ESTOP = "estop"
    ABORT = "abort"
    STOP = "stop"
    COMPLETION = "completion"
    RESTART = "restart"
    SKIP = "skip"
    PAUSE = "pause"
    RESUME = "resume"
    CONTINUE = "continue"


OPERATION_PRIORITY = {
    MissionOperation.ESTOP: 100,
    MissionOperation.ABORT: 90,
    MissionOperation.STOP: 80,
    MissionOperation.COMPLETION: 70,
    MissionOperation.RESTART: 60,
    MissionOperation.SKIP: 50,
    MissionOperation.PAUSE: 40,
    MissionOperation.RESUME: 40,
    MissionOperation.CONTINUE: 40,
}


class MissionOperationConflict(Exception):
    """Raised when begin() times out or a stale token is rejected."""

    def __init__(self, active: str, message: str | None = None) -> None:
        self.active = active
        self.message = message or f"mission operation rejected: {active} in progress"
        super().__init__(self.message)


@dataclass(frozen=True)
class MissionOperationToken:
    operation: MissionOperation
    priority: int
    generation: int
    started_mono_s: float
    preempted: asyncio.Event

    def raise_if_stale(self, current_generation: int) -> None:
        if self.generation != current_generation:
            raise MissionOperationConflict(
                self.operation.value,
                "mission operation token is stale",
            )

    def is_preempted(self) -> bool:
        return self.preempted.is_set()


class MissionOperationCoordinator:
    def __init__(self) -> None:
        # Fast in-memory mutations only — threading lock keeps begin_estop_nowait()
        # safe without awaiting (physical estop runs after token issue).
        self._sync_lock = threading.Lock()
        self._generation = 0
        self._active: MissionOperationToken | None = None

    def current_generation(self) -> int:
        return self._generation

    def is_current(self, token: MissionOperationToken) -> bool:
        return (
            token.generation == self._generation
            and self._active is token
            and not token.is_preempted()
        )

    def _preempt_active(self) -> None:
        if self._active is not None:
            self._active.preempted.set()

    def _issue_token(self, operation: MissionOperation) -> MissionOperationToken:
        priority = OPERATION_PRIORITY[operation]
        # Pause/resume/continue coordinate operator requests without invalidating
        # the active point run generation tracked by the orchestrator loop.
        if priority >= OPERATION_PRIORITY[MissionOperation.SKIP]:
            self._generation += 1
        token = MissionOperationToken(
            operation=operation,
            priority=priority,
            generation=self._generation,
            started_mono_s=time.monotonic(),
            preempted=asyncio.Event(),
        )
        self._active = token
        return token

    async def begin(
        self, operation: MissionOperation, *, timeout_s: float
    ) -> MissionOperationToken:
        priority = OPERATION_PRIORITY[operation]
        deadline = time.monotonic() + timeout_s
        while True:
            with self._sync_lock:
                active = self._active
                if active is None or active.is_preempted():
                    self._preempt_active()
                    return self._issue_token(operation)
                if priority > active.priority:
                    self._preempt_active()
                    return self._issue_token(operation)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise MissionOperationConflict(active.operation.value)
            await asyncio.sleep(min(0.01, max(0.0, deadline - time.monotonic())))

    async def finish(self, token: MissionOperationToken) -> None:
        with self._sync_lock:
            if self._active is token:
                self._active = None

    def begin_estop_nowait(self) -> MissionOperationToken:
        with self._sync_lock:
            self._preempt_active()
            return self._issue_token(MissionOperation.ESTOP)