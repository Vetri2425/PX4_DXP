"""Bounded Point mission event journal (512 events, process-local monotonic IDs)."""

from __future__ import annotations

import asyncio
import datetime
import threading
from collections import deque
from typing import Any, Callable, Awaitable

from logging_setup import get_logger
from models import PointMissionEvent

log = get_logger("server.point_events")

_JOURNAL_CAPACITY = 512
_EVENT_NAME = "point_mission_event"


class PointEventJournal:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._next_id = 1
        self._events: deque[PointMissionEvent] = deque(maxlen=_JOURNAL_CAPACITY)
        self._emit_cb: Callable[[str, dict[str, Any]], Awaitable[None]] | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

    def configure_emit(
        self,
        loop: asyncio.AbstractEventLoop,
        emit_cb: Callable[[str, dict[str, Any]], Awaitable[None]],
    ) -> None:
        self._loop = loop
        self._emit_cb = emit_cb

    def append(self, event: PointMissionEvent) -> PointMissionEvent:
        with self._lock:
            event_id = self._next_id
            self._next_id += 1
            stamped = event.model_copy(update={"event_id": event_id})
            self._events.append(stamped)
            payload = stamped.model_dump()
        self._schedule_emit(payload)
        return stamped

    def _schedule_emit(self, payload: dict[str, Any]) -> None:
        emit_cb = self._emit_cb
        loop = self._loop
        if emit_cb is None or loop is None:
            return

        def _dispatch() -> None:
            try:
                if loop.is_running():
                    asyncio.run_coroutine_threadsafe(
                        self._safe_emit(emit_cb, payload), loop
                    )
                else:
                    loop.create_task(self._safe_emit(emit_cb, payload))
            except Exception:
                log.exception("point event socket schedule failed")

        try:
            if threading.current_thread() is threading.main_thread() and loop.is_running():
                loop.create_task(self._safe_emit(emit_cb, payload))
            else:
                _dispatch()
        except Exception:
            log.exception("point event emit scheduling failed")

    @staticmethod
    async def _safe_emit(
        emit_cb: Callable[[str, dict[str, Any]], Awaitable[None]],
        payload: dict[str, Any],
    ) -> None:
        try:
            await emit_cb(_EVENT_NAME, payload)
        except Exception:
            log.exception("point event socket emit failed")

    def history(self, since_event_id: int | None = None) -> dict[str, Any]:
        with self._lock:
            events = list(self._events)
            latest = self._next_id - 1
            oldest = events[0].event_id if events else None
        if not events:
            return {
                "events": [],
                "latest_event_id": 0,
                "history_evicted": False,
                "oldest_available_event_id": None,
            }
        if since_event_id is None:
            selected = events
            evicted = False
        else:
            selected = [e for e in events if e.event_id > since_event_id]
            evicted = since_event_id < (oldest or 1) - 1
        return {
            "events": selected,
            "latest_event_id": latest,
            "history_evicted": evicted,
            "oldest_available_event_id": oldest,
        }


_journal: PointEventJournal | None = None


def get_point_event_journal() -> PointEventJournal:
    global _journal
    if _journal is None:
        _journal = PointEventJournal()
    return _journal


def reset_point_event_journal_for_tests() -> PointEventJournal:
    global _journal
    _journal = PointEventJournal()
    return _journal


def utc_ts() -> str:
    return datetime.datetime.utcnow().isoformat(timespec="milliseconds") + "Z"