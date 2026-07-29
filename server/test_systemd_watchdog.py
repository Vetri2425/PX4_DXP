#!/usr/bin/env python3
"""S1-a / S2 — systemd WATCHDOG + safety-task isolation from Socket.IO emits."""

from __future__ import annotations

import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import main
from config import TELEMETRY_HZ
from models import MissionState


class _FakeNotifier:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def notify(self, msg: str) -> None:
        self.calls.append(msg)


def test_watchdog_heartbeat_fires_when_ros_node_none():
    """Degraded boot (rclpy.init failed → ros_node is None) must still feed systemd.

    After S2 the heartbeat lives on the safety task (not telemetry). S1-a rule
    still applies: it must fire above the ros_node None continue.
    """
    notifier = _FakeNotifier()
    prev_node = main.ros_node
    prev_notifier = main._sd_notifier
    prev_offboard = main.offboard_ctrl
    prev_emergency = main.emergency_handler
    main.ros_node = None
    main.offboard_ctrl = None
    main.emergency_handler = None
    main._sd_notifier = notifier

    ticks = {"n": 0}
    need = TELEMETRY_HZ * 3 + 2

    async def _fast_sleep(_seconds: float):
        ticks["n"] += 1
        if ticks["n"] > need:
            raise asyncio.CancelledError()
        return None

    async def _run():
        prev_sleep = asyncio.sleep
        asyncio.sleep = _fast_sleep  # type: ignore[assignment]
        try:
            try:
                await main._safety_watchdog_loop()
            except asyncio.CancelledError:
                pass
        finally:
            asyncio.sleep = prev_sleep  # type: ignore[assignment]

    try:
        asyncio.run(_run())
    finally:
        main.ros_node = prev_node
        main._sd_notifier = prev_notifier
        main.offboard_ctrl = prev_offboard
        main.emergency_handler = prev_emergency

    assert "WATCHDOG=1" in notifier.calls, (
        f"expected WATCHDOG=1 with ros_node=None; got {notifier.calls!r} "
        f"after {ticks['n']} ticks"
    )


def test_stalled_emit_does_not_block_safety_estop():
    """A never-resolving Socket.IO emit must not prevent the safety task estop.

    Pre-S2 the E-stop shared the telemetry tick with sequential emits; a phone
    leaving WiFi with a full TCP buffer could stall the await and starve the
    abort. Safety task calls only get_state + estop_async — never sio.emit.
    """
    estops: list[str] = []

    class _EH:
        async def estop_async(self):
            estops.append("estop")
            return {"ok": True}

    class _Ros:
        def get_state(self):
            return {
                "rpp_state": -1,  # STALE
                "pose_age_ms": 5000.0,
                "rpp_debug_age_ms": 5000.0,
                "connected": False,
            }

    class _Offboard:
        state = MissionState.RUNNING

    prev = {
        "ros": main.ros_node,
        "off": main.offboard_ctrl,
        "eh": main.emergency_handler,
        "q": main._safety_abort_q,
        "sd": main._sd_notifier,
        "grace": main.SAFETY_STALE_GRACE_S,
    }

    async def _never_resolves(*_a, **_k):
        await asyncio.Future()  # wedged emit stand-in

    async def _run():
        main.ros_node = _Ros()
        main.offboard_ctrl = _Offboard()
        main.emergency_handler = _EH()
        main._safety_abort_q = asyncio.Queue()
        main._sd_notifier = None
        main.SAFETY_STALE_GRACE_S = 0.05

        wedged = asyncio.create_task(_never_resolves())
        safety = asyncio.create_task(main._safety_watchdog_loop())
        try:
            for _ in range(100):
                if estops:
                    break
                await asyncio.sleep(0.02)
            # Abort payload must be queued for telemetry — not emitted by safety.
            assert not main._safety_abort_q.empty(), "estop must enqueue safety_abort"
        finally:
            safety.cancel()
            wedged.cancel()
            try:
                await safety
            except (asyncio.CancelledError, Exception):
                pass
            try:
                await wedged
            except (asyncio.CancelledError, Exception):
                pass

    try:
        asyncio.run(_run())
    finally:
        main.ros_node = prev["ros"]
        main.offboard_ctrl = prev["off"]
        main.emergency_handler = prev["eh"]
        main._safety_abort_q = prev["q"]
        main._sd_notifier = prev["sd"]
        main.SAFETY_STALE_GRACE_S = prev["grace"]

    assert estops, "safety watchdog must estop while a Socket.IO emit is wedged"


def test_emit_authenticated_times_out_instead_of_hanging():
    """Per-SID emit is bounded — a hung sio.emit must not block forever."""
    prev_sids = main.authenticated_sids

    async def _hung_emit(*_a, **_k):
        await asyncio.Future()

    async def _run():
        main.authenticated_sids = lambda: ["sid-stuck"]  # type: ignore[assignment]
        prev_emit = main.sio.emit
        main.sio.emit = _hung_emit  # type: ignore[assignment]
        try:
            t0 = asyncio.get_event_loop().time()
            await main._emit_authenticated("telemetry", {"ok": True})
            elapsed = asyncio.get_event_loop().time() - t0
        finally:
            main.sio.emit = prev_emit  # type: ignore[assignment]
        # Timeout is 0.5s; allow a little scheduling slack.
        assert elapsed < 2.0, f"emit hung too long: {elapsed:.2f}s"
        assert elapsed >= 0.4, f"emit returned too fast (timeout not exercised?): {elapsed:.2f}s"

    try:
        asyncio.run(_run())
    finally:
        main.authenticated_sids = prev_sids


if __name__ == "__main__":
    test_watchdog_heartbeat_fires_when_ros_node_none()
    print("PASS: test_watchdog_heartbeat_fires_when_ros_node_none")
    test_stalled_emit_does_not_block_safety_estop()
    print("PASS: test_stalled_emit_does_not_block_safety_estop")
    test_emit_authenticated_times_out_instead_of_hanging()
    print("PASS: test_emit_authenticated_times_out_instead_of_hanging")
