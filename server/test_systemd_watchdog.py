#!/usr/bin/env python3
"""S1-a / S2 — systemd WATCHDOG=1 must fire in ROS-degraded mode."""

from __future__ import annotations

import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import main
from config import TELEMETRY_HZ


class _FakeNotifier:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def notify(self, msg: str) -> None:
        self.calls.append(msg)


def test_watchdog_heartbeat_fires_when_ros_node_none():
    """Degraded boot (rclpy.init failed → ros_node is None) must still feed systemd.

    WatchdogSec=15 + a skipped heartbeat permanently kills the unit via
    StartLimitBurst. The heartbeat attests loop liveness, not ROS health.
    """
    notifier = _FakeNotifier()
    prev_node = main.ros_node
    prev_notifier = main._sd_notifier
    main.ros_node = None
    main._sd_notifier = notifier

    ticks = {"n": 0}
    # EVERY_N = TELEMETRY_HZ * 3 → need that many sleep returns to fire once.
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
                await main._telemetry_loop()
            except asyncio.CancelledError:
                pass
        finally:
            asyncio.sleep = prev_sleep  # type: ignore[assignment]

    try:
        asyncio.run(_run())
    finally:
        main.ros_node = prev_node
        main._sd_notifier = prev_notifier

    assert "WATCHDOG=1" in notifier.calls, (
        f"expected WATCHDOG=1 with ros_node=None; got {notifier.calls!r} "
        f"after {ticks['n']} ticks"
    )


if __name__ == "__main__":
    test_watchdog_heartbeat_fires_when_ros_node_none()
    print("PASS: test_watchdog_heartbeat_fires_when_ros_node_none")
