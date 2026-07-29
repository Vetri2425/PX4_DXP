"""S5 — planning concurrency bound.

_plan_in_thread wraps the five heavy planner offloads (plan_path /
plan_segments) in a Semaphore(2): with CPUQuota=200% on the unit, more than
two concurrent plans just time-share until ALL of them hit the 15 s route
timeout. The semaphore turns total failure into a queue.

The design decision pinned here: the semaphore sits INSIDE the route's
asyncio.wait_for, so a request stuck in the queue still times out with the
route's own 504 rather than waiting forever for a permit.
"""
import asyncio
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(__file__))

import pytest

import routes.path as path_route

pytestmark = pytest.mark.anyio


@pytest.fixture
def anyio_backend():
    return "asyncio"


async def test_third_plan_queues_and_still_times_out():
    release = threading.Event()
    started = []

    def slow_plan():
        started.append(True)
        release.wait(timeout=5.0)
        return "done"

    t1 = asyncio.create_task(path_route._plan_in_thread(slow_plan))
    t2 = asyncio.create_task(path_route._plan_in_thread(slow_plan))
    # Let the two permit-holders actually enter the worker threads.
    for _ in range(50):
        if len(started) == 2:
            break
        await asyncio.sleep(0.02)
    assert len(started) == 2

    # Third request: both permits held -> it must queue, and a wait_for around
    # it (as every route does) must still fire.
    with pytest.raises(asyncio.TimeoutError):
        await asyncio.wait_for(path_route._plan_in_thread(slow_plan), timeout=0.2)
    # The queued call never reached the thread pool — the permit was never granted.
    assert len(started) == 2

    release.set()
    assert await t1 == "done"
    assert await t2 == "done"


async def test_permits_are_released_after_completion():
    # After the burst above, two fresh plans must both run immediately.
    ran = []

    def quick():
        ran.append(True)
        return 1

    r = await asyncio.gather(
        path_route._plan_in_thread(quick), path_route._plan_in_thread(quick)
    )
    assert r == [1, 1] and len(ran) == 2
