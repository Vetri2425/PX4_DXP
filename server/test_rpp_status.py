import time

from config import RPP_DONE, RPP_TRACKING
from rpp_status import RppStatusMonitor


def _debug(state_code: int) -> list[float]:
    return [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 10.0, float(state_code)]


def test_done_requires_fresh_debug_snapshot():
    monitor = RppStatusMonitor(done_settle_s=0.0)
    monitor.update(_debug(RPP_DONE))
    assert monitor.is_done() is True

    monitor.get_snapshot().timestamp = time.monotonic() - 10.0
    assert monitor.has_snapshot() is True
    assert monitor.has_snapshot(fresh=True) is False
    assert monitor.is_done() is False


def test_tracking_requires_fresh_debug_snapshot():
    monitor = RppStatusMonitor(done_settle_s=0.0)
    monitor.update(_debug(RPP_TRACKING))
    assert monitor.is_tracking() is True

    monitor.get_snapshot().timestamp = time.monotonic() - 10.0
    assert monitor.is_tracking() is False
