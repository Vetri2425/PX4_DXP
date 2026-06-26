"""RPP debug array decoder and done-detection helper."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from config import DONE_SETTLE_S, RPP_DEBUG_STALE_MS, RPP_DONE, RPP_STATE_NAMES


@dataclass
class RppSnapshot:
    xtrack_m:        float = 0.0
    heading_err_deg: float = 0.0
    lookahead_m:     float = 0.0
    speed_m_s:       float = 0.0
    kappa:           float = 0.0
    dist_to_goal_m:  float = 0.0
    pose_age_ms:     float = 0.0
    state_code:      int   = 0
    state_name:      str   = "IDLE"
    timestamp:       float = field(default_factory=time.monotonic)

    @classmethod
    def from_debug_array(cls, data: list[float]) -> "RppSnapshot":
        code = int(data[7])
        return cls(
            xtrack_m        = data[0],
            heading_err_deg = math.degrees(data[1]),
            lookahead_m     = data[2],
            speed_m_s       = data[3],
            kappa           = data[4],
            dist_to_goal_m  = data[5],
            pose_age_ms     = data[6],
            state_code      = code,
            state_name      = RPP_STATE_NAMES.get(code, "UNKNOWN"),
            timestamp       = time.monotonic(),
        )


class RppStatusMonitor:
    """Wraps the latest RppSnapshot with done-settle logic.

    Single source of truth for RPP state. Updated from
    `RosBridgeNode._cb_rpp_debug` on each /rpp/debug message.
    """

    def __init__(self, done_settle_s: float = DONE_SETTLE_S) -> None:
        self._done_settle_s = done_settle_s
        self._done_since: float | None = None
        self._snapshot = RppSnapshot()
        self._has_snapshot = False

    def update(self, data: list[float]) -> None:
        if len(data) >= 8:
            self._snapshot = RppSnapshot.from_debug_array(data)
            self._has_snapshot = True
            if self._snapshot.state_code == RPP_DONE:
                if self._done_since is None:
                    self._done_since = time.monotonic()
            else:
                self._done_since = None

    def reset(self) -> None:
        """Reset done-settle timer. Call when a new path is loaded so stale
        DONE state from the previous mission does not trigger instant
        auto-completion."""
        self._done_since = None

    def get_snapshot(self) -> RppSnapshot:
        return self._snapshot

    def snapshot_age_s(self) -> float | None:
        if not self._has_snapshot:
            return None
        return max(0.0, time.monotonic() - self._snapshot.timestamp)

    def is_fresh(self, max_age_s: float | None = None) -> bool:
        if not self._has_snapshot:
            return False
        limit = RPP_DEBUG_STALE_MS / 1000.0 if max_age_s is None else max_age_s
        age = self.snapshot_age_s()
        return age is not None and age <= limit

    def has_snapshot(self, *, fresh: bool = False) -> bool:
        if not self._has_snapshot:
            return False
        return self.is_fresh() if fresh else True

    def is_done(self) -> bool:
        """True only after DONE state has been held for `done_settle_s`."""
        if self._done_since is None:
            return False
        if not self.is_fresh():
            return False
        return (time.monotonic() - self._done_since) >= self._done_settle_s

    def is_tracking(self) -> bool:
        return self.is_fresh() and self._snapshot.state_code in (1, 2)
