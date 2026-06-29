"""BridgeHealthManager — MAVROS bridge liveness watchdog.

Phase 3A (default): OBSERVE-ONLY. Detects a frozen MAVROS bridge (no fresh
/mavros/state) and surfaces it via /api/health/bridge, the activity log, and
Socket.IO `bridge_health` events — but takes no corrective action.

Phase 3B (opt-in via ROVER_BRIDGE_AUTO_RECOVER=1): the same detector may
gracefully restart px4-dxp.service. That path is implemented here but gated
behind `auto_recover` (default False) and MUST be validated in the field
(kill mavros_node → observe clean auto-recovery) before being trusted.

Design notes:
- Liveness is keyed on /mavros/state freshness (`state_age_ms`) and the
  process-death-aware `fcu_connected`/publisher count — NOT pose freshness
  (pose can be legitimately absent without GPS/RTK).
- A recovery restart itself drops MAVROS ~11s; a cooldown prevents that from
  re-triggering. A backoff window caps recoveries; exhaustion → `failed`.
- Mid-mission, stop_async() is best-effort only — if the bridge is frozen it
  may not reach the FCU; PX4's own OFFBOARD failsafe is the real safety net.
"""
from __future__ import annotations

import asyncio
import time
from collections import deque
from typing import Any, Awaitable, Callable, Optional

from config import (
    BRIDGE_AUTO_RECOVER,
    BRIDGE_FROZEN_GRACE_S,
    BRIDGE_HEALTH_POLL_S,
    BRIDGE_RECOVERY_COOLDOWN_S,
    BRIDGE_RECOVERY_MAX,
    BRIDGE_RECOVERY_WINDOW_S,
    BRIDGE_STATE_STALE_MS,
)
from logging_setup import get_logger
from models import MissionState
from spray_safety import force_spray_off_confirmed, spray_off_blocks_success

log = get_logger("server.bridge")

HEALTHY = "healthy"
DEGRADED = "degraded"
RECOVERING = "recovering"
FAILED = "failed"


class BridgeHealthManager:
    def __init__(
        self,
        ros_node,
        offboard_ctrl,
        record: Callable[[str, str], None],
        emit: Callable[[str, dict], Awaitable[None]],
        *,
        auto_recover: bool = BRIDGE_AUTO_RECOVER,
    ) -> None:
        self._ros = ros_node
        self._ctrl = offboard_ctrl
        self._record = record
        self._emit = emit
        self._auto_recover = bool(auto_recover)

        self._health = HEALTHY
        self._degraded_since: Optional[float] = None  # monotonic
        self._cooldown_until: float = 0.0             # monotonic
        self._recovery_times: deque[float] = deque()  # monotonic, within window
        self._recovery_count = 0
        self._last_recovery_ts: Optional[float] = None      # wall clock
        self._last_recovery_reason: Optional[str] = None
        self._observe_alerted = False  # one-shot "would-recover" alert in 3A
        self._last_snapshot: dict[str, Any] = {}
        self._spray_recovery_required = False
        self._spray_recovery_reason: Optional[str] = None

        self._task: Optional[asyncio.Task] = None
        self._stop = asyncio.Event()

    # ── lifecycle ──────────────────────────────────────────────────────────────
    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._loop(), name="bridge-health")
            log.info(
                "BridgeHealthManager started (auto_recover=%s)", self._auto_recover
            )

    async def stop(self) -> None:
        self._stop.set()
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
            self._task = None

    # ── status (for /api/health/bridge) ────────────────────────────────────────
    def get_status(self) -> dict[str, Any]:
        now = time.monotonic()
        frozen_for_ms = (
            (now - self._degraded_since) * 1000.0
            if self._degraded_since is not None
            else 0.0
        )
        return {
            "health": self._health,
            "auto_recover": self._auto_recover,
            "frozen_for_ms": round(frozen_for_ms, 1),
            "recovery_count": self._recovery_count,
            "last_recovery_ts": self._last_recovery_ts,
            "last_recovery_reason": self._last_recovery_reason,
            "spray_recovery_required": self._spray_recovery_required,
            "spray_recovery_reason": self._spray_recovery_reason,
            **self._last_snapshot,
        }

    # ── detection loop ─────────────────────────────────────────────────────────
    async def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                await asyncio.sleep(BRIDGE_HEALTH_POLL_S)
                await self._tick()
            except asyncio.CancelledError:
                break
            except Exception:
                log.exception("bridge-health tick failed")

    async def _tick(self) -> None:
        if self._ros is None:
            return
        snap = self._ros.get_bridge_snapshot()
        self._last_snapshot = snap
        now = time.monotonic()

        # Suppress detection while MAVROS is coming back after a recovery.
        if now < self._cooldown_until:
            return

        frozen = self._is_frozen(snap)

        if not frozen:
            if self._health != HEALTHY:
                await self._transition(HEALTHY, "bridge link healthy")
            self._degraded_since = None
            self._observe_alerted = False
            return

        # Frozen.
        if self._degraded_since is None:
            self._degraded_since = now
            await self._transition(DEGRADED, self._reason(snap))

        frozen_for = now - self._degraded_since
        if frozen_for < BRIDGE_FROZEN_GRACE_S or self._health in (RECOVERING, FAILED):
            return

        # Frozen long enough to act.
        if not self._auto_recover:
            if not self._observe_alerted:
                self._observe_alerted = True
                msg = (
                    f"bridge frozen {frozen_for:.1f}s ({self._reason(snap)}); "
                    "auto-recovery disabled (observe-only)"
                )
                self._record("warning", msg)
                await self._safe_emit("bridge_health", {**self.get_status(), "note": msg})
            return

        await self._recover(self._reason(snap))

    def _is_frozen(self, snap: dict[str, Any]) -> bool:
        if not snap.get("fcu_connected", False):
            return True
        age = snap.get("state_age_ms")
        if age is None or age > BRIDGE_STATE_STALE_MS:
            return True
        pubs = snap.get("mavros_state_publishers")
        if pubs == 0:  # -1 means "graph query failed", not a real zero
            return True
        return False

    def _reason(self, snap: dict[str, Any]) -> str:
        bits = []
        if not snap.get("fcu_connected", False):
            bits.append("fcu_disconnected")
        age = snap.get("state_age_ms")
        if age is None:
            bits.append("no_state_msg")
        elif age > BRIDGE_STATE_STALE_MS:
            bits.append(f"state_age={age:.0f}ms")
        if snap.get("mavros_state_publishers") == 0:
            bits.append("no_state_publisher")
        return ",".join(bits) or "frozen"

    # ── transitions / emit ─────────────────────────────────────────────────────
    async def _transition(self, new_health: str, reason: str) -> None:
        old = self._health
        self._health = new_health
        level = "info" if new_health == HEALTHY else "warning"
        self._record(level, f"bridge health: {old} → {new_health} ({reason})")
        await self._safe_emit("bridge_health", self.get_status())

    async def _safe_emit(self, event: str, payload: dict) -> None:
        try:
            await self._emit(event, payload)
        except Exception:
            log.exception("bridge-health emit failed: %s", event)

    # ── recovery (Phase 3B — gated by auto_recover) ────────────────────────────
    def _within_backoff(self, now: float) -> bool:
        cutoff = now - BRIDGE_RECOVERY_WINDOW_S
        while self._recovery_times and self._recovery_times[0] < cutoff:
            self._recovery_times.popleft()
        return len(self._recovery_times) < BRIDGE_RECOVERY_MAX

    async def _recover(self, reason: str) -> None:
        now = time.monotonic()
        if not self._within_backoff(now):
            await self._transition(
                FAILED,
                f"recovery budget exhausted ({BRIDGE_RECOVERY_MAX} in "
                f"{BRIDGE_RECOVERY_WINDOW_S:.0f}s) — operator action required",
            )
            return

        self._recovery_times.append(now)
        self._recovery_count += 1
        self._last_recovery_ts = time.time()
        self._last_recovery_reason = reason
        await self._transition(RECOVERING, f"restarting px4-dxp ({reason})")

        # Mid-mission: confirm spray OFF, then best-effort soft stop. Bridge may
        # be frozen so either step can fail — PX4 OFFBOARD failsafe is the net.
        self._spray_recovery_required = False
        self._spray_recovery_reason = None
        if self._ctrl is not None and self._ctrl.state == MissionState.RUNNING:
            if self._ros is not None:
                try:
                    spray_off = await force_spray_off_confirmed(
                        self._ros, timeout_s=2.0
                    )
                    if spray_off_blocks_success(spray_off):
                        self._spray_recovery_required = True
                        self._spray_recovery_reason = spray_off.message
                        self._record(
                            "warning",
                            f"bridge recovery: {spray_off.message}",
                        )
                except Exception:
                    self._spray_recovery_required = True
                    self._spray_recovery_reason = "spray OFF confirmation failed"
                    log.exception("bridge recovery: force spray OFF failed")
            try:
                await asyncio.wait_for(self._ctrl.stop_async(), timeout=3.0)
            except Exception:
                log.warning("bridge recovery: best-effort stop_async failed/timed out")

        rc = await self._restart_px4dxp()
        await self._safe_emit(
            "bridge_recovery",
            {
                "reason": reason,
                "recovery_count": self._recovery_count,
                "restart_rc": rc,
                "ts": self._last_recovery_ts,
            },
        )
        # Give MAVROS time to come back before re-evaluating.
        self._cooldown_until = time.monotonic() + BRIDGE_RECOVERY_COOLDOWN_S
        self._degraded_since = None

    async def _restart_px4dxp(self) -> Optional[int]:
        try:
            proc = await asyncio.create_subprocess_exec(
                "sudo", "-n", "systemctl", "restart", "px4-dxp.service",
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.PIPE,
            )
            _, stderr = await asyncio.wait_for(proc.communicate(), timeout=40.0)
            if proc.returncode != 0:
                self._record(
                    "error",
                    f"bridge recovery: systemctl restart px4-dxp rc={proc.returncode} "
                    f"{(stderr or b'').decode(errors='replace').strip()}",
                )
            return proc.returncode
        except Exception as exc:
            self._record("error", f"bridge recovery: restart failed: {exc}")
            return None
