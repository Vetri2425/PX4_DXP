"""Pure, ROS-free spray actuator safety state machine (Spray Controller V2, Phase A).

Implements docs/Architecture/SPRAY_CONTROLLER_V2_PLAN.md §4 verbatim as an
explicit `enum.Enum` state machine + transition table, replacing the
scattered-boolean tracking (`_commanded`, `_off_confirmed`, `_cmd_seq`,
`_maybe_retry_off`, `_command_done`) in `spray_controller_node.py`.

HARD CONSTRAINTS (do not violate when editing this file):
  - No `rclpy` / ROS imports of any kind.
  - No I/O.
  - No wall-clock calls (`time.time()` / `time.monotonic()`). All timing is
    caller-injected via `now: float` (monotonic seconds) so this module is
    unit-testable on a plain Python 3 install with zero ROS present.

Two invariants enforced by construction (see plan §4):
  INVARIANT 1 — `spraying` is True only when `state == ON_CONFIRMED`.
    Dispatching an ON command never flips `spraying`; only a successful
    `on_ack` for the matching `cmd_seq` does. There is no optimistic-ON
    window anywhere in this module.
  INVARIANT 2 — every dispatched `SprayCommand` carries the state machine's
    monotonic `cmd_seq`. `on_ack(seq, ...)` applies a reply only if
    `seq == cmd_seq`; a stale/superseded reply is a no-op (`None`, no state
    change).

DISABLED modeling (plan §4's "spray_disabled -> DISABLED, then OFF_PENDING",
clarified in the Phase A brief): disabling latches an internal `_disabled`
flag and forces an OFF dispatch (or, if the actuator's OFF is *already*
confirmed at the moment of disable, promotes directly to DISABLED with no
redundant redispatch — nothing left to confirm). The terminal `DISABLED`
enum state is entered only once that OFF is confirmed by an ack. ON is
refused for the entire time `_disabled` is set — checked at the top of
`tick()`, before any ON dispatch is even considered, not as a periodic
sweep. Re-enabling (`enabled` False->True) always re-arms through
`OFF_UNCONFIRMED`, never straight to `OFF_CONFIRMED`, so a fresh OFF must be
re-confirmed before any ON can ever be accepted again.

Safety-loss handling: a `safety_ok=False` tick forces OFF immediately on
the True->False *edge* (reached with state `ON_*`, so `_drive_off_safety`
dispatches a forced OFF), keeps driving toward OFF while the off-state is
still unconfirmed, honors the RECOVERY backoff if an OFF ack fails, and
then falls SILENT once OFF is confirmed (`OFF_CONFIRMED`/`DISABLED`). It
deliberately does NOT re-dispatch a forced OFF on every sustained-unsafe
tick: at the ~50 Hz watchdog rate that would flood `/mavros/cmd/command`
for as long as the rover sits DISARMED (its normal idle state). This
mirrors both the legacy node (which stopped commanding once OFF was
confirmed) and the `_drive_off_disabled` path. The plan's transition table
lists "any state -> OFF (force=True)" for safety-loss as an *edge*
trigger, not a per-tick level command; the fail-safe guarantee (spray is
driven OFF and confirmed on loss of safety) is fully preserved.

RECOVERY modeling: `RECOVERY` is entered only from an `OFF_PENDING` ack
failure/timeout (never from an ON-side failure — a failed ON always routes
to a fresh OFF dispatch first, per the plan). While in `RECOVERY`, `tick()`
retries the OFF dispatch (returning to `OFF_PENDING`) once
`now >= recovery_deadline`, where the deadline is set on entry to
`min(backoff_base_s * 2**attempt, backoff_max_s)` seconds out and `attempt`
increments on every subsequent recovery entry. `attempt` (and therefore the
backoff) resets to 0 on any safety-loss edge (`safety_ok` True->False),
giving a fast `backoff_base_s` first retry right when it matters most, and
can also be reset explicitly by the caller via `note_event_reset(now)` on a
config/mode-change signal (the plan leaves the exact mechanism to the
implementer — this module exposes it as an explicit public method rather
than an implicit side channel, so the node can call it from wherever it
detects a mission/config change).
"""

from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Optional


class SprayState(enum.Enum):
    OFF_UNCONFIRMED = "OFF_UNCONFIRMED"
    OFF_CONFIRMED = "OFF_CONFIRMED"
    ON_PENDING = "ON_PENDING"
    ON_CONFIRMED = "ON_CONFIRMED"
    OFF_PENDING = "OFF_PENDING"
    RECOVERY = "RECOVERY"
    DISABLED = "DISABLED"


@dataclass(frozen=True)
class SprayCommand:
    on: bool  # True -> dispatch ON to MAVROS; False -> dispatch OFF
    seq: int  # monotonic cmd_seq to stamp on the dispatch
    force: bool  # True -> bypass retry throttle/backoff (safety-loss / disable / shutdown)


class SpraySafetyStateMachine:
    """Pure actuator command state machine — see module docstring and
    docs/Architecture/SPRAY_CONTROLLER_V2_PLAN.md §4 for the full spec.
    """

    def __init__(
        self,
        *,
        backoff_base_s: float = 0.5,
        backoff_max_s: float = 5.0,
        ack_timeout_s: float = 1.0,
    ) -> None:
        self._backoff_base_s = float(backoff_base_s)
        self._backoff_max_s = float(backoff_max_s)
        # A dispatched ON/OFF whose MAVROS ack never arrives (dropped reply,
        # a service call that never resolves) must NOT wedge the FSM forever
        # in ON_PENDING/OFF_PENDING with no retry — the legacy node re-sent
        # OFF unconditionally every ~500 ms and was resilient to this. Treat a
        # pending command with no ack within ack_timeout_s as the plan §4
        # `ack(timeout)` failure. Default 1.0 s is well above a healthy
        # MAVROS ack (<100 ms), so it never false-fires in normal operation.
        self._ack_timeout_s = float(ack_timeout_s)

        self._state: SprayState = SprayState.OFF_UNCONFIRMED
        self._cmd_seq: int = 0
        # Monotonic timestamp of the last transition INTO a *_PENDING state;
        # None whenever the state is not pending. Drives the ack timeout.
        self._pending_since: Optional[float] = None

        # Latched by a disable edge; cleared on re-enable. Gates whether a
        # confirmed OFF should settle into the terminal DISABLED state and
        # whether ON dispatch is refused.
        self._disabled: bool = False

        # RECOVERY backoff bookkeeping.
        self._recovery_attempt: int = 0
        self._recovery_deadline: Optional[float] = None

        # Edge detectors for enabled/safety_ok, evaluated at the top of tick().
        self._prev_enabled: bool = True
        self._prev_safety_ok: bool = True

    # -- read-only public state -------------------------------------------------

    @property
    def state(self) -> SprayState:
        return self._state

    @property
    def spraying(self) -> bool:
        """True ONLY iff state == ON_CONFIRMED. INVARIANT 1."""
        return self._state == SprayState.ON_CONFIRMED

    @property
    def commanded(self) -> bool:
        """True iff state in {ON_PENDING, ON_CONFIRMED} (active ON intent)."""
        return self._state in (SprayState.ON_PENDING, SprayState.ON_CONFIRMED)

    @property
    def cmd_seq(self) -> int:
        return self._cmd_seq

    # -- events -------------------------------------------------------------

    def tick(
        self, *, desired: bool, safety_ok: bool, enabled: bool, now: float
    ) -> Optional[SprayCommand]:
        """Advance the FSM by one control-loop tick.

        Returns a SprayCommand to dispatch to MAVROS, or None if there is
        nothing new to send this tick (already in flight, already settled,
        or waiting on a RECOVERY backoff deadline).
        """
        # --- enabled edge detection (checked first: DISABLED refuses ON
        # unconditionally, before any ON dispatch is ever considered) ---
        edge_to_disabled = (not enabled) and self._prev_enabled
        edge_to_enabled = enabled and (not self._prev_enabled)
        self._prev_enabled = enabled

        if edge_to_disabled:
            self._disabled = True
            self._recovery_attempt = 0
        if edge_to_enabled:
            self._disabled = False
            self._state = SprayState.OFF_UNCONFIRMED
            self._recovery_attempt = 0
            self._recovery_deadline = None
            self._pending_since = None

        # --- safety edge detection (only meaningful while enabled; a disable
        # takes precedence and is handled by the not-enabled branch below) ---
        edge_safety_lost = enabled and (not safety_ok) and self._prev_safety_ok
        self._prev_safety_ok = safety_ok

        if edge_safety_lost:
            self._recovery_attempt = 0
            # If we lose safety while already backing off a failed OFF, make
            # the next retry immediate — the plan's "fast first retry right
            # when it matters most" rule for the safety-loss edge.
            if self._state == SprayState.RECOVERY:
                self._recovery_deadline = now

        # Ack-timeout check runs in EVERY branch (disabled/unsafe/normal): a
        # wedged pending state must recover regardless of current inputs.
        timed_out = self._check_pending_timeout(now)
        if timed_out is not None:
            return timed_out

        if not enabled:
            return self._drive_off_disabled(now)

        if not safety_ok:
            return self._drive_off_safety(now)

        return self._normal_tick(desired, now)

    def on_ack(self, seq: int, success: bool, now: float) -> Optional[SprayCommand]:
        """Apply a MAVROS command-ack reply. INVARIANT 2: ignored unless
        seq matches the current cmd_seq.
        """
        if seq != self._cmd_seq:
            return None

        if self._state == SprayState.ON_PENDING:
            if success:
                self._pending_since = None
                self._state = SprayState.ON_CONFIRMED
                return None
            # Never latch a failed ON as ON: drive straight to a fresh OFF
            # dispatch instead of passing through RECOVERY (RECOVERY is
            # OFF-ack-failure territory only, per the plan's transition table).
            return self._dispatch_off(force=False, now=now)

        if self._state == SprayState.OFF_PENDING:
            if success:
                self._pending_since = None
                self._recovery_attempt = 0
                self._recovery_deadline = None
                self._state = SprayState.DISABLED if self._disabled else SprayState.OFF_CONFIRMED
                return None
            self._enter_recovery(now)
            return None

        # Stale-but-same-seq duplicate/late ack for a state that already
        # moved on (e.g. disable promoted OFF_CONFIRMED -> DISABLED without
        # bumping cmd_seq): no-op, no state change.
        return None

    def note_event_reset(self, now: float) -> None:
        """Explicit reset hook for a config/mode-change signal (plan §4's
        backoff-reset rule: "reset to 0.5s the moment a config/mode change
        or safety-loss edge occurs"). Safety-loss edges reset automatically
        inside tick(); this method exists for the node to call from
        wherever it detects a mission/config change so that edge is not an
        implicit side channel.

        Resets the backoff attempt counter to 0 and, if currently in
        RECOVERY, makes the next tick's backoff check pass immediately
        (fast first retry) rather than waiting out the stale deadline.
        """
        self._recovery_attempt = 0
        if self._state == SprayState.RECOVERY:
            self._recovery_deadline = now

    # -- internal helpers -----------------------------------------------------

    def _normal_tick(self, desired: bool, now: float) -> Optional[SprayCommand]:
        state = self._state

        if state == SprayState.OFF_UNCONFIRMED:
            # Only action from OFF_UNCONFIRMED: dispatch OFF. ON is never
            # accepted until OFF_CONFIRMED is reached, regardless of desired.
            return self._dispatch_off(force=False, now=now)

        if state == SprayState.RECOVERY:
            if self._recovery_deadline is not None and now >= self._recovery_deadline:
                return self._dispatch_off(force=False, now=now)
            return None

        if state == SprayState.OFF_CONFIRMED:
            if desired:
                return self._dispatch_on(now)
            return None

        if state in (SprayState.ON_PENDING, SprayState.ON_CONFIRMED):
            if not desired:
                return self._dispatch_off(force=False, now=now)
            return None

        if state == SprayState.OFF_PENDING:
            return None  # awaiting ack already

        if state == SprayState.DISABLED:
            # Should not be reachable while enabled == True (re-enable edge
            # always routes through OFF_UNCONFIRMED first), but stay inert
            # defensively rather than assert.
            return None

        return None

    def _drive_off_disabled(self, now: float) -> Optional[SprayCommand]:
        if self._state == SprayState.OFF_CONFIRMED:
            # Already confirmed off; nothing left to confirm, promote
            # directly to the terminal disabled state.
            self._state = SprayState.DISABLED
            return None
        if self._state in (SprayState.DISABLED, SprayState.OFF_PENDING):
            return None
        return self._dispatch_off(force=True, now=now)

    def _drive_off_safety(self, now: float) -> Optional[SprayCommand]:
        state = self._state
        # Already off-and-confirmed (or terminally disabled): stay quiet.
        # A sustained unsafe condition — most commonly the rover simply
        # sitting DISARMED — must NOT re-dispatch a forced OFF every tick, or
        # it floods /mavros/cmd/command at the ~50 Hz watchdog rate. The
        # legacy node went quiet once OFF was confirmed; this mirrors that,
        # and matches _drive_off_disabled above. The safety-loss *edge* still
        # forces OFF immediately (this is reached with state ON_* on that
        # edge); we only fall silent after OFF has actually been confirmed.
        if state in (SprayState.OFF_CONFIRMED, SprayState.DISABLED):
            return None
        if state == SprayState.OFF_PENDING:
            return None  # already dispatching OFF, awaiting ack
        if state == SprayState.RECOVERY:
            # An OFF ack failed while unsafe: honor the backoff deadline
            # rather than hammering OFF every tick.
            if self._recovery_deadline is not None and now >= self._recovery_deadline:
                return self._dispatch_off(force=True, now=now)
            return None
        # ON_PENDING, ON_CONFIRMED, or OFF_UNCONFIRMED: something is (or may
        # be) on, or the off-state is unconfirmed — force OFF now.
        return self._dispatch_off(force=True, now=now)

    def _dispatch_off(self, *, force: bool, now: float) -> SprayCommand:
        self._cmd_seq += 1
        self._state = SprayState.OFF_PENDING
        self._pending_since = now
        return SprayCommand(on=False, seq=self._cmd_seq, force=force)

    def _dispatch_on(self, now: float) -> SprayCommand:
        self._cmd_seq += 1
        self._state = SprayState.ON_PENDING
        self._pending_since = now
        return SprayCommand(on=True, seq=self._cmd_seq, force=False)

    def _enter_recovery(self, now: float) -> None:
        backoff = min(self._backoff_base_s * (2 ** self._recovery_attempt), self._backoff_max_s)
        self._recovery_deadline = now + backoff
        self._recovery_attempt += 1
        self._state = SprayState.RECOVERY
        self._pending_since = None

    def _check_pending_timeout(self, now: float) -> Optional[SprayCommand]:
        """A pending ON/OFF whose ack never arrived: synthesize the plan §4
        `ack(timeout)` failure so the FSM cannot wedge on a lost reply.
        """
        if self._pending_since is None:
            return None
        if (now - self._pending_since) < self._ack_timeout_s:
            return None
        if self._state == SprayState.ON_PENDING:
            # Never latch an unconfirmed ON: drive to a fresh OFF (same as an
            # ON ack failure). A later stale ON ack is ignored via cmd_seq.
            return self._dispatch_off(force=False, now=now)
        if self._state == SprayState.OFF_PENDING:
            self._enter_recovery(now)  # clears _pending_since; retry on backoff
            return None
        # Not actually pending (defensive): clear the stamp and move on.
        self._pending_since = None
        return None
