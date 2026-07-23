#!/usr/bin/env python3
"""Spray mode engines — pure, no rclpy (plan §7, §11).

Phase C lands the Dash engine here. Point (§7.3) will follow in Phase D.

The one rule for this file: **no ROS import.** Everything here is a plain
Python object that takes numbers in and returns numbers out, so the mode
logic is unit-testable off-robot (`test_spray_dash_v2.py` runs on the Mac,
which has no rclpy). The node (`spray_controller_node.py`) owns the ROS
glue and feeds these engines per tick.
"""

from __future__ import annotations

from dataclasses import dataclass


# Small epsilon for arc-length float comparisons (metres). Well below any
# real toggle distance or per-tick travel, so it only absorbs formatting noise.
_EPS = 1e-9


@dataclass(frozen=True)
class DashUpdate:
    """Result of one DashMeter tick — all read-only, for status/telemetry."""

    geometry_desired: bool  # would the pattern spray here (before safety gates)?
    armed: bool             # has metering started (pose on the path proper)?
    phase: str              # "on" | "off"
    s_dash: float           # monotonic arc-length the meter is tracking (m)
    s_at_last_toggle: float # arc-length of the last ideal toggle boundary (m)
    jump_rejected: bool     # this tick's raw projection.s was rejected as a glitch
    recovered: bool         # this tick a persistent relocation was accepted


class DashMeter:
    """Interval/dash metering by cumulative mission arc-length (plan §7.2).

    Locked decision #2 (operator-confirmed 2026-07-23, "continuous across
    mission"): the on/off pattern is metered against cumulative path
    arc-length and **does not reset** at entity or corner boundaries — a
    6-on/3-off pattern flows straight through a corner. Per-segment reset is
    explicitly NOT built here (see plan §7.2's open decision, now closed to
    continuous-across-mission).

    Corner handling is *not* this class's job: while the rover pivots at a
    corner the node's pivot-state gate (plan §5) already forces spray OFF, so
    the actuator command is deferred for free. This meter keeps integrating
    arc-length through the stop, so the phase still flips at the correct `s`
    and the pattern does not drift — exactly the §7.2 requirement, achieved by
    the existing gate rather than a special case here.

    Robustness (plan §7.2): `_project_onto_path` returns the *nearest*-segment
    arc-length, so on a retrace / self-overlapping path raw `s` can jump
    backward or lurch forward between ticks, which would double-fire or skip
    toggles. So this meter never consumes raw `s` directly:
      * it clamps to non-decreasing (monotonic `s_dash`), and
      * rejects any single-tick step larger than `speed*dt*jump_tolerance_factor`
        as a projection glitch, holding `s_dash` frozen, but
      * recovers: if `jump_reject_accept_after` consecutive rejects agree with
        each other (a stable new position, not noise) it accepts the relocation
        so the guard can never wedge permanently on legitimate travel.
    """

    def __init__(
        self,
        on_distance_m: float,
        off_distance_m: float,
        start_state: str = "on",
        *,
        jump_tolerance_factor: float = 3.0,
        jump_reject_accept_after: int = 5,
    ) -> None:
        on_d = float(on_distance_m)
        off_d = float(off_distance_m)
        if not (on_d > 0.0) or not (off_d > 0.0):
            raise ValueError(
                f"dash on/off distances must be > 0, got on={on_d}, off={off_d}"
            )
        if start_state not in ("on", "off"):
            raise ValueError(f"dash start_state must be 'on'|'off', got {start_state!r}")
        self.on_distance_m = on_d
        self.off_distance_m = off_d
        self.phase = start_state
        self.jump_tolerance_factor = max(1.0, float(jump_tolerance_factor))
        self.jump_reject_accept_after = max(1, int(jump_reject_accept_after))

        self.armed = False
        self.s_dash = 0.0
        self.s_at_last_toggle = 0.0
        self._reject_count = 0
        self._reject_ref: float | None = None

    def _target(self) -> float:
        return self.on_distance_m if self.phase == "on" else self.off_distance_m

    def update(
        self,
        raw_s: float,
        speed_mps: float,
        dt_s: float,
        xtrack_ok: bool,
        on_lead_m: float = 0.0,
        off_lead_m: float = 0.0,
    ) -> DashUpdate:
        """Advance the meter one control tick and return the desired phase.

        Args:
            raw_s: `projection.s` this tick (nearest-segment arc-length, m).
            speed_mps: rover ground speed (m/s), for the jump-tolerance window.
            dt_s: measured seconds since the last tick (monotonic-clock delta).
            xtrack_ok: cross-track within tolerance — the "on the path proper"
                arming trigger (plan §7.2). Metering stays disarmed (and
                geometry_desired False) until the first tick this is True, so
                the entry transit never meters.
            on_lead_m / off_lead_m: solenoid-lead distances (speed*delay + margin),
                same values continuous mode uses — the command flips this far
                before the ideal boundary so the physical paint lands on grid.
        """
        raw_s = float(raw_s)
        speed_mps = max(0.0, float(speed_mps))
        dt_s = max(0.0, float(dt_s))
        jump_rejected = False
        recovered = False

        if not self.armed:
            if not xtrack_ok:
                return DashUpdate(
                    False, False, self.phase, self.s_dash,
                    self.s_at_last_toggle, False, False,
                )
            # Arm at the first on-path tick: anchor the pattern here.
            self.armed = True
            self.s_dash = raw_s
            self.s_at_last_toggle = raw_s
            self._reject_count = 0
            self._reject_ref = None
        else:
            delta = raw_s - self.s_dash
            max_step = speed_mps * dt_s * self.jump_tolerance_factor
            if -_EPS <= delta <= max_step + _EPS:
                # Real forward travel (monotonic): accept.
                self.s_dash = raw_s
                self._reject_count = 0
                self._reject_ref = None
            else:
                # Backward or too-large step: treat as a projection glitch and
                # hold s_dash, unless it is a persistent, self-consistent move.
                jump_rejected = True
                if (
                    self._reject_ref is not None
                    and abs(raw_s - self._reject_ref) <= max_step + _EPS
                ):
                    self._reject_count += 1
                else:
                    self._reject_ref = raw_s
                    self._reject_count = 1
                if self._reject_count >= self.jump_reject_accept_after:
                    # Legitimate relocation (retrace snapped to a later pass, a
                    # resumed run): accept and re-anchor the pattern here rather
                    # than letting the toggle loop below burst-flip to catch up.
                    self.s_dash = raw_s
                    self.s_at_last_toggle = raw_s
                    self._reject_count = 0
                    self._reject_ref = None
                    jump_rejected = False
                    recovered = True

        # Toggle math on the ideal grid: each phase spans exactly `_target()`
        # metres of arc-length, so the pattern never drifts. `s_at_last_toggle`
        # advances by the full target (not to s_dash), and the command flips
        # `lead` metres early so the physical paint toggles on the ideal grid.
        # The loop covers the (practically impossible at ~50 Hz) case of more
        # than one boundary crossed in a single tick.
        guard = 0
        while self.armed:
            target = self._target()
            next_toggle = self.s_at_last_toggle + target
            # Lead for the UPCOMING flip: on->off uses the close (off) lead,
            # off->on uses the open (on) lead.
            lead = off_lead_m if self.phase == "on" else on_lead_m
            lead = max(0.0, min(float(lead), target))  # never past a full segment
            if self.s_dash >= next_toggle - lead - _EPS:
                self.phase = "off" if self.phase == "on" else "on"
                self.s_at_last_toggle = next_toggle
                guard += 1
                if guard > 10_000:  # pathological safety valve; unreachable in practice
                    break
            else:
                break

        return DashUpdate(
            geometry_desired=(self.armed and self.phase == "on"),
            armed=self.armed,
            phase=self.phase,
            s_dash=self.s_dash,
            s_at_last_toggle=self.s_at_last_toggle,
            jump_rejected=jump_rejected,
            recovered=recovered,
        )

    def mode_state(self) -> dict:
        """Small JSON-safe snapshot for SpraySessionStatus.mode_state (§6)."""
        return {
            "phase": self.phase,
            "armed": self.armed,
            "s_dash_m": round(self.s_dash, 4),
            "s_at_last_toggle_m": round(self.s_at_last_toggle, 4),
            "on_distance_m": self.on_distance_m,
            "off_distance_m": self.off_distance_m,
        }
