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

import math
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


@dataclass(frozen=True)
class PointUpdate:
    """Result of one PointMeter tick."""

    geometry_desired: bool  # spray at the dot (before safety gates)?
    phase: str              # transit|arriving|holding|dwelling|done
    target_index: int       # coordinate the rover is heading to / at
    exempt_pivot: bool      # bypass the pivot gate this tick (dwelling on a dot)
    done: bool              # whole point mission finished
    skipped_index: int      # index skipped by the watchdog this tick, else -1


class PointMeter:
    """Point / coordinate dwell-marking FSM (plan §7.3).

    Per point i: TRANSIT (rover navigating, spray off) → ARRIVING (near the
    dot, near-zero speed, optional heading, held for arrival_settle_s — a blip
    resets, no partial credit) → HOLDING/DWELLING (spray ON for dwell_s) →
    wait for a CONFIRMED actuator OFF → ADVANCE to the next dot.

    Two-sidedness (plan §7.3): this class only *watches* pose and decides when
    to spray. Something else (the planner + the driving controller) must
    actually stop the rover on each dot; without that the rover drives through,
    ARRIVING never satisfies, and every point is watchdog-skipped. That
    planner-side per-point hold is the OTHER half of Phase D.

    Safety-critical rules baked in here:
      * ADVANCE waits for `off_confirmed`, not merely for dwell_s — so a
        retrying OFF can never let the mission move off a still-spraying dot.
      * The unreachable-target watchdog skips a point after
        point_arrival_timeout_s, logs it (skipped_index), and NEVER counts it
        as sprayed — one bad coordinate can't wedge the whole mission.
      * `exempt_pivot` is True ONLY while dwelling within tolerance of the
        active dot — the narrow, node-side pivot-gate exemption (Rev 4 §7.3),
        never mode-wide (transits between dots keep the gate).
    """

    def __init__(
        self,
        coordinates,
        arrival_tolerance_m: float,
        arrival_settle_s: float,
        dwell_s: float,
        heading_tolerance_deg=None,
        *,
        point_arrival_max_speed_mps: float = 0.05,
        point_arrival_timeout_s: float = 60.0,
    ) -> None:
        self.coordinates = tuple((float(n), float(e)) for n, e in coordinates)
        if arrival_tolerance_m <= 0.0:
            raise ValueError(f"arrival_tolerance_m must be > 0, got {arrival_tolerance_m}")
        self.arrival_tolerance_m = float(arrival_tolerance_m)
        self.arrival_settle_s = max(0.0, float(arrival_settle_s))
        self.dwell_s = max(0.0, float(dwell_s))
        self.heading_tolerance_deg = (
            None if heading_tolerance_deg is None else float(heading_tolerance_deg)
        )
        self.max_speed_mps = max(0.0, float(point_arrival_max_speed_mps))
        self.timeout_s = max(0.0, float(point_arrival_timeout_s))

        self.target_index = 0
        self.phase = "done" if not self.coordinates else "transit"
        self.done = not self.coordinates
        self.skipped_indices: list[int] = []
        self._settle_start = None
        self._dwell_start = None
        self._transit_start = None

    def _heading_ok(self, yaw_ned: float, i: int) -> bool:
        """Optional heading gate: aligned with the approach bearing (prev→i).

        Position-only (heading_tolerance_deg=None) always passes. The first
        point has no approach bearing, so it is position-only too. CSV point
        lists carry no per-point heading, so None is the common case.
        """
        if self.heading_tolerance_deg is None or i == 0:
            return True
        pn, pe = self.coordinates[i - 1]
        tn, te = self.coordinates[i]
        bearing_ned = math.atan2(te - pe, tn - pn)  # NED: atan2(dE, dN)
        err = (yaw_ned - bearing_ned + math.pi) % (2.0 * math.pi) - math.pi
        return abs(math.degrees(err)) <= self.heading_tolerance_deg

    def _advance(self) -> None:
        self.target_index += 1
        self._settle_start = None
        self._dwell_start = None
        self._transit_start = None
        if self.target_index >= len(self.coordinates):
            self.phase = "done"
            self.done = True
        else:
            self.phase = "transit"

    def update(
        self,
        pose_n: float,
        pose_e: float,
        yaw_ned: float,
        speed_mps: float,
        now_s: float,
        off_confirmed: bool,
    ) -> PointUpdate:
        """Advance the FSM one control tick.

        pose_n/pose_e are the NOZZLE position (same frame continuous/dash
        project onto) so the dot lands under the nozzle. `off_confirmed` is the
        actuator FSM's OFF_CONFIRMED state; `now_s` is a monotonic clock.
        """
        if self.done:
            return PointUpdate(False, "done", self.target_index, False, True, -1)

        skipped_index = -1
        # Process instantaneous transitions in one tick (holding→dwelling,
        # advance→transit) with a bounded loop; the guard is far above the
        # number of real transitions possible per tick.
        for _ in range(8):
            if self.done:
                break
            i = self.target_index
            tn, te = self.coordinates[i]
            dist = math.hypot(pose_n - tn, pose_e - te)
            slow = speed_mps <= self.max_speed_mps
            arrived = dist <= self.arrival_tolerance_m and slow and self._heading_ok(yaw_ned, i)

            if self._transit_start is None:
                self._transit_start = now_s

            if self.phase == "transit":
                if arrived:
                    self.phase = "arriving"
                    self._settle_start = now_s
                    continue
                if now_s - self._transit_start > self.timeout_s:
                    skipped_index = i
                    self.skipped_indices.append(i)
                    self._advance()
                    continue
                break  # still transiting, nothing more this tick

            if self.phase == "arriving":
                if arrived:
                    if self._settle_start is None:
                        self._settle_start = now_s
                    if now_s - self._settle_start >= self.arrival_settle_s:
                        self.phase = "holding"
                        continue
                    break  # settling
                # Lost the arrival condition before settle completed — no
                # partial credit; reset the settle timer.
                self._settle_start = None
                if now_s - self._transit_start > self.timeout_s:
                    skipped_index = i
                    self.skipped_indices.append(i)
                    self._advance()
                    continue
                break

            if self.phase == "holding":
                # Entry to the dwell: spray ON, start the dwell clock.
                self._dwell_start = now_s
                self.phase = "dwelling"
                continue

            if self.phase == "dwelling":
                if now_s - self._dwell_start < self.dwell_s:
                    break  # keep spraying
                # Dwell complete: stop desiring spray NOW (→ off_wait), and only
                # advance once the actuator OFF is confirmed — never leave a
                # spraying dot even if the OFF command is still retrying.
                self.phase = "off_wait"
                continue

            if self.phase == "off_wait":
                if off_confirmed:
                    self._advance()
                    continue
                break  # dwell done, spray desired-off, waiting for OFF ack

            break  # unknown phase safety

        geometry_desired = self.phase in ("holding", "dwelling")
        exempt_pivot = geometry_desired and (
            not self.done
            and math.hypot(
                pose_n - self.coordinates[self.target_index][0],
                pose_e - self.coordinates[self.target_index][1],
            ) <= self.arrival_tolerance_m
        )
        return PointUpdate(
            geometry_desired=geometry_desired,
            phase=self.phase,
            target_index=self.target_index,
            exempt_pivot=exempt_pivot,
            done=self.done,
            skipped_index=skipped_index,
        )

    def mode_state(self) -> dict:
        return {
            "phase": self.phase,
            "target_index": self.target_index,
            "num_points": len(self.coordinates),
            "skipped": list(self.skipped_indices),
            "done": self.done,
        }
