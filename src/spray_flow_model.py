#!/usr/bin/env python3
"""Speed-proportional flow modulator (plan §7.5). Pure, no rclpy.

Problem: the actuator only ever commanded full-on or off, so paint density is
proportional to time-over-a-point ∝ 1/speed — a speed-up thins the line toward
gaps, a slow-down pools it. Fix: scale the pump command with ground speed so
paint-per-metre stays constant, exactly how precision-ag PWM rate controllers
hold application rate constant across a speed range.

This is decoupled from the safety FSM (plan §7.5): the FSM still answers only
"is spraying allowed" (ON_CONFIRMED / OFF / …); this modulator answers, layered
on top and active only while ON, "how much flow right now". It can only ever
move the command WITHIN [min_flow_value, on_value] — it can never keep spray on
longer, turn it on earlier, or override a gate.

Our actuator (mavlink_actuator, normalized) already accepts a continuously
variable command, so unlike classic ag PWM there is nothing to pulse-simulate:
we command a direct value between a calibrated floor and on_value.

    frac      = clamp(speed / rated_marking_speed, 0, 1)      # ↑ at higher speed
    target    = min_flow_value + (on_value - min_flow_value) * frac
    commanded = slew_limit(target, prev, max_slew_per_s * dt)  # never step the pump

`rated_marking_speed_mps` must be set at the TOP of the operating speed range:
the clamp caps flow at on_value once speed ≥ rated, so above rated the line
thins — setting it mid-range guarantees under-paint on every faster stretch.
"""

from __future__ import annotations


class FlowModulator:
    """Speed→flow with a slew limiter. Values are in the actuator's own units
    (normalized for mavlink_actuator: off_value..on_value)."""

    def __init__(
        self,
        min_flow_value: float,
        on_value: float,
        rated_marking_speed_mps: float,
        max_slew_per_s: float,
    ) -> None:
        self.min_flow_value = float(min_flow_value)
        self.on_value = float(on_value)
        # Guard a zero/negative rated speed → treat as "always full flow".
        self.rated_marking_speed_mps = max(1e-6, float(rated_marking_speed_mps))
        self.max_slew_per_s = max(0.0, float(max_slew_per_s))
        # Bounds are ordered so on_value < min_flow_value (unusual) still clamps.
        self._lo = min(self.min_flow_value, self.on_value)
        self._hi = max(self.min_flow_value, self.on_value)
        # Slew filter's previous value. Seeded to the floor so flow always
        # ramps UP from the safe minimum on the OFF→ON edge, never steps or
        # resumes a stale value (plan §7.5 slew-init).
        self._last = self.min_flow_value

    def reset(self) -> None:
        """Call on the OFF→ON edge: re-seed the slew filter to the floor."""
        self._last = self.min_flow_value

    def update(self, speed_mps: float, dt_s: float) -> float:
        """Advance one tick; return the commanded flow value.

        `dt_s` is the measured (monotonic) tick interval, not a nominal
        constant, so a slow/variable tick can't let the limiter over- or
        under-shoot.
        """
        frac = max(0.0, float(speed_mps)) / self.rated_marking_speed_mps
        frac = min(1.0, frac)  # saturate at rated speed → full flow
        target = self.min_flow_value + (self.on_value - self.min_flow_value) * frac
        target = max(self._lo, min(self._hi, target))

        dt_s = max(0.0, float(dt_s))
        if self.max_slew_per_s > 0.0 and dt_s > 0.0:
            max_step = self.max_slew_per_s * dt_s
            delta = target - self._last
            if delta > max_step:
                delta = max_step
            elif delta < -max_step:
                delta = -max_step
            self._last += delta
        else:
            self._last = target
        return self._last

    @property
    def value(self) -> float:
        return self._last
