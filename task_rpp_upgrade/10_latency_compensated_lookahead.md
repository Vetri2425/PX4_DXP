# 10 — Latency-compensated lookahead

**Agent:** Gemma 3 1B
**Estimated diff:** +20 lines
**Depends on:** 03 (spline arc length)
**Blocks:** —

## Goal

The full pipeline introduces ~80-120 ms of round-trip latency: pose age
(typically 30-50 ms via MAVROS) + the 50 Hz control loop period (20 ms) +
PX4 setpoint consumption (one tick at 50 Hz, 20 ms) + RoboClaw bus update
(~10 ms).

By the time our commanded velocity takes effect, the rover has already
moved. Shift the lookahead arc length forward by an estimate of this
delay times current speed to compensate.

## Math

```
τ_total = τ_pose + τ_control_loop + τ_actuator    # default 0.10 s
s_la_compensated = s_la + τ_total · v_cmd
```

That's it. Apply before the lookahead point lookup.

## Files to read first

- `src/rpp_controller_node.py` — lookahead block after spline integration
  from task 03.
- The existing P2.4 pose extrapolation — note the τ value used there.

## Scope

### A. Parameter

`rpp_lookahead_latency_s` (float, default 0.10). Range [0, 0.30].

### B. Wiring

After `s_la = self._last_s + L_d` is computed (post-task-03), add:

```
s_la_eff = s_la + self.get_parameter("rpp_lookahead_latency_s").value * v_cmd
```

Use `s_la_eff` for the spline lookup. Clamp to `[0, self._spline.length_m]`.

### C. Sanity check

If `τ_total · v_cmd > L_d`, the compensated lookahead would overtake the
nominal lookahead — log a warning (throttled) and clamp `s_la_eff = s_la`.

### D. Tests

`tests/test_latency_lookahead.py`:

- v_cmd = 0.4, τ = 0.1 → offset = 4 cm. Lookahead point shifts forward 4
  cm along path.
- v_cmd = 0 → offset = 0. Same as nominal.
- Beyond path end → clamped to path length.

## Out of scope

- Online latency estimation (we measure offline and set the param once).
- Different latency for different operations (one global value is fine).

## Acceptance criteria

- [ ] On 2 m square with compensation on, mean xtrack on straights drops
      by 0.5-1 cm.
- [ ] No new oscillation or instability.
- [ ] No regression with τ = 0.

## Notes for the agent

- Don't compensate by *also* increasing `L_d` — the existing P0.1 already
  scales `L_d` by velocity. Stacking would over-correct.
- If task 03's spline isn't merged yet, this task is blocked: arc-length
  lookahead is required.
