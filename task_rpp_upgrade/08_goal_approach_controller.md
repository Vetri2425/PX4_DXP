# 08 — Goal-approach controller (last 30 cm)

**Agent:** Haiku 4.5
**Estimated diff:** +80 lines
**Depends on:** 03 (spline projection)
**Blocks:** —

## Goal

Pure Pursuit is notorious for overshooting the final waypoint: the
lookahead point doesn't exist past the goal, so the controller falls back
to "drive toward the last point" with no decel discipline. Add a dedicated
state that activates inside a small radius of the goal and runs a position
PI controller instead.

## State machine extension

Existing states: STALE, IDLE, TRACKING, APPROACH, DONE.

The existing APPROACH state is a stub. Make it real:

- Entry condition: `dist_to_goal ≤ approach_radius_m` (default 0.30).
- Exit conditions:
  - `dist_to_goal ≤ goal_radius_m` AND `|v_measured| < settle_speed` →
    DONE.
  - Timeout: 8 s in APPROACH without reaching goal → DONE with warning
    (avoids infinite settle).

In APPROACH:

```
e_vec = (goal_n - pos_n, goal_e - pos_e)
v_cmd_mag = clamp(k_p · |e_vec|, v_min_approach, v_max_approach)
v_cmd = v_cmd_mag · unit(e_vec)
ψ_cmd = atan2(e_e, e_n)         # NED yaw toward goal
```

No Pure Pursuit, no lookahead — just go straight at the goal with a P-on-
distance speed.

`k_p` ≈ 1.5 → 30 cm error gives 0.45 m/s, 10 cm gives 0.15 m/s, 2 cm
gives `v_min_approach` = 0.03 m/s.

For pose stability at finish: zero velocity for 0.5 s before declaring
DONE. This prevents the next setpoint cycle from re-arming motion through
hysteresis.

## Files to read first

- `src/rpp_controller_node.py` — state machine, current APPROACH stub.

## Scope

### A. Parameters

- `rpp_enable_goal_approach` (bool, default False).
- `rpp_approach_radius_m` (float, default 0.30). Entry trigger.
- `rpp_goal_radius_m` (float, default 0.03). Done trigger.
- `rpp_approach_kp` (float, default 1.5).
- `rpp_v_min_approach` (float, default 0.03).
- `rpp_v_max_approach` (float, default 0.20).
- `rpp_approach_settle_s` (float, default 0.5).
- `rpp_approach_timeout_s` (float, default 8.0).

### B. Wiring

Add APPROACH-state branch to the control loop. The existing TRACKING
branch stays in charge until `dist_to_goal` crosses `approach_radius_m`.

DONE behaviour stays the same (publishes zero velocity).

### C. Tests

`tests/test_goal_approach.py`:

- Mock pose at goal_n - 0.20, goal_e + 0.0 → v_cmd = 1.5·0.20 = 0.30,
  pointing toward goal.
- Mock pose at goal_n - 0.02 → v_cmd = max(v_min, 1.5·0.02) = 0.03.
- Settle: pose at goal for 0.5 s consecutive → state becomes DONE.

## Out of scope

- Reverse-approach (rover overshoots, then reverses to goal). The
  differential-drive rover can pivot, so we don't need it.
- Heading-at-goal control (orient to a specific direction at the end).
  Future task.

## Acceptance criteria

- [ ] On any mission ending in a fixed waypoint, endpoint distance error
      < 3 cm consistently.
- [ ] No oscillation around the goal (settle hysteresis works).
- [ ] No regression on long missions (entry condition correctly fires
      only near the end).
- [ ] Timeout fires DONE if rover physically can't reach (e.g. obstacle)
      instead of hanging in APPROACH forever.

## Notes for the agent

- Watch out for the existing P2.4 pose-extrapolation — when pose freezes
  near goal, the extrapolated pose can phantom-pass the goal. Disable
  extrapolation inside APPROACH (extrapolation horizon → 0) to avoid
  false DONE.
- The 0.5 s settle window is intentional: shorter values cause flaky
  DONE due to RTK noise.
