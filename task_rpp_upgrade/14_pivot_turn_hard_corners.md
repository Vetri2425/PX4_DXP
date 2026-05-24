# 14 — Pivot-turn sub-state for HARD 90° corners

**Agent:** GLM (4.5 or 5.1)
**Estimated diff:** +120 lines (state machine extension + parameters)
**Depends on:** 03 (needs `PathGeometry.vertex_type()` and `next_hard_corner_s()`)
**Blocks:** —

## Goal

A marking rover's 90° corners must stay 90° — the painted line IS the
output. RPP's lookahead-based steering rounds corners; even with Stanley
blend (task 05), the rover overshoots corners by cutting wide.

The solution is NOT to smooth the path. It's to **stop and pivot** at
HARD corners: decelerate to the corner, spot-turn to align with the next
segment, then resume tracking. This is the same approach used by
agricultural robots (Naïo Oz) and line-marking machines.

## When pivot activates

Only at vertices classified as `HARD` by `PathGeometry.vertex_type()`
(turn angle ≥ `hard_corner_deg`, default 45°). SMOOTH and SOFT corners
are tracked normally by RPP.

## State machine extension

Add two states to the existing state machine:

```
states: STALE, IDLE, TRACKING, PIVOT_APPROACH, PIVOT_TURN, APPROACH, DONE
```

Transitions:

| From | To | Condition |
|---|---|---|
| TRACKING | PIVOT_APPROACH | `dist_to_next_hard_corner < approach_pivot_m` |
| PIVOT_APPROACH | PIVOT_TURN | `dist_to_vertex < pivot_entry_m` |
| PIVOT_APPROACH | TRACKING | hard corner disappears (path changed) |
| PIVOT_TURN | TRACKING | `|ψ_e| < settle_deg` AND `|ω_measured| < settle_rate_dps` |
| PIVOT_TURN | TRACKING | timeout (default 4 s) — safety cap |

### PIVOT_APPROACH

- Continue normal RPP tracking but cap `v_cmd ≤ v_pivot_approach_m_s`
  (default 0.10 m/s). Gradual decel before the corner.
- No change to steering — the rover is still tracking the path, just slow.

### PIVOT_TURN

- `v_cmd = 0` (full stop).
- `ψ_cmd = ψ_next_segment` (heading of the segment after the corner vertex).
- Publish yaw rate via `/rpp/yaw_rate_body` for crisper rotation than
  position-target alone.
- Disable pose extrapolation (P2.4) — extrapolating a stationary pose
  forward by Δt is pointless and risks false settle detection.
- Exit when heading error < `settle_deg` AND yaw rate < `settle_rate_dps`,
  OR after `timeout_s`.
- On exit: reset `s_proj` to the corner vertex arc-length, resume TRACKING.

## Design rationale

- **Why not track through HARD corners?** Even with small L_d, Pure Pursuit
  overshoots by L_d/2 at best. For a 90° turn at 0.4 m/s with L_d = 0.5 m,
  that's ~25 cm of overshoot. Pivot-turn gets within 2-5 cm.
- **Why 45° threshold?** Below 45° the rover can track through without
  stopping. Agricultural robots use similar thresholds (Naïo uses 60°).
- **Why not use ArduRover NAV_LOITER_TURNS?** We're on PX4 OFFBOARD. The
  pivot FSM runs entirely in the RPP node.
- **Why a separate task, not part of task 03?** The pivot sub-state is
  orthogonal to the geometry module. It can be developed, tested, and
  deployed independently. Geometry v2 must land first (task 03 provides
  `vertex_type()` and `next_hard_corner_s()`).

## Parameters

| Param | Default | Range | Meaning |
|---|---|---|---|
| `rpp_enable_pivot_turn` | False | bool | master switch for pivot sub-state |
| `rpp_pivot_approach_m` | 0.30 | [0.10, 1.0] | distance to begin slowdown |
| `rpp_pivot_entry_m` | 0.05 | [0.02, 0.15] | distance to vertex to enter PIVOT_TURN |
| `rpp_v_pivot_approach_m_s` | 0.10 | [0.03, 0.30] | speed cap during PIVOT_APPROACH |
| `rpp_pivot_settle_deg` | 3.0 | [1, 10] | heading tolerance to exit PIVOT_TURN |
| `rpp_pivot_settle_rate_dps` | 5.0 | [1, 20] | yaw-rate tolerance to exit PIVOT_TURN |
| `rpp_pivot_timeout_s` | 4.0 | [1, 10] | safety cap for PIVOT_TURN |

## Files to read first

- `src/rpp_controller_node.py` — state machine (`_update_state` method),
  control loop, velocity output block.
- `src/path_geometry.py` — `vertex_type()`, `next_hard_corner_s()`.

## Scope

### A. State machine

Extend `_update_state()` to handle `PIVOT_APPROACH` and `PIVOT_TURN`.
Add transition conditions per the table above.

### B. PIVOT_APPROACH velocity cap

In the velocity output block, when `state == PIVOT_APPROACH`:
```python
v_cmd = min(v_cmd, self._pivot_approach_v)
```

### C. PIVOT_TURN control

When `state == PIVOT_TURN`:
```python
v_cmd = 0.0
psi_cmd = self._path_geom.yaw_path(s_corner + 0.01)  # next segment heading
# Disable pose extrapolation
```

### D. Settle detection

Track `|ψ_e|` and `|ω|` (from IMU or yaw rate estimate). Exit PIVOT_TURN
when both are below thresholds.

### E. Timeout

If PIVOT_TURN exceeds `rpp_pivot_timeout_s`, log warning and resume
TRACKING. Avoid infinite hang.

### F. Diagnostics

Add `pivot_state` and `pivot_psi_err` to `/rpp/debug`.

## Tests

`tests/test_pivot_state.py`:

- HARD corner ahead → state transitions TRACKING → PIVOT_APPROACH at correct
  distance.
- PIVOT_APPROACH → PIVOT_TURN at correct entry distance.
- Settle conditions enforce both heading AND rate.
- Timeout fires after configured duration.
- No HARD corners in path → state stays in TRACKING throughout.
- Path with only SOFT corners → no pivot activation.

## Acceptance criteria (hardware A/B on bench)

Baseline (task 02) captured. Then with pivot turn on:

- [ ] **2 m square**: max xtrack at corners < 5 cm (vs 9.4 cm baseline).
- [ ] **2 m square**: pivot never times out on any corner.
- [ ] **Straight 5 m**: no regression — no pivot activation on straights.
- [ ] **Half-circle 180°**: no pivot activation on smooth curves (all SMOOTH).
- [ ] Heading error post-pivot exit < 3°.
- [ ] All `tests/test_pivot_state.py` pass.

## Out of scope

- Pen-up / pen-down at corners (separate channel, trajectory planner).
- Multi-segment continuous turns (only HARD vertices trigger pivot).
- Adaptive pivot threshold (start with fixed 45°, tune on hardware).

## References

- Naïo Oz agricultural robot uses a similar pivot-at-corner approach for
  row-end turns. Their threshold is ~60°.
- PX4 OFFBOARD mode supports zero-velocity position targets for spot-turns.