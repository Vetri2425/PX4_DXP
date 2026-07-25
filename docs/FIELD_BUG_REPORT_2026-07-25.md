# Field Bug Report — 2026-07-25 pre-line CSV runs

**Branch:** `Upgrade_Spray` · **Jetson git:** `a6869952`
**Bags:** `bags/25_07_2026/stg_2cbe5ef7_1784977916_20260725_{164205,164519,164707}/`
**Ground truth:** `bags/25_07_2026/curve_6_points-1.csv` (Emlid Reach RS3, 8 points, RTK FIX, lateral RMS 1.6–1.8 cm, `Samples=1`)

Three runs of one mission. `staged_mission.json` is byte-identical across all three (`md5 6e3eba5c…`), so every run-to-run difference is driving or placement — never planning. All findings below reproduce in all three runs.

**Mission geometry:** 97 waypoints, 4.804 m, 96 marked. Fitted arc R = 2.434 m, 113.1° total turn, 4.95 cm node spacing.

**Health baseline (clean):** 0 OFFBOARD drops, 0 setpoint gaps > 0.5 s, 0 RTK degraded, 0 EKF jumps, 0 pose-stale, 100% traversal (97/97).

---

## Verdict summary

| ID | Title | Sev | Status |
|---|---|---|---|
| [B6′](#b6-wgs84--px4-spherical-projection-mismatch) | WGS84 ↔ PX4-spherical projection mismatch across the stack | **P0** | Confirmed (new) |
| [B1](#b1-arc-fitter-discards-surveyed-control-points) | Arc fitter discards surveyed control points | **P1** | Confirmed |
| [B2](#b2-no-lateral-feedback--permanent-inside-cut-on-smooth-arcs) | No lateral feedback → permanent inside-cut on smooth arcs | **P1** | Confirmed (mechanism corrected) |
| [B3](#b3-spray-opens-10-s-late-stale-pivot-latch) | Spray opens 1.0 s late (stale pivot latch) | **P1** | Confirmed |
| [B4](#b4-spray-never-commanded-off-at-mission-end) | Spray never commanded OFF at mission end | **P1** | Confirmed |
| [B5](#b5-spurious-spray-pulse-on-path-load) | Spurious spray pulse on path load | **P1** | Confirmed |
| [B12](#b12-nan-blind-comparisons-manufacture-false-fails) | NaN-blind comparisons manufacture false FAILs | **P1** | Confirmed |
| [B7](#b7-1-verdict-reads-the-diluted-block) | §1 verdict reads the diluted block | P2 | Partly confirmed |
| [B8](#b8-7-infers-vertices-from-bend-angle) | §7 infers vertices from bend angle | P2 | Confirmed |
| [B9](#b9-8-reads-a-manifest-key-that-is-never-written) | §8 reads a manifest key that is never written | P2 | Confirmed |
| [B10](#b10-fcu-param-capture-calls-a-service-that-does-not-exist) | FCU param capture calls a non-existent service | P2 | Confirmed |
| [B14](#b14-preview--mission-fit_arcs-desync) | Preview ↔ mission `fit_arcs` desync | P2 | Confirmed (new) |
| [B15](#b15-pivot-release-band-collides-with-firmware-turn-threshold) | Pivot release band collides with firmware threshold | P2 | Confirmed (new) |
| [B16](#b16-controller-reports-done-mid-mission) | Controller reports `DONE` mid-mission | P2 | Confirmed (new) |
| [B13](#b13-rppdebug-publishes-unflagged-nan) | `/rpp/debug` publishes unflagged NaN | P3 | Confirmed |
| ~~B6~~ | ~~`gp_origin` is 3.84 cm off~~ | — | **Refuted** → B6′ |
| ~~B11~~ | ~~Entry pivot reverse-flip / long-way turn~~ | — | **Refuted** → B12, B15 |

---

## B6′ · WGS84 ↔ PX4-spherical projection mismatch

**Severity P0 — error is unbounded with distance.**

### Symptom

Deriving the local→WGS84 datum from each bag's own pose/GPS pairs gives `13.07201495, 80.26195694`, but the latched `gp_origin` says `13.07201530, 80.26195700` — a 3.84 cm north discrepancy, identical in all three runs.

That looked like a bad datum. It is not. Bin the implied shift by local north:

| north (m) | n | WGS84 model dN | PX4-spherical model dN |
|---|---|---|---|
| 5.0 | 785 | −3.32 cm | −0.71 cm |
| 5.5 | 159 | −3.64 cm | −0.87 cm |
| 6.0 | 123 | −3.90 cm | −0.86 cm |
| 6.5 | 113 | −4.13 cm | −0.84 cm |
| 7.0 | 111 | −4.37 cm | −0.83 cm |
| 7.5 | 443 | −4.62 cm | −0.78 cm |

A datum offset or antenna lever-arm is **constant** with distance. The WGS84 residual **walks −0.52 cm per metre of north**; under PX4's own projection it is flat. This is a *scale* error, not an offset. The remaining ~0.8 cm is degE7 quantisation (half of the 1.11 cm lat step — verified: `/mavros/global_position/global` sits exactly on the 1e-7 grid).

### Root cause

PX4 defines its local frame with a **spherical azimuthal-equidistant projection at R = 6 371 000 m**:

```cpp
// PX4-Autopilot/src/lib/geo/geo.h:55
static constexpr double CONSTANTS_RADIUS_OF_EARTH = 6371000;					// meters (m)
```

```cpp
// PX4-Autopilot/src/modules/ekf2/EKF/position_fusion.cpp:145
Vector2f Ekf::getLocalHorizontalPosition() const
{
	if (_local_origin_lat_lon.isInitialized()) {
		return _local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg());
```

The companion uses a **WGS84 geodesic** for the same conversion:

```python
# path_engine/ned.py:48
    geod = Geodesic.WGS84
    result = geod.Inverse(origin_lat, origin_lon, lat, lon)
    dist = result["s12"]
    bearing_rad = math.radians(result["azi1"])
    north = dist * math.cos(bearing_rad)
    east = dist * math.sin(bearing_rad)
    return (north, east)
```

At 13.07 °N the two models diverge by **−0.51 cm per metre north, +0.13 cm per metre east**:

| north (m) | error | east (m) | error |
|---|---|---|---|
| 1 | −0.51 cm | 1 | +0.13 cm |
| 5 | −2.54 cm | 3 | +0.39 cm |
| 7.7 | −3.90 cm | 10 | +1.29 cm |
| **100** | **−50.71 cm** | **100** | **+12.92 cm** |

### Load-bearing call sites

| Site | Consequence |
|---|---|
| `server/mission_placement.py:145` | anchor→EKF-origin translation — **−3.01 cm north baked into every placed mission** here |
| `src/point_ingest.py:279` | survey CSV → anchor-relative NED; stretches the plan +0.5% across its own extent |
| `path_engine/engine.py:666, 704` | georef ingest |
| `tools/analyze_mission.py:1670` | §12 commanded overlay, via `_import_ned_to_latlon()` at `:1622` |

### Why it hid

Both the planner *and* the original analysis used WGS84, so the errors cancelled inside the analysis. That is why the "derived datum" appeared to reconcile the error budget to 0.3 cm while the *correct* origin left a 2 cm residual. **The residual was real; the datum was not its cause.**

This is the same class as the already-fixed `georef.py` north-scale bug (`e483d53`), in a different module, with the opposite sign.

### Fix direction

Add a PX4-compatible `MapProjection` (spherical, R = 6 371 000, same `acos/k` form as `geo.cpp`) to `path_engine/ned.py`, and use it for **every** conversion crossing into or out of PX4 local NED — placement, ingest, `/path` publication, §12 overlay. Keep the WGS84 geodesic only for true ground distance between two lat/lon pairs (e.g. §8 miss distance).

> **Must be an all-or-nothing swap.** Mixing the two models in one pipeline is exactly what produced the cancelling errors above.

---

## B1 · Arc fitter discards surveyed control points

**Severity P1.**

### Symptom

The source CSV contains 8 surveyed RTK points. `staged_mission.json` declares `must_hit` at indices `[0, 95]` only — the two endpoints. All 6 interior surveyed shots lose their must-hit identity.

Reproduced bit-for-bit from the CSV with stock defaults:

```
fit_arcs=True   n pts 97   must_hit idx [0, 95]                       spray true 96
fit_arcs=False  n pts 98   must_hit idx [0, 11, 24, 42, 58, 68, 82, 96]
```

### Root cause

Provenance is correct at parse time and then overwritten. `path_engine/parsers/survey_csv.py:406` declares all 8 points as control (`"control_indices": list(range(len(pts)))`). `path_engine/engine.py:600-613` then replaces that metadata with the arc fitter's output:

```python
# path_engine/planners/arc_chain.py:591-599
    out: list[Point] = []
    control: list[int] = []
    for emit, fitted in zip(emits, is_fitted):
        if fitted:
            # Arc ends are control; interpolated arc fill is not.
            local_control = {0, len(emit) - 1}
        else:
            # Straight run: keep every surveyed vertex as a control point.
            local_control = set(range(len(emit)))
```

For a fitted run only the two arc endpoints survive. Interior surveyed points were replaced by tessellated circle fill and nothing maps them back onto the nearest emitted node. `engine.py:1166-1175` then derives `must_hit` as pure membership in `control_indices`.

For this mission the entire 8-point chain is a **single** fitted run (`_split_at_corners → [(0,7)]`), so all six interior points are lost. There is no partial survival.

The module docstring at `arc_chain.py:25-28` states the intended contract — every surveyed corner is must-hit, straight-run vertices stay control, interpolated fill does not — and is simply **silent on interior arc vertices**. The code implements "arc ⇒ only the ends", discarding measurements that were never fill.

### Downstream damage

`must_hit` gates RPP's `_simplify_path_for_profile` (`src/rpp_controller_node.py:1144-1208`) and the Phase-D point-hold FSM (`:1858-1884`). With only 2 protected points the controller simplified **72 of 97 nodes away**, and `spray_mode:point` would dwell only at the endpoints.

### What this bug is *not*

The arc fit does **not** introduce a systematic offset. Signed residuals surveyed→plan, in the mission's own planning frame:

```
-2.93  +3.40  +1.35  -2.24  +1.76  -2.63  +1.35  +0.00   cm
mean +0.007 cm   sd 2.20   max 3.40   sign changes 5 of 7
```

A least-squares circle fit is zero-mean in the radial residual by construction. The fit is unbiased; ±3.4 cm is still above the 1.7 cm survey noise floor, but it is scatter, not bias. Absolute placement error belongs to **B6′**.

### Config knobs — all at defaults, none would have helped

| Knob | Default | Would it preserve the 6? |
|---|---|---|
| `fit_arcs` | auto-ON for survey CSV (`path_manager.py:1094`) | Only by disabling arc recovery entirely — and see B14 |
| `fit_arcs_max_dev_m` | `MAX_ARC_DEVIATION_M = 0.15` (`arc_chain.py:56`) | No — decides *whether* to fit, never what stays must-hit. Max residual 3.40 cm is far inside 15 cm |
| `survey_tolerance_m` | `None` | **No — red herring.** Analysis-only; never reaches `PathEngine` |

### Fix direction

In `_fit_arc_run` / `fit_line_chain`, project every input surveyed vertex onto the emitted arc and either insert it at its on-circle bearing or mark the nearest emitted node as control. `local_control` must become `{0, len(emit)-1} | {mapped surveyed indices}`. `fit_line_chain` should return the surveyed→emitted index map so `engine.py:613` can preserve it.

Separately, `survey_tolerance_m` should be plumbed into the planner or renamed — its name implies it constrains planning when it only constrains the report.

---

## B2 · No lateral feedback → permanent inside-cut on smooth arcs

**Severity P1.** *(Mechanism and sign both corrected from the initial field read.)*

### Scope — this is a smooth-profile bug only

`tracking_profile=auto` split the mission into ≥3 runs. `/rpp/debug[40]` shows the profile alternating:

| run | profile | n | xtrack mean | lookahead (median) |
|---|---|---|---|---|
| straight | `segment` | 751 / 783 / 364 | **−0.05 / −0.23 / −0.26 cm** | 0.560 m |
| arc | `smooth` | 565 / 555 / 554 | **−5.44 / −5.14 / −5.28 cm** | **0.813 m** |

**The segment profile tracks straights to sub-3 mm.** The 5 cm belongs exclusively to the smooth arc run.

Sign: `/rpp/debug[0]` is documented `+ = right of path` (`rpp_controller_node.py:114`). Mean −0.054 m with path curvature median −0.26 (left turn) ⇒ left of path on a left turn ⇒ **INSIDE the curve. The rover cuts the corner.**

### Root cause 1 — the control law has no lateral term

`signed_xtrack` reaches the control law at exactly one place, wrapped in `abs()`, so the **sign is discarded**:

```python
# src/rpp_controller_node.py:3772-3787
        v_for_ld = max(min_v, self._last_speed_cmd if self._last_speed_cmd > 0.0
                       else max_v * 0.5)
        v_for_ld = 0.7 * v_for_ld + 0.3 * max_v
        l_d_raw = ld_gain * v_for_ld + xt_ld_gain * abs(signed_xtrack)
        l_d = self._clamp(l_d_raw, l_min, l_max)

        # Fix 1: curvature-aware minimum lookahead — on arcs, ensure l_d
        # spans at least 1/3 of the radius so the lookahead walk reliably
        # reaches past the foot. Without this, short lookaheads on tight
        # arcs can land at the rover position, triggering the IDLE path.
        kappa_path = self._path_curvature_at(seg_idx)
        if kappa_path > 1e-6:
            l_d = max(l_d, 0.35 / kappa_path)
```

Every other occurrence of `signed_xtrack` (`:3947`, `:3963`, `:3119`) is diagnostics. `xtrack_lookahead_gain` only *lengthens* the lookahead when off-path — it cannot pull the rover back. With no integral and no lateral term, **a steady offset is structurally permanent**.

### Root cause 2 — an undocumented curvature floor sets the lookahead

The `0.52 m` floor does **not** bind at 0.35 m/s: `l_d_raw` computes to 0.560 m (measured `debug[8]` median 0.5603). What binds is `l_d = max(l_d, 0.35 / kappa_path)` at line 3784 → 0.35 / 0.428 = **0.818 m** (measured `debug[2]` = 0.813 m).

Note this `max()` is applied **after** the `[l_min, l_max]` clamp, so it can silently exceed `max_lookahead_dist`.

### The equilibrium model

The offset is the closed-loop equilibrium of a projection-anchored pure pursuit under a pure-P heading loop:

```
e = L · ( L/(2R) − ω/RO_YAW_P ),    ω = v/R
```

With L = 0.814, R = 2.434, v = 0.35, `RO_YAW_P` = 1.5 → **e = 5.8 cm toward the centre**, θ_e = 5.49°. Measured: **5.1–5.4 cm inside**, θ_e = 5.9–6.0° (`debug[1]`). Model matches all three bags within 10%.

> The chord-cut term (`L²/2R` = +15.3 cm inside) and the heading-lag term (`ω/K` = −7.8 cm outward) **partially cancel**. Quoting either alone is numerology — the initial field analysis did exactly that and landed on the right magnitude for the wrong reason.

**"Scales with speed" does not survive scoping.** Restricted to the smooth run: v < 0.165 → −3.95 cm, v > 0.34 → −5.46 cm — but the low-speed samples also have lookahead 0.516 m (curvature floor inactive near the run end), so L and v move together. The 2.33 cm low-speed bin in the initial analysis was the *straight segment run* mixed in. At fixed L the model predicts e *decreasing* with v.

### Supporting: the arc structural floor is real and causal

`twist_to_setpoint_node.py:282-284` does send `yaw_rate` with `type_mask 455`, and `debug[10]` ≈ −0.098 rad/s = κ·v is present. But PX4's `DifferentialVelControl::generateVelocitySetpoint()` reads only `trajectory_setpoint.velocity[0]/[1]` and emits `(speed, bearing)` — `yawspeed` is never read. The feedforward is discarded and the yaw loop is pure-P. Separately, `yaw_rate_feedback_gain` is a no-op because the **gain is 0.00** (`debug[34]`), not because of the mode.

`a_lat_max` never binds: max κ_speed 0.4279 ⇒ v_lat = √(0.3/0.4279) = 0.837 m/s ≫ 0.35.

### Fix direction

1. Add a real lateral term — use the **signed** xtrack to bias the aim point or the commanded bearing.
2. The `0.35/κ` coefficient at `:3784` sets L on all arcs, and e scales ~L² — single highest-leverage knob.
3. Since PX4 discards `yawspeed`, any feedforward must be injected into the **bearing** (bias commanded velocity bearing by ω/`RO_YAW_P`).

---

## B3 · Spray opens 1.0 s late (stale pivot latch)

**Severity P1.**

### Symptom

| run | last `/rpp/segment_debug` | gate released | lag |
|---|---|---|---|
| 164205 | 36.976 s (profile=2 smooth, state=3 `CORNER_ALIGN`) | 37.983 | **+1.007 s** |
| 164519 | 33.740 | 34.767 | **+1.027 s** |
| 164707 | 21.700 | 22.747 | **+1.047 s** |

**9.5 / 9.9 / 9.7 cm of line left unpainted** at the start of every run. The node's own `/spray/status` reports `safety_reason='pivoting in place'` for the whole 16.5 s preceding release, while `/spray/debug[9]` (`geometry_desired`) is already `1` from path load — geometry wanted ON the whole time; only the gate held it.

### Root cause

The smooth profile never publishes `/rpp/segment_debug`. `rpp_controller_node.py:3745-3749` dispatches and returns before it:

```python
# src/rpp_controller_node.py:3745-3749
        if self._active_tracking_profile == "segment":
            self._control_segment_profile(
                pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal
            )
            return
        # ---- Step 1: Closest-point projection (segment, not vertex) ----
```

Everything after that `return` is the smooth profile and publishes only `_publish_debug(...)` (`:3958`). The last value on the wire is the `CORNER_ALIGN` from `_run_alignment_hold` (`:2449-2452`). The spray node then holds that stale state for the full timeout:

```python
# src/spray_controller_node.py:1583-1591
        if self._segment_state is None or self._segment_state_recv_time is None:
            return False
        timeout_s = max(0.0, float(self.get_parameter("segment_state_timeout_s").value))
        age_s = (
            self.get_clock().now() - self._segment_state_recv_time
        ).nanoseconds * 1e-9
        if age_s > timeout_s:
            return False
        return self._segment_state == _SEGMENT_STATE_CORNER_ALIGN
```

`segment_state_timeout_s = 1.0` (`:591`). The topic goes silent for 15.24 s — precisely the MARK span. The residual ~40 ms after gate release is `debounce_samples=3` at the 50 Hz watchdog (`_apply_debounce`, `:1163`).

### A test asserts this behaviour as correct

`src/test_spray_pivot_gate.py:114 test_stale_segment_state_is_permissive` asserts that a latched, message-free `CORNER_ALIGN` **must** suppress spray, then jumps the clock 2 s and asserts release. It encodes "hold spray off for the full `segment_state_timeout_s` after the last message" as the specification.

### Fix direction

The gate must fail **open** on staleness immediately — clear `_segment_state` on the first stale tick — or better, have the RPP publish `/rpp/segment_debug` (or `/rpp/progress`, already designed for this) in the smooth branch so a real tracking edge exists. Shortening `segment_state_timeout_s` only shrinks the gap.

---

## B4 · Spray never commanded OFF at mission end

**Severity P1.**

### Symptom

`/spray/desired` remains True until the bag ends — **5.5 / 4.9 / 5.0 s** after the mark span, stationary, with `/spray/state` confirming the valve is physically open. A puddle, and unbounded if the operator does not disarm.

### Root cause

The only OFF edge is geometric and speed-scaled:

```python
# src/spray_controller_node.py:475-499
            if src_kind is not None and math.isfinite(src_dist):
                distance_to_boundary = src_dist
                # ON is intentionally early by solenoid delay plus overspray
                # margin. OFF is early only by close delay; an explicit OFF
                # overspray margin delays shutoff so the MARK tail is not cut
                # short.
                on_lead = speed_mps * solenoid_open_delay_s + on_overspray_margin_m
                off_lead = max(
                    0.0,
                    speed_mps * solenoid_close_delay_s - off_overspray_margin_m,
                )
```

`off_lead = speed × 0.05`. At the measured terminal speed of 0.005–0.03 m/s that is **0.25–1.5 mm**, against a **13.7 mm** stopping gap. Final `/spray/debug`: `s=4.790, bnd_s=4.804, dist=0.0137, geometry_desired=1, desired=1`.

Reproduced in isolation with the real geometry:

```
stop 13.7 mm short, speed 0.008 -> geometry_desired=True   event=''
stop 13.7 mm short, speed 0.350 -> geometry_desired=False  event='off_early'
stop  0.0 mm short, speed 0.000 -> geometry_desired=False  event=''
```

**The shutoff works at 0.35 m/s and fails at the speed the rover actually finishes at.**

Three escape hatches all fail:

- **Crossing the terminal vertex** — `_next_boundary` (`:329-338`) requires `boundary.s > current_s + 1e-9`, and `_project_onto_path` clamps `t ∈ [0,1]`, so `s` can never exceed `cumulative_s[-1]`. The planner emits a zero-length trailing TRANSIT, so the sole `MARK_TO_TRANSIT` boundary sits exactly at the terminal station.
- **Fail-closed watchdog** — `active_timeout_s = 0.5` exists but is consumed only by `_legacy_active_watchdog_tick` (`:1208-1224`), which `_watchdog_tick` (`:1199-1204`) calls only when `use_distance_aware_spray` is **False**. In the default mode `/spray/active` going False at 52.215 s is literally ignored.
- **Mission completion** — `_rpp_kind_for` maps `REACHED_END → MARK_TO_TRANSIT` (`:98-99`) but is gated behind `consume_rpp_progress`, default `False` (`:598`), and `/rpp/progress` has **0 messages** in all three bags.

Only `shutdown_off()` (`:2007`) or a disarm forces OFF.

### Why tests missed it

Every decision test uses `speed_mps=1.0` (`test_spray_controller_v2.py:44`). `test_mark_to_transit_anticipatory_off` (`:103`) and `test_rpp_off_early_at_mark_end` (`test_spray_rpp_boundary.py:93`) pass only because `off_lead = 1.0 × 0.05 = 0.05 m` swallows the gap. At the frozen `segment_endpoint_approach_speed = 0.03` both scenarios fail. No test asserts the valve is OFF after the final waypoint.

### Fix direction

Add a terminal shutoff that is not speed-dependent: treat "projection within ε of the final station AND speed ≈ 0" as MARK end; floor `off_lead` with a fixed distance margin; and/or enable the built `REACHED_END` path. Independently, make a mission-level fail-closed watchdog live in distance-aware mode.

---

## B5 · Spurious spray pulse on path load

**Severity P1.**

### Symptom

A 0.32–0.36 s commanded-ON at t+21.11 / 22.85 / 13.93 s — well before the MARK span — with the rover **stationary** (0.1–0.2 cm of travel) 1–2 cm from the path start. `/spray/state` confirms 271 ms of real actuator ON. Deposits a blob at vertex 0.

### Root cause

Not a default-ON, not `dash_start_state` leakage (mode is `continuous`; `/spray/session_config` is not published in these runs), not a priming pulse, not an FSM artifact — **all candidate causes refuted**. It is a genuine geometric ON.

Frame-by-frame from `/spray/debug`, run 164205:

```
21.053  s=3.932  cflag=0  geo=0 safe=1 des=0   <- old 2-point path still latched
21.063  s=0.012  cflag=1  geo=1 safe=1 des=1   <- mission /path lands at 21.033; ON
21.464  s=0.014  cflag=1  geo=1 safe=0 des=0   <- RPP publishes CORNER_ALIGN at 21.397; OFF
```

The mission `/path` lands with the rover parked on vertex 0, which carries `z=3` (spray bit **and** must-hit bit), so `projection.current_flag` is immediately True. `_auto_safety_status` has **no "mission has started" condition**:

```python
# src/spray_controller_node.py:1599-1620
        if not self._armed:
            return False, "disarmed"
        require_offboard = bool(self.get_parameter("require_offboard").value)
        if require_offboard and self._mode != "OFFBOARD":
            return False, "not OFFBOARD"
        if self._session_mode == "point":
            if self._point_meter is None:
                return False, "point config not loaded"
        elif self._path_model is None:
            return False, "path not loaded"
        if not pose_fresh:
            return False, "pose stale"
        if not velocity_fresh:
            return False, "velocity stale"
        gps_ok, gps_reason = self._gps_gate()
        if not gps_ok:
            return False, gps_reason
```

Armed + OFFBOARD are already true ~22 s before path load, so **path arrival alone opens the valve**. The pulse ends only when the RPP reaches `CORNER_ALIGN`; before that it publishes `CORNER_STOP(5)` / `DONE(4)`, neither of which the gate treats as pivoting — by design (docstring at `:1564-1579`).

### Combined field signature

B5 then B3 produce **dot → gap → line** at the start of every run: a stationary blob at vertex 0, then ~10 cm unpainted, then the line. Look for this on the ground.

### Fix direction

Require positive evidence the run has started before honouring geometry at `s ≈ 0` — gate auto-spray until the RPP reports a tracking state, or suppress ON while stationary *and* the projection has not advanced from the first station. Any fix must not regress `test_spray_pivot_gate.py:86 test_corner_stop_does_not_suppress_spray` (mid-line corner coast-painting) or `test_spray_controller_v2.py:376`.

---

## B12 · NaN-blind comparisons manufacture false FAILs

**Severity P1** — this bug invalidates the verdicts the team acts on.

### Symptom

§3 prints `settle nan°` per pivot, then `worst settle 0.0°`, then `verdict FAIL` on a reverse-flip. Every one of those is wrong.

### Root cause (a) — sentinel treated as a real release

```python
# tools/analyze_mission.py:645
            i_rel = next((j for j in range(i_align + 1, n) if seg[j][1] == S_TRACK), n - 1)
```

No `S_TRACK` follows the ALIGN, so `i_rel` falls back to `n-1` — the **last sample of the bag**. Measured `i_align=1074, i_rel=2130`; `seg[2130]` state = 4 (`S_DONE`), `heading_err = NaN`. Consequence: the reported `settle_time_s = 36.34 s` is not a settle time at all — it is ALIGN-start to end-of-bag.

### Root cause (b) — argument order to `max()` swallows the NaN

`worst_settle` is initialised `0.0` and line 690 is `max(worst_settle, settle_deg)`. Python returns `b` only if `b > a`, and `nan > 0.0` is `False`:

```
max(0.0, nan) = 0.0        max(nan, 0.0) = nan
```

Not a filtered list, not an `or 0.0` fallback — a **silent NaN swallow that depends purely on argument order**. Downstream, `analyze_mission.py:1786` compares `0.0 > 3.0` and reports nothing: **a genuinely un-settled pivot would be graded perfect.**

### Root cause (c) — the false reverse-flip FAIL

Line 677's guard is `abs(math.degrees(he)) <= TURNING_BAND_DEG`. With `he = NaN` that is `False`, so the `continue` is skipped and post-mission DONE samples enter the reverse-flip scan:

| bundle | min_fwd incl. NaN | excl. NaN | NaN samples used | flip |
|---|---|---|---|---|
| 164205 | **−0.0245** | +0.0207 | 138 | True → **False** |
| 164519 | **−0.0474** | +0.0207 | 127 | True → **False** |
| 164707 | **−0.0216** | +0.0207 | 125 | True → **False** |

`FWD_EPS = −0.02`. **Every "reverse-flip detected during a pivot" FAIL in these three reports is manufactured by NaN-blind comparison.** With NaN excluded, §3 reads PASS.

### NaN-consistency audit

| Site | Filters NaN? |
|---|---|
| `:616` §1 overall | yes |
| `:623` §1 marking | yes |
| `:843` §4 commanded speed | yes |
| `:909` §6 pose-stale | **no** — `nan > 300` is False, so stale-unknown samples count as healthy |
| `:677` §3 reverse-flip | **no** — inverts the guard (c) |
| `:688` §3 settle | **no** — (a) |
| `:1380` §10 as-run config | **no** — latent; a midpoint stop-debug frame reads the whole RPP block as NaN |

Consistent in the two places that produce §1/§4 numbers; inconsistent in all four under §3/§6/§10, and the two §3 leaks are load-bearing — one suppresses a real check, one manufactures a false FAIL.

### Fix direction

Return the sentinel case distinctly (`i_rel=None`) so an unterminated ALIGN reports "pivot never released"; guard `settle_deg` with `math.isfinite`; replace `max(worst_settle, x)` with an explicit filter over finite values; add `if not math.isfinite(he): continue` at `:677`; audit every remaining `/rpp/*` comparison for the same failure mode.

---

## B7 · §1 verdict reads the diluted block

**Severity P2. Partly confirmed** — the analyzer *does* compute the correct number, it just never uses it.

### Symptom

§1 reports `overall RMS 2.71 cm` and drives the verdict from it. It also prints `marking : RMS 5.13` — the correct painted-span figure — and discards it.

```python
# tools/analyze_mission.py:614-631
    xt_all = [d[0] for (_t, d) in s.rpp if d and len(d) > 0 and math.isfinite(d[0])]
    out = {"available": True, "overall": _stat_block(xt_all)}
    # marking-only: xtrack while spray desired is ON (if we have that signal)
    spray = s.spray_active or s.spray_desired
    if spray:
        xt_mark = []
        for (t, d) in s.rpp:
            if not d or not math.isfinite(d[0]):
                continue
            on = _nearest(spray, t)
            if on:
                xt_mark.append(d[0])
        out["marking_only"] = _stat_block(xt_mark) if xt_mark else None
    ov = out["overall"]
    out["verdict"] = ("PASS" if ov and ov["rms_cm"] <= XTRACK_PROD_CM else "FAIL") if ov else "WARN"
```

`xt_all` has **no phase filter** — no armed, no OFFBOARD, no segment state, no spray gate. `marking_only` is computed correctly (n=760, 15.22 s span) and used for nothing: not the verdict at `:630`, not `worst_offenders` at `:1779-1780`.

Here 2.71 > 2.0 so it FAILs anyway — but **a run with a longer pivot would dilute below 2.0 and report PASS at 5.13 cm painted error.**

### The dilution is not "stationary samples" — it is synthetic placeholders

Measured: n=2890, NaN=117, finite=2773, **exact-`0.0` = 893 (32.2%)**. Those zeros are hardcoded, e.g. `src/rpp_controller_node.py:2116` (`CORNER_STOP` brake) and `:2367` (align release), published as `cross_track=0.0`. §1 averages 893 synthetic zeros into an RMS as if the rover were dead on the line.

This also explains the missing 32% in `bias -1.29 cm (L 44% / R 24%)` — `_stat_block` (`:476-493`) buckets `v < 0` and `v > 0` strictly, so `v == 0.0` lands in neither. 893/2773 = 0.322 exactly. The L/R split is an unlabelled readout of how much of the bag was not tracking.

### Fix direction

Gate the §1 verdict and `worst_offenders` on `marking_only` (falling back to `overall` only when no spray signal exists); exclude non-tracking samples from `overall` by joining against `/rpp/segment_debug` state; make `_stat_block` report `zero_frac` so L+R+zero sums to 100.

> Note `_nearest(spray, t)` at `:625` has no max-age guard — harmless here, but on a bag where the topic dies mid-run every later sample snaps to the last edge.

---

## B8 · §7 infers vertices from bend angle

**Severity P2.**

### Symptom

§7 reports `planned 97 pts (94 CAD vertices) -> tracked 61 pts`, `72 vertices removed by conditioning`, verdict WARN. The CSV had **8** surveyed points; the other 86 are densification output.

Measured directly: conditioned path vs raw `/path` = **0.00 cm mean, 0.00 sd, 0.00 max**, despite 97 → 61 nodes. Conditioning changed the geometry by nothing.

### Root cause

Vertices are inferred purely geometrically, and the must-hit bitfield is never consulted:

```python
# tools/analyze_mission.py:930-943
def _path_vertices(poly, bend_deg=VERTEX_BEND_DEG):
    """Interior points of *poly* where the heading turns by more than bend_deg.

    On a densified /path these are exactly the CAD-authored vertices: every
    other point is a resample sitting dead on its own leg.
    """
    out = []
    for i in range(1, len(poly) - 1):
        h0 = math.atan2(poly[i][1] - poly[i - 1][1], poly[i][0] - poly[i - 1][0])
        h1 = math.atan2(poly[i + 1][1] - poly[i][1], poly[i + 1][0] - poly[i][0])
        d = abs(_wrap(h1 - h0))
        if math.degrees(d) > bend_deg:
            out.append((i, poly[i], math.degrees(d)))
    return out
```

The docstring's premise holds for straight-line CAD geometry and **fails outright on an arc**, where every resample is by construction a bend. `VERTEX_BEND_DEG = 0.8` (`:87`); this plan bends 1.20° at every node → 94 of 97 classified as "CAD vertex".

The information exists and is unused: `collect()` stores `s.path_z` (`:575-578`), §8 uses it (`:1285-1287`), §12 uses it (`:1671-1672`), and `analyze_geometry_fidelity` (`:1106-1201`) never references it. Verified in the bag: `z & 2` is true only at `[0, 95]`, matching `staged_mission.json`.

Second defect — the WARN trigger counts *nodes removed*, not *geometry changed*:

```python
# tools/analyze_mission.py:1191-1201
        "dropped_total": len(all_dropped),
        "dropped_above_tolerance": len(intent),
        "worst_deviation_cm": worst,
        "xtrack_vs_planned": xt_true,
        "verdict": "FAIL" if intent else ("WARN" if all_dropped else "PASS"),
```

All 72 records carry `deviation_cm ≤ 0.04` and are tagged `"noise"`, yet `all_dropped` being non-empty forces WARN.

### Caveat for the fix

Switching §7 to `path_z` alone would make it check **nothing here** — the planner flags only the two endpoints (B1), and §7 examines only interior indices. **The §7 fix is necessary but not sufficient; B1 is the upstream defect**, and it is also why the real plan-vs-survey error is invisible to this section.

### Fix direction

Prefer `path_z & 2` as the vertex set when present, falling back to bend inference only when no flags exist — and state which was used in the report line. Change the WARN trigger from "any node removed" to "any removal that moved the geometry".

---

## B9 · §8 reads a manifest key that is never written

**Severity P2.**

### Symptom

§8 prints `source file unavailable (not recorded)` even though `manifest.plan_provenance.source_file` is correctly populated and the CSV exists on the Jetson. Absolute accuracy is dead **even on the rover**.

### Root cause 1 — key mismatch

The recorder writes `plan_provenance`:

```python
# tools/bag_autorecord.py:700
        self.manifest = {
            "schema": "bag_autorecord/manifest@1",
            "bundle": name,
            "identity": identity,
            "plan_provenance": _staged_mission(mission_id),
```

The analyzer reads `staged_mission`:

```python
# tools/analyze_mission.py:1215
    staged = ((manifest or {}).get("staged_mission") or {})
    src = staged.get("source_file")
    if not src or not os.path.isfile(src):
        return [], f"source file unavailable ({src or 'not recorded'})"
```

`manifest["staged_mission"]` has never existed. The same mismatch silently disables the operator's survey tolerance at `:961` — §7 reports `[built-in default — not set for this survey]` regardless of what was staged.

### Root cause 2 — on-rover absolute-path test

Even with the key fixed, `os.path.isfile("/home/flash/PX4_DXP/server/missions/curve_6_points-1.csv")` fails on any machine but the Jetson, and the file can be overwritten between run and analysis.

### Root cause 3 — no copy is ever made

`shutil` appears three times in `bag_autorecord.py` (`import`, `disk_usage`, `rmtree`) — never `copy`. `_snapshot_staged_artifact()` (`:486`) copies the staged JSON, not the source survey.

### The test mirrors the bug

```python
# tools/test_analyze_mission.py:290
    return {"staged_mission": {"source_file": str(csv)}}
```

Every §8 test constructs a manifest shape the recorder never produces. The suite is green on a feature that has never once run on a real bundle.

### Fix direction

Read `plan_provenance` (accept both keys for old bundles); add `_snapshot_source_file()` writing `bundle/source/<basename>` with a sha256 in `outcome.integrity`; make `_surveyed_latlon_from_source` prefer the bundled copy; build test fixtures via the recorder's own writer.

---

## B10 · FCU param capture calls a service that does not exist

**Severity P2.**

### Symptom

Every manifest carries `fcu_params.captured: false` with all 13 values null — no as-run record of `RO_YAW_P`, `PWM_AUX_MAX1`, etc. B2 depends on the former and spray flow on the latter, so both results are unreproducible.

### Root cause

```python
# tools/bag_autorecord.py:320-326
    values: dict = {}
    any_ok = False
    for pid in FCU_PARAM_NAMES:
        out = _run(
            ["ros2", "service", "call", "/mavros/param/get",
             "mavros_msgs/srv/ParamGet", f"{{param_id: '{pid}'}}"],
            timeout=4.0,
```

`/mavros/param/get` does not exist in MAVROS 2. Verified live on `192.168.1.102` with `px4-dxp` active — only `/mavros/param/pull` (`ParamPull`) and `/mavros/param/set` (`ParamSetV2`) are present. `mavros_msgs/srv/ParamGet` still ships as a message *definition*, which is why this passes review, but the ROS 2 param plugin exposes parameters through the standard parameter interface.

`ros2 service call` on a non-existent service blocks on "waiting for service", the 4 s timeout fires, the exception is swallowed at `:246-247`, every param records null, and nothing is logged. Secondary cost: **13 × 4 s ≈ 52 s of dead time inside `stop()`** before the analyzer spawn and retention run.

The correct interface is already in this repo:

```python
# server/ros_node.py:379
        if _HAS_PARAM_SRV:
            self._param_get_cli = self.create_client(
                GetParameters,
                "/mavros/param/get_parameters",
                callback_group=self._svc_group,
            )
```

Confirmed working on the rover: `ros2 param get /mavros/param RO_YAW_P` → `Double value is: 1.5`.

### Fix direction

Replace the 13 `ros2 service call` invocations with a single batched `GetParameters` request against the `/mavros/param` node. Record a `reason` string on failure and log it — an all-null block is currently indistinguishable from "params genuinely unset". Note the coercion at `:335`, `val = rv if rv != 0.0 else iv`, cannot distinguish a genuine `0.0` float from integer 0; typed `ParameterValue` removes the ambiguity.

---

## B14 · Preview ↔ mission `fit_arcs` desync

**Severity P2. New — found while confirming B1.**

`path_manager.py:886` (`preview_path`) **hardcodes** `fit_arcs=True`, while `plan_path` honours the kwarg via the resolution at `:1094`. Setting `fit_arcs=false` therefore desyncs preview from mission (97 vs 98 points), which trips the length guard in `mission_loading.py:78-87` and silently drops provenance to `None`.

This means the natural workaround for B1 — turning arc fitting off — is itself broken. Fix `preview_path` to honour the same resolution `plan_path` uses.

---

## B15 · Pivot release band collides with firmware turn threshold

**Severity P2. New — found while refuting B11.**

### Symptom

The run-boundary pivot dithers on its own release condition:

| bag | heading err at start | at release | duration | stuck < 5° |
|---|---|---|---|---|
| 164205 | −120.26° | −2.04° | 15.0 s | **8.9 s** @ median −2.26° |
| 164519 | −110.23° | −2.00° | 10.4 s | **4.9 s** @ median −2.43° |
| 164707 | −130.22° | −2.07° | 7.0 s | 0.5 s @ median −2.54° |

### Root cause

`segment_heading_tolerance_deg = 2.0` (`rpp_controller_node.py:397`) is **identical** to the firmware's `RD_TRANS_TRN_DRV = 0.0349 rad = 2.0°`, the SPOT_TURNING→DRIVING exit. Below 2° the rover stops rotating and starts driving; at exactly 2° the firmware's yaw rate is `RO_YAW_P × 0.035 = 0.052 rad/s`, just above the `segment_stop_yaw_rate_threshold = 0.05` release gate (`:2320-2340`). The pivot sits in the overlap.

### Fix direction

Open `segment_heading_tolerance_deg` to ~4–5° (clear of `RD_TRANS_TRN_DRV`), and/or raise `segment_stop_yaw_rate_threshold` above 0.052, and/or add a no-progress release.

---

## B16 · Controller reports `DONE` mid-mission

**Severity P2. New — found while confirming B13.**

The NaN bursts in `/rpp/debug` come in exactly **two per run** — one at the end (~1.2 s) and one **mid-mission** (20.1–21.2 / 21.9–22.9 / 12.9–14.0 s, 53–55 samples ≈ 1.05 s) at the run 0→run 1 boundary. So `/rpp/debug` reports **`DONE` for a full second in the middle of the mission**.

This matters because the server's mission-complete watcher keys on `DONE`. Candidate sites: `rpp_controller_node.py:2184` (`_hold_at_completion`), `:2985`, `:3605` (`_path_done` heartbeat). `_apply_run:1704-1705` clears `_path_done` per run, which is why tracking resumes.

Trace which of the three `DONE` publishers fires at a run boundary.

---

## B13 · `/rpp/debug` publishes unflagged NaN

**Severity P3.**

**100.0% of NaN samples carry state code 3 = `DONE`** (117/117, 111/111, 114/114). Single root cause:

```python
# src/rpp_controller_node.py:4278-4292
        """Publish (0, 0, 0) and a diagnostic. Used for IDLE/DONE/STALE/RTK_WAIT/JUMP_SKIP."""
        self._publish_velocity(0.0, 0.0)
        self._publish_yaw_rate(0.0)  # P3.1: zero yaw rate on stop
        self._last_speed_cmd = 0.0
        self._publish_debug(
            cross_track=float("nan"),
            heading_err=float("nan"),
            lookahead=float("nan"),
            speed=0.0,
            kappa=float("nan"),
            dist_goal=dist_to_goal,
```

`debug[0]/[1]/[2]/[4]` are NaN whenever the controller publishes a zero-velocity heartbeat. Intended ("no valid projection"), but nothing on the wire marks it, so every consumer must filter on `debug[7]` — and several in `analyze_mission.py` do not (B12).

**Fix direction:** emit `0.0` (or hold the last valid xtrack) plus an explicit validity flag rather than NaN.

---

## Refuted claims

Recorded so they are not re-raised.

### B6 — "`gp_origin` is 3.84 cm off the true datum" — REFUTED

`gp_origin` is correct to ~1 cm (degE7 quantisation). The 3.84 cm was a **projection-model artifact**, not a datum error: see [B6′](#b6-wgs84--px4-spherical-projection-mismatch). The antenna lever-arm does not explain it either — a fixed offset is constant with distance while the observed error is proportional to it; `EKF2_GPS_POS_X/Y = 0/0`; and local and global are the same EKF state through the same projection, so they cannot differ.

### B11 — "entry pivot turns the long way with a reverse-flip" — REFUTED

- **No long-way turn.** The rover turned exactly its initial error, the short way (−120.26° → −2.04°). The reported 268.5 / 247.6 / 261.3° is the pivot **plus** the 113° arc **plus** the ~40° final leg, measured in one window by the broken `i_rel` sentinel (B12a).
- **Not an entry pivot.** It occurs at the straight→arc *run boundary* (t = 21.5 / 23.3 / 14.4 s); the first 20 s is normal tracking at 0.35 m/s.
- **No sign/wrap bug.** `_enu_pose_to_ned` (`:1031-1049`) is correct, `heading_err = _angle_wrap(target − yaw_ned)`, and the ±75° clamp keeps PX4's forward projection at +0.0207 m/s.
- **The reverse is deliberate**, from the only path that commands a reverse vector (`:4123-4135`); PX4 then flips the bearing 180° by design.
- **The `flip=True` FAIL is an analyzer artifact** — see [B12](#b12-nan-blind-comparisons-manufacture-false-fails).

What is real at that pivot is [B15](#b15-pivot-release-band-collides-with-firmware-turn-threshold).

---

## Interaction: the errors are cancelling

Measured against surveyed truth over the painted span:

| term | value | owner |
|---|---|---|
| Plan placement vs truth | ~−3 cm north bias | B6′ |
| Arc-fit residual | ±3.4 cm, zero-mean | B1 (scatter, not bias) |
| Driven vs own plan | ~+5 cm inside the curve | B2 |
| **Net driven vs surveyed truth** | **~1 cm** | — |

The ~1 cm final accuracy is a **coincidence of opposing faults, not correctness**. Fixing B6′ or B2 alone moves the painted line 3–5 cm in one direction. They must be corrected together, with a re-survey after each step.

### Suggested order

1. **B6′** — unbounded with distance; everything else is measured in a frame this bug distorts. Fix first or every later measurement is suspect.
2. **B12, B7** — the analyzer manufactures false FAILs and hides real ones. Fix before using it to grade any subsequent change.
3. **B3, B4, B5** — cheap, independent, and each visibly damages paint.
4. **B1** (+ B14) — restores surveyed intent and re-arms point mode.
5. **B2** — needs a real lateral term; largest controller change, and its benefit is only measurable once 1–2 are done.
6. **B15, B16, B8, B9, B10, B13** — correctness and provenance cleanup.

---

## What no log can answer

**Where the paint physically landed.** Every number here describes the GPS antenna. Nozzle offset and spray latency sit between the antenna and the ground, and `nozzle_lateral_offset_m` is still uncalibrated. Only a physical re-survey of a painted line closes that gap — and given B2 and B6′, it is worth doing before and after any fix.

---

## Reproduction

```bash
python3 tools/analyze_mission.py bags/25_07_2026/stg_2cbe5ef7_1784977916_20260725_164205/ --quiet
```

`tools/analyze_mission.py` is stdlib-only and imports as a library on macOS without ROS:

```python
import importlib.util
spec = importlib.util.spec_from_file_location("am", "tools/analyze_mission.py")
am = importlib.util.module_from_spec(spec); spec.loader.exec_module(am)
bag, manifest = am._find_bag_dir("bags/25_07_2026/stg_2cbe5ef7_1784977916_20260725_164205/")
s = am.collect(bag)      # s.pose is (t, north, east, yaw) — north first
```

Two decode traps that cost time here: `s.pose` is `(t, n, e)`, **not** `(t, x, y)`; and `/path` `position.z` is a bitfield (bit0 spray, bit1 must-hit), so bit-test it rather than comparing `> 0.5`.

---

## FIX VERIFICATION — evening run 2026-07-25 18:20 (`stg_874827f3_1784983830_20260725_182055`)

Deployed `3634200` (B6′ `2dc7155`, B12/B7/B9 `22c02de`, B2 `3634200`). B2 A/B via
`ros2 param set /rpp_controller smooth_lateral_gain 1.5` + `smooth_curvature_ld_coeff 0.20`
(**runtime-only — resets on rpp-pipeline restart; defaults remain frozen 0.0/0.35**).

| metric | 3-run baseline | fixed run | owner |
|---|---|---|---|
| Plan vs surveyed truth, north bias | −1.47 cm (walks −0.52 cm/m) | **−0.02 cm** | B6′ |
| Plan vs surveyed truth, mean abs / max | 2.21 / 5.95 cm | 1.95 / **3.34 cm** (zero-mean arc-fit scatter) | B6′/B1 |
| Marking xtrack RMS / max (§1, marking basis) | 5.13 / 6.1 cm | **1.00 / 2.31 cm** | B2 |
| Driven vs own plan geometry RMS (§7) | 3.86 cm | **0.92 cm** | B2 |
| Driven vs surveyed truth, mean / max | 2.79 / 6.39 cm | **1.62 / 3.27 cm** | net |
| §3 reverse-flip verdict | FAIL (manufactured) | PASS, pivot reported NEVER RELEASED | B12 |

Health clean (0 drops / 0 gaps / 0 jumps / RTK FIXED). Plan length 4.804 → 4.811 m —
the visible frame-scale change (PX4 sphere vs WGS84).

Still open, reproduced on this run: **B1** (must_hit = endpoints only → §8 cannot pair
8 vs 2), **B3/B4/B5** (operator e-stopped ~15 s after completion to close the valve —
B4 signature), **B14, B15, B16, B8, B10, B13**.

---

## FIX VERIFICATION 2 — spray trio + disarm (run `stg_a0dd0306_1784986262_20260725_190115`)

Deployed **`d0ae176`** (B4 terminal spray shutoff + disarm-on-complete) and **`8da931d`**
(B3 stale-latch fail-open + smooth-profile `/rpp/segment_debug` `TRACK_SEGMENT` + B5
awaiting-tracking gate). Jetson git `8da931d7`. B2 runtime params still live
(`smooth_lateral_gain 1.5`, `smooth_curvature_ld_coeff 0.20`). Same mission/CSV as the
whole day (`curve_6_points-1.csv`); marking = the fitted arc run (97 pts, s=4.811 m,
last spray-flagged station idx 95 @ s=4.711, i.e. MARK ends 10 cm before the trailing
transit). Bag 47.2 s, 1419 pose samples.

### Verdict: **B3, B4, B5 all fixed; disarm confirmed.** Tracking held class (marginally worse this run, not from these fixes).

**Timeline (arc/MARK run):** run-boundary pivot `ALIGN` 293.24→302.94 s; first
`TRACK_SEGMENT` 302.938; spray desired ON 303.003; `TRACK`→`PRE_CORNER` 318.537; spray
desired OFF 319.086; `STOP`/`DONE` 322.0. Bag ends 323.77.

| Bug | Baseline signature (164xxx / 182055) | New bag `…_190115` | Verdict |
|---|---|---|---|
| **B5** spurious pulse on load | spray ON **+0.075–0.081 s** after marking `/path` lands, rover **stationary** at vertex 0 (~0.1 cm travel), 0.32–0.36 s blob; `desired 4 edges` | marking `/path` lands 292.895 s (rover parked 2.05 cm from v0, v=0.003 m/s); first `desired` ON **+10.108 s** later, only after tracking began (5.4 cm past v0, moving); **no pulse during the 9.7 s pivot**; `desired 3 edges` | **FIXED** |
| **B3** spray opens ~1 s late | spray ON **+1.0 s** after tracking start → 9.5–9.9 cm unpainted; smooth run publishes **zero** `/rpp/segment_debug`, pivot logged `NEVER RELEASED (topic went silent)` | first `TRACK_SEGMENT` 302.938 → spray ON 303.003 = **+0.065 s** (~a few mm); **1295 `TRACK`(state 1) msgs during the arc**; pivot now measurably released | **FIXED** |
| **B4** never OFF at mission end | `desired` held **True to bag end** (4.9–5.5 s stationary, operator e-stop closed valve); actuator open the whole time | last `desired` True→False **319.086 s**, projection **exactly at last spray station** (idx 95, 0.0 cm past MARK boundary), v=0.053 m/s; `desired` **False for the remaining 4.68 s / 243 samples to bag end**; actuator (`/spray/state`) closes 319.086, `/spray/active` 318.537 | **FIXED** |
| **Disarm** on complete | — | not in bag (`/mavros/state` = 0 samples recorded); **confirmed from live journal**: "completion disarm ok" **+45 ms** after mission-completed, FCU ended `armed:false` | **CONFIRMED (journal)** |

Edge counts: baselines `desired 4 edges` (initial-off + spurious ON + spurious OFF +
real ON-that-never-closed). New bag `desired 3 edges` (initial-off + one ON + one OFF)
= exactly **one clean pulse**. `commanded 3 / state 3` match; state↔desired latency
0.011 s; **0 misfire samples**.

### Tracking / placement — no functional regression, one metric marginally worse

| §  | metric | 164xxx baseline | 182055 (B2-tuned) | **new `…_190115`** | note |
|---|---|---|---|---|---|
| 1 | marking xtrack RMS / p95 / max | 5.13 / 5.96 / 6.1 cm (FAIL) | 1.00 / 2.17 / 2.31 cm | **1.41 / 3.91 / 4.14 cm** (PASS) | within class (≤2.0), but ~40% worse than 182055 |
| 2 | endpoint closest / resting, DONE | 1.7 / 1.9, True | 1.6 / 1.9, True | **1.7 / 1.8, True** (PASS) | unchanged |
| 6 | OFFBOARD drops / gaps>0.5 s / RTK deg / EKF jumps / pose-stale | 0 all | 0 all | **0 all** (PASS) | clean |
| 7 | driven-vs-planned RMS / max | 3.86 / 6.13 cm | 0.92 / 2.35 cm | **1.88 / 4.17 cm** (WARN*) | ~2× 182055; *WARN is the B8 "any node removed" artifact, geom Δ ≤0.04 cm |
| 9 | traversal | 97/97 | 97/97 | **97/97** (PASS) | complete |

The marking-RMS and §7 rise (vs 182055) is **run-to-run arc-entry driving variation, not
attributable to the spray/disarm commits** — those add a `/rpp/segment_debug` publish in
the smooth branch and touch the spray node + completion disarm; none alter the velocity
control law. B2 (arc inside-cut equilibrium) is still the structural driver and its runtime
gains were live in both runs. **Both figures remain within production class; flag and watch
on the next run, but nothing here regressed.**

### Side-effect sweep

- **§3 now reports FAIL** (`pivot settle 3.76° > 3.0°`) where baselines showed
  `NEVER RELEASED … PASS`. This is **not a regression** — it is a direct consequence of the
  B3 fix: with `TRACK_SEGMENT` now following the `ALIGN`, the analyzer can finally *measure*
  the pivot settle instead of vacuously passing. The 3.76° dither is **known-open B15**
  (release band collides with `RD_TRANS_TRN_DRV`), unrelated to spray.
- `/rpp/segment_debug` state histogram: `TRACK 1295 · PRE_CORNER 397 · ALIGN 485 · STOP 68 · DONE 121`.
  All `TRACK` fall inside the arc MARK span; `ALIGN/STOP/DONE` only at run boundaries. **No unexpected states during the smooth run.**
- No spray chatter (2 real transitions, ON then OFF), **0 OFFBOARD drops, 0 setpoint gaps**, RTK FIXED throughout.
- **B1 still open as expected:** `must_hit = [0, 95]` (endpoints only); §8 still `cannot pair 8 surveyed vs 2 local`. Not in scope for this run.

**Owners:** B3/B4/B5 → deployed `d0ae176`+`8da931d`, verified here. Marking-RMS watch → controller/B2. §3 FAIL → B15. B1/§8 pairing → still open.

**Reproduce:** `.venv/bin/python tools/analyze_mission.py bags/25_07_2026/stg_a0dd0306_1784986262_20260725_190115/`
(spray-edge / segment-state timings pulled via the library `collect()` — `s.seg` state codes
`1=TRACK 2=PRECORNER 3=ALIGN 4=DONE 5=STOP`, `s.spray_desired`, `s.path_z & 1` = spray bit).
