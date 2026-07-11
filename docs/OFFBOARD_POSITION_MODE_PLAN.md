# Implementation Plan — Native OFFBOARD Stop + Pivot via Firmware Position Control

**Status:** PLAN ONLY — not implemented.
**Revised:** 2026-07-11 (rev 2), after a composer self-review corrected three claims. Supersedes the rev-1 "angle-independent, 0-LOC, robust" framing.
**Scope:** firmware + companion `~/Vetri/PX4_DXP`.

> **⚠ SOURCE-OF-TRUTH — the flashed binary is a CI OVERLAY build, not a tree you read directly.**
> `.github/workflows/build_rover.yml` does: (1) fresh checkout of **stock PX4 v1.16.2** (`ref: v1.16.2`); (2) checkout the fork for `fork_patches/`; (3) `cp fork_patches/src/... → src/...` — overlay only the listed files onto stock. So authoritative source depends on the file:
> - **Overlaid (fork version runs):** read `fork_patches/src/...` — `RoverDifferential.{cpp,hpp}`, `module.yaml`, `DifferentialVelControl.{cpp,hpp}`, `RoboClaw.*`, `RoverLandDetector.cpp`, `mission_block.cpp`, EKF2 files.
> - **NOT overlaid (stock v1.16.2 runs):** `DifferentialPosControl` (absent from `fork_patches/`), `PurePursuit`, `RoverControl`, commander, etc. — read **stock v1.16.2**.
> - **Traps:** the workspace `src/.../DifferentialPosControl.cpp` (13 `rover_speed_setpoint` refs) is a **stale fork leftover CI overwrites and ignores** — editing it changes nothing. `~/px4-rover-build/` is a local build **mirror** of the overlay result (useful to read, not canonical).
> - **To PATCH firmware (Phase B/C):** add the file to `fork_patches/` **and** to the overlay list in `build_rover.yml`. Any PosControl change must be authored on **stock v1.16.2 PosControl**, never the fork `src/` copy.

---

## 0. Hardware evidence so far (log_23, 2026-07-11 — AUTO single-point go-to)

A QGC single-point go-to (AUTO_MISSION, not OFFBOARD, but the **same `goToPositionMode` controller** past the setpoint source) gives the first real evidence:

- **Stop: CLEAN & VALIDATED.** ≈5 cm error, exactly `NAV_ACC_RAD = 0.05`. Monotonic approach (285→5 cm, no blip), **zero overshoot, zero reversals, zero hunting**, rock-solid zero-creep hold 36 s (throttle ≈ 0, noise < 0.5 cm). Refutes the coast-past and hunting failure modes.
- **≤2 cm plausible:** hold noise < 0.5 cm ≪ 2 cm, so tightening `NAV_ACC_RAD` to 0.02 should stop 3 cm closer without hunting. Decider = one run at `NAV_ACC_RAD = 0.02`.
- **Pivot: UNTESTED.** Initial heading error only 6.6° (< 40° `RD_TRANS_DRV_TRN`), so it never spot-turned — drove-and-corrected. No evidence on spot-turn reverse-flip/overshoot. **Need a > 40°-mis-headed run.**
- **OFFBOARD ingestion: still unproven** — this was AUTO (`position_setpoint_triplet`); `rover_position_setpoint` never appeared. Stop behavior transfers (same controller); the OFFBOARD `trajectory_setpoint.position` path is source-verified only.

**Live params (authoritative 2026-07-11, from the ULog dump):** `NAV_ACC_RAD 0.05`, `RO_JERK_LIM 0.30` (G0 safe), `RO_DECEL_LIM 0.30`, `RO_MAX_THR_SPEED 0.96`, `RBCLW_QPPS_MAX 182655` (autotuned), `RD_TRANS_DRV_TRN 0.70` (40°), `RD_TRANS_TRN_DRV 0.0349` (2°), `RO_YAW_P 1.5`, `COM_OF_LOSS_T 30.0`.

**Gate status update:** G0 (NaN) — **CLEAR** (`RO_JERK_LIM = 0.30 > 0`). G1 (speed) — **DE-RISKED**: measured overshoot is ~15-30% (speed-loop PID transient), NOT the ~2× earlier hypothesized; `RO_MAX_THR_SPEED = 0.96` is roughly right and the stop was clean despite it. A refinement, not a blocker.

---

## 1. The functional requirement (from the operator)

The rover starts at an **arbitrary** angle and position — straight or not, near or far, no fixed-path assumption. From there it must:

1. **Stop cleanly at the first point**, unaffected by start geometry.
2. **After stopping, smoothly pivot** toward the next line / arc / point — cleanly for **any** heading change, short or large, unaffected by angle magnitude.
3. This applies to **every** transition: runtime entry, segment-to-segment, point navigation — not just the runtime entry.

Marking passes themselves stay on the companion's velocity tracking (keeps sub-2 cm cross-track). Position mode is used **only** for the stop+pivot transitions between passes.

---

## 2. Core finding — the primitive already exists and is compiled

Stock `DifferentialPosControl` (in the flashed CI #28 build, NOT the fork copy) already implements the full OFFBOARD position → stop → pivot → hold chain. **Verified from source:**

- `RoverDifferential.cpp:72` calls `updatePosControl()` every 10 ms.
- `DifferentialPosControl.cpp:65-72` gates output on `flag_control_position_enabled && flag_armed && runSanityChecks()`; when OFFBOARD + `offboard_control_mode.position`, it runs `generatePositionSetpoint()` (reads `trajectory_setpoint.position`, publishes `rover_position_setpoint`) then `goToPositionMode()`.
- `goToPositionMode()` (~286-316): if outside `NAV_ACC_RAD`, PurePursuit with start=current (degenerate segment) → bearing points **directly at the target** from any start geometry; publishes **positive** speed + bearing (no reverse-flip in this path). On arrival (≤ `NAV_ACC_RAD`): speed 0, hold current yaw.
- Overlay `DifferentialVelControl.cpp:172-176` spot-turn FSM: `DRIVING → SPOT_TURNING` **only when `|heading_error| > RD_TRANS_DRV_TRN`** (0.70 rad = **40°** on this vehicle), speed forced 0 in that state; `SPOT_TURNING → DRIVING` when error < `RD_TRANS_TRN_DRV` (2°). `wrap_pi` bounds error to ±π so a ~180° turn pivots in place with no reverse-flip.

**This satisfies requirement §1 for real corners, NOT for every angle — corrected from rev 1.** The pivot is **threshold-gated**: only a heading change **> 40°** produces a stop-and-pivot. Below 40° the FSM stays in `DRIVING` and **drives-while-steering — an arc, not an in-place pivot**. Feeding a commanded yaw does not change this; the pivot goes through the same FSM. To force in-place pivots at small angles you must lower `RD_TRANS_DRV_TRN`, at the cost of the rover spot-turning for small corrections during normal driving too.
- This 40° threshold mirrors the companion's own 45° corner threshold (pivot for real corners, flow through gentle ones) and is likely desirable — a hard stop-pivot at a 10° transition is usually not wanted. But it means the literal spec "clean pivot for every angle, short or large" is a **tuning decision** (`RD_TRANS_DRV_TRN`), not a free property.
- The primitive is code that already runs, merely unfed in OFFBOARD today because the companion only sends velocity setpoints. It fires **only** when `offboard_control_mode.position` is set — the flashed firmware does not do transitions on its own.

**Idle-safe — only for velocity-only.** A velocity-only OFFBOARD mission leaves `flag_control_position_enabled` false, so PosControl publishes nothing and today's line tracking is unaffected. It is **not** unconditionally safe: if position is enabled with bad/NAN XY, or across a mode switch, VelControl **holds the last `differential_velocity_setpoint`** until a new one arrives (see §7). The companion must never assert the position bit without a valid target.

---

## 3. The architecture (hybrid)

| Phase | OFFBOARD mask | Firmware controller | Behavior |
|---|---|---|---|
| Marking pass (line / arc / segment) | velocity (current mask 2503) | `DifferentialVelControl` | companion RPP tracking — **unchanged** |
| Stop + pivot at every transition (entry, corner, endpoint, point-nav) | **position** | stock `DifferentialPosControl` → VelControl spot-turn | drive-to-target, spot-turn from any heading, decelerate, stop, hold |

The companion emits a **position** setpoint (the next target point) at each stop/pivot, then switches back to **velocity** to track the following marking pass. One primitive covers stop *and* pivot because that is what pure-pursuit-with-spot-turn is.

This replaces the entire companion stop apparatus — CORNER_STOP, `_corner_brake_velocity`, the completion latch, `corner_stop_hold_s`, the runtime-entry hold, the velocity-vector pivot (BUG-T3 source) — with "send position target, await arrival."

---

## 3A. Transit integration with the PRE/MARK/AFT extension architecture

The path is already structured as **marking spans** (PRE → MARK → AFT per line) separated by **transit gaps** (AFT-of-line-N → PRE-of-line-N+1, and the runtime entry → first PRE). Each OFFBOARD mode owns one and never the other. This is what keeps precise tracking undisturbed and solves the yaw deficit with **zero firmware**.

```
   ── velocity mode (precise, spray as flagged) ──        ── velocity mode ──
  [ PRE → MARK → AFT ]                                   [ PRE → MARK → AFT ]
                       \                                 /
                        └── position mode: transit gap ─┘
                            (stop + pivot at the corner)
```

### Ownership rule (the invariant)
- **Velocity OFFBOARD owns PRE→MARK→AFT entirely** — the whole marked span, unchanged, sub-2 cm cross-track. Position mode is **never** active during a MARK segment.
- **Position OFFBOARD owns only the transit gap** — the corner transit and the runtime entry, both of which are spray-OFF.
- Every mode switch happens in a **spray-OFF buffer** (AFT or PRE), so no transition ever touches marked geometry.

### Transit sequence (corner or runtime entry)
At the end of an AFT (or from an arbitrary start, for the runtime entry):
1. **Ramp velocity to ~0 at the AFT end, then flip the mask to position.** The AFT buffer provides the room; the ~500 ms `flag_control_position_enabled` lag (§7) lands here with the last velocity command already near zero, so the stale-setpoint latch is harmless. **The extension buffer is what makes the switching lag a non-issue at this seam.**
2. **Position target = next PRE-start.** Firmware drives there, spot-turns if the corner exceeds `RD_TRANS_DRV_TRN` (40°), decelerates on its profile, stops within `NAV_ACC_RAD`. Clean corner/entry stop.
3. **Re-target to a point down the next line** (its MARK-start, or any collinear PRE/MARK/AFT point). The firmware's pure-pursuit bearing to that point **is the line heading**, so it spot-turns in place at PRE-start to face down the line — clean, firmware-native, angle-independent for real corners.
4. **Flip the mask back to velocity** as it begins moving down the line. RPP resumes precise tracking through PRE→MARK→AFT.

### Why the extensions are the enabler
1. **Buffers are the switch zones.** AFT (spray off) hosts velocity→position; PRE (spray off) hosts position→velocity and lets the velocity tracker null residual cross-track **before** spray turns on at MARK. No mode transition touches marked geometry.
2. **Collinearity gives the down-line yaw target — this is the 0-firmware fix for the yaw deficit.** Because PRE/MARK/AFT are collinear along the line, any downstream point is a valid "virtual target down the line." Aiming the position setpoint at it makes the firmware's pure-pursuit bearing equal the line heading, so the rover finishes the pivot facing correctly **without** `trajectory_setpoint.yaw` (Phase C firmware is not required for line-direction alignment when extensions exist).
3. **The PRE buffer covers the handoff transient.** Any residual heading/cross-track error at the velocity handoff is absorbed in the PRE run-up, so MARK entry is clean.

### Automatic gentle-vs-sharp behavior
Letting position mode drive corner-to-corner means the firmware's 40° spot-turn threshold decides pivot-vs-flow automatically: near-collinear joins flow through (no wasteful stop), real corners spot-turn. This matches the existing companion corner philosophy with no companion threshold code.

### What this does NOT change
Path engine, extension generation, run splitting, MARK velocity tracking, spray flag conditioning — all untouched. The change is confined to: emit a position setpoint at each transit gap, and hand back to velocity inside the PRE buffer.

---

## 4. What must be built (minimal, staged)

### Phase A — enable and validate the existing primitive (0 firmware LOC)
The bridge already works. The work is entirely companion-side + params + testing:
- **G1 speed calibration first (see §5).** The decel-to-stop profile depends on it.
- Companion: emit a position-masked `PositionTarget` (position + IGNORE velocity) at stop/pivot phases; resume velocity for marking.
- Bench-test that a bare position setpoint drives-stops-pivots-holds as the code implies.
- No firmware flash needed for Phase A if the flashed CI #28 already contains stock PosControl (confirm the flashed image exposes it — see §7 gate).

### Phase B — optional default-off feature flag (~15-25 LOC, firmware)
Only if a hard on/off gate is wanted for safety/rollback beyond "companion doesn't send the position bit":
- Author on **stock v1.16.2 PosControl**, place the modified copy in `fork_patches/src/.../DifferentialPosControl.*`, and add the `cp` line to `build_rover.yml`'s overlay step. **Never base it on the fork `src/` PosControl or `DifferentialOffboardMode`** (needs `RoverSpeedSetpoint.msg` etc., absent in v1.16.2 → hard compile error, per `patches.md`).
- Add `RD_OFFB_POS` (int, default 0) in the already-overlaid `module.yaml`.
- Gate: `if (_param_rd_offb_pos.get() && _offboard_control_mode.position)` before `generatePositionSetpoint()`.
- Rollback = param toggle, no re-flash (param lives in the image).

### Phase C — commanded final-yaw pivot (~15-25 LOC, firmware, stock PosControl only)
**Needed for clean line-start alignment.** Today `goToPositionMode` orients toward the target *point* and holds current yaw at arrival — it does **not** align to a commanded heading. For marking we want the rover at the line's start *facing down the line* before the velocity pass begins.
- Extend stock `DifferentialPosControl`: when `trajectory_setpoint.yaw` is finite, carry it through; after arrival (or as a second phase) set `differential_velocity_setpoint.bearing = yaw`, speed 0, so VelControl spot-turns to the commanded heading and holds.
- Note: stock `RoverPositionSetpoint.msg` documents `yaw` as "Mecanum only" and `generatePositionSetpoint` currently forces `yaw = NAN` — this extension changes that for differential.
- Alternative with 0 firmware LOC (evaluate first): send as the position target a point *down the line* so the direct bearing ≈ line heading and the rover arrives already aligned. Hacky near the start point; Phase C is the clean version.

---

## 5. Prerequisite gates (QGC params, do before any position setpoint)

### G0 — jerk/decel limits > 0 (SAFETY, hard gate)
`goToPositionMode` calls `computeMaxSpeedFromDistance(RO_JERK_LIM, RO_DECEL_LIM, distance, 0)` (build tree `DifferentialPosControl.cpp:264,273,292`). **Defaults are -1 (disabled)**, and with a disabled limit the speed setpoint can go **NaN** for a target ≳2 m away — a NaN speed command to the wheels. The `> FLT_EPSILON` guards only protect if both are positive.
- Our vehicle: `RO_DECEL_LIM = 0.30` (OK). **`RO_JERK_LIM` is UNVERIFIED — read it in QGC and confirm > 0.**
- **No position setpoint may be sent until both `RO_DECEL_LIM > 0` and `RO_JERK_LIM > 0` are confirmed.**

### G1 — speed calibration
`RO_MAX_THR_SPEED = 0.90` vs RoboClaw `RBCLW_QPPS_MAX` are uncoupled; bags show ~2× overspeed (`RO_SPEED_LIM = 0.30` never enforced on actual — it only clamps the setpoint). The firmware's decel-to-stop profile uses this map, so an uncalibrated value makes position-mode stops overshoot exactly as velocity mode does.
- Bench: measure true full-throttle straight-line speed; set `RO_MAX_THR_SPEED` (QGC) to it; re-verify commanded ≈ actual.
- **Blocks everything. Zero code.**

### G-pivot — decide `RD_TRANS_DRV_TRN`
The pivot is only in-place above this threshold (40° today). Decide whether small-angle transitions should arc (leave 40°) or pivot in place (lower it, accepting spot-turns during driving corrections). This is a spec decision, not a bug.

---

## 6. Companion changes (`PX4_DXP`)
On a branch off `main`, after Phase A bench validation. Implements the §3A transit model:
- **Setpoint bridge**: add a position-emit path publishing a position-masked `PositionTarget`; mask flips **atomically on one OCM message** (position/velocity bits set together on the same `offboard_control_mode` + matching `trajectory_setpoint`).
- **RPP**: at each transit gap (runtime entry, corner = AFT-end→next-PRE-start, final endpoint), run the §3A sequence — ramp velocity to ~0 in the AFT buffer, flip to position (target = next PRE-start), on arrival re-target a **collinear down-line point** for the pivot, then flip back to velocity inside the PRE buffer. MARK velocity tracking unchanged.
- **Down-line target from extensions** (0-firmware yaw): use a PRE/MARK/AFT collinear point as the pivot target so the firmware bearing = line heading. No `trajectory_setpoint.yaw` needed.
- **Arrival / handoff detection**: advance the transit state on `vehicle_local_position` velocity ≈ 0 and position within `NAV_ACC_RAD`; resume velocity when aligned and beginning down-line motion.
- **Keep old stop code behind a flag** (CORNER_STOP, `_corner_brake_velocity`, completion latch, `corner_stop_hold_s`, runtime-entry hold, velocity-vector pivot) until position mode is field-proven; delete after.

---

## 7. Switching risk — verified, must be handled
`vehicle_control_mode` is republished ~2 Hz or on `_status_changed`; OCM bit flips do **not** set `_status_changed`, so `flag_control_position_enabled` can lag the companion's mask change by **up to ~500 ms**. During the lag:
- velocity→position: Vel already sees `position` and stops feeding, but Pos is still gated off → a window on the **stale last `differential_velocity_setpoint`**.
- position→velocity: Pos may keep running `goToPositionMode` on the last position target until the flag clears.

Companion must: keep OFFBOARD engaged and the stream continuous, flip bits atomically, and tolerate ~500 ms of flag lag (hold a safe setpoint across the transition). **This transition is the primary integration risk — bench-test velocity→position→velocity in isolation before any mission.**

---

## 8. Build → flash → validate
1. G1 calibration (QGC).
2. Confirm the flashed CI #28 image already exposes stock PosControl OFFBOARD-position behavior (§7 gate) — if yes, Phase A needs no flash.
3. Bench, single position setpoint 1 m ahead from an off-heading start: confirm spot-turn → drive → stop → hold. Measure stop tolerance vs `NAV_ACC_RAD`. **Make-or-break.**
4. Bench, velocity→position→velocity transition: confirm no OFFBOARD drop, quantify any glitch.
5. Companion Phase A wiring behind a flag; single 2 m line (entry stop + endpoint stop + one pivot).
6. If line-start alignment is too loose, add Phase C (commanded yaw), re-flash via CI target `cubepilot_cubeorangeplus_rover` (flash CI artifacts, not the local Apple-Silicon build — functionally equivalent, not bit-identical).
7. Staged shapes: L-shape → square → triangle → circle. Auto-bag every run; compare validated-day metrics.

**Rollback:** companion phase flag; firmware `RD_OFFB_POS=0` (if Phase B) or re-flash `06309e41`. Nothing deleted until proven.

---

## 9. Open questions / UNCERTAIN (from source review)
1. Turn direction at **precisely ±π** heading error (sign of `wrap_pi` at the discontinuity) — pathological, verify on bench.
2. Whether the ~500 ms flag-lag glitch is noticeable on hardware (depends on slew, `COM_OF_LOSS_T`, last commanded speed).
3. Arrival hold robustness under GPS jump — stock `goToPositionMode` has **no `_stopped` latch**, only speed 0 + current yaw each cycle; may drift on an EKF jump.
4. Does the flashed CI #28 image actually enable OFFBOARD-position (is the behavior reachable without any flash), or does it need Phase B first? (§8 step 2)
5. Cross-track as it decelerates into a marking endpoint — does drive-to-point cut off-line near the end? (matters only if the endpoint is a marking point)
6. Spray-off timing across the velocity→position handoff.

---

## 10. Deliverable summary (rev 2)
| Question | Answer |
|---|---|
| Buildable? | **Yes** — stock PosControl (build tree) implements the OFFBOARD→position bridge; do NOT overlay the fork file. |
| Robust stop+pivot? | **Yes for corners > `RD_TRANS_DRV_TRN` (40°)** — direct bearing + spot-turn, speed 0 during turn, no reverse-flip. **NOT "any angle": below 40° it arcs, not pivots.** And only with `RO_DECEL_LIM > 0 && RO_JERK_LIM > 0` (else NaN speed ≳2 m). Only active when the companion sets the position bit. |
| Minimal patch | **0 firmware LOC** to enable the behavior; **~15-25 LOC** stock-PosControl for optional default-off `RD_OFFB_POS`; **~15-25 LOC** stock-PosControl for commanded final-yaw. **Never overlay fork PosControl.** |
| Final yaw (line-direction alignment) | **Not supported today** (`yaw` forced NAN) — needs the Phase C extension, or a companion target-point trick. |
| "Rollback without re-flash" | Only means "companion stops setting the position bit." A firmware `RD_OFFB_POS` flag only matters to *ignore* a companion that sets it. |
| Biggest risks | (1) NaN speed if jerk/decel ≤ 0 [G0]; (2) velocity↔position switching ~500 ms flag lag + VelControl holding the stale setpoint [§7] — bench-test both in isolation first. |

---

## 11. Out of scope
Velocity-mode line tracking math; path engine / run splitting / profile classification; reviving the DriveModes dispatcher; fork PosControl; any `RO_*`/`RD_*` gain retune beyond G1; ArduRover.

## 12. Fallback
If the bench make-or-break (§8 step 3) shows PosControl does not stop/pivot cleanly via OFFBOARD, fall back to companion velocity-only with an along-track completion test and a bounded fail-not-loop watchdog — structural ceiling ~2 cm, the known-degraded option.
