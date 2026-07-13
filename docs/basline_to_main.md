# Baseline → Main — Audited Status (not a blind rebuild list)

> **Audited 2026-07-13** against `test/colinear-fix` @ `7aa005f` — every line below was verified in the
> code on THIS branch (and mainline "absent" claims confirmed via `git merge-base --is-ancestor`).
> The old version of this file was a plain "features on main not here" scratch list; it was stale in both
> directions. This version records what is **actually** done / partial / pending on the baseline.

## Control mode (was stale — clarified)

The runtime drive **and** the two-phase runtime entry are **velocity-mode OFFBOARD**, not position control:

```
rpp_controller_node → /rpp/velocity_ned → twist_to_setpoint_node
  → /mavros/setpoint_raw/local (PositionTarget, type_mask = VELOCITY 3527) → PX4
```

`setpoint_raw/local`/`PositionTarget` is only the **transport** — `twist_to_setpoint_node.py:66` masks out
every position/accel/yaw bit and streams **vN/vE/vD only**. There is **no position controller** in the live
path. `docs/OFFBOARD_POSITION_MODE_PLAN.md` is a **deferred future plan**, not the current mechanism — do not
describe the rover as running a position-mode entry/stop.

---

## Status legend
✅ present & verified · 🟡 partial / differs from main · ⬜ genuinely absent (on main's lineage) · ⛔ intentionally NOT rebuilt

### Mission / placement
1. ✅ GPS-surveyed placement into live EKF frame — `mission_placement.py:131` (`P_live = P + L − R_anchor`)
2. ✅ Runtime entry transit (two-phase, spray-OFF lead-in) — `offboard_controller.py:373`, `main.py:366`
3. ⛔ Runtime-entry densified at 5 cm — **rejected here**: entry is 2-point by design (`offboard_controller.py:386`)
4. ✅ Pre-align before marking (`entry_prealign_enabled`, default **False** → footgun, see Attention) — `rpp_controller_node.py:338`
5. ✅ Affine / ref-point scale gate (no double-scale) — `path_engine/engine.py:422`
6. ✅ Mission clear API `POST /api/mission/clear` (ABORTED/ERROR handled) — `routes/mission.py:137`
7. ⬜ Mission-mode ops: skip / restart / operation coordinator / terminal cleanup / event journal — commit `a31f36d` NOT an ancestor

### Point navigation
8. ⬜ Point-navigation stack (Task_01) — no point-mission concept; only DXF/polyline tracking
9. ⬜ GPS lat/lon point-mission CSV ingest — only QGC-waypoint→polyline exists (`path_manager.py:144`), no lat/lon CSV
10. ⬜ Point-mission plan-and-stage E2E — DXF/path only (depends on 8/9)

### Path / geometry
11. 🟡 Uniform ≤5 cm densification — **not uniform**: MARK = 5 cm, TRANSIT/connectors = 15 cm (`straight_line.py:52`), no global enforce pass

### Spray
12. ⬜ Three spray modes (continuous / dash / point) — **only continuous** exists (`spray_controller_node.py:74`)
13. ⬜ Per-path spray-mode API + sidecar + hot-apply — only per-entity on/off sidecar (`path_manager.py:273`)
14. ⬜ Production spray architecture: path identity / fingerprint binding — controller comment confirms fields "do not exist here" (`offboard_controller.py:163`)
15. ✅ Actuator state machine + terminal safety (Task 17/18/19) — **present** (`spray_controller_node.py:803`)
16. ⬜ GPS_SURVEYED runtime safety gate for spray — **absent → SAFETY GAP** (`spray_controller_node.py:775`, no placement-quality check)
17. ✅ Controller-owned anticipatory latency — `spray_controller_node.py:351` (`anticipatory_margin_m`)
18. ✅ Startup OFF-reconciliation / recovery retries — `spray_controller_node.py:504`
19. ⬜ Continuous-mode param-contract hardening (gps_*/obstacle_*/point_*) — those params don't exist in this controller (N/A here)

### Joystick / manual
20. ⬜ Production virtual joystick (V2) — no `joystick_controller.py`, no manual-control socket event
21. ⬜ MANUAL_CONTROL float32 path + rejection logging — absent
22. ⬜ Throttle/steering tuning caps — absent (depends on 20)
23. ⬜ Corner-stop / reverse-brake yaw hold in twist bridge — only a zero-vel yaw-continuity hold exists (`twist_to_setpoint_node.py:250`), not the feature

### Auth / API / telemetry
24. ✅ Local password authentication — `auth.py:133` (PBKDF2) + scoped machine tokens
25. ✅ Auth disable bypass `ROVER_AUTH_DISABLED` + Socket.IO honors it — `config.py:134`, `auth.py:415`
26. ⬜ Swagger `X-Rover-Token` security scheme — token works, not surfaced in OpenAPI
27. ✅ Read-only rover monitoring telemetry API — `routes/telemetry.py:20`
28. ⬜ Activity log CSV export — `/activity` is JSON only (`routes/system.py:63`)
29. ⬜ Socket.IO AsyncAPI contract docs — absent
30. ⬜ GCS migration compatibility matrix — absent
31. ⬜ Telemetry `measured_speed_m_s` — only `speed_m_s` (`models.py:86`)
32. ⬜ GPS lat/lon 8-dp — **non-gap**: raw float64 passthrough already exceeds 8 dp (`ros_node.py:397`), just no explicit rounding

### RTK / NTRIP
33. ✅ LoRa/NTRIP RTCM self-healing (backoff reconnect) — `ntrip_rtcm_node.py:445`, `lora_rtcm_node.py:179`
34. ✅ NTRIP watchdog / reconnect-age — `rtk_manager.py:224` (`last_frame_age_s`, ≤10 s gate)
35. ⬜ 3D GPS label / plaintext-auth rejection — absent

### Capture / stability
36. ⬜ Bag capture race fix + GPS nuisance e-stop debounce — commit `948c29f` NOT an ancestor; generic pose-stale watchdog only
37. ✅ Complete mission capture evidence bundle — `bag_autorecord.py:255` (SHA256 bundle + manifest + retention)
38. 🟡 Fail-closed pack — baseline-level watchdog + D3 latch present; mainline spray-backend commits (`e3aae88`/`35a1518`) NOT ancestors
39. ✅ Final DONE gated on **measured** coast-down (not position) — `rpp_controller_node.py:1517` (`_hold_at_completion`)

### Intentionally OUT of "rebuild as-is"
- ⛔ #3 densified entry (velocity 2-point entry is the design)
- ⛔ #40 corner-stop param retunes — baseline knobs stay frozen (`0.50` / `0.08` / no hold); invariants I2/I4
- Docs-only / rename / PWM ON-range bump-then-revert

---

## Corrections vs the old list (doc was stale both ways)
- **Spray safety layer (15/17/18) IS present** — was mislabeled pending. Only the *modes/identity/param-contract* half (12/13/14/19) is absent.
- **RTK reliability core (33/34) IS present** — old note hedged "may not exist."
- **Capture bundle (37) is fully present**, not partial.
- **#32 is a non-gap** — precision already exceeds 8 dp.

## Needs attention (promote above the P1–P5 handoff)
- **🔴 Spray GPS-quality safety gap (#16)** — `_auto_safety_status` never checks GPS_SURVEYED provenance; a degraded-GPS mission can still spray. Cheap to close, real risk.
- **🟠 `entry_prealign_enabled` resets to False on every rpp-pipeline restart (#4 / handoff P4)** — surveyed entry pivot silently disappears until re-set.
- **🟠 Densification is 5 cm / 15 cm split (#11)** — not uniform; check during the P1 arc-flow investigation (connector spacing may feed the stop-pivot classification).

## Bottom line
Line + square runtime entry, placement, auth, completion latch, RTK core, spray safety layer, and recorder are
**done and field-proven** on this baseline. Point-nav, spray *modes*, joystick, mission-mode ops, and the
telemetry-polish backlog genuinely still live on main's lineage. First engineering item stays **P1 (arc flow)**.
Do **not** rebuild #3 or #40.
