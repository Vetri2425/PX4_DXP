# D0 — Runtime-entry pivot make-or-break bench

**Gate for `docs/RUNTIME_ENTRY_VELOCITY_PLAN.md`. No build (D1–D4) starts until this PASSES.**

**Question:** can velocity-OFFBOARD spot-turn ~150–180° from a **dead stop** cleanly (no reverse-flip, no oscillation), the way the runtime entry will need to?

**Sole-publisher rule:** publish the path through `path_publisher_node` → `/path` → RPP → `twist_to_setpoint`. Do **not** run `offboard_test.py` or any second publisher on `/mavros/setpoint_raw/local` (that was the invalid Jul-11 bench).

---

## 0. Pre-check (optional, cheap, disarmed-safe-ish)
`spin_in_place_test.py` spins in place from rest and reports final heading error. A clean 180° there is *encouraging* but NOT the gate — it skips the drive-to + stop seam. The hairpin below is the real test.

## 1. Setup
- Open space, ≥ 4 m clear ahead + room to hairpin. Operator on the kill switch.
- **Park the rover facing north-ish** so A→B is an easy start and the **169° turn at B is the thing under test**. (To also stress the start turn, park facing south — both turns then exercise the primitive.)
- RTK **FIXED**. Confirm: `ros2 topic echo /mavros/state --once` and telemetry `gps_fix_name`.
- Copy the path: `scp tools/d0_entry_pivot/hairpin.csv flash@192.168.1.102:~/PX4_DXP/tools/d0_entry_pivot/`

## 2. Force segment profile (so the corner does STOP→ALIGN, not a smooth arc)
```bash
ros2 param set /rpp_controller_node tracking_profile segment
```

## 3. Record the bag
```bash
ros2 bag record -o ~/bags/d0_$(date +%Y%m%d_%H%M%S) \
  /mavros/local_position/pose /mavros/local_position/velocity_local \
  /rpp/velocity_ned /rpp/yaw_rate_body /mavros/setpoint_raw/local \
  /rpp/debug /rpp/segment_debug /mavros/state
```

## 4. Run
1. Publish the path (anchors at live pose, spray irrelevant/off):
   ```bash
   ros2 run <pkg> path_publisher --ros-args \
     -p mission_file:=$HOME/PX4_DXP/tools/d0_entry_pivot/hairpin.csv \
     -p auto_origin:=true -p frame_id:=local_ned
   ```
2. Confirm `/rpp/segment_debug` shows the 3-point path loaded, state `TRACK_SEGMENT`.
3. Arm + switch OFFBOARD by your normal bench method (server mission start, or manual arm + `set_mode OFFBOARD`). Setpoints must already be streaming (they are — twist runs at 50 Hz).
4. Watch it: drive to B → brake → **spin ~169°** → drive out to C → stop.

## 5. PASS / FAIL bars
Watch `/rpp/segment_debug` (segment_state) and `/rpp/debug` live, then confirm in the bag.

| Check | PASS |
|---|---|
| Reaches B and **brakes to a confirmed stop** before turning | segment_state hits `CORNER_STOP (5)`; measured speed < 0.02 m/s with dwell **before** `CORNER_ALIGN (3)` |
| **No reverse-flip** during the turn | forward component of `/rpp/velocity_ned` vs pose yaw stays ≥ 0 the whole pivot; no sudden 180° bearing jump in `/mavros/setpoint_raw/local` |
| **No oscillation** | heading error decreases monotonically to tolerance — no back-and-forth swing across the target |
| Settles and launches | `CORNER_ALIGN` releases at |heading_err| ≤ `segment_heading_tolerance_deg` (2°), then `TRACK_SEGMENT` out to C |
| Turn magnitude | pose yaw actually changes ~169°, short way |

## 6. Verdict → record in the plan doc
- **PASS** → velocity mode is the whole-mission answer. Unblock **D3 → D1 → D2 → D4**.
- **FAIL** (reverse-flips, oscillates, or can't settle) → scope position mode for the **entry pivot only** (plan §9 / `OFFBOARD_POSITION_MODE_PLAN.md`, prove G1-OFFBOARD first). Do **not** touch corners or marking.

## 7. Restore
```bash
ros2 param set /rpp_controller_node tracking_profile auto
```
Paste the bag name + verdict into `docs/RUNTIME_ENTRY_VELOCITY_PLAN.md` §5.

---
*Substitute `<pkg>` and the exact node/param names for your launch setup; `rpp_pipeline` entrypoint is `rpp_start.sh`, not the launch file.*
