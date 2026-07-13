# Implementation Plan — Mission Alignment + Firmware Runtime Entry

**Status:** Epic 1–2 **done**. Auth **done**. Epic 3 active. **G1-OFFBOARD still OPEN** — next work is minimal position bridge (§5.4), then prove G1 *through* it. Do not re-fight the heartbeat.  
**Date:** 2026-07-11 (Epic 3 rewrite; §5.0 corrected after failed parallel-publisher bench)  
**Base branch:** `test/colinear-fix` (controller pin `cd44884`; do not re-land July RPP corner regressions)  
**Alignment reference (audit only):** `fix/runtime-entry-stop` — placement/auth ideas only; **never** densified entry Path  
**Firmware background:** `docs/OFFBOARD_POSITION_MODE_PLAN.md`  
**Mobile:** `rover-three-wheel-native` / `Path-Alignment` @ `3e222be` — auth contract live  

### Implementation progress
- **Epic 1 (done):** no ref double-scale; scale gate; placeholder `origin_gps` reject; staging `placement_mode` + `origin_gps`.
- **Epic 2 (done):** live EKF placement `P_live = P + L − R_anchor`; fail-closed RTK/pose/skew; surveyed ⊥ `auto_origin`.
- **Auth (done):** password → session; Socket.IO `auth:{token}`; machine token for bag; mobile wired.
- **G0:** CLEAR (`RO_JERK_LIM` / `RO_DECEL_LIM` > 0).
- **G1 stop quality (AUTO):** PASS on log_23 / log_24 @ 5 cm — **not** OFFBOARD proof.
- **G1-OFFBOARD:** still **OPEN**. 2026-07-11 live attempts **invalid** (twist co-publisher contamination). Not a firmware FAIL.
- **Ops bug fixed (commit):** `bff32ae` — `SuccessExitStatus=143 130` so `systemctl stop rpp-pipeline` can stick. Needs Jetson `daemon-reload` / deploy; unrelated to PosControl.
- **Epic 3 sequencing corrected (2026-07-11 evening):** do **not** prove G1 by stopping the heartbeat and bolting `offboard_test` beside it. Prove G1 **through** a minimal position-capable twist bridge (§5.4) — that *is* the first Epic 3 code.
- **A2/handoff:** still required for large-yaw G3 proof.

---

## 0. Goal

For a GPS-surveyed mission:

1. Align DXF → survey metric frame *(done)*.
2. Place into live EKF at start *(done)*.
3. **Runtime entry:** firmware OFFBOARD **position** go-to → first WP / first PRE *(Epic 3)*.
4. Handoff → baseline **velocity** RPP for PRE→MARK→AFT *(frozen knobs)*.

Spray **OFF** through entry. No companion densified entry Path.

---

## 1. Locked decisions

| # | Decision |
|---|---|
| D1 | Baseline RPP only — no July corner doublings / hold / completion-latch merge. |
| D2 | Entry ≠ RPP densified Path (anti-pattern). |
| D3 | Entry target = first WP (extensions off) or first **PRE** (extensions on). |
| D4 | Transport = firmware OFFBOARD **position** single XY (`NAV_ACC_RAD` stop). |
| D5 | Field: try 5 cm then 2 cm; if 2 cm hunts → C1 keep 5 cm. |
| D6 | Marking = velocity OFFBOARD + baseline RPP only. |
| D7 | Reference branch = audit only for placement/auth; not entry Path. |
| D8 | Optional second XY for heading (A2); v1 may skip and let RPP run0 pivot. |

---

## 2. What’s left (why Epic 3 exists)

| Layer | Status |
|---|---|
| Plan-time CRS | Done |
| Live EKF placement | Done |
| Client auth | Done |
| PosControl stop quality (AUTO) | Proven ~5 cm |
| OFFBOARD position **ingestion** | **Unproven** — make-or-break before ship |
| Companion entry state machine | **Missing** — Epic 3 |
| Velocity handoff → mark | **Missing** — Epic 3 |

---

## 3. Architecture (product flow)

```
 PLAN (done) → LOAD staged GPS_SURVEYED
      ↓
 START: place(source → live EKF)     [Epic 2 — done]
      ↓
 ENTRY_POSITION: stream position XY = entry_target, spray OFF
      ↓  (optional ENTRY_HEADING: second XY down-line)
 HANDOFF: clear position bit → velocity stream → publish full mark Path
      ↓
 MARKING: baseline RPP (unchanged)
```

| Concern | Owner |
|---|---|
| Free-space chase + stop | Firmware `DifferentialPosControl.goToPositionMode` |
| Position bit + XY stream | Companion setpoint bridge |
| Arrival / timeouts / abort | Companion |
| Mark track + corners | Baseline RPP |

---

## 4. Epics 1–2 (done — keep as contract)

Placement invariant (locked by tests):

```text
R_anchor = latlon_to_ned(rover_lat, rover_lon, anchor_lat, anchor_lon)
P_live   = P + L - R_anchor
```

`GPS_SURVEYED` ⊕ `auto_origin` → 422. Staging carries `placement_mode` + `origin_gps`. Do not reopen unless regression.

---

## 5. Epic 3 — Firmware runtime entry (CORE — build this)

### 5.0 Gate — corrected approach (2026-07-11 field lesson)

| Gate | Need | Status |
|---|---|---|
| **G1-OFFBOARD** | OFFBOARD + position mask; stop @ 5 cm, no hunt; **ULog has populated `rover_position_setpoint`** | **OPEN** — prior live runs **invalid** |
| First code | Minimal §5.4 position path **inside** the setpoint bridge (flagged) | **Next** |
| Full ENTRY_* lifecycle | Only after G1 PASS through that bridge | Blocked on G1 |
| C3/C4 helpers | `entry_target_from_plan`, MissionState table | OK anytime (survive fallback) |

#### Why “stop rpp + run offboard_test” is the wrong bench

`twist_to_setpoint` is the **permanent OFFBOARD heartbeat** — by design it owns `/mavros/setpoint_raw/local` at 50 Hz. Field attempts showed:

1. Stopping the pipeline fights the robot (and briefly stressed RTK 6→4). Even after `bff32ae` makes stop stick, a **second publisher** beside the heartbeat is a two-publisher conflict (bag: interleaved velocity+yaw with position msgs) — G1 contaminated, **not** a PosControl verdict.
2. Plan §5.4 already said: one owner of the topic. Entry must emit position **through** the bridge (mutex / mode switch), not via a bolted-on test node.

**Correct G1 path:** build a **minimal, flagged** position emit in `twist_to_setpoint` (or sibling under the same service) → exercise **one** publisher → bag ULog → hard-bar `rover_position_setpoint`. That proves ingestion **and** de-risks the production bridge.

**Anti-pattern (do not repeat):** thrash `systemctl stop` on a live armable robot to silence the heartbeat for a parallel `offboard_test`.

#### Minimal bridge slice for G1 (scope lock)

| In | Out |
|---|---|
| Flag / topic: latch one NED XY; publish position `PositionTarget` (`FRAME_LOCAL_NED`, position type_mask) at 50 Hz | Full ENTRY_* MissionState machine |
| Clear flag → resume velocity from RPP (or hold zero) | Densified entry Path |
| Server or CLI sets the latched XY (relative 1–2 m or surveyed point) | Blind merge of July entry stack |
| Disarmed-safe dry stream optional; armed G1 only with operator present | Fighting `Restart=` / drop-ins as the test method |

Prior art for **message shape only:** `src/offboard_test.py` `_make_position_setpoint` — not the production publisher.

#### G1-OFFBOARD pass checklist (through the bridge)

| Check | Pass |
|---|---|
| Sole publisher on `/mavros/setpoint_raw/local` during the run | Required |
| Mode = OFFBOARD | Required |
| **`rover_position_setpoint` populated** (non-NaN XY while approaching) | **Hard bar** |
| Final dist ≤ `NAV_ACC_RAD` (0.05), no hunt / no large reversals | Required |
| Hold stable, throttle ~0 | Required |

**FAIL →** C3/C4; do not expand lifecycle on hope.  
**PASS →** grow bridge into ENTRY_POSITION + handoff (§5.2–5.7).

#### Related ops note (not G1)

`bff32ae` (`SuccessExitStatus=143 130` on `rpp-pipeline.service`): real bug — SIGTERM exit 143 was `Failed` → `Restart=on-failure` resurrected every stop. Deploy via `daemon-reload` when convenient. Does **not** change the G1 architecture lesson above.

Known interface traps: `FRAME_BODY_OFFSET_NED (9)` rejected — use `FRAME_LOCAL_NED`; verify NED vs ENU in ULog.

### 5.1 Companion contract

| Field | Value |
|---|---|
| Mode | OFFBOARD |
| Mask | **position** enabled while in entry |
| Setpoint | Local NED XY = `entry_target` @ ≥ rate for `COM_OF_LOSS_T` |
| Velocity / yaw | Ignored while position owns the leg |
| Spray | Forced OFF entire entry (+ optional A2) |
| Arrival | `hypot(pos − target) ≤ NAV_ACC_RAD` **and** speed ≈ 0 for dwell T |
| After arrival | Clear position bit → velocity stream → publish **full** placed mark Path |

Never assert position bit without a finite XY. Never leave stale/NaN XY across the switch (VelControl can hold last velocity — see firmware plan §7).

### 5.2 State machine

```text
IDLE
  → PLACING            # Epic 2 resolve (already on start_async)
  → ENTRY_POSITION     # stream position XY = entry_target
  → [ENTRY_HEADING]    # optional A2 — second XY down first mark
  → HANDOFF_VELOCITY   # switch mask; publish mark Path; RPP wakes
  → MARKING / RUNNING  # baseline today
  → COMPLETED | ABORTED | ERROR
```

**v1 default:** skip `ENTRY_HEADING` if `|yaw_err| < RD_TRANS_DRV_TRN` (~40°) **only as a starting policy**.  
**Field risk:** log_24’s clean 163° spot-turn was **AUTO PosControl** — it does **not** transfer to velocity-RPP run0 from a dead stop (P4 freeze &lt;1 cm/s, reverse-flip history, BUG-T3). If arrival heading is arbitrary, handoff can re-hit the fragile velocity pivot.  
**Rule:** keep A2/`ENTRY_HEADING` implemented and **ready to force ON for G3**; do not call G3 PASS until at least one large-yaw-error entry→handoff bag is clean (or A2 is proven on that bag).

### 5.3 Entry target (was Epic 4 — in-scope here)

```text
entry_target_from_plan(placed_wps, spray_flags, extensions_enabled) → (n, e)
```

| Extensions | Target |
|---|---|
| Off | `placed_wps[0]` |
| On | First **PRE** point of first mark span (not MARK start) |

Expose on start: `entry_target_ned`, `placement_translation`, `placement_mode`, `entry_phase`.

### 5.4 Setpoint bridge (companion) — first Epic 3 deliverable

Today `twist_to_setpoint_node` is **velocity-only** and is the permanent heartbeat. Epic 3 / G1 need it (or a sibling under the same service) to own **position** when flagged:

| Option | Notes |
|---|---|
| **A (required path)** | Twist (or sibling) latches one NED XY; publishes position mask at 50 Hz; clear → velocity again. **This is how G1 is proven.** |
| **B (rejected for G1)** | External node on same topic while twist runs — field-proven two-publisher fail |

Prior art for message shape only: `src/offboard_test.py` `_make_position_setpoint`.

**Hard rule:** only one publisher owns `/mavros/setpoint_raw/local` at a time.

### 5.5 Arrival, timeout, abort

| Event | Action |
|---|---|
| Arrived (dist + speed + dwell) | → HANDOFF |
| Entry timeout / max distance | Abort = existing mission abort (stop-path / MANUAL / disarm policy) |
| E-stop mid-entry | Same estop path; spray already OFF |
| FCU disconnect / RPP unhealthy mid-entry | Safety abort (extend current watchdog to ENTRY_* states) |

### 5.6 `MissionState` blast radius (mandatory table)

New states touch spray, e-stop, telemetry, bridge_health, path load guards, auth password-change allowlist. **Do not silent-add enum values.**

| Consumer | ENTRY_POSITION / ENTRY_HEADING | HANDOFF |
|---|---|---|
| Spray auto | **OFF / deny** | deny until MARKING/RUNNING |
| `/load-to-controller` | **409** (busy) | **409** |
| Password change | **409** | **409** |
| E-stop / abort | **allowed** | **allowed** |
| Telemetry `mission_state` | show new names | show |
| Bridge health | treat as active mission | active |
| Auto-complete (RPP DONE) | **ignore** | **ignore** until MARKING |

### 5.7 Work breakdown (corrected order)

1. **Minimal §5.4 position path in twist bridge** (flagged; sole publisher).  
2. **G1-OFFBOARD through that bridge** — hard bar `rover_position_setpoint`; PASS/FAIL in this doc.  
3. If PASS: grow into ENTRY_POSITION + arrival + handoff + MissionState table.  
4. `entry_target_from_plan` + tests (can parallelize with 1–2; survives C3/C4).  
5. **ENTRY_HEADING/A2** stubbed; forced for large-yaw G3.  
6. Surveyed-only entry default (open Q).  
7. G2/G3 field campaign.  
8. If G1 FAIL: C3/C4 — do not expand position lifecycle.

### 5.8 Explicit non-goals (Epic 3)

- Densified companion entry Path / RPP acquisition leg.  
- Firmware position for mark corners or final stops.  
- Blind merge of `fix/runtime-entry-stop` entry/RPP July stack.  
- Claiming 2 cm stop before G2.  
- Using position mode for the whole mission.

### 5.9 Acceptance (Epic 3)

- [ ] G1-OFFBOARD ULog **through the position bridge** (sole publisher): `rover_position_setpoint` + clean stop @ 5 cm.  
- [ ] Surveyed start: entry → stop at PRE/WP0 → handoff → mark; spray OFF in entry.  
- [ ] No densified entry Path on `/path` during ENTRY_*.  
- [ ] Handoff: velocity mask + full mark Path; xtrack class baseline on square/line.  
- [ ] Abort/estop mid-entry safe.  
- [ ] LOCAL_NED auto-origin square still works (no surveyed entry required).  
- [ ] Corner knobs unchanged (`slowdown=0.50`, `brake_cap=0.08`, no `corner_stop_hold_s`).

---

## 6. Field gates & contingencies

| Step | Action | Pass |
|---|---|---|
| G0 | `RO_JERK_LIM`, `RO_DECEL_LIM`, `RO_SPEED_LIM` > 0 | Done |
| G1-AUTO | QGC go-to @ 5 cm | Done (log_23/24) — **not sufficient alone** |
| G1-OFFBOARD | Position via twist bridge (sole publisher) @ 5 cm; `rover_position_setpoint` hard bar | **OPEN** |
| G2 | Same @ 2 cm | Pass **or** C1 keep 5 cm |
| G3 | Full entry → handoff → one square/line | Mark xtrack <2 cm RMS; spray OFF in entry |

| If G2 hunts | Action |
|---|---|
| **C1** | Keep `NAV_ACC_RAD=0.05`; RPP cleans in PRE |
| **C2** | Companion arrival radius 5 cm with logging |
| **C3** | Coarse velocity chase **without** densified Path |
| **C4** | Disable firmware entry until PosControl OFFBOARD proven |

---

## 7. Open questions (resolve in Epic 3 / G1)

1. Always run A2 heading XY, or only if `|yaw_err| > RD_TRANS_DRV_TRN`?  
2. Entry-only for `GPS_SURVEYED`, or also LOCAL_NED when far from wp0?  
3. Max entry distance / timeout before abort?  
4. Bridge option A vs B (who owns `/mavros/setpoint_raw/local`)?  
5. Point-missions: same entry pipeline or later?

---

## 8. Doc / memory when Epic 3 ships

- Update `CLAUDE.md` / memory: firmware entry live; densified entry Path = **Rejected**.  
- Tracker: placement items closed; entry = firmware go-to.  
- Keep `OFFBOARD_POSITION_MODE_PLAN.md` as firmware background.

---

## 9. One-line summary

**Place (done) → minimal position bridge → G1-OFFBOARD through that bridge → ENTRY lifecycle → velocity handoff → baseline RPP; never densified entry Path; never two publishers on the setpoint topic.**
