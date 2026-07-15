# Spray Controller V2 — Phase A Handoff & Next-Phase Plan

**Status:** Phase A code **committed** on `Upgrade_Spray` @ `5e7927f`. **Field validation PENDING** (Jetson was offline 2026-07-15/16 — not yet deployed).
**Plan of record:** `docs/Architecture/SPRAY_CONTROLLER_V2_PLAN.md` (Rev 3). This doc is the running state + handoff so no context is lost between sessions.

---

## What Phase A delivered (Mode-1 continuous only, behavior-preserving)

| File | Role |
|---|---|
| `src/spray_fsm.py` | Pure actuator command FSM (§4): 7 states, cmd_seq stale-ack rejection, bounded-exponential RECOVERY backoff, **ack-timeout** (a never-resolving MAVROS reply can't wedge the FSM). `spraying == ON_CONFIRMED` only — no optimistic-ON, by construction. |
| `src/spray_session_config.py` | One versioned mission-config schema (§3), fail-static parse, node-local geometry fingerprint. |
| `src/spray_status.py` | Typed `SpraySessionStatus` + crash-proof JSON boundary (§6). Optional floats never inf/nan; non-serializable `mode_state` values/keys can't crash the publish (defect-#1 class closed). |
| `src/spray_controller_node.py` | Rewired onto FSM/config/status. External interface preserved. |
| `rpp_start.sh` | §9 failure isolation: CRITICAL (twist_to_setpoint, rpp_controller) vs AUXILIARY (spray_controller, xtrack_logger). Aux crashes never trip the global fail threshold / never restart the pipeline. Aux backoff is **non-blocking** (polled schedule), monotonic `/proc/uptime` clock. |

**Two intended external changes** (everything else identical to pre-V2):
1. `/spray/state` now reflects **confirmed-ON only** (was optimistic-on-dispatch).
2. New additive **`/spray/status`** topic (`std_msgs/String`, JSON).

**Tests:** 161 unit tests pass on Mac (node tests use rclpy stubs; pure modules run without ROS).

---

## Tomorrow's field test (Phase A parity)

**Goal:** confirm Mode-1 continuous spray is at **parity** with pre-V2, and the new FSM / `/spray/status` / node-isolation work. Not a tuning session.

**Dev pre-checks on the Jetson (before the operator run) — NOT on the operator sheet:**
- Deploy: `ssh flash@192.168.1.102`, `git checkout Upgrade_Spray && git pull` → `5e7927f`, `sudo systemctl restart rpp-pipeline`; `pgrep -af` all 3 nodes up.
- `ros2 topic echo /spray/status --once` → valid JSON, `fsm_state=OFF_CONFIRMED`, `spraying=false`, `gps_fix_name="not_evaluated"`.
- Isolation smoke: `pkill -f spray_controller_node` → pipeline does NOT restart; twist/rpp PIDs unchanged; spray respawns; journal shows "auxiliary … isolated restart".
- Dry-run (no paint): `POST /api/spray/test` on/off; disarm-mid-ON forces OFF.

**Operator field checklist (the printable A4 sheet):** deploy (pull + restart) → power-up + RTK_FIXED → arm/OFFBOARD/start → watch spray START at line start / STOP at line end, no corner leak, line quality = parity → E-stop and disarm both force spray OFF.

**Pass gate:** Mode-1 line quality matches pre-V2, no `/spray/status` publish crash / pipeline restart, spray fails safe OFF on disarm/E-stop.

---

## ➡ NEXT PHASE = Phase B — RTK/GPS fix-quality gate (§7.6)

Start once Phase A field-passes.

- Subscribe `/mavros/gpsstatus/gps1/raw` (GPSRAW); read `fix_type`. Require `fix_type >= spray_min_fix_type` (default **6 = RTK_FIXED**; 5 = RTK_FLOAT).
- **Staleness is a distinct failure** from bad-fix: `age(GPSRAW) > gps_fix_timeout_s` (~2.0s, looser than the 0.5s pose/vel gates) → fail with reason `"gps stale"`.
- **Asymmetric hysteresis:** drop instantly on the unsafe edge (no debounce); recover only after `gps_recover_hold_s` (~1.0s) continuously `>= threshold`.
- Mirrors `rpp_controller_node.py` P0.3. `SpraySessionStatus.gps_fix_ok`/`gps_fix_name` already exist — Phase A hardcodes `gps_fix_ok=True, gps_fix_name="not_evaluated"`; **Phase B makes them real** and wires the gate into the §5 gate stack (first-failing-gate reason surfaces to `safety_reason`).
- Bench-test flapping fix-type + a stale-GPSRAW case; field-validate an RTK dropout mid-mark forces OFF and recovers correctly.

**Then, in order (plan §12):** Phase C dash mode → Phase D point mode (two-sided: path_engine + node) → Phase E speed-proportional flow → Phase F full field-validation pass.

---

## Deferred review findings (logged, not blocking Phase A)

Surfaced by the max-effort review; fixed the confirmed high/medium ones, deferred these:

1. **Reassert-ON async race (highest-priority deferred).** `_reassert_on_command` bypasses the FSM; a late in-flight reassert ON can re-energize the actuator after an FSM OFF confirms (physical=ON while `/spray/state=false`). Pre-V2 + inherent to command-truthful control with no actuator feedback. A real fix needs physical feedback (`physical_feedback_supported=False` today).
2. **OFF-ack timeout escalates backoff** the same as a hard NACK → safety OFF-retry can stretch toward `backoff_max` (5s) under sustained MAVROS comms loss. In-spec; consider a lower OFF-retry cap if field data shows it matters.
3. **FSM monotonic vs ROS clock** diverge under `use_sim_time` (bench replay only; production is real-time).
4. **Aux PID-reuse** in the (≤30s) backoff window could make a dead node look alive (low probability).

---

## Branch / workflow state

- Active dev: **`Upgrade_Spray`** (Phases B–F land here). Frozen backup: `Upgrade_extensions`. Trusted merge target: `baseline_master`.
- Merge `Upgrade_Spray` → `baseline_master` only after ALL phases are verified, then branch the next work off baseline. (See memory `branch_lineage_workflow`.)
- All three branches were at `528ad6f`; `Upgrade_Spray` is now ahead at `5e7927f` (Phase A).
