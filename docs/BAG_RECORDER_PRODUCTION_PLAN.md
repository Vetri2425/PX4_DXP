# Implementation Plan — Production-Grade Auto Bag Recorder + Post-Mission Analysis

**Base branch:** `test/colinear-fix` (controller pin `cd44884`).
**Date:** 2026-07-13
**Reference (ideas only, do NOT blind-merge):** `main` / `fix/runtime-entry-stop` — `tools/bag_autorecord.py` (1077 L), `server/mission_debug_capture.py` (220 L), `config/rosbag_qos_overrides.yaml`, `bag-autorecord.service`, `tools/analyze_bags_quick.py`. Take the sound parts; reject the coupling (below).

---

## 0. Where we actually are (verified on this tree + Jetson)

- Baseline **already ships a working auto-recorder**: `tools/bag_autorecord.py` (201 L) + `bag-autorecord.service`, **enabled and active** on the Jetson, writing to `~/bags_jet/`. It polls `GET /api/mission/status` (read-only) and runs `ros2 bag record` while the mission state is active, finalising on terminal state or after `BAG_API_GRACE_S` if the API goes unreachable. Its own process group makes SIGINT finalise the bag cleanly.
- This is the **correct architecture** — a pure poll/observer, no server→recorder handshake. It is just **bare**: no manifest, no integrity, no disk management, no crash recovery, no QoS override, thin topic list, and no post-mission analysis.
- D0/D3 used **manual** `ros2 bag record` only because those benches bypass the mission API (raw `/path` + manual arm), so the poll never saw a "running" state.

**This plan hardens the existing decoupled recorder to production grade and adds behaviour analysis. It does not rebuild it and does not adopt main's coupling.**

---

## 1. The load-bearing invariant — the recorder is a PURE OBSERVER

The recorder observes; the rover never depends on it.

- **R1 — never block or fail a mission.** No code path in the mission lifecycle may wait on, require, or error because of the recorder. **Explicitly REJECTED:** main's `server/mission_debug_capture.py` `begin_capture()` writes a request file and *blocks on an ack with a timeout*, raising `CaptureUnavailable` → this is the 2026-07-02 "recorder did not acknowledge within 3s" **503 storm on mission start**. We keep the pull model only.
- **R2 — never fill the disk.** Preflight free-space + retention rotation + hard min-free floor. A full disk must degrade the recorder (skip/trim), never wedge `rover-server` / `rpp-pipeline` / logging.
- **R3 — always finalise.** API-grace finalise, post-mission tail, and crash reconciliation on restart. A power-cut bag is marked, never silently corrupt.
- **R4 — capture completeness.** Latched (TRANSIENT_LOCAL) topics — above all `/path` — must actually land in the bag.
- **R5 — mask secrets.** No token / password / NTRIP credential ever reaches a manifest, log line, or captured `statustext`.

Also rejected from `main`: the 15-file coupling of `a158fdb` into `offboard_controller.py` / `routes/mission.py` / `sockets/events.py` / `main.py`, and the control-dir request/ack/cancel/nack filesystem protocol. None of it is needed for a pull-based observer.

---

## 2. Keep (already sound on baseline)

Poll `GET /api/mission/status` (read-only) · start on active state, stop on terminal · own process-group SIGINT finalise · `BAG_API_GRACE_S` degrade-to-finalise when the API is unreachable · machine-token auth (`X-Rover-Token`) · systemd unit with hardening.

---

## 3. Gaps to close (the work — each independently landable)

### G1 — QoS override so latched `/path` is actually captured  **[highest priority]**
`/path` (and `/rpp/conditioned_path`, `*/identity`) are published **TRANSIENT_LOCAL** at mission start, *before* the recorder detects the state change and subscribes. A default `ros2 bag record` subscribes VOLATILE and **misses the already-latched message** — so the bag has no commanded path, and cross-track/stop analysis is impossible. Add `config/rosbag_qos_overrides.yaml` mapping the latched topics to `durability: transient_local, reliability: reliable`, and pass `--qos-profile-overrides-path` in `Recorder.start()`. **Without G1 every other analysis feature is worthless.**

### G2 — per-mission manifest bundle (the "every datum to represent the rover")
Each mission writes a bundle dir `<mission>_<utc>/` containing the bag **and** `manifest.json`:
- **identity:** mission_id / path name, placement_mode, origin_gps, staged flag (best-effort from read-only `GET /api/mission/loaded-path`; if unavailable, record with a minimal identity — never skip the bag).
- **timestamps:** recorder_start/end UTC + local (IST) readable; mission start/end from `/api/mission/status`.
- **as-run config:** FCU param snapshot (`COM_OF_LOSS_T`, `RO_YAW_P`, `RO_YAW_RATE_LIM`, `RD_TRANS_*`, `EKF2_WENC_CTRL`, `RBCLW_*`, `NAV_ACC_RAD`, `PWM_AUX_*`, …) + the RPP param block already embedded in `/rpp/debug[11..38]`. Behaviour must be attributable to the exact config that produced it.
- **environment:** git commit SHA of the deployed tree, service active-states (`rover-server`/`rpp-pipeline`/`px4-dxp`/`bag-autorecord`), ROS_DOMAIN_ID, hostname.
- **outcome:** mission terminal state, integrity result, file list.

### G3 — integrity + secret masking
SHA256 every file in the bundle into the manifest (`outcome.integrity`). Redact `token|password|secret|passwd` and `user:pass@host` patterns from all manifest strings, log output, and any captured `/mavros/statustext`. Never persist the machine token; read it, use it, never write it.

### G4 — disk management (production safety)
Preflight free-space check before each capture (`BAG_MIN_FREE_BYTES`, default 5 GiB); refuse to start and warn if below floor. Retention rotation by oldest-first when total bundle bytes exceed `BAG_MAX_TOTAL_BYTES` (default 50 GiB) or free drops under `BAG_LOW_FREE_BYTES` (default 2 GiB). A full disk degrades the recorder only — never the rover services.

### G5 — crash reconciliation
On daemon start, scan for bundles with no `end` timestamp / open bag and mark them `INCOMPLETE` (+ an `INCOMPLETE` sentinel file), finalising the manifest so a power-cut or OOM leaves labelled evidence, not silent corruption.

### G6 — post-mission behaviour analyser  **[the deliverable the operator asked for]**
See §4.

### G7 — topic completeness
Add to `TOPICS`: `/mavros/statustext` (failsafe/mode reasons), `/path/identity`, `/rpp/conditioned_path`(+identity), `/rpp/setpoint_bridge_debug`, `/spray/desired`, `/spray/commanded`, `/spray/debug`, `/spray/runtime_status`, `/mavros/global_position/global`. Drop the non-existent `/mavros/local_position/velocity_body`. Keep the set explicit (not `-a`) so bag size stays bounded and QoS overrides are targeted.

---

## 4. G6 — Post-mission behaviour analyser (detailed)

A standalone, dependency-light analyser (`tools/analyze_mission.py`) that reads one finalised bundle and emits `analysis.json` + a human-readable `report.txt`. Generalises the D0/D3 checkers from a single event to the whole mission. It must run **offline on any bag** and also be invokable automatically on finalise (§5).

**Inputs:** `/path` (geometry + spray flags), `/mavros/local_position/pose` (actual NED), `/mavros/local_position/velocity_local` (measured speed), `/rpp/debug` + `/rpp/segment_debug` (controller state, xtrack, params), `/mavros/state`, `/mavros/statustext`, `/mavros/gpsstatus/gps1/raw`, `/spray/*`.

**Report sections (per mission, and per run/corner where applicable):**
1. **Tracking** — cross-track error: RMS, median, p95, max (cm); left/right sign bias. Per marking run + overall. Verdict vs the ≤2 cm production class.
2. **Stops** — at every waypoint/corner/endpoint: closest approach, coast-past-after-arrival, resting distance, confirmed-stop dwell. (D3 metric, generalised to all stops.)
3. **Pivots** — at every corner/run-boundary: initial heading error, turn magnitude vs commanded, settle time, reverse-flip check (forward-component ≥ 0 while turning), oscillation count. (D0 metrics, per corner.)
4. **Speed** — commanded vs actual profile; per-segment cruise; corner-slowdown depth; accel/decel realised vs limits.
5. **Spray** — desired vs commanded vs actual state timing; ON/OFF offset relative to MARK boundaries (lead/lag cm); any misfire/leak (spray while transit, or gap during MARK).
6. **Health / anomalies** — OFFBOARD drops (mode flips off OFFBOARD while armed), setpoint-stream gaps > 0.5 s (failsafe risk), RTK degradation (fix_type < 6 during drive), EKF position jumps, pose-staleness gaps, and any `statustext` failsafe/reject lines.
7. **As-run config** — FCU + RPP param snapshot (from manifest + `/rpp/debug`), so every number above is attributable.
8. **Verdict** — overall PASS/FAIL against production thresholds, with the worst offenders listed (like the D-checkers).

**Design rules:** pure reader (never touches the robot); tolerant of missing topics (report WARN, not crash); no hardcoded shape assumptions (works for line/L/square/arc/point); numbers in cm/°/m·s⁻¹ with clear thresholds.

---

## 5. Integration with the current flow (minimal, decoupled)

- **Trigger:** unchanged — the existing poll of `GET /api/mission/status`. Start on the first active state (`arming`) so the pre-`running` `/path` publish is inside the recording window; combined with G1's QoS override this guarantees `/path` capture.
- **Identity enrichment:** best-effort read-only `GET /api/mission/loaded-path`; failure → minimal identity, still record. **No new server endpoints, no lifecycle hooks, no handshake.**
- **Auto-analysis:** on finalise, the daemon spawns `tools/analyze_mission.py <bundle>` as a detached best-effort step (a failed/absent analyser never affects the bag or the rover).
- **Server code touched: none required.** (Optional, later: surface the last analysis verdict on a read-only telemetry field — separate, out of this plan.)

---

## 6. Build order (each isolated, one deploy, one verification)

1. **P1 — G1 + G7** (QoS override + topic set). *Verify:* run an API-driven mission; confirm the bag contains `/path` with points and all listed topics. This alone makes bags analysable.
2. **P2 — G2 + G3** (manifest bundle + integrity + redaction). *Verify:* manifest has identity/params/services/SHA256; no secrets present; unit test the redactor + manifest writer.
3. **P3 — G4 + G5** (disk management + crash reconciliation). *Verify:* simulate low-free → refuse+warn; kill -9 mid-record → next start marks INCOMPLETE.
4. **P4 — G6** (`analyze_mission.py`). *Verify:* run on the D0 and D3 bags (already on Mac) — must reproduce the known numbers (D3 coast 9.9 cm; D0 pivot 169.6°). Then on a full square mission bag.
5. **P5** — wire auto-analysis on finalise + `.gitignore` bundles + systemd env additions + deploy.

Deploy per component: `bag_autorecord.py` is run by `bag-autorecord.service`, so changes need `sudo systemctl restart bag-autorecord` (never touches `rpp-pipeline`/MAVROS). `analyze_mission.py` is a pure tool (no restart).

---

## 7. Acceptance criteria

- [ ] Mission bag contains `/path` (+ conditioned_path) with points — TRANSIENT_LOCAL captured (G1).
- [ ] Every mission produces a bundle with `manifest.json`: identity, UTC+local timestamps, FCU+RPP param snapshot, service states, git SHA, per-file SHA256; **no secrets** anywhere (G2/G3).
- [ ] Low-free-space refuses capture with a warning; retention rotates oldest; rover services unaffected (G4).
- [ ] `kill -9` mid-record → next daemon start marks the bundle INCOMPLETE with a finalised manifest (G5).
- [ ] `analyze_mission.py` reproduces D0 (169.6° pivot, no reverse-flip) and D3 (coast 9.9 cm, stop on point) from their bags, and produces a full report + PASS/FAIL for a square mission (G6).
- [ ] Mission start/stop latency and success are **unchanged** with the recorder up, down, or disk-full (R1) — the decoupling invariant.
- [ ] No new server lifecycle coupling; `mission_debug_capture.py` handshake NOT introduced.

---

## 8. Non-goals

- The server→recorder request/ack handshake and `CaptureUnavailable` (rejected — R1).
- Blocking/failing a mission on capture readiness.
- `-a` (record-all) as the default — bounded explicit topic set only.
- Live streaming/telemetry of analysis to the app (separate future work).
- Re-recording firmware ULogs (PX4-side; out of scope — companion topics only).

---

## 9. "Mask the rover as production grade" — interpretation

Read two ways, both covered: (a) **represent** the rover fully — the manifest + topic set + as-run params capture every datum needed to reconstruct a mission's behaviour offline (§3 G2, §4); (b) **mask** sensitive data — redaction + no-token-persist + statustext scrubbing (§3 G3, R5). If the operator meant only one, the other is still correct to have.

---

## 10. One-line summary

**Keep the baseline's decoupled poll-recorder; add QoS-latched `/path` capture, a per-mission manifest with as-run params + integrity + secret masking, disk management + crash reconciliation, and a `analyze_mission.py` behaviour report — the recorder stays a pure observer that can never block, fail, or slow a mission.**
