# Merge waiver — `Upgrade_Spray` → `baseline_master`, 2026-07-27

105 commits (07-11 → 07-27), promoted with **five items explicitly NOT proven**,
listed here so the promotion never silently implies they were. Operator decision
2026-07-27 evening: merge now with this waiver; item 1 is scheduled for the very
next field session (2026-07-28, checklist row B1).

## Gate evidence at merge time

- Tests: **873 green** — 334 server + 475 path_engine + 64 tools; no skips.
- Three-way sync: mac == origin at merge SHA; Jetson on the same branch
  (docs-only lag pulled level as part of the merge).
- Field: pre-line checklist rows 1.1–1.3 and 2.1–2.4 **PASS** on 2026-07-27
  (multi-leg, surveyed curve, 90° corners, dash), 12 bags, full traversal on
  every readable bundle. Week audit graded in
  `docs/FIELD_CHECKLIST_2026-07-28.html` §A (~41 feature-groups verified).
- Register: `docs/OPEN_BUGS.md` current as of `472b062` (A17 registered,
  A12 flipped to FIXED with evidence).

## Waived items — known, named, and NOT covered by this promotion

1. **RTK-dropout gate trip UNPROVEN.** The Phase B spray gate (`fdf83b0`) ran
   default-ON through every 07-27 run without false-blocking, but the trip
   itself has never been witnessed: the fix never dropped (zero `fix_type`
   transitions; hand-shading a dual-antenna UM982 is insufficient).
   Closing method is scheduled: `POST /api/rtk/stop` mid-MARK, expect
   `spray_active_desired=true` while `spraying=false` within ~2 s, recovery
   after `gps_recover_hold_s`=1.0 s. **2026-07-28 row B1.**
   Note the asymmetry: this gate exists ONLY on the promoted branch — the
   previous baseline had no RTK spray gate at all, so the promotion is strictly
   safety-positive even with the trip unproven.

2. **Point mode, point flow, and the G0–G5/G6 handshake are WAIVED BY PRODUCT
   SCOPE, not verified.** Pre-line road marking is lines, not dots. The stack
   (12 commits) is default-OFF, its in-env rclpy tests have never run, its
   frontend control does not exist, and it is blocked on B1/B15 besides. It
   ships inert.

3. **A14 residue.** On boots where the GNSS driver reports no accuracy
   (~half, latched per boot), the spray gate is fix-type-only. The accuracy
   half (`spray_max_hrms_m=0.10`) is live and enforced on every reporting
   boot; telemetry now says `gps_accuracy_known` honestly. Full closure needs
   the driver-level root cause (PX4 `sensor_gps` emitting `eph/epv=0`).

4. **A17 open — line terminal shutoff.** Paint stops ~12 cm short of the last
   surveyed station on straight lines, extension-independent (7-run evidence,
   2026-07-27). Largest paint error remaining in the product. Registered with
   fix directions; physical confirmation via the 07-28 mark re-survey (row B6).

5. **Joystick stack deployed, never live-driven.** Heartbeat retype is
   journal-confirmed; arm-on-acquire / throttle 0.35 / steering axis await the
   07-28 drive test (row B2). Operator-triggered feature — no autonomous
   exposure.

## Also known at merge time (registered, not waiver-class)

A11 (queued immediately post-merge), A3 (staged, default-OFF, needs named A/B),
A7, A8, A13, A10, preline B1/B14/B15, start-truth xtrack gate, hardware
calibration B-section, frontend CSV-extensions control (separate repo, in
progress). NTRIP does not auto-start after a rover-server restart — operational
gotcha, twice bitten on 07-27; pre-flight now checks `/api/rtk/status`.
