# RPP Production-Readiness Review

Review date: 2026-06-19  
Repository: `main` at `d537eee`; local, `origin/main` and Jetson synchronized.  
Working tree at review: only this untracked document, `docs/RPP_PRODUCTION_READINESS_REVIEW.md`.

## 1. Baseline Correction

The previous review genuinely inspected commit `a0634a39e5283afd91232d428ded14ceb6a6a9e8`; it did not merely record the wrong hash. At that time `git rev-parse HEAD` returned `a0634a39...`. The current production baseline is `d537eeeb4838e4ec33683de1c70d6ee54e2ff57b`.

Both ancestry checks succeed:

```text
git merge-base --is-ancestor a327829 d537eee  # exit 0
git merge-base --is-ancestor a0634a39 d537eee # exit 0
```

`d537eee` is the immediate revert of `a0634a3`. The complete source delta is 18 changed lines in four spray files:

- `src/spray_controller_node.py`: direct-servo default `on_pwm_us` `2500 -> 1800`; clamp `3000 -> 2200`.
- `server/routes/spray_params.py`: matching schema default/range.
- `src/test_spray_backend.py` and `src/test_spray_manual_override.py`: matching expectations.

There are no changes between these commits in the planner, RPP, mission lifecycle, emergency handler, MAVROS bridge, service definitions, or PX4 parameter snapshots. Consequently, most earlier source observations remain applicable, but the repository baseline, spray PWM statement, severity assignments, and deployed-configuration conclusions required correction.

## 2. Executive Verdict

**CONDITIONAL GO for controlled validation; not yet an unconditional production GO.**

The `d537eee` source baseline contains the major anti-reversal, collinear-boundary, per-line extension, corner-stop and endpoint-stop fixes. No repository P0 was confirmed in this revision. Final production authorization remains conditional on live FCU/ROS parameter evidence, exact-baseline repeated field runs, Offboard-loss testing, malformed-path rejection, and spray-compensation validation.

If the live FCU still reports `COM_OF_LOSS_T=30`, the deployed system is **NO-GO** until corrected and stream-loss tested. That is a deployed PX4 configuration issue, not a defect introduced by repository revision `d537eee`.

## 3. Production-Readiness Score

| Area | Score | Basis |
|---|---:|---|
| Planner/geometry | 76/100 | Broad entity support and extension fixes; weak hard validation |
| Controller/RPP | 77/100 | Correct transforms, projection, curvature and anti-flip logic; start evidence incomplete |
| Corner FSM | 82/100 | True-stop, braking, settle gates and endpoint approach present |
| PX4 integration source | 75/100 | 50 Hz fail-zero bridge; firmware behavior external |
| Deployed configuration evidence | 38/100 | Historical dumps exist; live production dumps absent |
| Localization/mechanics evidence | 38/100 | Good trajectories but no complete calibration/noise budget |
| Tests/diagnostics | 70/100 | Strong pure tests; ROS/PX4 integration coverage incomplete |
| Field verification | 58/100 | Strong individual runs, insufficient exact-baseline matrix |
| **Overall** | **68/100** | **Conditional validation readiness** |

The former `49/100` is not retained. It improperly counted a historical FCU setting as a repository P0 and over-weighted several bounded source weaknesses.

## 4. Source and Deployment Truth

| Evidence layer | Established truth | Remaining evidence |
|---|---|---|
| Repository | Local/remote/Jetson commit reported as `d537eee`; local Git verifies HEAD and `origin/main` | Jetson command output was operator-verified, not independently queried in this Mac review |
| RPP defaults | Defaults are declared in `src/rpp_controller_node.py`; `rpp_start.sh` supplies no overrides | Live `ros2 param dump /rpp_controller` |
| Spray defaults | Default backend `mavlink_actuator`; normalized ON/OFF `1.0/-1.0`; direct-servo ON `1800`, max clamp `2200`, OFF `0` | Live spray parameter dump and active backend |
| PX4 parameters | Historical ULogs through June 15 record `COM_OF_LOSS_T=30`, `COM_OBL_RC_ACT=5`; repository snapshots vary | Live QGC/FCU `.params` export |
| PX4 firmware | ULogs identify firmware commit `54f0455ffcd755534539a7cf33a09a20bf71d29d` | Current ULog or `ver all`; source is in external PX4 fork |
| Systemd | Repository units inspected | `systemctl cat` from Jetson |
| Field results | Bags through June 18 inspected | Repetitions produced after `d537eee` with matching ULogs |

## 5. Required Fix Verification at `d537eee`

### BUG-T3 First-Run Protection

**Present.** Commit `510be9b` is an ancestor of `d537eee`. Both segment and smooth tracking call `_clamp_velocity_to_forward_cone()` before publishing velocity (`src/rpp_controller_node.py:2601-2604`, `3019-3022`). The helper constrains requested bearing to `+/-75 deg`, keeping positive forward projection and preventing PX4 reverse selection (`3149-3190`).

The first run intentionally does not use `_run_alignment_hold`; `src/test_smoke_rpp_controller.py:366` asserts this. Therefore:

- Reverse-flip protection: implemented.
- Stationary first-run STOP/ALIGN: not implemented.
- Whether moving acquisition meets the accuracy requirement at `90/120/180 deg`: needs field evidence.

### PRE/MARK/AFT Collinear Slowdown

**Present.** `a327829` is an ancestor of `d537eee`, followed by `7853a1b` and `cd44884`. Current conditioning merges collinear spray-flag runs without deleting flags (`src/rpp_controller_node.py:664-667`, `1069-1098`). Sub-threshold segment junctions retain `_last_speed_cmd`; only real corners reset it (`2363-2375`).

### Per-Line Extensions and Path Conditioning

**Present.** `c29fb67`, `7d38645`, `487ebbe`, `7853a1b`, `9d0d2e9`, `a10eccf`, `a327829`, and `cd44884` are all in the current history. The planner decomposes line chains per edge, generates PRE/MARK/AFT, inserts explicit TRANSIT connectors, realigns compensated boundaries, and densifies extension runs. RPP performs run splitting, connector absorption, collinear merging, simplification, smoothing and resampling.

### PX4 Hold/Loiter and Zero-Speed Changes

**External to this repository.** This repo contains patch specifications and historical statements, not the PX4 source tree. `docs/Progress/PROGRESS.md` records P1-P4 pushed to an external fork, while research documents also contain older “not validated” statements. The ULog firmware hash alone does not prove the exact P4/hold implementation.

Status: `NEEDS_RUNTIME_EVIDENCE`. Required proof is the current external PX4 commit/source plus a zero-speed and setpoint-loss ULog.

### Spray PWM after `d537eee`

For `mavlink_servo_pwm`, default ON is `1800 us`, values are clamped to `2200 us`, and OFF is `0 us`. For the default `mavlink_actuator` backend, the revert has no effect: ON/OFF remain normalized `1.0/-1.0`, with physical PWM determined by FCU output parameters.

Comments at `src/spray_controller_node.py:331-335` still describe a 3000 us normalized mapping and are stale relative to repository PX4 parameter snapshots that show 2000 us output maxima. Runtime behavior therefore depends on the live backend and FCU actuator output configuration.

## 6. Previous-Finding Reconciliation

Each prior substantive finding is assigned exactly one requested status.

| Previous finding | Status | `d537eee` determination |
|---|---|---|
| Review baseline was `a0634a39` | STALE_LINE_REFERENCE | It was true during the first audit but is not the production baseline |
| BUG-T3 first-run protection was missing | DISPROVEN | Forward-cone clamp exists in both tracking profiles |
| First run lacks stationary STOP/ALIGN | CONFIRMED_AT_D537EEE | First run intentionally skips `_run_alignment_hold` |
| Large-yaw start is therefore unsafe | OVERSTATED | Anti-reverse clamp bounds direction; accuracy still needs tests |
| PRE/MARK/AFT can trigger false slowdown | FIXED_BEFORE_D537EEE | Current merge and momentum-preservation fixes are present |
| Per-line PRE/MARK/AFT generation is absent | FIXED_BEFORE_D537EEE | Planner and sticky sidecar support are present |
| Non-finite geometry is not hard rejected | CONFIRMED_AT_D537EEE | Validator has no finite-value pass |
| POINT marking is functionally unsupported | CONFIRMED_AT_D537EEE | Single-point RPP runs advance/complete without a marking dwell |
| Runs shorter than 5 cm can be dropped | CONFIRMED_AT_D537EEE | RPP filters multi-run slivers below 0.05 m |
| Connectors below 20 cm may be absorbed | CONFIRMED_AT_D537EEE | Bounded by dual-corner gating; this is intentional conditioning |
| Shape grouping uses 5 cm join tolerance | CONFIRMED_AT_D537EEE | `group_join_tol_m=0.05` default remains |
| Planner speed metadata is discarded by `/path` | CONFIRMED_AT_D537EEE | Path carries coordinates and spray flags only |
| This necessarily causes wrong vehicle tracking speed | OVERSTATED | RPP deliberately uses global `mission_speed`; transit/mark speed profiles are not supported |
| Planner spray compensation can mismatch runtime speed | CONFIRMED_AT_D537EEE | Planner uses `PathSegment.speed`; RPP executes global speed |
| Planner and distance-aware spray can both compensate latency | CONFIRMED_AT_D537EEE | Both default enabled; combined boundary behavior needs calibration |
| Final DONE does not require measured zero speed | CONFIRMED_AT_D537EEE | RPP uses position/progress; server waits DONE time only |
| Final completion necessarily occurs while moving | NEEDS_RUNTIME_EVIDENCE | Latest bags suggest good stopping, but no universal speed gate exists |
| Emergency-handler lock may be `None` | CONFIRMED_AT_D537EEE | It directly enters `_controller._lock` after actuation |
| That lock bug prevents E-stop actuation | OVERSTATED | Stop-path, MANUAL and disarm are attempted before the lock failure |
| Alignment accepts up to 5 cm fit RMSE | CONFIRMED_AT_D537EEE | `RMSE_MAX=0.05` remains |
| Every accepted mission therefore has 5 cm alignment error | DISPROVEN | Threshold is an upper bound, not the measured residual |
| RPP smooth spacing defaults to 8 cm | CONFIRMED_AT_D537EEE | `path_resample_spacing_m=0.08` |
| Planner MARK spacing defaults to 5 cm | CONFIRMED_AT_D537EEE | `mark_spacing/line_spacing=0.05` |
| 0.67 m is a proven minimum safe radius | OVERSTATED | It is a no-saturation estimate, not a field-validated support bound |
| Validator warns below 0.3 m but does not reject | CONFIRMED_AT_D537EEE | `min_turn_radius_m=0.3`; violations are warnings |
| Closest/lookahead coordinates are missing from RPP debug | CONFIRMED_AT_D537EEE | Arrays publish errors/distances, not both point coordinates |
| `COM_OF_LOSS_T=30` is a repository-code defect | DISPROVEN | It is FCU configuration, not controlled by this repo runtime code |
| Historical ULogs contain `COM_OF_LOSS_T=30` | CONFIRMED_AT_D537EEE | Artifact fact remains true |
| Live production FCU still uses `30` | NEEDS_RUNTIME_EVIDENCE | Live parameter dump is absent |
| P4 zero-speed firmware patch is active | NEEDS_RUNTIME_EVIDENCE | PX4 implementation is external to this repo |
| Wheel fusion differs between snapshot and ULogs | NEEDS_RUNTIME_EVIDENCE | Live FCU dump must resolve the conflict |
| Equal-priority FIFO services can starve each other | NEEDS_RUNTIME_EVIDENCE | Configuration is present; no measured starvation evidence |
| Latest field metrics prove `d537eee` | OVERSTATED | Bags predate the final source commits/revert, though tracking delta is narrow |
| Arbitrary supported shapes meet 2 cm mean/3 cm p95 | NEEDS_RUNTIME_EVIDENCE | Required exact-baseline matrix is incomplete |

## 7. High-Priority Source Re-check

### First-Run Alignment

The previous P2 defect is revised to a **P2 verification gap**. The anti-flip control is real and unit-tested, so this is not an unresolved reversal defect. The remaining question is acquisition XTE and initial motion at large heading errors.

Required tests: stationary starts at `90`, `120`, `180`, and `-90 deg`; report maximum acquisition displacement, reverse-selection state, time to `<=2 deg`, and first MARK entry XTE.

### Non-Finite Planner Rejection

Confirmed source weakness. `PathValidator.validate_detailed()` checks counts, bounding box, curvature, gaps and intersections, but never calls `math.isfinite` over waypoints, speeds, transforms or computed metadata. Empty paths are warnings rather than hard errors.

Revised severity: **P1 input-hardening defect** for production CAD ingestion. The downstream setpoint bridge rejects non-finite velocities, but that is too late to guarantee correct mission geometry.

### Speed Metadata and Spray Compensation

`PathSegment.speed` is used by planner spray compensation (`lead=latency*speed`) and reporting. `RosBridgeNode.publish_path()` transmits only N/E coordinates and spray state in Z. RPP uses global `mission_speed`; it does not execute planner MARK/TRANSIT speeds.

This is not inherently a tracking defect because the controller design intentionally exposes one mission speed. It is a **P2 contract defect** when planner `marking_speed` differs from live `mission_speed`, and a spray-boundary risk because distance-aware spray also applies actual-speed anticipation by default.

### Final Physical Stop

RPP emits continuous zero velocity after DONE, and the server requires DONE for 1.0 s before marking completion. This is stronger than the former report implied. However, neither RPP DONE nor `RppStatusMonitor.is_done()` checks measured speed, yaw rate, final heading, or coast distance. Soft stop returns IDLE after 0.1 s.

Revised severity: **P2 acceptance/diagnostic gap**, not proof of uncontrolled stopping.

### Emergency Lock

`EmergencyHandler.estop_async()` attempts stop-path, MANUAL mode and disarm before entering `_controller._lock`. If e-stop is called before any lifecycle method creates the lazy lock, the state-update block can raise. Safety commands still run, but the API can fail and mission state may remain stale.

Revised severity: **P2 lifecycle robustness defect**, not P1 loss of actuation.

### Offboard-Loss Timeout

Historical ULogs and `docs/TODO/2026-06-13_pending_tasks.md` record `30 s` as an intentional test value that must become `0.3 s` before production. No live dump was supplied in this review.

Classification: **deployed configuration, NEEDS_RUNTIME_EVIDENCE**. If live value is `30`, severity is deployment **P0** and verdict becomes NO-GO. If live value is the validated production timeout, the source verdict is unaffected.

### Alignment RMSE

The code accepts multi-point alignment fits up to `0.05 m`. That does not mean actual error is 5 cm, but it permits missions whose fit residual is incompatible with a 2 cm mean target.

Revised severity: **P2 policy mismatch**. Precision mode should enforce an evidence-based tighter threshold or explicitly reject the 2 cm target when fit RMSE is too large.

### Spacing and Radius

- Planner MARK spacing: `0.05 m`.
- Planner TRANSIT spacing: `0.15 m`.
- RPP smooth resampling: `0.08 m`.
- Segment profile simplifies collinear samples rather than using fixed resampling.
- Validator radius threshold: `0.30 m`, warning only.
- RPP yaw command clamp: `0.45 rad/s`; smooth curvature speed floor: `0.30 m/s`.

The simple no-saturation estimate `R >= v_min/omega_max = 0.30/0.45 = 0.67 m` is useful but not a certified minimum: PX4 heading behavior, velocity-vector control, tire slip and acceleration transients also matter. Only the 1.5 m arc has repeated direct evidence in the inspected artifacts.

## 8. Current Spray Behavior

| Path | Current default/limit | Source truth | Runtime dependency |
|---|---:|---|---|
| `mavlink_servo_pwm` ON | `1800 us` | Reverted by `d537eee` | Live `actuator_backend` must select it |
| `mavlink_servo_pwm` maximum | `2200 us` | Code clamp and API schema agree | FCU command acceptance |
| Direct-servo OFF | `0 us` | Not clamped | Hardware/FCU must support 0 as intended |
| `mavlink_actuator` ON/OFF | `1.0/-1.0` | Default backend; unchanged by revert | FCU min/max/disarmed mapping |
| Auto spray | distance-aware `true` | Uses measured speed and projected nozzle position | Live offsets/delays/margins |
| Planner compensation | `true` by plan default | Shifts MARK start/end using planner speed | Can combine with controller anticipation |

The direct-servo revert tests pass. The normalized-backend comments claiming 3000 us are stale and must not be treated as evidence of actual output.

## 9. Geometry and Controller Conclusions

Confirmed strengths at `d537eee`:

- Consistent DXF East/North to NED conversion for supported entities.
- Exact preservation of line/resampling endpoints.
- Explicit TRANSIT insertion across disconnected geometry.
- Per-line extension generation with sticky configuration.
- Spray-state-preserving collinear run merging.
- Segment projection, signed XTE, bounded lookahead, Menger curvature and lateral-acceleration regulation.
- Forward-cone anti-reversal clamp on first and subsequent tracking.
- Hard-corner STOP, measured stop dwell, active braking, ALIGN settle and angle-aware watchdog.
- Dedicated low run-endpoint approach speed and continuous DONE zero output.
- 50 Hz MAVROS raw setpoint stream with 0.2 s stale-input zeroing.

Remaining source constraints:

- Runs under 5 cm may be dropped in multi-run missions.
- Dual-corner connectors below 20 cm may be absorbed.
- Self-intersections are warned, not rejected.
- Large paths use sampled intersection checks.
- POINT marking lacks an intentional dwell/action contract.
- Speed profiles do not survive flattening into `nav_msgs/Path`.
- Final completion lacks measured-rest and final-heading acceptance.

## 10. Tests Re-run at `d537eee`

| Test set | Result |
|---|---:|
| Planner extensions/engine/validator/spray/grouping | `188 passed` |
| Direct/normalized spray backend standalone tests | All passed |
| Spray manual override/master gate tests | All passed |
| Corner pivot mathematics | `17 passed` |
| Segment stop mathematics | `10 passed` |
| Sprint 2 geometry/projection/curvature checks | `15 passed` |

Limitations:

- Mac lacks ROS2/rclpy, so production-node ROS tests were not rerun here.
- Some standalone controller tests mirror equations instead of importing the production class.
- No automated stream-loss, physical stop, actuator PWM measurement, or PX4 SITL test is in this run.

## 11. Field Evidence

The prior numeric analysis remains valid as historical field evidence because the `a0634a3 -> d537eee` delta is spray-PWM-only. It does not, however, prove exact `d537eee` behavior for collinear momentum and current spray output because the newest analyzed bags predate some final source commits.

Strong historical results:

- Square `20260618_203927`: MARK RMS `0.64 cm`, median `0.50 cm`, 100% within 2 cm, pivot exits `<=1.74 deg`.
- Triangle `20260618_151630`: leg means `0.26/0.64/0.80 cm`, p95 `0.77/1.80/2.73 cm`, maximum `3.11 cm`.
- Three June 13 squares: p95 approximately `0.99-1.07 cm`.
- Three L-shapes: p95 `0.79-1.68 cm`.

Counter-evidence and gaps:

- Earlier triangle p95 reached `5.14-7.38 cm`.
- One repeated 1.5 m arc had p95 `3.49 cm`.
- Multi-shape p95 reached `4.23-5.97 cm`.
- No exact-baseline repeated CW/CCW matrix, wrong-heading starts, final-heading set, stream-loss set, battery comparison, or synchronized latest ULogs.

## 12. Confirmed Defects and Release Gaps

| Priority | Layer | File/function | Evidence at `d537eee` | Failure mode | Correct fix | Required test |
|---|---|---|---|---|---|---|
| P1 | Planner safety | `path_engine/validator.py:validate_detailed` | No finite-value validation | Malformed geometry can pass planning validation | Hard reject non-finite waypoints, transforms, speeds and metadata | NaN/Inf DXF, CSV and API tests |
| P2 | Spray contract | `path_engine/spray.py`; `server/ros_node.py:publish_path` | Planner speed/compensation not transmitted; distance-aware compensation also enabled | Boundary timing depends on mismatched or doubled compensation | Define one compensation owner or transmit speed profile | Multiple-speed painted boundary tests |
| P2 | Mission completion | RPP goal checks; `server/rpp_status.py:is_done` | No measured-rest or final-heading gate | Completion can precede proven physical settle | Add speed/yaw-rate/final-heading acceptance and telemetry | Repeated final-stop tests |
| P2 | E-stop lifecycle | `server/emergency.py:69-72` | Lazy lock may be `None` after safety commands | API exception and stale mission state | Use `_lifecycle_lock()` or lock-independent state update | E-stop before first mission lifecycle call |
| P2 | Alignment policy | `server/config.py:RMSE_MAX` | Allows 5 cm residual | Accepted alignment can exceed precision budget | Precision threshold tied to requested tolerance | Surveyed residual rejection tests |
| P2 | Start verification | RPP first-run cone clamp | Anti-flip exists; stationary align absent | Acquisition displacement unbounded by current evidence | Field-bound behavior or add explicit initial FSM | 90/120/180-degree start matrix |
| P3 | Diagnostics | RPP debug arrays/logger | Missing closest/lookahead coordinates, entity ID and regulation reason | Harder sub-2 cm root-cause analysis | Publish structured stamped diagnostics | Bag schema/replay test |

Potential deployment P0, not confirmed source defect:

| Priority | Layer | Evidence | Required resolution |
|---|---|---|---|
| P0 if live | PX4 configuration | Historical ULogs/TODO show `COM_OF_LOSS_T=30` | Live dump; set production value through QGC; kill-stream test |

## 13. Parameter Conclusions

Repository defaults remain:

| Layer | Key values | Assessment |
|---|---|---|
| RPP speed | mission `0.35`, hardware max `0.8`, min `0.15`, curvature floor `0.30` | Needs live dump and shape-specific bounds |
| Lookahead | min `0.52`, max `1.0`, time `1.6`, XTE gain `0.05` | Validated on selected paths, not arbitrary short geometry |
| Smooth curve | spacing `0.08`, `a_lat_max=0.3`, yaw clamp `0.45` | 1.5 m evidence; smaller radii constrained |
| Goal | tolerance `0.02`, approach `0.6`, segment endpoint speed `0.03` | Strong latest square evidence; physical settle gate absent |
| Corner | threshold `45 deg`, slowdown `0.5`, acceptance `0.05`, corner speed `0.08` | Field-backed at right/triangle corners |
| Stop/align | speed `0.02`, yaw rate `0.05`, stop dwell `0.30`, heading `2 deg`, align settle `0.20` | Sound code path; live params required |
| Connector | absorb `0.20`, dual-corner gate `20 deg` | Intentional CAD constraint |
| Planner | MARK `0.05`, TRANSIT `0.15`, speeds `0.35/0.50` | Spacing is active; speeds do not form executed profile |
| Alignment | max fit RMSE `0.05` | Too permissive for guaranteed 2 cm work |
| Spray direct PWM | ON `1800`, max `2200`, OFF `0` | Current revert behavior; backend-dependent |

PX4 controller, EKF, wheel geometry, output and failsafe values remain **runtime evidence**, even when historical snapshots are present.

## 14. Shape Support

| Classification | Shapes |
|---|---|
| Supported with useful historical evidence | straight, long straight, 1.5 m arc, square, L-shape, triangle |
| Implemented but not production-verified | circle, rectangle, rounded rectangle, line-arc-line, arc-line-arc, mixed paths, open/closed polygons |
| Requires explicit constraints | small radius, short lines, connectors below 20 cm, closely parallel paths, disconnected entities |
| Not approved for production | POINT marking, self-intersection, hairpin/near-180-degree reversal, arbitrary dense/sparse path without validation |
| Direction evidence missing | equivalent CW/CCW and left/right repeated sets |

## 15. Sub-2 cm Feasibility

**Achievable under controlled conditions; not yet established for arbitrary supported CAD paths.**

The best square and triangle results satisfy the requested XTE limits. The variation across earlier triangle, arc and multi-shape runs shows that code correctness alone does not bound field performance. A 2 cm mean target additionally requires:

- RTK repeatability near 1 cm RMS or better.
- Survey/alignment residual well below 2 cm.
- Calibrated wheel diameter/track width and bounded slip.
- Low latency/jitter and correct EKF delays.
- Shape radius/segment-length constraints.
- Repeated exact-baseline field evidence.

## 16. MISSING EVIDENCE REQUIRED FOR FINAL VERDICT

1. Live ROS parameters:

   ```bash
   ros2 param dump /rpp_controller > ~/rpp_controller_d537eee.yaml
   ros2 param dump /spray_controller > ~/spray_controller_d537eee.yaml
   ```

   Determines actual controller values, spray backend and PWM behavior.

2. Live FCU parameter export through QGC, or MAVLink shell output covering at least:

   ```text
   COM_OF_LOSS_T COM_OBL_RC_ACT RO_* RD_* EKF2_GPS_* EKF2_WENC_* CA_R_REV PWM_AUX_*
   ```

   Determines whether the historical 30-second timeout remains a deployment blocker and resolves wheel/output conflicts.

3. PX4 firmware identity and patch proof:

   ```text
   ver all
   ```

   Plus the external PX4 fork commit containing the flying `DifferentialVelControl` P3/P4 code. This determines zero-speed Hold/Loiter behavior.

4. Deployed services:

   ```bash
   systemctl cat px4-dxp rpp-pipeline rover-server bag-autorecord > ~/production_units_d537eee.txt
   ```

5. One ROS bag plus matching ULog for each exact-baseline acceptance run. Include CW/CCW, `90/120/180 deg` starts, final stop, stream loss, and spray boundary runs.

6. Mechanical/localization record: loaded left/right wheel diameter, track width, encoder PPR/gear ratio, antenna XYZ, and a 10-minute stationary RTK bag.

## 17. Required Before Unconditional Release

1. Prove live Offboard timeout/action and pass a kill-stream test.
2. Add hard non-finite path rejection.
3. Resolve ownership of planner versus distance-aware spray latency compensation.
4. Bound first-run large-yaw acquisition with three repetitions per angle.
5. Gate or at least independently verify physical stop and final heading.
6. Tighten or job-gate alignment RMSE for 2 cm work.
7. Capture/version live ROS, PX4, systemd and firmware configurations.
8. Complete three-run CW/CCW critical-shape matrix at `d537eee`.

## 18. Final Release Checklist

- [x] Local `main` and `origin/main` at `d537eee`.
- [x] BUG-T3 forward-cone clamp present.
- [x] Collinear PRE/MARK/AFT momentum fixes present.
- [x] Per-line extensions and endpoint stop fixes present.
- [x] Direct-servo PWM revert verified by source and tests.
- [ ] Live RPP and spray dumps captured.
- [ ] Live PX4 dump proves production Offboard timeout/action.
- [ ] External PX4 P4 source and flashed commit verified.
- [ ] Stream-loss test passed.
- [ ] Non-finite ingestion tests passed.
- [ ] Exact-baseline wrong-heading start matrix passed.
- [ ] Exact-baseline final stop/heading matrix passed.
- [ ] Exact-baseline CW/CCW shape repetitions passed.
- [ ] Spray boundaries verified at all supported mission speeds.
- [ ] Localization/mechanical accuracy budget measured.

## 19. Final Verdict

**CONDITIONAL GO**

`d537eee` is suitable for controlled production-validation runs. The prior repository-level NO-GO was too strong because it treated historical `COM_OF_LOSS_T=30` as a source-revision defect and overstated first-run and emergency-lock failure modes. Unconditional production GO is not supportable until the live FCU configuration, external PX4 zero-speed patch, exact-baseline repeated field matrix, physical final-stop behavior, malformed-path rejection, and spray compensation contract are verified.

If live `COM_OF_LOSS_T` remains `30`, this conditional verdict immediately becomes **NO-GO for deployment** until that setting is corrected and tested.
