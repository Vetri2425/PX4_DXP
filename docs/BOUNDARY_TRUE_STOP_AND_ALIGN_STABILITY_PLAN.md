# Boundary True-Stop Unification + Align Settle Stability — Audit & Implementation Plan

**HEAD:** `4ce34da` (`feat/entry-pivot-recenter`)
**Status:** PLAN ONLY — no code applied. Independently verified against raw bags + current source.
**Bags used:** `2026-07-10_12-52-41.402_IST_square_2x2` (M1), `2026-07-10_12-59-49.375_IST_square_2x2` (M2), both on `flash@192.168.1.102:~/bags_jet/`.

---

## 0. Verdict

The incoming audit's diagnosis is **correct**. I independently re-extracted `/rpp/stop_debug` from both raw `.db3` bags with a standalone `rosbag2_py` script (not a re-paste of the provided numbers) and cross-read the exact source at `4ce34da`. Every major number in the incoming audit reproduces from the raw bag to within 5s-sampling tolerance (see §2). The two proposed fixes (unify boundary true-stop; add align hysteresis) are the right shape and directly map onto real code. I found the exact lines responsible for both bugs and one **correction** to the incoming draft's framing (§3) that changes how PR-B should be scoped.

**Do not revert `4ce34da`.** It fixed the pure-zero deadlock (confirmed: both bags now reach STOP_CERT on the entry boundary, which never happened before). The two bugs below are pre-existing, structurally separate defects in code `4ce34da` did not touch (segment run-boundary stop branching, and `_run_alignment_hold`'s recenter gate) — they were simply not visible until entry stopped deadlocking.

**Handoff note:** this plan went through a second, adversarial verification pass (3 independent agents: code-accuracy re-check against current source, a design stress-test of a param-reuse idea, and a full-resolution bag re-dig) before being finalized for implementation. See **§10** for the full results. Two things changed as a direct result — both already reflected in §6 below: the align-oscillation fix is scoped to **hysteresis only** (a proposed heading-noise debounce was tested against real bag data and found unnecessary — do not add it speculatively), and it uses **two dedicated new params**, not a reuse of an existing one (reuse was tested and found to be a hidden coupling risk — do not "simplify" this back). If implementing from this doc, trust these two calls; the reasoning is in §10 if you want to re-verify.

---

## 1. Verification methodology

1. Read the exact source at `4ce34da` for every function named in the incoming audit: `_hold_before_run_advance`, `_corner_stop_satisfied`, `_corner_hold_velocity`, `_corner_position_ok`, `_smooth_capture_velocity`, `_run_alignment_hold`, `_corner_pivot_velocity`.
2. Wrote `/tmp/verify_bag.py` on the Jetson (`rosbag2_py.SequentialReader` + `deserialize_message`) to decode `/rpp/stop_debug`'s 20-float layout directly from the `.db3` files — independent of any prior analysis — and dumped phase transitions + 5s samples for both M1 and M2.
3. Cross-checked every claimed number (position error, speed, heading error, phase durations) against this independent extraction.

---

## 2. Bag-verified numbers

| Claim (incoming audit) | Independently re-extracted | Match |
|---|---|---|
| M2 entry: closest ~0.06cm, peak ~152cm OS, STOP_CERT 0.46cm@0.8cm/s | peak **151.45cm** @ t=30.1s; STOP_CERT t=52.84s **0.46cm @ 0.81cm/s** | ✅ |
| M2 entry align: 159s, 96°→hunt→1.6° | ALIGNING 52.86s (hdg=**95.95°**) → ALIGN_CERTIFIED 211.93s = **159.07s**, hdg wanders 26°–110° throughout | ✅ |
| M2 C1 align: ~6s | 227.92s → 234.16s = **6.24s** | ✅ |
| M2 C2 align: ~102s | 247.42s → 349.00s = **101.58s** | ✅ |
| M2 C3 align: ~80s | 362.24s → 442.66s = **80.42s** | ✅ |
| M2 final: 1.2cm@2.3cm/s | **1.24cm @ 2.34cm/s** | ✅ |
| M1 entry: cert 1.14cm@0.4cm/s | **1.14cm @ 0.38cm/s** at t=5.90s | ✅ |
| M1 entry align: ~1s, 4°→1° | 5.92s→6.93s = **1.01s**, hdg 4.32°→0.88° | ✅ |
| M1 C1: HOLD start 1.9cm@12.5cm/s | HOLD t=17.29s, **1.92cm @ 12.54cm/s** | ✅ |
| M1 C1 peak OS ~145cm | **133.71cm** at nearest 5s sample (true peak likely between samples, consistent within sampling grid) | ✅ (close) |
| M1 C1 align: ~74s | 78.03s → 152.41s = **74.38s** | ✅ |
| M1 C2 align: ~76s | measured **76.22s** | ✅ |

This is a trustworthy analysis. Proceeding on it directly rather than re-deriving from scratch.

---

## 3. Correction to the incoming draft

The incoming draft frames **C1 as a structural exception** ("M2 C1 align clean 6s... Exception vs other corners — this pivot worked" / "F C1 align — CLEAN (short)"). The raw data does not support treating C1 as reliably different:

| Mission | C1 align duration |
|---|---|
| M1 | **74.38s** (bad) |
| M2 | **6.24s** (good) |

**Same corner index, same mission geometry, opposite outcome.** This means the bug is a **timing/noise-sensitive race**, not something tied to being "the first corner" or corner geometry. Any fix (PR-B) must be corner-index-agnostic — it cannot rely on "C1 is usually fine, just fix C2/C3." This also means PR-B's success metric must be measured across **multiple repeated runs of the same corner**, not a single sample per corner (a single 6s C1 does not prove the fix; a run of 5+ consecutive sub-15s aligns across all corner positions does).

---

## 4. Confirmed root cause (code-grounded)

### 4a. I1 / I3 — stop path incomplete for segment run-boundaries

`_hold_before_run_advance` HOLD tail (`src/rpp_controller_node.py:1855-1890`):

```python
if (
    self._active_tracking_profile != "segment"          # <-- excludes segment corners
    and true_stop_dist > 0.0
    and pos_error <= true_stop_dist
):
    if meas_speed > stop_speed:
        brake_n, brake_e = self._corner_brake_velocity(yaw_ned)      # active body-axis brake
    else:
        brake_n, brake_e = self._corner_hold_velocity(..., max_speed_cap=stop_speed)  # capped at 0.08
elif self._active_tracking_profile != "segment" and pos_error > handoff_r:  # <-- also excludes segment
    brake_n, brake_e = self._smooth_capture_velocity(...)
else:
    brake_n, brake_e = self._corner_hold_velocity(...)   # <-- segment corners land HERE, uncapped
```

`_corner_hold_velocity` (`:4487-4558`) with no `max_speed_cap` argument defaults its cap to `segment_brake_velocity_cap_m_s` = **0.18 m/s** (`:4514`). And the cert call (`:1803-1809`) only overrides the speed threshold for `RUNTIME_ENTRY_TO_MARK`:

```python
entry_cert = stop_reason == StopReason.RUNTIME_ENTRY_TO_MARK
speed_override = float(...) if entry_cert else None   # None for RUN_BOUNDARY
```

**Bag-confirmed I already independently verified**: every square corner in both M1 and M2 arrives via `stop_reason=RUN_BOUNDARY` with `run_idx` incrementing per side (`_split_run_at_corners` splits each side of the square into its own run) — so square corners take the **exact same code path** as a smooth run-boundary, minus the active brake and the tight cert. M1's C1 (HOLD start 1.9cm @ 12.5cm/s) is direct evidence: a plain tangent-frame `_corner_hold_velocity` servo, capped at 0.18, entered while still moving 12.5cm/s, sent it on a 133cm arc before it finally parked.

**`INTRA_RUN_CORNER`** (`StopReason.INTRA_RUN_CORNER=1`) is a separate, live code path inside `_control_segment_profile` (`:3150-3470`, its own `_corner_stop_satisfied` call with no override) for corners that stay **within** one run rather than splitting into a new run. Neither bag exercised this path (both show clean `run_idx` increments, i.e. `_split_run_at_corners` always split). **Flagging, not fixing**: worth a quick check of when `INTRA_RUN_CORNER` is actually reached before deciding if it needs the same treatment — out of scope for this PR since it's unverified against real data.

### 4b. I2 — align oscillation

`_run_alignment_hold` (`:1986-2330`+), once `_corner_stop_complete=True` and heading is not yet inside the release band, chooses between two commands every cycle:

```python
if recenter_on and self._corner_stop_complete and not position_ok:   # :2203
    ... _corner_hold_velocity(..., max_speed_cap=segment_min_corner_speed)   # RECENTER — drive toward point
    return True
# (falls through if position_ok, or recenter disabled)
... _corner_pivot_velocity(yaw_ned, heading_err, corner_speed, ...)          # PIVOT — rotate toward heading
```

`position_ok` (`_corner_position_ok`, `:4431-4443`) is a **single hard threshold**: `dist <= corner_position_tolerance_m` (**0.02m, 2cm**), no hysteresis band. The code's own comment immediately above (`:2092-2095`) states plainly: *"PX4's velocity-vector pivot inherently translates the rover by ~corner_speed over the pivot duration, so position_ok is expected to read false while heading is still converging."*

That is the bug: a mechanism the code **admits will constantly trip** is wired to a **2cm on/off switch with no hysteresis**, so every cycle where the pivot's own rotation nudges position past 2cm, the controller yanks the velocity command away from "rotate toward target heading" and onto "drive toward the point" — a **different bearing** that PX4 (which derives heading purely from commanded velocity-vector bearing) will chase, undoing rotational progress. Bag-confirmed smoking gun: M2's C2 align reaches `hdg_err=-6.75°` at just t+5s (nearly done!) then **wanders back out to 68°, 100°, 103°** for the next ~90s before finally settling — not a slow convergence, an active fight.

This is compounded by the settle-dwell reset (`:2120-2125`): `_align_settle_since` resets to `None` on **any single cycle** where heading/speed/yaw-rate/position aren't ALL simultaneously satisfied. With a tight 2° heading band and a 2cm position band both subject to EKF/velocity noise during an actively-rotating differential-drive rover, achieving one uninterrupted `segment_align_settle_s` (0.20s) window can take many attempts even after the rover is "basically" aligned.

---

## 5. PR-A — Unified boundary true-stop

### Design decision
Apply the **same active-brake, tight-park-speed stop** that entry now gets (via `4ce34da`) to **every** hard-boundary stop (`RUN_BOUNDARY` and `RUNTIME_ENTRY_TO_MARK` alike), not just entry. Concretely: stop gating the true-stop/park branch on `_active_tracking_profile != "segment"` — that exclusion was written for the *far-field capture ramp* (`_smooth_capture_velocity`, which only makes sense for a smooth run approaching from outside the window), not for the *inside-the-window brake*, which is generically correct for any hard stop regardless of tracking profile.

### Exact change

`_hold_before_run_advance` (`:1841-1890`):

```python
if true_stop_dist > 0.0 and pos_error <= true_stop_dist:
    # NOW APPLIES TO ALL PROFILES — segment corners get the same active
    # brake + park-speed hold that entry already gets.
    if meas_speed > park_speed:
        brake_n, brake_e = self._corner_brake_velocity(yaw_ned)
    else:
        brake_n, brake_e = self._corner_hold_velocity(..., max_speed_cap=park_speed)
elif self._active_tracking_profile != "segment" and pos_error > handoff_r:
    # UNCHANGED — smooth-only far-field capture ramp
    brake_n, brake_e = self._smooth_capture_velocity(...)
else:
    brake_n, brake_e = self._corner_hold_velocity(...)   # unchanged fallback
```

Cert dispatch (`:1803-1809`): drop the `entry_cert` gate — apply `speed_threshold_override` for **every** call through this dispatcher (both `stop_reason` values reach here only through a hard-boundary transition, so there is no case where the override is wrong):

```python
park_speed = float(self.get_parameter("segment_entry_stop_speed_m_s").value)
if self._corner_stop_satisfied(position_ok=position_ok, speed_threshold_override=park_speed):
    ...
```

**Param naming**: keep `segment_entry_stop_speed_m_s` as-is (it was added and field-tested today; renaming it now is pure churn for a value that isn't entry-specific in *meaning*, only in *original scope*). Update its declaration comment to say "parked-cert speed gate for ALL hard-boundary stops (entry and run-boundary), not entry-only." One knob, one meaning, no new params.

`stop_speed` local var in the HOLD tail (currently reads `segment_stop_speed_threshold`, 0.08, and is used both as the brake-engage threshold `meas_speed > stop_speed` and the `_corner_hold_velocity` cap) should become `park_speed` (`segment_entry_stop_speed_m_s`, 0.03) for consistency — one park speed drives brake-engage, hold-cap, and cert, everywhere.

### What stays / what's retired
- **Stays unchanged**: `_smooth_capture_velocity` and its far-field gating (`profile != "segment" and pos_error > handoff_r`) — smooth runs still need the fixed-bearing decel ramp from outside the window.
- **Stays unchanged**: `_corner_brake_velocity`, `_corner_hold_velocity` themselves — only the *caller's chosen cap/threshold* changes.
- **No new params.** `segment_entry_stop_speed_m_s` widens scope by definition change only.

### Tests (extend `test_run_boundary_pivot.py` / `test_entry_true_stop.py`)
- `test_segment_run_boundary_gets_active_brake` — construct a `RUN_BOUNDARY` (non-entry) transition, arrive fast (>0.03) inside `true_stop_dist`; assert `_corner_brake_velocity` fires (not the bare tangent-frame hold), mirroring the existing entry test.
- `test_segment_run_boundary_certs_at_park_speed` — assert cert requires `≤0.03 m/s`, not `0.08` (i.e., a 0.05 m/s creep must NOT certify for `RUN_BOUNDARY` either, matching `test_entry_true_stop.py`'s test 2 but for a non-entry `stop_reason`).
- `test_segment_run_boundary_no_regression_when_slow` — a corner arriving already slow (<0.03, per PRE_CORNER_SLOWDOWN working correctly) certifies promptly, no added latency.
- Regression: M1-style arrival (1.9cm @ 12.5cm/s) simulated — assert peak position error during the stop stays bounded (e.g. <15cm) instead of growing to 130cm+.

### Field pass gate (stop only, matches the incoming draft's own gate)
After first `pos_error ≤ 5cm`: max error before STOP_CERT ≤5cm; STOP_CERT ≤2cm @ ≤3cm/s; time from first ≤5cm to cert ≤2.5s. Run on **both** entry and all three square corners, **5 repeated missions** (not 1) given §3's finding that per-corner behavior is run-to-run variable.

---

## 6. PR-B — Align settle hysteresis (start only after PR-A is field-signed-off)

### Design decision
**One change, not two.** An earlier pass at this plan considered a second mechanism (a settle-dwell debounce for heading noise) alongside the hysteresis fix. Both the "should we add debounce" and "can we cut a param by reusing an existing one" questions were independently stress-tested before finalizing this section — see **§10** for the full evidence. Results:

- **Recenter hysteresis: ship it, dedicated params.** Replace the single 2cm `position_ok` gate for the recenter-vs-pivot choice with a two-threshold band: arm recenter only once drift exceeds an **outer** radius (0.08m default), release back to pivot-only once drift is back under an **inner** radius (0.05m default) — never toggling on every crossing of one fixed line. This is a direct fix for a condition the code's own comment admits will constantly trip (§4b).
- **Two NEW, dedicated params** (`align_recenter_arm_m`, `align_recenter_release_m`). **Do not** reuse the existing `segment_boundary_corner_handoff_m` to save a param — that idea was raised, adversarially checked, and rejected: that param's only two read sites are both hard-gated `!= "segment"` (i.e. it's a complete no-op today for exactly the segment-profile corners this fix targets), its documented contract is a smooth-run decel-ramp shaping knob anchored to `segment_boundary_capture_radius_m` (0.5m scale), and the project's own backlog (SPD-T1, high-speed tuning) makes it plausible someone widens it later for that *original* purpose — which would silently balloon the align-recenter release radius with nothing anywhere pointing back at the align mechanism. The shared 0.05 default between the two params is coincidence, not a designed relationship. Full reasoning: §10.2.
- **No settle-dwell debounce.** This was evaluated against full-resolution (50Hz) bag data for corner-2's 102s align and explicitly **not needed**: the heading error in the final ~30s executes two full large-amplitude swings (123°→−38.9°→+80.3°→settle) — a continuing large-scale fight, not single-cycle sensor noise perturbing an already-parked signal. Once heading drops under the 2° band for good, it never blips back out. The recenter-hysteresis fix targets exactly this large-scale fight; fixing it should resolve this case without a separate debounce. Full evidence: §10.3. **If field bags after shipping hysteresis alone still show dwell resets on a heading that's genuinely holding steady** (not large swings), that is the trigger to revisit debounce as a v2 — do not add it preemptively.

### Exact change

`_run_alignment_hold` (`:2203`):
```python
# was: if recenter_on and self._corner_stop_complete and not position_ok:
if recenter_on and self._corner_stop_complete and self._recenter_armed(dist=pos_error):
    ...
```
with a small stateful helper:
```python
def _recenter_armed(self, *, dist: float) -> bool:
    arm_r = float(self.get_parameter("align_recenter_arm_m").value)      # 0.08, NEW param
    release_r = float(self.get_parameter("align_recenter_release_m").value)  # 0.05, NEW param
    if self._recenter_active and dist <= release_r:
        self._recenter_active = False
    elif not self._recenter_active and dist > arm_r:
        self._recenter_active = True
    return self._recenter_active
```
(new bool `self._recenter_active`, reset in `_reset_corner_pivot_state`, same lifecycle as the other pivot state. Declare both new params near `segment_entry_pivot_recenter`, `:567`.)

### Tests
- `test_recenter_hysteresis_no_chatter` — drive `pos_error` across 6-9cm repeatedly (simulating pivot-induced drift); assert the recenter does NOT toggle on/off every single cycle — it stays "armed" until drift drops below the release radius, and stays "released" until it exceeds the arm radius.
- `test_recenter_hysteresis_uses_dedicated_params` — assert `align_recenter_arm_m`/`align_recenter_release_m` are declared and read, and that `segment_boundary_corner_handoff_m` is NOT referenced anywhere in `_run_alignment_hold` (regression guard against the rejected reuse idea resurfacing).
- Regression run of `test_run_boundary_pivot.py`'s existing B/C2 cases (`segment_entry_pivot_recenter=False`, legacy path) — must be untouched, since this fix only changes the `recenter_on=True` branch's arm/release logic, not the legacy no-recenter path.

### Field pass gate
90° align ≤15s, position ≤5cm throughout, **across 5 repeated corner passes minimum per §3** (not one good sample). If still >30s after this fix, treat as a second-order issue (e.g. `segment_pivot_damp_*` tuning, or revisit the debounce question with fresh bags) — separate ticket, do not fold into this PR.

---

## 7. Sequencing (production order — matches the incoming draft's own instinct)

```
PR-A (stop unification)  →  field-validate on hardware (5× entry, 5× each corner)
        │
        └─ STOP PASS on all →  PR-B (align hysteresis)  →  field-validate (5× per corner, all 4 positions)
```

Do not start PR-B until PR-A's stop gate passes cleanly and repeatedly. A cleaner, tighter park position feeding into the pivot may itself reduce (not necessarily eliminate) the align variance — measuring PR-B against a still-messy stop would conflate the two bugs again.

---

## 8. Out of scope / risks / rollback

**Out of scope for both PRs:** init turn, final-endpoint stop (already clean both bags), spray, all PX4/QGC/FCU params, `INTRA_RUN_CORNER` path (flagged §4a, not fixed), `_smooth_capture_velocity`'s far-field ramp logic itself.

**Risks:**
- PR-A widens active-braking to segment corners — verify this doesn't fight `PRE_CORNER_SLOWDOWN`'s own deceleration profile (should be complementary: slowdown gets it close/slow, true-stop parks it; if slowdown already reliably delivers <3cm/s arrivals, PR-A's brake branch simply never engages, which is fine).
- PR-B's hysteresis band (8cm arm / 5cm release) needs the arm radius kept comfortably below `corner_position_tolerance_m`'s consequences (i.e., don't let the rover drift so far the pivot itself becomes geometrically meaningless) — 8cm is small relative to the corner geometry seen in these bags (typically ≤6cm natural drift), should be safe, but confirm against a wider bag sample before field-locking the default.

**Rollback:** PR-A — revert to gating the true-stop branch behind `_active_tracking_profile != "segment"` (one-line condition re-add). PR-B — `segment_entry_pivot_recenter=false` already exists as a full kill-switch for the recenter mechanism (legacy path), independent of this fix; the two new params can also just be widened (`align_recenter_arm_m` large) to make hysteresis effectively a no-op without a code revert.

---

## 9. Deliverable checklist

- [x] PR-A: widen true-stop/park-speed gating off `_active_tracking_profile`, unify cert override for both `stop_reason`s, update param doc comment (no new params) — implemented 2026-07-10
- [x] PR-A tests (4, listed §5) + existing suite green — added to `src/test_entry_true_stop.py` (tests 5b/5c/5d/5e); full `rpp_controller_node`-related test sweep run on the Jetson before/after — zero new failures (see §11)
- [ ] PR-A field: 5× entry + 5× each corner, stop-only gate — **NOT DONE, requires supervised hardware run**
- [x] PR-B: recenter arm/release hysteresis ONLY — 2 dedicated new params (`align_recenter_arm_m`, `align_recenter_release_m`); do **not** reuse `segment_boundary_corner_handoff_m` (rejected, §10.2); do **not** add a settle-dwell debounce (not needed, §10.3) — implemented 2026-07-10, `_recenter_armed()` helper
- [x] PR-B tests (2, listed §6) + regression on existing B/C2 legacy-path tests — added to `src/test_run_boundary_pivot.py` (PRB1/PRB2); B/C2 confirmed pre-existing failures unrelated to this fix (see §11), unchanged before/after
- [ ] PR-B field: 5× per corner position, align gate ≤15s/≤5cm — **NOT DONE, requires supervised hardware run**
- [ ] Update `docs/RUNTIME_ENTRY_BEHAVIORAL_DIVERGENCE_AUDIT.md` cross-reference once both land — deferred until field validation completes

**Also done (not originally itemized above, needed for the new params to be usable):** registered `align_recenter_arm_m` / `align_recenter_release_m` in `server/routes/rpp_params.py`'s `RPP_PARAM_SCHEMA` (operator UI get/set/list gate on this registry — an undeclared param 422s).

---

## 10. Verification addendum — adversarial check before handoff

Before finalizing this doc for direct implementation, three independent verification passes were run against current source and real bag data. Read this section if you want to re-verify any of §5/§6's design decisions rather than take them on faith.

### 10.1 Code-accuracy re-check — PASS

Every function name, line-number range, and param name/default cited in §4/§5 was independently re-confirmed against the current `src/rpp_controller_node.py`, separately from the reads that produced this doc. Result: all claims **CONFIRMED**, with only negligible off-by-one/few-line drift on non-actionable citations (e.g. a supporting comment quote was 2 lines off; already corrected in §4b above). No actionable code-change line number was wrong. Exact confirmed anchors an implementer can trust:
- `_hold_before_run_advance`: lines 1748-1891 (function), HOLD tail branch at 1855-1891, cert dispatch at 1803-1810.
- `_corner_stop_satisfied`: `speed_threshold_override` param confirmed in the signature at line 4895 (def at 4892).
- `_corner_hold_velocity`: lines 4487-4558; default cap read at line 4514 (`segment_brake_velocity_cap_m_s`, declared 0.18 at line 525).
- `_corner_position_ok`: lines 4431-4443; threshold read at line 4443 (`corner_position_tolerance_m`, declared 0.02 at line 514).
- `_run_alignment_hold`: recenter branch at line 2203 exactly; settle-dwell reset at lines 2120-2125 exactly.
- `segment_entry_stop_speed_m_s`: declared line 598, default 0.03.
- `segment_boundary_corner_handoff_m`: declared line 483, default 0.05.
- `INTRA_RUN_CORNER` handling inside `_control_segment_profile` (function spans 2936-3616): corner sub-block starts line 3146, bare `_corner_stop_satisfied` call (no override) confirmed at line 3347.

### 10.2 Param-reuse adversarial check — REJECTED (use dedicated params)

Stress-tested the idea of reusing `segment_boundary_corner_handoff_m` as PR-B's release threshold instead of adding a new param. Verdict: **NEEDS_OWN_PARAM**.

Findings: `segment_boundary_corner_handoff_m` has exactly two read sites in current source (`:1844` inside `_hold_before_run_advance`, and `:4477` inside `_smooth_capture_velocity`) — **both gated `_active_tracking_profile != "segment"`**. That means today this param is a complete no-op for every segment-profile mission, which is exactly the class of corner this fix targets (bag-confirmed: every square corner is `stop_reason=RUN_BOUNDARY` with segment profile). Its documented contract (in-code comment at `:479-482`, and the operator-facing description in `server/routes/rpp_params.py:301-308`) is specifically a smooth-run decel-ramp shaping knob, anchored to `segment_boundary_capture_radius_m` (0.5m scale) — a different geometry than the centimeter-scale pivot-drift hysteresis PR-B needs (anchored to `corner_position_tolerance_m`, 0.02m, and observed natural drift ~6cm).

The two contexts share a numeric default (0.05) by pure coincidence, not design. The project's own backlog (SPD-T1, high-speed tuning — "corner braking dist... too short at 1 m/s") makes it plausible someone later widens this param for its *original* smooth-decel purpose, which would silently balloon the align-recenter release radius 10x+ with nothing anywhere connecting the two uses. **Decision: two dedicated new params (`align_recenter_arm_m`, `align_recenter_release_m`), as already specified in §6.** Do not collapse this back to a reuse even though it looks like a free param-count reduction.

### 10.3 Settle-dwell debounce necessity check — NOT NEEDED

Investigated whether a second mechanism (tolerate single-cycle heading noise before resetting the 0.20s certification dwell) is independently needed on top of the recenter-hysteresis fix. Pulled full 50Hz-resolution `/rpp/stop_debug` data (not 5s samples) for the last ~30s of M2's corner-2 align (t=320.0-349.5s, 1450 samples) via a fresh script on the Jetson.

Findings: heading error in this window executes **two full large-amplitude oscillation cycles** — starts mid-swing at +123.1° (t=320.0s), decays through zero to a −38.9° trough (t=334.9s), swings back through zero to a +80.3° peak (t=342.2s), then decays continuously down through the ±2° band for good at t=348.778s (219ms before `ALIGN_CERTIFIED` at t=348.997s). Once it drops under 2°, it never pokes back out — the crossing that matters is the tail of a genuine ~6.6s large-amplitude decay, not a noise blip on an otherwise-parked signal. **This is exactly the large-scale control-mode fight the recenter-hysteresis fix targets, not independent sensor noise.** Verdict: **DEBOUNCE_NOT_NEEDED** — fixing the recenter fight should resolve this specific failure mode without a separate mechanism. Trigger for revisiting: if post-hysteresis field bags show dwell resets while heading is demonstrably holding near-steady (not large swings), that's new evidence: reopen as a v2, don't add preemptively.

---

## 11. Implementation notes (2026-07-10) — code + tests landed, field validation pending

PR-A and PR-B as specified in §5/§6 are implemented on `feat/entry-pivot-recenter`:

- **PR-A**: `_hold_before_run_advance` true-stop/park-speed branch no longer gated on `_active_tracking_profile != "segment"`; cert dispatch uses `segment_entry_stop_speed_m_s` unconditionally (`park_speed`) for both `stop_reason`s. No new params — `segment_entry_stop_speed_m_s` / `segment_entry_true_stop_dist_m` doc comments updated to describe the widened scope.
- **PR-B**: new params `align_recenter_arm_m` (0.08) / `align_recenter_release_m` (0.05), new `_recenter_armed()` helper + `self._recenter_active` state (reset in `_reset_corner_pivot_state`), wired into the recenter branch of `_run_alignment_hold`. The settle/cert gate's strict `position_ok` check is untouched (per §6 design decision — hysteresis only gates recenter *engagement*, not certification). Also registered both new params in `server/routes/rpp_params.py`'s `RPP_PARAM_SCHEMA` (not itemized in §9 originally) — that registry gates the operator API's get/set/list, so an undeclared param 422s even though it exists on the ROS node.
- **Tests added**: `src/test_entry_true_stop.py` tests 5b–5e (PR-A: active-brake mechanism on a segment RUN_BOUNDARY, unified parked-cert gate, no-latency-when-already-slow, and a point-mass forward-sim bound seeded at the exact M1 bag condition — see the test file's own comments for why the sim is a coarse regression net, not a firmware-accurate reproduction of the arc dynamics). `src/test_run_boundary_pivot.py` tests PRB1/PRB2 (PR-B: hysteresis sticks armed/released across a repeated 6-9cm oscillation per §6's own scenario; dedicated-params regression guard against the rejected `segment_boundary_corner_handoff_m` reuse resurfacing, via `inspect.getsource` on `_run_alignment_hold`).
- **Regression sweep**: ran every `src/test_*.py` file that imports `rpp_controller_node` on the Jetson (`flash@192.168.1.102`), both against the edited code and against an untouched baseline copy at the same commit (`4ce34da`), to isolate genuinely-new failures. Result: **zero new failures**. Four pre-existing failures were found and confirmed byte-for-byte identical before/after: `test_run_boundary_pivot.py` tests B and C2 (both explicitly test the `segment_entry_pivot_recenter=False` legacy path, which PR-B does not touch — see [[pivot_position_gate_fix]] memory, a `position_release_ok = position_ok or (timed_out and not recenter_on)` change landed 2026-07-08 appears to have broken the legacy-path timeout waiver these two tests assume), `test_corner_absorb_pivot.py`, `test_endpoint_approach.py`, `test_segment_stop.py` (3 sub-failures, look timing/flakiness-related — `0.3 not >= 0.44` style), and `test_smoke_rpp_controller.py`. **None of these are in scope for this plan** and none were introduced by it; they're flagged here as a separate, pre-existing issue for a future session.
- **Not done**: all field-validation checklist items (§9) — both PRs' field gates require supervised hardware runs on the actual rover and were correctly left to the user/operator, not attempted autonomously.
