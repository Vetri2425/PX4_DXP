# Ref-Point Double-Scaling Bug — Caller Audit & Fix Report

**Date:** 2026-06-19
**Status:** Audit complete · Implementation plan approved
**Scope:** DXF → local-NED → GPS alignment pipeline (`path_engine`, `server/routes/path.py`, `server/models.py`, frontend `pathApi.ts`/`App.tsx`)

---

## 1. Summary

GPS ground-control reference points entering the alignment engine are scaled **twice** by the DXF
unit factor ("Gap A"). The points arrive from the `/entities` preview already in local-NED metres,
then the engine multiplies them by `ref_unit_scale` a second time. For any DXF whose units are not
metres (cm / mm / inch), the affine solver compensates with a wildly wrong `scale_fit`
(≈100 for a cm drawing), corrupting every aligned mission.

**Fix (one line of intent):** *delete the multiply, rename the fields.* Ref points are pre-scaled
metric NED at the route boundary; the engine must not re-apply `unit_scale`. Add a scale-sanity
guard on the affine result using `abs(scale_fit − 1.0)`.

---

## 2. Root cause

| Stage | What happens | Correct? |
|---|---|---|
| DXF parser | Geometry scaled by `unit_scale` → metres | ✅ |
| `/entities` preview | Emits points in local-NED metres | ✅ |
| Frontend tap → `RefPoint` | Sends local-NED metres + GPS lat/lon | ✅ |
| **Engine "Gap A"** (`_plan_from_segments`) | Multiplies ref points by `ref_unit_scale` **again** | ❌ double scale |
| Affine solver (`ned.py`) | Absorbs error into `scale_fit` | ❌ masks the bug |

The affine fit hides the error rather than catching it, because a 2-point similarity transform has
zero overdetermination — residuals (RMSE) are always ~0 regardless of how wrong the scale is.

---

## 3. The two corrections, verified

### 3.1 Scale guard — `abs(scale_val − 1.0)` is correct (not `scale × unit_scale − 1`)

| State | `scale_fit` | `abs(scale·unit_scale − 1)` (proposed earlier) | `abs(scale − 1.0)` (correct) |
|---|---|---|---|
| Bug present (cm DXF) | ≈ 100 | ≈ 0 → **silently passes** ❌ | ≈ 99 → **rejects** ✅ |
| Fixed (cm DXF) | ≈ 1.0 | ≈ 0.99 → **falsely rejects** ❌ | ≈ 0 → **passes** ✅ |

The `scale × unit_scale` form is the exact inverse of what the post-fix contract needs — it was
written to validate the *pre-fix* pipeline. After the fix, `unit_scale` is already consumed by the
DXF parser and is irrelevant to the affine fit. **`abs(scale_val − 1.0)` is the only correct check.**

### 3.2 RMSE is not a usable guard

Earlier claim "RMSE is in double-scaled DXF units" was wrong: residuals in
[`ned.py`](path_engine/ned.py) are `predicted_NED − reference_NED` in the target frame — always
metres. The real problem is structural: a 2-point similarity solve is exactly determined, so RMSE is
zero by construction. Even with ≥3 noisy points, a uniform unit error absorbed into `scale_fit`
moves all points together and leaves residuals near zero. **RMSE never reliably catches this class
of error — the scale guard must.**

---

## 4. Caller audit — complete

| Caller | Provides | Has bug? | Affected by fix? |
|---|---|---|---|
| Frontend → `/align` → engine | metric NED | **Yes** | Fixed |
| Frontend → `/plan-and-stage` → engine | metric NED | **Yes** | Fixed |
| Frontend → `/plan` → engine | metric NED | **Yes** | Fixed |
| `plan_segments()` → engine | no ref_points in any test | No | No change |
| `test_affine_scale_is_unity_when_ref_points_share_metric_frame` | calls math fn directly | N/A | Comment update only |
| All other tests | `unit_scale = 1.0`, blind | Not detectable | Unaffected |

### Notes

- **`plan_segments()` not affected** — calls `_plan_from_segments` without `ref_unit_scale`, so it
  defaults to `1.0` (no-op). Not called by any route or `path_manager.py` (zero hits); only tests
  call it, none passing `ref_points_dxf`.
- **Existing fixtures are blind** — every integration test that exercises a route with `ref_points`
  uses a metres DXF (`soccer_field_penalty_area.dxf`, `square_2x2.dxf`, both confirmed
  `unit_scale = 1.0`; `test_staged_endpoints.py` sets `$INSUNITS = 6`). `× 1.0` is a no-op, so the
  suite has **never** exercised the double-scaling path and stays green under both broken and fixed
  behavior.
- **The one cm-DXF test** ([`test_path_api.py:1040`](server/test_path_api.py:1040)) calls
  `dxf_to_ned_affine` directly, bypassing the engine. Its assertions hold regardless of Gap A — it
  validates the math function in isolation. Its "Gap A regression" docstring needs updating: scaling
  now happens at the parser (geometry) and the route boundary (ref points), not inside the engine.

---

## 5. Approved implementation plan

**Step 0 — No further audit needed.** The above is exhaustive for this codebase.

**Step 1 — Remove Gap A.** [`path_engine/engine.py`](path_engine/engine.py) — drop the
`ref_unit_scale` parameter from `_plan_from_segments` and delete the multiply (Gap A block). Replace
with:
```python
# ref_points_dxf already arrive in local-NED metres from the /entities preview.
metric_ref_points_dxf = list(ref_points_dxf) if ref_points_dxf else None
```
Remove the `ref_unit_scale` argument from both callers (`plan_file`, `plan_dxf_entities`).

**Step 2 — Rename model fields.** [`server/models.py`](server/models.py):
```python
class RefPoint(BaseModel):
    """Maps a local-NED preview point to a real-world GPS coordinate."""
    local_north_m: float   # metres north in the /entities preview frame
    local_east_m: float    # metres east in the /entities preview frame
    lat: float
    lon: float
```
Rename (not just re-document) to prevent recurrence.

**Step 3 — Fix route construction.** [`server/routes/path.py`](server/routes/path.py) (both
`/align`/`/plan` sites) — remove the accidental double-swap, use one shared helper:
```python
ref_points_local_ned = [
    (pt.local_north_m, pt.local_east_m) for pt in req.ref_points
] if req.ref_points else None
# pass as ref_points_dxf=ref_points_local_ned
```

**Step 4 — Add scale-sanity guard** (after the affine fit, route layer), using only `scale_val`:
```python
scale_error = abs(scale_val - 1.0)
if not math.isfinite(scale_val) or scale_val <= 0:
    raise HTTPException(422, "Alignment failed: non-finite scale")
if scale_error > SCALE_HARD_LIMIT:      # e.g. 0.25 → reject outside [0.75, 1.25]
    raise HTTPException(422, f"Alignment scale {scale_val:.4f} outside safe range")
elif scale_error > SCALE_WARN_LIMIT:    # e.g. 0.10 → warn outside [0.90, 1.10]
    warnings.append(f"Alignment scale {scale_val:.4f} is unusual")
```
Limits live in `config.py`.

**Step 5 — Frontend rename.** [`pathApi.ts`](../Three_Wheel_v2/src/api/pathApi.ts) `RefPoint` type
→ `local_north_m` / `local_east_m`; update [`App.tsx`](../Three_Wheel_v2/App.tsx) call sites. No
arithmetic change.

**Step 6 — Pipeline regression test.** New test calling the route (or `plan_path`) with cm / mm /
inch / unitless DXFs + metric NED ref points; assert `abs(scale_fit − 1.0) < 0.01`. Add a
single-point translation test (anchor lands within tolerance of the GPS target).

**Step 7 — Update docstring** on [`test_path_api.py:1040`](server/test_path_api.py:1040): the engine
no longer applies Gap A; callers provide pre-scaled metric NED.

**Step 8 (later)** — `unit_scale` in `/entities` response for diagnostics; support ≥3 ref points;
read-only aligned-geometry overlay (taps stay in preview frame).

---

## 6. Risk & verification

- **Behavioral change is confined to non-metre DXFs.** All existing tests use `unit_scale = 1.0`, so
  the suite alone cannot prove the fix — Step 6's multi-unit regression test is the gate.
- **Two-point alignment cannot self-validate** (zero residual). The scale guard (Step 4) is the only
  runtime defense; do not rely on RMSE.
- Controller / RPP untouched — this is planner + route + model + frontend only.
