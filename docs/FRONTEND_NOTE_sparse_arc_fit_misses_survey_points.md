# Frontend note — sparse arc fit displaces surveyed points off the marked path

**Repo:** `Three_Wheel_v2` · **File:** `src/utils/roadMarkingCsvPath.ts`
**Date:** 2026-07-30 · **Severity:** high — the rover paints up to 7 cm off the surveyed marks on a ±2 cm spec

---

## Symptom

Operator surveyed a 6-point curve (`curve_6_points-1`), ran it 3 times, and the paint does not
land on the physical marks. Endpoints are hit; the four interior marks are missed.

Measured from the flight bags (absolute lat/lon, not local frame):

| surveyed point | absolute miss, 3 runs |
|---|---|
| pt0 (first) | 0.49 / 0.99 / 0.49 cm |
| pt1 | 2.63 / 3.68 / 3.68 cm |
| pt2 | 2.00 / 2.00 / 2.20 cm |
| pt3 | 1.32 / 2.73 / 0.31 cm |
| pt4 | 1.55 / 7.09 / 2.97 cm |
| pt5 (last) | 1.09 / 1.09 / 1.09 cm |

Straight-line missions (2 surveyed points) are unaffected — 0.00 cm. Only curves show it,
because a 2-point line has no interior points to displace.

## Not the rover, not the backend

Both were ruled out by measurement before looking at the app:

- **Controller:** tracks the path it is given to 1–2 cm RMS. Verified against `/path` in the bags.
- **Backend** (`POST /api/path/plan-trajectory`): stages the app payload verbatim. Every
  app-supplied point lies **0.00 cm** from the emitted waypoint polyline. `alignment_metadata`
  was `{method: "gps_origin", rotation_deg: 0, offset_n: 0, offset_e: 0, rmse: 0}` — no fit,
  no rotation, no scaling applied downstream.

The displacement is present in the payload the app sends.

## Cause

`trySparseArcFit` (line **1004**) → `fitCircleThroughEndpoints` (line **910**) →
`buildSparseArcSamples` (line **1064**).
Call site: line **2289** in `buildRoadMarkingFittedPath`. Entry: `localPointCsv.ts:920`.

`fitCircleThroughEndpoints` pins the **first and last** points and least-squares only the
interior ones — line **935**:

```ts
for (let i = 1; i < n - 1; i++) {
  const d = Math.hypot(points[i].north - cn, points[i].east - ce) - r;
  sum += d * d;
}
```

The surveyed polyline is then **replaced** by samples on that circle
(`buildSparseArcSamples`). Interior marks are wherever the circle happens to pass.

### Reproduced exactly

Re-implementing the same algorithm against the 6 surveyed points:

- fitted circle: centre `(2.2745, 0.7263)`, **R = 2.3876 m**
- radial residuals: `0.00  2.61  1.75  1.94  2.54  0.00` cm
- the 116 staged waypoints lie on that circle to **mean 5.8 mm / max 8.9 mm**

Confirmed independently by three signatures in the staged geometry: endpoints exact;
perpendicular deviation alternating in sign (`0, +1.62, −2.63, +1.31, −3.33, 0` cm);
vertex turns evened out while total turn is preserved (`21.0/25.6/7.5/28.4°` →
`25.2/19.3/14.8/22.2°`, sum 82.6° → 81.5°).

### Why every gate passed

| gate | value | this curve | outcome |
|---|---|---|---|
| `SPARSE_ARC_CORNER_TURN_DEG` (line 144) | 30° | max turn 28.4° | accepted as ONE arc |
| endpoint constraint (line 935) | pins 2 points | interior least-squared | interior free to drift |
| `sparseArcResidualGateM` (line 188) | `clamp(4 × RMS, 0.02, 0.15)` = **6.8 cm** | 2.61 cm | accepted, 2.6× margin |

The residual gate is `SPARSE_ARC_RESIDUAL_RMS_MULTIPLE = 4` × the survey's reported 1.7 cm RMS.
**The marking spec is ±2 cm. The gate is 6.8 cm.** A fit that misses every interior mark by more
than spec is inside the gate by a wide margin.

## What to change

**Make the fit interpolate every surveyed point, not just the two endpoints.**

Replace the single endpoint-constrained circle with a curve that passes through all points —
a **biarc chain** or a **G1 arc spline** through consecutive points. Each mark is then hit by
construction, and smoothness between marks is preserved. Keep `buildSparseArcSamples`'
tessellation-budget logic; only the underlying curve changes.

## What NOT to change

**Do not simply tighten the gate.** Lowering `SPARSE_ARC_RESIDUAL_RMS_MULTIPLE` (line 155) or
capping at 2 cm instead of `CORNER_TOLERANCE_M` (line 172) rejects this fit and drops it into
`buildWaypointFilletPath`. The existing docstring at lines **162–170** already documents why that
is worse: fillet radius comes from a local turn angle — a second difference of noisy position —
and on this same file the implied per-vertex radius swung **1.48 → 4.90 → 1.48 m** against a true
~2.4 m curve. That is the reported "jiggle".

Tightening the gate is only safe **after** the fallback is an interpolating fit.

## Decision needed before coding

The current code assumes the surveyed points are **noisy samples of a smooth curve**, so
smoothing through them is noise removal. The operator is treating them as **the marks that must
be hit**. Both readings are defensible; they are not compatible.

- Points are truth → interpolate (the change above).
- Points carry 1.7 cm of real RTK noise → forcing the curve through them bakes that noise into
  the paint, and no fit gives both.

Confirm which before implementing, because it decides the fix.

## Acceptance test

On `curve_6_points-1` (6 points, reported RMS 1.7 cm), assert the max distance from **each**
surveyed point to the emitted polyline is ≤ 1 cm, and that max turn between consecutive emitted
samples stays within the rover's `R_MIN_ROVER_M = 0.5 m` capability. Existing sparse-arc tests
live in `src/utils/roadMarkingCsvPath.sparseArc.test.ts`.
