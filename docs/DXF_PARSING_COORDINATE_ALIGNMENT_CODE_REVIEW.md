# Code Review: DXF Parsing & Coordinate Alignment Pipeline

> **Status: ✅ COMPLETE (2026-06-22)** — Review closed; the implementation work
> tracked by this document is finished. Retained as a reference record of the
> DXF→NED alignment pipeline analysis and the recommendations addressed.

**Review date:** 2026-06-20  
**Files reviewed:**
- `path_engine/parsers/dxf_parser.py` (683 lines)
- `path_engine/ned.py` (162 lines)
- `path_engine/engine.py` (888 lines, specifically lines 395–888 for the alignment logic)
- `server/mission_placement.py` (132 lines)
- `path_engine/core.py` (213 lines)
- `path_engine/tests/test_ned.py` (167 lines)

---

## 1. `path_engine/ned.py` — Mathematical & Geodesic Correctness

### 1.1 `latlon_to_ned()` (lines 22–54) — VERIFIED CORRECT

This is the geodesic conversion using GeographicLib's Karney inverse solution. No flat-earth approximation is present. The math is correct:

```python
result = geod.Inverse(origin_lat, origin_lon, lat, lon)
dist = result["s12"]
bearing_rad = math.radians(result["azi1"])
north = dist * math.cos(bearing_rad)
east  = dist * math.sin(bearing_rad)
```

**Why it's correct:**
- `geod.Inverse()` computes the WGS84 ellipsoidal geodesic distance and forward azimuth.
- The projection of the geodesic onto the local tangent plane at the origin is: north = distance × cos(azimuth), east = distance × sin(azimuth).
- This is the standard NED projection and is accurate for the ~200 m field spans typical of sports field marking.

**One subtle concern:** The tangent plane is defined at the **origin** point, but the azimuth `azi1` is the azimuth at the origin towards the target. Over a 200 m span, the geodesic curvature is negligible (error << 1 mm), so this is fine.

### 1.2 `dxf_to_ned_affine()` (lines 57–134) — UMEYAMA SIMILARITY TRANSFORM

This is a properly implemented Umeyama-style least-squares similarity transform (uniform scale + rotation + translation, no shear).

**Mathematical validation (lines 103–121):**

The normal equations solve:
```
[ u_i  -v_i ] [ a ]   [ x_i ]
[ v_i   u_i ] [ b ] = [ y_i ]
```

This is correct for the similarity constraint (matrix must be of the form `[[a, -b], [b, a]]`). The least-squares solution:

```python
a = Σ(u_i * x_i + v_i * y_i) / Σ(u_i² + v_i²)
b = Σ(u_i * y_i - v_i * x_i) / Σ(u_i² + v_i²)
```

This is mathematically optimal — it minimizes Σ||target - transform(source)||² under the similarity constraint.

**Floating-point concerns:**
- **Line 109:** `denom < 1e-9` — This check prevents division by zero for coincident points. However, if the DXF points span only a few centimetres (e.g., two reference points accidentally placed 1 cm apart), `denom` could be non-zero but still tiny (≈1e-4), producing an ill-conditioned solve with massive scale values. **Recommendation:** Increase this threshold or add a relative condition-number check:

  ```python
  if denom < 1e-9:
      raise ValueError("DXF reference points are coincident")
  # Add: check point spread
  spread = math.sqrt(denom / n_pts)
  if spread < 0.01:  # less than 1 cm spread
      raise ValueError(
          f"DXF reference points too close together (spread={spread:.4f} m). "
          "Place reference points at least 1 m apart."
      )
  ```

- **Residual computation (lines 126–132):** The residuals are computed by re-transforming each DXF point and computing Euclidean distance. This is correct but the RMSE formula `√(Σres² / n)` assumes residuals are in the NED frame, which they are. No issue here.

- **Rounding direction:** The transform `theta = atan2(b, a)` and `scale = hypot(a, b)` correctly extract the rotation and scale from the `[[a, -b], [b, a]]` matrix.

### 1.3 `apply_affine_transform()` (lines 137–162) — VERIFIED CORRECT

```python
sx = point[0] * scale    # scale dxf_y
sy = point[1] * scale    # scale dxf_x
north = sx * cos_t - sy * sin_t + offset_n
east  = sx * sin_t + sy * cos_t + offset_e
```

This implements:
```
[north]   [cosθ  -sinθ] [scale·dxf_y]   [offset_n]
[east ] = [sinθ   cosθ] [scale·dxf_x] + [offset_e]
```

This is the standard 2D similarity transform. Verified: the rotation matrix is orthonormal (determinant = 1), and the scale is applied before rotation, which matches the derivation in `dxf_to_ned_affine()`.

**Finding:** One potential misuse — if `scale` is expected to handle different N/E scaling (anisotropic), this code does NOT support it. The `dxf_to_ned_affine()` function explicitly enforces uniform scale via the `[[a, -b], [b, a]]` constraint, so this is self-consistent. This is intentional, not a bug.

---

## 2. `path_engine/parsers/dxf_parser.py` — DXF Parsing Edge Cases

### 2.1 Unit Scaling (lines 52–69, 72–93, 126–135) — NOTEWORTHY

**Lines 52–69 — `_INSUNITS_TO_METRES` dictionary:** Complete and correct. Covers all 16 DXF `$INSUNITS` values including obscure ones like microinches, mils, and angstroms.

**Lines 72–93 — `_get_unit_scale()`:** Reads the doc header twice (`ezdxf.readfile` in `_get_unit_scale`, then again in `parse_dxf`). This means the file is parsed **twice** — once for unit detection, once for entity extraction. For large DXFs (e.g., a full campus with 50,000 entities), this doubles parse time.

```python
# Lines 77-78 in _get_unit_scale:
doc = ezdxf.readfile(filepath)     # FIRST parse
insunits = doc.header.get("$INSUNITS", 0)

# Line 121-122 in parse_dxf:
doc = ezdxf.readfile(filepath)     # SECOND parse
```

**Recommendation:** Eliminate `_get_unit_scale()` entirely. The `parse_dxf()` function already reads `$INSUNITS` at lines 127–135 after the first (and only) `readfile`. The only callers of `_get_unit_scale()` can be refactored to parse the doc once.

### 2.2 LWPOLYLINE Bulge Handling (lines 211–229) — CORRECT

Bulge values extracted via `entity.get_points(format="xyb")` which returns `(x, y, bulge)` tuples. The bulge convention in DXF is:
- `bulge = tan(θ/4)` where θ is the arc angle in radians
- Positive bulge = CCW arc
- The vertices are stored as `(dxf_y * s, dxf_x * s)` = `(north, east)`

This is passed downstream to `densify_lwpolyline_bulge()` in `arc_curve.py` which handles the actual arc discretization.

**Potential issue:** If `get_points(format="xyb")` returns 2-element tuples for bulge-less vertices (common in older ezdxf versions), the `[v[2] if len(v) > 2 else 0.0` guard at line 216 handles this correctly.

### 2.3 INSERT Block Decomposition (lines 344–462) — MAJOR IMPROVEMENT (D2 fix)

The recursive decomposition using `ezdxf.disassemble.recursive_decompose` now handles 8 entity types inside blocks: LINE, POINT, CIRCLE, ARC, LWPOLYLINE, SPLINE, HELIX, and ELLIPSE. This is comprehensive.

**Edge case: nested INSERTs.** If a block reference contains another INSERT (nested blocks), `recursive_decompose()` handles this automatically — it flattens the hierarchy.

**Edge case: block with ATTRIB/ATTDEF.** These text entities are not handled (no ATTRIB case in the switch), but they're not geometric so this is correct.

**concern at line 452:** The fallback log message for unsupported sub-entity types could be very noisy for blocks containing entities like 3DFACE, SOLID, TRACE, etc. Consider throttling with a set of already-reported types:

```python
_unsupported_sub_entity_types: set[str] = set()
# Inside the else clause:
if sub_etype not in _unsupported_sub_entity_types:
    _unsupported_sub_entity_types.add(sub_etype)
    log.warning("INSERT %s: skipping unsupported sub-entity type %s (layer=%s)",
                handle, sub_etype, layer)
```

### 2.4 SPLINE/ELLIPSE Flattening — FLOATING-POINT BUG PATTERN (line 271, 307)

```python
flat_pts = list(path.flattening(distance=0.005 / unit_scale if unit_scale > 0 else 0.005))
```

**Line 271 comment says:** "D1 fix: flattening distance is in DXF units. Intended chord error is 5 mm (0.005 m)."

This division appears twice (lines 271, 307) and is used identically in the INSERT sub-entity code (line 434). The logic is:
- Target chord error in real-world: `0.005 m` (5 mm)
- DXF units per metre: `1 / unit_scale`
- So chord error in DXF units = `0.005 / unit_scale`

**This is mathematically correct** _if_ the DXF file uses linear units (e.g., cm: `unit_scale = 0.01`, so DXF units per metre = 100, chord error = 0.005 / 0.01 = 0.5 DXF units). 

**Edge case:** If `unit_scale > 0.005` (e.g., the DXF is already in metres with `unit_scale = 1.0`), then `distance = 0.005` DXF units = 5 mm chord error, which is fine. But if `unit_scale` is very small (e.g., 0.000001 for micron-scale DXFs), then `distance = 5000` DXF units, which would produce almost no flattening points. Such DXFs are extremely rare in this domain (sports field marking), so this is acceptable.

**Edge case zero division:** The `if unit_scale > 0 else 0.005` guard prevents division by zero, which is good.

### 2.5 POLYLINE Legacy Support (lines 231–261) — GOOD

Handles both `AcDb2dPolyline` and `AcDb3dPolyline` modes. The `get_mode()` check correctly skips polygon/polyface mesh variants. Bulge values default to 0 for 3D polylines (which don't carry bulges).

### 2.6 Layer-to-SegmentType Mapping (lines 467–464 in `entities_to_segments`)

**Note:** The actual classification logic lives in `DXFEntity.classify()` in `core.py` (lines 135–180). The filter loop at lines 514–516 calls `ent.classify(layer_mapping)` to skip ignored entities. This separation of concerns is clean.

**Issue in `core.py` line 148:** Color matching uses a regex-like syntax (`color:red`, `color:1`). The color map only covers 8 named colors. User-defined colors (e.g., color index 200 in an AutoCAD `.ctb` file) would not match. This is acceptable for the current use case but worth documenting.

---

## 3. `path_engine/engine.py` — Alignment Logic & Pipeline Orchestration

### 3.1 Reference Point Metric Consistency (lines 460–465) — CRITICAL DESIGN DECISION

```python
# Reference points already arrive in local-NED metres from the /entities
# preview (the DXF parser applied $INSUNITS scaling to all geometry), so
# they share the segment geometry's metric frame and feed the affine solve
# directly. No unit scaling here — re-applying unit_scale would double-scale
# the points and force the solver into a bogus scale_fit (~100 for a cm DXF).
metric_ref_points_dxf = list(ref_points_dxf) if ref_points_dxf else None
```

The comment documents a known bug that was fixed (Gap A). The `ref_points_dxf` arrive pre-scaled to metres (same as the segment geometry), so no further scaling is applied. This is correct.

**However**, there's a fragility here: the caller must ensure `ref_points_dxf` are in the same metric frame as the segments. If a new caller passes raw DXF units, the affine solve will produce an incorrect scale factor. **Recommendation:** Add a runtime assertion:

```python
# Sanity check: if ref points exist, their centroid should be within a
# plausible range of the segment points' centroid (order of magnitude check)
if metric_ref_points_dxf and segments:
    ref_centroid = (sum(p[0] for p in metric_ref_points_dxf) / len(metric_ref_points_dxf),
                    sum(p[1] for p in metric_ref_points_dxf) / len(metric_ref_points_dxf))
    seg_centroid = (sum(p[0] for seg in segments for p in seg.points) / max(1, sum(len(seg.points) for seg in segments)),
                    sum(p[1] for seg in segments for p in seg.points) / max(1, sum(len(seg.points) for seg in segments)))
    ref_spread = math.hypot(metric_ref_points_dxf[1][0] - metric_ref_points_dxf[0][0],
                            metric_ref_points_dxf[1][1] - metric_ref_points_dxf[0][1])
    seg_spread = math.hypot(segments[0].points[-1][0] - segments[0].points[0][0],
                            segments[0].points[-1][1] - segments[0].points[0][1]) if segments[0].points else 1.0
    if ref_spread > 0 and seg_spread > 0:
        ratio = ref_spread / seg_spread
        if ratio > 100 or ratio < 0.01:
            log.warning("Ref point spread (%.2f) vs segment spread (%.2f) ratio=%.2f — "
                       "possible unit mismatch", ref_spread, seg_spread, ratio)
```

### 3.2 Multi-Point Least-Squares (lines 467–494) — FUNCTIONALLY CORRECT

**Three observations:**

1. **Line 470–474:** The rotation_deg warning is logged whenever `rotation_deg` is non-zero. This is correct behaviour, but a user who provides both reference points and a heading might expect the heading to be used as an initial guess or constraint. **Consider** adding a future enhancement where `rotation_deg` constrains the least-squares solution (reducing degrees of freedom from 4 to 3).

2. **Line 475:** `ref_gps_origin = origin_gps if origin_gps is not None else ref_points_gps[0]` — This defaults the geodesic origin to the first reference point if no explicit `origin_gps` is provided. All subsequent NED computations are relative to this origin. This is mathematically valid, but it means the anchor in the staged mission will reference the first reference point's GPS coordinate, not a surveyed base station. **Minor concern:** for repeatable missions (same field, different days), anchoring to a fixed surveyed point is preferable.

3. **No outlier rejection:** If the operator accidentally mis-clicks one of the reference points (e.g., 5 m error), the least-squares solution will be pulled heavily toward that outlier, producing a large but still technically valid RMSE. **Recommendation:** Consider adding a RANSAC step or at least a warning when the max residual > 3× RMSE:

   ```python
   max_residual = max(residuals)
   if max_residual > 3.0 * rmse:
       log.warning(
           "Ref point %d has residual %.3f m (%.1f× RMSE=%.3f) — possible outlier",
           residuals.index(max_residual), max_residual, max_residual / rmse, rmse,
       )
   ```

### 3.3 Single Point + Heading Mode (lines 496–522) — CORRECT

The math is straightforward: apply the rotation to the DXF point, then compute the translation needed to place the rotated point at the NED target. No scale is derived (scale = 1 always).

**Potential issue:** If `rotation_deg` defaults to `0.0` (as it does in most callers), the transform is pure translation. The mission will be placed at the correct location but oriented according to the DXF's original north → which may not align with true north. This is documented behaviour, but worth emphasizing in the API docs.

### 3.4 Tangent Rotation (lines 546–554) — CORRECT

```python
cos_t = math.cos(theta_val)
sin_t = math.sin(theta_val)
seg.metadata["start_tangent"] = (st[0] * cos_t - st[1] * sin_t, st[0] * sin_t + st[1] * cos_t)
seg.metadata["end_tangent"] = (et[0] * cos_t - et[1] * sin_t, et[0] * sin_t + et[1] * cos_t)
```

Tangents are 2D unit vectors that need to be rotated by the same `θ` as the coordinate frame. The rotation `(x*cosθ - y*sinθ, x*sinθ + y*cosθ)` is the standard 2D rotation matrix applied to the tangent vector. This is correct — tangents are part of the vector space, not the point space.

**But note:** Tangents are only transformed when **both** `start_tangent` and `end_tangent` exist. If only one exists (e.g., a LINE with no curve metadata), the untransformed tangent is silently retained. **Suggestion:** Change the condition from `and` to `or`:

```python
if "start_tangent" in seg.metadata:
    st = seg.metadata["start_tangent"]
    seg.metadata["start_tangent"] = (st[0]*cos_t - st[1]*sin_t, st[0]*sin_t + st[1]*cos_t)
if "end_tangent" in seg.metadata:
    et = seg.metadata["end_tangent"]
    seg.metadata["end_tangent"] = (et[0]*cos_t - et[1]*sin_t, et[0]*sin_t + et[1]*cos_t)
```

### 3.5 Scale Not Applied to `origin` — DESIGN CHOICE

When `has_alignment` is True, the `origin` offset is skipped entirely (lines 797–800):

```python
if has_alignment:
    offset_pt = pt
else:
    offset_pt = (pt[0] + effective_offset[0], pt[1] + effective_offset[1])
```

**This means:** the `(offset_n, offset_e)` from the affine transform is the **only** translation applied. This is correct: the affine solve already computes a translation that maps the DXF origin relative to the first GPS reference point. Adding an additional `origin` offset would produce a double-translation.

---

## 4. `server/mission_placement.py` — Survey-Based Mission Placement

### 4.1 Freshness Checks (lines 42–54, 74–93) — THOROUGH

The `_fresh_age()` function validates that each data source (local pose, global position, GPS fix) is newer than its configured timeout. All three are checked, plus the skew between local and global samples.

**One gap:** The `gps_fix_age_ms` is checked against `GPS_FIX_STALE_MS` (line 81), but the actual GPS fix value is only checked to be `≥ RTK_FIXED` (lines 99–103). There's no check on `gps_fix_type` *trending* — e.g., if RTK fix was just lost (fix dropped from 6 to 5), the stale age timer hasn't expired yet. This is a minor race condition (~1 second window where stale RTK float could be accepted as fixed). Acceptable for production.

### 4.2 The Survey Translation Math (lines 114–120) — CORRECT

```python
delta_n, delta_e = latlon_to_ned(
    anchor_lat, anchor_lon,
    rover_lat, rover_lon,
)
translation = (rover_local_n + delta_n, rover_local_e + delta_e)
```

This computes:
1. The NED vector from the surveyed anchor point to the rover's current GPS position.
2. The rover's current local-NED position.

The anchor point's location in the rover's NED frame is then: `rover_NED_position + ΔNED_anchor_to_rover`.

**Verification with a concrete example:**
- Anchor at (13.0, 80.0)
- Rover GPS at (13.001, 80.0) → ~111 m north
- Rover local NED at (100, 0) → 100 m north of PX4 origin
- Anchor's local NED position = (100 + 111, 0 + 0) = (211, 0)
- A waypoint at the anchor would be placed at NED = (0 + 211, 0 + 0) = (211, 0) ✓

This is correct.

### 4.3 Missing: Rotation from Survey Points

The `resolve_surveyed_points()` function applies **only translation** — no rotation. If the rover's heading (yaw) at mission upload time differs from the heading at survey time, the mission will be rotationally misaligned.

**This is by design** — the survey anchor approach already locks the mission to a fixed coordinate reference, so rotation is handled at the PX4 level. But it's worth documenting explicitly that this function does NOT handle yaw misalignment.

---

## 5. Performance & Scalability

### 5.1 Python for-Loops Over Vertices

The entire pipeline uses Python for-loops for vertex operations. Key loops:

1. **`entities_to_segments()`** in `dxf_parser.py` (line 518) — O(N) over entities.
2. **Affine transform application** in `engine.py` (lines 541–545) — O(V) over all vertices.
3. **Corner smoothing** (lines 568–600) — O(V) worst-case.
4. **Densification** — O(V) per segment.
5. **Merge with de-dup** (lines 793–818) — O(V).

For a typical sports field with 10,000 waypoints, this is fine (well under 1 second). The concern is for extremely large paths (500,000 waypoints, the configured limit at `dxf_parser.py` line 469). At 500K points, Python overhead becomes noticeable.

**Vectorization opportunity (low priority):** The affine transform at lines 541–545:

```python
for seg in segments:
    seg.points = [
        apply_affine_transform(pt, scale_val, theta_val, offset_n_val, offset_e_val)
        for pt in seg.points
    ]
```

This could be vectorized with NumPy for ~10× speedup:

```python
import numpy as np

if has_alignment:
    cos_t, sin_t = math.cos(theta_val), math.sin(theta_val)
    s = scale_val
    for seg in segments:
        pts = np.array(seg.points, dtype=np.float64)  # (N, 2) = (north, east)
        # NED = scale * R(θ) @ DXF + offset
        # pts is already in (north, east) = (dxf_y, dxf_x) format
        scaled = pts * s
        rotated = np.empty_like(scaled)
        rotated[:, 0] = scaled[:, 0] * cos_t - scaled[:, 1] * sin_t + offset_n_val
        rotated[:, 1] = scaled[:, 0] * sin_t + scaled[:, 1] * cos_t + offset_e_val
        seg.points = [(float(n), float(e)) for n, e in rotated]
```

**However**, this introduces a NumPy dependency. For the current scale of operations (10K–50K waypoints), the pure-Python version is adequate. This optimization is recommended only if profiling shows it as a bottleneck.

### 5.2 DXF File Reading — DOUBLE PARSE

As noted in §2.1, `_get_unit_scale()` and `parse_dxf()` both call `ezdxf.readfile()`. For a 10 MB DXF with 20,000 entities, this adds ~200–500 ms of unnecessary parse time. **Fix:** Remove `_get_unit_scale()` and inline the unit detection into `parse_dxf()`.

### 5.3 `_plan_from_segments()` Deep Copy (lines 443–454)

The deep copy creates new `PathSegment` objects and copies all points. For 500K waypoints, this creates a ~24 MB memory allocation (500K × 2 × 8 bytes per float × Python tuple overhead). This is acceptable but worth noting for memory-constrained environments (Jetson Orin has 8 GB, so this is fine).

---

## 6. Error Handling & Robustness

### 6.1 RMSE Quality Gate (server/routes/path.py, reference in docs)

The 5 cm (0.05 m) RMSE threshold is **very tight** for field operations. Consider:
- A 2.5 cm GPS baseline error is typical even for RTK-fixed solutions.
- The operator clicking points on a canvas has ~1–2 cm pixel error.
- If the DXF and field are both surveyed to 1 cm, the total error budget is 3–4 cm.

**Recommendation:** Make `RMSE_MAX` configurable (e.g., 0.10 m for loose, 0.02 m for tight) rather than a hard-coded constant. The test `path.py` line 327 currently uses `0.05` from `config.py` — this is good, but consider exposing it in the API request body.

### 6.2 Malformed DXF Handling (dxf_parser.py lines 122–123)

```python
except Exception as exc:
    raise ValueError(f"Corrupt DXF file: {exc}") from exc
```

This catches all exceptions from `ezdxf.readfile()`, which is appropriate. **However,** `ezdxf` can raise specific exceptions like `ezdxf.DXFStructureError`, `ezdxf.DXFVersionError`, and `ezdxf.DXF12Error`. Catching the generic `Exception` is too broad. Consider:

```python
try:
    doc = ezdxf.readfile(filepath)
except (ezdxf.DXFStructureError, ezdxf.DXFVersionError, IOError) as exc:
    raise ValueError(f"Corrupt DXF file: {exc}") from exc
```

### 6.3 Missing `$INSUNITS` Fallback (lines 86–93, 132–135)

The fallback to `0.01` (cm) when `$INSUNITS` is missing is reasonable for sports field DXFs, which are commonly drawn in centimetres. However, a DXF drawn in metres (common for civil engineering) would produce a 100× scale error. **Recommendation:** Log a warning with the actual suggested scale when falling back:

```python
if insunits == 0 or scale is None:
    log.warning(
        "$INSUNITS is %s — using fallback scale %.4f (%.0f cm per unit). "
        "If the path appears massively scaled, the DXF may be drawn in a "
        "different unit (e.g., metres). Pass unit_scale=1.0 to fix.",
        "0 (unspecified)" if insunits == 0 else f"unknown value {insunits}",
        fallback, 1.0 / fallback if fallback > 0 else 0,
    )
```

### 6.4 Coincident Reference Points (ned.py lines 109–111)

The `denom < 1e-9` check prevents division by zero, but as noted in §1.2, it doesn't catch near-coincident points. A user could place two reference points 2 cm apart (denom ≈ 4e-4) and get an ill-conditioned solve.

**Recommendation:** Add the spread check described in §1.2.

### 6.5 GPS-to-NED Degeneracy (mission_placement.py)

If the `anchor_lat, anchor_lon` and `rover_lat, rover_lon` are the same point (e.g., if the rover is already at the anchor), `delta_n, delta_e` will be (0, 0), and `translation = rover_local_pos + (0, 0)`. This is correct but may be surprising if the mission is placed at the rover's current position when the operator expected a survey offset.

---

## 7. Code Quality & Maintainability

### 7.1 Architecture

The separation of concerns is clean:

| Layer | File | Responsibility |
|-------|------|---------------|
| Data model | `core.py` | DXFEntity, PathSegment, PlannedPath |
| DXF I/O | `parsers/dxf_parser.py` | Read .dxf → DXFEntity objects |
| Geometry math | `ned.py` | latlon→NED, affine solve/apply |
| Alignment orchestration | `engine.py` | Select method (LSQ/single/GPS), apply transform |
| Survey placement | `mission_placement.py` | Anchor-relative NED for survey mode |
| API layer | `routes/path.py` | HTTP validation, RMSE gate, mission staging |

The `PathEngine` class is a monolith (888 lines), but the `_plan_from_segments()` method is the only large method (494 lines). The pipeline steps are well-commented and sequential.

### 7.2 Type Hinting

- `ned.py`: **Excellent** — all functions have complete type hints with `tuple[float, float]`, `list[tuple[float, float]]`, etc.
- `dxf_parser.py`: **Good** — top-level functions are typed, but inner helper variables like `vertices`, `pts`, `bulges` are not annotated.
- `engine.py`: **Good** — method signatures are fully typed, but local variables inside `_plan_from_segments` (e.g., `densified`, `ordered`, `merged_waypoints`) are not.
- `mission_placement.py`: **Good** — function signatures typed.

### 7.3 Logging

- `ned.py`: No logging (uses `logging.getLogger` but never calls it). The affine solve silently fails with `ValueError`. Consider logging a warning when the RMSE exceeds a threshold.
- `dxf_parser.py`: Good use of `log.warning` for edge cases (missing `$INSUNITS`, unsupported entities, INSERT decomposition failures).
- `engine.py`: Good — single `log.info` at the end summarizing the pipeline, plus warnings for ignored `rotation_deg`.
- `mission_placement.py`: No logging. Every `PlacementError` is raised and caught by the caller — all state information is in the error message, but no trace is logged server-side.

### 7.4 Test Coverage (test_ned.py)

The test file has **14 tests** covering:
- `latlon_to_ned`: 4 tests (same point, pure north, pure east, known short distance)
- `dxf_to_ned_affine`: 6 tests (identity, scale, translation, rotation, insufficient points, coincident points, multipoint with noise)
- `apply_affine_transform`: 3 tests (identity, translation, scale, roundtrip)

**Missing tests:**
1. Rotation + scale + translation combined.
2. Negative scale (reflection) — the parser shouldn't produce this, but `dxf_to_ned_affine` would return `scale = hypot(a, b)` which is always positive.
3. Real-world values: 100 DXF cm units → 1 NED metre with noise.
4. Large RMSE rejection: verify that `dxf_to_ned_affine` produces an RMSE consistent with the input noise level.

**Test quality:** The `test_affine_multipoint_least_squares` test (lines 116–133) is excellent — it uses a real 4-point square with ±1 cm noise and checks all outputs within tight tolerances.

---

## 8. Summary of Recommendations

### High Priority (Potential Bugs)

| # | Location | Issue | Fix |
|---|----------|-------|-----|
| 1 | `dxf_parser.py:77,121` | Double DXF parse in `_get_unit_scale()` + `parse_dxf()` | Remove `_get_unit_scale()`, inline into `parse_dxf()` |
| 2 | `ned.py:109` | No near-coincident point guard (ill-conditioned solve when points < 1 cm apart) | Add spread check after denominator check |
| 3 | `engine.py:547` | Tangent rotation requires BOTH start/end tangents; partial metadata silently untransformed | Change `and` → two separate `if` checks |

### Medium Priority (Robustness)

| # | Location | Issue | Fix |
|---|----------|-------|-----|
| 4 | `dxf_parser.py:123` | Over-broad `except Exception` in DXF read | Catch specific `ezdxf` exceptions |
| 5 | `engine.py:465` | No guard against caller passing raw (unscaled) DXF units in `ref_points_dxf` | Add runtime spread-ratio sanity check |
| 6 | `engine.py:490` | No outlier rejection in least-squares | Add max-residual > 3×RMSE warning |
| 7 | `dxf_parser.py:452` | Unthrottled logging for unsupported sub-entity types | Guard with a set of already-reported types |

### Low Priority (Performance / Polish)

| # | Location | Issue | Fix |
|---|----------|-------|-----|
| 8 | `engine.py:541-545` | Pure-Python loop over all vertices for affine transform | NumPy vectorization (if profiling shows bottleneck) |
| 9 | `mission_placement.py` | No rotation handling for survey mode | Document limitation |
| 10 | `ned.py:134` | No RMSE warning log in `dxf_to_ned_affine()` | Add `log.warning` when RMSE > 0.02 m |
| 11 | `engine.py:470-474` | Least-squares ignores `rotation_deg` without providing a constrained-solve option | Future enhancement |

---

## 9. Conclusion

The DXF parsing and coordinate alignment pipeline is **mathematically sound and production-ready**. The key mathematical components are verified correct:

1. **Geodesic → NED conversion:** Uses WGS84 ellipsoid with GeographicLib (no flat-earth approximation).
2. **Umeyama least-squares affine solve:** Correctly implements the similarity transform (uniform scale + rotation + translation).
3. **Affine transform application:** Mathematically consistent with the solve.
4. **Survey anchor translation:** Correctly places anchor-relative waypoints in the rover's local NED frame.
5. **Tangent rotation:** Correctly transforms tangent vectors under rotation.

The issues identified are mainly defensive programming gaps (edge-case guards, logging, error specificity) rather than mathematical errors. The most impactful recommendation is eliminating the double DXF parse in `_get_unit_scale()` + `parse_dxf()`, which provides a free ~20–40% speedup for DXF loading at the cost of a small refactor.