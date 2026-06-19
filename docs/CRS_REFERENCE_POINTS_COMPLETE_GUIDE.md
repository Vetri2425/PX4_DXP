# CRS Reference Points — Full Code & Logic Guide

This document is a **complete reference** for how the CRS (Coordinate Reference System) / alignment flow works end-to-end: from the frontend capturing reference points, through the API, into the server, and down to the affine math that aligns DXF coordinates to real-world GPS.

---

## 1. DATA FLOW OVERVIEW

```
Frontend (canvas)
  │  User clicks point on DXF preview → gets (dxf_x, dxf_y) in DXF units
  │  User inputs GPS coordinate (lat, lon) for that same point
  │  Sends to backend: POST /api/path/plan with ref_points array
  ▼
Server route (routes/path.py → plan_path)
  │  Unpacks ref_points into:
  │    ref_points_dxf = [(dxf_y, dxf_x), ...]   ← DXF coordinates
  │    ref_points_gps = [(lat, lon), ...]         ← WGS84 coordinates
  │  Passes to PathManager.plan_path()
  ▼
PathManager (path_manager.py → plan_path)
  │  Passes to PathEngine.plan_file() / plan_dxf_entities()
  ▼
PathEngine (engine.py → _plan_from_segments)
  │  Key: metrics ref points from DXF units to metres using ref_unit_scale
  │  Then calls dxf_to_ned_affine() if ≥2 ref points
  │    OR single_point_heading transform if ==1 ref point
  │    OR gps_origin transform if origin_gps only
  ▼
ned.py → dxf_to_ned_affine()
  │  Least-squares fit: [NED] = scale * R(θ) @ [DXF] + offset
  │  Returns scale, theta, offset_n, offset_e, residuals, rmse
  ▼
Back to PathEngine:
  │  Applies affine transform to all segment points
  │  Continues pipeline (densify, optimize, extend, merge)
  ▼
Staged mission with anchor block for controller
```

---

## 2. FRONTEND REFERENCE POINT CAPTURE CONTRACT

From `docs/frontend_wiring.md` (Section 3, Step 2):

### What the frontend must capture per reference point

```jsonc
{
  "dxf_x": 0.0,      // DXF x-coordinate of the clicked point
  "dxf_y": 0.0,      // DXF y-coordinate of the clicked point
  "lat": 13.00000000, // WGS84 latitude (decimal degrees)
  "lon": 80.00000000  // WGS84 longitude (decimal degrees)
}
```

**Critical rule:** `dxf_x` and `dxf_y` are in **raw DXF coordinates** (what the DXF file stores, before any scaling). The engine auto-detects unit_scale from the DXF's `$INSUNITS` header and applies it internally.

### Where the frontend gets (dxf_x, dxf_y)

From `GET /api/path/{filename}/entities` → `entities[].preview_points`:

```jsonc
// preview_point is { north, east } in local NED metres
// Mapping to DXF coordinates:
dxf_x = east   // ← yes, DXF X maps to "east" in NED
dxf_y = north  // ← yes, DXF Y maps to "north" in NED
```

From `docs/DXF_ENTITY_FRONTEND_GUIDE.md` line 136-138:
```
- `north` is local NED north in metres.
- `east` is local NED east in metres.
- For `/api/path/plan` reference points, map clicked local NED back as:
  - `dxf_x = east`
  - `dxf_y = north`
```

### Where the frontend gets (lat, lon)

The operator provides these — typed in, picked from a map, or read from a survey point.

---

## 3. API REQUEST – Route Handler

**File:** `server/routes/path.py` lines 743–863

The `POST /api/path/plan` endpoint:

```python
# lines 783-784 — the critical unpacking:
ref_points_dxf = [(pt.dxf_y, pt.dxf_x) for pt in req.ref_points] if req.ref_points is not None else None
ref_points_gps = [(pt.lat, pt.lon) for pt in req.ref_points] if req.ref_points is not None else None
```

**NOTICE:** `dxf_y` comes before `dxf_x`! This is because the engine internally stores coordinates as `(north, east)` = `(dxf_y, dxf_x)`.

Then passed to `path_mgr.plan_path()`:

```python
path_mgr.plan_path(
    req.source,
    ...
    origin_gps=origin_gps,      # [lat, lon] WGS84 reference
    rotation_deg=req.rotation_deg,
    ref_points_dxf=ref_points_dxf,   # [(dxf_y, dxf_x), ...]
    ref_points_gps=ref_points_gps,   # [(lat, lon), ...]
    ...
)
```

---

## 4. PathManager → PathEngine

**File:** `server/path_manager.py` lines 833–1117 → calls `engine.plan_file()` or `engine.plan_dxf_entities()`

```python
# lines 875-876: pass through to engine
ref_points_dxf = kwargs.pop("ref_points_dxf", None)
ref_points_gps = kwargs.pop("ref_points_gps", None)
```

Then at lines 1043-1055:
```python
plan = engine.plan_file(
    fpath,
    ...
    origin_gps=origin_gps,
    rotation_deg=rotation_deg,
    ref_points_dxf=ref_points_dxf,
    ref_points_gps=ref_points_gps,
    ...
)
```

---

## 5. PathEngine — ALIGNMENT MATH (The CRS Core)

**File:** `path_engine/engine.py` lines 395–876, method `_plan_from_segments()`

### Step A: Scale reference points to metric frame (Gap A fix)

```python
# lines 466-470:
metric_ref_points_dxf = None
if ref_points_dxf:
    metric_ref_points_dxf = [
        (pt[0] * ref_unit_scale, pt[1] * ref_unit_scale) for pt in ref_points_dxf
    ]
```

**This is critical.** The DXF parser has already scaled all geometry vertices by `unit_scale` (e.g., 0.01 for cm-unit DXFs). But the reference points arrive in **raw DXF units**. Without scaling them by the same factor, the affine solve would compare cm vs metres and produce a wildly wrong scale.

### Step B: Method Selection (lines 472–543)

```python
# ─── LEAST SQUARES (≥2 ref points) ─────────────────────────────
if metric_ref_points_dxf and ref_points_gps and len(metric_ref_points_dxf) >= 2 and len(ref_points_gps) >= 2:
    # Convert GPS → NED relative to first ref GPS point
    ref_gps_origin = origin_gps if origin_gps is not None else ref_points_gps[0]
    ref_ned_points = []
    for gps_pt in ref_points_gps:
        n, e = latlon_to_ned(gps_pt[0], gps_pt[1], ref_gps_origin[0], ref_gps_origin[1])
        ref_ned_points.append((n, e))

    scale_val, theta_val, offset_n_val, offset_e_val, residuals, rmse = dxf_to_ned_affine(
        metric_ref_points_dxf, ref_ned_points
    )
    # → alignment_meta = { method: "least_squares", scale, rotation_deg, offset_n, offset_e, residuals, rmse, origin_gps }

# ─── SINGLE POINT + HEADING (1 ref point + rotation_deg) ───────
elif metric_ref_points_dxf and ref_points_gps and len(metric_ref_points_dxf) == 1 and len(ref_points_gps) == 1:
    ref_gps_origin = origin_gps if origin_gps is not None else ref_points_gps[0]
    n, e = latlon_to_ned(ref_points_gps[0][0], ref_points_gps[0][1], ref_gps_origin[0], ref_gps_origin[1])
    scale_val = 1.0
    theta_val = math.radians(rotation_deg)
    rp = metric_ref_points_dxf[0]
    rot_n = rp[0] * math.cos(theta_val) - rp[1] * math.sin(theta_val)
    rot_e = rp[0] * math.sin(theta_val) + rp[1] * math.cos(theta_val)
    offset_n_val = n - rot_n
    offset_e_val = e - rot_e
    # → alignment_meta = { method: "single_point_heading", ... }

# ─── GPS ORIGIN ONLY (no ref points, just origin_gps + rotation_deg) ─
elif origin_gps is not None:
    scale_val = 1.0
    theta_val = math.radians(rotation_deg)
    offset_n_val = 0.0
    offset_e_val = 0.0
    # → alignment_meta = { method: "gps_origin", ... }
```

### Step C: Apply Transform to all segment points (lines 545-559)

```python
if has_alignment:
    for seg in segments:
        seg.points = [
            apply_affine_transform(pt, scale_val, theta_val, offset_n_val, offset_e_val)
            for pt in seg.points
        ]
```

---

## 6. THE AFFINE MATH (dxf_to_ned_affine)

**File:** `path_engine/ned.py` lines 57–134

### Algorithm: Umeyama-style least-squares similarity transform

```python
def dxf_to_ned_affine(
    dxf_points: list[tuple[float, float]],     # [(dxf_y, dxf_x), ...] in metres
    ref_ned_points: list[tuple[float, float]],  # [(north, east), ...] in NED metres
) -> tuple[float, float, float, float, list[float], float]:
```

**Step 1:** Compute centroids of both point sets:
```python
mean_dy = sum(dxf_points[i][0]) / n  # mean of dxf_y
mean_dx = sum(dxf_points[i][1]) / n  # mean of dxf_x
mean_n  = sum(ned_points[i][0]) / n  # mean of north
mean_e  = sum(ned_points[i][1]) / n  # mean of east
```

**Step 2:** Center both sets:
```python
u = [pt[0] - mean_dy for pt in dxf_points]   # centered dxf_y
v = [pt[1] - mean_dx for pt in dxf_points]   # centered dxf_x
x = [pt[0] - mean_n  for pt in ned_points]    # centered north
y = [pt[1] - mean_e  for pt in ned_points]    # centered east
```

**Step 3:** Solve the normal equations. The transform matrix is:
```math
[ north ]   [ a  -b ] [ dxf_y ]
[ east  ] = [ b   a ] [ dxf_x ]
```

The least-squares solution:
```python
denom = sum(u_i² + v_i²)        # variance of DXF points
a = sum(u_i * x_i + v_i * y_i) / denom
b = sum(u_i * y_i - v_i * x_i) / denom
```

**Step 4:** Extract scale and rotation:
```python
scale = hypot(a, b)             # uniform scale factor
theta = atan2(b, a)             # rotation angle in radians
```

**Step 5:** Compute translation (in NED frame):
```python
offset_n = mean_n - (a * mean_dy - b * mean_dx)
offset_e = mean_e - (b * mean_dy + a * mean_dx)
```

**Step 6:** Compute per-point residuals and RMSE:
```python
for each (dxf_pt, ned_pt) pair:
    pred = apply_affine_transform(dxf_pt, scale, theta, offset_n, offset_e)
    res = hypot(ned_pt[0] - pred[0], ned_pt[1] - pred[1])
    residuals.append(res)
    sq_err_sum += res²
rmse = sqrt(sq_err_sum / n)
```

**Step 7:** Return `(scale, theta, offset_n, offset_e, residuals, rmse)`

---

## 7. apply_affine_transform

**File:** `path_engine/ned.py` lines 137–162

```python
def apply_affine_transform(
    point: tuple[float, float],     # (dxf_y, dxf_x)
    scale: float,
    theta: float,                    # radians
    offset_n: float,
    offset_e: float,
) -> tuple[float, float]:           # (north, east)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    sx = point[0] * scale   # scale dxf_y
    sy = point[1] * scale   # scale dxf_x
    north = sx * cos_t - sy * sin_t + offset_n
    east  = sx * sin_t + sy * cos_t + offset_e
    return (north, east)
```

---

## 8. latlon_to_ned (GPS → NED conversion)

**File:** `path_engine/ned.py` lines 22–54

Uses GeographicLib's Karney geodesic (WGS84 ellipsoid):

```python
def latlon_to_ned(lat, lon, origin_lat, origin_lon):
    result = Geodesic.WGS84.Inverse(origin_lat, origin_lon, lat, lon)
    dist = result["s12"]                   # geodesic distance in metres
    bearing_rad = math.radians(result["azi1"])   # forward azimuth in degrees
    north = dist * math.cos(bearing_rad)
    east  = dist * math.sin(bearing_rad)
    return (north, east)
```

---

## 9. RMSE QUALITY GATE

**File:** `server/routes/path.py` lines 834–840

After planning, the RMSE from `alignment_metadata` is checked:

```python
rmse = alignment_meta.get("rmse", 0.0)
if rmse > RMSE_MAX:         # RMSE_MAX = 0.05 m (from config.py)
    raise HTTPException(
        422,
        f"Alignment error too high (rmse={rmse:.3f} m, max {RMSE_MAX:.3f} m). "
        "Re-verify the reference points.",
    )
```

**Only `least_squares` mode produces a real RMSE.** `single_point_heading` reports rmse=0 and `gps_origin` has no residual — both bypass this gate by definition (as documented in `frontend_wiring.md` section 4.6).

---

## 10. STAGED MISSION ANCHOR

**File:** `server/routes/path.py` lines 883–943

When alignment was applied, the staged mission payload includes an `anchor` block:

```python
anchor = {
    "frame": "local_ned",
    "lat": origin_gps[0],
    "lon": origin_gps[1],
    "rotation_deg": alignment_meta.get("rotation_deg", 0.0),
    "scale": alignment_meta.get("scale", 1.0),
}
```

This is stored in the staged JSON artifact so the controller can re-project NED waypoints back to WGS84 if needed.

---

## 11. MODEL SCHEMAS (from server/models.py)

```python
class RefPoint(BaseModel):
    dxf_x: float    # DXF x coordinate
    dxf_y: float    # DXF y coordinate
    lat: float      # WGS84 latitude
    lon: float      # WGS84 longitude

class PathPlanRequest(BaseModel):
    source: str
    ref_points: Optional[list[RefPoint]] = None       # DXF→NED affine fit
    origin_gps: Optional[list[float]] = None           # [latitude, longitude]
    rotation_deg: float = 0.0
    # ... other fields ...

class AlignResponse(BaseModel):
    method: Optional[str] = None          # least_squares | single_point_heading | gps_origin
    rmse_m: float = 0.0
    scale: float = 1.0
    rotation_deg: float = 0.0
    offset_n: float = 0.0
    offset_e: float = 0.0
    origin_gps: Optional[list[float]] = None
    residuals: list[RefPointResidual]     # per-point residual
```

---

## 12. TEST COVERAGE (from server/test_path_api.py)

```python
# ── Least-squares alignment with 2 ref points ──
@pytest.mark.anyio
async def test_plan_api_dxf_ref_points():
    req = PathPlanRequest(
        source="square_2x2.dxf",
        close_loop=True,
        ref_points=[
            RefPoint(dxf_x=0.0, dxf_y=0.0, lat=13.0, lon=80.0),
            RefPoint(dxf_x=10.0, dxf_y=0.0, lat=13.0001, lon=80.0),
        ],
    )
    data = await plan_path(...)
    assert data.alignment_metadata["method"] == "least_squares"
    assert "scale" in data.alignment_metadata

# ── Single point + heading ──
@pytest.mark.anyio
async def test_plan_api_single_point_heading():
    req = PathPlanRequest(
        source="square_2x2.dxf",
        rotation_deg=30.0,
        ref_points=[
            RefPoint(dxf_x=5.0, dxf_y=5.0, lat=13.0001, lon=80.0001),
        ],
    )
    data = await plan_path(...)
    assert data.alignment_metadata["method"] == "single_point_heading"
    assert data.alignment_metadata["rotation_deg"] == 30.0

# ── Coincident points (should fail) ──
@pytest.mark.anyio
async def test_plan_api_coincident_ref_points():
    req = PathPlanRequest(
        source="square_2x2.dxf",
        ref_points=[
            RefPoint(dxf_x=0.0, dxf_y=0.0, lat=13.0, lon=80.0),
            RefPoint(dxf_x=0.0, dxf_y=0.0, lat=13.0, lon=80.0),
        ],
    )
    with pytest.raises(HTTPException, match="Planning error"):
        await plan_path(...)

# ── Scale unity regression (Gap A) ──
def test_affine_scale_is_unity_when_ref_points_share_metric_frame():
    # Raw DXF points in cm units: 1000 DXF units = 10 m
    raw_dxf = [(0.0, 0.0), (0.0, 1000.0)]     # stored as (dxf_y, dxf_x)
    ned    = [(0.0, 0.0), (0.0, 10.0)]        # 10 m east in NED
    unit_scale = 0.01  # cm → metres

    raw_scale = dxf_to_ned_affine(raw_dxf, ned)[0]
    assert abs(raw_scale - 1.0) > 0.5  # wrong without scaling

    metric_dxf = [(p[0] * unit_scale, p[1] * unit_scale) for p in raw_dxf]
    fixed_scale = dxf_to_ned_affine(metric_dxf, ned)[0]
    assert abs(fixed_scale - 1.0) < 1e-6  # correct after scaling
```

---

## 13. COMPLETE FILE LISTING

All files in the CRS alignment chain:

| File | Role |
|------|------|
| `docs/frontend_wiring.md` | Frontend flow & API contract |
| `docs/DXF_ENTITY_FRONTEND_GUIDE.md` | Entity preview & coordinate mapping |
| `server/models.py` | `RefPoint`, `PathPlanRequest`, `AlignResponse` models |
| `server/routes/path.py` | Route handlers: `plan_path`, `align_path`, `_stage_mission` |
| `server/path_manager.py` | `plan_path()` orchestrator |
| `path_engine/engine.py` | `_plan_from_segments()` alignment logic |
| `path_engine/ned.py` | `latlon_to_ned()`, `dxf_to_ned_affine()`, `apply_affine_transform()` |
| `path_engine/parsers/dxf_parser.py` | DXF file parsing & unit_scale detection |
| `server/test_path_api.py` | Full alignment test coverage |

---

## 14. QUICK REFERENCE: DEBUGGING A BAD CRS

| Symptom | Likely Cause | Check |
|---------|-------------|-------|
| `scale` far from 1.0 | Gap A: ref points not in metric frame | Is `ref_unit_scale` being applied in `engine.py` line 468-469? |
| RMSE > 0.05 m | Operator GPS↔DXF mapping wrong | Check the `residuals` array for which point is off |
| `method` = null / no mission staged | No alignment data sent | Did frontend send `ref_points` or `origin_gps`? |
| 422 "Alignment error too high" | RMSE gate triggered | Re-capture reference points more accurately |
| Offset wrong but scale & rotation correct | `single_point_heading` mode with bad point | Check the single ref point mapping |
| Rotation wrong in `least_squares` | Explicit `rotation_deg` being **ignored** | Documented behavior: least-squares derives rotation from the points |

---

## 15. COMPLETE API CALL EXAMPLE (for ChatGPT)

```http
POST /api/path/plan
Content-Type: application/json
X-Rover-Token: <token>

{
  "source": "soccer_pitch.dxf",
  "ref_points": [
    {
      "dxf_x": 0.0,
      "dxf_y": 0.0,
      "lat": 13.00000000,
      "lon": 80.00000000
    },
    {
      "dxf_x": 105.0,
      "dxf_y": 0.0,
      "lat": 13.00000944,
      "lon": 80.00000970
    }
  ],
  "rotation_deg": 0.0,
  "line_spacing": 0.05,
  "transit_spacing": 0.15,
  "marking_speed": 0.35,
  "transit_speed": 0.50,
  "optimize": true,
  "close_loop": false
}
```

Response alignment_metadata will look like:
```json
{
  "alignment_metadata": {
    "method": "least_squares",
    "scale": 0.9998,
    "rotation_deg": 0.287,
    "offset_n": 13.0003,
    "offset_e": 0.0012,
    "residuals": [0.003, 0.005],
    "rmse": 0.004,
    "origin_gps": [13.0, 80.0]
  },
  "mission_summary": {
    "mission_id": "stg_a1b2c3d4_1781000238",
    "num_waypoints": 8734,
    "total_length_m": 436.7,
    "estimated_paint_l": 5.24,
    "estimated_runtime_s": 1248.0,
    "rmse_m": 0.004
  }
}