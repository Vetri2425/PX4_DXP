# Frontend Wiring — DXF Alignment & Mission Handoff (Flow Logic)

This document describes the **logic and data contracts** of the ground-alignment
flow exposed by the rover server. It is implementation-agnostic: it specifies
*what* to call, *in what order*, *what each call returns*, and *how to branch* —
not how to render anything. Build the UI however you like.

For entity-level DXF selection, per-entity spray toggles, and PRE/AFT extension
configuration, see `docs/DXF_ENTITY_FRONTEND_GUIDE.md`.

- Base URL: `http://<jetson>:5001` (default `192.168.1.102`).
- Auth: send header `X-Rover-Token: <token>` unless the server runs with
  `ROVER_DISABLE_AUTH` set.
- Content type for JSON bodies: `application/json`.

---

## 1. Flow overview

```
parse-dxf ──▶ preview ──▶ plan ──▶ load-to-controller ──▶ mission/start
  (1)          (2)         (3)          (4)                   (5)
```

Each step depends on an output of the previous one:

| Step | Produces | Consumed by |
|------|----------|-------------|
| 1 parse-dxf | `filename` | steps 2 and 3 (`source`) |
| 2 preview | NED geometry + bounds | reference-point capture for step 3 |
| 3 plan | `mission_summary.mission_id` | step 4 |
| 4 load-to-controller | controller holds the path | step 5 |
| 5 mission/start | drives the mission | — |

The flow is **stateless between calls except for the staged artifact** written in
step 3 and keyed by `mission_id`. Nothing else is remembered server-side; the
client carries `filename` and `mission_id` forward.

---

## 2. Step 1 — `POST /api/path/parse-dxf`

Upload + parse a DXF. Multipart form, field name `file`.

**Returns**
```jsonc
{
  "filename":     "square_2x2.dxf",   // basename actually persisted; use as `source` later
  "num_entities": 4,
  "unit_scale":   1.0,                // metres per DXF unit (informational; engine applies it)
  "layer_names":  ["0"],
  "entities":     [ { "entity_type", "layer", "color", "entity_id", "is_mark", "length_m" } ]
}
```

**Logic**
- Persist `filename` for the session. It is the `source` value in steps 2 and 3.
- `unit_scale` is informational — the engine scales geometry and reference points
  internally. The client does not need to apply it.
- Errors: `415` wrong extension, `413` too large, `422` parse failure, `500`
  missing parser dependency.

---

## 3. Step 2 — `GET /api/path/{filename}/preview`

Returns the geometry in **local NED metres** for display and for capturing
reference points.

**Returns**
```jsonc
{
  "name": "square_2x2.dxf",
  "frame": "local_ned",
  "num_points": 168,
  "bounds": { "north_min": -0.035, "north_max": 2.035,
              "east_min":  -0.035, "east_max":  2.035 },
  "waypoints": [ { "north": 0.0, "east": 0.0, "spray": true }, ... ]
}
```

**Logic**
- Axes: `north` is the local +N axis, `east` is the local +E axis. `spray=true`
  marks MARK geometry, `false` marks TRANSIT.
- `bounds` gives the extent for fitting a view transform.
- **Reference-point capture is a client responsibility.** For each reference the
  client must produce a `{ dxf_x, dxf_y, lat, lon }` tuple:
  - `dxf_x, dxf_y` — the point in **DXF coordinates** (invert whatever
    view→data transform the client used to place it).
  - `lat, lon` — the **real-world WGS84** coordinate that DXF point maps to,
    supplied by the operator (typed, map-picked, surveyed, etc.).
- Errors: `404` unknown file, `504` parse exceeded the 15 s server cap.

---

## 4. Step 3 — `POST /api/path/plan`

Runs the planning pipeline, selects an alignment method, applies the transform,
gates on residual error, and (for aligned plans) stages a mission.

### 4.1 Method selection logic

The server picks **one** method by a priority chain. The client controls which
one runs purely by which fields it sends:

| Condition sent by client | Method that runs | Notes |
|--------------------------|------------------|-------|
| `ref_points` length ≥ 2 | `least_squares` | Full similarity fit (scale+rotation+translation). `rotation_deg` is ignored. Produces a real `rmse`. |
| `ref_points` length == 1 (+ `rotation_deg`) | `single_point_heading` | Translation from the point, heading from `rotation_deg`, `scale=1`, `rmse=0`. |
| no `ref_points`, `origin_gps` set (+ `rotation_deg`) | `gps_origin` | Anchor + heading. `scale=1`, no point fit, no residual. |
| neither `ref_points` nor `origin_gps` | *(no alignment)* | `alignment_metadata` is `null`, **no mission is staged**. |

Decision logic the client implements:
```
if refPoints.length >= 2:   send { source, ref_points }
elif refPoints.length == 1: send { source, ref_points, rotation_deg }
elif haveAnchorGps:         send { source, origin_gps, rotation_deg }
else:                       non-aligned plan (no staging, not part of this flow)
```

### 4.2 Minimal request bodies (one per method)

```jsonc
// least_squares
{ "source": "square_2x2.dxf",
  "ref_points": [
    { "dxf_x": 0, "dxf_y": 0, "lat": 13.00000000, "lon": 80.00000000 },
    { "dxf_x": 2, "dxf_y": 2, "lat": 13.00001797, "lon": 80.00001844 }
  ] }

// single_point_heading
{ "source": "square_2x2.dxf",
  "ref_points": [ { "dxf_x": 0, "dxf_y": 0, "lat": 13.0, "lon": 80.0 } ],
  "rotation_deg": 45 }

// gps_origin
{ "source": "square_2x2.dxf",
  "origin_gps": [13.0, 80.0],
  "rotation_deg": 30 }
```

### 4.3 Fields NOT to send

`selected_entities`, `overrides`, `order` are **not implemented** and cause a
hard `422` if non-null. Omit them (or send `null`). All other planning fields
(`line_spacing`, speeds, `optimize`, extensions, corner smoothing, `max_*`,
`include_waypoints`, etc.) are optional and default sensibly; they are not
transformation inputs. Speeds must be `> 0`.

### 4.4 Response

```jsonc
{
  "source": "square_2x2.dxf",
  "num_waypoints": 168,
  "num_segments": 4,
  "mark_length_m": 8.1, "transit_length_m": 0.0, "total_length_m": 8.1,
  "segments": [ ... ], "merged_waypoints": [[n,e], ...], "spray_flags": [true, ...],

  "alignment_metadata": {
    "method": "least_squares",          // which branch ran (or null if none)
    "scale": 0.997127,                  // ~1.0 expected
    "rotation_deg": 0.1785,
    "rmse": 0.0,                        // least_squares only; absent for gps_origin
    "origin_gps": [13.0, 80.0],         // the GPS anchor
    "offset_n": 0.0, "offset_e": 0.0
  },

  "planning_metadata": { ... },
  "warnings": [ ... ],

  "mission_summary": {                  // present ONLY when an alignment was applied
    "mission_id": "stg_xxxxxxxx_1781000238",
    "num_waypoints": 168,
    "total_length_m": 8.103,
    "estimated_paint_l": 0.097,
    "estimated_runtime_s": 23.2,
    "rmse_m": 0.0
  }
}
```

### 4.5 Branching logic on the response

- `alignment_metadata.method` tells you which transform ran. Surface
  `scale` and `rmse` for verification (scale far from `1.0`, or any RMSE-gate
  rejection, indicates a bad operator GPS↔DXF mapping).
- `mission_summary` is **non-null only when a method was applied**. If it is
  `null`, no mission was staged — do not advance to load.
- **Keep `mission_summary.mission_id`** — it is the only handle to the staged
  artifact for step 4.

### 4.6 Error handling

| Status | `detail` substring | Meaning / branch |
|--------|--------------------|------------------|
| `422` | `"Alignment error too high (rmse=…)"` | RMSE gate (> 0.05 m). **Nothing staged.** Re-capture reference points (back to step 2). |
| `422` | `"Preview fields not implemented"` | You sent `selected_entities`/`overrides`/`order`. Remove them. |
| `422` | `"Planning error: …"` | Invalid geometry/params (e.g. speed ≤ 0). |
| `404` | — | Unknown `source` file. |
| `504` | — | Planning exceeded the 15 s cap. |

The RMSE gate applies to `least_squares` only. `single_point_heading` (rmse 0)
and `gps_origin` (no residual) pass by definition.

---

## 5. Step 4 — `POST /api/path/load-to-controller`

Commits the **exact staged waypoints** to the controller. No re-planning.

**Request**
```jsonc
{ "mission_id": "stg_xxxxxxxx_1781000238" }
```

**Response (200)**
```jsonc
{ "status": "success", "mission_id": "...", "num_waypoints": 168, "anchor_loaded": true }
```

**Branching logic**

| Status | Meaning / branch |
|--------|------------------|
| `200` | Path loaded. Proceed to step 5. |
| `409` | A mission is RUNNING/transitional. Must stop the active mission first; do not load. |
| `404` | Staged mission not found or expired. Re-run step 3 to get a fresh `mission_id`. |
| `422` | Staged artifact unreadable or empty. Re-run step 3. |
| `503` | Controller not ready (ROS not up). |

**Lifecycle note:** the staged artifact has a TTL (default 1 hour) and is **not
persisted across a server restart**. Treat `mission_id` as ephemeral — never
store it beyond the active session. A `404` here always means "re-plan".

---

## 6. Step 5 — `POST /api/mission/start`

Begins the mission (arm → setpoint stream → OFFBOARD → drive). Body `{}` (or the
documented start options).

**Precondition logic (read from telemetry before allowing start):**
- `fcu_connected` must be true.
- `rpp_state` must be healthy — i.e. not `STALE`, `RTK_WAIT`, or `JUMP_SKIP`.
  Starting under those returns an error from the controller.

This step is independent of the alignment flow — it drives whatever the
controller currently holds (loaded in step 4).

---

## 7. Live state — telemetry

Prefer the Socket.IO telemetry stream (10 Hz) over polling for runtime state.
It carries the fields used to gate steps and show progress:

- Link/mode: `connected`, `armed`, `mode`
- Controller: `rpp_state`, `rpp_state_name`, `xtrack_m`, `dist_to_goal_m`, `speed_m_s`
- GPS: `gps_fix`, `gps_fix_name`, `gps_sat`, `hrms`, `vrms`, `lat`, `lon`

`GET /api/mission/status` is the polling equivalent for the mission lifecycle
(`state`, `rpp_state`, `dist_to_goal`, `xtrack`, `fcu_connected`,
`last_path_loaded`).

---

## 8. Scope boundary (aligned vs non-aligned loads)

This flow (staging + `load-to-controller`) is for **aligned DXF missions only**.
Built-in paths, CSV, and `.waypoints` files do not go through staging — they
load via `POST /api/mission/load` and have no `mission_summary`/`mission_id`.
A plan with no `ref_points` and no `origin_gps` returns waypoints inline with
`mission_summary: null` and is not part of this handoff.

---

## 9. Sequence summary (decision-focused)

```
1. parse-dxf(file)                         -> filename
2. preview(filename)                       -> geometry + bounds
   capture references                      -> [{dxf_x,dxf_y,lat,lon}, ...]
3. plan(source=filename, + method fields)
     422 rmse-gate?    -> re-capture refs, go to 2
     method == null?   -> not aligned, stop (use /api/mission/load instead)
     else              -> keep mission_id, show summary
4. load-to-controller(mission_id)
     409 running?      -> stop active mission, retry
     404 expired?      -> go to 3
     200               -> proceed
5. mission/start  (only when fcu_connected and rpp_state healthy)
```
