# Path, Upload, and Mission Load Endpoints

Scope: frontend path listing, plain-screen preview, upload/parse/plan flows, and mission-load handoff into `OffboardController`.

Out of scope: mission start/stop/abort, arming/mode control, telemetry broadcasts, watchdogs, and PX4 parameter flows.

## REST Endpoints

| Status | Method | Path | Source | Side effects | Response |
|---|---|---|---|---|---|
| Implemented | GET | `/api/paths` | `server/routes/path.py` | None | List built-in and uploaded path metadata: `name`, `description`, `num_points`, `source`. No coordinates. |
| Implemented | GET | `/api/path/{name}/preview` | `server/routes/path.py` | None | Local-NED display preview: `name`, `frame`, `num_points`, `bounds`, `waypoints[]` with `north`, `east`, `spray`. |
| Implemented | POST | `/api/path/upload` | `server/routes/path.py` | Saves uploaded file to missions dir | Validates `.waypoints`, `.csv`, or `.dxf`; returns `{"saved": filename, "size": bytes}`. Does not parse points for preview. |
| Implemented | POST | `/api/path/parse-dxf` | `server/routes/path.py` | Saves DXF to missions dir after successful parse | Returns DXF entity summaries: type, layer, color, entity id, mark flag, length, unit scale, layer names. Does not return route waypoints. |
| Implemented | POST | `/api/path/plan` | `server/routes/path.py` | None | Runs full planning pipeline and returns planned waypoints, spray flags, segments, lengths, alignment/planning metadata, warnings. Heavy endpoint; not the simple preview path. |
| Implemented | POST | `/api/mission/load` | `server/routes/mission.py` | Loads points into `OffboardController` | Accepts `path_name` or `mission_file`; returns `{"loaded": name, "num_points": N}`. Does not return coordinates. |
| Implemented / testing-only | POST | `/api/path/publish` | `server/routes/path.py` | Publishes named path directly to ROS `/path` | Bypasses `OffboardController`; useful for testing, not normal frontend preview/load flow. |
| Implemented | DELETE | `/api/path/{filename}` | `server/routes/path.py` | Deletes uploaded file from missions dir | Returns `{"deleted": filename}`. |

## Socket.IO Events

| Status | Event | Source | Side effects | Acknowledgment |
|---|---|---|---|---|
| Implemented | `mission_load` | `server/sockets/events.py` | Loads points into `OffboardController` | Emits `mission_loaded` with `name` and `num_points`, or `mission_error`. Does not return coordinates. |

## Frontend Flow

Plain-screen path preview should use the read-only preview endpoint:

```text
GET /api/paths
GET /api/path/{name}/preview
```

Mission load should happen only after the operator selects/confirms the path:

```text
POST /api/mission/load
```

Upload flow:

```text
POST /api/path/upload
GET /api/path/{saved_name}/preview
POST /api/mission/load
```

DXF inspection flow:

```text
POST /api/path/parse-dxf
GET /api/path/{saved_name}/preview
POST /api/path/plan
POST /api/mission/load
```

## Coordinate Contract

Preview coordinates are local NED:

```text
north -> path northing in metres
east  -> path easting in metres
frame -> local_ned
```

For plain SVG/canvas drawing:

```text
x = east
y = -north
scale and center using bounds
```

## Important Boundaries

- `GET /api/paths` lists names/counts only; it does not expose geometry.
- `GET /api/path/{name}/preview` is the lightweight coordinate endpoint for display.
- `POST /api/path/plan` is for full planning and validation, not simple click-to-preview.
- `POST /api/mission/load` changes controller state and intentionally returns only load status.
- `POST /api/path/upload` stores files only; preview should be requested separately.
