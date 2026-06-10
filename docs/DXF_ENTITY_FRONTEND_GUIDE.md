# DXF Entity Frontend Integration

This page covers the frontend-facing DXF entity workflow added for entity
selection, per-entity spray toggles, and extension preview/config.

Base URL: `http://<jetson>:5001`

Auth header for protected endpoints:

```http
X-Rover-Token: <token>
```

## Endpoints Added

Four endpoint surfaces are involved:

| Method | Endpoint | Purpose |
|---|---|---|
| `GET` | `/api/path/{name}/entities` | Fast entity-level DXF preview for rendering, clicking, and snapping. |
| `POST` | `/api/path/{name}/entities` | Save per-entity spray enable/disable decisions. |
| `GET` | `/api/path/{name}/extensions` | Read saved PRE/AFT extension settings for this DXF. |
| `POST` | `/api/path/{name}/extensions` | Save PRE/AFT extension enable/disable and lengths. |

Existing endpoints still matter:

| Method | Endpoint | Purpose |
|---|---|---|
| `POST` | `/api/path/parse-dxf` | Upload/save the DXF and get initial entity summaries. |
| `GET` | `/api/path/{name}/preview` | Dense planned waypoint preview with `spray` flags. |
| `POST` | `/api/path/plan` | Full planning/alignment/staging. Uses saved entity and extension settings. |

## Recommended Flow

```text
POST /api/path/parse-dxf
  -> filename

GET /api/path/{filename}/entities
  -> draw each entity separately
  -> show spray toggle per entity
  -> show extension preview if enabled

POST /api/path/{filename}/entities
  -> persist spray/no-spray decisions

POST /api/path/{filename}/extensions
  -> persist extension enable + lengths

GET /api/path/{filename}/entities
  -> redraw entities with saved spray state and extension preview

POST /api/path/plan
  -> planner applies saved spray overrides and saved extension config
```

## Entity Preview

Call:

```http
GET /api/path/soccer_pitch_fifa_edited.dxf/entities
```

Important response fields:

```jsonc
{
  "name": "soccer_pitch_fifa_edited.dxf",
  "frame": "local_ned",
  "bounds": {
    "north_min": 0.0,
    "north_max": 68.0,
    "east_min": 0.0,
    "east_max": 105.0
  },
  "extension_config": {
    "enabled": true,
    "pre_extension_m": 0.5,
    "aft_extension_m": 0.5
  },
  "entities": [
    {
      "entity_id": "9E",
      "entity_type": "LINE",
      "layer": "BOUNDARY",
      "default_is_mark": true,
      "is_mark": false,
      "length_m": 105.0,
      "geometry": {
        "start": [0.0, 0.0],
        "end": [0.0, 105.0]
      },
      "preview_points": [
        { "north": 0.0, "east": 0.0 },
        { "north": 0.0, "east": 105.0 }
      ],
      "extension_preview": {
        "enabled": false,
        "pre_length_m": 0.0,
        "aft_length_m": 0.0,
        "pre_points": [],
        "aft_points": []
      }
    }
  ]
}
```

Frontend use:

- Render each `entities[]` item as its own selectable shape.
- Use `entity_id` as the stable selection key.
- Use `preview_points` for canvas drawing and hit-testing.
- Use `geometry` for exact anchors such as line endpoints or circle centers.
- Use `default_is_mark` to show the original backend/layer decision.
- Use `is_mark` as the current operator-edited spray state.

Coordinate frame:

- `north` is local NED north in metres.
- `east` is local NED east in metres.
- For `/api/path/plan` reference points, map clicked local NED back as:
  - `dxf_x = east`
  - `dxf_y = north`

## Spray Toggle

Call:

```http
POST /api/path/soccer_pitch_fifa_edited.dxf/entities
Content-Type: application/json
```

Body:

```json
{
  "overrides": [
    { "entity_id": "9E", "is_mark": false },
    { "entity_id": "9F", "is_mark": true }
  ]
}
```

Response:

```json
{
  "name": "soccer_pitch_fifa_edited.dxf",
  "saved": true,
  "num_overrides": 2
}
```

Meaning:

- `is_mark: true` means spray ON for that entity.
- `is_mark: false` means transit/no-spray for that entity.
- Saved values are stored server-side in a hidden sidecar file.
- The next `GET /entities`, `GET /preview`, and `POST /plan` use the saved state.

## Extension Config

Extensions are configured per DXF file, not inside `/api/path/plan`.

Call:

```http
POST /api/path/soccer_pitch_fifa_edited.dxf/extensions
Content-Type: application/json
```

Body:

```json
{
  "enabled": true,
  "pre_extension_m": 0.5,
  "aft_extension_m": 0.5
}
```

Response:

```json
{
  "name": "soccer_pitch_fifa_edited.dxf",
  "saved": true,
  "enabled": true,
  "pre_extension_m": 0.5,
  "aft_extension_m": 0.5
}
```

Read current config:

```http
GET /api/path/soccer_pitch_fifa_edited.dxf/extensions
```

Rules:

- `enabled: false` disables extension preview and planning extensions.
- `pre_extension_m` is the no-spray run-in before a mark.
- `aft_extension_m` is the no-spray run-out after a mark.
- Lengths must be `>= 0`.
- Extensions apply only to entities whose effective `is_mark` is `true`.
- If the operator sets an entity to `is_mark: false`, that entity gets no extension preview and no marking extension in the final plan.

## Extension Preview

When extension config is enabled, `GET /entities` returns per-entity
`extension_preview`.

Example:

```jsonc
{
  "entity_id": "A1",
  "is_mark": true,
  "preview_points": [
    { "north": 0.0, "east": 0.0 },
    { "north": 1.0, "east": 0.0 }
  ],
  "extension_preview": {
    "enabled": true,
    "pre_length_m": 0.5,
    "aft_length_m": 0.25,
    "pre_points": [
      { "north": -0.5, "east": 0.0 },
      { "north": 0.0, "east": 0.0 }
    ],
    "aft_points": [
      { "north": 1.0, "east": 0.0 },
      { "north": 1.25, "east": 0.0 }
    ]
  }
}
```

Frontend rendering recommendation:

- Draw `preview_points` using spray/no-spray style from `is_mark`.
- Draw `extension_preview.pre_points` and `aft_points` as no-spray/transit style.
- Include extension points when fitting the canvas. The endpoint already expands
  top-level `bounds` to include extension preview points.

## Planning Contract

The frontend should not send extension fields in `/api/path/plan`.

Do this:

```text
POST /api/path/{filename}/extensions
POST /api/path/plan
```

Not this:

```jsonc
{
  "source": "soccer_pitch_fifa_edited.dxf",
  "enable_path_extensions": true,
  "pre_extension_m": 0.5,
  "aft_extension_m": 0.5
}
```

Those `/plan` fields are deprecated for API use. The planner reads the saved
extension config for the file.

