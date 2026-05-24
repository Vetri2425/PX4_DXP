# 05 — Path upload pipeline (DXF / SVG / CSV → backend)

**Agent:** GLM (4.5 or 5.1)
**Estimated diff:** ~300 lines, 3-4 files
**Depends on:** 04 (path list must exist to show the new upload)
**Blocks:** —

## Goal

Let the operator pick a file (DXF, SVG, or QGC `.waypoints`, or plain CSV) and
ship it to the backend's `path_manager`, which converts it to a NED polyline,
saves it to `server/missions/`, and exposes it via `/api/paths`.

## Files to read first

- `front-end/screens/draw.jsx` — `<UploadPanel>` already exists with file
  picker UI; it currently just sets local state.
- `front-end/screens/dxf.jsx` — DXF preview + inspector (already client-side).
- `front-end/lib/dxf-data.jsx` — DXF parsing helpers.
- `server/path_manager.py` — load/save logic; confirm it accepts
  `.waypoints`, `.csv`. (DXF / SVG conversion may need to happen frontend-side
  before upload — see Scope C.)
- `server/routes/path.py` — confirm `POST /api/path/upload` accepts
  `multipart/form-data` with field `file`. If it does not, this task is
  blocked — file a backend-extension task per
  `02_setup_and_conventions.md` § "Backend extension protocol".

## Scope

### A. api.jsx

- `api.uploadPath(file, opts) → Promise<{name, kind, length_m, n_points}>` →
  `POST /api/path/upload` multipart, with optional `opts.name` (override the
  default name derived from the filename).
- Use `FormData`. Do not set `Content-Type` manually — let the browser set
  the boundary. Token header goes via the existing `_headers()` if present.

### B. UploadPanel rewrite

- File input accepts `.dxf, .svg, .waypoints, .csv`.
- After file pick:
  - QGC `.waypoints` and `.csv` → upload directly (one POST).
  - SVG → parse to polyline client-side (existing dxf-data.jsx has a path
    flattener; if SVG needs a separate flatten, use the browser's `Path2D` and
    `getTotalLength()` sampler at 1 cm spacing). Upload the resulting polyline
    as a CSV (`n,e` per line).
  - DXF → use existing dxf-data.jsx to parse to polylines. Upload the
    aggregated polyline as a CSV.
- Show progress: pending → uploading → done / error.
- On success: call `r.refreshPaths()` (add this to store; trivial wrapper
  around the same fetch from task 04) so the gallery picks up the new entry.

### C. Optional: backend conversion

If the backend's `path_manager` already accepts DXF / SVG natively, prefer
sending the raw file and letting the backend convert. Inspect
`server/path_manager.py` first; if there's an obvious extension-dispatch with
DXF/SVG entries, just POST the raw file. Otherwise convert in the browser as
above.

### D. Error surfacing

- File too large (> 10 MB) → reject client-side with a clear message; do not
  upload.
- Backend returns 400 with a reason → show the reason in the panel, do not
  clear the file picker.
- Backend returns 5xx → toast "upload failed — retry?" with a retry button.

## Out of scope

- Editing the polyline after upload (that's the existing DXF inspector, which
  stays client-side).
- Multi-file batch upload.
- Re-naming an existing path on the server.
- Deleting paths (separate future task).

## Acceptance criteria

- [ ] Upload a small `.waypoints` file → appears in gallery within 1 s.
- [ ] Upload a DXF → converted client-side or server-side → appears in gallery
      with `n_points > 0`.
- [ ] Upload an SVG with one path → appears in gallery.
- [ ] Upload a corrupt file → backend rejects, panel shows reason, gallery
      unchanged.
- [ ] Upload an 11 MB file → blocked client-side, no request made.
- [ ] After successful upload, selecting the new entry shows the polyline
      preview on the map (task 04 behaviour, must still work).
- [ ] No memory leak from FileReader (release blob URL after upload).

## Notes for the agent

- If you find the backend lacks DXF/SVG handling and adding it would balloon
  this task, do the client-side flatten path. Backend-side conversion is
  nicer but only if it's already 80 % there.
- For DXF: rover paths are 2D plots in metres. Drop Z. Assume the DXF is
  already in plot-frame metres unless the file has a `$INSUNITS` header
  indicating mm or in — then scale.
