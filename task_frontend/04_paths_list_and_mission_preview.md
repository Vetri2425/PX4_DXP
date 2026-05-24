# 04 — Paths list fetch + mission preview on map

**Agent:** Haiku 4.5
**Estimated diff:** ~150 lines, 3 files
**Depends on:** —
**Blocks:** 05 (uploads must refresh this list)

## Goal

Replace the seeded `SEED_GALLERY` mock with the real path catalog from the
backend, and render the selected path's waypoints as a preview overlay on the
map screen.

## Files to read first

- `front-end/lib/store.jsx` — `SEED_GALLERY`, `activeJob` shape.
- `front-end/lib/api.jsx` — REST wrapper pattern.
- `front-end/screens/draw.jsx` — gallery panel that uses `r.gallery`.
- `front-end/screens/map.jsx` — current map render (no waypoints yet).
- `server/routes/path.py` — confirm `GET /api/paths` returns
  `{ paths: [{ name, kind, length_m, n_points, ... }] }` and
  `GET /api/paths/{name}` returns `{ name, points: [{n, e}, ...] }`.

## Scope

### A. api.jsx

Add two wrappers:

- `api.listPaths() → Promise<{paths: PathSummary[]}>` → `GET /api/paths`
- `api.getPath(name) → Promise<{name, points: {n,e}[]}>` → `GET /api/paths/{name}`

### B. store.jsx

- Add state `paths` (array, default `[]`).
- Add state `pathsLoading` (bool).
- Add state `selectedPathPoints` (array of `{n,e}`, default `null`).
- On mount, when `backendConnected` flips to true, call `api.listPaths()` and
  populate `paths`. On error, fall back to `SEED_GALLERY` and set a flag
  `pathsAreMock = true`.
- Add action `loadPathPreview(name)` that calls `api.getPath(name)` and stores
  result in `selectedPathPoints`.
- Expose `paths`, `pathsAreMock`, `selectedPathPoints`, `loadPathPreview` in
  the context value.

### C. Draw screen gallery

- Replace `r.gallery.map(...)` with `r.paths.map(...)` when not mock.
- Each entry shows: name, length, n_points (these come from the backend
  summary). For mock fallback, keep the existing visual.
- Clicking an entry: calls `r.loadPathPreview(name)` AND sets `r.activeJob`
  with the new metadata.

### D. Map screen overlay

- When `r.selectedPathPoints` is non-null, render the points as a polyline on
  the map canvas. Use a thin (1.5 px) accent-coloured stroke.
- Anchor the polyline relative to the rover's current `pos_n / pos_e` so the
  origin lines up. Do NOT do GPS projection — the path is already in local NED
  metres. 1 metre = whatever the map's current px/m scale is. Reuse the same
  scaler as the existing path-trace tweak (`tweaks.showTrace`).
- Add a small legend chip: "preview: <name> · <n_points> pts".

## Out of scope

- Path upload (task 05).
- Triggering `api.loadMission(name)` from the gallery — that already happens
  via `apiMissionLoad`. This task is preview-only.
- Map projection in lat/lon (task 09).
- Editing waypoints.

## Acceptance criteria

- [ ] Backend on with at least one path on disk → gallery shows real entries,
      pathsAreMock = false.
- [ ] Backend off → gallery shows seed entries, pathsAreMock = true, no
      console errors.
- [ ] Clicking a real entry → map shows polyline preview within 200 ms.
- [ ] Reconnecting to backend re-fetches paths (no need to refresh page).
- [ ] Selecting another path replaces the preview (does not stack).
- [ ] No request storm on reconnect (one call to `/api/paths`, not a loop).

## Notes for the agent

- Keep the `SEED_GALLERY` constant in place — it's the mock fallback.
- The map screen's existing path-trace toggle is a good template for the
  preview overlay rendering.
