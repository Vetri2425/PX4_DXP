# 06 — PX4 params read / write panel

**Agent:** Haiku 4.5
**Estimated diff:** ~200 lines, 2-3 files
**Depends on:** —
**Blocks:** —

## Goal

Replace seeded `SEED_PX4_PARAMS` with a live params table that reads from PX4
via MAVROS and lets the operator change them safely.

## Files to read first

- `front-end/lib/store.jsx` — `SEED_PX4_PARAMS` shape.
- `server/routes/params.py` — confirm endpoints exist:
  - `GET /api/params?names=...&group=...` → list / search
  - `GET /api/params/{name}` → single
  - `POST /api/params/{name}` body `{value: number}` → set
  - If they don't exist with this shape, file a backend-extension task and
    stop.
- `front-end/app.jsx` — find where `r.px4Params` is consumed (a screen or sub).

## Scope

### A. api.jsx

- `api.listParams(opts)` → REST GET with optional `group` or `search` query.
- `api.getParam(name)` → REST GET single.
- `api.setParam(name, value)` → REST POST.

### B. store.jsx

- Replace direct use of `SEED_PX4_PARAMS` with a `params` state, populated on
  first connect via `api.listParams()`.
- Keep `SEED_PX4_PARAMS` as mock fallback when offline.
- Add `paramsLoading`, `paramsError` flags.
- Add `setParam(name, value)` action that calls `api.setParam` then updates
  local state on success.

### C. Params panel UI

Find the existing screen that renders params (likely `screens/more.jsx` or a
sub-screen). Re-skin to:

- Group params by `group` field. Collapsible sections.
- Each row: name, current value, range hint, edit button.
- Edit opens a numeric input. Submit calls `setParam`. Cancel reverts.
- Show last-fetch timestamp at the top + a manual refresh button.
- Disable edit when `!backendConnected` (mock mode is read-only).

### D. Safety

Some params can brick the rover. Wrap these names in a "danger zone"
collapsed-by-default section and require a confirm modal:

- `COM_RCL_EXCEPT`
- `EKF2_*`
- `RD_*`
- `MOT_*`
- Anything not in a known group → also danger zone by default.

## Out of scope

- Param multi-edit / batch save.
- Param file import / export (use QGC for that).
- Saving to flash (the backend's set endpoint is responsible; if it needs an
  explicit "save to flash" call, expose it as a separate button only).

## Acceptance criteria

- [ ] Backend on → params panel populates within 3 s.
- [ ] Backend off → falls back to seed values, edit disabled, no errors.
- [ ] Changing a "safe" param → backend accepts → local value updates.
- [ ] Changing a "danger zone" param → confirm modal first.
- [ ] Setting an out-of-range value → backend rejects → modal stays open,
      shows reason.
- [ ] No automatic re-fetch loop (only refresh on connect or manual button).

## Notes for the agent

- The PX4 param list is several hundred entries. Fetch only the ones in
  `SEED_PX4_PARAMS` groups for the v1 panel; defer full list to a search.
- Numeric formatting: floats to 4 sig fig, ints as-is.
