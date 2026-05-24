# 03 — Token auth UI + connection status badge

**Agent:** Gemma 3 1B
**Estimated diff:** ~60 lines, 2 files
**Depends on:** none
**Blocks:** —

## Goal

Give the operator a way to:

1. See, at a glance, whether the SPA is talking to a real backend or running in
   mock mode.
2. Paste / clear the rover auth token without editing localStorage by hand.

## Files to read first

- `front-end/lib/api.jsx` — `setToken`, `getToken`, `isConnected` already exist.
- `front-end/lib/store.jsx` — `backendConnected`, `backendError` already in
  context.
- `front-end/tweaks-panel.jsx` — pattern for an existing settings drawer.
- `front-end/app.jsx` — where the status bar lives.

## Scope

### A. Connection badge in the top status bar

- One small pill, visible on every screen.
- Three states:
  - `backendConnected === true` → green dot + "live"
  - `backendConnected === false && backendError == null` → grey dot + "offline · mock"
  - `backendError != null` → red dot + "error" with tooltip = the error text
- Pill is read-only. Clicking it opens the tweaks panel scrolled to the
  "Connection" section (added below).

### B. Connection section inside tweaks panel

Add a new section labelled "Connection" with:

- Read-only line showing `api.isConnected` (live / offline).
- Read-only line showing the active base URL (currently always
  `http://localhost:5001` — pulled from `api.jsx`; do NOT make it editable in
  this task, that's task 10).
- Text input for the auth token. Pre-fills with `api.getToken()`. Save button
  calls `api.setToken(value)`. Clear button calls `api.setToken("")`.
- Hint text below: "Leave blank if backend runs with ROVER_DISABLE_AUTH=1."

## Out of scope

- Changing the base URL (task 10).
- Auto-generating a token (backend does this; we only display / paste).
- Validating the token by making a test call (just save it; let the next real
  call surface a 401).

## Acceptance criteria

- [ ] With backend off, badge shows grey "offline · mock" within 5 s of load.
- [ ] With backend on, badge shows green "live" within 2 s of load.
- [ ] After backend kill, badge transitions grey within 10 s (Socket.IO
      reconnect attempts time out).
- [ ] Setting a token, refreshing the page, opening tweaks → token persists.
- [ ] Clearing token, refreshing → empty.
- [ ] Tooltip on red error pill shows the actual `backendError` string.
- [ ] No new console errors. No new dependencies.

## Notes for the agent

- The badge component is < 20 lines. Inline it in `app.jsx` next to the
  existing header.
- The tweaks panel section follows the existing pattern (look at how
  "Typeface" / "Density" radios are defined). Add a `<TweakSection
  title="Connection">` wrapper if no such primitive exists; otherwise mimic
  the layout of the existing sections.
- Do NOT add any polling. `backendConnected` updates via the existing
  Socket.IO connect / disconnect handlers in `store.jsx`.
