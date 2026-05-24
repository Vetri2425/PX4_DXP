# Setup & Conventions (read once before any task)

## Local dev

Two terminals:

```
# Terminal 1 — backend (on Jetson, or Windows for UI-only iteration)
cd PX4_DXP/server
pip install -r requirements.txt
$env:ROVER_DISABLE_AUTH = "1"      # PowerShell
# or: export ROVER_DISABLE_AUTH=1   # bash
uvicorn main:app --reload --host 0.0.0.0 --port 5001

# Terminal 2 — static SPA
cd PX4_DXP/front-end
python -m http.server 3000
# open http://localhost:3000/PX4%20DXP.html
```

On Windows the helper `run_dev.ps1` at the repo root launches the SPA.

For pure UI iteration with no backend, just open the HTML — mock fallback
takes over automatically.

## Frontend code style

- **JSX without build step.** The HTML loads each `*.jsx` via
  `<script type="text/babel">`. Order in `PX4 DXP.html` matters: libs →
  `lib/*.jsx` → `tweaks-panel.jsx` → `screens/*.jsx` → `app.jsx`.
- **No imports / no exports.** Everything attaches to `window.*` or runs at
  top level. New shared utilities → attach to `window.api`, `window.useRover`,
  or define a top-level component the same way `<UploadPanel>` does.
- **React 18 hooks API.** `React.useState`, `React.useEffect`,
  `React.useContext`. No `import React`.
- **Functional components only.** No classes.
- **CSS in app shell.** Style classes are defined in the existing CSS in
  `PX4 DXP.html`. Reuse class names; do not invent ad-hoc inline styles
  unless a one-off.
- **Icons via `<I.name size={n} />`** from `lib/icons.jsx`. If you need a new
  icon, add it to `icons.jsx` matching the existing SVG pattern.
- **Buttons via `<Btn variant="primary|secondary|ghost" size="sm|md" full icon={...}>`** from `lib/ui.jsx`.
- **Strings:** no i18n yet. English, sentence case for buttons, lowercase for
  status pills ("connected", "armed", "offline").

## Async patterns

- `await api.someCall()` inside an `async` event handler. Wrap in `try/catch`
  and surface failures via `setBackendError` (global) or a local error state
  (per-screen).
- Never fire two REST calls back-to-back without `await` between them.
- Never start an interval inside a render — only in `useEffect`. Always return
  a cleanup function.

## State shape additions

When adding to the `RoverProvider` value object, group the new field with
similar concerns and update the spread at the bottom of `store.jsx`. Keep the
seed mock value in the same place as the real wiring so falling back is a
one-line conditional.

## Backend extension protocol

If a task needs a backend endpoint or event that does not exist:

1. Do NOT add it inline.
2. Stop the task.
3. Write a new file in this folder: `Nx_backend_extend_<thing>.md` describing
   exactly the endpoint, the request / response shape, and which existing
   `server/routes/*.py` it belongs in.
4. Mark the current task as blocked on that file.
5. Move on.

## Verification checklist (every task)

Before marking a task done:

- [ ] Backend running, SPA loaded → feature works end-to-end at least once.
- [ ] Backend killed → SPA still renders, no console errors, mock badge shows.
- [ ] Backend restarted → SPA reconnects within 5 s and resumes real data.
- [ ] DevTools Network tab: no command fires more than once per click.
- [ ] DevTools Console: no red errors, no infinite warnings.
- [ ] If the task touches `store.jsx`: the `t.*` shape is preserved (no fields
      removed, only added).

If any of these fail, the task is not done.
