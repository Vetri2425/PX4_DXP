# 11 — Screen-action audit + dead-button sweep

**Agent:** Haiku 4.5
**Estimated diff:** ~150 lines, touches every screen
**Depends on:** 03-10 (do this last)
**Blocks:** —

## Goal

After tasks 03-10 land, the SPA still contains buttons / menus / inputs that
were styled but never wired (legacy from the initial mock-only build). Walk
every screen, identify dead UI, and either wire it to the real backend
(preferred) or remove it.

## Files to read first

- All of `front-end/screens/*.jsx`.
- Cross-reference with the api surface in `front-end/lib/api.jsx` (post-task-10).

## Scope — per screen

For each of `home.jsx`, `drive.jsx`, `draw.jsx`, `dxf.jsx`, `map.jsx`,
`more.jsx`, `sub-1.jsx`, `sub-2.jsx`, `sub-3.jsx`:

1. List every interactive element (`<Btn>`, `<input>`, dropdown, toggle).
2. For each, determine if it has a real action.
3. Apply one of three fixes:
   - **Wire**: bind to an existing api call or store action.
   - **Disable**: if the feature isn't supported yet, set `disabled` and add
     a tooltip "coming soon".
   - **Remove**: if the feature was an early mockup that's no longer
     planned, delete the element.

### Specific known dead spots (start here)

- `drive.jsx` — joystick / manual control buttons. Wire to a manual-velocity
  endpoint if it exists (`POST /api/manual_cmd {vx, vy, wz}`); otherwise
  disable.
- `draw.jsx` — "Save & send", "Import .gcode", "Choose file" inside the
  upload-empty state. Some are duplicated from task 05; consolidate.
- `more.jsx` — Settings sub-cards. If they don't point to a real sub-screen
  with real content, remove or stub with "coming soon".
- `sub-3.jsx` — confirm it's actually reachable and serves a purpose; if
  not, delete the file and remove the route.

### Cross-cutting

- Every screen's primary CTA must work or be visibly disabled.
- No `onClick={() => {}}` placeholders should remain.
- Loading states: where wiring is async, show a small spinner / disabled
  state during the await.

## Out of scope

- Re-designing screens (visual changes that aren't enabling/disabling).
- Adding new features.
- Changing navigation structure.

## Acceptance criteria

- [ ] Click every interactive element in the SPA → either it does something
      real, or it's clearly disabled with a tooltip.
- [ ] No console errors, no `onClick={() => {}}` placeholders, no
      `console.log("TODO")` calls.
- [ ] Removed screens / components are also removed from the HTML script
      load order and from `app.jsx` routing.

## Notes for the agent

- Keep a short summary at the top of your PR description: "Wired: X, Y.
  Disabled: A, B. Removed: foo.jsx." This is the audit trail.
- If you find a button that probably *should* work but the backend endpoint
  is missing → write a new `Nx_backend_extend_<thing>.md` task file. Do not
  add server endpoints inline in this task.
