# task_frontend — Frontend Wiring Work Queue

Self-contained agent tasks to finish wiring the React SPA in `front-end/` to the
FastAPI + Socket.IO + rclpy backend in `server/`. Each task file is sized for a
specific small coding agent and is independently executable.

## How to use this folder

1. Pick a task file numbered `02_*` and up.
2. Read its `Agent` field — that's the recommended model.
3. Read the file paths listed under `Files to read first`. Do not skip this.
4. Implement only what's in `Scope`. Anything in `Out of scope` is for another task.
5. Verify against `Acceptance criteria`. Do not mark done if a criterion fails.
6. When done, move the file to `task_frontend/done/` (create the folder if needed).

## Agent capability legend

| Agent | Use for | Don't use for |
|---|---|---|
| **Gemma 3 1B** | Single-file mechanical edits, simple state plumbing, copy-paste from a pattern, < 30 lines of changes | Cross-file refactors, anything requiring judgment, anything that touches the backend |
| **Haiku 4.5** | One screen / one feature, well-bounded UI + store wiring, < 200 lines of changes | New ROS2 node behavior, multi-screen redesign |
| **GLM (4.5 / 5.1)** | Multi-file features touching frontend + backend, ROS2 message-flow understanding, > 200 lines, file upload pipelines | Frontend pixel-polish (Haiku is enough) |

## Hard rules for every task

- **Do not modify `server/` unless the task explicitly says so.** The backend
  API surface is fixed for this batch. If something is missing, raise it in a
  new task file rather than silently adding endpoints.
- **Do not edit `src/*.py` (ROS2 nodes).** Those are owned by `task_rpp_upgrade`.
- **Do not break mock fallback.** Every change must still render when the
  backend is unreachable (see `front-end/lib/store.jsx` mock tick).
- **Do not invent fields on `t` (telemetry).** If the backend doesn't publish a
  field, either request it via a new task or leave the mock value.
- **No build step.** The SPA is Babel-in-browser. No bundlers, no transpilers,
  no JSX → JS pre-compile. Keep it that way.
- **No new dependencies** unless the task specifies a CDN script. The HTML loads
  React 18 + socket.io-client from CDN — that is the entire dependency surface.

## What is already done (do not redo)

- `front-end/lib/api.jsx` — REST + Socket.IO wrapper, token auth, all base
  endpoints (arm, setMode, estop, mission load/start/stop/abort, telemetry).
- `front-end/lib/store.jsx` — `RoverProvider` context. Subscribes Socket.IO,
  maps telemetry into `t.*`, exposes `apiSetArmed`, `apiSetMode`, `apiMissionLoad`,
  `apiMissionStart`, `apiMissionStop`, `apiMissionAbort`, `triggerEStop`,
  `clearEStop`, `togglePlay`. Has mock tick fallback.
- Backend health: connect / disconnect / safety-abort events surfaced as
  `backendConnected`, `backendError` state.

## What is still mock / missing (the work queue)

| # | Task | Agent |
|---|---|---|
| 03 | Token auth + connection-status UI | Gemma 1B |
| 04 | Paths list fetch + mission preview on map | Haiku 4.5 |
| 05 | Path upload pipeline (DXF / SVG / CSV → backend) | GLM |
| 06 | PX4 params read / write panel | Haiku 4.5 |
| 07 | Logs panel + ROS node status (real) | Haiku 4.5 |
| 08 | Telemetry real wiring (charts, RPP widget) | Haiku 4.5 |
| 09 | Map: real GPS lat/lon + live path trace | Haiku 4.5 |
| 10 | Rover discovery picker (UDP beacon) | GLM |
| 11 | Screen-action audit + dead-button sweep | Haiku 4.5 |

Architecture and conventions live in `01_architecture.md` and `02_setup_and_conventions.md`. Read those once before picking up any task.
