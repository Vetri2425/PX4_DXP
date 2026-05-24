# 07 — Live logs panel + ROS node status

**Agent:** Haiku 4.5
**Estimated diff:** ~220 lines, 2-3 files + 1 small backend addition
**Depends on:** —
**Blocks:** —

## Goal

Replace `SEED_LOGS` and `SEED_ROS_NODES` with real data.

## Files to read first

- `front-end/lib/store.jsx` — both seeds.
- `server/sockets/events.py` — see if a `log` event is already emitted. If
  not, this task needs a small backend addition.
- `server/ros_node.py` — see what topic/node introspection is already done.

## Scope

### A. Backend extension (required, scoped)

If `/api/logs` and a Socket.IO `log` event don't exist, add:

- `GET /api/logs?limit=200&level=info|warn|error&since=<iso>` →
  `{ logs: [{t, lvl, src, msg}] }` — tails `journalctl -u px4-dxp.service` or
  the structured log file.
- Socket.IO `log` event broadcast on every new log line, at most 20 Hz
  (coalesce bursts).
- `GET /api/ros/nodes` → `{ nodes: [{ name, hz, status, cpu, topics }] }` —
  one snapshot, computed lazily (do not push at 10 Hz).

If you have to add these, keep them tiny — no log rotation logic, no full
log-search backend. Just a tail.

### B. api.jsx

- `api.listLogs(opts)` → GET wrapper.
- `api.listRosNodes()` → GET wrapper.
- `onLog` handler in `initSocket`.

### C. store.jsx

- `logs` state, capped at 500 entries (FIFO).
- `rosNodes` state, refreshed on connect + every 10 s while connected.
- Mock fallback to `SEED_LOGS` / `SEED_ROS_NODES`.
- New `onLog` socket handler appends to the head, drops the tail past 500.

### D. UI

Find the existing screens that render logs and nodes (likely `more.jsx` or
sub-screens 1/2/3). Re-wire to read from store. Add:

- Logs panel: level filter pills (info/warn/error/debug), auto-scroll toggle,
  pause/resume button (stops auto-scroll but new logs still arrive in store).
- Nodes panel: status colour dot (ok = green, warn = amber, error = red),
  refresh button, last-snapshot timestamp.

## Out of scope

- Log search across history.
- Persistent log storage from the frontend.
- Per-node restart buttons.

## Acceptance criteria

- [ ] Backend on → logs panel shows last 50 lines within 2 s, then live-tails.
- [ ] Spam 100 log lines in 1 s on backend → frontend shows them all without
      freezing (coalesce buffer, not infinite render loop).
- [ ] Level filter excludes other levels in real time.
- [ ] Pause stops scroll but does not stop ingestion.
- [ ] Nodes panel populates on connect, refreshes on demand.
- [ ] Disconnect → mock data shown, no error toast.

## Notes for the agent

- The mock seed has timestamps with milliseconds; preserve that precision.
- ROS introspection on Jetson can be slow (1-2 s for full node graph); the
  10 s polling cadence is intentional. Do not bump to 1 Hz.
