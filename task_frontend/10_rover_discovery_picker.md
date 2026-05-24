# 10 — Rover discovery picker (UDP beacon → base URL switch)

**Agent:** GLM (4.5 or 5.1)
**Estimated diff:** ~250 lines, 3 files + small backend addition
**Depends on:** 03 (settings panel exists)
**Blocks:** —

## Goal

The current frontend hard-codes `http://localhost:5001`. Let the operator
discover rovers on the LAN automatically (UDP beacon on port 5002), pick
one, and switch the base URL without editing code.

## Files to read first

- `server/beacon.py` — confirm a UDP broadcast already exists and what
  payload it ships.
- `front-end/lib/api.jsx` — `API_BASE`, `SOCKET_URL` constants.

## Scope

### A. Beacon payload contract

The beacon must broadcast (at minimum) every 2 s on UDP 5002:

```
{
  "type": "rover_beacon",
  "name": "DXP-01 Mercutio",
  "id": "dxp-01",
  "host": "192.168.1.102",
  "port": 5001,
  "auth_required": true | false,
  "fw_commit": "617cce5a",
  "uptime_s": 1234
}
```

If the existing beacon ships less than this, extend it. Keep it small.

### B. Discovery in the browser (the hard part)

Browsers cannot read UDP directly. Two options:

1. **Backend-mediated discovery** (preferred): the SPA POSTs to the *current*
   backend (or any reachable backend) at `/api/discover`, and that backend
   returns a list of beacons it has seen on the LAN. Each backend already
   listens to its own UDP socket — it can keep a 10 s cache of others.
2. **WebRTC + local helper**: rejected, overkill.

Go with option 1. If no backend is reachable yet (first boot, fresh laptop),
fall back to a manual base-URL entry box (already in task 03 settings panel
as read-only — make it editable in this task).

### C. Base URL lifting

- `api.jsx`: replace const `API_BASE` / `SOCKET_URL` with a getter that reads
  `localStorage["rover_base_url"]` (default `http://localhost:5001`).
- `api.setBaseUrl(url)`: persists + reconnects the Socket.IO client.
- Reconnect logic must tear down the old socket before creating the new one.

### D. Picker UI

In the settings panel (extends task 03 Connection section):

- "Discover rovers" button → calls `api.discover()` → shows a list of
  beacons with name, host, fix-type indicator (small green dot if their
  `auth_required` is satisfied by current token, grey if not).
- "Connect" button on each row → calls `api.setBaseUrl()`.
- "Manual entry" expander: text field for `http://host:port`, save button.

### E. Active rover surfacing

- The discovered name appears next to the connection badge from task 03.
- Persist the chosen rover ID in `localStorage["active_rover_id"]` so
  reloading reconnects to the same one.

## Out of scope

- mDNS / Bonjour (UDP beacon is enough).
- Multi-rover simultaneous control.
- Encrypted beacon payloads.
- Rover-side filtering by operator credential (server checks token on
  actual API calls; the beacon is public).

## Acceptance criteria

- [ ] Two rovers on the LAN broadcasting → discover button lists both within
      3 s.
- [ ] Picking rover B → all subsequent REST + Socket.IO calls hit B.
- [ ] Reload → reconnects to B without re-picking.
- [ ] Manual entry of a non-existent URL → save persists → next call fails
      → backend status badge shows error.
- [ ] No two Socket.IO clients alive simultaneously after a switch
      (DevTools network tab: only one WS connection open).
- [ ] If `/api/discover` doesn't exist on the current backend, the manual
      entry path still works.

## Notes for the agent

- Switching base URL while a mission is running on the current rover is
  *not* a graceful operation — show a confirm modal: "Switching will drop
  telemetry from <current>. Continue?"
- Keep the discover button's call rate-limited (no more than once per 2 s
  to match beacon cadence).
