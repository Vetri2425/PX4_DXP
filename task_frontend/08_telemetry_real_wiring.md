# 08 — Real telemetry wiring (charts, RPP widget, missing fields)

**Agent:** Haiku 4.5
**Estimated diff:** ~180 lines, 2-3 files + small backend additions
**Depends on:** —
**Blocks:** 09 (map needs real GPS)

## Goal

Stop showing fake numbers for `current`, `temp`, `hdop`, `rssi`, `motor[]`,
and add a live RPP cross-track widget. The voltage / battery charts should
also draw real history once that data exists.

## Files to read first

- `front-end/lib/store.jsx` — telemetry shape, `t.history`, the mock tick.
- `server/ros_node.py` — see which MAVROS topics are subscribed. The missing
  fields are likely available on `/mavros/battery`, `/mavros/wind_estimation`,
  `/mavros/gpsstatus/gps1/raw`, `/rpp/debug`, but may not yet be in the
  outgoing payload.
- `server/models.py` — the outgoing `TelemetryData` shape.

## Scope

### A. Backend payload additions (small)

Extend the telemetry payload (single new task, *not* per-field):

- `current_a` — from `/mavros/battery.current` (already subscribed).
- `hdop` — from `/mavros/gpsstatus/gps1/raw.eph / 100.0`.
- `motor_pwm[4]` — from `/mavros/rc/out` (channel 1-4 if available; if not
  available leave as `null`).
- `rpp_xtrack_m` — from `/rpp/debug[0]`.
- `rpp_heading_err_rad` — from `/rpp/debug[1]`.
- `rpp_lookahead_m` — from `/rpp/debug[2]`.
- `rpp_state_code` — from `/rpp/debug[7]`.

Push the 10 Hz Socket.IO `telemetry` event with these added.

`temp` and `rssi` are NOT available on standard MAVROS topics for this rover
build → leave them out of the payload, frontend keeps mock fallback for those
two only.

### B. api.jsx / store.jsx

- Extend `onTelemetry` mapping to populate the new `t.*` fields:
  - `t.current` ← `data.current_a`
  - `t.hdop` ← `data.hdop`
  - `t.motor` ← `data.motor_pwm` (when not null; map 1000-2000 PWM to 0-100 % for the bar visual: `pct = (pwm - 1000) / 10`)
  - `t.rppXtrack` ← `data.rpp_xtrack_m`
  - `t.rppHeadingErr` ← `data.rpp_heading_err_rad`
  - `t.rppLookahead` ← `data.rpp_lookahead_m`
  - `t.rppState` ← lookup from `[STALE,IDLE,TRACKING,APPROACH,DONE]`

### C. UI changes

- The HUD overlay (when `tweaks.showHud === true`) gets a row showing:
  `xtrack: ±X.X cm · heading_err: ±Y.Y° · LA: Z.ZZ m · state: TRACKING`.
- Coloured background for xtrack: green < 3 cm, amber 3-8 cm, red > 8 cm.
- The voltage / current / cpu sparkline charts in the existing telemetry
  panels now use the live values pushed in via `data.battery_v` /
  `data.current_a` (CPU stays mock — no backend source yet).

## Out of scope

- A full graph of xtrack over time (could be a future "rpp telemetry" sub).
- Tuning UI for RPP params (that's covered indirectly by task 06 params
  panel + future RPP-specific knobs).

## Acceptance criteria

- [ ] In OFFBOARD with a path running, HUD shows `state: TRACKING` and a
      non-zero `xtrack` that responds to physical disturbance.
- [ ] Charging the battery: `current` reads negative-ish or near-zero, not
      stuck at mock 4.2 A.
- [ ] HDOP from real RTK fix shows ~0.5-1.0, not the random mock.
- [ ] Disconnect → HUD pills hide (or show a small "—") instead of stale
      numbers.

## Notes for the agent

- If the backend doesn't currently subscribe to a topic you need, add the
  subscription in `server/ros_node.py` and surface the field in
  `models.TelemetryData`. Keep additions minimal — do not refactor the model.
- The 50 Hz `/rpp/debug` stream is much faster than the 10 Hz telemetry rate;
  snapshot the latest value, don't average.
