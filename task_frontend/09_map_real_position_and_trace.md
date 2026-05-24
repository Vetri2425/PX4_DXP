# 09 — Map: real GPS lat/lon + live path trace

**Agent:** Haiku 4.5
**Estimated diff:** ~200 lines, 1-2 files
**Depends on:** 04 (preview overlay), 08 (telemetry fields)
**Blocks:** —

## Goal

The map screen currently renders a stylised plot-frame view. Add a real
geographic mode that uses the live `lat / lon` from telemetry and draws the
rover's actual path as it moves.

## Files to read first

- `front-end/screens/map.jsx` — current renderer.
- `front-end/lib/store.jsx` — `t.lat`, `t.lon`, `t.heading`, `t.alt`.
- The preview overlay added in task 04.

## Scope

### A. Two map modes

A toggle at the top of the map screen: "Plot frame" (existing local-NED view)
and "Geo" (new lat/lon view).

- **Plot frame** = unchanged behaviour. Origin at rover, axes in metres,
  preview polyline drawn from task 04, optional trace.
- **Geo** = projects lat/lon onto a flat (equirectangular) view centred on
  the rover. Resolution: rover icon stays centred, world scrolls. Scale = 1
  metre per N pixels, configurable via existing tweaks.

### B. Live trace

In either mode, accumulate the last 600 samples of (pos_n, pos_e) — or
(lat, lon) in Geo mode — into a ring buffer in store. Render as a polyline
behind the rover icon. The existing `tweaks.showTrace` toggle controls
visibility.

Buffer in store, not in component state — otherwise re-renders thrash it.

### C. Rover icon

- Triangle pointing along `t.heading` (degrees, 0=North CW for both NED and
  the geographic view).
- Coloured by `t.rppState`: TRACKING green, APPROACH amber, DONE blue, IDLE
  grey, STALE red.

### D. Scale + reference

- Always-visible scale bar (e.g. "20 m").
- In Geo mode, small text showing the lat/lon to 6 decimals (about 11 cm
  precision).

## Out of scope

- Tile-based basemaps (OSM, Mapbox, satellite). Stay vector-only.
- Multi-rover view (one rover only).
- Offline tile cache.
- Path editing on the map.

## Acceptance criteria

- [ ] In Plot frame mode, behaviour matches current map minus the
      now-replaced gaps.
- [ ] In Geo mode, moving the rover ~1 m makes the world scroll ~1 m worth
      of pixels (verify against the scale bar).
- [ ] Trace draws a continuous polyline that follows the rover.
- [ ] Heading triangle rotates correctly (test by manually yawing the rover).
- [ ] Path preview from task 04 still works in both modes.
- [ ] Switching modes does not clear the trace.

## Notes for the agent

- For the equirectangular projection, latitude scaling = 111_320 m/deg,
  longitude scaling = `111_320 * cos(lat_rad)` m/deg. Linearise around the
  rover's current lat — fine for the bench area, no need for proper Mercator.
- Ring buffer: keep it as a plain array with a length cap and a `push`
  helper. Don't `slice(1)` per sample (that's quadratic) — overwrite an
  index modulo N.
