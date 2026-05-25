# PX.4_DXp React Native App — Design Specification

**Date:** 2026-05-25
**Status:** Approved
**Approach:** Clean-room React Native rewrite (Approach A)

---

## 1. App Identity

| Property | Value |
|---|---|
| Display Name | PX.4_DXp |
| Expo Slug | px4-dxp |
| Android Package | com.vetri.px4dxp |
| Orientation | Landscape (primary), Portrait (secondary) |
| Min SDK | 24 (Android 7.0) |
| Target SDK | 35 |
| TypeScript | Strict mode |
| Expo SDK | 54 |
| React Native | 0.81 |
| React | 19 |

---

## 2. Technology Stack

| Category | Choice | Rationale |
|---|---|---|
| Framework | Expo SDK 54 (Managed) | Fastest dev iteration, OTA updates, EAS Build |
| Routing | Expo Router v4 (file-based) | Type-safe deep linking, tab/stack compositing |
| State | Zustand | Lightweight, selective re-renders for high-frequency telemetry |
| Gestures | React Native Reanimated 3 + Gesture Handler | Joystick, drag waypoints, attitude indicator |
| Maps | react-native-maps (Google Maps) | Satellite tiles, marker clustering, no Mapbox account needed |
| SVG | react-native-svg | Attitude indicator, compass, waypoint markers |
| Real-time | Socket.IO client (v4) | Same protocol as web prototype, same event names |
| Blur | expo-blur | Backdrop-filter replacement for glass panels |
| Storage | AsyncStorage | Auth token, rover URL persistence |
| Fonts | Geist + Geist Mono (bundled) | Same typography as web prototype |

---

## 3. Directory Structure

```
px4-dxp/
├── app/                          # Expo Router v4 file-based routing
│   ├── _layout.tsx               # Root layout (providers, theme, fonts)
│   ├── (tabs)/                   # Tab group
│   │   ├── _layout.tsx           # Bottom tab navigator
│   │   ├── index.tsx             # Home/Dashboard
│   │   ├── map.tsx               # Mission planner
│   │   ├── draw.tsx              # DXF/Drawing
│   │   ├── drive.tsx             # Manual drive + joysticks
│   │   └── more.tsx              # Settings & tools
│   ├── connect.tsx               # Rover discovery & pairing
│   ├── camera.tsx                # Camera feed (future)
│   └── dxf-inspector.tsx         # DXF entity selector
├── components/
│   ├── ui/                       # Primitives
│   │   ├── Card.tsx
│   │   ├── Btn.tsx
│   │   ├── Pill.tsx
│   │   ├── Dot.tsx
│   │   ├── Bar.tsx
│   │   ├── Stat.tsx
│   │   ├── SliderRow.tsx
│   │   ├── SectionHeader.tsx
│   │   ├── AppBar.tsx
│   │   ├── IconBtn.tsx
│   │   └── ActionChip.tsx
│   ├── dashboard/                # Dashboard composites
│   │   ├── RoverHeroCard.tsx
│   │   ├── QuickActions.tsx
│   │   ├── SysDiagnostics.tsx
│   │   └── FleetCard.tsx
│   ├── map/                      # Mission planner composites
│   │   ├── MapView.tsx           # Google Maps wrapper
│   │   ├── WaypointMarker.tsx
│   │   ├── WpInspector.tsx
│   │   └── TelemetryChip.tsx
│   ├── drive/                    # Manual drive composites
│   │   ├── AttitudeIndicator.tsx
│   │   ├── HeadingDisc.tsx
│   │   ├── Joystick.tsx
│   │   ├── MotorTile.tsx
│   │   ├── MiniStat.tsx
│   │   └── ActionChip.tsx
│   └── dxf/                      # DXF/Drawing composites
│       ├── DxfPanel.tsx
│       ├── DxfPreview.tsx
│       └── EntityList.tsx
├── stores/                        # Zustand stores
│   ├── useConnectionStore.ts
│   ├── useTelemetryStore.ts
│   ├── useMissionStore.ts
│   ├── useDxfStore.ts
│   └── useUiStore.ts
├── hooks/
│   ├── useRover.ts               # Composed convenience hook
│   ├── useJoystick.ts            # Gesture handler for joysticks
│   └── useMapGestures.ts         # Long-press add, drag waypoints
├── services/
│   ├── api.ts                    # REST client (fetch wrapper, auth headers)
│   └── socket.ts                 # Socket.IO client (typed events)
├── theme/
│   ├── colors.ts                 # Dark navy palette
│   ├── typography.ts             # Font loading, scale
│   └── spacing.ts                # Consistent spacing
├── types/
│   ├── telemetry.ts
│   ├── mission.ts
│   └── socket-events.ts
├── core/
│   ├── geo/                      # Distance, bearing, coordinate math
│   └── parsers/                  # DXF, G-code (future)
├── assets/
│   └── fonts/                    # Geist, Geist Mono
├── app.json
├── package.json
└── tsconfig.json
```

---

## 4. State Architecture

### Zustand Stores

Each store is independent. Components subscribe to specific slices via `useStore(state => state.slice)` to avoid unnecessary re-renders.

**useConnectionStore:**
- `backendConnected: boolean`
- `backendError: string | null`
- `discoveredRovers: Rover[]`
- `discovering: boolean`
- `activeRoverUrl: string`
- Actions: `discover()`, `switchRover()`, `setBaseUrl()`

**useTelemetryStore:**
- `battery, voltage, current, temp, sats, hdop, fix, rssi, speed, heading, alt, roll, pitch, yaw, motor[]`
- `history: { v[], a[], cpu[] }` (last 40 readings for sparklines)
- Updated via Socket.IO `telemetry` event at 10Hz
- Actions: none (pure receiver, updated by socket service)

**useMissionStore:**
- `waypoints: Waypoint[]`
- `activeJob: Job | null`
- `drawProgress: number` (0–1)
- `missionMode: 'Manual' | 'Hold' | 'Draw' | 'Mission'`
- Actions: `setWaypoints()`, `startMission()`, `stopMission()`, `abortMission()`

**useDxfStore:**
- `dxfFile, dxfSelected, dxfOverrides, dxfOrder, dxfInspectorOpen`
- Actions: `setDxfFile()`, `setDxfInspectorOpen()`, `confirmSelection()`

**useUiStore:**
- `tab: 'home' | 'map' | 'draw' | 'drive' | 'more'`
- `stack: string[]` (sub-screen navigation)
- `armed: boolean`
- `emergency: boolean`
- Actions: `setTab()`, `push()`, `pop()`, `setArmed()`, `triggerEStop()`, `clearEStop()`

**useRover() convenience hook** composes all five stores into one object, matching the web prototype's API for easy migration.

---

## 5. Socket.IO Integration

### Connection

```typescript
// services/socket.ts
const socket = io(ROVER_URL, {
  transports: ['websocket', 'polling'],
  auth: { token: AsyncStorage.getItem('rover_token') },
  reconnection: true,
  reconnectionDelay: 1000,
  reconnectionAttempts: 10,
});
```

### Event Map

| Event | Direction | Payload | Store Updated |
|---|---|---|---|
| `telemetry` | Server → Client | `{ battery_pct, battery_v, gps_sat, gps_fix, speed_m_s, heading_ned_deg, alt, connected, armed, mode }` | useTelemetryStore, useUiStore |
| `mission_status` | Server → Client | `{ dist_to_goal }` | useMissionStore (progress) |
| `mission_completed` | Server → Client | `{ name }` | useMissionStore, useUiStore |
| `safety_abort` | Server → Client | `{ reason }` | useUiStore (emergency=true) |
| `arm_result` | Server → Client | `{ success, arm }` | useUiStore |
| `mode_result` | Server → Client | `{ success, mode }` | useUiStore |
| `arm` | Client → Server | `{ arm: bool, auth: token }` | — |
| `set_mode` | Client → Server | `{ mode, auth: token }` | — |
| `mission_start` | Client → Server | `{ auth: token }` | — |
| `mission_stop` | Client → Server | `{ auth: token }` | — |
| `mission_abort` | Client → Server | `{ auth: token }` | — |

### REST Endpoints

Same as web prototype:
- `POST /api/arm` — `{ arm: bool }`
- `POST /api/set_mode` — `{ mode }`
- `POST /api/estop` — `{}`
- `POST /api/mission/load` — `{ path_name }`
- `POST /api/mission/start` — `{ path_name? }`
- `POST /api/mission/stop` — `{}`
- `POST /api/mission/abort` — `{}`
- `GET /api/telemetry/latest`
- `GET /api/mission/status`
- `GET /api/paths`
- `POST /api/discover` — returns `{ beacons: [...] }`

### Auth

Same as web prototype: shared-secret token stored in AsyncStorage, sent as `X-Rover-Token` header on REST calls and `auth.token` on socket connection.

---

## 6. RN-Specific Adaptations

| Web Feature | RN Replacement |
|---|---|
| `window.api` global | Zustand stores + typed service modules |
| `localStorage` | `@react-native-async-storage/async-storage` |
| Inline CSS styles | `StyleSheet.create()` + theme constants from `theme/colors.ts` |
| `backdrop-filter: blur()` | `BlurView` from `expo-blur` |
| `<svg>` inline SVG | `react-native-svg` `Svg`, `Circle`, `Path`, `Line` |
| `pointermove` / `pointerup` | `GestureHandler` + `Reanimated 3` `useAnimatedGestureHandler` |
| `ReactDOM.createRoot` | Expo Router `_layout.tsx` |
| CSS `@keyframes pxpulse` | `withRepeat(withTiming(1, { duration: 1600 }), -1)` Reanimated |
| CSS `var(--mono)` font | Bundled Geist Mono via `@expo-google-fonts/geist` or custom font loading |
| CSS `var(--sans)` font | Bundled Geist via same mechanism |
| `position: absolute` overlays | RN `absoluteFillObject` + `zIndex` |
| `overflow: hidden` with `borderRadius` | RN `overflow: 'hidden'` on `View` (works on iOS, limited on Android — use `MaskedView` for clipped circles) |
| `window.addEventListener('pointermove')` | `Gesture.Pan()` with `onUpdate` callback |

---

## 7. Theme System

```typescript
// theme/colors.ts
export const C = {
  bg: '#0a0d12',
  bg2: '#0e1219',
  card: '#141923',
  card2: '#1a2030',
  line: 'rgba(255,255,255,0.07)',
  line2: 'rgba(255,255,255,0.12)',
  text: '#e6edf6',
  text2: '#a3adbf',
  text3: '#6b7585',
  accent: '#22d3ee',
  accent2: '#5eead4',
  warn: '#fbbf24',
  danger: '#fb7185',
  good: '#34d399',
  violet: '#a78bfa',
} as const;
```

Same palette as the web prototype. No changes — the dark navy "control room" theme carries over directly.

---

## 8. Safety-Critical Rules

These are non-negotiable because they affect real hardware:

1. **Never fake success** for arm/disarm, mode switch, emergency stop, mission start/stop. If the backend didn't confirm, the UI must show it didn't happen.
2. **Emergency stop** must always be accessible — never hidden behind loading states or disabled without explicit reason.
3. **Joystick commands** use throttle refs at 20 Hz — never add React state updates to the joystick movement path.
4. **Manual control** (joystick) emits at 20 Hz using `Reanimated` shared values. No React state in the hot path.
5. **GPS failsafe** has strict/relax/disable modes — keep acknowledge/resume/restart flows intact.
6. **Mission waypoint states** never downgrade: `completed` and `skipped` are terminal.

---

## 9. Phased Delivery Plan

### Phase 1: Navigation Shell + Theme + Connection
- Expo project init (SDK 54, TypeScript strict)
- Expo Router v4 tab layout
- Theme system (colors, typography, spacing)
- Font loading (Geist, Geist Mono)
- Connection screen (rover discovery, manual URL, token auth)
- Socket.IO service module
- REST API service module

### Phase 2: Dashboard Screen
- Home screen with rover hero card
- Quick-stats strip (BAT, SAT, RSSI, HZ)
- Active job card with progress bar
- Connection badge (live/offline/error)
- Emergency stop overlay
- Quick action grid

### Phase 3: Telemetry WebSocket
- Live telemetry at 10Hz
- Mission status updates
- Arm/mode result handling
- Safety abort handling
- Sparkline history (voltage, current, CPU)

### Phase 4: Mission Planner Map
- Google Maps with satellite tiles
- Waypoint markers with drag-to-edit
- Long-press to add waypoints
- Waypoint inspector panel
- Mission upload/start/stop
- Telemetry overlay chips (GPS, heading, speed)
- Path rendering between waypoints

### Phase 5: Manual Drive Screen
- Dual joystick (GestureHandler + Reanimated 3)
- Attitude indicator (pitch/roll SVG)
- Heading compass disc (SVG)
- Motor monitor grid
- E-stop button (always accessible, red gradient)
- Arm/disarm toggle
- Mode switch chips (Hold, Draw, Manual)

### Phase 6: Draw/DXF Screen
- Tab strip (DXF, Gallery, SVG, Draw, G-code)
- DXF file import and entity selection
- Drawing canvas (GestureHandler for freehand strokes)
- G-code viewer with syntax highlighting
- Gallery grid with SVG previews
- Scale/feed/pen settings

### Phase 7: More + Sub-screens
- More screen (index of tools)
- Settings screen
- ROS nodes viewer
- PX4 parameters editor
- Logs viewer
- Calibration flow
- Fleet management
- Firmware updates

### Phase 8: Polish & Native Integrations
- Camera feed (expo-camera)
- USB serial / Bluetooth (future — requires prebuild)
- Push notifications for safety events
- Landscape-optimized layouts
- Performance optimization (memo, Reanimated worklets)
- APK build via EAS Build

---

## 10. Key Packages

```json
{
  "dependencies": {
    "expo": "~54.0.0",
    "expo-router": "~4.0.0",
    "expo-blur": "~14.0.0",
    "expo-font": "~13.0.0",
    "expo-status-bar": "~2.0.0",
    "expo-constants": "~17.0.0",
    "react-native": "0.81.0",
    "react": "19.0.0",
    "react-native-screens": "~4.0.0",
    "react-native-safe-area-context": "~5.0.0",
    "react-native-svg": "~15.0.0",
    "react-native-maps": "~1.20.0",
    "react-native-reanimated": "~3.16.0",
    "react-native-gesture-handler": "~2.20.0",
    "@react-native-async-storage/async-storage": "~2.0.0",
    "zustand": "~5.0.0",
    "socket.io-client": "~4.8.0"
  }
}
```

(Exact versions will be pinned during `npx create-expo-app` and `npx expo install`)

**Note:** `react-native-maps` with Google Maps provider requires a Google Maps API key configured in `app.json` under `android.config.googleMaps.apiKey`. This key must be obtained before Phase 4.

---

## 11. Backend Boundary

The RN app connects to the **existing FastAPI backend** at `http://<rover-ip>:5001`. No backend changes required. The same REST endpoints and Socket.IO events used by the web prototype are reused verbatim.

If new events or endpoints are needed (e.g., joystick velocity commands), they will be flagged as `backend_requirement` blocks and handed off to the backend team (Jetson Claude).

---

## 12. What Is NOT In Scope

- **USB serial / Bluetooth Classic** — requires Expo prebuild and native modules; deferred to Phase 8+
- **Camera feed** — requires expo-camera; Phase 8
- **Offline mode** — app requires rover backend connection
- **iOS build** — Android-only for now (landscape tablet)
- **DXF parser** — will be ported from `lib/dxf-data.jsx` but complex parsing deferred
- **Fleet management** — multi-rover UI deferred to Phase 7