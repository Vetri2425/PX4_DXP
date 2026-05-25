// hooks/useRover.ts
import { useConnectionStore } from '../stores/useConnectionStore';
import { useTelemetryStore } from '../stores/useTelemetryStore';
import { useMissionStore } from '../stores/useMissionStore';
import { useDxfStore } from '../stores/useDxfStore';
import { useUiStore } from '../stores/useUiStore';

/**
 * Convenience hook that composes all Zustand stores.
 * Matches the web prototype's useRover() API for easy migration.
 *
 * IMPORTANT: For performance, prefer individual store hooks in components
 * that only need one slice. Use useRover() only in components that
 * genuinely need cross-store data (like the Dashboard).
 */
export function useRover() {
  const conn = useConnectionStore();
  const telemetry = useTelemetryStore();
  const mission = useMissionStore();
  const dxf = useDxfStore();
  const ui = useUiStore();

  return {
    // Connection
    backendConnected: conn.backendConnected,
    backendError: conn.backendError,
    discoveredRovers: conn.discoveredRovers,
    discovering: conn.discovering,
    activeRoverUrl: conn.activeRoverUrl,
    setBaseUrl: conn.setBaseUrl,
    discover: conn.discover,
    switchRover: conn.setBaseUrl,

    // Telemetry (alias 't' like web prototype)
    t: {
      battery: telemetry.battery,
      voltage: telemetry.voltage,
      current: telemetry.current,
      temp: telemetry.temp,
      sats: telemetry.sats,
      hdop: telemetry.hdop,
      fix: telemetry.fix,
      rssi: telemetry.rssi,
      speed: telemetry.speed,
      heading: telemetry.heading,
      alt: telemetry.alt,
      roll: telemetry.roll,
      pitch: telemetry.pitch,
      yaw: telemetry.yaw,
      motor: telemetry.motor,
      history: telemetry.history,
    },

    // Mission
    waypoints: mission.waypoints,
    activeJob: mission.activeJob,
    drawProgress: mission.drawProgress,
    setWaypoints: mission.setWaypoints,
    setActiveJob: mission.setActiveJob,
    setDrawProgress: mission.setDrawProgress,
    missionMode: mission.missionMode,
    setMissionMode: mission.setMissionMode,

    // DXF
    dxfFile: dxf.dxfFile,
    dxfSelected: dxf.dxfSelected,
    dxfOverrides: dxf.dxfOverrides,
    dxfOrder: dxf.dxfOrder,
    dxfInspectorOpen: dxf.dxfInspectorOpen,
    setDxfFile: dxf.setDxfFile,
    setDxfSelected: dxf.setDxfSelected,
    setDxfOverrides: dxf.setDxfOverrides,
    setDxfOrder: dxf.setDxfOrder,
    setDxfInspectorOpen: dxf.setDxfInspectorOpen,

    // UI
    tab: ui.tab,
    setTab: ui.setTab,
    stack: ui.stack,
    push: ui.push,
    pop: ui.pop,
    armed: ui.armed,
    emergency: ui.emergency,
    setArmed: ui.setArmed,
    triggerEStop: ui.triggerEStop,
    clearEStop: ui.clearEStop,
  };
}