// stores/useTelemetryStore.ts
import { create } from 'zustand';
import { gpsFixLabel } from '../types/telemetry';
import type { TelemetryData, TelemetryHistory } from '../types/telemetry';

/** #7 — explicit PX4 mode string → MissionMode mapping (no unsafe cast) */
const PX4_MODE_MAP: Record<string, string> = {
  // PX4 mode strings as reported by MAVROS custom_mode / base_mode
  MANUAL: 'Manual',
  STABILIZED: 'Manual',
  ACRO: 'Manual',
  ALTCTL: 'Manual',
  POSCTL: 'Manual',
  HOLD: 'Hold',
  AUTO_HOLD: 'Hold',
  AUTO_LOITER: 'Hold',
  MISSION: 'Mission',
  AUTO_MISSION: 'Mission',
  OFFBOARD: 'Draw',       // RPP pipeline uses OFFBOARD for drawing
  GUIDED: 'Draw',
  DRAW: 'Draw',
  // Fallback: anything unrecognised stays as passed through
};

export function mapPx4Mode(raw: string): string {
  const upper = raw.toUpperCase().replace(/ /g, '_');
  return PX4_MODE_MAP[upper] ?? PX4_MODE_MAP[raw] ?? raw;
}

interface TelemetryState {
  battery: number;
  voltage: number;
  current: number;
  temp: number;
  sats: number;
  hdop: number;
  fix: string;
  rssi: number;
  speed: number;
  heading: number;
  alt: number;
  roll: number;
  pitch: number;
  yaw: number;
  motor: [number, number, number, number];
  history: TelemetryHistory;

  updateFromSocket: (
    data: Partial<TelemetryData> & { connected?: boolean; armed?: boolean; mode?: string }
  ) => void;
}

export const useTelemetryStore = create<TelemetryState>((set) => ({
  battery: 78,
  voltage: 16.4,
  current: 4.2,
  temp: 38,
  sats: 14,
  hdop: 0.7,
  fix: '3D RTK',
  rssi: -54,
  speed: 0.42,
  heading: 124,
  alt: 0.34,
  roll: 0,
  pitch: 0,
  yaw: 124,
  motor: [82, 84, 79, 81],
  history: {
    v: Array.from({ length: 40 }, (_, i) => 16 + Math.sin(i / 4) * 0.3),
    a: Array.from({ length: 40 }, (_, i) => 4 + Math.sin(i / 3) * 0.6),
    cpu: Array.from({ length: 40 }, (_, i) => 30 + Math.sin(i / 5) * 8),
  },

  // #2 — assign every field from the socket payload; no `as any`
  updateFromSocket: (data) =>
    set((prev) => {
      // Motor array: accept up to 4 values
      let motor: [number, number, number, number] = prev.motor;
      if (Array.isArray(data.motor) && data.motor.length >= 2) {
        motor = [
          (data.motor[0] as number) ?? prev.motor[0],
          (data.motor[1] as number) ?? prev.motor[1],
          (data.motor[2] as number) ?? prev.motor[2],
          (data.motor[3] as number) ?? prev.motor[3],
        ];
      }

      return {
        battery: data.battery_pct ?? prev.battery,
        voltage: data.battery_v ?? prev.voltage,
        current: data.current ?? prev.current,
        temp: data.temp ?? prev.temp,
        sats: data.gps_sat ?? prev.sats,
        hdop: data.hdop ?? prev.hdop,
        fix: data.gps_fix != null ? gpsFixLabel(data.gps_fix) : prev.fix,
        rssi: data.rssi ?? prev.rssi,
        speed: data.speed_m_s ?? prev.speed,
        heading: data.heading_ned_deg ?? prev.heading,
        alt: data.alt ?? prev.alt,
        roll: data.roll ?? prev.roll,
        pitch: data.pitch ?? prev.pitch,
        yaw: data.yaw ?? (data.heading_ned_deg ?? prev.yaw),
        motor,
        history: {
          v: [...prev.history.v.slice(1), data.battery_v ?? prev.history.v[39]],
          a: [...prev.history.a.slice(1), data.current ?? prev.current],
          cpu: [...prev.history.cpu.slice(1), 30 + Math.random() * 4],
        },
      };
    }),
}));
