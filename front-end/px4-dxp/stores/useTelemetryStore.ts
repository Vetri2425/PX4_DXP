// stores/useTelemetryStore.ts
import { create } from 'zustand';
import { GPS_FIX_LABELS } from '../types/telemetry';
import type { TelemetryData, TelemetryHistory } from '../types/telemetry';

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

  updateFromSocket: (data: Partial<TelemetryData> & { connected?: boolean; armed?: boolean; mode?: string }) => void;
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

  updateFromSocket: (data) =>
    set((prev) => ({
      battery: data.battery_pct ?? prev.battery,
      voltage: data.battery_v ?? prev.voltage,
      sats: data.gps_sat ?? prev.sats,
      fix: data.gps_fix != null ? (GPS_FIX_LABELS[data.gps_fix] ?? prev.fix) : prev.fix,
      speed: data.speed_m_s ?? prev.speed,
      heading: data.heading_ned_deg ?? prev.heading,
      alt: data.alt ?? prev.alt,
      yaw: data.heading_ned_deg ?? prev.yaw,
      history: {
        v: [...prev.history.v.slice(1), data.battery_v ?? prev.history.v[39]],
        a: [...prev.history.a.slice(1), prev.current],
        cpu: [...prev.history.cpu.slice(1), 30 + Math.random() * 4],
      },
    })),
}));