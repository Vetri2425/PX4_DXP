// types/telemetry.ts
export interface TelemetryData {
  battery_pct: number;
  battery_v: number;
  current: number;
  temp: number;
  gps_sat: number;
  hdop: number;
  gps_fix: number;
  rssi: number;
  speed_m_s: number;
  heading_ned_deg: number;
  alt: number;
  roll: number;
  pitch: number;
  yaw: number;
  motor: [number, number, number, number];
}

export interface TelemetryHistory {
  v: number[];
  a: number[];
  cpu: number[];
}

export type GpsFixLabel = 'NO_FIX' | '2D' | '3D' | '3D_DGPS' | 'RTK_FLOAT' | 'RTK_FIXED';

export const GPS_FIX_LABELS: Record<number, GpsFixLabel> = {
  0: 'NO_FIX',
  1: '2D',
  2: '3D',
  3: '3D_DGPS',
  4: 'RTK_FLOAT',
  5: 'RTK_FIXED',
};