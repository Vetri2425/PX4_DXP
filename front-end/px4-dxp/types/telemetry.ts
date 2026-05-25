// types/telemetry.ts
// Field optionality reflects the actual server contract (server/main.py:236-259).
// Fields the server does NOT currently emit are marked `?` so the UI can
// render "—" rather than stale mock values.
export interface TelemetryData {
  // Emitted by server today:
  battery_pct?: number;
  battery_v?: number;
  gps_sat?: number;
  gps_fix?: number;
  speed_m_s?: number;
  heading_ned_deg?: number;
  alt?: number;
  lat?: number;
  lon?: number;
  pos_n?: number;
  pos_e?: number;
  xtrack_m?: number;
  heading_err_deg?: number;
  lookahead_m?: number;
  kappa?: number;
  dist_to_goal_m?: number;
  pose_age_ms?: number;
  rpp_state?: number;
  rpp_state_name?: string;

  // Not emitted yet — backend extension required. UI must tolerate missing.
  current?: number;
  temp?: number;
  hdop?: number;
  rssi?: number;
  roll?: number;
  pitch?: number;
  yaw?: number;
  motor?: [number, number, number, number] | number[];
}

export interface TelemetryHistory {
  v: number[];
  a: number[];
  cpu: number[];
}

export type GpsFixLabel = 'NO_FIX' | '2D' | '3D' | '3D_DGPS' | 'RTK_FLOAT' | 'RTK_FIXED' | 'STATIC';

// MAVROS GPS_FIX_TYPE values 0..8 (mavros_msgs/GPSRAW). Newer MAVROS reports
// RTK_FIXED as 6 (not 5). Partial<Record<...>> reflects that not every int
// has a label; callers must default to NO_FIX.
export const GPS_FIX_LABELS: Partial<Record<number, GpsFixLabel>> = {
  0: 'NO_FIX',
  1: 'NO_FIX',
  2: '2D',
  3: '3D',
  4: '3D_DGPS',
  5: 'RTK_FLOAT',
  6: 'RTK_FIXED',
  7: 'STATIC',
  8: 'STATIC',
};

export function gpsFixLabel(n: number | undefined): GpsFixLabel {
  if (n == null) return 'NO_FIX';
  return GPS_FIX_LABELS[n] ?? 'NO_FIX';
}