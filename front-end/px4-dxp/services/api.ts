// services/api.ts
import AsyncStorage from '@react-native-async-storage/async-storage';

let _baseUrl = 'http://192.168.1.102:5001';
let _token = '';

export async function initApi() {
  const savedUrl = await AsyncStorage.getItem('rover_base_url');
  if (savedUrl) _baseUrl = savedUrl;
  const savedToken = await AsyncStorage.getItem('rover_token');
  if (savedToken) _token = savedToken;
}

export function getBaseUrl(): string {
  return _baseUrl;
}

export async function setBaseUrl(url: string): Promise<void> {
  if (!url) return;
  let u = url.trim();
  if (!u.startsWith('http://') && !u.startsWith('https://')) u = 'http://' + u;
  if (u.endsWith('/')) u = u.slice(0, -1);
  if (u === _baseUrl) return;
  _baseUrl = u;
  await AsyncStorage.setItem('rover_base_url', u);
}

/** #18 — async, awaited, errors surfaced */
export async function setToken(token: string): Promise<void> {
  _token = token;
  await AsyncStorage.setItem('rover_token', token);
}

export function getToken(): string {
  return _token;
}

function headers(): Record<string, string> {
  const h: Record<string, string> = { 'Content-Type': 'application/json' };
  if (_token) h['X-Rover-Token'] = _token;
  return h;
}

/** #8 — AbortSignal.timeout() on every fetch. Default 5 s, estop gets 1.5 s. */
async function post(
  path: string,
  body: Record<string, unknown> = {},
  timeoutMs = 5000
): Promise<unknown> {
  const res = await fetch(`${_baseUrl}${path}`, {
    method: 'POST',
    headers: headers(),
    body: JSON.stringify(body),
    signal: AbortSignal.timeout(timeoutMs),
  });
  if (!res.ok) {
    const text = await res.text();
    throw new Error(text);
  }
  return res.json();
}

async function get(path: string, timeoutMs = 5000): Promise<unknown> {
  const res = await fetch(`${_baseUrl}${path}`, {
    headers: headers(),
    signal: AbortSignal.timeout(timeoutMs),
  });
  if (!res.ok) {
    const text = await res.text();
    throw new Error(text);
  }
  return res.json();
}

export const api = {
  // Connection
  discover: () => post('/api/discover') as Promise<{ beacons: unknown[] }>,

  // Vehicle control
  arm: (arm: boolean) => post('/api/arm', { arm }),
  disarm: () => post('/api/arm', { arm: false }),
  setMode: (mode: string) => post('/api/set_mode', { mode }),
  /** #8 — 1.5 s timeout for safety-critical estop */
  estop: () => post('/api/estop', {}, 1500),

  // Mission control
  loadMission: (name: string) => post('/api/mission/load', { path_name: name }),
  startMission: (name?: string) => post('/api/mission/start', name ? { path_name: name } : {}),
  stopMission: () => post('/api/mission/stop', {}),
  abortMission: () => post('/api/mission/abort', {}),

  // Status
  getTelemetry: () => get('/api/telemetry/latest'),
  getMissionStatus: () => get('/api/mission/status'),
  getPaths: () => get('/api/paths'),
};
