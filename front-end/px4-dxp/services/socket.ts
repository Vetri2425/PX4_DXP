// services/socket.ts
import { io, Socket } from 'socket.io-client';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { useConnectionStore } from '../stores/useConnectionStore';
import { useTelemetryStore, mapPx4Mode } from '../stores/useTelemetryStore';
import { useUiStore } from '../stores/useUiStore';
import { useMissionStore } from '../stores/useMissionStore';
import type { MissionMode } from '../types/mission';

let socket: Socket | null = null;
/** Track which URL the current socket was built for (#4) */
let socketUrl = '';

const VALID_MODES = new Set<MissionMode>(['Manual', 'Hold', 'Draw', 'Mission']);

function toMissionMode(raw: string): MissionMode {
  // #7 — map raw PX4 string through the explicit table, never cast blindly
  const mapped = mapPx4Mode(raw);
  return VALID_MODES.has(mapped as MissionMode)
    ? (mapped as MissionMode)
    : 'Manual'; // safe fallback
}

export async function initSocket(): Promise<Socket> {
  const url = useConnectionStore.getState().activeRoverUrl;

  // #4 — tear down the cached socket when the URL changes
  if (socket && socketUrl !== url) {
    socket.removeAllListeners();
    socket.disconnect();
    socket = null;
    socketUrl = '';
  }

  if (socket?.connected) return socket;

  const token = await AsyncStorage.getItem('rover_token');

  socket = io(url, {
    transports: ['websocket', 'polling'],
    auth: { token: token || '' },
    reconnection: true,
    reconnectionDelay: 1000,
    reconnectionDelayMax: 30000, // #13 — cap back-off
    reconnectionAttempts: Infinity, // #13 — keep trying indefinitely
  });
  socketUrl = url;

  socket.on('connect', () => {
    useConnectionStore.getState().setBackendConnected(true);
    useConnectionStore.getState().setBackendError(null);
    useUiStore.getState().appendLog('INFO', `Socket connected to ${url}`);
  });

  socket.on('disconnect', (reason) => {
    useConnectionStore.getState().setBackendConnected(false);
    useUiStore.getState().appendLog('WARN', `Socket disconnected: ${reason}`);
  });

  socket.on('connect_error', (err: Error) => {
    useConnectionStore.getState().setBackendError(err.message || 'Connection error');
    useUiStore.getState().appendLog('ERR', `Socket error: ${err.message}`);
  });

  socket.on('telemetry', (data: Record<string, unknown>) => {
    // #2 — typed through the store's own signature; no `as any`
    type TelSocketPayload = Parameters<ReturnType<typeof useTelemetryStore.getState>['updateFromSocket']>[0];
    useTelemetryStore.getState().updateFromSocket(data as TelSocketPayload);

    const raw = data as { armed?: boolean; mode?: string };
    if (raw.armed != null) useUiStore.getState().setArmed(Boolean(raw.armed));
    if (typeof raw.mode === 'string') {
      useMissionStore.getState().setMissionMode(toMissionMode(raw.mode));
    }
  });

  socket.on('mission_status', (data: { dist_to_goal?: number; total_distance?: number }) => {
    const job = useMissionStore.getState().activeJob;
    if (job && data.dist_to_goal != null) {
      // #15 — use real total_distance when available; fall back to job.paths as metres
      const total = data.total_distance ?? (job.paths > 0 ? job.paths : 20);
      useMissionStore.getState().setActiveJob({
        ...job,
        progress: Math.max(0, Math.min(1, 1 - data.dist_to_goal / total)),
      });
    }
  });

  socket.on('mission_completed', () => {
    useMissionStore.getState().setDrawProgress(1);
    useMissionStore.getState().setMissionMode('Hold');
    useUiStore.getState().setArmed(false);
    useUiStore.getState().appendLog('INFO', 'Mission completed');
  });

  socket.on('safety_abort', (data: { reason?: string }) => {
    useUiStore.getState().triggerEStop();
    useMissionStore.getState().setMissionMode('Hold');
    const msg = `SAFETY ABORT: ${data.reason ?? 'Unknown reason'}`;
    useConnectionStore.getState().setBackendError(msg);
    useUiStore.getState().appendLog('ERR', msg);
  });

  socket.on('arm_result', (data: { success: boolean; arm: boolean; message?: string }) => {
    if (data.success) {
      useUiStore.getState().setArmed(data.arm);
      useUiStore.getState().appendLog('INFO', `${data.arm ? 'Armed' : 'Disarmed'}`);
    } else {
      const msg = `Arm ${data.arm ? 'arm' : 'disarm'} rejected: ${data.message ?? 'unknown'}`;
      useConnectionStore.getState().setBackendError(msg);
      useUiStore.getState().appendLog('ERR', msg);
    }
  });

  socket.on('mode_result', (data: { success: boolean; mode: string; message?: string }) => {
    if (data.success) {
      useMissionStore.getState().setMissionMode(toMissionMode(data.mode));
    } else {
      const msg = `Mode '${data.mode}' rejected: ${data.message ?? 'unknown'}`;
      useConnectionStore.getState().setBackendError(msg);
      useUiStore.getState().appendLog('ERR', msg);
    }
  });

  // Server emits this when FCU heartbeat transitions connected → disconnected
  // (see server/main.py:317-320). The Socket.IO session is still alive, so
  // 'disconnect' won't fire — this is the only signal the autopilot dropped.
  socket.on('rover_disconnected', () => {
    const msg = 'FCU disconnected from autopilot';
    useConnectionStore.getState().setBackendError(msg);
    useUiStore.getState().appendLog('ERR', msg);
  });

  socket.on('socket_error', (data: { reason?: string }) => {
    const msg = data.reason ?? 'Socket error';
    useConnectionStore.getState().setBackendError(msg);
    useUiStore.getState().appendLog('ERR', msg);
  });

  return socket;
}

export function disconnectSocket(): void {
  if (socket) {
    socket.removeAllListeners();
    socket.disconnect();
    socket = null;
    socketUrl = '';
  }
}

export function getSocket(): Socket | null {
  return socket;
}

/** #13 — expose a manual reconnect action (used by ConnectionBadge) */
export function reconnectSocket(): void {
  if (!socket) return;
  if (!socket.connected) {
    socket.connect();
  }
}
