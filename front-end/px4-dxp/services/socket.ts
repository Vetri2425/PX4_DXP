// services/socket.ts
import { io, Socket } from 'socket.io-client';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { useConnectionStore } from '../stores/useConnectionStore';
import { useTelemetryStore } from '../stores/useTelemetryStore';
import { useUiStore } from '../stores/useUiStore';
import { useMissionStore } from '../stores/useMissionStore';

let socket: Socket | null = null;

export async function initSocket(): Promise<Socket> {
  if (socket?.connected) return socket;

  const url = useConnectionStore.getState().activeRoverUrl;
  const token = await AsyncStorage.getItem('rover_token');

  socket = io(url, {
    transports: ['websocket', 'polling'],
    auth: { token: token || '' },
    reconnection: true,
    reconnectionDelay: 1000,
    reconnectionAttempts: 10,
  });

  socket.on('connect', () => {
    useConnectionStore.getState().setBackendConnected(true);
    useConnectionStore.getState().setBackendError(null);
  });

  socket.on('disconnect', () => {
    useConnectionStore.getState().setBackendConnected(false);
  });

  socket.on('connect_error', (err: Error) => {
    useConnectionStore.getState().setBackendError(err.message || 'Connection error');
  });

  type TelPayload = Record<string, unknown> & {
    armed?: boolean;
    mode?: string;
  };

  socket.on('telemetry', (data: TelPayload) => {
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    useTelemetryStore.getState().updateFromSocket(data as any);
    if (data.armed != null) useUiStore.getState().setArmed(Boolean(data.armed));
    if (data.mode && typeof data.mode === 'string') {
      useMissionStore.getState().setMissionMode(
        data.mode as 'Manual' | 'Hold' | 'Draw' | 'Mission'
      );
    }
  });

  socket.on('mission_status', (data: { dist_to_goal?: number }) => {
    const job = useMissionStore.getState().activeJob;
    if (job && data.dist_to_goal != null) {
      useMissionStore.getState().setActiveJob({
        ...job,
        progress: Math.max(0, Math.min(1, 1 - data.dist_to_goal / 20)),
      });
    }
  });

  socket.on('mission_completed', () => {
    useMissionStore.getState().setDrawProgress(1);
    useMissionStore.getState().setMissionMode('Hold');
    useUiStore.getState().setArmed(false);
  });

  socket.on('safety_abort', (data: { reason?: string }) => {
    useUiStore.getState().triggerEStop();
    useMissionStore.getState().setMissionMode('Hold');
    useConnectionStore.getState().setBackendError(`SAFETY ABORT: ${data.reason || 'Unknown reason'}`);
  });

  socket.on('arm_result', (data: { success: boolean; arm: boolean }) => {
    if (data.success) useUiStore.getState().setArmed(data.arm);
  });

  socket.on('mode_result', (data: { success: boolean; mode: string }) => {
    if (data.success) {
      useMissionStore.getState().setMissionMode(data.mode as 'Manual' | 'Hold' | 'Draw' | 'Mission');
    }
  });

  socket.on('socket_error', (data: { reason?: string }) => {
    useConnectionStore.getState().setBackendError(data.reason || 'Socket error');
  });

  return socket;
}

export function disconnectSocket(): void {
  if (socket) {
    socket.disconnect();
    socket = null;
  }
}

export function getSocket(): Socket | null {
  return socket;
}

export function reconnectSocket(): void {
  if (socket && !socket.connected) {
    socket.connect();
  }
}
