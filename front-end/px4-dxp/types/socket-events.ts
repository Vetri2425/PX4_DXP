// types/socket-events.ts
import type { TelemetryData } from './telemetry';

export interface ServerToClientEvents {
  telemetry: (data: TelemetryPayload) => void;
  mission_status: (data: MissionStatusPayload) => void;
  mission_completed: (data: MissionCompletedPayload) => void;
  safety_abort: (data: SafetyAbortPayload) => void;
  arm_result: (data: ArmResultPayload) => void;
  mode_result: (data: ModeResultPayload) => void;
  rover_disconnected: () => void;
}

export interface ClientToServerEvents {
  arm: (data: { arm: boolean; auth: string }) => void;
  set_mode: (data: { mode: string; auth: string }) => void;
  mission_start: (data: { auth: string }) => void;
  mission_stop: (data: { auth: string }) => void;
  mission_abort: (data: { auth: string }) => void;
}

export interface TelemetryPayload extends TelemetryData {
  connected?: boolean;
  armed?: boolean;
  mode?: string;
}

export interface MissionStatusPayload {
  dist_to_goal?: number;
}

export interface MissionCompletedPayload {
  name: string;
}

export interface SafetyAbortPayload {
  reason: string;
}

export interface ArmResultPayload {
  success: boolean;
  arm: boolean;
}

export interface ModeResultPayload {
  success: boolean;
  mode: string;
}