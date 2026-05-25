// types/mission.ts
export type WaypointType = 'start' | 'pen-down' | 'pen-up' | 'turn' | 'end';

export interface Waypoint {
  id: string;
  latitude: number;
  longitude: number;
  type: WaypointType;
}

export type MissionMode = 'Manual' | 'Hold' | 'Draw' | 'Mission';

export interface Job {
  id: string;
  name: string;
  progress: number;
  eta: string;
  paths: number;
  done: number;
}