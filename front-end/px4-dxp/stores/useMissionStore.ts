// stores/useMissionStore.ts
import { create } from 'zustand';
import type { Waypoint, Job, MissionMode } from '../types/mission';

interface MissionState {
  waypoints: Waypoint[];
  activeJob: Job | null;
  drawProgress: number;
  missionMode: MissionMode;

  setWaypoints: (wp: Waypoint[] | ((prev: Waypoint[]) => Waypoint[])) => void;
  setActiveJob: (job: Job | null) => void;
  setDrawProgress: (p: number) => void;
  setMissionMode: (mode: MissionMode) => void;
}

export const useMissionStore = create<MissionState>((set) => ({
  waypoints: [],
  activeJob: null,
  drawProgress: 0,
  missionMode: 'Manual',

  setWaypoints: (wp) =>
    set((prev) => ({
      waypoints: typeof wp === 'function' ? wp(prev.waypoints) : wp,
    })),
  setActiveJob: (job) => set({ activeJob: job }),
  setDrawProgress: (p) => set({ drawProgress: p }),
  setMissionMode: (mode) => set({ missionMode: mode }),
}));