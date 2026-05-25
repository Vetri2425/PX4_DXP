// stores/useUiStore.ts
import { create } from 'zustand';

type TabId = 'home' | 'map' | 'draw' | 'drive' | 'more';
type SubScreen = 'connect' | 'camera' | 'ros' | 'px4' | 'calibrate' | 'logs' | 'firmware' | 'fleet' | 'settings';

/** #20 — structured error log entry */
export interface LogEntry {
  ts: number;
  level: 'INFO' | 'WARN' | 'ERR';
  msg: string;
}

interface UiState {
  tab: TabId;
  stack: SubScreen[];
  armed: boolean;
  emergency: boolean;
  /** #20 — error log buffer (max 200 entries, newest last) */
  errorLog: LogEntry[];

  setTab: (tab: TabId) => void;
  push: (screen: SubScreen) => void;
  pop: () => void;
  setArmed: (armed: boolean) => void;
  /** #1 — triggerEStop is now async; callers MUST call api.estop() first */
  triggerEStop: () => void;
  clearEStop: () => void;
  /** #20 — append to error log */
  appendLog: (level: LogEntry['level'], msg: string) => void;
}

export const useUiStore = create<UiState>((set) => ({
  tab: 'home',
  stack: [],
  armed: false,
  emergency: false,
  errorLog: [],

  setTab: (tab) => set({ tab, stack: [] }),
  push: (screen) => set((prev) => ({ stack: [...prev.stack, screen] })),
  pop: () => set((prev) => ({ stack: prev.stack.slice(0, -1) })),
  setArmed: (armed) => set({ armed }),
  triggerEStop: () => set({ emergency: true, armed: false }),
  clearEStop: () => set({ emergency: false }),
  appendLog: (level, msg) =>
    set((prev) => {
      const entry: LogEntry = { ts: Date.now(), level, msg };
      const log = [...prev.errorLog, entry];
      // Keep at most 200 entries
      return { errorLog: log.length > 200 ? log.slice(log.length - 200) : log };
    }),
}));
