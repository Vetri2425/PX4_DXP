// stores/useUiStore.ts
import { create } from 'zustand';

type TabId = 'home' | 'map' | 'draw' | 'drive' | 'more';
type SubScreen = 'connect' | 'camera' | 'ros' | 'px4' | 'calibrate' | 'logs' | 'firmware' | 'fleet' | 'settings';

interface UiState {
  tab: TabId;
  stack: SubScreen[];
  armed: boolean;
  emergency: boolean;

  setTab: (tab: TabId) => void;
  push: (screen: SubScreen) => void;
  pop: () => void;
  setArmed: (armed: boolean) => void;
  triggerEStop: () => void;
  clearEStop: () => void;
}

export const useUiStore = create<UiState>((set) => ({
  tab: 'home',
  stack: [],
  armed: false,
  emergency: false,

  setTab: (tab) => set({ tab, stack: [] }),
  push: (screen) => set((prev) => ({ stack: [...prev.stack, screen] })),
  pop: () => set((prev) => ({ stack: prev.stack.slice(0, -1) })),
  setArmed: (armed) => set({ armed }),
  triggerEStop: () => set({ emergency: true, armed: false }),
  clearEStop: () => set({ emergency: false }),
}));