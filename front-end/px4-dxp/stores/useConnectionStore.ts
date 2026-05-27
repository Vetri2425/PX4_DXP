// stores/useConnectionStore.ts
import { create } from 'zustand';
import AsyncStorage from '@react-native-async-storage/async-storage';

export interface Rover {
  name: string;
  host: string;
  port: number;
  status: 'connected' | 'standby' | 'offline';
}

interface ConnectionState {
  backendConnected: boolean;
  backendError: string | null;
  discoveredRovers: Rover[];
  discovering: boolean;
  activeRoverUrl: string;

  setBackendConnected: (connected: boolean) => void;
  setBackendError: (error: string | null) => void;
  setDiscovering: (discovering: boolean) => void;
  setDiscoveredRovers: (rovers: Rover[]) => void;
  setBaseUrl: (url: string) => Promise<void>;
  discover: () => Promise<void>;
  /** Hydrate last-used rover URL from AsyncStorage (called once at app boot) */
  hydrate: () => Promise<void>;
}

const DEFAULT_URL = 'http://192.168.1.102:5001';

export const useConnectionStore = create<ConnectionState>((set, get) => ({
  backendConnected: false,
  backendError: null,
  discoveredRovers: [],
  discovering: false,
  activeRoverUrl: DEFAULT_URL,

  setBackendConnected: (connected) => set({ backendConnected: connected }),
  setBackendError: (error) => set({ backendError: error }),
  setDiscovering: (discovering) => set({ discovering }),
  setDiscoveredRovers: (rovers) => set({ discoveredRovers: rovers }),

  setBaseUrl: async (url) => {
    if (!url) return;
    let u = url.trim();
    if (!u.startsWith('http://') && !u.startsWith('https://')) u = 'http://' + u;
    if (u.endsWith('/')) u = u.slice(0, -1);
    await AsyncStorage.setItem('rover_base_url', u);
    set({ activeRoverUrl: u });
  },

  discover: async () => {
    set({ discovering: true });
    try {
      const url = get().activeRoverUrl;
      const token = await AsyncStorage.getItem('rover_token');
      const res = await fetch(`${url}/api/discover`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', ...(token ? { 'X-Rover-Token': token } : {}) },
      });
      if (!res.ok) throw new Error('Discovery failed');
      const data = await res.json();
      set({ discoveredRovers: data.beacons || [], discovering: false });
    } catch {
      set({ discoveredRovers: [], discovering: false });
    }
  },

  hydrate: async () => {
    const saved = await AsyncStorage.getItem('rover_base_url');
    if (saved) {
      // Normalize same way as setBaseUrl
      let u = saved.trim();
      if (!u.startsWith('http://') && !u.startsWith('https://')) u = 'http://' + u;
      if (u.endsWith('/')) u = u.slice(0, -1);
      if (u !== get().activeRoverUrl) {
        set({ activeRoverUrl: u });
      }
    }
  },
}));