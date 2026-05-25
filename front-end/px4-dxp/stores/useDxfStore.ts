// stores/useDxfStore.ts
import { create } from 'zustand';

interface DxfEntity {
  id: string;
  type: string;
  layer: string;
  color: string;
  length: number;
  closed: boolean;
  [key: string]: unknown;
}

interface DxfFile {
  name: string;
  size: string;
  entities: DxfEntity[];
  bounds: { w: number; h: number };
  tint?: string;
}

interface DxfState {
  dxfFile: DxfFile | null;
  dxfSelected: Set<string> | null;
  dxfOverrides: Record<string, unknown>;
  dxfOrder: string[];
  dxfInspectorOpen: boolean;

  setDxfFile: (file: DxfFile | null) => void;
  setDxfSelected: (selected: Set<string> | null) => void;
  setDxfOverrides: (overrides: Record<string, unknown>) => void;
  setDxfOrder: (order: string[]) => void;
  setDxfInspectorOpen: (open: boolean) => void;
  confirmSelection: (selected: Set<string>, overrides: Record<string, unknown>, order: string[]) => void;
}

export const useDxfStore = create<DxfState>((set) => ({
  dxfFile: null,
  dxfSelected: null,
  dxfOverrides: {},
  dxfOrder: [],
  dxfInspectorOpen: false,

  setDxfFile: (file) => set({ dxfFile: file }),
  setDxfSelected: (selected) => set({ dxfSelected: selected }),
  setDxfOverrides: (overrides) => set({ dxfOverrides: overrides }),
  setDxfOrder: (order) => set({ dxfOrder: order }),
  setDxfInspectorOpen: (open) => set({ dxfInspectorOpen: open }),
  confirmSelection: (selected, overrides, order) =>
    set({ dxfSelected: selected, dxfOverrides: overrides, dxfOrder: order, dxfInspectorOpen: false }),
}));