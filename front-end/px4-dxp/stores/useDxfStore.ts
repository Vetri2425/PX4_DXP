// stores/useDxfStore.ts
import { create } from 'zustand';
import type { DXFEntityInfo, DXFParseResponse, PathPlanResponse } from '../services/api';

export type { DXFEntityInfo };

interface DxfState {
  // Parsed file from server
  dxfFile: DXFParseResponse | null;
  // Selected entity IDs (null = all selected)
  dxfSelected: Set<string> | null;
  dxfOverrides: Record<string, Record<string, unknown>>;
  dxfOrder: string[];
  dxfInspectorOpen: boolean;
  // Result of last /api/path/plan call
  planResult: PathPlanResponse | null;

  setDxfFile: (file: DXFParseResponse | null) => void;
  setDxfSelected: (selected: Set<string> | null) => void;
  setDxfOverrides: (overrides: Record<string, Record<string, unknown>>) => void;
  setDxfOrder: (order: string[]) => void;
  setDxfInspectorOpen: (open: boolean) => void;
  setPlanResult: (result: PathPlanResponse | null) => void;
  confirmSelection: (selected: Set<string>, overrides: Record<string, Record<string, unknown>>, order: string[]) => void;
  reset: () => void;
}

export const useDxfStore = create<DxfState>((set) => ({
  dxfFile: null,
  dxfSelected: null,
  dxfOverrides: {},
  dxfOrder: [],
  dxfInspectorOpen: false,
  planResult: null,

  setDxfFile: (file) => set({ dxfFile: file, dxfSelected: null, dxfOrder: [], dxfOverrides: {}, planResult: null }),
  setDxfSelected: (selected) => set({ dxfSelected: selected }),
  setDxfOverrides: (overrides) => set({ dxfOverrides: overrides }),
  setDxfOrder: (order) => set({ dxfOrder: order }),
  setDxfInspectorOpen: (open) => set({ dxfInspectorOpen: open }),
  setPlanResult: (result) => set({ planResult: result }),
  confirmSelection: (selected, overrides, order) =>
    set({ dxfSelected: selected, dxfOverrides: overrides, dxfOrder: order, dxfInspectorOpen: false }),
  reset: () => set({ dxfFile: null, dxfSelected: null, dxfOverrides: {}, dxfOrder: [], dxfInspectorOpen: false, planResult: null }),
}));