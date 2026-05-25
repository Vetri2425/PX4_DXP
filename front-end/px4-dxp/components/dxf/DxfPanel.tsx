// components/dxf/DxfPanel.tsx
import React from 'react';
import { View, Text, Pressable, StyleSheet, ScrollView } from 'react-native';
import { C } from '../../theme/colors';
import { Card } from '../ui/Card';
import { Btn } from '../ui/Btn';
import { Pill } from '../ui/Pill';
import { Icons } from '../icons';
import { useDxfStore } from '../../stores/useDxfStore';
import { useMissionStore } from '../../stores/useMissionStore';
import { useUiStore } from '../../stores/useUiStore';

const TEMPLATES = [
  { id: 'pitch', icon: '⚽', name: '5-a-side pitch', count: 42 },
  { id: 'court', icon: '🏀', name: 'Basketball court', count: 38 },
  { id: 'road', icon: '🚦', name: 'Road markings', count: 21 },
  { id: 'parking', icon: '🅿️', name: 'Parking lot', count: 55 },
  { id: 'runway', icon: '✈️', name: 'Runway markings', count: 18 },
  { id: 'logo', icon: '🎨', name: 'Logo / brand', count: 80 },
];

export function DxfPanel() {
  const { dxfFile, setDxfFile, dxfSelected, setDxfSelected, setDxfInspectorOpen } = useDxfStore();
  const { setActiveJob, setMissionMode } = useMissionStore();
  const { setTab } = useUiStore();

  if (!dxfFile) {
    // No file — show upload prompt + templates
    return (
      <View style={styles.container}>
        <Card pad={20} style={styles.uploadCard}>
          <View style={styles.uploadInner}>
            <View style={styles.uploadIcon}>
              <Icons.layers size={26} color={C.accent} />
            </View>
            <Text style={styles.uploadTitle}>Import a DXF</Text>
            <Text style={styles.uploadSub}>
              Up to 50 MB · LINE · CIRCLE · ARC · LWPOLYLINE · SPLINE
            </Text>
            <View style={styles.uploadBtns}>
              <Btn variant="primary" size="sm" icon={<Icons.upload size={14} color="#06202a" />}>
                Choose .dxf
              </Btn>
              <Btn variant="secondary" size="sm" icon={<Icons.cam size={14} color={C.text2} />}>
                Scan blueprint
              </Btn>
            </View>
          </View>
        </Card>

        <Text style={styles.templatesLabel}>Templates</Text>
        <View style={styles.templateGrid}>
          {TEMPLATES.map((tpl) => (
            <Pressable
              key={tpl.id}
              style={({ pressed }) => [styles.templateCard, { opacity: pressed ? 0.75 : 1 }]}
            >
              <Text style={styles.templateIcon}>{tpl.icon}</Text>
              <View>
                <Text style={styles.templateName}>{tpl.name}</Text>
                <Text style={styles.templateCount}>{tpl.count} entities</Text>
              </View>
            </Pressable>
          ))}
        </View>
      </View>
    );
  }

  // Has file
  const total = dxfFile.entities.length;
  const sel = dxfSelected ? dxfSelected.size : 0;

  const layerMap: Record<string, number> = {};
  dxfFile.entities.forEach((e) => {
    layerMap[e.layer] = (layerMap[e.layer] ?? 0) + 1;
  });

  return (
    <View style={styles.container}>
      <Card pad={14}>
        <View style={styles.fileRow}>
          {/* Thumbnail placeholder */}
          <View style={styles.thumb}>
            <Icons.layers size={32} color={C.accent} />
          </View>
          <View style={styles.fileMeta}>
            <Text style={styles.fileName}>{dxfFile.name}</Text>
            <Text style={styles.fileInfo}>
              {total} entities · {Object.keys(layerMap).length} layers · {dxfFile.size}
            </Text>
            <View style={styles.layerPills}>
              {Object.entries(layerMap).map(([l, c]) => (
                <Pill key={l} color={C.text2} dim>
                  <Text style={styles.layerPillText}>{l} · {c}</Text>
                </Pill>
              ))}
            </View>
          </View>
        </View>
        <View style={styles.fileActions}>
          <Btn
            variant="primary"
            size="sm"
            icon={<Icons.sliders size={13} color="#06202a" />}
            onPress={() => setDxfInspectorOpen(true)}
            style={styles.editBtn}
          >
            Edit selection ({sel}/{total})
          </Btn>
          <Btn
            variant="secondary"
            size="sm"
            icon={<Icons.trash size={13} color={C.text2} />}
            onPress={() => { setDxfFile(null); setDxfSelected(null); }}
          />
        </View>
      </Card>

      <Text style={styles.templatesLabel}>Run</Text>
      <Card pad={12}>
        <Text style={styles.runInfo}>
          {sel} entit{sel === 1 ? 'y' : 'ies'} selected
        </Text>
        <View style={styles.runBtns}>
          <Btn variant="secondary" style={styles.runBtn} icon={<Icons.target size={14} color={C.text2} />}>
            Dry run
          </Btn>
          <Btn
            variant="primary"
            style={styles.runBtn}
            icon={<Icons.play size={14} color="#06202a" />}
            onPress={() => {
              setActiveJob({
                id: 'dxf',
                name: dxfFile.name,
                progress: 0,
                eta: '—',
                paths: sel,
                done: 0,
              });
              setMissionMode('Draw');
              setTab('home');
            }}
          >
            Send to rover
          </Btn>
        </View>
      </Card>
    </View>
  );
}

const styles = StyleSheet.create({
  container: { paddingHorizontal: 16, paddingBottom: 16 },
  uploadCard: {
    borderStyle: 'dashed',
    borderColor: C.line2,
    marginBottom: 16,
  },
  uploadInner: { alignItems: 'center' },
  uploadIcon: {
    width: 56,
    height: 56,
    borderRadius: 28,
    backgroundColor: `${C.accent}1a`,
    alignItems: 'center',
    justifyContent: 'center',
    marginBottom: 10,
  },
  uploadTitle: { fontSize: 15, fontWeight: '600', color: C.text },
  uploadSub: { fontSize: 12, color: C.text3, marginTop: 4, textAlign: 'center' },
  uploadBtns: { flexDirection: 'row', gap: 8, marginTop: 14 },
  templatesLabel: {
    fontSize: 12,
    color: C.text3,
    fontWeight: '600',
    letterSpacing: 0.8,
    textTransform: 'uppercase',
    marginBottom: 8,
  },
  templateGrid: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  templateCard: {
    width: '47%',
    padding: 14,
    backgroundColor: C.card,
    borderWidth: 1,
    borderColor: C.line,
    borderRadius: 14,
    flexDirection: 'row',
    alignItems: 'center',
    gap: 12,
  },
  templateIcon: { fontSize: 28 },
  templateName: { fontSize: 13, fontWeight: '600', color: C.text },
  templateCount: { fontSize: 11, color: C.text3, marginTop: 2 },
  fileRow: { flexDirection: 'row', gap: 12, marginBottom: 12 },
  thumb: {
    width: 80,
    height: 80,
    borderRadius: 12,
    backgroundColor: '#fafafa',
    alignItems: 'center',
    justifyContent: 'center',
    borderWidth: 1,
    borderColor: C.line2,
    flexShrink: 0,
  },
  fileMeta: { flex: 1 },
  fileName: { fontSize: 14, fontWeight: '600', color: C.text },
  fileInfo: { fontSize: 11, color: C.text3, marginTop: 4 },
  layerPills: { flexDirection: 'row', flexWrap: 'wrap', gap: 4, marginTop: 6 },
  layerPillText: { fontSize: 10, color: C.text2 },
  fileActions: { flexDirection: 'row', gap: 8 },
  editBtn: { flex: 1 },
  runInfo: { fontSize: 12, color: C.text2, marginBottom: 10 },
  runBtns: { flexDirection: 'row', gap: 8 },
  runBtn: { flex: 1, alignSelf: 'auto', justifyContent: 'center', alignItems: 'center' },
});
