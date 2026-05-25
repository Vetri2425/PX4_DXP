// app/(tabs)/draw.tsx
import React, { useState } from 'react';
import { View, Text, Pressable, ScrollView, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { C } from '../../theme/colors';
import { AppBar } from '../../components/ui/AppBar';
import { IconBtn } from '../../components/ui/IconBtn';
import { Icons } from '../../components/icons';
import { DxfPanel } from '../../components/dxf/DxfPanel';
import { DrawCanvas } from '../../components/dxf/DrawCanvas';

type DrawTab = 'dxf' | 'gallery' | 'upload' | 'canvas' | 'gcode';

const TAB_ITEMS: { k: DrawTab; l: string; icon: keyof typeof Icons }[] = [
  { k: 'dxf', l: 'DXF', icon: 'layers' },
  { k: 'gallery', l: 'Gallery', icon: 'grid' },
  { k: 'upload', l: 'SVG', icon: 'upload' },
  { k: 'canvas', l: 'Draw', icon: 'draw' },
  { k: 'gcode', l: 'G-code', icon: 'terminal' },
];

const GCODE_SAMPLE = `; mountain ridge
G21 ; mm
G90 ; absolute
G0 X0 Y0 Z5
M3 ; pen down
G1 X12.4 Y8.6 F2400
G1 X28.1 Y14.2
G1 X42.8 Y19.0
G1 X51.3 Y22.7
G1 X68.9 Y17.4
G1 X84.5 Y12.1
M5 ; pen up
G0 X0 Y0
M30`;

function GcodeLineColor(line: string): string {
  const cmd = line.split(' ')[0];
  if (line.startsWith(';')) return C.text3;
  if (cmd.startsWith('G0')) return '#a78bfa';
  if (cmd.startsWith('G1')) return C.accent;
  if (cmd.startsWith('M')) return C.warn;
  return C.text2;
}

export default function DrawScreen() {
  const [activeTab, setActiveTab] = useState<DrawTab>('dxf');

  return (
    <SafeAreaView style={styles.safeArea} edges={['top']}>
      <AppBar
        title="New Drawing"
        subtitle="Upload SVG · draw · import G-code"
        trailing={<IconBtn icon={<Icons.refresh size={18} color={C.text2} />} />}
      />

      {/* Tab strip */}
      <View style={styles.tabStrip}>
        <View style={styles.tabContainer}>
          {TAB_ITEMS.map((t) => {
            const Ic = Icons[t.icon];
            const active = activeTab === t.k;
            return (
              <Pressable
                key={t.k}
                onPress={() => setActiveTab(t.k)}
                style={[styles.tabBtn, active && styles.tabBtnActive]}
              >
                <Ic size={13} color={active ? C.accent : C.text2} />
                <Text style={[styles.tabLabel, active && styles.tabLabelActive]}>
                  {t.l}
                </Text>
              </Pressable>
            );
          })}
        </View>
      </View>

      <ScrollView
        style={styles.scroll}
        contentContainerStyle={styles.content}
        showsVerticalScrollIndicator={false}
      >
        {activeTab === 'dxf' && <DxfPanel />}

        {activeTab === 'gallery' && (
          <View style={styles.placeholder}>
            <Icons.grid size={40} color={C.text3} />
            <Text style={styles.placeholderText}>Gallery</Text>
            <Text style={styles.placeholderSub}>SVG files from server</Text>
          </View>
        )}

        {activeTab === 'upload' && (
          <View style={[styles.placeholder, styles.uploadArea]}>
            <View style={styles.uploadIcon}>
              <Icons.upload size={26} color={C.accent} />
            </View>
            <Text style={styles.uploadTitle}>Drop SVG, PDF, or DXF</Text>
            <Text style={styles.uploadSub}>or pick from Files · iCloud · Dropbox</Text>
          </View>
        )}

        {activeTab === 'canvas' && <DrawCanvas />}

        {activeTab === 'gcode' && (
          <View style={styles.gcodeOuter}>
            <View style={styles.gcodeCard}>
              <View style={styles.gcodeHeader}>
                <Text style={styles.gcodeHeaderLabel}>mountain.gcode · 142 lines · 18.4 m</Text>
                <View style={styles.gcodeHeaderBtns}>
                  <IconBtn size={28} icon={<Icons.copy size={13} color={C.text2} />} />
                  <IconBtn size={28} icon={<Icons.download size={13} color={C.text2} />} />
                </View>
              </View>
              <ScrollView style={styles.gcodeScroll} nestedScrollEnabled>
                {GCODE_SAMPLE.split('\n').map((line, i) => (
                  <View key={i} style={styles.gcodeLine}>
                    <Text style={styles.gcodeLineNum}>{String(i + 1).padStart(3, ' ')}</Text>
                    <Text style={[styles.gcodeLineText, { color: GcodeLineColor(line) }]}>{line}</Text>
                  </View>
                ))}
              </ScrollView>
            </View>
          </View>
        )}
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: C.bg },
  tabStrip: { paddingHorizontal: 16, paddingBottom: 12 },
  tabContainer: {
    flexDirection: 'row',
    gap: 6,
    backgroundColor: C.card,
    padding: 4,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: C.line,
  },
  tabBtn: {
    flex: 1,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    gap: 4,
    paddingVertical: 8,
    paddingHorizontal: 4,
    borderRadius: 9,
  },
  tabBtnActive: { backgroundColor: `${C.accent}22` },
  tabLabel: { fontSize: 11, fontWeight: '600', color: C.text2, letterSpacing: 0.3 },
  tabLabelActive: { color: C.accent },
  scroll: { flex: 1 },
  content: { paddingBottom: 100 },
  placeholder: { alignItems: 'center', justifyContent: 'center', padding: 40, gap: 8 },
  placeholderText: { fontSize: 16, fontWeight: '600', color: C.text },
  placeholderSub: { fontSize: 12, color: C.text3 },
  uploadArea: {
    marginHorizontal: 16,
    borderRadius: 18,
    borderWidth: 1,
    borderStyle: 'dashed',
    borderColor: C.line2,
    backgroundColor: C.card,
  },
  uploadIcon: {
    width: 56,
    height: 56,
    borderRadius: 28,
    backgroundColor: `${C.accent}1a`,
    alignItems: 'center',
    justifyContent: 'center',
  },
  uploadTitle: { fontSize: 15, fontWeight: '600', color: C.text },
  uploadSub: { fontSize: 12, color: C.text3 },
  gcodeOuter: { paddingHorizontal: 16 },
  gcodeCard: {
    borderRadius: 14,
    overflow: 'hidden',
    borderWidth: 1,
    borderColor: C.line,
  },
  gcodeHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: 10,
    paddingHorizontal: 14,
    borderBottomWidth: 1,
    borderBottomColor: C.line,
    backgroundColor: C.card,
  },
  gcodeHeaderLabel: {
    fontSize: 11,
    color: C.text3,
    textTransform: 'uppercase',
    letterSpacing: 0.7,
    fontWeight: '600',
  },
  gcodeHeaderBtns: { flexDirection: 'row', gap: 6 },
  gcodeScroll: { height: 240, backgroundColor: '#0a0d12' },
  gcodeLine: { flexDirection: 'row', paddingHorizontal: 14, paddingVertical: 1 },
  gcodeLineNum: { fontSize: 11, color: C.text3, width: 32, flexShrink: 0 },
  gcodeLineText: { fontSize: 11 },
});
