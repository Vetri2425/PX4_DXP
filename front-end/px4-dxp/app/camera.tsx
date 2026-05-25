// app/camera.tsx
import React, { useState } from 'react';
import { View, Text, Pressable, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { router } from 'expo-router';
import Svg, { Rect, Path, Defs, LinearGradient, Stop, Line, Circle } from 'react-native-svg';
import { C } from '../theme/colors';
import { AppBar } from '../components/ui/AppBar';
import { Card } from '../components/ui/Card';
import { Stat } from '../components/ui/Stat';
import { IconBtn } from '../components/ui/IconBtn';
import { Icons } from '../components/icons';
import { Dot } from '../components/ui/Dot';
import { useTelemetryStore } from '../stores/useTelemetryStore';

export default function CameraScreen() {
  const { battery, fix } = useTelemetryStore();
  const [recording, setRecording] = useState(false);
  const [channel, setChannel] = useState<'front' | 'depth' | 'therm'>('front');

  return (
    <SafeAreaView style={styles.safeArea} edges={['top']}>
      <AppBar
        title="Camera"
        subtitle={`1080p · 30 fps · ${recording ? 'REC' : 'preview'}`}
        leading={<IconBtn icon={<Icons.chevL size={18} color={C.text2} />} onPress={() => router.back()} />}
        trailing={<IconBtn icon={<Icons.target size={18} color={C.text2} />} />}
      />

      <View style={styles.viewfinderWrap}>
        {/* Faux video feed */}
        <View style={styles.viewfinder}>
          <Svg
            viewBox="0 0 320 200"
            width="100%"
            height="100%"
            style={StyleSheet.absoluteFill}
            preserveAspectRatio="xMidYMid slice"
          >
            <Defs>
              <LinearGradient id="sky" x1="0" x2="0" y1="0" y2="1">
                <Stop offset="0" stopColor="#2a4060" />
                <Stop offset="1" stopColor="#0b1320" />
              </LinearGradient>
              <LinearGradient id="floor" x1="0" x2="0" y1="0" y2="1">
                <Stop offset="0" stopColor="#1a2218" />
                <Stop offset="1" stopColor="#080a0c" />
              </LinearGradient>
            </Defs>
            <Rect width={320} height={120} fill="url(#sky)" />
            <Rect y={120} width={320} height={80} fill="url(#floor)" />
            <Line x1={0} y1={120} x2={320} y2={120} stroke="rgba(255,255,255,0.1)" strokeWidth={1} />
            <Path d="M40,180 Q160,140 280,170" stroke={C.accent} strokeWidth={2} fill="none" strokeDasharray="3 3" />
            {/* Reticle */}
            <Path d="M150,100 L170,100 M160,90 L160,110" stroke={C.accent} strokeWidth={1.5} strokeLinecap="round" />
            <Rect x={148} y={88} width={24} height={24} fill="none" stroke={C.accent} strokeWidth={1} strokeDasharray="2 2" />
          </Svg>

          {/* HUD overlays */}
          <View style={styles.hudTopLeft}>
            {recording && (
              <View style={styles.recBadge}>
                <Dot color={C.danger} size={6} />
                <Text style={styles.recText}>REC 00:14</Text>
              </View>
            )}
            <Text style={styles.isoText}>ISO 800 · 1/60 · f/2.4</Text>
          </View>
          <Text style={styles.hudTopRight}>BAT {Math.round(battery)}% · {fix}</Text>

          {/* Channel selector + record button */}
          <View style={styles.hudBottom}>
            <View style={styles.channelRow}>
              {(['front', 'depth', 'therm'] as const).map((ch) => (
                <Pressable
                  key={ch}
                  onPress={() => setChannel(ch)}
                  style={[styles.channelBtn, channel === ch && styles.channelBtnActive]}
                >
                  <Text style={[styles.channelText, channel === ch && styles.channelTextActive]}>
                    {ch.toUpperCase()}
                  </Text>
                </Pressable>
              ))}
            </View>
            <Pressable
              onPress={() => setRecording((r) => !r)}
              style={[styles.recBtn, recording && styles.recBtnActive]}
            />
          </View>
        </View>
      </View>

      <View style={styles.statsRow}>
        <Card pad={12} style={styles.statsCard}>
          <View style={styles.statsGrid}>
            <Stat label="FPS" value="28.9" color={C.warn} />
            <Stat label="Latency" value="84" unit="ms" />
            <Stat label="Bitrate" value="3.4" unit="Mb/s" />
          </View>
        </Card>
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: C.bg },
  viewfinderWrap: { paddingHorizontal: 16, flex: 1 },
  viewfinder: {
    borderRadius: 18,
    overflow: 'hidden',
    borderWidth: 1,
    borderColor: C.line2,
    aspectRatio: 16 / 10,
    position: 'relative',
    backgroundColor: '#0a0d12',
  },
  hudTopLeft: {
    position: 'absolute',
    top: 10,
    left: 10,
    gap: 4,
  },
  recBadge: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 6,
    backgroundColor: 'rgba(0,0,0,0.55)',
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 9999,
  },
  recText: { fontSize: 11, fontWeight: '700', color: C.danger },
  isoText: { fontSize: 10, color: C.text, backgroundColor: 'rgba(0,0,0,0.4)', paddingHorizontal: 6, paddingVertical: 2, borderRadius: 4 },
  hudTopRight: {
    position: 'absolute',
    top: 10,
    right: 10,
    fontSize: 10,
    color: C.text,
  },
  hudBottom: {
    position: 'absolute',
    bottom: 10,
    left: 10,
    right: 10,
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  channelRow: { flexDirection: 'row', gap: 6 },
  channelBtn: {
    backgroundColor: 'rgba(0,0,0,0.55)',
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 9999,
    borderWidth: 1,
    borderColor: C.line,
  },
  channelBtnActive: { borderColor: `${C.accent}66` },
  channelText: { fontSize: 10, fontWeight: '600', color: C.text2, textTransform: 'uppercase', letterSpacing: 0.4 },
  channelTextActive: { color: C.accent },
  recBtn: {
    width: 44,
    height: 44,
    borderRadius: 22,
    borderWidth: 3,
    borderColor: '#fff',
    backgroundColor: 'transparent',
  },
  recBtnActive: { backgroundColor: C.danger },
  statsRow: { padding: 16, paddingBottom: 100 },
  statsCard: {},
  statsGrid: { flexDirection: 'row', gap: 8 },
});

