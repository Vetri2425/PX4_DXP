// app/logs.tsx
import React from 'react';
import { View, Text, ScrollView, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { router } from 'expo-router';
import { C } from '../theme/colors';
import { AppBar } from '../components/ui/AppBar';
import { Card } from '../components/ui/Card';
import { IconBtn } from '../components/ui/IconBtn';
import { Icons } from '../components/icons';

const LOG_LINES = [
  { ts: '16:39:54.001', level: 'INFO', msg: 'RoboClaw: Successfully connected on /dev/ttyS5' },
  { ts: '16:39:54.412', level: 'INFO', msg: 'EKF2: initialised with GPS fix type 5' },
  { ts: '16:39:55.100', level: 'WARN', msg: 'Compass divergence detected, recalibrate recommended' },
  { ts: '16:39:55.222', level: 'INFO', msg: 'Mission: 5 waypoints loaded' },
  { ts: '16:39:56.001', level: 'INFO', msg: 'MAVROS: heartbeat received from PX4' },
  { ts: '16:39:57.001', level: 'ERR',  msg: 'USB camera: frame drop detected (bandwidth)' },
  { ts: '16:39:58.100', level: 'INFO', msg: 'RPP pipeline: lookahead 0.82 m, xtrack 0.04 m' },
];

const LEVEL_COLOR: Record<string, string> = {
  INFO: C.good,
  WARN: C.warn,
  ERR: C.danger,
};

export default function LogsScreen() {
  return (
    <SafeAreaView style={styles.safeArea} edges={['top']}>
      <AppBar
        title="Logs & Diagnostics"
        subtitle="Rosout · MAVLink · uORB"
        leading={<IconBtn icon={<Icons.chevL size={18} color={C.text2} />} onPress={() => router.back()} />}
        trailing={<IconBtn icon={<Icons.download size={18} color={C.text2} />} />}
      />
      <View style={styles.logContainer}>
        <Card pad={0} style={styles.logCard}>
          <ScrollView style={styles.logScroll}>
            {LOG_LINES.map((l, i) => (
              <View key={i} style={styles.logLine}>
                <Text style={styles.logTs}>{l.ts}</Text>
                <Text style={[styles.logLevel, { color: LEVEL_COLOR[l.level] ?? C.text2 }]}>
                  {l.level.padEnd(4)}
                </Text>
                <Text style={styles.logMsg}>{l.msg}</Text>
              </View>
            ))}
          </ScrollView>
        </Card>
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: C.bg },
  logContainer: { flex: 1, paddingHorizontal: 16, paddingBottom: 100 },
  logCard: { flex: 1, overflow: 'hidden' },
  logScroll: { flex: 1, backgroundColor: '#0a0d12' },
  logLine: {
    flexDirection: 'row',
    paddingHorizontal: 12,
    paddingVertical: 4,
    gap: 8,
  },
  logTs: { fontSize: 10, color: C.text3, flexShrink: 0, width: 80 },
  logLevel: { fontSize: 10, fontWeight: '700', flexShrink: 0, width: 36 },
  logMsg: { fontSize: 11, color: C.text2, flexShrink: 1, flexWrap: 'wrap' },
});
