// app/(tabs)/more.tsx
import React from 'react';
import { View, Text, Pressable, ScrollView, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { router } from 'expo-router';
import { C } from '../../theme/colors';
import { AppBar } from '../../components/ui/AppBar';
import { Card } from '../../components/ui/Card';
import { SectionHeader } from '../../components/ui/SectionHeader';
import { Icons } from '../../components/icons';
import type { IconName } from '../../components/icons';

interface RowItem {
  icon: IconName;
  color: string;
  title: string;
  sub: string;
  route: string;
}

const OPERATIONS: RowItem[] = [
  { icon: 'cam', color: C.warn, title: 'Live camera', sub: '1080p · 30fps', route: '/camera' },
  { icon: 'ros', color: C.accent, title: 'ROS 2 nodes', sub: '18 alive · domain 42', route: '/ros-nodes' },
  { icon: 'sliders', color: '#a78bfa', title: 'PX4 parameters', sub: 'Tune & save', route: '/px4-params' },
  { icon: 'compass', color: C.warn, title: 'Calibration', sub: 'Compass · accel · gyro', route: '/calibrate' },
  { icon: 'log', color: C.text2, title: 'Logs & diagnostics', sub: 'Rosout · MAVLink · uORB', route: '/logs' },
];

const SYSTEM: RowItem[] = [
  { icon: 'firmware', color: C.good, title: 'Firmware & packages', sub: 'apt · pip · OTA', route: '/firmware' },
  { icon: 'fleet', color: C.accent2, title: 'Fleet', sub: '1 rover', route: '/fleet' },
  { icon: 'link', color: C.accent, title: 'Connect rover', sub: 'Wi-Fi · BT · serial', route: '/connect' },
  { icon: 'settings', color: C.text2, title: 'Settings', sub: 'Account · units · API', route: '/settings' },
];

function Row({ item }: { item: RowItem }) {
  const Ic = Icons[item.icon];
  return (
    <Pressable
      onPress={() => router.push(item.route as never)}
      style={({ pressed }) => [styles.row, { opacity: pressed ? 0.7 : 1 }]}
    >
      <View style={[styles.rowIcon, { backgroundColor: `${item.color}1c` }]}>
        <Ic size={18} color={item.color} />
      </View>
      <View style={styles.rowText}>
        <Text style={styles.rowTitle}>{item.title}</Text>
        <Text style={styles.rowSub}>{item.sub}</Text>
      </View>
      <Icons.chevR size={16} color={C.text3} />
    </Pressable>
  );
}

export default function MoreScreen() {
  return (
    <SafeAreaView style={styles.safeArea} edges={['top']}>
      <ScrollView contentContainerStyle={styles.content} showsVerticalScrollIndicator={false}>
        <AppBar
          title="More"
          subtitle="Maintenance, diagnostics, fleet"
        />

        <SectionHeader title="Operations" />
        <View style={styles.section}>
          <Card pad={0}>
            {OPERATIONS.map((item, i) => (
              <React.Fragment key={item.route}>
                <Row item={item} />
                {i < OPERATIONS.length - 1 && <View style={styles.divider} />}
              </React.Fragment>
            ))}
          </Card>
        </View>

        <SectionHeader title="System" />
        <View style={styles.section}>
          <Card pad={0}>
            {SYSTEM.map((item, i) => (
              <React.Fragment key={item.route}>
                <Row item={item} />
                {i < SYSTEM.length - 1 && <View style={styles.divider} />}
              </React.Fragment>
            ))}
          </Card>
        </View>

        <Text style={styles.version}>
          DXP 1.4.2 (build 2026.05.25) · PX4 v1.16.2 · ROS 2 Humble
        </Text>
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: C.bg },
  content: { paddingBottom: 100 },
  section: { paddingHorizontal: 16, paddingBottom: 4 },
  row: {
    flexDirection: 'row',
    alignItems: 'center',
    paddingHorizontal: 14,
    paddingVertical: 12,
    gap: 12,
  },
  rowIcon: {
    width: 36,
    height: 36,
    borderRadius: 10,
    alignItems: 'center',
    justifyContent: 'center',
    flexShrink: 0,
  },
  rowText: { flex: 1 },
  rowTitle: { fontSize: 15, fontWeight: '500', color: C.text },
  rowSub: { fontSize: 12, color: C.text3, marginTop: 1 },
  divider: {
    height: 1,
    backgroundColor: C.line,
    marginLeft: 64,
    marginRight: 14,
  },
  version: {
    textAlign: 'center',
    fontSize: 11,
    color: C.text3,
    paddingVertical: 16,
  },
});
