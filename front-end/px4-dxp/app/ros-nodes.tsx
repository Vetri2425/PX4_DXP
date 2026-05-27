// app/ros-nodes.tsx
import React from 'react';
import { View, Text, ScrollView, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { router } from 'expo-router';
import { C } from '../theme/colors';
import { AppBar } from '../components/ui/AppBar';
import { Card } from '../components/ui/Card';
import { Dot } from '../components/ui/Dot';
import { SectionHeader } from '../components/ui/SectionHeader';
import { IconBtn } from '../components/ui/IconBtn';
import { Icons } from '../components/icons';

const NODES = [
  { name: '/mavros', pkg: 'mavros', alive: true },
  { name: '/rpp_controller', pkg: 'px4_dxp', alive: true },
  { name: '/twist_to_setpoint', pkg: 'px4_dxp', alive: true },
  { name: '/ntrip_client', pkg: 'ntrip_client', alive: true },
  { name: '/robot_state_publisher', pkg: 'robot_state_publisher', alive: false },
];

export default function RosNodesScreen() {
  return (
    <SafeAreaView style={styles.safeArea} edges={['bottom']}>
      <AppBar
        title="ROS 2 Nodes"
        subtitle={`${NODES.filter((n) => n.alive).length} alive · domain 42`}
        leading={<IconBtn icon={<Icons.chevL size={18} color={C.text2} />} onPress={() => router.back()} />}
        trailing={<IconBtn icon={<Icons.refresh size={18} color={C.text2} />} />}
      />
      <ScrollView contentContainerStyle={styles.content}>
        <SectionHeader title="Active nodes" />
        <View style={styles.section}>
          <Card pad={0}>
            {NODES.map((node, i) => (
              <React.Fragment key={node.name}>
                <View style={styles.row}>
                  <Dot color={node.alive ? C.good : C.text3} size={8} pulse={node.alive} />
                  <View style={styles.meta}>
                    <Text style={styles.nodeName}>{node.name}</Text>
                    <Text style={styles.nodePkg}>{node.pkg}</Text>
                  </View>
                  <Text style={[styles.status, { color: node.alive ? C.good : C.text3 }]}>
                    {node.alive ? 'alive' : 'dead'}
                  </Text>
                </View>
                {i < NODES.length - 1 && <View style={styles.divider} />}
              </React.Fragment>
            ))}
          </Card>
        </View>
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: C.bg },
  content: { paddingBottom: 28 },
  section: { paddingHorizontal: 16 },
  row: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 12,
    paddingHorizontal: 14,
    paddingVertical: 12,
  },
  meta: { flex: 1 },
  nodeName: { fontSize: 14, fontWeight: '500', color: C.text },
  nodePkg: { fontSize: 11, color: C.text3, marginTop: 1 },
  status: { fontSize: 12, fontWeight: '600' },
  divider: { height: 1, backgroundColor: C.line, marginLeft: 42, marginRight: 14 },
});
