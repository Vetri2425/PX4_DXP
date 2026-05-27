// app/fleet.tsx
import React from 'react';
import { View, Text, ScrollView, Pressable, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { router } from 'expo-router';
import { C } from '../theme/colors';
import { AppBar } from '../components/ui/AppBar';
import { Card } from '../components/ui/Card';
import { Dot } from '../components/ui/Dot';
import { SectionHeader } from '../components/ui/SectionHeader';
import { IconBtn } from '../components/ui/IconBtn';
import { Icons } from '../components/icons';

const FLEET = [
  { id: 'dxp-01', name: 'DXP-01 Mercutio', status: 'connected' as const, battery: 78, ip: '192.168.1.102' },
  { id: 'dxp-02', name: 'DXP-02 Benvolio', status: 'offline' as const, battery: 45, ip: '192.168.1.103' },
];

const STATUS_COLOR = { connected: C.good, offline: C.text3, standby: C.accent };

export default function FleetScreen() {
  return (
    <SafeAreaView style={styles.safeArea} edges={['bottom']}>
      <AppBar
        title="Fleet"
        subtitle={`${FLEET.length} rovers`}
        leading={<IconBtn icon={<Icons.chevL size={18} color={C.text2} />} onPress={() => router.back()} />}
        trailing={<IconBtn icon={<Icons.plus size={18} color={C.text2} />} />}
      />
      <ScrollView contentContainerStyle={styles.content}>
        <SectionHeader title="Rovers" />
        <View style={styles.section}>
          {FLEET.map((rover) => {
            const sc = STATUS_COLOR[rover.status] ?? C.text3;
            return (
              <Card key={rover.id} pad={14} style={styles.card}>
                <View style={styles.cardHeader}>
                  <Dot color={sc} size={8} pulse={rover.status === 'connected'} />
                  <Text style={[styles.status, { color: sc }]}>{rover.status}</Text>
                </View>
                <Text style={styles.roverName}>{rover.name}</Text>
                <View style={styles.cardMeta}>
                  <Text style={styles.metaText}>{rover.ip}</Text>
                  <Text style={styles.metaText}>BAT {rover.battery}%</Text>
                </View>
              </Card>
            );
          })}
        </View>
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: C.bg },
  content: { paddingBottom: 28 },
  section: { paddingHorizontal: 16, gap: 10 },
  card: { marginBottom: 0 },
  cardHeader: { flexDirection: 'row', alignItems: 'center', gap: 6, marginBottom: 6 },
  status: { fontSize: 11, fontWeight: '600', textTransform: 'uppercase', letterSpacing: 0.6 },
  roverName: { fontSize: 16, fontWeight: '600', color: C.text, marginBottom: 6 },
  cardMeta: { flexDirection: 'row', gap: 12 },
  metaText: { fontSize: 12, color: C.text3 },
});
