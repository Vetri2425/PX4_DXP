// app/px4-params.tsx
import React from 'react';
import { View, Text, ScrollView, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { router } from 'expo-router';
import { C } from '../theme/colors';
import { AppBar } from '../components/ui/AppBar';
import { Card } from '../components/ui/Card';
import { SectionHeader } from '../components/ui/SectionHeader';
import { IconBtn } from '../components/ui/IconBtn';
import { Icons } from '../components/icons';

const PARAMS = [
  { name: 'NAV_ACC_RAD', value: '0.10', unit: 'm', desc: 'Waypoint acceptance radius' },
  { name: 'RO_YAW_RATE_P', value: '0.50', unit: '', desc: 'Yaw rate P gain' },
  { name: 'RO_YAW_RATE_I', value: '0.30', unit: '', desc: 'Yaw rate I gain' },
  { name: 'RBCLW_QPPS_MAX', value: '162162', unit: 'ticks/s', desc: 'RoboClaw max velocity' },
  { name: 'RD_TANK_MODE', value: '1', unit: '', desc: 'Tank steering mode (1=on)' },
  { name: 'GPS_YAW_OFFSET', value: '180.0', unit: '°', desc: 'GPS antenna yaw offset' },
];

export default function Px4ParamsScreen() {
  return (
    <SafeAreaView style={styles.safeArea} edges={['bottom']}>
      <AppBar
        title="PX4 Parameters"
        subtitle="Tune & save"
        leading={<IconBtn icon={<Icons.chevL size={18} color={C.text2} />} onPress={() => router.back()} />}
        trailing={<IconBtn icon={<Icons.search size={18} color={C.text2} />} />}
      />
      <ScrollView contentContainerStyle={styles.content}>
        <SectionHeader title="Key parameters" />
        <View style={styles.section}>
          <Card pad={0}>
            {PARAMS.map((p, i) => (
              <React.Fragment key={p.name}>
                <View style={styles.row}>
                  <View style={styles.meta}>
                    <Text style={styles.paramName}>{p.name}</Text>
                    <Text style={styles.paramDesc}>{p.desc}</Text>
                  </View>
                  <Text style={styles.paramValue}>
                    {p.value}{p.unit ? ` ${p.unit}` : ''}
                  </Text>
                </View>
                {i < PARAMS.length - 1 && <View style={styles.divider} />}
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
    paddingHorizontal: 14,
    paddingVertical: 12,
    gap: 12,
  },
  meta: { flex: 1 },
  paramName: { fontSize: 13, fontWeight: '600', color: C.text },
  paramDesc: { fontSize: 11, color: C.text3, marginTop: 2 },
  paramValue: { fontSize: 13, color: C.accent, fontWeight: '600' },
  divider: { height: 1, backgroundColor: C.line, marginHorizontal: 14 },
});
