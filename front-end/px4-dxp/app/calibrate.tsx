// app/calibrate.tsx
import React from 'react';
import { View, Text, ScrollView, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { router } from 'expo-router';
import { C } from '../theme/colors';
import { AppBar } from '../components/ui/AppBar';
import { Card } from '../components/ui/Card';
import { Btn } from '../components/ui/Btn';
import { SectionHeader } from '../components/ui/SectionHeader';
import { IconBtn } from '../components/ui/IconBtn';
import { Icons } from '../components/icons';
import { Dot } from '../components/ui/Dot';

const CAL_ITEMS = [
  { label: 'Compass', status: 'needs cal', ok: false },
  { label: 'Accelerometer', status: 'ok', ok: true },
  { label: 'Gyroscope', status: 'ok', ok: true },
  { label: 'Level horizon', status: 'ok', ok: true },
];

export default function CalibrateScreen() {
  return (
    <SafeAreaView style={styles.safeArea} edges={['bottom']}>
      <AppBar
        title="Calibration"
        subtitle="Compass · accel · gyro"
        leading={<IconBtn icon={<Icons.chevL size={18} color={C.text2} />} onPress={() => router.back()} />}
      />
      <ScrollView contentContainerStyle={styles.content}>
        <SectionHeader title="Status" />
        <View style={styles.section}>
          <Card pad={0}>
            {CAL_ITEMS.map((item, i) => (
              <React.Fragment key={item.label}>
                <View style={styles.row}>
                  <Dot color={item.ok ? C.good : C.warn} size={8} pulse={!item.ok} />
                  <Text style={styles.itemLabel}>{item.label}</Text>
                  <Text style={[styles.itemStatus, { color: item.ok ? C.good : C.warn }]}>
                    {item.status}
                  </Text>
                </View>
                {i < CAL_ITEMS.length - 1 && <View style={styles.divider} />}
              </React.Fragment>
            ))}
          </Card>
        </View>

        <SectionHeader title="Run calibration" />
        <View style={styles.section}>
          <Card pad={14}>
            <Text style={styles.calNote}>
              Place the rover on a flat surface and rotate it slowly in a full circle when prompted.
            </Text>
            <Btn
              variant="primary"
              size="md"
              icon={<Icons.compass size={15} color="#06202a" />}
              style={styles.calBtn}
            >
              Start compass calibration
            </Btn>
          </Card>
        </View>
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: C.bg },
  content: { paddingBottom: 28 },
  section: { paddingHorizontal: 16, paddingBottom: 4 },
  row: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 12,
    paddingHorizontal: 14,
    paddingVertical: 12,
  },
  itemLabel: { flex: 1, fontSize: 15, fontWeight: '500', color: C.text },
  itemStatus: { fontSize: 12, fontWeight: '600' },
  divider: { height: 1, backgroundColor: C.line, marginLeft: 42, marginRight: 14 },
  calNote: { fontSize: 13, color: C.text2, lineHeight: 20, marginBottom: 12 },
  calBtn: { alignSelf: 'flex-start' },
});
