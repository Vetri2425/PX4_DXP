// app/firmware.tsx
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

const PACKAGES = [
  { name: 'PX4 Firmware', version: 'v1.16.2', latest: 'v1.16.2', ok: true },
  { name: 'ROS 2 Humble', version: '0.10.3', latest: '0.10.3', ok: true },
  { name: 'MAVROS2', version: '2.4.0', latest: '2.5.1', ok: false },
  { name: 'px4_dxp', version: '1.4.2', latest: '1.4.2', ok: true },
];

export default function FirmwareScreen() {
  return (
    <SafeAreaView style={styles.safeArea} edges={['top']}>
      <AppBar
        title="Firmware & Packages"
        subtitle="apt · pip · OTA"
        leading={<IconBtn icon={<Icons.chevL size={18} color={C.text2} />} onPress={() => router.back()} />}
        trailing={<IconBtn icon={<Icons.refresh size={18} color={C.text2} />} />}
      />
      <ScrollView contentContainerStyle={styles.content}>
        <SectionHeader title="Versions" />
        <View style={styles.section}>
          <Card pad={0}>
            {PACKAGES.map((pkg, i) => (
              <React.Fragment key={pkg.name}>
                <View style={styles.row}>
                  <Dot color={pkg.ok ? C.good : C.warn} size={8} />
                  <View style={styles.meta}>
                    <Text style={styles.pkgName}>{pkg.name}</Text>
                    <Text style={styles.pkgVersion}>
                      {pkg.version}
                      {!pkg.ok && ` → ${pkg.latest} available`}
                    </Text>
                  </View>
                  {!pkg.ok && (
                    <Btn variant="accentGhost" size="sm" icon={<Icons.download size={13} color={C.accent} />}>
                      Update
                    </Btn>
                  )}
                </View>
                {i < PACKAGES.length - 1 && <View style={styles.divider} />}
              </React.Fragment>
            ))}
          </Card>
        </View>

        <SectionHeader title="PX4 OTA" />
        <View style={styles.section}>
          <Card pad={14}>
            <Text style={styles.otaNote}>Flash a new firmware image via QGroundControl or direct upload.</Text>
            <Btn variant="secondary" size="sm" icon={<Icons.upload size={14} color={C.text2} />}>
              Upload .px4 file
            </Btn>
          </Card>
        </View>
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
    gap: 12,
    paddingHorizontal: 14,
    paddingVertical: 12,
  },
  meta: { flex: 1 },
  pkgName: { fontSize: 14, fontWeight: '500', color: C.text },
  pkgVersion: { fontSize: 12, color: C.text3, marginTop: 2 },
  divider: { height: 1, backgroundColor: C.line, marginLeft: 42, marginRight: 14 },
  otaNote: { fontSize: 13, color: C.text2, lineHeight: 20, marginBottom: 12 },
});
