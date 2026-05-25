// app/(tabs)/index.tsx
import React from 'react';
import { ScrollView, View, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { C } from '../../theme/colors';
import { AppBar } from '../../components/ui/AppBar';
import { SectionHeader } from '../../components/ui/SectionHeader';
import { ConnectionBadge } from '../../components/dashboard/ConnectionBadge';
import { EmergencyOverlay } from '../../components/dashboard/EmergencyOverlay';
import { RoverHeroCard } from '../../components/dashboard/RoverHeroCard';
import { QuickActions } from '../../components/dashboard/QuickActions';
import { SysDiagnostics } from '../../components/dashboard/SysDiagnostics';
import { IconBtn } from '../../components/ui/IconBtn';
import { Icons } from '../../components/icons';
import { useUiStore } from '../../stores/useUiStore';

export default function HomeScreen() {
  const { push } = useUiStore();

  return (
    <SafeAreaView style={styles.safeArea} edges={['top']}>
      <ConnectionBadge />
      <EmergencyOverlay />
      <ScrollView
        style={styles.scroll}
        contentContainerStyle={styles.content}
        showsVerticalScrollIndicator={false}
      >
        <AppBar
          title="DXP"
          subtitle="Drawing Rover Workbench"
          leading={<IconBtn icon={<Icons.menu size={18} color={C.text2} />} />}
          trailing={
            <View style={styles.trailingRow}>
              <IconBtn icon={<Icons.bell size={18} color={C.text2} />} badge={0} />
              <IconBtn icon={<Icons.cam size={18} color={C.accent} />} accent onPress={() => push('camera')} />
            </View>
          }
        />

        <View style={styles.section}>
          <RoverHeroCard />
        </View>

        <SectionHeader title="Quick actions" />
        <QuickActions />

        <SectionHeader title="System" action={{ label: 'Open', onClick: () => push('ros') }} />
        <SysDiagnostics />

        <View style={styles.bottomSpacer} />
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: C.bg,
  },
  scroll: {
    flex: 1,
  },
  content: {
    paddingBottom: 100,
  },
  section: {
    paddingHorizontal: 16,
    paddingBottom: 12,
  },
  trailingRow: {
    flexDirection: 'row',
    gap: 8,
  },
  bottomSpacer: {
    height: 20,
  },
});
