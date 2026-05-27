// app/(tabs)/index.tsx
import React from 'react';
import { ScrollView, View, Text, Pressable, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { router } from 'expo-router';
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
import { useConnectionStore } from '../../stores/useConnectionStore';

export default function HomeScreen() {
  const { backendConnected, backendError, activeRoverUrl } = useConnectionStore();

  // ── Disconnected state ────────────────────────────────────────────────────
  if (!backendConnected) {
    return (
      <SafeAreaView style={styles.safeArea} edges={[]}>
        <ConnectionBadge />
        <View style={styles.disconnectedRoot}>
          <View style={styles.disconnectedCard}>
            <Icons.wifi size={48} color={C.text3} />
            <Text style={styles.disconnectedTitle}>No rover connected</Text>
            <Text style={styles.disconnectedSub}>
              {backendError
                ? `Error: ${backendError}`
                : 'Launch the connect screen to discover or manually connect to a rover.'}
            </Text>
            <Pressable
              style={({ pressed }) => [
                styles.connectBtn,
                { opacity: pressed ? 0.75 : 1 },
              ]}
              onPress={() => router.push('/connect')}
            >
              <Icons.wifi size={16} color={C.accent} />
              <Text style={styles.connectBtnText}>Open connect screen</Text>
            </Pressable>
          </View>
          <Text style={styles.disconnectedUrl}>Current URL: {activeRoverUrl}</Text>
        </View>
      </SafeAreaView>
    );
  }

  // ── Connected state (normal dashboard) ────────────────────────────────────
  return (
    <SafeAreaView style={styles.safeArea} edges={[]}>
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
              <IconBtn icon={<Icons.cam size={18} color={C.accent} />} accent onPress={() => router.push('/camera')} />
            </View>
          }
        />

        <View style={styles.section}>
          <RoverHeroCard />
        </View>

        <SectionHeader title="Quick actions" />
        <QuickActions />

        <SectionHeader title="System" action={{ label: 'Open', onClick: () => router.push('/ros-nodes') }} />
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
    paddingBottom: 110,
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
  // ── Disconnected state ──────────────────────────────────────────────────
  disconnectedRoot: {
    flex: 1,
    alignItems: 'center',
    justifyContent: 'center',
    padding: 32,
    gap: 16,
  },
  disconnectedCard: {
    alignItems: 'center',
    gap: 12,
    paddingHorizontal: 20,
    paddingVertical: 28,
    borderRadius: 16,
    backgroundColor: C.card,
    borderWidth: 1,
    borderColor: C.line,
    maxWidth: 300,
    width: '100%',
  },
  disconnectedTitle: {
    fontSize: 16,
    fontWeight: '700',
    color: C.text,
  },
  disconnectedSub: {
    fontSize: 12,
    color: C.text3,
    textAlign: 'center',
    lineHeight: 18,
  },
  connectBtn: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
    marginTop: 4,
    paddingHorizontal: 18,
    paddingVertical: 11,
    borderRadius: 10,
    backgroundColor: `${C.accent}18`,
    borderWidth: 1,
    borderColor: `${C.accent}33`,
  },
  connectBtnText: {
    fontSize: 13,
    fontWeight: '600',
    color: C.accent,
  },
  disconnectedUrl: {
    fontSize: 10,
    color: C.text3,
  },
});
