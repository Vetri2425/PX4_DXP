// app/connect.tsx — 3-step connection flow: Scan → Connecting → Done
import React, { useState, useEffect } from 'react';
import { View, Text, Pressable, TextInput, ScrollView, StyleSheet, ActivityIndicator } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { router } from 'expo-router';
import { C } from '../theme/colors';
import { AppBar } from '../components/ui/AppBar';
import { Card } from '../components/ui/Card';
import { Btn } from '../components/ui/Btn';
import { IconBtn } from '../components/ui/IconBtn';
import { Dot } from '../components/ui/Dot';
import { SectionHeader } from '../components/ui/SectionHeader';
import { Icons } from '../components/icons';
import { useConnectionStore, type Rover } from '../stores/useConnectionStore';
import { initSocket, getSocket } from '../services/socket';

type Step = 'scan' | 'connecting' | 'done';

/** #5 — wait for real Socket.IO 'connect' event with a 10 s timeout */
async function waitForSocketConnect(): Promise<void> {
  const sock = await initSocket();
  if (sock.connected) return; // already connected (same URL, cached)

  return new Promise<void>((resolve, reject) => {
    const TIMEOUT_MS = 10_000;

    const timer = setTimeout(() => {
      cleanup();
      reject(new Error('Connection timed out (10 s)'));
    }, TIMEOUT_MS);

    const onConnect = () => {
      cleanup();
      resolve();
    };
    const onError = (err: Error) => {
      cleanup();
      reject(err);
    };

    const cleanup = () => {
      clearTimeout(timer);
      sock.off('connect', onConnect);
      sock.off('connect_error', onError);
    };

    sock.once('connect', onConnect);
    sock.once('connect_error', onError);
  });
}

function RadarLoader() {
  return (
    <View style={radar.container}>
      {[40, 70, 100].map((s) => (
        <View
          key={s}
          style={[radar.ring, { width: s, height: s, marginLeft: -s / 2, marginTop: -s / 2 }]}
        />
      ))}
      <View style={radar.dot} />
    </View>
  );
}

const radar = StyleSheet.create({
  container: {
    width: 120,
    height: 120,
    position: 'relative',
    alignItems: 'center',
    justifyContent: 'center',
  },
  ring: {
    position: 'absolute',
    top: '50%',
    left: '50%',
    borderRadius: 9999,
    borderWidth: 1,
    borderColor: C.accent,
    opacity: 0.4,
  },
  dot: {
    width: 10,
    height: 10,
    borderRadius: 5,
    backgroundColor: C.accent,
    shadowColor: C.accent,
    shadowOpacity: 1,
    shadowRadius: 8,
    shadowOffset: { width: 0, height: 0 },
  },
});

export default function ConnectScreen() {
  const { discover, discoveredRovers, discovering, setBaseUrl, activeRoverUrl } = useConnectionStore();
  const [step, setStep] = useState<Step>('scan');
  const [selectedRover, setSelectedRover] = useState<Rover | null>(null);
  const [manualUrl, setManualUrl] = useState('');
  const [showManual, setShowManual] = useState(false);
  const [connectError, setConnectError] = useState<string | null>(null);

  useEffect(() => {
    discover();
  }, []);

  const handleConnect = async (rover: Rover) => {
    setSelectedRover(rover);
    setStep('connecting');
    setConnectError(null);
    try {
      const url = `http://${rover.host}:${rover.port}`;
      await setBaseUrl(url);
      // #5 — wait for real socket 'connect' event instead of sleeping
      await waitForSocketConnect();
      setStep('done');
    } catch (e) {
      setConnectError((e as Error).message || 'Connection failed');
      setStep('scan');
    }
  };

  const handleManualConnect = async () => {
    if (!manualUrl.trim()) return;
    setSelectedRover(null);
    setStep('connecting');
    setConnectError(null);
    try {
      await setBaseUrl(manualUrl.trim());
      // #5 — wait for real socket 'connect' event instead of sleeping
      await waitForSocketConnect();
      setStep('done');
    } catch (e) {
      setConnectError((e as Error).message || 'Connection failed');
      setStep('scan');
    }
  };

  if (step === 'connecting') {
    return (
      <SafeAreaView style={styles.safeArea}>
        <AppBar
          title="Connecting..."
          subtitle={selectedRover?.name ?? manualUrl}
          leading={
            <IconBtn
              icon={<Icons.chevL size={18} color={C.text2} />}
              onPress={() => { setStep('scan'); }}
            />
          }
        />
        <View style={styles.centreContent}>
          <RadarLoader />
          <Text style={styles.connectingTitle}>
            {selectedRover
              ? `Connecting to ${selectedRover.name}`
              : `Connecting to ${manualUrl}`}
          </Text>
          <Text style={styles.connectingSubtitle}>
            Verifying Socket.IO handshake... ROS 2 DDS discovery... MAVLink heartbeat...
          </Text>
          {connectError && (
            <View style={styles.errorBox}>
              <Text style={styles.errorText}>{connectError}</Text>
            </View>
          )}
          <Btn variant="secondary" onPress={() => setStep('scan')}>Cancel</Btn>
        </View>
      </SafeAreaView>
    );
  }

  if (step === 'done') {
    return (
      <SafeAreaView style={styles.safeArea}>
        <AppBar title="Connected" subtitle="Ready" />
        <View style={styles.centreContent}>
          <View style={styles.successIcon}>
            <Icons.check size={44} color={C.good} />
          </View>
          <Text style={styles.connectedTitle}>Connected</Text>
          <Text style={styles.connectedUrl}>{activeRoverUrl}</Text>
          <Btn variant="primary" onPress={() => router.back()}>Open dashboard</Btn>
        </View>
      </SafeAreaView>
    );
  }

  // Scan step
  return (
    <SafeAreaView style={styles.safeArea}>
      <ScrollView contentContainerStyle={styles.scrollContent} showsVerticalScrollIndicator={false}>
        <AppBar
          title="Connect a rover"
          subtitle={
            discovering
              ? 'Listening for UDP beacons...'
              : discoveredRovers.length > 0
                ? `${discoveredRovers.length} rover${discoveredRovers.length !== 1 ? 's' : ''} found`
                : 'No rovers discovered'
          }
          leading={
            <IconBtn
              icon={<Icons.chevL size={18} color={C.text2} />}
              onPress={() => router.back()}
            />
          }
          trailing={
            <IconBtn
              icon={<Icons.refresh size={18} color={C.text2} />}
              onPress={() => discover()}
            />
          }
        />

        <View style={styles.section}>
          {/* Discovered rovers */}
          {discoveredRovers.length > 0 && (
            <Card pad={0}>
              {discoveredRovers.map((r, i) => (
                <React.Fragment key={r.name + r.host}>
                  <Pressable
                    onPress={() => handleConnect(r)}
                    style={({ pressed }) => [styles.roverRow, { opacity: pressed ? 0.75 : 1 }]}
                  >
                    <View style={styles.roverIcon}>
                      <Icons.wifi size={18} color={C.accent} />
                    </View>
                    <View style={styles.roverMeta}>
                      <Text style={styles.roverName}>{r.name}</Text>
                      <Text style={styles.roverSub}>{r.host}:{r.port} · {r.status}</Text>
                    </View>
                    <Text style={styles.roverTag}>LAN</Text>
                  </Pressable>
                  {i < discoveredRovers.length - 1 && <View style={styles.divider} />}
                </React.Fragment>
              ))}
            </Card>
          )}

          {/* Scanning indicator */}
          {discovering && (
            <View style={styles.scanningRow}>
              <ActivityIndicator size="small" color={C.accent} />
              <Text style={styles.scanningText}>Listening on UDP port 5002...</Text>
            </View>
          )}

          {/* Empty state */}
          {!discovering && discoveredRovers.length === 0 && (
            <View style={styles.emptyState}>
              <Icons.wifi size={40} color={C.text3} />
              <Text style={styles.emptyTitle}>No rovers found</Text>
              <Text style={styles.emptySub}>
                Make sure the rover server is running and broadcasting on UDP 5002.
              </Text>
              <Btn
                variant="secondary"
                size="sm"
                icon={<Icons.refresh size={14} color={C.text2} />}
                onPress={() => discover()}
              >
                Scan again
              </Btn>
            </View>
          )}

          {/* Manual entry */}
          <SectionHeader title="Manual entry" />
          <Card pad={0}>
            <Pressable
              onPress={() => setShowManual(!showManual)}
              style={styles.roverRow}
            >
              <View style={[styles.roverIcon, { backgroundColor: `${C.text3}1c` }]}>
                <Icons.link size={18} color={C.text3} />
              </View>
              <View style={styles.roverMeta}>
                <Text style={styles.roverName}>Enter IP address manually</Text>
                <Text style={styles.roverSub}>
                  {showManual ? 'Tap to collapse' : 'For rovers outside auto-discovery range'}
                </Text>
              </View>
            </Pressable>
            {showManual && (
              <View style={styles.manualInputRow}>
                <TextInput
                  value={manualUrl}
                  onChangeText={setManualUrl}
                  placeholder="192.168.1.102:5001"
                  placeholderTextColor={C.text3}
                  style={styles.manualInput}
                  onSubmitEditing={handleManualConnect}
                  autoCapitalize="none"
                  keyboardType="url"
                />
                <Btn
                  variant="primary"
                  size="sm"
                  disabled={!manualUrl.trim()}
                  onPress={handleManualConnect}
                >
                  Connect
                </Btn>
              </View>
            )}
          </Card>

          {/* Other methods (stub) */}
          <SectionHeader title="Other methods" />
          <Card pad={0}>
            {[
              { icon: 'signal' as const, label: 'Bluetooth (serial bridge)', sub: 'Requires BT pairing on the rover', color: C.text3 },
              { icon: 'link' as const, label: 'USB tethered', sub: 'Direct connection via USB-C', color: C.text3 },
              { icon: 'fleet' as const, label: 'Simulation (SITL)', sub: 'PX4 SITL on localhost', color: C.text3 },
            ].map((item, i, arr) => (
              <React.Fragment key={item.label}>
                <View style={styles.roverRow}>
                  <View style={[styles.roverIcon, { backgroundColor: `${item.color}1c` }]}>
                    {React.createElement(Icons[item.icon], { size: 18, color: item.color })}
                  </View>
                  <View style={styles.roverMeta}>
                    <Text style={styles.roverName}>{item.label}</Text>
                    <Text style={styles.roverSub}>{item.sub}</Text>
                  </View>
                </View>
                {i < arr.length - 1 && <View style={styles.divider} />}
              </React.Fragment>
            ))}
          </Card>

          <View style={styles.baseUrlBox}>
            <Text style={styles.baseUrlText}>Current: {activeRoverUrl}</Text>
          </View>
        </View>
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: C.bg },
  scrollContent: { paddingBottom: 100 },
  section: { paddingHorizontal: 16, gap: 0 },
  centreContent: {
    flex: 1,
    alignItems: 'center',
    justifyContent: 'center',
    padding: 40,
    gap: 18,
  },
  connectingTitle: { fontSize: 14, color: C.text2, textAlign: 'center' },
  connectingSubtitle: { fontSize: 11, color: C.text3, textAlign: 'center' },
  errorBox: {
    padding: 12,
    borderRadius: 10,
    backgroundColor: `${C.danger}1a`,
    borderWidth: 1,
    borderColor: `${C.danger}33`,
    maxWidth: 280,
  },
  errorText: { color: C.danger, fontSize: 12, textAlign: 'center' },
  successIcon: {
    width: 96,
    height: 96,
    borderRadius: 48,
    backgroundColor: `${C.good}26`,
    alignItems: 'center',
    justifyContent: 'center',
  },
  connectedTitle: { fontSize: 18, fontWeight: '700', color: C.text },
  connectedUrl: { fontSize: 12, color: C.text3 },
  roverRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 12,
    paddingHorizontal: 14,
    paddingVertical: 12,
  },
  roverIcon: {
    width: 36,
    height: 36,
    borderRadius: 10,
    backgroundColor: `${C.accent}1c`,
    alignItems: 'center',
    justifyContent: 'center',
    flexShrink: 0,
  },
  roverMeta: { flex: 1 },
  roverName: { fontSize: 15, fontWeight: '500', color: C.text },
  roverSub: { fontSize: 12, color: C.text3, marginTop: 1 },
  roverTag: {
    fontSize: 10,
    color: C.accent,
    fontWeight: '600',
    backgroundColor: `${C.accent}1a`,
    paddingHorizontal: 7,
    paddingVertical: 3,
    borderRadius: 6,
  },
  divider: { height: 1, backgroundColor: C.line, marginLeft: 64, marginRight: 14 },
  scanningRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 10,
    padding: 10,
    marginBottom: 12,
    borderRadius: 12,
    backgroundColor: `${C.accent}0d`,
    borderWidth: 1,
    borderColor: `${C.accent}22`,
  },
  scanningText: { fontSize: 12, color: C.accent, fontWeight: '500' },
  emptyState: { alignItems: 'center', padding: 24, gap: 8 },
  emptyTitle: { fontSize: 14, fontWeight: '500', color: C.text2 },
  emptySub: { fontSize: 12, color: C.text3, textAlign: 'center', maxWidth: 260 },
  manualInputRow: {
    flexDirection: 'row',
    gap: 8,
    paddingHorizontal: 14,
    paddingBottom: 14,
  },
  manualInput: {
    flex: 1,
    padding: 10,
    borderRadius: 10,
    backgroundColor: C.card2,
    borderWidth: 1,
    borderColor: C.line2,
    color: C.text,
    fontSize: 13,
  },
  baseUrlBox: {
    marginTop: 14,
    padding: 10,
    borderRadius: 10,
    backgroundColor: C.card2,
  },
  baseUrlText: { fontSize: 11, color: C.text3 },
});
