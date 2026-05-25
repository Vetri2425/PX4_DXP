// app/(tabs)/drive.tsx
import React, { useState } from 'react';
import { View, Text, ScrollView, Pressable, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { C } from '../../theme/colors';
import { AppBar } from '../../components/ui/AppBar';
import { Card } from '../../components/ui/Card';
import { IconBtn } from '../../components/ui/IconBtn';
import { Icons } from '../../components/icons';
import { AttitudeIndicator } from '../../components/drive/AttitudeIndicator';
import { HeadingDisc } from '../../components/drive/HeadingDisc';
import { MiniStat } from '../../components/drive/MiniStat';
import { MotorTile } from '../../components/drive/MotorTile';
import { Joystick } from '../../components/drive/Joystick';
import { useTelemetryStore } from '../../stores/useTelemetryStore';
import { useUiStore } from '../../stores/useUiStore';
import { useMissionStore } from '../../stores/useMissionStore';
import { api } from '../../services/api';

interface ActionChipProps {
  on?: boolean;
  onPress?: () => void;
  icon: React.ReactNode;
  label: string;
  color?: string;
}

function ActionChip({ on, onPress, icon, label, color = C.accent }: ActionChipProps) {
  return (
    <Pressable
      onPress={onPress}
      style={({ pressed }) => [
        styles.chip,
        {
          backgroundColor: on ? `${color}26` : C.card2,
          borderColor: on ? `${color}66` : C.line2,
          opacity: pressed ? 0.75 : 1,
        },
      ]}
    >
      {icon}
      <Text style={[styles.chipLabel, { color: on ? color : C.text2 }]}>{label}</Text>
    </Pressable>
  );
}

export default function DriveScreen() {
  const { pitch, roll, heading, speed, alt, voltage, current, motor } = useTelemetryStore();
  const { armed, emergency, setArmed, triggerEStop, clearEStop, push, appendLog } = useUiStore();
  const { setMissionMode } = useMissionStore();
  const [penDown, setPenDown] = useState(false);
  const [headlights, setHeadlights] = useState(true);

  const handleArm = async () => {
    // #3 — never lie about arm state; only flip UI on backend confirmation (arm_result)
    try {
      await api.arm(!armed);
      // UI state is updated by the arm_result socket event, not here
    } catch (e) {
      appendLog('ERR', `Arm command failed: ${(e as Error).message}`);
    }
  };

  const handleEStop = async () => {
    // #1 — call backend BEFORE flipping local state
    try {
      await api.estop();
    } catch (e) {
      appendLog('WARN', `E-stop backend call failed: ${(e as Error).message}`);
      // Still trigger locally — hardware safety > UI consistency
    }
    triggerEStop();
  };

  const handleClearEStop = async () => {
    clearEStop();
  };

  const handleHold = async () => {
    try {
      await api.setMode('Hold');
    } catch { /* mode_result event will update store */ }
    setMissionMode('Hold');
  };

  return (
    <SafeAreaView style={styles.safeArea} edges={['top']}>
      <ScrollView
        style={styles.scroll}
        contentContainerStyle={styles.content}
        showsVerticalScrollIndicator={false}
      >
        <AppBar
          title="Manual Drive"
          subtitle={armed ? 'Armed · throttle live' : 'Disarmed · safe'}
          trailing={
            <View style={styles.trailingRow}>
              <IconBtn
                icon={<Icons.cam size={18} color={C.text2} />}
                onPress={() => push('camera')}
              />
              <IconBtn
                icon={armed
                  ? <Icons.unlock size={18} color={C.accent} />
                  : <Icons.lock size={18} color={C.text2} />
                }
                accent={armed}
                onPress={handleArm}
              />
            </View>
          }
        />

        {/* Attitude + Heading row */}
        <View style={styles.attitudeRow}>
          <Card pad={12} style={styles.attitudeCard}>
            <Text style={styles.sectionLabel}>ATTITUDE</Text>
            <AttitudeIndicator pitch={pitch} roll={roll} />
            <View style={styles.attitudeStats}>
              <Text style={styles.attitudeStat}>
                R: <Text style={{ color: C.accent }}>{roll.toFixed(1)}°</Text>
              </Text>
              <Text style={styles.attitudeStat}>
                P: <Text style={{ color: C.accent }}>{pitch.toFixed(1)}°</Text>
              </Text>
              <Text style={styles.attitudeStat}>
                Y: <Text style={{ color: C.accent }}>{Math.round(heading)}°</Text>
              </Text>
            </View>
          </Card>

          <Card pad={12} style={styles.headingCard}>
            <Text style={styles.sectionLabel}>HEADING</Text>
            <HeadingDisc heading={heading} />
          </Card>
        </View>

        {/* Telemetry strip */}
        <View style={styles.statsRow}>
          <MiniStat label="SPD" value={speed.toFixed(2)} unit="m/s" color={C.accent} />
          <MiniStat label="ALT" value={alt.toFixed(2)} unit="m" />
          <MiniStat label="VBAT" value={voltage.toFixed(1)} unit="V" />
          <MiniStat label="AMP" value={current.toFixed(1)} unit="A" color={C.warn} />
        </View>

        {/* Motor monitor */}
        <View style={styles.section}>
          <Card pad={12}>
            <View style={styles.motorHeader}>
              <Text style={styles.sectionLabel}>MOTORS</Text>
              <Text style={styles.motorSubtitle}>2× DC · 12V encoder</Text>
            </View>
            <View style={styles.motorGrid}>
              {(['FL', 'FR', 'RL', 'RR'] as const).map((m, i) => (
                <MotorTile key={m} label={m} value={motor[i] ?? 0} />
              ))}
            </View>
          </Card>
        </View>

        {/* Dual joysticks */}
        <View style={styles.joystickRow}>
          <View style={styles.joystickCell}>
            <Joystick
              label="DRIVE"
              hint="↑↓ throttle  ←→ yaw"
              disabled={!armed || emergency}
            />
          </View>
          <View style={styles.joystickCell}>
            <Joystick
              label="STEER"
              hint="↑↓ pitch  ←→ roll"
              disabled={!armed || emergency}
            />
          </View>
        </View>

        {/* Action chips */}
        <View style={styles.section}>
          <Card pad={12}>
            <View style={styles.chips}>
              <ActionChip
                on={penDown}
                onPress={() => setPenDown((p) => !p)}
                icon={<Icons.draw size={14} color={penDown ? '#a78bfa' : C.text2} />}
                label={penDown ? 'Pen down' : 'Pen up'}
                color="#a78bfa"
              />
              <ActionChip
                on={headlights}
                onPress={() => setHeadlights((h) => !h)}
                icon={<Icons.zap size={14} color={headlights ? C.warn : C.text2} />}
                label="Lights"
                color={C.warn}
              />
              <ActionChip
                onPress={handleHold}
                icon={<Icons.pause size={14} color={C.text2} />}
                label="Hold"
                color={C.accent}
              />
              <ActionChip
                onPress={() => push('camera')}
                icon={<Icons.cam size={14} color={C.text2} />}
                label="Camera"
                color={C.text2}
              />
            </View>
          </Card>
        </View>

        {/* E-stop */}
        <View style={styles.section}>
          <Pressable
            onPress={emergency ? handleClearEStop : handleEStop}
            style={({ pressed }) => [
              styles.estop,
              emergency ? styles.estopClear : styles.estopActive,
              { opacity: pressed ? 0.85 : 1 },
            ]}
          >
            {emergency ? (
              <>
                <Icons.refresh size={18} color="#3a2906" />
                <Text style={[styles.estopText, { color: '#3a2906' }]}>Clear E-Stop &amp; Resume</Text>
              </>
            ) : (
              <>
                <Icons.warn size={18} color="#3a0a14" />
                <Text style={[styles.estopText, { color: '#3a0a14' }]}>Emergency Stop</Text>
              </>
            )}
          </Pressable>
          <Text style={styles.estopHint}>
            Cuts motors instantly · hold for hardware kill (3s)
          </Text>
        </View>
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: C.bg },
  scroll: { flex: 1 },
  content: { paddingBottom: 100 },
  trailingRow: { flexDirection: 'row', gap: 8 },
  attitudeRow: {
    flexDirection: 'row',
    gap: 10,
    paddingHorizontal: 16,
    paddingBottom: 12,
  },
  attitudeCard: { flex: 1.3 },
  headingCard: { flex: 1 },
  sectionLabel: {
    fontSize: 10,
    color: C.text3,
    textTransform: 'uppercase',
    letterSpacing: 0.8,
    fontWeight: '600',
    marginBottom: 6,
  },
  attitudeStats: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginTop: 8,
  },
  attitudeStat: {
    fontSize: 11,
    color: C.text3,
  },
  statsRow: {
    flexDirection: 'row',
    gap: 6,
    paddingHorizontal: 16,
    paddingBottom: 12,
  },
  section: { paddingHorizontal: 16, paddingBottom: 14 },
  motorHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 10,
  },
  motorSubtitle: { fontSize: 11, color: C.text3 },
  motorGrid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 8,
  },
  joystickRow: {
    flexDirection: 'row',
    gap: 12,
    paddingHorizontal: 16,
    paddingBottom: 14,
  },
  joystickCell: { flex: 1 },
  chips: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  chip: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 6,
    paddingHorizontal: 12,
    paddingVertical: 7,
    borderRadius: 9999,
    borderWidth: 1,
  },
  chipLabel: { fontSize: 12, fontWeight: '600' },
  estop: {
    width: '100%',
    padding: 16,
    borderRadius: 16,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    gap: 10,
  },
  estopActive: {
    backgroundColor: C.danger,
    shadowColor: C.danger,
    shadowOpacity: 0.4,
    shadowRadius: 12,
    shadowOffset: { width: 0, height: 4 },
  },
  estopClear: {
    backgroundColor: C.warn,
    shadowColor: C.warn,
    shadowOpacity: 0.4,
    shadowRadius: 12,
    shadowOffset: { width: 0, height: 4 },
  },
  estopText: {
    fontWeight: '800',
    fontSize: 16,
    letterSpacing: 0.5,
    textTransform: 'uppercase',
  },
  estopHint: {
    textAlign: 'center',
    fontSize: 11,
    color: C.text3,
    marginTop: 6,
  },
});
