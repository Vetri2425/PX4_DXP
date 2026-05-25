// components/map/TelemetryChip.tsx
import React from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { C } from '../../theme/colors';
import { useTelemetryStore } from '../../stores/useTelemetryStore';

function Chip({ children }: { children: React.ReactNode }) {
  return (
    <View style={styles.chip}>
      <Text style={styles.chipText}>{children}</Text>
    </View>
  );
}

export function TelemetryChip() {
  const { fix, sats, hdop, heading, speed } = useTelemetryStore();
  return (
    <View style={styles.container}>
      <Chip>{fix} · {sats} sat · HDOP {hdop.toFixed(1)}</Chip>
      <Chip>HDG {Math.round(heading)}°</Chip>
      <Chip>{speed.toFixed(2)} m/s</Chip>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    position: 'absolute',
    top: 12,
    left: 12,
    gap: 5,
  },
  chip: {
    backgroundColor: 'rgba(20,25,35,0.75)',
    borderWidth: 1,
    borderColor: C.line2,
    paddingHorizontal: 9,
    paddingVertical: 4,
    borderRadius: 9999,
  },
  chipText: {
    fontSize: 11,
    color: C.text,
    fontWeight: '500',
  },
});
