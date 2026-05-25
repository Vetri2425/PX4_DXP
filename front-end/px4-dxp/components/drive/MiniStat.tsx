// components/drive/MiniStat.tsx
import React from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { C } from '../../theme/colors';

interface MiniStatProps {
  label: string;
  value: string | number;
  unit?: string;
  color?: string;
}

export function MiniStat({ label, value, unit, color = C.text }: MiniStatProps) {
  return (
    <View style={styles.container}>
      <Text style={styles.label}>{label}</Text>
      <View style={styles.valueRow}>
        <Text style={[styles.value, { color }]}>{value}</Text>
        {unit ? <Text style={styles.unit}>{unit}</Text> : null}
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    padding: 10,
    backgroundColor: C.card2,
    borderRadius: 10,
    borderWidth: 1,
    borderColor: C.line,
  },
  label: {
    fontSize: 10,
    color: C.text3,
    fontWeight: '600',
    letterSpacing: 0.5,
    marginBottom: 2,
  },
  valueRow: {
    flexDirection: 'row',
    alignItems: 'flex-end',
    gap: 2,
  },
  value: {
    fontSize: 16,
    fontWeight: '600',
  },
  unit: {
    fontSize: 9,
    color: C.text3,
    marginBottom: 1,
  },
});
