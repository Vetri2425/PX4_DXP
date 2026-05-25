// components/ui/Stat.tsx
import React from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { C } from '../../theme/colors';

interface StatProps {
  label: string;
  value: string | number;
  unit?: string;
  color?: string;
  icon?: React.ReactNode;
}

export function Stat({ label, value, unit, color = C.text, icon }: StatProps) {
  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <Text style={styles.label}>{label}</Text>
        {icon && <View style={[styles.iconWrap, { }]}>{icon}</View>}
      </View>
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
    borderRadius: 10,
    backgroundColor: 'rgba(0,0,0,0.25)',
    borderWidth: 1,
    borderColor: 'rgba(255,255,255,0.05)',
  },
  header: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  label: {
    fontSize: 10,
    color: C.text3,
    fontWeight: '600',
    letterSpacing: 0.5,
    textTransform: 'uppercase',
  },
  iconWrap: {
    opacity: 0.9,
  },
  valueRow: {
    flexDirection: 'row',
    alignItems: 'flex-end',
    marginTop: 2,
    gap: 2,
  },
  value: {
    fontSize: 15,
    fontWeight: '600',
    letterSpacing: -0.3,
  },
  unit: {
    fontSize: 10,
    color: C.text3,
    marginBottom: 1,
  },
});
