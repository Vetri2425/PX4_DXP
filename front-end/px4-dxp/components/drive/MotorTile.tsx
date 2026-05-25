// components/drive/MotorTile.tsx
import React from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { C } from '../../theme/colors';
import { Bar } from '../ui/Bar';

interface MotorTileProps {
  label: string;
  value: number; // 0-100
}

export function MotorTile({ label, value }: MotorTileProps) {
  const color = value > 90 ? C.warn : C.accent;
  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <Text style={styles.label}>{label}</Text>
        <Text style={[styles.value, { color }]}>{Math.round(value)}%</Text>
      </View>
      <Bar value={value} color={color} height={4} style={styles.bar} />
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    padding: 8,
    borderRadius: 10,
    backgroundColor: 'rgba(255,255,255,0.025)',
    borderWidth: 1,
    borderColor: C.line,
  },
  header: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 4,
  },
  label: {
    fontSize: 10,
    color: C.text3,
    fontWeight: '700',
    letterSpacing: 0.5,
  },
  value: {
    fontSize: 11,
    fontWeight: '600',
  },
  bar: {
    marginTop: 0,
  },
});
