// components/dashboard/SysDiagnostics.tsx
import React from 'react';
import { View, Text, Pressable, StyleSheet } from 'react-native';
import { C } from '../../theme/colors';
import { Card } from '../ui/Card';
import { Dot } from '../ui/Dot';
import { Btn } from '../ui/Btn';
import { router } from 'expo-router';
import { Icons } from '../icons';

interface SysTileProps {
  label: string;
  value: string | number;
  ok?: string;
  warn?: boolean;
}

function SysTile({ label, value, ok, warn }: SysTileProps) {
  const color = warn ? C.warn : ok ? C.good : C.text;
  return (
    <View style={styles.tile}>
      <Text style={styles.tileLabel}>{label}</Text>
      <View style={styles.tileValueRow}>
        <Dot color={color} size={6} pulse={!warn && !!ok} />
        <Text style={[styles.tileValue, { color }]}>{value}</Text>
      </View>
    </View>
  );
}

export function SysDiagnostics() {
  return (
    <View style={styles.container}>
      <Card pad={14}>
        <View style={styles.grid}>
          <SysTile label="ROS2 nodes" value="18/18" ok="18/18" />
          <SysTile label="uORB" value="245 Hz" ok="ok" />
          <SysTile label="EKF2" value="locked" ok="ok" />
          <SysTile label="Geofence" value="active" ok="3 zones" />
          <SysTile label="Pen" value="up" />
          <SysTile label="Storage" value="62%" warn />
        </View>
        <View style={styles.btnRow}>
          <Btn
            variant="secondary"
            size="sm"
            icon={<Icons.terminal size={13} color={C.text2} />}
            onPress={() => router.push('/logs')}
          >
            Logs
          </Btn>
          <Btn
            variant="secondary"
            size="sm"
            icon={<Icons.cpu size={13} color={C.text2} />}
            onPress={() => router.push('/ros-nodes')}
          >
            Nodes
          </Btn>
          <Btn
            variant="secondary"
            size="sm"
            icon={<Icons.sliders size={13} color={C.text2} />}
            onPress={() => router.push('/px4-params')}
          >
            Params
          </Btn>
        </View>
      </Card>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    paddingHorizontal: 16,
  },
  grid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 10,
    marginBottom: 12,
  },
  tile: {
    width: '30%',
    minWidth: 90,
    padding: 10,
    borderRadius: 10,
    backgroundColor: 'rgba(255,255,255,0.025)',
    borderWidth: 1,
    borderColor: C.line,
  },
  tileLabel: {
    fontSize: 10,
    color: C.text3,
    textTransform: 'uppercase',
    letterSpacing: 0.7,
    fontWeight: '600',
    marginBottom: 4,
  },
  tileValueRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 5,
  },
  tileValue: {
    fontSize: 13,
    fontWeight: '600',
  },
  btnRow: {
    flexDirection: 'row',
    gap: 8,
  },
});
