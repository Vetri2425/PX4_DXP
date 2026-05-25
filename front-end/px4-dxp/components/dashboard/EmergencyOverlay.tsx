// components/dashboard/EmergencyOverlay.tsx
import React from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { C } from '../../theme/colors';
import { Icons } from '../icons';
import { Btn } from '../ui/Btn';
import { useUiStore } from '../../stores/useUiStore';

export function EmergencyOverlay() {
  const { emergency, clearEStop } = useUiStore();

  if (!emergency) return null;

  return (
    <View style={styles.overlay}>
      <View style={styles.iconWrap}>
        <Icons.warn size={18} color={C.danger} />
      </View>
      <View style={styles.text}>
        <Text style={styles.title}>Emergency stop active</Text>
        <Text style={styles.subtitle}>Motors disarmed · pen lifted · holding position</Text>
      </View>
      <Btn variant="secondary" size="sm" onPress={clearEStop}>
        Clear
      </Btn>
    </View>
  );
}

const styles = StyleSheet.create({
  overlay: {
    position: 'absolute',
    top: 64,
    left: 16,
    right: 16,
    zIndex: 40,
    padding: 12,
    borderRadius: 14,
    backgroundColor: `${C.danger}22`,
    borderWidth: 1,
    borderColor: `${C.danger}66`,
    flexDirection: 'row',
    alignItems: 'center',
    gap: 10,
  },
  iconWrap: {
    width: 32,
    height: 32,
    borderRadius: 10,
    backgroundColor: C.danger,
    alignItems: 'center',
    justifyContent: 'center',
    flexShrink: 0,
  },
  text: {
    flex: 1,
    minWidth: 0,
  },
  title: {
    fontWeight: '700',
    fontSize: 13,
    color: C.danger,
  },
  subtitle: {
    fontSize: 11,
    color: C.text2,
    marginTop: 1,
  },
});
