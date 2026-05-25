// components/dashboard/ConnectionBadge.tsx
import React, { useState } from 'react';
import { View, Text, Pressable, StyleSheet } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { C } from '../../theme/colors';
import { Dot } from '../ui/Dot';
import { useConnectionStore } from '../../stores/useConnectionStore';

export function ConnectionBadge() {
  const { backendConnected, backendError, activeRoverUrl } = useConnectionStore();
  const [showTooltip, setShowTooltip] = useState(false);
  const insets = useSafeAreaInsets();

  let color: string;
  let label: string;

  if (backendError) {
    color = C.danger;
    label = 'error';
  } else if (backendConnected) {
    color = C.good;
    label = 'live';
  } else {
    color = C.text3;
    label = 'offline';
  }

  return (
    <View style={[styles.container, { top: insets.top + 8 }]}>
      <Pressable
        onPress={() => setShowTooltip(!showTooltip)}
        style={[
          styles.badge,
          {
            backgroundColor: `${color}1a`,
            borderColor: `${color}33`,
          },
        ]}
      >
        <Dot color={color} size={6} pulse={backendConnected && !backendError} />
        <Text style={[styles.label, { color }]}>{label}</Text>
      </Pressable>

      {showTooltip && (
        <Pressable style={styles.tooltipOverlay} onPress={() => setShowTooltip(false)}>
          <View style={styles.tooltip}>
            <Text style={styles.tooltipText}>
              {backendError
                ? `Error: ${backendError}`
                : backendConnected
                  ? `Connected · ${activeRoverUrl}`
                  : 'Offline · mock data active'}
            </Text>
          </View>
        </Pressable>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    position: 'absolute',
    right: 12,
    zIndex: 35,
    alignItems: 'flex-end',
  },
  badge: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 5,
    paddingHorizontal: 9,
    paddingVertical: 4,
    borderRadius: 9999,
    borderWidth: 1,
  },
  label: {
    fontSize: 10,
    fontWeight: '600',
    letterSpacing: 0.3,
  },
  tooltipOverlay: {
    position: 'absolute',
    top: 30,
    right: 0,
  },
  tooltip: {
    padding: 10,
    borderRadius: 8,
    backgroundColor: C.card,
    borderWidth: 1,
    borderColor: C.line,
    maxWidth: 220,
    shadowColor: '#000',
    shadowOpacity: 0.5,
    shadowRadius: 8,
    shadowOffset: { width: 0, height: 4 },
  },
  tooltipText: {
    fontSize: 11,
    color: C.text2,
  },
});
