// components/ui/Bar.tsx
import React from 'react';
import { View, StyleSheet, ViewStyle } from 'react-native';
import { C } from '../../theme/colors';

interface BarProps {
  /** 0-100 */
  value: number;
  color?: string;
  trackColor?: string;
  height?: number;
  style?: ViewStyle;
}

export function Bar({
  value,
  color = C.accent,
  trackColor = 'rgba(255,255,255,0.07)',
  height = 4,
  style,
}: BarProps) {
  const pct = Math.min(100, Math.max(0, value));
  return (
    <View
      style={[
        styles.track,
        { backgroundColor: trackColor, height, borderRadius: height / 2 },
        style,
      ]}
    >
      <View
        style={[
          styles.fill,
          {
            width: `${pct}%`,
            backgroundColor: color,
            height,
            borderRadius: height / 2,
          },
        ]}
      />
    </View>
  );
}

const styles = StyleSheet.create({
  track: {
    width: '100%',
    overflow: 'hidden',
  },
  fill: {},
});
