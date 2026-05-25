// components/ui/Pill.tsx
import React from 'react';
import { View, Text, StyleSheet, ViewStyle } from 'react-native';
import { C } from '../../theme/colors';

interface PillProps {
  children: React.ReactNode;
  color?: string;
  dim?: boolean;
  style?: ViewStyle;
}

export function Pill({ children, color = C.accent, dim = false, style }: PillProps) {
  return (
    <View
      style={[
        styles.pill,
        {
          backgroundColor: dim ? `${color}1a` : `${color}26`,
          borderColor: dim ? `${color}33` : `${color}4d`,
        },
        style,
      ]}
    >
      {typeof children === 'string' ? (
        <Text style={[styles.text, { color }]}>{children}</Text>
      ) : (
        <View style={styles.content}>{children}</View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  pill: {
    flexDirection: 'row',
    alignItems: 'center',
    paddingHorizontal: 7,
    paddingVertical: 3,
    borderRadius: 9999,
    borderWidth: 1,
  },
  content: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 4,
  },
  text: {
    fontSize: 10,
    fontWeight: '600',
    letterSpacing: 0.4,
  },
});
