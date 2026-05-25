// components/ui/Card.tsx
import React from 'react';
import { View, Pressable, StyleSheet, ViewStyle } from 'react-native';
import { C } from '../../theme/colors';

interface CardProps {
  children: React.ReactNode;
  pad?: number;
  accent?: boolean;
  onPress?: () => void;
  style?: ViewStyle;
}

export function Card({ children, pad = 16, accent, onPress, style }: CardProps) {
  const containerStyle: ViewStyle[] = [
    styles.card,
    { padding: pad },
    accent ? styles.accentCard : null,
    style ?? null,
  ].filter(Boolean) as ViewStyle[];

  if (onPress) {
    return (
      <Pressable
        onPress={onPress}
        style={({ pressed }) => [
          ...containerStyle,
          pressed ? styles.pressed : null,
        ]}
      >
        {children}
      </Pressable>
    );
  }
  return <View style={containerStyle}>{children}</View>;
}

const styles = StyleSheet.create({
  card: {
    backgroundColor: C.card,
    borderWidth: 1,
    borderColor: C.line,
    borderRadius: 18,
  },
  accentCard: {
    shadowColor: C.accent,
    shadowOpacity: 0.1,
    shadowRadius: 8,
    shadowOffset: { width: 0, height: 2 },
  },
  pressed: {
    opacity: 0.85,
  },
});
