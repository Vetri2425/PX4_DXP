// components/ui/Btn.tsx
import React from 'react';
import { Pressable, Text, StyleSheet, View, ViewStyle, TextStyle } from 'react-native';
import { C } from '../../theme/colors';

type Variant = 'primary' | 'secondary' | 'accentGhost' | 'danger' | 'warn' | 'ghost';
type Size = 'sm' | 'md' | 'lg';

interface BtnProps {
  children?: React.ReactNode;
  onPress?: () => void;
  variant?: Variant;
  size?: Size;
  icon?: React.ReactNode;
  disabled?: boolean;
  style?: ViewStyle;
}

const VARIANT_STYLES: Record<Variant, { bg: string; border: string; text: string }> = {
  primary: { bg: C.accent, border: C.accent, text: '#06202a' },
  secondary: { bg: 'rgba(255,255,255,0.06)', border: C.line2, text: C.text2 },
  accentGhost: { bg: `${C.accent}15`, border: `${C.accent}40`, text: C.accent },
  danger: { bg: `${C.danger}20`, border: `${C.danger}50`, text: C.danger },
  warn: { bg: `${C.warn}20`, border: `${C.warn}50`, text: C.warn },
  ghost: { bg: 'transparent', border: 'transparent', text: C.text2 },
};

const SIZE_STYLES: Record<Size, { px: number; py: number; fs: number; br: number }> = {
  sm: { px: 10, py: 5, fs: 12, br: 8 },
  md: { px: 14, py: 8, fs: 13, br: 10 },
  lg: { px: 18, py: 11, fs: 15, br: 12 },
};

export function Btn({
  children,
  onPress,
  variant = 'secondary',
  size = 'md',
  icon,
  disabled,
  style,
}: BtnProps) {
  const v = VARIANT_STYLES[variant];
  const s = SIZE_STYLES[size];

  return (
    <Pressable
      onPress={onPress}
      disabled={disabled}
      style={({ pressed }) => [
        styles.base,
        {
          backgroundColor: v.bg,
          borderColor: v.border,
          paddingHorizontal: s.px,
          paddingVertical: s.py,
          borderRadius: s.br,
          opacity: disabled ? 0.4 : pressed ? 0.75 : 1,
        },
        style,
      ]}
    >
      <View style={styles.inner}>
        {icon && <View style={children ? styles.iconGap : undefined}>{icon}</View>}
        {children ? (
          <Text style={[styles.label, { color: v.text, fontSize: s.fs }]}>
            {children}
          </Text>
        ) : null}
      </View>
    </Pressable>
  );
}

const styles = StyleSheet.create({
  base: {
    borderWidth: 1,
    alignSelf: 'flex-start',
  },
  inner: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  iconGap: {
    marginRight: 5,
  },
  label: {
    fontWeight: '600',
    letterSpacing: 0.2,
  },
});
