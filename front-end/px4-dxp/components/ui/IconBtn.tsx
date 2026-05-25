// components/ui/IconBtn.tsx
import React from 'react';
import { Pressable, View, Text, StyleSheet } from 'react-native';
import { C } from '../../theme/colors';

interface IconBtnProps {
  icon: React.ReactNode;
  onPress?: () => void;
  badge?: number;
  accent?: boolean;
  size?: number;
  disabled?: boolean;
}

export function IconBtn({ icon, onPress, badge, accent, size = 36, disabled }: IconBtnProps) {
  return (
    <Pressable
      onPress={onPress}
      disabled={disabled}
      style={({ pressed }) => [
        styles.btn,
        {
          width: size,
          height: size,
          borderRadius: size / 4,
          backgroundColor: accent
            ? `${C.accent}22`
            : 'rgba(255,255,255,0.06)',
          borderColor: accent ? `${C.accent}44` : C.line,
          opacity: disabled ? 0.4 : pressed ? 0.7 : 1,
        },
      ]}
    >
      <View style={styles.iconWrap}>{icon}</View>
      {badge != null && badge > 0 && (
        <View style={styles.badge}>
          <Text style={styles.badgeText}>{badge > 99 ? '99+' : badge}</Text>
        </View>
      )}
    </Pressable>
  );
}

const styles = StyleSheet.create({
  btn: {
    alignItems: 'center',
    justifyContent: 'center',
    borderWidth: 1,
    position: 'relative',
  },
  iconWrap: {
    alignItems: 'center',
    justifyContent: 'center',
  },
  badge: {
    position: 'absolute',
    top: -4,
    right: -4,
    minWidth: 16,
    height: 16,
    borderRadius: 8,
    backgroundColor: C.danger,
    alignItems: 'center',
    justifyContent: 'center',
    paddingHorizontal: 3,
  },
  badgeText: {
    color: '#fff',
    fontSize: 9,
    fontWeight: '700',
  },
});
