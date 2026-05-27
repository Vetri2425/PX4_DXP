// components/ui/Dot.tsx
import React, { useEffect, useRef, useMemo } from 'react';
import { Animated, StyleSheet } from 'react-native';
import { C } from '../../theme/colors';

interface DotProps {
  color?: string;
  size?: number;
  pulse?: boolean;
}

export const Dot = React.memo(function Dot({ color = C.good, size = 8, pulse = true }: DotProps) {
  const opacity = useRef(new Animated.Value(1)).current;

  useEffect(() => {
    if (!pulse) {
      opacity.setValue(1);
      return;
    }
    const anim = Animated.loop(
      Animated.sequence([
        Animated.timing(opacity, { toValue: 0.3, duration: 800, useNativeDriver: true }),
        Animated.timing(opacity, { toValue: 1, duration: 800, useNativeDriver: true }),
      ])
    );
    anim.start();
    return () => anim.stop();
  }, [pulse, opacity]);

  const baseDotStyle = useMemo(
    () => [styles.dot, { width: size, height: size, borderRadius: size / 2, backgroundColor: color }],
    [size, color]
  );

  return (
    <Animated.View
      style={pulse ? [baseDotStyle, { opacity }] : baseDotStyle}
    />
  );
});

const styles = StyleSheet.create({
  dot: {
    flexShrink: 0,
  },
});
