// components/drive/Joystick.tsx
import React, { useCallback } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import Animated, {
  useSharedValue,
  useAnimatedStyle,
  withTiming,
  runOnJS,
} from 'react-native-reanimated';
import { Gesture, GestureDetector } from 'react-native-gesture-handler';
import Svg, { Circle, Line } from 'react-native-svg';
import { C } from '../../theme/colors';
import { Card } from '../ui/Card';

interface JoystickProps {
  label: string;
  hint?: string;
  disabled?: boolean;
  onChange?: (x: number, y: number) => void;
}

const MAX_DIST = 52;

export function Joystick({ label, hint, disabled, onChange }: JoystickProps) {
  const knobX = useSharedValue(0);
  const knobY = useSharedValue(0);
  const isActive = useSharedValue(false);
  const normX = useSharedValue(0);
  const normY = useSharedValue(0);

  const notify = useCallback(
    (x: number, y: number) => {
      onChange?.(x, y);
    },
    [onChange]
  );

  const gesture = Gesture.Pan()
    .enabled(!disabled)
    .onStart(() => {
      isActive.value = true;
    })
    .onUpdate((e) => {
      const dx = Math.max(-MAX_DIST, Math.min(MAX_DIST, e.translationX));
      const dy = Math.max(-MAX_DIST, Math.min(MAX_DIST, e.translationY));
      knobX.value = dx;
      knobY.value = dy;
      normX.value = dx / MAX_DIST;
      normY.value = -(dy / MAX_DIST); // invert Y
      runOnJS(notify)(dx / MAX_DIST, -(dy / MAX_DIST));
    })
    .onEnd(() => {
      isActive.value = false;
      knobX.value = withTiming(0, { duration: 150 });
      knobY.value = withTiming(0, { duration: 150 });
      runOnJS(notify)(0, 0);
    });

  const knobStyle = useAnimatedStyle(() => ({
    transform: [
      { translateX: knobX.value },
      { translateY: knobY.value },
    ],
  }));

  const containerStyle = useAnimatedStyle(() => ({
    borderColor: isActive.value ? `${C.accent}66` : C.line2,
    shadowOpacity: isActive.value ? 0.4 : 0,
  }));

  const xDisplay = useAnimatedStyle(() => ({})); // purely visual

  return (
    <Card pad={12} style={disabled ? styles.disabled : undefined}>
      <View style={styles.header}>
        <Text style={styles.label}>{label}</Text>
        <Text style={styles.coords}>
          x  y
        </Text>
      </View>

      <GestureDetector gesture={gesture}>
        <Animated.View style={[styles.base, containerStyle, disabled ? styles.disabledBase : null]}>
          {/* Crosshair background */}
          <Svg
            viewBox="0 0 100 100"
            width="100%"
            height="100%"
            style={StyleSheet.absoluteFill}
          >
            <Circle cx={50} cy={50} r={32} fill="none" stroke="rgba(255,255,255,0.05)" strokeDasharray="2 3" />
            <Line x1={50} y1={20} x2={50} y2={80} stroke="rgba(255,255,255,0.04)" strokeWidth={1} />
            <Line x1={20} y1={50} x2={80} y2={50} stroke="rgba(255,255,255,0.04)" strokeWidth={1} />
          </Svg>

          {/* Animated knob */}
          <View style={styles.knobAnchor}>
            <Animated.View style={[styles.knob, knobStyle]} />
          </View>
        </Animated.View>
      </GestureDetector>

      {hint && <Text style={styles.hint}>{hint}</Text>}
    </Card>
  );
}

const styles = StyleSheet.create({
  disabled: {
    opacity: 0.5,
  },
  header: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 8,
  },
  label: {
    fontSize: 10,
    color: C.text3,
    textTransform: 'uppercase',
    letterSpacing: 0.8,
    fontWeight: '600',
  },
  coords: {
    fontSize: 10,
    color: C.text3,
  },
  base: {
    width: '100%',
    aspectRatio: 1,
    maxWidth: 140,
    alignSelf: 'center',
    borderRadius: 9999,
    backgroundColor: '#0a0d12',
    borderWidth: 1.5,
    overflow: 'hidden',
    position: 'relative',
    shadowColor: C.accent,
    shadowOffset: { width: 0, height: 0 },
    shadowRadius: 12,
  },
  disabledBase: {
    backgroundColor: '#181f2c',
  },
  knobAnchor: {
    position: 'absolute',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    alignItems: 'center',
    justifyContent: 'center',
  },
  knob: {
    width: 36,
    height: 36,
    borderRadius: 18,
    backgroundColor: '#0e7490',
    borderWidth: 1,
    borderColor: C.accent,
    shadowColor: C.accent,
    shadowOpacity: 0.4,
    shadowRadius: 8,
    shadowOffset: { width: 0, height: 4 },
  },
  hint: {
    marginTop: 8,
    textAlign: 'center',
    fontSize: 10,
    color: C.text3,
  },
});

