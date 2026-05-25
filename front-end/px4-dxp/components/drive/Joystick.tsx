// components/drive/Joystick.tsx
import React, { useCallback, useRef } from 'react';
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
/** #10 — throttle: minimum ms between JS-thread onChange calls (≤30 Hz = ~33 ms) */
const NOTIFY_INTERVAL_MS = 33;

export function Joystick({ label, hint, disabled, onChange }: JoystickProps) {
  // Animated knob position (UI thread)
  const knobX = useSharedValue(0);
  const knobY = useSharedValue(0);
  const isActive = useSharedValue(false);

  // #10 — last-notify timestamp, kept as a plain ref (JS thread)
  const lastNotifyMs = useRef(0);

  // #9 — track disabled state in a shared value so onUpdate can short-circuit on the UI thread
  const disabledSV = useSharedValue(disabled ? 1 : 0);
  // Keep the shared value in sync with the prop
  React.useEffect(() => {
    disabledSV.value = disabled ? 1 : 0;
    if (disabled) {
      // Snap knob back and zero output when mid-gesture
      knobX.value = withTiming(0, { duration: 100 });
      knobY.value = withTiming(0, { duration: 100 });
      runOnJS(notifyZero)();
    }
  }, [disabled]); // eslint-disable-line react-hooks/exhaustive-deps

  const notifyZero = useCallback(() => {
    onChange?.(0, 0);
  }, [onChange]);

  const notify = useCallback(
    (x: number, y: number) => {
      // #10 — throttle to NOTIFY_INTERVAL_MS on the JS thread
      const now = Date.now();
      if (now - lastNotifyMs.current < NOTIFY_INTERVAL_MS) return;
      lastNotifyMs.current = now;
      onChange?.(x, y);
    },
    [onChange]
  );

  const gesture = Gesture.Pan()
    .enabled(!disabled)
    .onStart(() => {
      if (disabledSV.value) return; // #9 — extra guard
      isActive.value = true;
    })
    .onUpdate((e) => {
      // #9 — short-circuit if disabled flipped during the gesture
      if (disabledSV.value) {
        knobX.value = withTiming(0, { duration: 100 });
        knobY.value = withTiming(0, { duration: 100 });
        runOnJS(notifyZero)();
        return;
      }
      const dx = Math.max(-MAX_DIST, Math.min(MAX_DIST, e.translationX));
      const dy = Math.max(-MAX_DIST, Math.min(MAX_DIST, e.translationY));
      knobX.value = dx;
      knobY.value = dy;
      // #10 — runOnJS calls are throttled inside notify()
      runOnJS(notify)(dx / MAX_DIST, -(dy / MAX_DIST));
    })
    .onEnd(() => {
      isActive.value = false;
      knobX.value = withTiming(0, { duration: 150 });
      knobY.value = withTiming(0, { duration: 150 });
      runOnJS(notifyZero)();
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

  return (
    <Card pad={12} style={disabled ? styles.disabled : undefined}>
      <View style={styles.header}>
        <Text style={styles.label}>{label}</Text>
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
  disabled: { opacity: 0.5 },
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
  disabledBase: { backgroundColor: '#181f2c' },
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
