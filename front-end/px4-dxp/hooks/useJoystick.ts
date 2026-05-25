// hooks/useJoystick.ts
import { useSharedValue, useAnimatedStyle, withTiming } from 'react-native-reanimated';
import { Gesture } from 'react-native-gesture-handler';

export function useJoystick(onChange?: (x: number, y: number) => void) {
  const knobX = useSharedValue(0);
  const knobY = useSharedValue(0);
  const isActive = useSharedValue(0);

  // Max translation in px — matches the container size math in Joystick.tsx
  const MAX_DIST = 52;

  const gesture = Gesture.Pan()
    .onStart(() => {
      isActive.value = 1;
    })
    .onUpdate((event) => {
      const dx = Math.max(-MAX_DIST, Math.min(MAX_DIST, event.translationX));
      const dy = Math.max(-MAX_DIST, Math.min(MAX_DIST, event.translationY));
      knobX.value = dx;
      knobY.value = dy;
      // Normalise to -1..1 and invert Y (up = positive)
      const nx = dx / MAX_DIST;
      const ny = -(dy / MAX_DIST);
      if (onChange) {
        'worklet';
        // Call on JS thread via runOnJS in component
      }
      // Store normalised values for the JS callback (used in component)
      (knobX as { _nx?: number })._nx = nx;
      (knobY as { _ny?: number })._ny = ny;
    })
    .onEnd(() => {
      isActive.value = 0;
      knobX.value = withTiming(0, { duration: 150 });
      knobY.value = withTiming(0, { duration: 150 });
      if (onChange) onChange(0, 0);
    });

  const animatedStyle = useAnimatedStyle(() => ({
    transform: [
      { translateX: knobX.value },
      { translateY: knobY.value },
    ],
  }));

  return { gesture, animatedStyle, isActive, knobX, knobY, MAX_DIST };
}
