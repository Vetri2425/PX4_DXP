// app/_layout.tsx
import { useEffect, useRef, useState } from 'react';
import { Stack } from 'expo-router';
import { StatusBar } from 'expo-status-bar';
import * as SystemUI from 'expo-system-ui';
import { GestureHandlerRootView } from 'react-native-gesture-handler';
import { SafeAreaProvider } from 'react-native-safe-area-context';
import { ActivityIndicator, View } from 'react-native';
import { C } from '../theme/colors';
import { initApi } from '../services/api';
import { disconnectSocket } from '../services/socket';
import { useConnectionStore } from '../stores/useConnectionStore';

// #21 — protect deep links so router.back() and similar always have a healthy stack
// When a user deep-links directly to /connect (or future shortcuts), this ensures
// the (tabs) root is synthesized as the initial screen for back navigation.
export const unstable_settings = {
  initialRouteName: '(tabs)',
};

export default function RootLayout() {
  // #14 — cancelled flag prevents setState after unmount / double-effect
  const cancelled = useRef(false);
  // Track boot state: true once init finishes
  const [booted, setBooted] = useState(false);

  useEffect(() => {
    cancelled.current = false;

    (async () => {
      try {
        await SystemUI.setBackgroundColorAsync(C.bg);
        await initApi();
        // Hydrate last-used rover URL (from previous successful connection)
        await useConnectionStore.getState().hydrate();
        // NOTE: Do NOT initSocket() during boot — it creates a stale socket
        // that interferes with explicit connection attempts from the connect
        // screen. Connection only happens when the user taps a rover.
      } catch {
        // Non-fatal
      } finally {
        if (!cancelled.current) {
          setBooted(true);
        }
      }
    })();

    return () => {
      cancelled.current = true;
      disconnectSocket();
    };
  }, []);

  // Show a minimal loading spinner while booting
  if (!booted) {
    return (
      <View style={{ flex: 1, backgroundColor: C.bg, alignItems: 'center', justifyContent: 'center' }}>
        <StatusBar style="light" />
        <ActivityIndicator size="large" color={C.accent} />
      </View>
    );
  }

  return (
    <GestureHandlerRootView style={{ flex: 1 }}>
      <SafeAreaProvider>
        <StatusBar style="light" />
        <Stack
          screenOptions={{
            headerShown: false,
            contentStyle: { backgroundColor: C.bg },
            animation: 'slide_from_right',
          }}
        >
          <Stack.Screen name="connect" />
          <Stack.Screen name="camera" />
          <Stack.Screen name="ros-nodes" />
          <Stack.Screen name="px4-params" />
          <Stack.Screen name="calibrate" />
          <Stack.Screen name="logs" />
          <Stack.Screen name="firmware" />
          <Stack.Screen name="fleet" />
          <Stack.Screen name="settings" />
        </Stack>
      </SafeAreaProvider>
    </GestureHandlerRootView>
  );
}
