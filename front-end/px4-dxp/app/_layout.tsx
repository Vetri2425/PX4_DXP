// app/_layout.tsx
import { useEffect } from 'react';
import { Stack } from 'expo-router';
import { StatusBar } from 'expo-status-bar';
import { GestureHandlerRootView } from 'react-native-gesture-handler';
import { SafeAreaProvider } from 'react-native-safe-area-context';
import { C } from '../theme/colors';
import { initApi } from '../services/api';
import { initSocket, disconnectSocket } from '../services/socket';

export default function RootLayout() {
  useEffect(() => {
    (async () => {
      await initApi();
      await initSocket();
    })();
    return () => {
      disconnectSocket();
    };
  }, []);

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
          {/* Sub-screens */}
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
