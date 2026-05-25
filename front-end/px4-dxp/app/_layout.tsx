// app/_layout.tsx
import { useEffect } from 'react';
import { Stack } from 'expo-router';
import { StatusBar } from 'expo-status-bar';
import { GestureHandlerRootView } from 'react-native-gesture-handler';
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
      <StatusBar style="light" />
      <Stack
        screenOptions={{
          headerShown: false,
          contentStyle: { backgroundColor: C.bg },
          animation: 'slide_from_right',
        }}
      />
    </GestureHandlerRootView>
  );
}
