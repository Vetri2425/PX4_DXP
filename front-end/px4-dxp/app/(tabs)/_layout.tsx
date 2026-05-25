// app/(tabs)/_layout.tsx
import { Tabs } from 'expo-router';
import { C } from '../../theme/colors';

const TAB_CONFIG = [
  { name: 'index', title: 'Home' },
  { name: 'map', title: 'Map' },
  { name: 'draw', title: 'Draw' },
  { name: 'drive', title: 'Drive' },
  { name: 'more', title: 'More' },
] as const;

export default function TabLayout() {
  return (
    <Tabs
      screenOptions={{
        headerShown: false,
        tabBarStyle: {
          backgroundColor: 'rgba(20,25,35,0.78)',
          borderTopColor: C.line,
          height: 70,
          paddingBottom: 8,
        },
        tabBarActiveTintColor: C.accent,
        tabBarInactiveTintColor: C.text3,
        tabBarLabelStyle: { fontSize: 11, fontWeight: '600' },
      }}
    >
      {TAB_CONFIG.map((tab) => (
        <Tabs.Screen
          key={tab.name}
          name={tab.name}
          options={{ title: tab.title }}
        />
      ))}
    </Tabs>
  );
}