// components/dashboard/QuickActions.tsx
import React from 'react';
import { View, Text, Pressable, StyleSheet } from 'react-native';
import { router } from 'expo-router';
import { C } from '../../theme/colors';
import { Icons } from '../icons';

const ACTIONS = [
  {
    key: 'drive',
    icon: 'drive' as const,
    title: 'Manual drive',
    sub: 'Joystick · keys',
    color: C.accent,
    tab: 'drive' as const,
  },
  {
    key: 'draw',
    icon: 'draw' as const,
    title: 'New drawing',
    sub: 'SVG · canvas',
    color: '#a78bfa',
    tab: 'draw' as const,
  },
  {
    key: 'map',
    icon: 'map' as const,
    title: 'Plan mission',
    sub: 'Waypoints · trace',
    color: C.accent2,
    tab: 'map' as const,
  },
  {
    key: 'cam',
    icon: 'cam' as const,
    title: 'Live camera',
    sub: '1080p · 30 fps',
    color: C.warn,
    tab: null,
    screen: 'camera' as const,
  },
];

const TAB_ROUTES: Record<string, string> = {
  drive: '/(tabs)/drive',
  draw: '/(tabs)/draw',
  map: '/(tabs)/map',
};

const SCREEN_ROUTES: Record<string, string> = {
  camera: '/camera',
};

export function QuickActions() {
  return (
    <View style={styles.grid}>
      {ACTIONS.map((q) => {
        const Ic = Icons[q.icon];
        return (
          <Pressable
            key={q.key}
            onPress={() => {
              if (q.tab) router.push(TAB_ROUTES[q.tab] as never);
              else if (q.screen) router.push(SCREEN_ROUTES[q.screen] as never);
            }}
            style={({ pressed }) => [styles.card, { opacity: pressed ? 0.8 : 1 }]}
          >
            <View style={[styles.iconWrap, { backgroundColor: `${q.color}1c` }]}>
              <Ic size={20} color={q.color} />
            </View>
            <Text style={styles.title}>{q.title}</Text>
            <Text style={styles.sub}>{q.sub}</Text>
          </Pressable>
        );
      })}
    </View>
  );
}

const styles = StyleSheet.create({
  grid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 10,
    paddingHorizontal: 16,
  },
  card: {
    width: '47%',
    padding: 14,
    backgroundColor: C.card,
    borderWidth: 1,
    borderColor: C.line,
    borderRadius: 16,
    gap: 8,
  },
  iconWrap: {
    width: 38,
    height: 38,
    borderRadius: 10,
    alignItems: 'center',
    justifyContent: 'center',
  },
  title: {
    fontSize: 14,
    fontWeight: '600',
    color: C.text,
  },
  sub: {
    fontSize: 11,
    color: C.text3,
    marginTop: -4,
  },
});
