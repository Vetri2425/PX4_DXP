// components/map/WpInspector.tsx
import React from 'react';
import { View, Text, Pressable, StyleSheet } from 'react-native';
import { C } from '../../theme/colors';
import { Icons } from '../icons';
import { IconBtn } from '../ui/IconBtn';
import type { Waypoint, WaypointType } from '../../types/mission';

const WP_TYPES: WaypointType[] = ['start', 'pen-down', 'pen-up', 'turn', 'end'];

interface WpInspectorProps {
  wp: Waypoint;
  index: number;
  onClose: () => void;
  onDelete: () => void;
  onType: (type: WaypointType) => void;
}

export function WpInspector({ wp, index, onClose, onDelete, onType }: WpInspectorProps) {
  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <View>
          <Text style={styles.title}>Waypoint #{index + 1}</Text>
          <Text style={styles.coords}>
            {wp.latitude.toFixed(6)}, {wp.longitude.toFixed(6)}
          </Text>
        </View>
        <View style={styles.actions}>
          <IconBtn
            size={32}
            icon={<Icons.trash size={14} color={C.danger} />}
            onPress={onDelete}
          />
          <IconBtn
            size={32}
            icon={<Icons.close size={14} color={C.text2} />}
            onPress={onClose}
          />
        </View>
      </View>
      <View style={styles.typeRow}>
        {WP_TYPES.map((tp) => (
          <Pressable
            key={tp}
            onPress={() => onType(tp)}
            style={[
              styles.typeBtn,
              tp === wp.type && styles.typeBtnActive,
            ]}
          >
            <Text style={[styles.typeBtnText, tp === wp.type && styles.typeBtnTextActive]}>
              {tp}
            </Text>
          </Pressable>
        ))}
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    position: 'absolute',
    bottom: 12,
    left: 12,
    right: 12,
    backgroundColor: 'rgba(15,20,28,0.92)',
    borderWidth: 1,
    borderColor: C.line2,
    borderRadius: 16,
    padding: 14,
  },
  header: {
    flexDirection: 'row',
    alignItems: 'flex-start',
    justifyContent: 'space-between',
    marginBottom: 10,
  },
  title: {
    fontSize: 11,
    color: C.text3,
    textTransform: 'uppercase',
    letterSpacing: 0.8,
    fontWeight: '600',
  },
  coords: {
    fontSize: 13,
    color: C.text2,
    marginTop: 2,
  },
  actions: {
    flexDirection: 'row',
    gap: 6,
  },
  typeRow: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 6,
  },
  typeBtn: {
    paddingHorizontal: 10,
    paddingVertical: 6,
    borderRadius: 9999,
    borderWidth: 1,
    borderColor: C.line,
    backgroundColor: C.card2,
  },
  typeBtnActive: {
    borderColor: `${C.accent}66`,
    backgroundColor: `${C.accent}26`,
  },
  typeBtnText: {
    fontSize: 11,
    fontWeight: '600',
    color: C.text2,
    letterSpacing: 0.3,
  },
  typeBtnTextActive: {
    color: C.accent,
  },
});
