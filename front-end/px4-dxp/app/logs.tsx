// app/logs.tsx — #20: reads from useUiStore.errorLog (live structured buffer)
import React, { useRef, useEffect } from 'react';
import { View, Text, FlatList, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { router } from 'expo-router';
import { C } from '../theme/colors';
import { AppBar } from '../components/ui/AppBar';
import { Card } from '../components/ui/Card';
import { IconBtn } from '../components/ui/IconBtn';
import { Icons } from '../components/icons';
import { useUiStore, type LogEntry } from '../stores/useUiStore';

const LEVEL_COLOR: Record<LogEntry['level'], string> = {
  INFO: C.good,
  WARN: C.warn,
  ERR: C.danger,
};

function formatTs(ms: number): string {
  const d = new Date(ms);
  const hh = String(d.getHours()).padStart(2, '0');
  const mm = String(d.getMinutes()).padStart(2, '0');
  const ss = String(d.getSeconds()).padStart(2, '0');
  const ms3 = String(d.getMilliseconds()).padStart(3, '0');
  return `${hh}:${mm}:${ss}.${ms3}`;
}

export default function LogsScreen() {
  const errorLog = useUiStore((s) => s.errorLog);
  const flatListRef = useRef<FlatList<LogEntry>>(null);

  // Auto-scroll to bottom on new entries
  useEffect(() => {
    flatListRef.current?.scrollToEnd({ animated: false });
  }, [errorLog.length]);

  return (
    <SafeAreaView style={styles.safeArea} edges={[]}>
      <AppBar
        title="Logs & Diagnostics"
        subtitle={`${errorLog.length} entries · Rosout · MAVLink · uORB`}
        leading={<IconBtn icon={<Icons.chevL size={18} color={C.text2} />} onPress={() => router.back()} />}
        trailing={<IconBtn icon={<Icons.download size={18} color={C.text2} />} />}
      />
      <View style={styles.logContainer}>
        <Card pad={0} style={styles.logCard}>
          <FlatList
            ref={flatListRef}
            data={errorLog}
            keyExtractor={(_, i) => String(i)}
            renderItem={({ item: entry }) => (
              <View style={styles.logLine}>
                <Text style={styles.logTs}>{formatTs(entry.ts)}</Text>
                <Text style={[styles.logLevel, { color: LEVEL_COLOR[entry.level] }]}>
                  {entry.level.padEnd(4)}
                </Text>
                <Text style={styles.logMsg} numberOfLines={3}>{entry.msg}</Text>
              </View>
            )}
            ListEmptyComponent={<Text style={styles.empty}>No log entries yet.</Text>}
            style={styles.logScroll}
            getItemLayout={(_data, index) => ({ length: 26, offset: 26 * index, index })}
            initialNumToRender={20}
            maxToRenderPerBatch={20}
            windowSize={5}
          />
        </Card>
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: C.bg },
  logContainer: { flex: 1, paddingHorizontal: 16, paddingBottom: 110 },
  logCard: { flex: 1, overflow: 'hidden' },
  logScroll: { flex: 1, backgroundColor: '#0a0d12' },
  logLine: {
    flexDirection: 'row',
    paddingHorizontal: 12,
    paddingVertical: 4,
    gap: 8,
    flexWrap: 'nowrap',
  },
  logTs: { fontSize: 10, color: C.text3, flexShrink: 0, width: 84 },
  logLevel: { fontSize: 10, fontWeight: '700', flexShrink: 0, width: 36 },
  logMsg: { fontSize: 11, color: C.text2, flexShrink: 1, flexWrap: 'wrap' },
  empty: { fontSize: 12, color: C.text3, padding: 16, textAlign: 'center' },
});
