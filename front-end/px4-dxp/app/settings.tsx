// app/settings.tsx
import React, { useState } from 'react';
import { View, Text, TextInput, ScrollView, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { router } from 'expo-router';
import { C } from '../theme/colors';
import { AppBar } from '../components/ui/AppBar';
import { Card } from '../components/ui/Card';
import { Btn } from '../components/ui/Btn';
import { SectionHeader } from '../components/ui/SectionHeader';
import { IconBtn } from '../components/ui/IconBtn';
import { Icons } from '../components/icons';
import { useConnectionStore } from '../stores/useConnectionStore';

export default function SettingsScreen() {
  const { activeRoverUrl, setBaseUrl } = useConnectionStore();
  const [url, setUrl] = useState(activeRoverUrl);

  return (
    <SafeAreaView style={styles.safeArea} edges={['top']}>
      <AppBar
        title="Settings"
        subtitle="Account · units · API"
        leading={<IconBtn icon={<Icons.chevL size={18} color={C.text2} />} onPress={() => router.back()} />}
      />
      <ScrollView contentContainerStyle={styles.content}>
        <SectionHeader title="Connection" />
        <View style={styles.section}>
          <Card pad={14}>
            <Text style={styles.label}>Rover base URL</Text>
            <TextInput
              value={url}
              onChangeText={setUrl}
              style={styles.input}
              placeholder="http://192.168.1.102:5001"
              placeholderTextColor={C.text3}
              autoCapitalize="none"
              keyboardType="url"
            />
            <Btn
              variant="primary"
              size="sm"
              style={styles.saveBtn}
              onPress={() => setBaseUrl(url)}
            >
              Save
            </Btn>
          </Card>
        </View>

        <SectionHeader title="Display" />
        <View style={styles.section}>
          <Card pad={14}>
            <Text style={styles.placeholder}>Units, theme, and display preferences</Text>
            <Text style={styles.placeholderSub}>Coming soon</Text>
          </Card>
        </View>
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: C.bg },
  content: { paddingBottom: 100 },
  section: { paddingHorizontal: 16, paddingBottom: 4 },
  label: { fontSize: 12, color: C.text2, fontWeight: '500', marginBottom: 8 },
  input: {
    padding: 10,
    borderRadius: 10,
    backgroundColor: C.card2,
    borderWidth: 1,
    borderColor: C.line2,
    color: C.text,
    fontSize: 13,
    marginBottom: 10,
  },
  saveBtn: { alignSelf: 'flex-start' },
  placeholder: { fontSize: 14, color: C.text2, fontWeight: '500' },
  placeholderSub: { fontSize: 12, color: C.text3, marginTop: 4 },
});
