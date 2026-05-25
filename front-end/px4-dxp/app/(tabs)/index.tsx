// app/(tabs)/index.tsx
import { View, Text, StyleSheet } from 'react-native';
import { C } from '../../theme/colors';

export default function HomeScreen() {
  return (
    <View style={styles.container}>
      <Text style={styles.title}>PX.4_DXp</Text>
      <Text style={styles.subtitle}>Drawing Rover Workbench</Text>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: C.bg,
    alignItems: 'center',
    justifyContent: 'center',
  },
  title: {
    color: C.text,
    fontSize: 28,
    fontWeight: '700',
    letterSpacing: -0.5,
  },
  subtitle: {
    color: C.text2,
    fontSize: 14,
    marginTop: 4,
  },
});