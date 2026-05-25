// app/(tabs)/draw.tsx
import { View, Text, StyleSheet } from 'react-native';
import { C } from '../../theme/colors';

export default function DrawScreen() {
  return (
    <View style={styles.container}>
      <Text style={styles.title}>New Drawing</Text>
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
  },
});