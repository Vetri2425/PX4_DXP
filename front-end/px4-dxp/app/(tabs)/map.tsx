// app/(tabs)/map.tsx
import React, { useState, useRef } from 'react';
import { View, Text, Pressable, StyleSheet } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import MapView, { Marker, Polyline, LongPressEvent, MapPressEvent } from 'react-native-maps';
import { C } from '../../theme/colors';
import { AppBar } from '../../components/ui/AppBar';
import { Card } from '../../components/ui/Card';
import { Btn } from '../../components/ui/Btn';
import { IconBtn } from '../../components/ui/IconBtn';
import { Icons } from '../../components/icons';
import { TelemetryChip } from '../../components/map/TelemetryChip';
import { WpInspector } from '../../components/map/WpInspector';
import { useMissionStore } from '../../stores/useMissionStore';
import type { Waypoint, WaypointType } from '../../types/mission';

const WP_COLORS: Record<WaypointType, string> = {
  start: C.good,
  'pen-down': C.accent,
  'pen-up': C.text3,
  turn: C.warn,
  end: C.danger,
};

// Initial region: Rover's operating site
const INITIAL_REGION = {
  latitude: 13.07203780,
  longitude: 80.26194903,
  latitudeDelta: 0.001,
  longitudeDelta: 0.001,
};

function missionLengthMeters(wps: Waypoint[]): number {
  if (wps.length < 2) return 0;
  let d = 0;
  for (let i = 1; i < wps.length; i++) {
    const dlat = (wps[i].latitude - wps[i - 1].latitude) * 111320;
    const dlon = (wps[i].longitude - wps[i - 1].longitude) * 111320 * Math.cos(wps[i].latitude * Math.PI / 180);
    d += Math.sqrt(dlat * dlat + dlon * dlon);
  }
  return d;
}

export default function MissionScreen() {
  const { waypoints, setWaypoints } = useMissionStore();
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [mapType, setMapType] = useState<'satellite' | 'standard'>('satellite');
  const mapRef = useRef<MapView>(null);

  const selectedWp = waypoints.find((w) => w.id === selectedId) ?? null;
  const selectedIdx = waypoints.findIndex((w) => w.id === selectedId);

  const handleLongPress = (e: LongPressEvent) => {
    const { latitude, longitude } = e.nativeEvent.coordinate;
    const id = `wp_${Date.now()}`;
    const newWp: Waypoint = { id, latitude, longitude, type: 'pen-down' };
    setWaypoints((prev) => [...prev, newWp]);
    setSelectedId(id);
  };

  const handleMapPress = (_e: MapPressEvent) => {
    if (selectedId) setSelectedId(null);
  };

  const deleteSelected = () => {
    setWaypoints((prev) => prev.filter((w) => w.id !== selectedId));
    setSelectedId(null);
  };

  const updateType = (type: WaypointType) => {
    setWaypoints((prev) =>
      prev.map((w) => (w.id === selectedId ? { ...w, type } : w))
    );
  };

  const updatePosition = (id: string, lat: number, lng: number) => {
    setWaypoints((prev) =>
      prev.map((w) => (w.id === id ? { ...w, latitude: lat, longitude: lng } : w))
    );
  };

  const toggleMapType = () => setMapType((t) => (t === 'satellite' ? 'standard' : 'satellite'));

  return (
    <SafeAreaView style={styles.safeArea} edges={['top']}>
      <View style={styles.container}>
        <AppBar
          title="Mission"
          subtitle={`${waypoints.length} waypoints`}
          trailing={
            <View style={styles.trailingRow}>
              <IconBtn icon={<Icons.target size={18} color={C.text2} />} onPress={() => mapRef.current?.animateToRegion(INITIAL_REGION)} />
              <IconBtn icon={<Icons.layers size={18} color={C.text2} />} onPress={toggleMapType} />
            </View>
          }
        />

        {/* Map */}
        <View style={styles.mapContainer}>
          <MapView
            ref={mapRef}
            style={StyleSheet.absoluteFill}
            mapType={mapType}
            initialRegion={INITIAL_REGION}
            onLongPress={handleLongPress}
            onPress={handleMapPress}
            showsUserLocation={false}
            showsCompass
          >
            {/* Route polyline */}
            {waypoints.length >= 2 && (
              <Polyline
                coordinates={waypoints.map((w) => ({
                  latitude: w.latitude,
                  longitude: w.longitude,
                }))}
                strokeColor={C.accent}
                strokeWidth={2}
                lineDashPattern={[6, 4]}
              />
            )}

            {/* Waypoint markers */}
            {waypoints.map((wp, i) => (
              <Marker
                key={wp.id}
                coordinate={{ latitude: wp.latitude, longitude: wp.longitude }}
                anchor={{ x: 0.5, y: 0.5 }}
                draggable
                onPress={() => setSelectedId(wp.id)}
                onDragEnd={(e) => updatePosition(wp.id, e.nativeEvent.coordinate.latitude, e.nativeEvent.coordinate.longitude)}
              >
                <View
                  style={[
                    styles.marker,
                    {
                      borderColor: WP_COLORS[wp.type],
                      backgroundColor: selectedId === wp.id
                        ? `${WP_COLORS[wp.type]}33`
                        : '#0a0d12',
                    },
                  ]}
                >
                  <Text style={[styles.markerText, { color: WP_COLORS[wp.type] }]}>
                    {i + 1}
                  </Text>
                </View>
              </Marker>
            ))}
          </MapView>

          {/* Telemetry overlay */}
          <TelemetryChip />

          {/* Map type toggle */}
          <Pressable onPress={toggleMapType} style={styles.mapTypeBtn}>
            <Text style={styles.mapTypeBtnText}>
              {mapType === 'satellite' ? 'SAT' : 'MAP'}
            </Text>
          </Pressable>

          {/* Hint pill */}
          {!selectedId && (
            <View style={styles.hintWrap}>
              <View style={styles.hintPill}>
                <Text style={styles.hintText}>Long-press to add · drag to move · tap to edit</Text>
              </View>
            </View>
          )}

          {/* Waypoint inspector */}
          {selectedId && selectedWp && (
            <WpInspector
              wp={selectedWp}
              index={selectedIdx}
              onClose={() => setSelectedId(null)}
              onDelete={deleteSelected}
              onType={updateType}
            />
          )}
        </View>

        {/* Bottom action sheet */}
        <View style={styles.bottomSheet}>
          <Card pad={12}>
            <View style={styles.sheetRow}>
              <View>
                <Text style={styles.sheetLabel}>Mission</Text>
                <Text style={styles.sheetValue}>
                  {waypoints.length} waypoints · {missionLengthMeters(waypoints).toFixed(1)} m
                </Text>
              </View>
              <View style={styles.sheetBtns}>
                <Btn variant="secondary" size="sm" icon={<Icons.upload size={13} color={C.text2} />}>
                  Upload
                </Btn>
                <Btn variant="primary" size="sm" icon={<Icons.play size={13} color="#06202a" />}>
                  Run
                </Btn>
              </View>
            </View>
          </Card>
        </View>
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: C.bg },
  container: { flex: 1 },
  trailingRow: { flexDirection: 'row', gap: 8 },
  mapContainer: {
    flex: 1,
    marginHorizontal: 16,
    borderRadius: 20,
    overflow: 'hidden',
    borderWidth: 1,
    borderColor: C.line2,
    position: 'relative',
  },
  marker: {
    width: 28,
    height: 28,
    borderRadius: 14,
    borderWidth: 2,
    alignItems: 'center',
    justifyContent: 'center',
  },
  markerText: {
    fontSize: 11,
    fontWeight: '700',
  },
  mapTypeBtn: {
    position: 'absolute',
    top: 12,
    right: 12,
    backgroundColor: 'rgba(20,25,35,0.7)',
    borderWidth: 1,
    borderColor: C.line2,
    paddingHorizontal: 10,
    paddingVertical: 6,
    borderRadius: 9999,
  },
  mapTypeBtnText: {
    fontSize: 11,
    fontWeight: '600',
    color: C.text2,
    letterSpacing: 0.6,
  },
  hintWrap: {
    position: 'absolute',
    bottom: 12,
    left: 0,
    right: 0,
    alignItems: 'center',
  },
  hintPill: {
    backgroundColor: 'rgba(20,25,35,0.78)',
    borderWidth: 1,
    borderColor: C.line2,
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 9999,
  },
  hintText: { fontSize: 11, color: C.text2, fontWeight: '500' },
  bottomSheet: { padding: 14, paddingBottom: 100 },
  sheetRow: { flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between' },
  sheetLabel: {
    fontSize: 11,
    color: C.text3,
    textTransform: 'uppercase',
    letterSpacing: 0.7,
    fontWeight: '600',
  },
  sheetValue: { fontSize: 14, fontWeight: '600', color: C.text, marginTop: 2 },
  sheetBtns: { flexDirection: 'row', gap: 6 },
});

