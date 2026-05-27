// components/dashboard/RoverHeroCard.tsx
import React, { useMemo } from 'react';
import { View, Text, StyleSheet, Pressable } from 'react-native';
import Svg, { Line, Path, Rect, Circle, Defs, LinearGradient, Stop } from 'react-native-svg';
import { useShallow } from 'zustand/react/shallow';
import { C } from '../../theme/colors';
import { Card } from '../ui/Card';
import { Dot } from '../ui/Dot';
import { Pill } from '../ui/Pill';
import { Icons } from '../icons';
import { useTelemetryStore } from '../../stores/useTelemetryStore';
import { useUiStore } from '../../stores/useUiStore';
import { useConnectionStore } from '../../stores/useConnectionStore';
import { useMissionStore } from '../../stores/useMissionStore';

/** #6 — show "—" for any value that hasn't been populated by a real telemetry frame */
function fmtNum(v: number, fallback: number, digits = 0): string {
  if (v === fallback) return '—';
  return digits > 0 ? v.toFixed(digits) : String(Math.round(v));
}

const MOCK_BATTERY = 78;
const MOCK_SATS = 14;
const MOCK_RSSI = -54;
const MOCK_HEADING = 124;

const SVG_GRID = (
  <>
    {Array.from({ length: 8 }).map((_, i) => (
      <Line key={`h${i}`} x1={i * 45} x2={i * 45} y1={0} y2={140} stroke="rgba(255,255,255,0.04)" strokeWidth={1} />
    ))}
    {Array.from({ length: 5 }).map((_, i) => (
      <Line key={`v${i}`} x1={0} x2={320} y1={i * 32} y2={i * 32} stroke="rgba(255,255,255,0.04)" strokeWidth={1} />
    ))}
  </>
);

export const RoverHeroCard = React.memo(function RoverHeroCard() {
  const { battery, sats, rssi, heading } = useTelemetryStore(
    useShallow((s) => ({ battery: s.battery, sats: s.sats, rssi: s.rssi, heading: s.heading }))
  );
  const { armed, emergency } = useUiStore(
    useShallow((s) => ({ armed: s.armed, emergency: s.emergency }))
  );
  const { backendConnected, activeRoverUrl } = useConnectionStore(
    useShallow((s) => ({ backendConnected: s.backendConnected, activeRoverUrl: s.activeRoverUrl }))
  );
  const missionMode = useMissionStore((s) => s.missionMode);

  // #6 — derive a human-readable location label from the URL
  const roverLabel = activeRoverUrl.replace(/^https?:\/\//, '').replace(/:5001$/, '');

  // #6 — show mock-data badge when disconnected; real badge when live
  const isLive = backendConnected;

  const quickStats = useMemo(() => [
    {
      l: 'BAT',
      v: fmtNum(battery, MOCK_BATTERY),
      u: isLive ? '%' : '',
      color: battery > 25 ? C.good : C.danger,
      icon: <Icons.battery size={11} color={battery > 25 ? C.good : C.danger} />,
    },
    {
      l: 'SAT',
      v: fmtNum(sats, MOCK_SATS),
      u: '',
      color: C.accent,
      icon: <Icons.satellite size={11} color={C.accent} />,
    },
    {
      l: 'RSSI',
      v: fmtNum(rssi, MOCK_RSSI),
      u: isLive ? 'dBm' : '',
      color: C.text,
      icon: <Icons.wifi size={11} color={C.text3} />,
    },
    {
      l: 'MODE',
      v: missionMode.toUpperCase(),
      u: '',
      color: '#a78bfa',
      icon: <Icons.zap size={11} color="#a78bfa" />,
    },
  ], [battery, sats, rssi, missionMode, isLive]);

  return (
    <Card
      pad={16}
      style={{ backgroundColor: '#182234', borderColor: `${C.accent}26` }}
    >
      {/* Header row */}
      <View style={styles.headerRow}>
        <View>
          <View style={styles.statusRow}>
            <Dot color={isLive ? C.good : C.danger} size={6} pulse={isLive} />
            {/* #6 — show real host/IP instead of hardcoded "Studio A" */}
            <Text style={styles.statusText}>
              {isLive ? `Connected · ${roverLabel}` : 'Disconnected'}
            </Text>
            {isLive ? (
              <Pill color={C.accent} dim>
                <Dot color={C.accent} size={4} pulse={false} />
                <Text style={[styles.pillLabel, { color: C.accent }]}> LIVE</Text>
              </Pill>
            ) : (
              <Pill color={C.text3} dim>
                <Dot color={C.text3} size={4} pulse={false} />
                <Text style={[styles.pillLabel, { color: C.text3 }]}> MOCK</Text>
              </Pill>
            )}
          </View>
          {/* #6 — rover name: show "—" until a telemetry frame names it */}
          <Text style={styles.roverName}>DXP-01</Text>
          <Text style={styles.roverSub}>PX4 v1.16.2 · ROS 2 Humble · Domain 42</Text>
        </View>
        <Pressable style={styles.switchBtn}>
          <Icons.fleet size={13} color={C.text2} />
          <Text style={styles.switchText}>Switch</Text>
        </Pressable>
      </View>

      {/* Map/trace view */}
      <View style={styles.mapContainer}>
        <Svg viewBox="0 0 320 140" width="100%" height="100%" preserveAspectRatio="xMidYMid slice">
          <Defs>
            <LinearGradient id="trail" x1="0" x2="1">
              <Stop offset="0" stopColor={C.accent} stopOpacity="0" />
              <Stop offset="1" stopColor={C.accent} stopOpacity="1" />
            </LinearGradient>
          </Defs>
          {SVG_GRID}
          <Path
            d="M40,110 C80,80 110,100 140,70 S200,40 240,55 S290,95 280,115"
            stroke="url(#trail)"
            strokeWidth={2.2}
            fill="none"
            strokeLinecap="round"
          />
          <Rect
            x={267} y={106} width={26} height={18} rx={3}
            fill="#1a2738" stroke={C.accent} strokeWidth={1.5}
            transform={`rotate(${heading - 90} 280 115)`}
          />
          <Circle cx={280} cy={115} r={3} fill={C.accent} />
          <Circle cx={280} cy={115} r={22} fill="none" stroke={C.accent} strokeOpacity={0.3} strokeDasharray="2 3" />
        </Svg>

        {/* Overlay pills */}
        <View style={styles.mapOverlayTop}>
          <Pill color={C.accent}>
            <Dot color={C.accent} size={6} pulse={false} />
            <Text style={[styles.pillLabel, { color: C.accent }]}> {missionMode.toUpperCase()}</Text>
          </Pill>
          {armed && (
            <Pill color={C.warn} dim>
              <Icons.unlock size={11} color={C.warn} />
              <Text style={[styles.pillLabel, { color: C.warn }]}> ARMED</Text>
            </Pill>
          )}
          {emergency && (
            <Pill color={C.danger}>
              <Icons.warn size={11} color={C.danger} />
              <Text style={[styles.pillLabel, { color: C.danger }]}> E-STOP</Text>
            </Pill>
          )}
        </View>

        {/* #6 — coordinate: show "—" until GPS is live */}
        <View style={styles.mapOverlayBottom}>
          <Text style={styles.coordText}>
            {isLive ? `HDG ${Math.round(heading)}°` : '— ·  —'}
          </Text>
        </View>
      </View>

      {/* Quick stats grid */}
      <View style={styles.statsGrid}>
        {quickStats.map((s) => (
          <View key={s.l} style={styles.statTile}>
            <View style={styles.statHeader}>
              <Text style={styles.statLabel}>{s.l}</Text>
              {s.icon}
            </View>
            <View style={styles.statValueRow}>
              <Text style={[styles.statValue, { color: s.color }]}>{s.v}</Text>
              {s.u ? <Text style={styles.statUnit}>{s.u}</Text> : null}
            </View>
          </View>
        ))}
      </View>
    </Card>
  );
});

const styles = StyleSheet.create({
  headerRow: {
    flexDirection: 'row',
    alignItems: 'flex-start',
    justifyContent: 'space-between',
    marginBottom: 12,
  },
  statusRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 6,
    marginBottom: 4,
  },
  statusText: {
    fontSize: 11,
    color: C.text3,
    textTransform: 'uppercase',
    letterSpacing: 0.8,
    fontWeight: '600',
  },
  pillLabel: {
    fontSize: 10,
    fontWeight: '600',
    letterSpacing: 0.4,
  },
  roverName: {
    fontSize: 22,
    fontWeight: '700',
    color: C.text,
    letterSpacing: -0.4,
  },
  roverSub: {
    fontSize: 12,
    color: C.text2,
    marginTop: 2,
  },
  switchBtn: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 5,
    paddingHorizontal: 10,
    paddingVertical: 6,
    borderRadius: 9999,
    borderWidth: 1,
    borderColor: C.line2,
    backgroundColor: C.card2,
  },
  switchText: {
    fontSize: 12,
    color: C.text2,
    fontWeight: '500',
  },
  mapContainer: {
    height: 140,
    borderRadius: 12,
    overflow: 'hidden',
    backgroundColor: '#0c1320',
    position: 'relative',
    marginBottom: 12,
  },
  mapOverlayTop: {
    position: 'absolute',
    top: 10,
    left: 12,
    flexDirection: 'row',
    gap: 6,
    alignItems: 'center',
  },
  mapOverlayBottom: {
    position: 'absolute',
    bottom: 10,
    right: 12,
  },
  coordText: {
    fontSize: 11,
    color: C.text3,
  },
  statsGrid: {
    flexDirection: 'row',
    gap: 6,
  },
  statTile: {
    flex: 1,
    padding: 10,
    borderRadius: 10,
    backgroundColor: 'rgba(0,0,0,0.25)',
    borderWidth: 1,
    borderColor: 'rgba(255,255,255,0.05)',
  },
  statHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 2,
  },
  statLabel: {
    fontSize: 10,
    color: C.text3,
    fontWeight: '600',
    letterSpacing: 0.5,
  },
  statValueRow: {
    flexDirection: 'row',
    alignItems: 'flex-end',
    gap: 1,
  },
  statValue: {
    fontSize: 15,
    fontWeight: '600',
  },
  statUnit: {
    fontSize: 9,
    color: C.text3,
    marginBottom: 1,
  },
});
