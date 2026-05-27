// components/drive/AttitudeIndicator.tsx
import React, { useMemo } from 'react';
import { View, StyleSheet } from 'react-native';
import Svg, { Path, Circle, Line, G } from 'react-native-svg';
import { C } from '../../theme/colors';

interface AttitudeIndicatorProps {
  pitch: number;
  roll: number;
  size?: number;
}

const ROLL_SCALE_TICKS = (
  <>
    {[0, -30, -60, 60, 30].map((deg) => (
      <G key={deg} transform={`rotate(${deg} 50 50)`}>
        <Line x1={50} y1={6} x2={50} y2={Math.abs(deg) % 30 === 0 ? 11 : 9}
          stroke="rgba(255,255,255,0.5)" strokeWidth={1.2} />
      </G>
    ))}
  </>
);

export const AttitudeIndicator = React.memo(function AttitudeIndicator({ pitch, roll, size = 140 }: AttitudeIndicatorProps) {
  const pitchOffset = (pitch / 90) * size * 0.55;

  const pitchTicks = useMemo(() =>
    [-30, -20, -10, 10, 20, 30].map((p) => {
      const tickOffset = -(p * (size * 0.55) / 90) + pitchOffset;
      const w = Math.abs(p) === 10 ? 38 : Math.abs(p) === 20 ? 28 : 20;
      return (
        <View key={p} style={[styles.pitchTick, {
          width: w, top: size / 2 - tickOffset,
          left: '50%', marginLeft: -(w / 2),
        }]} />
      );
    }),
  [pitchOffset, size]);

  return (
    <View
      style={[
        styles.container,
        { width: size, height: size, borderRadius: size / 2 },
      ]}
    >
      {/* Sky/ground ball — rotates with roll, translates with pitch */}
      <View
        style={[
          StyleSheet.absoluteFill,
          { transform: [{ rotate: `${-roll}deg` }] },
          styles.ballClip,
        ]}
      >
        {/* Extended sky */}
        <View
          style={[
            styles.sky,
            { transform: [{ translateY: pitchOffset }] },
          ]}
        />
        {/* Extended ground */}
        <View
          style={[
            styles.ground,
            { transform: [{ translateY: pitchOffset }] },
          ]}
        />
        {/* Horizon line */}
        <View
          style={[
            styles.horizon,
            { top: '50%', transform: [{ translateY: pitchOffset }] },
          ]}
        />
        {/* Pitch ladder ticks */}
        {pitchTicks}
      </View>

      {/* SVG overlay: reticle, roll triangle, scale marks */}
      <Svg
        viewBox="0 0 100 100"
        width={size}
        height={size}
        style={StyleSheet.absoluteFill}
      >
        {/* Centre reticle */}
        <Path
          d="M30,50 L42,50 M58,50 L70,50 M50,42 L50,46"
          stroke={C.accent}
          strokeWidth={2}
          strokeLinecap="round"
          fill="none"
        />
        <Circle cx={50} cy={50} r={2} fill={C.accent} />

        {/* Roll pointer triangle (rotates) */}
        <G transform={`rotate(${-roll} 50 50)`}>
          <Path d="M50,10 L46,17 L54,17 Z" fill={C.accent} />
        </G>

        {/* Fixed roll scale ticks */}
        {ROLL_SCALE_TICKS}
      </Svg>
    </View>
  );
});

const styles = StyleSheet.create({
  container: {
    alignSelf: 'center',
    backgroundColor: '#0a0d12',
    borderWidth: 1.5,
    borderColor: 'rgba(255,255,255,0.15)',
    overflow: 'hidden',
  },
  ballClip: {
    overflow: 'hidden',
  },
  sky: {
    position: 'absolute',
    left: '-50%',
    right: '-50%',
    top: '-100%',
    bottom: '50%',
    backgroundColor: '#155e75',
  },
  ground: {
    position: 'absolute',
    left: '-50%',
    right: '-50%',
    top: '50%',
    bottom: '-100%',
    backgroundColor: '#451a03',
  },
  horizon: {
    position: 'absolute',
    left: '-50%',
    right: '-50%',
    height: 2,
    backgroundColor: 'rgba(255,255,255,0.85)',
  },
  pitchTick: {
    position: 'absolute',
    height: 1.5,
    backgroundColor: 'rgba(255,255,255,0.5)',
  },
});
