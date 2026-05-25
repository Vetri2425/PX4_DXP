// components/drive/HeadingDisc.tsx
import React from 'react';
import { View, StyleSheet } from 'react-native';
import Svg, { Line, Text as SvgText, Path, G } from 'react-native-svg';
import { C } from '../../theme/colors';

interface HeadingDiscProps {
  heading: number;
  size?: number;
}

const CARDINALS: [string, number][] = [
  ['N', 0],
  ['E', 90],
  ['S', 180],
  ['W', 270],
];

export function HeadingDisc({ heading, size = 140 }: HeadingDiscProps) {
  return (
    <View
      style={[
        styles.container,
        { width: size, height: size, borderRadius: size / 2 },
      ]}
    >
      {/* Rotating tick disc */}
      <Svg
        viewBox="0 0 100 100"
        width={size}
        height={size}
        style={[StyleSheet.absoluteFill, { transform: [{ rotate: `${-heading}deg` }] }]}
      >
        {/* Cardinal labels */}
        {CARDINALS.map(([l, a]) => (
          <G key={l} transform={`rotate(${a} 50 50)`}>
            <SvgText
              x={50}
              y={18}
              textAnchor="middle"
              fontSize={9}
              fill={l === 'N' ? C.accent : C.text2}
              fontWeight="700"
            >
              {l}
            </SvgText>
            <Line
              x1={50}
              y1={22}
              x2={50}
              y2={26}
              stroke={l === 'N' ? C.accent : C.text2}
              strokeWidth={1.5}
            />
          </G>
        ))}
        {/* Minor ticks every 10° */}
        {Array.from({ length: 36 }).map((_, i) => {
          if (i % 9 === 0) return null;
          return (
            <Line
              key={i}
              x1={50}
              y1={9}
              x2={50}
              y2={i % 3 === 0 ? 13 : 11}
              stroke="rgba(255,255,255,0.3)"
              strokeWidth={0.8}
              transform={`rotate(${i * 10} 50 50)`}
            />
          );
        })}
      </Svg>

      {/* Fixed overlay: pointer + heading text */}
      <Svg
        viewBox="0 0 100 100"
        width={size}
        height={size}
        style={StyleSheet.absoluteFill}
      >
        {/* Top pointer */}
        <Path d="M50,6 L46,14 L54,14 Z" fill={C.accent} />
        {/* Heading value */}
        <SvgText
          x={50}
          y={52}
          textAnchor="middle"
          fontSize={14}
          fontWeight="700"
          fill={C.text}
        >
          {String(Math.round(heading)).padStart(3, '0')}°
        </SvgText>
        <SvgText
          x={50}
          y={64}
          textAnchor="middle"
          fontSize={7}
          fill={C.text3}
          letterSpacing={1}
        >
          MAG
        </SvgText>
      </Svg>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    alignSelf: 'center',
    backgroundColor: '#0a0d12',
    borderWidth: 1.5,
    borderColor: 'rgba(255,255,255,0.15)',
    overflow: 'hidden',
  },
});

