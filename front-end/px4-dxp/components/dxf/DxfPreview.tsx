// components/dxf/DxfPreview.tsx
import React from 'react';
import Svg, { Path } from 'react-native-svg';

interface DxfPreviewProps {
  seed?: string;
  size?: number;
}

/** Deterministic pseudo-drawing preview from a seed string. Identical algorithm to the web prototype. */
export function DxfPreview({ seed = '0', size = 100 }: DxfPreviewProps) {
  const hash = [...seed].reduce((a, c) => (a * 31 + c.charCodeAt(0)) >>> 0, 7);
  const rand = (i: number) => (((hash >> (i * 3)) & 0xff) / 255);

  const paths = Array.from({ length: 6 }).map((_, i) => {
    const a = rand(i) * 100;
    const b = rand(i + 2) * 100;
    const c = rand(i + 4) * 100;
    return `M${a * 0.4 + 10},${20 + i * 8} Q${b * 0.6 + 10},${10 + i * 9} ${c * 0.5 + 15},${30 + i * 7}`;
  });

  return (
    <Svg viewBox="0 0 100 100" width={size} height={size}>
      {paths.map((d, i) => (
        <Path
          key={i}
          d={d}
          stroke="#0a0d12"
          strokeWidth={1.4}
          fill="none"
          strokeLinecap="round"
        />
      ))}
    </Svg>
  );
}
