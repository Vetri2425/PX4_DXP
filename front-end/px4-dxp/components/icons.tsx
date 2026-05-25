// components/icons.tsx
import React from 'react';
import Svg, { Path, Circle, Rect, Line, Polyline, Polygon } from 'react-native-svg';
import type { SvgProps } from 'react-native-svg';

export interface IconProps extends SvgProps {
  size?: number;
  color?: string;
  strokeWidth?: number;
}

function Ico({
  size = 22,
  color = '#e6edf6',
  strokeWidth = 1.75,
  children,
  ...props
}: IconProps) {
  return (
    <Svg
      width={size}
      height={size}
      viewBox="0 0 24 24"
      fill="none"
      stroke={color}
      strokeWidth={strokeWidth}
      strokeLinecap="round"
      strokeLinejoin="round"
      {...props}
    >
      {children}
    </Svg>
  );
}

export const Icons = {
  home: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M3 11l9-7 9 7v9a2 2 0 0 1-2 2h-4v-7h-6v7H5a2 2 0 0 1-2-2z" />
    </Ico>
  ),
  map: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M9 4 3 7v13l6-3 6 3 6-3V4l-6 3-6-3z" />
      <Path d="M9 4v13" />
      <Path d="M15 7v13" />
    </Ico>
  ),
  drive: (p: IconProps) => (
    <Ico {...p}>
      <Circle cx={12} cy={12} r={9} />
      <Circle cx={12} cy={12} r={3} />
      <Path d="M12 3v3M12 18v3M3 12h3M18 12h3" />
    </Ico>
  ),
  draw: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M3 21l3-1 11-11-2-2L4 18l-1 3z" />
      <Path d="M14 6l4 4" />
      <Path d="M17 3l4 4-2 2-4-4z" />
    </Ico>
  ),
  more: (p: IconProps) => (
    <Ico {...p}>
      <Circle cx={5} cy={12} r={1.5} />
      <Circle cx={12} cy={12} r={1.5} />
      <Circle cx={19} cy={12} r={1.5} />
    </Ico>
  ),
  battery: (p: IconProps) => (
    <Ico {...p}>
      <Rect x={2} y={7} width={18} height={10} rx={2} />
      <Path d="M22 11v2" />
      <Path d="M6 11v2" strokeWidth={2} />
    </Ico>
  ),
  satellite: (p: IconProps) => (
    <Ico {...p}>
      <Path d="m13 7 4-4 3 3-4 4" />
      <Path d="m17 11-7 7" />
      <Path d="M8 19 5 22" />
      <Path d="m2 18 3-3" />
      <Circle cx={8} cy={15} r={3} />
    </Ico>
  ),
  wifi: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M5 12.55a11 11 0 0 1 14.08 0" />
      <Path d="M1.42 9a16 16 0 0 1 21.16 0" />
      <Path d="M8.53 16.11a6 6 0 0 1 6.95 0" />
      <Circle cx={12} cy={20} r={1} />
    </Ico>
  ),
  zap: (p: IconProps) => (
    <Ico {...p}>
      <Polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2" />
    </Ico>
  ),
  warn: (p: IconProps) => (
    <Ico {...p}>
      <Path d="m10.29 3.86-8.48 14.7A1 1 0 0 0 2.68 20H21.32a1 1 0 0 0 .87-1.5l-8.48-14.7a1 1 0 0 0-1.74 0z" />
      <Path d="M12 9v4" />
      <Path d="M12 17h.01" />
    </Ico>
  ),
  bell: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9" />
      <Path d="M13.73 21a2 2 0 0 1-3.46 0" />
    </Ico>
  ),
  cam: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M23 7 16 12 23 17z" />
      <Rect x={1} y={5} width={15} height={14} rx={2} />
    </Ico>
  ),
  menu: (p: IconProps) => (
    <Ico {...p}>
      <Line x1={3} y1={12} x2={21} y2={12} />
      <Line x1={3} y1={6} x2={21} y2={6} />
      <Line x1={3} y1={18} x2={21} y2={18} />
    </Ico>
  ),
  play: (p: IconProps) => (
    <Ico {...p}>
      <Polygon points="5 3 19 12 5 21 5 3" />
    </Ico>
  ),
  pause: (p: IconProps) => (
    <Ico {...p}>
      <Rect x={6} y={4} width={4} height={16} />
      <Rect x={14} y={4} width={4} height={16} />
    </Ico>
  ),
  stop: (p: IconProps) => (
    <Ico {...p}>
      <Rect x={3} y={3} width={18} height={18} rx={2} />
    </Ico>
  ),
  terminal: (p: IconProps) => (
    <Ico {...p}>
      <Polyline points="4 17 10 11 4 5" />
      <Line x1={12} y1={19} x2={20} y2={19} />
    </Ico>
  ),
  cpu: (p: IconProps) => (
    <Ico {...p}>
      <Rect x={4} y={4} width={16} height={16} rx={2} />
      <Rect x={9} y={9} width={6} height={6} />
      <Path d="M9 2v2M15 2v2M9 20v2M15 20v2M2 9h2M2 15h2M20 9h2M20 15h2" />
    </Ico>
  ),
  sliders: (p: IconProps) => (
    <Ico {...p}>
      <Line x1={4} y1={21} x2={4} y2={14} />
      <Line x1={4} y1={10} x2={4} y2={3} />
      <Line x1={12} y1={21} x2={12} y2={12} />
      <Line x1={12} y1={8} x2={12} y2={3} />
      <Line x1={20} y1={21} x2={20} y2={16} />
      <Line x1={20} y1={12} x2={20} y2={3} />
      <Line x1={1} y1={14} x2={7} y2={14} />
      <Line x1={9} y1={8} x2={15} y2={8} />
      <Line x1={17} y1={16} x2={23} y2={16} />
    </Ico>
  ),
  fleet: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M17 21v-2a4 4 0 0 0-4-4H5a4 4 0 0 0-4 4v2" />
      <Circle cx={9} cy={7} r={4} />
      <Path d="M23 21v-2a4 4 0 0 0-3-3.87" />
      <Path d="M16 3.13a4 4 0 0 1 0 7.75" />
    </Ico>
  ),
  unlock: (p: IconProps) => (
    <Ico {...p}>
      <Rect x={3} y={11} width={18} height={11} rx={2} />
      <Path d="M7 11V7a5 5 0 0 1 9.9-1" />
    </Ico>
  ),
  lock: (p: IconProps) => (
    <Ico {...p}>
      <Rect x={3} y={11} width={18} height={11} rx={2} />
      <Path d="M7 11V7a5 5 0 0 1 10 0v4" />
    </Ico>
  ),
  chevR: (p: IconProps) => (
    <Ico {...p}>
      <Polyline points="9 18 15 12 9 6" />
    </Ico>
  ),
  chevL: (p: IconProps) => (
    <Ico {...p}>
      <Polyline points="15 18 9 12 15 6" />
    </Ico>
  ),
  chevDown: (p: IconProps) => (
    <Ico {...p}>
      <Polyline points="6 9 12 15 18 9" />
    </Ico>
  ),
  settings: (p: IconProps) => (
    <Ico {...p}>
      <Circle cx={12} cy={12} r={3} />
      <Path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83-2.83l.06-.06A1.65 1.65 0 0 0 4.68 15a1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 2.83-2.83l.06.06A1.65 1.65 0 0 0 9 4.68a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 2.83l-.06.06A1.65 1.65 0 0 0 19.4 9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z" />
    </Ico>
  ),
  info: (p: IconProps) => (
    <Ico {...p}>
      <Circle cx={12} cy={12} r={10} />
      <Line x1={12} y1={16} x2={12} y2={12} />
      <Line x1={12} y1={8} x2={12} y2={8} />
    </Ico>
  ),
  close: (p: IconProps) => (
    <Ico {...p}>
      <Line x1={18} y1={6} x2={6} y2={18} />
      <Line x1={6} y1={6} x2={18} y2={18} />
    </Ico>
  ),
  check: (p: IconProps) => (
    <Ico {...p}>
      <Polyline points="20 6 9 17 4 12" />
    </Ico>
  ),
  plus: (p: IconProps) => (
    <Ico {...p}>
      <Line x1={12} y1={5} x2={12} y2={19} />
      <Line x1={5} y1={12} x2={19} y2={12} />
    </Ico>
  ),
  minus: (p: IconProps) => (
    <Ico {...p}>
      <Line x1={5} y1={12} x2={19} y2={12} />
    </Ico>
  ),
  refresh: (p: IconProps) => (
    <Ico {...p}>
      <Polyline points="23 4 23 10 17 10" />
      <Polyline points="1 20 1 14 7 14" />
      <Path d="M3.51 9a9 9 0 0 1 14.85-3.36L23 10M1 14l4.64 4.36A9 9 0 0 0 20.49 15" />
    </Ico>
  ),
  upload: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
      <Polyline points="17 8 12 3 7 8" />
      <Line x1={12} y1={3} x2={12} y2={15} />
    </Ico>
  ),
  download: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
      <Polyline points="7 10 12 15 17 10" />
      <Line x1={12} y1={15} x2={12} y2={3} />
    </Ico>
  ),
  link: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M10 13a5 5 0 0 0 7.54.54l3-3a5 5 0 0 0-7.07-7.07l-1.72 1.71" />
      <Path d="M14 11a5 5 0 0 0-7.54-.54l-3 3a5 5 0 0 0 7.07 7.07l1.71-1.71" />
    </Ico>
  ),
  layers: (p: IconProps) => (
    <Ico {...p}>
      <Polygon points="12 2 2 7 12 12 22 7 12 2" />
      <Polyline points="2 17 12 22 22 17" />
      <Polyline points="2 12 12 17 22 12" />
    </Ico>
  ),
  target: (p: IconProps) => (
    <Ico {...p}>
      <Circle cx={12} cy={12} r={10} />
      <Circle cx={12} cy={12} r={6} />
      <Circle cx={12} cy={12} r={2} />
    </Ico>
  ),
  compass: (p: IconProps) => (
    <Ico {...p}>
      <Circle cx={12} cy={12} r={10} />
      <Polygon points="16.24 7.76 14.12 14.12 7.76 16.24 9.88 9.88 16.24 7.76" />
    </Ico>
  ),
  navigation: (p: IconProps) => (
    <Ico {...p}>
      <Polygon points="3 11 22 2 13 21 11 13 3 11" />
    </Ico>
  ),
  flag: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M4 15s1-1 4-1 5 2 8 2 4-1 4-1V3s-1 1-4 1-5-2-8-2-4 1-4 1z" />
      <Line x1={4} y1={22} x2={4} y2={15} />
    </Ico>
  ),
  anchor: (p: IconProps) => (
    <Ico {...p}>
      <Circle cx={12} cy={5} r={3} />
      <Line x1={12} y1={22} x2={12} y2={8} />
      <Path d="M5 12H2a10 10 0 0 0 20 0h-3" />
    </Ico>
  ),
  eye: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z" />
      <Circle cx={12} cy={12} r={3} />
    </Ico>
  ),
  eyeOff: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19m-6.72-1.07a3 3 0 1 1-4.24-4.24" />
      <Line x1={1} y1={1} x2={23} y2={23} />
    </Ico>
  ),
  trash: (p: IconProps) => (
    <Ico {...p}>
      <Polyline points="3 6 5 6 21 6" />
      <Path d="M19 6l-1 14a2 2 0 0 1-2 2H8a2 2 0 0 1-2-2L5 6" />
      <Path d="M10 11v6" />
      <Path d="M14 11v6" />
      <Path d="M9 6V4a1 1 0 0 1 1-1h4a1 1 0 0 1 1 1v2" />
    </Ico>
  ),
  edit: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M11 4H4a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h14a2 2 0 0 0 2-2v-7" />
      <Path d="M18.5 2.5a2.121 2.121 0 0 1 3 3L12 15l-4 1 1-4 9.5-9.5z" />
    </Ico>
  ),
  copy: (p: IconProps) => (
    <Ico {...p}>
      <Rect x={9} y={9} width={13} height={13} rx={2} />
      <Path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1" />
    </Ico>
  ),
  search: (p: IconProps) => (
    <Ico {...p}>
      <Circle cx={11} cy={11} r={8} />
      <Line x1={21} y1={21} x2={16.65} y2={16.65} />
    </Ico>
  ),
  filter: (p: IconProps) => (
    <Ico {...p}>
      <Polygon points="22 3 2 3 10 12.46 10 19 14 21 14 12.46 22 3" />
    </Ico>
  ),
  sort: (p: IconProps) => (
    <Ico {...p}>
      <Line x1={3} y1={6} x2={21} y2={6} />
      <Line x1={3} y1={12} x2={15} y2={12} />
      <Line x1={3} y1={18} x2={9} y2={18} />
    </Ico>
  ),
  grid: (p: IconProps) => (
    <Ico {...p}>
      <Rect x={3} y={3} width={7} height={7} />
      <Rect x={14} y={3} width={7} height={7} />
      <Rect x={14} y={14} width={7} height={7} />
      <Rect x={3} y={14} width={7} height={7} />
    </Ico>
  ),
  list: (p: IconProps) => (
    <Ico {...p}>
      <Line x1={8} y1={6} x2={21} y2={6} />
      <Line x1={8} y1={12} x2={21} y2={12} />
      <Line x1={8} y1={18} x2={21} y2={18} />
      <Line x1={3} y1={6} x2={3.01} y2={6} />
      <Line x1={3} y1={12} x2={3.01} y2={12} />
      <Line x1={3} y1={18} x2={3.01} y2={18} />
    </Ico>
  ),
  log: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z" />
      <Polyline points="14 2 14 8 20 8" />
      <Line x1={16} y1={13} x2={8} y2={13} />
      <Line x1={16} y1={17} x2={8} y2={17} />
      <Polyline points="10 9 9 9 8 9" />
    </Ico>
  ),
  firmware: (p: IconProps) => (
    <Ico {...p}>
      <Path d="M22 16.92v3a2 2 0 0 1-2.18 2 19.79 19.79 0 0 1-8.63-3.07A19.5 19.5 0 0 1 4.64 13.5a19.79 19.79 0 0 1-3.07-8.67A2 2 0 0 1 3.55 2.7l3-.07a2 2 0 0 1 2 1.72 12.84 12.84 0 0 0 .7 2.81 2 2 0 0 1-.45 2.11L7.78 9.3a16 16 0 0 0 6.29 6.29l1.66-1.66a2 2 0 0 1 2.11-.45 12.84 12.84 0 0 0 2.81.7A2 2 0 0 1 22 16.92z" />
    </Ico>
  ),
  ros: (p: IconProps) => (
    <Ico {...p}>
      <Circle cx={12} cy={12} r={3} />
      <Path d="M12 2a10 10 0 0 1 10 10" />
      <Path d="M12 22A10 10 0 0 1 2 12" />
      <Path d="M2 12a10 10 0 0 1 10-10" />
      <Path d="M22 12a10 10 0 0 1-10 10" />
    </Ico>
  ),
  signal: (p: IconProps) => (
    <Ico {...p}>
      <Line x1={2} y1={20} x2={2} y2={20} />
      <Line x1={7} y1={16} x2={7} y2={20} />
      <Line x1={12} y1={12} x2={12} y2={20} />
      <Line x1={17} y1={8} x2={17} y2={20} />
      <Line x1={22} y1={4} x2={22} y2={20} />
    </Ico>
  ),
  altitude: (p: IconProps) => (
    <Ico {...p}>
      <Polyline points="22 12 18 12 15 21 9 3 6 12 2 12" />
    </Ico>
  ),
  speed: (p: IconProps) => (
    <Ico {...p}>
      <Circle cx={12} cy={12} r={10} />
      <Polyline points="12 6 12 12 16 14" />
    </Ico>
  ),
  heading: (p: IconProps) => (
    <Ico {...p}>
      <Circle cx={12} cy={12} r={10} />
      <Line x1={12} y1={8} x2={12} y2={12} />
      <Line x1={12} y1={12} x2={16} y2={14} />
    </Ico>
  ),
};

export type IconName = keyof typeof Icons;
