// theme/colors.ts
export const C = {
  bg: '#0a0d12',
  bg2: '#0e1219',
  card: '#141923',
  card2: '#1a2030',
  line: 'rgba(255,255,255,0.07)',
  line2: 'rgba(255,255,255,0.12)',
  text: '#e6edf6',
  text2: '#a3adbf',
  text3: '#6b7585',
  accent: '#22d3ee',
  accent2: '#5eead4',
  warn: '#fbbf24',
  danger: '#fb7185',
  good: '#34d399',
  violet: '#a78bfa',
} as const;

export type ColorKey = keyof typeof C;