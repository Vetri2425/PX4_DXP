// Shared UI primitives — cards, pills, buttons, gauges, etc.

const C = {
  bg: '#0a0d12', bg2: '#0e1219', card: '#141923', card2: '#1a2030',
  line: 'rgba(255,255,255,0.07)', line2: 'rgba(255,255,255,0.12)',
  text: '#e6edf6', text2: '#a3adbf', text3: '#6b7585',
  accent: '#22d3ee', accent2: '#5eead4',
  warn: '#fbbf24', danger: '#fb7185', good: '#34d399', violet: '#a78bfa',
};
window.C = C;

// ── Card ────────────────────────────────────────────────
function Card({ children, style, pad = 16, onClick, accent }) {
  return (
    <div onClick={onClick} style={{
      background: C.card, border: `1px solid ${C.line}`,
      borderRadius: 18, padding: pad, position: 'relative',
      ...(accent ? { boxShadow: `inset 0 1px 0 rgba(255,255,255,0.04), 0 0 0 1px ${accent}33` } : {}),
      ...style,
    }}>{children}</div>
  );
}

// ── Pill (status dot + label) ────────────────────────────
function Pill({ color = C.accent, children, style, dim }) {
  return (
    <span style={{
      display: 'inline-flex', alignItems: 'center', gap: 6,
      padding: '4px 9px', borderRadius: 999,
      background: dim ? `${color}1a` : `${color}22`,
      border: `1px solid ${color}33`,
      color: color, fontSize: 11, fontWeight: 600, letterSpacing: 0.2,
      ...style,
    }}>{children}</span>
  );
}

// ── Status dot — pulsing ────────────────────────────────
function Dot({ color = C.good, size = 8, pulse = true }) {
  return (
    <span style={{
      display: 'inline-block', width: size, height: size, borderRadius: '50%',
      background: color,
      boxShadow: pulse ? `0 0 0 0 ${color}66` : 'none',
      animation: pulse ? 'pxpulse 1.6s ease-out infinite' : 'none',
    }} />
  );
}

// ── Button ───────────────────────────────────────────────
function Btn({ children, variant = 'primary', size = 'md', icon, full, onClick, style, disabled }) {
  const sizes = {
    sm: { padding: '7px 12px', fontSize: 13, height: 32, gap: 6 },
    md: { padding: '10px 16px', fontSize: 14, height: 40, gap: 8 },
    lg: { padding: '14px 20px', fontSize: 15, height: 50, gap: 10 },
  };
  const variants = {
    primary:  { background: C.accent, color: '#06202a', border: '1px solid transparent', fontWeight: 600 },
    secondary:{ background: C.card2, color: C.text, border: `1px solid ${C.line2}`, fontWeight: 500 },
    ghost:    { background: 'transparent', color: C.text2, border: `1px solid ${C.line}`, fontWeight: 500 },
    danger:   { background: C.danger, color: '#3a0a14', border: '1px solid transparent', fontWeight: 700 },
    warn:     { background: C.warn, color: '#3a2906', border: '1px solid transparent', fontWeight: 600 },
    accentGhost: { background: `${C.accent}14`, color: C.accent, border: `1px solid ${C.accent}33`, fontWeight: 600 },
  };
  return (
    <button onClick={onClick} disabled={disabled} style={{
      display: 'inline-flex', alignItems: 'center', justifyContent: 'center',
      borderRadius: 12, cursor: 'pointer', whiteSpace: 'nowrap',
      width: full ? '100%' : 'auto',
      opacity: disabled ? 0.5 : 1,
      transition: 'transform .08s, opacity .15s',
      ...sizes[size], ...variants[variant], ...style,
    }}>
      {icon && <span style={{ display: 'inline-flex' }}>{icon}</span>}
      {children}
    </button>
  );
}

// ── Tiny stat tile ──────────────────────────────────────
function Stat({ label, value, unit, sub, color = C.text, icon, accent }) {
  return (
    <div style={{
      display:'flex', flexDirection:'column', gap: 4,
      padding: '12px 14px',
      background: C.card2, borderRadius: 14, border: `1px solid ${C.line}`,
      minWidth: 0,
    }}>
      <div style={{ display:'flex', justifyContent:'space-between', alignItems:'center', color: C.text3, fontSize: 11, textTransform: 'uppercase', letterSpacing: 0.6, fontWeight: 600 }}>
        <span>{label}</span>
        {icon && <span style={{ color: accent || C.text3 }}>{icon}</span>}
      </div>
      <div style={{ display:'flex', alignItems:'baseline', gap: 4 }}>
        <span style={{ fontFamily: 'var(--mono)', fontSize: 22, fontWeight: 600, color, letterSpacing: -0.5 }}>{value}</span>
        {unit && <span style={{ color: C.text3, fontSize: 12, fontFamily: 'var(--mono)' }}>{unit}</span>}
      </div>
      {sub && <div style={{ color: C.text3, fontSize: 11 }}>{sub}</div>}
    </div>
  );
}

// ── Bar gauge ───────────────────────────────────────────
function Bar({ value, max = 100, color = C.accent, height = 6, bg = 'rgba(255,255,255,0.06)' }) {
  const pct = Math.max(0, Math.min(100, (value / max) * 100));
  return (
    <div style={{ height, background: bg, borderRadius: 999, overflow: 'hidden' }}>
      <div style={{ width: `${pct}%`, height: '100%', background: color, borderRadius: 999, transition: 'width .3s' }} />
    </div>
  );
}

// ── Section header inside scrollable content ────────────
function SectionHeader({ title, action, accessory, sub }) {
  return (
    <div style={{ display:'flex', alignItems:'flex-end', justifyContent:'space-between', padding: '8px 4px 12px' }}>
      <div>
        <div style={{ fontSize: 13, color: C.text3, textTransform: 'uppercase', letterSpacing: 0.8, fontWeight: 600 }}>{title}</div>
        {sub && <div style={{ fontSize: 11, color: C.text3, marginTop: 2 }}>{sub}</div>}
      </div>
      {action && <button onClick={action.onClick} style={{
        background: 'transparent', border: 'none', color: C.accent, fontSize: 13, fontWeight: 500, cursor: 'pointer', padding: 0,
      }}>{action.label}</button>}
      {accessory}
    </div>
  );
}

// ── Top App Bar (within the iOS frame) ──────────────────
function AppBar({ title, subtitle, leading, trailing, sticky, dense }) {
  return (
    <div style={{
      position: sticky ? 'sticky' : 'relative', top: 0, zIndex: 5,
      padding: dense ? '8px 16px 8px' : '12px 16px 14px',
      background: `linear-gradient(180deg, ${C.bg}f8 60%, ${C.bg}00)`,
      backdropFilter: 'blur(20px)', WebkitBackdropFilter: 'blur(20px)',
      display: 'flex', alignItems: 'center', gap: 12,
    }}>
      {leading}
      <div style={{ flex: 1, minWidth: 0 }}>
        <div style={{ fontSize: dense ? 17 : 22, fontWeight: 700, letterSpacing: -0.4 }}>{title}</div>
        {subtitle && <div style={{ fontSize: 12, color: C.text2, marginTop: 1 }}>{subtitle}</div>}
      </div>
      {trailing}
    </div>
  );
}

// ── Round Icon Button (chrome) ─────────────────────────
function IconBtn({ icon, onClick, accent, size = 36, style, badge }) {
  return (
    <button onClick={onClick} style={{
      width: size, height: size, borderRadius: size/2,
      background: accent ? `${C.accent}1a` : C.card2,
      border: `1px solid ${accent ? C.accent + '33' : C.line2}`,
      color: accent ? C.accent : C.text,
      display:'inline-flex', alignItems:'center', justifyContent:'center',
      cursor: 'pointer', position: 'relative',
      ...style,
    }}>
      {icon}
      {badge != null && (
        <span style={{
          position:'absolute', top: -2, right: -2, minWidth: 16, height: 16, padding:'0 4px',
          borderRadius: 999, background: C.danger, color: '#3a0a14',
          fontSize: 10, fontWeight: 700, display:'inline-flex', alignItems:'center', justifyContent:'center',
        }}>{badge}</span>
      )}
    </button>
  );
}

// ── List Row (settings-style) ───────────────────────────
function Row({ icon, iconBg = C.card2, iconColor = C.text2, title, sub, detail, chevron = true, onClick, danger }) {
  return (
    <button onClick={onClick} style={{
      width: '100%', display: 'flex', alignItems: 'center', gap: 12,
      padding: '12px 14px', background: 'transparent', border: 'none', cursor: 'pointer',
      textAlign: 'left', color: danger ? C.danger : C.text,
    }}>
      {icon && (
        <span style={{
          width: 36, height: 36, borderRadius: 10, background: iconBg, color: iconColor,
          display:'inline-flex', alignItems:'center', justifyContent:'center',
        }}>{icon}</span>
      )}
      <span style={{ flex: 1, minWidth: 0 }}>
        <div style={{ fontSize: 15, fontWeight: 500 }}>{title}</div>
        {sub && <div style={{ fontSize: 12, color: C.text3, marginTop: 1 }}>{sub}</div>}
      </span>
      {detail && <span style={{ color: C.text2, fontSize: 13, fontFamily: 'var(--mono)' }}>{detail}</span>}
      {chevron && <I.chevR size={16} color={C.text3} />}
    </button>
  );
}

// ── Toggle ──────────────────────────────────────────────
function Toggle({ value, onChange, color = C.accent }) {
  return (
    <button onClick={() => onChange(!value)} style={{
      width: 44, height: 26, borderRadius: 999,
      background: value ? color : 'rgba(255,255,255,0.12)',
      border: 'none', position: 'relative', cursor: 'pointer', padding: 0,
      transition: 'background .15s',
    }}>
      <span style={{
        position: 'absolute', top: 3, left: value ? 21 : 3,
        width: 20, height: 20, borderRadius: '50%', background: '#fff',
        boxShadow: '0 1px 3px rgba(0,0,0,0.4)', transition: 'left .15s',
      }} />
    </button>
  );
}

// ── Sparkline ───────────────────────────────────────────
function Spark({ data, color = C.accent, height = 32, width = 100, fill = true }) {
  if (!data || !data.length) return null;
  const min = Math.min(...data), max = Math.max(...data);
  const range = max - min || 1;
  const pts = data.map((v, i) => {
    const x = (i / (data.length - 1)) * width;
    const y = height - 2 - ((v - min) / range) * (height - 4);
    return [x, y];
  });
  const path = pts.map(([x,y], i) => `${i===0?'M':'L'}${x.toFixed(1)},${y.toFixed(1)}`).join(' ');
  const area = `${path} L${width},${height} L0,${height} Z`;
  return (
    <svg width={width} height={height} style={{ display: 'block' }}>
      {fill && <path d={area} fill={`${color}22`} />}
      <path d={path} stroke={color} strokeWidth={1.5} fill="none" strokeLinejoin="round" strokeLinecap="round" />
    </svg>
  );
}

// ── Animated indeterminate stripe ───────────────────────
const __ui_keyframes = `
@keyframes pxpulse { 0%{box-shadow:0 0 0 0 currentColor} 70%{box-shadow:0 0 0 8px transparent} 100%{box-shadow:0 0 0 0 transparent} }
@keyframes pxspin { to { transform: rotate(360deg) } }
@keyframes pxstripe {
  0% { background-position: 0 0 }
  100% { background-position: 30px 0 }
}
@keyframes pxblip {
  0%, 100% { opacity: 0.4 } 50% { opacity: 1 }
}
@keyframes pxsweep {
  from { transform: rotate(0deg) } to { transform: rotate(360deg) }
}
`;
function GlobalCSS(){
  return <style>{__ui_keyframes}</style>;
}

Object.assign(window, { Card, Pill, Dot, Btn, Stat, Bar, SectionHeader, AppBar, IconBtn, Row, Toggle, Spark, GlobalCSS });
