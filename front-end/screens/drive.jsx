// Drive — joysticks, attitude indicator, motor monitor.
// The 3D-feeling attitude indicator is the novel moment.

function DriveScreen() {
  const r = useRover();
  const { t } = r;
  const [leftJ, setLeftJ] = React.useState({ x: 0, y: 0 });
  const [rightJ, setRightJ] = React.useState({ x: 0, y: 0 });
  const [penDown, setPenDown] = React.useState(false);
  const [headlights, setHeadlights] = React.useState(true);

  return (
    <div style={{ padding: '0 0 100px' }}>
      <AppBar
        title="Manual Drive"
        subtitle={r.armed ? 'Armed · throttle live' : 'Disarmed · safe'}
        trailing={
          <div style={{ display:'flex', gap: 8 }}>
            <IconBtn icon={<I.cam size={18}/>} onClick={() => r.push('camera')} />
            <IconBtn icon={r.armed ? <I.unlock size={18}/> : <I.lock size={18}/>}
                     accent={r.armed}
                     onClick={() => r.apiSetArmed(!r.armed)} />
          </div>
        }
      />

      {/* Attitude + heading row */}
      <div style={{ padding: '0 16px 12px', display:'grid', gridTemplateColumns:'1.3fr 1fr', gap: 10 }}>
        <Card pad={12}>
          <div style={{ fontSize: 10, color: C.text3, textTransform:'uppercase', letterSpacing: 0.8, fontWeight: 600, marginBottom: 6 }}>
            ATTITUDE
          </div>
          <AttitudeIndicator pitch={t.pitch} roll={t.roll} />
          <div style={{ display:'flex', justifyContent:'space-between', marginTop: 8, fontFamily:'var(--mono)', fontSize: 11 }}>
            <span style={{ color: C.text3 }}>R: <span style={{ color: C.accent }}>{t.roll.toFixed(1)}°</span></span>
            <span style={{ color: C.text3 }}>P: <span style={{ color: C.accent }}>{t.pitch.toFixed(1)}°</span></span>
            <span style={{ color: C.text3 }}>Y: <span style={{ color: C.accent }}>{Math.round(t.yaw)}°</span></span>
          </div>
        </Card>

        <Card pad={12}>
          <div style={{ fontSize: 10, color: C.text3, textTransform:'uppercase', letterSpacing: 0.8, fontWeight: 600, marginBottom: 6 }}>
            HEADING
          </div>
          <HeadingDisc heading={t.heading} />
        </Card>
      </div>

      {/* Telemetry strip */}
      <div style={{ padding: '0 16px 12px', display:'grid', gridTemplateColumns:'repeat(4, 1fr)', gap: 6 }}>
        <MiniStat label="SPD" v={t.speed.toFixed(2)} u="m/s" color={C.accent} />
        <MiniStat label="ALT" v={t.alt.toFixed(2)} u="m" />
        <MiniStat label="VBAT" v={t.voltage.toFixed(1)} u="V" />
        <MiniStat label="AMP" v={t.current.toFixed(1)} u="A" color={C.warn} />
      </div>

      {/* Motor monitor */}
      <div style={{ padding: '0 16px 14px' }}>
        <Card pad={12}>
          <div style={{ display:'flex', justifyContent:'space-between', alignItems:'center', marginBottom: 10 }}>
            <span style={{ fontSize: 10, color: C.text3, textTransform:'uppercase', letterSpacing: 0.8, fontWeight: 600 }}>MOTORS</span>
            <span style={{ fontSize: 11, color: C.text3, fontFamily:'var(--mono)' }}>4× brushless · 200KV</span>
          </div>
          <div style={{ display:'grid', gridTemplateColumns:'repeat(4,1fr)', gap: 8 }}>
            {['FL','FR','RL','RR'].map((m,i) => (
              <MotorTile key={m} label={m} value={t.motor[i]} />
            ))}
          </div>
        </Card>
      </div>

      {/* Joysticks */}
      <div style={{ padding: '0 16px 14px', display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 12 }}>
        <Joystick label="DRIVE" value={leftJ} onChange={setLeftJ}
                  hint="↑↓ throttle  ←→ yaw" disabled={!r.armed || r.emergency} />
        <Joystick label="STEER" value={rightJ} onChange={setRightJ}
                  hint="↑↓ pitch  ←→ roll" disabled={!r.armed || r.emergency} />
      </div>

      {/* Action chips */}
      <div style={{ padding: '0 16px 14px' }}>
        <Card pad={12}>
          <div style={{ display:'flex', gap: 8, flexWrap: 'wrap' }}>
            <ActionChip on={penDown} onClick={() => setPenDown(p => !p)}
                        icon={<I.draw size={14}/>} label={penDown ? 'Pen down' : 'Pen up'} color={C.violet}/>
            <ActionChip on={headlights} onClick={() => setHeadlights(h => !h)}
                        icon={<I.zap size={14}/>} label="Lights" color={C.warn}/>
            <ActionChip onClick={() => r.apiSetMode("Hold")} icon={<I.pause size={14}/>} label="Hold" color={C.accent}/>
            <ActionChip onClick={() => r.push('camera')} icon={<I.cam size={14}/>} label="Camera" color={C.text2}/>
          </div>
        </Card>
      </div>

      {/* E-stop bar (always reachable) */}
      <div style={{ padding: '0 16px 16px' }}>
        <button onClick={r.emergency ? r.clearEStop : r.triggerEStop} style={{
          width: '100%', padding: 16, borderRadius: 16, border: 'none', cursor: 'pointer',
          background: r.emergency
            ? `linear-gradient(135deg, #ffb84d, #fbbf24)`
            : `linear-gradient(135deg, #ff4d6d, #fb7185)`,
          color: r.emergency ? '#3a2906' : '#3a0a14',
          display:'flex', alignItems:'center', justifyContent:'center', gap: 10,
          fontWeight: 800, fontSize: 16, letterSpacing: 0.5, textTransform: 'uppercase',
          boxShadow: r.emergency ? `0 8px 24px ${C.warn}44` : `0 8px 24px ${C.danger}44`,
        }}>
          {r.emergency ? <><I.refresh size={18}/> Clear E-Stop & Resume</> : <><I.warn size={18}/> Emergency Stop</>}
        </button>
        <div style={{ textAlign:'center', fontSize: 11, color: C.text3, marginTop: 6 }}>
          Cuts motors instantly · hold for hardware kill (3s)
        </div>
      </div>
    </div>
  );
}

function MiniStat({ label, v, u, color = C.text }) {
  return (
    <div style={{ padding: '8px 10px', background: C.card2, borderRadius: 10, border: `1px solid ${C.line}` }}>
      <div style={{ fontSize: 10, color: C.text3, fontWeight: 600, letterSpacing: 0.5 }}>{label}</div>
      <div style={{ display:'flex', alignItems:'baseline', gap: 2, marginTop: 2 }}>
        <span style={{ fontFamily:'var(--mono)', fontSize: 16, fontWeight: 600, color }}>{v}</span>
        <span style={{ fontSize: 9, color: C.text3 }}>{u}</span>
      </div>
    </div>
  );
}

function MotorTile({ label, value }) {
  const color = value > 90 ? C.warn : C.accent;
  return (
    <div style={{ padding: '6px 8px', borderRadius: 10, background: 'rgba(255,255,255,0.025)', border: `1px solid ${C.line}` }}>
      <div style={{ display:'flex', justifyContent:'space-between', alignItems:'center' }}>
        <span style={{ fontSize: 10, color: C.text3, fontWeight: 700, letterSpacing: 0.5 }}>{label}</span>
        <span style={{ fontSize: 11, color, fontFamily:'var(--mono)', fontWeight: 600 }}>{Math.round(value)}%</span>
      </div>
      <div style={{ marginTop: 4 }}>
        <Bar value={value} color={color} height={4} />
      </div>
    </div>
  );
}

function ActionChip({ on, onClick, icon, label, color = C.accent }) {
  return (
    <button onClick={onClick} style={{
      display:'inline-flex', alignItems:'center', gap: 6,
      padding: '7px 12px', borderRadius: 999,
      background: on ? `${color}26` : C.card2,
      border: `1px solid ${on ? color + '66' : C.line2}`,
      color: on ? color : C.text2, fontSize: 12, fontWeight: 600, cursor: 'pointer',
    }}>
      {icon} {label}
    </button>
  );
}

// ───────────────────────────────────────────────
// Attitude indicator — pseudo-3D "ball" horizon.
// ───────────────────────────────────────────────
function AttitudeIndicator({ pitch, roll }) {
  // Sphere is 140px. We rotate the inner panel by roll, translate by pitch.
  const size = 140;
  const pitchOffset = (pitch / 90) * size * 0.55;
  return (
    <div style={{
      width: '100%', aspectRatio: '1 / 1', maxWidth: size,
      position: 'relative', margin: '0 auto',
      borderRadius: '50%', overflow: 'hidden',
      background: '#0a0d12',
      border: `1.5px solid ${C.line2}`,
      boxShadow: `inset 0 6px 16px rgba(0,0,0,0.5), inset 0 -2px 6px rgba(34,211,238,0.05)`,
    }}>
      {/* sky/ground ball */}
      <div style={{
        position: 'absolute', inset: 0,
        transform: `rotate(${-roll}deg)`,
      }}>
        <div style={{
          position: 'absolute', left: '-50%', right: '-50%', top: '-50%', bottom: '-50%',
          transform: `translateY(${pitchOffset}px)`,
        }}>
          {/* sky */}
          <div style={{ position:'absolute', left: 0, right: 0, top: 0, bottom: '50%',
                        background: `linear-gradient(180deg, #2dd4bf 0%, #155e75 100%)`,
                        boxShadow: 'inset 0 -30px 60px rgba(0,0,0,0.3)' }} />
          {/* ground */}
          <div style={{ position:'absolute', left: 0, right: 0, top: '50%', bottom: 0,
                        background: `linear-gradient(180deg, #a16207 0%, #451a03 100%)`,
                        boxShadow: 'inset 0 30px 60px rgba(0,0,0,0.3)' }} />
          {/* horizon */}
          <div style={{ position:'absolute', left: 0, right: 0, top: 'calc(50% - 1px)', height: 2,
                        background: '#fff', opacity: 0.85, boxShadow: '0 0 8px rgba(255,255,255,0.6)' }} />
          {/* pitch ladder */}
          {[-30,-20,-10,10,20,30].map(p => (
            <div key={p} style={{
              position:'absolute', left: '50%', top: `calc(50% - ${p * (size * 0.55 / 90)}px)`,
              transform: 'translateX(-50%)',
              width: Math.abs(p) === 10 ? 38 : Math.abs(p) === 20 ? 24 : 16,
              height: 1.5, background: 'rgba(255,255,255,0.5)',
            }} />
          ))}
        </div>
      </div>

      {/* center reticle */}
      <svg viewBox="0 0 100 100" style={{ position:'absolute', inset: 0 }}>
        <path d="M30,50 L42,50 M58,50 L70,50 M50,42 L50,46" stroke={C.accent} strokeWidth="2" strokeLinecap="round" fill="none" />
        <circle cx="50" cy="50" r="2" fill={C.accent} />
        {/* roll tick */}
        <g transform={`rotate(${-roll} 50 50)`}>
          <path d="M50,10 L46,17 L54,17 Z" fill={C.accent} />
        </g>
        {/* fixed scale */}
        {[0,-30,-60,60,30].map(deg => (
          <g key={deg} transform={`rotate(${deg} 50 50)`}>
            <line x1="50" y1="6" x2="50" y2={Math.abs(deg) % 30 === 0 ? 11 : 9}
                  stroke="rgba(255,255,255,0.5)" strokeWidth="1.2" />
          </g>
        ))}
      </svg>
    </div>
  );
}

// ───────────────────────────────────────────────
// Heading compass disc
// ───────────────────────────────────────────────
function HeadingDisc({ heading }) {
  const size = 140;
  return (
    <div style={{
      width: '100%', aspectRatio: '1 / 1', maxWidth: size,
      position: 'relative', margin: '0 auto',
      borderRadius: '50%', background: '#0a0d12',
      border: `1.5px solid ${C.line2}`,
      overflow: 'hidden',
      boxShadow: `inset 0 6px 16px rgba(0,0,0,0.5)`,
    }}>
      {/* rotating tick disc */}
      <div style={{ position:'absolute', inset: 0, transform: `rotate(${-heading}deg)`, transition: 'transform 0.25s linear' }}>
        <svg viewBox="0 0 100 100" width="100%" height="100%">
          {/* cardinal */}
          {[['N',0],['E',90],['S',180],['W',270]].map(([l,a]) => (
            <g key={l} transform={`rotate(${a} 50 50)`}>
              <text x="50" y="18" textAnchor="middle" fontSize="9"
                    fill={l === 'N' ? C.accent : C.text2} fontWeight="700" fontFamily="var(--mono)">{l}</text>
              <line x1="50" y1="22" x2="50" y2="26" stroke={l === 'N' ? C.accent : C.text2} strokeWidth="1.5" />
            </g>
          ))}
          {/* minor ticks */}
          {Array.from({length: 36}).map((_,i) => {
            if (i % 9 === 0) return null;
            return (
              <line key={i} x1="50" y1="9" x2="50" y2={i % 3 === 0 ? 13 : 11}
                    stroke="rgba(255,255,255,0.3)" strokeWidth="0.8"
                    transform={`rotate(${i*10} 50 50)`} />
            );
          })}
        </svg>
      </div>
      {/* fixed center pointer */}
      <svg viewBox="0 0 100 100" style={{ position:'absolute', inset: 0 }}>
        <path d="M50,6 L46,14 L54,14 Z" fill={C.accent} />
        <text x="50" y="52" textAnchor="middle" fontSize="14" fontWeight="700" fill={C.text} fontFamily="var(--mono)">
          {String(Math.round(heading)).padStart(3,'0')}°
        </text>
        <text x="50" y="64" textAnchor="middle" fontSize="7" fill={C.text3} letterSpacing="1">MAG</text>
      </svg>
    </div>
  );
}

// ───────────────────────────────────────────────
// Joystick
// ───────────────────────────────────────────────
function Joystick({ label, value, onChange, hint, disabled }) {
  const ref = React.useRef(null);
  const [active, setActive] = React.useState(false);
  const size = 140;

  const onDown = (e) => {
    if (disabled) return;
    setActive(true);
    e.preventDefault();
    const rect = ref.current.getBoundingClientRect();
    const cx = rect.left + rect.width/2, cy = rect.top + rect.height/2;
    const move = (ev) => {
      const dx = ev.clientX - cx, dy = ev.clientY - cy;
      const r = rect.width / 2 - 18;
      const dist = Math.hypot(dx, dy);
      const k = dist > r ? r / dist : 1;
      onChange({ x: (dx * k) / r, y: (dy * k) / r });
    };
    const up = () => {
      setActive(false);
      onChange({ x: 0, y: 0 });
      window.removeEventListener('pointermove', move);
      window.removeEventListener('pointerup', up);
    };
    window.addEventListener('pointermove', move);
    window.addEventListener('pointerup', up);
    move(e);
  };

  const knobX = value.x * (size/2 - 18);
  const knobY = value.y * (size/2 - 18);

  return (
    <Card pad={12} style={{ background: disabled ? C.card : C.card }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems:'center', marginBottom: 8 }}>
        <span style={{ fontSize: 10, color: C.text3, textTransform:'uppercase', letterSpacing: 0.8, fontWeight: 600 }}>{label}</span>
        <span style={{ fontSize: 10, color: C.text3, fontFamily:'var(--mono)' }}>
          {value.x.toFixed(2)}, {(-value.y).toFixed(2)}
        </span>
      </div>
      <div ref={ref}
           onPointerDown={onDown}
           style={{
             width: '100%', aspectRatio: '1/1', maxWidth: size, margin: '0 auto',
             borderRadius: '50%', position: 'relative',
             background: disabled
              ? `radial-gradient(circle at 30% 30%, #181f2c, #0a0d12)`
              : `radial-gradient(circle at 30% 30%, #1b2740, #0a0d12)`,
             border: `1.5px solid ${active ? C.accent + '66' : C.line2}`,
             boxShadow: active
               ? `inset 0 0 24px ${C.accent}33, 0 0 12px ${C.accent}44`
               : `inset 0 4px 16px rgba(0,0,0,0.5)`,
             touchAction: 'none', userSelect: 'none', cursor: disabled ? 'not-allowed' : 'grab',
             opacity: disabled ? 0.5 : 1,
           }}>
        {/* crosshair */}
        <svg viewBox="0 0 100 100" style={{ position:'absolute', inset: 0 }}>
          <circle cx="50" cy="50" r="32" fill="none" stroke="rgba(255,255,255,0.05)" strokeDasharray="2 3" />
          <line x1="50" y1="20" x2="50" y2="80" stroke="rgba(255,255,255,0.04)" />
          <line x1="20" y1="50" x2="80" y2="50" stroke="rgba(255,255,255,0.04)" />
        </svg>
        {/* knob */}
        <div style={{
          position:'absolute', top: '50%', left: '50%',
          transform: `translate(calc(-50% + ${knobX}px), calc(-50% + ${knobY}px))`,
          width: 36, height: 36, borderRadius: '50%',
          background: `radial-gradient(circle at 30% 30%, #44e6ff, #0e7490)`,
          boxShadow: `0 4px 12px ${C.accent}66, inset 0 1px 0 rgba(255,255,255,0.4)`,
          border: `1px solid ${C.accent}`,
          transition: active ? 'none' : 'transform 0.2s',
        }} />
      </div>
      {hint && (
        <div style={{ marginTop: 8, textAlign: 'center', fontSize: 10, color: C.text3, fontFamily:'var(--mono)' }}>{hint}</div>
      )}
    </Card>
  );
}

window.DriveScreen = DriveScreen;
