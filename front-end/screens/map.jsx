// Map / Mission Planner — full-bleed map with telemetry overlay,
// drag-to-edit waypoints (the novel gesture moment).

function MapScreen() {
  const r = useRover();
  const { t } = r;
  const [selected, setSelected] = React.useState(null);
  const [mapStyle, setMapStyle] = React.useState('dark'); // dark | satellite | grid
  const svgRef = React.useRef(null);
  const draggingRef = React.useRef(null);

  // SVG canvas is 360 × 460 in mapview coords.
  const W = 360, H = 460;

  // Update waypoint position on drag (pointer)
  const onPointerDown = (e, wp) => {
    e.preventDefault();
    e.stopPropagation();
    draggingRef.current = wp.id;
    setSelected(wp.id);
    const move = (ev) => {
      const rect = svgRef.current.getBoundingClientRect();
      const x = ((ev.clientX - rect.left) / rect.width) * W;
      const y = ((ev.clientY - rect.top) / rect.height) * H;
      r.setWaypoints(ws => ws.map(w => w.id === draggingRef.current ? { ...w, x: Math.max(10, Math.min(W-10,x)), y: Math.max(10, Math.min(H-10,y)) } : w));
    };
    const up = () => {
      draggingRef.current = null;
      window.removeEventListener('pointermove', move);
      window.removeEventListener('pointerup', up);
    };
    window.addEventListener('pointermove', move);
    window.addEventListener('pointerup', up);
  };

  // Add waypoint on long tap (down for 350ms)
  const tapTimer = React.useRef(null);
  const tapPos = React.useRef(null);
  const onCanvasPointerDown = (e) => {
    if (draggingRef.current) return;
    const rect = svgRef.current.getBoundingClientRect();
    tapPos.current = {
      x: ((e.clientX - rect.left) / rect.width) * W,
      y: ((e.clientY - rect.top) / rect.height) * H,
    };
    tapTimer.current = setTimeout(() => {
      if (!tapPos.current) return;
      const id = 'wp' + (r.waypoints.length + 1) + '_' + Date.now();
      r.setWaypoints(ws => [...ws, { id, x: tapPos.current.x, y: tapPos.current.y, type: 'pen-down' }]);
      setSelected(id);
      tapTimer.current = null;
    }, 350);
  };
  const onCanvasPointerUp = () => {
    if (tapTimer.current) clearTimeout(tapTimer.current);
    tapTimer.current = null;
    tapPos.current = null;
  };

  const deleteWp = () => {
    if (!selected) return;
    r.setWaypoints(ws => ws.filter(w => w.id !== selected));
    setSelected(null);
  };

  // Path string (closed mission loop)
  const pathStr = r.waypoints.map((w,i) => `${i===0?'M':'L'}${w.x},${w.y}`).join(' ');

  // Rover position — projected onto first segment by progress
  const wp0 = r.waypoints[0], wp1 = r.waypoints[1] || wp0;
  const roverPos = wp1 ? {
    x: wp0.x + (wp1.x - wp0.x) * (r.drawProgress * 2 % 1),
    y: wp0.y + (wp1.y - wp0.y) * (r.drawProgress * 2 % 1),
  } : { x: 100, y: 100 };

  return (
    <div style={{ position:'relative', height: '100%', display:'flex', flexDirection:'column' }}>
      {/* Top bar */}
      <AppBar
        title="Mission"
        subtitle={`${r.waypoints.length} waypoints · ${r.activeJob.name}`}
        leading={<IconBtn icon={<I.layers size={18} />} onClick={() => setMapStyle(s => s === 'dark' ? 'satellite' : s === 'satellite' ? 'grid' : 'dark')} />}
        trailing={
          <div style={{ display: 'flex', gap: 8 }}>
            <IconBtn icon={<I.recenter size={18}/>} />
            <IconBtn icon={<I.share size={18}/>} />
          </div>
        }
      />

      {/* Map canvas */}
      <div style={{ flex: 1, position:'relative', overflow:'hidden', margin: '0 16px', borderRadius: 20, border: `1px solid ${C.line2}` }}>
        <MapBackground style={mapStyle} />

        <svg
          ref={svgRef}
          viewBox={`0 0 ${W} ${H}`}
          width="100%" height="100%"
          style={{ position:'absolute', inset: 0, touchAction:'none', userSelect:'none' }}
          onPointerDown={onCanvasPointerDown}
          onPointerUp={onCanvasPointerUp}
          onPointerLeave={onCanvasPointerUp}
        >
          {/* path between waypoints */}
          <path d={pathStr} stroke={C.accent} strokeWidth="2" fill="none" strokeDasharray="6 4" strokeLinecap="round" opacity="0.7" />
          {/* drawn portion (solid up to roverPos progress) */}
          <path d={pathStr} stroke={C.accent2} strokeWidth="3" fill="none" strokeLinecap="round"
                strokeDasharray={`${r.drawProgress * 1500} 1500`} />

          {/* waypoints */}
          {r.waypoints.map((w, i) => (
            <Waypoint key={w.id} w={w} i={i} selected={selected === w.id}
                      onPointerDown={(e) => onPointerDown(e, w)} />
          ))}

          {/* Rover (sweeping radar) */}
          <g transform={`translate(${roverPos.x},${roverPos.y})`}>
            <circle r="22" fill={`${C.accent}11`} stroke={`${C.accent}55`} strokeDasharray="2 3" />
            <g style={{ transformOrigin: '0 0', animation: 'pxsweep 3s linear infinite' }}>
              <path d="M0,0 L18,0 A18,18 0 0 0 9,-16 Z" fill={`${C.accent}33`} />
            </g>
            <circle r="6" fill={C.accent} stroke="#06202a" strokeWidth="2"/>
            <line x1="0" y1="0" x2={Math.cos((t.heading-90)*Math.PI/180)*12} y2={Math.sin((t.heading-90)*Math.PI/180)*12} stroke="#06202a" strokeWidth="2.5" />
          </g>
        </svg>

        {/* Top-left telemetry chip */}
        <div style={{ position:'absolute', top: 12, left: 12, display:'flex', flexDirection:'column', gap: 6 }}>
          <ChipRow><I.gps size={11} color={C.accent}/> RTK 3D · {t.sats} sat · HDOP {t.hdop.toFixed(1)}</ChipRow>
          <ChipRow><I.compass size={11} color={C.accent}/> HDG {Math.round(t.heading)}°</ChipRow>
          <ChipRow><I.gauge size={11} color={C.accent}/> {t.speed.toFixed(2)} m/s</ChipRow>
        </div>

        {/* Top-right map style */}
        <div style={{ position:'absolute', top: 12, right: 12 }}>
          <button onClick={() => setMapStyle(s => s === 'dark' ? 'satellite' : s === 'satellite' ? 'grid' : 'dark')} style={{
            border: `1px solid ${C.line2}`, background: 'rgba(20,25,35,0.7)', backdropFilter:'blur(20px)',
            color: C.text2, padding: '6px 10px', borderRadius: 999, fontSize: 11, fontWeight: 600,
            display:'inline-flex', alignItems:'center', gap: 6, cursor: 'pointer', textTransform: 'uppercase', letterSpacing: 0.6,
          }}>
            <I.layers size={12} /> {mapStyle}
          </button>
        </div>

        {/* Selected waypoint inspector */}
        {selected && (
          <WpInspector
            wp={r.waypoints.find(w => w.id === selected)}
            onClose={() => setSelected(null)}
            onDelete={deleteWp}
            onType={(type) => r.setWaypoints(ws => ws.map(w => w.id === selected ? { ...w, type } : w))}
            index={r.waypoints.findIndex(w => w.id === selected)}
          />
        )}

        {/* Hint pill */}
        {!selected && (
          <div style={{ position:'absolute', bottom: 12, left: 12, right: 12, display:'flex', justifyContent:'center', pointerEvents:'none' }}>
            <div style={{
              background:'rgba(20,25,35,0.78)', backdropFilter:'blur(20px)',
              border: `1px solid ${C.line2}`, borderRadius: 999,
              padding: '6px 12px', fontSize: 11, color: C.text2, fontWeight: 500,
            }}>
              Long-press to add · drag to move · tap to edit
            </div>
          </div>
        )}
      </div>

      {/* Bottom action sheet */}
      <div style={{ padding: '14px 16px 100px' }}>
        <Card pad={12}>
          <div style={{ display:'flex', alignItems:'center', justifyContent:'space-between' }}>
            <div>
              <div style={{ fontSize: 11, color: C.text3, textTransform:'uppercase', letterSpacing:0.7, fontWeight: 600 }}>Mission</div>
              <div style={{ fontSize: 14, fontWeight: 600, marginTop: 2 }}>
                {r.waypoints.length} waypoints · {missionLength(r.waypoints).toFixed(1)} m
              </div>
            </div>
            <div style={{ display:'flex', gap: 6 }}>
              <Btn variant="secondary" size="sm" icon={<I.upload size={14}/>}>Upload</Btn>
              <Btn variant="primary" size="sm" icon={<I.play size={14}/>} onClick={() => r.apiMissionStart()}>Run</Btn>
            </div>
          </div>
        </Card>
      </div>
    </div>
  );
}

function missionLength(ws) {
  let d = 0;
  for (let i = 1; i < ws.length; i++) {
    const dx = ws[i].x - ws[i-1].x, dy = ws[i].y - ws[i-1].y;
    d += Math.sqrt(dx*dx + dy*dy);
  }
  return d / 12; // arbitrary scale → meters
}

function ChipRow({ children }) {
  return (
    <div style={{
      display:'inline-flex', alignItems:'center', gap: 6,
      padding: '4px 9px', borderRadius: 999,
      background: 'rgba(20,25,35,0.75)', backdropFilter: 'blur(20px)',
      border: `1px solid ${C.line2}`, color: C.text,
      fontSize: 11, fontFamily: 'var(--mono)', fontWeight: 500, width: 'fit-content',
    }}>{children}</div>
  );
}

function Waypoint({ w, i, selected, onPointerDown }) {
  const type = w.type || 'pen-down';
  const stroke = type === 'start' ? C.good : type === 'end' ? C.danger : type === 'pen-up' ? C.text3 : type === 'turn' ? C.warn : C.accent;
  return (
    <g transform={`translate(${w.x},${w.y})`}
       onPointerDown={onPointerDown}
       style={{ cursor:'grab' }}>
      {selected && <circle r="20" fill={`${stroke}22`} stroke={`${stroke}66`} strokeDasharray="2 3" />}
      <circle r="12" fill="#0a0d12" stroke={stroke} strokeWidth="2" />
      <text x="0" y="3.5" textAnchor="middle" fontSize="11" fontWeight="700" fill={stroke}
            fontFamily="var(--mono)">{i+1}</text>
    </g>
  );
}

function WpInspector({ wp, index, onClose, onDelete, onType }) {
  const types = ['start','pen-down','pen-up','turn','end'];
  return (
    <div style={{
      position:'absolute', bottom: 12, left: 12, right: 12,
      background:'rgba(15,20,28,0.92)', backdropFilter:'blur(24px)',
      border: `1px solid ${C.line2}`, borderRadius: 16, padding: 14,
    }}>
      <div style={{ display:'flex', alignItems:'center', justifyContent:'space-between' }}>
        <div>
          <div style={{ fontSize: 11, color: C.text3, textTransform:'uppercase', letterSpacing: 0.8, fontWeight: 600 }}>Waypoint #{index+1}</div>
          <div style={{ fontFamily:'var(--mono)', fontSize: 13, color: C.text2, marginTop: 2 }}>
            x:{wp.x.toFixed(0)}  y:{wp.y.toFixed(0)}
          </div>
        </div>
        <div style={{ display:'flex', gap: 6 }}>
          <IconBtn icon={<I.trash size={14}/>} onClick={onDelete} style={{ color: C.danger, borderColor: `${C.danger}33`, background: `${C.danger}14` }}/>
          <IconBtn icon={<I.x size={14}/>} onClick={onClose}/>
        </div>
      </div>
      <div style={{ display:'flex', gap: 6, marginTop: 10, flexWrap: 'wrap' }}>
        {types.map(tp => (
          <button key={tp} onClick={() => onType(tp)} style={{
            padding: '6px 10px', borderRadius: 999,
            background: tp === wp.type ? `${C.accent}26` : C.card2,
            border: `1px solid ${tp === wp.type ? C.accent + '66' : C.line}`,
            color: tp === wp.type ? C.accent : C.text2, fontSize: 11, fontWeight: 600, cursor: 'pointer',
            letterSpacing: 0.3,
          }}>{tp}</button>
        ))}
      </div>
    </div>
  );
}

function MapBackground({ style }) {
  if (style === 'satellite') {
    return (
      <div style={{ position:'absolute', inset: 0, background:
        `radial-gradient(40% 30% at 30% 30%, #2d3a2a 0%, #1a221c 100%),
         radial-gradient(60% 50% at 70% 60%, #1f2a32 0%, #0e1320 100%)`,
        }}>
        <svg viewBox="0 0 360 460" width="100%" height="100%" style={{ position:'absolute', inset: 0, opacity: 0.4 }}>
          {Array.from({length: 8}).map((_,i) => (
            <path key={i} d={`M${i*40-20},0 Q${i*40+10},${230} ${i*40},460`} stroke="rgba(120,180,140,0.15)" fill="none" />
          ))}
          {Array.from({length: 20}).map((_,i) => (
            <circle key={'t'+i} cx={Math.random()*360} cy={Math.random()*460} r={3+Math.random()*4} fill="rgba(80,110,70,0.4)" />
          ))}
        </svg>
      </div>
    );
  }
  if (style === 'grid') {
    return (
      <div style={{ position:'absolute', inset: 0, background: `linear-gradient(180deg, #0a0d12, #0c1018)`, overflow:'hidden' }}>
        <svg width="100%" height="100%" viewBox="0 0 360 460" style={{ position:'absolute', inset: 0 }}>
          <defs>
            <pattern id="grid" width="30" height="30" patternUnits="userSpaceOnUse">
              <path d="M30 0 L0 0 0 30" stroke="rgba(34,211,238,0.08)" strokeWidth="0.5" fill="none"/>
            </pattern>
            <pattern id="bigGrid" width="120" height="120" patternUnits="userSpaceOnUse">
              <path d="M120 0 L0 0 0 120" stroke="rgba(34,211,238,0.16)" strokeWidth="0.8" fill="none"/>
            </pattern>
          </defs>
          <rect width="100%" height="100%" fill="url(#grid)" />
          <rect width="100%" height="100%" fill="url(#bigGrid)" />
        </svg>
      </div>
    );
  }
  // dark "city" map (default)
  return (
    <div style={{ position:'absolute', inset: 0, background: '#0a0d12', overflow:'hidden' }}>
      <svg width="100%" height="100%" viewBox="0 0 360 460" style={{ position:'absolute', inset: 0 }}>
        {/* roads */}
        <path d="M0,80 L360,80" stroke="rgba(255,255,255,0.06)" strokeWidth="14"/>
        <path d="M0,200 L360,220" stroke="rgba(255,255,255,0.05)" strokeWidth="10"/>
        <path d="M0,360 L360,340" stroke="rgba(255,255,255,0.04)" strokeWidth="8"/>
        <path d="M100,0 L100,460" stroke="rgba(255,255,255,0.05)" strokeWidth="10"/>
        <path d="M240,0 L260,460" stroke="rgba(255,255,255,0.04)" strokeWidth="8"/>
        {/* blocks */}
        {[[30,100,60,80],[140,110,80,70],[270,100,80,80],[30,230,60,100],[140,240,80,100],[270,240,80,100],[30,370,60,80],[140,360,80,80],[270,360,80,80]].map(([x,y,w,h], i) => (
          <rect key={i} x={x} y={y} width={w} height={h} fill="rgba(255,255,255,0.03)" stroke="rgba(255,255,255,0.05)" />
        ))}
        {/* labels */}
        <text x="50" y="60" fontSize="9" fill="rgba(255,255,255,0.18)" fontFamily="var(--mono)">PINE ST</text>
        <text x="50" y="190" fontSize="9" fill="rgba(255,255,255,0.18)" fontFamily="var(--mono)">OAK AVE</text>
        <text x="180" y="350" fontSize="9" fill="rgba(255,255,255,0.18)" fontFamily="var(--mono)">STUDIO LOT</text>
      </svg>
    </div>
  );
}

window.MapScreen = MapScreen;
