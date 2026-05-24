// Home / Dashboard
function HomeScreen() {
  const r = useRover();
  const { t } = r;

  return (
    <div style={{ padding: '0 0 100px' }}>
      <AppBar
        title="DXP"
        subtitle="Drawing Rover Workbench"
        leading={<IconBtn icon={<I.menu size={18} />} />}
        trailing={
          <div style={{ display:'flex', gap: 8 }}>
            <IconBtn icon={<I.bell size={18} />} badge={3} />
            <IconBtn icon={<I.cam size={18} />} accent />
          </div>
        }
      />

      {/* Rover hero card */}
      <div style={{ padding: '0 16px 12px' }}>
        <Card pad={16} style={{ background: `linear-gradient(135deg, #182234, #0e1623)`, border: `1px solid ${C.accent}26` }}>
          <div style={{ display:'flex', alignItems:'flex-start', justifyContent:'space-between' }}>
            <div>
              <div style={{ display:'flex', alignItems:'center', gap: 8, color: C.text3, fontSize: 11, textTransform:'uppercase', letterSpacing: 0.8, fontWeight: 600 }}>
                <Dot color={r.connected ? C.good : C.danger} />
                {r.connected ? 'Connected · Studio A' : 'Disconnected'}
                {r.backendConnected && <Pill color={C.accent} dim><Dot color={C.accent} size={4} pulse={false} /> LIVE</Pill>}
                {!r.backendConnected && <Pill color={C.text3} dim><Dot color={C.text3} size={4} pulse={false} /> MOCK</Pill>}
              </div>
              <div style={{ fontSize: 24, fontWeight: 700, letterSpacing: -0.4, marginTop: 4 }}>DXP-01 Mercutio</div>
              <div style={{ color: C.text2, fontSize: 13, marginTop: 2 }}>PX4 v1.15 · ROS 2 Humble · Domain 42</div>
            </div>
            <button onClick={() => r.setTab('more') || r.push('fleet')} style={{
              border: `1px solid ${C.line2}`, background: C.card2, color: C.text2,
              padding: '6px 10px', borderRadius: 999, fontSize: 12, fontWeight: 500, cursor: 'pointer',
              display: 'inline-flex', alignItems:'center', gap: 6,
            }}>
              <I.fleet size={14} /> Switch
            </button>
          </div>

          {/* Rover SVG silhouette + path trace */}
          <div style={{ marginTop: 12, height: 140, position: 'relative', borderRadius: 12, overflow: 'hidden',
                        background: `radial-gradient(120% 80% at 50% 100%, #1c2841, #0c1320)` }}>
            <svg viewBox="0 0 320 140" width="100%" height="100%" preserveAspectRatio="xMidYMid slice">
              <defs>
                <linearGradient id="trail" x1="0" x2="1">
                  <stop offset="0" stopColor={C.accent} stopOpacity="0" />
                  <stop offset="1" stopColor={C.accent} stopOpacity="1" />
                </linearGradient>
              </defs>
              {/* grid */}
              {Array.from({length: 8}).map((_,i)=>(
                <line key={'h'+i} x1={i*45} x2={i*45} y1={0} y2={140} stroke="rgba(255,255,255,0.04)" />
              ))}
              {Array.from({length: 5}).map((_,i)=>(
                <line key={'v'+i} x1={0} x2={320} y1={i*32} y2={i*32} stroke="rgba(255,255,255,0.04)" />
              ))}
              {/* drawn path trace */}
              <path d="M40,110 C80,80 110,100 140,70 S200,40 240,55 S290,95 280,115"
                    stroke="url(#trail)" strokeWidth="2.2" fill="none" strokeLinecap="round" />
              {/* rover icon (top-down) */}
              <g transform={`translate(280 115) rotate(${t.heading-90})`}>
                <rect x="-13" y="-9" width="26" height="18" rx="3" fill="#1a2738" stroke={C.accent} strokeWidth="1.5"/>
                <rect x="-15" y="-12" width="6" height="5" rx="1" fill="#0a0d12" stroke={C.text3} strokeWidth="0.8"/>
                <rect x="9"  y="-12" width="6" height="5" rx="1" fill="#0a0d12" stroke={C.text3} strokeWidth="0.8"/>
                <rect x="-15" y="7"  width="6" height="5" rx="1" fill="#0a0d12" stroke={C.text3} strokeWidth="0.8"/>
                <rect x="9"  y="7"  width="6" height="5" rx="1" fill="#0a0d12" stroke={C.text3} strokeWidth="0.8"/>
                <circle cx="0" cy="0" r="3" fill={C.accent} />
                <line x1="0" y1="0" x2="0" y2="-7" stroke={C.accent} strokeWidth="2" />
              </g>
              {/* heading sweep */}
              <circle cx="280" cy="115" r="22" fill="none" stroke={C.accent} strokeOpacity="0.3" strokeDasharray="2 3" />
            </svg>

            {/* overlay tag */}
            <div style={{ position:'absolute', top: 10, left: 12, display:'flex', gap: 6, alignItems:'center' }}>
              <Pill color={C.accent}><Dot color={C.accent} size={6} /> {r.mode.toUpperCase()}</Pill>
              {r.armed && <Pill color={C.warn} dim><I.unlock size={11} /> ARMED</Pill>}
              {r.emergency && <Pill color={C.danger}><I.warn size={11} /> E-STOP</Pill>}
            </div>
            <div style={{ position:'absolute', bottom: 10, right: 12, fontFamily:'var(--mono)', fontSize: 11, color: C.text3 }}>
              42.3651°N · 71.0589°W
            </div>
          </div>

          {/* Quick-stats strip */}
          <div style={{ marginTop: 12, display:'grid', gridTemplateColumns:'repeat(4, 1fr)', gap: 6 }}>
            {[
              { l:'BAT', v: Math.round(t.battery), u:'%', c: t.battery > 25 ? C.good : C.danger, ic:<I.battery size={12}/> },
              { l:'SAT', v: t.sats, u:'', c: C.accent, ic:<I.satellite size={12}/> },
              { l:'RSSI', v: Math.round(t.rssi), u:'dBm', c: C.text, ic:<I.wifi size={12}/> },
              { l:'HZ', v: t.hz, u:'', c: C.violet, ic:<I.zap size={12}/> },
            ].map(s => (
              <div key={s.l} style={{
                padding: '8px 10px', borderRadius: 10, background: 'rgba(0,0,0,0.25)',
                border: '1px solid rgba(255,255,255,0.05)',
              }}>
                <div style={{ display:'flex', justifyContent:'space-between', color: C.text3, fontSize: 10, fontWeight: 600, letterSpacing: 0.5 }}>
                  <span>{s.l}</span><span style={{ color: s.c }}>{s.ic}</span>
                </div>
                <div style={{ fontFamily:'var(--mono)', fontSize: 15, fontWeight: 600, color: s.c, marginTop: 2 }}>
                  {s.v}<span style={{ color: C.text3, fontSize: 10, marginLeft: 1 }}>{s.u}</span>
                </div>
              </div>
            ))}
          </div>
        </Card>
      </div>

      {/* Active job card */}
      <div style={{ padding: '4px 16px 12px' }}>
        <Card>
          <div style={{ display:'flex', alignItems:'center', justifyContent:'space-between' }}>
            <div style={{ display:'flex', alignItems:'center', gap: 12 }}>
              <div style={{ width: 44, height: 44, borderRadius: 10, background: C.card2, display:'flex', alignItems:'center', justifyContent:'center', color: C.accent }}>
                <I.draw size={20} />
              </div>
              <div>
                <div style={{ fontSize: 11, color: C.text3, textTransform:'uppercase', letterSpacing: 0.8, fontWeight: 600 }}>Active Job</div>
                <div style={{ fontWeight: 600, fontSize: 15 }}>{r.activeJob.name}</div>
              </div>
            </div>
            <IconBtn icon={r.mode === 'Draw' ? <I.pause size={16} /> : <I.play size={16} />} accent onClick={r.togglePlay} />
          </div>
          <div style={{ marginTop: 12 }}>
            <div style={{ display:'flex', justifyContent:'space-between', fontSize: 12, color: C.text2, marginBottom: 6, fontFamily: 'var(--mono)' }}>
              <span>{Math.round(r.drawProgress * 100)}% · {Math.round(r.drawProgress * r.activeJob.paths)}/{r.activeJob.paths} paths</span>
              <span>ETA {r.activeJob.eta}</span>
            </div>
            <Bar value={r.drawProgress*100} color={C.accent} />
          </div>
          {r.dxfFile && r.activeJob.id === 'dxf' && (
            <div style={{ marginTop: 12, paddingTop: 12, borderTop: `1px solid ${C.line}`,
                          display:'flex', alignItems:'center', justifyContent:'space-between', gap: 10 }}>
              <div style={{ minWidth: 0 }}>
                <div style={{ fontSize: 11, color: C.text3, fontWeight: 600, textTransform:'uppercase', letterSpacing: 0.6 }}>DXF source</div>
                <div style={{ fontSize: 12, fontFamily:'var(--mono)', color: C.text2, marginTop: 2, overflow:'hidden', textOverflow:'ellipsis', whiteSpace:'nowrap' }}>
                  {r.dxfFile.name} · {r.dxfSelected?.size || 0}/{r.dxfFile.entities.length}
                </div>
              </div>
              <Btn variant="accentGhost" size="sm" icon={<I.sliders size={13}/>}
                   onClick={() => r.setDxfInspectorOpen(true)}>
                Edit
              </Btn>
            </div>
          )}
        </Card>
      </div>

      {/* Quick actions */}
      <SectionHeader title="Quick actions" />
      <div style={{ padding: '0 16px 16px', display:'grid', gridTemplateColumns:'repeat(2, 1fr)', gap: 10 }}>
        {[
          { i: <I.drive size={20}/>, t:'Manual drive', s:'Joystick · keys',  c: C.accent, k:'drive', a: () => r.setTab('drive') },
          { i: <I.draw size={20}/>,  t:'New drawing',  s:'SVG · canvas · G-code', c: C.violet, k:'draw', a: () => r.setTab('draw') },
          { i: <I.map size={20}/>,   t:'Plan mission', s:'Waypoints · trace', c: C.accent2, k:'map', a: () => r.setTab('map') },
          { i: <I.cam size={20}/>,   t:'Live camera',  s:'1080p · 30 fps', c: C.warn, k:'cam', a: () => r.push('camera') },
        ].map(q => (
          <button key={q.k} onClick={q.a} style={{
            display:'flex', flexDirection:'column', alignItems:'flex-start', gap: 8,
            padding: 14, background: C.card, border: `1px solid ${C.line}`, borderRadius: 16, cursor: 'pointer',
            color: C.text, textAlign: 'left',
          }}>
            <span style={{ color: q.c, width: 38, height: 38, borderRadius: 10, background: `${q.c}1c`, display:'inline-flex', alignItems:'center', justifyContent:'center' }}>{q.i}</span>
            <div>
              <div style={{ fontSize: 14, fontWeight: 600 }}>{q.t}</div>
              <div style={{ fontSize: 11, color: C.text3, marginTop: 2 }}>{q.s}</div>
            </div>
          </button>
        ))}
      </div>

      {/* System diagnostics row */}
      <SectionHeader title="System" action={{ label: 'Open', onClick: () => r.push('ros') }} />
      <div style={{ padding: '0 16px 12px' }}>
        <Card pad={14}>
          <div style={{ display:'grid', gridTemplateColumns:'repeat(3, 1fr)', gap: 12 }}>
            <SysTile label="ROS2 nodes" value={r.rosNodes.length} ok="18/18" />
            <SysTile label="uORB" value="245 Hz" ok="ok" />
            <SysTile label="EKF2" value="locked" ok="ok" />
            <SysTile label="Geofence" value="active" ok="3 zones" />
            <SysTile label="Pen" value="up" />
            <SysTile label="Storage" value="62%" warn />
          </div>
          <div style={{ display:'flex', gap: 8, marginTop: 12 }}>
            <Btn variant="secondary" size="sm" icon={<I.terminal size={14}/>} onClick={() => r.push('logs')}>Logs</Btn>
            <Btn variant="secondary" size="sm" icon={<I.cpu size={14}/>} onClick={() => r.push('ros')}>Nodes</Btn>
            <Btn variant="secondary" size="sm" icon={<I.sliders size={14}/>} onClick={() => r.push('px4')}>Params</Btn>
          </div>
        </Card>
      </div>

      {/* Calibration nudge */}
      <SectionHeader title="Maintenance" />
      <div style={{ padding: '0 16px 16px' }}>
        <Card pad={14} style={{ borderColor: `${C.warn}33` }}>
          <div style={{ display:'flex', alignItems:'center', gap: 12 }}>
            <div style={{ width: 40, height: 40, borderRadius: 10, background: `${C.warn}22`, color: C.warn,
                          display:'inline-flex', alignItems:'center', justifyContent:'center' }}>
              <I.warn size={18} />
            </div>
            <div style={{ flex: 1 }}>
              <div style={{ fontWeight: 600, fontSize: 14 }}>Compass needs calibration</div>
              <div style={{ color: C.text3, fontSize: 12, marginTop: 1 }}>Mag offset drift detected · 12 days since last cal</div>
            </div>
            <Btn variant="warn" size="sm" onClick={() => r.push('calibrate')}>Run</Btn>
          </div>
        </Card>
      </div>

      <SectionHeader title="Fleet" action={{ label: 'View all', onClick: () => r.push('fleet') }} />
      <div style={{ padding: '0 16px 32px' }}>
        <div style={{ display:'flex', gap: 10, overflowX:'auto', paddingBottom: 4, marginLeft: -2, marginRight: -2, paddingLeft: 2, paddingRight: 2 }}>
          {r.fleet.map(f => (
            <FleetCardSmall key={f.id} f={f} active={f.id === r.activeRover} onClick={() => r.setActiveRover(f.id)} />
          ))}
        </div>
      </div>
    </div>
  );
}

function SysTile({ label, value, ok, warn }) {
  const color = warn ? C.warn : ok ? C.good : C.text;
  return (
    <div style={{ padding: '8px 10px', borderRadius: 10, background: 'rgba(255,255,255,0.025)', border: `1px solid ${C.line}` }}>
      <div style={{ fontSize: 10, color: C.text3, textTransform:'uppercase', letterSpacing: 0.7, fontWeight: 600 }}>{label}</div>
      <div style={{ display:'flex', alignItems:'center', gap: 4, marginTop: 2 }}>
        <Dot color={color} size={6} pulse={!warn} />
        <span style={{ fontSize: 13, fontWeight: 600, color, fontFamily: 'var(--mono)' }}>{value}</span>
      </div>
    </div>
  );
}

function FleetCardSmall({ f, active, onClick }) {
  const stColor = { connected: C.good, standby: C.accent, charging: C.warn, offline: C.text3 }[f.status];
  return (
    <button onClick={onClick} style={{
      display:'flex', flexDirection:'column', justifyContent:'space-between',
      minWidth: 150, padding: 12, borderRadius: 14,
      background: active ? `linear-gradient(135deg, ${C.accent}1c, ${C.card})` : C.card,
      border: `1px solid ${active ? C.accent + '4c' : C.line}`,
      color: C.text, cursor: 'pointer', textAlign:'left', flexShrink: 0,
    }}>
      <div style={{ display:'flex', alignItems:'center', gap: 6, color: stColor, fontSize: 10, fontWeight: 600, textTransform: 'uppercase', letterSpacing: 0.6 }}>
        <Dot color={stColor} size={6} pulse={f.status === 'connected'} />{f.status}
      </div>
      <div style={{ marginTop: 8 }}>
        <div style={{ fontWeight: 600, fontSize: 13 }}>{f.name.split(' ')[0]}</div>
        <div style={{ color: C.text3, fontSize: 11 }}>{f.name.split(' ').slice(1).join(' ')}</div>
      </div>
      <div style={{ marginTop: 8, display:'flex', alignItems:'center', justifyContent:'space-between' }}>
        <span style={{ fontSize: 11, color: C.text3, fontFamily:'var(--mono)' }}><I.battery size={11} style={{verticalAlign:-2, marginRight: 4}}/>{f.battery}%</span>
        <I.chevR size={14} color={C.text3} />
      </div>
    </button>
  );
}

window.HomeScreen = HomeScreen;
