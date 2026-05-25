// Sub-screens (split for size)
function CalibrateScreen() {
  const [step, setStep] = React.useState(0);
  const stages = [
    { t:'Compass', desc:'Hold the rover and rotate it through all six axes.', icon: <I.compass size={28}/>, color: C.accent },
    { t:'Accelerometer', desc:'Place the rover on each of its six faces, hold still.', icon: <I.gauge size={28}/>, color: C.violet },
    { t:'Gyroscope', desc:'Place on flat ground and keep perfectly still.', icon: <I.target size={28}/>, color: C.warn },
    { t:'Pen actuator', desc:'Verify pen rests at Z=5mm and stamps at Z=0.', icon: <I.draw size={28}/>, color: C.accent2 },
  ];
  const s = stages[step];
  const r = useRover();
  return (
    <SubScreen title="Calibration" subtitle={`${step+1} of ${stages.length} · ${s.t}`}>
      <div style={{ padding: '0 16px' }}>
        <div style={{ display:'flex', gap: 4, marginBottom: 14 }}>
          {stages.map((x, i) => (
            <div key={i} style={{
              flex: 1, height: 3, borderRadius: 2,
              background: i <= step ? s.color : 'rgba(255,255,255,0.08)',
              transition: 'background .2s',
            }}/>
          ))}
        </div>

        <Card pad={0} style={{ overflow:'hidden' }}>
          <div style={{ aspectRatio: '4/3', position: 'relative', overflow:'hidden',
                        background: `radial-gradient(50% 50% at 50% 40%, ${s.color}22 0%, transparent 70%), #0a0d12` }}>
            <CalibrateAnim step={step} color={s.color} />
          </div>
          <div style={{ padding: 16 }}>
            <div style={{ display:'flex', alignItems:'center', gap: 12 }}>
              <div style={{ width: 48, height: 48, borderRadius: 12, background: `${s.color}22`, color: s.color,
                            display:'flex', alignItems:'center', justifyContent:'center' }}>{s.icon}</div>
              <div>
                <div style={{ fontSize: 16, fontWeight: 700 }}>{s.t}</div>
                <div style={{ fontSize: 12, color: C.text3, marginTop: 2 }}>{s.desc}</div>
              </div>
            </div>
            <div style={{ marginTop: 14, padding: '10px 12px', background: C.card2, borderRadius: 10, fontSize: 12, color: C.text2 }}>
              <I.info size={12} color={s.color} style={{verticalAlign:-2, marginRight: 6}}/>
              Sample bias σ converging · {Math.round(20 + step * 18)}% complete
            </div>
            <div style={{ marginTop: 14, display:'flex', gap: 8 }}>
              {step > 0 && <Btn variant="secondary" full onClick={() => setStep(step-1)}>Back</Btn>}
              <Btn variant="primary" full onClick={() => step === stages.length - 1 ? r.pop() : setStep(step+1)}>
                {step === stages.length - 1 ? 'Finish' : 'Next'}
              </Btn>
            </div>
          </div>
        </Card>
      </div>
    </SubScreen>
  );
}

function CalibrateAnim({ step, color }) {
  // Spinning 3D-ish rover wireframe
  return (
    <svg viewBox="0 0 240 180" width="100%" height="100%">
      {/* grid floor */}
      {Array.from({length: 6}).map((_,i) => (
        <line key={i} x1={0} y1={120 + i*10} x2={240} y2={120 + i*10} stroke={`${color}33`} strokeWidth="0.5"/>
      ))}
      {/* rover wireframe */}
      <g transform="translate(120 100)" style={{ animation: `pxsweep ${step === 2 ? '999' : '4'}s linear infinite`, transformOrigin: '120px 100px' }}>
        <rect x="-40" y="-20" width="80" height="40" fill="none" stroke={color} strokeWidth="1.5"/>
        <rect x="-44" y="-26" width="14" height="10" fill="none" stroke={color} strokeWidth="1"/>
        <rect x="30"  y="-26" width="14" height="10" fill="none" stroke={color} strokeWidth="1"/>
        <rect x="-44" y="16"  width="14" height="10" fill="none" stroke={color} strokeWidth="1"/>
        <rect x="30"  y="16"  width="14" height="10" fill="none" stroke={color} strokeWidth="1"/>
        <circle cx="0" cy="0" r="6" fill={color} opacity="0.5"/>
        <line x1="0" y1="0" x2="0" y2="-14" stroke={color} strokeWidth="2"/>
      </g>
      {/* axis arrows */}
      <g transform="translate(40 40)">
        <line x1="0" y1="0" x2="22" y2="0" stroke={C.danger} strokeWidth="1.5" markerEnd="url(#xa)"/>
        <line x1="0" y1="0" x2="0" y2="22" stroke={C.good} strokeWidth="1.5" markerEnd="url(#xa)"/>
        <line x1="0" y1="0" x2="-12" y2="-12" stroke={C.accent} strokeWidth="1.5" markerEnd="url(#xa)"/>
        <text x="26" y="3" fontSize="9" fill={C.danger} fontFamily="var(--mono)">x</text>
        <text x="3" y="28" fontSize="9" fill={C.good} fontFamily="var(--mono)">y</text>
        <text x="-22" y="-14" fontSize="9" fill={C.accent} fontFamily="var(--mono)">z</text>
      </g>
    </svg>
  );
}

// ─────────────────────────────────────────────
// LOGS
// ─────────────────────────────────────────────
function LogsScreen() {
  const r = useRover();
  const [filter, setFilter] = React.useState('all');
  const filtered = filter === 'all' ? r.logs : r.logs.filter(l => l.lvl === filter);

  return (
    <SubScreen title="Logs" subtitle="rosout · MAVLink · uORB"
               trailing={<IconBtn icon={<I.download size={18}/>}/>}>
      <div style={{ padding: '0 16px 12px' }}>
        <div style={{ display:'flex', gap: 6, background: C.card, padding: 4, borderRadius: 12, border: `1px solid ${C.line}` }}>
          {[
            { k:'all', l:'All', c: C.text2 },
            { k:'info', l:'Info', c: C.text2 },
            { k:'warn', l:'Warn', c: C.warn },
            { k:'error', l:'Error', c: C.danger },
            { k:'debug', l:'Debug', c: C.violet },
          ].map(x => (
            <button key={x.k} onClick={() => setFilter(x.k)} style={{
              flex: 1, padding: '8px 6px', border: 'none', borderRadius: 9, cursor: 'pointer',
              background: filter === x.k ? `${x.c}22` : 'transparent',
              color: filter === x.k ? x.c : C.text2,
              fontSize: 11, fontWeight: 600,
            }}>{x.l}</button>
          ))}
        </div>
      </div>
      <div style={{ padding: '0 16px' }}>
        <Card pad={0} style={{ overflow: 'hidden' }}>
          {filtered.map((l, i) => {
            const c = l.lvl === 'error' ? C.danger : l.lvl === 'warn' ? C.warn : l.lvl === 'debug' ? C.violet : C.good;
            return (
              <div key={i} style={{ padding: '10px 12px', borderBottom: i < filtered.length - 1 ? `1px solid ${C.line}` : 'none', fontFamily: 'var(--mono)', fontSize: 11 }}>
                <div style={{ display:'flex', gap: 8 }}>
                  <span style={{ color: C.text3 }}>{l.t}</span>
                  <span style={{ color: c, textTransform: 'uppercase', fontWeight: 700, letterSpacing: 0.5, width: 38 }}>{l.lvl}</span>
                  <span style={{ color: C.accent }}>{l.src}</span>
                </div>
                <div style={{ color: C.text, marginTop: 3, marginLeft: 80 }}>{l.msg}</div>
              </div>
            );
          })}
        </Card>
      </div>
    </SubScreen>
  );
}

// ─────────────────────────────────────────────
// FIRMWARE
// ─────────────────────────────────────────────
function FirmwareScreen() {
  const [pkg, setPkg] = React.useState([
    { n: 'px4_msgs', cur: '1.14.0', latest: '1.15.0', kind: 'apt', ready: true },
    { n: 'ros-humble-desktop', cur: '0.10.5', latest: '0.10.6', kind: 'apt', ready: true },
    { n: 'mavros', cur: '2.6.0', latest: '2.6.0', kind: 'apt', ready: false },
    { n: 'svg2gcode', cur: '0.4.1', latest: '0.5.0', kind: 'pip', ready: true },
    { n: 'PX4 firmware', cur: '1.15.0', latest: '1.15.2', kind: 'OTA', ready: true },
  ]);
  const r = useRover();
  const updates = pkg.filter(p => p.ready);

  return (
    <SubScreen title="Firmware & Packages" subtitle={`${updates.length} updates available`}>
      <div style={{ padding: '0 16px 14px' }}>
        <Card pad={14} style={{ background: `linear-gradient(135deg, ${C.good}22, ${C.card})`, border: `1px solid ${C.good}33` }}>
          <div style={{ display:'flex', alignItems:'center', gap: 12 }}>
            <div style={{ width: 44, height: 44, borderRadius: 12, background: `${C.good}33`, color: C.good,
                          display:'flex', alignItems:'center', justifyContent:'center' }}><I.firmware size={22}/></div>
            <div style={{ flex: 1 }}>
              <div style={{ fontWeight: 700 }}>System healthy</div>
              <div style={{ fontSize: 12, color: C.text2, marginTop: 2 }}>Last update 2 days ago · Backup OK</div>
            </div>
            <Btn variant="primary" size="sm" onClick={() => setPkg(p => p.map(x => ({...x, cur: x.latest, ready: false})))}>
              Update all
            </Btn>
          </div>
        </Card>
      </div>

      <SectionHeader title="Packages" />
      <div style={{ padding: '0 16px' }}>
        <Card pad={0}>
          {pkg.map((p, i) => (
            <div key={p.n} style={{ padding: '12px 14px', borderBottom: i < pkg.length - 1 ? `1px solid ${C.line}` : 'none', display: 'flex', alignItems:'center', gap: 10 }}>
              <span style={{
                fontSize: 9, fontWeight: 700, padding: '3px 6px', borderRadius: 4,
                color: p.kind === 'OTA' ? C.violet : p.kind === 'pip' ? C.warn : C.accent,
                background: p.kind === 'OTA' ? `${C.violet}22` : p.kind === 'pip' ? `${C.warn}22` : `${C.accent}22`,
                letterSpacing: 0.5,
              }}>{p.kind}</span>
              <div style={{ flex: 1, minWidth: 0 }}>
                <div style={{ fontFamily:'var(--mono)', fontSize: 13, fontWeight: 500 }}>{p.n}</div>
                <div style={{ fontSize: 11, color: C.text3, fontFamily:'var(--mono)' }}>
                  {p.cur} {p.ready && <span style={{ color: C.good }}>→ {p.latest}</span>}
                </div>
              </div>
              {p.ready ? (
                <Btn variant="accentGhost" size="sm" onClick={() => setPkg(arr => arr.map(x => x.n === p.n ? {...x, cur: x.latest, ready: false} : x))}>Update</Btn>
              ) : (
                <I.check size={16} color={C.good} />
              )}
            </div>
          ))}
        </Card>
      </div>
    </SubScreen>
  );
}

// ─────────────────────────────────────────────
// FLEET
// ─────────────────────────────────────────────
function FleetScreen() {
  const r = useRover();
  return (
    <SubScreen title="Fleet" subtitle={`${r.fleet.length} rovers · ${r.fleet.filter(f => f.status==='connected').length} live`}
               trailing={<IconBtn icon={<I.plus size={18}/>}/>}>
      <div style={{ padding: '0 16px' }}>
        {r.fleet.map(f => {
          const stColor = { connected: C.good, standby: C.accent, charging: C.warn, offline: C.text3 }[f.status];
          const active = f.id === r.activeRover;
          return (
            <div key={f.id} style={{ marginBottom: 10 }}>
              <Card pad={14} accent={active ? C.accent : null}>
                <div style={{ display:'flex', alignItems:'center', gap: 12 }}>
                  <div style={{
                    width: 48, height: 48, borderRadius: 12,
                    background: active ? `linear-gradient(135deg, ${C.accent}33, ${C.card2})` : C.card2,
                    border: `1px solid ${active ? C.accent + '66' : C.line}`,
                    color: active ? C.accent : C.text2,
                    display:'flex', alignItems:'center', justifyContent:'center',
                  }}><I.drive size={22}/></div>
                  <div style={{ flex: 1, minWidth: 0 }}>
                    <div style={{ display:'flex', alignItems:'center', gap: 8 }}>
                      <span style={{ fontSize: 15, fontWeight: 600 }}>{f.name}</span>
                      {active && <Pill color={C.accent} dim>active</Pill>}
                    </div>
                    <div style={{ display:'flex', gap: 12, marginTop: 4, color: C.text3, fontSize: 12, fontFamily:'var(--mono)' }}>
                      <span><Dot color={stColor} size={6} /> <span style={{ color: stColor, marginLeft: 4 }}>{f.status}</span></span>
                      <span><I.battery size={11} style={{verticalAlign:-1, marginRight:3}}/>{f.battery}%</span>
                      <span>{f.loc}</span>
                    </div>
                  </div>
                  {!active && <Btn variant="secondary" size="sm" onClick={() => r.setActiveRover(f.id)}>Switch</Btn>}
                </div>
              </Card>
            </div>
          );
        })}
        <Btn variant="ghost" full icon={<I.plus size={16}/>} onClick={() => r.push('connect')}>Add a rover</Btn>
      </div>
    </SubScreen>
  );
}

// ─────────────────────────────────────────────
// SETTINGS
// ─────────────────────────────────────────────
function SettingsScreen() {
  const r = useRover();
  const [voice, setVoice] = React.useState(true);
  const [haptics, setHaptics] = React.useState(true);
  const [units, setUnits] = React.useState('Metric');
  return (
    <SubScreen title="Settings">
      <div style={{ padding: '0 16px' }}>
        <SectionHeader title="Account" />
        <Card pad={14}>
          <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
            <div style={{ width: 48, height: 48, borderRadius: '50%', background: `linear-gradient(135deg, ${C.accent}, ${C.violet})`, color: '#0a0d12',
                          display:'flex', alignItems:'center', justifyContent:'center', fontWeight: 800 }}>SK</div>
            <div style={{ flex: 1 }}>
              <div style={{ fontSize: 15, fontWeight: 600 }}>Sayak</div>
              <div style={{ fontSize: 12, color: C.text3 }}>operator · 3 rovers · API token …f29c</div>
            </div>
            <I.chevR size={16} color={C.text3} />
          </div>
        </Card>

        <SectionHeader title="Preferences" />
        <Card pad={0}>
          <SettingsRow title="Voice feedback" sub="Announce mode changes" right={<Toggle value={voice} onChange={setVoice}/>} />
          <SettingsRow title="Haptics on warn/error" right={<Toggle value={haptics} onChange={setHaptics}/>} />
          <SettingsRow title="Units" right={
            <div style={{ display:'flex', gap: 4, background: C.card2, padding: 3, borderRadius: 8 }}>
              {['Metric','Imperial'].map(u => (
                <button key={u} onClick={() => setUnits(u)} style={{
                  padding: '4px 10px', borderRadius: 6, fontSize: 11, fontWeight: 600,
                  background: units === u ? C.accent + '33' : 'transparent',
                  color: units === u ? C.accent : C.text2, border: 'none', cursor: 'pointer',
                }}>{u}</button>
              ))}
            </div>
          } />
        </Card>

        <SectionHeader title="Developer" />
        <Card pad={0}>
          <Row icon={<I.terminal size={18}/>} iconBg={`${C.violet}1c`} iconColor={C.violet} title="API tokens" sub="ros://*.dxp.local"/>
          <div style={{ height: 1, background: C.line, margin: '0 14px 0 64px' }}/>
          <Row icon={<I.copy size={18}/>} iconBg={`${C.text3}1c`} iconColor={C.text3} title="Export configuration" sub=".yaml + .params + rosbag"/>
          <div style={{ height: 1, background: C.line, margin: '0 14px 0 64px' }}/>
          <Row icon={<I.refresh size={18}/>} iconBg={`${C.text3}1c`} iconColor={C.text3} title="Reset to defaults"/>
        </Card>

        <div style={{ marginTop: 16, marginBottom: 24 }}>
          <Btn variant="ghost" full style={{ color: C.danger, borderColor: `${C.danger}33` }} icon={<I.power size={16}/>}>
            Sign out
          </Btn>
        </div>
      </div>
    </SubScreen>
  );
}

function SettingsRow({ title, sub, right }) {
  return (
    <div style={{ padding: '12px 14px', display:'flex', alignItems:'center', gap: 10, borderBottom: `1px solid ${C.line}` }}>
      <div style={{ flex: 1 }}>
        <div style={{ fontSize: 14, fontWeight: 500 }}>{title}</div>
        {sub && <div style={{ fontSize: 11, color: C.text3, marginTop: 1 }}>{sub}</div>}
      </div>
      {right}
    </div>
  );
}


Object.assign(window, { CalibrateScreen, LogsScreen, FirmwareScreen, FleetScreen, SettingsScreen });
