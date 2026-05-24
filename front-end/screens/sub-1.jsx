// Sub-screens (split for size)
// Sub-screens pushed atop tabs: connect, camera, ros, px4, calibrate, logs,
// firmware, fleet, settings. Each is a full screen with a back navigation.

function SubScreen({ title, subtitle, children, trailing }) {
  const r = useRover();
  return (
    <div style={{ padding: '0 0 100px' }}>
      <AppBar
        title={title}
        subtitle={subtitle}
        leading={<IconBtn icon={<I.chevL size={18}/>} onClick={r.pop} />}
        trailing={trailing}
      />
      {children}
    </div>
  );
}

// ─────────────────────────────────────────────
// CONNECT FLOW
// ─────────────────────────────────────────────
function ConnectScreen() {
  const r = useRover();
  const [scanning, setScanning] = React.useState(true);
  const [step, setStep] = React.useState('scan'); // scan | connect | done
  React.useEffect(() => {
    if (step !== 'scan') return;
    const id = setTimeout(() => setScanning(false), 1800);
    return () => clearTimeout(id);
  }, [step]);

  const discovered = [
    { id:'dxp-01', name:'DXP-01 Mercutio', kind:'Wi-Fi', signal: -52, sec:'WPA3' },
    { id:'dxp-02', name:'DXP-02 Ophelia',  kind:'Wi-Fi', signal: -64, sec:'WPA3' },
    { id:'dxp-03', name:'DXP-03 Puck',     kind:'BT 5.3', signal: -71, sec:'pair' },
  ];

  if (step === 'connect') {
    return (
      <SubScreen title="Connecting…" subtitle="DXP-01 Mercutio">
        <div style={{ padding: '40px 16px', display:'flex', flexDirection:'column', alignItems:'center', gap: 18 }}>
          <RadarLoader />
          <div style={{ textAlign:'center' }}>
            <div style={{ fontSize: 14, color: C.text2 }}>Pairing over Wi-Fi · 5GHz</div>
            <div style={{ fontFamily:'var(--mono)', fontSize: 11, color: C.text3, marginTop: 6 }}>
              Negotiating WPA3 · ROS 2 DDS discovery · MAVLink heartbeat
            </div>
          </div>
          <Btn variant="primary" onClick={() => setStep('done')}>Continue</Btn>
        </div>
      </SubScreen>
    );
  }

  if (step === 'done') {
    return (
      <SubScreen title="Connected" subtitle="DXP-01 Mercutio · ready">
        <div style={{ padding: '40px 16px', display:'flex', flexDirection:'column', alignItems:'center', gap: 18 }}>
          <div style={{
            width: 96, height: 96, borderRadius: '50%', background: `${C.good}26`,
            display:'flex', alignItems:'center', justifyContent:'center', color: C.good,
          }}>
            <I.check size={44} />
          </div>
          <div style={{ textAlign:'center' }}>
            <div style={{ fontSize: 18, fontWeight: 700 }}>You're online</div>
            <div style={{ fontSize: 12, color: C.text3, marginTop: 4 }}>
              18 ROS nodes alive · PX4 v1.15 · 245 Hz uORB · 12 sats locked
            </div>
          </div>
          <div style={{ width: '100%', maxWidth: 280 }}>
            <Btn variant="primary" full onClick={() => { r.setConnected(true); r.pop(); }}>Open dashboard</Btn>
          </div>
        </div>
      </SubScreen>
    );
  }

  return (
    <SubScreen title="Connect a rover" subtitle={scanning ? 'Scanning nearby…' : `${discovered.length} devices found`}
               trailing={<IconBtn icon={<I.refresh size={18}/>} onClick={() => setScanning(true)} />}>
      <div style={{ padding: '0 16px' }}>
        <Card pad={0}>
          {discovered.map((d, i) => (
            <React.Fragment key={d.id}>
              <Row
                icon={d.kind === 'Wi-Fi' ? <I.wifi size={18}/> : <I.bt size={18}/>}
                iconBg={`${C.accent}1c`} iconColor={C.accent}
                title={d.name}
                sub={`${d.kind} · ${d.signal} dBm · ${d.sec}`}
                detail={d.signal > -60 ? 'Strong' : 'OK'}
                chevron={false}
                onClick={() => setStep('connect')}
              />
              {i < discovered.length - 1 && <div style={{ height: 1, background: C.line, margin: '0 14px 0 64px' }} />}
            </React.Fragment>
          ))}
        </Card>
        {scanning && <ScanningStripe />}

        <SectionHeader title="Other methods" />
        <Card pad={0}>
          <Row icon={<I.disk size={18}/>} iconBg={`${C.text3}1c`} iconColor={C.text3} title="USB · serial" sub="/dev/ttyUSB0 · 921600" onClick={() => setStep('connect')} />
          <div style={{ height: 1, background: C.line, margin: '0 14px 0 64px' }}/>
          <Row icon={<I.terminal size={18}/>} iconBg={`${C.text3}1c`} iconColor={C.text3} title="Manual IP" sub="ros://192.168.0.x:8765" onClick={() => setStep('connect')} />
        </Card>
      </div>
    </SubScreen>
  );
}

function RadarLoader() {
  return (
    <div style={{ width: 120, height: 120, position: 'relative' }}>
      {[40, 70, 100].map(s => (
        <div key={s} style={{
          position:'absolute', top: '50%', left: '50%', width: s, height: s,
          marginLeft: -s/2, marginTop: -s/2, borderRadius: '50%',
          border: `1px solid ${C.accent}`, opacity: 0.4,
        }}/>
      ))}
      <svg viewBox="0 0 120 120" style={{ position:'absolute', inset: 0, animation: 'pxsweep 1.6s linear infinite', transformOrigin:'60px 60px' }}>
        <defs>
          <radialGradient id="rgrad" cx="60" cy="60" r="60" gradientUnits="userSpaceOnUse">
            <stop offset="0" stopColor={C.accent} stopOpacity="0"/>
            <stop offset="1" stopColor={C.accent} stopOpacity="0.6"/>
          </radialGradient>
        </defs>
        <path d="M60,60 L120,60 A60,60 0 0 0 100,18 Z" fill="url(#rgrad)" />
      </svg>
      <div style={{ position:'absolute', top: '50%', left: '50%', width: 10, height: 10, marginLeft:-5, marginTop:-5, background: C.accent, borderRadius: '50%', boxShadow: `0 0 12px ${C.accent}` }}/>
    </div>
  );
}

function ScanningStripe() {
  return (
    <div style={{
      marginTop: 12, fontSize: 11, color: C.text3, fontFamily:'var(--mono)',
      display:'flex', alignItems:'center', gap: 8, justifyContent:'center',
    }}>
      <Dot color={C.accent} size={6}/> Listening on UDP 14550 · DDS multicast 239.255.0.1
    </div>
  );
}

// ─────────────────────────────────────────────
// CAMERA
// ─────────────────────────────────────────────
function CameraScreen() {
  const r = useRover();
  const { t } = r;
  const [rec, setRec] = React.useState(false);
  return (
    <SubScreen title="Camera" subtitle={`1080p · 30 fps · ${rec ? 'REC' : 'preview'}`}
               trailing={<IconBtn icon={<I.maximize size={18}/>} />}>
      <div style={{ padding: '0 16px' }}>
        <div style={{
          position:'relative', borderRadius: 18, overflow: 'hidden',
          border: `1px solid ${C.line2}`, aspectRatio: '16/10',
          background: `linear-gradient(180deg, #1a2540 0%, #0a0d12 60%, #15201c 100%)`,
        }}>
          {/* Faux scene */}
          <svg width="100%" height="100%" style={{ position:'absolute', inset: 0 }} viewBox="0 0 320 200" preserveAspectRatio="xMidYMid slice">
            {/* sky */}
            <rect width="320" height="120" fill="url(#skyG)"/>
            <defs>
              <linearGradient id="skyG" x1="0" x2="0" y1="0" y2="1">
                <stop offset="0" stopColor="#2a4060"/>
                <stop offset="1" stopColor="#0b1320"/>
              </linearGradient>
              <linearGradient id="floorG" x1="0" x2="0" y1="0" y2="1">
                <stop offset="0" stopColor="#1a2218"/>
                <stop offset="1" stopColor="#080a0c"/>
              </linearGradient>
            </defs>
            <rect y="120" width="320" height="80" fill="url(#floorG)"/>
            {/* far buildings */}
            <rect x="20" y="80" width="40" height="40" fill="#101820"/>
            <rect x="80" y="60" width="60" height="60" fill="#0e1622"/>
            <rect x="160" y="70" width="50" height="50" fill="#101a26"/>
            <rect x="220" y="50" width="80" height="70" fill="#0f1925"/>
            {/* horizon line */}
            <line x1="0" y1="120" x2="320" y2="120" stroke="rgba(255,255,255,0.1)"/>
            {/* drawn path on floor */}
            <path d="M40,180 Q160,140 280,170" stroke={C.accent} strokeWidth="2" fill="none" strokeDasharray="3 3"/>
            {/* lens distortion edges */}
            <circle cx="160" cy="100" r="180" fill="none" stroke="rgba(0,0,0,0.6)" strokeWidth="40" opacity="0.5"/>
          </svg>

          {/* HUD overlay */}
          <div style={{ position:'absolute', top: 10, left: 10, display:'flex', flexDirection:'column', gap: 4 }}>
            {rec && <div style={{ display:'inline-flex', alignItems:'center', gap: 6, padding: '4px 8px', background: 'rgba(0,0,0,0.55)', borderRadius: 999, fontSize: 11, fontWeight: 700, fontFamily:'var(--mono)', color: C.danger }}><Dot color={C.danger} size={6}/>REC 00:14</div>}
            <div style={{ fontFamily:'var(--mono)', fontSize: 10, color: C.text }}>ISO 800 · 1/60 · f/2.4</div>
          </div>
          <div style={{ position:'absolute', top: 10, right: 10, fontFamily:'var(--mono)', fontSize: 10, color: C.text }}>
            BAT {Math.round(t.battery)}% · {t.fix}
          </div>
          {/* center reticle */}
          <svg style={{ position:'absolute', inset: 0 }} viewBox="0 0 320 200">
            <path d="M150,100 L170,100 M160,90 L160,110" stroke={C.accent} strokeWidth="1.5" />
            <rect x="148" y="88" width="24" height="24" fill="none" stroke={C.accent} strokeWidth="1" strokeDasharray="2 2"/>
          </svg>
          {/* bottom toolbar */}
          <div style={{ position:'absolute', bottom: 10, left: 10, right: 10, display:'flex', justifyContent:'space-between', alignItems:'center' }}>
            <div style={{ display:'flex', gap: 6 }}>
              {['front','depth','therm'].map(c => (
                <button key={c} style={{
                  padding:'4px 8px', background: 'rgba(0,0,0,0.55)', backdropFilter:'blur(10px)',
                  border: `1px solid ${c === 'front' ? C.accent + '66' : C.line}`,
                  color: c === 'front' ? C.accent : C.text2, borderRadius: 999, fontSize: 10, fontWeight: 600, cursor:'pointer',
                  textTransform: 'uppercase', letterSpacing: 0.4,
                }}>{c}</button>
              ))}
            </div>
            <button onClick={() => setRec(r => !r)} style={{
              width: 44, height: 44, borderRadius: '50%', border: '3px solid #fff',
              background: rec ? C.danger : 'transparent', cursor: 'pointer', padding: 0,
            }}/>
          </div>
        </div>

        <SectionHeader title="Stream" />
        <Card pad={12}>
          <div style={{ display:'grid', gridTemplateColumns:'repeat(3,1fr)', gap: 8 }}>
            <Stat label="FPS" value="28.9" color={C.warn} />
            <Stat label="Latency" value="84" unit="ms" />
            <Stat label="Bitrate" value="3.4" unit="Mb/s" />
          </div>
          <div style={{ marginTop: 10, fontSize: 11, color: C.text3 }}>
            <I.warn size={11} color={C.warn} style={{ verticalAlign:-2, marginRight: 4 }}/> Camera node dropping frames — check USB bandwidth.
          </div>
        </Card>
      </div>
    </SubScreen>
  );
}

// ─────────────────────────────────────────────
// ROS 2 nodes
// ─────────────────────────────────────────────

Object.assign(window, { SubScreen, ConnectScreen, RadarLoader, CameraScreen });
