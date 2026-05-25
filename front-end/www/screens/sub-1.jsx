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
﻿function ConnectScreen() {
  const r = useRover();
  const [scanning, setScanning] = React.useState(false);
  const [step, setStep] = React.useState("scan");
  const [selectedRover, setSelectedRover] = React.useState(null);
  const [manualUrl, setManualUrl] = React.useState("");
  const [showManual, setShowManual] = React.useState(false);
  const [connectError, setConnectError] = React.useState(null);
  const [connecting, setConnecting] = React.useState(false);

  React.useEffect(() => {
    if (step === "scan" && !scanning) {
      handleDiscover();
    }
  }, [step]);

  const handleDiscover = async () => {
    setScanning(true);
    await r.discoverRovers();
    setScanning(false);
  };

  const handleConnect = async (rover) => {
    setSelectedRover(rover);
    setConnecting(true);
    setConnectError(null);
    try {
      const url = `http://${rover.host}:${rover.port}`;
      await r.switchRover(url, rover.name);
      await new Promise(res => setTimeout(res, 1500));
      setStep("done");
    } catch(e) {
      setConnectError(e.message || "Connection failed");
    } finally {
      setConnecting(false);
    }
  };

  const handleManualConnect = async () => {
    if (!manualUrl.trim()) return;
    setConnecting(true);
    setConnectError(null);
    try {
      let url = manualUrl.trim();
      if (!url.startsWith("http://") && !url.startsWith("https://")) url = "http://" + url;
      if (url.endsWith("/")) url = url.slice(0, -1);
      await r.switchRover(url, "Manual entry");
      await new Promise(res => setTimeout(res, 1500));
      setStep("done");
    } catch(e) {
      setConnectError(e.message || "Connection failed");
    } finally {
      setConnecting(false);
    }
  };

  if (connecting) {
    return (
      <SubScreen title="Connecting..." subtitle={selectedRover ? selectedRover.name : manualUrl}>
        <div style={{ padding: "40px 16px", display: "flex", flexDirection: "column", alignItems: "center", gap: 18 }}>
          <RadarLoader />
          <div style={{ textAlign: "center" }}>
            <div style={{ fontSize: 14, color: C.text2 }}>
              {selectedRover
                ? `Connecting to ${selectedRover.name} (${selectedRover.host}:${selectedRover.port})`
                : `Connecting to ${manualUrl}`}
            </div>
            <div style={{ fontFamily: "var(--mono)", fontSize: 11, color: C.text3, marginTop: 6 }}>
              Verifying Socket.IO handshake... ROS 2 DDS discovery... MAVLink heartbeat...
            </div>
          </div>
          {connectError && (
            <div style={{
              padding: "10px 14px", borderRadius: 10, background: C.danger + "1a",
              border: "1px solid " + C.danger + "33", color: C.danger,
              fontSize: 12, textAlign: "center", maxWidth: 280,
            }}>
              {connectError}
            </div>
          )}
          <Btn variant="secondary" onClick={() => { setConnecting(false); setStep("scan"); }}>
            Cancel
          </Btn>
        </div>
      </SubScreen>
    );
  }

  if (step === "done") {
    const roverName = selectedRover ? selectedRover.name : (localStorage.getItem("active_rover_name") || "rover");
    return (
      <SubScreen title="Connected" subtitle={roverName + " \u00b7 ready"}>
        <div style={{ padding: "40px 16px", display: "flex", flexDirection: "column", alignItems: "center", gap: 18 }}>
          <div style={{
            width: 96, height: 96, borderRadius: "50%", background: C.good + "26",
            display: "flex", alignItems: "center", justifyContent: "center", color: C.good,
          }}>
            <I.check size={44} />
          </div>
          <div style={{ textAlign: "center" }}>
            <div style={{ fontSize: 18, fontWeight: 700 }}>Connected</div>
            <div style={{ fontSize: 12, color: C.text3, marginTop: 4 }}>
              Base URL: {localStorage.getItem("rover_base_url") || "http://localhost:5001"}
            </div>
          </div>
          <div style={{ width: "100%", maxWidth: 280 }}>
            <Btn variant="primary" full onClick={() => { r.setConnected(true); r.pop(); }}>
              Open dashboard
            </Btn>
          </div>
        </div>
      </SubScreen>
    );
  }

  const rovers = r.discoveredRovers || [];

  return (
    <SubScreen
      title="Connect a rover"
      subtitle={scanning ? "Listening for UDP beacons..." : (rovers.length > 0 ? `${rovers.length} rover${rovers.length !== 1 ? "s" : ""} found` : "No rovers discovered")}
      trailing={<IconBtn icon={<I.refresh size={18} />} onClick={handleDiscover} />}
    >
      <div style={{ padding: "0 16px" }}>
        {rovers.length > 0 && (
          <Card pad={0}>
            {rovers.map((d, i) => {
              return (
                <React.Fragment key={d.id}>
                  <Row
                    icon={<I.wifi size={18} />}
                    iconBg={C.accent + "1c"} iconColor={C.accent}
                    title={d.name}
                    sub={`${d.host}:${d.port} \u00b7 ${d.type} \u00b7 v${d.version}`}
                    detail={"LAN"}
                    chevron={false}
                    onClick={() => handleConnect(d)}
                  />
                  {i < rovers.length - 1 && <div style={{ height: 1, background: C.line, margin: "0 14px 0 64px" }} />}
                </React.Fragment>
              );
            })}
          </Card>
        )}

        {scanning && (
          <div style={{
            marginTop: 12, padding: "10px 14px", borderRadius: 12,
            background: C.accent + "0d", border: "1px solid " + C.accent + "22",
            display: "flex", alignItems: "center", gap: 10,
          }}>
            <div style={{
              width: 16, height: 16, borderRadius: "50%",
              border: "2px solid " + C.accent,
              borderTopColor: "transparent",
              animation: "pxspin 0.8s linear infinite",
            }} />
            <span style={{ fontSize: 12, color: C.accent, fontWeight: 500 }}>
              Listening on UDP port 5002...
            </span>
          </div>
        )}

        {!scanning && rovers.length === 0 && (
          <div style={{ padding: "24px 16px", textAlign: "center" }}>
            <div style={{ marginBottom: 8 }}>
              <I.wifi size={40} color={C.text3} />
            </div>
            <div style={{ fontSize: 14, color: C.text2, fontWeight: 500 }}>No rovers found</div>
            <div style={{ fontSize: 12, color: C.text3, marginTop: 4 }}>
              Make sure the rover server is running and broadcasting on UDP 5002.
            </div>
            <div style={{ marginTop: 14 }}>
              <Btn variant="secondary" size="sm" icon={<I.refresh size={14} />} onClick={handleDiscover}>
                Scan again
              </Btn>
            </div>
          </div>
        )}

        <SectionHeader title="Manual entry" />
        <Card pad={0}>
          <div
            onClick={() => setShowManual(!showManual)}
            style={{ cursor: "pointer" }}
          >
            <Row
              icon={<I.disk size={18} />}
              iconBg={C.text3 + "1c"} iconColor={C.text3}
              title="Enter IP address manually"
              sub={showManual ? "Tap to collapse" : "For rovers outside auto-discovery range"}
            />
          </div>
          {showManual && (
            <div style={{ padding: "0 14px 14px" }}>
              <div style={{ display: "flex", gap: 8 }}>
                <input
                  type="text"
                  placeholder="192.168.1.102:5001"
                  value={manualUrl}
                  onChange={e => setManualUrl(e.target.value)}
                  onKeyDown={e => { if (e.key === "Enter") handleManualConnect(); }}
                  style={{
                    flex: 1, padding: "10px 12px", borderRadius: 10,
                    background: C.card2, border: "1px solid " + C.line2,
                    color: C.text, fontSize: 13, fontFamily: "var(--mono)",
                    outline: "none",
                  }}
                />
                <Btn variant="primary" size="sm" onClick={handleManualConnect} disabled={!manualUrl.trim()}>
                  Connect
                </Btn>
              </div>
              <div style={{ fontSize: 11, color: C.text3, marginTop: 6 }}>
                Format: host:port (default port is 5001)
              </div>
            </div>
          )}
        </Card>

        <SectionHeader title="Other methods" />
        <Card pad={0}>
          <Row icon={<I.bt size={18} />} iconBg={C.text3 + "1c"} iconColor={C.text3}
               title="Bluetooth (serial bridge)" sub="Requires BT pairing on the rover" />
          <div style={{ height: 1, background: C.line, margin: "0 14px 0 64px" }} />
          <Row icon={<I.cam size={18} />} iconBg={C.text3 + "1c"} iconColor={C.text3}
               title="USB tethered" sub="Direct connection via USB-C" />
          <div style={{ height: 1, background: C.line, margin: "0 14px 0 64px" }} />
          <Row icon={<I.fleet size={18} />} iconBg={C.text3 + "1c"} iconColor={C.text3}
               title="Simulation (SITL)" sub="PX4 SITL on localhost" />
        </Card>

        <div style={{
          marginTop: 14, padding: "8px 12px", borderRadius: 10,
          background: C.card2, fontSize: 11, fontFamily: "var(--mono)", color: C.text3,
        }}>
          Current base: {r.activeRoverUrl}
        </div>
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
