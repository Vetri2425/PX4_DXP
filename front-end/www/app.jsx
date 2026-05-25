// PX4 DXP — main shell. Owns iOS frame, tab bar, sub-screen stack, tweaks.

const TWEAK_DEFAULTS = /*EDITMODE-BEGIN*/{
  "typeface": "sans",
  "telemetryDensity": "heavy",
  "showTrace": true,
  "showHud": true,
  "accent": "#22d3ee"
}/*EDITMODE-END*/;

function TabBar() {
  const r = useRover();
  const tabs = [
    { id: 'home',  l: 'Home',  i: I.home },
    { id: 'map',   l: 'Map',   i: I.map },
    { id: 'draw',  l: 'Draw',  i: I.draw, fab: true },
    { id: 'drive', l: 'Drive', i: I.drive },
    { id: 'more',  l: 'More',  i: I.more },
  ];
  return (
    <div style={{
      position: 'absolute', left: 0, right: 0, bottom: 0, zIndex: 30,
      padding: '8px 14px 26px',
      background: `linear-gradient(180deg, transparent 0%, ${C.bg}f0 35%, ${C.bg} 100%)`,
      pointerEvents: 'none',
    }}>
      <div style={{
        display:'flex', gap: 4, alignItems:'center', justifyContent:'space-around',
        background: 'rgba(20,25,35,0.78)', backdropFilter: 'blur(28px) saturate(180%)',
        WebkitBackdropFilter: 'blur(28px) saturate(180%)',
        border: `1px solid ${C.line2}`, borderRadius: 22, padding: 8,
        boxShadow: '0 8px 32px rgba(0,0,0,0.4)',
        pointerEvents: 'auto',
      }}>
        {tabs.map(t => {
          const active = r.tab === t.id && r.stack.length === 0;
          if (t.fab) {
            return (
              <button key={t.id} onClick={() => r.setTab(t.id)} style={{
                width: 44, height: 44, borderRadius: 14, border: 'none', cursor: 'pointer',
                background: `linear-gradient(135deg, ${C.accent}, #0e7490)`,
                color: '#06202a',
                display:'flex', alignItems:'center', justifyContent:'center',
                boxShadow: `0 6px 16px ${C.accent}55`,
              }}>
                <t.i size={20} strokeWidth={2.2} />
              </button>
            );
          }
          const Ic = t.i;
          return (
            <button key={t.id} onClick={() => r.setTab(t.id)} style={{
              flex: 1, padding: '6px 4px', border: 'none', borderRadius: 12, cursor: 'pointer',
              background: 'transparent',
              color: active ? C.accent : C.text3,
              display:'flex', flexDirection:'column', alignItems:'center', gap: 2,
            }}>
              <Ic size={20} strokeWidth={active ? 2 : 1.6} />
              <span style={{ fontSize: 9.5, fontWeight: 600, letterSpacing: 0.3 }}>{t.l}</span>
            </button>
          );
        })}
      </div>
    </div>
  );
}

function DXFInspectorMount() {
  const r = useRover();
  if (!r.dxfInspectorOpen || !r.dxfFile) return null;
  return (
    <DXFInspector
      dxf={r.dxfFile}
      initialSelected={r.dxfSelected}
      onCancel={() => r.setDxfInspectorOpen(false)}
      onConfirm={({ selected, overrides, order }) => {
        r.setDxfSelected(selected);
        r.setDxfOverrides(overrides);
        r.setDxfOrder(order);
        r.setDxfInspectorOpen(false);
        r.setActiveJob({ id: 'dxf', name: r.dxfFile.name, progress: 0, eta: '—',
                         paths: selected.size, done: 0 });
      }}
    />
  );
}


function ConnectionBadge() {
  const r = useRover();
  const [tooltip, setTooltip] = React.useState(false);

  let color, label, dotColor;
  if (r.backendError) {
    color = C.danger;
    dotColor = C.danger;
    label = "error";
  } else if (r.backendConnected) {
    color = C.good;
    dotColor = C.good;
    label = "live";
  } else {
    color = C.text3;
    dotColor = C.text3;
    label = "offline";
  }

  return (
    <div style={{
      position: "absolute", top: 8, right: 12, zIndex: 35,
      display: "flex", alignItems: "center", gap: 6,
    }}>
      <button
        onClick={() => setTooltip(!tooltip)}
        onMouseEnter={() => setTooltip(true)}
        onMouseLeave={() => setTooltip(false)}
        style={{
          display: "inline-flex", alignItems: "center", gap: 6,
          padding: "4px 9px", borderRadius: 999,
          background: color + "1a", border: "1px solid " + color + "33",
          color: color, fontSize: 10, fontWeight: 600, letterSpacing: 0.3,
          cursor: "pointer", fontFamily: "var(--mono)",
        }}
      >
        <Dot color={dotColor} size={6} pulse={r.backendConnected && !r.backendError} />
        {label}
      </button>
      {tooltip && (
        <div style={{
          position: "absolute", top: 30, right: 0,
          padding: "8px 10px", borderRadius: 8,
          background: C.card, border: "1px solid " + C.line,
          fontSize: 11, color: C.text2, whiteSpace: "nowrap",
          zIndex: 36, boxShadow: "0 4px 14px rgba(0,0,0,0.5)",
        }}>
          {r.backendError
            ? "Error: " + r.backendError
            : r.backendConnected
              ? "Connected to " + (localStorage.getItem("rover_base_url") || "http://localhost:5001")
              : "Offline · mock data active"}
        </div>
      )}
    </div>
  );
}

function StatusOverlay() {
  // Top-of-frame bar that sits OUTSIDE the iOS status bar but always visible.
  const r = useRover();
  if (!r.emergency) return null;
  return (
    <div style={{
      position: 'absolute', top: 56, left: 16, right: 16, zIndex: 40,
      padding: '10px 14px', borderRadius: 14,
      background: `linear-gradient(135deg, ${C.danger}33, ${C.danger}15)`,
      border: `1px solid ${C.danger}66`,
      backdropFilter: 'blur(20px)',
      display:'flex', alignItems:'center', gap: 10,
    }}>
      <div style={{ width: 32, height: 32, borderRadius: 10, background: C.danger, color: '#3a0a14',
                    display:'flex', alignItems:'center', justifyContent:'center', flexShrink: 0 }}>
        <I.warn size={18} />
      </div>
      <div style={{ flex: 1, minWidth: 0 }}>
        <div style={{ fontWeight: 700, fontSize: 13, color: C.danger }}>Emergency stop active</div>
        <div style={{ fontSize: 11, color: C.text2, marginTop: 1 }}>Motors disarmed · pen lifted · holding position</div>
      </div>
      <Btn variant="secondary" size="sm" onClick={r.clearEStop}>Clear</Btn>
    </div>
  );
}

function ScreenSwitch() {
  const r = useRover();

  // Sub-screens override the tab
  if (r.stack.length > 0) {
    const top = r.stack[r.stack.length - 1];
    const map = {
      connect: ConnectScreen,
      camera: CameraScreen,
      ros: RosScreen,
      px4: Px4Screen,
      calibrate: CalibrateScreen,
      logs: LogsScreen,
      firmware: FirmwareScreen,
      fleet: FleetScreen,
      settings: SettingsScreen,
    };
    const Sub = map[top];
    if (Sub) return <Sub />;
  }

  if (r.tab === 'home')  return <HomeScreen />;
  if (r.tab === 'map')   return <MapScreen />;
  if (r.tab === 'drive') return <DriveScreen />;
  if (r.tab === 'draw')  return <DrawScreen />;
  if (r.tab === 'more')  return <MoreScreen />;
  return <HomeScreen />;
}

function App() {
  const [tweaks, setTweak] = useTweaks(TWEAK_DEFAULTS);
  // Apply telemetry density to root via a data attribute we read in style
  return (
    <RoverProvider>
      <GlobalCSS />
      <div className="px-app"
           data-typeface={tweaks.typeface}
           data-density={tweaks.telemetryDensity}
           style={{ display:'flex', justifyContent:'center', alignItems:'center', gap: 36, flexWrap:'wrap' }}>
        <IOSDevice dark={true} width={402} height={874}>
          <div style={{ position:'relative', height: '100%', overflow:'hidden', paddingTop: 54, fontFamily: 'var(--sans)' }}>
            <ConnectionBadge />
            <div style={{ position:'absolute', inset: '54px 0 0 0', overflow:'auto' }}>
              <ScreenSwitch />
            </div>
            <StatusOverlay />
            <TabBar />
            <DXFInspectorMount />
          </div>
        </IOSDevice>

        {/* Caption on the side */}
        <div style={{ maxWidth: 280, color: C.text2 }}>
          <div style={{ fontFamily: 'var(--mono)', fontSize: 10, color: C.accent, letterSpacing: 1, textTransform: 'uppercase', fontWeight: 700 }}>
            DXP · v1.4.2
          </div>
          <div style={{ fontSize: 28, fontWeight: 700, color: C.text, marginTop: 4, lineHeight: 1.15, letterSpacing: -0.5 }}>
            PX4 Drawing Rover<br/>Mobile Workbench
          </div>
          <div style={{ fontSize: 13, color: C.text2, marginTop: 10, lineHeight: 1.5 }}>
            Hi-fi prototype. Tap through Home, Map, Draw, Drive, More. The map supports long-press to add waypoints and drag to move them.
          </div>
          <div style={{ marginTop: 14, display: 'flex', gap: 8, flexWrap: 'wrap' }}>
            {['ROS2 Humble','PX4 v1.15','MAVLink','rosbridge','RTK GPS'].map(t => (
              <span key={t} style={{
                fontSize: 11, fontFamily:'var(--mono)', color: C.text3, padding: '4px 8px',
                background: 'rgba(255,255,255,0.04)', border: `1px solid ${C.line}`, borderRadius: 999,
              }}>{t}</span>
            ))}
          </div>
          <div style={{ marginTop: 18, fontSize: 11, color: C.text3 }}>
            Toggle <b style={{ color: C.text2 }}>Tweaks</b> in the toolbar to switch typography or telemetry density.
          </div>
        </div>
      </div>

      <TweaksPanel title="Tweaks">
        <TweakSection label="Typography" />
        <TweakRadio label="Typeface" value={tweaks.typeface}
                    options={[{value:'sans', label:'Sans'}, {value:'mono', label:'Mono'}]}
                    onChange={(v) => setTweak('typeface', v)} />
        <TweakSection label="Telemetry" />
        <TweakRadio label="Density" value={tweaks.telemetryDensity}
                    options={[
                      {value:'light', label:'Light'},
                      {value:'medium', label:'Medium'},
                      {value:'heavy', label:'Heavy'},
                    ]}
                    onChange={(v) => setTweak('telemetryDensity', v)} />
        <TweakToggle label="Show path trace" value={tweaks.showTrace}
                     onChange={(v) => setTweak('showTrace', v)} />
        <TweakToggle label="Show HUD overlays" value={tweaks.showHud}
                     onChange={(v) => setTweak('showHud', v)} />
      </TweaksPanel>
    </RoverProvider>
  );
}

ReactDOM.createRoot(document.getElementById('root')).render(<App />);
