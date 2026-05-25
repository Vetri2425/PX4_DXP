// Sub-screens (split for size)
function RosScreen() {
  const r = useRover();
  const [view, setView] = React.useState('nodes'); // nodes | topics | graph
  return (
    <SubScreen title="ROS 2" subtitle={`Humble · domain ${r.t.rosDomain} · ${r.t.hz} Hz`}
               trailing={<IconBtn icon={<I.refresh size={18}/>} />}>
      <div style={{ padding: '0 16px 12px' }}>
        <div style={{ display:'flex', gap: 6, background: C.card, padding: 4, borderRadius: 12, border: `1px solid ${C.line}` }}>
          {['nodes','topics','graph'].map(v => (
            <button key={v} onClick={() => setView(v)} style={{
              flex:1, padding:'8px 6px', border:'none', borderRadius: 9, cursor:'pointer',
              background: view === v ? `${C.accent}22` : 'transparent',
              color: view === v ? C.accent : C.text2,
              fontSize: 12, fontWeight: 600, textTransform: 'capitalize', letterSpacing: 0.3,
            }}>{v}</button>
          ))}
        </div>
      </div>

      {view === 'nodes' && (
        <div style={{ padding: '0 16px' }}>
          <Card pad={0}>
            {r.rosNodes.map((n, i) => (
              <div key={n.name} style={{ padding: '12px 14px', borderBottom: i < r.rosNodes.length - 1 ? `1px solid ${C.line}` : 'none' }}>
                <div style={{ display:'flex', justifyContent:'space-between', alignItems:'center', gap: 8 }}>
                  <div style={{ display:'flex', alignItems:'center', gap: 8, minWidth: 0 }}>
                    <Dot color={n.status === 'ok' ? C.good : n.status === 'warn' ? C.warn : C.danger} />
                    <span style={{ fontFamily:'var(--mono)', fontSize: 12, fontWeight: 500, color: C.text }}>{n.name}</span>
                  </div>
                  <span style={{ fontSize: 11, color: C.text3, fontFamily:'var(--mono)' }}>{n.hz} Hz</span>
                </div>
                <div style={{ display:'flex', justifyContent:'space-between', marginTop: 6, fontSize: 10, color: C.text3, fontFamily:'var(--mono)' }}>
                  <span>{n.topics} topics</span>
                  <span style={{ color: n.cpu > 20 ? C.warn : C.text3 }}>CPU {n.cpu}%</span>
                </div>
                <div style={{ marginTop: 6 }}>
                  <Bar value={n.cpu} max={100} color={n.cpu > 20 ? C.warn : C.accent} height={3} />
                </div>
              </div>
            ))}
          </Card>
        </div>
      )}

      {view === 'topics' && (
        <div style={{ padding: '0 16px' }}>
          <Card pad={0}>
            {[
              { t:'/cmd_vel', type:'geometry_msgs/Twist', hz:50, pubs: 1, subs: 1, c: C.accent },
              { t:'/imu/data', type:'sensor_msgs/Imu', hz:200, pubs: 1, subs: 3, c: C.accent },
              { t:'/odom', type:'nav_msgs/Odometry', hz:100, pubs: 1, subs: 4, c: C.accent },
              { t:'/scan', type:'sensor_msgs/LaserScan', hz:40, pubs: 1, subs: 2, c: C.accent },
              { t:'/pen/state', type:'std_msgs/Bool', hz:10, pubs: 1, subs: 2, c: C.violet },
              { t:'/path/trace', type:'nav_msgs/Path', hz:5, pubs: 1, subs: 1, c: C.violet },
              { t:'/camera/image_raw', type:'sensor_msgs/Image', hz:30, pubs: 1, subs: 1, c: C.warn },
              { t:'/diagnostics', type:'diagnostic_msgs/Status', hz:1, pubs: 9, subs: 2, c: C.text2 },
            ].map((topic, i, arr) => (
              <div key={topic.t} style={{ padding: '11px 14px', borderBottom: i < arr.length - 1 ? `1px solid ${C.line}` : 'none' }}>
                <div style={{ display:'flex', justifyContent:'space-between', alignItems:'center' }}>
                  <span style={{ fontFamily:'var(--mono)', fontSize: 12, fontWeight: 500, color: topic.c }}>{topic.t}</span>
                  <span style={{ fontSize: 11, color: C.text3, fontFamily:'var(--mono)' }}>{topic.hz} Hz</span>
                </div>
                <div style={{ fontSize: 10, color: C.text3, marginTop: 2, fontFamily:'var(--mono)' }}>
                  {topic.type} · pub {topic.pubs} · sub {topic.subs}
                </div>
              </div>
            ))}
          </Card>
        </div>
      )}

      {view === 'graph' && <NodeGraph />}
    </SubScreen>
  );
}

function NodeGraph() {
  // Simple radial: hub in center + leaves
  const nodes = [
    { id: 'px4_bridge', a: 0.0, c: C.accent },
    { id: 'imu_filter', a: 0.16, c: C.accent },
    { id: 'odom_fusion', a: 0.32, c: C.accent },
    { id: 'path_planner', a: 0.48, c: C.violet },
    { id: 'svg_renderer', a: 0.64, c: C.violet },
    { id: 'camera_node', a: 0.80, c: C.warn },
    { id: 'pen_actuator', a: 0.92, c: C.accent2 },
  ];
  return (
    <div style={{ padding: '0 16px' }}>
      <Card pad={12}>
        <div style={{ position:'relative', aspectRatio: '1/1', maxWidth: 320, margin: '0 auto' }}>
          <svg viewBox="0 0 320 320" width="100%" height="100%">
            <circle cx="160" cy="160" r="120" fill="none" stroke={C.line2} strokeDasharray="2 4"/>
            <circle cx="160" cy="160" r="60" fill="none" stroke={C.line2} strokeDasharray="2 4"/>
            {/* center hub */}
            <circle cx="160" cy="160" r="26" fill="#0a0d12" stroke={C.accent} strokeWidth="2"/>
            <text x="160" y="164" textAnchor="middle" fontSize="10" fontWeight="700" fill={C.accent} fontFamily="var(--mono)">rcl</text>
            {nodes.map(n => {
              const angle = n.a * Math.PI * 2;
              const x = 160 + Math.cos(angle) * 120, y = 160 + Math.sin(angle) * 120;
              return (
                <g key={n.id}>
                  <line x1="160" y1="160" x2={x} y2={y} stroke={`${n.c}55`} strokeWidth="1.5">
                    <animate attributeName="stroke-dasharray" values="0 200; 200 0" dur="2s" repeatCount="indefinite" />
                  </line>
                  <circle cx={x} cy={y} r="14" fill="#0a0d12" stroke={n.c} strokeWidth="1.5"/>
                  <text x={x} y={y - 22} textAnchor="middle" fontSize="9" fontFamily="var(--mono)" fill={C.text2}>/{n.id}</text>
                </g>
              );
            })}
          </svg>
        </div>
        <div style={{ marginTop: 8, textAlign:'center', fontSize: 11, color: C.text3 }}>
          {nodes.length} active · {nodes.length * 3} edges · 0 cycles
        </div>
      </Card>
    </div>
  );
}

// ─────────────────────────────────────────────
// PX4 Parameters
// ─────────────────────────────────────────────
function Px4Screen() {
  const r = useRover();
  const [params, setParams] = React.useState(r.px4Params);
  const [search, setSearch] = React.useState('');
  const [dirty, setDirty] = React.useState(false);

  const filtered = params.filter(p => p.name.toLowerCase().includes(search.toLowerCase()));
  const byGroup = {};
  filtered.forEach(p => { (byGroup[p.group] = byGroup[p.group] || []).push(p); });

  const update = (name, value) => {
    setParams(ps => ps.map(p => p.name === name ? { ...p, value } : p));
    setDirty(true);
  };

  return (
    <SubScreen title="PX4 Parameters" subtitle={`${params.length} loaded · ${dirty ? 'unsaved changes' : 'in sync'}`}
               trailing={dirty
                 ? <Btn variant="primary" size="sm" onClick={() => setDirty(false)}>Save</Btn>
                 : <IconBtn icon={<I.download size={18}/>} />}>
      <div style={{ padding: '0 16px 12px' }}>
        <div style={{ display:'flex', alignItems:'center', gap: 8, padding:'10px 14px', background: C.card, borderRadius: 14, border: `1px solid ${C.line}` }}>
          <I.search size={16} color={C.text3}/>
          <input value={search} onChange={e=>setSearch(e.target.value)} placeholder="MC_ROLL_P, MPC_…"
                 style={{ flex: 1, background: 'transparent', border: 'none', outline: 'none', color: C.text, fontFamily:'var(--mono)', fontSize: 13 }} />
          {search && <button onClick={()=>setSearch('')} style={{background:'none', border:'none', color: C.text3, cursor:'pointer', padding:0}}><I.x size={14}/></button>}
        </div>
      </div>

      <div style={{ padding: '0 16px' }}>
        {Object.entries(byGroup).map(([group, list]) => (
          <div key={group} style={{ marginBottom: 14 }}>
            <div style={{ fontSize: 11, color: C.text3, textTransform: 'uppercase', letterSpacing: 0.8, fontWeight: 600, padding: '4px 4px 8px' }}>{group}</div>
            <Card pad={0}>
              {list.map((p, i) => (
                <ParamRow key={p.name} p={p} onChange={v => update(p.name, v)} last={i === list.length - 1} />
              ))}
            </Card>
          </div>
        ))}
        {filtered.length === 0 && <div style={{ padding: 24, textAlign:'center', color: C.text3 }}>No matches</div>}
      </div>
    </SubScreen>
  );
}

function ParamRow({ p, onChange, last }) {
  const [open, setOpen] = React.useState(false);
  const pct = ((p.value - p.range[0]) / (p.range[1] - p.range[0])) * 100;
  return (
    <div style={{ padding: '12px 14px', borderBottom: last ? 'none' : `1px solid ${C.line}` }}>
      <button onClick={() => setOpen(o => !o)} style={{ width: '100%', display:'flex', alignItems:'center', justifyContent:'space-between',
              background:'transparent', border:'none', padding: 0, cursor: 'pointer', color: C.text }}>
        <span style={{ fontFamily:'var(--mono)', fontSize: 13, fontWeight: 500 }}>{p.name}</span>
        <span style={{ fontFamily:'var(--mono)', fontSize: 13, color: C.accent, fontWeight: 600 }}>
          {Number(p.value).toFixed(p.range[1] > 50 ? 0 : 2)}
        </span>
      </button>
      <div style={{ marginTop: 8 }}>
        <input type="range" min={p.range[0]} max={p.range[1]} step={p.range[1] > 50 ? 1 : 0.05} value={p.value}
               onChange={e => onChange(Number(e.target.value))}
               style={{ width:'100%', appearance:'none', height: 3,
                 background: `linear-gradient(to right, ${C.accent} 0%, ${C.accent} ${pct}%, rgba(255,255,255,0.08) ${pct}%, rgba(255,255,255,0.08) 100%)`,
                 borderRadius: 2, outline:'none' }}/>
        <div style={{ display:'flex', justifyContent:'space-between', marginTop: 4, fontFamily:'var(--mono)', fontSize: 10, color: C.text3 }}>
          <span>{p.range[0]}</span><span>{p.range[1]}</span>
        </div>
      </div>
    </div>
  );
}

// ─────────────────────────────────────────────
// CALIBRATION
// ─────────────────────────────────────────────

Object.assign(window, { RosScreen, Px4Screen });
