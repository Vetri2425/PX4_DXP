// More — index of remaining tools.

function MoreScreen() {
  const r = useRover();
  const items = [
    { i:<I.cam size={18}/>,      t:'Live camera',        s:'1080p · 30fps',          k:'camera',  c: C.warn },
    { i:<I.ros size={18}/>,      t:'ROS 2 nodes',        s:`${r.rosNodes.length} alive · domain 42`, k:'ros', c: C.accent },
    { i:<I.sliders size={18}/>,  t:'PX4 parameters',     s:'Tune & save',            k:'px4',     c: C.violet },
    { i:<I.calib size={18}/>,    t:'Calibration',        s:'Compass · accel · gyro', k:'calibrate', c: C.warn },
    { i:<I.logs size={18}/>,     t:'Logs & diagnostics', s:'Rosout · MAVLink · uORB',k:'logs',    c: C.text2 },
    { i:<I.firmware size={18}/>, t:'Firmware & packages',s:'apt · pip · OTA',        k:'firmware',c: C.good },
    { i:<I.fleet size={18}/>,    t:'Fleet',              s:'4 rovers',               k:'fleet',   c: C.accent2 },
    { i:<I.link size={18}/>,     t:'Connect rover',      s:'Wi-Fi · BT · serial',    k:'connect', c: C.accent },
    { i:<I.gear size={18}/>,     t:'Settings',           s:'Account · units · API',  k:'settings',c: C.text2 },
  ];
  return (
    <div style={{ padding: '0 0 100px' }}>
      <AppBar
        title="More"
        subtitle="Maintenance, diagnostics, fleet"
        trailing={<IconBtn icon={<I.search size={18}/>}/>}
      />

      <SectionHeader title="Operations" />
      <div style={{ padding: '0 16px 16px' }}>
        <Card pad={0}>
          {items.slice(0,5).map((x, i) => (
            <React.Fragment key={x.k}>
              <Row
                icon={x.i} iconBg={`${x.c}1c`} iconColor={x.c}
                title={x.t} sub={x.s}
                onClick={() => r.push(x.k)}
              />
              {i < 4 && <div style={{ height: 1, background: C.line, margin: '0 14px 0 64px' }}/>}
            </React.Fragment>
          ))}
        </Card>
      </div>

      <SectionHeader title="System" />
      <div style={{ padding: '0 16px 16px' }}>
        <Card pad={0}>
          {items.slice(5).map((x, i) => (
            <React.Fragment key={x.k}>
              <Row
                icon={x.i} iconBg={`${x.c}1c`} iconColor={x.c}
                title={x.t} sub={x.s}
                onClick={() => r.push(x.k)}
              />
              {i < items.slice(5).length - 1 && <div style={{ height: 1, background: C.line, margin: '0 14px 0 64px' }}/>}
            </React.Fragment>
          ))}
        </Card>
      </div>

      <div style={{ padding: '0 16px 16px', textAlign:'center', color: C.text3, fontSize: 11, fontFamily:'var(--mono)' }}>
        DXP 1.4.2 (build 2026.05.18) · PX4 v1.15 · ROS 2 Humble
      </div>
    </div>
  );
}

window.MoreScreen = MoreScreen;
