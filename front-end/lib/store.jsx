// Single rover state store — telemetry, mission, drawing, etc.
// React context. When backend is reachable, Socket.IO feeds real telemetry.
// When offline, the mock tick keeps the UI alive for layout work.

const RoverCtx = React.createContext(null);
const useRover = () => React.useContext(RoverCtx);

const SEED_WAYPOINTS = [
  { id: 'wp1', x: 110, y: 130, type: 'start' },
  { id: 'wp2', x: 220, y: 90,  type: 'pen-down' },
  { id: 'wp3', x: 280, y: 170, type: 'turn' },
  { id: 'wp4', x: 200, y: 240, type: 'pen-up' },
  { id: 'wp5', x: 90,  y: 220, type: 'end' },
];

const SEED_FLEET = [
  { id: 'dxp-01', name: 'DXP-01 Mercutio', status: 'connected', battery: 78, role: 'lead', loc: 'Studio A' },
  { id: 'dxp-02', name: 'DXP-02 Ophelia', status: 'standby', battery: 92, role: 'follower', loc: 'Studio A' },
  { id: 'dxp-03', name: 'DXP-03 Puck',     status: 'charging', battery: 24, role: 'idle',     loc: 'Dock-2' },
  { id: 'dxp-04', name: 'DXP-04 Hamlet',   status: 'offline',  battery: 0,  role: '—',        loc: 'Lab bench' },
];

const SEED_ROS_NODES = [
  { name: '/px4_bridge', hz: 250, status: 'ok', cpu: 12, topics: 18 },
  { name: '/path_planner', hz: 50, status: 'ok', cpu: 8, topics: 6 },
  { name: '/svg_renderer', hz: 30, status: 'ok', cpu: 4, topics: 3 },
  { name: '/imu_filter', hz: 200, status: 'ok', cpu: 6, topics: 4 },
  { name: '/odom_fusion', hz: 100, status: 'ok', cpu: 11, topics: 7 },
  { name: '/camera_node', hz: 30, status: 'warn', cpu: 22, topics: 5 },
  { name: '/joy_teleop', hz: 50, status: 'ok', cpu: 1, topics: 2 },
  { name: '/safety_monitor', hz: 20, status: 'ok', cpu: 2, topics: 9 },
  { name: '/pen_actuator', hz: 10, status: 'ok', cpu: 0.5, topics: 2 },
  { name: '/rosbridge_websocket', hz: 60, status: 'ok', cpu: 5, topics: 24 },
];

const SEED_PX4_PARAMS = [
  { name: 'MC_ROLL_P', value: 6.5, range: [0, 12], group: 'Multicopter Attitude' },
  { name: 'MC_PITCH_P', value: 6.5, range: [0, 12], group: 'Multicopter Attitude' },
  { name: 'MPC_XY_VEL_MAX', value: 12, range: [0, 20], group: 'Position Control' },
  { name: 'MPC_Z_VEL_MAX_UP', value: 3.0, range: [0, 8], group: 'Position Control' },
  { name: 'BAT1_N_CELLS', value: 4, range: [1, 14], group: 'Battery' },
  { name: 'COM_DISARM_LAND', value: 2.0, range: [0, 20], group: 'Commander' },
  { name: 'NAV_ACC_RAD', value: 0.5, range: [0.05, 10], group: 'Mission' },
  { name: 'EKF2_GPS_CHECK', value: 245, range: [0, 511], group: 'EKF2 Estimator' },
];

const SEED_LOGS = [
  { t: '09:42:11.204', lvl: 'info',  src: 'px4_bridge', msg: 'Heartbeat OK · 245Hz uORB' },
  { t: '09:42:11.156', lvl: 'info',  src: 'svg_renderer', msg: 'Parsed mountain.svg → 142 paths · 18.4m total' },
  { t: '09:42:10.911', lvl: 'warn',  src: 'camera_node', msg: 'Frame drop · 28/30 fps' },
  { t: '09:42:10.422', lvl: 'info',  src: 'path_planner', msg: 'Replanning around obstacle at (2.4, 1.8)' },
  { t: '09:42:09.880', lvl: 'info',  src: 'imu_filter', msg: 'Bias estimate converged · σ=0.003' },
  { t: '09:42:09.501', lvl: 'error', src: 'safety_monitor', msg: 'Geofence breach predicted in 3.2s — yielding' },
  { t: '09:42:09.118', lvl: 'info',  src: 'odom_fusion', msg: 'GPS lock acquired · 12 sats · HDOP 0.7' },
  { t: '09:42:08.660', lvl: 'info',  src: 'pen_actuator', msg: 'Pen down · servo PWM 1450' },
  { t: '09:42:08.302', lvl: 'info',  src: 'ros2_humble', msg: 'Discovery: 18 nodes alive on domain 42' },
  { t: '09:42:07.999', lvl: 'debug', src: 'mavlink', msg: 'STATUSTEXT id=245 sev=INFO "Position OK"' },
];

const SEED_GALLERY = [
  { id: 'm1', name: 'Mountain ridge', paths: 142, length: 18.4, hash: 'a3f1' },
  { id: 'm2', name: 'Hamlet portrait', paths: 308, length: 42.1, hash: 'c780' },
  { id: 'm3', name: 'Studio logo',   paths: 23,  length: 4.2,  hash: '11e9' },
  { id: 'm4', name: 'Map of Verona', paths: 521, length: 87.3, hash: '4f2c' },
  { id: 'm5', name: 'Spirograph A',  paths: 88,  length: 12.8, hash: 'b920' },
  { id: 'm6', name: 'Calligraphy 草', paths: 41,  length: 6.0,  hash: '7e44' },
];

function RoverProvider({ children }) {
  const [connected, setConnected] = React.useState(true);
  const [activeRover, setActiveRover] = React.useState('dxp-01');
  const [armed, setArmed] = React.useState(false);
  const [emergency, setEmergency] = React.useState(false);
  const [mode, setMode] = React.useState('Manual');
  const [tab, setTab] = React.useState('home');
  const [stack, setStack] = React.useState([]);

  const [waypoints, setWaypoints] = React.useState(SEED_WAYPOINTS);
  const [activeJob, setActiveJob] = React.useState({
    id: 'm1', name: 'Mountain ridge', progress: 0.42, eta: '4:18', paths: 142, done: 60,
  });
  const [drawProgress, setDrawProgress] = React.useState(0.42);

  const [dxfFile, setDxfFile] = React.useState(null);
  const [dxfSelected, setDxfSelected] = React.useState(null);
  const [dxfOverrides, setDxfOverrides] = React.useState({});
  const [dxfOrder, setDxfOrder] = React.useState([]);
  const [dxfInspectorOpen, setDxfInspectorOpen] = React.useState(false);

  // Backend connection state
  const [backendConnected, setBackendConnected] = React.useState(false);
  const [backendError, setBackendError] = React.useState(null);

  // Live telemetry
  const [t, setT] = React.useState({
    battery: 78, voltage: 16.4, current: 4.2, temp: 38,
    sats: 14, hdop: 0.7, fix: '3D RTK',
    rssi: -54, link: 'Wi-Fi 5GHz',
    speed: 0.42, heading: 124, alt: 0.34,
    roll: 0, pitch: 0, yaw: 124,
    motor: [82, 84, 79, 81],
    rosDomain: 42, nodesAlive: 18, hz: 245,
    history: {
      v: Array.from({length: 40}, (_,i) => 16 + Math.sin(i/4)*0.3 + Math.random()*0.1),
      a: Array.from({length: 40}, (_,i) => 4 + Math.sin(i/3)*0.6 + Math.random()*0.2),
      cpu: Array.from({length: 40}, (_,i) => 30 + Math.sin(i/5)*8 + Math.random()*4),
    },
  });

  // -- Backend Socket.IO integration -----------------------------------
  React.useEffect(() => {
    if (typeof window.api === 'undefined') {
      console.warn('[store] api.jsx not loaded; running mock-only mode');
      return;
    }

    const socket = window.api.initSocket({
      onConnect() {
        setBackendConnected(true);
        setBackendError(null);
        console.log('[store] backend connected');
      },
      onDisconnect() {
        setBackendConnected(false);
        console.log('[store] backend disconnected');
      },
      onTelemetry(data) {
        setT(prev => ({
          ...prev,
          battery: data.battery_pct ?? prev.battery,
          voltage: data.battery_v ?? prev.voltage,
          current: prev.current,
          temp: prev.temp,
          sats: data.gps_sat ?? prev.sats,
          hdop: prev.hdop,
          fix: data.gps_fix != null ? (['NO_FIX','2D','3D','3D_DGPS','RTK_FLOAT','RTK_FIXED'][data.gps_fix] ?? prev.fix) : prev.fix,
          rssi: prev.rssi,
          link: prev.link,
          speed: data.speed_m_s ?? prev.speed,
          heading: data.heading_ned_deg ?? prev.heading,
          alt: data.alt ?? prev.alt,
          roll: prev.roll,
          pitch: prev.pitch,
          yaw: data.heading_ned_deg ?? prev.yaw,
          motor: prev.motor,
          history: {
            v: [...prev.history.v.slice(1), data.battery_v ?? prev.history.v[39]],
            a: [...prev.history.a.slice(1), data.current ?? prev.history.a[39]],
            cpu: [...prev.history.cpu.slice(1), 30 + Math.random()*4],
          },
        }));
        if (data.connected != null) setConnected(data.connected);
        if (data.armed != null) setArmed(data.armed);
        if (data.mode) setMode(data.mode);
      },
      onMissionStatus(data) {
        // Update mission progress from backend
        if (data.dist_to_goal != null) {
          setActiveJob(prev => ({
            ...prev,
            progress: Math.max(0, Math.min(1, 1 - (data.dist_to_goal / 20))),
          }));
        }
      },
      onMissionCompleted(data) {
        setDrawProgress(1);
        setMode('Hold');
        setArmed(false);
        alert(`Mission completed: ${data.name}`);
      },
      onSafetyAbort(data) {
        setEmergency(true);
        setMode('Hold');
        setArmed(false);
        setBackendError(`SAFETY ABORT: ${data.reason}`);
      },
      onArmResult(data) {
        if (data.success) setArmed(data.arm);
      },
      onModeResult(data) {
        if (data.success) setMode(data.mode);
      },
      onError(data) {
        console.error('[store] socket error:', data);
        setBackendError(data.reason || 'Socket error');
      },
    });

    return () => {
      window.api.disconnectSocket();
    };
  }, []);

  // -- Mock telemetry tick (runs only when backend is offline) -----------
  React.useEffect(() => {
    if (backendConnected) return;
    const id = setInterval(() => {
      setT(prev => {
        const j = (v, mag, min, max) => Math.max(min, Math.min(max, v + (Math.random()-0.5)*mag));
        const driving = mode === 'Manual' || mode === 'Mission' || mode === 'Draw';
        const speedTarget = driving && !emergency ? 0.42 : 0;
        return {
          ...prev,
          battery: Math.max(2, prev.battery - (driving ? 0.005 : 0.001)),
          voltage: j(prev.voltage, 0.05, 14, 16.8),
          current: j(prev.current, 0.2, 0, 8),
          temp: j(prev.temp, 0.2, 35, 55),
          sats: Math.max(10, Math.min(18, prev.sats + (Math.random()<0.05 ? (Math.random()<0.5?-1:1) : 0))),
          hdop: j(prev.hdop, 0.05, 0.5, 1.4),
          rssi: j(prev.rssi, 1, -78, -42),
          speed: j(prev.speed, 0.04, 0, 1.2) * (driving ? 1 : 0.3) + speedTarget*0.2,
          heading: (prev.heading + (driving ? (Math.random()-0.5)*1.2 : 0) + 360) % 360,
          roll: j(prev.roll, 0.4, -8, 8),
          pitch: j(prev.pitch, 0.4, -6, 6),
          yaw: (prev.heading + 360) % 360,
          motor: prev.motor.map(m => j(m, 1.5, 60, 95)),
          history: {
            v: [...prev.history.v.slice(1), j(prev.history.v[39], 0.1, 14, 16.8)],
            a: [...prev.history.a.slice(1), j(prev.history.a[39], 0.3, 0, 8)],
            cpu: [...prev.history.cpu.slice(1), j(prev.history.cpu[39], 4, 5, 80)],
          },
          drawProgress: prev.drawProgress,
        };
      });
      if (mode === 'Draw' && !emergency) {
        setDrawProgress(p => Math.min(1, p + 0.0015));
      }
    }, 300);
    return () => clearInterval(id);
  }, [mode, emergency, backendConnected]);

  // Navigation
  const push = (screen) => setStack(s => [...s, screen]);
  const pop = () => setStack(s => s.slice(0, -1));
  const switchTab = (id) => { setTab(id); setStack([]); };

  // -- API-wrapped controls (fire-and-forget; local state updates immediately) --
  const togglePlay = async () => {
    if (emergency) return;
    if (mode === 'Draw') {
      setMode('Hold');
      try { await window.api?.stopMission(); } catch(e) {}
    } else {
      setMode('Draw');
      try { await window.api?.startMission(); } catch(e) {}
    }
  };

  const triggerEStop = async () => {
    setEmergency(true);
    setMode('Hold');
    setArmed(false);
    try { await window.api?.estop(); } catch(e) {}
  };

  const clearEStop = async () => {
    setEmergency(false);
    try {
      // After estop, we stay disarmed. Operator must re-arm manually.
      await window.api?.disarm();
    } catch(e) {}
  };

  const apiSetArmed = async (val) => {
    setArmed(val);
    try { await window.api?.arm(val); } catch(e) {}
  };

  const apiSetMode = async (m) => {
    setMode(m);
    try { await window.api?.setMode(m); } catch(e) {}
  };

  const apiMissionLoad = async (name) => {
    try { await window.api?.loadMission(name); } catch(e) {}
  };

  const apiMissionStart = async (name) => {
    setMode('Mission');
    try { await window.api?.startMission(name); } catch(e) {}
  };

  const apiMissionStop = async () => {
    setMode('Hold');
    try { await window.api?.stopMission(); } catch(e) {}
  };

  const apiMissionAbort = async () => {
    setMode('Hold');
    setArmed(false);
    try { await window.api?.abortMission(); } catch(e) {}
  };

  const value = {
    connected, setConnected, activeRover, setActiveRover,
    armed, setArmed, emergency, triggerEStop, clearEStop,
    mode, setMode,
    tab, setTab: switchTab,
    stack, push, pop,
    waypoints, setWaypoints,
    activeJob, setActiveJob, drawProgress, setDrawProgress, togglePlay,
    dxfFile, setDxfFile, dxfSelected, setDxfSelected,
    dxfOverrides, setDxfOverrides, dxfOrder, setDxfOrder,
    dxfInspectorOpen, setDxfInspectorOpen,
    t,
    fleet: SEED_FLEET,
    rosNodes: SEED_ROS_NODES,
    px4Params: SEED_PX4_PARAMS,
    logs: SEED_LOGS,
    gallery: SEED_GALLERY,
    // Backend integration
    backendConnected,
    backendError,
    apiSetArmed,
    apiSetMode,
    apiMissionLoad,
    apiMissionStart,
    apiMissionStop,
    apiMissionAbort,
  };

  return <RoverCtx.Provider value={value}>{children}</RoverCtx.Provider>;
}

window.RoverProvider = RoverProvider;
window.useRover = useRover;
