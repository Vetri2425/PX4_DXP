// Single rover state store — telemetry, mission, drawing, etc.
// React context. When backend is reachable, Socket.IO feeds real telemetry.
// When offline, the mock tick keeps the UI alive for layout work.

const RoverCtx = React.createContext(null);
const useRover = () => React.useContext(RoverCtx);

// MOCK-DATA-COMMENTED: SEED_WAYPOINTS - mock waypoint data removed

// MOCK-DATA-COMMENTED: SEED_FLEET - mock fleet data removed

// MOCK-DATA-COMMENTED: SEED_ROS_NODES - mock ROS nodes data removed

// MOCK-DATA-COMMENTED: SEED_PX4_PARAMS - mock PX4 params data removed

// MOCK-DATA-COMMENTED: SEED_LOGS - mock logs data removed

// MOCK-DATA-COMMENTED: SEED_GALLERY - mock gallery data removed

function RoverProvider({ children }) {
  const [connected, setConnected] = React.useState(false);
  const [activeRover, setActiveRover] = React.useState('dxp-01');
  const [activeRoverUrl, setActiveRoverUrl] = React.useState(
    () => localStorage.getItem("rover_base_url") || "http://localhost:5001"
  );
  const [discoveredRovers, setDiscoveredRovers] = React.useState([]);
  const [discovering, setDiscovering] = React.useState(false);
  const [armed, setArmed] = React.useState(false);
  const [emergency, setEmergency] = React.useState(false);
  const [mode, setMode] = React.useState('Manual');
  const [tab, setTab] = React.useState('home');
  const [stack, setStack] = React.useState([]);

  const [waypoints, setWaypoints] = React.useState([]);
  const [activeJob, setActiveJob] = React.useState(null);
  const [drawProgress, setDrawProgress] = React.useState(0);

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

    // MOCK-DATA-COMMENTED: Mock telemetry tick disabled for production

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
      await window.api?.disarm();
    } catch(e) {}
  };

  const discoverRovers = async () => {
    setDiscovering(true);
    try {
      const beacons = await window.api?.discover() || [];
      setDiscoveredRovers(beacons);
    } catch(e) {
      console.error("[store] discover failed:", e);
    } finally {
      setDiscovering(false);
    }
  };

  const switchRover = async (url, roverName) => {
    if (!url) return;
    setActiveRoverUrl(url);
    localStorage.setItem("active_rover_name", roverName || "");
    try {
      window.api?.setBaseUrl(url);
      await new Promise(r => setTimeout(r, 200));
    } catch(e) {
      console.error("[store] switch failed:", e);
    }
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
    activeRoverUrl, setActiveRoverUrl, discoveredRovers, discovering, discoverRovers, switchRover,
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
    fleet: [],
    rosNodes: [],
    px4Params: [],
    logs: [],
    gallery: [],
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
