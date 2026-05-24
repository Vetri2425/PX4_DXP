// PX4 DXP — API client for dev/verification flow.
// Wraps REST calls + Socket.IO. Falls back gracefully when backend is down.

const API_BASE = "http://localhost:5001";
const SOCKET_URL = "http://localhost:5001";

let _socket = null;
let _token = localStorage.getItem("rover_token") || "";
let _connected = false;

function _headers() {
  const h = { "Content-Type": "application/json" };
  if (_token) h["X-Rover-Token"] = _token;
  return h;
}

async function _post(path, body = {}) {
  const res = await fetch(`${API_BASE}${path}`, {
    method: "POST",
    headers: _headers(),
    body: JSON.stringify(body),
  });
  if (!res.ok) {
    const text = await res.text();
    throw new Error(text);
  }
  return res.json();
}

async function _get(path) {
  const res = await fetch(`${API_BASE}${path}`, { headers: _headers() });
  if (!res.ok) {
    const text = await res.text();
    throw new Error(text);
  }
  return res.json();
}

// -- Socket.IO lifecycle -----------------------------------------------
function initSocket(handlers = {}) {
  if (typeof io === "undefined") {
    console.warn("socket.io-client not loaded; skipping real-time connection");
    return null;
  }
  if (_socket) return _socket;

  _socket = io(SOCKET_URL, {
    transports: ["websocket", "polling"],
    auth: { token: _token },
    reconnection: true,
    reconnectionDelay: 1000,
    reconnectionAttempts: 10,
  });

  _socket.on("connect", () => {
    _connected = true;
    console.log("[api] socket connected");
    handlers.onConnect && handlers.onConnect();
  });

  _socket.on("disconnect", () => {
    _connected = false;
    console.log("[api] socket disconnected");
    handlers.onDisconnect && handlers.onDisconnect();
  });

  _socket.on("telemetry", (data) => {
    handlers.onTelemetry && handlers.onTelemetry(data);
  });

  _socket.on("mission_status", (data) => {
    handlers.onMissionStatus && handlers.onMissionStatus(data);
  });

  _socket.on("mission_completed", (data) => {
    handlers.onMissionCompleted && handlers.onMissionCompleted(data);
  });

  _socket.on("safety_abort", (data) => {
    handlers.onSafetyAbort && handlers.onSafetyAbort(data);
  });

  _socket.on("rover_disconnected", () => {
    handlers.onRoverDisconnected && handlers.onRoverDisconnected();
  });

  _socket.on("arm_result", (data) => {
    handlers.onArmResult && handlers.onArmResult(data);
  });

  _socket.on("mode_result", (data) => {
    handlers.onModeResult && handlers.onModeResult(data);
  });

  _socket.on("socket_error", (data) => {
    console.error("[api] socket error:", data);
    handlers.onError && handlers.onError(data);
  });

  return _socket;
}

function disconnectSocket() {
  if (_socket) {
    _socket.disconnect();
    _socket = null;
    _connected = false;
  }
}

// -- REST wrappers ----------------------------------------------------
const api = {
  // Config
  setToken(t) {
    _token = t;
    localStorage.setItem("rover_token", t);
    if (_socket) _socket.auth = { token: t };
  },
  getToken() { return _token; },

  // Connection state
  get isConnected() { return _connected; },

  // Vehicle control
  arm(val)       { return _post("/api/arm", { arm: val }); },
  disarm()       { return _post("/api/arm", { arm: false }); },
  setMode(mode)  { return _post("/api/set_mode", { mode }); },
  estop()        { return _post("/api/estop", {}); },

  // Mission control
  loadMission(name)   { return _post("/api/mission/load", { path_name: name }); },
  startMission(name)  { return _post("/api/mission/start", name ? { path_name: name } : {}); },
  stopMission()       { return _post("/api/mission/stop", {}); },
  abortMission()      { return _post("/api/mission/abort", {}); },

  // Telemetry / status
  getTelemetry()      { return _get("/api/telemetry/latest"); },
  getMissionStatus()  { return _get("/api/mission/status"); },

  // Socket.IO
  initSocket,
  disconnectSocket,
  get socket() { return _socket; },

  // Socket-based control (alternative to REST — fires and forgets, waits for result event)
  socketArm(val) {
    if (!_socket) return Promise.reject("not connected");
    _socket.emit("arm", { arm: val, auth: _token });
    return Promise.resolve();
  },
  socketSetMode(mode) {
    if (!_socket) return Promise.reject("not connected");
    _socket.emit("set_mode", { mode, auth: _token });
    return Promise.resolve();
  },
  socketMissionStart() {
    if (!_socket) return Promise.reject("not connected");
    _socket.emit("mission_start", { auth: _token });
    return Promise.resolve();
  },
  socketMissionStop() {
    if (!_socket) return Promise.reject("not connected");
    _socket.emit("mission_stop", { auth: _token });
    return Promise.resolve();
  },
  socketMissionAbort() {
    if (!_socket) return Promise.reject("not connected");
    _socket.emit("mission_abort", { auth: _token });
    return Promise.resolve();
  },
};

window.api = api;
