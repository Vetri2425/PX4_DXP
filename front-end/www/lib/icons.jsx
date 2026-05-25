// SVG icons. Stroke-based, 24px viewBox, lucide-style.
// All icons take {size, color, strokeWidth, style} props.

const Ico = ({ size = 22, color = 'currentColor', strokeWidth = 1.75, style, children }) => (
  <svg width={size} height={size} viewBox="0 0 24 24" fill="none"
       stroke={color} strokeWidth={strokeWidth} strokeLinecap="round" strokeLinejoin="round"
       style={style}>{children}</svg>
);

const I = {};

I.home = (p) => <Ico {...p}><path d="M3 11l9-7 9 7v9a2 2 0 0 1-2 2h-4v-7h-6v7H5a2 2 0 0 1-2-2z"/></Ico>;
I.map = (p) => <Ico {...p}><path d="M9 4 3 7v13l6-3 6 3 6-3V4l-6 3-6-3z"/><path d="M9 4v13"/><path d="M15 7v13"/></Ico>;
I.drive = (p) => <Ico {...p}><circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="3"/><path d="M12 3v3M12 18v3M3 12h3M18 12h3"/></Ico>;
I.draw = (p) => <Ico {...p}><path d="M3 21l3-1 11-11-2-2L4 18l-1 3z"/><path d="M14 6l4 4"/><path d="M17 3l4 4-2 2-4-4z"/></Ico>;
I.more = (p) => <Ico {...p}><circle cx="5" cy="12" r="1.5"/><circle cx="12" cy="12" r="1.5"/><circle cx="19" cy="12" r="1.5"/></Ico>;
I.menu = (p) => <Ico {...p}><path d="M3 6h18M3 12h18M3 18h18"/></Ico>;
I.bell = (p) => <Ico {...p}><path d="M6 8a6 6 0 1 1 12 0c0 5 2 6 2 6H4s2-1 2-6z"/><path d="M10 19a2 2 0 0 0 4 0"/></Ico>;
I.search = (p) => <Ico {...p}><circle cx="11" cy="11" r="7"/><path d="m20 20-3.5-3.5"/></Ico>;
I.plus = (p) => <Ico {...p}><path d="M12 5v14M5 12h14"/></Ico>;
I.x = (p) => <Ico {...p}><path d="M6 6l12 12M18 6 6 18"/></Ico>;
I.check = (p) => <Ico {...p}><path d="m5 12 5 5L20 7"/></Ico>;
I.chevR = (p) => <Ico {...p}><path d="m9 6 6 6-6 6"/></Ico>;
I.chevL = (p) => <Ico {...p}><path d="m15 6-6 6 6 6"/></Ico>;
I.chevD = (p) => <Ico {...p}><path d="m6 9 6 6 6-6"/></Ico>;
I.chevU = (p) => <Ico {...p}><path d="m6 15 6-6 6 6"/></Ico>;
I.battery = (p) => <Ico {...p}><rect x="2" y="7" width="17" height="10" rx="2"/><path d="M22 11v2"/><path d="M5 10v4M8 10v4M11 10v4"/></Ico>;
I.signal = (p) => <Ico {...p}><path d="M2 20h.01M6 20v-4M10 20v-8M14 20v-12M18 20V4"/></Ico>;
I.gps = (p) => <Ico {...p}><circle cx="12" cy="10" r="3"/><path d="M12 2a8 8 0 0 0-8 8c0 6 8 12 8 12s8-6 8-12a8 8 0 0 0-8-8z"/></Ico>;
I.compass = (p) => <Ico {...p}><circle cx="12" cy="12" r="9"/><path d="m14 9-1.5 5L8 15l1.5-5z"/></Ico>;
I.cam = (p) => <Ico {...p}><path d="M3 7a2 2 0 0 1 2-2h2l2-2h6l2 2h2a2 2 0 0 1 2 2v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/><circle cx="12" cy="13" r="4"/></Ico>;
I.wifi = (p) => <Ico {...p}><path d="M5 12a10 10 0 0 1 14 0M8.5 15.5a5 5 0 0 1 7 0"/><circle cx="12" cy="19" r="1"/></Ico>;
I.bt = (p) => <Ico {...p}><path d="M7 7l10 10-5 5V2l5 5L7 17"/></Ico>;
I.play = (p) => <Ico {...p}><path d="M6 4l14 8-14 8z" fill="currentColor" stroke="none"/></Ico>;
I.pause = (p) => <Ico {...p}><rect x="6" y="4" width="4" height="16" fill="currentColor" stroke="none"/><rect x="14" y="4" width="4" height="16" fill="currentColor" stroke="none"/></Ico>;
I.stop = (p) => <Ico {...p}><rect x="5" y="5" width="14" height="14" rx="2" fill="currentColor" stroke="none"/></Ico>;
I.skull = (p) => <Ico {...p}><path d="M5 11a7 7 0 1 1 14 0v4a2 2 0 0 1-2 2v3h-3v-2h-4v2H7v-3a2 2 0 0 1-2-2z"/><circle cx="9" cy="12" r="1.5" fill="currentColor"/><circle cx="15" cy="12" r="1.5" fill="currentColor"/></Ico>;
I.warn = (p) => <Ico {...p}><path d="M12 3 2 21h20z"/><path d="M12 10v5M12 18v.01"/></Ico>;
I.info = (p) => <Ico {...p}><circle cx="12" cy="12" r="9"/><path d="M12 8v.01M11 12h1v5"/></Ico>;
I.gear = (p) => <Ico {...p}><circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.7 1.7 0 0 0 .3 1.8l.1.1a2 2 0 1 1-2.8 2.8l-.1-.1a1.7 1.7 0 0 0-1.8-.3 1.7 1.7 0 0 0-1 1.5V21a2 2 0 0 1-4 0v-.1a1.7 1.7 0 0 0-1.1-1.5 1.7 1.7 0 0 0-1.8.3l-.1.1a2 2 0 1 1-2.8-2.8l.1-.1a1.7 1.7 0 0 0 .3-1.8 1.7 1.7 0 0 0-1.5-1H3a2 2 0 0 1 0-4h.1A1.7 1.7 0 0 0 4.6 9a1.7 1.7 0 0 0-.3-1.8l-.1-.1a2 2 0 1 1 2.8-2.8l.1.1a1.7 1.7 0 0 0 1.8.3H9a1.7 1.7 0 0 0 1-1.5V3a2 2 0 0 1 4 0v.1a1.7 1.7 0 0 0 1 1.5 1.7 1.7 0 0 0 1.8-.3l.1-.1a2 2 0 1 1 2.8 2.8l-.1.1a1.7 1.7 0 0 0-.3 1.8V9a1.7 1.7 0 0 0 1.5 1H21a2 2 0 0 1 0 4h-.1a1.7 1.7 0 0 0-1.5 1z"/></Ico>;
I.ros = (p) => <Ico {...p}><circle cx="12" cy="12" r="3"/><circle cx="4" cy="6" r="2"/><circle cx="20" cy="6" r="2"/><circle cx="4" cy="18" r="2"/><circle cx="20" cy="18" r="2"/><path d="M6 7l4 3M18 7l-4 3M6 17l4-3M18 17l-4-3"/></Ico>;
I.cpu = (p) => <Ico {...p}><rect x="4" y="4" width="16" height="16" rx="2"/><rect x="9" y="9" width="6" height="6"/><path d="M9 2v2M15 2v2M9 20v2M15 20v2M2 9h2M2 15h2M20 9h2M20 15h2"/></Ico>;
I.file = (p) => <Ico {...p}><path d="M14 3H6a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V9z"/><path d="M14 3v6h6"/></Ico>;
I.upload = (p) => <Ico {...p}><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><path d="M17 8l-5-5-5 5"/><path d="M12 3v12"/></Ico>;
I.download = (p) => <Ico {...p}><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><path d="M7 10l5 5 5-5"/><path d="M12 15V3"/></Ico>;
I.refresh = (p) => <Ico {...p}><path d="M21 12a9 9 0 0 1-15 6.7L3 16"/><path d="M3 12a9 9 0 0 1 15-6.7L21 8"/><path d="M21 3v5h-5M3 21v-5h5"/></Ico>;
I.power = (p) => <Ico {...p}><path d="M12 3v9"/><path d="M18.4 7a8 8 0 1 1-12.8 0"/></Ico>;
I.target = (p) => <Ico {...p}><circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="5"/><circle cx="12" cy="12" r="1" fill="currentColor"/></Ico>;
I.flag = (p) => <Ico {...p}><path d="M4 21V4h11l-1 4h7v9h-8l-1-4H4"/></Ico>;
I.path = (p) => <Ico {...p}><circle cx="5" cy="5" r="2"/><circle cx="19" cy="19" r="2"/><path d="M7 5h6a4 4 0 0 1 0 8h-2a4 4 0 0 0 0 8h6"/></Ico>;
I.calib = (p) => <Ico {...p}><path d="M3 12a9 9 0 1 0 9-9"/><path d="M12 7v5l3 2"/></Ico>;
I.logs = (p) => <Ico {...p}><path d="M4 4h12l4 4v12H4z"/><path d="M8 10h8M8 14h8M8 18h5"/></Ico>;
I.fleet = (p) => <Ico {...p}><rect x="2" y="8" width="9" height="9" rx="2"/><rect x="13" y="3" width="9" height="9" rx="2"/><rect x="13" y="14" width="9" height="7" rx="2"/></Ico>;
I.firmware = (p) => <Ico {...p}><path d="M12 3v12"/><path d="m8 11 4 4 4-4"/><rect x="3" y="17" width="18" height="4" rx="1"/></Ico>;
I.lock = (p) => <Ico {...p}><rect x="4" y="11" width="16" height="10" rx="2"/><path d="M8 11V7a4 4 0 0 1 8 0v4"/></Ico>;
I.unlock = (p) => <Ico {...p}><rect x="4" y="11" width="16" height="10" rx="2"/><path d="M8 11V7a4 4 0 0 1 8 0"/></Ico>;
I.layers = (p) => <Ico {...p}><path d="m12 3-10 6 10 6 10-6z"/><path d="m2 15 10 6 10-6"/></Ico>;
I.maximize = (p) => <Ico {...p}><path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/></Ico>;
I.recenter = (p) => <Ico {...p}><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/></Ico>;
I.dot = (p) => <Ico {...p}><circle cx="12" cy="12" r="4" fill="currentColor" stroke="none"/></Ico>;
I.arrowR = (p) => <Ico {...p}><path d="M5 12h14M13 6l6 6-6 6"/></Ico>;
I.arrowL = (p) => <Ico {...p}><path d="M19 12H5M11 18l-6-6 6-6"/></Ico>;
I.arrowU = (p) => <Ico {...p}><path d="M12 19V5M6 11l6-6 6 6"/></Ico>;
I.arrowD = (p) => <Ico {...p}><path d="M12 5v14M18 13l-6 6-6-6"/></Ico>;
I.image = (p) => <Ico {...p}><rect x="3" y="3" width="18" height="18" rx="2"/><circle cx="9" cy="9" r="2"/><path d="m21 15-5-5L5 21"/></Ico>;
I.gauge = (p) => <Ico {...p}><path d="M12 14l3-3"/><path d="M3.5 16a9 9 0 1 1 17 0"/></Ico>;
I.zap = (p) => <Ico {...p}><path d="M13 2 4 14h7l-1 8 9-12h-7z" fill="currentColor" stroke="none"/></Ico>;
I.link = (p) => <Ico {...p}><path d="M10 14a4 4 0 0 0 5.6 0l3-3a4 4 0 0 0-5.6-5.6l-1 1"/><path d="M14 10a4 4 0 0 0-5.6 0l-3 3a4 4 0 0 0 5.6 5.6l1-1"/></Ico>;
I.disk = (p) => <Ico {...p}><rect x="3" y="3" width="18" height="18" rx="2"/><path d="M7 3v8h10V3"/><circle cx="12" cy="15" r="2"/></Ico>;
I.terminal = (p) => <Ico {...p}><rect x="3" y="4" width="18" height="16" rx="2"/><path d="m7 9 3 3-3 3M13 15h5"/></Ico>;
I.share = (p) => <Ico {...p}><circle cx="6" cy="12" r="2"/><circle cx="18" cy="6" r="2"/><circle cx="18" cy="18" r="2"/><path d="M7.8 11 16 7M7.8 13 16 17"/></Ico>;
I.copy = (p) => <Ico {...p}><rect x="8" y="8" width="13" height="13" rx="2"/><path d="M16 8V5a2 2 0 0 0-2-2H5a2 2 0 0 0-2 2v9a2 2 0 0 0 2 2h3"/></Ico>;
I.trash = (p) => <Ico {...p}><path d="M3 6h18M8 6V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2M6 6l1 14a2 2 0 0 0 2 2h6a2 2 0 0 0 2-2l1-14"/></Ico>;
I.sliders = (p) => <Ico {...p}><path d="M4 21v-7M4 10V3M12 21v-9M12 8V3M20 21v-5M20 12V3M1 14h6M9 8h6M17 16h6"/></Ico>;
I.satellite = (p) => <Ico {...p}><path d="M5 11 11 5l3 3-6 6z"/><path d="m11 9 4 4"/><path d="M8 14l-3 3 3 3 3-3"/><path d="M14 11a3 3 0 0 1 3 3M14 7a7 7 0 0 1 7 7"/></Ico>;

window.I = I;
