// DXF mock dataset + entity geometry helpers.
// Sports fields, road markings, bike lanes.

const DXF_LAYERS = {
  // Sports
  TOUCHLINES:     { name: 'TOUCHLINES',     color: '#fafafa', lt: 'CONTINUOUS' },
  BOX_LINES:      { name: 'BOX_LINES',      color: '#fafafa', lt: 'CONTINUOUS' },
  MARKERS:        { name: 'MARKERS',        color: '#fde047', lt: 'CONTINUOUS' },
  SERVICE_LINES:  { name: 'SERVICE_LINES',  color: '#fafafa', lt: 'CONTINUOUS' },
  NET:            { name: 'NET',            color: '#22d3ee', lt: 'DASHED' },
  KEY:            { name: 'KEY',            color: '#fb7185', lt: 'CONTINUOUS' },
  THREE_PT:       { name: '3PT_ARC',        color: '#fde047', lt: 'CONTINUOUS' },
  // Road
  STRIPES:        { name: 'STRIPES',        color: '#fafafa', lt: 'CONTINUOUS' },
  STOP_LINE:      { name: 'STOP_LINE',      color: '#fafafa', lt: 'CONTINUOUS' },
  ARROW:          { name: 'ARROW',          color: '#fafafa', lt: 'CONTINUOUS' },
  CENTERLINE:     { name: 'CENTERLINE',     color: '#fde047', lt: 'DASHED' },
  BIKE_ICON:      { name: 'BIKE_ICON',      color: '#fafafa', lt: 'CONTINUOUS' },
  CHEVRON:        { name: 'CHEVRON',        color: '#fafafa', lt: 'CONTINUOUS' },
  LANE_OUTLINE:   { name: 'LANE_OUTLINE',   color: '#22d3ee', lt: 'CONTINUOUS' },
  DIMENSIONS:     { name: 'DIMENSIONS',     color: '#22d3ee', lt: 'DASHDOT' },
};

// Builder helpers (1 dxf unit = 10cm)
function makeBuilder() {
  const e = [];
  let id = 1;
  const next = () => `E${String(id++).padStart(3, '0')}`;
  return {
    entities: e,
    line: (x1, y1, x2, y2, layer) => e.push({
      id: next(), type: 'LINE', layer, color: DXF_LAYERS[layer].color,
      x1, y1, x2, y2, closed: false,
      length: Math.hypot(x2-x1, y2-y1) / 10,
    }),
    rect: (x, y, w, h, layer) => {
      const r = makeBuilder();
      e.push({ id: next(), type: 'LINE', layer, color: DXF_LAYERS[layer].color, x1: x, y1: y, x2: x+w, y2: y, closed: false, length: w/10 });
      e.push({ id: next(), type: 'LINE', layer, color: DXF_LAYERS[layer].color, x1: x+w, y1: y, x2: x+w, y2: y+h, closed: false, length: h/10 });
      e.push({ id: next(), type: 'LINE', layer, color: DXF_LAYERS[layer].color, x1: x+w, y1: y+h, x2: x, y2: y+h, closed: false, length: w/10 });
      e.push({ id: next(), type: 'LINE', layer, color: DXF_LAYERS[layer].color, x1: x, y1: y+h, x2: x, y2: y, closed: false, length: h/10 });
    },
    circle: (cx, cy, r, layer) => e.push({
      id: next(), type: 'CIRCLE', layer, color: DXF_LAYERS[layer].color,
      cx, cy, r, closed: true, length: (2 * Math.PI * r) / 10,
    }),
    arc: (cx, cy, r, a1, a2, layer) => e.push({
      id: next(), type: 'ARC', layer, color: DXF_LAYERS[layer].color,
      cx, cy, r, a1, a2, closed: false,
      length: (r * (Math.abs(a2-a1) * Math.PI / 180)) / 10,
    }),
    point: (x, y, layer) => e.push({
      id: next(), type: 'POINT', layer, color: DXF_LAYERS[layer].color,
      x, y, closed: false, length: 0,
    }),
  };
}

// ─── Soccer field — FIFA 105 × 68 m ───────────────
function buildSoccerFieldDXF() {
  const b = makeBuilder();
  const W = 1050, H = 680;
  // Touchlines
  b.line(0, 0, W, 0, 'TOUCHLINES');
  b.line(W, 0, W, H, 'TOUCHLINES');
  b.line(W, H, 0, H, 'TOUCHLINES');
  b.line(0, H, 0, 0, 'TOUCHLINES');
  b.line(W/2, 0, W/2, H, 'TOUCHLINES');
  // Penalty boxes (left/right)
  b.line(0, 138, 165, 138, 'BOX_LINES');
  b.line(165, 138, 165, 542, 'BOX_LINES');
  b.line(165, 542, 0, 542, 'BOX_LINES');
  b.line(W, 138, W-165, 138, 'BOX_LINES');
  b.line(W-165, 138, W-165, 542, 'BOX_LINES');
  b.line(W-165, 542, W, 542, 'BOX_LINES');
  // Goal boxes
  b.line(0, 247, 55, 247, 'BOX_LINES');
  b.line(55, 247, 55, 433, 'BOX_LINES');
  b.line(55, 433, 0, 433, 'BOX_LINES');
  b.line(W, 247, W-55, 247, 'BOX_LINES');
  b.line(W-55, 247, W-55, 433, 'BOX_LINES');
  b.line(W-55, 433, W, 433, 'BOX_LINES');
  // Markers
  b.circle(W/2, H/2, 91.5, 'MARKERS');
  b.point(W/2, H/2, 'MARKERS');
  b.point(110, H/2, 'MARKERS');
  b.point(W-110, H/2, 'MARKERS');
  b.arc(110, H/2, 91.5, -53, 53, 'MARKERS');
  b.arc(W-110, H/2, 91.5, 127, 233, 'MARKERS');
  b.arc(0, 0, 10, 0, 90, 'MARKERS');
  b.arc(W, 0, 10, 90, 180, 'MARKERS');
  b.arc(W, H, 10, 180, 270, 'MARKERS');
  b.arc(0, H, 10, 270, 360, 'MARKERS');
  return { name: 'soccer-field-fifa-spec.dxf', size: '184 KB', units: 'meters',
           bounds: { w: W, h: H }, tint: 'pitch', entities: b.entities };
}

// ─── Tennis court — ITF doubles 23.77 × 10.97 m ───
function buildTennisCourtDXF() {
  const b = makeBuilder();
  const W = 237.7, H = 109.7;
  const sideMargin = (H - 82.3) / 2; // singles court is 8.23m wide
  // Outer doubles rectangle
  b.line(0, 0, W, 0, 'TOUCHLINES');
  b.line(W, 0, W, H, 'TOUCHLINES');
  b.line(W, H, 0, H, 'TOUCHLINES');
  b.line(0, H, 0, 0, 'TOUCHLINES');
  // Singles sidelines
  b.line(0, sideMargin, W, sideMargin, 'TOUCHLINES');
  b.line(0, H - sideMargin, W, H - sideMargin, 'TOUCHLINES');
  // Net
  b.line(W/2, 0, W/2, H, 'NET');
  // Service lines (6.4m each side of net)
  b.line(W/2 - 64, sideMargin, W/2 - 64, H - sideMargin, 'SERVICE_LINES');
  b.line(W/2 + 64, sideMargin, W/2 + 64, H - sideMargin, 'SERVICE_LINES');
  // Center service line
  b.line(W/2 - 64, H/2, W/2 + 64, H/2, 'SERVICE_LINES');
  // Center marks on baselines (10cm = 1 dm)
  b.line(0, H/2, 1, H/2, 'MARKERS');
  b.line(W - 1, H/2, W, H/2, 'MARKERS');
  return { name: 'tennis-itf-doubles.dxf', size: '92 KB', units: 'meters',
           bounds: { w: W, h: H }, tint: 'court', entities: b.entities };
}

// ─── Basketball court — FIBA 28 × 15 m ───────────
function buildBasketballCourtDXF() {
  const b = makeBuilder();
  const W = 280, H = 150;
  // Outer
  b.line(0, 0, W, 0, 'TOUCHLINES');
  b.line(W, 0, W, H, 'TOUCHLINES');
  b.line(W, H, 0, H, 'TOUCHLINES');
  b.line(0, H, 0, 0, 'TOUCHLINES');
  // Half line
  b.line(W/2, 0, W/2, H, 'TOUCHLINES');
  // Center circle (1.8m)
  b.circle(W/2, H/2, 18, 'MARKERS');
  // Free-throw key (lane) — 4.9m wide × 5.8m long
  // Left
  b.line(0, H/2 - 24.5, 58, H/2 - 24.5, 'KEY');
  b.line(58, H/2 - 24.5, 58, H/2 + 24.5, 'KEY');
  b.line(58, H/2 + 24.5, 0, H/2 + 24.5, 'KEY');
  // Right
  b.line(W, H/2 - 24.5, W - 58, H/2 - 24.5, 'KEY');
  b.line(W - 58, H/2 - 24.5, W - 58, H/2 + 24.5, 'KEY');
  b.line(W - 58, H/2 + 24.5, W, H/2 + 24.5, 'KEY');
  // Free-throw circles (1.8m r)
  b.circle(58, H/2, 18, 'KEY');
  b.circle(W - 58, H/2, 18, 'KEY');
  // 3-point arcs (6.75m radius, 0.9m from sideline straight segments)
  b.arc(15.7, H/2, 67.5, -68, 68, 'THREE_PT');
  b.arc(W - 15.7, H/2, 67.5, 112, 248, 'THREE_PT');
  // Corner 3-pt straight extensions (3m from baseline)
  b.line(0, H/2 - 67.5 + 4, 30, H/2 - 67.5 + 4, 'THREE_PT');
  b.line(0, H/2 + 67.5 - 4, 30, H/2 + 67.5 - 4, 'THREE_PT');
  b.line(W, H/2 - 67.5 + 4, W - 30, H/2 - 67.5 + 4, 'THREE_PT');
  b.line(W, H/2 + 67.5 - 4, W - 30, H/2 + 67.5 - 4, 'THREE_PT');
  // Baskets
  b.circle(15.7, H/2, 2.25, 'MARKERS');
  b.circle(W - 15.7, H/2, 2.25, 'MARKERS');
  return { name: 'basketball-fiba-court.dxf', size: '78 KB', units: 'meters',
           bounds: { w: W, h: H }, tint: 'court', entities: b.entities };
}

// ─── Zebra crossing — 6 stripes × 5m long ────────
function buildZebraCrossingDXF() {
  const b = makeBuilder();
  const W = 60, H = 80;
  // Stripes: 4 dm wide, 5 dm gap, 5 dm tall... actual: stripes 50cm wide → 5 dm? Real: 500mm wide stripes, ~500mm gap, 4m long.
  // Use: stripe w=4, gap=4, count=7, length=40
  const stripeW = 4, gap = 4, length = 50, count = 7;
  const total = count * stripeW + (count - 1) * gap;
  const startX = (W - total) / 2;
  for (let i = 0; i < count; i++) {
    const x = startX + i * (stripeW + gap);
    b.rect(x, (H - length) / 2, stripeW, length, 'STRIPES');
  }
  // Stop line above
  b.line(0, (H - length) / 2 - 6, W, (H - length) / 2 - 6, 'STOP_LINE');
  // Optional: opposite stop line below
  b.line(0, (H + length) / 2 + 6, W, (H + length) / 2 + 6, 'STOP_LINE');
  return { name: 'zebra-crossing-eu.dxf', size: '34 KB', units: 'meters',
           bounds: { w: W, h: H }, tint: 'road', entities: b.entities };
}

// ─── Road arrows (lane direction markings) ───────
function buildRoadArrowsDXF() {
  const b = makeBuilder();
  const W = 90, H = 130;
  // Straight arrow — left lane (centered at x=22)
  arrowStraight(b, 22, 65, 'ARROW');
  // Turn-left arrow — middle lane (x=45)
  arrowTurnLeft(b, 45, 65, 'ARROW');
  // Straight + Right combined — right lane (x=68)
  arrowStraightRight(b, 68, 65, 'ARROW');
  // Lane separator dashed centerlines (one between each lane)
  for (let y = 0; y < H; y += 8) {
    if (y + 5 > H) continue;
    b.line(33.5, y, 33.5, y + 4, 'CENTERLINE');
    b.line(56.5, y, 56.5, y + 4, 'CENTERLINE');
  }
  return { name: 'road-arrows-lane-markings.dxf', size: '46 KB', units: 'meters',
           bounds: { w: W, h: H }, tint: 'road', entities: b.entities };
}

function arrowStraight(b, cx, cy, layer) {
  // Shaft (rectangle outline) 3w × 30h, head triangle
  const w = 4, shaft = 30, head = 16;
  // shaft
  b.line(cx - w, cy + shaft, cx - w, cy - shaft + head, layer);
  b.line(cx + w, cy + shaft, cx + w, cy - shaft + head, layer);
  b.line(cx - w, cy + shaft, cx + w, cy + shaft, layer);
  // head — triangle
  b.line(cx - w*2.5, cy - shaft + head, cx, cy - shaft, layer);
  b.line(cx, cy - shaft, cx + w*2.5, cy - shaft + head, layer);
  b.line(cx + w*2.5, cy - shaft + head, cx + w, cy - shaft + head, layer);
  b.line(cx - w*2.5, cy - shaft + head, cx - w, cy - shaft + head, layer);
}

function arrowTurnLeft(b, cx, cy, layer) {
  const w = 4, shaft = 28;
  // vertical shaft (right side)
  b.line(cx + w, cy + shaft, cx + w, cy - shaft + 18, layer);
  // horizontal arm
  b.line(cx + w, cy - shaft + 18, cx - shaft/2 + 6, cy - shaft + 18, layer);
  // arrow head left
  b.line(cx - shaft/2 + 6, cy - shaft + 18 - w*2.5, cx - shaft/2 - 4, cy - shaft + 18, layer);
  b.line(cx - shaft/2 - 4, cy - shaft + 18, cx - shaft/2 + 6, cy - shaft + 18 + w*2.5, layer);
  // back lines
  b.line(cx - w, cy + shaft, cx - w, cy - shaft + 18 + w*2.5, layer);
  b.line(cx - w, cy - shaft + 18 + w*2.5, cx - shaft/2 + 6, cy - shaft + 18 + w*2.5, layer);
  b.line(cx + w, cy + shaft, cx - w, cy + shaft, layer);
  b.line(cx + w, cy - shaft + 18, cx - shaft/2 + 6, cy - shaft + 18 - w*2.5, layer);
}

function arrowStraightRight(b, cx, cy, layer) {
  // simple combo: straight arrow + right branch
  arrowStraight(b, cx, cy, layer);
  // Right branch — horizontal arm out of midpoint
  const w = 4, shaft = 30;
  const midY = cy - shaft/3;
  b.line(cx + w, midY + 2, cx + w + 10, midY + 2, layer);
  b.line(cx + w + 10, midY + 2 + w*2, cx + w + 16, midY + 2, layer);
  b.line(cx + w + 16, midY + 2, cx + w + 10, midY + 2 - w*2, layer);
  b.line(cx + w + 10, midY + 2 - w*2, cx + w + 10, midY + 2 + w*2, layer);
}

// ─── Bike lane symbol ────────────────────────────
function buildBikeLaneDXF() {
  const b = makeBuilder();
  const W = 50, H = 110;
  // Lane outline
  b.line(0, 0, W, 0, 'LANE_OUTLINE');
  b.line(W, 0, W, H, 'LANE_OUTLINE');
  b.line(W, H, 0, H, 'LANE_OUTLINE');
  b.line(0, H, 0, 0, 'LANE_OUTLINE');

  // Bike icon (centered around y=35)
  const cx = W/2, cy = 35;
  // Wheels
  b.circle(cx - 11, cy, 6, 'BIKE_ICON');
  b.circle(cx + 11, cy, 6, 'BIKE_ICON');
  // Frame triangle (top tube, down tube, seat tube)
  b.line(cx - 11, cy, cx + 3, cy - 8, 'BIKE_ICON');
  b.line(cx + 3, cy - 8, cx + 11, cy, 'BIKE_ICON');
  b.line(cx - 11, cy, cx + 11, cy, 'BIKE_ICON');
  b.line(cx + 3, cy - 8, cx + 5, cy, 'BIKE_ICON');
  // Handlebar stem
  b.line(cx - 6, cy - 14, cx - 11, cy - 14, 'BIKE_ICON');
  b.line(cx - 8, cy - 14, cx - 1, cy - 8, 'BIKE_ICON');
  // Seat
  b.line(cx + 1, cy - 13, cx + 6, cy - 13, 'BIKE_ICON');
  b.line(cx + 3, cy - 13, cx + 3, cy - 8, 'BIKE_ICON');

  // Direction chevrons (3 down the lane, pointing forward = -Y)
  for (let i = 0; i < 3; i++) {
    const y = 60 + i * 14;
    b.line(cx - 10, y + 6, cx, y, 'CHEVRON');
    b.line(cx, y, cx + 10, y + 6, 'CHEVRON');
  }
  return { name: 'bike-lane-marking.dxf', size: '52 KB', units: 'meters',
           bounds: { w: W, h: H }, tint: 'road', entities: b.entities };
}

const DXF_TEMPLATES = [
  { id: 'soccer',  name: 'Soccer field (FIFA)',  icon: '⚽', build: buildSoccerFieldDXF },
  { id: 'tennis',  name: 'Tennis court (ITF)',   icon: '🎾', build: buildTennisCourtDXF },
  { id: 'basket',  name: 'Basketball (FIBA)',    icon: '🏀', build: buildBasketballCourtDXF },
  { id: 'zebra',   name: 'Zebra crossing',       icon: '🚸', build: buildZebraCrossingDXF },
  { id: 'arrow',   name: 'Road arrows',          icon: '⬆', build: buildRoadArrowsDXF },
  { id: 'bike',    name: 'Bike lane marking',    icon: '🚴', build: buildBikeLaneDXF },
];

// Render a single entity to SVG
function entityToSvg(en, key, opts = {}) {
  const stroke = opts.stroke || en.color || '#fafafa';
  const sw = opts.strokeWidth || 1.5;
  const dash = opts.dash;
  const common = { stroke, strokeWidth: sw, fill: 'none', strokeLinecap: 'round', strokeDasharray: dash };

  if (en.type === 'LINE') return <line key={key} x1={en.x1} y1={en.y1} x2={en.x2} y2={en.y2} {...common} />;
  if (en.type === 'CIRCLE') return <circle key={key} cx={en.cx} cy={en.cy} r={en.r} {...common} />;
  if (en.type === 'POINT') return <circle key={key} cx={en.x} cy={en.y} r={Math.max(2, sw*1.5)} fill={stroke} stroke="none" />;
  if (en.type === 'ARC') {
    const rad = a => a * Math.PI / 180;
    const x1 = en.cx + en.r * Math.cos(rad(en.a1)), y1 = en.cy + en.r * Math.sin(rad(en.a1));
    const x2 = en.cx + en.r * Math.cos(rad(en.a2)), y2 = en.cy + en.r * Math.sin(rad(en.a2));
    const sweep = ((en.a2 - en.a1) % 360 + 360) % 360;
    const large = sweep > 180 ? 1 : 0;
    return <path key={key} d={`M${x1},${y1} A${en.r},${en.r} 0 ${large} 1 ${x2},${y2}`} {...common} />;
  }
  return null;
}

function entityEndpoint(en, which = 'end') {
  if (en.type === 'LINE') return which === 'start' ? [en.x1, en.y1] : [en.x2, en.y2];
  if (en.type === 'CIRCLE') return [en.cx + en.r, en.cy];
  if (en.type === 'POINT') return [en.x, en.y];
  if (en.type === 'ARC') {
    const a = which === 'start' ? en.a1 : en.a2;
    return [en.cx + en.r * Math.cos(a*Math.PI/180), en.cy + en.r * Math.sin(a*Math.PI/180)];
  }
  return [0, 0];
}

function entityLabel(en) {
  if (en.type === 'LINE') return `Line · ${en.length.toFixed(2)} m`;
  if (en.type === 'CIRCLE') return `Circle · r=${(en.r/10).toFixed(2)} m`;
  if (en.type === 'POINT') return `Point`;
  if (en.type === 'ARC') return `Arc · r=${(en.r/10).toFixed(2)} m · ${Math.round(Math.abs(en.a2-en.a1))}°`;
  return en.type;
}

Object.assign(window, {
  DXF_LAYERS, DXF_TEMPLATES, buildSoccerFieldDXF, buildTennisCourtDXF,
  buildBasketballCourtDXF, buildZebraCrossingDXF, buildRoadArrowsDXF, buildBikeLaneDXF,
  entityToSvg, entityEndpoint, entityLabel,
});
