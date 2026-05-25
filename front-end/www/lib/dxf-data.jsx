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
// MOCK-DATA-COMMENTED: buildSoccerFieldDXF removed

// ─── Tennis court — ITF doubles 23.77 × 10.97 m ───
// MOCK-DATA-COMMENTED: buildTennisCourtDXF removed

// ─── Basketball court — FIBA 28 × 15 m ───────────
// MOCK-DATA-COMMENTED: buildBasketballCourtDXF removed

// ─── Zebra crossing — 6 stripes × 5m long ────────
// MOCK-DATA-COMMENTED: buildZebraCrossingDXF removed

// ─── Road arrows (lane direction markings) ───────
// MOCK-DATA-COMMENTED: buildRoadArrowsDXF removed

// MOCK-DATA-COMMENTED: arrowStraight removed

// ─── Bike lane symbol ────────────────────────────
// MOCK-DATA-COMMENTED: buildBikeLaneDXF removed

// MOCK-DATA-COMMENTED: DXF_TEMPLATES - mock sports/road marking templates removed
const DXF_TEMPLATES = [];

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
  DXF_LAYERS, DXF_TEMPLATES,
  entityToSvg, entityEndpoint, entityLabel,
  // MOCK-DATA-COMMENTED: build*DXF functions removed
});
