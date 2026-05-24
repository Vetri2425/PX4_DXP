// DXF Inspector — full-screen overlay for parsing/selecting DXF entities.

function DXFInspector({ dxf, initialSelected, onCancel, onConfirm }) {
  const [selected, setSelected] = React.useState(() => initialSelected || new Set(dxf.entities.map((e) => e.id)));
  const [expanded, setExpanded] = React.useState(null); // entity id being expanded
  const [view, setView] = React.useState('flat'); // flat | type | layer
  const [filterOpen, setFilterOpen] = React.useState(false);
  const [collapsedGroups, setCollapsedGroups] = React.useState(new Set());

  // Per-entity overrides
  const [overrides, setOverrides] = React.useState({}); // id -> { traverse, deleted, pen, scale, offsetX, offsetY }
  // Drag-reorder
  const [order, setOrder] = React.useState(() => dxf.entities.map((e) => e.id));

  const idMap = React.useMemo(() => {
    const m = {};
    dxf.entities.forEach((e) => m[e.id] = e);
    return m;
  }, [dxf]);

  const isDeleted = (id) => overrides[id]?.deleted;
  const isTraverse = (id) => overrides[id]?.traverse;
  const visibleIds = order.filter((id) => !isDeleted(id));
  const liveEntities = visibleIds.map((id) => idMap[id]);

  // Stats
  const drawIds = visibleIds.filter((id) => selected.has(id) && !isTraverse(id));
  const drawEntities = drawIds.map((id) => idMap[id]);
  const totalLen = drawEntities.reduce((s, e) => s + (e.length || 0), 0);
  const drawTimeSec = totalLen / 0.4; // 0.4 m/s
  const formatTime = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;

  // Bulk selectors
  const toggle = (id) => setSelected((s) => {
    const n = new Set(s);
    n.has(id) ? n.delete(id) : n.add(id);
    return n;
  });
  const setMany = (ids, on) => setSelected((s) => {
    const n = new Set(s);
    ids.forEach((i) => on ? n.add(i) : n.delete(i));
    return n;
  });
  const selectAll = () => setSelected(new Set(visibleIds));
  const invert = () => setSelected((s) => new Set(visibleIds.filter((i) => !s.has(i))));
  const selectClosed = () => setSelected(new Set(visibleIds.filter((i) => idMap[i].closed)));
  const selectByType = (type) => setSelected(new Set(visibleIds.filter((i) => idMap[i].type === type)));
  const selectByLayer = (layer) => setSelected(new Set(visibleIds.filter((i) => idMap[i].layer === layer)));

  // ─ Render ────────────────────────────────────────
  return (
    <div style={{
      position: 'absolute', inset: 0, zIndex: 80,
      background: C.bg, display: 'flex', flexDirection: 'column'
    }}>
      <DXFTopBar dxf={dxf} onCancel={onCancel} onConfirm={() => onConfirm({ selected, overrides, order })} />

      {/* Preview canvas */}
      <DXFPreview dxf={dxf} entities={liveEntities} selected={selected}
      isTraverse={isTraverse} order={order} idMap={idMap} />

      {/* Stats strip */}
      <DXFStatsStrip
        selectedCount={drawIds.length}
        total={visibleIds.length}
        totalLen={totalLen}
        time={formatTime(drawTimeSec)} />
      

      {/* View toggle + filter button */}
      <div style={{ padding: '0 16px 8px', display: 'flex', gap: 8, alignItems: 'center' }}>
        <div style={{ display: 'flex', gap: 4, background: C.card, padding: 3, borderRadius: 10, border: `1px solid ${C.line}`, flex: 1 }}>
          {[{ k: 'flat', l: 'Flat' }, { k: 'type', l: 'By type' }, { k: 'layer', l: 'By layer' }].map((v) =>
          <button key={v.k} onClick={() => setView(v.k)} style={{
            flex: 1, padding: '6px 4px', border: 'none', borderRadius: 8, cursor: 'pointer',
            background: view === v.k ? `${C.accent}22` : 'transparent',
            color: view === v.k ? C.accent : C.text2,
            fontSize: 11, fontWeight: 600
          }}>{v.l}</button>
          )}
        </div>
        <IconBtn icon={<I.sliders size={16} />} onClick={() => setFilterOpen(true)} size={32} accent />
      </div>

      {/* Entity list (scrolls) */}
      <div style={{ flex: 1, overflow: 'auto', padding: '0 16px 12px' }}>
        {view === 'flat' &&
        <FlatList
          ids={visibleIds} idMap={idMap} order={order} setOrder={setOrder}
          selected={selected} toggle={toggle} expanded={expanded} setExpanded={setExpanded}
          overrides={overrides} setOverrides={setOverrides} />

        }
        {view === 'type' &&
        <GroupedList groupBy="type" idMap={idMap} ids={visibleIds}
        collapsed={collapsedGroups} setCollapsed={setCollapsedGroups}
        selected={selected} toggle={toggle} setMany={setMany}
        expanded={expanded} setExpanded={setExpanded}
        overrides={overrides} setOverrides={setOverrides} />
        }
        {view === 'layer' &&
        <GroupedList groupBy="layer" idMap={idMap} ids={visibleIds}
        collapsed={collapsedGroups} setCollapsed={setCollapsedGroups}
        selected={selected} toggle={toggle} setMany={setMany}
        expanded={expanded} setExpanded={setExpanded}
        overrides={overrides} setOverrides={setOverrides} />
        }
      </div>

      {/* Bottom action bar */}
      <div style={{
        padding: '12px 16px 28px', borderTop: `1px solid ${C.line}`,
        background: `linear-gradient(180deg, transparent, ${C.bg})`,
        display: 'flex', gap: 10
      }}>
        <Btn variant="secondary" full onClick={onCancel}>Cancel</Btn>
        <Btn variant="primary" full icon={<I.check size={16} />}
        onClick={() => onConfirm({ selected, overrides, order })}>
          Use selection ({drawIds.length})
        </Btn>
      </div>

      {/* Filter sheet */}
      {filterOpen &&
      <FilterSheet
        dxf={dxf} idMap={idMap} visibleIds={visibleIds} selected={selected}
        setMany={setMany} selectAll={selectAll} invert={invert}
        selectClosed={selectClosed} selectByType={selectByType} selectByLayer={selectByLayer}
        onClose={() => setFilterOpen(false)} />

      }
    </div>);

}

// ─── Top bar ─────────────────────────────────────
function DXFTopBar({ dxf, onCancel }) {
  const usedLayers = new Set(dxf.entities.map((e) => e.layer)).size;
  return (
    <div style={{
      padding: '54px 14px 14px',
      borderBottom: `1px solid ${C.line}`,
      background: `linear-gradient(180deg, ${C.bg}fa, ${C.bg}cc)`,
      backdropFilter: 'blur(20px)', display: 'flex', alignItems: 'center', gap: 12
    }}>
      <IconBtn icon={<I.x size={18} />} onClick={onCancel} />
      <div style={{ flex: 1, minWidth: 0 }}>
        <div style={{ fontSize: 11, color: C.text3, fontWeight: 600, textTransform: 'uppercase', letterSpacing: 0.7 }}>
          DXF Inspector
        </div>
        <div style={{ fontSize: 15, fontWeight: 600, fontFamily: 'var(--mono)' }}>{dxf.name}</div>
        <div style={{ fontSize: 11, color: C.text3, fontFamily: 'var(--mono)' }}>
          {dxf.entities.length} entities · {usedLayers} layer{usedLayers === 1 ? '' : 's'} · {dxf.size}
        </div>
      </div>
    </div>);

}

// ─── Preview canvas ──────────────────────────────
function DXFPreview({ dxf, entities, selected, isTraverse, order, idMap }) {
  const w = dxf.bounds.w,h = dxf.bounds.h;
  // Adaptive grid + stroke widths — small templates get finer grids & thinner strokes
  const small = Math.max(w, h) < 200;
  const gMinor = small ? 5 : 50;
  const gMajor = small ? 25 : 250;
  const drawSW = Math.max(0.8, Math.min(w, h) * 0.012);
  const padding = Math.max(2, Math.min(w, h) * 0.03);
  const drawList = order.filter((id) => selected.has(id) && !isTraverse(id) && entities.some((e) => e.id === id));
  // Travel path: pen-up moves between consecutive drawn entities
  const travel = [];
  for (let i = 1; i < drawList.length; i++) {
    const prev = idMap[drawList[i - 1]];
    const cur = idMap[drawList[i]];
    if (!prev || !cur) continue;
    const [px, py] = entityEndpoint(prev, 'end');
    const [cx, cy] = entityEndpoint(cur, 'start');
    travel.push(`M${px},${py} L${cx},${cy}`);
  }

  return (
    <div style={{ padding: '12px 16px 8px' }}>
      <div style={{
        position: 'relative', borderRadius: 14, overflow: 'hidden',
        background: '#fafafa', border: `1px solid ${C.line2}`,
        aspectRatio: `${w} / ${h}`,
        maxHeight: 280,
        margin: '0 auto',
      }}>
        {/* Grid */}
        <svg viewBox={`${-padding} ${-padding} ${w + padding*2} ${h + padding*2}`} preserveAspectRatio="xMidYMid meet" width="100%" height="100%">
          <defs>
            <pattern id="dxgrid" width={gMinor} height={gMinor} patternUnits="userSpaceOnUse">
              <path d={`M${gMinor} 0 L0 0 0 ${gMinor}`} stroke="rgba(0,0,0,0.06)" strokeWidth="0.5" fill="none" />
            </pattern>
            <pattern id="dxgrid2" width={gMajor} height={gMajor} patternUnits="userSpaceOnUse">
              <path d={`M${gMajor} 0 L0 0 0 ${gMajor}`} stroke="rgba(0,0,0,0.12)" strokeWidth="0.8" fill="none" />
            </pattern>
          </defs>
          <rect x={-padding} y={-padding} width={w + padding*2} height={h + padding*2} fill="url(#dxgrid)" />
          <rect x={-padding} y={-padding} width={w + padding*2} height={h + padding*2} fill="url(#dxgrid2)" />

          {/* Tint behind everything */}
          {dxf.tint === 'pitch' &&
          <rect x="0" y="0" width={w} height={h} fill="rgba(34,197,94,0.10)" />
          }
          {dxf.tint === 'court' &&
          <rect x="0" y="0" width={w} height={h} fill="rgba(56,189,248,0.08)" />
          }
          {dxf.tint === 'road' &&
          <rect x="0" y="0" width={w} height={h} fill="rgba(80,80,90,0.10)" />
          }

          {/* Selected entities */}
          {entities.filter((e) => selected.has(e.id) && !isTraverse(e.id)).
          map((e, i) => entityToSvg(e, e.id, { stroke: '#0a0d12', strokeWidth: drawSW * 2 }))}

          {/* Traverse entities (dimmed dotted) */}
          {entities.filter((e) => selected.has(e.id) && isTraverse(e.id)).
          map((e, i) => entityToSvg(e, 't' + e.id, { stroke: '#9ca3af', strokeWidth: drawSW * 1.3, dash: `${drawSW*3} ${drawSW*3}` }))}

          {/* Travel path between selected entities */}
          {travel.map((d, i) =>
          <path key={'tv' + i} d={d} stroke="#ec4899" strokeWidth={drawSW * 1.3} fill="none"
          strokeDasharray={`${drawSW*2} ${drawSW*3}`} opacity="0.7" />
          )}
        </svg>

        {/* Corner labels */}
        <div style={{ position: 'absolute', top: 6, left: 8, fontSize: 10, color: 'rgba(0,0,0,0.5)', fontFamily: 'var(--mono)' }}>
          0,0
        </div>
        <div style={{ position: 'absolute', bottom: 6, right: 8, fontSize: 10, color: 'rgba(0,0,0,0.5)', fontFamily: 'var(--mono)' }}>
          {(w / 10).toFixed(2)} × {(h / 10).toFixed(2)} m
        </div>
        {/* Travel-path legend */}
        <div style={{ position: 'absolute', top: 8, right: 8, display: 'flex', gap: 8, fontSize: 9, fontFamily: 'var(--mono)' }}>
          <span style={{ display: 'inline-flex', alignItems: 'center', gap: 4, color: 'rgba(0,0,0,0.65)' }}>
            <span style={{ width: 14, height: 2, background: '#0a0d12' }} /> draw
          </span>
          <span style={{ display: 'inline-flex', alignItems: 'center', gap: 4, color: 'rgba(236,72,153,0.85)' }}>
            <span style={{ width: 14, borderTop: '2px dashed #ec4899' }} /> travel
          </span>
        </div>
      </div>
    </div>);

}

// ─── Stats strip ─────────────────────────────────
function DXFStatsStrip({ selectedCount, total, totalLen, time }) {
  return (
    <div style={{ padding: '0 16px 10px', display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: 6 }}>
      <DStat label="Selected" value={`${selectedCount}/${total}`} color={C.accent} />
      <DStat label="Total length" value={`${totalLen.toFixed(1)} m`} />
      <DStat label="Est. time" value={time} color={C.warn} />
    </div>);

}
function DStat({ label, value, color = C.text }) {
  return (
    <div style={{ padding: '6px 10px', borderRadius: 10, background: C.card2, border: `1px solid ${C.line}` }}>
      <div style={{ fontSize: 9, color: C.text3, textTransform: 'uppercase', letterSpacing: 0.6, fontWeight: 600 }}>{label}</div>
      <div style={{ fontFamily: 'var(--mono)', fontSize: 14, fontWeight: 600, color, marginTop: 1 }}>{value}</div>
    </div>);

}

// ─── Flat list ───────────────────────────────────
function FlatList({ ids, idMap, order, setOrder, selected, toggle, expanded, setExpanded, overrides, setOverrides }) {
  const dragIdRef = React.useRef(null);
  const onDragStart = (id) => () => {dragIdRef.current = id;};
  const onDragOver = (id) => (e) => {
    e.preventDefault();
    const drag = dragIdRef.current;
    if (!drag || drag === id) return;
    setOrder((o) => {
      const arr = o.slice();
      const di = arr.indexOf(drag),ti = arr.indexOf(id);
      if (di < 0 || ti < 0) return o;
      arr.splice(di, 1);
      arr.splice(ti, 0, drag);
      return arr;
    });
  };
  const onDragEnd = () => {dragIdRef.current = null;};

  return (
    <div>
      {ids.map((id, idx) =>
      <EntityRow key={id} en={idMap[id]} idx={idx} checked={selected.has(id)} onToggle={() => toggle(id)}
      expanded={expanded === id} onExpand={() => setExpanded(expanded === id ? null : id)}
      overrides={overrides[id]} setOverride={(patch) => setOverrides((o) => ({ ...o, [id]: { ...o[id], ...patch } }))}
      draggable
      onDragStart={onDragStart(id)} onDragOver={onDragOver(id)} onDragEnd={onDragEnd} />
      )}
    </div>);

}

// ─── Grouped list ────────────────────────────────
function GroupedList({ groupBy, idMap, ids, collapsed, setCollapsed, selected, toggle, setMany, expanded, setExpanded, overrides, setOverrides }) {
  // Group ids by entity[groupBy]
  const groups = {};
  ids.forEach((id) => {
    const k = idMap[id][groupBy];
    (groups[k] = groups[k] || []).push(id);
  });

  return (
    <div>
      {Object.entries(groups).map(([k, gIds]) => {
        const allOn = gIds.every((i) => selected.has(i));
        const someOn = gIds.some((i) => selected.has(i));
        const isCollapsed = collapsed.has(k);
        const layerColor = groupBy === 'layer' ? DXF_LAYERS[k]?.color : C.accent;

        return (
          <div key={k} style={{ marginBottom: 10 }}>
            <div style={{
              display: 'flex', alignItems: 'center', gap: 10, padding: '10px 12px',
              background: C.card, border: `1px solid ${C.line}`, borderRadius: 12
            }}>
              <Checkbox checked={allOn} indeterminate={!allOn && someOn}
              onClick={() => setMany(gIds, !allOn)} />
              <button onClick={() => setCollapsed((s) => {const n = new Set(s);n.has(k) ? n.delete(k) : n.add(k);return n;})}
              style={{ background: 'none', border: 'none', color: C.text, cursor: 'pointer', display: 'flex', alignItems: 'center', gap: 8, flex: 1, padding: 0, textAlign: 'left' }}>
                <span style={{ transform: isCollapsed ? 'rotate(-90deg)' : 'rotate(0)', transition: 'transform .15s', color: C.text3 }}>
                  <I.chevD size={14} />
                </span>
                {groupBy === 'layer' &&
                <span style={{ width: 10, height: 10, borderRadius: 2, background: layerColor, flexShrink: 0,
                  boxShadow: 'inset 0 0 0 1px rgba(0,0,0,0.2)' }} />
                }
                <span style={{ fontSize: 13, fontWeight: 600, fontFamily: 'var(--mono)' }}>{k}</span>
                <span style={{ fontSize: 11, color: C.text3, fontFamily: 'var(--mono)' }}>
                  {gIds.filter((i) => selected.has(i)).length}/{gIds.length}
                </span>
              </button>
            </div>
            {!isCollapsed &&
            <div style={{ paddingLeft: 10, marginTop: 4 }}>
                {gIds.map((id, idx) =>
              <EntityRow key={id} en={idMap[id]} idx={idx} checked={selected.has(id)} onToggle={() => toggle(id)}
              expanded={expanded === id} onExpand={() => setExpanded(expanded === id ? null : id)}
              overrides={overrides[id]} setOverride={(p) => setOverrides((o) => ({ ...o, [id]: { ...o[id], ...p } }))} />
              )}
              </div>
            }
          </div>);

      })}
    </div>);

}

// ─── Entity row ──────────────────────────────────
function EntityRow({ en, checked, onToggle, expanded, onExpand, overrides = {}, setOverride, draggable, onDragStart, onDragOver, onDragEnd }) {
  const layerColor = DXF_LAYERS[en.layer]?.color || C.text2;
  const typeIcon = { LINE: '─', CIRCLE: '○', POINT: '·', ARC: '⌒' }[en.type] || '◇';

  return (
    <div draggable={draggable} onDragStart={onDragStart} onDragOver={onDragOver} onDragEnd={onDragEnd}
    style={{
      marginBottom: 4, background: C.card, border: `1px solid ${C.line}`, borderRadius: 10,
      opacity: overrides.traverse ? 0.65 : 1
    }}>
      <div style={{ padding: '10px 10px', display: 'flex', alignItems: 'center', gap: 10 }}>
        {draggable &&
        <span style={{ color: C.text3, cursor: 'grab', fontSize: 14, lineHeight: 1, userSelect: 'none' }}>⋮⋮</span>
        }
        <Checkbox checked={checked} onClick={onToggle} />
        <button onClick={onExpand} style={{
          flex: 1, display: 'flex', alignItems: 'center', gap: 10, padding: 0,
          background: 'none', border: 'none', color: C.text, cursor: 'pointer', textAlign: 'left'
        }}>
          <span style={{
            width: 26, height: 26, borderRadius: 6, background: C.card2,
            border: `1px solid ${C.line}`, display: 'inline-flex', alignItems: 'center', justifyContent: 'center',
            color: layerColor, fontSize: 16, fontWeight: 700, flexShrink: 0
          }}>{typeIcon}</span>
          <span style={{ flex: 1, minWidth: 0 }}>
            <div style={{ fontSize: 13, fontWeight: 500, fontFamily: 'var(--mono)' }}>
              {en.id} · {entityLabel(en)}
              {overrides.traverse && <span style={{ color: C.warn, marginLeft: 6 }}>· traverse</span>}
            </div>
          </span>
          <span style={{ color: C.text3, transform: expanded ? 'rotate(180deg)' : 'rotate(0)', transition: 'transform .15s' }}>
            <I.chevD size={14} />
          </span>
        </button>
      </div>

      {expanded &&
      <div style={{ borderTop: `1px solid ${C.line}`, padding: '10px 12px', background: 'rgba(255,255,255,0.015)' }}>
          {/* Metadata grid */}
          <div style={{ display: 'grid', gridTemplateColumns: 'auto 1fr', gap: '4px 12px', fontSize: 11, fontFamily: 'var(--mono)', color: C.text2 }}>
            <span style={{ color: C.text3 }}>type</span><span>{en.type}</span>
            <span style={{ color: C.text3 }}>layer</span>
            <span style={{ display: 'inline-flex', alignItems: 'center', gap: 4 }}>
              <span style={{ width: 8, height: 8, borderRadius: 1, background: layerColor }} />{en.layer}
            </span>
            <span style={{ color: C.text3 }}>color</span>
            <span style={{ display: 'inline-flex', alignItems: 'center', gap: 4 }}>
              <span style={{ width: 8, height: 8, borderRadius: 1, background: en.color }} />{en.color}
            </span>
            <span style={{ color: C.text3 }}>length</span><span>{en.length.toFixed(3)} m</span>
            <span style={{ color: C.text3 }}>closed</span><span>{en.closed ? 'yes' : 'no'}</span>
            {en.type === 'CIRCLE' && <>
              <span style={{ color: C.text3 }}>center</span><span>{(en.cx / 10).toFixed(2)}, {(en.cy / 10).toFixed(2)}</span>
              <span style={{ color: C.text3 }}>radius</span><span>{(en.r / 10).toFixed(2)} m</span>
            </>}
            {en.type === 'LINE' && <>
              <span style={{ color: C.text3 }}>p1</span><span>{(en.x1 / 10).toFixed(2)}, {(en.y1 / 10).toFixed(2)}</span>
              <span style={{ color: C.text3 }}>p2</span><span>{(en.x2 / 10).toFixed(2)}, {(en.y2 / 10).toFixed(2)}</span>
            </>}
            {en.type === 'ARC' && <>
              <span style={{ color: C.text3 }}>center</span><span>{(en.cx / 10).toFixed(2)}, {(en.cy / 10).toFixed(2)}</span>
              <span style={{ color: C.text3 }}>arc</span><span>{en.a1}° → {en.a2}°</span>
            </>}
            {en.type === 'POINT' && <>
              <span style={{ color: C.text3 }}>at</span><span>{(en.x / 10).toFixed(2)}, {(en.y / 10).toFixed(2)}</span>
            </>}
          </div>

          {/* Pen + scale/offset */}
          <div style={{ marginTop: 12, display: 'flex', flexDirection: 'column', gap: 8 }}>
            <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
              <span style={{ fontSize: 11, color: C.text3, textTransform: 'uppercase', letterSpacing: 0.6, fontWeight: 600 }}>Pen</span>
              <div style={{ display: 'flex', gap: 4 }}>
                {[
              ['#0a0d12', 'Black 0.7'], ['#dc2626', 'Red 0.5'], ['#2563eb', 'Blue 0.5'], ['#16a34a', 'Green 0.7']].
              map(([c, n]) =>
              <button key={c} onClick={() => setOverride({ pen: c })}
              style={{ width: 22, height: 22, borderRadius: 5, background: c, border: `1.5px solid ${(overrides.pen || '#0a0d12') === c ? C.accent : 'rgba(255,255,255,0.1)'}`, cursor: 'pointer', padding: 0 }}
              title={n} />
              )}
              </div>
            </div>
            <NumberPair label="Offset" x={overrides.offsetX || 0} y={overrides.offsetY || 0}
          onX={(v) => setOverride({ offsetX: v })} onY={(v) => setOverride({ offsetY: v })} />
            <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
              <span style={{ fontSize: 11, color: C.text3, textTransform: 'uppercase', letterSpacing: 0.6, fontWeight: 600, width: 50 }}>Scale</span>
              <input type="range" min="0.1" max="3" step="0.05" value={overrides.scale || 1}
            onChange={(e) => setOverride({ scale: Number(e.target.value) })}
            style={{ flex: 1, accentColor: C.accent }} />
              <span style={{ fontFamily: 'var(--mono)', fontSize: 11, color: C.accent, width: 36, textAlign: 'right' }}>
                {(overrides.scale || 1).toFixed(2)}x
              </span>
            </div>
          </div>

          {/* Action buttons */}
          <div style={{ marginTop: 12, display: 'flex', gap: 6, flexWrap: 'wrap' }}>
            <ActChip on={overrides.traverse} icon={<I.path size={12} />}
          label="Traverse only" color={C.warn}
          onClick={() => setOverride({ traverse: !overrides.traverse })} />
            <ActChip icon={<I.trash size={12} />} label="Delete" color={C.danger}
          onClick={() => setOverride({ deleted: true })} />
          </div>
        </div>
      }
    </div>);

}

function ActChip({ on, icon, label, color = C.accent, onClick }) {
  return (
    <button onClick={onClick} style={{
      display: 'inline-flex', alignItems: 'center', gap: 5,
      padding: '5px 9px', borderRadius: 999,
      background: on ? `${color}22` : C.card2,
      border: `1px solid ${on ? color + '66' : C.line2}`,
      color: on ? color : C.text2, fontSize: 11, fontWeight: 600, cursor: 'pointer'
    }}>{icon} {label}</button>);

}

function NumberPair({ label, x, y, onX, onY }) {
  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
      <span style={{ fontSize: 11, color: C.text3, textTransform: 'uppercase', letterSpacing: 0.6, fontWeight: 600, width: 50 }}>{label}</span>
      <input type="number" step="0.1" value={x} onChange={(e) => onX(Number(e.target.value))}
      style={inputSty} />
      <input type="number" step="0.1" value={y} onChange={(e) => onY(Number(e.target.value))}
      style={inputSty} />
      <span style={{ fontSize: 10, color: C.text3 }}>m</span>
    </div>);

}
const inputSty = {
  width: 60, padding: '4px 8px', background: C.card2, border: `1px solid ${C.line2}`,
  borderRadius: 6, color: C.text, fontFamily: 'var(--mono)', fontSize: 12, outline: 'none'
};

function Checkbox({ checked, indeterminate, onClick }) {
  return (
    <button onClick={onClick} style={{
      width: 20, height: 20, borderRadius: 5, padding: 0, cursor: 'pointer',
      background: checked || indeterminate ? C.accent : 'transparent',
      border: `1.5px solid ${checked || indeterminate ? C.accent : C.text3}`,
      display: 'inline-flex', alignItems: 'center', justifyContent: 'center', flexShrink: 0,
      color: '#06202a'
    }}>
      {checked && <I.check size={13} strokeWidth={3} />}
      {indeterminate && !checked && <div style={{ width: 10, height: 2, background: '#06202a' }} />}
    </button>);

}

// ─── Filter sheet ────────────────────────────────
function FilterSheet({ dxf, idMap, visibleIds, selected, setMany, selectAll, invert, selectClosed, selectByType, selectByLayer, onClose }) {
  const [lenMin, setLenMin] = React.useState(0);
  const [lenMax, setLenMax] = React.useState(200);

  const types = [...new Set(visibleIds.map((i) => idMap[i].type))];
  const layers = [...new Set(visibleIds.map((i) => idMap[i].layer))];
  const colors = [...new Set(visibleIds.map((i) => idMap[i].color))];

  const selectByLengthRange = () => {
    const ids = visibleIds.filter((i) => {
      const e = idMap[i];
      return e.length >= lenMin && e.length <= lenMax;
    });
    setMany(visibleIds, false);
    setMany(ids, true);
  };
  const selectByColor = (c) => setMany(visibleIds.filter((i) => idMap[i].color === c), true);

  return (
    <div onClick={onClose} style={{
      position: 'absolute', inset: 0, zIndex: 90,
      background: 'rgba(0,0,0,0.5)', backdropFilter: 'blur(8px)',
      display: 'flex', alignItems: 'flex-end'
    }}>
      <div onClick={(e) => e.stopPropagation()} style={{
        width: '100%', background: C.card, borderTopLeftRadius: 20, borderTopRightRadius: 20,
        padding: '14px 16px 28px', maxHeight: '80%', overflow: 'auto', borderTop: `1px solid ${C.line2}`
      }}>
        <div style={{ width: 36, height: 4, background: C.text3, borderRadius: 2, margin: '0 auto 16px' }} />

        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 12 }}>
          <div style={{ fontSize: 16, fontWeight: 700 }}>Bulk select</div>
          <IconBtn icon={<I.x size={14} />} size={28} onClick={onClose} />
        </div>

        <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap', marginBottom: 16 }}>
          <FilterChip label="Select all" onClick={selectAll} />
          <FilterChip label="Invert" onClick={invert} />
          <FilterChip label="Only closed" onClick={selectClosed} />
          <FilterChip label="Clear" onClick={() => setMany(visibleIds, false)} danger />
        </div>

        <FSect label="By type">
          {types.map((t) =>
          <FilterChip key={t} label={`${t} (${visibleIds.filter((i) => idMap[i].type === t).length})`}
          onClick={() => selectByType(t)} />
          )}
        </FSect>

        <FSect label="By layer">
          {layers.map((l) =>
          <FilterChip key={l} icon={<span style={{ width: 8, height: 8, borderRadius: 1, background: DXF_LAYERS[l]?.color, display: 'inline-block' }} />}
          label={`${l} (${visibleIds.filter((i) => idMap[i].layer === l).length})`}
          onClick={() => selectByLayer(l)} />
          )}
        </FSect>

        <FSect label="By color">
          {colors.map((c) =>
          <FilterChip key={c}
          icon={<span style={{ width: 12, height: 12, borderRadius: 2, background: c, display: 'inline-block', border: '1px solid rgba(255,255,255,0.2)' }} />}
          label={c}
          onClick={() => selectByColor(c)} />
          )}
        </FSect>

        <FSect label="By length / radius">
          <div style={{ display: 'flex', gap: 8, alignItems: 'center', width: '100%', marginBottom: 8 }}>
            <span style={{ fontSize: 11, color: C.text3, fontFamily: 'var(--mono)' }}>{lenMin.toFixed(0)} m</span>
            <input type="range" min="0" max="200" value={lenMin}
            onChange={(e) => setLenMin(Number(e.target.value))}
            style={{ flex: 1, accentColor: C.accent }} />
            <span style={{ fontSize: 11, color: C.text3, fontFamily: 'var(--mono)' }}>{lenMin.toFixed(0)}</span>
          </div>
          <div style={{ display: 'flex', gap: 8, alignItems: 'center', width: '100%' }}>
            <span style={{ fontSize: 11, color: C.text3, fontFamily: 'var(--mono)' }}>{lenMax.toFixed(0)} m</span>
            <input type="range" min="0" max="200" value={lenMax}
            onChange={(e) => setLenMax(Number(e.target.value))}
            style={{ flex: 1, accentColor: C.accent }} />
            <span style={{ fontSize: 11, color: C.text3, fontFamily: 'var(--mono)' }}>{lenMax.toFixed(0)}</span>
          </div>
          <Btn variant="primary" size="sm" onClick={selectByLengthRange} style={{ marginTop: 10 }}>
            Select {lenMin}–{lenMax} m
          </Btn>
        </FSect>
      </div>
    </div>);

}
function FSect({ label, children }) {
  return (
    <div style={{ marginBottom: 14 }}>
      <div style={{ fontSize: 11, color: C.text3, textTransform: 'uppercase', letterSpacing: 0.7, fontWeight: 600, marginBottom: 8 }}>
        {label}
      </div>
      <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap' }}>{children}</div>
    </div>);

}
function FilterChip({ label, onClick, icon, danger }) {
  return (
    <button onClick={onClick} style={{
      display: 'inline-flex', alignItems: 'center', gap: 6,
      padding: '7px 12px', borderRadius: 999,
      background: danger ? `${C.danger}14` : C.card2,
      border: `1px solid ${danger ? C.danger + '33' : C.line2}`,
      color: danger ? C.danger : C.text, fontSize: 12, fontWeight: 600, cursor: 'pointer',
      fontFamily: 'var(--mono)'
    }}>{icon} {label}</button>);

}

window.DXFInspector = DXFInspector;