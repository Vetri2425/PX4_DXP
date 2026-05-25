// Draw / Plot job — SVG upload, finger canvas, gallery, G-code, DXF.

function DrawScreen() {
  const r = useRover();
  const [tab, setTab] = React.useState(r.dxfFile ? 'dxf' : 'gallery');
  const [selected, setSelected] = React.useState(r.gallery[0] || null);
  const [scale, setScale] = React.useState(80);
  const [feed, setFeed] = React.useState(40);
  const [pen, setPen] = React.useState('Black 0.7mm');

  return (
    <div style={{ padding: '0 0 110px' }}>
      <AppBar
        title="New Drawing"
        subtitle="Upload SVG · draw · pick · import G-code"
        trailing={<IconBtn icon={<I.refresh size={18}/>} />}
      />

      {/* Tab strip */}
      <div style={{ padding: '0 16px 12px' }}>
        <div style={{ display:'flex', gap: 6, background: C.card, padding: 4, borderRadius: 12, border: `1px solid ${C.line}` }}>
          {[
            { k:'dxf',     l:'DXF',     i:<I.layers size={14}/> },
            { k:'gallery', l:'Gallery', i:<I.image size={14}/> },
            { k:'upload',  l:'SVG',     i:<I.upload size={14}/> },
            { k:'canvas',  l:'Draw',    i:<I.draw size={14}/> },
            { k:'gcode',   l:'G-code',  i:<I.terminal size={14}/> },
          ].map(x => (
            <button key={x.k} onClick={() => setTab(x.k)} style={{
              flex: 1, padding: '8px 6px', borderRadius: 9, border: 'none', cursor: 'pointer',
              background: tab === x.k ? C.accent + '22' : 'transparent',
              color: tab === x.k ? C.accent : C.text2,
              fontSize: 11, fontWeight: 600,
              display:'inline-flex', alignItems:'center', justifyContent:'center', gap: 4, letterSpacing: 0.3,
            }}>{x.i} {x.l}</button>
          ))}
        </div>
      </div>

      {/* Tab content */}
      {tab === 'dxf'     && <DxfPanel />}
      {tab === 'gallery' && <GalleryGrid items={r.gallery} selected={selected} onSelect={setSelected} />}
      {tab === 'upload'  && <UploadPanel onPick={setSelected} />}
      {tab === 'canvas'  && <DrawCanvas />}
      {tab === 'gcode'   && <GcodePanel />}

      {/* Preview */}
      {(tab === 'gallery' || tab === 'upload') && selected && (
        <>
          <SectionHeader title="Preview & settings" />
          <div style={{ padding: '0 16px' }}>
            <Card>
              <div style={{ display:'flex', gap: 12 }}>
                <div style={{
                  width: 100, height: 100, borderRadius: 12,
                  background: '#fafafa', display:'flex', alignItems:'center', justifyContent:'center',
                  flexShrink: 0, border: `1px solid ${C.line2}`,
                }}>
                  <PreviewSvg seed={selected.hash} />
                </div>
                <div style={{ flex: 1, minWidth: 0 }}>
                  <div style={{ fontSize: 15, fontWeight: 600 }}>{selected.name}</div>
                  <div style={{ fontSize: 12, color: C.text3, fontFamily:'var(--mono)', marginTop: 4 }}>
                    {selected.paths} paths · {selected.length} m
                  </div>
                  <div style={{ fontSize: 12, color: C.text3, fontFamily:'var(--mono)', marginTop: 2 }}>
                    sha:{selected.hash}
                  </div>
                  <div style={{ display:'flex', gap: 6, marginTop: 8 }}>
                    <Pill color={C.accent} dim>SVG</Pill>
                    <Pill color={C.violet} dim>{pen.split(' ')[0]}</Pill>
                  </div>
                </div>
              </div>

              <div style={{ marginTop: 14, display:'flex', flexDirection:'column', gap: 10 }}>
                <SliderRow label="Scale" value={scale} unit="%" onChange={setScale} min={10} max={150} />
                <SliderRow label="Feed rate" value={feed} unit="cm/s" onChange={setFeed} min={5} max={80} />
              </div>

              <div style={{ marginTop: 12 }}>
                <div style={{ fontSize: 11, color: C.text3, textTransform:'uppercase', letterSpacing:0.7, fontWeight: 600, marginBottom: 6 }}>Pen</div>
                <div style={{ display:'flex', gap: 6, flexWrap:'wrap' }}>
                  {['Black 0.7mm', 'Red 0.5mm', 'Brush 2mm', 'White 1.0mm'].map(p => (
                    <button key={p} onClick={() => setPen(p)} style={{
                      padding: '6px 10px', borderRadius: 999,
                      background: p === pen ? `${C.accent}22` : C.card2,
                      border: `1px solid ${p === pen ? C.accent + '66' : C.line2}`,
                      color: p === pen ? C.accent : C.text2, fontSize: 11, fontWeight: 600, cursor: 'pointer',
                    }}>{p}</button>
                  ))}
                </div>
              </div>
            </Card>

            <div style={{ marginTop: 12, display:'grid', gridTemplateColumns:'1fr 1fr', gap: 10 }}>
              <Btn variant="secondary" icon={<I.gauge size={16}/>} full>Dry run</Btn>
              <Btn variant="primary" icon={<I.play size={16}/>} full onClick={() => {
                r.setActiveJob({ id: selected.id, name: selected.name, progress: 0, eta: '—', paths: selected.paths, done: 0 });
                r.setMode('Draw');
                r.setTab('home');
              }}>Send to rover</Btn>
            </div>
          </div>
        </>
      )}
    </div>
  );
}

function SliderRow({ label, value, unit, onChange, min, max }) {
  return (
    <div>
      <div style={{ display:'flex', justifyContent:'space-between', alignItems:'center', marginBottom: 4 }}>
        <span style={{ fontSize: 12, color: C.text2, fontWeight: 500 }}>{label}</span>
        <span style={{ fontFamily:'var(--mono)', color: C.accent, fontSize: 13, fontWeight: 600 }}>{value}{unit}</span>
      </div>
      <input type="range" value={value} min={min} max={max}
             onChange={e => onChange(Number(e.target.value))}
             style={{
               width: '100%', appearance: 'none', height: 4, borderRadius: 2,
               background: `linear-gradient(to right, ${C.accent} 0%, ${C.accent} ${((value-min)/(max-min))*100}%, rgba(255,255,255,0.1) ${((value-min)/(max-min))*100}%, rgba(255,255,255,0.1) 100%)`,
               outline:'none',
             }} />
    </div>
  );
}

function GalleryGrid({ items, selected, onSelect }) {
  return (
    <div style={{ padding: '0 16px 16px' }}>
      <div style={{ display:'grid', gridTemplateColumns:'repeat(3, 1fr)', gap: 8 }}>
        {items.map(it => (
          <button key={it.id} onClick={() => onSelect(it)} style={{
            padding: 6, borderRadius: 12, cursor: 'pointer',
            background: selected?.id === it.id ? `${C.accent}22` : C.card,
            border: `1px solid ${selected?.id === it.id ? C.accent + '66' : C.line}`,
            color: C.text, textAlign: 'left',
          }}>
            <div style={{
              aspectRatio: '1/1', borderRadius: 8, background: '#fafafa',
              display:'flex', alignItems:'center', justifyContent:'center', marginBottom: 6,
            }}>
              <PreviewSvg seed={it.hash} small />
            </div>
            <div style={{ fontSize: 11, fontWeight: 600, color: C.text, textWrap: 'pretty' }}>{it.name}</div>
            <div style={{ fontSize: 10, color: C.text3, fontFamily:'var(--mono)', marginTop: 2 }}>{it.paths}p · {it.length}m</div>
          </button>
        ))}
      </div>
    </div>
  );
}

function UploadPanel({ onPick }) {
  const r = useRover();
  return (
    <div style={{ padding: '0 16px 16px' }}>
      <Card pad={20} style={{ borderStyle: 'dashed', borderColor: C.line2 }}>
        <div style={{ textAlign:'center' }}>
          <div style={{ width: 56, height: 56, borderRadius: '50%', background: `${C.accent}1a`, color: C.accent,
                        display:'inline-flex', alignItems:'center', justifyContent:'center', marginBottom: 10 }}>
            <I.upload size={26} />
          </div>
          <div style={{ fontSize: 15, fontWeight: 600 }}>Drop SVG, PDF, or DXF</div>
          <div style={{ fontSize: 12, color: C.text3, marginTop: 4 }}>or pick from iCloud · Files · Dropbox</div>
          <div style={{ marginTop: 14, display:'flex', gap: 8, justifyContent:'center' }}>
            <Btn variant="primary" size="sm" icon={<I.upload size={14}/>} onClick={() => onPick(r.gallery[1] || { id: 'upload', name: 'Uploaded SVG', paths: 0, length: 0, hash: '0000' })}>Choose file</Btn>
            <Btn variant="secondary" size="sm">Camera scan</Btn>
          </div>
        </div>
      </Card>
      <div style={{ marginTop: 12, fontSize: 11, color: C.text3 }}>
        <I.info size={11} style={{ verticalAlign: -2, marginRight: 4 }}/> Max 20MB · paths are auto-optimized & ordered for shortest pen travel.
      </div>
    </div>
  );
}

function DrawCanvas() {
  const canvasRef = React.useRef(null);
  const [strokes, setStrokes] = React.useState([]);
  const drawingRef = React.useRef(null);

  const onDown = (e) => {
    const rect = canvasRef.current.getBoundingClientRect();
    drawingRef.current = [[e.clientX - rect.left, e.clientY - rect.top]];
    setStrokes(s => [...s, drawingRef.current]);
    const move = (ev) => {
      if (!drawingRef.current) return;
      const x = ev.clientX - rect.left, y = ev.clientY - rect.top;
      drawingRef.current.push([x, y]);
      setStrokes(s => [...s.slice(0,-1), [...drawingRef.current]]);
    };
    const up = () => {
      drawingRef.current = null;
      window.removeEventListener('pointermove', move);
      window.removeEventListener('pointerup', up);
    };
    window.addEventListener('pointermove', move);
    window.addEventListener('pointerup', up);
  };

  const clear = () => setStrokes([]);
  const undo = () => setStrokes(s => s.slice(0, -1));

  return (
    <div style={{ padding: '0 16px 16px' }}>
      <Card pad={0} style={{ overflow:'hidden' }}>
        <div style={{
          padding: '10px 14px',
          display: 'flex', justifyContent: 'space-between', alignItems: 'center',
          borderBottom: `1px solid ${C.line}`,
        }}>
          <span style={{ fontSize: 11, color: C.text3, textTransform: 'uppercase', letterSpacing: 0.7, fontWeight: 600 }}>
            Draw on canvas — {strokes.length} stroke{strokes.length !== 1 ? 's' : ''}
          </span>
          <div style={{ display:'flex', gap: 6 }}>
            <IconBtn size={28} icon={<I.refresh size={13}/>} onClick={undo} />
            <IconBtn size={28} icon={<I.trash size={13}/>} onClick={clear} />
          </div>
        </div>
        <div
          ref={canvasRef}
          onPointerDown={onDown}
          style={{
            height: 260, background: '#fafafa', position: 'relative', cursor: 'crosshair',
            touchAction: 'none', userSelect: 'none',
          }}>
          <svg width="100%" height="100%" style={{ position:'absolute', inset: 0 }}>
            <defs>
              <pattern id="dot" width="14" height="14" patternUnits="userSpaceOnUse">
                <circle cx="7" cy="7" r="0.8" fill="#0a0d1230" />
              </pattern>
            </defs>
            <rect width="100%" height="100%" fill="url(#dot)" />
            {strokes.map((stroke, i) => (
              <path key={i}
                    d={stroke.map((p, j) => `${j===0?'M':'L'}${p[0]},${p[1]}`).join(' ')}
                    stroke="#0a0d12" strokeWidth="2.5" fill="none" strokeLinecap="round" strokeLinejoin="round" />
            ))}
          </svg>
          {strokes.length === 0 && (
            <div style={{ position:'absolute', inset: 0, display:'flex', alignItems:'center', justifyContent:'center',
                          pointerEvents:'none', color: '#888', fontSize: 13 }}>
              Draw with your finger or Apple Pencil
            </div>
          )}
        </div>
      </Card>
      <div style={{ marginTop: 12, display:'grid', gridTemplateColumns:'1fr 1fr', gap: 10 }}>
        <Btn variant="secondary" full icon={<I.gauge size={16}/>}>Simulate</Btn>
        <Btn variant="primary" full icon={<I.upload size={16}/>}>Save & send</Btn>
      </div>
    </div>
  );
}

function GcodePanel() {
  const SAMPLE = `; mountain ridge — generated by svg2gcode
G21 ; mm
G90 ; absolute
G0 X0 Y0 Z5
M3 ; pen down
G1 X12.4 Y8.6 F2400
G1 X28.1 Y14.2
G1 X42.8 Y19.0
G1 X51.3 Y22.7
G1 X68.9 Y17.4
G1 X84.5 Y12.1
G1 X101.2 Y18.5
M5 ; pen up
G0 X0 Y0
M30`;
  return (
    <div style={{ padding: '0 16px 16px' }}>
      <Card pad={0} style={{ overflow: 'hidden' }}>
        <div style={{ padding:'10px 14px', display:'flex', justifyContent:'space-between', alignItems:'center', borderBottom: `1px solid ${C.line}` }}>
          <span style={{ fontSize: 11, color: C.text3, textTransform:'uppercase', letterSpacing:0.7, fontWeight: 600 }}>
            mountain.gcode · 142 lines · 18.4 m
          </span>
          <div style={{ display:'flex', gap: 6 }}>
            <IconBtn size={28} icon={<I.copy size={13}/>} />
            <IconBtn size={28} icon={<I.download size={13}/>} />
          </div>
        </div>
        <pre style={{
          margin: 0, padding: 14, height: 240, overflow: 'auto', background: '#0a0d12',
          color: C.text2, fontFamily: 'var(--mono)', fontSize: 11, lineHeight: 1.55, fontWeight: 400,
        }}>
{SAMPLE.split('\n').map((line, i) => {
  const cmd = line.split(' ')[0];
  let color = C.text2;
  if (cmd.startsWith('G0')) color = C.violet;
  else if (cmd.startsWith('G1')) color = C.accent;
  else if (cmd.startsWith('M')) color = C.warn;
  else if (line.startsWith(';')) color = C.text3;
  return (
    <div key={i}>
      <span style={{ color: C.text3, marginRight: 14, userSelect: 'none' }}>{String(i+1).padStart(3, ' ')}</span>
      <span style={{ color }}>{line}</span>
    </div>
  );
})}
        </pre>
      </Card>
      <div style={{ marginTop: 12, display:'grid', gridTemplateColumns:'1fr 1fr', gap: 10 }}>
        <Btn variant="secondary" full icon={<I.upload size={16}/>}>Import .gcode</Btn>
        <Btn variant="primary" full icon={<I.play size={16}/>}>Run on rover</Btn>
      </div>
    </div>
  );
}

// Deterministic pseudo-drawing preview from a seed string
function PreviewSvg({ seed = '0', small }) {
  const hash = [...seed].reduce((a, c) => a * 31 + c.charCodeAt(0), 7) >>> 0;
  const rand = (i) => (((hash >> (i * 3)) & 0xff) / 255);
  const paths = Array.from({ length: small ? 4 : 6 }).map((_, i) => {
    const a = rand(i) * 100, b = rand(i+2) * 100, c = rand(i+4) * 100;
    return `M${a*0.4+10},${20+i*8} Q${b*0.6+10},${10+i*9} ${c*0.5+15},${30+i*7}`;
  });
  return (
    <svg viewBox="0 0 100 100" width="100%" height="100%" style={{ padding: small ? 0 : 4 }}>
      {paths.map((d, i) => <path key={i} d={d} stroke="#0a0d12" strokeWidth={small ? 1 : 1.4} fill="none" strokeLinecap="round"/>)}
    </svg>
  );
}

// ─── DXF tab ─────────────────────────────────────────
function DxfPanel() {
  const r = useRover();
  const hasFile = !!r.dxfFile;

  const loadTemplate = (tpl) => {
    const dxf = tpl.build ? tpl.build() : null;
    if (!dxf) return; // (others are visual stubs)
    r.setDxfFile(dxf);
    r.setDxfSelected(new Set(dxf.entities.map(e => e.id)));
    r.setDxfOverrides({});
    r.setDxfOrder(dxf.entities.map(e => e.id));
    r.setDxfInspectorOpen(true);
  };

  if (!hasFile) {
    return (
      <div style={{ padding: '0 16px 16px' }}>
        <Card pad={20} style={{ borderStyle: 'dashed', borderColor: C.line2 }}>
          <div style={{ textAlign:'center' }}>
            <div style={{ width: 56, height: 56, borderRadius: '50%', background: `${C.accent}1a`, color: C.accent,
                          display:'inline-flex', alignItems:'center', justifyContent:'center', marginBottom: 10 }}>
              <I.layers size={26} />
            </div>
            <div style={{ fontSize: 15, fontWeight: 600 }}>Import a DXF</div>
            <div style={{ fontSize: 12, color: C.text3, marginTop: 4 }}>
              Up to 50 MB · LINE · CIRCLE · ARC · POINT · LWPOLYLINE · SPLINE
            </div>
            <div style={{ marginTop: 14, display:'flex', gap: 8, justifyContent:'center', flexWrap:'wrap' }}>
              <Btn variant="primary" size="sm" icon={<I.upload size={14}/>}
                   onClick={() => loadTemplate(DXF_TEMPLATES[0])}>Choose .dxf</Btn>
              <Btn variant="secondary" size="sm" icon={<I.cam size={14}/>}>Scan blueprint</Btn>
            </div>
          </div>
        </Card>

        <SectionHeader title="Templates" sub="Pre-built sports & road marking DXFs"/>
        <div style={{ display:'grid', gridTemplateColumns:'1fr 1fr', gap: 8 }}>
          {DXF_TEMPLATES.map(tpl => (
            <button key={tpl.id} onClick={() => loadTemplate(tpl)} disabled={!tpl.build}
                    style={{
                      padding: 14, background: C.card, border: `1px solid ${C.line}`, borderRadius: 14,
                      cursor: tpl.build ? 'pointer' : 'not-allowed', color: C.text, textAlign: 'left',
                      opacity: tpl.build ? 1 : 0.5, display:'flex', alignItems:'center', gap: 12,
                    }}>
              <span style={{ fontSize: 28 }}>{tpl.icon}</span>
              <span>
                <div style={{ fontSize: 13, fontWeight: 600 }}>{tpl.name}</div>
                <div style={{ fontSize: 11, color: C.text3, fontFamily:'var(--mono)' }}>
                  {tpl.build ? `${tpl.build().entities.length} entities` : `${tpl.count} entities · soon`}
                </div>
              </span>
            </button>
          ))}
        </div>
      </div>
    );
  }

  // Has file — show summary + persistent entity panel
  const total = r.dxfFile.entities.length;
  const sel = r.dxfSelected ? r.dxfSelected.size : 0;
  const byLayer = {};
  r.dxfFile.entities.forEach(e => { byLayer[e.layer] = (byLayer[e.layer] || 0) + 1; });

  return (
    <div style={{ padding: '0 16px 16px' }}>
      {/* File card */}
      <Card pad={14}>
        <div style={{ display:'flex', alignItems:'flex-start', gap: 12 }}>
          <div style={{
            width: 100, height: 100, borderRadius: 12,
            background: '#fafafa', display:'flex', alignItems:'center', justifyContent:'center',
            flexShrink: 0, border: `1px solid ${C.line2}`, overflow:'hidden',
          }}>
            <MiniDXFThumb dxf={r.dxfFile}/>
          </div>
          <div style={{ flex: 1, minWidth: 0 }}>
            <div style={{ fontSize: 14, fontWeight: 600, fontFamily:'var(--mono)' }}>{r.dxfFile.name}</div>
            <div style={{ fontSize: 11, color: C.text3, fontFamily:'var(--mono)', marginTop: 4 }}>
              {total} entities · {Object.keys(byLayer).length} layers · {r.dxfFile.size}
            </div>
            <div style={{ marginTop: 8, display:'flex', gap: 4, flexWrap:'wrap' }}>
              {Object.entries(byLayer).map(([l, c]) => (
                <span key={l} style={{
                  fontSize: 10, padding: '2px 7px', borderRadius: 999, fontFamily:'var(--mono)',
                  background: `${DXF_LAYERS[l]?.color || C.text2}1a`,
                  color: DXF_LAYERS[l]?.color || C.text2,
                  border: `1px solid ${DXF_LAYERS[l]?.color || C.text2}33`,
                }}>{l} · {c}</span>
              ))}
            </div>
          </div>
        </div>
        <div style={{ marginTop: 12, display:'flex', gap: 8 }}>
          <Btn variant="primary" size="sm" icon={<I.sliders size={14}/>} full
               onClick={() => r.setDxfInspectorOpen(true)}>
            Edit selection ({sel}/{total})
          </Btn>
          <Btn variant="secondary" size="sm" icon={<I.trash size={14}/>}
               onClick={() => { r.setDxfFile(null); r.setDxfSelected(null); }}>Remove</Btn>
        </div>
      </Card>

      {/* Quick action — send */}
      <SectionHeader title="Run"/>
      <Card pad={12}>
        <div style={{ fontSize: 12, color: C.text2 }}>
          {sel} entit{sel === 1 ? 'y' : 'ies'} selected · estimated {Math.round(sumLen(r.dxfFile, r.dxfSelected) / 0.4 / 60)} min
        </div>
        <div style={{ marginTop: 10, display:'grid', gridTemplateColumns:'1fr 1fr', gap: 8 }}>
          <Btn variant="secondary" full icon={<I.gauge size={14}/>}>Dry run</Btn>
          <Btn variant="primary" full icon={<I.play size={14}/>}
               onClick={() => {
                 r.setActiveJob({ id: 'dxf', name: r.dxfFile.name, progress: 0, eta: '—', paths: sel, done: 0 });
                 r.setMode('Draw');
                 r.setTab('home');
               }}>Send to rover</Btn>
        </div>
      </Card>
    </div>
  );
}

function sumLen(dxf, selectedSet) {
  if (!dxf || !selectedSet) return 0;
  return dxf.entities.filter(e => selectedSet.has(e.id)).reduce((s, e) => s + (e.length || 0), 0);
}

function MiniDXFThumb({ dxf }) {
  const w = dxf.bounds.w, h = dxf.bounds.h;
  const tintFill = { pitch:'rgba(34,197,94,0.12)', court:'rgba(56,189,248,0.10)', road:'rgba(80,80,90,0.10)' }[dxf.tint];
  return (
    <svg viewBox={`-20 -20 ${w+40} ${h+40}`} width="100%" height="100%" preserveAspectRatio="xMidYMid meet">
      {tintFill && <rect x="0" y="0" width={w} height={h} fill={tintFill}/>}
      {dxf.entities.map(e => entityToSvg(e, e.id, { stroke: '#0a0d12', strokeWidth: 4 }))}
    </svg>
  );
}

window.DxfPanel = DxfPanel;
window.DrawScreen = DrawScreen;
window.PreviewSvg = PreviewSvg;
