// components/dxf/DrawCanvas.tsx
import React, { useState, useCallback, useRef } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { Gesture, GestureDetector } from 'react-native-gesture-handler';
import Svg, { Path, Rect, Defs, Pattern, Circle } from 'react-native-svg';
import { C } from '../../theme/colors';
import { Card } from '../ui/Card';
import { IconBtn } from '../ui/IconBtn';
import { Btn } from '../ui/Btn';
import { Icons } from '../icons';

type Stroke = [number, number][];

/** #12 — minimum ms between React state updates during drawing (~30 Hz) */
const UPDATE_INTERVAL_MS = 33;

export function DrawCanvas() {
  // Committed strokes (only updated on onEnd or throttled)
  const [strokes, setStrokes] = useState<Stroke[]>([]);

  // #12 — live stroke lives in a ref; SVG re-render is driven by a counter
  const liveStrokeRef = useRef<Stroke>([]);
  const [renderTick, setRenderTick] = useState(0);
  const lastUpdateMs = useRef(0);
  const isDrawing = useRef(false);

  const forceRender = useCallback(() => setRenderTick((t) => t + 1), []);

  const gesture = Gesture.Pan()
    .runOnJS(true)
    .onStart((e) => {
      isDrawing.current = true;
      liveStrokeRef.current = [[e.x, e.y]];
      lastUpdateMs.current = Date.now();
      forceRender();
    })
    .onUpdate((e) => {
      liveStrokeRef.current = [...liveStrokeRef.current, [e.x, e.y]];

      // #12 — throttle state updates; only re-render at ~30 Hz
      const now = Date.now();
      if (now - lastUpdateMs.current >= UPDATE_INTERVAL_MS) {
        lastUpdateMs.current = now;
        forceRender();
      }
    })
    .onEnd(() => {
      isDrawing.current = false;
      // #12 — commit completed stroke to state on onEnd, then clear ref
      const finished = [...liveStrokeRef.current];
      liveStrokeRef.current = [];
      if (finished.length > 1) {
        setStrokes((prev) => [...prev, finished]);
      }
      forceRender();
    });

  const undo = useCallback(() => setStrokes((s) => s.slice(0, -1)), []);
  const clear = useCallback(() => {
    setStrokes([]);
    liveStrokeRef.current = [];
    forceRender();
  }, [forceRender]);

  const toPath = (stroke: Stroke) =>
    stroke.map(([x, y], i) => `${i === 0 ? 'M' : 'L'}${x},${y}`).join(' ');

  const liveStroke = liveStrokeRef.current;

  return (
    <View style={styles.outer}>
      <Card pad={0} style={styles.card}>
        {/* Toolbar */}
        <View style={styles.toolbar}>
          <Text style={styles.toolbarLabel}>
            {strokes.length} stroke{strokes.length !== 1 ? 's' : ''}
          </Text>
          <View style={styles.toolbarBtns}>
            <IconBtn size={28} icon={<Icons.chevL size={13} color={C.text2} />} onPress={undo} />
            <IconBtn size={28} icon={<Icons.trash size={13} color={C.text2} />} onPress={clear} />
          </View>
        </View>

        {/* Canvas */}
        <GestureDetector gesture={gesture}>
          <View style={styles.canvas}>
            <Svg width="100%" height="100%" style={StyleSheet.absoluteFill}>
              <Defs>
                <Pattern id="dots" width={14} height={14} patternUnits="userSpaceOnUse">
                  <Circle cx={7} cy={7} r={0.8} fill="rgba(10,13,18,0.3)" />
                </Pattern>
              </Defs>
              <Rect width="100%" height="100%" fill="url(#dots)" />

              {/* Committed strokes */}
              {strokes.map((stroke, i) => (
                <Path
                  key={i}
                  d={toPath(stroke)}
                  stroke="#0a0d12"
                  strokeWidth={2.5}
                  fill="none"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
              ))}

              {/* #12 — live in-progress stroke from ref (no extra state write per point) */}
              {liveStroke.length > 1 && (
                <Path
                  d={toPath(liveStroke)}
                  stroke="#0a0d12"
                  strokeWidth={2.5}
                  fill="none"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
              )}
            </Svg>
            {strokes.length === 0 && !isDrawing.current && (
              <Text style={styles.emptyText}>Draw with your finger</Text>
            )}
          </View>
        </GestureDetector>
      </Card>

      <View style={styles.actions}>
        <Btn variant="secondary" style={styles.actionBtn} icon={<Icons.target size={15} color={C.text2} />}>
          Simulate
        </Btn>
        <Btn variant="primary" style={styles.actionBtn} icon={<Icons.upload size={15} color="#06202a" />}>
          Save &amp; send
        </Btn>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  outer: { paddingHorizontal: 16 },
  card: { overflow: 'hidden', marginBottom: 12 },
  toolbar: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: 10,
    paddingHorizontal: 14,
    borderBottomWidth: 1,
    borderBottomColor: C.line,
  },
  toolbarLabel: {
    fontSize: 11,
    color: C.text3,
    textTransform: 'uppercase',
    letterSpacing: 0.7,
    fontWeight: '600',
  },
  toolbarBtns: { flexDirection: 'row', gap: 6 },
  canvas: {
    height: 260,
    backgroundColor: '#fafafa',
    alignItems: 'center',
    justifyContent: 'center',
  },
  emptyText: {
    fontSize: 13,
    color: '#888',
    position: 'absolute',
  },
  actions: { flexDirection: 'row', gap: 10 },
  actionBtn: { flex: 1, alignSelf: 'auto', justifyContent: 'center', alignItems: 'center' },
});
