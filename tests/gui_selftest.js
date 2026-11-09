// Runs inside tag_graph_maker.html when opened with ?selftest=1&map=testmap/warehouse_slam.yaml&auto=1  (see tests/run_gui_selftest.sh)
(async () => {
  const T = window.__tg, S = T.S, out = []; const ok = (c, m) => out.push((c ? 'PASS ' : 'FAIL ') + m);
  while (!S.graph || !S.graph.nodes.length) await new Promise(r => setTimeout(r, 100));
  const n0 = S.graph.nodes.length, e0 = S.graph.edges.length, v0 = S.lines.v.length;
  ok(S.chk.diagonal === 0 && S.chk.crossings === 0, `auto layout: ${n0} tags, ${e0} lanes, 0 diagonal, 0 crossings`);
  // 1. a drawn DIAGONAL drag must become an axis-aligned lane
  T.addLine({ x: -10, y: 5 }, { x: -6, y: 8 }); T.doRefresh();
  ok(S.chk.diagonal === 0, 'a diagonal drag produced no diagonal lane (' + (S.lines.h.length + S.lines.v.length) + ' lines)');
  // 2. a long extra vertical lane in an open area adds tags and still crosses cleanly
  T.addLine({ x: -3.77, y: 0 }, { x: -3.77, y: 12 }); T.doRefresh();
  ok(S.graph.nodes.length >= n0 && S.chk.crossings === 0 && S.chk.nodesOnLane === 0, 'drawn lane: ' + S.graph.nodes.length + ' tags, no crossings / no tag on a foreign lane');
  // 3. erase an area, then undo restores the counts
  const before = S.graph.edges.length; document.getElementById('undoBtn').click(); document.getElementById('undoBtn').click(); T.doRefresh();
  ok(S.graph.edges.length === e0 && S.lines.v.length === v0, 'undo x2 restores the auto layout (' + S.graph.edges.length + ' lanes)');
  document.getElementById('redoBtn').click(); T.doRefresh();
  // 4. the real erase function on a rectangle in the middle of the map, then undo
  const m = S.map, cx = m.ox + m.w * m.res / 2, cy = m.oy + m.h * m.res / 2, eB = S.graph.edges.length;
  T.eraseRect({ a: { x: cx - 4, y: cy - 4 }, b: { x: cx + 4, y: cy + 4 } }); T.doRefresh();
  ok(S.graph.edges.length < eB && S.chk.diagonal === 0 && S.chk.crossings === 0, `erase area: ${eB} -> ${S.graph.edges.length} lanes, still clean`);
  document.getElementById('undoBtn').click(); T.doRefresh();
  ok(S.graph.edges.length === eB, 'undo after erase restores ' + eB + ' lanes');
  // 5. frame calibration from two reference points
  const fr = { dx: 13.55, dy: -10, yawDeg: 3 };
  S.refs = [{ map: { x: -20, y: 10 }, world: T.TG.toWorld(fr, -20, 10) }, { map: { x: -10, y: 25 }, world: T.TG.toWorld(fr, -10, 25) }];
  document.getElementById('solveBtn').click();
  ok(Math.abs(parseFloat(document.getElementById('fdx').value) - 13.55) < 0.02 && Math.abs(parseFloat(document.getElementById('fyaw').value) - 3) < 0.05, 'reference points solve the simulator frame (dx ' + document.getElementById('fdx').value + ', yaw ' + document.getElementById('fyaw').value + ')');
  // 6. exports contain every tag
  const g = S.graph, y = T.TG.tagMapYaml(g, { frame: T.S.frame }), j = T.TG.exportJson(g, { frame: T.S.frame, name: 't' });
  ok((y.match(/^  \d+: \{x:/gm) || []).length === g.nodes.length && j.edges.length === g.edges.length, 'yaml + json export hold all ' + g.nodes.length + ' tags and ' + g.edges.length + ' lanes');
  const el = document.createElement('pre'); el.id = 'selftest'; el.textContent = out.join('\n') + '\nDONE'; document.body.appendChild(el);
})();
