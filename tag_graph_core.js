/* Core logic of the AprilTag Graph Maker (no GUI, works in the browser as window.TagGraph and in node as module.exports).
 *
 * Idea: the graph is described by LANE LINES - vertical lines x = c over [y0, y1] and horizontal lines y = c over [x0, x1], all in
 * the MAP frame. The nodes (= AprilTags) and edges (= lanes) are DERIVED from them:
 *   - a node at every end of a line, at every crossing of two lines and every <= maxEdge metres along a line;
 *   - an edge between two consecutive nodes of a line.
 * So every lane is axis-aligned (never diagonal), parallel lanes are just parallel lines, and two lanes only ever meet in a node
 * (a crossing is always a node: no cross-cross edges, no overlapping lanes because collinear overlapping lines are merged).
 * The occupancy map decides what is valid (differential-drive robot: it stops at every node and spins on the spot):
 *   node valid  <=> clearance to walls/unknown >= nodeClear = circumscribed radius of the robot + margin  (room for the spot turn)
 *   lane valid  <=> both nodes valid and clearance along it >= laneClear = half the robot width + margin
 */
(function (root) {
  'use strict';
  const TOL = 1e-6;
  const key = (x, y) => Math.round(x * 1000) + ',' + Math.round(y * 1000);

  /* ---------------- robot ---------------- */
  function robotParams(p) {
    const L = p.length, W = p.width, m = p.margin ?? 0.15;
    const rCirc = Math.hypot(L / 2, W / 2);
    return { length: L, width: W, margin: m, rCirc, nodeClear: rCirc + m, laneClear: W / 2 + m };
  }

  /* ---------------- clearance field from the map image ---------------- */
  /** Felzenszwalb & Huttenlocher 1-D squared distance transform. */
  function dt1d(f, n, d, v, z) {
    let k = 0; v[0] = 0; z[0] = -Infinity; z[1] = Infinity;
    for (let q = 1; q < n; q++) {
      let s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
      while (s <= z[k]) { k--; s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]); }
      k++; v[k] = q; z[k] = s; z[k + 1] = Infinity;
    }
    k = 0;
    for (let q = 0; q < n; q++) { while (z[k + 1] < q) k++; d[q] = (q - v[k]) * (q - v[k]) + f[v[k]]; }
  }
  /** Euclidean distance (cells) from every cell to the nearest blocked cell (blocked = 1). */
  function edt(blocked, w, h) {
    const INF = 1e12, g = new Float64Array(w * h);
    for (let i = 0; i < w * h; i++) g[i] = blocked[i] ? 0 : INF;
    const n = Math.max(w, h), f = new Float64Array(n), d = new Float64Array(n), v = new Int32Array(n), z = new Float64Array(n + 1);
    for (let x = 0; x < w; x++) { for (let y = 0; y < h; y++) f[y] = g[y * w + x]; dt1d(f, h, d, v, z); for (let y = 0; y < h; y++) g[y * w + x] = d[y]; }
    for (let y = 0; y < h; y++) { for (let x = 0; x < w; x++) f[x] = g[y * w + x]; dt1d(f, w, d, v, z); for (let x = 0; x < w; x++) g[y * w + x] = d[x]; }
    const out = new Float32Array(w * h);
    for (let i = 0; i < w * h; i++) out[i] = Math.sqrt(g[i]);
    return out;
  }
  /**
   * map = {w, h, res, ox, oy, gray: Uint8Array (row 0 = top, as in the .pgm)}; freeGray: pixels at least this bright are free,
   * everything darker (walls AND grey "unknown") blocks. Returns the field {w,h,res,ox,oy,dt (metres)}.
   */
  function buildField(map, freeGray) {
    const n = map.w * map.h, blocked = new Uint8Array(n);
    const fg = freeGray ?? 240, negate = !!(map.meta && map.meta.negate);
    for (let i = 0; i < n; i++) { const g = negate ? 255 - map.gray[i] : map.gray[i]; blocked[i] = g >= fg ? 0 : 1; }
    const d = edt(blocked, map.w, map.h);
    for (let i = 0; i < n; i++) d[i] *= map.res;
    return { w: map.w, h: map.h, res: map.res, ox: map.ox, oy: map.oy, dt: d };
  }
  /** clearance (m) at a map-frame point; 0 outside the map. */
  function clearAt(f, x, y) {
    if (!f) return Infinity;                                              // no map loaded: everything is valid
    const c = Math.floor((x - f.ox) / f.res), r = f.h - 1 - Math.floor((y - f.oy) / f.res);
    return (c < 0 || r < 0 || c >= f.w || r >= f.h) ? 0 : f.dt[r * f.w + c];
  }
  function segmentClear(f, a, b, minClear) {
    const n = Math.max(2, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / 0.05));
    let worst = Infinity;
    for (let i = 0; i <= n; i++) {
      const c = clearAt(f, a.x + (b.x - a.x) * i / n, a.y + (b.y - a.y) * i / n);
      if (c < worst) worst = c;
      if (worst < minClear) return { ok: false, worst };
    }
    return { ok: true, worst };
  }

  /* ---------------- automatic lines (aisle centre-lines + lattice) ---------------- */
  /** centres of corridors: along `axis` ('x' = vertical corridors, scan rows). returns positions in metres. */
  function aisleLines(f, axis, edgeClear, minLen, opts) {
    const o = Object.assign({ maxW: 5.0, minW: 0.8, step: 2 }, opts || {});
    const n = axis === 'x' ? f.h : f.w, len = axis === 'x' ? f.w : f.h, votes = new Map();
    for (let i = 0; i < n; i += o.step) {
      let s = -1;
      for (let j = 0; j <= len; j++) {
        const cap = j < len && (axis === 'x' ? f.dt[i * f.w + j] : f.dt[j * f.w + i]) >= edgeClear;
        if (cap && s < 0) s = j;
        if (!cap && s >= 0) {
          const w = (j - s) * f.res;
          if (w >= o.minW && w <= o.maxW) { const k = Math.round((s + j - 1) / 2 / 2.0); votes.set(k, (votes.get(k) || 0) + 1); }
          s = -1;
        }
      }
    }
    const score = new Map();
    for (const k of votes.keys()) score.set(k, (votes.get(k - 1) || 0) + votes.get(k) + (votes.get(k + 1) || 0));
    const out = [];
    for (const [k, sc] of [...score.entries()].sort((a, b) => b[1] - a[1])) {
      if (sc * o.step * f.res < minLen) break;
      const pos = k * 2 + 0.5;                                              // cell index along the scanned direction
      out.push(axis === 'x' ? f.ox + (pos + 0.5) * f.res : f.oy + (f.h - 1 - pos + 0.5) * f.res);
    }
    return out;
  }
  function mergeLines(aisle, lo, hi, spacing, minSep) {
    const kept = [];
    for (const p of [...aisle].sort((a, b) => a - b)) if (kept.every(q => Math.abs(p - q) >= minSep)) kept.push(p);
    const n = Math.floor((hi - lo) / spacing + 1e-9) + 1, start = lo + ((hi - lo) - (n - 1) * spacing) / 2;
    for (let i = 0; i < n; i++) { const v = start + i * spacing; if (kept.every(q => Math.abs(v - q) >= minSep)) kept.push(v); }
    return kept.sort((a, b) => a - b);
  }
  /** Full automatic layout inside bounds {x0,x1,y0,y1}. Returns lines (full width/height; invalid parts are dropped by derive()). */
  function autoLines(f, bounds, rp, opts) {
    const o = Object.assign({ spacing: 2.0, minSep: 1.0, minAisle: 2.5 }, opts || {});
    const ax = aisleLines(f, 'x', rp.laneClear, o.minAisle).filter(x => x >= bounds.x0 && x <= bounds.x1);
    const ay = aisleLines(f, 'y', rp.laneClear, o.minAisle).filter(y => y >= bounds.y0 && y <= bounds.y1);
    const X = mergeLines(ax, bounds.x0, bounds.x1, o.spacing, o.minSep), Y = mergeLines(ay, bounds.y0, bounds.y1, o.spacing, o.minSep);
    return { v: X.map(x => ({ x, y0: bounds.y0, y1: bounds.y1 })), h: Y.map(y => ({ y, x0: bounds.x0, x1: bounds.x1 })),
             info: { aisleX: ax.length, aisleY: ay.length } };
  }

  /* ---------------- lines -> nodes + edges ---------------- */
  /** Merge collinear overlapping/touching lines (no overlapping lanes), normalise ranges. */
  function normalizeLines(lines, snap) {
    snap = snap ?? 0.02;
    const out = { v: [], h: [] };
    const merge = (arr, c, a, b) => {
      const g = [];
      for (const l of [...arr].map(l => ({ c: l[c], a: Math.min(l[a], l[b]), b: Math.max(l[a], l[b]) })).sort((p, q) => p.c - q.c || p.a - q.a)) {
        const last = g[g.length - 1];
        if (last && Math.abs(last.c - l.c) <= snap && l.a <= last.b + snap) { last.b = Math.max(last.b, l.b); last.n++; last.cs += l.c; }
        else g.push({ c: l.c, a: l.a, b: l.b, n: 1, cs: l.c });
      }
      return g.map(l => ({ c: l.cs / l.n, a: l.a, b: l.b }));
    };
    merge(lines.v || [], 'x', 'y0', 'y1').forEach(l => out.v.push({ x: l.c, y0: l.a, y1: l.b }));
    merge(lines.h || [], 'y', 'x0', 'x1').forEach(l => out.h.push({ y: l.c, x0: l.a, x1: l.b }));
    return out;
  }

  /**
   * Derive the graph. opts: {maxEdge (m, default 2.0), keepLargestOnly (default true)}.
   * Returns {nodes:[{id,x,y}], edges:[[i,j]], dropped:{nodes:[{x,y,clear}], edges:[{a,b,worst}]}, isolated, lines}.
   */
  function derive(lines, field, rp, opts) {
    const o = Object.assign({ maxEdge: 2.0, keepLargestOnly: true }, opts || {});
    const L = normalizeLines(lines);
    const cand = new Map(), list = [];
    const node = (x, y) => { const k = key(x, y); if (!cand.has(k)) { cand.set(k, list.length); list.push({ x, y }); } return cand.get(k); };
    const cross = (l, vertical) => {                                      // coordinates along line l where perpendicular lines cross it
      const pts = [vertical ? l.y0 : l.x0, vertical ? l.y1 : l.x1];
      const others = vertical ? L.h : L.v;
      for (const q of others) {
        const c = vertical ? l.x : l.y;
        const qa = vertical ? q.x0 : q.y0, qb = vertical ? q.x1 : q.y1, qc = vertical ? q.y : q.x;
        const la = vertical ? l.y0 : l.x0, lb = vertical ? l.y1 : l.x1;
        if (c >= qa - TOL && c <= qb + TOL && qc >= la - TOL && qc <= lb + TOL) pts.push(qc);
      }
      return [...new Set(pts.map(p => Math.round(p * 1000) / 1000))].sort((a, b) => a - b);
    };
    const chain = (l, vertical) => {                                      // ordered node indices along one line, with subdivision
      let pts = cross(l, vertical), withSub = [pts[0]];
      for (let i = 1; i < pts.length; i++) {
        const gap = pts[i] - pts[i - 1], n = Math.ceil(gap / o.maxEdge - 1e-9);
        for (let s = 1; s < n; s++) withSub.push(pts[i - 1] + gap * s / n);
        withSub.push(pts[i]);
      }
      return withSub.map(p => vertical ? node(l.x, p) : node(p, l.y));
    };
    const edgeSet = new Map();
    const addChain = (ids) => { for (let i = 0; i + 1 < ids.length; i++) { const a = ids[i], b = ids[i + 1]; if (a !== b) edgeSet.set(Math.min(a, b) + '>' + Math.max(a, b), [Math.min(a, b), Math.max(a, b)]); } };
    L.v.forEach(l => addChain(chain(l, true)));
    L.h.forEach(l => addChain(chain(l, false)));

    // validity from the map
    const clear = list.map(p => clearAt(field, p.x, p.y)), valid = clear.map(c => c >= rp.nodeClear);
    const droppedEdges = [], keptEdges = [];
    for (const [a, b] of edgeSet.values()) {
      if (!valid[a] || !valid[b]) continue;
      const r = segmentClear(field, list[a], list[b], rp.laneClear);
      (r.ok ? keptEdges : droppedEdges).push(r.ok ? [a, b] : { a, b, worst: r.worst });
    }
    // connected components over valid nodes
    const adj = list.map(() => []);
    keptEdges.forEach(([a, b]) => { adj[a].push(b); adj[b].push(a); });
    const comp = list.map(() => -1); let nc = 0; const sizes = [];
    for (let i = 0; i < list.length; i++) {
      if (!valid[i] || comp[i] >= 0) continue;
      let size = 0; const st = [i]; comp[i] = nc;
      while (st.length) { const u = st.pop(); size++; for (const v of adj[u]) if (comp[v] < 0) { comp[v] = nc; st.push(v); } }
      sizes.push(size); nc++;
    }
    const best = sizes.indexOf(Math.max(...sizes, 0));
    const useNode = list.map((_, i) => valid[i] && (!o.keepLargestOnly || comp[i] === best));
    const remap = new Map(), nodes = [];
    list.forEach((p, i) => { if (useNode[i] && adj[i].length > 0) { remap.set(i, nodes.length); nodes.push({ id: -1, x: p.x, y: p.y }); } });
    const edges = keptEdges.filter(([a, b]) => remap.has(a) && remap.has(b)).map(([a, b]) => [remap.get(a), remap.get(b)]);
    return {
      nodes, edges, lines: L,
      dropped: { nodes: list.map((p, i) => ({ x: p.x, y: p.y, clear: clear[i], valid: valid[i] })).filter(p => !p.valid),
                 edges: droppedEdges.map(e => ({ a: list[e.a], b: list[e.b], worst: e.worst })) },
      isolated: o.keepLargestOnly ? list.filter((_, i) => valid[i] && comp[i] !== best && comp[i] >= 0).length : 0,
    };
  }

  /* ---------------- ids ---------------- */
  /** ids row by row (south -> north, west -> east) from startId; startNodeIndex (optional) gets id `startTag` (swap). */
  function assignIds(graph, opts) {
    const o = Object.assign({ startId: 0, startNodeIndex: null, startTag: 1 }, opts || {});
    const order = graph.nodes.map((_, i) => i).sort((a, b) => graph.nodes[a].y - graph.nodes[b].y || graph.nodes[a].x - graph.nodes[b].x);
    order.forEach((i, k) => { graph.nodes[i].id = o.startId + k; });
    if (o.startNodeIndex !== null && o.startNodeIndex >= 0 && graph.nodes[o.startNodeIndex]) {
      const a = graph.nodes[o.startNodeIndex], b = graph.nodes.find(n => n.id === o.startTag);
      if (b && a !== b) { const t = a.id; a.id = b.id; b.id = t; }
    }
    return graph;
  }

  /* ---------------- frames (map frame <-> simulator world frame) ---------------- */
  /** frame = {dx, dy, yawDeg}: world = R(yaw) * map + (dx, dy). */
  function toWorld(fr, x, y) { const c = Math.cos(fr.yawDeg * Math.PI / 180), s = Math.sin(fr.yawDeg * Math.PI / 180); return { x: c * x - s * y + fr.dx, y: s * x + c * y + fr.dy }; }
  function toMap(fr, x, y) { const c = Math.cos(fr.yawDeg * Math.PI / 180), s = Math.sin(fr.yawDeg * Math.PI / 180), px = x - fr.dx, py = y - fr.dy; return { x: c * px + s * py, y: -s * px + c * py }; }
  /** From reference points seen in both frames: pairs = [{map:{x,y}, world:{x,y}}, ...] (1 pair: translation only, keeps yawDeg0). */
  function solveFrame(pairs, yawDeg0) {
    if (!pairs.length) return null;
    let yaw = (yawDeg0 || 0), scale = 1;
    if (pairs.length >= 2) {
      const [a, b] = pairs, mv = { x: b.map.x - a.map.x, y: b.map.y - a.map.y }, wv = { x: b.world.x - a.world.x, y: b.world.y - a.world.y };
      yaw = (Math.atan2(wv.y, wv.x) - Math.atan2(mv.y, mv.x)) * 180 / Math.PI;
      yaw = ((yaw + 540) % 360) - 180;
      scale = Math.hypot(wv.x, wv.y) / Math.hypot(mv.x, mv.y);
    }
    const c = Math.cos(yaw * Math.PI / 180), s = Math.sin(yaw * Math.PI / 180);
    let dx = 0, dy = 0;
    pairs.forEach(p => { dx += p.world.x - (c * p.map.x - s * p.map.y); dy += p.world.y - (s * p.map.x + c * p.map.y); });
    dx /= pairs.length; dy /= pairs.length;
    const err = pairs.map(p => { const w = toWorld({ dx, dy, yawDeg: yaw }, p.map.x, p.map.y); return Math.hypot(w.x - p.world.x, w.y - p.world.y); });
    return { dx, dy, yawDeg: yaw, scale, maxErr: Math.max(...err) };
  }

  /* ---------------- checks ---------------- */
  function segCross(p, q, r, s) {                                         // proper crossing of two segments
    const d = (a, b, c) => (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    const d1 = d(r, s, p), d2 = d(r, s, q), d3 = d(p, q, r), d4 = d(p, q, s);
    return d1 * d2 < -1e-9 && d3 * d4 < -1e-9;
  }
  function check(graph, minSep) {
    const N = graph.nodes, E = graph.edges, res = { edges: E.length, nodes: N.length, diagonal: 0, crossings: 0, nodesOnLane: 0, tooClose: [] };
    for (const [a, b] of E) if (Math.abs(N[a].x - N[b].x) > 1e-6 && Math.abs(N[a].y - N[b].y) > 1e-6) res.diagonal++;
    for (let i = 0; i < E.length; i++) for (let j = i + 1; j < E.length; j++) {
      const [a, b] = E[i], [c, d] = E[j];
      if (a === c || a === d || b === c || b === d) continue;
      if (segCross(N[a], N[b], N[c], N[d])) res.crossings++;
    }
    for (const [a, b] of E) for (let n = 0; n < N.length; n++) {
      if (n === a || n === b) continue;
      const p = N[a], q = N[b], r = N[n], cr = (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
      if (Math.abs(cr) < 1e-6 && r.x >= Math.min(p.x, q.x) - 1e-6 && r.x <= Math.max(p.x, q.x) + 1e-6 && r.y >= Math.min(p.y, q.y) - 1e-6 && r.y <= Math.max(p.y, q.y) + 1e-6) res.nodesOnLane++;
    }
    const ms = minSep ?? 0.9;                                             // parallel lines closer than minSep
    const near = (vals, axis) => { const s = [...new Set(vals.map(v => Math.round(v * 100) / 100))].sort((a, b) => a - b); for (let i = 1; i < s.length; i++) if (s[i] - s[i - 1] < ms) res.tooClose.push(axis + '=' + s[i - 1].toFixed(2) + ' / ' + s[i].toFixed(2)); };
    if (graph.lines) { near(graph.lines.v.map(l => l.x), 'x'); near(graph.lines.h.map(l => l.y), 'y'); }
    return res;
  }

  /* ---------------- exports ---------------- */
  const f3 = v => (Math.round(v * 1000) / 1000);
  /** tag map in the format the navigation stack loads. frame null -> map frame, else world frame. */
  function tagMapYaml(graph, o) {
    const fr = o.frame, tag = o.tagSize ?? 0.2;
    const yaw = fr ? fr.yawDeg * Math.PI / 180 : 0;
    const nodes = [...graph.nodes].sort((a, b) => a.id - b.id);
    let y = `# AprilTag graph: node = tag id -> (x, y) in the ${fr ? 'WORLD (simulator)' : 'MAP'} frame, edge = lane (axis-aligned in the map frame).\n` +
            `# Generated by the AprilTag Graph Maker ${o.date || ''}\n` + (o.header ? o.header.split('\n').map(l => '# ' + l).join('\n') + '\n' : '') +
            `family: tag36h11\ntag_size: ${tag}\nframe: ${fr ? 'map' : 'map'}\n` +
            (fr ? `source_frame_transform: {dx: ${f3(fr.dx)}, dy: ${f3(fr.dy)}, yaw_deg: ${f3(fr.yawDeg)}}   # world = R(yaw) * map + (dx, dy)\n` : '') +
            `nodes:\n`;
    for (const n of nodes) { const p = fr ? toWorld(fr, n.x, n.y) : n; y += `  ${n.id}: {x: ${f3(p.x)}, y: ${f3(p.y)}, yaw: ${f3(yaw)}}\n`; }
    y += 'edges:\n';
    for (const [a, b] of graph.edges) y += `- {from: ${graph.nodes[a].id}, to: ${graph.nodes[b].id}, is_bidirectional: true}\n`;
    return y;
  }
  function coordsCsv(graph, fr) {
    let s = 'tag_id,x_map_m,y_map_m,x_world_m,y_world_m,tag_yaw_world_rad\n';
    for (const n of [...graph.nodes].sort((a, b) => a.id - b.id)) {
      const w = fr ? toWorld(fr, n.x, n.y) : null;
      s += `${n.id},${f3(n.x)},${f3(n.y)},${w ? f3(w.x) : ''},${w ? f3(w.y) : ''},${fr ? f3(fr.yawDeg * Math.PI / 180) : ''}\n`;
    }
    return s;
  }
  /** SDF snippet: one flat tag model per node in the WORLD frame; textures: model://agv_tags/materials/textures/tag36_11_NNNNN.png */
  function sdfBlock(graph, fr, tagZ) {
    const z = tagZ ?? 0.011, yaw = fr ? fr.yawDeg * Math.PI / 180 : 0;
    let s = `    <!-- ===== ${graph.nodes.length} AprilTag floor markers (AprilTag Graph Maker) ===== -->\n`;
    for (const n of [...graph.nodes].sort((a, b) => a.id - b.id)) {
      const p = fr ? toWorld(fr, n.x, n.y) : n, id = String(n.id).padStart(5, '0');
      s += `    <model name="tag_${n.id}"><static>true</static><pose>${p.x.toFixed(3)} ${p.y.toFixed(3)} ${z} 0 0 ${yaw.toFixed(5)}</pose><link name="link">` +
           `<visual name="v"><pose>0 0 0.0015 0 0 3.14159265</pose><cast_shadows>false</cast_shadows><geometry><plane><normal>0 0 1</normal><size>0.25 0.25</size></plane></geometry>` +
           `<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular><pbr><metal>` +
           `<albedo_map>model://agv_tags/materials/textures/tag36_11_${id}.png</albedo_map><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr></material></visual></link></model>\n`;
    }
    return s;
  }
  /** everything the Python exporter needs. */
  function exportJson(graph, o) {
    const fr = o.frame || { dx: 0, dy: 0, yawDeg: 0 };
    return {
      format: 'apriltag-graph/1', created: o.date || '', name: o.name || 'layout', tag_size: o.tagSize ?? 0.2, map: o.mapName || '',
      map_origin: o.mapOrigin || null, frame_map_to_world: { dx: fr.dx, dy: fr.dy, yaw_deg: fr.yawDeg }, frame_is_set: !!o.frame,
      robot: o.robot || null, start_tag: o.startTag ?? 1,
      nodes: [...graph.nodes].sort((a, b) => a.id - b.id).map(n => { const w = toWorld(fr, n.x, n.y); return { id: n.id, map: { x: f3(n.x), y: f3(n.y) }, world: { x: f3(w.x), y: f3(w.y) }, yaw_world: f3(fr.yawDeg * Math.PI / 180) }; }),
      edges: graph.edges.map(([a, b]) => ({ from: graph.nodes[a].id, to: graph.nodes[b].id })),
    };
  }

  const api = { robotParams, buildField, clearAt, segmentClear, edt, aisleLines, mergeLines, autoLines, normalizeLines, derive, assignIds,
                toWorld, toMap, solveFrame, check, tagMapYaml, coordsCsv, sdfBlock, exportJson };
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else root.TagGraph = api;
})(typeof window !== 'undefined' ? window : globalThis);
